#include "wifilink.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/message_buffer.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

#define OUT_BUFFER_BYTES 8192   // about 3 s of rows queued for the laptop
#define MSG_MAX 320             // longest line sent, including its '\n'
#define RX_LINE_MAX 96          // longest command accepted
#define CMD_QUEUE_LEN 8
#define TASK_STACK_BYTES 4096
#define TASK_PRIORITY 1         // same as the logger loop, so it never preempts it
#define SEND_TIMEOUT_MS 1000    // a laptop that stops taking data is dropped after this

static const char *TAG = "wifilink";
static MessageBufferHandle_t outBuf = NULL;
static QueueHandle_t cmdQueue = NULL;
static volatile bool clientConnected = false;

static void wifiEvent(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    (void)data;
    if (id == WIFI_EVENT_AP_STACONNECTED)
        ESP_LOGI(TAG, "A device joined the WiFi network");
    else if (id == WIFI_EVENT_AP_STADISCONNECTED)
        ESP_LOGI(TAG, "A device left the WiFi network");
}

static bool sendAll(int sock, const char *data, size_t len)
{
    while (len > 0) {
        int n = send(sock, data, len, 0);
        if (n <= 0) return false;
        data += n;
        len -= (size_t)n;
    }
    return true;
}

static void closeClient(int *client, const char *why)
{
    if (*client < 0) return;
    clientConnected = false;
    close(*client);
    *client = -1;
    ESP_LOGI(TAG, "Laptop disconnected (%s)", why);
}

static void configureClient(int sock)
{
    // Blocking sends with a timeout; receives poll with MSG_DONTWAIT.
    fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) & ~O_NONBLOCK);
    struct timeval tv = {
        .tv_sec = SEND_TIMEOUT_MS / 1000,
        .tv_usec = (SEND_TIMEOUT_MS % 1000) * 1000,
    };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    int one = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
}

// Returns a reply for the laptop, or NULL when the command went to the logger loop
// (which replies itself). Sets *wifiOff for WIFI,OFF.
static const char *handleCommandLine(const char *line, bool *wifiOff)
{
    wifi_cmd_t cmd = {0};
    if (strcmp(line, "LOG") == 0) {
        cmd.type = WIFI_CMD_LOG;
    } else if (strcmp(line, "CAL,LEVEL") == 0) {
        cmd.type = WIFI_CMD_CAL_LEVEL;
    } else if (strcmp(line, "CAL,PITCH") == 0) {
        cmd.type = WIFI_CMD_CAL_PITCH;
    } else if (strncmp(line, "CAL,SAVE,", 9) == 0) {
        cmd.type = WIFI_CMD_CAL_SAVE;
        const char *p = line + 9;
        for (int i = 0; i < 3; i++) {
            char *end;
            cmd.value[i] = strtof(p, &end);
            if (end == p || !isfinite(cmd.value[i]) || *end != (i < 2 ? ',' : '\0'))
                return "#ERR,CAL,bad offset (expected CAL,SAVE,x,y,z in cm)";
            p = end + 1;
        }
    } else if (strcmp(line, "WIFI,OFF") == 0) {
        *wifiOff = true;
        return "#WIFI,OFF";
    } else {
        return "#ERR,unknown command";
    }
    if (xQueueSend(cmdQueue, &cmd, 0) != pdTRUE) return "#ERR,busy, try again";
    return NULL;
}

static void stopWifi(int *client, int listenSock)
{
    clientConnected = false;
    closeClient(client, "WiFi turned off from the laptop");   // sends the queued reply first
    close(listenSock);
    vTaskDelay(pdMS_TO_TICKS(500));         // let the reply and FIN leave before the radio stops
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_OK) err = esp_wifi_deinit();
    if (err == ESP_OK)
        ESP_LOGW(TAG, "WiFi off until the next reset");
    else
        ESP_LOGE(TAG, "Turning WiFi off failed: %s", esp_err_to_name(err));
}

static void serverTask(void *arg)
{
    (void)arg;
    int listenSock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(WIFI_LINK_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    int one = 1;
    if (listenSock < 0 ||
        setsockopt(listenSock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) != 0 ||
        bind(listenSock, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
        listen(listenSock, 1) != 0) {
        ESP_LOGE(TAG, "TCP server setup failed: errno %d", errno);
        if (listenSock >= 0) close(listenSock);
        vTaskDelete(NULL);
        return;
    }
    // Non-blocking accept, so a laptop that reconnects is picked up while one is still open.
    fcntl(listenSock, F_SETFL, fcntl(listenSock, F_GETFL, 0) | O_NONBLOCK);

    int client = -1;
    static char msg[MSG_MAX];
    char rx[RX_LINE_MAX];
    size_t rxLen = 0;
    bool wifiOff = false;
    while (!wifiOff) {
        int ns = accept(listenSock, NULL, NULL);
        if (ns >= 0) {
            closeClient(&client, "replaced by a new connection");
            configureClient(ns);
            client = ns;
            rxLen = 0;
            clientConnected = true;
            ESP_LOGI(TAG, "Laptop connected");
        }

        // Also the loop's wait: wakes for each queued line, or after 20 ms.
        size_t n = xMessageBufferReceive(outBuf, msg, sizeof(msg), pdMS_TO_TICKS(20));
        if (client < 0) continue;              // nobody to send to: the line is dropped
        if (n > 0 && !sendAll(client, msg, n)) {
            closeClient(&client, "send failed or timed out");
            continue;
        }

        char in[64];
        int got = recv(client, in, sizeof(in), MSG_DONTWAIT);
        if (got == 0) {
            closeClient(&client, "closed by the laptop");
            continue;
        }
        if (got < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) closeClient(&client, "receive failed");
            continue;
        }
        for (int i = 0; i < got && client >= 0 && !wifiOff; i++) {
            if (in[i] != '\n') {
                if (rxLen < sizeof(rx) - 1) rx[rxLen++] = in[i];   // overlong lines get rejected
                continue;
            }
            if (rxLen > 0 && rx[rxLen - 1] == '\r') rxLen--;
            rx[rxLen] = '\0';
            const char *reply = rxLen > 0 ? handleCommandLine(rx, &wifiOff) : NULL;
            rxLen = 0;
            if (reply != NULL) {
                char out[96];
                int len = snprintf(out, sizeof(out), "%s\n", reply);
                if (!sendAll(client, out, (size_t)len)) closeClient(&client, "send failed");
            }
        }
    }
    stopWifi(&client, listenSock);
    vTaskDelete(NULL);
}

bool wifiLinkStart(void)
{
    if (outBuf != NULL) return true;

    esp_err_t err = nvs_flash_init();     // the WiFi driver keeps its RF calibration in NVS
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) err = nvs_flash_init();
    }
    if (err == ESP_OK) err = esp_netif_init();
    if (err == ESP_OK) {
        err = esp_event_loop_create_default();
        if (err == ESP_ERR_INVALID_STATE) err = ESP_OK;      // already created
    }
    if (err == ESP_OK && esp_netif_create_default_wifi_ap() == NULL) err = ESP_FAIL;

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    if (err == ESP_OK) err = esp_wifi_init(&init);
    if (err == ESP_OK) err = esp_wifi_set_storage(WIFI_STORAGE_RAM);   // no flash writes per boot
    if (err == ESP_OK)
        err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifiEvent, NULL);

    wifi_config_t config = {
        .ap = {
            .ssid = WIFI_LINK_SSID,
            .ssid_len = sizeof(WIFI_LINK_SSID) - 1,
            .password = WIFI_LINK_PASSWORD,
            .channel = WIFI_LINK_CHANNEL,
            .max_connection = 2,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    if (err == ESP_OK) err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err == ESP_OK) err = esp_wifi_set_config(WIFI_IF_AP, &config);
    if (err == ESP_OK) err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi setup failed: %s", esp_err_to_name(err));
        return false;
    }

    outBuf = xMessageBufferCreate(OUT_BUFFER_BYTES);
    cmdQueue = xQueueCreate(CMD_QUEUE_LEN, sizeof(wifi_cmd_t));
    if (outBuf == NULL || cmdQueue == NULL ||
        xTaskCreate(serverTask, "wifilink", TASK_STACK_BYTES, NULL, TASK_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Out of memory starting the WiFi link");
        esp_wifi_stop();
        return false;
    }
    ESP_LOGI(TAG, "WiFi \"%s\" up; the laptop connects to 192.168.4.1:%d",
             WIFI_LINK_SSID, WIFI_LINK_PORT);
    return true;
}

void wifiLinkSendLine(const char *line)
{
    static char msg[MSG_MAX];       // logger task only, so one buffer is enough
    if (outBuf == NULL || !clientConnected) return;
    int len = snprintf(msg, sizeof(msg), "%s\n", line);
    if (len <= 0 || len >= (int)sizeof(msg)) return;
    (void)xMessageBufferSend(outBuf, msg, (size_t)len, 0);
}

bool wifiLinkGetCommand(wifi_cmd_t *cmd)
{
    return cmdQueue != NULL && xQueueReceive(cmdQueue, cmd, 0) == pdTRUE;
}
