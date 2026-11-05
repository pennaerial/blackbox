#include "wireless.h"

#include <limits.h>
#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "soc/soc_caps.h"

#define RX_BUFFER_BYTES 512
#define TX_TASK_STACK_BYTES 3072
#define TX_TASK_PRIORITY 1

static const char *TAG = "wireless";
static uart_port_t radioUart;
static QueueHandle_t txQueue = NULL;
static bool rxEnabled = false;

typedef struct {
    size_t length;
    char data[WIRELESS_MAX_LINE_LENGTH + 1];
} wireless_line_t;

static void transmitTask(void *arg)
{
    (void)arg;
    wireless_line_t line;
    for (;;) {
        if (xQueueReceive(txQueue, &line, portMAX_DELAY) != pdTRUE) continue;

        // No UART TX ring buffer: only this worker waits for FIFO space.
        // The pending queue retains the newest sample while this one is sent.
        int written = uart_write_bytes(radioUart, line.data, line.length);
        if (written != (int)line.length) {
            ESP_LOGE(TAG, "UART write failed (%d/%u bytes)", written,
                     (unsigned)line.length);
            // End a possible partial row so the next row starts a fresh line.
            (void)uart_write_bytes(radioUart, "\n", 1);
        }
    }
}

esp_err_t wirelessSetup(const wireless_config_t *config)
{
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    if (txQueue != NULL) return ESP_ERR_INVALID_STATE;
    if ((int)config->uart_port < 0 || config->uart_port >= SOC_UART_HP_NUM ||
        !GPIO_IS_VALID_OUTPUT_GPIO(config->tx_pin) ||
        (config->rx_pin != GPIO_NUM_NC && !GPIO_IS_VALID_GPIO(config->rx_pin)) ||
        config->tx_pin == config->rx_pin || config->baud_rate <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

#if defined(CONFIG_ESP_CONSOLE_UART) && CONFIG_ESP_CONSOLE_UART
    if (config->uart_port == CONFIG_ESP_CONSOLE_UART_NUM) {
        return ESP_ERR_INVALID_ARG;
    }
#endif
    if (uart_is_driver_installed(config->uart_port)) return ESP_ERR_INVALID_STATE;

    QueueHandle_t queue = xQueueCreate(1, sizeof(wireless_line_t));
    if (queue == NULL) return ESP_ERR_NO_MEM;

    // RX buffer is required by the UART driver even for a TX-only connection.
    // TX buffering is handled by our one-slot queue, keeping telemetry fresh.
    esp_err_t err = uart_driver_install(config->uart_port, RX_BUFFER_BYTES,
                                        0, 0, NULL, 0);
    if (err != ESP_OK) {
        vQueueDelete(queue);
        return err;
    }

    const uart_config_t uartConfig = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    err = uart_param_config(config->uart_port, &uartConfig);
    if (err != ESP_OK) goto fail;

    err = uart_set_pin(config->uart_port, config->tx_pin, config->rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) goto fail;

    radioUart = config->uart_port;
    rxEnabled = config->rx_pin != GPIO_NUM_NC;
    txQueue = queue;
    if (xTaskCreate(transmitTask, "sik_tx", TX_TASK_STACK_BYTES, NULL,
                    TX_TASK_PRIORITY, NULL) != pdPASS) {
        txQueue = NULL;
        rxEnabled = false;
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    ESP_LOGI(TAG, "SiK UART%d ready at %d baud (RF link not verified)",
             (int)radioUart, config->baud_rate);
    return ESP_OK;

fail:
    (void)uart_driver_delete(config->uart_port);
    vQueueDelete(queue);
    return err;
}

esp_err_t wirelessSendLine(const char *line)
{
    if (txQueue == NULL) return ESP_ERR_INVALID_STATE;
    if (line == NULL) return ESP_ERR_INVALID_ARG;

    size_t length = strnlen(line, WIRELESS_MAX_LINE_LENGTH + 1);
    if (length > WIRELESS_MAX_LINE_LENGTH) return ESP_ERR_INVALID_SIZE;
    if (length == 0 || memchr(line, '\r', length) != NULL ||
        memchr(line, '\n', length) != NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    wireless_line_t message = {0};
    memcpy(message.data, line, length);
    message.data[length] = '\n';
    message.length = length + 1;
    return xQueueOverwrite(txQueue, &message) == pdPASS ? ESP_OK : ESP_FAIL;
}

int wirelessRead(uint8_t *buffer, size_t capacity, TickType_t timeout_ticks)
{
    if (txQueue == NULL || !rxEnabled || buffer == NULL ||
        capacity == 0 || capacity > INT_MAX) {
        return -1;
    }
    return uart_read_bytes(radioUart, buffer, (uint32_t)capacity, timeout_ticks);
}
