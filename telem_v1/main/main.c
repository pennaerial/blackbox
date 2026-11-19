/*
 * ESP32 logger: BME680, MS4525DO, LSM6DSO, battery power module, and an SPI SD card.
 * Pin assignments are in pins.h.
 *
 * Startup: the logger brings up its own WiFi network and waits for the laptop
 * (ground_station/dashboard.py --wifi). It does NOT log at power-on: each "start new log"
 * from the laptop closes the open log and starts the next log_NNN.csv. The laptop also runs
 * the IMU mounting calibration (saved as cal_NNN.txt) and can turn WiFi off until reset.
 *
 * Rows hold raw sensor values; the ground station zeroes and calibrates airspeed and scales
 * battery voltage and current. It zeroes the airspeed from the first ~2 s of rows it sees, so
 * keep the pitot OPEN but out of any draft then. Covering it is a pressure transient, not a zero.
 */

#include "main.h"
#include "gyro.h"
#include "airspeed.h"
#include "bme680.h"
#include "imucal.h"
#include "sdcard.h"
#include "power.h"
#include "wifilink.h"
#include "wireless.h"
#include "pins.h"

#include <inttypes.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LOOP_MS 50            // Nominal 20 Hz logging.
#define RADIO_UART UART_NUM_1 // Not the console UART; any pins via pins.h.
#define ENABLE_RADIO 0        // 1 = send CSV rows to the SiK radio; 0 = skip it (bench testing).
#define SD_PIN_TEST 0        // 1 = TEMPORARY wiring test instead of the logger; set back to 0 after.
#define LOG_IF_WIFI_FAILS 1   // 1 = start a log at boot if WiFi cannot start, since nothing else can.
#define STATUS_MS 1000        // How often the laptop gets a #STATUS line.

static const char *TAG = "logger";

#if SD_PIN_TEST
// Drives the SD output lines high one at a time (3 s each) so each can be checked
// with a multimeter at the SD module's header. Also reports the MISO input level.
static void sdPinTest(void)
{
    const gpio_num_t outs[] = {PIN_SD_CS, PIN_SD_SCLK, PIN_SD_MOSI};
    const char *names[] = {"CS", "SCK", "MOSI(DI)"};
    for (int i = 0; i < 3; i++)
    {
        gpio_reset_pin(outs[i]);
        gpio_set_direction(outs[i], GPIO_MODE_OUTPUT);
        gpio_set_level(outs[i], 0);
    }
    gpio_reset_pin(PIN_SD_MISO);
    gpio_set_direction(PIN_SD_MISO, GPIO_MODE_INPUT);

    ESP_LOGW(TAG, "SD PIN TEST: CS=GPIO%d SCK=GPIO%d MOSI=GPIO%d MISO=GPIO%d",
             PIN_SD_CS, PIN_SD_SCLK, PIN_SD_MOSI, PIN_SD_MISO);
    for (;;)
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
                gpio_set_level(outs[j], j == i);
            ESP_LOGI(TAG, "%-8s HIGH (~3.3 V), other two LOW (~0 V) | MISO reads %d",
                     names[i], gpio_get_level(PIN_SD_MISO));
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
}
#endif

// Sends a '#' message to the laptop and echoes it on the USB console.
__attribute__((format(printf, 1, 2))) static void reply(const char *fmt, ...)
{
    static char msg[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);
    wifiLinkSendLine(msg);
    ESP_LOGI(TAG, "%s", msg);
}

// Each call closes the open log, if any, and starts the next log_NNN.csv.
static void startLog(void)
{
    const char *err = sdStartLog();
    if (err != NULL)
        reply("#ERR,LOG,%s", err);
    else
        reply("#LOG,%s", sdLogName());
}

static void handleCommand(const wifi_cmd_t *cmd, bool imuOk)
{
    switch (cmd->type)
    {
    case WIFI_CMD_LOG:
        startLog();
        break;
    case WIFI_CMD_CAL_LEVEL:
    case WIFI_CMD_CAL_PITCH:
    {
        cal_pose_t pose = cmd->type == WIFI_CMD_CAL_LEVEL ? CAL_POSE_LEVEL : CAL_POSE_PITCH;
        if (!imuOk)
        {
            reply("#ERR,CAL,IMU not set up");
            break;
        }
        imuCalBegin(pose);
        reply("#CAL,%s,MEASURING,%d", imuCalPoseName(pose), IMU_CAL_SAMPLES);
        break;
    }
    case WIFI_CMD_CAL_SAVE:
    {
        static char text[1200];
        if (!imuCalFormatFile(text, sizeof(text), cmd->value, sdLogName(),
                              esp_timer_get_time() / 1000))
        {
            reply("#ERR,CAL,measure both poses first");
            break;
        }
        char name[16];
        const char *err = sdWriteCalibration(text, name, sizeof(name));
        if (err != NULL)
            reply("#ERR,CAL,%s", err);
        else
            reply("#CALSAVED,%s", name);
        break;
    }
    }
}

// #STATUS,t_ms,<log name or ->,sd,imu,airspeed,bme,power (1 = working, 0 = not)
static void sendStatus(int64_t nowMs, bool imuOk, bool airspeedOk, bool bmeOk, bool powerOk)
{
    const char *log = sdLogName();
    char msg[96];
    snprintf(msg, sizeof(msg), "#STATUS,%" PRId64 ",%s,%d,%d,%d,%d,%d", nowMs,
             log != NULL ? log : "-", sdMounted(), imuOk, airspeedOk, bmeOk, powerOk);
    wifiLinkSendLine(msg);
}

void app_main(void)
{
#if SD_PIN_TEST
    sdPinTest();
#endif
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "ESP32 logger starting...");

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus);
    bool bmeOk = false;
    bool imuOk = false;
    bool airspeedOk = false;
    if (err == ESP_OK)
    {
        bmeOk = bmeSetup(bus);
        if (!bmeOk)
            ESP_LOGW(TAG, "BME680 NOT found (check wiring / address)");

        imuOk = imuSetup(bus);
        if (!imuOk)
            ESP_LOGW(TAG, "IMU NOT set up");

        airspeedOk = airspeedSetup(bus);
    }
    else
    {
        ESP_LOGE(TAG, "I2C setup failed: %s; sensor fields will be NaN", esp_err_to_name(err));
    }

    // Mount only: a log starts when the laptop asks for one (or below if WiFi fails).
    if (!sdSetup())
        ESP_LOGW(TAG, "SD card NOT available; the mount is retried when a log is started");

    bool powerOk = powerSetup();
    if (!powerOk)
        ESP_LOGW(TAG, "Power module NOT set up; battery fields will be NaN");

    bool radioOk = false;
#if ENABLE_RADIO
    // CSV rows only go to the radio; ESP-IDF log messages stay on the USB console.
    const wireless_config_t radio_config = {
        .uart_port = RADIO_UART,
        .tx_pin = PIN_RADIO_TX,
        .rx_pin = PIN_RADIO_RX,
        .baud_rate = WIRELESS_SIK_DEFAULT_BAUD,
    };
    err = wirelessSetup(&radio_config);
    radioOk = err == ESP_OK;
    if (!radioOk)
        ESP_LOGW(TAG, "Radio UART NOT set up: %s", esp_err_to_name(err));
#else
    ESP_LOGI(TAG, "Radio disabled (ENABLE_RADIO = 0)");
#endif

    if (!wifiLinkStart())
    {
#if LOG_IF_WIFI_FAILS
        ESP_LOGE(TAG, "WiFi failed, so nothing can start a log: starting one now");
        startLog();
#else
        ESP_LOGE(TAG, "WiFi failed; no log will be started");
#endif
    }

    TickType_t lastLoop = xTaskGetTickCount();
    int64_t lastStatus = 0;
    for (;;)
    {
        xTaskDelayUntil(&lastLoop, pdMS_TO_TICKS(LOOP_MS));

        wifi_cmd_t cmd;
        while (wifiLinkGetCommand(&cmd))
            handleCommand(&cmd, imuOk);

        float alt = NAN, press = NAN, bmeT = NAN, hum = NAN;
        if (bmeOk)
            bmeRead(&alt, &press, &bmeT, &hum);

        // Not zeroed or calibrated; the ground station does that.
        float dp = NAN, pitotT = NAN;
        if (airspeedOk)
            readAirspeed(&dp, &pitotT);

        float ax = NAN, ay = NAN, az = NAN, gx = NAN, gy = NAN, gz = NAN;
        if (imuOk)
            imuRead(&ax, &ay, &az, &gx, &gy, &gz);

        // Instantaneous (not averaged), so current spikes show up in the log. Pin signal in
        // mV plus raw ADC counts; the ground station scales them to battery volts and amps.
        float battVmV = NAN, battAmV = NAN, battVAdc = NAN, battAAdc = NAN;
        int vMv, aMv, vAdc, aAdc;
        if (powerOk && powerRead(&vMv, &aMv, &vAdc, &aAdc))
        {
            battVmV = vMv;
            battAmV = aMv;
            battVAdc = vAdc;
            battAAdc = aAdc;
        }

        // Column order must match the header in sdcard.c and LOG_COLS in dashboard.py.
        int64_t nowMs = esp_timer_get_time() / 1000;
        char line[256];
        snprintf(line, sizeof(line),
                 "%" PRIu64 ",%.2f,%.2f,%.2f,%.1f,%.2f,%.1f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.0f,%.0f,%.0f,%.0f",
                 (uint64_t)nowMs, alt, press, bmeT, hum, dp, pitotT,
                 ax, ay, az, gx, gy, gz, battVmV, battAmV, battVAdc, battAAdc);
        puts(line);
        if (radioOk)
            wirelessSendLine(line);
        wifiLinkSendLine(line);
        if (!sdLogLine(line))
            reply("#ERR,LOG,SD write failed; logging stopped");

        const float imu[6] = {ax, ay, az, gx, gy, gz};
        cal_pose_t pose;
        if (imuCalAdd(imu, &pose))
        {
            char result[256];
            imuCalFormatResult(pose, result, sizeof(result));
            reply("%s", result);
        }

        if (nowMs - lastStatus >= STATUS_MS)
        {
            lastStatus = nowMs;
            sendStatus(nowMs, imuOk, airspeedOk, bmeOk, powerOk);
        }
    }
}
