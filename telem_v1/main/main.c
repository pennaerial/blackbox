/*
 * ESP32 logger: BME680, MS4525DO, LSM6DSO, battery power module, and an SPI SD card.
 * Pin assignments are in pins.h.
 * During startup keep the board still, and leave the pitot OPEN but out of any draft.
 * Covering it creates a pressure transient, not a true zero.
 */

#include "main.h"
#include "gyro.h"
#include "airspeed.h"
#include "bme680.h"
#include "sdcard.h"
#include "power.h"
#include "wireless.h"
#include "pins.h"

#include <inttypes.h>
#include <math.h>
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
        if (airspeedOk)
            calibrateAirspeed();
    }
    else
    {
        ESP_LOGE(TAG, "I2C setup failed: %s; sensor fields will be NaN", esp_err_to_name(err));
    }

    bool sdOk = sdSetup();
    if (!sdOk)
        ESP_LOGW(TAG, "SD card NOT available (logging to console only)");

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
    ESP_LOGI(TAG, "Radio disabled (ENABLE_RADIO = 0); CSV goes to USB console and SD only");
#endif

    TickType_t lastLoop = xTaskGetTickCount();
    for (;;)
    {
        xTaskDelayUntil(&lastLoop, pdMS_TO_TICKS(LOOP_MS));

        float alt = NAN, press = NAN, bmeT = NAN, hum = NAN;
        if (bmeOk)
            bmeRead(&alt, &press, &bmeT, &hum);

        float dp = NAN, pitotT = NAN, v = NAN;
        if (airspeedOk)
            readAirspeed(press, &dp, &pitotT, &v);

        float ax = NAN, ay = NAN, az = NAN, gx = NAN, gy = NAN, gz = NAN;
        if (imuOk)
            imuRead(&ax, &ay, &az, &gx, &gy, &gz);

        // Instantaneous (not averaged), so current spikes show up in the log.
        float battV = NAN, battA = NAN;
        int battVRaw, battARaw;
        if (powerOk)
            powerRead(&battV, &battA, &battVRaw, &battARaw);

        char line[220];
        snprintf(line, sizeof(line),
                 "%" PRIu64 ",%.2f,%.2f,%.2f,%.1f,%.2f,%.1f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f",
                 (uint64_t)(esp_timer_get_time() / 1000), alt, press, bmeT, hum, dp, pitotT, v,
                 ax, ay, az, gx, gy, gz, battV, battA);
        puts(line);
        if (radioOk)
            wirelessSendLine(line);

        if (sdOk)
            sdOk = sdLogLine(line);
    }
}
