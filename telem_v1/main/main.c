/*
 * ESP32-C6 logger: BME680, MS4525DO, LSM6DSO, and an SPI SD card.
 * Existing wiring: I2C SDA=21, SCL=22; SD MOSI=23, MISO=19, SCLK=18, CS=5.
 * Keep the pitot covered and the board still during startup calibration.
 */

#include "main.h"
#include "gyro.h"
#include "airspeed.h"
#include "bme680.h"
#include "sdcard.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22
#define LOOP_MS 50                  // Nominal 20 Hz logging.

static const char *TAG = "logger";

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "ESP32 logger starting...");

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus);
    bool bmeOk = false;
    bool imuOk = false;
    bool airspeedOk = false;
    if (err == ESP_OK) {
        bmeOk = bmeSetup(bus);
        if (!bmeOk) ESP_LOGW(TAG, "BME680 NOT found (check wiring / address)");

        imuOk = imuSetup(bus);
        if (!imuOk) ESP_LOGW(TAG, "IMU NOT set up");

        airspeedOk = airspeedSetup(bus);
        if (airspeedOk) calibrateAirspeed();
    } else {
        ESP_LOGE(TAG, "I2C setup failed: %s; sensor fields will be NaN", esp_err_to_name(err));
    }

    bool sdOk = sdSetup();
    if (!sdOk) ESP_LOGW(TAG, "SD card NOT available (logging to console only)");

    TickType_t lastLoop = xTaskGetTickCount();
    for (;;) {
        xTaskDelayUntil(&lastLoop, pdMS_TO_TICKS(LOOP_MS));

        float alt = NAN, press = NAN, bmeT = NAN, hum = NAN;
        if (bmeOk) bmeRead(&alt, &press, &bmeT, &hum);

        float dp = NAN, pitotT = NAN, v = NAN;
        if (airspeedOk) readAirspeed(&dp, &pitotT, &v);

        float ax = NAN, ay = NAN, az = NAN, gx = NAN, gy = NAN, gz = NAN;
        if (imuOk) imuRead(&ax, &ay, &az, &gx, &gy, &gz);

        char line[220];
        snprintf(line, sizeof(line),
                 "%" PRIu64 ",%.2f,%.2f,%.2f,%.1f,%.2f,%.1f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f",
                 (uint64_t)(esp_timer_get_time() / 1000), alt, press, bmeT, hum, dp, pitotT, v,
                 ax, ay, az, gx, gy, gz);
        puts(line);

        if (sdOk) sdOk = sdLogLine(line);
    }
}
