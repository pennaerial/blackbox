#include "airspeed.h"

#include <math.h>
#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define AIRSPEED_ADDR 0x28
#define I2C_TIMEOUT_MS 100

static const char *TAG = "airspeed";
static i2c_master_dev_handle_t airspeed = NULL;
static float zero_offset_pa = 0.0f;
static float filt_pa = 0.0f;

bool airspeedSetup(i2c_master_bus_handle_t bus)
{
    if (bus == NULL) return false;
    if (airspeed != NULL) return true;

    i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AIRSPEED_ADDR,
        .scl_speed_hz = 100000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &config, &airspeed);
    if (err != ESP_OK) ESP_LOGE(TAG, "Device setup failed: %s", esp_err_to_name(err));
    return err == ESP_OK;
}

static bool readAirspeedSensor(float *dp_pa, float *temp_c)
{
    uint8_t d[4];
    if (airspeed == NULL ||
        i2c_master_receive(airspeed, d, sizeof(d), I2C_TIMEOUT_MS) != ESP_OK) return false;

    if ((d[0] >> 6) != 0) return false;                 // Status must be 0.

    uint16_t p_raw = ((d[0] & 0x3F) << 8) | d[1];
    uint16_t t_raw = (d[2] << 3) | (d[3] >> 5);

    // -1..+1 psi range, sign matches PX4.
    float psi = -(((float)p_raw - 0.1f * 16383.0f) * 2.0f / (0.8f * 16383.0f) - 1.0f);

    *dp_pa = psi * 6894.757f;
    *temp_c = (float)t_raw * 200.0f / 2047.0f - 50.0f;
    return true;
}

void calibrateAirspeed(void)
{
    ESP_LOGI(TAG, "Calibrating airspeed - keep pitot covered...");
    float sum = 0, dp, t;
    int n = 0, tries = 0;
    while (n < 100 && tries < 500) {
        if (readAirspeedSensor(&dp, &t)) { sum += dp; n++; }
        tries++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (n > 0) {
        zero_offset_pa = sum / n;
        ESP_LOGI(TAG, "Airspeed zero offset: %.2f Pa (%d samples)", zero_offset_pa, n);
    } else {
        ESP_LOGW(TAG, "Airspeed sensor not responding at 0x28!");
    }
    filt_pa = 0;
}

bool readAirspeed(float *dp_pa, float *temp_c, float *airspeed_ms)
{
    float rawDp, t;
    if (!readAirspeedSensor(&rawDp, &t)) return false;

    rawDp -= zero_offset_pa;
    filt_pa += 0.2f * (rawDp - filt_pa);
    float rho = 101325.0f / (287.058f * (t + 273.15f));
    *airspeed_ms = (filt_pa > 2.0f) ? sqrtf(2.0f * filt_pa / rho) : 0.0f;
    *dp_pa = filt_pa;
    *temp_c = t;
    return true;
}
