#include "bme680.h"

#include <math.h>
#include <string.h>
#include "bme68x.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_TIMEOUT_MS 100

static const char *TAG = "bme680";
static i2c_master_dev_handle_t bme = NULL;
static struct bme68x_dev sensor;
static struct bme68x_conf settings;
static float baseline_hpa = 0;

static BME68X_INTF_RET_TYPE sensorRead(uint8_t reg, uint8_t *data, uint32_t len, void *context)
{
    esp_err_t err = i2c_master_transmit_receive((i2c_master_dev_handle_t)context,
                                               &reg, 1, data, len, I2C_TIMEOUT_MS);
    return err == ESP_OK ? BME68X_INTF_RET_SUCCESS : -1;
}

static BME68X_INTF_RET_TYPE sensorWrite(uint8_t reg, const uint8_t *data, uint32_t len, void *context)
{
    uint8_t buffer[256];
    if (len > sizeof(buffer) - 1) return -1;
    buffer[0] = reg;
    memcpy(buffer + 1, data, len);
    esp_err_t err = i2c_master_transmit((i2c_master_dev_handle_t)context,
                                       buffer, len + 1, I2C_TIMEOUT_MS);
    return err == ESP_OK ? BME68X_INTF_RET_SUCCESS : -1;
}

static void sensorDelay(uint32_t period_us, void *context)
{
    (void)context;
    if (period_us < 1000) {
        esp_rom_delay_us(period_us);
    } else {
        // Round up and allow a full tick so the requested minimum always elapses.
        vTaskDelay(pdMS_TO_TICKS((period_us + 999) / 1000) + 1);
    }
}

static bool performReading(struct bme68x_data *data)
{
    if (bme == NULL || bme68x_set_op_mode(BME68X_FORCED_MODE, &sensor) != BME68X_OK) return false;
    sensorDelay(bme68x_get_meas_dur(BME68X_FORCED_MODE, &settings, &sensor), NULL);

    uint8_t count = 0;
    return bme68x_get_data(BME68X_FORCED_MODE, data, &count, &sensor) == BME68X_OK &&
           count > 0 && (data->status & BME68X_NEW_DATA_MSK);
}

bool bmeSetup(i2c_master_bus_handle_t bus)
{
    if (bus == NULL) return false;
    if (bme != NULL) return baseline_hpa > 0;

    const uint8_t addrs[] = {0x77, 0x76};
    for (size_t i = 0; i < sizeof(addrs); i++) {
        i2c_device_config_t config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addrs[i],
            .scl_speed_hz = 100000,
        };
        if (i2c_master_bus_add_device(bus, &config, &bme) != ESP_OK) return false;

        sensor = (struct bme68x_dev) {
            .intf_ptr = bme,
            .intf = BME68X_I2C_INTF,
            .read = sensorRead,
            .write = sensorWrite,
            .delay_us = sensorDelay,
        };
        if (bme68x_init(&sensor) == BME68X_OK) break;
        i2c_master_bus_rm_device(bme);
        bme = NULL;
    }
    if (bme == NULL) return false;

    settings = (struct bme68x_conf) {
        .os_hum = BME68X_OS_1X,
        .os_temp = BME68X_OS_1X,
        .os_pres = BME68X_OS_1X,
        .filter = BME68X_FILTER_SIZE_3,
        .odr = BME68X_ODR_NONE,
    };
    struct bme68x_heatr_conf heater = {.enable = BME68X_DISABLE};
    if (bme68x_set_conf(&settings, &sensor) != BME68X_OK ||
        bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heater, &sensor) != BME68X_OK) goto fail;

    // Discard the first reading, then take the startup pressure baseline.
    struct bme68x_data data;
    performReading(&data);
    vTaskDelay(pdMS_TO_TICKS(50));
    if (!performReading(&data) || !isfinite(data.pressure) || data.pressure <= 0) goto fail;

    baseline_hpa = data.pressure / 100.0f;
    ESP_LOGI(TAG, "Altitude zero set. Baseline: %.2f hPa", baseline_hpa);
    return true;

fail:
    i2c_master_bus_rm_device(bme);
    bme = NULL;
    return false;
}

bool bmeRead(float *alt_m, float *press_hpa, float *temp_c, float *hum_pct)
{
    struct bme68x_data data;
    if (baseline_hpa <= 0 || !performReading(&data)) return false;

    *press_hpa = data.pressure / 100.0f;
    *temp_c = data.temperature;
    *hum_pct = data.humidity;
    // Same barometric formula as before, using this reading's pressure for altitude too.
    *alt_m = 44330.0f * (1.0f - powf(*press_hpa / baseline_hpa, 0.1903f));
    return true;
}
