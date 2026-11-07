#include "power.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_timer.h"

// ESP32-C6 ADC1 pins: GPIO1 = channel 1, GPIO0 = channel 0.
#define VOLTAGE_PIN 1
#define CURRENT_PIN 0

#define VOLTAGE_DIVIDER_RATIO 10.11f
#define CURRENT_AMPS_PER_VOLT 36.364f
// The reference sketch does not apply its unused CURRENT_ADC_OFFSET (5.17).

static const char *TAG = "power";
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_channel_t voltage_channel;
static adc_channel_t current_channel;
static adc_cali_handle_t voltage_cali = NULL;
static adc_cali_handle_t current_cali = NULL;

static float voltage_samples[POWER_WINDOW_SIZE];
static float current_samples[POWER_WINDOW_SIZE];
static float voltage_sum;
static float current_sum;
static int sample_index;
static int sample_count;
static int latest_voltage_raw;
static int latest_current_raw;
static int64_t last_sample_us;

static esp_err_t createCalibration(adc_channel_t channel, adc_cali_handle_t *handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t config = {
        .unit_id = ADC_UNIT_1,
        .chan = channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    return adc_cali_create_scheme_curve_fitting(&config, handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    (void)channel;
    adc_cali_line_fitting_config_t config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
#if CONFIG_IDF_TARGET_ESP32
        .default_vref = 1100,
#endif
    };
    return adc_cali_create_scheme_line_fitting(&config, handle);
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static void deleteCalibration(adc_cali_handle_t handle)
{
    if (handle == NULL) return;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_delete_scheme_curve_fitting(handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_delete_scheme_line_fitting(handle);
#endif
}

bool powerSetup(void)
{
    if (voltage_cali != NULL && current_cali != NULL) return true;

    adc_unit_t voltage_unit, current_unit;
    esp_err_t err = adc_oneshot_io_to_channel(VOLTAGE_PIN, &voltage_unit, &voltage_channel);
    if (err == ESP_OK) err = adc_oneshot_io_to_channel(CURRENT_PIN, &current_unit, &current_channel);
    if (err != ESP_OK || voltage_unit != ADC_UNIT_1 || current_unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "GPIO%d and GPIO%d must both be ADC1 pins; select the ESP32-C6 target",
                 VOLTAGE_PIN, CURRENT_PIN);
        return false;
    }

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_chan_cfg_t channel_config = {
        // ESP-IDF's name for the attenuation used by Arduino ADC_11db.
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    err = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (err == ESP_OK) err = adc_oneshot_config_channel(adc_handle, voltage_channel, &channel_config);
    if (err == ESP_OK) err = adc_oneshot_config_channel(adc_handle, current_channel, &channel_config);
    if (err == ESP_OK) err = createCalibration(voltage_channel, &voltage_cali);
    if (err == ESP_OK) err = createCalibration(current_channel, &current_cali);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC setup failed: %s", esp_err_to_name(err));
        deleteCalibration(voltage_cali);
        deleteCalibration(current_cali);
        if (adc_handle != NULL) adc_oneshot_del_unit(adc_handle);
        adc_handle = NULL;
        voltage_cali = NULL;
        current_cali = NULL;
        return false;
    }
    return true;
}

bool powerRead(float *voltage_v, float *current_a, int *voltage_raw, int *current_raw)
{
    if (voltage_v == NULL || current_a == NULL || voltage_raw == NULL || current_raw == NULL ||
        voltage_cali == NULL || current_cali == NULL) return false;

    int voltageRaw = 0, currentRaw = 0;
    int voltageMilliVolts = 0;
    int currentMilliVolts = 0;
    if (adc_oneshot_read(adc_handle, voltage_channel, &voltageRaw) != ESP_OK ||
        adc_oneshot_read(adc_handle, current_channel, &currentRaw) != ESP_OK ||
        adc_cali_raw_to_voltage(voltage_cali, voltageRaw, &voltageMilliVolts) != ESP_OK ||
        adc_cali_raw_to_voltage(current_cali, currentRaw, &currentMilliVolts) != ESP_OK) {
        return false;
    }

    // Use each raw sample for both its diagnostic count and calibrated voltage.
    *voltage_raw = voltageRaw;
    *current_raw = currentRaw;
    *voltage_v = voltageMilliVolts / 1000.0f * VOLTAGE_DIVIDER_RATIO;
    *current_a = currentMilliVolts / 1000.0f * CURRENT_AMPS_PER_VOLT;
    return true;
}

bool powerReadAverage(float *voltage_v, float *current_a, int *voltage_raw, int *current_raw)
{
    if (voltage_v == NULL || current_a == NULL || voltage_raw == NULL || current_raw == NULL ||
        voltage_cali == NULL || current_cali == NULL) return false;

    int64_t now_us = esp_timer_get_time();
    if (sample_count == 0 || now_us - last_sample_us >= POWER_SAMPLE_INTERVAL_MS * 1000LL) {
        float voltage, current;
        int voltageRaw, currentRaw;
        if (!powerRead(&voltage, &current, &voltageRaw, &currentRaw)) return false;

        if (sample_count == POWER_WINDOW_SIZE) {
            voltage_sum -= voltage_samples[sample_index];
            current_sum -= current_samples[sample_index];
        } else {
            sample_count++;
        }

        voltage_samples[sample_index] = voltage;
        current_samples[sample_index] = current;
        voltage_sum += voltage;
        current_sum += current;
        sample_index = (sample_index + 1) % POWER_WINDOW_SIZE;
        latest_voltage_raw = voltageRaw;
        latest_current_raw = currentRaw;
        last_sample_us = now_us;
    }

    *voltage_v = voltage_sum / sample_count;
    *current_a = current_sum / sample_count;
    *voltage_raw = latest_voltage_raw;
    *current_raw = latest_current_raw;
    return true;
}
