#include "power.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

// ESP32 ADC1 channels. GPIO34/35 are input-only and not shared with ADC2, which Wi-Fi uses.
#define VOLTAGE_CHANNEL ADC_CHANNEL_6  // GPIO34
#define CURRENT_CHANNEL ADC_CHANNEL_7  // GPIO35

// Used only when the chip has no Vref burned into eFuse.
#define DEFAULT_VREF_MV 1100

// Voltage calibration, measured on the ESP32; re-check on other targets.
// Actual battery: 14.96 V
// Previous reading: 8.34 V
// Previous scaling factor: 10.0
#define VOLTAGE_DIVIDER_RATIO 17.93f

// Current scale: 15 raw ADC counts = 2.2 A, assuming 0 counts = 0 A.
// Raw counts were measured on the ESP32 ADC and must be re-measured on other targets.
#define CURRENT_AMPS_PER_ADC (2.2f / 15.0f)

static const char *TAG = "power";
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t cali_handle = NULL;

bool powerSetup(void)
{
    if (cali_handle != NULL) return true;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
        .default_vref = DEFAULT_VREF_MV,
    };
#else
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
        .default_vref = DEFAULT_VREF_MV,
    };
#endif

    esp_err_t err = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (err == ESP_OK) err = adc_oneshot_config_channel(adc_handle, CURRENT_CHANNEL, &channel_config);
    if (err == ESP_OK) err = adc_oneshot_config_channel(adc_handle, VOLTAGE_CHANNEL, &channel_config);
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
    if (err == ESP_OK) err = adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle);
#else
    if (err == ESP_OK) err = adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle);
#endif
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC setup failed: %s", esp_err_to_name(err));
        if (adc_handle != NULL) adc_oneshot_del_unit(adc_handle);
        adc_handle = NULL;
        cali_handle = NULL;
        return false;
    }
    return true;
}

bool powerRead(float *voltage_v, float *current_a, int *voltage_raw, int *current_raw)
{
    int voltageMilliVolts = 0;
    if (cali_handle == NULL ||
        adc_oneshot_read(adc_handle, CURRENT_CHANNEL, current_raw) != ESP_OK ||
        adc_oneshot_read(adc_handle, VOLTAGE_CHANNEL, voltage_raw) != ESP_OK ||
        adc_cali_raw_to_voltage(cali_handle, *voltage_raw, &voltageMilliVolts) != ESP_OK) {
        return false;
    }

    *voltage_v = voltageMilliVolts / 1000.0f * VOLTAGE_DIVIDER_RATIO;
    *current_a = *current_raw * CURRENT_AMPS_PER_ADC;
    return true;
}
