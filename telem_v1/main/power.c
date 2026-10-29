#include "power.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

// ESP32-C6 ADC channels
#define CURRENT_CHANNEL ADC_CHANNEL_1  // GPIO1
#define VOLTAGE_CHANNEL ADC_CHANNEL_0  // GPIO0

// Voltage calibration
// Actual battery: 14.96 V
// Previous reading: 8.34 V
// Previous scaling factor: 10.0
#define VOLTAGE_DIVIDER_RATIO 17.93f

// Current scale: 15 raw ADC counts = 2.2 A, assuming 0 counts = 0 A.
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
    // VOLTAGE_DIVIDER_RATIO was measured with this scheme created for the current channel.
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = CURRENT_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    esp_err_t err = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (err == ESP_OK) err = adc_oneshot_config_channel(adc_handle, CURRENT_CHANNEL, &channel_config);
    if (err == ESP_OK) err = adc_oneshot_config_channel(adc_handle, VOLTAGE_CHANNEL, &channel_config);
    if (err == ESP_OK) err = adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle);
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
