
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_err.h"

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

void app_main(void)
{
    adc_oneshot_unit_handle_t adc_handle;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(
        &init_config, &adc_handle
    ));

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        adc_handle, CURRENT_CHANNEL, &channel_config
    ));

    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        adc_handle, VOLTAGE_CHANNEL, &channel_config
    ));

    // Initialize ADC calibration
    adc_cali_handle_t cali_handle;

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = CURRENT_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(
        &cali_config, &cali_handle
    ));

    while (1) {
        int currentRaw = 0;
        int voltageRaw = 0;

        int voltageMilliVolts = 0;

        // Read ADC channels
        ESP_ERROR_CHECK(adc_oneshot_read(
            adc_handle, CURRENT_CHANNEL, &currentRaw
        ));

        ESP_ERROR_CHECK(adc_oneshot_read(
            adc_handle, VOLTAGE_CHANNEL, &voltageRaw
        ));

        // Convert the voltage ADC reading to millivolts
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(
            cali_handle, voltageRaw, &voltageMilliVolts
        ));

        // Convert millivolts to volts
        float voltageSignal = voltageMilliVolts / 1000.0f;

        // Calculate actual battery voltage
        float batteryVoltage =
            voltageSignal * VOLTAGE_DIVIDER_RATIO;

        // Scale raw ADC counts directly using the current calibration.
        float batteryCurrent = currentRaw * CURRENT_AMPS_PER_ADC;

        // Print measurements

        printf("Battery Voltage: %.2f V\n", batteryVoltage);
        printf("Battery Current: %.2f A\n", batteryCurrent);
        printf("Current ADC: %d | Voltage ADC: %d\n",
               currentRaw, voltageRaw);
        printf("-------------------------\n");

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
