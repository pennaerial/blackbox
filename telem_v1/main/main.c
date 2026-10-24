
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
#define VOLTAGE_DIVIDER_RATIO 17.9376f

// Current calibration (placeholder values)
#define CURRENT_SENSITIVITY 0.050f  // V per ampere
#define CURRENT_ZERO_OFFSET 0.0f    // V at zero current

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

        int currentMilliVolts = 0;
        int voltageMilliVolts = 0;

        // Read ADC channels
        ESP_ERROR_CHECK(adc_oneshot_read(
            adc_handle, CURRENT_CHANNEL, &currentRaw
        ));

        ESP_ERROR_CHECK(adc_oneshot_read(
            adc_handle, VOLTAGE_CHANNEL, &voltageRaw
        ));

        // Convert raw ADC readings to millivolts
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(
            cali_handle, currentRaw, &currentMilliVolts
        ));

        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(
            cali_handle, voltageRaw, &voltageMilliVolts
        ));

        // Convert millivolts to volts
        float currentSignal = currentMilliVolts / 1000.0f;
        float voltageSignal = voltageMilliVolts / 1000.0f;

        // Calculate actual battery voltage
        float batteryVoltage =
            voltageSignal * VOLTAGE_DIVIDER_RATIO;

        // Calculate battery current
        // Requires calibration for your power module
        float batteryCurrent =
            (currentSignal - CURRENT_ZERO_OFFSET)
            / CURRENT_SENSITIVITY;

        // Print measurements
        printf("Battery Voltage: %.2f V\n", batteryVoltage);
        printf("Battery Current: %.2f A (uncalibrated)\n",
               batteryCurrent);
        printf("Current ADC: %d | Voltage ADC: %d\n",
               currentRaw, voltageRaw);
        printf("-------------------------\n");

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
