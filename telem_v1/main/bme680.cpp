#include "bme680.h"

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

static Adafruit_BME680 bme;
static float baseline_hpa = 0;

bool bmeSetup()
{
    if (!bme.begin(0x77) && !bme.begin(0x76)) return false;

    bme.setTemperatureOversampling(BME680_OS_1X);
    bme.setHumidityOversampling(BME680_OS_1X);
    bme.setPressureOversampling(BME680_OS_1X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(0, 0);                              // heater off = fast reads

    // Throw away the first reading, then take the baseline
    bme.performReading();
    delay(50);
    if (bme.performReading()) {
        baseline_hpa = bme.pressure / 100.0;
        Serial.printf("Altitude zero set. Baseline: %.2f hPa\n", baseline_hpa);
        return true;
    }
    return false;
}

bool bmeRead(float *alt_m, float *press_hpa, float *temp_c, float *hum_pct)
{
    if (!bme.performReading()) return false;

    *press_hpa = bme.pressure / 100.0;
    *temp_c = bme.temperature;
    *hum_pct = bme.humidity;
    *alt_m = bme.readAltitude(baseline_hpa);
    return true;
}
