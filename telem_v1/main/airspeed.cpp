#include "airspeed.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define AIRSPEED_ADDR 0x28

static float zero_offset_pa = 0.0f;
static float filt_pa = 0.0f;

static bool readAirspeedSensor(float *dp_pa, float *temp_c)
{
    uint8_t d[4];
    if (Wire.requestFrom((int)AIRSPEED_ADDR, 4) != 4) return false;
    for (int i = 0; i < 4; i++) d[i] = Wire.read();

    if ((d[0] >> 6) != 0) return false;                 // status must be 0

    uint16_t p_raw = ((d[0] & 0x3F) << 8) | d[1];
    uint16_t t_raw = (d[2] << 3) | (d[3] >> 5);

    // -1..+1 psi range, sign matches PX4
    float psi = -(((float)p_raw - 0.1f * 16383.0f) * 2.0f / (0.8f * 16383.0f) - 1.0f);

    *dp_pa  = psi * 6894.757f;
    *temp_c = (float)t_raw * 200.0f / 2047.0f - 50.0f;
    return true;
}

void calibrateAirspeed()
{
    Serial.println("Calibrating airspeed - keep pitot covered...");
    float sum = 0, dp, t;
    int n = 0, tries = 0;
    while (n < 100 && tries < 500) {
        if (readAirspeedSensor(&dp, &t)) { sum += dp; n++; }
        tries++;
        delay(10);
    }
    if (n > 0) {
        zero_offset_pa = sum / n;
        Serial.printf("Airspeed zero offset: %.2f Pa (%d samples)\n", zero_offset_pa, n);
    } else {
        Serial.println("Airspeed sensor not responding at 0x28!");
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
