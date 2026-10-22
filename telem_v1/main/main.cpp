/*
 * ESP32-WROOM combined logger
 *   - BME680        (I2C, altitude relative to startup pressure)
 *   - MS4525DO      (I2C 0x28, PX4 airspeed v1.3)
 *   - LSM6DSO IMU   (I2C 0x6A/0x6B, raw register driver)
 *   - SD card       (VSPI)
 *
 * I2C : SDA=21  SCL=22
 * VSPI: MOSI=23 MISO=19 SCLK=18 CS=5
 *
 * Keep the pitot covered / no wind at power-up (airspeed zero calibration).
 * Keep the board still at power-up (altitude zero point).
 */

#include "main.h"
#include "gyro.h"
#include "airspeed.h"
#include "bme680.h"
#include "sdcard.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define LOOP_MS 50                  // 20 Hz logging

static bool imuOk = false;
static bool bmeOk = false;
static bool sdOk = false;
static uint32_t lastLoop = 0;

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nESP32 logger starting...");

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    bmeOk = bmeSetup();
    if (!bmeOk) Serial.println("BME680 NOT found (check wiring / address)");

    imuOk = imuSetup();
    if (!imuOk) Serial.println("IMU NOT set up");

    calibrateAirspeed();

    sdOk = sdSetup();
    if (!sdOk) Serial.println("SD card NOT available (logging to serial only)");

    lastLoop = millis();
}

void loop()
{
    if (millis() - lastLoop < LOOP_MS) return;
    lastLoop += LOOP_MS;

    // --- BME680 ---
    float alt = NAN, press = NAN, bmeT = NAN, hum = NAN;
    if (bmeOk) bmeRead(&alt, &press, &bmeT, &hum);

    // --- Airspeed ---
    float dp = NAN, pitotT = NAN, v = NAN;
    readAirspeed(&dp, &pitotT, &v);

    // --- IMU ---
    float ax = NAN, ay = NAN, az = NAN, gx = NAN, gy = NAN, gz = NAN;
    if (imuOk) imuRead(&ax, &ay, &az, &gx, &gy, &gz);

    // --- Output ---
    char line[220];
    snprintf(line, sizeof(line),
             "%lu,%.2f,%.2f,%.2f,%.1f,%.2f,%.1f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f",
             (unsigned long)millis(), alt, press, bmeT, hum, dp, pitotT, v,
             ax, ay, az, gx, gy, gz);

    Serial.println(line);

    if (sdOk) sdLogLine(line);
}
