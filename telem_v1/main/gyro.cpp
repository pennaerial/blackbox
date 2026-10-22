#include "gyro.h"

#include <Arduino.h>
#include <Wire.h>

// LSM6DSO registers and conversion factors.
#define LSM_WHO_AM_I     0x0F   // reads 0x6C for LSM6DSO
#define LSM_CTRL1_XL     0x10
#define LSM_CTRL2_G      0x11
#define LSM_CTRL3_C      0x12
#define LSM_OUT_X_L_G    0x22
#define LSM_OUT_X_L_XL   0x28
#define GYRO_DPS_PER_LSB 0.00875f   // 250 dps range
#define ACCEL_G_PER_LSB  0.000061f  // +/-2 g range

static uint8_t imuAddr = 0;

static bool imuReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(imuAddr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)imuAddr, (int)len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

static void imuWriteReg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(imuAddr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

bool imuSetup()
{
    const uint8_t addrs[2] = {0x6B, 0x6A};
    for (int i = 0; i < 2; i++) {
        imuAddr = addrs[i];
        uint8_t who = 0;
        if (imuReadRegs(LSM_WHO_AM_I, &who, 1)) {
            Serial.printf("IMU found at 0x%02X, WHO_AM_I = 0x%02X\n", imuAddr, who);
            if (who != 0x6C) {
                Serial.println("WARNING: not an LSM6DSO (expected 0x6C).");
                return false;
            }
            imuWriteReg(LSM_CTRL3_C, 0x44);       // BDU on, auto-increment on
            imuWriteReg(LSM_CTRL1_XL, 0x40);      // accel 104 Hz, +/-2 g
            imuWriteReg(LSM_CTRL2_G, 0x40);       // gyro 104 Hz, 250 dps
            return true;
        }
    }
    return false;
}

bool imuRead(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    uint8_t g[6], a[6];
    if (!imuReadRegs(LSM_OUT_X_L_G, g, 6)) return false;
    if (!imuReadRegs(LSM_OUT_X_L_XL, a, 6)) return false;

    *gx = (int16_t)(g[1] << 8 | g[0]) * GYRO_DPS_PER_LSB;
    *gy = (int16_t)(g[3] << 8 | g[2]) * GYRO_DPS_PER_LSB;
    *gz = (int16_t)(g[5] << 8 | g[4]) * GYRO_DPS_PER_LSB;
    *ax = (int16_t)(a[1] << 8 | a[0]) * ACCEL_G_PER_LSB;
    *ay = (int16_t)(a[3] << 8 | a[2]) * ACCEL_G_PER_LSB;
    *az = (int16_t)(a[5] << 8 | a[4]) * ACCEL_G_PER_LSB;
    return true;
}
