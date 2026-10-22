#include "gyro.h"

#include <stdint.h>
#include "esp_log.h"

#define LSM_WHO_AM_I     0x0F
#define LSM_CTRL1_XL     0x10
#define LSM_CTRL2_G      0x11
#define LSM_CTRL3_C      0x12
#define LSM_OUT_X_L_G    0x22
#define LSM_OUT_X_L_XL   0x28
#define GYRO_DPS_PER_LSB 0.00875f   // 250 dps range.
#define ACCEL_G_PER_LSB  0.000061f  // +/-2 g range.
#define I2C_TIMEOUT_MS   100

static const char *TAG = "gyro";
static i2c_master_dev_handle_t imu = NULL;

static bool imuReadRegs(uint8_t reg, uint8_t *buf, size_t len)
{
    return imu != NULL &&
           i2c_master_transmit_receive(imu, &reg, 1, buf, len, I2C_TIMEOUT_MS) == ESP_OK;
}

static bool imuWriteReg(uint8_t reg, uint8_t val)
{
    const uint8_t data[] = {reg, val};
    return i2c_master_transmit(imu, data, sizeof(data), I2C_TIMEOUT_MS) == ESP_OK;
}

bool imuSetup(i2c_master_bus_handle_t bus)
{
    if (bus == NULL) return false;
    if (imu != NULL) return true;

    const uint8_t addrs[] = {0x6B, 0x6A};
    for (size_t i = 0; i < sizeof(addrs); i++) {
        i2c_device_config_t config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addrs[i],
            .scl_speed_hz = 100000,
        };
        if (i2c_master_bus_add_device(bus, &config, &imu) != ESP_OK) return false;

        uint8_t who = 0;
        if (imuReadRegs(LSM_WHO_AM_I, &who, 1)) {
            ESP_LOGI(TAG, "IMU found at 0x%02X, WHO_AM_I = 0x%02X", addrs[i], who);
            if (who == 0x6C &&
                imuWriteReg(LSM_CTRL3_C, 0x44) &&     // BDU and auto-increment on.
                imuWriteReg(LSM_CTRL1_XL, 0x40) &&    // Accel 104 Hz, +/-2 g.
                imuWriteReg(LSM_CTRL2_G, 0x40)) {     // Gyro 104 Hz, 250 dps.
                return true;
            }
            ESP_LOGW(TAG, "Unexpected IMU ID or configuration write failed");
        }
        i2c_master_bus_rm_device(imu);
        imu = NULL;
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
