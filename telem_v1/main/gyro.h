#ifndef V1_GYRO_H
#define V1_GYRO_H

#include <stdbool.h>
#include "driver/i2c_master.h"

// LSM6DSO gyroscope and accelerometer, using an initialized shared I2C bus.
bool imuSetup(i2c_master_bus_handle_t bus);
bool imuRead(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);

#endif /* V1_GYRO_H */
