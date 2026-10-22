#ifndef V1_GYRO_H
#define V1_GYRO_H

// LSM6DSO gyroscope and accelerometer. Initialize Wire before setup.
bool imuSetup(void);
bool imuRead(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);

#endif /* V1_GYRO_H */
