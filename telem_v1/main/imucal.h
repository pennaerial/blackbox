#ifndef V1_IMUCAL_H
#define V1_IMUCAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// IMU mounting calibration capture. The logger only averages the raw IMU over two still
// poses (aircraft level, then nose raised) and saves them with the CG-to-IMU offset typed
// on the laptop. The mount rotation is worked out on the ground station from these values.
// Call everything from the logger task.

#define IMU_CAL_SAMPLES 60      // logger rows averaged per pose (3 s at 20 Hz)

typedef enum {
    CAL_POSE_LEVEL,
    CAL_POSE_PITCH,
    CAL_POSE_COUNT,
} cal_pose_t;

// "LEVEL" / "PITCH", as used in the laptop protocol.
const char *imuCalPoseName(cal_pose_t pose);

// Starts averaging the next IMU_CAL_SAMPLES valid readings for this pose, replacing any
// earlier result for it and cancelling a measurement of the other pose still in progress.
void imuCalBegin(cal_pose_t pose);

// Adds one reading (ax, ay, az in g; gx, gy, gz in deg/s). Readings with a NaN are skipped.
// Returns true when this reading finished the pose being measured; *done gets that pose.
bool imuCalAdd(const float imu[6], cal_pose_t *done);

// "#CAL,<POSE>,DONE,n,ax,ay,az,gx,gy,gz,<the six standard deviations>" for the laptop.
void imuCalFormatResult(cal_pose_t pose, char *buf, size_t len);

// Text of a cal_NNN.txt file. False until both poses are measured, or if buf is too small.
// r_cm is the IMU position from the CG in cm (x forward, y left wing, z up).
bool imuCalFormatFile(char *buf, size_t len, const float r_cm[3], const char *logName,
                      int64_t t_ms);

#endif /* V1_IMUCAL_H */
