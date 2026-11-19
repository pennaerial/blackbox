#include "imucal.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    bool done;
    int n;
    float mean[6];      // ax, ay, az (g), gx, gy, gz (deg/s), raw sensor axes
    float sd[6];        // standard deviation of each: shows whether it was held still
} pose_result_t;

static const char *const POSE_NAMES[CAL_POSE_COUNT] = {"LEVEL", "PITCH"};
static const char *const FILE_KEYS[CAL_POSE_COUNT] = {"level", "pitch"};

static pose_result_t results[CAL_POSE_COUNT];
static int activePose = -1;
static int count;
static double sum[6], sumSq[6];

const char *imuCalPoseName(cal_pose_t pose)
{
    return POSE_NAMES[pose];
}

void imuCalBegin(cal_pose_t pose)
{
    activePose = pose;
    count = 0;
    memset(sum, 0, sizeof(sum));
    memset(sumSq, 0, sizeof(sumSq));
    results[pose].done = false;
}

bool imuCalAdd(const float imu[6], cal_pose_t *done)
{
    if (activePose < 0) return false;
    for (int i = 0; i < 6; i++)
        if (!isfinite(imu[i])) return false;

    for (int i = 0; i < 6; i++) {
        sum[i] += imu[i];
        sumSq[i] += (double)imu[i] * imu[i];
    }
    if (++count < IMU_CAL_SAMPLES) return false;

    pose_result_t *r = &results[activePose];
    r->n = count;
    for (int i = 0; i < 6; i++) {
        double mean = sum[i] / count;
        r->mean[i] = (float)mean;
        r->sd[i] = (float)sqrt(fmax(sumSq[i] / count - mean * mean, 0.0));
    }
    r->done = true;
    *done = (cal_pose_t)activePose;
    activePose = -1;
    return true;
}

void imuCalFormatResult(cal_pose_t pose, char *buf, size_t len)
{
    const pose_result_t *r = &results[pose];
    snprintf(buf, len,
             "#CAL,%s,DONE,%d,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f",
             POSE_NAMES[pose], r->n, r->mean[0], r->mean[1], r->mean[2], r->mean[3], r->mean[4],
             r->mean[5], r->sd[0], r->sd[1], r->sd[2], r->sd[3], r->sd[4], r->sd[5]);
}

bool imuCalFormatFile(char *buf, size_t len, const float r_cm[3], const char *logName,
                      int64_t t_ms)
{
    if (!results[CAL_POSE_LEVEL].done || !results[CAL_POSE_PITCH].done) return false;

    int used = snprintf(buf, len,
                        "# PennAiR black box IMU mounting calibration, written by the logger.\n"
                        "# Means and standard deviations of the raw IMU axes in each still pose.\n"
                        "# The ground station works out the mount rotation from the two poses.\n"
                        "t_ms,%" PRId64 "\n"
                        "log,%s\n",
                        t_ms, logName != NULL ? logName : "none");
    for (int p = 0; p < CAL_POSE_COUNT && used > 0 && (size_t)used < len; p++) {
        const pose_result_t *r = &results[p];
        used += snprintf(buf + used, len - used,
                         "%s_samples,%d\n"
                         "%s_acc_g,%.5f,%.5f,%.5f\n"
                         "%s_acc_sd_g,%.5f,%.5f,%.5f\n"
                         "%s_gyro_dps,%.4f,%.4f,%.4f\n"
                         "%s_gyro_sd_dps,%.4f,%.4f,%.4f\n",
                         FILE_KEYS[p], r->n,
                         FILE_KEYS[p], r->mean[0], r->mean[1], r->mean[2],
                         FILE_KEYS[p], r->sd[0], r->sd[1], r->sd[2],
                         FILE_KEYS[p], r->mean[3], r->mean[4], r->mean[5],
                         FILE_KEYS[p], r->sd[3], r->sd[4], r->sd[5]);
    }
    if (used > 0 && (size_t)used < len) {
        used += snprintf(buf + used, len - used,
                         "# IMU position from the CG: x forward, y toward the left wing, z up.\n"
                         "r_imu_from_cg_cm,%.2f,%.2f,%.2f\n",
                         r_cm[0], r_cm[1], r_cm[2]);
    }
    return used > 0 && (size_t)used < len;
}
