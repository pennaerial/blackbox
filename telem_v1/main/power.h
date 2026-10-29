#ifndef V1_POWER_H
#define V1_POWER_H

#include <stdbool.h>

// Battery voltage (GPIO0) and current (GPIO1) from the power module on ADC1.
// Calibration is provisional; see the constants in power.c.
bool powerSetup(void);
bool powerRead(float *voltage_v, float *current_a, int *voltage_raw, int *current_raw);

#endif /* V1_POWER_H */
