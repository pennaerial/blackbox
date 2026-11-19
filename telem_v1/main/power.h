#ifndef V1_POWER_H
#define V1_POWER_H

#include <stdbool.h>

#define POWER_SAMPLE_INTERVAL_MS 100
#define POWER_PRINT_INTERVAL_MS 500
#define POWER_WINDOW_SIZE 50

// Battery voltage and current on the ADC1 pins set in pins.h (12 bits).
// Call setup once, then use the read functions from a single task.
bool powerSetup(void);

// Instantaneous readings; does not update the rolling average. Outputs are the signal at
// each pin in millivolts (ADC counts converted with the chip's eFuse calibration) and the
// raw 12-bit counts. Scaling to battery volts and amps is done on the ground station.
// Both read functions require four non-null outputs and successful setup.
// On failure, return false without changing the outputs or averaging state.
bool powerRead(int *voltage_mv, int *current_mv, int *voltage_raw, int *current_raw);

// Call every 100 ms (or faster) to maintain a nominal 5-second rolling average.
// Samples immediately on the first call, then at most once per 100 ms; calls
// between samples return cached averages. Averages use only collected samples
// during warm-up; raw outputs are the latest successful sample, not averages.
// Slower calls or failed reads lengthen the window; no background task runs.
// The caller can print at POWER_PRINT_INTERVAL_MS while continuing to sample.
// The averages are of the pin signal in millivolts, like powerRead().
bool powerReadAverage(float *voltage_mv, float *current_mv, int *voltage_raw, int *current_raw);

#endif /* V1_POWER_H */
