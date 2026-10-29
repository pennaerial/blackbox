#ifndef V1_AIRSPEED_H
#define V1_AIRSPEED_H

#include <stdbool.h>
#include "driver/i2c_master.h"

// Register the MS4525DO on an initialized shared I2C bus before calibration.
bool airspeedSetup(i2c_master_bus_handle_t bus);
void calibrateAirspeed(void);
// press_hpa sets air density; pass NAN to use sea-level standard pressure.
// dp_pa is the filtered, zeroed reading before the wind-tunnel calibration.
bool readAirspeed(float press_hpa, float *dp_pa, float *temp_c, float *airspeed_ms);

#endif /* V1_AIRSPEED_H */
