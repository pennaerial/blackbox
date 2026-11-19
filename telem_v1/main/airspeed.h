#ifndef V1_AIRSPEED_H
#define V1_AIRSPEED_H

#include <stdbool.h>
#include "driver/i2c_master.h"

// Register the MS4525DO on an initialized shared I2C bus; warns if it does not answer.
bool airspeedSetup(i2c_master_bus_handle_t bus);
// Average of up to 8 fresh readings: differential pressure in Pa and sensor temperature.
// dp_pa is not zeroed, filtered or calibrated; the ground station does all of that.
bool readAirspeed(float *dp_pa, float *temp_c);

#endif /* V1_AIRSPEED_H */
