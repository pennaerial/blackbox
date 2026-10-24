#ifndef V1_BME680_H
#define V1_BME680_H

#include <stdbool.h>
#include "driver/i2c_master.h"

// BME680 environmental sensor, using an initialized shared I2C bus.
bool bmeSetup(i2c_master_bus_handle_t bus);
bool bmeRead(float *alt_m, float *press_hpa, float *temp_c, float *hum_pct);

#endif /* V1_BME680_H */
