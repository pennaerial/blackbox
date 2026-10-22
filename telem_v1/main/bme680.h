#ifndef V1_BME680_H
#define V1_BME680_H

// BME680 environmental sensor. Initialize Wire before setup.
bool bmeSetup(void);
bool bmeRead(float *alt_m, float *press_hpa, float *temp_c, float *hum_pct);

#endif /* V1_BME680_H */
