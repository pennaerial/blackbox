#ifndef V1_AIRSPEED_H
#define V1_AIRSPEED_H

// MS4525DO airspeed sensor. Initialize Wire before calibration/readings.
void calibrateAirspeed(void);
bool readAirspeed(float *dp_pa, float *temp_c, float *airspeed_ms);

#endif /* V1_AIRSPEED_H */
