/*
 * ESP32-C6 logger: BME680, MS4525DO, LSM6DSO, and an SPI SD card.
 * Existing wiring: I2C SDA=21, SCL=22; SD MOSI=23, MISO=19, SCLK=18, CS=5.
 * Keep the pitot covered and the board still during startup calibration.
 */

#include "main.h"
#include "gyro.h"
#include "airspeed.h"
#include "bme680.h"
#include "sdcard.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>

void app_main(void)
{

}
