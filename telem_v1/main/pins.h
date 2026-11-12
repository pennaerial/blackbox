#ifndef V1_PINS_H
#define V1_PINS_H

// Every GPIO the logger uses, for the ESP32. Change wiring here only, and check the
// physical board before changing a pin.

#include "driver/gpio.h"
#include "sdkconfig.h"

// I2C bus: BME680, MS4525DO airspeed, LSM6DSO IMU.
#define PIN_I2C_SDA GPIO_NUM_21
#define PIN_I2C_SCL GPIO_NUM_22

// SD card over SPI.
#define PIN_SD_MOSI GPIO_NUM_23
#define PIN_SD_MISO GPIO_NUM_19
#define PIN_SD_SCLK GPIO_NUM_18
#define PIN_SD_CS GPIO_NUM_5

// Power module; both must be ADC1 pins.
#if CONFIG_IDF_TARGET_ESP32C6
// ESP32-C6 bench wiring used for the current calibration.
#define PIN_POWER_VOLTAGE GPIO_NUM_1
#define PIN_POWER_CURRENT GPIO_NUM_0
#else
// GPIO0/1 are not ADC1 pins on the ESP32. Proposed, not yet wired.
#define PIN_POWER_VOLTAGE GPIO_NUM_34
#define PIN_POWER_CURRENT GPIO_NUM_35
#endif

// SiK telemetry radio UART (wireless.c, set up in main.c). Proposed, check before wiring.
#define PIN_RADIO_TX GPIO_NUM_6  // ESP32 TX -> radio RX
#define PIN_RADIO_RX GPIO_NUM_7  // ESP32 RX <- radio TX

#endif /* V1_PINS_H */
