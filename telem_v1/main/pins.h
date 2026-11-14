#ifndef V1_PINS_H
#define V1_PINS_H

// Every GPIO the logger uses, for the ESP32. Change wiring here only, and check the
// physical board before changing a pin.

#include "driver/gpio.h"
#include "sdkconfig.h"

// I2C bus: BME680, MS4525DO airspeed, LSM6DSO IMU.
#define PIN_I2C_SDA GPIO_NUM_26
#define PIN_I2C_SCL GPIO_NUM_27

// SD card over SPI.
#define PIN_SD_MOSI GPIO_NUM_33
#define PIN_SD_MISO GPIO_NUM_34 // Input-only on the ESP32, no internal pull-up.
#define PIN_SD_SCLK GPIO_NUM_32
#define PIN_SD_CS GPIO_NUM_23

// Power module; both must be ADC1 pins (GPIO32-39). 36 = VP, 39 = VN, both input-only.
#define PIN_POWER_VOLTAGE GPIO_NUM_36
#define PIN_POWER_CURRENT GPIO_NUM_39


// SiK telemetry radio UART (wireless.c, set up in main.c when ENABLE_RADIO is 1). Not wired.
// GPIO6-11 drive the ESP32's flash, so never use them here.
#define PIN_RADIO_TX GPIO_NUM_NC // ESP32 TX -> radio RX
#define PIN_RADIO_RX GPIO_NUM_NC // ESP32 RX <- radio TX

#endif /* V1_PINS_H */
