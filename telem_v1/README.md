# Telemetry logger firmware

Native ESP-IDF firmware for the ESP32, using ESP-IDF 6.1. The application is
C, with no Arduino or Adafruit dependency. Bosch's BSD-licensed BME68x C driver
is included under `components/bme68x/` for sensor compensation.

`main/main.c` owns the shared I2C bus and runs the logger from `app_main()`.
The matching `.c`/`.h` pairs in `main/` handle:

- `gyro`: LSM6DSO gyroscope and accelerometer, probing 0x6B then 0x6A.
- `airspeed`: MS4525DO at 0x28, startup zero calibration and filtered,
  wind-tunnel-calibrated airspeed.
- `bme680`: temperature, humidity, pressure and altitude relative to startup,
  probing 0x77 then 0x76. Altitude uses the same pressure sample as the CSV row.
- `sdcard`: FAT-formatted SD card over SPI, CSV file creation and periodic sync.
- `power`: battery voltage (GPIO34) and current (GPIO35) on ADC1. Not yet called
  by the logger or written to the CSV. Its calibration was measured on the
  ESP32-C6 and must be redone on the ESP32.

## Wiring and behavior

Pin numbers and sensor settings are preserved from the original logger source:

| Signal | GPIO |
| --- | --- |
| I2C SDA | 21 |
| I2C SCL | 22 |
| SD MOSI | 23 |
| SD MISO | 19 |
| SD SCLK | 18 |
| SD CS | 5 |

I2C runs at 400 kHz and the SD SPI maximum is 4 MHz, matching the updated Arduino
logger. If a sensor is unreliable, drop the I2C devices back to 100 kHz. These
assignments still need checking against the physical board and wiring before
hardware testing.

During startup keep the board still, and leave the pitot open but out of any
draft. Do not cover it: that creates a pressure transient, not a true zero. The
calibration log reports the zero offset and its noise, and warns above 1.5 Pa rms.

The airspeed conversion assumes a -1 to +1 psi sensor. Each loop averages up to
8 fresh readings within 8 ms, then applies a 0.25 low-pass filter. The filtered
value is mapped through the PennAiR wind-tunnel calibration table in `airspeed.c`,
reduced by a 1.5 Pa soft deadband, and converted using air density from the
BME680 pressure (101325 Pa if unavailable) and the pitot temperature. After 10 s
within 4 Pa of zero, the zero offset slowly re-adjusts to cancel drift. The
`dp_pa` column is the filtered value before the calibration table. Redo the
wind-tunnel sweep if the probe, tubing or mounting changes.

The IMU keeps the original 104 Hz, +/-2 g and 250 degrees/s settings. The BME680
uses 1x oversampling, filter coefficient 3 and no gas heater. Its readings are
non-blocking: each row reports the measurement started on the previous loop.

The logger has a nominal 50 ms period. Actual throughput depends on sensor and
SD I/O latency. Missing or failed sensor readings are written as `nan`.
Console output contains CSV rows plus ESP-IDF diagnostic messages. SD files are
named `log_001.csv` through `log_999.csv`, with the original CSV columns and
units, and are synced approximately once a second. If all filenames are used,
SD logging is disabled; existing files are kept. Mount failures never trigger
formatting, and write/sync failures stop SD logging while console output continues.

## Build

In an activated ESP-IDF 6.1 terminal, from `telem_v1/`, select the target once:

```text
idf.py set-target esp32
idf.py build
```

`sdkconfig.defaults` supplies the target and a 1 kHz FreeRTOS tick for the 10 ms
calibration delays and 50 ms logging period. An existing `sdkconfig` takes
precedence over defaults; check these settings when reusing an older build.
Building needs no sensor-library downloads because the Bosch source is local.
