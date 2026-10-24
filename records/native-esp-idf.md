# Native ESP-IDF logger

`telem_v1/` uses native ESP-IDF C APIs for the ESP32-C6, with no Arduino or
Adafruit dependency. Bosch's unmodified BME68x C driver supplies BME680
compensation. The target was confirmed by the member; physical wiring has not
been verified.

The existing device modules were converted to ESP-IDF I2C, SD-over-SPI/FATFS,
FreeRTOS timing and an `app_main()` entry point. Pin numbers, sensor settings,
airspeed calculations and CSV columns were retained. Altitude now uses the
same pressure sample as its CSV row. SD logging stops on write errors or
filename exhaustion instead of overwriting an existing log.

The complete firmware, including bootloader and Bosch driver, compiled and
linked successfully for `esp32c6` using the installed ESP-IDF 6.1 toolchain.
It has not been flashed or tested against sensors or an SD card. See the
[firmware guide](../telem_v1/README.md) for setup, assumptions and build commands.
