# Bosch BME68x sensor driver

Unmodified Bosch C driver for the BME680/BME688, from
[BME68x_SensorAPI](https://github.com/boschsensortec/BME68x_SensorAPI/tree/80ea120a8b8ac987d7d79eb68a9ed796736be845),
commit `80ea120a8b8ac987d7d79eb68a9ed796736be845`.

`bme68x.c`, `bme68x.h`, `bme68x_defs.h`, and `LICENSE` are upstream files.
`CMakeLists.txt` integrates them as a local ESP-IDF component. The application
provides ESP-IDF I2C and delay callbacks in `main/bme680.c`. Floating-point
temperature, pressure, and humidity compensation uses the driver's default
`BME68X_USE_FPU` configuration. No Arduino or BSEC dependency is required.
