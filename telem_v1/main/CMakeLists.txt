idf_component_register(
    SRCS
        "main.c"
        "gyro.c"
        "airspeed.c"
        "bme680.c"
        "sdcard.c"
        "power.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c
    PRIV_REQUIRES esp_adc esp_driver_spi esp_driver_sdspi fatfs sdmmc esp_timer esp_rom bme68x
)
