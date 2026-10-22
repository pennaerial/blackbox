#include "sdcard.h"

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <stdio.h>

#define SPI_MOSI  23
#define SPI_MISO  19
#define SPI_SCLK  18
#define SD_CS      5

static File logFile;
static uint32_t lastFlush = 0;

bool sdSetup()
{
    SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI, SD_CS);
    if (!SD.begin(SD_CS, SPI, 20000000)) return false;

    char name[24];
    for (int i = 1; i < 1000; i++) {
        snprintf(name, sizeof(name), "/log_%03d.csv", i);
        if (!SD.exists(name)) break;
    }
    logFile = SD.open(name, FILE_WRITE);
    if (!logFile) return false;

    Serial.printf("Logging to %s\n", name);
    logFile.println("t_ms,alt_m,press_hpa,bme_temp_c,hum_pct,dp_pa,pitot_temp_c,airspeed_ms,"
                    "ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps");
    logFile.flush();
    return true;
}

void sdLogLine(const char *line)
{
    logFile.println(line);
    if (millis() - lastFlush > 1000) {      // flush once a second
        logFile.flush();
        lastFlush = millis();
    }
}
