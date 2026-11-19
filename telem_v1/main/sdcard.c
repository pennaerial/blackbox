#include "sdcard.h"
#include "pins.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#define MOUNT_POINT "/sdcard"
#define MAX_FILE_NUMBER 999

static const char *TAG = "sdcard";
static sdmmc_card_t *card = NULL;
static FILE *logFile = NULL;
static char logName[16];       // "log_NNN.csv" while a log is open
static int64_t lastFlush = 0;

static bool flushLog(void)
{
    return fflush(logFile) == 0 && fsync(fileno(logFile)) == 0;
}

// First unused MOUNT_POINT/<prefix>_NNN.<ext>; name gets the bare file name.
static bool nextFreeName(const char *prefix, const char *ext, char *path, size_t pathLen,
                         char *name, size_t nameLen)
{
    for (int i = 1; i <= MAX_FILE_NUMBER; i++) {
        snprintf(name, nameLen, "%s_%03d.%s", prefix, i, ext);
        snprintf(path, pathLen, MOUNT_POINT "/%s", name);
        struct stat st;
        if (stat(path, &st) == 0) continue;
        return errno == ENOENT;
    }
    return false;
}

bool sdSetup(void)
{
    if (card != NULL) return true;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 4000;
    spi_bus_config_t bus_config = {
        .mosi_io_num = PIN_SD_MOSI,
        .miso_io_num = PIN_SD_MISO,
        .sclk_io_num = PIN_SD_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    esp_err_t err = spi_bus_initialize(host.slot, &bus_config, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI setup failed: %s", esp_err_to_name(err));
        return false;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_SD_CS;
    slot_config.host_id = host.slot;
    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 2,             // the open log plus a calibration file
        .allocation_unit_size = 16 * 1024,
    };
    err = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(err));
        card = NULL;
        spi_bus_free(host.slot);
        return false;
    }
    ESP_LOGI(TAG, "SD card mounted; no log until the laptop starts one");
    return true;
}

bool sdMounted(void)
{
    return card != NULL;
}

static void closeLog(void)
{
    if (logFile == NULL) return;
    fclose(logFile);           // flushes; the previous log is kept as it is
    logFile = NULL;
    ESP_LOGI(TAG, "Closed %s", logName);
    logName[0] = '\0';
}

const char *sdStartLog(void)
{
    closeLog();
    if (!sdSetup()) return "SD card not found";     // retries the mount, e.g. card inserted late

    char path[48];
    if (!nextFreeName("log", "csv", path, sizeof(path), logName, sizeof(logName))) {
        logName[0] = '\0';
        return "no free log name (log_001.csv to log_999.csv all used)";
    }
    logFile = fopen(path, "w");
    if (logFile == NULL) {
        logName[0] = '\0';
        return "could not create the log file";
    }
    // Must match the row format in main.c and LOG_COLS in ground_station/dashboard.py.
    if (fputs("t_ms,alt_m,press_hpa,bme_temp_c,hum_pct,dp_raw_pa,pitot_temp_c,"
              "ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
              "batt_v_sig_mv,batt_a_sig_mv,batt_v_adc,batt_a_adc\r\n", logFile) == EOF || !flushLog()) {
        fclose(logFile);
        logFile = NULL;
        logName[0] = '\0';
        return "could not write the log header";
    }
    lastFlush = esp_timer_get_time() / 1000;
    ESP_LOGI(TAG, "Logging to %s", logName);
    return NULL;
}

const char *sdLogName(void)
{
    return logFile != NULL ? logName : NULL;
}

bool sdLogLine(const char *line)
{
    if (logFile == NULL) return true;

    bool ok = fprintf(logFile, "%s\r\n", line) >= 0;
    int64_t now = esp_timer_get_time() / 1000;
    if (ok && now - lastFlush > 1000) {
        ok = flushLog();
        lastFlush = now;
    }
    if (!ok) {
        ESP_LOGE(TAG, "SD write/sync failed; stopping %s", logName);
        fclose(logFile);
        logFile = NULL;
        logName[0] = '\0';
    }
    return ok;
}

const char *sdWriteCalibration(const char *text, char *name, size_t nameLen)
{
    if (!sdSetup()) return "SD card not found";
    char path[48];
    if (!nextFreeName("cal", "txt", path, sizeof(path), name, nameLen))
        return "no free calibration name (cal_001.txt to cal_999.txt all used)";
    FILE *f = fopen(path, "w");
    if (f == NULL) return "could not create the calibration file";
    bool ok = fputs(text, f) != EOF && fflush(f) == 0 && fsync(fileno(f)) == 0;
    ok = fclose(f) == 0 && ok;
    if (!ok) return "could not write the calibration file";
    ESP_LOGI(TAG, "Saved calibration to %s", name);
    return NULL;
}
