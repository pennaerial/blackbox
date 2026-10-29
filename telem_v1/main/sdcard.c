#include "sdcard.h"

#include <errno.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#define SPI_MOSI 23
#define SPI_MISO 19
#define SPI_SCLK 18
#define SD_CS 5
#define MOUNT_POINT "/sdcard"

static const char *TAG = "sdcard";
static FILE *logFile = NULL;
static sdmmc_card_t *card = NULL;
static int64_t lastFlush = 0;

static bool flushLog(void)
{
    return fflush(logFile) == 0 && fsync(fileno(logFile)) == 0;
}

bool sdSetup(void)
{
    if (logFile != NULL) return true;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 4000;
    spi_bus_config_t bus_config = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_SCLK,
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
    slot_config.gpio_cs = SD_CS;
    slot_config.host_id = host.slot;
    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 1,
        .allocation_unit_size = 16 * 1024,
    };
    err = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(err));
        spi_bus_free(host.slot);
        return false;
    }

    char name[40];
    for (int i = 1; i < 1000; i++) {
        snprintf(name, sizeof(name), MOUNT_POINT "/log_%03d.csv", i);
        struct stat st;
        if (stat(name, &st) == 0) continue;
        if (errno != ENOENT) break;
        logFile = fopen(name, "w");
        break;
    }
    if (logFile == NULL) {
        ESP_LOGE(TAG, "Cannot create a new log file (log_001.csv through log_999.csv)");
        goto fail;
    }

    ESP_LOGI(TAG, "Logging to %s", name);
    if (fputs("t_ms,alt_m,press_hpa,bme_temp_c,hum_pct,dp_pa,pitot_temp_c,airspeed_ms,"
              "ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\r\n", logFile) == EOF || !flushLog()) {
        ESP_LOGE(TAG, "Failed to write CSV header");
        fclose(logFile);
        logFile = NULL;
        goto fail;
    }
    return true;

fail:
    esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
    card = NULL;
    spi_bus_free(host.slot);
    return false;
}

bool sdLogLine(const char *line)
{
    if (logFile == NULL) return false;

    bool ok = fprintf(logFile, "%s\r\n", line) >= 0;
    int64_t now = esp_timer_get_time() / 1000;
    if (ok && now - lastFlush > 1000) {
        ok = flushLog();
        lastFlush = now;
    }
    if (!ok) {
        ESP_LOGE(TAG, "SD write/sync failed; stopping SD logging");
        fclose(logFile);
        logFile = NULL;
    }
    return ok;
}
