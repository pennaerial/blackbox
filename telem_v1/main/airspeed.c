#include "airspeed.h"

#include <stdint.h>
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#define AIRSPEED_ADDR   0x28
#define I2C_TIMEOUT_MS  100

#define AIR_SAMPLES     8       // Fresh readings averaged per loop (use 4 if LOOP_MS <= 10).
#define AIR_BUDGET_US   8000    // Time allowed for collecting those readings.

// Zeroing, filtering, the wind-tunnel calibration and the airspeed conversion are done on
// the ground station (ground_station/dashboard.py) from the logged dp_raw_pa column.

static const char *TAG = "airspeed";
static i2c_master_dev_handle_t airspeed = NULL;

// Returns false for stale data (sensor not refreshed yet) or garbage.
static bool readAirspeedOnce(float *dp_pa, float *temp_c)
{
    uint8_t d[4];
    if (airspeed == NULL ||
        i2c_master_receive(airspeed, d, sizeof(d), I2C_TIMEOUT_MS) != ESP_OK) return false;

    if ((d[0] >> 6) != 0) return false;                 // Status must be 0 (0b10 = stale).

    uint16_t p_raw = ((d[0] & 0x3F) << 8) | d[1];
    uint16_t t_raw = (d[2] << 3) | (d[3] >> 5);

    // Valid output is 10%..90% of full scale (1638..14745).
    if (p_raw < 1500 || p_raw > 14900) return false;

    // -1..+1 psi range, sign matches PX4. 1 count is about 1.05 Pa.
    float psi = -(((float)p_raw - 0.1f * 16383.0f) * 2.0f / (0.8f * 16383.0f) - 1.0f);

    *dp_pa = psi * 6894.757f;
    *temp_c = (float)t_raw * 200.0f / 2047.0f - 50.0f;
    return true;
}

// The sensor refreshes about every 0.5 ms and stale reads are rejected, so averaging
// fresh readings cuts the noise by roughly sqrt(want).
static bool readAirspeedAvg(float *dp_pa, float *temp_c, int want)
{
    float sdp = 0, st = 0;
    int got = 0;
    int64_t start = esp_timer_get_time();
    while (got < want && esp_timer_get_time() - start < AIR_BUDGET_US) {
        float dp, t;
        if (readAirspeedOnce(&dp, &t)) { sdp += dp; st += t; got++; }
        esp_rom_delay_us(600);
    }
    if (got == 0) return false;
    *dp_pa = sdp / got;
    *temp_c = st / got;
    return true;
}

bool airspeedSetup(i2c_master_bus_handle_t bus)
{
    if (bus == NULL) return false;
    if (airspeed != NULL) return true;

    i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AIRSPEED_ADDR,
        .scl_speed_hz = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &config, &airspeed);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Device setup failed: %s", esp_err_to_name(err));
        airspeed = NULL;
        return false;
    }

    // Registering the device does not talk to it, so check that it answers. Stay set up
    // either way: a failed read is logged as nan and the next loop tries again.
    float dp, t;
    if (readAirspeedAvg(&dp, &t, AIR_SAMPLES))
        ESP_LOGI(TAG, "MS4525DO at 0x28: %.1f Pa, %.1f C (not zeroed)", dp, t);
    else
        ESP_LOGW(TAG, "Airspeed sensor not responding at 0x28!");
    return true;
}

bool readAirspeed(float *dp_pa, float *temp_c)
{
    return readAirspeedAvg(dp_pa, temp_c, AIR_SAMPLES);
}
