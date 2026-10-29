#include "airspeed.h"

#include <math.h>
#include <stdint.h>
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define AIRSPEED_ADDR   0x28
#define I2C_TIMEOUT_MS  100

#define AIR_SAMPLES     8       // Fresh readings averaged per loop (use 4 if LOOP_MS <= 10).
#define AIR_BUDGET_US   8000    // Time allowed for collecting those readings.
#define AIR_FILTER      0.25f   // IIR on the averaged dp: higher = faster but noisier.
#define AIR_DEADBAND_PA 1.5f    // Subtracted from the calibrated dp before the sqrt (noise floor).
#define AIR_USE_CAL     1       // 1 = apply the wind-tunnel calibration table (airCal).
#define AIR_AUTOZERO    1       // 1 = slowly re-zero whenever the reading has been quiet for 10 s.

static const char *TAG = "airspeed";
static i2c_master_dev_handle_t airspeed = NULL;
static float zero_offset_pa = 0.0f;
static float filt_pa = 0.0f;

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
    if (err != ESP_OK) ESP_LOGE(TAG, "Device setup failed: %s", esp_err_to_name(err));
    return err == ESP_OK;
}

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

// Calibration from the PennAiR wind-tunnel sweep: measured dp -> tunnel reference dp, in Pa.
// The probe + tubing under-reads pressure (about 19% at speed, much more at low speed), so this
// maps the measured value back onto the reference. Redo the sweep and replace both tables
// if the probe, tubing, or mounting ever changes.
static const float CAL_MEAS[] = {0.00f, 7.47f, 14.70f, 23.41f, 32.38f, 51.31f, 93.16f, 117.07f,
                                 136.25f, 158.67f, 183.33f, 200.52f, 223.18f, 239.87f, 264.53f, 301.15f};
static const float CAL_TRUE[] = {0.00f, 12.45f, 24.91f, 37.36f, 49.82f, 74.73f, 124.54f, 149.45f,
                                 174.36f, 199.27f, 224.18f, 249.09f, 274.00f, 298.91f, 323.82f, 373.63f};
static const int CAL_N = sizeof(CAL_MEAS) / sizeof(CAL_MEAS[0]);
#define CAL_HIGH_GAIN 1.24f     // Reference/measured ratio above the last table point.

static float airCal(float m)
{
    if (m <= 0.0f) return m * (CAL_TRUE[1] / CAL_MEAS[1]);      // Keep sign for noise below zero.
    for (int i = 1; i < CAL_N; i++) {
        if (m <= CAL_MEAS[i]) {
            float f = (m - CAL_MEAS[i - 1]) / (CAL_MEAS[i] - CAL_MEAS[i - 1]);
            return CAL_TRUE[i - 1] + f * (CAL_TRUE[i] - CAL_TRUE[i - 1]);
        }
    }
    return CAL_TRUE[CAL_N - 1] + CAL_HIGH_GAIN * (m - CAL_MEAS[CAL_N - 1]);
}

void calibrateAirspeed(void)
{
    ESP_LOGI(TAG, "Calibrating airspeed - leave pitot OPEN and out of any draft...");
    vTaskDelay(pdMS_TO_TICKS(500));             // Let the sensor settle after power-up.
    float sum = 0, sum2 = 0, dp, t;
    int n = 0, tries = 0;
    while (n < 200 && tries < 600) {
        // Only accept plausible no-wind readings (within 300 Pa of zero).
        if (readAirspeedAvg(&dp, &t, AIR_SAMPLES) && fabsf(dp) < 300.0f) {
            sum += dp;
            sum2 += dp * dp;
            n++;
        }
        tries++;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (n >= 20) {
        zero_offset_pa = sum / n;
        float sd = sqrtf(fmaxf(sum2 / n - zero_offset_pa * zero_offset_pa, 0.0f));
        ESP_LOGI(TAG, "Airspeed zero offset: %.2f Pa, noise %.2f Pa rms (%d samples)",
                 zero_offset_pa, sd, n);
        if (sd > 1.5f) ESP_LOGW(TAG, "Noisy zero - draft, or tube touched? Reset to recalibrate.");
    } else {
        ESP_LOGW(TAG, "Airspeed sensor not responding at 0x28!");
    }
    filt_pa = 0;
}

bool readAirspeed(float press_hpa, float *dp_pa, float *temp_c, float *airspeed_ms)
{
    float rawDp, t;
    if (!readAirspeedAvg(&rawDp, &t, AIR_SAMPLES)) return false;

    rawDp -= zero_offset_pa;
    filt_pa += AIR_FILTER * (rawDp - filt_pa);

#if AIR_AUTOZERO
    // Within a few Pa of zero for 10 s counts as no wind; nudging the offset then cancels
    // warm-up and temperature drift on the ground.
    static int64_t quietSince = -1;
    int64_t now = esp_timer_get_time() / 1000;
    if (fabsf(filt_pa) < 4.0f) {
        if (quietSince < 0) quietSince = now;
        if (now - quietSince > 10000) zero_offset_pa += 0.002f * filt_pa;
    } else {
        quietSince = -1;
    }
#endif

    float p_pa = isnan(press_hpa) ? 101325.0f : press_hpa * 100.0f;
    float rho = p_pa / (287.058f * (t + 273.15f));

#if AIR_USE_CAL
    float dp_c = airCal(filt_pa);
#else
    float dp_c = filt_pa;
#endif

    // Soft deadband: subtract the dead zone instead of hard-cutting, so there is no jump.
    float eff = dp_c - AIR_DEADBAND_PA;
    *airspeed_ms = (eff > 0.0f) ? sqrtf(2.0f * eff / rho) : 0.0f;
    *dp_pa = filt_pa;
    *temp_c = t;
    return true;
}
