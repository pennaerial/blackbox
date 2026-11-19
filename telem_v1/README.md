# Telemetry logger firmware

Native ESP-IDF firmware for the ESP32, using ESP-IDF 6.1. The application is
C, with no Arduino or Adafruit dependency. Bosch's BSD-licensed BME68x C driver
is included under `components/bme68x/` for sensor compensation.

`main/main.c` owns the shared I2C bus and runs the logger from `app_main()`.
The matching `.c`/`.h` pairs in `main/` handle:

- `gyro`: LSM6DSO gyroscope and accelerometer, probing 0x6B then 0x6A.
- `airspeed`: MS4525DO at 0x28. Logs the raw differential pressure (averaged, not
  zeroed or calibrated) and the sensor temperature.
- `bme680`: temperature, humidity, pressure and altitude relative to startup,
  probing 0x77 then 0x76. Altitude uses the same pressure sample as the CSV row.
- `sdcard`: FAT-formatted SD card over SPI. Starts a new `log_NNN.csv` on command,
  syncs it about once a second, and writes `cal_NNN.txt` calibration files.
- `wifilink`: the logger's own WiFi network and a TCP link to the laptop (see
  [Field setup over WiFi](#field-setup-over-wifi)).
- `imucal`: averages the raw IMU over the two still poses of the mounting calibration.
- `power`: battery voltage (GPIO36) and current (GPIO39) on ADC1. The logger calls
  `powerRead()` once per row and writes the instantaneous pin signal in millivolts
  (converted with the chip's eFuse ADC calibration) and the raw 12-bit ADC counts.
  `powerReadAverage()` (50-sample, nominal 5-second rolling average of the signal)
  is available but not used by the logger.
- `wireless`: sends each CSV row to the SiK telemetry radio on UART1 at 57600 baud
  (TX/RX pins in `pins.h`). Not wired: `ENABLE_RADIO` is 0 in `main.c`, so radio
  setup is skipped. Log messages stay on the USB console.

## Wiring and behavior

All GPIO assignments are in `main/pins.h`; change them there only. Pin numbers and
sensor settings are preserved from the original logger source:

| Signal | GPIO |
| --- | --- |
| I2C SDA | 26 |
| I2C SCL | 27 |
| SD MOSI | 33 |
| SD MISO | 34 |
| SD SCLK | 32 |
| SD CS | 23 |
| Battery voltage | 36 (VP) |
| Battery current | 39 (VN) |

I2C runs at 400 kHz and the SD SPI maximum is 4 MHz, matching the updated Arduino
logger. If a sensor is unreliable, drop the I2C devices back to 100 kHz. These
assignments still need checking against the physical board and wiring before
hardware testing.

### Field setup over WiFi

The logger does **not** log at power-on. On reset it starts its own WiFi network
(`blackbox`, password and channel in `main/wifilink.h`) and waits. On the laptop, join that
network, then from `ground_station/` run `uv run dashboard.py --wifi`. The window shows the
live data with the usual diagrams, plus a logger bar:

- **Start new log**: closes the open log, if any, and starts the next `log_NNN.csv`. Press
  it again after a false start; the earlier file is kept. A log keeps recording until power
  is cut, and at most about the last second is lost.
- **Calibrate IMU...**: a wizard. Hold the aircraft level and still, then nose up (about
  30-45 degrees, wings level), then type the IMU's position from the CG in cm (x forward,
  y toward the left wing, z up). The logger averages the raw IMU over 60 rows (3 s) per
  pose and saves both poses and the offset to the next `cal_NNN.txt`. The dashboard works
  out the mount rotation, which may be at any angle, and checks that each pose was held
  still and that the nose was raised at least 15 degrees.
- **Turn WiFi off...**: after a confirmation, turns the logger's WiFi off to save power,
  until it is power-cycled. A recording log carries on. Only this button turns WiFi off: if
  the laptop drops out of range or the dashboard closes, WiFi stays on and the dashboard
  reconnects.

The bar shows whether a log is recording, the SD card and which sensors are working, from a
status line the logger sends every second. If WiFi cannot start at boot, the logger starts a
log on its own, since nothing else could (`LOG_IF_WIFI_FAILS` in `main.c`).

The link is text lines over TCP port 3333 at 192.168.4.1. The logger sends every CSV row,
plus lines starting with `#` (status, replies and errors). The commands are listed in
`main/wifilink.h`. To replay a log in aircraft axes, pass its calibration:
`uv run dashboard.py --replay log_004.csv --cal cal_002.txt`.

### Raw logging; calibration on the ground station

The logger records raw sensor values, and `ground_station/dashboard.py` applies the
airspeed and battery calibration, live and on replay. Its settings (wind-tunnel table,
filter, divider ratio, amps per volt) are at the top of that file. A calibration
change therefore also applies to logs that are already recorded. The ESP32 keeps only
the sensor conversions that need data the ground never sees: the MS4525DO counts to Pa
(assuming a -1 to +1 psi sensor) with up to 8 fresh readings averaged within 8 ms per
loop, and ADC counts to millivolts using the chip's eFuse calibration.

On replay the dashboard zeroes the airspeed from the first 40 rows (about 2 s) of a log,
so for 2 s after pressing **Start new log** leave the pitot open but out of any draft.
Do not cover it: that creates a pressure transient, not a true zero. Live, the zero is
taken when the dashboard connects or when **A** is pressed. The dashboard shows the
zero offset and its noise, and warns above 1.5 Pa rms. After the zero it applies a
0.25 low-pass filter, maps the value through the PennAiR wind-tunnel calibration table,
subtracts a 1.5 Pa soft deadband, and converts using air density from the BME680
pressure (101325 Pa if unavailable) and the pitot temperature. After 10 s within 4 Pa
of zero, the zero offset slowly re-adjusts to cancel drift. Redo the wind-tunnel sweep
if the probe, tubing or mounting changes.

Battery voltage is the voltage signal x 10.11. Current is the current signal x 19.65 A/V,
or 0 A at or below 150 mV (the ESP32 ADC floor). The voltage calibration was measured
on the C6 and must be re-checked on the ESP32.

The IMU keeps the original 104 Hz, +/-2 g and 250 degrees/s settings. The BME680
uses 1x oversampling, filter coefficient 3 and no gas heater. Its readings are
non-blocking: each row reports the measurement started on the previous loop.

The logger has a nominal 50 ms period. Actual throughput depends on sensor and
SD I/O latency. Missing or failed sensor readings are written as `nan`.
Console output contains CSV rows plus ESP-IDF diagnostic messages. SD files are
named `log_001.csv` through `log_999.csv` and are synced approximately once a second.
Their columns are `t_ms`, `alt_m`, `press_hpa`, `bme_temp_c`, `hum_pct`, `dp_raw_pa`
(Pa, not zeroed), `pitot_temp_c`, `ax_g`/`ay_g`/`az_g`, `gx_dps`/`gy_dps`/`gz_dps`,
`batt_v_sig_mv` and `batt_a_sig_mv` (pin signal, mV), and `batt_v_adc` and
`batt_a_adc` (raw ADC counts). Logs from older firmware have calibrated `dp_pa`,
`airspeed_ms`, `batt_v` and `batt_a` columns instead; the dashboard reads both. If all
filenames are used, a new log cannot start; existing files are kept. Mount failures never
trigger formatting, and the mount is retried when a log is started. A write/sync failure
stops that log, the laptop is told, and console output continues.

`cal_NNN.txt` files are `key,values` lines: the time and the log that was recording, then
for each pose (`level`, `pitch`) the sample count and the means and standard deviations
of the raw accelerometer (g) and gyro (deg/s) axes, and `r_imu_from_cg_cm`.

## Build

In VS Code, open [blackbox.code-workspace](../blackbox.code-workspace) with
**File > Open Workspace from File**. It keeps `penn-air` as the first workspace
folder and adds `telem_v1` as the firmware project. Run **ESP-IDF: Pick a Workspace
Folder** and select `telem_v1` once; the extension normally remembers it on reopen.
Then use **ESP-IDF: Build Your Project** (Ctrl+E, then B on Windows).

Keep installation paths and serial ports in personal VS Code **User** settings,
not this repository's `.vscode/settings.json`. The ESP-IDF extension inherits
`idf.currentSetup` and `idf.port` from there. Choose the ESP32's port when flashing;
the SiK radio has a separate port. ESP-IDF's setup/port commands may write a new
folder override, which takes precedence over User settings.

Windows and macOS each build into their own local `build/` directory. It and the
generated `sdkconfig` / `sdkconfig.old` files are already ignored by Git. Share
the source and `sdkconfig.defaults`; keep SDK installation paths, serial ports
and build output on each computer. Use the same ESP-IDF version and target on
both computers, and put configuration changes intended for everyone in
`sdkconfig.defaults`.

In an activated ESP-IDF 6.1 terminal, from `telem_v1/`, select the target once:

```text
idf.py set-target esp32
idf.py build
```

`sdkconfig.defaults` supplies the target and a 1 kHz FreeRTOS tick for the 50 ms
logging period. An existing `sdkconfig` takes
precedence over defaults; check these settings when reusing an older build.
Building needs no sensor-library downloads because the Bosch source is local.
