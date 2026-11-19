# blackbox
Design of black box and live telemetry system for micro and regular class plane in testing and competition use.

See [the firmware guide](telem_v1/README.md) for the native ESP-IDF logger,
device modules, preserved pin assignments, and ESP32 build instructions.

`ground_station/dashboard.py` is a laptop-side live dashboard for the logger's CSV lines.
The logger does not log at power-on. At the field, join the logger's `blackbox` WiFi and
run `uv run dashboard.py --wifi` from `ground_station/` to start logs, calibrate the IMU
mounting and turn the WiFi off before flight (see the
[firmware guide](telem_v1/README.md#field-setup-over-wifi)). Over USB serial, run
`uv run dashboard.py` (or pass the port, e.g. `uv run dashboard.py COM5`).
`uv run dashboard.py --replay log_007.csv` plays back an SD card log instead; copy
logs into `ground_station/logs/`. Through the telemetry radio, pass the ground
radio's port and its baud: `uv run dashboard.py COM7 --baud 57600`.
`uv run radio_settings.py` reads the radio's settings without Mission Planner. The
logger records raw sensor values. The dashboard applies the airspeed and battery
calibration, so calibration changes go in `dashboard.py` and also apply to old logs.
The dashboard's `LOG_COLS` must match the CSV header in `telem_v1/main/sdcard.c` and the
row format in `telem_v1/main/main.c`.
