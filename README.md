# blackbox
Design of black box and live telemetry system for micro and regular class plane in testing and competition use.

See [the firmware guide](telem_v1/README.md) for the native ESP-IDF logger,
device modules, preserved pin assignments, and ESP32 build instructions.

`ground_station/dashboard.py` is a laptop-side live dashboard that reads the
logger's CSV lines over USB serial. From `ground_station/`, run
`uv run dashboard.py` (or pass the port, e.g. `uv run dashboard.py COM5`).
`uv run dashboard.py --replay log_007.csv` plays back an SD card log instead; copy
logs into `ground_station/logs/`. Through the telemetry radio, pass the ground
radio's port and its baud: `uv run dashboard.py COM7 --baud 57600`.
`uv run radio_settings.py` reads the radio's settings without Mission Planner. The
dashboard's column list must match the CSV header in `telem_v1/main/sdcard.c` and the row
format in `telem_v1/main/main.c`.
