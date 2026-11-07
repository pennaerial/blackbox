# Flight logs

Copy `log_NNN.csv` files from the logger's SD card here to replay them with
`uv run dashboard.py --replay <name>.csv` from `ground_station/`. `--replay` with no
name plays the newest file in this folder.

`t_ms` counts milliseconds since the board booted, and the logger has no clock, so
the files carry no date. Rename each one when you copy it, for example
`2026-11-07_field_flight2.csv`, so the date and flight are not lost.
