#!/usr/bin/env python3
"""
Read a SiK telemetry radio's settings over its USB serial port, without Mission Planner.

Run from ground_station/ with the radio plugged in by USB:
       uv run radio_settings.py            (auto-detects the port)
       uv run radio_settings.py COM7

Read-only: enters AT command mode with "+++", prints the firmware version (ATI) and
every parameter (ATI5), then leaves command mode (ATO). It does not change settings.
Close the dashboard first; only one program can hold the port.
"""
import argparse
import sys
import time

import serial
import serial.tools.list_ports

# SiK SERIAL_SPEED options, most likely first.
BAUDS = (57600, 115200, 9600, 19200, 38400, 4800, 2400, 1200)
GUARD_S = 1.2   # SiK needs ~1 s with no serial data on each side of "+++"


def find_port(arg):
    if arg:
        return arg
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        text = " ".join(filter(None, [p.description, p.manufacturer])).lower()
        if "ftdi" in text or "usb serial" in text:
            return p.device
    if len(ports) == 1:
        return ports[0].device
    sys.exit("Could not pick the radio's port; pass it, e.g.  uv run radio_settings.py COM7")


def reply(ser, wait):
    time.sleep(wait)
    return ser.read(ser.in_waiting or 1).decode("ascii", "replace")


def command(ser, cmd, wait=0.6):
    ser.reset_input_buffer()
    ser.write(cmd.encode("ascii") + b"\r")
    text = reply(ser, wait)
    return "\n".join(ln for ln in text.replace("\r", "\n").split("\n")
                     if ln.strip() and ln.strip() != cmd)   # drop the radio's echo


def enter_command_mode(ser):
    time.sleep(GUARD_S)
    ser.reset_input_buffer()
    ser.write(b"+++")
    return "OK" in reply(ser, GUARD_S + 0.3)


def main():
    ap = argparse.ArgumentParser(description="Read a SiK radio's settings (read-only).")
    ap.add_argument("port", nargs="?", help="serial port, e.g. COM7 (auto-detected if omitted)")
    port = find_port(ap.parse_args().port)

    for baud in BAUDS:
        print(f"Trying {port} at {baud} baud ...")
        try:
            ser = serial.Serial(port, baud, timeout=0)
        except serial.SerialException as e:
            sys.exit(f"Cannot open {port}: {e} (is the dashboard or another program using it?)")
        with ser:
            if not enter_command_mode(ser):
                continue
            print(f"\nRadio answered at {baud} baud (its SERIAL_SPEED).\n")
            print("Firmware:", command(ser, "ATI"))
            print("\nParameters (ATI5):")
            print(command(ser, "ATI5", wait=1.0))
            command(ser, "ATO")   # back to data mode
            return
    sys.exit("No reply to +++ at any baud. Check the port, and that nothing else is open on it.")


if __name__ == "__main__":
    main()
