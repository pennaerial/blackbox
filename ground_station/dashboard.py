#!/usr/bin/env python3
"""
ESP32 sensor dashboard
  - 3D plane that follows the IMU (complementary filter: accel + gyro)
  - Big readout cards for every channel
  - Scrolling plots for airspeed, altitude, accel, gyro

Run from ground_station/:
       uv run dashboard.py --wifi     (field setup: join the logger's "blackbox" WiFi first. Adds
                                       start-new-log, IMU calibration and WiFi-off controls)
       uv run dashboard.py            (USB: auto-detects the port)
       uv run dashboard.py COM5       (or pass the port yourself)
       uv run dashboard.py COM7 --baud 57600
                                      (through the SiK telemetry radio instead of USB)
       uv run dashboard.py --replay log_007.csv [--speed 2] [--cal cal_002.txt]
                                      (play back an SD card log; bare names are looked up in logs/,
                                       and --replay with no name plays the newest log there.
                                       --cal draws the aircraft using that IMU mounting calibration)

The logger does not log at power-on. Over WiFi, "Start new log" closes the open log and starts the
next log_NNN.csv; press it again to throw away a false start. Closing this window or losing the
WiFi link leaves the logger's WiFi on; only "Turn WiFi off" turns it off, until a power cycle.

Without uv: pip install pyserial pyqtgraph PySide6 PyOpenGL numpy, then python dashboard.py

Expects the ESP32 to print one CSV line per sample (the same `line` that goes to the SD card):
  t_ms,alt_m,press_hpa,bme_temp_c,hum_pct,dp_raw_pa,pitot_temp_c,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,
  batt_v_sig_mv,batt_a_sig_mv,batt_v_adc,batt_a_adc
Other lines (ESP-IDF log messages) are ignored, and so are logs from firmware before the
raw-logging change (16 or 14 columns, calibrated on the ESP32).

The logger records raw values; this file does the calibration (settings below):
  - airspeed: zero from the first rows (pitot open, no draft), low-pass filter, slow auto-zero
    on the ground, wind-tunnel table, deadband and air density -> dp_pa and airspeed_ms.
    Live, the zero is taken when the dashboard connects, so connect before flight or press A
    with the plane still.
  - battery: pin signal mV -> batt_v and batt_a.

Keys: Z = zero attitude, Y = zero yaw only, G = re-calibrate gyro (hold still),
      A = re-zero airspeed (pitot open, no draft), R = restart replay, C = clear the 3D path

The "3D path" tab estimates the flight path without GPS: airspeed along the gyro heading for
x/y, baro altitude for z. Heading drifts and wind is ignored, so treat it as a sketch.
"""
import argparse
import math
import queue
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np
import serial
import serial.tools.list_ports
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtWidgets

# ======================= Settings =======================
PORT = None            # None = auto-detect (or pass as command-line argument)
BAUD = 115200          # USB console baud (ESP-IDF default); the radio uses 57600 via --baud
LOG_DIR = Path(__file__).parent / "logs"   # where --replay looks for SD card logs
HISTORY_S = 20         # seconds of history shown in the plots
PLOT_AXIS_W = 64       # y-axis width (px) shared by all plots so their time axes line up
BUF_N = 3000           # samples kept in memory
FRAME_MS = 16          # UI refresh (~60 fps)

TAU = 0.5              # complementary filter time constant (s): higher = trust gyro longer
ACC_GATE = 0.3         # ignore accel for tilt correction when |a| is more than this far from 1 g
GYRO_CAL_SAMPLES = 100  # samples averaged at start for gyro bias (board must be still)
STILL_DPS = 2.0         # board counts as "still" when every gyro axis is within this of the bias
BIAS_ALPHA = 0.01       # how fast the bias estimate tracks while still (per sample)

PATH_GRID_M = 400       # size of the ground grid in the 3D path tab (m)
PATH_STEP_M = 0.5       # store a path point after moving this far (m)
PATH_MAX_POINTS = 20000  # halve the stored path when it grows past this
PATH_PLANE_SCALE = 2.0  # size of the plane marker on the path (about 7 m wingspan)

# WiFi link to the logger (see telem_v1/main/wifilink.h)
WIFI_HOST = "192.168.4.1"   # the logger's address on its own network
WIFI_PORT = 3333            # WIFI_LINK_PORT
WIFI_SSID = "blackbox"      # WIFI_LINK_SSID, shown as a hint while not connected
LINK_SILENT_S = 5.0         # reconnect if nothing arrives for this long (status comes every 1 s)
STATUS_STALE_S = 3.0        # logger status older than this is shown as unknown

# IMU mounting calibration checks (the wizard warns; it does not block)
CAL_STILL_GYRO_SD = 1.0     # deg/s: more spread than this in a pose means the aircraft moved
CAL_STILL_ACC_SD = 0.02     # g
CAL_MIN_PITCH_DEG = 15.0    # the nose-up pose must differ from level by at least this

# If the plane tilts the wrong way, flip the sign of that axis here (roll, pitch, yaw).
SIGN = (1, 1, 1)


def remap(x, y, z):
    """Map sensor axes to plane axes (x = nose, y = left wing, z = up).
    Change this if the board is mounted rotated, e.g. return y, -x, z"""
    return x, y, z


# ======================= Calibration =======================
# The ESP32 logs raw values; these turn them into airspeed and battery readings. They were
# moved here from airspeed.c and power.c, so a change also applies to logs already recorded.

# Airspeed (MS4525DO + pitot)
AIR_ZERO_ROWS = 40       # rows averaged for the zero (~2 s at 20 Hz); pitot open, out of any draft
AIR_ZERO_MAX_PA = 300.0  # only readings this close to 0 Pa count as no-wind
AIR_ZERO_GIVE_UP = 200   # rows to wait for AIR_ZERO_ROWS good ones before zeroing with what there is
AIR_ZERO_NOISY_PA = 1.5  # warn when the zero is noisier than this (rms): draft, or tube touched
AIR_FILTER = 0.25        # IIR on the zeroed dp: higher = faster but noisier
AIR_DEADBAND_PA = 1.5    # subtracted from the calibrated dp before the sqrt (noise floor)
AIR_USE_CAL = True       # apply the wind-tunnel calibration table
AIR_AUTOZERO = True      # slowly re-zero whenever the reading has been quiet for 10 s
AIR_QUIET_PA = 4.0
AIR_QUIET_MS = 10000
AIR_AUTOZERO_GAIN = 0.002
SEA_LEVEL_PA = 101325.0  # used for air density when the BME680 pressure is missing

# PennAiR wind-tunnel sweep: measured dp -> tunnel reference dp, in Pa. The probe + tubing
# under-reads (about 19% at speed, much more at low speed). Redo the sweep and replace both
# tables if the probe, tubing or mounting changes.
CAL_MEAS = [0.00, 7.47, 14.70, 23.41, 32.38, 51.31, 93.16, 117.07,
            136.25, 158.67, 183.33, 200.52, 223.18, 239.87, 264.53, 301.15]
CAL_TRUE = [0.00, 12.45, 24.91, 37.36, 49.82, 74.73, 124.54, 149.45,
            174.36, 199.27, 224.18, 249.09, 274.00, 298.91, 323.82, 373.63]
CAL_HIGH_GAIN = 1.24     # reference/measured ratio above the last table point

# Battery power module. Calibration was measured on the ESP32-C6 bench wiring; re-check it
# on the ESP32.
VOLTAGE_DIVIDER_RATIO = 10.11
# Current re-fit on the ESP32 against an inline power meter (log_018 + bench video,
# 2026-11-14): 2.443 V signal at the 48 A full-throttle plateau. Old value was 36.364.
CURRENT_AMPS_PER_VOLT = 19.65
# Idle sits at 142 mV (ADC floor); 95% of idle samples in log_018 were <= ~150 mV.
# Signals at or below this read 0 A, so real currents under ~3 A also read 0.
CURRENT_FLOOR_MV = 150

# Columns the logger writes now (must match sdcard.c and the row format in main.c).
LOG_COLS = ["t_ms", "alt_m", "press_hpa", "bme_temp_c", "hum_pct", "dp_raw_pa", "pitot_temp_c",
            "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
            "batt_v_sig_mv", "batt_a_sig_mv", "batt_v_adc", "batt_a_adc"]
# Filled in by Calibration from the raw columns.
CALIBRATED_COLS = ["dp_pa", "airspeed_ms", "batt_v", "batt_a"]
COLS = LOG_COLS + CALIBRATED_COLS
I = {n: i for i, n in enumerate(COLS)}


# ======================= Helpers =======================
def parse(line: bytes):
    """One CSV row as a list in COLS order, calibrated columns still nan; None if not a row."""
    parts = line.decode("ascii", "ignore").strip().split(",")
    if len(parts) != len(LOG_COLS):
        return None
    try:
        vals = [float(p) for p in parts]
    except ValueError:
        return None
    return vals + [math.nan] * len(CALIBRATED_COLS)


def air_cal(m):
    """Wind-tunnel calibration: measured dp -> reference dp (Pa)."""
    if m <= 0.0:
        return m * (CAL_TRUE[1] / CAL_MEAS[1])      # keep the sign for noise below zero
    for i in range(1, len(CAL_MEAS)):
        if m <= CAL_MEAS[i]:
            f = (m - CAL_MEAS[i - 1]) / (CAL_MEAS[i] - CAL_MEAS[i - 1])
            return CAL_TRUE[i - 1] + f * (CAL_TRUE[i] - CAL_TRUE[i - 1])
    return CAL_TRUE[-1] + CAL_HIGH_GAIN * (m - CAL_MEAS[-1])


class Calibration:
    """Fills dp_pa, airspeed_ms, batt_v and batt_a from the raw columns, row by row in time
    order."""

    def __init__(self):
        self.reset()

    def reset(self):
        """Start again from an airspeed zero (new log, reboot, or A key)."""
        self.zero = None
        self.zero_buf = []
        self.zero_tries = 0
        self.filt = 0.0
        self.quiet_since = None
        self.note = ""

    @property
    def zeroing(self):
        return self.zero is None and self.zero_tries > 0

    def apply(self, r):
        self.power(r)
        self.airspeed(r)

    def power(self, r):
        v_mv, a_mv = r[I["batt_v_sig_mv"]], r[I["batt_a_sig_mv"]]
        if math.isfinite(v_mv):
            r[I["batt_v"]] = v_mv / 1000.0 * VOLTAGE_DIVIDER_RATIO
        if math.isfinite(a_mv):
            r[I["batt_a"]] = 0.0 if a_mv <= CURRENT_FLOOR_MV else a_mv / 1000.0 * CURRENT_AMPS_PER_VOLT

    def airspeed(self, r):
        raw = r[I["dp_raw_pa"]]
        if not math.isfinite(raw):     # no reading this row
            return
        if self.zero is None and not self.take_zero(raw):
            return                     # dp_pa and airspeed_ms stay nan while zeroing
        self.filt += AIR_FILTER * ((raw - self.zero) - self.filt)

        if AIR_AUTOZERO:
            # Within a few Pa of zero for 10 s counts as no wind; nudging the offset then
            # cancels warm-up and temperature drift on the ground.
            t = r[I["t_ms"]]
            if abs(self.filt) < AIR_QUIET_PA:
                if self.quiet_since is None:
                    self.quiet_since = t
                if t - self.quiet_since > AIR_QUIET_MS:
                    self.zero += AIR_AUTOZERO_GAIN * self.filt
            else:
                self.quiet_since = None

        press, temp = r[I["press_hpa"]], r[I["pitot_temp_c"]]
        p_pa = press * 100.0 if math.isfinite(press) else SEA_LEVEL_PA
        rho = p_pa / (287.058 * (temp + 273.15))
        dp_c = air_cal(self.filt) if AIR_USE_CAL else self.filt
        eff = dp_c - AIR_DEADBAND_PA       # soft deadband: no jump at the edge
        r[I["airspeed_ms"]] = math.sqrt(2.0 * eff / rho) if eff > 0.0 else 0.0
        r[I["dp_pa"]] = self.filt

    def take_zero(self, raw):
        """Collect one zero reading; True once the zero is set."""
        self.zero_tries += 1
        if abs(raw) < AIR_ZERO_MAX_PA:
            self.zero_buf.append(raw)
        if len(self.zero_buf) < AIR_ZERO_ROWS and self.zero_tries < AIR_ZERO_GIVE_UP:
            return False
        if len(self.zero_buf) >= 20:
            z = np.array(self.zero_buf)
            self.zero, sd = float(z.mean()), float(z.std())
            self.note = f"airspeed zero {self.zero:.2f} Pa, noise {sd:.2f} Pa rms"
            if sd > AIR_ZERO_NOISY_PA:
                self.note += " - NOISY: draft, or tube touched? Press A to re-zero"
        else:
            self.zero = 0.0
            self.note = "airspeed NOT zeroed (no still readings) - press A with the pitot still"
        print(self.note)
        self.filt = 0.0
        return True


# ======================= IMU mounting calibration =======================
# The logger saves the raw accelerometer/gyro means of two still poses (cal_NNN.txt); the
# rotation from IMU axes to aircraft axes is worked out here.
BODY_POS = ("nose", "left wing", "up")      # aircraft axes as in remap(): x, y, z
BODY_NEG = ("tail", "right wing", "down")


def unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def pose_angle_deg(a, b):
    """Angle between two accelerometer readings, i.e. how far the aircraft was tilted."""
    return math.degrees(math.acos(float(np.clip(unit(a).dot(unit(b)), -1.0, 1.0))))


def mount_rotation(level_acc, pitch_acc):
    """Rotation R with aircraft = R @ imu (x = nose, y = left wing, z = up, as in remap()).
    At rest the accelerometer reads +1 g straight up, so the level pose gives the up axis.
    Raising the nose tips that reading toward the nose, so the part of the nose-up reading
    that is not along 'up' gives the nose axis. The angle itself does not matter."""
    z = unit(level_acc)
    p = np.asarray(pitch_acc, dtype=float)
    x = unit(p - p.dot(z) * z)
    return np.vstack([x, np.cross(z, x), z])


def describe_mount(R):
    """Where each IMU axis points on the aircraft, e.g. 'IMU +x -> nose (4 deg off)'."""
    lines = []
    for i, name in enumerate("xyz"):
        col = R[:, i]                       # IMU axis i, in aircraft axes
        j = int(np.argmax(np.abs(col)))
        where = BODY_POS[j] if col[j] > 0 else BODY_NEG[j]
        off = math.degrees(math.acos(min(1.0, abs(float(col[j])))))
        lines.append(f"IMU +{name} -> {where} ({off:.0f} deg off)")
    return "\n".join(lines)


def read_cal_file(path):
    """A cal_NNN.txt from the logger as {key: [numbers]} (text values stay strings)."""
    vals = {}
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        key, *rest = line.split(",")
        try:
            vals[key] = [float(x) for x in rest]
        except ValueError:
            vals[key] = rest
    return vals


def rotation_from_cal_file(path):
    v = read_cal_file(path)
    return mount_rotation(v["level_acc_g"], v["pitch_acc_g"])


# ======================= WiFi link =======================
class WifiLink:
    """TCP link to the logger over its own WiFi network. A background thread connects,
    reconnects after a drop and queues received lines; the GUI thread calls lines() and send().
    Losing the link or closing the dashboard leaves the logger's WiFi on."""

    def __init__(self, host, port=WIFI_PORT):
        self.host, self.port = host, port
        self.rx = queue.Queue()
        self.lock = threading.Lock()
        self.sock = None
        self.stopped = False
        threading.Thread(target=self.run, daemon=True).start()

    @property
    def connected(self):
        return self.sock is not None

    def run(self):
        while not self.stopped:
            try:
                s = socket.create_connection((self.host, self.port), timeout=2.0)
            except OSError:
                time.sleep(1.0)
                continue
            s.settimeout(1.0)
            with self.lock:
                self.sock = s
            self.read_until_closed(s)
            with self.lock:
                self.sock = None
            try:
                s.close()
            except OSError:
                pass

    def read_until_closed(self, s):
        buf, last = b"", time.time()
        while not self.stopped:
            try:
                data = s.recv(4096)
            except socket.timeout:
                if time.time() - last > LINK_SILENT_S:
                    return              # silent link (walked out of range): reconnect
                continue
            except OSError:
                return
            if not data:
                return
            last = time.time()
            *lines, buf = (buf + data).split(b"\n")
            for line in lines:
                self.rx.put(line)
            if len(buf) > 10000:        # never a real line: drop garbage
                buf = b""

    def lines(self):
        out = []
        while True:
            try:
                out.append(self.rx.get_nowait())
            except queue.Empty:
                return out

    def send(self, text):
        """Send one command line; False if not connected."""
        with self.lock:
            if self.sock is None:
                return False
            try:
                self.sock.sendall((text + "\n").encode("ascii"))
                return True
            except OSError:
                return False

    def stop(self):
        """Stop reconnecting (after WIFI,OFF). The logger closes the connection itself."""
        self.stopped = True

    def close(self):
        """Disconnect without turning the logger's WiFi off (dashboard closing)."""
        self.stopped = True
        with self.lock:
            if self.sock is not None:
                try:
                    self.sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass


def find_port(arg=None):
    if arg:
        return arg
    if PORT:
        return PORT
    ports = list(serial.tools.list_ports.comports())
    keys = ("cp210", "ch340", "ch910", "usb serial", "uart", "silicon labs", "esp")
    for p in ports:
        text = " ".join(filter(None, [p.description, p.manufacturer])).lower()
        if any(k in text for k in keys):
            return p.device
    if len(ports) == 1:
        return ports[0].device
    print("Could not pick a serial port automatically. Available ports:")
    for p in ports:
        print(f"  {p.device}  -  {p.description}")
    print("Run again with the port as an argument, e.g.  uv run dashboard.py COM5")
    sys.exit(1)


def find_log(name):
    """A path as given, else the name inside LOG_DIR; no name means the newest log there."""
    if not name:
        logs = sorted(LOG_DIR.glob("*.csv"), key=lambda p: p.stat().st_mtime)
        if not logs:
            sys.exit(f"No .csv logs in {LOG_DIR}")
        return logs[-1]
    path = Path(name)
    if not path.exists() and (LOG_DIR / path).exists():
        path = LOG_DIR / path
    if not path.exists():
        sys.exit(f"Log not found: {name} (also looked in {LOG_DIR})")
    return path


class Replay:
    """Plays an SD card log back in real time (times `speed`), paced by its t_ms column."""

    def __init__(self, path, speed=1.0):
        with open(path, "rb") as f:
            self.rows = [r for r in map(parse, f) if r is not None]   # header and junk drop out
        self.name = Path(path).name
        self.speed = speed
        self.restart()

    def restart(self):
        self.i = 0
        self.start = time.time()

    def due(self):
        """Rows whose time has come since the last call."""
        t = self.rows[0][0] + (time.time() - self.start) * self.speed * 1000.0
        j = self.i
        while j < len(self.rows) and self.rows[j][0] <= t:
            j += 1
        rows, self.i = self.rows[self.i:j], j
        return rows

    def done(self):
        return self.i >= len(self.rows)

    def status(self):
        t0 = self.rows[0][0]
        now = self.rows[max(self.i - 1, 0)][0] - t0
        end = self.rows[-1][0] - t0
        msg = f"Replay {self.name}  x{self.speed:g}   |   {now / 1000:.1f} / {end / 1000:.1f} s"
        return msg + ("   |   finished (R to restart)" if self.done() else "")


def wrap_pi(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


class Attitude:
    """Roll/pitch/yaw (radians) from a 6-axis IMU. Yaw has no reference, so it drifts."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.roll = self.pitch = self.yaw = 0.0
        self.ready = False

    def update(self, a, g, dt, use_gyro=True, freeze_yaw=False):
        ax, ay, az = a
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.hypot(ay, az))

        if not self.ready or not use_gyro:
            self.roll, self.pitch = roll_acc, pitch_acc
            if not self.ready:
                self.yaw = 0.0
            self.ready = True
            return

        p, q, r = (math.radians(v) for v in g)
        sr, cr = math.sin(self.roll), math.cos(self.roll)
        cp = math.cos(self.pitch)
        cp = cp if abs(cp) > 0.05 else (0.05 if cp >= 0 else -0.05)
        tp = math.sin(self.pitch) / cp

        roll_g = self.roll + (p + (q * sr + r * cr) * tp) * dt
        pitch_g = self.pitch + (q * cr - r * sr) * dt
        if not freeze_yaw:
            self.yaw = wrap_pi(self.yaw + ((q * sr + r * cr) / cp) * dt)

        amag = math.sqrt(ax * ax + ay * ay + az * az)
        if abs(amag - 1.0) < ACC_GATE:
            k = dt / (TAU + dt)
            roll_g += k * wrap_pi(roll_acc - roll_g)
            pitch_g += k * wrap_pi(pitch_acc - pitch_g)
        self.roll, self.pitch = wrap_pi(roll_g), wrap_pi(pitch_g)


def box(center, size, color):
    cx, cy, cz = center
    sx, sy, sz = (s / 2 for s in size)
    v = np.array([[cx + dx * sx, cy + dy * sy, cz + dz * sz]
                  for dx in (-1, 1) for dy in (-1, 1) for dz in (-1, 1)], dtype=float)
    f = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                  [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    return v, f, np.tile(color, (12, 1))


def build_plane():
    parts = [
        box((0, 0, 0), (3.2, 0.35, 0.35), (0.85, 0.88, 0.92, 1)),       # fuselage
        box((1.75, 0, 0), (0.3, 0.25, 0.25), (0.95, 0.25, 0.25, 1)),    # nose
        box((0.3, 0, 0), (0.9, 3.6, 0.07), (0.30, 0.60, 0.95, 1)),      # wing
        box((-1.45, 0, 0.05), (0.55, 1.3, 0.06), (0.30, 0.60, 0.95, 1)),  # h-tail
        box((-1.45, 0, 0.35), (0.55, 0.06, 0.5), (0.95, 0.50, 0.25, 1)),  # fin
    ]
    verts, faces, cols, off = [], [], [], 0
    for v, f, c in parts:
        verts.append(v)
        faces.append(f + off)
        cols.append(c)
        off += len(v)
    md = gl.MeshData(vertexes=np.vstack(verts), faces=np.vstack(faces), faceColors=np.vstack(cols))
    return gl.GLMeshItem(meshdata=md, smooth=False, shader="shaded", glOptions="opaque")


class Card(QtWidgets.QFrame):
    def __init__(self, title, unit="", color="#4fc3f7", big=26):
        super().__init__()
        self.setObjectName("card")
        self.setStyleSheet("#card{background:#1b1f27;border-radius:8px;}")
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(12, 6, 12, 6)
        lay.setSpacing(0)
        head = QtWidgets.QLabel(f"{title.upper()}  {unit}".strip())
        head.setStyleSheet("color:#8b93a1;font-size:10px;")
        self.v = QtWidgets.QLabel("--")
        self.v.setStyleSheet(f"color:{color};font-size:{big}px;font-weight:600;"
                             "font-family:Consolas,Menlo,monospace;")
        self.sub = QtWidgets.QLabel("")
        self.sub.setStyleSheet("color:#6c7482;font-size:10px;")
        lay.addWidget(head)
        lay.addWidget(self.v)
        lay.addWidget(self.sub)

    def set(self, value, fmt="{:.2f}", sub=""):
        self.v.setText(fmt.format(value) if math.isfinite(value) else "--")
        self.sub.setText(sub)


# ======================= IMU calibration wizard =======================
CAL_INTRO = (
    "This measures how the IMU sits in the aircraft, so its readings can be turned into "
    "aircraft axes after the flight. The IMU can be mounted at any angle.\n\n"
    "You need the aircraft powered on, about a minute, and a ruler.\n"
    "  1. Hold the aircraft level and still.\n"
    "  2. Raise the nose and hold it still.\n"
    "  3. Enter where the IMU is relative to the centre of gravity (CG).\n\n"
    "The logger saves the result on its SD card as cal_NNN.txt. A log that is recording keeps "
    "recording during calibration.")
CAL_LEVEL = (
    "Hold the aircraft level: fuselage horizontal and wings level, as in straight and level "
    "flight. Keep it completely still, press Measure, and hold it there for about 3 seconds.")
CAL_PITCH = (
    "Raise the nose about 30-45 degrees and keep the wings level. Resting the tail on the "
    "ground is fine, and the angle does not need to be exact, but it must be nose UP, not "
    "down. Keep it still, press Measure, and hold it there for about 3 seconds.")
CAL_OFFSET = (
    "Measure where the IMU is relative to the aircraft's CG, along the aircraft's axes. "
    "A ruler measurement to about half a centimetre is enough. Use 0 for an axis where "
    "the IMU is level with the CG.")


class CalWizard(QtWidgets.QDialog):
    """Step-by-step IMU mounting calibration. The logger averages the IMU in each pose and
    saves the result; this window guides the steps and checks the measurements."""
    REPLY_TIMEOUT_MS = 8000

    def __init__(self, dash):
        super().__init__(dash)
        self.dash = dash
        self.setWindowTitle("IMU mounting calibration")
        self.setMinimumWidth(560)
        self.setStyleSheet("QDialog{background:#1b1f27;} QLabel{color:#c7ccd6;font-size:13px;}")
        self.poses = {}          # "LEVEL"/"PITCH" -> {"acc", "gyro", "acc_sd", "gyro_sd"}
        self.waiting = None      # reply awaited: "LEVEL", "PITCH" or "SAVE"
        self.timer = QtCore.QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.no_reply)

        lay = QtWidgets.QVBoxLayout(self)
        self.stack = QtWidgets.QStackedWidget()
        lay.addWidget(self.stack, 1)
        self.stack.addWidget(self.page("Calibrate the IMU mounting", CAL_INTRO)[0])
        self.measure_btn, self.result = {}, {}
        for pose, title, text in (("LEVEL", "Step 1 of 3: hold the aircraft level", CAL_LEVEL),
                                  ("PITCH", "Step 2 of 3: raise the nose", CAL_PITCH)):
            page, box = self.page(title, text)
            btn = QtWidgets.QPushButton("Measure")
            btn.clicked.connect(lambda _=False, p=pose: self.measure(p))
            res = QtWidgets.QLabel("")
            res.setWordWrap(True)
            box.addWidget(btn)
            box.addWidget(res)
            box.addStretch(1)
            self.measure_btn[pose], self.result[pose] = btn, res
            self.stack.addWidget(page)
        self.stack.addWidget(self.offset_page())
        page, box = self.page("Calibration saved", "")
        self.done_label = box.itemAt(1).widget()
        self.stack.addWidget(page)

        nav = QtWidgets.QHBoxLayout()
        self.back_btn = QtWidgets.QPushButton("Back")
        self.next_btn = QtWidgets.QPushButton("Next")
        close_btn = QtWidgets.QPushButton("Close")
        self.back_btn.clicked.connect(lambda: self.go(self.stack.currentIndex() - 1))
        self.next_btn.clicked.connect(self.next_clicked)
        close_btn.clicked.connect(self.close)
        nav.addWidget(close_btn)
        nav.addStretch(1)
        nav.addWidget(self.back_btn)
        nav.addWidget(self.next_btn)
        lay.addLayout(nav)
        self.update_nav()

    def page(self, title, text):
        page = QtWidgets.QWidget()
        box = QtWidgets.QVBoxLayout(page)
        head = QtWidgets.QLabel(title)
        head.setStyleSheet("color:#e6e9ef;font-size:17px;font-weight:600;")
        body = QtWidgets.QLabel(text)
        body.setWordWrap(True)
        box.addWidget(head)
        box.addWidget(body)
        return page, box

    def offset_page(self):
        page, box = self.page("Step 3 of 3: IMU position from the CG", CAL_OFFSET)
        form = QtWidgets.QFormLayout()
        self.offset = []
        for axis, hint in (("x", "forward of the CG (+), behind it (-)"),
                           ("y", "toward the left wing (+), toward the right (-)"),
                           ("z", "above the CG (+), below it (-)")):
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(-200.0, 200.0)
            spin.setDecimals(1)
            spin.setSingleStep(0.5)
            spin.setSuffix(" cm")
            spin.valueChanged.connect(self.update_distance)
            label = QtWidgets.QLabel(f"{axis}: {hint}")
            form.addRow(label, spin)
            self.offset.append(spin)
        box.addLayout(form)
        self.distance = QtWidgets.QLabel("")
        self.mount = QtWidgets.QLabel("")
        self.mount.setStyleSheet("color:#8b93a1;font-family:Consolas,Menlo,monospace;")
        self.save_status = QtWidgets.QLabel("")
        self.save_status.setWordWrap(True)
        for w in (self.distance, self.mount, self.save_status):
            box.addWidget(w)
        box.addStretch(1)
        self.update_distance()
        return page

    # ---------- steps ----------
    def go(self, i):
        self.stack.setCurrentIndex(max(0, min(i, self.stack.count() - 1)))
        if self.stack.currentIndex() == 3:
            R = self.rotation()
            self.mount.setText("IMU mounting found:\n" + describe_mount(R) if R is not None else "")
        self.update_nav()

    def next_clicked(self):
        i = self.stack.currentIndex()
        if i == 3:
            self.save()
        elif i == self.stack.count() - 1:
            self.close()
        else:
            self.go(i + 1)

    def update_nav(self):
        i, busy = self.stack.currentIndex(), self.waiting is not None
        last = self.stack.count() - 1
        self.back_btn.setVisible(0 < i < last)
        self.back_btn.setEnabled(not busy)
        self.next_btn.setText({3: "Save to SD card", last: "Done"}.get(i, "Next"))
        ok = {1: "LEVEL" in self.poses, 2: self.pitch_ok(), 3: self.rotation() is not None}
        self.next_btn.setEnabled(not busy and ok.get(i, True))
        for btn in self.measure_btn.values():
            btn.setEnabled(not busy)

    def measure(self, pose):
        if not self.dash.link.send(f"CAL,{pose}"):
            self.result[pose].setText("Not connected to the logger.")
            return
        self.poses.pop(pose, None)
        self.wait_for(pose)
        self.result[pose].setText("Asking the logger to measure...")

    def save(self):
        x, y, z = (s.value() for s in self.offset)
        if not self.dash.link.send(f"CAL,SAVE,{x:.2f},{y:.2f},{z:.2f}"):
            self.save_status.setText("Not connected to the logger.")
            return
        self.wait_for("SAVE")
        self.save_status.setText("Saving on the logger...")

    def wait_for(self, what):
        self.waiting = what
        self.timer.start(self.REPLY_TIMEOUT_MS)
        self.update_nav()

    def no_reply(self):
        label = self.save_status if self.waiting == "SAVE" else self.result.get(self.waiting)
        self.waiting = None
        if label is not None:
            label.setText("No answer from the logger. Check the WiFi connection and try again.")
        self.update_nav()

    # ---------- replies from the logger ----------
    def on_message(self, parts):
        """parts: a '#' line from the logger without the '#', split on commas."""
        kind = parts[0]
        if kind == "CAL" and len(parts) >= 3 and parts[1] in self.result:
            pose = parts[1]
            if parts[2] == "MEASURING":
                self.result[pose].setText("Measuring... keep it still (about 3 s).")
                self.timer.start(self.REPLY_TIMEOUT_MS)
            elif parts[2] == "DONE" and len(parts) >= 16:
                v = [float(p) for p in parts[4:16]]
                self.poses[pose] = {"acc": v[0:3], "gyro": v[3:6], "acc_sd": v[6:9],
                                    "gyro_sd": v[9:12]}
                self.finish_wait()
                self.show_pose(pose)
        elif kind == "CALSAVED" and len(parts) >= 2:
            self.finish_wait()
            R = self.rotation()
            self.done_label.setText(
                f"Saved as {parts[1]} on the logger's SD card.\n\n"
                f"IMU mounting:\n{describe_mount(R)}\n\n"
                "The dashboard now draws the aircraft using this calibration. For post-flight "
                f"analysis, copy {parts[1]} off the card along with the logs.")
            self.dash.apply_rotation(R)
            self.go(self.stack.count() - 1)
        elif kind == "ERR":
            what = self.waiting
            self.finish_wait()
            label = self.save_status if what == "SAVE" else self.result.get(what, self.save_status)
            label.setText("Logger: " + ",".join(parts[2:]))

    def finish_wait(self):
        self.waiting = None
        self.timer.stop()
        self.update_nav()

    # ---------- checks ----------
    def still_note(self, p):
        g, a = max(p["gyro_sd"]), max(p["acc_sd"])
        if g > CAL_STILL_GYRO_SD or a > CAL_STILL_ACC_SD:
            return (f"\nWARNING: it moved during the measurement (gyro spread {g:.1f} deg/s, "
                    f"accel spread {a:.3f} g). Hold it steadier and measure again.")
        return "\nHeld still: yes."

    def show_pose(self, pose):
        p = self.poses[pose]
        mag = float(np.linalg.norm(p["acc"]))
        text = "Done."
        if abs(mag - 1.0) > 0.1:
            text += f"\nWARNING: the accelerometer reads {mag:.2f} g at rest, expected about 1 g."
        if pose == "LEVEL":
            tilt = pose_angle_deg(p["acc"], (0, 0, 1))
            text += f"\nThe IMU's z axis is {tilt:.0f} deg from vertical."
            if "PITCH" in self.poses:
                self.show_pitch_check()
        else:
            text += self.pitch_text()
        self.result[pose].setText(text + self.still_note(p))
        self.update_nav()

    def show_pitch_check(self):
        p = self.poses["PITCH"]
        self.result["PITCH"].setText("Done." + self.pitch_text() + self.still_note(p))

    def pitch_text(self):
        if "LEVEL" not in self.poses:
            return "\nMeasure the level pose (step 1) first."
        angle = pose_angle_deg(self.poses["LEVEL"]["acc"], self.poses["PITCH"]["acc"])
        if angle < CAL_MIN_PITCH_DEG:
            return (f"\nThe nose is only {angle:.0f} deg from level. Raise it more (30-45 deg) "
                    "and measure again.")
        return f"\nNose raised about {angle:.0f} deg from level."

    def pitch_ok(self):
        return ("LEVEL" in self.poses and "PITCH" in self.poses and
                pose_angle_deg(self.poses["LEVEL"]["acc"], self.poses["PITCH"]["acc"])
                >= CAL_MIN_PITCH_DEG)

    def rotation(self):
        if not self.pitch_ok():
            return None
        return mount_rotation(self.poses["LEVEL"]["acc"], self.poses["PITCH"]["acc"])

    def update_distance(self):
        x, y, z = (s.value() for s in self.offset)
        self.distance.setText(f"Distance from the CG: {math.sqrt(x * x + y * y + z * z):.1f} cm")


# ======================= Main window =======================
class Dashboard(QtWidgets.QMainWindow):
    def __init__(self, port, replay=None, baud=BAUD, link=None, R=None):
        super().__init__()
        self.setWindowTitle("ESP32 Sensor Dashboard" + (" - replay" if replay else "")
                            + (" - WiFi" if link else ""))
        self.port = port
        self.replay = replay
        self.baud = baud
        self.link = link                    # WifiLink, or None for serial / replay
        self.R = R                          # IMU -> aircraft rotation from a calibration, or None
        self.logger = None                  # last #STATUS from the logger
        self.logger_t = 0.0
        self.logger_note = ""
        self.logger_shown = None
        self.wizard = None
        self.ser = None
        self.rx = bytearray()
        self.buf = np.full((BUF_N, len(COLS)), np.nan)
        self.n = 0
        self.last_t = None
        self.latest = None
        self.last_rx = 0.0
        self.last_open_try = 0.0
        self.rate_count, self.rate_t, self.rate = 0, time.time(), 0.0
        self.att = Attitude()
        self.cal = Calibration()
        self.off = [0.0, 0.0, 0.0]          # degrees, raw (before SIGN)
        self.cal_buf, self.gyro_bias, self.calibrating = [], np.zeros(3), True
        self.still = False
        self.reset_path()
        self.build_ui()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(FRAME_MS)

    # ---------- UI ----------
    def build_ui(self):
        root = QtWidgets.QWidget()
        root.setObjectName("root")
        root.setStyleSheet("#root{background:#0e1116;}")
        self.setCentralWidget(root)
        outer = QtWidgets.QVBoxLayout(root)
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setStyleSheet(
            "QTabWidget::pane{border:none;}"
            "QTabBar::tab{background:#1b1f27;color:#8b93a1;padding:6px 16px;border-radius:6px;"
            "margin-right:4px;}QTabBar::tab:selected{background:#2a3142;color:#e6e9ef;}")
        outer.addWidget(self.tabs, 1)
        live = QtWidgets.QWidget()
        live_lay = QtWidgets.QVBoxLayout(live)
        live_lay.setContentsMargins(0, 6, 0, 0)
        self.tabs.addTab(live, "Live")

        # top strip
        self.c_air = Card("Airspeed", "m/s", "#4fc3f7", big=40)
        self.c_alt = Card("Altitude", "m", "#81e6a0", big=34)
        self.c_press = Card("Pressure", "hPa", "#e0c36a")
        self.c_temp = Card("Baro temp", "C", "#ff9d6e")
        self.c_hum = Card("Humidity", "%", "#7fd6e6")
        self.c_ptemp = Card("Pitot temp", "C", "#ff9d6e")
        self.c_dp = Card("Diff pressure", "Pa", "#c59bff")
        self.c_g = Card("|Accel|", "g", "#ff7b8a")
        self.c_batt_v = Card("Battery", "V", "#f2e86d")
        self.c_batt_a = Card("Current", "A", "#f2a65a")
        top = QtWidgets.QHBoxLayout()
        for c, s in ((self.c_air, 3), (self.c_alt, 3), (self.c_press, 2), (self.c_temp, 2),
                     (self.c_hum, 2), (self.c_ptemp, 2), (self.c_dp, 2), (self.c_g, 2),
                     (self.c_batt_v, 2), (self.c_batt_a, 2)):
            top.addWidget(c, s)
        live_lay.addLayout(top)

        mid = QtWidgets.QHBoxLayout()
        live_lay.addLayout(mid, 1)

        # left: 3D + attitude + raw IMU
        left = QtWidgets.QVBoxLayout()
        self.view = gl.GLViewWidget()
        self.view.setMinimumSize(420, 340)
        self.view.setBackgroundColor("#12151b")
        self.view.setCameraPosition(distance=10, elevation=20, azimuth=-150)
        grid = gl.GLGridItem()
        grid.setSize(8, 8)
        grid.setSpacing(1, 1)
        grid.translate(0, 0, -1.5)
        self.view.addItem(grid)
        self.plane = build_plane()
        self.view.addItem(self.plane)
        left.addWidget(self.view, 1)

        self.c_roll = Card("Roll", "deg", "#ffd166")
        self.c_pitch = Card("Pitch", "deg", "#ffd166")
        self.c_yaw = Card("Yaw", "deg", "#ffd166")
        self.c_ax = Card("Accel X", "g", "#ff5d5d", big=20)
        self.c_ay = Card("Accel Y", "g", "#5dff8a", big=20)
        self.c_az = Card("Accel Z", "g", "#5da8ff", big=20)
        self.c_gx = Card("Gyro X", "dps", "#ff5d5d", big=20)
        self.c_gy = Card("Gyro Y", "dps", "#5dff8a", big=20)
        self.c_gz = Card("Gyro Z", "dps", "#5da8ff", big=20)
        for cards in ((self.c_roll, self.c_pitch, self.c_yaw),
                      (self.c_ax, self.c_ay, self.c_az),
                      (self.c_gx, self.c_gy, self.c_gz)):
            row = QtWidgets.QHBoxLayout()
            for c in cards:
                row.addWidget(c)
            left.addLayout(row)
        mid.addLayout(left, 5)

        # right: plots
        pg.setConfigOption("background", "#12151b")
        pg.setConfigOption("foreground", "#9aa3b2")
        gw = pg.GraphicsLayoutWidget()
        mid.addWidget(gw, 6)
        self.curves = {}
        first = None

        def add_plot(title, unit, series):
            nonlocal first
            p = gw.addPlot(title=title)
            p.showGrid(x=True, y=True, alpha=0.2)
            p.setLabel("left", unit)
            # Same y-axis width on every plot so the linked time axes line up at any window size;
            # no SI prefix, so altitude stays in m rather than "m (x0.001)".
            left_axis = p.getAxis("left")
            left_axis.setWidth(PLOT_AXIS_W)
            left_axis.enableAutoSIPrefix(False)
            p.getAxis("bottom").setTickSpacing(5, 1)   # labels every 5 s on every plot
            p.setXRange(-HISTORY_S, 0, padding=0)
            p.disableAutoRange(pg.ViewBox.XAxis)
            if first is None:
                first = p
            else:
                p.setXLink(first)
            if len(series) > 1:
                p.addLegend(offset=(5, 5))
            for col, name, color in series:
                self.curves[col] = p.plot(pen=pg.mkPen(color, width=2), name=name, connect="finite")
            gw.nextRow()

        add_plot("Airspeed", "m/s", [("airspeed_ms", "airspeed", "#4fc3f7")])
        add_plot("Altitude (relative)", "m", [("alt_m", "alt", "#81e6a0")])
        add_plot("Acceleration", "g", [("ax_g", "x", "#ff5d5d"), ("ay_g", "y", "#5dff8a"),
                                       ("az_g", "z", "#5da8ff")])
        add_plot("Gyro", "deg/s", [("gx_dps", "x", "#ff5d5d"), ("gy_dps", "y", "#5dff8a"),
                                   ("gz_dps", "z", "#5da8ff")])

        # bottom bar
        bar = QtWidgets.QHBoxLayout()
        btn_css = ("QPushButton{background:#1f2430;color:#c7ccd6;border:1px solid #2c3342;"
                   "border-radius:6px;padding:6px 12px;}QPushButton:hover{background:#2a3142;}")
        for text, fn in (("Zero attitude (Z)", self.zero_attitude),
                         ("Zero yaw (Y)", self.zero_yaw),
                         ("Re-cal gyro (G)", self.start_cal),
                         ("Re-zero airspeed (A)", self.cal.reset)):
            b = QtWidgets.QPushButton(text)
            b.setStyleSheet(btn_css)
            b.clicked.connect(fn)
            bar.addWidget(b)
        bar.addStretch(1)
        self.status = QtWidgets.QLabel("")
        self.status.setStyleSheet("color:#8b93a1;font-size:12px;")
        bar.addWidget(self.status)
        if self.link is not None:
            outer.addLayout(self.build_logger_bar(btn_css))
        outer.addLayout(bar)
        self.build_path_tab(btn_css)

    def build_logger_bar(self, btn_css):
        """WiFi mode: logger state and its controls."""
        bar = QtWidgets.QHBoxLayout()
        self.rec = QtWidgets.QLabel("")
        self.sensors = QtWidgets.QLabel("")
        self.sensors.setStyleSheet("color:#8b93a1;font-size:12px;")
        self.note = QtWidgets.QLabel("")
        self.note.setStyleSheet("color:#ffb347;font-size:12px;")
        for w in (self.rec, self.sensors, self.note):
            bar.addWidget(w)
            bar.addSpacing(16)
        bar.addStretch(1)
        self.logger_btns = []
        for text, fn in (("Start new log", self.start_log),
                         ("Calibrate IMU...", self.open_wizard),
                         ("Turn WiFi off...", self.wifi_off)):
            b = QtWidgets.QPushButton(text)
            b.setStyleSheet(btn_css)
            b.clicked.connect(fn)
            bar.addWidget(b)
            self.logger_btns.append(b)
        return bar

    def build_path_tab(self, btn_css):
        """Estimated 3D flight path: dead reckoning from airspeed + heading, baro altitude."""
        tab = QtWidgets.QWidget()
        lay = QtWidgets.QHBoxLayout(tab)
        lay.setContentsMargins(0, 6, 0, 0)
        self.tabs.addTab(tab, "3D path")

        self.path_view = gl.GLViewWidget()
        self.path_view.setBackgroundColor("#12151b")
        self.path_view.setCameraPosition(distance=PATH_GRID_M, elevation=30, azimuth=-135)
        grid = gl.GLGridItem()
        grid.setSize(PATH_GRID_M, PATH_GRID_M)
        grid.setSpacing(PATH_GRID_M / 20, PATH_GRID_M / 20)
        self.path_view.addItem(grid)
        self.path_line = gl.GLLinePlotItem(pos=np.zeros((1, 3)), color=(0.31, 0.76, 0.97, 1),
                                           width=2, antialias=True)
        self.path_view.addItem(self.path_line)
        self.path_drop = gl.GLLinePlotItem(pos=np.zeros((2, 3)), color=(0.55, 0.58, 0.63, 0.6),
                                           width=1, antialias=True)   # marker down to the ground
        self.path_view.addItem(self.path_drop)
        self.path_plane = build_plane()
        self.path_view.addItem(self.path_plane)
        lay.addWidget(self.path_view, 1)

        side = QtWidgets.QVBoxLayout()
        self.c_dist = Card("Distance flown", "m", "#4fc3f7")
        self.c_home = Card("From start", "m", "#81e6a0")
        self.c_path_alt = Card("Altitude", "m", "#81e6a0")
        for c in (self.c_dist, self.c_home, self.c_path_alt):
            side.addWidget(c)
        self.follow = QtWidgets.QCheckBox("Follow the plane")
        self.follow.setChecked(True)
        self.follow.setStyleSheet("color:#c7ccd6;")
        side.addWidget(self.follow)
        for text, fn in (("Fit whole path", self.fit_path), ("Clear path (C)", self.reset_path)):
            b = QtWidgets.QPushButton(text)
            b.setStyleSheet(btn_css)
            b.clicked.connect(fn)
            side.addWidget(b)
        note = QtWidgets.QLabel(
            "Estimated, not GPS: airspeed along the gyro heading, with baro altitude. "
            "Heading drifts and wind is not included, so the track wanders over time. "
            "Without airspeed the path only moves up and down.")
        note.setWordWrap(True)
        note.setStyleSheet("color:#6c7482;font-size:11px;")
        side.addWidget(note)
        side.addStretch(1)
        panel = QtWidgets.QWidget()
        panel.setFixedWidth(240)
        panel.setLayout(side)
        lay.addWidget(panel)

    # ---------- Actions ----------
    def start_log(self):
        """Every press closes the open log and starts the next log_NNN.csv."""
        self.logger_note = "" if self.link.send("LOG") else "Not connected"

    def open_wizard(self):
        if self.wizard is None or not self.wizard.isVisible():
            self.wizard = CalWizard(self)
        self.wizard.show()
        self.wizard.raise_()

    def wifi_off(self):
        recording = self.logger_fresh() and self.logger["log"]
        text = ("Turn the logger's WiFi off to save power?\n\nYou cannot start a new log or "
                "calibrate again until the logger is power-cycled. A log that is recording "
                "keeps recording.")
        if not recording:
            text += "\n\nWARNING: no log is recording. Start one first unless you mean this."
        buttons = QtWidgets.QMessageBox.StandardButton
        if QtWidgets.QMessageBox.question(self, "Turn WiFi off", text, buttons.Yes | buttons.No,
                                          buttons.No) != buttons.Yes:
            return
        if self.link.send("WIFI,OFF"):
            self.link.stop()
            self.logger_note = "WiFi turned off: power-cycle the logger to reconnect"
        else:
            self.logger_note = "Not connected, so WiFi was not turned off"

    def apply_rotation(self, R):
        """Draw the aircraft in aircraft axes from now on (after a calibration)."""
        self.R = R
        self.off = [0.0, 0.0, 0.0]
        self.start_cal()                    # the gyro bias was in the old axes

    def to_body(self, x, y, z):
        if self.R is not None:
            return tuple(float(v) for v in self.R @ (x, y, z))
        return remap(x, y, z)

    def logger_fresh(self):
        return self.logger is not None and time.time() - self.logger_t < STATUS_STALE_S

    def zero_attitude(self):
        self.off = [math.degrees(self.att.roll), math.degrees(self.att.pitch), math.degrees(self.att.yaw)]

    def zero_yaw(self):
        self.off[2] = math.degrees(self.att.yaw)

    def start_cal(self):
        self.calibrating = True
        self.cal_buf = []
        self.gyro_bias = np.zeros(3)
        self.att.reset()

    def reset_state(self):
        self.buf[:] = np.nan
        self.n = 0
        self.last_t = None
        self.start_cal()
        self.cal.reset()
        self.reset_path()

    def reset_path(self):
        self.pos = np.zeros(3)              # metres: x, y from the start point, z = baro altitude
        self.path = [self.pos.copy()]
        self.path_dist = 0.0
        self.path_dirty = True

    def fit_path(self):
        p = np.array(self.path)
        lo, hi = p.min(axis=0), p.max(axis=0)
        size = float(np.max(hi - lo))
        self.follow.setChecked(False)
        self.path_view.setCameraPosition(pos=pg.Vector(*((lo + hi) / 2)),
                                         distance=max(size * 1.6, 20.0))

    def keyPressEvent(self, e):
        k = e.text().lower()
        if k == "z":
            self.zero_attitude()
        elif k == "y":
            self.zero_yaw()
        elif k == "g":
            self.start_cal()
        elif k == "a":
            self.cal.reset()
        elif k == "c":
            self.reset_path()
        elif k == "r" and self.replay is not None:
            self.reset_state()
            self.replay.restart()
        else:
            super().keyPressEvent(e)

    def closeEvent(self, e):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        if self.link is not None:
            self.link.close()               # the logger's WiFi stays on
        super().closeEvent(e)

    # ---------- Data ----------
    def tick(self):
        if self.replay is not None:
            self.handle_rows(self.replay.due())
        elif self.link is not None:
            self.poll_wifi()
        else:
            self.poll_serial()
        self.render()

    def poll_wifi(self):
        rows = []
        for line in self.link.lines():
            text = line.decode("ascii", "ignore").strip()
            if text.startswith("#"):
                self.handle_logger_message(text[1:].split(","))
            else:
                r = parse(line)
                if r is not None:
                    rows.append(r)
        self.handle_rows(rows)

    def handle_logger_message(self, parts):
        """A '#' line from the logger, without the '#', split on commas."""
        kind = parts[0]
        if kind == "STATUS" and len(parts) >= 8:
            on = [p == "1" for p in parts[3:8]]
            self.logger = {"log": None if parts[2] == "-" else parts[2], "sd": on[0],
                           "sensors": dict(zip(("IMU", "airspeed", "baro", "power"), on[1:]))}
            self.logger_t = time.time()
        elif kind in ("CAL", "CALSAVED") or parts[:2] == ["ERR", "CAL"]:
            if self.wizard is not None:
                self.wizard.on_message(parts)
        elif kind == "LOG" and len(parts) >= 2:
            self.logger_note = f"Started {parts[1]}"
            if self.logger is not None:
                self.logger["log"] = parts[1]
        elif kind == "ERR":
            self.logger_note = "Logger: " + ",".join(parts[1:])
        elif kind == "WIFI":
            self.logger_note = "WiFi turned off: power-cycle the logger to reconnect"

    def poll_serial(self):
        now = time.time()
        if self.ser is None:
            if now - self.last_open_try > 1.0:
                self.last_open_try = now
                try:
                    self.ser = serial.Serial(self.port, self.baud, timeout=0)
                    self.rx.clear()
                except (serial.SerialException, OSError):
                    self.ser = None
            return
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
        except (serial.SerialException, OSError):
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            return
        if not data:
            return
        self.rx += data
        if len(self.rx) > 50000:          # safety: never let garbage pile up
            del self.rx[:-2000]
        *lines, rest = self.rx.split(b"\n")
        self.rx = bytearray(rest)
        self.handle_rows(r for r in map(parse, lines) if r is not None)

    def handle_rows(self, rows):
        batch = []
        for row in rows:
            if self.last_t is not None and row[0] < self.last_t:   # ESP32 rebooted
                self.push(batch)
                batch = []
                self.reset_state()
            self.cal.apply(row)    # raw -> airspeed and battery, before anything reads them
            self.process(row)
            batch.append(row)
            self.last_t = row[0]
        self.push(batch)

    def process(self, r):
        self.latest = r
        self.last_rx = time.time()
        self.rate_count += 1
        a = self.to_body(r[I["ax_g"]], r[I["ay_g"]], r[I["az_g"]])
        g = self.to_body(r[I["gx_dps"]], r[I["gy_dps"]], r[I["gz_dps"]])
        if not all(math.isfinite(v) for v in (*a, *g)):
            return
        dt = 0.02 if self.last_t is None else (r[0] - self.last_t) / 1000.0
        dt = min(max(dt, 1e-4), 0.2)
        if self.calibrating:
            self.cal_buf.append(g)
            self.att.update(a, g, dt, use_gyro=False)
            if len(self.cal_buf) >= GYRO_CAL_SAMPLES:
                self.gyro_bias = np.mean(self.cal_buf, axis=0)
                self.calibrating = False
                self.att.yaw = 0.0
        else:
            amag = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
            still = (all(abs(g[i] - self.gyro_bias[i]) < STILL_DPS for i in range(3))
                     and abs(amag - 1.0) < 0.05)
            if still:   # keep refining the bias while the board sits still
                self.gyro_bias = self.gyro_bias + BIAS_ALPHA * (np.array(g) - self.gyro_bias)
            self.still = still
            g = tuple(g[i] - self.gyro_bias[i] for i in range(3))
            self.att.update(a, g, dt, freeze_yaw=still)
        self.advance_path(r, dt)

    def angles(self):
        """Roll, pitch, yaw in degrees as drawn: zero offsets and SIGN applied."""
        return (SIGN[0] * (math.degrees(self.att.roll) - self.off[0]),
                SIGN[1] * (math.degrees(self.att.pitch) - self.off[1]),
                SIGN[2] * (math.degrees(self.att.yaw) - self.off[2]))

    def advance_path(self, r, dt):
        """Dead reckoning: move along the nose direction at airspeed; height from the baro."""
        _, pitch, yaw = (math.radians(v) for v in self.angles())
        v = r[I["airspeed_ms"]]
        v = v if math.isfinite(v) else 0.0
        # Nose direction of the drawn plane, Rz(yaw) * Ry(pitch) * x.
        nose = np.array([math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw),
                         -math.sin(pitch)])
        step = v * dt * nose
        new = self.pos + step
        alt = r[I["alt_m"]]
        if math.isfinite(alt):
            new[2] = alt
        self.path_dist += float(np.linalg.norm(step[:2]))
        self.pos = new
        if np.linalg.norm(new - self.path[-1]) >= PATH_STEP_M:
            self.path.append(new.copy())
            if len(self.path) > PATH_MAX_POINTS:
                self.path = self.path[::2]
        self.path_dirty = True

    def push(self, batch):
        if not batch:
            return
        new = np.array(batch)
        k = len(new)
        if k >= BUF_N:
            self.buf = new[-BUF_N:].copy()
        else:
            self.buf = np.roll(self.buf, -k, axis=0)
            self.buf[-k:] = new
        self.n = min(self.n + k, BUF_N)

    # ---------- Drawing ----------
    def render(self):
        now = time.time()
        if now - self.rate_t >= 1.0:
            self.rate = self.rate_count / (now - self.rate_t)
            self.rate_count, self.rate_t = 0, now
        if self.link is not None:
            self.render_logger_bar()
        if self.replay is not None:
            msg, receiving = self.replay.status(), not self.replay.done()
        elif self.link is not None:
            if self.link.stopped:
                msg, receiving = "WiFi off", False
            elif not self.link.connected:
                msg, receiving = f"Connecting to {self.link.host}:{self.link.port} ...", False
            elif now - self.last_rx > 1.0:
                msg, receiving = f"{self.link.host}: connected, no data", False
            else:
                msg, receiving = f"WiFi {self.link.host}   |   {self.rate:.0f} Hz", True
        elif self.ser is None:
            msg, receiving = f"Waiting for {self.port} ...", False
        elif now - self.last_rx > 1.0:
            msg, receiving = f"{self.port}: connected, no data", False
        else:
            msg, receiving = f"{self.port} @ {self.baud}   |   {self.rate:.0f} Hz", True
        if receiving:
            if self.cal.zeroing:
                msg += "   |   zeroing airspeed - pitot open, no draft"
            if self.calibrating:
                msg += "   |   calibrating gyro - keep still"
            elif self.still:
                msg += "   |   still (yaw held, bias tracking)"
        if self.cal.note:
            msg += f"   |   {self.cal.note}"
        self.status.setText(msg)

        r = self.latest
        if r is None:
            return
        v = r[I["airspeed_ms"]]
        sub = f"{v * 3.6:.0f} km/h  |  {v * 2.237:.0f} mph" if math.isfinite(v) else ""
        self.c_air.set(v, "{:.1f}", sub)
        self.c_alt.set(r[I["alt_m"]], "{:.2f}")
        self.c_press.set(r[I["press_hpa"]], "{:.1f}")
        self.c_temp.set(r[I["bme_temp_c"]], "{:.1f}")
        self.c_hum.set(r[I["hum_pct"]], "{:.0f}")
        self.c_ptemp.set(r[I["pitot_temp_c"]], "{:.1f}")
        raw_dp, v_mv, a_mv = r[I["dp_raw_pa"]], r[I["batt_v_sig_mv"]], r[I["batt_a_sig_mv"]]
        self.c_dp.set(r[I["dp_pa"]], "{:.1f}", f"raw {raw_dp:.1f} Pa" if math.isfinite(raw_dp) else "")
        ax, ay, az = r[I["ax_g"]], r[I["ay_g"]], r[I["az_g"]]
        self.c_g.set(math.sqrt(ax * ax + ay * ay + az * az), "{:.2f}")
        self.c_batt_v.set(r[I["batt_v"]], "{:.2f}", f"signal {v_mv:.0f} mV" if math.isfinite(v_mv) else "")
        self.c_batt_a.set(r[I["batt_a"]], "{:.1f}", f"signal {a_mv:.0f} mV" if math.isfinite(a_mv) else "")
        for c, col, f in ((self.c_ax, "ax_g", "{:+.3f}"), (self.c_ay, "ay_g", "{:+.3f}"),
                          (self.c_az, "az_g", "{:+.3f}"), (self.c_gx, "gx_dps", "{:+.1f}"),
                          (self.c_gy, "gy_dps", "{:+.1f}"), (self.c_gz, "gz_dps", "{:+.1f}")):
            c.set(r[I[col]], f)

        roll, pitch, yaw = self.angles()
        self.c_roll.set(roll, "{:+.1f}")
        self.c_pitch.set(pitch, "{:+.1f}")
        self.c_yaw.set(yaw, "{:+.1f}")

        # M = Rz(yaw) * Ry(pitch) * Rx(roll)
        self.plane.resetTransform()
        self.plane.rotate(yaw, 0, 0, 1, local=True)
        self.plane.rotate(pitch, 0, 1, 0, local=True)
        self.plane.rotate(roll, 1, 0, 0, local=True)

        if self.n > 1:
            d = self.buf[-self.n:]
            x = (d[:, 0] - d[-1, 0]) / 1000.0
            for col, curve in self.curves.items():
                curve.setData(x, d[:, I[col]])

        if self.tabs.currentIndex() == 1:
            self.render_path(roll, pitch, yaw)

    def render_logger_bar(self):
        link, fresh = self.link, self.logger_fresh()
        if link.stopped:
            rec, color = "WiFi OFF", "#8b93a1"
        elif not link.connected:
            rec, color = f'Not connected: join WiFi "{WIFI_SSID}"', "#8b93a1"
        elif not fresh:
            rec, color = "Connected, waiting for the logger...", "#8b93a1"
        elif not self.logger["sd"]:
            rec, color = "NO SD CARD", "#ff5d5d"
        elif self.logger["log"]:
            rec, color = f"REC  {self.logger['log']}", "#5dff8a"
        else:
            rec, color = "NOT LOGGING", "#ffb347"
        sensors = ("   ".join(f"{n} {'ok' if ok else 'MISSING'}"
                              for n, ok in self.logger["sensors"].items()) if fresh else "")
        usable = link.connected and not link.stopped
        shown = (rec, color, sensors, self.logger_note, usable)
        if shown == self.logger_shown:      # restyling every frame is wasted work
            return
        self.logger_shown = shown
        self.rec.setText(rec)
        self.rec.setStyleSheet(f"color:{color};font-size:15px;font-weight:600;")
        self.sensors.setText(sensors)
        self.note.setText(self.logger_note)
        for b in self.logger_btns:
            b.setEnabled(usable)

    def render_path(self, roll, pitch, yaw):
        if self.path_dirty:
            self.path_dirty = False
            self.path_line.setData(pos=np.array(self.path + [self.pos]))
        x, y, z = self.pos
        self.path_drop.setData(pos=np.array([[x, y, z], [x, y, 0.0]]))
        self.path_plane.resetTransform()
        self.path_plane.translate(x, y, z)
        self.path_plane.rotate(yaw, 0, 0, 1, local=True)
        self.path_plane.rotate(pitch, 0, 1, 0, local=True)
        self.path_plane.rotate(roll, 1, 0, 0, local=True)
        self.path_plane.scale(PATH_PLANE_SCALE, PATH_PLANE_SCALE, PATH_PLANE_SCALE, local=True)
        if self.follow.isChecked():
            self.path_view.setCameraPosition(pos=pg.Vector(x, y, z))
        self.c_dist.set(self.path_dist, "{:.0f}")
        self.c_home.set(math.hypot(x, y), "{:.0f}")
        self.c_path_alt.set(z, "{:.1f}")


def main():
    ap = argparse.ArgumentParser(description="Live or replayed black box dashboard.")
    ap.add_argument("port", nargs="?", help="serial port, e.g. COM5 (auto-detected if omitted)")
    ap.add_argument("--baud", type=int, default=BAUD,
                    help=f"serial baud (default {BAUD} for USB; 57600 for the SiK radio)")
    ap.add_argument("--replay", nargs="?", const="", metavar="CSV",
                    help="play back an SD card log; a bare name is looked up in logs/, "
                         "no name plays the newest log there")
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed multiplier (default 1)")
    ap.add_argument("--wifi", nargs="?", const=WIFI_HOST, metavar="HOST",
                    help=f'connect over the logger\'s WiFi (join "{WIFI_SSID}" first; '
                         f"default address {WIFI_HOST}), with logging and calibration controls")
    ap.add_argument("--cal", metavar="TXT",
                    help="a cal_NNN.txt from the SD card (bare names are looked up in logs/): "
                         "draw the aircraft using that IMU mounting calibration")
    args = ap.parse_args()
    if args.wifi and (args.replay is not None or args.port):
        sys.exit("--wifi cannot be combined with a serial port or --replay")

    R = None
    if args.cal:
        path = Path(args.cal)
        if not path.exists() and (LOG_DIR / path).exists():
            path = LOG_DIR / path
        try:
            R = rotation_from_cal_file(path)
        except (OSError, KeyError, ValueError) as e:
            sys.exit(f"Cannot use calibration {args.cal}: {e}")
        print(f"Using IMU mounting from {path.name}:\n{describe_mount(R)}")

    replay = None
    if args.replay is not None:
        replay = Replay(find_log(args.replay), args.speed)
        if not replay.rows:
            sys.exit(f"No data rows in {replay.name}")

    pg.setConfigOptions(antialias=True)
    # Two 3D views (Live and 3D path) need one shared GL context, or their shaders fail to draw.
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
    app = pg.mkQApp("ESP32 Dashboard")
    if args.wifi:
        win = Dashboard(None, link=WifiLink(args.wifi), R=R)
    else:
        win = Dashboard(None if replay else find_port(args.port), replay, args.baud, R=R)
    win.resize(1500, 900)
    win.show()
    pg.exec()


if __name__ == "__main__":
    main()
