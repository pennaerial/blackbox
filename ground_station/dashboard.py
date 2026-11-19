#!/usr/bin/env python3
"""
ESP32 sensor dashboard
  - 3D plane that follows the IMU (complementary filter: accel + gyro)
  - Big readout cards for every channel
  - Scrolling plots for airspeed, altitude, accel, gyro

Run from ground_station/:
       uv run dashboard.py            (auto-detects the port)
       uv run dashboard.py COM5       (or pass the port yourself)
       uv run dashboard.py COM7 --baud 57600
                                      (through the SiK telemetry radio instead of USB)
       uv run dashboard.py --replay log_007.csv [--speed 2]
                                      (play back an SD card log; bare names are looked up in logs/,
                                       and --replay with no name plays the newest log there)

Without uv: pip install pyserial pyqtgraph PySide6 PyOpenGL numpy, then python dashboard.py

Expects the ESP32 to print one CSV line per sample (the same `line` that goes to the SD card):
  t_ms,alt_m,press_hpa,bme_temp_c,hum_pct,dp_raw_pa,pitot_temp_c,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,
  batt_v_sig_mv,batt_a_sig_mv,batt_v_adc,batt_a_adc
Other lines (ESP-IDF log messages) are ignored. Older logs, which the ESP32 calibrated itself
(16 columns, or 14 without battery), still replay.

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
import sys
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

# If the plane tilts the wrong way, flip the sign of that axis here (roll, pitch, yaw).
SIGN = (1, 1, 1)


def remap(x, y, z):
    """Map sensor axes to plane axes (x = nose, y = left wing, z = up).
    Change this if the board is mounted rotated, e.g. return y, -x, z"""
    return x, y, z


# ======================= Calibration =======================
# The ESP32 logs raw values; these turn them into airspeed and battery readings. They were
# moved here from airspeed.c and power.c, so old logs and new logs use the same numbers.

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
# Older firmware calibrated on the ESP32 and logged these (the first 14 before the battery).
OLD_COLS = ["t_ms", "alt_m", "press_hpa", "bme_temp_c", "hum_pct", "dp_pa", "pitot_temp_c",
            "airspeed_ms", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "batt_v", "batt_a"]
# Every column the dashboard keeps: calibrated values first, then the raw ones.
COLS = OLD_COLS + [c for c in LOG_COLS if c not in OLD_COLS]
I = {n: i for i, n in enumerate(COLS)}
# Formats are told apart by column count (17, 16 or 14).
FORMATS = {len(cols): [I[c] for c in cols] for cols in (LOG_COLS, OLD_COLS, OLD_COLS[:14])}


# ======================= Helpers =======================
def parse(line: bytes):
    """One CSV row as a list in COLS order; columns the log does not have are nan."""
    parts = line.decode("ascii", "ignore").strip().split(",")
    idx = FORMATS.get(len(parts))
    if idx is None:
        return None
    try:
        vals = [float(p) for p in parts]
    except ValueError:
        return None
    row = [math.nan] * len(COLS)
    for i, v in zip(idx, vals):
        row[i] = v
    return row


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
    order. Rows from older logs have no raw columns and keep the values the ESP32 logged."""

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
        if not math.isfinite(raw):     # old log, or no reading this row
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


# ======================= Main window =======================
class Dashboard(QtWidgets.QMainWindow):
    def __init__(self, port, replay=None, baud=BAUD):
        super().__init__()
        self.setWindowTitle("ESP32 Sensor Dashboard" + (" - replay" if replay else ""))
        self.port = port
        self.replay = replay
        self.baud = baud
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
        outer.addLayout(bar)
        self.build_path_tab(btn_css)

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
        super().closeEvent(e)

    # ---------- Data ----------
    def tick(self):
        if self.replay is not None:
            self.handle_rows(self.replay.due())
        else:
            self.poll_serial()
        self.render()

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
        a = remap(r[I["ax_g"]], r[I["ay_g"]], r[I["az_g"]])
        g = remap(r[I["gx_dps"]], r[I["gy_dps"]], r[I["gz_dps"]])
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
        if self.replay is not None:
            msg, receiving = self.replay.status(), not self.replay.done()
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
    args = ap.parse_args()

    replay = None
    if args.replay is not None:
        replay = Replay(find_log(args.replay), args.speed)
        if not replay.rows:
            sys.exit(f"No data rows in {replay.name}")

    pg.setConfigOptions(antialias=True)
    # Two 3D views (Live and 3D path) need one shared GL context, or their shaders fail to draw.
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
    app = pg.mkQApp("ESP32 Dashboard")
    win = Dashboard(None if replay else find_port(args.port), replay, args.baud)
    win.resize(1500, 900)
    win.show()
    pg.exec()


if __name__ == "__main__":
    main()
