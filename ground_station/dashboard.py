#!/usr/bin/env python3
"""
ESP32 sensor dashboard
  - 3D plane that follows the IMU (complementary filter: accel + gyro)
  - Big readout cards for every channel
  - Scrolling plots for airspeed, altitude, accel, gyro

Run from ground_station/:
       uv run dashboard.py            (auto-detects the port)
       uv run dashboard.py COM5       (or pass the port yourself)

Without uv: pip install pyserial pyqtgraph PySide6 PyOpenGL numpy, then python dashboard.py

Expects the ESP32 to print one CSV line per sample (the same `line` that goes to the SD card):
  t_ms,alt_m,press_hpa,bme_temp_c,hum_pct,dp_pa,pitot_temp_c,airspeed_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps
Other lines (ESP-IDF log messages) are ignored.

Keys: Z = zero attitude, Y = zero yaw only, G = re-calibrate gyro (hold still)
"""
import math
import sys
import time

import numpy as np
import serial
import serial.tools.list_ports
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtWidgets

# ======================= Settings =======================
PORT = None            # None = auto-detect (or pass as command-line argument)
BAUD = 115200          # must match the firmware console baud (ESP-IDF default)
HISTORY_S = 20         # seconds of history shown in the plots
BUF_N = 3000           # samples kept in memory
FRAME_MS = 16          # UI refresh (~60 fps)

TAU = 0.5              # complementary filter time constant (s): higher = trust gyro longer
ACC_GATE = 0.3         # ignore accel for tilt correction when |a| is more than this far from 1 g
GYRO_CAL_SAMPLES = 100  # samples averaged at start for gyro bias (board must be still)
STILL_DPS = 2.0         # board counts as "still" when every gyro axis is within this of the bias
BIAS_ALPHA = 0.01       # how fast the bias estimate tracks while still (per sample)

# If the plane tilts the wrong way, flip the sign of that axis here (roll, pitch, yaw).
SIGN = (1, 1, 1)


def remap(x, y, z):
    """Map sensor axes to plane axes (x = nose, y = left wing, z = up).
    Change this if the board is mounted rotated, e.g. return y, -x, z"""
    return x, y, z


COLS = ["t_ms", "alt_m", "press_hpa", "bme_temp_c", "hum_pct", "dp_pa", "pitot_temp_c",
        "airspeed_ms", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"]
I = {n: i for i, n in enumerate(COLS)}


# ======================= Helpers =======================
def parse(line: bytes):
    parts = line.decode("ascii", "ignore").strip().split(",")
    if len(parts) != len(COLS):
        return None
    try:
        return [float(p) for p in parts]
    except ValueError:
        return None


def find_port():
    if len(sys.argv) > 1:
        return sys.argv[1]
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
    def __init__(self, port):
        super().__init__()
        self.setWindowTitle("ESP32 Sensor Dashboard")
        self.port = port
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
        self.off = [0.0, 0.0, 0.0]          # degrees, raw (before SIGN)
        self.cal_buf, self.gyro_bias, self.calibrating = [], np.zeros(3), True
        self.still = False
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

        # top strip
        self.c_air = Card("Airspeed", "m/s", "#4fc3f7", big=40)
        self.c_alt = Card("Altitude", "m", "#81e6a0", big=34)
        self.c_press = Card("Pressure", "hPa", "#e0c36a")
        self.c_temp = Card("Baro temp", "C", "#ff9d6e")
        self.c_hum = Card("Humidity", "%", "#7fd6e6")
        self.c_ptemp = Card("Pitot temp", "C", "#ff9d6e")
        self.c_dp = Card("Diff pressure", "Pa", "#c59bff")
        self.c_g = Card("|Accel|", "g", "#ff7b8a")
        top = QtWidgets.QHBoxLayout()
        for c, s in ((self.c_air, 3), (self.c_alt, 3), (self.c_press, 2), (self.c_temp, 2),
                     (self.c_hum, 2), (self.c_ptemp, 2), (self.c_dp, 2), (self.c_g, 2)):
            top.addWidget(c, s)
        outer.addLayout(top)

        mid = QtWidgets.QHBoxLayout()
        outer.addLayout(mid, 1)

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
                         ("Re-cal gyro (G)", self.start_cal)):
            b = QtWidgets.QPushButton(text)
            b.setStyleSheet(btn_css)
            b.clicked.connect(fn)
            bar.addWidget(b)
        bar.addStretch(1)
        self.status = QtWidgets.QLabel("")
        self.status.setStyleSheet("color:#8b93a1;font-size:12px;")
        bar.addWidget(self.status)
        outer.addLayout(bar)

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

    def keyPressEvent(self, e):
        k = e.text().lower()
        if k == "z":
            self.zero_attitude()
        elif k == "y":
            self.zero_yaw()
        elif k == "g":
            self.start_cal()
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
        self.poll_serial()
        self.render()

    def poll_serial(self):
        now = time.time()
        if self.ser is None:
            if now - self.last_open_try > 1.0:
                self.last_open_try = now
                try:
                    self.ser = serial.Serial(self.port, BAUD, timeout=0)
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

        batch = []
        for ln in lines:
            row = parse(ln)
            if row is None:
                continue
            if self.last_t is not None and row[0] < self.last_t:   # ESP32 rebooted
                self.push(batch)
                batch = []
                self.reset_state()
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
        if self.ser is None:
            msg = f"Waiting for {self.port} ..."
        elif now - self.last_rx > 1.0:
            msg = f"{self.port}: connected, no data"
        else:
            msg = f"{self.port} @ {BAUD}   |   {self.rate:.0f} Hz"
            if self.calibrating:
                msg += "   |   calibrating gyro - keep still"
            elif self.still:
                msg += "   |   still (yaw held, bias tracking)"
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
        self.c_dp.set(r[I["dp_pa"]], "{:.1f}")
        ax, ay, az = r[I["ax_g"]], r[I["ay_g"]], r[I["az_g"]]
        self.c_g.set(math.sqrt(ax * ax + ay * ay + az * az), "{:.2f}")
        for c, col, f in ((self.c_ax, "ax_g", "{:+.3f}"), (self.c_ay, "ay_g", "{:+.3f}"),
                          (self.c_az, "az_g", "{:+.3f}"), (self.c_gx, "gx_dps", "{:+.1f}"),
                          (self.c_gy, "gy_dps", "{:+.1f}"), (self.c_gz, "gz_dps", "{:+.1f}")):
            c.set(r[I[col]], f)

        roll = SIGN[0] * (math.degrees(self.att.roll) - self.off[0])
        pitch = SIGN[1] * (math.degrees(self.att.pitch) - self.off[1])
        yaw = SIGN[2] * (math.degrees(self.att.yaw) - self.off[2])
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


def main():
    pg.setConfigOptions(antialias=True)
    app = pg.mkQApp("ESP32 Dashboard")
    win = Dashboard(find_port())
    win.resize(1500, 900)
    win.show()
    pg.exec()


if __name__ == "__main__":
    main()
