import argparse
import csv
import json
import math
import os
import random
import sys
import threading
import subprocess
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox

# Configure Matplotlib before importing its backend/config machinery.
os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(__file__).resolve().parent.parent / ".cache" / "matplotlib"),
)
Path(os.environ["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)
import matplotlib
_headless_cli = "--self-test" in sys.argv or "--benchmark" in sys.argv
matplotlib.use("Agg" if _headless_cli else "TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

try:
    from .imu import parse_imu_line
except ImportError:
    from imu import parse_imu_line

try:
    import serial
except Exception:
    serial = None


# ============================================================
# AIR MOUSE GLOVE — ROBUST EMBEDDED / SOFTWARE SIMULATION
# ============================================================

APP_DIR = Path(__file__).resolve().parent.parent
LOG_DIR = APP_DIR / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)
# Keep tool caches inside the project so headless runs also work when the
# user's global configuration directory is read-only.
os.environ.setdefault("MPLCONFIGDIR", str(APP_DIR / ".cache" / "matplotlib"))
Path(os.environ["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)

BG = "#eeeeee"
WHITE = "#ffffff"
BLACK = "#111111"
GRAY = "#666666"
MID = "#999999"
LIGHT = "#d5d5d5"
GRID = "#e2e2e2"


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


@dataclass
class SensorSample:
    t: float
    true_roll: float
    true_pitch: float
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    dropped: bool = False


class MotionGenerator:
    """Produces deterministic hand orientation and angular velocity profiles."""

    def __init__(self):
        self.mode = "Slow"

    def state(self, t):
        m = self.mode
        if m == "Stationary":
            roll = 0.0
            pitch = 0.0
        elif m == "Slow":
            roll = math.radians(10.0) * math.sin(t * 0.9)
            pitch = math.radians(8.0) * math.cos(t * 0.7)
        elif m == "Fast":
            roll = math.radians(22.0) * math.sin(t * 2.0)
            pitch = math.radians(18.0) * math.cos(t * 1.6)
        elif m == "Circular":
            roll = math.radians(14.0) * math.sin(t * 1.15)
            pitch = math.radians(14.0) * math.cos(t * 1.15)
        elif m == "Sudden":
            p = t % 6.0
            roll = math.radians(20.0 if 1.0 < p < 2.0 else 0.0)
            pitch = math.radians(-18.0 if 3.0 < p < 4.0 else 0.0)
        elif m == "Tremor":
            roll = math.radians(3.0) * math.sin(t * 1.0) + math.radians(2.7) * math.sin(t * 14.0)
            pitch = math.radians(2.5) * math.cos(t * 0.9) + math.radians(2.2) * math.sin(t * 17.0)
        elif m == "Figure-8":
            roll = math.radians(16.0) * math.sin(t * 1.0)
            pitch = math.radians(11.0) * math.sin(t * 2.0)
        elif m == "Precision":
            roll = math.radians(5.0) * math.sin(t * 0.35)
            pitch = math.radians(4.0) * math.sin(t * 0.25 + 0.7)
        else:
            roll = pitch = 0.0

        dt = 1e-3
        r2, p2 = self._state_only(t + dt)
        gx = math.degrees((r2 - roll) / dt)
        gy = math.degrees((p2 - pitch) / dt)
        gz = 0.0
        return roll, pitch, gx, gy, gz

    def _state_only(self, t):
        m = self.mode
        if m == "Stationary":
            return 0.0, 0.0
        if m == "Slow":
            return math.radians(10.0) * math.sin(t * 0.9), math.radians(8.0) * math.cos(t * 0.7)
        if m == "Fast":
            return math.radians(22.0) * math.sin(t * 2.0), math.radians(18.0) * math.cos(t * 1.6)
        if m == "Circular":
            return math.radians(14.0) * math.sin(t * 1.15), math.radians(14.0) * math.cos(t * 1.15)
        if m == "Sudden":
            p = t % 6.0
            return math.radians(20.0 if 1.0 < p < 2.0 else 0.0), math.radians(-18.0 if 3.0 < p < 4.0 else 0.0)
        if m == "Tremor":
            return math.radians(3.0) * math.sin(t * 1.0) + math.radians(2.7) * math.sin(t * 14.0), math.radians(2.5) * math.cos(t * 0.9) + math.radians(2.2) * math.sin(t * 17.0)
        if m == "Figure-8":
            return math.radians(16.0) * math.sin(t * 1.0), math.radians(11.0) * math.sin(t * 2.0)
        if m == "Precision":
            return math.radians(5.0) * math.sin(t * 0.35), math.radians(4.0) * math.sin(t * 0.25 + 0.7)
        return 0.0, 0.0


class VirtualMPU6050:
    """Models sensor noise, bias, drift, quantization, saturation and packet loss."""

    ACC_LSB_PER_G = 16384.0     # +/-2 g
    GYRO_LSB_PER_DPS = 131.0    # +/-250 deg/s
    ACC_MAX = 2.0
    GYRO_MAX = 250.0

    def __init__(self, motion):
        self.motion = motion
        self.noise = 0.08
        self.accel_noise = 0.012
        self.bias = 0.03
        self.drift = 0.015
        self.sample_jitter_ms = 2.0
        self.packet_loss = 0.0
        self.quantization = True
        self.time = 0.0

    def sample(self, t):
        roll, pitch, gx, gy, gz = self.motion.state(t)

        # Body-frame gravity approximation.
        ax = math.sin(pitch)
        ay = math.sin(roll) * math.cos(pitch)
        az = math.cos(roll) * math.cos(pitch)

        drift_x = self.drift * math.sin(t * 0.07)
        drift_y = self.drift * math.cos(t * 0.05)

        ax += self.bias + drift_x + np.random.normal(0.0, self.accel_noise * max(self.noise, 0.05))
        ay -= self.bias * 0.65 + drift_y + np.random.normal(0.0, self.accel_noise * max(self.noise, 0.05))
        az += np.random.normal(0.0, self.accel_noise * 0.7 * max(self.noise, 0.05))

        gx += self.bias * 8.0 + np.random.normal(0.0, self.noise * 7.0)
        gy -= self.bias * 5.0 + np.random.normal(0.0, self.noise * 7.0)
        gz += np.random.normal(0.0, self.noise * 4.0)

        ax = clamp(ax, -self.ACC_MAX, self.ACC_MAX)
        ay = clamp(ay, -self.ACC_MAX, self.ACC_MAX)
        az = clamp(az, -self.ACC_MAX, self.ACC_MAX)
        gx = clamp(gx, -self.GYRO_MAX, self.GYRO_MAX)
        gy = clamp(gy, -self.GYRO_MAX, self.GYRO_MAX)
        gz = clamp(gz, -self.GYRO_MAX, self.GYRO_MAX)

        if self.quantization:
            ax = round(ax * self.ACC_LSB_PER_G) / self.ACC_LSB_PER_G
            ay = round(ay * self.ACC_LSB_PER_G) / self.ACC_LSB_PER_G
            az = round(az * self.ACC_LSB_PER_G) / self.ACC_LSB_PER_G
            gx = round(gx * self.GYRO_LSB_PER_DPS) / self.GYRO_LSB_PER_DPS
            gy = round(gy * self.GYRO_LSB_PER_DPS) / self.GYRO_LSB_PER_DPS
            gz = round(gz * self.GYRO_LSB_PER_DPS) / self.GYRO_LSB_PER_DPS

        dropped = random.random() < self.packet_loss
        return SensorSample(t, roll, pitch, ax, ay, az, gx, gy, gz, dropped)


class Calibration:
    def __init__(self):
        self.acc_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.active = False
        self.samples = []
        self.target = 50

    def start(self, target=50):
        self.active = True
        self.target = max(20, int(target))
        self.samples.clear()

    def feed(self, sample):
        if not self.active or sample.dropped:
            return False
        self.samples.append(sample)
        if len(self.samples) >= self.target:
            acc = np.array([[s.ax, s.ay, s.az] for s in self.samples], dtype=float)
            gyro = np.array([[s.gx, s.gy, s.gz] for s in self.samples], dtype=float)
            self.acc_bias = np.mean(acc, axis=0) - np.array([0.0, 0.0, 1.0])
            self.gyro_bias = np.mean(gyro, axis=0)
            self.active = False
            return True
        return False


class AdaptiveFusion:
    """Complementary IMU fusion + adaptive EMA stabilization."""

    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.prev_roll = 0.0
        self.prev_pitch = 0.0
        self.noise_estimate = 0.25
        self.mode = "Adaptive"
        self.dead_zone_deg = 0.7
        self.fixed_alpha = 0.22
        self.last_alpha = 0.22
        self.ready = False

    def reset(self):
        self.roll = self.pitch = 0.0
        self.prev_roll = self.prev_pitch = 0.0
        self.noise_estimate = 0.25
        self.ready = False

    def process(self, s, acc_bias, gyro_bias, dt):
        if s.dropped:
            return self.roll, self.pitch, self.last_alpha, True

        ax = s.ax - acc_bias[0]
        ay = s.ay - acc_bias[1]
        az = s.az - acc_bias[2]

        gx = s.gx - gyro_bias[0]
        gy = s.gy - gyro_bias[1]

        acc_roll = math.degrees(math.atan2(ay, max(1e-6, az)))
        acc_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

        if not self.ready:
            self.roll = acc_roll
            self.pitch = acc_pitch
            self.prev_roll = acc_roll
            self.prev_pitch = acc_pitch
            self.ready = True

        gyro_roll = self.roll + gx * dt
        gyro_pitch = self.pitch + gy * dt

        gyro_mag = math.hypot(gx, gy)
        disagreement = math.hypot(acc_roll - gyro_roll, acc_pitch - gyro_pitch)
        self.noise_estimate = 0.94 * self.noise_estimate + 0.06 * disagreement

        if self.mode == "Raw":
            alpha_gyro = 0.0
        elif self.mode == "Fixed":
            alpha_gyro = 0.78
        else:
            # Dynamic movement -> trust gyro more; quiet motion -> trust accelerometer more.
            dynamic = clamp(gyro_mag / 90.0, 0.0, 1.0)
            alpha_gyro = 0.58 + 0.32 * dynamic
            if self.noise_estimate > 2.0:
                alpha_gyro -= 0.08
            alpha_gyro = clamp(alpha_gyro, 0.45, 0.92)

        fused_roll = alpha_gyro * gyro_roll + (1.0 - alpha_gyro) * acc_roll
        fused_pitch = alpha_gyro * gyro_pitch + (1.0 - alpha_gyro) * acc_pitch

        delta = math.hypot(
            fused_roll - self.prev_roll,
            fused_pitch - self.prev_pitch
        )

        if self.mode == "Adaptive":
            if delta < max(self.dead_zone_deg, self.noise_estimate * 0.35):
                ema = 0.08
            elif delta < 3.0:
                ema = 0.18
            elif delta < 7.0:
                ema = 0.30
            else:
                ema = 0.48
        elif self.mode == "Fixed":
            ema = self.fixed_alpha
        else:
            ema = 1.0

        self.roll = ema * fused_roll + (1.0 - ema) * self.prev_roll
        self.pitch = ema * fused_pitch + (1.0 - ema) * self.prev_pitch
        self.prev_roll = self.roll
        self.prev_pitch = self.pitch
        self.last_alpha = ema
        return self.roll, self.pitch, ema, False


class CursorMapper:
    def __init__(self):
        self.x = 50.0
        self.y = 50.0
        self.raw_x = 50.0
        self.raw_y = 50.0
        self.sensitivity = 0.75
        self.dead_zone = 0.8
        self.invert_x = False
        self.invert_y = False

    def reset(self):
        self.x = self.y = self.raw_x = self.raw_y = 50.0

    def update(self, roll, pitch, raw_roll, raw_pitch):
        # Map orientation to an absolute screen position.  Accumulating every
        # frame causes inevitable edge drift; a smooth tanh curve gives useful
        # movement while guaranteeing that the cursor remains bounded.
        def axis(angle):
            if abs(angle) <= self.dead_zone:
                return 0.0
            active = angle - math.copysign(self.dead_zone, angle)
            return math.tanh(active * self.sensitivity / 28.0) * 45.0

        fx = axis(roll)
        fy = axis(pitch)
        rx = math.tanh(raw_roll * self.sensitivity / 32.0) * 45.0
        ry = math.tanh(raw_pitch * self.sensitivity / 32.0) * 45.0

        if self.invert_y:
            fy = -fy
            ry = -ry
        if self.invert_x:
            fx = -fx
            rx = -rx

        self.raw_x = clamp(50.0 + rx, 0.0, 100.0)
        self.raw_y = clamp(50.0 + ry, 0.0, 100.0)
        self.x = clamp(50.0 + fx, 0.0, 100.0)
        self.y = clamp(50.0 + fy, 0.0, 100.0)
        return self.x, self.y, self.raw_x, self.raw_y


class Metrics:
    def __init__(self):
        self.reset()

    def reset(self):
        self.raw_error = deque(maxlen=500)
        self.filtered_error = deque(maxlen=500)
        self.raw_signal = deque(maxlen=40)
        self.filtered_signal = deque(maxlen=40)
        self.raw_jitter = deque(maxlen=250)
        self.filtered_jitter = deque(maxlen=250)
        self.latency = 0.0
        self.truth_available = True

    def update(self, raw_roll, raw_pitch, filt_roll, filt_pitch, true_roll, true_pitch, alpha, sampling, truth=True):
        self.truth_available = truth
        if truth:
            raw_e = math.hypot(raw_roll - true_roll, raw_pitch - true_pitch)
            filt_e = math.hypot(filt_roll - true_roll, filt_pitch - true_pitch)
            self.raw_error.append(raw_e)
            self.filtered_error.append(filt_e)

        self.raw_signal.append(float(raw_roll))
        self.filtered_signal.append(float(filt_roll))
        if len(self.raw_signal) >= 5:
            w = min(9, len(self.raw_signal))
            raw_arr = np.asarray(self.raw_signal, dtype=float)
            filt_arr = np.asarray(self.filtered_signal, dtype=float)
            raw_res = raw_arr[-1] - float(np.mean(raw_arr[-w:]))
            filt_res = filt_arr[-1] - float(np.mean(filt_arr[-w:]))
            self.raw_jitter.append(raw_res)
            self.filtered_jitter.append(filt_res)

        self.latency = 35.0 + (1.0 - alpha) * 500.0 / max(sampling, 1)

    @property
    def raw_rms(self):
        return float(np.sqrt(np.mean(np.square(self.raw_error)))) if self.raw_error else 0.0

    @property
    def filtered_rms(self):
        return float(np.sqrt(np.mean(np.square(self.filtered_error)))) if self.filtered_error else 0.0

    @property
    def raw_jitter_rms(self):
        return float(np.sqrt(np.mean(np.square(self.raw_jitter)))) if self.raw_jitter else 0.0

    @property
    def filtered_jitter_rms(self):
        return float(np.sqrt(np.mean(np.square(self.filtered_jitter)))) if self.filtered_jitter else 0.0

    @property
    def reduction(self):
        if self.raw_jitter_rms <= 1e-9:
            return 0.0
        return clamp((1.0 - self.filtered_jitter_rms / self.raw_jitter_rms) * 100.0, -999.0, 99.9)

    @property
    def stability(self):
        jitter = self.filtered_jitter_rms
        return clamp(100.0 * (1.0 - jitter / 3.0), 0.0, 100.0)


class Recorder:
    def __init__(self):
        self.active = False
        self.fp = None
        self.writer = None
        self.path = None

    def start(self):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = LOG_DIR / f"air_mouse_{stamp}.csv"
        self.fp = self.path.open("w", newline="")
        self.writer = csv.writer(self.fp)
        self.writer.writerow([
            "timestamp", "mode", "ax", "ay", "az", "gx", "gy", "gz",
            "roll", "pitch", "noise_estimate", "alpha", "filtered_x",
            "filtered_y", "cursor_x", "cursor_y"
        ])
        self.active = True

    def write(self, values):
        if self.active and self.writer:
            self.writer.writerow(values)
            self.fp.flush()

    def stop(self):
        self.active = False
        if self.fp:
            self.fp.close()
        self.fp = None
        self.writer = None
        return self.path


class WokwiBridge:
    def __init__(self):
        self.port = "rfc2217://localhost:4000"
        self.ser = None
        self.latest = None
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.status = "Not connected"
        self.remote_left_clicks = 0
        self.remote_right_clicks = 0
        self.last_frame_time = None

    def connect(self):
        if serial is None:
            self.status = "PySerial unavailable"
            return False
        try:
            if self.ser:
                self.ser.close()
            self.ser = serial.serial_for_url(
                self.port,
                baudrate=115200,
                timeout=0.05
            )
            self.running = True
            self.thread = threading.Thread(target=self._reader, daemon=True)
            self.thread.start()
            self.status = "Connected"
            return True
        except Exception as exc:
            self.status = f"Failed: {exc}"
            return False

    def _reader(self):
        while self.running and self.ser:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line.startswith("IMU,"):
                    values = line.split(",")
                    if len(values) == 7:
                        try:
                            data = [float(v) for v in values[1:]]
                        except ValueError:
                            # The firmware emits an all-ERROR frame if the
                            # simulated/physical IMU cannot be read.
                            continue
                        with self.lock:
                            self.latest = data
                            self.last_frame_time = time.time()
                            self.status = "Streaming"
                elif line == "EVENT,LEFT":
                    self.remote_left_clicks += 1
                elif line == "EVENT,RIGHT":
                    self.remote_right_clicks += 1
            except Exception as exc:
                self.status = f"Read failed: {exc}"
                self.running = False
                break

    def read(self):
        with self.lock:
            return None if self.latest is None else list(self.latest)

    def close(self):
        self.running = False
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.status = "Disconnected"
        self.last_frame_time = None

    def consume_clicks(self):
        """Return and clear click events received from the remote firmware."""
        with self.lock:
            clicks = self.remote_left_clicks, self.remote_right_clicks
            self.remote_left_clicks = 0
            self.remote_right_clicks = 0
            return clicks


class AirMouseEngine:
    def __init__(self):
        self.motion = MotionGenerator()
        self.sensor = VirtualMPU6050(self.motion)
        self.cal = Calibration()
        self.fusion = AdaptiveFusion()
        self.cursor = CursorMapper()
        self.metrics = Metrics()
        self.recorder = Recorder()
        self.wokwi = WokwiBridge()

        self.source = "Synthetic"
        self.running = True
        self.t0 = time.perf_counter()
        self.sim_time = 0.0
        self.last_step = time.perf_counter()
        self.sample_count = 0
        self.dropped_count = 0
        self.left_clicks = 0
        self.right_clicks = 0
        self.last_sample = None
        self.last_result = None
        self.events = deque(maxlen=8)
        self.remote_left_clicks = 0
        self.remote_right_clicks = 0

        self.raw_roll_hist = deque(maxlen=300)
        self.filt_roll_hist = deque(maxlen=300)
        self.true_roll_hist = deque(maxlen=300)
        self.gyro_hist = deque(maxlen=300)
        self.path_x = deque(maxlen=300)
        self.path_y = deque(maxlen=300)
        self.path_raw_x = deque(maxlen=300)
        self.path_raw_y = deque(maxlen=300)

    def set_source(self, source):
        self.source = source

    def calibrate(self):
        self.cal.start(50)
        self.log_event("Calibration started — keep sensor still")

    def click(self, side):
        if side == "Left":
            self.left_clicks += 1
        else:
            self.right_clicks += 1
        self.log_event(f"{side.upper()} CLICK event")

    def log_event(self, text):
        self.events.appendleft(time.strftime("%H:%M:%S") + "  " + text)

    def reset(self):
        self.t0 = time.perf_counter()
        self.sim_time = 0.0
        self.sample_count = 0
        self.dropped_count = 0
        self.left_clicks = 0
        self.right_clicks = 0
        self.cal = Calibration()
        self.fusion.reset()
        self.cursor.reset()
        self.metrics.reset()
        for h in (
            self.raw_roll_hist, self.filt_roll_hist, self.true_roll_hist,
            self.gyro_hist, self.path_x, self.path_y,
            self.path_raw_x, self.path_raw_y
        ):
            h.clear()
        self.events.clear()
        self.remote_left_clicks = 0
        self.remote_right_clicks = 0
        self.log_event("System reset")

    def _wokwi_sample(self):
        data = self.wokwi.read()
        if data is None:
            return None
        return SensorSample(
            t=self.sim_time,
            true_roll=0.0,
            true_pitch=0.0,
            ax=data[0], ay=data[1], az=data[2],
            gx=data[3], gy=data[4], gz=data[5]
        )

    def step(self):
        if self.source in ("Wokwi", "Wokwi CLI"):
            sample = self._wokwi_sample()
            if sample is None:
                # Keep the selected source visible while the serial bridge is
                # starting; a synthetic frame keeps the UI responsive.
                sample = self.sensor.sample(self.sim_time)
        else:
            sample = self.sensor.sample(self.sim_time)

        if sample.dropped:
            self.dropped_count += 1

        dt = 1.0 / max(1, self.sensor_sample_rate())
        raw_roll = math.degrees(math.atan2(sample.ay, max(1e-6, sample.az)))
        raw_pitch = math.degrees(math.atan2(-sample.ax, math.sqrt(sample.ay**2 + sample.az**2)))

        self.cal.feed(sample)
        filt_roll, filt_pitch, alpha, dropped = self.fusion.process(
            sample,
            self.cal.acc_bias,
            self.cal.gyro_bias,
            dt
        )

        cx, cy, rx, ry = self.cursor.update(
            filt_roll,
            filt_pitch,
            raw_roll,
            raw_pitch
        )

        if self.source == "Synthetic":
            self.metrics.update(
                raw_roll,
                raw_pitch,
                filt_roll,
                filt_pitch,
                sample.true_roll,
                sample.true_pitch,
                alpha,
                self.sensor_sample_rate()
            )
        else:
            # With real/Wokwi input, use output jitter as the stability measure.
            if self.last_result:
                synthetic_true_roll = self.last_result["filtered_roll"]
                synthetic_true_pitch = self.last_result["filtered_pitch"]
                self.metrics.update(
                    raw_roll, raw_pitch,
                    filt_roll, filt_pitch,
                    synthetic_true_roll, synthetic_true_pitch,
                    alpha, self.sensor_sample_rate(), truth=False
                )

        self.raw_roll_hist.append(raw_roll)
        self.filt_roll_hist.append(filt_roll)
        self.true_roll_hist.append(sample.true_roll)
        self.gyro_hist.append(math.sqrt(sample.gx**2 + sample.gy**2 + sample.gz**2))

        self.path_x.append(cx)
        self.path_y.append(cy)
        self.path_raw_x.append(rx)
        self.path_raw_y.append(ry)

        self.sample_count += 1
        self.sim_time += dt

        if self.recorder.active:
            self.recorder.write([
                sample.t, self.motion.mode, sample.ax, sample.ay, sample.az,
                sample.gx, sample.gy, sample.gz, sample.true_roll,
                sample.true_pitch, self.fusion.noise_estimate, alpha, filt_roll,
                filt_pitch, cx, cy
            ])

        self.last_result = {
            "sample": sample,
            "raw_roll": raw_roll,
            "raw_pitch": raw_pitch,
            "filtered_roll": filt_roll,
            "filtered_pitch": filt_pitch,
            "alpha": alpha,
            "cursor_x": cx,
            "cursor_y": cy,
            "raw_cursor_x": rx,
            "raw_cursor_y": ry,
        }
        remote_left, remote_right = self.wokwi.consume_clicks()
        if remote_left or remote_right:
            self.left_clicks += remote_left
            self.right_clicks += remote_right
            self.log_event(f"REMOTE CLICK  L={remote_left} R={remote_right}")
        return self.last_result

    def sensor_sample_rate(self):
        return self._sampling_rate

    @property
    def sampling_rate(self):
        return self._sampling_rate

    @sampling_rate.setter
    def sampling_rate(self, value):
        self._sampling_rate = clamp(int(value), 20, 100)

    _sampling_rate = 50

    def benchmark(self):
        modes = ["Stationary", "Slow", "Fast", "Tremor", "Sudden"]
        old_mode = self.motion.mode
        old_source = self.source
        old_noise = self.sensor.noise
        self.source = "Synthetic"
        rows = []
        for mode in modes:
            self.reset()
            self.motion.mode = mode
            self.sensor.noise = 0.35
            for _ in range(int(self.sampling_rate * 2.5)):
                self.step()
            rows.append({
                "mode": mode,
                "sample_count": self.sample_count,
                "raw_rms_deg": round(self.metrics.raw_rms, 4),
                "filtered_rms_deg": round(self.metrics.filtered_rms, 4),
                "raw_jitter_deg": round(self.metrics.raw_jitter_rms, 4),
                "filtered_jitter_deg": round(self.metrics.filtered_jitter_rms, 4),
                "reduction_percent": round(self.metrics.reduction, 2),
                "stability_percent": round(self.metrics.stability, 2),
                "latency_ms": round(self.metrics.latency, 3),
            })
        self.motion.mode = old_mode
        self.source = old_source
        self.sensor.noise = old_noise
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = LOG_DIR / "benchmark.json"
        path.write_text(json.dumps(rows, indent=2))
        csv_path = LOG_DIR / "benchmark.csv"
        with csv_path.open("w", newline="") as fp:
            writer = csv.DictWriter(fp, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        self.log_event(f"Benchmark saved: {path.name} / {csv_path.name}")
        return rows, path


class AirMouseApp:
    def __init__(self, root):
        self.root = root
        self.engine = AirMouseEngine()
        self.engine.sampling_rate = 50
        self.engine.motion.mode = "Slow"
        self.running = True
        self.last_ui = time.perf_counter()

        root.title("AI-Assisted Air Mouse Glove — Integrated Simulation")
        root.geometry("1600x1000")
        root.minsize(1320, 820)
        root.configure(bg=BG)

        self._styles()
        self._build()
        self._build_plots()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.tick()

    def _styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background=WHITE)
        style.configure("App.TFrame", background=BG)
        style.configure("Card.TFrame", background=WHITE, relief="solid", borderwidth=1)
        style.configure("Title.TLabel", background=BG, foreground=BLACK, font=("DejaVu Sans", 20, "bold"))
        style.configure("Sub.TLabel", background=BG, foreground=GRAY, font=("DejaVu Sans", 9))
        style.configure("Flow.TLabel", background=BG, foreground="#245b8a", font=("DejaVu Sans", 10, "bold"))
        style.configure("Card.TLabel", background=WHITE, foreground=BLACK, font=("DejaVu Sans", 10, "bold"))
        style.configure("Value.TLabel", background=WHITE, foreground=BLACK, font=("DejaVu Sans Mono", 9))
        style.configure("Muted.TLabel", background=WHITE, foreground=GRAY, font=("DejaVu Sans", 8))
        style.configure("Action.TButton", font=("DejaVu Sans", 9, "bold"), padding=6)
        style.configure("TCheckbutton", background=WHITE)
        style.configure("TRadiobutton", background=WHITE)

    def card(self, parent, title):
        f = ttk.Frame(parent, style="Card.TFrame", padding=9)
        ttk.Label(f, text=title, style="Card.TLabel").pack(anchor="w", pady=(0, 6))
        return f

    def _build(self):
        self.outer = ttk.Frame(self.root, style="App.TFrame", padding=10)
        self.outer.pack(fill="both", expand=True)
        self.outer.columnconfigure(0, weight=1)
        self.outer.rowconfigure(2, weight=1)
        self.outer.rowconfigure(3, weight=1)

        header = ttk.Frame(self.outer, style="App.TFrame")
        header.grid(row=0, column=0, sticky="ew", pady=(0, 7))
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text="AI-ASSISTED WIRELESS AIR MOUSE GLOVE", style="Title.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(header, text="IMU acquisition  •  calibration  •  sensor fusion  •  adaptive stabilization  •  cursor simulation", style="Sub.TLabel").grid(row=1, column=0, sticky="w")
        ttk.Label(header, text="INPUT  →  CALIBRATE  →  FUSE  →  ADAPTIVE FILTER  →  CURSOR", style="Flow.TLabel").grid(row=2, column=0, sticky="w", pady=(5, 0))
        self.status = tk.StringVar(value="● LIVE  |  SYNTHETIC")
        ttk.Label(header, textvariable=self.status, background=BG, foreground=BLACK, font=("DejaVu Sans", 10, "bold")).grid(row=0, column=1, rowspan=2, sticky="e")

        # Info row
        info = ttk.Frame(self.outer, style="App.TFrame")
        info.grid(row=1, column=0, sticky="ew")
        for i in range(5):
            info.columnconfigure(i, weight=1)

        sensor = self.card(info, "VIRTUAL MPU6050")
        sensor.grid(row=0, column=0, sticky="nsew", padx=4)
        self.sensor_text = tk.StringVar()
        ttk.Label(sensor, textvariable=self.sensor_text, style="Value.TLabel", justify="left").pack(anchor="w")

        fusion = self.card(info, "FUSION / ADAPTIVE PROCESSOR")
        fusion.grid(row=0, column=1, sticky="nsew", padx=4)
        self.fusion_text = tk.StringVar()
        ttk.Label(fusion, textvariable=self.fusion_text, style="Value.TLabel", justify="left").pack(anchor="w")

        perf = self.card(info, "PERFORMANCE")
        perf.grid(row=0, column=2, sticky="nsew", padx=4)
        self.perf_text = tk.StringVar()
        ttk.Label(perf, textvariable=self.perf_text, style="Value.TLabel", justify="left").pack(anchor="w")

        events = self.card(info, "EVENT LOG")
        events.grid(row=0, column=3, sticky="nsew", padx=4)
        self.event_text = tk.StringVar()
        ttk.Label(events, textvariable=self.event_text, style="Muted.TLabel", justify="left").pack(anchor="w")

        source = self.card(info, "INPUT")
        source.grid(row=0, column=4, sticky="nsew", padx=4)
        self.source_var = tk.StringVar(value="Synthetic")
        ttk.Radiobutton(source, text="Synthetic", value="Synthetic", variable=self.source_var, command=self.change_source).pack(anchor="w")
        ttk.Radiobutton(source, text="Wokwi RFC2217", value="Wokwi", variable=self.source_var, command=self.change_source).pack(anchor="w")
        ttk.Radiobutton(source, text="Wokwi CLI", value="Wokwi CLI", variable=self.source_var, command=self.change_source).pack(anchor="w")
        ttk.Button(source, text="CONNECT WOKWI", style="Action.TButton", command=self.connect_wokwi).pack(anchor="w", fill="x", pady=(5, 0))
        self.wokwi_text = tk.StringVar(value="Not connected")
        ttk.Label(source, textvariable=self.wokwi_text, style="Muted.TLabel", wraplength=210, justify="left").pack(anchor="w", pady=(4, 0))
        ttk.Label(source, text="Wokwi must already be running on port 4000.", style="Muted.TLabel", wraplength=210, justify="left").pack(anchor="w", pady=(5, 0))

        # Main graph row
        graph_row = ttk.Frame(self.outer, style="App.TFrame")
        graph_row.grid(row=2, column=0, sticky="nsew", pady=(7, 0))
        graph_row.columnconfigure(0, weight=3)
        graph_row.columnconfigure(1, weight=2)
        graph_row.rowconfigure(0, weight=1)

        sig_card = self.card(graph_row, "RAW VS FILTERED ORIENTATION")
        sig_card.grid(row=0, column=0, sticky="nsew", padx=(4, 3))
        self.signal_host = ttk.Frame(sig_card, style="Card.TFrame")
        self.signal_host.pack(fill="both", expand=True)

        screen_card = self.card(graph_row, "VIRTUAL LAPTOP SCREEN")
        screen_card.grid(row=0, column=1, sticky="nsew", padx=(3, 4))
        self.screen = tk.Canvas(screen_card, bg=WHITE, highlightthickness=1, highlightbackground=LIGHT)
        self.screen.pack(fill="both", expand=True)

        # Bottom row
        bottom = ttk.Frame(self.outer, style="App.TFrame")
        bottom.grid(row=3, column=0, sticky="nsew", pady=(7, 0))
        bottom.columnconfigure(0, weight=3)
        bottom.columnconfigure(1, weight=2)
        bottom.rowconfigure(0, weight=1)

        traj_card = self.card(bottom, "CURSOR TRAJECTORY")
        traj_card.grid(row=0, column=0, sticky="nsew", padx=(4, 3))
        self.traj_host = ttk.Frame(traj_card, style="Card.TFrame")
        self.traj_host.pack(fill="both", expand=True)

        controls_card = self.card(bottom, "SIMULATION / TEST CONTROLS")
        controls_card.grid(row=0, column=1, sticky="nsew", padx=(3, 4))
        self.build_controls(controls_card)

    def build_controls(self, parent):
        row = ttk.Frame(parent, style="Card.TFrame")
        row.pack(fill="x")
        ttk.Label(row, text="Movement", style="Muted.TLabel").grid(row=0, column=0, sticky="w")
        self.mode_var = tk.StringVar(value="Slow")
        self.mode_box = ttk.Combobox(row, textvariable=self.mode_var, state="readonly", values=[
            "Stationary", "Slow", "Fast", "Circular", "Sudden", "Tremor", "Figure-8", "Precision"
        ], width=14)
        self.mode_box.grid(row=1, column=0, sticky="ew", pady=(2, 5))
        self.mode_box.bind("<<ComboboxSelected>>", lambda e: setattr(self.engine.motion, "mode", self.mode_var.get()))

        ttk.Label(row, text="Filter", style="Muted.TLabel").grid(row=0, column=1, sticky="w", padx=(8, 0))
        self.filter_var = tk.StringVar(value="Adaptive")
        filter_box = ttk.Combobox(row, textvariable=self.filter_var, state="readonly", values=["Adaptive", "Fixed", "Raw"], width=10)
        filter_box.grid(row=1, column=1, sticky="ew", pady=(2, 5), padx=(8, 0))
        filter_box.bind("<<ComboboxSelected>>", lambda e: setattr(self.engine.fusion, "mode", self.filter_var.get()))

        for c in (0, 1):
            row.columnconfigure(c, weight=1)

        self._slider(parent, "Sensor noise", 0.0, 0.8, self.engine.sensor.noise, self.set_noise, 3)
        self._slider(parent, "Accel noise", 0.0, 0.08, self.engine.sensor.accel_noise, self.set_acc_noise, 3)
        self._slider(parent, "Bias", 0.0, 0.15, self.engine.sensor.bias, self.set_bias, 3)
        self._slider(parent, "Drift", 0.0, 0.10, self.engine.sensor.drift, self.set_drift, 3)
        self._slider(parent, "Packet loss", 0.0, 0.25, self.engine.sensor.packet_loss, self.set_packet_loss, 3)
        self._slider(parent, "Sampling Hz", 20, 100, self.engine.sampling_rate, self.set_sampling, 1)
        self._slider(parent, "Sensitivity", 0.2, 1.8, self.engine.cursor.sensitivity, self.set_sensitivity, 2)
        self._slider(parent, "Dead zone °", 0.0, 3.0, self.engine.cursor.dead_zone, self.set_dead_zone, 2)

        flags = ttk.Frame(parent, style="Card.TFrame")
        flags.pack(fill="x", pady=(3, 4))
        self.quant_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(flags, text="16-bit quantization", variable=self.quant_var, command=lambda: setattr(self.engine.sensor, "quantization", self.quant_var.get())).pack(side="left")
        self.invert_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(flags, text="Invert Y", variable=self.invert_var, command=lambda: setattr(self.engine.cursor, "invert_y", self.invert_var.get())).pack(side="left", padx=(10, 0))
        self.invert_x_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(flags, text="Invert X", variable=self.invert_x_var, command=lambda: setattr(self.engine.cursor, "invert_x", self.invert_x_var.get())).pack(side="left", padx=(10, 0))

        buttons = ttk.Frame(parent, style="Card.TFrame")
        buttons.pack(fill="x", pady=2)
        specs = [
            ("CALIBRATE", self.calibrate),
            ("RECORD", self.toggle_record),
            ("BENCHMARK", self.run_benchmark),
            ("RESET", self.reset),
        ]
        for i, (name, cmd) in enumerate(specs):
            ttk.Button(buttons, text=name, style="Action.TButton", command=cmd).grid(row=0, column=i, sticky="ew", padx=2)
            buttons.columnconfigure(i, weight=1)

        click_row = ttk.Frame(parent, style="Card.TFrame")
        click_row.pack(fill="x", pady=(5, 2))
        ttk.Button(click_row, text="LEFT CLICK", command=lambda: self.engine.click("Left")).pack(side="left", fill="x", expand=True, padx=(0, 3))
        ttk.Button(click_row, text="RIGHT CLICK", command=lambda: self.engine.click("Right")).pack(side="left", fill="x", expand=True, padx=(3, 0))

        self.test_text = tk.StringVar(value="Ready for test")
        ttk.Label(parent, textvariable=self.test_text, style="Muted.TLabel", justify="left").pack(anchor="w", pady=(4, 0))
        ttk.Label(
            parent,
            text="Demo: Stationary → CALIBRATE → Tremor + noise → Fast → Circular → BENCHMARK",
            style="Muted.TLabel", wraplength=470, justify="left"
        ).pack(anchor="w", pady=(7, 0))

    def _slider(self, parent, name, lo, hi, value, callback, resolution):
        row = ttk.Frame(parent, style="Card.TFrame")
        row.pack(fill="x", pady=1)
        ttk.Label(row, text=name, width=15, style="Muted.TLabel").pack(side="left")
        value_text = tk.StringVar()
        integer_scale = float(lo).is_integer() and float(hi - lo) >= 10
        step = 1.0 if integer_scale else 10.0 ** (-resolution)
        value_text.set(f"{value:.0f}" if integer_scale else f"{value:.{resolution}f}")

        def changed(raw):
            numeric = float(raw)
            callback(numeric)
            value_text.set(f"{numeric:.0f}" if integer_scale else f"{numeric:.{resolution}f}")

        scale = tk.Scale(
            row, from_=lo, to=hi, resolution=step, orient="horizontal",
            showvalue=False, bg=WHITE, fg=BLACK, troughcolor="#cfcfcf",
            highlightthickness=0, bd=0, command=changed
        )
        scale.set(value)
        scale.pack(side="left", fill="x", expand=True)
        ttk.Label(row, textvariable=value_text, width=8, anchor="e", style="Value.TLabel").pack(side="right", padx=(5, 0))

    def set_noise(self, v): self.engine.sensor.noise = v
    def set_acc_noise(self, v): self.engine.sensor.accel_noise = v
    def set_bias(self, v): self.engine.sensor.bias = v
    def set_drift(self, v): self.engine.sensor.drift = v
    def set_packet_loss(self, v): self.engine.sensor.packet_loss = v
    def set_sampling(self, v): self.engine.sampling_rate = int(v)
    def set_sensitivity(self, v): self.engine.cursor.sensitivity = v
    def set_dead_zone(self, v): self.engine.cursor.dead_zone = v

    def change_source(self):
        source = self.source_var.get()
        self.engine.set_source(source)
        if source == "Synthetic":
            self.status.set("● LIVE  |  SYNTHETIC")
            self.wokwi_text.set("Synthetic input selected")
        else:
            self.status.set("● WAITING  |  WOKWI")
            self.wokwi_text.set("Click CONNECT WOKWI after starting Wokwi")

    def connect_wokwi(self):
        self.engine.wokwi.port = "rfc2217://localhost:4000"
        ok = self.engine.wokwi.connect()
        if ok:
            selected = self.source_var.get()
            self.engine.source = selected if selected in ("Wokwi", "Wokwi CLI") else "Wokwi"
            self.status.set("● LIVE  |  WOKWI")
            self.wokwi_text.set("Connected; waiting for IMU frames…")
        else:
            self.source_var.set("Synthetic")
            self.engine.source = "Synthetic"
            self.status.set("● FALLBACK  |  SYNTHETIC")
            self.wokwi_text.set(f"Wokwi unavailable: {self.engine.wokwi.status}")

    def calibrate(self):
        self.engine.calibrate()
        self.test_text.set("Calibration active: keep the simulated/real IMU still.")

    def toggle_record(self):
        if not self.engine.recorder.active:
            self.engine.recorder.start()
            self.test_text.set(f"Recording: {self.engine.recorder.path.name}")
        else:
            path = self.engine.recorder.stop()
            self.test_text.set(f"Recording saved: {path.name if path else 'n/a'}")

    def run_benchmark(self):
        self.status.set("● RUNNING  |  BENCHMARK")
        self.root.update_idletasks()
        rows, path = self.engine.benchmark()
        best = max(rows, key=lambda r: r["reduction_percent"])
        self.test_text.set(
            f"Benchmark complete: {path.name}\n"
            f"Best reduction: {best['mode']} = {best['reduction_percent']:.1f}%"
        )
        self.status.set("● LIVE  |  SYNTHETIC")
        self.engine.source = "Synthetic"
        self.source_var.set("Synthetic")

    def reset(self):
        self.engine.reset()
        self.mode_var.set(self.engine.motion.mode)
        self.filter_var.set(self.engine.fusion.mode)
        self.test_text.set("System reset")

    def _build_plots(self):
        fig = Figure(figsize=(7, 4), dpi=90, facecolor=WHITE)
        self.ax1 = fig.add_subplot(211)
        self.ax2 = fig.add_subplot(212)
        for ax in (self.ax1, self.ax2):
            ax.set_facecolor(WHITE)
            ax.grid(True, color=GRID, linewidth=0.6)
            ax.tick_params(labelsize=7)

        self.ax1.set_title("Roll orientation — raw vs adaptive output", fontsize=9, loc="left")
        self.ax1.set_ylabel("degrees", fontsize=8)
        self.raw_line, = self.ax1.plot([], [], color="#d97706", linewidth=1.1, label="Raw / noisy")
        self.filt_line, = self.ax1.plot([], [], color="#1769aa", linewidth=2.2, label="Filtered output")
        self.true_line, = self.ax1.plot([], [], color="#2e8b57", linestyle="--", linewidth=1.2, label="Reference")
        self.ax1.legend(frameon=False, fontsize=7, loc="upper right")
        self.ax1.set_xlim(0, 300)
        self.ax1.set_ylim(-35, 35)

        self.ax2.set_title("Gyroscope magnitude", fontsize=9, loc="left")
        self.ax2.set_xlabel("samples", fontsize=8)
        self.ax2.set_ylabel("deg/s", fontsize=8)
        self.gyro_line, = self.ax2.plot([], [], color="#7c3aed", linewidth=1.5, label="Gyro magnitude")
        self.ax2.legend(frameon=False, fontsize=7, loc="upper right")
        self.ax2.set_xlim(0, 300)
        self.ax2.set_ylim(0, 100)

        fig.subplots_adjust(left=0.08, right=0.98, top=0.94, bottom=0.10, hspace=0.38)
        self.canvas = FigureCanvasTkAgg(fig, self.signal_host)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        fig3 = Figure(figsize=(7, 3), dpi=90, facecolor=WHITE)
        self.at = fig3.add_subplot(111)
        self.at.set_facecolor(WHITE)
        self.at.grid(True, color=GRID, linewidth=0.6)
        self.at.set_xlim(0, 100)
        self.at.set_ylim(0, 100)
        self.at.set_xlabel("screen X", fontsize=8)
        self.at.set_ylabel("screen Y", fontsize=8)
        self.target_line, = self.at.plot([75], [70], marker="o", markersize=11, markerfacecolor="none", markeredgecolor=BLACK, linewidth=1.5)
        self.traj_line, = self.at.plot([], [], color="#1769aa", linewidth=1.8, label="Filtered path")
        self.raw_traj_line, = self.at.plot([], [], color="#d97706", linewidth=1.0, linestyle="--", label="Raw path")
        self.dot, = self.at.plot([], [], "o", color="#1769aa", markersize=7, label="Cursor")
        self.at.legend(frameon=False, fontsize=7, loc="lower left")
        fig3.subplots_adjust(left=0.10, right=0.98, top=0.94, bottom=0.18)
        self.traj_canvas = FigureCanvasTkAgg(fig3, self.traj_host)
        self.traj_canvas.get_tk_widget().pack(fill="both", expand=True)

    def _draw_screen(self):
        c = self.screen
        c.delete("all")
        w = max(300, c.winfo_width())
        h = max(220, c.winfo_height())
        l, t, r, b = 20, 15, w - 15, h - 20

        c.create_rectangle(l, t, r, b, outline=LIGHT)
        for i in range(1, 10):
            x = l + (r-l)*i/10
            y = t + (b-t)*i/10
            c.create_line(x, t, x, b, fill="#eeeeee")
            c.create_line(l, y, r, y, fill="#eeeeee")

        def sx(v): return l + (r-l) * v / 100.0
        def sy(v): return b - (b-t) * v / 100.0

        tx, ty = 75, 70
        c.create_oval(sx(tx)-9, sy(ty)-9, sx(tx)+9, sy(ty)+9, outline=BLACK, width=2)
        c.create_text(sx(tx)+12, sy(ty)-12, anchor="w", text="TARGET", fill=GRAY, font=("DejaVu Sans", 8, "bold"))

        rawx, rawy = self.engine.cursor.raw_x, self.engine.cursor.raw_y
        fx, fy = self.engine.cursor.x, self.engine.cursor.y

        c.create_oval(sx(rawx)-4, sy(rawy)-4, sx(rawx)+4, sy(rawy)+4, outline=MID)
        c.create_oval(sx(fx)-6, sy(fy)-6, sx(fx)+6, sy(fy)+6, fill=BLACK)

        c.create_text(l, b+7, anchor="w", text="0,0", fill=GRAY, font=("DejaVu Sans", 7))
        c.create_text(r, b+7, anchor="e", text="100,100", fill=GRAY, font=("DejaVu Sans", 7))

        dist = math.hypot(fx-tx, fy-ty)
        c.create_text(l+5, t+5, anchor="nw", text=f"Cursor  X={fx:5.1f}  Y={fy:5.1f}   Target error={dist:4.1f}", fill=BLACK, font=("DejaVu Sans Mono", 8, "bold"))

    def tick(self):
        if not self.running:
            return
        try:
            # Limit work so UI remains responsive even at high simulated rates.
            result = self.engine.step()
            self.update_widgets(result)
        except Exception as exc:
            self.status.set(f"● ERROR  |  {type(exc).__name__}")
            self.test_text.set(str(exc)[:90])

        delay = max(8, int(1000 / max(self.engine.sampling_rate, 1)))
        self.root.after(delay, self.tick)

    def update_widgets(self, d):
        s = d["sample"]
        self.sensor_text.set(
            f"Accel  X {s.ax:+6.3f} g   Y {s.ay:+6.3f} g   Z {s.az:+6.3f} g\n"
            f"Gyro   X {s.gx:+7.2f}   Y {s.gy:+7.2f}   Z {s.gz:+7.2f} °/s\n"
            f"Range  ±2 g / ±250 °/s   •   16-bit {'ON' if self.engine.sensor.quantization else 'OFF'}\n"
            f"Drops  {self.engine.dropped_count:4d} / {self.engine.sample_count:4d}"
        )
        self.fusion_text.set(
            f"Mode       {self.engine.fusion.mode}\n"
            f"Roll       raw {d['raw_roll']:+6.2f}°   out {d['filtered_roll']:+6.2f}°\n"
            f"Pitch      raw {d['raw_pitch']:+6.2f}°   out {d['filtered_pitch']:+6.2f}°\n"
            f"Motion     {math.hypot(s.gx, s.gy):6.2f} °/s\n"
            f"Noise est. {self.engine.fusion.noise_estimate:6.3f}°  |  α {d['alpha']:5.3f}\n"
            f"Calibration {'IN PROGRESS' if self.engine.cal.active else 'READY'}"
        )
        self.perf_text.set(
            f"RAW ERROR          {self.engine.metrics.raw_rms:7.3f}°\n"
            f"FILTERED ERROR     {self.engine.metrics.filtered_rms:7.3f}°\n"
            f"NOISE REDUCTION    {self.engine.metrics.reduction:7.1f}%\n"
            f"CURSOR STABILITY   {self.engine.metrics.stability:7.1f}%\n"
            f"Raw jitter         {self.engine.metrics.raw_jitter_rms:7.3f}°\n"
            f"Filtered jitter    {self.engine.metrics.filtered_jitter_rms:7.3f}°\n"
            f"Estimated latency  {self.engine.metrics.latency:7.1f} ms\n"
            f"Samples            {self.engine.sample_count:7d}"
        )
        remote = f"Remote clicks  L={self.engine.wokwi.remote_left_clicks} R={self.engine.wokwi.remote_right_clicks}"
        self.event_text.set(("\n".join(list(self.engine.events)[:5]) + "\n" + remote).strip() or "Ready")
        self.wokwi_text.set(self.engine.wokwi.status)

        if self.engine.source in ("Wokwi", "Wokwi CLI"):
            if self.engine.wokwi.latest is None:
                self.status.set("● WAITING  |  WOKWI — no IMU frame yet")
            else:
                self.status.set(f"● LIVE  |  {self.engine.source.upper()}")
        else:
            self.status.set(f"● LIVE  |  SYNTHETIC • {self.engine.motion.mode.upper()}")

        x = np.arange(len(self.engine.raw_roll_hist))
        self.raw_line.set_data(x, list(self.engine.raw_roll_hist))
        self.filt_line.set_data(x, list(self.engine.filt_roll_hist))
        self.true_line.set_data(x, list(self.engine.true_roll_hist))
        self.gyro_line.set_data(x, list(self.engine.gyro_hist))

        if len(self.engine.gyro_hist):
            ymax = max(50.0, min(300.0, max(self.engine.gyro_hist) * 1.2))
            self.ax2.set_ylim(0, ymax)

        self.traj_line.set_data(list(self.engine.path_x), list(self.engine.path_y))
        self.raw_traj_line.set_data(list(self.engine.path_raw_x), list(self.engine.path_raw_y))
        self.dot.set_data([self.engine.cursor.x], [self.engine.cursor.y])

        self.canvas.draw_idle()
        self.traj_canvas.draw_idle()
        self._draw_screen()

    def close(self):
        self.running = False
        self.engine.running = False
        self.engine.wokwi.close()
        if self.engine.recorder.active:
            self.engine.recorder.stop()
        self.root.destroy()


def self_test():
    engine = AirMouseEngine()
    engine.sampling_rate = 50
    engine.motion.mode = "Tremor"
    engine.sensor.noise = 0.35
    for _ in range(500):
        engine.step()
    assert engine.sample_count == 500
    assert parse_imu_line("IMU,0.0,0.0,1.0,0.0,0.0,0.0") is not None
    assert parse_imu_line("IMU,ERROR,ERROR,ERROR,ERROR,ERROR,ERROR") is None

    calibration_engine = AirMouseEngine()
    calibration_engine.motion.mode = "Stationary"
    calibration_engine.calibrate()
    for _ in range(60):
        calibration_engine.step()
    assert not calibration_engine.cal.active
    assert np.linalg.norm(calibration_engine.cal.gyro_bias) > 0.0

    engine.recorder.start()
    engine.step()
    recording_path = engine.recorder.stop()
    assert recording_path and recording_path.exists()

    assert 0.0 <= engine.cursor.x <= 100.0
    assert 0.0 <= engine.cursor.y <= 100.0
    print("AIR MOUSE BACKEND SELF-TEST")
    print(f"samples            : {engine.sample_count}")
    print(f"raw RMS error       : {engine.metrics.raw_rms:.3f} deg")
    print(f"filtered RMS error  : {engine.metrics.filtered_rms:.3f} deg")
    print(f"noise reduction     : {engine.metrics.reduction:.1f}%")
    print(f"stability           : {engine.metrics.stability:.1f}%")
    print(f"cursor              : ({engine.cursor.x:.1f}, {engine.cursor.y:.1f})")
    print("SELF TEST PASSED")


def cli_benchmark():
    engine = AirMouseEngine()
    rows, path = engine.benchmark()
    print("AIR MOUSE BENCHMARK")
    for row in rows:
        print(
            f"{row['mode']:<12} raw={row['raw_jitter_deg']:>7.3f}  "
            f"filtered={row['filtered_jitter_deg']:>7.3f}  "
            f"reduction={row['reduction_percent']:>6.1f}%  "
            f"stability={row['stability_percent']:>6.1f}%"
        )
    print(f"report: {path}")


def main():
    parser = argparse.ArgumentParser(description="Air Mouse Glove simulation")
    parser.add_argument("--self-test", action="store_true", help="run headless backend self-test")
    parser.add_argument("--benchmark", action="store_true", help="run headless benchmark")
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return
    if args.benchmark:
        cli_benchmark()
        return

    try:
        root = tk.Tk()
    except tk.TclError as exc:
        print(f"GUI unavailable: {exc}. Run this command from a graphical desktop session.", file=sys.stderr)
        raise SystemExit(1)
    AirMouseApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
