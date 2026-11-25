"""Stationary-sample bias estimation."""

import numpy as np


def estimate_bias(samples):
    """Return accelerometer and gyroscope bias from stationary samples."""
    if not samples:
        raise ValueError("at least one calibration sample is required")
    acc = np.asarray([[s.ax, s.ay, s.az] for s in samples], dtype=float)
    gyro = np.asarray([[s.gx, s.gy, s.gz] for s in samples], dtype=float)
    return np.mean(acc, axis=0) - np.array([0.0, 0.0, 1.0]), np.mean(gyro, axis=0)

