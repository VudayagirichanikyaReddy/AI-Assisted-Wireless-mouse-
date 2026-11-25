"""Explainable adaptive exponential smoothing."""

from math import hypot


def adaptive_alpha(delta, noise_estimate, dead_zone=0.7):
    """Choose smoothing strength: quiet input is smoothed more strongly."""
    if delta < max(dead_zone, noise_estimate * 0.35):
        return 0.08
    if delta < 3.0:
        return 0.18
    if delta < 7.0:
        return 0.30
    return 0.48


def motion_magnitude(gx, gy):
    return hypot(gx, gy)

