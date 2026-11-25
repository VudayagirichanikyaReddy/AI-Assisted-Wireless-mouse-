"""Bounded 2-D cursor mapping."""


def clamp_cursor(x, y):
    return max(0.0, min(100.0, x)), max(0.0, min(100.0, y))

