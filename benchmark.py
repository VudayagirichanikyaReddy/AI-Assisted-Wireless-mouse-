"""Benchmark entry point using the same AirMouseEngine as the GUI."""

from air_mouse import AirMouseEngine


def run():
    return AirMouseEngine().benchmark()

