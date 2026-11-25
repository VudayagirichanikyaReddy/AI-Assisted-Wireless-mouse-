"""Input data types and safe parsing for IMU streams."""

from dataclasses import dataclass


@dataclass
class IMUReading:
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float


def parse_imu_line(line: str):
    """Parse ``IMU,ax,ay,az,gx,gy,gz``; return None for bad frames."""
    fields = line.strip().split(",")
    if len(fields) != 7 or fields[0] != "IMU":
        return None
    try:
        values = [float(value) for value in fields[1:]]
    except (TypeError, ValueError):
        return None
    return IMUReading(*values)

