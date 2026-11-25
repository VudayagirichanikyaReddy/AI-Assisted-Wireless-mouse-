import subprocess
import sys
from pathlib import Path

PROJECT = sys.argv[1] if len(sys.argv) > 1 else str(Path(__file__).resolve().parent.parent / "wokwi")

cmd = [
    "wokwi-cli",
    PROJECT,
    "--timeout",
    "0"
]

p = subprocess.Popen(
    cmd,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    bufsize=1,
)

for line in p.stdout:
    line = line.rstrip()

    if line.startswith("IMU,"):
        print(line, flush=True)

    elif line.startswith("AIR_MOUSE_WOKWI_READY"):
        print(line, flush=True)

p.wait()
