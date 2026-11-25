#!/bin/bash
cd "$(dirname "$0")"
export MPLCONFIGDIR="${MPLCONFIGDIR:-$PWD/.cache/matplotlib}"
export PLATFORMIO_CORE_DIR="${PLATFORMIO_CORE_DIR:-$PWD/.platformio}"
mkdir -p "$MPLCONFIGDIR" "$PLATFORMIO_CORE_DIR"
echo "=== AIR MOUSE GLOVE ==="
echo "Project: $PWD"
echo
echo "Python:"
python/.venv/bin/python --version
echo
python/.venv/bin/python -c "import numpy,scipy,matplotlib,serial; print('NumPy      OK'); print('SciPy      OK'); print('Matplotlib OK'); print('PySerial   OK')"
echo
echo "Wokwi:"
test -f wokwi/diagram.json && echo "diagram.json OK"
test -f wokwi/wokwi.toml && echo "wokwi.toml OK"
test -f wokwi/src/air_mouse.ino && echo "firmware OK"
test -f wokwi/.pio/build/esp32dev/firmware.bin && echo "firmware.bin OK"
test -f wokwi/.pio/build/esp32dev/firmware.elf && echo "firmware.elf OK"
echo
echo "Python simulator:"
test -f python/air_mouse.py && echo "air_mouse.py OK"
command -v wokwi-cli >/dev/null 2>&1 && echo "wokwi-cli OK" || echo "wokwi-cli MISSING"
