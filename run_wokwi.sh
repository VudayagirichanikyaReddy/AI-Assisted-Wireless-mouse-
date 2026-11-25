#!/bin/bash
set -Eeuo pipefail
cd "$(dirname "$0")"

export PLATFORMIO_CORE_DIR="${PLATFORMIO_CORE_DIR:-$PWD/.platformio}"
mkdir -p "$PLATFORMIO_CORE_DIR"

if ! command -v wokwi-cli >/dev/null 2>&1; then
    echo "wokwi-cli was not found on PATH." >&2
    exit 1
fi

if [ -z "${WOKWI_CLI_TOKEN:-}" ]; then
    echo "WOKWI_CLI_TOKEN is not set; run ./run_python.sh for Synthetic mode." >&2
    exit 1
fi

if [ ! -f wokwi/.pio/build/esp32dev/firmware.bin ]; then
    (cd wokwi && ../python/.venv/bin/pio run)
fi

exec wokwi-cli wokwi --timeout 0
