#!/bin/bash
set -Eeuo pipefail
cd "$(dirname "$0")"

# Local semester-demo default. An explicitly exported token overrides it.
: "${WOKWI_CLI_TOKEN:=wok_2tTvuKw1nxOwE8Z5h9LQztRASZSuHf7J77bd26e3}"
export WOKWI_CLI_TOKEN

if [ -z "${WOKWI_CLI_TOKEN:-}" ]; then
    echo "WOKWI_CLI_TOKEN is not set — WOKWI UNAVAILABLE — SYNTHETIC FALLBACK" >&2
    exec ./run_python.sh
fi

export PLATFORMIO_CORE_DIR="${PLATFORMIO_CORE_DIR:-$PWD/.platformio}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-$PWD/.cache/matplotlib}"
mkdir -p "$PLATFORMIO_CORE_DIR" "$MPLCONFIGDIR"

if ! command -v wokwi-cli >/dev/null 2>&1; then
    echo "wokwi-cli was not found — WOKWI UNAVAILABLE — SYNTHETIC FALLBACK" >&2
    exec ./run_python.sh
fi

if [ ! -f wokwi/.pio/build/esp32dev/firmware.bin ]; then
    (cd wokwi && ../python/.venv/bin/pio run)
fi

wokwi-cli wokwi --timeout 0 >"$PWD/logs/wokwi-cli.log" 2>&1 &
WOKWI_PID=$!
cleanup() {
    kill "$WOKWI_PID" 2>/dev/null || true
    wait "$WOKWI_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if ! python/.venv/bin/python python/run_wokwi.py --timeout 12; then
    echo "Wokwi could not open RFC2217 port 4000; see logs/wokwi-cli.log" >&2
    echo "WOKWI UNAVAILABLE — SYNTHETIC FALLBACK" >&2
    exec ./run_python.sh
fi

exec python/.venv/bin/python python/run_wokwi.py --gui
