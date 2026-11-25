#!/bin/bash
set -e
cd "$(dirname "$0")"
export MPLCONFIGDIR="${MPLCONFIGDIR:-$PWD/.cache/matplotlib}"
mkdir -p "$MPLCONFIGDIR"
exec python/.venv/bin/python python/air_mouse.py "$@"
