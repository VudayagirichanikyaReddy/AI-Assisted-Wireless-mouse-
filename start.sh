#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJECT_DIR"

if [ ! -x "$PROJECT_DIR/python/.venv/bin/python" ]; then
    echo "Python environment is missing: $PROJECT_DIR/python/.venv" >&2
    echo "Run the project setup once, then use ./start.sh from then on." >&2
    exit 1
fi

exec "$PROJECT_DIR/run_integrated.sh"
