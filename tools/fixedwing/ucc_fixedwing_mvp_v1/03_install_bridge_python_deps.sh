#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${UCC_WORKSPACE:-$HOME/vio_sim_ws}"
VENV="${AIRSIM_VENV:-$WORKSPACE/airsim_pyenv}"

if [[ ! -x "$VENV/bin/python" ]]; then
  echo "[ERROR] AirSim Python environment not found: $VENV" >&2
  exit 1
fi

exec "$VENV/bin/python" -m pip install -r "$SCRIPT_DIR/requirements.txt"
