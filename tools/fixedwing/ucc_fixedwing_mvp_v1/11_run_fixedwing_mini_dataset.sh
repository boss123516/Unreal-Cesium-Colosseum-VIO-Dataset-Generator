#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${UCC_WORKSPACE:-$HOME/vio_sim_ws}"
VENV="${AIRSIM_VENV:-$WORKSPACE/airsim_pyenv}"
PYCLIENT="${AIRSIM_PYCLIENT:-$WORKSPACE/Colosseum/PythonClient}"
SYSTEM_DIST_PACKAGES="${SYSTEM_DIST_PACKAGES:-/usr/lib/python3/dist-packages}"

if [[ ! -x "$VENV/bin/python" ]]; then
  echo "[ERROR] AirSim Python environment not found: $VENV" >&2
  exit 1
fi

export PYTHONPATH="$SYSTEM_DIST_PACKAGES:$PYCLIENT${PYTHONPATH:+:$PYTHONPATH}"
exec "$VENV/bin/python" -u "$SCRIPT_DIR/fixedwing_mini_dataset.py" "$@"
