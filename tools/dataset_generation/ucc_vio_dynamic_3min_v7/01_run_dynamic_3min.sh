#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VENV="${AIRSIM_VENV:-$HOME/vio_sim_ws/airsim_pyenv}"
PYCLIENT="${AIRSIM_PYCLIENT:-$HOME/vio_sim_ws/Colosseum/PythonClient}"

if [[ ! -d "$VENV" ]]; then
  echo "[ERROR] AirSim Python venv not found: $VENV" >&2
  exit 1
fi

if [[ ! -d "$PYCLIENT" ]]; then
  echo "[ERROR] AirSim PythonClient not found: $PYCLIENT" >&2
  exit 1
fi

if ! ss -ltn 2>/dev/null | grep -q ':41451'; then
  echo "[ERROR] AirSim RPC port 41451 is not open." >&2
  echo "        Start Unreal Play/PIE first, then rerun this script." >&2
  exit 1
fi

# shellcheck disable=SC1091
source "$VENV/bin/activate"
export PYTHONPATH="$PYCLIENT:${PYTHONPATH:-}"

exec python3 "$SCRIPT_DIR/v7_dynamic_3min.py" \
  --duration-sec 180 \
  --speed-mps 30 \
  --camera-hz 10 \
  --imu-hz 100 \
  "$@"
