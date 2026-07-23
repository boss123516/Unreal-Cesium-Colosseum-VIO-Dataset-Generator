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
  echo "[ERROR] Colosseum PythonClient not found: $PYCLIENT" >&2
  exit 1
fi

if ! ss -ltn 2>/dev/null | grep -q ':41451'; then
  echo "[ERROR] AirSim RPC port 41451 is not open." >&2
  echo "        Start Unreal Play/PIE, wait for the buildings, then rerun." >&2
  exit 1
fi

# shellcheck disable=SC1091
source "$VENV/bin/activate"
export PYTHONPATH="$PYCLIENT:${PYTHONPATH:-}"

exec python3 -u "$SCRIPT_DIR/building_orbit_30m.py" "$@"
