#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${UCC_WORKSPACE:-$HOME/vio_sim_ws}"
VENV="${AIRSIM_VENV:-$WORKSPACE/airsim_pyenv}"
PYCLIENT="${AIRSIM_PYCLIENT:-$WORKSPACE/Colosseum/PythonClient}"

if [[ ! -x "$VENV/bin/python" ]]; then
  echo "[ERROR] AirSim Python environment not found: $VENV" >&2
  exit 1
fi

if [[ ! -d "$PYCLIENT/airsim" ]]; then
  echo "[ERROR] Colosseum PythonClient not found: $PYCLIENT" >&2
  exit 1
fi

if ! timeout 1 bash -c "</dev/tcp/127.0.0.1/41451" 2>/dev/null; then
  echo "[ERROR] AirSim RPC port 41451 is not open." >&2
  echo "        Apply the profile, restart Unreal Play/PIE, then rerun." >&2
  exit 1
fi

export PYTHONPATH="$PYCLIENT${PYTHONPATH:+:$PYTHONPATH}"
exec "$VENV/bin/python" "$SCRIPT_DIR/synthetic_kinematics_probe.py" "$@"
