#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
WORKSPACE="${UCC_WORKSPACE:-$HOME/vio_sim_ws}"
VENV="${AIRSIM_VENV:-$WORKSPACE/airsim_pyenv}"
PYCLIENT="${AIRSIM_PYCLIENT:-$WORKSPACE/Colosseum/PythonClient}"
BRIDGE="$REPO_ROOT/tools/fixedwing/ucc_fixedwing_mvp_v1/09_run_gazebo_airsim_bridge.sh"
ARTIFACT_ROOT="${BRIDGE_ARTIFACT_ROOT:-$WORKSPACE/artifacts/fixedwing_kau_400m}"

if [[ ! -x "$VENV/bin/python" ]]; then
  echo "[ERROR] Python environment not found: $VENV" >&2
  exit 1
fi

if ! ss -ltn 2>/dev/null | grep -q ':41451'; then
  echo "[ERROR] AirSim RPC port 41451 is not open." >&2
  exit 1
fi

if pgrep -f '[g]azebo_airsim_bridge.py' >/dev/null; then
  echo "[ERROR] another Gazebo-to-AirSim bridge is already running." >&2
  exit 1
fi

PYTHONPATH="$PYCLIENT${PYTHONPATH:+:$PYTHONPATH}" "$VENV/bin/python" - <<'PY'
import json
import math
import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
settings = json.loads(client.getSettingsString())
vehicle = settings.get("Vehicles", {}).get("Drone1", {})
spawn_z = float(vehicle.get("Z", float("nan")))
if settings.get("PhysicsEngineName") != "ExternalPhysicsEngine":
    raise SystemExit("[ERROR] AirSim is not using ExternalPhysicsEngine")
if not math.isclose(spawn_z, -400.0, abs_tol=1.0e-6):
    raise SystemExit(
        f"[ERROR] Drone1 runtime Z is {spawn_z}; required -400. "
        "Stop PIE, run 00_apply_400m_profile.sh, and restart PIE."
    )
print("[PREFLIGHT] ExternalPhysicsEngine and KAU 400 m spawn confirmed")
PY

mkdir -p "$ARTIFACT_ROOT"
exec "$BRIDGE" \
  --duration-sec "${BRIDGE_DURATION_SEC:-0}" \
  --summary "$ARTIFACT_ROOT/bridge_summary.json" \
  --state-log "$ARTIFACT_ROOT/bridge_state.csv" \
  "$@"
