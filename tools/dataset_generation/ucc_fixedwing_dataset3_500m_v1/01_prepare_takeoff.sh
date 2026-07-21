#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${UCC_WORKSPACE:-$HOME/vio_sim_ws}"
VENV="${AIRSIM_VENV:-$WORKSPACE/airsim_pyenv}"
OUTPUT="${PREPARE_METADATA:-$WORKSPACE/artifacts/fixedwing_dataset3/prepare_mission.json}"

if [[ ! -x "$VENV/bin/python" ]]; then
  echo "[ERROR] Python environment not found: $VENV" >&2
  exit 1
fi

if ! ss -lun 2>/dev/null | grep -q ':14580'; then
  echo "[ERROR] PX4 MAVLink input port 14580 is not open." >&2
  echo "        Start PX4/Gazebo with 08_run_gz_rc_cessna_ucc.sh first." >&2
  exit 1
fi

if ! ss -ltn 2>/dev/null | grep -q ':41451'; then
  echo "[ERROR] AirSim RPC port 41451 is not open." >&2
  echo "        Apply the 500 m profile and start Unreal Play/PIE first." >&2
  exit 1
fi

if pgrep -f '[g]azebo_airsim_bridge.py' >/dev/null; then
  echo "[ERROR] another Gazebo-to-AirSim bridge is already running." >&2
  echo "        Stop it before preparing the 500 m reanchor." >&2
  exit 1
fi

"$VENV/bin/python" -u "$SCRIPT_DIR/px4_fixedwing_mission.py" prepare \
  --target-relative-altitude-m 100 \
  --prepare-timeout-sec 180 \
  --output "$OUTPUT" \
  "$@"

echo "[NEXT] Takeoff is stable; starting the 500 m bridge immediately."
exec "$SCRIPT_DIR/02_run_500m_bridge.sh"
