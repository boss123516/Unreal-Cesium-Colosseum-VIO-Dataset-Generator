#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
WORKSPACE="${UCC_WORKSPACE:-$HOME/vio_sim_ws}"
VENV="${AIRSIM_VENV:-$WORKSPACE/airsim_pyenv}"
PYCLIENT="${AIRSIM_PYCLIENT:-$WORKSPACE/Colosseum/PythonClient}"
SYSTEM_DIST_PACKAGES="${SYSTEM_DIST_PACKAGES:-/usr/lib/python3/dist-packages}"
DATASET_ROOT="${DATASET_OUTPUT_ROOT:-$WORKSPACE/datasets}"
DURATION_SEC="${DURATION_SEC:-180}"
SPEED_MPS="${SPEED_MPS:-19}"
BANK_LIMIT_DEG="${BANK_LIMIT_DEG:-28}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="${DATASET_OUTPUT:-$DATASET_ROOT/ucc_fixedwing_dataset3_500m_$STAMP}"
READY_FILE="$OUTPUT/.capture_ready"
RECORDER="$REPO_ROOT/tools/fixedwing/ucc_fixedwing_mvp_v1/fixedwing_mini_dataset.py"
MISSION="$SCRIPT_DIR/px4_fixedwing_mission.py"

if [[ ! -x "$VENV/bin/python" ]]; then
  echo "[ERROR] Python environment not found: $VENV" >&2
  exit 1
fi

if [[ -e "$OUTPUT" ]]; then
  echo "[ERROR] output already exists: $OUTPUT" >&2
  exit 1
fi

if ! ss -ltn 2>/dev/null | grep -q ':41451'; then
  echo "[ERROR] AirSim RPC port 41451 is not open." >&2
  exit 1
fi

if ! gz topic -l 2>/dev/null | grep -q '^/ucc/fixed_wing/kinematics$'; then
  echo "[ERROR] Gazebo fixed-wing kinematics topic is not available." >&2
  exit 1
fi

if ! pgrep -f '[g]azebo_airsim_bridge.py' >/dev/null; then
  echo "[ERROR] Gazebo-to-AirSim bridge is not running." >&2
  echo "        Run 01_prepare_takeoff.sh; it starts the bridge after takeoff." >&2
  exit 1
fi

export PYTHONPATH="$SYSTEM_DIST_PACKAGES:$PYCLIENT${PYTHONPATH:+:$PYTHONPATH}"

"$VENV/bin/python" - <<'PY'
import json
import math
import time
import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
settings = json.loads(client.getSettingsString())
vehicle = settings.get("Vehicles", {}).get("Drone1", {})
if settings.get("PhysicsEngineName") != "ExternalPhysicsEngine":
    raise SystemExit("[ERROR] AirSim is not using ExternalPhysicsEngine")
if not math.isclose(float(vehicle.get("Z", float("nan"))), -500.0, abs_tol=1e-6):
    raise SystemExit("[ERROR] AirSim runtime profile is not the 500 m profile")

first = client.simGetGroundTruthKinematics(vehicle_name="Drone1").position
time.sleep(1.0)
second = client.simGetGroundTruthKinematics(vehicle_name="Drone1").position
distance = math.sqrt(
    (second.x_val - first.x_val) ** 2
    + (second.y_val - first.y_val) ** 2
    + (second.z_val - first.z_val) ** 2
)
altitude_m = 500.0 - float(second.z_val)
if distance < 5.0:
    raise SystemExit(
        f"[ERROR] AirSim aircraft moved only {distance:.2f} m in 1 s; "
        "the Gazebo bridge is not injecting live fixed-wing motion"
    )
if not 450.0 <= altitude_m <= 550.0:
    raise SystemExit(
        f"[ERROR] current local altitude is {altitude_m:.1f} m; "
        "restart the bridge after prepare takeoff so it reanchors at 500 m"
    )
print(
    f"[PREFLIGHT] live bridge motion={distance:.1f} m/s-equivalent, "
    f"local_altitude={altitude_m:.1f} m"
)
PY

recorder_pid=""
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$recorder_pid" ]] && kill -0 "$recorder_pid" 2>/dev/null; then
    kill -INT "$recorder_pid" 2>/dev/null || true
    wait "$recorder_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

"$VENV/bin/python" -u "$RECORDER" \
  --output "$OUTPUT" \
  --duration-sec "$DURATION_SEC" \
  --camera-hz 10 \
  --imu-hz 100 \
  --max-camera-source-gap-ms 300 \
  --max-camera-schedule-jitter-ms 500 \
  --max-camera-gt-skew-ms 200 \
  --start-delay-sec 5 \
  --nominal-altitude-m 500 \
  --altitude-tolerance-m 50 \
  --altitude-reference-ned-z-m 0 \
  --required-turn-bank-deg 5 \
  --max-abs-roll-deg 35 \
  --straight-bank-deg 3 \
  --minimum-straight-fraction 0.10 \
  --ready-file "$READY_FILE" &
recorder_pid=$!

for _ in $(seq 1 600); do
  if [[ -f "$READY_FILE" ]]; then
    break
  fi
  if ! kill -0 "$recorder_pid" 2>/dev/null; then
    wait "$recorder_pid"
    echo "[ERROR] recorder exited before becoming ready" >&2
    exit 1
  fi
  sleep 0.1
done

if [[ ! -f "$READY_FILE" ]]; then
  echo "[ERROR] recorder did not become ready within 60 seconds" >&2
  exit 1
fi

"$VENV/bin/python" -u "$MISSION" dynamic \
  --duration-sec "$DURATION_SEC" \
  --speed-mps "$SPEED_MPS" \
  --bank-limit-deg "$BANK_LIMIT_DEG" \
  --output "$OUTPUT/flight_mission.json"

set +e
wait "$recorder_pid"
recorder_status=$?
set -e
recorder_pid=""
trap - EXIT INT TERM

if [[ "$recorder_status" -ne 0 ]]; then
  echo "[ERROR] dataset validation failed: $OUTPUT/timing_report.json" >&2
  exit "$recorder_status"
fi

echo "[DATASET3_COMPLETE] $OUTPUT"
