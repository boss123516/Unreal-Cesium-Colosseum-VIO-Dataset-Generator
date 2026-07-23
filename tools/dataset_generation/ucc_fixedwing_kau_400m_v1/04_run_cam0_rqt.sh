#!/usr/bin/env bash
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${UCC_WORKSPACE:-$HOME/vio_sim_ws}"
VENV="${AIRSIM_VENV:-$WORKSPACE/airsim_pyenv}"
PYCLIENT="${AIRSIM_PYCLIENT:-$WORKSPACE/Colosseum/PythonClient}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
ROS_SITE_PACKAGES="${ROS_SITE_PACKAGES:-/opt/ros/jazzy/lib/python3.12/site-packages}"
SYSTEM_DIST_PACKAGES="${SYSTEM_DIST_PACKAGES:-/usr/lib/python3/dist-packages}"
IMAGE_TOPIC="${CAM0_IMAGE_TOPIC:-/ucc/cam0/image_raw}"
VIEW_HZ="${CAM0_VIEW_HZ:-5}"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "[ERROR] ROS 2 setup not found: $ROS_SETUP" >&2
  exit 1
fi

if [[ ! -x "$VENV/bin/python" ]]; then
  echo "[ERROR] AirSim Python environment not found: $VENV" >&2
  exit 1
fi

if ! ss -ltn 2>/dev/null | grep -q ':41451'; then
  echo "[ERROR] AirSim RPC port 41451 is not open." >&2
  echo "        Start Unreal Play/PIE first." >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$VENV/bin/activate"
export PYTHONPATH="$ROS_SITE_PACKAGES:$SYSTEM_DIST_PACKAGES:$PYCLIENT${PYTHONPATH:+:$PYTHONPATH}"

publisher_pid=""
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$publisher_pid" ]] && kill -0 "$publisher_pid" 2>/dev/null; then
    kill -INT "$publisher_pid" 2>/dev/null || true
    wait "$publisher_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

"$VENV/bin/python" -u "$SCRIPT_DIR/airsim_cam0_ros2.py" \
  --hz "$VIEW_HZ" \
  --image-topic "$IMAGE_TOPIC" &
publisher_pid=$!

for _ in $(seq 1 100); do
  if ros2 topic list 2>/dev/null | grep -qx "$IMAGE_TOPIC"; then
    break
  fi
  if ! kill -0 "$publisher_pid" 2>/dev/null; then
    wait "$publisher_pid"
    echo "[ERROR] cam0 ROS 2 publisher exited before the topic was ready." >&2
    exit 1
  fi
  sleep 0.1
done

if ! ros2 topic list 2>/dev/null | grep -qx "$IMAGE_TOPIC"; then
  echo "[ERROR] ROS 2 image topic was not ready within 10 seconds." >&2
  exit 1
fi

echo "[CAM0_RQT_READY] topic=$IMAGE_TOPIC rate=${VIEW_HZ}Hz"
ros2 run rqt_image_view rqt_image_view --on-top "$IMAGE_TOPIC"

trap - EXIT INT TERM
cleanup
