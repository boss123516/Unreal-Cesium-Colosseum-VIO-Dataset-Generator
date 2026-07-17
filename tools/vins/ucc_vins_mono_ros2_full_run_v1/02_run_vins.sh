#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 /absolute/path/to/ucc_dataset" >&2
  exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DATASET_ROOT="$(realpath "$1")"
VINS_WS="${VINS_WS:-$HOME/vins_mono_ros2_ws}"
ROS_DISTRO_EXPECTED="${ROS_DISTRO_EXPECTED:-humble}"
RUNTIME_DIR="$VINS_WS/ucc_runtime"
CONFIG_PATH="$RUNTIME_DIR/ucc_config.yaml"
RESULT_DIR="$VINS_WS/output/ucc_latest"
LOG_DIR="$VINS_WS/logs"
LOG_PATH="$LOG_DIR/ucc_vins_$(date +%Y%m%d_%H%M%S).log"
USE_RVIZ="${USE_RVIZ:-true}"
PLAYBACK_RATE="${PLAYBACK_RATE:-1.0}"

"$SCRIPT_DIR/01_validate_prepare_dataset.sh" "$DATASET_ROOT"

# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO_EXPECTED/setup.bash"
# shellcheck disable=SC1090
source "$VINS_WS/install/setup.bash"

mkdir -p "$LOG_DIR"

if [[ -d "$RESULT_DIR" ]] && find "$RESULT_DIR" -mindepth 1 -print -quit | grep -q .; then
  BACKUP_RESULT="${RESULT_DIR}_backup_$(date +%Y%m%d_%H%M%S)"
  mv "$RESULT_DIR" "$BACKUP_RESULT"
  echo "[BACKUP] Previous result moved to: $BACKUP_RESULT"
fi
mkdir -p "$RESULT_DIR"

LAUNCH_PID=""

cleanup() {
  local status=$?

  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[STOP] Stopping VINS launch group..."
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true

    for _ in $(seq 1 50); do
      if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done

    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
      kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
    fi
  fi

  exit "$status"
}
trap cleanup EXIT INT TERM

echo "=== Start VINS ==="
echo "Config : $CONFIG_PATH"
echo "Log    : $LOG_PATH"
echo "RViz   : $USE_RVIZ"

setsid ros2 launch vins_estimator ucc_vins.launch.py \
  config_file:="$CONFIG_PATH" \
  use_rviz:="$USE_RVIZ" \
  >"$LOG_PATH" 2>&1 &
LAUNCH_PID=$!

sleep 5

if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
  echo "[ERROR] VINS launch exited early." >&2
  tail -n 100 "$LOG_PATH" >&2
  exit 1
fi

echo "=== Play Camera + IMU ==="
python3 "$VINS_WS/tools/ucc/ucc_dataset_player.py" \
  --dataset-root "$DATASET_ROOT" \
  --rate-scale "$PLAYBACK_RATE"

echo "[INFO] Playback complete. Waiting for estimator flush..."
sleep 5

kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true
LAUNCH_PID=""

RESULT_CSV="$RESULT_DIR/vins_result_no_loop.csv"

echo
echo "=== Result ==="
echo "CSV : $RESULT_CSV"
echo "Log : $LOG_PATH"

if [[ ! -s "$RESULT_CSV" ]]; then
  echo "[ERROR] VINS result CSV is missing or empty." >&2
  echo >&2
  echo "Last VINS log lines:" >&2
  tail -n 120 "$LOG_PATH" >&2
  exit 3
fi

RESULT_LINES="$(wc -l < "$RESULT_CSV")"
echo "Rows: $RESULT_LINES"

if [[ "$RESULT_LINES" -lt 5 ]]; then
  echo "[WARN] Very few trajectory rows were generated."
  echo "       Check feature count, initialization motion, intrinsics, and extrinsics."
fi

echo
echo "[OK] VINS run finished."
echo "Open the trajectory CSV with:"
echo "  head '$RESULT_CSV'"
