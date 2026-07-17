#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 /absolute/path/to/ucc_dataset" >&2
  exit 2
fi

DATASET_ROOT="$(realpath "$1")"
VINS_WS="${VINS_WS:-$HOME/vins_mono_ros2_ws}"
RUNTIME_DIR="$VINS_WS/ucc_runtime"
CONFIG_PATH="$RUNTIME_DIR/ucc_config.yaml"
RESULT_DIR="$VINS_WS/output/ucc_latest"
SUPPORT_PATH="$VINS_WS/src/VINS-MONO-ROS2/config_pkg/support_files"

if [[ ! -d "$DATASET_ROOT" ]]; then
  echo "[ERROR] Dataset directory not found: $DATASET_ROOT" >&2
  exit 1
fi

for relative in \
  mav0/cam0/data.csv \
  mav0/cam0/data \
  mav0/imu0/data.csv
do
  if [[ ! -e "$DATASET_ROOT/$relative" ]]; then
    echo "[ERROR] Missing: $DATASET_ROOT/$relative" >&2
    echo "Expected EuRoC-like layout:" >&2
    echo "  DATASET_ROOT/mav0/cam0/data.csv" >&2
    echo "  DATASET_ROOT/mav0/cam0/data/*.png" >&2
    echo "  DATASET_ROOT/mav0/imu0/data.csv" >&2
    exit 1
  fi
done

mkdir -p "$RUNTIME_DIR"

python3 "$VINS_WS/tools/ucc/generate_ucc_config.py" \
  --dataset-root "$DATASET_ROOT" \
  --output-config "$CONFIG_PATH" \
  --result-dir "$RESULT_DIR" \
  --support-path "$SUPPORT_PATH"

CAMERA_FILES="$(find "$DATASET_ROOT/mav0/cam0/data" -maxdepth 1 -type f -name '*.png' | wc -l)"
CAMERA_ROWS="$(grep -Ev '^[[:space:]]*(#|$)' "$DATASET_ROOT/mav0/cam0/data.csv" | wc -l)"
IMU_ROWS="$(grep -Ev '^[[:space:]]*(#|$)' "$DATASET_ROOT/mav0/imu0/data.csv" | wc -l)"

echo
echo "=== Dataset validation ==="
echo "Camera PNG : $CAMERA_FILES"
echo "Camera CSV : $CAMERA_ROWS"
echo "IMU CSV    : $IMU_ROWS"

if [[ "$CAMERA_FILES" -ne "$CAMERA_ROWS" ]]; then
  echo "[ERROR] Camera PNG count and data.csv rows do not match." >&2
  exit 1
fi

if [[ "$IMU_ROWS" -lt "$((CAMERA_ROWS * 5))" ]]; then
  echo "[WARN] IMU count is unexpectedly low relative to camera count." >&2
fi

cat > "$RUNTIME_DIR/last_dataset.env" <<EOF
DATASET_ROOT='$DATASET_ROOT'
CONFIG_PATH='$CONFIG_PATH'
RESULT_DIR='$RESULT_DIR'
EOF

echo
echo "[OK] Dataset is prepared."
echo "Config : $CONFIG_PATH"
echo "Result : $RESULT_DIR"
echo
echo "Next:"
echo "  ./02_run_vins.sh '$DATASET_ROOT'"
