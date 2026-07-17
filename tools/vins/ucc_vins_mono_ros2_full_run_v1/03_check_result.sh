#!/usr/bin/env bash
set -euo pipefail

VINS_WS="${VINS_WS:-$HOME/vins_mono_ros2_ws}"
RESULT_DIR="$VINS_WS/output/ucc_latest"
RESULT_CSV="$RESULT_DIR/vins_result_no_loop.csv"

if [[ ! -f "$RESULT_CSV" ]]; then
  echo "[ERROR] Result not found: $RESULT_CSV" >&2
  exit 1
fi

echo "=== VINS result ==="
echo "Path : $RESULT_CSV"
echo "Size : $(du -h "$RESULT_CSV" | awk '{print $1}')"
echo "Rows : $(wc -l < "$RESULT_CSV")"
echo
echo "First 5 rows:"
head -n 5 "$RESULT_CSV"
echo
echo "Last 5 rows:"
tail -n 5 "$RESULT_CSV"

echo
echo "Latest VINS log:"
LATEST_LOG="$(find "$VINS_WS/logs" -maxdepth 1 -type f -name 'ucc_vins_*.log' | sort | tail -n 1)"
if [[ -n "$LATEST_LOG" ]]; then
  echo "$LATEST_LOG"
  grep -Ei \
    'initial|failure|error|warn|feature|result path|waiting for image and imu' \
    "$LATEST_LOG" | tail -n 50 || true
fi
