#!/usr/bin/env bash
set -euo pipefail

LOG_ROOT="${1:-$HOME/vio_sim_ws/UE_5.6/Engine/Saved/Logs}"

echo "Searching for CESIUM_CAM0_BRIDGE messages..."
grep -Rhs --include='*.log' '\[CESIUM_CAM0_BRIDGE\]' \
  "$LOG_ROOT" \
  "$HOME/.config/Epic" \
  2>/dev/null | tail -n 50 || true

echo
echo "Required line:"
echo "  [CESIUM_CAM0_BRIDGE] READY"
