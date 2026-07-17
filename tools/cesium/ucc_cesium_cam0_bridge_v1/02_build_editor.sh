#!/usr/bin/env bash
set -euo pipefail

DEFAULT_PROJECT="$HOME/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator/sim/UCCVioDatasetSim"
DEFAULT_UE="$HOME/vio_sim_ws/UE_5.6"

PROJECT_ROOT="${1:-$DEFAULT_PROJECT}"
UE_ROOT="${UE_ROOT:-$DEFAULT_UE}"

UPROJECT="$PROJECT_ROOT/UCCVioDatasetSim.uproject"
BUILD_SH="$UE_ROOT/Engine/Build/BatchFiles/Linux/Build.sh"
LOG_DIR="$HOME/vio_sim_ws/logs"
LOG_PATH="$LOG_DIR/ucc_cesium_cam0_bridge_build_$(date +%Y%m%d_%H%M%S).log"

if [[ ! -x "$BUILD_SH" ]]; then
  echo "[ERROR] UE build script not found: $BUILD_SH" >&2
  exit 1
fi

if [[ ! -f "$UPROJECT" ]]; then
  echo "[ERROR] Project not found: $UPROJECT" >&2
  exit 1
fi

if pgrep -f 'UnrealEditor.*UCCVioDatasetSim' >/dev/null 2>&1; then
  echo "[ERROR] UnrealEditor is still running for this project." >&2
  echo "        Stop PIE and close the editor before building." >&2
  exit 1
fi

mkdir -p "$LOG_DIR"

echo "[BUILD] UCCVioDatasetSimEditor"
echo "[LOG]   $LOG_PATH"

set +e
"$BUILD_SH" \
  UCCVioDatasetSimEditor \
  Linux \
  Development \
  -Project="$UPROJECT" \
  -WaitMutex \
  2>&1 | tee "$LOG_PATH"
status=${PIPESTATUS[0]}
set -e

if [[ $status -ne 0 ]]; then
  echo "[ERROR] Build failed. Log: $LOG_PATH" >&2
  exit "$status"
fi

if ! grep -Eq 'Result:[[:space:]]+Succeeded|BUILD SUCCESSFUL' "$LOG_PATH"; then
  echo "[WARN] Build command returned success, but the usual success marker was not found."
fi

echo
echo "[OK] Build finished."
echo "Start UnrealEditor, open HighAltitudeCity, then press Play."
echo "Watch Output Log for:"
echo "  [CESIUM_CAM0_BRIDGE] READY"
