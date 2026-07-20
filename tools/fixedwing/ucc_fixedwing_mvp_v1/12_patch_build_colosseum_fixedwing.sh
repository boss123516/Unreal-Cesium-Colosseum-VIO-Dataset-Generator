#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
PROJECT_ROOT="${UCC_PROJECT_ROOT:-$REPO_ROOT/sim/UCCVioDatasetSim}"
UE_ROOT="${UE_ROOT:-$HOME/vio_sim_ws/UE_5.6}"
PROJECT="$PROJECT_ROOT/UCCVioDatasetSim.uproject"
BUILD_SCRIPT="$UE_ROOT/Engine/Build/BatchFiles/Linux/Build.sh"
PATCHER="$SCRIPT_DIR/apply_colosseum_fixedwing_patch.py"

for path in "$PROJECT" "$BUILD_SCRIPT" "$PATCHER"; do
  if [[ ! -e "$path" ]]; then
    echo "[ERROR] required path not found: $path" >&2
    exit 1
  fi
done

if pgrep -f 'UnrealEditor.*UCCVioDatasetSim' >/dev/null 2>&1; then
  echo "[ERROR] stop UnrealEditor before patching the AirSim plugin" >&2
  exit 1
fi

python3 "$PATCHER" --project "$PROJECT_ROOT"
"$BUILD_SCRIPT" UCCVioDatasetSimEditor Linux Development "$PROJECT" -WaitMutex
echo "[OK] Colosseum fixed-wing runtime patch built successfully"
