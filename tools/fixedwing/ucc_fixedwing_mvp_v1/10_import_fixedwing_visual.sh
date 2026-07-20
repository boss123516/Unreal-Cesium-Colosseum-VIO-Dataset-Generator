#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
PROJECT_ROOT="${UCC_PROJECT_ROOT:-$REPO_ROOT/sim/UCCVioDatasetSim}"
UE_ROOT="${UE_ROOT:-$HOME/vio_sim_ws/UE_5.6}"
EDITOR_CMD="$UE_ROOT/Engine/Binaries/Linux/UnrealEditor-Cmd"
PROJECT="$PROJECT_ROOT/UCCVioDatasetSim.uproject"
SOURCE_FBX="$SCRIPT_DIR/assets/rc_cessna_body.fbx"
IMPORT_SCRIPT="$SCRIPT_DIR/import_fixedwing_visual.py"
LOG_PATH="${UCC_FIXEDWING_IMPORT_LOG:-$HOME/vio_sim_ws/logs/fixedwing_visual_import.log}"
ASSET_FILE="$PROJECT_ROOT/Content/FixedWing/SM_RCCessna.uasset"

for path in "$EDITOR_CMD" "$PROJECT" "$SOURCE_FBX" "$IMPORT_SCRIPT"; do
  if [[ ! -e "$path" ]]; then
    echo "[ERROR] required path not found: $path" >&2
    exit 1
  fi
done

if pgrep -f 'UnrealEditor.*UCCVioDatasetSim' >/dev/null 2>&1; then
  echo "[ERROR] UnrealEditor is running for UCCVioDatasetSim" >&2
  exit 1
fi

mkdir -p "$(dirname -- "$LOG_PATH")"
export UCC_FIXEDWING_FBX="$SOURCE_FBX"

"$EDITOR_CMD" \
  "$PROJECT" \
  -run=pythonscript \
  -script="$IMPORT_SCRIPT" \
  -unattended \
  -nop4 \
  -nosplash \
  -nullrhi \
  >"$LOG_PATH" 2>&1

if [[ ! -f "$ASSET_FILE" ]]; then
  echo "[ERROR] fixed-wing visual asset missing after import: $ASSET_FILE" >&2
  exit 1
fi

echo "[OK] fixed-wing visual imported: /Game/FixedWing/SM_RCCessna"
