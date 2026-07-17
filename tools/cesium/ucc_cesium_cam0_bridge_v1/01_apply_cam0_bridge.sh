#!/usr/bin/env bash
set -euo pipefail

DEFAULT_PROJECT="$HOME/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator/sim/UCCVioDatasetSim"
PROJECT_ROOT="${1:-$DEFAULT_PROJECT}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

MODULE_DIR="$PROJECT_ROOT/Source/UCCVioDatasetSim"
BUILD_CS="$MODULE_DIR/UCCVioDatasetSim.Build.cs"
UPROJECT="$PROJECT_ROOT/UCCVioDatasetSim.uproject"

if [[ ! -f "$UPROJECT" ]]; then
  echo "[ERROR] Unreal project not found: $UPROJECT" >&2
  echo "Usage: $0 /absolute/path/to/UCCVioDatasetSim" >&2
  exit 1
fi

if [[ ! -f "$BUILD_CS" ]]; then
  echo "[ERROR] Module Build.cs not found: $BUILD_CS" >&2
  exit 1
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP_ROOT="$PROJECT_ROOT/.backup_cesium_cam0_bridge_$STAMP"
mkdir -p "$BACKUP_ROOT"

cp -a "$BUILD_CS" "$BACKUP_ROOT/"
for name in CesiumAirSimCameraBridgeSubsystem.h CesiumAirSimCameraBridgeSubsystem.cpp; do
  if [[ -f "$MODULE_DIR/$name" ]]; then
    cp -a "$MODULE_DIR/$name" "$BACKUP_ROOT/"
  fi
done

python3 - "$BUILD_CS" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")

if '"AirSim"' in text and '"CesiumRuntime"' in text:
    print("[OK] Build.cs already contains AirSim and CesiumRuntime dependencies.")
    raise SystemExit(0)

pattern = re.compile(
    r'(public\s+UCCVioDatasetSim\s*\(\s*ReadOnlyTargetRules\s+Target\s*\)'
    r'\s*:\s*base\s*\(\s*Target\s*\)\s*\{)',
    re.MULTILINE,
)

match = pattern.search(text)
if not match:
    raise SystemExit(
        "[ERROR] Could not find UCCVioDatasetSim constructor in Build.cs."
    )

injection = (
    match.group(1)
    + '\n\t\tPrivateDependencyModuleNames.AddRange('
      'new string[] { "AirSim", "CesiumRuntime" });'
)

text = text[:match.start()] + injection + text[match.end():]
path.write_text(text, encoding="utf-8")
print("[OK] Added AirSim and CesiumRuntime module dependencies.")
PY

install -m 0644 \
  "$SCRIPT_DIR/SourcePatch/CesiumAirSimCameraBridgeSubsystem.h" \
  "$MODULE_DIR/CesiumAirSimCameraBridgeSubsystem.h"

install -m 0644 \
  "$SCRIPT_DIR/SourcePatch/CesiumAirSimCameraBridgeSubsystem.cpp" \
  "$MODULE_DIR/CesiumAirSimCameraBridgeSubsystem.cpp"

echo
echo "[OK] cam0 bridge source installed."
echo "[BACKUP] $BACKUP_ROOT"
echo
echo "Next:"
echo "  $SCRIPT_DIR/02_build_editor.sh \"$PROJECT_ROOT\""
