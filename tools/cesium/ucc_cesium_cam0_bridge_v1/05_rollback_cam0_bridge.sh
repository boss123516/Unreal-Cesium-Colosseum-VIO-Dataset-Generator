#!/usr/bin/env bash
set -euo pipefail

DEFAULT_PROJECT="$HOME/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator/sim/UCCVioDatasetSim"
PROJECT_ROOT="${1:-$DEFAULT_PROJECT}"
MODULE_DIR="$PROJECT_ROOT/Source/UCCVioDatasetSim"
BUILD_CS="$MODULE_DIR/UCCVioDatasetSim.Build.cs"

rm -f \
  "$MODULE_DIR/CesiumAirSimCameraBridgeSubsystem.h" \
  "$MODULE_DIR/CesiumAirSimCameraBridgeSubsystem.cpp"

python3 - "$BUILD_CS" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
text = re.sub(
    r'\s*PrivateDependencyModuleNames\.AddRange\(\s*'
    r'new\s+string\[\]\s*\{\s*"AirSim"\s*,\s*"CesiumRuntime"\s*\}\s*\);\s*',
    '\n',
    text,
    count=1,
)
path.write_text(text, encoding="utf-8")
print("[OK] Removed bridge dependencies from Build.cs.")
PY

echo "[OK] Bridge source removed. Rebuild the editor."
