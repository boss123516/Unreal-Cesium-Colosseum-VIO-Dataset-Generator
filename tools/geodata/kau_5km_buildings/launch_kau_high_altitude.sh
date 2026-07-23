#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
WORKSPACE_ROOT="${UCC_WORKSPACE:-$HOME/vio_sim_ws}"
UE_ROOT="${UE_ROOT:-$WORKSPACE_ROOT/UE_5.6}"
UNREAL_EDITOR="$UE_ROOT/Engine/Binaries/Linux/UnrealEditor"
PROJECT_FILE="$REPO_ROOT/sim/UCCVioDatasetSim/UCCVioDatasetSim.uproject"
GEODATA_DIR="${KAU_GEODATA_DIR:-$SCRIPT_DIR}"
LEGACY_GEODATA_DIR="$WORKSPACE_ROOT/geodata/kau_5km_buildings"
TILESET_RELATIVE_PATH="output/tiles3d/tileset.json"
TILESET_URL="http://127.0.0.1:8765/tileset.json"

if [[ ! -f "$GEODATA_DIR/$TILESET_RELATIVE_PATH" &&
      -f "$LEGACY_GEODATA_DIR/$TILESET_RELATIVE_PATH" ]]; then
    GEODATA_DIR="$LEGACY_GEODATA_DIR"
fi

if [[ ! -x "$UNREAL_EDITOR" ]]; then
    echo "[ERROR] UnrealEditor를 찾을 수 없습니다: $UNREAL_EDITOR" >&2
    exit 1
fi

if [[ ! -f "$PROJECT_FILE" ]]; then
    echo "[ERROR] Unreal 프로젝트를 찾을 수 없습니다: $PROJECT_FILE" >&2
    exit 1
fi

if [[ ! -f "$GEODATA_DIR/$TILESET_RELATIVE_PATH" ]]; then
    echo "[ERROR] 생성된 tileset.json을 찾을 수 없습니다." >&2
    echo "        expected: $GEODATA_DIR/$TILESET_RELATIVE_PATH" >&2
    echo "        KAU_GEODATA_DIR로 데이터 디렉터리를 지정할 수 있습니다." >&2
    exit 1
fi

python3 "$SCRIPT_DIR/serve_tiles.py" \
    --directory "$GEODATA_DIR/output/tiles3d" &
TILE_SERVER_PID=$!

cleanup() {
    if kill -0 "$TILE_SERVER_PID" 2>/dev/null; then
        kill -TERM "$TILE_SERVER_PID" 2>/dev/null || true
        wait "$TILE_SERVER_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

for _ in {1..50}; do
    if curl --silent --fail --output /dev/null "$TILESET_URL"; then
        break
    fi
    sleep 0.1
done

if ! curl --silent --fail --output /dev/null "$TILESET_URL"; then
    echo "[ERROR] 로컬 3D Tiles 서버가 준비되지 않았습니다: $TILESET_URL" >&2
    exit 1
fi

echo "[KAU_LAUNCHER] tiles=$GEODATA_DIR/output/tiles3d"
echo "[KAU_LAUNCHER] project=$PROJECT_FILE"
"$UNREAL_EDITOR" \
    "$PROJECT_FILE" \
    /Game/Maps/HighAltitudeCity \
    "$@"
