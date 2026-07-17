#!/usr/bin/env bash
set -Eeuo pipefail

REPO="${REPO:-$HOME/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator}"
PROJECT="${PROJECT:-$REPO/sim/UCCVioDatasetSim}"
UPROJECT="${UPROJECT:-$PROJECT/UCCVioDatasetSim.uproject}"
CESIUM_ZIP="${CESIUM_ZIP:-$HOME/vio_sim_ws/downloads/cesium/CesiumForUnreal-56-main.zip}"

PLUGIN_PARENT="$PROJECT/Plugins"
PLUGIN_DEST="$PLUGIN_PARENT/CesiumForUnreal"

STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP_ROOT="$HOME/vio_sim_ws/backups/ucc_cesium_$STAMP"

fail() {
    echo "[ERROR] $*" >&2
    exit 1
}

cleanup() {
    if [[ -n "${TMP_DIR:-}" && -d "$TMP_DIR" ]]; then
        rm -rf "$TMP_DIR"
    fi
}
trap cleanup EXIT

[[ -d "$REPO/.git" ]] || fail "Git repository가 없습니다: $REPO"
[[ -d "$PROJECT" ]] || fail "Unreal project가 없습니다: $PROJECT"
[[ -f "$UPROJECT" ]] || fail ".uproject가 없습니다: $UPROJECT"
[[ -f "$CESIUM_ZIP" ]] || fail "Cesium ZIP이 없습니다: $CESIUM_ZIP"

echo "========== Cesium ZIP 검증 =========="

mapfile -t DESCRIPTORS < <(
    unzip -Z1 "$CESIUM_ZIP" |
    grep -E '(^|/)CesiumForUnreal\.uplugin$' || true
)

if (( ${#DESCRIPTORS[@]} != 1 )); then
    printf '[ERROR] CesiumForUnreal.uplugin 탐색 결과: %d개\n' \
        "${#DESCRIPTORS[@]}" >&2
    printf '%s\n' "${DESCRIPTORS[@]:-}" >&2
    exit 1
fi

DESCRIPTOR_PATH="${DESCRIPTORS[0]}"
PLUGIN_ROOT="${DESCRIPTOR_PATH%/CesiumForUnreal.uplugin}"

echo "ZIP          : $CESIUM_ZIP"
echo "SHA256       : $(sha256sum "$CESIUM_ZIP" | awk '{print $1}')"
echo "Plugin root  : $PLUGIN_ROOT"
echo "Descriptor   : $DESCRIPTOR_PATH"

TMP_DIR="$(mktemp -d)"

echo
echo "========== Plugin 압축 해제 =========="

unzip -q "$CESIUM_ZIP" "${PLUGIN_ROOT}/*" -d "$TMP_DIR"

EXTRACTED_ROOT="$TMP_DIR/$PLUGIN_ROOT"
EXTRACTED_DESCRIPTOR="$EXTRACTED_ROOT/CesiumForUnreal.uplugin"

[[ -f "$EXTRACTED_DESCRIPTOR" ]] ||
    fail "압축 해제 후 descriptor가 없습니다: $EXTRACTED_DESCRIPTOR"

[[ -d "$EXTRACTED_ROOT/Source/CesiumRuntime" ]] ||
    fail "CesiumRuntime source가 없습니다."

[[ -d "$EXTRACTED_ROOT/Source/CesiumEditor" ]] ||
    fail "CesiumEditor source가 없습니다."

mkdir -p "$PLUGIN_PARENT"
mkdir -p "$BACKUP_ROOT"

if [[ -e "$PLUGIN_DEST" ]]; then
    echo "[INFO] 기존 Plugin을 백업합니다."
    mv "$PLUGIN_DEST" "$BACKUP_ROOT/CesiumForUnreal"
fi

mv "$EXTRACTED_ROOT" "$PLUGIN_DEST"

[[ -f "$PLUGIN_DEST/CesiumForUnreal.uplugin" ]] ||
    fail "최종 Plugin 설치에 실패했습니다."

echo "[OK] Plugin 설치 완료:"
echo "     $PLUGIN_DEST"

echo
echo "========== Descriptor 확인 =========="

python3 - "$PLUGIN_DEST/CesiumForUnreal.uplugin" <<'PY'
import json
import sys
from pathlib import Path

descriptor_path = Path(sys.argv[1])

with descriptor_path.open("r", encoding="utf-8") as file:
    descriptor = json.load(file)

print(f"FriendlyName   : {descriptor.get('FriendlyName')}")
print(f"Version        : {descriptor.get('Version')}")
print(f"VersionName    : {descriptor.get('VersionName')}")
print(f"EngineVersion  : {descriptor.get('EngineVersion')}")
print(f"CanContainContent: {descriptor.get('CanContainContent')}")

modules = descriptor.get("Modules", [])
print("Modules:")
for module in modules:
    print(
        f"  - {module.get('Name')} "
        f"(Type={module.get('Type')}, "
        f"LoadingPhase={module.get('LoadingPhase')})"
    )
PY

echo
echo "========== 설치 결과 =========="

find "$PLUGIN_DEST" \
    -maxdepth 2 \
    -type f \
    \( -name '*.uplugin' -o -name '*.Build.cs' \) \
    -print | sort

echo
echo "[OK] Cesium Plugin 설치가 완료되었습니다."
echo "[INFO] 기존 파일 백업 경로: $BACKUP_ROOT"
