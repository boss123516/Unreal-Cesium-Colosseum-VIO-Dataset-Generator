#!/usr/bin/env bash
set -Eeuo pipefail

REPO="${REPO:-$HOME/research/Unreal-Cesium-Colosseum-VIO-Dataset-Generator}"
PROJECT="${PROJECT:-$REPO/sim/UCCVioDatasetSim}"
UPROJECT="${UPROJECT:-$PROJECT/UCCVioDatasetSim.uproject}"
CESIUM_ZIP="${CESIUM_ZIP:-$HOME/vio_sim_ws/downloads/cesium/CesiumForUnreal-56-main.zip}"
UE_ROOT="${UE_ROOT:-$HOME/vio_sim_ws/UE_5.6}"

PLUGIN_PARENT="$PROJECT/Plugins"
PLUGIN_DEST="$PLUGIN_PARENT/CesiumForUnreal"
INSTALL_SCRIPT="$REPO/scripts/setup/install_cesium_plugin.sh"
STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP_ROOT="$HOME/vio_sim_ws/backups/ucc_cesium_$STAMP"
BUILD_LOG="$HOME/vio_sim_ws/ucc_cesium_build.log"

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

echo "========== 1. 경로 검증 =========="

[[ -d "$REPO/.git" ]] || fail "Git repository가 없습니다: $REPO"
[[ -d "$PROJECT" ]] || fail "Unreal project가 없습니다: $PROJECT"
[[ -f "$UPROJECT" ]] || fail ".uproject가 없습니다: $UPROJECT"
[[ -f "$CESIUM_ZIP" ]] || fail "Cesium ZIP이 없습니다: $CESIUM_ZIP"
[[ -x "$UE_ROOT/Engine/Build/BatchFiles/Linux/Build.sh" ]] ||
    fail "Unreal Build.sh를 찾을 수 없습니다: $UE_ROOT"

echo "REPO       : $REPO"
echo "PROJECT    : $PROJECT"
echo "CESIUM ZIP : $CESIUM_ZIP"
echo "UE ROOT    : $UE_ROOT"

echo
echo "========== 2. Cesium ZIP 검증 =========="

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

echo "SHA256      : $(sha256sum "$CESIUM_ZIP" | awk '{print $1}')"
echo "Plugin root : $PLUGIN_ROOT"
echo "Descriptor  : $DESCRIPTOR_PATH"

echo
echo "========== 3. Plugin 압축 해제 및 설치 =========="

TMP_DIR="$(mktemp -d)"
unzip -q "$CESIUM_ZIP" "${PLUGIN_ROOT}/*" -d "$TMP_DIR"

EXTRACTED_ROOT="$TMP_DIR/$PLUGIN_ROOT"
EXTRACTED_DESCRIPTOR="$EXTRACTED_ROOT/CesiumForUnreal.uplugin"

[[ -f "$EXTRACTED_DESCRIPTOR" ]] ||
    fail "압축 해제 후 descriptor가 없습니다: $EXTRACTED_DESCRIPTOR"

[[ -d "$EXTRACTED_ROOT/Source/CesiumRuntime" ]] ||
    fail "CesiumRuntime source가 없습니다."

[[ -d "$EXTRACTED_ROOT/Source/CesiumEditor" ]] ||
    fail "CesiumEditor source가 없습니다."

mkdir -p "$PLUGIN_PARENT" "$BACKUP_ROOT"

if [[ -e "$PLUGIN_DEST" ]]; then
    echo "[INFO] 기존 Plugin을 백업합니다."
    mv "$PLUGIN_DEST" "$BACKUP_ROOT/CesiumForUnreal"
fi

mv "$EXTRACTED_ROOT" "$PLUGIN_DEST"

[[ -f "$PLUGIN_DEST/CesiumForUnreal.uplugin" ]] ||
    fail "최종 Plugin 설치에 실패했습니다."

echo "[OK] Plugin 설치 완료: $PLUGIN_DEST"

echo
echo "========== 4. Plugin descriptor 확인 =========="

python3 - "$PLUGIN_DEST/CesiumForUnreal.uplugin" <<'PY'
import json
import sys
from pathlib import Path

descriptor_path = Path(sys.argv[1])

with descriptor_path.open("r", encoding="utf-8") as file:
    descriptor = json.load(file)

print(f"FriendlyName      : {descriptor.get('FriendlyName')}")
print(f"Version           : {descriptor.get('Version')}")
print(f"VersionName       : {descriptor.get('VersionName')}")
print(f"EngineVersion     : {descriptor.get('EngineVersion')}")
print(f"CanContainContent : {descriptor.get('CanContainContent')}")
print("Modules:")

for module in descriptor.get("Modules", []):
    print(
        f"  - {module.get('Name')} "
        f"(Type={module.get('Type')}, "
        f"LoadingPhase={module.get('LoadingPhase')})"
    )
PY

echo
echo "========== 5. Git ignore 적용 =========="

IGNORE_RULE='sim/UCCVioDatasetSim/Plugins/CesiumForUnreal/'

touch "$REPO/.gitignore"

if ! grep -Fxq "$IGNORE_RULE" "$REPO/.gitignore"; then
    {
        printf '\n# External Unreal plugins\n'
        printf '%s\n' "$IGNORE_RULE"
    } >> "$REPO/.gitignore"
fi

git -C "$REPO" check-ignore -q \
    "$PLUGIN_DEST/CesiumForUnreal.uplugin" ||
    fail "Cesium Plugin 경로가 Git에서 제외되지 않았습니다."

echo "[OK] Git ignore 적용 완료"

echo
echo "========== 6. .uproject Plugin 활성화 =========="

cp "$UPROJECT" "$BACKUP_ROOT/UCCVioDatasetSim.uproject"

python3 - "$UPROJECT" <<'PY'
import json
import sys
from pathlib import Path

uproject = Path(sys.argv[1])
project = json.loads(uproject.read_text(encoding="utf-8"))
plugins = project.setdefault("Plugins", [])

entry = next(
    (plugin for plugin in plugins if plugin.get("Name") == "CesiumForUnreal"),
    None,
)

if entry is None:
    plugins.append({
        "Name": "CesiumForUnreal",
        "Enabled": True,
    })
else:
    entry["Enabled"] = True

uproject.write_text(
    json.dumps(project, indent=4) + "\n",
    encoding="utf-8",
)
PY

echo "[OK] CesiumForUnreal enabled in $UPROJECT"

echo
echo "========== 7. 프로젝트 빌드 =========="

"$UE_ROOT/Engine/Build/BatchFiles/Linux/Build.sh" \
    UCCVioDatasetSimEditor \
    Linux \
    Development \
    -Project="$UPROJECT" \
    -WaitMutex \
    -NoHotReloadFromIDE \
    2>&1 | tee "$BUILD_LOG"

echo
echo "========== 8. 최종 확인 =========="

[[ -f "$PLUGIN_DEST/CesiumForUnreal.uplugin" ]] ||
    fail "Plugin descriptor가 없습니다."

python3 - "$UPROJECT" <<'PY'
import json
import sys
from pathlib import Path

uproject = Path(sys.argv[1])
project = json.loads(uproject.read_text(encoding="utf-8"))

entry = next(
    (
        plugin
        for plugin in project.get("Plugins", [])
        if plugin.get("Name") == "CesiumForUnreal"
    ),
    None,
)

if not entry or entry.get("Enabled") is not True:
    raise SystemExit("[ERROR] CesiumForUnreal 활성화를 확인하지 못했습니다.")

print("[OK] .uproject Plugin 활성화 확인")
PY

echo
echo "Git 상태:"
git -C "$REPO" status --short

echo
echo "=============================================="
echo "[SUCCESS] Cesium 설치 및 프로젝트 빌드 완료"
echo "Build log : $BUILD_LOG"
echo "Backup    : $BACKUP_ROOT"
echo "=============================================="
