#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE_ROOT="${WORKSPACE_ROOT:-$HOME/vio_sim_ws}"

CESIUM_VERSION="v2.28.0"
CESIUM_DOWNLOAD_DIR="$WORKSPACE_ROOT/downloads/cesium"

COLOSSEUM_VERSION="v2.3.0"
COLOSSEUM_ROOT="$WORKSPACE_ROOT/Colosseum"

fail() {
    echo "[ERROR] $*" >&2
    exit 1
}

for command_name in git gh; do
    command -v "$command_name" >/dev/null 2>&1 \
        || fail "Missing command: $command_name"
done

mkdir -p "$CESIUM_DOWNLOAD_DIR"

echo "=================================================="
echo " Fetch external simulator dependencies"
echo "=================================================="

echo
echo "[1/4] Resolving Cesium for Unreal UE 5.6 package"

mapfile -t CESIUM_ASSETS < <(
    gh release view "$CESIUM_VERSION" \
        --repo CesiumGS/cesium-unreal \
        --json assets \
        --jq '.assets[].name'
)

if [[ "${#CESIUM_ASSETS[@]}" -eq 0 ]]; then
    fail "No assets found in Cesium release $CESIUM_VERSION"
fi

CESIUM_ASSET=""
for asset in "${CESIUM_ASSETS[@]}"; do
    if [[ "$asset" == *.zip && "$asset" == *"-56-"* ]]; then
        CESIUM_ASSET="$asset"
        break
    fi
done

if [[ -z "$CESIUM_ASSET" ]]; then
    echo "[ERROR] Could not find a UE 5.6 ZIP in release $CESIUM_VERSION." >&2
    echo "Available assets:" >&2
    printf '  %s\n' "${CESIUM_ASSETS[@]}" >&2
    exit 1
fi

CESIUM_ZIP="$CESIUM_DOWNLOAD_DIR/$CESIUM_ASSET"

echo "[OK] Selected asset: $CESIUM_ASSET"

echo
echo "[2/4] Downloading Cesium for Unreal"

if [[ -s "$CESIUM_ZIP" ]]; then
    echo "[SKIP] Already downloaded: $CESIUM_ZIP"
else
    gh release download "$CESIUM_VERSION" \
        --repo CesiumGS/cesium-unreal \
        --pattern "$CESIUM_ASSET" \
        --dir "$CESIUM_DOWNLOAD_DIR"
fi

[[ -s "$CESIUM_ZIP" ]] || fail "Cesium ZIP download failed"
ls -lh "$CESIUM_ZIP"

echo
echo "[3/4] Cloning Colosseum"

if [[ -d "$COLOSSEUM_ROOT/.git" ]]; then
    echo "[SKIP] Already cloned: $COLOSSEUM_ROOT"
else
    git clone \
        --branch "$COLOSSEUM_VERSION" \
        --single-branch \
        --depth 1 \
        https://github.com/CodexLabsLLC/Colosseum.git \
        "$COLOSSEUM_ROOT"
fi

echo
echo "[4/4] Verifying dependencies"

echo "Cesium:"
echo "  version: $CESIUM_VERSION"
echo "  asset:   $CESIUM_ASSET"
echo "  path:    $CESIUM_ZIP"

echo
echo "Colosseum:"
git -C "$COLOSSEUM_ROOT" branch --show-current
git -C "$COLOSSEUM_ROOT" log -1 --oneline

echo
echo "=================================================="
echo " Dependency fetch completed"
echo "=================================================="
echo "No Cesium extraction or Colosseum build was started."
