#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE_ROOT="${WORKSPACE_ROOT:-$HOME/vio_sim_ws}"

CESIUM_VERSION="v2.28.0"
CESIUM_ASSET="CesiumForUnreal-56-v2.28.0.zip"
CESIUM_DOWNLOAD_DIR="$WORKSPACE_ROOT/downloads/cesium"
CESIUM_ZIP="$CESIUM_DOWNLOAD_DIR/$CESIUM_ASSET"

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
echo "[1/4] Downloading Cesium for Unreal"

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
echo "[2/4] Cloning Colosseum"

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
echo "[3/4] Verifying versions"

echo "Cesium package:"
echo "  $CESIUM_ZIP"

echo
echo "Colosseum:"
git -C "$COLOSSEUM_ROOT" branch --show-current
git -C "$COLOSSEUM_ROOT" log -1 --oneline

echo
echo "[4/4] Complete"
echo "No Cesium extraction or Colosseum build was started."
