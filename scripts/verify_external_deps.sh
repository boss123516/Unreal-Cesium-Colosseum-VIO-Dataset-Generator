#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set -a
source "$REPO_ROOT/VERSION.env"
set +a

failed=0

check_path() {
    local name="$1"
    local path="$2"

    if [[ -e "$path" ]]; then
        echo "[OK] $name: $path"
    else
        echo "[MISSING] $name: $path"
        failed=1
    fi
}

check_path "Unreal Engine" "$UE_ROOT"

if [[ -n "${CESIUM_ROOT:-}" ]]; then
    check_path "Cesium for Unreal" "$CESIUM_ROOT"
fi

if [[ -n "${COLOSSEUM_ROOT:-}" ]]; then
    check_path "Colosseum" "$COLOSSEUM_ROOT"
fi

exit "$failed"
