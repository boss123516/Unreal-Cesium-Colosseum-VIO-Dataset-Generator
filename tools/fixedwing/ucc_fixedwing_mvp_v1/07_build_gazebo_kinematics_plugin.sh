#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="$SCRIPT_DIR/gz_plugin"
BUILD_DIR="${UCC_GZ_PLUGIN_BUILD:-$HOME/vio_sim_ws/build/ucc_fixedwing_gz_plugin}"

cmake -S "$SOURCE_DIR" -B "$BUILD_DIR" -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$BUILD_DIR"

PLUGIN="$BUILD_DIR/libUccKinematicsPublisher.so"
if [[ ! -s "$PLUGIN" ]]; then
  echo "[ERROR] plugin was not produced: $PLUGIN" >&2
  exit 1
fi

echo "[OK] $PLUGIN"
