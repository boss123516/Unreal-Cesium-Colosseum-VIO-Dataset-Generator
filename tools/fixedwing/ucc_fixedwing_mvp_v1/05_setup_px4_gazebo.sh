#!/usr/bin/env bash
set -euo pipefail

PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"

if [[ ! -d "$PX4_ROOT/.git" ]]; then
  echo "[ERROR] PX4 repository not found: $PX4_ROOT" >&2
  exit 1
fi

if command -v gz >/dev/null 2>&1; then
  echo "[OK] Gazebo already available: $(command -v gz)"
  gz --versions || true
  exit 0
fi

echo "[INFO] PX4's official Ubuntu setup will install Gazebo and build packages."
echo "[INFO] This step requires the local user's sudo password."
exec bash "$PX4_ROOT/Tools/setup/ubuntu.sh" --no-nuttx
