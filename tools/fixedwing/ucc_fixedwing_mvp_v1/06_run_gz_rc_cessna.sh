#!/usr/bin/env bash
set -euo pipefail

PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
PX4_VENV="${PX4_VENV:-$HOME/vio_sim_ws/px4_pyenv}"

if [[ ! -d "$PX4_ROOT/.git" ]]; then
  echo "[ERROR] PX4 repository not found: $PX4_ROOT" >&2
  exit 1
fi

if [[ ! -x "$PX4_VENV/bin/python" ]]; then
  echo "[ERROR] PX4 Python environment not found: $PX4_VENV" >&2
  exit 1
fi

if ! command -v gz >/dev/null 2>&1; then
  echo "[ERROR] Gazebo CLI not found. Run 05_setup_px4_gazebo.sh first." >&2
  exit 1
fi

# shellcheck disable=SC1091
source "$PX4_VENV/bin/activate"
cd "$PX4_ROOT"
exec make px4_sitl gz_rc_cessna "$@"
