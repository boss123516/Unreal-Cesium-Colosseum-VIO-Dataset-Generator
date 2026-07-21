#!/usr/bin/env bash
set -euo pipefail

MODEL_NAME="${GAZEBO_FOLLOW_MODEL:-rc_cessna_ucc_0}"
GUI_CONFIG="${GZ_GUI_CONFIG:-$HOME/.gz/sim/8/gui.config}"

if [[ ! -f "$GUI_CONFIG" ]]; then
  echo "[ERROR] Gazebo GUI config not found: $GUI_CONFIG" >&2
  exit 1
fi

if ! gz topic -l 2>/dev/null | grep -q '^/world/.*/clock$'; then
  echo "[ERROR] Gazebo server is not running." >&2
  echo "        Start 08_run_gz_rc_cessna_ucc.sh first." >&2
  exit 1
fi

if pgrep -f '[g]z sim -g' >/dev/null; then
  echo "[ERROR] a Gazebo GUI is already running." >&2
  exit 1
fi

gz sim -g --gui-config "$GUI_CONFIG" &
gui_pid=$!

cleanup() {
  trap - EXIT INT TERM
  if kill -0 "$gui_pid" 2>/dev/null; then
    kill -TERM "$gui_pid" 2>/dev/null || true
  fi
  wait "$gui_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

for _ in $(seq 1 30); do
  if gz service -l 2>/dev/null | grep -q '^/gui/follow$'; then
    break
  fi
  if ! kill -0 "$gui_pid" 2>/dev/null; then
    wait "$gui_pid"
    echo "[ERROR] Gazebo GUI exited before follow mode was ready." >&2
    exit 1
  fi
  sleep 1
done

if ! gz service -l 2>/dev/null | grep -q '^/gui/follow$'; then
  echo "[ERROR] Gazebo follow service was not ready within 30 seconds." >&2
  exit 1
fi

gz service -s /gui/follow \
  --reqtype gz.msgs.StringMsg \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req "data: \"$MODEL_NAME\"" >/dev/null
gz service -s /gui/follow/offset \
  --reqtype gz.msgs.Vector3d \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'x: -12, y: 0, z: 4' >/dev/null

echo "[GAZEBO_FOLLOW_READY] model=$MODEL_NAME offset=(-12,0,4)"
wait "$gui_pid"
trap - EXIT INT TERM
