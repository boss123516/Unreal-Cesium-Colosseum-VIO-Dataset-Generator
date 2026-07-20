#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
PLUGIN_BUILD="${UCC_GZ_PLUGIN_BUILD:-$HOME/vio_sim_ws/build/ucc_fixedwing_gz_plugin}"
MODEL_ROOT="$SCRIPT_DIR/gz_models"
PX4_MODEL_ROOT="$PX4_ROOT/Tools/simulation/gz/models"
PX4_WORLD_ROOT="$PX4_ROOT/Tools/simulation/gz/worlds"
PX4_SERVER_CONFIG="$PX4_ROOT/src/modules/simulation/gz_bridge/server.config"
PX4_PLUGIN_BUILD="$PX4_ROOT/build/px4_sitl_default/src/modules/simulation/gz_plugins"
PX4_RUN_ROOT="$PX4_ROOT/build/px4_sitl_default/src/modules/simulation/gz_bridge"
PX4_BIN="$PX4_ROOT/build/px4_sitl_default/bin/px4"
WORLD="${PX4_GZ_WORLD:-default}"

# Gazebo Transport otherwise may not discover a loopback-only server during
# the duplicate-world guard below.
export GZ_IP="${GZ_IP:-127.0.0.1}"

for path in \
  "$PLUGIN_BUILD/libUccKinematicsPublisher.so" \
  "$MODEL_ROOT/rc_cessna_ucc/model.sdf" \
  "$PX4_WORLD_ROOT/$WORLD.sdf" \
  "$PX4_SERVER_CONFIG" \
  "$PX4_PLUGIN_BUILD/libOpticalFlowSystem.so" \
  "$PX4_PLUGIN_BUILD/libGstCameraSystem.so" \
  "$PX4_BIN"; do
  if [[ ! -e "$path" ]]; then
    echo "[ERROR] required path not found: $path" >&2
    exit 1
  fi
done

if gz topic -l 2>/dev/null | grep -q '^/world/.*/clock$'; then
  echo "[ERROR] a Gazebo world is already running" >&2
  exit 1
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH="$PLUGIN_BUILD:$PX4_PLUGIN_BUILD${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$MODEL_ROOT:$PX4_MODEL_ROOT:$PX4_WORLD_ROOT${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SERVER_CONFIG_PATH="$PX4_SERVER_CONFIG"

setsid gz sim --verbose="${GZ_VERBOSE:-1}" -r -s "$PX4_WORLD_ROOT/$WORLD.sdf" &
gazebo_pid=$!

cleanup() {
  trap - EXIT INT TERM

  if kill -0 "$gazebo_pid" 2>/dev/null; then
    kill -TERM -- "-$gazebo_pid" 2>/dev/null || true

    for _ in $(seq 1 20); do
      if ! kill -0 "$gazebo_pid" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done

    if kill -0 "$gazebo_pid" 2>/dev/null; then
      kill -KILL -- "-$gazebo_pid" 2>/dev/null || true
    fi
  fi

  wait "$gazebo_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

for _ in $(seq 1 30); do
  if gz service -i --service "/world/$WORLD/scene/info" 2>&1 | grep -q 'Service providers'; then
    break
  fi
  sleep 1
done

if ! gz service -i --service "/world/$WORLD/scene/info" 2>&1 | grep -q 'Service providers'; then
  echo "[ERROR] Gazebo world did not become ready" >&2
  exit 1
fi

cd "$PX4_RUN_ROOT"
PX4_SYS_AUTOSTART=4003 \
PX4_SIM_MODEL=gz_rc_cessna_ucc \
PX4_GZ_MODELS="$MODEL_ROOT" \
PX4_GZ_WORLD="$WORLD" \
PX4_GZ_STANDALONE=1 \
"$PX4_BIN"
