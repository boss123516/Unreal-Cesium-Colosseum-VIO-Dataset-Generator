#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VINS_WS="${VINS_WS:-$HOME/vins_mono_ros2_ws}"
REPO_DIR="$VINS_WS/src/VINS-MONO-ROS2"
ROS_DISTRO_EXPECTED="${ROS_DISTRO_EXPECTED:-humble}"

if [[ ! -f "/opt/ros/$ROS_DISTRO_EXPECTED/setup.bash" ]]; then
  echo "[ERROR] ROS 2 $ROS_DISTRO_EXPECTED is not installed." >&2
  echo "        Expected: /opt/ros/$ROS_DISTRO_EXPECTED/setup.bash" >&2
  exit 1
fi

echo "=== 1/6 System dependencies ==="
sudo apt-get update
sudo apt-get install -y \
  git \
  build-essential \
  cmake \
  pkg-config \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-opencv \
  python3-numpy \
  libeigen3-dev \
  libceres-dev \
  libboost-all-dev \
  libopencv-dev \
  libsuitesparse-dev \
  libgoogle-glog-dev \
  libgflags-dev \
  "ros-$ROS_DISTRO_EXPECTED-cv-bridge" \
  "ros-$ROS_DISTRO_EXPECTED-image-transport" \
  "ros-$ROS_DISTRO_EXPECTED-message-filters" \
  "ros-$ROS_DISTRO_EXPECTED-tf2" \
  "ros-$ROS_DISTRO_EXPECTED-tf2-ros" \
  "ros-$ROS_DISTRO_EXPECTED-rviz2"

# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO_EXPECTED/setup.bash"

echo "=== 2/6 rosdep ==="
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

echo "=== 3/6 Clone repository ==="
mkdir -p "$VINS_WS/src"

if [[ -d "$REPO_DIR/.git" ]]; then
  echo "[INFO] Repository already exists: $REPO_DIR"
  echo "[INFO] Existing local changes are preserved; git pull is not forced."
else
  git clone \
    https://github.com/boss123516/VINS-MONO-ROS2.git \
    "$REPO_DIR"
fi

echo "=== 4/6 Install UCC launch and tools ==="
install -m 0644 \
  "$SCRIPT_DIR/patches/ucc_vins.launch.py" \
  "$REPO_DIR/vins_estimator/launch/ucc_vins.launch.py"

mkdir -p "$VINS_WS/tools/ucc"
install -m 0755 \
  "$SCRIPT_DIR/tools/generate_ucc_config.py" \
  "$VINS_WS/tools/ucc/generate_ucc_config.py"
install -m 0755 \
  "$SCRIPT_DIR/tools/ucc_dataset_player.py" \
  "$VINS_WS/tools/ucc/ucc_dataset_player.py"

echo "=== 5/6 Resolve ROS dependencies ==="
rosdep install \
  --from-paths "$VINS_WS/src" \
  --ignore-src \
  --rosdistro "$ROS_DISTRO_EXPECTED" \
  -r -y

echo "=== 6/6 Build ==="
cd "$VINS_WS"
colcon build \
  --symlink-install \
  --parallel-workers 4 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "[OK] VINS-MONO-ROS2 setup completed."
echo "Workspace : $VINS_WS"
echo "Repository: $REPO_DIR"
echo
echo "Next command:"
echo "  cd $SCRIPT_DIR"
echo "  ./01_validate_prepare_dataset.sh /absolute/path/to/ucc_dataset"
