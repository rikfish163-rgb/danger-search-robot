#!/usr/bin/env bash
set -euo pipefail

# Build the controller in a Linux-native space.  The repository is on an
# NTFS bind mount, so putting CMake/EmPy output beside the source can enter
# uninterruptible I/O and make Docker unable to stop.  Runtime dependencies
# continue to come from the normal workspace devel space.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NATIVE_ROOT="${CATKIN_NATIVE_ROOT:-/root/catkin_native}"
BASE_DEVEL="${CATKIN_DEVEL_SPACE:-$ROOT_DIR/devel}"
BUILD_DIR="${CATKIN_NATIVE_BUILD_SPACE:-$NATIVE_ROOT/unitree_build}"
DEVEL_DIR="${CATKIN_NATIVE_DEVEL_SPACE:-$NATIVE_ROOT/unitree_devel}"
INSTALL_DIR="${CATKIN_NATIVE_INSTALL_SPACE:-$NATIVE_ROOT/unitree_install}"
PACKAGE_DIR="$ROOT_DIR/src/SimEnv/src/unitree_guide/unitree_guide/unitree_guide"
DANGER_PACKAGE_DIR="$ROOT_DIR/src/danger_search_robot"
DANGER_BUILD_DIR="${DANGER_NATIVE_BUILD_SPACE:-$NATIVE_ROOT/danger_build}"
DANGER_DEVEL_DIR="${DANGER_NATIVE_DEVEL_SPACE:-$NATIVE_ROOT/danger_devel}"

source /opt/ros/noetic/setup.bash
if [ ! -f "$BASE_DEVEL/setup.bash" ]; then
  echo "Missing runtime devel setup: $BASE_DEVEL/setup.bash" >&2
  exit 1
fi
source "$BASE_DEVEL/setup.bash"
export CMAKE_PREFIX_PATH="$BASE_DEVEL:/opt/ros/noetic"

mkdir -p "$BUILD_DIR" "$DEVEL_DIR" "$INSTALL_DIR"
cmake -S "$PACKAGE_DIR" -B "$BUILD_DIR" \
  -DCATKIN_DEVEL_PREFIX="$DEVEL_DIR" \
  -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
  -DLIBTORCH_ROOT="${LIBTORCH_ROOT:-/root/ros1_isolated/deps/libtorch}"
cmake --build "$BUILD_DIR" -- -j1

test -x "$DEVEL_DIR/lib/unitree_guide/junior_ctrl"
test -x "$DEVEL_DIR/lib/unitree_guide/state_from_gazebo"
echo "native unitree_guide build PASS"
echo "  UNITREE_CTRL_BINARY=$DEVEL_DIR/lib/unitree_guide/junior_ctrl"

# The projection node is also compiled away from the NTFS bind mount.  This
# keeps a source change in the fixed workspace from silently continuing to
# run an old C++ executable from /root/catkin_ws/devel.
cmake -S "$DANGER_PACKAGE_DIR" -B "$DANGER_BUILD_DIR" \
  -DCMAKE_PREFIX_PATH="$BASE_DEVEL;/opt/ros/noetic" \
  -DCATKIN_DEVEL_PREFIX="$DANGER_DEVEL_DIR" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$DANGER_BUILD_DIR" --target nearest_azimuth_projection_node -- -j1
test -x "$DANGER_DEVEL_DIR/lib/danger_search_robot/nearest_azimuth_projection_node"
echo "native mapping projection build PASS"
echo "  DANGER_PROJECTION_BINARY=$DANGER_DEVEL_DIR/lib/danger_search_robot/nearest_azimuth_projection_node"
