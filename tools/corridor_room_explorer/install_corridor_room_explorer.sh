#!/usr/bin/env bash
set -euo pipefail

WS="${1:-$HOME/catkin_ws}"
PKG="$WS/src/danger_search_robot"

if [[ ! -f "$PKG/package.xml" || ! -f "$PKG/CMakeLists.txt" ]]; then
  echo "ERROR: danger_search_robot not found at $PKG" >&2
  exit 1
fi

mkdir -p \
  "$PKG/exploration/scripts" \
  "$PKG/exploration/config" \
  "$PKG/exploration/launch"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATCH_ROOT="$SCRIPT_DIR/danger_search_robot"

cp "$PATCH_ROOT/exploration/scripts/corridor_room_exploration_node.py" \
   "$PKG/exploration/scripts/corridor_room_exploration_node.py"

cp "$PATCH_ROOT/exploration/config/corridor_room_exploration.yaml" \
   "$PKG/exploration/config/corridor_room_exploration.yaml"

cp "$PATCH_ROOT/exploration/launch/corridor_room_exploration.launch" \
   "$PKG/exploration/launch/corridor_room_exploration.launch"

chmod +x "$PKG/exploration/scripts/corridor_room_exploration_node.py"

PKG_DIR="$PKG" python3 - <<'PY'
from pathlib import Path
import os

pkg = Path(os.environ["PKG_DIR"])
cmake = pkg / "CMakeLists.txt"
text = cmake.read_text()

target = "exploration/scripts/corridor_room_exploration_node.py"

if target not in text:
    block = (
        "\ncatkin_install_python(PROGRAMS\n"
        f"  {target}\n"
        "  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}\n"
        ")\n"
    )
    marker = "## Mark executable scripts (Python etc.) for installation"
    if marker not in text:
        raise RuntimeError("Python install marker not found in CMakeLists.txt")
    text = text.replace(marker, block + "\n" + marker, 1)
    cmake.write_text(text)
PY

python3 -m py_compile \
  "$PKG/exploration/scripts/corridor_room_exploration_node.py"

source /opt/ros/noetic/setup.bash
cd "$WS"
catkin_make -j4

echo
echo "Installed corridor_room_explorer."
echo "Dry-run launch:"
echo "  roslaunch danger_search_robot corridor_room_exploration.launch dry_run:=true"
echo
echo "Active launch after marker verification:"
echo "  roslaunch danger_search_robot corridor_room_exploration.launch dry_run:=false"
echo
echo "Do not run hybrid_exploration.launch or explore_lite.launch at the same time."
