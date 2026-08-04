#!/usr/bin/env bash
set -euo pipefail

WS="${1:-$HOME/catkin_ws}"
PKG="$WS/src/danger_search_robot"
PATCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"

if [[ ! -f "$PKG/package.xml" || ! -f "$PKG/CMakeLists.txt" ]]; then
  echo "ERROR: danger_search_robot not found at $PKG" >&2
  exit 1
fi

SOURCE_ROOT="$PATCH_DIR/danger_search_robot/exploration/graph_nbv"
TARGET_ROOT="$PKG/exploration/graph_nbv"

mkdir -p \
  "$TARGET_ROOT/scripts" \
  "$TARGET_ROOT/config" \
  "$TARGET_ROOT/launch" \
  "$TARGET_ROOT/docs"

for rel in \
  scripts/graph_nbv_node.py \
  config/graph_nbv_stage_b.yaml \
  launch/graph_nbv_stage_b.launch \
  docs/architecture.md
do
  src="$SOURCE_ROOT/$rel"
  dst="$TARGET_ROOT/$rel"

  if [[ -f "$dst" ]]; then
    cp -a "$dst" "${dst}.bak_${STAMP}"
  fi

  cp "$src" "$dst"
done

chmod +x "$TARGET_ROOT/scripts/graph_nbv_node.py"

PKG_DIR="$PKG" python3 - <<'PY'
from pathlib import Path
import os

pkg = Path(os.environ["PKG_DIR"])
cmake = pkg / "CMakeLists.txt"
text = cmake.read_text()

script = "exploration/graph_nbv/scripts/graph_nbv_node.py"

if script not in text:
    text += (
        "\ncatkin_install_python(PROGRAMS\n"
        f"  {script}\n"
        "  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}\n"
        ")\n"
    )
    cmake.write_text(text)

package_xml = pkg / "package.xml"
xml = package_xml.read_text()

dependencies = [
    "actionlib",
    "actionlib_msgs",
    "geometry_msgs",
    "move_base_msgs",
    "nav_msgs",
    "rospy",
    "std_msgs",
    "tf2_ros",
    "visualization_msgs",
]

missing = [
    dependency
    for dependency in dependencies
    if (
        f"<depend>{dependency}</depend>" not in xml
        and f"<exec_depend>{dependency}</exec_depend>" not in xml
    )
]

if missing:
    insertion = "".join(
        f"  <exec_depend>{dependency}</exec_depend>\n"
        for dependency in missing
    )
    if "</package>" not in xml:
        raise RuntimeError("Invalid package.xml")
    xml = xml.replace(
        "</package>",
        insertion + "</package>",
        1,
    )
    package_xml.write_text(xml)
PY

python3 -m py_compile \
  "$TARGET_ROOT/scripts/graph_nbv_node.py"

source /opt/ros/noetic/setup.bash
cd "$WS"
catkin_make -j4

source "$WS/devel/setup.bash"

echo
echo "Checking launch:"
roslaunch danger_search_robot graph_nbv_stage_b.launch --nodes

echo
echo "Graph NBV Stage B installed."
echo "Backup timestamp: $STAMP"
echo
echo "Dry run:"
echo "  roslaunch danger_search_robot graph_nbv_stage_b.launch dry_run:=true"
echo
echo "Active run after validation:"
echo "  roslaunch danger_search_robot graph_nbv_stage_b.launch dry_run:=false"
