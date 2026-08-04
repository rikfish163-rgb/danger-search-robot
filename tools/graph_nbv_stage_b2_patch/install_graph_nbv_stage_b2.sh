#!/usr/bin/env bash
set -euo pipefail

WS="${1:-$HOME/catkin_ws}"
PKG="$WS/src/danger_search_robot"
PATCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"

if [[ ! -f "$PKG/package.xml" || ! -f "$PKG/CMakeLists.txt" ]]; then
  echo "ERROR: danger_search_robot not found: $PKG" >&2
  exit 1
fi

SOURCE="$PATCH_DIR/danger_search_robot/exploration/graph_nbv"
TARGET="$PKG/exploration/graph_nbv"

mkdir -p \
  "$TARGET/scripts" \
  "$TARGET/config" \
  "$TARGET/launch" \
  "$TARGET/docs"

for rel in \
  scripts/graph_nbv_stage_b2_node.py \
  config/graph_nbv_stage_b2.yaml \
  launch/graph_nbv_stage_b2.launch \
  docs/stage_b2_architecture.md
do
  src="$SOURCE/$rel"
  dst="$TARGET/$rel"

  if [[ -f "$dst" ]]; then
    cp -a "$dst" "${dst}.bak_${STAMP}"
  fi

  cp "$src" "$dst"
done

chmod +x "$TARGET/scripts/graph_nbv_stage_b2_node.py"

PKG_DIR="$PKG" python3 - <<'PY'
from pathlib import Path
import os

pkg = Path(os.environ["PKG_DIR"])
cmake = pkg / "CMakeLists.txt"
text = cmake.read_text()

script = "exploration/graph_nbv/scripts/graph_nbv_stage_b2_node.py"

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

for dependency in (
    "actionlib",
    "actionlib_msgs",
    "geometry_msgs",
    "move_base_msgs",
    "nav_msgs",
    "rospy",
    "std_msgs",
    "tf2_ros",
    "visualization_msgs",
):
    if (
        f"<depend>{dependency}</depend>" not in xml
        and f"<exec_depend>{dependency}</exec_depend>" not in xml
    ):
        xml = xml.replace(
            "</package>",
            f"  <exec_depend>{dependency}</exec_depend>\n"
            "</package>",
            1,
        )

package_xml.write_text(xml)
PY

python3 -m py_compile \
  "$TARGET/scripts/graph_nbv_stage_b2_node.py"

source /opt/ros/noetic/setup.bash
cd "$WS"
catkin_make -j4

source "$WS/devel/setup.bash"

echo
echo "Launch check:"
roslaunch danger_search_robot graph_nbv_stage_b2.launch --nodes

echo
echo "Graph NBV Stage B.2 installed."
echo "Backup timestamp: $STAMP"
echo
echo "Dry run:"
echo "  roslaunch danger_search_robot graph_nbv_stage_b2.launch dry_run:=true"
echo
echo "Active:"
echo "  roslaunch danger_search_robot graph_nbv_stage_b2.launch dry_run:=false"
