#!/usr/bin/env bash
set -euo pipefail

WS="${1:-$HOME/catkin_ws}"
PKG="$WS/src/danger_search_robot"
PATCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"

source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"

rospack find teb_local_planner >/dev/null

mkdir -p "$PKG/navigation/config" "$PKG/navigation/launch"

FILES=(
  navigation/config/teb_local_planner.yaml
  navigation/config/costmap_common_teb.yaml
  navigation/config/global_costmap_teb.yaml
  navigation/config/local_costmap_teb.yaml
  navigation/launch/move_base_teb.launch
)

for rel in "${FILES[@]}"; do
  src="$PATCH_DIR/danger_search_robot/$rel"
  dst="$PKG/$rel"
  [[ -f "$dst" ]] && cp -a "$dst" "${dst}.bak_${STAMP}"
  cp "$src" "$dst"
done

if ! grep -q '<exec_depend>teb_local_planner</exec_depend>' "$PKG/package.xml"; then
  cp -a "$PKG/package.xml" "$PKG/package.xml.bak_${STAMP}"
  sed -i '/<\/package>/i\  <exec_depend>teb_local_planner</exec_depend>'     "$PKG/package.xml"
fi

roslaunch danger_search_robot move_base_teb.launch --nodes

echo "Installed TEB profile. Backup timestamp: $STAMP"
echo "Launch: roslaunch danger_search_robot move_base_teb.launch"
