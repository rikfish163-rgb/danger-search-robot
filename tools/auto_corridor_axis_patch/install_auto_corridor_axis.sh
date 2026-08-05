#!/usr/bin/env bash
set -euo pipefail

WS="${1:-$HOME/catkin_ws}"
PKG="$WS/src/danger_search_robot"
PATCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"

if [[ ! -f "$PKG/package.xml" ]]; then
  echo "ERROR: package not found: $PKG" >&2
  exit 1
fi

TARGET_SCRIPT="$PKG/exploration/scripts/corridor_room_exploration_node.py"
TARGET_CONFIG="$PKG/exploration/config/corridor_room_exploration.yaml"
TARGET_LAUNCH="$PKG/exploration/launch/corridor_room_exploration.launch"

mkdir -p \
  "$PKG/exploration/scripts" \
  "$PKG/exploration/config" \
  "$PKG/exploration/launch"

for file in "$TARGET_SCRIPT" "$TARGET_CONFIG" "$TARGET_LAUNCH"; do
  if [[ -f "$file" ]]; then
    cp -a "$file" "${file}.bak_auto_axis_${STAMP}"
  fi
done

cp \
  "$PATCH_DIR/danger_search_robot/exploration/scripts/corridor_room_exploration_node.py" \
  "$TARGET_SCRIPT"

cp \
  "$PATCH_DIR/danger_search_robot/exploration/config/corridor_room_exploration.yaml" \
  "$TARGET_CONFIG"

cp \
  "$PATCH_DIR/danger_search_robot/exploration/launch/corridor_room_exploration.launch" \
  "$TARGET_LAUNCH"

chmod +x "$TARGET_SCRIPT"

python3 -m py_compile "$TARGET_SCRIPT"

echo
echo "Automatic corridor-axis patch installed."
echo "Backup suffix: .bak_auto_axis_${STAMP}"
echo
echo "First test:"
echo "  roslaunch danger_search_robot corridor_room_exploration.launch dry_run:=true"
echo
echo "Do not run hybrid_exploration.launch or explore_lite.launch simultaneously."
