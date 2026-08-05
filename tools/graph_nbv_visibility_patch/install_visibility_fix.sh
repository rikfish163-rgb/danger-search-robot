#!/usr/bin/env bash
set -euo pipefail
WS="${1:-$HOME/catkin_ws}"
PKG="$WS/src/danger_search_robot"
PATCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"

SRC="$PATCH_DIR/danger_search_robot/exploration/graph_nbv"
DST="$PKG/exploration/graph_nbv"

for rel in scripts/graph_nbv_node.py config/graph_nbv_stage_b.yaml; do
  [[ -f "$DST/$rel" ]] && cp -a "$DST/$rel" "$DST/$rel.bak_visibility_${STAMP}"
  cp "$SRC/$rel" "$DST/$rel"
done
chmod +x "$DST/scripts/graph_nbv_node.py"
python3 -m py_compile "$DST/scripts/graph_nbv_node.py"
source /opt/ros/noetic/setup.bash
cd "$WS"
catkin_make -j4
echo "Visibility-aware Graph NBV fix installed. Backup timestamp: $STAMP"
