#!/usr/bin/env bash
set -euo pipefail

# Fail before a mission starts if one of the live sensor or result-pipeline
# links is missing.  A running Gazebo process is not enough: the previous
# false run had Livox data but no RGB/depth bridge, so YOLO silently saw zero
# frames for the first room.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$ROOT_DIR/devel/setup.bash"

failures=0

require_node() {
  local node="$1"
  if rosnode list 2>/dev/null | grep -Fxq "$node"; then
    echo "preflight node ok: $node"
  else
    echo "preflight node missing: $node" >&2
    failures=$((failures + 1))
  fi
}

require_publisher() {
  local topic="$1"
  local expected_publishers="${2:-1}"
  local info publishers count
  info="$(timeout --foreground 8s rostopic info "$topic" 2>&1 || true)"
  publishers="$(printf '%s\n' "$info" | awk '
    /^Publishers:/ { in_publishers = 1; next }
    /^Subscribers:/ { in_publishers = 0 }
    in_publishers && /^[[:space:]]*\*[[:space:]]/ { print }
  ' || true)"
  count="$(printf '%s\n' "$publishers" | sed '/^[[:space:]]*$/d' | wc -l)"
  if [ "$count" -eq "$expected_publishers" ]; then
    echo "preflight publisher ok: $topic ($count)"
  else
    echo "preflight publisher missing/ambiguous: $topic (expected $expected_publishers, got $count)" >&2
    printf '%s\n' "$info" >&2
    failures=$((failures + 1))
  fi
}

require_service() {
  local service="$1"
  if rosservice list 2>/dev/null | grep -Fxq "$service"; then
    echo "preflight service ok: $service"
  else
    echo "preflight service missing: $service" >&2
    failures=$((failures + 1))
  fi
}

if ! timeout --foreground 8s rosservice list >/dev/null 2>&1; then
  echo "preflight failed: ROS master is not reachable at ${ROS_MASTER_URI:-unset}" >&2
  exit 78
fi

for node in \
  /gazebo_sim_rgb_bridge \
  /gazebo_sim_depth_bridge \
  /yolo_detector_node \
  /danger_localization_node \
  /danger_result_writer \
  /fastlio_2d_projection; do
  require_node "$node"
done

# Exactly one publisher is expected for the mission's bridged streams.  This
# catches both the missing-bridge case and accidental duplicate stacks.
require_publisher /clock
require_publisher /livox/Pointcloud2
require_publisher /sim_rgb/image_raw
require_publisher /sim_depth/points
require_publisher /yolo/detections
require_publisher /map_confirmed

for service in \
  /gazebo/get_model_state \
  /gazebo/set_model_state \
  /call_elevator \
  /fastlio_2d_projection/save_current_floor \
  /danger_result_writer/reset \
  /target_manager/reset; do
  require_service "$service"
done

if [ "$failures" -ne 0 ]; then
  echo "exploration stack preflight FAILED: $failures check(s)" >&2
  exit 78
fi

echo "exploration stack preflight PASS"
