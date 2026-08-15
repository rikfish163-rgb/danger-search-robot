#!/usr/bin/env bash
set -euo pipefail

# Start the fixed-container support stack in dependency order.  In particular
# the Gazebo RGB/depth bridges must exist before vision_stack starts; otherwise
# the YOLO node is alive but receives no images.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_DIR="${EXPLORATION_STACK_RUNTIME_DIR:-/tmp/ros1_exploration_stack}"
mkdir -p "$RUNTIME_DIR"

source /opt/ros/noetic/setup.bash
CATKIN_DEVEL_SPACE="${CATKIN_DEVEL_SPACE:-$ROOT_DIR/devel}"
source "$CATKIN_DEVEL_SPACE/setup.bash"

node_present() {
  rosnode list 2>/dev/null | grep -Fxq "$1"
}

wait_for_node() {
  local node="$1"
  local log_path="$2"
  local deadline=$((SECONDS + ${STACK_NODE_TIMEOUT_SECONDS:-45}))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if node_present "$node"; then
      echo "stack node ready: $node"
      return 0
    fi
    sleep 1
  done
  echo "stack node failed to appear: $node" >&2
  if [ -f "$log_path" ]; then
    tail -n 80 "$log_path" >&2 || true
  fi
  return 1
}

start_binary() {
  local node="$1"
  local log_name="$2"
  shift 2
  if node_present "$node"; then
    echo "stack node already running: $node"
    return 0
  fi
  local log_path="$RUNTIME_DIR/$log_name"
  echo "starting $node -> $log_path"
  nohup "$@" >"$log_path" 2>&1 &
  echo $! >"$RUNTIME_DIR/${node#/}.pid"
  wait_for_node "$node" "$log_path"
}

start_launch() {
  local log_name="$1"
  local required_nodes="$2"
  shift 2
  local log_path="$RUNTIME_DIR/$log_name"
  local present=0
  local node
  for node in $required_nodes; do
    if node_present "$node"; then
      present=$((present + 1))
    fi
  done
  if [ "$present" -eq 0 ]; then
    echo "starting launch -> $log_path"
    nohup "$@" >"$log_path" 2>&1 &
    echo $! >"$RUNTIME_DIR/${log_name%.log}.pid"
  elif [ "$present" -ne "$(wc -w <<<"$required_nodes")" ]; then
    echo "refusing partial support launch: $required_nodes" >&2
    return 1
  else
    echo "support launch already running: $required_nodes"
  fi
  for node in $required_nodes; do
    wait_for_node "$node" "$log_path"
  done
}

if ! timeout --foreground 8s rosservice list >/dev/null 2>&1; then
  echo "ROS master is not reachable at ${ROS_MASTER_URI:-unset}" >&2
  exit 78
fi

# TF and truth localization are needed by both the live detector and the
# deterministic runner.  The static map transform is intentionally a named
# node so a second invocation does not create duplicate TF publishers.
start_binary /gazebo_truth_body_tf gazebo_truth_body_tf.log \
  python3 "$ROOT_DIR/src/danger_search_robot/scripts/gazebo_truth_body_tf.py" \
  _body_frame:=body
start_binary /world_map_tf world_map_tf.log \
  /opt/ros/noetic/lib/tf/static_transform_publisher \
  0 0 0 0 0 0 world map 10 __name:=world_map_tf
start_binary /gazebo_truth_base_tf gazebo_truth_base_tf.log \
  python3 "$ROOT_DIR/src/danger_search_robot/scripts/gazebo_truth_body_tf.py" \
  _body_frame:=truth_base __name:=gazebo_truth_base_tf

# These two nodes were previously started manually after the mission had
# already entered room 3.  Keep them before vision_stack and before the run.
start_binary /gazebo_sim_rgb_bridge gazebo_sim_rgb_bridge.log \
  "$CATKIN_DEVEL_SPACE/lib/danger_search_robot/gazebo_image_to_ros" \
  /gazebo/generated_world/a1_gazebo/base/real_sense/image \
  /sim_rgb/image_raw real_sense_color_optical_frame \
  __name:=gazebo_sim_rgb_bridge
start_binary /gazebo_sim_depth_bridge gazebo_sim_depth_bridge.log \
  "$CATKIN_DEVEL_SPACE/lib/danger_search_robot/gazebo_depth_to_pointcloud" \
  /gazebo/generated_world/a1_gazebo/base/real_sense_depth/image \
  /sim_depth/points real_sense_depth_optical_frame 1.0472 \
  __name:=gazebo_sim_depth_bridge

start_launch vision_stack.log \
  "/yolo_detector_node /danger_localization_node /danger_result_writer" \
  roslaunch danger_search_robot vision_stack.launch \
  image_topic:=/sim_rgb/image_raw pointcloud_topic:=/sim_depth/points \
  yolo_start_delay:=8 reset_results:=true start_recording:=true

start_launch fastlio_2d_projection.log \
  /fastlio_2d_projection \
  roslaunch danger_search_robot fastlio_2d_projection.launch \
  input_cloud_topic:=/livox/Pointcloud2 sensor_frame:=body \
  clear_all_floor_maps:=true

start_binary /mapping_health_watchdog mapping_health_watchdog.log \
  python3 "$ROOT_DIR/src/danger_search_robot/scripts/mapping_health_watchdog.py" \
  _cloud_topic:=/livox/Pointcloud2 \
  _map_topic:=/map_confirmed \
  _status_topic:=/fastlio_2d_projection/status \
  _max_cloud_age:=0.75 \
  _max_map_age:=1.25

"$ROOT_DIR/tools/check_exploration_stack.sh"
