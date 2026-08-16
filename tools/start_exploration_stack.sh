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
source "$ROOT_DIR/tools/ros1_runtime_paths.sh"
FLOOR_COUNT="${FLOOR_COUNT:-3}"

node_present() {
  # rosnode list can retain a dead XML-RPC registration briefly after a
  # roslaunch/move_base crash.  A ping proves that the node is responsive.
  rosnode ping -c1 "$1" 2>&1 | grep -Fq "xmlrpc reply from"
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

wait_for_sensor_topic() {
  local topic="$1"
  local deadline=$((SECONDS + ${SENSOR_TOPIC_TIMEOUT_SECONDS:-30}))
  while [ "$SECONDS" -lt "$deadline" ]; do
    local sample
    sample="$(timeout --foreground 6s rostopic hz "$topic" 2>&1 || true)"
    if grep -q "average rate" <<<"$sample"; then
      echo "sensor topic ready: $topic"
      return 0
    fi
    sleep 1
  done
  echo "sensor topic has no frames: $topic" >&2
  return 1
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

# A registered bridge process is not enough: Gazebo can expose the transport
# topic while its render context is unavailable.  Fail before YOLO starts so a
# mission can never silently run with active=0 and no RGB/depth frames.
wait_for_sensor_topic /sim_rgb/image_raw
wait_for_sensor_topic /sim_depth/points

# Door/elevator services are part of the runnable mission contract.  Starting
# this node here makes a clean fixed-container restart equivalent to the
# previously manual command and lets the preflight check fail early if the
# generated scene/config pair is inconsistent.
start_binary /building_generator_classic_control building_generator_classic_control.log \
  python3 "$ROOT_DIR/src/SimEnv/src/building_generator_classic/scripts/building_generator_classic_control" \
  --door-config "$ROOT_DIR/generated_building/door_config.yaml" \
  --elevator-config "$ROOT_DIR/generated_building/elevator_config.yaml"

start_launch vision_stack.log \
  "/yolo_detector_node /danger_localization_node /danger_result_writer" \
  roslaunch danger_search_robot vision_stack.launch \
  image_topic:=/sim_rgb/image_raw pointcloud_topic:=/sim_depth/points \
  yolo_start_delay:=8 reset_results:=true start_recording:=true \
  runtime_result_file:="$ROS1_RUNTIME_RESULT_FILE"

if node_present /danger_result_writer; then
  active_result_file="$(rosparam get /danger_result_writer/runtime_result_file 2>/dev/null || true)"
  if [ "$active_result_file" != "$ROS1_RUNTIME_RESULT_FILE" ]; then
    echo "refusing stale result-writer runtime: result=$active_result_file" >&2
    echo "expected result=$ROS1_RUNTIME_RESULT_FILE" >&2
    exit 79
  fi
fi

projection_binary="${DANGER_PROJECTION_BINARY:-/root/catkin_native/danger_devel/lib/danger_search_robot/nearest_azimuth_projection_node}"
if node_present /fastlio_2d_projection; then
  active_state_file="$(rosparam get /fastlio_2d_projection/floor_state_file 2>/dev/null || true)"
  active_maps_root="$(rosparam get /fastlio_2d_projection/floor_maps_root 2>/dev/null || true)"
  if [ "$active_state_file" != "$ROS1_RUNTIME_STATE_DIR/floor_state.json" ] || \
     [ "$active_maps_root" != "$ROS1_RUNTIME_MAPS_DIR" ]; then
    echo "refusing stale projection runtime: state=$active_state_file maps=$active_maps_root" >&2
    echo "expected state=$ROS1_RUNTIME_STATE_DIR/floor_state.json maps=$ROS1_RUNTIME_MAPS_DIR" >&2
    exit 79
  fi
  echo "support node already running: /fastlio_2d_projection (native runtime paths verified)"
elif [ -x "$projection_binary" ]; then
  projection_log="$RUNTIME_DIR/fastlio_2d_projection.log"
  echo "starting native /fastlio_2d_projection -> $projection_log"
  rosparam load "$ROOT_DIR/src/danger_search_robot/config/fastlio_2d_projection.yaml" \
    /fastlio_2d_projection
  rosparam set /fastlio_2d_projection/input_cloud_topic /livox/Pointcloud2
  rosparam set /fastlio_2d_projection/sensor_frame body
  rosparam set /fastlio_2d_projection/floor_state_file "$ROS1_RUNTIME_STATE_DIR/floor_state.json"
  rosparam set /fastlio_2d_projection/floor_maps_root "$ROS1_RUNTIME_MAPS_DIR"
  rosparam set /fastlio_2d_projection/floor_count "$FLOOR_COUNT"
  rosparam set /fastlio_2d_projection/clear_all_floor_maps true
  nohup "$projection_binary" __name:=fastlio_2d_projection \
    >"$projection_log" 2>&1 &
  echo $! >"$RUNTIME_DIR/fastlio_2d_projection.pid"
  wait_for_node /fastlio_2d_projection "$projection_log"
else
  start_launch fastlio_2d_projection.log \
    /fastlio_2d_projection \
    roslaunch danger_search_robot mapping/launch/fastlio_2d_projection.launch \
    input_cloud_topic:=/livox/Pointcloud2 sensor_frame:=body \
    clear_all_floor_maps:=true \
    floor_state_file:="$ROS1_RUNTIME_STATE_DIR/floor_state.json" \
    floor_maps_root:="$ROS1_RUNTIME_MAPS_DIR" \
    floor_count:="$FLOOR_COUNT"
fi

start_binary /mapping_health_watchdog mapping_health_watchdog.log \
  python3 "$ROOT_DIR/src/danger_search_robot/scripts/mapping_health_watchdog.py" \
  _cloud_topic:=/livox/Pointcloud2 \
  _map_topic:=/map_raw \
  _confirmed_map_topic:=/map_confirmed \
  _expected_map_frame:=world \
  _status_topic:=/fastlio_2d_projection/status \
  _max_cloud_age:=0.75 \
  _max_map_age:=1.25 \
  _max_wall_silence:=8.0

"$ROOT_DIR/tools/check_exploration_stack.sh"
