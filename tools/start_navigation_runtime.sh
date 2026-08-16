#!/usr/bin/env bash
set -euo pipefail

# Start the deterministic navigation half of the fixed ROS1 runtime.  The
# support stack (Gazebo truth TF, camera bridges, vision, and map projection)
# is intentionally started first by tools/start_exploration_stack.sh.
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
RUNTIME_DIR="${EXPLORATION_STACK_RUNTIME_DIR:-/tmp/ros1_exploration_stack}"
WAIT_SECONDS="${NAVIGATION_WAIT_SECONDS:-60}"
ACTIVATE_RL_MODE="${ACTIVATE_RL_MODE:-1}"
RESET_RL_MODE="${RESET_RL_MODE:-1}"
# Use the continuously cleared projection for TEB's short-horizon navigation.
# The debounced /map_confirmed layer remains the conservative Graph-NBV and
# clearance-check input; using it for the entrance costmap can turn transient
# confirmed cells into a complete corridor blockage.
MAP_TOPIC="${NAVIGATION_MAP_TOPIC:-/map_raw}"
mkdir -p "$RUNTIME_DIR"

source /opt/ros/noetic/setup.bash
CATKIN_DEVEL_SPACE="${CATKIN_DEVEL_SPACE:-$ROOT_DIR/devel}"
source "$CATKIN_DEVEL_SPACE/setup.bash"

node_present() {
  # The master may list a stale XML-RPC registration after a killed
  # roslaunch.  Only a successful ping counts as a running navigation node.
  rosnode ping -c1 "$1" 2>&1 | grep -Fq "xmlrpc reply from"
}

topic_present() {
  rostopic list 2>/dev/null | grep -Fxq "$1"
}

wait_for_tf() {
  local parent_frame="$1"
  local child_frame="$2"
  local deadline=$((SECONDS + WAIT_SECONDS))
  while [ "$SECONDS" -lt "$deadline" ]; do
    local sample
    sample="$(timeout --foreground 5s rosrun tf tf_echo "$parent_frame" "$child_frame" 2>&1 || true)"
    if grep -q "^At time" <<<"$sample"; then
      echo "navigation TF ready: $parent_frame->$child_frame"
      return 0
    fi
    sleep 1
  done
  echo "navigation TF failed to appear: $parent_frame->$child_frame" >&2
  return 1
}

wait_for_mapping_health() {
  local deadline=$((SECONDS + WAIT_SECONDS))
  while [ "$SECONDS" -lt "$deadline" ]; do
    local sample
    sample="$(timeout --foreground 5s rostopic echo -n 1 /mapping_health 2>&1 || true)"
    if grep -q 'healthy.*true' <<<"$sample"; then
      echo "mapping health ready: /mapping_health healthy=true"
      return 0
    fi
    sleep 1
  done
  echo "mapping health failed to become healthy" >&2
  return 1
}

wait_for_node() {
  local node="$1"
  local deadline=$((SECONDS + WAIT_SECONDS))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if node_present "$node"; then
      echo "navigation node ready: $node"
      return 0
    fi
    sleep 1
  done
  echo "navigation node failed to appear: $node" >&2
  return 1
}

wait_for_service() {
  local service="$1"
  local deadline=$((SECONDS + WAIT_SECONDS))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if rosservice list 2>/dev/null | grep -Fxq "$service"; then
      echo "navigation service ready: $service"
      return 0
    fi
    sleep 1
  done
  echo "navigation service failed to appear: $service" >&2
  return 1
}

controller_process_present() {
  ps -eo comm= 2>/dev/null | grep -Fxq "junior_ctrl"
}

start_native_controller() {
  if controller_process_present; then
    echo "native controller already running: junior_ctrl"
    return 0
  fi

  local controller_binary="${UNITREE_CTRL_BINARY:-/root/catkin_native/unitree_devel/lib/unitree_guide/junior_ctrl}"
  local libtorch_root="${LIBTORCH_ROOT:-/root/ros1_isolated/deps/libtorch}"
  if [ ! -x "$controller_binary" ]; then
    echo "native controller executable missing: $controller_binary" >&2
    return 78
  fi

  local log_path="$RUNTIME_DIR/junior_ctrl.log"
  echo "starting native controller -> $log_path"
  (
    cd "$ROOT_DIR/src/SimEnv"
    LD_LIBRARY_PATH="$libtorch_root/lib:${CATKIN_DEVEL_SPACE}/lib:/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}" \
      "$controller_binary"
  ) >"$log_path" 2>&1 &
  echo $! >"$RUNTIME_DIR/junior_ctrl.pid"

  local deadline=$((SECONDS + WAIT_SECONDS))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if controller_process_present; then
      echo "native controller ready: junior_ctrl"
      return 0
    fi
    sleep 1
  done
  echo "native controller failed to stay running: junior_ctrl" >&2
  tail -n 80 "$log_path" >&2 || true
  return 1
}

if ! timeout --foreground 8s rosservice list >/dev/null 2>&1; then
  echo "ROS master is not reachable at ${ROS_MASTER_URI:-unset}" >&2
  exit 78
fi

wait_for_node /gazebo
start_native_controller
wait_for_node /unitree_gazebo_servo
for topic in /clock "$MAP_TOPIC" /Odometry_gazebo; do
  if ! topic_present "$topic"; then
    echo "navigation prerequisite topic missing: $topic" >&2
    exit 78
  fi
done
wait_for_tf world body
wait_for_mapping_health

if node_present /move_base; then
  active_global_map="$(rosparam get /move_base/global_costmap/static_layer/map_topic 2>/dev/null || true)"
  active_local_map="$(rosparam get /move_base/local_costmap/static_layer/map_topic 2>/dev/null || true)"
  if [ "$active_global_map" != "$MAP_TOPIC" ] || [ "$active_local_map" != "$MAP_TOPIC" ]; then
    echo "refusing stale move_base map: global=$active_global_map local=$active_local_map expected=$MAP_TOPIC" >&2
    exit 79
  fi
  echo "navigation launch already running: /move_base (map=$MAP_TOPIC)"
else
  log_path="$RUNTIME_DIR/move_base_teb_gazebo_truth.log"
  echo "starting move_base/TEB map=$MAP_TOPIC -> $log_path"
  nohup roslaunch danger_search_robot move_base_teb_gazebo_truth.launch \
    robot_base_frame:=body map_topic:="$MAP_TOPIC" >"$log_path" 2>&1 &
  echo $! >"$RUNTIME_DIR/move_base_teb_gazebo_truth.pid"
fi
wait_for_node /move_base
wait_for_node /cmd_vel_arbiter
# move_base registers its node name before costmaps, the global planner, and
# the action server are fully initialized.  The make_plan service is useful,
# but it is not sufficient: check_exploration_stack also performs a real
# actionlib client handshake and checks TF/costmap freshness before launch of
# the exploration controller is allowed.
wait_for_service /move_base/make_plan
wait_for_service /move_base/GlobalPlanner/make_plan

if [ "$ACTIVATE_RL_MODE" = "1" ]; then
  echo "activating junior_ctrl RL /cmd_vel mode"
  activate_args=(--timeout "$WAIT_SECONDS")
  if [ "$RESET_RL_MODE" = "1" ]; then
    activate_args+=(--reset)
  fi
  python3 "$ROOT_DIR/tools/activate_rl_cmd_vel_mode.py" "${activate_args[@]}"
fi

CHECK_NAVIGATION_RUNTIME=1 "$ROOT_DIR/tools/check_exploration_stack.sh"
echo "navigation runtime ready: move_base/TEB -> cmd_vel_arbiter -> /cmd_vel -> RL"
