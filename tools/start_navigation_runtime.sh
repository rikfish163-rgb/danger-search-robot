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

if ! timeout --foreground 8s rosservice list >/dev/null 2>&1; then
  echo "ROS master is not reachable at ${ROS_MASTER_URI:-unset}" >&2
  exit 78
fi

for node in /gazebo /unitree_gazebo_servo; do
  wait_for_node "$node"
done
for topic in /clock "$MAP_TOPIC" /Odometry_gazebo; do
  if ! topic_present "$topic"; then
    echo "navigation prerequisite topic missing: $topic" >&2
    exit 78
  fi
done

if node_present /move_base; then
  echo "navigation launch already running: /move_base"
else
  log_path="$RUNTIME_DIR/move_base_teb_gazebo_truth.log"
  echo "starting move_base/TEB map=$MAP_TOPIC -> $log_path"
  nohup roslaunch danger_search_robot move_base_teb_gazebo_truth.launch \
    robot_base_frame:=body map_topic:="$MAP_TOPIC" >"$log_path" 2>&1 &
  echo $! >"$RUNTIME_DIR/move_base_teb_gazebo_truth.pid"
fi
wait_for_node /move_base
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
echo "navigation runtime ready: move_base/TEB -> /cmd_vel -> RL"
