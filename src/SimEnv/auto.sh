#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WORKSPACE_DIR"

as_ros_bool() {
  case "$1" in
    1|true|TRUE|True|yes|YES|on|ON) printf "true" ;;
    0|false|FALSE|False|no|NO|off|OFF) printf "false" ;;
    *) printf "%s" "$1" ;;
  esac
}

SEED="${SEED:-}"
FLOOR_COUNT="${FLOOR_COUNT:-3}"
ROOMS_PER_FLOOR="${ROOMS_PER_FLOOR:-4}"
BUILDING_WIDTH="${BUILDING_WIDTH:-20.0}"
BUILDING_LENGTH="${BUILDING_LENGTH:-36.0}"
DANGER_COUNT="${DANGER_COUNT:-3:6}"
DISTRACTOR_COUNT="${DISTRACTOR_COUNT:-4:8}"
GUI="${GUI:-true}"
PAUSED="${PAUSED:-true}"
AUTO_UNPAUSE="$(as_ros_bool "${AUTO_UNPAUSE:-1}")"
AUTO_UNPAUSE_DELAY="${AUTO_UNPAUSE_DELAY:-6}"
START_CONTROLLER="${START_CONTROLLER:-1}"
START_VIRTUAL_JOY="${START_VIRTUAL_JOY:-0}"
CONTROLLER_FOREGROUND="${CONTROLLER_FOREGROUND:-1}"
START_BUILDING_CONTROL="${START_BUILDING_CONTROL:-1}"
ENABLE_SENSOR_DATA_DEFAULT="${ENABLE_SENSORS:-1}"
ENABLE_SENSOR_DATA="$(as_ros_bool "${ENABLE_SENSOR_DATA:-$ENABLE_SENSOR_DATA_DEFAULT}")"
ENABLE_LIVOX="$(as_ros_bool "${ENABLE_LIVOX:-$ENABLE_SENSOR_DATA}")"
ENABLE_LIVOX_IMU="$(as_ros_bool "${ENABLE_LIVOX_IMU:-$ENABLE_LIVOX}")"
ENABLE_REALSENSE_INPUT="${ENABLE_REALSENSE:-${ENABLE_DEPTH_CAMERA:-$ENABLE_SENSOR_DATA}}"
ENABLE_REALSENSE="$(as_ros_bool "$ENABLE_REALSENSE_INPUT")"
ENABLE_REALSENSE_ROS_PLUGIN="$(as_ros_bool "${ENABLE_REALSENSE_ROS_PLUGIN:-0}")"
ENABLE_FRONT_CAMERA="$(as_ros_bool "${ENABLE_FRONT_CAMERA:-0}")"
ENABLE_REFEREE_ODOM="$(as_ros_bool "${ENABLE_REFEREE_ODOM:-1}")"
ENABLE_GROUND_TRUTH="$(as_ros_bool "${ENABLE_GROUND_TRUTH:-1}")"
ENABLE_FOOT_CONTACT_SENSOR="$(as_ros_bool "${ENABLE_FOOT_CONTACT_SENSOR:-0}")"
ENABLE_FOOT_FORCE_VISUAL="$(as_ros_bool "${ENABLE_FOOT_FORCE_VISUAL:-0}")"
ENABLE_JOY_NODE="$(as_ros_bool "${ENABLE_JOY_NODE:-0}")"
ENABLE_POINTCLOUD_CONVERTER="$(as_ros_bool "${ENABLE_POINTCLOUD_CONVERTER:-$ENABLE_LIVOX}")"
POINTCLOUD_USE_GROUND_TRUTH_ODOM="$(as_ros_bool "${POINTCLOUD_USE_GROUND_TRUTH_ODOM:-1}")"
WRITE_GENERATED_TRUTH_COPY="$(as_ros_bool "${WRITE_GENERATED_TRUTH_COPY:-1}")"
UNITREE_CTRL_DT="${UNITREE_CTRL_DT:-0.004}"
UNITREE_LOG_WAIT_WARNINGS="$(as_ros_bool "${UNITREE_LOG_WAIT_WARNINGS:-0}")"
ROBOT_SPAWN_TIMEOUT="${ROBOT_SPAWN_TIMEOUT:-120}"
CONTROLLER_SPAWNER_TIMEOUT="${CONTROLLER_SPAWNER_TIMEOUT:-120}"
GAZEBO_PHYSICS_MAX_STEP_SIZE="${GAZEBO_PHYSICS_MAX_STEP_SIZE:-0.002}"
GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE="${GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE:-500}"
GAZEBO_PHYSICS_ODE_ITERS="${GAZEBO_PHYSICS_ODE_ITERS:-40}"
GAZEBO_PHYSICS_CONTACT_MAX_CORRECTING_VEL="${GAZEBO_PHYSICS_CONTACT_MAX_CORRECTING_VEL:-5.0}"
ROBOT_X="${ROBOT_X:-0.0}"
ROBOT_Y="${ROBOT_Y:--3.2}"
ROBOT_Z="${ROBOT_Z:-0.6}"
ROBOT_YAW="${ROBOT_YAW:-1.5708}"

schedule_unpause_physics() {
  if [ "$AUTO_UNPAUSE" != "true" ]; then
    return
  fi

  (
    echo "Waiting for Gazebo physics service..."

    while true; do
      if rosservice list 2>/dev/null | grep -q '^/gazebo/unpause_physics$'; then
        echo "Gazebo physics service ready. Unpausing..."

        rosservice call /gazebo/unpause_physics >/dev/null 2>&1 || true

        sleep 2

        if rostopic list 2>/dev/null | grep -q '^/clock$'; then
          echo "Gazebo simulation clock detected."
        fi

        exit 0
      fi

      sleep 1
    done
  ) &
}

wait_for_robot_spawn() {
  local timeout="$ROBOT_SPAWN_TIMEOUT"
  local deadline=$((SECONDS + timeout))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      echo "roslaunch exited during startup. Last log lines:" >&2
      tail -n 80 "$WORKSPACE_DIR/logs/competition_gazebo.log" >&2
      exit 1
    fi
    if timeout 1s rosservice call /gazebo/get_model_state "{model_name: 'a1_gazebo', relative_entity_name: 'world'}" 2>/dev/null | grep -q "success: True"; then
      return
    fi
    if grep -a -q "Successfully spawned entity" "$WORKSPACE_DIR/logs/competition_gazebo.log" 2>/dev/null; then
      return
    fi
    if grep -a -E -q "Spawn service failed|Service call failed" "$WORKSPACE_DIR/logs/competition_gazebo.log" 2>/dev/null; then
      echo "Robot spawn failed. Last log lines:" >&2
      tail -n 80 "$WORKSPACE_DIR/logs/competition_gazebo.log" >&2
      exit 1
    fi
    sleep 0.2
  done

  echo "Timed out waiting for robot spawn. Last log lines:" >&2
  tail -n 80 "$WORKSPACE_DIR/logs/competition_gazebo.log" >&2
  exit 1
}

echo "Terminating previous Gazebo, launch, controller, and optional joystick processes..."
pkill -f "roslaunch unitree_guide multi_floor_gazeboSim.launch" 2>/dev/null || true
pkill -f "building_generator_classic_control" 2>/dev/null || true
pkill -f "gzserver|gzclient|gazebo" 2>/dev/null || true
pkill -f "junior_ctrl" 2>/dev/null || true
pkill -f "virtual_joy.py" 2>/dev/null || true

echo "Sourcing ROS environment..."
source /opt/ros/noetic/setup.bash
if [ ! -f "$WORKSPACE_DIR/devel/setup.bash" ]; then
  echo "Missing $WORKSPACE_DIR/devel/setup.bash. Run catkin_make in this workspace before starting the simulation." >&2
  exit 1
fi
source "$WORKSPACE_DIR/devel/setup.bash"
export PATH="/opt/ros/noetic/bin:${PATH:-}"
export PYTHONPATH="$WORKSPACE_DIR/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$WORKSPACE_DIR/devel/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${LD_LIBRARY_PATH:-}"
export ROS_PACKAGE_PATH="$WORKSPACE_DIR/src:/opt/ros/noetic/share:${ROS_PACKAGE_PATH:-}"
export CMAKE_PREFIX_PATH="$WORKSPACE_DIR/devel:/opt/ros/noetic${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export PYTHONPATH="$WORKSPACE_DIR/src/SimEnv/src/building_generator_classic:$WORKSPACE_DIR/src/SimEnv/src/building_generator_core:${PYTHONPATH:-}"

GENERATOR_SCRIPT="$WORKSPACE_DIR/src/SimEnv/src/building_obstacles/scripts/generate_competition_scene.py"
BUILDING_CONTROL_SCRIPT="$WORKSPACE_DIR/src/SimEnv/src/building_generator_classic/scripts/building_generator_classic_control"
UNITREE_GAZEBO_MODELS="$WORKSPACE_DIR/src/SimEnv/src/unitree_guide/unitree_ros/unitree_gazebo/models"
SCENE_OUTPUT_DIR="$WORKSPACE_DIR/generated_building"
RESULTS_DIR="$WORKSPACE_DIR/results"
mkdir -p "$SCENE_OUTPUT_DIR" "$RESULTS_DIR" "$WORKSPACE_DIR/logs"

echo "Generating competition scene..."
GENERATOR_ARGS=(
  --output-dir "$SCENE_OUTPUT_DIR"
  --results-dir "$RESULTS_DIR"
  --floor-count "$FLOOR_COUNT"
  --rooms-per-floor "$ROOMS_PER_FLOOR"
  --width "$BUILDING_WIDTH"
  --length "$BUILDING_LENGTH"
  --danger-count "$DANGER_COUNT"
  --distractor-count "$DISTRACTOR_COUNT"
  --robot-x "$ROBOT_X"
  --robot-y "$ROBOT_Y"
  --robot-z "$ROBOT_Z"
  --robot-yaw "$ROBOT_YAW"
)
if [ -n "$SEED" ]; then
  GENERATOR_ARGS+=(--seed "$SEED")
fi
GENERATOR_ARGS+=(--physics-max-step-size "$GAZEBO_PHYSICS_MAX_STEP_SIZE")
GENERATOR_ARGS+=(--physics-real-time-update-rate "$GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE")
GENERATOR_ARGS+=(--physics-ode-iters "$GAZEBO_PHYSICS_ODE_ITERS")
GENERATOR_ARGS+=(--physics-contact-max-correcting-vel "$GAZEBO_PHYSICS_CONTACT_MAX_CORRECTING_VEL")
if [ "$WRITE_GENERATED_TRUTH_COPY" = "false" ]; then
  GENERATOR_ARGS+=(--no-generated-truth-copy)
fi
python3 "$GENERATOR_SCRIPT" "${GENERATOR_ARGS[@]}" \
  > "$SCENE_OUTPUT_DIR/scene_manifest.stdout.json"

export BUILDING_WORLD_FILE="$SCENE_OUTPUT_DIR/competition_scene.world"
export COMPETITION_ROBOT_X="$ROBOT_X"
export COMPETITION_ROBOT_Y="$ROBOT_Y"
export COMPETITION_ROBOT_Z="$ROBOT_Z"
export COMPETITION_ROBOT_YAW="$ROBOT_YAW"
export UNITREE_CTRL_DT
export UNITREE_LOG_WAIT_WARNINGS
export CONTROLLER_SPAWNER_TIMEOUT
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:$SCENE_OUTPUT_DIR:$UNITREE_GAZEBO_MODELS"
export GAZEBO_PLUGIN_PATH="$WORKSPACE_DIR/devel/lib:/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH:-}"

echo "=========================================="
echo "Competition scene is ready"
echo "  Workspace: $WORKSPACE_DIR"
echo "  World:   $BUILDING_WORLD_FILE"
echo "  Truth:   $RESULTS_DIR/danger_truth.json"
echo "  Manifest:$SCENE_OUTPUT_DIR/scene_manifest.json"
echo "  Result:  $RESULTS_DIR/detected_danger.json"
echo "  Robot pose: x=$ROBOT_X y=$ROBOT_Y z=$ROBOT_Z yaw=$ROBOT_YAW"
echo "  Sensor data default: $ENABLE_SENSOR_DATA"
echo "  Livox lidar: $ENABLE_LIVOX"
echo "  Livox IMU: $ENABLE_LIVOX_IMU"
echo "  RealSense depth camera: $ENABLE_REALSENSE"
echo "  Front RGB camera: $ENABLE_FRONT_CAMERA"
echo "  PointCloud2 converter: $ENABLE_POINTCLOUD_CONVERTER"
echo "  Ground truth topics: $ENABLE_GROUND_TRUTH"
echo "  Referee odom: $ENABLE_REFEREE_ODOM"
echo "  Foot contact sensors: $ENABLE_FOOT_CONTACT_SENSOR"
echo "  Foot force visual: $ENABLE_FOOT_FORCE_VISUAL"
echo "  Gazebo starts paused: $PAUSED"
echo "  Auto unpause: $AUTO_UNPAUSE after ${AUTO_UNPAUSE_DELAY}s"
echo "  Unitree wait warnings: $UNITREE_LOG_WAIT_WARNINGS"
echo "  Robot spawn timeout: ${ROBOT_SPAWN_TIMEOUT}s"
echo "  Controller spawner timeout: ${CONTROLLER_SPAWNER_TIMEOUT}s"
echo "  Gazebo physics: max_step=$GAZEBO_PHYSICS_MAX_STEP_SIZE update_rate=$GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE ode_iters=$GAZEBO_PHYSICS_ODE_ITERS"
echo "  Gazebo plugin path: $GAZEBO_PLUGIN_PATH"
echo "=========================================="

if [ "$START_VIRTUAL_JOY" = "1" ]; then
  echo "Starting virtual joystick. This may require uinput permissions."
  rosrun unitree_guide virtual_joy.py > "$WORKSPACE_DIR/logs/virtual_joy.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/virtual_joy.pid"
fi

echo "Launching Gazebo, Unitree A1 model, sensors, and ROS interfaces..."
roslaunch unitree_guide multi_floor_gazeboSim.launch \
  gui:="$GUI" \
  paused:="$PAUSED" \
  user_debug:=False \
  rname:=a1 \
  robot_x:="$ROBOT_X" \
  robot_y:="$ROBOT_Y" \
  robot_z:="$ROBOT_Z" \
  robot_yaw:="$ROBOT_YAW" \
  controller_spawner_timeout:="$CONTROLLER_SPAWNER_TIMEOUT" \
  enable_sensor_data:="$ENABLE_SENSOR_DATA" \
  enable_livox:="$ENABLE_LIVOX" \
  enable_livox_imu:="$ENABLE_LIVOX_IMU" \
  enable_realsense:="$ENABLE_REALSENSE" \
  enable_realsense_ros_plugin:="$ENABLE_REALSENSE_ROS_PLUGIN" \
  enable_front_camera:="$ENABLE_FRONT_CAMERA" \
  enable_referee_odom:="$ENABLE_REFEREE_ODOM" \
  enable_ground_truth:="$ENABLE_GROUND_TRUTH" \
  enable_foot_contact_sensor:="$ENABLE_FOOT_CONTACT_SENSOR" \
  enable_foot_force_visual:="$ENABLE_FOOT_FORCE_VISUAL" \
  enable_joy_node:="$ENABLE_JOY_NODE" \
  enable_pointcloud_converter:="$ENABLE_POINTCLOUD_CONVERTER" \
  pointcloud_use_ground_truth_odom:="$POINTCLOUD_USE_GROUND_TRUTH_ODOM" \
  > "$WORKSPACE_DIR/logs/competition_gazebo.log" 2>&1 &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$WORKSPACE_DIR/logs/competition_gazebo.pid"
wait_for_robot_spawn
echo "Waiting for Gazebo controller_manager..."

until rosservice list 2>/dev/null | grep -q "/a1_gazebo/controller_manager/list_controllers"
do
    sleep 1
done

echo "controller_manager ready."
if [ "$START_BUILDING_CONTROL" = "1" ]; then
  echo "Starting building door/elevator control service..."
  python3 "$BUILDING_CONTROL_SCRIPT" \
    --door-config "$SCENE_OUTPUT_DIR/door_config.yaml" \
    --elevator-config "$SCENE_OUTPUT_DIR/elevator_config.yaml" \
    > "$WORKSPACE_DIR/logs/building_control.log" 2>&1 &
  echo $! > "$WORKSPACE_DIR/logs/building_control.pid"
fi

if [ "$START_CONTROLLER" = "1" ]; then
  if [ "$CONTROLLER_FOREGROUND" = "1" ]; then
    echo "Starting junior_ctrl controller in the foreground."
    echo "UNITREE_CTRL_DT=$UNITREE_CTRL_DT seconds."
    echo "Use keyboard input in this terminal: 2 = stand, 4 = RL keyboard walk, 6 = RL /cmd_vel mode, 8 = reset."
    echo "In RL keyboard walk mode: W/S = forward/back, A/D = left/right, J/L = turn, Space = stop."

    echo "Unpausing Gazebo..."
    rosservice call /gazebo/unpause_physics >/dev/null 2>&1 || true

    sleep 2

    LIBTORCH_ROOT="${LIBTORCH_ROOT:-/root/ros1_isolated/deps/libtorch}"
    (
      cd "$WORKSPACE_DIR/src/SimEnv"
      LD_LIBRARY_PATH="$LIBTORCH_ROOT/lib:${LD_LIBRARY_PATH:-}" \
      "$WORKSPACE_DIR/devel/lib/unitree_guide/junior_ctrl"
    ) || true
    echo "junior_ctrl exited; keeping Gazebo running for inspection. Press Ctrl-C to stop this script."
    wait "$LAUNCH_PID"
    exit 0
  else
    echo "Starting junior_ctrl controller in the background. Keyboard state switching may not be available."
    echo "UNITREE_CTRL_DT=$UNITREE_CTRL_DT seconds."
    LIBTORCH_ROOT="${LIBTORCH_ROOT:-/root/ros1_isolated/deps/libtorch}"
    (
      cd "$WORKSPACE_DIR/src/SimEnv"
      LD_LIBRARY_PATH="$LIBTORCH_ROOT/lib:${LD_LIBRARY_PATH:-}" \
      "$WORKSPACE_DIR/devel/lib/unitree_guide/junior_ctrl"
    ) > "$WORKSPACE_DIR/logs/junior_ctrl.log" 2>&1 &
    echo $! > "$WORKSPACE_DIR/logs/junior_ctrl.pid"
    schedule_unpause_physics
  fi
else
  schedule_unpause_physics
fi

echo "Simulation startup command completed."
echo "Controller mode remains governed by unitree_guide keyboard/joy input. Mode 4 uses RL with keyboard axes; mode 6 keeps the original RL /cmd_vel logic."
wait "$LAUNCH_PID"
