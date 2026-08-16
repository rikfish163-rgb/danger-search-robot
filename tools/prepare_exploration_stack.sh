#!/usr/bin/env bash
set -euo pipefail

# Reset only low-frequency mission state and make all doors usable.  It does
# not kill ROS/Gazebo processes; use auto.sh or restart the fixed container
# when a completely new simulation instance is required.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FLOOR_COUNT="${FLOOR_COUNT:-3}"
ROOMS_PER_FLOOR="${ROOMS_PER_FLOOR:-4}"
RESET_ROBOT_POSE="${RESET_ROBOT_POSE:-1}"
ROBOT_X="${ROBOT_X:-0.0}"
ROBOT_Y="${ROBOT_Y:--3.2}"
ROBOT_Z="${ROBOT_Z:-0.6}"
ROBOT_YAW="${ROBOT_YAW:-1.5708}"
source /opt/ros/noetic/setup.bash
source "$ROOT_DIR/devel/setup.bash"
source "$ROOT_DIR/tools/ros1_runtime_paths.sh"

STATE_FILE="$ROS1_RUNTIME_STATE_DIR/floor_state.json"
ANCHOR_FILE="$ROS1_RUNTIME_STATE_DIR/floor_transition_anchor.json"

validate_scene_contract() {
  local metadata_path="$ROOT_DIR/generated_building/layout_metadata.json"
  [ -f "$metadata_path" ] || {
    echo "scene contract failed: missing $metadata_path" >&2
    return 1
  }

  python3 - "$metadata_path" "$FLOOR_COUNT" "$ROOMS_PER_FLOOR" <<'PY'
import json
import sys

metadata_path, expected_floor_count, expected_rooms = sys.argv[1:]
expected_floor_count = int(expected_floor_count)
expected_rooms = int(expected_rooms)
with open(metadata_path, encoding="utf-8") as stream:
    metadata = json.load(stream)

floors = metadata.get("floors")
if not isinstance(floors, list):
    raise SystemExit("scene contract failed: metadata.floors is not a list")
if len(floors) != expected_floor_count:
    raise SystemExit(
        "scene contract failed: requested floor_count=%d, scene has %d"
        % (expected_floor_count, len(floors))
    )
room_counts = []
for index, floor in enumerate(floors):
    rooms = floor.get("rooms") if isinstance(floor, dict) else None
    count = len(rooms) if isinstance(rooms, list) else -1
    room_counts.append(count)
    if count != expected_rooms:
        raise SystemExit(
            "scene contract failed: floor_%d has %d rooms, requested %d"
            % (index, count, expected_rooms)
        )

door_specs = metadata.get("door_specs", [])
elevator_ids = {
    str(spec.get("id"))
    for spec in door_specs
    if isinstance(spec, dict) and spec.get("kind") == "elevator"
}
expected_elevator_ids = {
    "elevator_floor_%d" % index for index in range(expected_floor_count)
}
if not expected_elevator_ids.issubset(elevator_ids):
    missing = sorted(expected_elevator_ids - elevator_ids)
    raise SystemExit(
        "scene contract failed: missing elevator doors: %s" % ", ".join(missing)
    )

print(
    "scene contract PASS: floors=%d rooms_per_floor=%d room_counts=%s elevators=%d"
    % (len(floors), expected_rooms, room_counts, len(expected_elevator_ids))
)
PY
}

validate_scene_contract

python3 "$ROOT_DIR/src/danger_search_robot/scripts/elevator_floor_transition.py" \
  --state-file "$STATE_FILE" \
  --anchor-file "$ANCHOR_FILE" \
  init --current-floor 0 --floor-height 2.6 --floor-count "$FLOOR_COUNT" --force

open_door() {
  local door_id="$1"
  local attempts="${DOOR_SERVICE_ATTEMPTS:-2}"
  local attempt
  for attempt in $(seq 1 "$attempts"); do
    echo "opening door: $door_id (attempt $attempt/$attempts)"
    if timeout --foreground "${DOOR_SERVICE_TIMEOUT_SECONDS:-60}s" \
      rosservice call /set_door_state \
      "{door_id: $door_id, open: true}"; then
      return 0
    fi
    echo "door service did not return for $door_id; waiting before retry" >&2
    sleep 3
  done
  echo "failed to open door after $attempts attempts: $door_id" >&2
  return 1
}

cancel_stale_navigation() {
  # A killed Graph-NBV process can leave move_base's last action active.  If
  # set_model_state resets the robot while that goal is still live, the robot
  # moves before the next gate sample and the measured entrance origin is
  # wrong.  Cancel the action and hold the safety arbiter at zero first.
  if rostopic list 2>/dev/null | grep -Fxq /move_base/cancel; then
    echo "cancelling stale move_base goals before pose reset"
    timeout --foreground 5s \
      rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "{}" \
      >/dev/null 2>&1 || true
  fi
  timeout --foreground 2s \
    rostopic pub -r 10 /cmd_vel_safety geometry_msgs/Twist "{}" \
    >/dev/null 2>&1 || true
}

door_ids=(main_entrance)
for floor in $(seq 0 $((FLOOR_COUNT - 1))); do
  door_ids+=("elevator_floor_${floor}")
done
for door_id in "${door_ids[@]}"; do
  open_door "$door_id"
done

cancel_stale_navigation

if [ "$RESET_ROBOT_POSE" = "1" ]; then
  python3 "$ROOT_DIR/tools/set_a1_pose.py" \
    "$ROBOT_X" "$ROBOT_Y" "$ROBOT_Z" "$ROBOT_YAW"
fi

"$ROOT_DIR/tools/start_exploration_stack.sh"

# This is a new floor-0 mission, so stale projected occupancy from a previous
# run must be removed after the doors are open and the robot is stationary.
timeout --foreground "${MAP_RESET_TIMEOUT_SECONDS:-30}s" \
  rosservice call /fastlio_2d_projection/clear_map "{}"
timeout --foreground "${MAP_RESET_TIMEOUT_SECONDS:-30}s" \
  rosservice call /fastlio_2d_projection/clear_candidates "{}"
if rosservice list 2>/dev/null | grep -Fxq /move_base/clear_costmaps; then
  timeout --foreground "${MAP_RESET_TIMEOUT_SECONDS:-30}s" \
    rosservice call /move_base/clear_costmaps "{}"
fi
if rosservice list 2>/dev/null | grep -Fxq /danger_result_writer/reset; then
  timeout --foreground "${MAP_RESET_TIMEOUT_SECONDS:-30}s" \
    rosservice call /danger_result_writer/reset "{}"
fi
if rosservice list 2>/dev/null | grep -Fxq /target_manager/reset; then
  timeout --foreground "${MAP_RESET_TIMEOUT_SECONDS:-30}s" \
    rosservice call /target_manager/reset "{}"
fi

# ROS parameters survive a node restart.  Remove private mission parameters so
# a prior opt-in cannot silently change the next run.
rosparam delete /mission_manager 2>/dev/null || true
rosparam delete /graph_nbv 2>/dev/null || true
rosparam delete /nbv_sim_time_watchdog 2>/dev/null || true

echo "exploration runtime reset PASS: floor_count=$FLOOR_COUNT map=cleared doors=open pose=reset=$RESET_ROBOT_POSE state=$STATE_FILE maps=$ROS1_RUNTIME_MAPS_DIR"
