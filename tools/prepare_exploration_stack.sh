#!/usr/bin/env bash
set -euo pipefail

# Reset only low-frequency mission state and make all doors usable.  It does
# not kill ROS/Gazebo processes; use auto.sh or restart the fixed container
# when a completely new simulation instance is required.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$ROOT_DIR/devel/setup.bash"

python3 "$ROOT_DIR/src/danger_search_robot/scripts/elevator_floor_transition.py" \
  --state-file "$ROOT_DIR/results/floor_state.json" \
  --anchor-file "$ROOT_DIR/results/floor_transition_anchor.json" \
  init --current-floor 0 --floor-height 2.6 --floor-count 3 --force

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

for door_id in main_entrance elevator_floor_0 elevator_floor_1 elevator_floor_2; do
  open_door "$door_id"
done

"$ROOT_DIR/tools/start_exploration_stack.sh"
