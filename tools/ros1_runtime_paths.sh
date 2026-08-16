#!/usr/bin/env bash

# Runtime-only files must not be written to the NTFS bind mount while ROS is
# live.  The fixed container maps /dev/shm/ros1_recovery to
# ${CATKIN_NATIVE_ROOT}; callers may override ROS1_RUNTIME_ROOT for another
# native filesystem during an isolated run.

if [ -z "${ROS1_RUNTIME_ROOT:-}" ]; then
  if [ -n "${CATKIN_NATIVE_ROOT:-}" ]; then
    ROS1_RUNTIME_ROOT="$CATKIN_NATIVE_ROOT/ros1_runtime"
  else
    ROS1_RUNTIME_ROOT="/tmp/ros1_runtime"
  fi
fi

export ROS1_RUNTIME_ROOT
export ROS1_RUNTIME_STATE_DIR="${ROS1_RUNTIME_STATE_DIR:-$ROS1_RUNTIME_ROOT/mission_state}"
export ROS1_RUNTIME_MAPS_DIR="${ROS1_RUNTIME_MAPS_DIR:-$ROS1_RUNTIME_ROOT/floors}"
export ROS1_RUNTIME_RESULT_FILE="${ROS1_RUNTIME_RESULT_FILE:-$ROS1_RUNTIME_ROOT/detected_danger.json}"

mkdir -p "$ROS1_RUNTIME_STATE_DIR" "$ROS1_RUNTIME_MAPS_DIR"
