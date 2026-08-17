#!/usr/bin/env bash
set -euo pipefail

# Record one complete fixed three-floor replay from a camera that follows the
# simulated A1. Gazebo's GUI camera is deliberately not used: the old x11grab
# recording showed a fixed entrance/world view instead of the robot's
# perspective. The POV sensor is attached to the A1 pose and streamed from
# its ROS image topic to host-side ffmpeg.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPLAY="$ROOT_DIR/tools/replay_ros1_fixed.sh"
RESULTS_DIR="$ROOT_DIR/results"
BASELINE_FILE="$ROOT_DIR/tools/ros1_fixed_baseline.env"
# shellcheck disable=SC1090
source "$BASELINE_FILE"

CONTAINER_NAME="$BASELINE_CONTAINER_NAME"
CAMERA_MODEL="${THREE_FLOOR_POV_CAMERA_MODEL:-exploration_rgb_camera}"
CAMERA_TOPIC="${THREE_FLOOR_POV_CAMERA_TOPIC:-/exploration_camera/exploration_camera/image_raw}"
CAMERA_SDF="/root/catkin_ws/tools/exploration_rgb_camera.sdf"
FOLLOW_SCRIPT="/root/catkin_ws/tools/follow_a1_rgb_camera.py"
POV_OFFSET_X="${THREE_FLOOR_POV_OFFSET_X:-0.35}"
POV_OFFSET_Z="${THREE_FLOOR_POV_OFFSET_Z:-0.55}"
POV_FPS="${THREE_FLOOR_POV_FPS:-10}"

VIDEO_NAME="${THREE_FLOOR_VIDEO_NAME:-three_floor_robot_pov_$(date +%Y%m%d_%H%M%S).mp4}"
VIDEO_PATH="$RESULTS_DIR/$VIDEO_NAME"
TMP_VIDEO="/dev/shm/three_floor_robot_pov_$$.mp4"
STREAM_LOG="/dev/shm/three_floor_robot_pov_$$.stream.log"
FFMPEG_LOG="/dev/shm/three_floor_robot_pov_$$.ffmpeg.log"
FOLLOW_PID_FILE="/tmp/three_floor_robot_pov_$$.follow.pid"
STREAM_PID=""
RECORDING_PUBLISHED=0

die() {
  echo "three-floor robot POV recording FAILED: $*" >&2
  exit 2
}

container_bash() {
  docker exec "$CONTAINER_NAME" bash -lc "$1"
}

stop_stream() {
  if [ -n "$STREAM_PID" ] && kill -0 "$STREAM_PID" 2>/dev/null; then
    kill -INT "$STREAM_PID" 2>/dev/null || true
    wait "$STREAM_PID" 2>/dev/null || true
  fi
  STREAM_PID=""
}

cleanup_pov_camera() {
  container_bash "
    if [ -f '$FOLLOW_PID_FILE' ]; then
      follow_pid=\$(cat '$FOLLOW_PID_FILE' 2>/dev/null || true)
      if [ -n \"\$follow_pid\" ]; then kill -TERM \"\$follow_pid\" 2>/dev/null || true; fi
      rm -f '$FOLLOW_PID_FILE'
    fi
    source /opt/ros/noetic/setup.bash
    rosservice call /gazebo/delete_model \"{model_name: '$CAMERA_MODEL'}\" >/dev/null 2>&1 || true
  " >/dev/null 2>&1 || true
}

publish_recording() {
  if [ -f "$TMP_VIDEO" ]; then
    mkdir -p "$RESULTS_DIR"
    cp "$TMP_VIDEO" "$VIDEO_PATH"
    RECORDING_PUBLISHED=1
    echo "robot POV recording saved: $VIDEO_PATH"
  else
    echo "robot POV recording file was not produced: $TMP_VIDEO" >&2
  fi
}

finish_recording() {
  local status=$?
  trap - EXIT INT TERM
  stop_stream
  cleanup_pov_camera
  if [ "$RECORDING_PUBLISHED" -eq 0 ]; then
    publish_recording
  fi
  exit "$status"
}
trap finish_recording EXIT INT TERM

command -v docker >/dev/null 2>&1 || die "docker is unavailable"
command -v ffmpeg >/dev/null 2>&1 || die "host ffmpeg is unavailable"
command -v ffprobe >/dev/null 2>&1 || die "host ffprobe is unavailable"
mkdir -p "$RESULTS_DIR"

"$REPLAY" check

echo "restarting fixed headless simulation for robot POV recording"
REPLAY_GAZEBO_GUI=false "$REPLAY" restart

echo "spawning robot POV camera: model=$CAMERA_MODEL topic=$CAMERA_TOPIC"
container_bash "
  source /opt/ros/noetic/setup.bash
  source /root/catkin_ws/devel/setup.bash
  if ! rosservice call /gazebo/get_model_state \"{model_name: '$CAMERA_MODEL', relative_entity_name: world}\" 2>/dev/null | grep -q 'success: True'; then
    rosrun gazebo_ros spawn_model -sdf -file '$CAMERA_SDF' -model '$CAMERA_MODEL'
  fi
"

echo "starting camera follower: offset_x=$POV_OFFSET_X offset_z=$POV_OFFSET_Z"
container_bash "
  source /opt/ros/noetic/setup.bash
  source /root/catkin_ws/devel/setup.bash
  export POV_CAMERA_MODEL='$CAMERA_MODEL'
  export POV_CAMERA_OFFSET_X='$POV_OFFSET_X'
  export POV_CAMERA_OFFSET_Z='$POV_OFFSET_Z'
  nohup python3 '$FOLLOW_SCRIPT' >'/tmp/three_floor_robot_pov_$$.follow.log' 2>&1 < /dev/null &
  echo \$! > '$FOLLOW_PID_FILE'
"

echo "starting ROS camera stream: $CAMERA_TOPIC"
docker exec "$CONTAINER_NAME" bash -lc "
  source /opt/ros/noetic/setup.bash
  source /root/catkin_ws/devel/setup.bash
  exec python3 -u /root/catkin_ws/tools/stream_ros_image.py \
    --topic '$CAMERA_TOPIC' --fps '$POV_FPS' --wait-timeout 30
" 2>"$STREAM_LOG" |
  ffmpeg -hide_banner -loglevel warning -y \
    -f rawvideo -pixel_format rgb24 -video_size 640x480 \
    -framerate "$POV_FPS" -i - \
    -an -c:v libx264 -preset ultrafast -crf 23 \
    -pix_fmt yuv420p -movflags +faststart "$TMP_VIDEO" \
    >"$FFMPEG_LOG" 2>&1 &
STREAM_PID=$!

for _ in $(seq 1 30); do
  if grep -Fq "stream ready:" "$STREAM_LOG" 2>/dev/null; then
    break
  fi
  if ! kill -0 "$STREAM_PID" 2>/dev/null; then
    tail -n 80 "$STREAM_LOG" >&2 || true
    tail -n 80 "$FFMPEG_LOG" >&2 || true
    die "robot POV stream exited before receiving a frame"
  fi
  sleep 1
done
grep -Fq "stream ready:" "$STREAM_LOG" || {
  tail -n 80 "$STREAM_LOG" >&2 || true
  die "robot POV camera topic did not produce a frame"
}

echo "preparing three-floor exploration stack"
REPLAY_GAZEBO_GUI=false "$REPLAY" prepare

REPLAY_VIEW_HOLD_WALL_SECONDS="${THREE_FLOOR_POV_VIEW_HOLD_WALL_SECONDS:-1.5}" \
REPLAY_FINE_VIEW_HOLD_WALL_SECONDS="${THREE_FLOOR_POV_FINE_VIEW_HOLD_WALL_SECONDS:-2.0}" \
REPLAY_WALL_VIEW_HOLD_WALL_SECONDS="${THREE_FLOOR_POV_WALL_VIEW_HOLD_WALL_SECONDS:-1.5}" \
MISSION_TIMEOUT_SECONDS="${THREE_FLOOR_RECORD_TIMEOUT_SECONDS:-1800}" \
  REPLAY_GAZEBO_GUI=false "$REPLAY" run
REPLAY_GAZEBO_GUI=false "$REPLAY" score

stop_stream
ffprobe -v error \
  -show_entries format=duration:stream=codec_name,width,height \
  -of default=noprint_wrappers=1 "$TMP_VIDEO"
publish_recording
echo "full three-floor robot POV recording and score completed"
