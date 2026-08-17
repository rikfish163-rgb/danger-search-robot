#!/usr/bin/env bash
set -euo pipefail

# Record one complete fixed three-floor replay.  The mission remains the same
# as replay_ros1_fixed.sh; only Gazebo GUI is enabled and the host Xvfb display
# is captured.  The video is encoded in /dev/shm while ROS is live, then copied
# to results after the mission and official score have completed.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPLAY="$ROOT_DIR/tools/replay_ros1_fixed.sh"
RESULTS_DIR="$ROOT_DIR/results"
DISPLAY_VALUE="${ROS1_DISPLAY:-:99}"
VIDEO_NAME="${THREE_FLOOR_VIDEO_NAME:-three_floor_exploration_$(date +%Y%m%d_%H%M%S).mp4}"
VIDEO_PATH="$RESULTS_DIR/$VIDEO_NAME"
TMP_VIDEO="/dev/shm/three_floor_exploration_${$}.mp4"
FFMPEG_LOG="/dev/shm/three_floor_exploration_${$}.ffmpeg.log"
VIDEO_PID=""
RECORDING_PUBLISHED=0

die() {
  echo "three-floor recording FAILED: $*" >&2
  exit 2
}

stop_recording() {
  if [ -n "$VIDEO_PID" ] && kill -0 "$VIDEO_PID" 2>/dev/null; then
    kill -INT "$VIDEO_PID" 2>/dev/null || true
    wait "$VIDEO_PID" 2>/dev/null || true
  fi
  VIDEO_PID=""
}

publish_recording() {
  if [ -f "$TMP_VIDEO" ]; then
    mkdir -p "$RESULTS_DIR"
    cp "$TMP_VIDEO" "$VIDEO_PATH"
    RECORDING_PUBLISHED=1
    echo "recording saved: $VIDEO_PATH"
  else
    echo "recording file was not produced: $TMP_VIDEO" >&2
  fi
}

finish_recording() {
  local status=$?
  trap - EXIT INT TERM
  if [ "$RECORDING_PUBLISHED" -eq 1 ]; then
    exit "$status"
  fi
  stop_recording
  publish_recording
  exit "$status"
}
trap finish_recording EXIT INT TERM

command -v ffmpeg >/dev/null 2>&1 || die "ffmpeg is unavailable"
command -v ffprobe >/dev/null 2>&1 || die "ffprobe is unavailable"
command -v xdpyinfo >/dev/null 2>&1 || die "xdpyinfo is unavailable"
DISPLAY="$DISPLAY_VALUE" xdpyinfo >/dev/null 2>&1 || die "X display is unavailable: $DISPLAY_VALUE"
mkdir -p "$RESULTS_DIR"

"$REPLAY" check

echo "starting X11 recording on $DISPLAY_VALUE: video=$VIDEO_PATH"
ffmpeg -hide_banner -loglevel warning -y \
  -f x11grab -draw_mouse 1 -video_size 1280x1024 -framerate 8 \
  -i "$DISPLAY_VALUE.0+0,0" \
  -c:v libx264 -preset ultrafast -crf 28 -pix_fmt yuv420p \
  -movflags +faststart "$TMP_VIDEO" >"$FFMPEG_LOG" 2>&1 &
VIDEO_PID=$!
sleep 2
kill -0 "$VIDEO_PID" 2>/dev/null || {
  tail -n 80 "$FFMPEG_LOG" >&2 || true
  die "ffmpeg exited before the mission started"
}

echo "starting fixed GUI simulation"
REPLAY_GAZEBO_GUI=true "$REPLAY" restart
echo "checking that the recorded display contains a rendered Gazebo frame"
PROBE_PNG="/dev/shm/three_floor_exploration_${$}.png"
ffmpeg -hide_banner -loglevel error -y \
  -f x11grab -video_size 1280x1024 \
  -i "$DISPLAY_VALUE.0+0,0" -frames:v 1 "$PROBE_PNG"
test -s "$PROBE_PNG" || die "Gazebo display probe is empty"

REPLAY_GAZEBO_GUI=true "$REPLAY" prepare
MISSION_TIMEOUT_SECONDS="${THREE_FLOOR_RECORD_TIMEOUT_SECONDS:-3600}" \
  REPLAY_GAZEBO_GUI=true "$REPLAY" run
REPLAY_GAZEBO_GUI=true "$REPLAY" score

stop_recording
ffprobe -v error -show_entries format=duration:stream=width,height,codec_name \
  -of default=noprint_wrappers=1 "$TMP_VIDEO"
publish_recording
echo "full three-floor recording and score completed"
