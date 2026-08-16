#!/usr/bin/env bash
set -euo pipefail

# Run after full_three_floor_rerun.py exits.  Keeping this outside the ROS
# process prevents a slow NTFS bind mount from holding ROS callbacks open.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_DIR="${THREE_FLOOR_RUNTIME_DIR:-/tmp/three_floor_runtime}"
RESULTS_DIR="${THREE_FLOOR_RESULTS_DIR:-$ROOT_DIR/results}"
PUBLISH_TIMEOUT_SECONDS="${THREE_FLOOR_PUBLISH_TIMEOUT_SECONDS:-20}"
source "$ROOT_DIR/tools/ros1_runtime_paths.sh"
FLOOR_STATE_FILE="${FLOOR_STATE_FILE:-$ROS1_RUNTIME_STATE_DIR/floor_state.json}"
FLOOR_ANCHOR_FILE="${FLOOR_ANCHOR_FILE:-$ROS1_RUNTIME_STATE_DIR/floor_transition_anchor.json}"

mkdir -p "$RESULTS_DIR"

publish_one() {
  local source_path="$1"
  local target_path="$2"
  local temporary_path="${target_path}.tmp.$$"

  if [ ! -s "$source_path" ]; then
    echo "missing runtime artifact: $source_path" >&2
    return 1
  fi
  rm -f "$temporary_path"
  if ! timeout --foreground "${PUBLISH_TIMEOUT_SECONDS}s" \
      cp -- "$source_path" "$temporary_path"; then
    echo "timed out publishing $source_path -> $target_path" >&2
    return 1
  fi
  chmod 0644 "$temporary_path" || true
  if ! timeout --foreground "${PUBLISH_TIMEOUT_SECONDS}s" \
      mv -f -- "$temporary_path" "$target_path"; then
    echo "timed out committing $temporary_path -> $target_path" >&2
    return 1
  fi
  echo "published $target_path"
}

failed=0
publish_one "$RUNTIME_DIR/full_three_floor_summary.json" \
  "$RESULTS_DIR/full_three_floor_summary.json" || failed=1
publish_one "$RUNTIME_DIR/retry_three_floor_full_run.log" \
  "$RESULTS_DIR/retry_three_floor_full_run.log" || failed=1
publish_one "$ROS1_RUNTIME_RESULT_FILE" \
  "$RESULTS_DIR/detected_danger.json" || failed=1
publish_one "$FLOOR_STATE_FILE" \
  "$RESULTS_DIR/floor_state.json" || failed=1
publish_one "$FLOOR_ANCHOR_FILE" \
  "$RESULTS_DIR/floor_transition_anchor.json" || failed=1
exit "$failed"
