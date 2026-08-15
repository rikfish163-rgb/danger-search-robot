#!/usr/bin/env bash
set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_DIR="${THREE_FLOOR_RUNTIME_DIR:-/tmp/three_floor_runtime_$$}"
LOCK_PATH="${THREE_FLOOR_LOCK_PATH:-/tmp/three_floor_rerun.lock}"
mkdir -p "$RUNTIME_DIR"
export THREE_FLOOR_RUNTIME_DIR="$RUNTIME_DIR"

exec 9>"$LOCK_PATH"
if ! flock -n 9; then
  echo "another three-floor runner is already active: $LOCK_PATH" >&2
  exit 75
fi

if ! "$ROOT_DIR/tools/check_exploration_stack.sh"; then
  echo "exploration stack preflight failed; refusing to start the mission" >&2
  exit 78
fi

python3 "$ROOT_DIR/results/full_three_floor_rerun.py"
run_status=$?

"$ROOT_DIR/tools/publish_three_floor_runtime.sh"
publish_status=$?

if [ "$run_status" -ne 0 ]; then
  exit "$run_status"
fi
exit "$publish_status"
