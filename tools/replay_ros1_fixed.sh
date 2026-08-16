#!/usr/bin/env bash
set -euo pipefail

# One host-side entrypoint for the validated ROS1/Gazebo mission.  It keeps
# Docker, scene generation, support-stack startup, the three-floor runner, and
# the official score check on the same fixed baseline.  The command is
# intentionally fail-closed: a missing image, changed scene, stale checkout,
# or changed native executable stops before a mission can produce misleading
# results.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASELINE_FILE="$ROOT_DIR/tools/ros1_fixed_baseline.env"

# shellcheck disable=SC1090
source "$BASELINE_FILE"

CONTAINER_HELPER="$ROOT_DIR/tools/ros1_fixed_container.sh"
CONTAINER_NAME="$BASELINE_CONTAINER_NAME"
IMAGE_REF="$BASELINE_IMAGE_REF"
DISPLAY_VALUE="${ROS1_DISPLAY:-$BASELINE_DISPLAY}"

LOCKED_PATHS=(
  generated_building/building_config.json
  generated_building/competition_scene.world
  generated_building/danger_truth.json
  generated_building/door_config.yaml
  generated_building/elevator_config.yaml
  generated_building/generation_checks.json
  generated_building/layout_metadata.json
  generated_building/model.sdf
  generated_building/scene_manifest.json
  generated_building/scene_manifest.stdout.json
  generated_building/world.sdf
  src/SimEnv/src/building_generator_core/building_generator_core/exporter.py
)

usage() {
  cat <<'EOF'
Usage: tools/replay_ros1_fixed.sh {check|status|start|restart|prepare|run|score|all}

Commands:
  check    Verify the Git, scene, Docker image/container, mounts, and native binaries.
  status   Show the fixed container status without starting a mission.
  start    Idempotently start the fixed container, roscore, and Gazebo scene.
  restart  Restart the fixed container, then launch the exact seeded scene.
  prepare  Start the scene if needed and reset the exploration support stack.
  run      Run the complete three-floor scan (expects prepare to be complete).
  score    Run the official Gitee evaluator on the latest detection result.
  all      Fresh container restart + scene + prepare + full scan + official score.

The normal next-run command is:
  tools/replay_ros1_fixed.sh all

Set ALLOW_BASELINE_DRIFT=1 only for deliberate debugging of a changed checkout.
It does not change the Docker image or native-binary checks.

If /dev/shm was cleared, the native build/devel backup is restored automatically
from BASELINE_NATIVE_ARCHIVE_HOST.  If that host backup is unavailable, build
the native trees explicitly with tools/ros1_fixed_container.sh build.
EOF
}

die() {
  echo "replay baseline FAILED: $*" >&2
  exit 2
}

docker_bash_timeout() {
  local seconds="$1"
  local command="$2"
  timeout --foreground "${seconds}s" docker exec \
    --env ROS_MASTER_URI=http://127.0.0.1:11311 \
    --env ROS_HOSTNAME=localhost \
    --env GAZEBO_MASTER_URI=http://127.0.0.1:11345 \
    --env DISPLAY="$DISPLAY_VALUE" \
    --env CATKIN_NATIVE_ROOT="$BASELINE_NATIVE_BUILD_CONTAINER" \
    --env ROS1_RUNTIME_ROOT="$BASELINE_RUNTIME_ROOT_CONTAINER" \
    "$CONTAINER_NAME" bash -lc "$command"
}

docker_bash() {
  docker_bash_timeout "${DOCKER_EXEC_TIMEOUT_SECONDS:-30}" "$1"
}

container_running() {
  [ "$(docker inspect --format '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null || true)" = true ]
}

validate_git_contract() {
  git -C "$ROOT_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1 || \
    die "not a Git worktree: $ROOT_DIR"

  if ! git -C "$ROOT_DIR" merge-base --is-ancestor "$BASELINE_CODE_ANCHOR" HEAD; then
    if [ "${ALLOW_BASELINE_DRIFT:-0}" != 1 ]; then
      die "HEAD does not contain the validated code anchor $BASELINE_CODE_ANCHOR; set ALLOW_BASELINE_DRIFT=1 only for deliberate debugging"
    fi
    echo "WARNING: Git baseline drift is explicitly allowed: anchor=$BASELINE_CODE_ANCHOR"
  fi

  local staged_or_uncommitted=0
  if ! (cd "$ROOT_DIR" && git diff --quiet -- "${LOCKED_PATHS[@]}" ); then
    staged_or_uncommitted=1
  fi
  if ! (cd "$ROOT_DIR" && git diff --cached --quiet -- "${LOCKED_PATHS[@]}" ); then
    staged_or_uncommitted=1
  fi
  if [ "$staged_or_uncommitted" -ne 0 ] && [ "${ALLOW_BASELINE_DRIFT:-0}" != 1 ]; then
    die "locked scene/runtime files have local changes; commit or revert them before replay (or set ALLOW_BASELINE_DRIFT=1 deliberately)"
  fi
  if [ "$staged_or_uncommitted" -ne 0 ]; then
    echo "WARNING: locked scene/runtime files are locally modified; replay is diagnostic only"
  fi
}

validate_scene_contract() {
  python3 - \
    "$ROOT_DIR/generated_building/building_config.json" \
    "$ROOT_DIR/generated_building/scene_manifest.json" \
    "$ROOT_DIR/generated_building/layout_metadata.json" \
    "$ROOT_DIR/generated_building/danger_truth.json" \
    "$BASELINE_SCENE_SEED" "$BASELINE_FLOOR_COUNT" "$BASELINE_ROOMS_PER_FLOOR" \
    "$BASELINE_DANGER_COUNT" "$BASELINE_DISTRACTOR_COUNT" <<'PY'
import json
import sys

config_path, manifest_path, metadata_path, truth_path, seed, floors, rooms, dangers, distractors = sys.argv[1:]
seed = int(seed)
floors = int(floors)
rooms = int(rooms)
dangers = int(dangers)
distractors = int(distractors)

def load(path):
    with open(path, encoding="utf-8") as stream:
        return json.load(stream)

config = load(config_path)
manifest = load(manifest_path)
metadata = load(metadata_path)
truth = load(truth_path)

checks = {
    "building_config.seed": config.get("seed") == seed,
    "building_config.num_floors": config.get("num_floors") == floors,
    "building_config.room_count": config.get("room_count") == floors * rooms,
    "building_config.danger_count": config.get("danger_count") == dangers,
    "building_config.distractor_count": config.get("distractor_count") == distractors,
    "scene_manifest.seed": manifest.get("seed") == seed,
    "scene_manifest.danger_count": manifest.get("danger_count") == dangers,
    "scene_manifest.distractor_count": manifest.get("distractor_count") == distractors,
    "truth.seed": truth.get("seed") == seed,
    "truth.danger_sources": len(truth.get("danger_sources", [])) == dangers,
    "truth.distraction_sources": len(truth.get("distraction_sources", [])) == distractors,
    "layout.floor_count": len(metadata.get("floors", [])) == floors,
}
for index, floor in enumerate(metadata.get("floors", [])):
    checks[f"layout.floor_{index}.room_count"] = len(floor.get("rooms", [])) == rooms

failed = [name for name, passed in checks.items() if not passed]
if failed:
    raise SystemExit("scene contract failed: " + ", ".join(failed))

print(
    "scene contract PASS: seed=%d floors=%d rooms_per_floor=%d dangers=%d distractors=%d"
    % (seed, floors, rooms, dangers, distractors)
)
PY
}

require_fixed_image() {
  command -v docker >/dev/null 2>&1 || die "docker command is unavailable"
  local actual_id
  actual_id="$(docker image inspect "$IMAGE_REF" --format '{{.Id}}' 2>/dev/null || true)"
  [ -n "$actual_id" ] || die "fixed image is missing locally: $IMAGE_REF"
  [ "$actual_id" = "$BASELINE_IMAGE_ID" ] || \
    die "fixed image digest mismatch: expected=$BASELINE_IMAGE_ID actual=$actual_id"
  echo "image contract PASS: $IMAGE_REF ($actual_id)"
}

native_host_ready() {
  local controller_path="$BASELINE_NATIVE_BUILD_HOST/unitree_devel/lib/unitree_guide/junior_ctrl"
  local projection_path="$BASELINE_NATIVE_BUILD_HOST/danger_devel/lib/danger_search_robot/nearest_azimuth_projection_node"
  [ -x "$controller_path" ] && [ -x "$projection_path" ] || return 1
  [ "$(sha256sum "$controller_path" | awk '{print $1}')" = "$BASELINE_CONTROLLER_SHA256" ] || return 1
  [ "$(sha256sum "$projection_path" | awk '{print $1}')" = "$BASELINE_PROJECTION_SHA256" ] || return 1
}

ensure_native_runtime() {
  if native_host_ready; then
    echo "native host backup contract PASS: $BASELINE_NATIVE_BUILD_HOST"
    return 0
  fi

  local archive_host="${ROS1_NATIVE_ARCHIVE_HOST:-$BASELINE_NATIVE_ARCHIVE_HOST}"
  [ -f "$archive_host" ] || die "native runtime is missing or changed at $BASELINE_NATIVE_BUILD_HOST and backup is unavailable: $archive_host"
  local archive_sha
  archive_sha="$(sha256sum "$archive_host" | awk '{print $1}')"
  [ "$archive_sha" = "$BASELINE_NATIVE_ARCHIVE_SHA256" ] || \
    die "native backup SHA256 mismatch: expected=$BASELINE_NATIVE_ARCHIVE_SHA256 actual=$archive_sha"

  echo "restoring native build/devel trees into $BASELINE_NATIVE_BUILD_HOST from $archive_host"
  mkdir -p "$BASELINE_NATIVE_BUILD_HOST"
  timeout --foreground "${NATIVE_RESTORE_TIMEOUT_SECONDS:-180}s" \
    tar --zstd -xpf "$archive_host" -C "$BASELINE_NATIVE_BUILD_HOST"
  native_host_ready || die "native backup restored but binary hashes still do not match the baseline"
  echo "native host restore PASS: controller=$BASELINE_CONTROLLER_SHA256 projection=$BASELINE_PROJECTION_SHA256"
}

validate_container_contract() {
  container_running || die "fixed container is not running: $CONTAINER_NAME"

  local actual_id actual_network mounts expected_mount
  actual_id="$(docker inspect --format '{{.Image}}' "$CONTAINER_NAME")"
  [ "$actual_id" = "$BASELINE_IMAGE_ID" ] || \
    die "container image mismatch: expected=$BASELINE_IMAGE_ID actual=$actual_id"

  actual_network="$(docker inspect --format '{{.HostConfig.NetworkMode}}' "$CONTAINER_NAME")"
  [ "$actual_network" = "$BASELINE_NETWORK_NAME" ] || \
    die "container network mismatch: expected=$BASELINE_NETWORK_NAME actual=$actual_network"

  mounts="$(docker inspect --format '{{range .Mounts}}{{.Source}}=>{{.Destination}}{{"\n"}}{{end}}' "$CONTAINER_NAME")"
  for expected_mount in \
    "$ROOT_DIR=>/root/catkin_ws" \
    "$BASELINE_DEPS_HOST=>/root/ros1_isolated/deps" \
    "$BASELINE_NATIVE_BUILD_HOST=>$BASELINE_NATIVE_BUILD_CONTAINER"; do
    printf '%s\n' "$mounts" | grep -Fqx "$expected_mount" || \
      die "container mount missing: $expected_mount"
  done
  echo "container contract PASS: name=$CONTAINER_NAME network=$actual_network mounts=3"
}

validate_native_contract() {
  local controller projection
  controller="$(docker_bash "sha256sum '$BASELINE_NATIVE_BUILD_CONTAINER/unitree_devel/lib/unitree_guide/junior_ctrl'" | awk '{print $1}')"
  projection="$(docker_bash "sha256sum '$BASELINE_NATIVE_BUILD_CONTAINER/danger_devel/lib/danger_search_robot/nearest_azimuth_projection_node'" | awk '{print $1}')"
  [ "$controller" = "$BASELINE_CONTROLLER_SHA256" ] || \
    die "junior_ctrl SHA256 mismatch: expected=$BASELINE_CONTROLLER_SHA256 actual=$controller"
  [ "$projection" = "$BASELINE_PROJECTION_SHA256" ] || \
    die "projection node SHA256 mismatch: expected=$BASELINE_PROJECTION_SHA256 actual=$projection"
  echo "native binary contract PASS: junior_ctrl=$controller projection=$projection"
}

ensure_container() {
  validate_git_contract
  validate_scene_contract
  require_fixed_image
  ensure_native_runtime
  "$CONTAINER_HELPER" up
  "$CONTAINER_HELPER" check
  validate_container_contract
  validate_native_contract
}

ros_master_ready() {
  docker_bash_timeout 8 'source /opt/ros/noetic/setup.bash; timeout --foreground 5s rosservice list >/dev/null 2>&1'
}

gazebo_ready() {
  docker_bash_timeout 8 'source /opt/ros/noetic/setup.bash; timeout --foreground 5s rosservice list 2>/dev/null | grep -Fxq /gazebo/get_world_properties'
}

robot_ready() {
  docker_bash_timeout 8 "source /opt/ros/noetic/setup.bash; timeout --foreground 5s rosservice call /gazebo/get_model_state \"{model_name: 'a1_gazebo', relative_entity_name: 'world'}\" 2>/dev/null | grep -q 'success: True'"
}

wait_for_master() {
  local deadline=$((SECONDS + ${ROS_MASTER_TIMEOUT_SECONDS:-45}))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if ros_master_ready; then
      echo "roscore ready"
      return 0
    fi
    sleep 1
  done
  docker_bash 'tail -n 80 /tmp/roscore_fixed.log 2>/dev/null || true' >&2 || true
  die "ROS master did not become ready"
}

wait_for_robot() {
  local deadline=$((SECONDS + ${SIM_START_TIMEOUT_SECONDS:-240}))
  local last_report=0
  while [ "$SECONDS" -lt "$deadline" ]; do
    if robot_ready; then
      echo "Gazebo robot ready: a1_gazebo"
      return 0
    fi
    if [ $((SECONDS - last_report)) -ge 10 ]; then
      echo "waiting for seeded Gazebo scene and robot..."
      last_report="$SECONDS"
    fi
    sleep 1
  done
  docker_bash 'tail -n 100 /tmp/ros1_fixed_auto.log 2>/dev/null || true' >&2 || true
  die "Gazebo robot did not become ready within ${SIM_START_TIMEOUT_SECONDS:-240}s"
}

start_roscore() {
  if ros_master_ready; then
    echo "roscore already ready"
    return 0
  fi
  echo "starting roscore inside $CONTAINER_NAME"
  docker_bash 'source /opt/ros/noetic/setup.bash; if ! pgrep -f "[r]osmaster" >/dev/null 2>&1; then nohup roscore > /tmp/roscore_fixed.log 2>&1 < /dev/null & echo $! > /tmp/roscore_fixed.pid; fi'
  wait_for_master
}

launch_seeded_sim() {
  echo "launching seeded Gazebo scene: seed=$BASELINE_SCENE_SEED"
  docker_bash "nohup env \
    SEED=$BASELINE_SCENE_SEED \
    FLOOR_COUNT=$BASELINE_FLOOR_COUNT \
    ROOMS_PER_FLOOR=$BASELINE_ROOMS_PER_FLOOR \
    BUILDING_WIDTH=20.0 BUILDING_LENGTH=36.0 \
    DANGER_COUNT=$BASELINE_DANGER_COUNT_SPEC \
    DISTRACTOR_COUNT=$BASELINE_DISTRACTOR_COUNT_SPEC \
    GUI=false PAUSED=true AUTO_UNPAUSE=1 AUTO_UNPAUSE_DELAY=6 \
    START_CONTROLLER=1 CONTROLLER_FOREGROUND=0 START_VIRTUAL_JOY=0 \
    START_BUILDING_CONTROL=1 ENABLE_SENSOR_DATA=1 ENABLE_LIVOX=1 \
    ENABLE_REALSENSE=1 ENABLE_REALSENSE_ROS_PLUGIN=0 ENABLE_FRONT_CAMERA=0 \
    START_CAMERA_BRIDGES=1 ENABLE_REFEREE_ODOM=1 ENABLE_GROUND_TRUTH=1 \
    ENABLE_POINTCLOUD_CONVERTER=1 POINTCLOUD_USE_GROUND_TRUTH_ODOM=1 \
    PUBLISH_ODOM_TF=0 WRITE_GENERATED_TRUTH_COPY=1 \
    ROBOT_X=$BASELINE_ROBOT_X ROBOT_Y=$BASELINE_ROBOT_Y \
    ROBOT_Z=$BASELINE_ROBOT_Z ROBOT_YAW=$BASELINE_ROBOT_YAW \
    src/SimEnv/auto.sh > /tmp/ros1_fixed_auto.log 2>&1 < /dev/null & echo \$! > /tmp/ros1_fixed_auto.pid"
  wait_for_robot
}

start_sim() {
  start_roscore
  if gazebo_ready && robot_ready; then
    echo "Gazebo scene already ready; keeping the running fixed instance"
    return 0
  fi
  launch_seeded_sim
}

restart_container_and_sim() {
  ensure_container
  if container_running; then
    echo "restarting fixed container to remove stale ROS/Gazebo processes"
    timeout --foreground "${DOCKER_RESTART_TIMEOUT_SECONDS:-30}s" \
      docker restart --time 10 "$CONTAINER_NAME" >/dev/null || \
      die "fixed container restart timed out; inspect $CONTAINER_NAME before retrying"
  else
    "$CONTAINER_HELPER" up >/dev/null
  fi
  start_roscore
  launch_seeded_sim
}

prepare_stack() {
  start_sim
  echo "preparing three-floor exploration stack"
  docker_bash_timeout "${PREPARE_TIMEOUT_SECONDS:-300}" "export \
    FLOOR_COUNT=$BASELINE_FLOOR_COUNT \
    ROOMS_PER_FLOOR=$BASELINE_ROOMS_PER_FLOOR \
    RESET_ROBOT_POSE=1 \
    ROBOT_X=$BASELINE_ROBOT_X ROBOT_Y=$BASELINE_ROBOT_Y \
    ROBOT_Z=$BASELINE_ROBOT_Z ROBOT_YAW=$BASELINE_ROBOT_YAW; \
    tools/prepare_exploration_stack.sh"
}

run_mission() {
  if ! ros_master_ready || ! gazebo_ready || ! robot_ready; then
    die "ROS/Gazebo is not ready; run tools/replay_ros1_fixed.sh prepare first"
  fi
  local runtime_dir="${THREE_FLOOR_RUNTIME_DIR:-/tmp/three_floor_replay_$(date +%Y%m%d_%H%M%S)}"
  echo "running full three-floor mission: runtime=$runtime_dir"
  docker_bash_timeout "${MISSION_TIMEOUT_SECONDS:-1200}" "export \
    EXPECTED_DANGER_COUNT=$BASELINE_DANGER_COUNT \
    FINE_ALL_ROOMS=1 FINE_WALL_VIEWS=1 \
    VIEW_HOLD_WALL_SECONDS=4.0 \
    FINE_VIEW_HOLD_WALL_SECONDS=4.0 \
    WALL_VIEW_HOLD_WALL_SECONDS=3.0 \
    THREE_FLOOR_RUNTIME_DIR=$runtime_dir; \
    tools/run_three_floor_rerun.sh"
}

validate_score() {
  local score_path="$ROOT_DIR/results/evaluation_result.json"
  [ -f "$score_path" ] || die "score output is missing: $score_path"
  python3 - "$score_path" "$BASELINE_DANGER_COUNT" "$BASELINE_EXPECTED_SCORE" <<'PY'
import json
import sys

path, expected_count, expected_score = sys.argv[1:]
expected_count = int(expected_count)
expected_score = float(expected_score)
with open(path, encoding="utf-8") as stream:
    data = json.load(stream)
metrics = data.get("metrics", {})
scores = data.get("scores", {})
checks = {
    "truth_count": metrics.get("truth_count") == expected_count,
    "detected_count": metrics.get("detected_count") == expected_count,
    "correct": metrics.get("correct") == expected_count,
    "missed": metrics.get("missed") == 0,
    "false_alarms": metrics.get("false_alarms") == 0,
    "technical_objective_total": abs(float(scores.get("technical_objective_total", -1)) - expected_score) < 1e-6,
}
failed = [name for name, passed in checks.items() if not passed]
if failed:
    raise SystemExit("score contract failed: " + ", ".join(failed))
print(
    "score contract PASS: truth=%d detected=%d correct=%d missed=%d false_alarms=%d total=%.1f time=%.2f"
    % (
        metrics["truth_count"], metrics["detected_count"], metrics["correct"],
        metrics["missed"], metrics["false_alarms"],
        float(scores["technical_objective_total"]),
        float(metrics.get("exploration_time", 0.0)),
    )
)
PY
}

score_mission() {
  echo "running official Gitee evaluator"
  docker_bash_timeout "${SCORE_TIMEOUT_SECONDS:-60}" 'tools/evaluate_gitee_score.sh --verbose'
  validate_score
}

cmd_check() {
  ensure_container
  echo "fixed replay baseline PASS: id=$BASELINE_ID branch=$(git -C "$ROOT_DIR" branch --show-current)"
}

cmd_status() {
  echo "baseline=$BASELINE_ID image=$IMAGE_REF container=$CONTAINER_NAME network=$BASELINE_NETWORK_NAME"
  "$CONTAINER_HELPER" status
}

cmd_start() {
  ensure_container
  start_sim
}

cmd_restart() {
  restart_container_and_sim
}

cmd_prepare() {
  ensure_container
  prepare_stack
}

cmd_run() {
  ensure_container
  run_mission
}

cmd_score() {
  ensure_container
  score_mission
}

cmd_all() {
  cmd_check
  restart_container_and_sim
  prepare_stack
  run_mission
  score_mission
}

case "${1:-}" in
  check) cmd_check ;;
  status) cmd_status ;;
  start) cmd_start ;;
  restart) cmd_restart ;;
  prepare) cmd_prepare ;;
  run) cmd_run ;;
  score) cmd_score ;;
  all) cmd_all ;;
  *) usage >&2; exit 64 ;;
esac
