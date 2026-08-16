#!/usr/bin/env bash
set -euo pipefail

# One reproducible ROS1/Gazebo container for this workspace.  The image is
# built from a pinned ROS base and the container uses a private bridge network
# so an old host-network ROS master cannot silently join a new run.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_HOST="$(cd "$SCRIPT_DIR/.." && pwd)"
DEPS_HOST="${ROS1_DEPS_HOST:-/media/hetaisheng/044A81D94A81C83E/ros1_isolated_local/deps}"
# The recovery runtime is the only container whose image, native binaries,
# private network, and live validation have been verified together.  Keeping
# these defaults aligned prevents `status`/`prepare` from silently operating
# on an older idle container while the recovery container is running.
NATIVE_BUILD_HOST="${ROS1_NATIVE_BUILD_HOST:-/dev/shm/ros1_recovery}"
NATIVE_BUILD_CONTAINER="${ROS1_NATIVE_BUILD_CONTAINER:-/root/catkin_native}"
RUNTIME_ROOT_CONTAINER="${ROS1_RUNTIME_ROOT_CONTAINER:-$NATIVE_BUILD_CONTAINER/ros1_runtime}"
CONTAINER_NAME="${ROS1_CONTAINER_NAME:-simenv-ros1-recovery}"
NETWORK_NAME="${ROS1_NETWORK_NAME:-ros1-simenv-recovery}"
DEFAULT_RUNTIME_IMAGE="danger-search-robot/ros1-fixed:20260816"
IMAGE_REF="${ROS1_IMAGE_REF:-$DEFAULT_RUNTIME_IMAGE}"
DOCKERFILE_PATH="$WORKSPACE_HOST/docker/ros1-fixed/Dockerfile"
XVFB_DISPLAY="${ROS1_XVFB_DISPLAY:-:99}"

ensure_xvfb_display() {
  local display="$1"
  if [[ ! "$display" =~ ^:([0-9]+)$ ]]; then
    return 0
  fi

  local socket_path="/tmp/.X11-unix/X${BASH_REMATCH[1]}"
  if [ -S "$socket_path" ]; then
    return 0
  fi
  command -v Xvfb >/dev/null 2>&1 || {
    echo "ROS1 display is unavailable and host Xvfb is not installed: $display" >&2
    return 1
  }

  local log_path="/tmp/ros1-fixed-xvfb-${BASH_REMATCH[1]}.log"
  nohup Xvfb "$display" -screen 0 1280x1024x24 -ac -nolisten tcp \
    >"$log_path" 2>&1 &
  local deadline=$((SECONDS + 10))
  while [ "$SECONDS" -lt "$deadline" ]; do
    [ -S "$socket_path" ] && return 0
    sleep 0.2
  done
  echo "failed to start Xvfb display $display; log=$log_path" >&2
  return 1
}

container_display() {
  # Gazebo camera sensors require a render context even when its GUI is off.
  # Use an explicit display only when the operator supplied one; otherwise a
  # dedicated, unauthenticated Xvfb avoids inheriting an inaccessible desktop
  # display into the container.
  if [ -n "${ROS1_DISPLAY:-}" ]; then
    printf '%s\n' "$ROS1_DISPLAY"
    return 0
  fi
  ensure_xvfb_display "$XVFB_DISPLAY"
  printf '%s\n' "$XVFB_DISPLAY"
}

usage() {
  cat <<'EOF'
Usage: tools/ros1_fixed_container.sh {up|down|status|build|check|prepare|exec [command ...]}

Environment overrides:
  ROS1_DEPS_HOST       Read-only ROS dependency tree on the host.
  ROS1_NATIVE_BUILD_HOST  Native build/devel scratch space (default: /dev/shm/ros1_recovery).
  ROS1_RUNTIME_ROOT_CONTAINER  Runtime-only state root (default: /root/catkin_native/ros1_runtime).
  ROS1_CONTAINER_NAME  Fixed container name (default: simenv-ros1-recovery).
  ROS1_NETWORK_NAME    Private Docker bridge network name (default: ros1-simenv-recovery).
  ROS1_IMAGE_REF       Pinned ROS image reference.
  ROS1_DISPLAY         Explicit X display; otherwise start/use ROS1_XVFB_DISPLAY.
  ROS1_XVFB_DISPLAY    Dedicated Xvfb display (default: :99).
  FLOOR_COUNT           Scene floor count for prepare (default: 3).
  ROOMS_PER_FLOOR       Scene room count per floor for prepare (default: 4).
  RESET_ROBOT_POSE      Reset the initial Gazebo pose during prepare (default: 1).
  ALLOW_SHARED_SIMENV=1  Bypass the stale simenv-run0810 conflict guard.
EOF
}

container_exists() {
  docker inspect "$CONTAINER_NAME" >/dev/null 2>&1
}

container_running() {
  [ "$(docker inspect --format '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null || true)" = true ]
}

check_paths() {
  [ -d "$WORKSPACE_HOST" ] || { echo "workspace missing: $WORKSPACE_HOST" >&2; exit 2; }
  [ -d "$DEPS_HOST" ] || { echo "ROS deps missing: $DEPS_HOST" >&2; exit 2; }
  mkdir -p "$NATIVE_BUILD_HOST"
  if ! docker image inspect "$IMAGE_REF" >/dev/null 2>&1; then
    if [ "$IMAGE_REF" != "$DEFAULT_RUNTIME_IMAGE" ]; then
      echo "ROS1_IMAGE_REF is not available locally: $IMAGE_REF" >&2
      exit 2
    fi
    [ -f "$DOCKERFILE_PATH" ] || {
      echo "fixed runtime Dockerfile missing: $DOCKERFILE_PATH" >&2
      exit 2
    }
    echo "building fixed ROS1 runtime image: $IMAGE_REF"
    docker build --pull=false --tag "$IMAGE_REF" \
      --file "$DOCKERFILE_PATH" "$WORKSPACE_HOST/docker/ros1-fixed"
  fi
}

check_stale_simenv() {
  [ "${ALLOW_SHARED_SIMENV:-0}" = 1 ] && return
  local stale=""
  while IFS= read -r name; do
    [ -n "$name" ] || continue
    [ "$name" = "$CONTAINER_NAME" ] && continue
    case "$name" in
      simenv-run0810*|simenv-ros1-*)
        stale="${stale}${name}\n"
        ;;
    esac
  done < <(docker ps --format '{{.Names}}')
  if [ -n "$stale" ]; then
    printf 'refusing to start: stale ROS1 containers are still running:\n%b' "$stale" >&2
    echo 'Stop/quarantine those exact containers first, or set ALLOW_SHARED_SIMENV=1 deliberately.' >&2
    exit 3
  fi
}

status() {
  if ! container_exists; then
    echo "container=$CONTAINER_NAME absent"
    return 0
  fi
  docker inspect "$CONTAINER_NAME" --format \
    'container={{.Name}} id={{.Id}} image={{.Config.Image}} state={{.State.Status}} started={{.State.StartedAt}} network={{.HostConfig.NetworkMode}}'
  docker inspect "$CONTAINER_NAME" --format \
    '{{range .Mounts}}{{.Source}}=>{{.Destination}} mode={{.Mode}} rw={{.RW}}{{"\n"}}{{end}}'
}

runtime_check() {
  container_running || {
    echo "runtime check failed: container is not running: $CONTAINER_NAME" >&2
    return 5
  }

  local expected_id actual_id
  expected_id="$(docker image inspect "$IMAGE_REF" --format '{{.Id}}')"
  actual_id="$(docker inspect --format '{{.Image}}' "$CONTAINER_NAME")"
  if [ "$expected_id" != "$actual_id" ]; then
    echo "runtime check failed: image digest mismatch" >&2
    echo "expected=$expected_id actual=$actual_id" >&2
    return 4
  fi

  # Keep this bounded: a broken Docker/containerd task must be reported as
  # unavailable instead of making the operator wait on an uninterruptible
  # docker exec forever.
  timeout --foreground 15s docker exec "$CONTAINER_NAME" bash -lc '
    set -euo pipefail
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash
    test "${ROS_DISTRO:-}" = noetic
    test "${ROS_VERSION:-}" = 1
    test -x /root/catkin_native/unitree_devel/lib/unitree_guide/junior_ctrl
    test -x /root/catkin_native/danger_devel/lib/danger_search_robot/nearest_azimuth_projection_node
    rospack find move_base >/dev/null
    rospack find teb_local_planner >/dev/null
    printf "runtime contract PASS: ROS=%s.%s native_controller=%s native_projection=%s move_base=%s teb=%s\\n" \
      "$ROS_VERSION" "$ROS_DISTRO" \
      /root/catkin_native/unitree_devel/lib/unitree_guide/junior_ctrl \
      /root/catkin_native/danger_devel/lib/danger_search_robot/nearest_azimuth_projection_node \
      "$(rospack find move_base)" "$(rospack find teb_local_planner)"
  '
}

build_native() {
  container_running || {
    echo "native build failed: container is not running: $CONTAINER_NAME" >&2
    return 5
  }
  timeout --foreground 900s docker exec "$CONTAINER_NAME" bash -lc \
    'source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash; /root/catkin_ws/tools/build_ros1_native.sh'
}

prepare_runtime() {
  container_running || {
    echo "runtime prepare failed: container is not running: $CONTAINER_NAME" >&2
    return 5
  }
  local display_value
  display_value="$(container_display)"
  timeout --foreground 300s docker exec \
    --env DISPLAY="$display_value" \
    --env FLOOR_COUNT="${FLOOR_COUNT:-3}" \
    --env ROOMS_PER_FLOOR="${ROOMS_PER_FLOOR:-4}" \
    --env RESET_ROBOT_POSE="${RESET_ROBOT_POSE:-1}" \
    --env ROS1_RUNTIME_ROOT="$RUNTIME_ROOT_CONTAINER" \
    --env ROBOT_X="${ROBOT_X:-0.0}" \
    --env ROBOT_Y="${ROBOT_Y:--3.2}" \
    --env ROBOT_Z="${ROBOT_Z:-0.6}" \
    --env ROBOT_YAW="${ROBOT_YAW:-1.5708}" \
    "$CONTAINER_NAME" bash -lc \
    'source /opt/ros/noetic/setup.bash; source /root/catkin_ws/devel/setup.bash; /root/catkin_ws/tools/prepare_exploration_stack.sh'
}

up() {
  check_paths
  # A private ROS network prevents cross-master topic leakage, but it does not
  # prevent old host-network simulations from consuming CPU or writing to the
  # same bind mount.  Always fail closed until those historical runs are
  # stopped (or explicitly overridden).
  check_stale_simenv
  docker network inspect "$NETWORK_NAME" >/dev/null 2>&1 || \
    docker network create --driver bridge "$NETWORK_NAME" >/dev/null
  local display_value
  display_value="$(container_display)"

  if container_exists; then
    local expected_id actual_id
    expected_id="$(docker image inspect "$IMAGE_REF" --format '{{.Id}}')"
    actual_id="$(docker inspect --format '{{.Image}}' "$CONTAINER_NAME")"
    if [ "$expected_id" != "$actual_id" ]; then
      echo "container exists with a different image: $CONTAINER_NAME" >&2
      echo "expected=$expected_id actual=$actual_id" >&2
      exit 4
    fi
    if ! container_running; then
      docker start "$CONTAINER_NAME" >/dev/null
    fi
  else
    local x11_args=()
    if [ -d /tmp/.X11-unix ]; then
      x11_args+=(--mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix)
    fi
    docker run -d --init \
      --name "$CONTAINER_NAME" \
      --hostname "$CONTAINER_NAME" \
      --network "$NETWORK_NAME" \
      --workdir /root/catkin_ws \
      --shm-size=2g \
      --env ROS_MASTER_URI=http://127.0.0.1:11311 \
      --env ROS_HOSTNAME=localhost \
      --env GAZEBO_MASTER_URI=http://127.0.0.1:11345 \
      --env DISPLAY="$display_value" \
      --env CATKIN_NATIVE_ROOT="$NATIVE_BUILD_CONTAINER" \
      --env ROS1_RUNTIME_ROOT="$RUNTIME_ROOT_CONTAINER" \
      --env CATKIN_DEVEL_SPACE=/root/catkin_ws/devel \
      --env UNITREE_CTRL_BINARY="$NATIVE_BUILD_CONTAINER/unitree_devel/lib/unitree_guide/junior_ctrl" \
      --env DANGER_PROJECTION_BINARY="$NATIVE_BUILD_CONTAINER/danger_devel/lib/danger_search_robot/nearest_azimuth_projection_node" \
      --mount type=bind,src="$WORKSPACE_HOST",dst=/root/catkin_ws \
      --mount type=bind,src="$DEPS_HOST",dst=/root/ros1_isolated/deps,readonly \
      --mount type=bind,src="$NATIVE_BUILD_HOST",dst="$NATIVE_BUILD_CONTAINER" \
      "${x11_args[@]}" \
      "$IMAGE_REF" tail -f /dev/null >/dev/null
  fi
  status
}

down() {
  if container_exists && container_running; then
    docker stop --time 10 "$CONTAINER_NAME" >/dev/null
  fi
  status
}

exec_in_container() {
  container_running || { echo "container is not running: $CONTAINER_NAME" >&2; exit 5; }
  if [ "$#" -eq 0 ]; then
    set -- bash
  fi
  local display_value
  display_value="$(container_display)"
  docker exec -it \
    --env ROS_MASTER_URI=http://127.0.0.1:11311 \
    --env ROS_HOSTNAME=localhost \
    --env DISPLAY="$display_value" \
    --env ROS1_RUNTIME_ROOT="$RUNTIME_ROOT_CONTAINER" \
    "$CONTAINER_NAME" "$@"
}

case "${1:-}" in
  up) shift; up "$@" ;;
  down) shift; down "$@" ;;
  status) shift; status "$@" ;;
  build) shift; build_native "$@" ;;
  check) shift; check_paths; runtime_check "$@" ;;
  prepare) shift; check_paths; prepare_runtime "$@" ;;
  exec) shift; exec_in_container "$@" ;;
  *) usage >&2; exit 64 ;;
esac
