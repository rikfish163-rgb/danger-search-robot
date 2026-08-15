#!/usr/bin/env bash
set -euo pipefail

# One reproducible ROS1/Gazebo container for this workspace.  The image is
# built from a pinned ROS base and the container uses a private bridge network
# so an old host-network ROS master cannot silently join a new run.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_HOST="$(cd "$SCRIPT_DIR/.." && pwd)"
DEPS_HOST="${ROS1_DEPS_HOST:-/media/hetaisheng/044A81D94A81C83E/ros1_isolated_local/deps}"
NATIVE_BUILD_HOST="${ROS1_NATIVE_BUILD_HOST:-/dev/shm/ros1_stable}"
NATIVE_BUILD_CONTAINER="${ROS1_NATIVE_BUILD_CONTAINER:-/root/catkin_native}"
CONTAINER_NAME="${ROS1_CONTAINER_NAME:-simenv-ros1-stable}"
NETWORK_NAME="${ROS1_NETWORK_NAME:-ros1-simenv-stable}"
DEFAULT_RUNTIME_IMAGE="danger-search-robot/ros1-fixed:20260816"
IMAGE_REF="${ROS1_IMAGE_REF:-$DEFAULT_RUNTIME_IMAGE}"
DOCKERFILE_PATH="$WORKSPACE_HOST/docker/ros1-fixed/Dockerfile"

usage() {
  cat <<'EOF'
Usage: tools/ros1_fixed_container.sh {up|down|status|exec [command ...]}

Environment overrides:
  ROS1_DEPS_HOST       Read-only ROS dependency tree on the host.
  ROS1_NATIVE_BUILD_HOST  Native build/devel scratch space (default: /dev/shm/ros1_stable).
  ROS1_CONTAINER_NAME  Fixed container name (default: simenv-ros1-stable).
  ROS1_NETWORK_NAME    Private Docker bridge network name (default: ros1-simenv-stable).
  ROS1_IMAGE_REF       Pinned ROS image reference.
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
      simenv-run0810*) stale="${stale}${name}\n" ;;
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

up() {
  check_paths
  # Once the pinned container exists, it is already isolated on its private
  # network.  Only block creation of a new container; stale historical
  # containers must not prevent routine reuse of the fixed one.
  if ! container_exists; then
    check_stale_simenv
  fi
  docker network inspect "$NETWORK_NAME" >/dev/null 2>&1 || \
    docker network create --driver bridge "$NETWORK_NAME" >/dev/null

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
      --env DISPLAY="${DISPLAY:-:99}" \
      --env CATKIN_NATIVE_ROOT="$NATIVE_BUILD_CONTAINER" \
      --env CATKIN_DEVEL_SPACE=/root/catkin_ws/devel \
      --env UNITREE_CTRL_BINARY="$NATIVE_BUILD_CONTAINER/unitree_devel/lib/unitree_guide/junior_ctrl" \
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
  docker exec -it \
    --env ROS_MASTER_URI=http://127.0.0.1:11311 \
    --env ROS_HOSTNAME=localhost \
    --env DISPLAY="${DISPLAY:-:99}" \
    "$CONTAINER_NAME" "$@"
}

case "${1:-}" in
  up) shift; up "$@" ;;
  down) shift; down "$@" ;;
  status) shift; status "$@" ;;
  exec) shift; exec_in_container "$@" ;;
  *) usage >&2; exit 64 ;;
esac
