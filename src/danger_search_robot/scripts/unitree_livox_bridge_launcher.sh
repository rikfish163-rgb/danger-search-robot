#!/bin/bash
set -e

PKG_PATH="$(rospack find danger_search_robot)"

exec /usr/bin/python3 -u \
  "$PKG_PATH/sensor_adapter/scripts/unitree_livox_bridge.py" \
  "$@"
