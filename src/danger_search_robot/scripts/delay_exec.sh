#!/bin/bash
set -e

DELAY="$1"
shift

echo "[delay_exec] Waiting ${DELAY} seconds before starting:"
echo "[delay_exec] $*"

sleep "$DELAY"

exec "$@"
