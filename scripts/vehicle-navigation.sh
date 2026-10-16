#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/internal/env.sh"
: "${INTRAMURAL_NAVIGATION_CONFIG:?Set INTRAMURAL_NAVIGATION_CONFIG to a prepared real map directory}"
exec "$INTRAMURAL_ROOT/scripts/run.sh" ros2 launch /work/launch/vehicle-navigation.launch.py \
  "navigation_config:=$INTRAMURAL_NAVIGATION_CONFIG" "$@"
