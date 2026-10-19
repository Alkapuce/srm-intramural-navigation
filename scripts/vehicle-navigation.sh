#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/internal/env.sh"
: "${INTRAMURAL_NAVIGATION_CONFIG:?Set INTRAMURAL_NAVIGATION_CONFIG to a prepared real map directory}"
mkdir -p "$INTRAMURAL_ROOT/cache/runtime"
exec 9>"$INTRAMURAL_ROOT/cache/runtime/vehicle-navigation.lock"
flock -n 9 || { echo '实车导航已在运行；请使用现有终端，或先 Ctrl-C 退出旧实例。' >&2; exit 75; }
"$INTRAMURAL_ROOT/scripts/run.sh" python3 /work/scripts/internal/relocalize.py \
  --map-config "$INTRAMURAL_NAVIGATION_CONFIG"
exec "$INTRAMURAL_ROOT/scripts/run.sh" ros2 launch /work/launch/vehicle-navigation.launch.py \
  "navigation_config:=$INTRAMURAL_NAVIGATION_CONFIG" \
  "vehicle_config:=${INTRAMURAL_VEHICLE_CONFIG:-$INTRAMURAL_ROOT/cache/vehicle-current}" "$@"
