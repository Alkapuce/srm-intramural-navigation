#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/internal/env.sh"
exec "$INTRAMURAL_ROOT/scripts/run.sh" ros2 launch /work/launch/vehicle-sensors.launch.py \
  "vehicle_config:=${INTRAMURAL_VEHICLE_CONFIG:-/work/cache/vehicle-srm-reference}" "$@"
