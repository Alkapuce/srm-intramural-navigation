#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/internal/env.sh"
[[ $# == 1 ]] || { echo 'Usage: scripts/site-mapping.sh <new bag directory>'; exit 2; }
output="$(realpath -m -- "$1")"
case "$output" in
  "$INTRAMURAL_ROOT"/artifacts/*|"$INTRAMURAL_ROOT"/cache/*) ;;
  *) echo 'Bag output must be under project artifacts/ or cache/' >&2; exit 2 ;;
esac
# WAL storage prevents a concurrent reader from aborting the recording writer.
exec "$INTRAMURAL_ROOT/scripts/run.sh" ros2 bag record --storage sqlite3 --storage-preset-profile resilient \
  -o "/work/${output#"$INTRAMURAL_ROOT"/}" \
  /livox/lidar /livox/imu /intramural_robot/aft_mapped_to_init \
  /intramural_robot/cloud_registered /tf /tf_static
