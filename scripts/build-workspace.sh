#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/internal/env.sh"
export INTRAMURAL_SOURCE_OVERLAY=0
exec "$INTRAMURAL_ROOT/scripts/run.sh" colcon build --symlink-install \
  --parallel-workers 2 --event-handlers desktop_notification- \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DFETCHCONTENT_SOURCE_DIR_SMALL_GICP=/work/cache/small_gicp \
  -DFETCHCONTENT_SOURCE_DIR_LIVOX_SDK=/work/cache/livox-sdk2 "$@"
