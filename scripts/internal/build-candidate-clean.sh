#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
[[ $# -eq 1 && "$1" =~ ^[a-zA-Z0-9_-]+$ ]] || { echo 'Usage: scripts/internal/build-candidate-clean.sh <unique-name>' >&2; exit 2; }
prefix="/work/cache/candidate-$1"
export INTRAMURAL_SOURCE_OVERLAY=0
exec "$INTRAMURAL_ROOT/scripts/run.sh" colcon --log-base "$prefix-log" build \
  --build-base "$prefix-build" \
  --install-base "$prefix-install" \
  --symlink-install --parallel-workers 2 --event-handlers desktop_notification- \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DFETCHCONTENT_SOURCE_DIR_SMALL_GICP=/work/cache/small_gicp \
  -DFETCHCONTENT_SOURCE_DIR_LIVOX_SDK=/work/cache/livox-sdk2
