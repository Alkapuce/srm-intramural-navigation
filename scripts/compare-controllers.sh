#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
revision="$(git -C "$root" rev-parse HEAD)"
export INTRAMURAL_ROS_DOMAIN_ID="${INTRAMURAL_ROS_DOMAIN_ID:-96}"
exec "$root/scripts/run.sh" env INTRAMURAL_SOURCE_REVISION="$revision" \
  python3 /work/scripts/validation/compare-controllers.py "$@"
