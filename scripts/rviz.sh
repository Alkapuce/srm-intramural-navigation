#!/usr/bin/env bash
set -euo pipefail
exec "$(dirname -- "${BASH_SOURCE[0]}")/run.sh" bash /work/scripts/internal/graphics-entrypoint.sh rviz2 "$@"
