#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/internal/env.sh"
python3 "$INTRAMURAL_ROOT/scripts/internal/prepare-map.py"
exec "$INTRAMURAL_ROOT/scripts/run.sh" bash /work/scripts/internal/graphics-entrypoint.sh \
  python3 /work/scripts/validation/verify-stage2.py "$@"
