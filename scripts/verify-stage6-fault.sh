#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/internal/env.sh"
export INTRAMURAL_TRANSPORT=serial-sim
"$INTRAMURAL_ROOT/scripts/run.sh" python3 /work/scripts/internal/generate-arena.py
exec "$INTRAMURAL_ROOT/scripts/run.sh" bash /work/scripts/internal/graphics-entrypoint.sh \
  python3 /work/scripts/validation/verify-stage6-fault.py "$@"
