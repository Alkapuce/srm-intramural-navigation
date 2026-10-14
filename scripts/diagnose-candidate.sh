#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/internal/env.sh"
exec "$INTRAMURAL_ROOT/scripts/run.sh" python3 /work/scripts/internal/diagnose-candidate.py "$@"
