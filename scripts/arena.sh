#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/internal/env.sh"
transport=direct
previous=
for item in "$@"; do
  if [[ "$previous" == --transport ]]; then transport="$item"; fi
  previous="$item"
done
export INTRAMURAL_TRANSPORT="$transport"
"$INTRAMURAL_ROOT/scripts/run.sh" python3 /work/scripts/internal/generate-arena.py
exec "$INTRAMURAL_ROOT/scripts/run.sh" bash /work/scripts/internal/graphics-entrypoint.sh python3 /work/scripts/internal/arena-process.py "$@"
