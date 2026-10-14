#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/internal/env.sh"
python3 "$INTRAMURAL_ROOT/scripts/prepare-sources.py"
exec docker build -f "$INTRAMURAL_ROOT/docker/Dockerfile" \
  -t "$INTRAMURAL_IMAGE" "$INTRAMURAL_ROOT" "$@"
