#!/usr/bin/env bash
set -euo pipefail
exec "$(dirname -- "${BASH_SOURCE[0]}")/run.sh" python3 /work/scripts/validation/verify-gui.py "$@"
