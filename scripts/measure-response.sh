#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$root/scripts/run.sh" python3 /work/scripts/validation/measure-response.py "$@"
