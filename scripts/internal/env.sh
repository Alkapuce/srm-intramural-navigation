#!/usr/bin/env bash
set -euo pipefail
INTRAMURAL_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
INTRAMURAL_IMAGE="${INTRAMURAL_IMAGE:-intramural-humble:stage2}"
export INTRAMURAL_ROOT INTRAMURAL_IMAGE
