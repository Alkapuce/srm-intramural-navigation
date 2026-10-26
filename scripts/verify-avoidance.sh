#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export INTRAMURAL_ROS_DOMAIN_ID="${INTRAMURAL_ROS_DOMAIN_ID:-91}"
export INTRAMURAL_IGN_PARTITION="${INTRAMURAL_IGN_PARTITION:-srm_intramural_avoidance}"
export INTRAMURAL_TRANSPORT=serial-sim
exec "$root/scripts/run.sh" python3 /work/scripts/validation/verify-avoidance.py "$@"
