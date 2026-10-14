#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
if [[ "${INTRAMURAL_SOURCE_OVERLAY:-1}" == 1 && -f "${INTRAMURAL_INSTALL_PREFIX:-/work/nav_ws/install}/setup.bash" ]]; then
  source "${INTRAMURAL_INSTALL_PREFIX:-/work/nav_ws/install}/setup.bash"
fi
exec "$@"
