#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/internal/env.sh"
python3 "$INTRAMURAL_ROOT/scripts/prepare-sources.py"
if command -v docker >/dev/null 2>&1 && [[ "${INTRAMURAL_NATIVE:-0}" != 1 ]]; then
  docker image inspect "$INTRAMURAL_IMAGE" --format '{{.Id}}'
fi
exec "$INTRAMURAL_ROOT/scripts/run.sh" bash -c '
  set -e
  cat /etc/os-release
  printf "ROS_DISTRO=%s\n" "$ROS_DISTRO"
  if command -v ign >/dev/null 2>&1; then
    ign gazebo --versions
  else
    echo "Ignition Gazebo not installed (native host mode)"
  fi
  graphics=$(glxinfo -B 2>/dev/null || echo "Headless / no glxinfo")
  printf "%s\n" "$graphics"
  if [[ "${LIBGL_ALWAYS_SOFTWARE:-0}" == 0 ]] && grep -q "Accelerated: no" <<< "$graphics"; then
    echo "GPU requested but hardware acceleration is unavailable" >&2
    exit 1
  fi
  colcon list --names-only
  for package in pb2025_nav_bringup point_lio small_gicp_relocalization \
    pb_omni_pid_pursuit_controller rmu_gazebo_simulator rmoss_gz_plugins; do
    ros2 pkg prefix "$package"
  done
'
