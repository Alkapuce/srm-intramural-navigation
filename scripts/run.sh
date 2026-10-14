#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/internal/env.sh"
domain="${INTRAMURAL_ROS_DOMAIN_ID:-71}"
[[ "$domain" =~ ^[0-9]{1,3}$ ]] && (( 10#$domain <= 232 )) || { echo 'ROS domain must be an integer in 0..232' >&2; exit 2; }
domain=$((10#$domain))
mkdir -p "$INTRAMURAL_ROOT/cache/home" "$INTRAMURAL_ROOT/cache/runtime" "$INTRAMURAL_ROOT/artifacts"
chmod 700 "$INTRAMURAL_ROOT/cache/runtime"

if [[ "${INTRAMURAL_NATIVE:-0}" == 1 ]] || ! command -v docker >/dev/null 2>&1; then
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    export ROS_DOMAIN_ID="$domain"
    export ROS_LOCALHOST_ONLY=1
    export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$INTRAMURAL_ROOT/config/fastdds-services.xml}"
    export IGN_IP=127.0.0.1
    export IGN_PARTITION="${INTRAMURAL_IGN_PARTITION:-srm_intramural}"
    export LIBGL_ALWAYS_SOFTWARE="${INTRAMURAL_SOFTWARE_RENDERING:-${LIBGL_ALWAYS_SOFTWARE:-0}}"
    # ROS environment hooks read optional unset variables under their own shell contract.
    set +u
    source /opt/ros/humble/setup.bash
    prefix="${INTRAMURAL_INSTALL_PREFIX:-$INTRAMURAL_ROOT/nav_ws/install}"
    if [[ "${INTRAMURAL_SOURCE_OVERLAY:-1}" == 1 && -f "$prefix/setup.bash" ]]; then
      source "$prefix/setup.bash"
    fi
    set -u
    cd "$INTRAMURAL_ROOT/nav_ws"
    cmd=()
    for arg in "$@"; do
      if [[ "$arg" == /work/* ]]; then
        cmd+=("$INTRAMURAL_ROOT/${arg#/work/}")
      elif [[ "$arg" == *=/work/* ]]; then
        prefix_arg="${arg%%=/work/*}="
        suffix_arg="${arg#*=}"
        cmd+=("${prefix_arg}${INTRAMURAL_ROOT}/${suffix_arg#/work/}")
      else
        cmd+=("$arg")
      fi
    done
    [[ ${#cmd[@]} -gt 0 ]] || cmd=(bash --noprofile --norc)
    exec "${cmd[@]}"
  fi
fi
args=(--rm --init -i --network host --ipc host --ulimit core=0 --user "$(id -u):$(id -g)"
  -e "INTRAMURAL_SOURCE_OVERLAY=${INTRAMURAL_SOURCE_OVERLAY:-1}"
  -e HOME=/work/cache/home -e ROS_LOG_DIR=/work/nav_ws/log/runtime
  -e "MESA_D3D12_DEFAULT_ADAPTER_NAME=${INTRAMURAL_GPU_ADAPTER:-NVIDIA}"
  -e "LIBGL_ALWAYS_SOFTWARE=${INTRAMURAL_SOFTWARE_RENDERING:-0}"
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/work/config/fastdds-services.xml
  -e "INTRAMURAL_TRANSPORT=${INTRAMURAL_TRANSPORT:-direct}"
  -e "INTRAMURAL_INSTALL_PREFIX=${INTRAMURAL_INSTALL_PREFIX:-/work/nav_ws/install}"
  -e "ROS_DOMAIN_ID=$domain" -e ROS_LOCALHOST_ONLY=1 -e QT_X11_NO_MITSHM=1
  -e IGN_IP=127.0.0.1 -e "IGN_PARTITION=${INTRAMURAL_IGN_PARTITION:-srm_intramural}"
  -e MAKEFLAGS=-j2 -e CMAKE_BUILD_PARALLEL_LEVEL=2
  --label "io.srm.intramural.root=$INTRAMURAL_ROOT"
  -v "$INTRAMURAL_ROOT:/work" -w /work/nav_ws)
if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
  args+=(-e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:ro)
  display_authority="${XAUTHORITY:-$HOME/.Xauthority}"
  if [[ -f "$display_authority" ]]; then
    args+=(-v "$display_authority:/tmp/intramural.Xauthority:ro" -e XAUTHORITY=/tmp/intramural.Xauthority)
  fi
fi
if [[ -d /mnt/wslg ]]; then
  if [[ ! -e "$INTRAMURAL_ROOT/cache/runtime/wayland-0" && ! -L "$INTRAMURAL_ROOT/cache/runtime/wayland-0" ]]; then
    ln -s /mnt/wslg/runtime-dir/wayland-0 "$INTRAMURAL_ROOT/cache/runtime/wayland-0"
  fi
  args+=(-v /mnt/wslg:/mnt/wslg:ro -e XDG_RUNTIME_DIR=/work/cache/runtime
    -e WAYLAND_DISPLAY=wayland-0 -e PULSE_SERVER=unix:/mnt/wslg/PulseServer)
fi
if [[ -e /dev/dxg && -d /usr/lib/wsl/lib ]]; then
  args+=(--device /dev/dxg -v /usr/lib/wsl:/usr/lib/wsl:ro
    -e LD_LIBRARY_PATH=/usr/lib/wsl/lib:/opt/ros/humble/lib)
fi
if [[ -d /dev/dri ]]; then
  for render_device in /dev/dri/renderD*; do
    [[ -c "$render_device" ]] || continue
    args+=(--device "$render_device" --group-add "$(stat -c '%g' "$render_device")")
  done
fi
if [[ -n "${INTRAMURAL_SERIAL_DEVICE:-}" ]]; then
  serial_device="$(readlink -f -- "$INTRAMURAL_SERIAL_DEVICE")"
  [[ "$serial_device" == /dev/* && -c "$serial_device" ]] || { echo 'Explicit serial device must resolve to a character device under /dev' >&2; exit 2; }
  args+=(--device "$serial_device:/dev/intramural-base" --group-add "$(stat -c '%g' "$serial_device")")
fi
if [[ -t 0 && -t 1 ]]; then args+=(-t); fi
if [[ -n "${INTRAMURAL_CONTAINER_NAME:-}" ]]; then
  args+=(--name "$INTRAMURAL_CONTAINER_NAME")
fi
[[ $# -gt 0 ]] || set -- bash --noprofile --norc
exec docker run "${args[@]}" "$INTRAMURAL_IMAGE" "$@"
