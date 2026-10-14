#!/usr/bin/env bash
set -euo pipefail
graphics=$(glxinfo -B)
if [[ "${LIBGL_ALWAYS_SOFTWARE:-0}" == 0 ]] && ! grep -q 'Accelerated: yes' <<< "$graphics"; then
  printf '%s\n' 'GPU 模式未取得硬件加速，请检查显示授权及 WSL / Linux 渲染设备；需要软件模式时设置 INTRAMURAL_SOFTWARE_RENDERING=1。' >&2
  exit 1
fi
sed -n '/OpenGL renderer string:/p' <<< "$graphics"
exec "$@"
