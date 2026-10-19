#!/usr/bin/env bash
set -euo pipefail
echo "检查图形显示（DISPLAY=${DISPLAY:-未设置}）…" >&2
if ! graphics=$(timeout --foreground --kill-after=2s 20s glxinfo -B); then
  echo '图形预检失败或超过 20 秒；尚未启动仿真。请检查 DISPLAY、WSLg 和 GPU 驱动。' >&2
  exit 1
fi
if [[ "${LIBGL_ALWAYS_SOFTWARE:-0}" == 0 ]] && ! grep -q 'Accelerated: yes' <<< "$graphics"; then
  printf '%s\n' 'GPU 模式未取得硬件加速，请检查显示授权及 WSL / Linux 渲染设备；需要软件模式时设置 INTRAMURAL_SOFTWARE_RENDERING=1。' >&2
  exit 1
fi
sed -n '/OpenGL renderer string:/p' <<< "$graphics"
exec "$@"
