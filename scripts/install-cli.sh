#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
bin="$HOME/.local/bin"
mkdir -p "$bin"
for name in srm-map srm-pulse srm-nav srm-rviz; do
  target="$bin/$name"
  if [[ -e "$target" || -L "$target" ]]; then
    [[ -L "$target" && "$(readlink -f -- "$target")" == "$root/scripts/$name" ]] || {
      echo "已有其他命令，拒绝覆盖: $target" >&2; exit 1;
    }
  fi
done
for name in srm-map srm-pulse srm-nav srm-rviz; do
  [[ -L "$bin/$name" ]] || ln -s "$root/scripts/$name" "$bin/$name"
  echo "$name -> $root/scripts/$name"
done
case ":$PATH:" in
  *":$bin:"*) ;;
  *) echo '当前 shell 尚未包含 ~/.local/bin；执行 export PATH="$HOME/.local/bin:$PATH"，或重新登录。' ;;
esac
