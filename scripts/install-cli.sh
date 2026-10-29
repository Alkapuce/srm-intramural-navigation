#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
bin="$HOME/.local/bin"
completions="${XDG_DATA_HOME:-$HOME/.local/share}/bash-completion/completions"
commands=(srm-map srm-pulse srm-nav)
# The optional viewer is a development-machine entry, absent on the NUC.
[[ ! -f "$root/scripts/srm-rviz" ]] || commands+=(srm-rviz)
mkdir -p "$bin" "$completions"
for name in "${commands[@]}"; do
  target="$bin/$name"
  if [[ -e "$target" || -L "$target" ]]; then
    [[ -L "$target" && "$(readlink -f -- "$target")" == "$root/scripts/$name" ]] || {
      echo "已有其他命令，拒绝覆盖: $target" >&2; exit 1;
    }
  fi
done
for name in "${commands[@]}"; do
  target="$completions/$name"
  if [[ -e "$target" || -L "$target" ]]; then
    [[ -L "$target" && "$(readlink -f -- "$target")" == "$root/scripts/internal/srm-completion.bash" ]] || {
      echo "已有其他补全，拒绝覆盖: $target" >&2; exit 1;
    }
  fi
done
for name in "${commands[@]}"; do
  [[ -L "$bin/$name" ]] || ln -s "$root/scripts/$name" "$bin/$name"
  [[ -L "$completions/$name" ]] || ln -s "$root/scripts/internal/srm-completion.bash" "$completions/$name"
  echo "$name -> $root/scripts/$name"
done
echo "Bash Tab 补全已安装；新终端自动加载，当前终端执行: source \"$root/scripts/internal/srm-completion.bash\""
case ":$PATH:" in
  *":$bin:"*) ;;
  *) echo '当前 shell 尚未包含 ~/.local/bin；执行 export PATH="$HOME/.local/bin:$PATH"，或重新登录。' ;;
esac
