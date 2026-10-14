#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:-}"
[[ $# -gt 0 ]] && shift || true
case "$mode" in
  simulation) exec "$root/scripts/arena.sh" --localization ideal "$@" ;;
  navigation) exec "$root/scripts/arena.sh" --localization sensors "$@" ;;
  serial-sim) exec "$root/scripts/arena.sh" --localization sensors --transport serial-sim "$@" ;;
  mapping) exec "$root/scripts/map-arena.sh" "$@" ;;
  diagnose) exec "$root/scripts/diagnose-candidate.sh" "$@" ;;
  *) echo 'Usage: scripts/candidate.sh {simulation|navigation|serial-sim|mapping|diagnose} [mode options]' >&2; exit 2 ;;
esac
