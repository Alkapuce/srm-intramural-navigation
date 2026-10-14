#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
[[ $# -eq 2 && "$1" == --output ]] || { echo 'Usage: scripts/reproduce-candidate.sh --output /work/artifacts/<new-directory>' >&2; exit 2; }
output="$2"
[[ "$output" == /work/artifacts/* ]] || { echo 'Output must be under /work/artifacts' >&2; exit 2; }
name="${output##*/}"
[[ "$name" =~ ^[a-zA-Z0-9_-]+$ ]] || { echo 'Output directory basename must be a simple unique name' >&2; exit 2; }
host_output="$root/${output#/work/}"
[[ ! -e "$host_output" && ! -e "$root/cache/candidate-$name-build" ]] || { echo 'Output or build directory exists; choose a fresh name' >&2; exit 2; }
mkdir -p "$host_output"
"$root/scripts/check-environment.sh" > "$host_output/environment.log" 2>&1
python3 "$root/scripts/prepare-sources.py" > "$host_output/sources.log" 2>&1
"$root/scripts/internal/build-candidate-clean.sh" "$name" > "$host_output/build.log" 2>&1
export INTRAMURAL_INSTALL_PREFIX="/work/cache/candidate-$name-install"
"$root/scripts/run.sh" python3 -m unittest discover -s /work/tests -v > "$host_output/tests.log" 2>&1
"$root/scripts/verify-stage34.sh" --mode sensors --transport serial-sim --rounds 1 --output "$output" > "$host_output/figure8-run.log" 2>&1
echo "PASS clean candidate build, tests and serial-simulation lap: $host_output"
