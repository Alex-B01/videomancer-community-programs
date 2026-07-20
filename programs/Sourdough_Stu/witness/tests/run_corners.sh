#!/usr/bin/env bash
# ============================================================================
# run_corners.sh -- WITNESS Lite Gate-2 five-corner functional suite.
# Analyzes the SDK + witness program (image-tester source ordering, which
# resolves the _kit symlinks) once, ghdl -a's the SHARED five-corner harness
# (_kit/tests/video_corner_pkg.vhd) + the WITNESS testbenches EXPLICITLY (they
# are not globbed), then runs each testbench, printing only the machine-readable
# RESULT / MEASURED_LATENCY / VIT_DONE lines.
#
#   corners (per corner 0..4 [+5 6 SD]): H1a dry-identity . H1b wet-identity .
#     H2 tiling . H8 latency16+sync . H9 clamp . H12 inert-NULL
#   gaze  (1080p24/30/5994): centroid/store-evolution/LAG-rate-class/EVADE/hue
#
# Usage:  ./run_corners.sh [corner_indices...]      (default 0 1 2 3 4)
#         ./run_corners.sh gaze                      (gaze/state proxies only)
# Env: routes ghdl --workdir OUTSIDE the repo; jsonschema<4.18 already pinned.
# ============================================================================
set -uo pipefail
trap 'rc=$?; [ "${KEEP_WORK:-1}" = 0 ] && [ -n "${WORK:-}" ] && rm -rf "$WORK"; echo "RUN_CORNERS_DONE (exit $rc)"' EXIT

ROOT=/Users/hellomello/Developer/Videomancer
SDK=$ROOT/videomancer-community-programs/videomancer-sdk
PROG=$ROOT/videomancer-community-programs/programs/Sourdough_Stu/witness
KIT=$ROOT/videomancer-community-programs/programs/Sourdough_Stu/_kit
TESTS=$PROG/tests
WORK=${CORNERS_WORK:-/private/tmp/witness_corners_work}
[ -n "${CORNERS_WORK:-}" ] && KEEP_WORK=1 || KEEP_WORK=0
VENVPY=$SDK/tools/vhdl-image-tester/.venv/bin/python

# shellcheck disable=SC1091
source "$SDK/build/oss-cad-suite/environment" 2>/dev/null || true
export PATH="$(dirname "$(command -v ghdl)"):$PATH"

mkdir -p "$WORK"; rm -f "$WORK"/*.cf 2>/dev/null || true

echo "== source order + analyze SDK + witness (symlinks resolved) =="
"$VENVPY" -c "
from vhdl_image_tester.core.sim_runner import _ordered_sdk_sources, _ordered_program_sources
from pathlib import Path
for f in _ordered_sdk_sources('hd_hdmi','yuv444_30b'): print(f)
for f in _ordered_program_sources(Path('$PROG')): print(f)
" > "$WORK/sources.txt"
while IFS= read -r f; do [ -z "$f" ] && continue
  ghdl -a --std=08 -frelaxed --workdir="$WORK" "$f" || { echo "ANALYZE FAIL: $f"; exit 1; }
done < "$WORK/sources.txt"

echo "== analyze SHARED harness + WITNESS testbenches (explicit) =="
ghdl -a --std=08 -frelaxed --workdir="$WORK" "$KIT/tests/video_corner_pkg.vhd" || exit 1
ghdl -a --std=08 -frelaxed --workdir="$WORK" "$TESTS/tb_witness_corners.vhd"   || exit 1
ghdl -a --std=08 -frelaxed --workdir="$WORK" "$TESTS/tb_witness_gaze.vhd"      || exit 1

run() { local u=$1; shift; ghdl --elab-run --std=08 -frelaxed --workdir="$WORK" -o "$WORK/$u" "$u" "$@" 2>&1 \
        | grep -E "RESULT|MEASURED_LATENCY|VIT_DONE|H1a|H1b|H12|H9|H2|GAZE|====|error|bound check|TIMEOUT" | grep -Ev "metavalue|:warning:" || true; }

if [ "${1:-}" = "gaze" ]; then
  echo "== gaze/state proxies (1080p30 ref, then 24/5994 rate class) =="
  run tb_witness_gaze
  exit 0
fi

CORNERS=("$@"); [ ${#CORNERS[@]} -eq 0 ] && CORNERS=(0 1 2 3 4)
for c in "${CORNERS[@]}"; do
  echo "== elaborate+run corner $c =="
  run tb_witness_corners -gG_CORNER="$c"
done

echo "== gaze/state proxies =="
run tb_witness_gaze
