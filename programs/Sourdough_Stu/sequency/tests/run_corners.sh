#!/usr/bin/env bash
# Analyze SDK + SEQUENCY + harness once, then run tb_sequency_corners for each
# corner index passed as arguments (default: all seven 0..6, incl. 480p/PAL SD).
# Usage:  ./run_corners.sh [corner_indices...]
set -euo pipefail

# Reliable completion marker: a trap on EXIT fires even on early/error exit or a
# non-zero grep under pipefail, so an auto-report waiter never orphans.
# $WORK (set below) is an ephemeral build dir OUTSIDE the repo; the trap removes it
# on exit unless the caller pinned CORNERS_WORK. KEEP_WORK defaults to 1 so an early
# exit never rm's anything. See build-artifact hygiene in agentic-pipeline.md.
trap 'rc=$?; [ "${KEEP_WORK:-1}" = 0 ] && [ -n "${WORK:-}" ] && rm -rf "$WORK"; echo "RUN_CORNERS_DONE (exit $rc)"' EXIT

ROOT=/Users/hellomello/Developer/Videomancer
SDK=$ROOT/videomancer-community-programs/videomancer-sdk
PROG=$ROOT/videomancer-community-programs/programs/Sourdough_Stu/sequency
TESTS=$PROG/tests
WORK=${CORNERS_WORK:-/tmp/sequency_corners_work}
[ -n "${CORNERS_WORK:-}" ] && KEEP_WORK=1 || KEEP_WORK=0
VENVPY=$SDK/tools/vhdl-image-tester/.venv/bin/python

# shellcheck disable=SC1091
source "$SDK/build/oss-cad-suite/environment"
export PATH="$(dirname "$(command -v ghdl)"):$PATH"
export LZX_VIT_PROGRAMS_DIR=$ROOT/videomancer-community-programs/programs/Sourdough_Stu

mkdir -p "$WORK"
rm -f "$WORK"/*.cf 2>/dev/null || true

echo "== resolving SDK + program source order =="
LIST="$WORK/sources.txt"
"$VENVPY" -c "
from vhdl_image_tester.core.sim_runner import _ordered_sdk_sources, _ordered_program_sources
from pathlib import Path
for f in _ordered_sdk_sources('hd_hdmi','yuv444_30b'): print(f)
for f in _ordered_program_sources(Path('$PROG')): print(f)
" > "$LIST"

echo "== analyzing $(wc -l < "$LIST") SDK/program files =="
while IFS= read -r f; do
  [ -z "$f" ] && continue
  ghdl -a --std=08 -frelaxed --workdir="$WORK" "$f" || { echo "ANALYZE FAIL: $f"; exit 1; }
done < "$LIST"

echo "== analyzing harness =="
ghdl -a --std=08 -frelaxed --workdir="$WORK" "$TESTS/video_corner_pkg.vhd"
ghdl -a --std=08 -frelaxed --workdir="$WORK" "$TESTS/tb_sequency_corners.vhd"

CORNERS=("$@"); [ ${#CORNERS[@]} -eq 0 ] && CORNERS=(0 1 2 3 4 5 6)
for c in "${CORNERS[@]}"; do
  echo "== elaborate+run corner $c =="
  # -o "$WORK/…" redirects the elaborated binary + e~*.o into $WORK; --workdir
  # alone leaves them in CWD (llvm backend), which is how they leaked into tests/.
  ghdl --elab-run --std=08 -frelaxed --workdir="$WORK" -o "$WORK/tb_sequency_corners" \
       tb_sequency_corners -gG_CORNER="$c" 2>&1 | grep -E "RESULT|MEASURED_LATENCY|HOOK|====|VIT_DONE|error|bound check|TIMEOUT" || true
done
