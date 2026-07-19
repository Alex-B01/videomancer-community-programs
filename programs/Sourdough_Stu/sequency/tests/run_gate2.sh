#!/usr/bin/env bash
# ============================================================================
# run_gate2.sh -- SEQUENCY Gate-2 functional suite (all hooks, reproducible).
# Analyzes the SDK+program once (image-tester source ordering) then runs every
# hook testbench, printing only the machine-readable RESULT/MEASURED lines.
#
#   corners  (per corner 0..6): #1 identity . #2 div16 . #5 U/V . #7 lat52+sync . #10 clamp
#   identity_adv               : #1 STRONGEST (exhaustive 65536-vertex, bit-exact @52)
#   fader                      : #6 dry + wet
#   state                      : #3 REGEN wired/evolving/bounded . #4 Spectral Hold frozen/effective
#   null                       : #8 rate-class NULL (1080p30 vs 1080p24)
#   vblank                     : #9 no-vblank-compute NULL (VBL 3 vs 50 lines)
#   inert                      : #11 reserved CROSSTALK+Chroma+Dithered+Rotate inert
#
# Usage: ./run_gate2.sh [quick]      (quick => adversarial sweep reduced to 4096)
# Env: source videomancer-sdk/build/oss-cad-suite/environment first (or rely on PATH).
# ============================================================================
set -uo pipefail
# Build artifacts live in $WORK OUTSIDE the repo (set below); the trap removes it
# on exit unless the caller pinned GATE2_WORK. KEEP_WORK defaults to 1 so an early
# exit (before $WORK is set) never rm's anything. See build-artifact hygiene in
# docs/runbooks/agentic-pipeline.md.
trap 'rc=$?; [ "${KEEP_WORK:-1}" = 0 ] && [ -n "${WORK:-}" ] && rm -rf "$WORK"; echo "RUN_GATE2_DONE (exit $rc)"' EXIT

ROOT=/Users/hellomello/Developer/Videomancer
SDK=$ROOT/videomancer-community-programs/videomancer-sdk
PROG=$ROOT/videomancer-community-programs/programs/Sourdough_Stu/sequency
TESTS=$PROG/tests
WORK=${GATE2_WORK:-/tmp/sequency_gate2_work}
[ -n "${GATE2_WORK:-}" ] && KEEP_WORK=1 || KEEP_WORK=0
VENVPY=$SDK/tools/vhdl-image-tester/.venv/bin/python
ADV_SWEEP=65535
[ "${1:-}" = "quick" ] && ADV_SWEEP=4095

# shellcheck disable=SC1091
source "$SDK/build/oss-cad-suite/environment" 2>/dev/null || true
mkdir -p "$WORK"; rm -f "$WORK"/*.cf 2>/dev/null || true
FILT='grep -Ev "metavalue|:warning:"'

echo "== source order + analyze design =="
"$VENVPY" -c "
from vhdl_image_tester.core.sim_runner import _ordered_sdk_sources, _ordered_program_sources
from pathlib import Path
for f in _ordered_sdk_sources('hd_hdmi','yuv444_30b'): print(f)
for f in _ordered_program_sources(Path('$PROG')): print(f)
" > "$WORK/sources.txt"
while IFS= read -r f; do [ -z "$f" ] && continue
  ghdl -a --std=08 -frelaxed --workdir="$WORK" "$f" || { echo "ANALYZE FAIL: $f"; exit 1; }
done < "$WORK/sources.txt"

echo "== analyze harness =="
for t in video_corner_pkg tb_sequency_corners tb_sequency_identity_adv tb_sequency_fader \
         tb_sequency_state tb_sequency_null tb_sequency_vblank tb_sequency_inert; do
  ghdl -a --std=08 -frelaxed --workdir="$WORK" "$TESTS/$t.vhd" || { echo "ANALYZE FAIL: $t"; exit 1; }
done

# First arg = unit; the rest are generics. -o "$WORK/<unit>" is REQUIRED, not
# optional: --workdir holds only the work library, but GHDL's llvm backend writes
# the elaborated binary + e~<unit>.o to CWD unless -o redirects them into $WORK.
run() { local u=$1; shift; ghdl --elab-run --std=08 -frelaxed --workdir="$WORK" -o "$WORK/$u" "$u" "$@" 2>&1 | grep -E "RESULT|MEASURED_LATENCY|VIT_DONE|IDENTITY FAIL|FAILED" | grep -Ev "metavalue"; }

echo "== #1/#2/#5/#7/#10 five corners (+SD extras) =="
for c in 0 1 2 3 4 5 6; do run tb_sequency_corners -gG_CORNER=$c; done
echo "== #1 STRONGEST exhaustive-vertex adversarial (sweep=$ADV_SWEEP) =="
run tb_sequency_identity_adv -gSWEEP_MAX=$ADV_SWEEP
echo "== #6 fader =="        ; run tb_sequency_fader
echo "== #3/#4 REGEN+Hold =="; run tb_sequency_state
echo "== #8 rate-class =="   ; run tb_sequency_null
echo "== #9 vblank null =="  ; run tb_sequency_vblank
echo "== #11 inert =="       ; run tb_sequency_inert
