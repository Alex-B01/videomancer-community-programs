#!/usr/bin/env bash
# ============================================================================
# HOWLBOX Gate 2 five-corner runner.  Analyses SDK+program+harness ONCE into an
# out-of-repo workdir, then:
#   * tb_howlbox_corners : hooks 4,5,6,7,8 per corner (TRUE h-blanking geometry)
#   * tb_howlbox_rate    : hook 1 (rate class) + 2 (evolution); collects the
#     two-level Delta-fields per corner and aggregates the wall-clock +/-25%
#     (one-rate-class) band -- WITH a bypassed control that MUST fail (teeth).
#   * tb_howlbox_recover : hook 3 self-recovery (1080p30)
#   * tb_howlbox_normshift : normalizer component proof (all 3 coeffs x corners)
# Corners: 0 NTSC .1 720p60 .2 1080p30(ref) .3 1080i5994 .4 1080p24
# Usage:  ./run_corners.sh            (all)
#         ./run_corners.sh rate       (rate hook only)
# ============================================================================
set -uo pipefail
ROOT=/Users/hellomello/Developer/Videomancer
WT=$ROOT/.claude/worktrees/distracted-wu-3ac3c2
SDK=$ROOT/videomancer-community-programs/videomancer-sdk
HB=$WT/videomancer-community-programs/programs/Sourdough_Stu/howlbox
TESTS=$HB/tests
WORK=${HOWLBOX_WORK:-/private/tmp/claude-501/-Users-hellomello-Developer-Videomancer--claude-worktrees-distracted-wu-3ac3c2/d89c3a97-41ab-49e2-9e34-42ba47412d89/scratchpad/hbwork}
VENVPY=$SDK/tools/vhdl-image-tester/.venv/bin/python
MODE=${1:-all}

source "$SDK/build/oss-cad-suite/environment"
export PATH="$(dirname "$(command -v ghdl)"):$PATH"
mkdir -p "$WORK"; rm -f "$WORK"/*.cf 2>/dev/null || true
A="ghdl -a --std=08 -frelaxed --workdir=$WORK"
R="ghdl --elab-run --std=08 -frelaxed --workdir=$WORK"
FILT='RESULT|MEASURED_LATENCY|RATE_STARVE|NORMSHIFT|====|VIT_DONE|error|bound check|TIMEOUT'

echo "== analysing SDK+program+harness =="
"$VENVPY" -c "
from vhdl_image_tester.core.sim_runner import _ordered_sdk_sources, _ordered_program_sources
from pathlib import Path
for f in _ordered_sdk_sources('hd_hdmi','yuv444_30b'): print(f)
for f in _ordered_program_sources(Path('$HB')): print(f)
" > "$WORK/sources.txt"
while IFS= read -r f; do [ -z "$f" ] && continue
  $A "$f" >/dev/null 2>&1 || { echo "ANALYZE FAIL: $f"; $A "$f"; exit 1; }
done < "$WORK/sources.txt"
$A "$TESTS/video_corner_pkg.vhd"
for t in corners rate recover normshift; do
  [ -f "$TESTS/tb_howlbox_$t.vhd" ] && $A "$TESTS/tb_howlbox_$t.vhd"
done
echo "ANALYZE OK"

if [ "$MODE" = all ] || [ "$MODE" = corners ]; then
  for c in 0 1 2 3 4; do
    $R -o "$WORK/tb_howlbox_corners" tb_howlbox_corners -gG_CORNER=$c --ieee-asserts=disable 2>&1 \
      | grep -E "$FILT" | sed 's#.*): ##'
  done
fi

if [ "$MODE" = all ] || [ "$MODE" = recover ]; then
  [ -f "$TESTS/tb_howlbox_recover.vhd" ] && \
    $R -o "$WORK/tb_howlbox_recover" tb_howlbox_recover --ieee-asserts=disable 2>&1 \
      | grep -E "$FILT" | sed 's#.*): ##'
fi

if [ "$MODE" = all ] || [ "$MODE" = normshift ]; then
  [ -f "$TESTS/tb_howlbox_normshift.vhd" ] && \
    $R -o "$WORK/tb_howlbox_normshift" tb_howlbox_normshift --ieee-asserts=disable 2>&1 \
      | grep -E "$FILT" | sed 's#.*): ##'
fi

if [ "$MODE" = all ] || [ "$MODE" = rate ]; then
  echo "== rate class: measuring Delta-fields per corner =="
  RF="$WORK/rate.txt"; : > "$RF"
  for c in 0 1 2 3 4; do
    $R -o "$WORK/tb_howlbox_rate" tb_howlbox_rate -gG_CORNER=$c --ieee-asserts=disable 2>&1 \
      | grep -E "RATE_STARVE|RESULT HOOK2|TIMEOUT" | sed 's#.*): ##' | tee -a "$RF"
  done
  echo "== HOOK1 rate-class aggregation (wall-clock = Delta x real per-field period) =="
  "$VENVPY" - "$RF" <<'PY'
import sys, re
period = {0:1000/59.94, 14:1000/60, 7:1000/30, 2:1000/59.94, 3:1000/24}  # ms/field
name   = {0:"NTSC",14:"720p60",7:"1080p30",2:"1080i5994",3:"1080p24"}
rows={}
for ln in open(sys.argv[1]):
    m=re.search(r'RATE_STARVE (\S+) id=(\d+) f1=(-?\d+) f2=(-?\d+) delta=(-?\d+)',ln)
    if m: rows[int(m.group(2))]=(m.group(1),int(m.group(5)))
if 7 not in rows: print("RESULT HOOK1_rateclass: FAIL (no reference 1080p30 datapoint)"); sys.exit()
ref = rows[7][1]*period[7]
print(f"  {'corner':10} {'id':>3} {'delta':>6} {'wall_ms':>9} {'ratio':>7}")
worst=0.0; band_fail=[]
for cid,(nm,d) in sorted(rows.items(), key=lambda x:list(name).index(x[0]) if x[0] in name else 99):
    wall=d*period[cid]; r=wall/ref
    print(f"  {nm:10} {cid:>3} {d:>6} {wall:>9.1f} {r:>7.3f}")
    if abs(r-1.0)>0.25: band_fail.append(f"{nm}={r:.3f}")
    worst=max(worst,abs(r-1.0))
# bypassed control: same (reference) delta at every corner -> wall spreads with period
byp=[rows[7][1]*period[c] for c in rows]; byp_spread=max(byp)/min(byp)
print(f"  [normalized]  worst |ratio-1| = {worst*100:.1f}%  (band = +/-25%)")
print(f"  [bypassed ctl] wall spread   = {byp_spread:.2f}x  (must be >1.5x => test has teeth)")
ok = (not band_fail) and byp_spread>1.5
print("RESULT HOOK1_rateclass: " + ("PASS" if ok else "FAIL") +
      ("" if not band_fail else " OUT-OF-BAND: "+", ".join(band_fail)) +
      f" (teeth: bypassed spreads {byp_spread:.2f}x)")
PY
fi
echo "RUN_CORNERS_DONE"
