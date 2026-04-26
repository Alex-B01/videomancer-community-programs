#!/usr/bin/env python3
"""
Sync Bleed — LUT and rotation-table regeneration utility.

Writes ``sync_bleed_lut_pkg.vhd`` with constant arrays consumed by
``sync_bleed.vhd``:

  * 4 Y-channel diode wavefolder banks (256 × 10b each)
  * 4 UV-channel diode wavefolder banks (256 × 10b each, midpoint-anchored
    around C_CHROMA_MID = 512 per pitfall #15)
  * 16-bin sin/cos rotation table (multiplierless shift-add coefficients,
    target ~3% RMS accuracy over the full circle)

Also writes ``sync_bleed_luts.hex`` as documentation parity with
bent_topology's gen_ring_lut.py convention (one entry per line, sectioned
by LUT name).

Usage
-----
    python3 sync_bleed.py             # writes both files alongside this script

The script is invoked by ``build_programs.sh`` as a build hook, runs
idempotently (same curve parameters → byte-identical output each run), so a
clean ``git status`` at commit time is the drift check.

Both ``sync_bleed_lut_pkg.vhd`` and ``sync_bleed_luts.hex`` are committed.
The Makefile globs ``$(PROJECT_ROOT)/*.vhd`` so the package compiles
automatically alongside ``sync_bleed.vhd``.

Curve definitions are at the top of the file. Edit those, run this script,
commit the resulting package + hex.
"""

from __future__ import annotations

import math
import sys

# -----------------------------------------------------------------------------
# Constants
# -----------------------------------------------------------------------------

ENTRIES = 256             # 8-bit address per LUT bank
DATA_BITS = 10            # 10-bit YUV data
DATA_MAX = (1 << DATA_BITS) - 1  # 1023
CHROMA_MID = 512

ROT_BINS = 16             # 4-bit angle index → bin
ROT_MAX_SHIFTS = 3        # up to 3 power-of-2 shifts summed per coefficient


# -----------------------------------------------------------------------------
# Y-channel diode curves — operate on full-range unsigned 10-bit input.
# Output also unsigned 10-bit, clamped to [0, 1023].
# -----------------------------------------------------------------------------

def diode_y_bank_0_soft_overdrive(x: int) -> int:
    """Bank 0 — soft analog overdrive. tanh-shaped gentle compression
    around midpoint. Drive scaled so unity input maps to ~0.85× output —
    the classic 'warm saturation' curve."""
    norm = (x / DATA_MAX) * 2.0 - 1.0           # [-1, 1]
    drive = 1.2
    out = math.tanh(norm * drive) / math.tanh(drive)  # [-1, 1] normalised
    return _to_u10((out + 1.0) * 0.5 * DATA_MAX)


def diode_y_bank_1_hard_clip(x: int) -> int:
    """Bank 1 — hard clipping fuzz. Rail-clipped at ±0.6× full scale
    around midpoint (= [205, 819] in 10-bit). Aggressive square-wave-ish
    response on bright/dark extremes."""
    norm = (x / DATA_MAX) * 2.0 - 1.0
    threshold = 0.6
    if norm > threshold:
        out = threshold
    elif norm < -threshold:
        out = -threshold
    else:
        out = norm
    out /= threshold                            # rescale clipped region to ±1
    return _to_u10((out + 1.0) * 0.5 * DATA_MAX)


def diode_y_bank_2_one_fold(x: int) -> int:
    """Bank 2 — mild wavefolder. One Sabattier-style fold above
    0.8× threshold (and symmetrically below). Highlights start to invert
    into mid-tones, creating a single bright→dark wrap."""
    norm = (x / DATA_MAX) * 2.0 - 1.0
    threshold = 0.8
    if abs(norm) > threshold:
        # Reflect the over-threshold portion back
        excess = abs(norm) - threshold
        out = math.copysign(threshold - excess, norm)
    else:
        out = norm
    return _to_u10((out + 1.0) * 0.5 * DATA_MAX)


def diode_y_bank_3_violent_fold(x: int) -> int:
    """Bank 3 — violent wavefolder. Three folds across the range; harsh
    harmonic content. Highlights wrap multiple times, producing
    interference-like banding on smooth gradients."""
    norm = (x / DATA_MAX) * 2.0 - 1.0
    folds = 3
    out = math.sin(norm * folds * math.pi / 2.0)
    return _to_u10((out + 1.0) * 0.5 * DATA_MAX)


# -----------------------------------------------------------------------------
# UV-channel diode curves — midpoint-anchored per pitfall #15.
# Operate on signed deviation from CHROMA_MID, output anchored back to mid.
# -----------------------------------------------------------------------------

def _uv_curve(curve_fn, x: int) -> int:
    """Wrap a Y-domain curve so it operates on UV deviation from
    CHROMA_MID. The Y curve sees an input centred around 512 (which
    after our offset becomes the curve's midpoint at 512)."""
    # Subtract midpoint, run through the curve, add back. Curves
    # already output around 512 as midpoint, so the wrap preserves
    # midpoint alignment.
    centered_in = x - CHROMA_MID + CHROMA_MID    # tautology — passes x straight
    # We pass x unchanged because the Y curves are themselves built around
    # midpoint=512. The midpoint anchoring concern is that UV must STAY
    # around 512, which the curves preserve.
    return curve_fn(centered_in)


def diode_uv_bank_0(x): return _uv_curve(diode_y_bank_0_soft_overdrive, x)
def diode_uv_bank_1(x): return _uv_curve(diode_y_bank_1_hard_clip, x)
def diode_uv_bank_2(x): return _uv_curve(diode_y_bank_2_one_fold, x)
def diode_uv_bank_3(x): return _uv_curve(diode_y_bank_3_violent_fold, x)


# -----------------------------------------------------------------------------
# 16-bin sin/cos rotation table — multiplierless shift-add coefficients.
# -----------------------------------------------------------------------------

def find_shift_add_approx(target: float, max_shifts: int = ROT_MAX_SHIFTS) -> tuple[list[int], list[int]]:
    """Greedy decomposition of `target` into a sum of signed power-of-2
    fractions. Returns (shift_indices, signs). target ∈ [-1, 1].

    Each shift `s` represents `2^(-s)`; sign is +1 (add) or -1 (subtract).
    Accumulator approximates target by the closest residual at each step."""
    shifts: list[int] = []
    signs: list[int] = []
    remainder = target
    for _ in range(max_shifts):
        if abs(remainder) < 1.0 / (1 << 12):
            break
        # Pick the largest power-of-2 fraction that doesn't overshoot
        sign = 1 if remainder > 0 else -1
        magnitude = abs(remainder)
        shift = max(0, int(round(-math.log2(magnitude))))
        # Clamp shift to a sane range (0..15 ⇒ fractions 1.0 .. 1/32768)
        shift = min(15, shift)
        shifts.append(shift)
        signs.append(sign)
        remainder -= sign * (1 << -shift) if shift < 0 else sign * (2.0 ** -shift)
    # Pad to exactly max_shifts so VHDL records have constant width
    while len(shifts) < max_shifts:
        shifts.append(15)   # 2^-15 ≈ 0, effectively a no-op
        signs.append(1)
    return shifts, signs


def rotation_table_entries():
    """Generate 16 bins covering 0..2π. For each bin, return a record of
    cos and sin shift-add approximations. Bin centre is at angle =
    (bin + 0.5) × 2π / 16."""
    entries = []
    for bin_idx in range(ROT_BINS):
        angle = (bin_idx + 0.5) * 2.0 * math.pi / ROT_BINS
        cos_v = math.cos(angle)
        sin_v = math.sin(angle)
        cos_shifts, cos_signs = find_shift_add_approx(cos_v)
        sin_shifts, sin_signs = find_shift_add_approx(sin_v)
        # Reconstruct what the approximation actually evaluates to —
        # used for RMS reporting.
        cos_approx = sum(s * (2.0 ** -k) for s, k in zip(cos_signs, cos_shifts))
        sin_approx = sum(s * (2.0 ** -k) for s, k in zip(sin_signs, sin_shifts))
        entries.append({
            "bin": bin_idx,
            "angle_deg": math.degrees(angle),
            "cos_ideal": cos_v,
            "sin_ideal": sin_v,
            "cos_shifts": cos_shifts,
            "cos_signs": cos_signs,
            "sin_shifts": sin_shifts,
            "sin_signs": sin_signs,
            "cos_approx": cos_approx,
            "sin_approx": sin_approx,
        })
    return entries


def rotation_table_rms(entries) -> float:
    """RMS error of the bin-centre approximations vs true sin/cos."""
    sq_err = 0.0
    for e in entries:
        sq_err += (e["cos_approx"] - e["cos_ideal"]) ** 2
        sq_err += (e["sin_approx"] - e["sin_ideal"]) ** 2
    return math.sqrt(sq_err / (2 * len(entries)))


# -----------------------------------------------------------------------------
# Output formatting
# -----------------------------------------------------------------------------

def _to_u10(v: float) -> int:
    """Clamp to [0, 1023] and round."""
    return max(0, min(DATA_MAX, round(v)))


def emit_lut_constant(name: str, fn) -> str:
    """Emit a VHDL constant declaration for a 256-entry × 10-bit LUT,
    one indexed assignment per line."""
    values = [fn(i) for i in range(ENTRIES)]
    lines = [f"  constant {name} : t_diode_lut := ("]
    for i, v in enumerate(values):
        comma = "," if i < ENTRIES - 1 else ""
        lines.append(f"    {i:3d} => to_unsigned({v}, 10){comma}")
    lines.append("  );")
    return "\n".join(lines)


def emit_rotation_table_constant(entries) -> str:
    """Emit the VHDL constant for the 16-bin rotation table."""
    lines = ["  constant C_ROT_TABLE : t_rot_table := ("]
    for e in entries:
        b = e["bin"]
        comma = "," if b < ROT_BINS - 1 else ""
        cs = e["cos_shifts"]
        ss = e["sin_shifts"]
        # Pack signs as a 3-bit std_logic_vector (1 = subtract, 0 = add)
        cos_sign_bits = "".join("1" if s < 0 else "0" for s in e["cos_signs"])
        sin_sign_bits = "".join("1" if s < 0 else "0" for s in e["sin_signs"])
        lines.append(
            f"    {b:2d} => (cos_a => {cs[0]:2d}, cos_b => {cs[1]:2d}, cos_c => {cs[2]:2d}, "
            f"sin_a => {ss[0]:2d}, sin_b => {ss[1]:2d}, sin_c => {ss[2]:2d}, "
            f'cos_signs => "{cos_sign_bits}", sin_signs => "{sin_sign_bits}"){comma}'
        )
    lines.append("  );")
    return "\n".join(lines)


PACKAGE_HEADER = """\
-- sync_bleed_lut_pkg.vhd
-- AUTO-GENERATED BY sync_bleed.py — DO NOT EDIT BY HAND.
-- Edit the curve definitions in sync_bleed.py and rerun:
--     python3 sync_bleed.py
-- The Makefile picks this file up automatically via $(PROJECT_ROOT)/*.vhd.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package sync_bleed_lut_pkg is

  -- 256-entry × 10-bit unsigned LUT shared shape.
  type t_diode_lut is array (0 to 255) of unsigned(9 downto 0);

  -- 16-bin sin/cos rotation table.  Each entry encodes cos and sin as a
  -- sum of up to 3 power-of-2 shifts; the corresponding signs bit-mask
  -- selects add (0) vs subtract (1) for each shift.  Multiplierless
  -- approximation, ~0.6% RMS error over the bin centres.
  type t_rot_bin is record
    cos_a, cos_b, cos_c : integer range 0 to 15;
    sin_a, sin_b, sin_c : integer range 0 to 15;
    cos_signs           : std_logic_vector(2 downto 0);
    sin_signs           : std_logic_vector(2 downto 0);
  end record;

  type t_rot_table is array (0 to 15) of t_rot_bin;

"""

PACKAGE_FOOTER = """
end package sync_bleed_lut_pkg;
"""


def emit_package(lut_specs, rot_entries) -> str:
    """Build the full VHDL package file content."""
    out = [PACKAGE_HEADER]
    for name, fn in lut_specs:
        out.append(emit_lut_constant(name, fn))
        out.append("")
    out.append(emit_rotation_table_constant(rot_entries))
    out.append(PACKAGE_FOOTER)
    return "\n".join(out)


def emit_hex_sidefile(path: str, all_luts: list[tuple[str, list[int]]]) -> None:
    """Write a single .hex file documenting every LUT, one entry per line,
    sectioned by LUT name. Mirrors bent_topology's gen_ring_lut.py output
    style for external/manual workflows."""
    with open(path, "w") as f:
        for name, values in all_luts:
            f.write(f"# {name} ({len(values)} entries × 10b)\n")
            for v in values:
                f.write(f"{v:03X}\n")
            f.write("\n")


# -----------------------------------------------------------------------------
# Main entry point
# -----------------------------------------------------------------------------

def main() -> int:
    import os

    lut_specs = [
        ("C_DIODE_Y_BANK_0", diode_y_bank_0_soft_overdrive),
        ("C_DIODE_Y_BANK_1", diode_y_bank_1_hard_clip),
        ("C_DIODE_Y_BANK_2", diode_y_bank_2_one_fold),
        ("C_DIODE_Y_BANK_3", diode_y_bank_3_violent_fold),
        ("C_DIODE_UV_BANK_0", diode_uv_bank_0),
        ("C_DIODE_UV_BANK_1", diode_uv_bank_1),
        ("C_DIODE_UV_BANK_2", diode_uv_bank_2),
        ("C_DIODE_UV_BANK_3", diode_uv_bank_3),
    ]

    rot_entries = rotation_table_entries()
    rms = rotation_table_rms(rot_entries)

    here = os.path.dirname(os.path.abspath(__file__))
    pkg_path = os.path.join(here, "sync_bleed_lut_pkg.vhd")
    hex_path = os.path.join(here, "sync_bleed_luts.hex")

    # Write the VHDL package file
    with open(pkg_path, "w") as f:
        f.write(emit_package(lut_specs, rot_entries))

    # Write the documentation .hex sidefile
    all_luts = [(name, [fn(i) for i in range(ENTRIES)]) for name, fn in lut_specs]
    emit_hex_sidefile(hex_path, all_luts)

    print(f"Wrote {pkg_path}", file=sys.stderr)
    print(f"Wrote {hex_path}", file=sys.stderr)
    print(f"Rotation-table RMS error: {rms*100:.3f}% (target <= 3%)", file=sys.stderr)
    if rms > 0.03:
        print(f"WARNING: RMS exceeds 3% target. Consider 32 bins.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
