#!/usr/bin/env python3
"""Generate ringing wave LUT for Bent Topology BRAM initialisation.

Produces the same damped-sinusoid values embedded in the VHDL constant
C_RING_LUT. Run this to regenerate ring_lut.hex for external BRAM init
workflows (e.g. SB_RAM40_4K INIT_x attributes).

Usage:
    python3 gen_ring_lut.py [output_file]   (default: ring_lut.hex)
"""
import math
import sys


def generate_ring_lut(freq=6.0, decay=4.0, entries=1024):
    lut = []
    for i in range(entries):
        t = i / (entries - 1)
        offset = math.sin(2 * math.pi * freq * t) * math.exp(-decay * t)
        val = max(0, min(255, round(128 + 127 * offset)))
        lut.append(val)
    return lut


if __name__ == "__main__":
    lut = generate_ring_lut()
    outfile = sys.argv[1] if len(sys.argv) > 1 else "ring_lut.hex"
    with open(outfile, "w") as f:
        for val in lut:
            f.write(f"{val:02X}\n")
    print(f"Wrote {len(lut)} entries to {outfile}")
