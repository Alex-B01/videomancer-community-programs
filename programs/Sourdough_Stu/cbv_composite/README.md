# CBV Composite

**Status: Stage 1 (codec round trip) — not a usable instrument yet.**

CBV Composite puts the Syntonie CBV001 composite video bender on a virtual
composite signal path inside Videomancer. The incoming Y/U/V picture is
encoded to a composite signal (NTSC or PAL carrier, colour burst, virtual
sync), passed through a behavioural model of the CBV001 circuit (Enhance,
ringing and smear loops, boost), and decoded by a virtual TV (clamp, Y/C
split, demodulator, burst-locked colour engine, bent sync with a "Tame"
control) whose picture is read out on the true raster one line late. In
Stage 1 the CBV001 cell is an exact identity stub and the sync side is
absent, so the program is an encoder/decoder round trip used to measure
cost and verify the codec bit-exactly against the reference model.

## Controls (full map; Stage 1 reads only Encode, Decode and the timing ID)

| Control | Name | Notes |
|---|---|---|
| Pot 1 | Enhance | Stage 2 |
| Pot 2 | Ring | Stage 2 |
| Pot 3 | Ring Freq | Stage 2 |
| Pot 4 | Smear / Boost | Stage 2 |
| Pot 5 | Tame | Stage 3 |
| Pot 6 | Leak | Stage 3 |
| Toggle 7 | Sync (Bent / Clean) | Stage 3 |
| Toggle 8 | Encode (NTSC / PAL) | Stage 1 |
| Toggle 9 | Decode (NTSC / PAL-S) | Stage 1 |
| Toggle 10 | Hue Bend (Stepped / Smooth) | Stage 4 |
| Toggle 11 | Clamp (Porch / Picture) | Stage 3 |
| Slider | Colour | Stage 4 |

All 15 video timings are supported.

## Credits

The circuit facts this program models come from the **Syntonie CBV001
Rev C schematic** (CC BY-SA 4.0) and **Phil Baljeu's Archer trace** of the
circuit. The program models the circuit's behaviour; it copies no
schematic artwork.

## Licence

GPL-3.0-only (see the SPDX headers in each source file).
