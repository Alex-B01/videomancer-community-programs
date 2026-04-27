-- Sync Bleed
-- Copyright (C) 2026 Alex Ball (Sourdough Stu)
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Analog dirty mixer / broken TBC.  Five-stage pipeline driven by a
-- dual envelope follower (luma mean + edge energy, top/bottom split):
--
--   S0  Input register, hcount/vcount, lfsr16 advance, edge magnitude,
--       dual envelope accumulators, frame-phase drift accumulator
--   S0b Y input shift-register chain (fixed comb tap @ stage 6)
--   S1  TBC jitter compute, BRAM read addresses (4 total: Y tbc + Y prev,
--       U tbc, V tbc)
--   S2  Per-channel BRAM clocked reads (Y bank 2-port, U/V banks 1-port)
--   S3  BRAM output registration (idiom #7) + to_01 sanitiser
--   S4  Diode wavefolder LUT lookup (Y + UV, 4 banks each, env-CV-selected)
--   S5  NTSC comb (H + V, opposite-signed UV inject) + sin/cos hue twist
--   S6  Sin/cos rotation shift-add (multiplierless, 16-bin table) + UV swap
--   S7  Capacitor bleed IIR (per-channel inline, idiom #3, avid-gated,
--       hsync-reset, single-pass — no multi-frame DC bias issue)
--   S8  Edge ringing add to Y (companion-piped from S0)
--   S9  Tape noise + sync-crush band mux
--   S10-S13  Master Mix interpolator_u (4 clocks)
--
-- Pipeline depth: C_PROCESSING_DELAY_CLKS = 18
--
-- BRAM: Y bank 2-port (TBC + prev-line) = 10 EBR; U bank, V bank 1-port = 5
-- EBR each. Diode LUTs Y+UV 4 banks each = 8 EBR. Total 28 / 32 EBR (88%).
--
-- Layer 0 de-risk (2026-04-26) verified: Yosys replicates BRAM storage per
-- read port (no shared-storage primitive on iCE40). The original blueprint's
-- "free 3-tap = 15 EBR" claim was 5× off. Path Y' uses asymmetric port
-- counts and an input shift-register chain for the fixed NTSC comb tap.
-- See docs/guides/hardware-constraints.md "BRAM read-port cost".
--
-- =====  M9 STATUS — STAGE 5 SYNC-CRUSH + TAPE NOISE + MASTER MIX  =====
-- Prior milestones: M2-M7 shipped on feature/sync-bleed; M8 staged in
-- this same commit chain.  This commit lands Stage 5 — the final stage
-- of the pipeline.  Pipeline depth target 18 reached.
--
-- Components landed:
--   * Tape noise: lfsr16-derived signed-around-zero noise, gain
--     modulated by cv_global (envelope drives noise amplitude).  Added
--     to all three channels (Y, U, V).  Pitfall #15 satisfied because
--     we ADD signed noise (centred around 0) to UV (centred around 512)
--     — UV stays around midpoint, no pull toward green.
--   * Sync-crush bands: per-line trigger at hsync_falling.  Fires when
--     Toggle 11 enabled AND cv_global > threshold AND density-mask
--     conditions met AND LFSR bit set.  When active for a line:
--     Y → 0 (true black), UV → C_CHROMA_MID (NOT 0 — pitfall #15).
--   * Master mix interpolator_u × 3 channels (Y, U, V).  Dry input from
--     sync delay shift register at depth 14; wet input from sync-crush
--     mux at S12 (also depth 14).  4-cycle interpolator latency.
--   * Knob 6 (Envelope Reactivity) further scales the cv_global feed
--     into noise amp + crush rate (envelope coupling controlled by user).
--
-- Pipeline depth bumped 12 → 18 (S11 tape noise + S12 sync-crush mux +
-- 4-cycle interpolator).  Sync delay shift register sized to match.
--
-- All 12 controls now functional:
--   Knobs 1-6 + Slider drive their respective stages.  Toggles 7-11
--   modulate AC/DC, edge phase, UV swap, diode base, sync-crush enable.
--
-- =====  M8 NOTES (preceding) — STAGE 4 CAP BLEED IIR + EDGE RINGING  =====
-- Stage 4: per-channel inline IIR with fractional bits
-- (idiom #3) + edge ringing add to Y.  Toggles 7 (AC/DC) and 8 (Edge
-- Phase Invert) come alive.  Knobs 1 (Cap Bleed) and 2 (Edge Resonance)
-- come alive.
--
-- Pipeline depth bumped 10 → 12 (S11 IIR state register + S12 smear
-- register with edge ringing add).
--
-- IIR uses signed 19-bit state (1 sign + 10 data + 8 fractional bits)
-- per idiom #3.  Single-pass per line — pitfall #3's signed shift_right
-- DC bias does NOT apply because state resets each hsync_falling and
-- there's no multi-frame feedback loop.  avid-gated update (pitfall #1).
-- UV state resets to C_CHROMA_MID << 8 = 131072 to avoid green collapse
-- during blanking (pitfalls #2, #15).
--
-- Edge magnitude (s_edge_d0, computed at S0) companion-piped through to
-- S11 timing for the ringing add.  Knob 2 upper-3-bits + sigma-delta
-- dither sets the ringing scale; Toggle 8 flips the sign (bright vs
-- dark outline).
--
-- Subsequent commit adds Stage 5 sync-crush bands + tape noise + master
-- mix interpolator (M9).
--
-- =====  M7 NOTES (preceding) — STAGE 3 NTSC COMB + SIN/COS HUE TWIST  =====
-- Prior milestones: M2 (d2ccea1), M3 (075eda3), M4+M5 (3e7bb0e), M6.
-- Lands Stage 3:
--   * NTSC horizontal comb: delta_h = s_y_diode - s_y_chain(6)_aligned
--     (high-frequency Y energy injected into UV with opposite signs to
--      produce rainbow moiré, NOT magenta/green axis — pitfall #14).
--   * NTSC vertical comb: delta_v = s_y_diode - s_y_prev_d (Y from prev
--     active line, same column).  Same opposite-signed UV inject.
--   * Knob 4 (NTSC Comb) shift table + sigma-delta dither scales the
--     combined delta before injection.
--   * Sin/cos hue twist: bit-sliced shift-add rotation on UV-centred
--     coordinates.  16-bin C_ROT_TABLE from sync_bleed_lut_pkg gives
--     ~0.6% RMS approximation of true rotation, multiplierless.  Angle
--     = (s_y_diode + (cv_global << 2) + Knob 3) mod 1024.  Sigma-delta
--     dither between adjacent bins from angle LSBs.
--   * UV swap (Toggle 9) post-rotation.
--
-- Pipeline depth bumped 5 → 9 (S5 comb compute + S6 UV inject + S7
-- rotation lookup + S8 shift-add + UV swap + register).
--
-- Subsequent commits add Stage 4 IIR cap bleed + edge ringing (M8),
-- Stage 5 sync-crush + tape noise + master mix interpolator (M9).
--
-- =====  M6 NOTES (preceding) — STAGE 2 DIODE WAVEFOLDER LUTS  =====
-- Lands Stage 2: envelope-CV-driven diode wavefolder LUTs
-- with sigma-delta dither between adjacent banks, Toggle 10 (Diode Base)
-- biasing the active bank range (Soft = 0..2, Hard = 1..3).
--
-- Eight 256×10 ROMs initialised from sync_bleed_lut_pkg constants (Python-
-- generated curves: tanh soft, hard ±0.6 clip, one-fold, three-fold).
-- Y-side reads each bank once at s_y_tbc(9..2); UV-side reads each bank
-- twice (U + V) per dual-port ROM pattern — Yosys is expected to map this
-- as 1 EBR per bank (dual-port read on the same storage), but if we get
-- replication it'd be 2 EBR per UV bank → 12 EBR total.  Verified post-
-- synth.
--
-- Pipeline depth bumped 3 → 5 (S3 LUT read + S4 LUT mux register).  Sync
-- delay sized to match.  Wet path now: s_wet_y/u/v <= s_y_diode/etc.
--
-- Subsequent commits add Stage 3 NTSC comb + sin/cos hue twist (M7),
-- Stage 4 IIR cap bleed + edge ringing (M8), Stage 5 sync-crush + tape
-- noise + master mix interpolator (M9).
--
-- =====  M5 NOTES (preceding commit 3e7bb0e) — STAGE 1 BRAM + TBC =====
--   * Free-running s_wr_addr (avid-gated, mod 2048) so prev-line tap at
--     offset 1920 actually returns the same X column from the prior line.
--   * Y bank inferred BRAM with 2 read ports (TBC + prev-line, 10 EBR);
--     U and V banks 1 read port each (TBC only, 5 EBR each).
--   * 7-stage Y input shift-register chain — declared for M7 fixed comb
--     tap.  Synthesised in M5 with no consumer (Yosys may prune; comes
--     alive at M7 NTSC comb compute).
--   * TBC jitter compute (post-M10 redesign): per-line LFSR-based offset
--     latched at hsync_start, magnitude scaled by Knob 5 + envelope ×
--     Knob 6 boost.  See p_tbc_jitter.
--
-- Pipeline depth bumped DOWN: was 18 (M2 placeholder for the eventual
-- master-mix interpolator output), now 3 (S0 input register + S1 BRAM
-- read + S2 BRAM output reg).  Each subsequent stage milestone grows
-- the depth back up; M9 lands at 18 with the interpolator.  Sync delay
-- shift register sized to match.
--
-- Wet path now wires through Stage 1 BRAM — output is the TBC tap.
-- At Knob 5 = 0 with all CVs at 0: output = wr_addr - 16 read,
-- a small fixed horizontal offset (no jitter).  As Knob 5 increases:
-- TBC tearing modulated by content + envelope + motion + drift.
--
-- Subsequent commits add Stage 2 diode LUTs (M6), Stage 3 NTSC comb +
-- sin/cos hue twist (M7), Stage 4 IIR cap bleed + edge ringing (M8),
-- Stage 5 sync-crush + tape noise + master mix interpolator (M9).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_stream_pkg.all;     -- t_video_stream_yuv444_30b
use work.core_pkg.all;             -- t_spi_ram
use work.video_timing_pkg.all;     -- t_video_timing_port
use work.sync_bleed_lut_pkg.all;   -- t_diode_lut, t_rot_table, C_DIODE_*, C_ROT_TABLE

architecture sync_bleed of program_top is

    -- ========================================================================
    -- Pipeline & buffer constants
    -- ========================================================================
    -- Pipeline depth grows incrementally per milestone.  Final target
    -- reached at M9: 18 clocks from data_in to data_out.
    --   1 clk  : S0   input registration, hcount/vcount, lfsr advance,
    --                 edge mag, wr_addr increment, Y chain shift
    --   1 clk  : S1   TBC jitter compute (registered)
    --   1 clk  : S2   BRAM read
    --   1 clk  : S3   BRAM output registration + to_01 sanitise
    --   1 clk  : S4   Diode LUT bank reads (8 ROMs in parallel)
    --   1 clk  : S5   Diode LUT bank-select mux + register
    --   1 clk  : S6   NTSC comb compute (delta_h + delta_v + scale)
    --   1 clk  : S7   UV inject (opposite-signed) + register
    --   1 clk  : S8a  Sin/cos rotation: shift-add products
    --   1 clk  : S8b  Sin/cos rotation: sum + clamp + UV swap mux
    --   1 clk  : S9   IIR state register update (avid-gated, hsync-reset)
    --   1 clk  : S10  Smear register (IIR output + edge ringing add)
    --   1 clk  : S11  Tape noise add register (Y/U/V, env-modulated gain)
    --   1 clk  : S12  Sync-crush band mux register (Y→0, UV→C_CHROMA_MID
    --                 when band-active; otherwise pass-through)
    --   4 clk  : S13-S16  interpolator_u (master mix, 3 instances)
    --   1 clk  : S17  output register (concurrent assignment to data_out
    --                 with broadcast-safe clamp R1 — actually combinational,
    --                 just listed for completeness)
    -- ─────────
    --  18 total at M9 (final)
    constant C_PROCESSING_DELAY_CLKS : integer := 18;
    -- Dry-path tap depth into the master mix interpolator (a-input).
    -- Interpolator is 4 cycles, so dry tap must be at depth 14 = 18 − 4.
    constant C_DRY_TAP_DEPTH         : integer := 14;
    -- IIR-gate depth — avid and hsync_start gating signals for p_iir_state
    -- need to align with the IIR's data input (s_y_rot/u_rot/v_rot) at
    -- depth 10.  Pipeline: S0..S8b = 10 register stages from data_in to
    -- s_y_rot.  Gating on s_avid_d0 (depth 1) caused 9 cycles of blanking-
    -- zero data to be processed at line start, dragging UV state to 0
    -- (= green collapse).  See pitfall #19 in video-fpga-pitfalls.md.
    constant C_IIR_GATE_DEPTH        : integer := 10;

    -- BRAM line buffer geometry — 2048 deep covers 1080p active line (1920) + headroom.
    constant C_BUF_SIZE      : integer := 2048;
    constant C_BUF_DEPTH     : integer := 11;
    -- Baseline TBC read offset.  Set to 1 (minimum safe value to avoid
    -- BRAM read-write race at offset 0) so that with Knob 5 = 0 the wet
    -- path is effectively aligned with the dry path at the master-mix
    -- interpolator.  Originally 16 to "simulate analog TBC delay" but
    -- this produced a visible static right-shift on stills with the
    -- right-edge wrapping back to the left edge.  Knob 5 (TBC Jitter)
    -- still adds 0..16 px of dynamic per-line offset on top.
    constant C_TBC_BASE      : integer := 1;

    -- Active video frame dimensions.  Spatial-half boundary and the
    -- envelope-accumulator divisor are now derived at runtime from the
    -- live video timing (see s_active_half_height_dyn and
    -- s_avg_shift_dyn below) so the program is resolution-portable
    -- across all supported video modes — pitfall #5 mitigation.

    -- Envelope accumulator width.  Worst-case half-frame at 1080p is
    -- 1920 × 540 × 1023 ≈ 1.06e9 < 2^30, so 32 bits gives comfortable
    -- headroom for any supported mode.
    constant C_ENV_ACC_WIDTH      : integer := 32;

    -- Centred-unsigned chroma neutral (idiom #1 — pitfalls #2, #15).
    constant C_CHROMA_MID    : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Broadcast-safe output ranges (R1).
    constant C_Y_MIN         : unsigned(9 downto 0) := to_unsigned(64,  10);
    constant C_Y_MAX         : unsigned(9 downto 0) := to_unsigned(940, 10);
    constant C_UV_MIN        : unsigned(9 downto 0) := to_unsigned(64,  10);
    constant C_UV_MAX        : unsigned(9 downto 0) := to_unsigned(960, 10);

    -- ========================================================================
    -- Timing infrastructure
    -- ========================================================================
    signal s_timing      : t_video_timing_port;
    signal s_hcount      : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');

    -- ========================================================================
    -- Register decode (knobs + slider)
    -- ========================================================================
    -- Knobs 1-6 (registers_in 0..5) and slider (register 7).
    -- All knobs are 10-bit unsigned 0..1023.
    -- M2 skeleton: declared but unused — wired in at later milestones.
    signal s_cap_bleed       : unsigned(9 downto 0);   -- Knob 1 (Reg 0)
    signal s_edge_resonance  : unsigned(9 downto 0);   -- Knob 2 (Reg 1)
    signal s_chroma_twist    : unsigned(9 downto 0);   -- Knob 3 (Reg 2)
    signal s_ntsc_comb       : unsigned(9 downto 0);   -- Knob 4 (Reg 3)
    signal s_tbc_jitter      : unsigned(9 downto 0);   -- Knob 5 (Reg 4)
    signal s_envelope_react  : unsigned(9 downto 0);   -- Knob 6 (Reg 5)
    signal s_master_mix      : unsigned(9 downto 0);   -- Slider (Reg 7)

    -- ========================================================================
    -- Toggle decode (registers_in(6) bit-packed, bits[4:0])
    -- ========================================================================
    signal s_ac_dc           : std_logic;   -- Toggle 7  bit 0
    signal s_edge_phase_inv  : std_logic;   -- Toggle 8  bit 1
    signal s_uv_swap         : std_logic;   -- Toggle 9  bit 2
    signal s_diode_base      : std_logic;   -- Toggle 10 bit 3
    signal s_sync_crush      : std_logic;   -- Toggle 11 bit 4

    -- ========================================================================
    -- LFSR — shared chaos source (tape noise + sync-crush band trigger).
    -- Lives in S0 advance-every-clock domain; consumers tap different bits.
    -- Declared at M2 but unused; wired in at M9.
    -- ========================================================================
    signal s_lfsr_q          : std_logic_vector(15 downto 0);

    -- ========================================================================
    -- Edge magnitude (S0) — |Y(n) - Y(n-1)|, used by both envelope edge
    -- accumulator (M3) and edge ringing add (M8).
    -- ========================================================================
    signal s_y_d_prev_pix    : unsigned(9 downto 0) := (others => '0');
    signal s_edge_d0         : unsigned(9 downto 0) := (others => '0');

    -- ========================================================================
    -- Analysis branch (M3) — dual envelope follower, top/bottom spatial split.
    -- ========================================================================

    -- vcount: line counter within the active frame.  Reset on vsync_falling,
    -- increment on hsync_falling.  Sized for 1080 + headroom (11 bits = 2047).
    signal s_vcount          : unsigned(10 downto 0) := (others => '0');
    signal s_in_top_half     : std_logic;

    -- Per-half luma + edge accumulators.  Reset on vsync_falling, summed
    -- during avid in the appropriate half.
    signal s_luma_acc_top    : unsigned(C_ENV_ACC_WIDTH - 1 downto 0) := (others => '0');
    signal s_luma_acc_bot    : unsigned(C_ENV_ACC_WIDTH - 1 downto 0) := (others => '0');
    signal s_edge_acc_top    : unsigned(C_ENV_ACC_WIDTH - 1 downto 0) := (others => '0');
    signal s_edge_acc_bot    : unsigned(C_ENV_ACC_WIDTH - 1 downto 0) := (others => '0');

    -- Vsync-latched + peak-held + leaked CVs.  10 bits each (matched to
    -- knob/slider scale, easy to consume downstream).
    signal s_cv_luma_top     : unsigned(9 downto 0) := (others => '0');
    signal s_cv_luma_bot     : unsigned(9 downto 0) := (others => '0');
    signal s_cv_edge_top     : unsigned(9 downto 0) := (others => '0');
    signal s_cv_edge_bot     : unsigned(9 downto 0) := (others => '0');

    -- Combined global CV: max of top vs bottom (luma+edge sum), halved to
    -- fit 10 bits.  Used by stages that need a single CV (diode bank
    -- select, sync-crush rate, tape noise gain).  Init to 0 so sync-crush
    -- band trigger (cv > C_CRUSH_TH) doesn't fire on undefined startup.
    signal s_cv_global       : unsigned(9 downto 0) := (others => '0');

    -- Frame-phase drift — 16-bit DDS phase, advances at frame rate.  Speed
    -- = (cv_global + envelope_react) >> 4 so phase advance is envelope-
    -- modulated (busy frames drift faster).
    signal s_phase_drift     : unsigned(15 downto 0);
    signal s_phase_speed     : unsigned(9 downto 0);

    -- Pre-registered spatial-half CV selection.  Breaks the critical path
    -- s_in_top_half → cv_luma_top/bot mux → 14-bit signed adds inside
    -- p_tbc_jitter (was capping Fmax at ~65 MHz at M9).  Aesthetically
    -- invisible: cv_luma_top/bot only update at vsync, so a 1-cycle
    -- register on the spatial-half mux doesn't lag anything visible.
    signal s_cv_luma_half_d  : unsigned(9 downto 0) := (others => '0');

    -- Resolution-aware divisor for the envelope accumulator.  Counts
    -- active pixels in the top spatial half each frame; the priority-
    -- encoded MSB position + 1 gives ceil(log2(half-frame-pixels)) =
    -- the correct shift amount for any video mode + decimation.
    -- Defaults to 20 (1080p hardware) so the first frame works before
    -- the count latches.  Pitfall #5 mitigation.
    signal s_pixel_count_top : unsigned(23 downto 0) := (others => '0');
    signal s_avg_shift_dyn   : integer range 4 to 24 := 20;

    -- Resolution-aware spatial-half boundary.  Latched from s_vcount at
    -- each vsync_start = total active row count of the just-completed
    -- frame; halved for the s_in_top_half comparison.  Default 540 so
    -- the first frame after reset uses 1080p calibration; settles to
    -- the correct value after the first complete frame.  Pitfall #5
    -- mitigation alongside s_avg_shift_dyn.
    signal s_active_half_height_dyn : unsigned(10 downto 0)
                                      := to_unsigned(540, 11);

    -- Resolution-aware prev-line tap offset for the BRAM ring buffer.
    -- Latched from a per-line active-pixel counter at hsync_start =
    -- active pixel count of the just-completed line.  The NTSC comb's
    -- vertical delta tap (s_y_prev) reads at wr_addr - this value, so
    -- it lands on the same column from the previous active line at
    -- any video mode.  Default 1920 = 1080p line length.  Pitfall #5.
    signal s_pixels_per_line     : unsigned(C_BUF_DEPTH - 1 downto 0)
                                   := (others => '0');
    signal s_prev_offset_dyn     : unsigned(C_BUF_DEPTH - 1 downto 0)
                                   := to_unsigned(1920, C_BUF_DEPTH);

    -- ========================================================================
    -- Stage 1 (M4-M5): registered inputs, BRAM, shift-register chain.
    -- ========================================================================

    -- Registered inputs at S0 — separates data_in from combinational logic.
    signal s_y_d0            : std_logic_vector(9 downto 0) := (others => '0');
    signal s_u_d0            : std_logic_vector(9 downto 0) := (others => '0');
    signal s_v_d0            : std_logic_vector(9 downto 0) := (others => '0');
    signal s_avid_d0         : std_logic := '0';

    -- Free-running write address — avid-gated, mod 2048.  Different from
    -- s_hcount: this counter advances ONLY during active video, so the
    -- prev-line tap at offset 1920 returns the same X column from the
    -- previous active line regardless of blanking duration.
    signal s_wr_addr         : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');

    -- 7-stage Y input shift-register chain.  Stage 0 = registered input,
    -- stage 6 = input delayed by 6 active samples → consumed by M7 NTSC
    -- comb compute as the fixed comb tap.  No EBR cost.
    type t_y_chain is array (0 to 6) of std_logic_vector(9 downto 0);
    signal s_y_chain         : t_y_chain := (others => (others => '0'));

    -- Inferred BRAM banks — Y bank 2-port (TBC + prev-line, 10 EBR);
    -- U and V banks 1-port (TBC only, 5 EBR each).  No init values
    -- (gravity_bleed Option D pattern — see hardware-constraints.md
    -- "BRAM inference anti-patterns").
    type t_bram is array (0 to C_BUF_SIZE - 1) of std_logic_vector(9 downto 0);
    signal bram_y            : t_bram;
    signal bram_u            : t_bram;
    signal bram_v            : t_bram;

    -- Read addresses (combinational at S0) → BRAM read processes at S1.
    signal s_rd_addr_tbc_y   : unsigned(C_BUF_DEPTH - 1 downto 0);
    signal s_rd_addr_prev_y  : unsigned(C_BUF_DEPTH - 1 downto 0);
    signal s_rd_addr_tbc_u   : unsigned(C_BUF_DEPTH - 1 downto 0);
    signal s_rd_addr_tbc_v   : unsigned(C_BUF_DEPTH - 1 downto 0);

    -- Raw BRAM read outputs (clocked at S1).  Sanitised + registered at S2.
    signal s_y_tbc_raw       : std_logic_vector(9 downto 0);
    signal s_y_prev_raw      : std_logic_vector(9 downto 0);
    signal s_u_tbc_raw       : std_logic_vector(9 downto 0);
    signal s_v_tbc_raw       : std_logic_vector(9 downto 0);

    -- Sanitised Stage 1 outputs at S2 (idiom #7 + to_01 GHDL guard).
    signal s_y_tbc           : unsigned(9 downto 0);
    signal s_y_prev          : unsigned(9 downto 0);
    signal s_u_tbc           : unsigned(9 downto 0);
    signal s_v_tbc           : unsigned(9 downto 0);

    -- Computed TBC offset — REGISTERED at end of S0 to break the
    -- combinational chain s_in_top_half → cv mux → adds/shifts → rd_addr
    -- → BRAM read DFF.  At M7 this chain was the critical path, capping
    -- Fmax at ~46 MHz.  Registering the offset adds 1 cycle to pipeline
    -- and brings Fmax back over the 74.25 MHz target.  Aesthetically
    -- invisible: the offset is now 1 active-pixel "stale" relative to
    -- the wr_addr but TBC reads are bounded by the offset anyway.
    -- Distinct from s_tbc_jitter (the raw Knob 5 register value).
    signal s_tbc_offset      : unsigned(8 downto 0) := (others => '0');

    -- Sigma-delta dither for Knob 5 lower 7 bits (idiom #15).  Smooths
    -- perceived control across adjacent shift-amount levels.
    signal s_jitter_dither_acc : unsigned(6 downto 0) := (others => '0');
    signal s_jitter_shift_eff  : integer range 0 to 7 := 0;

    -- ========================================================================
    -- Stage 2 (M6): Diode wavefolder LUTs.
    -- ========================================================================

    -- Eight ROM signals initialised from package constants — 4 Y-side
    -- banks + 4 UV-side banks.  Each bank holds 256 × 10b values mapping
    -- input → wavefolded output.  Initialised at declaration so Yosys
    -- recognises them as ROM (different inference path from the line
    -- buffers; init values are required for ROM, prohibited for RAM).
    signal lut_y_b0   : t_diode_lut := C_DIODE_Y_BANK_0;
    signal lut_y_b1   : t_diode_lut := C_DIODE_Y_BANK_1;
    signal lut_y_b2   : t_diode_lut := C_DIODE_Y_BANK_2;
    signal lut_y_b3   : t_diode_lut := C_DIODE_Y_BANK_3;
    signal lut_uv_b0  : t_diode_lut := C_DIODE_UV_BANK_0;
    signal lut_uv_b1  : t_diode_lut := C_DIODE_UV_BANK_1;
    signal lut_uv_b2  : t_diode_lut := C_DIODE_UV_BANK_2;
    signal lut_uv_b3  : t_diode_lut := C_DIODE_UV_BANK_3;

    -- Per-bank LUT read outputs at S3 (clocked).
    signal s_y_diode_b0  : unsigned(9 downto 0);
    signal s_y_diode_b1  : unsigned(9 downto 0);
    signal s_y_diode_b2  : unsigned(9 downto 0);
    signal s_y_diode_b3  : unsigned(9 downto 0);
    signal s_u_diode_b0  : unsigned(9 downto 0);
    signal s_u_diode_b1  : unsigned(9 downto 0);
    signal s_u_diode_b2  : unsigned(9 downto 0);
    signal s_u_diode_b3  : unsigned(9 downto 0);
    signal s_v_diode_b0  : unsigned(9 downto 0);
    signal s_v_diode_b1  : unsigned(9 downto 0);
    signal s_v_diode_b2  : unsigned(9 downto 0);
    signal s_v_diode_b3  : unsigned(9 downto 0);

    -- Bank index combinational compute + registered for S3 use.
    -- cv_global upper bits select base bank; Toggle 10 biases the range
    -- (Soft = 0..2, Hard = 1..3); sigma-delta dither between adjacent
    -- banks from cv_global lower bits gives smooth transitions.
    signal s_diode_dither_acc  : unsigned(3 downto 0) := (others => '0');
    signal s_diode_bank_idx_d3 : integer range 0 to 3 := 0;

    -- Diode LUT outputs at S4 (post-mux + register) — fed into Stage 3
    -- at M7.  UV inits to C_CHROMA_MID so transient sim 'U's don't
    -- propagate into rotation/IIR as green (pitfalls #4, #6).
    signal s_y_diode  : unsigned(9 downto 0) := (others => '0');
    signal s_u_diode  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_v_diode  : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- ========================================================================
    -- Stage 3 (M7): NTSC comb (H + V) + sin/cos hue twist.
    -- ========================================================================

    -- Companion-piped Y signals to align with s_y_diode at S5 timing.
    -- s_y_chain_d4 = s_y_chain(6) registered through to S4 (5-cycle delay
    -- from data_in.y) so the H comb delta is a horizontal high-pass.
    signal s_y_chain_d1      : std_logic_vector(9 downto 0) := (others => '0');
    signal s_y_chain_d2      : std_logic_vector(9 downto 0) := (others => '0');
    signal s_y_chain_d3      : std_logic_vector(9 downto 0) := (others => '0');
    signal s_y_chain_d4      : std_logic_vector(9 downto 0) := (others => '0');
    -- Companion-piped s_y_prev to align with s_y_diode at S5 timing.
    -- s_y_prev arrives at S2 (1-cycle BRAM read + 1-cycle output reg) so
    -- needs 2 more register stages to reach S4.
    signal s_y_prev_d3       : unsigned(9 downto 0) := (others => '0');
    signal s_y_prev_d4       : unsigned(9 downto 0) := (others => '0');

    -- Pre-comb Y at S4 (= s_y_diode) re-registered to S5 for the inject
    -- alignment (delta computed at S5; UV inject happens at S6).
    signal s_y_diode_d5      : unsigned(9 downto 0) := (others => '0');
    -- Diode UV companion-piped through to S6 to align with the comb delta.
    signal s_u_diode_d5      : unsigned(9 downto 0) := (others => '0');
    signal s_v_diode_d5      : unsigned(9 downto 0) := (others => '0');

    -- Comb delta (signed, registered at S5).  Includes Knob 4 shift table
    -- + sigma-delta dither.
    signal s_comb_dither_acc : unsigned(6 downto 0) := (others => '0');
    signal s_comb_shift_eff  : integer range 0 to 7 := 0;
    signal s_comb_delta      : signed(11 downto 0) := (others => '0');

    -- UV-injected output of Stage 3a (S6).  Inits to C_CHROMA_MID
    -- (pitfalls #4, #6).
    signal s_u_combed        : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_v_combed        : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- Companion-pipe Y through Stage 3 (no comb processing; Y is just
    -- delayed to keep alignment with the rotated UV).
    signal s_y_combed        : unsigned(9 downto 0) := (others => '0');

    -- Sin/cos hue twist (S7).  Angle composition + bin lookup combinational
    -- from s_y_combed at S6, registered at S7.  Rotation applied to UV
    -- centred around C_CHROMA_MID = 512.
    signal s_rot_angle       : unsigned(9 downto 0);
    signal s_rot_bin         : t_rot_bin;
    signal s_rot_dither_acc  : unsigned(5 downto 0) := (others => '0');
    signal s_rot_bin_eff     : integer range 0 to 15 := 0;

    -- Intermediate rotation products at S7a (after the first shift-add stage).
    -- These hold cos*u, sin*u, cos*v, sin*v separately so the second stage
    -- only does the final sum + clamp.  Splitting the rotation into two
    -- pipeline stages buys ~13 ns of timing budget (vs all-in-one which
    -- was failing at ~25 ns critical path).
    signal s_cos_u_d7a       : signed(13 downto 0) := (others => '0');
    signal s_sin_u_d7a       : signed(13 downto 0) := (others => '0');
    signal s_cos_v_d7a       : signed(13 downto 0) := (others => '0');
    signal s_sin_v_d7a       : signed(13 downto 0) := (others => '0');
    -- Y companion-pipe through S7a so S7b output keeps Y aligned.
    signal s_y_combed_d7a    : unsigned(9 downto 0) := (others => '0');
    signal s_uv_swap_d7a     : std_logic := '0';

    -- Stage 3 output (post-rotation + UV swap), at S7b timing.
    -- Init values to safe defaults so GHDL doesn't propagate 'U' into the
    -- downstream IIR state at simulation start (pitfall #4).  Y → 0, UV →
    -- C_CHROMA_MID per pitfall #6 (UV needs more protection — neutral grey
    -- is safe, 0 would render as green).
    signal s_y_rot           : unsigned(9 downto 0) := (others => '0');
    signal s_u_rot           : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_v_rot           : unsigned(9 downto 0) := C_CHROMA_MID;

    -- ========================================================================
    -- Stage 4 (M8): Capacitor bleed IIR + edge ringing.
    -- ========================================================================

    -- 19-bit signed IIR state per channel (1 sign + 10 data + 8 fractional).
    -- Idiom #3: fractional bits give smooth slow IIR without per-pixel
    -- staircase quantisation.  Resets at hsync_falling: Y → 0, UV → 512<<8.
    -- avid-gated update (pitfall #1).  Single-pass per line — pitfall #3
    -- bias does NOT apply (no multi-frame feedback).
    constant C_IIR_FRAC_BITS    : integer := 8;
    constant C_IIR_STATE_WIDTH  : integer := 19;
    -- 512 << 8 = 131072, the UV neutral state (pitfall #15 — UV resting
    -- value must be C_CHROMA_MID, never 0).
    constant C_IIR_UV_RESET     : integer := 512 * 256;

    signal s_iir_y           : signed(C_IIR_STATE_WIDTH - 1 downto 0)
                              := (others => '0');
    signal s_iir_u           : signed(C_IIR_STATE_WIDTH - 1 downto 0)
                              := to_signed(C_IIR_UV_RESET, C_IIR_STATE_WIDTH);
    signal s_iir_v           : signed(C_IIR_STATE_WIDTH - 1 downto 0)
                              := to_signed(C_IIR_UV_RESET, C_IIR_STATE_WIDTH);

    -- IIR k effective.  k_y_base from Knob 1 upper-3 (clamped 1..7);
    -- per-half envelope CV reduces k (faster decay = lighter smear in
    -- bright/active halves).  Sigma-delta dither on Knob 1 lower 7 bits
    -- per idiom #15.  k_uv_eff = k_y_eff + C_K_UV_OFFSET (UV trails
    -- slightly longer than Y, emphasising chromatic ghosting).
    constant C_K_UV_OFFSET   : integer := 1;
    signal s_k_y_dither_acc  : unsigned(6 downto 0) := (others => '0');
    signal s_k_y_eff         : integer range 1 to 7 := 1;
    signal s_k_uv_eff        : integer range 1 to 7 := 1;

    -- IIR output (S9 → S10 timing) — combinational from state's upper 10
    -- bits.  Drives the smear register at S10.
    signal s_iir_y_out       : unsigned(9 downto 0);
    signal s_iir_u_out       : unsigned(9 downto 0);
    signal s_iir_v_out       : unsigned(9 downto 0);

    -- Edge magnitude companion-pipe — s_edge_d0 (registered at S0 = depth
    -- 1 from data_in) is shifted through to S10 timing for alignment with
    -- the IIR output at S11.  Pipe length = 10 cycles to align with
    -- s_y_rot (= depth-10 from data_in via processing).  At depth 12 the
    -- shift-register is 9 stages wide (s_edge_d0 + 8 more = 9 levels of
    -- alignment).
    constant C_EDGE_PIPE_LEN : integer := 10;
    type t_edge_pipe is array (0 to C_EDGE_PIPE_LEN - 1) of unsigned(9 downto 0);
    signal s_edge_pipe       : t_edge_pipe := (others => (others => '0'));

    -- Edge ringing scale.  Knob 2 upper-3 → shift count, lower-7 → dither.
    signal s_ring_dither_acc : unsigned(6 downto 0) := (others => '0');
    signal s_ring_shift_eff  : integer range 0 to 15 := 15;

    -- Stage 4 output (smear) at S10 timing — drives the wet path at M8.
    signal s_smear_y         : unsigned(9 downto 0) := (others => '0');
    signal s_smear_u         : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_smear_v         : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Companion-pipe of s_y_rot (= IIR's input at S9) through to S10
    -- so the AC/DC mode at smear register can compute the mirror
    -- output = 2*input − state.  Two register stages: S9 and S10.
    signal s_y_rot_d9        : unsigned(9 downto 0) := (others => '0');
    signal s_y_rot_d10       : unsigned(9 downto 0) := (others => '0');

    -- ========================================================================
    -- Stage 5 (M9): Sync-crush bands + tape noise + master mix.
    -- ========================================================================

    -- Tape noise gain shift — driven by cv_global (envelope drives noise
    -- amplitude).  cv at 0 → shift 15 (silent); cv at max → shift 5
    -- (loud noise), with Knob 6 (Envelope Reactivity) scaling the
    -- coupling strength.
    signal s_noise_amp_shift : integer range 0 to 15 := 15;

    -- Sync-crush band trigger state.  s_band_active latches per-line at
    -- hsync_falling; deasserts at next hsync_falling unless re-fired.
    signal s_line_count        : unsigned(9 downto 0) := (others => '0');
    signal s_band_density_mask : unsigned(7 downto 0);
    signal s_band_active       : std_logic := '0';
    constant C_CRUSH_TH        : integer := 256;     -- cv_global > 256 to fire (~25% envelope)

    -- Stage 5a tape noise output (S11).
    signal s_noisy_y           : unsigned(9 downto 0) := (others => '0');
    signal s_noisy_u           : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_noisy_v           : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Stage 5b sync-crush mux output (S12) — drives interpolator b-input.
    signal s_crushed_y         : unsigned(9 downto 0) := (others => '0');
    signal s_crushed_u         : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_crushed_v         : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Dry-path taps for the master mix interpolator.  At depth 14 from
    -- data_in, sourced from the sync delay shift register.  These align
    -- with s_crushed_* (also at depth 14) so the interpolator's a/b
    -- inputs are the same source pixel.
    signal s_data_in_d14_y     : std_logic_vector(9 downto 0) := (others => '0');
    signal s_data_in_d14_u     : std_logic_vector(9 downto 0) := (others => '0');
    signal s_data_in_d14_v     : std_logic_vector(9 downto 0) := (others => '0');
    -- avid at the interpolator's input — used as enable for the
    -- interpolator instances (4-cycle sync between dry and wet).
    signal s_avid_d14          : std_logic := '0';

    -- IIR-gate signals at depth 10 (matches s_y_rot/u_rot/v_rot data
    -- timing).  See pitfall #19 — gating the IIR at depth 1 while
    -- consuming data at depth 10 caused per-line blanking-zero
    -- contamination of UV state.
    signal s_avid_d10          : std_logic := '0';
    signal s_hsync_start_d10   : std_logic := '0';

    -- Master mix interpolator outputs at depth 18.
    signal s_interp_y          : unsigned(9 downto 0);
    signal s_interp_u          : unsigned(9 downto 0);
    signal s_interp_v          : unsigned(9 downto 0);

    -- "Wet" output — at M9 this is the master mix output post-interpolator.
    -- Connects to data_out with R1 broadcast-safe clamp.
    signal s_wet_y             : unsigned(9 downto 0);
    signal s_wet_u             : unsigned(9 downto 0);
    signal s_wet_v             : unsigned(9 downto 0);

begin

    -- ========================================================================
    -- Knob decode (concurrent — pots are 10-bit per register).
    -- ========================================================================
    s_cap_bleed       <= unsigned(registers_in(0));
    s_edge_resonance  <= unsigned(registers_in(1));
    s_chroma_twist    <= unsigned(registers_in(2));
    s_ntsc_comb       <= unsigned(registers_in(3));
    s_tbc_jitter      <= unsigned(registers_in(4));
    s_envelope_react  <= unsigned(registers_in(5));
    s_master_mix      <= unsigned(registers_in(7));

    -- ========================================================================
    -- Toggle decode (concurrent — registers_in(6) bit-packed).
    -- ========================================================================
    s_ac_dc           <= registers_in(6)(0);
    s_edge_phase_inv  <= registers_in(6)(1);
    s_uv_swap         <= registers_in(6)(2);
    s_diode_base      <= registers_in(6)(3);
    s_sync_crush      <= registers_in(6)(4);

    -- ========================================================================
    -- Module: video_timing_generator — extracts edge pulses from sync inputs.
    -- ========================================================================
    timing_gen : entity work.video_timing_generator
        port map(
            clk         => clk,
            ref_hsync_n => data_in.hsync_n,
            ref_vsync_n => data_in.vsync_n,
            ref_avid    => data_in.avid,
            timing      => s_timing
        );

    -- ========================================================================
    -- Module: lfsr16 — declared at M2, unused until M9.
    -- ========================================================================
    lfsr_inst : entity work.lfsr16
        port map(
            clk    => clk,
            enable => '1',
            load   => '0',
            seed   => x"BEEF",
            q      => s_lfsr_q
        );

    -- ========================================================================
    -- S0a: hcount — 0 at first active pixel, increments while avid=1.
    -- ========================================================================
    p_hcount : process(clk)
    begin
        if rising_edge(clk) then
            if data_in.avid = '0' then
                s_hcount <= (others => '0');
            else
                s_hcount <= s_hcount + 1;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- S0: Edge magnitude — |Y(n) - Y(n-1)| as 10-bit unsigned.
    -- Used at M3 by the envelope edge-energy accumulator and at M8 by the
    -- edge-ringing add.
    -- ========================================================================
    p_edge_mag : process(clk)
        variable v_y_now : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_y_now := unsigned(data_in.y);
            s_y_d_prev_pix <= v_y_now;
            if v_y_now > s_y_d_prev_pix then
                s_edge_d0 <= v_y_now - s_y_d_prev_pix;
            else
                s_edge_d0 <= s_y_d_prev_pix - v_y_now;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- S0c: vcount — line counter within active frame.  Reset on
    -- vsync_falling, increment on hsync_falling.
    -- ========================================================================
    p_vcount : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_vcount <= (others => '0');
            elsif s_timing.hsync_start = '1' then
                s_vcount <= s_vcount + 1;
            end if;
        end if;
    end process;

    -- s_in_top_half is REGISTERED (1-cycle lag).  Comparison from a wide
    -- s_vcount + downstream combinational fan-out into TBC compute and
    -- envelope accumulators created the critical path at M7 (Fmax 40 MHz
    -- pre-fix).  Registering breaks the chain at no aesthetic cost since
    -- s_in_top_half changes only at hsync boundaries (line transitions).
    -- Compares against the runtime-derived half height (pitfall #5).
    p_in_top_half : process(clk)
    begin
        if rising_edge(clk) then
            if s_vcount < s_active_half_height_dyn then
                s_in_top_half <= '1';
            else
                s_in_top_half <= '0';
            end if;
        end if;
    end process;

    -- Latch the just-completed frame's active row count and halve it
    -- for the spatial-split comparison.  At vsync_start, s_vcount holds
    -- the full row count; shift_right by 1 gives the midpoint.  Used
    -- by p_in_top_half to drive the spatial-half flag.
    p_active_half_runtime : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_active_half_height_dyn <= shift_right(s_vcount, 1);
            end if;
        end if;
    end process;

    -- ========================================================================
    -- S0d (resolution-aware): count active pixels in the top spatial half
    -- and derive log2(count) as the dynamic envelope-divisor shift.  This
    -- is the runtime-derived equivalent of a static `÷ half-frame-pixels`
    -- and is the program's pitfall #5 mitigation for the envelope
    -- accumulator.
    --
    -- The priority encoder finds the MSB position of the just-completed
    -- frame's top-half pixel count.  Updated once per frame at vsync_start.
    -- Default = 20 so the first frame after reset uses 1080p calibration.
    -- ========================================================================
    p_avg_shift_runtime : process(clk)
        variable v_msb : integer range 4 to 23;
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                -- Find the MSB position of the just-completed frame's
                -- half-frame pixel count.  Use MSB + 1 (= ceil(log2(N)))
                -- as the divisor shift so the truncated mean fits in 10
                -- bits without overflow at peak frames.
                v_msb := 4;
                for i in 23 downto 4 loop
                    if s_pixel_count_top(i) = '1' then
                        v_msb := i;
                        exit;
                    end if;
                end loop;
                s_avg_shift_dyn   <= v_msb + 1;
                s_pixel_count_top <= (others => '0');
            elsif data_in.avid = '1' and s_in_top_half = '1' then
                s_pixel_count_top <= s_pixel_count_top + 1;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- S0d/e: Per-half luma + edge accumulators.  Sum during active video
    -- in whichever half the current pixel lies; reset all four on
    -- vsync_falling AFTER the per-half latch step below.
    -- ========================================================================
    p_envelope_acc : process(clk)
        variable v_luma_top_mean : unsigned(C_ENV_ACC_WIDTH - 1 downto 0);
        variable v_luma_bot_mean : unsigned(C_ENV_ACC_WIDTH - 1 downto 0);
        variable v_edge_top_mean : unsigned(C_ENV_ACC_WIDTH - 1 downto 0);
        variable v_edge_bot_mean : unsigned(C_ENV_ACC_WIDTH - 1 downto 0);
        variable v_leak_y_top    : unsigned(9 downto 0);
        variable v_leak_y_bot    : unsigned(9 downto 0);
        variable v_leak_e_top    : unsigned(9 downto 0);
        variable v_leak_e_bot    : unsigned(9 downto 0);
        variable v_lat_y_top     : unsigned(9 downto 0);
        variable v_lat_y_bot     : unsigned(9 downto 0);
        variable v_lat_e_top     : unsigned(9 downto 0);
        variable v_lat_e_bot     : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                -- Latch the per-half means by shifting accumulator down
                -- to 10-bit data range.  s_avg_shift_dyn = log2(half-frame
                -- pixel count), derived at runtime per video mode/decim
                -- (see p_avg_shift_runtime — pitfall #5 mitigation).
                -- Imprecise but bounded; used as a CV, not a precise mean.
                v_luma_top_mean := shift_right(s_luma_acc_top, s_avg_shift_dyn);
                v_luma_bot_mean := shift_right(s_luma_acc_bot, s_avg_shift_dyn);
                v_edge_top_mean := shift_right(s_edge_acc_top, s_avg_shift_dyn);
                v_edge_bot_mean := shift_right(s_edge_acc_bot, s_avg_shift_dyn);

                -- Clamp to 10-bit (lower 10 bits of each mean).
                v_lat_y_top := v_luma_top_mean(9 downto 0);
                v_lat_y_bot := v_luma_bot_mean(9 downto 0);
                v_lat_e_top := v_edge_top_mean(9 downto 0);
                v_lat_e_bot := v_edge_bot_mean(9 downto 0);

                -- Leak: cv_old - (cv_old >> 8) ≈ 99.6% retention per frame.
                -- At 60fps: half-life ≈ 175 frames ≈ 2.9s.  Adjust shift if
                -- decay too slow/fast in hardware playtest.
                v_leak_y_top := s_cv_luma_top - shift_right(s_cv_luma_top, 8);
                v_leak_y_bot := s_cv_luma_bot - shift_right(s_cv_luma_bot, 8);
                v_leak_e_top := s_cv_edge_top - shift_right(s_cv_edge_top, 8);
                v_leak_e_bot := s_cv_edge_bot - shift_right(s_cv_edge_bot, 8);

                -- Peak-hold: max(latched_mean, leaked_old).  Instant attack,
                -- slow decay.
                if v_lat_y_top > v_leak_y_top then s_cv_luma_top <= v_lat_y_top;
                else                                s_cv_luma_top <= v_leak_y_top; end if;
                if v_lat_y_bot > v_leak_y_bot then s_cv_luma_bot <= v_lat_y_bot;
                else                                s_cv_luma_bot <= v_leak_y_bot; end if;
                if v_lat_e_top > v_leak_e_top then s_cv_edge_top <= v_lat_e_top;
                else                                s_cv_edge_top <= v_leak_e_top; end if;
                if v_lat_e_bot > v_leak_e_bot then s_cv_edge_bot <= v_lat_e_bot;
                else                                s_cv_edge_bot <= v_leak_e_bot; end if;

                -- Reset accumulators for the next frame.
                s_luma_acc_top <= (others => '0');
                s_luma_acc_bot <= (others => '0');
                s_edge_acc_top <= (others => '0');
                s_edge_acc_bot <= (others => '0');

            elsif data_in.avid = '1' then
                -- Active video: sum into the appropriate half's accumulators.
                if s_in_top_half = '1' then
                    s_luma_acc_top <= s_luma_acc_top
                                      + resize(unsigned(data_in.y), C_ENV_ACC_WIDTH);
                    s_edge_acc_top <= s_edge_acc_top
                                      + resize(s_edge_d0, C_ENV_ACC_WIDTH);
                else
                    s_luma_acc_bot <= s_luma_acc_bot
                                      + resize(unsigned(data_in.y), C_ENV_ACC_WIDTH);
                    s_edge_acc_bot <= s_edge_acc_bot
                                      + resize(s_edge_d0, C_ENV_ACC_WIDTH);
                end if;
            end if;
        end if;
    end process;

    -- Combined global CV: max of (luma + edge) per half, halved to 10-bit.
    -- 11-bit intermediate to avoid overflow on the sum.
    p_cv_global : process(clk)
        variable v_top_sum : unsigned(10 downto 0);
        variable v_bot_sum : unsigned(10 downto 0);
        variable v_max_sum : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            v_top_sum := resize(s_cv_luma_top, 11) + resize(s_cv_edge_top, 11);
            v_bot_sum := resize(s_cv_luma_bot, 11) + resize(s_cv_edge_bot, 11);
            if v_top_sum > v_bot_sum then v_max_sum := v_top_sum;
            else                          v_max_sum := v_bot_sum; end if;
            -- ÷2 → 10-bit.  Saturates at 1023 if both halves fully bright +
            -- edge-saturated, which is fine.
            s_cv_global <= v_max_sum(10 downto 1);
        end if;
    end process;

    -- ========================================================================
    -- Frame-phase drift — SDK frame_phase_accumulator instance.
    -- Speed = (cv_global + envelope_react) >> 4: envelope-modulated drift.
    -- M3 status: declared, advance happens, output not consumed yet.
    -- ========================================================================
    p_phase_speed : process(clk)
        variable v_sum     : unsigned(10 downto 0);
        variable v_shifted : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            v_sum     := resize(s_cv_global, 11) + resize(s_envelope_react, 11);
            v_shifted := shift_right(v_sum, 4);
            s_phase_speed <= v_shifted(9 downto 0);
        end if;
    end process;

    phase_drift_inst : entity work.frame_phase_accumulator
        generic map(
            G_PHASE_WIDTH => 16,
            G_SPEED_WIDTH => 10
        )
        port map(
            clk     => clk,
            vsync_n => data_in.vsync_n,
            enable  => '1',
            speed   => s_phase_speed,
            phase   => s_phase_drift
        );

    -- ========================================================================
    -- S0: Input register — separates data_in from downstream combinational
    -- logic.
    -- ========================================================================
    p_input_reg : process(clk)
    begin
        if rising_edge(clk) then
            s_y_d0    <= data_in.y;
            s_u_d0    <= data_in.u;
            s_v_d0    <= data_in.v;
            s_avid_d0 <= data_in.avid;
        end if;
    end process;

    -- ========================================================================
    -- S0: Free-running write address — mod 2048, avid-gated.
    -- Distinct from s_hcount (which resets per line) so that prev-line
    -- access at offset 1920 actually returns the same column from the
    -- previous active line.  variable_delay_u-style semantics.
    -- ========================================================================
    p_wr_addr : process(clk)
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_wr_addr <= s_wr_addr + 1;     -- wraps mod 2048 naturally
            end if;
        end if;
    end process;

    -- ========================================================================
    -- S0: Y input shift-register chain — fixed NTSC comb tap at stage 6.
    -- Declared at M5; consumer arrives at M7 (NTSC comb compute).  Yosys
    -- may prune until then; that's fine.
    -- ========================================================================
    p_y_chain : process(clk)
    begin
        if rising_edge(clk) then
            s_y_chain(0) <= s_y_d0;
            for i in 1 to 6 loop
                s_y_chain(i) <= s_y_chain(i - 1);
            end loop;
        end if;
    end process;

    -- ========================================================================
    -- S0: Sigma-delta dither for Knob 5 lower 7 bits (idiom #15).
    -- Effective shift amount alternates between two adjacent levels at a
    -- density set by Knob 5 LSBs, giving smooth perceived TBC control.
    -- ========================================================================
    p_jitter_dither : process(clk)
        variable v_acc_next : unsigned(7 downto 0);
        variable v_shift_hi : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            -- Sigma-delta accumulator on Knob 5 lower 7 bits.
            v_acc_next := ('0' & s_jitter_dither_acc) +
                          ('0' & unsigned(registers_in(4)(6 downto 0)));
            s_jitter_dither_acc <= v_acc_next(6 downto 0);

            -- Knob 5 upper 3 bits = base shift level (0..7); s_jitter_shift_eff
            -- = 7 maps to "TBC jitter contributes 1/128 of v_sum" (essentially
            -- nothing); s_jitter_shift_eff = 0 maps to full v_sum.
            v_shift_hi := 7 - to_integer(unsigned(registers_in(4)(9 downto 7)));
            -- When dither acc overflows, use one less shift (more jitter).
            if v_acc_next(7) = '1' and v_shift_hi > 0 then
                s_jitter_shift_eff <= v_shift_hi - 1;
            else
                s_jitter_shift_eff <= v_shift_hi;
            end if;
        end if;
    end process;

    -- Pre-register the spatial-half CV selection (Fmax fix — see signal
    -- declaration s_cv_luma_half_d above).  Used by p_k_eff for per-half
    -- IIR smear modulation.
    p_cv_half_reg : process(clk)
    begin
        if rising_edge(clk) then
            if s_in_top_half = '1' then
                s_cv_luma_half_d <= s_cv_luma_top;
            else
                s_cv_luma_half_d <= s_cv_luma_bot;
            end if;
        end if;
    end process;

    p_tbc_jitter : process(clk)
        variable v_random  : unsigned(3 downto 0);  -- 0..15 from LFSR
        variable v_knob    : unsigned(3 downto 0);  -- 0..15 from Knob 5
        variable v_env     : unsigned(3 downto 0);  -- 0..15 from envelope*react
        variable v_max     : unsigned(4 downto 0);  -- knob+env, clamped below
    begin
        if rising_edge(clk) then
            -- TBC jitter is a PER-LINE effect.  Real analog TBC errors
            -- shift whole lines (line-rate timing instability) and
            -- adjacent lines having different offsets produces the
            -- characteristic horizontal "tearing" / "wobble".  Per-pixel
            -- offset variation produces pixel-rate scramble, not tear,
            -- so the offset is latched at hsync_start to hold constant
            -- within a line.
            --
            -- Each line samples LFSR(3..0) (0..15) and clamps to v_max.
            -- v_max is derived from BOTH:
            --   * Knob 5 (TBC Jitter): primary intensity control.
            --   * Envelope × Knob 6 (Env React): video content modulates
            --     jitter intensity — bright/active content → more tear.
            -- This makes the input video itself a modulation source: the
            -- TBC tear gets stronger when content is bright/edgey.
            -- s_phase_speed = (cv_global + env_react) >> 4 is the
            -- pre-computed envelope×react composite (0..127); we take
            -- its upper 4 bits (0..7) as the env contribution.  Knob 6=0
            -- masks the env contribution to 0; Knob 6 max boosts v_max
            -- by up to 7 above the knob 5 setting.
            if s_timing.hsync_start = '1' then
                v_random := unsigned(s_lfsr_q(3 downto 0));
                v_knob   := unsigned(registers_in(4)(9 downto 6));
                if s_envelope_react(9 downto 6) = "0000" then
                    v_env := (others => '0');
                else
                    v_env := "0" & s_phase_speed(6 downto 4);
                end if;
                -- Sum, clamp to 15.
                v_max := resize(v_knob, 5) + resize(v_env, 5);
                if v_max > to_unsigned(15, 5) then
                    v_max := to_unsigned(15, 5);
                end if;

                if unsigned(registers_in(4)) = to_unsigned(0, 10) then
                    s_tbc_offset <= (others => '0');
                elsif v_random > v_max(3 downto 0) then
                    s_tbc_offset <= "00000" & v_max(3 downto 0);
                else
                    s_tbc_offset <= "00000" & v_random;
                end if;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- S0d (resolution-aware): count active pixels per line; latch at
    -- hsync_start as the runtime prev-line offset.  Replaces the static
    -- C_PREV_OFFSET = 1920 so the NTSC comb's vertical delta tap reads
    -- exactly 1 line back at any video mode + decimation.  At hardware
    -- 1080p this lands on 1920; at decim=4 it lands on 480.  Default
    -- 1920 so the first frame after reset uses 1080p calibration.
    -- Pitfall #5 mitigation alongside s_avg_shift_dyn and
    -- s_active_half_height_dyn.
    -- ========================================================================
    p_prev_offset_runtime : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_pixels_per_line /= to_unsigned(0, C_BUF_DEPTH) then
                    s_prev_offset_dyn <= s_pixels_per_line;
                end if;
                s_pixels_per_line <= (others => '0');
            elsif data_in.avid = '1' then
                s_pixels_per_line <= s_pixels_per_line + 1;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- S0/S1: BRAM read addresses (combinational from s_wr_addr + jitter).
    -- All four address arithmetic done in 11-bit unsigned (modular,
    -- wraps mod 2048 = C_BUF_SIZE).  s_prev_offset_dyn is the runtime-
    -- derived line-length offset (pitfall #5 mitigation).
    -- ========================================================================
    p_rd_addrs : process(s_wr_addr, s_tbc_offset, s_prev_offset_dyn)
        variable v_tbc_offset : unsigned(C_BUF_DEPTH - 1 downto 0);
    begin
        v_tbc_offset := to_unsigned(C_TBC_BASE, C_BUF_DEPTH)
                        + resize(s_tbc_offset, C_BUF_DEPTH);
        s_rd_addr_tbc_y  <= s_wr_addr - v_tbc_offset;
        s_rd_addr_tbc_u  <= s_wr_addr - v_tbc_offset;
        s_rd_addr_tbc_v  <= s_wr_addr - v_tbc_offset;
        s_rd_addr_prev_y <= s_wr_addr - s_prev_offset_dyn;
    end process;

    -- ========================================================================
    -- S1: BRAM write processes — one per channel, gated by s_avid_d0.
    -- Per gravity_bleed Option D canonical pattern (no init values, single
    -- clocked write process per array).
    -- ========================================================================
    p_wr_y : process(clk)
    begin
        if rising_edge(clk) then
            if s_avid_d0 = '1' then
                bram_y(to_integer(s_wr_addr)) <= s_y_d0;
            end if;
        end if;
    end process;

    p_wr_u : process(clk)
    begin
        if rising_edge(clk) then
            if s_avid_d0 = '1' then
                bram_u(to_integer(s_wr_addr)) <= s_u_d0;
            end if;
        end if;
    end process;

    p_wr_v : process(clk)
    begin
        if rising_edge(clk) then
            if s_avid_d0 = '1' then
                bram_v(to_integer(s_wr_addr)) <= s_v_d0;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- S1: BRAM read processes — Y bank 2 reads (TBC + prev), U+V banks 1
    -- read each (TBC only).  Each read is its own clocked process so Yosys
    -- infers them as independent SB_RAM40_4K read ports replicating the
    -- storage (Layer 0 verified pattern).
    -- ========================================================================
    p_rd_y_tbc : process(clk)
    begin
        if rising_edge(clk) then
            s_y_tbc_raw <= bram_y(to_integer(s_rd_addr_tbc_y));
        end if;
    end process;

    p_rd_y_prev : process(clk)
    begin
        if rising_edge(clk) then
            s_y_prev_raw <= bram_y(to_integer(s_rd_addr_prev_y));
        end if;
    end process;

    p_rd_u_tbc : process(clk)
    begin
        if rising_edge(clk) then
            s_u_tbc_raw <= bram_u(to_integer(s_rd_addr_tbc_u));
        end if;
    end process;

    p_rd_v_tbc : process(clk)
    begin
        if rising_edge(clk) then
            s_v_tbc_raw <= bram_v(to_integer(s_rd_addr_tbc_v));
        end if;
    end process;

    -- ========================================================================
    -- S2: BRAM output registration (idiom #7) + to_01 sanitiser.
    -- Breaks the BRAM-to-logic critical path and stops GHDL 'U'
    -- propagation.  At M5 these signals drive directly into s_wet_*.
    -- ========================================================================
    p_bram_out_reg : process(clk)
    begin
        if rising_edge(clk) then
            s_y_tbc  <= unsigned(to_01(unsigned(s_y_tbc_raw),  '0'));
            s_y_prev <= unsigned(to_01(unsigned(s_y_prev_raw), '0'));
            s_u_tbc  <= unsigned(to_01(unsigned(s_u_tbc_raw),  '0'));
            s_v_tbc  <= unsigned(to_01(unsigned(s_v_tbc_raw),  '0'));
        end if;
    end process;

    -- ========================================================================
    -- S3: Diode wavefolder LUT reads.  Y-side: 4 ROMs each read once at
    -- s_y_tbc(9..2).  UV-side: 4 ROMs each read TWICE (U + V channels) —
    -- relies on Yosys mapping this as dual-port ROM (1 EBR per bank).
    -- If Yosys replicates instead, we get 12 EBR for Stage 2 vs 8
    -- expected; will refactor in that case.
    -- ========================================================================

    -- Y bank reads (4 processes, 1 read port each)
    p_rd_lut_y0 : process(clk) begin
        if rising_edge(clk) then
            s_y_diode_b0 <= lut_y_b0(to_integer(s_y_tbc(9 downto 2)));
        end if;
    end process;
    p_rd_lut_y1 : process(clk) begin
        if rising_edge(clk) then
            s_y_diode_b1 <= lut_y_b1(to_integer(s_y_tbc(9 downto 2)));
        end if;
    end process;
    p_rd_lut_y2 : process(clk) begin
        if rising_edge(clk) then
            s_y_diode_b2 <= lut_y_b2(to_integer(s_y_tbc(9 downto 2)));
        end if;
    end process;
    p_rd_lut_y3 : process(clk) begin
        if rising_edge(clk) then
            s_y_diode_b3 <= lut_y_b3(to_integer(s_y_tbc(9 downto 2)));
        end if;
    end process;

    -- UV bank reads — 8 processes (4 banks × 2 channels), shared storage
    p_rd_lut_u0 : process(clk) begin
        if rising_edge(clk) then
            s_u_diode_b0 <= lut_uv_b0(to_integer(s_u_tbc(9 downto 2)));
        end if;
    end process;
    p_rd_lut_v0 : process(clk) begin
        if rising_edge(clk) then
            s_v_diode_b0 <= lut_uv_b0(to_integer(s_v_tbc(9 downto 2)));
        end if;
    end process;
    p_rd_lut_u1 : process(clk) begin
        if rising_edge(clk) then
            s_u_diode_b1 <= lut_uv_b1(to_integer(s_u_tbc(9 downto 2)));
        end if;
    end process;
    p_rd_lut_v1 : process(clk) begin
        if rising_edge(clk) then
            s_v_diode_b1 <= lut_uv_b1(to_integer(s_v_tbc(9 downto 2)));
        end if;
    end process;
    p_rd_lut_u2 : process(clk) begin
        if rising_edge(clk) then
            s_u_diode_b2 <= lut_uv_b2(to_integer(s_u_tbc(9 downto 2)));
        end if;
    end process;
    p_rd_lut_v2 : process(clk) begin
        if rising_edge(clk) then
            s_v_diode_b2 <= lut_uv_b2(to_integer(s_v_tbc(9 downto 2)));
        end if;
    end process;
    p_rd_lut_u3 : process(clk) begin
        if rising_edge(clk) then
            s_u_diode_b3 <= lut_uv_b3(to_integer(s_u_tbc(9 downto 2)));
        end if;
    end process;
    p_rd_lut_v3 : process(clk) begin
        if rising_edge(clk) then
            s_v_diode_b3 <= lut_uv_b3(to_integer(s_v_tbc(9 downto 2)));
        end if;
    end process;

    -- ========================================================================
    -- S2/S3: Diode bank index — sigma-delta dither between adjacent banks
    -- driven by cv_global lower bits, base bank from upper bits, Toggle 10
    -- biases the active range (Soft = 0..2, Hard = 1..3).
    -- ========================================================================
    p_diode_bank : process(clk)
        variable v_acc_next : unsigned(4 downto 0);
        variable v_base_idx : integer range 0 to 3;
    begin
        if rising_edge(clk) then
            -- Sigma-delta accumulator on cv_global lower 4 bits.  When the
            -- accumulator overflows, advance one bank — perceived smoothly.
            v_acc_next := ('0' & s_diode_dither_acc) + ('0' & s_cv_global(3 downto 0));
            s_diode_dither_acc <= v_acc_next(3 downto 0);

            -- Base bank from cv_global bits [9:8] (top 2 bits of 10-bit CV).
            v_base_idx := to_integer(s_cv_global(9 downto 8));
            -- Toggle 10 (Diode Base) shifts the active range.
            if s_diode_base = '1' and v_base_idx < 3 then
                v_base_idx := v_base_idx + 1;
            end if;

            -- Dither overflow advances one more bank (clamped at 3).
            if v_acc_next(4) = '1' and v_base_idx < 3 then
                s_diode_bank_idx_d3 <= v_base_idx + 1;
            else
                s_diode_bank_idx_d3 <= v_base_idx;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- S4: Diode LUT bank-select mux + register.  4-way mux selects the
    -- live bank's output for each channel.
    -- ========================================================================
    p_diode_mux : process(clk)
    begin
        if rising_edge(clk) then
            case s_diode_bank_idx_d3 is
                when 0 =>
                    s_y_diode <= s_y_diode_b0;
                    s_u_diode <= s_u_diode_b0;
                    s_v_diode <= s_v_diode_b0;
                when 1 =>
                    s_y_diode <= s_y_diode_b1;
                    s_u_diode <= s_u_diode_b1;
                    s_v_diode <= s_v_diode_b1;
                when 2 =>
                    s_y_diode <= s_y_diode_b2;
                    s_u_diode <= s_u_diode_b2;
                    s_v_diode <= s_v_diode_b2;
                when others =>
                    s_y_diode <= s_y_diode_b3;
                    s_u_diode <= s_u_diode_b3;
                    s_v_diode <= s_v_diode_b3;
            end case;
        end if;
    end process;

    -- ========================================================================
    -- Stage 3 companion pipes — align s_y_chain(6), s_y_prev with the
    -- s_y_diode timing at S5.  s_y_chain(6) at clock T is data_in.y[T-7];
    -- s_y_diode at clock T is data_in.y[T-5] (with TBC + diode processing).
    -- So they're naturally 2 active samples apart already; the companion
    -- pipes here just bring s_y_chain into the same pipeline-register
    -- timing for the comb delta to be a clean clocked subtraction.
    -- ========================================================================
    p_stage3_pipes : process(clk)
    begin
        if rising_edge(clk) then
            s_y_chain_d1 <= s_y_chain(6);
            s_y_chain_d2 <= s_y_chain_d1;
            s_y_chain_d3 <= s_y_chain_d2;
            s_y_chain_d4 <= s_y_chain_d3;
            s_y_prev_d3  <= s_y_prev;
            s_y_prev_d4  <= s_y_prev_d3;
            s_y_diode_d5 <= s_y_diode;
            s_u_diode_d5 <= s_u_diode;
            s_v_diode_d5 <= s_v_diode;
        end if;
    end process;

    -- ========================================================================
    -- S5 prep: NTSC comb shift amount + sigma-delta dither (Knob 4).
    -- ========================================================================
    p_comb_dither : process(clk)
        variable v_acc_next : unsigned(7 downto 0);
        variable v_shift_hi : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            v_acc_next := ('0' & s_comb_dither_acc) +
                          ('0' & unsigned(registers_in(3)(6 downto 0)));
            s_comb_dither_acc <= v_acc_next(6 downto 0);

            v_shift_hi := 7 - to_integer(unsigned(registers_in(3)(9 downto 7)));
            if v_acc_next(7) = '1' and v_shift_hi > 0 then
                s_comb_shift_eff <= v_shift_hi - 1;
            else
                s_comb_shift_eff <= v_shift_hi;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- S5: NTSC comb compute.  delta_h = s_y_diode - s_y_chain_d4 (horizontal
    -- high-pass, ~2-sample edge).  delta_v = s_y_diode - s_y_prev_d4
    -- (vertical, prev-line same column).  Sum, then shift by Knob 4 scale.
    -- Pitfall #14: opposite-signed UV inject is required to avoid the
    -- magenta/green axis collapse — handled at S6.
    -- ========================================================================
    p_comb_compute : process(clk)
        variable v_y_now             : signed(11 downto 0);
        variable v_y_chain           : signed(11 downto 0);
        variable v_y_prev            : signed(11 downto 0);
        variable v_delta_h           : signed(11 downto 0);
        variable v_delta_v           : signed(11 downto 0);
        variable v_delta             : signed(11 downto 0);
        variable v_env_boost         : integer range 0 to 2;
        variable v_comb_shift_pixel  : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            v_y_now   := signed(resize(s_y_diode,                     12));
            v_y_chain := signed(resize(unsigned(s_y_chain_d4),        12));
            v_y_prev  := signed(resize(s_y_prev_d4,                   12));

            v_delta_h := v_y_now - v_y_chain;
            v_delta_v := v_y_now - v_y_prev;
            v_delta   := v_delta_h + v_delta_v;

            -- Knob 6 unlocks an envelope-driven boost on top of Knob 4:
            -- frames with high cv_global get more visible NTSC comb
            -- artefacts.  Originally tried gating on cv_edge_half_d but
            -- at sim resolution the per-half edge accumulator rarely
            -- reaches its upper bits (even on bars, cv_edge stays around
            -- 30..50 of 1023).  cv_global aggregates luma+edge across
            -- both halves and is more likely to engage at sim while still
            -- being meaningful at hardware.  Pitfall #20 zero-gate:
            -- env_react = 0 → boost = 0.  Capped at 2 so Knob 4 stays
            -- primary.
            v_env_boost := 0;
            if (s_envelope_react(9) and s_cv_global(9)) = '1' then
                v_env_boost := v_env_boost + 1;
            end if;
            if (s_envelope_react(8) and s_cv_global(8)) = '1' then
                v_env_boost := v_env_boost + 1;
            end if;
            v_comb_shift_pixel := s_comb_shift_eff - v_env_boost;
            if v_comb_shift_pixel < 0 then
                v_comb_shift_pixel := 0;
            end if;

            -- Scale by Knob 4 shift (with env boost); signed shift_right is
            -- OK here (single-pass, no multi-frame feedback — pitfall #3
            -- doesn't apply).
            s_comb_delta <= shift_right(v_delta, v_comb_shift_pixel);
        end if;
    end process;

    -- ========================================================================
    -- S6: NTSC UV inject (opposite-signed) + Y companion register.
    --   s_u_combed = clamp(s_u_diode_d5 + s_comb_delta, 0, 1023)
    --   s_v_combed = clamp(s_v_diode_d5 - s_comb_delta, 0, 1023)
    -- Pitfall #14: U gets +delta, V gets -delta to produce hue rotation
    -- around grey rather than the magenta/green axis.
    -- ========================================================================
    p_comb_inject : process(clk)
        variable v_u_sum : signed(11 downto 0);
        variable v_v_sum : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            v_u_sum := signed(resize(s_u_diode_d5, 12)) + s_comb_delta;
            v_v_sum := signed(resize(s_v_diode_d5, 12)) - s_comb_delta;
            -- Clamp to 10-bit unsigned (pre-rotation; rotation has its
            -- own clamp at the output).
            if v_u_sum < 0 then
                s_u_combed <= (others => '0');
            elsif v_u_sum > 1023 then
                s_u_combed <= to_unsigned(1023, 10);
            else
                s_u_combed <= unsigned(v_u_sum(9 downto 0));
            end if;
            if v_v_sum < 0 then
                s_v_combed <= (others => '0');
            elsif v_v_sum > 1023 then
                s_v_combed <= to_unsigned(1023, 10);
            else
                s_v_combed <= unsigned(v_v_sum(9 downto 0));
            end if;
            -- Y passes through unchanged at this stage (rotation only
            -- touches UV; Y is just delayed to maintain alignment).
            s_y_combed <= s_y_diode_d5;
        end if;
    end process;

    -- ========================================================================
    -- S7 prep: Sin/cos rotation angle compute + bin lookup (combinational).
    -- Angle = (s_y_combed - (cv_global << 2) + Knob 3) mod 1024.
    -- Bin = angle(9 downto 6).  Sigma-delta dither between adjacent bins
    -- from angle(5 downto 0).
    --
    -- Envelope SUBTRACTS from the angle (rather than adding) so that bright
    -- frames rotate hue OPPOSITE to the per-pixel Y-driven rotation.  This
    -- pulls the hue character "the other way" on energetic content,
    -- producing more obvious image-modulated colour swings than the
    -- previous additive design (where Y and cv reinforced each other and
    -- the visual result felt like a constant hue offset across content).
    -- Subtraction on unsigned wraps mod 4096 in 12-bit; the lower-10-bit
    -- mask then takes mod 1024.  No negative values escape, no out-of-
    -- range index — the angle just lands at different valid bins per
    -- frame depending on cv.
    -- ========================================================================
    p_rot_angle : process(s_y_combed, s_cv_global, s_chroma_twist)
        variable v_angle : unsigned(11 downto 0);
    begin
        v_angle := resize(s_y_combed, 12)
                 - resize(shift_left(s_cv_global, 2), 12)
                 + resize(s_chroma_twist, 12);
        -- mod 1024 = lower 10 bits
        s_rot_angle <= v_angle(9 downto 0);
    end process;

    p_rot_dither : process(clk)
        variable v_acc_next : unsigned(6 downto 0);
        variable v_bin_base : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            -- Sigma-delta accumulator on angle lower 6 bits.
            v_acc_next := ('0' & s_rot_dither_acc) +
                          ('0' & s_rot_angle(5 downto 0));
            s_rot_dither_acc <= v_acc_next(5 downto 0);

            v_bin_base := to_integer(s_rot_angle(9 downto 6));
            -- Dither overflow advances one bin (wraps 15 → 0).
            if v_acc_next(6) = '1' then
                if v_bin_base = 15 then
                    s_rot_bin_eff <= 0;
                else
                    s_rot_bin_eff <= v_bin_base + 1;
                end if;
            else
                s_rot_bin_eff <= v_bin_base;
            end if;
        end if;
    end process;

    -- Bin coefficient lookup — combinational from C_ROT_TABLE constant.
    s_rot_bin <= C_ROT_TABLE(s_rot_bin_eff);

    -- ========================================================================
    -- S7a: Sin/cos rotation — first half (compute the 4 shift-add products
    -- cos*u, sin*u, cos*v, sin*v in parallel and register).  Splitting the
    -- rotation into two stages keeps the combinational depth per cycle
    -- under the 13.47 ns budget at 74.25 MHz.  Without this split, Fmax
    -- was 40.97 MHz (FAIL).
    -- ========================================================================
    p_rotation_s7a : process(clk)
        variable v_u_c, v_v_c       : signed(11 downto 0);
        variable v_cos_u, v_cos_v   : signed(13 downto 0);
        variable v_sin_u, v_sin_v   : signed(13 downto 0);

        procedure shift_add_3 (val      : in  signed(11 downto 0);
                               sa, sb, sc : in integer range 0 to 15;
                               signs    : in std_logic_vector(2 downto 0);
                               result   : out signed(13 downto 0)) is
            variable acc  : signed(13 downto 0);
            variable t    : signed(13 downto 0);
        begin
            acc := (others => '0');
            t   := resize(shift_right(val, sa), 14);
            if signs(0) = '1' then acc := acc - t; else acc := acc + t; end if;
            t   := resize(shift_right(val, sb), 14);
            if signs(1) = '1' then acc := acc - t; else acc := acc + t; end if;
            t   := resize(shift_right(val, sc), 14);
            if signs(2) = '1' then acc := acc - t; else acc := acc + t; end if;
            result := acc;
        end procedure;
    begin
        if rising_edge(clk) then
            v_u_c := signed(resize(s_u_combed, 12)) - to_signed(512, 12);
            v_v_c := signed(resize(s_v_combed, 12)) - to_signed(512, 12);

            shift_add_3(v_u_c, s_rot_bin.cos_a, s_rot_bin.cos_b, s_rot_bin.cos_c,
                        s_rot_bin.cos_signs, v_cos_u);
            shift_add_3(v_u_c, s_rot_bin.sin_a, s_rot_bin.sin_b, s_rot_bin.sin_c,
                        s_rot_bin.sin_signs, v_sin_u);
            shift_add_3(v_v_c, s_rot_bin.cos_a, s_rot_bin.cos_b, s_rot_bin.cos_c,
                        s_rot_bin.cos_signs, v_cos_v);
            shift_add_3(v_v_c, s_rot_bin.sin_a, s_rot_bin.sin_b, s_rot_bin.sin_c,
                        s_rot_bin.sin_signs, v_sin_v);

            s_cos_u_d7a   <= v_cos_u;
            s_sin_u_d7a   <= v_sin_u;
            s_cos_v_d7a   <= v_cos_v;
            s_sin_v_d7a   <= v_sin_v;
            s_y_combed_d7a <= s_y_combed;
            s_uv_swap_d7a <= s_uv_swap;
        end if;
    end process;

    -- ========================================================================
    -- S7b: Sin/cos rotation — second half (sum the products, add midpoint,
    -- clamp, UV swap mux, register).  Cleanly meets timing.
    -- ========================================================================
    p_rotation_s7b : process(clk)
        variable v_u_rot, v_v_rot : signed(13 downto 0);
        variable v_u_out, v_v_out : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- u_rot = u_c*cos - v_c*sin;  v_rot = u_c*sin + v_c*cos
            v_u_rot := s_cos_u_d7a - s_sin_v_d7a;
            v_v_rot := s_sin_u_d7a + s_cos_v_d7a;

            -- Add midpoint back (signed comparison handles overflow).
            v_u_out := v_u_rot + to_signed(512, 14);
            v_v_out := v_v_rot + to_signed(512, 14);

            if s_uv_swap_d7a = '1' then
                -- Swap U and V.
                if v_v_out < 0 then s_u_rot <= (others => '0');
                elsif v_v_out > 1023 then s_u_rot <= to_unsigned(1023, 10);
                else s_u_rot <= unsigned(v_v_out(9 downto 0)); end if;
                if v_u_out < 0 then s_v_rot <= (others => '0');
                elsif v_u_out > 1023 then s_v_rot <= to_unsigned(1023, 10);
                else s_v_rot <= unsigned(v_u_out(9 downto 0)); end if;
            else
                if v_u_out < 0 then s_u_rot <= (others => '0');
                elsif v_u_out > 1023 then s_u_rot <= to_unsigned(1023, 10);
                else s_u_rot <= unsigned(v_u_out(9 downto 0)); end if;
                if v_v_out < 0 then s_v_rot <= (others => '0');
                elsif v_v_out > 1023 then s_v_rot <= to_unsigned(1023, 10);
                else s_v_rot <= unsigned(v_v_out(9 downto 0)); end if;
            end if;

            s_y_rot <= s_y_combed_d7a;
        end if;
    end process;

    -- ========================================================================
    -- S0 → S10: edge magnitude companion-pipe.  s_edge_d0 (depth 1) is
    -- shifted through to align with the IIR output at S10/S11 timing.
    -- ========================================================================
    p_edge_pipe : process(clk)
    begin
        if rising_edge(clk) then
            s_edge_pipe(0) <= s_edge_d0;
            for i in 1 to C_EDGE_PIPE_LEN - 1 loop
                s_edge_pipe(i) <= s_edge_pipe(i - 1);
            end loop;
        end if;
    end process;

    -- ========================================================================
    -- S0/S1: IIR k_eff compute — Knob 1 (Cap Bleed) + per-half envelope
    -- modulation + sigma-delta dither.  k_y_eff: 1..7, where lower k =
    -- slower decay = longer smear.  k_uv_eff = k_y_eff + C_K_UV_OFFSET.
    -- ========================================================================
    p_k_eff : process(clk)
        variable v_acc_next : unsigned(7 downto 0);
        variable v_k_base   : integer range 0 to 7;
        variable v_k_env    : integer range -7 to 7;
        variable v_k_y      : integer range -7 to 14;
        variable v_cv_half  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- Sigma-delta accumulator on Knob 1 lower 7 bits.
            v_acc_next := ('0' & s_k_y_dither_acc) +
                          ('0' & unsigned(registers_in(0)(6 downto 0)));
            s_k_y_dither_acc <= v_acc_next(6 downto 0);

            -- Base k = Knob1(9..7).  Raw 0 promoted to 1 below.
            -- For our signed-IIR form (`state += (input - state) >> k`):
            --   k=1 → fast tracking = NO smear (state ≈ input)
            --   k=7 → slow tracking = MAX smear (state stuck on old value)
            -- So Knob 1 = 0 → k=1 (no smear, light decay) per design intent.
            -- (NB: gravity_bleed uses the inverted mapping `7 - top3`
            -- because its unsigned-IIR identity has the opposite k polarity.
            -- The two forms give the same aesthetic from the same knob;
            -- only the internal k value differs.)
            v_k_base := to_integer(unsigned(registers_in(0)(9 downto 7)));

            -- Per-half envelope CV — bright/edge-active halves shorten
            -- smear.  v_k_env in 0..7 (cv >> 7 maps 0..1023 to 0..7).
            if s_in_top_half = '1' then
                v_cv_half := s_cv_luma_top;
            else
                v_cv_half := s_cv_luma_bot;
            end if;
            v_k_env := to_integer(v_cv_half(9 downto 7));

            -- Combine + sigma-delta dither overflow + clamp 1..7.
            v_k_y := v_k_base - v_k_env;
            if v_acc_next(7) = '1' and v_k_y > 1 then
                v_k_y := v_k_y - 1;
            end if;
            if v_k_y < 1 then
                s_k_y_eff <= 1;
            elsif v_k_y > 7 then
                s_k_y_eff <= 7;
            else
                s_k_y_eff <= v_k_y;
            end if;

            -- UV trails slightly longer.  Same dither + base; +offset.
            if v_k_y + C_K_UV_OFFSET < 1 then
                s_k_uv_eff <= 1;
            elsif v_k_y + C_K_UV_OFFSET > 7 then
                s_k_uv_eff <= 7;
            else
                s_k_uv_eff <= v_k_y + C_K_UV_OFFSET;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- S0/S1: Edge ringing scale — Knob 2 (Edge Resonance) shift table +
    -- sigma-delta dither.  Higher Knob 2 = lower shift = louder ringing.
    -- ========================================================================
    p_ring_dither : process(clk)
        variable v_acc_next : unsigned(7 downto 0);
        variable v_shift_hi : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            v_acc_next := ('0' & s_ring_dither_acc) +
                          ('0' & unsigned(registers_in(1)(6 downto 0)));
            s_ring_dither_acc <= v_acc_next(6 downto 0);

            -- Knob 2 = 0 → shift 15 (fully attenuated).  Knob 2 max →
            -- shift 1.  v_shift_hi = 15 - Knob2(9..7) << 1 (so step is
            -- 2 shifts at a time, mapping 0..7 → 15, 13, 11, 9, 7, 5, 3, 1).
            v_shift_hi := 15 - 2 * to_integer(unsigned(registers_in(1)(9 downto 7)));
            -- Dither overflow → one less shift (slightly louder).
            if v_acc_next(7) = '1' and v_shift_hi > 1 then
                s_ring_shift_eff <= v_shift_hi - 1;
            else
                s_ring_shift_eff <= v_shift_hi;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- S9: IIR state register — per-channel inline IIR with fractional bits
    -- (idiom #3).  Reset on hsync_falling (line boundary, capacitor
    -- discharge).  Update gated by avid (pitfall #1: state must NOT decay
    -- during blanking).  Single-pass per line — pitfall #3 signed
    -- shift_right bias does NOT apply.
    --
    -- AC/DC coupling (Toggle 7) flips the polarity of Y's delta:
    --   DC (0): bright pixels smear into dark (delta = err >> k, normal).
    --   AC (1): dark pixels cast negative shadow-smears into bright
    --           (delta = -err >> k, inverted polarity).
    -- ========================================================================
    p_iir_state : process(clk)
        variable v_input_y : signed(C_IIR_STATE_WIDTH - 1 downto 0);
        variable v_input_u : signed(C_IIR_STATE_WIDTH - 1 downto 0);
        variable v_input_v : signed(C_IIR_STATE_WIDTH - 1 downto 0);
        variable v_err_y   : signed(C_IIR_STATE_WIDTH - 1 downto 0);
        variable v_err_u   : signed(C_IIR_STATE_WIDTH - 1 downto 0);
        variable v_err_v   : signed(C_IIR_STATE_WIDTH - 1 downto 0);
        variable v_delta_y : signed(C_IIR_STATE_WIDTH - 1 downto 0);
        variable v_delta_u : signed(C_IIR_STATE_WIDTH - 1 downto 0);
        variable v_delta_v : signed(C_IIR_STATE_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            -- Both reset and update gates use depth-10 signals to align
            -- with s_y_rot/u_rot/v_rot timing (pitfall #19).  Gating on
            -- depth-1 signals here would cause 9 cycles of blanking-zero
            -- input per active line → UV state collapses to 0 = green.
            if s_hsync_start_d10 = '1' then
                -- Reset at hsync_falling (depth-10 aligned): Y → 0
                -- (black), UV → 512<<8 (neutral grey, pitfall #15 —
                -- never 0 for UV).
                s_iir_y <= (others => '0');
                s_iir_u <= to_signed(C_IIR_UV_RESET, C_IIR_STATE_WIDTH);
                s_iir_v <= to_signed(C_IIR_UV_RESET, C_IIR_STATE_WIDTH);
            elsif s_avid_d10 = '1' then
                -- IIR update: state += (input - state) >> k_eff.
                -- Inputs are 10-bit unsigned, shifted left by
                -- C_IIR_FRAC_BITS to enter the state's data range
                -- (state(18..8) = data, state(7..0) = fractional acc).
                -- Use to_signed(... * 256) to avoid the "0" & slv overload
                -- ambiguity flagged in testing-guide.md.
                v_input_y := to_signed(to_integer(s_y_rot) * 256,
                                       C_IIR_STATE_WIDTH);
                v_input_u := to_signed(to_integer(s_u_rot) * 256,
                                       C_IIR_STATE_WIDTH);
                v_input_v := to_signed(to_integer(s_v_rot) * 256,
                                       C_IIR_STATE_WIDTH);

                v_err_y := v_input_y - s_iir_y;
                v_err_u := v_input_u - s_iir_u;
                v_err_v := v_input_v - s_iir_v;

                v_delta_y := shift_right(v_err_y, s_k_y_eff);
                v_delta_u := shift_right(v_err_u, s_k_uv_eff);
                v_delta_v := shift_right(v_err_v, s_k_uv_eff);

                -- IIR state ALWAYS converges to input (no AC/DC sign flip
                -- here).  AC/DC is applied at the smear-register stage
                -- as a MIRROR (output = 2*input − state) which produces
                -- the "negative shadow-smear" effect without making the
                -- IIR runaway-unstable.  Earlier versions flipped the
                -- delta sign in the state-update path; this caused
                -- state to diverge to ±2^18 (R7 stuck-state violation).
                s_iir_y <= s_iir_y + v_delta_y;
                s_iir_u <= s_iir_u + v_delta_u;
                s_iir_v <= s_iir_v + v_delta_v;
            end if;
        end if;
    end process;

    -- IIR output: combinational from state's upper 10 data bits.
    s_iir_y_out <= unsigned(s_iir_y(C_IIR_STATE_WIDTH - 2 downto C_IIR_FRAC_BITS));
    s_iir_u_out <= unsigned(s_iir_u(C_IIR_STATE_WIDTH - 2 downto C_IIR_FRAC_BITS));
    s_iir_v_out <= unsigned(s_iir_v(C_IIR_STATE_WIDTH - 2 downto C_IIR_FRAC_BITS));

    -- ========================================================================
    -- s_y_rot companion pipe to S10 — needed by the smear register's
    -- AC/DC mode mirror compute.  s_y_rot is at S9 timing; s_y_rot_d9 is
    -- 1 cycle later (S10), s_y_rot_d10 is 2 cycles later (matches smear
    -- register timing for the IIR input that drove the current state).
    -- ========================================================================
    p_y_rot_pipe : process(clk)
    begin
        if rising_edge(clk) then
            s_y_rot_d9  <= s_y_rot;
            s_y_rot_d10 <= s_y_rot_d9;
        end if;
    end process;

    -- ========================================================================
    -- S10: Smear register — IIR output (with optional AC/DC mirror) +
    -- edge ringing add (Y only).
    --
    -- AC/DC coupling (Toggle 7):
    --   DC (0): smear_y = state (capacitor smear lags input — bright
    --           pixels leave bright trails into dark areas).
    --   AC (1): smear_y = 2*input − state (capacitor anti-smear —
    --           dark pixels cast NEGATIVE shadow-smears into bright
    --           areas, and vice versa).  Stable: input drives state
    --           normally; AC just MIRRORS the smear contribution.
    --
    -- Toggle 8 (Edge Phase Invert) flips the ringing sign:
    --   off (0): bright outlines (edge adds to Y).
    --   on  (1): dark outlines  (edge subtracts from Y).
    -- ========================================================================
    p_smear_reg : process(clk)
        -- 14-bit signed throughout: v_y_smear in AC mode reaches ~2*input
        -- (max ~2046), and the boosted v_edge_sh adds another ±1023 — sum
        -- up to ~3000 must not wrap before the final clamp catches it.
        variable v_y_iir              : signed(13 downto 0);
        variable v_y_in               : signed(13 downto 0);
        variable v_y_smear            : signed(13 downto 0);
        variable v_edge               : signed(13 downto 0);
        variable v_edge_sh            : signed(13 downto 0);
        variable v_y_sum              : signed(13 downto 0);
        variable v_env_boost          : integer range 0 to 2;
        variable v_ring_shift_pixel   : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            -- Apply AC/DC at the smear contribution.  v_y_smear is the
            -- value that gets the edge-ringing add and the final clamp.
            v_y_iir := signed(resize(s_iir_y_out, 14));
            v_y_in  := signed(resize(s_y_rot_d10, 14));
            if s_ac_dc = '1' then
                -- AC: mirror state around input.  output = 2*input − state.
                v_y_smear := shift_left(v_y_in, 1) - v_y_iir;
            else
                v_y_smear := v_y_iir;
            end if;

            -- Edge ringing: shift the aligned edge magnitude, optionally
            -- negate (Toggle 8), add to Y output.
            -- Knob 6 unlocks an envelope-driven boost on top of Knob 2:
            -- when both s_envelope_react upper bits AND s_cv_global upper
            -- bits are set, reduce the shift by up to 2 (4× louder edge
            -- ringing on busy frames).  Pitfall #20 zero-gate: env_react
            -- = 0 → no boost.  Capped at 2 so it never overrides Knob 2's
            -- primary control.
            v_env_boost := 0;
            if (s_envelope_react(9) and s_cv_global(9)) = '1' then
                v_env_boost := v_env_boost + 1;
            end if;
            if (s_envelope_react(8) and s_cv_global(8)) = '1' then
                v_env_boost := v_env_boost + 1;
            end if;
            v_ring_shift_pixel := s_ring_shift_eff - v_env_boost;
            if v_ring_shift_pixel < 1 then
                v_ring_shift_pixel := 1;
            end if;
            -- 2× edge boost (shift_left 1) before the Knob 2 attenuation.
            -- Natural images have soft adjacent-pixel deltas (5..30) which
            -- the original mid-Knob-2 shift = 7 rounded to ~0; Toggle 8
            -- (Edge Phase Invert) had nothing to flip.  Doubling the edge
            -- magnitude here lets soft edges register at moderate Knob 2
            -- settings while preserving Knob 2 = 0 → silent (shift = 15
            -- still drops a 2× boosted edge to ~0).
            v_edge    := shift_left(signed(resize(s_edge_pipe(C_EDGE_PIPE_LEN - 1), 14)), 1);
            v_edge_sh := shift_right(v_edge, v_ring_shift_pixel);
            if s_edge_phase_inv = '1' then
                v_edge_sh := -v_edge_sh;
            end if;

            -- Add to smear Y, clamp to 10-bit unsigned.
            v_y_sum := v_y_smear + v_edge_sh;
            if v_y_sum < 0 then
                s_smear_y <= (others => '0');
            elsif v_y_sum > 1023 then
                s_smear_y <= to_unsigned(1023, 10);
            else
                s_smear_y <= unsigned(v_y_sum(9 downto 0));
            end if;

            -- UV passes through the IIR output unchanged at this stage.
            -- (AC/DC affects Y only per the design — UV always DC-coupled.)
            s_smear_u <= s_iir_u_out;
            s_smear_v <= s_iir_v_out;
        end if;
    end process;

    -- ========================================================================
    -- Stage 5a (S11): Tape noise add — env-modulated signed noise
    -- centred around 0 from lfsr16.  Adding signed-around-0 noise to
    -- UV (centred around 512) keeps UV around midpoint — no green
    -- collapse (pitfall #15).
    -- ========================================================================

    -- Combinational: noise amplitude shift derived from cv_global.
    -- High envelope → low shift = loud noise.  Knob 6 (Envelope
    -- Reactivity) scales this coupling.
    p_noise_amp : process(s_cv_global, s_envelope_react)
        variable v_eff_cv : unsigned(10 downto 0);
    begin
        -- Combine envelope CV with reactivity knob.
        v_eff_cv := resize(s_cv_global, 11) + resize(s_envelope_react, 11);
        -- Shift = 15 - (eff_cv >> 8), clamped to [5, 15] range.
        if v_eff_cv > 1792 then
            s_noise_amp_shift <= 5;
        elsif v_eff_cv > 1536 then
            s_noise_amp_shift <= 7;
        elsif v_eff_cv > 1024 then
            s_noise_amp_shift <= 9;
        elsif v_eff_cv > 512  then
            s_noise_amp_shift <= 11;
        elsif v_eff_cv > 256  then
            s_noise_amp_shift <= 13;
        else
            s_noise_amp_shift <= 15;
        end if;
    end process;

    p_tape_noise : process(clk)
        variable v_noise         : signed(9 downto 0);
        variable v_y_sum         : signed(11 downto 0);
        variable v_y_signed      : signed(10 downto 0);
        variable v_y_dev         : signed(10 downto 0);
        variable v_y_abs         : unsigned(10 downto 0);
        variable v_y_extreme     : integer range 0 to 3;
        variable v_noise_shift   : integer range 0 to 31;
    begin
        if rising_edge(clk) then
            -- Tape noise is Y-domain ONLY (luma graininess).  Adding the
            -- same signed noise to U and V would displace UV along the
            -- magenta/green axis (pitfall #14 — equal-signed UV offset is
            -- NOT chroma modulation, it's a single-axis colour wiggle).
            -- For a future "chroma noise" we'd use independent LFSR taps
            -- or opposite-signed UV; deferred to V2.
            v_noise := signed(resize(unsigned(s_lfsr_q(15 downto 8)), 10))
                       - to_signed(128, 10);

            -- Per-pixel Y-correlation: noise is loudest at midtones
            -- (where tape grit naturally lives) and quieter at extremes.
            -- v_y_dev = abs(s_smear_y - 512) → 0 at midtone, 511 at extremes.
            -- Upper 2 bits give 0..3 of additional shift → up to 8× quieter
            -- at peak white/black.  Always-on (no env_react gate) — this
            -- is a character change, not a modulation hookup.  Pitfall #5
            -- not relevant here: thresholds at Y=384/640 are scene-luma,
            -- not envelope-CV-derived.
            v_y_signed := signed(resize(s_smear_y, 11));
            v_y_dev    := v_y_signed - to_signed(512, 11);
            if v_y_dev(10) = '1' then
                v_y_abs := unsigned(-v_y_dev);
            else
                v_y_abs := unsigned(v_y_dev);
            end if;
            v_y_extreme := to_integer(v_y_abs(8 downto 7));
            v_noise_shift := s_noise_amp_shift + v_y_extreme;
            v_noise := shift_right(v_noise, v_noise_shift);

            v_y_sum := signed(resize(s_smear_y, 12)) + resize(v_noise, 12);

            if    v_y_sum < 0    then s_noisy_y <= (others => '0');
            elsif v_y_sum > 1023 then s_noisy_y <= to_unsigned(1023, 10);
            else                       s_noisy_y <= unsigned(v_y_sum(9 downto 0));
            end if;

            -- UV pass through unchanged (no tape noise inject).
            s_noisy_u <= s_smear_u;
            s_noisy_v <= s_smear_v;
        end if;
    end process;

    -- ========================================================================
    -- Stage 5b prep: sync-crush band density mask (combinational from
    -- cv_global upper bits) — higher CV → narrower mask = bands fire more
    -- frequently per frame.
    -- ========================================================================
    p_band_density : process(s_cv_global)
    begin
        case to_integer(s_cv_global(9 downto 7)) is
            when 0      => s_band_density_mask <= "11111111";  -- never
            when 1      => s_band_density_mask <= "01111111";  -- ~1/128 lines
            when 2      => s_band_density_mask <= "00111111";  -- ~1/64
            when 3      => s_band_density_mask <= "00011111";  -- ~1/32
            when 4      => s_band_density_mask <= "00001111";  -- ~1/16
            when 5      => s_band_density_mask <= "00000111";  -- ~1/8
            when 6      => s_band_density_mask <= "00000011";  -- ~1/4
            when others => s_band_density_mask <= "00000001";  -- every other line
        end case;
    end process;

    -- ========================================================================
    -- Stage 5b: sync-crush band trigger.  Latches s_band_active for the
    -- current line at hsync_falling.  Conditions: Toggle 11 enabled,
    -- cv_global > C_CRUSH_TH, line_count masked-equal-to-zero, LFSR bit set.
    -- ========================================================================
    p_band_trigger : process(clk)
        variable v_density_match  : boolean;
        variable v_threshold_drop : integer range 0 to 127;
        variable v_eff_threshold  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_line_count <= (others => '0');
                s_band_active <= '0';
            elsif s_timing.hsync_start = '1' then
                v_density_match := (s_line_count(7 downto 0) and s_band_density_mask)
                                   = "00000000";

                -- Knob 6 (Env React) lowers the sync-crush threshold by up
                -- to 127 LSBs (env_react upper 7 bits = 0..127), so at
                -- full env_react crush triggers at cv_global > 129 instead
                -- of > 256.  Pitfall #20 zero-gate: env_react = 0 → drop
                -- = 0 → effective threshold = C_CRUSH_TH unchanged.
                -- Range (256..129) keeps the threshold meaningfully
                -- gated; the original 256..193 (env_react(9:4)) was too
                -- narrow to validate at sim cv_global magnitudes.
                v_threshold_drop := to_integer(unsigned(s_envelope_react(9 downto 3)));
                v_eff_threshold  := to_unsigned(C_CRUSH_TH - v_threshold_drop, 10);

                if s_sync_crush = '1'
                   and s_lfsr_q(0) = '1'
                   and v_density_match
                   and s_cv_global > v_eff_threshold then
                    s_band_active <= '1';
                else
                    s_band_active <= '0';
                end if;
                s_line_count <= s_line_count + 1;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Stage 5b: sync-crush mux at S12.  When s_band_active is asserted,
    -- force Y → 0 and UV → C_CHROMA_MID (NOT 0 — pitfall #15).
    -- Otherwise pass s_noisy_*.
    -- ========================================================================
    p_band_mux : process(clk)
    begin
        if rising_edge(clk) then
            if s_band_active = '1' then
                s_crushed_y <= (others => '0');
                s_crushed_u <= C_CHROMA_MID;
                s_crushed_v <= C_CHROMA_MID;
            else
                s_crushed_y <= s_noisy_y;
                s_crushed_u <= s_noisy_u;
                s_crushed_v <= s_noisy_v;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Stage 5c: Master mix — interpolator_u × 3 channels.
    -- a-input: dry path tap from sync delay shift register at depth 14.
    -- b-input: wet path s_crushed_*.
    -- t: registers_in(7) (Master Mix slider).
    -- 4-cycle latency → outputs at depth 18.
    -- ========================================================================
    interp_y : entity work.interpolator_u
        generic map(
            G_WIDTH      => 10,
            G_FRAC_BITS  => 10,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_avid_d14,
            a      => unsigned(s_data_in_d14_y),
            b      => s_crushed_y,
            t      => s_master_mix,
            result => s_interp_y,
            valid  => open
        );

    interp_u : entity work.interpolator_u
        generic map(
            G_WIDTH      => 10,
            G_FRAC_BITS  => 10,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_avid_d14,
            a      => unsigned(s_data_in_d14_u),
            b      => s_crushed_u,
            t      => s_master_mix,
            result => s_interp_u,
            valid  => open
        );

    interp_v : entity work.interpolator_u
        generic map(
            G_WIDTH      => 10,
            G_FRAC_BITS  => 10,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_avid_d14,
            a      => unsigned(s_data_in_d14_v),
            b      => s_crushed_v,
            t      => s_master_mix,
            result => s_interp_v,
            valid  => open
        );

    -- ========================================================================
    -- M9 wet path: master-mix interpolator output.  Goes through R1 clamp
    -- at the data_out concurrent assignment below.
    -- ========================================================================
    s_wet_y <= s_interp_y;
    s_wet_u <= s_interp_u;
    s_wet_v <= s_interp_v;

    -- ========================================================================
    -- Output port: broadcast-safe clamp (R1).
    -- Y → [64, 940], U/V → [64, 960] (per pre-hardware safety audit).
    -- ========================================================================
    data_out.y <= std_logic_vector(C_Y_MIN)  when s_wet_y < C_Y_MIN  else
                  std_logic_vector(C_Y_MAX)  when s_wet_y > C_Y_MAX  else
                  std_logic_vector(s_wet_y);
    data_out.u <= std_logic_vector(C_UV_MIN) when s_wet_u < C_UV_MIN else
                  std_logic_vector(C_UV_MAX) when s_wet_u > C_UV_MAX else
                  std_logic_vector(s_wet_u);
    data_out.v <= std_logic_vector(C_UV_MIN) when s_wet_v < C_UV_MIN else
                  std_logic_vector(C_UV_MAX) when s_wet_v > C_UV_MAX else
                  std_logic_vector(s_wet_v);

    -- ========================================================================
    -- Sync delay: C_PROCESSING_DELAY_CLKS = 18.
    -- Carries sync flags + Y/U/V dry-path companion pipes for the
    -- master-mix interpolator's "a" input at depth 14.  All variables
    -- initialised to inactive defaults (idiom #4) so GHDL doesn't
    -- propagate 'U' through the early pipeline.
    -- ========================================================================
    p_sync_delay : process(clk)
        type t_sync_delay is array(0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic;
        type t_data_delay is array(0 to C_PROCESSING_DELAY_CLKS - 1)
            of std_logic_vector(9 downto 0);
        variable v_avid        : t_sync_delay := (others => '0');
        variable v_hsync       : t_sync_delay := (others => '1');
        variable v_vsync       : t_sync_delay := (others => '1');
        variable v_field       : t_sync_delay := (others => '1');
        -- hsync_start pulse pipe — needed at depth 10 for IIR reset
        -- (1-cycle pulse, must be piped through registers, can't be
        -- derived combinationally from v_hsync without extra edge logic).
        variable v_hsync_start : t_sync_delay := (others => '0');
        variable v_y_clean     : t_data_delay := (others => (others => '0'));
        variable v_u_clean     : t_data_delay := (others => (others => '0'));
        variable v_v_clean     : t_data_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_avid        := data_in.avid          & v_avid(0        to C_PROCESSING_DELAY_CLKS - 2);
            v_hsync       := data_in.hsync_n       & v_hsync(0       to C_PROCESSING_DELAY_CLKS - 2);
            v_vsync       := data_in.vsync_n       & v_vsync(0       to C_PROCESSING_DELAY_CLKS - 2);
            v_field       := data_in.field_n       & v_field(0       to C_PROCESSING_DELAY_CLKS - 2);
            v_hsync_start := s_timing.hsync_start  & v_hsync_start(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_y_clean     := data_in.y             & v_y_clean(0     to C_PROCESSING_DELAY_CLKS - 2);
            v_u_clean     := data_in.u             & v_u_clean(0     to C_PROCESSING_DELAY_CLKS - 2);
            v_v_clean     := data_in.v             & v_v_clean(0     to C_PROCESSING_DELAY_CLKS - 2);

            -- Sync outputs at full pipeline depth (18 cycles).
            data_out.avid    <= v_avid(C_PROCESSING_DELAY_CLKS - 1);
            data_out.hsync_n <= v_hsync(C_PROCESSING_DELAY_CLKS - 1);
            data_out.vsync_n <= v_vsync(C_PROCESSING_DELAY_CLKS - 1);
            data_out.field_n <= v_field(C_PROCESSING_DELAY_CLKS - 1);

            -- IIR-gate taps at depth 10 (= s_y_rot data timing).  Tap
            -- index = depth - 1 (variable indices 0-based, v_array(K) at
            -- clock T+1 = data_in at T-K = K+1 cycles delayed).
            s_avid_d10        <= v_avid(C_IIR_GATE_DEPTH - 1);
            s_hsync_start_d10 <= v_hsync_start(C_IIR_GATE_DEPTH - 1);

            -- Dry-path data taps at depth 14 (= interpolator's "a" input).
            s_data_in_d14_y <= v_y_clean(C_DRY_TAP_DEPTH - 1);
            s_data_in_d14_u <= v_u_clean(C_DRY_TAP_DEPTH - 1);
            s_data_in_d14_v <= v_v_clean(C_DRY_TAP_DEPTH - 1);
            -- avid at depth 14 enables the interpolator instances.
            s_avid_d14      <= v_avid(C_DRY_TAP_DEPTH - 1);
        end if;
    end process;

end architecture sync_bleed;
