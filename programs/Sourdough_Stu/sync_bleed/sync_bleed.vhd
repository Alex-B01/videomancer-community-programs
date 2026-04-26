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
-- =====  M7 STATUS — STAGE 3 NTSC COMB + SIN/COS HUE TWIST  =====
-- Prior milestones: M2 (d2ccea1), M3 (075eda3), M4+M5 (3e7bb0e), M6 (this).
-- This commit lands Stage 3:
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
--   * TBC jitter compute combining four modulation sources scaled by
--     Knob 5 (s_tbc_jitter): content-luma deviation, per-half envelope
--     CV, motion proxy (Y_now - Y_prev_line), frame-phase drift.
--     Sigma-delta dither on Knob 5 LSBs (idiom #15).
--   * Motion proxy register: signed(s_y_d2) - signed(s_y_prev), updated
--     at S2.  TBC jitter compute at S1 uses the previous clock's motion
--     proxy (1-pixel pipeline lag, aesthetically irrelevant).
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
    -- Pipeline depth grows incrementally per milestone.  Final target = 18
    -- at M9 (interpolator output drives data_out).  Currently at M7:
    --   1 clk  : S0   input registration, hcount/vcount, lfsr advance,
    --                 edge mag, wr_addr increment, Y chain shift
    --   1 clk  : S1   TBC jitter compute (registered — broke the
    --                 critical path from s_in_top_half to BRAM read DFF)
    --   1 clk  : S2   BRAM read (rd_addr combinational from registered
    --                 s_tbc_offset)
    --   1 clk  : S3   BRAM output registration + to_01 sanitise
    --   1 clk  : S4   Diode LUT bank reads (8 ROMs in parallel)
    --   1 clk  : S5   Diode LUT bank-select mux + register
    --   1 clk  : S6   NTSC comb compute (delta_h + delta_v + scale)
    --   1 clk  : S7   UV inject (opposite-signed) + register
    --   1 clk  : S8a  Sin/cos rotation: shift-add products
    --                 (cos*u, sin*u, cos*v, sin*v in parallel)
    --   1 clk  : S8b  Sin/cos rotation: sum + clamp + UV swap mux
    -- ─────────
    --  10 total at M7
    --
    -- Future targets:
    --   M8 (IIR + ringing):       +2  → 12 clocks
    --   M9 (sync-crush + interp): +6  → 18 clocks (with 4-clock interp)
    constant C_PROCESSING_DELAY_CLKS : integer := 10;

    -- BRAM line buffer geometry — 2048 deep covers 1080p active line (1920) + headroom.
    constant C_BUF_SIZE      : integer := 2048;
    constant C_BUF_DEPTH     : integer := 11;
    constant C_PREV_OFFSET   : integer := 1920;     -- prev-line tap offset
    constant C_TBC_BASE      : integer := 16;       -- baseline TBC read offset
                                                    -- (with Knob 5 = 0, read is wr_addr - 16)

    -- Active video frame dimensions (1080p).  Used for spatial-half flag
    -- and envelope accumulator divisor.  Resolution-dependent — pitfall #5.
    constant C_ACTIVE_HEIGHT      : integer := 1080;
    constant C_ACTIVE_HALF_HEIGHT : integer := 540;
    -- Half-frame pixel count = 1920 × 540 ≈ 2^20.  Right-shift the
    -- accumulator by this to get a ~10-bit mean.  Pitfall #5 risk: at
    -- sim resolution (480 × 270 ≈ 2^17) this gives smaller CVs.  Live
    -- with it for V1; document in test-plan.
    constant C_AVG_SHIFT          : integer := 20;

    -- Envelope accumulator width.  1920 × 540 × 1023 ≈ 1.06e9 < 2^30, so
    -- 32 bits gives comfortable headroom.
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
    -- select, sync-crush rate, tape noise gain).
    signal s_cv_global       : unsigned(9 downto 0);

    -- Frame-phase drift — 16-bit DDS phase, advances at frame rate.  Speed
    -- = (cv_global + envelope_react) >> 4 so phase advance is envelope-
    -- modulated (busy frames drift faster).
    signal s_phase_drift     : unsigned(15 downto 0);
    signal s_phase_speed     : unsigned(9 downto 0);

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

    -- Motion proxy: signed(s_y_d2) − signed(s_y_prev), updated at S2 timing.
    -- TBC jitter compute at S1 reads the previous clock's value (1-pixel
    -- pipeline lag, aesthetically irrelevant — "more motion → more tear").
    signal s_y_d1            : std_logic_vector(9 downto 0) := (others => '0');
    signal s_y_d2            : std_logic_vector(9 downto 0) := (others => '0');
    signal s_motion_proxy    : signed(10 downto 0) := (others => '0');

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
    -- at M7, currently drive the wet path directly.
    signal s_y_diode  : unsigned(9 downto 0);
    signal s_u_diode  : unsigned(9 downto 0);
    signal s_v_diode  : unsigned(9 downto 0);

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

    -- UV-injected output of Stage 3a (S6).
    signal s_u_combed        : unsigned(9 downto 0);
    signal s_v_combed        : unsigned(9 downto 0);
    -- Companion-pipe Y through Stage 3 (no comb processing; Y is just
    -- delayed to keep alignment with the rotated UV).
    signal s_y_combed        : unsigned(9 downto 0);

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
    signal s_y_rot           : unsigned(9 downto 0);
    signal s_u_rot           : unsigned(9 downto 0);
    signal s_v_rot           : unsigned(9 downto 0);

    -- ========================================================================
    -- "Wet" output — at M7 this is the Stage 3 sin/cos rotation output.
    -- Subsequent milestones layer Stages 4-5 on top.  Master mix
    -- interpolator arrives at M9; until then output is direct s_*_rot.
    -- ========================================================================
    signal s_wet_y           : unsigned(9 downto 0);
    signal s_wet_u           : unsigned(9 downto 0);
    signal s_wet_v           : unsigned(9 downto 0);

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
    p_in_top_half : process(clk)
    begin
        if rising_edge(clk) then
            if s_vcount < to_unsigned(C_ACTIVE_HALF_HEIGHT, s_vcount'length) then
                s_in_top_half <= '1';
            else
                s_in_top_half <= '0';
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
                -- Latch the per-half means by shifting accumulator down to
                -- 10-bit data range.  C_AVG_SHIFT = 20 → ÷2^20 ≈ ÷(half-frame
                -- pixel count).  Imprecise but bounded; used as a CV, not
                -- a computed mean.
                v_luma_top_mean := shift_right(s_luma_acc_top, C_AVG_SHIFT);
                v_luma_bot_mean := shift_right(s_luma_acc_bot, C_AVG_SHIFT);
                v_edge_top_mean := shift_right(s_edge_acc_top, C_AVG_SHIFT);
                v_edge_bot_mean := shift_right(s_edge_acc_bot, C_AVG_SHIFT);

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
    -- logic.  Companion-pipes data_in to track BRAM read pipeline timing
    -- for motion proxy (s_y_d2 vs s_y_prev at S2).
    -- ========================================================================
    p_input_reg : process(clk)
    begin
        if rising_edge(clk) then
            s_y_d0    <= data_in.y;
            s_u_d0    <= data_in.u;
            s_v_d0    <= data_in.v;
            s_avid_d0 <= data_in.avid;
            s_y_d1    <= s_y_d0;        -- companion pipe for motion proxy
            s_y_d2    <= s_y_d1;
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

    -- TBC jitter compute (combinational at S0).  Combines four modulation
    -- sources, all scaled by the dithered Knob 5 shift amount.  Result is
    -- clamped to non-negative 9-bit so the BRAM read at wr_addr - C_TBC_BASE
    -- - s_tbc_offset never reads "into the future".
    --
    -- Sources:
    --   * Content luma deviation: s_y_d0 - 512  (signed 11-bit, [-512, 511])
    --   * Per-half envelope CV:   cv_luma_half  (10-bit, scaled << 1)
    --   * Motion proxy (1-clock lag): s_motion_proxy << 2
    --   * Frame-phase drift:      s_phase_drift(15..7)
    p_tbc_jitter : process(clk)
        variable v_cv_half  : unsigned(9 downto 0);
        variable v_content  : signed(11 downto 0);
        variable v_envelope : signed(11 downto 0);
        variable v_motion   : signed(11 downto 0);
        variable v_drift    : signed(11 downto 0);
        variable v_sum      : signed(13 downto 0);
        variable v_shifted  : signed(13 downto 0);
        variable v_abs      : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            -- Pick the spatial-half CV.
            if s_in_top_half = '1' then
                v_cv_half := s_cv_luma_top;
            else
                v_cv_half := s_cv_luma_bot;
            end if;

            v_content  := signed(resize(unsigned(s_y_d0), 12)) - to_signed(512, 12);
            v_envelope := signed(resize(v_cv_half, 12)) sll 1;
            v_motion   := resize(s_motion_proxy, 12) sll 2;
            -- s_phase_drift(15..7) is 9-bit unsigned; resize to 12-bit
            -- unsigned (zero-extend), then cast to signed.  Avoids the
            -- "0" & slv overload ambiguity flagged in testing-guide.md.
            v_drift    := signed(resize(s_phase_drift(15 downto 7), 12));

            v_sum     := resize(v_content,  14) + resize(v_envelope, 14)
                       + resize(v_motion,   14) + resize(v_drift,    14);
            v_shifted := shift_right(v_sum, s_jitter_shift_eff);

            -- Take absolute value, clamp to 9 bits.
            if v_shifted(13) = '1' then
                v_abs := unsigned(-v_shifted);
            else
                v_abs := unsigned(v_shifted);
            end if;
            if v_abs > to_unsigned(511, 14) then
                s_tbc_offset <= to_unsigned(511, 9);
            else
                s_tbc_offset <= v_abs(8 downto 0);
            end if;
        end if;
    end process;

    -- ========================================================================
    -- S0/S1: BRAM read addresses (combinational from s_wr_addr + jitter).
    -- All four address arithmetic done in 13-bit signed before truncating
    -- to the 11-bit BRAM addr (R6 — gravity_bleed pattern).
    -- ========================================================================
    p_rd_addrs : process(s_wr_addr, s_tbc_offset)
        variable v_tbc_offset : unsigned(C_BUF_DEPTH - 1 downto 0);
    begin
        v_tbc_offset := to_unsigned(C_TBC_BASE, C_BUF_DEPTH)
                        + resize(s_tbc_offset, C_BUF_DEPTH);
        s_rd_addr_tbc_y  <= s_wr_addr - v_tbc_offset;
        s_rd_addr_tbc_u  <= s_wr_addr - v_tbc_offset;
        s_rd_addr_tbc_v  <= s_wr_addr - v_tbc_offset;
        s_rd_addr_prev_y <= s_wr_addr - to_unsigned(C_PREV_OFFSET, C_BUF_DEPTH);
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
    -- S2: Motion proxy register — signed(s_y_d2) - signed(s_y_prev).
    -- Updated each clock; consumed by p_tbc_jitter at S0 of the NEXT
    -- clock (1-pixel pipeline lag).  s_y_prev arrives at S2 from BRAM,
    -- s_y_d2 is data_in companion-piped to match.
    -- ========================================================================
    p_motion_proxy : process(clk)
        variable v_y_now  : signed(10 downto 0);
        variable v_y_prev : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            v_y_now  := signed(resize(unsigned(s_y_d2), 11));
            v_y_prev := signed(resize(s_y_prev,        11));
            s_motion_proxy <= v_y_now - v_y_prev;
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
        variable v_y_now    : signed(11 downto 0);
        variable v_y_chain  : signed(11 downto 0);
        variable v_y_prev   : signed(11 downto 0);
        variable v_delta_h  : signed(11 downto 0);
        variable v_delta_v  : signed(11 downto 0);
        variable v_delta    : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            v_y_now   := signed(resize(s_y_diode,                     12));
            v_y_chain := signed(resize(unsigned(s_y_chain_d4),        12));
            v_y_prev  := signed(resize(s_y_prev_d4,                   12));

            v_delta_h := v_y_now - v_y_chain;
            v_delta_v := v_y_now - v_y_prev;
            v_delta   := v_delta_h + v_delta_v;
            -- Scale by Knob 4 shift; signed arithmetic shift_right is OK
            -- here (single-pass, no multi-frame feedback — pitfall #3
            -- doesn't apply).
            s_comb_delta <= shift_right(v_delta, s_comb_shift_eff);
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
    -- Angle = (s_y_combed + (cv_global << 2) + Knob 3) mod 1024.
    -- Bin = angle(9 downto 6).  Sigma-delta dither between adjacent bins
    -- from angle(5 downto 0).
    -- ========================================================================
    p_rot_angle : process(s_y_combed, s_cv_global, s_chroma_twist)
        variable v_angle : unsigned(11 downto 0);
    begin
        v_angle := resize(s_y_combed, 12)
                 + resize(shift_left(s_cv_global, 2), 12)
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
    -- M7 wet path: Stage 3 sin/cos rotation output.  Subsequent milestones
    -- layer Stages 4-5 on top.  Master mix interpolator arrives at M9.
    -- ========================================================================
    s_wet_y <= s_y_rot;
    s_wet_u <= s_u_rot;
    s_wet_v <= s_v_rot;

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
    -- Sync delay: C_PROCESSING_DELAY_CLKS = 3 at M5.
    -- Carries only sync flags — the dry-path data taps will be added back
    -- at M9 (master-mix interpolator's "a" input).  At M5 the wet path
    -- comes from Stage 1 BRAM (s_y_tbc/s_u_tbc/s_v_tbc), which is itself
    -- 3 cycles deep — so syncs and data align if both are 3-cycle delayed.
    -- All sync variables initialised to inactive defaults (idiom #4).
    -- ========================================================================
    p_sync_delay : process(clk)
        type t_sync_delay is array(0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic;
        variable v_avid    : t_sync_delay := (others => '0');
        variable v_hsync   : t_sync_delay := (others => '1');
        variable v_vsync   : t_sync_delay := (others => '1');
        variable v_field   : t_sync_delay := (others => '1');
    begin
        if rising_edge(clk) then
            v_avid    := data_in.avid    & v_avid(0    to C_PROCESSING_DELAY_CLKS - 2);
            v_hsync   := data_in.hsync_n & v_hsync(0   to C_PROCESSING_DELAY_CLKS - 2);
            v_vsync   := data_in.vsync_n & v_vsync(0   to C_PROCESSING_DELAY_CLKS - 2);
            v_field   := data_in.field_n & v_field(0   to C_PROCESSING_DELAY_CLKS - 2);

            -- Sync outputs at full pipeline depth.
            data_out.avid    <= v_avid(C_PROCESSING_DELAY_CLKS - 1);
            data_out.hsync_n <= v_hsync(C_PROCESSING_DELAY_CLKS - 1);
            data_out.vsync_n <= v_vsync(C_PROCESSING_DELAY_CLKS - 1);
            data_out.field_n <= v_field(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process;

end architecture sync_bleed;
