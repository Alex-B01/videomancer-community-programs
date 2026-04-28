-- Videomancer Community Program by Sourdough Stu
-- Copyright (C) 2026 AB (Sourdough_Stu)
-- File: sync_bleed.vhd - Sync Bleed Program for Videomancer
-- License: GNU General Public License v3.0
-- https://github.com/lzxindustries/videomancer-community-programs
--
-- This file is free software: you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation, either version 3 of the License, or
-- (at your option) any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
-- GNU General Public License for more details.
--
-- You should have received a copy of the GNU General Public License
-- along with this program. If not, see <https://www.gnu.org/licenses/>.
--
-- Program Name:
--   Sync Bleed
--
-- Author:
--   AB (Sourdough_Stu)
--
-- Overview:
--   Analog dirty mixer / broken TBC simulator. A chain of degraded
--   analog video processors: capacitor-bleed smear, diode wavefolder,
--   NTSC chroma comb, sin/cos hue rotation, TBC line jitter, and
--   tape noise (Y-domain). Driven by an envelope follower that
--   accumulates luma + edge magnitude across the active frame; per-
--   half accumulators (top/bottom) are max'd into a single content
--   CV that modulates each stage's character. Knob 6 (Envelope
--   Reactivity) amplifies the content coupling: at zero the program
--   runs as a static effect chain; cranked, the controls become
--   content-modulated.
--
--   Resolution-portable (1080p / 720p / 480p) via runtime-counted
--   envelope divisor, spatial-half boundary, and prev-line BRAM
--   offset.
--
--   Resource usage: 7657 / 7680 LCs (99%), 29 / 32 EBR (90%) —
--   tight LC budget; BRAM holds line buffers for TBC + 8 diode LUT
--   banks (4 Y + 4 UV).
--
-- Pipeline (19 clocks total):
--   S0    Input register, hcount/vcount, LFSR, edge magnitude,
--         BRAM write address, Y shift-register chain
--   S1    TBC jitter offset compute (per-line, hsync-latched)
--   S2    BRAM read (Y dual-port, U and V single-port each)
--   S3    BRAM output register + sanitiser
--   S4    Diode LUT bank reads (8 ROMs in parallel)
--   S5    Diode LUT bank-pair cross-fade (Y, U, V)
--   S6    NTSC comb compute (delta_h + delta_v + black-attenuation)
--   S7    NTSC UV inject (opposite-signed) + register
--   S8a   Sin/cos rotation: shift-add products for both bins
--   S8b   Sin/cos rotation: cross-fade blend + clamp + UV swap
--   S9    IIR state register (avid-gated, hsync-reset, Bleed Freeze)
--   S10   Smear register (IIR + AC/DC mirror + edge ringing add)
--   S11   Tape noise add (Y-domain only, midtone-correlated)
--   S12   Pass-through register
--   S13-S16  interpolator_u x 3 (master mix, 4-cycle latency)
--   S17   Output register + broadcast-safe clamp
--
-- Parameters:
--   Knob 1     (Reg 0)    Cap Bleed       IIR smear decay rate
--   Knob 2     (Reg 1)    Edge Resonance  Edge-ringing amplitude
--   Knob 3     (Reg 2)    Chroma Twist    Sin/cos rotation angle
--   Knob 4     (Reg 3)    NTSC Comb       Comb delta magnitude
--   Knob 5     (Reg 4)    TBC Jitter      Per-line offset range
--   Knob 6     (Reg 5)    Envelope React  Content-coupling strength
--   Toggle 7   (Reg 6.0)  AC / DC         IIR mirror polarity
--   Toggle 8   (Reg 6.1)  Edge Phase      Ringing sign invert
--   Toggle 9   (Reg 6.2)  UV Swap         Chroma U/V exchange
--   Toggle 10  (Reg 6.3)  Diode Base      Bank range: Soft (0..2),
--                                         Hard (1..3)
--   Toggle 11  (Reg 6.4)  Bleed Freeze    IIR line-reset gate +
--                                         slow-decay (persistent smear)
--   Slider 12  (Reg 7)    Master Mix     Dry / wet blend
--
-- ========================================================================

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
    -- Total pipeline depth (data_in to data_out): 19 cycles.  See the
    -- Pipeline section in the file header for the per-stage breakdown.
    -- When changing pipeline depth, also audit C_DRY_TAP_DEPTH,
    -- C_IIR_GATE_DEPTH, and C_EDGE_PIPE_LEN.
    constant C_PROCESSING_DELAY_CLKS : integer := 19;
    -- Dry-path tap depth into the master mix interpolator (a-input).
    -- Interpolator is 4 cycles, so dry tap = 19 − 4 = 15.
    constant C_DRY_TAP_DEPTH         : integer := 15;
    -- IIR-gate depth — avid and hsync_start gating signals for p_iir_state
    -- align with the IIR's data input (s_y_rot/u_rot/v_rot) at depth 11.
    -- Gating on shallower-depth signals would let blanking-zero data poison
    -- the IIR state at line start (UV would collapse toward 0 = green).
    constant C_IIR_GATE_DEPTH        : integer := 11;

    -- BRAM line buffer geometry — 2048 deep covers 1080p active line (1920) + headroom.
    constant C_BUF_SIZE      : integer := 2048;
    constant C_BUF_DEPTH     : integer := 11;
    -- Baseline TBC read offset.  Set to 1 (minimum value that clears the
    -- iCE40 inferred-BRAM read-before-write conflict zone).  Wet path
    -- reads already-written data; costs a 1-pixel structural shift
    -- between dry and wet at all-knobs-zero (imperceptible visually).
    -- Knob 5 (TBC Jitter) adds 0..16 px of dynamic offset on top.
    constant C_TBC_BASE      : integer := 1;

    -- Active video frame dimensions.  Spatial-half boundary and the
    -- envelope-accumulator divisor are derived at runtime from the
    -- live video timing (see s_active_half_height_dyn and
    -- s_avg_shift_dyn below) so the program is resolution-portable
    -- across all supported video modes.

    -- Envelope accumulator width.  Worst-case half-frame at 1080p is
    -- 1920 × 540 × 1023 ≈ 1.06e9 < 2^30, so 32 bits gives comfortable
    -- headroom for any supported mode.
    constant C_ENV_ACC_WIDTH      : integer := 32;

    -- Centred-unsigned chroma neutral (UV resting state = mid-grey, not 0).
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
    -- Toggle 11 (registers_in(6)(4)) — Bleed Freeze.  When active,
    -- p_iir_state skips its per-line reset so state persists across
    -- hsync boundaries and continues to evolve from one line to the
    -- next.  Pixel-level IIR updates keep running, so smear character
    -- accumulates across the frame instead of resetting every line —
    -- produces persistent trailing smear / ghost-image character.
    signal s_bleed_freeze    : std_logic;   -- Toggle 11 bit 4

    -- ========================================================================
    -- LFSR — shared chaos source (tape noise + jitter trigger).
    -- Lives in S0 advance-every-clock domain; consumers tap different bits.
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
    -- The init also keeps cv_global from sourcing 'U's into downstream
    -- threshold checks before the envelope accumulator has accumulated
    -- a frame's worth of data.
    signal s_cv_global       : unsigned(9 downto 0) := (others => '0');

    -- Effective cv for bank/blend modulation.
    -- s_cv_eff = clamp(s_cv_global + (s_envelope_react >> 2), 1023).
    -- Knob 6 contribution: 0..255 added to cv → max ~1 bank step
    -- shift via env_react alone.  Used by p_diode_bank (hysteresis
    -- comparison) and p_diode_mux (cross-fade weight) to give Knob 6
    -- a perceivable push through the bank ladder.  s_cv_global itself
    -- stays content-driven for other consumers (noise, comb, edge).
    signal s_cv_eff          : unsigned(9 downto 0) := (others => '0');


    -- Phase-speed: (cv_global + envelope_react) >> 4 — used by p_jitter_offset
    -- as the envelope-modulated jitter boost source.
    signal s_phase_speed     : unsigned(9 downto 0);

    -- Pre-registered spatial-half CV selection.  Breaks the critical path
    -- s_in_top_half → cv_luma_top/bot mux → 14-bit signed adds inside
    -- p_tbc_jitter.  Aesthetically invisible: cv_luma_top/bot only
    -- update at vsync, so a 1-cycle register on the spatial-half mux
    -- doesn't lag anything visible.
    signal s_cv_luma_half_d  : unsigned(9 downto 0) := (others => '0');

    -- Resolution-aware divisor for the envelope accumulator.  Counts
    -- active pixels in the top spatial half each frame; the priority-
    -- encoded MSB position + 1 gives ceil(log2(half-frame-pixels)) =
    -- the correct shift amount for any video mode + decimation.
    -- Defaults to 20 (1080p hardware) so the first frame works before
    -- the count latches.
    signal s_pixel_count_top : unsigned(23 downto 0) := (others => '0');
    signal s_avg_shift_dyn   : integer range 4 to 24 := 20;

    -- Resolution-aware spatial-half boundary.  Latched from s_vcount at
    -- each vsync_start = total active row count of the just-completed
    -- frame; halved for the s_in_top_half comparison.  Default 540 so
    -- the first frame after reset uses 1080p calibration; settles to
    -- the correct value after the first complete frame.
    signal s_active_half_height_dyn : unsigned(10 downto 0)
                                      := to_unsigned(540, 11);

    -- Resolution-aware prev-line tap offset for the BRAM ring buffer.
    -- Latched from a per-line active-pixel counter at hsync_start =
    -- active pixel count of the just-completed line.  The NTSC comb's
    -- vertical delta tap (s_y_prev) reads at wr_addr - this value, so
    -- it lands on the same column from the previous active line at
    -- any video mode.  Default 1920 = 1080p line length.
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
    -- U and V banks 1-port (TBC only, 5 EBR each).  No init values:
    -- iCE40 BRAM inference requires uninitialised arrays (init values
    -- are required for ROM, prohibited for RAM).
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

    -- Sanitised Stage 1 outputs at S2 (registered + to_01 GHDL guard).
    signal s_y_tbc           : unsigned(9 downto 0);
    signal s_y_prev          : unsigned(9 downto 0);
    signal s_u_tbc           : unsigned(9 downto 0);
    signal s_v_tbc           : unsigned(9 downto 0);

    -- Computed TBC offset — REGISTERED at end of S0 to break the
    -- combinational chain s_in_top_half → cv mux → adds/shifts → rd_addr
    -- → BRAM read DFF.  Without registering, this chain was the critical
    -- path capping Fmax below the 74.25 MHz target.  Aesthetically
    -- invisible: the offset is 1 active-pixel "stale" relative to the
    -- wr_addr but TBC reads are bounded by the offset anyway.
    -- Distinct from s_tbc_jitter (the raw Knob 5 register value).
    signal s_tbc_offset      : unsigned(8 downto 0) := (others => '0');

    -- Sigma-delta dither for Knob 5 lower 7 bits.  Smooths
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
    -- Cross-fade blend (read banks N and N+1, blend by frame-stable
    -- weight) avoids the spatial mottling a per-pixel sigma-delta pick
    -- would produce — especially harsh on Hard Clip (Bank 3 = Violent
    -- Fold inverts black/white).  Cross-fade blend gives the same
    -- average colour with no toggle texture.
    signal s_diode_bank_n_d3   : integer range 0 to 3 := 0;
    signal s_diode_bank_n1_d3  : integer range 0 to 3 := 0;
    -- Held bank base (post-hysteresis).  Prevents per-frame flicker when
    -- cv_global hovers near the 256/512/768 bank-boundary cliffs on
    -- bright live content.  Updated only when cv crosses a boundary by
    -- C_BANK_HYST margin, otherwise holds.
    signal s_bank_base_held    : integer range 0 to 3 := 0;
    -- Vsync-latched dither rate.  Holds cv_global lower 4 bits stable
    -- Diode LUT outputs at S4 (post-mux + register) — fed into Stage 3.
    -- UV inits to C_CHROMA_MID so transient sim 'U's don't propagate
    -- into rotation/IIR as green.
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

    -- UV-injected output of Stage 3a (S6).  Inits to C_CHROMA_MID so
    -- transient sim 'U's don't propagate as green.
    signal s_u_combed        : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_v_combed        : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- Companion-pipe Y through Stage 3 (no comb processing; Y is just
    -- delayed to keep alignment with the rotated UV).
    signal s_y_combed        : unsigned(9 downto 0) := (others => '0');

    -- Sin/cos hue twist (S7).  Angle composition + bin lookup combinational
    -- from s_y_combed at S6, registered at S7.  Rotation applied to UV
    -- centred around C_CHROMA_MID = 512.
    signal s_rot_angle       : unsigned(9 downto 0);
    -- Cross-fade rotation (pipelined): read TWO consecutive bins
    -- (bin_base, bin_base+1 mod 16) and blend their rotation results
    -- based on angle(5:3) (3-bit weight, 8 levels per bin).  Eliminates
    -- the per-frame "violent snap" when cv shifts the angle base across
    -- a bin boundary.  S7b is split into s7b1 (rotations + deltas) and
    -- s7b2 (blend cascade + clamp) to meet timing.
    signal s_rot_bin_n       : t_rot_bin;       -- bin_base
    signal s_rot_bin_n1      : t_rot_bin;       -- (bin_base + 1) mod 16
    signal s_rot_bin_base    : integer range 0 to 15 := 0;
    signal s_rot_bin_next    : integer range 0 to 15 := 0;
    signal s_rot_weight_d7a  : unsigned(2 downto 0) := (others => '0');

    -- Intermediate rotation products at S7a (after the first shift-add stage).
    -- These hold cos*u, sin*u, cos*v, sin*v separately so the second stage
    -- only does the final sum + clamp.  Splitting the rotation into two
    -- pipeline stages buys ~13 ns of timing budget (vs all-in-one which
    -- was failing at ~25 ns critical path).
    -- Two parallel paths: _n suffix = bin_base, _n1 suffix = bin_base+1.
    signal s_cos_u_n_d7a     : signed(13 downto 0) := (others => '0');
    signal s_sin_u_n_d7a     : signed(13 downto 0) := (others => '0');
    signal s_cos_v_n_d7a     : signed(13 downto 0) := (others => '0');
    signal s_sin_v_n_d7a     : signed(13 downto 0) := (others => '0');
    signal s_cos_u_n1_d7a    : signed(13 downto 0) := (others => '0');
    signal s_sin_u_n1_d7a    : signed(13 downto 0) := (others => '0');
    signal s_cos_v_n1_d7a    : signed(13 downto 0) := (others => '0');
    signal s_sin_v_n1_d7a    : signed(13 downto 0) := (others => '0');

    -- S7b1 intermediate registers.  Hold the two-bin rotation results
    -- and their deltas, ready for the blend cascade in s7b2.  Width
    -- is 14-bit signed: rotation outputs reach ~±5800 in extreme cases,
    -- deltas can reach ~±11600 — the delta saturates the 14-bit range,
    -- so it is widened to 15-bit signed to avoid wrap distortion at
    -- extreme rotation transitions.
    signal s_u_rot_n_d7b1    : signed(13 downto 0) := (others => '0');
    signal s_v_rot_n_d7b1    : signed(13 downto 0) := (others => '0');
    signal s_u_delta_d7b1    : signed(14 downto 0) := (others => '0');
    signal s_v_delta_d7b1    : signed(14 downto 0) := (others => '0');
    signal s_rot_weight_d7b1 : unsigned(2 downto 0) := (others => '0');
    signal s_uv_swap_d7b1    : std_logic := '0';
    signal s_y_combed_d7b1   : unsigned(9 downto 0) := (others => '0');
    -- Y companion-pipe through S7a so S7b output keeps Y aligned.
    signal s_y_combed_d7a    : unsigned(9 downto 0) := (others => '0');
    signal s_uv_swap_d7a     : std_logic := '0';

    -- Stage 3 output (post-rotation + UV swap), at S7b timing.
    -- Init values to safe defaults so GHDL doesn't propagate 'U' into
    -- the downstream IIR state at simulation start.  Y → 0, UV →
    -- C_CHROMA_MID (neutral grey is safe; 0 would render as green).
    signal s_y_rot           : unsigned(9 downto 0) := (others => '0');
    signal s_u_rot           : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_v_rot           : unsigned(9 downto 0) := C_CHROMA_MID;

    -- ========================================================================
    -- Stage 4 (M8): Capacitor bleed IIR + edge ringing.
    -- ========================================================================

    -- 19-bit signed IIR state per channel (1 sign + 10 data + 8 fractional).
    -- Fractional bits give smooth slow IIR without per-pixel staircase
    -- quantisation.  Resets at hsync_falling: Y → 0, UV → 512<<8.
    -- Update is avid-gated so state does not advance during blanking.
    constant C_IIR_FRAC_BITS    : integer := 8;
    constant C_IIR_STATE_WIDTH  : integer := 19;
    -- 512 << 8 = 131072, the UV neutral state (UV resting value must
    -- be C_CHROMA_MID, never 0).
    constant C_IIR_UV_RESET     : integer := 512 * 256;

    signal s_iir_y           : signed(C_IIR_STATE_WIDTH - 1 downto 0)
                              := (others => '0');
    signal s_iir_u           : signed(C_IIR_STATE_WIDTH - 1 downto 0)
                              := to_signed(C_IIR_UV_RESET, C_IIR_STATE_WIDTH);
    signal s_iir_v           : signed(C_IIR_STATE_WIDTH - 1 downto 0)
                              := to_signed(C_IIR_UV_RESET, C_IIR_STATE_WIDTH);

    -- IIR k effective.  k_y_base from Knob 1 upper-3 (clamped 1..7);
    -- per-half envelope CV reduces k (faster decay = lighter smear in
    -- bright/active halves).  Sigma-delta dither on Knob 1 lower 7 bits.
    -- k_uv_eff = k_y_eff + C_K_UV_OFFSET (UV trails slightly longer
    -- than Y, emphasising chromatic ghosting).
    constant C_K_UV_OFFSET   : integer := 1;
    signal s_k_y_dither_acc  : unsigned(6 downto 0) := (others => '0');
    signal s_k_y_eff         : integer range 1 to 7 := 1;
    signal s_k_uv_eff        : integer range 1 to 7 := 1;

    -- IIR output (S9 → S10 timing) — combinational from state's upper 10
    -- bits.  Drives the smear register at S10.
    signal s_iir_y_out       : unsigned(9 downto 0);
    signal s_iir_u_out       : unsigned(9 downto 0);
    signal s_iir_v_out       : unsigned(9 downto 0);

    -- Edge magnitude companion-pipe — s_edge_d0 (registered at S0 =
    -- depth 1 from data_in) is shifted through to align with the smear
    -- stage that consumes it.  C_EDGE_PIPE_LEN must match the pipeline
    -- depth between edge compute and smear consumption — if it lags,
    -- phantom edge rings appear on bright transitions ("rapid brightness
    -- shift to white" flicker on contrasty content).  v_edge_idx = C-1
    -- → with C=11 the 5-tap window spans indices {10,9,8,7,6} = depths
    -- {12,11,10,9,8}.
    constant C_EDGE_PIPE_LEN : integer := 11;
    type t_edge_pipe is array (0 to C_EDGE_PIPE_LEN - 1) of unsigned(9 downto 0);
    signal s_edge_pipe       : t_edge_pipe := (others => (others => '0'));

    -- Edge ringing scale.  Knob 2 upper-3 → shift count, lower-7 → dither.
    signal s_ring_dither_acc : unsigned(6 downto 0) := (others => '0');
    signal s_ring_shift_eff  : integer range 0 to 15 := 15;

    -- Stage 4 output (smear) at S10 timing — drives the wet path.
    signal s_smear_y         : unsigned(9 downto 0) := (others => '0');
    signal s_smear_u         : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_smear_v         : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Companion-pipe of s_y_rot (= IIR's input at S9) through to S10
    -- so the AC/DC mode at smear register can compute the mirror
    -- output = 2*input − state.  Two register stages: S9 and S10.
    signal s_y_rot_d9        : unsigned(9 downto 0) := (others => '0');
    signal s_y_rot_d10       : unsigned(9 downto 0) := (others => '0');

    -- ========================================================================
    -- Stage 5: Tape noise + master mix.  Knob 5 jitter (small per-line
    -- LFSR offset) is on a separate path.
    -- ========================================================================

    -- Tape noise gain shift — driven by cv_global (envelope drives noise
    -- amplitude).  cv at 0 → shift 15 (silent); cv at max → shift 5
    -- (loud noise), with Knob 6 (Envelope Reactivity) scaling the
    -- coupling strength.
    signal s_noise_amp_shift : integer range 0 to 15 := 15;

    -- Hysteresis margin (in cv_global LSBs) for the diode-bank base
    -- selection.  cv crossing a 256-cell boundary by less than this
    -- amount does NOT change the held bank — prevents bright live
    -- content with cv hovering at a bank boundary from flickering
    -- between adjacent banks per frame.  Wide margin keeps each bank's
    -- curve recognisable before the cross-fade ramp kicks in.
    constant C_BANK_HYST       : integer := 128;

    -- CV attack-time shift.  Without this, cv_global jumps frame-to-
    -- frame (instant peak attack from the leak/peak logic) and most
    -- env-driven controls inherit that flicker.  This shift converts
    -- the peak attack into a slow IIR ramp:
    --   new_cv = old_cv + (latched_mean - old_cv) >> C_CV_ATTACK_SHIFT
    -- C_CV_ATTACK_SHIFT = 3 → 8-frame attack ≈ 267 ms @ 30 fps,
    --                        ≈ 133 ms @ 60 fps.
    -- Tunable: higher values = smoother but laggier.  Decay (= leak by
    -- shift_right 8 in p_envelope_acc) is unchanged.
    constant C_CV_ATTACK_SHIFT : integer := 3;

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
    -- Depth-10 Y tap: drives content-driven IIR k_eff modulation per
    -- pixel.  Aligned with s_y_rot / IIR data timing.
    signal s_y_d10             : std_logic_vector(9 downto 0) := (others => '0');
    signal s_data_in_d14_u     : std_logic_vector(9 downto 0) := (others => '0');
    signal s_data_in_d14_v     : std_logic_vector(9 downto 0) := (others => '0');
    -- avid at the interpolator's input — used as enable for the
    -- interpolator instances (4-cycle sync between dry and wet).
    signal s_avid_d14          : std_logic := '0';

    -- IIR-gate signals at depth 10 (matches s_y_rot/u_rot/v_rot data
    -- timing).  Gating the IIR at a shallower depth than its data
    -- input would cause blanking-zero contamination of UV state at
    -- line start.
    signal s_avid_d10          : std_logic := '0';
    signal s_hsync_start_d10   : std_logic := '0';

    -- Master mix interpolator outputs at depth 18.
    signal s_interp_y          : unsigned(9 downto 0);
    signal s_interp_u          : unsigned(9 downto 0);
    signal s_interp_v          : unsigned(9 downto 0);

    -- "Wet" output — master mix output post-interpolator.
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
    s_bleed_freeze    <= registers_in(6)(4);

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
    -- Module: lfsr16 — shared chaos source for tape noise + jitter.
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
    -- Consumed by the envelope edge-energy accumulator and by the
    -- edge-ringing add.
    -- ========================================================================
    p_edge_mag : process(clk)
        variable v_y_now    : unsigned(9 downto 0);
        variable v_raw_mag  : unsigned(9 downto 0);
        variable v_clipped  : unsigned(9 downto 0);
        variable v_smoothed : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_y_now := unsigned(data_in.y);
            s_y_d_prev_pix <= v_y_now;
            if v_y_now > s_y_d_prev_pix then
                v_raw_mag := v_y_now - s_y_d_prev_pix;
            else
                v_raw_mag := s_y_d_prev_pix - v_y_now;
            end if;
            -- Two-step despeckle: clamp per-pixel edge magnitude at 256
            -- (25% of 10-bit range) to clip noise spikes while preserving
            -- real gradient edges, then run a 1-pole IIR low-pass to
            -- smooth what remains.
            if v_raw_mag > to_unsigned(255, 10) then
                v_clipped := to_unsigned(255, 10);
            else
                v_clipped := v_raw_mag;
            end if;
            -- 1-pole IIR: new = old + (clipped - old) >> 1
            -- Equivalent to half-weight running average.  Smooths
            -- single-pixel spikes that remain post-clamp.
            if v_clipped > s_edge_d0 then
                v_smoothed := s_edge_d0 + shift_right(v_clipped - s_edge_d0, 1);
            else
                v_smoothed := s_edge_d0 - shift_right(s_edge_d0 - v_clipped, 1);
            end if;
            s_edge_d0 <= v_smoothed;
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
    -- envelope accumulators would otherwise be the critical path.
    -- Registering breaks the chain at no aesthetic cost since
    -- s_in_top_half changes only at hsync boundaries (line transitions).
    -- Compares against the runtime-derived half height.
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
    -- is the runtime-derived equivalent of a static `÷ half-frame-pixels`,
    -- making the envelope accumulator portable across video modes.
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
        variable v_attack_sum    : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                -- Latch the per-half means by shifting accumulator down
                -- to 10-bit data range.  s_avg_shift_dyn = log2(half-frame
                -- pixel count), derived at runtime per video mode (see
                -- p_avg_shift_runtime).  Imprecise but bounded; used as a
                -- CV, not a precise mean.
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

                -- Slow attack (was instant peak-hold).  When latched mean
                -- exceeds leaked old, IIR-converge toward it at
                -- 1/2^C_CV_ATTACK_SHIFT per frame instead of jumping.
                -- Eliminates per-frame cv flicker that drove visible
                -- strobing in many env-modulated stages (rotation
                -- angle, comb shift, density mask, dither rate, etc.).
                -- Slow decay branch unchanged — still leak-only.
                --
                -- Compute in 11-bit unsigned then clamp to 10-bit max
                -- (1023).  Without the clamp, v_leak (up to ~1023) plus
                -- shifted_diff (up to ~127) can sum to >1023, wrapping
                -- the 10-bit cv signal to a tiny value and triggering
                -- threshold-gated misbehaviour downstream.
                if v_lat_y_top > v_leak_y_top then
                    v_attack_sum := resize(v_leak_y_top, 11)
                                  + resize(shift_right(v_lat_y_top - v_leak_y_top, C_CV_ATTACK_SHIFT), 11);
                    if v_attack_sum > to_unsigned(1023, 11) then
                        s_cv_luma_top <= to_unsigned(1023, 10);
                    else
                        s_cv_luma_top <= v_attack_sum(9 downto 0);
                    end if;
                else
                    s_cv_luma_top <= v_leak_y_top;
                end if;
                if v_lat_y_bot > v_leak_y_bot then
                    v_attack_sum := resize(v_leak_y_bot, 11)
                                  + resize(shift_right(v_lat_y_bot - v_leak_y_bot, C_CV_ATTACK_SHIFT), 11);
                    if v_attack_sum > to_unsigned(1023, 11) then
                        s_cv_luma_bot <= to_unsigned(1023, 10);
                    else
                        s_cv_luma_bot <= v_attack_sum(9 downto 0);
                    end if;
                else
                    s_cv_luma_bot <= v_leak_y_bot;
                end if;
                if v_lat_e_top > v_leak_e_top then
                    v_attack_sum := resize(v_leak_e_top, 11)
                                  + resize(shift_right(v_lat_e_top - v_leak_e_top, C_CV_ATTACK_SHIFT), 11);
                    if v_attack_sum > to_unsigned(1023, 11) then
                        s_cv_edge_top <= to_unsigned(1023, 10);
                    else
                        s_cv_edge_top <= v_attack_sum(9 downto 0);
                    end if;
                else
                    s_cv_edge_top <= v_leak_e_top;
                end if;
                if v_lat_e_bot > v_leak_e_bot then
                    v_attack_sum := resize(v_leak_e_bot, 11)
                                  + resize(shift_right(v_lat_e_bot - v_leak_e_bot, C_CV_ATTACK_SHIFT), 11);
                    if v_attack_sum > to_unsigned(1023, 11) then
                        s_cv_edge_bot <= to_unsigned(1023, 10);
                    else
                        s_cv_edge_bot <= v_attack_sum(9 downto 0);
                    end if;
                else
                    s_cv_edge_bot <= v_leak_e_bot;
                end if;

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
    -- s_cv_eff = cv_global + (env_react >> 2), clamped to 1023.
    -- Drives bank hysteresis and cross-fade weight (p_diode_bank,
    -- p_diode_mux).  Lets Knob 6 push the bank ladder up by ~1 step
    -- at full env_react, independent of content cv.
    -- ========================================================================
    p_cv_eff : process(clk)
        variable v_sum : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            v_sum := resize(s_cv_global, 11)
                   + resize(shift_right(s_envelope_react, 2), 11);
            -- Cheap saturate: bit 10 of 11-bit sum = overflow flag.
            if v_sum(10) = '1' then
                s_cv_eff <= (others => '1');  -- clamp to 1023
            else
                s_cv_eff <= v_sum(9 downto 0);
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Phase speed = (cv_global + envelope_react) >> 4.  Used by
    -- p_jitter_offset as the envelope-modulated jitter boost source.
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
    -- S0: Sigma-delta dither for Knob 5 lower 7 bits.
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
        variable v_random  : unsigned(4 downto 0);  -- 0..31 from LFSR (was 0..15)
        variable v_knob    : unsigned(4 downto 0);  -- 0..31 from Knob 5
        variable v_env     : unsigned(4 downto 0);  -- 0..15 from envelope×react
        variable v_max     : unsigned(5 downto 0);  -- knob+env, clamped below
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
            -- Each line samples LFSR(4..0) (0..31) and clamps to v_max.
            -- v_max is derived from BOTH:
            --   * Knob 5 (TBC Jitter): primary intensity control,
            --     0..31 from upper 5 bits.  Tear range extends to 31 px
            --     max — safely within the smallest line buffer at
            --     decim=4 (480 px).
            --   * Envelope × Knob 6 (Env React): video content modulates
            --     jitter intensity — bright/active content → more tear.
            -- This makes the input video itself a modulation source: the
            -- TBC tear gets stronger when content is bright/edgey.
            -- s_phase_speed = (cv_global + env_react) >> 4 is the
            -- pre-computed envelope×react composite (0..127); we take
            -- its upper 4 bits (0..15) as the env contribution.  Knob 6=0
            -- masks the env contribution to 0; Knob 6 max boosts v_max
            -- by up to 15 above the knob 5 setting.
            if s_timing.hsync_start = '1' then
                v_random := unsigned(s_lfsr_q(4 downto 0));
                v_knob   := unsigned(registers_in(4)(9 downto 5));
                -- Env contribution is 3-bit (max env-jitter boost +7
                -- on top of Knob 5) so Knob 6 doesn't overpower jitter
                -- relative to its effect on edge ring/comb.
                if s_envelope_react(9 downto 6) = "0000" then
                    v_env := (others => '0');
                else
                    v_env := "00" & s_phase_speed(6 downto 4);
                end if;
                -- Sum, clamp to 31.
                v_max := resize(v_knob, 6) + resize(v_env, 6);
                if v_max > to_unsigned(31, 6) then
                    v_max := to_unsigned(31, 6);
                end if;

                -- Knob 5 (TBC Jitter) alone drives s_tbc_offset —
                -- gentle 0..31 px per-line wiggle, no big tearing.
                if unsigned(registers_in(4)) = to_unsigned(0, 10) then
                    s_tbc_offset <= (others => '0');
                elsif v_random > v_max(4 downto 0) then
                    s_tbc_offset <= "0000" & v_max(4 downto 0);
                else
                    s_tbc_offset <= "0000" & v_random;
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
    -- derived line-length offset.
    -- ========================================================================
    p_rd_addrs : process(s_wr_addr, s_tbc_offset, s_prev_offset_dyn)
        variable v_tbc_offset_y  : unsigned(C_BUF_DEPTH - 1 downto 0);
        variable v_tbc_offset_uv : unsigned(C_BUF_DEPTH - 1 downto 0);
    begin
        -- Y gets the full TBC jitter offset; UV gets HALF.
        -- Reflects analog video where luma and chroma are encoded on
        -- separate subcarriers — TBC failures shift luma noticeably
        -- while chroma follows at reduced magnitude (the recognizable
        -- "broken VCR" look: edges/peaks tear, color follows softer).
        v_tbc_offset_y  := to_unsigned(C_TBC_BASE, C_BUF_DEPTH)
                         + resize(s_tbc_offset, C_BUF_DEPTH);
        v_tbc_offset_uv := to_unsigned(C_TBC_BASE, C_BUF_DEPTH)
                         + resize(shift_right(s_tbc_offset, 1), C_BUF_DEPTH);
        s_rd_addr_tbc_y  <= s_wr_addr - v_tbc_offset_y;
        s_rd_addr_tbc_u  <= s_wr_addr - v_tbc_offset_uv;
        s_rd_addr_tbc_v  <= s_wr_addr - v_tbc_offset_uv;
        s_rd_addr_prev_y <= s_wr_addr - s_prev_offset_dyn;
    end process;

    -- ========================================================================
    -- S1: BRAM write processes — one per channel, gated by s_avid_d0.
    -- Single clocked write process per array (canonical iCE40 BRAM
    -- inference pattern; init values prohibited for RAM).
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
    -- read each (TBC only).  Each read is its own clocked process so
    -- Yosys infers them as independent SB_RAM40_4K read ports
    -- replicating the storage.
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
    -- S2: BRAM output registration + to_01 sanitiser.
    -- Breaks the BRAM-to-logic critical path and stops GHDL 'U'
    -- propagation.
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
        variable v_base_idx : integer range 0 to 3;
        variable v_next_idx : integer range 0 to 3;
        variable v_cv       : integer range 0 to 1023;
    begin
        if rising_edge(clk) then
            -- Cross-fade weight is sourced from s_cv_global(7:6) directly
            -- in p_diode_mux (frame-stable since cv_global is itself
            -- vsync-latched + slow-attacked).

            -- Base bank with HYSTERESIS to prevent flicker on bright live
            -- content where cv_global hovers near the 256/512/768 bank
            -- boundaries.  s_bank_base_held only updates when cv crosses
            -- the boundary into the new bank by at least C_BANK_HYST
            -- margin; otherwise it holds.  Uses s_cv_eff (=
            -- cv_global + env_react>>2) so
            -- Knob 6 can push the bank ladder up.
            v_cv := to_integer(s_cv_eff);
            case s_bank_base_held is
                when 0 =>
                    if v_cv >= 256 + C_BANK_HYST then
                        s_bank_base_held <= 1;
                    end if;
                when 1 =>
                    if v_cv >= 512 + C_BANK_HYST then
                        s_bank_base_held <= 2;
                    elsif v_cv < 256 - C_BANK_HYST then
                        s_bank_base_held <= 0;
                    end if;
                when 2 =>
                    if v_cv >= 768 + C_BANK_HYST then
                        s_bank_base_held <= 3;
                    elsif v_cv < 512 - C_BANK_HYST then
                        s_bank_base_held <= 1;
                    end if;
                when others =>  -- 3
                    if v_cv < 768 - C_BANK_HYST then
                        s_bank_base_held <= 2;
                    end if;
            end case;
            v_base_idx := s_bank_base_held;
            -- Toggle 10 (Diode Base) shifts the active range up by 1
            -- (Soft = 0..2, Hard = 1..3).
            if s_diode_base = '1' and v_base_idx < 3 then
                v_base_idx := v_base_idx + 1;
            end if;

            -- Cross-fade indices: bank N (current base) and bank N+1
            -- (clamped at 3).  When at bank 3, n1 = n → diff = 0 → blend
            -- = bank 3 (no overshoot beyond available banks).
            if v_base_idx < 3 then
                v_next_idx := v_base_idx + 1;
            else
                v_next_idx := 3;
            end if;
            s_diode_bank_n_d3  <= v_base_idx;
            s_diode_bank_n1_d3 <= v_next_idx;
        end if;
    end process;

    -- ========================================================================
    -- S4: Diode LUT bank-select cross-fade across all 3 channels (Y, U, V).
    --
    -- Weight = s_cv_eff(9:8) directly — top 2 bits of cv, giving 4
    -- distinct zones across the full cv range:
    --   cv 0..255  → weight 0 (pure bank N)
    --   cv 256..511 → weight 1 (small blend toward N+1)
    --   cv 512..767 → weight 2 (mid blend)
    --   cv 768..1023 → weight 3 (mostly N+1)
    -- cv(9:8) is constant within each 256-LSB chunk, so bank flips
    -- (which happen at 384/640/896 with C_BANK_HYST=128) occur INSIDE a
    -- constant-weight zone — the weight stays the same across the flip,
    -- only the bank index advances → smoother handoff with each cv zone
    -- holding its own character.
    --
    -- Critical path: 4-way mux × 2 + 11-bit sub + 2 cond shift+add +
    -- 3-input add → about 5 LUT levels per channel.
    -- ========================================================================
    p_diode_mux : process(clk)
        variable v_y_n,    v_y_n1   : signed(10 downto 0);
        variable v_u_n,    v_u_n1   : signed(10 downto 0);
        variable v_v_n,    v_v_n1   : signed(10 downto 0);
        variable v_y_diff, v_u_diff, v_v_diff : signed(10 downto 0);
        variable v_y_p1, v_y_p0     : signed(10 downto 0);
        variable v_u_p1, v_u_p0     : signed(10 downto 0);
        variable v_v_p1, v_v_p0     : signed(10 downto 0);
        variable v_y_blend, v_u_blend, v_v_blend : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- Bank-N values (Y, U, V parallel)
            case s_diode_bank_n_d3 is
                when 0 =>
                    v_y_n := signed(resize(s_y_diode_b0, 11));
                    v_u_n := signed(resize(s_u_diode_b0, 11));
                    v_v_n := signed(resize(s_v_diode_b0, 11));
                when 1 =>
                    v_y_n := signed(resize(s_y_diode_b1, 11));
                    v_u_n := signed(resize(s_u_diode_b1, 11));
                    v_v_n := signed(resize(s_v_diode_b1, 11));
                when 2 =>
                    v_y_n := signed(resize(s_y_diode_b2, 11));
                    v_u_n := signed(resize(s_u_diode_b2, 11));
                    v_v_n := signed(resize(s_v_diode_b2, 11));
                when others =>
                    v_y_n := signed(resize(s_y_diode_b3, 11));
                    v_u_n := signed(resize(s_u_diode_b3, 11));
                    v_v_n := signed(resize(s_v_diode_b3, 11));
            end case;

            -- Bank-N+1 values (Y, U, V parallel)
            case s_diode_bank_n1_d3 is
                when 0 =>
                    v_y_n1 := signed(resize(s_y_diode_b0, 11));
                    v_u_n1 := signed(resize(s_u_diode_b0, 11));
                    v_v_n1 := signed(resize(s_v_diode_b0, 11));
                when 1 =>
                    v_y_n1 := signed(resize(s_y_diode_b1, 11));
                    v_u_n1 := signed(resize(s_u_diode_b1, 11));
                    v_v_n1 := signed(resize(s_v_diode_b1, 11));
                when 2 =>
                    v_y_n1 := signed(resize(s_y_diode_b2, 11));
                    v_u_n1 := signed(resize(s_u_diode_b2, 11));
                    v_v_n1 := signed(resize(s_v_diode_b2, 11));
                when others =>
                    v_y_n1 := signed(resize(s_y_diode_b3, 11));
                    v_u_n1 := signed(resize(s_u_diode_b3, 11));
                    v_v_n1 := signed(resize(s_v_diode_b3, 11));
            end case;

            v_y_diff := v_y_n1 - v_y_n;
            v_u_diff := v_u_n1 - v_u_n;
            v_v_diff := v_v_n1 - v_v_n;

            -- Weight = s_cv_eff(9:8): 4 zones across cv 0..1023.
            -- Using s_cv_eff lets Knob 6 push the blend toward higher
            -- banks even with content cv held lower.
            v_y_p1 := (others => '0'); v_y_p0 := (others => '0');
            v_u_p1 := (others => '0'); v_u_p0 := (others => '0');
            v_v_p1 := (others => '0'); v_v_p0 := (others => '0');
            if s_cv_eff(9) = '1' then
                v_y_p1 := shift_right(v_y_diff, 1);
                v_u_p1 := shift_right(v_u_diff, 1);
                v_v_p1 := shift_right(v_v_diff, 1);
            end if;
            if s_cv_eff(8) = '1' then
                v_y_p0 := shift_right(v_y_diff, 2);
                v_u_p0 := shift_right(v_u_diff, 2);
                v_v_p0 := shift_right(v_v_diff, 2);
            end if;
            v_y_blend := v_y_n + v_y_p1 + v_y_p0;
            v_u_blend := v_u_n + v_u_p1 + v_u_p0;
            v_v_blend := v_v_n + v_v_p1 + v_v_p0;

            -- Blend bounded to [min(n,n1), max(n,n1)] ⊆ [0, 1023].
            s_y_diode <= unsigned(v_y_blend(9 downto 0));
            s_u_diode <= unsigned(v_u_blend(9 downto 0));
            s_v_diode <= unsigned(v_v_blend(9 downto 0));
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
    -- Opposite-signed UV inject (handled at S6) is required to produce
    -- hue rotation around grey rather than the magenta/green axis.
    -- ========================================================================
    p_comb_compute : process(clk)
        variable v_y_now             : signed(11 downto 0);
        variable v_y_chain           : signed(11 downto 0);
        variable v_y_prev            : signed(11 downto 0);
        variable v_delta_h           : signed(11 downto 0);
        variable v_delta_v           : signed(11 downto 0);
        variable v_delta             : signed(11 downto 0);
        variable v_env_boost         : integer range 0 to 1;
        variable v_comb_shift_pixel  : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            v_y_now   := signed(resize(s_y_diode,                     12));
            v_y_chain := signed(resize(unsigned(s_y_chain_d4),        12));
            v_y_prev  := signed(resize(s_y_prev_d4,                   12));

            v_delta_h := v_y_now - v_y_chain;
            v_delta_v := v_y_now - v_y_prev;
            v_delta   := v_delta_h + v_delta_v;

            -- Knob 6 unlocks an envelope-driven boost: shift_eff -1 (=
            -- 2× louder comb) when both envelope_react and cv_eff have
            -- their MSB set.  Gating on cv_eff (= cv_global + env_react>>2)
            -- preserves Knob 6 amplification but requires SOME content
            -- brightness — boost activates at cv_eff ≥ 512, which Knob 6
            -- max alone gives 255/1023 (won't fire on dark), but
            -- mid-content + Knob 6 max reaches 512 readily.
            v_env_boost := 0;
            if (s_envelope_react(9) and s_cv_eff(9)) = '1' then
                v_env_boost := 1;
            end if;
            v_comb_shift_pixel := s_comb_shift_eff - v_env_boost;
            -- Floor at 1 to prevent UV saturation at Knob 4 max +
            -- env_boost.  At shift_pixel = 0, comb_delta would be the
            -- full v_delta which saturates v_combed at 0/1023; floor at
            -- 1 caps max comb_delta at v_delta/2 = ±511, keeping UV
            -- linear for typical v_delta values.
            if v_comb_shift_pixel < 1 then
                v_comb_shift_pixel := 1;
            end if;

            -- Black-content attenuation: gate on the ORIGINAL input Y
            -- (s_y_chain_d4) rather than the post-LUT s_y_diode.  Hard
            -- Clip mode uses Bank 3 (Violent Fold) which maps black
            -- input (Y=0) to WHITE output (1023) via sin(-3π/2)=+1.
            -- Two-step ramp: input_y < 64 → /4, < 192 → /2, else
            -- unchanged.
            if unsigned(s_y_chain_d4) < to_unsigned(64, 10) then
                s_comb_delta <= shift_right(v_delta, v_comb_shift_pixel + 2);
            elsif unsigned(s_y_chain_d4) < to_unsigned(192, 10) then
                s_comb_delta <= shift_right(v_delta, v_comb_shift_pixel + 1);
            else
                s_comb_delta <= shift_right(v_delta, v_comb_shift_pixel);
            end if;
        end if;
    end process;

    -- ========================================================================
    -- S6: NTSC UV inject (opposite-signed) + Y companion register.
    --   s_u_combed = clamp(s_u_diode_d5 + s_comb_delta, 0, 1023)
    --   s_v_combed = clamp(s_v_diode_d5 - s_comb_delta, 0, 1023)
    -- U gets +delta, V gets -delta to produce hue rotation around grey
    -- rather than the magenta/green axis.
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

    -- Cross-fade rotation bin selection: the angle's upper 4 bits select
    -- bin_base; bin_base+1 (mod 16) is the adjacent bin.  Both bins'
    -- coefficients are looked up; the s7b stages blend their rotation
    -- results based on angle(5:3) (3-bit weight, 8 levels) — gives smooth
    -- hue interpolation through bin transitions.
    p_rot_bin_select : process(clk)
        variable v_bin_base : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            v_bin_base := to_integer(s_rot_angle(9 downto 6));
            s_rot_bin_base <= v_bin_base;
            if v_bin_base = 15 then
                s_rot_bin_next <= 0;
            else
                s_rot_bin_next <= v_bin_base + 1;
            end if;
        end if;
    end process;

    s_rot_bin_n  <= C_ROT_TABLE(s_rot_bin_base);
    s_rot_bin_n1 <= C_ROT_TABLE(s_rot_bin_next);

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

            -- Path N: rotation with bin_base coefficients
            shift_add_3(v_u_c, s_rot_bin_n.cos_a, s_rot_bin_n.cos_b, s_rot_bin_n.cos_c,
                        s_rot_bin_n.cos_signs, v_cos_u);
            shift_add_3(v_u_c, s_rot_bin_n.sin_a, s_rot_bin_n.sin_b, s_rot_bin_n.sin_c,
                        s_rot_bin_n.sin_signs, v_sin_u);
            shift_add_3(v_v_c, s_rot_bin_n.cos_a, s_rot_bin_n.cos_b, s_rot_bin_n.cos_c,
                        s_rot_bin_n.cos_signs, v_cos_v);
            shift_add_3(v_v_c, s_rot_bin_n.sin_a, s_rot_bin_n.sin_b, s_rot_bin_n.sin_c,
                        s_rot_bin_n.sin_signs, v_sin_v);
            s_cos_u_n_d7a <= v_cos_u;
            s_sin_u_n_d7a <= v_sin_u;
            s_cos_v_n_d7a <= v_cos_v;
            s_sin_v_n_d7a <= v_sin_v;

            -- Path N+1: rotation with bin_base+1 coefficients (parallel
            -- hardware, same cycle as Path N).  Doubles the s7a logic.
            -- The 14-bit shift_add_3 outputs are independent of Path N
            -- so the two paths have NO cross-coupling here — synthesis
            -- can place them in parallel LCs without timing pressure.
            shift_add_3(v_u_c, s_rot_bin_n1.cos_a, s_rot_bin_n1.cos_b, s_rot_bin_n1.cos_c,
                        s_rot_bin_n1.cos_signs, v_cos_u);
            shift_add_3(v_u_c, s_rot_bin_n1.sin_a, s_rot_bin_n1.sin_b, s_rot_bin_n1.sin_c,
                        s_rot_bin_n1.sin_signs, v_sin_u);
            shift_add_3(v_v_c, s_rot_bin_n1.cos_a, s_rot_bin_n1.cos_b, s_rot_bin_n1.cos_c,
                        s_rot_bin_n1.cos_signs, v_cos_v);
            shift_add_3(v_v_c, s_rot_bin_n1.sin_a, s_rot_bin_n1.sin_b, s_rot_bin_n1.sin_c,
                        s_rot_bin_n1.sin_signs, v_sin_v);
            s_cos_u_n1_d7a <= v_cos_u;
            s_sin_u_n1_d7a <= v_sin_u;
            s_cos_v_n1_d7a <= v_cos_v;
            s_sin_v_n1_d7a <= v_sin_v;

            -- Pipe the blend weight (angle bits 5:3, 0..7) and
            -- companion signals through s7a → s7b1 → s7b2.
            s_rot_weight_d7a <= s_rot_angle(5 downto 3);
            s_y_combed_d7a   <= s_y_combed;
            s_uv_swap_d7a    <= s_uv_swap;
        end if;
    end process;

    -- ========================================================================
    -- S7b1: compute both bins' rotation results and the inter-bin deltas.
    -- u_rot = u*cos - v*sin; v_rot = u*sin + v*cos (per bin path).
    -- Delta widened to 15-bit signed: rotation outputs reach ~±5800
    -- in extreme cases, so |rot_n1 - rot_n| can hit ~±11600 which
    -- exceeds 14-bit signed range (-8192..+8191).
    -- ========================================================================
    p_rotation_s7b1 : process(clk)
        variable v_u_rot_n,  v_v_rot_n  : signed(13 downto 0);
        variable v_u_rot_n1, v_v_rot_n1 : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            v_u_rot_n  := s_cos_u_n_d7a  - s_sin_v_n_d7a;
            v_v_rot_n  := s_sin_u_n_d7a  + s_cos_v_n_d7a;
            v_u_rot_n1 := s_cos_u_n1_d7a - s_sin_v_n1_d7a;
            v_v_rot_n1 := s_sin_u_n1_d7a + s_cos_v_n1_d7a;

            s_u_rot_n_d7b1 <= v_u_rot_n;
            s_v_rot_n_d7b1 <= v_v_rot_n;
            -- Widen to 15-bit before the subtract so the difference
            -- can't overflow.
            s_u_delta_d7b1 <= resize(v_u_rot_n1, 15) - resize(v_u_rot_n, 15);
            s_v_delta_d7b1 <= resize(v_v_rot_n1, 15) - resize(v_v_rot_n, 15);

            s_rot_weight_d7b1 <= s_rot_weight_d7a;
            s_uv_swap_d7b1    <= s_uv_swap_d7a;
            s_y_combed_d7b1   <= s_y_combed_d7a;
        end if;
    end process;

    -- ========================================================================
    -- S7b2: Cross-fade blend cascade + midpoint + clamp + UV swap.
    -- Split from s7b1 at the natural boundary (after the deltas are
    -- registered) to keep combinational depth within timing budget.
    -- ========================================================================
    p_rotation_s7b2 : process(clk)
        variable v_u_blend, v_v_blend : signed(15 downto 0);
        variable v_u_out,   v_v_out   : signed(15 downto 0);
        variable v_w                  : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            -- Cross-fade blend: result = base + (delta * w / 8), w 0..7.
            -- Decompose w into bits and conditionally add (delta >> k).
            -- Using 16-bit signed accumulator: base ±5800, plus three
            -- partials each up to ±5800/2 = ±2900, total worst case
            -- ±5800 + ±2900*3 ≈ ±14500 which fits comfortably in 16-bit
            -- signed (-32768..+32767).  +512 midpoint then well-bounded.
            v_w := to_integer(s_rot_weight_d7b1);
            v_u_blend := resize(s_u_rot_n_d7b1, 16);
            v_v_blend := resize(s_v_rot_n_d7b1, 16);

            -- Bit 2 of weight (= 4 of 8): add delta * 4/8 = delta/2
            if (v_w / 4) mod 2 = 1 then
                v_u_blend := v_u_blend + resize(shift_right(s_u_delta_d7b1, 1), 16);
                v_v_blend := v_v_blend + resize(shift_right(s_v_delta_d7b1, 1), 16);
            end if;
            -- Bit 1 (= 2 of 8): add delta * 2/8 = delta/4
            if (v_w / 2) mod 2 = 1 then
                v_u_blend := v_u_blend + resize(shift_right(s_u_delta_d7b1, 2), 16);
                v_v_blend := v_v_blend + resize(shift_right(s_v_delta_d7b1, 2), 16);
            end if;
            -- Bit 0 (= 1 of 8): add delta * 1/8
            if v_w mod 2 = 1 then
                v_u_blend := v_u_blend + resize(shift_right(s_u_delta_d7b1, 3), 16);
                v_v_blend := v_v_blend + resize(shift_right(s_v_delta_d7b1, 3), 16);
            end if;

            -- Add midpoint back, clamp to 10-bit unsigned.
            v_u_out := v_u_blend + to_signed(512, 16);
            v_v_out := v_v_blend + to_signed(512, 16);

            if s_uv_swap_d7b1 = '1' then
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

            s_y_rot <= s_y_combed_d7b1;
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
    begin
        if rising_edge(clk) then
            -- Sigma-delta accumulator on Knob 1 lower 7 bits.
            v_acc_next := ('0' & s_k_y_dither_acc) +
                          ('0' & unsigned(registers_in(0)(6 downto 0)));
            s_k_y_dither_acc <= v_acc_next(6 downto 0);

            -- Base k = Knob1(9..7).  Raw 0 promoted to 1 below.
            -- For the signed-IIR form (`state += (input - state) >> k`):
            --   k=1 → fast tracking = NO smear (state ≈ input)
            --   k=7 → slow tracking = MAX smear (state stuck on old value)
            -- So Knob 1 = 0 → k=1 (no smear, light decay).
            v_k_base := to_integer(unsigned(registers_in(0)(9 downto 7)));

            -- Per-pixel content luma drives k_eff so there's no spatial
            -- cliff at the half-frame row — k_eff varies smoothly with
            -- image content.  s_y_d10 is the input Y at depth 10
            -- (= s_y_rot timing, same pixel the IIR is processing).
            -- v_k_env in 0..7 from upper 3 bits.  Bright pixels shorten
            -- the smear; dark pixels lengthen it.
            v_k_env := to_integer(unsigned(s_y_d10(9 downto 7)));

            -- Combine + sigma-delta dither overflow + clamp 1..7.
            -- Dither bumps k UP so the LERP goes from k_base toward
            -- k_base + 1 as the lower knob bits increase — knob is
            -- monotonically increasing in smear character.
            v_k_y := v_k_base - v_k_env;
            if v_acc_next(7) = '1' and v_k_y < 7 then
                v_k_y := v_k_y + 1;
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

            -- Knob 2 mapping 8 - top3 (range 1..8): on natural content
            -- v_edge typically reaches ±50..300, so shift=1 at max knob
            -- gives max v_edge_sh ≈ ±150 (visible); on bars (v_edge ±2550)
            -- it saturates locally on edge pixels — the ringing
            -- characteristic, not a uniform white overlay.  Combined
            -- with explicit zero-gate at knob<64 in p_smear_reg, gives
            -- strict null at zero.
            v_shift_hi := 8 - to_integer(unsigned(registers_in(1)(9 downto 7)));
            s_ring_shift_eff <= v_shift_hi;
        end if;
    end process;

    -- ========================================================================
    -- S9: IIR state register — per-channel inline IIR with fractional
    -- bits.  Reset on hsync_falling (line boundary, capacitor discharge).
    -- Update gated by avid so state does not decay during blanking.
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
            -- Bleed Freeze gates ONLY the per-line reset, not the
            -- per-pixel update.  When active, state continues to evolve
            -- toward the input each pixel but persists across line
            -- boundaries → smear character builds up across the frame
            -- instead of resetting at every hsync.
            -- Gates use depth-11 signals to align with s_y_rot timing.
            -- Gating on shallower signals would cause cycles of
            -- blanking-zero input per active line → UV state collapses
            -- to 0 = green.
            if s_hsync_start_d10 = '1' and s_bleed_freeze = '0' then
                -- Reset at hsync_falling (depth-11 aligned): Y → 0
                -- (black), UV → 512<<8 (neutral grey — never 0 for UV).
                s_iir_y <= (others => '0');
                s_iir_u <= to_signed(C_IIR_UV_RESET, C_IIR_STATE_WIDTH);
                s_iir_v <= to_signed(C_IIR_UV_RESET, C_IIR_STATE_WIDTH);
            elsif s_avid_d10 = '1' then
                -- IIR update: state += (input - state) >> k_eff.
                -- Inputs are 10-bit unsigned, shifted left by
                -- C_IIR_FRAC_BITS to enter the state's data range
                -- (state(18..8) = data, state(7..0) = fractional acc).
                -- Use to_signed(... * 256) to avoid "0" & slv overload
                -- ambiguity.
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

                -- Bleed Freeze: stochastic update skipping.  When
                -- freeze is on, only update the IIR state when
                -- LFSR(2:0) = "000" (= 1/8 of pixels).  Effectively 8×
                -- slower decay = visible long trailing smear.  Combined
                -- with the line-reset gate above, state carries across
                -- lines AND decays slowly.
                if s_bleed_freeze = '1' and s_lfsr_q(2 downto 0) /= "000" then
                    v_delta_y := (others => '0');
                    v_delta_u := (others => '0');
                    v_delta_v := (others => '0');
                end if;

                -- IIR state ALWAYS converges to input (no AC/DC sign
                -- flip here).  AC/DC is applied at the smear-register
                -- stage as a MIRROR (output = 2*input − state) which
                -- produces the "negative shadow-smear" effect without
                -- making the IIR runaway-unstable.
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
        variable v_env_boost          : integer range 0 to 1;
        variable v_ring_shift_pixel   : integer range 0 to 15;
        variable v_edge_idx           : integer range 4 to C_EDGE_PIPE_LEN - 1;
    begin
        if rising_edge(clk) then
            -- Apply AC/DC at the smear contribution.
            v_y_iir := signed(resize(s_iir_y_out, 14));
            v_y_in  := signed(resize(s_y_rot_d10, 14));
            if s_ac_dc = '1' then
                v_y_smear := shift_left(v_y_in, 1) - v_y_iir;
            else
                -- Near-black threshold in DC mode.  When s_iir_y_out is
                -- below 16 (= upper 6 bits all zero, ≈ 1.5% of full Y
                -- range), force v_y_smear to 0.  Suppresses the
                -- per-pixel IIR-tracks-noise residual on near-black
                -- content; AC mode swallows it via the negative-then-
                -- clamp path, but DC mode passes it through directly.
                -- Cheap 6-bit zero-detect.
                if s_iir_y_out(9 downto 4) = "000000" then
                    v_y_smear := (others => '0');
                else
                    v_y_smear := v_y_iir;
                end if;
            end if;

            -- Edge ringing.  Env-boost gate uses AND-with-cv_eff
            -- (cv + env_react>>2).  Same logic as p_comb_compute —
            -- preserves Knob 6 amplification but won't fire on dark
            -- content.
            v_env_boost := 0;
            if (s_envelope_react(9) and s_cv_eff(9)) = '1' then
                v_env_boost := 1;
            end if;
            v_ring_shift_pixel := s_ring_shift_eff - v_env_boost;
            if v_ring_shift_pixel < 1 then
                v_ring_shift_pixel := 1;
            end if;

            -- Edge follow kicks in at TBC offset ≥ 4.  Slice (3:2)
            -- gives a 0..3 index range into the edge pipe.
            v_edge_idx := C_EDGE_PIPE_LEN - 1
                          - to_integer(s_tbc_offset(3 downto 2));
            v_edge    := shift_left(
                signed(resize(s_edge_pipe(v_edge_idx),     14)) +
                signed(resize(s_edge_pipe(v_edge_idx - 1), 14)) +
                signed(resize(s_edge_pipe(v_edge_idx - 2), 14)) +
                signed(resize(s_edge_pipe(v_edge_idx - 3), 14)) +
                signed(resize(s_edge_pipe(v_edge_idx - 4), 14)), 1);
            v_edge_sh := shift_right(v_edge, v_ring_shift_pixel);

            -- Zero-gate: knob 2 < 64 (1/16 of full scale) → strict
            -- null, no residual ring even at "off" position.
            if unsigned(registers_in(1)) < to_unsigned(64, 10) then
                v_edge_sh := (others => '0');
            end if;

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
    -- collapse.
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
            -- Tape noise is Y-domain ONLY (luma graininess).  Adding
            -- the same signed noise to U and V would displace UV along
            -- the magenta/green axis (equal-signed UV offset is not
            -- chroma modulation, it's a single-axis colour wiggle).
            v_noise := signed(resize(unsigned(s_lfsr_q(15 downto 8)), 10))
                       - to_signed(128, 10);

            -- Per-pixel Y-correlation: noise is loudest at midtones
            -- (where tape grit naturally lives) and quieter at extremes.
            -- v_y_dev = abs(s_smear_y - 512) → 0 at midtone, 511 at
            -- extremes.  Upper 2 bits give 0..3 of additional shift →
            -- up to 8× quieter at peak white/black.
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
    -- Stage 5b: 1-cycle pass-through register kept for downstream
    -- pipeline alignment.
    -- ========================================================================
    p_band_mux : process(clk)
    begin
        if rising_edge(clk) then
            s_crushed_y <= s_noisy_y;
            s_crushed_u <= s_noisy_u;
            s_crushed_v <= s_noisy_v;
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
    -- Wet path: master-mix interpolator output.  Goes through R1 clamp
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
    -- initialised to inactive defaults so GHDL doesn't propagate 'U'
    -- through the early pipeline.
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
            -- Per-pixel content luma at depth 10, for content-driven
            -- k_eff modulation in p_k_eff.
            s_y_d10           <= v_y_clean(C_IIR_GATE_DEPTH - 1);

            -- Dry-path data taps at depth 14 (= interpolator's "a" input).
            s_data_in_d14_y <= v_y_clean(C_DRY_TAP_DEPTH - 1);
            s_data_in_d14_u <= v_u_clean(C_DRY_TAP_DEPTH - 1);
            s_data_in_d14_v <= v_v_clean(C_DRY_TAP_DEPTH - 1);
            -- avid at depth 14 enables the interpolator instances.
            s_avid_d14      <= v_avid(C_DRY_TAP_DEPTH - 1);
        end if;
    end process;

end architecture sync_bleed;
