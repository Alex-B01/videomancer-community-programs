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
-- =====  M5 STATUS — STAGE 1 BRAM + TBC OFFSET COMPUTE  =====
-- Prior milestones: M2 skeleton passthru (commit d2ccea1), M3 analysis
-- branch (commit 075eda3).  This commit lands Stage 1:
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

architecture sync_bleed of program_top is

    -- ========================================================================
    -- Pipeline & buffer constants
    -- ========================================================================
    -- Pipeline depth grows incrementally per milestone.  Final target = 18
    -- at M9 (interpolator output drives data_out).  Currently at M5:
    --   1 clk  : S0   input registration, hcount/vcount, lfsr advance,
    --                 edge mag, wr_addr increment, Y chain shift
    --   1 clk  : S1   BRAM read (TBC jitter compute is combinational
    --                 from registered inputs at S0)
    --   1 clk  : S2   BRAM output registration + to_01 sanitise
    -- ─────────
    --   3 total at M5
    --
    -- Future targets:
    --   M6 (diode LUTs):      +2  →  5 clocks
    --   M7 (comb + twist):    +4  →  9 clocks
    --   M8 (IIR + ringing):   +2  → 11 clocks
    --   M9 (sync-crush + interp): +7 → 18 clocks (with 4-clock interp)
    constant C_PROCESSING_DELAY_CLKS : integer := 3;

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

    -- Computed TBC offset (combinational at S0).  Combines four modulation
    -- sources, scaled by Knob 5 with sigma-delta dither.  Always
    -- non-negative (clamped) — TBC reads are at wr_addr - C_TBC_BASE -
    -- s_tbc_offset, and we never want to read into the future.
    -- Distinct from s_tbc_jitter (the raw Knob 5 register value).
    signal s_tbc_offset      : unsigned(8 downto 0);

    -- Sigma-delta dither for Knob 5 lower 7 bits (idiom #15).  Smooths
    -- perceived control across adjacent shift-amount levels.
    signal s_jitter_dither_acc : unsigned(6 downto 0) := (others => '0');
    signal s_jitter_shift_eff  : integer range 0 to 7 := 0;

    -- ========================================================================
    -- "Wet" output — at M5 this is the Stage 1 BRAM tap (TBC-jittered Y/U/V).
    -- Subsequent milestones layer Stages 2-5 on top.  Master mix
    -- interpolator arrives at M9; until then output is direct s_*_tbc.
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

    -- s_in_top_half is concurrent — true while current pixel's vcount is
    -- in the top half of the active frame.
    s_in_top_half <= '1' when s_vcount < to_unsigned(C_ACTIVE_HALF_HEIGHT, s_vcount'length)
                          else '0';

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
    p_tbc_jitter : process(s_y_d0, s_in_top_half, s_cv_luma_top, s_cv_luma_bot,
                          s_motion_proxy, s_phase_drift, s_jitter_shift_eff)
        variable v_cv_half  : unsigned(9 downto 0);
        variable v_content  : signed(11 downto 0);
        variable v_envelope : signed(11 downto 0);
        variable v_motion   : signed(11 downto 0);
        variable v_drift    : signed(11 downto 0);
        variable v_sum      : signed(13 downto 0);
        variable v_shifted  : signed(13 downto 0);
        variable v_abs      : unsigned(13 downto 0);
    begin
        -- Pick the spatial-half CV.
        if s_in_top_half = '1' then
            v_cv_half := s_cv_luma_top;
        else
            v_cv_half := s_cv_luma_bot;
        end if;

        v_content  := signed(resize(unsigned(s_y_d0), 12)) - to_signed(512, 12);
        v_envelope := signed(resize(v_cv_half, 12)) sll 1;
        v_motion   := resize(s_motion_proxy, 12) sll 2;
        v_drift    := resize(signed("0" & s_phase_drift(15 downto 7)), 12);

        v_sum     := resize(v_content,  14) + resize(v_envelope, 14)
                   + resize(v_motion,   14) + resize(v_drift,    14);
        v_shifted := shift_right(v_sum, s_jitter_shift_eff);

        -- Take absolute value, clamp to 9 bits (max C_TBC_BASE-able offset
        -- before underflow concerns: with 1920-pixel buffer we have plenty
        -- of headroom, but bound by 9-bit just to keep it sane).
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
    -- M5 wet path: Stage 1 BRAM TBC tap.  Subsequent milestones layer
    -- Stages 2-5 on top.  Master mix interpolator arrives at M9.
    -- ========================================================================
    s_wet_y <= s_y_tbc;
    s_wet_u <= s_u_tbc;
    s_wet_v <= s_v_tbc;

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
