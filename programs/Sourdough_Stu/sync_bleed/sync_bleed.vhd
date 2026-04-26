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
-- =====  M2 STATUS — SKELETON PASSTHRU  =====
-- This commit lands the entity body with full register decode, sync delay,
-- and R1 output clamp.  Internal data path is currently *passthru* (the
-- dry-path tap wired to data_out).  Subsequent commits add the analysis
-- branch (M3), Stage 1 (M4-M5), Stage 2 diode LUTs (M6), Stage 3 comb +
-- twist (M7), Stage 4 IIR (M8), Stage 5 sync-crush + master mix (M9).

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
    -- Pipeline:
    --   1 clk  : S0   input registration, hcount/vcount, lfsr advance, edge mag
    --   1 clk  : S1   TBC jitter compute, BRAM read addrs
    --   2 clk  : S2   BRAM read (clocked)
    --   1 clk  : S3   BRAM output registration (idiom #7) + to_01
    --   2 clk  : S4   diode LUT read + bank mux
    --   2 clk  : S5   NTSC comb compute + UV inject
    --   2 clk  : S6   sin/cos rotation table + shift-add + UV swap
    --   1 clk  : S7   capacitor IIR state update
    --   1 clk  : S8   edge ringing add
    --   1 clk  : S9   tape noise + sync-crush mux
    --   4 clk  : S10-S13  interpolator_u master mix
    -- ─────────
    --  18 total
    constant C_PROCESSING_DELAY_CLKS : integer := 18;

    -- BRAM line buffer geometry — 2048 deep covers 1080p active line (1920) + headroom.
    constant C_BUF_SIZE      : integer := 2048;
    constant C_BUF_DEPTH     : integer := 11;
    constant C_PREV_OFFSET   : integer := 1920;     -- prev-line tap offset

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
    -- Sync delay outputs — 18-cycle dry-path taps for output and the
    -- master-mix interpolator's "a" input.  In M2 the d18 tap is wired
    -- straight to data_out (passthru behaviour, modulo 18-cycle latency).
    -- ========================================================================
    signal s_y_d18           : std_logic_vector(9 downto 0);
    signal s_u_d18           : std_logic_vector(9 downto 0);
    signal s_v_d18           : std_logic_vector(9 downto 0);

    -- ========================================================================
    -- "Wet" output — at M2 this is just the d18 dry tap (passthru).
    -- Subsequent milestones replace with actual processed output before
    -- the master mix interpolator.
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
    -- M2 wet-path stub: just the 18-cycle-delayed clean signal.
    -- Subsequent milestones replace this with stage outputs before the
    -- master-mix interpolator. interpolator is added at M9; for now
    -- s_wet_* feeds straight into the output clamp.
    -- ========================================================================
    s_wet_y <= unsigned(s_y_d18);
    s_wet_u <= unsigned(s_u_d18);
    s_wet_v <= unsigned(s_v_d18);

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
    -- Carries sync flags + Y/U/V dry-path taps + edge magnitude.
    -- All variables initialised to inactive defaults (idiom #4) so GHDL
    -- doesn't propagate 'U' through the early pipeline.
    -- ========================================================================
    p_sync_delay : process(clk)
        type t_sync_delay is array(0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic;
        type t_data_delay is array(0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic_vector(9 downto 0);
        variable v_avid    : t_sync_delay := (others => '0');
        variable v_hsync   : t_sync_delay := (others => '1');
        variable v_vsync   : t_sync_delay := (others => '1');
        variable v_field   : t_sync_delay := (others => '1');
        variable v_y_clean : t_data_delay := (others => (others => '0'));
        variable v_u_clean : t_data_delay := (others => (others => '0'));
        variable v_v_clean : t_data_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_avid    := data_in.avid    & v_avid(0    to C_PROCESSING_DELAY_CLKS - 2);
            v_hsync   := data_in.hsync_n & v_hsync(0   to C_PROCESSING_DELAY_CLKS - 2);
            v_vsync   := data_in.vsync_n & v_vsync(0   to C_PROCESSING_DELAY_CLKS - 2);
            v_field   := data_in.field_n & v_field(0   to C_PROCESSING_DELAY_CLKS - 2);
            v_y_clean := data_in.y       & v_y_clean(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_u_clean := data_in.u       & v_u_clean(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_v_clean := data_in.v       & v_v_clean(0 to C_PROCESSING_DELAY_CLKS - 2);

            -- Sync outputs at full pipeline depth.
            data_out.avid    <= v_avid(C_PROCESSING_DELAY_CLKS - 1);
            data_out.hsync_n <= v_hsync(C_PROCESSING_DELAY_CLKS - 1);
            data_out.vsync_n <= v_vsync(C_PROCESSING_DELAY_CLKS - 1);
            data_out.field_n <= v_field(C_PROCESSING_DELAY_CLKS - 1);

            -- 18-clock taps: dry path (M2 passthru output, becomes interp 'a' at M9).
            s_y_d18 <= v_y_clean(C_PROCESSING_DELAY_CLKS - 1);
            s_u_d18 <= v_u_clean(C_PROCESSING_DELAY_CLKS - 1);
            s_v_d18 <= v_v_clean(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process;

end architecture sync_bleed;
