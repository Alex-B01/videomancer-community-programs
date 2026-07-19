-- Copyright (C) 2026 AB (Sourdough_Stu)
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: howlbox.vhd - HOWLBOX Program for Videomancer (Program 3)
--
-- Program Name:
--   HOWLBOX  (analog bend: the virtual bent enhancer, CBV001 / "Basic Cable")
--
-- Author:
--   AB (Sourdough_Stu)
--
-- Overview:
--   One nonlinear virtual amplifier, processed per pixel, one pixel/clock, raster
--   order (no grid, no framebuffer). Two operating-point nodes -- node_E (drive)
--   and node_S (supply) -- are fed by 6 knobs + 5 toggles through an adds/shifts
--   coupling network and drive five behaviours: resonant peaking, a horizontal
--   2nd-order resonant IIR (ring), a comparator rupture, a chroma NCO rainbow, and
--   content-destabilised virtual sync (H-phase remap on the DATA path only). A
--   correlated STARVE death envelope first destabilises then collapses the picture
--   (build B, death-flare; Q1 resolved 2026-07-19). Temporal lags (STARVE, body
--   vactrol, Rainbow spin) run through a born-in, promotion-ready rate normalizer
--   so a knob means the same wall-clock feel at every format. True dry/wet crossfade.
--
--   HOWLBOX is the Phase-1 multiplier/LC-discipline validator: EXACTLY 4 runtime
--   multiplies (M1..M4), everything else shift / LUT / compare.
--
-- Pipeline (16 processing + 4 interpolator = 20 clocks; divisible by 4):
--   S0  Input latch + sample NCO phase / STARVE-sag / body accumulators
--   S1  OP-node coupling: node_E, node_S from knobs + lags (adds/shifts)
--   S2  AC/DC condition + peaking HF (3-tap horizontal LPF, shift-add)
--   S3  Peaking product ............................ MULT M1  (node_E times hf)
--   S4  Peaking sum -> IIR input  x = clamp(base + p1 >> kP)   (R3)
--   S5  IIR taps ................................... MULT M2 (a1 times y[n-1]),
--                                                    MULT M3 (a2 times y[n-2])
--   S6  IIR combine (recursion close), freeze-during-blanking (R7), r<1
--   S7  Rupture: comparator vs node_S-collapsed threshold, invert/fold (0 mult)
--   S8  NCO sine: quarter-wave LUT + quadrant reflection (registered)
--   S9  Chroma leak product ........................ MULT M4 (depth times sine)
--   S10 Remap write (avid-gated) + sync-stress read offset (shift/LUT, 0 mult)
--   S11 Remap read  (256x30 delay line, 1-clk BRAM latency)
--   S12 Remap register (R6 addr bounded to [0,255])
--   S13 Recombine Y/U/V
--   S14 Balance (Fmax / div-4 justification)
--   S15 R1 clamp -> wet (y in [64,940], u/v in [64,960])
--   B   Dry/wet interpolator_u (Y,U,V) with t = fader, dry delay-matched  (+4)
--   Total data_in -> data_out latency = 20.
--
-- Resources / EBR map (physical, power-of-two):
--   H-phase remap delay : 256 (2^8) x 30-bit, 1W + 1R -> 2 EBR
--                         (256x16 mode: 30 bits => 2 banks). ONLY memory.
--   NCO quarter-wave sine LUT (256x10) : combinational distributed LUT -> 0 EBR
--   Coupling nodes, lags, NCO phase, IIR prev-pixel taps : registers -> 0 EBR
--   TOTAL EBR = 2  (vs physical ceiling 32).
--
-- Runtime multiplies: 4 (M1..M4), all inline registered. <= 5 cap. 0 multiplier_s.
--   Every other operator is shift / LUT / compare (H4: a plain a b product is a
--   LUT multiplier on this DSP-less die).
--
-- Estimated LC ~4,920 (paper) vs 5,500 comfortable target. Serial pipeline
--   (howler-class); H3 register-bound rule does not apply.
--
-- Register map (SPI: pots -> registers_in(0..5); toggles -> registers_in(6) bits
--   0..4; fader -> registers_in(7); timing id -> registers_in(8)(3 downto 0)):
--   Pot 1 (0)      ENHANCE      master operating point (drive -> node_E)
--   Pot 2 (1)      RING         resonance & regeneration (IIR)
--   Pot 3 (2)      RAINBOW      chroma-regen depth + spin (NCO)
--   Pot 4 (3)      RUPTURE      comparator threshold (init = no rupture)
--   Pot 5 (4)      SYNC STRESS  brightness-coupled H-phase instability
--   Pot 6 (5)      STARVE       correlated systemic death (rate-normalized)
--   Tog 7 (6)(0)   Coupling     AC / DC          (1 = DC / full signal)
--   Tog 8 (6)(1)   Ring flavor  overshoot / undershoot
--   Tog 9 (6)(2)   Rainbow flav U / V leak        (1 = V leak)
--   Tog 10(6)(3)   Body contact luma-vactrol self-bend  (1 = On)
--   Tog 11(6)(4)   Crackle      edge-gated bias dirt      (1 = On)
--   Fader (7)      Dry / Wet    true crossfade (interpolator_u)
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.clamp_pkg.all;
use work.rate_normalizer_pkg.all;
use work.howlbox_sine_pkg.all;

architecture howlbox of program_top is

  -- ======================================================================
  -- Constants
  -- ======================================================================
  constant C_PROCESSING_DELAY_CLKS : integer := 16;         -- data_in -> wet (s_out)
  constant C_SYNC_DELAY_CLKS       : integer := 20;         -- 16 + 4 (interpolator)

  constant C_REMAP_BITS  : integer := 8;                    -- 256-deep (2^8), R6
  constant C_REMAP_SIZE  : integer := 256;
  constant C_REMAP_WIDTH : integer := 30;                   -- YUV444 packed

  -- Broadcast-safe output range (R1)
  constant C_Y_MIN : integer := 64;
  constant C_Y_MAX : integer := 940;
  constant C_C_MIN : integer := 64;
  constant C_C_MAX : integer := 960;

  -- Coupling shifts (small fixed, adds/shifts only -- 3.1)
  constant C_K_BODY : integer := 2;   -- body -> node_E
  constant C_K_ES   : integer := 2;   -- sag saps node_E
  constant C_K_SE   : integer := 3;   -- hot drive draws down node_S

  -- Death-flare envelope (build B)
  constant C_FLARE_KNEE : integer := 384;  -- starvation depth at which flare saturates
  constant C_K_RF   : integer := 2;   -- flare boosts ring r
  constant C_K_RC   : integer := 1;   -- collapse pulls ring r down (keeps r<1)
  constant C_K_RE   : integer := 2;   -- node_E boosts ring r
  constant C_K_RBF  : integer := 2;   -- flare blooms rainbow depth
  constant C_K_RBC  : integer := 1;   -- collapse fades rainbow depth
  constant C_K_RUPF : integer := 2;   -- flare collapses rupture threshold toward signal

  -- IIR resonator (r < 1 guaranteed: a1 capped, a2 = a1>>1 => pole r = sqrt(a2/512))
  constant C_R_MAX  : integer := 480;      -- a1 cap (< 512 => loop gain < 1)   R7
  constant C_KR     : integer := 9;        -- feedback >> 9  (a/512)
  constant C_KP     : integer := 9;        -- peaking product >> 9

  -- Chroma NCO
  constant C_KLEAK    : integer := 10;     -- leak product >> 10
  constant C_NCO_BASE : integer := 2;      -- base spatial phase step

  -- Sync-stress remap (0 mult -- content highpass + stress-selected shift)
  constant C_MIN_LAG       : integer := 2;    -- resting read lag (~ no shift at zero stress)
  constant C_MAX_DISP      : integer := 120;  -- bounded so read stays in-window
  constant C_DISP_SHIFTMAX : integer := 12;   -- zero stress => bright >> 12 = 0
  constant C_K_SYNCAMP     : integer := 5;    -- amplified as node_S falls

  -- Crackle
  constant C_EDGE_THRESH : integer := 64;

  -- Rate-normalizer nominal shifts @ 1080p30 reference
  constant C_SHIFT_STARVE : t_shift := 9;
  constant C_SHIFT_BODY   : t_shift := 8;
  constant C_SHIFT_SPIN   : t_shift := 4;

  -- ======================================================================
  -- Parameter signals (raw from register port; scaling lives in TOML)
  -- ======================================================================
  signal s_enhance    : unsigned(9 downto 0);
  signal s_ring       : unsigned(9 downto 0);
  signal s_rainbow    : unsigned(9 downto 0);
  signal s_rupture    : unsigned(9 downto 0);
  signal s_syncstress : unsigned(9 downto 0);
  signal s_starve     : unsigned(9 downto 0);
  signal s_dc_couple  : std_logic;
  signal s_ring_under : std_logic;
  signal s_rainbow_v  : std_logic;
  signal s_body_on    : std_logic;
  signal s_crackle_on : std_logic;
  signal s_fader      : unsigned(9 downto 0);
  signal s_timing_id  : unsigned(3 downto 0);

  -- ======================================================================
  -- Rate normalizer
  -- ======================================================================
  signal s_base_shift : t_shift_array(2 downto 0) :=
    (0 => C_SHIFT_STARVE, 1 => C_SHIFT_BODY, 2 => C_SHIFT_SPIN);
  signal s_norm_shift : t_shift_array(2 downto 0) :=
    (0 => C_SHIFT_STARVE, 1 => C_SHIFT_BODY, 2 => C_SHIFT_SPIN);

  -- ======================================================================
  -- Raster tracking (avid-edge active-width measurement, howler-style)
  -- ======================================================================
  signal s_prev_hsync_n     : std_logic := '1';
  signal s_prev_vsync_n     : std_logic := '1';
  signal s_prev_avid        : std_logic := '0';
  signal s_vsync_start      : std_logic := '0';
  signal s_active_pixel     : unsigned(11 downto 0) := (others => '0');
  signal s_active_width_reg : unsigned(11 downto 0) := to_unsigned(720, 12);
  signal s_luma_follow      : unsigned(9 downto 0) := (others => '0');
  signal s_edge             : std_logic := '0';

  -- ======================================================================
  -- Temporal envelopes (per-frame/field state; advance at vsync_start)
  -- ======================================================================
  signal s_sag          : unsigned(15 downto 0) := (others => '0');  -- STARVE death
  signal s_body         : unsigned(15 downto 0) := (others => '0');  -- luma vactrol
  signal s_nco_temporal : unsigned(9 downto 0)  := (others => '0');  -- Rainbow spin
  signal s_nco_spatial  : unsigned(9 downto 0)  := (others => '0');  -- per-pixel phase

  -- ======================================================================
  -- Derived control (slow, per-frame effective; referenced directly in stages)
  -- ======================================================================
  signal s_node_e        : unsigned(9 downto 0)  := (others => '0');
  signal s_node_s        : unsigned(9 downto 0)  := to_unsigned(1023, 10);
  signal s_flare         : unsigned(10 downto 0) := (others => '0');
  signal s_collapse      : unsigned(10 downto 0) := (others => '0');
  signal s_a1            : unsigned(11 downto 0) := (others => '0');
  signal s_a2            : unsigned(11 downto 0) := (others => '0');
  signal s_rainbow_depth : unsigned(9 downto 0)  := (others => '0');
  signal s_rupt_thresh   : unsigned(10 downto 0) := to_unsigned(1023, 11);

  -- ======================================================================
  -- Crackle LFSR
  -- ======================================================================
  signal s_lfsr : std_logic_vector(15 downto 0);

  -- ======================================================================
  -- Combinational quarter-wave sine lookup (distributed LUT ROM -> 0 EBR)
  -- ======================================================================
  signal s_sine_idx : unsigned(7 downto 0);
  signal s_sine_mag : std_logic_vector(9 downto 0);

  -- ======================================================================
  -- Pixel pipeline stages (signed luma/chroma, 12-bit headroom)
  -- ======================================================================
  -- S0
  signal s0_y, s0_u, s0_v : signed(11 downto 0) := (others => '0');
  signal s0_avid          : std_logic := '0';
  signal s0_phase         : unsigned(9 downto 0) := (others => '0');
  signal s_iny_d1         : unsigned(9 downto 0) := (others => '0');
  signal s_iny_d2         : unsigned(9 downto 0) := (others => '0');
  -- S1
  signal s1_y, s1_u, s1_v : signed(11 downto 0) := (others => '0');
  signal s1_avid          : std_logic := '0';
  signal s1_phase         : unsigned(9 downto 0) := (others => '0');
  signal s1_yd1, s1_yd2   : signed(11 downto 0) := (others => '0');
  -- S2
  signal s2_base          : signed(11 downto 0) := (others => '0');
  signal s2_hf            : signed(12 downto 0) := (others => '0');
  signal s2_u, s2_v       : signed(11 downto 0) := (others => '0');
  signal s2_avid          : std_logic := '0';
  signal s2_phase         : unsigned(9 downto 0) := (others => '0');
  -- S3
  signal s3_p1            : signed(25 downto 0) := (others => '0');   -- M1 product
  signal s3_base          : signed(11 downto 0) := (others => '0');
  signal s3_u, s3_v       : signed(11 downto 0) := (others => '0');
  signal s3_avid          : std_logic := '0';
  signal s3_phase         : unsigned(9 downto 0) := (others => '0');
  -- S4
  signal s4_x             : signed(12 downto 0) := (others => '0');
  signal s4_u, s4_v       : signed(11 downto 0) := (others => '0');
  signal s4_avid          : std_logic := '0';
  signal s4_phase         : unsigned(9 downto 0) := (others => '0');
  -- S5
  signal s5_x             : signed(12 downto 0) := (others => '0');
  signal s5_t1, s5_t2     : signed(25 downto 0) := (others => '0');   -- M2, M3 products
  signal s5_u, s5_v       : signed(11 downto 0) := (others => '0');
  signal s5_avid          : std_logic := '0';
  signal s5_phase         : unsigned(9 downto 0) := (others => '0');
  -- S6 (IIR recursion close + register taps)
  signal s6_y             : signed(12 downto 0) := (others => '0');
  signal s6_y_d1          : signed(12 downto 0) := (others => '0');   -- y[n-2] tap
  signal s6_u, s6_v       : signed(11 downto 0) := (others => '0');
  signal s6_avid          : std_logic := '0';
  signal s6_phase         : unsigned(9 downto 0) := (others => '0');
  -- S7
  signal s7_y             : signed(12 downto 0) := (others => '0');
  signal s7_u, s7_v       : signed(11 downto 0) := (others => '0');
  signal s7_avid          : std_logic := '0';
  signal s7_phase         : unsigned(9 downto 0) := (others => '0');
  -- S8
  signal s8_y             : signed(12 downto 0) := (others => '0');
  signal s8_sine          : signed(12 downto 0) := (others => '0');
  signal s8_u, s8_v       : signed(11 downto 0) := (others => '0');
  signal s8_avid          : std_logic := '0';
  -- S9
  signal s9_y, s9_u, s9_v : signed(11 downto 0) := (others => '0');
  signal s9_leak          : signed(25 downto 0) := (others => '0');   -- M4 product
  signal s9_avid          : std_logic := '0';
  -- S10 (write data + addresses)
  signal s10_y, s10_u, s10_v : signed(11 downto 0) := (others => '0');
  signal s10_avid            : std_logic := '0';
  signal s_wr_ptr            : unsigned(C_REMAP_BITS - 1 downto 0) := (others => '0');
  signal s10_wr_addr         : unsigned(C_REMAP_BITS - 1 downto 0) := (others => '0');
  signal s_rd_addr           : unsigned(C_REMAP_BITS - 1 downto 0) := (others => '0');
  -- S11/S12 (BRAM read)
  signal s_rd_data       : std_logic_vector(C_REMAP_WIDTH - 1 downto 0) := (others => '0');
  signal s12_rd          : std_logic_vector(C_REMAP_WIDTH - 1 downto 0) := (others => '0');
  -- S13/S14
  signal s13_y, s13_u, s13_v : unsigned(9 downto 0) := (others => '0');
  signal s14_y, s14_u, s14_v : unsigned(9 downto 0) := (others => '0');
  -- S15 wet output
  signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

  -- ======================================================================
  -- Remap delay line (the ONLY memory: 256 x 30 -> 2 EBR). No init (bitstream 0).
  -- ======================================================================
  type t_remap is array (0 to C_REMAP_SIZE - 1) of std_logic_vector(C_REMAP_WIDTH - 1 downto 0);
  signal s_remap : t_remap;

  -- ======================================================================
  -- Sync + dry delay (R5/R8)
  -- ======================================================================
  signal s_avid_d    : std_logic;
  signal s_hsync_n_d : std_logic;
  signal s_vsync_n_d : std_logic;
  signal s_field_n_d : std_logic;
  signal s_y_dry     : std_logic_vector(9 downto 0);
  signal s_u_dry     : std_logic_vector(9 downto 0);
  signal s_v_dry     : std_logic_vector(9 downto 0);

  -- Interpolator outputs
  signal s_mix_y, s_mix_u, s_mix_v : unsigned(9 downto 0);
  signal s_vy, s_vu, s_vv          : std_logic;

  -- ----------------------------------------------------------------------
  -- Local helper: clamp a signed value into an arbitrary [lo,hi] integer band.
  -- ----------------------------------------------------------------------
  function fn_clamp_band(v : signed; lo : integer; hi : integer) return signed is
    variable r : signed(v'length - 1 downto 0);
  begin
    if v < lo then
      r := to_signed(lo, v'length);
    elsif v > hi then
      r := to_signed(hi, v'length);
    else
      r := v;
    end if;
    return r;
  end function;

begin

  -- ======================================================================
  -- Register mapping (zero fabric on knob response; raw values only)
  -- ======================================================================
  s_enhance    <= unsigned(registers_in(0));
  s_ring       <= unsigned(registers_in(1));
  s_rainbow    <= unsigned(registers_in(2));
  s_rupture    <= unsigned(registers_in(3));
  s_syncstress <= unsigned(registers_in(4));
  s_starve     <= unsigned(registers_in(5));
  s_dc_couple  <= registers_in(6)(0);
  s_ring_under <= registers_in(6)(1);
  s_rainbow_v  <= registers_in(6)(2);
  s_body_on    <= registers_in(6)(3);
  s_crackle_on <= registers_in(6)(4);
  s_fader      <= unsigned(registers_in(7));
  s_timing_id  <= unsigned(registers_in(8)(3 downto 0));

  -- ======================================================================
  -- Rate normalizer (born-in, promotion-ready; governs 3 temporal constants)
  -- ======================================================================
  rate_norm_inst : entity work.rate_normalizer
    generic map (G_NUM_COEFFS => 3)
    port map (
      clk        => clk,
      timing_id  => s_timing_id,
      base_shift => s_base_shift,
      norm_shift => s_norm_shift
    );

  -- ======================================================================
  -- Crackle noise source
  -- ======================================================================
  lfsr_inst : entity work.lfsr16
    port map (clk => clk, enable => '1', seed => x"1234", load => '0', q => s_lfsr);

  -- ======================================================================
  -- Combinational quarter-wave sine ROM (distributed LUT -> 0 EBR)
  --   fold: 2nd/4th quarter mirror the index; sign from phase MSB.
  -- ======================================================================
  s_sine_idx <= s7_phase(7 downto 0) when s7_phase(8) = '0'
                else not s7_phase(7 downto 0);
  s_sine_mag <= C_SINE_QUARTER(to_integer(s_sine_idx));

  -- ======================================================================
  -- Raster tracking: active-pixel counter, avid-edge width, vsync pulse,
  -- fast luma follower, input luma taps, crackle edge.
  -- ======================================================================
  p_raster : process (clk)
    variable v_new : signed(12 downto 0);
  begin
    if rising_edge(clk) then
      s_prev_hsync_n <= data_in.hsync_n;
      s_prev_vsync_n <= data_in.vsync_n;
      s_prev_avid    <= data_in.avid;
      s_vsync_start  <= '0';

      if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
        s_active_pixel <= (others => '0');
      elsif data_in.avid = '1' then
        s_active_pixel <= s_active_pixel + 1;
      end if;

      if data_in.avid = '0' and s_prev_avid = '1' then
        s_active_width_reg <= s_active_pixel;
      end if;

      if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
        s_vsync_start <= '1';
      end if;

      -- input luma taps (for the peaking LPF and crackle edge)
      s_iny_d1 <= unsigned(data_in.y);
      s_iny_d2 <= s_iny_d1;
      if abs(signed(resize(unsigned(data_in.y), 12)) - signed(resize(s_iny_d1, 12)))
           > to_signed(C_EDGE_THRESH, 12) then
        s_edge <= '1';
      else
        s_edge <= '0';
      end if;

      -- fast luma envelope follower (brightness proxy for the body vactrol)
      if data_in.avid = '1' then
        v_new := signed(resize(s_luma_follow, 13))
          + shift_right(signed(resize(unsigned(data_in.y), 13))
                        - signed(resize(s_luma_follow, 13)), 6);
        s_luma_follow <= fn_clamp_s_to_u(v_new, 10);
      end if;
    end if;
  end process p_raster;

  -- ======================================================================
  -- Free-running spatial NCO phase (per active pixel, RAINBOW-set frequency)
  -- ======================================================================
  p_nco_spatial : process (clk)
  begin
    if rising_edge(clk) then
      if data_in.avid = '1' then
        s_nco_spatial <= s_nco_spatial
          + to_unsigned(C_NCO_BASE, 10)
          + shift_right(s_rainbow, 3);
      end if;
    end if;
  end process p_nco_spatial;

  -- ======================================================================
  -- Temporal envelopes: advance sag / body / spin ONCE per vsync (per field).
  -- Rate-normalized shifts -> constant wall-clock feel across formats.  R7.
  -- ======================================================================
  p_temporal : process (clk)
    variable v_target : signed(17 downto 0);
    variable v_diff   : signed(17 downto 0);
  begin
    if rising_edge(clk) then
      if s_vsync_start = '1' then
        -- STARVE death envelope: follows the STARVE knob (mapped to 16-bit scale)
        v_target := signed(resize(shift_left(resize(s_starve, 16), 6), 18));
        v_diff   := v_target - signed(resize(s_sag, 18));
        s_sag    <= fn_clamp_s_to_u(
          signed(resize(s_sag, 18)) + shift_right(v_diff, s_norm_shift(0)), 16);

        -- body-contact vactrol: follows luma envelope when engaged, else relaxes to 0
        if s_body_on = '1' then
          v_target := signed(resize(shift_left(resize(s_luma_follow, 16), 6), 18));
        else
          v_target := (others => '0');
        end if;
        v_diff := v_target - signed(resize(s_body, 18));
        s_body <= fn_clamp_s_to_u(
          signed(resize(s_body, 18)) + shift_right(v_diff, s_norm_shift(1)), 16);

        -- Rainbow temporal spin: per-frame phase advance, rate-normalized
        s_nco_temporal <= s_nco_temporal + shift_right(s_rainbow, s_norm_shift(2));
      end if;
    end if;
  end process p_temporal;

  -- ======================================================================
  -- Derived control (adds/shifts + clamps, NO multiplies). Death-flare (B):
  --   rising STARVE -> node_S falls -> flare (peaks) then collapse (wins);
  --   ring r ALWAYS < 1, rupture threshold falls toward signal, rainbow blooms.
  -- Uses last-cycle registered node values (all slow, per-frame effective).
  -- ======================================================================
  p_nodes : process (clk)
    variable v_body_hi   : unsigned(9 downto 0);
    variable v_sag_hi    : unsigned(9 downto 0);
    variable v_e_raw     : signed(13 downto 0);
    variable v_node_e    : signed(13 downto 0);
    variable v_s_sup     : signed(13 downto 0);
    variable v_node_s    : signed(13 downto 0);
    variable v_crackle   : signed(13 downto 0);
    variable v_starve_d  : signed(13 downto 0);
    variable v_flare     : signed(13 downto 0);
    variable v_collapse  : signed(13 downto 0);
    variable v_rgain     : signed(13 downto 0);
    variable v_depth     : signed(13 downto 0);
    variable v_thresh    : signed(13 downto 0);
  begin
    if rising_edge(clk) then
      v_sag_hi  := s_sag(15 downto 6);
      v_body_hi := s_body(15 downto 6);

      -- E_raw = ENHANCE + (body >> kBody)
      v_e_raw := signed(resize(s_enhance, 14))
               + signed(resize(shift_right(v_body_hi, C_K_BODY), 14));

      -- crackle bias dirt on the supply (edge-gated LFSR), tog11
      if s_crackle_on = '1' and s_edge = '1' then
        v_crackle := resize(signed('0' & s_lfsr(4 downto 0)) - to_signed(16, 7), 14);
      else
        v_crackle := (others => '0');
      end if;

      -- node_E = clamp( E_raw - (sag >> kES) )      -- starvation saps drive
      v_node_e := v_e_raw - signed(resize(shift_right(v_sag_hi, C_K_ES), 14));
      s_node_e <= fn_clamp_s_to_u(v_node_e, 10);

      -- node_S = clamp( (STARVE_base - sag) - (E_raw >> kSE) + crackle )
      v_s_sup  := to_signed(1023, 14) - signed(resize(v_sag_hi, 14));
      v_node_s := v_s_sup - signed(resize(shift_right(v_e_raw(11 downto 0), C_K_SE), 14))
                + v_crackle;
      s_node_s <= fn_clamp_s_to_u(v_node_s, 10);

      -- death-flare envelope from the (registered) supply depth
      v_starve_d := to_signed(1023, 14) - signed(resize(s_node_s, 14));
      if v_starve_d < 0 then
        v_flare := (others => '0');
      elsif v_starve_d > to_signed(C_FLARE_KNEE, 14) then
        v_flare := to_signed(C_FLARE_KNEE, 14);
      else
        v_flare := v_starve_d;
      end if;
      v_collapse := v_starve_d - v_flare;
      s_flare    <= unsigned(v_flare(10 downto 0));
      s_collapse <= unsigned(v_collapse(10 downto 0));

      -- ring pole radius a1 (r<1): RING + node_E boost + flare peak - collapse
      v_rgain := signed(resize(s_ring, 14))
               + signed(resize(shift_right(s_node_e, C_K_RE), 14))
               + signed(resize(shift_right(s_flare, C_K_RF), 14))
               - signed(resize(shift_right(s_collapse, C_K_RC), 14));
      v_rgain := fn_clamp_band(v_rgain, 0, C_R_MAX);
      s_a1 <= unsigned(v_rgain(11 downto 0));
      s_a2 <= unsigned(shift_right(v_rgain(11 downto 0), 1));   -- fixed pole ratio

      -- rainbow depth: bloom on flare, fade on collapse
      v_depth := signed(resize(s_rainbow, 14))
               + signed(resize(shift_right(s_flare, C_K_RBF), 14))
               - signed(resize(shift_right(s_collapse, C_K_RBC), 14));
      s_rainbow_depth <= fn_clamp_s_to_u(v_depth, 10);

      -- rupture threshold: collapses toward the signal as flare rises (init clean)
      v_thresh := to_signed(1023, 14) - signed(resize(s_rupture, 14))
                - signed(resize(shift_right(s_flare, C_K_RUPF), 14));
      v_thresh := fn_clamp_band(v_thresh, 32, 1023);
      s_rupt_thresh <= unsigned(v_thresh(10 downto 0));
    end if;
  end process p_nodes;

  -- ======================================================================
  -- S0: input latch + sample phase
  -- ======================================================================
  p_s0 : process (clk)
  begin
    if rising_edge(clk) then
      s0_y     <= signed(resize(unsigned(data_in.y), 12));
      s0_u     <= signed(resize(unsigned(data_in.u), 12));
      s0_v     <= signed(resize(unsigned(data_in.v), 12));
      s0_avid  <= data_in.avid;
      s0_phase <= s_nco_spatial + s_nco_temporal;
    end if;
  end process p_s0;

  -- ======================================================================
  -- S1: OP-node coupling stage (nodes computed in p_nodes; carry pixel + taps)
  -- ======================================================================
  p_s1 : process (clk)
  begin
    if rising_edge(clk) then
      s1_y     <= s0_y;
      s1_u     <= s0_u;
      s1_v     <= s0_v;
      s1_avid  <= s0_avid;
      s1_phase <= s0_phase;
      s1_yd1   <= s0_y;          -- previous-pixel luma taps for the LPF
      s1_yd2   <= s1_yd1;
    end if;
  end process p_s1;

  -- ======================================================================
  -- S2: AC/DC condition + peaking highpass (3-tap horizontal LPF, shift-add)
  -- ======================================================================
  p_s2 : process (clk)
    variable v_lpf_sum : signed(14 downto 0);
    variable v_lpf     : signed(12 downto 0);
    variable v_hf      : signed(12 downto 0);
  begin
    if rising_edge(clk) then
      -- LPF = (y[n-2] + 2 y[n-1] + y[n]) / 4  (centered near the current pixel)
      v_lpf_sum := resize(s1_yd2, 15) + resize(shift_left(s1_yd1, 1), 15)
                   + resize(s1_y, 15);
      v_lpf := resize(shift_right(v_lpf_sum, 2), 13);
      v_hf  := resize(s1_y, 13) - v_lpf;
      s2_hf <= v_hf;

      -- AC/DC: DC keeps the full signal, AC keeps the highpass recentered at mid.
      if s_dc_couple = '1' then
        s2_base <= s1_y;
      else
        s2_base <= resize(fn_clamp_band(v_hf + to_signed(512, 13), 0, 1023), 12);
      end if;

      s2_u     <= s1_u;
      s2_v     <= s1_v;
      s2_avid  <= s1_avid;
      s2_phase <= s1_phase;
    end if;
  end process p_s2;

  -- ======================================================================
  -- S3: Peaking product ............................ MULT M1
  -- ======================================================================
  p_s3 : process (clk)
    variable v_ne : signed(12 downto 0);
  begin
    if rising_edge(clk) then
      v_ne     := resize(signed('0' & std_logic_vector(s_node_e)), 13);
      s3_p1    <= v_ne * s2_hf;                                    -- MULT M1
      s3_base  <= s2_base;
      s3_u     <= s2_u;
      s3_v     <= s2_v;
      s3_avid  <= s2_avid;
      s3_phase <= s2_phase;
    end if;
  end process p_s3;

  -- ======================================================================
  -- S4: Peaking sum -> IIR input   x = clamp(base + p1 >> kP)   (R3)
  -- ======================================================================
  p_s4 : process (clk)
    variable v_sum : signed(13 downto 0);
  begin
    if rising_edge(clk) then
      v_sum    := resize(s3_base, 14) + resize(shift_right(s3_p1, C_KP), 14);
      s4_x     <= resize(fn_clamp_band(v_sum, 0, 1023), 13);      -- keep [0,1023]
      s4_u     <= s3_u;
      s4_v     <= s3_v;
      s4_avid  <= s3_avid;
      s4_phase <= s3_phase;
    end if;
  end process p_s4;

  -- ======================================================================
  -- S5: IIR taps .................. MULT M2 (a1 y[n-1]), MULT M3 (a2 y[n-2])
  --   y[n-1] = s6_y (last output), y[n-2] = s6_y_d1.
  -- ======================================================================
  p_s5 : process (clk)
    variable v_a1 : signed(12 downto 0);
    variable v_a2 : signed(12 downto 0);
  begin
    if rising_edge(clk) then
      v_a1     := resize(signed('0' & std_logic_vector(s_a1)), 13);
      v_a2     := resize(signed('0' & std_logic_vector(s_a2)), 13);
      s5_t1    <= v_a1 * s6_y;                                    -- MULT M2
      s5_t2    <= v_a2 * s6_y_d1;                                 -- MULT M3
      s5_x     <= s4_x;
      s5_u     <= s4_u;
      s5_v     <= s4_v;
      s5_avid  <= s4_avid;
      s5_phase <= s4_phase;
    end if;
  end process p_s5;

  -- ======================================================================
  -- S6: IIR combine (recursion close). r<1 + freeze-during-blanking (R7):
  --   during blanking the input is forced to 0 so the loop DECAYS to zero.
  -- ======================================================================
  p_s6 : process (clk)
    variable v_fb : signed(13 downto 0);
    variable v_x  : signed(13 downto 0);
    variable v_y  : signed(13 downto 0);
  begin
    if rising_edge(clk) then
      v_fb := resize(shift_right(s5_t1 - s5_t2, C_KR), 14);
      if s_ring_under = '1' then           -- tog8 overshoot / undershoot polarity
        v_fb := -v_fb;
      end if;
      if s5_avid = '1' then
        v_x := resize(s5_x, 14);
      else
        v_x := (others => '0');            -- freeze-during-blanking -> decay
      end if;
      v_y      := v_x + v_fb;
      s6_y     <= resize(fn_clamp_band(v_y, 0, 1023), 13);
      s6_y_d1  <= s6_y;                     -- y[n-2] tap
      s6_u     <= s5_u;
      s6_v     <= s5_v;
      s6_avid  <= s5_avid;
      s6_phase <= s5_phase;
    end if;
  end process p_s6;

  -- ======================================================================
  -- S7: Rupture -- fold above a node_S-collapsed threshold (solarization, 0 mult)
  -- ======================================================================
  p_s7 : process (clk)
    variable v_thr  : signed(12 downto 0);
    variable v_fold : signed(13 downto 0);
  begin
    if rising_edge(clk) then
      v_thr := resize(signed('0' & std_logic_vector(s_rupt_thresh)), 13);
      if s6_y > v_thr then
        v_fold := resize(shift_left(v_thr, 1), 14) - resize(s6_y, 14);  -- (thr<<1) - y
        s7_y   <= resize(fn_clamp_band(v_fold, 0, 1023), 13);
      else
        s7_y   <= s6_y;
      end if;
      s7_u     <= s6_u;
      s7_v     <= s6_v;
      s7_avid  <= s6_avid;
      s7_phase <= s6_phase;
    end if;
  end process p_s7;

  -- ======================================================================
  -- S8: NCO sine (quarter-wave LUT + quadrant reflection). Carry luma/chroma.
  -- ======================================================================
  p_s8 : process (clk)
    variable v_mag : signed(12 downto 0);
  begin
    if rising_edge(clk) then
      v_mag := resize(signed('0' & s_sine_mag), 13);
      if s7_phase(9) = '0' then
        s8_sine <= v_mag;
      else
        s8_sine <= -v_mag;
      end if;
      s8_y    <= s7_y;
      s8_u    <= s7_u;
      s8_v    <= s7_v;
      s8_avid <= s7_avid;
    end if;
  end process p_s8;

  -- ======================================================================
  -- S9: Chroma leak product ........................... MULT M4 (depth times sine)
  --   Carry raw luma/chroma; the leak is routed and applied at S10.
  -- ======================================================================
  p_s9 : process (clk)
    variable v_depth : signed(12 downto 0);
  begin
    if rising_edge(clk) then
      v_depth := resize(signed('0' & std_logic_vector(s_rainbow_depth)), 13);
      s9_leak <= v_depth * s8_sine;                              -- MULT M4
      s9_y    <= resize(s8_y, 12);
      s9_u    <= s8_u;
      s9_v    <= s8_v;
      s9_avid <= s8_avid;
    end if;
  end process p_s9;

  -- ======================================================================
  -- S10: apply chroma leak (route by tog9) + write pointer + sync-stress read
  --   offset (0 mult, R6). displacement grows with SYNC STRESS and content
  --   brightness, amplified as node_S falls; 0 at zero stress (resting lag ~
  --   no shift). Width-class keeps the shift proportional across formats.
  -- ======================================================================
  p_s10 : process (clk)
    variable v_leak : signed(12 downto 0);
    variable v_sh   : integer;
    variable v_disp : integer;
    variable v_amp  : integer;
    variable v_addr : integer;
  begin
    if rising_edge(clk) then
      v_leak := resize(shift_right(s9_leak, C_KLEAK), 13);
      s10_y  <= s9_y;
      if s_rainbow_v = '1' then            -- tog9: V leak
        s10_u <= s9_u;
        s10_v <= resize(fn_clamp_band(resize(s9_v, 14) + resize(v_leak, 14), 0, 1023), 12);
      else                                 -- U leak
        s10_u <= resize(fn_clamp_band(resize(s9_u, 14) + resize(v_leak, 14), 0, 1023), 12);
        s10_v <= s9_v;
      end if;
      s10_avid    <= s9_avid;
      s10_wr_addr <= s_wr_ptr;
      if s9_avid = '1' then
        s_wr_ptr <= s_wr_ptr + 1;          -- free-running rolling window
      end if;

      -- stress-selected shift: zero stress -> shift 12 -> displacement 0
      v_sh := C_DISP_SHIFTMAX - to_integer(s_syncstress(9 downto 6));
      if v_sh < 0 then
        v_sh := 0;
      elsif v_sh > C_DISP_SHIFTMAX then
        v_sh := C_DISP_SHIFTMAX;
      end if;
      v_disp := to_integer(shift_right(unsigned(std_logic_vector(s9_y(9 downto 0))), v_sh));

      -- amplified as the supply falls (a starving supply cannot hold H-timing)
      v_amp  := to_integer(shift_right(to_unsigned(1023, 11) - resize(s_node_s, 11),
                                       C_K_SYNCAMP));
      v_disp := v_disp + v_amp;

      -- width-class scale (compare-selected shift keeps the shift proportional)
      if s_active_width_reg < to_unsigned(1024, 12) then
        v_disp := v_disp / 2;
      end if;
      if v_disp > C_MAX_DISP then
        v_disp := C_MAX_DISP;
      end if;

      -- read address, wrapped to [0,255] (low 8 bits = mod 256)   R6
      v_addr    := to_integer(s_wr_ptr) - C_MIN_LAG - v_disp;
      s_rd_addr <= unsigned(to_signed(v_addr, 16)(C_REMAP_BITS - 1 downto 0));
    end if;
  end process p_s10;

  -- ======================================================================
  -- Remap BRAM write (avid-gated). Separate process, no init on array.
  -- ======================================================================
  p_remap_write : process (clk)
  begin
    if rising_edge(clk) then
      if s10_avid = '1' then
        s_remap(to_integer(s10_wr_addr)) <=
          std_logic_vector(s10_y(9 downto 0)) &
          std_logic_vector(s10_u(9 downto 0)) &
          std_logic_vector(s10_v(9 downto 0));
      end if;
    end if;
  end process p_remap_write;

  -- ======================================================================
  -- S11: Remap BRAM read (1-clk latency). Separate registered read process.
  -- ======================================================================
  p_remap_read : process (clk)
  begin
    if rising_edge(clk) then
      s_rd_data <= s_remap(to_integer(s_rd_addr));
    end if;
  end process p_remap_read;

  -- ======================================================================
  -- S12: register the read output (R6: addr already bounded to 8 bits)
  -- ======================================================================
  p_s12 : process (clk)
  begin
    if rising_edge(clk) then
      s12_rd <= s_rd_data;
    end if;
  end process p_s12;

  -- ======================================================================
  -- S13: recombine Y/U/V from the remapped (delayed) pixel
  -- ======================================================================
  p_s13 : process (clk)
  begin
    if rising_edge(clk) then
      s13_y <= unsigned(s12_rd(29 downto 20));
      s13_u <= unsigned(s12_rd(19 downto 10));
      s13_v <= unsigned(s12_rd(9 downto 0));
    end if;
  end process p_s13;

  -- ======================================================================
  -- S14: balance stage (Fmax / divisible-by-4 justification)
  -- ======================================================================
  p_s14 : process (clk)
  begin
    if rising_edge(clk) then
      s14_y <= s13_y;
      s14_u <= s13_u;
      s14_v <= s13_v;
    end if;
  end process p_s14;

  -- ======================================================================
  -- S15: R1 clamp -> wet (broadcast-safe range)
  -- ======================================================================
  p_s15 : process (clk)
  begin
    if rising_edge(clk) then
      if s14_y < C_Y_MIN then
        s_out_y <= to_unsigned(C_Y_MIN, 10);
      elsif s14_y > C_Y_MAX then
        s_out_y <= to_unsigned(C_Y_MAX, 10);
      else
        s_out_y <= s14_y;
      end if;

      if s14_u < C_C_MIN then
        s_out_u <= to_unsigned(C_C_MIN, 10);
      elsif s14_u > C_C_MAX then
        s_out_u <= to_unsigned(C_C_MAX, 10);
      else
        s_out_u <= s14_u;
      end if;

      if s14_v < C_C_MIN then
        s_out_v <= to_unsigned(C_C_MIN, 10);
      elsif s14_v > C_C_MAX then
        s_out_v <= to_unsigned(C_C_MAX, 10);
      else
        s_out_v <= s14_v;
      end if;
    end if;
  end process p_s15;

  -- ======================================================================
  -- Dry/wet crossfade (interpolator_u for Y,U,V; t = fader). R1 clamp at ports.
  -- ======================================================================
  mix_y_inst : entity work.interpolator_u
    generic map (G_WIDTH => 10, G_FRAC_BITS => 10, G_OUTPUT_MIN => C_Y_MIN, G_OUTPUT_MAX => C_Y_MAX)
    port map (clk => clk, enable => '1', a => unsigned(s_y_dry), b => s_out_y,
              t => s_fader, result => s_mix_y, valid => s_vy);

  mix_u_inst : entity work.interpolator_u
    generic map (G_WIDTH => 10, G_FRAC_BITS => 10, G_OUTPUT_MIN => C_C_MIN, G_OUTPUT_MAX => C_C_MAX)
    port map (clk => clk, enable => '1', a => unsigned(s_u_dry), b => s_out_u,
              t => s_fader, result => s_mix_u, valid => s_vu);

  mix_v_inst : entity work.interpolator_u
    generic map (G_WIDTH => 10, G_FRAC_BITS => 10, G_OUTPUT_MIN => C_C_MIN, G_OUTPUT_MAX => C_C_MAX)
    port map (clk => clk, enable => '1', a => unsigned(s_v_dry), b => s_out_v,
              t => s_fader, result => s_mix_v, valid => s_vv);

  -- ======================================================================
  -- Sync + dry delay pipeline (R5: fixed depth, NO parameter influence; R8)
  -- ======================================================================
  p_delay : process (clk)
    type t_sync_delay is array (0 to C_SYNC_DELAY_CLKS - 1) of std_logic;
    type t_data_delay is array (0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic_vector(9 downto 0);
    variable v_avid  : t_sync_delay := (others => '0');
    variable v_hsync : t_sync_delay := (others => '1');
    variable v_vsync : t_sync_delay := (others => '1');
    variable v_field : t_sync_delay := (others => '1');
    variable v_yd    : t_data_delay := (others => (others => '0'));
    variable v_ud    : t_data_delay := (others => (others => '0'));
    variable v_vd    : t_data_delay := (others => (others => '0'));
  begin
    if rising_edge(clk) then
      v_avid  := data_in.avid    & v_avid(0 to C_SYNC_DELAY_CLKS - 2);
      v_hsync := data_in.hsync_n & v_hsync(0 to C_SYNC_DELAY_CLKS - 2);
      v_vsync := data_in.vsync_n & v_vsync(0 to C_SYNC_DELAY_CLKS - 2);
      v_field := data_in.field_n & v_field(0 to C_SYNC_DELAY_CLKS - 2);
      v_yd    := data_in.y & v_yd(0 to C_PROCESSING_DELAY_CLKS - 2);
      v_ud    := data_in.u & v_ud(0 to C_PROCESSING_DELAY_CLKS - 2);
      v_vd    := data_in.v & v_vd(0 to C_PROCESSING_DELAY_CLKS - 2);

      s_avid_d    <= v_avid(C_SYNC_DELAY_CLKS - 1);
      s_hsync_n_d <= v_hsync(C_SYNC_DELAY_CLKS - 1);
      s_vsync_n_d <= v_vsync(C_SYNC_DELAY_CLKS - 1);
      s_field_n_d <= v_field(C_SYNC_DELAY_CLKS - 1);
      s_y_dry     <= v_yd(C_PROCESSING_DELAY_CLKS - 1);
      s_u_dry     <= v_ud(C_PROCESSING_DELAY_CLKS - 1);
      s_v_dry     <= v_vd(C_PROCESSING_DELAY_CLKS - 1);
    end if;
  end process p_delay;

  -- ======================================================================
  -- Output assignment (each port driven exactly once -- R2)
  -- ======================================================================
  data_out.y <= std_logic_vector(s_mix_y);
  data_out.u <= std_logic_vector(s_mix_u);
  data_out.v <= std_logic_vector(s_mix_v);

  data_out.avid    <= s_avid_d;
  data_out.hsync_n <= s_hsync_n_d;
  data_out.vsync_n <= s_vsync_n_d;
  data_out.field_n <= s_field_n_d;

end architecture howlbox;
