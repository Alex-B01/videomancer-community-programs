-- Videomancer community program - WITNESS Lite (Program 8, REV-D)
-- Copyright (C) 2026 AB (Sourdough_Stu)
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Program Name:
--   WITNESS Lite -- "the machine that looks at you."
--
-- Overview (REV-D re-scope, after two smokes over the die: 10,367 then 9,838):
--   A BLOCK-SPACE first-moment gaze tracker. Per active pixel the T7/hue-selected
--   weight (luma | |luma-cell| motion proxy, + always-on sat>>shift hue bias) folds
--   into one unified 64x32x8b mass/motion thumbnail (Bresenham-tiled). In vblank a
--   small serial unit scans the 2048 cells for the FIRST MOMENTS ONLY -> a
--   block-space centroid, made a pixel anchor by one per-frame (cx_blk*active_w)>>6
--   (and cy analogous) -> a rate-normalized 1-pole LAG (K3, rate_normalizer G=2) ->
--   the lagged gaze. A coordinate engine emits r^2 (adds-only) about the gaze,
--   feeding an IRIS radial vignette (K1/K6). PULL (K4) is a bounded horizontal
--   shear. EVADE (T8) flinches the gaze. Output blends dry/wet (Fader x3), R1-clamped.
--   Dropped vs full WITNESS: RINGS, ORBIT/stamps, ELLIPSE, TRAIL (K2/K5/T9/T10/T11
--   are DECLARED but INERT -- SEQUENCY precedent). No framebuffer.
--
-- Pipeline (as-built, total latency = C_PROCESSING_DELAY_CLKS = 16, ÷4):
--   R  input register .................................... 1  (cum 1)
--   PULL variable_delay_u read (delay 0 @ neutral) ....... 2  (cum 3)
--   PULL edge-clamp register ............................. 1  (cum 4)
--   IRIS interpolator_u (a=y, b=dark, t=darken) .......... 4  (cum 8)   (gain I1 parallel from r^2)
--   IRIS->fader luma pad ................................. 4  (cum 12)  <- fader input
--   FADER 3x interpolator_u (a=dry, b=wet, t=fader) + R1 clamp . 4  (cum 16)
--   Every composite is (a=passthrough, b=effect, t=amount, t=0 at neutral) -> wet==dry
--   bit-exact at load-neutral; the fader gives identity at any mix. Sync + dry Y/U/V
--   ride the 34x64 delay BRAM at a fixed depth, sync carried the last 4 (fader
--   companion) -> data_out sync delayed EXACTLY 16, param-independent (R5, R8).
--
-- EBR map (power-of-two) -- 9 EBR:
--   unified mass/motion thumbnail 2048x8 (inside witness_moments) ... 4
--   PULL shear delay 256x30 (variable_delay_u) ..................... 2
--   sync + dry Y/U/V 64x34 (variable_delay_u) ...................... 3   = 9 (vs MAX_EBR 14)
--
-- Multiplier ledger: 4x interpolator_u (fader 3 + IRIS 1) + 1 shared vblank
--   multiplier_s + narrowed divider. RINGS/stamp/ellipse/orbit multiplies all gone.
--
-- Register map: K1 IRIS(live) K2 RINGS(inert) K3 LAG(live) K4 PULL(live)
--   K5 ORBIT(inert) K6 EDGE(live) T7 Track(live) T8 Evade(live) T9 Ellipse(inert)
--   T10 Stamp Set(inert) T11 Trail(inert) Fader(live). Timing(8)(3:0)->rate_normalizer.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.rate_normalizer_pkg.all;

architecture witness of program_top is

  constant C_PROCESSING_DELAY_CLKS : integer := 16;    -- total latency (÷4)
  constant C_DRY_DELAY  : integer := 9;                -- variable_delay programmed (+2 -> dry @ fader input, cum 12)
  constant C_FADER_LAT  : integer := 4;                -- interpolator_u depth (sync carried here -> 16)
  constant C_IRIS_PAD   : integer := 4;                -- luma pad IRIS-out -> fader input
  constant C_UV_DELAY   : integer := 8;                -- U/V carry PULL-out -> fader input

  ------------------------------------------------------------------------
  -- LIVE parameters (K2/K5/T9/T10/T11 are declared in TOML but INERT: never read)
  ------------------------------------------------------------------------
  signal s_k_iris   : unsigned(9 downto 0) := (others => '1');
  signal s_k_lag    : unsigned(9 downto 0) := (others => '0');
  signal s_k_pull   : unsigned(9 downto 0) := to_unsigned(512, 10);
  signal s_k_edge   : unsigned(9 downto 0) := to_unsigned(512, 10);
  signal s_t_track  : std_logic := '0';
  signal s_t_evade  : std_logic := '0';
  signal s_fader    : unsigned(9 downto 0) := (others => '0');
  signal s_timing   : unsigned(3 downto 0) := (others => '0');

  ------------------------------------------------------------------------
  -- input registers + position tracking
  ------------------------------------------------------------------------
  signal s_y_in, s_u_in, s_v_in : unsigned(9 downto 0) := (others => '0');
  signal s_avid, s_hs_n, s_vs_n, s_fld_n : std_logic := '0';
  signal s_avid_d : std_logic := '0';
  signal s_vs_d   : std_logic := '1';
  signal s_new_line, s_new_frame, s_frame_end : std_logic := '0';
  signal s_px, s_py : unsigned(11 downto 0) := (others => '0');
  signal s_active_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
  signal s_active_h : unsigned(11 downto 0) := to_unsigned(1080, 12);
  signal s_hmeas    : unsigned(11 downto 0) := (others => '0');

  ------------------------------------------------------------------------
  -- rate normalizer (G=2: cx, cy) + gaze engine
  ------------------------------------------------------------------------
  signal s_base_shift   : t_shift_array(1 downto 0);
  signal s_norm_shift   : t_shift_array(1 downto 0);
  signal s_norm_shift_b : t_shift_array(1 downto 0);
  signal s_norm_use_b   : std_logic;

  signal s_cx_lag, s_cy_lag : signed(12 downto 0);
  signal s_cx2, s_cy2       : unsigned(23 downto 0);
  signal s_solve_done : std_logic;

  signal s_dx, s_dy : signed(12 downto 0);
  signal s_r2       : unsigned(23 downto 0);

  -- hue-octant approximator
  signal s_hue_oct  : unsigned(2 downto 0);
  signal s_hue_frac : unsigned(9 downto 0);
  signal s_hue_sat  : unsigned(10 downto 0);

  -- bresenham indexer
  signal s_cell_addr : unsigned(15 downto 0);
  signal s_blk_x, s_blk_y : unsigned(15 downto 0);
  signal s_blk_first, s_blk_last : std_logic;
  signal s_frac_x, s_frac_y : unsigned(12 downto 0);

  -- PULL shear
  signal s_pull_in, s_pull_out : unsigned(29 downto 0);
  signal s_pull_delay : unsigned(7 downto 0) := (others => '0');
  signal s_shear_acc  : signed(15 downto 0) := (others => '0');
  signal s_yP, s_uP, s_vP : unsigned(9 downto 0) := (others => '0');

  -- IRIS
  signal s_iris_t : unsigned(9 downto 0) := (others => '0');
  signal s_r2_iris : unsigned(23 downto 0) := (others => '0');
  signal s_yI     : unsigned(9 downto 0);

  -- luma pad (IRIS-out -> fader input) + U/V carry
  type t_padi is array (0 to C_IRIS_PAD - 1) of unsigned(9 downto 0);
  signal s_yI_pad : t_padi := (others => (others => '0'));
  type t_uv is array (0 to C_UV_DELAY - 1) of unsigned(9 downto 0);
  signal s_uv_u : t_uv := (others => (others => '0'));
  signal s_uv_v : t_uv := (others => (others => '0'));
  signal s_wetY, s_wetU, s_wetV : unsigned(9 downto 0);

  -- sync + dry delay (34x64) + 4 fader-companion -> 16
  signal s_sd_in, s_sd_out : unsigned(33 downto 0);
  signal s_dryY, s_dryU, s_dryV : unsigned(9 downto 0);
  type t_sync4 is array (0 to C_FADER_LAT - 1) of std_logic_vector(3 downto 0);
  signal s_sync_pipe : t_sync4 := (others => (others => '0'));

  signal s_faderY, s_faderU, s_faderV : unsigned(9 downto 0);

begin

  ------------------------------------------------------------------------
  -- parameter capture (LIVE only; inert controls left unread)
  ------------------------------------------------------------------------
  p_params : process (clk)
  begin
    if rising_edge(clk) then
      s_k_iris  <= unsigned(registers_in(0));
      s_k_lag   <= unsigned(registers_in(2));
      s_k_pull  <= unsigned(registers_in(3));
      s_k_edge  <= unsigned(registers_in(5));
      s_t_track <= registers_in(6)(0);
      s_t_evade <= registers_in(6)(1);
      s_fader   <= unsigned(registers_in(7));
      s_timing  <= unsigned(registers_in(8)(3 downto 0));
    end if;
  end process p_params;

  s_base_shift(0) <= 1 + to_integer(s_k_lag(9 downto 7));
  s_base_shift(1) <= 1 + to_integer(s_k_lag(9 downto 7));

  u_ratenorm : entity work.rate_normalizer
    generic map (G_NUM_COEFFS => 2)
    port map (clk => clk, timing_id => s_timing, base_shift => s_base_shift,
              norm_shift => s_norm_shift, norm_shift_b => s_norm_shift_b, norm_use_b => s_norm_use_b);

  ------------------------------------------------------------------------
  -- input register + position / edges + measured active dims
  ------------------------------------------------------------------------
  p_input : process (clk)
  begin
    if rising_edge(clk) then
      s_y_in <= unsigned(data_in.y);
      s_u_in <= unsigned(data_in.u);
      s_v_in <= unsigned(data_in.v);
      s_avid <= data_in.avid;
      s_hs_n <= data_in.hsync_n;
      s_vs_n <= data_in.vsync_n;
      s_fld_n<= data_in.field_n;
      s_avid_d <= data_in.avid;
      s_vs_d   <= data_in.vsync_n;

      s_new_line  <= '0';
      s_new_frame <= '0';
      s_frame_end <= '0';

      if data_in.avid = '1' and s_avid_d = '0' then
        s_new_line <= '1';
        s_px <= (others => '0');
        s_py <= s_py + 1;
        s_hmeas <= s_hmeas + 1;
      elsif data_in.avid = '1' then
        s_px <= s_px + 1;
      end if;

      if data_in.avid = '0' and s_avid_d = '1' then
        if s_px /= 0 then s_active_w <= s_px + 1; end if;
      end if;

      if data_in.vsync_n = '0' and s_vs_d = '1' then
        s_new_frame <= '1';
        s_frame_end <= '1';
        s_py <= (others => '0');
        if s_hmeas /= 0 then s_active_h <= s_hmeas; end if;
        s_hmeas <= (others => '0');
      end if;
    end if;
  end process p_input;

  ------------------------------------------------------------------------
  -- hue-octant, bresenham, gaze engine, coordinate engine
  ------------------------------------------------------------------------
  u_hue : entity work.witness_hue
    port map (clk => clk, u => s_u_in, v => s_v_in,
              octant => s_hue_oct, frac => s_hue_frac, sat => s_hue_sat);

  u_bres : entity work.bresenham_indexer
    generic map (G_BLOCKS_X => 64, G_BLOCKS_Y => 32)
    port map (clk => clk, avid => s_avid, hsync_n => s_hs_n, vsync_n => s_vs_n,
              block_x => s_blk_x, block_y => s_blk_y, cell_addr => s_cell_addr,
              block_x_first => s_blk_first, block_x_last => s_blk_last,
              frac_x => s_frac_x, frac_y => s_frac_y);

  u_moments : entity work.witness_moments
    port map (clk => clk, active => s_avid, cell_addr => s_cell_addr(10 downto 0),
              luma => s_y_in, sat => s_hue_sat, t7_track => s_t_track,
              frame_end => s_frame_end, active_w => s_active_w, active_h => s_active_h,
              t8_evade => s_t_evade,
              lag_shift => s_norm_shift(0), lag_shift_b => s_norm_shift_b(0), lag_use_b => s_norm_use_b,
              cx_lag => s_cx_lag, cy_lag => s_cy_lag, cx2 => s_cx2, cy2 => s_cy2,
              solve_done => s_solve_done);

  u_coord : entity work.witness_coord
    port map (clk => clk, active => s_avid, new_line => s_new_line, new_frame => s_new_frame,
              cx_lag => s_cx_lag, cy_lag => s_cy_lag, cx2 => s_cx2, cy2 => s_cy2,
              dx_o => s_dx, dy_o => s_dy, r2_o => s_r2);

  ------------------------------------------------------------------------
  -- PULL bounded shear (K4)
  ------------------------------------------------------------------------
  p_shear : process (clk)
    variable v_off : signed(15 downto 0);
  begin
    if rising_edge(clk) then
      if s_new_frame = '1' then
        s_shear_acc <= (others => '0');
      elsif s_new_line = '1' then
        s_shear_acc <= s_shear_acc + resize(signed('0' & s_k_pull) - to_signed(512, 11), 16);
      end if;
      v_off := shift_right(s_shear_acc, 4);
      if v_off < 0 then s_pull_delay <= (others => '0');
      elsif v_off > 255 then s_pull_delay <= to_unsigned(255, 8);
      else s_pull_delay <= unsigned(v_off(7 downto 0)); end if;
    end if;
  end process p_shear;

  s_pull_in <= s_y_in & s_u_in & s_v_in;
  u_pull : entity work.variable_delay_u
    generic map (G_WIDTH => 30, G_DEPTH => 8)
    port map (clk => clk, enable => '1', delay => s_pull_delay,
              a => s_pull_in, result => s_pull_out, valid => open);

  p_pullclamp : process (clk)
  begin
    if rising_edge(clk) then
      s_yP <= s_pull_out(29 downto 20);
      s_uP <= s_pull_out(19 downto 10);
      s_vP <= s_pull_out(9 downto 0);
    end if;
  end process p_pullclamp;

  ------------------------------------------------------------------------
  -- IRIS: r^2 vs radius^2(K1) + K6 EDGE falloff -> darken (t=0 -> identity)
  ------------------------------------------------------------------------
  p_iris_gain : process (clk)
    variable v_rad2 : unsigned(23 downto 0);
    variable v_over : unsigned(23 downto 0);
  begin
    if rising_edge(clk) then
      s_r2_iris <= s_r2;
      v_rad2 := shift_left(resize(s_k_iris, 24), 13);           -- open (1023) -> darken 0
      if s_r2_iris <= v_rad2 then
        s_iris_t <= (others => '0');
      else
        v_over := s_r2_iris - v_rad2;
        s_iris_t <= resize(shift_right(v_over, 8 + to_integer(s_k_edge(9 downto 8))), 10);
      end if;
    end if;
  end process p_iris_gain;

  u_iris : entity work.interpolator_u
    generic map (G_WIDTH => 10, G_FRAC_BITS => 10, G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
    port map (clk => clk, enable => '1',
              a => s_yP, b => to_unsigned(64, 10), t => s_iris_t, result => s_yI, valid => open);

  ------------------------------------------------------------------------
  -- luma pad (IRIS-out -> fader input) + U/V carry (PULL-out -> fader input)
  ------------------------------------------------------------------------
  p_carry : process (clk)
  begin
    if rising_edge(clk) then
      s_yI_pad(0) <= s_yI;
      for i in 1 to C_IRIS_PAD - 1 loop s_yI_pad(i) <= s_yI_pad(i - 1); end loop;
      s_uv_u(0) <= s_uP;
      s_uv_v(0) <= s_vP;
      for i in 1 to C_UV_DELAY - 1 loop
        s_uv_u(i) <= s_uv_u(i - 1);
        s_uv_v(i) <= s_uv_v(i - 1);
      end loop;
    end if;
  end process p_carry;
  s_wetY <= s_yI_pad(C_IRIS_PAD - 1);
  s_wetU <= s_uv_u(C_UV_DELAY - 1);
  s_wetV <= s_uv_v(C_UV_DELAY - 1);

  ------------------------------------------------------------------------
  -- SYNC + DRY delay (34x64) + 4 fader-companion FFs -> exactly 16
  ------------------------------------------------------------------------
  s_sd_in <= s_y_in & s_u_in & s_v_in & s_hs_n & s_vs_n & s_fld_n & s_avid;
  u_syncdry : entity work.variable_delay_u
    generic map (G_WIDTH => 34, G_DEPTH => 6)
    port map (clk => clk, enable => '1', delay => to_unsigned(C_DRY_DELAY, 6),
              a => s_sd_in, result => s_sd_out, valid => open);

  s_dryY <= s_sd_out(33 downto 24);
  s_dryU <= s_sd_out(23 downto 14);
  s_dryV <= s_sd_out(13 downto 4);

  p_synccarry : process (clk)
  begin
    if rising_edge(clk) then
      s_sync_pipe(0) <= std_logic_vector(s_sd_out(3 downto 0));
      for i in 1 to C_FADER_LAT - 1 loop s_sync_pipe(i) <= s_sync_pipe(i - 1); end loop;
    end if;
  end process p_synccarry;

  ------------------------------------------------------------------------
  -- FADER: 3x interpolator (a=dry, b=wet, t=fader) with R1 broadcast clamp
  ------------------------------------------------------------------------
  u_fader_y : entity work.interpolator_u
    generic map (G_WIDTH => 10, G_FRAC_BITS => 10, G_OUTPUT_MIN => 64, G_OUTPUT_MAX => 940)
    port map (clk => clk, enable => '1', a => s_dryY, b => s_wetY, t => s_fader,
              result => s_faderY, valid => open);
  u_fader_u : entity work.interpolator_u
    generic map (G_WIDTH => 10, G_FRAC_BITS => 10, G_OUTPUT_MIN => 64, G_OUTPUT_MAX => 960)
    port map (clk => clk, enable => '1', a => s_dryU, b => s_wetU, t => s_fader,
              result => s_faderU, valid => open);
  u_fader_v : entity work.interpolator_u
    generic map (G_WIDTH => 10, G_FRAC_BITS => 10, G_OUTPUT_MIN => 64, G_OUTPUT_MAX => 960)
    port map (clk => clk, enable => '1', a => s_dryV, b => s_wetV, t => s_fader,
              result => s_faderV, valid => open);

  ------------------------------------------------------------------------
  -- data_out : each field driven exactly once (R2); sync delayed EXACTLY 16 (R5/R8)
  ------------------------------------------------------------------------
  data_out.y       <= std_logic_vector(s_faderY);
  data_out.u       <= std_logic_vector(s_faderU);
  data_out.v       <= std_logic_vector(s_faderV);
  data_out.hsync_n <= s_sync_pipe(C_FADER_LAT - 1)(3);
  data_out.vsync_n <= s_sync_pipe(C_FADER_LAT - 1)(2);
  data_out.field_n <= s_sync_pipe(C_FADER_LAT - 1)(1);
  data_out.avid    <= s_sync_pipe(C_FADER_LAT - 1)(0);

end architecture witness;
