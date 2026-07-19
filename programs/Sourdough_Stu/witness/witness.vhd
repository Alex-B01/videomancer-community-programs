-- Videomancer community program - WITNESS (Program 8)
-- Copyright (C) 2026 AB (Sourdough_Stu)
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Program Name:
--   WITNESS -- "the machine that looks at the subject."
--
-- Overview:
--   A full-resolution, multiply-free moments/centroid gaze tracker drives radial
--   optics. Per active pixel a weighted mass (luma or motion, always plus a
--   self-nulling chroma-saturation bias -- REV-B/OQ-2) is accumulated into wide
--   nested running-sum registers; in the following vblank ONE shared multiplier +
--   serial divider (off the pixel path) yields the gaze centroid and, on T9, a
--   second-moment ellipse. A rate-normalized 1-pole LAG renders the gaze with
--   format-consistent inertia. A coordinate engine emits r^2 = dx^2+dy^2 (adds
--   only) about the lagged gaze, feeding IRIS (radial vignette), RINGS (zone-plate
--   sin(k*r^2)) and PULL (pseudo-radial horizontal shear). Baked stamps orbit the
--   gaze; a 64x32 motion-history thumbnail supplies motion mass and the T11 trail.
--   Everything composites 1 px/clk and blends dry/wet (3x interpolator).
--   No framebuffer -- persistence is the 64x32 thumbnail only. Div-1 full clock.
--
-- Pipeline (as-built, total latency = C_PROCESSING_DELAY_CLKS = 36, divisible by 4):
--   R  input register .................................. 1   (cum 1)
--   PULL variable_delay_u read (delay 0 @ neutral) ..... 2   (cum 3)
--   PULL edge-clamp register ........................... 1   (cum 4)
--   RINGS  k*r^2 (inline * , the sole datapath mult) ... 4   (cum 8)
--   RINGS  quarter-wave sin LUT + quadrant reflect ..... 1   (cum 9)
--   RINGS  K2-gated fringe add into luma ............... 1   (cum 10)
--   IRIS   r^2 vs radius^2(K1) + K6 edge falloff -> gain 2   (cum 12)
--   IRIS   interpolator_u (a=y, b=dark, t=darken) ...... 4   (cum 16)
--   TRAIL  motion-history nearest-neighbour read ....... 2   (cum 18)
--   TRAIL  interpolator_u (a=y, b=ghost, t=T11 alpha) .. 4   (cum 22)
--   STAMP  <=8 bbox tests + sprite-ROM read ............ 3   (cum 25)
--   STAMP  interpolator_u (a=y, b=sprite, t=K5 alpha) .. 4   (cum 29)
--   ELLIPSE outline overlay (T9) ....................... 3   (cum 32)  <- fader input
--   FADER  3x interpolator_u (a=dry, b=wet, t=fader) + R1 clamp . 4  (cum 36)
--   Every composite is (a=passthrough,b=effect,t=amount) with t=0 at neutral, so
--   wet == dry bit-exact at load-neutral; the fader gives identity at any mix.
--   Sync + dry Y/U/V ride the 34-bit x 64 delay BRAM at a fixed 32, then sync is
--   carried the last 4 (fader-companion FFs) -> data_out sync delayed EXACTLY 36,
--   independent of every knob/toggle/fader (R5, R8).
--
-- EBR map (physical, power-of-two total depth) -- 11 EBR:
--   motion-history 64x32x8b (2048x8, 1W/1R, no init) ......... 4  (witness_motion_history)
--   PULL shear delay 256x30 (variable_delay_u) ............... 2  (u_pull)
--   sync + dry Y/U/V 64x34 (variable_delay_u) ................ 3  (u_syncdry)
--   stamp sprite ROM 512x8 (registered ROM, sets A/B) ........ 2  (witness_stamp; may pack to 1)
--                                                           = 11 (vs MAX_EBR 14; 10 if stamp packs)
--
-- LC estimate (Gate-0 PAPER, REV-B, register-split): (a) 2,154 + (b) 1,294 +
--   (c) 1,900 = ~5,350 (band 5,050-5,850). Multiplier ledger (§3.3): 1 vblank
--   multiplier_s + 6 interpolator_u (IRIS/Trail/Stamp/Fader x3) + 1 inline *
--   (RINGS). Gate-1 synth smoke is authoritative.
--
-- Register map (registers_in):
--   Pot 1 (0)   IRIS   vignette radius        quart_out   init 1023 (open)
--   Pot 2 (1)   RINGS  zone-plate frequency   quad_in     init 0    (off)
--   Pot 3 (2)   LAG    gaze inertia (rate-norm) quad_in   init 0    (instant)
--   Pot 4 (3)   PULL   pseudo-radial shear     linear/bip init 512  (none)
--   Pot 5 (4)   ORBIT  stamp count + speed     quad_in    init 0    (no stamps)
--   Pot 6 (5)   EDGE   iris falloff            linear     init 512
--   Tog 7 (6)(0) Track  Brightness / Motion (+hue bias)   init Brightness
--   Tog 8 (6)(1) Evade  Track / Evade (gesture)           init Track
--   Tog 9 (6)(2) Ellipse Off / Fit                        init Off
--   Tog 10(6)(3) Stamp Set A / B                          init Set A
--   Tog 11(6)(4) Trail  Off / Trail                       init Off
--   Fader (7)   Dry / Wet crossfade           linear     init 0    (dry)
--   Timing(8)(3:0) video_timing_id -> rate_normalizer
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.rate_normalizer_pkg.all;
use work.witness_sine_pkg.all;

architecture witness of program_top is

  constant C_PROCESSING_DELAY_CLKS : integer := 36;    -- total latency (÷4)
  constant C_SYNCDRY_TOTAL         : integer := 32;    -- dry/sync BRAM sample delay (fader input)
  constant C_FADER_LAT             : integer := 4;     -- interpolator_u depth (sync carried here -> 36)
  constant C_SAT_SHIFT            : integer := 3;      -- hue saturation bias >> (self-nulling)

  ------------------------------------------------------------------------
  -- parameters (registered off the port; musical scaling lives in the TOML)
  ------------------------------------------------------------------------
  signal s_k_iris   : unsigned(9 downto 0) := (others => '1');
  signal s_k_rings  : unsigned(9 downto 0) := (others => '0');
  signal s_k_lag    : unsigned(9 downto 0) := (others => '0');
  signal s_k_pull   : unsigned(9 downto 0) := to_unsigned(512, 10);
  signal s_k_orbit  : unsigned(9 downto 0) := (others => '0');
  signal s_k_edge   : unsigned(9 downto 0) := to_unsigned(512, 10);
  signal s_t_track  : std_logic := '0';
  signal s_t_evade  : std_logic := '0';
  signal s_t_ell    : std_logic := '0';
  signal s_t_setb   : std_logic := '0';
  signal s_t_trail  : std_logic := '0';
  signal s_fader    : unsigned(9 downto 0) := (others => '0');
  signal s_timing   : unsigned(3 downto 0) := (others => '0');

  ------------------------------------------------------------------------
  -- input registers + position tracking
  ------------------------------------------------------------------------
  signal s_y_in, s_u_in, s_v_in : unsigned(9 downto 0) := (others => '0');
  signal s_avid, s_hs_n, s_vs_n, s_fld_n : std_logic := '0';
  signal s_avid_d : std_logic := '0';
  signal s_vs_d   : std_logic := '1';
  signal s_new_line : std_logic := '0';
  signal s_new_frame: std_logic := '0';
  signal s_frame_end: std_logic := '0';
  signal s_px : unsigned(11 downto 0) := (others => '0');
  signal s_py : unsigned(11 downto 0) := (others => '0');

  ------------------------------------------------------------------------
  -- rate normalizer (G=3: cx-lag, cy-lag, orbit-speed)
  ------------------------------------------------------------------------
  signal s_base_shift : t_shift_array(2 downto 0);
  signal s_norm_shift : t_shift_array(2 downto 0);
  signal s_norm_shift_b : t_shift_array(2 downto 0);
  signal s_norm_use_b : std_logic;

  ------------------------------------------------------------------------
  -- gaze engine <-> coordinate engine
  ------------------------------------------------------------------------
  signal s_cx_lag, s_cy_lag : signed(12 downto 0);
  signal s_cx2, s_cy2       : unsigned(23 downto 0);
  signal s_ell_maj2, s_ell_min2 : unsigned(23 downto 0);
  signal s_ell_tilt : signed(2 downto 0);
  signal s_solve_done : std_logic;
  signal s_weight : unsigned(11 downto 0) := (others => '0');

  signal s_dx, s_dy : signed(12 downto 0);
  signal s_r2       : unsigned(23 downto 0);

  ------------------------------------------------------------------------
  -- hue-octant approximator
  ------------------------------------------------------------------------
  signal s_hue_oct  : unsigned(2 downto 0);
  signal s_hue_frac : unsigned(9 downto 0);
  signal s_hue_sat  : unsigned(10 downto 0);

  ------------------------------------------------------------------------
  -- bresenham indexer / motion-history
  ------------------------------------------------------------------------
  signal s_cell_addr : unsigned(15 downto 0);
  signal s_blk_x, s_blk_y : unsigned(15 downto 0);
  signal s_blk_first, s_blk_last : std_logic;
  signal s_frac_x, s_frac_y : unsigned(12 downto 0);
  signal s_mh_rd : std_logic_vector(7 downto 0);
  signal s_mh_wr : std_logic_vector(7 downto 0);
  signal s_mh_waddr : unsigned(10 downto 0) := (others => '0');

  ------------------------------------------------------------------------
  -- orbit phase
  ------------------------------------------------------------------------
  signal s_orbit_ang : unsigned(7 downto 0) := (others => '0');
  signal s_stamp_alpha : unsigned(7 downto 0);
  signal s_stamp_hit   : std_logic;

  ------------------------------------------------------------------------
  -- PULL shear
  ------------------------------------------------------------------------
  signal s_pull_in   : unsigned(29 downto 0);
  signal s_pull_out  : unsigned(29 downto 0);
  signal s_pull_delay: unsigned(7 downto 0) := (others => '0');
  signal s_shear_acc : signed(15 downto 0) := (others => '0');
  signal s_yP, s_uP, s_vP : unsigned(9 downto 0) := (others => '0');

  ------------------------------------------------------------------------
  -- sync + dry delay (34-bit x 64 @ 32) then +4 fader-companion -> 36
  ------------------------------------------------------------------------
  signal s_sd_in, s_sd_out : unsigned(33 downto 0);
  signal s_dryY, s_dryU, s_dryV : unsigned(9 downto 0);
  type t_sync4 is array (0 to C_FADER_LAT - 1) of std_logic_vector(3 downto 0);
  signal s_sync_pipe : t_sync4 := (others => (others => '0'));

  ------------------------------------------------------------------------
  -- RINGS pipeline
  ------------------------------------------------------------------------
  signal s_r2_g0 : unsigned(23 downto 0) := (others => '0');
  signal s_k2_g0 : unsigned(9 downto 0)  := (others => '0');
  signal s_ring_prod : unsigned(33 downto 0) := (others => '0');
  signal s_ring_ph   : unsigned(9 downto 0)  := (others => '0');
  signal s_ring_fr   : signed(10 downto 0)   := (others => '0');

  ------------------------------------------------------------------------
  -- Y wet pipeline nodes
  ------------------------------------------------------------------------
  signal s_yR   : unsigned(9 downto 0) := (others => '0');  -- after RINGS
  signal s_iris_t : unsigned(9 downto 0) := (others => '0');
  signal s_yI   : unsigned(9 downto 0);  -- IRIS interp out
  signal s_trail_t : unsigned(9 downto 0) := (others => '0');
  signal s_trail_b : unsigned(9 downto 0) := (others => '0');
  signal s_yT   : unsigned(9 downto 0);  -- Trail interp out
  signal s_stamp_t : unsigned(9 downto 0) := (others => '0');
  signal s_stamp_b : unsigned(9 downto 0) := (others => '0');
  signal s_yS   : unsigned(9 downto 0);  -- Stamp interp out
  signal s_yE   : unsigned(9 downto 0) := (others => '0');  -- after Ellipse overlay

  ------------------------------------------------------------------------
  -- U/V wet carry (PULL out -> fader input): delay to align with wet Y
  ------------------------------------------------------------------------
  constant C_UV_DELAY : integer := 29;
  type t_uv is array (0 to C_UV_DELAY - 1) of unsigned(9 downto 0);
  signal s_uv_u : t_uv := (others => (others => '0'));
  signal s_uv_v : t_uv := (others => (others => '0'));
  signal s_wetU, s_wetV : unsigned(9 downto 0);

  -- r^2 alignment delays into IRIS + Ellipse
  signal s_r2_iris : unsigned(23 downto 0) := (others => '0');
  signal s_r2_ell  : unsigned(23 downto 0) := (others => '0');

  -- fader / clamp outputs
  signal s_faderY, s_faderU, s_faderV : unsigned(9 downto 0);

  -- helper: saturating unsigned add to 10-bit, floor 0
  function add10(a : unsigned; b : signed) return unsigned is
    variable v : signed(12 downto 0);
  begin
    v := signed('0' & a) + resize(b, 13);
    if v < 0 then return to_unsigned(0, 10);
    elsif v > 1023 then return to_unsigned(1023, 10);
    else return unsigned(v(9 downto 0)); end if;
  end function;

begin

  ------------------------------------------------------------------------
  -- parameter capture
  ------------------------------------------------------------------------
  p_params : process (clk)
  begin
    if rising_edge(clk) then
      s_k_iris  <= unsigned(registers_in(0));
      s_k_rings <= unsigned(registers_in(1));
      s_k_lag   <= unsigned(registers_in(2));
      s_k_pull  <= unsigned(registers_in(3));
      s_k_orbit <= unsigned(registers_in(4));
      s_k_edge  <= unsigned(registers_in(5));
      s_t_track <= registers_in(6)(0);
      s_t_evade <= registers_in(6)(1);
      s_t_ell   <= registers_in(6)(2);
      s_t_setb  <= registers_in(6)(3);
      s_t_trail <= registers_in(6)(4);
      s_fader   <= unsigned(registers_in(7));
      s_timing  <= unsigned(registers_in(8)(3 downto 0));
    end if;
  end process p_params;

  -- LAG base shift from K3 (cx,cy share coeff 0/1); ORBIT base shift from K5 (coeff 2)
  s_base_shift(0) <= 1 + to_integer(s_k_lag(9 downto 7));
  s_base_shift(1) <= 1 + to_integer(s_k_lag(9 downto 7));
  s_base_shift(2) <= 2 + to_integer(s_k_orbit(9 downto 7));

  u_ratenorm : entity work.rate_normalizer
    generic map (G_NUM_COEFFS => 3)
    port map (
      clk => clk, timing_id => s_timing,
      base_shift => s_base_shift,
      norm_shift => s_norm_shift, norm_shift_b => s_norm_shift_b, norm_use_b => s_norm_use_b
    );

  ------------------------------------------------------------------------
  -- input register + position / edges
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

      if data_in.avid = '1' and s_avid_d = '0' then       -- avid rising: new active line
        s_new_line <= '1';
        s_px <= (others => '0');
        s_py <= s_py + 1;
      elsif data_in.avid = '1' then
        s_px <= s_px + 1;
      end if;

      if data_in.vsync_n = '0' and s_vs_d = '1' then       -- vsync active: frame boundary
        s_new_frame <= '1';
        s_frame_end <= '1';
        s_py <= (others => '0');
      end if;
    end if;
  end process p_input;

  ------------------------------------------------------------------------
  -- hue-octant approximator (parallel, off datapath)
  ------------------------------------------------------------------------
  u_hue : entity work.witness_hue
    port map (clk => clk, u => s_u_in, v => s_v_in,
              octant => s_hue_oct, frac => s_hue_frac, sat => s_hue_sat);

  ------------------------------------------------------------------------
  -- bresenham indexer + motion-history store
  ------------------------------------------------------------------------
  u_bres : entity work.bresenham_indexer
    generic map (G_BLOCKS_X => 64, G_BLOCKS_Y => 32)
    port map (
      clk => clk, avid => s_avid, hsync_n => s_hs_n, vsync_n => s_vs_n,
      block_x => s_blk_x, block_y => s_blk_y, cell_addr => s_cell_addr,
      block_x_first => s_blk_first, block_x_last => s_blk_last,
      frac_x => s_frac_x, frac_y => s_frac_y
    );

  u_motion : entity work.witness_motion_history
    port map (
      clk => clk, wr_en => s_avid, wr_addr => s_mh_waddr, wr_data => s_mh_wr,
      rd_addr => s_cell_addr(10 downto 0), rd_data => s_mh_rd
    );

  -- motion cell decay + inject (read-modify-write, saturating). Gate-2 tunes the
  -- motion proxy; here: activity = luma high bits; T11 inscribes brighter.
  p_motion : process (clk)
    variable v_old, v_dec, v_inj, v_new : unsigned(7 downto 0);
  begin
    if rising_edge(clk) then
      s_mh_waddr <= s_cell_addr(10 downto 0);
      v_old := unsigned(s_mh_rd);
      if v_old > 4 then v_dec := v_old - 4; else v_dec := (others => '0'); end if;
      if s_t_trail = '1' then v_inj := s_y_in(9 downto 2);
      else                    v_inj := s_y_in(9 downto 2); end if;
      if v_inj > v_dec then v_new := v_inj; else v_new := v_dec; end if;
      s_mh_wr <= std_logic_vector(v_new);
    end if;
  end process p_motion;

  ------------------------------------------------------------------------
  -- moments / centroid gaze engine (weight = base + sat>>C_SAT_SHIFT)
  ------------------------------------------------------------------------
  p_weight : process (clk)
    variable v_base : unsigned(9 downto 0);
    variable v_bias : unsigned(11 downto 0);
  begin
    if rising_edge(clk) then
      if s_t_track = '1' then
        v_base := unsigned(s_mh_rd) & "00";              -- Motion (scale 8b->10b)
      else
        v_base := s_y_in;                                 -- Brightness
      end if;
      v_bias := resize(shift_right(s_hue_sat, C_SAT_SHIFT), 12);   -- self-nulling
      s_weight <= resize(v_base, 12) + v_bias;
    end if;
  end process p_weight;

  u_moments : entity work.witness_moments
    port map (
      clk => clk, active => s_avid, new_line => s_new_line, frame_end => s_frame_end,
      weight => s_weight, t8_evade => s_t_evade, t9_ellipse => s_t_ell,
      lag_shift => s_norm_shift(0), lag_shift_b => s_norm_shift_b(0), lag_use_b => s_norm_use_b,
      cx_lag => s_cx_lag, cy_lag => s_cy_lag, cx2 => s_cx2, cy2 => s_cy2,
      ell_maj2 => s_ell_maj2, ell_min2 => s_ell_min2, ell_tilt => s_ell_tilt,
      solve_done => s_solve_done
    );

  u_coord : entity work.witness_coord
    port map (
      clk => clk, active => s_avid, new_line => s_new_line, new_frame => s_new_frame,
      cx_lag => s_cx_lag, cy_lag => s_cy_lag, cx2 => s_cx2, cy2 => s_cy2,
      dx_o => s_dx, dy_o => s_dy, r2_o => s_r2
    );

  ------------------------------------------------------------------------
  -- orbit phase advance (rate-normalized speed, coeff 2)
  ------------------------------------------------------------------------
  p_orbit : process (clk)
  begin
    if rising_edge(clk) then
      if s_new_frame = '1' then
        s_orbit_ang <= s_orbit_ang + resize(shift_right(s_k_orbit(9 downto 2), s_norm_shift(2)), 8);
      end if;
    end if;
  end process p_orbit;

  u_stamp : entity work.witness_stamp
    port map (
      clk => clk, active => s_avid, new_frame => s_new_frame, en => s_k_orbit(9),
      count => s_k_orbit(9 downto 6), setb => s_t_setb, orbit_angle => s_orbit_ang,
      gaze_x => s_cx_lag, gaze_y => s_cy_lag, px => s_px, py => s_py,
      stamp_alpha => s_stamp_alpha, stamp_hit => s_stamp_hit
    );

  ------------------------------------------------------------------------
  -- STAGE R already registered (s_y_in ..). PULL shear (per-line accumulator).
  ------------------------------------------------------------------------
  p_shear : process (clk)
    variable v_off : signed(15 downto 0);
  begin
    if rising_edge(clk) then
      if s_new_frame = '1' then
        s_shear_acc <= (others => '0');
      elsif s_new_line = '1' then
        -- accumulate (K4-512) per line -> reads as radial (|dy| grows from gaze)
        s_shear_acc <= s_shear_acc + resize(signed('0' & s_k_pull) - to_signed(512, 11), 16);
      end if;
      -- delay-port latency is FIXED (0 @ neutral) so latency stays param-independent;
      -- clamp non-negative into the 256-deep line (spec §5b: within active width).
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

  -- PULL edge-clamp register (P2)
  p_pullclamp : process (clk)
  begin
    if rising_edge(clk) then
      s_yP <= s_pull_out(29 downto 20);
      s_uP <= s_pull_out(19 downto 10);
      s_vP <= s_pull_out(9 downto 0);
    end if;
  end process p_pullclamp;

  ------------------------------------------------------------------------
  -- RINGS: k*r^2 (the sole datapath multiply) -> quarter-wave sin -> modulate
  ------------------------------------------------------------------------
  p_rings : process (clk)
    variable v_idx : integer range 0 to 255;
    variable v_smag: signed(10 downto 0);
    variable v_gate: unsigned(9 downto 0);
    variable v_dsh : integer range 2 to 5;
  begin
    if rising_edge(clk) then
      -- G1 (4): register operands then the inline product (phase wraps -> low bits)
      s_r2_g0 <= s_r2;
      s_k2_g0 <= s_k_rings;
      s_ring_prod <= resize(s_k2_g0 * s_r2_g0(23 downto 14), 34);  -- 10 x 10 -> 20b (THE datapath *)
      s_ring_ph   <= s_ring_prod(19 downto 10);             -- 10-bit phase
      -- G2 (1): quarter-wave sin LUT via ONE shared read: bit8 reflects the
      -- index, bit9 negates -> a single distributed-ROM lookup (not 4 copies).
      if s_ring_ph(8) = '1' then
        v_idx := 255 - to_integer(s_ring_ph(7 downto 0));
      else
        v_idx := to_integer(s_ring_ph(7 downto 0));
      end if;
      v_smag := signed('0' & C_SINE_QUARTER(v_idx));
      if s_ring_ph(9) = '1' then s_ring_fr <= -v_smag; else s_ring_fr <= v_smag; end if;
      -- G3 (1): K2-gated fringe add into luma (K2=0 -> no fringe -> identity).
      -- Depth is a SHIFT of K2 (no 2nd datapath multiply): higher K2 -> less shift.
      v_gate := s_k2_g0;
      if v_gate = 0 then
        s_yR <= s_yP;
      else
        v_dsh := 5 - to_integer(s_k2_g0(9 downto 8));
        s_yR  <= add10(s_yP, resize(shift_right(s_ring_fr, v_dsh), 11));
      end if;
    end if;
  end process p_rings;

  ------------------------------------------------------------------------
  -- IRIS: r^2 vs radius^2(K1) + K6 edge falloff -> darken amount (t)
  ------------------------------------------------------------------------
  p_iris_gain : process (clk)
    variable v_rad2 : unsigned(23 downto 0);
    variable v_over : unsigned(23 downto 0);
  begin
    if rising_edge(clk) then
      s_r2_iris <= s_r2_g0;                                     -- align r^2 to IRIS
      -- radius^2 threshold grows with K1 via a SHIFT (no multiply): open (1023)
      -- -> threshold > max r^2 -> darken 0 everywhere (identity); closed -> darken.
      v_rad2 := shift_left(resize(s_k_iris, 24), 13);
      if s_r2_iris <= v_rad2 then
        s_iris_t <= (others => '0');                            -- inside iris: no darken (identity)
      else
        v_over := s_r2_iris - v_rad2;
        -- K6 edge band: wider band -> softer; scale overflow into a 10-bit t
        s_iris_t <= resize(shift_right(v_over, 8 + to_integer(s_k_edge(9 downto 8))), 10);
      end if;
    end if;
  end process p_iris_gain;

  u_iris : entity work.interpolator_u
    generic map (G_WIDTH => 10, G_FRAC_BITS => 10, G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
    port map (clk => clk, enable => '1',
              a => s_yR, b => to_unsigned(64, 10), t => s_iris_t, result => s_yI, valid => open);

  ------------------------------------------------------------------------
  -- TRAIL: motion-history ghost (T11). t=0 when off -> identity.
  ------------------------------------------------------------------------
  p_trail : process (clk)
  begin
    if rising_edge(clk) then
      s_trail_b <= unsigned(s_mh_rd) & "00";                   -- ghost luma
      if s_t_trail = '1' then
        s_trail_t <= (unsigned(s_mh_rd) & "00");               -- alpha from staleness
      else
        s_trail_t <= (others => '0');
      end if;
    end if;
  end process p_trail;

  u_trail : entity work.interpolator_u
    generic map (G_WIDTH => 10, G_FRAC_BITS => 10, G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
    port map (clk => clk, enable => '1',
              a => s_yI, b => s_trail_b, t => s_trail_t, result => s_yT, valid => open);

  ------------------------------------------------------------------------
  -- STAMP: sprite composite (K5). t=0 when no stamps -> identity.
  ------------------------------------------------------------------------
  p_stampsel : process (clk)
  begin
    if rising_edge(clk) then
      s_stamp_b <= to_unsigned(940, 10);                       -- bright stamp luma
      if s_k_orbit(9) = '1' and s_stamp_hit = '1' then
        s_stamp_t <= s_stamp_alpha & "00";
      else
        s_stamp_t <= (others => '0');
      end if;
    end if;
  end process p_stampsel;

  u_stamp_c : entity work.interpolator_u
    generic map (G_WIDTH => 10, G_FRAC_BITS => 10, G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
    port map (clk => clk, enable => '1',
              a => s_yT, b => s_stamp_b, t => s_stamp_t, result => s_yS, valid => open);

  ------------------------------------------------------------------------
  -- ELLIPSE overlay (T9) - 3 registers; coarse outline test vs second-moment axes
  ------------------------------------------------------------------------
  p_ellipse : process (clk)
  begin
    if rising_edge(clk) then
      s_r2_ell <= s_r2_iris;
      -- axis thresholds already arrive in r^2 units -> direct compare, no multiply
      if s_t_ell = '1' and s_r2_ell >= s_ell_min2 and s_r2_ell <= s_ell_maj2 then
        s_yE <= to_unsigned(900, 10);                          -- outline band
      else
        s_yE <= s_yS;
      end if;
    end if;
  end process p_ellipse;

  ------------------------------------------------------------------------
  -- U/V wet carry (align PULL-out U/V to the fader input)
  ------------------------------------------------------------------------
  p_uvcarry : process (clk)
  begin
    if rising_edge(clk) then
      s_uv_u(0) <= s_uP;
      s_uv_v(0) <= s_vP;
      for i in 1 to C_UV_DELAY - 1 loop
        s_uv_u(i) <= s_uv_u(i - 1);
        s_uv_v(i) <= s_uv_v(i - 1);
      end loop;
    end if;
  end process p_uvcarry;
  s_wetU <= s_uv_u(C_UV_DELAY - 1);
  s_wetV <= s_uv_v(C_UV_DELAY - 1);

  ------------------------------------------------------------------------
  -- SYNC + DRY delay (34b x 64 @ 32) + 4 fader-companion FFs -> exactly 36
  ------------------------------------------------------------------------
  s_sd_in <= s_y_in & s_u_in & s_v_in & s_hs_n & s_vs_n & s_fld_n & s_avid;
  u_syncdry : entity work.variable_delay_u
    generic map (G_WIDTH => 34, G_DEPTH => 6)
    port map (clk => clk, enable => '1', delay => to_unsigned(C_SYNCDRY_TOTAL - 2, 6),
              a => s_sd_in, result => s_sd_out, valid => open);

  s_dryY <= s_sd_out(33 downto 24);
  s_dryU <= s_sd_out(23 downto 14);
  s_dryV <= s_sd_out(13 downto 4);

  p_synccarry : process (clk)
  begin
    if rising_edge(clk) then
      s_sync_pipe(0) <= std_logic_vector(s_sd_out(3 downto 0));  -- {hs,vs,fld,avid} @32
      for i in 1 to C_FADER_LAT - 1 loop
        s_sync_pipe(i) <= s_sync_pipe(i - 1);
      end loop;
    end if;
  end process p_synccarry;

  ------------------------------------------------------------------------
  -- FADER: 3x interpolator (a=dry, b=wet, t=fader) with R1 broadcast clamp
  ------------------------------------------------------------------------
  u_fader_y : entity work.interpolator_u
    generic map (G_WIDTH => 10, G_FRAC_BITS => 10, G_OUTPUT_MIN => 64, G_OUTPUT_MAX => 940)
    port map (clk => clk, enable => '1',
              a => s_dryY, b => s_yE, t => s_fader, result => s_faderY, valid => open);

  u_fader_u : entity work.interpolator_u
    generic map (G_WIDTH => 10, G_FRAC_BITS => 10, G_OUTPUT_MIN => 64, G_OUTPUT_MAX => 960)
    port map (clk => clk, enable => '1',
              a => s_dryU, b => s_wetU, t => s_fader, result => s_faderU, valid => open);

  u_fader_v : entity work.interpolator_u
    generic map (G_WIDTH => 10, G_FRAC_BITS => 10, G_OUTPUT_MIN => 64, G_OUTPUT_MAX => 960)
    port map (clk => clk, enable => '1',
              a => s_dryV, b => s_wetV, t => s_fader, result => s_faderV, valid => open);

  ------------------------------------------------------------------------
  -- data_out : each field driven exactly once (R2). Sync delayed EXACTLY 36 (R5/R8).
  ------------------------------------------------------------------------
  data_out.y       <= std_logic_vector(s_faderY);
  data_out.u       <= std_logic_vector(s_faderU);
  data_out.v       <= std_logic_vector(s_faderV);
  data_out.hsync_n <= s_sync_pipe(C_FADER_LAT - 1)(3);
  data_out.vsync_n <= s_sync_pipe(C_FADER_LAT - 1)(2);
  data_out.field_n <= s_sync_pipe(C_FADER_LAT - 1)(1);
  data_out.avid    <= s_sync_pipe(C_FADER_LAT - 1)(0);

end architecture witness;
