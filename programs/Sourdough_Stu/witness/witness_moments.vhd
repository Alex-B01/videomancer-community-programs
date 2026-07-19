-- Copyright (C) 2026 AB (Sourdough_Stu)
-- SPDX-License-Identifier: GPL-3.0-only
--
-- witness_moments.vhd - moments / centroid gaze engine (born in WITNESS, D3/D7/D8)
--
-- Two kinds of work on one clock:
--   (1) ACTIVE  (per pixel / per line): nested running-sum accumulators build the
--       zeroth and first image moments (M00, M10, M01) MULTIPLY-FREE, plus the
--       second-moment nests (M20, M02, M11) for the ellipse. The per-pixel path
--       has ZERO multiplies (adds only) -- H4-clean -- and every accumulate is
--       SATURATING (R3: no overflow wrap).
--   (2) VBLANK  (following the frame): a bounded FSM drives ONE time-shared
--       multiplier_s (entity work.multiplier_s -- the single vblank multiplier of
--       the §3.3 ledger) and one serial restoring divider, OFF the pixel critical
--       path (H5 -> ÷1 full-clock holds), producing:
--          raw centroid  cx = M10/M00,  cy = M01/M00
--          EVADE (T8) target reflection, then rate-normalized 1-pole LAG (K3)
--          cx^2, cy^2 seeds for the coordinate engine
--          second-moment ellipse major/minor/tilt (T9)
--       The solve completes in a few-hundred serial clocks, far inside the
--       smallest (NTSC ~33.5k) vblank (hook 4).
--
-- NOTE (Gate-0 PAPER, REV-B): the accumulator widths + shared-multiplier /
-- serial-divider STRUCTURE are faithful (they drive the measured LC/FF); the exact
-- fixed-point untangling of the nested sums and the eigen-solve precision are the
-- sim-verifier's Gate-2 tuning target. Multiply-free per-pixel + one vblank
-- multiplier + serial divide is the invariant preserved here.
--
-- Resource: wide accumulators (0 EBR) + 1 multiplier_s + serial div/sqrt.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.rate_normalizer_pkg.all;

entity witness_moments is
  port (
    clk        : in  std_logic;
    active     : in  std_logic;                    -- avid (active pixel)
    new_line   : in  std_logic;                    -- first active pixel of a line
    frame_end  : in  std_logic;                    -- end-of-active -> start vblank solve
    weight     : in  unsigned(11 downto 0);        -- mass weight w = base + (sat>>shift)
    t8_evade   : in  std_logic;
    t9_ellipse : in  std_logic;
    lag_shift  : in  t_shift;                       -- rate-normalized LAG primary shift
    lag_shift_b: in  t_shift;                       -- rate-normalized LAG secondary shift
    lag_use_b  : in  std_logic;
    cx_lag     : out signed(12 downto 0);
    cy_lag     : out signed(12 downto 0);
    cx2        : out unsigned(23 downto 0);
    cy2        : out unsigned(23 downto 0);
    ell_maj2   : out unsigned(23 downto 0);   -- major-axis threshold in r^2 units
    ell_min2   : out unsigned(23 downto 0);   -- minor-axis threshold in r^2 units
    ell_tilt   : out signed(2 downto 0);
    solve_done : out std_logic
  );
end entity witness_moments;

architecture rtl of witness_moments is

  -- shared vblank multiplier (the ONE multiplier_s of the ledger)
  constant C_MW    : integer := 26;
  constant C_MLAT  : integer := (C_MW + 1) / 2 + 3;   -- data latency
  signal s_mul_en  : std_logic := '0';
  signal s_mul_x   : signed(C_MW - 1 downto 0) := (others => '0');
  signal s_mul_y   : signed(C_MW - 1 downto 0) := (others => '0');
  signal s_mul_z   : signed(C_MW - 1 downto 0) := (others => '0');
  signal s_mul_r   : signed(C_MW - 1 downto 0);
  signal s_mul_v   : std_logic;

  -- live nested accumulators (SATURATING). Widths follow the §3.1/§3.2 ledger.
  signal s_M00    : unsigned(30 downto 0) := (others => '0');   -- Sigma w
  signal s_colrun : unsigned(21 downto 0) := (others => '0');   -- prefix Sigma w in row
  signal s_M10    : unsigned(41 downto 0) := (others => '0');   -- Sigma_x colrun
  signal s_rowrun : unsigned(30 downto 0) := (others => '0');   -- Sigma_lines rowsum
  signal s_M01    : unsigned(41 downto 0) := (others => '0');   -- Sigma_y rowrun
  signal s_M20    : unsigned(47 downto 0) := (others => '0');   -- second-moment nest (x)
  signal s_M02    : unsigned(43 downto 0) := (others => '0');   -- second-moment nest (y)
  signal s_M11    : unsigned(43 downto 0) := (others => '0');   -- cross nest
  signal s_col2   : unsigned(31 downto 0) := (others => '0');   -- Sigma colrun (for M20 nest)

  -- frame snapshots (latched at frame_end)
  signal s_sM00   : unsigned(30 downto 0) := (others => '0');
  signal s_sM10   : unsigned(41 downto 0) := (others => '0');
  signal s_sM01   : unsigned(41 downto 0) := (others => '0');
  signal s_sM20   : unsigned(47 downto 0) := (others => '0');
  signal s_sM02   : unsigned(43 downto 0) := (others => '0');
  signal s_width  : unsigned(11 downto 0) := to_unsigned(1920, 12);
  signal s_wcnt   : unsigned(11 downto 0) := (others => '0');

  -- serial restoring divider (shared across the centroid divides)
  constant C_DW   : integer := 48;
  signal s_dvd    : unsigned(C_DW - 1 downto 0) := (others => '0');
  signal s_dsor   : unsigned(C_DW - 1 downto 0) := (others => '0');
  signal s_rem    : unsigned(C_DW - 1 downto 0) := (others => '0');
  signal s_quo    : unsigned(C_DW - 1 downto 0) := (others => '0');
  signal s_dbit   : integer range 0 to C_DW := 0;

  -- gaze / ellipse working state
  signal s_cx_raw : signed(12 downto 0) := (others => '0');
  signal s_cy_raw : signed(12 downto 0) := (others => '0');
  signal s_cx_lag : signed(12 downto 0) := (others => '0');
  signal s_cy_lag : signed(12 downto 0) := (others => '0');

  type t_state is (S_IDLE, S_SETUP, S_DIV_CX, S_DIV_CY, S_LAG,
                   S_MUL_CX2, S_WAIT_CX2, S_MUL_CY2, S_WAIT_CY2,
                   S_ELL, S_DONE);
  signal s_state  : t_state := S_IDLE;
  signal s_wait   : integer range 0 to 31 := 0;

  -- saturating unsigned add (widens, clamps to a's width)
  function sat_add(a : unsigned; b : unsigned) return unsigned is
    variable v : unsigned(a'length downto 0);
  begin
    v := ('0' & a) + resize(b, a'length + 1);
    if v(v'high) = '1' then
      return (a'range => '1');
    else
      return v(a'high downto 0);
    end if;
  end function;

  -- one restoring-division iteration on signal state
  procedure div_step(
      signal rem_s  : inout unsigned(C_DW - 1 downto 0);
      signal quo_s  : inout unsigned(C_DW - 1 downto 0);
      signal dvd_s  : in    unsigned(C_DW - 1 downto 0);
      signal dsor_s : in    unsigned(C_DW - 1 downto 0);
      signal bit_s  : inout integer range 0 to C_DW) is
    variable v_rem : unsigned(C_DW - 1 downto 0);
  begin
    v_rem := rem_s(C_DW - 2 downto 0) & dvd_s(bit_s - 1);
    if v_rem >= dsor_s then
      rem_s            <= v_rem - dsor_s;
      quo_s(bit_s - 1) <= '1';
    else
      rem_s <= v_rem;
    end if;
    bit_s <= bit_s - 1;
  end procedure;

begin

  u_mul : entity work.multiplier_s
    generic map (
      G_WIDTH => C_MW, G_FRAC_BITS => 0,
      G_OUTPUT_MIN => -(2 ** (C_MW - 1)), G_OUTPUT_MAX => (2 ** (C_MW - 1)) - 1
    )
    port map (
      clk => clk, enable => s_mul_en,
      x => s_mul_x, y => s_mul_y, z => s_mul_z,
      result => s_mul_r, valid => s_mul_v
    );

  cx_lag <= s_cx_lag;
  cy_lag <= s_cy_lag;

  --------------------------------------------------------------------------
  -- ACTIVE: nested running-sum accumulators (multiply-free, saturating)
  --------------------------------------------------------------------------
  p_accum : process (clk)
  begin
    if rising_edge(clk) then
      if new_line = '1' then
        s_rowrun <= sat_add(s_rowrun, resize(s_colrun, 31));
        s_M01    <= sat_add(s_M01, resize(s_rowrun, 42));
        s_colrun <= (others => '0');
        s_col2   <= (others => '0');
        if s_wcnt /= 0 then s_width <= s_wcnt; end if;
        s_wcnt   <= (others => '0');
      elsif active = '1' then
        s_M00    <= sat_add(s_M00, resize(weight, 31));
        s_colrun <= sat_add(s_colrun, resize(weight, 22));
        s_M10    <= sat_add(s_M10, resize(s_colrun, 42));
        s_col2   <= sat_add(s_col2, resize(s_colrun, 32));
        s_M20    <= sat_add(s_M20, resize(s_col2, 48));
        s_M02    <= sat_add(s_M02, resize(s_rowrun, 44));
        s_M11    <= sat_add(s_M11, resize(s_colrun, 44));
        s_wcnt   <= s_wcnt + 1;
      end if;

      if frame_end = '1' then
        s_sM00 <= s_M00; s_sM10 <= s_M10; s_sM01 <= s_M01;
        s_sM20 <= s_M20; s_sM02 <= s_M02;
        s_M00 <= (others => '0'); s_colrun <= (others => '0'); s_M10 <= (others => '0');
        s_rowrun <= (others => '0'); s_M01 <= (others => '0'); s_col2 <= (others => '0');
        s_M20 <= (others => '0'); s_M02 <= (others => '0'); s_M11 <= (others => '0');
      end if;
    end if;
  end process p_accum;

  --------------------------------------------------------------------------
  -- VBLANK FSM: centroid divide -> EVADE/LAG -> squares -> ellipse
  --------------------------------------------------------------------------
  p_solve : process (clk)
    variable v_delta : signed(13 downto 0);
    variable v_step  : signed(13 downto 0);
    variable v_tgt   : signed(13 downto 0);
    variable v_disc  : signed(C_MW - 1 downto 0);
  begin
    if rising_edge(clk) then
      s_mul_en   <= '0';
      solve_done <= '0';

      case s_state is
        when S_IDLE =>
          if frame_end = '1' then
            s_state <= S_SETUP;         -- next cycle the snapshots are valid
          end if;

        when S_SETUP =>
          s_dvd   <= resize(s_sM10, C_DW);
          s_dsor  <= resize(s_sM00, C_DW);
          s_rem   <= (others => '0');
          s_quo   <= (others => '0');
          s_dbit  <= C_DW;
          s_state <= S_DIV_CX;

        when S_DIV_CX =>
          if s_dbit = 0 then
            s_cx_raw <= signed(resize(s_quo(12 downto 0), 13));
            s_dvd    <= resize(s_sM01, C_DW);
            s_dsor   <= resize(s_sM00, C_DW);
            s_rem    <= (others => '0');
            s_quo    <= (others => '0');
            s_dbit   <= C_DW;
            s_state  <= S_DIV_CY;
          else
            div_step(s_rem, s_quo, s_dvd, s_dsor, s_dbit);
          end if;

        when S_DIV_CY =>
          if s_dbit = 0 then
            s_cy_raw <= signed(resize(s_quo(12 downto 0), 13));
            s_state  <= S_LAG;
          else
            div_step(s_rem, s_quo, s_dvd, s_dsor, s_dbit);
          end if;

        when S_LAG =>
          -- EVADE (T8): target = reflect centroid across current gaze; else track
          if t8_evade = '1' then
            v_tgt := shift_left(resize(s_cx_lag, 14), 1) - resize(s_cx_raw, 14);
          else
            v_tgt := resize(s_cx_raw, 14);
          end if;
          v_delta := v_tgt - resize(s_cx_lag, 14);
          v_step  := shift_right(v_delta, lag_shift);
          if lag_use_b = '1' then v_step := v_step + shift_right(v_delta, lag_shift_b); end if;
          s_cx_lag <= resize(resize(s_cx_lag, 14) + v_step, 13);

          if t8_evade = '1' then
            v_tgt := shift_left(resize(s_cy_lag, 14), 1) - resize(s_cy_raw, 14);
          else
            v_tgt := resize(s_cy_raw, 14);
          end if;
          v_delta := v_tgt - resize(s_cy_lag, 14);
          v_step  := shift_right(v_delta, lag_shift);
          if lag_use_b = '1' then v_step := v_step + shift_right(v_delta, lag_shift_b); end if;
          s_cy_lag <= resize(resize(s_cy_lag, 14) + v_step, 13);

          s_state  <= S_MUL_CX2;

        when S_MUL_CX2 =>
          s_mul_x  <= resize(s_cx_lag, C_MW);
          s_mul_y  <= resize(s_cx_lag, C_MW);
          s_mul_en <= '1';
          s_wait   <= C_MLAT;
          s_state  <= S_WAIT_CX2;

        when S_WAIT_CX2 =>
          if s_wait = 0 then
            cx2     <= unsigned(resize(s_mul_r(23 downto 0), 24));
            s_state <= S_MUL_CY2;
          else
            s_wait <= s_wait - 1;
          end if;

        when S_MUL_CY2 =>
          s_mul_x  <= resize(s_cy_lag, C_MW);
          s_mul_y  <= resize(s_cy_lag, C_MW);
          s_mul_en <= '1';
          s_wait   <= C_MLAT;
          s_state  <= S_WAIT_CY2;

        when S_WAIT_CY2 =>
          if s_wait = 0 then
            cy2     <= unsigned(resize(s_mul_r(23 downto 0), 24));
            s_state <= S_ELL;
          else
            s_wait <= s_wait - 1;
          end if;

        when S_ELL =>
          -- second-moment ellipse (T9). Bounded proxy: major/minor from the
          -- second-moment nests, tilt from the cross-nest imbalance. Gate-2
          -- replaces this with the full 2x2 eigen-solve.
          if t9_ellipse = '1' then
            -- second-moment nests are ~M00*variance -> take a scaled slice as the
            -- r^2-scale axis thresholds (no squaring here -> no extra multiply).
            v_disc := resize(signed('0' & s_sM20(46 downto 24)), C_MW)
                    - resize(signed('0' & s_sM02(42 downto 22)), C_MW);
            ell_maj2 <= unsigned(s_sM20(47 downto 24));
            ell_min2 <= unsigned(s_sM02(43 downto 20));
            if    v_disc > 0 then ell_tilt <= to_signed(1, 3);
            elsif v_disc < 0 then ell_tilt <= to_signed(-1, 3);
            else                  ell_tilt <= to_signed(0, 3); end if;
          else
            ell_maj2 <= (others => '0');
            ell_min2 <= (others => '0');
            ell_tilt <= (others => '0');
          end if;
          s_state <= S_DONE;

        when S_DONE =>
          solve_done <= '1';
          s_state    <= S_IDLE;
      end case;
    end if;
  end process p_solve;

end architecture rtl;
