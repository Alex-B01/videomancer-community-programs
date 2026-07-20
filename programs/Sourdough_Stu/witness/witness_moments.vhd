-- Copyright (C) 2026 AB (Sourdough_Stu)
-- SPDX-License-Identifier: GPL-3.0-only
--
-- witness_moments.vhd - THUMBNAIL-SPACE moments / centroid gaze engine
--                       (born in WITNESS; REV-C, D3 committed architecture)
--
-- REV-C rework (the Gate-1 smoke fired: REV-B full-res 42-48b accumulators + a
-- 48b divider measured 3,370 LC and would not place). Moments are now computed
-- in THUMBNAIL SPACE on a 64x32 = 2048-cell mass store:
--
--   ACTIVE raster: the T7/hue-selected weight  w = base + (sat>>C_SAT_SHIFT)  is
--     folded into the Bresenham-addressed cell of an internal 64x32x8b MASS
--     THUMBNAIL (saturating running accumulate, cleared each frame by the scan
--     -> intra-frame, all-formats-clean). ZERO datapath latency.
--   VBLANK: a bounded FSM SCANS the 2048 cells (<< NTSC's ~33.5k) computing the
--     image moments M00/M10/M01 (+ a block-resolution ellipse) MULTIPLY-FREE via
--     nested running sums untangled by SHIFTS (grid = 64x32, powers of two:
--     rowM10 = (rowM00<<6) - Sigma colrun ;  M01 = (M00<<5) - Sigma rowrun). The
--     small x in [0,63] / y in [0,31] operands collapse the accumulators + divider
--     vs REV-B's full-res 42b hardware. ONE shared multiplier_s + one serial
--     divider (the §3 ledger) then: divide -> block centroid; scale to live pixels
--     by the measured active width/height; EVADE + rate-normalized LAG; cx^2/cy^2
--     seeds for the coordinate engine; k*cx / k*cy seeds for the RINGS second-
--     difference DDA (D4); block-resolution ellipse (T9). All OFF the pixel path.
--
-- Internal store follows the video_line_buffer.vhd inference pattern (no init,
-- registered read) -> Yosys `found RAM` (hook 6 asserts the mass thumbnail).
--
-- NOTE (Gate-0, REV-C): the thumbnail-scan STRUCTURE + narrow widths are faithful
-- (they drive the measured LC); the exact fixed-point scaling + ellipse precision
-- are the sim-verifier's Gate-2 tuning target.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.rate_normalizer_pkg.all;

entity witness_moments is
  port (
    clk         : in  std_logic;
    active      : in  std_logic;
    cell_addr   : in  unsigned(10 downto 0);        -- Bresenham cell (y*64 + x)
    weight      : in  unsigned(11 downto 0);        -- w = base + (sat>>shift)
    frame_end   : in  std_logic;                    -- start vblank scan/solve
    active_w    : in  unsigned(11 downto 0);        -- measured active width  (px)
    active_h    : in  unsigned(11 downto 0);        -- measured active height (lines)
    t8_evade    : in  std_logic;
    t9_ellipse  : in  std_logic;
    k2_rings    : in  unsigned(9 downto 0);
    lag_shift   : in  t_shift;
    lag_shift_b : in  t_shift;
    lag_use_b   : in  std_logic;
    cx_lag      : out signed(12 downto 0);
    cy_lag      : out signed(12 downto 0);
    cx2         : out unsigned(23 downto 0);
    cy2         : out unsigned(23 downto 0);
    ell_maj2    : out unsigned(23 downto 0);
    ell_min2    : out unsigned(23 downto 0);
    ell_tilt    : out signed(2 downto 0);
    ring_phase0 : out signed(23 downto 0);
    ring_dpx0   : out signed(23 downto 0);
    ring_dpy0   : out signed(23 downto 0);
    ring_twok   : out signed(23 downto 0);
    solve_done  : out std_logic
  );
end entity witness_moments;

architecture rtl of witness_moments is

  constant C_MW    : integer := 26;
  constant C_MLAT  : integer := (C_MW + 1) / 2 + 3;
  signal s_mul_en  : std_logic := '0';
  signal s_mul_x   : signed(C_MW - 1 downto 0) := (others => '0');
  signal s_mul_y   : signed(C_MW - 1 downto 0) := (others => '0');
  signal s_mul_z   : signed(C_MW - 1 downto 0) := (others => '0');
  signal s_mul_r   : signed(C_MW - 1 downto 0);
  signal s_mul_v   : std_logic;

  -- internal mass thumbnail store (64x32x8, no init -> found RAM)
  signal s_m_rd_addr : unsigned(10 downto 0) := (others => '0');
  signal s_m_wr_addr : unsigned(10 downto 0) := (others => '0');
  signal s_m_wr_data : std_logic_vector(7 downto 0) := (others => '0');
  signal s_m_wr_en   : std_logic := '0';
  signal s_m_rd_data : std_logic_vector(7 downto 0);

  -- active-path RMW pipeline
  signal s_active_d  : std_logic := '0';
  signal s_rmw_addr  : unsigned(10 downto 0) := (others => '0');
  signal s_rmw_w     : unsigned(11 downto 0) := (others => '0');
  constant C_MASS_SHIFT : integer := 6;

  -- vblank scan state
  signal s_scan_i   : unsigned(11 downto 0) := (others => '0');   -- 0..2048
  signal s_acc_vld  : std_logic := '0';
  signal s_acc_i    : unsigned(10 downto 0) := (others => '0');

  -- narrow nested accumulators (thumbnail space)
  signal s_M00    : unsigned(19 downto 0) := (others => '0');
  signal s_M10    : unsigned(25 downto 0) := (others => '0');
  signal s_M01    : unsigned(25 downto 0) := (others => '0');
  signal s_colrun : unsigned(14 downto 0) := (others => '0');
  signal s_rowM00 : unsigned(14 downto 0) := (others => '0');
  signal s_rowXV  : unsigned(20 downto 0) := (others => '0');
  signal s_rowrun : unsigned(19 downto 0) := (others => '0');
  signal s_M01acc : unsigned(24 downto 0) := (others => '0');
  signal s_minx, s_maxx : unsigned(5 downto 0) := (others => '0');
  signal s_miny, s_maxy : unsigned(4 downto 0) := (others => '0');

  -- serial restoring divider (narrow)
  constant C_DW   : integer := 26;
  signal s_dvd    : unsigned(C_DW - 1 downto 0) := (others => '0');
  signal s_dsor   : unsigned(C_DW - 1 downto 0) := (others => '0');
  signal s_rem    : unsigned(C_DW - 1 downto 0) := (others => '0');
  signal s_quo    : unsigned(C_DW - 1 downto 0) := (others => '0');
  signal s_dbit   : integer range 0 to C_DW := 0;

  -- gaze working state
  signal s_cxb, s_cyb   : unsigned(6 downto 0) := (others => '0');
  signal s_cxp, s_cyp   : signed(12 downto 0) := (others => '0');
  signal s_cx_lag, s_cy_lag : signed(12 downto 0) := (others => '0');
  signal s_kcxr         : signed(23 downto 0) := (others => '0');

  type t_state is (S_IDLE, S_SCAN, S_FIN01, S_DIVCX, S_DIVCY,
                   S_MUL, S_SCX, S_SCY, S_LAG,
                   S_CX2, S_CY2, S_KCX, S_KCY, S_RING, S_ELL, S_DONE);
  signal s_state : t_state := S_IDLE;
  signal s_ret   : t_state := S_IDLE;
  signal s_wait  : integer range 0 to 31 := 0;

  function sat8(a : unsigned) return std_logic_vector is
  begin
    if a > 255 then return x"FF"; else return std_logic_vector(a(7 downto 0)); end if;
  end function;

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
      rem_s <= v_rem - dsor_s;
      quo_s(bit_s - 1) <= '1';
    else
      rem_s <= v_rem;
    end if;
    bit_s <= bit_s - 1;
  end procedure;

begin

  u_mass : entity work.witness_motion_history
    port map (clk => clk, wr_en => s_m_wr_en, wr_addr => s_m_wr_addr, wr_data => s_m_wr_data,
              rd_addr => s_m_rd_addr, rd_data => s_m_rd_data);

  u_mul : entity work.multiplier_s
    generic map (G_WIDTH => C_MW, G_FRAC_BITS => 0,
                 G_OUTPUT_MIN => -(2 ** (C_MW - 1)), G_OUTPUT_MAX => (2 ** (C_MW - 1)) - 1)
    port map (clk => clk, enable => s_mul_en, x => s_mul_x, y => s_mul_y, z => s_mul_z,
              result => s_mul_r, valid => s_mul_v);

  cx_lag    <= s_cx_lag;
  cy_lag    <= s_cy_lag;
  ring_twok <= shift_left(resize(signed('0' & k2_rings), 24), 1);

  p_solve : process (clk)
    variable v_v      : unsigned(7 downto 0);
    variable v_x      : unsigned(5 downto 0);
    variable v_y      : unsigned(4 downto 0);
    variable v_colrun : unsigned(14 downto 0);
    variable v_rowM00 : unsigned(14 downto 0);
    variable v_rowXV  : unsigned(20 downto 0);
    variable v_rowrun : unsigned(19 downto 0);
    variable v_m00s   : unsigned(25 downto 0);
    variable v_delta, v_step, v_tgt : signed(13 downto 0);
    variable v_ex     : unsigned(5 downto 0);
    variable v_ey     : unsigned(4 downto 0);
  begin
    if rising_edge(clk) then
      s_mul_en   <= '0';
      solve_done <= '0';
      s_m_wr_en  <= '0';
      s_active_d <= active;
      s_rmw_addr <= cell_addr;
      s_rmw_w    <= weight;

      case s_state is
        ----------------------------------------------------------------
        when S_IDLE =>
          -- ACTIVE mass RMW: saturating-accumulate weight into the read cell
          s_m_rd_addr <= cell_addr;
          if s_active_d = '1' then
            s_m_wr_addr <= s_rmw_addr;
            s_m_wr_data <= sat8(resize(unsigned(s_m_rd_data), 20)
                                + resize(shift_right(s_rmw_w, C_MASS_SHIFT), 20));
            s_m_wr_en   <= '1';
          end if;
          if frame_end = '1' then
            s_M00 <= (others => '0'); s_M10 <= (others => '0'); s_M01 <= (others => '0');
            s_colrun <= (others => '0'); s_rowM00 <= (others => '0'); s_rowXV <= (others => '0');
            s_rowrun <= (others => '0'); s_M01acc <= (others => '0');
            s_minx <= (others => '1'); s_maxx <= (others => '0');
            s_miny <= (others => '1'); s_maxy <= (others => '0');
            s_scan_i <= (others => '0'); s_acc_vld <= '0';
            s_state  <= S_SCAN;
          end if;

        ----------------------------------------------------------------
        when S_SCAN =>
          s_m_rd_addr <= s_scan_i(10 downto 0);   -- present next read
          s_acc_i     <= s_scan_i(10 downto 0);
          s_acc_vld   <= '1';

          if s_acc_vld = '1' then
            v_v := unsigned(s_m_rd_data);
            v_x := s_acc_i(5 downto 0);
            v_y := s_acc_i(10 downto 6);
            -- clear the cell just read
            s_m_wr_addr <= s_acc_i; s_m_wr_data <= (others => '0'); s_m_wr_en <= '1';
            -- nested running sums (variables hold the final per-cell values)
            if v_x = 0 then
              v_colrun := resize(v_v, 15);
              v_rowM00 := resize(v_v, 15);
              v_rowXV  := resize(v_v, 21);
            else
              v_colrun := s_colrun + resize(v_v, 15);
              v_rowM00 := s_rowM00 + resize(v_v, 15);
              v_rowXV  := s_rowXV + resize(v_colrun, 21);
            end if;
            s_colrun <= v_colrun; s_rowM00 <= v_rowM00; s_rowXV <= v_rowXV;
            s_M00    <= s_M00 + resize(v_v, 20);
            if v_v /= 0 then
              if v_x < s_minx then s_minx <= v_x; end if;
              if v_x > s_maxx then s_maxx <= v_x; end if;
              if v_y < s_miny then s_miny <= v_y; end if;
              if v_y > s_maxy then s_maxy <= v_y; end if;
            end if;
            if v_x = 63 then    -- end of row: fold horizontal, advance vertical
              s_M10    <= s_M10 + shift_left(resize(v_rowM00, 26), 6) - resize(v_rowXV, 26);
              v_rowrun := s_rowrun + resize(v_rowM00, 20);
              s_rowrun <= v_rowrun;
              s_M01acc <= s_M01acc + resize(v_rowrun, 25);
            end if;
          end if;

          if s_scan_i = 2048 then
            s_state <= S_FIN01;
          else
            s_scan_i <= s_scan_i + 1;
          end if;

        when S_FIN01 =>
          -- M01 = (M00<<5) - M01acc ; prime the cx divide (M10 / M00)
          v_m00s := shift_left(resize(s_M00, 26), 5);
          if v_m00s >= resize(s_M01acc, 26) then
            s_M01 <= v_m00s - resize(s_M01acc, 26);
          else
            s_M01 <= (others => '0');
          end if;
          s_dvd  <= resize(s_M10, C_DW);
          s_dsor <= resize(s_M00, C_DW);
          s_rem  <= (others => '0'); s_quo <= (others => '0'); s_dbit <= C_DW;
          s_state <= S_DIVCX;

        when S_DIVCX =>
          if s_dbit = 0 then
            s_cxb  <= resize(s_quo(6 downto 0), 7);
            s_dvd  <= resize(s_M01, C_DW);
            s_dsor <= resize(s_M00, C_DW);
            s_rem  <= (others => '0'); s_quo <= (others => '0'); s_dbit <= C_DW;
            s_state <= S_DIVCY;
          else
            div_step(s_rem, s_quo, s_dvd, s_dsor, s_dbit);
          end if;

        when S_DIVCY =>
          if s_dbit = 0 then
            s_cyb   <= resize(s_quo(6 downto 0), 7);
            s_state <= S_SCX;
          else
            div_step(s_rem, s_quo, s_dvd, s_dsor, s_dbit);
          end if;

        ----------------------------------------------------------------
        when S_MUL =>
          if s_wait = 0 then s_state <= s_ret; else s_wait <= s_wait - 1; end if;

        when S_SCX =>                 -- cx_pix = (cxb * active_w) >> 6
          s_mul_x <= resize(signed('0' & s_cxb), C_MW);
          s_mul_y <= resize(signed('0' & active_w), C_MW);
          s_mul_en <= '1'; s_wait <= C_MLAT; s_ret <= S_SCY; s_state <= S_MUL;

        when S_SCY =>
          s_cxp   <= resize(shift_right(s_mul_r, 6), 13);
          s_mul_x <= resize(signed('0' & s_cyb), C_MW);
          s_mul_y <= resize(signed('0' & active_h), C_MW);
          s_mul_en <= '1'; s_wait <= C_MLAT; s_ret <= S_LAG; s_state <= S_MUL;

        when S_LAG =>
          s_cyp <= resize(shift_right(s_mul_r, 5), 13);
          if t8_evade = '1' then v_tgt := shift_left(resize(s_cx_lag, 14), 1) - resize(s_cxp, 14);
          else                   v_tgt := resize(s_cxp, 14); end if;
          v_delta := v_tgt - resize(s_cx_lag, 14);
          v_step  := shift_right(v_delta, lag_shift);
          if lag_use_b = '1' then v_step := v_step + shift_right(v_delta, lag_shift_b); end if;
          s_cx_lag <= resize(resize(s_cx_lag, 14) + v_step, 13);
          if t8_evade = '1' then v_tgt := shift_left(resize(s_cy_lag, 14), 1) - resize(s_cyp, 14);
          else                   v_tgt := resize(s_cyp, 14); end if;
          v_delta := v_tgt - resize(s_cy_lag, 14);
          v_step  := shift_right(v_delta, lag_shift);
          if lag_use_b = '1' then v_step := v_step + shift_right(v_delta, lag_shift_b); end if;
          s_cy_lag <= resize(resize(s_cy_lag, 14) + v_step, 13);
          s_state  <= S_CX2;

        when S_CX2 =>
          s_mul_x <= resize(s_cx_lag, C_MW); s_mul_y <= resize(s_cx_lag, C_MW);
          s_mul_en <= '1'; s_wait <= C_MLAT; s_ret <= S_CY2; s_state <= S_MUL;

        when S_CY2 =>
          cx2 <= unsigned(s_mul_r(23 downto 0));
          s_mul_x <= resize(s_cy_lag, C_MW); s_mul_y <= resize(s_cy_lag, C_MW);
          s_mul_en <= '1'; s_wait <= C_MLAT; s_ret <= S_KCX; s_state <= S_MUL;

        when S_KCX =>
          cy2 <= unsigned(s_mul_r(23 downto 0));
          s_mul_x <= resize(signed('0' & k2_rings), C_MW); s_mul_y <= resize(s_cx_lag, C_MW);
          s_mul_en <= '1'; s_wait <= C_MLAT; s_ret <= S_KCY; s_state <= S_MUL;

        when S_KCY =>
          s_kcxr <= resize(s_mul_r, 24);   -- k*cx
          s_mul_x <= resize(signed('0' & k2_rings), C_MW); s_mul_y <= resize(s_cy_lag, C_MW);
          s_mul_en <= '1'; s_wait <= C_MLAT; s_ret <= S_RING; s_state <= S_MUL;

        when S_RING =>                    -- s_mul_r = k*cy
          ring_phase0 <= (others => '0');
          ring_dpx0   <= resize(signed('0' & k2_rings), 24) - shift_left(s_kcxr, 1);
          ring_dpy0   <= resize(signed('0' & k2_rings), 24) - shift_left(resize(s_mul_r, 24), 1);
          s_state     <= S_ELL;

        when S_ELL =>
          if t9_ellipse = '1' and s_maxx >= s_minx and s_maxy >= s_miny then
            v_ex := s_maxx - s_minx;
            v_ey := s_maxy - s_miny;
            ell_maj2 <= shift_left(resize(v_ex, 24), 12);
            ell_min2 <= shift_left(resize(v_ey, 24), 12);
            if    v_ex > v_ey then ell_tilt <= to_signed(1, 3);
            elsif v_ex < v_ey then ell_tilt <= to_signed(-1, 3);
            else                   ell_tilt <= to_signed(0, 3); end if;
          else
            ell_maj2 <= (others => '0'); ell_min2 <= (others => '0'); ell_tilt <= (others => '0');
          end if;
          s_state <= S_DONE;

        when S_DONE =>
          solve_done <= '1';
          s_state    <= S_IDLE;
      end case;
    end if;
  end process p_solve;

end architecture rtl;
