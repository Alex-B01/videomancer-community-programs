-- Copyright (C) 2026 AB (Sourdough_Stu)
-- SPDX-License-Identifier: GPL-3.0-only
--
-- witness_moments.vhd - BLOCK-SPACE first-moment gaze engine (WITNESS Lite, REV-D, D3)
--
-- REV-D re-scope (two smokes over the die: REV-B 10,367, REV-C 9,838). The gaze is
-- now BLOCK-SPACE first-moment only, on ONE unified 64x32x8b mass/motion thumbnail:
--
--   ACTIVE raster: read the Bresenham-addressed cell (its running average), form
--     base = luma (T7=Brightness) or |luma - cell| (T7=Motion, the |current-average|
--     motion proxy), add the always-on self-nulling hue bias (sat>>C_SAT_SHIFT),
--     and fold w into the cell as a running average (EMA). ONE store serves both
--     weights (the separate motion-history store is gone with Trail). No init,
--     registered read -> Yosys `found RAM` (video_line_buffer.vhd pattern).
--   VBLANK: a small serial unit scans the 2048 cells for the FIRST MOMENTS ONLY
--     (M00, M10 = Sigma x*mass, M01 = Sigma y*mass) multiply-free via nested sums
--     untangled by shifts (grid 64x32 -> <<6 / <<5), divides -> BLOCK centroid
--     (cx_blk in [0,63], cy_blk in [0,31]). ONE shared multiplier_s does just 4
--     per-frame products: block->pixel scale (cx*active_w>>6, cy*active_h>>5) and
--     the coordinate-engine r^2 seeds (cx_px^2, cy_px^2 -- unavoidable for the
--     Euclidean r^2 of D3/§0). NO second moments, NO ellipse, NO RINGS/orbit
--     products -> the 6-product REV-C tree collapses to 4. EVADE + rate-normalized
--     LAG (G=2) render the gaze.
--
-- All off the pixel path (H5). NOTE (Gate-0 REV-D): the scan STRUCTURE + narrow
-- widths are faithful (they drive the LC); fixed-point scaling precision is the
-- sim-verifier's Gate-2 tuning target.

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
    luma        : in  unsigned(9 downto 0);         -- current pixel luma
    sat         : in  unsigned(10 downto 0);        -- hue-octant chroma saturation
    t7_track    : in  std_logic;                    -- '0' Brightness, '1' Motion
    frame_end   : in  std_logic;
    active_w    : in  unsigned(11 downto 0);
    active_h    : in  unsigned(11 downto 0);
    t8_evade    : in  std_logic;
    lag_shift   : in  t_shift;
    lag_shift_b : in  t_shift;
    lag_use_b   : in  std_logic;
    cx_lag      : out signed(12 downto 0);
    cy_lag      : out signed(12 downto 0);
    cx2         : out unsigned(23 downto 0);
    cy2         : out unsigned(23 downto 0);
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

  -- unified mass/motion thumbnail (64x32x8, no init -> found RAM)
  signal s_m_rd_addr : unsigned(10 downto 0) := (others => '0');
  signal s_m_wr_addr : unsigned(10 downto 0) := (others => '0');
  signal s_m_wr_data : std_logic_vector(7 downto 0) := (others => '0');
  signal s_m_wr_en   : std_logic := '0';
  signal s_m_rd_data : std_logic_vector(7 downto 0);

  -- active RMW pipeline (2-stage: A = weight from BRAM read; B = EMA + clamp + write,
  -- split so the BRAM-read -> write-data chain no longer bounds Fmax)
  signal s_active_d  : std_logic := '0';
  signal s_rmw_addr  : unsigned(10 downto 0) := (others => '0');
  signal s_rmw_luma  : unsigned(7 downto 0) := (others => '0');
  signal s_rmw_bias  : unsigned(7 downto 0) := (others => '0');
  signal s_rmw_t7    : std_logic := '0';
  signal s_rmw_w2    : unsigned(9 downto 0) := (others => '0');   -- stage-A weight
  signal s_rmw_old2  : unsigned(7 downto 0) := (others => '0');   -- stage-A cell value
  signal s_rmw_addr2 : unsigned(10 downto 0) := (others => '0');
  signal s_rmw_en2   : std_logic := '0';
  constant C_SAT_SHIFT : integer := 3;
  constant C_AVG_SHIFT : integer := 3;

  -- vblank scan
  signal s_scan_i   : unsigned(11 downto 0) := (others => '0');
  signal s_acc_vld  : std_logic := '0';
  signal s_acc_i    : unsigned(10 downto 0) := (others => '0');

  -- narrow nested accumulators (block space)
  signal s_M00    : unsigned(19 downto 0) := (others => '0');
  signal s_M10    : unsigned(25 downto 0) := (others => '0');
  signal s_M01    : unsigned(25 downto 0) := (others => '0');
  signal s_colrun : unsigned(14 downto 0) := (others => '0');
  signal s_rowM00 : unsigned(14 downto 0) := (others => '0');
  signal s_rowXV  : unsigned(20 downto 0) := (others => '0');
  signal s_rowrun : unsigned(19 downto 0) := (others => '0');
  signal s_M01acc : unsigned(24 downto 0) := (others => '0');

  constant C_DW   : integer := 22;   -- divider narrowed (M10/M00 pre-scaled >>4; quotient <= 63)
  signal s_dvd    : unsigned(C_DW - 1 downto 0) := (others => '0');
  signal s_dsor   : unsigned(C_DW - 1 downto 0) := (others => '0');
  signal s_rem    : unsigned(C_DW - 1 downto 0) := (others => '0');
  signal s_quo    : unsigned(C_DW - 1 downto 0) := (others => '0');
  signal s_dbit   : integer range 0 to C_DW := 0;

  signal s_cxb, s_cyb   : unsigned(6 downto 0) := (others => '0');
  signal s_cxp, s_cyp   : signed(12 downto 0) := (others => '0');
  signal s_cx_lag, s_cy_lag : signed(12 downto 0) := (others => '0');

  type t_state is (S_IDLE, S_SCAN, S_FIN01, S_DIVCX, S_DIVCY,
                   S_MUL, S_SCX, S_SCY, S_LAG, S_CX2, S_CY2, S_DONE);
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

  u_store : entity work.witness_motion_history
    port map (clk => clk, wr_en => s_m_wr_en, wr_addr => s_m_wr_addr, wr_data => s_m_wr_data,
              rd_addr => s_m_rd_addr, rd_data => s_m_rd_data);

  u_mul : entity work.multiplier_s
    generic map (G_WIDTH => C_MW, G_FRAC_BITS => 0,
                 G_OUTPUT_MIN => -(2 ** (C_MW - 1)), G_OUTPUT_MAX => (2 ** (C_MW - 1)) - 1)
    port map (clk => clk, enable => s_mul_en, x => s_mul_x, y => s_mul_y, z => s_mul_z,
              result => s_mul_r, valid => s_mul_v);

  cx_lag <= s_cx_lag;
  cy_lag <= s_cy_lag;

  p_solve : process (clk)
    variable v_v      : unsigned(7 downto 0);
    variable v_x      : unsigned(5 downto 0);
    variable v_y      : unsigned(4 downto 0);
    variable v_colrun : unsigned(14 downto 0);
    variable v_rowM00 : unsigned(14 downto 0);
    variable v_rowXV  : unsigned(20 downto 0);
    variable v_rowrun : unsigned(19 downto 0);
    variable v_m00s   : unsigned(25 downto 0);
    variable v_base   : unsigned(8 downto 0);
    variable v_w      : unsigned(9 downto 0);
    variable v_diff   : signed(10 downto 0);
    variable v_ndiff  : signed(10 downto 0);
    variable v_old    : unsigned(7 downto 0);
    variable v_new    : signed(11 downto 0);
    variable v_delta, v_step, v_tgt : signed(13 downto 0);
  begin
    if rising_edge(clk) then
      s_mul_en   <= '0';
      solve_done <= '0';
      s_m_wr_en  <= '0';
      s_active_d <= active;
      s_rmw_addr <= cell_addr;
      s_rmw_luma <= luma(9 downto 2);                         -- 8-bit luma
      s_rmw_bias <= resize(shift_right(sat, C_SAT_SHIFT), 8); -- hue saturation bias
      s_rmw_t7   <= t7_track;

      case s_state is
        ----------------------------------------------------------------
        when S_IDLE =>
          s_m_rd_addr <= cell_addr;
          -- Stage A: from the BRAM read, form base (luma | |luma-cell|) + hue bias
          if s_active_d = '1' then
            v_old := unsigned(s_m_rd_data);
            if s_rmw_t7 = '1' then                            -- Motion: |luma - cell|
              v_diff  := resize(signed('0' & s_rmw_luma), 11) - resize(signed('0' & v_old), 11);
              v_ndiff := -v_diff;
              if v_diff < 0 then v_base := unsigned(v_ndiff(8 downto 0));
              else               v_base := unsigned(v_diff(8 downto 0)); end if;
            else                                              -- Brightness: luma
              v_base := resize(s_rmw_luma, 9);
            end if;
            s_rmw_w2    <= resize(v_base, 10) + resize(s_rmw_bias, 10);
            s_rmw_old2  <= v_old;
            s_rmw_addr2 <= s_rmw_addr;
            s_rmw_en2   <= '1';
          else
            s_rmw_en2   <= '0';
          end if;
          -- Stage B: running average (EMA) + clamp + write (from registered A)
          if s_rmw_en2 = '1' then
            v_new := resize(signed('0' & s_rmw_old2), 12)
                     + shift_right(resize(signed('0' & s_rmw_w2), 12) - signed('0' & s_rmw_old2), C_AVG_SHIFT);
            if v_new < 0 then s_m_wr_data <= (others => '0');
            else              s_m_wr_data <= sat8(unsigned(v_new(11 downto 0))); end if;
            s_m_wr_addr <= s_rmw_addr2;
            s_m_wr_en   <= '1';
          end if;
          if frame_end = '1' then
            s_M00 <= (others => '0'); s_M10 <= (others => '0'); s_M01 <= (others => '0');
            s_colrun <= (others => '0'); s_rowM00 <= (others => '0'); s_rowXV <= (others => '0');
            s_rowrun <= (others => '0'); s_M01acc <= (others => '0');
            s_scan_i <= (others => '0'); s_acc_vld <= '0';
            s_state  <= S_SCAN;
          end if;

        ----------------------------------------------------------------
        when S_SCAN =>
          s_m_rd_addr <= s_scan_i(10 downto 0);
          s_acc_i     <= s_scan_i(10 downto 0);
          s_acc_vld   <= '1';
          if s_acc_vld = '1' then
            v_v := unsigned(s_m_rd_data);
            v_x := s_acc_i(5 downto 0);
            v_y := s_acc_i(10 downto 6);
            s_m_wr_addr <= s_acc_i; s_m_wr_data <= (others => '0'); s_m_wr_en <= '1';  -- clear
            if v_x = 0 then
              v_colrun := resize(v_v, 15); v_rowM00 := resize(v_v, 15); v_rowXV := resize(v_v, 21);
            else
              v_colrun := s_colrun + resize(v_v, 15);
              v_rowM00 := s_rowM00 + resize(v_v, 15);
              v_rowXV  := s_rowXV + resize(v_colrun, 21);
            end if;
            s_colrun <= v_colrun; s_rowM00 <= v_rowM00; s_rowXV <= v_rowXV;
            s_M00    <= s_M00 + resize(v_v, 20);
            if v_x = 63 then
              s_M10    <= s_M10 + shift_left(resize(v_rowM00, 26), 6) - resize(v_rowXV, 26);
              v_rowrun := s_rowrun + resize(v_rowM00, 20);
              s_rowrun <= v_rowrun;
              s_M01acc <= s_M01acc + resize(v_rowrun, 25);
            end if;
          end if;
          if s_scan_i = 2048 then s_state <= S_FIN01; else s_scan_i <= s_scan_i + 1; end if;

        when S_FIN01 =>
          v_m00s := shift_left(resize(s_M00, 26), 5);
          if v_m00s >= resize(s_M01acc, 26) then s_M01 <= v_m00s - resize(s_M01acc, 26);
          else s_M01 <= (others => '0'); end if;
          s_dvd  <= resize(shift_right(s_M10, 4), C_DW); s_dsor <= resize(shift_right(s_M00, 4), C_DW);
          s_rem  <= (others => '0'); s_quo <= (others => '0'); s_dbit <= C_DW;
          s_state <= S_DIVCX;

        when S_DIVCX =>
          if s_dbit = 0 then
            s_cxb  <= resize(s_quo(6 downto 0), 7);
            s_dvd  <= resize(shift_right(s_M01, 4), C_DW); s_dsor <= resize(shift_right(s_M00, 4), C_DW);
            s_rem  <= (others => '0'); s_quo <= (others => '0'); s_dbit <= C_DW;
            s_state <= S_DIVCY;
          else div_step(s_rem, s_quo, s_dvd, s_dsor, s_dbit); end if;

        when S_DIVCY =>
          if s_dbit = 0 then s_cyb <= resize(s_quo(6 downto 0), 7); s_state <= S_SCX;
          else div_step(s_rem, s_quo, s_dvd, s_dsor, s_dbit); end if;

        ----------------------------------------------------------------
        when S_MUL =>
          if s_wait = 0 then s_state <= s_ret; else s_wait <= s_wait - 1; end if;

        when S_SCX =>                 -- cx_px = (cxb * active_w) >> 6
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

        when S_CX2 =>                 -- coordinate-engine r^2 seeds
          s_mul_x <= resize(s_cx_lag, C_MW); s_mul_y <= resize(s_cx_lag, C_MW);
          s_mul_en <= '1'; s_wait <= C_MLAT; s_ret <= S_CY2; s_state <= S_MUL;

        when S_CY2 =>
          cx2 <= unsigned(s_mul_r(23 downto 0));
          s_mul_x <= resize(s_cy_lag, C_MW); s_mul_y <= resize(s_cy_lag, C_MW);
          s_mul_en <= '1'; s_wait <= C_MLAT; s_ret <= S_DONE; s_state <= S_MUL;

        when S_DONE =>
          cy2 <= unsigned(s_mul_r(23 downto 0));
          solve_done <= '1';
          s_state    <= S_IDLE;
      end case;
    end if;
  end process p_solve;

end architecture rtl;
