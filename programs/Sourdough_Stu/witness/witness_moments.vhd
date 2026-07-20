-- Copyright (C) 2026 AB (Sourdough_Stu)
-- SPDX-License-Identifier: GPL-3.0-only
--
-- witness_moments.vhd - BLOCK-SPACE first-moment gaze engine (WITNESS Lite,
--                       REV-D Option-A: block-space vignette, MULTIPLY-FREE)
--
--   ACTIVE raster: read the Bresenham-addressed cell, form base = luma
--     (Brightness) or |luma - cell| (Motion), add the always-on hue bias
--     (sat>>C_SAT_SHIFT), fold w into the cell as a running average (EMA). ONE
--     unified 64x32x8b store (no init, registered read -> found RAM). The RMW is
--     2-staged so the BRAM-read -> write chain does not bound Fmax.
--   VBLANK: scan the 2048 cells for the FIRST MOMENTS ONLY (M00/M10/M01,
--     multiply-free nested sums untangled by <<6/<<5), divide -> the BLOCK
--     centroid (cx_blk in [0,63], cy_blk in [0,31]), then EVADE + rate-normalized
--     LAG (G=2) ON THE BLOCK CENTROID -> the lagged BLOCK gaze.
--
-- REV-D Option-A: the vignette radius is now block-resolution (witness_coord uses
-- the block gaze + Bresenham block coords), so the pixel-precision r^2 recurrence,
-- the cx^2/cy^2 seeds, AND the block->pixel scaling are all gone -> the shared
-- vblank multiplier_s is REMOVED (0 products). This engine is now fully
-- multiply-free; it relieves the ~95%-fill congestion that capped ÷1 Fmax.
-- The blocky vignette edge is the authorized on-theme deviation.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.rate_normalizer_pkg.all;

entity witness_moments is
  port (
    clk         : in  std_logic;
    active      : in  std_logic;
    cell_addr   : in  unsigned(10 downto 0);
    luma        : in  unsigned(9 downto 0);
    sat         : in  unsigned(10 downto 0);
    t7_track    : in  std_logic;
    frame_end   : in  std_logic;
    t8_evade    : in  std_logic;
    lag_shift   : in  t_shift;
    lag_shift_b : in  t_shift;
    lag_use_b   : in  std_logic;
    cx_blk_lag  : out signed(8 downto 0);           -- lagged BLOCK gaze x
    cy_blk_lag  : out signed(8 downto 0);           -- lagged BLOCK gaze y
    solve_done  : out std_logic
  );
end entity witness_moments;

architecture rtl of witness_moments is

  -- unified mass/motion thumbnail (64x32x8, no init -> found RAM)
  signal s_m_rd_addr : unsigned(10 downto 0) := (others => '0');
  signal s_m_wr_addr : unsigned(10 downto 0) := (others => '0');
  signal s_m_wr_data : std_logic_vector(7 downto 0) := (others => '0');
  signal s_m_wr_en   : std_logic := '0';
  signal s_m_rd_data : std_logic_vector(7 downto 0);

  -- active RMW pipeline (3-stage: A base | B +bias | C EMA+write -- keeps the
  -- BRAM-read -> write chain off the Fmax-critical path)
  signal s_active_d  : std_logic := '0';
  signal s_rmw_addr  : unsigned(10 downto 0) := (others => '0');
  signal s_rmw_luma  : unsigned(7 downto 0) := (others => '0');
  signal s_rmw_bias  : unsigned(7 downto 0) := (others => '0');
  signal s_rmw_t7    : std_logic := '0';
  signal s_base      : unsigned(8 downto 0) := (others => '0');   -- stage-A base
  signal s_base_bias : unsigned(7 downto 0) := (others => '0');
  signal s_base_old  : unsigned(7 downto 0) := (others => '0');
  signal s_base_addr : unsigned(10 downto 0) := (others => '0');
  signal s_base_en   : std_logic := '0';
  signal s_rmw_w2    : unsigned(9 downto 0) := (others => '0');   -- stage-B weight
  signal s_rmw_old2  : unsigned(7 downto 0) := (others => '0');
  signal s_rmw_addr2 : unsigned(10 downto 0) := (others => '0');
  signal s_rmw_en2   : std_logic := '0';
  constant C_SAT_SHIFT : integer := 3;
  constant C_AVG_SHIFT : integer := 3;

  -- vblank scan
  signal s_scan_i   : unsigned(11 downto 0) := (others => '0');
  signal s_acc_vld  : std_logic := '0';
  signal s_acc_i    : unsigned(10 downto 0) := (others => '0');

  signal s_M00    : unsigned(19 downto 0) := (others => '0');
  signal s_M10    : unsigned(25 downto 0) := (others => '0');
  signal s_M01    : unsigned(25 downto 0) := (others => '0');
  signal s_colrun : unsigned(14 downto 0) := (others => '0');
  signal s_rowM00 : unsigned(14 downto 0) := (others => '0');
  signal s_rowXV  : unsigned(20 downto 0) := (others => '0');
  signal s_rowrun : unsigned(19 downto 0) := (others => '0');
  signal s_M01acc : unsigned(24 downto 0) := (others => '0');
  signal s_rowm10 : unsigned(25 downto 0) := (others => '0');   -- pipelined row M10 contribution
  signal s_m10go  : std_logic := '0';

  constant C_DW   : integer := 22;   -- divider (M10/M00 pre-scaled >>4; quotient <= 63)
  signal s_dvd    : unsigned(C_DW - 1 downto 0) := (others => '0');
  signal s_dsor   : unsigned(C_DW - 1 downto 0) := (others => '0');
  signal s_rem    : unsigned(C_DW - 1 downto 0) := (others => '0');
  signal s_quo    : unsigned(C_DW - 1 downto 0) := (others => '0');
  signal s_dbit   : integer range 0 to C_DW := 0;

  signal s_cxb, s_cyb : unsigned(6 downto 0) := (others => '0');   -- raw block centroid
  signal s_cxbl, s_cybl : signed(8 downto 0) := (others => '0');   -- lagged block gaze
  signal s_deltax, s_deltay : signed(9 downto 0) := (others => '0');
  signal s_stepx, s_stepy   : signed(9 downto 0) := (others => '0');

  -- LAG pipelined 3 ways (deltas | barrel shifts | adds) so the runtime shift
  -- does not bound Fmax; free (vblank slack).
  type t_state is (S_IDLE, S_SCAN, S_DRAIN, S_FIN01, S_DIVCX, S_DIVCY, S_LAG1, S_LAG2, S_LAG3, S_DONE);
  signal s_state : t_state := S_IDLE;

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

  cx_blk_lag <= s_cxbl;
  cy_blk_lag <= s_cybl;

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
    variable v_delta, v_step, v_tgt : signed(9 downto 0);
  begin
    if rising_edge(clk) then
      solve_done <= '0';
      s_m_wr_en  <= '0';
      s_active_d <= active;
      s_rmw_addr <= cell_addr;
      s_rmw_luma <= luma(9 downto 2);
      s_rmw_bias <= resize(shift_right(sat, C_SAT_SHIFT), 8);
      s_rmw_t7   <= t7_track;

      case s_state is
        ----------------------------------------------------------------
        when S_IDLE =>
          s_m_rd_addr <= cell_addr;
          -- Stage A: from the BRAM read, form base = luma | |luma - cell|
          if s_active_d = '1' then
            v_old := unsigned(s_m_rd_data);
            if s_rmw_t7 = '1' then
              v_diff  := resize(signed('0' & s_rmw_luma), 11) - resize(signed('0' & v_old), 11);
              v_ndiff := -v_diff;
              if v_diff < 0 then v_base := unsigned(v_ndiff(8 downto 0));
              else               v_base := unsigned(v_diff(8 downto 0)); end if;
            else
              v_base := resize(s_rmw_luma, 9);
            end if;
            s_base      <= v_base;
            s_base_bias <= s_rmw_bias;
            s_base_old  <= v_old;
            s_base_addr <= s_rmw_addr;
            s_base_en   <= '1';
          else
            s_base_en   <= '0';
          end if;
          -- Stage B: add the hue bias
          if s_base_en = '1' then
            s_rmw_w2    <= resize(s_base, 10) + resize(s_base_bias, 10);
            s_rmw_old2  <= s_base_old;
            s_rmw_addr2 <= s_base_addr;
            s_rmw_en2   <= '1';
          else
            s_rmw_en2   <= '0';
          end if;
          -- Stage C: running average (EMA) + clamp + write
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
            s_rowrun <= (others => '0'); s_M01acc <= (others => '0'); s_m10go <= '0';
            s_scan_i <= (others => '0'); s_acc_vld <= '0';
            s_state  <= S_SCAN;
          end if;

        ----------------------------------------------------------------
        when S_SCAN =>
          -- pipelined M10: add the previous row's contribution (2-input add, no BRAM)
          if s_m10go = '1' then s_M10 <= s_M10 + s_rowm10; s_m10go <= '0'; end if;
          s_m_rd_addr <= s_scan_i(10 downto 0);
          s_acc_i     <= s_scan_i(10 downto 0);
          s_acc_vld   <= '1';
          if s_acc_vld = '1' then
            v_v := unsigned(s_m_rd_data);
            v_x := s_acc_i(5 downto 0);
            v_y := s_acc_i(10 downto 6);
            s_m_wr_addr <= s_acc_i; s_m_wr_data <= (others => '0'); s_m_wr_en <= '1';
            if v_x = 0 then
              v_colrun := resize(v_v, 15); v_rowM00 := resize(v_v, 15); v_rowXV := resize(v_v, 21);
            else
              v_colrun := s_colrun + resize(v_v, 15);
              v_rowM00 := s_rowM00 + resize(v_v, 15);
              v_rowXV  := s_rowXV + resize(v_colrun, 21);
            end if;
            s_colrun <= v_colrun; s_rowM00 <= v_rowM00; s_rowXV <= v_rowXV;
            s_M00    <= s_M00 + resize(v_v, 20);
            if v_x = 63 then    -- register the row M10 contribution (sub); add lands next cycle
              s_rowm10 <= shift_left(resize(v_rowM00, 26), 6) - resize(v_rowXV, 26);
              s_m10go  <= '1';
              v_rowrun := s_rowrun + resize(v_rowM00, 20);
              s_rowrun <= v_rowrun;
              s_M01acc <= s_M01acc + resize(v_rowrun, 25);
            end if;
          end if;
          if s_scan_i = 2048 then s_state <= S_DRAIN; else s_scan_i <= s_scan_i + 1; end if;

        when S_DRAIN =>          -- flush the last row's pipelined M10 add
          if s_m10go = '1' then s_M10 <= s_M10 + s_rowm10; s_m10go <= '0'; end if;
          s_state <= S_FIN01;

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
          if s_dbit = 0 then s_cyb <= resize(s_quo(6 downto 0), 7); s_state <= S_LAG1;
          else div_step(s_rem, s_quo, s_dvd, s_dsor, s_dbit); end if;

        when S_LAG1 =>          -- EVADE target + delta (subtract)
          if t8_evade = '1' then v_tgt := shift_left(resize(s_cxbl, 10), 1) - resize(signed('0' & s_cxb), 10);
          else                   v_tgt := resize(signed('0' & s_cxb), 10); end if;
          s_deltax <= v_tgt - resize(s_cxbl, 10);
          if t8_evade = '1' then v_tgt := shift_left(resize(s_cybl, 10), 1) - resize(signed('0' & s_cyb), 10);
          else                   v_tgt := resize(signed('0' & s_cyb), 10); end if;
          s_deltay <= v_tgt - resize(s_cybl, 10);
          s_state  <= S_LAG2;

        when S_LAG2 =>          -- rate-normalized step (the runtime barrel shift, isolated)
          v_step := shift_right(s_deltax, lag_shift);
          if lag_use_b = '1' then v_step := v_step + shift_right(s_deltax, lag_shift_b); end if;
          s_stepx <= v_step;
          v_step := shift_right(s_deltay, lag_shift);
          if lag_use_b = '1' then v_step := v_step + shift_right(s_deltay, lag_shift_b); end if;
          s_stepy <= v_step;
          s_state <= S_LAG3;

        when S_LAG3 =>          -- integrate
          s_cxbl  <= resize(resize(s_cxbl, 10) + s_stepx, 9);
          s_cybl  <= resize(resize(s_cybl, 10) + s_stepy, 9);
          s_state <= S_DONE;

        when S_DONE =>
          solve_done <= '1';
          s_state    <= S_IDLE;
      end case;
    end if;
  end process p_solve;

end architecture rtl;
