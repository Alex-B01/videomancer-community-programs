-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_demod.vhd - CBV Composite chroma demodulator, LPF, /4 (block 6, spec §5.10)
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
-- Credits: Syntonie CBV001 Rev C schematic (CC BY-SA 4.0), Phil Baljeu's
-- Archer trace. Behavioural model only; no schematic artwork is copied.
--
--   md  = ((8/P) x_d + psi_d) mod 8 of the newest notch sample n
--         RULED Q4 (spec rev 3 §13): minus 2 (one quarter cycle) on comb samples,
--         whose chroma is centred at n - h/2.
--   du  = S8[md] applied to 2 cS, dv likewise (RULED Q8: H applied to 2 cS,
--         negation after the shift-sum). cS is s13 (RULED Q14).
--   lu, lv : 2 cascaded OP2, k = kc[P], F = 8 (no clear); state s23: |S| <= 2^21
--            (|du| <= 8192, DC gain 1, floor drift < 1/k LSB), so nothing wraps
--   U4  = (sum of lu over the 4 tags of a group) >> 2 (RULED Q3): groups by
--         r = (x_n - act0) mod 4, valid at r = 2 (P=8) / 1 (P=4); a group restarts
--         at r = valid + 1. (A line wrap at cpl = 2 mod 4 skips the restart
--         residue: those blanking samples join the next group.)
--   Widths (RULED Q14 + guard): du/dv, lu/lv, U4/V4 are s14 values except the
--         single corner +8192 (cS = -4096 negated, md = 6); one guard bit (s15)
--         keeps that exact, so nothing wraps or saturates.
--
-- Pipeline: 16 c | 17 2c, partial H sums | 18 H | 19 du, dv | 20 pre1 |
--   21 S1 | 22 pre2 | 23 S2 (lu) | 24 U4, V4 (+ valid). Latency 8.
-- Resources (paper): ~250 LC, no EBR, no multipliers.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_pkg.all;

entity cbv_demod is
  port (
    clk       : in  std_logic;
    i_x       : in  unsigned(11 downto 0);  -- Y/C domain, stage 13
    i_psi     : in  unsigned(2 downto 0);
    i_tid     : in  t_tid;
    i_c       : in  signed(12 downto 0);    -- cS, stage 16
    i_comb_en : in  std_logic;              -- stage 16
    o_du      : out signed(14 downto 0);    -- stage 19
    o_dv      : out signed(14 downto 0);
    o_u4      : out signed(14 downto 0);    -- stage 24
    o_v4      : out signed(14 downto 0);
    o_valid   : out std_logic               -- stage 24: U4/V4 loaded on this clock
  );
end entity cbv_demod;

architecture rtl of cbv_demod is
  type t_m_pipe is array (14 to 16) of unsigned(2 downto 0);
  type t_x_pipe is array (14 to 23) of unsigned(1 downto 0);
  type t_b_pipe is array (14 to 23) of boolean;
  signal s_mp  : t_m_pipe := (others => (others => '0'));
  signal s_xp  : t_x_pipe := (others => (others => '0'));
  signal s_p8p : t_b_pipe := (others => false);
  signal s_vr  : t_x_pipe := (others => (others => '0'));   -- valid residue

  signal s_ha17, s_hb17, s_c2_17 : signed(14 downto 0) := (others => '0');
  signal s_md17, s_md18 : unsigned(2 downto 0) := (others => '0');
  signal s_h18, s_c2_18 : signed(14 downto 0) := (others => '0');
  signal s_du19, s_dv19 : signed(14 downto 0) := (others => '0');
  signal s_y1u, s_y1v   : signed(14 downto 0);
  signal s_l2u, s_l2v   : signed(14 downto 0);
  signal s_lu23, s_lv23 : signed(15 downto 0);
  signal s_au, s_av     : signed(17 downto 0) := (others => '0');
  signal s_u4, s_v4     : signed(14 downto 0) := (others => '0');
  signal s_valid        : std_logic := '0';

begin

  p_demod : process(clk)
    variable f    : t_fmt;
    variable v_m  : unsigned(2 downto 0);
    variable v_md : unsigned(2 downto 0);
    variable v_cv, v_cu : signed(14 downto 0);
    variable v_nu, v_nv : std_logic;
    variable v_sum_u, v_sum_v : signed(17 downto 0);
  begin
    if rising_edge(clk) then
      f := f_fmt(i_tid);
      -- carrier index of the newest sample (stage 13 -> 14..16)
      if f.p8 then
        v_m := i_x(2 downto 0) + i_psi;
      else
        v_m := (i_x(1 downto 0) & '0') + i_psi;
      end if;
      s_mp(14) <= v_m;
      s_mp(15 to 16) <= s_mp(14 to 15);
      s_xp(14) <= i_x(1 downto 0) - to_unsigned(f.act0 mod 4, 2);   -- (x_n - act0) mod 4
      s_vr(14) <= to_unsigned(f.r4v, 2);
      s_xp(15 to 23) <= s_xp(14 to 22);
      s_p8p(14) <= f.p8;
      s_p8p(15 to 23) <= s_p8p(14 to 22);
      s_vr(15 to 23) <= s_vr(14 to 22);

      -- stage 17: 2c and partial H(2c) = c + c>>2 + c>>3 + c>>5
      if i_comb_en = '1' then
        v_md := s_mp(16) - 2;
      else
        v_md := s_mp(16);
      end if;
      s_md17  <= v_md;
      s_c2_17 <= shift_left(resize(i_c, 15), 1);
      s_ha17  <= resize(i_c, 15) + resize(shift_right(i_c, 2), 15);
      s_hb17  <= resize(shift_right(i_c, 3), 15) + resize(shift_right(i_c, 5), 15);
      -- stage 18
      s_h18   <= s_ha17 + s_hb17;
      s_c2_18 <= s_c2_17;
      s_md18  <= s_md17;
      -- stage 19: du = 2c S8[md], dv = 2c C8[md]
      -- sign-magnitude select, one incrementer per channel (same values)
      case to_integer(s_md18) is
        when 0 | 4  => v_cu := (others => '0');
        when 2 | 6  => v_cu := s_c2_18;
        when others => v_cu := s_h18;
      end case;
      if s_md18 >= 5 then v_nu := '1'; else v_nu := '0'; end if;
      s_du19 <= f_negif(v_cu, v_nu);
      case to_integer(s_md18) is
        when 2 | 6  => v_cv := (others => '0');
        when 0 | 4  => v_cv := s_c2_18;
        when others => v_cv := s_h18;
      end case;
      if s_md18 >= 3 and s_md18 <= 5 then v_nv := '1'; else v_nv := '0'; end if;
      s_dv19 <= f_negif(v_cv, v_nv);
      -- stage 24: /4 integrate-and-dump on (x_n - act0) mod 4 (RULED Q3)
      v_sum_u := s_au + resize(s_lu23, 18);
      v_sum_v := s_av + resize(s_lv23, 18);
      if s_xp(23) = s_vr(23) + 1 then
        s_au <= resize(s_lu23, 18);
        s_av <= resize(s_lv23, 18);
      else
        s_au <= v_sum_u;
        s_av <= v_sum_v;
      end if;
      if s_xp(23) = s_vr(23) then
        s_u4    <= resize(shift_right(v_sum_u, 2), 15);
        s_v4    <= resize(shift_right(v_sum_v, 2), 15);
        s_valid <= '1';
      else
        s_valid <= '0';
      end if;
    end if;
  end process;

  -- stage 20..23: two OP2 per channel (kc, F = 8, never cleared), cbv_op2
  u_p1u : entity work.cbv_op2 generic map (G_WX => 15, G_WS => 23)
    port map (clk => clk, i_x => s_du19, i_p8 => s_p8p(19), i_clr => '0', o_s => open, o_y => s_y1u);
  u_p1v : entity work.cbv_op2 generic map (G_WX => 15, G_WS => 23)
    port map (clk => clk, i_x => s_dv19, i_p8 => s_p8p(19), i_clr => '0', o_s => open, o_y => s_y1v);
  u_p2u : entity work.cbv_op2 generic map (G_WX => 15, G_WS => 23)
    port map (clk => clk, i_x => s_y1u, i_p8 => s_p8p(21), i_clr => '0', o_s => open, o_y => s_l2u);
  u_p2v : entity work.cbv_op2 generic map (G_WX => 15, G_WS => 23)
    port map (clk => clk, i_x => s_y1v, i_p8 => s_p8p(21), i_clr => '0', o_s => open, o_y => s_l2v);
  s_lu23 <= resize(s_l2u, 16);
  s_lv23 <= resize(s_l2v, 16);

  o_du    <= s_du19;
  o_dv    <= s_dv19;
  o_u4    <= s_u4;
  o_v4    <= s_v4;
  o_valid <= s_valid;

end architecture rtl;
