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
--         PROVISIONAL Q4: minus 2 (one quarter cycle) on comb samples, whose
--         chroma is centred at n - h/2.
--   du  = 2c S8[md], dv = 2c C8[md] with H*(2c) (Q8: H applied to 2c,
--         negation after the shift-sum)
--   lu, lv : 2 cascaded OP2, k = kc[P], F = 8 (no clear)
--   U4  = (sum of lu over x_d mod 4 = 0..3) >> 2, valid at x_d mod 4 = 3
--         (PROVISIONAL Q3: phase referenced to x_d, not x_d - act0)
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
    i_c       : in  signed(15 downto 0);    -- stage 16
    i_comb_en : in  std_logic;              -- stage 16
    o_du      : out signed(17 downto 0);    -- stage 19
    o_dv      : out signed(17 downto 0);
    o_u4      : out signed(17 downto 0);    -- stage 24
    o_v4      : out signed(17 downto 0);
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

  signal s_ha17, s_hb17, s_c2_17 : signed(17 downto 0) := (others => '0');
  signal s_md17, s_md18 : unsigned(2 downto 0) := (others => '0');
  signal s_h18, s_c2_18 : signed(17 downto 0) := (others => '0');
  signal s_du19, s_dv19 : signed(17 downto 0) := (others => '0');
  signal s_pu20, s_pv20 : signed(27 downto 0) := (others => '0');
  signal s_s1u, s_s1v   : signed(27 downto 0) := (others => '0');
  signal s_pu22, s_pv22 : signed(27 downto 0) := (others => '0');
  signal s_s2u, s_s2v   : signed(27 downto 0) := (others => '0');
  signal s_lu23, s_lv23 : signed(19 downto 0);
  signal s_au, s_av     : signed(21 downto 0) := (others => '0');
  signal s_u4, s_v4     : signed(17 downto 0) := (others => '0');
  signal s_valid        : std_logic := '0';

  function f_pre(x : signed; p8 : boolean) return signed is
    variable v : signed(27 downto 0);
  begin
    v := resize(x, 28);
    if p8 then
      return shift_left(v, 6) - v;                    -- a = 2, b = 8
    else
      return shift_left(v, 7) - shift_left(v, 4);     -- a = 1, b = 4
    end if;
  end function;

  function f_step(s : signed; pre : signed; p8 : boolean) return signed is
  begin
    if p8 then
      return s - shift_right(s, 2) + shift_right(s, 8) + pre;
    else
      return s - shift_right(s, 1) + shift_right(s, 4) + pre;
    end if;
  end function;

begin

  p_demod : process(clk)
    variable f    : t_fmt;
    variable v_m  : unsigned(2 downto 0);
    variable v_md : unsigned(2 downto 0);
    variable v_cv : signed(17 downto 0);
    variable v_sum_u, v_sum_v : signed(21 downto 0);
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
      s_xp(14) <= i_x(1 downto 0);
      s_xp(15 to 23) <= s_xp(14 to 22);
      s_p8p(14) <= f.p8;
      s_p8p(15 to 23) <= s_p8p(14 to 22);

      -- stage 17: 2c and partial H(2c) = c + c>>2 + c>>3 + c>>5
      if i_comb_en = '1' then
        v_md := s_mp(16) - 2;
      else
        v_md := s_mp(16);
      end if;
      s_md17  <= v_md;
      s_c2_17 <= shift_left(resize(i_c, 18), 1);
      s_ha17  <= resize(i_c, 18) + resize(shift_right(i_c, 2), 18);
      s_hb17  <= resize(shift_right(i_c, 3), 18) + resize(shift_right(i_c, 5), 18);
      -- stage 18
      s_h18   <= s_ha17 + s_hb17;
      s_c2_18 <= s_c2_17;
      s_md18  <= s_md17;
      -- stage 19: du = 2c S8[md], dv = 2c C8[md]
      case to_integer(s_md18) is
        when 0 | 4  => s_du19 <= (others => '0');
        when 1 | 3  => s_du19 <= s_h18;
        when 2      => s_du19 <= s_c2_18;
        when 5 | 7  => s_du19 <= -s_h18;
        when others => s_du19 <= -s_c2_18;
      end case;
      case to_integer(s_md18) is
        when 2 | 6  => v_cv := (others => '0');
        when 1 | 7  => v_cv := s_h18;
        when 0      => v_cv := s_c2_18;
        when 3 | 5  => v_cv := -s_h18;
        when others => v_cv := -s_c2_18;
      end case;
      s_dv19 <= v_cv;
      -- stage 20..23: two OP2 per channel (kc, F = 8)
      s_pu20 <= f_pre(s_du19, s_p8p(19));
      s_pv20 <= f_pre(s_dv19, s_p8p(19));
      s_s1u  <= f_step(s_s1u, s_pu20, s_p8p(20));
      s_s1v  <= f_step(s_s1v, s_pv20, s_p8p(20));
      s_pu22 <= f_pre(shift_right(s_s1u, 8), s_p8p(21));
      s_pv22 <= f_pre(shift_right(s_s1v, 8), s_p8p(21));
      s_s2u  <= f_step(s_s2u, s_pu22, s_p8p(22));
      s_s2v  <= f_step(s_s2v, s_pv22, s_p8p(22));
      -- stage 24: /4 integrate-and-dump on x_d mod 4
      v_sum_u := s_au + resize(s_lu23, 22);
      v_sum_v := s_av + resize(s_lv23, 22);
      if s_xp(23) = "00" then
        s_au <= resize(s_lu23, 22);
        s_av <= resize(s_lv23, 22);
      else
        s_au <= v_sum_u;
        s_av <= v_sum_v;
      end if;
      if s_xp(23) = "11" then
        s_u4    <= resize(shift_right(v_sum_u, 2), 18);
        s_v4    <= resize(shift_right(v_sum_v, 2), 18);
        s_valid <= '1';
      else
        s_valid <= '0';
      end if;
    end if;
  end process;

  s_lu23 <= resize(shift_right(s_s2u, 8), 20);
  s_lv23 <= resize(shift_right(s_s2v, 8), 20);

  o_du    <= s_du19;
  o_dv    <= s_dv19;
  o_u4    <= s_u4;
  o_v4    <= s_v4;
  o_valid <= s_valid;

end architecture rtl;
