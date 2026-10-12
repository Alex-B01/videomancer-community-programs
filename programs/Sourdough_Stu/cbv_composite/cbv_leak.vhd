-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_leak.vhd - CBV Composite leak mix and separator input (spec rev 4.1 §5.5, §5.6, M18')
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
-- All terms on o's tag (stage O = C_ST_O):
--   pic  = max(0, o - 717) >> 1
--   Lsum = sat(ea + pic + col, 0, 4095)          (col = 0 when C_COLOUR_LEAK = false)
--   leak = (LK8 Lsum) >> 8, LK8 = P3 >> 2         (MUL u8 x u12, registered in and out)
--   s    = T7 ? comp : sat(o - leak, s14)         (stage O + 5 = C_ST_S)
-- LK8 and T7 latch at this block's x = 0 (R-LATCH).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_variant_pkg.all;
use work.cbv_pkg.all;

entity cbv_leak is
  port (
    clk    : in  std_logic;
    i_o    : in  signed(13 downto 0);          -- stage O
    i_ea   : in  unsigned(11 downto 0);        -- stage O
    i_col  : in  unsigned(9 downto 0);         -- stage O
    i_sol  : in  std_logic;                    -- tag x = 0 at stage O
    i_p3   : in  std_logic_vector(9 downto 0); -- registers_in(2) Leak
    i_t7   : in  std_logic;                    -- registers_in(6)(0) Sync: '1' = Clean
    i_comp : in  signed(13 downto 0);          -- comp on the same tag, stage O + 4
    o_pic  : out unsigned(11 downto 0);        -- T1d (stage O + 1)
    o_leak : out unsigned(11 downto 0);        -- T1d (stage O + 4)
    o_s    : out signed(13 downto 0)           -- stage O + 5
  );
end entity cbv_leak;

architecture rtl of cbv_leak is
  signal s_lk8 : unsigned(7 downto 0) := (others => '0');
  signal s_t7  : std_logic := '1';
  signal s_t7p : std_logic_vector(1 to 4) := (others => '1');
  signal s_lkp : unsigned(7 downto 0) := (others => '0');
  type t_o is array (1 to 4) of signed(13 downto 0);
  signal s_od  : t_o := (others => (others => '0'));
  signal s_pic1 : unsigned(11 downto 0) := (others => '0');
  signal s_t1  : unsigned(12 downto 0) := (others => '0');
  signal s_ls2 : unsigned(11 downto 0) := (others => '0');
  signal s_ma, s_mb3 : unsigned(11 downto 0) := (others => '0');
  signal s_la3 : unsigned(7 downto 0) := (others => '0');
  signal s_pr4 : unsigned(19 downto 0) := (others => '0');
  signal s_s5  : signed(13 downto 0) := (others => '0');
  signal s_lk2 : unsigned(7 downto 0) := (others => '0');
begin
  p_leak : process(clk)
    variable v_lk : unsigned(7 downto 0);
    variable v_t7 : std_logic;
    variable v_p  : signed(14 downto 0);
    variable v_l  : unsigned(13 downto 0);
  begin
    if rising_edge(clk) then
      if i_sol = '1' then
        v_lk := unsigned(i_p3(9 downto 2)); v_t7 := i_t7;
        s_lk8 <= v_lk; s_t7 <= v_t7;
      else
        v_lk := s_lk8; v_t7 := s_t7;
      end if;
      s_t7p(1) <= v_t7; s_t7p(2 to 4) <= s_t7p(1 to 3);
      s_od(1) <= i_o; s_od(2 to 4) <= s_od(1 to 3);
      -- O+1: pic, ea + col
      v_p := resize(i_o, 15) - 717;
      if v_p > 0 then s_pic1 <= unsigned(shift_right(v_p, 1)(11 downto 0)); else s_pic1 <= (others => '0'); end if;
      s_t1 <= resize(i_ea, 13) + resize(i_col, 13);
      s_lkp <= v_lk;
      -- O+2: Lsum
      v_l := resize(s_t1, 14) + resize(s_pic1, 14);
      if v_l > 4095 then s_ls2 <= to_unsigned(4095, 12); else s_ls2 <= v_l(11 downto 0); end if;
      s_lk2 <= s_lkp;
      -- O+3: MUL input registers; O+4: product
      s_mb3 <= s_ls2;
      s_la3 <= s_lk2;
      s_pr4 <= s_la3 * s_mb3;
      -- O+5: separator input
      if s_t7p(4) = '1' then
        s_s5 <= i_comp;
      else
        s_s5 <= f_sat(resize(s_od(4), 16) - signed(resize(shift_right(s_pr4, 8), 16)), 14);
      end if;
    end if;
  end process;
  o_pic  <= s_pic1;
  o_leak <= resize(shift_right(s_pr4, 8), 12);
  o_s    <= s_s5;
end architecture rtl;
