-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_op2s.vhd - CBV Composite split-form shift one-pole OP2, two constant sets (spec §5.1)
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
-- k = 2^-a + sg 2^-b, state F fraction bits (spec §5.1):
--   pre = ((X << F) >> a) + sg ((X << F) >> b)            (registered, clock 1)
--   S   = S - (S >> a) - sg (S >> b) + pre                (clock 2; 0 on a clear sample)
--   y   = S >> F
-- Two constant sets (generics G_K0 / G_K1) picked per sample by i_sel: the
-- runtime 13.5 / 27 MHz select of the SD bitstream (R4.1-9). In the HD
-- bitstream both sets are the same and the select prunes away.
-- Area structure (bit-identical): shift operands muxed before 2-operand adds,
-- each add in its own assignment (carry-chain $alu, no compressor tree); the
-- signed term is one add/sub chain (f_addsub).
-- Latency 2 (input -> S register).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_variant_pkg.all;
use work.cbv_pkg.all;

entity cbv_op2s is
  generic (
    G_WX : natural := 15;
    G_WS : natural := 24;
    G_F  : natural := 8;
    G_K0 : t_k2 := (1, 4, -1);
    G_K1 : t_k2 := (1, 4, -1)
  );
  port (
    clk   : in  std_logic;
    i_x   : in  signed(G_WX - 1 downto 0);
    i_sel : in  std_logic;        -- '1' = set 1
    i_clr : in  std_logic;        -- this sample's state is cleared
    o_s   : out signed(G_WS - 1 downto 0);
    o_y   : out signed(G_WS - 1 - G_F downto 0)
  );
end entity cbv_op2s;

architecture rtl of cbv_op2s is
  function f_sub(sg : integer) return std_logic is
  begin
    if sg < 0 then return '1'; else return '0'; end if;
  end function;
  signal s_xf, s_pa, s_pb, s_pre_n : signed(G_WS - 1 downto 0);
  signal s_pre : signed(G_WS - 1 downto 0) := (others => '0');
  signal s_sel : std_logic := '0';
  signal s_clr : std_logic := '0';
  signal s_s   : signed(G_WS - 1 downto 0) := (others => '0');
  signal s_sa, s_sb, s_t1, s_t2, s_nx : signed(G_WS - 1 downto 0);
  signal w_sgb_pre, w_sgb_s : std_logic;
  signal w_zb_pre, w_zb_s : boolean;
begin
  -- pre (stage 1)
  s_xf <= shift_left(resize(i_x, G_WS), G_F);
  s_pa <= shift_right(s_xf, G_K1.a) when i_sel = '1' else shift_right(s_xf, G_K0.a);
  s_pb <= shift_right(s_xf, G_K1.b) when i_sel = '1' else shift_right(s_xf, G_K0.b);
  w_zb_pre  <= (G_K1.sg = 0) when i_sel = '1' else (G_K0.sg = 0);
  w_sgb_pre <= f_sub(G_K1.sg) when i_sel = '1' else f_sub(G_K0.sg);
  s_pre_n <= s_pa when w_zb_pre else f_addsub(s_pa, s_pb, w_sgb_pre);

  p_pre : process(clk)
  begin
    if rising_edge(clk) then
      s_pre <= s_pre_n;
      s_sel <= i_sel;
      s_clr <= i_clr;
    end if;
  end process;

  -- recursion (stage 2): S - (S>>a) - sg (S>>b) + pre
  s_sa <= shift_right(s_s, G_K1.a) when s_sel = '1' else shift_right(s_s, G_K0.a);
  s_sb <= shift_right(s_s, G_K1.b) when s_sel = '1' else shift_right(s_s, G_K0.b);
  w_zb_s  <= (G_K1.sg = 0) when s_sel = '1' else (G_K0.sg = 0);
  w_sgb_s <= not f_sub(G_K1.sg) when s_sel = '1' else not f_sub(G_K0.sg);   -- -sg
  s_t1 <= s_s - s_sa;
  s_t2 <= s_pre when w_zb_s else f_addsub(s_pre, s_sb, w_sgb_s);
  s_nx <= s_t1 + s_t2;

  p_s : process(clk)
  begin
    if rising_edge(clk) then
      if s_clr = '1' then
        s_s <= (others => '0');
      else
        s_s <= s_nx;
      end if;
    end if;
  end process;

  o_s <= s_s;
  o_y <= s_s(G_WS - 1 downto G_F);
end architecture rtl;
