-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_op2.vhd - CBV Composite split-form shift one-pole OP2, k = kc[P], F = 8 (spec §5.1)
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
-- kc = 2^-a - 2^-b: P = 4: 7/16 (a = 1, b = 4); P = 8: 63/256 (a = 2, b = 8).
--   pre[n] = (X << 8) >> a - (X << 8) >> b          (exact: F = 8 >= b)
--   S[n]   = S[n-1] - (S[n-1] >> a) + (S[n-1] >> b) + pre[n]
--   y      = S >> 8 ; S <- 0 on a clear sample
-- Area structure (bit-identical to the direct formula): the runtime P select
-- picks the shift operands first (2:1 muxes), then 2-operand adds, each in
-- its own assignment so yosys keeps carry-chain $alu cells instead of
-- merging them into a multi-operand LUT compressor tree:
--   pre = pa - pb, pa = X * (P8 ? 64 : 128), pb = X * (P8 ? 1 : 16)
--   S'  = (S - sa) + (sb + pre), sa = S >> (P8 ? 2 : 1), sb = S >> (P8 ? 8 : 4)
-- Pipeline: clock 1 pre (and the sample's P / clear flags), clock 2 S.
-- Latency 2 (input -> S register; y is S >> 8, wiring).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity cbv_op2 is
  generic (
    G_WX : natural := 15;   -- input width (signed)
    G_WS : natural := 24    -- state width (signed; >= G_WX + 8 + 1)
  );
  port (
    clk   : in  std_logic;
    i_x   : in  signed(G_WX - 1 downto 0);
    i_p8  : in  boolean;          -- P = 8 for this sample
    i_clr : in  std_logic;        -- this sample's state is cleared
    o_s   : out signed(G_WS - 1 downto 0);
    o_y   : out signed(G_WS - 9 downto 0)
  );
end entity cbv_op2;

architecture rtl of cbv_op2 is
  signal s_x, s_pa, s_pb : signed(G_WS - 1 downto 0);
  signal s_pre : signed(G_WS - 1 downto 0) := (others => '0');
  signal s_p8  : boolean := false;
  signal s_clr : std_logic := '0';
  signal s_s   : signed(G_WS - 1 downto 0) := (others => '0');
  signal s_sa, s_sb, s_t1, s_t2, s_nx : signed(G_WS - 1 downto 0);
begin
  -- pre (stage 1)
  s_x  <= resize(i_x, G_WS);
  s_pa <= shift_left(s_x, 6) when i_p8 else shift_left(s_x, 7);
  s_pb <= s_x when i_p8 else shift_left(s_x, 4);

  p_pre : process(clk)
  begin
    if rising_edge(clk) then
      s_pre <= s_pa - s_pb;
      s_p8  <= i_p8;
      s_clr <= i_clr;
    end if;
  end process;

  -- recursion (stage 2)
  s_sa <= shift_right(s_s, 2) when s_p8 else shift_right(s_s, 1);
  s_sb <= shift_right(s_s, 8) when s_p8 else shift_right(s_s, 4);
  s_t1 <= s_s - s_sa;
  s_t2 <= s_sb + s_pre;
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
  o_y <= s_s(G_WS - 1 downto 8);

end architecture rtl;
