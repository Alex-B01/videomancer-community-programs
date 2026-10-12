-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_op2l.vhd - CBV Composite line-rate AC-coupling one-pole OP2L (spec §5.1, [G6])
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
-- OP2L (k_in / k_out, spec §5.1), k = 2^-a + sg 2^-b, F fraction bits:
--   within a line S is frozen; per sample  D <- D + pre - (S>>a) - sg (S>>b);
--   at tag x = 0: S <- S + D, D <- 0.
-- Order at the x = 0 sample (ambiguity A3, sequential reading of the spec):
--   the sample's own increment is taken first, with the old S, and belongs to
--   the closing line (D includes it); then S <- S + D and D <- 0. The x = 0
--   sample's output already uses the new S (Q11: x = 0 updates apply to the line
--   that starts there).
-- Implementation (bit-identical): T = S + D is kept as one accumulator, so the
-- line update is S <- T + incr (one adder); K = (S>>a) + sg (S>>b) is a per-line
-- register, recomputed from the new S on the update edge.
-- Pipeline: X at stage s | s+1 pre | s+2 incr = pre - K | s+3 T/S update |
--   o_y = S >> F valid for the sample at s+3 (the caller registers w at s+4).
-- G_PER_SAMPLE (cut G6): plain OP2 (S <- S - S>>a - sg S>>b + pre each sample)
--   at the same latency (pre s+1, pipeline s+2, S s+3).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_variant_pkg.all;
use work.cbv_pkg.all;

entity cbv_op2l is
  generic (
    G_WX         : natural := 14;
    G_WS         : natural := 42;
    G_F          : natural := 27;
    G_K0         : t_k2 := (22, 23, 1);
    G_K1         : t_k2 := (23, 24, 1);
    G_PER_SAMPLE : boolean := false
  );
  port (
    clk   : in  std_logic;
    i_x   : in  signed(G_WX - 1 downto 0);
    i_sol : in  std_logic;        -- tag x = 0 of the sample at stage s
    i_sel : in  std_logic;        -- '1' = set 1
    o_y   : out signed(G_WS - 1 - G_F downto 0)
  );
end entity cbv_op2l;

architecture rtl of cbv_op2l is
  function f_k(s : signed; sel : std_logic) return signed is
    variable v_k : t_k2;
    variable v_a, v_b : signed(s'length - 1 downto 0);
  begin
    if sel = '1' then v_k := G_K1; else v_k := G_K0; end if;
    v_a := shift_right(s, v_k.a);
    v_b := shift_right(s, v_k.b);
    if v_k.sg > 0 then return v_a + v_b;
    elsif v_k.sg < 0 then return v_a - v_b;
    else return v_a; end if;
  end function;
  function f_pre(x : signed; sel : std_logic) return signed is
    variable v_x : signed(G_WS - 1 downto 0);
  begin
    v_x := shift_left(resize(x, G_WS), G_F);
    return f_k(v_x, sel);
  end function;

  signal s_pre : signed(G_WS - 1 downto 0) := (others => '0');
  signal s_sol1, s_sol2, s_sel1, s_sel2 : std_logic := '0';
  signal s_inc : signed(G_WS - 1 downto 0) := (others => '0');
  signal s_t   : signed(G_WS - 1 downto 0) := (others => '0');
  signal s_s   : signed(G_WS - 1 downto 0) := (others => '0');
  signal s_kk  : signed(G_WS - 1 downto 0) := (others => '0');
  signal w_tn  : signed(G_WS - 1 downto 0);
begin

  g_line : if not G_PER_SAMPLE generate
    w_tn <= s_t + s_inc;
    p_l : process(clk)
    begin
      if rising_edge(clk) then
        -- s+1
        s_pre  <= f_pre(i_x, i_sel);
        s_sol1 <= i_sol;
        s_sel1 <= i_sel;
        -- s+2: increment with the line's K
        s_inc  <= s_pre - s_kk;
        s_sol2 <= s_sol1;
        s_sel2 <= s_sel1;
        -- s+3: accumulate; line update at the x = 0 sample
        s_t <= w_tn;
        if s_sol2 = '1' then
          s_s  <= w_tn;
          s_kk <= f_k(w_tn, s_sel2);
        end if;
      end if;
    end process;
  end generate;

  g_samp : if G_PER_SAMPLE generate
    p_s : process(clk)
    begin
      if rising_edge(clk) then
        s_pre  <= f_pre(i_x, i_sel);
        s_sel1 <= i_sel;
        s_inc  <= s_pre;              -- pipeline register (latency match)
        s_sel2 <= s_sel1;
        s_s    <= s_s - f_k(s_s, s_sel2) + s_inc;
      end if;
    end process;
  end generate;

  o_y <= s_s(G_WS - 1 downto G_F);
end architecture rtl;
