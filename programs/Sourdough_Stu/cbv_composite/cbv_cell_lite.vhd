-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_cell_lite.vhd - CBV Composite CBV001 cell-lite + edge leak source (spec rev 4.1 §5.4, §5.5, M7')
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
-- (stage V = C_ST_VIN; constants per §6.2 and class, G2 forces the 13.5 set)
--   HPin  = OP2L(v_in, k_in, F = 27);   w  = sat(v_in - (HPin >> 27), s14)    V+4
--   LPs   = OP2(w, k_sh, F = 13);       hf = w - (LPs >> 13)  (s15)           V+7
--   s     = w + (hf >> 1) + (hf >> 5) + (hf >> 6)   (s16)                     V+9
--   vk    = 2 s - (s >> 3) - (s >> 8)   (s17, K = 1.87109)                    V+10
--   vv    = sat(vk, -1793, 7745)                                             V+11
--   HPout = OP2L(vv, k_out, F = 27);    o  = sat(vv - (HPout >> 27), s14)     V+15
-- Edge leak source (§5.5): LPe = OP2(w, k_e, F = 8); ed = sat(2 (w - (LPe >> 8)), s16);
--   ea = min(|ed|, 4095), delayed to o's tag (FF, 7 x 12).
-- The OP2L line update uses the tag x = 0 of each OP2L's own input sample.
-- State widths: OP2L s42 (|X| < 2^13, F = 27); LPs s28; LPe s23.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_variant_pkg.all;
use work.cbv_pkg.all;

entity cbv_cell_lite is
  port (
    clk   : in  std_logic;
    i_vin : in  signed(13 downto 0);   -- stage V
    i_sol : in  std_logic;             -- tag x = 0 at stage V
    i_ed  : in  std_logic;             -- 27 MHz constant set (latched tid class), stage V
    o_w   : out signed(13 downto 0);   -- V+4  (T1c)
    o_hf  : out signed(14 downto 0);   -- V+7  (T1c)
    o_vv  : out signed(13 downto 0);   -- V+11 (T1c)
    o_o   : out signed(13 downto 0);   -- V+15 (T1c)
    o_ea  : out unsigned(11 downto 0)  -- aligned with o_o (T1d)
  );
end entity cbv_cell_lite;

architecture rtl of cbv_cell_lite is
  function f_set(k : t_k2s; hi : boolean) return t_k2 is
  begin
    if C_HD then return k(2); elsif hi then return k(1); else return k(0); end if;
  end function;
  constant C_KI0 : t_k2 := f_set(C_K_IN, false);
  constant C_KI1 : t_k2 := f_set(C_K_IN, not C_SD_ONE_SET);
  constant C_KO0 : t_k2 := f_set(C_K_OUT, false);
  constant C_KO1 : t_k2 := f_set(C_K_OUT, not C_SD_ONE_SET);
  constant C_KS0 : t_k2 := f_set(C_K_SH, false);
  constant C_KS1 : t_k2 := f_set(C_K_SH, not C_SD_ONE_SET);
  constant C_KE0 : t_k2 := f_set(C_K_E, false);
  constant C_KE1 : t_k2 := f_set(C_K_E, not C_SD_ONE_SET);

  type t_s14 is array (natural range <>) of signed(13 downto 0);
  signal s_vd   : t_s14(1 to 3) := (others => (others => '0'));
  signal s_hpin : signed(14 downto 0);
  signal s_w4   : signed(13 downto 0) := (others => '0');
  signal s_w5, s_w6, s_w7 : signed(13 downto 0) := (others => '0');
  signal s_lps  : signed(14 downto 0);
  signal s_lpe  : signed(14 downto 0);
  signal s_hf7  : signed(14 downto 0) := (others => '0');
  signal s_sa8, s_sb8 : signed(15 downto 0) := (others => '0');
  signal s_s9   : signed(15 downto 0) := (others => '0');
  signal s_vka10 : signed(17 downto 0) := (others => '0');
  signal s_s8_10 : signed(17 downto 0) := (others => '0');
  signal s_vv11 : signed(13 downto 0) := (others => '0');
  signal s_vvd  : t_s14(1 to 3) := (others => (others => '0'));
  signal s_hpout : signed(14 downto 0);
  signal s_o15  : signed(13 downto 0) := (others => '0');
  signal s_ed7  : signed(15 downto 0) := (others => '0');
  type t_u12 is array (natural range <>) of unsigned(11 downto 0);
  signal s_ea   : t_u12(8 to 15) := (others => (others => '0'));
  -- per-sample flags along the cell: sol / set select
  signal s_sol  : std_logic_vector(1 to 11) := (others => '0');
  signal s_sel  : std_logic_vector(1 to 11) := (others => '0');
begin

  -- flags pipe (stage V+k = index k)
  p_flags : process(clk)
  begin
    if rising_edge(clk) then
      s_sol(1) <= i_sol; s_sol(2 to 11) <= s_sol(1 to 10);
      s_sel(1) <= i_ed;  s_sel(2 to 11) <= s_sel(1 to 10);
    end if;
  end process;

  -- HPin: line-rate AC coupling (G6: per sample)
  u_hpin : entity work.cbv_op2l
    generic map (G_WX => 14, G_WS => 42, G_F => 27, G_K0 => C_KI0, G_K1 => C_KI1,
                 G_PER_SAMPLE => C_AC_PER_SAMPLE)
    port map (clk => clk, i_x => i_vin, i_sol => i_sol, i_sel => i_ed, o_y => s_hpin);

  -- LPs (shelf split), LPe (edge detector) on w
  u_lps : entity work.cbv_op2s
    generic map (G_WX => 14, G_WS => 28, G_F => 13, G_K0 => C_KS0, G_K1 => C_KS1)
    port map (clk => clk, i_x => s_w4, i_sel => s_sel(4), i_clr => '0', o_s => open, o_y => s_lps);
  u_lpe : entity work.cbv_op2s
    generic map (G_WX => 14, G_WS => 23, G_F => 8, G_K0 => C_KE0, G_K1 => C_KE1)
    port map (clk => clk, i_x => s_w4, i_sel => s_sel(4), i_clr => '0', o_s => open, o_y => s_lpe);

  -- HPout: line-rate AC coupling on vv
  u_hpout : entity work.cbv_op2l
    generic map (G_WX => 14, G_WS => 42, G_F => 27, G_K0 => C_KO0, G_K1 => C_KO1,
                 G_PER_SAMPLE => C_AC_PER_SAMPLE)
    port map (clk => clk, i_x => s_vv11, i_sol => s_sol(11), i_sel => s_sel(11), o_y => s_hpout);

  p_cell : process(clk)
    variable v_ed : signed(16 downto 0);
    variable v_a  : unsigned(16 downto 0);
  begin
    if rising_edge(clk) then
      -- v_in delayed to V+3
      s_vd(1) <= i_vin; s_vd(2 to 3) <= s_vd(1 to 2);
      -- V+4: w
      s_w4 <= f_sat(resize(s_vd(3), 16) - resize(s_hpin, 16), 14);
      s_w5 <= s_w4; s_w6 <= s_w5; s_w7 <= s_w6;
      -- V+7: hf = w - LPs (s15); ed = sat(2 (w - LPe), s16)
      s_hf7 <= resize(s_w6, 15) - s_lps;
      v_ed  := shift_left(resize(s_w6, 17) - resize(s_lpe, 17), 1);
      s_ed7 <= f_sat(v_ed, 16);
      -- V+8: shelf partial sums; ea = min(|ed|, 4095)
      s_sa8 <= resize(s_w7, 16) + resize(shift_right(s_hf7, 1), 16);
      s_sb8 <= resize(shift_right(s_hf7, 5), 16) + resize(shift_right(s_hf7, 6), 16);
      v_a := unsigned(abs(resize(s_ed7, 17)));
      if v_a > 4095 then s_ea(8) <= to_unsigned(4095, 12); else s_ea(8) <= v_a(11 downto 0); end if;
      s_ea(9 to 15) <= s_ea(8 to 14);
      -- V+9: s
      s_s9 <= s_sa8 + s_sb8;
      -- V+10: 2 s - (s >> 3); s >> 8
      s_vka10 <= shift_left(resize(s_s9, 18), 1) - resize(shift_right(s_s9, 3), 18);
      s_s8_10 <= resize(shift_right(s_s9, 8), 18);
      -- V+11: vv = sat(vk, -1793, 7745)
      s_vv11 <= f_satr(s_vka10 - s_s8_10, -1793, 7745, 14);
      -- vv delayed to V+14
      s_vvd(1) <= s_vv11; s_vvd(2 to 3) <= s_vvd(1 to 2);
      -- V+15: o
      s_o15 <= f_sat(resize(s_vvd(3), 16) - resize(s_hpout, 16), 14);
    end if;
  end process;

  o_w  <= s_w4;
  o_hf <= s_hf7;
  o_vv <= s_vv11;
  o_o  <= s_o15;
  o_ea <= s_ea(15);
end architecture rtl;
