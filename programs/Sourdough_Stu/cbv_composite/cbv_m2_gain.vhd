-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_m2_gain.vhd - CBV Composite M2 gain (ACC), PAL-S sign, killer gate (block 8, spec §5.11)
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
--   Uo = (uu gU) >> 12,  Vo = (vq gV) >> 12   (gU/gV of ACCq from line l-2)
--   PAL-S (T9 = 1): Vo <- sw_tv Vo (sw_pic: this line's switch, read live; Q12)
--   col_on = 0: Uo = Vo = 0
--   U8 = sat((Uo >> 2) + 128, 0, 255), V8 likewise (spec §5.14)
-- gU, gV, col_on and T9 latch at this block's x = 0 (R-LATCH).
-- One inferred multiplier (19 x 17), two slots per /4 sample.
-- Pipeline (e6 = uu/vq valid at stage 30): e7/e8 issue, e8/e9 products,
--   e9 Uo, e10 gate/sign/U8 conversion, e11 output (stage 35). Latency 5.
-- Resources (paper): ~230 LC; 1 multiplier; no EBR.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_pkg.all;

entity cbv_m2_gain is
  port (
    clk      : in  std_logic;
    i_uu     : in  signed(18 downto 0);    -- stage 30
    i_vq     : in  signed(18 downto 0);
    i_valid  : in  std_logic;
    i_sol    : in  std_logic;              -- this block's x = 0 (stage 30)
    i_gu     : in  unsigned(15 downto 0);
    i_gv     : in  unsigned(15 downto 0);
    i_col_on : in  std_logic;
    i_t9     : in  std_logic;              -- registers_in(6)(2), latched here
    i_sw_pic : in  std_logic;              -- '1' = -1
    o_u8     : out unsigned(7 downto 0);   -- stage 35
    o_v8     : out unsigned(7 downto 0);
    o_valid  : out std_logic
  );
end entity cbv_m2_gain;

architecture rtl of cbv_m2_gain is
  signal s_gu, s_gv : unsigned(15 downto 0) := to_unsigned(C_GU_FINAL, 16);
  signal s_col, s_t9 : std_logic := '1';
  signal s_vq_h  : signed(18 downto 0) := (others => '0');
  signal s_iss   : std_logic := '0';
  signal s_ma    : signed(18 downto 0) := (others => '0');
  signal s_mb    : signed(16 downto 0) := (others => '0');
  signal s_sl1, s_sl2 : unsigned(1 downto 0) := "11";
  signal s_prod  : signed(35 downto 0) := (others => '0');
  signal s_uo    : signed(23 downto 0) := (others => '0');
  signal s_u8a, s_v8a : unsigned(7 downto 0) := to_unsigned(128, 8);
  signal s_va    : std_logic := '0';
  signal s_u8, s_v8 : unsigned(7 downto 0) := to_unsigned(128, 8);
  signal s_ov    : std_logic := '0';

  function f_u8(z : signed(23 downto 0)) return unsigned is
    variable v : signed(23 downto 0);
  begin
    v := shift_right(z, 2) + 128;
    if v < 0 then return to_unsigned(0, 8);
    elsif v > 255 then return to_unsigned(255, 8);
    else return unsigned(v(7 downto 0)); end if;
  end function;
begin

  p_m2 : process(clk)
    variable v_gu, v_gv : unsigned(15 downto 0);
    variable v_vo : signed(23 downto 0);
    variable v_uo : signed(23 downto 0);
  begin
    if rising_edge(clk) then
      -- per-line latches
      if i_sol = '1' then
        s_gu <= i_gu; s_gv <= i_gv; s_col <= i_col_on; s_t9 <= i_t9;
        v_gu := i_gu; v_gv := i_gv;
      else
        v_gu := s_gu; v_gv := s_gv;
      end if;
      -- issue: (uu, gU) at e7, (vq, gV) at e8
      s_sl1 <= "11";
      s_iss <= '0';
      if i_valid = '1' then
        s_ma <= i_uu; s_mb <= signed('0' & v_gu); s_sl1 <= "00";
        s_vq_h <= i_vq; s_iss <= '1';
      elsif s_iss = '1' then
        s_ma <= s_vq_h; s_mb <= signed('0' & s_gv); s_sl1 <= "01";
      end if;
      s_prod <= s_ma * s_mb;
      s_sl2  <= s_sl1;
      s_va   <= '0';
      if s_sl2 = "00" then
        s_uo <= resize(shift_right(s_prod, 12), 24);
      elsif s_sl2 = "01" then
        v_vo := resize(shift_right(s_prod, 12), 24);
        v_uo := s_uo;
        if s_t9 = '1' and i_sw_pic = '1' then
          v_vo := -v_vo;
        end if;
        if s_col = '0' then
          v_uo := (others => '0');
          v_vo := (others => '0');
        end if;
        s_u8a <= f_u8(v_uo);
        s_v8a <= f_u8(v_vo);
        s_va  <= '1';
      end if;
      -- output register (stage 35)
      s_ov <= s_va;
      if s_va = '1' then
        s_u8 <= s_u8a;
        s_v8 <= s_v8a;
      end if;
    end if;
  end process;

  o_u8    <= s_u8;
  o_v8    <= s_v8;
  o_valid <= s_ov;

end architecture rtl;
