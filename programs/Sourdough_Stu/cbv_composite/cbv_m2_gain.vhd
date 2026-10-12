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
--   vq' = (T9 = PAL-S and sw_pic = -1) ? -vq : vq   (sw_pic read live; RULED Q12)
--   uuh = uu >> 1, vqh = vq' >> 1                      (RULED Q14)
--   U8 = sat(((uuh gU9) >> 10) + 128, 0, 255), V8 likewise with vqh, gV9
--   gU9 / gV9 = (ACCq K) >> 15 of line l-2 (u12); col_on = 0: U8 = V8 = 128
-- gU9, gV9, col_on and T9 latch at this block's x = 0 (R-LATCH).
-- One inferred multiplier (14 x 13: s13 halves + guard corner, u12 gain), two
-- slots per /4 sample.
-- Pipeline (e6 = uu/vq valid at stage 30): e7/e8 issue, e8/e9 products,
--   e9/e10 U8/V8 conversion, e11 output (stage 35). Latency 5.
-- Resources (paper): ~380 LC; 1 multiplier; no EBR.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_pkg.all;

entity cbv_m2_gain is
  generic (
    G_CODEC_ONLY : boolean := false
  );
  port (
    clk      : in  std_logic;
    i_uu     : in  signed(13 downto 0);    -- stage 30
    i_vq     : in  signed(13 downto 0);
    i_valid  : in  std_logic;
    i_sol    : in  std_logic;              -- this block's x = 0 (stage 30)
    i_gu     : in  unsigned(11 downto 0);  -- gU9 (line l-2)
    i_gv     : in  unsigned(11 downto 0);  -- gV9
    i_col_on : in  std_logic;
    i_t9     : in  std_logic;              -- registers_in(6)(2), latched here
    i_sw_pic : in  std_logic;              -- '1' = -1
    o_u8     : out unsigned(7 downto 0);   -- stage 35
    o_v8     : out unsigned(7 downto 0);
    o_valid  : out std_logic
  );
end entity cbv_m2_gain;

architecture rtl of cbv_m2_gain is
  function f_gu0 return natural is
  begin
    if G_CODEC_ONLY then return C_GU9_CODEC; else return C_GU9_FINAL; end if;
  end function;
  function f_gv0 return natural is
  begin
    if G_CODEC_ONLY then return C_GV9_CODEC; else return C_GV9_FINAL; end if;
  end function;
  signal s_gu    : unsigned(11 downto 0) := to_unsigned(f_gu0, 12);
  signal s_gv    : unsigned(11 downto 0) := to_unsigned(f_gv0, 12);
  signal s_col, s_t9 : std_logic := '1';
  signal s_vqh   : signed(13 downto 0) := (others => '0');
  signal s_iss   : std_logic := '0';
  signal s_ma    : signed(13 downto 0) := (others => '0');
  signal s_mb    : signed(12 downto 0) := (others => '0');
  signal s_sl1, s_sl2 : unsigned(1 downto 0) := "11";
  signal s_col1, s_col2 : std_logic := '1';
  signal s_prod  : signed(26 downto 0) := (others => '0');
  signal s_u8a   : unsigned(7 downto 0) := to_unsigned(128, 8);
  signal s_va    : std_logic := '0';
  signal s_u8, s_v8 : unsigned(7 downto 0) := to_unsigned(128, 8);
  signal s_ov    : std_logic := '0';

  function f_u8(p : signed(26 downto 0)) return unsigned is
    variable v : signed(26 downto 0);
  begin
    v := shift_right(p, 10) + 128;
    if v < 0 then return to_unsigned(0, 8);
    elsif v > 255 then return to_unsigned(255, 8);
    else return unsigned(v(7 downto 0)); end if;
  end function;
begin

  p_m2 : process(clk)
    variable v_gu  : unsigned(11 downto 0);
    variable v_col : std_logic;
    variable v_t9  : std_logic;
    variable v_vq  : signed(14 downto 0);
    variable v_c8  : unsigned(7 downto 0);
  begin
    if rising_edge(clk) then
      -- per-line latches
      if i_sol = '1' then
        s_gu <= i_gu; s_gv <= i_gv; s_col <= i_col_on; s_t9 <= i_t9;
        v_gu := i_gu; v_col := i_col_on; v_t9 := i_t9;
      else
        v_gu := s_gu; v_col := s_col; v_t9 := s_t9;
      end if;
      -- issue: (uuh, gU9) at e7, (vqh, gV9) at e8
      s_sl1 <= "11";
      s_iss <= '0';
      if i_valid = '1' then
        if v_t9 = '1' and i_sw_pic = '1' then
          v_vq := -resize(i_vq, 15);
        else
          v_vq := resize(i_vq, 15);
        end if;
        s_ma  <= resize(shift_right(i_uu, 1), 14);
        s_mb  <= signed('0' & v_gu);
        s_sl1 <= "00";
        s_vqh <= resize(shift_right(v_vq, 1), 14);
        s_iss <= '1';
        s_col1 <= v_col;
      elsif s_iss = '1' then
        s_ma  <= s_vqh;
        s_mb  <= signed('0' & s_gv);
        s_sl1 <= "01";
      end if;
      s_prod <= s_ma * s_mb;
      s_sl2  <= s_sl1;
      s_col2 <= s_col1;
      s_va   <= '0';
      if s_col2 = '0' then
        v_c8 := to_unsigned(128, 8);
      else
        v_c8 := f_u8(s_prod);
      end if;
      if s_sl2 = "00" then
        s_u8a <= v_c8;
      elsif s_sl2 = "01" then
        s_v8  <= v_c8;
        s_u8  <= s_u8a;
        s_va  <= '1';
      end if;
      -- output strobe (stage 35)
      s_ov <= s_va;
    end if;
  end process;

  o_u8    <= s_u8;
  o_v8    <= s_v8;
  o_valid <= s_ov;

end architecture rtl;
