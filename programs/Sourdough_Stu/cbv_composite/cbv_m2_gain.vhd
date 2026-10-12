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
-- Digit-serial (values identical): one inferred 14 x 7 multiplier, four slots
-- per /4 sample -- uuh*gU9[5:0], uuh*gU9[11:6], vqh*gV9[5:0], vqh*gV9[11:6];
--   U8 = sat((((hi << 6) + lo + 2^17) >> 10), 0, 255)  (+128 folded as 2^17).
-- Pipeline (e0 = U4 at stage 24; uu at e5, vq at e7 from M1): issue e6..e9,
--   products e7..e10, U8 at e9, V8 at e11, strobe e12 (stage 36).
-- Resources (paper): ~250 LC; 1 multiplier (14 x 7); no EBR.
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
    i_uu     : in  signed(13 downto 0);    -- M1 uu (on i_uvalid, stage 29)
    i_vq     : in  signed(13 downto 0);    -- M1 vq (on i_valid, stage 31)
    i_uvalid : in  std_logic;
    i_valid  : in  std_logic;
    i_sol    : in  std_logic;              -- this block's x = 0 (stage 30)
    i_gu     : in  unsigned(11 downto 0);  -- gU9 (line l-2)
    i_gv     : in  unsigned(11 downto 0);  -- gV9
    i_col_on : in  std_logic;
    i_t9     : in  std_logic;              -- registers_in(6)(2), latched here
    i_sw_pic : in  std_logic;              -- '1' = -1
    o_u8     : out unsigned(7 downto 0);   -- stage 36
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
  constant C_RND : signed(27 downto 0) := to_signed(2 ** 17, 28);   -- +128 after >> 10
  signal s_gu    : unsigned(11 downto 0) := to_unsigned(f_gu0, 12);
  signal s_gv    : unsigned(11 downto 0) := to_unsigned(f_gv0, 12);
  signal s_col, s_t9 : std_logic := '1';
  signal s_uh, s_vh : signed(13 downto 0) := (others => '0');
  signal s_iu, s_iv : std_logic := '0';
  signal s_ma    : signed(13 downto 0) := (others => '0');
  signal s_mb    : signed(6 downto 0) := (others => '0');
  signal s_sl1, s_sl2 : unsigned(2 downto 0) := "111";
  signal s_colf  : std_logic := '1';
  signal s_prod  : signed(20 downto 0) := (others => '0');
  signal s_lo    : signed(27 downto 0) := (others => '0');
  signal s_u8a   : unsigned(7 downto 0) := to_unsigned(128, 8);
  signal s_va    : std_logic := '0';
  signal s_u8, s_v8 : unsigned(7 downto 0) := to_unsigned(128, 8);
  signal s_ov    : std_logic := '0';

  function f_u8(z : signed(27 downto 0)) return unsigned is
    variable v : signed(27 downto 0);
  begin
    v := shift_right(z, 10);
    if v < 0 then return to_unsigned(0, 8);
    elsif v > 255 then return to_unsigned(255, 8);
    else return unsigned(v(7 downto 0)); end if;
  end function;
begin

  p_m2 : process(clk)
    variable v_gu, v_gv : unsigned(11 downto 0);
    variable v_col, v_t9 : std_logic;
    variable v_vq  : signed(14 downto 0);
    variable v_vh  : signed(13 downto 0);
    variable v_sum : signed(27 downto 0);
    variable v_c8  : unsigned(7 downto 0);
  begin
    if rising_edge(clk) then
      -- per-line latches
      if i_sol = '1' then
        s_gu <= i_gu; s_gv <= i_gv; s_col <= i_col_on; s_t9 <= i_t9;
        v_gu := i_gu; v_gv := i_gv; v_col := i_col_on; v_t9 := i_t9;
      else
        v_gu := s_gu; v_gv := s_gv; v_col := s_col; v_t9 := s_t9;
      end if;
      -- issue: e5 uuh*gl, e6 uuh*gh, e7 vqh*gl, e8 vqh*gh
      s_sl1 <= "111";
      s_iu  <= '0';
      s_iv  <= '0';
      if i_uvalid = '1' then
        s_uh  <= resize(shift_right(i_uu, 1), 14);
        s_ma  <= resize(shift_right(i_uu, 1), 14);
        s_mb  <= signed("0" & v_gu(5 downto 0));
        s_sl1 <= "000";
        s_iu  <= '1';
        s_colf <= v_col;
      elsif s_iu = '1' then
        s_ma  <= s_uh;
        s_mb  <= signed("0" & s_gu(11 downto 6));
        s_sl1 <= "001";
      elsif i_valid = '1' then
        if v_t9 = '1' and i_sw_pic = '1' then
          v_vq := -resize(i_vq, 15);
        else
          v_vq := resize(i_vq, 15);
        end if;
        v_vh := resize(shift_right(v_vq, 1), 14);
        s_vh  <= v_vh;
        s_ma  <= v_vh;
        s_mb  <= signed("0" & v_gv(5 downto 0));
        s_sl1 <= "010";
        s_iv  <= '1';
      elsif s_iv = '1' then
        s_ma  <= s_vh;
        s_mb  <= signed("0" & s_gv(11 downto 6));
        s_sl1 <= "011";
      end if;
      -- product (14 x 7)
      s_prod <= s_ma * s_mb;
      s_sl2  <= s_sl1;
      -- accumulate / convert
      s_va <= '0';
      v_sum := shift_left(resize(s_prod, 28), 6) + s_lo;
      if s_colf = '0' then
        v_c8 := to_unsigned(128, 8);
      else
        v_c8 := f_u8(v_sum);
      end if;
      case to_integer(s_sl2) is
        when 0 | 2 => s_lo <= resize(s_prod, 28) + C_RND;
        when 1     => s_u8a <= v_c8;
        when 3     => s_v8 <= v_c8; s_u8 <= s_u8a; s_va <= '1';
        when others => null;
      end case;
      s_ov <= s_va;
    end if;
  end process;

  o_u8    <= s_u8;
  o_v8    <= s_v8;
  o_valid <= s_ov;

end architecture rtl;
