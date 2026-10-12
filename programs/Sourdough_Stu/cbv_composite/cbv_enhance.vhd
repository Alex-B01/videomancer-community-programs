-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_enhance.vhd - CBV Composite Enhance, 2N7000 2-D ROM (spec rev 4.1 §5.3, M6')
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
-- Nearest point (default, spec §5.3):
--   xs = comp + 1024; i = sat((xs + 16) >> 5, 0, 127); row = (P1 31 + 512) >> 10
--   v_in = 8 ROM[row][i]                       for -1024 <= comp <= 3040
--        = 8 ROM[row][0]   + 2 (comp + 1024)   for comp < -1024
--        = 8 ROM[row][127] + 2 (comp - 3040)   for comp > 3040
--   v_in = sat(v_in, s14)
-- Interpolated (C_ENH_INTERP, owner measurement variant; rev-2 §5.3 form):
--   xi = sat(comp + 1024, 0, 4095); i = xi >> 5, f = xi & 31;
--   v_in = 8 R_i + (((R_{i+1} - R_i) f) >> 2); R_128 = R_127 + 8 (slope-2 point);
--   if xi saturated: + 2 (comp + 1024 - xi); v_in = sat(v_in, s14).
--   ROM split even / odd i (2 x 2048 x 10, address row*64 + i/2).
-- row latches at this block's x = 0 (R-LATCH, registers_in(0)).
-- EBR: nearest 4096 x 10 -> 10 EBR (4096x1 each); interpolated 2 x 2048 x 10 -> 10 EBR.
-- Pipeline: nearest 11 address register | 12 ROM read | 13 v_in (latency 3);
--   interpolated 11 | 12 | 13 select R_i, R_i+1 | 14 dR f | 15 v_in (latency 5).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_variant_pkg.all;
use work.cbv_pkg.all;
use work.cbv_enh_rom_pkg.all;

entity cbv_enhance is
  port (
    clk    : in  std_logic;
    i_comp : in  signed(13 downto 0);          -- stage C_ST_COMP
    i_sol  : in  std_logic;                    -- tag x = 0 at stage C_ST_COMP
    i_p1   : in  std_logic_vector(9 downto 0); -- registers_in(0)
    o_vin  : out signed(13 downto 0);          -- stage C_ST_VIN
    o_row  : out unsigned(4 downto 0)          -- T1b (aligned with o_vin)
  );
end entity cbv_enhance;

architecture rtl of cbv_enhance is
  signal s_row : unsigned(4 downto 0) := to_unsigned(9, 5);
begin

  -- per-line row (R-LATCH at this block's x = 0)
  p_row : process(clk)
  begin
    if rising_edge(clk) then
      if i_sol = '1' then
        s_row <= resize(shift_right(unsigned(i_p1) * to_unsigned(31, 5) + 512, 10), 5);
      end if;
    end if;
  end process;

  g_near : if not C_ENH_INTERP generate
    type t_rom is array (0 to 4095) of std_logic_vector(9 downto 0);
    function f_init return t_rom is
      variable r : t_rom;
    begin
      for k in 0 to 4095 loop r(k) := std_logic_vector(to_signed(C_ENH(k), 10)); end loop;
      return r;
    end function;
    constant C_ROM : t_rom := f_init;          -- 10 EBR
    signal s_a     : unsigned(11 downto 0) := (others => '0');
    signal s_q     : std_logic_vector(9 downto 0) := (others => '0');
    signal s_ext11, s_ext12 : signed(14 downto 0) := (others => '0');   -- 2 (comp - edge) or 0
    signal s_row11, s_row12, s_row13 : unsigned(4 downto 0) := (others => '0');
    signal s_vin   : signed(13 downto 0) := (others => '0');
    signal w_row   : unsigned(4 downto 0);
  begin
    w_row <= resize(shift_right(unsigned(i_p1) * to_unsigned(31, 5) + 512, 10), 5) when i_sol = '1' else s_row;

    p_a : process(clk)
      variable v_xs : signed(15 downto 0);
      variable v_i  : integer;
      variable v_c  : integer;
    begin
      if rising_edge(clk) then
        -- 11: index, extrapolation term
        v_xs := resize(i_comp, 16) + 1024;
        v_i  := to_integer(shift_right(v_xs + 16, 5));
        if v_i < 0 then v_i := 0; elsif v_i > 127 then v_i := 127; end if;
        s_a <= w_row & to_unsigned(v_i, 7);
        v_c := to_integer(i_comp);
        if v_c < -1024 then
          s_ext11 <= to_signed(2 * (v_c + 1024), 15);
        elsif v_c > 3040 then
          s_ext11 <= to_signed(2 * (v_c - 3040), 15);
        else
          s_ext11 <= (others => '0');
        end if;
        s_row11 <= w_row;
        -- 12: ROM read (registered)
        s_q     <= C_ROM(to_integer(s_a));
        s_ext12 <= s_ext11;
        s_row12 <= s_row11;
        -- 13: v_in
        s_vin   <= f_sat(shift_left(resize(signed(s_q), 16), 3) + resize(s_ext12, 16), 14);
        s_row13 <= s_row12;
      end if;
    end process;
    o_vin <= s_vin;
    o_row <= s_row13;
  end generate;

  g_interp : if C_ENH_INTERP generate
    type t_rom2 is array (0 to 2047) of std_logic_vector(9 downto 0);
    function f_init(odd : natural) return t_rom2 is
      variable r : t_rom2;
    begin
      for rr in 0 to 31 loop
        for k in 0 to 63 loop
          r(rr * 64 + k) := std_logic_vector(to_signed(C_ENH(rr * 128 + 2 * k + odd), 10));
        end loop;
      end loop;
      return r;
    end function;
    constant C_ROME : t_rom2 := f_init(0);     -- 5 EBR
    constant C_ROMO : t_rom2 := f_init(1);     -- 5 EBR
    signal s_ae, s_ao : unsigned(10 downto 0) := (others => '0');
    signal s_qe, s_qo : std_logic_vector(9 downto 0) := (others => '0');
    signal s_odd11, s_odd12, s_end11, s_end12 : std_logic := '0';
    signal s_f11, s_f12, s_f13 : unsigned(4 downto 0) := (others => '0');
    signal s_ext11, s_ext12, s_ext13, s_ext14 : signed(14 downto 0) := (others => '0');
    signal s_ri13, s_dr13 : signed(10 downto 0) := (others => '0');
    signal s_ri14 : signed(10 downto 0) := (others => '0');
    signal s_pr14 : signed(15 downto 0) := (others => '0');
    signal s_row11, s_row12, s_row13, s_row14, s_row15 : unsigned(4 downto 0) := (others => '0');
    signal s_vin  : signed(13 downto 0) := (others => '0');
    signal w_row  : unsigned(4 downto 0);
  begin
    w_row <= resize(shift_right(unsigned(i_p1) * to_unsigned(31, 5) + 512, 10), 5) when i_sol = '1' else s_row;

    p_i : process(clk)
      variable v_xs : integer;
      variable v_xi : integer;
      variable v_i  : integer;
      variable v_ke, v_ko : integer;
      variable v_r0, v_r1 : signed(10 downto 0);
    begin
      if rising_edge(clk) then
        -- 11: xi, i, f; even / odd addresses for R_i and R_i+1
        v_xs := to_integer(i_comp) + 1024;
        if v_xs < 0 then v_xi := 0; elsif v_xs > 4095 then v_xi := 4095; else v_xi := v_xs; end if;
        v_i := v_xi / 32;
        s_f11 <= to_unsigned(v_xi mod 32, 5);
        if v_i mod 2 = 0 then
          v_ke := v_i / 2; v_ko := v_i / 2;          -- R_i even, R_i+1 = odd[i/2]
          s_odd11 <= '0';
        else
          v_ke := (v_i + 1) / 2; v_ko := v_i / 2;    -- R_i odd, R_i+1 = even[(i+1)/2]
          s_odd11 <= '1';
        end if;
        if v_ke > 63 then v_ke := 63; end if;        -- i = 127: R_128 synthesised below
        if v_i = 127 then s_end11 <= '1'; else s_end11 <= '0'; end if;
        s_ae <= w_row & to_unsigned(v_ke, 6);
        s_ao <= w_row & to_unsigned(v_ko, 6);
        s_ext11 <= to_signed(2 * (v_xs - v_xi), 15);
        s_row11 <= w_row;
        -- 12: ROM reads
        s_qe <= C_ROME(to_integer(s_ae));
        s_qo <= C_ROMO(to_integer(s_ao));
        s_odd12 <= s_odd11; s_end12 <= s_end11; s_f12 <= s_f11; s_ext12 <= s_ext11; s_row12 <= s_row11;
        -- 13: R_i, dR = R_i+1 - R_i (R_128 = R_127 + 8)
        if s_odd12 = '0' then
          v_r0 := resize(signed(s_qe), 11); v_r1 := resize(signed(s_qo), 11);
        else
          v_r0 := resize(signed(s_qo), 11);
          if s_end12 = '1' then v_r1 := resize(signed(s_qo), 11) + 8;
          else v_r1 := resize(signed(s_qe), 11); end if;
        end if;
        s_ri13 <= v_r0;
        s_dr13 <= v_r1 - v_r0;
        s_f13 <= s_f12; s_ext13 <= s_ext12; s_row13 <= s_row12;
        -- 14: (dR f) >> 2
        s_pr14 <= shift_right(resize(s_dr13 * signed('0' & s_f13), 16), 2);
        s_ri14 <= s_ri13; s_ext14 <= s_ext13; s_row14 <= s_row13;
        -- 15: v_in
        s_vin <= f_sat(shift_left(resize(s_ri14, 17), 3) + resize(s_pr14, 17) + resize(s_ext14, 17), 14);
        s_row15 <= s_row14;
      end if;
    end process;
    o_vin <= s_vin;
    o_row <= s_row15;
  end generate;

end architecture rtl;
