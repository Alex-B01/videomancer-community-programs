-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_slider.vhd - CBV Composite Colour slider decode: lock, burst offset, detune (spec rev 4.1 §5.9, §6.3)
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
-- c = S12 - 512 (registers_in(7)):
--   |c| <= 41 : locked (burst_off = 0; DA <- 0 on this line)
--   c < -41   : burst_off = (BO[(-c - 41) >> 3] BOFF) >> 8
--   c > 41    : INC = (DT16[(c - 41) >> 3] cpl) >> (8 + log2 P); DA <- DA + INC per line
--   DA u16 (wraps, modular phase); dm_l = DA >> 13.
-- The two products are formed bit-serially from the input-domain line start
-- (registers_in(7) is line l's value from then on) and applied at the Y/C-
-- domain line start of line l (R-LATCH), ~40+ clocks later.
-- (Ambiguity A4: DA steps by this line's INC at this line's decoder start.)
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_variant_pkg.all;
use work.cbv_pkg.all;

entity cbv_slider is
  port (
    clk      : in  std_logic;
    i_sol_in : in  std_logic;                    -- input domain x = 0
    i_tid_in : in  t_tid;
    i_s12    : in  std_logic_vector(9 downto 0); -- registers_in(7)
    i_sol_yc : in  std_logic;                    -- Y/C domain x = 0
    o_boff   : out unsigned(5 downto 0);         -- burst_off of the line (Y/C domain)
    o_dm     : out unsigned(2 downto 0);         -- dm_l
    o_da     : out unsigned(15 downto 0)         -- DA (T5)
  );
end entity cbv_slider;

architecture rtl of cbv_slider is
  constant C_LP : natural := 2 + boolean'pos(C_HD);   -- log2 P
  signal s_mode   : unsigned(1 downto 0) := "00";     -- 0 locked, 1 burst offset, 2 detune
  signal s_k      : natural range 0 to 58 := 0;
  signal s_cnt    : unsigned(3 downto 0) := (others => '1');
  signal s_ma     : unsigned(25 downto 0) := (others => '0');   -- DT16 << step
  signal s_mb     : unsigned(11 downto 0) := (others => '0');   -- cpl bits
  signal s_pa     : unsigned(25 downto 0) := (others => '0');
  signal s_ba     : unsigned(14 downto 0) := (others => '0');   -- BO << step
  signal s_bb     : unsigned(5 downto 0) := (others => '0');    -- BOFF bits
  signal s_pb     : unsigned(14 downto 0) := (others => '0');
  signal s_inc_n  : unsigned(15 downto 0) := (others => '0');
  signal s_boff_n : unsigned(5 downto 0) := (others => '0');
  signal s_mode_n : unsigned(1 downto 0) := "00";
  signal s_da     : unsigned(15 downto 0) := (others => '0');
  signal s_boff   : unsigned(5 downto 0) := (others => '0');
begin
  p_sl : process(clk)
    variable v_c : integer;
    variable f   : t_fmt;
  begin
    if rising_edge(clk) then
      if i_sol_in = '1' then
        f := f_fmt(i_tid_in);
        v_c := to_integer(unsigned(i_s12)) - 512;
        if v_c < -41 then
          s_mode <= "01"; s_k <= (-v_c - 41) / 8;
          s_ba <= to_unsigned(C_BO((-v_c - 41) / 8), 15);
        elsif v_c > 41 then
          s_mode <= "10"; s_k <= (v_c - 41) / 8;
          s_ba <= (others => '0');
        else
          s_mode <= "00"; s_k <= 0;
          s_ba <= (others => '0');
        end if;
        if v_c > 41 then
          s_ma <= to_unsigned(C_DT16((v_c - 41) / 8), 26);
        else
          s_ma <= (others => '0');
        end if;
        s_mb <= to_unsigned(f.cpl, 12);
        s_bb <= to_unsigned(f.boff, 6);
        s_pa <= (others => '0');
        s_pb <= (others => '0');
        s_cnt <= (others => '0');
      elsif s_cnt < 12 then
        if s_mb(0) = '1' then s_pa <= s_pa + s_ma; end if;
        if s_bb(0) = '1' then s_pb <= s_pb + s_ba; end if;
        s_ma <= shift_left(s_ma, 1);
        s_mb <= shift_right(s_mb, 1);
        s_ba <= shift_left(s_ba, 1);
        s_bb <= shift_right(s_bb, 1);
        s_cnt <= s_cnt + 1;
      elsif s_cnt = 12 then
        s_inc_n  <= resize(shift_right(s_pa, 8 + C_LP), 16);
        s_boff_n <= resize(shift_right(s_pb, 8), 6);
        s_mode_n <= s_mode;
        s_cnt <= s_cnt + 1;
      end if;
      -- decoder line start: apply
      if i_sol_yc = '1' then
        case to_integer(s_mode_n) is
          when 0 => s_da <= (others => '0'); s_boff <= (others => '0');
          when 1 => s_boff <= s_boff_n;
          when others => s_da <= s_da + s_inc_n; s_boff <= (others => '0');
        end case;
      end if;
    end if;
  end process;
  o_boff <= s_boff;
  o_dm   <= s_da(15 downto 13);
  o_da   <= s_da;
end architecture rtl;
