-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_tame_buffer.vhd - CBV Composite Tame buffer + read side (block 12, spec §1.1, §1.3)
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
-- One physical buffer per plane, single bank, rolling base address (no A/B
-- banks, no case-muxed multi-bank access):
--   EBR map #9 : Y  2048 x 10       -> 5 EBR (2048x2 each)
--   EBR map #10: UV  512 x 16 (U8V8) -> 2 EBR (512x8 each)
-- Each RAM: one write process (registered write-port inputs from cbv_output,
-- stage 36) and one registered read process (video_line_buffer.vhd
-- structure). Zero-initialised (spec-stage1 §1 reset state). A same-clock
-- read and write of one address returns the old word.
--
-- Read side (input domain; spec §1.1 boundary stages):
--   RB  : RB <- B_prev (the write base in use) at the input-domain line start;
--         the x = 0 sample already uses the new RB (RULED Q11)
--   2   Y addr = (RB + x - act0) mod 2048;
--       UV addr = (RB/4 + ((x - act0) >> 2) + 1) mod 512 (one word ahead, so
--       the single read port delivers next; cur = the word read one sample
--       earlier), f = (x - act0) & 3
--   3   EBR read   4 EBR output register (+ previous word)
--   5   interp 1: at f = 0 latch cur, d = next - cur
--   6   interp 2: U10 = 4 cur + d f (= cur10 + (d10 f) >> 2 exactly; RULED Q15)
--   7   R1 clamp Y 64..940, U/V 64..960
--   8   AVID gate (Y 64, U = V = 512) + output register
-- Resources (paper): ~175 LC; 7 EBR.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_pkg.all;

entity cbv_tame_buffer is
  port (
    clk      : in  std_logic;
    -- write port (stage 36 registers)
    i_we_y   : in  std_logic;
    i_wa_y   : in  unsigned(10 downto 0);
    i_wd_y   : in  unsigned(9 downto 0);
    i_we_uv  : in  std_logic;
    i_wa_uv  : in  unsigned(8 downto 0);
    i_wd_uv  : in  std_logic_vector(15 downto 0);
    i_b      : in  unsigned(10 downto 0);
    -- read side, input domain stage 1
    i_x      : in  unsigned(11 downto 0);
    i_sol    : in  std_logic;
    i_tid    : in  t_tid;
    i_avid7  : in  std_logic;                -- sync chain tap 7
    o_y      : out unsigned(9 downto 0);     -- stage 8
    o_u      : out unsigned(9 downto 0);
    o_v      : out unsigned(9 downto 0)
  );
end entity cbv_tame_buffer;

architecture rtl of cbv_tame_buffer is
  type t_ram_y  is array (0 to 2047) of std_logic_vector(9 downto 0);
  type t_ram_uv is array (0 to 511) of std_logic_vector(15 downto 0);
  signal s_ram_y  : t_ram_y  := (others => (others => '0'));   -- 5 EBR
  signal s_ram_uv : t_ram_uv := (others => (others => '0'));   -- 2 EBR

  signal s_rb    : unsigned(10 downto 0) := (others => '0');
  signal s_ra_y  : unsigned(10 downto 0) := (others => '0');
  signal s_ra_uv : unsigned(8 downto 0) := (others => '0');
  signal s_f2, s_f3, s_f4, s_f5 : unsigned(1 downto 0) := (others => '0');
  signal s_q_y   : std_logic_vector(9 downto 0) := (others => '0');
  signal s_q_uv  : std_logic_vector(15 downto 0) := (others => '0');
  signal s_y4, s_y5, s_y6 : unsigned(9 downto 0) := (others => '0');
  signal s_uv4, s_uv4p : std_logic_vector(15 downto 0) := (others => '0');
  signal s_cu5, s_cv5 : signed(9 downto 0) := (others => '0');
  signal s_du5, s_dv5 : signed(9 downto 0) := (others => '0');
  signal s_u6, s_v6   : signed(11 downto 0) := (others => '0');
  signal s_y7, s_u7, s_v7 : unsigned(9 downto 0) := (others => '0');
  signal s_y8 : unsigned(9 downto 0) := to_unsigned(64, 10);
  signal s_u8, s_v8 : unsigned(9 downto 0) := to_unsigned(512, 10);
begin

  -- Y RAM: write
  p_y_wr : process(clk)
  begin
    if rising_edge(clk) then
      if i_we_y = '1' then
        s_ram_y(to_integer(i_wa_y)) <= std_logic_vector(i_wd_y);
      end if;
    end if;
  end process;

  -- Y RAM: registered read (stage 3)
  p_y_rd : process(clk)
  begin
    if rising_edge(clk) then
      s_q_y <= s_ram_y(to_integer(s_ra_y));
    end if;
  end process;

  -- UV RAM: write
  p_uv_wr : process(clk)
  begin
    if rising_edge(clk) then
      if i_we_uv = '1' then
        s_ram_uv(to_integer(i_wa_uv)) <= i_wd_uv;
      end if;
    end if;
  end process;

  -- UV RAM: registered read (stage 3)
  p_uv_rd : process(clk)
  begin
    if rising_edge(clk) then
      s_q_uv <= s_ram_uv(to_integer(s_ra_uv));
    end if;
  end process;

  p_read : process(clk)
    variable f     : t_fmt;
    variable v_p   : signed(13 downto 0);
    variable v_cu, v_cv : signed(9 downto 0);
    variable v_u, v_v   : signed(11 downto 0);
    variable v_y   : unsigned(9 downto 0);
    variable v_rb  : unsigned(10 downto 0);
  begin
    if rising_edge(clk) then
      f := f_fmt(i_tid);
      if i_sol = '1' then
        v_rb := i_b;
        s_rb <= i_b;
      else
        v_rb := s_rb;
      end if;
      -- stage 2: addresses
      v_p := resize(signed('0' & i_x), 14) - to_signed(f.act0, 14);
      s_ra_y  <= v_rb + unsigned(v_p(10 downto 0));
      s_ra_uv <= v_rb(10 downto 2) + unsigned(v_p(10 downto 2)) + 1;
      s_f2    <= unsigned(v_p(1 downto 0));
      -- stage 3 (EBR read in p_y_rd / p_uv_rd)
      s_f3 <= s_f2;
      -- stage 4: EBR output register (+ previous word)
      s_y4  <= unsigned(s_q_y);
      s_uv4 <= s_q_uv;
      s_uv4p <= s_uv4;
      s_f4  <= s_f3;
      -- stage 5: cur / d at f = 0 (cur = word read one sample earlier)
      if s_f4 = 0 then
        v_cu := resize(signed('0' & s_uv4p(15 downto 8)), 10) - 128;
        v_cv := resize(signed('0' & s_uv4p(7 downto 0)), 10) - 128;
        s_cu5 <= v_cu;
        s_cv5 <= v_cv;
        s_du5 <= (resize(signed('0' & s_uv4(15 downto 8)), 10) - 128) - v_cu;
        s_dv5 <= (resize(signed('0' & s_uv4(7 downto 0)), 10) - 128) - v_cv;
      end if;
      s_f5 <= s_f4;
      s_y5 <= s_y4;
      -- stage 6: U10 = 4 cur + d f  (offset 512)
      s_u6 <= shift_left(resize(s_cu5, 12), 2) + resize(s_du5 * signed('0' & s_f5), 12) + 512;
      s_v6 <= shift_left(resize(s_cv5, 12), 2) + resize(s_dv5 * signed('0' & s_f5), 12) + 512;
      s_y6 <= s_y5;
      -- stage 7: R1 clamp
      if s_y6 < 64 then v_y := to_unsigned(64, 10);
      elsif s_y6 > 940 then v_y := to_unsigned(940, 10);
      else v_y := s_y6; end if;
      s_y7 <= v_y;
      v_u := s_u6; v_v := s_v6;
      if v_u < 64 then v_u := to_signed(64, 12); elsif v_u > 960 then v_u := to_signed(960, 12); end if;
      if v_v < 64 then v_v := to_signed(64, 12); elsif v_v > 960 then v_v := to_signed(960, 12); end if;
      s_u7 <= unsigned(v_u(9 downto 0));
      s_v7 <= unsigned(v_v(9 downto 0));
      -- stage 8: AVID gate + output register
      if i_avid7 = '1' then
        s_y8 <= s_y7; s_u8 <= s_u7; s_v8 <= s_v7;
      else
        s_y8 <= to_unsigned(64, 10); s_u8 <= to_unsigned(512, 10); s_v8 <= to_unsigned(512, 10);
      end if;
    end if;
  end process;

  o_y <= s_y8;
  o_u <= s_u8;
  o_v <= s_v8;

end architecture rtl;
