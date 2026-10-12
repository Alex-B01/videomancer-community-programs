-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_ycsplit.vhd - CBV Composite Y/C split: notch + comb RAM (block 5, spec §5.9)
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
-- n = newest sample (stage 13), h = P/2, c = n - h/2 (centred sample).
--   notch : yN = (sd[n] + sd[n-h]) >> 1, cN = (sd[n] - sd[n-h]) >> 1
--   comb  : word = sat(sd[c] >> 1, -512, 511) written when AVID(c) = 1 at
--           pointer p (mod LAG, +1 per written word), read-before-write:
--           yC = (sd[c] + 2 old) >> 1, cC = (sd[c] - 2 old) >> 1
--   comb_en = AVID(c) and col_on and aline >= LAG/actw and not(1080 and T8=PAL)
--   y = col_on ? (comb_en ? yC : yN) : sd[c];   cS = sat(comb_en ? cC : cN, s13)
--   RULED Q16 (spec rev 3 §13): AVID of the centred sample c gates comb write/enable;
--   col_on / T8 latch at the Y/C-domain x = 0 of the newest sample (RULED Q9).
--   cS = sat(c_out, -4096, 4095) (s13, RULED Q14).
--
-- Comb RAM (EBR map #8): bank A 2048 x 10 (5 EBR, 2048x2 each) for p < 2048,
--   bank B 512 x 10 (2 EBR, 512x8 each) for 2048 <= p < 2560. Total 7 EBR,
--   one write + one registered read per bank, separate processes per bank
--   (video_line_buffer.vhd structure), output mux after the registered reads.
--
-- Pipeline: 13 sd (+ history) | 14 notch, comb RAM input registers |
--   15 RAM read/write | 16 comb, select -> y, c. Latency 3.
-- Resources (paper): (a) ~125 LC, (b) ~84 LC; 7 EBR.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_variant_pkg.all;
use work.cbv_pkg.all;

entity cbv_ycsplit is
  port (
    clk       : in  std_logic;
    i_sd      : in  signed(15 downto 0);    -- stage 13
    i_avid    : in  std_logic;              -- Y/C domain (newest sample), stage 13
    i_sol     : in  std_logic;
    i_aline   : in  unsigned(1 downto 0);
    i_tid     : in  t_tid;
    i_t8      : in  std_logic;              -- latched at the domain's x = 0
    i_col_on  : in  std_logic;              -- colour engine (live)
    o_y       : out signed(15 downto 0);    -- stage 16
    o_c       : out signed(12 downto 0);    -- cS (s13)
    o_comb_en : out std_logic;
    o_old     : out signed(9 downto 0);     -- cR_old (comb RAM read word)
    o_col_on  : out std_logic               -- col_on as used on this sample
  );
end entity cbv_ycsplit;

architecture rtl of cbv_ycsplit is
  type t_ram_a is array (0 to 2047) of std_logic_vector(9 downto 0);
  type t_ram_b is array (0 to 511) of std_logic_vector(9 downto 0);
  signal s_ram_a : t_ram_a := (others => (others => '0'));   -- 5 EBR
  signal s_ram_b : t_ram_b := (others => (others => '0'));   -- 2 EBR

  signal s_sdh1, s_sdh2, s_sdh3, s_sdh4 : signed(15 downto 0) := (others => '0');
  signal s_avh1, s_avh2 : std_logic := '0';
  signal s_col  : std_logic := '1';
  signal s_p    : unsigned(11 downto 0) := (others => '0');
  -- stage 14
  signal s_yn14, s_cn14, s_sdc14 : signed(15 downto 0) := (others => '0');
  signal s_ce14, s_col14 : std_logic := '0';
  signal s_wa14  : unsigned(11 downto 0) := (others => '0');
  signal s_wd14  : std_logic_vector(9 downto 0) := (others => '0');
  signal s_we14  : std_logic := '0';
  -- stage 15
  signal s_rd_a, s_rd_b : std_logic_vector(9 downto 0) := (others => '0');
  signal s_sel15 : std_logic := '0';
  signal s_yn15, s_cn15, s_sdc15 : signed(15 downto 0) := (others => '0');
  signal s_ce15, s_col15 : std_logic := '0';
  signal s_old15 : signed(9 downto 0);
  -- stage 16
  signal s_y16 : signed(15 downto 0) := (others => '0');
  signal s_c16 : signed(12 downto 0) := (others => '0');
  signal s_ce16, s_col16 : std_logic := '0';
  signal s_old16 : signed(9 downto 0) := (others => '0');
begin

  p_split : process(clk)
    variable f      : t_fmt;
    variable v_col  : std_logic;
    variable v_sdnh : signed(15 downto 0);
    variable v_sdc  : signed(15 downto 0);
    variable v_avc  : std_logic;
    variable v_ce   : std_logic;
    variable v_w    : signed(15 downto 0);
  begin
    if rising_edge(clk) then
      f := f_fmt(i_tid);
      -- per-line col_on (R-LATCH at this domain's x = 0)
      if i_sol = '1' then
        v_col := i_col_on;
        s_col <= i_col_on;
      else
        v_col := s_col;
      end if;
      -- history (s_sdhK = sd[n-K] while sd[n] is at stage 13)
      s_sdh1 <= i_sd; s_sdh2 <= s_sdh1; s_sdh3 <= s_sdh2; s_sdh4 <= s_sdh3;
      s_avh1 <= i_avid; s_avh2 <= s_avh1;
      if C_P8 then
        v_sdnh := s_sdh4; v_sdc := s_sdh2; v_avc := s_avh2;
      else
        v_sdnh := s_sdh2; v_sdc := s_sdh1; v_avc := s_avh1;
      end if;
      -- stage 14: notch + comb RAM input registers
      s_yn14  <= resize(shift_right(resize(i_sd, 17) + resize(v_sdnh, 17), 1), 16);
      s_cn14  <= resize(shift_right(resize(i_sd, 17) - resize(v_sdnh, 17), 1), 16);
      s_sdc14 <= v_sdc;
      if v_avc = '1' and v_col = '1' and to_integer(i_aline) >= f.lagl
         and not (f.cls = C_CLS_1080 and i_t8 = '1') then
        v_ce := '1';
      else
        v_ce := '0';
      end if;
      s_ce14  <= v_ce;
      s_col14 <= v_col;
      v_w := shift_right(v_sdc, 1);
      s_wd14 <= std_logic_vector(f_satr(v_w, -512, 511, 10));
      s_wa14 <= s_p;
      s_we14 <= v_avc;
      if v_avc = '1' then
        if to_integer(s_p) >= f.lag - 1 then
          s_p <= (others => '0');
        else
          s_p <= s_p + 1;
        end if;
      end if;
      -- stage 15
      s_sel15 <= s_wa14(11);
      s_yn15 <= s_yn14; s_cn15 <= s_cn14; s_sdc15 <= s_sdc14;
      s_ce15 <= s_ce14; s_col15 <= s_col14;
      -- stage 16
      if s_col15 = '1' then
        if s_ce15 = '1' then
          s_y16 <= resize(shift_right(resize(s_sdc15, 18) + shift_left(resize(s_old15, 18), 1), 1), 16);
        else
          s_y16 <= s_yn15;
        end if;
      else
        s_y16 <= s_sdc15;
      end if;
      if s_ce15 = '1' then
        s_c16 <= f_sat(shift_right(resize(s_sdc15, 18) - shift_left(resize(s_old15, 18), 1), 1), 13);
      else
        s_c16 <= f_sat(s_cn15, 13);
      end if;
      s_ce16  <= s_ce15;
      s_col16 <= s_col15;
      s_old16 <= s_old15;
    end if;
  end process;

  -- comb RAM bank A: write
  p_ram_a_wr : process(clk)
  begin
    if rising_edge(clk) then
      if s_we14 = '1' and s_wa14(11) = '0' then
        s_ram_a(to_integer(s_wa14(10 downto 0))) <= s_wd14;
      end if;
    end if;
  end process;

  -- comb RAM bank A: registered read (same-clock read returns the old word)
  p_ram_a_rd : process(clk)
  begin
    if rising_edge(clk) then
      s_rd_a <= s_ram_a(to_integer(s_wa14(10 downto 0)));
    end if;
  end process;

  -- comb RAM bank B: write
  p_ram_b_wr : process(clk)
  begin
    if rising_edge(clk) then
      if s_we14 = '1' and s_wa14(11) = '1' then
        s_ram_b(to_integer(s_wa14(8 downto 0))) <= s_wd14;
      end if;
    end if;
  end process;

  -- comb RAM bank B: registered read
  p_ram_b_rd : process(clk)
  begin
    if rising_edge(clk) then
      s_rd_b <= s_ram_b(to_integer(s_wa14(8 downto 0)));
    end if;
  end process;

  s_old15 <= signed(s_rd_b) when s_sel15 = '1' else signed(s_rd_a);

  o_y       <= s_y16;
  o_c       <= s_c16;
  o_comb_en <= s_ce16;
  o_old     <= s_old16;
  o_col_on  <= s_col16;

end architecture rtl;
