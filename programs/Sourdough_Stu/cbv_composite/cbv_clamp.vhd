-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_clamp.vhd - CBV Composite TV clamp, porch mode (block 4, spec §5.8)
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
-- hb = x_d + dh_l (dh_l = 0 in Stage 1). The window starts at the first
-- sample of the line with hb >= CA and sums exactly 2^CLS samples of o;
-- BP = sum >> CLS; BPk = (BP*61) >> 6 (Porch) or (BP*77) >> 6 (Picture,
-- CA_in / CLS_in; T11 is tied to Porch in Stage 1). sd = o - BPk; BPk holds
-- until the next window. RULED Q10 (spec rev 3 §13): the window's own samples use the
-- previous BPk; the new BPk applies from the first sample after the window.
--
-- Pipeline: 10 o, decoder domain (x_d) | 11 window sum | 12 BP | 13 BPk, sd.
-- Latency 3. Resources (paper): ~90 LC, no EBR, constant multiplies as
-- shift-adds.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_pkg.all;

entity cbv_clamp is
  port (
    clk      : in  std_logic;
    i_o      : in  signed(13 downto 0);     -- stage 10
    i_x      : in  unsigned(11 downto 0);   -- decoder domain, stage 10
    i_sol    : in  std_logic;
    i_tid    : in  t_tid;
    i_dh     : in  signed(11 downto 0);     -- dh_l (0 in Stage 1)
    i_pic    : in  std_logic;               -- T11 Picture (tied '0' in Stage 1)
    o_sd     : out signed(15 downto 0);     -- stage 13
    o_bpk    : out signed(15 downto 0);
    o_bpk_ld : out std_logic                -- '1' on the clock BPk takes a new value
  );
end entity cbv_clamp;

architecture rtl of cbv_clamp is
  signal s_armed : std_logic := '0';
  signal s_wcnt  : unsigned(6 downto 0) := (others => '0');
  signal s_acc   : signed(21 downto 0) := (others => '0');
  signal s_cls11 : std_logic := '0';
  signal s_cls12 : std_logic := '0';
  signal s_pic11, s_pic12 : std_logic := '0';
  signal s_sh11  : natural range 0 to 7 := 4;
  signal s_bp12  : signed(15 downto 0) := (others => '0');
  signal s_bpk   : signed(15 downto 0) := (others => '0');
  signal s_ld13  : std_logic := '0';
  signal s_o11, s_o12 : signed(13 downto 0) := (others => '0');
  signal s_sd13  : signed(15 downto 0) := (others => '0');
begin

  p_clamp : process(clk)
    variable f     : t_fmt;
    variable v_hb  : signed(13 downto 0);
    variable v_ca  : integer;
    variable v_sh  : natural range 0 to 7;
    variable v_arm : std_logic;
    variable v_bp  : signed(21 downto 0);
    variable v_k   : signed(22 downto 0);
  begin
    if rising_edge(clk) then
      f := f_fmt(i_tid);
      if i_pic = '1' then
        v_ca := f.ca_in; v_sh := f.clsh_in;
      else
        v_ca := f.ca;    v_sh := f.clsh;
      end if;
      v_hb := resize(signed('0' & i_x), 14) + resize(i_dh, 14);
      -- stage 10 -> 11: window sum
      v_arm := s_armed or i_sol;
      s_cls11 <= '0';
      if s_wcnt /= 0 then
        s_acc  <= s_acc + resize(i_o, 22);
        s_wcnt <= s_wcnt - 1;
        if s_wcnt = 1 then
          s_cls11 <= '1';
        end if;
        s_armed <= v_arm;
      elsif v_arm = '1' and v_hb >= v_ca then
        s_acc   <= resize(i_o, 22);
        s_wcnt  <= to_unsigned(2 ** v_sh - 1, 7);
        s_armed <= '0';
        s_sh11  <= v_sh;
        s_pic11 <= i_pic;
      else
        s_armed <= v_arm;
      end if;
      s_o11 <= i_o;
      -- stage 12: BP = sum >> CLS
      s_cls12 <= s_cls11;
      s_pic12 <= s_pic11;
      if s_cls11 = '1' then
        v_bp := shift_right(s_acc, s_sh11);
        s_bp12 <= resize(v_bp, 16);
      end if;
      s_o12 <= s_o11;
      -- stage 13: BPk (61/64 or 77/64), sd = o - BPk (old BPk on this edge)
      s_ld13 <= s_cls12;
      if s_cls12 = '1' then
        if s_pic12 = '1' then
          v_k := shift_left(resize(s_bp12, 23), 6) + shift_left(resize(s_bp12, 23), 3)
               + shift_left(resize(s_bp12, 23), 2) + resize(s_bp12, 23);
        else
          v_k := shift_left(resize(s_bp12, 23), 6) - shift_left(resize(s_bp12, 23), 1)
               - resize(s_bp12, 23);
        end if;
        s_bpk <= resize(shift_right(v_k, 6), 16);
      end if;
      s_sd13 <= resize(s_o12, 16) - s_bpk;
    end if;
  end process;

  o_sd     <= s_sd13;
  o_bpk    <= s_bpk;
  o_bpk_ld <= s_ld13;

end architecture rtl;
