-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_domain.vhd - CBV Composite raster domain (block 1, spec §2.1-§2.2)
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
-- One raster "domain": the sync flags of a sample registered at this stage,
-- plus local counters rebuilt from the delayed hsync_n (spec §2.1: domains are
-- 1-bit delay chains + local x, psi, counters).
--   x      : 0 on the first sample with hsync_n = 0 (hsync fall), +1 otherwise
--            (saturates at 4095 if sync is absent)
--   psi    : (psi + D8) mod 8 at each x = 0, reset 0, never reset per field
--   lp     : line parity, toggled at each x = 0, reset 0
--   tid    : registers_in(8)(3:0) latched at x = 0 (R-LATCH, §2.2)
--   reg6   : registers_in(6)(4:0) latched at x = 0 (R-LATCH)
--   vline  : evaluated at x = bst: 1 if vsync_n = 0 there or on any of the
--            3 lines before; holds until the next x = bst
--   aline  : active line index in field (saturates at 3): reset at the vsync_n
--            fall, +1 at each line start after a line that carried AVID
--   a0_m   : measured active start (spec rev 4.1 §2.1): the table act0 from x = 0
--            until the line's first AVID sample, then that sample's x for the
--            rest of the line (ambiguity A2: before the first AVID sample the
--            line's own a0_m is not yet known; = act0 for SDK rasters, A1)
-- Convention (RULED Q11, spec rev 3 §13): an update made at x = 0 applies to the
-- line that starts at that x = 0 (sample x = 0 already sees the new psi / lp / tid).
-- Latency: 1 clock (all outputs aligned with the registered sample).
-- Resources: ~35 LC, no EBR.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_variant_pkg.all;
use work.cbv_pkg.all;

entity cbv_domain is
  port (
    clk     : in  std_logic;
    i_hs_n  : in  std_logic;
    i_vs_n  : in  std_logic;
    i_avid  : in  std_logic;
    i_tid   : in  t_tid;
    i_reg6  : in  std_logic_vector(4 downto 0);
    o_hs_n  : out std_logic;
    o_vs_n  : out std_logic;
    o_avid  : out std_logic;
    o_x     : out unsigned(11 downto 0);
    o_sol   : out std_logic;
    o_psi   : out unsigned(2 downto 0);
    o_lp    : out std_logic;
    o_tid   : out t_tid;
    o_reg6  : out std_logic_vector(4 downto 0);
    o_vline : out std_logic;
    o_aline : out unsigned(1 downto 0);
    o_a0m   : out unsigned(11 downto 0)
  );
end entity cbv_domain;

architecture rtl of cbv_domain is
  signal s_hs_n  : std_logic := '1';
  signal s_vs_n  : std_logic := '1';
  signal s_avid  : std_logic := '0';
  signal s_x     : unsigned(11 downto 0) := (others => '1');
  signal s_sol   : std_logic := '0';
  signal s_psi   : unsigned(2 downto 0) := (others => '0');
  signal s_lp    : std_logic := '0';
  signal s_tid   : t_tid := (others => '0');
  signal s_reg6  : std_logic_vector(4 downto 0) := (others => '0');
  signal s_vhold : std_logic := '0';
  signal s_vcnt  : unsigned(1 downto 0) := (others => '0');
  signal s_vnow  : std_logic;
  signal s_atbst : std_logic;
  signal s_aline : unsigned(1 downto 0) := (others => '0');
  signal s_had   : std_logic := '0';
  signal s_a0m   : unsigned(11 downto 0) := to_unsigned(119, 12);
  signal s_a0ok  : std_logic := '0';
begin

  p_reg : process(clk)
    variable v_sol : boolean;
  begin
    if rising_edge(clk) then
      s_hs_n <= i_hs_n;
      s_vs_n <= i_vs_n;
      s_avid <= i_avid;
      v_sol := (i_hs_n = '0') and (s_hs_n = '1');
      -- a0_m: first AVID sample of the line
      if v_sol then
        s_a0m  <= to_unsigned(f_fmt(i_tid).act0, 12);
        s_a0ok <= '0';
        if i_avid = '1' then
          s_a0m  <= (others => '0');
          s_a0ok <= '1';
        end if;
      elsif i_avid = '1' and s_a0ok = '0' then
        if s_x /= x"FFF" then s_a0m <= s_x + 1; else s_a0m <= s_x; end if;
        s_a0ok <= '1';
      end if;
      if v_sol then
        s_x    <= (others => '0');
        s_sol  <= '1';
        s_tid  <= i_tid;
        s_reg6 <= i_reg6;
        s_psi  <= s_psi + to_unsigned(f_fmt(i_tid).d8, 3);
        s_lp   <= not s_lp;
      else
        if s_x /= x"FFF" then
          s_x <= s_x + 1;
        end if;
        s_sol <= '0';
      end if;
      -- vline (evaluated on the registered sample at x = bst)
      if s_atbst = '1' then
        s_vhold <= s_vnow;
        if s_vs_n = '0' then
          s_vcnt <= "11";
        elsif s_vcnt /= 0 then
          s_vcnt <= s_vcnt - 1;
        end if;
      end if;
      -- active-line-in-field counter
      if i_vs_n = '0' and s_vs_n = '1' then
        s_aline <= (others => '0');
        s_had   <= '0';
      elsif v_sol then
        if s_had = '1' and s_aline /= "11" then
          s_aline <= s_aline + 1;
        end if;
        s_had <= '0';
      elsif i_avid = '1' then
        s_had <= '1';
      end if;
    end if;
  end process;

  s_atbst <= '1' when s_x = to_unsigned(f_fmt(s_tid).bst, 12) else '0';
  s_vnow  <= '1' when (s_vs_n = '0') or (s_vcnt /= 0) else '0';

  o_hs_n  <= s_hs_n;
  o_vs_n  <= s_vs_n;
  o_avid  <= s_avid;
  o_x     <= s_x;
  o_sol   <= s_sol;
  o_psi   <= s_psi;
  o_lp    <= s_lp;
  o_tid   <= s_tid;
  o_reg6  <= s_reg6;
  o_vline <= s_vnow when s_atbst = '1' else s_vhold;
  o_aline <= s_aline;
  o_a0m   <= s_a0m;

end architecture rtl;
