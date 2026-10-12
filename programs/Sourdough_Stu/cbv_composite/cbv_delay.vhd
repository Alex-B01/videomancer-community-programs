-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_delay.vhd - CBV Composite fixed delay line: EBR circular buffer or FF chain (spec §3.1)
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
-- o(t) = i(t - G_D), bit-identical to a G_D-deep FF chain (R4.1-12).
-- EBR form (G_EBR): 256 x 16 RAM (1 EBR), input register -> RAM write at
--   pointer p and registered read of the same address (read-before-write,
--   pitfall #23) -> output register. p counts modulo D' = G_D - 2, so the word
--   read was written D' clocks earlier: total delay = D' + 2 = G_D.
-- Power-up: RAM and registers are zero, so G_INIT is stored XOR-ed (a zero
-- word reads back as G_INIT): active-low sync bits idle high from reset,
-- exactly like the FF chain's initial values.
-- Use for W x D >= ~150 FF (the pointer + mux cost ~15-20 LC).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity cbv_delay is
  generic (
    G_W    : natural := 16;     -- width (<= 16 for one EBR)
    G_D    : natural := 20;     -- delay in clocks (4 .. 258 for the EBR form)
    G_EBR  : boolean := true;
    G_INIT : std_logic_vector(15 downto 0) := (others => '0')
  );
  port (
    clk : in  std_logic;
    i_d : in  std_logic_vector(G_W - 1 downto 0);
    o_d : out std_logic_vector(G_W - 1 downto 0)
  );
end entity cbv_delay;

architecture rtl of cbv_delay is
  constant C_INIT : std_logic_vector(G_W - 1 downto 0) := G_INIT(G_W - 1 downto 0);
begin

  gen_ebr : if G_EBR generate
    constant C_DM : natural := G_D - 2;
    type t_ram is array (0 to 255) of std_logic_vector(G_W - 1 downto 0);
    signal s_ram : t_ram := (others => (others => '0'));   -- 1 EBR (256x16)
    signal s_p   : unsigned(7 downto 0) := (others => '0');
    signal s_in  : std_logic_vector(G_W - 1 downto 0) := (others => '0');
    signal s_q   : std_logic_vector(G_W - 1 downto 0) := (others => '0');
  begin
    p_in : process(clk)
    begin
      if rising_edge(clk) then
        s_in <= i_d xor C_INIT;
        if to_integer(s_p) = C_DM - 1 then
          s_p <= (others => '0');
        else
          s_p <= s_p + 1;
        end if;
      end if;
    end process;

    p_wr : process(clk)
    begin
      if rising_edge(clk) then
        s_ram(to_integer(s_p)) <= s_in;
      end if;
    end process;

    p_rd : process(clk)
    begin
      if rising_edge(clk) then
        s_q <= s_ram(to_integer(s_p));
      end if;
    end process;

    o_d <= s_q xor C_INIT;
  end generate;

  gen_ff : if not G_EBR generate
    type t_chain is array (1 to G_D) of std_logic_vector(G_W - 1 downto 0);
    signal s_c : t_chain := (others => C_INIT);
  begin
    p_ff : process(clk)
    begin
      if rising_edge(clk) then
        s_c(1) <= i_d;
        s_c(2 to G_D) <= s_c(1 to G_D - 1);
      end if;
    end process;
    o_d <= s_c(G_D);
  end generate;

end architecture rtl;
