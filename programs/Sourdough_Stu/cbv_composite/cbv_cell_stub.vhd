-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_cell_stub.vhd - CBV Composite cell, Stage-1 identity stub (block 3)
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
-- Stage 1 (spec-stage1.md §1 block 3): o = comp exactly (gain 1, offset 0,
-- latency 0). r2 = r3 = 0, ed = 0, lvl0 unused. No Enhance ROM, S2/S3 RAMs or
-- EXPCSD ROM. Stage 2 replaces this entity with the real CBV001 cell
-- (spec §5.3-§5.7, +15 clk); the port list below is the interface the rest
-- of the program consumes. If the replacement adds latency, the core's
-- C_CELL_LAT constant moves the clamp-domain tap of the sync-flag chain.
-- Resources: 0 LC, 0 EBR.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity cbv_cell_stub is
  port (
    clk    : in  std_logic;
    i_comp : in  signed(13 downto 0);
    o_o    : out signed(13 downto 0);   -- cell output (cv)
    o_r2   : out signed(11 downto 0);   -- S2 loop state (r-units), Stage 3 leak
    o_r3   : out signed(11 downto 0);   -- S3 loop state (r-units), Stage 3 leak
    o_ed   : out signed(13 downto 0)    -- edge detector, Stage 3 leak
  );
end entity cbv_cell_stub;

architecture stub of cbv_cell_stub is
begin
  o_o  <= i_comp;
  o_r2 <= (others => '0');
  o_r3 <= (others => '0');
  o_ed <= (others => '0');
end architecture stub;
