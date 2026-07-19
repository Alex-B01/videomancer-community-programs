-- Copyright (C) 2026 AB (Sourdough_Stu)
-- SPDX-License-Identifier: GPL-3.0-only
--
-- witness_motion_history.vhd - 64x32x8b motion-history image store (born in WITNESS)
--
-- A persistent 2048-cell x 8-bit per-cell staleness/mass thumbnail, tiled by the
-- shared bresenham_indexer (cell_addr = block_y*64 + block_x). Serves TWO roles
-- with ZERO extra EBR: the motion-mass weight for the moments engine (T7 Motion)
-- AND the T11 Trail ghost overlay (rendered nearest-neighbour).
--
-- Canonical video_line_buffer.vhd memory pattern: a single 1-write / 1-read
-- dual-port array, registered read, NO initialisation values -> Yosys infers
-- EBR (2048x8 packs to 2048x2 x4 = 4 physical EBR). Initialising the array, or
-- a combinational read, collapses it to flip-flops (the gravity_bleed 4204%
-- anti-pattern) -- so this file deliberately leaves s_ram uninitialised and
-- registers the read. Decay / inject arithmetic lives in witness.vhd; this
-- module is pure storage.
--
-- Power-of-two depth: 2048 (11-bit address). EBR: 4.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity witness_motion_history is
  generic (
    G_ADDR : positive := 11;   -- 2048 cells = 64 x 32
    G_DATA : positive := 8
  );
  port (
    clk     : in  std_logic;
    wr_en   : in  std_logic;
    wr_addr : in  unsigned(G_ADDR - 1 downto 0);
    wr_data : in  std_logic_vector(G_DATA - 1 downto 0);
    rd_addr : in  unsigned(G_ADDR - 1 downto 0);
    rd_data : out std_logic_vector(G_DATA - 1 downto 0)
  );
end entity witness_motion_history;

architecture rtl of witness_motion_history is
  type t_ram is array (0 to (2 ** G_ADDR) - 1) of std_logic_vector(G_DATA - 1 downto 0);
  signal s_ram : t_ram;                                    -- NO init: BRAM-inference contract
  signal s_rd  : std_logic_vector(G_DATA - 1 downto 0);
begin

  -- write port
  p_wr : process (clk)
  begin
    if rising_edge(clk) then
      if wr_en = '1' then
        s_ram(to_integer(wr_addr)) <= wr_data;
      end if;
    end if;
  end process p_wr;

  -- read port (registered -> EBR)
  p_rd : process (clk)
  begin
    if rising_edge(clk) then
      s_rd <= s_ram(to_integer(rd_addr));
    end if;
  end process p_rd;

  rd_data <= s_rd;

end architecture rtl;
