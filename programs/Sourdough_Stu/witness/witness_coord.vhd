-- Copyright (C) 2026 AB (Sourdough_Stu)
-- SPDX-License-Identifier: GPL-3.0-only
--
-- witness_coord.vhd - coordinate engine (born in WITNESS)
--
-- Generates, per OUTPUT pixel, the radial term  r2 = dx^2 + dy^2  relative to the
-- lagged gaze, MULTIPLY-FREE. REV-D Fmax form: the square recurrence is kept as a
-- 2-INPUT add by carrying the increment in its own register --
--     dx^2 += dx_inc ;  dx_inc += 2      (dx_inc = 2*dx+1)
-- so the feedback add is a single 24-bit carry chain (not the 3-input dx^2+2dx+1),
-- the shortest recurrence at ~95% fill. Gaze inputs are registered locally; the
-- per-line/frame seeds dx^2 = cx^2, dy^2 = cy^2 come from the moments engine's
-- shared multiplier (cx2/cy2). ZERO multiply here. r2 feeds the IRIS vignette.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity witness_coord is
  port (
    clk       : in  std_logic;
    active    : in  std_logic;
    new_line  : in  std_logic;
    new_frame : in  std_logic;
    cx_lag    : in  signed(12 downto 0);
    cy_lag    : in  signed(12 downto 0);
    cx2       : in  unsigned(23 downto 0);
    cy2       : in  unsigned(23 downto 0);
    dx_o      : out signed(12 downto 0);
    dy_o      : out signed(12 downto 0);
    r2_o      : out unsigned(23 downto 0)
  );
end entity witness_coord;

architecture rtl of witness_coord is
  signal s_cxl   : signed(12 downto 0) := (others => '0');
  signal s_cyl   : signed(12 downto 0) := (others => '0');
  signal s_dx    : signed(12 downto 0) := (others => '0');
  signal s_dy    : signed(12 downto 0) := (others => '0');
  signal s_dx2   : signed(23 downto 0) := (others => '0');   -- >= 0, <= 3.7M
  signal s_dy2   : signed(23 downto 0) := (others => '0');
  signal s_dxinc : signed(14 downto 0) := (others => '0');   -- 2*dx+1
  signal s_dyinc : signed(14 downto 0) := (others => '0');   -- 2*dy+1
  signal s_first : std_logic := '1';
begin

  dx_o <= s_dx;
  dy_o <= s_dy;

  p_coord : process (clk)
  begin
    if rising_edge(clk) then
      s_cxl <= cx_lag;
      s_cyl <= cy_lag;

      if new_frame = '1' then
        s_dy    <= -s_cyl;
        s_dy2   <= signed(resize(cy2, 24));
        s_dyinc <= shift_left(resize(-s_cyl, 15), 1) + 1;    -- 2*(-cy)+1
        s_first <= '1';
      end if;

      if new_line = '1' then
        s_dx    <= -s_cxl;
        s_dx2   <= signed(resize(cx2, 24));
        s_dxinc <= shift_left(resize(-s_cxl, 15), 1) + 1;    -- 2*(-cx)+1
        if s_first = '1' then
          s_first <= '0';
        else
          s_dy2   <= s_dy2 + resize(s_dyinc, 24);            -- 2-input carry chain
          s_dyinc <= s_dyinc + 2;
          s_dy    <= s_dy + 1;
        end if;
      elsif active = '1' then
        r2_o    <= unsigned(s_dx2 + s_dy2);
        s_dx2   <= s_dx2 + resize(s_dxinc, 24);              -- 2-input carry chain
        s_dxinc <= s_dxinc + 2;
        s_dx    <= s_dx + 1;
      end if;
    end if;
  end process p_coord;

end architecture rtl;
