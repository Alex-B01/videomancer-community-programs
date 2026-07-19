-- Copyright (C) 2026 AB (Sourdough_Stu)
-- SPDX-License-Identifier: GPL-3.0-only
--
-- witness_coord.vhd - coordinate engine (born in WITNESS)
--
-- Generates, per OUTPUT pixel, the radial term  r2 = dx^2 + dy^2  relative to
-- the lagged gaze (cx_lag, cy_lag), MULTIPLY-FREE:
--     dx increments by 1 per active pixel;   dx^2(x+1) = dx^2(x) + 2*dx + 1
--     dy increments by 1 per active line;    dy^2(y+1) = dy^2(y) + 2*dy + 1
-- The per-line/frame seeds dx^2 = cx_lag^2 and dy^2 = cy_lag^2 are supplied by
-- the moments engine's shared vblank multiplier (cx2/cy2 inputs) -- so this
-- engine itself has ZERO multiply. r2 feeds IRIS (K1/K6) and RINGS (K2).
--
-- Resource: adders + a few registers. 0 EBR, 0 multiply.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity witness_coord is
  port (
    clk       : in  std_logic;
    active    : in  std_logic;                    -- active pixel strobe (avid)
    new_line  : in  std_logic;                    -- first active pixel of a line
    new_frame : in  std_logic;                    -- frame boundary (vsync)
    cx_lag    : in  signed(12 downto 0);          -- lagged gaze x (live pixels)
    cy_lag    : in  signed(12 downto 0);          -- lagged gaze y (live pixels)
    cx2       : in  unsigned(23 downto 0);        -- cx_lag^2  (from shared vblank mult)
    cy2       : in  unsigned(23 downto 0);        -- cy_lag^2  (from shared vblank mult)
    dx_o      : out signed(12 downto 0);          -- x - cx_lag (PULL uses |dy|)
    dy_o      : out signed(12 downto 0);          -- y - cy_lag
    r2_o      : out unsigned(23 downto 0)         -- dx^2 + dy^2 (radial term)
  );
end entity witness_coord;

architecture rtl of witness_coord is
  signal s_dx    : signed(12 downto 0)  := (others => '0');
  signal s_dy    : signed(12 downto 0)  := (others => '0');
  signal s_dx2   : signed(25 downto 0)  := (others => '0');  -- always >= 0
  signal s_dy2   : signed(25 downto 0)  := (others => '0');
  signal s_first : std_logic := '1';    -- first active line of the frame
begin

  dx_o <= s_dx;
  dy_o <= s_dy;

  p_coord : process (clk)
    variable v_dx2n : signed(26 downto 0);
    variable v_dy2n : signed(26 downto 0);
  begin
    if rising_edge(clk) then
      if new_frame = '1' then
        s_dy    <= -cy_lag;
        s_dy2   <= resize(signed('0' & cy2), 26);
        s_first <= '1';
      end if;

      if new_line = '1' then
        -- reset the column sweep for this line
        s_dx  <= -cx_lag;
        s_dx2 <= resize(signed('0' & cx2), 26);
        -- advance the row (dy^2 += 2*dy + 1), except on the frame's first line
        if s_first = '1' then
          s_first <= '0';
        else
          v_dy2n := resize(s_dy2, 27) + shift_left(resize(s_dy, 27), 1) + 1;
          s_dy2  <= resize(v_dy2n, 26);
          s_dy   <= s_dy + 1;
        end if;
      elsif active = '1' then
        -- radial term for the current pixel, then advance the column sweep
        r2_o   <= unsigned(resize(s_dx2 + s_dy2, 24));
        v_dx2n := resize(s_dx2, 27) + shift_left(resize(s_dx, 27), 1) + 1;
        s_dx2  <= resize(v_dx2n, 26);
        s_dx   <= s_dx + 1;
      end if;
    end if;
  end process p_coord;

end architecture rtl;
