-- Copyright (C) 2026 AB (Sourdough_Stu)
-- SPDX-License-Identifier: GPL-3.0-only
--
-- witness_coord.vhd - BLOCK-SPACE radial term (WITNESS Lite, REV-D Option-A)
--
-- The IRIS vignette radius is BLOCK resolution, relative to the lagged block gaze:
--     dbx = block_x - cx_blk ;  dby = block_y - cy_blk ;  dr2 = dbx^2 + dby^2
-- This replaced the per-pixel r^2 = dx^2+dy^2 recurrence (the 24-bit feedback
-- carry chain that capped ÷1 Fmax at 95% fill). REV-D Fmax pass: the squares are a
-- 64-entry SQUARE LUT (|delta| clamped to the 0..63 block range) rather than a
-- fabric multiply -- a single distributed-ROM read, no deep carry, so this block
-- no longer bounds Fmax. The LUT constant is folded at elaboration (0 fabric mult).
-- The vignette falloff is quantized to 64x32 blocks -- the authorized deviation.
--
-- Resource: 2 subtract/abs/clamp + 2 LUT reads + 1 add, 3 pipeline stages. 0 EBR, 0 mult.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity witness_coord is
  port (
    clk     : in  std_logic;
    active  : in  std_logic;
    block_x : in  unsigned(5 downto 0);          -- current pixel block col (0..63)
    block_y : in  unsigned(4 downto 0);          -- current pixel block row (0..31)
    cx_blk  : in  signed(8 downto 0);            -- lagged block gaze x
    cy_blk  : in  signed(8 downto 0);            -- lagged block gaze y
    r2_o    : out unsigned(17 downto 0)          -- block-space dr^2
  );
end entity witness_coord;

architecture rtl of witness_coord is
  type t_sq is array (0 to 63) of unsigned(11 downto 0);
  function gen_sq return t_sq is                 -- i*i folded at elaboration (0 fabric mult)
    variable v : t_sq;
  begin
    for i in 0 to 63 loop v(i) := to_unsigned(i * i, 12); end loop;
    return v;
  end function;
  constant C_SQ : t_sq := gen_sq;

  signal s_adx, s_ady   : unsigned(5 downto 0) := (others => '0');    -- |delta| clamped 0..63
  signal s_dbx2, s_dby2 : unsigned(11 downto 0) := (others => '0');
begin

  p_coord : process (clk)
    variable v_dbx, v_dby : signed(10 downto 0);
  begin
    if rising_edge(clk) then
      -- stage 1: |block - gaze|, clamped to the 0..63 block range
      v_dbx := resize(signed('0' & block_x), 11) - resize(cx_blk, 11);
      if v_dbx(10) = '1' then v_dbx := -v_dbx; end if;
      if v_dbx > 63 then s_adx <= to_unsigned(63, 6); else s_adx <= unsigned(v_dbx(5 downto 0)); end if;
      v_dby := resize(signed('0' & block_y), 11) - resize(cy_blk, 11);
      if v_dby(10) = '1' then v_dby := -v_dby; end if;
      if v_dby > 63 then s_ady <= to_unsigned(63, 6); else s_ady <= unsigned(v_dby(5 downto 0)); end if;
      -- stage 2: square via LUT (no fabric multiply)
      s_dbx2 <= C_SQ(to_integer(s_adx));
      s_dby2 <= C_SQ(to_integer(s_ady));
      -- stage 3: radial term
      r2_o <= resize(s_dbx2 + s_dby2, 18);
    end if;
  end process p_coord;

end architecture rtl;
