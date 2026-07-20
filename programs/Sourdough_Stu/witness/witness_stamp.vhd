-- Copyright (C) 2026 AB (Sourdough_Stu)
-- SPDX-License-Identifier: GPL-3.0-only
--
-- witness_stamp.vhd - orbiting-stamp compositor front end (born in WITNESS, K5/T10)
--
-- Up to 8 baked sprites orbit the lagged gaze. To keep the front end small, the 8
-- stamp CENTRES are precomputed ONCE per frame by reading a single shared, packed
-- orbit table (C_ORBIT_XY, from witness.py) sequentially -- one lookup + add per
-- clock during vblank -> 8 registered centres, ZERO multiply, ONE orbit ROM (not
-- 16 per-pixel copies). Per output pixel: <=8 bounding-box comparators (the §3.1
-- cost) test the registered centres and pick the first sprite the pixel lands in;
-- its 16x16 sprite pixel is read from the 512x8 sprite ROM (set A/B via T10) with
-- a REGISTERED read so Yosys infers EBR (hook 6 `found RAM`). witness.vhd
-- alpha-composites the result with an interpolator_u (S2). K5 sets the live count.
--
-- Latency (per-pixel path): 3 clocks (bbox+addr -> ROM read -> output register).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.witness_stamp_pkg.all;

entity witness_stamp is
  port (
    clk         : in  std_logic;
    active      : in  std_logic;
    new_frame   : in  std_logic;                  -- vblank: recompute orbit centres
    en          : in  std_logic;                  -- K5 > 0
    count       : in  unsigned(3 downto 0);       -- number of live stamps (0..8)
    setb        : in  std_logic;                  -- T10: '0' set A, '1' set B
    orbit_angle : in  unsigned(7 downto 0);       -- current orbit phase
    gaze_x      : in  signed(12 downto 0);
    gaze_y      : in  signed(12 downto 0);
    px          : in  unsigned(11 downto 0);      -- current active column
    py          : in  unsigned(11 downto 0);      -- current active line
    stamp_alpha : out unsigned(7 downto 0);       -- sprite value (0 = transparent)
    stamp_hit   : out std_logic
  );
end entity witness_stamp;

architecture rtl of witness_stamp is
  constant C_HALF : integer := C_SPRITE_N / 2;              -- 8
  signal s_rom    : t_stamp_rom := C_STAMP_ROM;             -- initialised sprite ROM -> EBR
  signal s_addr   : unsigned(8 downto 0) := (others => '0');
  signal s_hit    : std_logic := '0';
  signal s_hit_d  : std_logic := '0';
  signal s_q      : std_logic_vector(7 downto 0) := (others => '0');

  -- 4 registered orbit centres (REV-C: 8->4 slots) recomputed sequentially each frame
  constant C_SLOTS : integer := 4;
  type t_cen is array (0 to C_SLOTS - 1) of signed(12 downto 0);
  signal s_cx   : t_cen := (others => (others => '0'));
  signal s_cy   : t_cen := (others => (others => '0'));
  signal s_load : integer range 0 to C_SLOTS := 0;         -- sequential loader index
begin

  ------------------------------------------------------------------------
  -- centre precompute: ONE shared orbit ROM, one slot per clock (vblank)
  ------------------------------------------------------------------------
  p_centres : process (clk)
    variable v_ang : unsigned(7 downto 0);
    variable v_xy  : std_logic_vector(2 * C_ORBIT_BITS - 1 downto 0);
    variable v_ox  : signed(C_ORBIT_BITS - 1 downto 0);
    variable v_oy  : signed(C_ORBIT_BITS - 1 downto 0);
  begin
    if rising_edge(clk) then
      if new_frame = '1' then
        s_load <= 0;
      elsif s_load < C_SLOTS then
        v_ang := orbit_angle + to_unsigned(s_load * 64, 8);  -- 90 deg spacing (4 slots), wraps
        v_xy  := C_ORBIT_XY(to_integer(v_ang));              -- single shared ROM read
        v_ox  := signed(v_xy(2 * C_ORBIT_BITS - 1 downto C_ORBIT_BITS));
        v_oy  := signed(v_xy(C_ORBIT_BITS - 1 downto 0));
        s_cx(s_load) <= resize(gaze_x + resize(v_ox, 13), 13);
        s_cy(s_load) <= resize(gaze_y + resize(v_oy, 13), 13);
        s_load <= s_load + 1;
      end if;
    end if;
  end process p_centres;

  ------------------------------------------------------------------------
  -- per-pixel bbox against the registered centres (no ROM here)
  ------------------------------------------------------------------------
  p_stamp : process (clk)
    -- (px - cx + C_HALF) lands in [0, C_SPRITE_N) exactly when the pixel is
    -- inside the sprite bbox -> one unsigned range test per axis, no abs/neg.
    variable v_ox   : signed(13 downto 0);
    variable v_oy   : signed(13 downto 0);
    variable v_hit  : std_logic;
    variable v_addr : integer range 0 to C_ROM_ENTRIES - 1;
    variable v_base : integer range 0 to C_ROM_ENTRIES - 1;
  begin
    if rising_edge(clk) then
      v_hit  := '0';
      v_addr := 0;
      if setb = '1' then v_base := 256; else v_base := 0; end if;

      for k in 0 to C_SLOTS - 1 loop
        if en = '1' and k < to_integer(count) and v_hit = '0' then
          v_ox := (resize(signed('0' & px), 14) - resize(s_cx(k), 14)) + C_HALF;
          v_oy := (resize(signed('0' & py), 14) - resize(s_cy(k), 14)) + C_HALF;
          if v_ox >= 0 and v_ox < C_SPRITE_N and v_oy >= 0 and v_oy < C_SPRITE_N then
            v_hit  := '1';
            v_addr := v_base + to_integer(v_oy) * C_SPRITE_N + to_integer(v_ox);
          end if;
        end if;
      end loop;

      -- stage 1: latched address + hit
      s_addr <= to_unsigned(v_addr, 9);
      s_hit  <= v_hit and active;
      -- stage 2: registered sprite-ROM read (EBR)
      s_q    <= s_rom(to_integer(s_addr));
      s_hit_d <= s_hit;
      -- stage 3: output register
      stamp_alpha <= unsigned(s_q);
      stamp_hit   <= s_hit_d;
    end if;
  end process p_stamp;

end architecture rtl;
