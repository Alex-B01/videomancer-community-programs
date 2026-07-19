-- Copyright (C) 2026 AB (Sourdough_Stu)
-- SPDX-License-Identifier: GPL-3.0-only
--
-- rate_normalizer.vhd - Temporal-rate normalizer (born in HOWLBOX, promotion-ready)
--
-- Purpose:
--   Maps a video timing id (registers_in(8)(3 downto 0), 1 of 15) to a per-coeff
--   shift adjustment so that a per-frame/field lag "v += (target-v) >> shift"
--   keeps the same wall-clock time constant across every format. A knob then
--   means the same seconds of settle at NTSC and 1080p24.
--
--   norm_shift(i) = clamp( base_shift(i) + class_delta(timing_id) )
--
--   base_shift is the nominal per-coeff shift at the 1080p30 reference. A slower
--   rate class uses a SMALLER shift (fewer updates/sec -> bigger per-step jump);
--   a faster class uses a LARGER shift. class_delta ~ {-1, 0, +1, +1} over the
--   {24/25, 30, 50, 60} Hz update-cadence classes (interlaced formats update per
--   field, so 59.94 fields is a 60 Hz class -- see LUT below).
--
--   Clean interface, no HOWLBOX-specific coupling inside -> liftable to the
--   shared kit unchanged (mirrors the five-corner harness born in SEQUENCY).
--
-- Resource: ~1 small LUT + adder + clamp per coeff, 1 register stage. 0 EBR.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package rate_normalizer_pkg is
  -- Per-coefficient right-shift amount (e-fold rate for a 1-pole lag).
  subtype t_shift is integer range 0 to 31;
  type    t_shift_array is array (natural range <>) of t_shift;
end package rate_normalizer_pkg;

--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.rate_normalizer_pkg.all;

entity rate_normalizer is
  generic (
    G_NUM_COEFFS : positive := 3        -- = 3 for HOWLBOX (starve, body, spin)
  );
  port (
    clk        : in  std_logic;
    timing_id  : in  unsigned(3 downto 0);                       -- registers_in(8) low nibble
    base_shift : in  t_shift_array(G_NUM_COEFFS - 1 downto 0);   -- nominal shift @ 1080p30 ref
    norm_shift : out t_shift_array(G_NUM_COEFFS - 1 downto 0)    -- = base_shift + class_delta
  );
end entity rate_normalizer;

architecture rtl of rate_normalizer is

  -- Update-cadence rate class per timing id (video_timing_pkg encoding):
  --   id : format        cadence  class  delta
  --    0 : NTSC          59.94 fl   60     +1
  --    1 : 1080i50       50 fl      50     +1
  --    2 : 1080i5994     59.94 fl   60     +1
  --    3 : 1080p24       24 fr      24     -1
  --    4 : 480p          59.94 fr   60     +1
  --    5 : 720p50        50 fr      50     +1
  --    6 : 720p5994      59.94 fr   60     +1
  --    7 : 1080p30       30 fr      30      0   (reference)
  --    8 : PAL           50 fl      50     +1
  --    9 : 1080p2398     23.98 fr   24     -1
  --   10 : 1080i60       60 fl      60     +1
  --   11 : 1080p25       25 fr      25     -1
  --   12 : 576p          50 fr      50     +1
  --   13 : 1080p2997     29.97 fr   30      0
  --   14 : 720p60        60 fr      60     +1
  --   15 : (unused)                          0
  type t_delta_lut is array (0 to 15) of integer range -2 to 2;
  constant C_CLASS_DELTA : t_delta_lut := (
    0 => 1,  1 => 1,  2 => 1,  3 => -1, 4 => 1,  5 => 1,  6 => 1,  7 => 0,
    8 => 1,  9 => -1, 10 => 1, 11 => -1, 12 => 1, 13 => 0, 14 => 1, 15 => 0);

begin

  p_norm : process (clk)
    variable v_delta : integer range -2 to 2;
    variable v_sum   : integer;
  begin
    if rising_edge(clk) then
      v_delta := C_CLASS_DELTA(to_integer(timing_id));
      for i in 0 to G_NUM_COEFFS - 1 loop
        v_sum := base_shift(i) + v_delta;
        -- clamp into a sane, always-lagging range (never 0 = instant settle)
        if v_sum < 1 then
          v_sum := 1;
        elsif v_sum > 31 then
          v_sum := 31;
        end if;
        norm_shift(i) <= v_sum;
      end loop;
    end if;
  end process p_norm;

end architecture rtl;
