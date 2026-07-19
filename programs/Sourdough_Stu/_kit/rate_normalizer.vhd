-- Copyright (C) 2026 AB (Sourdough_Stu)
-- SPDX-License-Identifier: GPL-3.0-only
--
-- rate_normalizer.vhd - Temporal-rate normalizer (born in HOWLBOX, promotion-ready)
--
-- Purpose:
--   Maps a video timing id (registers_in(8)(3 downto 0), 1 of 15) to a per-coeff
--   scaling of a per-frame/field lag step (v += a scaled (target-v)) so the lag
--   keeps the same wall-clock time constant across every format. A knob then
--   means the same seconds of settle at NTSC and 1080p24.
--
--   To hold the wall-clock feel constant, the per-step change must scale by
--   ref_rate/class_rate = 30/rate. Pure integer shifts hit ONLY the power-of-2
--   classes (30 -> x1.0, 60 -> x0.5); 24/25/50 Hz need non-power-of-2 factors,
--   so this module emits a shift-ADD (multiply-free): the consumer computes
--       step = (delta >> norm_shift) [+ (delta >> norm_shift_b)  when norm_use_b]
--   norm_shift is the primary (dominant) shift; norm_shift_b is a secondary
--   correction shift added only for the non-power-of-2 rate classes.
--
--     class  30/rate  scheme (rel. base)        achieved   wall-clock vs 30 Hz
--     60 Hz  x0.500   >>+1                       x0.500     1.00  (exact)
--     50 Hz  x0.600   >>+1 + >>+3                 x0.625     0.96  (-4%)
--     30 Hz  x1.000   >>+0            (ref)       x1.000     1.00
--     25 Hz  x1.200   >>+0 + >>+2                 x1.250     0.96  (-4%)
--     24 Hz  x1.250   >>+0 + >>+2                 x1.250     1.00  (exact)
--   Every class lands within +-4% of the 30 Hz reference (well inside +-25%);
--   30/60 Hz stay EXACT single shifts (norm_use_b='0') so they never regress.
--
--   Clean interface, no HOWLBOX-specific coupling inside -> liftable to the
--   shared kit unchanged (interlaced formats update per field, so 59.94 fields
--   is a 60 Hz class -- see LUT below).
--
-- Resource: small LUTs + adders/clamps, 1 register stage. 0 EBR. Multiply-free.
--   Advances at vsync (off the HD critical path).

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
    G_NUM_COEFFS : positive := 3        -- shared-kit default = 3 (HOWLBOX: starve/body/spin; WITNESS: LAG cx, LAG cy, ORBIT speed)
  );
  port (
    clk          : in  std_logic;
    timing_id    : in  unsigned(3 downto 0);                        -- registers_in(8) low nibble
    base_shift   : in  t_shift_array(G_NUM_COEFFS - 1 downto 0);    -- nominal shift @ 1080p30 ref
    norm_shift   : out t_shift_array(G_NUM_COEFFS - 1 downto 0);    -- primary  : step = (delta >> norm_shift)
    norm_shift_b : out t_shift_array(G_NUM_COEFFS - 1 downto 0);    -- secondary: + (delta >> norm_shift_b)
    norm_use_b   : out std_logic                                    -- 1 => add secondary (non-pow2 rate class)
  );
end entity rate_normalizer;

architecture rtl of rate_normalizer is

  -- Update-cadence rate class per timing id (video_timing_pkg encoding).
  -- da/db are shift deltas relative to base_shift; ub enables the secondary term.
  --   id : format        cadence  class  (da, db, ub)   scale
  --    0 : NTSC          59.94 fl   60    (1, 0, '0')    x0.5
  --    1 : 1080i50       50 fl      50    (1, 3, '1')    x0.625
  --    2 : 1080i5994     59.94 fl   60    (1, 0, '0')    x0.5
  --    3 : 1080p24       24 fr      24    (0, 2, '1')    x1.25
  --    4 : 480p          59.94 fr   60    (1, 0, '0')    x0.5
  --    5 : 720p50        50 fr      50    (1, 3, '1')    x0.625
  --    6 : 720p5994      59.94 fr   60    (1, 0, '0')    x0.5
  --    7 : 1080p30       30 fr      30    (0, 0, '0')    x1.0 (reference)
  --    8 : PAL           50 fl      50    (1, 3, '1')    x0.625
  --    9 : 1080p2398     23.98 fr   24    (0, 2, '1')    x1.25
  --   10 : 1080i60       60 fl      60    (1, 0, '0')    x0.5
  --   11 : 1080p25       25 fr      25    (0, 2, '1')    x1.25 (~x1.2)
  --   12 : 576p          50 fr      50    (1, 3, '1')    x0.625
  --   13 : 1080p2997     29.97 fr   30    (0, 0, '0')    x1.0
  --   14 : 720p60        60 fr      60    (1, 0, '0')    x0.5
  --   15 : (unused)                        (0, 0, '0')   x1.0
  type t_class is record
    da : integer range -2 to 3;
    db : integer range -2 to 3;
    ub : std_logic;
  end record;
  type t_class_lut is array (0 to 15) of t_class;
  constant C_CLASS : t_class_lut := (
    0  => (1, 0, '0'),  1  => (1, 3, '1'),  2  => (1, 0, '0'),  3  => (0, 2, '1'),
    4  => (1, 0, '0'),  5  => (1, 3, '1'),  6  => (1, 0, '0'),  7  => (0, 0, '0'),
    8  => (1, 3, '1'),  9  => (0, 2, '1'),  10 => (1, 0, '0'),  11 => (0, 2, '1'),
    12 => (1, 3, '1'),  13 => (0, 0, '0'),  14 => (1, 0, '0'),  15 => (0, 0, '0'));

begin

  p_norm : process (clk)
    variable v_c : t_class;
    variable v_a : integer;
    variable v_b : integer;
  begin
    if rising_edge(clk) then
      v_c := C_CLASS(to_integer(timing_id));
      norm_use_b <= v_c.ub;
      for i in 0 to G_NUM_COEFFS - 1 loop
        v_a := base_shift(i) + v_c.da;
        v_b := base_shift(i) + v_c.db;
        -- clamp into a sane, always-lagging range (never 0 = instant settle)
        if v_a < 1 then v_a := 1; elsif v_a > 31 then v_a := 31; end if;
        if v_b < 1 then v_b := 1; elsif v_b > 31 then v_b := 31; end if;
        norm_shift(i)   <= v_a;
        norm_shift_b(i) <= v_b;
      end loop;
    end if;
  end process p_norm;

end architecture rtl;
