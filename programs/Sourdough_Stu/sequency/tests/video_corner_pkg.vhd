-- ============================================================================
-- video_corner_pkg.vhd  --  Five-corner multi-frame verification harness (core)
-- ----------------------------------------------------------------------------
-- Phase-0 shared machinery, born in service of SEQUENCY (Program 5) Gate 2.
--
-- Defines the five broadcast "corner" timings used by every program's
-- functional testbench:  NTSC . 720p60 . 1080p30 . 1080i59.94 . 1080p24.
--
-- Geometry is TRUE horizontal blanking (clocks_per_line + active_width taken
-- verbatim from videomancer-sdk .../video_sync_pkg.vhd) so that:
--   * block/Bresenham tiling sees the real 720/1280/1920 active widths, and
--   * the horizontal blanking interval (>= pipeline depth) truly drains.
-- Vertical geometry is left to the testbench (reduced-raster for sim speed;
-- flip to true height for programs whose spec has vblank-iteration ceilings).
--
-- PROMOTABLE TO SHARED: this package has no SEQUENCY-specific content. Move to
-- videomancer-sdk/tests/vhdl/ (or a workspace tests/common/) when the 2nd
-- program needs it; the corner table and helpers are program-agnostic.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package video_corner_pkg is

  constant C_NCORNERS : integer := 5;

  type t_corner is record
    id              : integer;   -- ABI video_timing_id (registers_in(8))
    clocks_per_line : integer;   -- TRUE htotal from video_sync_pkg
    active_width    : integer;   -- TRUE active pixels/line (720/1280/1920)
    exp_blocks      : integer;   -- active_width / 16  (45/80/120)
    is_interlaced   : boolean;
  end record;

  type t_corner_arr is array (0 to C_NCORNERS - 1) of t_corner;

  -- Index order used by every corner testbench:
  --   0 NTSC . 1 720p60 . 2 1080p30(ref) . 3 1080i5994 . 4 1080p24
  constant CORNERS : t_corner_arr := (
    0 => (id =>  0, clocks_per_line =>  858, active_width =>  720, exp_blocks =>  45, is_interlaced => true),
    1 => (id => 14, clocks_per_line => 1650, active_width => 1280, exp_blocks =>  80, is_interlaced => false),
    2 => (id =>  7, clocks_per_line => 2200, active_width => 1920, exp_blocks => 120, is_interlaced => false),
    3 => (id =>  2, clocks_per_line => 2200, active_width => 1920, exp_blocks => 120, is_interlaced => true),
    4 => (id =>  3, clocks_per_line => 2750, active_width => 1920, exp_blocks => 120, is_interlaced => false)
  );

  function corner_name(i : integer) return string;
  function str_pf(ok : boolean) return string;   -- "PASS"/"FAIL" for RESULT lines

end package video_corner_pkg;

package body video_corner_pkg is
  function corner_name(i : integer) return string is
  begin
    case i is
      when 0      => return "NTSC";
      when 1      => return "720p60";
      when 2      => return "1080p30";
      when 3      => return "1080i5994";
      when 4      => return "1080p24";
      when others => return "UNKNOWN";
    end case;
  end function;

  function str_pf(ok : boolean) return string is
  begin
    if ok then return "PASS"; else return "FAIL"; end if;
  end function;
end package body video_corner_pkg;
