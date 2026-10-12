-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_0_pkg.vhd - CBV Composite shared package (format table, ROM tables, helpers)
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
-- Credits:
--   Circuit facts modelled by this program come from the Syntonie CBV001
--   Rev C schematic (CC BY-SA 4.0) and Phil Baljeu's Archer trace. The
--   program models the circuit's behaviour; it copies no schematic artwork.
--
-- File name: the "0" sorts this package before every other program file, so
-- the SDK image tester / alignment runner (alphabetical + entity toposort,
-- packages not tracked) analyse it first. Synthesis (yosys-ghdl) is
-- order-independent.
--
-- Contents (spec = plans/cbv_composite/spec.md rev 2, spec-stage1.md):
--   t_fmt / f_fmt   per-timing-ID constants, spec §6.1 (+ §2.3 class table)
--   C_SIN_Q         quarter-wave sine ROM, 256 x 12, 1.0 = 2047 (spec §5.11)
--   C_ATAN          CORDIC arctangent table, 1/1024-cycle units with 8 fraction bits
--   C_KN            output knee table KN[k] = round(307 tanh(16k/307)), k = 0..64 (§5.14)
--   C_BURST_*       burst sample values per carrier index m (§5.2)
--   f_h             H*z = (z>>1)+(z>>3)+(z>>4)+(z>>6) (§5.2)
--   f_sat           saturation (R3: saturate, never wrap)
--   pipeline stage numbers (see cbv_composite_core.vhd header)
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package cbv_pkg is

  subtype t_tid is std_logic_vector(3 downto 0);

  -- format classes (spec §2.3)
  constant C_CLS_SD   : natural := 0;
  constant C_CLS_ED   : natural := 1;
  constant C_CLS_720  : natural := 2;
  constant C_CLS_1080 : natural := 3;

  type t_fmt is record
    cls      : natural range 0 to 3;
    p8       : boolean;      -- P = 8 samples per subcarrier cycle (else P = 4)
    tri      : boolean;      -- HD tri-level virtual sync
    d8       : natural range 0 to 7;  -- carrier phase step per line, eighth-cycles
    ye       : natural;      -- encoder luma delay YE (matches the 2-pole group delay)
    h2       : natural;      -- notch half-delay h/2 = P/4
    cpl      : natural;
    act0     : natural;
    actw     : natural;
    hs       : natural;
    hs2      : natural;      -- 2 hs (HD tri-level second half)
    bst      : natural;
    blen     : natural;
    lim      : natural;
    s        : natural;      -- rolling-base stride S = actw + lim + 4
    ca       : natural;      -- porch clamp window start
    clsh     : natural;      -- porch clamp window length = 2^clsh
    ca_in    : natural;      -- picture clamp window start (T11, Stage 3)
    clsh_in  : natural;
    porch_at : natural;
    ga       : natural;
    lag      : natural;      -- comb RAM modulo
    lagl     : natural;      -- LAG / actw (comb-enable first active line)
    mref     : natural;
    -- derived
    sd_v1    : natural;      -- cpl/2 - hs
    sd_v2    : natural;      -- cpl/2
    sd_v3    : natural;      -- cpl - hs
    hd_v     : natural;      -- floor(0.85 cpl)
    thr      : natural;      -- killer threshold floor(MREF/10)       (PROVISIONAL Q13)
    thr_hi   : natural;      -- (thr * 1149) >> 10                    (PROVISIONAL Q13)
    mref20   : natural;      -- floor(MREF/20)
    woff_uv  : natural;      -- chroma write offset h/2 + YE           (PROVISIONAL Q2)
  end record;

  function f_fmt(tid : t_tid) return t_fmt;

  -- quarter-wave sine, T[k] = round(2047 sin(2 pi k / 1024)), k = 0..255   (PROVISIONAL Q7)
  type t_sin_rom is array (0 to 255) of integer range 0 to 2047;
  constant C_SIN_Q : t_sin_rom := (
    0,13,25,38,50,63,75,88,100,113,126,138,151,163,176,188,201,213,226,238,251,263,275,288,
    300,313,325,338,350,362,375,387,399,412,424,436,449,461,473,485,497,510,522,534,546,558,
    570,582,594,606,618,630,642,654,666,678,690,701,713,725,737,748,760,772,783,795,807,818,
    830,841,852,864,875,887,898,909,920,932,943,954,965,976,987,998,1009,1020,1031,1042,1052,
    1063,1074,1085,1095,1106,1116,1127,1137,1148,1158,1168,1179,1189,1199,1209,1219,1229,1239,
    1249,1259,1269,1279,1289,1299,1308,1318,1328,1337,1347,1356,1365,1375,1384,1393,1402,1411,
    1421,1430,1439,1447,1456,1465,1474,1483,1491,1500,1508,1517,1525,1533,1542,1550,1558,1566,
    1574,1582,1590,1598,1606,1614,1621,1629,1637,1644,1652,1659,1666,1674,1681,1688,1695,1702,
    1709,1716,1723,1729,1736,1743,1749,1756,1762,1769,1775,1781,1787,1793,1799,1805,1811,1817,
    1823,1828,1834,1840,1845,1850,1856,1861,1866,1871,1876,1881,1886,1891,1896,1901,1905,1910,
    1914,1919,1923,1927,1932,1936,1940,1944,1948,1951,1955,1959,1962,1966,1969,1973,1976,1979,
    1983,1986,1989,1992,1994,1997,2000,2003,2005,2008,2010,2012,2015,2017,2019,2021,2023,2025,
    2027,2028,2030,2032,2033,2035,2036,2037,2038,2039,2040,2041,2042,2043,2044,2045,2045,2046,
    2046,2046,2047,2047,2047);

  -- CORDIC A_i = round(atan(2^-i) / (2 pi) * 2^18), i = 0..11        (PROVISIONAL Q6)
  type t_atan is array (0 to 11) of integer;
  constant C_ATAN : t_atan := (32768, 19344, 10221, 5188, 2604, 1303, 652, 326, 163, 81, 41, 20);

  -- knee, KN[k] = round(307 tanh(16 k / 307)), k = 0..64
  type t_kn is array (0 to 64) of integer range 0 to 511;
  constant C_KN : t_kn := (
    0,16,32,48,63,78,93,107,121,134,147,159,170,181,191,201,210,218,225,233,239,245,251,256,
    260,265,269,272,276,279,281,284,286,288,290,291,293,294,296,297,298,299,299,300,301,301,
    302,302,303,303,304,304,304,305,305,305,305,305,306,306,306,306,306,306,306);

  -- burst values, index m = 0..7 (spec §5.2, round = floor(z + 1/2))
  type t_burst is array (0 to 7) of integer range -256 to 255;
  constant C_BURST_NTSC  : t_burst := (0, -145, -205, -145, 0, 145, 205, 145);  -- round(-205 sin)
  constant C_BURST_PAL_P : t_burst := (145, 0, -145, -205, -145, 0, 145, 205);  -- sw = +1
  constant C_BURST_PAL_M : t_burst := (-145, -205, -145, 0, 145, 205, 145, 0);  -- sw = -1

  -- fixed constants
  constant C_SYNC_SD   : integer := -410;
  constant C_SYNC_HD   : integer := 430;
  constant C_KNEE_T    : integer := 716;
  constant C_GU_FINAL  : integer := 4242;  -- x8/7 x0.90625
  constant C_GV_FINAL  : integer := 3007;  -- x0.81013 x0.90625
  constant C_GU_CODEC  : integer := 4681;
  constant C_GV_CODEC  : integer := 3318;
  constant C_ACCQ_MAX  : integer := 16384;
  constant C_ACCQ_RST  : integer := 4096;

  -- pipeline stage numbers (clock index; stage 1 = input register). The
  -- "newest-sample tag" at stage s is the luma-tag input sample delayed
  -- s - 1 + YE clocks (cbv_composite_core.vhd header).
  constant C_ST_IN    : natural := 1;
  constant C_ST_ENC   : natural := 6;   -- encoder domain (Uf, Yd)
  constant C_ST_COMP  : natural := 10;  -- comp
  constant C_ST_CLAMP : natural := 10;  -- clamp input (o = comp, cell stub latency 0)
  constant C_ST_SD    : natural := 13;  -- sd; Y/C-domain registers
  constant C_ST_YC    : natural := 16;  -- chosen y, c
  constant C_ST_DU    : natural := 19;  -- du, dv; burst gate accumulation input
  constant C_ST_U4    : natural := 24;  -- U4, V4 (/4 valid)
  constant C_ST_M1    : natural := 30;  -- uu, vq
  constant C_ST_UV8   : natural := 35;  -- U8/V8, Yw, write domain registers
  constant C_ST_WR    : natural := 36;  -- write-port registers (RAM write on the next edge)

  function f_h(z : signed) return signed;
  function f_sat(z : signed; w : natural) return signed;
  function f_satr(z : signed; lo, hi : integer; w : natural) return signed;

end package cbv_pkg;

package body cbv_pkg is

  function f_mk(cls : natural; cpl, act0, hs, bst, blen, lim, ca, clsh, ca_in, clsh_in,
                porch_at, ga, lag, mref : natural) return t_fmt is
    variable r : t_fmt;
  begin
    r.cls      := cls;
    r.p8       := (cls >= C_CLS_720);
    r.tri      := (cls >= C_CLS_720);
    if cls = C_CLS_1080 then r.d8 := 4; else r.d8 := 2; end if;
    if r.p8 then r.ye := 6; r.h2 := 2; else r.ye := 3; r.h2 := 1; end if;
    r.cpl      := cpl;
    r.act0     := act0;
    if cls = C_CLS_720 then r.actw := 1280;
    elsif cls = C_CLS_1080 then r.actw := 1920;
    else r.actw := 720; end if;
    r.hs       := hs;
    r.hs2      := 2 * hs;
    r.bst      := bst;
    r.blen     := blen;
    r.lim      := lim;
    r.s        := r.actw + lim + 4;
    r.ca       := ca;
    r.clsh     := clsh;
    r.ca_in    := ca_in;
    r.clsh_in  := clsh_in;
    r.porch_at := porch_at;
    r.ga       := ga;
    r.lag      := lag;
    r.lagl     := lag / r.actw;
    r.mref     := mref;
    r.sd_v1    := cpl / 2 - hs;
    r.sd_v2    := cpl / 2;
    r.sd_v3    := cpl - hs;
    r.hd_v     := (cpl * 85) / 100;
    r.thr      := mref / 10;
    r.thr_hi   := (r.thr * 1149) / 1024;
    r.mref20   := mref / 20;
    r.woff_uv  := r.h2 + r.ye;
    return r;
  end function;

  -- spec §6.1 (act0 = cpl - hfp - fw from SDK video_sync_pkg.vhd)
  --                                   cls         cpl  act0  hs bst blen lim  CA clsh CA_in clsh_in porch GA   LAG   MREF
  constant C_F_NTSC  : t_fmt := f_mk(C_CLS_SD,    858, 119, 63,  68, 28, 32,  98, 4, 263, 5,  71, 171, 1440,  9452);
  constant C_F_PAL   : t_fmt := f_mk(C_CLS_SD,    864, 132, 63,  68, 36, 40, 106, 4, 276, 5,  71, 172, 1440, 12153);
  constant C_F_480P  : t_fmt := f_mk(C_CLS_ED,    858, 122, 63,  73, 28, 32, 103, 4, 266, 5,  79, 171, 1440,  9452);
  constant C_F_576P  : t_fmt := f_mk(C_CLS_ED,    864, 132, 63,  73, 36, 40, 111, 4, 276, 5,  79, 172, 1440, 12153);
  constant C_F_720A  : t_fmt := f_mk(C_CLS_720,  1650, 260, 40, 109, 72, 80, 183, 6, 516, 6, 124, 330, 2560, 24306);
  constant C_F_720B  : t_fmt := f_mk(C_CLS_720,  1980, 260, 40, 109, 72, 80, 183, 6, 516, 6, 124, 396, 2560, 24306);
  constant C_F_1080A : t_fmt := f_mk(C_CLS_1080, 2200, 192, 44, 117, 48, 56, 167, 4, 576, 6, 132, 440, 1920, 16204);
  constant C_F_1080B : t_fmt := f_mk(C_CLS_1080, 2640, 192, 44, 117, 48, 56, 167, 4, 576, 6, 132, 528, 1920, 16204);
  constant C_F_1080C : t_fmt := f_mk(C_CLS_1080, 2750, 192, 44, 117, 48, 56, 167, 4, 576, 6, 132, 550, 1920, 16204);

  function f_fmt(tid : t_tid) return t_fmt is
  begin
    case tid is
      when "0000" => return C_F_NTSC;    -- 0  ntsc
      when "1000" => return C_F_PAL;     -- 8  pal
      when "0100" => return C_F_480P;    -- 4  480p
      when "1100" => return C_F_576P;    -- 12 576p
      when "1110" => return C_F_720A;    -- 14 720p60
      when "0110" => return C_F_720A;    -- 6  720p59.94
      when "0101" => return C_F_720B;    -- 5  720p50
      when "1010" => return C_F_1080A;   -- 10 1080i60
      when "0010" => return C_F_1080A;   -- 2  1080i59.94
      when "0111" => return C_F_1080A;   -- 7  1080p30
      when "1101" => return C_F_1080A;   -- 13 1080p29.97
      when "0001" => return C_F_1080B;   -- 1  1080i50
      when "1011" => return C_F_1080B;   -- 11 1080p25
      when "0011" => return C_F_1080C;   -- 3  1080p24
      when "1001" => return C_F_1080C;   -- 9  1080p23.98
      when others => return C_F_NTSC;    -- 15 (no SDK timing): ntsc constants
    end case;
  end function;

  function f_h(z : signed) return signed is
  begin
    return shift_right(z, 1) + shift_right(z, 3) + shift_right(z, 4) + shift_right(z, 6);
  end function;

  function f_sat(z : signed; w : natural) return signed is
    constant C_MAX : signed(z'length - 1 downto 0) := resize(to_signed(2 ** (w - 1) - 1, w + 1), z'length);
    constant C_MIN : signed(z'length - 1 downto 0) := resize(to_signed(-(2 ** (w - 1)), w + 1), z'length);
  begin
    if z > C_MAX then return to_signed(2 ** (w - 1) - 1, w);
    elsif z < C_MIN then return to_signed(-(2 ** (w - 1)), w);
    else return resize(z, w); end if;
  end function;

  function f_satr(z : signed; lo, hi : integer; w : natural) return signed is
  begin
    if z > hi then return to_signed(hi, w);
    elsif z < lo then return to_signed(lo, w);
    else return resize(z, w); end if;
  end function;

end package body cbv_pkg;
