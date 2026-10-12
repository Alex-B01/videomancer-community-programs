-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_02_pkg.vhd - CBV Composite shared package (format table, bitstream selects, ROM tables, helpers)
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
-- Spec: plans/cbv_composite/spec.md rev 4.1 (+ rev-3 rulings Q1-Q17, §13).
-- Bitstream selects (R4.1-9): P, kc, k4c, the KSP grammar, the sync form and
-- the per-class constant sets come from work.core_config_pkg.C_ENABLE_HD at
-- elaboration (exact: sd_* bitstreams only run SD/ED timings, hd_* only HD).
-- SD-vs-ED and 720-vs-1080 stay runtime selects from the timing ID.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.core_config_pkg.all;
use work.cbv_variant_pkg.all;
use work.cbv_tables_pkg.all;

package cbv_pkg is

  subtype t_tid is std_logic_vector(3 downto 0);

  -- ------------------------------------------------------------ bitstream selects
  constant C_HD  : boolean := C_ENABLE_HD;          -- HD bitstream: P = 8, tri-level sync
  constant C_P8  : boolean := C_HD;
  constant C_H2  : natural := 2 - boolean'pos(not C_HD);           -- h/2 = P/4: 2 / 1
  constant C_YE  : natural := (3 + 3 * boolean'pos(C_HD)) * (1 - boolean'pos(C_ENC_1POLE))
                              + (2 + boolean'pos(C_HD)) * boolean'pos(C_ENC_1POLE);  -- 6 / 3 (G3: 3 / 2)
  constant C_KQ  : natural := 5 + 5 * boolean'pos(C_HD);           -- UV word offset 10 / 5 (Q2)
  constant C_R4V : natural := 1 + boolean'pos(C_HD);               -- /4 strobe residue 2 / 1 (Q3)

  -- format classes (spec §2.3)
  constant C_CLS_SD   : natural := 0;
  constant C_CLS_ED   : natural := 1;
  constant C_CLS_720  : natural := 2;
  constant C_CLS_1080 : natural := 3;

  type t_fmt is record
    cls      : natural range 0 to 3;
    ed       : boolean;      -- 27 MHz constant set (SD bitstream, ED class, unless G2)
    d8       : natural range 0 to 7;
    cpl      : natural;
    act0     : natural;
    actw     : natural;
    hs       : natural;
    hs2      : natural;
    bst      : natural;
    blen     : natural;
    lim      : natural;
    gi       : natural;
    s        : natural;
    ca       : natural;      -- CA_porch
    clsh     : natural;      -- window 2^clsh
    caspan   : natural;
    porch_at : natural;
    ga       : natural;
    syncend  : natural;
    lag      : natural;
    lagl     : natural;
    mref3    : natural;
    thr      : natural;
    thr_hi   : natural;
    rinv16   : natural;
    hpmin    : natural;
    hpmax    : natural;
    boff     : natural;
    sd_v1    : natural;      -- cpl/2 - hs
    sd_v2    : natural;      -- cpl/2
    sd_v3    : natural;      -- cpl - hs
    hd_v     : natural;      -- floor(0.85 cpl)
  end record;

  function f_fmt(tid : t_tid) return t_fmt;

  -- ------------------------------------------------------------ constant tables
  type t_kn is array (0 to 64) of integer range 0 to 511;
  constant C_KN : t_kn := (                    -- KN[k] = round(307 tanh(16k/307))
    0,16,32,48,63,78,93,107,121,134,147,159,170,181,191,201,210,218,225,233,239,245,251,256,
    260,265,269,272,276,279,281,284,286,288,290,291,293,294,296,297,298,299,299,300,301,301,
    302,302,303,303,304,304,304,305,305,305,305,305,306,306,306,306,306,306,306);
  type t_tanh is array (0 to 63) of integer range 0 to 256;
  constant C_TANH : t_tanh := (                -- TANH[k] = round(256 tanh(k/16))
    0,16,32,47,63,77,92,105,118,131,142,153,163,172,180,188,195,201,207,212,217,221,225,229,
    232,234,237,239,241,243,244,246,247,248,249,250,250,251,252,252,253,253,253,254,254,254,
    254,255,255,255,255,255,255,255,255,255,256,256,256,256,256,256,256,256);
  type t_t59 is array (0 to 58) of integer;
  constant C_BO : t_t59 := (                   -- BO[k] = round(256 (k/58)^1.5)
    0,1,2,3,5,6,9,11,13,16,18,21,24,27,30,34,37,41,44,48,52,56,60,64,68,72,77,81,86,91,95,100,
    105,110,115,120,125,130,136,141,147,152,158,163,169,175,181,187,193,199,205,211,217,224,
    230,236,243,249,256);
  constant C_DT16 : t_t59 := (                 -- DT16[k] = round(10066.3 (k/58)^2)
    0,3,12,27,48,75,108,147,192,242,299,362,431,506,587,673,766,865,970,1080,1197,1320,1448,
    1583,1724,1870,2023,2181,2346,2517,2693,2876,3064,3259,3459,3666,3878,4097,4321,4551,4788,
    5030,5279,5533,5793,6060,6332,6610,6894,7185,7481,7783,8091,8406,8726,9052,9384,9722,10066);
  type t_burst is array (0 to 7) of integer range -256 to 255;
  constant C_BURST_NTSC  : t_burst := (0, -145, -205, -145, 0, 145, 205, 145);
  constant C_BURST_PAL_P : t_burst := (145, 0, -145, -205, -145, 0, 145, 205);
  constant C_BURST_PAL_M : t_burst := (-145, -205, -145, 0, 145, 205, 145, 0);

  constant C_SYNC_SD  : integer := -410;
  constant C_SYNC_HD  : integer := 430;
  constant C_KNEE_T   : integer := 716;
  constant C_ACCQ_MAX : integer := 16384;
  constant C_ACCQ_RST : integer := 4096;

  -- ------------------------------------------------------------ shift one-pole constants (§6.2)
  -- (a, b, sigma) with k = 2^-a + sigma 2^-b; index 0 = 13.5 MHz, 1 = 27 MHz, 2 = 74.25 MHz
  type t_k2 is record
    a  : natural range 0 to 31;
    b  : natural range 0 to 31;
    sg : integer range -1 to 1;
  end record;
  type t_k2s is array (0 to 2) of t_k2;
  constant C_K_IN  : t_k2s := ((22, 23, 1), (23, 24, 1), (24, 24, 0));   -- 3*2^-23, 3*2^-24, 2^-24
  constant C_K_OUT : t_k2s := ((18, 21, 1), (19, 22, 1), (20, 23, -1));  -- 9*2^-21, 9*2^-22, 7*2^-23
  constant C_K_SH  : t_k2s := ((2, 4, 1), (3, 5, 1), (4, 10, 1));        -- 5/16, 5/32, 65/1024
  constant C_K_E   : t_k2s := ((0, 3, -1), (1, 3, 1), (2, 5, 1));        -- 7/8, 5/8, 9/32
  constant C_K_REL : t_k2s := ((16, 18, 1), (17, 19, 1), (18, 23, -1));  -- 5*2^-18, 5*2^-19, 31*2^-23
  -- bitstream-fixed: kc (encoder), k4c (/4 poles)
  constant C_KC  : t_k2 := (a => 1 + boolean'pos(C_HD), b => 4 + 4 * boolean'pos(C_HD), sg => -1);
                                    -- 7/16 = 2^-1 - 2^-4 | 63/256 = 2^-2 - 2^-8
  constant C_K4C : t_k2 := (a => boolean'pos(C_HD), b => 3, sg => 2 * boolean'pos(C_HD) - 1);
                                    -- 7/8 = 2^0 - 2^-3   | 5/8 = 2^-1 + 2^-3
  -- constant-set index of this bitstream for a format (runtime ED select in SD)
  function f_kidx(ed : boolean) return natural;

  -- ------------------------------------------------------------ pipeline stage numbers
  -- Stage s = clock index after the input register (stage 1). The newest-sample
  -- tag at stage s >= 6 is the luma-tag input sample from s - 1 + YE clocks before.
  constant C_ST_ENC   : natural := 6;                    -- encoder domain
  constant C_ST_COMP  : natural := 10;                   -- comp (T1)
  constant C_ST_VIN   : natural := C_ST_COMP + 3 + 2 * boolean'pos(C_ENH_INTERP);   -- v_in (T1b)
  constant C_ST_O     : natural := C_ST_VIN + 15;        -- o (T1c)
  constant C_ST_S     : natural := C_ST_O + 5;           -- separator input s (T1d/T1e)
  constant C_ST_DH    : natural := C_ST_S + 17;          -- dh_l (T1e)
  constant C_ST_CLAMP : natural := C_ST_DH;              -- clamp input (o delayed to dh_l's tag)
  constant C_ST_SD    : natural := C_ST_CLAMP + 3;       -- sd; Y/C domain (T2)
  constant C_ST_YC    : natural := C_ST_SD + 3;          -- y, cS (T3)
  constant C_ST_DU    : natural := C_ST_YC + 3;          -- du, dv; gate accumulation input
  constant C_ST_I4    : natural := C_ST_DU + 1;          -- I4 strobe (T4)
  constant C_ST_U4    : natural := C_ST_I4 + 8;          -- U4, V4 from the /4 poles (commit + 4 slots)
  constant C_ST_UV8   : natural := C_ST_U4 + 6;          -- U8/V8 strobe, write domain
  constant C_ST_WR    : natural := C_ST_UV8 + 1;         -- write-port registers (T6), L_w


  -- ------------------------------------------------------------ Stage-3 taps (spec-stage2 §7)
  -- Driven on cbv_composite_core's o_taps port (open in program_top: pruned).
  -- Each field is on its own stage; x_* are the tag x of the named domains.
  type t_cbv_taps is record
    -- T1 (C_ST_COMP)
    comp : signed(13 downto 0); m_c : unsigned(2 downto 0); m_b : unsigned(2 downto 0);
    kh : unsigned(2 downto 0); psi : unsigned(2 downto 0);
    -- T1b (C_ST_VIN)
    v_in : signed(13 downto 0); row : unsigned(4 downto 0);
    -- T1c (w VIN+4, hf VIN+7, vv VIN+11, o C_ST_O)
    w : signed(13 downto 0); hf : signed(14 downto 0); vv : signed(13 downto 0); o : signed(13 downto 0);
    -- T1d (ea, col at O; pic O+1; leak O+4)
    ea : unsigned(11 downto 0); pic : unsigned(11 downto 0); col : unsigned(9 downto 0);
    leak : unsigned(11 downto 0);
    -- T1e (s C_ST_S; f S+5; ti S+6; bp, sl S+7; hphase, hper, ef S+8; dh8 S+10; z S+14; dh_l C_ST_DH)
    s : signed(13 downto 0); f : signed(13 downto 0); ti : signed(13 downto 0); bp : signed(13 downto 0);
    sl : std_logic; hphase : unsigned(19 downto 0); hper : unsigned(19 downto 0); ef : signed(20 downto 0);
    dh8 : signed(20 downto 0); z : unsigned(5 downto 0); dh_l : signed(11 downto 0);
    -- T2 (C_ST_SD; CA per line)
    sd : signed(15 downto 0); ca : unsigned(11 downto 0); bpk : signed(15 downto 0); bpk_ld : std_logic;
    -- T3 (C_ST_YC)
    y : signed(15 downto 0); cs : signed(12 downto 0); comb_en : std_logic; cr_old : signed(9 downto 0);
    -- T4 (md, du, dv C_ST_DU; strobe, I4 C_ST_I4; commit/pole inputs I4+4; U4/V4 on u4_valid)
    md : unsigned(2 downto 0); du : signed(14 downto 0); dv : signed(14 downto 0);
    strobe : std_logic; i4u : signed(14 downto 0); i4v : signed(14 downto 0);
    commit : std_logic; pin_u : signed(14 downto 0); pin_v : signed(14 downto 0);
    u4 : signed(14 downto 0); v4 : signed(14 downto 0); u4_valid : std_logic;
    -- T5 (per line, on t5_stb)
    t5_stb : std_logic; bu : signed(19 downto 0); bv : signed(19 downto 0); mag : unsigned(20 downto 0);
    sw_tv : std_logic; sw_pic : std_logic; idc : unsigned(2 downto 0); am : unsigned(36 downto 0);
    kcnt : unsigned(10 downto 0); col_on : std_logic; accq : unsigned(14 downto 0);
    gu9 : unsigned(11 downto 0); gv9 : unsigned(11 downto 0); codes : std_logic_vector(19 downto 0);
    da : unsigned(15 downto 0); dm_l : unsigned(2 downto 0);
    -- T6 (C_ST_WR)
    we_y : std_logic; wa_y : unsigned(10 downto 0); yw : unsigned(9 downto 0);
    we_uv : std_logic; wa_uv : unsigned(8 downto 0); q : signed(11 downto 0);
    u8 : unsigned(7 downto 0); v8 : unsigned(7 downto 0); b : unsigned(10 downto 0); a0_m : unsigned(11 downto 0);
    -- tags
    x_o : unsigned(11 downto 0); x_s : unsigned(11 downto 0);
    x_cl : unsigned(11 downto 0); x_yc : unsigned(11 downto 0); x_wr : unsigned(11 downto 0);
  end record;

  -- ------------------------------------------------------------ helpers
  function f_sat(z : signed; w : natural) return signed;
  function f_satr(z : signed; lo, hi : integer; w : natural) return signed;
  -- a + b (sub = '0') or a - b (sub = '1') as ONE carry chain: a + (b xor sub) + sub
  function f_addsub(a, b : signed; sub : std_logic) return signed;
  -- b or -b (neg = '1'), one incrementer: (b xor neg) + neg
  function f_negif(b : signed; neg : std_logic) return signed;
  -- CSD2(g) (spec §5.1): p1 = 2^e1 nearest g (ties to the larger), r = g - p1;
  -- p2 = 2^e2 nearest |r| (ties larger), sg = sign(r) (0: r = 0). g >= 1.
  -- (Synthesizable: used per line in the colour engine.)
  type t_csd2 is record
    e1 : natural range 0 to 15;
    e2 : natural range 0 to 15;
    sg : integer range -1 to 1;
  end record;
  function f_csd2(g : unsigned(11 downto 0)) return t_csd2;

end package cbv_pkg;

package body cbv_pkg is

  function f_kidx(ed : boolean) return natural is
  begin
    if C_HD then return 2;
    elsif ed and not C_SD_ONE_SET then return 1;
    else return 0; end if;
  end function;

  function f_mk(cls : natural; cpl, act0, hs, bst, blen, lim, ca, clsh, caspan, porch_at, ga, lag,
                rinv16 : natural; gi : natural) return t_fmt is
    variable r : t_fmt;
  begin
    r.cls      := cls;
    r.ed       := (cls = C_CLS_ED);
    if cls = C_CLS_1080 then r.d8 := 4; else r.d8 := 2; end if;
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
    r.gi       := gi;
    r.s        := r.actw + lim + 4;
    r.ca       := ca;
    r.clsh     := clsh;
    r.caspan   := caspan;
    r.porch_at := porch_at;
    r.ga       := ga;
    if cls >= C_CLS_720 then r.syncend := 2 * hs + 4; else r.syncend := hs + 2; end if;
    r.lag      := lag;
    r.lagl     := lag / r.actw;
    r.mref3    := 205 * blen;
    r.thr      := r.mref3 / 10;
    r.thr_hi   := (r.thr * 1149) / 1024;
    r.rinv16   := rinv16;
    r.hpmin    := (256 * cpl * 956 + 500) / 1000;
    r.hpmax    := (256 * cpl * 1044 + 500) / 1000;
    r.boff     := (85 * blen + 50) / 100;
    r.sd_v1    := cpl / 2 - hs;
    r.sd_v2    := cpl / 2;
    r.sd_v3    := cpl - hs;
    r.hd_v     := (cpl * 85) / 100;
    return r;
  end function;

  -- spec §6.1. GI from cbv_tables_pkg (TUNE).
  --                                      cls      cpl  act0 hs bst blen lim  CA clsh span porch GA   LAG  RINV16  GI
  function f_fmt(tid : t_tid) return t_fmt is
    variable g : natural;
  begin
    g := C_GI(to_integer(unsigned(tid)));
    case tid is
      when "0000" => return f_mk(C_CLS_SD,    858, 119, 63,  68, 28, 32,  98, 4, 309,  71, 171, 1440, 2048, g);  -- ntsc
      when "1000" => return f_mk(C_CLS_SD,    864, 132, 63,  68, 36, 40, 106, 4, 314,  71, 172, 1440, 1638, g);  -- pal
      when "0100" => return f_mk(C_CLS_ED,    858, 122, 63,  73, 28, 32, 103, 4, 307,  79, 171, 1440, 2048, g);  -- 480p
      when "1100" => return f_mk(C_CLS_ED,    864, 132, 63,  73, 36, 40, 111, 4, 309,  79, 172, 1440, 1638, g);  -- 576p
      when "1110" | "0110" =>
                     return f_mk(C_CLS_720,  1650, 260, 40, 109, 72, 80, 183, 6, 589, 124, 330, 2560,  819, g);  -- 720p60/59.94
      when "0101" => return f_mk(C_CLS_720,  1980, 260, 40, 109, 72, 80, 183, 6, 589, 124, 396, 2560,  819, g);  -- 720p50
      when "1010" | "0010" | "0111" | "1101" =>
                     return f_mk(C_CLS_1080, 2200, 192, 44, 117, 48, 56, 167, 4, 793, 132, 440, 1920, 1170, g);  -- 1080 2200
      when "0001" | "1011" =>
                     return f_mk(C_CLS_1080, 2640, 192, 44, 117, 48, 56, 167, 4, 793, 132, 528, 1920, 1170, g);  -- 1080 2640
      when "0011" | "1001" =>
                     return f_mk(C_CLS_1080, 2750, 192, 44, 117, 48, 56, 167, 4, 793, 132, 550, 1920, 1170, g);  -- 1080p24
      when others => return f_mk(C_CLS_SD,    858, 119, 63,  68, 28, 32,  98, 4, 309,  71, 171, 1440, 2048, g);  -- 15: ntsc
    end case;
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

  function f_addsub(a, b : signed; sub : std_logic) return signed is
    variable v_b  : signed(a'length - 1 downto 0);
    variable v_ci : signed(a'length - 1 downto 0) := (others => '0');
  begin
    v_b := resize(b, a'length) xor (a'length - 1 downto 0 => sub);
    v_ci(0) := sub;
    return a + v_b + v_ci;
  end function;

  function f_negif(b : signed; neg : std_logic) return signed is
    variable v_ci : signed(b'length - 1 downto 0) := (others => '0');
  begin
    v_ci(0) := neg;
    return (b xor (b'length - 1 downto 0 => neg)) + v_ci;
  end function;

  -- nearest power of two (linear distance, ties to the larger) of n >= 1:
  -- with 2^e <= n < 2^(e+1), n >= 1.5 * 2^e iff bit e-1 of n is set.
  function f_np2(n : unsigned(12 downto 0)) return natural is
    variable e : natural := 0;
  begin
    for k in 0 to 12 loop
      if n(k) = '1' then e := k; end if;
    end loop;
    if e > 0 then
      if n(e - 1) = '1' then return e + 1; end if;
    end if;
    return e;
  end function;

  function f_csd2(g : unsigned(11 downto 0)) return t_csd2 is
    variable r  : t_csd2;
    variable v_r : signed(13 downto 0);
    variable v_a : unsigned(12 downto 0);
  begin
    r.e1 := f_np2('0' & g);
    v_r := signed("00" & g) - shift_left(to_signed(1, 14), r.e1);
    if v_r = 0 then
      r.e2 := 0; r.sg := 0;
    else
      v_a := resize(unsigned(abs(v_r)), 13);
      r.e2 := f_np2(v_a);
      if v_r > 0 then r.sg := 1; else r.sg := -1; end if;
    end if;
    return r;
  end function;

end package body cbv_pkg;
