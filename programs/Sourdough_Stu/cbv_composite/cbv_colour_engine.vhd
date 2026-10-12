-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_colour_engine.vhd - CBV Composite per-line colour engine (block 9, spec §5.12)
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
-- Credits: Syntonie CBV001 Rev C schematic (CC BY-SA 4.0), Phil Baljeu's
-- Archer trace. Behavioural model only; no schematic artwork is copied.
--
-- Burst gate (keyed at the Y/C domain, stage 13, newest notch sample; Q9):
--   opens at the first sample of the line with hb = x_d + dh_l >= bst +
--   burst_off + 1 (burst_off = 0, dh_l = 0 in Stage 1) and lasts blen samples;
--   no gate on a line with vline = 1; a gate sample with AVID = 1 adds 0
--   (inhibit). Bsum_u/v = sum du/dv (sat s20), aligned at stage 19.
--   sw_tv toggles at each gate start when T9 = PAL-S; sw_pic = the value
--   after this line's toggle, used for the picture (Q12).
-- Serial engine after the gate closes (one line's work, ~75 clk):
--   PAL-S: (bu', bv') = ((Bu c + Bv s) >> 11, (Bv c - Bu s) >> 11) with c, s =
--          cos/sin(phi_g) from M1 (bit-serial multiply; Q6), CORDIC on
--          (-bu', |bv'|): ee = atan - 128; ident on bv' sw_tv.
--   NTSC : CORDIC on (Bu, Bv): beta; ee = wrap(phi_g + 512 - beta) (Q5: the
--          spec's wrap(beta - phi_g - 512) has the sign that makes the APC
--          diverge with APC_ph -= kp ee; flagged).
--   CORDIC: x < 0 -> pre-rotate by 1/2 cycle; 12 vectoring iterations,
--          y >= 0 -> rotate -atan(2^-i); mag = x (gain 1.64676), beta = z
--          (1/1024 cycle, 8 fraction bits). (Q6)
--   APC  : if mag > floor(MREF/20): apc_int = sat(apc_int + (ee>>11) + (ee>>13),
--          +-33.0), APC_ph -= (ee>>5) + (ee>>9); then APC_ph -= apc_int (mod 1024.0)
--   ACC  : d = mag<<13 - am; am += (d>>10) - (d>>13) (F = 13; reset MREF)
--   Killer: thr = floor(MREF/10); on: count lines am < thr; off: am > (thr*1149)>>10;
--          count > 1000 toggles col_on (Q13)
--   Divider: ACCq = min(16384, floor(MREF*4096/am)), restoring, 14 iterations
--          after the saturation test; gU/gV = (ACCq*K)>>12 (bit-serial)
-- Result timing (R-LATCH, §2.2): APC_ph, col_on, ident apply from line l+1
-- (consumers latch at their own x = 0); ACCq / gU / gV from line l apply from
-- l+2 (two-stage register at the Y/C-domain x = 0, consumers latch after).
-- Resources (paper): ~490 LC (CORDIC shifters dominate), no EBR, no `*`.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_pkg.all;

entity cbv_colour_engine is
  generic (
    G_CODEC_ONLY : boolean := false
  );
  port (
    clk        : in  std_logic;
    -- Y/C domain, stage 13
    i_x        : in  unsigned(11 downto 0);
    i_sol      : in  std_logic;
    i_vline    : in  std_logic;
    i_avid     : in  std_logic;
    i_tid      : in  t_tid;
    i_t9       : in  std_logic;
    i_dh       : in  signed(11 downto 0);    -- dh_l (0 in Stage 1)
    -- demod, stage 19
    i_du       : in  signed(17 downto 0);
    i_dv       : in  signed(17 downto 0);
    -- M1 line coefficients (cos/sin of phi_g, s12, 1.0 = 2047)
    i_cos      : in  signed(11 downto 0);
    i_sin      : in  signed(11 downto 0);
    -- per-line results
    o_apc_ph   : out unsigned(17 downto 0);  -- 10.8, 1/1024 cycle
    o_col_on   : out std_logic;
    o_sw_pic   : out std_logic;              -- '1' = sw_tv -1 for this line's picture
    o_gu_line  : out unsigned(15 downto 0);
    o_gv_line  : out unsigned(15 downto 0);
    o_accq_line: out unsigned(14 downto 0);
    -- T5 per-line record (valid on o_t5_stb)
    o_t5_stb   : out std_logic;
    o_bsum_u   : out signed(19 downto 0);
    o_bsum_v   : out signed(19 downto 0);
    o_mag      : out unsigned(23 downto 0);
    o_beta     : out unsigned(17 downto 0);
    o_ee       : out signed(17 downto 0);
    o_apc_int  : out signed(15 downto 0);
    o_am       : out unsigned(36 downto 0);
    o_accq     : out unsigned(14 downto 0);
    o_gu       : out unsigned(15 downto 0);
    o_gv       : out unsigned(15 downto 0);
    o_kcnt     : out unsigned(10 downto 0);
    o_sw_tv    : out std_logic;              -- '1' = -1
    o_idc      : out unsigned(2 downto 0)
  );
end entity cbv_colour_engine;

architecture rtl of cbv_colour_engine is

  function f_ku return integer is
  begin
    if G_CODEC_ONLY then return C_GU_CODEC; else return C_GU_FINAL; end if;
  end function;
  function f_kv return integer is
  begin
    if G_CODEC_ONLY then return C_GV_CODEC; else return C_GV_FINAL; end if;
  end function;
  constant C_KU : integer := f_ku;
  constant C_KV : integer := f_kv;

  -- gate (stage 13 -> 19)
  signal s_armed  : std_logic := '0';
  signal s_grem   : unsigned(6 downto 0) := (others => '0');
  type t_gp is array (14 to 19) of std_logic;
  signal s_gopen, s_gact, s_gin, s_glast, s_gt9 : t_gp := (others => '0');

  -- burst sums (stage 20)
  signal s_bu, s_bv : signed(19 downto 0) := (others => '0');
  signal s_start    : std_logic := '0';

  -- PAL switch
  signal s_sw_tv  : std_logic := '0';   -- '0' = +1
  signal s_sw_pic : std_logic := '0';
  signal s_idc    : unsigned(2 downto 0) := (others => '0');

  -- serial engine
  type t_state is (ST_IDLE, ST_LAT, ST_MULC, ST_MULS, ST_ROT, ST_CPRE, ST_CIT, ST_EE, ST_APC1, ST_APC2,
                   ST_AM1, ST_AM2, ST_AM3, ST_KILL, ST_DIV0, ST_DIV, ST_GAIN0, ST_GAIN, ST_DONE);
  signal s_st     : t_state := ST_IDLE;
  signal s_cnt    : unsigned(4 downto 0) := (others => '0');
  signal s_pal    : std_logic := '0';
  signal s_lbu, s_lbv : signed(19 downto 0) := (others => '0');
  signal s_lc, s_ls   : signed(11 downto 0) := (others => '0');
  -- bit-serial multipliers (two accumulators, shared iteration count)
  signal s_ma_a, s_ma_b : signed(21 downto 0) := (others => '0');  -- multiplicands
  signal s_mb_a, s_mb_b : signed(15 downto 0) := (others => '0');  -- multipliers (bit-serial)
  signal s_pa, s_pb     : signed(38 downto 0) := (others => '0');
  signal s_p1, s_p2     : signed(38 downto 0) := (others => '0');
  signal s_bvr          : signed(27 downto 0) := (others => '0');  -- bv'
  -- CORDIC
  signal s_cx, s_cy : signed(23 downto 0) := (others => '0');
  signal s_cz       : unsigned(17 downto 0) := (others => '0');
  -- results / state
  signal s_mag      : unsigned(23 downto 0) := (others => '0');
  signal s_beta     : unsigned(17 downto 0) := (others => '0');
  signal s_ee       : signed(17 downto 0) := (others => '0');
  signal s_apc_ph   : unsigned(17 downto 0) := (others => '0');
  signal s_apc_int  : signed(15 downto 0) := (others => '0');
  signal s_am       : unsigned(36 downto 0) := (others => '0');
  signal s_am_old   : unsigned(36 downto 0) := (others => '0');
  signal s_am_ok    : std_logic := '0';
  signal s_am_tid   : t_tid := (others => '0');
  signal s_d        : signed(38 downto 0) := (others => '0');
  signal s_t        : signed(38 downto 0) := (others => '0');
  signal s_col_on   : std_logic := '1';
  signal s_kcnt     : unsigned(10 downto 0) := (others => '0');
  signal s_rem      : unsigned(37 downto 0) := (others => '0');
  signal s_q        : unsigned(14 downto 0) := to_unsigned(C_ACCQ_RST, 15);
  -- results for l+1 / l+2 staging
  signal s_accq_res, s_accq_prev, s_accq_line : unsigned(14 downto 0) := to_unsigned(C_ACCQ_RST, 15);
  signal s_gu_res, s_gu_prev, s_gu_line : unsigned(15 downto 0) := to_unsigned(C_KU, 16);
  signal s_gv_res, s_gv_prev, s_gv_line : unsigned(15 downto 0) := to_unsigned(C_KV, 16);
  signal s_stb      : std_logic := '0';

  -- one bit-serial step: P := (P_hi +- a) & P_lo, then P := P sra 1
  function f_mstep(p : signed(38 downto 0); a : signed(21 downto 0); bit : std_logic;
                   last : boolean) return signed is
    variable v_hi : signed(22 downto 0);
    variable v_p  : signed(38 downto 0);
  begin
    v_hi := p(38 downto 16);
    if bit = '1' then
      if last then
        v_hi := v_hi - resize(a, 23);
      else
        v_hi := v_hi + resize(a, 23);
      end if;
    end if;
    v_p := v_hi & p(15 downto 0);
    return shift_right(v_p, 1);
  end function;

begin

  -- ------------------------------------------------------------------ burst gate
  p_gate : process(clk)
    variable f      : t_fmt;
    variable v_hb   : signed(13 downto 0);
    variable v_arm  : std_logic;
    variable v_open, v_act, v_last : std_logic;
  begin
    if rising_edge(clk) then
      f := f_fmt(i_tid);
      v_hb := resize(signed('0' & i_x), 14) + resize(i_dh, 14);
      v_arm := s_armed or i_sol;
      v_open := '0'; v_act := '0'; v_last := '0';
      if i_sol = '1' then
        s_grem <= (others => '0');
      end if;
      if s_grem /= 0 and i_sol = '0' then
        v_act := '1';
        s_grem <= s_grem - 1;
        if s_grem = 1 then
          v_last := '1';
        end if;
        s_armed <= v_arm;
      elsif v_arm = '1' and v_hb >= f.bst + 1 then
        s_armed <= '0';
        if i_vline = '0' then
          v_open := '1'; v_act := '1';
          s_grem <= to_unsigned(f.blen - 1, 7);
          if f.blen = 1 then v_last := '1'; end if;
        end if;
      else
        s_armed <= v_arm;
      end if;
      s_gopen(14) <= v_open;
      s_gact(14)  <= v_act;
      s_gin(14)   <= v_act and not i_avid;
      s_glast(14) <= v_last;
      s_gt9(14)   <= i_t9;
      s_gopen(15 to 19) <= s_gopen(14 to 18);
      s_gact(15 to 19)  <= s_gact(14 to 18);
      s_gin(15 to 19)   <= s_gin(14 to 18);
      s_glast(15 to 19) <= s_glast(14 to 18);
      s_gt9(15 to 19)   <= s_gt9(14 to 18);
    end if;
  end process;

  -- ------------------------------------------------------------------ burst sums, serial engine
  p_engine : process(clk)
    variable f      : t_fmt;
    variable v_du, v_dv : signed(19 downto 0);
    variable v_x, v_y   : signed(23 downto 0);
    variable v_sx, v_sy : signed(23 downto 0);
    variable v_i    : natural range 0 to 31;
    variable v_k    : signed(17 downto 0);
    variable v_ki   : signed(17 downto 0);
    variable v_int  : signed(16 downto 0);
    variable v_ph   : signed(18 downto 0);
    variable v_rot  : signed(39 downto 0);
    variable v_r2   : unsigned(37 downto 0);
    variable v_bm   : std_logic;
  begin
    if rising_edge(clk) then
      f := f_fmt(i_tid);
      s_stb <= '0';

      -- per-line staging for ACCq / gU / gV (l -> l+1 -> l+2)
      if i_sol = '1' then
        s_accq_prev <= s_accq_res;  s_accq_line <= s_accq_prev;
        s_gu_prev   <= s_gu_res;    s_gu_line   <= s_gu_prev;
        s_gv_prev   <= s_gv_res;    s_gv_line   <= s_gv_prev;
      end if;

      -- burst accumulation (stage 19 -> 20)
      if s_gin(19) = '1' then
        v_du := resize(i_du, 20); v_dv := resize(i_dv, 20);
      else
        v_du := (others => '0'); v_dv := (others => '0');
      end if;
      if s_gopen(19) = '1' then
        s_bu <= v_du;
        s_bv <= v_dv;
        if s_gt9(19) = '1' then
          s_sw_tv  <= not s_sw_tv;
          s_sw_pic <= not s_sw_tv;
        end if;
      elsif s_gact(19) = '1' then
        s_bu <= f_sat(resize(s_bu, 22) + resize(v_du, 22), 20);
        s_bv <= f_sat(resize(s_bv, 22) + resize(v_dv, 22), 20);
      end if;
      s_start <= s_glast(19);

      case s_st is
        when ST_IDLE =>
          if s_start = '1' then
            s_st <= ST_LAT;
          end if;

        when ST_LAT =>
          s_lbu <= s_bu;
          s_lbv <= s_bv;
          s_lc  <= i_cos;
          s_ls  <= i_sin;
          s_pal <= s_gt9(19);  -- T9 of this line (latched at the line start)
          if s_am_ok = '0' or s_am_tid /= i_tid then
            s_am_old <= shift_left(to_unsigned(f.mref, 37), 13);
          else
            s_am_old <= s_am;
          end if;
          s_am_ok  <= '1';
          s_am_tid <= i_tid;
          if s_gt9(19) = '1' then
            s_ma_a <= resize(s_bu, 22); s_mb_a <= resize(i_cos, 16);  -- Bu c
            s_ma_b <= resize(s_bv, 22); s_mb_b <= resize(i_cos, 16);  -- Bv c
            s_pa <= (others => '0'); s_pb <= (others => '0');
            s_cnt <= (others => '0');
            s_st <= ST_MULC;
          else
            s_cx <= resize(s_bu, 24);
            s_cy <= resize(s_bv, 24);
            s_st <= ST_CPRE;
          end if;

        when ST_MULC | ST_MULS | ST_GAIN =>
          v_i := to_integer(s_cnt);
          s_pa <= f_mstep(s_pa, s_ma_a, s_mb_a(v_i), v_i = 15);
          s_pb <= f_mstep(s_pb, s_ma_b, s_mb_b(v_i), v_i = 15);
          s_cnt <= s_cnt + 1;
          if v_i = 15 then
            s_cnt <= (others => '0');
            if s_st = ST_MULC then
              -- next: Bv s, Bu s (store Bu c, Bv c)
              s_p1 <= f_mstep(s_pa, s_ma_a, s_mb_a(v_i), true);
              s_p2 <= f_mstep(s_pb, s_ma_b, s_mb_b(v_i), true);
              s_ma_a <= resize(s_lbv, 22); s_mb_a <= resize(s_ls, 16);  -- Bv s
              s_ma_b <= resize(s_lbu, 22); s_mb_b <= resize(s_ls, 16);  -- Bu s
              s_pa <= (others => '0'); s_pb <= (others => '0');
              s_st <= ST_MULS;
            elsif s_st = ST_MULS then
              s_st <= ST_ROT;
            else
              s_st <= ST_DONE;
            end if;
          end if;

        when ST_ROT =>
          -- bu' = (Bu c + Bv s) >> 11, bv' = (Bv c - Bu s) >> 11
          v_rot := resize(s_p1, 40) + resize(s_pa, 40);
          v_rot := shift_right(v_rot, 11);
          s_cx  <= -resize(v_rot, 24);                         -- -bu'
          v_rot := resize(s_p2, 40) - resize(s_pb, 40);
          v_rot := shift_right(v_rot, 11);
          s_bvr <= resize(v_rot, 28);
          s_cy  <= resize(abs(v_rot), 24);                     -- |bv'|
          s_st  <= ST_CPRE;

        when ST_CPRE =>
          if s_cx < 0 then
            s_cx <= -s_cx;
            s_cy <= -s_cy;
            s_cz <= to_unsigned(131072, 18);   -- 1/2 cycle
          else
            s_cz <= (others => '0');
          end if;
          s_cnt <= (others => '0');
          s_st  <= ST_CIT;

        when ST_CIT =>
          v_i  := to_integer(s_cnt);
          v_sx := shift_right(s_cx, v_i);
          v_sy := shift_right(s_cy, v_i);
          if s_cy >= 0 then
            s_cx <= s_cx + v_sy;
            s_cy <= s_cy - v_sx;
            s_cz <= s_cz + to_unsigned(C_ATAN(v_i), 18);
          else
            s_cx <= s_cx - v_sy;
            s_cy <= s_cy + v_sx;
            s_cz <= s_cz - to_unsigned(C_ATAN(v_i), 18);
          end if;
          if v_i = 11 then
            s_st <= ST_EE;
          else
            s_cnt <= s_cnt + 1;
          end if;

        when ST_EE =>
          s_mag  <= unsigned(s_cx);
          s_beta <= s_cz;
          if s_pal = '1' then
            s_ee <= signed(s_cz - to_unsigned(32768, 18));                     -- atan - 128.0
          else
            s_ee <= signed(s_apc_ph + to_unsigned(131072, 18) - s_cz);         -- Q5
          end if;
          s_st <= ST_APC1;

        when ST_APC1 =>
          if s_mag > to_unsigned(f.mref20, 24) then
            v_ki  := shift_right(s_ee, 11) + shift_right(s_ee, 13);
            v_int := resize(s_apc_int, 17) + resize(v_ki, 17);
            s_apc_int <= f_satr(v_int, -8448, 8448, 16);
            v_k   := shift_right(s_ee, 5) + shift_right(s_ee, 9);
            v_ph  := signed('0' & s_apc_ph) - resize(v_k, 19);
            s_apc_ph <= unsigned(v_ph(17 downto 0));
          end if;
          -- ident (PAL-S): bv' sw_tv < 0 -> idc++, else idc-- ; idc > 3 -> flip
          if s_pal = '1' then
            if (s_bvr < 0 and s_sw_tv = '0') or (s_bvr > 0 and s_sw_tv = '1') then
              if s_idc = 3 then
                s_idc   <= (others => '0');
                s_sw_tv <= not s_sw_tv;
              else
                s_idc <= s_idc + 1;
              end if;
            elsif s_idc /= 0 then
              s_idc <= s_idc - 1;
            end if;
          end if;
          s_st <= ST_APC2;

        when ST_APC2 =>
          v_ph := signed('0' & s_apc_ph) - resize(s_apc_int, 19);
          s_apc_ph <= unsigned(v_ph(17 downto 0));
          s_st <= ST_AM1;

        when ST_AM1 =>
          s_d  <= signed(resize(shift_left(resize(s_mag, 37), 13), 39)) - signed(resize(s_am_old, 39));
          s_st <= ST_AM2;

        when ST_AM2 =>
          s_t  <= shift_right(s_d, 10) - shift_right(s_d, 13);
          s_st <= ST_AM3;

        when ST_AM3 =>
          s_am <= unsigned(resize(signed(resize(s_am_old, 39)) + s_t, 37));
          s_st <= ST_KILL;

        when ST_KILL =>
          if s_col_on = '1' then
            v_bm := '0';
            if s_am < shift_left(to_unsigned(f.thr, 37), 13) then v_bm := '1'; end if;
          else
            v_bm := '0';
            if s_am > shift_left(to_unsigned(f.thr_hi, 37), 13) then v_bm := '1'; end if;
          end if;
          if v_bm = '1' then
            if s_kcnt = 1000 then          -- count would pass 1000 -> toggle
              s_kcnt   <= (others => '0');
              s_col_on <= not s_col_on;
            else
              s_kcnt <= s_kcnt + 1;
            end if;
          else
            s_kcnt <= (others => '0');
          end if;
          s_st <= ST_DIV0;

        when ST_DIV0 =>
          s_cnt <= (others => '0');
          if resize(shift_left(to_unsigned(f.mref, 38), 11), 38) >= resize(s_am, 38) then
            s_q  <= to_unsigned(C_ACCQ_MAX, 15);
            s_st <= ST_GAIN0;
          else
            s_rem <= shift_left(to_unsigned(f.mref, 38), 11);
            s_q   <= (others => '0');
            s_st  <= ST_DIV;
          end if;

        when ST_DIV =>
          v_r2 := shift_left(s_rem, 1);
          if v_r2 >= resize(s_am, 38) then
            s_rem <= v_r2 - resize(s_am, 38);
            s_q   <= s_q(13 downto 0) & '1';
          else
            s_rem <= v_r2;
            s_q   <= s_q(13 downto 0) & '0';
          end if;
          if s_cnt = 13 then
            s_st <= ST_GAIN0;
          else
            s_cnt <= s_cnt + 1;
          end if;

        when ST_GAIN0 =>
          s_ma_a <= signed(resize(s_q, 22)); s_mb_a <= to_signed(C_KU, 16);
          s_ma_b <= signed(resize(s_q, 22)); s_mb_b <= to_signed(C_KV, 16);
          s_pa <= (others => '0'); s_pb <= (others => '0');
          s_cnt <= (others => '0');
          s_st <= ST_GAIN;

        when ST_DONE =>
          s_accq_res <= s_q;
          s_gu_res   <= unsigned(resize(shift_right(s_pa, 12), 16));
          s_gv_res   <= unsigned(resize(shift_right(s_pb, 12), 16));
          s_stb      <= '1';
          s_st       <= ST_IDLE;
      end case;
    end if;
  end process;

  o_apc_ph    <= s_apc_ph;
  o_col_on    <= s_col_on;
  o_sw_pic    <= s_sw_pic;
  o_gu_line   <= s_gu_line;
  o_gv_line   <= s_gv_line;
  o_accq_line <= s_accq_line;

  o_t5_stb  <= s_stb;
  o_bsum_u  <= s_lbu;
  o_bsum_v  <= s_lbv;
  o_mag     <= s_mag;
  o_beta    <= s_beta;
  o_ee      <= s_ee;
  o_apc_int <= s_apc_int;
  o_am      <= s_am;
  o_accq    <= s_accq_res;
  o_gu      <= s_gu_res;
  o_gv      <= s_gv_res;
  o_kcnt    <= s_kcnt;
  o_sw_tv   <= s_sw_tv;
  o_idc     <= s_idc;

end architecture rtl;
