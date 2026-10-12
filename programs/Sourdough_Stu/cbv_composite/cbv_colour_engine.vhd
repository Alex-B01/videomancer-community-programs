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
-- Burst gate (keyed on tag n, the newest notch sample, Y/C domain stage 13;
--   RULED Q9, spec rev 3 §13): opens at the first sample of the line with
--   n.x + dh_l >= bst + burst_off + P/4 (burst_off = 0, dh_l = 0 in Stage 1)
--   and lasts blen samples;
--   no gate on a line with vline = 1; a gate sample with AVID = 1 adds 0
--   (inhibit). Bsum_u/v = sum du/dv (sat s20), aligned at stage 19.
--   sw_tv toggles at each gate start when T9 = PAL-S; sw_pic = the value
--   after this line's toggle, used for the picture (RULED Q12).
-- Serial engine after the gate closes (one line's work, <= ~200 clk; the
-- shortest budget is ~740 clk: SD/ED, gate end ~x 110 of cpl 858):
--   PAL-S: (bu', bv') = ((Bu c + Bv s) >> 11, (Bv c - Bu s) >> 11) with c, s =
--          cos/sin(phi_g) from M1, CORDIC on (-bu', |bv'|): ee = beta - 32768;
--          ident on bv' sw_tv (RULED Q6, serial rotation ruled).
--   NTSC : CORDIC on (Bu, Bv): ee = wrap(Phi_g + 131072 - beta) (RULED Q5).
--   CORDIC: x < 0 -> pre-rotate by 1/2 cycle; 12 vectoring iterations,
--          y >= 0 -> (x + y>>i, y - x>>i, z + A_i); mag = x, beta = z (RULED Q6)
--   APC  : if mag > floor(MREF/20): apc_int = sat(apc_int + (ee>>11) + (ee>>13),
--          +-8448), APC_ph -= (ee>>5) + (ee>>9); then APC_ph -= apc_int (mod 2^18)
--   ACC  : d = mag<<13 - am_old; am = am_old + (d>>10) - (d>>13) (u37; am_old =
--          MREF<<13 on the first run and on a timing-ID change)
--   Killer: thr = floor(MREF/10); on: am < thr<<13; off: am > thr_hi<<13;
--          the 1,001st consecutive line toggles col_on (RULED Q13)
--   Divider: ACCq = min(16384, floor(MREF*2^25/am)), restoring, 14 iterations
--          after the saturation test; gU9/gV9 = (ACCq*K)>>15, u12 (RULED Q14)
-- Result timing (R-LATCH, §2.2): APC_ph, col_on, ident apply from line l+1
-- (consumers latch at their own x = 0); ACCq / gU9 / gV9 from line l apply from
-- l+2 (two-stage register at the Y/C-domain x = 0, consumers latch after).
--
-- Area structure (values identical to the direct formulas above):
--   * per-format constants registered once per line at x = 0 (no f_fmt()
--     lookups in the arithmetic paths);
--   * products: two right-shift bit-serial accumulators, 14 steps (c, s sign-
--     extended), each step adding c_k Bu + s_k Bv (resp. c_k Bv - s_k Bu) via a
--     4-way addend select {0, Bu, Bv, Bu+Bv} -- both products of a sum in one
--     pass, no stored partial products; the same pair forms ACCq*KU, ACCq*KV;
--   * CORDIC: x>>i and y>>i by i one-bit shifts of copy registers (no barrel
--     shifters); the x/y add/sub units also perform the source load and the
--     pre-rotation negation;
--   * divider: one subtractor whose sign is the compare;
--   * threshold compares on the upper bits only (low 13 bits of thr<<13 are 0).
-- Resources (paper): ~1,000 LUT4, no EBR, no `*`.
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
    i_du       : in  signed(14 downto 0);
    i_dv       : in  signed(14 downto 0);
    -- M1 line coefficients (cos/sin of phi_g, s12, 1.0 = 2047)
    i_cos      : in  signed(11 downto 0);
    i_sin      : in  signed(11 downto 0);
    -- per-line results
    o_apc_ph   : out unsigned(17 downto 0);  -- 10.8, 1/1024 cycle
    o_col_on   : out std_logic;
    o_sw_pic   : out std_logic;              -- '1' = sw_tv -1 for this line's picture
    o_gu_line  : out unsigned(11 downto 0);   -- gU9
    o_gv_line  : out unsigned(11 downto 0);   -- gV9
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
    o_gu       : out unsigned(11 downto 0);
    o_gv       : out unsigned(11 downto 0);
    o_bur      : out signed(27 downto 0);    -- bu' (PAL-S lines)
    o_bvr      : out signed(27 downto 0);    -- bv'
    o_sw_pic_r : out std_logic;
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
  constant C_KU  : integer := f_ku;
  constant C_KV  : integer := f_kv;
  constant C_GU0 : integer := (C_ACCQ_RST * C_KU) / 32768;   -- 530 / 585
  constant C_GV0 : integer := (C_ACCQ_RST * C_KV) / 32768;   -- 375 / 414
  constant C_KUV : signed(13 downto 0) := to_signed(C_KU, 14);
  constant C_KVV : signed(13 downto 0) := to_signed(C_KV, 14);
  constant C_F0  : t_fmt := f_fmt("0000");

  -- per-line format registers (latched at the Y/C-domain x = 0)
  signal r_bsth  : unsigned(11 downto 0) := to_unsigned(C_F0.bst + C_F0.h2, 12);
  signal r_blen  : unsigned(6 downto 0)  := to_unsigned(C_F0.blen, 7);
  signal r_mref  : unsigned(14 downto 0) := to_unsigned(C_F0.mref, 15);
  signal r_mref20: unsigned(10 downto 0) := to_unsigned(C_F0.mref20, 11);
  signal r_thr   : unsigned(11 downto 0) := to_unsigned(C_F0.thr, 12);
  signal r_thrhi : unsigned(11 downto 0) := to_unsigned(C_F0.thr_hi, 12);
  signal r_t9    : std_logic := '0';

  -- gate (stage 13 -> 19)
  signal s_armed  : std_logic := '0';
  signal s_grem   : unsigned(6 downto 0) := (others => '0');
  type t_gp is array (14 to 19) of std_logic;
  signal s_gopen, s_gact, s_gin, s_glast, s_gt9 : t_gp := (others => '0');

  -- burst sums (stage 20; also the T5 Bsum values)
  signal s_bu, s_bv : signed(19 downto 0) := (others => '0');
  signal s_start    : std_logic := '0';

  -- PAL switch
  signal s_sw_tv  : std_logic := '0';   -- '0' = +1
  signal s_sw_pic : std_logic := '0';
  signal s_idc    : unsigned(2 downto 0) := (others => '0');

  type t_state is (ST_IDLE, ST_INIT, ST_MUL, ST_SET0, ST_SET1, ST_CLD, ST_CSH, ST_CUP, ST_EE,
                   ST_APC1, ST_APC2, ST_AM1, ST_AM2, ST_AM3, ST_KILL, ST_DIV0, ST_DIV1, ST_DIV,
                   ST_GAIN, ST_DONE);
  signal s_st    : t_state := ST_IDLE;
  signal s_cnt   : unsigned(3 downto 0) := (others => '0');   -- multiply / divide step
  signal s_it    : unsigned(3 downto 0) := (others => '0');   -- CORDIC iteration
  signal s_shc   : unsigned(3 downto 0) := (others => '0');   -- shift countdown
  signal s_pal   : std_logic := '0';
  signal s_gain  : std_logic := '0';   -- multiplier mode: '1' = ACCq * K
  signal s_neg   : std_logic := '0';   -- pre-rotation taken
  signal s_bvn   : std_logic := '0';   -- bv' < 0

  -- bit-serial accumulators (hi 23 bits + lo 14 bits)
  signal s_suv, s_dvu : signed(20 downto 0) := (others => '0');   -- Bu+Bv, Bv-Bu (gain: ACCq)
  signal s_pu, s_pv   : signed(36 downto 0) := (others => '0');
  signal s_bur, s_bvr : signed(23 downto 0) := (others => '0');   -- bu', bv' (T5)

  -- CORDIC
  signal s_cx, s_cy   : signed(23 downto 0) := (others => '0');
  signal s_xs, s_ys   : signed(23 downto 0) := (others => '0');
  signal s_cz         : unsigned(17 downto 0) := (others => '0');

  -- results / state
  signal s_ee       : signed(17 downto 0) := (others => '0');
  signal s_apc_ph   : unsigned(17 downto 0) := (others => '0');
  signal s_apc_int  : signed(15 downto 0) := (others => '0');
  signal s_am       : unsigned(36 downto 0) := shift_left(to_unsigned(C_F0.mref, 37), 13);
  signal s_am_ok    : std_logic := '0';
  signal s_am_tid   : t_tid := (others => '0');
  signal s_d        : signed(38 downto 0) := (others => '0');   -- d, then t
  signal s_col_on   : std_logic := '1';
  signal s_kcnt     : unsigned(10 downto 0) := (others => '0');
  signal s_rem      : unsigned(37 downto 0) := (others => '0');
  signal s_q        : unsigned(14 downto 0) := to_unsigned(C_ACCQ_RST, 15);
  signal s_accq_res, s_accq_prev, s_accq_line : unsigned(14 downto 0) := to_unsigned(C_ACCQ_RST, 15);
  signal s_gu_res, s_gu_prev, s_gu_line : unsigned(11 downto 0) := to_unsigned(C_GU0, 12);
  signal s_gv_res, s_gv_prev, s_gv_line : unsigned(11 downto 0) := to_unsigned(C_GV0, 12);
  signal s_stb      : std_logic := '0';

  -- datapath wires
  signal w_k        : natural range 0 to 15;
  signal w_cb, w_sb : std_logic;        -- multiplier bits of this step (U, V)
  signal w_au, w_av : signed(20 downto 0);
  signal w_selu, w_selv : std_logic_vector(1 downto 0);
  signal w_subu, w_subv : std_logic;
  signal w_last     : boolean;
  signal w_hu, w_hv : signed(22 downto 0);
  signal w_xop, w_yop : std_logic;      -- '1' = subtract
  signal w_xn, w_yn : signed(23 downto 0);
  signal w_zn       : unsigned(17 downto 0);

begin

  -- ------------------------------------------------------------------ burst gate
  p_gate : process(clk)
    variable f      : t_fmt;
    variable v_hb   : signed(13 downto 0);
    variable v_arm  : std_logic;
    variable v_open, v_act, v_last : std_logic;
  begin
    if rising_edge(clk) then
      if i_sol = '1' then
        f := f_fmt(i_tid);
        r_bsth   <= to_unsigned(f.bst + f.h2, 12);
        r_blen   <= to_unsigned(f.blen, 7);
        r_mref   <= to_unsigned(f.mref, 15);
        r_mref20 <= to_unsigned(f.mref20, 11);
        r_thr    <= to_unsigned(f.thr, 12);
        r_thrhi  <= to_unsigned(f.thr_hi, 12);
        r_t9     <= i_t9;
      end if;
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
      elsif v_arm = '1' and i_sol = '0' and v_hb >= signed('0' & r_bsth) then
        s_armed <= '0';
        if i_vline = '0' then
          v_open := '1'; v_act := '1';
          s_grem <= r_blen - 1;
          if r_blen = 1 then v_last := '1'; end if;
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

  -- ------------------------------------------------------------------ datapath (combinational)
  -- multiplier step k: bits of (c, s) sign-extended to 14 bits, or of (KU, KV)
  w_k    <= to_integer(s_cnt);
  w_cb   <= C_KUV(w_k) when s_gain = '1' else i_cos(minimum(w_k, 11));
  w_sb   <= C_KVV(w_k) when s_gain = '1' else i_sin(minimum(w_k, 11));
  w_last <= (w_k = 13);
  -- addends: U = c_k Bu + s_k Bv ; V = c_k Bv - s_k Bu  (gain: U = KU_k q, V = KV_k q)
  -- 2-bit selects; in gain mode both select bits are the K bit, picking s_suv / s_dvu (= ACCq)
  w_selu <= (w_cb & w_cb) when s_gain = '1' else (w_cb & w_sb);
  w_selv <= (w_sb & w_sb) when s_gain = '1' else (w_cb & w_sb);
  with w_selu select w_au <=
    s_suv            when "11",
    resize(s_bu, 21) when "10",
    resize(s_bv, 21) when "01",
    (others => '0')  when others;
  -- V: c_k Bv - s_k Bu = (10) +Bv, (01) -Bu, (11) +(Bv - Bu): magnitude select + sign
  with w_selv select w_av <=
    s_dvu            when "11",
    resize(s_bv, 21) when "10",
    resize(s_bu, 21) when "01",
    (others => '0')  when others;
  w_subu <= '1' when w_last else '0';
  w_subv <= '1' when (w_selv = "01") xor w_last else '0';
  w_hu <= f_addsub(s_pu(36 downto 14), w_au, w_subu);
  w_hv <= f_addsub(s_pv(36 downto 14), w_av, w_subv);

  -- CORDIC add/sub units: x' = x -+ ys, y' = y -+ xs, z' = z -+ A_i
  w_xn <= f_addsub(s_cx, s_ys, w_xop);
  w_yn <= f_addsub(s_cy, s_xs, w_yop);
  w_zn <= unsigned(f_addsub(signed(s_cz), signed(to_unsigned(C_ATAN(to_integer(s_it)), 18)), s_cy(23)));
  -- op select: SET1 (load / pre-rotation) or vectoring step (y >= 0: x + ys, y - xs)
  w_xop <= (s_pal xor s_neg) when s_st = ST_SET1 else
           '1' when s_cy < 0 else '0';
  w_yop <= ((s_pal and s_bvn) xor s_neg) when s_st = ST_SET1 else
           '0' when s_cy < 0 else '1';


  -- ------------------------------------------------------------------ burst sums + sequencer
  p_engine : process(clk)
    variable v_du, v_dv : signed(19 downto 0);
    variable v_ki   : signed(17 downto 0);
    variable v_int  : signed(16 downto 0);
    variable v_k    : signed(17 downto 0);
    variable v_ph   : signed(18 downto 0);
    variable v_bm   : std_logic;
    variable v_hi   : unsigned(23 downto 0);
    variable v_bu1, v_bv1 : signed(23 downto 0);
  begin
    if rising_edge(clk) then
      s_stb <= '0';

      -- per-line staging for ACCq / gU9 / gV9 (l -> l+1 -> l+2)
      if i_sol = '1' then
        s_accq_prev <= s_accq_res;  s_accq_line <= s_accq_prev;
        s_gu_prev   <= s_gu_res;    s_gu_line   <= s_gu_prev;
        s_gv_prev   <= s_gv_res;    s_gv_line   <= s_gv_prev;
      end if;

      -- burst accumulation (stage 19 -> 20)
      -- (a non-inhibited sample accumulates; an inhibited one adds 0 = holds)
      v_du := resize(i_du, 20); v_dv := resize(i_dv, 20);
      if s_gopen(19) = '1' then
        if s_gin(19) = '1' then
          s_bu <= v_du;
          s_bv <= v_dv;
        else
          s_bu <= (others => '0');
          s_bv <= (others => '0');
        end if;
        if s_gt9(19) = '1' then
          s_sw_tv  <= not s_sw_tv;
          s_sw_pic <= not s_sw_tv;
        end if;
      elsif s_gact(19) = '1' and s_gin(19) = '1' then
        s_bu <= f_sat(resize(s_bu, 22) + resize(v_du, 22), 20);
        s_bv <= f_sat(resize(s_bv, 22) + resize(v_dv, 22), 20);
      end if;
      s_start <= s_glast(19);

      case s_st is
        when ST_IDLE =>
          if s_start = '1' then
            s_st <= ST_INIT;
          end if;

        when ST_INIT =>
          s_pal  <= r_t9;
          s_gain <= '0';
          if s_am_ok = '0' or s_am_tid /= i_tid then      -- am_old
            s_am <= shift_left(resize(r_mref, 37), 13);
          end if;
          s_am_ok  <= '1';
          s_am_tid <= i_tid;
          s_suv <= resize(s_bu, 21) + resize(s_bv, 21);
          s_dvu <= resize(s_bv, 21) - resize(s_bu, 21);
          s_pu  <= (others => '0');
          s_pv  <= (others => '0');
          s_cnt <= (others => '0');
          if r_t9 = '1' then
            s_st <= ST_MUL;
          else
            s_st <= ST_SET0;
          end if;

        when ST_MUL =>   -- 14 steps: rotation (Bu c + Bv s, Bv c - Bu s) or gains
          s_pu  <= shift_right(w_hu & s_pu(13 downto 0), 1);
          s_pv  <= shift_right(w_hv & s_pv(13 downto 0), 1);
          s_cnt <= s_cnt + 1;
          if w_last then
            s_cnt <= (others => '0');
            if s_gain = '1' then
              s_st <= ST_DONE;
            else
              s_st <= ST_SET0;
            end if;
          end if;

        when ST_SET0 =>  -- sources into the copy registers; x, y cleared
          v_bu1 := resize(shift_right(s_pu, 11), 24);   -- bu'
          v_bv1 := resize(shift_right(s_pv, 11), 24);   -- bv'
          if s_pal = '1' then
            s_bur <= v_bu1;
            s_bvr <= v_bv1;
            s_ys  <= v_bu1;
            s_xs  <= v_bv1;
            if v_bu1 > 0 then s_neg <= '1'; else s_neg <= '0'; end if;      -- x0 = -bu' < 0
            if v_bv1 < 0 then s_bvn <= '1'; else s_bvn <= '0'; end if;
          else
            s_ys  <= resize(s_bu, 24);
            s_xs  <= resize(s_bv, 24);
            if s_bu < 0 then s_neg <= '1'; else s_neg <= '0'; end if;
            s_bvn <= '0';
          end if;
          s_cx <= (others => '0');
          s_cy <= (others => '0');
          s_st <= ST_SET1;

        when ST_SET1 =>  -- x = |x0| (pre-rotated), y = pre-rotated y0
          s_cx <= w_xn;
          s_cy <= w_yn;
          if s_neg = '1' then
            s_cz <= to_unsigned(131072, 18);
          else
            s_cz <= (others => '0');
          end if;
          s_it <= (others => '0');
          s_st <= ST_CLD;

        when ST_CLD =>
          s_xs  <= s_cx;
          s_ys  <= s_cy;
          s_shc <= s_it;
          s_st  <= ST_CSH;

        when ST_CSH =>   -- x >> i, y >> i by i one-bit arithmetic shifts
          if s_shc /= 0 then
            s_xs  <= shift_right(s_xs, 1);
            s_ys  <= shift_right(s_ys, 1);
            s_shc <= s_shc - 1;
          else
            s_st <= ST_CUP;
          end if;

        when ST_CUP =>
          s_cx <= w_xn;
          s_cy <= w_yn;
          s_cz <= w_zn;
          if s_it = 11 then
            s_st <= ST_EE;
          else
            s_it <= s_it + 1;
            s_st <= ST_CLD;
          end if;

        when ST_EE =>    -- mag = x, beta = z
          if s_pal = '1' then
            s_ee <= signed(s_cz - to_unsigned(32768, 18));
          else
            s_ee <= signed(s_apc_ph + to_unsigned(131072, 18) - s_cz);
          end if;
          s_st <= ST_APC1;

        when ST_APC1 =>
          -- mag > floor(MREF/20) (MREF/20 < 2048: upper bits or an 11-bit compare)
          if s_cx(23 downto 11) /= 0 or unsigned(s_cx(10 downto 0)) > r_mref20 then
            v_ki  := shift_right(s_ee, 11) + shift_right(s_ee, 13);
            v_int := resize(s_apc_int, 17) + resize(v_ki, 17);
            s_apc_int <= f_satr(v_int, -8448, 8448, 16);
            v_k   := shift_right(s_ee, 5) + shift_right(s_ee, 9);
            v_ph  := signed('0' & s_apc_ph) - resize(v_k, 19);
            s_apc_ph <= unsigned(v_ph(17 downto 0));
          end if;
          -- ident (PAL-S)
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

        when ST_AM1 =>   -- d = mag<<13 - am_old
          s_d  <= signed("00" & unsigned(s_cx(23 downto 0)) & "0000000000000") - signed("00" & s_am);
          s_st <= ST_AM2;

        when ST_AM2 =>   -- t = (d>>10) - (d>>13)
          s_d  <= shift_right(s_d, 10) - shift_right(s_d, 13);
          s_st <= ST_AM3;

        when ST_AM3 =>
          s_am <= unsigned(resize(signed("00" & s_am) + s_d, 37));
          s_st <= ST_KILL;

        when ST_KILL =>
          v_hi := s_am(36 downto 13);
          if s_col_on = '1' then
            if v_hi < r_thr then v_bm := '1'; else v_bm := '0'; end if;
          else
            if v_hi > r_thrhi or (v_hi = r_thrhi and s_am(12 downto 0) /= 0) then
              v_bm := '1';
            else
              v_bm := '0';
            end if;
          end if;
          if v_bm = '1' then
            if s_kcnt = 1000 then
              s_kcnt   <= (others => '0');
              s_col_on <= not s_col_on;
            else
              s_kcnt <= s_kcnt + 1;
            end if;
          else
            s_kcnt <= (others => '0');
          end if;
          s_rem <= shift_left(resize(r_mref, 38), 11);    -- MREF << 11
          s_st  <= ST_DIV1;

        when ST_DIV1 =>  -- saturation test: MREF<<11 >= am
          s_cnt <= (others => '0');
          s_q   <= (others => '0');
          if signed('0' & s_rem) - signed("00" & s_am) >= 0 then
            s_q  <= to_unsigned(C_ACCQ_MAX, 15);
            s_st <= ST_GAIN;
          else
            s_st <= ST_DIV;
          end if;

        when ST_DIV =>   -- 14 restoring steps on rem<<1
          if signed(s_rem & '0') - signed("00" & s_am) >= 0 then
            s_rem <= unsigned(resize(signed(s_rem & '0') - signed("00" & s_am), 38));
            s_q   <= s_q(13 downto 0) & '1';
          else
            s_rem <= s_rem(36 downto 0) & '0';
            s_q   <= s_q(13 downto 0) & '0';
          end if;
          if s_cnt = 13 then
            s_cnt <= (others => '0');
            s_st  <= ST_GAIN;
          else
            s_cnt <= s_cnt + 1;
          end if;

        when ST_GAIN =>  -- multiplier in gain mode: ACCq * KU, ACCq * KV
          s_gain <= '1';
          s_suv  <= signed(resize(s_q, 21));
          s_dvu  <= signed(resize(s_q, 21));
          s_pu   <= (others => '0');
          s_pv   <= (others => '0');
          s_cnt  <= (others => '0');
          s_st   <= ST_MUL;

        when ST_DONE =>
          s_accq_res <= s_q;
          s_gu_res   <= unsigned(resize(shift_right(s_pu, 15), 12));
          s_gv_res   <= unsigned(resize(shift_right(s_pv, 15), 12));
          s_gain     <= '0';
          s_stb      <= '1';
          s_st       <= ST_IDLE;

        when others =>
          s_st <= ST_IDLE;
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
  o_bsum_u  <= s_bu;
  o_bsum_v  <= s_bv;
  o_mag     <= unsigned(s_cx);
  o_beta    <= s_cz;
  o_ee      <= s_ee;
  o_apc_int <= s_apc_int;
  o_am      <= s_am;
  o_accq    <= s_accq_res;
  o_gu      <= s_gu_res;
  o_gv      <= s_gv_res;
  o_kcnt    <= s_kcnt;
  o_sw_tv   <= s_sw_tv;
  o_idc     <= s_idc;
  o_bur     <= resize(s_bur, 28);
  o_bvr     <= resize(s_bvr, 28);
  o_sw_pic_r <= s_sw_pic;

end architecture rtl;
