-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_colour.vhd - CBV Composite colour decode (iii) minimal engine / (iv) fixed (spec rev 4.1 §5.10, §5.11, M14')
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
-- (iii), C_COLOUR = 1:
--  Burst gate (tag n, Y/C domain C_ST_SD): opens at the first sample with
--   n.x + dh_l >= bst + burst_off + P/4 [Q9], lasts blen; no gate on vline lines;
--   a gate sample adds 0 if AVID(n) = 1, n.x < SYNCEND, or |dh_l| >= GI[fmt]
--   (R4.1-1, GI in cbv_tables_pkg, TUNE). Bu, Bv = running-sat sums of du, dv
--   (s20). At the first gate sample, if T9 = PAL-S: sw_tv <- -sw_tv, sw_pic <-
--   the new value [Q12].
--  Serial engine after the gate: mag = max(|Bu|,|Bv|) + (min >> 1); ident (Bv
--   sign vs sw_tv, flip at idc = 3); ACC am = am_old + (d >> 10) - (d >> 13),
--   d = mag 2^13 - am_old (am_old = MREF3 2^13 on the first run / tid change);
--   killer [Q13] (toggle on the 1,001st line); ACCq = MREF3 2^11 >= am ? 16384
--   : floor(MREF3 2^25 / am); gU9 = (ACCq KU) >> 15, gV9 = (ACCq KV) >> 15
--   (bit-serial), then CSD2 -> (e1, e2, sg) codes. col_on / ident apply from
--   l+1, the gain codes from l+2 (§2.2).
--  Picture (/4): U4h = U4 >> 1; V4s = (T9 and sw_pic = -1) ? -V4 : V4 (exact
--   s15); V4h = V4s >> 1; U8 = sat(((U4h 2^e1 + sg U4h 2^e2) >> 10) + 128, 0, 255);
--   col_on = 0 -> 128 / 128. One barrel shifter time-shared over 4 clocks.
-- (iv), C_COLOUR = 0: no gate / engine; col_on = 1; U8 = sat(((U4h 530) >> 10)
--   + 128), V8 = sat(((V4h 375) >> 10) + 128) as shift-adds; PAL-S: sw_pic = -1 on
--   LP_d = 1 lines.
-- Latency: U4 valid (C_ST_U4) -> U8/V8 strobe at C_ST_U4 + 6 = C_ST_UV8.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_variant_pkg.all;
use work.cbv_pkg.all;

entity cbv_colour is
  generic (
    G_CODEC_ONLY : boolean := false
  );
  port (
    clk      : in  std_logic;
    -- Y/C domain (tag n), stage C_ST_SD
    i_x      : in  unsigned(11 downto 0);
    i_sol    : in  std_logic;
    i_vline  : in  std_logic;
    i_avid   : in  std_logic;
    i_lp     : in  std_logic;
    i_tid    : in  t_tid;
    i_t9     : in  std_logic;                -- latched at the Y/C x = 0
    i_dh     : in  signed(11 downto 0);      -- dh_l of tag n
    i_boff   : in  unsigned(5 downto 0);     -- burst_off of the line
    -- demod, stage C_ST_DU
    i_du     : in  signed(14 downto 0);
    i_dv     : in  signed(14 downto 0);
    -- /4 chroma, stage C_ST_U4
    i_u4     : in  signed(14 downto 0);
    i_v4     : in  signed(14 downto 0);
    i_valid  : in  std_logic;
    i_sol4   : in  std_logic;                -- x = 0 at stage C_ST_U4 (picture latches)
    i_t9r    : in  std_logic;                -- registers_in(6)(2) (picture latch)
    -- outputs
    o_col_on : out std_logic;
    o_u8     : out unsigned(7 downto 0);     -- stage C_ST_UV8
    o_v8     : out unsigned(7 downto 0);
    o_valid  : out std_logic;
    -- T5 per-line record (strobe)
    o_t5_stb : out std_logic;
    o_bu     : out signed(19 downto 0);
    o_bv     : out signed(19 downto 0);
    o_mag    : out unsigned(20 downto 0);
    o_am     : out unsigned(36 downto 0);
    o_accq   : out unsigned(14 downto 0);
    o_gu9    : out unsigned(11 downto 0);
    o_gv9    : out unsigned(11 downto 0);
    o_codes  : out std_logic_vector(19 downto 0);  -- e1u,e2u,sgu(2),e1v,e2v,sgv(2) of the line
    o_kcnt   : out unsigned(10 downto 0);
    o_sw_tv  : out std_logic;
    o_sw_pic : out std_logic;
    o_idc    : out unsigned(2 downto 0)
  );
end entity cbv_colour;

architecture rtl of cbv_colour is
  function f_ku return integer is
  begin
    if G_CODEC_ONLY then return 4681; else return 4242; end if;
  end function;
  function f_kv return integer is
  begin
    if G_CODEC_ONLY then return 3318; else return 3007; end if;
  end function;
  constant C_KU  : integer := f_ku;
  constant C_KV  : integer := f_kv;
  constant C_GU0 : integer := (C_ACCQ_RST * C_KU) / 32768;   -- 530 / 585
  constant C_GV0 : integer := (C_ACCQ_RST * C_KV) / 32768;   -- 375 / 414
  function f_code(g : natural) return std_logic_vector is
    variable c : t_csd2;
    variable r : std_logic_vector(9 downto 0);
  begin
    c := f_csd2(to_unsigned(g, 12));
    r(9 downto 6) := std_logic_vector(to_unsigned(c.e1, 4));
    r(5 downto 2) := std_logic_vector(to_unsigned(c.e2, 4));
    if c.sg > 0 then r(1 downto 0) := "01"; elsif c.sg < 0 then r(1 downto 0) := "11"; else r(1 downto 0) := "00"; end if;
    return r;
  end function;
  constant C_CODE0 : std_logic_vector(19 downto 0) := f_code(C_GU0) & f_code(C_GV0);

  signal s_col_on : std_logic := '1';
  signal s_sw_pic : std_logic := '0';
  -- picture
  signal r_codes  : std_logic_vector(19 downto 0) := C_CODE0;
  signal r_t9     : std_logic := '0';
  signal r_col    : std_logic := '1';
  signal s_uh, s_vh : signed(13 downto 0) := (others => '0');
  signal s_ph     : unsigned(2 downto 0) := (others => '1');
  signal s_sh, s_acc : signed(26 downto 0) := (others => '0');
  signal s_u8, s_v8, s_u8a : unsigned(7 downto 0) := to_unsigned(128, 8);
  signal s_ok     : std_logic := '0';
  signal s_vpend  : std_logic := '0';
  signal s_colf   : std_logic := '1';
  signal s_lpp    : std_logic_vector(1 to 30) := (others => '0');
  -- codes from the engine (line l+2)
  signal s_codes_line : std_logic_vector(19 downto 0) := C_CODE0;

  function f_u8(z : signed(26 downto 0)) return unsigned is
    variable v : signed(26 downto 0);
  begin
    v := shift_right(z, 10) + 128;
    if v < 0 then return to_unsigned(0, 8);
    elsif v > 255 then return to_unsigned(255, 8);
    else return unsigned(v(7 downto 0)); end if;
  end function;
begin

  -- ==================================================================== (iii)
  g_iii : if C_COLOUR = 1 generate
    constant C_F0 : t_fmt := f_fmt("0000");
    signal r_bsth  : unsigned(11 downto 0) := to_unsigned(C_F0.bst + C_H2, 12);
    signal r_blen  : unsigned(6 downto 0)  := to_unsigned(C_F0.blen, 7);
    signal r_send  : unsigned(11 downto 0) := to_unsigned(C_F0.syncend, 12);
    signal r_gi    : unsigned(6 downto 0)  := to_unsigned(C_F0.gi, 7);
    signal r_mref  : unsigned(14 downto 0) := to_unsigned(C_F0.mref3, 15);
    signal r_thr   : unsigned(11 downto 0) := to_unsigned(C_F0.thr, 12);
    signal r_thrhi : unsigned(11 downto 0) := to_unsigned(C_F0.thr_hi, 12);
    signal s_armed : std_logic := '0';
    signal s_grem  : unsigned(6 downto 0) := (others => '0');
    type t_gp is array (1 to 6) of std_logic;
    signal s_gopen, s_gact, s_gin, s_glast, s_gt9 : t_gp := (others => '0');
    signal s_bu, s_bv : signed(19 downto 0) := (others => '0');
    signal s_start : std_logic := '0';
    signal s_sw_tv : std_logic := '0';
    signal s_idc   : unsigned(2 downto 0) := (others => '0');
    type t_state is (ST_IDLE, ST_MAG, ST_AM1, ST_AM2, ST_AM3, ST_KILL, ST_DIV1, ST_DIV,
                     ST_GAIN, ST_MUL, ST_CSD, ST_DONE);
    signal s_st    : t_state := ST_IDLE;
    signal s_cnt   : unsigned(3 downto 0) := (others => '0');
    signal s_mag   : unsigned(20 downto 0) := (others => '0');
    signal s_am    : unsigned(36 downto 0) := shift_left(to_unsigned(C_F0.mref3, 37), 13);
    signal s_am_ok : std_logic := '0';
    signal s_am_tid : t_tid := (others => '0');
    signal s_d     : signed(38 downto 0) := (others => '0');
    signal s_kcnt  : unsigned(10 downto 0) := (others => '0');
    signal s_rem   : unsigned(37 downto 0) := (others => '0');
    signal s_q     : unsigned(14 downto 0) := to_unsigned(C_ACCQ_RST, 15);
    signal s_pu, s_pv : unsigned(28 downto 0) := (others => '0');
    signal s_ma    : unsigned(28 downto 0) := (others => '0');
    signal s_kb    : unsigned(12 downto 0) := (others => '0');
    signal s_kbv   : unsigned(12 downto 0) := (others => '0');
    signal s_accq_res : unsigned(14 downto 0) := to_unsigned(C_ACCQ_RST, 15);
    signal s_gu_res, s_gv_res : unsigned(11 downto 0) := to_unsigned(C_GU0, 12);
    signal s_code_res, s_code_prev : std_logic_vector(19 downto 0) := C_CODE0;
    signal s_stb   : std_logic := '0';
  begin
    -- gate (stage SD -> SD+6 = DU)
    p_gate : process(clk)
      variable f      : t_fmt;
      variable v_hb   : signed(13 downto 0);
      variable v_arm  : std_logic;
      variable v_open, v_act, v_last, v_inh : std_logic;
      variable v_dha  : unsigned(11 downto 0);
    begin
      if rising_edge(clk) then
        if i_sol = '1' then
          f := f_fmt(i_tid);
          r_bsth  <= to_unsigned(f.bst + C_H2, 12);
          r_blen  <= to_unsigned(f.blen, 7);
          r_send  <= to_unsigned(f.syncend, 12);
          r_gi    <= to_unsigned(f.gi, 7);
          r_mref  <= to_unsigned(f.mref3, 15);
          r_thr   <= to_unsigned(f.thr, 12);
          r_thrhi <= to_unsigned(f.thr_hi, 12);
        end if;
        v_hb := resize(signed('0' & i_x), 14) + resize(i_dh, 14);
        v_arm := s_armed or i_sol;
        v_open := '0'; v_act := '0'; v_last := '0';
        if i_sol = '1' then s_grem <= (others => '0'); end if;
        if s_grem /= 0 and i_sol = '0' then
          v_act := '1';
          s_grem <= s_grem - 1;
          if s_grem = 1 then v_last := '1'; end if;
          s_armed <= v_arm;
        elsif v_arm = '1' and i_sol = '0' and v_hb >= signed('0' & r_bsth) + signed("00000000" & i_boff) then
          s_armed <= '0';
          if i_vline = '0' then
            v_open := '1'; v_act := '1';
            s_grem <= r_blen - 1;
            if r_blen = 1 then v_last := '1'; end if;
          end if;
        else
          s_armed <= v_arm;
        end if;
        -- inhibits: AVID(n), n.x < SYNCEND, |dh_l| >= GI
        v_dha := unsigned(abs(i_dh));
        if i_avid = '1' or i_x < r_send or v_dha >= r_gi then v_inh := '1'; else v_inh := '0'; end if;
        s_gopen(1) <= v_open;
        s_gact(1)  <= v_act;
        s_gin(1)   <= v_act and not v_inh;
        s_glast(1) <= v_last;
        s_gt9(1)   <= i_t9;
        s_gopen(2 to 6) <= s_gopen(1 to 5);
        s_gact(2 to 6)  <= s_gact(1 to 5);
        s_gin(2 to 6)   <= s_gin(1 to 5);
        s_glast(2 to 6) <= s_glast(1 to 5);
        s_gt9(2 to 6)   <= s_gt9(1 to 5);
      end if;
    end process;

    p_eng : process(clk)
      variable v_au, v_av : unsigned(19 downto 0);
      variable v_hi : unsigned(23 downto 0);
      variable v_bm : std_logic;
      variable v_cu, v_cv : t_csd2;
    begin
      if rising_edge(clk) then
        s_stb <= '0';
        -- staging (l -> l+1 -> l+2) at the Y/C x = 0
        if i_sol = '1' then
          s_code_prev <= s_code_res; s_codes_line <= s_code_prev;
        end if;
        -- burst sums (DU -> DU+1)
        if s_gopen(6) = '1' then
          if s_gin(6) = '1' then s_bu <= resize(i_du, 20); s_bv <= resize(i_dv, 20);
          else s_bu <= (others => '0'); s_bv <= (others => '0'); end if;
          if s_gt9(6) = '1' then
            s_sw_tv  <= not s_sw_tv;
            s_sw_pic <= not s_sw_tv;
          end if;
        elsif s_gact(6) = '1' and s_gin(6) = '1' then
          s_bu <= f_sat(resize(s_bu, 22) + resize(i_du, 22), 20);
          s_bv <= f_sat(resize(s_bv, 22) + resize(i_dv, 22), 20);
        end if;
        s_start <= s_glast(6);

        case s_st is
          when ST_IDLE =>
            if s_start = '1' then s_st <= ST_MAG; end if;

          when ST_MAG =>     -- mag, am_old, ident
            v_au := unsigned(abs(s_bu)); v_av := unsigned(abs(s_bv));
            if v_au >= v_av then s_mag <= resize(v_au, 21) + resize(shift_right(v_av, 1), 21);
            else s_mag <= resize(v_av, 21) + resize(shift_right(v_au, 1), 21); end if;
            if s_am_ok = '0' or s_am_tid /= i_tid then
              s_am <= shift_left(resize(r_mref, 37), 13);
            end if;
            s_am_ok <= '1'; s_am_tid <= i_tid;
            if s_gt9(6) = '1' then     -- ident (PAL-S), this line's T9
              if (s_bv < 0 and s_sw_tv = '0') or (s_bv > 0 and s_sw_tv = '1') then
                if s_idc = 3 then s_idc <= (others => '0'); s_sw_tv <= not s_sw_tv;
                else s_idc <= s_idc + 1; end if;
              elsif s_idc /= 0 then
                s_idc <= s_idc - 1;
              end if;
            end if;
            s_st <= ST_AM1;

          when ST_AM1 =>
            s_d  <= signed("00000" & s_mag & "0000000000000") - signed("00" & s_am);
            s_st <= ST_AM2;
          when ST_AM2 =>
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
              if v_hi > r_thrhi or (v_hi = r_thrhi and s_am(12 downto 0) /= 0) then v_bm := '1'; else v_bm := '0'; end if;
            end if;
            if v_bm = '1' then
              if s_kcnt = 1000 then s_kcnt <= (others => '0'); s_col_on <= not s_col_on;
              else s_kcnt <= s_kcnt + 1; end if;
            else
              s_kcnt <= (others => '0');
            end if;
            s_rem <= shift_left(resize(r_mref, 38), 11);
            s_st  <= ST_DIV1;

          when ST_DIV1 =>
            s_cnt <= (others => '0');
            s_q   <= (others => '0');
            if signed('0' & s_rem) - signed("00" & s_am) >= 0 then
              s_q  <= to_unsigned(C_ACCQ_MAX, 15);
              s_st <= ST_GAIN;
            else
              s_st <= ST_DIV;
            end if;
          when ST_DIV =>
            if signed(s_rem & '0') - signed("00" & s_am) >= 0 then
              s_rem <= unsigned(resize(signed(s_rem & '0') - signed("00" & s_am), 38));
              s_q   <= s_q(13 downto 0) & '1';
            else
              s_rem <= s_rem(36 downto 0) & '0';
              s_q   <= s_q(13 downto 0) & '0';
            end if;
            if s_cnt = 13 then s_cnt <= (others => '0'); s_st <= ST_GAIN;
            else s_cnt <= s_cnt + 1; end if;

          when ST_GAIN =>    -- ACCq KU, ACCq KV: 13-step shift-add (both at once)
            s_ma  <= resize(s_q, 29);
            s_kb  <= to_unsigned(C_KU, 13);
            s_kbv <= to_unsigned(C_KV, 13);
            s_pu  <= (others => '0'); s_pv <= (others => '0');
            s_cnt <= (others => '0');
            s_st  <= ST_MUL;
          when ST_MUL =>
            if s_kb(0) = '1' then s_pu <= s_pu + s_ma; end if;
            if s_kbv(0) = '1' then s_pv <= s_pv + s_ma; end if;
            s_ma  <= shift_left(s_ma, 1);
            s_kb  <= shift_right(s_kb, 1);
            s_kbv <= shift_right(s_kbv, 1);
            if s_cnt = 12 then s_st <= ST_CSD; else s_cnt <= s_cnt + 1; end if;

          when ST_CSD =>
            s_gu_res <= resize(shift_right(s_pu, 15), 12);
            s_gv_res <= resize(shift_right(s_pv, 15), 12);
            s_st <= ST_DONE;
          when ST_DONE =>
            v_cu := f_csd2(s_gu_res);
            v_cv := f_csd2(s_gv_res);
            s_code_res(19 downto 16) <= std_logic_vector(to_unsigned(v_cu.e1, 4));
            s_code_res(15 downto 12) <= std_logic_vector(to_unsigned(v_cu.e2, 4));
            if v_cu.sg > 0 then s_code_res(11 downto 10) <= "01"; elsif v_cu.sg < 0 then s_code_res(11 downto 10) <= "11"; else s_code_res(11 downto 10) <= "00"; end if;
            s_code_res(9 downto 6) <= std_logic_vector(to_unsigned(v_cv.e1, 4));
            s_code_res(5 downto 2) <= std_logic_vector(to_unsigned(v_cv.e2, 4));
            if v_cv.sg > 0 then s_code_res(1 downto 0) <= "01"; elsif v_cv.sg < 0 then s_code_res(1 downto 0) <= "11"; else s_code_res(1 downto 0) <= "00"; end if;
            s_accq_res <= s_q;
            s_stb <= '1';
            s_st  <= ST_IDLE;
        end case;
      end if;
    end process;

    o_t5_stb <= s_stb;
    o_bu     <= s_bu;
    o_bv     <= s_bv;
    o_mag    <= s_mag;
    o_am     <= s_am;
    o_accq   <= s_accq_res;
    o_gu9    <= s_gu_res;
    o_gv9    <= s_gv_res;
    o_codes  <= s_code_res;
    o_kcnt   <= s_kcnt;
    o_sw_tv  <= s_sw_tv;
    o_idc    <= s_idc;
  end generate;

  -- ==================================================================== (iv)
  g_iv : if C_COLOUR = 0 generate
    p_lp : process(clk)
    begin
      if rising_edge(clk) then
        -- PAL-S switch from the decoder line parity (latched at the picture x = 0 below)
        if i_sol = '1' then
          s_sw_pic <= i_lp;
        end if;
      end if;
    end process;
    o_t5_stb <= '0';
    o_bu <= (others => '0'); o_bv <= (others => '0'); o_mag <= (others => '0');
    o_am <= (others => '0'); o_accq <= to_unsigned(C_ACCQ_RST, 15);
    o_gu9 <= to_unsigned(530, 12); o_gv9 <= to_unsigned(375, 12);
    o_codes <= C_CODE0; o_kcnt <= (others => '0'); o_sw_tv <= '0'; o_idc <= (others => '0');
  end generate;

  -- ==================================================================== picture (/4)
  p_pic : process(clk)
    variable v_v   : signed(14 downto 0);
    variable v_e   : natural range 0 to 15;
    variable v_x   : signed(13 downto 0);
    variable v_sub : std_logic;
    variable v_sum : signed(26 downto 0);
    variable v_c8  : unsigned(7 downto 0);
    variable v_codes : std_logic_vector(19 downto 0);
    variable v_t9, v_col : std_logic;
  begin
    if rising_edge(clk) then
      -- per-line latches at the picture stage x = 0
      if i_sol4 = '1' then
        r_codes <= s_codes_line; r_t9 <= i_t9r; r_col <= s_col_on;
        v_codes := s_codes_line; v_t9 := i_t9r; v_col := s_col_on;
      else
        v_codes := r_codes; v_t9 := r_t9; v_col := r_col;
      end if;
      s_ok <= '0';
      if i_valid = '1' then
        s_uh <= resize(shift_right(i_u4, 1), 14);
        if v_t9 = '1' and s_sw_pic = '1' then v_v := -i_v4; else v_v := i_v4; end if;
        s_vh <= resize(shift_right(v_v, 1), 14);
        s_colf <= v_col;
        s_ph <= "000";
      elsif s_ph /= "111" then
        s_ph <= s_ph + 1;
      end if;
      -- phases: 0 U<<e1 | 1 acc <- (U<<e1), U<<e2 | 2 U8, V<<e1 | 3 acc <- (V<<e1), V<<e2
      -- | next clock (any phase): V8, strobe. Fits a 4-clock frame.
      if C_COLOUR = 1 then
        case to_integer(s_ph) is
          when 0 => v_x := s_uh; v_e := to_integer(unsigned(r_codes(19 downto 16)));
          when 1 => v_x := s_uh; v_e := to_integer(unsigned(r_codes(15 downto 12)));
          when 2 => v_x := s_vh; v_e := to_integer(unsigned(r_codes(9 downto 6)));
          when others => v_x := s_vh; v_e := to_integer(unsigned(r_codes(5 downto 2)));
        end case;
        if (s_ph = 1 and r_codes(11 downto 10) = "00") or (s_ph = 3 and r_codes(1 downto 0) = "00") then
          s_sh <= (others => '0');
        elsif s_ph <= 3 then
          s_sh <= shift_left(resize(v_x, 27), v_e);
        end if;
        if s_vpend = '1' then v_sub := r_codes(1); else v_sub := r_codes(11); end if;
        v_sum := f_addsub(s_acc, s_sh, v_sub);
      else
        -- (iv): U 530 = 512 + 16 + 2 ; V 375 = 256 + 64 + 32 + 16 + 8 - 1
        case to_integer(s_ph) is
          when 0 => s_sh <= shift_left(resize(s_uh, 27), 9) + shift_left(resize(s_uh, 27), 4);
          when 1 => s_sh <= shift_left(resize(s_uh, 27), 1);
          when 2 => s_sh <= shift_left(resize(s_vh, 27), 8) + shift_left(resize(s_vh, 27), 6) + shift_left(resize(s_vh, 27), 5);
          when 3 => s_sh <= shift_left(resize(s_vh, 27), 4) + shift_left(resize(s_vh, 27), 3) - resize(s_vh, 27);
          when others => null;
        end case;
        v_sum := s_acc + s_sh;
      end if;
      if s_colf = '0' then v_c8 := to_unsigned(128, 8); else v_c8 := f_u8(v_sum); end if;
      s_vpend <= '0';
      if s_vpend = '1' then                       -- V done (previous frame), strobe
        s_v8 <= v_c8; s_u8 <= s_u8a; s_ok <= '1';
      end if;
      case to_integer(s_ph) is
        when 1 => s_acc <= s_sh;                  -- U first term
        when 2 => s_u8a <= v_c8;                  -- U = first + second term
        when 3 => s_acc <= s_sh; s_vpend <= '1';  -- V first term; V completes next clock
        when others => null;
      end case;
    end if;
  end process;

  o_col_on <= s_col_on;
  o_u8     <= s_u8;
  o_v8     <= s_v8;
  o_valid  <= s_ok;
  o_sw_pic <= s_sw_pic;
end architecture rtl;
