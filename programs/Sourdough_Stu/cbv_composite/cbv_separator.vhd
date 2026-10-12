-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_separator.vhd - CBV Composite virtual-TV sync separator, AFC and Tame (spec rev 4.1 §5.6, M15')
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
-- Stage S0 = C_ST_S (input s), dh_l at S0 + 17 = C_ST_DH. Tag = o's luma tag.
--  Sep Filter  sp1, sp2 = cascaded OP2, k = KSP[P][bw] (cbv_tables_pkg, TUNE),
--              F = 6; P = 4 three-term OP2-3, P = 8 two-term (R4.1-4); bw = (P4 5) >> 10
--              latched per line, carried with each sample through the poles.
--              State s21 (spec s20 + 1 guard: floor drift at |s| = 2^13 could touch
--              -2^19; identical wherever s20 holds).
--  Tip (F 24)  if f < tip >> 24 then tip = f << 24 else OP2(f, k_rel)       S0+6
--              (f = the filtered separator signal sp2 >> 6; ambiguity A1)
--  Slicer      ti = tip >> 24; sl = f < ti + max(57, (bp - ti) >> 1)
--              = f < max(ti + 57, (bp + ti) >> 1)  (exact); edge = sl and not sl_prev;
--              since = edge ? 0 : sat(since + 1, 4095); at since = porch_at:
--              bp += (f - bp) >> 1                                          S0+7
--  AFC (u12.8) 1. h = hphase + 256; 2. on an edge: err = (h < hper >> 1) ? h : h - hper;
--              if |err| < GA 256: ef += ((err - ef) KA) >> 7 (KA 65 Fast / 18 Slow, T10),
--              hper = sat(cpl 256 + ef, HPMIN, HPMAX); 3. if h >= hper: h -= hper.  S0+8
--              Reset and on a timing-ID change: hphase = 0, hper = cpl 256, ef = tip
--              = bp = since = 0.
--  Tame        rh = (x - x_e[P][bw]) mod cpl; dh8 = hphase - 256 rh wrapped into
--              (-128 cpl, 128 cpl]; dhq = min(|dh8| >> 4, 16383); TL = (T RINV16) >> 10;
--              z = min(63, (TL dhq) >> 16); dh_l = sign(dh8) ((lim TANH[z]) >> 8);
--              G4: dh_l = sign(dh8) min(lim, (T dhq) >> 14); T7 Clean or T = 0 -> 0.
-- Per-line values (bw, T, T7, T10) latch at the x = 0 of the stage that uses
-- them (R-LATCH); timing-ID constants latch at the S0 line start.
-- Timing note (A8): the AFC edge update (err -> ef -> hper -> wrap) is one
-- sample-rate recursion of ~8 carry chains; it is reported by the Gate-3 probe.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_variant_pkg.all;
use work.cbv_tables_pkg.all;
use work.cbv_pkg.all;

entity cbv_separator is
  port (
    clk     : in  std_logic;
    i_s     : in  signed(13 downto 0);          -- stage S0
    i_x     : in  unsigned(11 downto 0);        -- tag x at S0
    i_sol   : in  std_logic;
    i_tid   : in  t_tid;                        -- latched at this domain's x = 0
    i_p2    : in  std_logic_vector(9 downto 0); -- registers_in(1) Tame (post-curve)
    i_p4    : in  std_logic_vector(9 downto 0); -- registers_in(3) Sep Filter
    i_t7    : in  std_logic;                    -- registers_in(6)(0): '1' = Clean
    i_t10   : in  std_logic;                    -- registers_in(6)(3): '1' = Slow
    o_dh    : out signed(11 downto 0);          -- dh_l, stage S0 + 17
    -- taps (T1e), each on its own stage
    o_f     : out signed(13 downto 0);          -- sp2 >> 6, S0+5
    o_ti    : out signed(13 downto 0);          -- S0+6
    o_bp    : out signed(13 downto 0);          -- S0+7
    o_sl    : out std_logic;                    -- S0+7
    o_hph   : out unsigned(19 downto 0);        -- S0+8
    o_hper  : out unsigned(19 downto 0);
    o_ef    : out signed(20 downto 0);
    o_dh8   : out signed(20 downto 0);          -- S0+10
    o_z     : out unsigned(5 downto 0)          -- S0+14
  );
end entity cbv_separator;

architecture rtl of cbv_separator is
  constant C_F0 : t_fmt := f_fmt("0000");
  function f_ksp(bw : natural) return t_terms is
  begin
    if C_P8 then return C_KSP8(bw); else return C_KSP4(bw); end if;
  end function;
  function f_xe(bw : natural) return natural is
  begin
    if C_P8 then return C_XE8(bw); else return C_XE4(bw); end if;
  end function;
  function f_bw(p4 : std_logic_vector(9 downto 0)) return unsigned is
  begin
    return resize(shift_right(unsigned(p4) * to_unsigned(5, 3), 10), 3);
  end function;
  function f_kr(ed : boolean) return t_k2 is
  begin
    return C_K_REL(f_kidx(ed));
  end function;

  -- Sep Filter one-pole with runtime step: S <- S - sum s_j (S >> a_j) + pre,
  -- pre = sum s_j ((X << 6) >> a_j). Term 0 always has s_0 = +1.
  function f_term(z : signed; bw : unsigned(2 downto 0); j : natural) return signed is
    variable v : signed(z'length - 1 downto 0) := (others => '0');
    variable t : t_terms;
  begin
    for b in 0 to 4 loop
      if to_integer(bw) = b then
        t := f_ksp(b);
        if t(j).sg /= 0 then v := shift_right(z, t(j).sh); end if;
      end if;
    end loop;
    return v;
  end function;
  function f_tneg(bw : unsigned(2 downto 0); j : natural) return std_logic is
    variable t : t_terms;
    variable r : std_logic := '0';
  begin
    for b in 0 to 4 loop
      if to_integer(bw) = b then
        t := f_ksp(b);
        if t(j).sg < 0 then r := '1'; end if;
      end if;
    end loop;
    return r;
  end function;

  constant C_WS : natural := 21;
  -- per-line latched (S0 domain)
  signal r_cpl8   : unsigned(19 downto 0) := to_unsigned(C_F0.cpl * 256, 20);
  signal r_cpl    : unsigned(11 downto 0) := to_unsigned(C_F0.cpl, 12);
  signal r_ga8    : unsigned(19 downto 0) := to_unsigned(C_F0.ga * 256, 20);
  signal r_pat    : unsigned(11 downto 0) := to_unsigned(C_F0.porch_at, 12);
  signal r_hpmin  : unsigned(19 downto 0) := to_unsigned(C_F0.hpmin, 20);
  signal r_hpmax  : unsigned(19 downto 0) := to_unsigned(C_F0.hpmax, 20);
  signal r_lim    : unsigned(6 downto 0)  := to_unsigned(C_F0.lim, 7);
  signal r_ed     : std_logic := '0';
  signal r_tid    : t_tid := (others => '0');
  signal r_bw     : unsigned(2 downto 0) := (others => '0');
  signal w_bw0    : unsigned(2 downto 0);
  -- per-sample pipes, index = stage offset from S0
  type t_u3 is array (natural range <>) of unsigned(2 downto 0);
  signal s_bwp    : t_u3(1 to 4) := (others => (others => '0'));
  signal s_solp   : std_logic_vector(1 to 17) := (others => '0');
  signal s_rstp   : std_logic_vector(1 to 8) := (others => '0');
  type t_u12 is array (natural range <>) of unsigned(11 downto 0);
  signal s_rhp    : t_u12(2 to 9) := (others => (others => '0'));
  -- Sep Filter
  signal s_pre1, s_st1, s_pre2, s_st2 : signed(C_WS - 1 downto 0) := (others => '0');
  signal w_y1, w_y2 : signed(C_WS - 7 downto 0);
  signal s_f5, s_f6 : signed(13 downto 0) := (others => '0');
  -- tip / slicer / porch
  signal s_tpre   : signed(37 downto 0) := (others => '0');
  signal s_tip    : signed(37 downto 0) := (others => '0');
  signal s_bp     : signed(13 downto 0) := (others => '0');
  signal s_since  : unsigned(11 downto 0) := (others => '0');
  signal s_sl, s_edge : std_logic := '0';
  -- AFC
  signal s_hph    : unsigned(19 downto 0) := (others => '0');
  signal s_hper   : unsigned(19 downto 0) := to_unsigned(C_F0.cpl * 256, 20);
  signal s_ef     : signed(20 downto 0) := (others => '0');
  signal r_ka     : std_logic := '0';
  -- Tame
  signal s_dh8r, s_dh8 : signed(20 downto 0) := (others => '0');
  signal s_dhq    : unsigned(13 downto 0) := (others => '0');
  signal s_sg11, s_sg12, s_sg13, s_sg14, s_sg15, s_sg16 : std_logic := '0';
  signal s_tln    : unsigned(9 downto 0) := (others => '0');
  signal s_tlc    : unsigned(10 downto 0) := (others => '0');
  signal s_tc     : unsigned(9 downto 0) := (others => '0');
  signal s_tll    : unsigned(10 downto 0) := (others => '0');
  signal s_tpl    : unsigned(9 downto 0) := (others => '0');
  signal s_ma     : unsigned(10 downto 0) := (others => '0');
  signal s_mt     : unsigned(9 downto 0) := (others => '0');
  signal s_mb     : unsigned(13 downto 0) := (others => '0');
  signal s_pr     : unsigned(24 downto 0) := (others => '0');
  signal s_z      : unsigned(5 downto 0) := (others => '0');
  signal s_hz     : unsigned(6 downto 0) := (others => '0');
  signal s_tz     : unsigned(8 downto 0) := (others => '0');
  signal s_m16    : unsigned(15 downto 0) := (others => '0');
  signal s_lim16  : unsigned(6 downto 0) := (others => '0');
  signal s_clean  : std_logic := '1';
  signal s_dh     : signed(11 downto 0) := (others => '0');
begin

  -- ================================================================ per-line latches (S0) + pipes
  p_line : process(clk)
    variable f    : t_fmt;
    variable v_bw : unsigned(2 downto 0);
    variable v_rh : integer;
  begin
    if rising_edge(clk) then
      s_solp(1) <= i_sol; s_solp(2 to 17) <= s_solp(1 to 16);
      s_rstp(1) <= '0';
      if i_sol = '1' then
        f := f_fmt(i_tid);
        r_cpl8  <= to_unsigned(f.cpl * 256, 20);
        r_cpl   <= to_unsigned(f.cpl, 12);
        r_ga8   <= to_unsigned(f.ga * 256, 20);
        r_pat   <= to_unsigned(f.porch_at, 12);
        r_hpmin <= to_unsigned(f.hpmin, 20);
        r_hpmax <= to_unsigned(f.hpmax, 20);
        r_lim   <= to_unsigned(f.lim, 7);
        if f.ed then r_ed <= '1'; else r_ed <= '0'; end if;
        r_tid   <= i_tid;
        if i_tid /= r_tid then s_rstp(1) <= '1'; end if;
        v_bw := f_bw(i_p4);
        r_bw <= v_bw;
      else
        v_bw := r_bw;
      end if;
      s_rstp(2 to 8) <= s_rstp(1 to 7);
      s_bwp(1) <= v_bw; s_bwp(2 to 4) <= s_bwp(1 to 3);
      -- rh = (x - x_e) mod cpl, computed here and carried to S0+8
      v_rh := to_integer(i_x) - f_xe(to_integer(v_bw));
      if v_rh < 0 then v_rh := v_rh + to_integer(r_cpl); end if;
      if v_rh < 0 then v_rh := 0; end if;
      s_rhp(2) <= to_unsigned(v_rh mod 4096, 12);
      s_rhp(3 to 9) <= s_rhp(2 to 8);
    end if;
  end process;

  -- ================================================================ Sep Filter (S0+1 .. S0+4)
  w_bw0 <= f_bw(i_p4) when i_sol = '1' else r_bw;   -- step of the sample at S0
  w_y1 <= s_st1(C_WS - 1 downto 6);
  w_y2 <= s_st2(C_WS - 1 downto 6);
  p_sep : process(clk)
    variable v_xf : signed(C_WS - 1 downto 0);
    variable v_p, v_q : signed(C_WS - 1 downto 0);
    variable v_t1, v_t2 : signed(C_WS - 1 downto 0);
  begin
    if rising_edge(clk) then
      -- pole 1: pre at S0+1 (bw of the sample), S at S0+2
      v_xf := shift_left(resize(i_s, C_WS), 6);
      v_p  := f_term(v_xf, w_bw0, 0);
      v_p  := f_addsub(v_p, f_term(v_xf, w_bw0, 1), f_tneg(w_bw0, 1));
      v_p  := f_addsub(v_p, f_term(v_xf, w_bw0, 2), f_tneg(w_bw0, 2));
      s_pre1 <= v_p;
      -- S update for the sample at S0+1 (its step: s_bwp(1))
      v_t1 := s_st1 - f_term(s_st1, s_bwp(1), 0);
      v_t2 := f_addsub(s_pre1, f_term(s_st1, s_bwp(1), 1), not f_tneg(s_bwp(1), 1));
      v_t2 := f_addsub(v_t2, f_term(s_st1, s_bwp(1), 2), not f_tneg(s_bwp(1), 2));
      s_st1 <= v_t1 + v_t2;
      -- pole 2: pre at S0+3, S at S0+4
      v_xf := shift_left(resize(w_y1, C_WS), 6);
      v_q  := f_term(v_xf, s_bwp(2), 0);
      v_q  := f_addsub(v_q, f_term(v_xf, s_bwp(2), 1), f_tneg(s_bwp(2), 1));
      v_q  := f_addsub(v_q, f_term(v_xf, s_bwp(2), 2), f_tneg(s_bwp(2), 2));
      s_pre2 <= v_q;
      v_t1 := s_st2 - f_term(s_st2, s_bwp(3), 0);
      v_t2 := f_addsub(s_pre2, f_term(s_st2, s_bwp(3), 1), not f_tneg(s_bwp(3), 1));
      v_t2 := f_addsub(v_t2, f_term(s_st2, s_bwp(3), 2), not f_tneg(s_bwp(3), 2));
      s_st2 <= v_t1 + v_t2;
      -- S0+5: filtered signal f
      s_f5 <= resize(w_y2, 14);
    end if;
  end process;

  -- ================================================================ tip, slicer, porch, AFC
  p_sync : process(clk)
    variable v_k    : t_k2;
    variable v_ti   : signed(13 downto 0);
    variable v_a, v_b, v_th : signed(15 downto 0);
    variable v_sl   : std_logic;
    variable v_edge : std_logic;
    variable v_since : unsigned(11 downto 0);
    variable v_t    : signed(37 downto 0);
    variable v_h    : unsigned(19 downto 0);
    variable v_err  : signed(21 downto 0);
    variable v_d    : signed(22 downto 0);
    variable v_m    : signed(29 downto 0);
    variable v_ef   : signed(20 downto 0);
    variable v_hp   : signed(21 downto 0);
    variable v_hper : unsigned(19 downto 0);
    variable v_ka   : std_logic;
  begin
    if rising_edge(clk) then
      if r_ed = '1' then v_k := f_kr(true); else v_k := f_kr(false); end if;
      -- S0+5: tip pre from f (= w_y2 of the sample at S0+4)
      v_t := shift_left(resize(w_y2, 38), 24);
      if v_k.sg > 0 then
        s_tpre <= shift_right(v_t, v_k.a) + shift_right(v_t, v_k.b);
      elsif v_k.sg < 0 then
        s_tpre <= shift_right(v_t, v_k.a) - shift_right(v_t, v_k.b);
      else
        s_tpre <= shift_right(v_t, v_k.a);
      end if;
      -- S0+6: tip
      if s_rstp(5) = '1' then
        s_tip <= (others => '0');
      elsif s_f5 < s_tip(37 downto 24) then
        s_tip <= shift_left(resize(s_f5, 38), 24);
      else
        if v_k.sg > 0 then
          s_tip <= (s_tip - shift_right(s_tip, v_k.a)) - (shift_right(s_tip, v_k.b) - s_tpre);
        elsif v_k.sg < 0 then
          s_tip <= (s_tip - shift_right(s_tip, v_k.a)) + (shift_right(s_tip, v_k.b) + s_tpre);
        else
          s_tip <= (s_tip - shift_right(s_tip, v_k.a)) + s_tpre;
        end if;
      end if;
      s_f6 <= s_f5;
      -- S0+7: slicer, since, porch level
      v_ti := s_tip(37 downto 24);
      v_a  := resize(v_ti, 16) + 57;
      v_b  := shift_right(resize(s_bp, 16) + resize(v_ti, 16), 1);
      if v_a >= v_b then v_th := v_a; else v_th := v_b; end if;
      if resize(s_f6, 16) < v_th then v_sl := '1'; else v_sl := '0'; end if;
      v_edge := v_sl and not s_sl;
      if s_rstp(6) = '1' then
        s_sl <= '0'; s_edge <= '0'; s_since <= (others => '0'); s_bp <= (others => '0');
      else
        s_sl   <= v_sl;
        s_edge <= v_edge;
        if v_edge = '1' then
          v_since := (others => '0');
        elsif s_since /= x"FFF" then
          v_since := s_since + 1;
        else
          v_since := s_since;
        end if;
        s_since <= v_since;
        if v_since = r_pat then
          s_bp <= s_bp + resize(shift_right(resize(s_f6, 15) - resize(s_bp, 15), 1), 14);
        end if;
      end if;
      -- S0+8: AFC (sample with edge flag s_edge)
      if s_solp(7) = '1' then r_ka <= i_t10; v_ka := i_t10; else v_ka := r_ka; end if;
      if s_rstp(7) = '1' then
        s_hph <= (others => '0'); s_hper <= r_cpl8; s_ef <= (others => '0');
      else
        v_h := s_hph + 256;
        v_hper := s_hper;
        v_ef := s_ef;
        if s_edge = '1' then
          if v_h < shift_right(s_hper, 1) then
            v_err := signed(resize(v_h, 22));
          else
            v_err := signed(resize(v_h, 22)) - signed(resize(s_hper, 22));
          end if;
          if abs(v_err) < signed(resize(r_ga8, 22)) then
            v_d := resize(v_err, 23) - resize(s_ef, 23);
            if v_ka = '1' then    -- Slow: 18 = 16 + 2
              v_m := shift_left(resize(v_d, 30), 4) + shift_left(resize(v_d, 30), 1);
            else                  -- Fast: 65 = 64 + 1
              v_m := shift_left(resize(v_d, 30), 6) + resize(v_d, 30);
            end if;
            v_ef := s_ef + resize(shift_right(v_m, 7), 21);
            v_hp := signed(resize(r_cpl8, 22)) + resize(v_ef, 22);
            if v_hp < signed(resize(r_hpmin, 22)) then v_hper := r_hpmin;
            elsif v_hp > signed(resize(r_hpmax, 22)) then v_hper := r_hpmax;
            else v_hper := unsigned(v_hp(19 downto 0)); end if;
          end if;
        end if;
        if v_h >= v_hper then v_h := v_h - v_hper; end if;
        s_hph  <= v_h;
        s_hper <= v_hper;
        s_ef   <= v_ef;
      end if;
    end if;
  end process;

  -- ================================================================ Tame (S0+9 .. S0+17)
  -- TL candidate from registers_in continuously, latched at the MUL stage's x = 0
  p_tl : process(clk)
    variable v_t : unsigned(9 downto 0);
    variable v_p : unsigned(21 downto 0);
  begin
    if rising_edge(clk) then
      v_t := unsigned(i_p2);
      s_tc <= v_t;
      -- (T RINV16) >> 10 as shift-adds; RINV16 2048 / 1638 (SD), 819 / 1170 (HD)
      if not C_HD then
        if r_lim = 32 then
          v_p := shift_left(resize(s_tc, 22), 11);
        else  -- 1638 = 2^11 - 2^8 - 2^7 - 2^4 - 2^3 - 2^1
          v_p := shift_left(resize(s_tc, 22), 11) - shift_left(resize(s_tc, 22), 8)
               - shift_left(resize(s_tc, 22), 7) - shift_left(resize(s_tc, 22), 4)
               - shift_left(resize(s_tc, 22), 3) - shift_left(resize(s_tc, 22), 1);
        end if;
      else
        if r_lim = 80 then  -- 819 = 2^9 + 2^8 + 2^5 + 2^4 + 2^1 + 2^0
          v_p := shift_left(resize(s_tc, 22), 9) + shift_left(resize(s_tc, 22), 8)
               + shift_left(resize(s_tc, 22), 5) + shift_left(resize(s_tc, 22), 4)
               + shift_left(resize(s_tc, 22), 1) + resize(s_tc, 22);
        else                -- 1170 = 2^10 + 2^7 + 2^4 + 2^1
          v_p := shift_left(resize(s_tc, 22), 10) + shift_left(resize(s_tc, 22), 7)
               + shift_left(resize(s_tc, 22), 4) + shift_left(resize(s_tc, 22), 1);
        end if;
      end if;
      s_tlc <= resize(shift_right(v_p, 10), 11);
      s_tpl <= s_tc;
    end if;
  end process;

  p_tame : process(clk)
    variable v_d  : signed(20 downto 0);
    variable v_c8 : signed(20 downto 0);
    variable v_a  : unsigned(20 downto 0);
    variable v_z  : unsigned(24 downto 0);
    variable v_m  : unsigned(15 downto 0);
    variable v_hd : unsigned(23 downto 0);
    variable v_l  : unsigned(15 downto 0);
  begin
    if rising_edge(clk) then
      -- S0+9: dh8 raw = hphase - 256 rh
      s_dh8r <= signed(resize(s_hph, 21)) - signed(resize(s_rhp(9) & x"00", 21));
      -- S0+10: wrap into (-128 cpl, 128 cpl]
      v_c8 := signed(resize(r_cpl8, 21));
      v_d  := s_dh8r;
      if v_d > shift_right(v_c8, 1) then
        v_d := v_d - v_c8;
      elsif v_d <= -shift_right(v_c8, 1) then
        v_d := v_d + v_c8;
      end if;
      s_dh8 <= v_d;
      -- S0+11: dhq, sign
      v_a := unsigned(abs(s_dh8));
      if shift_right(v_a, 4) > 16383 then s_dhq <= to_unsigned(16383, 14);
      else s_dhq <= resize(shift_right(v_a, 4), 14); end if;
      if s_dh8 < 0 then s_sg11 <= '1'; else s_sg11 <= '0'; end if;
      -- S0+12: MUL input registers (TL, T latched at this stage's x = 0)
      if s_solp(11) = '1' then
        s_tll <= s_tlc; s_tln <= s_tpl;
        s_ma  <= s_tlc; s_mt  <= s_tpl;
      else
        s_ma  <= s_tll; s_mt  <= s_tln;
      end if;
      s_mb <= s_dhq;
      s_sg12 <= s_sg11;
      -- S0+13: product
      if C_TAME_HARD then
        s_pr <= resize(s_mt * s_mb, 25);
      else
        s_pr <= s_ma * s_mb;
      end if;
      s_sg13 <= s_sg12;
      -- S0+14: z = min(63, (TL dhq) >> 16)  (G4: h = min(lim, (T dhq) >> 14))
      v_z := shift_right(s_pr, 16);
      if v_z > 63 then s_z <= to_unsigned(63, 6); else s_z <= v_z(5 downto 0); end if;
      v_hd := resize(shift_right(s_pr, 14), 24);
      if v_hd > r_lim then s_hz <= r_lim; else s_hz <= v_hd(6 downto 0); end if;
      s_sg14 <= s_sg13;
      -- S0+15: TANH[z]
      s_tz <= to_unsigned(C_TANH(to_integer(s_z)), 9);
      s_sg15 <= s_sg14;
      s_lim16 <= s_hz;
      -- S0+16: lim TANH[z] (shift-adds: 32 / 40 / 80 / 56)
      case to_integer(r_lim) is
        when 32 => v_m := shift_left(resize(s_tz, 16), 5);
        when 40 => v_m := shift_left(resize(s_tz, 16), 5) + shift_left(resize(s_tz, 16), 3);
        when 80 => v_m := shift_left(resize(s_tz, 16), 6) + shift_left(resize(s_tz, 16), 4);
        when others => v_m := shift_left(resize(s_tz, 16), 6) - shift_left(resize(s_tz, 16), 3);  -- 56
      end case;
      s_m16 <= v_m;
      s_sg16 <= s_sg15;
      -- S0+17: dh_l (T7 Clean or T = 0 -> 0); T7 / T latched at this stage's x = 0
      if s_solp(16) = '1' then
        if i_t7 = '1' or unsigned(i_p2) = 0 then s_clean <= '1'; else s_clean <= '0'; end if;
      end if;
      if C_TAME_HARD then
        v_l := resize(s_lim16, 16);
      else
        v_l := shift_right(s_m16, 8);
      end if;
      if (s_solp(16) = '1' and (i_t7 = '1' or unsigned(i_p2) = 0)) or (s_solp(16) = '0' and s_clean = '1') then
        s_dh <= (others => '0');
      elsif s_sg16 = '1' then
        s_dh <= -signed(resize(v_l, 12));
      else
        s_dh <= signed(resize(v_l, 12));
      end if;
    end if;
  end process;

  o_dh   <= s_dh;
  o_f    <= s_f5;
  o_ti   <= s_tip(37 downto 24);
  o_bp   <= s_bp;
  o_sl   <= s_sl;
  o_hph  <= s_hph;
  o_hper <= s_hper;
  o_ef   <= s_ef;
  o_dh8  <= s_dh8;
  o_z    <= s_z;
end architecture rtl;
