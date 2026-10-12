-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_demod.vhd - CBV Composite chroma demodulator, /4 integrate-and-dump, /4 poles (spec rev 4.1 §5.9, M13')
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
--   md  = ((8/P) x_n + psi_n - (comb_en ? 2 : 0) + dm_l) mod 8      [Q4]
--   du  = S8[md] applied to 2 cS, dv with C8: 0, +-2cS, +-H(2cS)      [Q8]
--         s15 with a guard bit: +8192 when cS = -4096 negated (R4.1-10)
--   Acc adds du every sample and is never reset at the line wrap. A strobe is a
--   sample with (x_n - a0_m) mod 4 = r_P (2 / 1): I4 = (Acc + du) >> 2, Acc <- 0.
--   Drop rule (R4.1-11): a strobe less than 4 clocks after the previous one
--   discards the previous I4 (it never enters the poles). Implemented as a
--   pending register committed 3 clocks after its strobe if no newer strobe came.
--   Poles: two cascaded OP2 per channel at the /4 rate, k = k4c (5/8 | 7/8),
--   F = 6, states s21, one adder set time-shared U1, V1, U2, V2 (4 clocks).
--   U4 = U2 >> 6, V4 = V2 >> 6 (s15), valid with o_valid.
-- dm_l latches at this block's x = 0 (R-LATCH).
-- Pipeline: YC (cS) | +1 2cS, partial H | +2 H | +3 du, dv (C_ST_DU) |
--   +4 I4, strobe (C_ST_I4) | +7 commit | +8..+11 U1, V1, U2, V2 | U4 valid at I4 + 8.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_variant_pkg.all;
use work.cbv_pkg.all;

entity cbv_demod is
  port (
    clk       : in  std_logic;
    -- Y/C domain (tag n), stage C_ST_SD
    i_x       : in  unsigned(11 downto 0);
    i_a0m     : in  unsigned(11 downto 0);
    i_psi     : in  unsigned(2 downto 0);
    i_sol     : in  std_logic;
    -- stage C_ST_YC
    i_c       : in  signed(12 downto 0);    -- cS
    i_comb_en : in  std_logic;
    i_dm      : in  unsigned(2 downto 0);   -- dm_l (slider detune), per line
    -- outputs
    o_md      : out unsigned(2 downto 0);   -- T4, stage C_ST_DU
    o_du      : out signed(14 downto 0);    -- stage C_ST_DU
    o_dv      : out signed(14 downto 0);
    o_strobe  : out std_logic;              -- stage C_ST_I4
    o_i4u     : out signed(14 downto 0);
    o_i4v     : out signed(14 downto 0);
    o_commit  : out std_logic;              -- pole input taken (T4), I4 + 3
    o_pin_u   : out signed(14 downto 0);
    o_pin_v   : out signed(14 downto 0);
    o_u4      : out signed(14 downto 0);    -- stage C_ST_U4
    o_v4      : out signed(14 downto 0);
    o_valid   : out std_logic
  );
end entity cbv_demod;

architecture rtl of cbv_demod is
  type t_m_pipe is array (14 to 16) of unsigned(2 downto 0);
  type t_r_pipe is array (14 to 19) of unsigned(1 downto 0);
  signal s_mp  : t_m_pipe := (others => (others => '0'));
  signal s_rp  : t_r_pipe := (others => (others => '0'));
  signal s_solp : std_logic_vector(14 to 16) := (others => '0');
  signal s_dm  : unsigned(2 downto 0) := (others => '0');

  signal s_ha17, s_hb17, s_c2_17 : signed(14 downto 0) := (others => '0');
  signal s_md17, s_md18, s_md19 : unsigned(2 downto 0) := (others => '0');
  signal s_h18, s_c2_18 : signed(14 downto 0) := (others => '0');
  signal s_du19, s_dv19 : signed(14 downto 0) := (others => '0');
  -- integrate and dump
  signal s_accu, s_accv : signed(17 downto 0) := (others => '0');
  signal s_i4u, s_i4v   : signed(14 downto 0) := (others => '0');
  signal s_stb          : std_logic := '0';
  -- pending / commit
  signal s_pu, s_pv     : signed(14 downto 0) := (others => '0');
  signal s_age          : unsigned(2 downto 0) := (others => '1');
  signal s_cmt          : std_logic := '0';
  signal s_xu, s_xv     : signed(14 downto 0) := (others => '0');
  -- poles (time-shared)
  signal s_ph           : unsigned(2 downto 0) := (others => '1');
  signal s_u1, s_v1, s_u2, s_v2 : signed(20 downto 0) := (others => '0');
  signal s_ok           : std_logic := '0';
  signal w_s, w_x, w_n  : signed(20 downto 0);

  -- one /4 pole step: S - (S>>a) - sg (S>>b) + ((X<<6)>>a) + sg ((X<<6)>>b)
  function f_step(s : signed(20 downto 0); x : signed(14 downto 0)) return signed is
    variable v_xf, v_p, v_t1, v_t2 : signed(20 downto 0);
    variable v_sub : std_logic;
  begin
    v_xf := shift_left(resize(x, 21), 6);
    if C_K4C.sg < 0 then v_sub := '1'; else v_sub := '0'; end if;
    v_p  := f_addsub(shift_right(v_xf, C_K4C.a), shift_right(v_xf, C_K4C.b), v_sub);
    v_t1 := s - shift_right(s, C_K4C.a);
    v_t2 := f_addsub(v_p, shift_right(s, C_K4C.b), not v_sub);
    return v_t1 + v_t2;
  end function;
begin

  p_demod : process(clk)
    variable v_m  : unsigned(2 downto 0);
    variable v_md : unsigned(2 downto 0);
    variable v_cv, v_cu : signed(14 downto 0);
    variable v_nu, v_nv : std_logic;
    variable v_su, v_sv : signed(17 downto 0);
  begin
    if rising_edge(clk) then
      -- carrier index and /4 residue of the newest sample (stage SD -> SD+1..)
      if C_P8 then
        v_m := i_x(2 downto 0) + i_psi;
      else
        v_m := (i_x(1 downto 0) & '0') + i_psi;
      end if;
      s_mp(14) <= v_m;
      s_mp(15 to 16) <= s_mp(14 to 15);
      s_rp(14) <= i_x(1 downto 0) - i_a0m(1 downto 0);
      s_rp(15 to 19) <= s_rp(14 to 18);
      s_solp(14) <= i_sol; s_solp(15 to 16) <= s_solp(14 to 15);

      -- YC+1: md (dm_l latched at this stage's x = 0), 2cS, partial H(2cS)
      if s_solp(16) = '1' then s_dm <= i_dm; v_md := s_mp(16) + i_dm; else v_md := s_mp(16) + s_dm; end if;
      if i_comb_en = '1' then v_md := v_md - 2; end if;
      s_md17  <= v_md;
      s_c2_17 <= shift_left(resize(i_c, 15), 1);
      s_ha17  <= resize(i_c, 15) + resize(shift_right(i_c, 2), 15);
      s_hb17  <= resize(shift_right(i_c, 3), 15) + resize(shift_right(i_c, 5), 15);
      -- YC+2
      s_h18   <= s_ha17 + s_hb17;
      s_c2_18 <= s_c2_17;
      s_md18  <= s_md17;
      -- YC+3: du, dv (sign-magnitude select, one incrementer each)
      case to_integer(s_md18) is
        when 0 | 4  => v_cu := (others => '0');
        when 2 | 6  => v_cu := s_c2_18;
        when others => v_cu := s_h18;
      end case;
      if s_md18 >= 5 then v_nu := '1'; else v_nu := '0'; end if;
      s_du19 <= f_negif(v_cu, v_nu);
      case to_integer(s_md18) is
        when 2 | 6  => v_cv := (others => '0');
        when 0 | 4  => v_cv := s_c2_18;
        when others => v_cv := s_h18;
      end case;
      if s_md18 >= 3 and s_md18 <= 5 then v_nv := '1'; else v_nv := '0'; end if;
      s_dv19 <= f_negif(v_cv, v_nv);
      s_md19 <= s_md18;

      -- DU+1: integrate and dump (strobe on (x_n - a0_m) mod 4 = r_P)
      v_su := s_accu + resize(s_du19, 18);
      v_sv := s_accv + resize(s_dv19, 18);
      s_stb <= '0';
      if s_rp(19) = to_unsigned(C_R4V, 2) then
        s_i4u <= resize(shift_right(v_su, 2), 15);
        s_i4v <= resize(shift_right(v_sv, 2), 15);
        s_accu <= (others => '0');
        s_accv <= (others => '0');
        s_stb <= '1';
      else
        s_accu <= v_su;
        s_accv <= v_sv;
      end if;

      -- pending I4 and commit (drop rule)
      s_cmt <= '0';
      if s_stb = '1' then
        s_pu <= s_i4u; s_pv <= s_i4v;
        s_age <= (others => '0');
      elsif s_age < 3 then
        s_age <= s_age + 1;
        if s_age = 2 then              -- 3 clocks without a newer strobe
          s_cmt <= '1';
          s_xu <= s_pu; s_xv <= s_pv;
        end if;
      end if;

      -- poles, time-shared: U1, V1, U2, V2
      s_ok <= '0';
      if s_cmt = '1' then
        s_ph <= "001";
        s_u1 <= w_n;
      else
        case to_integer(s_ph) is
          when 1 => s_v1 <= w_n; s_ph <= "010";
          when 2 => s_u2 <= w_n; s_ph <= "011";
          when 3 => s_v2 <= w_n; s_ph <= "111"; s_ok <= '1';
          when others => null;
        end case;
      end if;
    end if;
  end process;

  -- shared adder set operands
  w_s <= s_u1 when s_cmt = '1' else
         s_v1 when s_ph = 1 else
         s_u2 when s_ph = 2 else
         s_v2;
  w_x <= resize(s_xu, 21) when s_cmt = '1' else
         resize(s_xv, 21) when s_ph = 1 else
         resize(s_u1(20 downto 6), 21) when s_ph = 2 else
         resize(s_v1(20 downto 6), 21);
  w_n <= f_step(w_s, w_x(14 downto 0));

  o_md     <= s_md19;
  o_du     <= s_du19;
  o_dv     <= s_dv19;
  o_strobe <= s_stb;
  o_i4u    <= s_i4u;
  o_i4v    <= s_i4v;
  o_commit <= s_cmt;
  o_pin_u  <= s_xu;
  o_pin_v  <= s_xv;
  o_u4     <= s_u2(20 downto 6);
  o_v4     <= s_v2(20 downto 6);
  o_valid  <= s_ok;
end architecture rtl;
