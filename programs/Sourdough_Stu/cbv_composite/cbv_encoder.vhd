-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_encoder.vhd - CBV Composite virtual composite encoder (block 2, spec §5.2)
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
-- Y,U,V -> comp (cv, s14; white 0.714 V = 1024).
--
-- Two sample streams meet at the modulator (PROVISIONAL Q1):
--   chroma : the input sample stream (stage 1) through Us/Vs + 2 OP2 poles
--            -> Uf, Vf at stage 6 ("5 (poles)").
--   luma   : the luma-tag stream = input delayed YE more (cbv_composite_core
--            builds it). Yd, x, psi, LP, vline, hsync/vsync -> SYNC, burst, m
--            all come from the luma-tag sample, so comp is placed on the true
--            raster and the chroma filter's group delay (~YE) is compensated.
--   gateE  : input AVID delayed with Uf (chroma sample).
--
-- Pipeline (clock = stage number, cbv_pkg C_ST_*):
--   2  Uc/Vc (0 outside input AVID), Us = Uc - Uc>>3, Vs = Vc + Vc>>2 - Vc>>6
--   3  pole 1 pre    = (X<<8)>>a - (X<<8)>>b        (kc = 2^-a - 2^-b, F = 8)
--   4  pole 1 S      = S - S>>a + S>>b + pre        (cleared on the x = 0 sample)
--   5  pole 2 pre    (X = S1 >> 8)
--   6  pole 2 S      -> Uf = S2 >> 8 ; encoder domain registers (luma tag)
--   7  H*Uf, H*Vf ; m, sw, burst window, SYNC level from the domain
--   8  S8[m]*Uf, sw*C8[m]*Vf selected (negation after the H shift-sum, Q8)
--   9  C = burst ? BURST[m] : (gateE ? tU + tV : 0)
--   10 comp = sat14(Yd + C + SYNC)                    -> o_comp
--   kc: P=4 7/16 (a=1, b=4); P=8 63/256 (a=2, b=8); F = 8 as tabled (Q17).
-- Latency: chroma stage 1 -> comp 9 clocks (luma-tag stage 1 -> comp 9).
-- Resources (paper): (a) ~320 LC, (b) ~160 LC (luma tag chain is in the core).
-- No EBR, no multipliers.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_pkg.all;

entity cbv_encoder is
  port (
    clk         : in  std_logic;
    -- chroma stream, stage 1
    i_u         : in  unsigned(9 downto 0);
    i_v         : in  unsigned(9 downto 0);
    i_avid      : in  std_logic;
    i_sol       : in  std_logic;   -- input-domain x = 0 (pole clear)
    i_p8        : in  boolean;     -- input-domain P = 8
    -- luma-tag stream, stage 5 (registered here into stage 6)
    i_y_lt      : in  unsigned(9 downto 0);
    i_hs_lt     : in  std_logic;
    i_vs_lt     : in  std_logic;
    i_avid_lt   : in  std_logic;
    i_tid       : in  t_tid;
    i_reg6      : in  std_logic_vector(4 downto 0);
    i_burst_en  : in  std_logic;   -- G_TEST_BURST_GAIN (1 = normal)
    -- outputs, stage 10
    o_comp      : out signed(13 downto 0);
    o_m         : out unsigned(2 downto 0);
    o_psi       : out unsigned(2 downto 0)
  );
end entity cbv_encoder;

architecture rtl of cbv_encoder is
  -- chroma
  signal s_us2, s_vs2   : signed(11 downto 0) := (others => '0');
  signal s_pu3, s_pv3   : signed(21 downto 0) := (others => '0');
  signal s_s1u, s_s1v   : signed(21 downto 0) := (others => '0');
  signal s_pu5, s_pv5   : signed(21 downto 0) := (others => '0');
  signal s_s2u, s_s2v   : signed(21 downto 0) := (others => '0');
  signal s_sol2, s_sol3, s_sol4, s_sol5 : std_logic := '0';
  signal s_p8_2, s_p8_3, s_p8_4, s_p8_5 : boolean := false;
  signal s_gate         : std_logic_vector(2 to 9) := (others => '0');
  signal s_uf6, s_vf6   : signed(13 downto 0);
  -- encoder domain (stage 6)
  signal d_hs, d_vs, d_avid, d_sol, d_lp, d_vline : std_logic;
  signal d_x            : unsigned(11 downto 0);
  signal d_psi          : unsigned(2 downto 0);
  signal d_tid          : t_tid;
  signal d_reg6         : std_logic_vector(4 downto 0);
  signal d_aline        : unsigned(1 downto 0);
  signal s_y6           : unsigned(9 downto 0) := (others => '0');
  -- modulator
  signal s_hu7, s_hv7, s_uf7, s_vf7 : signed(13 downto 0) := (others => '0');
  signal s_m7, s_m8, s_m9, s_m10    : unsigned(2 downto 0) := (others => '0');
  signal s_psi7, s_psi8, s_psi9, s_psi10 : unsigned(2 downto 0) := (others => '0');
  signal s_sw7          : std_logic := '0';
  signal s_t8_7         : std_logic := '0';
  signal s_bf7, s_bf8   : std_logic := '0';
  signal s_sync7, s_sync8, s_sync9 : signed(10 downto 0) := (others => '0');
  signal s_yd7, s_yd8, s_yd9 : unsigned(9 downto 0) := (others => '0');
  signal s_tu8, s_tv8   : signed(13 downto 0) := (others => '0');
  signal s_bv8          : signed(8 downto 0) := (others => '0');
  signal s_c9           : signed(13 downto 0) := (others => '0');
  signal s_comp10       : signed(13 downto 0) := (others => '0');

  -- OP2 with kc[P], F = 8: pre = (X<<8)>>a - (X<<8)>>b (exact for F >= b)
  function f_pre(x : signed; p8 : boolean) return signed is
    variable v : signed(21 downto 0);
  begin
    v := resize(x, 22);
    if p8 then  -- a = 2, b = 8: X*64 - X
      return shift_left(v, 6) - v;
    else        -- a = 1, b = 4: X*128 - X*16
      return shift_left(v, 7) - shift_left(v, 4);
    end if;
  end function;

  function f_step(s : signed; pre : signed; p8 : boolean) return signed is
  begin
    if p8 then
      return s - shift_right(s, 2) + shift_right(s, 8) + pre;
    else
      return s - shift_right(s, 1) + shift_right(s, 4) + pre;
    end if;
  end function;

begin

  -- ------------------------------------------------------------------ chroma
  p_chroma : process(clk)
    variable v_uc, v_vc : signed(11 downto 0);
  begin
    if rising_edge(clk) then
      -- stage 2
      if i_avid = '1' then
        v_uc := resize(signed('0' & i_u) - 512, 12);
        v_vc := resize(signed('0' & i_v) - 512, 12);
      else
        v_uc := (others => '0');
        v_vc := (others => '0');
      end if;
      s_us2  <= v_uc - shift_right(v_uc, 3);
      s_vs2  <= v_vc + shift_right(v_vc, 2) - shift_right(v_vc, 6);
      s_sol2 <= i_sol;
      s_p8_2 <= i_p8;
      s_gate(2) <= i_avid;
      -- stage 3
      s_pu3  <= f_pre(s_us2, s_p8_2);
      s_pv3  <= f_pre(s_vs2, s_p8_2);
      s_sol3 <= s_sol2;
      s_p8_3 <= s_p8_2;
      -- stage 4 (states cleared on the x = 0 sample)
      if s_sol3 = '1' then
        s_s1u <= (others => '0');
        s_s1v <= (others => '0');
      else
        s_s1u <= f_step(s_s1u, s_pu3, s_p8_3);
        s_s1v <= f_step(s_s1v, s_pv3, s_p8_3);
      end if;
      s_sol4 <= s_sol3;
      s_p8_4 <= s_p8_3;
      -- stage 5
      s_pu5  <= f_pre(shift_right(s_s1u, 8), s_p8_4);
      s_pv5  <= f_pre(shift_right(s_s1v, 8), s_p8_4);
      s_sol5 <= s_sol4;
      s_p8_5 <= s_p8_4;
      -- stage 6
      if s_sol5 = '1' then
        s_s2u <= (others => '0');
        s_s2v <= (others => '0');
      else
        s_s2u <= f_step(s_s2u, s_pu5, s_p8_5);
        s_s2v <= f_step(s_s2v, s_pv5, s_p8_5);
      end if;
      s_gate(3 to 9) <= s_gate(2 to 8);
    end if;
  end process;

  s_uf6 <= resize(shift_right(s_s2u, 8), 14);
  s_vf6 <= resize(shift_right(s_s2v, 8), 14);

  -- ------------------------------------------------------------------ encoder domain (luma tag, stage 6)
  u_dom : entity work.cbv_domain
    port map (
      clk => clk, i_hs_n => i_hs_lt, i_vs_n => i_vs_lt, i_avid => i_avid_lt,
      i_tid => i_tid, i_reg6 => i_reg6,
      o_hs_n => d_hs, o_vs_n => d_vs, o_avid => d_avid, o_x => d_x, o_sol => d_sol,
      o_psi => d_psi, o_lp => d_lp, o_tid => d_tid, o_reg6 => d_reg6,
      o_vline => d_vline, o_aline => d_aline);

  -- ------------------------------------------------------------------ modulator
  p_mod : process(clk)
    variable f      : t_fmt;
    variable v_m    : unsigned(2 downto 0);
    variable v_x    : integer;
    variable v_sync : integer;
    variable v_cv   : signed(13 downto 0);
    variable v_bv   : integer;
  begin
    if rising_edge(clk) then
      s_y6 <= i_y_lt;   -- Yd source: luma-tag stage 5 -> 6
      -- stage 7 (domain registers at stage 6)
      f := f_fmt(d_tid);
      if f.p8 then
        v_m := d_x(2 downto 0) + d_psi;
      else
        v_m := (d_x(1 downto 0) & '0') + d_psi;
      end if;
      s_m7   <= v_m;
      s_psi7 <= d_psi;
      s_hu7  <= f_h(s_uf6);
      s_hv7  <= f_h(s_vf6);
      s_uf7  <= s_uf6;
      s_vf7  <= s_vf6;
      s_t8_7 <= d_reg6(1);
      s_sw7  <= d_reg6(1) and d_lp;  -- sw = -1 iff T8 = PAL and LP = 1
      v_x := to_integer(d_x);
      if v_x >= f.bst and v_x < f.bst + f.blen and d_vline = '0' then
        s_bf7 <= '1';
      else
        s_bf7 <= '0';
      end if;
      v_sync := 0;
      if not f.tri then
        if d_hs = '0' then
          v_sync := C_SYNC_SD;
        elsif d_vs = '0' and (v_x < f.sd_v1 or (v_x >= f.sd_v2 and v_x < f.sd_v3)) then
          v_sync := C_SYNC_SD;
        end if;
      else
        if v_x < f.hs then
          v_sync := -C_SYNC_HD;
        elsif v_x < f.hs2 then
          v_sync := C_SYNC_HD;
        elsif d_vs = '0' and v_x < f.hd_v then
          v_sync := -C_SYNC_HD;
        end if;
      end if;
      s_sync7 <= to_signed(v_sync, 11);
      if d_avid = '1' then
        s_yd7 <= s_y6;
      else
        s_yd7 <= (others => '0');
      end if;

      -- stage 8: carrier products (S8/C8 in {0, +-H, +-1}; negation after the shift-sum)
      case to_integer(s_m7) is
        when 0 | 4  => s_tu8 <= (others => '0');
        when 1 | 3  => s_tu8 <= s_hu7;
        when 2      => s_tu8 <= s_uf7;
        when 5 | 7  => s_tu8 <= -s_hu7;
        when others => s_tu8 <= -s_uf7;   -- 6
      end case;
      case to_integer(s_m7) is
        when 2 | 6  => v_cv := (others => '0');
        when 1 | 7  => v_cv := s_hv7;
        when 0      => v_cv := s_vf7;
        when 3 | 5  => v_cv := -s_hv7;
        when others => v_cv := -s_vf7;    -- 4
      end case;
      if s_sw7 = '1' then
        s_tv8 <= -v_cv;
      else
        s_tv8 <= v_cv;
      end if;
      if s_t8_7 = '0' then
        v_bv := C_BURST_NTSC(to_integer(s_m7));
      elsif s_sw7 = '0' then
        v_bv := C_BURST_PAL_P(to_integer(s_m7));
      else
        v_bv := C_BURST_PAL_M(to_integer(s_m7));
      end if;
      if i_burst_en = '1' then
        s_bv8 <= to_signed(v_bv, 9);
      else
        s_bv8 <= (others => '0');
      end if;
      s_bf8   <= s_bf7;
      s_sync8 <= s_sync7;
      s_yd8   <= s_yd7;
      s_m8    <= s_m7;
      s_psi8  <= s_psi7;

      -- stage 9: C (burst replaces C in the burst window)
      if s_bf8 = '1' then
        s_c9 <= resize(s_bv8, 14);
      elsif s_gate(8) = '1' then
        s_c9 <= s_tu8 + s_tv8;
      else
        s_c9 <= (others => '0');
      end if;
      s_sync9 <= s_sync8;
      s_yd9   <= s_yd8;
      s_m9    <= s_m8;
      s_psi9  <= s_psi8;

      -- stage 10: comp
      s_comp10 <= f_sat(resize(signed('0' & s_yd9), 16) + resize(s_c9, 16) + resize(s_sync9, 16), 14);
      s_m10    <= s_m9;
      s_psi10  <= s_psi9;
    end if;
  end process;

  o_comp <= s_comp10;
  o_m    <= s_m10;
  o_psi  <= s_psi10;

end architecture rtl;
