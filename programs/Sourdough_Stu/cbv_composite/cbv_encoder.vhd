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
-- Two sample streams meet at the modulator (RULED Q1, spec rev 3 §13):
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
--   8  S8[m]*Uf, sw*C8[m]*Vf selected (negation after the H shift-sum, RULED Q8)
--   9  C = burst ? BURST[m] : (gateE ? tU + tV : 0)
--   10 comp = sat14(Yd + C + SYNC)                    -> o_comp
--   kc: P=4 7/16 (a=1, b=4); P=8 63/256 (a=2, b=8); F = 8 (RULED Q17: §5.1 rule is F >= a + 3).
--   P and kc are bitstream constants (C_ENABLE_HD, R4.1-9). G3 (C_ENC_1POLE):
--   one pole, Uf delayed 2 to stage 6 (YE 3 / 2 in the core).
-- Stepped Hue Bend (spec rev 4.1 §5.2, M5'), at stage 7 on the luma tag x:
--   kh = ((P5 >> 4) (Ycode >> 3)) >> 10 (0 when AVID(x) = 0), sgn = -1 iff PAL and LP
--   m_c = ((8/P) x + psi + sgn kh) mod 8 (chroma), m_b = ((8/P) x + psi) mod 8 (burst)
-- Colour-leak source (§5.5, G5): cm = max(|Uf|,|Vf|) + (min >> 1), sat 2047, 0
--   when gateE = 0; col = (cm 410) >> 10 at stage 8 (0 when C_COLOUR_LEAK = false).
-- C_MONO: luma-only composite comp = sat(Yd + SYNC): no chroma poles, carrier,
--   burst or Hue Bend; YE = 0, so the luma tag is the input sample; the leak's
--   cm = |Uc| + |Vc| of that input sample (0 outside AVID), sat 2047.
-- Latency: chroma stage 1 -> comp 9 clocks (luma-tag stage 1 -> comp 9).
-- Resources (paper): (a) ~320 LC, (b) ~160 LC (luma tag chain is in the core).
-- No EBR, no multipliers.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_variant_pkg.all;
use work.cbv_pkg.all;

entity cbv_encoder is
  port (
    clk         : in  std_logic;
    -- chroma stream, stage 1
    i_u         : in  unsigned(9 downto 0);
    i_v         : in  unsigned(9 downto 0);
    i_avid      : in  std_logic;
    i_sol       : in  std_logic;   -- input-domain x = 0 (pole clear)
    -- luma-tag stream, stage 5 (registered here into stage 6)
    i_y_lt      : in  unsigned(9 downto 0);
    i_hs_lt     : in  std_logic;
    i_vs_lt     : in  std_logic;
    i_avid_lt   : in  std_logic;
    i_tid       : in  t_tid;
    i_reg6      : in  std_logic_vector(4 downto 0);
    i_p5        : in  std_logic_vector(9 downto 0);   -- registers_in(4) Hue Bend
    i_burst_en  : in  std_logic;   -- G_TEST_BURST_GAIN (1 = normal)
    -- outputs, stage 10
    o_comp      : out signed(13 downto 0);
    o_m         : out unsigned(2 downto 0);   -- m_c (T1)
    o_mb        : out unsigned(2 downto 0);   -- m_b
    o_kh        : out unsigned(2 downto 0);
    o_psi       : out unsigned(2 downto 0);
    o_col       : out unsigned(9 downto 0);   -- colour-leak source, stage 8
    o_sol       : out std_logic;              -- encoder-domain x = 0 (stage 6)
    o_tid       : out t_tid
  );
end entity cbv_encoder;

architecture rtl of cbv_encoder is
  -- chroma
  signal s_us2, s_vs2   : signed(11 downto 0) := (others => '0');
  signal s_sol2, s_sol3, s_sol4 : std_logic := '0';
  signal s_y1u, s_y1v   : signed(11 downto 0);
  signal s_y2u, s_y2v   : signed(11 downto 0) := (others => '0');
  signal s_d5u, s_d5v   : signed(11 downto 0) := (others => '0');
  signal s_mb7, s_mb8, s_mb9, s_mb10 : unsigned(2 downto 0) := (others => '0');
  signal s_kh7, s_kh8, s_kh9, s_kh10 : unsigned(2 downto 0) := (others => '0');
  signal s_p5   : unsigned(5 downto 0) := (others => '0');
  signal s_cm7  : unsigned(10 downto 0) := (others => '0');
  type t_mcm is array (2 to 6) of unsigned(10 downto 0);
  signal s_mcm  : t_mcm := (others => (others => '0'));
  signal s_col8 : unsigned(9 downto 0) := (others => '0');
  signal s_gate         : std_logic_vector(2 to 9) := (others => '0');
  signal s_uf6, s_vf6   : signed(13 downto 0);
  signal s_hu_a, s_hu_b, s_hv_a, s_hv_b : signed(13 downto 0);
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
  signal s_su8          : std_logic := '0';
  signal s_bv8          : signed(8 downto 0) := (others => '0');
  signal s_c9           : signed(13 downto 0) := (others => '0');
  signal s_comp10       : signed(13 downto 0) := (others => '0');

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
      s_gate(2) <= i_avid;
      -- per-sample P / clear flags for the second pole (sample at stage 4)
      s_sol3 <= s_sol2; s_sol4 <= s_sol3;
      s_gate(3 to 9) <= s_gate(2 to 8);
    end if;
  end process;

  -- two cascaded OP2 per channel (pre 3 / S 4, pre 5 / S 6); cleared on x = 0.
  -- State s20: |Us|, |Vs| <= 632 so |S| <= 632*256 + 1/k < 2^18 (nothing wraps)
  u_p1u : entity work.cbv_op2s generic map (G_WX => 12, G_WS => 20, G_F => 8, G_K0 => C_KC, G_K1 => C_KC)
    port map (clk => clk, i_x => s_us2, i_sel => '0', i_clr => s_sol2, o_s => open, o_y => s_y1u);
  u_p1v : entity work.cbv_op2s generic map (G_WX => 12, G_WS => 20, G_F => 8, G_K0 => C_KC, G_K1 => C_KC)
    port map (clk => clk, i_x => s_vs2, i_sel => '0', i_clr => s_sol2, o_s => open, o_y => s_y1v);
  g_2pole : if not C_ENC_1POLE and not C_MONO generate
    u_p2u : entity work.cbv_op2s generic map (G_WX => 12, G_WS => 20, G_F => 8, G_K0 => C_KC, G_K1 => C_KC)
      port map (clk => clk, i_x => s_y1u, i_sel => '0', i_clr => s_sol4, o_s => open, o_y => s_y2u);
    u_p2v : entity work.cbv_op2s generic map (G_WX => 12, G_WS => 20, G_F => 8, G_K0 => C_KC, G_K1 => C_KC)
      port map (clk => clk, i_x => s_y1v, i_sel => '0', i_clr => s_sol4, o_s => open, o_y => s_y2v);
  end generate;
  g_1pole : if C_ENC_1POLE and not C_MONO generate     -- G3: one pole, pipelined 2 to stage 6
    p_d2 : process(clk)
    begin
      if rising_edge(clk) then
        s_d5u <= s_y1u; s_d5v <= s_y1v;
        s_y2u <= s_d5u; s_y2v <= s_d5v;
      end if;
    end process;
  end generate;

  s_uf6 <= resize(s_y2u, 14) when not C_MONO else (others => '0');
  s_vf6 <= resize(s_y2v, 14) when not C_MONO else (others => '0');

  -- mono colour-leak source: |Uc| + |Vc| of the input sample, carried 2 -> 7
  g_mcm : if C_MONO generate
    p_mcm : process(clk)
      variable v_u, v_v : signed(11 downto 0);
    begin
      if rising_edge(clk) then
        if i_avid = '1' then
          v_u := resize(signed('0' & i_u) - 512, 12); v_v := resize(signed('0' & i_v) - 512, 12);
        else
          v_u := (others => '0'); v_v := (others => '0');
        end if;
        s_mcm(2) <= resize(unsigned(abs(v_u)), 11) + resize(unsigned(abs(v_v)), 11);
        s_mcm(3 to 6) <= s_mcm(2 to 5);
      end if;
    end process;
  end generate;

  -- H*Uf = (Uf>>1 + Uf>>3) + (Uf>>4 + Uf>>6), 2-operand adds
  s_hu_a <= shift_right(s_uf6, 1) + shift_right(s_uf6, 3);
  s_hu_b <= shift_right(s_uf6, 4) + shift_right(s_uf6, 6);
  s_hv_a <= shift_right(s_vf6, 1) + shift_right(s_vf6, 3);
  s_hv_b <= shift_right(s_vf6, 4) + shift_right(s_vf6, 6);

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
    variable v_p5   : unsigned(5 downto 0);
    variable v_kh   : unsigned(2 downto 0);
    variable v_au, v_av : unsigned(14 downto 0);
    variable v_cm   : unsigned(15 downto 0);
  begin
    if rising_edge(clk) then
      s_y6 <= i_y_lt;   -- Yd source: luma-tag stage 5 -> 6
      -- stage 7 (domain registers at stage 6)
      f := f_fmt(d_tid);
      if C_P8 then
        v_m := d_x(2 downto 0) + d_psi;
      else
        v_m := (d_x(1 downto 0) & '0') + d_psi;
      end if;
      -- Hue Bend: P5 latched at this domain's x = 0 (R-LATCH)
      if d_sol = '1' then
        v_p5 := unsigned(i_p5(9 downto 4));
        s_p5 <= unsigned(i_p5(9 downto 4));
      else
        v_p5 := s_p5;
      end if;
      if d_avid = '1' and not C_MONO then
        v_kh := resize(shift_right(v_p5 * s_y6(9 downto 3), 10), 3);
      else
        v_kh := (others => '0');
      end if;
      s_kh7 <= v_kh;
      s_mb7 <= v_m;
      if d_reg6(1) = '1' and d_lp = '1' then
        s_m7 <= v_m - v_kh;
      else
        s_m7 <= v_m + v_kh;
      end if;
      -- colour-leak source cm (chroma of this comp clock, gateE)
      v_au := unsigned(abs(resize(s_uf6, 15)));
      v_av := unsigned(abs(resize(s_vf6, 15)));
      if v_au >= v_av then v_cm := resize(v_au, 16) + resize(shift_right(v_av, 1), 16);
      else v_cm := resize(v_av, 16) + resize(shift_right(v_au, 1), 16); end if;
      if not C_COLOUR_LEAK then
        s_cm7 <= (others => '0');
      elsif C_MONO then
        s_cm7 <= s_mcm(6);
      elsif s_gate(6) = '0' then
        s_cm7 <= (others => '0');
      elsif v_cm > 2047 then
        s_cm7 <= to_unsigned(2047, 11);
      else
        s_cm7 <= v_cm(10 downto 0);
      end if;
      s_col8 <= resize(shift_right(s_cm7 * to_unsigned(410, 9), 10), 10);
      s_psi7 <= d_psi;
      s_hu7  <= s_hu_a + s_hu_b;
      s_hv7  <= s_hv_a + s_hv_b;
      s_uf7  <= s_uf6;
      s_vf7  <= s_vf6;
      s_t8_7 <= d_reg6(1);
      s_sw7  <= d_reg6(1) and d_lp;  -- sw = -1 iff T8 = PAL and LP = 1
      v_x := to_integer(d_x);
      if v_x >= f.bst and v_x < f.bst + f.blen and d_vline = '0' and not C_MONO then
        s_bf7 <= '1';
      else
        s_bf7 <= '0';
      end if;
      v_sync := 0;
      if not C_HD then
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
      -- sign-magnitude: |S8| in {0, H, 1} with sign m in 5..7; |C8| with sign m in
      -- 3..5 (xor sw). U keeps magnitude + sign (applied in the stage-9 add/sub),
      -- V is negated here (one incrementer). Values identical to +-H*z, +-z.
      case to_integer(s_m7) is
        when 0 | 4      => s_tu8 <= (others => '0');
        when 2 | 6      => s_tu8 <= s_uf7;
        when others     => s_tu8 <= s_hu7;
      end case;
      if s_m7 >= 5 then s_su8 <= '1'; else s_su8 <= '0'; end if;
      case to_integer(s_m7) is
        when 2 | 6      => v_cv := (others => '0');
        when 0 | 4      => v_cv := s_vf7;
        when others     => v_cv := s_hv7;
      end case;
      if (s_m7 >= 3 and s_m7 <= 5) xor (s_sw7 = '1') then
        s_tv8 <= f_negif(v_cv, '1');
      else
        s_tv8 <= v_cv;
      end if;
      if s_t8_7 = '0' then
        v_bv := C_BURST_NTSC(to_integer(s_mb7));
      elsif s_sw7 = '0' then
        v_bv := C_BURST_PAL_P(to_integer(s_mb7));
      else
        v_bv := C_BURST_PAL_M(to_integer(s_mb7));
      end if;
      if i_burst_en = '1' then
        s_bv8 <= to_signed(v_bv, 9);
      else
        s_bv8 <= (others => '0');
      end if;
      s_bf8   <= s_bf7;
      s_sync8 <= s_sync7;
      s_yd8   <= s_yd7;
      s_m8    <= s_m7;  s_mb8 <= s_mb7;  s_kh8 <= s_kh7;
      s_psi8  <= s_psi7;

      -- stage 9: C (burst replaces C in the burst window)
      if s_bf8 = '1' then
        s_c9 <= resize(s_bv8, 14);
      elsif s_gate(8) = '1' then
        s_c9 <= f_addsub(s_tv8, s_tu8, s_su8);
      else
        s_c9 <= (others => '0');
      end if;
      s_sync9 <= s_sync8;
      s_yd9   <= s_yd8;
      s_m9    <= s_m8;  s_mb9 <= s_mb8;  s_kh9 <= s_kh8;
      s_psi9  <= s_psi8;

      -- stage 10: comp
      s_comp10 <= f_sat(resize(signed('0' & s_yd9), 16) + resize(s_c9, 16) + resize(s_sync9, 16), 14);
      s_m10    <= s_m9;  s_mb10 <= s_mb9;  s_kh10 <= s_kh9;
      s_psi10  <= s_psi9;
    end if;
  end process;

  o_comp <= s_comp10;
  o_m    <= s_m10;
  o_mb   <= s_mb10;
  o_kh   <= s_kh10;
  o_col  <= s_col8;
  o_sol  <= d_sol;
  o_tid  <= d_tid;
  o_psi  <= s_psi10;

end architecture rtl;
