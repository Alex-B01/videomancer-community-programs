-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_output.vhd - CBV Composite output gain/knee + Tame-buffer write side (block 11, spec §5.14, §1.3)
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
-- Luma: Yw = y - (y>>3) + (y>>5) (x0.90625); if Yw > 716: knee
--   Yw = 716 + KN[e>>4] + (((KN[(e>>4)+1] - KN[e>>4]) (e & 15)) >> 4),
--   e = sat(Yw - 716, 0, 1023). G_CODEC_ONLY: Yw = y. Written sat(Yw, 0, 1023).
--   The luma is delayed so that it reaches the write register together
--   with U4/V4's results (spec §5.10 "aligned at the write").
-- Write side (rolling base, spec §1.3), write domain = stage 36 newest tag x_w:
--   B   : B <- (B + S) mod 2048 at every write-domain line start (B_0 = 0);
--         the x = 0 sample already uses the new B (RULED Q11)
--   Y   : p = x_w - h/2 - act0 (notch-centred luma); we = 0 <= p < actw on
--         every line; address (B + p) mod 2048
--   U/V : at each U4-valid tag ((x_w - act0) mod 4 = 2 / 1): word index
--         q = (x_w - act0 - kq)/4, kq = 10 (P=8) / 5 (P=4), exact (RULED Q2);
--         we = 0 <= q < actw/4; address (B/4 + q) mod 512; word (U8, V8)
--   dh_l = 0 in Stage 1 (port reserved).
-- G_LOOPBACK (H2): the write data are the raw input aligned to the write
--   stage: Y of the input sample at the luma position; U/V = (Ucode >> 2,
--   Vcode >> 2) of the input sample at 4 q + act0.
-- Pipeline: 16 y | 17 ya, yb | 18 yg | 19 knee index | 20 KN, dk | 21 dk*f |
--   22 knee | 23 sat | 24 -> delay -> 36 | 37 write registers (T6). RAM write
--   on the next edge.
-- Resources (paper): ~160 LC + luma delay ~120 FF; no EBR (RAMs in cbv_tame_buffer).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_pkg.all;

entity cbv_output is
  generic (
    G_CODEC_ONLY : boolean := false;
    G_LOOPBACK   : boolean := false
  );
  port (
    clk      : in  std_logic;
    i_y      : in  signed(15 downto 0);     -- stage 16
    i_u8     : in  unsigned(7 downto 0);    -- stage 36
    i_v8     : in  unsigned(7 downto 0);
    i_uv_ok  : in  std_logic;               -- stage 36
    -- write domain, stage 36
    i_x      : in  unsigned(11 downto 0);
    i_sol    : in  std_logic;
    i_tid    : in  t_tid;
    i_dh     : in  signed(11 downto 0);     -- dh_l (0 in Stage 1)
    -- raw input, stage 1 (G_LOOPBACK only)
    i_raw_y  : in  unsigned(9 downto 0);
    i_raw_u  : in  unsigned(9 downto 0);
    i_raw_v  : in  unsigned(9 downto 0);
    -- write port registers, stage 37 (T6)
    o_we_y   : out std_logic;
    o_wa_y   : out unsigned(10 downto 0);
    o_wd_y   : out unsigned(9 downto 0);
    o_we_uv  : out std_logic;
    o_wa_uv  : out unsigned(8 downto 0);
    o_q      : out signed(11 downto 0);              -- UV word index q (T6)
    o_wd_uv  : out std_logic_vector(15 downto 0);   -- U8 & V8
    o_b      : out unsigned(10 downto 0)            -- write base B used by the stage-37 sample
  );
end entity cbv_output;

architecture rtl of cbv_output is
  constant C_YDLY : natural := C_ST_UV8 - 24;   -- 12
  type t_ydly is array (1 to C_YDLY) of unsigned(9 downto 0);
  signal s_ydly : t_ydly := (others => (others => '0'));

  signal s_y17, s_ya17, s_yb17 : signed(15 downto 0) := (others => '0');
  signal s_yg18, s_yg19, s_yg20, s_yg21 : signed(15 downto 0) := (others => '0');
  signal s_big19, s_big20, s_big21 : std_logic := '0';
  signal s_ki19 : unsigned(5 downto 0) := (others => '0');
  signal s_kb20, s_kb21 : signed(10 downto 0) := (others => '0');  -- 716 + KN[k]
  signal s_ef19, s_ef20 : unsigned(3 downto 0) := (others => '0');
  signal s_dk20 : unsigned(4 downto 0) := (others => '0');
  signal s_pr21 : unsigned(4 downto 0) := (others => '0');
  signal s_yw22 : signed(15 downto 0) := (others => '0');
  signal s_yw23, s_yw24 : unsigned(9 downto 0) := (others => '0');

  signal s_b    : unsigned(10 downto 0) := (others => '0');
  signal s_b36  : unsigned(10 downto 0) := (others => '0');
  signal s_q36  : signed(11 downto 0) := (others => '0');
  signal s_we_y, s_we_uv : std_logic := '0';
  signal s_wa_y  : unsigned(10 downto 0) := (others => '0');
  signal s_wd_y  : unsigned(9 downto 0) := (others => '0');
  signal s_wa_uv : unsigned(8 downto 0) := (others => '0');
  signal s_wd_uv : std_logic_vector(15 downto 0) := (others => '0');

  -- loopback raw chain (simulation only; never driven when G_LOOPBACK = false):
  -- s_raw(d) = stage-1 input delayed d clocks
  constant C_RAW_LEN : natural := 56;
  type t_raw is array (1 to C_RAW_LEN) of std_logic_vector(29 downto 0);
  signal s_raw : t_raw := (others => (others => '0'));
begin

  -- ------------------------------------------------------------------ luma gain / knee
  p_luma : process(clk)
    variable v_e  : signed(15 downto 0);
    variable v_k  : natural range 0 to 63;
    variable v_yw : signed(15 downto 0);
  begin
    if rising_edge(clk) then
      -- 17
      s_ya17 <= i_y - shift_right(i_y, 3);
      s_yb17 <= shift_right(i_y, 5);
      s_y17  <= i_y;
      -- 18 (G_CODEC_ONLY: identity gain, no knee)
      if G_CODEC_ONLY then
        s_yg18 <= s_y17;
      else
        s_yg18 <= s_ya17 + s_yb17;
      end if;
      -- 19: knee index registered first (timing: saturate | ROM + add in separate clocks)
      v_e := f_satr(resize(s_yg18, 17) - C_KNEE_T, 0, 1023, 16);
      s_ki19 <= unsigned(v_e(9 downto 4));
      s_ef19 <= unsigned(v_e(3 downto 0));
      if s_yg18 > C_KNEE_T and not G_CODEC_ONLY then s_big19 <= '1'; else s_big19 <= '0'; end if;
      s_yg19 <= s_yg18;
      -- 20: KN reads straight into arithmetic (keeps the 65-entry table in LUTs, A6)
      v_k := to_integer(s_ki19);
      s_dk20   <= resize(to_unsigned(C_KN(v_k + 1), 9) - to_unsigned(C_KN(v_k), 9), 5);
      s_kb20   <= to_signed(C_KNEE_T, 11) + signed(resize(to_unsigned(C_KN(v_k), 9), 11));
      s_ef20   <= s_ef19;
      s_big20  <= s_big19;
      s_yg20   <= s_yg19;
      -- 21
      s_pr21   <= resize(shift_right(s_dk20 * s_ef20, 4), 5);
      s_kb21   <= s_kb20;
      s_big21  <= s_big20;
      s_yg21   <= s_yg20;
      -- 22
      if s_big21 = '1' then
        v_yw := resize(s_kb21, 16) + signed(resize(s_pr21, 16));
      else
        v_yw := s_yg21;
      end if;
      s_yw22 <= v_yw;
      -- 23, 24
      s_yw23 <= unsigned(f_satr(resize(s_yw22, 17), 0, 1023, 11)(9 downto 0));
      s_yw24 <= s_yw23;
      -- delay 25 .. 36
      s_ydly(1) <= s_yw24;
      s_ydly(2 to C_YDLY) <= s_ydly(1 to C_YDLY - 1);
    end if;
  end process;

  -- ------------------------------------------------------------------ write side (stage 36 -> 37)
  p_write : process(clk)
    variable f     : t_fmt;
    variable v_py  : signed(13 downto 0);
    variable v_pu  : signed(13 downto 0);
    variable v_yd  : unsigned(9 downto 0);
    variable v_uvd : std_logic_vector(15 downto 0);
    variable v_r   : std_logic_vector(29 downto 0);
    variable v_b   : unsigned(10 downto 0);
  begin
    if rising_edge(clk) then
      f := f_fmt(i_tid);
      if i_sol = '1' then
        v_b := s_b + to_unsigned(f.s, 11);
        s_b <= v_b;
      else
        v_b := s_b;
      end if;
      s_b36 <= v_b;
      v_py := resize(signed('0' & i_x), 14) - to_signed(f.h2 + f.act0, 14) + resize(i_dh, 14);
      v_pu := resize(signed('0' & i_x), 14) - to_signed(f.kq + f.act0, 14) + resize(i_dh, 14);
      v_yd := s_ydly(C_YDLY);
      v_uvd := std_logic_vector(i_u8) & std_logic_vector(i_v8);
      if G_LOOPBACK then
        v_r := s_raw(C_ST_UV8 - 1 + f.ye + f.h2);
        v_yd := unsigned(v_r(29 downto 20));
        v_r := s_raw(C_ST_UV8 - 1 + f.ye + f.kq);
        v_uvd := v_r(19 downto 12) & v_r(9 downto 2);
      end if;
      -- Y
      if v_py >= 0 and v_py < f.actw then
        s_we_y <= '1';
      else
        s_we_y <= '0';
      end if;
      s_wa_y <= v_b + unsigned(v_py(10 downto 0));
      s_wd_y <= v_yd;
      -- U/V
      if i_uv_ok = '1' and v_pu >= 0 and v_pu < f.actw then
        s_we_uv <= '1';
      else
        s_we_uv <= '0';
      end if;
      s_wa_uv <= v_b(10 downto 2) + unsigned(v_pu(10 downto 2));
      s_q36   <= resize(shift_right(v_pu, 2), 12);
      s_wd_uv <= v_uvd;
      -- pragma translate_off
      assert not (i_uv_ok = '1' and v_pu(1 downto 0) /= "00" and i_x /= x"FFF")
        report "cbv_output: /4 result not on a U4-valid tag" severity warning;
      -- pragma translate_on
    end if;
  end process;

  g_loop : if G_LOOPBACK generate
    p_raw : process(clk)
    begin
      if rising_edge(clk) then
        s_raw(1) <= std_logic_vector(i_raw_y) & std_logic_vector(i_raw_u) & std_logic_vector(i_raw_v);
        s_raw(2 to C_RAW_LEN) <= s_raw(1 to C_RAW_LEN - 1);
      end if;
    end process;
  end generate;

  o_we_y  <= s_we_y;
  o_wa_y  <= s_wa_y;
  o_wd_y  <= s_wd_y;
  o_we_uv <= s_we_uv;
  o_wa_uv <= s_wa_uv;
  o_wd_uv <= s_wd_uv;
  o_b     <= s_b36;
  o_q     <= s_q36;

end architecture rtl;
