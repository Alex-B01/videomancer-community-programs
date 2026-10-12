-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_composite_core.vhd - CBV Composite program core, Stage 2 assembly (sync-domain program, spec rev 4.1)
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
-- The whole program with test generics; cbv_composite.vhd (architecture of
-- program_top) instantiates it with the shipping values. Testbenches use the
-- o_taps record port (spec-stage2 §7; open in program_top, so synthesis prunes
-- it) and the test ports below.
--
-- Generics (defaults from cbv_variant_pkg): G_CODEC_ONLY, G_LOOPBACK.
-- Test ports (tied in program_top): i_test_burst_gain ('1'), i_test_dh_en /
--   i_test_dh (L6: force dh_l at the separator output; '0').
--
-- Stage table (clock index; stage 1 = input register; tags per spec §2.1):
--   1      input register, input domain, 8-deep sync chain tap 1
--   2-10   encoder (chroma 2-6, luma tag + domain 6, comp 10)        T1
--   11-13  Enhance (nearest; 15 with C_ENH_INTERP)                   T1b
--   14-28  cell-lite (w 17, hf 20, vv 24, o 28) + ea                 T1c
--   28-33  leak mix, separator input s at 33                        T1d
--   33-50  separator / AFC / Tame, dh_l at 50                       T1e
--   50-53  clamp (o delayed to dh_l's tag), sd at 53                T2
--   53-56  Y/C split                                                T3
--   56-60  demod, du 59, /4 strobe 60, commit 64, poles 65-68        T4
--   53-~   burst gate (53-59), serial colour engine                 T5
--   68-74  colour picture (U8/V8 strobe 74), write domain 74
--   75     write-port registers (L_w = 75)                          T6
--   (all shifted by +2 with C_ENH_INTERP)
-- Boundary (spec §1.1): 8 = C_PROCESSING_DELAY_CLKS; one 8-deep chain carries
--   AVID, HSYNC_n, VSYNC_n, FIELD_n; data_out.avid is its tap 8.        T7
--
-- EBR map (physical):
--   Enhance ROM 4096x10 (or 2 x 2048x10) ...... 10  (cbv_enhance)
--   comb RAM 2048x10 + 512x10 .................. 7  (cbv_ycsplit)
--   Tame buffer Y 2048x10 ...................... 5  (cbv_tame_buffer)
--   Tame buffer UV 512x16 ...................... 2  (cbv_tame_buffer)
--   delay lines (C_DELAY_EBR, R4.1-12) ......... 4  (cbv_delay):
--     D1 comp 10 -> 32 (14 b x 22); D2 {o, hs, avid} 28 -> 49 (16 b x 21);
--     D3 {col, hs, vs, avid} 8 -> 27 (13 b x 19); D4 {dh_l, hs, vs, avid} 50 -> 73 (15 b x 23)
--   program total 28 (29 on dual configs incl. the core's dual_sync_delay); 2 spare.
-- Register map: registers_in(0) Enhance, (1) Tame, (2) Leak, (3) Sep Filter,
--   (4) Hue Bend, (5) Clamp Pos, (6)(4:0) T7-T11, (7) Colour, (8)(3:0) timing ID.
--   Never read: registers_in(8)(9:4), registers_in(9..11). All per-line values
--   latch at x = 0 of the consuming domain (R-LATCH).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.cbv_variant_pkg.all;
use work.cbv_pkg.all;

entity cbv_composite_core is
  generic (
    G_CODEC_ONLY : boolean := C_CODEC_ONLY;
    G_LOOPBACK   : boolean := C_LOOPBACK
  );
  port (
    clk               : in  std_logic;
    registers_in      : in  t_spi_ram;
    data_in           : in  t_video_stream_yuv444_30b;
    data_out          : out t_video_stream_yuv444_30b;
    i_test_burst_gain : in  std_logic := '1';
    i_test_dh_en      : in  std_logic := '0';
    i_test_dh         : in  signed(11 downto 0) := (others => '0');
    o_taps            : out t_cbv_taps
  );
end entity cbv_composite_core;

architecture rtl of cbv_composite_core is

  constant C_SYNC_DEPTH : integer := 8;   -- = C_PROCESSING_DELAY_CLKS (cbv_composite.vhd)

  -- domain record (subset of cbv_domain outputs)
  type t_dom is record
    hs, vs, avid, sol, lp, vline : std_logic;
    x, a0m : unsigned(11 downto 0);
    psi : unsigned(2 downto 0);
    tid : t_tid;
    reg6 : std_logic_vector(4 downto 0);
    aline : unsigned(1 downto 0);
  end record;
  signal di, dl, dsp, dc, dy, dw : t_dom;   -- input, leak (O), separator (S), clamp, Y/C, write

  signal s_reg6 : std_logic_vector(4 downto 0);
  signal s_tid  : t_tid;

  -- stage 1
  signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
  type t_sync is array (1 to C_SYNC_DEPTH) of std_logic_vector(3 downto 0);
  signal s_sync : t_sync := (others => "0111");

  -- luma-tag stream (YE tap) and its flag chain up to stage 8
  type t_ye is array (1 to 6) of std_logic_vector(12 downto 0);
  signal s_ye  : t_ye := (others => (12 => '1', 11 => '1', others => '0'));
  signal s_lt0 : std_logic_vector(12 downto 0);
  type t_fl is array (1 to 8) of std_logic_vector(2 downto 0);   -- (hs, vs, avid)
  signal s_fl  : t_fl := (others => "110");
  type t_yl is array (1 to 5) of unsigned(9 downto 0);
  signal s_yl  : t_yl := (others => (others => '0'));

  -- encoder
  signal s_comp : signed(13 downto 0);
  signal s_m_c, s_m_b, s_kh, s_psi_e : unsigned(2 downto 0);
  signal s_col8 : unsigned(9 downto 0);
  signal s_esol : std_logic_vector(7 to C_ST_VIN) := (others => '0');   -- encoder-domain x = 0 pipe
  signal s_eed  : std_logic_vector(7 to C_ST_VIN) := (others => '0');   -- 27 MHz set flag pipe
  signal de_sol : std_logic;
  signal de_tid : t_tid;

  -- Enhance / cell / leak
  signal s_vin  : signed(13 downto 0);
  signal s_row  : unsigned(4 downto 0);
  signal s_w, s_vv, s_o : signed(13 downto 0);
  signal s_hf   : signed(14 downto 0);
  signal s_ea   : unsigned(11 downto 0);
  signal s_pic, s_leak : unsigned(11 downto 0);
  signal s_sep_in : signed(13 downto 0);
  signal s_comp_al : signed(13 downto 0);
  signal s_col_al  : unsigned(9 downto 0);

  -- delay lines
  signal w_d1_i, w_d1_o : std_logic_vector(13 downto 0);
  signal w_d2_i, w_d2_o : std_logic_vector(15 downto 0);
  signal w_d3_i, w_d3_o : std_logic_vector(12 downto 0);
  signal w_d4_i, w_d4_o : std_logic_vector(14 downto 0);
  signal s_col28 : unsigned(9 downto 0) := (others => '0');
  signal s_fl28  : std_logic_vector(2 downto 0) := "110";
  type t_fl5 is array (C_ST_O to C_ST_S - 1) of std_logic_vector(2 downto 0);
  signal s_fls   : t_fl5 := (others => "110");
  type t_vsp is array (C_ST_O + 1 to C_ST_CLAMP - 1) of std_logic;
  signal s_vs_p  : t_vsp := (others => '1');
  signal s_o50   : signed(13 downto 0) := (others => '0');
  type t_fl3 is array (C_ST_CLAMP - 1 to C_ST_SD - 1) of std_logic_vector(2 downto 0);
  signal s_flc   : t_fl3 := (others => "110");

  -- separator
  signal s_dh_sep, s_dh50 : signed(11 downto 0);
  signal s_f, s_ti, s_bp : signed(13 downto 0);
  signal s_sl : std_logic;
  signal s_hph, s_hper : unsigned(19 downto 0);
  signal s_ef, s_dh8 : signed(20 downto 0);
  signal s_z : unsigned(5 downto 0);
  type t_dhp is array (C_ST_CLAMP + 1 to C_ST_SD) of signed(11 downto 0);
  signal s_dhp : t_dhp := (others => (others => '0'));
  signal s_dh_wr : signed(11 downto 0) := (others => '0');

  -- clamp, Y/C, demod, colour
  signal s_sd, s_bpk : signed(15 downto 0);
  signal s_bpk_ld : std_logic;
  signal s_ca : unsigned(11 downto 0);
  signal s_y, s_c16 : signed(15 downto 0);
  signal s_cs : signed(12 downto 0);
  signal s_comb_en : std_logic;
  signal s_old : signed(9 downto 0);
  signal s_col_yc, s_col_on : std_logic;
  signal s_md : unsigned(2 downto 0);
  signal s_du, s_dv, s_i4u, s_i4v, s_pin_u, s_pin_v, s_u4, s_v4 : signed(14 downto 0);
  signal s_stb4, s_cmt, s_u4_ok : std_logic;
  signal s_boff : unsigned(5 downto 0);
  signal s_dm : unsigned(2 downto 0);
  signal s_da : unsigned(15 downto 0);
  signal s_u8, s_v8 : unsigned(7 downto 0);
  signal s_uv_ok : std_logic;
  type t_solp is array (C_ST_SD + 1 to C_ST_U4) of std_logic;
  signal s_solp : t_solp := (others => '0');
  -- T5
  signal t5_stb, t5_sw_tv, t5_sw_pic : std_logic;
  signal t5_bu, t5_bv : signed(19 downto 0);
  signal t5_mag : unsigned(20 downto 0);
  signal t5_am : unsigned(36 downto 0);
  signal t5_accq : unsigned(14 downto 0);
  signal t5_gu9, t5_gv9 : unsigned(11 downto 0);
  signal t5_codes : std_logic_vector(19 downto 0);
  signal t5_kcnt : unsigned(10 downto 0);
  signal t5_idc : unsigned(2 downto 0);

  -- write port, read side
  signal s_we_y, s_we_uv : std_logic;
  signal s_wa_y  : unsigned(10 downto 0);
  signal s_wd_y  : unsigned(9 downto 0);
  signal s_wa_uv : unsigned(8 downto 0);
  signal s_q     : signed(11 downto 0);
  signal s_wd_uv : std_logic_vector(15 downto 0);
  signal s_b     : unsigned(10 downto 0);
  signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0);

  function f_ed(t : t_tid) return std_logic is
  begin
    if f_fmt(t).ed then return '1'; else return '0'; end if;
  end function;

begin

  s_reg6 <= registers_in(6)(4 downto 0);
  s_tid  <= registers_in(8)(3 downto 0);

  -- ================================================================ stage 1
  p_in : process(clk)
  begin
    if rising_edge(clk) then
      s_in_y <= unsigned(data_in.y);
      s_in_u <= unsigned(data_in.u);
      s_in_v <= unsigned(data_in.v);
    end if;
  end process;

  u_dom_in : entity work.cbv_domain
    port map (clk => clk, i_hs_n => data_in.hsync_n, i_vs_n => data_in.vsync_n, i_avid => data_in.avid,
              i_tid => s_tid, i_reg6 => s_reg6,
              o_hs_n => di.hs, o_vs_n => di.vs, o_avid => di.avid, o_x => di.x, o_sol => di.sol,
              o_psi => di.psi, o_lp => di.lp, o_tid => di.tid, o_reg6 => di.reg6,
              o_vline => di.vline, o_aline => di.aline, o_a0m => di.a0m);

  -- 8-deep sync chain (idiom #18)
  p_sync : process(clk)
  begin
    if rising_edge(clk) then
      s_sync(1) <= data_in.avid & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
      s_sync(2 to C_SYNC_DEPTH) <= s_sync(1 to C_SYNC_DEPTH - 1);
    end if;
  end process;

  -- ================================================================ luma-tag stream
  p_lt : process(clk)
  begin
    if rising_edge(clk) then
      s_ye(1) <= di.hs & di.vs & di.avid & std_logic_vector(s_in_y);
      s_ye(2 to 6) <= s_ye(1 to 5);
      s_fl(2 to 8) <= s_fl(1 to 7);
      s_yl(2 to 5) <= s_yl(1 to 4);
    end if;
  end process;
  s_lt0   <= s_ye(C_YE);                    -- input delayed YE (bitstream constant)
  s_fl(1) <= s_lt0(12 downto 10);
  s_yl(1) <= unsigned(s_lt0(9 downto 0));

  -- ================================================================ encoder
  u_enc : entity work.cbv_encoder
    port map (
      clk => clk,
      i_u => s_in_u, i_v => s_in_v, i_avid => di.avid, i_sol => di.sol,
      i_y_lt => s_yl(5), i_hs_lt => s_fl(5)(2), i_vs_lt => s_fl(5)(1), i_avid_lt => s_fl(5)(0),
      i_tid => s_tid, i_reg6 => s_reg6, i_p5 => registers_in(4), i_burst_en => i_test_burst_gain,
      o_comp => s_comp, o_m => s_m_c, o_mb => s_m_b, o_kh => s_kh, o_psi => s_psi_e,
      o_col => s_col8, o_sol => de_sol, o_tid => de_tid);

  -- encoder-domain line start / constant-set flag, piped to Enhance (10) and the cell (13)
  p_esol : process(clk)
  begin
    if rising_edge(clk) then
      s_esol(7) <= de_sol; s_esol(8 to C_ST_VIN) <= s_esol(7 to C_ST_VIN - 1);
      s_eed(7)  <= f_ed(de_tid); s_eed(8 to C_ST_VIN) <= s_eed(7 to C_ST_VIN - 1);
    end if;
  end process;

  -- ================================================================ Enhance, cell-lite
  u_enh : entity work.cbv_enhance
    port map (clk => clk, i_comp => s_comp, i_sol => s_esol(C_ST_COMP), i_p1 => registers_in(0),
              o_vin => s_vin, o_row => s_row);

  u_cell : entity work.cbv_cell_lite
    port map (clk => clk, i_vin => s_vin, i_sol => s_esol(C_ST_VIN), i_ed => s_eed(C_ST_VIN),
              o_w => s_w, o_hf => s_hf, o_vv => s_vv, o_o => s_o, o_ea => s_ea);

  -- ================================================================ delay lines (§3.1)
  -- D3: {col, hs, vs, avid} stage 8 -> 27 (active-low sync bits stored inverted-at-idle)
  w_d3_i <= std_logic_vector(s_col8) & s_fl(8);
  u_d3 : entity work.cbv_delay
    generic map (G_W => 13, G_D => C_ST_O - 1 - 8, G_EBR => C_DELAY_EBR, G_INIT => x"0006")
    port map (clk => clk, i_d => w_d3_i, o_d => w_d3_o);
  -- D1: comp stage 10 -> O + 4 (separator input select)
  w_d1_i <= std_logic_vector(s_comp);
  u_d1 : entity work.cbv_delay
    generic map (G_W => 14, G_D => C_ST_O + 4 - C_ST_COMP, G_EBR => C_DELAY_EBR, G_INIT => x"0000")
    port map (clk => clk, i_d => w_d1_i, o_d => w_d1_o);
  s_comp_al <= signed(w_d1_o);

  p_o_align : process(clk)
  begin
    if rising_edge(clk) then
      s_col28 <= unsigned(w_d3_o(12 downto 3));
      s_fl28  <= w_d3_o(2 downto 0);
      s_fls(C_ST_O + 1 to C_ST_S - 1) <= s_fls(C_ST_O to C_ST_S - 2);
      s_vs_p(C_ST_O + 1) <= s_fl28(1);
      s_vs_p(C_ST_O + 2 to C_ST_CLAMP - 1) <= s_vs_p(C_ST_O + 1 to C_ST_CLAMP - 2);
      s_o50 <= signed(w_d2_o(15 downto 2));
      s_flc(C_ST_CLAMP to C_ST_SD - 1) <= s_flc(C_ST_CLAMP - 1 to C_ST_SD - 2);
    end if;
  end process;
  s_fls(C_ST_O) <= s_fl28;

  -- D2: {o, hs, avid} stage O -> clamp - 1 (o registered once more to the clamp stage)
  w_d2_i <= std_logic_vector(s_o) & s_fl28(2) & s_fl28(0);
  u_d2 : entity work.cbv_delay
    generic map (G_W => 16, G_D => C_ST_CLAMP - 1 - C_ST_O, G_EBR => C_DELAY_EBR, G_INIT => x"0002")
    port map (clk => clk, i_d => w_d2_i, o_d => w_d2_o);
  s_flc(C_ST_CLAMP - 1) <= w_d2_o(1) & s_vs_p(C_ST_CLAMP - 1) & w_d2_o(0);   -- (hs, vs, avid) of the tag at clamp - 1

  -- ================================================================ leak, separator
  u_dom_l : entity work.cbv_domain
    port map (clk => clk, i_hs_n => w_d3_o(2), i_vs_n => w_d3_o(1), i_avid => w_d3_o(0),
              i_tid => s_tid, i_reg6 => s_reg6,
              o_hs_n => dl.hs, o_vs_n => dl.vs, o_avid => dl.avid, o_x => dl.x, o_sol => dl.sol,
              o_psi => dl.psi, o_lp => dl.lp, o_tid => dl.tid, o_reg6 => dl.reg6,
              o_vline => dl.vline, o_aline => dl.aline, o_a0m => dl.a0m);

  u_leak : entity work.cbv_leak
    port map (clk => clk, i_o => s_o, i_ea => s_ea, i_col => s_col28, i_sol => dl.sol,
              i_p3 => registers_in(2), i_t7 => s_reg6(0), i_comp => s_comp_al,
              o_pic => s_pic, o_leak => s_leak, o_s => s_sep_in);

  u_dom_s : entity work.cbv_domain
    port map (clk => clk, i_hs_n => s_fls(C_ST_S - 1)(2), i_vs_n => s_fls(C_ST_S - 1)(1),
              i_avid => s_fls(C_ST_S - 1)(0), i_tid => s_tid, i_reg6 => s_reg6,
              o_hs_n => dsp.hs, o_vs_n => dsp.vs, o_avid => dsp.avid, o_x => dsp.x, o_sol => dsp.sol,
              o_psi => dsp.psi, o_lp => dsp.lp, o_tid => dsp.tid, o_reg6 => dsp.reg6,
              o_vline => dsp.vline, o_aline => dsp.aline, o_a0m => dsp.a0m);

  u_sep : entity work.cbv_separator
    port map (clk => clk, i_s => s_sep_in, i_x => dsp.x, i_sol => dsp.sol, i_tid => dsp.tid,
              i_p2 => registers_in(1), i_p4 => registers_in(3), i_t7 => s_reg6(0), i_t10 => s_reg6(3),
              o_dh => s_dh_sep, o_f => s_f, o_ti => s_ti, o_bp => s_bp, o_sl => s_sl,
              o_hph => s_hph, o_hper => s_hper, o_ef => s_ef, o_dh8 => s_dh8, o_z => s_z);

  s_dh50 <= i_test_dh when i_test_dh_en = '1' else s_dh_sep;   -- L6 test hook

  -- ================================================================ clamp (stage 50)
  u_dom_c : entity work.cbv_domain
    port map (clk => clk, i_hs_n => s_flc(C_ST_CLAMP - 1)(2), i_vs_n => s_flc(C_ST_CLAMP - 1)(1), i_avid => s_flc(C_ST_CLAMP - 1)(0),
              i_tid => s_tid, i_reg6 => s_reg6,
              o_hs_n => dc.hs, o_vs_n => dc.vs, o_avid => dc.avid, o_x => dc.x, o_sol => dc.sol,
              o_psi => dc.psi, o_lp => dc.lp, o_tid => dc.tid, o_reg6 => dc.reg6,
              o_vline => dc.vline, o_aline => dc.aline, o_a0m => dc.a0m);

  u_clamp : entity work.cbv_clamp
    port map (clk => clk, i_o => s_o50, i_x => dc.x, i_sol => dc.sol, i_tid => dc.tid,
              i_dh => s_dh50, i_pic => dc.reg6(4), i_p6 => registers_in(5),
              o_sd => s_sd, o_bpk => s_bpk, o_bpk_ld => s_bpk_ld, o_ca => s_ca);

  -- dh_l of tag n to the gate (stage SD) and to the write stage (D4)
  p_dh : process(clk)
  begin
    if rising_edge(clk) then
      s_dhp(C_ST_CLAMP + 1) <= s_dh50; s_dhp(C_ST_CLAMP + 2 to C_ST_SD) <= s_dhp(C_ST_CLAMP + 1 to C_ST_SD - 1);
      s_dh_wr <= signed(w_d4_o(14 downto 3));
    end if;
  end process;
  -- D4: {dh_l, hs, vs, avid} of the clamp-stage tag -> write domain input (UV8 - 1)
  w_d4_i <= std_logic_vector(s_dh50) & dc.hs & dc.vs & dc.avid;
  u_d4 : entity work.cbv_delay
    generic map (G_W => 15, G_D => C_ST_UV8 - 1 - C_ST_CLAMP, G_EBR => C_DELAY_EBR, G_INIT => x"0006")
    port map (clk => clk, i_d => w_d4_i, o_d => w_d4_o);

  -- ================================================================ Y/C (stage 53)
  u_dom_yc : entity work.cbv_domain
    port map (clk => clk, i_hs_n => s_flc(C_ST_SD - 1)(2), i_vs_n => s_flc(C_ST_SD - 1)(1),
              i_avid => s_flc(C_ST_SD - 1)(0), i_tid => s_tid, i_reg6 => s_reg6,
              o_hs_n => dy.hs, o_vs_n => dy.vs, o_avid => dy.avid, o_x => dy.x, o_sol => dy.sol,
              o_psi => dy.psi, o_lp => dy.lp, o_tid => dy.tid, o_reg6 => dy.reg6,
              o_vline => dy.vline, o_aline => dy.aline, o_a0m => dy.a0m);

  u_yc : entity work.cbv_ycsplit
    port map (clk => clk, i_sd => s_sd, i_avid => dy.avid, i_sol => dy.sol, i_aline => dy.aline,
              i_tid => dy.tid, i_t8 => dy.reg6(1), i_col_on => s_col_on,
              o_y => s_y, o_c => s_cs, o_comb_en => s_comb_en, o_old => s_old, o_col_on => s_col_yc);

  -- ================================================================ slider, demod, colour
  u_sl : entity work.cbv_slider
    port map (clk => clk, i_sol_in => di.sol, i_tid_in => di.tid, i_s12 => registers_in(7),
              i_sol_yc => dy.sol, o_boff => s_boff, o_dm => s_dm, o_da => s_da);

  u_demod : entity work.cbv_demod
    port map (clk => clk, i_x => dy.x, i_a0m => dy.a0m, i_psi => dy.psi, i_sol => dy.sol,
              i_c => s_cs, i_comb_en => s_comb_en, i_dm => s_dm,
              o_md => s_md, o_du => s_du, o_dv => s_dv, o_strobe => s_stb4, o_i4u => s_i4u, o_i4v => s_i4v,
              o_commit => s_cmt, o_pin_u => s_pin_u, o_pin_v => s_pin_v,
              o_u4 => s_u4, o_v4 => s_v4, o_valid => s_u4_ok);

  p_solp : process(clk)
  begin
    if rising_edge(clk) then
      s_solp(C_ST_SD + 1) <= dy.sol;
      s_solp(C_ST_SD + 2 to C_ST_U4) <= s_solp(C_ST_SD + 1 to C_ST_U4 - 1);
    end if;
  end process;

  u_col : entity work.cbv_colour
    generic map (G_CODEC_ONLY => G_CODEC_ONLY)
    port map (clk => clk, i_x => dy.x, i_sol => dy.sol, i_vline => dy.vline, i_avid => dy.avid,
              i_lp => dy.lp, i_tid => dy.tid, i_t9 => dy.reg6(2), i_dh => s_dhp(C_ST_SD), i_boff => s_boff,
              i_du => s_du, i_dv => s_dv, i_u4 => s_u4, i_v4 => s_v4, i_valid => s_u4_ok,
              i_sol4 => s_solp(C_ST_U4), i_t9r => s_reg6(2),
              o_col_on => s_col_on, o_u8 => s_u8, o_v8 => s_v8, o_valid => s_uv_ok,
              o_t5_stb => t5_stb, o_bu => t5_bu, o_bv => t5_bv, o_mag => t5_mag, o_am => t5_am,
              o_accq => t5_accq, o_gu9 => t5_gu9, o_gv9 => t5_gv9, o_codes => t5_codes,
              o_kcnt => t5_kcnt, o_sw_tv => t5_sw_tv, o_sw_pic => t5_sw_pic, o_idc => t5_idc);

  -- ================================================================ write domain + output
  u_dom_wr : entity work.cbv_domain
    port map (clk => clk, i_hs_n => w_d4_o(2), i_vs_n => w_d4_o(1), i_avid => w_d4_o(0),
              i_tid => s_tid, i_reg6 => s_reg6,
              o_hs_n => dw.hs, o_vs_n => dw.vs, o_avid => dw.avid, o_x => dw.x, o_sol => dw.sol,
              o_psi => dw.psi, o_lp => dw.lp, o_tid => dw.tid, o_reg6 => dw.reg6,
              o_vline => dw.vline, o_aline => dw.aline, o_a0m => dw.a0m);

  u_out : entity work.cbv_output
    generic map (G_CODEC_ONLY => G_CODEC_ONLY, G_LOOPBACK => G_LOOPBACK)
    port map (
      clk => clk, i_y => s_y, i_u8 => s_u8, i_v8 => s_v8, i_uv_ok => s_uv_ok,
      i_x => dw.x, i_a0m => dw.a0m, i_sol => dw.sol, i_tid => dw.tid, i_dh => s_dh_wr,
      i_raw_y => s_in_y, i_raw_u => s_in_u, i_raw_v => s_in_v,
      o_we_y => s_we_y, o_wa_y => s_wa_y, o_wd_y => s_wd_y,
      o_we_uv => s_we_uv, o_wa_uv => s_wa_uv, o_q => s_q, o_wd_uv => s_wd_uv, o_b => s_b);

  -- ================================================================ Tame buffer + read side
  u_tame : entity work.cbv_tame_buffer
    port map (
      clk => clk,
      i_we_y => s_we_y, i_wa_y => s_wa_y, i_wd_y => s_wd_y,
      i_we_uv => s_we_uv, i_wa_uv => s_wa_uv, i_wd_uv => s_wd_uv, i_b => s_b,
      i_x => di.x, i_sol => di.sol, i_avid_n => data_in.avid, i_tid => di.tid,
      i_avid7 => s_sync(C_SYNC_DEPTH - 1)(3),
      o_y => s_out_y, o_u => s_out_u, o_v => s_out_v);

  data_out.y       <= std_logic_vector(s_out_y);
  data_out.u       <= std_logic_vector(s_out_u);
  data_out.v       <= std_logic_vector(s_out_v);
  data_out.avid    <= s_sync(C_SYNC_DEPTH)(3);
  data_out.hsync_n <= s_sync(C_SYNC_DEPTH)(2);
  data_out.vsync_n <= s_sync(C_SYNC_DEPTH)(1);
  data_out.field_n <= s_sync(C_SYNC_DEPTH)(0);

  -- ================================================================ taps
  o_taps <= (comp => s_comp, m_c => s_m_c, m_b => s_m_b, kh => s_kh, psi => s_psi_e,
             v_in => s_vin, row => s_row, w => s_w, hf => s_hf, vv => s_vv, o => s_o,
             ea => s_ea, pic => s_pic, col => s_col28, leak => s_leak,
             s => s_sep_in, f => s_f, ti => s_ti, bp => s_bp, sl => s_sl, hphase => s_hph,
             hper => s_hper, ef => s_ef, dh8 => s_dh8, z => s_z, dh_l => s_dh50,
             sd => s_sd, ca => s_ca, bpk => s_bpk, bpk_ld => s_bpk_ld,
             y => s_y, cs => s_cs, comb_en => s_comb_en, cr_old => s_old,
             md => s_md, du => s_du, dv => s_dv, strobe => s_stb4, i4u => s_i4u, i4v => s_i4v,
             commit => s_cmt, pin_u => s_pin_u, pin_v => s_pin_v, u4 => s_u4, v4 => s_v4,
             u4_valid => s_u4_ok,
             t5_stb => t5_stb, bu => t5_bu, bv => t5_bv, mag => t5_mag, sw_tv => t5_sw_tv,
             sw_pic => t5_sw_pic, idc => t5_idc, am => t5_am, kcnt => t5_kcnt, col_on => s_col_on,
             accq => t5_accq, gu9 => t5_gu9, gv9 => t5_gv9, codes => t5_codes, da => s_da, dm_l => s_dm,
             we_y => s_we_y, wa_y => s_wa_y, yw => s_wd_y, we_uv => s_we_uv, wa_uv => s_wa_uv,
             q => s_q, u8 => unsigned(s_wd_uv(15 downto 8)), v8 => unsigned(s_wd_uv(7 downto 0)),
             b => s_b, a0_m => dw.a0m,
             x_o => dl.x, x_s => dsp.x, x_cl => dc.x, x_yc => dy.x, x_wr => dw.x);

end architecture rtl;
