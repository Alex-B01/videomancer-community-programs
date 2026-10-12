-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_composite_core.vhd - CBV Composite program core (Stage 1: codec round trip)
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
-- program_top) instantiates it with the shipping values. Testbenches
-- instantiate this entity directly and read the T1-T6 taps from the o_taps
-- record port (spec-stage1.md §6; T7 = data_out). The t*_ tap signals below
-- carry the same values (external names do not work in the oss-cad GHDL build).
--
-- Generics / test port (spec-stage1.md §1):
--   G_CODEC_ONLY      true: identity output gain, no knee, codec chroma gains
--   G_LOOPBACK        true: Tame-buffer write port takes the raw input
--   i_test_burst_gain G_TEST_BURST_GAIN as a port ('1' normal, '0' burst
--                     forced to 0): H7 switches it mid-run, which a generic
--                     cannot do. The wrapper ties it to '1'.
--
-- Sample tags. Stage s = clock index after the input register (stage 1).
--   chroma stream  : stage 1 .. 6 (Uf, Vf)
--   luma-tag stream: the input delayed YE (3 / 6) more; the "newest sample"
--                    at every stage s >= 6 is the input-register sample from
--                    s - 1 + YE clocks earlier (encoder, decoder and write
--                    domains are this stream's flags, delayed, with local x).
--
-- Stage table (write side; boundary latency is the read side, see below):
--   1  input register; input domain (x, sol, tid); 8-deep sync chain tap 1
--   2-6   encoder chroma (Us/Vs, 2 OP2)               cbv_encoder
--   6-10  encoder domain, modulator, comp             cbv_encoder       T1 @10
--   10    cell stub o = comp (latency 0)              cbv_cell_stub
--   10-13 decoder domain (x_d), clamp, sd             cbv_clamp         T2 @13
--   13-16 Y/C split (notch, comb RAM)                 cbv_ycsplit       T3 @16
--   16-24 demod, LPF, /4                              cbv_demod         T4 @24
--   13-~95 burst gate, serial colour engine           cbv_colour_engine T5 per line
--   24-31 M1 rotation (uu 29, vq 31)                 cbv_m1_rotate
--   29-36 M2 gain, PAL-S, killer, U8/V8               cbv_m2_gain
--   16-36 luma gain/knee + delay; 36 write domain     cbv_output
--   37    write-port registers (L_w = 37)             cbv_output        T6 @37
-- Boundary (data_in -> data_out), spec §1.1:
--   1 input register | 2 read address | 3 EBR read | 4 EBR output register |
--   5 UV interp 1 | 6 UV interp 2 | 7 R1 clamp | 8 AVID gate + output reg
--   = 8 = C_PROCESSING_DELAY_CLKS. One 8-deep chain carries AVID, HSYNC_n,
--   VSYNC_n, FIELD_n; data_out.avid is its tap 8.                      T7
--
-- EBR map (physical; all memories power-of-two deep):
--   comb RAM bank A 2048x10 ........ 5   (cbv_ycsplit)
--   comb RAM bank B  512x10 ........ 2   (cbv_ycsplit)
--   Tame buffer Y  2048x10 ......... 5   (cbv_tame_buffer)
--   Tame buffer UV  512x16 ......... 2   (cbv_tame_buffer)
--   sine quarter-wave 256x12 ....... 1   (cbv_m1_rotate)
--   program total ................. 15   (16 on dual configs incl. the core's)
-- Multipliers (inferred `*`, none in a 1-clock recursion; no multiplier_s):
--   M1 14x12 as two registered 14x6 halves (4 slots), M2 one 14x7 digit-serial
--   (4 half-slots); plus small constant/LUT-size
--   products (knee dk*f 5x4, read-side d*f 10x3).
--
-- LC: spec-stage1.md §4 paper ~4,100 / ~3,600 packed. Measured (yosys
--   synth_ice40 -noflatten, program only, sd_dual): 5af85f4 8,880 LUT4 /
--   3,678 FF; rev 3 8,063 / 3,557; area pass 6,011 / 3,409 (bit-identical taps).
--
-- Register map (Stage 1 reads only):
--   registers_in(6)(1) T8 Encode (0 NTSC, 1 PAL)
--   registers_in(6)(2) T9 Decode (0 NTSC, 1 PAL-S)
--   registers_in(8)(3:0) timing ID
--   Not read in Stage 1: registers_in(0..5), (7), (6)(0,3,4) (T7 = Clean,
--   T10 = Stepped, T11 = Porch), (8)(9:4). Never read: registers_in(9..11).
--   All per-line values latch at x = 0 of the consuming domain (R-LATCH).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.cbv_pkg.all;

entity cbv_composite_core is
  generic (
    G_CODEC_ONLY : boolean := false;
    G_LOOPBACK   : boolean := false
  );
  port (
    clk               : in  std_logic;
    registers_in      : in  t_spi_ram;
    data_in           : in  t_video_stream_yuv444_30b;
    data_out          : out t_video_stream_yuv444_30b;
    i_test_burst_gain : in  std_logic := '1';
    o_taps            : out t_cbv_taps      -- Gate-2 taps (open in program_top)
  );
end entity cbv_composite_core;

architecture rtl of cbv_composite_core is

  constant C_SYNC_DEPTH : integer := 8;   -- = C_PROCESSING_DELAY_CLKS (cbv_composite.vhd)

  signal s_reg6 : std_logic_vector(4 downto 0);
  signal s_tid  : t_tid;

  -- ---------------------------------------------------------------- stage 1
  signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
  signal di_hs, di_vs, di_avid, di_sol, di_lp, di_vline : std_logic;
  signal di_x   : unsigned(11 downto 0);
  signal di_psi : unsigned(2 downto 0);
  signal di_tid : t_tid;
  signal di_reg6 : std_logic_vector(4 downto 0);
  signal di_aline : unsigned(1 downto 0);
  signal s_p8_in : boolean;

  -- 8-deep sync chain (AVID, HSYNC_n, VSYNC_n, FIELD_n)
  type t_sync is array (1 to C_SYNC_DEPTH) of std_logic_vector(3 downto 0);
  signal s_sync : t_sync := (others => "0111");

  -- luma-tag stream: YE tap, then flag chain fl(s) (hs, vs, avid) and Y chain
  type t_ye is array (1 to 6) of std_logic_vector(12 downto 0);
  signal s_ye : t_ye := (others => (12 => '1', 11 => '1', others => '0'));
  signal s_lt0 : std_logic_vector(12 downto 0);
  type t_fl is array (1 to C_ST_UV8) of std_logic_vector(2 downto 0);
  signal s_fl : t_fl := (others => "110");
  type t_yl is array (1 to 5) of unsigned(9 downto 0);
  signal s_yl : t_yl := (others => (others => '0'));

  -- encoder / cell
  signal s_comp : signed(13 downto 0);
  signal s_m_enc, s_psi_enc : unsigned(2 downto 0);
  signal s_o    : signed(13 downto 0);
  signal s_r2, s_r3 : signed(11 downto 0);
  signal s_ed   : signed(13 downto 0);

  -- decoder domain (clamp input, stage 10)
  signal dd_hs, dd_vs, dd_avid, dd_sol, dd_lp, dd_vline : std_logic;
  signal dd_x   : unsigned(11 downto 0);
  signal dd_psi : unsigned(2 downto 0);
  signal dd_tid : t_tid;
  signal dd_reg6 : std_logic_vector(4 downto 0);
  signal dd_aline : unsigned(1 downto 0);
  signal s_sd   : signed(15 downto 0);
  signal s_bpk  : signed(15 downto 0);
  signal s_bpk_ld : std_logic;

  -- Y/C domain (stage 13)
  signal dy_hs, dy_vs, dy_avid, dy_sol, dy_lp, dy_vline : std_logic;
  signal dy_x   : unsigned(11 downto 0);
  signal dy_psi : unsigned(2 downto 0);
  signal dy_tid : t_tid;
  signal dy_reg6 : std_logic_vector(4 downto 0);
  signal dy_aline : unsigned(1 downto 0);
  signal s_y16 : signed(15 downto 0);
  signal s_c16 : signed(12 downto 0);   -- cS
  signal s_comb_en : std_logic;
  signal s_old  : signed(9 downto 0);
  signal s_col_yc : std_logic;

  -- line-start pipe from the Y/C domain to M1 (24) and M2 (30)
  type t_solp is array (14 to C_ST_M1) of std_logic;
  signal s_solp : t_solp := (others => '0');

  -- demod
  signal s_du, s_dv : signed(14 downto 0);
  signal s_u4, s_v4 : signed(14 downto 0);
  signal s_u4_ok : std_logic;

  -- colour engine
  signal s_apc_ph : unsigned(17 downto 0);
  signal s_col_on, s_sw_pic : std_logic;
  signal s_gu_line, s_gv_line : unsigned(11 downto 0);
  signal s_accq_line : unsigned(14 downto 0);
  signal s_cos, s_sin : signed(11 downto 0);

  -- M1 / M2
  signal s_uu, s_vq : signed(13 downto 0);
  signal s_m1_ok, s_m1_uok : std_logic;
  signal s_u8, s_v8 : unsigned(7 downto 0);
  signal s_uv_ok : std_logic;

  -- write domain (stage 36)
  signal dw_hs, dw_vs, dw_avid, dw_sol, dw_lp, dw_vline : std_logic;
  signal dw_x   : unsigned(11 downto 0);
  signal dw_psi : unsigned(2 downto 0);
  signal dw_tid : t_tid;
  signal dw_reg6 : std_logic_vector(4 downto 0);
  signal dw_aline : unsigned(1 downto 0);

  -- write port, read side
  signal s_we_y, s_we_uv : std_logic;
  signal s_wa_y  : unsigned(10 downto 0);
  signal s_wd_y  : unsigned(9 downto 0);
  signal s_wa_uv : unsigned(8 downto 0);
  signal s_q     : signed(11 downto 0);
  signal s_wd_uv : std_logic_vector(15 downto 0);
  signal s_b     : unsigned(10 downto 0);
  signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0);

  constant C_DH0 : signed(11 downto 0) := (others => '0');   -- dh_l = 0 (Stage 1)

  -- ================================================================ taps (spec-stage1 §6)
  -- T1 (stage 10; tag: decoder domain dd_* at stage 10)
  signal t1_comp : signed(13 downto 0);
  signal t1_m    : unsigned(2 downto 0);
  signal t1_psi  : unsigned(2 downto 0);
  -- T2 (stage 13)
  signal t2_sd     : signed(15 downto 0);
  signal t2_bpk    : signed(15 downto 0);
  signal t2_bpk_ld : std_logic;
  -- T3 (stage 16)
  signal t3_y       : signed(15 downto 0);
  signal t3_cs      : signed(12 downto 0);
  signal t3_comb_en : std_logic;
  signal t3_cr_old  : signed(9 downto 0);
  -- T4 (stage 24, valid on t4_valid)
  signal t4_u4    : signed(14 downto 0);
  signal t4_v4    : signed(14 downto 0);
  signal t4_valid : std_logic;
  -- T5 (per line, valid on t5_stb)
  signal t5_stb     : std_logic;
  signal t5_bsum_u  : signed(19 downto 0);
  signal t5_bsum_v  : signed(19 downto 0);
  signal t5_mag     : unsigned(23 downto 0);
  signal t5_beta    : unsigned(17 downto 0);
  signal t5_ee      : signed(17 downto 0);
  signal t5_apc_ph  : unsigned(17 downto 0);
  signal t5_apc_int : signed(15 downto 0);
  signal t5_am      : unsigned(36 downto 0);
  signal t5_accq    : unsigned(14 downto 0);
  signal t5_gu9     : unsigned(11 downto 0);
  signal t5_gv9     : unsigned(11 downto 0);
  signal t5_bur     : signed(27 downto 0);    -- bu' (PAL-S lines)
  signal t5_bvr     : signed(27 downto 0);    -- bv'
  signal t5_sw_pic  : std_logic;
  signal t5_col_on  : std_logic;
  signal t5_kcnt    : unsigned(10 downto 0);
  signal t5_sw_tv   : std_logic;
  signal t5_idc     : unsigned(2 downto 0);
  -- T6 (stage 37 write-port registers)
  signal t6_we_y   : std_logic;
  signal t6_wa_y   : unsigned(10 downto 0);
  signal t6_yw     : unsigned(9 downto 0);
  signal t6_we_uv  : std_logic;
  signal t6_wa_uv  : unsigned(8 downto 0);
  signal t6_q      : signed(11 downto 0);
  signal t6_u8     : unsigned(7 downto 0);
  signal t6_v8     : unsigned(7 downto 0);
  signal t6_b      : unsigned(10 downto 0);
  -- domain x tags for the testbench
  signal t_x_dec   : unsigned(11 downto 0);   -- stage 10
  signal t_x_yc    : unsigned(11 downto 0);   -- stage 13
  signal t_x_wr    : unsigned(11 downto 0);   -- stage 36

begin

  s_reg6 <= registers_in(6)(4 downto 0);
  s_tid  <= registers_in(8)(3 downto 0);

  -- ================================================================ stage 1: input register, input domain
  p_in : process(clk)
  begin
    if rising_edge(clk) then
      s_in_y <= unsigned(data_in.y);
      s_in_u <= unsigned(data_in.u);
      s_in_v <= unsigned(data_in.v);
    end if;
  end process;

  u_dom_in : entity work.cbv_domain
    port map (
      clk => clk, i_hs_n => data_in.hsync_n, i_vs_n => data_in.vsync_n, i_avid => data_in.avid,
      i_tid => s_tid, i_reg6 => s_reg6,
      o_hs_n => di_hs, o_vs_n => di_vs, o_avid => di_avid, o_x => di_x, o_sol => di_sol,
      o_psi => di_psi, o_lp => di_lp, o_tid => di_tid, o_reg6 => di_reg6,
      o_vline => di_vline, o_aline => di_aline);

  s_p8_in <= f_fmt(di_tid).p8;

  -- ================================================================ 8-deep sync chain (idiom #18)
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
      s_ye(1) <= di_hs & di_vs & di_avid & std_logic_vector(s_in_y);
      s_ye(2 to 6) <= s_ye(1 to 5);
      s_fl(2 to C_ST_UV8) <= s_fl(1 to C_ST_UV8 - 1);
      s_yl(2 to 5) <= s_yl(1 to 4);
    end if;
  end process;

  s_lt0   <= s_ye(6) when s_p8_in else s_ye(3);       -- input delayed YE
  s_fl(1) <= s_lt0(12 downto 10);                      -- hs, vs, avid at luma-tag stage 1
  s_yl(1) <= unsigned(s_lt0(9 downto 0));

  -- ================================================================ encoder (block 2)
  u_enc : entity work.cbv_encoder
    port map (
      clk => clk,
      i_u => s_in_u, i_v => s_in_v, i_avid => di_avid, i_sol => di_sol, i_p8 => s_p8_in,
      i_y_lt => s_yl(5), i_hs_lt => s_fl(5)(2), i_vs_lt => s_fl(5)(1), i_avid_lt => s_fl(5)(0),
      i_tid => s_tid, i_reg6 => s_reg6, i_burst_en => i_test_burst_gain,
      o_comp => s_comp, o_m => s_m_enc, o_psi => s_psi_enc);

  -- ================================================================ cell (block 3, Stage-1 stub)
  u_cell : entity work.cbv_cell_stub
    port map (clk => clk, i_comp => s_comp, o_o => s_o, o_r2 => s_r2, o_r3 => s_r3, o_ed => s_ed);

  -- ================================================================ decoder domain + clamp (block 4)
  u_dom_dec : entity work.cbv_domain
    port map (
      clk => clk, i_hs_n => s_fl(C_ST_CLAMP - 1)(2), i_vs_n => s_fl(C_ST_CLAMP - 1)(1),
      i_avid => s_fl(C_ST_CLAMP - 1)(0), i_tid => s_tid, i_reg6 => s_reg6,
      o_hs_n => dd_hs, o_vs_n => dd_vs, o_avid => dd_avid, o_x => dd_x, o_sol => dd_sol,
      o_psi => dd_psi, o_lp => dd_lp, o_tid => dd_tid, o_reg6 => dd_reg6,
      o_vline => dd_vline, o_aline => dd_aline);

  u_clamp : entity work.cbv_clamp
    port map (
      clk => clk, i_o => s_o, i_x => dd_x, i_sol => dd_sol, i_tid => dd_tid,
      i_dh => C_DH0, i_pic => '0',
      o_sd => s_sd, o_bpk => s_bpk, o_bpk_ld => s_bpk_ld);

  -- ================================================================ Y/C domain + split (block 5)
  u_dom_yc : entity work.cbv_domain
    port map (
      clk => clk, i_hs_n => s_fl(C_ST_SD - 1)(2), i_vs_n => s_fl(C_ST_SD - 1)(1),
      i_avid => s_fl(C_ST_SD - 1)(0), i_tid => s_tid, i_reg6 => s_reg6,
      o_hs_n => dy_hs, o_vs_n => dy_vs, o_avid => dy_avid, o_x => dy_x, o_sol => dy_sol,
      o_psi => dy_psi, o_lp => dy_lp, o_tid => dy_tid, o_reg6 => dy_reg6,
      o_vline => dy_vline, o_aline => dy_aline);

  u_yc : entity work.cbv_ycsplit
    port map (
      clk => clk, i_sd => s_sd, i_avid => dy_avid, i_sol => dy_sol, i_aline => dy_aline,
      i_tid => dy_tid, i_t8 => dy_reg6(1), i_col_on => s_col_on,
      o_y => s_y16, o_c => s_c16, o_comb_en => s_comb_en, o_old => s_old, o_col_on => s_col_yc);

  -- ================================================================ demod (block 6)
  u_demod : entity work.cbv_demod
    port map (
      clk => clk, i_x => dy_x, i_psi => dy_psi, i_tid => dy_tid,
      i_c => s_c16, i_comb_en => s_comb_en,
      o_du => s_du, o_dv => s_dv, o_u4 => s_u4, o_v4 => s_v4, o_valid => s_u4_ok);

  -- ================================================================ colour engine (block 9)
  u_eng : entity work.cbv_colour_engine
    generic map (G_CODEC_ONLY => G_CODEC_ONLY)
    port map (
      clk => clk, i_x => dy_x, i_sol => dy_sol, i_vline => dy_vline, i_avid => dy_avid,
      i_tid => dy_tid, i_t9 => dy_reg6(2), i_dh => C_DH0,
      i_du => s_du, i_dv => s_dv, i_cos => s_cos, i_sin => s_sin,
      o_apc_ph => s_apc_ph, o_col_on => s_col_on, o_sw_pic => s_sw_pic,
      o_gu_line => s_gu_line, o_gv_line => s_gv_line, o_accq_line => s_accq_line,
      o_t5_stb => t5_stb, o_bsum_u => t5_bsum_u, o_bsum_v => t5_bsum_v, o_mag => t5_mag,
      o_beta => t5_beta, o_ee => t5_ee, o_apc_int => t5_apc_int, o_am => t5_am,
      o_accq => t5_accq, o_gu => t5_gu9, o_gv => t5_gv9, o_kcnt => t5_kcnt,
      o_sw_tv => t5_sw_tv, o_idc => t5_idc, o_bur => t5_bur, o_bvr => t5_bvr,
      o_sw_pic_r => t5_sw_pic);
  t5_apc_ph <= s_apc_ph;
  t5_col_on <= s_col_on;

  -- line-start pipe (Y/C domain x = 0 -> M1 at 24, M2 at 30)
  p_solp : process(clk)
  begin
    if rising_edge(clk) then
      s_solp(14) <= dy_sol;
      s_solp(15 to C_ST_M1) <= s_solp(14 to C_ST_M1 - 1);
    end if;
  end process;

  -- ================================================================ M1 (block 7), M2 (block 8)
  u_m1 : entity work.cbv_m1_rotate
    port map (
      clk => clk, i_u4 => s_u4, i_v4 => s_v4, i_valid => s_u4_ok, i_sol => s_solp(C_ST_U4),
      i_apc_ph => s_apc_ph, o_uu => s_uu, o_vq => s_vq, o_valid => s_m1_ok, o_uvalid => s_m1_uok,
      o_cos => s_cos, o_sin => s_sin);

  u_m2 : entity work.cbv_m2_gain
    generic map (G_CODEC_ONLY => G_CODEC_ONLY)
    port map (
      clk => clk, i_uu => s_uu, i_vq => s_vq, i_uvalid => s_m1_uok, i_valid => s_m1_ok, i_sol => s_solp(C_ST_M1 - 1),
      i_gu => s_gu_line, i_gv => s_gv_line, i_col_on => s_col_on, i_t9 => s_reg6(2),
      i_sw_pic => s_sw_pic, o_u8 => s_u8, o_v8 => s_v8, o_valid => s_uv_ok);

  -- ================================================================ write domain + output (block 11)
  u_dom_wr : entity work.cbv_domain
    port map (
      clk => clk, i_hs_n => s_fl(C_ST_UV8 - 1)(2), i_vs_n => s_fl(C_ST_UV8 - 1)(1),
      i_avid => s_fl(C_ST_UV8 - 1)(0), i_tid => s_tid, i_reg6 => s_reg6,
      o_hs_n => dw_hs, o_vs_n => dw_vs, o_avid => dw_avid, o_x => dw_x, o_sol => dw_sol,
      o_psi => dw_psi, o_lp => dw_lp, o_tid => dw_tid, o_reg6 => dw_reg6,
      o_vline => dw_vline, o_aline => dw_aline);

  u_out : entity work.cbv_output
    generic map (G_CODEC_ONLY => G_CODEC_ONLY, G_LOOPBACK => G_LOOPBACK)
    port map (
      clk => clk, i_y => s_y16, i_u8 => s_u8, i_v8 => s_v8, i_uv_ok => s_uv_ok,
      i_x => dw_x, i_sol => dw_sol, i_tid => dw_tid, i_dh => C_DH0,
      i_raw_y => s_in_y, i_raw_u => s_in_u, i_raw_v => s_in_v,
      o_we_y => s_we_y, o_wa_y => s_wa_y, o_wd_y => s_wd_y,
      o_we_uv => s_we_uv, o_wa_uv => s_wa_uv, o_q => s_q, o_wd_uv => s_wd_uv, o_b => s_b);

  -- ================================================================ Tame buffer + read side (block 12)
  u_tame : entity work.cbv_tame_buffer
    port map (
      clk => clk,
      i_we_y => s_we_y, i_wa_y => s_wa_y, i_wd_y => s_wd_y,
      i_we_uv => s_we_uv, i_wa_uv => s_wa_uv, i_wd_uv => s_wd_uv, i_b => s_b,
      i_x => di_x, i_sol => di_sol, i_tid => di_tid, i_avid7 => s_sync(C_SYNC_DEPTH - 1)(3),
      o_y => s_out_y, o_u => s_out_u, o_v => s_out_v);

  -- ================================================================ output (stage 8)
  data_out.y       <= std_logic_vector(s_out_y);
  data_out.u       <= std_logic_vector(s_out_u);
  data_out.v       <= std_logic_vector(s_out_v);
  data_out.avid    <= s_sync(C_SYNC_DEPTH)(3);
  data_out.hsync_n <= s_sync(C_SYNC_DEPTH)(2);
  data_out.vsync_n <= s_sync(C_SYNC_DEPTH)(1);
  data_out.field_n <= s_sync(C_SYNC_DEPTH)(0);

  -- ================================================================ tap wiring
  t1_comp <= s_comp;  t1_m <= s_m_enc;  t1_psi <= s_psi_enc;
  t2_sd <= s_sd;  t2_bpk <= s_bpk;  t2_bpk_ld <= s_bpk_ld;
  t3_y <= s_y16;  t3_cs <= s_c16;  t3_comb_en <= s_comb_en;  t3_cr_old <= s_old;
  t4_u4 <= s_u4;  t4_v4 <= s_v4;  t4_valid <= s_u4_ok;
  t6_we_y <= s_we_y;  t6_wa_y <= s_wa_y;  t6_yw <= s_wd_y;
  t6_we_uv <= s_we_uv;  t6_wa_uv <= s_wa_uv;  t6_q <= s_q;
  t6_u8 <= unsigned(s_wd_uv(15 downto 8));  t6_v8 <= unsigned(s_wd_uv(7 downto 0));
  t6_b <= s_b;
  t_x_dec <= dd_x;  t_x_yc <= dy_x;  t_x_wr <= dw_x;

  o_taps <= (t1_comp => t1_comp, t1_m => t1_m, t1_psi => t1_psi, x_dec => t_x_dec,
             t2_sd => t2_sd, t2_bpk => t2_bpk, t2_bpk_ld => t2_bpk_ld, x_yc => t_x_yc,
             t3_y => t3_y, t3_cs => t3_cs, t3_comb_en => t3_comb_en, t3_cr_old => t3_cr_old,
             t4_u4 => t4_u4, t4_v4 => t4_v4, t4_valid => t4_valid,
             t5_stb => t5_stb, t5_bsum_u => t5_bsum_u, t5_bsum_v => t5_bsum_v,
             t5_bur => t5_bur, t5_bvr => t5_bvr, t5_mag => t5_mag, t5_beta => t5_beta,
             t5_ee => t5_ee, t5_apc_ph => t5_apc_ph, t5_apc_int => t5_apc_int, t5_am => t5_am,
             t5_accq => t5_accq, t5_gu9 => t5_gu9, t5_gv9 => t5_gv9, t5_col_on => t5_col_on,
             t5_kcnt => t5_kcnt, t5_sw_tv => t5_sw_tv, t5_sw_pic => t5_sw_pic, t5_idc => t5_idc,
             t6_we_y => t6_we_y, t6_wa_y => t6_wa_y, t6_yw => t6_yw, t6_we_uv => t6_we_uv,
             t6_wa_uv => t6_wa_uv, t6_q => t6_q, t6_u8 => t6_u8, t6_v8 => t6_v8, t6_b => t6_b,
             x_wr => t_x_wr);

end architecture rtl;
