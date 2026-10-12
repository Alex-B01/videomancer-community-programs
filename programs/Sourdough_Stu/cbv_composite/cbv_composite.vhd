-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_composite.vhd - CBV Composite Program for Videomancer (program_top)
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
-- Program Name:
--   CBV Composite
--
-- Author:
--   Sourdough Stu
--
-- Credits:
--   The circuit facts this program models come from the Syntonie CBV001
--   Rev C schematic (CC BY-SA 4.0) and Phil Baljeu's Archer trace. The
--   program models the circuit's behaviour; it copies no schematic artwork.
--
-- Overview:
--   A virtual composite signal path: Y/U/V is encoded to composite (NTSC or
--   PAL carrier, burst, virtual sync), passed through a model of the
--   Syntonie CBV001 composite bender, then decoded by a virtual TV (clamp,
--   Y/C split, demodulator, burst-locked colour engine) into a one-line
--   "Tame" buffer that is read out on the true raster one line late.
--
--   STATUS: Stage 1 (codec round trip), spec rev 3 (rulings Q1-Q17 applied). The CBV001 cell is an identity stub
--   (cbv_cell_stub.vhd), the sync side (separator, AFC, Tame) is absent
--   (dh_l = 0), Hue Bend and the colour slider are off. Not a usable
--   instrument yet. Stage plan: plans/cbv_composite/spec.md (rev 2) and
--   spec-stage1.md in the workspace.
--
-- Pipeline / latency:
--   Boundary (this file's C_PROCESSING_DELAY_CLKS): 8 clocks, spec §1.1 --
--     1 input register | 2 read address | 3 EBR read | 4 EBR output register |
--     5 UV interpolation 1 | 6 UV interpolation 2 | 7 R1 clamp |
--     8 AVID gate + output register.
--   Sync/AVID/field: one 8-deep chain, data_out.avid = tap 8.
--   Content lands one line late (Tame buffer, rolling base, spec §1.3); the
--   write side (L_w = 37 clocks, cbv_composite_core.vhd stage table) does not
--   enter the boundary number.
--
-- Files (one entity per block of spec-stage1.md §1):
--   cbv_0_pkg.vhd          format table, ROM tables, helpers
--   cbv_domain.vhd         raster domain (x, psi, LP, vline, R-LATCH)    block 1
--   cbv_encoder.vhd        composite encoder                            block 2
--   cbv_cell_stub.vhd      CBV001 cell, Stage-1 identity stub            block 3
--   cbv_clamp.vhd          clamp (porch)                                 block 4
--   cbv_ycsplit.vhd        notch + comb RAM                              block 5
--   cbv_demod.vhd          demod, LPF, /4                                block 6
--   cbv_m1_rotate.vhd      M1 rotation + sine ROM                        block 7
--   cbv_m2_gain.vhd        M2 gain, PAL-S, killer gate                   block 8
--   cbv_colour_engine.vhd  burst gate + per-line colour engine           block 9
--   cbv_output.vhd         gain/knee, rolling-base write side            block 11
--   cbv_tame_buffer.vhd    Tame buffer RAMs + read side                  block 12
--   cbv_composite_core.vhd wiring, sync chain, test generics, taps T1-T7
--   (block 10, separator / AFC / Tame, is absent in Stage 1)
--
-- EBR map (physical): comb RAM 5 + 2, Tame Y 5, Tame UV 2, sine ROM 1 =
--   15 program EBR (16 on dual configs with the core's dual_sync_delay).
--
-- LC: see cbv_composite_core.vhd (measured per block). Two inferred
--   multiplier blocks (M1 14x12 as two 14x6 halves, M2 14x7 digit-serial,
--   spec rev 3 Q14); no multiplier_s.
--
-- Register map (full control map in cbv_composite.toml; Stage 1 reads only):
--   Tog 8  (registers_in(6)(1)) : Encode  NTSC / PAL
--   Tog 9  (registers_in(6)(2)) : Decode  NTSC / PAL-S
--   registers_in(8)(3:0)        : timing ID (class, P, every per-format constant)
--   Not read in Stage 1: Pots 1-6 (registers_in(0..5)), Slider (registers_in(7)),
--   Tog 7/10/11 (registers_in(6)(0,3,4)), registers_in(8)(9:4).
--   Never read: registers_in(9..11) (core-only).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture cbv_composite of program_top is

  -- Full program_top boundary latency (spec §1.1). Bare literal: the SDK
  -- packer reads only an integer literal. Mirrored in cbv_composite.toml
  -- processing_delay_clks; edit both together.
  constant C_PROCESSING_DELAY_CLKS : integer := 8;

begin

  u_core : entity work.cbv_composite_core
    generic map (
      G_CODEC_ONLY => false,
      G_LOOPBACK   => false
    )
    port map (
      clk               => clk,
      registers_in      => registers_in,
      data_in           => data_in,
      data_out          => data_out,
      i_test_burst_gain => '1',
      o_taps            => open
    );

end architecture cbv_composite;
