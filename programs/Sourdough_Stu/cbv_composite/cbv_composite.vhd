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
--   PAL carrier, burst, virtual sync, stepped Hue Bend), passed through a
--   behavioural model of the Syntonie CBV001 (2N7000 Enhance ROM + AC-coupled
--   shelf/gain cell), then decoded by a virtual TV: leak mix -> sync separator
--   (Sep Filter) -> AFC -> Tame-limited horizontal displacement dh_l, clamp
--   (position / gain), comb Y/C split, demod + /4, and colour decode (iii)
--   with ACC, killer and PAL-S, into a one-line Tame buffer read out on the
--   true raster one line late. Spec: plans/cbv_composite/spec.md rev 4.1 and
--   spec-stage2.md (workspace).
--
--   STATUS: Stage 2 (assembly), not yet verified. Variants (colour iii/iv,
--   cuts G1-G6, delay-line EBR, Enhance interpolation) are constants in
--   cbv_00_variant_pkg.vhd; tuned tables in cbv_01_tables_pkg.vhd.
--
-- Pipeline / latency:
--   Boundary (this file's C_PROCESSING_DELAY_CLKS): 8 clocks, spec §1.1 --
--     1 input register | 2 read address | 3 EBR read | 4 EBR output register |
--     5 UV interpolation 1 | 6 UV interpolation 2 | 7 R1 clamp |
--     8 AVID gate + output register.
--   Sync/AVID/field: one 8-deep chain, data_out.avid = tap 8.
--   Write side L_w = 75 clocks (cbv_composite_core.vhd stage table); it does
--   not enter the boundary number (rolling base, tag rule).
--
-- Files (one entity per block):
--   cbv_00_variant_pkg / cbv_01_tables_pkg / cbv_02_pkg / cbv_03_enh_rom_pkg (generated)
--   cbv_domain, cbv_delay (EBR delay lines), cbv_op2s, cbv_op2l (primitives)
--   cbv_encoder, cbv_enhance, cbv_cell_lite, cbv_leak, cbv_separator,
--   cbv_clamp, cbv_ycsplit, cbv_slider, cbv_demod, cbv_colour, cbv_output,
--   cbv_tame_buffer, cbv_composite_core (wiring, taps, test ports)
--
-- EBR map (physical): Enhance ROM 10, comb RAM 5 + 2, Tame Y 5, Tame UV 2,
--   delay lines 4 = 28 program EBR (29 on dual configs incl. the core's).
--
-- Register map:
--   Pot 1-6 (registers_in(0..5)) : Enhance, Tame, Leak, Sep Filter, Hue Bend, Clamp Pos
--   Tog 7-11 (registers_in(6)(4:0)): Sync, Encode, Decode, AFC, Clamp Gain
--   Slider (registers_in(7))     : Colour
--   registers_in(8)(3:0)         : timing ID
--   Never read: registers_in(8)(9:4), registers_in(9..11) (core-only).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.cbv_variant_pkg.all;

architecture cbv_composite of program_top is

  -- Full program_top boundary latency (spec §1.1). Bare literal: the SDK
  -- packer reads only an integer literal. Mirrored in cbv_composite.toml
  -- processing_delay_clks; edit both together.
  constant C_PROCESSING_DELAY_CLKS : integer := 8;

begin

  u_core : entity work.cbv_composite_core
    generic map (
      G_CODEC_ONLY => C_CODEC_ONLY,
      G_LOOPBACK   => C_LOOPBACK
    )
    port map (
      clk               => clk,
      registers_in      => registers_in,
      data_in           => data_in,
      data_out          => data_out,
      i_test_burst_gain => '1',
      i_test_dh_en      => '0',
      i_test_dh         => (others => '0'),
      o_taps            => open
    );

end architecture cbv_composite;
