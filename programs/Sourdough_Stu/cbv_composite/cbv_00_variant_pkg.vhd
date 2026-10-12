-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_00_variant_pkg.vhd - CBV Composite build variants (spec-stage2.md §2)
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
-- Compile-time variants. The SDK build passes no generics: the smoke driver
-- copies the program to scratch and rewrites these constants per run. The
-- committed tree keeps the defaults (run R1: colour (iii), un-cut, delay EBR).
-- (File name "00": the SDK image tester / alignment runner analyse program
-- files alphabetically + entity toposort and do not track packages.)
--------------------------------------------------------------------------------
package cbv_variant_pkg is

  constant C_COLOUR        : natural := 1;      -- 1 = (iii) minimal colour engine; 0 = (iv) fixed
  constant C_KNEE_CLIP     : boolean := false;  -- G1: output knee -> clip at 1023
  constant C_SD_ONE_SET    : boolean := false;  -- G2: SD bitstream uses the 13.5 MHz constant set for ED too
  constant C_ENC_1POLE     : boolean := false;  -- G3: encoder chroma 1 pole (YE 3 / 2)
  constant C_TAME_HARD     : boolean := false;  -- G4: Tame hard limit instead of tanh
  constant C_COLOUR_LEAK   : boolean := true;   -- G5: false removes the saturated-colour leak term
  constant C_AC_PER_SAMPLE : boolean := false;  -- G6: k_in / k_out as per-sample OP2 instead of OP2L
  constant C_DELAY_EBR     : boolean := true;   -- false: every delay line as flip-flops (measures §3.1)
  constant C_ENH_INTERP    : boolean := false;  -- Enhance: false = nearest point (spec), true = interpolated (+~283 LC)
  -- test only (defaults of the core generics G_CODEC_ONLY / G_LOOPBACK)
  constant C_CODEC_ONLY    : boolean := false;
  constant C_LOOPBACK      : boolean := false;

end package cbv_variant_pkg;
