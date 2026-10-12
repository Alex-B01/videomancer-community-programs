-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_01_tables_pkg.vhd - CBV Composite tunable tables (spec rev 4.1 §5.6, §5.10, §6.2)
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
-- Every value the model agent tunes lives here, so tuned values drop in
-- without RTL changes. GI and KSP are tuned (rev41-tuning.md); values still
-- marked TUNE are placeholders. (The Tame taper is the TOML quart_in curve.)
--   GI[tid]      burst-gate torn-line inhibit |dh_l| >= GI (R4.1-1); tuned
--   KSP[P][bw]   Sep Filter one-pole coefficients as up to 3 shift terms
--                k = sum s_j 2^-a_j (R4.1-4); P = 4 uses the 3-term OP2-3
--   XE[P][bw]    Tame reference offset x_e (R4.1-7); start 7 (P = 8) / 4 (P = 4)
--------------------------------------------------------------------------------
package cbv_tables_pkg is

  type t_int16 is array (0 to 15) of integer;

  -- GI per timing ID (index = tid; 15 = unused). TUNED (evidence/model/rev41-tuning.md §1):
  -- SD/ED 10, 720p 8, 1080 24.
  --                          0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
  constant C_GI : t_int16 := (10, 24, 24, 24, 10,  8,  8, 24, 10, 24, 24, 24, 10, 24,  8, 10);

  -- Sep Filter terms: for step bw (0 Narrow .. 4 Open), term j (0..2):
  -- shift a_j and sign s_j in {-1, 0, +1}; k = sum_j s_j 2^-a_j.
  type t_term is record
    sh : natural range 0 to 15;
    sg : integer range -1 to 1;
  end record;
  type t_terms is array (0 to 2) of t_term;
  type t_ksp is array (0 to 4) of t_terms;

  -- P = 4 (SD/ED bitstream), OP2-3. TUNED (rev41-tuning.md §3; replaces the
  -- spec rev 4.1 set, which failed the NTSC acceptance; orchestrator ruling)
  constant C_KSP4 : t_ksp := (
    0 => ((1, 1), (3, 1), (0, 0)),     -- Narrow 5/8   = 2^-1 + 2^-3
    1 => ((1, 1), (2, 1), (5, -1)),    -- Low    23/32 = 2^-1 + 2^-2 - 2^-5
    2 => ((1, 1), (2, 1), (0, 0)),     -- Mid    3/4   = 2^-1 + 2^-2
    3 => ((0, 1), (3, -1), (5, -1)),   -- Wide   27/32 = 2^0 - 2^-3 - 2^-5
    4 => ((0, 1), (4, -1), (0, 0)));   -- Open   15/16 = 2^0 - 2^-4
  -- P = 8 (HD bitstream), 2-term. Accepted as specified (rev41-tuning.md §3)
  constant C_KSP8 : t_ksp := (
    0 => ((2, 1), (5, -1), (0, 0)),    -- Narrow 7/32  = 2^-2 - 2^-5
    1 => ((2, 1), (6, 1), (0, 0)),     -- Low    17/64 = 2^-2 + 2^-6
    2 => ((2, 1), (4, 1), (0, 0)),     -- Mid    5/16  = 2^-2 + 2^-4
    3 => ((1, 1), (4, -1), (0, 0)),    -- Wide   7/16  = 2^-1 - 2^-4
    4 => ((1, 1), (2, 1), (0, 0)));    -- Open   3/4   = 2^-1 + 2^-2

  -- x_e per Sep Filter step. TUNE (calibration, spec-stage2 §6.1)
  type t_xe is array (0 to 4) of natural;
  constant C_XE4 : t_xe := (4, 4, 4, 4, 4);   -- TUNE
  constant C_XE8 : t_xe := (7, 7, 7, 7, 7);   -- TUNE

end package cbv_tables_pkg;
