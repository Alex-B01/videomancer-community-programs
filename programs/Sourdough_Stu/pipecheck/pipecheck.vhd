-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: pipecheck.vhd - Pipecheck Program for Videomancer
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
--   Pipecheck
--
-- Author:
--   Sourdough Stu
--
-- Overview:
--   SDK 1.0 pipeline re-validation program. Deliberately minimal and exact:
--     proc amp (contrast/brightness on Y, saturation on U/V)
--       -> luma invert (toggle) -> dry/wet master mix (fader)
--       -> bypass mux -> blanking gate.
--   Per-pixel and stateless: no BRAM, no spatial index, no temporal state,
--   identical behaviour at every one of the 15 video timings.
--
-- Pipeline (program_top boundary latency, data_in -> data_out):
--   Wet leg:
--     P0      1 clk  : IO-align pad register on Y/U/V (s_pad_y/u/v), the /4 padding
--     P1-P10 10 clk  : proc_amp_u x3 (G_WIDTH=10: data=10, valid=9 -- valid unused)
--     P11     1 clk  : luma invert register (Y <- NOT Y when toggle 7), U/V companion
--     P12-P15 4 clk  : interpolator_u x3 master mix (data and valid aligned)
--     out     0 clk  : bypass mux + AVID blanking gate (combinational, registered sources)
--   ---------
--   1 + 10 + 1 + 4 = 16 (padding: P0 input register, 15 -> 16, /4)
--   Delay chain (one process, 16 deep):
--     H/V/field/AVID : tap 16                       -> data_out at 16
--     Dry Y/U/V      : tap 12 (16 - 4 mix), +4 mix  -> data_out at 16
--     Bypass Y/U/V   : tap 16                       -> data_out at 16
--   At the mix input the dry leg (12) and the wet leg (1 + 10 + 1 = 12) carry
--   the same input pixel.
--
-- Output stage (Gate 0 resolution 1: gate everything):
--   bypass mux (mix result vs 16-deep delayed raw input), THEN the blanking
--   gate: Y=64, U=V=512 whenever the delay-matched AVID is low. data_out.avid
--   is the chain AVID tap, never a DSP valid.
--
-- EBR map:
--   (none)                         : 0 physical EBR
--   Program EBR = 0 (full design: 0, or 1 on dual configs for the core's
--   dual_sync_delay). Delay chains are whole-array shifts -> flip-flops.
--
-- LC estimate (spec section 7, nominal):
--   (a) 3x proc_amp_u 1,200 + 3x interpolator_u 525 + clamp 75 + invert 10
--       + output mux 30                                        = 1,840
--   (b) sync chain 64 + dry/bypass chain 480 + pad 30 + U/V reg 20 = 594
--   (c) glue 20% of (a)                                        =   368
--   Program-only ~2,802 LC (2,710 .. 2,894); full design worst config ~4,529.
--   Multipliers: 3 Booth arrays (proc_amp_u) + 3 inferred (interpolator_u);
--   no multiply in program code.
--
-- Register map:
--   Pot 1  (registers_in(0))    : Contrast   -- proc_amp_y.contrast (512 = 1.0x)
--   Pot 2  (registers_in(1))    : Brightness -- proc_amp_y.brightness (512 = none)
--   Pot 3  (registers_in(2))    : Saturation -- proc_amp_u/v.contrast (0 = grey,
--                                              512 = identity)
--   Tog 7  (registers_in(6)(0)) : Luma Invert (0 = off, 1 = Y <- NOT Y)
--   Tog 11 (registers_in(6)(4)) : Bypass (1 = latency-matched raw input, gated)
--   Fader  (registers_in(7))    : Mix (0 = dry exact, 1023 = wet)
--   Not read: registers_in(3..5), registers_in(6)(3 downto 1), registers_in(8)
--   (timing ID unused: resolution-independent), registers_in(9..11) (core-only).
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture pipecheck of program_top is

    -- ========================================================================
    -- Latency (idiom #6 -- see the stage table in the file header)
    --   1 clk  : P0 IO-align pad register (wet leg)
    --  10 clk  : proc_amp_u (G_WIDTH=10 data latency)
    --   1 clk  : P11 luma invert register
    --   4 clk  : interpolator_u master mix
    -- ---------
    --   1 + 10 + 1 + 4 = 16 (padding: P0 input register, 15 -> 16, /4)
    -- ========================================================================
    constant C_PROCESSING_DELAY_CLKS : integer := 16;
    -- Documentation constants for the wet-leg stages.
    constant C_PAD_CLKS      : integer := 1;
    constant C_PROC_AMP_CLKS : integer := 10;
    constant C_INVERT_CLKS   : integer := 1;
    -- interpolator_u pipeline depth (dsp/interpolator.vhd): data and valid
    -- aligned, 4 cycles.
    constant C_MIX_LATENCY_CLKS : integer := 4;
    -- Dry-data tap: mix latency shorter than the sync tap, so that
    -- data-through-mix and sync arrive at data_out together (12).
    constant C_DRY_TAP : integer := C_PROCESSING_DELAY_CLKS - C_MIX_LATENCY_CLKS;

    -- Chroma midpoint (saturation pivot, blanking U/V)
    constant C_CHROMA_MID : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) :=
        to_unsigned(512, C_VIDEO_DATA_WIDTH);
    -- Studio blanking luma
    constant C_BLANK_Y : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) :=
        to_unsigned(64, C_VIDEO_DATA_WIDTH);

    -- ========================================================================
    -- Parameter signals
    -- ========================================================================
    signal s_contrast    : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_brightness  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_saturation  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_invert      : std_logic;
    signal s_bypass      : std_logic;
    signal s_mix_amount  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    -- ========================================================================
    -- P0: IO-align pad (wet leg)
    -- ========================================================================
    signal s_pad_y       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_pad_u       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_pad_v       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');

    -- ========================================================================
    -- P1-P10: proc amp results
    -- ========================================================================
    signal s_proc_y      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_proc_u      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_proc_v      : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    -- ========================================================================
    -- P11: luma invert register
    -- ========================================================================
    signal s_inv_y       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_inv_u       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_inv_v       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');

    -- ========================================================================
    -- P12-P15: master mix results
    -- ========================================================================
    signal s_mix_y       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_mix_u       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_mix_v       : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);

    -- ========================================================================
    -- Delay chain taps
    -- ========================================================================
    signal s_hsync_n_d   : std_logic := '1';
    signal s_vsync_n_d   : std_logic := '1';
    signal s_field_n_d   : std_logic := '1';
    signal s_avid_d      : std_logic := '0';
    signal s_dry_y       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_dry_u       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_dry_v       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_byp_y       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_byp_u       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_byp_v       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');

    -- ========================================================================
    -- Output stage: bypass mux result (before the blanking gate)
    -- ========================================================================
    signal s_sel_y       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_sel_u       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_sel_v       : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);

begin

    -- ========================================================================
    -- Register mapping
    -- ========================================================================
    s_contrast   <= unsigned(registers_in(0));   -- Pot 1: Contrast
    s_brightness <= unsigned(registers_in(1));   -- Pot 2: Brightness
    s_saturation <= unsigned(registers_in(2));   -- Pot 3: Saturation
    s_invert     <= registers_in(6)(0);          -- Tog 7: Luma Invert
    s_bypass     <= registers_in(6)(4);          -- Tog 11: Bypass
    s_mix_amount <= unsigned(registers_in(7));   -- Fader: Mix

    -- ========================================================================
    -- P0: IO-align pad register (wet leg only), C_PAD_CLKS = 1
    -- ========================================================================
    p_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_pad_y <= unsigned(data_in.y);
            s_pad_u <= unsigned(data_in.u);
            s_pad_v <= unsigned(data_in.v);
        end if;
    end process p_pad;

    -- ========================================================================
    -- P1-P10: proc amps, C_PROC_AMP_CLKS = 10 (data latency at G_WIDTH=10).
    -- enable tied '1' (kintsugi precedent): data latency is fixed; valid
    -- leads data by 1 and is left open.
    -- r = clamp(floor((a - 512) x c / 512) + 2b - 512, 0, 1023);
    -- c = b = 512 is the bit-exact identity.
    -- ========================================================================
    proc_amp_y : entity work.proc_amp_u
        generic map(G_WIDTH => C_VIDEO_DATA_WIDTH)
        port map(
            clk        => clk,
            enable     => '1',
            a          => s_pad_y,
            contrast   => s_contrast,
            brightness => s_brightness,
            result     => s_proc_y,
            valid      => open
        );

    -- Saturation: chroma gain about neutral (brightness tied to mid).
    proc_amp_u : entity work.proc_amp_u
        generic map(G_WIDTH => C_VIDEO_DATA_WIDTH)
        port map(
            clk        => clk,
            enable     => '1',
            a          => s_pad_u,
            contrast   => s_saturation,
            brightness => C_CHROMA_MID,
            result     => s_proc_u,
            valid      => open
        );

    proc_amp_v : entity work.proc_amp_u
        generic map(G_WIDTH => C_VIDEO_DATA_WIDTH)
        port map(
            clk        => clk,
            enable     => '1',
            a          => s_pad_v,
            contrast   => s_saturation,
            brightness => C_CHROMA_MID,
            result     => s_proc_v,
            valid      => open
        );

    -- ========================================================================
    -- P11: luma invert register, C_INVERT_CLKS = 1. Bitwise NOT (1023 - Y),
    -- yuv_amplifier precedent (Gate 0 resolution 2). Not AVID-gated: nothing
    -- downstream holds state and the output gate handles blanking.
    -- ========================================================================
    p_invert : process(clk)
    begin
        if rising_edge(clk) then
            if s_invert = '1' then
                s_inv_y <= not s_proc_y;
            else
                s_inv_y <= s_proc_y;
            end if;
            s_inv_u <= s_proc_u;
            s_inv_v <= s_proc_v;
        end if;
    end process p_invert;

    -- ========================================================================
    -- P12-P15: master mix, C_MIX_LATENCY_CLKS = 4. a = dry (tap 12),
    -- b = wet (12), t = fader. enable free-running '1' (never proc_amp valid,
    -- which leads its data by 1 and would drop the last active pixel).
    -- R1 clamp via the interpolator's own output generics (Gate 0 resolution 4).
    -- ========================================================================
    mix_y : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 64,
            G_OUTPUT_MAX => 940
        )
        port map(
            clk    => clk,
            enable => '1',
            a      => unsigned(s_dry_y),
            b      => s_inv_y,
            t      => s_mix_amount,
            result => s_mix_y,
            valid  => open
        );

    mix_u : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 64,
            G_OUTPUT_MAX => 960
        )
        port map(
            clk    => clk,
            enable => '1',
            a      => unsigned(s_dry_u),
            b      => s_inv_u,
            t      => s_mix_amount,
            result => s_mix_u,
            valid  => open
        );

    mix_v : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 64,
            G_OUTPUT_MAX => 960
        )
        port map(
            clk    => clk,
            enable => '1',
            a      => unsigned(s_dry_v),
            b      => s_inv_v,
            t      => s_mix_amount,
            result => s_mix_v,
            valid  => open
        );

    -- ========================================================================
    -- Sync, dry and bypass delay chain (one process, one depth).
    -- Index 0 = newest; tap k-1 = k clocks to the tap register.
    -- No knob, toggle or fader touches chain depth or chain data (R5).
    -- ========================================================================
    p_delay : process(clk)
        type t_sync_delay is array (0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic;
        type t_data_delay is array (0 to C_PROCESSING_DELAY_CLKS - 1)
            of std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
        variable v_hsync_delay : t_sync_delay := (others => '1');
        variable v_vsync_delay : t_sync_delay := (others => '1');
        variable v_field_delay : t_sync_delay := (others => '1');
        variable v_avid_delay  : t_sync_delay := (others => '0');
        variable v_y_delay     : t_data_delay := (others => (others => '0'));
        variable v_u_delay     : t_data_delay := (others => (others => '0'));
        variable v_v_delay     : t_data_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_hsync_delay := data_in.hsync_n & v_hsync_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_vsync_delay := data_in.vsync_n & v_vsync_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_field_delay := data_in.field_n & v_field_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_avid_delay  := data_in.avid    & v_avid_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_y_delay     := data_in.y       & v_y_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_u_delay     := data_in.u       & v_u_delay(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_v_delay     := data_in.v       & v_v_delay(0 to C_PROCESSING_DELAY_CLKS - 2);

            -- Sync + AVID: full chain (16)
            s_hsync_n_d <= v_hsync_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_vsync_n_d <= v_vsync_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_field_n_d <= v_field_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_avid_d    <= v_avid_delay(C_PROCESSING_DELAY_CLKS - 1);
            -- Dry: 12, then +4 through the mix = 16
            s_dry_y     <= v_y_delay(C_DRY_TAP - 1);
            s_dry_u     <= v_u_delay(C_DRY_TAP - 1);
            s_dry_v     <= v_v_delay(C_DRY_TAP - 1);
            -- Bypass: full chain (16)
            s_byp_y     <= v_y_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_byp_u     <= v_u_delay(C_PROCESSING_DELAY_CLKS - 1);
            s_byp_v     <= v_v_delay(C_PROCESSING_DELAY_CLKS - 1);
        end if;
    end process p_delay;

    -- ========================================================================
    -- Output stage (SDK 1.0 co-timing, Gate 0 resolution 1).
    -- 1) Bypass mux: latency-matched raw input vs mix result.
    -- 2) Blanking gate on the delay-matched AVID: Y=64, U=V=512 outside it.
    -- Combinational from registered sources; each data_out port driven once.
    -- Bypass reads registers_in(6)(4) directly (registers latch on HSYNC, so
    -- a bypass flip lands on a line edge).
    -- ========================================================================
    s_sel_y <= s_byp_y when s_bypass = '1' else std_logic_vector(s_mix_y);
    s_sel_u <= s_byp_u when s_bypass = '1' else std_logic_vector(s_mix_u);
    s_sel_v <= s_byp_v when s_bypass = '1' else std_logic_vector(s_mix_v);

    data_out.y <= s_sel_y when s_avid_d = '1'
                  else std_logic_vector(C_BLANK_Y);
    data_out.u <= s_sel_u when s_avid_d = '1'
                  else std_logic_vector(C_CHROMA_MID);
    data_out.v <= s_sel_v when s_avid_d = '1'
                  else std_logic_vector(C_CHROMA_MID);

    -- AVID and sync come only from the chain (never a DSP valid).
    data_out.avid    <= s_avid_d;
    data_out.hsync_n <= s_hsync_n_d;
    data_out.vsync_n <= s_vsync_n_d;
    data_out.field_n <= s_field_n_d;

end architecture pipecheck;
