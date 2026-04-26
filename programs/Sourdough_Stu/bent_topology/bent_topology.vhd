-- Bent Topology
-- Copyright (C) 2026 Alex Ball
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Multi-stage circuit-bent glitch suite.
-- Pipeline depth: C_PROCESSING_DELAY_CLKS = 16
--   1  : grid warp register
--   1  : alignment register + BRAM read
--   1  : smear IIR filter register
--   2  : smear delay (was 2; now 4 → see clocks 4–7)
--   1  : ring LUT addr register (clock 4)
--   1  : ring LUT data registered (clock 5)
--   1  : sin/cos LUT data registered (clock 6) ── NEW
--   1  : UV chroma rotation registered (clock 7) ── NEW
--   4  : ring interpolator (clocks 7→11)
--   1  : color crush register (clock 12)
--   4  : feedback / master mix interpolators (clocks 12→16)
--  ---
--  16  total
-- BRAM: ~13 EBR (1 × video_line_buffer G_DEPTH=11 = 10 EBR; ringing LUT = 2 EBR; sin/cos LUT = 1 EBR)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library work;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.clamp_pkg.all;

architecture bent_topology of program_top is

    constant C_PROCESSING_DELAY_CLKS : integer := 16;
    constant C_WR_PIPE_LEN           : integer := 15;

    -- Centred-unsigned YUV neutral chroma (per SDK convention)
    constant C_CHROMA_MID            : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- -------------------------------------------------------------------------
    -- Ringing LUT (1024 × 8-bit damped sinusoid, elaboration-time computed)
    -- -------------------------------------------------------------------------
    type t_ring_lut is array(0 to 1023) of std_logic_vector(7 downto 0);

    function fn_init_ring_lut return t_ring_lut is
        variable r    : t_ring_lut;
        variable t_r  : real;
        variable val  : real;
        variable ival : integer;
    begin
        for i in 0 to 1023 loop
            t_r  := real(i) / 1023.0;
            val  := 128.0 + 127.0 * sin(2.0 * MATH_PI * 6.0 * t_r) * exp(-4.0 * t_r);
            ival := integer(val);
            if ival < 0   then ival := 0;   end if;
            if ival > 255 then ival := 255; end if;
            r(i) := std_logic_vector(to_unsigned(ival, 8));
        end loop;
        return r;
    end function;

    signal s_ring_rom : t_ring_lut := fn_init_ring_lut;

    -- -------------------------------------------------------------------------
    -- Sin/Cos LUT (256 × 16-bit packed cos+sin, Q1.7 signed, elaboration-time)
    -- Cost: 1 EBR (256 × 16 = 4 kbits). Indexed by 8-bit angle (full circle).
    -- High byte = cos(2π·i/256), low byte = sin(2π·i/256), each scaled by 127.
    -- -------------------------------------------------------------------------
    constant C_SINCOS_DEPTH : integer := 256;
    type t_sincos_lut is array(0 to C_SINCOS_DEPTH - 1) of std_logic_vector(15 downto 0);

    function fn_init_sincos_lut return t_sincos_lut is
        variable r          : t_sincos_lut;
        variable angle_rad  : real;
        variable cos_int    : integer;
        variable sin_int    : integer;
    begin
        for i in 0 to C_SINCOS_DEPTH - 1 loop
            angle_rad := 2.0 * MATH_PI * real(i) / real(C_SINCOS_DEPTH);
            cos_int   := integer(round(cos(angle_rad) * 127.0));
            sin_int   := integer(round(sin(angle_rad) * 127.0));
            if cos_int > 127  then cos_int := 127;  end if;
            if cos_int < -128 then cos_int := -128; end if;
            if sin_int > 127  then sin_int := 127;  end if;
            if sin_int < -128 then sin_int := -128; end if;
            r(i) := std_logic_vector(to_signed(cos_int, 8))
                  & std_logic_vector(to_signed(sin_int, 8));
        end loop;
        return r;
    end function;

    signal s_sincos_rom : t_sincos_lut := fn_init_sincos_lut;

    -- -------------------------------------------------------------------------
    -- Helper functions for grid alpha-blend (B5) and chaos UV saturation (B8)
    -- -------------------------------------------------------------------------
    -- Alpha-blend input pixel toward 1023 (white) by alpha/16. alpha=0 → input passes
    -- through unchanged; alpha=15 → fully white. Used for Y at grid-on pixels.
    function fn_grid_blend_y(input : std_logic_vector(9 downto 0); alpha : unsigned(3 downto 0))
        return std_logic_vector is
        variable v_in : integer;
        variable v_alpha : integer;
    begin
        v_in    := to_integer(unsigned(input));
        v_alpha := to_integer(alpha);
        return std_logic_vector(to_unsigned(
            (v_in * (16 - v_alpha) + 1023 * v_alpha) / 16, 10));
    end function;

    -- Alpha-blend input pixel toward 512 (neutral grey) by alpha/16. alpha=0 → input
    -- passes through; alpha=15 → fully neutral. Used for U and V at grid-on pixels so
    -- that the grid line desaturates the underlying chroma rather than tinting it.
    function fn_grid_blend_uv(input : std_logic_vector(9 downto 0); alpha : unsigned(3 downto 0))
        return std_logic_vector is
        variable v_in : integer;
        variable v_alpha : integer;
    begin
        v_in    := to_integer(unsigned(input));
        v_alpha := to_integer(alpha);
        return std_logic_vector(to_unsigned(
            (v_in * (16 - v_alpha) + 512 * v_alpha) / 16, 10));
    end function;

    -- Chaos macro UV saturation: scale (uv - 512) by a factor selected from chaos.
    --   sat_level 0 → 0.5×  (heavy desat — chaos low = mellow)
    --   sat_level 3 → 1.0×  (passthrough)
    --   sat_level 7 → 1.75× (boost — chaos high = vivid)
    -- Cheap shift+add multiply, no `multiplier_s` needed.
    function fn_chaos_sat(uv : std_logic_vector(9 downto 0); sat_level : unsigned(2 downto 0))
        return std_logic_vector is
        variable v_in    : integer;
        variable v_dev   : integer;
        variable v_scale : integer;
    begin
        v_in  := to_integer(unsigned(uv));
        v_dev := v_in - 512;
        case to_integer(sat_level) is
            when 0      => v_scale := v_dev * 4;    -- 0.5×
            when 1      => v_scale := v_dev * 5;    -- 0.625×
            when 2      => v_scale := v_dev * 6;    -- 0.75×
            when 3      => v_scale := v_dev * 8;    -- 1.0×
            when 4      => v_scale := v_dev * 10;   -- 1.25×
            when 5      => v_scale := v_dev * 12;   -- 1.5×
            when 6      => v_scale := v_dev * 13;   -- 1.625×
            when others => v_scale := v_dev * 14;   -- 1.75×
        end case;
        -- Clamp to [0, 1023] after re-adding midpoint
        if (512 + v_scale / 8) < 0 then
            return std_logic_vector(to_unsigned(0, 10));
        elsif (512 + v_scale / 8) > 1023 then
            return std_logic_vector(to_unsigned(1023, 10));
        else
            return std_logic_vector(to_unsigned(512 + v_scale / 8, 10));
        end if;
    end function;

    -- -------------------------------------------------------------------------
    -- Timing infrastructure
    -- -------------------------------------------------------------------------
    signal s_timing    : t_video_timing_port;
    signal s_line_bank : std_logic := '0';
    signal s_hcount    : unsigned(10 downto 0) := (others => '0');
    signal s_vcount    : unsigned(9  downto 0) := (others => '0');

    -- -------------------------------------------------------------------------
    -- Toggle aliases (bit-packed from registers_in(6))
    -- -------------------------------------------------------------------------
    signal s_warp_axis      : std_logic;  -- bit 0: '0'=H grid, '1'=V grid
    signal s_smear_dir      : std_logic;  -- bit 1: '0'=H IIR,   '1'=V blend
    signal s_ring_polarity  : std_logic;  -- bit 2: '0'=bright,  '1'=dark
    signal s_luma_invert    : std_logic;  -- bit 3: '0'=normal,  '1'=invert Y
    signal s_chroma_isolate : std_logic;  -- bit 4: '0'=full,    '1'=Y-only glitch

    -- -------------------------------------------------------------------------
    -- Sync-delay taps (driven in p_sync_delay)
    -- Pipeline depth grew 14 → 16, so all taps below ring shifted by +2.
    -- -------------------------------------------------------------------------
    signal s_avid_smear_en : std_logic;                      -- avid,  3 clocks (unchanged)
    signal s_avid_d7       : std_logic;                      -- avid,  7 clocks (was d5)
    signal s_avid_d12      : std_logic;                      -- avid, 12 clocks (was d10)
    signal s_clean_y_d12   : std_logic_vector(9 downto 0);   -- Y,    12 clocks (was d10)
    signal s_clean_u_d12   : std_logic_vector(9 downto 0);   -- U,    12 clocks
    signal s_clean_v_d12   : std_logic_vector(9 downto 0);   -- V,    12 clocks

    -- -------------------------------------------------------------------------
    -- Write-back address
    -- -------------------------------------------------------------------------
    signal s_wr_addr_delayed : unsigned(10 downto 0);

    -- -------------------------------------------------------------------------
    -- Stage 1: Grid warp
    -- B5: grid alpha-blend (warp depth controls grid visibility, not just bend)
    -- B6: grid frequency via continuous phase accumulator (no more bit-sliced steps)
    -- B7: sigma-delta dither on warp depth for smoother bend control
    -- -------------------------------------------------------------------------
    signal s_y_for_warp        : std_logic_vector(9 downto 0);
    signal s_warp_shift_raw    : integer range 0 to 15;
    signal s_warp_shift        : integer range 0 to 15;          -- after B7 dither
    signal s_displacement_raw  : unsigned(9 downto 0);
    signal s_displacement      : unsigned(9 downto 0);            -- after B8 chaos bonus
    signal s_grid_alpha_raw    : unsigned(3 downto 0);            -- 0..15 from upper 4 bits
    signal s_grid_alpha        : unsigned(3 downto 0);            -- after B7 dither

    -- B7 sigma-delta dither accumulator (shared by warp shift + grid alpha + crush shift)
    signal s_warp_dither_acc   : unsigned(5 downto 0) := (others => '0');
    signal s_warp_dither_carry : std_logic := '0';
    signal s_crush_dither_acc  : unsigned(5 downto 0) := (others => '0');
    signal s_crush_dither_carry: std_logic := '0';

    -- B6 grid frequency phase accumulators
    signal s_grid_step    : unsigned(9 downto 0);                 -- knob value = step
    signal s_grid_phase_h : unsigned(15 downto 0) := (others => '0');
    signal s_grid_phase_v : unsigned(15 downto 0) := (others => '0');
    signal s_grid_chk_h   : unsigned(15 downto 0);                -- phase + displacement << 6
    signal s_grid_chk_v   : unsigned(15 downto 0);
    signal s_grid_h_on    : std_logic;
    signal s_grid_v_on    : std_logic;
    signal s_on_grid      : std_logic;

    signal s_grid_y      : std_logic_vector(9 downto 0);
    signal s_grid_u      : std_logic_vector(9 downto 0);
    signal s_grid_v      : std_logic_vector(9 downto 0);
    -- registered (clock 1):
    signal s_warped_y    : std_logic_vector(9 downto 0);
    signal s_warped_u    : std_logic_vector(9 downto 0);
    signal s_warped_v    : std_logic_vector(9 downto 0);

    -- -------------------------------------------------------------------------
    -- Stage 2: Alignment + Smear
    -- -------------------------------------------------------------------------
    -- clock 2: alignment register
    signal s_warped_d1_y : std_logic_vector(9 downto 0);
    signal s_warped_d1_u : std_logic_vector(9 downto 0);
    signal s_warped_d1_v : std_logic_vector(9 downto 0);
    -- line buffer output (available clock 2)
    signal s_prev_y      : std_logic_vector(9 downto 0);
    -- IIR filter outputs (combinational from filter state)
    signal s_filtered_y  : signed(10 downto 0);
    signal s_filtered_u  : signed(10 downto 0);
    signal s_filtered_v  : signed(10 downto 0);
    -- Freeze-during-blanking feeds: substitute filter output when avid is low.
    -- variable_filter_s updates s_y_reg every clock (enable only drives 'valid'),
    -- so without this the state decays toward 0 during blanking causing chroma
    -- collapse to green.  Feeding back the filter output makes err=0 → no change.
    signal s_smear_y_feed : signed(10 downto 0);
    signal s_smear_u_feed : signed(10 downto 0);
    signal s_smear_v_feed : signed(10 downto 0);
    signal s_smear_cutoff: unsigned(7 downto 0);
    -- vertical blend intermediate
    signal s_smear_v_shift : integer range 0 to 7;
    signal s_vert_blend_y  : unsigned(9 downto 0);
    -- smear output (clock 3):
    signal s_smeared_y   : std_logic_vector(9 downto 0);
    signal s_smeared_u   : std_logic_vector(9 downto 0);
    signal s_smeared_v   : std_logic_vector(9 downto 0);
    -- 4-clock delay from smear → ringing interp (clock 7) — was d2, now d4 to align
    -- with the rotated UV at clock 7 (sin/cos LUT + multiply add 2 stages).
    signal s_smeared_y_d4 : std_logic_vector(9 downto 0);
    signal s_smeared_u_d4 : std_logic_vector(9 downto 0);
    signal s_smeared_v_d4 : std_logic_vector(9 downto 0);

    -- -------------------------------------------------------------------------
    -- Stage 3: Ringing
    -- =========================================================================
    --  Existing damped-sinusoid offset (Y brightness ringing) preserved on Y.
    --  NEW for Option D: UV undergoes a true 2D chroma rotation around the
    --  centred-unsigned midpoint (512, 512), with rotation angle driven by the
    --  ring LUT value (0–255 → 0–2π). Replaces the old equal-signed UV offset
    --  which produced a single magenta/green axis.
    -- =========================================================================

    -- Stage 3a: ring LUT addr/data (clocks 4–5, unchanged)
    signal s_ring_bram_addr  : unsigned(9 downto 0);          -- clock 4
    signal s_ring_raw_8bit   : std_logic_vector(7 downto 0);  -- clock 5
    -- Polarity-mux output, then 2-clock pipeline alignment for downstream
    signal s_ring_val_8bit       : std_logic_vector(7 downto 0);  -- clock 5 (combinational)
    signal s_ring_val_8bit_d1    : std_logic_vector(7 downto 0);  -- clock 6
    signal s_ring_val_8bit_d2    : std_logic_vector(7 downto 0);  -- clock 7

    -- Y brightness-ring path (offset add at clock 7)
    signal s_ring_offset_d2  : integer;                       -- combinational at clock 7
    signal s_rung_y_full     : unsigned(9 downto 0);          -- combinational at clock 7

    -- Stage 3b: sin/cos LUT (clock 6)
    signal s_sincos_data     : std_logic_vector(15 downto 0); -- clock 6
    signal s_cos_theta       : signed(7 downto 0);            -- clock 6 (combinational from data)
    signal s_sin_theta       : signed(7 downto 0);            -- clock 6 (combinational from data)

    -- Stage 3c: UV chroma rotation (clock 7)
    -- Need s_smeared_*_d3 (between p_smear_delay's stages — see below).
    signal s_smeared_u_d3    : std_logic_vector(9 downto 0);  -- clock 6
    signal s_smeared_v_d3    : std_logic_vector(9 downto 0);  -- clock 6
    signal s_rotated_u       : unsigned(9 downto 0);          -- clock 7 (registered)
    signal s_rotated_v       : unsigned(9 downto 0);          -- clock 7 (registered)

    -- Stage 3d: ring interpolator (clocks 7 → 11) — output unchanged in shape
    signal s_rung_interp_y   : unsigned(9 downto 0);          -- clock 11 (was 9)
    signal s_rung_interp_u   : unsigned(9 downto 0);
    signal s_rung_interp_v   : unsigned(9 downto 0);

    -- -------------------------------------------------------------------------
    -- Stage 4: Color crush (clock 12)
    -- B7: sigma-delta dither smooths the 9 discrete shift levels
    -- -------------------------------------------------------------------------
    signal s_crush_shift_raw : integer range 0 to 15;
    signal s_crush_shift     : integer range 0 to 15;     -- after B7 dither
    signal s_crush_mask      : unsigned(9 downto 0);
    signal s_crushed_y       : std_logic_vector(9 downto 0);  -- clock 12
    signal s_crushed_u       : std_logic_vector(9 downto 0);
    signal s_crushed_v       : std_logic_vector(9 downto 0);

    -- B8: Chaos macro signals — chaos drives multiple parameters with progressive scaling
    signal s_chaos           : unsigned(9 downto 0);
    signal s_chaos_sat       : unsigned(2 downto 0);      -- 0..7 → UV scaling around 512
    signal s_chaos_ring_bonus: unsigned(7 downto 0);      -- added to ring amp
    signal s_chaos_warp_bonus: unsigned(5 downto 0);      -- added to displacement
    signal s_ring_amp_eff    : unsigned(9 downto 0);      -- ring_amp + chaos_ring_bonus, clamped
    signal s_crushed_u_chaos : std_logic_vector(9 downto 0);  -- crushed_u after chaos sat
    signal s_crushed_v_chaos : std_logic_vector(9 downto 0);

    -- -------------------------------------------------------------------------
    -- Stage 5: Feedback interpolator + line buffer
    -- -------------------------------------------------------------------------
    signal s_feedback_y  : unsigned(9 downto 0);  -- clock 16

    -- -------------------------------------------------------------------------
    -- Stage 6: Master mix interpolators (clock 16, was 14)
    -- -------------------------------------------------------------------------
    signal s_master_y : unsigned(9 downto 0);
    signal s_master_u : unsigned(9 downto 0);
    signal s_master_v : unsigned(9 downto 0);

begin

    -- =========================================================================
    -- Timing generator
    -- =========================================================================
    timing_gen : entity work.video_timing_generator
        port map(
            clk         => clk,
            ref_hsync_n => data_in.hsync_n,
            ref_vsync_n => data_in.vsync_n,
            ref_avid    => data_in.avid,
            timing      => s_timing
        );

    -- =========================================================================
    -- Toggle decode (concurrent assignments from bit-packed registers_in(6))
    -- =========================================================================
    s_warp_axis      <= registers_in(6)(0);
    s_smear_dir      <= registers_in(6)(1);
    s_ring_polarity  <= registers_in(6)(2);
    s_luma_invert    <= registers_in(6)(3);
    s_chroma_isolate <= registers_in(6)(4);

    -- =========================================================================
    -- Line bank toggle: flip on each hsync_start; reset on vsync_start
    -- =========================================================================
    p_bank : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_line_bank <= '0';
            elsif s_timing.hsync_start = '1' then
                s_line_bank <= not s_line_bank;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- H/V counters (own — t_video_timing_port has no hcount field)
    -- =========================================================================
    p_hcount : process(clk)
    begin
        if rising_edge(clk) then
            if data_in.avid = '0' then
                s_hcount <= (others => '0');
            else
                s_hcount <= s_hcount + 1;
            end if;
        end if;
    end process;

    p_vcount : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_vcount <= (others => '0');
            elsif s_timing.hsync_start = '1' then
                s_vcount <= s_vcount + 1;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Stage 1: Grid Warp
    -- B5: Grid alpha-blends with input pixel (warp depth controls visibility).
    -- B6: Grid frequency via continuous phase accumulator (no bit-sliced steps).
    -- B7: Sigma-delta dither smooths warp depth shift transitions.
    -- B8: Chaos macro adds bonus displacement.
    -- Combinational detection; registered output at clock 1.
    -- =========================================================================

    -- Luma invert (Toggle 10): complement Y before displacement
    s_y_for_warp <= not data_in.y when s_luma_invert = '1' else data_in.y;

    -- Warp Depth (Knob 1):
    --   Upper 4 bits select right-shift amount (raw): high knob → small shift → large bend.
    --   Same upper 4 bits also drive the grid alpha (visibility): 0 = invisible, 15 = white.
    --   Lower 6 bits drive sigma-delta dither (B7) for smoother transitions between shifts.
    s_warp_shift_raw  <= 15 - to_integer(unsigned(registers_in(0)(9 downto 6)));
    s_grid_alpha_raw  <= unsigned(registers_in(0)(9 downto 6));

    -- B7 dither accumulator: increments by lower-6-bit knob density per clock; carry
    -- bumps the active shift by 1 (= more displacement) and grid alpha by 1.
    p_warp_dither : process(clk)
        variable v_acc_next : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            v_acc_next := ('0' & s_warp_dither_acc) + ('0' & unsigned(registers_in(0)(5 downto 0)));
            s_warp_dither_acc   <= v_acc_next(5 downto 0);
            s_warp_dither_carry <= v_acc_next(6);
        end if;
    end process;

    -- Apply dither: when carry, use one-finer shift (more displacement) and one-step-up alpha
    s_warp_shift <= s_warp_shift_raw - 1
                    when (s_warp_dither_carry = '1' and s_warp_shift_raw > 0)
                    else s_warp_shift_raw;
    s_grid_alpha <= s_grid_alpha_raw + 1
                    when (s_warp_dither_carry = '1' and s_grid_alpha_raw < 15)
                    else s_grid_alpha_raw;

    -- Displacement = Y >> warp_shift, plus B8 chaos bonus (small)
    s_displacement_raw <= shift_right(unsigned(s_y_for_warp), s_warp_shift);
    s_displacement     <= s_displacement_raw + resize(s_chaos_warp_bonus, 10);

    -- B6 Grid frequency phase accumulators
    --   Knob 2 (warp_freq) value is the per-pixel phase increment. Continuous control:
    --   knob=0 → step=0 → no phase advance → no grid lines anywhere.
    --   knob=1023 → max step → finest grid.
    --   16-bit phase, grid-on when upper 3 bits = "000" (~1/8 duty cycle).
    s_grid_step <= unsigned(registers_in(1));

    p_grid_phase_h : process(clk)
    begin
        if rising_edge(clk) then
            if data_in.avid = '0' then
                s_grid_phase_h <= (others => '0');  -- restart per line
            else
                s_grid_phase_h <= s_grid_phase_h + s_grid_step;
            end if;
        end if;
    end process;

    p_grid_phase_v : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_grid_phase_v <= (others => '0');  -- restart per frame
            elsif s_timing.hsync_start = '1' then
                s_grid_phase_v <= s_grid_phase_v + s_grid_step;
            end if;
        end if;
    end process;

    -- Apply warp displacement to phase before grid-on check (bends the grid).
    -- Displacement is 10-bit; shift_left by 6 = align with the 16-bit phase magnitude.
    -- (Using shift_left instead of "000000" string concat to avoid the GHDL `&`
    --  ambiguity trap — see _meta/found_issues.md.)
    s_grid_chk_h <= s_grid_phase_h + shift_left(resize(s_displacement, 16), 6);
    s_grid_chk_v <= s_grid_phase_v + shift_left(resize(s_displacement, 16), 6);

    s_grid_h_on <= '1' when s_grid_chk_h(15 downto 13) = "000" else '0';
    s_grid_v_on <= '1' when s_grid_chk_v(15 downto 13) = "000" else '0';

    -- Toggle 7: '0' = V-axis grid (vertical lines, bent by H), '1' = H-axis grid
    s_on_grid <= s_grid_v_on when s_warp_axis = '0' else s_grid_h_on;

    -- B5 Grid pixel: alpha-blend input toward white (Y) / midpoint (UV) when grid-on.
    --   alpha=0 (low warp depth) → grid invisible (input passes through unchanged).
    --   alpha=15 (high warp depth) → grid pixel is fully white (Y) + neutral grey (UV).
    --   In between → blended with the underlying scene content.
    s_grid_y <= fn_grid_blend_y (data_in.y, s_grid_alpha) when s_on_grid = '1' else data_in.y;
    s_grid_u <= fn_grid_blend_uv(data_in.u, s_grid_alpha) when s_on_grid = '1' else data_in.u;
    s_grid_v <= fn_grid_blend_uv(data_in.v, s_grid_alpha) when s_on_grid = '1' else data_in.v;

    -- Register grid output → s_warped at clock 1
    p_grid_reg : process(clk)
    begin
        if rising_edge(clk) then
            s_warped_y <= s_grid_y;
            s_warped_u <= s_grid_u;
            s_warped_v <= s_grid_v;
        end if;
    end process;

    -- =========================================================================
    -- Stage 2: Smear
    -- Alignment register at clock 2; smear output registered at clock 3.
    -- =========================================================================

    -- Alignment register: warped → warped_d1 at clock 2 (aligns with s_prev_y)
    p_align : process(clk)
    begin
        if rising_edge(clk) then
            s_warped_d1_y <= s_warped_y;
            s_warped_d1_u <= s_warped_u;
            s_warped_d1_v <= s_warped_v;
        end if;
    end process;

    -- Smear freeze-during-blanking: when avid is low, feed the filter its own output
    -- so err = a - state = 0 and the IIR state holds.
    s_smear_y_feed <= signed('0' & unsigned(s_warped_d1_y)) when s_avid_smear_en = '1'
                 else s_filtered_y;
    s_smear_u_feed <= signed('0' & unsigned(s_warped_d1_u)) when s_avid_smear_en = '1'
                 else s_filtered_u;
    s_smear_v_feed <= signed('0' & unsigned(s_warped_d1_v)) when s_avid_smear_en = '1'
                 else s_filtered_v;

    -- Smear cutoff: Knob 3 upper 4 bits → 16 IIR cutoff levels.
    -- Range: 0x10 (v_k=1, hard) → 0x50 (v_k=5, softest used).
    --
    -- For 10-bit video, the silent-death rule is v_k ≤ W-2 = 8, so cutoff ≤ 0x80 is
    -- "mathematically safe" (filter still updates). But at v_k=8, err >> 8 = 0 unless
    -- err > 256 — i.e. only big transitions update state. On natural images this means
    -- the filter heavily averages within each line and converges to the line's mean
    -- colour (typically the dominant background hue, often green for foliage). So while
    -- v_k=8 is silent-death-safe, it's *aesthetically* greenwashing.
    --
    -- v_k=5 (cutoff 0x50 = 80) gives `err >> 5` ≈ `err / 32`, so a typical pixel
    -- transition of 100 LSB updates state by 3 per clock. Filter still smears heavily
    -- but tracks the image content rather than collapsing to mean. Lower nibble values
    -- (0x10..0x4F) drive the filter's internal sigma-delta dither for sub-step
    -- granularity between coarse v_k levels.
    --
    -- Replaces the previous 8-entry table whose top three values (0x92, 0xA9, 0xC0)
    -- were in the silent-death zone and produced UV→0 → green-collapse on hardware.
    with unsigned(registers_in(2)(9 downto 6)) select s_smear_cutoff <=
        to_unsigned( 16, 8) when "0000",   -- v_k=1
        to_unsigned( 20, 8) when "0001",   -- v_k=1, sigma-delta density 4
        to_unsigned( 24, 8) when "0010",   -- v_k=1, sigma-delta density 8
        to_unsigned( 32, 8) when "0011",   -- v_k=2
        to_unsigned( 36, 8) when "0100",   -- v_k=2, sigma-delta density 4
        to_unsigned( 44, 8) when "0101",   -- v_k=2, sigma-delta density 12
        to_unsigned( 48, 8) when "0110",   -- v_k=3
        to_unsigned( 52, 8) when "0111",   -- v_k=3, sigma-delta density 4
        to_unsigned( 56, 8) when "1000",   -- v_k=3, sigma-delta density 8
        to_unsigned( 60, 8) when "1001",   -- v_k=3, sigma-delta density 12
        to_unsigned( 64, 8) when "1010",   -- v_k=4
        to_unsigned( 68, 8) when "1011",   -- v_k=4, sigma-delta density 4
        to_unsigned( 72, 8) when "1100",   -- v_k=4, sigma-delta density 8
        to_unsigned( 76, 8) when "1101",   -- v_k=4, sigma-delta density 12
        to_unsigned( 80, 8) when "1110",   -- v_k=5 (softest used)
        to_unsigned( 80, 8) when others;   -- v_k=5 (saturate top of knob)

    -- Vertical blend shift: 0 (heavy blend = mostly previous line) = Knob 3 max
    --                       7 (no blend = mostly current line)     = Knob 3 zero
    -- Now uses 4-bit slice for finer granularity, mapped to 0..7 range.
    s_smear_v_shift <= 7 - (to_integer(unsigned(registers_in(2)(9 downto 6))) / 2);

    -- Vertical blend Y (combinational): WEIGHTED BLEND between current and previous line.
    -- Was: clamp(warped + (prev >> shift)) — additive saturation, not a real blend.
    --      At max knob, shift=0 → out = warped + prev → clamps to 1023 → bright white.
    -- Now: out = (warped * (8 - blend_w) + prev * blend_w) / 8
    --      where blend_w = 8 - smear_v_shift (so shift=0 → blend_w=8 → all prev,
    --                                          shift=7 → blend_w=1 → mostly current)
    -- Energy-conserving: at warped=prev=1023, output is 1023 (no over-saturation).
    p_vert_blend : process(s_warped_d1_y, s_prev_y, s_smear_v_shift)
        variable v_blend_w : integer range 0 to 8;
        variable v_warp    : integer;
        variable v_prev    : integer;
    begin
        v_blend_w := 8 - s_smear_v_shift;
        v_warp    := to_integer(unsigned(s_warped_d1_y));
        v_prev    := to_integer(unsigned(s_prev_y));
        s_vert_blend_y <= fn_clamp_int_to_u(
            (v_warp * (8 - v_blend_w) + v_prev * v_blend_w) / 8, 10);
    end process;

    -- IIR horizontal filter instances (Y, U, V)
    -- Y is used in H mode only; U/V always horizontal regardless of Toggle 8.
    smear_y : entity work.variable_filter_s
        generic map(G_WIDTH => 11)
        port map(
            clk       => clk,
            enable    => s_avid_smear_en,
            a         => s_smear_y_feed,
            cutoff    => s_smear_cutoff,
            low_pass  => s_filtered_y,
            high_pass => open,
            valid     => open
        );

    smear_u : entity work.variable_filter_s
        generic map(G_WIDTH => 11)
        port map(
            clk       => clk,
            enable    => s_avid_smear_en,
            a         => s_smear_u_feed,
            cutoff    => s_smear_cutoff,
            low_pass  => s_filtered_u,
            high_pass => open,
            valid     => open
        );

    smear_v : entity work.variable_filter_s
        generic map(G_WIDTH => 11)
        port map(
            clk       => clk,
            enable    => s_avid_smear_en,
            a         => s_smear_v_feed,
            cutoff    => s_smear_cutoff,
            low_pass  => s_filtered_v,
            high_pass => open,
            valid     => open
        );

    -- Smear output register at clock 3
    -- Toggle 8 = '1': vertical blend for Y; '0': horizontal IIR.
    -- U/V: always horizontal IIR.
    p_smear_reg : process(clk)
    begin
        if rising_edge(clk) then
            if s_smear_dir = '1' then
                s_smeared_y <= std_logic_vector(s_vert_blend_y);
            else
                s_smeared_y <= std_logic_vector(fn_clamp_s_to_u(s_filtered_y, 10));
            end if;
            s_smeared_u <= std_logic_vector(fn_clamp_s_to_u(s_filtered_u, 10));
            s_smeared_v <= std_logic_vector(fn_clamp_s_to_u(s_filtered_v, 10));
        end if;
    end process;

    -- 4-clock delay: smear output (clock 3) → ringing interpolator 'a' input (clock 7)
    -- Was 2 stages → s_smeared_*_d2 at clock 5; now 4 stages → s_smeared_*_d4 at clock 7
    -- to align with the rotated UV produced by the new Option D rotation pipeline.
    -- Also exposes intermediate s_smeared_*_d3 (clock 6) used as rotation input.
    p_smear_delay : process(clk)
        type t_d4 is array(0 to 3) of std_logic_vector(9 downto 0);
        variable vy : t_d4 := (others => (others => '0'));
        variable vu : t_d4 := (others => (others => '0'));
        variable vv : t_d4 := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            vy := s_smeared_y & vy(0 to 2);
            vu := s_smeared_u & vu(0 to 2);
            vv := s_smeared_v & vv(0 to 2);
            -- Intermediate tap at d3 (clock 6) feeds the chroma rotation
            s_smeared_u_d3 <= vu(2);
            s_smeared_v_d3 <= vv(2);
            -- Final tap at d4 (clock 7) feeds ring_interp 'a' input (and Y rung path)
            s_smeared_y_d4 <= vy(3);
            s_smeared_u_d4 <= vu(3);
            s_smeared_v_d4 <= vv(3);
        end if;
    end process;

    -- =========================================================================
    -- Stage 3: Ringing
    -- =========================================================================

    -- Stage 3a: Register BRAM address at clock 4 (from s_smeared_y at clock 3)
    p_ring_addr : process(clk)
    begin
        if rising_edge(clk) then
            s_ring_bram_addr <= unsigned(s_smeared_y);
        end if;
    end process;

    -- Synchronous ROM read: 1-clock latency → s_ring_raw_8bit at clock 5
    p_ring_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_ring_raw_8bit <= s_ring_rom(to_integer(s_ring_bram_addr));
        end if;
    end process;

    -- Polarity toggle (Toggle 9): '1' = invert LUT output (dark echo)
    -- Combinational at clock 5; pipelined 2 more clocks for alignment with rotated UV.
    s_ring_val_8bit <= s_ring_raw_8bit when s_ring_polarity = '0'
                  else std_logic_vector(to_unsigned(255, 8) - unsigned(s_ring_raw_8bit));

    p_ring_val_pipe : process(clk)
    begin
        if rising_edge(clk) then
            s_ring_val_8bit_d1 <= s_ring_val_8bit;       -- clock 6
            s_ring_val_8bit_d2 <= s_ring_val_8bit_d1;     -- clock 7
        end if;
    end process;

    -- Y brightness ringing offset (combinational at clock 7)
    -- Centre of LUT (128) = zero offset; scaled ×4 for ±512 amplitude on 10-bit.
    s_ring_offset_d2 <= (to_integer(unsigned(s_ring_val_8bit_d2)) - 128) * 4;

    s_rung_y_full <= fn_clamp_int_to_u(
        to_integer(unsigned(s_smeared_y_d4)) + s_ring_offset_d2, 10);

    -- Stage 3b: Sin/Cos LUT read (clock 6)
    -- Indexed by (ring_val_8bit XOR 0x80) at clock 5. Output cos/sin at clock 6.
    -- The XOR re-centres the ring LUT's 128 resting value to angle index 0
    -- (identity rotation). Without this, the damped sinusoid mostly produces
    -- ring_val ≈ 128 → LUT index 128 → angle π → identity-negate, which the
    -- blend with original UV cancels to neutral grey.
    p_sincos_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_sincos_data <= s_sincos_rom(to_integer(unsigned(s_ring_val_8bit) xor x"80"));
        end if;
    end process;

    -- Unpack: high byte = cos, low byte = sin (both signed 8-bit, Q1.7)
    s_cos_theta <= signed(s_sincos_data(15 downto 8));
    s_sin_theta <= signed(s_sincos_data( 7 downto 0));

    -- Stage 3c: UV chroma rotation (clock 7)
    -- Rotates the chroma vector (U-512, V-512) by angle θ around the midpoint.
    --   U' = 512 + ((U-512) * cos θ - (V-512) * sin θ) >> 7
    --   V' = 512 + ((U-512) * sin θ + (V-512) * cos θ) >> 7
    -- Inputs: s_smeared_u_d3, s_smeared_v_d3 (clock 6); s_cos_theta, s_sin_theta (clock 6).
    -- Output: s_rotated_u, s_rotated_v at clock 7 (registered, clamped to [0, 1023]).
    --
    -- Worst-case widths: (U-512) is signed 11-bit ≤ ±512; cos/sin signed 8-bit ≤ ±127.
    -- Product ≤ ±65024 (signed 18-bit). Sum of two products ≤ ±130048 (signed 19-bit).
    -- Right shift by 7 → signed 12-bit ≤ ±1024. Add 512 → range [-512, +1536].
    -- Clamp to [0, 1023] → unsigned 10-bit. ✓
    p_rotate_uv : process(clk)
        variable v_u_centred : signed(10 downto 0);
        variable v_v_centred : signed(10 downto 0);
        variable v_u_cos     : signed(18 downto 0);  -- 11-bit × 8-bit signed
        variable v_v_sin     : signed(18 downto 0);
        variable v_u_sin     : signed(18 downto 0);
        variable v_v_cos     : signed(18 downto 0);
        variable v_u_rot     : signed(19 downto 0);  -- sum: one extra bit for headroom
        variable v_v_rot     : signed(19 downto 0);
        variable v_u_scaled  : integer;
        variable v_v_scaled  : integer;
    begin
        if rising_edge(clk) then
            v_u_centred := signed('0' & s_smeared_u_d3) - to_signed(512, 11);
            v_v_centred := signed('0' & s_smeared_v_d3) - to_signed(512, 11);

            v_u_cos := v_u_centred * s_cos_theta;
            v_v_sin := v_v_centred * s_sin_theta;
            v_u_sin := v_u_centred * s_sin_theta;
            v_v_cos := v_v_centred * s_cos_theta;

            v_u_rot := resize(v_u_cos, 20) - resize(v_v_sin, 20);
            v_v_rot := resize(v_u_sin, 20) + resize(v_v_cos, 20);

            -- Right-shift by 7 (Q1.7 scale) and add midpoint
            v_u_scaled := to_integer(shift_right(v_u_rot, 7)) + 512;
            v_v_scaled := to_integer(shift_right(v_v_rot, 7)) + 512;

            s_rotated_u <= fn_clamp_int_to_u(v_u_scaled, 10);
            s_rotated_v <= fn_clamp_int_to_u(v_v_scaled, 10);
        end if;
    end process;

    -- Stage 3d: Ringing interpolator
    -- a = smeared (no ring); b = rung-Y / rotated-UV (full ring); t = Knob 4 (ring amp).
    -- Enable at clock 7 (s_avid_d7). Output at clock 11.
    ring_interp_y : entity work.interpolator_u
        generic map(
            G_WIDTH      => 10,
            G_FRAC_BITS  => 10,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_avid_d7,
            a      => unsigned(s_smeared_y_d4),
            b      => s_rung_y_full,
            t      => unsigned(std_logic_vector(s_ring_amp_eff)),
            result => s_rung_interp_y,
            valid  => open
        );

    ring_interp_u : entity work.interpolator_u
        generic map(
            G_WIDTH      => 10,
            G_FRAC_BITS  => 10,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_avid_d7,
            a      => unsigned(s_smeared_u_d4),
            b      => s_rotated_u,
            t      => unsigned(std_logic_vector(s_ring_amp_eff)),
            result => s_rung_interp_u,
            valid  => open
        );

    ring_interp_v : entity work.interpolator_u
        generic map(
            G_WIDTH      => 10,
            G_FRAC_BITS  => 10,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_avid_d7,
            a      => unsigned(s_smeared_v_d4),
            b      => s_rotated_v,
            t      => unsigned(std_logic_vector(s_ring_amp_eff)),
            result => s_rung_interp_v,
            valid  => open
        );

    -- =========================================================================
    -- Stage 4: Color Crush
    -- Register output at clock 12.
    -- Chroma Isolate (Toggle 11): substitute clean UV for crushed UV.
    --
    -- B2 fix (correctness + usable range):
    --   (a) Map 16-step knob upper 4 bits to shifts 0..8 instead of 0..15.
    --       Max shift = 8 leaves Y with 4 brightness levels (mask 0x300, top 2 bits
    --       preserved) and UV with 4 chroma levels around 512 — heavy posterization
    --       but image content remains visible. Max shift = 10 (mask=0) was a "useless"
    --       black-screen position; the upper third of the knob travel was wasted there.
    --       New mapping uses the case below to cover shift 0..8 across all 16 knob
    --       positions, with each shift level repeated 1-2 times for a smooth feel.
    --   (b) UV crushes around the 512 midpoint, NOT around 0. Implemented as
    --       sign-and-magnitude: extract sign of (UV-512), mask the magnitude, restore.
    --       Bit-mask AND on two's complement doesn't round toward 0 for negatives —
    --       sign-and-magnitude does. Result: UV crushes toward neutral grey =
    --       progressive desaturation. Y still crushes around 0 (heavy crush → dark
    --       posterization, the right semantic for luma).
    --   Pattern reference: boneoh/yuv_bit_crush/yuv_bit_crush.vhd uses RPDF dither
    --   and a similar midpoint-aware approach for UV.
    -- =========================================================================
    -- Knob upper 4 bits → raw shift 0..8 (max preserves top 2 bits, never wipes to 0).
    -- 16 knob positions map to 9 shift levels with each level used 1-2× for smoothness.
    with to_integer(unsigned(registers_in(4)(9 downto 6))) select s_crush_shift_raw <=
        0 when  0,
        1 when  1,  1 when  2,
        2 when  3,  2 when  4,
        3 when  5,  3 when  6,
        4 when  7,  4 when  8,
        5 when  9,  5 when 10,
        6 when 11,  6 when 12,
        7 when 13,  7 when 14,
        8 when others;   -- knob 15 = max useful crush (top 2 bits of Y, UV ±256 around 512)

    -- B7 sigma-delta dither for crush shift: smooths the discrete shift transitions
    -- using lower 6 bits of the knob as dither density.
    p_crush_dither : process(clk)
        variable v_acc_next : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            v_acc_next := ('0' & s_crush_dither_acc) + ('0' & unsigned(registers_in(4)(5 downto 0)));
            s_crush_dither_acc   <= v_acc_next(5 downto 0);
            s_crush_dither_carry <= v_acc_next(6);
        end if;
    end process;

    -- Apply: when carry fires, bump shift up by 1 (= more crush)
    s_crush_shift <= s_crush_shift_raw + 1
                     when (s_crush_dither_carry = '1' and s_crush_shift_raw < 8)
                     else s_crush_shift_raw;

    -- Safe mask: shift_left of all-ones, clamped; shift of 0 = full pass-through
    s_crush_mask <= (others => '1') when s_crush_shift = 0
               else shift_left(to_unsigned(1023, 10), s_crush_shift) and to_unsigned(1023, 10);

    p_crush_reg : process(clk)
        -- Helper variables for UV midpoint-anchored crush via sign-and-magnitude
        variable v_u_in     : integer;
        variable v_v_in     : integer;
        variable v_u_dev    : integer;   -- signed deviation from 512
        variable v_v_dev    : integer;
        variable v_u_mag    : unsigned(9 downto 0);  -- magnitude (0..512)
        variable v_v_mag    : unsigned(9 downto 0);
        variable v_u_masked : integer;   -- magnitude after mask
        variable v_v_masked : integer;
    begin
        if rising_edge(clk) then
            -- Y: crush around 0 (heavy crush → black)
            s_crushed_y <= std_logic_vector(unsigned(s_rung_interp_y) and s_crush_mask);

            if s_chroma_isolate = '1' then
                -- Y-only mode: bypass UV crush entirely
                s_crushed_u <= s_clean_u_d12;
                s_crushed_v <= s_clean_v_d12;
            else
                -- UV: crush around 512 midpoint via sign-and-magnitude masking.
                v_u_in  := to_integer(unsigned(s_rung_interp_u));
                v_v_in  := to_integer(unsigned(s_rung_interp_v));
                v_u_dev := v_u_in - 512;
                v_v_dev := v_v_in - 512;

                -- U channel: separate sign, mask the magnitude, recombine
                if v_u_dev >= 0 then
                    v_u_mag    := to_unsigned(v_u_dev, 10);
                    v_u_masked := to_integer(v_u_mag and s_crush_mask);
                    s_crushed_u <= std_logic_vector(to_unsigned(512 + v_u_masked, 10));
                else
                    v_u_mag    := to_unsigned(-v_u_dev, 10);
                    v_u_masked := to_integer(v_u_mag and s_crush_mask);
                    s_crushed_u <= std_logic_vector(to_unsigned(512 - v_u_masked, 10));
                end if;

                -- V channel: same pattern
                if v_v_dev >= 0 then
                    v_v_mag    := to_unsigned(v_v_dev, 10);
                    v_v_masked := to_integer(v_v_mag and s_crush_mask);
                    s_crushed_v <= std_logic_vector(to_unsigned(512 + v_v_masked, 10));
                else
                    v_v_mag    := to_unsigned(-v_v_dev, 10);
                    v_v_masked := to_integer(v_v_mag and s_crush_mask);
                    s_crushed_v <= std_logic_vector(to_unsigned(512 - v_v_masked, 10));
                end if;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- B8: Chaos macro knob — drives multiple parameters with progressive engagement.
    -- Replaces the previous design where chaos only affected the V-smear feedback
    -- (which was invisible in default H-smear mode → wasted control).
    --
    -- Now chaos always does *something* visible regardless of mode:
    --   • Always: UV saturation scaling (low chaos = mellow/desaturated, high = vivid)
    --   • Always: bonus added to ring amp (chaos gives extra ring activity)
    --   • Always: bonus added to warp displacement (chaos perturbs the bend)
    --   • V-smear mode: existing BRAM feedback blend (kept as-is below)
    -- =========================================================================
    s_chaos            <= unsigned(registers_in(5));
    s_chaos_sat        <= s_chaos(9 downto 7);              -- 8 levels (0..7)
    s_chaos_ring_bonus <= "00" & s_chaos(9 downto 4);       -- 0..63 << 0 = 0..63 added to ring
    s_chaos_warp_bonus <= s_chaos(9 downto 4);              -- 0..63 added to warp displacement

    -- Apply chaos UV saturation to crushed UV (combinational, sourced at clock 12)
    s_crushed_u_chaos <= fn_chaos_sat(s_crushed_u, s_chaos_sat);
    s_crushed_v_chaos <= fn_chaos_sat(s_crushed_v, s_chaos_sat);

    -- Effective ring amp: knob 4 + chaos bonus, saturating at 1023
    s_ring_amp_eff <= to_unsigned(1023, 10)
                     when (to_integer(unsigned(registers_in(3))) + to_integer(s_chaos_ring_bonus)) > 1023
                     else to_unsigned(
                          to_integer(unsigned(registers_in(3))) + to_integer(s_chaos_ring_bonus), 10);

    -- =========================================================================
    -- Stage 5: Feedback Blend + Line Buffer (Y only)
    -- Feedback interpolator: a=clean, b=crushed, t=Knob 6 (existing chaos in V-mode).
    -- At t=0: writes clean → V-mode smear reads unglitched prev line (no chaos).
    -- At t>0: writes glitch → chaos accumulates across lines.
    -- =========================================================================
    feedback_interp : entity work.interpolator_u
        generic map(
            G_WIDTH      => 10,
            G_FRAC_BITS  => 10,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_avid_d12,
            a      => unsigned(s_clean_y_d12),
            b      => unsigned(s_crushed_y),
            t      => unsigned(registers_in(5)),
            result => s_feedback_y,
            valid  => open
        );

    lb_y : entity work.video_line_buffer
        generic map(G_WIDTH => 10, G_DEPTH => 11)
        port map(
            clk       => clk,
            i_ab      => s_line_bank,
            i_wr_addr => s_wr_addr_delayed,
            i_rd_addr => s_hcount,
            i_data    => std_logic_vector(s_feedback_y),
            o_data    => s_prev_y
        );

    -- =========================================================================
    -- Stage 6: Master Mix
    -- Three interpolator_u instances: a=clean, b=crushed, t=Slider 12.
    -- Output at clock 16 = C_PROCESSING_DELAY_CLKS.
    -- =========================================================================
    master_y : entity work.interpolator_u
        generic map(
            G_WIDTH      => 10,
            G_FRAC_BITS  => 10,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_avid_d12,
            a      => unsigned(s_clean_y_d12),
            b      => unsigned(s_crushed_y),
            t      => unsigned(registers_in(7)),
            result => s_master_y,
            valid  => open
        );

    master_u : entity work.interpolator_u
        generic map(
            G_WIDTH      => 10,
            G_FRAC_BITS  => 10,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_avid_d12,
            a      => unsigned(s_clean_u_d12),
            b      => unsigned(s_crushed_u_chaos),
            t      => unsigned(registers_in(7)),
            result => s_master_u,
            valid  => open
        );

    master_v : entity work.interpolator_u
        generic map(
            G_WIDTH      => 10,
            G_FRAC_BITS  => 10,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_avid_d12,
            a      => unsigned(s_clean_v_d12),
            b      => unsigned(s_crushed_v_chaos),
            t      => unsigned(registers_in(7)),
            result => s_master_v,
            valid  => open
        );

    data_out.y <= std_logic_vector(s_master_y);
    data_out.u <= std_logic_vector(s_master_u);
    data_out.v <= std_logic_vector(s_master_v);

    -- =========================================================================
    -- Sync delay: 16-deep shift register for all sync signals and clean bypass.
    -- Provides all delay taps and drives data_out sync ports.
    -- (Pipeline grew 14 → 16 to accommodate the Option D chroma rotation.)
    -- =========================================================================
    p_sync_delay : process(clk)
        type t_sync_dly  is array(0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic;
        type t_data_dly  is array(0 to C_PROCESSING_DELAY_CLKS - 1)
                            of std_logic_vector(9 downto 0);
        variable v_avid  : t_sync_dly := (others => '0');
        variable v_hsync : t_sync_dly := (others => '1');
        variable v_vsync : t_sync_dly := (others => '1');
        variable v_field : t_sync_dly := (others => '1');
        variable v_y     : t_data_dly := (others => (others => '0'));
        variable v_u     : t_data_dly := (others => (others => '0'));
        variable v_v     : t_data_dly := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_avid  := data_in.avid    & v_avid(0  to C_PROCESSING_DELAY_CLKS - 2);
            v_hsync := data_in.hsync_n & v_hsync(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_vsync := data_in.vsync_n & v_vsync(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_field := data_in.field_n & v_field(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_y     := data_in.y       & v_y(0    to C_PROCESSING_DELAY_CLKS - 2);
            v_u     := data_in.u       & v_u(0    to C_PROCESSING_DELAY_CLKS - 2);
            v_v     := data_in.v       & v_v(0    to C_PROCESSING_DELAY_CLKS - 2);

            -- Sync outputs at full depth (index 15 = 16 effective clocks)
            data_out.avid    <= v_avid(C_PROCESSING_DELAY_CLKS - 1);
            data_out.hsync_n <= v_hsync(C_PROCESSING_DELAY_CLKS - 1);
            data_out.vsync_n <= v_vsync(C_PROCESSING_DELAY_CLKS - 1);
            data_out.field_n <= v_field(C_PROCESSING_DELAY_CLKS - 1);

            -- 3-clock tap: smear IIR filter enable (index 2 = 3 effective clocks)
            s_avid_smear_en <= v_avid(2);

            -- 7-clock tap: ringing interpolator enable (index 6 = 7 effective clocks)
            -- Was 5 clocks; +2 for the sin/cos LUT lookup + rotation registers.
            s_avid_d7 <= v_avid(6);

            -- 12-clock tap: feedback + master mix enables, clean bypass
            -- (index 11 = 12 effective clocks; was 10, +2 for the rotation pipeline)
            s_avid_d12    <= v_avid(11);
            s_clean_y_d12 <= v_y(11);
            s_clean_u_d12 <= v_u(11);
            s_clean_v_d12 <= v_v(11);
        end if;
    end process;

    -- =========================================================================
    -- Write-back address: s_hcount delayed C_WR_PIPE_LEN=15 clocks.
    -- 15-element variable array → 15 effective clocks.
    -- (Was 13; +2 to track the master-mix output's new clock 16 position.)
    -- =========================================================================
    p_wr_addr : process(clk)
        type t_addr_pipe is array(0 to C_WR_PIPE_LEN - 1) of unsigned(10 downto 0);
        variable v_pipe : t_addr_pipe := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_pipe            := s_hcount & v_pipe(0 to C_WR_PIPE_LEN - 2);
            s_wr_addr_delayed <= v_pipe(C_WR_PIPE_LEN - 1);
        end if;
    end process;

end architecture bent_topology;
