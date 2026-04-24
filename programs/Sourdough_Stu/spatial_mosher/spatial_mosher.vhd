-- Spatial Mosher
-- Copyright (C) 2025 LZX Industries LLC
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Circuit-bent geometry and datamoshing emulator.
-- Pipeline depth: C_PROCESSING_DELAY_CLKS = 17
-- BRAM: 30 EBR (3 × video_line_buffer, G_DEPTH=11 = 10 EBR each)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_stream_pkg.all;   -- t_video_stream_yuv444_30b
use work.core_pkg.all;           -- t_spi_ram
use work.video_timing_pkg.all;   -- t_video_timing_port
use work.clamp_pkg.all;          -- fn_clamp_s_to_u, fn_clamp_u, fn_clamp_int_to_u

architecture spatial_mosher of program_top is

    constant C_PROCESSING_DELAY_CLKS : integer := 17;
    constant C_WR_PIPE_LEN           : integer := 13;

    -- Timing infrastructure
    signal s_timing          : t_video_timing_port;
    signal s_line_bank       : std_logic := '0';
    signal s_hcount          : unsigned(10 downto 0) := (others => '0');

    -- Toggle aliases (bit-packed from registers_in(6))
    signal s_scramble_target : std_logic;
    signal s_chaos_gate      : std_logic;
    signal s_polarity        : std_logic;
    signal s_interleave      : std_logic;
    signal s_filter_mode     : std_logic;

    -- Sync-delay taps driven inside p_sync_delay
    signal s_avid_d          : std_logic;  -- 3-clock delayed avid (decay enable)
    signal s_avid_d_interp   : std_logic;  -- 13-clock delayed avid (interpolator enable)
    signal s_data_in_d2_y    : std_logic_vector(9 downto 0);   -- 3-clock (motion seeding)
    signal s_data_in_d13_y   : std_logic_vector(9 downto 0);   -- 13-clock (interp i_a dry path)
    signal s_data_in_d13_u   : std_logic_vector(9 downto 0);
    signal s_data_in_d13_v   : std_logic_vector(9 downto 0);

    -- Address pipeline
    signal s_scramble_offset   : unsigned(10 downto 0);
    signal s_rd_addr_y         : unsigned(10 downto 0);  -- registered before BRAM
    signal s_rd_addr_u         : unsigned(10 downto 0);
    signal s_rd_addr_v         : unsigned(10 downto 0);
    signal s_rd_addr_scrambled : unsigned(10 downto 0);
    signal s_chroma_frac       : unsigned(7 downto 0);
    signal s_wr_addr_delayed   : unsigned(10 downto 0);

    -- LFSR / noise
    signal s_lfsr_q          : std_logic_vector(15 downto 0);
    signal s_noise_amplitude  : unsigned(9 downto 0);
    signal s_noise_en         : std_logic;
    signal s_noise_masked     : unsigned(9 downto 0);

    -- Motion seeding
    signal s_luma_diff        : unsigned(10 downto 0);
    signal s_motion_thresh    : unsigned(7 downto 0);
    signal s_motion_en        : std_logic;
    signal s_motion_boost     : unsigned(10 downto 0);

    -- Frame drift
    signal s_frame_phase      : unsigned(15 downto 0);

    -- Line buffer outputs (std_logic_vector — matches video_line_buffer o_data)
    signal s_prev_y           : std_logic_vector(9 downto 0);
    signal s_prev_u           : std_logic_vector(9 downto 0);
    signal s_prev_v           : std_logic_vector(9 downto 0);

    -- IIR filter outputs (signed(10 downto 0) — G_WIDTH=11)
    signal s_filtered_y       : signed(10 downto 0);
    signal s_filtered_u       : signed(10 downto 0);
    signal s_filtered_v       : signed(10 downto 0);
    signal s_filter_coeff     : unsigned(7 downto 0);

    -- Freeze-during-blanking feed for IIR filters (R7: prevents filter→multiplier→BRAM
    -- self-feedback loop from collapsing to zero over long V-blank intervals).
    signal s_filter_y_feed    : signed(10 downto 0);
    signal s_filter_u_feed    : signed(10 downto 0);
    signal s_filter_v_feed    : signed(10 downto 0);

    -- Decay multiply outputs (signed(10 downto 0) — G_WIDTH=11, result already scaled/clamped)
    signal s_decayed_y        : signed(10 downto 0);
    signal s_decayed_u        : signed(10 downto 0);
    signal s_decayed_v        : signed(10 downto 0);

    -- S&H state
    signal s_sh_counter       : unsigned(10 downto 0) := (others => '0');  -- 11-bit: max threshold 2046
    signal s_sh_threshold     : unsigned(10 downto 0);
    signal s_is_held          : std_logic := '0';
    signal s_held_y           : std_logic_vector(9 downto 0) := (others => '0');
    signal s_held_u           : std_logic_vector(9 downto 0) := (others => '0');
    signal s_held_v           : std_logic_vector(9 downto 0) := (others => '0');
    signal s_sh_output_y      : std_logic_vector(9 downto 0);
    signal s_sh_output_u      : std_logic_vector(9 downto 0);
    signal s_sh_output_v      : std_logic_vector(9 downto 0);
    signal s_sh_output_u_final: std_logic_vector(9 downto 0);
    signal s_sh_output_v_final: std_logic_vector(9 downto 0);

    -- Grid Mod bit-shift
    signal s_grid_mod_shift   : integer range 0 to 15;
    signal s_luma_scaled      : unsigned(9 downto 0);

    -- Hue rotation (sin_cos LUT — combinational)
    signal s_hue_sin          : signed(9 downto 0);
    signal s_hue_cos          : signed(9 downto 0);
    signal s_sin_final        : signed(9 downto 0);
    signal s_cos_final        : signed(9 downto 0);

    -- Module B alignment (11-clock delay to reach clock 12)
    signal s_sh_aligned_y     : std_logic_vector(9 downto 0);
    signal s_sh_aligned_u     : std_logic_vector(9 downto 0);
    signal s_sh_aligned_v     : std_logic_vector(9 downto 0);
    signal s_is_held_aligned  : std_logic;

    -- Blend + balance output (registered at clock 13 — write-back tap)
    signal s_final_y          : unsigned(9 downto 0);
    signal s_final_u          : unsigned(9 downto 0);
    signal s_final_v          : unsigned(9 downto 0);

    -- Write-back (std_logic_vector — matches video_line_buffer i_data)
    signal s_writeback_y      : std_logic_vector(9 downto 0);
    signal s_writeback_u      : std_logic_vector(9 downto 0);
    signal s_writeback_v      : std_logic_vector(9 downto 0);

    -- Interleave MUX outputs
    signal s_mux_y            : unsigned(9 downto 0);
    signal s_mux_u            : unsigned(9 downto 0);
    signal s_mux_v            : unsigned(9 downto 0);

    -- Interpolator outputs
    signal s_interp_y         : unsigned(9 downto 0);
    signal s_interp_u         : unsigned(9 downto 0);
    signal s_interp_v         : unsigned(9 downto 0);

begin

    -- Toggle extraction (concurrent, from bit-packed registers_in(6))
    s_scramble_target <= registers_in(6)(0);
    s_chaos_gate      <= registers_in(6)(1);
    s_polarity        <= registers_in(6)(2);
    s_interleave      <= registers_in(6)(3);
    s_filter_mode     <= registers_in(6)(4);

    -- Write-back connections: tapped from s_final at clock 13 (before MUX and interpolator)
    s_writeback_y <= std_logic_vector(s_final_y);
    s_writeback_u <= std_logic_vector(s_final_u);
    s_writeback_v <= std_logic_vector(s_final_v);

    -- Module D: Timing generator (supplies hsync_start for bank toggle)
    timing_gen : entity work.video_timing_generator
        port map(
            clk         => clk,
            ref_hsync_n => data_in.hsync_n,
            ref_vsync_n => data_in.vsync_n,
            ref_avid    => data_in.avid,
            timing      => s_timing
        );

    -- Bank toggle: one flip-flop, shared between line buffer i_ab and interleave MUX
    -- MUST initialise to '0' (declared above): prevents 'U' state on first power-up
    process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                s_line_bank <= not s_line_bank;
            end if;
        end if;
    end process;

    -- Pixel counter: s_hcount = 0 at first active pixel, increments each avid clock
    -- Derived directly from data_in.avid (NOT s_timing.avid — avoids 1-2 clock phase offset)
    process(clk)
    begin
        if rising_edge(clk) then
            if data_in.avid = '0' then
                s_hcount <= (others => '0');
            else
                s_hcount <= s_hcount + 1;
            end if;
        end if;
    end process;

    -- Module C: LFSR chaos source
    -- Output port is q (not lfsr_out). Default seed x"ACE1" built in; load='0' ignores seed input.
    chaos_gen : entity work.lfsr16
        port map(
            clk    => clk,
            enable => '1',
            load   => '0',
            seed   => x"0000",
            q      => s_lfsr_q
        );

    -- Module F: DDS-style phase accumulator. Ports: vsync_n, enable, speed, phase.
    -- At Chaos=0: speed=0 → phase frozen (no drift). At Chaos=1023: fast drift.
    drift_phase : entity work.frame_phase_accumulator
        generic map(G_PHASE_WIDTH => 16, G_SPEED_WIDTH => 10)
        port map(
            clk     => clk,
            vsync_n => data_in.vsync_n,
            enable  => '1',
            speed   => unsigned(registers_in(4)),
            phase   => s_frame_phase
        );

    -- Noise amplitude: chaos[9:7] selects how many LFSR bits contribute (0→1px, 7→1023px)
    -- Both amplitude AND probability scale together — small and rare at low chaos.
    with unsigned(registers_in(4)(9 downto 7)) select s_noise_amplitude <=
        resize(unsigned(s_lfsr_q(0 downto 0)), 10)  when "000",   -- 0–1 px
        resize(unsigned(s_lfsr_q(3 downto 0)), 10)  when "001",   -- 0–15 px
        resize(unsigned(s_lfsr_q(5 downto 0)), 10)  when "010",   -- 0–63 px
        resize(unsigned(s_lfsr_q(7 downto 0)), 10)  when "011",   -- 0–255 px
        unsigned(s_lfsr_q(9 downto 0))              when others;  -- 0–1023 px

    -- Probability gate: noise fires when lfsr[9:0] < chaos. True null at chaos=0.
    -- MUST cast both sides to unsigned — std_logic_vector comparison is invalid VHDL.
    s_noise_en <= '1' when (unsigned(s_lfsr_q(9 downto 0)) < unsigned(registers_in(4)))
             else '0';

    -- Chaos Gate toggle: '1'=free-running, '0'=bright-areas-only
    s_noise_masked <= s_noise_amplitude
                      when (s_noise_en = '1' and s_chaos_gate = '1')
                 else s_noise_amplitude and (unsigned(data_in.y(9 downto 2)) & to_unsigned(0, 2))
                      when (s_noise_en = '1' and s_chaos_gate = '0')
                 else (others => '0');

    -- Motion boost: 1-clock registered to avoid combinational loop through address path.
    -- 1-pixel lag is visually imperceptible at video rates.
    process(clk)
    begin
        if rising_edge(clk) then
            s_motion_boost <= to_unsigned(64, 11) when s_motion_en = '1'
                         else (others => '0');
        end if;
    end process;

    -- Full scramble offset: wraps modulo 2048 intentionally (glitch aesthetic).
    -- All inputs are registered signals, so this combinational addition is fast.
    s_scramble_offset <= resize(unsigned(registers_in(0)), 11)  -- Scramble knob
                       + resize(s_noise_masked, 11)              -- LFSR noise
                       + resize(s_frame_phase(10 downto 0), 11) -- drift
                       + s_motion_boost;                         -- motion seed boost

    -- Polarity toggle: look ahead (+) or look behind (-) in previous line
    s_rd_addr_scrambled <= s_hcount + s_scramble_offset when s_polarity = '1'
                      else s_hcount - s_scramble_offset;

    -- Scramble Target toggle: which channel set gets displaced address
    -- Y vs UV selection; chroma gets independent ±1/8 offset for colour fringing
    s_chroma_frac <= s_scramble_offset(10 downto 3);  -- 8-bit, 1/8 of Y offset
    -- Note: s_rd_addr_y / s_rd_addr_u / s_rd_addr_v are registered 1 clock (below)
    -- so this combinational computation drives a register, not directly the BRAM.
    process(clk)
    begin
        if rising_edge(clk) then
            if s_scramble_target = '1' then
                s_rd_addr_y <= s_rd_addr_scrambled;
                s_rd_addr_u <= s_hcount - resize(s_chroma_frac, 11);  -- leads Y
                s_rd_addr_v <= s_hcount + resize(s_chroma_frac, 11);  -- lags Y
            else
                s_rd_addr_y <= s_hcount;
                s_rd_addr_u <= s_rd_addr_scrambled - resize(s_chroma_frac, 11);
                s_rd_addr_v <= s_rd_addr_scrambled + resize(s_chroma_frac, 11);
            end if;
        end if;
    end process;

    -- Motion seeding: compare current line vs previous line at (approximately) same position.
    -- s_data_in_d2_y is data_in.y delayed 3 clocks (v_y_clean(2)) to match BRAM output timing.
    -- s_motion_thresh: inverted 8-bit from Chaos. Chaos=0 → thresh=255 (silent). Chaos=max → thresh=0.
    s_luma_diff    <= unsigned(abs(signed('0' & s_prev_y) - signed('0' & s_data_in_d2_y)));
    s_motion_thresh <= not unsigned(registers_in(4)(9 downto 2));
    s_motion_en    <= '1' when (s_luma_diff(9 downto 2) > s_motion_thresh) else '0';

    -- Each line buffer: G_DEPTH=11 → 2048 pixels → 10 EBR. Three instances = 30 EBR total.
    -- i_wr_addr and i_rd_addr are unsigned(10 downto 0) — matches G_DEPTH-1 downto 0.
    lb_y : entity work.video_line_buffer
        generic map(G_WIDTH => 10, G_DEPTH => 11)
        port map(
            clk       => clk,
            i_ab      => s_line_bank,
            i_wr_addr => s_wr_addr_delayed,
            i_rd_addr => s_rd_addr_y,
            i_data    => s_writeback_y,
            o_data    => s_prev_y
        );

    lb_u : entity work.video_line_buffer
        generic map(G_WIDTH => 10, G_DEPTH => 11)
        port map(
            clk       => clk,
            i_ab      => s_line_bank,
            i_wr_addr => s_wr_addr_delayed,
            i_rd_addr => s_rd_addr_u,
            i_data    => s_writeback_u,
            o_data    => s_prev_u
        );

    lb_v : entity work.video_line_buffer
        generic map(G_WIDTH => 10, G_DEPTH => 11)
        port map(
            clk       => clk,
            i_ab      => s_line_bank,
            i_wr_addr => s_wr_addr_delayed,
            i_rd_addr => s_rd_addr_v,
            i_data    => s_writeback_v,
            o_data    => s_prev_v
        );

    -- Filter coefficient: cutoff upper nibble = shift amount.
    -- Higher cutoff → larger shift → slower IIR → softer colour transition at tears.
    -- Soft (Toggle5='0'): to_unsigned(192,8) → v_k=12 (very gradual)
    -- Hard (Toggle5='1'): to_unsigned(32,8)  → v_k=2  (fast snap)
    -- Soft: cutoff=128 → v_k=8 (err/256 per clock — genuinely soft, ~½-line convergence).
    -- Hard: cutoff=32  → v_k=2 (err/4 per clock — fast snap at tear edges).
    -- Earlier spec value 192 (v_k=12) was inert for 10-bit data: integer shift of
    -- any err < 4096 truncates to 0, so filter state never updated. See
    -- `_meta/found_issues.md` entry "Soft Filter Mode numerically inert".
    s_filter_coeff <= to_unsigned(128, 8) when s_filter_mode = '0'
                 else to_unsigned(32, 8);

    -- Feed mux with two roles:
    --   1. Freeze-during-blanking guard (R7): self-feedback when s_avid_d='0' so
    --      filter state holds during H/V blank intervals.
    --   2. Metavalue sanitisation (GHDL-sim only): `to_01(..., '0')` maps any
    --      undefined 'U'/'X' bits from uninitialised BRAM to '0'. Without this,
    --      the first BRAM read (before any write has landed) injects 'U' into
    --      the filter state and contaminates s_y_reg permanently — all subsequent
    --      arithmetic produces 'X', which shows as 0 in the captured output.
    --      On real iCE40 hardware BRAM initialises to 0 by default, so this is
    --      a simulation-only guard. Harmless on hardware (extra mask gate).
    s_filter_y_feed <= signed('0' & to_01(unsigned(s_prev_y), '0')) when s_avid_d = '1'
                  else s_filtered_y;
    s_filter_u_feed <= signed('0' & to_01(unsigned(s_prev_u), '0')) when s_avid_d = '1'
                  else s_filtered_u;
    s_filter_v_feed <= signed('0' & to_01(unsigned(s_prev_v), '0')) when s_avid_d = '1'
                  else s_filtered_v;

    -- Three instances — one per channel. Input: zero-extended 11-bit signed via feed mux.
    -- Output low_pass: signed(10 downto 0), pixel value in bits [9:0].
    -- 0-clock contribution to pipeline (combinational from IIR state register).
    filter_y : entity work.variable_filter_s
        generic map(G_WIDTH => 11)
        port map(
            clk      => clk,
            enable   => s_avid_d,
            a        => s_filter_y_feed,
            cutoff   => s_filter_coeff,
            low_pass => s_filtered_y,
            high_pass => open,
            valid    => open
        );

    filter_u : entity work.variable_filter_s
        generic map(G_WIDTH => 11)
        port map(
            clk      => clk,
            enable   => s_avid_d,
            a        => s_filter_u_feed,
            cutoff   => s_filter_coeff,
            low_pass => s_filtered_u,
            high_pass => open,
            valid    => open
        );

    filter_v : entity work.variable_filter_s
        generic map(G_WIDTH => 11)
        port map(
            clk      => clk,
            enable   => s_avid_d,
            a        => s_filter_v_feed,
            cutoff   => s_filter_coeff,
            low_pass => s_filtered_v,
            high_pass => open,
            valid    => open
        );

    -- Decay: scale filtered previous-line pixel by Knob 2 Feedback.
    -- G_WIDTH=11: x and y are signed(10 downto 0); result is signed(10 downto 0), already scaled.
    -- No bit-slicing of result — G_OUTPUT_MIN/MAX do the clamping to pixel range.
    -- s_filtered_y is signed(10 downto 0) — feed directly as x.
    -- registers_in(1) is std_logic_vector(9 downto 0) → '0' & unsigned(.) = signed(10 downto 0) for y.
    decay_y : entity work.multiplier_s
        generic map(G_WIDTH => 11, G_FRAC_BITS => 10, G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(
            clk    => clk,
            enable => s_avid_d,
            x      => s_filtered_y,
            y      => signed('0' & unsigned(registers_in(1))),
            z      => (others => '0'),
            result => s_decayed_y,
            valid  => open
        );

    decay_u : entity work.multiplier_s
        generic map(G_WIDTH => 11, G_FRAC_BITS => 10, G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(
            clk    => clk,
            enable => s_avid_d,
            x      => s_filtered_u,
            y      => signed('0' & unsigned(registers_in(1))),
            z      => (others => '0'),
            result => s_decayed_u,
            valid  => open
        );

    decay_v : entity work.multiplier_s
        generic map(G_WIDTH => 11, G_FRAC_BITS => 10, G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(
            clk    => clk,
            enable => s_avid_d,
            x      => s_filtered_v,
            y      => signed('0' & unsigned(registers_in(1))),
            z      => (others => '0'),
            result => s_decayed_v,
            valid  => open
        );

    -- Grid Mod: bit-shift replaces multiplier (~450 LC saved). 16 discrete levels.
    -- Inverted top nibble: knob=0 → shift=15 (≈zero luma contribution, threshold = Knob3 only)
    --                      knob=1023 → shift=0 (full luma contribution)
    s_grid_mod_shift <= 15 - to_integer(unsigned(registers_in(3)(9 downto 6)));
    s_luma_scaled    <= shift_right(unsigned(data_in.y), s_grid_mod_shift);

    -- S&H threshold: 11-bit sum (two 10-bit values → max 2046)
    s_sh_threshold <= resize(unsigned(registers_in(2)), 11) + resize(s_luma_scaled, 11);

    -- S&H counter: MUST be 11-bit — threshold can reach 2046, which wraps a 10-bit counter.
    -- Counter resets each blanking interval and on threshold expiry.
    process(clk)
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                if s_sh_counter >= s_sh_threshold then
                    s_held_y     <= data_in.y;
                    s_held_u     <= data_in.u;
                    s_held_v     <= data_in.v;
                    s_sh_counter <= (others => '0');
                    s_is_held    <= '0';
                else
                    s_sh_counter <= s_sh_counter + 1;
                    s_is_held    <= '1';
                end if;
            else
                s_sh_counter <= (others => '0');
                s_is_held    <= '0';
            end if;
        end if;
    end process;

    -- S&H output MUX: frozen held pixel or live input
    s_sh_output_y <= s_held_y when s_is_held = '1' else data_in.y;
    s_sh_output_u <= s_held_u when s_is_held = '1' else data_in.u;
    s_sh_output_v <= s_held_v when s_is_held = '1' else data_in.v;

    -- Hue LUT: fully combinational (no clock port). angle_in is std_logic_vector(9 downto 0).
    -- sin_out / cos_out are signed(9 downto 0), range -511 to +511.
    lut_sincos : entity work.sin_cos_full_lut_10x10
        port map(
            angle_in => std_logic_vector(s_frame_phase(9 downto 0)),
            sin_out  => s_hue_sin,
            cos_out  => s_hue_cos
        );

    -- 4-level magnitude from chaos[9:8]. "00" = HARD ZERO — guaranteed null at chaos=0.
    -- Must use process (not with...select) because two outputs are assigned simultaneously.
    p_hue_scale : process(registers_in(4), s_hue_sin, s_hue_cos)
    begin
        case unsigned(registers_in(4)(9 downto 8)) is
            when "00" =>
                s_sin_final <= (others => '0');   -- hard zero regardless of phase
                s_cos_final <= (others => '0');
            when "01" =>
                s_sin_final <= resize(signed(s_hue_sin(9 downto 4)), 10);  -- subtle >>4
                s_cos_final <= resize(signed(s_hue_cos(9 downto 4)), 10);
            when "10" =>
                s_sin_final <= resize(signed(s_hue_sin(9 downto 3)), 10);  -- medium >>3
                s_cos_final <= resize(signed(s_hue_cos(9 downto 3)), 10);
            when others =>
                s_sin_final <= resize(signed(s_hue_sin(9 downto 2)), 10);  -- full >>2
                s_cos_final <= resize(signed(s_hue_cos(9 downto 2)), 10);
        end case;
    end process;

    -- Apply hue offset to UV of held pixels only. Y unchanged (brightness preserved).
    -- fn_clamp_int_to_u: use work.clamp_pkg.all (context clause, not here).
    s_sh_output_u_final <= std_logic_vector(fn_clamp_int_to_u(
        to_integer(unsigned(s_sh_output_u)) + to_integer(s_sin_final), 10))
        when s_is_held = '1' else s_sh_output_u;
    s_sh_output_v_final <= std_logic_vector(fn_clamp_int_to_u(
        to_integer(unsigned(s_sh_output_v)) + to_integer(s_cos_final), 10))
        when s_is_held = '1' else s_sh_output_v;

    -- S&H output ready at clock 1. Decay ready at clock 12. Delay S&H by 11 clocks.
    -- 11-element variable array (indices 0 to 10) + signal assignment = 11 effective clocks.
    process(clk)
        type t_sh_pipe is array(0 to 10) of std_logic_vector(9 downto 0);
        variable v_sh_y : t_sh_pipe := (others => (others => '0'));
        variable v_sh_u : t_sh_pipe := (others => (others => '0'));
        variable v_sh_v : t_sh_pipe := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_sh_y := s_sh_output_y       & v_sh_y(0 to 9);
            v_sh_u := s_sh_output_u_final & v_sh_u(0 to 9);
            v_sh_v := s_sh_output_v_final & v_sh_v(0 to 9);
            s_sh_aligned_y <= v_sh_y(10);
            s_sh_aligned_u <= v_sh_u(10);
            s_sh_aligned_v <= v_sh_v(10);
        end if;
    end process;

    -- s_is_held also delayed 11 clocks to arrive at the blend+balance stage (clock 12→13)
    process(clk)
        type t_held_pipe is array(0 to 10) of std_logic;
        variable v_held : t_held_pipe := (others => '0');
    begin
        if rising_edge(clk) then
            v_held           := s_is_held & v_held(0 to 9);
            s_is_held_aligned <= v_held(10);
        end if;
    end process;

    -- Combine decay (Module A, clock 12) + S&H aligned (Module B, clock 12).
    -- Apply balance darkening via saturating subtraction (replaces 3 × multiplier_s).
    -- s_decayed_y is signed(10 downto 0), clamped 0–1023 by multiplier_s G_OUTPUT_MIN/MAX.
    -- All three channels must be treated consistently to avoid colour-shift artifact on held pixels.
    process(clk)
        variable v_blend_y, v_blend_u, v_blend_v : integer;
        variable v_balance : integer;
    begin
        if rising_edge(clk) then
            v_balance := to_integer(unsigned(registers_in(5)));  -- Knob 6 Balance

            -- Blend: decay result + S&H aligned output (sum up to 2046, clamp to 1023)
            v_blend_y := to_integer(s_decayed_y) + to_integer(unsigned(s_sh_aligned_y));
            v_blend_u := to_integer(s_decayed_u) + to_integer(unsigned(s_sh_aligned_u));
            v_blend_v := to_integer(s_decayed_v) + to_integer(unsigned(s_sh_aligned_v));

            if s_is_held_aligned = '1' then
                -- Balance: subtract from held pixels only (saturates at 0 — no wrap)
                s_final_y <= fn_clamp_int_to_u(v_blend_y - v_balance, 10);
                s_final_u <= fn_clamp_int_to_u(v_blend_u - v_balance, 10);
                s_final_v <= fn_clamp_int_to_u(v_blend_v - v_balance, 10);
            else
                s_final_y <= fn_clamp_int_to_u(v_blend_y, 10);
                s_final_u <= fn_clamp_int_to_u(v_blend_u, 10);
                s_final_v <= fn_clamp_int_to_u(v_blend_v, 10);
            end if;
        end if;
    end process;

    -- Bypass path: s_data_in_d13_y is 13-clock delayed clean data (from p_sync_delay).
    -- When Interleave='1': all lines processed. When '0': odd lines pass clean through.
    -- s_line_bank toggles each hsync — '0' = even line, '1' = odd line (by convention).
    s_mux_y <= s_final_y                       when (s_interleave = '1' or s_line_bank = '0')
          else unsigned(s_data_in_d13_y);
    s_mux_u <= s_final_u                       when (s_interleave = '1' or s_line_bank = '0')
          else unsigned(s_data_in_d13_u);
    s_mux_v <= s_final_v                       when (s_interleave = '1' or s_line_bank = '0')
          else unsigned(s_data_in_d13_v);

    -- i_a = dry (clean input at 13-clock tap), i_b = wet (processed), t = Fader knob.
    -- Interpolator ports: a, b (unsigned), t (unsigned G_FRAC_BITS bits), result (unsigned).
    -- G_FRAC_BITS=10: t=0 → result=a (full dry); t=1023 → result≈b (full wet).
    interp_y : entity work.interpolator_u
        generic map(G_WIDTH => 10, G_FRAC_BITS => 10, G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(
            clk    => clk,
            enable => s_avid_d_interp,
            a      => unsigned(s_data_in_d13_y),
            b      => s_mux_y,
            t      => unsigned(registers_in(7)),
            result => s_interp_y,
            valid  => open
        );

    interp_u : entity work.interpolator_u
        generic map(G_WIDTH => 10, G_FRAC_BITS => 10, G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(
            clk    => clk,
            enable => s_avid_d_interp,
            a      => unsigned(s_data_in_d13_u),
            b      => s_mux_u,
            t      => unsigned(registers_in(7)),
            result => s_interp_u,
            valid  => open
        );

    interp_v : entity work.interpolator_u
        generic map(G_WIDTH => 10, G_FRAC_BITS => 10, G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(
            clk    => clk,
            enable => s_avid_d_interp,
            a      => unsigned(s_data_in_d13_v),
            b      => s_mux_v,
            t      => unsigned(registers_in(7)),
            result => s_interp_v,
            valid  => open
        );

    -- Pixel data outputs — driven only here.
    -- R1 (hardware-constraints §Constraint 7): clamp to BT.601 broadcast-safe range
    -- before the port assignment. Y ∈ [64, 940], U/V ∈ [64, 960] in 10-bit. This
    -- protects the ADV7391 encoder and any downstream recorder/display from
    -- super-black / super-white / illegal chroma values.
    data_out.y <= std_logic_vector(to_unsigned(64,  10)) when s_interp_y < to_unsigned(64,  10) else
                  std_logic_vector(to_unsigned(940, 10)) when s_interp_y > to_unsigned(940, 10) else
                  std_logic_vector(s_interp_y);
    data_out.u <= std_logic_vector(to_unsigned(64,  10)) when s_interp_u < to_unsigned(64,  10) else
                  std_logic_vector(to_unsigned(960, 10)) when s_interp_u > to_unsigned(960, 10) else
                  std_logic_vector(s_interp_u);
    data_out.v <= std_logic_vector(to_unsigned(64,  10)) when s_interp_v < to_unsigned(64,  10) else
                  std_logic_vector(to_unsigned(960, 10)) when s_interp_v > to_unsigned(960, 10) else
                  std_logic_vector(s_interp_v);

    -- Sync delay: C_PROCESSING_DELAY_CLKS=17 entries. Sync outputs at index 16.
    -- Also provides: 3-clock avid (v_avid(2)), 13-clock avid (v_avid(12)),
    --               3-clock data_in.y (v_y_clean(2)), 13-clock data (v_y_clean(12)).
    p_sync_delay : process(clk)
        type t_sync_delay is array(0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic;
        type t_data_delay is array(0 to C_PROCESSING_DELAY_CLKS - 1)
            of std_logic_vector(9 downto 0);
        variable v_avid    : t_sync_delay := (others => '0');
        variable v_hsync   : t_sync_delay := (others => '1');
        variable v_vsync   : t_sync_delay := (others => '1');
        variable v_field   : t_sync_delay := (others => '1');
        variable v_y_clean : t_data_delay := (others => (others => '0'));
        variable v_u_clean : t_data_delay := (others => (others => '0'));
        variable v_v_clean : t_data_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_avid    := data_in.avid    & v_avid(0  to C_PROCESSING_DELAY_CLKS - 2);
            v_hsync   := data_in.hsync_n & v_hsync(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_vsync   := data_in.vsync_n & v_vsync(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_field   := data_in.field_n & v_field(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_y_clean := data_in.y & v_y_clean(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_u_clean := data_in.u & v_u_clean(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_v_clean := data_in.v & v_v_clean(0 to C_PROCESSING_DELAY_CLKS - 2);

            -- Sync outputs: full 17-clock depth (index 16)
            data_out.avid    <= v_avid(C_PROCESSING_DELAY_CLKS - 1);
            data_out.hsync_n <= v_hsync(C_PROCESSING_DELAY_CLKS - 1);
            data_out.vsync_n <= v_vsync(C_PROCESSING_DELAY_CLKS - 1);
            data_out.field_n <= v_field(C_PROCESSING_DELAY_CLKS - 1);

            -- 3-clock tap: motion seeding (aligns with BRAM output at clock 3)
            -- index 2 = 3 effective clocks (v_y_clean(0)=1 clk, (1)=2 clks, (2)=3 clks)
            s_data_in_d2_y <= v_y_clean(2);

            -- 13-clock tap: clean bypass for interleave MUX and interpolator i_a
            -- index 12 = 13 effective clocks (C_PROCESSING_DELAY_CLKS - 4 interpolator = 13)
            s_data_in_d13_y <= v_y_clean(12);
            s_data_in_d13_u <= v_u_clean(12);
            s_data_in_d13_v <= v_v_clean(12);

            -- avid taps for module enables
            -- 1 addr + 2 BRAM = 3 clocks → decay multiplier enable
            s_avid_d       <= v_avid(2);   -- index 2 = 3 effective clocks
            -- 13 clocks → interpolator enable (MUX output is combinational at clock 13)
            s_avid_d_interp <= v_avid(12);  -- index 12 = 13 effective clocks
        end if;
    end process;

    -- Write-back address: delay s_hcount by C_WR_PIPE_LEN=13 clocks.
    -- At clock T+13, delivers s_hcount from clock T — matching pixel now exiting the blend+balance register.
    -- 13-element variable array (indices 0 to 12) + signal = 13 effective clocks.
    process(clk)
        type t_addr_pipe is array(0 to C_WR_PIPE_LEN - 1) of unsigned(10 downto 0);
        variable v_pipe : t_addr_pipe := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_pipe            := s_hcount & v_pipe(0 to C_WR_PIPE_LEN - 2);
            s_wr_addr_delayed <= v_pipe(C_WR_PIPE_LEN - 1);
        end if;
    end process;

end architecture spatial_mosher;
