-- Bent Topology
-- Copyright (C) 2026 Alex Ball
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Multi-stage circuit-bent glitch suite.
-- Pipeline depth: C_PROCESSING_DELAY_CLKS = 14
-- BRAM: ~12 EBR (1 × video_line_buffer G_DEPTH=11 = 10 EBR; ringing LUT = 2 EBR)

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

    constant C_PROCESSING_DELAY_CLKS : integer := 14;
    constant C_WR_PIPE_LEN           : integer := 13;

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
    -- -------------------------------------------------------------------------
    signal s_avid_smear_en : std_logic;                      -- avid, 3 clocks
    signal s_avid_d5       : std_logic;                      -- avid, 5 clocks
    signal s_avid_d10      : std_logic;                      -- avid, 10 clocks
    signal s_clean_y_d10   : std_logic_vector(9 downto 0);   -- Y,    10 clocks
    signal s_clean_u_d10   : std_logic_vector(9 downto 0);   -- U,    10 clocks
    signal s_clean_v_d10   : std_logic_vector(9 downto 0);   -- V,    10 clocks

    -- -------------------------------------------------------------------------
    -- Write-back address
    -- -------------------------------------------------------------------------
    signal s_wr_addr_delayed : unsigned(10 downto 0);

    -- -------------------------------------------------------------------------
    -- Stage 1: Grid warp
    -- -------------------------------------------------------------------------
    signal s_y_for_warp  : std_logic_vector(9 downto 0);
    signal s_warp_shift  : integer range 0 to 15;
    signal s_displacement: unsigned(9 downto 0);
    signal s_freq_select : integer range 0 to 9;
    signal s_h_check     : unsigned(10 downto 0);
    signal s_v_check     : unsigned(10 downto 0);
    signal s_grid_h_on   : std_logic;
    signal s_grid_v_on   : std_logic;
    signal s_on_grid     : std_logic;
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
    signal s_smear_cutoff: unsigned(7 downto 0);
    -- vertical blend intermediate
    signal s_smear_v_shift : integer range 0 to 7;
    signal s_vert_blend_y  : unsigned(9 downto 0);
    -- smear output (clock 3):
    signal s_smeared_y   : std_logic_vector(9 downto 0);
    signal s_smeared_u   : std_logic_vector(9 downto 0);
    signal s_smeared_v   : std_logic_vector(9 downto 0);
    -- 2-clock delay from smear → ringing interp (clock 5):
    signal s_smeared_y_d2 : std_logic_vector(9 downto 0);
    signal s_smeared_u_d2 : std_logic_vector(9 downto 0);
    signal s_smeared_v_d2 : std_logic_vector(9 downto 0);

    -- -------------------------------------------------------------------------
    -- Stage 3: Ringing
    -- -------------------------------------------------------------------------
    signal s_ring_bram_addr  : unsigned(9 downto 0);          -- clock 4
    signal s_ring_raw_8bit   : std_logic_vector(7 downto 0);  -- clock 5
    signal s_ring_val_8bit   : std_logic_vector(7 downto 0);  -- after polarity mux
    signal s_ring_offset     : integer;                        -- -128..+127
    signal s_ring_offset_s   : integer;                        -- scaled ×4
    signal s_rung_y_full     : unsigned(9 downto 0);
    signal s_rung_u_full     : unsigned(9 downto 0);
    signal s_rung_v_full     : unsigned(9 downto 0);
    -- interpolator_u outputs (clock 9):
    signal s_rung_interp_y   : unsigned(9 downto 0);
    signal s_rung_interp_u   : unsigned(9 downto 0);
    signal s_rung_interp_v   : unsigned(9 downto 0);

    -- -------------------------------------------------------------------------
    -- Stage 4: Color crush
    -- -------------------------------------------------------------------------
    signal s_crush_shift : integer range 0 to 9;
    signal s_crush_mask  : unsigned(9 downto 0);
    signal s_crushed_y   : std_logic_vector(9 downto 0);  -- clock 10
    signal s_crushed_u   : std_logic_vector(9 downto 0);
    signal s_crushed_v   : std_logic_vector(9 downto 0);

    -- -------------------------------------------------------------------------
    -- Stage 5: Feedback interpolator + line buffer
    -- -------------------------------------------------------------------------
    signal s_feedback_y  : unsigned(9 downto 0);  -- clock 14

    -- -------------------------------------------------------------------------
    -- Stage 6: Master mix interpolators
    -- -------------------------------------------------------------------------
    signal s_master_y : unsigned(9 downto 0);  -- clock 14
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
    -- Combinational detection; registered output at clock 1.
    -- =========================================================================

    -- Luma invert (Toggle 10): complement Y before displacement
    s_y_for_warp <= not data_in.y when s_luma_invert = '1' else data_in.y;

    -- Warp Depth (Knob 1): upper 4 bits select right-shift amount.
    -- High knob → small shift → large displacement.
    s_warp_shift  <= 15 - to_integer(unsigned(registers_in(0)(9 downto 6)));
    s_displacement <= shift_right(unsigned(s_y_for_warp), s_warp_shift);

    -- Warp Frequency (Knob 2): upper 3 bits select which counter bits define grid period.
    -- Bit pair of displaced counter = "00" marks the grid zone (~25 % of period).
    s_freq_select <= to_integer(unsigned(registers_in(1)(9 downto 7)));

    -- Displaced counter checks
    s_h_check <= s_hcount + resize(s_displacement, 11);
    s_v_check <= resize(s_vcount, 11) + resize(s_displacement, 11);

    -- Grid-on flags: check the 2-bit window at the selected frequency
    s_grid_h_on <= '1' when s_h_check(s_freq_select + 1 downto s_freq_select) = "00" else '0';
    s_grid_v_on <= '1' when s_v_check(s_freq_select + 1 downto s_freq_select) = "00" else '0';

    -- Toggle 7: '0' = V-axis grid (vertical lines, bent by H), '1' = H-axis grid
    s_on_grid <= s_grid_v_on when s_warp_axis = '0' else s_grid_h_on;

    -- Grid pixel: white (Y=1023, UV=512) or pass-through
    s_grid_y <= "1111111111" when s_on_grid = '1' else data_in.y;
    s_grid_u <= "1000000000" when s_on_grid = '1' else data_in.u;
    s_grid_v <= "1000000000" when s_on_grid = '1' else data_in.v;

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

    -- Smear cutoff: Knob 3 upper 3 bits → 8 discrete IIR cutoff levels (32 = hard, 192 = soft)
    with unsigned(registers_in(2)(9 downto 7)) select s_smear_cutoff <=
        to_unsigned( 32, 8) when "000",
        to_unsigned( 55, 8) when "001",
        to_unsigned( 78, 8) when "010",
        to_unsigned(101, 8) when "011",
        to_unsigned(123, 8) when "100",
        to_unsigned(146, 8) when "101",
        to_unsigned(169, 8) when "110",
        to_unsigned(192, 8) when others;

    -- Vertical blend shift: 0 (heavy blend) = Knob 3 max; 7 (no blend) = Knob 3 zero
    s_smear_v_shift <= 7 - to_integer(unsigned(registers_in(2)(9 downto 7)));

    -- Vertical blend Y (combinational): warped_d1 + (prev_y >> shift), clamped
    s_vert_blend_y <= fn_clamp_int_to_u(
        to_integer(unsigned(s_warped_d1_y)) +
        to_integer(shift_right(unsigned(s_prev_y), s_smear_v_shift)), 10);

    -- IIR horizontal filter instances (Y, U, V)
    -- Y is used in H mode only; U/V always horizontal regardless of Toggle 8.
    smear_y : entity work.variable_filter_s
        generic map(G_WIDTH => 11)
        port map(
            clk       => clk,
            enable    => s_avid_smear_en,
            a         => signed('0' & unsigned(s_warped_d1_y)),
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
            a         => signed('0' & unsigned(s_warped_d1_u)),
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
            a         => signed('0' & unsigned(s_warped_d1_v)),
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

    -- 2-clock delay: smear output (clock 3) → ringing interpolator 'a' input (clock 5)
    p_smear_delay : process(clk)
        type t_d2 is array(0 to 1) of std_logic_vector(9 downto 0);
        variable vy : t_d2 := (others => (others => '0'));
        variable vu : t_d2 := (others => (others => '0'));
        variable vv : t_d2 := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            vy := s_smeared_y & vy(0 to 0);
            vu := s_smeared_u & vu(0 to 0);
            vv := s_smeared_v & vv(0 to 0);
            s_smeared_y_d2 <= vy(1);
            s_smeared_u_d2 <= vu(1);
            s_smeared_v_d2 <= vv(1);
        end if;
    end process;

    -- =========================================================================
    -- Stage 3: Ringing (BRAM LUT, Stairs-inspired)
    -- =========================================================================

    -- Register BRAM address at clock 4 (from s_smeared_y at clock 3)
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
    s_ring_val_8bit <= s_ring_raw_8bit when s_ring_polarity = '0'
                  else std_logic_vector(to_unsigned(255, 8) - unsigned(s_ring_raw_8bit));

    -- Scale 8-bit offset to 10-bit signed range (-512..+508)
    -- Centre of LUT (128) = zero offset; integer arithmetic, unbounded for clamp
    s_ring_offset   <= to_integer(unsigned(s_ring_val_8bit)) - 128;
    s_ring_offset_s <= s_ring_offset * 4;

    -- Compute rung values (combinational, clock 5)
    s_rung_y_full <= fn_clamp_int_to_u(
        to_integer(unsigned(s_smeared_y_d2)) + s_ring_offset_s, 10);
    s_rung_u_full <= fn_clamp_int_to_u(
        to_integer(unsigned(s_smeared_u_d2)) + s_ring_offset_s, 10);
    s_rung_v_full <= fn_clamp_int_to_u(
        to_integer(unsigned(s_smeared_v_d2)) + s_ring_offset_s, 10);

    -- Ringing interpolator Y: a=smeared (no ring), b=rung (full ring), t=Knob 4
    -- Enable at clock 5 (v_avid(4)). Output at clock 9.
    ring_interp_y : entity work.interpolator_u
        generic map(
            G_WIDTH      => 10,
            G_FRAC_BITS  => 10,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_avid_d5,
            a      => unsigned(s_smeared_y_d2),
            b      => s_rung_y_full,
            t      => unsigned(registers_in(3)),
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
            enable => s_avid_d5,
            a      => unsigned(s_smeared_u_d2),
            b      => s_rung_u_full,
            t      => unsigned(registers_in(3)),
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
            enable => s_avid_d5,
            a      => unsigned(s_smeared_v_d2),
            b      => s_rung_v_full,
            t      => unsigned(registers_in(3)),
            result => s_rung_interp_v,
            valid  => open
        );

    -- =========================================================================
    -- Stage 4: Color Crush
    -- Register output at clock 10.
    -- Chroma Isolate (Toggle 11): substitute clean UV for crushed UV.
    -- =========================================================================
    s_crush_shift <= to_integer(unsigned(registers_in(4)(9 downto 6)));

    -- Safe mask: shift_left of all-ones, clamped; shift of 0 = full pass-through
    s_crush_mask <= (others => '1') when s_crush_shift = 0
               else shift_left(to_unsigned(1023, 10), s_crush_shift) and to_unsigned(1023, 10);

    p_crush_reg : process(clk)
    begin
        if rising_edge(clk) then
            s_crushed_y <= std_logic_vector(unsigned(s_rung_interp_y) and s_crush_mask);
            if s_chroma_isolate = '1' then
                s_crushed_u <= s_clean_u_d10;
                s_crushed_v <= s_clean_v_d10;
            else
                s_crushed_u <= std_logic_vector(unsigned(s_rung_interp_u) and s_crush_mask);
                s_crushed_v <= std_logic_vector(unsigned(s_rung_interp_v) and s_crush_mask);
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Stage 5: Feedback Blend + Line Buffer (Y only)
    -- Feedback interpolator: a=clean, b=crushed, t=Knob 6.
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
            enable => s_avid_d10,
            a      => unsigned(s_clean_y_d10),
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
    -- Output at clock 14 = C_PROCESSING_DELAY_CLKS.
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
            enable => s_avid_d10,
            a      => unsigned(s_clean_y_d10),
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
            enable => s_avid_d10,
            a      => unsigned(s_clean_u_d10),
            b      => unsigned(s_crushed_u),
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
            enable => s_avid_d10,
            a      => unsigned(s_clean_v_d10),
            b      => unsigned(s_crushed_v),
            t      => unsigned(registers_in(7)),
            result => s_master_v,
            valid  => open
        );

    data_out.y <= std_logic_vector(s_master_y);
    data_out.u <= std_logic_vector(s_master_u);
    data_out.v <= std_logic_vector(s_master_v);

    -- =========================================================================
    -- Sync delay: 14-deep shift register for all sync signals and clean bypass.
    -- Provides all delay taps and drives data_out sync ports.
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

            -- Sync outputs at full depth (index 13 = 14 effective clocks)
            data_out.avid    <= v_avid(C_PROCESSING_DELAY_CLKS - 1);
            data_out.hsync_n <= v_hsync(C_PROCESSING_DELAY_CLKS - 1);
            data_out.vsync_n <= v_vsync(C_PROCESSING_DELAY_CLKS - 1);
            data_out.field_n <= v_field(C_PROCESSING_DELAY_CLKS - 1);

            -- 3-clock tap: smear IIR filter enable (index 2 = 3 effective clocks)
            s_avid_smear_en <= v_avid(2);

            -- 5-clock tap: ringing interpolator enable (index 4 = 5 effective clocks)
            s_avid_d5 <= v_avid(4);

            -- 10-clock tap: feedback + master mix enables, clean bypass
            -- (index 9 = 10 effective clocks)
            s_avid_d10    <= v_avid(9);
            s_clean_y_d10 <= v_y(9);
            s_clean_u_d10 <= v_u(9);
            s_clean_v_d10 <= v_v(9);
        end if;
    end process;

    -- =========================================================================
    -- Write-back address: s_hcount delayed C_WR_PIPE_LEN=13 clocks.
    -- 13-element variable array → 13 effective clocks.
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
