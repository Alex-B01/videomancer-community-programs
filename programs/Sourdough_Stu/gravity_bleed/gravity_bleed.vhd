-- Gravity Bleed
-- Copyright (C) 2026 Alex Ball (Sourdough Stu)
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Organic feedback decay — wet-paint drips, thermal turbulence, crystal stalactites.
-- Y/U/V each use 2-bank line ping-pong (previous-line feedback).
--
-- Pipeline depth: C_PROCESSING_DELAY_CLKS = 10
-- BRAM: 6 banks × 2048×10b (canonical per-bank inferred RAM, no init values,
-- each with own clocked read/write process — see hardware-constraints.md
-- "BRAM inference anti-patterns"). Target ≤30 EBR / 32-EBR HX4K ceiling.
--
-- Path A passive enrichments (always-on, no controls): A1 edge gate-bias, A2 per-channel
-- chroma jitter scaled by turb amp, A3 edge-modulated k_eff for multi-band feel,
-- A4 vsync-rate triangle breathing on density LSB.
-- Path C reduced from Option D refactor (2026-04-25): Viscosity drives only IIR k
-- (decay rate) — multi-tap history depth coupling removed to fit EBR budget.
--
-- IIR is reformulated in unsigned-domain (BT.601 0..1023):
--   out = ((cur << k) - cur + prev) >> k
-- equivalent to cur + (prev-cur)/2^k but using only unsigned shifts, which truncate
-- toward 0 symmetrically. The signed form's shift_right rounded toward -inf and
-- caused state to drift toward the signed-min floor over many frames (chroma collapse
-- to BT.601 floor → green wash). The unsigned form is bias-free.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_stream_pkg.all;   -- t_video_stream_yuv444_30b
use work.core_pkg.all;           -- t_spi_ram
use work.video_timing_pkg.all;   -- t_video_timing_port
use work.clamp_pkg.all;          -- fn_clamp_int_to_u

architecture gravity_bleed of program_top is

    -- ========================================================================
    -- Pipeline & buffer constants
    -- ========================================================================
    constant C_PROCESSING_DELAY_CLKS : integer := 10;
    constant C_WB_ADDR_DELAY         : integer := 6;
    constant C_BUF_SIZE              : integer := 2048;
    constant C_BUF_DEPTH             : integer := 11;

    -- Path A1: edge magnitude above this OR-contributes to gate_pass
    -- (effective threshold scales with knob: edge > threshold>>2 fires)
    -- Path A3: edge magnitude above this decrements k_eff by 1 (sketch detail bleeds faster)
    constant C_EDGE_TH_K     : unsigned(9 downto 0) := to_unsigned(100, 10);

    -- Ice peak-hold: per-line decay constant. 4 ≈ 256-line fade (~1/4 screen at 1080p).
    constant C_ICE_CONST     : signed(10 downto 0) := to_signed(4, 11);

    -- ========================================================================
    -- Timing infrastructure
    -- ========================================================================
    signal s_timing      : t_video_timing_port;
    signal s_hcount      : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');
    signal s_line_bank   : std_logic := '0';                          -- toggles each hsync (Y/U/V ping-pong)

    -- ========================================================================
    -- Toggle aliases (registers_in(6) bit-packed)
    -- ========================================================================
    signal s_gate_invert    : std_logic;
    signal s_liquid_ice     : std_logic;
    signal s_spectral_inv   : std_logic;
    signal s_gravity_inv    : std_logic;
    signal s_freeze         : std_logic;

    -- ========================================================================
    -- Sync delay taps
    -- Filter dropped: pipeline shortened from 11→10 clocks. Interp dry-path tap
    -- moves from d7→d6.
    -- ========================================================================
    signal s_avid_d3        : std_logic;                              -- 3-clk: blend gate
    signal s_avid_d_interp  : std_logic;                              -- 6-clk: interp enable
    signal s_data_in_d3_y   : std_logic_vector(9 downto 0);           -- 3-clk: blend "cur" path
    signal s_data_in_d3_u   : std_logic_vector(9 downto 0);
    signal s_data_in_d3_v   : std_logic_vector(9 downto 0);
    signal s_data_in_d6_y   : std_logic_vector(9 downto 0);           -- 6-clk: dry path for interp
    signal s_data_in_d6_u   : std_logic_vector(9 downto 0);
    signal s_data_in_d6_v   : std_logic_vector(9 downto 0);
    signal s_edge_d3        : unsigned(9 downto 0);                   -- edge mag aligned to S4 blend

    -- ========================================================================
    -- LFSR shared source for turbulence smoothing AND chroma jitter (Path A2)
    -- ========================================================================
    signal s_lfsr_q         : std_logic_vector(15 downto 0);

    -- ========================================================================
    -- Edge detector (S0 — Path A1 + A3)
    -- ========================================================================
    signal s_y_d_prev_pix   : unsigned(9 downto 0) := (others => '0');
    signal s_edge_d0        : unsigned(9 downto 0) := (others => '0');

    -- ========================================================================
    -- Wind luma-gating (S0)
    -- ========================================================================
    signal s_wind_eff       : unsigned(9 downto 0);

    -- ========================================================================
    -- Frame-phase breathing (Path A4)
    -- ========================================================================
    signal s_breath_counter : unsigned(11 downto 0) := (others => '0');
    signal s_breath         : signed(5 downto 0);
    signal s_density_eff    : unsigned(9 downto 0);

    -- ========================================================================
    -- Turbulence generator
    -- ========================================================================
    signal s_frame_counter  : unsigned(11 downto 0) := (others => '0');
    signal s_density_mask   : unsigned(11 downto 0);
    signal s_turb_target    : signed(11 downto 0) := (others => '0');
    signal s_turb_smooth    : signed(15 downto 0) := (others => '0');  -- accumulator with headroom
    signal s_turb_y         : signed(10 downto 0);
    signal s_turb_u         : signed(10 downto 0);
    signal s_turb_v         : signed(10 downto 0);

    -- ========================================================================
    -- Address compute (S1)
    -- ========================================================================
    signal s_chroma_decouple_signed : signed(10 downto 0);            -- registers_in(5) - 512, range ±512
    signal s_chroma_frac    : signed(10 downto 0);                    -- small UV addr offset
    signal s_rd_addr_y      : unsigned(C_BUF_DEPTH - 1 downto 0);
    signal s_rd_addr_u      : unsigned(C_BUF_DEPTH - 1 downto 0);
    signal s_rd_addr_v      : unsigned(C_BUF_DEPTH - 1 downto 0);
    signal s_wb_addr        : unsigned(C_BUF_DEPTH - 1 downto 0);

    -- ========================================================================
    -- Bank selector (S1 — Y/U/V all share same line-ping-pong)
    -- ========================================================================
    signal s_k_y_base       : unsigned(2 downto 0);                   -- 1..7 (raw 0 promoted to 1)
    signal s_bank_sel       : std_logic;                              -- '0'/'1' chooses which bank to READ

    -- ========================================================================
    -- Sigma-delta dither (Viscosity LSB → ±1 to k_eff)
    -- ========================================================================
    signal s_sigma_acc      : unsigned(7 downto 0) := (others => '0');
    signal s_dither_carry   : std_logic := '0';
    signal s_k_y_dithered   : unsigned(3 downto 0);                   -- can reach 8 before clamp

    -- ========================================================================
    -- Inferred BRAM arrays — canonical iCE40 inference pattern.
    -- 6 banks (2 Y + 2 U + 2 V) × 2048 × 10b. NO init values (would block
    -- BRAM inference; bitstream defaults to zero on hardware, GHDL sim is
    -- protected by `to_01(...,'0')` wrap on s_prev_*).
    -- Each bank gets its OWN clocked read process (per-bank output register)
    -- and its OWN clocked write process (gated by wb_en + bank-select).
    -- See hardware-constraints.md "BRAM inference anti-patterns" / canonical
    -- video_line_buffer.vhd reference.
    -- ========================================================================
    type t_bram is array (0 to C_BUF_SIZE - 1) of std_logic_vector(9 downto 0);

    signal bram_y_0 : t_bram;
    signal bram_y_1 : t_bram;
    signal bram_u_a : t_bram;
    signal bram_u_b : t_bram;
    signal bram_v_a : t_bram;
    signal bram_v_b : t_bram;

    -- Per-bank read output registers (1 per bank, written by per-bank read process).
    -- Combinational mux below selects between 0/A and 1/B based on s_bank_sel.
    signal s_y_0_out : std_logic_vector(9 downto 0);
    signal s_y_1_out : std_logic_vector(9 downto 0);
    signal s_u_a_out : std_logic_vector(9 downto 0);
    signal s_u_b_out : std_logic_vector(9 downto 0);
    signal s_v_a_out : std_logic_vector(9 downto 0);
    signal s_v_b_out : std_logic_vector(9 downto 0);

    -- BRAM read outputs (combinational mux of per-bank registers, S3 boundary)
    signal s_prev_y         : std_logic_vector(9 downto 0);
    signal s_prev_u         : std_logic_vector(9 downto 0);
    signal s_prev_v         : std_logic_vector(9 downto 0);

    -- Sanitised prev (to_01) used by the unsigned IIR Liquid blend AND the Ice
    -- peak-hold compare. to_01 protects against 'U' propagation from
    -- uninitialised BRAM in GHDL simulation.
    signal s_prev_y_san     : unsigned(9 downto 0);
    signal s_prev_u_san     : unsigned(9 downto 0);
    signal s_prev_v_san     : unsigned(9 downto 0);

    -- ========================================================================
    -- Blend / gate (S4 — registered)
    -- ========================================================================
    signal s_k_y_eff        : unsigned(2 downto 0);
    signal s_k_uv_eff       : unsigned(2 downto 0);
    signal s_k_offset_uv    : signed(3 downto 0);
    signal s_blend_y        : unsigned(9 downto 0);
    signal s_blend_u        : unsigned(9 downto 0);
    signal s_blend_v        : unsigned(9 downto 0);
    signal s_wb_en          : std_logic;

    -- ========================================================================
    -- Spectral invert (S5 — registered)
    -- ========================================================================
    signal s_inv_y          : unsigned(9 downto 0);
    signal s_inv_u          : unsigned(9 downto 0);
    signal s_inv_v          : unsigned(9 downto 0);
    signal s_inv_wb_en      : std_logic;

    -- ========================================================================
    -- Broadcast-safe clamp + write-back (S6 — registered)
    -- ========================================================================
    signal s_wet_y          : unsigned(9 downto 0);
    signal s_wet_u          : unsigned(9 downto 0);
    signal s_wet_v          : unsigned(9 downto 0);
    signal s_wet_wb_en      : std_logic;

    -- ========================================================================
    -- Interpolator output (S11)
    -- ========================================================================
    signal s_interp_y       : unsigned(9 downto 0);
    signal s_interp_u       : unsigned(9 downto 0);
    signal s_interp_v       : unsigned(9 downto 0);

begin

    -- ========================================================================
    -- Toggle / knob extraction (concurrent)
    -- ========================================================================
    s_gate_invert  <= registers_in(6)(0);
    s_liquid_ice   <= registers_in(6)(1);
    s_spectral_inv <= registers_in(6)(2);
    s_gravity_inv  <= registers_in(6)(3);
    s_freeze       <= registers_in(6)(4);

    -- Centered chroma_decouple knob: 0..1023 → -512..+511
    s_chroma_decouple_signed <= signed(resize(unsigned(registers_in(5)), 11)) - to_signed(512, 11);

    -- ========================================================================
    -- Module: video_timing_generator
    -- ========================================================================
    timing_gen : entity work.video_timing_generator
        port map(
            clk         => clk,
            ref_hsync_n => data_in.hsync_n,
            ref_vsync_n => data_in.vsync_n,
            ref_avid    => data_in.avid,
            timing      => s_timing
        );

    -- ========================================================================
    -- Module: lfsr16 — shared chaos source for turbulence + per-channel jitter
    -- ========================================================================
    lfsr_inst : entity work.lfsr16
        port map(
            clk    => clk,
            enable => '1',
            load   => '0',
            seed   => x"0000",
            q      => s_lfsr_q
        );

    -- ========================================================================
    -- S0a: hcount derived from data_in.avid (0 at first active pixel)
    -- ========================================================================
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

    -- ========================================================================
    -- S0b: line_bank toggle (per hsync) — drives Y/U/V ping-pong bank selector
    -- ========================================================================
    process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                s_line_bank <= not s_line_bank;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- S0: Edge detector (Path A1 / A3) — |Y(n) - Y(n-1)| as 10-bit unsigned
    -- ========================================================================
    process(clk)
        variable v_y_now : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_y_now := unsigned(data_in.y);
            s_y_d_prev_pix <= v_y_now;
            if v_y_now > s_y_d_prev_pix then
                s_edge_d0 <= v_y_now - s_y_d_prev_pix;
            else
                s_edge_d0 <= s_y_d_prev_pix - v_y_now;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- S0: Wind luma-gating — 8-step bit-shift table on Y top-3 (no multiplier)
    -- ========================================================================
    process(clk)
        variable v_wind_in : unsigned(9 downto 0);
        variable v_y_top3  : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            v_wind_in := unsigned(registers_in(2));
            v_y_top3  := unsigned(data_in.y(9 downto 7));
            case v_y_top3 is
                when "000"  => s_wind_eff <= (others => '0');
                when "001"  => s_wind_eff <= shift_right(v_wind_in, 3);
                when "010"  => s_wind_eff <= shift_right(v_wind_in, 2);
                when "011"  => s_wind_eff <= shift_right(v_wind_in, 1) - shift_right(v_wind_in, 3);
                when "100"  => s_wind_eff <= shift_right(v_wind_in, 1);
                when "101"  => s_wind_eff <= shift_right(v_wind_in, 1) + shift_right(v_wind_in, 3);
                when "110"  => s_wind_eff <= v_wind_in - shift_right(v_wind_in, 2);
                when others => s_wind_eff <= v_wind_in;
            end case;
        end if;
    end process;

    -- ========================================================================
    -- Frame-phase breathing (Path A4) — vsync-rate counter, triangle wave on density LSB
    -- Period: 4096 frames ≈ 68 s at 60 Hz (subtle "alive at idle")
    -- ========================================================================
    process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_breath_counter <= s_breath_counter + 1;
                s_frame_counter  <= s_frame_counter  + 1;
            end if;
        end if;
    end process;

    -- Triangle wave: bit 11 selects rising/falling, bits 10..6 give 5-bit slope (0..31)
    process(s_breath_counter)
        variable v_mag : unsigned(5 downto 0);
    begin
        v_mag := "0" & s_breath_counter(10 downto 6);
        if s_breath_counter(11) = '0' then
            s_breath <= signed(v_mag);
        else
            s_breath <= -signed(v_mag);
        end if;
    end process;

    -- Density effective: clamp(reg(4) + breath, 0, 1023)
    process(s_breath, registers_in)
        variable v_sum : signed(11 downto 0);
    begin
        v_sum := signed(resize(unsigned(registers_in(4)), 12)) + resize(s_breath, 12);
        if v_sum < 0 then
            s_density_eff <= (others => '0');
        elsif v_sum > 1023 then
            s_density_eff <= to_unsigned(1023, 10);
        else
            s_density_eff <= unsigned(v_sum(9 downto 0));
        end if;
    end process;

    -- ========================================================================
    -- Density mask: top 3 bits of density_eff select mask width
    -- 000 → "111111111111" (re-pick every 4096 frames, ~68s @ 60Hz)
    -- ...
    -- 111 → "000000000000" (re-pick every frame)
    -- ========================================================================
    process(s_density_eff)
    begin
        case s_density_eff(9 downto 7) is
            when "000"  => s_density_mask <= "111111111111";
            when "001"  => s_density_mask <= "011111111111";
            when "010"  => s_density_mask <= "001111111111";
            when "011"  => s_density_mask <= "000111111111";
            when "100"  => s_density_mask <= "000011111111";
            when "101"  => s_density_mask <= "000001111111";
            when "110"  => s_density_mask <= "000000111111";
            when others => s_density_mask <= (others => '0');
        end case;
    end process;

    -- ========================================================================
    -- Turbulence generator: target re-pick + per-clock IIR glide
    -- ========================================================================
    process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                if (s_frame_counter and s_density_mask) = to_unsigned(0, 12) then
                    -- Re-pick centered ±1024 from LFSR
                    s_turb_target <= signed(resize(unsigned(s_lfsr_q(10 downto 0)), 12)) - to_signed(1024, 12);
                end if;
            end if;
        end if;
    end process;

    -- Per-clock IIR glide: smooth_accum += (target - smooth_accum) >> 4
    process(clk)
        variable v_diff : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            v_diff := resize(s_turb_target, 16) - s_turb_smooth;
            s_turb_smooth <= s_turb_smooth + shift_right(v_diff, 4);
        end if;
    end process;

    -- Y turbulence amplitude scaling: case on Reg 4(9..7) (turbulence_amp top-3)
    process(clk)
        variable v_scale : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            v_scale := unsigned(registers_in(3)(9 downto 7));
            case v_scale is
                when "000"  => s_turb_y <= (others => '0');
                when "001"  => s_turb_y <= resize(shift_right(s_turb_smooth, 6), 11);
                when "010"  => s_turb_y <= resize(shift_right(s_turb_smooth, 5), 11);
                when "011"  => s_turb_y <= resize(shift_right(s_turb_smooth, 4), 11);
                when "100"  => s_turb_y <= resize(shift_right(s_turb_smooth, 3), 11);
                when "101"  => s_turb_y <= resize(shift_right(s_turb_smooth, 2), 11);
                when "110"  => s_turb_y <= resize(shift_right(s_turb_smooth, 1), 11);
                when others => s_turb_y <= resize(s_turb_smooth, 11);
            end case;
        end if;
    end process;

    -- Per-channel chroma jitter (Path A2) — independent UV from different LFSR slices
    process(clk)
        variable v_scale  : unsigned(2 downto 0);
        variable v_u_raw  : signed(10 downto 0);
        variable v_v_raw  : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            v_scale := unsigned(registers_in(3)(9 downto 7));
            v_u_raw := signed(resize(unsigned(s_lfsr_q(7 downto 0)),  11)) - to_signed(128, 11);
            v_v_raw := signed(resize(unsigned(s_lfsr_q(15 downto 8)), 11)) - to_signed(128, 11);
            case v_scale is
                when "000"  => s_turb_u <= (others => '0');           s_turb_v <= (others => '0');
                when "001"  => s_turb_u <= shift_right(v_u_raw, 6);   s_turb_v <= shift_right(v_v_raw, 6);
                when "010"  => s_turb_u <= shift_right(v_u_raw, 5);   s_turb_v <= shift_right(v_v_raw, 5);
                when "011"  => s_turb_u <= shift_right(v_u_raw, 4);   s_turb_v <= shift_right(v_v_raw, 4);
                when "100"  => s_turb_u <= shift_right(v_u_raw, 3);   s_turb_v <= shift_right(v_v_raw, 3);
                when "101"  => s_turb_u <= shift_right(v_u_raw, 2);   s_turb_v <= shift_right(v_v_raw, 2);
                when "110"  => s_turb_u <= shift_right(v_u_raw, 1);   s_turb_v <= shift_right(v_v_raw, 1);
                when others => s_turb_u <= v_u_raw;                   s_turb_v <= v_v_raw;
            end case;
        end if;
    end process;

    -- ========================================================================
    -- S1: Address compute (registered) — wraps modulo 2048 by truncation
    -- ========================================================================
    -- Chroma frac for U leads / V lags — scaled chroma_decouple
    s_chroma_frac <= shift_right(s_chroma_decouple_signed, 3);  -- ±64 max

    process(clk)
        variable v_addr_y : signed(12 downto 0);
        variable v_addr_u : signed(12 downto 0);
        variable v_addr_v : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            v_addr_y := signed(resize(s_hcount, 13))
                        + signed(resize(s_wind_eff, 13))
                        + resize(s_turb_y, 13);
            v_addr_u := v_addr_y + resize(s_turb_u, 13) - resize(s_chroma_frac, 13);
            v_addr_v := v_addr_y + resize(s_turb_v, 13) + resize(s_chroma_frac, 13);
            s_rd_addr_y <= unsigned(v_addr_y(C_BUF_DEPTH - 1 downto 0));
            s_rd_addr_u <= unsigned(v_addr_u(C_BUF_DEPTH - 1 downto 0));
            s_rd_addr_v <= unsigned(v_addr_v(C_BUF_DEPTH - 1 downto 0));
        end if;
    end process;

    -- ========================================================================
    -- S1: Bank selector — Y/U/V all read from "previous line" bank
    -- ========================================================================
    -- Viscosity → k_y: knob HIGH = sticky/strong feedback (small k); knob LOW = clean (large k).
    -- High viscosity physically means content persists, so feedback gain (1/2^k) is large → k small.
    -- Mapping: k_y = 7 - reg(9..7), with the all-ones case promoted from 0 to 1 to prevent
    -- infinite feedback runaway (k must be ≥1 so each pass loses at least half the energy).
    s_k_y_base <= "001" when registers_in(1)(9 downto 7) = "111"
             else to_unsigned(7, 3) - unsigned(registers_in(1)(9 downto 7));

    -- Read bank = line_bank XOR gravity_invert. Write bank = NOT read bank
    -- (current line writes to the bank we're not reading from).
    s_bank_sel <= s_line_bank xor s_gravity_inv;

    -- ========================================================================
    -- S2-S3: BRAM reads — one clocked process per bank (canonical iCE40
    -- inference pattern, matches videomancer-sdk video_line_buffer.vhd).
    -- Output mux is combinational, OUTSIDE the clocked processes.
    -- ========================================================================
    p_rd_y_0 : process(clk)
    begin
        if rising_edge(clk) then
            s_y_0_out <= bram_y_0(to_integer(s_rd_addr_y));
        end if;
    end process;

    p_rd_y_1 : process(clk)
    begin
        if rising_edge(clk) then
            s_y_1_out <= bram_y_1(to_integer(s_rd_addr_y));
        end if;
    end process;

    p_rd_u_a : process(clk)
    begin
        if rising_edge(clk) then
            s_u_a_out <= bram_u_a(to_integer(s_rd_addr_u));
        end if;
    end process;

    p_rd_u_b : process(clk)
    begin
        if rising_edge(clk) then
            s_u_b_out <= bram_u_b(to_integer(s_rd_addr_u));
        end if;
    end process;

    p_rd_v_a : process(clk)
    begin
        if rising_edge(clk) then
            s_v_a_out <= bram_v_a(to_integer(s_rd_addr_v));
        end if;
    end process;

    p_rd_v_b : process(clk)
    begin
        if rising_edge(clk) then
            s_v_b_out <= bram_v_b(to_integer(s_rd_addr_v));
        end if;
    end process;

    -- Combinational read-bank mux (s_bank_sel='0' → read A/0; '1' → read B/1)
    s_prev_y <= s_y_1_out when s_bank_sel = '1' else s_y_0_out;
    s_prev_u <= s_u_b_out when s_bank_sel = '1' else s_u_a_out;
    s_prev_v <= s_v_b_out when s_bank_sel = '1' else s_v_a_out;

    -- Sanitised raw BRAM read for the IIR Liquid blend AND the Ice peak-hold.
    -- to_01 protects against 'U' propagation from uninitialised BRAM (GHDL sim).
    s_prev_y_san <= to_01(unsigned(s_prev_y), '0');
    s_prev_u_san <= to_01(unsigned(s_prev_u), '0');
    s_prev_v_san <= to_01(unsigned(s_prev_v), '0');

    -- ========================================================================
    -- Sigma-delta dither on Viscosity LSB (Reg 1(6..0)) → ±1 to k_eff
    -- 7-bit accumulator with 8-bit overflow → carry sets dither bit
    -- ========================================================================
    process(clk)
        variable v_acc_next : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            v_acc_next     := resize(s_sigma_acc, 9) + resize(unsigned(registers_in(1)(6 downto 0)), 9);
            s_sigma_acc    <= v_acc_next(7 downto 0);
            s_dither_carry <= v_acc_next(8);
        end if;
    end process;

    s_k_y_dithered <= ("0" & s_k_y_base) + ("000" & s_dither_carry);

    -- ========================================================================
    -- S4: blend (Liquid IIR + Ice MAX), gate, A3 edge-modulated k_eff (registered)
    -- Path D: blend moved up one stage (S5 → S4) since filter no longer sits
    -- between BRAM read and blend. Cur/edge taps shifted from d4 to d3.
    -- ========================================================================
    -- k_y_eff = clamp(1, k_y_dithered - (edge > C_EDGE_TH_K ? 1 : 0), 7)
    process(s_k_y_dithered, s_edge_d3)
        variable v_k : signed(4 downto 0);
    begin
        if s_edge_d3 > C_EDGE_TH_K then
            v_k := signed("0" & s_k_y_dithered) - 1;
        else
            v_k := signed("0" & s_k_y_dithered);
        end if;
        if v_k < 1 then
            s_k_y_eff <= "001";
        elsif v_k > 7 then
            s_k_y_eff <= "111";
        else
            s_k_y_eff <= unsigned(v_k(2 downto 0));
        end if;
    end process;

    -- k_uv_eff = clamp(1, k_y_eff + chroma_decouple_offset, 7)
    s_k_offset_uv <= resize(shift_right(s_chroma_decouple_signed, 6), 4);  -- ±8

    process(s_k_y_eff, s_k_offset_uv)
        variable v_k : signed(4 downto 0);
    begin
        v_k := signed("00" & s_k_y_eff) + resize(s_k_offset_uv, 5);
        if v_k < 1 then
            s_k_uv_eff <= "001";
        elsif v_k > 7 then
            s_k_uv_eff <= "111";
        else
            s_k_uv_eff <= unsigned(v_k(2 downto 0));
        end if;
    end process;

    -- Blend + gate registered at S4
    process(clk)
        variable v_cur_y, v_cur_u, v_cur_v          : unsigned(9 downto 0);
        variable v_acc_y, v_acc_u, v_acc_v          : unsigned(17 downto 0);
        variable v_liq_y, v_liq_u, v_liq_v          : unsigned(9 downto 0);
        variable v_ice_y, v_ice_u, v_ice_v          : signed(10 downto 0);
        variable v_prev_s_y, v_prev_s_u, v_prev_s_v : signed(10 downto 0);
        variable v_cur_s_y, v_cur_s_u, v_cur_s_v    : signed(10 downto 0);
        variable v_threshold                        : unsigned(9 downto 0);
        variable v_gate_pass_luma, v_gate_pass_edge : std_logic;
    begin
        if rising_edge(clk) then
            v_cur_y := unsigned(s_data_in_d3_y);
            v_cur_u := unsigned(s_data_in_d3_u);
            v_cur_v := unsigned(s_data_in_d3_v);

            -- Liquid (unsigned, bias-free):
            --   out = cur + (prev - cur)/2^k = ((cur << k) - cur + prev) >> k
            -- All operands unsigned; unsigned shift_right truncates toward 0
            -- symmetrically, so no chroma drift attractor across multi-frame cascade.
            -- 18-bit acc: max value 1023*128 + 1023 = 131967 fits in 18 bits.
            -- k_eff range 1..7 so (cur<<k) >= 2*cur > cur, subtraction is non-negative.
            v_acc_y := shift_left(resize(v_cur_y, 18), to_integer(s_k_y_eff))
                       - resize(v_cur_y, 18) + resize(s_prev_y_san, 18);
            v_acc_u := shift_left(resize(v_cur_u, 18), to_integer(s_k_uv_eff))
                       - resize(v_cur_u, 18) + resize(s_prev_u_san, 18);
            v_acc_v := shift_left(resize(v_cur_v, 18), to_integer(s_k_uv_eff))
                       - resize(v_cur_v, 18) + resize(s_prev_v_san, 18);
            v_liq_y := resize(shift_right(v_acc_y, to_integer(s_k_y_eff)), 10);
            v_liq_u := resize(shift_right(v_acc_u, to_integer(s_k_uv_eff)), 10);
            v_liq_v := resize(shift_right(v_acc_v, to_integer(s_k_uv_eff)), 10);

            -- Ice: out = MAX(cur, prev - C_ICE_CONST). Signed math handles the
            -- prev < C_ICE_CONST underflow path; the MAX with cur (>=0) clamps.
            v_prev_s_y := signed("0" & s_prev_y_san);
            v_prev_s_u := signed("0" & s_prev_u_san);
            v_prev_s_v := signed("0" & s_prev_v_san);
            v_cur_s_y  := signed("0" & v_cur_y);
            v_cur_s_u  := signed("0" & v_cur_u);
            v_cur_s_v  := signed("0" & v_cur_v);
            if (v_prev_s_y - C_ICE_CONST) > v_cur_s_y then v_ice_y := v_prev_s_y - C_ICE_CONST;
            else                                            v_ice_y := v_cur_s_y; end if;
            if (v_prev_s_u - C_ICE_CONST) > v_cur_s_u then v_ice_u := v_prev_s_u - C_ICE_CONST;
            else                                            v_ice_u := v_cur_s_u; end if;
            if (v_prev_s_v - C_ICE_CONST) > v_cur_s_v then v_ice_v := v_prev_s_v - C_ICE_CONST;
            else                                            v_ice_v := v_cur_s_v; end if;

            -- Mux Liquid / Ice. Ice MAX-with-cur guarantees non-negative result
            -- (cur is unsigned 0..1023), so the low 10 bits are safe.
            if s_liquid_ice = '0' then
                s_blend_y <= v_liq_y;
                s_blend_u <= v_liq_u;
                s_blend_v <= v_liq_v;
            else
                s_blend_y <= unsigned(v_ice_y(9 downto 0));
                s_blend_u <= unsigned(v_ice_u(9 downto 0));
                s_blend_v <= unsigned(v_ice_v(9 downto 0));
            end if;

            -- Gate: luma compare (per gate_invert) OR'd with edge bias (Path A1)
            v_threshold := unsigned(registers_in(0));
            if s_gate_invert = '0' then
                if v_cur_y >= v_threshold then v_gate_pass_luma := '1';
                else v_gate_pass_luma := '0'; end if;
            else
                if v_cur_y <= v_threshold then v_gate_pass_luma := '1';
                else v_gate_pass_luma := '0'; end if;
            end if;
            if s_edge_d3 > shift_right(v_threshold, 2) then
                v_gate_pass_edge := '1';
            else
                v_gate_pass_edge := '0';
            end if;
            s_wb_en <= s_avid_d3 AND (NOT s_freeze) AND (v_gate_pass_luma OR v_gate_pass_edge);
        end if;
    end process;

    -- ========================================================================
    -- S5: spectral invert (registered)
    -- ========================================================================
    process(clk)
    begin
        if rising_edge(clk) then
            s_inv_y <= s_blend_y;
            if s_spectral_inv = '1' then
                s_inv_u <= to_unsigned(1023, 10) - s_blend_u;
                s_inv_v <= to_unsigned(1023, 10) - s_blend_v;
            else
                s_inv_u <= s_blend_u;
                s_inv_v <= s_blend_v;
            end if;
            s_inv_wb_en <= s_wb_en;
        end if;
    end process;

    -- ========================================================================
    -- S6: broadcast-safe clamp + write-back stage (registered)
    -- ========================================================================
    -- Y clamp [64, 940], UV clamp [64, 960]. The IIR + spectral path is
    -- normally well within range (inputs come from BT.601-clean sources via
    -- the SDK's BT.601 conditioner), but the clamp guards against:
    --   - black/white-bar test sources at the BT.601 limits
    --   - any future addition that could overshoot
    process(clk)
    begin
        if rising_edge(clk) then
            if    s_inv_y < to_unsigned(64,  10) then s_wet_y <= to_unsigned(64,  10);
            elsif s_inv_y > to_unsigned(940, 10) then s_wet_y <= to_unsigned(940, 10);
            else                                       s_wet_y <= s_inv_y;
            end if;
            if    s_inv_u < to_unsigned(64,  10) then s_wet_u <= to_unsigned(64,  10);
            elsif s_inv_u > to_unsigned(960, 10) then s_wet_u <= to_unsigned(960, 10);
            else                                       s_wet_u <= s_inv_u;
            end if;
            if    s_inv_v < to_unsigned(64,  10) then s_wet_v <= to_unsigned(64,  10);
            elsif s_inv_v > to_unsigned(960, 10) then s_wet_v <= to_unsigned(960, 10);
            else                                       s_wet_v <= s_inv_v;
            end if;
            s_wet_wb_en <= s_inv_wb_en;
        end if;
    end process;

    -- ========================================================================
    -- BRAM write-back — one clocked process per bank (canonical pattern).
    -- Each process gated by `wb_en AND (bank_sel matches "write to opposite
    -- of read bank")`. Yosys sees this as a per-port write enable on each
    -- inferred RAM, which maps cleanly to SB_RAM40_4K WREN.
    --
    -- Read bank chosen by s_bank_sel; write bank = NOT s_bank_sel.
    --   s_bank_sel = '0' → read A/0, write to B/1 banks
    --   s_bank_sel = '1' → read B/1, write to A/0 banks
    -- ========================================================================
    p_wr_y_0 : process(clk)  -- writes when bank_sel = '1' (read=1, write=0)
    begin
        if rising_edge(clk) then
            if s_wet_wb_en = '1' and s_bank_sel = '1' then
                bram_y_0(to_integer(s_wb_addr)) <= std_logic_vector(s_wet_y);
            end if;
        end if;
    end process;

    p_wr_y_1 : process(clk)  -- writes when bank_sel = '0' (read=0, write=1)
    begin
        if rising_edge(clk) then
            if s_wet_wb_en = '1' and s_bank_sel = '0' then
                bram_y_1(to_integer(s_wb_addr)) <= std_logic_vector(s_wet_y);
            end if;
        end if;
    end process;

    p_wr_u_a : process(clk)  -- writes when bank_sel = '1' (read=B, write=A)
    begin
        if rising_edge(clk) then
            if s_wet_wb_en = '1' and s_bank_sel = '1' then
                bram_u_a(to_integer(s_wb_addr)) <= std_logic_vector(s_wet_u);
            end if;
        end if;
    end process;

    p_wr_u_b : process(clk)  -- writes when bank_sel = '0' (read=A, write=B)
    begin
        if rising_edge(clk) then
            if s_wet_wb_en = '1' and s_bank_sel = '0' then
                bram_u_b(to_integer(s_wb_addr)) <= std_logic_vector(s_wet_u);
            end if;
        end if;
    end process;

    p_wr_v_a : process(clk)
    begin
        if rising_edge(clk) then
            if s_wet_wb_en = '1' and s_bank_sel = '1' then
                bram_v_a(to_integer(s_wb_addr)) <= std_logic_vector(s_wet_v);
            end if;
        end if;
    end process;

    p_wr_v_b : process(clk)
    begin
        if rising_edge(clk) then
            if s_wet_wb_en = '1' and s_bank_sel = '0' then
                bram_v_b(to_integer(s_wb_addr)) <= std_logic_vector(s_wet_v);
            end if;
        end if;
    end process;

    -- ========================================================================
    -- S7-S10: interpolator_u (4-clk, dry/wet mix)
    -- a = data_in delayed 6 clk (clean dry), b = clamped wet, t = Master Mix
    -- ========================================================================
    interp_y : entity work.interpolator_u
        generic map(G_WIDTH => 10, G_FRAC_BITS => 10, G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(
            clk    => clk,
            enable => s_avid_d_interp,
            a      => unsigned(s_data_in_d6_y),
            b      => s_wet_y,
            t      => unsigned(registers_in(7)),
            result => s_interp_y,
            valid  => open
        );

    interp_u : entity work.interpolator_u
        generic map(G_WIDTH => 10, G_FRAC_BITS => 10, G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(
            clk    => clk,
            enable => s_avid_d_interp,
            a      => unsigned(s_data_in_d6_u),
            b      => s_wet_u,
            t      => unsigned(registers_in(7)),
            result => s_interp_u,
            valid  => open
        );

    interp_v : entity work.interpolator_u
        generic map(G_WIDTH => 10, G_FRAC_BITS => 10, G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(
            clk    => clk,
            enable => s_avid_d_interp,
            a      => unsigned(s_data_in_d6_v),
            b      => s_wet_v,
            t      => unsigned(registers_in(7)),
            result => s_interp_v,
            valid  => open
        );

    -- ========================================================================
    -- Output port assignments — broadcast-safe clamp at port (R1, redundant guard)
    -- Mirrors spatial_mosher pattern: dry-side could push values outside safe range
    -- ========================================================================
    data_out.y <= std_logic_vector(to_unsigned(64,  10)) when s_interp_y < to_unsigned(64,  10)
             else std_logic_vector(to_unsigned(940, 10)) when s_interp_y > to_unsigned(940, 10)
             else std_logic_vector(s_interp_y);
    data_out.u <= std_logic_vector(to_unsigned(64,  10)) when s_interp_u < to_unsigned(64,  10)
             else std_logic_vector(to_unsigned(960, 10)) when s_interp_u > to_unsigned(960, 10)
             else std_logic_vector(s_interp_u);
    data_out.v <= std_logic_vector(to_unsigned(64,  10)) when s_interp_v < to_unsigned(64,  10)
             else std_logic_vector(to_unsigned(960, 10)) when s_interp_v > to_unsigned(960, 10)
             else std_logic_vector(s_interp_v);

    -- ========================================================================
    -- Sync delay: C_PROCESSING_DELAY_CLKS = 10
    -- Carries sync flags + clean data taps + edge magnitude alignment
    -- ========================================================================
    p_sync_delay : process(clk)
        type t_sync_delay is array(0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic;
        type t_data_delay is array(0 to C_PROCESSING_DELAY_CLKS - 1) of std_logic_vector(9 downto 0);
        type t_edge_delay is array(0 to C_PROCESSING_DELAY_CLKS - 1) of unsigned(9 downto 0);
        variable v_avid    : t_sync_delay := (others => '0');
        variable v_hsync   : t_sync_delay := (others => '1');
        variable v_vsync   : t_sync_delay := (others => '1');
        variable v_field   : t_sync_delay := (others => '1');
        variable v_y_clean : t_data_delay := (others => (others => '0'));
        variable v_u_clean : t_data_delay := (others => (others => '0'));
        variable v_v_clean : t_data_delay := (others => (others => '0'));
        variable v_edge    : t_edge_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_avid    := data_in.avid    & v_avid(0    to C_PROCESSING_DELAY_CLKS - 2);
            v_hsync   := data_in.hsync_n & v_hsync(0   to C_PROCESSING_DELAY_CLKS - 2);
            v_vsync   := data_in.vsync_n & v_vsync(0   to C_PROCESSING_DELAY_CLKS - 2);
            v_field   := data_in.field_n & v_field(0   to C_PROCESSING_DELAY_CLKS - 2);
            v_y_clean := data_in.y       & v_y_clean(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_u_clean := data_in.u       & v_u_clean(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_v_clean := data_in.v       & v_v_clean(0 to C_PROCESSING_DELAY_CLKS - 2);
            v_edge    := s_edge_d0       & v_edge(0    to C_PROCESSING_DELAY_CLKS - 2);

            -- Sync outputs at full pipeline depth (index 9 = 10 effective clocks)
            data_out.avid    <= v_avid(C_PROCESSING_DELAY_CLKS - 1);
            data_out.hsync_n <= v_hsync(C_PROCESSING_DELAY_CLKS - 1);
            data_out.vsync_n <= v_vsync(C_PROCESSING_DELAY_CLKS - 1);
            data_out.field_n <= v_field(C_PROCESSING_DELAY_CLKS - 1);

            -- 3-clock taps: blend "cur" path aligned with s_prev_*_san at S4 input
            s_data_in_d3_y <= v_y_clean(2);
            s_data_in_d3_u <= v_u_clean(2);
            s_data_in_d3_v <= v_v_clean(2);
            s_edge_d3      <= v_edge(2);

            -- 6-clock taps: dry path for interpolator (a-input), aligned with s_wet_* at S7
            s_data_in_d6_y <= v_y_clean(5);
            s_data_in_d6_u <= v_u_clean(5);
            s_data_in_d6_v <= v_v_clean(5);

            -- Blend gate (3-clock avid → matches s_data_in_d3_* at S4 blend input)
            s_avid_d3       <= v_avid(2);
            -- Interpolator enable (6-clock avid → matches s_wet_* at S7)
            s_avid_d_interp <= v_avid(5);
        end if;
    end process;

    -- ========================================================================
    -- Write-back address: hcount delayed C_WB_ADDR_DELAY = 6 clocks
    -- (matches s_wet_* register at S6; BRAM write process samples on next edge)
    -- ========================================================================
    process(clk)
        type t_addr_pipe is array(0 to C_WB_ADDR_DELAY - 1) of unsigned(C_BUF_DEPTH - 1 downto 0);
        variable v_pipe : t_addr_pipe := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_pipe     := s_hcount & v_pipe(0 to C_WB_ADDR_DELAY - 2);
            s_wb_addr  <= v_pipe(C_WB_ADDR_DELAY - 1);
        end if;
    end process;

end architecture gravity_bleed;
