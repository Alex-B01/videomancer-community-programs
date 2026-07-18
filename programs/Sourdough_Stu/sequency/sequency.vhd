-- Videomancer community program - SEQUENCY (Walsh-Hadamard organ)
-- Copyright (C) 2026 AB (Sourdough_Stu)
-- License: GNU General Public License v3.0
-- https://github.com/lzxindustries/videomancer-community-programs
--
-- This file is free software: you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation, either version 3 of the License, or
-- (at your option) any later version.  Distributed WITHOUT ANY WARRANTY.
--
-- Program Name:
--   SEQUENCY  (Program 5 - the Walsh-Hadamard organ)
--
-- Author:
--   AB (Sourdough_Stu)
--
-- Overview:
--   16-pixel *horizontal* Walsh-Hadamard blocks on LUMA (Y).  Y pixels gather
--   serially into a 16-wide bank (S2P); a completed block runs a pipelined
--   forward WHT (4 radix-2 ranks, all add/sub) to 16 sequency coefficients; the
--   coefficients are *played* (QUANTIZE deadzone, TILT, SCRAMBLE, REGEN vertical
--   feedback, ADAPT); an inverse WHT (honest 19-bit datapath) reconstructs 16 Y
--   pixels (P2S), then a true dry/wet interpolator_u fader blends transformed-Y
--   against delay-matched dry-Y.  U and V are unprocessed passthrough,
--   delay-matched to Y (bit-exact).  The only RAM is the Y coefficient line
--   buffer carrying REGEN's vertical (line-to-line) feedback.  No framebuffer,
--   no temporal state -> all-formats native.  H*H = 16*I (symmetric) so
--   inverse = forward then >>4; neutral knobs round-trip Y bit-exact.  Y-only
--   per user re-commit (post-Gate-1); chroma-through-organ is a future
--   time-multiplexed variant (spec S8), NOT a trim of this build.
--
-- Pipeline stages + total latency (spec S1, latency divisible by 4):
--   G  gather (S2P, 16-wide Y bank, avid-anchored mod-16)            16   cum 16
--   F1 forward WHT ranks 1-2 (h=1,h=2); datapath 10->~12b             1   cum 17
--   F2 forward WHT ranks 3-4 (h=4,h=8) -> 16 coeffs (14-bit)          1   cum 18
--   M1 QUANTIZE(hard) + TILT + ADAPT                                  1   cum 19
--   M2 SCRAMBLE(XOR) + REGEN(+prev line) + Y buffer write             1   cum 20
--   I1 inverse WHT ranks 1-2 (19-bit signed datapath)                1   cum 21
--   I2 inverse WHT ranks 3-4, >>4 normalize, load P2S bank            1   cum 22
--   P  P2S scatter (combinational read of block bank at delayed ph)   1   cum 23
--   B  dry/wet interpolator_u x1 (Y); U/V passthrough ride 28-delay   4   cum 27
--   C  broadcast-safe output clamp R1                                 1   cum 28
--   TOTAL LATENCY = 28  (/4 ok).  Sync (avid/hsync_n/vsync_n/field_n)
--   delayed by EXACTLY 28 (fixed shift register; datapath < 1 line so no
--   line-delay BRAM is in the video path -- the coeff buffer is a side path).
--
-- EBR map (physical, power-of-two; spec S2):
--   cbuf_y : 2048 x 12b, 1W1R dual-port, single storage  -> 6 physical EBR
--   TOTAL = 6 physical EBR (vs program ceiling 14, hard 32).  Ping-pong block
--   banks + weight/permute tables are registers/LUT -> 0 EBR.  No chroma
--   buffer in v1 (chroma deferred to spec S8).
--
-- Coefficient datapath widths (honest sizing per spec S3.1 / A6):
--   FW = 14  forward + manip domain.  Centered 10b input, x16 forward growth
--            reaches +/-8192 -> exactly 14b signed.  Coeff STORAGE in cbuf_y is
--            truncated to 12b (feedback aesthetic, off the bit-exact path).
--   IW = 19  inverse domain.  Inverse of 14b coeffs grows another x16 to
--            +/-131072 before the >>4 normalize (rule input + 2*log2(N) =
--            10 + 8 = 18b unsigned -> 19b signed).  Narrower drops MSBs and
--            breaks the bit-exact round-trip (hook #1).
--
-- LC estimate (spec Y-only ledger ~4,665 / 5,500; ~835 LC margin).  Two WHT
--   networks only: Y-forward (10->14b ~800) + Y-inverse (19b ~1,050).
--   Multiplier ledger: ZERO multiplier_s (WHT is add/sub; fader is
--   interpolator_u; all gains are shift/add by compile-time constants).
--
-- Register map (spec S4; musical scaling lives in TOML, zero fabric):
--   Knob1 registers_in(0)  QUANTIZE  sequency-weighted deadzone (hard)
--   Knob2 registers_in(1)  TILT      bass<->treble (bipolar, 512 = flat)
--   Knob3 registers_in(2)  SCRAMBLE  basis XOR permute depth
--   Knob4 registers_in(3)  REGEN     coefficient (vertical) feedback
--   Knob5 registers_in(4)  CROSSTALK v1 RESERVED / INERT (no chroma path in
--                                     v1); register read, NOT acted on (S8).
--   Knob6 registers_in(5)  ADAPT     energy -> quantizer self-mod (intra-frame)
--   Tog7  registers_in(6)(0) Quantizer  hard/dithered  (v1 Hard; "Dithered" INERT)
--   Tog8  registers_in(6)(1) Scramble   XOR/rotate     (v1 XOR;  "Rotate"  INERT)
--   Tog9  registers_in(6)(2) ADAPT polarity  normal/inverted
--   Tog10 registers_in(6)(3) Spectral Hold   live/hold (freeze coeff buffer)
--   Tog11 registers_in(6)(4) Chroma     v1 RESERVED / INERT (fixed "Luma only");
--                                     register read, NOT acted on (S8).
--   Fader registers_in(7)  Dry/Wet   true crossfade (interpolator_u, Y)
--   registers_in(8)(3 downto 0) timing ID -- read for cross-check only; block
--                              tiling self-calibrates from avid edges.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture sequency of program_top is

    -- ====================================================================
    -- Constants
    -- ====================================================================
    constant N          : integer := 16;                 -- WHT block size
    constant CW         : integer := C_VIDEO_DATA_WIDTH;  -- 10 (video width)
    constant FW         : integer := 14;   -- forward / coefficient signed width (10 + log2(N))
    constant IW         : integer := 19;   -- inverse datapath signed width (10 + 2*log2(N) + sign)
    constant BW         : integer := 12;   -- coeff line-buffer width
    constant ABITS      : integer := 11;   -- 2048-deep buffer address
    constant BUF_DEPTH  : integer := 2 ** ABITS;          -- 2048

    constant C_LAT      : integer := 28;   -- total latency = sync delay depth
    constant C_PRE      : integer := 23;   -- input->fader delay (28 - 4 - 1)

    -- ====================================================================
    -- Array types
    -- ====================================================================
    type t_fwd  is array(0 to N-1) of signed(FW-1 downto 0);
    type t_inv  is array(0 to N-1) of signed(IW-1 downto 0);
    type t_pxv  is array(0 to N-1) of std_logic_vector(CW-1 downto 0);
    type t_p2s  is array(0 to 1)   of t_pxv;   -- double-buffered Y scatter banks

    -- sequency of each natural-order Hadamard index (sign-change count)
    type t_seqtab is array(0 to N-1) of integer range 0 to N-1;
    constant SEQ : t_seqtab :=
        (0, 15, 7, 8, 3, 12, 4, 11, 1, 14, 6, 9, 2, 13, 5, 10);

    -- ====================================================================
    -- Y coefficient line buffer (the ONLY RAM)
    -- ====================================================================
    type t_cbram is array(0 to BUF_DEPTH-1) of std_logic_vector(BW-1 downto 0);
    signal cbuf_y : t_cbram := (others => (others => '0'));  -- 2048x12 1W1R -> 6 physical EBR

    -- ====================================================================
    -- Small helpers (all multiply-free at run time)
    -- ====================================================================
    function b2i(s : std_logic) return integer is
    begin
        if s = '1' then return 1; else return 0; end if;
    end function;

    -- one radix-2 WHT rank, stride h, on the forward-width vector
    function fwd_rank(v : t_fwd; h : integer) return t_fwd is
        variable o : t_fwd := v;
    begin
        for i in 0 to N-1 loop
            if ((i / h) mod 2) = 0 then
                o(i)     := v(i) + v(i + h);
                o(i + h) := v(i) - v(i + h);
            end if;
        end loop;
        return o;
    end function;

    -- one radix-2 WHT rank on the inverse-width vector
    function inv_rank(v : t_inv; h : integer) return t_inv is
        variable o : t_inv := v;
    begin
        for i in 0 to N-1 loop
            if ((i / h) mod 2) = 0 then
                o(i)     := v(i) + v(i + h);
                o(i + h) := v(i) - v(i + h);
            end if;
        end loop;
        return o;
    end function;

    function to_inv(v : t_fwd) return t_inv is
        variable o : t_inv;
    begin
        for i in 0 to N-1 loop
            o(i) := resize(v(i), IW);
        end loop;
        return o;
    end function;

    -- saturate to the 14-bit coefficient range, returned in FW bits
    function sat_coef(x : signed) return signed is
    begin
        if x > 8191 then
            return to_signed(8191, FW);
        elsif x < -8192 then
            return to_signed(-8192, FW);
        else
            return resize(x, FW);
        end if;
    end function;

    -- center a 10-bit unsigned pixel to signed FW: [-512 .. 511]
    function centr(p : std_logic_vector) return signed is
    begin
        return resize(signed('0' & p), FW) - to_signed(512, FW);
    end function;

    -- de-center (already >>4 normalized) + clamp to [0,1023] -> 10-bit.
    -- In the else-branch t is in [0,1023]; take the low CW bits as an UNSIGNED
    -- magnitude.  A signed resize(t, CW) here would fold bit 9 (the +512 place
    -- becomes the 10-bit sign bit), collapsing every luma >= 512 to luma-512.
    function decen(c : signed) return std_logic_vector is
        variable t : signed(FW downto 0);
    begin
        t := resize(c, FW + 1) + to_signed(512, FW + 1);
        if t < to_signed(0, FW + 1) then
            return (CW - 1 downto 0 => '0');
        elsif t > to_signed(1023, FW + 1) then
            return std_logic_vector(to_unsigned(1023, CW));
        else
            return std_logic_vector(t(CW - 1 downto 0));  -- unsigned low bits, no sign fold
        end if;
    end function;

    -- broadcast-safe clamp
    function clampr(v : unsigned(CW-1 downto 0); lo : integer; hi : integer)
        return unsigned is
    begin
        if v < to_unsigned(lo, CW) then
            return to_unsigned(lo, CW);
        elsif v > to_unsigned(hi, CW) then
            return to_unsigned(hi, CW);
        else
            return v;
        end if;
    end function;

    -- coeff <-> line-buffer packing (14b coeff truncated to 12b for the
    -- feedback storage path; REGEN is an aesthetic, not the bit-exact path)
    function pack_y(c : signed) return std_logic_vector is
    begin  -- 14b coeff -> 12b (>>2)
        return std_logic_vector(resize(shift_right(c, 2), BW));
    end function;
    function unpack_y(q : std_logic_vector) return signed is
    begin  -- 12b -> FW, <<2
        return shift_left(resize(signed(q), FW), 2);
    end function;

    -- ====================================================================
    -- Parameter signals (spec S4)
    -- ====================================================================
    signal s_quant   : unsigned(CW-1 downto 0);
    signal s_tilt    : unsigned(CW-1 downto 0);
    signal s_scram   : unsigned(CW-1 downto 0);
    signal s_regen   : unsigned(CW-1 downto 0);
    signal s_xtalk_r : unsigned(CW-1 downto 0);  -- Knob5 CROSSTALK: RESERVED/INERT (read, unused)
    signal s_adapt   : unsigned(CW-1 downto 0);
    signal s_tog_dither : std_logic;   -- Tog7 "Dithered": retired-mode, inert (read, unused)
    signal s_tog_rotate : std_logic;   -- Tog8 "Rotate":   retired-mode, inert (read, unused)
    signal s_adapt_inv  : std_logic;
    signal s_hold       : std_logic;
    signal s_chroma_r   : std_logic;   -- Tog11 Chroma: RESERVED/INERT (read, unused)
    signal s_fader   : unsigned(CW-1 downto 0);
    signal s_timing_id : std_logic_vector(3 downto 0);  -- cross-check only

    -- ====================================================================
    -- Input / gather (S2P) + block-tiling counters
    -- ====================================================================
    signal s_prev_avid : std_logic := '0';
    signal s_ph    : unsigned(3 downto 0) := (others => '0');   -- block phase 0..15
    signal s_apx   : unsigned(ABITS-1 downto 0) := (others => '0'); -- active pixel idx
    -- P2S ping-pong bank parity.  Derived from a dedicated FF (NOT s_apx(4)) so it
    -- is reset to a deterministic line-start value: held '0' through blanking, so
    -- block 0 of every line always uses bank 0 regardless of the prior line's
    -- block-count parity (odd for 720-wide SD = 45 blocks).  See p_in.
    signal s_blk_par : std_logic := '0';
    signal s_gather : t_pxv;    -- Y gather bank (S2P)
    signal s_xin    : t_fwd;    -- latched, centered Y block (transform input)

    -- ====================================================================
    -- Transform pipeline datapath (Y only)
    -- ====================================================================
    signal s_f1, s_f2, s_m1, s_m2 : t_fwd;
    signal s_i1                   : t_inv;

    -- pipeline control (valid / parity / base address travel with the block)
    signal s_v    : std_logic_vector(0 to 6) := (others => '0');
    signal s_par  : std_logic_vector(0 to 6) := (others => '0');
    type   t_base is array(0 to 6) of unsigned(ABITS-1 downto 0);
    signal s_base : t_base := (others => (others => '0'));

    -- ====================================================================
    -- REGEN coefficient buffer plumbing (Y)
    -- ====================================================================
    signal s_cbuf_y_q : std_logic_vector(BW-1 downto 0);
    signal s_apx_d1   : unsigned(ABITS-1 downto 0) := (others => '0');
    -- howler-style signal init (no resets on iCE40): defined-but-stale at
    -- top-of-frame rather than X (matches the spec's <=1-line-stale caveat).
    signal s_prev     : t_fwd := (others => (others => '0'));  -- prev-line coeffs assembled (per phase)
    signal s_prev_blk : t_fwd := (others => (others => '0'));  -- latched at block complete, consumed at M2

    -- write-back serializer
    signal s_wb_bank : t_fwd;
    signal s_wb_base : unsigned(ABITS-1 downto 0) := (others => '0');
    signal s_wb_cnt  : unsigned(3 downto 0) := (others => '0');
    signal s_wb_run  : std_logic := '0';
    signal s_wr_en   : std_logic := '0';
    signal s_wr_addr : unsigned(ABITS-1 downto 0) := (others => '0');
    signal s_wr_y    : std_logic_vector(BW-1 downto 0);

    -- ====================================================================
    -- P2S scatter (double buffered, Y) + output delay lines
    -- ====================================================================
    signal s_p2s   : t_p2s;

    signal out_ph_d  : unsigned(3 downto 0) := (others => '0');
    signal out_par_d : std_logic := '0';
    signal dry_y : std_logic_vector(CW-1 downto 0) := (others => '0');  -- Y dry, delay 23
    signal pass_u, pass_v : std_logic_vector(CW-1 downto 0) := (others => '0'); -- U/V passthrough, delay 27

    -- Y fader endpoints
    signal a_y, b_y : unsigned(CW-1 downto 0);
    signal mix_y    : unsigned(CW-1 downto 0);
    signal mix_vy   : std_logic;

    -- final clamp registers
    signal s_out_y, s_out_u, s_out_v : unsigned(CW-1 downto 0) := (others => '0');

    -- sync delay (28 deep)
    signal s_avid_d, s_hs_d, s_vs_d, s_fld_d : std_logic;

begin

    -- ====================================================================
    -- Register map
    -- ====================================================================
    s_quant      <= unsigned(registers_in(0));
    s_tilt       <= unsigned(registers_in(1));
    s_scram      <= unsigned(registers_in(2));
    s_regen      <= unsigned(registers_in(3));
    s_xtalk_r    <= unsigned(registers_in(4));  -- CROSSTALK: read, inert (no chroma path in v1)
    s_adapt      <= unsigned(registers_in(5));
    s_tog_dither <= registers_in(6)(0);
    s_tog_rotate <= registers_in(6)(1);
    s_adapt_inv  <= registers_in(6)(2);
    s_hold       <= registers_in(6)(3);
    s_chroma_r   <= registers_in(6)(4);          -- Chroma: read, inert (no chroma path in v1)
    s_fader      <= unsigned(registers_in(7));
    s_timing_id  <= registers_in(8)(3 downto 0);

    -- ====================================================================
    -- Stage G : gather (S2P) + avid-anchored mod-16 block tiling
    --   block index = avid-anchored mod-16 counter; reset on avid rising,
    --   terminates at avid falling; active widths are exact multiples of 16.
    -- ====================================================================
    p_in : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_avid <= data_in.avid;

            if data_in.avid = '1' and s_prev_avid = '0' then
                -- avid rising: start of active line.  s_blk_par is already '0'
                -- (held through blanking) so block 0 of every line uses bank 0.
                s_ph  <= to_unsigned(1, 4);
                s_apx <= to_unsigned(1, ABITS);
                s_gather(0) <= data_in.y;
            elsif data_in.avid = '1' then
                s_gather(to_integer(s_ph)) <= data_in.y;
                if s_ph = 15 then
                    s_ph <= (others => '0');
                    s_blk_par <= not s_blk_par;   -- next block flips ping-pong parity
                    -- block complete: snapshot centered Y block into transform input
                    for i in 0 to 14 loop
                        s_xin(i) <= centr(s_gather(i));
                    end loop;
                    s_xin(15) <= centr(data_in.y);
                else
                    s_ph <= s_ph + 1;
                end if;
                s_apx <= s_apx + 1;
            else
                -- blanking: hold ping-pong parity at its deterministic line-start
                -- value so the prior line's block-count parity cannot leak into
                -- the first block of the next line (the SD odd-block identity bug).
                s_blk_par <= '0';
            end if;
        end if;
    end process;

    -- ====================================================================
    -- Pipeline control: valid / parity / base address travel with the block
    -- ====================================================================
    p_ctrl : process(clk)
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' and s_ph = 15 then
                s_v(0)    <= '1';
                s_par(0)  <= s_blk_par;                       -- ping-pong parity (write side)
                s_base(0) <= s_apx(ABITS-1 downto 4) & "0000"; -- block base addr
            else
                s_v(0) <= '0';
            end if;
            for k in 1 to 6 loop
                s_v(k)    <= s_v(k-1);
                s_par(k)  <= s_par(k-1);
                s_base(k) <= s_base(k-1);
            end loop;
        end if;
    end process;

    -- ====================================================================
    -- Stage F1/F2 : forward WHT (2 radix-2 ranks per clock)
    -- ====================================================================
    p_fwd : process(clk)
    begin
        if rising_edge(clk) then
            s_f1 <= fwd_rank(fwd_rank(s_xin, 1), 2);  -- ranks 1,2
            s_f2 <= fwd_rank(fwd_rank(s_f1, 4), 8);   -- ranks 3,4
        end if;
    end process;

    -- ====================================================================
    -- Stage M1 : QUANTIZE (hard, sequency-weighted deadzone) + TILT + ADAPT
    -- ====================================================================
    p_m1 : process(clk)
        variable E           : unsigned(20 downto 0);
        variable adapt_delta : unsigned(20 downto 0);
        variable qeff        : unsigned(11 downto 0);
        variable thr         : unsigned(19 downto 0);
        variable c, qc, tc   : signed(FW-1 downto 0);
        variable sh          : integer range 0 to 7;
        variable bshift      : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            -- ADAPT: intra-frame block energy from Y AC coeffs (16->1 add tree)
            E := (others => '0');
            for i in 1 to 15 loop
                E := E + resize(unsigned(abs(s_f2(i))), E'length);
            end loop;
            if s_adapt = 0 then
                adapt_delta := (others => '0');
            else  -- shift-based scale (>>10 .. >>7), multiply-free
                adapt_delta := shift_right(E, 10 - to_integer(s_adapt(9 downto 8)));
            end if;

            -- effective quantize amount (ADAPT polarity via tog9)
            if s_adapt_inv = '0' then
                qeff := resize(s_quant, 12) + resize(adapt_delta(11 downto 0), 12);
            elsif resize(s_quant, 12) > resize(adapt_delta(11 downto 0), 12) then
                qeff := resize(s_quant, 12) - resize(adapt_delta(11 downto 0), 12);
            else
                qeff := (others => '0');
            end if;
            if qeff > 1023 then
                qeff := to_unsigned(1023, 12);
            end if;

            -- QUANTIZE (hard, sequency-weighted deadzone) + TILT (2-band shift)
            for i in 0 to N-1 loop
                c := s_f2(i);

                -- QUANTIZE: hard deadzone, threshold = qeff*SEQ(i) >> 3.
                -- SEQ(i) is a compile-time constant per unrolled lane, so the
                -- product is a constant-multiply (shift/add) not a DSP.
                thr := resize(shift_right(qeff * to_unsigned(SEQ(i), 5), 3), 20);
                if abs(c) <= signed(resize(thr, FW)) then
                    qc := (others => '0');
                else
                    qc := c;
                end if;

                -- TILT: 2-band (bass = SEQ<8, treble = SEQ>=8) shift tilt
                if s_tilt(9) = '1' then          -- >= 512: cut bass
                    bshift := to_integer(shift_right(s_tilt - 512, 7));
                    if SEQ(i) < 8 then sh := bshift; else sh := 0; end if;
                else                              -- < 512: cut treble
                    bshift := to_integer(shift_right(512 - s_tilt, 7));
                    if SEQ(i) >= 8 then sh := bshift; else sh := 0; end if;
                end if;
                tc := shift_right(qc, sh);

                s_m1(i) <= tc;
            end loop;
        end if;
    end process;

    -- ====================================================================
    -- Stage M2 : SCRAMBLE (XOR permute) + REGEN (vertical feedback, saturating,
    --            loop gain < 1).  Also loads the write-back serializer.
    --            (CROSSTALK removed with chroma -- Knob5 is reserved/inert.)
    -- ====================================================================
    p_m2 : process(clk)
        variable mask  : integer range 0 to 15;
        variable sc    : t_fwd;
        variable fb    : signed(FW-1 downto 0);
        variable rgsh  : integer range 0 to 8;
        variable rg_en : boolean;
    begin
        if rising_edge(clk) then
            mask := to_integer(s_scram(9 downto 6));       -- 0..15 (0 = identity)
            rg_en := (s_regen(9 downto 7) /= "000");
            if rg_en then rgsh := 8 - to_integer(s_regen(9 downto 7)); else rgsh := 0; end if;

            -- SCRAMBLE: permute lanes by index XOR mask (bijective, 16:1 mux)
            for i in 0 to N-1 loop
                sc(i) := s_m1(to_integer(
                    to_unsigned(i, 4) xor to_unsigned(mask, 4)));
            end loop;

            -- REGEN: add gain*(prev-line stored coeff), saturate (loop gain < 1)
            for i in 0 to N-1 loop
                if rg_en then
                    fb := shift_right(s_prev_blk(i), rgsh);
                else
                    fb := (others => '0');
                end if;
                s_m2(i) <= sat_coef(resize(sc(i), FW+2) + resize(fb, FW+2));
            end loop;
        end if;
    end process;

    -- ====================================================================
    -- Stage I1/I2 : inverse WHT (2 ranks/clk) + >>4 normalize + P2S load
    -- ====================================================================
    p_inv : process(clk)
        variable vinv  : t_inv;
        variable par_i : integer range 0 to 1;
    begin
        if rising_edge(clk) then
            -- I1: inverse ranks 1,2 (19-bit datapath)
            s_i1 <= inv_rank(inv_rank(to_inv(s_m2), 1), 2);

            -- I2: inverse ranks 3,4 + >>4 normalize + de-center + clamp,
            -- loaded into the double-buffered P2S bank selected by block parity
            par_i := b2i(s_par(6));
            vinv  := inv_rank(inv_rank(s_i1, 4), 8);
            for i in 0 to N-1 loop
                if s_v(6) = '1' then
                    s_p2s(par_i)(i) <= decen(resize(shift_right(vinv(i), 4), FW));
                end if;
            end loop;
        end if;
    end process;

    -- ====================================================================
    -- REGEN Y coefficient line buffer (1W1R dual-port; read this block, write
    -- the trailing block -> single storage, no replication).  Spectral Hold
    -- (tog10) freezes writes so the vertical-feedback spectrum persists.
    -- ====================================================================
    p_cbuf : process(clk)
    begin
        if rising_edge(clk) then
            -- write port (trailing block) -- gated by Spectral Hold
            if s_wr_en = '1' and s_hold = '0' then
                cbuf_y(to_integer(s_wr_addr)) <= s_wr_y;
            end if;
            -- read port (current block being gathered), registered
            s_cbuf_y_q <= cbuf_y(to_integer(s_apx));
        end if;
    end process;

    -- assemble prev-line coeff bank (read latency = 1) and latch per block
    p_prev : process(clk)
    begin
        if rising_edge(clk) then
            s_apx_d1 <= s_apx;
            s_prev(to_integer(s_apx_d1(3 downto 0))) <= unpack_y(s_cbuf_y_q);
            if data_in.avid = '1' and s_ph = 15 then
                s_prev_blk <= s_prev;
            end if;
        end if;
    end process;

    -- write-back serializer: streams a block's post-REGEN coeffs to the buffer
    p_wb : process(clk)
    begin
        if rising_edge(clk) then
            s_wr_en <= '0';
            if s_v(4) = '1' then                 -- M2 result valid -> load
                s_wb_bank <= s_m2;
                s_wb_base <= s_base(4);
                s_wb_cnt  <= (others => '0');
                s_wb_run  <= '1';
            elsif s_wb_run = '1' then
                s_wr_en   <= '1';
                s_wr_addr <= s_wb_base + resize(s_wb_cnt, ABITS);
                s_wr_y    <= pack_y(s_wb_bank(to_integer(s_wb_cnt)));
                if s_wb_cnt = 15 then
                    s_wb_run <= '0';
                else
                    s_wb_cnt <= s_wb_cnt + 1;
                end if;
            end if;
        end if;
    end process;

    -- ====================================================================
    -- Output-side delay lines.  Y wet+dry ride P2S/dry-delay 23 -> fader(4) ->
    -- clamp(1) = 28.  U/V passthrough delayed 27 -> clamp(1) = 28.  Sync 28.
    -- All paths land at exactly 28 -> constant latency for every pixel.
    -- ====================================================================
    p_pre : process(clk)     -- phase/parity + Y dry, delayed C_PRE = 23
        type a4  is array(0 to C_PRE-1) of unsigned(3 downto 0);
        type a1  is array(0 to C_PRE-1) of std_logic;
        type a10 is array(0 to C_PRE-1) of std_logic_vector(CW-1 downto 0);
        variable ph_d  : a4  := (others => (others => '0'));
        variable par_d : a1  := (others => '0');
        variable y_d   : a10 := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            ph_d  := s_ph       & ph_d(0 to C_PRE-2);
            par_d := s_blk_par  & par_d(0 to C_PRE-2);   -- ping-pong parity (read side)
            y_d   := data_in.y  & y_d(0 to C_PRE-2);
            out_ph_d  <= ph_d(C_PRE-1);
            out_par_d <= par_d(C_PRE-1);
            dry_y     <= y_d(C_PRE-1);
        end if;
    end process;

    p_uv : process(clk)      -- U/V unprocessed passthrough, delayed 27 (-> clamp -> 28)
        type a10 is array(0 to C_LAT-2) of std_logic_vector(CW-1 downto 0);
        variable u_d, v_d : a10 := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            u_d := data_in.u & u_d(0 to C_LAT-3);
            v_d := data_in.v & v_d(0 to C_LAT-3);
            pass_u <= u_d(C_LAT-2);   -- delay 27
            pass_v <= v_d(C_LAT-2);
        end if;
    end process;

    p_sync : process(clk)
        type a1 is array(0 to C_LAT-1) of std_logic;
        variable av : a1 := (others => '0');
        variable hs : a1 := (others => '1');
        variable vs : a1 := (others => '1');
        variable fl : a1 := (others => '1');
    begin
        if rising_edge(clk) then
            av := data_in.avid    & av(0 to C_LAT-2);
            hs := data_in.hsync_n & hs(0 to C_LAT-2);
            vs := data_in.vsync_n & vs(0 to C_LAT-2);
            fl := data_in.field_n & fl(0 to C_LAT-2);
            s_avid_d <= av(C_LAT-1);
            s_hs_d   <= hs(C_LAT-1);
            s_vs_d   <= vs(C_LAT-1);
            s_fld_d  <= fl(C_LAT-1);
        end if;
    end process;

    -- ====================================================================
    -- Stage P : P2S combinational Y scatter -> Y fader endpoints
    -- ====================================================================
    a_y <= unsigned(dry_y);                                           -- dry Y (delay 23)
    b_y <= unsigned(s_p2s(b2i(out_par_d))(to_integer(out_ph_d)));     -- wet Y (delay 23)

    -- ====================================================================
    -- Stage B : Y dry/wet fader (interpolator_u x1, 4-cycle) -- principle #4.
    -- U/V need no blend (dry = wet); they ride the 27-delay straight to clamp.
    -- ====================================================================
    mix_y_inst : entity work.interpolator_u
        generic map(G_WIDTH => CW, G_FRAC_BITS => CW,
                    G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(clk => clk, enable => '1', a => a_y, b => b_y,
                 t => s_fader, result => mix_y, valid => mix_vy);

    -- ====================================================================
    -- Stage C : broadcast-safe clamp R1 (Y[64,940], U/V[64,960])
    -- ====================================================================
    p_clamp : process(clk)
    begin
        if rising_edge(clk) then
            s_out_y <= clampr(mix_y, 64, 940);
            s_out_u <= clampr(unsigned(pass_u), 64, 960);
            s_out_v <= clampr(unsigned(pass_v), 64, 960);
        end if;
    end process;

    -- ====================================================================
    -- Output assignment (sync delayed EXACTLY 28)
    -- ====================================================================
    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= s_avid_d;
    data_out.hsync_n <= s_hs_d;
    data_out.vsync_n <= s_vs_d;
    data_out.field_n <= s_fld_d;

end sequency;
