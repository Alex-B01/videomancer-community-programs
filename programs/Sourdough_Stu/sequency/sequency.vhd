-- Copyright (C) 2026 AB (Sourdough_Stu)
-- SPDX-License-Identifier: GPL-3.0-only
-- File: sequency.vhd - SEQUENCY: the Walsh-Hadamard organ (Program 5)
--
-- This file is free software under the GNU General Public License v3.0.
-- https://github.com/lzxindustries/videomancer-community-programs
--
-- Program Name:
--   Sequency  (Y-ONLY, R2SDF streaming WHT — REV-F v1.0 completion, latency 56)
--
-- Author:
--   AB (Sourdough_Stu)
--
-- Overview:
--   16-point HORIZONTAL Walsh-Hadamard blocks on the Y (luma) channel, computed
--   by R2SDF (Radix-2 Single-path Delay Feedback): a streaming transform, 1
--   px/clk, ZERO multipliers. Data flows through 4 forward stages (delay depths
--   D=8,4,2,1), a serial quantizer / coefficient-play region, and 4 inverse
--   stages (D=1,2,4,8). H*Ht = 16*I -> >>4 normalize. Coefficients are "played"
--   one per clock between forward and inverse: QUANTIZE (sequency-weighted hard
--   deadzone, barrel-shift + per-sequency LUT), TILT (runtime per-sequency
--   attenuation, knob 2), SCRAMBLE (coefficient-index XOR), REGEN (vertical
--   coefficient feedback via a line buffer, saturating, freeze-during-blank),
--   ADAPT (Y block-energy -> deadzone threshold). Blended dry/wet via one
--   interpolator_u on Y.
--   No framebuffer, no temporal state -> natively all-formats. Block index =
--   avid-anchored mod-16 counter (720/1280/1920 all divide by 16 exactly).
--
--   Y-ONLY topology: a single wht_organ_lane on Y with the FULL feature set
--   (QUANTIZE/TILT/SCRAMBLE/REGEN/ADAPT + dry/wet fader). U and V are PASSTHROUGH,
--   delay-matched to the Y organ+fader latency via the shared BRAM circular delay
--   + a fixed interpolator/leak alignment tail. The U/V organ lanes and the packed
--   U/V REGEN coefficient buffer are REMOVED. Shared control-derive (TILT
--   strength/dir, SCRAMBLE key, ADAPT threshold, and the precomputed QUANTIZE
--   deadzone thresholds q_th0..3) lives in the top and drives the one Y lane.
--
--   REV-F (v1.0 completion pass) — the four formerly-dead controls are now LIVE:
--     Knob 5  CROSSTALK : output-stage Y->C effect-residual leak. Injects
--                         k*(Y_out - Y_dry) into U/V, OPPOSITE-signed (NTSC
--                         cross-color shimmer, sync_bleed S6 precedent). Bipolar,
--                         center 512 = OFF; k = top-3-bits-of-|distance| -> BARREL
--                         SHIFT (no multiply); polarity from the knob's side.
--     Tog 11  CHROMA    : hard-off master enable for the leak ("Luma only" = off /
--                         "Chroma too" = on). Default = "Luma only" (leak off at load).
--     Tog 7   DITHERED  : threshold-scaled free-running-LFSR dither on the QUANTIZE
--                         deadzone-edge compare (DC exempt; Hard = bit-exact REV-E).
--     Tog 8   ROTATE    : 4-bit barrel-rotate of the coefficient index (alt to XOR;
--                         index 0 -> 0 keeps DC home). All 12 controls now functional.
--   Load-neutral: the residual is identically 0 at fader-dry OR effect-neutral, so
--   U/V stay bit-exact regardless of Knob 5 position. The leak is single-lane FF +
--   combinational (NOT the deferred chroma organ) and adds ZERO memory.
--
--   Scope note: the full-4:4:4-chroma build (3x wht_organ_lane) was congestion-
--   bound (~88% fill, Fmax stuck at 56.5->62.9->66.1 MHz across 3 timing
--   iterations); reverted to Y-only — the comfortable, proven scope — by user
--   decision (cut losses on full-chroma HD timing).
--
-- Pipeline stages + total latency:
--   Organ core (C_PROCESSING_DELAY_CLKS = 48, /4):
--     R   input register ................................. 1  (cum 1)
--     F1  forward SDF D=8  (W=12) ........................ 9  (cum 10)
--     F2  forward SDF D=4  (W=13) ........................ 5  (cum 15)
--     F3  forward SDF D=2  (W=14) ........................ 3  (cum 18)
--     F4  forward SDF D=1  (W=15) -> coeff stream ........ 2  (cum 20)
--     Q1  capture + TILT(atten) + SCRAMBLE-idx (XOR/Rot) +
--         ADAPT tap ...................................... 1  (cum 21)
--     Q2a REGEN BRAM-read isolation: register ram_y read +
--         pre-shift/gate feedback term (Gate-1 timing fix)  1  (cum 22)
--     Q2b REGEN add (TILT+REGEN, 2-input) + R3 sat + store  1  (cum 23)
--     Q3  QUANTIZE deadzone: band-select precomputed thr
--         (q_th0..3) + magnitude compare, DC exempt,
--         + optional threshold-scaled LFSR dither (Tog 7)  1  (cum 24)
--     I1  inverse SDF D=1  (W=17, widened +1) ............ 2  (cum 26)
--     I2  inverse SDF D=2  (W=18) ........................ 3  (cum 29)
--     I3  inverse SDF D=4  (W=19) ........................ 5  (cum 34)
--     I4  inverse SDF D=8  (W=20) -> reconstructed x16 ... 9  (cum 43)
--     S   +8 round, >>4 normalize, R1 broadcast clamp .... 2  (cum 45)
--     P   /4 sync-discipline pad (pure delay) ............ 3  (cum 48)
--   Dry/wet fader (interpolator_u, Y only) -> Y_out ...... 4  (cum 52)
--   REV-F Y->C residual leak (U/V path):
--     L1  residual = Y_out - Y_dry(52), master-gated ..... 1  (cum 53)
--     L2  k barrel-shift (top-3 knob bits; NO multiply) .. 1  (cum 54)
--     L3  opposite-signed inject (U +=/-= , V mirror) .... 1  (cum 55)
--     L4  R1 clamp [64,960] -> broadcast-safe U/V ........ 1  (cum 56)
--   Faded-Y +4 pure-delay pipe (52 -> 56, realign w/ leak) 4  -> total 56
--   => C_PROCESSING_DELAY_CLKS = 48 (lane core incl. Q2a + /4 pad),
--      C_SYNC_DELAY_CLKS = 56 (= 48 + interpolator 4 + leak retime 4; all /4).
--      Sync + dry-Y + U/V passthrough ride the BRAM circular delay (48); dry-Y
--      feeds the Y fader (-> 52), and the full passthrough word (dryY/U/V/sync)
--      rides a fixed +4 tail to 52 then the 4-clock leak stage to 56, while the
--      faded Y is +4-padded to 56 — every data_out port aligned at 56, FIXED and
--      independent of every knob/toggle/fader (R5/R8). The leak is post-fader,
--      top-level, so the organ core stays 48 (untouched). The 64-deep BRAM delay
--      (3 EBR) covers latency 56 with margin 8 -> no EBR change from the retime.
--      GATE-1 TIMING FIX (this build): the pre-fix organ was 44 (end-to-end 48)
--      but Fmax stuck at 70.97 < 76 on the INTRINSIC REGEN Q2 store path (ram_y
--      BRAM read -> 18b add -> R3 double-saturate -> reg, one clock). Q2 is now
--      split: Q2a isolates the BRAM read (+ pre-shifts the feedback term) so the
--      Q2b add + double-saturate start from fabric FFs. +1 real stage (organ 45);
--      a +3 /4 pad -> lane 48, end-to-end 52 (both /4). REGEN is line-granular
--      (~2048-clk loop) so the +1 read->write clock is behaviorally negligible;
--      round-trip stays bit-exact at neutral / QUANTIZE=0. Gate 2 re-verifies.
--
-- EBR map (physical, power-of-two) — 9 EBR total:
--   REGEN Y coeff line buffer   2048 x 12  (2048x2 mode)  1W/1R  = 6 EBR
--   Sync + dry-Y + U/V delay     64  x 34  (256x16 mode)  1W/1R  = 3 EBR
--     (dryY10 + passU10 + passV10 + avid/hs/vs/fld = 34 bits, width-limited)
--   -------------------------------------------------------------------------
--   TOTAL = 6 + 3 = 9 EBR   (budget MAX_EBR 20, device 32)
--   The packed U+V REGEN buffer (8 EBR) is REMOVED with the U/V lanes. The
--   delay stays 3 EBR (not the reference prototype's 2) because it also carries
--   the Y fader's dry tap (dryY 10b) alongside U/V+sync (34b > 32b -> 3 EBR);
--   sequency_sdf had no fader so its delay was 24b/2 EBR. 9 << 14 budget.
--   REV-F adds ZERO memory: the Crosstalk leak, the dither LFSR, and the barrel-
--   rotate are all FF + combinational; the leak's Y_dry reference is the dryY
--   already carried in the 34-bit delay word (no new port). The 64-deep delay
--   still covers latency 56 (margin 8) -> EBR unchanged at 9.
--
-- LC estimate (honest, paper): Y-only single wht_organ_lane (measured 1-lane
--   R2SDF skeleton ~1,500 LC + levered feature deltas ~1,700 for
--   REGEN/TILT/SCRAMBLE/ADAPT/qth-precompute + one interpolator_u fader + the
--   delay/tail/REGEN-address glue) => ~3,700 LC paper. Multipliers in the organ:
--   0 (all shift/add). The only * in the design is inside the single SDK
--   interpolator_u (sanctioned Y fader). Zero runtime multiplies elsewhere.
--   REDUCTION RATIONALE: the full-4:4:4 3-lane build was congestion-bound at
--   ~88% fill — Fmax climbed 56.5 -> 62.9 -> 66.1 MHz across three timing
--   iterations (CROSSTALK removal, then the Q3 threshold hoist) but never
--   cleared the 76 MHz HD gate; the failing endpoints clustered on the packed
--   U+V REGEN store (ram_uv BRAM read -> 18b add + R3 sat -> write). User cut
--   losses on full-chroma HD timing and reverted to the proven Y-only scope.
--   Dropping the U/V organ lanes + the ram_uv buffer drops fill 88% -> 48% and
--   frees the congestion, BUT the REGEN feedback store was INTRINSIC, not
--   congestion: the pre-fix Y-only smoke (hd_hdmi, seed 1) = 3,698 LC (48%) /
--   9 EBR / Fmax 70.97 MHz — LC & EBR comfortable, but Fmax < the 76 gate on the
--   Y REGEN Q2 store: ram_y BRAM read (2.15 ns clk-to-q) -> v_sum 18b add + R3
--   double-saturate (s_c2 16b & s_cur_y 12b) -> register, one clock (pre-route
--   74.75, also < 76 -> not seed noise).
--   GATE-1 TIMING FIX (this build, approved Y-only scope): the Q2 stage is split
--   — Q2a registers the ram_y read and pre-shifts/gates the feedback term OFF the
--   arithmetic; Q2b does the add + double-saturate from fabric FFs (no BRAM
--   clk-to-q, no barrel shift on that path). +1 real stage (organ 44->45) + a +3
--   /4-discipline pad -> lane 48, end-to-end 52 (both /4). Costs ~0.1k LC (Q2a +
--   pad + write-pipeline FFs); EBR unchanged (9). Post-fix smoke (hd_hdmi, seed
--   1): 3,782 LC (49%) / 9 EBR / Fmax 89.85 MHz (+13.85 over the 76 gate) —
--   comfortable clearance, as predicted at ~48% fill; the REGEN store is no
--   longer the wall. The shared-threshold hoist (s_q_th -> q_th0..3) is KEPT.
--   REV-F COMPLETION DELTA: four now-live controls add small serial add/sub/shift
--   + a 13-bit LFSR + the leak/Y-pad register banks (4 leak stages carrying
--   U/V/sync + 4 Y-delay + a widened 34b tail) + a dithered-threshold register bank
--   at the top. No new structural class, no memory, zero runtime multiply (k is a
--   barrel shift). RE-SMOKE (hd_hdmi, seed 1): 4,250 LC (55%) / 9 EBR / Fmax 87.54
--   MHz (-2.31 vs the REV-E 89.85; critical path = the SDK interpolator multiply,
--   NOT the leak). LC is +468 over the measured base (above the +105-190 paper
--   band -- the leak uses shallow Fmax-protective stages + the top dither adds a
--   second threshold bank), still comfortable: ~1,250 LC under the 5,500 target.
--   Latency 52 -> 56 (/4). Gate-3 owns the worst-seed re-confirmation.
--
-- Register map (SPI: pots -> registers_in(0..5), toggles -> (6) bits 0..4,
--               fader -> (7), timing ID -> (8)(3 downto 0) [unused: all-formats]):
--   Knob 1  registers_in(0)     QUANTIZE   sequency-weighted deadzone
--   Knob 2  registers_in(1)     TILT       bass<->treble, attenuate-only (512=flat)
--   Knob 3  registers_in(2)     SCRAMBLE   basis permutation depth (index XOR/Rotate)
--   Knob 4  registers_in(3)     REGEN      vertical coefficient feedback
--   Knob 5  registers_in(4)     CROSSTALK  LIVE: Y->C residual leak (bipolar, 512=OFF;
--                                          k = top-3-bits -> barrel shift, no multiply)
--   Knob 6  registers_in(5)     ADAPT      energy -> quantizer self-mod (Y-driven)
--   Tog 7   registers_in(6)(0)  Quantizer  LIVE: Hard / Dithered (LFSR deadzone-edge)
--   Tog 8   registers_in(6)(1)  ScrMode    LIVE: XOR / Rotate (rotate 0->0 keeps DC)
--   Tog 9   registers_in(6)(2)  ADAPT pol  Normal/Inverted
--   Tog 10  registers_in(6)(3)  SpecHold   Live/Hold
--   Tog 11  registers_in(6)(4)  Chroma     LIVE: leak master (Luma only=off / Chroma too=on)
--   Fader   registers_in(7)     Dry/Wet    interpolator_u crossfade (gates the leak)
--
-- No resets (iCE40 has none): every register/BRAM relies on signal init.
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- wht_organ_lane : one complete R2SDF Walsh-Hadamard organ. Generic/reusable;
--   in the Y-only build it is instantiated ONCE (on Y) — U/V are passthrough.
--   Forward SDF -> Q-region (feature apply) -> inverse SDF -> scale. Seams:
--     coef_tap  : this lane's Q1 (tilted) coefficient — ADAPT tap (Y lane only;
--                 U/V leave it open)
--     regen_prev: previous-line stored coefficient (RAM owned by top, packable)
--     regen_cur : this-line coefficient to store (post REGEN, saturated)
--   Generics select the REGEN store width (12 = Y full-res, 8 = U/V packed) and
--   the R1 broadcast-safe output clamp (Y [64,940], U/V [64,960]).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity wht_organ_lane is
  generic (
    G_REGEN_W  : integer := 12;   -- REGEN stored coeff width (Y=12, U/V=8)
    G_CLAMP_LO : integer := 64;   -- R1 output floor
    G_CLAMP_HI : integer := 940   -- R1 output ceiling (Y=940, U/V=960)
  );
  port (
    clk         : in  std_logic;
    pos         : in  unsigned(3 downto 0);           -- avid-anchored mod-16 index
    -- shared control (derived once at top; identical to all 3 lanes)
    -- QUANTIZE deadzone thresholds, precomputed per C_QSHIFT band (qeff<<k +
    -- ADAPT, clamped >=0) and OPTIONALLY dithered at the top (Tog 7 subtracts a
    -- band-scaled LFSR sample). Block-stable -> hoisted out of the per-coefficient
    -- Q3 critical path at the top; the lane just band-selects + compares.
    q_th0       : in  signed(17 downto 0);            -- band k=0 threshold (opt. dithered)
    q_th1       : in  signed(17 downto 0);            -- band k=1 threshold
    q_th2       : in  signed(17 downto 0);            -- band k=2 threshold
    q_th3       : in  signed(17 downto 0);            -- band k=3 threshold
    scr_key     : in  unsigned(3 downto 0);           -- SCRAMBLE index key (XOR key / rotate amount)
    scr_mode    : in  std_logic;                      -- Tog 8: 0 = XOR, 1 = 4-bit barrel-Rotate (0->0 keeps DC)
    tilt_dir    : in  std_logic;                      -- 0 = cut highs (bass), 1 = cut lows
    tilt_str    : in  unsigned(1 downto 0);           -- runtime TILT strength 0..3
    regen_sh    : in  unsigned(1 downto 0);           -- REGEN feedback right-shift-1 (gain <=1/2)
    regen_en    : in  std_logic;                      -- REGEN enable (knob4 > 0)
    hold        : in  std_logic;                      -- Spectral Hold (freeze coeff source)
    -- REGEN storage seam (line-buffer RAM lives in the top so U/V can pack)
    regen_prev  : in  signed(G_REGEN_W - 1 downto 0);
    regen_cur   : out signed(G_REGEN_W - 1 downto 0);
    -- ADAPT tap seam (Y lane only; U/V leave it open)
    coef_tap    : out signed(15 downto 0);            -- Q1 tilted coefficient
    -- video datapath
    d_in        : in  std_logic_vector(9 downto 0);
    d_out       : out std_logic_vector(9 downto 0)
  );
end entity wht_organ_lane;

architecture rtl of wht_organ_lane is

  -- ------- interconnect buses (20-bit carries fwd 12..15 and inv 17..20) -----
  type t_bus is array (natural range <>) of signed(19 downto 0);
  signal s_d_in  : signed(10 downto 0) := (others => '0');   -- 10u -> 11s
  signal f_in    : t_bus(1 to 4) := (others => (others => '0'));
  signal f_out   : t_bus(1 to 4) := (others => (others => '0'));
  signal i_in    : t_bus(1 to 4) := (others => (others => '0'));
  signal i_out   : t_bus(1 to 4) := (others => (others => '0'));

  -- ------- Q-region registers (coeff/quant path widened 15 -> 16b) -----------
  signal s_c_tilt : signed(15 downto 0) := (others => '0');  -- Q1: forward coeff after TILT
  signal s_c2     : signed(15 downto 0) := (others => '0');  -- Q2b: post REGEN (R3 sat)
  signal s_c3     : signed(15 downto 0) := (others => '0');  -- Q3: post QUANTIZE deadzone
  signal s_seq1   : unsigned(3 downto 0) := (others => '0'); -- true sequency, pipelined
  signal s_seq2   : unsigned(3 downto 0) := (others => '0');

  -- Q2a REGEN BRAM-read isolation registers (Gate-1 timing fix): capture the
  -- ram_y read (regen_prev) + the pre-shifted/pre-gated feedback term OFF the
  -- arithmetic, so the Q2b add starts from fabric FFs, not the BRAM output.
  -- Breaks the 2.15 ns ram_y clk-to-q off the 18b add + double-saturate
  -- (hardware-constraints Constraint 4). +1 pipeline stage (line-granular REGEN,
  -- ~2048-clk loop -> behaviorally negligible; Gate 2 re-verifies).
  signal s_regen_term : signed(17 downto 0) := (others => '0');           -- shifted+gated feedback
  signal s_prev_y_r   : signed(G_REGEN_W - 1 downto 0) := (others => '0');-- isolated read (Hold replay)
  signal s_c_tilt_d   : signed(15 downto 0) := (others => '0');           -- pipelined tilt coeff
  signal s_seq1a      : unsigned(3 downto 0) := (others => '0');          -- pipelined sequency
  signal s_hold_d     : std_logic := '0';                                -- pipelined Spectral Hold

  signal s_scaled : signed(19 downto 0) := (others => '0');
  signal s_d_out  : std_logic_vector(9 downto 0) := (others => '0');

  -- P : /4 sync-discipline pad. The REGEN BRAM-read isolation (Q2a) adds 1 real
  --     stage -> lane core 45; pad +3 so lane latency = 48 (still /4) and the
  --     end-to-end sync depth stays /4 at 52 (hard rule). 3 x 10b = 30 FF.
  type t_dpad is array (0 to 2) of std_logic_vector(9 downto 0);
  signal s_dpad : t_dpad := (others => (others => '0'));

  -- ------- QUANTIZE per-sequency deadzone weighting (constant LUT) -----------
  -- Effective deadzone = qeff << C_QSHIFT[seq_eff]: high sequencies starve first.
  -- C_QSHIFT now selects which of the top's 4 precomputed thresholds (q_th0..3)
  -- to compare against — the shift+add that built each threshold is hoisted.
  type t_qsh is array (0 to 15) of integer range 0 to 3;
  constant C_QSHIFT : t_qsh := (0,0,1,1, 2,2,2,2, 3,3,3,3, 3,3,3,3);

  -- ------- stage control offsets (latency from s_d_in reg to each stage) -----
  constant OFF_F1 : integer := 0;  constant OFF_F2 : integer := 9;
  constant OFF_F3 : integer := 14; constant OFF_F4 : integer := 17;
  constant OFF_Q  : integer := 19;
  -- Inverse offsets are +1 vs the pre-fix build: the Q2a REGEN BRAM-read
  -- isolation stage delays the coefficient stream reaching the inverse SDF by
  -- exactly one clock, so each inverse butterfly's mod-16 control phase shifts +1.
  constant OFF_I1 : integer := 23; constant OFF_I2 : integer := 25;
  constant OFF_I3 : integer := 28; constant OFF_I4 : integer := 33;

  -- R2SDF stage control: select bit = bit log2(D) of (pos - stage offset).
  function ctl(pos : unsigned(3 downto 0); off, bitn : integer) return std_logic is
    variable p : unsigned(3 downto 0);
  begin
    p := pos - to_unsigned(off mod 16, 4);
    return p(bitn);
  end function;

  -- Saturate a signed value into a w-bit signed range (R3, no wrap).
  function sat_s(v : signed; w : natural) return signed is
    constant HI : integer := 2**(w - 1) - 1;
    constant LO : integer := -(2**(w - 1));
  begin
    if v > to_signed(HI, v'length) then
      return to_signed(HI, w);
    elsif v < to_signed(LO, v'length) then
      return to_signed(LO, w);
    else
      return resize(v, w);
    end if;
  end function;

  -- Runtime TILT: per-sequency ATTENUATE-ONLY right-shift (0..3). str=0 => flat.
  -- Cannot amplify -> cannot overflow -> no datapath growth (spec D4).
  function tilt_amt(seq_eff : unsigned(3 downto 0);
                    dir : std_logic; str : unsigned(1 downto 0)) return integer is
    variable band : integer;  -- 0..3
    variable dist : integer;
    variable sh   : integer;
  begin
    band := to_integer(seq_eff(3 downto 2));
    if dir = '0' then
      dist := 3 - band;      -- bass tilt: high bands cut most
    else
      dist := band;          -- treble tilt: low bands cut most
    end if;
    sh := to_integer(str) - dist;
    if sh < 0 then sh := 0; end if;
    return sh;
  end function;

  -- SCRAMBLE index permutation (Tog 8). XOR = full basis chaos (a non-zero key
  -- relocates DC at depth); Rotate = 4-bit barrel-rotate = structured rebasing
  -- that maps index 0 -> 0, so the DC coefficient stays home (brightness kept).
  -- key 0 -> identity in BOTH modes (Scramble knob at 0 is neutral either way).
  function scramble_idx(seq : unsigned(3 downto 0);
                        key : unsigned(3 downto 0);
                        mode : std_logic) return unsigned is
  begin
    if mode = '0' then
      return seq xor key;                                     -- XOR permutation
    else
      return rotate_left(seq, to_integer(key(3 downto 2)));   -- rotate 0..3 (0->0)
    end if;
  end function;

begin

  -- ==========================================================================
  -- R : input register (10u -> 11s; DC bias rides coeff 0, round-trips exact)
  -- ==========================================================================
  p_in : process(clk)
  begin
    if rising_edge(clk) then
      s_d_in <= signed(resize(unsigned(d_in), 11));
    end if;
  end process;

  f_in(1) <= resize(s_d_in, 20);

  -- ==========================================================================
  -- F1..F4 : forward SDF stages  D = 8,4,2,1  (widths 12..15, UNCHANGED)
  -- ==========================================================================
  g_fwd : for k in 1 to 4 generate
    constant D    : integer := 2 ** (4 - k);
    constant BITN : integer := 3 - (k - 1);
    constant OFF  : integer_vector(1 to 4) := (OFF_F1, OFF_F2, OFF_F3, OFF_F4);
    constant W    : integer := 11 + k;
  begin
    b_stage : block
      type t_d is array (0 to D - 1) of signed(W - 1 downto 0);
      signal dl : t_d := (others => (others => '0'));
      signal o  : signed(W - 1 downto 0) := (others => '0');
    begin
      p_s : process(clk)
        variable head : signed(W - 1 downto 0);
        variable c    : std_logic;
      begin
        if rising_edge(clk) then
          head := dl(D - 1);
          c    := ctl(pos, OFF(k), BITN);
          if c = '0' then
            dl(0) <= resize(f_in(k)(W - 2 downto 0), W);
            o     <= head;
          else
            dl(0) <= head - resize(f_in(k)(W - 2 downto 0), W);
            o     <= head + resize(f_in(k)(W - 2 downto 0), W);
          end if;
          for j in 1 to D - 1 loop
            dl(j) <= dl(j - 1);
          end loop;
        end if;
      end process;
      f_out(k) <= resize(o, 20);
    end block;
  end generate;

  f_in(2) <= f_out(1);  f_in(3) <= f_out(2);  f_in(4) <= f_out(3);

  -- ==========================================================================
  -- Q1 : capture forward coeff, apply runtime TILT (attenuate-only), scramble
  --      the parameter index, and expose the Q1 tap (ADAPT; Y lane only).
  -- ==========================================================================
  p_q1 : process(clk)
    variable v_seq  : unsigned(3 downto 0);
    variable v_seff : unsigned(3 downto 0);
    variable v_tsh  : integer;
  begin
    if rising_edge(clk) then
      v_seq  := pos - to_unsigned(OFF_Q mod 16, 4);   -- true sequency of this coeff
      v_seff := scramble_idx(v_seq, scr_key, scr_mode); -- SCRAMBLE (XOR or Rotate)
      v_tsh  := tilt_amt(v_seff, tilt_dir, tilt_str);
      s_c_tilt <= shift_right(resize(f_out(4), 16), v_tsh);  -- >> only (no overflow)
      s_seq1   <= v_seq;
    end if;
  end process;

  coef_tap <= s_c_tilt;   -- Q1 register out; ADAPT tap (top consumes Y lane only)

  -- ==========================================================================
  -- Q2a : REGEN BRAM-read isolation (Gate-1 timing fix — hardware-constraints
  --       Constraint 4). Register the ram_y read (regen_prev) and PRE-compute the
  --       shifted + gated feedback term one clock early, so the Q2b add + double-
  --       saturate below start from fabric FFs instead of the BRAM output — this
  --       lifts the 2.15 ns ram_y clk-to-q (and the barrel shift) off the 18b add
  --       + double-saturate critical path. Coeff / sequency / Hold are pipelined
  --       alongside so Q2b stays aligned. This +1 stage is behaviorally negligible
  --       (REGEN is line-granular, ~2048-clk loop); Gate 2 re-verifies semantics.
  -- ==========================================================================
  p_q2a : process(clk)
  begin
    if rising_edge(clk) then
      -- REGEN feedback term: prev-line stored coeff >> (regen_sh+1) (gain <= 1/2),
      -- gated (DC seq 0 / REGEN off -> 0). Pre-shifted here, OFF the Q2b adder.
      if regen_en = '1' and s_seq1 /= 0 then
        s_regen_term <= shift_right(resize(regen_prev, 18),
                                    to_integer(regen_sh) + 1);
      else
        s_regen_term <= (others => '0');
      end if;
      s_prev_y_r <= regen_prev;   -- isolated read, retained for the Hold replay
      s_c_tilt_d <= s_c_tilt;
      s_seq1a    <= s_seq1;
      s_hold_d   <= hold;
    end if;
  end process;

  -- ==========================================================================
  -- Q2b : REGEN vertical feedback add (2-input: TILT coeff + isolated feedback
  --       term), R3 saturating clamp to 16b, store the resonant AC coefficient
  --       (reduced precision), Spectral-Hold source select. DC (true seq 0) is
  --       excluded from feedback + store. Inputs are all Q2a fabric FFs (no BRAM
  --       clk-to-q, no barrel shift on this path). (CROSSTALK inject removed.)
  -- ==========================================================================
  p_q2b : process(clk)
    variable v_sum : signed(17 downto 0);
  begin
    if rising_edge(clk) then
      v_sum := resize(s_c_tilt_d, 18) + s_regen_term;  -- 2-input add, fabric inputs

      if s_hold_d = '1' then
        -- Spectral Hold: replay the frozen (stored) coefficient into the inverse
        s_c2 <= resize(s_prev_y_r, 16);
      else
        s_c2 <= sat_s(v_sum, 16);                 -- R3 clamp, coeff path 16b
      end if;

      -- store resonant coeff for next line (AC only; saturated to store width)
      if s_seq1a = 0 then
        regen_cur <= (others => '0');
      else
        regen_cur <= sat_s(v_sum, G_REGEN_W);
      end if;

      s_seq2 <= s_seq1a;
    end if;
  end process;

  -- ==========================================================================
  -- Q3 : QUANTIZE hard deadzone. Band-select the (optionally top-dithered)
  --      threshold q_th0..3 by C_QSHIFT[scrambled seq], compare |s_c2|, select.
  --      Single-stage band-select + compare (REV-E form) -> holds Fmax: the Tog-7
  --      LFSR dither and its band-scaled SUBTRACT live at the TOP (block-stable,
  --      slack), so this per-coefficient clock adds NOTHING over REV-E except the
  --      scramble 2:1 mux. Reformulated mag >= (thr - noise): an 18b add in series
  --      with the 18b compare in ONE clock crashed Fmax to 57 MHz. DC (true seq 0)
  --      exempt -> QUANTIZE=0 keeps everything (identity core) and DC is bit-
  --      identical Hard vs Dithered. In Hard mode q_th* = the plain band threshold
  --      -> bit-exact REV-E.
  -- ==========================================================================
  p_q3 : process(clk)
    variable v_seff : unsigned(3 downto 0);
    variable v_th   : signed(17 downto 0);
    variable v_mag  : unsigned(15 downto 0);
    variable v_keep : std_logic;
  begin
    if rising_edge(clk) then
      v_seff := scramble_idx(s_seq2, scr_key, scr_mode);
      case C_QSHIFT(to_integer(v_seff)) is           -- select (opt. dithered) band thr
        when 0      => v_th := q_th0;
        when 1      => v_th := q_th1;
        when 2      => v_th := q_th2;
        when others => v_th := q_th3;
      end case;
      v_mag := unsigned(abs(s_c2));

      if s_seq2 = 0 then
        v_keep := '1';                                    -- DC exempt (identity core)
      elsif resize(signed('0' & std_logic_vector(v_mag)), 18) >= v_th then
        v_keep := '1';
      else
        v_keep := '0';
      end if;

      if v_keep = '1' then s_c3 <= s_c2; else s_c3 <= (others => '0'); end if;
    end if;
  end process;

  i_in(1) <= resize(s_c3, 20);

  -- ==========================================================================
  -- I1..I4 : inverse SDF stages  D = 1,2,4,8  (widths 17..20, WIDENED +1, A6)
  -- ==========================================================================
  g_inv : for k in 1 to 4 generate
    constant D    : integer := 2 ** (k - 1);
    constant BITN : integer := k - 1;
    constant OFF  : integer_vector(1 to 4) := (OFF_I1, OFF_I2, OFF_I3, OFF_I4);
    constant W    : integer := 16 + k;                    -- 17,18,19,20 (was 16..19)
  begin
    b_stage : block
      type t_d is array (0 to D - 1) of signed(W - 1 downto 0);
      signal dl : t_d := (others => (others => '0'));
      signal o  : signed(W - 1 downto 0) := (others => '0');
    begin
      p_s : process(clk)
        variable head : signed(W - 1 downto 0);
        variable c    : std_logic;
      begin
        if rising_edge(clk) then
          head := dl(D - 1);
          c    := ctl(pos, OFF(k), BITN);
          if c = '0' then
            dl(0) <= resize(i_in(k)(W - 2 downto 0), W);
            o     <= head;
          else
            dl(0) <= head - resize(i_in(k)(W - 2 downto 0), W);
            o     <= head + resize(i_in(k)(W - 2 downto 0), W);
          end if;
          for j in 1 to D - 1 loop
            dl(j) <= dl(j - 1);
          end loop;
        end if;
      end process;
      i_out(k) <= resize(o, 20);
    end block;
  end generate;

  i_in(2) <= i_out(1);  i_in(3) <= i_out(2);  i_in(4) <= i_out(3);

  -- ==========================================================================
  -- S : +8 round, >>4 normalize (H*Ht = 16*I), R1 broadcast-safe clamp
  -- ==========================================================================
  p_out : process(clk)
    variable v : signed(19 downto 0);
  begin
    if rising_edge(clk) then
      s_scaled <= i_out(4) + to_signed(8, 20);
      v := shift_right(s_scaled, 4);
      if v < to_signed(G_CLAMP_LO, 20) then
        s_d_out <= std_logic_vector(to_unsigned(G_CLAMP_LO, 10));
      elsif v > to_signed(G_CLAMP_HI, 20) then
        s_d_out <= std_logic_vector(to_unsigned(G_CLAMP_HI, 10));
      else
        s_d_out <= std_logic_vector(resize(unsigned(v(9 downto 0)), 10));
      end if;
    end if;
  end process;

  -- ==========================================================================
  -- P : /4 sync-discipline pad (+3). The REGEN BRAM-read isolation (Q2a) adds one
  --     real stage (lane core 44 -> 45); this pad brings the lane latency to 48 so
  --     the end-to-end sync depth stays divisible by 4 at 52 (hard rule). Pure
  --     delay on the already-clamped output; no arithmetic.
  -- ==========================================================================
  p_pad : process(clk)
  begin
    if rising_edge(clk) then
      s_dpad(0) <= s_d_out;
      s_dpad(1) <= s_dpad(0);
      s_dpad(2) <= s_dpad(1);
    end if;
  end process;

  d_out <= s_dpad(2);

end architecture rtl;

-- ============================================================================
--  TOP : architecture sequency of program_top
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;

architecture sequency of program_top is

  -- Organ core stays 48 (untouched — the leak is post-fader, top-level). REV-F
  -- retime 52 -> 56 (/4): the Y->C residual leak injects into U/V one output stage
  -- after the fader, so the whole passthrough set (U/V/dry/sync) rides four more
  -- fixed alignment clocks and the faded Y is +4-padded to realign. Sync depth is
  -- FIXED at 56, independent of every knob/toggle/fader (R5/R8).
  constant C_PROCESSING_DELAY_CLKS : integer := 48;  -- lane core incl. Q2a + /4 pad (/4)
  constant C_SYNC_DELAY_CLKS       : integer := 56;  -- + interpolator 4 + leak retime 4 (/4)
  constant C_LAT                   : integer := C_PROCESSING_DELAY_CLKS;

  -- R1 broadcast-safe clamp for the leaked U/V (chroma range [64,960]).
  function clamp_uv(v : signed) return std_logic_vector is
  begin
    if v < to_signed(64, v'length) then
      return std_logic_vector(to_unsigned(64, 10));
    elsif v > to_signed(960, v'length) then
      return std_logic_vector(to_unsigned(960, 10));
    else
      return std_logic_vector(to_unsigned(to_integer(v), 10));
    end if;
  end function;

  -- REGEN address is generated in the Q1 stage's time frame. avid delayed to
  -- ~Q1 gates a per-line active-coeff counter (freeze-during-blank, R7). The
  -- exact phase is Gate-2 tunable (hook #4); any consistent per-line reset
  -- gives a correct 1-line delay.
  constant C_AVID_Q_DELAY : integer := 20;
  constant OFF_Q          : integer := 19;    -- must match the lane

  -- ------- controls (registered shadow) --------------------------------------
  signal s_quant    : unsigned(9 downto 0) := (others => '0');
  signal s_scr_key  : unsigned(3 downto 0) := (others => '0');
  signal s_tilt_dir : std_logic := '0';
  signal s_tilt_str : unsigned(1 downto 0) := (others => '0');
  signal s_regen_sh : unsigned(1 downto 0) := (others => '0');
  signal s_regen_en : std_logic := '0';
  signal s_dith     : std_logic := '0';       -- Tog 7  Dithered enable (0=Hard)
  signal s_scr_mode : std_logic := '0';       -- Tog 8  Scramble mode (0=XOR, 1=Rotate)
  signal s_adapt_en : std_logic := '0';
  signal s_adapt_sh : integer range 0 to 15 := 0;
  signal s_pol      : std_logic := '0';       -- ADAPT polarity (tog 9)
  signal s_hold     : std_logic := '0';       -- Spectral Hold (tog 10)
  signal s_fader    : unsigned(9 downto 0) := (others => '0');

  -- ------- CROSSTALK leak controls (Knob 5 + Tog 11, LIVE) -------------------
  -- Bipolar Knob 5: center 512 = OFF; sign picks the opposite-signed U/V polarity
  -- pairing; |distance| top-3-bits pick the barrel-shift strength (NO multiply).
  -- Tog 11 is the hard-off master enable. All block-stable (decoded once/frame).
  signal s_leak_en  : std_logic := '0';       -- master (Tog 11) AND off-center detent
  signal s_leak_pol : std_logic := '0';       -- 1 = U+= / V-= ; 0 = U-= / V+=
  signal s_leak_sh  : integer range 0 to 7 := 7;  -- residual right-shift (b=1->7 .. b=7->1)

  -- ------- free-running dither LFSR (Tog 7) ----------------------------------
  -- 13-bit maximal Galois/Fibonacci LFSR (taps 13,4,3,1). Survives vsync -> the
  -- one temporal element; stochastic, rate-invariant -> all-formats exempt (spec
  -- A8/§5). Init non-zero so it never locks. Low 5 bits drive the dither noise.
  signal s_lfsr : std_logic_vector(12 downto 0) := (others => '1');

  -- ------- position counter (avid-anchored mod-16) ---------------------------
  signal s_pos    : unsigned(3 downto 0) := (others => '0');
  signal s_avid_q : std_logic := '0';

  -- ------- ADAPT tap (Y-lane Q1 coefficient; drives the shared deadzone) ------
  signal s_tap_y : signed(15 downto 0);

  -- ------- ADAPT (shared, Y block-energy driven) -----------------------------
  signal s_en_acc  : unsigned(17 downto 0) := (others => '0');
  signal s_en_blk  : unsigned(17 downto 0) := (others => '0');
  signal s_adapt_d : signed(15 downto 0) := (others => '0');

  -- ------- QUANTIZE deadzone thresholds (block-stable, hoisted from lane Q3) --
  -- q_th(k) = (QUANTIZE << k) + ADAPT delta, clamped >=0, for each C_QSHIFT band
  -- k=0..3. Depend only on QUANTIZE (per-frame) + ADAPT (per-block) -> stable
  -- across a 16-coeff block, so each lane's per-coefficient Q3 reduces to a 4:1
  -- select + compare. Computed once (shared by all 3 lanes) — Gate-1 timing fix
  -- (removes the 17b add + barrel-shift + >=0 clamp from the critical path).
  type t_qth is array (0 to 3) of signed(17 downto 0);
  signal s_q_th   : t_qth := (others => (others => '0'));  -- base band thresholds
  -- Tog-7 dithered thresholds: s_q_th(k) minus a band-scaled LFSR sample, computed
  -- at the TOP (block-stable, slack) and registered separately so the per-band
  -- subtract never touches the per-coefficient Q3 compare path. These feed the lane.
  signal s_q_th_d : t_qth := (others => (others => '0'));

  -- ------- REGEN line buffer (Y coefficient feedback only) -------------------
  -- Y : 2048 x 12 = 6 EBR.  (The packed U+V coeff buffer is removed with the
  -- U/V organ lanes; chroma no longer resonates.)
  type t_ram_y is array (0 to 2047) of std_logic_vector(11 downto 0);
  signal ram_y : t_ram_y := (others => (others => '0'));

  signal s_qavid    : std_logic := '0';                 -- avid delayed to Q1
  signal s_qa_sr    : std_logic_vector(0 to C_AVID_Q_DELAY - 1) := (others => '0');
  signal s_qidx     : unsigned(10 downto 0) := (others => '0');
  signal s_wr_addr0 : unsigned(10 downto 0) := (others => '0');
  signal s_wr_addr1 : unsigned(10 downto 0) := (others => '0');
  signal s_wr_addr  : unsigned(10 downto 0) := (others => '0');
  signal s_wr_en0   : std_logic := '0';
  signal s_wr_en1   : std_logic := '0';
  signal s_wr_en    : std_logic := '0';
  signal s_prev_y : signed(11 downto 0) := (others => '0');
  signal s_cur_y  : signed(11 downto 0);

  -- ------- Y lane output -----------------------------------------------------
  signal s_org_y : std_logic_vector(9 downto 0);

  -- ------- sync + dry-Y + U/V passthrough circular delay (64 x 34 = 3 EBR) ----
  -- Carries dryY (Y fader tap) + U/V (passthrough) + sync, all delayed to 48.
  type t_dly is array (0 to 63) of std_logic_vector(33 downto 0);
  signal s_dly_ram : t_dly := (others => (others => '0'));
  signal s_wa      : unsigned(5 downto 0) := (others => '0');
  signal s_rd_q    : std_logic_vector(33 downto 0) := (others => '0');
  signal s_rd_q2   : std_logic_vector(33 downto 0) := (others => '0');

  signal s_dry_y : std_logic_vector(9 downto 0);

  -- ------- passthrough +4 alignment tail (48 -> 52) --------------------------
  -- Carries the FULL 34-bit delay word (dryY10 + passU10 + passV10 + avid/hs/vs/
  -- fld) from the BRAM exit (48) to the fader output depth (52). dryY rides along
  -- (REV-F: the leak's Y_dry reference) so the residual can be formed at 52 with
  -- no extra BRAM port. U/V/sync then enter the 4-clock leak stage below (52->56).
  type t_tail is array (0 to 3) of std_logic_vector(33 downto 0);
  signal s_tail : t_tail := (others => (others => '1'));

  -- ------- Y fader output ----------------------------------------------------
  signal s_mix_y : unsigned(9 downto 0);

  -- ------- REV-F Y->C residual leak pipeline (52 -> 56) ----------------------
  -- L1: residual = Y_out - Y_dry (gated by master enable); carry U/V/sync.
  -- L2: k barrel-shift (residual >> s_leak_sh, NO multiply); carry U/V/sync.
  -- L3: opposite-signed inject (U +=/-= inj, V mirror) per Knob-5 polarity.
  -- L4: R1 clamp [64,960] -> broadcast-safe U/V; sync lands at 56.
  signal s_lk_res  : signed(12 downto 0) := (others => '0');       -- @53
  signal s_lk_u1   : unsigned(9 downto 0) := (others => '0');
  signal s_lk_v1   : unsigned(9 downto 0) := (others => '0');
  signal s_lk_sy1  : std_logic_vector(3 downto 0) := (others => '1');
  signal s_lk_inj  : signed(12 downto 0) := (others => '0');       -- @54
  signal s_lk_u2   : unsigned(9 downto 0) := (others => '0');
  signal s_lk_v2   : unsigned(9 downto 0) := (others => '0');
  signal s_lk_sy2  : std_logic_vector(3 downto 0) := (others => '1');
  signal s_lk_usum : signed(12 downto 0) := (others => '0');       -- @55
  signal s_lk_vsum : signed(12 downto 0) := (others => '0');
  signal s_lk_sy3  : std_logic_vector(3 downto 0) := (others => '1');
  signal s_lk_uo   : std_logic_vector(9 downto 0) := (others => '0'); -- @56
  signal s_lk_vo   : std_logic_vector(9 downto 0) := (others => '0');
  signal s_lk_sy4  : std_logic_vector(3 downto 0) := (others => '1');

  -- ------- faded-Y +4 pure-delay pipe (52 -> 56) -----------------------------
  -- Realigns the faded Y with the leaked U/V (Y itself is NOT leaked). 4 x 10 FF.
  type t_y56 is array (0 to 3) of unsigned(9 downto 0);
  signal s_y56 : t_y56 := (others => (others => '0'));

begin

  -- ==========================================================================
  -- Control decode (no fabric on knob response; musical scaling lives in TOML).
  -- ==========================================================================
  p_ctrl : process(clk)
    variable v_td  : signed(10 downto 0);
    variable v_tm  : unsigned(9 downto 0);
    variable v_reg : unsigned(9 downto 0);
    variable v_adp : unsigned(9 downto 0);
    variable v_xt  : signed(10 downto 0);    -- Knob 5 distance from center (bipolar)
    variable v_xd  : unsigned(10 downto 0);  -- |distance|, 0..512
    variable v_xb  : unsigned(2 downto 0);   -- top-3-bits leak-strength band 0..7
  begin
    if rising_edge(clk) then
      s_quant <= unsigned(registers_in(0));

      -- TILT (knob 2): bipolar around 512; magnitude -> strength 0..3, attenuate only
      v_td := signed('0' & registers_in(1)) - to_signed(512, 11);
      if v_td(10) = '1' then
        s_tilt_dir <= '0';                      -- < 512 : bass tilt (cut highs)
        v_tm := unsigned(resize(-v_td, 10));
      else
        s_tilt_dir <= '1';                      -- > 512 : treble tilt (cut lows)
        v_tm := unsigned(resize(v_td, 10));
      end if;
      if    v_tm >= 384 then s_tilt_str <= "11";
      elsif v_tm >= 256 then s_tilt_str <= "10";
      elsif v_tm >= 128 then s_tilt_str <= "01";
      else                   s_tilt_str <= "00";
      end if;

      -- SCRAMBLE (knob 3): top 4 bits = XOR key (0 -> identity)
      s_scr_key <= unsigned(registers_in(2)(9 downto 6));

      -- REGEN (knob 4): enable + feedback shift (gain <= 1/2; bigger knob -> less shift)
      v_reg := unsigned(registers_in(3));
      if v_reg > 16 then s_regen_en <= '1'; else s_regen_en <= '0'; end if;
      s_regen_sh <= not v_reg(9 downto 8);      -- strong->00 (>>1) .. weak->11 (>>4)

      -- CROSSTALK (knob 5): LIVE — bipolar control for the output-stage Y->C
      -- effect-residual leak. center 512 = OFF (b=0 center detent). Sign selects
      -- the opposite-signed U/V polarity pairing; the top 3 bits of |distance|
      -- select the leak strength as a barrel-SHIFT amount (H4: NO runtime multiply).
      v_xt := signed('0' & registers_in(4)) - to_signed(512, 11);
      if v_xt(10) = '1' then
        s_leak_pol <= '0';                       -- knob < 512 : U -= , V +=
        v_xd := unsigned(resize(-v_xt, 11));
      else
        s_leak_pol <= '1';                       -- knob >= 512 : U += , V -=
        v_xd := unsigned(resize(v_xt, 11));
      end if;
      if v_xd > 511 then v_xb := "111";          -- saturate distance to 9 bits (top 3)
      else                v_xb := v_xd(8 downto 6); end if;
      if v_xb = "000" then                        -- center detent -> leak OFF
        s_leak_en <= '0';
        s_leak_sh <= 7;
      else
        s_leak_en <= registers_in(6)(4);          -- Tog 11 master (Chroma too = enable)
        s_leak_sh <= 8 - to_integer(v_xb);        -- b=1 -> >>7 (subtle) .. b=7 -> >>1 (vivid)
      end if;

      -- ADAPT (knob 6): energy -> deadzone; bigger knob -> smaller shift -> bigger delta
      v_adp := unsigned(registers_in(5));
      if v_adp > 16 then s_adapt_en <= '1'; else s_adapt_en <= '0'; end if;
      case v_adp(9 downto 8) is                 -- shift LUT (no multiply): 12,9,6,3
        when "00"   => s_adapt_sh <= 12;
        when "01"   => s_adapt_sh <= 9;
        when "10"   => s_adapt_sh <= 6;
        when others => s_adapt_sh <= 3;
      end case;

      s_dith     <= registers_in(6)(0);         -- Tog 7 : Hard / Dithered
      s_scr_mode <= registers_in(6)(1);         -- Tog 8 : XOR / Rotate
      s_pol      <= registers_in(6)(2);
      s_hold     <= registers_in(6)(3);
      s_fader    <= unsigned(registers_in(7));
    end if;
  end process;

  -- ==========================================================================
  -- Free-running dither LFSR (Tog 7 source). 13-bit maximal Fibonacci LFSR
  -- (taps 13,4,3,1). NOT reset at vsync -> stochastic, rate-invariant noise
  -- (all-formats exempt, spec A8). Runs every clock; consumed only when Dithered.
  -- ==========================================================================
  p_lfsr : process(clk)
    variable v_fb : std_logic;
  begin
    if rising_edge(clk) then
      v_fb   := s_lfsr(12) xor s_lfsr(3) xor s_lfsr(2) xor s_lfsr(0);
      s_lfsr <= s_lfsr(11 downto 0) & v_fb;
    end if;
  end process;

  -- ==========================================================================
  -- Position counter : free-runs, re-anchors at avid rising edge (mod 16).
  -- ==========================================================================
  p_pos : process(clk)
  begin
    if rising_edge(clk) then
      s_avid_q <= data_in.avid;
      if data_in.avid = '1' and s_avid_q = '0' then
        s_pos <= to_unsigned(1, 4);             -- this clock registers pixel 0
      else
        s_pos <= s_pos + 1;
      end if;
    end if;
  end process;

  -- ==========================================================================
  -- ADAPT (shared): accumulate |Y tap| across a 16-coeff block; latch at block
  -- start; scale by knob 6 + polarity -> signed threshold delta for all lanes.
  -- Intra-frame, Y-driven (spec 3.2 lever 1). 0 when disabled (identity-safe).
  -- ==========================================================================
  p_adapt : process(clk)
    variable v_tapseq : unsigned(3 downto 0);
    variable v_mag    : unsigned(17 downto 0);
    variable v_delta  : signed(15 downto 0);
  begin
    if rising_edge(clk) then
      v_tapseq := s_pos - to_unsigned(OFF_Q mod 16, 4);   -- sequency at the Y tap
      v_mag    := resize(unsigned(abs(s_tap_y)), 18);
      if v_tapseq = 0 then
        s_en_blk <= s_en_acc;                 -- latch previous block's energy
        s_en_acc <= v_mag;                    -- start new block
      else
        s_en_acc <= s_en_acc + v_mag;
      end if;

      if s_adapt_en = '0' then
        s_adapt_d <= (others => '0');
      else
        v_delta := signed('0' & std_logic_vector(
                     resize(shift_right(s_en_blk, s_adapt_sh), 15)));
        if s_pol = '1' then s_adapt_d <= -v_delta; else s_adapt_d <= v_delta; end if;
      end if;
    end if;
  end process;

  -- ==========================================================================
  -- QUANTIZE threshold precompute (shared, block-stable). For each C_QSHIFT band
  -- k, q_th(k) = (QUANTIZE << k) + ADAPT delta, clamped >= 0. Registered here
  -- once instead of rebuilt in every lane's per-coefficient Q3 path (Gate-1
  -- timing fix). QUANTIZE<<3 max = 1023*8 = 8184 (fits 14b); the sum fits 18b.
  -- ==========================================================================
  p_qth : process(clk)
    variable v_raw : signed(17 downto 0);
  begin
    if rising_edge(clk) then
      for k in 0 to 3 loop
        v_raw := signed(resize(shift_left(resize(s_quant, 14), k), 18))
                 + resize(s_adapt_d, 18);
        if v_raw < 0 then
          s_q_th(k) <= (others => '0');           -- threshold >= 0
        else
          s_q_th(k) <= v_raw;
        end if;
      end loop;
    end if;
  end process;

  -- ==========================================================================
  -- QUANTIZE dither (Tog 7, TOP-level). Per band k: q_th_d(k) = q_th(k) - (LFSR
  -- sample << k) — a threshold that jitters both ways at the deadzone edge, scaled
  -- with the band (shift by the CONSTANT k, no barrel shifter) so the sparkle
  -- lives at the deadzone edge. Reads the ALREADY-REGISTERED s_q_th (one clean
  -- subtract, block-stable -> slack; NOT on the Q3 compare path). Gated OFF unless
  -- Dithered AND a real deadzone exists (thr > 0) -> QUANTIZE=0 stays keep-all
  -- identity and Hard mode passes the plain threshold (bit-exact REV-E). The +1
  -- clock is on the block-stable threshold, not the coefficient data -> latency 56
  -- unchanged, identity exact (thr=0 at QUANTIZE=0 regardless of phase).
  -- ==========================================================================
  p_qth_dith : process(clk)
    variable v_noise : signed(17 downto 0);
  begin
    if rising_edge(clk) then
      for k in 0 to 3 loop
        if s_dith = '1' and s_q_th(k) > 0 then
          v_noise := shift_left(resize(signed(s_lfsr(4 downto 0)), 18), k);
          s_q_th_d(k) <= s_q_th(k) - v_noise;
        else
          s_q_th_d(k) <= s_q_th(k);
        end if;
      end loop;
    end if;
  end process;

  -- ==========================================================================
  -- REGEN address pipeline + line-buffer read/write (freeze-during-blank, R7).
  --   read  @ s_qidx (Q1 time)  -> prev-line coeff at Q2
  --   write @ s_qidx delayed 2  -> resonant coeff produced at Q2
  -- Single Y coeff buffer. Writes gated by qavid AND not-hold (Spectral Hold
  -- freezes the buffer).
  -- ==========================================================================
  p_regen_ctl : process(clk)
  begin
    if rising_edge(clk) then
      -- avid delayed to ~Q1 via a 1-bit shift chain (cheap, sync machinery)
      s_qa_sr <= data_in.avid & s_qa_sr(0 to C_AVID_Q_DELAY - 2);
      s_qavid <= s_qa_sr(C_AVID_Q_DELAY - 1);

      if s_qavid = '0' then                    -- outside active coeff stream
        s_qidx <= (others => '0');
      else
        s_qidx <= s_qidx + 1;
      end if;

      -- write address/enable pipeline: align to when regen_cur is valid. The Q2a
      -- BRAM-read isolation pushed regen_cur one clock later (Q2 -> Q2b), so this
      -- is now a 3-deep delay of s_qidx (was 2). Read stays ahead of write
      -- (gap 2 -> 3 clocks, both << the ~2048-clock line) — no read/write hazard.
      s_wr_addr0 <= s_qidx;   s_wr_addr1 <= s_wr_addr0;   s_wr_addr <= s_wr_addr1;
      s_wr_en0   <= s_qavid and (not s_hold);
      s_wr_en1   <= s_wr_en0;
      s_wr_en    <= s_wr_en1;
    end if;
  end process;

  -- Separate read / write processes per channel = the SDK-proven dual-port EBR
  -- inference pattern (port A = registered read, port B = gated write).
  p_regen_rd : process(clk)
  begin
    if rising_edge(clk) then
      -- registered read (prev line) -> arrives at the Y lane's Q2
      s_prev_y <= signed(ram_y(to_integer(s_qidx)));
    end if;
  end process;

  p_regen_wr : process(clk)
  begin
    if rising_edge(clk) then
      -- write (this line's resonant Y coeff), frozen when Spectral Hold engaged
      if s_wr_en = '1' then
        ram_y(to_integer(s_wr_addr)) <= std_logic_vector(s_cur_y);
      end if;
    end if;
  end process;

  -- ==========================================================================
  -- Single Y organ lane (full-res REGEN, full feature set). U/V are passthrough
  -- (see the circular delay below) — no U/V lane instances.
  -- ==========================================================================
  u_y : entity work.wht_organ_lane
    generic map (G_REGEN_W => 12, G_CLAMP_LO => 64, G_CLAMP_HI => 940)
    port map (clk => clk, pos => s_pos,
              q_th0 => s_q_th_d(0), q_th1 => s_q_th_d(1),
              q_th2 => s_q_th_d(2), q_th3 => s_q_th_d(3),
              scr_key => s_scr_key, scr_mode => s_scr_mode,
              tilt_dir => s_tilt_dir, tilt_str => s_tilt_str,
              regen_sh => s_regen_sh, regen_en => s_regen_en,
              hold => s_hold,
              regen_prev => s_prev_y, regen_cur => s_cur_y,
              coef_tap => s_tap_y,
              d_in => data_in.y, d_out => s_org_y);

  -- ==========================================================================
  -- Sync + dry-Y + U/V passthrough circular delay (48). Pure delay -> runs
  -- unconditionally (blanking-safe). dryY feeds the Y fader; U/V + sync feed the
  -- +4 tail. Data rides BRAM, not FF (saves ~440 FF/ch, A7).
  -- ==========================================================================
  p_dly : process(clk)
  begin
    if rising_edge(clk) then
      s_wa <= s_wa + 1;
      s_dly_ram(to_integer(s_wa)) <=
        data_in.y & data_in.u & data_in.v &
        data_in.avid & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
      s_rd_q  <= s_dly_ram(to_integer(s_wa - to_unsigned(C_LAT - 2, 6)));
      s_rd_q2 <= s_rd_q;
    end if;
  end process;

  s_dry_y <= s_rd_q2(33 downto 24);           -- dry-Y at 48 (fader input)

  -- ==========================================================================
  -- Dry/wet fader : ONE interpolator_u, on Y only (principle #4). +4 clocks.
  -- Output s_mix_y (= Y_out) at 52. The single * in the design lives here (the
  -- organ + the REV-F leak are multiply-free: k is a barrel shift).
  -- ==========================================================================
  mix_y : entity work.interpolator_u
    generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                 G_OUTPUT_MIN => 64, G_OUTPUT_MAX => 940)
    port map (clk => clk, enable => '1',
              a => unsigned(s_dry_y), b => unsigned(s_org_y),
              t => s_fader, result => s_mix_y, valid => open);

  -- ==========================================================================
  -- Passthrough +4 alignment tail (48 -> 52). Advances the full delay word
  -- (dryY + U/V + sync) to the fader-output depth so the leak can form the
  -- residual (Y_out - Y_dry) at 52 and hand U/V/sync to the leak stage. FIXED
  -- depth, independent of every knob/toggle/fader (R5/R8).
  -- ==========================================================================
  p_tail : process(clk)
  begin
    if rising_edge(clk) then
      s_tail(0) <= s_rd_q2;                    -- full 34-bit word at 48 -> 49
      s_tail(1) <= s_tail(0);                  -- 50
      s_tail(2) <= s_tail(1);                  -- 51
      s_tail(3) <= s_tail(2);                  -- 52 (dryY/U/V/sync aligned to Y_out)
    end if;
  end process;

  -- ==========================================================================
  -- REV-F Y->C effect-residual leak (52 -> 56). Injects k*(Y_out - Y_dry) into
  -- U/V, opposite-signed (NTSC cross-color shimmer, sync_bleed S6 precedent).
  -- Self-nulls: residual is identically 0 at fader-dry or effect-neutral, so
  -- chroma stays bit-exact regardless of Knob 5 (load-neutral contract). k is a
  -- barrel shift (NO multiply). R3-safe: the inject saturates at the R1 clamp.
  --   L1 @53 residual (gated by master enable)   L2 @54 k barrel-shift
  --   L3 @55 opposite-signed inject               L4 @56 R1 clamp [64,960]
  -- ==========================================================================
  p_leak : process(clk)
    variable v_res  : signed(12 downto 0);
    variable v_usum : signed(12 downto 0);
    variable v_vsum : signed(12 downto 0);
  begin
    if rising_edge(clk) then
      -- L1: effect residual = Y_out(52) - Y_dry(52), gated OFF by the master
      -- enable (Tog 11 "Luma only" OR Knob 5 center) -> residual forced 0.
      if s_leak_en = '1' then
        v_res := signed(resize(s_mix_y, 13))
               - signed(resize(unsigned(s_tail(3)(33 downto 24)), 13));
      else
        v_res := (others => '0');
      end if;
      s_lk_res <= v_res;
      s_lk_u1  <= unsigned(s_tail(3)(23 downto 14));
      s_lk_v1  <= unsigned(s_tail(3)(13 downto 4));
      s_lk_sy1 <= s_tail(3)(3 downto 0);

      -- L2: k barrel-shift (top-3-knob-bits -> right-shift; NO runtime multiply)
      s_lk_inj <= shift_right(s_lk_res, s_leak_sh);
      s_lk_u2  <= s_lk_u1;
      s_lk_v2  <= s_lk_v1;
      s_lk_sy2 <= s_lk_sy1;

      -- L3: opposite-signed inject (U and V take opposite polarity -> rainbow)
      if s_leak_pol = '1' then
        v_usum := signed(resize(s_lk_u2, 13)) + s_lk_inj;   -- U += , V -=
        v_vsum := signed(resize(s_lk_v2, 13)) - s_lk_inj;
      else
        v_usum := signed(resize(s_lk_u2, 13)) - s_lk_inj;   -- U -= , V +=
        v_vsum := signed(resize(s_lk_v2, 13)) + s_lk_inj;
      end if;
      s_lk_usum <= v_usum;
      s_lk_vsum <= v_vsum;
      s_lk_sy3  <= s_lk_sy2;

      -- L4: R1 broadcast-safe clamp [64,960] (leak inject cannot wrap, R3)
      s_lk_uo  <= clamp_uv(s_lk_usum);
      s_lk_vo  <= clamp_uv(s_lk_vsum);
      s_lk_sy4 <= s_lk_sy3;
    end if;
  end process;

  -- ==========================================================================
  -- Faded-Y +4 pure-delay pipe (52 -> 56): realign the faded Y with the leaked
  -- U/V. Y is NOT leaked (the leak is Y->C only); this is pure delay.
  -- ==========================================================================
  p_y56 : process(clk)
  begin
    if rising_edge(clk) then
      s_y56(0) <= s_mix_y;                     -- 53
      s_y56(1) <= s_y56(0);                    -- 54
      s_y56(2) <= s_y56(1);                    -- 55
      s_y56(3) <= s_y56(2);                    -- 56
    end if;
  end process;

  -- ==========================================================================
  -- Output assignment (every data_out port driven exactly once, R2). All ports
  -- land at 56: Y = faded organ +4 pad; U/V = leaked passthrough; sync = tail +
  -- leak carry. Sync depth FIXED at 56, param-independent (R5/R8).
  -- ==========================================================================
  data_out.y       <= std_logic_vector(s_y56(3));
  data_out.u       <= s_lk_uo;
  data_out.v       <= s_lk_vo;
  data_out.avid    <= s_lk_sy4(3);
  data_out.hsync_n <= s_lk_sy4(2);
  data_out.vsync_n <= s_lk_sy4(1);
  data_out.field_n <= s_lk_sy4(0);

end architecture sequency;
