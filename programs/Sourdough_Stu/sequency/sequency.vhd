-- Copyright (C) 2026 AB (Sourdough_Stu)
-- SPDX-License-Identifier: GPL-3.0-only
-- File: sequency.vhd - SEQUENCY: the Walsh-Hadamard organ (Program 5)
--
-- This file is free software under the GNU General Public License v3.0.
-- https://github.com/lzxindustries/videomancer-community-programs
--
-- Program Name:
--   Sequency  (Y-ONLY, R2SDF streaming WHT — committed build)
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
--   Y-ONLY committed topology: a single wht_organ_lane on Y with the FULL
--   feature set (QUANTIZE/TILT/SCRAMBLE/REGEN/ADAPT + dry/wet fader). U and V
--   are PASSTHROUGH, delay-matched to the Y organ+fader latency (48) via the
--   shared BRAM circular delay + the +4 interpolator-alignment tail (the
--   sequency_sdf reference-prototype pattern). The U/V organ lanes and the
--   packed U/V REGEN coefficient buffer are REMOVED. Shared control-derive
--   (TILT strength/dir, SCRAMBLE key, ADAPT threshold, and the precomputed
--   QUANTIZE deadzone thresholds q_th0..3) lives in the top and drives the one
--   Y lane. Knob 5 (CROSSTALK) and Toggle 11 (Chroma) are RESERVED/INERT (read,
--   not acted on): chroma is passthrough, there is no cross-lane datapath. All
--   12 controls stay declared.
--
--   Scope note: the full-4:4:4-chroma build (3x wht_organ_lane) was congestion-
--   bound (~88% fill, Fmax stuck at 56.5->62.9->66.1 MHz across 3 timing
--   iterations); reverted to Y-only — the comfortable, proven scope — by user
--   decision (cut losses on full-chroma HD timing).
--
-- Pipeline stages + total latency:
--   Organ core (C_PROCESSING_DELAY_CLKS = 44, /4):
--     R  input register .................................. 1  (cum 1)
--     F1 forward SDF D=8  (W=12) ......................... 9  (cum 10)
--     F2 forward SDF D=4  (W=13) ......................... 5  (cum 15)
--     F3 forward SDF D=2  (W=14) ......................... 3  (cum 18)
--     F4 forward SDF D=1  (W=15) -> coeff stream ......... 2  (cum 20)
--     Q1 capture + TILT(atten) + SCRAMBLE-index + ADAPT tap  1  (cum 21)
--     Q2 REGEN add (TILT+REGEN, 2-input) + R3 sat + store .. 1  (cum 22)
--     Q3 QUANTIZE deadzone: band-select precomputed thr
--        (q_th0..3) + magnitude compare, DC exempt ....... 1  (cum 23)
--     I1 inverse SDF D=1  (W=17, widened +1) ............. 2  (cum 25)
--     I2 inverse SDF D=2  (W=18) ......................... 3  (cum 28)
--     I3 inverse SDF D=4  (W=19) ......................... 5  (cum 33)
--     I4 inverse SDF D=8  (W=20) -> reconstructed x16 .... 9  (cum 42)
--     S  +8 round, >>4 normalize, R1 broadcast clamp ..... 2  (cum 44)
--   Dry/wet fader (interpolator_u, Y only) ............... 4  -> total 48
--   => C_PROCESSING_DELAY_CLKS = 44 (organ), C_SYNC_DELAY_CLKS = 48 (= 44+4,
--      the interpolator tail; howler discipline). Sync + dry-Y + U/V passthrough
--      ride the BRAM circular delay (44); dry-Y feeds the Y fader (-> 48) and
--      U/V + sync ride a shared +4 tail so every data_out port is aligned at 48,
--      FIXED and independent of every knob/toggle/fader (R5/R8).
--      NOTE: spec headline "latency 44" = the organ core; the mandatory 4-clock
--      interpolator_u fader makes the sync-aligned end-to-end 48 (both /4). This
--      is the spec's own 44-vs-48 re-point (spec S1); flagged to orchestrator.
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
--   sequency_sdf had no fader so its delay was 24b/2 EBR. 9 << 20 budget.
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
--   frees the congestion, BUT the REGEN feedback store is INTRINSIC, not
--   congestion: MEASURED Gate-1 smoke (hd_hdmi, seed 1) = 3,698 LC (48%) /
--   9 EBR / Fmax 70.97 MHz — LC & EBR comfortable, but Fmax STILL < the 76 gate
--   (up from the 3-lane 66.1, not cleared). Critical path = the Y REGEN Q2
--   store: ram_y BRAM read (2.15 ns clk-to-q) -> v_sum 18b add + R3 double-
--   saturate (s_c2 16b & s_cur_y 12b) -> register; the BRAM read feeding the
--   adder in a single clock is the wall (pre-route 74.75, also < 76 -> not seed
--   noise). Deeper fix = pipeline the REGEN add (BRAM-read -> reg -> add ->
--   store), which re-points latency 48 -> 52 (touches the REGEN loop + sync/dry
--   depth); OR a latency-neutral REGEN feedback-term retime (register the
--   shifted regen_prev off the adder's critical input, needs sim re-verify);
--   OR reduced REGEN precision. Each is an architectural tradeoff and the task
--   fixed latency at 48 -> NOT applied here; ESCALATED to the orchestrator. The
--   shared-threshold hoist (s_q_th -> q_th0..3) is KEPT. Latency 48 (/4).
--
-- Register map (SPI: pots -> registers_in(0..5), toggles -> (6) bits 0..4,
--               fader -> (7), timing ID -> (8)(3 downto 0) [unused: all-formats]):
--   Knob 1  registers_in(0)     QUANTIZE   sequency-weighted deadzone
--   Knob 2  registers_in(1)     TILT       bass<->treble, attenuate-only (512=flat)
--   Knob 3  registers_in(2)     SCRAMBLE   basis permutation depth (index XOR)
--   Knob 4  registers_in(3)     REGEN      vertical coefficient feedback
--   Knob 5  registers_in(4)     CROSSTALK  RESERVED/INERT (read, not acted on)
--   Knob 6  registers_in(5)     ADAPT      energy -> quantizer self-mod (Y-driven)
--   Tog 7   registers_in(6)(0)  Quantizer  Hard/Dithered (Dithered inert v1)
--   Tog 8   registers_in(6)(1)  ScrMode    XOR/Rotate    (Rotate inert v1)
--   Tog 9   registers_in(6)(2)  ADAPT pol  Normal/Inverted
--   Tog 10  registers_in(6)(3)  SpecHold   Live/Hold
--   Tog 11  registers_in(6)(4)  Chroma     RESERVED/INERT (U/V passthrough; read, ignored)
--   Fader   registers_in(7)     Dry/Wet    interpolator_u crossfade
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
    -- ADAPT, clamped >=0). Block-stable -> hoisted out of the per-coefficient Q3
    -- critical path at the top (Gate-1 timing fix); the lane just band-selects.
    q_th0       : in  signed(17 downto 0);            -- band k=0 threshold
    q_th1       : in  signed(17 downto 0);            -- band k=1 threshold
    q_th2       : in  signed(17 downto 0);            -- band k=2 threshold
    q_th3       : in  signed(17 downto 0);            -- band k=3 threshold
    scr_key     : in  unsigned(3 downto 0);           -- SCRAMBLE index XOR key
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
  signal s_c2     : signed(15 downto 0) := (others => '0');  -- Q2: post REGEN (R3 sat)
  signal s_c3     : signed(15 downto 0) := (others => '0');  -- Q3: post QUANTIZE deadzone
  signal s_seq1   : unsigned(3 downto 0) := (others => '0'); -- true sequency, pipelined
  signal s_seq2   : unsigned(3 downto 0) := (others => '0');

  signal s_scaled : signed(19 downto 0) := (others => '0');
  signal s_d_out  : std_logic_vector(9 downto 0) := (others => '0');

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
  constant OFF_I1 : integer := 22; constant OFF_I2 : integer := 24;
  constant OFF_I3 : integer := 27; constant OFF_I4 : integer := 32;

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
      v_seff := v_seq xor scr_key;                     -- SCRAMBLE: permute param index
      v_tsh  := tilt_amt(v_seff, tilt_dir, tilt_str);
      s_c_tilt <= shift_right(resize(f_out(4), 16), v_tsh);  -- >> only (no overflow)
      s_seq1   <= v_seq;
    end if;
  end process;

  coef_tap <= s_c_tilt;   -- Q1 register out; ADAPT tap (top consumes Y lane only)

  -- ==========================================================================
  -- Q2 : REGEN vertical feedback add (2-input: TILT coeff + REGEN prev-line),
  --      R3 saturating clamp to 16b, store the resonant AC coefficient (reduced
  --      precision), and the Spectral-Hold source select. DC (true seq 0) is
  --      excluded from feedback. (CROSSTALK inject removed — Gate-1 rework.)
  -- ==========================================================================
  p_q2 : process(clk)
    variable v_regen : signed(17 downto 0);
    variable v_sum   : signed(17 downto 0);
  begin
    if rising_edge(clk) then
      -- REGEN feedback term: prev-line stored coeff >> (regen_sh+1)  (gain <= 1/2)
      if regen_en = '1' and s_seq1 /= 0 then
        v_regen := shift_right(resize(regen_prev, 18),
                               to_integer(regen_sh) + 1);
      else
        v_regen := (others => '0');
      end if;
      v_sum := resize(s_c_tilt, 18) + v_regen;   -- 2-input add (no CROSSTALK term)

      if hold = '1' then
        -- Spectral Hold: replay the frozen (stored) coefficient into the inverse
        s_c2 <= resize(regen_prev, 16);
      else
        s_c2 <= sat_s(v_sum, 16);                 -- R3 clamp, coeff path 16b
      end if;

      -- store resonant coeff for next line (AC only; saturated to store width)
      if s_seq1 = 0 then
        regen_cur <= (others => '0');
      else
        regen_cur <= sat_s(v_sum, G_REGEN_W);
      end if;

      s_seq2 <= s_seq1;
    end if;
  end process;

  -- ==========================================================================
  -- Q3 : QUANTIZE hard deadzone. threshold = (qeff << C_QSHIFT[seq_eff]) + ADAPT.
  --      DC (true seq 0) exempt -> QUANTIZE=0 keeps everything (identity core).
  --
  --      TIMING FIX (Gate 1): the threshold is block-stable (qeff is per-frame,
  --      ADAPT is per-block), so its shift + 17b add + >=0 clamp are precomputed
  --      once per block at the top (q_th0..3, one per C_QSHIFT band). This clock
  --      now does ONLY: band-select (xor + LUT + 4:1 mux) + magnitude compare +
  --      select — no barrel-shift, no wide add, no clamp on the per-coeff path.
  --      Bit-identical to the old inline threshold (same value, computed 1 clk
  --      earlier); ADAPT's block phase shifts by 1 clk (heuristic, within tol),
  --      identity is exact (thr=0 at QUANTIZE=0 regardless of phase).
  -- ==========================================================================
  p_q3 : process(clk)
    variable v_seff : unsigned(3 downto 0);
    variable v_th   : signed(17 downto 0);
    variable v_mag  : unsigned(15 downto 0);
    variable v_keep : std_logic;
  begin
    if rising_edge(clk) then
      v_seff := s_seq2 xor scr_key;
      case C_QSHIFT(to_integer(v_seff)) is           -- select precomputed band thr
        when 0      => v_th := q_th0;
        when 1      => v_th := q_th1;
        when 2      => v_th := q_th2;
        when others => v_th := q_th3;
      end case;
      v_mag := unsigned(abs(s_c2));

      if s_seq2 = 0 then
        v_keep := '1';                                    -- DC exempt
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

  d_out <= s_d_out;

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

  constant C_PROCESSING_DELAY_CLKS : integer := 44;  -- organ core (/4)
  constant C_SYNC_DELAY_CLKS       : integer := 48;  -- + interpolator (/4)
  constant C_LAT                   : integer := C_PROCESSING_DELAY_CLKS;

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
  signal s_xt_reserved : unsigned(9 downto 0) := (others => '0');  -- Knob 5 CROSSTALK: reserved/inert (read, unused)
  signal s_adapt_en : std_logic := '0';
  signal s_adapt_sh : integer range 0 to 15 := 0;
  signal s_pol      : std_logic := '0';       -- ADAPT polarity (tog 9)
  signal s_hold     : std_logic := '0';       -- Spectral Hold (tog 10)
  signal s_chroma   : std_logic := '1';       -- Tog 11 Chroma: reserved/inert (read, unused)
  signal s_fader    : unsigned(9 downto 0) := (others => '0');

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
  signal s_q_th : t_qth := (others => (others => '0'));

  -- ------- REGEN line buffer (Y coefficient feedback only) -------------------
  -- Y : 2048 x 12 = 6 EBR.  (The packed U+V coeff buffer is removed with the
  -- U/V organ lanes; chroma no longer resonates.)
  type t_ram_y is array (0 to 2047) of std_logic_vector(11 downto 0);
  signal ram_y : t_ram_y := (others => (others => '0'));

  signal s_qavid    : std_logic := '0';                 -- avid delayed to Q1
  signal s_qa_sr    : std_logic_vector(0 to C_AVID_Q_DELAY - 1) := (others => '0');
  signal s_qidx     : unsigned(10 downto 0) := (others => '0');
  signal s_wr_addr1 : unsigned(10 downto 0) := (others => '0');
  signal s_wr_addr  : unsigned(10 downto 0) := (others => '0');
  signal s_wr_en1   : std_logic := '0';
  signal s_wr_en    : std_logic := '0';
  signal s_prev_y : signed(11 downto 0) := (others => '0');
  signal s_cur_y  : signed(11 downto 0);

  -- ------- Y lane output -----------------------------------------------------
  signal s_org_y : std_logic_vector(9 downto 0);

  -- ------- sync + dry-Y + U/V passthrough circular delay (64 x 34 = 3 EBR) ----
  -- Carries dryY (Y fader tap) + U/V (passthrough) + sync, all delayed to 44.
  type t_dly is array (0 to 63) of std_logic_vector(33 downto 0);
  signal s_dly_ram : t_dly := (others => (others => '0'));
  signal s_wa      : unsigned(5 downto 0) := (others => '0');
  signal s_rd_q    : std_logic_vector(33 downto 0) := (others => '0');
  signal s_rd_q2   : std_logic_vector(33 downto 0) := (others => '0');

  signal s_dry_y : std_logic_vector(9 downto 0);

  -- ------- U/V + sync +4 tail (44 -> 48, align with the Y interpolator) -------
  -- 24 bits = passU10 + passV10 + avid/hs/vs/fld. U/V skip the fader (dry=wet
  -- passthrough) so they ride this tail to reach the same 48 as the faded Y.
  type t_uvs4 is array (0 to 3) of std_logic_vector(23 downto 0);
  signal s_uvs_tail : t_uvs4 := (others => (others => '1'));

  -- ------- Y fader output ----------------------------------------------------
  signal s_mix_y : unsigned(9 downto 0);

begin

  -- ==========================================================================
  -- Control decode (no fabric on knob response; musical scaling lives in TOML).
  -- ==========================================================================
  p_ctrl : process(clk)
    variable v_td  : signed(10 downto 0);
    variable v_tm  : unsigned(9 downto 0);
    variable v_reg : unsigned(9 downto 0);
    variable v_adp : unsigned(9 downto 0);
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

      -- CROSSTALK (knob 5): RESERVED / INERT. Dropped in the Gate-1 timing rework
      -- (the cross-lane V-tap -> mix -> Y-lane Q2 3-input add crashed Fmax to 56.5
      -- MHz). Read for forward-compat + to keep the 12-control map; NOT acted on.
      s_xt_reserved <= unsigned(registers_in(4));

      -- ADAPT (knob 6): energy -> deadzone; bigger knob -> smaller shift -> bigger delta
      v_adp := unsigned(registers_in(5));
      if v_adp > 16 then s_adapt_en <= '1'; else s_adapt_en <= '0'; end if;
      case v_adp(9 downto 8) is                 -- shift LUT (no multiply): 12,9,6,3
        when "00"   => s_adapt_sh <= 12;
        when "01"   => s_adapt_sh <= 9;
        when "10"   => s_adapt_sh <= 6;
        when others => s_adapt_sh <= 3;
      end case;

      s_pol    <= registers_in(6)(2);
      s_hold   <= registers_in(6)(3);
      s_chroma <= registers_in(6)(4);
      s_fader  <= unsigned(registers_in(7));
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

      -- write address/enable pipeline: align to when regen_cur is valid (Q2)
      s_wr_addr1 <= s_qidx;   s_wr_addr <= s_wr_addr1;
      s_wr_en1   <= s_qavid and (not s_hold);
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
              q_th0 => s_q_th(0), q_th1 => s_q_th(1),
              q_th2 => s_q_th(2), q_th3 => s_q_th(3),
              scr_key => s_scr_key,
              tilt_dir => s_tilt_dir, tilt_str => s_tilt_str,
              regen_sh => s_regen_sh, regen_en => s_regen_en,
              hold => s_hold,
              regen_prev => s_prev_y, regen_cur => s_cur_y,
              coef_tap => s_tap_y,
              d_in => data_in.y, d_out => s_org_y);

  -- ==========================================================================
  -- Sync + dry-Y + U/V passthrough circular delay (44). Pure delay -> runs
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

  s_dry_y <= s_rd_q2(33 downto 24);

  -- Chroma toggle (Tog 11): RESERVED/INERT. U/V are PASSTHROUGH (dry=wet, no
  -- blend), not organ-processed; s_chroma is read but not acted on. U/V + sync
  -- take the +4 tail below to reach 48 (the Y fader output depth). Passthrough
  -- does no arithmetic, so U/V need no R1 clamp (input is already in-range).

  -- ==========================================================================
  -- Dry/wet fader : ONE interpolator_u, on Y only (principle #4). +4 clocks.
  -- U/V carry no fader (passthrough). The single * in the design lives here.
  -- ==========================================================================
  mix_y : entity work.interpolator_u
    generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                 G_OUTPUT_MIN => 64, G_OUTPUT_MAX => 940)
    port map (clk => clk, enable => '1',
              a => unsigned(s_dry_y), b => unsigned(s_org_y),
              t => s_fader, result => s_mix_y, valid => open);

  -- ==========================================================================
  -- U/V + sync +4 tail : advance the 44-delayed U/V passthrough AND sync bits to
  -- 48 (interpolator tail), keeping U/V + data_out.sync aligned with the faded Y
  -- — a FIXED delay, independent of every knob/toggle/fader (R5/R8).
  -- ==========================================================================
  p_uvs_tail : process(clk)
  begin
    if rising_edge(clk) then
      s_uvs_tail(0) <= s_rd_q2(23 downto 0);   -- U,V,avid,hs,vs,fld at 44
      s_uvs_tail(1) <= s_uvs_tail(0);
      s_uvs_tail(2) <= s_uvs_tail(1);
      s_uvs_tail(3) <= s_uvs_tail(2);          -- now at 48
    end if;
  end process;

  -- ==========================================================================
  -- Output assignment (every data_out port driven exactly once, R2).
  --   Y  = faded organ (48).   U/V = passthrough via the +4 tail (48).
  -- ==========================================================================
  data_out.y       <= std_logic_vector(s_mix_y);
  data_out.u       <= s_uvs_tail(3)(23 downto 14);
  data_out.v       <= s_uvs_tail(3)(13 downto 4);
  data_out.avid    <= s_uvs_tail(3)(3);
  data_out.hsync_n <= s_uvs_tail(3)(2);
  data_out.vsync_n <= s_uvs_tail(3)(1);
  data_out.field_n <= s_uvs_tail(3)(0);

end architecture sequency;
