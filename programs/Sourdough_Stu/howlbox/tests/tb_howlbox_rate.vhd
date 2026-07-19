-- ============================================================================
-- tb_howlbox_rate.vhd  --  HOWLBOX Gate 2 hooks #1 (rate-class, THE headline) +
--                         #2 (STARVE state-evolution).  Per corner; run across
--                         all five and aggregate the wall-clock spread in-script.
-- ----------------------------------------------------------------------------
-- THE kit-composition proof: the born-in rate_normalizer must hold a knob's
-- wall-clock feel constant across the 2.5x rate spread (NTSC/1080i5994/720p60 =
-- 60Hz class .. 1080p30 = 30Hz ref .. 1080p24 = 24Hz class).
--
-- OBSERVABILITY (black-box; this GHDL NULL-derefs on external names): s_sag is
-- read out through the CHROMA-leak ripple, which is immune to the death-flare's
-- luma perturbation:
--   * STARVE=380 (steady sag_hi=380, just under the flare knee 384 => pre-knee,
--     LINEAR, collapse=0), RAINBOW=0, all else 0, fader wet, DC, tog9=U-leak.
--   * With RAINBOW=0 the NCO still spins spatially (base step 2) so the U channel
--     carries a spatial sine of amplitude  ~ rainbow_depth = flare>>2 = sag_hi>>2.
--   * U-RANGE (max-min of data_out.u over an active line) = 2x that amplitude,
--     phase-invariant and displacement-invariant (a shifted/constant-input sine
--     keeps its range) => a CLEAN, monotone, linear proxy for s_sag.
--
-- #1 rate class : count output fields (= sag steps) until U-range crosses a fixed
--    level L; cross_field ~ 2^norm_shift(0).  wall-clock = cross_field x real
--    per-field period.  A bypassed/un-normalized shift would make cross_field the
--    SAME at every corner => wall-clock spreads 2.5x (script proves teeth).
-- #2 evolution  : U-range must rise monotonically toward target (non-frozen);
--    a stuck accumulator stays at 0 and MUST fail.
--
-- Emits "RATE_STARVE <corner> cross_field=<n> urange_end=<x> mono=<0|1>";
-- run_corners.sh multiplies cross_field by the real per-field period and asserts
-- the +/-25% (one-rate-class) band across corners.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.video_corner_pkg.all;

entity tb_howlbox_rate is
  generic (
    G_CORNER    : integer := 2;
    G_ACT_LINES : integer := 1;    -- 1 active line (AW>=512 sweeps a full NCO sine)
    G_VBL_LINES : integer := 3;    -- vsync in lines 1..3 => exactly 1 sag step / field
    G_STARVE    : integer := 380;  -- steady sag_hi (< flare knee 384: pre-knee/linear)
    G_LC1       : integer := 6;    -- lower U-range crossing level (~ sag_hi 24)
    G_LC2       : integer := 16;   -- upper U-range crossing level (~ sag_hi 64)
    G_STOP_VS   : integer := 210   -- max fields (60Hz/shift10 reaches L2 ~189)
  );
end entity;

architecture tb of tb_howlbox_rate is
  -- The rate-class hook depends ONLY on reg8=timing_id (-> norm_shift) and the
  -- once-per-vsync sag cadence; it is INDEPENDENT of physical line length (unlike
  -- the sync/width hooks in tb_howlbox_corners, which use TRUE geometry).  HOWLBOX
  -- has no vblank ceiling (hook 8c NULL), so no true-blanking constraint applies
  -- here.  Reduced geometry (AW>=512 keeps a full NCO sine) makes all 5 corners
  -- fast.  The REAL timing_id and interlace flag are retained per corner.
  constant CPL : integer := 700;                       -- reduced clocks/line
  constant AW  : integer := 600;                        -- >=512 => full NCO sine swing
  constant AVS : integer := CPL - AW;
  constant LPF : integer := G_ACT_LINES + G_VBL_LINES;
  constant ILV : boolean := CORNERS(G_CORNER).is_interlaced;

  signal clk : std_logic := '0';
  signal running : boolean := true;
  signal data_in, data_out : t_video_stream_yuv444_30b;
  signal regs : t_spi_ram := (others => (others => '0'));
  signal fldn_val : std_logic := '1';
begin
  clk_gen : process begin
    while running loop clk<='0'; wait for 5 ns; clk<='1'; wait for 5 ns; end loop; wait;
  end process;

  dut : entity work.program_top(howlbox)
    port map (clk => clk, registers_in => regs, data_in => data_in, data_out => data_out);

  -- Static registers (drive the WHOLE array so no element stays metavalue 'U'):
  -- STARVE step engaged from t0; RAINBOW 0; DC couple; fader wet.
  p_regs : process(clk)
    variable r : t_spi_ram := (others => (others => '0'));
  begin
    if rising_edge(clk) then
      r := (others => (others => '0'));
      r(5) := std_logic_vector(to_unsigned(G_STARVE, 10));            -- STARVE
      r(6) := std_logic_vector(to_unsigned(1, 10));                  -- DC couple, tog9 = U-leak
      r(7) := std_logic_vector(to_unsigned(1023, 10));               -- fader WET
      r(8) := std_logic_vector(to_unsigned(CORNERS(G_CORNER).id, 10));
      regs <= r;
    end if;
  end process;

  -- Stimulus: FLAT luma + FLAT chroma (U=V=512).  Any output-U variation is purely
  -- the sag-driven NCO chroma leak.
  p_gen : process(clk)
    variable cx, ly : integer := 0;
    variable act : boolean;
  begin
    if rising_edge(clk) then
      act := (cx >= AVS) and (ly < G_ACT_LINES);
      data_in.hsync_n <= '1'; data_in.vsync_n <= '1'; data_in.field_n <= fldn_val;
      if cx < 44 then data_in.hsync_n <= '0'; end if;
      if (ly >= G_ACT_LINES) and (ly < G_ACT_LINES + 3) then data_in.vsync_n <= '0'; end if;
      if act then data_in.avid <= '1'; else data_in.avid <= '0'; end if;
      data_in.y <= std_logic_vector(to_unsigned(512, 10));
      data_in.u <= std_logic_vector(to_unsigned(512, 10));
      data_in.v <= std_logic_vector(to_unsigned(512, 10));
      if cx = CPL-1 then
        cx := 0;
        if ly = LPF-1 then ly := 0; if ILV then fldn_val <= not fldn_val; end if;
        else ly := ly + 1; end if;
      else cx := cx + 1; end if;
    end if;
  end process;

  -- Monitor: per OUTPUT field, U-range = max-min(data_out.u) over active pixels;
  -- detect the crossing field + monotonicity.
  p_mon : process(clk)
    constant NAME : string := corner_name(G_CORNER);
    variable pav, pvs : std_logic := '0';
    variable umin : integer := 1023;
    variable umax : integer := 0;
    variable fld : integer := 0;
    variable uv : integer;
    type ia is array(0 to G_STOP_VS) of integer;
    variable urange : ia := (others => -1);
    variable f1, f2 : integer := -1;   -- fields to reach L1, L2
    variable mono_ok : boolean := true;
    variable mono_i  : integer := 1;
    variable prev_ur : integer := 0;
    variable i : integer;
    variable cyc : integer := 0;
  begin
    if rising_edge(clk) then
      cyc := cyc + 1;
      if cyc > 4000000 then report "TIMEOUT rate" severity failure; end if;
      -- accumulate U min/max over active output pixels of the current field
      if data_out.avid = '1' then
        uv := to_integer(unsigned(data_out.u));
        if uv < umin then umin := uv; end if;
        if uv > umax then umax := uv; end if;
      end if;

      -- output field boundary = data_out.vsync_n falling edge
      if data_out.vsync_n = '0' and pvs = '1' then
        if fld <= G_STOP_VS then
          if umax >= umin then urange(fld) := umax - umin; else urange(fld) := 0; end if;
        end if;
        umin := 1023; umax := 0;         -- reset for next field
        fld := fld + 1;
      end if;

      pav := data_out.avid; pvs := data_out.vsync_n;

      if fld > G_STOP_VS or (not running) then
        -- Two-level crossing: field to reach L1 and L2.  The DELTA (f2-f1) cancels
        -- the constant pipeline/warmup offset and is proportional to 2^norm_shift(0)
        -- exactly -> the clean rate-class metric the aggregator uses.
        -- skip fields 0..2: the remap RAM is still filling then, so its unwritten
        -- reads decode as metavalue->0 and inflate early U-range.  Delta = f2-f1
        -- cancels this constant offset anyway.
        for i in 3 to G_STOP_VS loop
          if f1 < 0 and urange(i) >= G_LC1 then f1 := i; end if;
          if f2 < 0 and urange(i) >= G_LC2 then f2 := i; end if;
        end loop;
        -- monotonicity: U-range non-decreasing across the measured span (allow
        -- equal; a FROZEN accumulator stays flat at ~0 and never crosses).
        prev_ur := -1;
        for i in 3 to G_STOP_VS loop
          if urange(i) >= 0 then
            if urange(i) + 2 < prev_ur then mono_ok := false; end if;  -- allow tiny quant dips
            prev_ur := urange(i);
          end if;
        end loop;
        if mono_ok then mono_i := 1; else mono_i := 0; end if;
        report "==== HOWLBOX rate " & NAME & " (id=" &
               integer'image(CORNERS(G_CORNER).id) & ") ====";
        report "RATE_STARVE " & NAME & " id=" & integer'image(CORNERS(G_CORNER).id) &
               " f1=" & integer'image(f1) & " f2=" & integer'image(f2) &
               " delta=" & integer'image(f2 - f1) &
               " urange_end=" & integer'image(urange(G_STOP_VS)) &
               " mono=" & integer'image(mono_i) severity note;
        report "RESULT HOOK2_sag_evolve " & NAME & ": " &
               str_pf(f1 > 0 and f2 > f1 and urange(G_STOP_VS) > urange(5) and mono_ok) &
               " (early=" & integer'image(urange(5)) & " end=" &
               integer'image(urange(G_STOP_VS)) & " f1=" & integer'image(f1) &
               " f2=" & integer'image(f2) & " non-frozen)" severity note;
        report "VIT_DONE rate " & NAME severity note;
        running <= false;
        stop;
      end if;
      if fld > G_STOP_VS then running <= false; end if;
    end if;
  end process;
end architecture;
