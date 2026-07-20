-- ============================================================================
-- tb_witness_gaze.vhd  --  WITNESS Lite Gate 2 gaze / temporal-state hooks
-- ----------------------------------------------------------------------------
-- The gaze (block centroid) is only externally visible through the IRIS it
-- steers.  This GHDL 6.0-dev NULL-derefs on VHDL-2008 external names (SEQUENCY-
-- confirmed), so cx_blk/cy_blk/the store are observed through the OUTPUT via a
-- BEHAVIORAL PROXY: max output-Y over a band's rows (mB / mA).
--
--   PROXY: a full-width bright BAND (Y=940) at a known raster row drives the
--   first-moment centroid to that row's block.  With a TIGHT iris (K1=0, radius^2
--   ~0) the IRIS darkens the band the FARTHER the lagged gaze is (vertically) from
--   it, so the per-field MAX-Y over the band's rows rises toward 940 as the gaze
--   arrives and dips when it is repelled/lagging.  Max-over-row is invariant to a
--   horizontal shift, so this reads the VERTICAL gaze (cy_blk) even while the PULL
--   wet-path defect (tb_witness_corners H1b) displaces content horizontally.
--   Horizontal gaze (cx_blk) and precise ±½-block/rate-class resolution are NOT
--   cleanly recoverable through the (gentle, block-quantized) iris + corrupted wet
--   path -> the gaze/temporal hooks are BLOCKED pending the H1b PULL fix; this TB
--   is the regression scaffold to RE-RUN once the wet path is clean.
--
-- Three DUTs at 1080p30(ref) / 1080p24 / 1080i5994 share ONE reduced-raster
-- stimulus; only registers_in(8) timing id differs (the rate_normalizer input).
--
--   G1 centroid track : settled mB@band high (gaze reached the band).
--   G2 store evolution: mB DIPS at the band step then RECOVERS (store not frozen).
--   G3 LAG rate-class : fields-to-recover x real field-period ~ same wall-clock
--                       within +-25% across 24/30/5994 (coarse; proxy-limited).
--   G4 EVADE (T8)     : mB DROPS on evade (gaze repelled), RECOVERS on release
--                       (no stuck state, R7).
--
-- NOTE (verifier): NEVER edits program VHDL.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

library work;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.video_corner_pkg.all;

entity tb_witness_gaze is
  generic (
    G_ACT_LINES : integer := 32;   -- active lines/field (1:1 line->block-row)
    G_VBL_LINES : integer := 3     -- blank lines/field (>2048 clk => solve completes)
  );
end entity;

architecture tb of tb_witness_gaze is
  constant CPL : integer := 2200;                       -- 1080p30-class htotal (true h-blank)
  constant AW  : integer := 1920;
  constant AVS : integer := CPL - AW;
  constant LPF : integer := G_ACT_LINES + G_VBL_LINES;
  constant NROWS : integer := G_ACT_LINES;

  -- band: bright full-width strip; A before the step, B after (block rows ~4 -> ~24)
  constant BH      : integer := 4;
  constant A_TOP   : integer := 4;               -- warmup band top line  (block ~4)
  constant B_TOP   : integer := 24;              -- post-step band top    (block ~24)
  constant A_CEN   : integer := A_TOP + BH/2;
  constant B_CEN   : integer := B_TOP + BH/2;
  constant STEP_FIELD  : integer := 4;
  constant EVADE_FIELD : integer := 16;
  constant REL_FIELD   : integer := 20;
  constant STOP_FIELD  : integer := 24;
  constant TOL     : integer := 4;               -- +-4 lines ~ +-2.7 block-rows (PULL-confounded proxy)
  constant WARM    : integer := 2*CPL;

  -- real field periods (ms*100 -> integer) for wall-clock rate-class math
  --   1080p30 30.00 f/s -> 3337 ; 1080p24 24.00 -> 4167 ; 1080i5994 59.94 -> 1668
  type per_t is array(0 to 2) of integer;
  constant PERIOD : per_t := (0 => 3337, 1 => 4167, 2 => 1668);   -- x0.01 ms / field
  constant TIDS   : per_t := (0 => 7,    1 => 3,    2 => 2);       -- timing ids

  signal clk : std_logic := '0';
  signal running : boolean := true;
  signal di : t_video_stream_yuv444_30b;
  type do_arr is array(0 to 2) of t_video_stream_yuv444_30b;
  signal do : do_arr;
  type reg_arr is array(0 to 2) of t_spi_ram;
  signal regs : reg_arr := (others => (others => (others => '0')));
  signal cur_field : integer := 0;

  -- mB(dut,field) = max output-Y over the B-band rows; mA(field) = same over A-band
  -- rows for dut0. Row-max is invariant to the PULL horizontal shift (rows are not
  -- reordered vertically); high value => gaze vertically ON that band, low => off.
  type mb_arr is array(0 to 2, 0 to STOP_FIELD) of integer;
  signal mB : mb_arr := (others => (others => -1));
  type ma_arr is array(0 to STOP_FIELD) of integer;
  signal mA : ma_arr := (others => -1);
  constant THRESH : integer := 900;   -- band=940 out; gaze-on >~900, gaze-off dips below
begin

  clk_gen : process begin
    while running loop clk<='0'; wait for 5 ns; clk<='1'; wait for 5 ns; end loop; wait;
  end process;

  gen_dut : for i in 0 to 2 generate
    dut : entity work.program_top(witness)
      port map (clk => clk, registers_in => regs(i), data_in => di, data_out => do(i));
  end generate;

  -- registers: common LIVE effect; per-DUT timing id; T8 EVADE in [EVADE,REL)
  p_regs : process(cur_field)
    variable r : t_spi_ram;
  begin
    for i in 0 to 2 loop
      r := (others => (others => '0'));
      r(0) := std_logic_vector(to_unsigned(0, 10));     -- K1 IRIS TIGHT (radius^2=0 -> vertical distance darkens)
      r(2) := std_logic_vector(to_unsigned(128, 10));   -- K3 LAG moderate (base_shift 2 -> multi-field settle, resolvable)
      r(3) := std_logic_vector(to_unsigned(512, 10));   -- K4 PULL neutral
      r(5) := std_logic_vector(to_unsigned(0, 10));     -- K6 EDGE hard (min falloff shift => strongest darkening)
      r(7) := std_logic_vector(to_unsigned(1023,10));   -- Fader WET (gaze visible)
      r(8) := std_logic_vector(to_unsigned(TIDS(i), 10));
      if cur_field >= EVADE_FIELD and cur_field < REL_FIELD then
        r(6) := std_logic_vector(to_unsigned(2, 10));   -- T8 EVADE on (bit1)
      end if;
      regs(i) <= r;
    end loop;
  end process;

  -- stimulus: full-width bright band (Y~900) at band_top; dark (Y~80) elsewhere.
  -- U/V neutral (512) => hue bias self-nulls => pure brightness centroid.
  p_gen : process(clk)
    variable cx, ly, fld : integer := 0;
    variable act : boolean; variable ac, top : integer;
  begin
    if rising_edge(clk) then
      act := (cx >= AVS) and (ly < G_ACT_LINES);
      ac  := cx - AVS;
      di.hsync_n <= '1'; di.vsync_n <= '1'; di.field_n <= '1';
      if cx < 44 then di.hsync_n <= '0'; end if;
      if (ly >= G_ACT_LINES) and (ly < G_ACT_LINES + 2) then di.vsync_n <= '0'; end if;
      if fld < STEP_FIELD then top := A_TOP; else top := B_TOP; end if;
      if act then
        di.avid <= '1';
        if (ly >= top) and (ly < top + BH) then
          di.y <= std_logic_vector(to_unsigned(940, 10));   -- band (== output Y clamp max)
        else
          di.y <= std_logic_vector(to_unsigned(64, 10));    -- dark background
        end if;
        di.u <= std_logic_vector(to_unsigned(512, 10));
        di.v <= std_logic_vector(to_unsigned(512, 10));
      else
        di.avid <= '0';
        di.y <= std_logic_vector(to_unsigned(64, 10));
        di.u <= std_logic_vector(to_unsigned(512, 10));
        di.v <= std_logic_vector(to_unsigned(512, 10));
      end if;
      cur_field <= fld;
      if cx = CPL - 1 then
        cx := 0;
        if ly = LPF - 1 then ly := 0; fld := fld + 1; else ly := ly + 1; end if;
      else cx := cx + 1; end if;
    end if;
  end process;

  -- per-field max output-Y over the A-band and B-band row windows; argmax-free so
  -- the PULL horizontal shift (rows preserved) does not disturb it.
  p_meas : process(clk)
    variable cyc : integer := 0;
    variable pvs : std_logic := '1';
    variable oline : integer := 0;
    variable ofld  : integer := 0;
    variable pav : std_logic := '0';
    type m3 is array(0 to 2) of integer;
    variable curB : m3 := (others => 0);
    variable curA : integer := 0;
    variable yi, dd : integer;
  begin
    if rising_edge(clk) then
      cyc := cyc + 1;
      -- inside a band-row window: track running max-Y per DUT (dut0 avid = line clock)
      if do(0).avid = '1' then
        if oline >= B_TOP and oline < B_TOP + BH then
          for dd in 0 to 2 loop
            if not is_x(do(dd).y) then
              yi := to_integer(unsigned(do(dd).y));
              if yi > curB(dd) then curB(dd) := yi; end if;
            end if;
          end loop;
        end if;
        if oline >= A_TOP and oline < A_TOP + BH and not is_x(do(0).y) then
          yi := to_integer(unsigned(do(0).y));
          if yi > curA then curA := yi; end if;
        end if;
      end if;
      if do(0).avid = '1' and pav = '0' then oline := oline + 1; end if;
      -- output field boundary
      if do(0).vsync_n = '0' and pvs = '1' then
        if cyc > WARM and ofld <= STOP_FIELD then
          for dd in 0 to 2 loop mB(dd, ofld) <= curB(dd); end loop;
          mA(ofld) <= curA;
        end if;
        curB := (others => 0); curA := 0; oline := 0; ofld := ofld + 1;
      end if;
      pav := do(0).avid;
      pvs := do(0).vsync_n;

      if cur_field >= STOP_FIELD then running <= false; stop; end if;
      if cyc > 40000000 then report "TIMEOUT" severity failure; end if;
    end if;
  end process;

  -- final evaluation (max-Y-at-band-row dynamics = gaze vertical proximity)
  p_eval : process
    variable settle : per_t := (others => -1);
    variable wc : per_t;
    variable dd, f : integer;
    variable g1, g2, g4a, g4b : boolean;
    variable ratemin, ratemax, ref : integer;
  begin
    wait until cur_field >= STOP_FIELD;
    wait for 1 ns;

    -- print the dut30 B-band max-Y history (visibility of the LAG/EVADE dynamics)
    for f in 0 to STOP_FIELD loop
      report "GAZE_TRACE f=" & integer'image(f) & " mA=" & integer'image(mA(f)) &
        " mB30=" & integer'image(mB(0,f)) & " mB24=" & integer'image(mB(1,f)) &
        " mB60=" & integer'image(mB(2,f)) severity note;
    end loop;

    -- G1 centroid: gaze reaches A pre-step (mA high) AND reaches B post-settle (mB high)
    g1 := (mA(STEP_FIELD-1) >= THRESH) and (mB(0, EVADE_FIELD-1) >= THRESH);

    -- G2 evolution: mB(B-band) is LOW just after the step (gaze lagging at A) then
    -- RECOVERS high (gaze moved to B) -> the store/gaze evolved (not frozen).
    g2 := (mB(0, STEP_FIELD) < THRESH) and (mB(0, EVADE_FIELD-1) >= THRESH);

    -- G3 rate-class: fields after the step until mB >= THRESH, per DUT -> wall-clock
    for dd in 0 to 2 loop
      for f in STEP_FIELD to EVADE_FIELD-1 loop
        if settle(dd) < 0 and mB(dd, f) >= THRESH then settle(dd) := f - STEP_FIELD + 1; end if;
      end loop;
      if settle(dd) < 0 then settle(dd) := EVADE_FIELD - STEP_FIELD; end if;
      wc(dd) := settle(dd) * PERIOD(dd);
    end loop;
    ref := wc(0); ratemin := ref - (ref/4); ratemax := ref + (ref/4);
    report "GAZE rate-class wall-clock (x0.01ms): p30=" & integer'image(wc(0)) &
      " (" & integer'image(settle(0)) & "f)  p24=" & integer'image(wc(1)) &
      " (" & integer'image(settle(1)) & "f)  i5994=" & integer'image(wc(2)) &
      " (" & integer'image(settle(2)) & "f)  ref+-25%=[" & integer'image(ratemin) &
      "," & integer'image(ratemax) & "]" severity note;

    -- G4 EVADE: mB DROPS during the evade window (gaze repelled from B) then RECOVERS
    -- after release (no stuck state, R7).
    g4a := mB(0, EVADE_FIELD+2) < THRESH;
    g4b := mB(0, STOP_FIELD-1) >= THRESH;

    report "RESULT G1_centroid_track 1080p30: " & str_pf(g1) &
      " (mA@A=" & integer'image(mA(STEP_FIELD-1)) & " mB@B=" & integer'image(mB(0,EVADE_FIELD-1)) & ")" severity note;
    report "RESULT G2_store_evolution 1080p30: " & str_pf(g2) &
      " (mB dip@step=" & integer'image(mB(0,STEP_FIELD)) & " recover=" & integer'image(mB(0,EVADE_FIELD-1)) & ")" severity note;
    report "RESULT G3_LAG_rateclass  24/30/5994: " &
      str_pf(wc(1) >= ratemin and wc(1) <= ratemax and wc(2) >= ratemin and wc(2) <= ratemax) severity note;
    report "RESULT G4_EVADE_repel_return 1080p30: " & str_pf(g4a and g4b) &
      " (repelled=" & str_pf(g4a) & " returned=" & str_pf(g4b) & ")" severity note;
    report "VIT_DONE gaze" severity note;
    wait;
  end process;

end architecture;
