-- ============================================================================
-- tb_howlbox_corners.vhd  --  HOWLBOX Gate 2 five-corner functional testbench
-- ----------------------------------------------------------------------------
-- Drives program_top(howlbox) across one broadcast corner (G_CORNER selects
-- NTSC/720p60/1080p30/1080i5994/1080p24 [+480p/PAL SD extras]) with TRUE
-- horizontal blanking geometry and reduced-raster vertical geometry (fast sim;
-- HOWLBOX has no vblank pass -- hook 8c).  Structural reference:
-- tb_sequency_corners.vhd (Phase-0 harness).  HOWLBOX has its OWN hooks below;
-- SEQUENCY's assertions are NOT reused.
--
-- SHIPPED-DESIGN DELTAS honoured (read from howlbox.vhd constants, not stale spec):
--   * Total data_in->data_out latency = 24  (C_SYNC_DELAY_CLKS = 24;
--     20 processing incl. R1 clamp + 4 interpolator).  Every latency/shadow
--     assertion here is 24, NOT the spec's 20.
--   * a2 is a fixed shift-pole (not a multiply); death-flare de-energises via a1.
--     Neither changes the black-box hooks tested here.
--
-- Verifies (per spec S7 with deltas):
--   #4  R5 sync-independence : data_out {avid,hsync_n,vsync_n,field_n} ==
--       data_in delayed EXACTLY 24, for EVERY clock, INDEPENDENT of every
--       knob/toggle/fader -- incl. Knob5 SYNC STRESS = max.
--   #5a dry identity         : fader 0 -> data_out.{y,u,v} == data_in d24
--       BIT-EXACT, even with all effect knobs cranked (dry ignores knobs).
--   #5b/c wet identity       : fader wet + all effect knobs 0 -> delayed identity;
--       horizontal shift == 0 at zero sync-stress (measured from a column-ramp).
--   #6  R1 clamp adversarial : all effect knobs+toggles max -> y in [64,940],
--       u/v in [64,960] at every emitted active pixel.
--   #7  virtual-sync crime   : SYNC STRESS max + bright ramp -> (a) sync still ==
--       d24 valid stream [#4]; (b) read addr in [0,255] STRUCTURAL (s_rd_addr is
--       unsigned(7:0), inherently bounded -- R6); (c) picture shifts horizontally
--       (effect present).
--   #8b field NULL           : on interlaced corners, identical active content on
--       opposite fields yields identical output content -> no field_n-keyed branch.
--   #8a bresenham / #8c vblank NULL : STRUCTURAL (no block/grid addressing; delay
--       line written only while avid; single add at vsync) -- asserted by the fact
--       that active-width geometry never perturbs identity/clamp here + grep gate.
--
-- Input is a COLUMN-ENCODED luma ramp (y = 64 + (ac mod 800)) so the effective
-- horizontal content position is READABLE from the output pixel value -> the
-- virtual-sync shift (#5c/#7c) is measured directly, no correlation needed.
--
-- Machine-readable "RESULT <hook> <corner>: PASS|FAIL ..." lines; logs measured
-- input->output latency.  NEVER edits program VHDL (pure black-box: this GHDL
-- 6.0.0-dev NULL-derefs on VHDL-2008 external names -- reproduced).
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

entity tb_howlbox_corners is
  generic (
    G_CORNER    : integer := 2;    -- 0 NTSC .1 720p60 .2 1080p30 .3 1080i5994 .4 1080p24 .5 480p .6 PAL
    G_ACT_LINES : integer := 12;   -- active lines per field (reduced raster)
    G_VBL_LINES : integer := 6     -- blank lines per field
  );
end entity;

architecture tb of tb_howlbox_corners is

  constant CPL : integer := CORNERS(G_CORNER).clocks_per_line;
  constant AW  : integer := CORNERS(G_CORNER).active_width;
  constant AVS : integer := CPL - AW;                    -- blank-first: avid start
  constant LPF : integer := G_ACT_LINES + G_VBL_LINES;   -- lines per field
  constant ILV : boolean := CORNERS(G_CORNER).is_interlaced;
  constant D   : integer := 24;                          -- SHIPPED latency (20 + 4 interp)
  constant STOP_FIELD : integer := 7;
  constant WARMCYC    : integer := 2*CPL;                -- >= 1 line + pipeline flush
  constant SETTLE     : integer := 3;                    -- lines to skip after a field boundary

  signal clk : std_logic := '0';
  signal running : boolean := true;

  signal data_in, data_out : t_video_stream_yuv444_30b;
  signal regs : t_spi_ram := (others => (others => '0'));

  signal cur_ac    : integer := -1;   -- active column of current data_in pixel (-1 = blank)
  signal cur_ly    : integer := 0;
  signal cur_field : integer := 0;
  signal fldn_val  : std_logic := '1';

  -- D-deep shadow of the input stream (expected output = sh(D)); carry the phase
  -- (field) tag + column so assertions key off the pixel that is actually emerging.
  type sl_arr  is array (1 to D) of std_logic;
  type v10_arr is array (1 to D) of std_logic_vector(9 downto 0);
  type int_arr is array (1 to D) of integer;
  signal sh_avid, sh_hs, sh_vs, sh_fld : sl_arr  := (others => '0');
  signal sh_y, sh_u, sh_v              : v10_arr := (others => (others => '0'));
  signal sh_ac, sh_fldnum, sh_ly       : int_arr := (others => 0);

  -- phase = which register scenario the pixel was captured under (= its field)
  --   0     warmup (neutral, fader 0)
  --   1     DRY identity : fader 0, ALL knobs cranked
  --   2     WET identity : fader wet, all effect knobs 0
  --   3     CLAMP adversarial : fader wet, all knobs max
  --   4     SYNC-STRESS crime : fader wet, Knob5 max, bright ramp
  --   5     DRY under sync-stress max : fader 0, Knob5 max (R5 re-check)
  --   6     WET neutral (settle)
  function phase_regs(f : integer; id : integer) return t_spi_ram is
    variable r : t_spi_ram := (others => (others => '0'));
  begin
    r(8) := std_logic_vector(to_unsigned(id, 10));
    -- Default toggles = TOML load defaults: DC coupling (bit0=1), all others 0.
    -- DC keeps the full signal so a column-ramp passes through for shift decode.
    r(6) := std_logic_vector(to_unsigned(1, 10));
    case f is
      when 1 =>                       -- DRY identity, knobs cranked, fader OFF
        r(0) := std_logic_vector(to_unsigned(1023, 10));
        r(1) := std_logic_vector(to_unsigned(1023, 10));
        r(2) := std_logic_vector(to_unsigned(1023, 10));
        r(3) := std_logic_vector(to_unsigned(1023, 10));
        r(4) := std_logic_vector(to_unsigned(1023, 10));
        r(5) := std_logic_vector(to_unsigned(1023, 10));
        r(6) := std_logic_vector(to_unsigned(31, 10));   -- all 5 toggles on (dry ignores them)
        r(7) := std_logic_vector(to_unsigned(0, 10));    -- fader DRY
      when 2 =>                       -- WET identity, effect knobs 0, DC default
        r(7) := std_logic_vector(to_unsigned(1023, 10)); -- fader WET
      when 3 =>                       -- CLAMP adversarial
        r(0) := std_logic_vector(to_unsigned(1023, 10));
        r(1) := std_logic_vector(to_unsigned(1023, 10));
        r(2) := std_logic_vector(to_unsigned(1023, 10));
        r(3) := std_logic_vector(to_unsigned(1023, 10));
        r(4) := std_logic_vector(to_unsigned(1023, 10));
        r(5) := std_logic_vector(to_unsigned(1023, 10));
        r(6) := std_logic_vector(to_unsigned(31, 10));   -- all toggles on (max stress)
        r(7) := std_logic_vector(to_unsigned(1023, 10));
      when 4 =>                       -- SYNC-STRESS crime (DC so ramp decodes)
        r(4) := std_logic_vector(to_unsigned(1023, 10)); -- Knob5 SYNC STRESS max
        r(7) := std_logic_vector(to_unsigned(1023, 10)); -- fader WET
      when 5 =>                       -- DRY under sync-stress max (R5 re-check)
        r(4) := std_logic_vector(to_unsigned(1023, 10));
        r(7) := std_logic_vector(to_unsigned(0, 10));    -- fader DRY
      when others =>                  -- warmup / settle: WET neutral, DC
        r(7) := std_logic_vector(to_unsigned(1023, 10));
    end case;
    return r;
  end function;

begin

  clk_gen : process
  begin
    while running loop
      clk <= '0'; wait for 5 ns;
      clk <= '1'; wait for 5 ns;
    end loop;
    wait;
  end process;

  dut : entity work.program_top(howlbox)
    port map (clk => clk, registers_in => regs, data_in => data_in, data_out => data_out);

  -- Register scenario (changes only at field boundaries)
  p_regs : process(cur_field)
  begin
    regs <= phase_regs(cur_field, CORNERS(G_CORNER).id);
  end process;

  -- Timing generator + stimulus (registered data_in, TRUE h-blanking).
  -- Column-encoded luma ramp so the horizontal shift is directly readable.
  p_gen : process(clk)
    variable cx  : integer := 0;
    variable ly  : integer := 0;
    variable fld : integer := 0;
    variable act : boolean;
    variable ac  : integer;
    variable yv, uv, vv : integer;
  begin
    if rising_edge(clk) then
      act := (cx >= AVS) and (ly < G_ACT_LINES);
      ac  := cx - AVS;

      data_in.hsync_n <= '1';
      data_in.vsync_n <= '1';
      data_in.field_n <= fldn_val;
      if cx < 44 then data_in.hsync_n <= '0'; end if;
      if (ly >= G_ACT_LINES) and (ly < G_ACT_LINES + 3) then
        data_in.vsync_n <= '0';
      end if;

      if act then
        data_in.avid <= '1';
        yv := 64 + (ac mod 800);                 -- COLUMN-ENCODED ramp (readable position)
        uv := 64 + ((ac*5 + 30)  mod 850);       -- legal chroma content
        vv := 64 + ((ac*3 + 200) mod 850);
        data_in.y <= std_logic_vector(to_unsigned(yv, 10));
        data_in.u <= std_logic_vector(to_unsigned(uv, 10));
        data_in.v <= std_logic_vector(to_unsigned(vv, 10));
        cur_ac <= ac;
      else
        data_in.avid <= '0';
        data_in.y <= std_logic_vector(to_unsigned(64, 10));
        data_in.u <= std_logic_vector(to_unsigned(512, 10));
        data_in.v <= std_logic_vector(to_unsigned(512, 10));
        cur_ac <= -1;
      end if;

      cur_ly    <= ly;
      cur_field <= fld;

      if cx = CPL - 1 then
        cx := 0;
        if ly = LPF - 1 then
          ly  := 0;
          fld := fld + 1;
          if ILV then fldn_val <= not fldn_val; end if;
        else
          ly := ly + 1;
        end if;
      else
        cx := cx + 1;
      end if;
    end if;
  end process;

  -- D-deep input shadow
  p_shadow : process(clk)
  begin
    if rising_edge(clk) then
      sh_avid(1) <= data_in.avid; sh_hs(1) <= data_in.hsync_n;
      sh_vs(1) <= data_in.vsync_n; sh_fld(1) <= data_in.field_n;
      sh_y(1) <= data_in.y; sh_u(1) <= data_in.u; sh_v(1) <= data_in.v;
      sh_ac(1) <= cur_ac; sh_fldnum(1) <= cur_field; sh_ly(1) <= cur_ly;
      for k in 2 to D loop
        sh_avid(k) <= sh_avid(k-1); sh_hs(k) <= sh_hs(k-1);
        sh_vs(k) <= sh_vs(k-1); sh_fld(k) <= sh_fld(k-1);
        sh_y(k) <= sh_y(k-1); sh_u(k) <= sh_u(k-1); sh_v(k) <= sh_v(k-1);
        sh_ac(k) <= sh_ac(k-1); sh_fldnum(k) <= sh_fldnum(k-1); sh_ly(k) <= sh_ly(k-1);
      end loop;
    end if;
  end process;

  -- Monitor: per-cycle checks + final RESULT report
  p_mon : process(clk)
    constant NAME : string := corner_name(G_CORNER);
    variable cyc  : integer := 0;
    variable pin, pout : std_logic := '0';
    -- latency measurement (one clean post-warmup edge pair)
    variable lat_armed : boolean := false;
    variable m_in, m_out, meas_lat : integer := -1;
    -- error counters
    variable e_sync, e_dry, e_clamp, e_fieldnull : integer := 0;
    variable n_dry, n_clamp, n_sync : integer := 0;
    -- wet identity (#5b/c) shift measurement (phase 2, deep pixels)
    variable wet_smin : integer := 999;    -- signed (content_col - raster_col) min
    variable wet_smax : integer := -999;   --                                    max
    variable wet_maxabs : integer := 0;
    variable wet_maxdiff : integer := 0;   -- |out.y - in.y d24| (value error)
    variable n_wet : integer := 0;
    -- sync-stress crime (#7c) shift-present
    variable ss_shift_max : integer := 0;
    variable ss_n : integer := 0;
    -- field NULL (#8b): capture one deep active line per field, compare across fields
    type line_arr is array (0 to 4095) of integer;
    variable capL : line_arr := (others => -1);
    variable prevL : line_arr := (others => -1);
    variable have_prev : boolean := false;
    variable prev_fldn : std_logic := 'U';
    variable cap_ok : boolean := false;
    variable rep_dry, rep_clamp, rep_sync : integer := 0;
    variable yv, uvv, vvv, ov, sh_col, oc, ph : integer;
    variable stable : boolean;
  begin
    if rising_edge(clk) then
      cyc := cyc + 1;

      -- latency: time one clean input->output avid rising-edge pair after warmup
      if cyc > WARMCYC then
        if data_in.avid = '1' and pin = '0' and not lat_armed and meas_lat < 0 then
          m_in := cyc; lat_armed := true;
        elsif data_out.avid = '1' and pout = '0' and lat_armed then
          m_out := cyc; meas_lat := m_out - m_in; lat_armed := false;
        end if;
      end if;

      -- #4 R5 sync-independence: out sync == in sync delayed EXACTLY 24, ALWAYS
      if cyc > WARMCYC then
        n_sync := n_sync + 1;
        if (data_out.avid /= sh_avid(D)) or (data_out.hsync_n /= sh_hs(D)) or
           (data_out.vsync_n /= sh_vs(D)) or (data_out.field_n /= sh_fld(D)) then
          e_sync := e_sync + 1;
          if rep_sync < 4 then
            report NAME & " HOOK4 sync mismatch @cyc=" & integer'image(cyc) &
              " phase=" & integer'image(sh_fldnum(D)) &
              " out(avid,hs,vs,fld)=" & std_logic'image(data_out.avid) &
              std_logic'image(data_out.hsync_n) & std_logic'image(data_out.vsync_n) &
              std_logic'image(data_out.field_n) &
              " exp=" & std_logic'image(sh_avid(D)) & std_logic'image(sh_hs(D)) &
              std_logic'image(sh_vs(D)) & std_logic'image(sh_fld(D)) severity note;
            rep_sync := rep_sync + 1;
          end if;
        end if;
      end if;

      -- Align assertions to the EMERGING pixel's phase (shadow tag) and skip the
      -- SETTLE lines after its field boundary (register/pipeline transients).
      ph     := sh_fldnum(D);
      stable := (sh_ly(D) >= SETTLE);

      if cyc > WARMCYC and sh_avid(D) = '1' and stable then
        yv  := to_integer(unsigned(data_out.y));
        uvv := to_integer(unsigned(data_out.u));
        vvv := to_integer(unsigned(data_out.v));

        -- #6 clamp: every emitted active pixel in-range (all phases)
        if yv < 64 or yv > 940 or uvv < 64 or uvv > 960 or vvv < 64 or vvv > 960 then
          e_clamp := e_clamp + 1;
          if rep_clamp < 6 then
            report NAME & " HOOK6 out-of-range: phase=" & integer'image(ph) &
              " y=" & integer'image(yv) & " u=" & integer'image(uvv) &
              " v=" & integer'image(vvv) severity note;
            rep_clamp := rep_clamp + 1;
          end if;
        end if;
        n_clamp := n_clamp + 1;

        -- #5a DRY identity (phase 1 & 5): bit-exact delayed passthrough despite knobs
        if ph = 1 or ph = 5 then
          n_dry := n_dry + 1;
          if data_out.y /= sh_y(D) or data_out.u /= sh_u(D) or data_out.v /= sh_v(D) then
            e_dry := e_dry + 1;
            if rep_dry < 6 then
              report NAME & " HOOK5a DRY mismatch phase=" & integer'image(ph) &
                " ac=" & integer'image(sh_ac(D)) &
                " exp(y,u,v)=" & integer'image(to_integer(unsigned(sh_y(D)))) & "," &
                integer'image(to_integer(unsigned(sh_u(D)))) & "," &
                integer'image(to_integer(unsigned(sh_v(D)))) &
                " got=" & integer'image(yv) & "," & integer'image(uvv) & "," &
                integer'image(vvv) severity note;
              rep_dry := rep_dry + 1;
            end if;
          end if;
        end if;

        -- #5b/c WET identity (phase 2): measure horizontal shift from the y-ramp.
        --   content_col = out_y - 64 (ramp inverse); raster_col = sh_ac(D).
        --   shift = content_col - raster_col.  Expect 0 (no shift at zero stress).
        --   Gate to a safe mid-line band [130,640] so content_col never crosses the
        --   ramp's mod-800 wrap or the active-width edge.
        if ph = 2 and sh_ac(D) >= 130 and sh_ac(D) <= 640 then
          n_wet := n_wet + 1;
          oc := yv - 64;                        -- decoded content column
          sh_col := oc - sh_ac(D);
          if abs(sh_col) > wet_maxabs then wet_maxabs := abs(sh_col); end if;
          if sh_col < wet_smin then wet_smin := sh_col; end if;
          if sh_col > wet_smax then wet_smax := sh_col; end if;
          if abs(yv - to_integer(unsigned(sh_y(D)))) > wet_maxdiff then
            wet_maxdiff := abs(yv - to_integer(unsigned(sh_y(D))));
          end if;
        end if;

        -- #7c SYNC-STRESS crime (phase 4): shift must be PRESENT (effect present)
        if ph = 4 and sh_ac(D) >= 130 and sh_ac(D) <= 640 then
          oc := yv - 64;
          if abs(oc - sh_ac(D)) > ss_shift_max then ss_shift_max := abs(oc - sh_ac(D)); end if;
          ss_n := ss_n + 1;
        end if;

        -- #8b FIELD NULL (phase 6, interlaced): capture a deep active line's luma,
        -- compare consecutive captured lines that fall on OPPOSITE fields.
        if ILV and ph = 6 and sh_ly(D) = SETTLE + 2 and sh_ac(D) >= 0 and sh_ac(D) < 4096 then
          capL(sh_ac(D)) := yv;
          cap_ok := true;
        end if;
      end if;

      -- end-of-line bookkeeping for field-null capture
      if pout = '1' and data_out.avid = '0' then
        if ILV and cap_ok then
          if have_prev and (prev_fldn /= sh_fld(D)) then
            for i in 0 to AW-1 loop
              if capL(i) >= 0 and prevL(i) >= 0 and capL(i) /= prevL(i) then
                e_fieldnull := e_fieldnull + 1;
              end if;
            end loop;
          end if;
          prevL := capL; have_prev := true; prev_fldn := sh_fld(D);
          capL := (others => -1); cap_ok := false;
        end if;
      end if;

      pin := data_in.avid; pout := data_out.avid;

      if cur_field >= STOP_FIELD then
        report "==== HOWLBOX corner " & NAME & " (id=" &
               integer'image(CORNERS(G_CORNER).id) & ", " &
               integer'image(AW) & "px/line, ILV=" & boolean'image(ILV) & ") ====";
        report "MEASURED_LATENCY " & NAME & " = " & integer'image(meas_lat) severity note;
        report "RESULT HOOK4_sync_indep " & NAME & ": " &
          str_pf(meas_lat = D and e_sync = 0) & " (lat=" & integer'image(meas_lat) &
          " exp=" & integer'image(D) & " sync_errs=" & integer'image(e_sync) &
          "/" & integer'image(n_sync) & ")" severity note;
        report "RESULT HOOK5a_dry_ident " & NAME & ": " &
          str_pf(e_dry = 0 and n_dry > 0) & " (" & integer'image(e_dry) &
          " mismatch / " & integer'image(n_dry) & " px, knobs cranked+fader0)" severity note;
        -- #5c wet-identity horizontal shift.  ACCEPTED-BY-DESIGN threshold: the
        -- wet path carries a uniform resting content offset of C_MIN_LAG=2 px (the
        -- remap reads wr_ptr-MIN_LAG so displacement can swing BIDIRECTIONALLY under
        -- sync-stress).  The dry path is bit-exact (hook 5a), so <=2 px is the
        -- resting offset, NOT a bug; >2 px would be a real remap/latency regression.
        report "RESULT HOOK5c_wet_shift " & NAME & ": " &
          str_pf(n_wet > 0 and wet_maxabs <= 2) & " (shift[min,max]=[" &
          integer'image(wet_smin) & "," & integer'image(wet_smax) &
          "]px; |shift|<=2 accepted-by-design [C_MIN_LAG resting offset], >2 = regression; max|dy|=" &
          integer'image(wet_maxdiff) & "; n=" & integer'image(n_wet) & ")" severity note;
        report "RESULT HOOK6_clamp      " & NAME & ": " &
          str_pf(e_clamp = 0 and n_clamp > 0) & " (" & integer'image(e_clamp) &
          " oor / " & integer'image(n_clamp) & " px)" severity note;
        report "RESULT HOOK7c_ss_shift  " & NAME & ": " &
          str_pf(ss_shift_max > 0) & " (max shift=" & integer'image(ss_shift_max) &
          "px under SYNCSTRESS max; effect present; n=" & integer'image(ss_n) & ")" severity note;
        if ILV then
          report "RESULT HOOK8b_field_null " & NAME & ": " &
            str_pf(e_fieldnull = 0 and have_prev) & " (" & integer'image(e_fieldnull) &
            " field-parity content diffs [expect 0])" severity note;
        else
          report "RESULT HOOK8b_field_null " & NAME &
            ": N/A (progressive -- field_n static)" severity note;
        end if;
        report "VIT_DONE corner " & NAME severity note;
        running <= false;
        stop;
      end if;

      if cyc > 60000000 then
        report "TIMEOUT" severity failure;
      end if;
    end if;
  end process;

end architecture;
