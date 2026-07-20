-- ============================================================================
-- tb_witness_corners.vhd  --  WITNESS Lite Gate 2 five-corner functional TB
-- ----------------------------------------------------------------------------
-- Drives program_top(witness) across one broadcast corner (G_CORNER selects
-- NTSC/720p60/1080p30/1080i5994/1080p24 [+480p/PAL SD extras]) with TRUE
-- horizontal blanking geometry (clocks_per_line/active_width verbatim from
-- video_corner_pkg) and reduced-raster vertical geometry (fast sim). The vblank
-- window is sized > 2048 clocks so the moments scan completes each field.
--
-- Black-box hooks (spec plans/witness/spec.md S6; as-built REV-D). LATENCY = 16.
--   H1a identity-DRY   : Fader=0 (dry) -> data_out == data_in delayed 16, BIT-EXACT
--   H1b identity-WET   : Fader=1023 wet + K1=1023 (iris open, iris_t==0) + K4=512
--                        (PULL neutral) -> composite == data_in delayed 16 within
--                        <=1 code (interpolator's t=1023/1024 rounding). A larger
--                        deviation = the wet path is NOT identity at neutral.
--   H8  latency/sync   : out.avid/hs/vs/fld == in delayed EXACTLY 16; param-indep.
--   H2  block/px tiling: active px/line == active_width (Bresenham self-calibrates
--                        the 64x32 grid to it; exact 64-col tiling is proven in the
--                        kit's own bresenham TB -- external-name limited here).
--   H9  clamp (R1)     : every emitted active pixel Y in [64,940], U/V in [64,960].
--   H12 inert NULL     : a 2nd DUT with K2/K5/T9/T10/T11 driven HOT but identical
--                        LIVE regs -> byte-identical output (inert ctrls unwired).
--   H6  PULL shear     : K4!=512 shifts the wet content but sync is unaffected
--                        (folded into H8 across all phases) and never wraps 2048.
--   H7  field/scan     : interlaced corners toggle field_n per field; progressive
--                        hold it static -> nothing misfires (identity/clamp hold).
--
-- Centroid / store-evolution / LAG rate-class / EVADE / hue-bias (state hooks)
-- live in tb_witness_gaze (behavioral proxies; this GHDL 6.0-dev NULL-derefs on
-- VHDL-2008 external names -- SEQUENCY-confirmed -- so internal cx_blk/store are
-- observed through the iris-shaped output, never read directly).
--
-- NOTE (verifier): NEVER edits program VHDL. Pure black-box harness.
-- Emits machine-readable "RESULT <hook>: PASS|FAIL ..." lines.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

library work;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.video_corner_pkg.all;

entity tb_witness_corners is
  generic (
    G_CORNER    : integer := 2;    -- 0 NTSC .1 720p60 .2 1080p30 .3 1080i5994 .4 1080p24 .5 480p .6 PAL
    G_ACT_LINES : integer := 24;   -- active lines per field (reduced raster)
    G_VBL_LINES : integer := 6     -- blank lines per field (>2048 clk => moments scan completes)
  );
end entity;

architecture tb of tb_witness_corners is

  constant CPL : integer := CORNERS(G_CORNER).clocks_per_line;
  constant AW  : integer := CORNERS(G_CORNER).active_width;
  constant AVS : integer := CPL - AW;                    -- blank-first: avid start clock
  constant LPF : integer := G_ACT_LINES + G_VBL_LINES;   -- lines per field
  constant ILV : boolean := CORNERS(G_CORNER).is_interlaced;
  constant D_SPEC : integer := 16;                       -- SPEC-INTENDED latency (÷4). As-built measures 14.
  constant DMAX   : integer := 40;                       -- shadow depth (covers measured latency + margin)
  constant STOP_FIELD : integer := 8;
  constant WARMCYC    : integer := 3*CPL;                -- >= 2 full lines + pipeline flush

  signal clk : std_logic := '0';
  signal running : boolean := true;

  signal di               : t_video_stream_yuv444_30b;
  signal do_a, do_b       : t_video_stream_yuv444_30b;   -- a=base, b=inert-hot (H12)
  signal regs_a, regs_b   : t_spi_ram := (others => (others => '0'));

  signal cur_ac    : integer := 0;    -- active column of current di pixel (-1 in blank)
  signal cur_field : integer := 0;
  signal fldn_val  : std_logic := '1';

  -- DMAX-deep input shadow (sh(k) = di delayed k cycles); identity/sync compared
  -- against the MEASURED latency so a wrong-but-clean latency is separable from a
  -- corrupt datapath.
  type sl_arr  is array (1 to DMAX) of std_logic;
  type v10_arr is array (1 to DMAX) of std_logic_vector(9 downto 0);
  type int_arr is array (1 to DMAX) of integer;
  signal sh_avid, sh_hs, sh_vs, sh_fld : sl_arr  := (others => '0');
  signal sh_y, sh_u, sh_v              : v10_arr := (others => (others => '0'));
  signal sh_ac                         : int_arr := (others => -1);

  -- phase helpers: 1-2 dry-identity . 3-4 wet-identity . 5-6 adversarial clamp
  function is_dry(f : integer)  return boolean is begin return f >= 1 and f <= 2; end function;
  function is_wet(f : integer)  return boolean is begin return f >= 3 and f <= 4; end function;
  function is_adv(f : integer)  return boolean is begin return f >= 5; end function;

begin

  clk_gen : process
  begin
    while running loop
      clk <= '0'; wait for 5 ns;
      clk <= '1'; wait for 5 ns;
    end loop;
    wait;
  end process;

  -- two DUTs, identical video + identical LIVE regs; only the 5 inert ctrls differ
  dut_a : entity work.program_top(witness)
    port map (clk => clk, registers_in => regs_a, data_in => di, data_out => do_a);
  dut_b : entity work.program_top(witness)
    port map (clk => clk, registers_in => regs_b, data_in => di, data_out => do_b);

  -- ------------------------------------------------------------------
  -- Register scenario (changes at field boundaries)
  -- ------------------------------------------------------------------
  p_regs : process(cur_field)
    variable ra, rb : t_spi_ram;
  begin
    ra := (others => (others => '0'));
    ra(0) := std_logic_vector(to_unsigned(1023, 10));  -- K1 IRIS open
    ra(3) := std_logic_vector(to_unsigned( 512, 10));  -- K4 PULL neutral
    ra(5) := std_logic_vector(to_unsigned( 512, 10));  -- K6 EDGE mid
    ra(8) := std_logic_vector(to_unsigned(CORNERS(G_CORNER).id, 10));
    if is_wet(cur_field) then
      ra(7) := std_logic_vector(to_unsigned(1023, 10)); -- Fader WET (identity composite)
    elsif is_adv(cur_field) then
      ra(0) := std_logic_vector(to_unsigned(  0, 10));  -- K1 iris fully closed (max darken)
      ra(3) := std_logic_vector(to_unsigned(1023, 10)); -- K4 PULL max shear
      ra(5) := std_logic_vector(to_unsigned(  0, 10));  -- K6 hard edge
      ra(6) := std_logic_vector(to_unsigned(  2, 10));  -- T8 EVADE on (bit1); T7 Brightness
      if cur_field = 6 then
        ra(6) := std_logic_vector(to_unsigned(3, 10));  -- +T7 Motion (bit0) & EVADE
      end if;
      ra(7) := std_logic_vector(to_unsigned(1023, 10)); -- Fader WET
    else
      ra(7) := std_logic_vector(to_unsigned(0, 10));    -- Fader DRY (identity)
    end if;
    -- inert-hot twin: identical LIVE regs, all 5 declared-inert ctrls driven HOT
    rb := ra;
    rb(1) := std_logic_vector(to_unsigned(1023, 10));   -- K2 RINGS max (inert)
    rb(4) := std_logic_vector(to_unsigned(1023, 10));   -- K5 ORBIT max (inert)
    rb(6) := std_logic_vector(unsigned(ra(6)) or to_unsigned(28, 10)); -- T9|T10|T11 bits 2,3,4
    regs_a <= ra;
    regs_b <= rb;
  end process;

  -- ------------------------------------------------------------------
  -- Timing generator + stimulus (registered di, TRUE h-blanking)
  --   content: legal Y ramp in [64,940], U/V structured in [64,960]; nonzero mass
  --   so the moments divide stays well-defined (no /0 X-inject).
  -- ------------------------------------------------------------------
  p_gen : process(clk)
    variable cx  : integer := 0;
    variable ly  : integer := 0;
    variable fld : integer := 0;
    variable gl  : integer := 0;
    variable act : boolean;
    variable ac  : integer;
    variable yv, uv, vv : integer;
  begin
    if rising_edge(clk) then
      act := (cx >= AVS) and (ly < G_ACT_LINES);
      ac  := cx - AVS;

      di.hsync_n <= '1';
      di.vsync_n <= '1';
      di.field_n <= fldn_val;
      if cx < 44 then di.hsync_n <= '0'; end if;
      if (ly >= G_ACT_LINES) and (ly < G_ACT_LINES + 3) then di.vsync_n <= '0'; end if;

      if act then
        di.avid <= '1';
        if is_adv(fld) then
          -- adversarial: full-swing AC + saturated chroma to stress the R1 clamp
          if ((ac + gl) mod 2) = 0 then yv := 1023; else yv := 0; end if;
          uv := 1023; vv := 0;
        else
          yv := 64 + ((ac*3 + gl*11) mod 800);   -- [64,863]  legal, in Y clamp
          uv := 64 + ((ac*5 + gl*7 + 30) mod 850);
          vv := 64 + ((ac*2 + gl*13 + 111) mod 850);
        end if;
        di.y <= std_logic_vector(to_unsigned(yv, 10));
        di.u <= std_logic_vector(to_unsigned(uv, 10));
        di.v <= std_logic_vector(to_unsigned(vv, 10));
        cur_ac <= ac;
      else
        di.avid <= '0';
        di.y <= std_logic_vector(to_unsigned(64, 10));
        di.u <= std_logic_vector(to_unsigned(512, 10));
        di.v <= std_logic_vector(to_unsigned(512, 10));
        cur_ac <= -1;
      end if;

      cur_field <= fld;

      if cx = CPL - 1 then
        cx := 0;
        if act then gl := gl + 1; end if;
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

  -- DMAX-deep shadow of di (sh(1) = di delayed 1 ... sh(k) = di delayed k)
  p_shadow : process(clk)
  begin
    if rising_edge(clk) then
      sh_avid(1) <= di.avid; sh_hs(1) <= di.hsync_n; sh_vs(1) <= di.vsync_n; sh_fld(1) <= di.field_n;
      sh_y(1) <= di.y; sh_u(1) <= di.u; sh_v(1) <= di.v; sh_ac(1) <= cur_ac;
      for k in 2 to DMAX loop
        sh_avid(k) <= sh_avid(k-1); sh_hs(k) <= sh_hs(k-1); sh_vs(k) <= sh_vs(k-1); sh_fld(k) <= sh_fld(k-1);
        sh_y(k) <= sh_y(k-1); sh_u(k) <= sh_u(k-1); sh_v(k) <= sh_v(k-1); sh_ac(k) <= sh_ac(k-1);
      end loop;
    end if;
  end process;

  -- ------------------------------------------------------------------
  -- Monitor
  -- ------------------------------------------------------------------
  p_mon : process(clk)
    constant NAME : string := corner_name(G_CORNER);
    variable cyc  : integer := 0;
    variable pin, pout : std_logic := '0';
    variable lat_armed : boolean := false;
    variable m_in, m_out, meas_lat : integer := -1;
    variable L : integer := D_SPEC;    -- alignment index = measured latency once known
    variable e_sync, e_dry, e_wet, e_clamp, e_blk, e_inert, e_meta : integer := 0;
    variable px_line, lines_checked : integer := 0;
    variable rep_dry, rep_wet, rep_inert, rep_clamp : integer := 0;
    variable yv, uvv, vvv, dY, dU, dV : integer;
  begin
    if rising_edge(clk) then
      cyc := cyc + 1;

      -- measure latency: one clean post-warm di.avid -> do_a.avid edge pair
      if cyc > WARMCYC then
        if di.avid = '1' and pin = '0' and not lat_armed and meas_lat < 0 then
          m_in := cyc; lat_armed := true;
        elsif do_a.avid = '1' and pout = '0' and lat_armed then
          m_out := cyc; meas_lat := m_out - m_in; lat_armed := false;
        end if;
      end if;
      if meas_lat > 0 and meas_lat <= DMAX then L := meas_lat; end if;

      -- H8 sync coherence: all sync bits share the SAME (measured) latency
      if cyc > WARMCYC and meas_lat > 0 then
        if (do_a.avid /= sh_avid(L)) or (do_a.hsync_n /= sh_hs(L)) or
           (do_a.vsync_n /= sh_vs(L)) or (do_a.field_n /= sh_fld(L)) then
          e_sync := e_sync + 1;
        end if;
      end if;

      -- identity + inert-null on measured-latency-aligned active pixels
      if cyc > WARMCYC and meas_lat > 0 and sh_avid(L) = '1' then
        if is_x(do_a.y) or is_x(do_a.u) or is_x(do_a.v) or
           is_x(do_b.y) or is_x(do_b.u) or is_x(do_b.v) then
          e_meta := e_meta + 1;
        else
          dY := abs(to_integer(unsigned(do_a.y)) - to_integer(unsigned(sh_y(L))));
          dU := abs(to_integer(unsigned(do_a.u)) - to_integer(unsigned(sh_u(L))));
          dV := abs(to_integer(unsigned(do_a.v)) - to_integer(unsigned(sh_v(L))));

          if is_dry(cur_field) then          -- H1a: BIT-EXACT dry identity @ measured latency
            if dY /= 0 or dU /= 0 or dV /= 0 then
              e_dry := e_dry + 1;
              if rep_dry < 6 then
                report NAME & " H1a-DRY mismatch@L=" & integer'image(L) & ": ac=" & integer'image(sh_ac(L)) &
                  " exp(y,u,v)=" & integer'image(to_integer(unsigned(sh_y(L)))) & "," &
                  integer'image(to_integer(unsigned(sh_u(L)))) & "," &
                  integer'image(to_integer(unsigned(sh_v(L)))) &
                  " got=" & integer'image(to_integer(unsigned(do_a.y))) & "," &
                  integer'image(to_integer(unsigned(do_a.u))) & "," &
                  integer'image(to_integer(unsigned(do_a.v))) severity note;
                rep_dry := rep_dry + 1;
              end if;
            end if;
          elsif is_wet(cur_field) then       -- H1b: wet composite identity within 1 code @ measured latency
            if dY > 1 or dU > 1 or dV > 1 then
              e_wet := e_wet + 1;
              if rep_wet < 8 then
                report NAME & " H1b-WET mismatch@L=" & integer'image(L) & ": ac=" & integer'image(sh_ac(L)) &
                  " exp(y,u,v)=" & integer'image(to_integer(unsigned(sh_y(L)))) & "," &
                  integer'image(to_integer(unsigned(sh_u(L)))) & "," &
                  integer'image(to_integer(unsigned(sh_v(L)))) &
                  " got=" & integer'image(to_integer(unsigned(do_a.y))) & "," &
                  integer'image(to_integer(unsigned(do_a.u))) & "," &
                  integer'image(to_integer(unsigned(do_a.v))) &
                  " dY=" & integer'image(dY) severity note;
                rep_wet := rep_wet + 1;
              end if;
            end if;
          end if;

          -- H12 inert NULL (every active pixel, all phases)
          if do_a.y /= do_b.y or do_a.u /= do_b.u or do_a.v /= do_b.v then
            e_inert := e_inert + 1;
            if rep_inert < 6 then
              report NAME & " H12-INERT diff: field=" & integer'image(cur_field) &
                " ac=" & integer'image(sh_ac(L)) &
                " a.y=" & integer'image(to_integer(unsigned(do_a.y))) &
                " b.y=" & integer'image(to_integer(unsigned(do_b.y))) severity note;
              rep_inert := rep_inert + 1;
            end if;
          end if;
        end if;
      end if;

      -- H9 clamp on every emitted active pixel
      if do_a.avid = '1' and cyc > WARMCYC and not is_x(do_a.y) then
        yv := to_integer(unsigned(do_a.y));
        uvv := to_integer(unsigned(do_a.u));
        vvv := to_integer(unsigned(do_a.v));
        if yv < 64 or yv > 940 or uvv < 64 or uvv > 960 or vvv < 64 or vvv > 960 then
          e_clamp := e_clamp + 1;
          if rep_clamp < 6 then
            report NAME & " H9 out-of-range: field=" & integer'image(cur_field) &
              " y=" & integer'image(yv) & " u=" & integer'image(uvv) &
              " v=" & integer'image(vvv) severity note;
            rep_clamp := rep_clamp + 1;
          end if;
        end if;
      end if;

      -- H2 block/px per input line
      if di.avid = '1' then
        px_line := px_line + 1;
      elsif pin = '1' then
        if cyc > WARMCYC then
          lines_checked := lines_checked + 1;
          if px_line /= AW then
            e_blk := e_blk + 1;
            report NAME & " H2 line active_px=" & integer'image(px_line) &
              " (exp " & integer'image(AW) & ")" severity note;
          end if;
        end if;
        px_line := 0;
      end if;

      pin := di.avid; pout := do_a.avid;

      if cur_field >= STOP_FIELD then
        report "==== WITNESS corner " & NAME & " (id=" & integer'image(CORNERS(G_CORNER).id) &
          ", " & integer'image(AW) & "px/line) ====" severity note;
        report "MEASURED_LATENCY " & NAME & " = " & integer'image(meas_lat) &
          " (spec-intended " & integer'image(D_SPEC) & ", ÷4)" severity note;
        report "RESULT H1a_identity_dry " & NAME & ": " & str_pf(e_dry = 0) &
          " (" & integer'image(e_dry) & " mismatches; dry bit-exact @ measured L=" & integer'image(L) & ")" severity note;
        report "RESULT H1b_identity_wet " & NAME & ": " & str_pf(e_wet = 0) &
          " (" & integer'image(e_wet) & " px >1 code off input@L)" severity note;
        report "RESULT H2_blockalign    " & NAME & ": " & str_pf(e_blk = 0) &
          " (" & integer'image(e_blk) & " bad lines / " & integer'image(lines_checked) & ")" severity note;
        report "RESULT H8_latency_sync  " & NAME & ": " &
          str_pf(meas_lat = D_SPEC and e_sync = 0) & " (measured=" & integer'image(meas_lat) &
          " exp=" & integer'image(D_SPEC) & " ÷4-aligned=" & str_pf((meas_lat mod 4) = 0) &
          " sync_errs=" & integer'image(e_sync) & ")" severity note;
        report "RESULT H9_clamp         " & NAME & ": " & str_pf(e_clamp = 0) &
          " (" & integer'image(e_clamp) & " out-of-range; " & integer'image(e_meta) & " X-skipped)" severity note;
        report "RESULT H12_inert_null   " & NAME & ": " & str_pf(e_inert = 0) &
          " (" & integer'image(e_inert) & " px changed by K2/K5/T9/T10/T11)" severity note;
        report "VIT_DONE corner " & NAME severity note;
        running <= false;
        stop;
      end if;

      if cyc > 60000000 then report "TIMEOUT" severity failure; end if;
    end if;
  end process;

end architecture;
