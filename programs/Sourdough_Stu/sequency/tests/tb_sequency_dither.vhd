-- ============================================================================
-- tb_sequency_dither.vhd  --  Hook #14 (NEW, REV-F): Dithered QUANTIZE (Tog 7)
--   -- deadzone-edge sparkle with the DC coefficient preserved.
-- ----------------------------------------------------------------------------
-- TWO aligned DUTs, identical input, identical knobs EXCEPT Tog 7:
--   hard : Tog7 = 0 (Hard)     -> deterministic keep/kill at the plain threshold
--   dith : Tog7 = 1 (Dithered) -> threshold jitters +/- a band-scaled LFSR sample
-- Both DUTs' dither LFSRs free-run from the same all-1s seed in lockstep, so the
-- dithered decisions are deterministic given pixel index -> the Hard-vs-Dithered
-- diff is a clean, reproducible measure of the deadzone-edge sparkle.
--
-- The QUANTIZE deadzone is DC-EXEMPT (true seq 0 always kept; and DC = 16*mean >=
-- 1024 > any threshold <=1023 for legal video, so DC survives regardless) and the
-- dither only subtracts from the AC band thresholds, so:
--   * a flat / DC-only field is BIT-IDENTICAL Hard vs Dithered (AC coeffs ~0 -> no
--     sparkle; the block DC / brightness rides through untouched in both modes), and
--   * on AC content the per-16-block DC (block sum) is preserved (AC basis funcs
--     sum to 0 over a block; the DC coeff is identical) while individual pixels
--     sparkle where a coefficient straddles the jittered threshold.
-- (NB: a DC-only field is NOT expected to round-trip bit-exact to INPUT at
-- QUANTIZE>0 -- the streaming R2SDF's per-line pipeline fill perturbs the first
-- block of each line; forward.inverse is a clean identity only at QUANTIZE=0. That
-- edge effect is unrelated to dither, so #14c asserts Hard==Dithered, not ==input.)
--
-- Three content/quantize phases (static Tog7 per DUT -> no register transient; the
-- Hard-vs-Dithered compare is valid every cycle since both DUTs see identical
-- input+registers, so content/QUANTIZE changes never desync the pair):
--   AC   (QUANT=Q_ON, AC-rich)  : #14a sparkle>0 ; #14b block-DC preserved (|dsum|<=TOL)
--   FLAT (QUANT=Q_ON, DC-only)  : #14c hard==dith bit-exact (brightness untouched by dither)
--   Q0   (QUANT=0,    AC-rich)  : #14d hard==dith bit-exact (dither gated off, no deadzone)
--
-- NEVER edits program VHDL. Pure black-box.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
library work;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.video_corner_pkg.all;

entity tb_sequency_dither is
  generic (
    G_CORNER    : integer := 2;
    G_ACT_LINES : integer := 6;
    G_VBL_LINES : integer := 3
  );
end entity;

architecture tb of tb_sequency_dither is
  constant CPL : integer := CORNERS(G_CORNER).clocks_per_line;
  constant AW  : integer := CORNERS(G_CORNER).active_width;
  constant AVS : integer := CPL - AW;
  constant LPF : integer := G_ACT_LINES + G_VBL_LINES;
  constant ILV : boolean := CORNERS(G_CORNER).is_interlaced;
  constant D   : integer := 56;
  constant TID : integer := CORNERS(G_CORNER).id;
  constant Q_ON : integer := 400;               -- deadzone that AC coeffs straddle
  constant TOL  : integer := 64;                -- block-DC reconstruction-rounding band:
                                                -- << block sum ~8000, so DC IS preserved
                                                -- (killing DC would swing it by 100s)
  constant STOPF   : integer := 5;
  constant WARMCYC : integer := 2*CPL;
  -- phase(field): 0 warm .1-2 AC .3 FLAT .4 Q0
  signal clk : std_logic := '0'; signal running : boolean := true;
  signal di : t_video_stream_yuv444_30b;
  signal do_h, do_d : t_video_stream_yuv444_30b;
  signal r_h, r_d : t_spi_ram := (others => (others => '0'));
  signal cur_field : integer := 0;
  signal cur_phase : integer := 0;   -- 0 warm/AC .1 AC .2 FLAT .3 Q0

  type v10 is array(1 to D) of std_logic_vector(9 downto 0);
  type sl  is array(1 to D) of std_logic;
  type ip  is array(1 to D) of integer;
  signal sh_y  : v10 := (others => (others => '0'));
  signal sh_av : sl  := (others => '0');
  signal sh_ph : ip  := (others => 0);

  function phase_of(f : integer) return integer is
  begin
    if    f <= 2 then return 1;   -- warm + AC
    elsif f  = 3 then return 2;   -- FLAT
    else             return 3;    -- Q0
    end if;
  end function;
begin
  clk_gen : process begin
    while running loop clk<='0'; wait for 5 ns; clk<='1'; wait for 5 ns; end loop; wait;
  end process;

  -- Static per-DUT toggle; QUANTIZE follows the phase (both DUTs identically).
  p_regs : process(clk)
    variable q : integer;
  begin
    if rising_edge(clk) then
      if phase_of(cur_field) = 3 then q := 0; else q := Q_ON; end if;
      r_h <= (others => (others=>'0'));
      r_h(0) <= std_logic_vector(to_unsigned(q,10));      -- QUANTIZE
      r_h(1) <= std_logic_vector(to_unsigned(512,10));    -- TILT flat
      r_h(7) <= std_logic_vector(to_unsigned(1023,10));   -- fader wet
      r_h(8) <= std_logic_vector(to_unsigned(TID,10));
      r_d <= r_h;
      r_d(6) <= std_logic_vector(to_unsigned(1,10));      -- Tog7 Dithered (bit0)
      cur_phase <= phase_of(cur_field);
    end if;
  end process;

  dut_h : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_h, data_in=>di, data_out=>do_h);
  dut_d : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_d, data_in=>di, data_out=>do_d);

  stim : process(clk)
    variable cx,ly,fld,gl : integer := 0; variable avon : boolean; variable ac,blk,yy : integer;
  begin
    if rising_edge(clk) then
      avon := (cx>=AVS) and (ly<G_ACT_LINES); ac := cx-AVS;
      di.hsync_n<='1'; di.vsync_n<='1'; di.field_n<= '1' when (fld mod 2)=0 else '0';
      if cx<44 then di.hsync_n<='0'; end if;
      if (ly>=G_ACT_LINES) and (ly<G_ACT_LINES+2) then di.vsync_n<='0'; end if;
      if avon then di.avid<='1';
        if phase_of(fld) = 2 then                       -- FLAT: horizontally-flat line ->
          blk := 0;                                     -- EVERY horizontal 16-window is
          yy  := 400 + ((gl*53) mod 400);               -- DC-only (level varies per line
                                                        -- only; WHT is horizontal, REGEN off)
                                                        -- so it round-trips exact at any phase
        else                                            -- AC-rich (straddles deadzone)
          yy  := 400 + ((ac*29 + gl*17 + ac*ac*3) mod 240);
        end if;
        di.y <= std_logic_vector(to_unsigned(yy,10));
        di.u <= std_logic_vector(to_unsigned(512,10));  -- U/V passthrough (Crosstalk off)
        di.v <= std_logic_vector(to_unsigned(512,10));
      else di.avid<='0'; di.y<=std_logic_vector(to_unsigned(64,10));
        di.u<=std_logic_vector(to_unsigned(512,10)); di.v<=std_logic_vector(to_unsigned(512,10)); end if;
      if cx=CPL-1 then cx:=0; if avon then gl:=gl+1; end if;
        if ly=LPF-1 then ly:=0; fld:=fld+1; else ly:=ly+1; end if;
      else cx:=cx+1; end if;
      cur_field <= fld;
    end if;
  end process;

  shadow : process(clk) begin
    if rising_edge(clk) then
      sh_y(1)<=di.y; sh_av(1)<=di.avid; sh_ph(1)<=cur_phase;
      for k in 2 to D loop sh_y(k)<=sh_y(k-1); sh_av(k)<=sh_av(k-1); sh_ph(k)<=sh_ph(k-1); end loop;
    end if;
  end process;

  mon : process(clk)
    constant NAME : string := corner_name(G_CORNER);
    variable cyc : integer := 0;
    variable pav : std_logic := '0';
    variable armed : boolean := false;      -- true from the first clean avid-rising
    variable ocol : integer := 0;           -- output active column (mod-16 block clock)
    variable sh_blk, sd_blk : integer := 0; -- per-block Y sums (hard, dith)
    variable n_sparkle, e_dcsum, maxdsum, n_blk : integer := 0;
    variable e_flat, e_q0, n_flat, n_q0 : integer := 0;
    variable hy, dy, ph : integer := 0;
  begin
    if rising_edge(clk) then
      cyc := cyc + 1;
      if cyc>WARMCYC and do_h.avid='1' and pav='0' then    -- avid-rising: arm + block anchor
        armed := true; ocol := 0; sh_blk := 0; sd_blk := 0;
      end if;
      if armed and do_h.avid='1' and sh_av(D)='1' then
        hy := to_integer(unsigned(do_h.y));
        dy := to_integer(unsigned(do_d.y));
        ph := sh_ph(D);                       -- phase of THIS output pixel (delay-aligned)

        if ph = 1 then                        -- AC phase: sparkle + block-DC
          if hy /= dy then n_sparkle := n_sparkle + 1; end if;
          sh_blk := sh_blk + hy; sd_blk := sd_blk + dy;
          ocol := ocol + 1;
          if (ocol mod 16) = 0 then
            n_blk := n_blk + 1;
            if abs(sh_blk - sd_blk) > maxdsum then maxdsum := abs(sh_blk - sd_blk); end if;
            if abs(sh_blk - sd_blk) > TOL then e_dcsum := e_dcsum + 1; end if;
            sh_blk := 0; sd_blk := 0;
          end if;
        else                                  -- non-AC pixel: discard any partial AC block
          sh_blk := 0; sd_blk := 0;
          if ph = 2 then                      -- FLAT (DC-only field): brightness untouched
            -- #14c: a flat/DC-only field is BIT-IDENTICAL Hard vs Dithered -> the
            -- dither perturbs only AC thresholds; the block DC/brightness rides
            -- through unchanged. (DC = 16*mean >= 1024 > any threshold <=1023, so DC
            -- is always kept; the dither cannot move it. The AC coeffs are ~0 so no
            -- sparkle -> the flat field renders identically in both modes.)
            n_flat := n_flat + 1;
            if hy /= dy then e_flat := e_flat + 1; end if;
          else                                -- Q0: dither gated off
            n_q0 := n_q0 + 1;
            if hy /= dy then e_q0 := e_q0 + 1; end if;
          end if;
        end if;
      end if;
      pav := do_h.avid;

      if cur_field>=STOPF then
        report "==== SEQUENCY dither "&NAME&" (id="&integer'image(TID)&") ====" severity note;
        report "RESULT HOOK14a_sparkle     "&NAME&": "&str_pf(n_sparkle>0)&
          " ("&integer'image(n_sparkle)&" Hard!=Dithered px at deadzone edge)" severity note;
        report "RESULT HOOK14b_block_dc     "&NAME&": "&str_pf(e_dcsum=0 and n_blk>0)&
          " ("&integer'image(e_dcsum)&"/"&integer'image(n_blk)&" blocks |dsum|>"&integer'image(TOL)&
          "; max|dsum|="&integer'image(maxdsum)&")" severity note;
        report "RESULT HOOK14c_flat_dc_kept  "&NAME&": "&str_pf(e_flat=0 and n_flat>0)&
          " ("&integer'image(e_flat)&" Hard!=Dithered on flat/DC-only field / "&
          integer'image(n_flat)&" px; brightness untouched by dither)" severity note;
        report "RESULT HOOK14d_q0_null      "&NAME&": "&str_pf(e_q0=0 and n_q0>0)&
          " ("&integer'image(e_q0)&" Hard!=Dithered w/ QUANTIZE=0 / "&integer'image(n_q0)&" px)" severity note;
        report "VIT_DONE dither "&NAME severity note;
        running<=false; stop;
      end if;
      if cyc>40000000 then report "TIMEOUT" severity failure; end if;
    end if;
  end process;
end architecture;
