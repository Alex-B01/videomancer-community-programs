-- ============================================================================
-- tb_witness_gaze_top.vhd  --  TOP-LEVEL integrated gaze-update -> IRIS render
--   (the coverage GAP: tb_witness_corners never asserted a gaze-anchored vignette;
--    the gaze/scan was only ever driven directly in isolation, never through
--    program_top's own vsync -> s_frame_end -> moments-scan -> gaze -> coord -> iris).
-- ----------------------------------------------------------------------------
-- Drives program_top(witness) end-to-end with a localized bright BLOB in the
-- LOWER-RIGHT and asks: does the IRIS vignette track the blob (gaze UPDATED) or
-- stay at the reset corner (0,0) (gaze FROZEN, the hardware symptom)?  Two DUTs
-- share the stimulus; only K1 differs (tight vs open) so (off-on) = pure IRIS
-- darkening at each pixel, content-independent:
--   gaze tracks blob  -> darkening at the BLOB ~ 0, at the CORNER(0,0) large.
--   gaze frozen (0,0) -> darkening at the CORNER ~ 0, at the BLOB large  => FAIL.
-- Whichever way this lands tells triage whether the top-level scan works IN SIM
-- (=> hardware-only trigger: vsync/vblank/power-on) or is broken in sim too.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
library work;
use work.video_stream_pkg.all;
use work.core_pkg.all;

entity tb_witness_gaze_top is
  generic ( G_ACT_LINES : integer := 32; G_VBL_LINES : integer := 6 );
end entity;

architecture tb of tb_witness_gaze_top is
  constant CPL : integer := 2200; constant AW : integer := 1920; constant AVS : integer := CPL-AW;
  constant LPF : integer := G_ACT_LINES + G_VBL_LINES;
  -- blob = lower-right rectangle ; corner probe = upper-left (~block 0,0)
  constant BR0 : integer := 24; constant BR1 : integer := 30;  constant BC0 : integer := 1300; constant BC1 : integer := 1850;
  constant KR0 : integer := 1;  constant KR1 : integer := 5;   constant KC0 : integer := 60;   constant KC1 : integer := 520;
  constant BG  : integer := 0; constant BLOB : integer := 920;   -- dark bg (zero mass) => gaze must track TO the off-center blob
  constant STOP_FR : integer := 12; constant SETTLE_FR : integer := 10; constant WARM : integer := 2*CPL;
  signal clk : std_logic := '0'; signal running : boolean := true;
  signal di : t_video_stream_yuv444_30b; signal do_on, do_off : t_video_stream_yuv444_30b;
  signal r_on, r_off : t_spi_ram := (others => (others => '0'));
  signal frame_no : integer := 0;
  signal bs_on,bc_on,ks_on,kc_on,bs_off,ks_off : integer := 0;
  function str_pf(b : boolean) return string is begin if b then return "PASS"; else return "FAIL"; end if; end function;
begin
  clk <= not clk after 5 ns when running else '0';
  r_on(0)<=std_logic_vector(to_unsigned(60,10));  r_on(3)<=std_logic_vector(to_unsigned(512,10));
  r_on(5)<=std_logic_vector(to_unsigned(0,10));    r_on(7)<=std_logic_vector(to_unsigned(1023,10)); r_on(8)<=std_logic_vector(to_unsigned(7,10));
  r_off(0)<=std_logic_vector(to_unsigned(1023,10)); r_off(3)<=std_logic_vector(to_unsigned(512,10));
  r_off(5)<=std_logic_vector(to_unsigned(0,10));   r_off(7)<=std_logic_vector(to_unsigned(1023,10)); r_off(8)<=std_logic_vector(to_unsigned(7,10));
  dut_on  : entity work.program_top(witness) port map(clk=>clk, registers_in=>r_on,  data_in=>di, data_out=>do_on);
  dut_off : entity work.program_top(witness) port map(clk=>clk, registers_in=>r_off, data_in=>di, data_out=>do_off);

  p_gen : process(clk)
    variable cx,ly,fld : integer := 0; variable act : boolean; variable ac,yv : integer;
  begin
    if rising_edge(clk) then
      act := (cx>=AVS) and (ly<G_ACT_LINES); ac := cx-AVS;
      di.hsync_n<='1'; di.vsync_n<='1'; di.field_n<='1';
      if cx<44 then di.hsync_n<='0'; end if;
      if (ly>=G_ACT_LINES) and (ly<G_ACT_LINES+2) then di.vsync_n<='0'; end if;
      if act then di.avid<='1';
        if (ly>=BR0 and ly<=BR1 and ac>=BC0 and ac<=BC1) then yv:=BLOB; else yv:=BG; end if;
        di.y<=std_logic_vector(to_unsigned(yv,10)); di.u<=std_logic_vector(to_unsigned(512,10)); di.v<=std_logic_vector(to_unsigned(512,10));
      else di.avid<='0'; di.y<=std_logic_vector(to_unsigned(64,10)); di.u<=std_logic_vector(to_unsigned(512,10)); di.v<=std_logic_vector(to_unsigned(512,10)); end if;
      frame_no<=fld;
      if cx=CPL-1 then cx:=0; if ly=LPF-1 then ly:=0; fld:=fld+1; else ly:=ly+1; end if; else cx:=cx+1; end if;
    end if;
  end process;

  p_meas : process(clk)
    variable pav : std_logic := '0'; variable oline,ocol : integer := 0; variable yon,yoff : integer;
  begin
    if rising_edge(clk) then
      if do_on.avid='1' and pav='0' then oline:=oline+1; ocol:=0; end if;
      if do_on.avid='1' then
        if frame_no=SETTLE_FR and not is_x(do_on.y) and not is_x(do_off.y) then
          yon:=to_integer(unsigned(do_on.y)); yoff:=to_integer(unsigned(do_off.y));
          if oline>=BR0 and oline<=BR1 and ocol>=BC0 and ocol<=BC1 then bs_on<=bs_on+yon; bc_on<=bc_on+1; bs_off<=bs_off+yoff;
          elsif oline>=KR0 and oline<=KR1 and ocol>=KC0 and ocol<=KC1 then ks_on<=ks_on+yon; kc_on<=kc_on+1; ks_off<=ks_off+yoff; end if;
        end if;
        ocol:=ocol+1;
      end if;
      if do_on.vsync_n='0' and frame_no/=SETTLE_FR then oline:=0; end if;
      pav:=do_on.avid;
    end if;
  end process;

  eval : process
    variable b_on,b_off,k_on,k_off,bd,kd : integer;
  begin
    wait until frame_no>=STOP_FR; wait for 1 ns;
    b_on:=bs_on/maximum(bc_on,1); b_off:=bs_off/maximum(bc_on,1);
    k_on:=ks_on/maximum(kc_on,1); k_off:=ks_off/maximum(kc_on,1);
    bd:=b_off-b_on; kd:=k_off-k_on;   -- IRIS darkening at BLOB (gaze) vs CORNER(0,0)
    report "GAZETOP blob(on/off)=" & integer'image(b_on) & "/" & integer'image(b_off) & " dark=" & integer'image(bd) &
      " | corner(on/off)=" & integer'image(k_on) & "/" & integer'image(k_off) & " dark=" & integer'image(kd) severity note;
    report "GT_INFO iris darkening (needs a bright region OFF the gaze to be large): max=" &
      integer'image(maximum(bd,kd)) & " codes" severity note;
    -- dark bg => blob is the ONLY mass => a healthy gaze tracks ONTO it => the blob
    -- sits at r^2~0 => barely darkened (blob_dark small). A gaze FROZEN at reset (0,0)
    -- leaves the off-center blob far from the gaze => heavily darkened (blob_dark large).
    report "RESULT GT2_gaze_tracks_blob: " & str_pf(bd < 150) &
      " (blob_dark " & integer'image(bd) & " <150 => gaze tracked onto the blob; " &
      "large => gaze FROZEN away from the blob / stuck at reset corner)" severity note;
    report "VIT_DONE gazetop" severity note;
    running<=false; stop;
  end process;
end architecture;
