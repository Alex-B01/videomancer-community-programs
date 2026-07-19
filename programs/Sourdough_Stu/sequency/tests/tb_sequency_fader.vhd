-- ============================================================================
-- tb_sequency_fader.vhd  --  Hook #6: dry/wet fader endpoints
-- ----------------------------------------------------------------------------
-- 1080p30, neutral knobs, legal content.  Two DUTs:
--   dry : fader=0    -> out must equal in delayed 56 (dry path bypasses organ)
--   wet : fader=1023 -> out must equal in delayed 56 (organ round-trip)
-- The dry endpoint isolates the fader + dry delay from the WHT organ: dry PASS
-- + wet FAIL localizes the identity defect to the organ reconstruction path.
-- Latency 56 (REV-F: +4 Y->C leak retime after the fader tail). Y content kept in
-- [64,940] so the interpolator output clamp is a no-op on both endpoints. Crosstalk
-- is left at Knob5=0/Tog11 off (leak master OFF) so the dry-Y here IS out.y.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
library work;
use work.video_stream_pkg.all;
use work.core_pkg.all;

entity tb_sequency_fader is end entity;

architecture tb of tb_sequency_fader is
  constant CPL : integer := 2200; constant AW : integer := 1920; constant AVS : integer := CPL-AW;
  constant NL  : integer := 10;   constant VBL : integer := 5;   constant LPF : integer := NL+VBL;
  constant D   : integer := 56;   -- REV-F latency (+4 leak retime)
  signal clk : std_logic := '0'; signal running : boolean := true;
  signal di : t_video_stream_yuv444_30b;
  signal do_dry, do_wet : t_video_stream_yuv444_30b;
  signal r_dry, r_wet : t_spi_ram := (others => (others => '0'));
  signal cur_field : integer := 0;
  type v10 is array(1 to D) of std_logic_vector(9 downto 0);
  type sl  is array(1 to D) of std_logic;
  signal sh_y : v10 := (others=>(others=>'0'));  signal sh_av : sl := (others=>'0');
begin
  clk_gen : process begin
    while running loop clk<='0'; wait for 5 ns; clk<='1'; wait for 5 ns; end loop; wait;
  end process;

  r_dry(1) <= std_logic_vector(to_unsigned(512,10)); r_dry(8) <= std_logic_vector(to_unsigned(7,10));
  r_dry(7) <= std_logic_vector(to_unsigned(0,10));      -- fader DRY
  r_wet(1) <= std_logic_vector(to_unsigned(512,10)); r_wet(8) <= std_logic_vector(to_unsigned(7,10));
  r_wet(7) <= std_logic_vector(to_unsigned(1023,10));   -- fader WET

  dut_dry : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_dry, data_in=>di, data_out=>do_dry);
  dut_wet : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_wet, data_in=>di, data_out=>do_wet);

  stim : process(clk)
    variable cx,ly,fld,gl : integer := 0; variable avon : boolean; variable ac : integer;
  begin
    if rising_edge(clk) then
      avon := (cx>=AVS) and (ly<NL); ac := cx-AVS;
      di.hsync_n<='1'; di.vsync_n<='1'; di.field_n<='1';
      if avon then di.avid<='1';
        di.y <= std_logic_vector(to_unsigned(64 + ((ac*7+gl*13) mod 800),10));
      else di.avid<='0'; di.y<=std_logic_vector(to_unsigned(64,10)); end if;
      di.u<=std_logic_vector(to_unsigned(512,10)); di.v<=std_logic_vector(to_unsigned(512,10));
      if cx=CPL-1 then cx:=0; if avon then gl:=gl+1; end if;
        if ly=LPF-1 then ly:=0; fld:=fld+1; else ly:=ly+1; end if;
      else cx:=cx+1; end if;
      cur_field <= fld;
    end if;
  end process;

  shadow : process(clk) begin
    if rising_edge(clk) then
      sh_y(1)<=di.y; sh_av(1)<=di.avid;
      for k in 2 to D loop sh_y(k)<=sh_y(k-1); sh_av(k)<=sh_av(k-1); end loop;
    end if;
  end process;

  mon : process(clk)
    variable cyc, e_dry, e_wet : integer := 0;
  begin
    if rising_edge(clk) then
      cyc := cyc+1;
      if cyc>200 and sh_av(D)='1' then
        if do_dry.y /= sh_y(D) then e_dry := e_dry+1; end if;
        if do_wet.y /= sh_y(D) then e_wet := e_wet+1; end if;
      end if;
      if cur_field>=3 then
        report "RESULT HOOK6_fader_dry  1080p30 (fader=0):    " &
          work.video_corner_pkg.str_pf(e_dry=0) & " (" & integer'image(e_dry) & " mismatches)" severity note;
        report "RESULT HOOK6_fader_wet  1080p30 (fader=1023): " &
          work.video_corner_pkg.str_pf(e_wet=0) & " (" & integer'image(e_wet) &
          " mismatches -- organ path)" severity note;
        report "VIT_DONE fader" severity note;
        running<=false; stop;
      end if;
      if cyc>20000000 then report "TIMEOUT" severity failure; end if;
    end if;
  end process;
end architecture;
