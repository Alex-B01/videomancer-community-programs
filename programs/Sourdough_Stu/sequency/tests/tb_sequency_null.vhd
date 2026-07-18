-- ============================================================================
-- tb_sequency_null.vhd  --  Hook #8: no-temporal-state rate-class NULL check
-- ----------------------------------------------------------------------------
-- Same pixel CONTENT + same active-effect knobs driven at two rate classes
-- (1080p30 vs 1080p24; TRUE htotal 2200 vs 2750, different blanking).  Capture
-- one warmed field of active output per DUT in raster order; assert byte-
-- identical (allow <=1 top-of-frame line stale-coeff tolerance).  A *null* is
-- the correct result: output must depend only on current-frame content +
-- intra-frame feedback, never on frame rate / blanking geometry.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
library work;
use work.video_stream_pkg.all;
use work.core_pkg.all;

entity tb_sequency_null is end entity;

architecture tb of tb_sequency_null is
  constant AW  : integer := 1920;
  constant ACT : integer := 6;    constant VBL : integer := 4;  constant LPF : integer := ACT+VBL;
  constant SKIP : integer := 2*ACT*AW;   -- skip 2 fields (warm pipeline + REGEN)
  constant CAP  : integer := ACT*AW;     -- capture 1 field
  type cap_arr is array(0 to CAP-1) of integer;
  signal clk : std_logic := '0';
  signal running : boolean := true;
  signal di_a, di_b : t_video_stream_yuv444_30b;
  signal do_a, do_b : t_video_stream_yuv444_30b;
  signal regs : t_spi_ram := (others => (others => '0'));
  signal capA, capB : cap_arr := (others => -1);
  signal doneA, doneB : boolean := false;

  -- shared content: identical function of (global active line, active col)
  function yv(gl,ac : integer) return integer is begin return 64 + ((ac*9 + gl*7)  mod 800); end;
  function uv(gl,ac : integer) return integer is begin return 64 + ((ac*3 + gl*11) mod 850); end;
  function vv(gl,ac : integer) return integer is begin return 64 + ((ac*5 + gl)    mod 850); end;
  function pack(y,u,v : integer) return integer is begin return (y*1024 + u)*1024 + v; end;
begin
  clk_gen : process begin
    while running loop clk<='0'; wait for 5 ns; clk<='1'; wait for 5 ns; end loop; wait;
  end process;

  -- identical ACTIVE effect for both DUTs; only registers_in(8) timing id differs
  regs(0) <= std_logic_vector(to_unsigned(400,10));   -- QUANTIZE
  regs(1) <= std_logic_vector(to_unsigned(512,10));   -- TILT flat
  regs(3) <= std_logic_vector(to_unsigned(320,10));   -- REGEN (intra-frame vertical)
  regs(7) <= std_logic_vector(to_unsigned(1023,10));  -- fader wet

  dut_a : entity work.program_top(sequency) port map(clk=>clk, registers_in=>regs, data_in=>di_a, data_out=>do_a);
  dut_b : entity work.program_top(sequency) port map(clk=>clk, registers_in=>regs, data_in=>di_b, data_out=>do_b);

  -- timing gen A : 1080p30 (cpl 2200)
  gen_a : process(clk)
    variable cx,ly,gl : integer := 0; variable avon : boolean; variable ac : integer;
    constant CPL : integer := 2200; constant AVS : integer := CPL-AW;
  begin
    if rising_edge(clk) then
      avon := (cx>=AVS) and (ly<ACT); ac := cx-AVS;
      di_a.hsync_n<='1'; di_a.vsync_n<='1'; di_a.field_n<='1';
      if avon then di_a.avid<='1';
        di_a.y<=std_logic_vector(to_unsigned(yv(gl,ac),10));
        di_a.u<=std_logic_vector(to_unsigned(uv(gl,ac),10));
        di_a.v<=std_logic_vector(to_unsigned(vv(gl,ac),10));
      else di_a.avid<='0'; di_a.y<=std_logic_vector(to_unsigned(64,10));
        di_a.u<=std_logic_vector(to_unsigned(512,10)); di_a.v<=std_logic_vector(to_unsigned(512,10)); end if;
      if cx=CPL-1 then cx:=0; if avon then gl:=gl+1; end if;
        if ly=LPF-1 then ly:=0; else ly:=ly+1; end if;
      else cx:=cx+1; end if;
    end if;
  end process;

  -- timing gen B : 1080p24 (cpl 2750)
  gen_b : process(clk)
    variable cx,ly,gl : integer := 0; variable avon : boolean; variable ac : integer;
    constant CPL : integer := 2750; constant AVS : integer := CPL-AW;
  begin
    if rising_edge(clk) then
      avon := (cx>=AVS) and (ly<ACT); ac := cx-AVS;
      di_b.hsync_n<='1'; di_b.vsync_n<='1'; di_b.field_n<='1';
      if avon then di_b.avid<='1';
        di_b.y<=std_logic_vector(to_unsigned(yv(gl,ac),10));
        di_b.u<=std_logic_vector(to_unsigned(uv(gl,ac),10));
        di_b.v<=std_logic_vector(to_unsigned(vv(gl,ac),10));
      else di_b.avid<='0'; di_b.y<=std_logic_vector(to_unsigned(64,10));
        di_b.u<=std_logic_vector(to_unsigned(512,10)); di_b.v<=std_logic_vector(to_unsigned(512,10)); end if;
      if cx=CPL-1 then cx:=0; if avon then gl:=gl+1; end if;
        if ly=LPF-1 then ly:=0; else ly:=ly+1; end if;
      else cx:=cx+1; end if;
    end if;
  end process;

  -- capture A: active-output pixels [SKIP, SKIP+CAP)
  cap_a : process(clk) variable oc : integer := 0; begin
    if rising_edge(clk) then
      if do_a.avid='1' then
        if oc>=SKIP and oc<SKIP+CAP then
          capA(oc-SKIP) <= pack(to_integer(unsigned(do_a.y)),to_integer(unsigned(do_a.u)),to_integer(unsigned(do_a.v)));
        end if;
        if oc = SKIP+CAP then doneA<=true; end if;
        oc := oc + 1;
      end if;
    end if;
  end process;
  cap_b : process(clk) variable oc : integer := 0; begin
    if rising_edge(clk) then
      if do_b.avid='1' then
        if oc>=SKIP and oc<SKIP+CAP then
          capB(oc-SKIP) <= pack(to_integer(unsigned(do_b.y)),to_integer(unsigned(do_b.u)),to_integer(unsigned(do_b.v)));
        end if;
        if oc = SKIP+CAP then doneB<=true; end if;
        oc := oc + 1;
      end if;
    end if;
  end process;

  fin : process(clk)
    variable e, e_body, rep : integer := 0;
  begin
    if rising_edge(clk) then
      if doneA and doneB then
        for i in 0 to CAP-1 loop
          if capA(i) /= capB(i) then
            e := e + 1;
            if i >= AW then e_body := e_body + 1; end if;   -- exclude top-of-frame line
            if rep<6 then
              report "HOOK8 rate-class diff at px " & integer'image(i) &
                " a=" & integer'image(capA(i)) & " b=" & integer'image(capB(i)) severity note;
              rep := rep + 1;
            end if;
          end if;
        end loop;
        report "RESULT HOOK8_rateclass_null (1080p30 vs 1080p24): " &
          work.video_corner_pkg.str_pf(e_body=0) & " (" & integer'image(e) &
          " total diffs, " & integer'image(e_body) & " outside top-of-frame line)" severity note;
        report "VIT_DONE null" severity note;
        running<=false; stop;
      end if;
    end if;
  end process;
end architecture;
