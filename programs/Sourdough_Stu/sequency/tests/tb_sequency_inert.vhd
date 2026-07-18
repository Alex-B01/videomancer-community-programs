-- ============================================================================
-- tb_sequency_inert.vhd  --  Hook #11: reserved/retired-control inertness guard
-- ----------------------------------------------------------------------------
-- Two DUTs on IDENTICAL 1080p30 timing + identical pixel content; registers
-- differ ONLY in the v1 reserved/retired controls:
--   Knob 5 CROSSTALK (reg0..sweep), Tog 11 Chroma, Tog 7 "Dithered",
--   Tog 8 "Rotate".  Aligned timing => outputs compared live every cycle.
--   Any difference means a reserved/retired path is accidentally half-wired.
-- Baseline B also runs an active effect (QUANTIZE+TILT+REGEN) so the guard is
-- meaningful (the effect must be identical whether reserved bits are 0 or 1).
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
library work;
use work.video_stream_pkg.all;
use work.core_pkg.all;

entity tb_sequency_inert is end entity;

architecture tb of tb_sequency_inert is
  constant CPL : integer := 2200; constant AW : integer := 1920; constant AVS : integer := CPL-AW;
  constant ACT : integer := 12;   constant VBL : integer := 5;   constant LPF : integer := ACT+VBL;
  signal clk : std_logic := '0';
  signal running : boolean := true;
  signal di : t_video_stream_yuv444_30b;
  signal do_a, do_b : t_video_stream_yuv444_30b;
  signal ra, rb : t_spi_ram := (others => (others => '0'));
  signal cur_field : integer := 0;
begin
  clk_gen : process begin
    while running loop clk<='0'; wait for 5 ns; clk<='1'; wait for 5 ns; end loop; wait;
  end process;

  -- Common ACTIVE effect on both; A leaves reserved=0, B drives them all hot.
  gen_regs : process(clk)
  begin
    if rising_edge(clk) then
      ra <= (others => (others=>'0')); rb <= (others => (others=>'0'));
      ra(0) <= std_logic_vector(to_unsigned(600,10));  -- QUANTIZE
      ra(1) <= std_logic_vector(to_unsigned(700,10));  -- TILT
      ra(3) <= std_logic_vector(to_unsigned(300,10));  -- REGEN
      ra(7) <= std_logic_vector(to_unsigned(1023,10)); -- fader wet
      ra(8) <= std_logic_vector(to_unsigned(7,10));    -- 1080p30 id
      rb <= ra;                                        -- copy active effect...
      rb(4) <= std_logic_vector(to_unsigned(1023,10)); -- Knob5 CROSSTALK hot (reserved)
      rb(6) <= std_logic_vector(to_unsigned(2#10011#,10)); -- tog7 Dithered + tog8 Rotate + tog11 Chroma
    end if;
  end process;

  dut_a : entity work.program_top(sequency) port map(clk=>clk, registers_in=>ra, data_in=>di, data_out=>do_a);
  dut_b : entity work.program_top(sequency) port map(clk=>clk, registers_in=>rb, data_in=>di, data_out=>do_b);

  stim : process(clk)
    variable cx,ly,fld,gl : integer := 0; variable avon : boolean; variable ac : integer;
  begin
    if rising_edge(clk) then
      avon := (cx>=AVS) and (ly<ACT); ac := cx-AVS;
      di.hsync_n<='1'; di.vsync_n<='1'; di.field_n<='1';
      if cx<44 then di.hsync_n<='0'; end if;
      if avon then
        di.avid<='1';
        di.y <= std_logic_vector(to_unsigned(64 + ((ac*9+gl*7) mod 800),10));
        di.u <= std_logic_vector(to_unsigned(64 + ((ac*3+gl*11) mod 850),10));
        di.v <= std_logic_vector(to_unsigned(64 + ((ac*5+gl) mod 850),10));
      else
        di.avid<='0'; di.y<=std_logic_vector(to_unsigned(64,10));
        di.u<=std_logic_vector(to_unsigned(512,10)); di.v<=std_logic_vector(to_unsigned(512,10));
      end if;
      if cx=CPL-1 then cx:=0; if avon then gl:=gl+1; end if;
        if ly=LPF-1 then ly:=0; fld:=fld+1; else ly:=ly+1; end if;
      else cx:=cx+1; end if;
      cur_field <= fld;
    end if;
  end process;

  mon : process(clk)
    variable cyc : integer := 0; variable e : integer := 0; variable rep : integer := 0;
  begin
    if rising_edge(clk) then
      cyc := cyc + 1;
      if cyc>300 then
        if (do_a.y/=do_b.y) or (do_a.u/=do_b.u) or (do_a.v/=do_b.v) or
           (do_a.avid/=do_b.avid) then
          e := e + 1;
          if rep<6 then
            report "HOOK11 reserved-ctrl CHANGED output: a.y=" &
              integer'image(to_integer(unsigned(do_a.y))) & " b.y=" &
              integer'image(to_integer(unsigned(do_b.y))) severity note;
            rep := rep+1;
          end if;
        end if;
      end if;
      if cur_field>=4 then
        report "RESULT HOOK11_inert 1080p30: " &
          work.video_corner_pkg.str_pf(e=0) & " (" & integer'image(e) & " diffs)" severity note;
        report "VIT_DONE inert" severity note;
        running<=false; stop;
      end if;
      if cyc>20000000 then report "TIMEOUT" severity failure; end if;
    end if;
  end process;
end architecture;
