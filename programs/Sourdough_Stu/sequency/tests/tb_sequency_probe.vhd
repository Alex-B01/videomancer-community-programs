library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
library work;
use work.video_stream_pkg.all;
use work.core_pkg.all;

entity tb_sequency_probe is end entity;

architecture tb of tb_sequency_probe is
  constant CPL : integer := 160;   -- clocks/line (hblank 96 >> 28)
  constant AVS : integer := 96;    -- avid start
  constant AW  : integer := 64;    -- 4 blocks
  signal clk : std_logic := '0';
  signal running : boolean := true;
  signal data_in, data_out : t_video_stream_yuv444_30b;
  signal regs : t_spi_ram := (others => (others => '0'));
  -- test Y per line index
  type yv_arr is array(0 to 9) of integer;
  constant YTEST : yv_arr := (100, 300, 500, 511, 512, 513, 600, 800, 1023, 64);
begin
  clk_gen : process begin
    while running loop clk <= '0'; wait for 5 ns; clk <= '1'; wait for 5 ns; end loop; wait;
  end process;

  -- neutral knobs, fader wet
  regs(1) <= std_logic_vector(to_unsigned(512,10));   -- TILT flat
  regs(7) <= std_logic_vector(to_unsigned(1023,10));  -- fader WET

  dut : entity work.program_top(sequency)
    port map(clk=>clk, registers_in=>regs, data_in=>data_in, data_out=>data_out);

  stim : process(clk)
    variable cx : integer := 0;
    variable ln : integer := 0;
    variable yv : integer;
  begin
    if rising_edge(clk) then
      data_in.hsync_n <= '1'; data_in.vsync_n <= '1'; data_in.field_n <= '1';
      if (cx >= AVS) and (ln < 10) then
        data_in.avid <= '1';
        yv := YTEST(ln);
        data_in.y <= std_logic_vector(to_unsigned(yv,10));
      else
        data_in.avid <= '0';
        data_in.y <= std_logic_vector(to_unsigned(64,10));
      end if;
      data_in.u <= std_logic_vector(to_unsigned(512,10));
      data_in.v <= std_logic_vector(to_unsigned(512,10));
      if cx = CPL-1 then cx := 0; ln := ln + 1; else cx := cx + 1; end if;
      if ln = 14 then running <= false; stop; end if;
    end if;
  end process;

  -- capture: print first 18 active output pixels per output line
  mon : process(clk)
    variable pav : std_logic := '0';
    variable n : integer := 0;
    variable outln : integer := 0;
    variable s : string(1 to 200);
    variable p : integer;
  begin
    if rising_edge(clk) then
      if data_out.avid='1' then
        if pav='0' then n := 0; end if;
        if n < 18 then
          report "outline " & integer'image(outln) & " px" & integer'image(n) &
                 " y=" & integer'image(to_integer(unsigned(data_out.y)));
        end if;
        n := n + 1;
      elsif pav='1' then
        outln := outln + 1;
      end if;
      pav := data_out.avid;
    end if;
  end process;
end architecture;
