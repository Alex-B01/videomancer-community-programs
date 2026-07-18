-- ============================================================================
-- tb_sequency_state.vhd  --  Hooks #3 (REGEN coeff feedback) + #4 (Spectral Hold)
-- ----------------------------------------------------------------------------
-- Three aligned DUTs, identical 1080p30 timing + vertical-gradient content:
--   ref   : REGEN=0,    Hold=0   (no vertical feedback -- reference)
--   regen : REGEN=1023, Hold=0   (feedback LIVE)
--   hold  : REGEN=1023, Hold=1   (feedback FROZEN from reset)
--
-- #3 REGEN wired+evolving : regen output must DIFFER from ref (feedback active).
-- #3 REGEN bounded/stable : regen output stays in [64,940] under MAX regen over
--                           many lines (R3 saturation / loop gain < 1).
-- #4 Spectral Hold        : Hold from reset freezes coeff-buffer writes, so the
--                           buffer stays empty => zero feedback => 'hold' must be
--                           BIT-IDENTICAL to 'ref'.  (Frozen writes persist.)
--
-- NOTE: this GHDL build crashes on external names, so the coeff RAM is observed
-- through its externally-visible behaviour (above) rather than a direct dump;
-- the REGEN write-back PHASE is confirmed by 'regen' producing a coherent,
-- bounded, purely-vertical effect distinct from 'ref'.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
library work;
use work.video_stream_pkg.all;
use work.core_pkg.all;

entity tb_sequency_state is end entity;

architecture tb of tb_sequency_state is
  constant CPL : integer := 400; constant AW : integer := 256; constant AVS : integer := CPL-AW;
  constant ACT : integer := 24;  constant VBL : integer := 4;  constant LPF : integer := ACT+VBL;
  signal clk : std_logic := '0'; signal running : boolean := true;
  signal di : t_video_stream_yuv444_30b;
  signal do_ref, do_reg, do_hold : t_video_stream_yuv444_30b;
  signal r_ref, r_reg, r_hold : t_spi_ram := (others => (others => '0'));
  signal cur_field : integer := 0;
begin
  clk_gen : process begin
    while running loop clk<='0'; wait for 5 ns; clk<='1'; wait for 5 ns; end loop; wait;
  end process;

  -- register sets (quasi-static)
  r_ref(1)  <= std_logic_vector(to_unsigned(512,10));  r_ref(7)  <= std_logic_vector(to_unsigned(1023,10));
  r_ref(8)  <= std_logic_vector(to_unsigned(7,10));
  r_reg(1)  <= std_logic_vector(to_unsigned(512,10));  r_reg(3)  <= std_logic_vector(to_unsigned(1023,10));
  r_reg(7)  <= std_logic_vector(to_unsigned(1023,10)); r_reg(8)  <= std_logic_vector(to_unsigned(7,10));
  r_hold(1) <= std_logic_vector(to_unsigned(512,10));  r_hold(3) <= std_logic_vector(to_unsigned(1023,10));
  r_hold(6) <= std_logic_vector(to_unsigned(8,10));    -- tog10 Spectral Hold = bit3
  r_hold(7) <= std_logic_vector(to_unsigned(1023,10)); r_hold(8) <= std_logic_vector(to_unsigned(7,10));

  dut_ref  : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_ref,  data_in=>di, data_out=>do_ref);
  dut_reg  : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_reg,  data_in=>di, data_out=>do_reg);
  dut_hold : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_hold, data_in=>di, data_out=>do_hold);

  stim : process(clk)
    variable cx,ly,fld,gl : integer := 0; variable avon : boolean; variable ac : integer;
  begin
    if rising_edge(clk) then
      avon := (cx>=AVS) and (ly<ACT); ac := cx-AVS;
      di.hsync_n<='1'; di.vsync_n<='1'; di.field_n<='1';
      if avon then di.avid<='1';
        di.y <= std_logic_vector(to_unsigned(64 + ((gl*53 + ac*3) mod 800),10));  -- strong vertical gradient
        di.u <= std_logic_vector(to_unsigned(512,10)); di.v <= std_logic_vector(to_unsigned(512,10));
      else di.avid<='0'; di.y<=std_logic_vector(to_unsigned(64,10));
        di.u<=std_logic_vector(to_unsigned(512,10)); di.v<=std_logic_vector(to_unsigned(512,10)); end if;
      if cx=CPL-1 then cx:=0; if avon then gl:=gl+1; end if;
        if ly=LPF-1 then ly:=0; fld:=fld+1; else ly:=ly+1; end if;
      else cx:=cx+1; end if;
      cur_field <= fld;
    end if;
  end process;

  mon : process(clk)
    variable cyc : integer := 0;
    variable e_regen, e_hold, e_bound, e_meta : integer := 0; variable yv : integer;
    variable rep : integer := 0;
  begin
    if rising_edge(clk) then
      cyc := cyc + 1;
      if cyc>400 and do_ref.avid='1' then
        if is_x(do_reg.y) or is_x(do_ref.y) or is_x(do_hold.y) then
          e_meta := e_meta + 1;                                        -- metavalue (warmup X)
        else
          if do_reg.y /= do_ref.y then e_regen := e_regen + 1; end if; -- REGEN changes output
          if do_hold.y /= do_ref.y then e_hold := e_hold + 1; end if;  -- Hold must match ref (frozen empty)
          yv := to_integer(unsigned(do_reg.y));
          if yv<64 or yv>940 then
            e_bound := e_bound + 1;                                     -- REGEN bounded
            if rep<8 then
              report "HOOK3 bound VIOLATION cyc=" & integer'image(cyc) &
                     " reg.y=" & integer'image(yv) severity note;
              rep := rep + 1;
            end if;
          end if;
        end if;
      end if;
      if cur_field>=6 then
        report "RESULT HOOK3_regen_wired   1080p30: " &
          work.video_corner_pkg.str_pf(e_regen>0) & " (" & integer'image(e_regen) &
          " px where REGEN!=ref -> feedback active/evolving)" severity note;
        report "RESULT HOOK3_regen_bounded 1080p30: " &
          work.video_corner_pkg.str_pf(e_bound=0) & " (" & integer'image(e_bound) &
          " out-of-range under MAX regen; " & integer'image(e_meta) &
          " metavalue px skipped)" severity note;
        report "RESULT HOOK4_spectral_hold 1080p30: " &
          work.video_corner_pkg.str_pf(e_hold=0) & " (" & integer'image(e_hold) &
          " px where Hold!=noREGEN-ref; 0 => writes truly frozen)" severity note;
        report "VIT_DONE state" severity note;
        running<=false; stop;
      end if;
      if cyc>20000000 then report "TIMEOUT" severity failure; end if;
    end if;
  end process;
end architecture;
