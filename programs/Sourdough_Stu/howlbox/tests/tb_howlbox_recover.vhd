-- ============================================================================
-- tb_howlbox_recover.vhd  --  HOWLBOX Gate 2 hook #3 (R7 self-recovery), 1080p30.
-- ----------------------------------------------------------------------------
-- Load-bearing R7: the resonant loop (gain<1) and STARVE death both self-recover;
-- no latch, no power-cycle.  Black-box (this GHDL NULL-derefs external names).
--
-- Phases (by field), high-contrast content (alternating 0/1023) that MAXIMALLY
-- excites the IIR; DC couple, fader wet:
--   A  fields 0..2  : RING=1023, EXTENDED vblank (G_VBL_LINES long) -> #3a the
--      resonator is driven hard then held through a long blank; freeze-during-
--      blanking forces the loop input to 0 every blank clock (r<1 => drains).
--   B  fields 3..6  : RING=1023 + STARVE=1023 -> death-flare destabilise, maximal
--      stress (build sag).
--   C  fields 7..10 : STARVE released to 0 -> #3b recover; the knob takes effect
--      immediately (derived controls relax) though the sag settle is slow.
--
-- Asserts:
--   #3-bounded  : EVERY active output pixel in [64,940]/[64,960] across A+B+C
--                 (resonator never latches / blows up -- r<1).
--   #3-nometa   : no metavalue ('U'/'X') on data_out after warmup (no X-latch).
--   #3a-blank   : during the extended blank, data_out.y stays in-range (no
--                 unbounded growth while the loop is starved of input).
--   #3b-recover : after STARVE release the output still RESPONDS to input
--                 (per-line luma range > 0) i.e. it did not freeze at a collapsed
--                 constant -> self-recovery underway, no power-cycle.
--   #3c-postblank: first active pixel after each blank is a valid in-range sample.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
library work;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.video_corner_pkg.all;

entity tb_howlbox_recover is
  generic (
    G_CORNER    : integer := 2;    -- 1080p30
    G_ACT_LINES : integer := 6;
    G_VBL_LINES : integer := 60;   -- EXTENDED vblank (~60*700 = 42k blank clocks/field)
    G_STOPF     : integer := 11
  );
end entity;

architecture tb of tb_howlbox_recover is
  constant CPL : integer := 700;
  constant AW  : integer := 600;
  constant AVS : integer := CPL - AW;
  constant LPF : integer := G_ACT_LINES + G_VBL_LINES;
  signal clk : std_logic := '0';
  signal running : boolean := true;
  signal data_in, data_out : t_video_stream_yuv444_30b;
  signal regs : t_spi_ram := (others => (others => '0'));
  signal cur_field : integer := 0;
begin
  clk_gen : process begin
    while running loop clk<='0'; wait for 5 ns; clk<='1'; wait for 5 ns; end loop; wait;
  end process;

  dut : entity work.program_top(howlbox)
    port map (clk => clk, registers_in => regs, data_in => data_in, data_out => data_out);

  p_regs : process(clk)
    variable r : t_spi_ram := (others => (others => '0'));
  begin
    if rising_edge(clk) then
      r := (others => (others => '0'));
      r(1) := std_logic_vector(to_unsigned(1023, 10));   -- RING max (all phases)
      r(6) := std_logic_vector(to_unsigned(1, 10));      -- DC couple
      r(7) := std_logic_vector(to_unsigned(1023, 10));   -- fader WET
      r(8) := std_logic_vector(to_unsigned(CORNERS(G_CORNER).id, 10));
      if cur_field >= 3 and cur_field <= 6 then
        r(5) := std_logic_vector(to_unsigned(1023, 10)); -- STARVE max (phase B)
      end if;
      regs <= r;
    end if;
  end process;

  p_gen : process(clk)
    variable cx, ly, fld : integer := 0;
    variable act : boolean; variable ac : integer;
  begin
    if rising_edge(clk) then
      act := (cx >= AVS) and (ly < G_ACT_LINES);
      ac  := cx - AVS;
      data_in.hsync_n <= '1'; data_in.vsync_n <= '1'; data_in.field_n <= '1';
      if cx < 44 then data_in.hsync_n <= '0'; end if;
      if (ly >= G_ACT_LINES) and (ly < G_ACT_LINES + 3) then data_in.vsync_n <= '0'; end if;
      if act then
        data_in.avid <= '1';
        if (ac mod 2) = 0 then data_in.y <= std_logic_vector(to_unsigned(1023,10));
        else                   data_in.y <= std_logic_vector(to_unsigned(0,10)); end if;
        data_in.u <= std_logic_vector(to_unsigned(300,10));
        data_in.v <= std_logic_vector(to_unsigned(700,10));
      else
        data_in.avid <= '0';
        data_in.y <= std_logic_vector(to_unsigned(64,10));
        data_in.u <= std_logic_vector(to_unsigned(512,10));
        data_in.v <= std_logic_vector(to_unsigned(512,10));
      end if;
      cur_field <= fld;
      if cx = CPL-1 then cx := 0;
        if ly = LPF-1 then ly := 0; fld := fld + 1; else ly := ly + 1; end if;
      else cx := cx + 1; end if;
    end if;
  end process;

  p_mon : process(clk)
    constant NAME : string := corner_name(G_CORNER);
    variable cyc : integer := 0;
    variable pav : std_logic := '0';
    variable e_bound, e_meta, e_blank, e_postblank : integer := 0;
    variable n_act : integer := 0;
    -- per-line luma range in phase C (recovery: must respond to input)
    variable lmin, lmax : integer := 0;
    variable c_lines_resp, c_lines_tot : integer := 0;
    variable yv, uvv, vvv : integer;
    constant WARM : integer := 4*CPL;
  begin
    if rising_edge(clk) then
      cyc := cyc + 1;

      if cyc > WARM then
        if is_x(data_out.y) or is_x(data_out.u) or is_x(data_out.v) or is_x(data_out.avid) then
          e_meta := e_meta + 1;
        else
          if data_out.avid = '1' then
            yv := to_integer(unsigned(data_out.y));
            uvv := to_integer(unsigned(data_out.u));
            vvv := to_integer(unsigned(data_out.v));
            n_act := n_act + 1;
            if yv<64 or yv>940 or uvv<64 or uvv>960 or vvv<64 or vvv>960 then
              e_bound := e_bound + 1;
            end if;
            if pav = '0' then  -- first active pixel after a blank (#3c)
              if yv<64 or yv>940 then e_postblank := e_postblank + 1; end if;
              lmin := yv; lmax := yv;
            else
              if yv < lmin then lmin := yv; end if;
              if yv > lmax then lmax := yv; end if;
            end if;
          else
            -- #3a: during blank, output must stay in-range (no unbounded growth)
            if not is_x(data_out.y) then
              yv := to_integer(unsigned(data_out.y));
              if yv<0 or yv>1023 then e_blank := e_blank + 1; end if;
            end if;
            -- line just ended: in phase C, did luma respond (range>0)?
            if pav = '1' and cur_field >= 8 then
              c_lines_tot := c_lines_tot + 1;
              if (lmax - lmin) > 4 then c_lines_resp := c_lines_resp + 1; end if;
            end if;
          end if;
        end if;
      end if;
      pav := data_out.avid;

      if cur_field >= G_STOPF then
        report "==== HOWLBOX recover " & NAME & " ====";
        report "RESULT HOOK3_bounded    " & NAME & ": " & str_pf(e_bound = 0 and n_act > 0) &
          " (" & integer'image(e_bound) & " oor / " & integer'image(n_act) &
          " active px; RING+STARVE max + long blank)" severity note;
        report "RESULT HOOK3_nometa     " & NAME & ": " & str_pf(e_meta = 0) &
          " (" & integer'image(e_meta) & " metavalue out px after warmup)" severity note;
        report "RESULT HOOK3a_blankbound " & NAME & ": " & str_pf(e_blank = 0) &
          " (" & integer'image(e_blank) & " blank-px oor thru extended vblank)" severity note;
        report "RESULT HOOK3b_recover   " & NAME & ": " &
          str_pf(c_lines_resp > 0 and 2*c_lines_resp >= c_lines_tot) &
          " (" & integer'image(c_lines_resp) & "/" & integer'image(c_lines_tot) &
          " post-release lines respond to input => not frozen)" severity note;
        report "RESULT HOOK3c_postblank " & NAME & ": " & str_pf(e_postblank = 0) &
          " (" & integer'image(e_postblank) & " invalid first-post-blank px)" severity note;
        report "VIT_DONE recover " & NAME severity note;
        running <= false; stop;
      end if;
      if cyc > 40000000 then report "TIMEOUT recover" severity failure; end if;
    end if;
  end process;
end architecture;
