-- ============================================================================
-- tb_sequency_state.vhd  --  Hooks #3 (REGEN coeff-buffer state) + #4 (Spectral
--   Hold), re-verified under the Gate-1 REGEN timing-fix skew (Q2a BRAM-read
--   isolation: +1 read->write clock, +1 ADAPT/hold phase shift).
-- ----------------------------------------------------------------------------
-- Three aligned DUTs, identical 1080p30-class timing, VERTICALLY-CONSTANT input
-- (di.y depends only on active column, NOT on line) so state evolution is cleanly
-- isolable — a memoryless lane is line-invariant; any line-to-line change is the
-- REGEN coefficient RAM evolving:
--   ref  : REGEN=0,    Hold=0              (memoryless reference)
--   live : REGEN=1023,  Hold=0             (vertical feedback LIVE, evolving)
--   held : REGEN=1023,  Hold=0 fields 0-1 -> Hold=1 field>=2  (WARM then FREEZE)
--
-- The old TB engaged Hold FROM RESET, replaying an EMPTY coeff RAM (-> flat 64,
-- != ref) and mis-asserted hold==ref: a testbench-methodology bug.  Spectral Hold
-- must be warmed first (fill the RAM), THEN frozen.
--
-- #3a REGEN wired      : live.y != ref.y somewhere -> feedback is active.
-- #3b REGEN evolving   : ref line L == ref line L+1 (memoryless, sanity); live
--                        line L != line L+1 for some L -> the coeff RAM EVOLVES
--                        line-to-line (a frozen RAM in a feedback program = fail).
-- #3c REGEN bounded    : live.y & held.y in [64,940] over many lines (R3 sat,
--                        loop gain <= 1/2).
-- #4  Spectral Hold    : in the stable hold phase (field>=4) consecutive held
--   (frozen)             output lines are BIT-IDENTICAL -> the spectrum is frozen
--                        while the (constant) input keeps being re-rendered.
-- #4  Hold non-degen.  : the frozen held line is NON-FLAT (column structure) ->
--                        the RAM was WARMED before freeze (rules out the empty-
--                        buffer degenerate that fooled the old TB).
-- #4  Hold effective   : held.y != live.y during the hold phase -> Hold changes
--                        behaviour (freezes vs. keeps evolving).
--
-- NOTE (verifier): pure black-box (this GHDL NULL-derefs on external names); the
-- coeff RAM is observed through its externally-visible line-to-line behaviour.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
library work;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.video_corner_pkg.all;

entity tb_sequency_state is end entity;

architecture tb of tb_sequency_state is
  constant CPL : integer := 400; constant AW : integer := 256; constant AVS : integer := CPL-AW;
  constant ACT : integer := 16;  constant VBL : integer := 4;  constant LPF : integer := ACT+VBL;
  constant HOLD_FIELD : integer := 2;   -- held DUT engages Hold at this input field
  constant STABLE     : integer := 4;   -- assert held frozen for output field>=STABLE
  constant STOPF      : integer := 7;
  constant WARM       : integer := 3*CPL;
  signal clk : std_logic := '0'; signal running : boolean := true;
  signal di : t_video_stream_yuv444_30b;
  signal do_ref, do_live, do_held : t_video_stream_yuv444_30b;
  signal r_ref, r_live, r_held : t_spi_ram := (others => (others => '0'));
  signal cur_field : integer := 0;
begin
  clk_gen : process begin
    while running loop clk<='0'; wait for 5 ns; clk<='1'; wait for 5 ns; end loop; wait;
  end process;

  -- phased register control (Hold engages on the held DUT at HOLD_FIELD)
  p_regs : process(clk)
  begin
    if rising_edge(clk) then
      r_ref  <= (others => (others=>'0'));
      r_ref(1)  <= std_logic_vector(to_unsigned(512,10));   -- TILT flat
      r_ref(7)  <= std_logic_vector(to_unsigned(1023,10));  -- fader wet
      r_ref(8)  <= std_logic_vector(to_unsigned(7,10));     -- 1080p30 id

      r_live <= (others => (others=>'0'));
      r_live(1) <= std_logic_vector(to_unsigned(512,10));
      r_live(3) <= std_logic_vector(to_unsigned(1023,10));  -- REGEN max
      r_live(7) <= std_logic_vector(to_unsigned(1023,10));
      r_live(8) <= std_logic_vector(to_unsigned(7,10));

      r_held <= (others => (others=>'0'));
      r_held(1) <= std_logic_vector(to_unsigned(512,10));
      r_held(3) <= std_logic_vector(to_unsigned(1023,10));  -- REGEN max (warm the RAM)
      r_held(7) <= std_logic_vector(to_unsigned(1023,10));
      r_held(8) <= std_logic_vector(to_unsigned(7,10));
      if cur_field >= HOLD_FIELD then
        r_held(6) <= std_logic_vector(to_unsigned(8,10));   -- tog10 Spectral Hold (bit3)
      else
        r_held(6) <= std_logic_vector(to_unsigned(0,10));
      end if;
    end if;
  end process;

  dut_ref  : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_ref,  data_in=>di, data_out=>do_ref);
  dut_live : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_live, data_in=>di, data_out=>do_live);
  dut_held : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_held, data_in=>di, data_out=>do_held);

  -- VERTICALLY-CONSTANT stimulus (function of active column only; strong within-
  -- block AC so the coeff RAM has real content to resonate/freeze).
  stim : process(clk)
    variable cx,ly,fld : integer := 0; variable avon : boolean; variable ac : integer;
  begin
    if rising_edge(clk) then
      avon := (cx>=AVS) and (ly<ACT); ac := cx-AVS;
      di.hsync_n<='1'; di.vsync_n<='1'; di.field_n<='1';
      if avon then di.avid<='1';
        di.y <= std_logic_vector(to_unsigned(64 + ((ac*37) mod 800),10));  -- line-invariant
        di.u <= std_logic_vector(to_unsigned(512,10)); di.v <= std_logic_vector(to_unsigned(512,10));
      else di.avid<='0'; di.y<=std_logic_vector(to_unsigned(64,10));
        di.u<=std_logic_vector(to_unsigned(512,10)); di.v<=std_logic_vector(to_unsigned(512,10)); end if;
      if cx=CPL-1 then cx:=0; if ly=LPF-1 then ly:=0; fld:=fld+1; else ly:=ly+1; end if;
      else cx:=cx+1; end if;
      cur_field <= fld;
    end if;
  end process;

  mon : process(clk)
    variable cyc : integer := 0;
    variable pav : std_logic := '0';
    variable col, oline : integer := 0;
    variable this_full, prev_full : boolean := false;   -- full-line guard (skip mid-warmup partial capture)
    type iarr is array(0 to AW-1) of integer;
    variable cur_ref, prev_ref   : iarr := (others => -1);
    variable cur_live, prev_live : iarr := (others => -1);
    variable cur_held, prev_held : iarr := (others => -1);
    variable e_wired, e_bound, e_meta       : integer := 0;
    variable e_ref_var, e_live_evolve       : integer := 0;  -- ref should be 0; live > 0
    variable e_held_frozen, e_effective     : integer := 0;
    variable held_nondegen                  : integer := 0;  -- #frozen lines with structure
    variable yv, hmin, hmax, d, i           : integer;
  begin
    if rising_edge(clk) then
      cyc := cyc + 1;

      if cyc>WARM and do_ref.avid='1' then
        if is_x(do_ref.y) or is_x(do_live.y) or is_x(do_held.y) then
          e_meta := e_meta + 1;
        else
          if do_live.y /= do_ref.y then e_wired := e_wired + 1; end if;
          if cur_field >= HOLD_FIELD+1 and do_held.y /= do_live.y then
            e_effective := e_effective + 1;                    -- Hold changes vs live
          end if;
          yv := to_integer(unsigned(do_live.y));
          if yv<64 or yv>940 then e_bound := e_bound + 1; end if;
          yv := to_integer(unsigned(do_held.y));
          if yv<64 or yv>940 then e_bound := e_bound + 1; end if;
          if col < AW then
            cur_ref(col)  := to_integer(unsigned(do_ref.y));
            cur_live(col) := to_integer(unsigned(do_live.y));
            cur_held(col) := to_integer(unsigned(do_held.y));
            col := col + 1;
          end if;
        end if;
      elsif pav='1' and do_ref.avid='0' then
        -- output line just ended: line-to-line compares (only between two FULLY
        -- captured lines — the first post-WARM line starts mid-raster/partial).
        this_full := (col = AW);
        if cyc>WARM and oline>=1 and prev_full and this_full then
          -- ref must be line-invariant (memoryless + constant input)
          for i in 0 to AW-1 loop
            if prev_ref(i)>=0 and cur_ref(i)>=0 and cur_ref(i)/=prev_ref(i) then
              e_ref_var := e_ref_var + 1;
            end if;
          end loop;
          -- live must evolve: count columns that changed since prev line
          d := 0;
          for i in 0 to AW-1 loop
            if prev_live(i)>=0 and cur_live(i)>=0 and cur_live(i)/=prev_live(i) then d := d + 1; end if;
          end loop;
          if d > 0 then e_live_evolve := e_live_evolve + 1; end if;
          -- held must be FROZEN in the stable phase, and non-degenerate
          if cur_field >= STABLE then
            for i in 0 to AW-1 loop
              if prev_held(i)>=0 and cur_held(i)>=0 and cur_held(i)/=prev_held(i) then
                e_held_frozen := e_held_frozen + 1;
              end if;
            end loop;
            hmin := 9999; hmax := -1;
            for i in 0 to AW-1 loop
              if cur_held(i)>=0 then
                if cur_held(i)<hmin then hmin:=cur_held(i); end if;
                if cur_held(i)>hmax then hmax:=cur_held(i); end if;
              end if;
            end loop;
            if hmax-hmin > 8 then held_nondegen := held_nondegen + 1; end if;
          end if;
        end if;
        prev_ref := cur_ref; prev_live := cur_live; prev_held := cur_held;
        prev_full := this_full;
        cur_ref := (others=>-1); cur_live := (others=>-1); cur_held := (others=>-1);
        col := 0; oline := oline + 1;
      end if;
      pav := do_ref.avid;

      if cur_field>=STOPF then
        report "RESULT HOOK3_regen_wired    1080p30: " & str_pf(e_wired>0) &
          " (" & integer'image(e_wired) & " px REGEN!=ref -> feedback active)" severity note;
        report "RESULT HOOK3_regen_evolving 1080p30: " & str_pf(e_live_evolve>0 and e_ref_var=0) &
          " (" & integer'image(e_live_evolve) & " lines live evolves; ref line-var=" &
          integer'image(e_ref_var) & " [expect 0])" severity note;
        report "RESULT HOOK3_regen_bounded  1080p30: " & str_pf(e_bound=0) &
          " (" & integer'image(e_bound) & " oor; " & integer'image(e_meta) & " warmup-X skipped)" severity note;
        report "RESULT HOOK4_hold_frozen    1080p30: " & str_pf(e_held_frozen=0 and held_nondegen>0) &
          " (" & integer'image(e_held_frozen) & " line-to-line changes in hold [expect 0]; " &
          integer'image(held_nondegen) & " frozen lines with structure)" severity note;
        report "RESULT HOOK4_hold_effective 1080p30: " & str_pf(e_effective>0) &
          " (" & integer'image(e_effective) & " px held!=live in hold phase)" severity note;
        report "VIT_DONE state" severity note;
        running<=false; stop;
      end if;
      if cyc>20000000 then report "TIMEOUT" severity failure; end if;
    end if;
  end process;
end architecture;
