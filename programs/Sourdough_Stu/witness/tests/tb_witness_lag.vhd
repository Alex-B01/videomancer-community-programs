-- ============================================================================
-- tb_witness_lag.vhd  --  WITNESS Lite gaze-DYNAMICS diagnostic (G3 LAG rate-class,
--   G4 EVADE, 1080i5994 anomaly) -- DIRECT sub-entity instantiation.
-- ----------------------------------------------------------------------------
-- witness_moments exposes cx_blk_lag / cy_blk_lag as OUTPUT PORTS, so the
-- block-space gaze is observed DIRECTLY (no wet path, no iris proxy, no VHDL-2008
-- external names).  rate_normalizer(G=2) is wired to witness_moments exactly as
-- witness.vhd does it, at three timing ids (1080p30 / 1080p24 / 1080i5994) sharing
-- one synthetic centroid stimulus, isolating the LAG/rate-class from the datapath.
--
-- Stimulus: each "frame" sweeps all 2048 cells (active) writing luma=940 into a
-- 2-row BLOCK band (block_y in [TY,TY+1]) and 0 elsewhere -> the moments solve puts
-- the centroid at cy_blk = TY.  The store is cleared by the scan each frame, so the
-- centroid target JUMPS in one frame when TY steps -> the ONLY lag is the 1-pole LAG.
--
--   norm_shift dump : the exact norm_shift/_b/use_b each rate_normalizer emits.
--   G3 rate-class   : frames-to-settle(cy) x real field-period == same wall-clock
--                     within +-25% across 24/30/5994.
--   1080i anomaly   : is 1080i5994 settling FASTER (should be SLOWEST: shift 3)?
--   G4 EVADE        : on T8 with the gaze already settled ON the centroid, does
--                     cy_blk_lag get repelled and then return (no stuck, R7)?
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
library work;
use work.rate_normalizer_pkg.all;

entity tb_witness_lag is end entity;

architecture tb of tb_witness_lag is
  constant NDUT : integer := 3;
  constant TIDS : integer_vector(0 to 2) := (7, 3, 2);          -- 1080p30 / 1080p24 / 1080i5994
  constant PERIOD : integer_vector(0 to 2) := (3337, 4167, 1668); -- x0.01 ms/field (30 / 24 / 59.94)
  constant BASE : integer := 2;                                -- base_shift @ K3~128
  constant TY0 : integer := 6;
  constant TY1 : integer := 24;
  constant STEP_FR  : integer := 6;
  constant EVADE_FR : integer := 24;   -- give i5994 (slow) time to reach its floor first
  constant REL_FR   : integer := 30;
  constant STOP_FR  : integer := 42;
  constant STOL     : integer := 4;    -- settle tolerance (>= the shift-2 dead-zone)
  constant NCELL : integer := 2048;
  constant VBL   : integer := 2200;

  signal clk : std_logic := '0';
  signal running : boolean := true;
  signal active, frame_end, t8 : std_logic := '0';
  signal cell_addr : unsigned(10 downto 0) := (others => '0');
  signal luma : unsigned(9 downto 0) := (others => '0');
  signal sat  : unsigned(10 downto 0) := (others => '0');
  signal t7   : std_logic := '0';
  signal frame_no : integer := 0;

  signal base_sh : t_shift_array(1 downto 0) := (others => BASE);
  type sh_arr is array(0 to 2) of t_shift_array(1 downto 0);
  signal nsh, nshb : sh_arr;
  signal nub : std_logic_vector(0 to 2);
  type cy_arr is array(0 to 2) of signed(8 downto 0);
  signal cyl, cxl : cy_arr;

  type rec_t is array(0 to 2, 0 to STOP_FR) of integer;
  signal cyrec : rec_t := (others => (others => -99));

  function str_pf(b : boolean) return string is
  begin if b then return "PASS"; else return "FAIL"; end if; end function;
begin
  clk <= not clk after 5 ns when running else '0';

  gen : for i in 0 to 2 generate
    rn : entity work.rate_normalizer generic map (G_NUM_COEFFS => 2)
      port map (clk=>clk, timing_id=>to_unsigned(TIDS(i),4), base_shift=>base_sh,
                norm_shift=>nsh(i), norm_shift_b=>nshb(i), norm_use_b=>nub(i));
    mo : entity work.witness_moments
      port map (clk=>clk, active=>active, cell_addr=>cell_addr, luma=>luma, sat=>sat,
                t7_track=>t7, frame_end=>frame_end, t8_evade=>t8,
                lag_shift=>nsh(i)(0), lag_shift_b=>nshb(i)(0), lag_use_b=>nub(i),
                cx_blk_lag=>cxl(i), cy_blk_lag=>cyl(i), solve_done=>open);
  end generate;

  -- driver: per frame -> ACTIVE cell sweep (band at TY) then VBLANK (frame_end + scan)
  drv : process
    variable ty : integer;
    variable by : integer;
  begin
    while running loop
      if frame_no < STEP_FR then ty := TY0; else ty := TY1; end if;
      if frame_no >= EVADE_FR and frame_no < REL_FR then t8 <= '1'; else t8 <= '0'; end if;
      -- ACTIVE: write the band into the store
      active <= '1';
      for c in 0 to NCELL-1 loop
        cell_addr <= to_unsigned(c, 11);
        by := c / 64;
        if by >= ty and by <= ty+1 then luma <= to_unsigned(940,10); else luma <= to_unsigned(0,10); end if;
        wait until rising_edge(clk);
      end loop;
      active <= '0'; luma <= (others=>'0');
      -- VBLANK: pulse frame_end, let the 2048-cell scan + divide + LAG run
      frame_end <= '1'; wait until rising_edge(clk); frame_end <= '0';
      for c in 0 to VBL-1 loop wait until rising_edge(clk); end loop;
      frame_no <= frame_no + 1;
      if frame_no >= STOP_FR then running <= false; end if;
    end loop;
    wait;
  end process;

  -- record cy_blk_lag per DUT at each frame boundary (value settled from prev solve)
  rec : process(clk)
    variable pfe : std_logic := '0';
  begin
    if rising_edge(clk) then
      if frame_end = '1' and pfe = '0' then
        for i in 0 to 2 loop
          if frame_no <= STOP_FR then cyrec(i, frame_no) <= to_integer(cyl(i)); end if;
        end loop;
      end if;
      pfe := frame_end;
    end if;
  end process;

  eval : process
    variable settle : integer_vector(0 to 2) := (others => -1);
    variable wc : integer_vector(0 to 2);
    variable ref, lo, hi : integer;
    variable evmin, evmax, rel : integer;
  begin
    wait until frame_no >= STOP_FR;
    wait for 1 ns;
    report "LAG norm_shift dump (base=" & integer'image(BASE) & "): " &
      "p30 id7 sh=" & integer'image(nsh(0)(0)) & " shb=" & integer'image(nshb(0)(0)) & " ub=" & std_logic'image(nub(0)) &
      " | p24 id3 sh=" & integer'image(nsh(1)(0)) & " shb=" & integer'image(nshb(1)(0)) & " ub=" & std_logic'image(nub(1)) &
      " | i5994 id2 sh=" & integer'image(nsh(2)(0)) & " shb=" & integer'image(nshb(2)(0)) & " ub=" & std_logic'image(nub(2)) severity note;
    for f in 0 to STOP_FR loop
      report "LAG_TRACE f=" & integer'image(f) & " cy_p30=" & integer'image(cyrec(0,f)) &
        " cy_p24=" & integer'image(cyrec(1,f)) & " cy_i5994=" & integer'image(cyrec(2,f)) severity note;
    end loop;
    -- settle: frames after step until |cy - TY1| <= 2 (dead-zone tolerance)
    for i in 0 to 2 loop
      for f in STEP_FR to EVADE_FR-1 loop
        if settle(i) < 0 and cyrec(i,f) /= -99 and abs(cyrec(i,f)-TY1) <= STOL then settle(i) := f - STEP_FR + 1; end if;
      end loop;
      if settle(i) < 0 then settle(i) := EVADE_FR - STEP_FR; end if;
      wc(i) := settle(i) * PERIOD(i);
    end loop;
    ref := wc(0); lo := ref-(ref/4); hi := ref+(ref/4);
    report "RESULT G3_LAG_rateclass 24/30/5994: " &
      str_pf(wc(1)>=lo and wc(1)<=hi and wc(2)>=lo and wc(2)<=hi) &
      " (settle f: p30=" & integer'image(settle(0)) & " p24=" & integer'image(settle(1)) &
      " i5994=" & integer'image(settle(2)) & "; wall x0.01ms " & integer'image(wc(0)) & "/" &
      integer'image(wc(1)) & "/" & integer'image(wc(2)) & " ref+-25%=[" & integer'image(lo) & "," & integer'image(hi) & "])" severity note;
    report "RESULT G3b_i5994_not_fastest: " & str_pf(settle(2) >= settle(0)) &
      " (i5994 shift=" & integer'image(nsh(2)(0)) & ">p30 shift=" & integer'image(nsh(0)(0)) &
      " => i5994 must settle SLOWER; settle i5994=" & integer'image(settle(2)) & " p30=" & integer'image(settle(0)) & ")" severity note;
    -- EVADE: gaze settled on centroid (=TY1) at REL-1... check evade window pushed cy AWAY from TY1
    evmin := 99; evmax := -99;
    for f in EVADE_FR to REL_FR-1 loop
      if cyrec(0,f) < evmin then evmin := cyrec(0,f); end if;
      if cyrec(0,f) > evmax then evmax := cyrec(0,f); end if;
    end loop;
    rel := cyrec(0, STOP_FR-1);
    report "RESULT G4_EVADE_repel_return: " &
      str_pf((abs(evmin-TY1) > 3 or abs(evmax-TY1) > 3) and abs(rel-TY1) <= 3) &
      " (pre-evade cy=" & integer'image(cyrec(0,EVADE_FR-1)) & " evade[min,max]=[" & integer'image(evmin) &
      "," & integer'image(evmax) & "] released cy=" & integer'image(rel) & " (centroid=" & integer'image(TY1) & "))" severity note;
    report "VIT_DONE lag" severity note;
    wait;
  end process;
end architecture;
