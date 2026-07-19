-- ============================================================================
-- tb_howlbox_normshift.vhd  --  Rate-normalizer COMPONENT proof (hook #1 support).
--   Updated for the shift-ADD scheme (commit 2f48ce5): the normalizer now emits
--   norm_shift (primary) + norm_shift_b (secondary) + norm_use_b (enable), and the
--   consumer computes  step = (delta>>norm_shift) [+ (delta>>norm_shift_b) if ub].
-- ----------------------------------------------------------------------------
-- Instantiates the ACTUAL shipped rate_normalizer with HOWLBOX's real base shifts
-- (STARVE=9, BODY=8, SPIN=4) and drives every timing_id.  Two independent checks:
--
--   (A) EXACT-LUT : norm_shift(i)=clamp(base+da), norm_shift_b(i)=clamp(base+db),
--       norm_use_b=ub  for the design's per-class (da,db,ub) -- catches wiring/
--       clamp bugs.  60/30 Hz classes MUST have norm_use_b='0' (exact single shift,
--       bit-identical -> never regress).
--   (B) SCALE   : the shift-add produces a per-step coefficient rel_scale =
--       2^-da + (ub? 2^-db : 0) relative to the 30 Hz base; the achieved wall-clock
--       ratio vs 30 Hz = 30 / (rel_scale * rate).  Assert |ratio-1| <= 0.25 for
--       every real class (expect ~+-4%).  This is the INDEPENDENT proof that the
--       scheme actually holds the feel constant (not just mirrors the LUT).
--
-- Expected primary/secondary (base+da / base+db) at the five corners:
--   NTSC 0 (1,0,'0')  10 9 5 / -   .  1080i5994 2 (1,0,'0') 10 9 5 / -
--   720p60 14(1,0,'0')10 9 5 / -   .  1080p24   3 (0,2,'1')  9 8 4 / 11 10 6 ub=1
--   1080p30 7 (0,0,'0') 9 8 4 / - (ref)
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
library work;
use work.rate_normalizer_pkg.all;
use work.video_corner_pkg.all;

entity tb_howlbox_normshift is end entity;

architecture tb of tb_howlbox_normshift is
  constant NC : integer := 3;
  signal clk : std_logic := '0';
  signal running : boolean := true;
  signal tid : unsigned(3 downto 0) := (others => '0');
  signal base : t_shift_array(NC-1 downto 0) := (0 => 9, 1 => 8, 2 => 4);  -- STARVE,BODY,SPIN
  signal nshift, nshift_b : t_shift_array(NC-1 downto 0);
  signal nuse_b : std_logic;

  -- independent spec of the design's per-class scheme (da, db, ub) and rate(Hz)
  type cls is record da, db : integer; ub : std_logic; rate : real; end record;
  type clut is array(0 to 15) of cls;
  constant EXP : clut := (
    0 =>(1,0,'0',59.94), 1 =>(1,3,'1',50.0), 2 =>(1,0,'0',59.94), 3 =>(0,2,'1',24.0),
    4 =>(1,0,'0',59.94), 5 =>(1,3,'1',50.0), 6 =>(1,0,'0',59.94), 7 =>(0,0,'0',30.0),
    8 =>(1,3,'1',50.0), 9 =>(0,2,'1',23.98), 10=>(1,0,'0',60.0), 11=>(0,2,'1',25.0),
    12=>(1,3,'1',50.0), 13=>(0,0,'0',29.97), 14=>(1,0,'0',60.0), 15=>(0,0,'0',30.0));

  function clampsh(v : integer) return integer is
  begin if v<1 then return 1; elsif v>31 then return 31; else return v; end if; end function;
begin
  clk_gen : process begin
    while running loop clk<='0'; wait for 5 ns; clk<='1'; wait for 5 ns; end loop; wait;
  end process;

  dut : entity work.rate_normalizer
    generic map (G_NUM_COEFFS => NC)
    port map (clk=>clk, timing_id=>tid, base_shift=>base,
              norm_shift=>nshift, norm_shift_b=>nshift_b, norm_use_b=>nuse_b);

  stim : process
    variable e_lut, e_useb, e_scale : integer := 0;
    variable e0,e1,e2, eb0,eb1,eb2, id : integer;
    variable rel, ratio : real;
  begin
    for id in 0 to 15 loop
      tid <= to_unsigned(id, 4);
      wait until rising_edge(clk); wait until rising_edge(clk);
      -- (A) EXACT-LUT
      e0:=clampsh(9+EXP(id).da); e1:=clampsh(8+EXP(id).da); e2:=clampsh(4+EXP(id).da);
      eb0:=clampsh(9+EXP(id).db); eb1:=clampsh(8+EXP(id).db); eb2:=clampsh(4+EXP(id).db);
      if nshift(0)/=e0 or nshift(1)/=e1 or nshift(2)/=e2 or
         nshift_b(0)/=eb0 or nshift_b(1)/=eb1 or nshift_b(2)/=eb2 then
        e_lut := e_lut + 1;
        report "NORMSHIFT LUT MISMATCH id=" & integer'image(id) &
          " got ns(" & integer'image(nshift(0)) & "," & integer'image(nshift(1)) & "," &
          integer'image(nshift(2)) & ") nsb(" & integer'image(nshift_b(0)) & "," &
          integer'image(nshift_b(1)) & "," & integer'image(nshift_b(2)) & ")" severity note;
      end if;
      -- use_b must match; and 60/30 classes must be '0' (exact single shift)
      if nuse_b /= EXP(id).ub then e_useb := e_useb + 1;
        report "NORMSHIFT use_b MISMATCH id=" & integer'image(id) &
          " got " & std_logic'image(nuse_b) severity note; end if;
      -- (B) SCALE: rel_scale relative to 30 Hz base coeff, achieved ratio vs 30 Hz
      rel := 2.0**(9 - nshift(0));
      if nuse_b = '1' then rel := rel + 2.0**(9 - nshift_b(0)); end if;
      ratio := 30.0 / (rel * EXP(id).rate);
      if id /= 15 and abs(ratio - 1.0) > 0.25 then e_scale := e_scale + 1;
        report "NORMSHIFT SCALE OOB id=" & integer'image(id) severity note; end if;
      if id=0 or id=14 or id=7 or id=2 or id=3 then
        report "NORMSHIFT id=" & integer'image(id) & " ns=" & integer'image(nshift(0)) &
          "/" & integer'image(nshift(1)) & "/" & integer'image(nshift(2)) &
          " nsb=" & integer'image(nshift_b(0)) & "/" & integer'image(nshift_b(1)) &
          "/" & integer'image(nshift_b(2)) & " ub=" & std_logic'image(nuse_b) &
          " scale_ratio_vs30=" & real'image(ratio) severity note;
      end if;
    end loop;
    report "RESULT HOOK1_normshift ALLIDS: " & str_pf(e_lut=0 and e_useb=0 and e_scale=0) &
      " (lut_err=" & integer'image(e_lut) & " useb_err=" & integer'image(e_useb) &
      " scale_oob=" & integer'image(e_scale) & " over 16 ids x 3 coeffs)" severity note;
    report "VIT_DONE normshift" severity note;
    running <= false; stop;
  end process;
end architecture;
