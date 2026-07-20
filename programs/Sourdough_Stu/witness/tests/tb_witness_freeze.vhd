-- ============================================================================
-- tb_witness_freeze.vhd  --  GAZE-UPDATE-PATH freeze reproduction (hardware gap).
-- ----------------------------------------------------------------------------
-- Hardware symptom (Gate-5): gaze FROZEN at its reset value (block ~0,0) -> IRIS
-- off-screen/inert, Track/LAG/EVADE inert; PULL + Fader (gaze-independent) fine.
-- The gaze latch s_cxlq/s_cylq (Q5) in witness_moments has NO reset and holds its
-- init 0 until the FSM completes a vblank scan+solve (S_LAG3). This TB drives
-- witness_moments DIRECTLY (cx/cy_blk_lag are output ports) under each degenerate
-- condition and reports the settled gaze, to pin which link freezes it at (0,0):
--
--   G_MODE 0 NORMAL      : blob mass + frame_end each frame  -> gaze tracks blob.
--   G_MODE 1 SCAN_NEVER  : blob mass, active, but frame_end NEVER  (#2: vblank scan
--                          never triggers) -> gaze holds RESET (0,0).
--   G_MODE 2 EMPTY_NOACC : active NEVER (thumbnail never written)  (#1: no accumulate)
--                          + frame_end -> M00=0.
--   G_MODE 3 BLACK       : all-black active (luma=0) + frame_end  (#3: 0/0 divide)
--                          -> what does the 2-phase divider emit for M00=0?
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
library work;
use work.rate_normalizer_pkg.all;

entity tb_witness_freeze is
  generic ( G_MODE : integer := 0 );
end entity;

architecture tb of tb_witness_freeze is
  constant NCELL : integer := 2048; constant VBL : integer := 2400;
  constant TX : integer := 40; constant TY : integer := 20;   -- blob block (non-corner)
  constant NFR : integer := 14;
  signal clk : std_logic := '0'; signal running : boolean := true;
  signal active, frame_end, t8 : std_logic := '0';
  signal cell_addr : unsigned(10 downto 0) := (others => '0');
  signal luma : unsigned(9 downto 0) := (others => '0');
  signal sat  : unsigned(10 downto 0) := (others => '0');
  signal lsh, lshb : t_shift := 1; signal lub : std_logic := '0';
  signal cxl, cyl : signed(8 downto 0);
  signal frame_no : integer := 0;
  signal cx_final, cy_final : integer := -999;
  function str_pf(b : boolean) return string is
  begin if b then return "PASS"; else return "FAIL"; end if; end function;
begin
  clk <= not clk after 5 ns when running else '0';

  mo : entity work.witness_moments
    port map (clk=>clk, active=>active, cell_addr=>cell_addr, luma=>luma, sat=>sat,
              t7_track=>'0', frame_end=>frame_end, t8_evade=>t8,
              lag_shift=>lsh, lag_shift_b=>lshb, lag_use_b=>lub,
              cx_blk_lag=>cxl, cy_blk_lag=>cyl, solve_done=>open);

  drv : process
    variable by : integer;
  begin
    while running loop
      -- ACTIVE cell sweep (unless the mode kills accumulation)
      if G_MODE /= 2 then
        active <= '1';
        for c in 0 to NCELL-1 loop
          cell_addr <= to_unsigned(c, 11);
          by := c / 64;
          if G_MODE = 3 then luma <= to_unsigned(0,10);                      -- all-black
          elsif (by >= TY and by <= TY+1) then luma <= to_unsigned(940,10);  -- blob band
          else luma <= to_unsigned(0,10); end if;
          wait until rising_edge(clk);
        end loop;
        active <= '0'; luma <= (others=>'0');
      else
        active <= '0';                                                       -- never accumulate
        for c in 0 to NCELL-1 loop wait until rising_edge(clk); end loop;
      end if;
      -- VBLANK: pulse frame_end (unless SCAN_NEVER), let the solve run
      if G_MODE /= 1 then frame_end <= '1'; wait until rising_edge(clk); frame_end <= '0';
      else wait until rising_edge(clk); end if;
      for c in 0 to VBL-1 loop wait until rising_edge(clk); end loop;
      frame_no <= frame_no + 1;
    end loop;
    wait;
  end process;

  eval : process
  begin
    wait until frame_no >= NFR; wait for 1 ns;
    cx_final <= to_integer(cxl); cy_final <= to_integer(cyl); wait for 1 ns;
    report "FREEZE mode=" & integer'image(G_MODE) & " settled gaze block = (" &
      integer'image(cx_final) & "," & integer'image(cy_final) & ")  [blob=(" &
      integer'image(TX) & "," & integer'image(TY) & ") reset=(0,0)]" severity note;
    if G_MODE = 0 then
      report "RESULT F0_normal_tracks_blob: " & str_pf(abs(cy_final-TY)<=3) &
        " (cy=" & integer'image(cy_final) & " ~ blob " & integer'image(TY) & ")" severity note;
    elsif G_MODE = 1 then
      report "RESULT F1_scan_never_FREEZES_at_reset: " & str_pf(cx_final=0 and cy_final=0) &
        " (gaze=(" & integer'image(cx_final) & "," & integer'image(cy_final) &
        ") == reset (0,0) => REPRODUCES the hardware freeze)" severity note;
    elsif G_MODE = 2 then
      report "RESULT F2_noaccum_M00zero: gaze=(" & integer'image(cx_final) & "," &
        integer'image(cy_final) & ") (empty thumbnail / no accumulate)" severity note;
    else
      report "RESULT F3_black_M00zero_divide: gaze=(" & integer'image(cx_final) & "," &
        integer'image(cy_final) & ") (M00=0 -> 2-phase divider output; 127 => darken-all, NOT 0,0)" severity note;
    end if;
    report "VIT_DONE freeze" severity note;
    running <= false; stop;
  end process;
end architecture;
