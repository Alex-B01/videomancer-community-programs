-- ============================================================================
-- tb_sequency_rotate.vhd  --  Hook #15 (NEW, REV-F): Scramble mode XOR / Rotate
--   (Tog 8) -- Rotate keeps the DC coefficient HOME (index 0->0); XOR relocates it.
-- ----------------------------------------------------------------------------
-- SCRAMBLE remaps which per-sequency TILT band each coefficient receives:
--   scramble_idx(seq,key,mode): XOR -> seq xor key   (key/=0 relocates DC: 0->key)
--                               Rot -> rotate_left(seq, key[3:2])  (0->0, DC home)
-- The DC coefficient is TRUE seq 0. TREBLE tilt (dir=1, str=3) cuts the LOW band
-- (band0, where DC lives) by >>3 and leaves band3 uncut, so the DC's per-sequency
-- treatment is directly visible as block brightness:
--   * key=0 (no scramble) : DC -> seq 0 -> band0 -> CUT  (reference "home" treatment)
--   * Rotate key=15       : DC -> rotate_left(0,3) = 0 -> band0 -> CUT, IDENTICAL to
--                           key=0 -> Rotate keeps DC HOME (its treatment is unchanged)
--   * XOR key=15          : DC -> 0 xor 15 = 15 -> band3 -> NOT cut -> DC RELOCATED to
--                           a different band -> brightness differs from the home reference
-- (Empirically confirmed: key0=rot=70, xor=559 on a flat 559 field -- Rotate tracks
-- the no-scramble reference; XOR diverges. "DC home" = "== the unscrambled treatment".)
--
-- THREE aligned DUTs (identical input; treble tilt; differ only in scramble):
--   ref: XOR mode, key=0  (no-scramble reference: DC gets its home band treatment)
--   xor: XOR mode, key=15 (in FLAT/AC phases; key=0 in the NULL phase)
--   rot: Rotate  , key=15 (in FLAT/AC phases; key=0 in the NULL phase)
-- Phased content (both DUTs change identically -> comparisons valid every cycle):
--   FLAT (QUANT=0, DC-only)  : #15a rot.y == ref.y BIT-EXACT (Rotate keeps DC home) ;
--                              #15b xor.y /= ref.y and mean_xor >> mean_ref (XOR relocates DC)
--   AC   (QUANT=on, key=15)  : #15c rot.y /= xor.y somewhere (genuinely different permutation)
--   NULL (key=0)             : #15d rot.y == xor.y == ref.y (key 0 = identity in both modes)
--
-- Runs x corners (block-local -> corner-invariant). NEVER edits program VHDL.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
library work;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.video_corner_pkg.all;

entity tb_sequency_rotate is
  generic (
    G_CORNER    : integer := 2;
    G_ACT_LINES : integer := 6;
    G_VBL_LINES : integer := 3
  );
end entity;

architecture tb of tb_sequency_rotate is
  constant CPL : integer := CORNERS(G_CORNER).clocks_per_line;
  constant AW  : integer := CORNERS(G_CORNER).active_width;
  constant AVS : integer := CPL - AW;
  constant LPF : integer := G_ACT_LINES + G_VBL_LINES;
  constant ILV : boolean := CORNERS(G_CORNER).is_interlaced;
  constant D   : integer := 56;
  constant TID : integer := CORNERS(G_CORNER).id;
  constant STOPF   : integer := 4;
  constant WARMCYC : integer := 2*CPL;
  -- phase(field): 0 warm/FLAT .1 FLAT .2 AC(key15) .3 AC(key0 null)
  signal clk : std_logic := '0'; signal running : boolean := true;
  signal di : t_video_stream_yuv444_30b;
  signal do_0, do_x, do_r : t_video_stream_yuv444_30b;   -- key0 ref, XOR key15, Rotate key15
  signal r_0, r_x, r_r : t_spi_ram := (others => (others => '0'));
  signal cur_field : integer := 0;
  signal cur_phase : integer := 0;

  type sl  is array(1 to D) of std_logic;
  type ip  is array(1 to D) of integer;
  signal sh_av : sl  := (others => '0');
  signal sh_ph : ip  := (others => 0);

  function phase_of(f : integer) return integer is
  begin
    if    f <= 1 then return 1;   -- warm + FLAT
    elsif f  = 2 then return 2;   -- AC key15
    else             return 3;    -- AC key0 (null)
    end if;
  end function;
begin
  clk_gen : process begin
    while running loop clk<='0'; wait for 5 ns; clk<='1'; wait for 5 ns; end loop; wait;
  end process;

  p_regs : process(clk)
    variable ph, q, key : integer;
  begin
    if rising_edge(clk) then
      ph := phase_of(cur_field);
      if ph = 1 then q := 0;   else q := 500; end if;   -- FLAT isolates TILT; AC engages deadzone
      if ph = 3 then key := 0; else key := 1023; end if; -- key0 => scramble identity (null)
      -- reference: XOR mode, key ALWAYS 0 (DC gets its home band treatment)
      r_0 <= (others => (others=>'0'));
      r_0(0) <= std_logic_vector(to_unsigned(q,10));
      r_0(1) <= std_logic_vector(to_unsigned(1023,10));   -- TREBLE tilt str3 (cuts band0/DC)
      r_0(7) <= std_logic_vector(to_unsigned(1023,10));
      r_0(8) <= std_logic_vector(to_unsigned(TID,10));
      r_x <= r_0;
      r_x(2) <= std_logic_vector(to_unsigned(key,10));    -- XOR key15 (phases 1,2) / 0 (null)
      r_r <= r_x;
      r_r(6) <= std_logic_vector(to_unsigned(2,10));      -- Tog8 Rotate (bit1)
      cur_phase <= ph;
    end if;
  end process;

  dut_0 : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_0, data_in=>di, data_out=>do_0);
  dut_x : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_x, data_in=>di, data_out=>do_x);
  dut_r : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_r, data_in=>di, data_out=>do_r);

  stim : process(clk)
    variable cx,ly,fld,gl : integer := 0; variable avon : boolean; variable ac,yy : integer;
  begin
    if rising_edge(clk) then
      avon := (cx>=AVS) and (ly<G_ACT_LINES); ac := cx-AVS;
      di.hsync_n<='1'; di.vsync_n<='1'; di.field_n<= '1' when (fld mod 2)=0 else '0';
      if cx<44 then di.hsync_n<='0'; end if;
      if (ly>=G_ACT_LINES) and (ly<G_ACT_LINES+2) then di.vsync_n<='0'; end if;
      if avon then di.avid<='1';
        if phase_of(fld) = 1 then
          yy := 400 + ((gl*53) mod 400);                 -- horizontally-flat (DC-only) line
        else
          yy := 400 + ((ac*29 + gl*17 + ac*ac*3) mod 240); -- AC-rich
        end if;
        di.y <= std_logic_vector(to_unsigned(yy,10));
        di.u <= std_logic_vector(to_unsigned(512,10));
        di.v <= std_logic_vector(to_unsigned(512,10));
      else di.avid<='0'; di.y<=std_logic_vector(to_unsigned(64,10));
        di.u<=std_logic_vector(to_unsigned(512,10)); di.v<=std_logic_vector(to_unsigned(512,10)); end if;
      if cx=CPL-1 then cx:=0; if avon then gl:=gl+1; end if;
        if ly=LPF-1 then ly:=0; fld:=fld+1; else ly:=ly+1; end if;
      else cx:=cx+1; end if;
      cur_field <= fld;
    end if;
  end process;

  shadow : process(clk) begin
    if rising_edge(clk) then
      sh_av(1)<=di.avid; sh_ph(1)<=cur_phase;
      for k in 2 to D loop sh_av(k)<=sh_av(k-1); sh_ph(k)<=sh_ph(k-1); end loop;
    end if;
  end process;

  mon : process(clk)
    constant NAME : string := corner_name(G_CORNER);
    variable cyc : integer := 0;
    variable pav : std_logic := '0';
    variable armed : boolean := false;
    variable ocol : integer := 0;                      -- output active col (skip first block/line)
    variable e_home, e_xrelo, n_flat : integer := 0;   -- flat: rot==ref, xor!=ref
    variable sum_0, sum_x : integer := 0;
    variable n_diff, n_ac, n_null_diff, n_null : integer := 0;
    variable v0, vx, vr, ph : integer;
    variable mean_0, mean_x : integer := 0;
  begin
    if rising_edge(clk) then
      cyc := cyc + 1;
      if cyc>WARMCYC and do_0.avid='1' and pav='0' then armed := true; ocol := 0; end if;
      if armed and do_0.avid='1' and sh_av(D)='1' then
        v0 := to_integer(unsigned(do_0.y));
        vx := to_integer(unsigned(do_x.y));
        vr := to_integer(unsigned(do_r.y));
        ph := sh_ph(D);
        if ph = 1 then                        -- FLAT (DC-only) under treble tilt
          -- exclude the first 16-block of each line: the streaming R2SDF's per-line
          -- pipeline fill (blank->content step) injects spurious AC there, which XOR
          -- and Rotate remap differently -> not a DC-home question.
          if ocol >= 16 then
            n_flat := n_flat + 1;
            if vr /= v0 then e_home  := e_home  + 1; end if;   -- #15a Rotate tracks ref (DC home)
            if vx  = v0 then e_xrelo := e_xrelo + 1; end if;   -- #15b XOR DIVERGES (DC relocated)
            sum_0 := sum_0 + v0; sum_x := sum_x + vx;
          end if;
        elsif ph = 2 then                     -- AC key15: modes must differ
          n_ac := n_ac + 1;
          if vr /= vx then n_diff := n_diff + 1; end if;
        else                                  -- AC key0: both modes identity -> all three equal
          n_null := n_null + 1;
          if vr /= vx or vr /= v0 then n_null_diff := n_null_diff + 1; end if;
        end if;
        ocol := ocol + 1;
      end if;
      pav := do_0.avid;

      if cur_field>=STOPF then
        if n_flat>0 then mean_0 := sum_0/n_flat; mean_x := sum_x/n_flat; end if;
        report "==== SEQUENCY rotate "&NAME&" (id="&integer'image(TID)&
          ") flat mean ref="&integer'image(mean_0)&" xor="&integer'image(mean_x)&" ====" severity note;
        -- #15a Rotate keeps DC HOME: rotate output == no-scramble reference, bit-exact
        report "RESULT HOOK15a_rot_dc_home  "&NAME&": "&str_pf(e_home=0 and n_flat>0)&
          " ("&integer'image(e_home)&" rot!=ref on flat / "&integer'image(n_flat)&
          " px; DC treated as unscrambled)" severity note;
        -- #15b XOR RELOCATES DC: xor diverges from the reference, DC out of the cut band
        report "RESULT HOOK15b_xor_moves_dc "&NAME&": "&
          str_pf(e_xrelo=0 and n_flat>0 and mean_x > mean_0 + 100)&
          " ("&integer'image(e_xrelo)&" xor==ref on flat [want 0]; mean xor "&
          integer'image(mean_x)&" vs ref "&integer'image(mean_0)&")" severity note;
        -- #15c genuinely different permutation on AC content
        report "RESULT HOOK15c_diff_perm    "&NAME&": "&str_pf(n_diff>0)&
          " ("&integer'image(n_diff)&" rot!=xor px / "&integer'image(n_ac)&" AC px)" severity note;
        -- #15d key=0 => both modes identity => rot==xor==ref (toggle inert at key 0)
        report "RESULT HOOK15d_key0_null    "&NAME&": "&str_pf(n_null_diff=0 and n_null>0)&
          " ("&integer'image(n_null_diff)&" divergent w/ key=0 / "&integer'image(n_null)&" px)" severity note;
        report "VIT_DONE rotate "&NAME severity note;
        running<=false; stop;
      end if;
      if cyc>40000000 then report "TIMEOUT" severity failure; end if;
    end if;
  end process;
end architecture;
