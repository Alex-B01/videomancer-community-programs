-- ============================================================================
-- tb_sequency_identity_adv.vhd  --  Hook #1 (round-trip identity) — STRONGEST.
-- ----------------------------------------------------------------------------
-- ADVERSARIAL exhaustive-vertex identity oracle, PROMOTED from the Gate-0
-- scratchpad (tb_sequency_sdf_adv.vhd) into the permanent Gate-2 harness and
-- ADAPTED to the committed Y-only build: architecture program_top(sequency),
-- LATENCY 52, and the R1 broadcast-safe output clamp [64,940] on Y.
--
-- WHY vertices: WHT coefficient + SDF butterfly-intermediate magnitudes are
-- LINEAR in the input, so they hit their extremes at the VERTICES of the input
-- box (each pixel at a rail). Random interior data almost never reaches the
-- razor-thin width margins the +1-bit inverse widen (A6) was re-derived around.
-- The complete vertex set {rail}^16 = 65536 blocks is the worst-case set for any
-- width-truncation / low-bit-drop bug.
--
-- ORACLE (neutral knobs, QUANTIZE=0, TILT=512 flat, fader wet): the organ is a
-- pure 52-clk delay of Y, so
--     expected_out.y = clamp(in.y delayed 52, 64, 940)     (Y rides the R1 clamp)
--     expected_out.u = in.u delayed 52   (U/V are un-clamped passthrough)
--     expected_out.v = in.v delayed 52
-- The clamp is a NO-OP for legal-range vertices, so:
--   * Phase B EXHAUSTIVE at {G_LO=64,G_HI=940} (clamp-legal rails) => bit-exact
--     identity for all 65536 sign patterns, ZERO clamp masking — the strongest
--     exact proof over the operating range.
--   * Phase A named worst cases at the TRUE rails {0,1023} (DC=16368, forward
--     stage-1 head+in=2046-in-2047 razor margin) => oracle applies the clamp;
--     verifies the R1 clamp + gross reconstruction at maximum internal magnitude.
-- Re-run this after any REGEN/inverse width change (§7-A6) — it stresses exactly
-- the margins a widen can break.  NEVER edits program VHDL.
--
-- Toy 128-block/line raster (block-local identity is width-independent; the true
-- five-corner geometry is tb_sequency_corners). REGEN off => the 2048-deep coeff
-- buffer is not fed back; blocks tile 16-aligned from avid-rising.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
library work;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.video_corner_pkg.all;

entity tb_sequency_identity_adv is
  generic (
    SWEEP_MAX : integer := 65535;   -- 65535 = full exhaustive vertex sweep
    G_LO      : integer := 64;      -- Phase-B low rail  (clamp-legal => exact)
    G_HI      : integer := 940      -- Phase-B high rail (clamp-legal => exact)
  );
end entity;

architecture tb of tb_sequency_identity_adv is
  constant LAT : integer := 52;             -- Gate-1 timing-fix end-to-end latency
  constant HBL : integer := 64;             -- > LAT: pipeline drains each line
  constant BPL : integer := 128;            -- blocks/active-line (128*16 = 2048 = buffer depth)

  signal clk  : std_logic := '0';
  signal din  : t_video_stream_yuv444_30b :=
    (y=>(others=>'0'), u=>(others=>'0'), v=>(others=>'0'),
     avid=>'0', hsync_n=>'1', vsync_n=>'1', field_n=>'0');
  signal dout : t_video_stream_yuv444_30b;
  signal regs : t_spi_ram := (others => (others => '0'));
  signal done : boolean := false;

  type t_h10 is array(0 to LAT) of std_logic_vector(9 downto 0);
  type t_h1  is array(0 to LAT) of std_logic;
  type t_h16 is array(0 to LAT) of std_logic_vector(15 downto 0);
  type t_hi  is array(0 to LAT) of integer;

  function clampy(v : integer) return integer is
  begin
    if v < 64 then return 64; elsif v > 940 then return 940; else return v; end if;
  end function;

  -- parity of low 4 bits
  function par4(x : integer) return std_logic is
    variable v : unsigned(3 downto 0) := to_unsigned(x mod 16, 4);
  begin
    return v(0) xor v(1) xor v(2) xor v(3);
  end;

  -- Walsh/Hadamard basis row r -> 16-bit vertex signature (bit c = +1 when
  -- popcount(r AND c) even).
  function walsh_sig(r : integer; invert : boolean) return std_logic_vector is
    variable s : std_logic_vector(15 downto 0) := (others => '0');
    variable b : std_logic;
  begin
    for c in 0 to 15 loop
      if par4(to_integer(to_unsigned(r,4) and to_unsigned(c,4))) = '0'
        then b := '1'; else b := '0'; end if;
      if invert then b := not b; end if;
      s(c) := b;
    end loop;
    return s;
  end;
begin
  clk <= not clk after 5 ns when not done else '0';

  -- NEUTRAL knobs: QUANTIZE=0, TILT=512 (flat — critical: TILT=0 would max-cut
  -- highs and break identity), fader wet. Everything else 0.
  regs(1) <= std_logic_vector(to_unsigned(512, 10));
  regs(7) <= std_logic_vector(to_unsigned(1023, 10));

  dut : entity work.program_top(sequency)
    port map (clk=>clk, registers_in=>regs, data_in=>din, data_out=>dout);

  p_stim : process
    variable hy, hu, hv : t_h10 := (others => (others => '0'));
    variable ha : t_h1 := (others => '0');
    variable hh : t_h1 := (others => '1');
    variable hs : t_h1 := (others => '1');
    variable hf : t_h1 := (others => '0');
    variable hsig : t_h16 := (others => (others => '0'));
    variable hpos : t_hi  := (others => 0);
    variable cur_sig : std_logic_vector(15 downto 0) := (others => '0');
    variable cur_pos : integer := 0;
    variable armed   : boolean := false;
    variable compares: integer := 0;
    variable e_y, e_uv, e_sync : integer := 0;
    variable rep     : integer := 0;
    variable gcnt    : integer := 0;
    variable lo_sl, hi_sl : std_logic_vector(9 downto 0);

    procedure tick is
    begin
      wait until rising_edge(clk);
      for i in LAT downto 1 loop
        hy(i):=hy(i-1); hu(i):=hu(i-1); hv(i):=hv(i-1);
        ha(i):=ha(i-1); hh(i):=hh(i-1); hs(i):=hs(i-1); hf(i):=hf(i-1);
        hsig(i):=hsig(i-1); hpos(i):=hpos(i-1);
      end loop;
      hy(0):=din.y; hu(0):=din.u; hv(0):=din.v;
      ha(0):=din.avid; hh(0):=din.hsync_n; hs(0):=din.vsync_n; hf(0):=din.field_n;
      hsig(0):=cur_sig; hpos(0):=cur_pos;
      gcnt := gcnt + 1;
    end;

    procedure check is
      variable exp_y : integer;
    begin
      if armed then
        if dout.avid /= ha(LAT) or dout.hsync_n /= hh(LAT) or
           dout.vsync_n /= hs(LAT) or dout.field_n /= hf(LAT) then
          e_sync := e_sync + 1;
        end if;
        if ha(LAT) = '1' then
          compares := compares + 1;
          -- Y rides the R1 clamp; U/V are un-clamped passthrough.
          exp_y := clampy(to_integer(unsigned(hy(LAT))));
          if to_integer(unsigned(dout.y)) /= exp_y then
            e_y := e_y + 1;
            if rep < 8 then
              report "IDENTITY FAIL blk=0x"&to_hstring(hsig(LAT))
                   &" posInBlk="&integer'image(hpos(LAT))
                   &" exp_y="&integer'image(exp_y)
                   &" got_y="&integer'image(to_integer(unsigned(dout.y)))
                   &" @gcnt="&integer'image(gcnt) severity note;
              rep := rep + 1;
            end if;
          end if;
          if dout.u /= hu(LAT) or dout.v /= hv(LAT) then
            e_uv := e_uv + 1;
          end if;
        end if;
      end if;
    end;

    procedure hblank(n : integer) is
    begin
      for i in 0 to n-1 loop
        din.avid <= '0';
        din.hsync_n <= '0' when i < 8 else '1';
        din.vsync_n <= '1';
        din.y <= (others=>'0'); din.u <= (others=>'0'); din.v <= (others=>'0');
        cur_sig := (others=>'0'); cur_pos := 0;
        tick; check;
      end loop;
    end;

    -- drive one 16-px block; pixel p = HI-symbol if sig(p)='1' else LO-symbol
    procedure send_block(sig : std_logic_vector(15 downto 0);
                         lo, hi : std_logic_vector(9 downto 0)) is
    begin
      for p in 0 to 15 loop
        din.avid <= '1'; din.hsync_n <= '1'; din.vsync_n <= '1';
        if sig(p) = '1' then din.y<=hi; din.u<=hi; din.v<=hi;
        else                 din.y<=lo; din.u<=lo; din.v<=lo; end if;
        cur_sig := sig; cur_pos := p;
        tick; check;
      end loop;
    end;
  begin
    lo_sl := std_logic_vector(to_unsigned(G_LO, 10));
    hi_sl := std_logic_vector(to_unsigned(G_HI, 10));

    -- warmup: 2 lines (fills pipeline + 64-deep delay RAM), not yet armed
    for l in 0 to 1 loop
      hblank(HBL);
      for b in 0 to BPL-1 loop send_block(x"0000", lo_sl, hi_sl); end loop;
    end loop;
    armed := true;

    ------------------------------------------------------------------ Phase A
    -- TRUE rails {0,1023}: max internal magnitude (DC=16368, fwd-stage1=2046).
    -- Oracle applies the R1 clamp -> verifies clamp + gross reconstruction.
    lo_sl := (others=>'0'); hi_sl := (others=>'1');
    hblank(HBL); for b in 0 to 7 loop send_block(x"FFFF", lo_sl, hi_sl); end loop; -- all-1023
    hblank(HBL); for b in 0 to 7 loop send_block(x"0000", lo_sl, hi_sl); end loop; -- all-0
    hblank(HBL); for b in 0 to 7 loop send_block(x"AAAA", lo_sl, hi_sl); end loop; -- alt
    hblank(HBL); for b in 0 to 7 loop send_block(x"5555", lo_sl, hi_sl); end loop; -- alt'
    hblank(HBL);                                                                   -- 16 impulses
    for q in 0 to 15 loop send_block(std_logic_vector(shift_left(to_unsigned(1,16), q)), lo_sl, hi_sl); end loop;
    hblank(HBL); for r in 0 to 15 loop send_block(walsh_sig(r,false), lo_sl, hi_sl); end loop; -- Walsh +
    hblank(HBL); for r in 0 to 15 loop send_block(walsh_sig(r,true),  lo_sl, hi_sl); end loop; -- Walsh -
    report "PHASE A (rails 0/1023, clamped oracle) done: "
           &integer'image(compares)&" active compares, e_y="&integer'image(e_y) severity note;

    ------------------------------------------------------------------ Phase B
    -- EXHAUSTIVE 65536 vertex blocks at clamp-legal rails {G_LO,G_HI}: bit-exact
    -- identity, no clamp masking. The strongest hook-#1 assertion.
    lo_sl := std_logic_vector(to_unsigned(G_LO, 10));
    hi_sl := std_logic_vector(to_unsigned(G_HI, 10));
    for base in 0 to SWEEP_MAX loop
      if (base mod BPL) = 0 then hblank(HBL); end if;
      if (base mod 8192) = 0 then
        report "  sweep progress: block 0x"
               &to_hstring(std_logic_vector(to_unsigned(base,16)))
               &" (e_y="&integer'image(e_y)&")" severity note;
      end if;
      send_block(std_logic_vector(to_unsigned(base,16)), lo_sl, hi_sl);
    end loop;

    -- drain the last block through the pipeline
    hblank(HBL);

    report "PHASE B (exhaustive "&integer'image(SWEEP_MAX+1)
           &" vertex blocks @ {"&integer'image(G_LO)&","&integer'image(G_HI)
           &"}) done" severity note;
    report "RESULT HOOK1_identity_adv : "&str_pf(e_y=0)
           &" ("&integer'image(e_y)&" Y mismatches / "
           &integer'image(compares)&" active compares)" severity note;
    report "RESULT HOOK5_uvpass_adv   : "&str_pf(e_uv=0)
           &" ("&integer'image(e_uv)&" U/V mismatches)" severity note;
    report "RESULT HOOK7_sync_adv     : "&str_pf(e_sync=0)
           &" ("&integer'image(e_sync)&" sync-delay mismatches)" severity note;
    report "VIT_DONE identity_adv" severity note;
    assert e_y=0 and e_uv=0 and e_sync=0
      report "ADVERSARIAL IDENTITY TB FAILED" severity failure;
    done <= true;
    wait;
  end process;
end architecture tb;
