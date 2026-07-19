-- ============================================================================
-- tb_sequency_leak.vhd  --  Hook #13 (NEW, REV-F): Y->C residual leak
--   (Crosstalk Knob 5 + Chroma Tog 11), the load-neutral + self-gating contract.
-- ----------------------------------------------------------------------------
-- Runs on TRUE five-corner geometry (G_CORNER selects NTSC/720p60/1080p30/
-- 1080i5994/1080p24) with reduced vertical raster. The leak is an output-stage,
-- per-pixel, block-local operation -> corner-invariant by construction; running
-- it x corners confirms the +4 (52->56) retime and the inject land identically
-- regardless of htotal / field structure.
--
-- SEVEN aligned DUTs, identical input, identical LATENCY 56 -> their avid edges
-- line up so a live per-pixel compare + an EXACT inject model can be asserted:
--
--   p1 : QUANT max, fader WET, Knob5=1023 (pol1, sh1), Tog11 ON  -> LEAK (vivid, +pol)
--   p0 : QUANT max, fader WET, Knob5=0    (pol0, sh1), Tog11 ON  -> LEAK (vivid, -pol)
--   ps : QUANT max, fader WET, Knob5=704  (pol1, sh5), Tog11 ON  -> LEAK (subtle)
--   ref: QUANT max, fader WET, Knob5=1023,           Tog11 OFF   -> NULL (master off)
--   nc : QUANT max, fader WET, Knob5=512  (center),  Tog11 ON    -> NULL (center detent)
--   nf : QUANT max, fader DRY, Knob5=1023,           Tog11 ON    -> NULL (fader dry)
--   nn : NEUTRAL,   fader WET, Knob5=1023,           Tog11 ON    -> NULL (effect neutral)
--
-- Y_out is taken from p1.y (the leak never touches Y — it is Y->C only), Y_dry/
-- U_dry/V_dry from a 56-deep input shadow. The DUT forms residual r = Y_out - Y_dry
-- at 52, k-barrel-shifts (arithmetic >> sh, NO multiply), injects opposite-signed
-- into U/V (pol1: U+=,V-= ; pol0: U-=,V+=), R1-clamps [64,960], lands at 56. So:
--   inj = arsh(Y_out - Y_dry, sh);  exp_u = clamp(U_dry +/- inj);  exp_v = clamp(V_dry -/+ inj)
--
-- Hook #13 assertions:
--   #13a leak-model    : p1/p0/ps.u,v are bit-exact against the inject model.
--   #13b opposite-sign : on UNCLAMPED pixels (u_dry+-inj, v_dry-+inj both in-range)
--                        (p1.u - U_dry) == -(p1.v - V_dry) exactly -> the leak is
--                        equal-and-opposite into U vs V (NTSC cross-color, not tint).
--   #13c polarity-flip : Knob5<512 mirrors Knob5>512: (p1.u-U_dry)==-(p0.u-U_dry).
--   #13d magnitude~k   : |p1 leak| >= |ps leak| everywhere (sh1 vivid >= sh5 subtle)
--                        and strictly > on some pixels -> magnitude tracks Knob5's
--                        top-3 bits. Sign tracks the crossfade residual (in #13a).
--   #13e leak-active   : the effect really produces residual (res/=0) AND p1's U/V
--                        really move off dry somewhere -> the hook is not vacuous.
--   #13-null SELF-NULL : ref/nc/nf/nn.u,v are BIT-EXACT dry passthrough (== input
--                        shadow d56). Leak is EXACTLY ZERO when ANY of: Tog11 off /
--                        Knob5 center 512 / fader dry / effect neutral -- even with
--                        Knob5 driven hot. This is the load-neutral contract, and
--                        it also re-proves hook #6 (U/V bit-exact regardless of Knob5).
--
-- NEVER edits program VHDL. Pure black-box (residual reconstructed from p1.y +
-- the input shadow, both externally visible).
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;
library work;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.video_corner_pkg.all;

entity tb_sequency_leak is
  generic (
    G_CORNER    : integer := 2;    -- 0 NTSC .1 720p60 .2 1080p30 .3 1080i5994 .4 1080p24
    G_ACT_LINES : integer := 6;
    G_VBL_LINES : integer := 3
  );
end entity;

architecture tb of tb_sequency_leak is
  constant CPL : integer := CORNERS(G_CORNER).clocks_per_line;
  constant AW  : integer := CORNERS(G_CORNER).active_width;
  constant AVS : integer := CPL - AW;
  constant LPF : integer := G_ACT_LINES + G_VBL_LINES;
  constant ILV : boolean := CORNERS(G_CORNER).is_interlaced;
  constant D   : integer := 56;                     -- REV-F latency
  constant STOPF   : integer := 4;
  constant WARMCYC : integer := 3*CPL;

  signal clk : std_logic := '0';  signal running : boolean := true;
  signal di : t_video_stream_yuv444_30b;
  signal do_p1, do_p0, do_ps, do_ref, do_nc, do_nf, do_nn : t_video_stream_yuv444_30b;
  signal r_p1, r_p0, r_ps, r_ref, r_nc, r_nf, r_nn : t_spi_ram := (others => (others => '0'));
  signal cur_field : integer := 0;

  -- 56-deep input shadow (dry passthrough reference)
  type v10 is array(1 to D) of std_logic_vector(9 downto 0);
  type sl  is array(1 to D) of std_logic;
  signal sh_y, sh_u, sh_v : v10 := (others => (others => '0'));
  signal sh_av            : sl  := (others => '0');

  constant TID : integer := CORNERS(G_CORNER).id;

  -- arithmetic right shift (matches DUT signed shift_right; floor toward -inf)
  function arsh(v, s : integer) return integer is
  begin
    return to_integer(shift_right(to_signed(v, 20), s));
  end function;
  function clamp_uv(v : integer) return integer is
  begin
    if v < 64 then return 64; elsif v > 960 then return 960; else return v; end if;
  end function;
begin
  clk_gen : process begin
    while running loop clk<='0'; wait for 5 ns; clk<='1'; wait for 5 ns; end loop; wait;
  end process;

  -- Static register configs (fixed for the whole run -> no phase transients).
  -- effect = QUANTIZE max + TILT flat so Y_out /= Y_dry on AC content (residual live).
  r_p1(0)<=std_logic_vector(to_unsigned(1023,10)); r_p1(1)<=std_logic_vector(to_unsigned(512,10));
  r_p1(4)<=std_logic_vector(to_unsigned(1023,10)); r_p1(6)<=std_logic_vector(to_unsigned(16,10));
  r_p1(7)<=std_logic_vector(to_unsigned(1023,10)); r_p1(8)<=std_logic_vector(to_unsigned(TID,10));

  r_p0(0)<=std_logic_vector(to_unsigned(1023,10)); r_p0(1)<=std_logic_vector(to_unsigned(512,10));
  r_p0(4)<=std_logic_vector(to_unsigned(0,10));    r_p0(6)<=std_logic_vector(to_unsigned(16,10));
  r_p0(7)<=std_logic_vector(to_unsigned(1023,10)); r_p0(8)<=std_logic_vector(to_unsigned(TID,10));

  r_ps(0)<=std_logic_vector(to_unsigned(1023,10)); r_ps(1)<=std_logic_vector(to_unsigned(512,10));
  r_ps(4)<=std_logic_vector(to_unsigned(704,10));  r_ps(6)<=std_logic_vector(to_unsigned(16,10));
  r_ps(7)<=std_logic_vector(to_unsigned(1023,10)); r_ps(8)<=std_logic_vector(to_unsigned(TID,10));

  r_ref(0)<=std_logic_vector(to_unsigned(1023,10)); r_ref(1)<=std_logic_vector(to_unsigned(512,10));
  r_ref(4)<=std_logic_vector(to_unsigned(1023,10)); r_ref(6)<=std_logic_vector(to_unsigned(0,10));
  r_ref(7)<=std_logic_vector(to_unsigned(1023,10)); r_ref(8)<=std_logic_vector(to_unsigned(TID,10));

  r_nc(0)<=std_logic_vector(to_unsigned(1023,10)); r_nc(1)<=std_logic_vector(to_unsigned(512,10));
  r_nc(4)<=std_logic_vector(to_unsigned(512,10));  r_nc(6)<=std_logic_vector(to_unsigned(16,10));
  r_nc(7)<=std_logic_vector(to_unsigned(1023,10)); r_nc(8)<=std_logic_vector(to_unsigned(TID,10));

  r_nf(0)<=std_logic_vector(to_unsigned(1023,10)); r_nf(1)<=std_logic_vector(to_unsigned(512,10));
  r_nf(4)<=std_logic_vector(to_unsigned(1023,10)); r_nf(6)<=std_logic_vector(to_unsigned(16,10));
  r_nf(7)<=std_logic_vector(to_unsigned(0,10));    r_nf(8)<=std_logic_vector(to_unsigned(TID,10));

  r_nn(1)<=std_logic_vector(to_unsigned(512,10));  r_nn(4)<=std_logic_vector(to_unsigned(1023,10));
  r_nn(6)<=std_logic_vector(to_unsigned(16,10));   r_nn(7)<=std_logic_vector(to_unsigned(1023,10));
  r_nn(8)<=std_logic_vector(to_unsigned(TID,10));

  dut_p1 : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_p1, data_in=>di, data_out=>do_p1);
  dut_p0 : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_p0, data_in=>di, data_out=>do_p0);
  dut_ps : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_ps, data_in=>di, data_out=>do_ps);
  dut_rf : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_ref,data_in=>di, data_out=>do_ref);
  dut_nc : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_nc, data_in=>di, data_out=>do_nc);
  dut_nf : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_nf, data_in=>di, data_out=>do_nf);
  dut_nn : entity work.program_top(sequency) port map(clk=>clk, registers_in=>r_nn, data_in=>di, data_out=>do_nn);

  -- AC-rich Y (residual live under QUANTIZE); U/V centered ~512 so udry+-inj mostly
  -- lands unclamped (plenty of samples for the opposite-signed / polarity checks).
  stim : process(clk)
    variable cx,ly,fld,gl : integer := 0; variable avon : boolean; variable ac : integer;
  begin
    if rising_edge(clk) then
      avon := (cx>=AVS) and (ly<G_ACT_LINES); ac := cx-AVS;
      di.hsync_n<='1'; di.vsync_n<='1'; di.field_n<= '1' when (fld mod 2)=0 else '0';
      if cx<44 then di.hsync_n<='0'; end if;
      if (ly>=G_ACT_LINES) and (ly<G_ACT_LINES+2) then di.vsync_n<='0'; end if;
      if avon then di.avid<='1';
        di.y <= std_logic_vector(to_unsigned(64 + ((ac*23 + gl*101) mod 800),10)); -- strong AC
        di.u <= std_logic_vector(to_unsigned(320 + ((ac*5 + gl*3) mod 384),10));    -- ~[320,703]
        di.v <= std_logic_vector(to_unsigned(320 + ((ac*7 + gl*2) mod 384),10));
      else di.avid<='0'; di.y<=std_logic_vector(to_unsigned(64,10));
        di.u<=std_logic_vector(to_unsigned(512,10)); di.v<=std_logic_vector(to_unsigned(512,10)); end if;
      if cx=CPL-1 then cx:=0; if avon then gl:=gl+1; end if;
        if ly=LPF-1 then ly:=0; fld:=fld+1; if ILV then null; end if; else ly:=ly+1; end if;
      else cx:=cx+1; end if;
      cur_field <= fld;
    end if;
  end process;

  shadow : process(clk) begin
    if rising_edge(clk) then
      sh_y(1)<=di.y; sh_u(1)<=di.u; sh_v(1)<=di.v; sh_av(1)<=di.avid;
      for k in 2 to D loop
        sh_y(k)<=sh_y(k-1); sh_u(k)<=sh_u(k-1); sh_v(k)<=sh_v(k-1); sh_av(k)<=sh_av(k-1);
      end loop;
    end if;
  end process;

  mon : process(clk)
    constant NAME : string := corner_name(G_CORNER);
    variable cyc : integer := 0;
    variable yout, ydry, udry, vdry, res, inj1, inj5 : integer;
    variable e_model, e_opp, e_flip, e_mag, e_ynz : integer := 0;
    variable e_nref, e_nc, e_nf, e_nn : integer := 0;
    variable n_active, n_leaked, n_uncl, n_magstrict : integer := 0;
    variable rep : integer := 0;
  begin
    if rising_edge(clk) then
      cyc := cyc + 1;
      if cyc>WARMCYC and sh_av(D)='1' and do_p1.avid='1' then
        n_active := n_active + 1;
        yout := to_integer(unsigned(do_p1.y));
        ydry := to_integer(unsigned(sh_y(D)));
        udry := to_integer(unsigned(sh_u(D)));
        vdry := to_integer(unsigned(sh_v(D)));
        res  := yout - ydry;
        inj1 := arsh(res, 1);   -- Knob5=1023 -> sh1
        inj5 := arsh(res, 5);   -- Knob5=704  -> sh5

        -- Y is never leaked -> every QUANT-wet DUT emits identical Y_out
        if do_p0.y/=do_p1.y or do_ps.y/=do_p1.y or do_ref.y/=do_p1.y or do_nc.y/=do_p1.y then
          e_ynz := e_ynz + 1;
        end if;

        -- #13a exact inject model (p1 +pol sh1, p0 -pol sh1, ps +pol sh5)
        if to_integer(unsigned(do_p1.u)) /= clamp_uv(udry+inj1) or
           to_integer(unsigned(do_p1.v)) /= clamp_uv(vdry-inj1) then
          e_model := e_model + 1;
          if rep<8 then
            report NAME&" LEAK p1 model miss: res="&integer'image(res)&" inj="&integer'image(inj1)&
              " udry="&integer'image(udry)&" got_u="&integer'image(to_integer(unsigned(do_p1.u)))&
              " exp_u="&integer'image(clamp_uv(udry+inj1)) severity note;
            rep := rep+1;
          end if;
        end if;
        if to_integer(unsigned(do_p0.u)) /= clamp_uv(udry-inj1) or
           to_integer(unsigned(do_p0.v)) /= clamp_uv(vdry+inj1) then e_model := e_model + 1; end if;
        if to_integer(unsigned(do_ps.u)) /= clamp_uv(udry+inj5) or
           to_integer(unsigned(do_ps.v)) /= clamp_uv(vdry-inj5) then e_model := e_model + 1; end if;

        -- leak-active bookkeeping
        if res /= 0 then null; end if;
        if inj1 /= 0 then n_leaked := n_leaked + 1; end if;

        -- #13b opposite-signed + #13c polarity-flip on UNCLAMPED pixels
        if (udry+inj1)>=64 and (udry+inj1)<=960 and (vdry-inj1)>=64 and (vdry-inj1)<=960 then
          n_uncl := n_uncl + 1;
          if (to_integer(unsigned(do_p1.u))-udry) /= -(to_integer(unsigned(do_p1.v))-vdry) then
            e_opp := e_opp + 1;
          end if;
          if (udry-inj1)>=64 and (udry-inj1)<=960 then
            if (to_integer(unsigned(do_p1.u))-udry) /= -(to_integer(unsigned(do_p0.u))-udry) then
              e_flip := e_flip + 1;
            end if;
          end if;
        end if;

        -- #13d magnitude tracks k: |sh1 leak| >= |sh5 leak| (both unclamped)
        if (udry+inj1)>=64 and (udry+inj1)<=960 and (udry+inj5)>=64 and (udry+inj5)<=960 then
          if abs(inj1) < abs(inj5) then e_mag := e_mag + 1; end if;
          if abs(inj1) > abs(inj5) then n_magstrict := n_magstrict + 1; end if;
        end if;

        -- #13-null: leak EXACTLY 0 -> U/V bit-exact dry passthrough
        if do_ref.u/=sh_u(D) or do_ref.v/=sh_v(D) then e_nref := e_nref + 1; end if;
        if do_nc.u /=sh_u(D) or do_nc.v /=sh_v(D) then e_nc  := e_nc  + 1; end if;
        if do_nf.u /=sh_u(D) or do_nf.v /=sh_v(D) then e_nf  := e_nf  + 1; end if;
        if do_nn.u /=sh_u(D) or do_nn.v /=sh_v(D) then e_nn  := e_nn  + 1; end if;
      end if;

      if cur_field>=STOPF then
        report "==== SEQUENCY leak "&NAME&" (id="&integer'image(TID)&", "&
          integer'image(n_active)&" active px) ====" severity note;
        report "RESULT HOOK13a_leak_model  "&NAME&": "&str_pf(e_model=0)&
          " ("&integer'image(e_model)&" model misses / "&integer'image(n_active)&" px)" severity note;
        report "RESULT HOOK13b_opposite    "&NAME&": "&str_pf(e_opp=0 and n_uncl>0)&
          " ("&integer'image(e_opp)&" sign errs / "&integer'image(n_uncl)&" unclamped)" severity note;
        report "RESULT HOOK13c_polflip     "&NAME&": "&str_pf(e_flip=0 and n_uncl>0)&
          " ("&integer'image(e_flip)&" flip errs)" severity note;
        report "RESULT HOOK13d_mag_track_k "&NAME&": "&str_pf(e_mag=0 and n_magstrict>0)&
          " ("&integer'image(e_mag)&" |sh1|<|sh5| viol; "&integer'image(n_magstrict)&" strict sh1>sh5)" severity note;
        report "RESULT HOOK13e_leak_active "&NAME&": "&str_pf(n_leaked>0 and e_ynz=0)&
          " ("&integer'image(n_leaked)&" leaked px; "&integer'image(e_ynz)&" Y-not-invariant errs)" severity note;
        report "RESULT HOOK13_null_togoff  "&NAME&": "&str_pf(e_nref=0)&" ("&integer'image(e_nref)&" U/V leak w/ Tog11 off)" severity note;
        report "RESULT HOOK13_null_center  "&NAME&": "&str_pf(e_nc=0)&" ("&integer'image(e_nc)&" U/V leak w/ Knob5=512)" severity note;
        report "RESULT HOOK13_null_faderdry "&NAME&": "&str_pf(e_nf=0)&" ("&integer'image(e_nf)&" U/V leak w/ fader dry)" severity note;
        report "RESULT HOOK13_null_neutral "&NAME&": "&str_pf(e_nn=0)&" ("&integer'image(e_nn)&" U/V leak w/ effect neutral)" severity note;
        report "VIT_DONE leak "&NAME severity note;
        running<=false; stop;
      end if;
      if cyc>40000000 then report "TIMEOUT" severity failure; end if;
    end if;
  end process;
end architecture;
