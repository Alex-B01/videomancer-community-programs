-- Copyright (C) 2026 Sourdough Stu
-- SPDX-License-Identifier: GPL-3.0-only
--
-- File: cbv_m1_rotate.vhd - CBV Composite M1 rotation with sine LUT (block 7, spec §5.11)
--
-- This file is free software: you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation, either version 3 of the License, or
-- (at your option) any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
-- GNU General Public License for more details.
--
-- You should have received a copy of the GNU General Public License
-- along with this program. If not, see <https://www.gnu.org/licenses/>.
--
-- Credits: Syntonie CBV001 Rev C schematic (CC BY-SA 4.0), Phil Baljeu's
-- Archer trace. Behavioural model only; no schematic artwork is copied.
--
-- phi = (APC_ph + alpha_d + theta) mod 1024 (integer part of APC_ph's 10.8),
-- alpha_d = 0 (slider ignored) and theta = 0 (Hue Bend off) in Stage 1, so
-- phi is latched once per line at this block's x = 0 (R-LATCH; APC result of
-- line l-1). c = cos(phi), s = sin(phi) from the quarter-wave ROM (s12,
-- 1.0 = 2047; PROVISIONAL Q7), read twice per line through its single port.
--   uu = (U4 c + V4 s) >> 11,  vq = (V4 c - U4 s) >> 11
-- One inferred multiplier (18 x 12), time-shared over the 4 clocks of each /4
-- sample: slots U4*c, V4*s, V4*c, U4*s.
--
-- Pipeline (e0 = U4 loaded at stage 24): e1-e4 operand registers, e2-e5
--   products, e3-e6 accumulate, outputs registered at e6 (stage 30). Latency 6.
-- EBR map #11: sine quarter-wave ROM 256 x 12 -> 1 EBR (256x16).
-- Resources (paper): ~400 LC incl. LUT logic and phase; 1 multiplier.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cbv_pkg.all;

entity cbv_m1_rotate is
  port (
    clk      : in  std_logic;
    i_u4     : in  signed(17 downto 0);     -- stage 24
    i_v4     : in  signed(17 downto 0);
    i_valid  : in  std_logic;               -- stage 24
    i_sol    : in  std_logic;               -- this block's x = 0 (stage 24)
    i_apc_ph : in  unsigned(17 downto 0);
    o_uu     : out signed(18 downto 0);     -- stage 30
    o_vq     : out signed(18 downto 0);
    o_valid  : out std_logic;
    o_cos    : out signed(11 downto 0);     -- line coefficients (also used by the engine)
    o_sin    : out signed(11 downto 0)
  );
end entity cbv_m1_rotate;

architecture rtl of cbv_m1_rotate is
  type t_rom is array (0 to 255) of std_logic_vector(11 downto 0);
  function f_rom_init return t_rom is
    variable r : t_rom;
  begin
    for k in 0 to 255 loop
      r(k) := std_logic_vector(to_unsigned(C_SIN_Q(k), 12));
    end loop;
    return r;
  end function;
  constant C_ROM : t_rom := f_rom_init;   -- 1 EBR (256x16)

  signal s_rom_a : unsigned(7 downto 0) := (others => '0');
  signal s_rom_q : std_logic_vector(11 downto 0) := (others => '0');
  signal s_phi   : unsigned(9 downto 0) := (others => '0');
  signal s_lstep : unsigned(2 downto 0) := (others => '1');
  signal s_neg1, s_one1, s_neg2, s_one2 : std_logic := '0';
  signal s_negq, s_oneq : std_logic := '0';
  signal s_cos   : signed(11 downto 0) := to_signed(2047, 12);
  signal s_sin   : signed(11 downto 0) := (others => '0');

  -- frame
  signal s_ph    : unsigned(2 downto 0) := (others => '1');
  signal s_ma    : signed(17 downto 0) := (others => '0');
  signal s_mb    : signed(11 downto 0) := (others => '0');
  signal s_slot1 : unsigned(2 downto 0) := (others => '1');
  signal s_prod  : signed(29 downto 0) := (others => '0');
  signal s_slot2 : unsigned(2 downto 0) := (others => '1');
  signal s_au    : signed(29 downto 0) := (others => '0');
  signal s_av    : signed(29 downto 0) := (others => '0');
  signal s_uu_f  : signed(30 downto 0) := (others => '0');
  signal s_uu, s_vq : signed(18 downto 0) := (others => '0');
  signal s_ov    : std_logic := '0';

  -- quarter-wave addressing for sin(p): returns address; neg / one flags
  procedure p_qaddr(p : in unsigned(9 downto 0); a : out unsigned(7 downto 0);
                    neg : out std_logic; one : out std_logic) is
    variable i : unsigned(7 downto 0);
  begin
    i := p(7 downto 0);
    neg := p(9);
    one := '0';
    if p(8) = '0' then
      a := i;
    else
      if i = 0 then
        one := '1';
        a := (others => '0');
      else
        a := (not i) + 1;          -- 256 - i
      end if;
    end if;
  end procedure;

begin

  -- sine ROM, registered read (EBR)
  p_rom : process(clk)
  begin
    if rising_edge(clk) then
      s_rom_q <= C_ROM(to_integer(s_rom_a));
    end if;
  end process;

  -- per-line phase and coefficient fetch
  p_coef : process(clk)
    variable v_a   : unsigned(7 downto 0);
    variable v_neg, v_one : std_logic;
    variable v_val : signed(11 downto 0);
  begin
    if rising_edge(clk) then
      if i_sol = '1' then
        s_phi   <= i_apc_ph(17 downto 8);
        s_lstep <= (others => '0');
      elsif s_lstep /= "111" then
        s_lstep <= s_lstep + 1;
      end if;
      case to_integer(s_lstep) is
        when 0 =>        -- address sin(phi)
          p_qaddr(s_phi, v_a, v_neg, v_one);
          s_rom_a <= v_a; s_neg1 <= v_neg; s_one1 <= v_one;
        when 1 =>        -- address cos(phi) = sin(phi + 256)
          p_qaddr(s_phi + 256, v_a, v_neg, v_one);
          s_rom_a <= v_a; s_neg2 <= v_neg; s_one2 <= v_one;
        when others => null;
      end case;
      -- ROM data: sin at lstep 2, cos at lstep 3
      if s_lstep = 2 or s_lstep = 3 then
        if (s_lstep = 2 and s_one1 = '1') or (s_lstep = 3 and s_one2 = '1') then
          v_val := to_signed(2047, 12);
        else
          v_val := signed(s_rom_q);
        end if;
        if (s_lstep = 2 and s_neg1 = '1') or (s_lstep = 3 and s_neg2 = '1') then
          v_val := -v_val;
        end if;
        if s_lstep = 2 then
          s_sin <= v_val;
        else
          s_cos <= v_val;
        end if;
      end if;
    end if;
  end process;

  -- time-shared multiplier, 4 slots per /4 sample
  p_mul : process(clk)
  begin
    if rising_edge(clk) then
      -- slot sequencer: issue at e1 (valid seen) .. e4; U4/V4 are read before
      -- the next valid can reload them (valid spacing >= 4 clocks)
      s_slot1 <= "111";
      if i_valid = '1' then
        s_ma <= i_u4; s_mb <= s_cos; s_slot1 <= "000"; s_ph <= "001";
      else
        case to_integer(s_ph) is
          when 1 => s_ma <= i_v4; s_mb <= s_sin; s_slot1 <= "001"; s_ph <= "010";
          when 2 => s_ma <= i_v4; s_mb <= s_cos; s_slot1 <= "010"; s_ph <= "011";
          when 3 => s_ma <= i_u4; s_mb <= s_sin; s_slot1 <= "011"; s_ph <= "111";
          when others => null;
        end case;
      end if;
      -- product (e2..e5)
      s_prod  <= s_ma * s_mb;
      s_slot2 <= s_slot1;
      -- accumulate (e3..e6); outputs registered with the last slot (e6 = stage 30)
      s_ov <= '0';
      case to_integer(s_slot2) is
        when 0 => s_au <= s_prod;
        when 1 => s_uu_f <= resize(s_au, 31) + resize(s_prod, 31);
        when 2 => s_av <= s_prod;
        when 3 =>
          s_uu <= resize(shift_right(s_uu_f, 11), 19);
          s_vq <= resize(shift_right(resize(s_av, 31) - resize(s_prod, 31), 11), 19);
          s_ov <= '1';
        when others => null;
      end case;
    end if;
  end process;

  o_uu    <= s_uu;
  o_vq    <= s_vq;
  o_valid <= s_ov;
  o_cos   <= s_cos;
  o_sin   <= s_sin;

end architecture rtl;
