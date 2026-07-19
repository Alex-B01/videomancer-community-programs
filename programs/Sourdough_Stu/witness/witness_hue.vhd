-- Copyright (C) 2026 AB (Sourdough_Stu)
-- SPDX-License-Identifier: GPL-3.0-only
--
-- witness_hue.vhd - hue-octant approximator (born in WITNESS; REV-B / OQ-2, D8)
--
-- Per active pixel: combinational U/V sign & magnitude compares -> a 3-bit hue
-- octant + an in-octant fraction + a chroma-saturation magnitude
--     sat ~= |U-512| + |V-512|
-- ZERO EBR, ZERO multiply (adds / compares / shifts only). The moments mass
-- weight becomes  w = base + (sat >> C_SAT_SHIFT)  (self-nulling on neutral
-- content) so the gaze also drifts toward the most-saturated region. The 3-bit
-- octant is wired and reserved for the flagged target-hue upgrade (§4.5 / M3);
-- unused in the always-on default. Outputs registered (one 15-FF stage).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity witness_hue is
  port (
    clk    : in  std_logic;
    u      : in  unsigned(9 downto 0);   -- chroma U (offset-512 encoded)
    v      : in  unsigned(9 downto 0);   -- chroma V (offset-512 encoded)
    octant : out unsigned(2 downto 0);   -- 0..7 hue octant (reserved: target-hue upgrade)
    frac   : out unsigned(9 downto 0);   -- in-octant fraction proxy (min magnitude)
    sat    : out unsigned(10 downto 0)   -- |U-512| + |V-512|  (0..1022)
  );
end entity witness_hue;

architecture rtl of witness_hue is
  constant C_MID : signed(11 downto 0) := to_signed(512, 12);
begin

  p_hue : process (clk)
    variable v_du     : signed(11 downto 0);
    variable v_dv     : signed(11 downto 0);
    variable v_ndu    : signed(11 downto 0);
    variable v_ndv    : signed(11 downto 0);
    variable v_au     : unsigned(10 downto 0);   -- |du|
    variable v_av     : unsigned(10 downto 0);   -- |dv|
    variable v_udom   : std_logic;               -- '1' when |du| >= |dv| (U dominates)
  begin
    if rising_edge(clk) then
      v_du := signed(resize(u, 12)) - C_MID;
      v_dv := signed(resize(v, 12)) - C_MID;
      v_ndu := -v_du;
      v_ndv := -v_dv;

      if v_du >= 0 then v_au := unsigned(v_du(10 downto 0));
      else              v_au := unsigned(v_ndu(10 downto 0)); end if;
      if v_dv >= 0 then v_av := unsigned(v_dv(10 downto 0));
      else              v_av := unsigned(v_ndv(10 downto 0)); end if;

      if v_au >= v_av then v_udom := '1'; else v_udom := '0'; end if;

      -- 8-octant code: { sign(du), sign(dv), V-dominates }
      octant <= v_du(11) & v_dv(11) & (not v_udom);

      -- saturation magnitude (L1 chroma distance from neutral)
      sat    <= resize(v_au, 11) + resize(v_av, 11);

      -- fraction proxy = the SMALLER magnitude (WITNESS ignores it by default)
      if v_udom = '1' then frac <= resize(v_av, 10);
      else                 frac <= resize(v_au, 10); end if;
    end if;
  end process p_hue;

end architecture rtl;
