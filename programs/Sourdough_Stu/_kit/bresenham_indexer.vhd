-- Copyright (C) 2026 AB (Sourdough_Stu)
-- SPDX-License-Identifier: GPL-3.0-only
--
-- bresenham_indexer.vhd - division-free G_BLOCKS_X x G_BLOCKS_Y raster tiler
--                         (born in WITNESS; shared-kit, promotion-ready)
--
-- Purpose:
--   Maps an arbitrary active raster (measured live, never hard-coded) onto an
--   exact G_BLOCKS_X x G_BLOCKS_Y cell grid using two Bresenham/DDA error
--   accumulators -- NO divider, NO multiplier. First of six planned consumers
--   (progs 1/2/7/8/10/12); factored general.
--
-- Mechanism (self-calibrating):
--   * Column: err_x += G_BLOCKS_X each active pixel; when err_x >= active_width
--     subtract active_width and block_x++. Over one active line this yields
--     block_x(p) = floor(p * G_BLOCKS_X / active_width), i.e. block_x sweeps
--     0..G_BLOCKS_X-1 exactly, whatever the width (720/1280/1920/...).
--   * Row: err_y += G_BLOCKS_Y each active line; when err_y >= active_height
--     subtract active_height and block_y++. block_y sweeps 0..G_BLOCKS_Y-1.
--   * active_width  is COUNTED from avid rising..falling and latched at the end
--     of each active line;  active_height is the active-line count latched at
--     the frame boundary (vsync). No 2048 literal anywhere.
--
-- Seeding: active_width/active_height initialise to sane HD defaults (signal
--   initialisation only -- iCE40 has no reset) so line 0 of the very first
--   frame never divides against zero; both self-correct after one line/frame.
--
-- Outputs:
--   block_x (0..G_BLOCKS_X-1), block_y (0..G_BLOCKS_Y-1),
--   cell_addr = block_y*G_BLOCKS_X + block_x (concatenation, no multiply when
--     G_BLOCKS_X is a power of two -- 64 here -> block_y & block_x),
--   block_x_first / block_x_last strobes (avid rising / falling),
--   frac_x / frac_y = in-cell error remainder (sub-block fraction; WITNESS
--     ignores it -- nearest-neighbour). All outputs registered.
--
-- Resource: 2 error accumulators + 2 measure counters + control. 0 EBR, 0 mult.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity bresenham_indexer is
  generic (
    G_BLOCKS_X   : positive := 64;   -- must be a power of two for the concat cell_addr
    G_BLOCKS_Y   : positive := 32;   -- must be a power of two for the concat cell_addr
    G_ACC_WIDTH  : positive := 13;   -- covers active_width + G_BLOCKS_X (1920+64 < 8192)
    G_SEED_WIDTH : positive := 1920; -- initial active_width  (self-corrects after line 0)
    G_SEED_HEIGHT: positive := 1080  -- initial active_height (self-corrects after frame 0)
  );
  port (
    clk           : in  std_logic;
    avid          : in  std_logic;                    -- active-video (high = active pixel)
    hsync_n       : in  std_logic;                    -- active-low hsync (unused edges reserved)
    vsync_n       : in  std_logic;                    -- active-low vsync (frame boundary)
    block_x       : out unsigned(15 downto 0);        -- 0 .. G_BLOCKS_X-1
    block_y       : out unsigned(15 downto 0);        -- 0 .. G_BLOCKS_Y-1
    cell_addr     : out unsigned(15 downto 0);        -- block_y*G_BLOCKS_X + block_x
    block_x_first : out std_logic;                    -- strobe: first active pixel of line
    block_x_last  : out std_logic;                    -- strobe: last active pixel of line
    frac_x        : out unsigned(G_ACC_WIDTH - 1 downto 0);
    frac_y        : out unsigned(G_ACC_WIDTH - 1 downto 0)
  );
end entity bresenham_indexer;

architecture rtl of bresenham_indexer is

  constant C_BX : unsigned(G_ACC_WIDTH - 1 downto 0) := to_unsigned(G_BLOCKS_X, G_ACC_WIDTH);
  constant C_BY : unsigned(G_ACC_WIDTH - 1 downto 0) := to_unsigned(G_BLOCKS_Y, G_ACC_WIDTH);

  -- edge detect
  signal s_avid_d    : std_logic := '0';
  signal s_vsync_n_d : std_logic := '1';

  -- live-measured raster geometry (seeded, self-calibrating)
  signal s_active_w  : unsigned(G_ACC_WIDTH - 1 downto 0) := to_unsigned(G_SEED_WIDTH,  G_ACC_WIDTH);
  signal s_active_h  : unsigned(G_ACC_WIDTH - 1 downto 0) := to_unsigned(G_SEED_HEIGHT, G_ACC_WIDTH);
  signal s_wcount    : unsigned(G_ACC_WIDTH - 1 downto 0) := (others => '0'); -- active px this line
  signal s_hcount    : unsigned(G_ACC_WIDTH - 1 downto 0) := (others => '0'); -- active lines this frame

  -- error accumulators + block indices
  signal s_err_x     : unsigned(G_ACC_WIDTH - 1 downto 0) := (others => '0');
  signal s_err_y     : unsigned(G_ACC_WIDTH - 1 downto 0) := (others => '0');
  signal s_block_x   : unsigned(15 downto 0) := (others => '0');
  signal s_block_y   : unsigned(15 downto 0) := (others => '0');

begin

  p_index : process (clk)
    variable v_avid_rise : boolean;
    variable v_avid_fall : boolean;
    variable v_vsync_act : boolean;   -- vsync_n falling edge = frame boundary
    variable v_err_x     : unsigned(G_ACC_WIDTH - 1 downto 0);
    variable v_err_y     : unsigned(G_ACC_WIDTH - 1 downto 0);
  begin
    if rising_edge(clk) then
      v_avid_rise := (avid = '1') and (s_avid_d = '0');
      v_avid_fall := (avid = '0') and (s_avid_d = '1');
      v_vsync_act := (vsync_n = '0') and (s_vsync_n_d = '1');

      s_avid_d      <= avid;
      s_vsync_n_d   <= vsync_n;
      block_x_first <= '0';
      block_x_last  <= '0';

      ------------------------------------------------------------------
      -- Frame boundary: latch measured active_height, reset row DDA.
      ------------------------------------------------------------------
      if v_vsync_act then
        if s_hcount /= 0 then
          s_active_h <= s_hcount;   -- self-calibrate to whatever we measured
        end if;
        s_hcount   <= (others => '0');
        s_err_y    <= (others => '0');
        s_block_y  <= (others => '0');
      end if;

      ------------------------------------------------------------------
      -- Start of an active line: reset column DDA, advance row DDA.
      ------------------------------------------------------------------
      if v_avid_rise then
        s_err_x       <= (others => '0');
        s_block_x     <= (others => '0');
        s_wcount      <= (others => '0');
        block_x_first <= '1';

        -- row DDA: one step per active line
        v_err_y := s_err_y + C_BY;
        if v_err_y >= s_active_h then
          s_err_y   <= v_err_y - s_active_h;
          if s_block_y < to_unsigned(G_BLOCKS_Y - 1, s_block_y'length) then
            s_block_y <= s_block_y + 1;
          end if;
        else
          s_err_y <= v_err_y;
        end if;
        s_hcount <= s_hcount + 1;
      end if;

      ------------------------------------------------------------------
      -- Active pixel: column DDA + width measure.
      ------------------------------------------------------------------
      if avid = '1' then
        s_wcount <= s_wcount + 1;
        v_err_x  := s_err_x + C_BX;
        if v_err_x >= s_active_w then
          s_err_x   <= v_err_x - s_active_w;
          if s_block_x < to_unsigned(G_BLOCKS_X - 1, s_block_x'length) then
            s_block_x <= s_block_x + 1;
          end if;
        else
          s_err_x <= v_err_x;
        end if;
      end if;

      ------------------------------------------------------------------
      -- End of an active line: latch measured active_width.
      ------------------------------------------------------------------
      if v_avid_fall then
        if s_wcount /= 0 then
          s_active_w <= s_wcount;
        end if;
        block_x_last <= '1';
      end if;

      -- registered outputs
      block_x   <= s_block_x;
      block_y   <= s_block_y;
      cell_addr <= resize(s_block_y(4 downto 0) & s_block_x(5 downto 0), 16);
      frac_x    <= s_err_x;
      frac_y    <= s_err_y;
    end if;
  end process p_index;

end architecture rtl;
