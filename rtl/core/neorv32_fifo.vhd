-- ================================================================================ --
-- NEORV32 - Generic Single-Clock FIFO                                              --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_fifo is
  generic (
    FIFO_DEPTH : natural := 4;     -- number of FIFO entries; has to be a power of two; min 1
    FIFO_WIDTH : natural := 32;    -- size of data elements in FIFO
    FIFO_SAFE  : boolean := false; -- true = allow read/write only if data/space available
    OUT_GATE   : boolean := false  -- true = output zero if no data is available
  );
  port (
    -- control and status --
    clk_i   : in  std_ulogic; -- clock, rising edge
    rstn_i  : in  std_ulogic; -- async reset, low-active
    clear_i : in  std_ulogic; -- sync reset, high-active
    half_o  : out std_ulogic; -- FIFO is at least half full
    -- write port --
    wdata_i : in  std_ulogic_vector(FIFO_WIDTH-1 downto 0); -- write data
    we_i    : in  std_ulogic; -- write enable
    free_o  : out std_ulogic; -- at least one entry is free when set
    -- read port --
    re_i    : in  std_ulogic; -- read enable
    rdata_o : out std_ulogic_vector(FIFO_WIDTH-1 downto 0); -- read data
    avail_o : out std_ulogic  -- data available when set
  );
end neorv32_fifo;

architecture neorv32_fifo_rtl of neorv32_fifo is

  -- make sure FIFO depth is a power of two --
  constant fifo_depth_c : natural := cond_sel_natural_f(is_power_of_two_f(FIFO_DEPTH), FIFO_DEPTH, 2**index_size_f(FIFO_DEPTH));
  constant awidth_c     : natural := index_size_f(fifo_depth_c); -- address width

  -- memory core --
  type fifo_mem_t is array (0 to fifo_depth_c-1) of std_ulogic_vector(FIFO_WIDTH-1 downto 0);
  signal fifo_mem : fifo_mem_t;

  -- direct output data --
  signal rdata : std_ulogic_vector(FIFO_WIDTH-1 downto 0);

  -- control and status --
  signal we, re, match, full, empty, half, half_ff, free_ff, avail_ff : std_ulogic;

  -- write/read pointers --
  signal w_pnt, w_nxt, r_pnt, r_nxt, level : std_ulogic_vector(awidth_c downto 0);

begin

  -- Pointers -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  pointer_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      w_pnt <= (others => '0');
      r_pnt <= (others => '0');
    elsif rising_edge(clk_i) then
      w_pnt <= w_nxt;
      r_pnt <= r_nxt;
    end if;
  end process pointer_reg;

  -- pointer update --
  w_nxt <= (others => '0') when (clear_i = '1') else std_ulogic_vector(unsigned(w_pnt) + 1) when (we = '1') else w_pnt;
  r_nxt <= (others => '0') when (clear_i = '1') else std_ulogic_vector(unsigned(r_pnt) + 1) when (re = '1') else r_pnt;

  -- access control --
  re <= re_i when (FIFO_SAFE = false) else (re_i and (not empty)); -- SAFE = read only if data available
  we <= we_i when (FIFO_SAFE = false) else (we_i and (not full));  -- SAFE = write only if free space left


  -- Status ---------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- more than 1 FIFO entry --
  check_large:
  if (fifo_depth_c > 1) generate
    match <= '1' when (r_pnt(awidth_c-1 downto 0) = w_pnt(awidth_c-1 downto 0)) else '0';
    full  <= '1' when (r_pnt(awidth_c) /= w_pnt(awidth_c)) and (match = '1') else '0';
    empty <= '1' when (r_pnt(awidth_c)  = w_pnt(awidth_c)) and (match = '1') else '0';
    level <= std_ulogic_vector(unsigned(w_pnt) - unsigned(r_pnt));
    half  <= level(level'left-1) or full;
  end generate;

  -- just 1 FIFO entry --
  check_small:
  if (fifo_depth_c = 1) generate
    match <= '1' when (r_pnt(0) = w_pnt(0)) else '0';
    full  <= not match;
    empty <= match;
    level <= (others => full);
    half  <= full;
  end generate;

  -- synchronous status --
  status_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      free_ff  <= '0';
      avail_ff <= '0';
      half_ff  <= '0';
    elsif rising_edge(clk_i) then
      free_ff  <= not full;
      avail_ff <= not empty;
      half_ff  <= half;
    end if;
  end process status_reg;
  free_o  <= free_ff;
  avail_o <= avail_ff;
  half_o  <= half_ff;


  -- Memory Access -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- more than 1 FIFO entry --
  fifo_write_large:
  if (fifo_depth_c > 1) generate
    memory_core: process(clk_i)
    begin
      if rising_edge(clk_i) then
        if (we = '1') then
          fifo_mem(to_integer(unsigned(w_pnt(awidth_c-1 downto 0)))) <= wdata_i;
        end if;
        rdata <= fifo_mem(to_integer(unsigned(r_pnt(awidth_c-1 downto 0))));
      end if;
    end process memory_core;
  end generate;

  -- just 1 FIFO entry --
  fifo_write_small:
  if (fifo_depth_c = 1) generate
    memory_core: process(clk_i)
    begin
      if rising_edge(clk_i) then
        if (we = '1') then
          fifo_mem(0) <= wdata_i;
        end if;
      end if;
    end process memory_core;
    rdata <= fifo_mem(0); -- "async" read; not additional output register required
  end generate;

  -- output gate (output zero if no data available) --
  rdata_o <= rdata when (avail_ff = '1') or (OUT_GATE = false) else (others => '0');


end neorv32_fifo_rtl;
