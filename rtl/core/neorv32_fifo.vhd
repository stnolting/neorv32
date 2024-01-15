-- #################################################################################################
-- # << NEORV32 - Generic Single-Clock FIFO Component >>                                           #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2024, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_fifo is
  generic (
    FIFO_DEPTH : natural := 4;     -- number of fifo entries; has to be a power of two; min 1
    FIFO_WIDTH : natural := 32;    -- size of data elements in fifo
    FIFO_RSYNC : boolean := false; -- false = async read; true = sync read
    FIFO_SAFE  : boolean := false; -- true = allow read/write only if data available
    FULL_RESET : boolean := false  -- true = reset all memory cells (cannot be mapped to BRAM)
  );
  port (
    -- control --
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

  -- FIFO storage --
  type fifo_mem_t is array (0 to fifo_depth_c-1) of std_ulogic_vector(FIFO_WIDTH-1 downto 0);
  signal fifo_mem : fifo_mem_t;

  -- FIFO control --
  signal we,    re    : std_ulogic; -- write-/read-enable
  signal w_pnt, r_pnt : std_ulogic_vector(index_size_f(fifo_depth_c) downto 0); -- write/read pointer

  -- status --
  signal match, empty, full, half, free, avail : std_ulogic;

  -- fill level --
  signal diff : std_ulogic_vector(index_size_f(fifo_depth_c) downto 0);

begin

  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  re <= re_i when (FIFO_SAFE = false) else (re_i and avail); -- SAFE = read only if data available
  we <= we_i when (FIFO_SAFE = false) else (we_i and free);  -- SAFE = write only if free space left


  -- Pointers -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  pointer_update: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      w_pnt <= (others => '0');
      r_pnt <= (others => '0');
    elsif rising_edge(clk_i) then
      -- write port --
      if (clear_i = '1') then
        w_pnt <= (others => '0');
      elsif (we = '1') then
        w_pnt <= std_ulogic_vector(unsigned(w_pnt) + 1);
      end if;
      -- read port --
      if (clear_i = '1') then
        r_pnt <= (others => '0');
      elsif (re = '1') then
        r_pnt <= std_ulogic_vector(unsigned(r_pnt) + 1);
      end if;
    end if;
  end process pointer_update;

  check_large:
  if (fifo_depth_c > 1) generate
    match <= '1' when (r_pnt(r_pnt'left-1 downto 0) = w_pnt(w_pnt'left-1 downto 0)) else '0';
    full  <= '1' when (r_pnt(r_pnt'left) /= w_pnt(w_pnt'left)) and (match = '1') else '0';
    empty <= '1' when (r_pnt(r_pnt'left)  = w_pnt(w_pnt'left)) and (match = '1') else '0';
    diff  <= std_ulogic_vector(unsigned(w_pnt) - unsigned(r_pnt));
    half  <= diff(diff'left-1) or full;
  end generate;

  check_small:
  if (fifo_depth_c = 1) generate
    match <= '1' when (r_pnt(0) = w_pnt(0)) else '0';
    full  <= not match;
    empty <= match;
    half  <= full;
  end generate;

  free  <= not full;
  avail <= not empty;


  -- Write Data -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  memory_full_reset: -- cannot be mapped to block RAM!
  if FULL_RESET generate
    fifo_write: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        fifo_mem <= (others => (others => '0')); -- full reset of memory cells
      elsif rising_edge(clk_i) then
        if (we = '1') then
          fifo_mem(to_integer(unsigned(w_pnt(w_pnt'left-1 downto 0)))) <= wdata_i;
        end if;
      end if;
    end process fifo_write;
  end generate;

  memory_no_reset: -- no reset to infer block RAM
  if not FULL_RESET generate
    fifo_write: process(clk_i)
    begin
      if rising_edge(clk_i) then
        if (we = '1') then
          fifo_mem(to_integer(unsigned(w_pnt(w_pnt'left-1 downto 0)))) <= wdata_i;
        end if;
      end if;
    end process fifo_write;
  end generate;


  -- Read Data ------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  fifo_read_async: -- asynchronous read
  if not FIFO_RSYNC generate
    rdata_o <= fifo_mem(to_integer(unsigned(r_pnt(r_pnt'left-1 downto 0))));
    -- status --
    free_o  <= free;
    avail_o <= avail;
    half_o  <= half;
  end generate;

  fifo_read_sync: -- synchronous read
  if FIFO_RSYNC generate
    sync_read: process(clk_i)
    begin
      if rising_edge(clk_i) then
        rdata_o <= fifo_mem(to_integer(unsigned(r_pnt(r_pnt'left-1 downto 0))));
      end if;
    end process sync_read;
    -- status --
    sync_status_flags: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        free_o  <= '0';
        avail_o <= '0';
        half_o  <= '0';
      elsif rising_edge(clk_i) then
        free_o  <= free;
        avail_o <= avail;
        half_o  <= half;
      end if;
    end process sync_status_flags;
  end generate;


end neorv32_fifo_rtl;
