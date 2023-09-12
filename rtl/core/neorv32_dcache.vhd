-- #################################################################################################
-- # << NEORV32 - Processor-Internal Data Cache >>                                                 #
-- # ********************************************************************************************* #
-- # Configurable number of cache blocks (cache lines) and block size.                             #
-- # Cache is direct mapped (single set) and uses "write through" write strategy.                  #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
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

entity neorv32_dcache is
  generic (
    DCACHE_NUM_BLOCKS : natural range 1 to 256; -- number of blocks (min 1), has to be a power of 2
    DCACHE_BLOCK_SIZE : natural range 4 to 2**16; -- block size in bytes (min 4), has to be a power of 2
    DCACHE_UC_PBEGIN  : std_ulogic_vector(3 downto 0) -- begin of uncached address space (page number)
  );
  port (
    clk_i     : in  std_ulogic; -- global clock, rising edge
    rstn_i    : in  std_ulogic; -- global reset, low-active, async
    clear_i   : in  std_ulogic; -- cache clear
    cpu_req_i : in  bus_req_t;  -- request bus
    cpu_rsp_o : out bus_rsp_t;  -- response bus
    bus_req_o : out bus_req_t;  -- request bus
    bus_rsp_i : in  bus_rsp_t   -- response bus
  );
end neorv32_dcache;

architecture neorv32_dcache_rtl of neorv32_dcache is

  -- cache layout --
  constant cache_offset_size_c : natural := index_size_f(DCACHE_BLOCK_SIZE/4); -- offset addresses full 32-bit words

  -- cache memory --
  component neorv32_dcache_memory
  generic (
    DCACHE_NUM_BLOCKS : natural; -- number of blocks (min 1), has to be a power of 2
    DCACHE_BLOCK_SIZE : natural  -- block size in bytes (min 4), has to be a power of 2
  );
  port (
    -- global control --
    clk_i        : in  std_ulogic; -- global clock, rising edge
    clear_i      : in  std_ulogic; -- invalidate whole cache
    hit_o        : out std_ulogic; -- hit access
    -- host cache access (read-only) --
    host_addr_i  : in  std_ulogic_vector(31 downto 0); -- access address
    host_rdata_o : out std_ulogic_vector(31 downto 0); -- read data
    host_rstat_o : out std_ulogic; -- access status
    -- ctrl cache access --
    ctrl_addr_i  : in  std_ulogic_vector(31 downto 0); -- access address
    ctrl_we_i    : in  std_ulogic; -- write enable (full-word)
    ctrl_ben_i   : in  std_ulogic_vector(03 downto 0); -- byte enable
    ctrl_wdata_i : in  std_ulogic_vector(31 downto 0); -- write data (full word)
    ctrl_wstat_i : in  std_ulogic; -- access status
    ctrl_rdata_o : out std_ulogic_vector(31 downto 0)  -- read data
  );
  end component;

  -- cache interface --
  type cache_if_t is record
    host_rdata : std_ulogic_vector(31 downto 0); -- host read data
    host_rstat : std_ulogic; -- access error
    ctrl_addr  : std_ulogic_vector(31 downto 0); -- access address
    ctrl_we    : std_ulogic; -- write enable
    ctrl_ben   : std_ulogic_vector(03 downto 0); -- byte-enable
    ctrl_rdata : std_ulogic_vector(31 downto 0); -- read data
    ctrl_wdata : std_ulogic_vector(31 downto 0); -- write data
    ctrl_wstat : std_ulogic; -- access error
    hit        : std_ulogic; -- hit access
    clear      : std_ulogic; -- invalidate cache
  end record;
  signal cache : cache_if_t;

  -- control engine --
  type ctrl_engine_state_t is (S_IDLE, S_CHECK, S_DOWNLOAD_REQ, S_DOWNLOAD_WAIT, S_DIRECT_REQ,
                               S_DIRECT_WAIT, S_RESYNC, S_RESYNC_READ, S_RESYNC_WRITE, S_CLEAR);
  type ctrl_t is record
    state         : ctrl_engine_state_t; -- current state
    state_nxt     : ctrl_engine_state_t; -- next state
    addr_reg      : std_ulogic_vector(31 downto 0); -- address register for block download
    addr_reg_nxt  : std_ulogic_vector(31 downto 0);
    re_buf        : std_ulogic; -- read request
    re_buf_nxt    : std_ulogic;
    we_buf        : std_ulogic; -- write request
    we_buf_nxt    : std_ulogic;
    clear_buf     : std_ulogic; -- clear request
    clear_buf_nxt : std_ulogic;
  end record;
  signal ctrl : ctrl_t;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (is_power_of_two_f(DCACHE_NUM_BLOCKS) = false) report
    "NEORV32 PROCESSOR CONFIG ERROR! d-cache number of blocks <DCACHE_NUM_BLOCKS> has to be a power of 2." severity error;
  assert not (is_power_of_two_f(DCACHE_BLOCK_SIZE) = false) report
    "NEORV32 PROCESSOR CONFIG ERROR! d-cache block size <DCACHE_BLOCK_SIZE> has to be a power of 2." severity error;


  -- Control Engine FSM Sync ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_engine_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl.state     <= S_CLEAR; -- to reset cache information memory, which does not have an explicit reset
      ctrl.addr_reg  <= (others => '0');
      ctrl.re_buf    <= '0';
      ctrl.we_buf    <= '0';
      ctrl.clear_buf <= '0';
    elsif rising_edge(clk_i) then
      ctrl.state     <= ctrl.state_nxt;
      ctrl.addr_reg  <= ctrl.addr_reg_nxt;
      ctrl.re_buf    <= ctrl.re_buf_nxt;
      ctrl.we_buf    <= ctrl.we_buf_nxt;
      ctrl.clear_buf <= ctrl.clear_buf_nxt;
    end if;
  end process ctrl_engine_sync;


  -- Control Engine FSM Comb ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_engine_comb: process(ctrl, cache, clear_i, cpu_req_i, bus_rsp_i)
  begin
    -- control defaults --
    ctrl.state_nxt     <= ctrl.state;
    ctrl.addr_reg_nxt  <= ctrl.addr_reg;

    -- request buffer --
    ctrl.re_buf_nxt    <= ctrl.re_buf or cpu_req_i.re;
    ctrl.we_buf_nxt    <= ctrl.we_buf or cpu_req_i.we;
    ctrl.clear_buf_nxt <= ctrl.clear_buf or clear_i;

    -- cache defaults --
    cache.clear        <= '0';
    cache.ctrl_addr    <= ctrl.addr_reg;
    cache.ctrl_we      <= '0';
    cache.ctrl_ben     <= "1111";
    cache.ctrl_wdata   <= bus_rsp_i.data;
    cache.ctrl_wstat   <= bus_rsp_i.err;

    -- host interface defaults --
    cpu_rsp_o.data     <= cache.host_rdata;
    cpu_rsp_o.ack      <= '0';
    cpu_rsp_o.err      <= '0';

    -- peripheral bus interface defaults --
    bus_req_o.addr     <= ctrl.addr_reg;
    bus_req_o.data     <= cpu_req_i.data;
    bus_req_o.ben      <= cpu_req_i.ben;
    bus_req_o.re       <= '0';
    bus_req_o.we       <= '0';
    bus_req_o.src      <= cpu_req_i.src;
    bus_req_o.priv     <= cpu_req_i.priv;
    bus_req_o.rvso     <= cpu_req_i.rvso;

    -- fsm --
    case ctrl.state is

      when S_IDLE => -- wait for host access request or cache control operation
      -- ------------------------------------------------------------
        ctrl.addr_reg_nxt <= cpu_req_i.addr;
        if (ctrl.clear_buf = '1') then -- invalidate cache
          ctrl.state_nxt <= S_CLEAR;
        elsif (cpu_req_i.re = '1') or (ctrl.re_buf = '1') or (cpu_req_i.we = '1') or (ctrl.we_buf = '1') then
          if (unsigned(cpu_req_i.addr(31 downto 28)) >= unsigned(DCACHE_UC_PBEGIN)) or (cpu_req_i.rvso = '1') then -- uncached access -> direct access
            ctrl.state_nxt <= S_DIRECT_REQ;
          else -- cached access
            ctrl.state_nxt <= S_CHECK;
          end if;
        end if;

      when S_CHECK => -- check if cache hit
      -- ------------------------------------------------------------
        if (ctrl.re_buf = '1') then -- read access
          -- calculate block base address (in case we need to download it) --
          ctrl.addr_reg_nxt((cache_offset_size_c+2)-1 downto 2) <= (others => '0'); -- block-aligned
          ctrl.addr_reg_nxt(1 downto 0) <= "00"; -- word-aligned
          --
          if (cache.hit = '1') then -- HIT -> done
            ctrl.re_buf_nxt <= '0';
            ctrl.we_buf_nxt <= '0';
            if (cache.host_rstat = '1') then -- erroneous read access?
              cpu_rsp_o.err <= '1';
            else
              cpu_rsp_o.ack <= '1';
            end if;
            ctrl.state_nxt <= S_IDLE;
          else -- cache MISS -> download block
            ctrl.state_nxt <= S_DOWNLOAD_REQ;
          end if;
        else -- write access
          if (cache.hit = '1') then -- data word in cache -> also write to cache
            ctrl.state_nxt <= S_RESYNC_WRITE;
          else -- write-through
            ctrl.state_nxt <= S_DIRECT_REQ;
          end if;
        end if;


      when S_DOWNLOAD_REQ => -- download new cache block: request new word
      -- ------------------------------------------------------------
        bus_req_o.re   <= '1'; -- request new read transfer
        ctrl.state_nxt <= S_DOWNLOAD_WAIT;

      when S_DOWNLOAD_WAIT => -- download new cache block: wait for bus response
      -- ------------------------------------------------------------
        if (bus_rsp_i.ack = '1') or (bus_rsp_i.err = '1') then -- ACK or ERROR -> write to cache and get next word (store ERROR flag in cache)
          cache.ctrl_we     <= '1'; -- write to cache
          ctrl.addr_reg_nxt <= std_ulogic_vector(unsigned(ctrl.addr_reg) + 4);
          if (and_reduce_f(ctrl.addr_reg((cache_offset_size_c+2)-1 downto 2)) = '1') then -- block complete?
            ctrl.state_nxt <= S_RESYNC;
          else -- get next word
            ctrl.state_nxt <= S_DOWNLOAD_REQ;
          end if;
        end if;


      when S_DIRECT_REQ => -- direct uncached access: request access
      -- ------------------------------------------------------------
        bus_req_o.re   <= ctrl.re_buf;
        bus_req_o.we   <= ctrl.we_buf;
        ctrl.state_nxt <= S_DIRECT_WAIT;

      when S_DIRECT_WAIT => -- direct uncached access: wait for bus response
      -- ------------------------------------------------------------
        ctrl.re_buf_nxt <= '0';
        ctrl.we_buf_nxt <= '0';
        cpu_rsp_o.data  <= bus_rsp_i.data;
        if (bus_rsp_i.err = '1') then
          cpu_rsp_o.err  <= '1';
          ctrl.state_nxt <= S_IDLE;
        elsif (bus_rsp_i.ack = '1') then
          cpu_rsp_o.ack  <= '1';
          ctrl.state_nxt <= S_IDLE;
        end if;


      when S_RESYNC => -- re-sync host/cache access
      -- ------------------------------------------------------------
        ctrl.addr_reg_nxt <= cpu_req_i.addr; -- restore original access address
        if (ctrl.we_buf = '1') then -- write access
          ctrl.state_nxt <= S_RESYNC_WRITE;
        else -- read access
          ctrl.state_nxt <= S_CHECK; -- should HIT now
        end if;

      when S_RESYNC_WRITE => -- finalize cached write access
      -- ------------------------------------------------------------
        bus_req_o.we     <= '1'; -- trigger bus write access
        cache.ctrl_we    <= '1'; -- write to cache
        cache.ctrl_ben   <= cpu_req_i.ben;
        cache.ctrl_addr  <= cpu_req_i.addr;
        cache.ctrl_wdata <= cpu_req_i.data;
        cache.ctrl_wstat <= '0'; -- no error possible here
        ctrl.state_nxt   <= S_DIRECT_WAIT;


      when S_CLEAR => -- invalidate all cache entries
      -- ------------------------------------------------------------
        ctrl.clear_buf_nxt <= '0';
        cache.clear        <= '1';
        ctrl.state_nxt     <= S_IDLE;

      when others => -- undefined
      -- ------------------------------------------------------------
        ctrl.state_nxt <= S_IDLE;

    end case;
  end process ctrl_engine_comb;


	-- Cache Memory ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_dcache_memory_inst: neorv32_dcache_memory
  generic map (
    DCACHE_NUM_BLOCKS => DCACHE_NUM_BLOCKS,
    DCACHE_BLOCK_SIZE => DCACHE_BLOCK_SIZE
  )
  port map (
    -- global control --
    clk_i        => clk_i,
    clear_i      => cache.clear,
    hit_o        => cache.hit,
    -- host cache access --
    host_addr_i  => cpu_req_i.addr,
    host_rdata_o => cache.host_rdata,
    host_rstat_o => cache.host_rstat,
    -- ctrl cache access --
    ctrl_addr_i  => cache.ctrl_addr,
    ctrl_we_i    => cache.ctrl_we,
    ctrl_ben_i   => cache.ctrl_ben,
    ctrl_wdata_i => cache.ctrl_wdata,
    ctrl_wstat_i => cache.ctrl_wstat,
    ctrl_rdata_o => cache.ctrl_rdata
  );

end neorv32_dcache_rtl;


-- ###########################################################################################################################################
-- ###########################################################################################################################################


-- #################################################################################################
-- # << NEORV32 - Data Cache Memory >>                                                             #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
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

entity neorv32_dcache_memory is
  generic (
    DCACHE_NUM_BLOCKS : natural; -- number of blocks (min 1), has to be a power of 2
    DCACHE_BLOCK_SIZE : natural  -- block size in bytes (min 4), has to be a power of 2
  );
  port (
    -- global control --
    clk_i        : in  std_ulogic; -- global clock, rising edge
    clear_i      : in  std_ulogic; -- invalidate whole cache
    hit_o        : out std_ulogic;  -- hit access
    -- host cache access (read-only) --
    host_addr_i  : in  std_ulogic_vector(31 downto 0); -- access address
    host_rdata_o : out std_ulogic_vector(31 downto 0); -- read data
    host_rstat_o : out std_ulogic; -- access status
    -- ctrl cache access --
    ctrl_addr_i  : in  std_ulogic_vector(31 downto 0); -- access address
    ctrl_we_i    : in  std_ulogic; -- write enable
    ctrl_ben_i   : in  std_ulogic_vector(03 downto 0); -- byte enable
    ctrl_wdata_i : in  std_ulogic_vector(31 downto 0); -- write data (full word)
    ctrl_wstat_i : in  std_ulogic; -- access status
    ctrl_rdata_o : out std_ulogic_vector(31 downto 0)  -- read data
  );
end neorv32_dcache_memory;

architecture neorv32_dcache_memory_rtl of neorv32_dcache_memory is

  -- cache layout --
  constant cache_offset_size_c : natural := index_size_f(DCACHE_BLOCK_SIZE/4); -- offset addresses full 32-bit words
  constant cache_index_size_c  : natural := index_size_f(DCACHE_NUM_BLOCKS);
  constant cache_tag_size_c    : natural := 32 - (cache_offset_size_c + cache_index_size_c + 2); -- 2 additional bits for byte offset
  constant cache_entries_c     : natural := DCACHE_NUM_BLOCKS * (DCACHE_BLOCK_SIZE/4); -- number of 32-bit entries (per set)

  -- status flag memory --
  signal valid_flag : std_ulogic_vector(DCACHE_NUM_BLOCKS-1 downto 0);
  signal valid      : std_ulogic;

  -- tag memory --
  type tag_mem_t is array (0 to DCACHE_NUM_BLOCKS-1) of std_ulogic_vector(cache_tag_size_c-1 downto 0);
  signal tag_mem : tag_mem_t;
  signal tag     : std_ulogic_vector(cache_tag_size_c-1 downto 0);

  -- access address decomposition --
  type acc_addr_t is record
    tag    : std_ulogic_vector(cache_tag_size_c-1 downto 0);
    index  : std_ulogic_vector(cache_index_size_c-1 downto 0);
    offset : std_ulogic_vector(cache_offset_size_c-1 downto 0);
  end record;
  signal host_acc_addr, ctrl_acc_addr : acc_addr_t;

  -- cache data memory --
  type cache_mem_t is array (0 to cache_entries_c-1) of std_ulogic_vector(7 downto 0);
  signal cache_data_memory_b0 : cache_mem_t; -- byte 0
  signal cache_data_memory_b1 : cache_mem_t; -- byte 1
  signal cache_data_memory_b2 : cache_mem_t; -- byte 2
  signal cache_data_memory_b3 : cache_mem_t; -- byte 3
  signal cache_err_memory     : std_ulogic_vector(cache_entries_c-1 downto 0); -- access error flag

  -- cache data memory access --
  signal cache_rdata  : std_ulogic_vector(32 downto 0);
  signal cache_index  : std_ulogic_vector(cache_index_size_c-1 downto 0);
  signal cache_offset : std_ulogic_vector(cache_offset_size_c-1 downto 0);
  signal cache_addr   : std_ulogic_vector((cache_index_size_c+cache_offset_size_c)-1 downto 0); -- index & offset

begin

	-- Access Address Decomposition -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  host_acc_addr.tag    <= host_addr_i(31 downto 31-(cache_tag_size_c-1));
  host_acc_addr.index  <= host_addr_i(31-cache_tag_size_c downto 2+cache_offset_size_c);
  host_acc_addr.offset <= host_addr_i(2+(cache_offset_size_c-1) downto 2); -- discard byte offset

  ctrl_acc_addr.tag    <= ctrl_addr_i(31 downto 31-(cache_tag_size_c-1));
  ctrl_acc_addr.index  <= ctrl_addr_i(31-cache_tag_size_c downto 2+cache_offset_size_c);
  ctrl_acc_addr.offset <= ctrl_addr_i(2+(cache_offset_size_c-1) downto 2); -- discard byte offset


	-- Status Flag Memory ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  status_memory: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- write access --
      if (clear_i = '1') then -- invalidate entire cache
        valid_flag <= (others => '0');
      elsif (ctrl_we_i = '1') then -- control write access: make current block valid
        valid_flag(to_integer(unsigned(cache_index))) <= '1';
      end if;
      -- sync read access --
      valid <= valid_flag(to_integer(unsigned(cache_index)));
    end if;
  end process status_memory;


	-- Tag Memory -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tag_memory: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (ctrl_we_i = '1') then -- write access
        tag_mem(to_integer(unsigned(cache_index))) <= ctrl_acc_addr.tag;
      else -- read access
        tag <= tag_mem(to_integer(unsigned(cache_index)));
      end if;
    end if;
  end process tag_memory;

  -- hit? --
  hit_o <= '1' when (host_acc_addr.tag = tag) and (valid = '1') else '0';


	-- Cache Data Memory ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cache_mem_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- write access --
      if (ctrl_we_i = '1') and (ctrl_ben_i(0) = '1') then
        cache_data_memory_b0(to_integer(unsigned(cache_addr))) <= ctrl_wdata_i(07 downto 00);
      end if;
      if (ctrl_we_i = '1') and (ctrl_ben_i(1) = '1') then
        cache_data_memory_b1(to_integer(unsigned(cache_addr))) <= ctrl_wdata_i(15 downto 08);
      end if;
      if (ctrl_we_i = '1') and (ctrl_ben_i(2) = '1') then
        cache_data_memory_b2(to_integer(unsigned(cache_addr))) <= ctrl_wdata_i(23 downto 16);
      end if;
      if (ctrl_we_i = '1') and (ctrl_ben_i(3) = '1') then
        cache_data_memory_b3(to_integer(unsigned(cache_addr))) <= ctrl_wdata_i(31 downto 24);
      end if;
      if (ctrl_we_i = '1') then
        cache_err_memory(to_integer(unsigned(cache_addr))) <= ctrl_wstat_i;
      end if;
      -- read access --
      cache_rdata(07 downto 00) <= cache_data_memory_b0(to_integer(unsigned(cache_addr)));
      cache_rdata(15 downto 08) <= cache_data_memory_b1(to_integer(unsigned(cache_addr)));
      cache_rdata(23 downto 16) <= cache_data_memory_b2(to_integer(unsigned(cache_addr)));
      cache_rdata(31 downto 24) <= cache_data_memory_b3(to_integer(unsigned(cache_addr)));
      cache_rdata(32)           <=     cache_err_memory(to_integer(unsigned(cache_addr)));
    end if;
  end process cache_mem_access;

  -- data output --
  host_rdata_o <= cache_rdata(31 downto 0);
  ctrl_rdata_o <= cache_rdata(31 downto 0);
  host_rstat_o <= cache_rdata(32) and valid;

  -- cache block ram access address --
  cache_addr <= cache_index & cache_offset;

  -- cache access select --
  cache_index  <= host_acc_addr.index  when (ctrl_we_i = '0') else ctrl_acc_addr.index;
  cache_offset <= host_acc_addr.offset when (ctrl_we_i = '0') else ctrl_acc_addr.offset;


end neorv32_dcache_memory_rtl;
