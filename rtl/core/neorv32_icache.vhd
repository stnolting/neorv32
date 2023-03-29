-- #################################################################################################
-- # << NEORV32 - Processor-Internal Instruction Cache >>                                          #
-- # ********************************************************************************************* #
-- # Direct mapped (ICACHE_NUM_SETS = 1) or 2-way set-associative (ICACHE_NUM_SETS = 2).           #
-- # Least recently used replacement policy (if ICACHE_NUM_SETS > 1).                              #
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

entity neorv32_icache is
  generic (
    ICACHE_NUM_BLOCKS : natural; -- number of blocks (min 1), has to be a power of 2
    ICACHE_BLOCK_SIZE : natural; -- block size in bytes (min 4), has to be a power of 2
    ICACHE_NUM_SETS   : natural  -- associativity / number of sets (1=direct_mapped), has to be a power of 2
  );
  port (
    -- global control --
    clk_i        : in  std_ulogic; -- global clock, rising edge
    rstn_i       : in  std_ulogic; -- global reset, low-active, async
    clear_i      : in  std_ulogic; -- cache clear
    -- host controller interface --
    host_addr_i  : in  std_ulogic_vector(31 downto 0); -- bus access address
    host_rdata_o : out std_ulogic_vector(31 downto 0); -- bus read data
    host_re_i    : in  std_ulogic; -- read enable
    host_ack_o   : out std_ulogic; -- bus transfer acknowledge
    host_err_o   : out std_ulogic; -- bus transfer error
    -- peripheral bus interface --
    bus_cached_o : out std_ulogic; -- set if cached (!) access in progress
    bus_addr_o   : out std_ulogic_vector(31 downto 0); -- bus access address
    bus_rdata_i  : in  std_ulogic_vector(31 downto 0); -- bus read data
    bus_re_o     : out std_ulogic; -- read enable
    bus_ack_i    : in  std_ulogic; -- bus transfer acknowledge
    bus_err_i    : in  std_ulogic  -- bus transfer error
  );
end neorv32_icache;

architecture neorv32_icache_rtl of neorv32_icache is

  -- cache layout --
  constant cache_offset_size_c : natural := index_size_f(ICACHE_BLOCK_SIZE/4); -- offset addresses full 32-bit words

  -- cache memory --
  component neorv32_icache_memory
  generic (
    ICACHE_NUM_BLOCKS : natural; -- number of blocks (min 1), has to be a power of 2
    ICACHE_BLOCK_SIZE : natural; -- block size in bytes (min 4), has to be a power of 2
    ICACHE_NUM_SETS   : natural  -- associativity; 1=direct-mapped, 2=2-way set-associative
  );
  port (
    -- global control --
    clk_i        : in  std_ulogic; -- global clock, rising edge
    clear_i      : in  std_ulogic; -- invalidate whole cache
    hit_o        : out std_ulogic; -- hit access
    -- host cache access (read-only) --
    host_addr_i  : in  std_ulogic_vector(31 downto 0); -- access address
    host_re_i    : in  std_ulogic; -- read enable
    host_rdata_o : out std_ulogic_vector(31 downto 0); -- read data
    host_rstat_o : out std_ulogic; -- read status
    -- ctrl cache access (write-only) --
    ctrl_en_i    : in  std_ulogic; -- control interface enable
    ctrl_addr_i  : in  std_ulogic_vector(31 downto 0); -- access address
    ctrl_we_i    : in  std_ulogic; -- write enable (full-word)
    ctrl_wdata_i : in  std_ulogic_vector(31 downto 0); -- write data
    ctrl_wstat_i : in  std_ulogic  -- write status
  );
  end component;

  -- cache interface --
  type cache_if_t is record
    clear      : std_ulogic; -- cache clear
    host_addr  : std_ulogic_vector(31 downto 0); -- cpu access address
    host_rdata : std_ulogic_vector(31 downto 0); -- cpu read data
    host_rstat : std_ulogic; -- cpu read status
    hit        : std_ulogic; -- hit access
    ctrl_en    : std_ulogic; -- control access enable
    ctrl_addr  : std_ulogic_vector(31 downto 0); -- control access address
    ctrl_we    : std_ulogic; -- control write enable
    ctrl_wdata : std_ulogic_vector(31 downto 0); -- control write data
    ctrl_wstat : std_ulogic; -- control write status
  end record;
  signal cache : cache_if_t;

  -- control engine --
  type ctrl_engine_state_t is (S_IDLE, S_CLEAR, S_CHECK, S_DOWNLOAD_REQ, S_DOWNLOAD_GET, S_RESYNC);
  type ctrl_t is record
    state         : ctrl_engine_state_t; -- current state
    state_nxt     : ctrl_engine_state_t; -- next state
    addr_reg      : std_ulogic_vector(31 downto 0); -- address register for block download
    addr_reg_nxt  : std_ulogic_vector(31 downto 0);
    re_buf        : std_ulogic; -- read request buffer
    re_buf_nxt    : std_ulogic;
    clear_buf     : std_ulogic; -- clear request buffer
    clear_buf_nxt : std_ulogic;
  end record;
  signal ctrl : ctrl_t;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- configuration --
  assert not (is_power_of_two_f(ICACHE_NUM_BLOCKS) = false) report
    "NEORV32 PROCESSOR CONFIG ERROR! i-cache number of blocks <ICACHE_NUM_BLOCKS> has to be a power of 2." severity error;
  assert not (is_power_of_two_f(ICACHE_BLOCK_SIZE) = false) report
    "NEORV32 PROCESSOR CONFIG ERROR! i-cache block size <ICACHE_BLOCK_SIZE> has to be a power of 2." severity error;
  assert not ((is_power_of_two_f(ICACHE_NUM_SETS) = false)) report
    "NEORV32 PROCESSOR CONFIG ERROR! i-cache associativity <ICACHE_NUM_SETS> has to be a power of 2." severity error;
  assert not (ICACHE_NUM_BLOCKS < 1) report
    "NEORV32 PROCESSOR CONFIG ERROR! i-cache number of blocks <ICACHE_NUM_BLOCKS> has to be >= 1." severity error;
  assert not (ICACHE_BLOCK_SIZE < 4) report
    "NEORV32 PROCESSOR CONFIG ERROR! i-cache block size <ICACHE_BLOCK_SIZE> has to be >= 4." severity error;
  assert not ((ICACHE_NUM_SETS = 0) or (ICACHE_NUM_SETS > 2)) report
    "NEORV32 PROCESSOR CONFIG ERROR! i-cache associativity <ICACHE_NUM_SETS> has to be 1 (direct-mapped) or 2 (2-way set-associative)." severity error;


  -- Control Engine FSM Sync ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_engine_fsm_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl.state     <= S_CLEAR; -- to reset cache information memory, which does not have an explicit reset
      ctrl.re_buf    <= '0';
      ctrl.clear_buf <= '0';
      ctrl.addr_reg  <= (others => '0');
    elsif rising_edge(clk_i) then
      ctrl.state     <= ctrl.state_nxt;
      ctrl.re_buf    <= ctrl.re_buf_nxt;
      ctrl.clear_buf <= ctrl.clear_buf_nxt;
      ctrl.addr_reg  <= ctrl.addr_reg_nxt;
    end if;
  end process ctrl_engine_fsm_sync;


  -- Control Engine FSM Comb ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_engine_fsm_comb: process(ctrl, cache, clear_i, host_addr_i, host_re_i, bus_rdata_i, bus_ack_i, bus_err_i)
  begin
    -- control defaults --
    ctrl.state_nxt     <= ctrl.state;
    ctrl.addr_reg_nxt  <= ctrl.addr_reg;
    ctrl.re_buf_nxt    <= ctrl.re_buf or host_re_i;
    ctrl.clear_buf_nxt <= ctrl.clear_buf or clear_i; -- buffer clear request from CPU

    -- cache defaults --
    cache.clear        <= '0';
    cache.host_addr    <= host_addr_i;
    cache.ctrl_en      <= '0';
    cache.ctrl_addr    <= ctrl.addr_reg;
    cache.ctrl_we      <= '0';
    cache.ctrl_wdata   <= bus_rdata_i;
    cache.ctrl_wstat   <= bus_err_i;

    -- host interface defaults --
    host_ack_o         <= '0';
    host_err_o         <= '0';
    host_rdata_o       <= cache.host_rdata;

    -- peripheral bus interface defaults --
    bus_addr_o         <= ctrl.addr_reg;
    bus_re_o           <= '0';

    -- fsm --
    case ctrl.state is

      when S_IDLE => -- wait for host access request or cache control operation
      -- ------------------------------------------------------------
        if (ctrl.clear_buf = '1') then -- cache control operation?
          ctrl.state_nxt <= S_CLEAR;
        elsif (host_re_i = '1') or (ctrl.re_buf = '1') then -- cache access
          ctrl.state_nxt <= S_CHECK;
        end if;

      when S_CHECK => -- finalize host access if cache hit
      -- ------------------------------------------------------------
        -- calculate block base address - in case we need to download it --
        ctrl.addr_reg_nxt <= host_addr_i;
        ctrl.addr_reg_nxt((cache_offset_size_c+2)-1 downto 2) <= (others => '0'); -- block-aligned
        ctrl.addr_reg_nxt(1 downto 0) <= "00"; -- word-aligned
        --
        ctrl.re_buf_nxt <= '0';
        if (cache.hit = '1') then -- cache HIT
          if (cache.host_rstat = '1') then -- data word from cache marked as faulty?
            host_err_o <= '1';
          else
            host_ack_o <= '1';
          end if;
          ctrl.state_nxt <= S_IDLE;
        else -- cache MISS
          ctrl.state_nxt <= S_DOWNLOAD_REQ;
        end if;

      when S_DOWNLOAD_REQ => -- download new cache block: request new word
      -- ------------------------------------------------------------
        bus_re_o       <= '1'; -- request new read transfer
        ctrl.state_nxt <= S_DOWNLOAD_GET;

      when S_DOWNLOAD_GET => -- download new cache block: wait for bus response
      -- ------------------------------------------------------------
        cache.ctrl_en <= '1'; -- cache update operation
        if (bus_ack_i = '1') or (bus_err_i = '1') then -- ACK or ERROR = write to cache and get next word (store ERROR flag in cache)
          cache.ctrl_we <= '1'; -- write to cache
          if (and_reduce_f(ctrl.addr_reg((cache_offset_size_c+2)-1 downto 2)) = '1') then -- block complete?
            ctrl.state_nxt <= S_RESYNC;
          else -- get next word
            ctrl.addr_reg_nxt <= std_ulogic_vector(unsigned(ctrl.addr_reg) + 4);
            ctrl.state_nxt    <= S_DOWNLOAD_REQ;
          end if;
        end if;

      when S_RESYNC => -- re-sync host/cache access: cache read-latency
      -- ------------------------------------------------------------
        ctrl.state_nxt <= S_CHECK;

      when S_CLEAR => -- invalidate all cache entries
      -- ------------------------------------------------------------
        ctrl.clear_buf_nxt <= '0';
        cache.clear        <= '1';
        ctrl.state_nxt     <= S_IDLE;

      when others => -- undefined
      -- ------------------------------------------------------------
        ctrl.state_nxt <= S_IDLE;

    end case;
  end process ctrl_engine_fsm_comb;

  -- cached access? --
  bus_cached_o <= '1' when (ctrl.state = S_DOWNLOAD_REQ) or (ctrl.state = S_DOWNLOAD_GET) else '0';


  -- Cache Memory ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_icache_memory_inst: neorv32_icache_memory
  generic map (
    ICACHE_NUM_BLOCKS => ICACHE_NUM_BLOCKS,
    ICACHE_BLOCK_SIZE => ICACHE_BLOCK_SIZE,
    ICACHE_NUM_SETS   => ICACHE_NUM_SETS
  )
  port map (
    -- global control --
    clk_i        => clk_i,
    clear_i      => cache.clear,
    hit_o        => cache.hit,
    -- host cache access (read-only) --
    host_addr_i  => cache.host_addr,
    host_re_i    => host_re_i,
    host_rdata_o => cache.host_rdata,
    host_rstat_o => cache.host_rstat,
    -- ctrl cache access (write-only) --
    ctrl_en_i    => cache.ctrl_en,
    ctrl_addr_i  => cache.ctrl_addr,
    ctrl_we_i    => cache.ctrl_we,
    ctrl_wdata_i => cache.ctrl_wdata,
    ctrl_wstat_i => cache.ctrl_wstat
  );

end neorv32_icache_rtl;


-- ###########################################################################################################################################
-- ###########################################################################################################################################


-- #################################################################################################
-- # << NEORV32 - Instruction Cache Memory >>                                                      #
-- # ********************************************************************************************* #
-- # Direct mapped (ICACHE_NUM_SETS = 1) or 2-way set-associative (ICACHE_NUM_SETS = 2).           #
-- # Least recently used replacement policy (if ICACHE_NUM_SETS > 1).                              #
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

entity neorv32_icache_memory is
  generic (
    ICACHE_NUM_BLOCKS : natural; -- number of blocks (min 1), has to be a power of 2
    ICACHE_BLOCK_SIZE : natural; -- block size in bytes (min 4), has to be a power of 2
    ICACHE_NUM_SETS   : natural  -- associativity; 1=direct-mapped, 2=2-way set-associative
  );
  port (
    -- global control --
    clk_i        : in  std_ulogic; -- global clock, rising edge
    clear_i      : in  std_ulogic; -- invalidate whole cache
    hit_o        : out std_ulogic; -- hit access
    -- host cache access (read-only) --
    host_addr_i  : in  std_ulogic_vector(31 downto 0); -- access address
    host_re_i    : in  std_ulogic; -- read enable
    host_rdata_o : out std_ulogic_vector(31 downto 0); -- read data
    host_rstat_o : out std_ulogic; -- read status
    -- ctrl cache access (write-only) --
    ctrl_en_i    : in  std_ulogic; -- control interface enable
    ctrl_addr_i  : in  std_ulogic_vector(31 downto 0); -- access address
    ctrl_we_i    : in  std_ulogic; -- write enable (full-word)
    ctrl_wdata_i : in  std_ulogic_vector(31 downto 0); -- write data
    ctrl_wstat_i : in  std_ulogic  -- write status
  );
end neorv32_icache_memory;

architecture neorv32_icache_memory_rtl of neorv32_icache_memory is

  -- cache layout --
  constant cache_offset_size_c : natural := index_size_f(ICACHE_BLOCK_SIZE/4); -- offset addresses full 32-bit words
  constant cache_index_size_c  : natural := index_size_f(ICACHE_NUM_BLOCKS);
  constant cache_tag_size_c    : natural := 32 - (cache_offset_size_c + cache_index_size_c + 2); -- 2 additional bits for byte offset
  constant cache_entries_c     : natural := ICACHE_NUM_BLOCKS * (ICACHE_BLOCK_SIZE/4); -- number of 32-bit entries (per set)

  -- status flag memory --
  signal valid_flag_s0 : std_ulogic_vector(ICACHE_NUM_BLOCKS-1 downto 0);
  signal valid_flag_s1 : std_ulogic_vector(ICACHE_NUM_BLOCKS-1 downto 0);
  signal valid         : std_ulogic_vector(1 downto 0); -- valid flag read data

  -- tag memory --
  type tag_mem_t is array (0 to ICACHE_NUM_BLOCKS-1) of std_ulogic_vector(cache_tag_size_c-1 downto 0);
  signal tag_mem_s0, tag_mem_s1 : tag_mem_t;
  type tag_rd_t is array (0 to 1) of std_ulogic_vector(cache_tag_size_c-1 downto 0);
  signal tag : tag_rd_t; -- tag read data

  -- access status --
  signal hit : std_ulogic_vector(1 downto 0);

  -- access address decomposition --
  type acc_addr_t is record
    tag    : std_ulogic_vector(cache_tag_size_c-1 downto 0);
    index  : std_ulogic_vector(cache_index_size_c-1 downto 0);
    offset : std_ulogic_vector(cache_offset_size_c-1 downto 0);
  end record;
  signal host_acc_addr, ctrl_acc_addr : acc_addr_t;

  -- cache data memory (32-bit data + 1-bit status) --
  type cache_mem_t is array (0 to cache_entries_c-1) of std_ulogic_vector(31+1 downto 0);
  signal cache_data_memory_s0 : cache_mem_t; -- set 0
  signal cache_data_memory_s1 : cache_mem_t; -- set 1

  -- cache data memory access --
  type cache_rdata_t is array (0 to 1) of std_ulogic_vector(31+1 downto 0);
  signal cache_rd     : cache_rdata_t;
  signal cache_index  : std_ulogic_vector(cache_index_size_c-1 downto 0);
  signal cache_offset : std_ulogic_vector(cache_offset_size_c-1 downto 0);
  signal cache_addr   : std_ulogic_vector((cache_index_size_c+cache_offset_size_c)-1 downto 0); -- index & offset
  signal set_select   : std_ulogic;

  -- access history --
  type history_t is record
    re_ff          : std_ulogic;
    last_used_set  : std_ulogic_vector(ICACHE_NUM_BLOCKS-1 downto 0);
    to_be_replaced : std_ulogic;
  end record;
  signal history : history_t;

begin

  -- Access Address Decomposition -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  host_acc_addr.tag    <= host_addr_i(31 downto 31-(cache_tag_size_c-1));
  host_acc_addr.index  <= host_addr_i(31-cache_tag_size_c downto 2+cache_offset_size_c);
  host_acc_addr.offset <= host_addr_i(2+(cache_offset_size_c-1) downto 2); -- discard byte offset

  ctrl_acc_addr.tag    <= ctrl_addr_i(31 downto 31-(cache_tag_size_c-1));
  ctrl_acc_addr.index  <= ctrl_addr_i(31-cache_tag_size_c downto 2+cache_offset_size_c);
  ctrl_acc_addr.offset <= ctrl_addr_i(2+(cache_offset_size_c-1) downto 2); -- discard byte offset


  -- Cache Access History -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  access_history: process(clk_i)
  begin
    if rising_edge(clk_i) then
      history.re_ff <= host_re_i;
      if (clear_i = '1') then -- invalidate cache
        history.last_used_set <= (others => '1');
      elsif (history.re_ff = '1') and (or_reduce_f(hit) = '1') and (ctrl_en_i = '0') then -- store last accessed set that caused a hit
        history.last_used_set(to_integer(unsigned(cache_index))) <= not hit(0);
      end if;
      history.to_be_replaced <= history.last_used_set(to_integer(unsigned(cache_index)));
    end if;
  end process access_history;

  -- which set is going to be replaced? -> opposite of last used set = least recently used set --
  set_select <= '0' when (ICACHE_NUM_SETS = 1) else (not history.to_be_replaced);


  -- Status Flag Memory ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  status_memory: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- write access --
      if (clear_i = '1') then -- invalidate cache
        valid_flag_s0 <= (others => '0');
        valid_flag_s1 <= (others => '0');
      elsif (ctrl_en_i = '1') and (ctrl_we_i = '1') then -- make current block valid
        if (set_select = '0') then
          valid_flag_s0(to_integer(unsigned(cache_index))) <= '1';
        else
          valid_flag_s1(to_integer(unsigned(cache_index))) <= '1';
        end if;
      end if;
      -- sync read access --
      valid(0) <= valid_flag_s0(to_integer(unsigned(cache_index)));
      valid(1) <= valid_flag_s1(to_integer(unsigned(cache_index)));
    end if;
  end process status_memory;


  -- Tag Memory -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tag_memory: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (ctrl_en_i = '1') and (ctrl_we_i = '1') then -- write access
        if (set_select = '0') then
          tag_mem_s0(to_integer(unsigned(cache_index))) <= ctrl_acc_addr.tag;
        else
          tag_mem_s1(to_integer(unsigned(cache_index))) <= ctrl_acc_addr.tag;
        end if;
      end if;
      tag(0) <= tag_mem_s0(to_integer(unsigned(cache_index)));
      tag(1) <= tag_mem_s1(to_integer(unsigned(cache_index)));
    end if;
  end process tag_memory;

  -- comparator --
  comparator: process(host_acc_addr, tag, valid)
  begin
    hit <= (others => '0');
    for i in 0 to ICACHE_NUM_SETS-1 loop
      if (host_acc_addr.tag = tag(i)) and (valid(i) = '1') then
        hit(i) <= '1';
      end if;
    end loop; -- i
  end process comparator;

  -- global hit --
  hit_o <= '1' when (or_reduce_f(hit) = '1') else '0';


  -- Cache Data Memory ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cache_mem_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (ctrl_we_i = '1') then -- write access from control (full-word)
        if (set_select = '0') or (ICACHE_NUM_SETS = 1) then
          cache_data_memory_s0(to_integer(unsigned(cache_addr))) <= ctrl_wstat_i & ctrl_wdata_i;
        else
          cache_data_memory_s1(to_integer(unsigned(cache_addr))) <= ctrl_wstat_i & ctrl_wdata_i;
        end if;
      end if;
      -- read access from host (full-word) --
      cache_rd(0) <= cache_data_memory_s0(to_integer(unsigned(cache_addr)));
      cache_rd(1) <= cache_data_memory_s1(to_integer(unsigned(cache_addr)));
    end if;
  end process cache_mem_access;

  -- data output --
  host_rdata_o <= cache_rd(0)(31 downto 0) when (hit(0) = '1') or (ICACHE_NUM_SETS = 1) else cache_rd(1)(31 downto 0);
  host_rstat_o <= cache_rd(0)(32)          when (hit(0) = '1') or (ICACHE_NUM_SETS = 1) else cache_rd(1)(32);

  -- cache block ram access address --
  cache_addr <= cache_index & cache_offset;

  -- cache access select --
  cache_index  <= host_acc_addr.index  when (ctrl_en_i = '0') else ctrl_acc_addr.index;
  cache_offset <= host_acc_addr.offset when (ctrl_en_i = '0') else ctrl_acc_addr.offset;


end neorv32_icache_memory_rtl;
