-- ================================================================================ --
-- NEORV32 SoC - Generic Cache                                                      --
-- -------------------------------------------------------------------------------- --
-- Configurable generic cache module. The cache is direct-mapped and implements     --
-- "write-through" write strategy. Cached read accesses are implemented as bursts.  --
--                                                                                  --
-- Uncached / direct accesses: Several bus transaction types will bypass the cache: --
-- * atomic memory operations                                                       --
-- * accesses to the explicit "uncached address space page" (or higher),            --
--   which is defined by the 4 most significant address bits (UC_BEGIN)             --
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

entity neorv32_cache is
  generic (
    NUM_BLOCKS : natural range 2 to 1024;       -- number of cache blocks (min 2), has to be a power of 2
    BLOCK_SIZE : natural range 4 to 32768;      -- cache block size in bytes (min 4), has to be a power of 2
    UC_BEGIN   : std_ulogic_vector(3 downto 0); -- begin of uncached address space (page number / 4 MSBs of address)
    READ_ONLY  : boolean                        -- read-only accesses for host
  );
  port (
    clk_i      : in  std_ulogic; -- global clock, rising edge
    rstn_i     : in  std_ulogic; -- global reset, low-active, async
    host_req_i : in  bus_req_t;  -- host request
    host_rsp_o : out bus_rsp_t;  -- host response
    bus_req_o  : out bus_req_t;  -- bus request
    bus_rsp_i  : in  bus_rsp_t   -- bus response
  );
end neorv32_cache;

architecture neorv32_cache_rtl of neorv32_cache is

  -- make sure cache sizes are a power of two --
  constant block_num_c  : natural := 2**index_size_f(NUM_BLOCKS);
  constant block_size_c : natural := 2**index_size_f(BLOCK_SIZE);

  -- cache layout --
  constant offset_size_c : natural := index_size_f(block_size_c/4); -- word offset
  constant index_size_c  : natural := index_size_f(block_num_c);
  constant tag_size_c    : natural := 32 - (offset_size_c + index_size_c + 2);

  -- cache memory component --
  component neorv32_cache_memory
    generic (
      NUM_BLOCKS : natural;
      BLOCK_SIZE : natural
    );
    port (
      rstn_i  : in  std_ulogic;
      clk_i   : in  std_ulogic;
      clr_i   : in  std_ulogic;
      inv_i   : in  std_ulogic;
      new_i   : in  std_ulogic;
      hit_o   : out std_ulogic;
      addr_i  : in  std_ulogic_vector(31 downto 0);
      we_i    : in  std_ulogic_vector(3 downto 0);
      wdata_i : in  std_ulogic_vector(31 downto 0);
      rdata_o : out std_ulogic_vector(31 downto 0)
    );
  end component;

  -- control -> cache interface --
  type cache_o_t is record
    cmd_clr : std_ulogic;
    cmd_inv : std_ulogic;
    cmd_new : std_ulogic;
    addr    : std_ulogic_vector(31 downto 0);
    data    : std_ulogic_vector(31 downto 0);
    we      : std_ulogic_vector(3 downto 0);
  end record;
  signal cache_o : cache_o_t;

  -- cache -> control interface --
  type cache_i_t is record
    sta_hit : std_ulogic;
    data    : std_ulogic_vector(31 downto 0);
  end record;
  signal cache_i : cache_i_t;

  -- control arbiter --
  type state_t is (
    S_IDLE, S_LOOKUP, S_DIRECT_RSP, S_CLEAR,
    S_DOWNLOAD_START, S_DOWNLOAD_WAIT, S_DOWNLOAD_RUN, S_DOWNLOAD_DONE, WAITING
  );
  type ctrl_t is record
    state    : state_t; -- state machine
    buf_req  : std_ulogic; -- access request buffer
    buf_sync : std_ulogic; -- synchronization request buffer
    buf_err  : std_ulogic; -- bus access error buffer
    buf_dir  : std_ulogic; -- direct/uncached access buffer
    tag      : std_ulogic_vector(tag_size_c-1 downto 0); -- tag
    idx      : std_ulogic_vector(index_size_c-1 downto 0); -- index
    ofs_int  : std_ulogic_vector(offset_size_c-1 downto 0); -- cache address offset
    ofs_ext  : std_ulogic_vector(offset_size_c downto 0); -- bus address offset
  end record;
  signal ctrl, ctrl_nxt : ctrl_t;

begin

  -- Control Engine FSM Sync ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_engine_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl.state    <= S_IDLE;
      ctrl.buf_req  <= '0';
      ctrl.buf_sync <= '0';
      ctrl.buf_err  <= '0';
      ctrl.buf_dir  <= '0';
      ctrl.tag      <= (others => '0');
      ctrl.idx      <= (others => '0');
      ctrl.ofs_int  <= (others => '0');
      ctrl.ofs_ext  <= (others => '0');
    elsif rising_edge(clk_i) then
      ctrl <= ctrl_nxt;
    end if;
  end process ctrl_engine_sync;


  -- Control Engine FSM Comb ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_engine_comb: process(ctrl, host_req_i, cache_i, bus_rsp_i)
  begin
    -- control engine defaults --
    ctrl_nxt.state    <= ctrl.state;
    ctrl_nxt.buf_req  <= ctrl.buf_req or host_req_i.stb;
    ctrl_nxt.buf_sync <= ctrl.buf_sync or host_req_i.fence;
    ctrl_nxt.buf_err  <= ctrl.buf_err;
    ctrl_nxt.buf_dir  <= '0';
    ctrl_nxt.tag      <= ctrl.tag;
    ctrl_nxt.idx      <= ctrl.idx;
    ctrl_nxt.ofs_int  <= ctrl.ofs_int;
    ctrl_nxt.ofs_ext  <= ctrl.ofs_ext;

    -- cache access defaults --
    cache_o.cmd_clr <= '0';
    cache_o.cmd_inv <= '0';
    cache_o.cmd_new <= '0';
    cache_o.addr    <= host_req_i.addr;
    cache_o.we      <= (others => '0');
    cache_o.data    <= host_req_i.data;

    -- host response defaults --
    host_rsp_o.ack  <= '0';
    host_rsp_o.err  <= '0';
    host_rsp_o.data <= cache_i.data; -- cache read data (for cache hit)

    -- bus interface defaults --
    bus_req_o       <= host_req_i;
    bus_req_o.stb   <= '0'; -- no request by default
    bus_req_o.burst <= '0'; -- no burst by default
    bus_req_o.fence <= '0'; -- no fence by default

    -- fsm --
    case ctrl.state is

      when S_IDLE => -- wait for request
      -- ------------------------------------------------------------
        if (host_req_i.fence = '1') or (ctrl.buf_sync = '1') then -- (pending) sync request
          ctrl_nxt.state <= S_CLEAR;
        elsif (host_req_i.stb = '1') or (ctrl.buf_req = '1') then -- (pending) access request
          if (unsigned(host_req_i.addr(31 downto 28)) >= unsigned(UC_BEGIN)) or -- uncached address space
             (host_req_i.amo = '1') then -- atomic access
            ctrl_nxt.buf_dir <= '1';
          end if;
          ctrl_nxt.state <= S_LOOKUP;
        end if;

      when S_LOOKUP => -- check access request
      -- ------------------------------------------------------------
        ctrl_nxt.tag     <= host_req_i.addr(31 downto 32-tag_size_c);
        ctrl_nxt.idx     <= host_req_i.addr((offset_size_c+2+index_size_c)-1 downto offset_size_c+2);
        ctrl_nxt.ofs_ext <= (others => '0');
        ctrl_nxt.ofs_int <= (others => '0');
        ctrl_nxt.buf_req <= '0'; -- access about to be completed
        --
        if (ctrl.buf_dir = '1') then -- direct/uncached access; no cache update
          bus_req_o.stb  <= '1';
          ctrl_nxt.state <= S_DIRECT_RSP;
        elsif (cache_i.sta_hit = '1') then -- cache HIT
          if (host_req_i.rw = '0') or (READ_ONLY = true) then -- read from cache
            host_rsp_o.ack <= '1';
            ctrl_nxt.state <= S_IDLE;
          else -- write to main memory and also to the cache (write-through)
            cache_o.we     <= host_req_i.ben;
            bus_req_o.stb  <= '1';
            ctrl_nxt.state <= S_DIRECT_RSP;
          end if;
        else -- cache MISS
          if (host_req_i.rw = '0') or (READ_ONLY = true) then -- get block from main memory
            ctrl_nxt.state <= S_DOWNLOAD_START;
          else -- write to main memory (write-through); no cache update
            bus_req_o.stb  <= '1';
            ctrl_nxt.state <= S_DIRECT_RSP;
          end if;
        end if;

      when S_DIRECT_RSP => -- wait for direct memory access response
      -- ------------------------------------------------------------
        host_rsp_o <= bus_rsp_i; -- cache bypass
        if (bus_rsp_i.ack = '1') then
          ctrl_nxt.state <= S_IDLE;
        end if;

      when S_CLEAR => -- invalidate entire cache
      -- ------------------------------------------------------------
        if (READ_ONLY = false) then
          bus_req_o.fence <= '1';
        end if;
        cache_o.cmd_clr   <= '1';
        ctrl_nxt.buf_sync <= '0';
        ctrl_nxt.state    <= S_IDLE;

      when S_DOWNLOAD_START => -- start download block transfer
      -- ------------------------------------------------------------
        cache_o.addr    <= ctrl.tag & ctrl.idx & ctrl.ofs_int & "00";
        cache_o.cmd_new <= '1'; -- set new block (set tag and make valid)
        bus_req_o.addr  <= ctrl.tag & ctrl.idx & ctrl.ofs_ext(offset_size_c-1 downto 0) & "00";
        bus_req_o.rw    <= '0'; -- read access
        bus_req_o.stb   <= '1'; -- send initial (burst/locking) request
        bus_req_o.lock  <= '1'; -- this is a locked transfer
        bus_req_o.burst <= '1'; -- this is a burst transfer
        bus_req_o.ben   <= (others => '1'); -- full-word access
        ctrl_nxt.state  <= S_DOWNLOAD_WAIT;

      when S_DOWNLOAD_WAIT => -- wait for exclusive (=locked) bus access
      -- ------------------------------------------------------------
        cache_o.addr    <= ctrl.tag & ctrl.idx & ctrl.ofs_int & "00";
        cache_o.data    <= bus_rsp_i.data;
        cache_o.we      <= (others => '1'); -- just keep writing full words
        bus_req_o.addr  <= ctrl.tag & ctrl.idx & ctrl.ofs_ext(offset_size_c-1 downto 0) & "00";
        bus_req_o.rw    <= '0'; -- read access
        bus_req_o.lock  <= '1'; -- this is a locked transfer
        bus_req_o.burst <= '1'; -- this is a burst transfer
        bus_req_o.ben   <= (others => '1'); -- full-word access
        -- wait for initial ACK to start actual bursting --
        if (bus_rsp_i.ack = '1') then
          ctrl_nxt.buf_err <= bus_rsp_i.err; -- buffer bus error
          ctrl_nxt.ofs_int <= std_ulogic_vector(unsigned(ctrl.ofs_int) + 1);
          ctrl_nxt.ofs_ext <= std_ulogic_vector(unsigned(ctrl.ofs_ext) + 1);
          ctrl_nxt.state   <= S_DOWNLOAD_RUN;
        end if;

      when S_DOWNLOAD_RUN => -- send read requests and get data responses
      -- ------------------------------------------------------------
        cache_o.addr    <= ctrl.tag & ctrl.idx & ctrl.ofs_int & "00";
        cache_o.data    <= bus_rsp_i.data;
        cache_o.we      <= (others => '1'); -- just keep writing full words
        bus_req_o.addr  <= ctrl.tag & ctrl.idx & ctrl.ofs_ext(offset_size_c-1 downto 0) & "00";
        bus_req_o.rw    <= '0'; -- read access
        bus_req_o.lock  <= '1'; -- this is a locked transfer
        bus_req_o.burst <= '1'; -- this is a burst transfer
        bus_req_o.ben   <= (others => '1'); -- full-word access
        -- send requests --
        if (ctrl.ofs_ext(offset_size_c) = '0') then
          ctrl_nxt.ofs_ext <= std_ulogic_vector(unsigned(ctrl.ofs_ext) + 1); -- next cache word
          bus_req_o.stb    <= '1'; -- request next transfer
        end if;
        -- receive responses --
        if (bus_rsp_i.ack = '1') then
          ctrl_nxt.buf_err <= ctrl.buf_err or bus_rsp_i.err; -- accumulate bus errors
          ctrl_nxt.ofs_int <= std_ulogic_vector(unsigned(ctrl.ofs_int) + 1); -- next main memory location
          if (and_reduce_f(ctrl.ofs_int) = '1') then -- block completed
            ctrl_nxt.state <= S_DOWNLOAD_DONE;
          end if;
        end if;

      when S_DOWNLOAD_DONE => -- delay cycle for host cache access
      -- ------------------------------------------------------------
        if (ctrl.buf_err = '1') then -- any errors during the burst?
          cache_o.cmd_inv <= '1'; -- invalidate the downloaded cache block
          host_rsp_o.ack  <= '1';
          host_rsp_o.err  <= '1';
          ctrl_nxt.state  <= S_IDLE;
        else
          ctrl_nxt.state <= S_LOOKUP;
        end if;

      when others => -- undefined
      -- ------------------------------------------------------------
        ctrl_nxt.state <= S_IDLE;

    end case;
  end process ctrl_engine_comb;


  -- Cache Memory Core ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cache_memory_inst: neorv32_cache_memory
  generic map (
    NUM_BLOCKS => block_num_c, -- number of blocks (min 2), has to be a power of 2
    BLOCK_SIZE => block_size_c -- block size in bytes (min 4), has to be a power of 2
  )
  port map (
    -- global control --
    rstn_i  => rstn_i,          -- global reset, async, low-active
    clk_i   => clk_i,           -- global clock, rising edge
    -- management --
    clr_i   => cache_o.cmd_clr, -- clear entire cache
    inv_i   => cache_o.cmd_inv, -- invalidate accessed block
    new_i   => cache_o.cmd_new, -- make accessed block valid and set tag
    hit_o   => cache_i.sta_hit, -- cache hit
    -- cache access --
    addr_i  => cache_o.addr,    -- access address
    we_i    => cache_o.we,      -- byte-wide data write enable
    wdata_i => cache_o.data,    -- write data
    rdata_o => cache_i.data     -- read data
  );

end neorv32_cache_rtl;


-- ================================================================================ --
-- NEORV32 SoC - Generic Cache: Data and Status Memory (direct-mapped)              --
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

entity neorv32_cache_memory is
  generic (
    NUM_BLOCKS : natural; -- number of blocks (min 2), has to be a power of 2
    BLOCK_SIZE : natural  -- block size in bytes (min 4), has to be a power of 2
  );
  port (
    -- global control --
    rstn_i  : in  std_ulogic;                     -- global reset, async, low-active
    clk_i   : in  std_ulogic;                     -- global clock, rising edge
    -- management --
    clr_i   : in  std_ulogic;                     -- clear entire cache
    inv_i   : in  std_ulogic;                     -- invalidate accessed block
    new_i   : in  std_ulogic;                     -- make accessed block valid and set tag
    hit_o   : out std_ulogic;                     -- cache hit
    -- cache access --
    addr_i  : in  std_ulogic_vector(31 downto 0); -- access address
    we_i    : in  std_ulogic_vector(3 downto 0);  -- byte-wide data write enable
    wdata_i : in  std_ulogic_vector(31 downto 0); -- write data
    rdata_o : out std_ulogic_vector(31 downto 0)  -- read data
  );
end neorv32_cache_memory;

architecture neorv32_cache_memory_rtl of neorv32_cache_memory is

  -- cache layout --
  constant offset_size_c : natural := index_size_f(BLOCK_SIZE/4); -- offset addresses full 32-bit words
  constant index_size_c  : natural := index_size_f(NUM_BLOCKS); -- index size
  constant tag_size_c    : natural := 32 - (offset_size_c + index_size_c + 2); -- +2 bits for byte offset

  -- status flag memory --
  signal valid_mem : std_ulogic_vector(NUM_BLOCKS-1 downto 0);
  signal valid_mem_rd : std_ulogic;

  -- tag memory --
  type tag_mem_t is array (0 to NUM_BLOCKS-1) of std_ulogic_vector(tag_size_c-1 downto 0);
  signal tag_mem : tag_mem_t;
  signal tag_mem_rd : std_ulogic_vector(tag_size_c-1 downto 0);

  -- data memory --
  type data_mem_t is array (0 to (NUM_BLOCKS * (BLOCK_SIZE/4))-1) of std_ulogic_vector(7 downto 0);
  signal data_mem_b0, data_mem_b1, data_mem_b2, data_mem_b3 : data_mem_t; -- byte-wide sub-memories

  -- cache access --
  signal acc_tag : std_ulogic_vector(tag_size_c-1 downto 0);
  signal tag_ff  : std_ulogic_vector(tag_size_c-1 downto 0);
  signal acc_idx : std_ulogic_vector(index_size_c-1 downto 0);
  signal acc_off : std_ulogic_vector(offset_size_c-1 downto 0);
  signal acc_adr : std_ulogic_vector((index_size_c+offset_size_c)-1 downto 0);

begin

  -- Access Address Decomposition -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_tag <= addr_i(31 downto 31-(tag_size_c-1)); -- tag
  acc_idx <= addr_i(31-tag_size_c downto 2+offset_size_c); -- index (cache block select)
  acc_off <= addr_i(2+(offset_size_c-1) downto 2); -- word offset within block
  acc_adr <= acc_idx & acc_off; -- RAM address

  -- tag pipeline stage --
  tag_buffer: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      tag_ff <= (others => '0');
    elsif rising_edge(clk_i) then
      tag_ff <= acc_tag;
    end if;
  end process tag_buffer;


  -- Status Memory --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  status_memory: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      valid_mem    <= (others => '0');
      valid_mem_rd <= '0';
    elsif rising_edge(clk_i) then
      if (clr_i = '1') then -- invalidate entire cache
        valid_mem <= (others => '0');
      elsif (inv_i = '1') then -- invalidate accessed block
        valid_mem(to_integer(unsigned(acc_idx))) <= '0';
      elsif (new_i = '1') then -- make accessed block valid
        valid_mem(to_integer(unsigned(acc_idx))) <= '1';
      end if;
      valid_mem_rd <= valid_mem(to_integer(unsigned(acc_idx)));
    end if;
  end process status_memory;


  -- Tag Memory -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tag_memory: process(clk_i) -- no reset to allow inferring of blockRAM
  begin
    if rising_edge(clk_i) then
      if (new_i = '1') then -- set new cache entry
        tag_mem(to_integer(unsigned(acc_idx))) <= acc_tag;
      end if;
      tag_mem_rd <= tag_mem(to_integer(unsigned(acc_idx)));
    end if;
  end process tag_memory;

  -- access status (1 cycle latency due to sync memory read) --
  hit_o <= '1' when (valid_mem_rd = '1') and (tag_mem_rd = tag_ff) else '0'; -- cache access hit


  -- Data Memory ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  data_memory: process(clk_i) -- no reset to allow inferring of blockRAM
  begin
    if rising_edge(clk_i) then
      if (we_i(0) = '1') then
        data_mem_b0(to_integer(unsigned(acc_adr))) <= wdata_i(7 downto 0);
      end if;
      if (we_i(1) = '1') then
        data_mem_b1(to_integer(unsigned(acc_adr))) <= wdata_i(15 downto 8);
      end if;
      if (we_i(2) = '1') then
        data_mem_b2(to_integer(unsigned(acc_adr))) <= wdata_i(23 downto 16);
      end if;
      if (we_i(3) = '1') then
        data_mem_b3(to_integer(unsigned(acc_adr))) <= wdata_i(31 downto 24);
      end if;
      rdata_o( 7 downto  0) <= data_mem_b0(to_integer(unsigned(acc_adr)));
      rdata_o(15 downto  8) <= data_mem_b1(to_integer(unsigned(acc_adr)));
      rdata_o(23 downto 16) <= data_mem_b2(to_integer(unsigned(acc_adr)));
      rdata_o(31 downto 24) <= data_mem_b3(to_integer(unsigned(acc_adr)));
    end if;
  end process data_memory;


end neorv32_cache_memory_rtl;
