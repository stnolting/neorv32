-- ================================================================================ --
-- NEORV32 - Generic Cache                                                          --
-- -------------------------------------------------------------------------------- --
-- Configurable generic cache module. The cache is direct-mapped and implements     --
-- "write-back" and "write-allocate" strategies.                                    --
--                                                                                  --
-- All requests targeting the "uncached address space page" (or higher), defined by --
-- the 4 most significant address bits, well as all atomic (reservation set)        --
-- operations will always **bypass** the cache resulting in "direct accesses".      --
--                                                                                  --
-- A fence request will first flush the data cache (write back modified blocks to   --
-- main memory before invalidating all cache blocks to force a re-fetch from main   --
-- memory. After this, the fence request is forwarded to the downstream memory      --
-- system.                                                                          --
--                                                                                  --
-- Simplified cache architecture ("-->" = direction of access requests):            --
--                                                                                  --
--               Direct Access        +----------+                                  --
--          /|----------------------->| Register |---------------------->|\         --
--         | |                        +----------+                       | |        --
-- Host -->| |                                                           | |--> Bus --
--         | |   +--------------+   +--------------+   +-------------+   | |        --
--          \|-->| Host Arbiter |-->| Cache Memory |<--| Bus Arbiter |-->|/         --
--               +--------------+   +--------------+   +-------------+              --
--                                                                                  --
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
    UC_ENABLE  : boolean;                       -- enable direct/uncached accesses
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

  -- host access arbiter (handle CPU accesses to cache) --
  component neorv32_cache_host
  generic (
    READ_ONLY : boolean
  );
  port (
    rstn_i     : in  std_ulogic;
    clk_i      : in  std_ulogic;
    req_i      : in  bus_req_t;
    rsp_o      : out bus_rsp_t;
    bus_sync_o : out std_ulogic;
    bus_miss_o : out std_ulogic;
    bus_busy_i : in  std_ulogic;
    dirty_o    : out std_ulogic;
    hit_i      : in  std_ulogic;
    addr_o     : out std_ulogic_vector(31 downto 0);
    we_o       : out std_ulogic_vector(3 downto 0);
    swe_o      : out std_ulogic;
    wdata_o    : out std_ulogic_vector(31 downto 0);
    wstat_o    : out std_ulogic;
    rdata_i    : in  std_ulogic_vector(31 downto 0);
    rstat_i    : in  std_ulogic
  );
  end component;

  -- cache memory core (cache memory and management) --
  component neorv32_cache_memory
  generic (
    NUM_BLOCKS : natural;
    BLOCK_SIZE : natural;
    READ_ONLY  : boolean
  );
  port (
    rstn_i   : in  std_ulogic;
    clk_i    : in  std_ulogic;
    inval_i  : in  std_ulogic;
    new_i    : in  std_ulogic;
    dirty_i  : in  std_ulogic;
    hit_o    : out std_ulogic;
    dirty_o  : out std_ulogic;
    base_o   : out std_ulogic_vector(31 downto 0);
    addr_i   : in  std_ulogic_vector(31 downto 0);
    we_i     : in  std_ulogic_vector(3 downto 0);
    swe_i    : in  std_ulogic;
    wdata_i  : in  std_ulogic_vector(31 downto 0);
    wstat_i  : in  std_ulogic;
    rdata_o  : out std_ulogic_vector(31 downto 0);
    rstat_o  : out std_ulogic
  );
  end component;

  -- bus access arbiter (handle cache misses) --
  component neorv32_cache_bus
  generic (
    NUM_BLOCKS : natural;
    BLOCK_SIZE : natural;
    READ_ONLY  : boolean
  );
  port (
    rstn_i     : in  std_ulogic;
    clk_i      : in  std_ulogic;
    host_req_i : in  bus_req_t;
    bus_req_o  : out bus_req_t;
    bus_rsp_i  : in  bus_rsp_t;
    cmd_sync_i : in  std_ulogic;
    cmd_miss_i : in  std_ulogic;
    cmd_busy_o : out std_ulogic;
    inval_o    : out std_ulogic;
    new_o      : out std_ulogic;
    dirty_i    : in  std_ulogic;
    base_i     : in  std_ulogic_vector(31 downto 0);
    addr_o     : out std_ulogic_vector(31 downto 0);
    we_o       : out std_ulogic_vector(3 downto 0);
    swe_o      : out std_ulogic;
    wdata_o    : out std_ulogic_vector(31 downto 0);
    wstat_o    : out std_ulogic;
    rdata_i    : in  std_ulogic_vector(31 downto 0)
  );
  end component;

  -- make sure cache sizes are a power of two --
  constant block_num_c  : natural := 2**index_size_f(NUM_BLOCKS);
  constant block_size_c : natural := 2**index_size_f(BLOCK_SIZE);

  -- bus de-mux control for direct/uncached or caches access --
  signal dir_acc_d, dir_acc_q : std_ulogic;

  -- internal bus system --
  signal bus_req, dir_req_d, dir_req_q, cache_req : bus_req_t;
  signal bus_rsp, dir_rsp_d, dir_rsp_q, cache_rsp : bus_rsp_t;

  -- cache memory module interface --
  type cache_in_t is record
    addr  : std_ulogic_vector(31 downto 0);
    we    : std_ulogic_vector(3 downto 0);
    swe   : std_ulogic;
    wdata : std_ulogic_vector(31 downto 0);
    wstat : std_ulogic;
  end record;
  signal cache_in_host, cache_in_bus, cache_in : cache_in_t;
  --
  type cache_out_t is record
    rdata : std_ulogic_vector(31 downto 0);
    rstat : std_ulogic;
  end record;
  signal cache_out : cache_out_t;

  -- cache status --
  signal cache_stat_dirty, cache_stat_hit : std_ulogic;
  signal cache_stat_base : std_ulogic_vector(31 downto 0);

  -- operation commands --
  signal cache_cmd_inval, cache_cmd_new, cache_cmd_dirty, bus_cmd_sync, bus_cmd_miss, bus_cmd_busy : std_ulogic;

begin

  -- Check if Direct/Uncached Access --------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  dir_acc_d <= '1' when UC_ENABLE and -- direct accesses implemented
                        ((unsigned(host_req_i.addr(31 downto 28)) >= unsigned(UC_BEGIN)) or -- uncached memory page
                         (host_req_i.amo = '1')) else '0'; -- atomic memory operation

  -- request splitter: cached or direct access --
  req_splitter: process(host_req_i, dir_acc_d)
  begin
    -- default: pass-through all bus signals --
    cache_req <= host_req_i;
    dir_req_d <= host_req_i;
    -- direct access --
    dir_req_d.stb   <= host_req_i.stb and dir_acc_d;
    dir_req_d.fence <= '0'; -- no fence requests from this side
    -- cached access --
    cache_req.stb <= host_req_i.stb and (not dir_acc_d);
  end process req_splitter;

  -- direct/uncached access path pipeline stage --
  direct_acc_enable:
  if UC_ENABLE generate
    bus_buffer: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        dir_acc_q <= '0';
        dir_req_q <= req_terminate_c;
        dir_rsp_q <= rsp_terminate_c;
      elsif rising_edge(clk_i) then
        -- is direct access? --
        if (dir_acc_q = '0') and (host_req_i.stb = '1') and (dir_acc_d = '1') then
          dir_acc_q <= '1';
        elsif (dir_acc_q = '1') and ((dir_rsp_q.ack = '1') or (dir_rsp_q.err = '1')) then
          dir_acc_q <= '0';
        end if;
        -- bus request buffer --
        if READ_ONLY then -- do not propagate STB on write access, issue ERR instead
          dir_req_q     <= dir_req_d;
          dir_req_q.stb <= dir_req_d.stb and (not dir_req_d.rw); -- read accesses only
          dir_rsp_q     <= dir_rsp_d;
          dir_rsp_q.err <= dir_rsp_d.err or (dir_req_d.stb and dir_req_d.rw); -- error on write access
        else
          dir_req_q <= dir_req_d;
          dir_rsp_q <= dir_rsp_d;
        end if;
      end if;
    end process bus_buffer;

    -- internal response switch --
    host_rsp_o <= cache_rsp when (dir_acc_q = '0') else dir_rsp_q;
  end generate;

  -- direct accesses not implemented --
  direct_acc_disable:
  if not UC_ENABLE generate
    dir_req_q  <= req_terminate_c;
    host_rsp_o <= cache_rsp;
  end generate;


  -- Host Access Arbiter (Handle *Cached* CPU Bus Requests) ---------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cache_host_inst: neorv32_cache_host
  generic map (
    READ_ONLY => READ_ONLY -- host accesses are read-only
  )
  port map (
    -- global control --
    rstn_i     => rstn_i,               -- global reset, async, low-active
    clk_i      => clk_i,                -- global clock, rising edge
    -- host access port --
    req_i      => cache_req,            -- request
    rsp_o      => cache_rsp,            -- response
    -- bus unit interface --
    bus_sync_o => bus_cmd_sync,         -- sync cache and main memory
    bus_miss_o => bus_cmd_miss,         -- cache miss
    bus_busy_i => bus_cmd_busy,         -- bus operation in progress
    -- cache status interface --
    dirty_o    => cache_cmd_dirty,      -- make accessed block dirty
    hit_i      => cache_stat_hit,       -- cache hit
    -- cache data interface --
    addr_o     => cache_in_host.addr,   -- access address
    we_o       => cache_in_host.we,     -- byte-wide data write enable
    swe_o      => cache_in_host.swe,    -- status write enable
    wdata_o    => cache_in_host.wdata,  -- write data
    wstat_o    => cache_in_host.wstat,  -- write status
    rdata_i    => cache_out.rdata,      -- read data
    rstat_i    => cache_out.rstat       -- read status
  );


  -- Cache Memory Core (Cache Storage and Status Management) --------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cache_memory_inst: neorv32_cache_memory
  generic map (
    NUM_BLOCKS => block_num_c,  -- number of blocks (min 2), has to be a power of 2
    BLOCK_SIZE => block_size_c, -- block size in bytes (min 4), has to be a power of 2
    READ_ONLY  => READ_ONLY     -- cache is read-only (for host)
  )
  port map (
    -- global control --
    rstn_i   => rstn_i,           -- global reset, async, low-active
    clk_i    => clk_i,            -- global clock, rising edge
    -- management --
    inval_i  => cache_cmd_inval,  -- make accessed block invalid
    new_i    => cache_cmd_new,    -- make accessed block valid, clean and set tag
    dirty_i  => cache_cmd_dirty,  -- make accessed block dirty
    -- status --
    hit_o    => cache_stat_hit,   -- cache hit
    dirty_o  => cache_stat_dirty, -- accessed block is dirty
    base_o   => cache_stat_base,  -- base address of current block
    -- cache access --
    addr_i   => cache_in.addr,    -- access address
    we_i     => cache_in.we,      -- byte-wide data write enable
    swe_i    => cache_in.swe,     -- status write enable
    wdata_i  => cache_in.wdata,   -- write data
    wstat_i  => cache_in.wstat,   -- write status
    rdata_o  => cache_out.rdata,  -- read data
    rstat_o  => cache_out.rstat   -- read status
  );

  -- cache access switch --
  cache_in <= cache_in_host when (bus_cmd_busy = '0') else cache_in_bus;


  -- Bus Access Arbiter (Handle Cache Miss and Flush/Reload) --------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cache_bus_inst: neorv32_cache_bus
  generic map (
    NUM_BLOCKS => block_num_c,  -- number of blocks (min 2), has to be a power of 2
    BLOCK_SIZE => block_size_c, -- block size in bytes (min 4), has to be a power of 2
    READ_ONLY  => READ_ONLY     -- read-only bus accesses
  )
  port map (
    -- global control --
    rstn_i     => rstn_i,              -- global reset, async, low-active
    clk_i      => clk_i,               -- global clock, rising edge
    -- host access port --
    host_req_i => host_req_i,          -- request
    -- bus access port --
    bus_req_o  => bus_req,             -- request
    bus_rsp_i  => bus_rsp,             -- response
    -- operation interface --
    cmd_sync_i => bus_cmd_sync,        -- sync cache and main memory
    cmd_miss_i => bus_cmd_miss,        -- cache miss
    cmd_busy_o => bus_cmd_busy,        -- bus operation in progress
    -- cache status interface --
    inval_o    => cache_cmd_inval,     -- invalidate accessed block
    new_o      => cache_cmd_new,       -- set new cache entry
    dirty_i    => cache_stat_dirty,    -- accessed block is dirty
    base_i     => cache_stat_base,     -- base address of accessed block
    -- cache data interface --
    addr_o     => cache_in_bus.addr,   -- access address
    we_o       => cache_in_bus.we,     -- byte-wide data write enable
    swe_o      => cache_in_bus.swe,    -- status write enable
    wdata_o    => cache_in_bus.wdata,  -- write data
    wstat_o    => cache_in_bus.wstat,  -- write status
    rdata_i    => cache_out.rdata      -- read data
  );


  -- Bus Access Switch ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_switch_enable:
  if UC_ENABLE generate
    -- Use a real switch here to buffer direct access requests during
    -- out-of-band cache operations (downstream memory synchronization).
    neorv32_cache_bus_switch: entity neorv32.neorv32_bus_switch
    generic map (
      PORT_A_READ_ONLY => READ_ONLY,
      PORT_B_READ_ONLY => READ_ONLY
    )
    port map (
      clk_i    => clk_i,
      rstn_i   => rstn_i,
      a_lock_i => bus_cmd_busy, -- cache accesses have exclusive access
      a_req_i  => bus_req,
      a_rsp_o  => bus_rsp,
      b_req_i  => dir_req_q,
      b_rsp_o  => dir_rsp_d,
      x_req_o  => bus_req_o,
      x_rsp_i  => bus_rsp_i
    );
  end generate;

  bus_switch_disable:
  if not UC_ENABLE generate
    bus_req_o <= bus_req;
    bus_rsp   <= bus_rsp_i;
  end generate;


end neorv32_cache_rtl;


-- ================================================================================ --
-- NEORV32 CPU - Generic Cache: Host Access Controller                              --
-- -------------------------------------------------------------------------------- --
-- Handle host accesses to the cache (check for hit/miss) or bypass cache if        --
-- direct/uncached access. If a cache miss occurs or a fence request is received an --
-- according command is sent to the bus interface unit.                             --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cache_host is
  generic (
    READ_ONLY : boolean -- host accesses are read-only
  );
  port (
    -- global control --
    rstn_i     : in  std_ulogic;                     -- global reset, async, low-active
    clk_i      : in  std_ulogic;                     -- global clock, rising edge
    -- host access port --
    req_i      : in  bus_req_t;                      -- request
    rsp_o      : out bus_rsp_t;                      -- response
    -- bus unit interface --
    bus_sync_o : out std_ulogic;                     -- sync cache and main memory
    bus_miss_o : out std_ulogic;                     -- cache miss
    bus_busy_i : in  std_ulogic;                     -- bus operation in progress
    -- cache status interface --
    dirty_o    : out std_ulogic;                     -- make accessed block dirty
    hit_i      : in  std_ulogic;                     -- cache hit
    -- cache data interface --
    addr_o     : out std_ulogic_vector(31 downto 0); -- access address
    we_o       : out std_ulogic_vector(3 downto 0);  -- byte-wide data write enable
    swe_o      : out std_ulogic;                     -- status write enable
    wdata_o    : out std_ulogic_vector(31 downto 0); -- write data
    wstat_o    : out std_ulogic;                     -- write status
    rdata_i    : in  std_ulogic_vector(31 downto 0); -- read data
    rstat_i    : in  std_ulogic                      -- read status
  );
end neorv32_cache_host;

architecture neorv32_cache_host_rtl of neorv32_cache_host is

  -- control engine --
  type ctrl_state_t is (S_IDLE, S_CHECK, S_WAIT_MISS, S_WAIT_SYNC, S_ERROR);
  type ctrl_t is record
    state,    state_nxt    : ctrl_state_t; -- FSM state
    req_buf,  req_buf_nxt  : std_ulogic; -- access request buffer
    sync_buf, sync_buf_nxt : std_ulogic; -- flush/reload (sync with main memory) request buffer
  end record;
  signal ctrl : ctrl_t;

begin

  -- Control Engine FSM Sync ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_engine_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl.state    <= S_IDLE;
      ctrl.req_buf  <= '0';
      ctrl.sync_buf <= '0';
    elsif rising_edge(clk_i) then
      ctrl.state    <= ctrl.state_nxt;
      ctrl.req_buf  <= ctrl.req_buf_nxt;
      ctrl.sync_buf <= ctrl.sync_buf_nxt;
    end if;
  end process ctrl_engine_sync;


  -- Control Engine FSM Comb ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_engine_comb: process(ctrl, req_i, hit_i, rdata_i, rstat_i, bus_busy_i)
  begin
    -- control defaults --
    ctrl.state_nxt    <= ctrl.state;
    ctrl.req_buf_nxt  <= ctrl.req_buf or req_i.stb;
    ctrl.sync_buf_nxt <= ctrl.sync_buf or req_i.fence;

    -- cache access defaults --
    dirty_o <= '0';
    addr_o  <= req_i.addr;
    we_o    <= (others => '0');
    swe_o   <= '0'; -- host cannot alter status bits
    wdata_o <= req_i.data;
    wstat_o <= '0'; -- host cannot alter status bits

    -- bus unit command defaults --
    bus_sync_o <= '0';
    bus_miss_o <= '0';

    -- host interface defaults --
    rsp_o <= rsp_terminate_c;

    -- fsm --
    case ctrl.state is

      when S_IDLE => -- wait for host request
      -- ------------------------------------------------------------
        if (ctrl.sync_buf = '1') then -- flush and reload cache (sync with main memory)
          bus_sync_o     <= '1'; -- trigger bus unit: sync operation
          ctrl.state_nxt <= S_WAIT_SYNC;
        elsif (req_i.stb = '1') or (ctrl.req_buf = '1') then -- (pending) access request
          if (req_i.rw = '1') and READ_ONLY then -- invalid write access?
            ctrl.state_nxt <= S_ERROR;
          else
            ctrl.state_nxt <= S_CHECK;
          end if;
        end if;

      when S_CHECK => -- check if cache hit
      -- ------------------------------------------------------------
        rsp_o.data       <= rdata_i; -- output read data
        ctrl.req_buf_nxt <= '0'; -- access request completed
        if (hit_i = '1') then
          if (req_i.rw = '1') and (not READ_ONLY) then -- write access
            dirty_o <= '1'; -- cache block is dirty now
            we_o    <= req_i.ben; -- finalize write access
          end if;
          rsp_o.ack      <= not rstat_i; -- data word fine?
          rsp_o.err      <= rstat_i; -- data word faulty?
          ctrl.state_nxt <= S_IDLE;
        else -- cache miss
          bus_miss_o     <= '1'; -- trigger bus unit: cache miss
          ctrl.state_nxt <= S_WAIT_MISS;
        end if;

      when S_WAIT_SYNC => -- wait for bus engine to handle cache sync
      -- ------------------------------------------------------------
        ctrl.sync_buf_nxt <= '0'; -- sync operation has been issued
        if (bus_busy_i = '0') then
          ctrl.state_nxt <= S_IDLE;
        end if;

      when S_WAIT_MISS => -- wait for bus engine to handle cache miss
      -- ------------------------------------------------------------
        if (bus_busy_i = '0') then
          ctrl.state_nxt <= S_CHECK; -- redo cache access
        end if;

      when S_ERROR => -- access error
      -- ------------------------------------------------------------
        rsp_o.err      <= '1';
        ctrl.state_nxt <= S_IDLE;

      when others => -- undefined
      -- ------------------------------------------------------------
        ctrl.state_nxt <= S_IDLE;

    end case;
  end process ctrl_engine_comb;


end neorv32_cache_host_rtl;


-- ================================================================================ --
-- NEORV32 CPU - Generic Cache: Data and Status Memory (direct-mapped)              --
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
    BLOCK_SIZE : natural; -- block size in bytes (min 4), has to be a power of 2
    READ_ONLY  : boolean  -- cache is read-only (for host)
  );
  port (
    -- global control --
    rstn_i   : in  std_ulogic;                     -- global reset, async, low-active
    clk_i    : in  std_ulogic;                     -- global clock, rising edge
    -- management --
    inval_i  : in  std_ulogic;                     -- make accessed block invalid
    new_i    : in  std_ulogic;                     -- make accessed block valid, clean and set tag
    dirty_i  : in  std_ulogic;                     -- make accessed block dirty
    -- status --
    hit_o    : out std_ulogic;                     -- cache hit
    dirty_o  : out std_ulogic;                     -- accessed block is dirty
    base_o   : out std_ulogic_vector(31 downto 0); -- base address of current block
    -- cache access --
    addr_i   : in  std_ulogic_vector(31 downto 0); -- access address
    we_i     : in  std_ulogic_vector(3 downto 0);  -- byte-wide data write enable
    swe_i    : in  std_ulogic;                     -- status write enable
    wdata_i  : in  std_ulogic_vector(31 downto 0); -- write data
    wstat_i  : in  std_ulogic;                     -- write status
    rdata_o  : out std_ulogic_vector(31 downto 0); -- read data
    rstat_o  : out std_ulogic                      -- read status
  );
end neorv32_cache_memory;

architecture neorv32_cache_memory_rtl of neorv32_cache_memory is

  -- cache layout --
  constant offset_size_c : natural := index_size_f(BLOCK_SIZE/4); -- offset addresses full 32-bit words
  constant index_size_c  : natural := index_size_f(NUM_BLOCKS); -- index size
  constant tag_size_c    : natural := 32 - (offset_size_c + index_size_c + 2); -- 2 additional bits for byte offset

  -- status flag memory --
  signal valid_mem,    dirty_mem    : std_ulogic_vector(NUM_BLOCKS-1 downto 0);
  signal valid_mem_rd, dirty_mem_rd : std_ulogic;

  -- tag memory --
  type tag_mem_t is array (0 to NUM_BLOCKS-1) of std_ulogic_vector(tag_size_c-1 downto 0);
  signal tag_mem    : tag_mem_t;
  signal tag_mem_rd : std_ulogic_vector(tag_size_c-1 downto 0);

  -- cache data memory --
  type data_mem_t is array (0 to (NUM_BLOCKS * (BLOCK_SIZE/4))-1) of std_ulogic_vector(7 downto 0);
  signal data_mem_b0, data_mem_b1, data_mem_b2, data_mem_b3 : data_mem_t; -- byte-wide sub-memories
  signal data_mem_rd : std_ulogic_vector(31 downto 0);

  -- cache data status memory (used for the bus error response - just mark individual words as faults and not the entire block) --
  signal stat_mem    : std_ulogic_vector((NUM_BLOCKS * (BLOCK_SIZE/4))-1 downto 0);
  signal stat_mem_rd : std_ulogic;

  -- access address decomposition --
  signal acc_tag, acc_tag_ff : std_ulogic_vector(tag_size_c-1 downto 0);
  signal acc_idx, acc_idx_ff : std_ulogic_vector(index_size_c-1 downto 0);
  signal acc_off : std_ulogic_vector(offset_size_c-1 downto 0);
  signal acc_adr : std_ulogic_vector((index_size_c+offset_size_c)-1 downto 0);

begin

  -- Access Address Decomposition -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_tag <= addr_i(31 downto 31-(tag_size_c-1));
  acc_idx <= addr_i(31-tag_size_c downto 2+offset_size_c);
  acc_off <= addr_i(2+(offset_size_c-1) downto 2);
  acc_adr <= acc_idx & acc_off;

  -- access buffer (tag + index) --
  access_buffer: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      acc_tag_ff <= (others => '0');
      acc_idx_ff <= (others => '0');
    elsif rising_edge(clk_i) then
      acc_tag_ff <= acc_tag;
      acc_idx_ff <= acc_idx;
    end if;
  end process access_buffer;


  -- Status Flag Memory ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  status_memory: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      valid_mem <= (others => '0');
      dirty_mem <= (others => '0');
    elsif rising_edge(clk_i) then
      if (new_i = '1') then -- set new block
        valid_mem(to_integer(unsigned(acc_idx))) <= '1'; -- valid
        dirty_mem(to_integer(unsigned(acc_idx))) <= '0'; -- clean
      else
        if (inval_i = '1') then -- invalidate current block
          valid_mem(to_integer(unsigned(acc_idx))) <= '0';
        end if;
        if (dirty_i = '1') then -- make current block dirty
          dirty_mem(to_integer(unsigned(acc_idx))) <= '1';
        end if;
      end if;
      -- sync read --
      valid_mem_rd <= valid_mem(to_integer(unsigned(acc_idx)));
      dirty_mem_rd <= dirty_mem(to_integer(unsigned(acc_idx)));
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


  -- Access Status (1 Cycle Latency) --------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  hit_o   <= '1' when (valid_mem_rd = '1') and (tag_mem_rd = acc_tag_ff) else '0'; -- cache access hit
  dirty_o <= '1' when (valid_mem_rd = '1') and (dirty_mem_rd = '1') and (not READ_ONLY) else '0'; -- accessed block is dirty

  -- base address of accessed block --
  base_o(31 downto 31-(tag_size_c-1))          <= tag_mem_rd;
  base_o(31-tag_size_c downto 2+offset_size_c) <= acc_idx_ff;
  base_o(2+(offset_size_c-1) downto 0)         <= (others => '0');


  -- Cache Data Memory ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cache_mem_access: process(clk_i) -- no reset to allow inferring of blockRAM
  begin
    if rising_edge(clk_i) then
      -- write access --
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
      if (swe_i = '1') then
        stat_mem(to_integer(unsigned(acc_adr))) <= wstat_i;
      end if;
      -- read access --
      data_mem_rd(07 downto 00) <= data_mem_b0(to_integer(unsigned(acc_adr)));
      data_mem_rd(15 downto 08) <= data_mem_b1(to_integer(unsigned(acc_adr)));
      data_mem_rd(23 downto 16) <= data_mem_b2(to_integer(unsigned(acc_adr)));
      data_mem_rd(31 downto 24) <= data_mem_b3(to_integer(unsigned(acc_adr)));
      stat_mem_rd <= stat_mem(to_integer(unsigned(acc_adr)));
    end if;
  end process cache_mem_access;

  -- read-data + status --
  rdata_o <= data_mem_rd;
  rstat_o <= stat_mem_rd and valid_mem_rd;


end neorv32_cache_memory_rtl;


-- ================================================================================ --
-- NEORV32 CPU - Generic Cache: Bus Interface Unit                                  --
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

entity neorv32_cache_bus is
  generic (
    NUM_BLOCKS : natural; -- number of blocks (min 2), has to be a power of 2
    BLOCK_SIZE : natural; -- block size in bytes (min 4), has to be a power of 2
    READ_ONLY  : boolean  -- read-only bus accesses
  );
  port (
    -- global control --
    rstn_i      : in  std_ulogic;                     -- global reset, async, low-active
    clk_i       : in  std_ulogic;                     -- global clock, rising edge
    -- host access port --
    host_req_i  : in  bus_req_t;                      -- request
    -- bus access port --
    bus_req_o   : out bus_req_t;                      -- request
    bus_rsp_i   : in  bus_rsp_t;                      -- response
    -- operation interface --
    cmd_sync_i  : in  std_ulogic;                     -- sync cache and main memory
    cmd_miss_i  : in  std_ulogic;                     -- cache miss
    cmd_busy_o  : out std_ulogic;                     -- bus operation in progress
    -- cache status interface --
    inval_o     : out std_ulogic;                     -- invalidate accessed block
    new_o       : out std_ulogic;                     -- set new cache entry
    dirty_i     : in  std_ulogic;                     -- accessed block is dirty
    base_i      : in  std_ulogic_vector(31 downto 0); -- base address of accessed block
    -- cache data interface --
    addr_o      : out std_ulogic_vector(31 downto 0); -- access address
    we_o        : out std_ulogic_vector(3 downto 0);  -- byte-wide data write enable
    swe_o       : out std_ulogic;                     -- status write enable
    wdata_o     : out std_ulogic_vector(31 downto 0); -- write data
    wstat_o     : out std_ulogic;                     -- write status
    rdata_i     : in  std_ulogic_vector(31 downto 0)  -- read data
  );
end neorv32_cache_bus;

architecture neorv32_cache_bus_rtl of neorv32_cache_bus is

  -- cache layout --
  constant offset_size_c : natural := index_size_f(BLOCK_SIZE/4); -- WORD offset!
  constant index_size_c  : natural := index_size_f(NUM_BLOCKS);
  constant tag_size_c    : natural := 32 - (offset_size_c + index_size_c + 2);

  -- control fsm --
  type state_t is (S_IDLE, S_CHECK, S_DOWNLOAD_REQ, S_DOWNLOAD_RSP, S_UPLOAD_GET,
                   S_UPLOAD_REQ, S_UPLOAD_RSP, S_FLUSH_START, S_FLUSH_READ, S_FLUSH_CHECK);
  signal state, upret, state_nxt, upret_nxt: state_t;

  -- address generator --
  type addr_t is record
    tag : std_ulogic_vector(tag_size_c-1 downto 0);
    idx : std_ulogic_vector(index_size_c-1 downto 0);
    ofs : std_ulogic_vector(offset_size_c-1 downto 0); -- WORD offset!
  end record;
  signal haddr, baddr, addr, addr_nxt : addr_t;

begin

  -- Address Decomposition ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- base address of original host access --
  haddr.tag <= host_req_i.addr(31 downto (32-tag_size_c));
  haddr.idx <= (others => '0'); -- unused
  haddr.ofs <= (others => '0'); -- unused

  -- base address of indexed cache block --
  baddr.tag <= base_i(31 downto (32-tag_size_c));
  baddr.idx <= base_i((offset_size_c+2+index_size_c)-1 downto offset_size_c+2);
  baddr.ofs <= (others => '0'); -- unused


  -- Control Engine FSM Sync ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_engine_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      state    <= S_IDLE;
      upret    <= S_IDLE;
      addr.tag <= (others => '0');
      addr.idx <= (others => '0');
      addr.ofs <= (others => '0');
    elsif rising_edge(clk_i) then
      state <= state_nxt;
      upret <= upret_nxt;
      addr  <= addr_nxt;
    end if;
  end process ctrl_engine_sync;


  -- Control Engine FSM Comb ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_engine_comb: process(state, upret, addr, haddr, baddr, host_req_i, bus_rsp_i, cmd_sync_i, cmd_miss_i, rdata_i, dirty_i)
  begin
    -- control engine defaults --
    state_nxt <= state;
    upret_nxt <= upret;
    addr_nxt  <= addr;

    -- cache access defaults --
    addr_o  <= addr.tag & addr.idx & addr.ofs & "00"; -- always word-aligned
    we_o    <= (others => '0');
    swe_o   <= '0';
    wdata_o <= bus_rsp_i.data;
    wstat_o <= bus_rsp_i.err;

    -- cache command defaults --
    inval_o <= '0';
    new_o   <= '0';

    -- bus interface defaults --
    bus_req_o       <= req_terminate_c; -- all-zero
    bus_req_o.addr  <= addr.tag & addr.idx & addr.ofs & "00"; -- always word-aligned
    bus_req_o.data  <= rdata_i;
    bus_req_o.ben   <= (others => '1'); -- full-word writes only
    bus_req_o.src   <= '0'; -- cache accesses are always data accesses
    bus_req_o.priv  <= '0'; -- cache accesses are always "unprivileged" accesses
    bus_req_o.amo   <= '0'; -- cache accesses can never be an atomic memory operation set operation
    bus_req_o.amoop <= (others => '0'); -- cache accesses can never be an atomic memory operation set operation
    bus_req_o.debug <= host_req_i.debug;
    if (state = S_IDLE) then
      bus_req_o.sleep <= host_req_i.sleep;
    else
      bus_req_o.sleep <= '0';
    end if;

    -- fsm --
    case state is

      when S_IDLE => -- wait for request
      -- ------------------------------------------------------------
        addr_nxt.ofs <= (others => '0'); -- align block base address for upload/download (and flush)
        if (cmd_sync_i = '1') then -- cache sync
          state_nxt <= S_FLUSH_START;
        elsif (cmd_miss_i = '1') then -- cache miss
          state_nxt <= S_CHECK;
        end if;

      when S_CHECK => -- check if accessed block is dirty (cache address is still applied by host controller!)
      -- ------------------------------------------------------------
        upret_nxt    <= S_DOWNLOAD_REQ; -- go straight to S_DOWNLOAD_REQ when S_UPLOAD_GET has completed (if executed)
        addr_nxt.idx <= baddr.idx; -- index of reference cache block
        if (dirty_i = '1') and (not READ_ONLY) then -- block is dirty, upload first
          addr_nxt.tag <= baddr.tag; -- base address (tag + index) of accessed block
          state_nxt    <= S_UPLOAD_GET;
        else -- block is clean, download new block
          addr_nxt.tag <= haddr.tag; -- base address (tag + index) of requested block
          state_nxt    <= S_DOWNLOAD_REQ;
        end if;


      when S_DOWNLOAD_REQ => -- download new cache block: request new word
      -- ------------------------------------------------------------
        bus_req_o.rw  <= '0'; -- read access
        bus_req_o.stb <= '1'; -- request new transfer
        state_nxt     <= S_DOWNLOAD_RSP;

      when S_DOWNLOAD_RSP => -- download new cache block: wait for bus response
      -- ------------------------------------------------------------
        bus_req_o.rw <= '0'; -- read access
        we_o         <= (others => '1'); -- cache: full-word write (write all the time until ACK/ERR)
        swe_o        <= '1'; -- cache: write status bit (bus error response)
        new_o        <= '1'; -- set new block (set tag, make valid, make clean)
        if (bus_rsp_i.ack = '1') or (bus_rsp_i.err = '1') then -- wait for response
          addr_nxt.ofs <= std_ulogic_vector(unsigned(addr.ofs) + 1);
          if (and_reduce_f(addr.ofs) = '1') then -- block completed? offset will be all-zero again after block completion
            state_nxt <= S_IDLE;
          else -- get next word
            state_nxt <= S_DOWNLOAD_REQ;
          end if;
        end if;


      when S_UPLOAD_GET => -- upload dirty cache block: read word from cache
      -- ------------------------------------------------------------
        if READ_ONLY then
          state_nxt <= S_IDLE;
        else
          bus_req_o.rw <= '1'; -- write access
          state_nxt    <= S_UPLOAD_REQ;
        end if;

      when S_UPLOAD_REQ => -- upload dirty cache block: request bus write
      -- ------------------------------------------------------------
        if READ_ONLY then
          state_nxt <= S_IDLE;
        else
          bus_req_o.rw  <= '1'; -- write access
          bus_req_o.stb <= '1'; -- request new transfer
          state_nxt     <= S_UPLOAD_RSP;
        end if;

      when S_UPLOAD_RSP => -- upload dirty cache block: wait for bus response
      -- ------------------------------------------------------------
        if READ_ONLY then
          state_nxt <= S_IDLE;
        else
          bus_req_o.rw <= '1'; -- write access
          new_o        <= '1'; -- set new block (set tag, make valid, make clean)
          if (bus_rsp_i.ack = '1') or (bus_rsp_i.err = '1') then -- wait for response
            addr_nxt.ofs <= std_ulogic_vector(unsigned(addr.ofs) + 1);
            if (and_reduce_f(addr.ofs) = '1') then -- block completed? offset will be all-zero again after block completion
              state_nxt <= upret; -- go back to "upload-done return state"
            else -- get next word
              state_nxt <= S_UPLOAD_GET;
            end if;
          end if;
        end if;


      when S_FLUSH_START => -- start checking for dirty blocks
      -- ------------------------------------------------------------
        addr_nxt.idx <= (others => '0'); -- start with index 0
        upret_nxt    <= S_FLUSH_CHECK; -- come back to S_FLUSH_CHECK after block upload
        state_nxt    <= S_FLUSH_READ;

      when S_FLUSH_READ => -- cache read access latency cycle
      -- ------------------------------------------------------------
        state_nxt <= S_FLUSH_CHECK;

      when S_FLUSH_CHECK => -- check if currently indexed block is dirty
      -- ------------------------------------------------------------
        addr_nxt.tag <= baddr.tag; -- tag of currently index block
        inval_o      <= '1'; -- invalidate currently index block
        if (dirty_i = '1') and (not READ_ONLY) then -- block dirty?
          state_nxt <= S_UPLOAD_GET;
        else -- move on to next block
          addr_nxt.idx <= std_ulogic_vector(unsigned(addr.idx) + 1);
          if (and_reduce_f(addr.idx) = '1') then -- all blocks done?
            bus_req_o.fence <= '1'; -- forward fence request to downstream memories
            state_nxt       <= S_IDLE;
          else -- go to next block
            state_nxt <= S_FLUSH_READ;
          end if;
        end if;


      when others => -- undefined
      -- ------------------------------------------------------------
        state_nxt <= S_IDLE;

    end case;
  end process ctrl_engine_comb;

  -- bus arbiter operation in progress --
  cmd_busy_o <= '0' when (state = S_IDLE) else '1';


end neorv32_cache_bus_rtl;
