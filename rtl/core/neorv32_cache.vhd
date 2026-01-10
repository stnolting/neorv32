-- ================================================================================ --
-- NEORV32 SoC - Generic Cache                                                      --
-- -------------------------------------------------------------------------------- --
-- Configurable generic cache module. The cache is direct-mapped and implements     --
-- "write-through" write strategy. Locked bursts operations are used if BURSTS_EN   --
-- is true; otherwise block transfers are split into single-transfers.              --
--                                                                                  --
-- Uncached / direct accesses: Several bus transaction types will bypass the cache: --
-- * atomic memory operations                                                       --
-- * accesses to the explicit "uncached address space page" (or higher),            --
--   which is defined by the 4 most significant address bits (UC_BEGIN)             --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  --
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
    NUM_BLOCKS : natural range 1 to 1024;       -- number of cache blocks, has to be a power of 2
    BLOCK_SIZE : natural range 4 to 32768;      -- cache block size in bytes, has to be a power of 2
    UC_BEGIN   : std_ulogic_vector(3 downto 0); -- begin of uncached address space (4 MSBs of address)
    READ_ONLY  : boolean;                       -- read-only accesses for host
    BURSTS_EN  : boolean                        -- enable issuing of burst transfers
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

  -- only emit bursts if enabled and if block size is at least 8 bytes --
  constant bursts_en_c : boolean := BURSTS_EN and boolean(BLOCK_SIZE >= 8);

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
      new_i   : in  std_ulogic;
      hit_o   : out std_ulogic;
      addr_i  : in  std_ulogic_vector(31 downto 0);
      we_i    : in  std_ulogic_vector(3 downto 0);
      wdata_i : in  std_ulogic_vector(31 downto 0);
      wstat_i : in  std_ulogic;
      rdata_o : out std_ulogic_vector(31 downto 0);
      rstat_o : out std_ulogic
    );
  end component;

  -- control -> cache interface --
  type cache_o_t is record
    cmd_clr : std_ulogic;
    cmd_new : std_ulogic;
    addr    : std_ulogic_vector(31 downto 0);
    data    : std_ulogic_vector(31 downto 0);
    stat    : std_ulogic;
    we      : std_ulogic_vector(3 downto 0);
  end record;
  signal cache_o : cache_o_t;

  -- cache -> control interface --
  type cache_i_t is record
    hit  : std_ulogic;
    data : std_ulogic_vector(31 downto 0);
    stat : std_ulogic;
  end record;
  signal cache_i : cache_i_t;

  -- cache bypass response buffer --
  signal bp_rsp : bus_rsp_t;

  -- control arbiter --
  type state_t is (
    S_IDLE, S_CHECK, S_DIRECT_REQ, S_DIRECT_RSP, S_CLEAR, S_DOWNLOAD_START, S_DOWNLOAD_WAIT, S_DOWNLOAD_RUN, S_DONE
  );
  type ctrl_t is record
    state   : state_t; -- state machine
    buf_req : std_ulogic; -- access request buffer
    buf_syn : std_ulogic; -- synchronization request buffer
    buf_dir : std_ulogic; -- direct/uncached access buffer
    tag_idx : std_ulogic_vector((tag_size_c + index_size_c)-1 downto 0); -- tag & index
    ofs_int : std_ulogic_vector(offset_size_c-1 downto 0); -- cache address offset
    ofs_ext : std_ulogic_vector(offset_size_c downto 0); -- bus address offset
  end record;
  signal ctrl, ctrl_nxt : ctrl_t;

begin

  -- Control Engine FSM Sync ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_engine_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl.state   <= S_IDLE;
      ctrl.buf_req <= '0';
      ctrl.buf_syn <= '0';
      ctrl.buf_dir <= '0';
      ctrl.tag_idx <= (others => '0');
      ctrl.ofs_int <= (others => '0');
      ctrl.ofs_ext <= (others => '0');
    elsif rising_edge(clk_i) then
      ctrl <= ctrl_nxt;
    end if;
  end process ctrl_engine_sync;


  -- Control Engine FSM Comb ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_engine_comb: process(ctrl, host_req_i, cache_i, bus_rsp_i, bp_rsp)
  begin
    -- control engine defaults --
    ctrl_nxt.state   <= ctrl.state;
    ctrl_nxt.buf_req <= ctrl.buf_req or host_req_i.stb;
    ctrl_nxt.buf_syn <= ctrl.buf_syn or host_req_i.fence;
    ctrl_nxt.buf_dir <= '0';
    ctrl_nxt.tag_idx <= ctrl.tag_idx;
    ctrl_nxt.ofs_int <= ctrl.ofs_int;
    ctrl_nxt.ofs_ext <= ctrl.ofs_ext;

    -- cache access defaults --
    cache_o.cmd_clr <= '0';
    cache_o.cmd_new <= '0';
    cache_o.addr    <= host_req_i.addr;
    cache_o.we      <= (others => '0');
    cache_o.data    <= host_req_i.data;
    cache_o.stat    <= '0';

    -- host response defaults --
    host_rsp_o      <= rsp_terminate_c; -- default: all off
    host_rsp_o.data <= cache_i.data; -- cache read data (for cache hit)

    -- bus interface defaults --
    bus_req_o      <= req_terminate_c; -- default: all off
    bus_req_o.meta <= host_req_i.meta;
    bus_req_o.addr <= ctrl.tag_idx & ctrl.ofs_ext(offset_size_c-1 downto 0) & "00"; -- cache access
    bus_req_o.ben  <= (others => '1'); -- cache updates use full-word accesses only

    -- fsm --
    case ctrl.state is

      when S_IDLE => -- wait for request
      -- ------------------------------------------------------------
        if (ctrl.buf_syn = '1') then -- pending sync request
          ctrl_nxt.state <= S_CLEAR;
        elsif (host_req_i.stb = '1') or (ctrl.buf_req = '1') then -- (pending) access request
          if (unsigned(host_req_i.addr(31 downto 28)) >= unsigned(UC_BEGIN)) or (host_req_i.amo = '1') then
            ctrl_nxt.buf_dir <= '1'; -- uncached address space access / atomic operation
          end if;
          ctrl_nxt.state <= S_CHECK;
        end if;

      when S_CHECK => -- check access request
      -- ------------------------------------------------------------
        ctrl_nxt.tag_idx <= host_req_i.addr(31 downto 32-(tag_size_c + index_size_c));
        ctrl_nxt.ofs_ext <= (others => '0');
        ctrl_nxt.ofs_int <= (others => '0');
        ctrl_nxt.buf_req <= '0'; -- access about to be completed
        --
        if (ctrl.buf_dir = '1') then -- direct/uncached access; no cache update
          ctrl_nxt.state <= S_DIRECT_REQ;
        elsif (cache_i.hit = '1') then -- cache HIT
          if (host_req_i.rw = '0') or READ_ONLY then -- read from cache
            host_rsp_o.ack <= '1';
            host_rsp_o.err <= cache_i.stat;
            ctrl_nxt.state <= S_IDLE;
          else -- write to main memory and also to the cache
            cache_o.we     <= host_req_i.ben;
            ctrl_nxt.state <= S_DIRECT_REQ; -- write-through
          end if;
        else -- cache MISS
          if (host_req_i.rw = '0') or READ_ONLY then -- read miss
            ctrl_nxt.state <= S_DOWNLOAD_START; -- get block from main memory
          else -- write miss
            ctrl_nxt.state <= S_DIRECT_REQ; -- write-through
          end if;
        end if;

      when S_DIRECT_REQ => -- direct memory access request
      -- ------------------------------------------------------------
        bus_req_o      <= host_req_i;
        bus_req_o.stb  <= '1';
        ctrl_nxt.state <= S_DIRECT_RSP;

      when S_DIRECT_RSP => -- direct memory access response
      -- ------------------------------------------------------------
        bus_req_o     <= host_req_i;
        bus_req_o.stb <= '0'; -- to prevent pass-through of STB signal
        host_rsp_o    <= bp_rsp; -- registered response
        if (bp_rsp.ack = '1') then
          ctrl_nxt.state <= S_IDLE;
        end if;

      when S_CLEAR => -- invalidate entire cache
      -- ------------------------------------------------------------
        bus_req_o.fence  <= bool_to_ulogic_f(not READ_ONLY);
        cache_o.cmd_clr  <= '1';
        ctrl_nxt.buf_syn <= '0';
        ctrl_nxt.state   <= S_IDLE;

      when S_DOWNLOAD_START => -- start block download / send single request (if no bursts)
      -- ------------------------------------------------------------
        cache_o.addr    <= ctrl.tag_idx & ctrl.ofs_int & "00";
        cache_o.cmd_new <= '1'; -- set new block (set tag and make valid)
        bus_req_o.addr  <= ctrl.tag_idx & ctrl.ofs_ext(offset_size_c-1 downto 0) & "00";
        bus_req_o.rw    <= '0'; -- read access
        bus_req_o.stb   <= '1'; -- send initial (burst/locking) request
        bus_req_o.lock  <= '1'; -- this is a locked transfer
        bus_req_o.burst <= bool_to_ulogic_f(bursts_en_c); -- this is a burst transfer
        bus_req_o.ben   <= (others => '1'); -- full-word access
        ctrl_nxt.state  <= S_DOWNLOAD_WAIT;

      when S_DOWNLOAD_WAIT => -- wait for exclusive (=locked) bus access / response
      -- ------------------------------------------------------------
        cache_o.addr    <= ctrl.tag_idx & ctrl.ofs_int & "00";
        cache_o.cmd_new <= '1'; -- set new block (set tag and make valid)
        cache_o.data    <= bus_rsp_i.data;
        cache_o.stat    <= bus_rsp_i.err;
        cache_o.we      <= (others => '1'); -- just keep writing full words
        bus_req_o.addr  <= ctrl.tag_idx & ctrl.ofs_ext(offset_size_c-1 downto 0) & "00";
        bus_req_o.rw    <= '0'; -- read access
        bus_req_o.lock  <= '1'; -- this is a locked transfer
        bus_req_o.burst <= bool_to_ulogic_f(bursts_en_c); -- this is a burst transfer
        bus_req_o.ben   <= (others => '1'); -- full-word access
        -- wait for initial ACK to start actual bursting --
        if (bus_rsp_i.ack = '1') then
          ctrl_nxt.ofs_int <= std_ulogic_vector(unsigned(ctrl.ofs_int) + 1);
          ctrl_nxt.ofs_ext <= std_ulogic_vector(unsigned(ctrl.ofs_ext) + 1);
          if bursts_en_c then
            ctrl_nxt.state <= S_DOWNLOAD_RUN;
          elsif (and_reduce_f(ctrl.ofs_int) = '1') then -- block completed
            ctrl_nxt.state <= S_DONE;
          else
            ctrl_nxt.state <= S_DOWNLOAD_START;
          end if;
        end if;

      when S_DOWNLOAD_RUN => -- bursts enabled: send read requests and get data responses
      -- ------------------------------------------------------------
        if bursts_en_c then
          cache_o.addr    <= ctrl.tag_idx & ctrl.ofs_int & "00";
          cache_o.cmd_new <= '1'; -- set new block (set tag and make valid)
          cache_o.data    <= bus_rsp_i.data;
          cache_o.stat    <= bus_rsp_i.err;
          cache_o.we      <= (others => '1'); -- just keep writing full words
          bus_req_o.addr  <= ctrl.tag_idx & ctrl.ofs_ext(offset_size_c-1 downto 0) & "00";
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
            ctrl_nxt.ofs_int <= std_ulogic_vector(unsigned(ctrl.ofs_int) + 1); -- next main memory location
            if (and_reduce_f(ctrl.ofs_int) = '1') then -- block completed
              ctrl_nxt.state <= S_DONE;
            end if;
          end if;
        else -- single transfers only
          ctrl_nxt.state <= S_IDLE;
        end if;

      when S_DONE => -- delay cycle for host cache access
      -- ------------------------------------------------------------
        ctrl_nxt.state <= S_CHECK;

      when others => -- undefined
      -- ------------------------------------------------------------
        ctrl_nxt.state <= S_IDLE;

    end case;
  end process ctrl_engine_comb;


  -- Cache Bypass (Direct Access) Response Buffer -------------------------------------------
  -- -------------------------------------------------------------------------------------------
  response_buf: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bp_rsp <= rsp_terminate_c;
    elsif rising_edge(clk_i) then
      bp_rsp.ack <= bus_rsp_i.ack;
      bp_rsp.err <= bus_rsp_i.err;
      if (bus_rsp_i.ack = '1') then -- reduce switching activity
        bp_rsp.data <= bus_rsp_i.data;
      end if;
    end if;
  end process response_buf;


  -- Cache Memory Core ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cache_memory_inst: neorv32_cache_memory
  generic map (
    NUM_BLOCKS => block_num_c, -- number of blocks, has to be a power of 2
    BLOCK_SIZE => block_size_c -- block size in bytes, has to be a power of 2
  )
  port map (
    -- global control --
    rstn_i  => rstn_i,          -- global reset, async, low-active
    clk_i   => clk_i,           -- global clock, rising edge
    -- management --
    clr_i   => cache_o.cmd_clr, -- clear entire cache
    new_i   => cache_o.cmd_new, -- make accessed block valid and set tag
    hit_o   => cache_i.hit,     -- cache hit
    -- cache access --
    addr_i  => cache_o.addr,    -- access address
    we_i    => cache_o.we,      -- byte-wide data write enable
    wdata_i => cache_o.data,    -- write data
    wstat_i => cache_o.stat,    -- write status
    rdata_o => cache_i.data,    -- read data
    rstat_o => cache_i.stat     -- read status
  );

end neorv32_cache_rtl;


-- ================================================================================ --
-- NEORV32 SoC - Generic Cache - Data and Status Memory (direct-mapped)             --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2026 Stephan Nolting. All rights reserved.                         --
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
    NUM_BLOCKS : natural; -- number of blocks, has to be a power of 2, min 1
    BLOCK_SIZE : natural  -- block size in bytes, has to be a power of 2, min 4
  );
  port (
    -- global control --
    rstn_i  : in  std_ulogic;                     -- global reset, async, low-active
    clk_i   : in  std_ulogic;                     -- global clock, rising edge
    -- management --
    clr_i   : in  std_ulogic;                     -- clear entire cache
    new_i   : in  std_ulogic;                     -- make accessed block valid & clean and set tag
    hit_o   : out std_ulogic;                     -- cache hit
    -- cache access --
    addr_i  : in  std_ulogic_vector(31 downto 0); -- access address
    we_i    : in  std_ulogic_vector(3 downto 0);  -- byte-wide data write enable
    wdata_i : in  std_ulogic_vector(31 downto 0); -- write data
    wstat_i : in  std_ulogic;                     -- write status
    rdata_o : out std_ulogic_vector(31 downto 0); -- read data
    rstat_o : out std_ulogic                      -- read status
  );
end neorv32_cache_memory;

architecture neorv32_cache_memory_rtl of neorv32_cache_memory is

  -- cache layout --
  constant offset_size_c : natural := index_size_f(BLOCK_SIZE/4); -- offset addresses full 32-bit words
  constant index_size_c  : natural := index_size_f(NUM_BLOCKS); -- index size
  constant adr_size_c    : natural := offset_size_c + index_size_c; -- RAM address size
  constant tag_size_c    : natural := 32 - (offset_size_c + index_size_c + 2); -- +2 bits for byte offset

  -- status flags --
  signal valid : std_ulogic_vector(NUM_BLOCKS-1 downto 0);
  signal valid_rd : std_ulogic;

  -- cache access --
  signal tag, tag_reg, tag_rd: std_ulogic_vector(tag_size_c-1 downto 0);
  signal addr : std_ulogic_vector(adr_size_c-1 downto 0);
  signal wdata, rdata : std_ulogic_vector(32 downto 0);

begin

  -- Access Address Decomposition -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tag  <= addr_i(31 downto 31-(tag_size_c-1)); -- tag
  addr <= addr_i(adr_size_c+1 downto 2); -- RAM address

  -- tag pipeline register --
  tag_buffer: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      tag_reg <= (others => '0');
    elsif rising_edge(clk_i) then
      tag_reg <= tag;
    end if;
  end process tag_buffer;


  -- Status Memory --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  status_mem_large:
  if (NUM_BLOCKS > 1) generate
    status_memory: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        valid    <= (others => '0');
        valid_rd <= '0';
      elsif rising_edge(clk_i) then
        if (clr_i = '1') then -- invalidate entire cache
          valid <= (others => '0');
        elsif (new_i = '1') then -- make indexed block valid
          valid(to_integer(unsigned(addr_i(31-tag_size_c downto 2+offset_size_c)))) <= '1';
        end if;
        valid_rd <= valid(to_integer(unsigned(addr_i(31-tag_size_c downto 2+offset_size_c)))); -- sync read
      end if;
    end process status_memory;
  end generate;

  -- single-entry only --
  status_mem_small:
  if (NUM_BLOCKS = 1) generate
    status_memory: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        valid(0) <= '0';
      elsif rising_edge(clk_i) then
        if (clr_i = '1') then -- invalidate
          valid(0) <= '0';
        elsif (new_i = '1') then -- make valid
          valid(0) <= '1';
        end if;
      end if;
    end process status_memory;
    valid_rd <= valid(0);
  end generate;


  -- Tag Memory -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tag_memory: entity neorv32.neorv32_prim_spram
  generic map (
    AWIDTH => index_size_c,
    DWIDTH => tag_size_c,
    OUTREG => false
  )
  port map (
    clk_i  => clk_i,
    en_i   => '1',
    rw_i   => new_i,
    addr_i => addr_i(31-tag_size_c downto 2+offset_size_c), -- index
    data_i => tag,
    data_o => tag_rd
  );

  -- access status (1 cycle latency due to sync memory read) --
  hit_o <= '1' when (valid_rd = '1') and (tag_rd = tag_reg) else '0'; -- cache hit


  -- Data Memory ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- data memories for bytes 0 to 2 --
  data_mem_gen:
  for i in 0 to 2 generate
    data_mem: entity neorv32.neorv32_prim_spram
    generic map (
      AWIDTH => adr_size_c,
      DWIDTH => 8,
      OUTREG => false
    )
    port map (
      clk_i  => clk_i,
      en_i   => '1',
      rw_i   => we_i(i),
      addr_i => addr,
      data_i => wdata(i*8+7 downto i*8),
      data_o => rdata(i*8+7 downto i*8)
    );
  end generate;

  -- data memory for byte 3 plus status flag --
  data_stat_mem: entity neorv32.neorv32_prim_spram
  generic map (
    AWIDTH => adr_size_c,
    DWIDTH => 9,
    OUTREG => false
  )
  port map (
    clk_i  => clk_i,
    en_i   => '1',
    rw_i   => we_i(3),
    addr_i => addr,
    data_i => wdata(32 downto 24),
    data_o => rdata(32 downto 24)
  );

  -- memory data --
  wdata   <= wstat_i & wdata_i;
  rdata_o <= rdata(31 downto 0);
  rstat_o <= rdata(32);

end neorv32_cache_memory_rtl;
