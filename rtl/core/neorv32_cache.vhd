-- ================================================================================ --
-- NEORV32 SoC - Generic Cache                                                      --
-- -------------------------------------------------------------------------------- --
-- Configurable generic cache module. The cache is direct-mapped and implements     --
-- "write-through" write strategy. Locked burst operations are used if BURSTS_EN is --
-- enabled; block transfers are split into single-transfers otherwise.              --
-- Total cache size in bytes is NUM_BLOCKS x BLOCK_SIZE.                            --
--                                                                                  --
-- Bus transaction types that will *bypass* the cache:                              --
-- * any atomic memory operation                                                    --
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

  -- cache data & tag RAM wrapper --
  -- [NOTE] We use component instantiation here to allow easy black-box instantiation for
  -- late component binding (e.g. when using the VHDL-to-Verilog flow with Verilog memory IP).
  component neorv32_cache_ram
  generic (
    TAG_WIDTH : natural;
    IDX_WIDTH : natural;
    OFS_WIDTH : natural
  );
  port (
    clk_i     : in  std_ulogic;
    addr_i    : in  std_ulogic_vector(31 downto 0);
    sta_we_i  : in  std_ulogic;
    vld_i     : in  std_ulogic;
    vld_o     : out std_ulogic;
    tag_o     : out std_ulogic_vector(31 downto 0);
    data_we_i : in  std_ulogic_vector(3 downto 0);
    data_i    : in  std_ulogic_vector(31 downto 0);
    data_o    : out std_ulogic_vector(31 downto 0)
  );
  end component;

  -- only emit bursts if enabled and if block size is at least 8 bytes --
  constant bursts_en_c : boolean := BURSTS_EN and boolean(BLOCK_SIZE >= 8);

  -- cache layout --
  constant block_num_c    : natural := 2**index_size_f(NUM_BLOCKS); -- extend if not a power of two
  constant block_size_c   : natural := 2**index_size_f(BLOCK_SIZE); -- extend if not a power of two
  constant offset_width_c : natural := index_size_f(block_size_c/4); -- word offset
  constant index_width_c  : natural := index_size_f(block_num_c);
  constant tag_width_c    : natural := 32 - (offset_width_c + index_width_c + 2);

  -- control -> cache interface --
  type cache_o_t is record
    set  : std_ulogic;
    vld  : std_ulogic;
    addr : std_ulogic_vector(31 downto 0);
    data : std_ulogic_vector(31 downto 0);
    we   : std_ulogic_vector(3 downto 0);
  end record;
  signal cache_o : cache_o_t;

  -- cache -> control interface --
  type cache_i_t is record
    hit  : std_ulogic;
    tag  : std_ulogic_vector(31 downto 0);
    data : std_ulogic_vector(31 downto 0);
  end record;
  signal cache_i : cache_i_t;

  -- control arbiter --
  type state_t is (
    S_IDLE, S_CHECK, S_BYPASS, S_CLEAR_START, S_CLEAR_RUN,
    S_UPDATE_START, S_UPDATE_WAIT, S_UPDATE_RUN, S_UPDATE_DONE
  );
  type ctrl_t is record
    state   : state_t; -- state machine
    bus_err : std_ulogic; -- access error
    buf_req : std_ulogic; -- access request buffer
    buf_syn : std_ulogic; -- synchronization request buffer
    buf_bp  : std_ulogic; -- cache bypass access buffer
    bp_req  : std_ulogic; -- cache bypass bus request (STB)
    hit     : std_ulogic; -- forced cache hit
    tag     : std_ulogic_vector(tag_width_c-1 downto 0); -- tag
    idx     : std_ulogic_vector(index_width_c-1 downto 0); -- index
    ofs_int : std_ulogic_vector(offset_width_c-1 downto 0); -- cache address offset
    ofs_ext : std_ulogic_vector(offset_width_c downto 0); -- bus address offset
  end record;
  signal ctrl, ctrl_nxt : ctrl_t;

  -- status check --
  signal valid_rd : std_ulogic;
  signal tag_reg  : std_ulogic_vector(tag_width_c-1 downto 0);

begin

  -- Control Engine FSM Sync ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_engine_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl.state   <= S_IDLE;
      ctrl.bus_err <= '0';
      ctrl.buf_req <= '0';
      ctrl.buf_syn <= '1'; -- start with cache clearing
      ctrl.buf_bp  <= '0';
      ctrl.bp_req  <= '0';
      ctrl.hit     <= '0';
      ctrl.tag     <= (others => '0');
      ctrl.idx     <= (others => '0');
      ctrl.ofs_int <= (others => '0');
      ctrl.ofs_ext <= (others => '0');
    elsif rising_edge(clk_i) then
      ctrl <= ctrl_nxt;
    end if;
  end process ctrl_engine_sync;


  -- Control Engine FSM Comb ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_engine_comb: process(ctrl, host_req_i, cache_i, bus_rsp_i)
  begin
    -- control engine defaults --
    ctrl_nxt.state   <= ctrl.state;
    ctrl_nxt.bus_err <= ctrl.bus_err;
    ctrl_nxt.buf_req <= ctrl.buf_req or host_req_i.stb;
    ctrl_nxt.buf_syn <= ctrl.buf_syn or host_req_i.fence;
    ctrl_nxt.buf_bp  <= '0';
    ctrl_nxt.bp_req  <= '0';
    ctrl_nxt.hit     <= '0';
    ctrl_nxt.tag     <= ctrl.tag;
    ctrl_nxt.idx     <= ctrl.idx;
    ctrl_nxt.ofs_int <= ctrl.ofs_int;
    ctrl_nxt.ofs_ext <= ctrl.ofs_ext;

    -- cache access defaults --
    cache_o.set  <= '0';
    cache_o.vld  <= '0';
    cache_o.addr <= host_req_i.addr;
    cache_o.we   <= (others => '0');
    cache_o.data <= host_req_i.data;

    -- host response defaults --
    host_rsp_o      <= rsp_terminate_c; -- default: all off
    host_rsp_o.data <= cache_i.data; -- cache read data (for cache hit)

    -- bus interface defaults --
    bus_req_o      <= req_terminate_c; -- default: all off
    bus_req_o.meta <= host_req_i.meta;
    bus_req_o.addr <= ctrl.tag & ctrl.idx & ctrl.ofs_ext(offset_width_c-1 downto 0) & "00"; -- cache access
    bus_req_o.ben  <= (others => '1'); -- cache updates use full-word accesses only

    -- fsm --
    case ctrl.state is

      when S_IDLE => -- wait for request
      -- ------------------------------------------------------------
        ctrl_nxt.bus_err <= '0'; -- reset bus error flag
        if (ctrl.buf_syn = '1') then -- pending sync request
          ctrl_nxt.state <= S_CLEAR_START;
        elsif (host_req_i.stb = '1') or (ctrl.buf_req = '1') then -- (pending) access request
          if (unsigned(host_req_i.addr(31 downto 28)) >= unsigned(UC_BEGIN)) or (host_req_i.amo = '1') then
            ctrl_nxt.buf_bp <= '1'; -- uncached address space access / atomic operation -> bypass cache
          end if;
          ctrl_nxt.state <= S_CHECK;
        end if;

      when S_CHECK => -- check access request
      -- ------------------------------------------------------------
        ctrl_nxt.tag     <= host_req_i.addr(31 downto 31-(tag_width_c-1));
        ctrl_nxt.idx     <= host_req_i.addr(31-tag_width_c downto 32-(tag_width_c + index_width_c));
        ctrl_nxt.ofs_ext <= (others => '0');
        ctrl_nxt.ofs_int <= (others => '0');
        ctrl_nxt.buf_req <= '0'; -- access about to be completed
        ctrl_nxt.bp_req  <= '1'; -- in case we are doing a bypass request (set STB for one cycle)
        --
        if (ctrl.buf_bp = '1') then -- cache bypass
          ctrl_nxt.state <= S_BYPASS;
        elsif (host_req_i.rw = '0') or READ_ONLY then -- read
          if (cache_i.hit = '1') then -- cache HIT: read from cache
            host_rsp_o.ack <= '1';
            ctrl_nxt.state <= S_IDLE;
          else -- cache MISS: get missing block from main memory
            ctrl_nxt.state <= S_UPDATE_START;
          end if;
        else -- write
          if (cache_i.hit = '1') then -- cache HIT: also write to cache
            cache_o.we <= host_req_i.ben;
          end if;
          ctrl_nxt.state <= S_BYPASS; -- write-through
        end if;

      when S_BYPASS => -- cache bypass
      -- ------------------------------------------------------------
        bus_req_o     <= host_req_i;
        bus_req_o.stb <= ctrl.bp_req; -- one-shot
        host_rsp_o    <= bus_rsp_i;
        if (bus_rsp_i.ack = '1') then
          ctrl_nxt.state <= S_IDLE;
        end if;

      when S_CLEAR_START => -- start cache clearing
      -- ------------------------------------------------------------
        bus_req_o.fence  <= '1';
        ctrl_nxt.buf_syn <= '0';
        ctrl_nxt.idx     <= (others => '0');
        ctrl_nxt.state   <= S_CLEAR_RUN;

      when S_CLEAR_RUN => -- clear block by block
      -- ------------------------------------------------------------
        cache_o.addr <= ctrl.tag & ctrl.idx & ctrl.ofs_int & "00";
        cache_o.vld  <= '0';
        cache_o.set  <= '1'; -- invalidate current block
        ctrl_nxt.idx <= std_ulogic_vector(unsigned(ctrl.idx) + 1);
        if (and_reduce_f(ctrl.idx) = '1') then -- all blocks done
          ctrl_nxt.state <= S_IDLE;
        end if;

      when S_UPDATE_START => -- start block download / send single request (if no bursts)
      -- ------------------------------------------------------------
        bus_req_o.rw    <= '0'; -- read access
        bus_req_o.stb   <= '1'; -- send request
        bus_req_o.lock  <= '1'; -- this is a locked transfer
        bus_req_o.burst <= bool_to_ulogic_f(bursts_en_c); -- this is a burst transfer
        ctrl_nxt.state  <= S_UPDATE_WAIT;

      when S_UPDATE_WAIT => -- wait for exclusive/locked bus access
      -- ------------------------------------------------------------
        cache_o.addr    <= ctrl.tag & ctrl.idx & ctrl.ofs_int & "00";
        cache_o.data    <= bus_rsp_i.data;
        cache_o.we      <= (others => '1'); -- write full words
        bus_req_o.rw    <= '0'; -- read access
        bus_req_o.lock  <= '1'; -- this is a locked transfer
        bus_req_o.burst <= bool_to_ulogic_f(bursts_en_c); -- this is a burst transfer
        -- wait for initial ACK to start actual bursting --
        if (bus_rsp_i.ack = '1') then
          ctrl_nxt.bus_err <= ctrl.bus_err or bus_rsp_i.err; -- accumulate bus errors
          ctrl_nxt.ofs_int <= std_ulogic_vector(unsigned(ctrl.ofs_int) + 1);
          ctrl_nxt.ofs_ext <= std_ulogic_vector(unsigned(ctrl.ofs_ext) + 1);
          if bursts_en_c then
            ctrl_nxt.state <= S_UPDATE_RUN;
          elsif (and_reduce_f(ctrl.ofs_int) = '1') then -- block completed
            ctrl_nxt.state <= S_UPDATE_DONE;
          else
            ctrl_nxt.state <= S_UPDATE_START;
          end if;
        end if;

      when S_UPDATE_RUN => -- bursts enabled: send read requests and get data responses
      -- ------------------------------------------------------------
        if bursts_en_c then
          cache_o.addr    <= ctrl.tag & ctrl.idx & ctrl.ofs_int & "00";
          cache_o.data    <= bus_rsp_i.data;
          cache_o.we      <= (others => '1'); -- write full words
          bus_req_o.rw    <= '0'; -- read access
          bus_req_o.lock  <= '1'; -- this is a locked transfer
          bus_req_o.burst <= '1'; -- this is a burst transfer
          -- send requests --
          if (ctrl.ofs_ext(offset_width_c) = '0') then
            ctrl_nxt.ofs_ext <= std_ulogic_vector(unsigned(ctrl.ofs_ext) + 1); -- next cache word
            bus_req_o.stb    <= '1'; -- request next transfer
          end if;
          -- receive responses --
          if (bus_rsp_i.ack = '1') then
            ctrl_nxt.bus_err <= ctrl.bus_err or bus_rsp_i.err; -- accumulate bus errors
            ctrl_nxt.ofs_int <= std_ulogic_vector(unsigned(ctrl.ofs_int) + 1); -- next main memory location
            if (and_reduce_f(ctrl.ofs_int) = '1') then -- block completed
              ctrl_nxt.state <= S_UPDATE_DONE;
            end if;
          end if;
        else -- single transfers only
          ctrl_nxt.state <= S_IDLE;
        end if;

      when S_UPDATE_DONE => -- any error during block update?
      -- ------------------------------------------------------------
        cache_o.vld  <= '1'; -- set valid flag; only relevant if no bus error
        ctrl_nxt.hit <= '1'; -- force cache hit to skip valid-read latency; only relevant if no bus error
        if (ctrl.bus_err = '0') then
          cache_o.set    <= '1'; -- set tag and make valid
          ctrl_nxt.state <= S_CHECK;
        else
          host_rsp_o.ack <= '1';
          host_rsp_o.err <= '1';
          ctrl_nxt.state <= S_IDLE;
        end if;

      when others => -- undefined
      -- ------------------------------------------------------------
        ctrl_nxt.state <= S_IDLE;

    end case;
  end process ctrl_engine_comb;


  -- Cache Data and Status Memory (Wrapper) -------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cache_ram_inst: neorv32_cache_ram
  generic map (
    TAG_WIDTH => tag_width_c,
    IDX_WIDTH => index_width_c,
    OFS_WIDTH => offset_width_c
  )
  port map (
    clk_i     => clk_i,
    addr_i    => cache_o.addr,
    sta_we_i  => cache_o.set,
    vld_i     => cache_o.vld,
    vld_o     => valid_rd,
    tag_o     => cache_i.tag,
    data_we_i => cache_o.we,
    data_i    => cache_o.data,
    data_o    => cache_i.data
  );

  -- tag buffer to compensate status memory read latency --
  tag_buffer: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      tag_reg <= (others => '0');
    elsif rising_edge(clk_i) then
      tag_reg <= cache_o.addr(31 downto 31-(tag_width_c-1));
    end if;
  end process tag_buffer;

  -- cache hit --
  cache_i.hit <= '1' when ((valid_rd = '1') and (cache_i.tag(tag_width_c-1 downto 0) = tag_reg)) or
                           (ctrl.hit = '1') else '0';

end neorv32_cache_rtl;
