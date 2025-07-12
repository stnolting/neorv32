-- ================================================================================ --
-- NEORV32 SoC - Execution Tracer                                                   --
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

entity neorv32_tracer is
  generic (
    TRACE_DEPTH  : natural range 1 to 2**15; -- trace buffer depth (has to be a power of two)
    DUAL_CORE_EN : boolean := false -- trace the dual-core configuration
  );
  port (
    clk_i     : in  std_ulogic;   -- global clock line
    rstn_i    : in  std_ulogic;   -- global reset line, low-active, async
    trace0_i  : in  trace_port_t; -- CPU 0 trace port
    trace1_i  : in  trace_port_t; -- CPU 1 trace port
    bus_req_i : in  bus_req_t;    -- bus request
    bus_rsp_o : out bus_rsp_t;    -- bus response
    irq_o     : out std_ulogic    -- tracing-done interrupt
  );
end neorv32_tracer;

architecture neorv32_tracer_rtl of neorv32_tracer is

  -- control register bits --
  constant ctrl_enable_c  : natural :=  0; -- r/w: module enable; reset module if 0
  constant ctrl_hsel_c    : natural :=  1; -- r/w: selected hart for tracing
  constant ctrl_start_c   : natural :=  2; -- r/w: start tracing; flag always reads as zero
  constant ctrl_stop_c    : natural :=  3; -- r/w: stop tracing; flag always reads as zero
  constant ctrl_run_c     : natural :=  4; -- r/-: tracing is running when set
  constant ctrl_avail_c   : natural :=  5; -- r/-: trace data available
  constant ctrl_irq_clr_c : natural :=  6; -- r/w: clear pending interrupt by writing one
  constant data_tbm_lsb_c : natural :=  7; -- r/-: log2(RX FIFO size) LSB
  constant data_tbm_msb_c : natural := 10; -- r/-: log2(RX FIFO size) MSB

  -- helpers --
  constant log2_tbm_c : natural := index_size_f(TRACE_DEPTH);

  -- trace buffer --
  component neorv32_tracer_buffer
  generic (
    TRACE_DEPTH : natural
  );
  port (
    clk_i   : in  std_ulogic;
    rstn_i  : in  std_ulogic;
    trace_i : in  trace_port_t;
    en_i    : in  std_ulogic;
    run_i   : in  std_ulogic;
    re_i    : in  std_ulogic;
    src_o   : out std_ulogic_vector(31 downto 0);
    dst_o   : out std_ulogic_vector(31 downto 0)
  );
  end component;

  -- control registers --
  signal ctrl_en, ctrl_hsel, ctrl_start, ctrl_stop, ctrl_iclr : std_ulogic;
  signal stop_addr : std_ulogic_vector(30 downto 0);

  -- trace arbiter --
  type state_t is (S_OFFLINE, S_GET_SRC, S_GET_DST);
  type arbiter_t is record
    state : state_t; -- FSM state
    astop : std_ulogic; -- auto-stop tracing at given address
    run   : std_ulogic; -- tracing in progress
    src   : std_ulogic_vector(31 downto 0); -- source address
    dst   : std_ulogic_vector(31 downto 0); -- destination address
    trap  : std_ulogic; -- trap entry
    first : std_ulogic; -- first trap packet
    push  : std_ulogic; -- push SRC + DST to trace buffer
  end record;
  signal arbiter : arbiter_t;

  -- trace buffer interface --
  type fifo_t is record
    we,    re    : std_ulogic; -- write/read enable
    wdata, rdata : std_ulogic_vector(63 downto 0); -- write/read data
    avail, free  : std_ulogic; -- FIFO status
    clear        : std_ulogic; -- sync clear
  end record;
  signal fifo : fifo_t;

  -- misc --
  signal over_check : std_ulogic; -- FIFO overflow checker
  signal over_trash : std_ulogic; -- discard data from trace buffer
  signal trace      : trace_port_t; -- trace input stream
  signal irq_pend   : std_ulogic; -- interrupt generator

begin

  -- Bus Access -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bus_rsp_o  <= rsp_terminate_c;
      ctrl_en    <= '0';
      ctrl_hsel  <= '0';
      ctrl_start <= '0';
      ctrl_stop  <= '0';
      ctrl_iclr  <= '0';
      stop_addr  <= (others => '0');
    elsif rising_edge(clk_i) then
      -- bus handshake --
      bus_rsp_o.ack <= bus_req_i.stb;
      bus_rsp_o.err <= '0';
      -- write access --
      ctrl_start <= '0';
      ctrl_stop  <= '0';
      ctrl_iclr  <= '0';
      if (bus_req_i.stb = '1') and (bus_req_i.rw = '1') then
        if (bus_req_i.addr(3 downto 2) = "00") then -- control register
          ctrl_en    <= bus_req_i.data(ctrl_enable_c);
          ctrl_hsel  <= bus_req_i.data(ctrl_hsel_c) and bool_to_ulogic_f(DUAL_CORE_EN);
          ctrl_start <= bus_req_i.data(ctrl_start_c);
          ctrl_stop  <= bus_req_i.data(ctrl_stop_c);
          ctrl_iclr  <= bus_req_i.data(ctrl_irq_clr_c);
        end if;
        if (bus_req_i.addr(3 downto 2) = "01") then -- stop-address register
          stop_addr <= bus_req_i.data(31 downto 1);
        end if;
      end if;
      -- read access --
      bus_rsp_o.data <= (others => '0');
      if (bus_req_i.stb = '1') and (bus_req_i.rw = '0') then
        case bus_req_i.addr(3 downto 2) is
          when "00" => -- control register
            bus_rsp_o.data(ctrl_enable_c) <= ctrl_en;
            bus_rsp_o.data(ctrl_hsel_c)   <= ctrl_hsel and bool_to_ulogic_f(DUAL_CORE_EN);
            bus_rsp_o.data(ctrl_run_c)    <= arbiter.run;
            bus_rsp_o.data(ctrl_avail_c)  <= fifo.avail;
            bus_rsp_o.data(data_tbm_msb_c downto data_tbm_lsb_c) <= std_ulogic_vector(to_unsigned(log2_tbm_c, 4));
          when "01" => -- stop-address register
            bus_rsp_o.data <= stop_addr & '0';
          when "10" => -- trace data: source
            bus_rsp_o.data <= fifo.rdata(31 downto 0);
          when others => -- trace data: destination
            bus_rsp_o.data <= fifo.rdata(63 downto 32);
        end case;
      end if;
    end if;
  end process bus_access;


  -- Trace Control Arbiter ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trace_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      arbiter.state <= S_OFFLINE;
      arbiter.astop <= '0';
      arbiter.src   <= (others => '0');
      arbiter.dst   <= (others => '0');
      arbiter.trap  <= '0';
      arbiter.first <= '0';
      arbiter.push  <= '0';
    elsif rising_edge(clk_i) then
      -- defaults --
      arbiter.push <= '0';

      -- stop tracing at address --
      if (ctrl_en = '0') or (arbiter.run = '0') then
        arbiter.astop <= '0';
      elsif (trace.valid = '1') and (trace.pc(31 downto 1) = stop_addr) then
        arbiter.astop <= '1';
      end if;

      -- fsm --
      case arbiter.state is

        when S_OFFLINE => -- tracing disabled
        -- ------------------------------------------------------------
          arbiter.trap  <= '0'; -- no trap yet
          arbiter.first <= '1'; -- this will be the first trace packet
          if (ctrl_en = '1') and (ctrl_start = '1') then
            arbiter.state <= S_GET_SRC;
          end if;

        when S_GET_SRC => -- get delta source address
        -- ------------------------------------------------------------
          if (ctrl_en = '0') or (ctrl_stop = '1') or (arbiter.astop = '1') then
            arbiter.state <= S_OFFLINE;
          elsif (trace.mode(1) = '0') then -- halt tracing when we are in debug-mode
            arbiter.trap <= arbiter.trap or trace.trap;
            if (trace.valid = '1') or (trace.trap = '1') then
              arbiter.src(31 downto 1) <= trace.pc(31 downto 1);
            end if;
            if (trace.delta = '1') then -- non-linear PC change
              arbiter.state <= S_GET_DST;
            end if;
          end if;

        when S_GET_DST => -- get delta destination address
        -- ------------------------------------------------------------
          arbiter.src(0) <= arbiter.trap;
          arbiter.dst    <= trace.pc(31 downto 1) & arbiter.first;
          if (ctrl_en = '0') or (ctrl_stop = '1') or (arbiter.astop = '1') then
            arbiter.state <= S_OFFLINE;
          elsif (trace.mode(1) = '1') then -- discard this packet if we have entered debug-mode
            arbiter.state <= S_GET_SRC;
          elsif (trace.valid = '1') then -- first instruction of branch destination
            arbiter.push  <= '1';
            arbiter.trap  <= '0';
            arbiter.first <= '0';
            arbiter.state <= S_GET_SRC;
          end if;

        when others => -- undefined
        -- ------------------------------------------------------------
          arbiter.state <= S_OFFLINE;

      end case;
    end if;
  end process trace_arbiter;

  -- tracing in process --
  arbiter.run <= '0' when (arbiter.state = S_OFFLINE) else '1';

  -- trace select --
  trace <= trace0_i when (ctrl_hsel = '0') or (DUAL_CORE_EN = false) else trace1_i;


  -- Interrupt Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_generator: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      irq_pend <= '0';
    elsif rising_edge(clk_i) then
      if (ctrl_en = '0') then
        irq_pend <= '0';
      elsif (arbiter.astop = '1') then
        irq_pend <= '1';
      elsif (ctrl_iclr = '1') then
        irq_pend <= '0';
      end if;
    end if;
  end process irq_generator;

  -- output to CPU --
  irq_o <= irq_pend;


  -- Trace Buffer (implemented as FIFO) -----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trace_buffer_inst: entity neorv32.neorv32_fifo
  generic map (
    FIFO_DEPTH => TRACE_DEPTH,
    FIFO_WIDTH => 2*32,
    FIFO_RSYNC => true,
    FIFO_SAFE  => true,
    FULL_RESET => false,
    OUT_GATE   => false
  )
  port map (
    -- control and status --
    clk_i   => clk_i,
    rstn_i  => rstn_i,
    clear_i => fifo.clear,
    half_o  => open,
    level_o => open,
    -- write port --
    wdata_i => fifo.wdata,
    we_i    => fifo.we,
    free_o  => fifo.free,
    -- read port --
    re_i    => fifo.re,
    rdata_o => fifo.rdata,
    avail_o => fifo.avail
  );

  -- FIFO access --
  fifo.clear <= not ctrl_en;
  fifo.we    <= arbiter.push;
  fifo.wdata <= arbiter.dst & arbiter.src;
  fifo.re    <= '1' when (over_trash = '1') or
                         ((bus_req_i.stb = '1') and (bus_req_i.rw = '0') and (bus_req_i.addr(3 downto 2) = "11")) else '0';

  -- discard oldest entry if overflowing --
  discard: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      over_check <= '0';
      over_trash <= '0';
    elsif rising_edge(clk_i) then
      if (over_check = '0') or (ctrl_en = '0') or (arbiter.run = '0') then
        over_check <= not fifo.free;
        over_trash <= '0';
      else
        over_check <= '0';
        over_trash <= '1';
      end if;
    end if;
  end process discard;

end neorv32_tracer_rtl;
