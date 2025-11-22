-- ================================================================================ --
-- NEORV32 CPU - Trace Generator                                                    --
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

entity neorv32_cpu_trace is
  port (
    -- global control --
    clk_i       : in  std_ulogic; -- global clock, rising edge
    rstn_i      : in  std_ulogic; -- global reset, low-active, async
    ctrl_i      : in  ctrl_bus_t; -- main control bus
    -- operands --
    rs1_rdata_i : std_ulogic_vector(XLEN-1 downto 0);     -- rs1 read data
    rs2_rdata_i : std_ulogic_vector(XLEN-1 downto 0);     -- rs2 read data
    rd_wdata_i  : std_ulogic_vector(XLEN-1 downto 0);     -- rd write data
    mem_ben_i   : std_ulogic_vector((XLEN/8)-1 downto 0); -- memory byte-enable
    mem_addr_i  : std_ulogic_vector(XLEN-1 downto 0);     -- memory address
    mem_wdata_i : std_ulogic_vector(XLEN-1 downto 0);     -- memory write data
    -- trace port --
    trace_o     : out trace_port_t -- execution trace port
  );
end neorv32_cpu_trace;

architecture neorv32_cpu_trace_rtl of neorv32_cpu_trace is

  -- trace arbiter
  type arbiter_t is record
    state : std_ulogic; -- sampling phase
    order : std_ulogic_vector(31 downto 0); -- instruction counter
    entry : std_ulogic; -- trap entry
    done  : std_ulogic; -- execution of current instruction completed
    valid : std_ulogic; -- commit trace data
  end record;
  signal arbiter : arbiter_t;

  -- trace buffer --
  signal trace_buf : trace_port_t;

begin

  -- Trace Arbiter --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trace_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      arbiter.state <= '0';
      arbiter.order <= (others => '0');
      arbiter.entry <= '0';
    elsif rising_edge(clk_i) then
      -- sampling control --
      if (arbiter.state = '0') then
        arbiter.state <= ctrl_i.cnt_event(cnt_event_ir_c); -- start trace cycle when in EXECUTE state
      elsif (arbiter.done = '1') then -- commit trace data when back in DISPATCH stage
        arbiter.state <= '0';
      end if;
      -- instruction counter --
      if (arbiter.valid = '1') then
        arbiter.order <= std_ulogic_vector(unsigned(arbiter.order) + 1);
      end if;
      -- trap-entry detector --
      if (arbiter.valid = '1') then
        arbiter.entry <= '0';
      else
        arbiter.entry <= arbiter.entry or ctrl_i.cpu_trap;
      end if;
    end if;
  end process trace_arbiter;

  -- commit trace state when back in DISPATCH stage (or when entering sleep mode) --
  arbiter.done  <= '1' when (ctrl_i.if_ready = '1') or (ctrl_i.cnt_event(cnt_event_cy_c) = '0') else '0';
  arbiter.valid <= arbiter.state and arbiter.done;


  -- Trace Buffer ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trace_buffer: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      trace_buf <= trace_port_terminate_c;
    elsif rising_edge(clk_i) then

      -- trace data is valid --
      trace_buf.valid <= arbiter.valid;

      -- instruction metadata --
      trace_buf.order <= arbiter.order;
      trace_buf.insn  <= ctrl_i.ir_funct12 & ctrl_i.rf_rs1 & ctrl_i.ir_funct3 & ctrl_i.rf_rd & ctrl_i.ir_opcode;
      trace_buf.trap  <= ctrl_i.cpu_sync_exc;
      trace_buf.halt  <= not ctrl_i.cnt_event(cnt_event_cy_c);
      trace_buf.intr  <= arbiter.entry;
      trace_buf.ixl   <= "01"; -- XLEN = 32-bit
      if (ctrl_i.cnt_event(cnt_event_ir_c) = '1') then
        trace_buf.mode  <= ctrl_i.cpu_priv & ctrl_i.cpu_priv;
        trace_buf.debug <= ctrl_i.cpu_debug;
        trace_buf.compr <= ctrl_i.cnt_event(cnt_event_compr_c);
      end if;

      -- integer register --
      trace_buf.rs1_addr  <= ctrl_i.rf_rs1;
      trace_buf.rs2_addr  <= ctrl_i.rf_rs2;
      trace_buf.rs1_rdata <= rs1_rdata_i;
      trace_buf.rs2_rdata <= rs2_rdata_i;
      if (ctrl_i.rf_wb_en = '1') then
        trace_buf.rd_addr <= ctrl_i.rf_rd;
      elsif (trace_buf.valid = '1') then
        trace_buf.rd_addr <= (others => '0');
      end if;
      trace_buf.rd_rdata <= rd_wdata_i;

      -- program counter --
      trace_buf.pc_rdata <= ctrl_i.pc_cur;
      trace_buf.pc_wdata <= ctrl_i.pc_nxt;

      -- control and status register --
      if (ctrl_i.csr_we = '1') or (ctrl_i.csr_re = '1') then
        trace_buf.csr_addr <= ctrl_i.csr_addr;
      elsif (trace_buf.valid = '1') then
        trace_buf.csr_addr <= (others => '0');
      end if;
      trace_buf.csr_rdata <= rd_wdata_i;
      trace_buf.csr_wdata <= ctrl_i.csr_wdata;

      -- memory access --
      if (ctrl_i.lsu_req = '1') then
        if (ctrl_i.lsu_rw = '0') or (ctrl_i.lsu_rmw = '1') then
          trace_buf.mem_rmask <= mem_ben_i;
        end if;
        if (ctrl_i.lsu_rw = '1') or (ctrl_i.lsu_rmw = '1') then
          trace_buf.mem_wmask <= mem_ben_i;
        end if;
      elsif (trace_buf.valid = '1') then
        trace_buf.mem_rmask <= (others => '0');
        trace_buf.mem_wmask <= (others => '0');
      end if;
      trace_buf.mem_addr  <= mem_addr_i;
      trace_buf.mem_rdata <= rd_wdata_i;
      trace_buf.mem_wdata <= mem_wdata_i;

    end if;
  end process trace_buffer;

  -- trace output --
  trace_o <= trace_buf;


end neorv32_cpu_trace_rtl;
