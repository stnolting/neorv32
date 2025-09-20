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
    clk_i   : in  std_ulogic; -- global clock, rising edge
    rstn_i  : in  std_ulogic; -- global reset, low-active, async
    ctrl_i  : in  ctrl_bus_t; -- main control bus
    -- trace port --
    trace_o : out trace_port_t -- execution trace port
  );
end neorv32_cpu_trace;

architecture neorv32_cpu_trace_rtl of neorv32_cpu_trace is

  -- trace arbiter
  type arbiter_t is record
    state : std_ulogic; -- sampling phase
    trap  : std_ulogic; -- trap entry
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
      arbiter.trap  <= '0';
    elsif rising_edge(clk_i) then
      if (arbiter.state = '0') then
        arbiter.state <= ctrl_i.cnt_event(cnt_event_ir_c); -- start trace cycle when in EXECUTE state
      elsif (arbiter.done = '1') then -- commit trace data when back in DISPATCH stage
        arbiter.state <= '0';
      end if;
      arbiter.trap <= (arbiter.trap or ctrl_i.cpu_trap) and (not arbiter.valid); -- trap-entry detector
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
      trace_buf.valid <= arbiter.valid;
      if (arbiter.valid = '1') then
        trace_buf.order <= std_ulogic_vector(unsigned(trace_buf.order) + 1);
      end if;
      trace_buf.pc   <= ctrl_i.pc_cur;
      trace_buf.insn <= ctrl_i.ir_funct12 & ctrl_i.rf_rs1 & ctrl_i.ir_funct3 & ctrl_i.rf_rd & ctrl_i.ir_opcode;
      trace_buf.intr <= arbiter.trap;
      if (ctrl_i.cnt_event(cnt_event_ir_c) = '1') then
        trace_buf.mode  <= ctrl_i.cpu_priv;
        trace_buf.debug <= ctrl_i.cpu_debug;
        trace_buf.compr <= ctrl_i.cnt_event(cnt_event_compr_c);
      end if;
    end if;
  end process trace_buffer;

  -- trace output --
  trace_o <= trace_buf;

end neorv32_cpu_trace_rtl;
