-- ================================================================================ --
-- NEORV32 SoC - External Bus Interface (XBUS)                                      --
-- -------------------------------------------------------------------------------- --
-- Converts internal bus transactions into Wishbone b4-compatible bus transactions. --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_xbus is
  generic (
    TIMEOUT_VAL : natural; -- cycles after an UNACKNOWLEDGED bus access triggers a bus fault exception
    REGSTAGE_EN : boolean  -- add XBUS register stage
  );
  port (
    clk_i      : in  std_ulogic; -- global clock line
    rstn_i     : in  std_ulogic; -- global reset line, low-active
    bus_req_i  : in  bus_req_t;  -- bus request
    bus_rsp_o  : out bus_rsp_t;  -- bus response
    --
    xbus_adr_o : out std_ulogic_vector(31 downto 0); -- address
    xbus_dat_i : in  std_ulogic_vector(31 downto 0); -- read data
    xbus_dat_o : out std_ulogic_vector(31 downto 0); -- write data
    xbus_tag_o : out std_ulogic_vector(2 downto 0); -- access tag
    xbus_we_o  : out std_ulogic; -- read/write
    xbus_sel_o : out std_ulogic_vector(3 downto 0); -- byte enable
    xbus_stb_o : out std_ulogic; -- strobe
    xbus_cyc_o : out std_ulogic; -- valid cycle
    xbus_ack_i : in  std_ulogic; -- transfer acknowledge
    xbus_err_i : in  std_ulogic  -- transfer error
  );
end neorv32_xbus;

architecture neorv32_xbus_rtl of neorv32_xbus is

  -- register stage --
  signal bus_req : bus_req_t;
  signal bus_rsp : bus_rsp_t;

  -- bus arbiter --
  signal pending     : std_ulogic;
  signal bus_rw      : std_ulogic;
  signal timeout     : std_ulogic;
  signal timeout_cnt : std_ulogic_vector(index_size_f(TIMEOUT_VAL) downto 0);

begin

  -- Configuration Info ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (TIMEOUT_VAL = 0) report
    "[NEORV32] External Bus Interface (XBUS): NO auto-timeout defined - can cause permanent CPU stall!" severity warning;


  -- Register Stage -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  reg_stage_enable:
  if REGSTAGE_EN generate
    reg_stage: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        bus_req   <= req_terminate_c;
        bus_rsp_o <= rsp_terminate_c;
      elsif rising_edge(clk_i) then
        -- request --
        if (bus_req_i.stb = '1') then -- keep all signals stable ...
          bus_req <= bus_req_i;
        end if;
        bus_req.stb <= bus_req_i.stb; -- ... except for STB that is single-shot
        -- response --
        bus_rsp_o <= bus_rsp;
      end if;
    end process reg_stage;
  end generate;

  reg_state_disable:
  if not REGSTAGE_EN generate
    bus_req   <= bus_req_i;
    bus_rsp_o <= bus_rsp;
  end generate;


  -- Bus Arbiter ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      timeout_cnt <= (others => '0');
      pending     <= '0';
      bus_rw      <= '0';
    elsif rising_edge(clk_i) then
      if (pending = '0') then -- idle, waiting for request
        timeout_cnt <= std_ulogic_vector(to_unsigned(TIMEOUT_VAL, timeout_cnt'length));
        pending     <= bus_req.stb;
      else -- busy, transfer in progress
        timeout_cnt <= std_ulogic_vector(unsigned(timeout_cnt) - 1);
        if (xbus_ack_i = '1') or (xbus_err_i = '1') or (timeout = '1') then
          pending <= '0';
        end if;
      end if;
      bus_rw <= bus_req.rw;
    end if;
  end process arbiter;

  -- bus timeout --
  timeout <= '1' when (TIMEOUT_VAL /= 0) and (or_reduce_f(timeout_cnt) = '0') else '0';


  -- Bus Access -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  xbus_adr_o <= bus_req.addr;
  xbus_dat_o <= bus_req.data;
  xbus_we_o  <= bus_req.rw;
  xbus_sel_o <= bus_req.ben;
  xbus_stb_o <= bus_req.stb;
  xbus_cyc_o <= bus_req.stb or pending;
  xbus_tag_o <= bus_req.src & '0' & bus_req.priv; -- instr/data, secure, privileged/unprivileged

  -- response gating --
  bus_rsp.data <= xbus_dat_i when (pending = '1') and (bus_rw = '0') else (others => '0'); -- no read-back if WRITE operation
  bus_rsp.ack  <= xbus_ack_i when (pending = '1') else '0';
  bus_rsp.err  <= (xbus_err_i or timeout) when (pending = '1') else '0';


end neorv32_xbus_rtl;
