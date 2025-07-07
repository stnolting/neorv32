-- ================================================================================ --
-- NEORV32 SoC - External Bus Interface (XBUS)                                      --
-- -------------------------------------------------------------------------------- --
-- Converts processor-internal bus transactions into "registered feedback"          --
-- Wishbone-compatible bus accesses.                                                --
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
    xbus_adr_o : out std_ulogic_vector(31 downto 0); -- address
    xbus_dat_i : in  std_ulogic_vector(31 downto 0); -- read data
    xbus_dat_o : out std_ulogic_vector(31 downto 0); -- write data
    xbus_cti_o : out std_ulogic_vector(2 downto 0); -- cycle type
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
  signal pending : std_ulogic;
  signal locked  : std_ulogic;

  -- no-response timeout --
  constant log2_timeout_c : natural := index_size_f(TIMEOUT_VAL);
  constant timeout_c : unsigned(log2_timeout_c downto 0) := to_unsigned(TIMEOUT_VAL, log2_timeout_c+1);
  signal timecnt : unsigned(log2_timeout_c downto 0);
  signal timeout : std_ulogic;

begin

  -- Optional Register Stage(s) -------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  reg_stage_inst: entity neorv32.neorv32_bus_reg
  generic map (
    REQ_REG_EN => REGSTAGE_EN,
    RSP_REG_EN => REGSTAGE_EN
  )
  port map (
    clk_i        => clk_i,
    rstn_i       => rstn_i,
    host_req_i   => bus_req_i,
    host_rsp_o   => bus_rsp_o,
    device_req_o => bus_req,
    device_rsp_i => bus_rsp
  );


  -- Bus Arbiter ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      locked  <= '0';
      pending <= '0';
    elsif rising_edge(clk_i) then
      if (pending = '0') then -- idle, waiting for request
        locked  <= bus_req.stb and bus_req.lock;
        pending <= bus_req.stb;
      else -- access in progress
        if (locked = '0') then -- single access
          if (timeout = '1') or (xbus_err_i = '1') or (xbus_ack_i = '1') then
            pending <= '0';
          end if;
        else -- locked access (multiple accesses)
          if (timeout = '1') or (bus_req.lock = '0') then
            pending <= '0';
          end if;
        end if;
      end if;
    end if;
  end process arbiter;


  -- Bus Timeout ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  timeout_enabled:
  if TIMEOUT_VAL > 0 generate
    timeout_counter: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        timecnt <= (others => '0');
        timeout <= '0';
      elsif rising_edge(clk_i) then
        if (pending = '0') then
          timecnt <= timeout_c;
          timeout <= '0';
        else
          if (or_reduce_f(std_ulogic_vector(timecnt)) = '1') then
            timecnt <= timecnt - 1;
          else
            timeout <= '1';
          end if;
        end if;
      end if;
    end process timeout_counter;
  end generate;

  timeout_disabled:
  if TIMEOUT_VAL = 0 generate
    timecnt <= (others => '0');
    timeout <= '0';
  end generate;

  -- no-timeout warning --
  assert not (TIMEOUT_VAL = 0) report "[NEORV32] XBUS: no bus-timeout configured!" severity warning;


  -- XBUS Interface -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  xbus_adr_o <= bus_req.addr;
  xbus_dat_o <= bus_req.data;
  xbus_we_o  <= bus_req.rw;
  xbus_sel_o <= bus_req.ben;
  xbus_stb_o <= bus_req.stb;
  xbus_cyc_o <= bus_req.stb or pending;

  -- cycle type identifier (for the ENTIRE access; no burst termination type supported!) --
  xbus_cti_o <= "001" when (bus_req.amo = '1') else -- constant address burst
                "010" when (bus_req.burst = '1') else -- incrementing address burst
                "000"; -- single access

  -- access meta data (compatible to AXI4 "xPROT") --
  xbus_tag_o(2) <= bus_req.src; -- 0 = data access, 1 = instruction fetch
  xbus_tag_o(1) <= '0'; -- always "secure" access
  xbus_tag_o(0) <= bus_req.priv or bus_req.debug; -- 0 = unprivileged access, 1 = privileged access

  -- response gating --
  bus_rsp.data <= xbus_dat_i when (pending = '1') else (others => '0');
  bus_rsp.ack  <= pending and (timeout or xbus_err_i or xbus_ack_i);
  bus_rsp.err  <= pending and (timeout or xbus_err_i);


end neorv32_xbus_rtl;
