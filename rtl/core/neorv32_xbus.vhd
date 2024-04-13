-- ================================================================================ --
-- NEORV32 SoC - External Bus Interface (XBUS)                                      --
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
    -- Wishbone Interface Configuration --
    BUS_TIMEOUT : natural; -- cycles after an UNACKNOWLEDGED bus access triggers a bus fault exception
    PIPE_MODE   : boolean; -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
    ASYNC_RX    : boolean; -- use register buffer for RX data when false
    ASYNC_TX    : boolean  -- use register buffer for TX data when false
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
    xbus_we_o  : out std_ulogic; -- read/write
    xbus_sel_o : out std_ulogic_vector(03 downto 0); -- byte enable
    xbus_stb_o : out std_ulogic; -- strobe
    xbus_cyc_o : out std_ulogic; -- valid cycle
    xbus_ack_i : in  std_ulogic; -- transfer acknowledge
    xbus_err_i : in  std_ulogic  -- transfer error
  );
end neorv32_xbus;

architecture neorv32_xbus_rtl of neorv32_xbus is

  -- auto-configuration --
  constant async_rx_c : boolean := ASYNC_RX and PIPE_MODE; -- classic mode requires a sync RX path for the inter-cycle pause

  -- timeout enable --
  constant timeout_en_c : boolean := boolean(BUS_TIMEOUT /= 0); -- timeout enabled if BUS_TIMEOUT > 0

  -- bus arbiter
  type ctrl_t is record
    state    : std_ulogic;
    state_ff : std_ulogic;
    we       : std_ulogic;
    adr      : std_ulogic_vector(31 downto 0);
    wdat     : std_ulogic_vector(31 downto 0);
    rdat     : std_ulogic_vector(31 downto 0);
    sel      : std_ulogic_vector(03 downto 0);
    ack      : std_ulogic;
    err      : std_ulogic;
    timeout  : std_ulogic_vector(index_size_f(BUS_TIMEOUT) downto 0);
  end record;
  signal ctrl    : ctrl_t;
  signal stb_int : std_ulogic;
  signal cyc_int : std_ulogic;

  -- async RX gating --
  signal ack_gated   : std_ulogic;
  signal err_gated   : std_ulogic;
  signal rdata_gated : std_ulogic_vector(31 downto 0);

begin

  -- Configuration Info ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert false report
    "[NEORV32] External Bus Interface (XBUS) - " &
    cond_sel_string_f(PIPE_MODE, "PIPELINED", "CLASSIC/STANDARD") & " Wishbone b4 protocol, " &
    cond_sel_string_f(boolean(BUS_TIMEOUT /= 0), "auto-timeout, ", "NO auto-timeout, ") &
    cond_sel_string_f(async_rx_c, "ASYNC ", "registered ") & "RX, " &
    cond_sel_string_f(ASYNC_TX, "ASYNC ", "registered ") & "TX"
    severity note;

  -- async RX override warning --
  assert not ((ASYNC_RX = true) and (async_rx_c = false)) report
    "[NEORV32] XBUS - Non-pipelined/standard mode requires sync RX (auto-disabling async RX)." severity warning;

  -- zero timeout warning --
  assert not (BUS_TIMEOUT = 0) report
    "[NEORV32] XBUS - NO auto-timeout defined; can cause permanent CPU stall!" severity warning;


  -- Bus Arbiter -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl.state    <= '0';
      ctrl.state_ff <= '0';
      ctrl.we       <= '0';
      ctrl.adr      <= (others => '0');
      ctrl.wdat     <= (others => '0');
      ctrl.rdat     <= (others => '0');
      ctrl.sel      <= (others => '0');
      ctrl.timeout  <= (others => '0');
      ctrl.ack      <= '0';
      ctrl.err      <= '0';
    elsif rising_edge(clk_i) then
      -- defaults --
      ctrl.state_ff <= ctrl.state;
      ctrl.rdat     <= (others => '0'); -- required for internal output gating
      ctrl.ack      <= '0';
      ctrl.err      <= '0';
      ctrl.timeout  <= std_ulogic_vector(to_unsigned(BUS_TIMEOUT, index_size_f(BUS_TIMEOUT)+1));
      -- state machine --
      if (ctrl.state = '0') then -- IDLE, waiting for host request
        -- ------------------------------------------------------------
        if (bus_req_i.stb = '1') then -- request
          -- buffer (and gate) all outgoing signals --
          ctrl.we    <= bus_req_i.rw;
          ctrl.adr   <= bus_req_i.addr;
          ctrl.wdat  <= bus_req_i.data;
          ctrl.sel   <= bus_req_i.ben;
          ctrl.state <= '1';
        end if;
      else -- BUSY, transfer in progress
        -- ------------------------------------------------------------
        if (ctrl.we = '0') then -- sync output gate (keep output zero if write access)
          ctrl.rdat <= xbus_dat_i;
        end if;
        if (xbus_ack_i = '1') then -- normal bus termination
          ctrl.ack   <= '1';
          ctrl.state <= '0';
        elsif (xbus_err_i = '1') or ((timeout_en_c = true) and (or_reduce_f(ctrl.timeout) = '0')) then -- bus error or timeout
          ctrl.err   <= '1';
          ctrl.state <= '0';
        end if;
        -- timeout counter --
        if (timeout_en_c = true) then
          ctrl.timeout <= std_ulogic_vector(unsigned(ctrl.timeout) - 1); -- timeout counter
        end if;
      end if;
    end if;
  end process bus_arbiter;

  -- host access --
  stb_int <=  bus_req_i.stb                when (ASYNC_TX = true) else (ctrl.state and (not ctrl.state_ff));
  cyc_int <= (bus_req_i.stb or ctrl.state) when (ASYNC_TX = true) else  ctrl.state;

  xbus_adr_o <= bus_req_i.addr when (ASYNC_TX  = true) else ctrl.adr;
  xbus_dat_o <= bus_req_i.data when (ASYNC_TX  = true) else ctrl.wdat;
  xbus_we_o  <= bus_req_i.rw   when (ASYNC_TX  = true) else ctrl.we;
  xbus_sel_o <= bus_req_i.ben  when (ASYNC_TX  = true) else ctrl.sel;
  xbus_stb_o <= stb_int        when (PIPE_MODE = true) else cyc_int;
  xbus_cyc_o <= cyc_int;

  -- device response --
  ack_gated   <= xbus_ack_i when (ctrl.state = '1') else '0'; -- CPU ACK gate for "async" RX
  err_gated   <= xbus_err_i when (ctrl.state = '1') else '0'; -- CPU ERR gate for "async" RX
  rdata_gated <= xbus_dat_i when (ctrl.state = '1') and (ctrl.we = '0') else (others => '0'); -- async output gate

  bus_rsp_o.data <= ctrl.rdat when (async_rx_c = false) else rdata_gated;
  bus_rsp_o.ack  <= ctrl.ack  when (async_rx_c = false) else ack_gated;
  bus_rsp_o.err  <= ctrl.err  when (async_rx_c = false) else err_gated;


end neorv32_xbus_rtl;
