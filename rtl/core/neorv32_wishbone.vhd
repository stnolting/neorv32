-- #################################################################################################
-- # << NEORV32 - External Bus Interface (WISHBONE) >>                                             #
-- # ********************************************************************************************* #
-- # Wishbone accesses can have a response latency of up to BUS_TIMEOUT - 1 cycles or an infinite  #
-- # response time if BUS_TIMEOUT = 0 (not recommended!).                                          #
-- #                                                                                               #
-- # The Wishbone gateway registers all outgoing signals. These signals will remain stable (gated) #
-- # if there is no active Wishbone access. By default, also the incoming signals are registered,  #
-- # too. this can be disabled by setting ASYNC_RX = false.                                        #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_wishbone is
  generic (
    -- Wishbone Interface Configuration --
    BUS_TIMEOUT : natural; -- cycles after an UNACKNOWLEDGED bus access triggers a bus fault exception
    PIPE_MODE   : boolean; -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
    BIG_ENDIAN  : boolean; -- byte order: true=big-endian, false=little-endian
    ASYNC_RX    : boolean; -- use register buffer for RX data when false
    ASYNC_TX    : boolean  -- use register buffer for TX data when false
  );
  port (
    clk_i     : in  std_ulogic; -- global clock line
    rstn_i    : in  std_ulogic; -- global reset line, low-active
    bus_req_i : in  bus_req_t;  -- bus request
    bus_rsp_o : out bus_rsp_t;  -- bus response
    --
    wb_tag_o  : out std_ulogic_vector(02 downto 0); -- request tag
    wb_adr_o  : out std_ulogic_vector(31 downto 0); -- address
    wb_dat_i  : in  std_ulogic_vector(31 downto 0); -- read data
    wb_dat_o  : out std_ulogic_vector(31 downto 0); -- write data
    wb_we_o   : out std_ulogic; -- read/write
    wb_sel_o  : out std_ulogic_vector(03 downto 0); -- byte enable
    wb_stb_o  : out std_ulogic; -- strobe
    wb_cyc_o  : out std_ulogic; -- valid cycle
    wb_ack_i  : in  std_ulogic; -- transfer acknowledge
    wb_err_i  : in  std_ulogic  -- transfer error
  );
end neorv32_wishbone;

architecture neorv32_wishbone_rtl of neorv32_wishbone is

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
    src      : std_ulogic;
    priv     : std_ulogic;
  end record;
  signal ctrl    : ctrl_t;
  signal stb_int : std_ulogic;
  signal cyc_int : std_ulogic;
  signal rdata   : std_ulogic_vector(31 downto 0);

  -- endianness conversion --
  signal end_wdata  : std_ulogic_vector(31 downto 0);
  signal end_byteen : std_ulogic_vector(03 downto 0);

  -- async RX gating --
  signal ack_gated   : std_ulogic;
  signal err_gated   : std_ulogic;
  signal rdata_gated : std_ulogic_vector(31 downto 0);

begin

  -- Configuration Info ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert false report
    "NEORV32 PROCESSOR CONFIG NOTE: Ext. Bus Interface - " &
    cond_sel_string_f(PIPE_MODE, "PIPELINED", "CLASSIC/STANDARD") & " Wishbone protocol, " &
    cond_sel_string_f(boolean(BUS_TIMEOUT /= 0), "auto-timeout (" & integer'image(BUS_TIMEOUT) & " cycles), ", "NO auto-timeout, ") &
    cond_sel_string_f(BIG_ENDIAN, "BIG", "LITTLE") & "-endian byte order, " &
    cond_sel_string_f(async_rx_c, "ASYNC ", "registered ") & "RX, " &
    cond_sel_string_f(ASYNC_TX, "ASYNC ", "registered ") & "TX"
    severity note;

  -- async RX override warning --
  assert not ((ASYNC_RX = true) and (async_rx_c = false)) report
    "NEORV32 PROCESSOR CONFIG WARNING! Ext. Bus Interface - Non-pipelined/standard mode requires sync RX (auto-disabling async RX)." severity warning;

  -- zero timeout warning --
  assert not (BUS_TIMEOUT = 0) report
    "NEORV32 PROCESSOR CONFIG WARNING! Ext. Bus Interface - NO auto-timeout defined; can cause permanent CPU stall!" severity warning;


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
      ctrl.src      <= '0';
      ctrl.priv     <= '0';
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
        if (bus_req_i.we = '1') or (bus_req_i.re = '1') then -- request
          -- buffer (and gate) all outgoing signals --
          ctrl.we    <= bus_req_i.we;
          ctrl.adr   <= bus_req_i.addr;
          ctrl.src   <= bus_req_i.src;
          ctrl.priv  <= bus_req_i.priv;
          ctrl.wdat  <= end_wdata;
          ctrl.sel   <= end_byteen;
          ctrl.state <= '1';
        end if;

      else -- BUSY, transfer in progress
        -- ------------------------------------------------------------
        if (ctrl.we = '0') then -- sync output gate (keep output zero if write access)
          ctrl.rdat <= wb_dat_i;
        end if;
        if (wb_ack_i = '1') then -- normal bus termination
          ctrl.ack   <= '1';
          ctrl.state <= '0';
        elsif (wb_err_i = '1') or ((timeout_en_c = true) and (or_reduce_f(ctrl.timeout) = '0')) then -- bus error or timeout
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

  -- endianness conversion --
  end_wdata  <= bswap32_f(bus_req_i.data) when (BIG_ENDIAN = true) else bus_req_i.data;
  end_byteen <= bit_rev_f(bus_req_i.ben)  when (BIG_ENDIAN = true) else bus_req_i.ben;

  -- host access --
  ack_gated   <= wb_ack_i when (ctrl.state = '1') else '0'; -- CPU ACK gate for "async" RX
  err_gated   <= wb_err_i when (ctrl.state = '1') else '0'; -- CPU ERR gate for "async" RX
  rdata_gated <= wb_dat_i when (ctrl.state = '1') and (ctrl.we = '0') else (others => '0'); -- async output gate

  rdata          <= ctrl.rdat when (async_rx_c = false) else rdata_gated;
  bus_rsp_o.data <= rdata when (BIG_ENDIAN = false) else bswap32_f(rdata); -- endianness conversion
  bus_rsp_o.ack  <= ctrl.ack when (async_rx_c = false) else ack_gated;
  bus_rsp_o.err  <= ctrl.err when (async_rx_c = false) else err_gated;

  -- wishbone interface --
  wb_tag_o(0) <= bus_req_i.priv when (ASYNC_TX = true) else ctrl.priv; -- 0 = unprivileged (U-mode), 1 = privileged (M-mode)
  wb_tag_o(1) <= '0'; -- 0 = secure, 1 = non-secure
  wb_tag_o(2) <= bus_req_i.src when (ASYNC_TX = true) else ctrl.src; -- 0 = data access, 1 = instruction access

  stb_int <=  (bus_req_i.we or bus_req_i.re)                when (ASYNC_TX = true) else (ctrl.state and (not ctrl.state_ff));
  cyc_int <= ((bus_req_i.we or bus_req_i.re) or ctrl.state) when (ASYNC_TX = true) else  ctrl.state;

  wb_adr_o <= bus_req_i.addr when (ASYNC_TX = true) else ctrl.adr;
  wb_dat_o <= bus_req_i.data when (ASYNC_TX = true) else ctrl.wdat;
  wb_we_o  <= (bus_req_i.we or (ctrl.we and ctrl.state)) when (ASYNC_TX = true) else ctrl.we;
  wb_sel_o <= end_byteen when (ASYNC_TX = true) else ctrl.sel;
  wb_stb_o <= stb_int    when (PIPE_MODE = true) else cyc_int;
  wb_cyc_o <= cyc_int;


end neorv32_wishbone_rtl;
