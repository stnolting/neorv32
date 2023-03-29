-- #################################################################################################
-- # << NEORV32 - Bus Switch >>                                                                    #
-- # ********************************************************************************************* #
-- # Allows to access a single peripheral bus ("p_bus") by two controller ports. Controller port A #
-- # ("ca_bus") has priority over controller port B ("cb_bus").                                    #
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

entity neorv32_busswitch is
  generic (
    PORT_CA_READ_ONLY : boolean; -- set if controller port A is read-only
    PORT_CB_READ_ONLY : boolean  -- set if controller port B is read-only
  );
  port (
    -- global control --
    clk_i           : in  std_ulogic; -- global clock, rising edge
    rstn_i          : in  std_ulogic; -- global reset, low-active, async
    -- controller interface a --
    ca_bus_priv_i   : in  std_ulogic; -- current privilege level
    ca_bus_cached_i : in  std_ulogic; -- set if cached transfer
    ca_bus_addr_i   : in  std_ulogic_vector(31 downto 0); -- bus access address
    ca_bus_rdata_o  : out std_ulogic_vector(31 downto 0); -- bus read data
    ca_bus_wdata_i  : in  std_ulogic_vector(31 downto 0); -- bus write data
    ca_bus_ben_i    : in  std_ulogic_vector(03 downto 0); -- byte enable
    ca_bus_we_i     : in  std_ulogic; -- write enable
    ca_bus_re_i     : in  std_ulogic; -- read enable
    ca_bus_ack_o    : out std_ulogic; -- bus transfer acknowledge
    ca_bus_err_o    : out std_ulogic; -- bus transfer error
    -- controller interface b --
    cb_bus_priv_i   : in  std_ulogic; -- current privilege level
    cb_bus_cached_i : in  std_ulogic; -- set if cached transfer
    cb_bus_addr_i   : in  std_ulogic_vector(31 downto 0); -- bus access address
    cb_bus_rdata_o  : out std_ulogic_vector(31 downto 0); -- bus read data
    cb_bus_wdata_i  : in  std_ulogic_vector(31 downto 0); -- bus write data
    cb_bus_ben_i    : in  std_ulogic_vector(03 downto 0); -- byte enable
    cb_bus_we_i     : in  std_ulogic; -- write enable
    cb_bus_re_i     : in  std_ulogic; -- read enable
    cb_bus_ack_o    : out std_ulogic; -- bus transfer acknowledge
    cb_bus_err_o    : out std_ulogic; -- bus transfer error
    -- peripheral bus --
    p_bus_priv_o    : out std_ulogic; -- current privilege level
    p_bus_cached_o  : out std_ulogic; -- set if cached transfer
    p_bus_src_o     : out std_ulogic; -- access source: 0 = A, 1 = B
    p_bus_addr_o    : out std_ulogic_vector(31 downto 0); -- bus access address
    p_bus_rdata_i   : in  std_ulogic_vector(31 downto 0); -- bus read data
    p_bus_wdata_o   : out std_ulogic_vector(31 downto 0); -- bus write data
    p_bus_ben_o     : out std_ulogic_vector(03 downto 0); -- byte enable
    p_bus_we_o      : out std_ulogic; -- write enable
    p_bus_re_o      : out std_ulogic; -- read enable
    p_bus_ack_i     : in  std_ulogic; -- bus transfer acknowledge
    p_bus_err_i     : in  std_ulogic  -- bus transfer error
  );
end neorv32_busswitch;

architecture neorv32_busswitch_rtl of neorv32_busswitch is

  -- access requests --
  signal ca_rd_req_buf,  ca_wr_req_buf  : std_ulogic;
  signal cb_rd_req_buf,  cb_wr_req_buf  : std_ulogic;
  signal ca_req_current, ca_req_pending : std_ulogic;
  signal cb_req_current, cb_req_pending : std_ulogic;

  -- internal bus lines --
  signal ca_bus_ack, cb_bus_ack : std_ulogic;
  signal ca_bus_err, cb_bus_err : std_ulogic;
  signal p_bus_we,   p_bus_re   : std_ulogic;

  -- access arbiter --
  type arbiter_state_t is (IDLE, A_BUSY, A_RETIRE, B_BUSY, B_RETIRE);
  type arbiter_t is record
    state     : arbiter_state_t;
    state_nxt : arbiter_state_t;
    bus_sel   : std_ulogic;
    re_trig   : std_ulogic;
    we_trig   : std_ulogic;
  end record;
  signal arbiter : arbiter_t;

begin

  -- Access Arbiter -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  arbiter_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      arbiter.state <= IDLE;
      ca_rd_req_buf <= '0';
      ca_wr_req_buf <= '0';
      cb_rd_req_buf <= '0';
      cb_wr_req_buf <= '0';
    elsif rising_edge(clk_i) then
      arbiter.state <= arbiter.state_nxt;
      -- port A requests --
      ca_rd_req_buf <= (ca_rd_req_buf or ca_bus_re_i) and (not (ca_bus_err or ca_bus_ack));
      ca_wr_req_buf <= (ca_wr_req_buf or ca_bus_we_i) and (not (ca_bus_err or ca_bus_ack)) and bool_to_ulogic_f(PORT_CA_READ_ONLY = false);
      -- port B requests --
      cb_rd_req_buf <= (cb_rd_req_buf or cb_bus_re_i) and (not (cb_bus_err or cb_bus_ack));
      cb_wr_req_buf <= (cb_wr_req_buf or cb_bus_we_i) and (not (cb_bus_err or cb_bus_ack)) and bool_to_ulogic_f(PORT_CB_READ_ONLY = false);
    end if;
  end process arbiter_sync;

  -- any current requests? --
  ca_req_current <= (ca_bus_re_i or ca_bus_we_i) when (PORT_CA_READ_ONLY = false) else ca_bus_re_i;
  cb_req_current <= (cb_bus_re_i or cb_bus_we_i) when (PORT_CB_READ_ONLY = false) else cb_bus_re_i;

  -- any pending requests? --
  ca_req_pending <= (ca_rd_req_buf or ca_wr_req_buf) when (PORT_CA_READ_ONLY = false) else ca_rd_req_buf;
  cb_req_pending <= (cb_rd_req_buf or cb_wr_req_buf) when (PORT_CB_READ_ONLY = false) else cb_rd_req_buf;

  -- FSM --
  arbiter_comb: process(arbiter, ca_req_current, cb_req_current, ca_req_pending, cb_req_pending,
                        ca_rd_req_buf, ca_wr_req_buf, cb_rd_req_buf, cb_wr_req_buf, p_bus_ack_i, p_bus_err_i)
  begin
    -- arbiter defaults --
    arbiter.state_nxt <= arbiter.state;
    arbiter.bus_sel   <= '0';
    arbiter.we_trig   <= '0';
    arbiter.re_trig   <= '0';

    -- state machine --
    case arbiter.state is

      when IDLE => -- wait for requests
      -- ------------------------------------------------------------
        if (ca_req_current = '1') then -- current request from port A?
          arbiter.bus_sel   <= '0';
          arbiter.state_nxt <= A_BUSY;
        elsif (ca_req_pending = '1') then -- pending request from port A?
          arbiter.bus_sel   <= '0';
          arbiter.state_nxt <= A_RETIRE;
        elsif (cb_req_current = '1') then -- pending request from port B?
          arbiter.bus_sel   <= '1';
          arbiter.state_nxt <= B_BUSY;
        elsif (cb_req_pending = '1') then -- current request from port B?
          arbiter.bus_sel   <= '1';
          arbiter.state_nxt <= B_RETIRE;
        end if;

      when A_BUSY => -- port A pending access
      -- ------------------------------------------------------------
        arbiter.bus_sel <= '0'; -- access from port A
        if (p_bus_err_i = '1') or (p_bus_ack_i = '1') then
          if (cb_req_pending = '1') or (cb_req_current = '1') then -- any request from B?
            arbiter.state_nxt <= B_RETIRE;
          else
            arbiter.state_nxt <= IDLE;
          end if;
        end if;

      when A_RETIRE => -- retire port A pending access
      -- ------------------------------------------------------------
        arbiter.bus_sel   <= '0'; -- access from port A
        arbiter.we_trig   <= ca_wr_req_buf;
        arbiter.re_trig   <= ca_rd_req_buf;
        arbiter.state_nxt <= A_BUSY;

      when B_BUSY => -- port B pending access
      -- ------------------------------------------------------------
        arbiter.bus_sel <= '1'; -- access from port B
        if (p_bus_err_i = '1') or (p_bus_ack_i = '1') then
          if (ca_req_pending = '1') or (ca_req_current = '1') then -- any request from A?
            arbiter.state_nxt <= A_RETIRE;
          else
            arbiter.state_nxt <= IDLE;
          end if;
        end if;

      when B_RETIRE => -- retire port B pending access
      -- ------------------------------------------------------------
        arbiter.bus_sel   <= '1'; -- access from port B
        arbiter.we_trig   <= cb_wr_req_buf;
        arbiter.re_trig   <= cb_rd_req_buf;
        arbiter.state_nxt <= B_BUSY;

      when others => -- undefined
      -- ------------------------------------------------------------
        arbiter.state_nxt <= IDLE;

    end case;
  end process arbiter_comb;


  -- Peripheral Bus Switch ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  p_bus_addr_o   <= ca_bus_addr_i when (arbiter.bus_sel = '0') else cb_bus_addr_i;

  p_bus_wdata_o  <= cb_bus_wdata_i when (PORT_CA_READ_ONLY = true) else
                    ca_bus_wdata_i when (PORT_CB_READ_ONLY = true) else
                    ca_bus_wdata_i when (arbiter.bus_sel = '0')    else cb_bus_wdata_i;

  p_bus_ben_o    <= cb_bus_ben_i when (PORT_CA_READ_ONLY = true) else
                    ca_bus_ben_i when (PORT_CB_READ_ONLY = true) else
                    ca_bus_ben_i when (arbiter.bus_sel = '0')    else cb_bus_ben_i;

  p_bus_cached_o <= ca_bus_cached_i when (arbiter.bus_sel = '0') else cb_bus_cached_i;
  p_bus_priv_o   <= ca_bus_priv_i   when (arbiter.bus_sel = '0') else cb_bus_priv_i;

  p_bus_we       <= ca_bus_we_i when (arbiter.bus_sel = '0') else cb_bus_we_i;
  p_bus_re       <= ca_bus_re_i when (arbiter.bus_sel = '0') else cb_bus_re_i;
  p_bus_we_o     <= p_bus_we or arbiter.we_trig;
  p_bus_re_o     <= p_bus_re or arbiter.re_trig;

  p_bus_src_o    <= arbiter.bus_sel;

  ca_bus_rdata_o <= p_bus_rdata_i;
  cb_bus_rdata_o <= p_bus_rdata_i;

  ca_bus_ack     <= p_bus_ack_i when (arbiter.bus_sel = '0') else '0';
  cb_bus_ack     <= p_bus_ack_i when (arbiter.bus_sel = '1') else '0';
  ca_bus_ack_o   <= ca_bus_ack;
  cb_bus_ack_o   <= cb_bus_ack;

  ca_bus_err     <= p_bus_err_i when (arbiter.bus_sel = '0') else '0';
  cb_bus_err     <= p_bus_err_i when (arbiter.bus_sel = '1') else '0';
  ca_bus_err_o   <= ca_bus_err;
  cb_bus_err_o   <= cb_bus_err;


end neorv32_busswitch_rtl;
