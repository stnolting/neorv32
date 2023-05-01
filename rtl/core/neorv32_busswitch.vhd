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
    PORT_A_READ_ONLY : boolean; -- set if port A is read-only
    PORT_B_READ_ONLY : boolean  -- set if port B is read-only
  );
  port (
    clk_i   : in  std_ulogic; -- global clock, rising edge
    rstn_i  : in  std_ulogic; -- global reset, low-active, async
    a_req_i : in  bus_req_t;  -- host port A: request bus
    a_rsp_o : out bus_rsp_t;  -- host port A: response bus
    b_req_i : in  bus_req_t;  -- host port B: request bus
    b_rsp_o : out bus_rsp_t;  -- host port B: response bus
    x_req_o : out bus_req_t;  -- device port request bus
    x_rsp_i : in  bus_rsp_t   -- device port response bus
  );
end neorv32_busswitch;

architecture neorv32_busswitch_rtl of neorv32_busswitch is

  -- access requests --
  signal a_rd_req_buf,  a_wr_req_buf  : std_ulogic;
  signal b_rd_req_buf,  b_wr_req_buf  : std_ulogic;
  signal a_req_current, a_req_pending : std_ulogic;
  signal b_req_current, b_req_pending : std_ulogic;

  -- internal bus lines --
  signal a_bus_ack, b_bus_ack : std_ulogic;
  signal a_bus_err, b_bus_err : std_ulogic;
  signal x_bus_we,  x_bus_re   : std_ulogic;

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
      a_rd_req_buf <= '0';
      a_wr_req_buf <= '0';
      b_rd_req_buf <= '0';
      b_wr_req_buf <= '0';
    elsif rising_edge(clk_i) then
      arbiter.state <= arbiter.state_nxt;
      -- port A requests --
      a_rd_req_buf <= (a_rd_req_buf or a_req_i.re) and (not (a_bus_err or a_bus_ack));
      a_wr_req_buf <= (a_wr_req_buf or a_req_i.we) and (not (a_bus_err or a_bus_ack)) and bool_to_ulogic_f(PORT_A_READ_ONLY = false);
      -- port B requests --
      b_rd_req_buf <= (b_rd_req_buf or b_req_i.re) and (not (b_bus_err or b_bus_ack));
      b_wr_req_buf <= (b_wr_req_buf or b_req_i.we) and (not (b_bus_err or b_bus_ack)) and bool_to_ulogic_f(PORT_B_READ_ONLY = false);
    end if;
  end process arbiter_sync;

  -- any current requests? --
  a_req_current <= (a_req_i.re or a_req_i.we) when (PORT_A_READ_ONLY = false) else a_req_i.re;
  b_req_current <= (b_req_i.re or b_req_i.we) when (PORT_B_READ_ONLY = false) else b_req_i.re;

  -- any pending requests? --
  a_req_pending <= (a_rd_req_buf or a_wr_req_buf) when (PORT_A_READ_ONLY = false) else a_rd_req_buf;
  b_req_pending <= (b_rd_req_buf or b_wr_req_buf) when (PORT_B_READ_ONLY = false) else b_rd_req_buf;

  -- FSM --
  arbiter_comb: process(arbiter, a_req_current, b_req_current, a_req_pending, b_req_pending,
                        a_rd_req_buf, a_wr_req_buf, b_rd_req_buf, b_wr_req_buf, x_rsp_i)
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
        if (a_req_current = '1') then -- current request from port A?
          arbiter.bus_sel   <= '0';
          arbiter.state_nxt <= A_BUSY;
        elsif (a_req_pending = '1') then -- pending request from port A?
          arbiter.bus_sel   <= '0';
          arbiter.state_nxt <= A_RETIRE;
        elsif (b_req_current = '1') then -- pending request from port B?
          arbiter.bus_sel   <= '1';
          arbiter.state_nxt <= B_BUSY;
        elsif (b_req_pending = '1') then -- current request from port B?
          arbiter.bus_sel   <= '1';
          arbiter.state_nxt <= B_RETIRE;
        end if;

      when A_BUSY => -- port A pending access
      -- ------------------------------------------------------------
        arbiter.bus_sel <= '0'; -- access from port A
        if (x_rsp_i.err = '1') or (x_rsp_i.ack = '1') then
-- [COMMENT NOTE] Direct return to IDLE to further promote port A access requests.
--        if (b_req_pending = '1') or (b_req_current = '1') then -- any request from B?
--          arbiter.state_nxt <= B_RETIRE;
--        else
            arbiter.state_nxt <= IDLE;
--        end if;
        end if;

      when A_RETIRE => -- retire port A pending access
      -- ------------------------------------------------------------
        arbiter.bus_sel   <= '0'; -- access from port A
        arbiter.we_trig   <= a_wr_req_buf;
        arbiter.re_trig   <= a_rd_req_buf;
        arbiter.state_nxt <= A_BUSY;

      when B_BUSY => -- port B pending access
      -- ------------------------------------------------------------
        arbiter.bus_sel <= '1'; -- access from port B
        if (x_rsp_i.err = '1') or (x_rsp_i.ack = '1') then
          if (a_req_pending = '1') or (a_req_current = '1') then -- any request from A?
            arbiter.state_nxt <= A_RETIRE;
          else
            arbiter.state_nxt <= IDLE;
          end if;
        end if;

      when B_RETIRE => -- retire port B pending access
      -- ------------------------------------------------------------
        arbiter.bus_sel   <= '1'; -- access from port B
        arbiter.we_trig   <= b_wr_req_buf;
        arbiter.re_trig   <= b_rd_req_buf;
        arbiter.state_nxt <= B_BUSY;

      when others => -- undefined
      -- ------------------------------------------------------------
        arbiter.state_nxt <= IDLE;

    end case;
  end process arbiter_comb;


  -- Peripheral Bus Switch ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  x_req_o.addr <= a_req_i.addr when (arbiter.bus_sel = '0') else b_req_i.addr;

  x_req_o.data <= b_req_i.data when (PORT_A_READ_ONLY = true) else
                  a_req_i.data when (PORT_B_READ_ONLY = true) else
                  a_req_i.data when (arbiter.bus_sel = '0')   else b_req_i.data;

  x_req_o.ben  <= b_req_i.ben when (PORT_A_READ_ONLY = true) else
                  a_req_i.ben when (PORT_B_READ_ONLY = true) else
                  a_req_i.ben when (arbiter.bus_sel = '0')   else b_req_i.ben;

  x_req_o.priv <= a_req_i.priv when (arbiter.bus_sel = '0') else b_req_i.priv;
  x_req_o.src  <= a_req_i.src  when (arbiter.bus_sel = '0') else b_req_i.src;

  x_bus_we     <= a_req_i.we when (arbiter.bus_sel = '0') else b_req_i.we;
  x_bus_re     <= a_req_i.re when (arbiter.bus_sel = '0') else b_req_i.re;
  x_req_o.we   <= x_bus_we or arbiter.we_trig;
  x_req_o.re   <= x_bus_re or arbiter.re_trig;

  a_rsp_o.data <= x_rsp_i.data;
  b_rsp_o.data <= x_rsp_i.data;

  a_bus_ack    <= x_rsp_i.ack when (arbiter.bus_sel = '0') else '0';
  b_bus_ack    <= x_rsp_i.ack when (arbiter.bus_sel = '1') else '0';
  a_rsp_o.ack  <= a_bus_ack;
  b_rsp_o.ack  <= b_bus_ack;

  a_bus_err    <= x_rsp_i.err when (arbiter.bus_sel = '0') else '0';
  b_bus_err    <= x_rsp_i.err when (arbiter.bus_sel = '1') else '0';
  a_rsp_o.err  <= a_bus_err;
  b_rsp_o.err  <= b_bus_err;


end neorv32_busswitch_rtl;
