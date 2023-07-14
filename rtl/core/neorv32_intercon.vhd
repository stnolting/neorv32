-- #################################################################################################
-- # << NEORV32 - 2-to-1 Bus Switch >>                                                             #
-- # ********************************************************************************************* #
-- # Allows to access a single bus ("p_bus") by two controller ports. Controller port A ("ca_bus") #
--# has priority over controller port B ("cb_bus").                                                #
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
-- [NOTE] Directly return to IDLE state to further promote port A access requests.
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


-- ############################################################################################################################
-- ############################################################################################################################


-- #################################################################################################
-- # << NEORV32 - Processor Interconnect >>                                                        #
-- # ********************************************************************************************* #
-- # Bus gateway to distribute the core's access to the processor's main memory sections:          #
-- # -> IMEM - internal instruction memory [optional]                                              #
-- # -> DMEM - internal data memory [optional]                                                     #
-- # -> XIP  - memory-mapped XIP flash [optional]                                                  #
-- # -> BOOT - internal bootloader ROM [optional]                                                  #
-- # -> IO   - internal IO devices [mandatory]                                                     #
-- # All accesses that do not match any of these sections are redirected to the "external" port.   #
-- # The gateway-internal bus monitor ensures that all processor-internal accesses are completed   #
-- # within a fixed time window.                                                                   #
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

entity neorv32_gateway is
  generic (
    TIMEOUT     : positive; -- internal bus timeout cycles
    -- IMEM port --
    IMEM_ENABLE : boolean;
    IMEM_BASE   : std_ulogic_vector(31 downto 0);
    IMEM_SIZE   : positive;
    -- DMEM port --
    DMEM_ENABLE : boolean;
    DMEM_BASE   : std_ulogic_vector(31 downto 0);
    DMEM_SIZE   : positive;
    -- XIP port --
    XIP_ENABLE  : boolean;
    XIP_BASE    : std_ulogic_vector(31 downto 0);
    XIP_SIZE    : positive;
    -- BOOT ROM port --
    BOOT_ENABLE : boolean;
    BOOT_BASE   : std_ulogic_vector(31 downto 0);
    BOOT_SIZE   : positive;
    -- IO port --
    IO_ENABLE   : boolean;
    IO_REQ_REG  : boolean;
    IO_RSP_REG  : boolean;
    IO_BASE     : std_ulogic_vector(31 downto 0);
    IO_SIZE     : positive;
    -- EXTERNAL port --
    EXT_ENABLE  : boolean
  );
  port (
    -- global control --
    clk_i      : in  std_ulogic; -- global clock, rising edge
    rstn_i     : in  std_ulogic; -- global reset, low-active, async
    -- host port --
    main_req_i : in  bus_req_t;  -- host request
    main_rsp_o : out bus_rsp_t;  -- host response
    -- section ports --
    imem_req_o : out bus_req_t;
    imem_rsp_i : in  bus_rsp_t;
    dmem_req_o : out bus_req_t;
    dmem_rsp_i : in  bus_rsp_t;
    xip_req_o  : out bus_req_t;
    xip_rsp_i  : in  bus_rsp_t;
    boot_req_o : out bus_req_t;
    boot_rsp_i : in  bus_rsp_t;
    io_req_o   : out bus_req_t;
    io_rsp_i   : in  bus_rsp_t;
    ext_req_o  : out bus_req_t;
    ext_rsp_i  : in  bus_rsp_t
  );
end neorv32_gateway;

architecture neorv32_gateway_rtl of neorv32_gateway is

  -- port select --
  constant port_imem_c : natural := 0;
  constant port_dmem_c : natural := 1;
  constant port_xip_c  : natural := 2;
  constant port_boot_c : natural := 3;
  constant port_io_c   : natural := 4;
  constant port_ext_c  : natural := 5;
  signal port_en : std_ulogic_vector(5 downto 0);

  -- IO buffer stage --
  signal io_req : bus_req_t;
  signal io_rsp : bus_rsp_t;

  -- response summary --
  signal int_rsp : bus_rsp_t;

  -- bus monitor --
  type keeper_t is record
    busy : std_ulogic;
    cnt  : std_ulogic_vector(index_size_f(TIMEOUT) downto 0);
    err  : std_ulogic;
    halt : std_ulogic;
  end record;
  signal keeper : keeper_t;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert false report
    "NEORV32 PROCESSOR CONFIG NOTE: Processor-internal bus timeout after " & integer'image(TIMEOUT) & " cycles." severity note;


  -- Address Section Decoder ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  port_en(port_imem_c) <= '1' when (main_req_i.addr(31 downto index_size_f(IMEM_SIZE)) = IMEM_BASE(31 downto index_size_f(IMEM_SIZE))) and (IMEM_ENABLE = true) else '0';
  port_en(port_dmem_c) <= '1' when (main_req_i.addr(31 downto index_size_f(DMEM_SIZE)) = DMEM_BASE(31 downto index_size_f(DMEM_SIZE))) and (DMEM_ENABLE = true) else '0';
  port_en(port_xip_c)  <= '1' when (main_req_i.addr(31 downto index_size_f(XIP_SIZE))  = XIP_BASE( 31 downto index_size_f(XIP_SIZE)))  and (XIP_ENABLE  = true) else '0';
  port_en(port_boot_c) <= '1' when (main_req_i.addr(31 downto index_size_f(BOOT_SIZE)) = BOOT_BASE(31 downto index_size_f(BOOT_SIZE))) and (BOOT_ENABLE = true) else '0';
  port_en(port_io_c)   <= '1' when (main_req_i.addr(31 downto index_size_f(IO_SIZE))   = IO_BASE(  31 downto index_size_f(IO_SIZE)))   and (IO_ENABLE   = true) else '0';

  -- accesses to the "void" (= no section is matched) are redirected to the external bus interface --
  port_en(port_ext_c) <= '1' when (port_en(port_imem_c) = '0') and
                                  (port_en(port_dmem_c) = '0') and
                                  (port_en(port_xip_c)  = '0') and
                                  (port_en(port_boot_c) = '0') and
                                  (port_en(port_io_c)   = '0') and (EXT_ENABLE = true) else '0';


  -- Bus Request ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  request: process(main_req_i, port_en)
  begin
    imem_req_o    <= main_req_i;
    imem_req_o.we <= main_req_i.we and port_en(port_imem_c);
    imem_req_o.re <= main_req_i.re and port_en(port_imem_c);
    --
    dmem_req_o    <= main_req_i;
    dmem_req_o.we <= main_req_i.we and port_en(port_dmem_c);
    dmem_req_o.re <= main_req_i.re and port_en(port_dmem_c);
    --
    xip_req_o     <= main_req_i;
    xip_req_o.we  <= main_req_i.we and port_en(port_xip_c);
    xip_req_o.re  <= main_req_i.re and port_en(port_xip_c);
    --
    boot_req_o    <= main_req_i;
    boot_req_o.we <= main_req_i.we and port_en(port_boot_c);
    boot_req_o.re <= main_req_i.re and port_en(port_boot_c);
    --
    io_req        <= main_req_i;
    io_req.we     <= main_req_i.we and port_en(port_io_c);
    io_req.re     <= main_req_i.re and port_en(port_io_c);
    --
    ext_req_o     <= main_req_i;
    ext_req_o.we  <= main_req_i.we and port_en(port_ext_c);
    ext_req_o.re  <= main_req_i.re and port_en(port_ext_c);
  end process request;


  -- IO Port Register Stages ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  io_req_buffer_true:
  if (IO_REQ_REG = true) generate
    io_req_buffer: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        io_req_o <= req_terminate_c;
      elsif rising_edge(clk_i) then
        io_req_o <= io_req;
      end if;
    end process io_req_buffer;
  end generate;

  io_req_buffer_false:
  if (IO_REQ_REG = false) generate
    io_req_o <= io_req;
  end generate;


  io_rsp_buffer_true:
  if (IO_RSP_REG = true) generate
    io_rsp_buffer: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        io_rsp <= rsp_terminate_c;
      elsif rising_edge(clk_i) then
        io_rsp <= io_rsp_i;
      end if;
    end process io_rsp_buffer;
  end generate;

  io_rsp_buffer_false:
  if (IO_RSP_REG = false) generate
    io_rsp <= io_rsp_i;
  end generate;


  -- Bus Response ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  response: process(imem_rsp_i, dmem_rsp_i, boot_rsp_i, xip_rsp_i, io_rsp, ext_rsp_i)
    variable tmp_v : bus_rsp_t;
  begin
    tmp_v := rsp_terminate_c; -- start will all-zero
    if (IMEM_ENABLE = true) then
      tmp_v.data := tmp_v.data or imem_rsp_i.data;
      tmp_v.ack  := tmp_v.ack  or imem_rsp_i.ack;
      tmp_v.err  := tmp_v.err  or imem_rsp_i.err;
    end if;
    if (DMEM_ENABLE = true) then
      tmp_v.data := tmp_v.data or dmem_rsp_i.data;
      tmp_v.ack  := tmp_v.ack  or dmem_rsp_i.ack;
      tmp_v.err  := tmp_v.err  or dmem_rsp_i.err;
    end if;
    if (XIP_ENABLE = true) then
      tmp_v.data := tmp_v.data or xip_rsp_i.data;
      tmp_v.ack  := tmp_v.ack  or xip_rsp_i.ack;
      tmp_v.err  := tmp_v.err  or xip_rsp_i.err;
    end if;
    if (BOOT_ENABLE = true) then
      tmp_v.data := tmp_v.data or boot_rsp_i.data;
      tmp_v.ack  := tmp_v.ack  or boot_rsp_i.ack;
      tmp_v.err  := tmp_v.err  or boot_rsp_i.err;
    end if;
    if (IO_ENABLE = true) then
      tmp_v.data := tmp_v.data or io_rsp.data;
      tmp_v.ack  := tmp_v.ack  or io_rsp.ack;
      tmp_v.err  := tmp_v.err  or io_rsp.err;
    end if;
    if (EXT_ENABLE = true) then
      tmp_v.data := tmp_v.data or ext_rsp_i.data;
      tmp_v.ack  := tmp_v.ack  or ext_rsp_i.ack;
      tmp_v.err  := tmp_v.err  or ext_rsp_i.err;
    end if;
    int_rsp <= tmp_v;
  end process response;

  -- host response --
  main_rsp_o.data <= int_rsp.data;
  main_rsp_o.ack  <= int_rsp.ack;
  main_rsp_o.err  <= keeper.err;


  -- Bus Monitor (aka "the KEEPER") ---------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_monitor: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      keeper.busy <= '0';
      keeper.cnt  <= (others => '0');
      keeper.err  <= '0';
      keeper.halt <= '0';
    elsif rising_edge(clk_i) then
      -- defaults --
      keeper.err  <= '0';
      keeper.halt <= port_en(port_xip_c) or port_en(port_ext_c); -- no timeout if XIP or EXTERNAL access
      -- fsm --
      if (keeper.busy = '0') then -- bus idle
        keeper.cnt <= std_ulogic_vector(to_unsigned(TIMEOUT, keeper.cnt'length));
        if (main_req_i.re = '1') or (main_req_i.we = '1') then
          keeper.busy <= '1';
        end if;
      else -- bus access in progress
        keeper.cnt <= std_ulogic_vector(unsigned(keeper.cnt) - 1);
        if (int_rsp.ack = '1') then -- normal access termination
          keeper.busy <= '0';
        elsif (int_rsp.err = '1') or ((or_reduce_f(keeper.cnt) = '0') and (keeper.halt = '0')) then -- bus error or timeout
          keeper.err  <= '1';
          keeper.busy <= '0';
        end if;
      end if;
    end if;
  end process bus_monitor;


end neorv32_gateway_rtl;
