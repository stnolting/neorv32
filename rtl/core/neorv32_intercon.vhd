-- #################################################################################################
-- # << NEORV32 - Processor Bus Infrastructure: 2-to-1 Bus Switch >>                               #
-- # ********************************************************************************************* #
-- # Allows to access a single device bus X by two controller ports A and B.                       #
-- # Controller port A has priority over controller port B.                                        #
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

entity neorv32_bus_switch is
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
end neorv32_bus_switch;

architecture neorv32_bus_switch_rtl of neorv32_bus_switch is

  -- access arbiter --
  type arbiter_state_t is (IDLE, BUSY_A, BUSY_B);
  type arbiter_t is record
    state     : arbiter_state_t;
    state_nxt : arbiter_state_t;
    a_req     : std_ulogic;
    b_req     : std_ulogic;
    sel       : std_ulogic;
    stb       : std_ulogic;
  end record;
  signal arbiter : arbiter_t;

  -- internal bus lines --
  signal a_bus_ack, b_bus_ack, a_bus_err, b_bus_err : std_ulogic;

begin

  -- Access Arbiter -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  arbiter_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      arbiter.state <= IDLE;
      arbiter.a_req <= '0';
      arbiter.b_req <= '0';
    elsif rising_edge(clk_i) then
      arbiter.state <= arbiter.state_nxt;
      -- set on STB, clear on ACK or ERR --
      arbiter.a_req <= (arbiter.a_req or a_req_i.stb) and (a_bus_err nor a_bus_ack);
      arbiter.b_req <= (arbiter.b_req or b_req_i.stb) and (b_bus_err nor b_bus_ack);
    end if;
  end process arbiter_sync;

  -- fsm --
  arbiter_comb: process(arbiter, a_req_i, b_req_i, x_rsp_i)
  begin
    -- arbiter defaults --
    arbiter.state_nxt <= arbiter.state;
    arbiter.sel       <= '0';
    arbiter.stb       <= '0';

    -- state machine --
    case arbiter.state is

      when IDLE => -- wait for requests
      -- ------------------------------------------------------------
        if (a_req_i.stb = '1') or (arbiter.a_req = '1') then -- any request from port A (prioritized)?
          arbiter.sel       <= '0';
          arbiter.stb       <= '1';
          arbiter.state_nxt <= BUSY_A;
        elsif (b_req_i.stb = '1') or (arbiter.b_req = '1') then -- any request from port B?
          arbiter.sel       <= '1';
          arbiter.stb       <= '1';
          arbiter.state_nxt <= BUSY_B;
        end if;

      when BUSY_A => -- port A access in progress
      -- ------------------------------------------------------------
        arbiter.sel <= '0';
        if (x_rsp_i.err = '1') or (x_rsp_i.ack = '1') then
          arbiter.state_nxt <= IDLE;
        end if;

      when BUSY_B => -- port B access in progress
      -- ------------------------------------------------------------
        arbiter.sel <= '1';
        if (x_rsp_i.err = '1') or (x_rsp_i.ack = '1') then
          arbiter.state_nxt <= IDLE;
        end if;

      when others => -- undefined
      -- ------------------------------------------------------------
        arbiter.state_nxt <= IDLE;

    end case;
  end process arbiter_comb;


  -- Request Switch -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  x_req_o.addr <= a_req_i.addr when (arbiter.sel = '0') else b_req_i.addr;
  x_req_o.rvso <= a_req_i.rvso when (arbiter.sel = '0') else b_req_i.rvso;
  x_req_o.priv <= a_req_i.priv when (arbiter.sel = '0') else b_req_i.priv;
  x_req_o.src  <= a_req_i.src  when (arbiter.sel = '0') else b_req_i.src;
  x_req_o.rw   <= a_req_i.rw   when (arbiter.sel = '0') else b_req_i.rw;

  x_req_o.data <= b_req_i.data when (PORT_A_READ_ONLY = true) else
                  a_req_i.data when (PORT_B_READ_ONLY = true) else
                  a_req_i.data when (arbiter.sel = '0')  else b_req_i.data;

  x_req_o.ben  <= b_req_i.ben when (PORT_A_READ_ONLY = true) else
                  a_req_i.ben when (PORT_B_READ_ONLY = true) else
                  a_req_i.ben when (arbiter.sel = '0')  else b_req_i.ben;

  x_req_o.stb  <= arbiter.stb;


  -- Response Switch ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  a_rsp_o.data <= x_rsp_i.data;
  b_rsp_o.data <= x_rsp_i.data;

  a_bus_ack    <= x_rsp_i.ack when (arbiter.sel = '0') else '0';
  b_bus_ack    <= x_rsp_i.ack when (arbiter.sel = '1') else '0';
  a_rsp_o.ack  <= a_bus_ack;
  b_rsp_o.ack  <= b_bus_ack;

  a_bus_err    <= x_rsp_i.err when (arbiter.sel = '0') else '0';
  b_bus_err    <= x_rsp_i.err when (arbiter.sel = '1') else '0';
  a_rsp_o.err  <= a_bus_err;
  b_rsp_o.err  <= b_bus_err;


end neorv32_bus_switch_rtl;


-- ############################################################################################################################
-- ############################################################################################################################


-- #################################################################################################
-- # << NEORV32 - Processor Bus Infrastructure: Section Gateway >>                                 #
-- # ********************************************************************************************* #
-- # Bus gateway to distribute the core's access to the processor's main address sections:         #
-- # -> IMEM - internal instruction memory                                                         #
-- # -> DMEM - internal data memory                                                                #
-- # -> XIP  - memory-mapped XIP flash                                                             #
-- # -> BOOT - internal bootloader ROM                                                             #
-- # -> IO   - internal IO devices                                                                 #
-- # All accesses that do not match any of these sections are redirected to the "external" port.   #
-- # The gateway-internal bus monitor ensures that all processor-internal accesses are completed   #
-- # within a fixed time window; a bus error is triggered otherwise.                               #
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

entity neorv32_bus_gateway is
  generic (
    TIMEOUT     : natural; -- internal bus timeout cycles
    -- IMEM port --
    IMEM_ENABLE : boolean;
    IMEM_BASE   : std_ulogic_vector(31 downto 0);
    IMEM_SIZE   : natural;
    -- DMEM port --
    DMEM_ENABLE : boolean;
    DMEM_BASE   : std_ulogic_vector(31 downto 0);
    DMEM_SIZE   : natural;
    -- XIP port --
    XIP_ENABLE  : boolean;
    XIP_BASE    : std_ulogic_vector(31 downto 0);
    XIP_SIZE    : natural;
    -- BOOT ROM port --
    BOOT_ENABLE : boolean;
    BOOT_BASE   : std_ulogic_vector(31 downto 0);
    BOOT_SIZE   : natural;
    -- IO port --
    IO_ENABLE   : boolean;
    IO_BASE     : std_ulogic_vector(31 downto 0);
    IO_SIZE     : natural;
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
end neorv32_bus_gateway;

architecture neorv32_bus_gateway_rtl of neorv32_bus_gateway is

  -- port select --
  signal port_sel : std_ulogic_vector(5 downto 0);

  -- list of enabled gateway ports --
  type port_en_list_t is array (0 to 5) of boolean;
  constant port_en_list_c : port_en_list_t := (IMEM_ENABLE, DMEM_ENABLE, XIP_ENABLE, BOOT_ENABLE, IO_ENABLE, EXT_ENABLE);

  -- gateway ports combined as arrays --
  type port_req_t is array (0 to 5) of bus_req_t;
  type port_rsp_t is array (0 to 5) of bus_rsp_t;
  signal port_req : port_req_t;
  signal port_rsp : port_rsp_t;

  -- summarized response --
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

  -- Address Section Decoder ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  port_sel(0) <= '1' when (main_req_i.addr(31 downto index_size_f(IMEM_SIZE)) = IMEM_BASE(31 downto index_size_f(IMEM_SIZE))) and (IMEM_ENABLE = true) else '0';
  port_sel(1) <= '1' when (main_req_i.addr(31 downto index_size_f(DMEM_SIZE)) = DMEM_BASE(31 downto index_size_f(DMEM_SIZE))) and (DMEM_ENABLE = true) else '0';
  port_sel(2) <= '1' when (main_req_i.addr(31 downto index_size_f(XIP_SIZE))  = XIP_BASE( 31 downto index_size_f(XIP_SIZE)))  and (XIP_ENABLE  = true) else '0';
  port_sel(3) <= '1' when (main_req_i.addr(31 downto index_size_f(BOOT_SIZE)) = BOOT_BASE(31 downto index_size_f(BOOT_SIZE))) and (BOOT_ENABLE = true) else '0';
  port_sel(4) <= '1' when (main_req_i.addr(31 downto index_size_f(IO_SIZE))   = IO_BASE(  31 downto index_size_f(IO_SIZE)))   and (IO_ENABLE   = true) else '0';

  -- accesses to the "void" are redirected to the external bus interface --
  port_sel(5) <= '1' when ((port_sel(4 downto 0) = "00000") and (EXT_ENABLE = true)) else '0';


  -- Gateway Ports --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  imem_req_o <= port_req(0); port_rsp(0) <= imem_rsp_i;
  dmem_req_o <= port_req(1); port_rsp(1) <= dmem_rsp_i;
  xip_req_o  <= port_req(2); port_rsp(2) <= xip_rsp_i;
  boot_req_o <= port_req(3); port_rsp(3) <= boot_rsp_i;
  io_req_o   <= port_req(4); port_rsp(4) <= io_rsp_i;
  ext_req_o  <= port_req(5); port_rsp(5) <= ext_rsp_i;

  -- bus request --
  request: process(main_req_i, port_sel)
  begin
    for i in 0 to 5 loop 
      port_req(i) <= req_terminate_c;
      if (port_en_list_c(i) = true) then
        port_req(i)     <= main_req_i;
        port_req(i).stb <= main_req_i.stb and port_sel(i);
      end if;
    end loop;
  end process request;

  -- bus response --
  response: process(port_rsp)
    variable tmp_v : bus_rsp_t;
  begin
    tmp_v := rsp_terminate_c; -- start with all-zero
    for i in 0 to 5 loop -- logical OR of all response signals
      if (port_en_list_c(i) = true) then
        tmp_v.data := tmp_v.data or port_rsp(i).data;
        tmp_v.ack  := tmp_v.ack  or port_rsp(i).ack;
        tmp_v.err  := tmp_v.err  or port_rsp(i).err;
      end if;
    end loop;
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
      keeper.err  <= '0'; -- default
      keeper.halt <= port_sel(2) or port_sel(5); -- no timeout if XIP or EXTERNAL access
      if (keeper.busy = '0') then -- bus idle
        keeper.cnt  <= std_ulogic_vector(to_unsigned(TIMEOUT, keeper.cnt'length));
        keeper.busy <= main_req_i.stb;
      else -- bus access in progress
        keeper.cnt <= std_ulogic_vector(unsigned(keeper.cnt) - 1);
        if (int_rsp.err = '1') or ((or_reduce_f(keeper.cnt) = '0') and (keeper.halt = '0')) then -- bus error or timeout
          keeper.err  <= '1';
          keeper.busy <= '0';
        elsif (int_rsp.ack = '1') then -- normal access termination
          keeper.busy <= '0';
        end if;
      end if;
    end if;
  end process bus_monitor;


end neorv32_bus_gateway_rtl;


-- ############################################################################################################################
-- ############################################################################################################################


-- #################################################################################################
-- # << NEORV32 - Processor Bus Infrastructure: IO Switch >>                                       #
-- # ********************************************************************************************* #
-- # Simple address decoding switch for the processor's internal IO/peripheral devices.            #
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

entity neorv32_bus_io_switch is
  generic (
    DEV_SIZE  : natural; -- size of a single IO device, has to be a power of two
    -- device port enable and base address --
    DEV_00_EN : boolean; DEV_00_BASE : std_ulogic_vector(31 downto 0);
    DEV_01_EN : boolean; DEV_01_BASE : std_ulogic_vector(31 downto 0);
    DEV_02_EN : boolean; DEV_02_BASE : std_ulogic_vector(31 downto 0);
    DEV_03_EN : boolean; DEV_03_BASE : std_ulogic_vector(31 downto 0);
    DEV_04_EN : boolean; DEV_04_BASE : std_ulogic_vector(31 downto 0);
    DEV_05_EN : boolean; DEV_05_BASE : std_ulogic_vector(31 downto 0);
    DEV_06_EN : boolean; DEV_06_BASE : std_ulogic_vector(31 downto 0);
    DEV_07_EN : boolean; DEV_07_BASE : std_ulogic_vector(31 downto 0);
    DEV_08_EN : boolean; DEV_08_BASE : std_ulogic_vector(31 downto 0);
    DEV_09_EN : boolean; DEV_09_BASE : std_ulogic_vector(31 downto 0);
    DEV_10_EN : boolean; DEV_10_BASE : std_ulogic_vector(31 downto 0);
    DEV_11_EN : boolean; DEV_11_BASE : std_ulogic_vector(31 downto 0);
    DEV_12_EN : boolean; DEV_12_BASE : std_ulogic_vector(31 downto 0);
    DEV_13_EN : boolean; DEV_13_BASE : std_ulogic_vector(31 downto 0);
    DEV_14_EN : boolean; DEV_14_BASE : std_ulogic_vector(31 downto 0);
    DEV_15_EN : boolean; DEV_15_BASE : std_ulogic_vector(31 downto 0);
    DEV_16_EN : boolean; DEV_16_BASE : std_ulogic_vector(31 downto 0);
    DEV_17_EN : boolean; DEV_17_BASE : std_ulogic_vector(31 downto 0);
    DEV_18_EN : boolean; DEV_18_BASE : std_ulogic_vector(31 downto 0);
    DEV_19_EN : boolean; DEV_19_BASE : std_ulogic_vector(31 downto 0);
    DEV_20_EN : boolean; DEV_20_BASE : std_ulogic_vector(31 downto 0)
  );
  port (
    -- host port --
    main_req_i   : in  bus_req_t; -- host request
    main_rsp_o   : out bus_rsp_t; -- host response
    -- device ports --
    dev_00_req_o : out bus_req_t; dev_00_rsp_i : in bus_rsp_t;
    dev_01_req_o : out bus_req_t; dev_01_rsp_i : in bus_rsp_t;
    dev_02_req_o : out bus_req_t; dev_02_rsp_i : in bus_rsp_t;
    dev_03_req_o : out bus_req_t; dev_03_rsp_i : in bus_rsp_t;
    dev_04_req_o : out bus_req_t; dev_04_rsp_i : in bus_rsp_t;
    dev_05_req_o : out bus_req_t; dev_05_rsp_i : in bus_rsp_t;
    dev_06_req_o : out bus_req_t; dev_06_rsp_i : in bus_rsp_t;
    dev_07_req_o : out bus_req_t; dev_07_rsp_i : in bus_rsp_t;
    dev_08_req_o : out bus_req_t; dev_08_rsp_i : in bus_rsp_t;
    dev_09_req_o : out bus_req_t; dev_09_rsp_i : in bus_rsp_t;
    dev_10_req_o : out bus_req_t; dev_10_rsp_i : in bus_rsp_t;
    dev_11_req_o : out bus_req_t; dev_11_rsp_i : in bus_rsp_t;
    dev_12_req_o : out bus_req_t; dev_12_rsp_i : in bus_rsp_t;
    dev_13_req_o : out bus_req_t; dev_13_rsp_i : in bus_rsp_t;
    dev_14_req_o : out bus_req_t; dev_14_rsp_i : in bus_rsp_t;
    dev_15_req_o : out bus_req_t; dev_15_rsp_i : in bus_rsp_t;
    dev_16_req_o : out bus_req_t; dev_16_rsp_i : in bus_rsp_t;
    dev_17_req_o : out bus_req_t; dev_17_rsp_i : in bus_rsp_t;
    dev_18_req_o : out bus_req_t; dev_18_rsp_i : in bus_rsp_t;
    dev_19_req_o : out bus_req_t; dev_19_rsp_i : in bus_rsp_t;
    dev_20_req_o : out bus_req_t; dev_20_rsp_i : in bus_rsp_t
  );
end neorv32_bus_io_switch;

architecture neorv32_bus_io_switch_rtl of neorv32_bus_io_switch is

  -- ------------------------------------------------------------------------------------------- --
  -- How to add another device port                                                              --
  -- ------------------------------------------------------------------------------------------- --
  -- 1. Increment <num_devs_physical_c> (must not exceed <num_devs_logical_c>).                  --
  -- 2. Append another pair of "DEV_xx_EN" and "DEV_xx_BASE" generics.                           --
  -- 3. Append these two generics to the according <dev_en_list_c> and <dev_base_list_c> arrays. --
  -- 4. Append another pair of "dev_xx_req_o" and "dev_xx_rsp_i" ports.                          --
  -- 5. Append these two ports to the according <dev_req> and <dev_rsp> array assignments in     --
  --    the "Combine Device Ports" section.                                                      --
  -- ------------------------------------------------------------------------------------------- --

  -- module configuration --
  constant num_devs_physical_c : natural := 21; -- actual number of devices, max num_devs_logical_c
  constant num_devs_logical_c  : natural := 32; -- logical max number of devices; do not change!

  -- address bits for access decoding --
  constant abb_lo_c : natural := index_size_f(DEV_SIZE); -- low address boundary bit
  constant abb_hi_c : natural := (index_size_f(DEV_SIZE) + index_size_f(num_devs_logical_c)) - 1; -- high address boundary bit

  -- list of enabled device ports --
  type dev_en_list_t is array (0 to num_devs_physical_c-1) of boolean;
  constant dev_en_list_c : dev_en_list_t := (
    DEV_00_EN, DEV_01_EN, DEV_02_EN, DEV_03_EN,
    DEV_04_EN, DEV_05_EN, DEV_06_EN, DEV_07_EN,
    DEV_08_EN, DEV_09_EN, DEV_10_EN, DEV_11_EN,
    DEV_12_EN, DEV_13_EN, DEV_14_EN, DEV_15_EN,
    DEV_16_EN, DEV_17_EN, DEV_18_EN, DEV_19_EN,
    DEV_20_EN
  );

  -- list of device base addresses --
  type dev_base_list_t is array (0 to num_devs_physical_c-1) of std_ulogic_vector(31 downto 0);
  constant dev_base_list_c : dev_base_list_t := (
    DEV_00_BASE, DEV_01_BASE, DEV_02_BASE, DEV_03_BASE,
    DEV_04_BASE, DEV_05_BASE, DEV_06_BASE, DEV_07_BASE,
    DEV_08_BASE, DEV_09_BASE, DEV_10_BASE, DEV_11_BASE,
    DEV_12_BASE, DEV_13_BASE, DEV_14_BASE, DEV_15_BASE,
    DEV_16_BASE, DEV_17_BASE, DEV_18_BASE, DEV_19_BASE,
    DEV_20_BASE
  );

  -- device ports combined as arrays --
  type dev_req_t is array (0 to num_devs_physical_c-1) of bus_req_t;
  type dev_rsp_t is array (0 to num_devs_physical_c-1) of bus_rsp_t;
  signal dev_req : dev_req_t;
  signal dev_rsp : dev_rsp_t;

begin

  -- Combine Device Ports -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  dev_00_req_o <= dev_req(00); dev_rsp(00) <= dev_00_rsp_i;
  dev_01_req_o <= dev_req(01); dev_rsp(01) <= dev_01_rsp_i;
  dev_02_req_o <= dev_req(02); dev_rsp(02) <= dev_02_rsp_i;
  dev_03_req_o <= dev_req(03); dev_rsp(03) <= dev_03_rsp_i;
  dev_04_req_o <= dev_req(04); dev_rsp(04) <= dev_04_rsp_i;
  dev_05_req_o <= dev_req(05); dev_rsp(05) <= dev_05_rsp_i;
  dev_06_req_o <= dev_req(06); dev_rsp(06) <= dev_06_rsp_i;
  dev_07_req_o <= dev_req(07); dev_rsp(07) <= dev_07_rsp_i;
  dev_08_req_o <= dev_req(08); dev_rsp(08) <= dev_08_rsp_i;
  dev_09_req_o <= dev_req(09); dev_rsp(09) <= dev_09_rsp_i;
  dev_10_req_o <= dev_req(10); dev_rsp(10) <= dev_10_rsp_i;
  dev_11_req_o <= dev_req(11); dev_rsp(11) <= dev_11_rsp_i;
  dev_12_req_o <= dev_req(12); dev_rsp(12) <= dev_12_rsp_i;
  dev_13_req_o <= dev_req(13); dev_rsp(13) <= dev_13_rsp_i;
  dev_14_req_o <= dev_req(14); dev_rsp(14) <= dev_14_rsp_i;
  dev_15_req_o <= dev_req(15); dev_rsp(15) <= dev_15_rsp_i;
  dev_16_req_o <= dev_req(16); dev_rsp(16) <= dev_16_rsp_i;
  dev_17_req_o <= dev_req(17); dev_rsp(17) <= dev_17_rsp_i;
  dev_18_req_o <= dev_req(18); dev_rsp(18) <= dev_18_rsp_i;
  dev_19_req_o <= dev_req(19); dev_rsp(19) <= dev_19_rsp_i;
  dev_20_req_o <= dev_req(20); dev_rsp(20) <= dev_20_rsp_i;


  -- Device Requests ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  access_gen:
  for i in 0 to (num_devs_physical_c-1) generate

    access_gen_enabled:
    if (dev_en_list_c(i) = true) generate
      req_gen: process(main_req_i)
      begin
        dev_req(i) <= main_req_i;
        if (main_req_i.addr(abb_hi_c downto abb_lo_c) = dev_base_list_c(i)(abb_hi_c downto abb_lo_c)) then
          dev_req(i).stb <= main_req_i.stb;
        else
          dev_req(i).stb <= '0';
        end if;
      end process req_gen;
    end generate;

    access_gen_disabled:
    if (dev_en_list_c(i) = false) generate
      dev_req(i) <= req_terminate_c;
    end generate;

  end generate;


  -- Global Response ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_response: process(dev_rsp)
    variable tmp_v : bus_rsp_t;
  begin
    tmp_v := rsp_terminate_c; -- start with all-zero
    for i in 0 to (num_devs_physical_c-1) loop -- logical OR of all response signals
      if (dev_en_list_c(i) = true) then
        tmp_v.data := tmp_v.data or dev_rsp(i).data;
        tmp_v.ack  := tmp_v.ack  or dev_rsp(i).ack;
        tmp_v.err  := tmp_v.err  or dev_rsp(i).err;
      end if;
    end loop;
    main_rsp_o <= tmp_v;
  end process;


end neorv32_bus_io_switch_rtl;


-- ############################################################################################################################
-- ############################################################################################################################


-- #################################################################################################
-- # << NEORV32 - Processor Bus Infrastructure: Reservation Set Control >>                         #
-- # ********************************************************************************************* #
-- # Reservation set controller for the A (atomic) ISA extension's LR.W (load-reservate) and SC.W  #
-- # (store-conditional) instructions. Only a single reservation set is supported.                 #
-- # The reservation set's granularity can be configured via the GRANULARITY generic.              #
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

entity neorv32_bus_reservation_set is
  generic (
    GRANULARITY : natural -- reservation set granularity in bytes; has to be power of 2, min 4
  );
  port (
    -- global control --
    clk_i       : in  std_ulogic; -- global clock, rising edge
    rstn_i      : in  std_ulogic; -- global reset, low-active, async
    -- external status and control --
    rvs_addr_o  : out std_ulogic_vector(31 downto 0);
    rvs_valid_o : out std_ulogic;
    rvs_clear_i : in  std_ulogic;
    -- core/cpu port --
    core_req_i  : in  bus_req_t;
    core_rsp_o  : out bus_rsp_t;
    -- system ports --
    sys_req_o   : out bus_req_t;
    sys_rsp_i   : in  bus_rsp_t
  );
end neorv32_bus_reservation_set;

architecture neorv32_bus_reservation_set_rtl of neorv32_bus_reservation_set is

  -- reservation set granularity address boundary bit --
  constant abb_c : natural := index_size_f(GRANULARITY);

  -- reservation set --
  type rsvs_t is record
    state : std_ulogic_vector(01 downto 0);
    addr  : std_ulogic_vector(31 downto abb_c);
    valid : std_ulogic;
    match : std_ulogic;
  end record;
  signal rsvs : rsvs_t;

  -- ACK override for failed SC.W --
  signal ack_local : std_ulogic;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (is_power_of_two_f(GRANULARITY) = false) report
    "NEORV32 PROCESSOR CONFIG ERROR: Reservation set granularity has to be a power of 2." severity error;
  assert not (GRANULARITY < 4) report
    "NEORV32 PROCESSOR CONFIG ERROR: Reservation set granularity has to be at least 4 bytes wide." severity error;


  -- Reservation Set Control ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rvs_control: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      rsvs.state <= "00";
      rsvs.addr  <= (others => '0');
    elsif rising_edge(clk_i) then
      case rsvs.state is

        when "10" => -- active reservation: wait for condition to invalidate reservation
        -- --------------------------------------------------------------------
          if (core_req_i.stb = '1') and (core_req_i.rw = '0') and (core_req_i.rvso = '1') then -- another LR instruction overriding the current reservation
            rsvs.addr <= core_req_i.addr(31 downto abb_c);
          end if;
          --
          if (rvs_clear_i = '1') then -- external clear request (highest priority)
            rsvs.state <= "00"; -- invalidate reservation
          elsif (core_req_i.stb = '1') and (core_req_i.rw = '1') then -- write access

            if (core_req_i.rvso = '1') then -- this is a SC operation
              if (rsvs.match = '1') then -- SC to reservated address
                rsvs.state <= "11"; -- execute SC instruction (reservation still valid)
              else -- SC to any other address
                rsvs.state <= "00"; -- invalidate reservation
              end if;

            elsif (rsvs.match = '1') then -- normal write to reservated address
              rsvs.state <= "00"; -- invalidate reservation
            end if;

          end if;

        when "11" => -- active reservation: invalidate reservation at the end of bus access
        -- --------------------------------------------------------------------
          if (sys_rsp_i.ack = '1') or (sys_rsp_i.err = '1') then
            rsvs.state <= "00";
          end if;

        when others => -- "0-" no active reservation: wait for new registration request
        -- --------------------------------------------------------------------
          if (core_req_i.stb = '1') and (core_req_i.rw = '0') and (core_req_i.rvso = '1') then -- load-reservate instruction
            rsvs.addr  <= core_req_i.addr(31 downto abb_c);
            rsvs.state <= "10";
          end if;

      end case;
    end if;
  end process rvs_control;

  -- address match? --
  rsvs.match <= '1' when (core_req_i.addr(31 downto abb_c) = rsvs.addr) else '0';

  -- reservation valid? --
  rsvs.valid <= rsvs.state(1);

  -- status for external system --
  rvs_valid_o                  <= rsvs.valid;
  rvs_addr_o(31 downto abb_c)  <= rsvs.addr;
  rvs_addr_o(abb_c-1 downto 0) <= (others => '0');


  -- System Bus Interface -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- gated request --
  bus_request: process(core_req_i, rsvs.valid)
  begin
    sys_req_o <= core_req_i;
    if (core_req_i.rvso = '1') and (core_req_i.rw = '1') then -- SC operation
      sys_req_o.stb <= core_req_i.stb and rsvs.valid; -- write allowed if reservation still valid
    else -- normal memory request or LR
      sys_req_o.stb <= core_req_i.stb;
    end if;
  end process bus_request;

  -- if a SC.W instruction fails there will be no write-request being send to the bus system
  -- so we need to provide a local ACK to complete the bus access
  ack_override: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ack_local <= '0';
    elsif rising_edge(clk_i) then
      ack_local <= core_req_i.rvso and core_req_i.stb and core_req_i.rw and (not rsvs.valid);
    end if;
  end process ack_override;

  -- response --
  core_rsp_o.err <= sys_rsp_i.err;
  core_rsp_o.ack <= sys_rsp_i.ack or ack_local; -- generate local ACK if SC fails
  -- inject 1 into read data's LSB if SC fails --
  core_rsp_o.data(31 downto 1) <= sys_rsp_i.data(31 downto 1);
  core_rsp_o.data(0) <= sys_rsp_i.data(0) or (core_req_i.rvso and core_req_i.rw and (not rsvs.valid));


end neorv32_bus_reservation_set_rtl;
