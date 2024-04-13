-- ================================================================================ --
-- NEORV32 SoC - Processor Bus Infrastructure: Prioritizing 2-to-1 Bus Switch       --
-- -------------------------------------------------------------------------------- --
-- Allows to access a single device bus X by two controller ports A and B.          --
-- Controller port A has priority over controller port B.                           --
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

entity neorv32_bus_switch is
  generic (
    PORT_A_READ_ONLY : boolean; -- set if port A is read-only
    PORT_B_READ_ONLY : boolean  -- set if port B is read-only
  );
  port (
    clk_i   : in  std_ulogic; -- global clock, rising edge
    rstn_i  : in  std_ulogic; -- global reset, low-active, async
    a_req_i : in  bus_req_t;  -- host port A: request bus (PRIORITIZED)
    a_rsp_o : out bus_rsp_t;  -- host port A: response bus
    b_req_i : in  bus_req_t;  -- host port B: request bus
    b_rsp_o : out bus_rsp_t;  -- host port B: response bus
    x_req_o : out bus_req_t;  -- device port request bus
    x_rsp_i : in  bus_rsp_t   -- device port response bus
  );
end neorv32_bus_switch;

architecture neorv32_bus_switch_rtl of neorv32_bus_switch is

  -- access arbiter --
  type arbiter_t is record
    state, state_nxt : std_ulogic_vector(1 downto 0);
    a_req, b_req     : std_ulogic;
    sel,   stb       : std_ulogic;
  end record;
  signal arbiter : arbiter_t;

  -- FSM states --
  constant IDLE   : std_ulogic_vector(1 downto 0) := "00";
  constant BUSY_A : std_ulogic_vector(1 downto 0) := "01";
  constant BUSY_B : std_ulogic_vector(1 downto 0) := "10";

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
      arbiter.a_req <= (arbiter.a_req or a_req_i.stb) and (not arbiter.state(0)); -- clear STB buffer in BUSY_A
      arbiter.b_req <= (arbiter.b_req or b_req_i.stb) and (not arbiter.state(1)); -- clear STB buffer in BUSY_B
    end if;
  end process arbiter_sync;

  -- fsm --
  arbiter_comb: process(arbiter, a_req_i, b_req_i, x_rsp_i)
  begin
    -- defaults --
    arbiter.state_nxt <= arbiter.state;
    arbiter.sel       <= '0';
    arbiter.stb       <= '0';

    -- state machine --
    case arbiter.state is

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

      when others => -- IDLE: wait for requests
      -- ------------------------------------------------------------
        if (a_req_i.stb = '1') or (arbiter.a_req = '1') then -- request from port A (prioritized)?
          arbiter.sel       <= '0';
          arbiter.stb       <= '1';
          arbiter.state_nxt <= BUSY_A;
        elsif (b_req_i.stb = '1') or (arbiter.b_req = '1') then -- request from port B?
          arbiter.sel       <= '1';
          arbiter.stb       <= '1';
          arbiter.state_nxt <= BUSY_B;
        end if;

    end case;
  end process arbiter_comb;


  -- Request Switch -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  x_req_o.addr  <= a_req_i.addr when (arbiter.sel = '0') else b_req_i.addr;
  x_req_o.rvso  <= a_req_i.rvso when (arbiter.sel = '0') else b_req_i.rvso;
  x_req_o.priv  <= a_req_i.priv when (arbiter.sel = '0') else b_req_i.priv;
  x_req_o.src   <= a_req_i.src  when (arbiter.sel = '0') else b_req_i.src;
  x_req_o.rw    <= a_req_i.rw   when (arbiter.sel = '0') else b_req_i.rw;
  x_req_o.fence <= a_req_i.fence or b_req_i.fence; -- propagate any fence operations

  x_req_o.data  <= b_req_i.data when PORT_A_READ_ONLY    else
                   a_req_i.data when PORT_B_READ_ONLY    else
                   a_req_i.data when (arbiter.sel = '0') else b_req_i.data;

  x_req_o.ben   <= b_req_i.ben when PORT_A_READ_ONLY     else
                   a_req_i.ben when PORT_B_READ_ONLY     else
                   a_req_i.ben when (arbiter.sel = '0')  else b_req_i.ben;

  x_req_o.stb   <= arbiter.stb;


  -- Response Switch ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  a_rsp_o.data <= x_rsp_i.data;
  a_rsp_o.ack  <= x_rsp_i.ack when (arbiter.sel = '0') else '0';
  a_rsp_o.err  <= x_rsp_i.err when (arbiter.sel = '0') else '0';

  b_rsp_o.data <= x_rsp_i.data;
  b_rsp_o.ack  <= x_rsp_i.ack when (arbiter.sel = '1') else '0';
  b_rsp_o.err  <= x_rsp_i.err when (arbiter.sel = '1') else '0';


end neorv32_bus_switch_rtl;


-- ############################################################################################################################
-- ############################################################################################################################


-- ================================================================================ --
-- NEORV32 SoC - Processor Bus Infrastructure: Section Gateway                      --
-- -------------------------------------------------------------------------------- --
-- Bus gateway to distribute accesses to 5 non-overlapping address sub-spaces       --
-- (A..E). All accesses that do not match any of these sections are redirected to   --
-- the "X" port. The gateway-internal bus monitor ensures that all accesses are     --
-- completed within a bound time window (if *_TMO_EN is true). Otherwise, a bus     --
-- error is triggered.                                                              --
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

entity neorv32_bus_gateway is
  generic (
    TIMEOUT  : natural; -- internal bus timeout cycles
    -- port A --
    A_ENABLE : boolean; -- port enable
    A_BASE   : std_ulogic_vector(31 downto 0); -- port address space base address
    A_SIZE   : natural; -- port address space size in bytes (power of two!)
    A_TMO_EN : boolean; -- port timeout enable
    -- port B --
    B_ENABLE : boolean;
    B_BASE   : std_ulogic_vector(31 downto 0);
    B_SIZE   : natural;
    B_TMO_EN : boolean;
    -- port C --
    C_ENABLE : boolean;
    C_BASE   : std_ulogic_vector(31 downto 0);
    C_SIZE   : natural;
    C_TMO_EN : boolean;
    -- port D --
    D_ENABLE : boolean;
    D_BASE   : std_ulogic_vector(31 downto 0);
    D_SIZE   : natural;
    D_TMO_EN : boolean;
    -- port E --
    E_ENABLE : boolean;
    E_BASE   : std_ulogic_vector(31 downto 0);
    E_SIZE   : natural;
    E_TMO_EN : boolean;
    -- port X --
    X_ENABLE : boolean;
    X_TMO_EN : boolean
  );
  port (
    -- global control --
    clk_i   : in  std_ulogic; -- global clock, rising edge
    rstn_i  : in  std_ulogic; -- global reset, low-active, async
    -- host port --
    req_i   : in  bus_req_t;  -- host request
    rsp_o   : out bus_rsp_t;  -- host response
    -- section ports --
    a_req_o : out bus_req_t;
    a_rsp_i : in  bus_rsp_t;
    b_req_o : out bus_req_t;
    b_rsp_i : in  bus_rsp_t;
    c_req_o : out bus_req_t;
    c_rsp_i : in  bus_rsp_t;
    d_req_o : out bus_req_t;
    d_rsp_i : in  bus_rsp_t;
    e_req_o : out bus_req_t;
    e_rsp_i : in  bus_rsp_t;
    x_req_o : out bus_req_t;
    x_rsp_i : in  bus_rsp_t
  );
end neorv32_bus_gateway;

architecture neorv32_bus_gateway_rtl of neorv32_bus_gateway is

  -- port select --
  signal port_sel : std_ulogic_vector(5 downto 0);

  -- port enable list --
  type port_en_list_t is array (0 to 5) of boolean;
  constant port_en_list_c : port_en_list_t := (A_ENABLE, B_ENABLE, C_ENABLE, D_ENABLE, E_ENABLE, X_ENABLE);

  -- port timeout enable list --
  constant tmo_en_list_c : std_ulogic_vector(5 downto 0) := (
    bool_to_ulogic_f(X_TMO_EN),
    bool_to_ulogic_f(E_TMO_EN),
    bool_to_ulogic_f(D_TMO_EN),
    bool_to_ulogic_f(C_TMO_EN),
    bool_to_ulogic_f(B_TMO_EN),
    bool_to_ulogic_f(A_TMO_EN)
  );

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
  port_sel(0) <= '1' when (req_i.addr(31 downto index_size_f(A_SIZE)) = A_BASE(31 downto index_size_f(A_SIZE))) and A_ENABLE else '0';
  port_sel(1) <= '1' when (req_i.addr(31 downto index_size_f(B_SIZE)) = B_BASE(31 downto index_size_f(B_SIZE))) and B_ENABLE else '0';
  port_sel(2) <= '1' when (req_i.addr(31 downto index_size_f(C_SIZE)) = C_BASE(31 downto index_size_f(C_SIZE))) and C_ENABLE else '0';
  port_sel(3) <= '1' when (req_i.addr(31 downto index_size_f(D_SIZE)) = D_BASE(31 downto index_size_f(D_SIZE))) and D_ENABLE else '0';
  port_sel(4) <= '1' when (req_i.addr(31 downto index_size_f(E_SIZE)) = E_BASE(31 downto index_size_f(E_SIZE))) and E_ENABLE else '0';

  -- accesses to the "void" are redirected to the X port --
  port_sel(5) <= '1' when ((port_sel(4 downto 0) = "00000") and X_ENABLE) else '0';


  -- Gateway Ports --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  a_req_o <= port_req(0); port_rsp(0) <= a_rsp_i;
  b_req_o <= port_req(1); port_rsp(1) <= b_rsp_i;
  c_req_o <= port_req(2); port_rsp(2) <= c_rsp_i;
  d_req_o <= port_req(3); port_rsp(3) <= d_rsp_i;
  e_req_o <= port_req(4); port_rsp(4) <= e_rsp_i;
  x_req_o <= port_req(5); port_rsp(5) <= x_rsp_i;

  -- bus request --
  request: process(req_i, port_sel)
  begin
    for i in 0 to 5 loop
      port_req(i) <= req_terminate_c;
      if port_en_list_c(i) then
        port_req(i)     <= req_i;
        port_req(i).stb <= req_i.stb and port_sel(i);
      end if;
    end loop;
  end process request;

  -- bus response --
  response: process(port_rsp)
    variable tmp_v : bus_rsp_t;
  begin
    tmp_v := rsp_terminate_c; -- start with all-zero
    for i in 0 to 5 loop -- OR all response signals
      if port_en_list_c(i) then
        tmp_v.data := tmp_v.data or port_rsp(i).data;
        tmp_v.ack  := tmp_v.ack  or port_rsp(i).ack;
        tmp_v.err  := tmp_v.err  or port_rsp(i).err;
      end if;
    end loop;
    int_rsp <= tmp_v;
  end process response;

  -- host response --
  rsp_o.data <= int_rsp.data;
  rsp_o.ack  <= int_rsp.ack;
  rsp_o.err  <= keeper.err;


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
      keeper.halt <= or_reduce_f(port_sel and (not tmo_en_list_c)); -- no timeout if *_TMO_EN = true
      if (keeper.busy = '0') then -- bus idle
        keeper.cnt  <= std_ulogic_vector(to_unsigned(TIMEOUT, keeper.cnt'length));
        keeper.busy <= req_i.stb;
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


-- ================================================================================ --
-- NEORV32 SoC - Processor Bus Infrastructure: IO Switch                            --
-- -------------------------------------------------------------------------------- --
-- Simple switch for accessing one out of several (IO) devices.                     --
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


  -- Request --------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_request_gen:
  for i in 0 to (num_devs_physical_c-1) generate

    bus_request_port_enabled:
    if dev_en_list_c(i) generate
      bus_request: process(main_req_i)
      begin
        dev_req(i) <= main_req_i;
        if (main_req_i.addr(abb_hi_c downto abb_lo_c) = dev_base_list_c(i)(abb_hi_c downto abb_lo_c)) then
          dev_req(i).stb <= main_req_i.stb;
        else
          dev_req(i).stb <= '0';
        end if;
      end process bus_request;
    end generate;

    bus_request_port_disabled:
    if not dev_en_list_c(i) generate
      dev_req(i) <= req_terminate_c;
    end generate;

  end generate; -- /bus_request_gen


  -- Response -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_response: process(dev_rsp)
    variable tmp_v : bus_rsp_t;
  begin
    tmp_v := rsp_terminate_c; -- start with all-zero
    for i in 0 to (num_devs_physical_c-1) loop -- logical OR all response signals
      if dev_en_list_c(i) then
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


-- ================================================================================ --
-- NEORV32 SoC - PProcessor Bus Infrastructure: Reservation Set Control             --
-- -------------------------------------------------------------------------------- --
-- Reservation set controller for the A (atomic) ISA extension's LR.W               --
-- (load-reservate) and SC.W (store-conditional) instructions. Only a single        --
-- reservation set is supported. The reservation set's granularity can be           --
-- configured via the GRANULARITY generic.                                          --
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

entity neorv32_bus_reservation_set is
  generic (
    GRANULARITY : natural range 4 to natural'high -- reservation set granularity in bytes; has to be power of 2, min 4
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

  -- auto-configuration --
  constant granularity_valid_c : boolean := is_power_of_two_f(GRANULARITY);
  constant granularity_c       : natural := cond_sel_natural_f(granularity_valid_c, GRANULARITY, 2**index_size_f(GRANULARITY));

  -- reservation set granularity address boundary bit --
  constant abb_c : natural := index_size_f(granularity_c);

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
  assert not (granularity_valid_c = false) report
    "[NEORV32] Auto-adjusting invalid reservation set granularity configuration." severity warning;


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
