-- #################################################################################################
-- # << NEORV32 - Execute In Place (XIP) Module >>                                                 #
-- # ********************************************************************************************* #
-- # This module allows the CPU to execute code (and read constant data) directly from an SPI      #
-- # flash memory. Two host ports are implemented: one  for accessing the control and status       #
-- # registers (mapped to the processor's IO space) and one for the actual instruction/data fetch. #
-- # The actual address space mapping of the "instruction/data interface" is done by programming   #
-- # special control register bits.                                                                #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
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

entity neorv32_xip is
  port (
    -- global control --
    clk_i       : in  std_ulogic; -- global clock line
    rstn_i      : in  std_ulogic; -- global reset line, low-active
    -- host access: control register access port --
    ct_addr_i   : in  std_ulogic_vector(31 downto 0); -- address
    ct_rden_i   : in  std_ulogic; -- read enable
    ct_wren_i   : in  std_ulogic; -- write enable
    ct_data_i   : in  std_ulogic_vector(31 downto 0); -- data in
    ct_data_o   : out std_ulogic_vector(31 downto 0); -- data out
    ct_ack_o    : out std_ulogic; -- transfer acknowledge
    -- host access: instruction fetch access port (read-only) --
    if_addr_i   : in  std_ulogic_vector(31 downto 0); -- address
    if_rden_i   : in  std_ulogic; -- read enable
    if_data_o   : out std_ulogic_vector(31 downto 0); -- data out
    if_ack_o    : out std_ulogic; -- transfer acknowledge
    -- status --
    xip_en_o    : out std_ulogic; -- XIP enable
    xip_acc_o   : out std_ulogic; -- pending XIP access
    xip_page_o  : out std_ulogic_vector(03 downto 0); -- XIP page
    -- clock generator --
    clkgen_en_o : out std_ulogic; -- enable clock generator
    clkgen_i    : in  std_ulogic_vector(07 downto 0);
    -- SPI device interface --
    spi_csn_o   : out std_ulogic; -- chip-select, low-active
    spi_clk_o   : out std_ulogic; -- serial clock
    spi_data_i  : in  std_ulogic; -- device data output
    spi_data_o  : out std_ulogic  -- controller data output
  );
end neorv32_xip;

architecture neorv32_xip_rtl of neorv32_xip is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(xip_size_c); -- low address boundary bit

  -- CT register access control --
  signal ct_acc_en : std_ulogic; -- module access enable
  signal ct_addr   : std_ulogic_vector(31 downto 0); -- access address
  signal ct_wren   : std_ulogic; -- word write enable
  signal ct_rden   : std_ulogic; -- read enable

  -- control register --
  constant ctrl_enable_c      : natural :=  0; -- r/w: module enable
  constant ctrl_spi_prsc0_c   : natural :=  1; -- r/w: SPI clock prescaler select - bit 0
  constant ctrl_spi_prsc1_c   : natural :=  2; -- r/w: SPI clock prescaler select - bit 1
  constant ctrl_spi_prsc2_c   : natural :=  3; -- r/w: SPI clock prescaler select - bit 2
  constant ctrl_spi_cpol_c    : natural :=  4; -- r/w: SPI (idle) clock polarity
  constant ctrl_spi_cpha_c    : natural :=  5; -- r/w: SPI clock phase
  constant ctrl_spi_nbytes0_c : natural :=  6; -- r/w: SPI number of bytes in transmission (1..9) - bit 0
  constant ctrl_spi_nbytes3_c : natural :=  9; -- r/w: SPI number of bytes in transmission (1..9) - bit 3
  constant ctrl_xip_enable_c  : natural := 10; -- r/w: XIP access mode enable
  constant ctrl_xip_abytes0_c : natural := 11; -- r/w: XIP number of address bytes (0=1,1=2,2=3,3=4) - bit 0
  constant ctrl_xip_abytes1_c : natural := 12; -- r/w: XIP number of address bytes (0=1,1=2,2=3,3=4) - bit 1
  constant ctrl_rd_cmd0_c     : natural := 13; -- r/w: SPI flash read command - bit 0
  constant ctrl_rd_cmd7_c     : natural := 20; -- r/w: SPI flash read command - bit 7
  constant ctrl_page0_c       : natural := 21; -- r/w: XIP memory page - bit 0
  constant ctrl_page3_c       : natural := 24; -- r/w: XIP memory page - bit 3
  constant ctrl_spi_csen_c    : natural := 25; -- r/w: SPI chip-select enabled
  constant ctrl_highspeed_c   : natural := 26; -- r/w: SPI high-speed mode enable (ignoring ctrl_spi_prsc)
  --
  constant ctrl_phy_busy_c    : natural := 30; -- r/-: SPI PHY is busy when set
  constant ctrl_xip_busy_c    : natural := 31; -- r/-: XIP access in progress
  --
  signal ctrl : std_ulogic_vector(26 downto 0);

  -- Direct SPI access registers --
  signal spi_data_lo : std_ulogic_vector(31 downto 0);
  signal spi_data_hi : std_ulogic_vector(31 downto 0); -- write-only!
  signal spi_trigger : std_ulogic; -- trigger direct SPI operation

  -- XIP access address --
  signal xip_addr : std_ulogic_vector(31 downto 0);

  -- SPI access fetch arbiter --
  type arbiter_state_t is (S_DIRECT, S_IDLE, S_TRIG, S_BUSY);
  type arbiter_t is record
    state     : arbiter_state_t;
    state_nxt : arbiter_state_t;
    addr      : std_ulogic_vector(31 downto 0);
    busy      : std_ulogic;
  end record;
  signal arbiter : arbiter_t;

  -- SPI clock --
  signal spi_clk_en : std_ulogic;

  -- Component: SPI PHY --
  component neorv32_xip_phy
  port (
    -- global control --
    clk_i        : in  std_ulogic; -- clock
    spi_clk_en_i : in  std_ulogic; -- pre-scaled SPI clock-enable
    -- operation configuration --
    cf_enable_i  : in  std_ulogic; -- module enable (reset if low)
    cf_cpha_i    : in  std_ulogic; -- clock phase
    cf_cpol_i    : in  std_ulogic; -- clock idle polarity
    -- operation control --
    op_start_i   : in  std_ulogic; -- trigger new transmission
    op_csen_i    : in  std_ulogic; -- actually enabled device for transmission
    op_busy_o    : out std_ulogic; -- transmission in progress when set
    op_nbytes_i  : in  std_ulogic_vector(03 downto 0); -- actual number of bytes to transmit (1..9)
    op_wdata_i   : in  std_ulogic_vector(71 downto 0); -- write data
    op_rdata_o   : out std_ulogic_vector(31 downto 0); -- read data
    -- SPI interface --
    spi_csn_o    : out std_ulogic;
    spi_clk_o    : out std_ulogic;
    spi_data_i   : in  std_ulogic;
    spi_data_o   : out std_ulogic
  );
  end component;

  -- PHY interface --
  type phy_if_t is record
    start : std_ulogic; -- trigger new transmission
    busy  : std_ulogic; -- transmission in progress when set
    wdata : std_ulogic_vector(71 downto 0); -- write data
    rdata : std_ulogic_vector(31 downto 0); -- read data
  end record;
  signal phy_if : phy_if_t;

begin

  -- Access Control (IO/CTRL port) ----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ct_acc_en <= '1' when (ct_addr_i(hi_abb_c downto lo_abb_c) = xip_base_c(hi_abb_c downto lo_abb_c)) else '0';
  ct_addr   <= xip_base_c(31 downto lo_abb_c) & ct_addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
  ct_wren   <= ct_acc_en and ct_wren_i;
  ct_rden   <= ct_acc_en and ct_rden_i;


  -- Control Read/Write Access --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_rw_access : process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl                    <= (others => '-');
      ctrl(ctrl_enable_c)     <= '0'; -- required
      ctrl(ctrl_xip_enable_c) <= '0'; -- required
      spi_data_lo             <= (others => '-');
      spi_data_hi             <= (others => '-');
      spi_trigger             <= '-';
      --
      ct_data_o               <= (others => '-');
      ct_ack_o                <= '-';
    elsif rising_edge(clk_i) then
      -- access acknowledge --
      ct_ack_o <= ct_wren or ct_rden;

      -- defaults --
      spi_trigger <= '0';

      -- write access --
      if (ct_wren = '1') then -- only full-word writes!

        -- control register --
        if (ct_addr = xip_ctrl_addr_c) then
          ctrl(ctrl_enable_c)                                <= ct_data_i(ctrl_enable_c);
          ctrl(ctrl_spi_prsc2_c downto ctrl_spi_prsc0_c)     <= ct_data_i(ctrl_spi_prsc2_c downto ctrl_spi_prsc0_c);
          ctrl(ctrl_spi_cpol_c)                              <= ct_data_i(ctrl_spi_cpol_c);
          ctrl(ctrl_spi_cpha_c)                              <= ct_data_i(ctrl_spi_cpha_c);
          ctrl(ctrl_spi_nbytes3_c downto ctrl_spi_nbytes0_c) <= ct_data_i(ctrl_spi_nbytes3_c downto ctrl_spi_nbytes0_c);
          ctrl(ctrl_xip_enable_c)                            <= ct_data_i(ctrl_xip_enable_c);
          ctrl(ctrl_xip_abytes1_c downto ctrl_xip_abytes0_c) <= ct_data_i(ctrl_xip_abytes1_c downto ctrl_xip_abytes0_c);
          ctrl(ctrl_rd_cmd7_c downto ctrl_rd_cmd0_c)         <= ct_data_i(ctrl_rd_cmd7_c downto ctrl_rd_cmd0_c);
          ctrl(ctrl_page3_c downto ctrl_page0_c)             <= ct_data_i(ctrl_page3_c downto ctrl_page0_c);
          ctrl(ctrl_spi_csen_c)                              <= ct_data_i(ctrl_spi_csen_c);
          ctrl(ctrl_highspeed_c)                             <= ct_data_i(ctrl_highspeed_c);
        end if;

        -- SPI direct data access register lo --
        if (ct_addr = xip_data_lo_addr_c) then
          spi_data_lo <= ct_data_i;
        end if;

        -- SPI direct data access register hi --
        if (ct_addr = xip_data_hi_addr_c) then
          spi_data_hi <= ct_data_i;
          spi_trigger <= '1'; -- trigger direct SPI transaction
        end if;
      end if;

      -- read access --
      ct_data_o <= (others => '0');
      if (ct_rden = '1') then
        case ct_addr(3 downto 2) is
          when "00" => -- 'xip_ctrl_addr_c' - control register
            ct_data_o(ctrl_enable_c)                                <= ctrl(ctrl_enable_c);
            ct_data_o(ctrl_spi_prsc2_c downto ctrl_spi_prsc0_c)     <= ctrl(ctrl_spi_prsc2_c downto ctrl_spi_prsc0_c);
            ct_data_o(ctrl_spi_cpol_c)                              <= ctrl(ctrl_spi_cpol_c);
            ct_data_o(ctrl_spi_cpha_c)                              <= ctrl(ctrl_spi_cpha_c);
            ct_data_o(ctrl_spi_nbytes3_c downto ctrl_spi_nbytes0_c) <= ctrl(ctrl_spi_nbytes3_c downto ctrl_spi_nbytes0_c);
            ct_data_o(ctrl_xip_enable_c)                            <= ctrl(ctrl_xip_enable_c);
            ct_data_o(ctrl_xip_abytes1_c downto ctrl_xip_abytes0_c) <= ctrl(ctrl_xip_abytes1_c downto ctrl_xip_abytes0_c);
            ct_data_o(ctrl_rd_cmd7_c downto ctrl_rd_cmd0_c)         <= ctrl(ctrl_rd_cmd7_c downto ctrl_rd_cmd0_c);
            ct_data_o(ctrl_page3_c downto ctrl_page0_c)             <= ctrl(ctrl_page3_c downto ctrl_page0_c);
            ct_data_o(ctrl_spi_csen_c)                              <= ctrl(ctrl_spi_csen_c);
            ct_data_o(ctrl_highspeed_c)                             <= ctrl(ctrl_highspeed_c);
            --
            ct_data_o(ctrl_phy_busy_c) <= phy_if.busy;
            ct_data_o(ctrl_xip_busy_c) <= arbiter.busy;
          when "10" => -- 'xip_data_lo_addr_c' - SPI direct data access register lo
            ct_data_o <= phy_if.rdata;
          when others => -- unavailable (not implemented or write-only)
            ct_data_o <= (others => '0');
        end case;
      end if;
    end if;
  end process ctrl_rw_access;

  -- XIP enabled --
  xip_en_o <= ctrl(ctrl_enable_c);

  -- XIP page output --
  xip_page_o <= ctrl(ctrl_page3_c downto ctrl_page0_c);


  -- XIP Address Computation Logic ----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  xip_access_logic: process(arbiter.addr, ctrl)
    variable tmp_v : std_ulogic_vector(31 downto 0);
  begin
    tmp_v(31 downto 28) := "0000";
    tmp_v(27 downto 00) := arbiter.addr(27 downto 00);
    case ctrl(ctrl_xip_abytes1_c downto ctrl_xip_abytes0_c) is -- shift address bits to be MSB-aligned
      when "00"   => xip_addr <= tmp_v(07 downto 0) & x"000000"; -- 1 address byte
      when "01"   => xip_addr <= tmp_v(15 downto 0) & x"0000";   -- 2 address bytes
      when "10"   => xip_addr <= tmp_v(23 downto 0) & x"00";     -- 3 address bytes
      when others => xip_addr <= tmp_v(31 downto 0);             -- 4 address bytes
    end case;
  end process xip_access_logic;


  -- SPI Access Arbiter ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  arbiter_sync: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (ctrl(ctrl_enable_c) = '0') or (ctrl(ctrl_xip_enable_c) = '0') then -- sync reset
        arbiter.state <= S_DIRECT;
      else
        arbiter.state <= arbiter.state_nxt;
      end if;
      arbiter.addr <= if_addr_i; -- buffer address (reducing fan-out on CPU's address net)
    end if;
  end process arbiter_sync;


  -- FSM - combinatorial part --
  arbiter_comb: process(arbiter, ctrl, xip_addr, phy_if, if_rden_i, if_addr_i, spi_data_hi, spi_data_lo, spi_trigger)
  begin
    -- arbiter defaults --
    arbiter.state_nxt <= arbiter.state;

    -- bus interface defaults --
    if_data_o <= (others => '0');
    if_ack_o  <= '0';

    -- SPI PHY interface defaults --
    phy_if.start <= '0';
    phy_if.wdata <= ctrl(ctrl_rd_cmd7_c downto ctrl_rd_cmd0_c) & xip_addr & x"00000000"; -- MSB-aligned: CMD + address + 32-bit zero data

    -- fsm --
    case arbiter.state is

      when S_DIRECT => -- XIP access disabled: allow direct SPI access
      -- ------------------------------------------------------------
        phy_if.wdata      <= spi_data_hi & spi_data_lo & x"00"; -- MSB-aligned data
        phy_if.start      <= spi_trigger;
        arbiter.state_nxt <= S_IDLE;

      when S_IDLE => -- XIP: wait for new bus request
      -- ------------------------------------------------------------
        if (if_rden_i = '1') and (if_addr_i(31 downto 28) = ctrl(ctrl_page3_c downto ctrl_page0_c)) then
          arbiter.state_nxt <= S_TRIG;
        end if;

      when S_TRIG => -- XIP: trigger flash read
      -- ------------------------------------------------------------
        phy_if.start      <= '1';
        arbiter.state_nxt <= S_BUSY;

      when S_BUSY => -- XIP: wait for PHY to complete operation
      -- ------------------------------------------------------------
        if (phy_if.busy = '0') then
          if_data_o         <= phy_if.rdata;
          if_ack_o          <= '1';
          arbiter.state_nxt <= S_IDLE;
        end if;

      when others => -- undefined
      -- ------------------------------------------------------------
        arbiter.state_nxt <= S_IDLE;

    end case;
  end process arbiter_comb;

  -- arbiter status --
  arbiter.busy <= '1' when (arbiter.state = S_TRIG) or (arbiter.state = S_BUSY) else '0'; -- actual XIP access in progress

  -- status output --
  xip_acc_o <= arbiter.busy;


  -- SPI Clock Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- enable clock generator --
  clkgen_en_o <= ctrl(ctrl_enable_c);

  -- clock select --
  spi_clk_en <= clkgen_i(to_integer(unsigned(ctrl(ctrl_spi_prsc2_c downto ctrl_spi_prsc0_c)))) or ctrl(ctrl_highspeed_c);


  -- SPI Physical Interface -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_xip_phy_inst: neorv32_xip_phy
  port map (
    -- global control --
    clk_i        => clk_i,
    spi_clk_en_i => spi_clk_en,
    -- operation configuration --
    cf_enable_i  => ctrl(ctrl_enable_c),   -- module enable (reset if low)
    cf_cpha_i    => ctrl(ctrl_spi_cpha_c), -- clock phase
    cf_cpol_i    => ctrl(ctrl_spi_cpol_c), -- clock idle polarity
    -- operation control --
    op_start_i   => phy_if.start,          -- trigger new transmission
    op_csen_i    => ctrl(ctrl_spi_csen_c), -- actually enabled device for transmission
    op_busy_o    => phy_if.busy,           -- transmission in progress when set
    op_nbytes_i  => ctrl(ctrl_spi_nbytes3_c downto ctrl_spi_nbytes0_c), -- actual number of bytes to transmit
    op_wdata_i   => phy_if.wdata,          -- write data
    op_rdata_o   => phy_if.rdata,          -- read data
    -- SPI interface --
    spi_csn_o    => spi_csn_o,
    spi_clk_o    => spi_clk_o,
    spi_data_i   => spi_data_i,
    spi_data_o   => spi_data_o
  );


end neorv32_xip_rtl;


-- ############################################################################################################################
-- ############################################################################################################################


-- #################################################################################################
-- # << NEORV32 - XIP Module - SPI Physical Interface >>                                           #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
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

entity neorv32_xip_phy is
  port (
    -- global control --
    clk_i        : in  std_ulogic; -- clock
    spi_clk_en_i : in  std_ulogic; -- pre-scaled SPI clock-enable
    -- operation configuration --
    cf_enable_i  : in  std_ulogic; -- module enable (reset if low)
    cf_cpha_i    : in  std_ulogic; -- clock phase
    cf_cpol_i    : in  std_ulogic; -- clock idle polarity
    -- operation control --
    op_start_i   : in  std_ulogic; -- trigger new transmission
    op_csen_i    : in  std_ulogic; -- actually enabled device for transmission
    op_busy_o    : out std_ulogic; -- transmission in progress when set
    op_nbytes_i  : in  std_ulogic_vector(03 downto 0); -- actual number of bytes to transmit (1..9)
    op_wdata_i   : in  std_ulogic_vector(71 downto 0); -- write data
    op_rdata_o   : out std_ulogic_vector(31 downto 0); -- read data
    -- SPI interface --
    spi_csn_o    : out std_ulogic;
    spi_clk_o    : out std_ulogic;
    spi_data_i   : in  std_ulogic;
    spi_data_o   : out std_ulogic
  );
end neorv32_xip_phy;

architecture neorv32_xip_phy_rtl of neorv32_xip_phy is

  -- controller --
  type ctrl_state_t is (S_IDLE, S_START, S_RTX_A, S_RTX_B, S_DONE);
  type ctrl_t is record
    state   : ctrl_state_t;
    sreg    : std_ulogic_vector(71 downto 0); -- only the lowest 32-bit are used as RX data
    bitcnt  : std_ulogic_vector(06 downto 0);
    di_sync : std_ulogic;
    csen    : std_ulogic;
  end record;
  signal ctrl : ctrl_t;

begin

  -- Serial Interface Control Unit ----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  control_unit: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (cf_enable_i = '0') then
        spi_clk_o    <= '0';
        spi_csn_o    <= '1';
        --
        ctrl.state   <= S_IDLE;
        ctrl.csen    <= '-';
        ctrl.sreg    <= (others => '-');
        ctrl.bitcnt  <= (others => '-');
        ctrl.di_sync <= '-';
      else
        -- defaults --
        spi_clk_o    <= cf_cpol_i;
        spi_csn_o    <= '1'; -- de-selected by default
        ctrl.di_sync <= spi_data_i;

        -- fsm --
        case ctrl.state is

          when S_IDLE => -- wait for new transmission trigger
          -- ------------------------------------------------------------
            ctrl.bitcnt <= op_nbytes_i & "000"; -- number of bytes
            ctrl.csen   <= op_csen_i;
            if (op_start_i = '1') then
              ctrl.sreg  <= op_wdata_i;
              ctrl.state <= S_START;
            end if;

          when S_START => -- sync start of transmission
          -- ------------------------------------------------------------
            spi_csn_o <= not ctrl.csen;
            if (spi_clk_en_i = '1') then
              ctrl.state <= S_RTX_A;
            end if;

          when S_RTX_A => -- first half of bit transmission
          -- ------------------------------------------------------------
            spi_csn_o <= not ctrl.csen;
            spi_clk_o <= cf_cpha_i xor cf_cpol_i;
            if (spi_clk_en_i = '1') then
              ctrl.bitcnt <= std_ulogic_vector(unsigned(ctrl.bitcnt) - 1);
              ctrl.state  <= S_RTX_B;
            end if;

          when S_RTX_B => -- second half of bit transmission
          -- ------------------------------------------------------------
            spi_csn_o <= not ctrl.csen;
            spi_clk_o <= not (cf_cpha_i xor cf_cpol_i);
            if (spi_clk_en_i = '1') then
              ctrl.sreg <= ctrl.sreg(ctrl.sreg'left-1 downto 0) & ctrl.di_sync;
              if (or_reduce_f(ctrl.bitcnt) = '0') then -- all bits transferred?
                ctrl.state <= S_DONE; -- transmission done
              else
                ctrl.state <= S_RTX_A; -- next bit
              end if;
            end if;

          when S_DONE => -- transmission done
          -- ------------------------------------------------------------
            spi_csn_o <= not ctrl.csen;
            if (spi_clk_en_i = '1') then
              ctrl.state <= S_IDLE;
            end if;

          when others => -- undefined
          -- ------------------------------------------------------------
            ctrl.state <= S_IDLE;

        end case;
      end if;
    end if;
  end process control_unit;

  -- serial unit busy --
  op_busy_o <= '0' when (ctrl.state = S_IDLE) else '1';

  -- serial data output --
  spi_data_o <= ctrl.sreg(ctrl.sreg'left);

  -- RX data --
  op_rdata_o <= ctrl.sreg(31 downto 0);


end neorv32_xip_phy_rtl;
