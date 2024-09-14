-- ================================================================================ --
-- NEORV32 SoC - Execute In-Place Module (XIP)                                      --
-- -------------------------------------------------------------------------------- --
-- This module allows the CPU to execute code (and read constant data) directly     --
-- from an SPI flash memory by transforming bus accesses into SPI transmissions.    --
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

entity neorv32_xip is
  generic (
    XIP_CACHE_EN : boolean -- external XIP cache is enabled
  );
  port (
    clk_i       : in  std_ulogic; -- global clock line
    rstn_i      : in  std_ulogic; -- global reset line, low-active
    bus_req_i   : in  bus_req_t;  -- bus request
    bus_rsp_o   : out bus_rsp_t;  -- bus response
    xip_req_i   : in  bus_req_t;  -- XIP request
    xip_rsp_o   : out bus_rsp_t;  -- XIP response
    clkgen_en_o : out std_ulogic; -- enable clock generator
    clkgen_i    : in  std_ulogic_vector(7 downto 0);
    spi_csn_o   : out std_ulogic; -- chip-select, low-active
    spi_clk_o   : out std_ulogic; -- serial clock
    spi_dat_i   : in  std_ulogic; -- device data output
    spi_dat_o   : out std_ulogic  -- controller data output
  );
end neorv32_xip;

architecture neorv32_xip_rtl of neorv32_xip is

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
  constant ctrl_spi_csen_c    : natural := 21; -- r/w: SPI chip-select enabled
  constant ctrl_highspeed_c   : natural := 22; -- r/w: SPI high-speed mode enable (ignoring ctrl_spi_prsc)
  constant ctrl_cdiv0_c       : natural := 23; -- r/w: clock divider bit 0
  constant ctrl_cdiv1_c       : natural := 24; -- r/w: clock divider bit 1
  constant ctrl_cdiv2_c       : natural := 25; -- r/w: clock divider bit 2
  constant ctrl_cdiv3_c       : natural := 26; -- r/w: clock divider bit 3
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
  type arbiter_state_t is (S_DIRECT, S_IDLE, S_CHECK, S_TRIG, S_BUSY, S_ERROR);
  type arbiter_t is record
    state          : arbiter_state_t;
    state_nxt      : arbiter_state_t;
    addr           : std_ulogic_vector(31 downto 0);
    addr_lookahead : std_ulogic_vector(31 downto 0);
    xip_acc_err    : std_ulogic;
    busy           : std_ulogic;
    tmo_cnt        : std_ulogic_vector(2 downto 0); -- timeout counter for auto CS de-assert (burst mode only)
  end record;
  signal arbiter : arbiter_t;

  -- Clock generator --
  signal cdiv_cnt   : std_ulogic_vector(3 downto 0);
  signal spi_clk_en : std_ulogic;

  -- Component: SPI PHY --
  component neorv32_xip_phy
    port (
      -- global control --
      rstn_i       : in  std_ulogic; -- reset, async, low-active
      clk_i        : in  std_ulogic; -- clock
      spi_clk_en_i : in  std_ulogic; -- pre-scaled SPI clock-enable
      -- operation configuration --
      cf_enable_i  : in  std_ulogic; -- module enable (reset if low)
      cf_cpha_i    : in  std_ulogic; -- clock phase
      cf_cpol_i    : in  std_ulogic; -- clock idle polarity
      -- operation control --
      op_start_i   : in  std_ulogic; -- trigger new transmission
      op_final_i   : in  std_ulogic; -- end current transmission
      op_csen_i    : in  std_ulogic; -- actually enabled device for transmission
      op_busy_o    : out std_ulogic; -- transmission in progress when set
      op_nbytes_i  : in  std_ulogic_vector(3 downto 0); -- actual number of bytes to transmit (1..9)
      op_wdata_i   : in  std_ulogic_vector(71 downto 0); -- write data
      op_rdata_o   : out std_ulogic_vector(31 downto 0); -- read data
      -- SPI interface --
      spi_csn_o    : out std_ulogic;
      spi_clk_o    : out std_ulogic;
      spi_dat_i    : in  std_ulogic;
      spi_dat_o    : out std_ulogic
    );
  end component;

  -- SPI PHY interface --
  type phy_if_t is record
    start : std_ulogic; -- trigger new transmission
    final : std_ulogic; -- stop current transmission
    busy  : std_ulogic; -- transmission in progress when set
    wdata : std_ulogic_vector(71 downto 0); -- write data
    rdata : std_ulogic_vector(31 downto 0); -- read data
  end record;
  signal phy_if : phy_if_t;

begin

  -- Control Bus Access ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_bus_access : process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bus_rsp_o   <= rsp_terminate_c;
      ctrl        <= (others => '0');
      spi_data_lo <= (others => '0');
      spi_data_hi <= (others => '0');
      spi_trigger <= '0';
    elsif rising_edge(clk_i) then
      -- bus handshake --
      bus_rsp_o.ack  <= bus_req_i.stb;
      bus_rsp_o.err  <= '0';
      bus_rsp_o.data <= (others => '0');

      -- defaults --
      spi_trigger <= '0';

      if (bus_req_i.stb = '1') then

        -- write access --
        if (bus_req_i.rw = '1') then
          -- control register --
          if (bus_req_i.addr(3 downto 2) = "00") then
            ctrl(ctrl_enable_c)                                <= bus_req_i.data(ctrl_enable_c);
            ctrl(ctrl_spi_prsc2_c downto ctrl_spi_prsc0_c)     <= bus_req_i.data(ctrl_spi_prsc2_c downto ctrl_spi_prsc0_c);
            ctrl(ctrl_spi_cpol_c)                              <= bus_req_i.data(ctrl_spi_cpol_c);
            ctrl(ctrl_spi_cpha_c)                              <= bus_req_i.data(ctrl_spi_cpha_c);
            ctrl(ctrl_spi_nbytes3_c downto ctrl_spi_nbytes0_c) <= bus_req_i.data(ctrl_spi_nbytes3_c downto ctrl_spi_nbytes0_c);
            ctrl(ctrl_xip_enable_c)                            <= bus_req_i.data(ctrl_xip_enable_c);
            ctrl(ctrl_xip_abytes1_c downto ctrl_xip_abytes0_c) <= bus_req_i.data(ctrl_xip_abytes1_c downto ctrl_xip_abytes0_c);
            ctrl(ctrl_rd_cmd7_c downto ctrl_rd_cmd0_c)         <= bus_req_i.data(ctrl_rd_cmd7_c downto ctrl_rd_cmd0_c);
            ctrl(ctrl_spi_csen_c)                              <= bus_req_i.data(ctrl_spi_csen_c);
            ctrl(ctrl_highspeed_c)                             <= bus_req_i.data(ctrl_highspeed_c);
            ctrl(ctrl_cdiv3_c downto ctrl_cdiv0_c)             <= bus_req_i.data(ctrl_cdiv3_c downto ctrl_cdiv0_c);
          end if;
          -- SPI direct data access register lo --
          if (bus_req_i.addr(3 downto 2) = "10") then
            spi_data_lo <= bus_req_i.data;
          end if;
          -- SPI direct data access register hi --
          if (bus_req_i.addr(3 downto 2) = "11") then
            spi_data_hi <= bus_req_i.data;
            spi_trigger <= '1'; -- trigger direct SPI transaction
          end if;

        -- read access --
        else
          case bus_req_i.addr(3 downto 2) is
            when "00" => -- 'xip_ctrl_addr_c' - control register
              bus_rsp_o.data(ctrl_enable_c)                                <= ctrl(ctrl_enable_c);
              bus_rsp_o.data(ctrl_spi_prsc2_c downto ctrl_spi_prsc0_c)     <= ctrl(ctrl_spi_prsc2_c downto ctrl_spi_prsc0_c);
              bus_rsp_o.data(ctrl_spi_cpol_c)                              <= ctrl(ctrl_spi_cpol_c);
              bus_rsp_o.data(ctrl_spi_cpha_c)                              <= ctrl(ctrl_spi_cpha_c);
              bus_rsp_o.data(ctrl_spi_nbytes3_c downto ctrl_spi_nbytes0_c) <= ctrl(ctrl_spi_nbytes3_c downto ctrl_spi_nbytes0_c);
              bus_rsp_o.data(ctrl_xip_enable_c)                            <= ctrl(ctrl_xip_enable_c);
              bus_rsp_o.data(ctrl_xip_abytes1_c downto ctrl_xip_abytes0_c) <= ctrl(ctrl_xip_abytes1_c downto ctrl_xip_abytes0_c);
              bus_rsp_o.data(ctrl_rd_cmd7_c downto ctrl_rd_cmd0_c)         <= ctrl(ctrl_rd_cmd7_c downto ctrl_rd_cmd0_c);
              bus_rsp_o.data(ctrl_spi_csen_c)                              <= ctrl(ctrl_spi_csen_c);
              bus_rsp_o.data(ctrl_highspeed_c)                             <= ctrl(ctrl_highspeed_c);
              bus_rsp_o.data(ctrl_cdiv3_c downto ctrl_cdiv0_c)             <= ctrl(ctrl_cdiv3_c downto ctrl_cdiv0_c);
              --
              bus_rsp_o.data(ctrl_phy_busy_c) <= phy_if.busy;
              bus_rsp_o.data(ctrl_xip_busy_c) <= arbiter.busy;
            when "10" => -- 'xip_data_lo_addr_c' - SPI direct data access register lo
              bus_rsp_o.data <= phy_if.rdata;
            when others => -- unavailable (not implemented or write-only)
              bus_rsp_o.data <= (others => '0');
          end case;
        end if;

      end if;
    end if;
  end process ctrl_bus_access;


  -- XIP Address Computation Logic ----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  xip_access_logic: process(arbiter.addr, ctrl)
    variable tmp_v : std_ulogic_vector(31 downto 0);
  begin
    tmp_v(31 downto 28) := "0000";
    tmp_v(27 downto 02) := arbiter.addr(27 downto 02);
    tmp_v(01 downto 00) := "00"; -- always align to 32-bit boundary; sub-word read accesses are handled by the CPU logic
    case ctrl(ctrl_xip_abytes1_c downto ctrl_xip_abytes0_c) is -- shift address bits to be MSB-aligned
      when "00"   => xip_addr <= tmp_v(07 downto 0) & x"000000"; -- 1 address byte
      when "01"   => xip_addr <= tmp_v(15 downto 0) & x"0000";   -- 2 address bytes
      when "10"   => xip_addr <= tmp_v(23 downto 0) & x"00";     -- 3 address bytes
      when others => xip_addr <= tmp_v(31 downto 0);             -- 4 address bytes
    end case;
  end process xip_access_logic;


  -- SPI Access Arbiter ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  arbiter_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      arbiter.state          <= S_DIRECT;
      arbiter.addr           <= (others => '0');
      arbiter.addr_lookahead <= (others => '0');
      arbiter.xip_acc_err    <= '0';
      arbiter.tmo_cnt        <= (others => '0');
    elsif rising_edge(clk_i) then
      -- state control --
      if (ctrl(ctrl_enable_c) = '0') or (ctrl(ctrl_xip_enable_c) = '0') then -- sync reset
        arbiter.state <= S_DIRECT;
      else
        arbiter.state <= arbiter.state_nxt;
      end if;
      -- address look-ahead --
      if (xip_req_i.stb = '1') and (xip_req_i.rw = '0') then
        arbiter.addr <= xip_req_i.addr; -- buffer address (reducing fan-out on CPU's address net)
      end if;
      arbiter.addr_lookahead <= std_ulogic_vector(unsigned(arbiter.addr) + 4); -- prefetch address of *next* linear access
      -- XIP access error? --
      if (arbiter.state = S_DIRECT) then
        arbiter.xip_acc_err <= xip_req_i.stb;
      else
        arbiter.xip_acc_err <= '0';
      end if;
      -- pending flash access timeout --
      if (ctrl(ctrl_enable_c) = '0') or (ctrl(ctrl_xip_enable_c) = '0') or (arbiter.state = S_BUSY) then -- sync reset
        arbiter.tmo_cnt <= (others => '0');
      elsif (arbiter.tmo_cnt(arbiter.tmo_cnt'left) = '0') then -- stop if maximum reached
        arbiter.tmo_cnt <= std_ulogic_vector(unsigned(arbiter.tmo_cnt) + 1);
      end if;
    end if;
  end process arbiter_sync;


  -- FSM - combinatorial part --
  arbiter_comb: process(arbiter, ctrl, xip_addr, phy_if, xip_req_i, spi_data_hi, spi_data_lo, spi_trigger)
  begin
    -- arbiter defaults --
    arbiter.state_nxt <= arbiter.state;

    -- bus interface defaults --
    xip_rsp_o.data <= (others => '0');
    xip_rsp_o.ack  <= '0';
    xip_rsp_o.err  <= arbiter.xip_acc_err;

    -- SPI PHY interface defaults --
    phy_if.start <= '0';
    phy_if.final <= arbiter.tmo_cnt(arbiter.tmo_cnt'left) or (not bool_to_ulogic_f(XIP_CACHE_EN)); -- terminate if timeout or if burst mode not enabled
    phy_if.wdata <= ctrl(ctrl_rd_cmd7_c downto ctrl_rd_cmd0_c) & xip_addr & x"00000000"; -- MSB-aligned: CMD + address + 32-bit zero data

    -- fsm --
    case arbiter.state is

      when S_DIRECT => -- XIP access disabled; direct SPI access
      -- ------------------------------------------------------------
        phy_if.wdata      <= spi_data_hi & spi_data_lo & x"00"; -- MSB-aligned data
        phy_if.start      <= spi_trigger;
        phy_if.final      <= '1'; -- do not keep CS active after transmission is done
        arbiter.state_nxt <= S_IDLE;

      when S_IDLE => -- wait for new bus request
      -- ------------------------------------------------------------
        if (xip_req_i.stb = '1') then
          if (xip_req_i.rw = '0') then
            arbiter.state_nxt <= S_CHECK;
          else
            arbiter.state_nxt <= S_ERROR;
          end if;
        end if;

      when S_CHECK => -- check if we can resume flash access
      -- ------------------------------------------------------------
        if (arbiter.addr(27 downto 2) = arbiter.addr_lookahead(27 downto 2)) and XIP_CACHE_EN and -- access to *next linear* address
           (arbiter.tmo_cnt(arbiter.tmo_cnt'left) = '0') then -- no "pending access" timeout yet
          phy_if.start      <= '1'; -- resume flash access
          arbiter.state_nxt <= S_BUSY;
        else
          phy_if.final      <= '1'; -- restart flash access
          arbiter.state_nxt <= S_TRIG;
        end if;

      when S_TRIG => -- trigger NEW flash read
      -- ------------------------------------------------------------
        phy_if.start      <= '1';
        arbiter.state_nxt <= S_BUSY;

      when S_BUSY => -- wait for PHY to complete operation
      -- ------------------------------------------------------------
        xip_rsp_o.data <= bswap_f(phy_if.rdata); -- convert incrementing byte-read to little-endian
        if (phy_if.busy = '0') then
          xip_rsp_o.ack     <= '1';
          arbiter.state_nxt <= S_IDLE;
        end if;

      when S_ERROR => -- access error
      -- ------------------------------------------------------------
        xip_rsp_o.err     <= '1';
        arbiter.state_nxt <= S_IDLE;

      when others => -- undefined
      -- ------------------------------------------------------------
        arbiter.state_nxt <= S_IDLE;

    end case;
  end process arbiter_comb;

  -- arbiter status --
  arbiter.busy <= '1' when (arbiter.state = S_TRIG) or (arbiter.state = S_BUSY) else '0'; -- actual XIP access in progress


  -- SPI Clock Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  clock_generator: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      spi_clk_en <= '0';
      cdiv_cnt   <= (others => '0');
    elsif rising_edge(clk_i) then
      spi_clk_en <= '0'; -- default
      if (ctrl(ctrl_enable_c) = '0') then -- reset/disabled
        cdiv_cnt <= (others => '0');
      elsif (clkgen_i(to_integer(unsigned(ctrl(ctrl_spi_prsc2_c downto ctrl_spi_prsc0_c)))) = '1') or
            (ctrl(ctrl_highspeed_c) = '1') then -- pre-scaled clock
        if (cdiv_cnt = ctrl(ctrl_cdiv3_c downto ctrl_cdiv0_c)) then -- clock divider for fine-tuning
          spi_clk_en <= '1';
          cdiv_cnt   <= (others => '0');
        else
          cdiv_cnt <= std_ulogic_vector(unsigned(cdiv_cnt) + 1);
        end if;
      end if;
    end if;
  end process clock_generator;

  -- enable clock generator --
  clkgen_en_o <= ctrl(ctrl_enable_c);


  -- SPI Physical Interface -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_xip_phy_inst: neorv32_xip_phy
  port map (
    -- global control --
    rstn_i       => rstn_i,
    clk_i        => clk_i,
    spi_clk_en_i => spi_clk_en,
    -- operation configuration --
    cf_enable_i  => ctrl(ctrl_enable_c),   -- module enable (reset if low)
    cf_cpha_i    => ctrl(ctrl_spi_cpha_c), -- clock phase
    cf_cpol_i    => ctrl(ctrl_spi_cpol_c), -- clock idle polarity
    -- operation control --
    op_start_i   => phy_if.start,          -- trigger new transmission
    op_final_i   => phy_if.final,          -- end current transmission
    op_csen_i    => ctrl(ctrl_spi_csen_c), -- actually enabled device for transmission
    op_busy_o    => phy_if.busy,           -- transmission in progress when set
    op_nbytes_i  => ctrl(ctrl_spi_nbytes3_c downto ctrl_spi_nbytes0_c), -- actual number of bytes to transmit
    op_wdata_i   => phy_if.wdata,          -- write data
    op_rdata_o   => phy_if.rdata,          -- read data
    -- SPI interface --
    spi_csn_o    => spi_csn_o,
    spi_clk_o    => spi_clk_o,
    spi_dat_i    => spi_dat_i,
    spi_dat_o    => spi_dat_o
  );


end neorv32_xip_rtl;


-- ================================================================================ --
-- NEORV32 SoC - XIP Module - SPI Physical Interface                                --
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

entity neorv32_xip_phy is
  port (
    -- global control --
    rstn_i       : in  std_ulogic; -- reset, async, low-active
    clk_i        : in  std_ulogic; -- clock
    spi_clk_en_i : in  std_ulogic; -- pre-scaled SPI clock-enable
    -- operation configuration --
    cf_enable_i  : in  std_ulogic; -- module enable (reset if low)
    cf_cpha_i    : in  std_ulogic; -- clock phase
    cf_cpol_i    : in  std_ulogic; -- clock idle polarity
    -- operation control --
    op_start_i   : in  std_ulogic; -- trigger new transmission
    op_final_i   : in  std_ulogic; -- end current transmission
    op_csen_i    : in  std_ulogic; -- actually enabled device for transmission
    op_busy_o    : out std_ulogic; -- transmission in progress when set
    op_nbytes_i  : in  std_ulogic_vector(3 downto 0); -- actual number of bytes to transmit (1..9)
    op_wdata_i   : in  std_ulogic_vector(71 downto 0); -- write data
    op_rdata_o   : out std_ulogic_vector(31 downto 0); -- read data
    -- SPI interface --
    spi_csn_o    : out std_ulogic;
    spi_clk_o    : out std_ulogic;
    spi_dat_i    : in  std_ulogic;
    spi_dat_o    : out std_ulogic
  );
end neorv32_xip_phy;

architecture neorv32_xip_phy_rtl of neorv32_xip_phy is

  -- serial engine --
  type ctrl_state_t is (S_IDLE, S_WAIT, S_START, S_SYNC, S_RTX_A, S_RTX_B, S_DONE);
  type ctrl_t is record
    state   : ctrl_state_t;
    sreg    : std_ulogic_vector(71 downto 0); -- only the lowest 32-bit are used as RX data
    bitcnt  : std_ulogic_vector(06 downto 0);
    di_sync : std_ulogic;
    csen    : std_ulogic;
  end record;
  signal ctrl : ctrl_t;

begin

  -- Serial Interface Engine ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  serial_engine: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      spi_clk_o    <= '0';
      spi_csn_o    <= '1';
      ctrl.state   <= S_IDLE;
      ctrl.csen    <= '0';
      ctrl.sreg    <= (others => '0');
      ctrl.bitcnt  <= (others => '0');
      ctrl.di_sync <= '0';
    elsif rising_edge(clk_i) then
      if (cf_enable_i = '0') then -- sync reset
        spi_clk_o    <= '0';
        spi_csn_o    <= '1';
        ctrl.state   <= S_IDLE;
        ctrl.csen    <= '0';
        ctrl.sreg    <= (others => '0');
        ctrl.bitcnt  <= (others => '0');
        ctrl.di_sync <= '0';
      else -- fsm
        case ctrl.state is

          when S_IDLE => -- wait for new transmission trigger
          -- ------------------------------------------------------------
            spi_csn_o   <= '1'; -- flash disabled
            spi_clk_o   <= cf_cpol_i;
            ctrl.bitcnt <= op_nbytes_i & "000"; -- number of bytes
            ctrl.csen   <= op_csen_i;
            if (op_start_i = '1') then
              ctrl.state <= S_START;
            end if;

          when S_START => -- start of transmission (keep current spi_csn_o state!)
          -- ------------------------------------------------------------
            ctrl.sreg <= op_wdata_i;
            if (spi_clk_en_i = '1') then
              ctrl.state <= S_SYNC;
            end if;

          when S_WAIT => -- wait for resume transmission trigger
          -- ------------------------------------------------------------
            spi_csn_o   <= not ctrl.csen; -- keep CS active
            ctrl.bitcnt <= "0100000"; -- 4 bytes = 32-bit read data
            if (op_final_i = '1') then -- terminate pending flash access
              ctrl.state  <= S_IDLE;
            elsif (op_start_i = '1') then -- resume flash access
              ctrl.state  <= S_SYNC;
            end if;

          when S_SYNC => -- synchronize SPI clock
          -- ------------------------------------------------------------
            spi_csn_o <= not ctrl.csen; -- enable flash
            if (spi_clk_en_i = '1') then
              if (cf_cpha_i = '1') then -- clock phase shift
                spi_clk_o <= not cf_cpol_i;
              end if;
              ctrl.state <= S_RTX_A;
            end if;

          when S_RTX_A => -- first half of bit transmission
          -- ------------------------------------------------------------
            if (spi_clk_en_i = '1') then
              spi_clk_o    <= not (cf_cpha_i xor cf_cpol_i);
              ctrl.di_sync <= spi_dat_i;
              ctrl.bitcnt  <= std_ulogic_vector(unsigned(ctrl.bitcnt) - 1);
              ctrl.state   <= S_RTX_B;
            end if;

          when S_RTX_B => -- second half of bit transmission
          -- ------------------------------------------------------------
            if (spi_clk_en_i = '1') then
              ctrl.sreg <= ctrl.sreg(ctrl.sreg'left-1 downto 0) & ctrl.di_sync;
              if (or_reduce_f(ctrl.bitcnt) = '0') then -- all bits transferred?
                spi_clk_o  <= cf_cpol_i;
                ctrl.state <= S_DONE; -- transmission done
              else
                spi_clk_o  <= cf_cpha_i xor cf_cpol_i;
                ctrl.state <= S_RTX_A; -- next bit
              end if;
            end if;

          when S_DONE => -- transmission done
          -- ------------------------------------------------------------
            if (spi_clk_en_i = '1') then
              ctrl.state <= S_WAIT;
            end if;

          when others => -- undefined
          -- ------------------------------------------------------------
            ctrl.state <= S_IDLE;

        end case;
      end if;
    end if;
  end process serial_engine;

  -- serial unit busy --
  op_busy_o <= '0' when (ctrl.state = S_IDLE) or (ctrl.state = S_WAIT) else '1';

  -- serial data output --
  spi_dat_o <= ctrl.sreg(ctrl.sreg'left);

  -- RX data --
  op_rdata_o <= ctrl.sreg(31 downto 0);


end neorv32_xip_phy_rtl;
