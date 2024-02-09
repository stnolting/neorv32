-- #################################################################################################
-- # << NEORV32 - Execute In-Place (XIP) Module >>                                                 #
-- # ********************************************************************************************* #
-- # This module allows the CPU to execute code (and read constant data) directly from an SPI      #
-- # flash memory. Two host ports are implemented: one  for accessing the control and status       #
-- # registers (mapped to the processor's IO space) and one for the actual instruction/data fetch. #
-- # The actual address space mapping of the "instruction/data interface" is done by programming   #
-- # special control register bits.                                                                #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # The NEORV32 RISC-V Processor, https://github.com/stnolting/neorv32                            #
-- # Copyright (c) 2024, Stephan Nolting. All rights reserved.                                     #
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
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_xip is
  generic (
    XIP_CACHE_EN         : boolean;                 -- implement XIP cache?
    XIP_CACHE_NUM_BLOCKS : natural range 1 to 256;  -- number of blocks (min 1), has to be a power of 2
    XIP_CACHE_BLOCK_SIZE : natural range 1 to 2**16 -- block size in bytes (min 4), has to be a power of 2
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
  constant ctrl_burst_en_c    : natural := 29; -- r/-: XIP burst mode enable (when cache is implemented)
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

  -- cache access --
  signal cache_clear : std_ulogic;
  signal xip_req     : bus_req_t;
  signal xip_rsp     : bus_rsp_t;

  -- Clock generator --
  signal cdiv_cnt   : std_ulogic_vector(3 downto 0);
  signal spi_clk_en : std_ulogic;

  -- Component: XIP cache --
  component neorv32_xip_cache
    generic (
      CACHE_NUM_BLOCKS : natural range 1 to 256;  -- number of blocks (min 1), has to be a power of 2
      CACHE_BLOCK_SIZE : natural range 1 to 2**16 -- block size in bytes (min 4), has to be a power of 2
    );
    port (
      clk_i     : in  std_ulogic; -- global clock, rising edge
      rstn_i    : in  std_ulogic; -- global reset, low-active, async
      clear_i   : in  std_ulogic; -- cache clear
      cpu_req_i : in  bus_req_t;  -- request bus
      cpu_rsp_o : out bus_rsp_t;  -- response bus
      bus_req_o : out bus_req_t;  -- request bus
      bus_rsp_i : in  bus_rsp_t   -- response bus
    );
  end component;

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
      bus_rsp_o.ack  <= '0';
      bus_rsp_o.err  <= '0';
      bus_rsp_o.data <= (others => '0');
      ctrl           <= (others => '0');
      spi_data_lo    <= (others => '0');
      spi_data_hi    <= (others => '0');
      spi_trigger    <= '0';
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
              bus_rsp_o.data(ctrl_burst_en_c) <= bool_to_ulogic_f(XIP_CACHE_EN);
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


  -- XIP Cache ------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_xip_cache_inst_true:
  if XIP_CACHE_EN generate
    neorv32_xip_cache_inst: neorv32_xip_cache
    generic map (
      CACHE_NUM_BLOCKS => XIP_CACHE_NUM_BLOCKS,
      CACHE_BLOCK_SIZE => XIP_CACHE_BLOCK_SIZE
    )
    port map (
      clk_i     => clk_i,
      rstn_i    => rstn_i,
      clear_i   => cache_clear,
      cpu_req_i => xip_req_i,
      cpu_rsp_o => xip_rsp_o,
      bus_req_o => xip_req,
      bus_rsp_i => xip_rsp
    );
    -- clear cache when entire module or XIP-mode is disabled --
    cache_clear <= '1' when (ctrl(ctrl_enable_c) = '0') or (ctrl(ctrl_xip_enable_c) = '0') else '0';
  end generate;

  neorv32_xip_cache_inst_false:
  if not XIP_CACHE_EN generate
    xip_req   <= xip_req_i;
    xip_rsp_o <= xip_rsp;
  end generate;


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
      if (xip_req.stb = '1') and (xip_req.rw = '0') then
        arbiter.addr <= xip_req.addr; -- buffer address (reducing fan-out on CPU's address net)
      end if;
      arbiter.addr_lookahead <= std_ulogic_vector(unsigned(arbiter.addr) + 4); -- prefetch address of *next* linear access
      -- XIP access error? --
      if (arbiter.state = S_DIRECT) then
        arbiter.xip_acc_err <= xip_req.stb;
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
  arbiter_comb: process(arbiter, ctrl, xip_addr, phy_if, xip_req, spi_data_hi, spi_data_lo, spi_trigger)
  begin
    -- arbiter defaults --
    arbiter.state_nxt <= arbiter.state;

    -- bus interface defaults --
    xip_rsp.data <= (others => '0');
    xip_rsp.ack  <= '0';
    xip_rsp.err  <= arbiter.xip_acc_err;

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
        if (xip_req.stb = '1') then
          if (xip_req.rw = '0') then
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
        xip_rsp.data <= bswap32_f(phy_if.rdata); -- convert incrementing byte-read to little-endian
        if (phy_if.busy = '0') then
          xip_rsp.ack       <= '1';
          arbiter.state_nxt <= S_IDLE;
        end if;

      when S_ERROR => -- access error
      -- ------------------------------------------------------------
        xip_rsp.err       <= '1';
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


-- ############################################################################################################################
-- ############################################################################################################################


-- #################################################################################################
-- # << NEORV32 - XIP Module - SPI Physical Interface >>                                           #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # The NEORV32 RISC-V Processor, https://github.com/stnolting/neorv32                            #
-- # Copyright (c) 2024, Stephan Nolting. All rights reserved.                                     #
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
-- #################################################################################################

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
    op_nbytes_i  : in  std_ulogic_vector(03 downto 0); -- actual number of bytes to transmit (1..9)
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


-- ############################################################################################################################
-- ############################################################################################################################


-- #################################################################################################
-- # << NEORV32 - XIP Cache >>                                                                     #
-- # ********************************************************************************************* #
-- # Simple directed-mapped read-only cache to accelerate XIP (SPI) flash accesses.                #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # The NEORV32 RISC-V Processor, https://github.com/stnolting/neorv32                            #
-- # Copyright (c) 2024, Stephan Nolting. All rights reserved.                                     #
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
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_xip_cache is
  generic (
    CACHE_NUM_BLOCKS : natural range 1 to 256;  -- number of blocks (min 1), has to be a power of 2
    CACHE_BLOCK_SIZE : natural range 1 to 2**16 -- block size in bytes (min 4), has to be a power of 2
  );
  port (
    clk_i     : in  std_ulogic; -- global clock, rising edge
    rstn_i    : in  std_ulogic; -- global reset, low-active, async
    clear_i   : in  std_ulogic; -- cache clear
    cpu_req_i : in  bus_req_t;  -- request bus
    cpu_rsp_o : out bus_rsp_t;  -- response bus
    bus_req_o : out bus_req_t;  -- request bus
    bus_rsp_i : in  bus_rsp_t   -- response bus
  );
end neorv32_xip_cache;

architecture neorv32_xip_cache_rtl of neorv32_xip_cache is

  -- auto configuration --
  constant block_num_c   : natural := cond_sel_natural_f(is_power_of_two_f(CACHE_NUM_BLOCKS), CACHE_NUM_BLOCKS, 2**index_size_f(CACHE_NUM_BLOCKS));
  constant block_size_c  : natural := cond_sel_natural_f(is_power_of_two_f(CACHE_BLOCK_SIZE), CACHE_BLOCK_SIZE, 2**index_size_f(CACHE_BLOCK_SIZE));
  constant offset_size_c : natural := index_size_f(block_size_c/4); -- offset addresses full 32-bit words

  -- cache layout --
  constant index_size_c : natural := index_size_f(block_num_c);
  constant tag_size_c   : natural := 32 - (offset_size_c + index_size_c + 2); -- 2 additional bits for byte offset
  constant entries_c    : natural := block_num_c * (block_size_c/4); -- number of 32-bit entries (per set)

  -- cache interface --
  type cache_if_t is record
    host_rdata : std_ulogic_vector(31 downto 0); -- cpu read data
    host_rderr : std_ulogic; -- cpu read error
    hit        : std_ulogic; -- hit access
    ctrl_en    : std_ulogic; -- control access enable
    ctrl_we    : std_ulogic; -- control write enable
  end record;
  signal cache : cache_if_t;

  -- control engine --
  type ctrl_engine_state_t is (S_IDLE, S_CHECK, S_DOWNLOAD_REQ, S_DOWNLOAD_GET, S_RESYNC, S_ERROR);
  signal state,    state_nxt    : ctrl_engine_state_t; -- FSM state
  signal addr_reg, addr_reg_nxt : std_ulogic_vector(31 downto 0); -- address register for block download

  -- cache memory --
  type tag_mem_t is array (0 to block_num_c-1) of std_ulogic_vector(tag_size_c-1 downto 0);
  type data_mem_t is array (0 to entries_c-1) of std_ulogic_vector(31+1 downto 0); -- data word + ERR status
  signal tag_mem   : tag_mem_t;
  signal data_mem  : data_mem_t;
  signal tag_rd    : std_ulogic_vector(tag_size_c-1 downto 0); -- tag read data
  signal data_rd   : std_ulogic_vector(31+1 downto 0); -- data word + ERR status
  signal valid_mem : std_ulogic_vector(block_num_c-1 downto 0);
  signal valid_rd  : std_ulogic; -- valid flag read data

  -- access address decomposition --
  type acc_addr_t is record
    tag    : std_ulogic_vector(tag_size_c-1 downto 0);
    index  : std_ulogic_vector(index_size_c-1 downto 0);
    offset : std_ulogic_vector(offset_size_c-1 downto 0);
  end record;
  signal host_acc, ctrl_acc : acc_addr_t;

  -- cache data memory access --
  signal cache_index  : std_ulogic_vector(index_size_c-1 downto 0);
  signal cache_offset : std_ulogic_vector(offset_size_c-1 downto 0);
  signal cache_addr   : std_ulogic_vector((index_size_c+offset_size_c)-1 downto 0); -- index & offset

begin

  -- Control Engine FSM Sync ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_engine_fsm_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      state    <= S_IDLE;
      addr_reg <= (others => '0');
    elsif rising_edge(clk_i) then
      state    <= state_nxt;
      addr_reg <= addr_reg_nxt;
    end if;
  end process ctrl_engine_fsm_sync;


  -- Control Engine FSM Comb ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_engine_fsm_comb: process(state, addr_reg, cache, clear_i, cpu_req_i, bus_rsp_i)
  begin
    -- control defaults --
    state_nxt      <= state;
    addr_reg_nxt   <= addr_reg;

    -- cache defaults --
    cache.ctrl_en  <= '0';
    cache.ctrl_we  <= '0';

    -- host response defaults --
    cpu_rsp_o.ack  <= '0';
    cpu_rsp_o.err  <= '0';
    cpu_rsp_o.data <= (others => '0');

    -- bus interface defaults --
    bus_req_o.data <= (others => '0');
    bus_req_o.ben  <= (others => '0');
    bus_req_o.src  <= cpu_req_i.src;
    bus_req_o.priv <= cpu_req_i.priv;
    bus_req_o.addr <= addr_reg;
    bus_req_o.rw   <= '0'; -- read-only
    bus_req_o.stb  <= '0';
    bus_req_o.rvso <= cpu_req_i.rvso;

    -- fsm --
    case state is

      when S_IDLE => -- wait for host access request or cache control operation
      -- ------------------------------------------------------------
        if (cpu_req_i.stb = '1') then
          if (cpu_req_i.rw = '1') or (clear_i = '1') then -- write access or cache being cleared
            state_nxt <= S_ERROR;
          else -- actual cache access
            state_nxt <= S_CHECK;
          end if;
        end if;

      when S_CHECK => -- finalize host access if cache hit
      -- ------------------------------------------------------------
        -- calculate block base address in case we need to download it --
        addr_reg_nxt                               <= cpu_req_i.addr;
        addr_reg_nxt((offset_size_c+2)-1 downto 0) <= (others => '0'); -- block-aligned
        --
        cpu_rsp_o.data <= cache.host_rdata; -- output read data in case we have a hit
        if (cache.hit = '1') then -- cache HIT
          cpu_rsp_o.err <=     cache.host_rderr;
          cpu_rsp_o.ack <= not cache.host_rderr;
          state_nxt     <= S_IDLE;
        else -- cache MISS
          state_nxt <= S_DOWNLOAD_REQ;
        end if;

      when S_DOWNLOAD_REQ => -- download new cache block: request new word
      -- ------------------------------------------------------------
        bus_req_o.stb <= '1'; -- request new read transfer
        state_nxt     <= S_DOWNLOAD_GET;

      when S_DOWNLOAD_GET => -- download new cache block: wait for bus response
      -- ------------------------------------------------------------
        cache.ctrl_en <= '1'; -- cache update operation
        if (bus_rsp_i.ack = '1') or (bus_rsp_i.err = '1') then -- ACK or ERROR = write to cache and get next word (store ERROR flag in cache)
          cache.ctrl_we <= '1'; -- write to cache
          if (and_reduce_f(addr_reg((offset_size_c+2)-1 downto 2)) = '1') then -- block complete?
            state_nxt <= S_RESYNC;
          else -- get next word
            addr_reg_nxt <= std_ulogic_vector(unsigned(addr_reg) + 4);
            state_nxt    <= S_DOWNLOAD_REQ;
          end if;
        end if;

      when S_RESYNC => -- re-sync host/cache access: cache read-latency dummy cycle
      -- ------------------------------------------------------------
        state_nxt <= S_CHECK;

      when others => -- S_ERROR: error
      -- ------------------------------------------------------------
        cpu_rsp_o.err <= '1';
        state_nxt     <= S_IDLE;

    end case;
  end process ctrl_engine_fsm_comb;


  -- Access Address Decomposition -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  host_acc.tag    <= cpu_req_i.addr(31 downto 31-(tag_size_c-1));
  host_acc.index  <= cpu_req_i.addr(31-tag_size_c downto 2+offset_size_c);
  host_acc.offset <= cpu_req_i.addr(2+(offset_size_c-1) downto 2); -- discard byte offset

  ctrl_acc.tag    <= addr_reg(31 downto 31-(tag_size_c-1));
  ctrl_acc.index  <= addr_reg(31-tag_size_c downto 2+offset_size_c);
  ctrl_acc.offset <= addr_reg(2+(offset_size_c-1) downto 2); -- discard byte offset


  -- Status Flag Memory ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  status_memory: process(rstn_i, clk_i) -- single-port RAM
  begin
    if (rstn_i = '0') then
      valid_mem <= (others => '0');
      valid_rd  <= '0';
    elsif rising_edge(clk_i) then
      if (clear_i = '1') then -- invalidate cache
        valid_mem <= (others => '0');
      elsif (cache.ctrl_we = '1') then -- make current block valid
        valid_mem(to_integer(unsigned(cache_index))) <= '1';
      end if;
      valid_rd <= valid_mem(to_integer(unsigned(cache_index)));
    end if;
  end process status_memory;


  -- Cache Data Memory ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cache_memory: process(clk_i) -- single-port RAM
  begin
    if rising_edge(clk_i) then -- no reset to allow mapping to blockRAM
      if (cache.ctrl_we = '1') then -- update cache block
        data_mem(to_integer(unsigned(cache_addr))) <= bus_rsp_i.err & bus_rsp_i.data;
        tag_mem(to_integer(unsigned(cache_index))) <= ctrl_acc.tag;
      end if;
      data_rd <= data_mem(to_integer(unsigned(cache_addr)));
      tag_rd  <= tag_mem(to_integer(unsigned(cache_index)));
    end if;
  end process cache_memory;

  -- cache access select --
  cache_index  <= host_acc.index  when (cache.ctrl_en = '0') else ctrl_acc.index;
  cache_offset <= host_acc.offset when (cache.ctrl_en = '0') else ctrl_acc.offset;
  cache_addr   <= cache_index & cache_offset; -- resulting ram access address

  -- hit = tag match and valid entry --
  cache.hit <= '1' when (host_acc.tag = tag_rd) and (valid_rd = '1') else '0';

  -- data output --
  cache.host_rdata <= data_rd(31 downto 0);
  cache.host_rderr <= data_rd(32);


end neorv32_xip_cache_rtl;
