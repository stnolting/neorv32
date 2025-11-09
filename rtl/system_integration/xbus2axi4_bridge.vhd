-- ================================================================================ --
-- NEORV32 SoC - XBUS to AXI4-Compatible Bridge                                     --
-- -------------------------------------------------------------------------------- --
-- This bridge supports single read/write transfers and read-bursts.                --
-- [IMPORTANT] Write-bursts are not supported yet!                                  --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity xbus2axi4_bridge is
  generic (
    BURST_EN  : boolean; -- enable burst transfers
    BURST_LEN : natural range 4 to 1024 -- bytes per burst, has to be a multiple of 4
  );
  port (
    -- Global control --
    clk           : in  std_logic;
    resetn        : in  std_logic;
    -- XBUS device interface --
    xbus_adr_i    : in  std_ulogic_vector(31 downto 0);
    xbus_dat_i    : in  std_ulogic_vector(31 downto 0);
    xbus_cti_i    : in  std_ulogic_vector(2 downto 0);
    xbus_tag_i    : in  std_ulogic_vector(2 downto 0);
    xbus_we_i     : in  std_ulogic;
    xbus_sel_i    : in  std_ulogic_vector(3 downto 0);
    xbus_stb_i    : in  std_ulogic;
    xbus_dat_o    : out std_ulogic_vector(31 downto 0);
    xbus_ack_o    : out std_ulogic;
    xbus_err_o    : out std_ulogic;
    -- AXI4 host write address channel --
    m_axi_awaddr  : out std_logic_vector(31 downto 0);
    m_axi_awlen   : out std_logic_vector(7 downto 0);
    m_axi_awsize  : out std_logic_vector(2 downto 0);
    m_axi_awburst : out std_logic_vector(1 downto 0);
    m_axi_awcache : out std_logic_vector(3 downto 0);
    m_axi_awprot  : out std_logic_vector(2 downto 0);
    m_axi_awvalid : out std_logic;
    m_axi_awready : in  std_logic;
    -- AXI4 host write data channel --
    m_axi_wdata   : out std_logic_vector(31 downto 0);
    m_axi_wstrb   : out std_logic_vector(3 downto 0);
    m_axi_wlast   : out std_logic;
    m_axi_wvalid  : out std_logic;
    m_axi_wready  : in  std_logic;
    -- AXI4 host read address channel --
    m_axi_araddr  : out std_logic_vector(31 downto 0);
    m_axi_arlen   : out std_logic_vector(7 downto 0);
    m_axi_arsize  : out std_logic_vector(2 downto 0);
    m_axi_arburst : out std_logic_vector(1 downto 0);
    m_axi_arcache : out std_logic_vector(3 downto 0);
    m_axi_arprot  : out std_logic_vector(2 downto 0);
    m_axi_arvalid : out std_logic;
    m_axi_arready : in  std_logic;
    -- AXI4 host read data channel --
    m_axi_rdata   : in  std_logic_vector(31 downto 0);
    m_axi_rresp   : in  std_logic_vector(1 downto 0);
    m_axi_rlast   : in  std_logic;
    m_axi_rvalid  : in  std_logic;
    m_axi_rready  : out std_logic;
    -- AXI4 host write response channel --
    m_axi_bresp   : in  std_logic_vector(1 downto 0);
    m_axi_bvalid  : in  std_logic;
    m_axi_bready  : out std_logic
  );
end entity;

architecture xbus2axi4_bridge_rtl of xbus2axi4_bridge is

  type state_t is (S_IDLE, S_SINGLE, S_BURST_RUN, S_BURST_END);
  signal state : state_t;
  signal arvalid, awvalid, wvalid, xbus_rd_ack, xbus_rd_err, xbus_wr_ack, xbus_wr_err : std_ulogic;
  constant blen_c : std_logic_vector(7 downto 0) := std_logic_vector(to_unsigned((BURST_LEN/4)-1, 8));

begin

  -- handshake arbiter --
  arbiter: process(resetn, clk)
  begin
    if (resetn = '0') then
      arvalid <= '0';
      awvalid <= '0';
      wvalid  <= '0';
      state   <= S_IDLE;
    elsif rising_edge(clk) then
      -- AXI handshake --
      arvalid <= arvalid and std_ulogic(not m_axi_arready);
      awvalid <= awvalid and std_ulogic(not m_axi_awready);
      wvalid  <= wvalid  and std_ulogic(not m_axi_wready);

      -- state machine --
      case state is

        when S_SINGLE => -- single read/write transfer in progress
        -- ------------------------------------------------------------
          if (m_axi_rvalid = '1') or (m_axi_bvalid = '1') then
            state <= S_IDLE;
          end if;

        when S_BURST_RUN => -- burst read transfer in progress
        -- ------------------------------------------------------------
          if not BURST_EN then -- burst not supported
            state <= S_IDLE;
          elsif (m_axi_rlast = '1') then -- burst completed by device
            state <= S_BURST_END;
          end if;

        when S_BURST_END => -- end of burst
        -- ------------------------------------------------------------
          if not BURST_EN then -- burst not supported
            state <= S_IDLE;
          elsif (xbus_cti_i = "000") then -- burst completed by host
            state <= S_IDLE;
          end if;

        when others => -- S_IDLE: wait for access request
        -- ------------------------------------------------------------
          arvalid <= '0';
          awvalid <= '0';
          wvalid  <= '0';
          if (xbus_stb_i = '1') then -- access request
            if (xbus_cti_i = "010") and BURST_EN then -- incrementing address burst access
              arvalid <= '1'; -- read-only
              state   <= S_BURST_RUN;
            else -- single access (read/write)
              arvalid <= not xbus_we_i;
              awvalid <= xbus_we_i;
              wvalid  <= xbus_we_i;
              state   <= S_SINGLE;
            end if;
          end if;

      end case;
    end if;
  end process arbiter;

  -- AXI read address channel --
  m_axi_araddr  <= std_logic_vector(xbus_adr_i);
  m_axi_arlen   <= blen_c when (state = S_BURST_RUN) and BURST_EN else (others => '0'); -- burst length
  m_axi_arsize  <= "010"; -- 4 bytes per transfer
  m_axi_arburst <= "01"; -- incrementing bursts only
  m_axi_arcache <= "0011"; -- recommended by Vivado
  m_axi_arprot  <= std_logic_vector(xbus_tag_i);
  m_axi_arvalid <= std_logic(arvalid);

  -- AXI read data channel --
  m_axi_rready  <= '1'; -- always ready for read response
  xbus_rd_ack   <= '1' when (m_axi_rvalid = '1') and (m_axi_rresp  = "00") else '0';
  xbus_rd_err   <= '1' when (m_axi_rvalid = '1') and (m_axi_rresp /= "00") else '0';
  xbus_dat_o    <= std_ulogic_vector(m_axi_rdata);

  -- AXI write address channel --
  m_axi_awaddr  <= std_logic_vector(xbus_adr_i);
  m_axi_awlen   <= (others => '0'); -- burst length = 1
  m_axi_awsize  <= "010"; -- 4 bytes per transfer
  m_axi_awburst <= "01"; -- incrementing bursts only
  m_axi_awcache <= "0011"; -- recommended by Vivado
  m_axi_awprot  <= std_logic_vector(xbus_tag_i);
  m_axi_awvalid <= std_logic(awvalid);

  -- AXI write data channel --
  m_axi_wdata   <= std_logic_vector(xbus_dat_i);
  m_axi_wstrb   <= std_logic_vector(xbus_sel_i);
  m_axi_wlast   <= '1'; -- single transfers only; so this is also the last word
  m_axi_wvalid  <= std_logic(wvalid);

  -- AXI write response channel --
  m_axi_bready  <= '1'; -- always ready for write response
  xbus_wr_ack   <= '1' when (m_axi_bvalid = '1') and (m_axi_bresp  = "00") else '0';
  xbus_wr_err   <= '1' when (m_axi_bvalid = '1') and (m_axi_bresp /= "00") else '0';

  -- XBUS response --
  xbus_ack_o    <= xbus_rd_ack or xbus_wr_ack;
  xbus_err_o    <= xbus_rd_err or xbus_wr_err;

end architecture;
