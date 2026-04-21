-- ================================================================================ --
-- NEORV32 SoC - XBUS to AXI4-Compatible Bridge                                     --
-- -------------------------------------------------------------------------------- --
-- Supported transfers: Single Transfers + Incrementing Address Bursts.             --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  --
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

  constant blen_c : std_ulogic_vector(7 downto 0) := std_ulogic_vector(to_unsigned((BURST_LEN/4)-1, 8));
  signal arvalid, awvalid, wvalid, wb_ack, xbus_rd_ack, xbus_rd_err, xbus_wr_ack, xbus_wr_err : std_ulogic;
  signal state : std_ulogic_vector(1 downto 0);

begin

  -- AXI transfer arbiter --
  arbiter: process(resetn, clk)
  begin
    if (resetn = '0') then
      arvalid <= '0';
      awvalid <= '0';
      wvalid  <= '0';
      wb_ack  <= '0';
      state   <= (others => '0');
    elsif rising_edge(clk) then
      -- AXI handshake --
      arvalid <= arvalid and std_ulogic(not m_axi_arready);
      awvalid <= awvalid and std_ulogic(not m_axi_awready);
      wvalid  <= wvalid  and std_ulogic(not m_axi_wready);
      -- state machine --
      wb_ack <= '0';
      case state is

        when "00" => -- idle; wait for access request
        -- ------------------------------------------------------------
          arvalid <= xbus_stb_i and (not xbus_we_i);
          awvalid <= xbus_stb_i and xbus_we_i;
          wvalid  <= xbus_stb_i and xbus_we_i;
          if (xbus_stb_i = '1') then
            if (xbus_cti_i = "000") then -- single transfer
              state <= "01";
            elsif BURST_EN and (xbus_cti_i = "010") then -- incrementing address burst
              state <= '1' & xbus_we_i;
            end if;
          end if;

        when "01" => -- single transfer in progress
        -- ------------------------------------------------------------
          if (m_axi_rvalid = '1') or (m_axi_bvalid = '1') then
            state <= "00";
          end if;

        when "10" | "11" => -- burst transfer in progress
        -- ------------------------------------------------------------
          if (state(0) = '1') and BURST_EN then
            if (wvalid = '1') or ((xbus_cti_i = "010") and (xbus_stb_i = '1')) then
              wb_ack <= '1'; -- issue BURST_LEN-1 local ACKs during write-burst
            end if;
          end if;
          if (xbus_cti_i = "000") or (BURST_EN = false) then
            state <= "00";
          end if;

        when others => -- undefined
        -- ------------------------------------------------------------
          state <= "00";

      end case;
    end if;
  end process arbiter;

  -- AXI read address channel --
  m_axi_araddr  <= std_logic_vector(xbus_adr_i);
  m_axi_arlen   <= std_logic_vector(blen_c) when BURST_EN and (state(1) = '1') else (others => '0'); -- burst length
  m_axi_arsize  <= "010"; -- 4 bytes per transfer
  m_axi_arburst <= "01"; -- incrementing bursts only
  m_axi_arcache <= "0011"; -- recommended by Vivado
  m_axi_arprot  <= std_logic_vector(xbus_tag_i);
  m_axi_arvalid <= std_logic(arvalid);

  -- AXI read data channel --
  m_axi_rready  <= '1'; -- always ready for read response
  xbus_rd_ack   <= '1' when (m_axi_rvalid = '1') and (m_axi_rresp(1) = '0') else '0'; -- OKAY(00)/EXOKAY(01)
  xbus_rd_err   <= '1' when (m_axi_rvalid = '1') and (m_axi_rresp(1) = '1') else '0'; -- SLVERR(10)/DECERR(11)
  xbus_dat_o    <= std_ulogic_vector(m_axi_rdata);

  -- AXI write address channel --
  m_axi_awaddr  <= std_logic_vector(xbus_adr_i);
  m_axi_awlen   <= std_logic_vector(blen_c) when BURST_EN and (state(1) = '1') else (others => '0'); -- burst length
  m_axi_awsize  <= "010"; -- 4 bytes per transfer
  m_axi_awburst <= "01"; -- incrementing bursts only
  m_axi_awcache <= "0011"; -- recommended by Vivado
  m_axi_awprot  <= std_logic_vector(xbus_tag_i);
  m_axi_awvalid <= std_logic(awvalid);

  -- AXI write data channel --
  m_axi_wdata   <= std_logic_vector(xbus_dat_i);
  m_axi_wstrb   <= std_logic_vector(xbus_sel_i);
  m_axi_wlast   <= '1' when (xbus_cti_i = "000") else '0'; -- last word of transfer
  m_axi_wvalid  <= '1' when (wvalid = '1') or (BURST_EN and (state = "11") and (xbus_stb_i = '1')) else '0';

  -- AXI write response channel --
  m_axi_bready  <= '1'; -- always ready for write response
  xbus_wr_ack   <= '1' when (m_axi_bvalid = '1') and (m_axi_bresp(1) = '0') else '0'; -- OKAY(00)/EXOKAY(01)
  xbus_wr_err   <= '1' when (m_axi_bvalid = '1') and (m_axi_bresp(1) = '1') else '0'; -- SLVERR(10)/DECERR(11)

  -- XBUS response --
  xbus_ack_o    <= '1' when BURST_EN and (wb_ack = '1') else (xbus_rd_ack or xbus_wr_ack);
  xbus_err_o    <= '0' when BURST_EN and (wb_ack = '1') else (xbus_rd_err or xbus_wr_err);

end architecture;
