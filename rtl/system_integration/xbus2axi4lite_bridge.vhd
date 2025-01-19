-- ================================================================================ --
-- NEORV32 SoC - XBUS to AXI4-Lite Bridge                                           --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;

entity xbus2axi4lite_bridge is
  port (
    -- Global control --
    clk           : in  std_logic;
    resetn        : in  std_logic;
    -- XBUS device interface --
    xbus_adr_i    : in  std_ulogic_vector(31 downto 0);
    xbus_dat_i    : in  std_ulogic_vector(31 downto 0);
    xbus_tag_i    : in  std_ulogic_vector(2 downto 0);
    xbus_we_i     : in  std_ulogic;
    xbus_sel_i    : in  std_ulogic_vector(3 downto 0);
    xbus_stb_i    : in  std_ulogic;
    xbus_cyc_i    : in  std_ulogic;
    xbus_ack_o    : out std_ulogic;
    xbus_err_o    : out std_ulogic;
    xbus_dat_o    : out std_ulogic_vector(31 downto 0);
    -- AXI4-Lite host write address channel --
    m_axi_awaddr  : out std_logic_vector(31 downto 0);
    m_axi_awprot  : out std_logic_vector(2 downto 0);
    m_axi_awvalid : out std_logic;
    m_axi_awready : in  std_logic;
    -- AXI4-Lite host write data channel --
    m_axi_wdata   : out std_logic_vector(31 downto 0);
    m_axi_wstrb   : out std_logic_vector(3 downto 0);
    m_axi_wvalid  : out std_logic;
    m_axi_wready  : in  std_logic;
    -- AXI4-Lite host read address channel --
    m_axi_araddr  : out std_logic_vector(31 downto 0);
    m_axi_arprot  : out std_logic_vector(2 downto 0);
    m_axi_arvalid : out std_logic;
    m_axi_arready : in  std_logic;
    -- AXI4-Lite host read data channel --
    m_axi_rdata   : in  std_logic_vector(31 downto 0);
    m_axi_rresp   : in  std_logic_vector(1 downto 0);
    m_axi_rvalid  : in  std_logic;
    m_axi_rready  : out std_logic;
    -- AXI4-Lite host write response channel --
    m_axi_bresp   : in  std_logic_vector(1 downto 0);
    m_axi_bvalid  : in  std_logic;
    m_axi_bready  : out std_logic
  );
end entity;

architecture xbus2axi4lite_bridge_rtl of xbus2axi4lite_bridge is

  signal arvalid, awvalid, wvalid, xbus_rd_ack, xbus_rd_err, xbus_wr_ack, xbus_wr_err : std_ulogic;

begin

  -- channel handshake arbiter --
  axi_handshake: process(resetn, clk)
  begin
    if (resetn = '0') then
      arvalid <= '0';
      awvalid <= '0';
      wvalid  <= '0';
    elsif rising_edge(clk) then
      --         /------------- Set ------------\    /-------- Hold -------\     /---------- Clear ----------\
      arvalid <= (xbus_stb_i and (not xbus_we_i)) or (arvalid and xbus_cyc_i and std_ulogic(not m_axi_arready));
      awvalid <= (xbus_stb_i and (    xbus_we_i)) or (awvalid and xbus_cyc_i and std_ulogic(not m_axi_awready));
      wvalid  <= (xbus_stb_i and (    xbus_we_i)) or (wvalid  and xbus_cyc_i and std_ulogic(not m_axi_wready));
    end if;
  end process axi_handshake;

  -- AXI read address channel --
  m_axi_araddr  <= std_logic_vector(xbus_adr_i);
  m_axi_arprot  <= std_logic_vector(xbus_tag_i);
  m_axi_arvalid <= std_logic(arvalid);

  -- AXI read data channel --
  m_axi_rready  <= '1';
  xbus_dat_o    <= std_ulogic_vector(m_axi_rdata);
  xbus_rd_ack   <= '1' when (m_axi_rvalid = '1') and (m_axi_rresp  = "00") else '0';
  xbus_rd_err   <= '1' when (m_axi_rvalid = '1') and (m_axi_rresp /= "00") else '0';

  -- AXI write address channel --
  m_axi_awaddr  <= std_logic_vector(xbus_adr_i);
  m_axi_awprot  <= std_logic_vector(xbus_tag_i);
  m_axi_awvalid <= std_logic(awvalid);

  -- AXI write data channel --
  m_axi_wdata   <= std_logic_vector(xbus_dat_i);
  m_axi_wstrb   <= std_logic_vector(xbus_sel_i);
  m_axi_wvalid  <= std_logic(wvalid);

  -- AXI write response channel --
  m_axi_bready  <= '1';
  xbus_wr_ack   <= '1' when (m_axi_bvalid = '1') and (m_axi_bresp  = "00") else '0';
  xbus_wr_err   <= '1' when (m_axi_bvalid = '1') and (m_axi_bresp /= "00") else '0';

  -- XBUS response --
  xbus_ack_o <= xbus_rd_ack or xbus_wr_ack;
  xbus_err_o <= xbus_rd_err or xbus_wr_err;

end architecture xbus2axi4lite_bridge_rtl;
