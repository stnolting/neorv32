-- ================================================================================ --
-- NEORV32 SoC - XBUS to AXI4-Lite Bridge (single non-overlapping transfers only)   --
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

  signal ready : std_ulogic_vector(2 downto 0);
  signal xbus_rd_ack, xbus_rd_err, xbus_wr_ack, xbus_wr_err : std_ulogic;

begin

  -- channel handshake arbiter --
  axi_handshake: process(resetn, clk)
  begin
    if (resetn = '0') then
      ready <= (others => '0');
    elsif rising_edge(clk) then
      ready(0) <= xbus_cyc_i and (ready(0) or std_ulogic(m_axi_arready)) and not std_ulogic(m_axi_rvalid);
      ready(1) <= xbus_cyc_i and (ready(1) or std_ulogic(m_axi_awready)) and not std_ulogic(m_axi_bvalid);
      ready(2) <= xbus_cyc_i and (ready(2) or std_ulogic(m_axi_wready)) and not std_ulogic(m_axi_bvalid);
    end if;
  end process axi_handshake;

  -- AXI read address channel --
  m_axi_araddr  <= std_logic_vector(xbus_adr_i);
  m_axi_arprot  <= std_logic_vector(xbus_tag_i);
  m_axi_arvalid <= std_logic(xbus_cyc_i and (not xbus_we_i) and (not ready(0)));

  -- AXI read data channel --
  m_axi_rready  <= '1';
  xbus_dat_o    <= std_ulogic_vector(m_axi_rdata);
  xbus_rd_ack   <= '1' when (m_axi_rvalid = '1') and (m_axi_rresp  = "00") else '0';
  xbus_rd_err   <= '1' when (m_axi_rvalid = '1') and (m_axi_rresp /= "00") else '0';

  -- AXI write address channel --
  m_axi_awaddr  <= std_logic_vector(xbus_adr_i);
  m_axi_awprot  <= std_logic_vector(xbus_tag_i);
  m_axi_awvalid <= std_logic(xbus_cyc_i and xbus_we_i and (not ready(1)));

  -- AXI write data channel --
  m_axi_wdata   <= std_logic_vector(xbus_dat_i);
  m_axi_wstrb   <= std_logic_vector(xbus_sel_i);
  m_axi_wvalid  <= std_logic(xbus_cyc_i and xbus_we_i and (not ready(2)));

  -- AXI write response channel --
  m_axi_bready  <= '1';
  xbus_wr_ack   <= '1' when (m_axi_bvalid = '1') and (m_axi_bresp  = "00") else '0';
  xbus_wr_err   <= '1' when (m_axi_bvalid = '1') and (m_axi_bresp /= "00") else '0';

  -- XBUS response --
  xbus_ack_o <= xbus_rd_ack when (xbus_we_i = '0') else xbus_wr_ack;
  xbus_err_o <= xbus_rd_err when (xbus_we_i = '0') else xbus_wr_err;

end architecture xbus2axi4lite_bridge_rtl;
