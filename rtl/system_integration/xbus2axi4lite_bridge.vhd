-- ================================================================================ --
-- NEORV32 SoC - XBUS to AXI4-Lite Bridge (non-overlapping single transfers only)   --
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
    -- ------------------------------------------------------------
    -- Global Control
    -- ------------------------------------------------------------
    clk            : in  std_logic;
    resetn         : in  std_logic; -- low-active
    -- ------------------------------------------------------------
    -- XBUS Device Interface
    -- ------------------------------------------------------------
    xbus_adr_i   : in  std_ulogic_vector(31 downto 0); -- address
    xbus_dat_i   : in  std_ulogic_vector(31 downto 0); -- write data
    xbus_tag_i   : in  std_ulogic_vector(2 downto 0);  -- access tag
    xbus_we_i    : in  std_ulogic;                     -- read/write
    xbus_sel_i   : in  std_ulogic_vector(3 downto 0);  -- byte enable
    xbus_stb_i   : in  std_ulogic;                     -- strobe
    xbus_cyc_i   : in  std_ulogic;                     -- valid cycle
    xbus_ack_o   : out std_ulogic;                     -- transfer acknowledge
    xbus_err_o   : out std_ulogic;                     -- transfer error
    xbus_dat_o   : out std_ulogic_vector(31 downto 0); -- read data
    -- ------------------------------------------------------------
    -- AXI4-Lite Host Interface
    -- ------------------------------------------------------------
    -- Clock and Reset --
--  m_axi_aclk     : in  std_logic := '0'; -- just to satisfy Vivado, but not actually used
--  m_axi_aresetn  : in  std_logic := '0'; -- just to satisfy Vivado, but not actually used
    -- Write Address Channel --
    m_axi_awaddr   : out std_logic_vector(31 downto 0);
    m_axi_awprot   : out std_logic_vector(2 downto 0);
    m_axi_awvalid  : out std_logic;
    m_axi_awready  : in  std_logic := '0';
    -- Write Data Channel --
    m_axi_wdata    : out std_logic_vector(31 downto 0);
    m_axi_wstrb    : out std_logic_vector(3 downto 0);
    m_axi_wvalid   : out std_logic;
    m_axi_wready   : in  std_logic := '0';
    -- Read Address Channel --
    m_axi_araddr   : out std_logic_vector(31 downto 0);
    m_axi_arprot   : out std_logic_vector(2 downto 0);
    m_axi_arvalid  : out std_logic;
    m_axi_arready  : in  std_logic := '0';
    -- Read Data Channel --
    m_axi_rdata    : in  std_logic_vector(31 downto 0) := x"00000000";
    m_axi_rresp    : in  std_logic_vector(1 downto 0) := "11"; -- error by default
    m_axi_rvalid   : in  std_logic := '0';
    m_axi_rready   : out std_logic;
    -- Write Response Channel --
    m_axi_bresp    : in  std_logic_vector(1 downto 0) := "11"; -- error by default
    m_axi_bvalid   : in  std_logic := '0';
    m_axi_bready   : out std_logic
  );
end entity;

architecture xbus2axi4lite_bridge_rtl of xbus2axi4lite_bridge is

  -- AXI bridge control --
  signal axi_radr_received, axi_wadr_received, axi_wdat_received : std_ulogic;

begin

  -- Wishbone-to-AXI4-Lite Bridge -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  axi_arbiter: process(resetn, clk)
  begin
    if (resetn = '0') then
      axi_radr_received <= '0';
      axi_wadr_received <= '0';
      axi_wdat_received <= '0';
    elsif rising_edge(clk) then
      if (xbus_cyc_i = '0') then
        axi_radr_received <= '0';
        axi_wadr_received <= '0';
        axi_wdat_received <= '0';
      else -- pending access
        if (xbus_we_i = '0') then -- read
          if (m_axi_arready = '1') then -- read address received by interconnect?
            axi_radr_received <= '1';
          end if;
        else -- write
          if (m_axi_awready = '1') then -- write address received by interconnect?
            axi_wadr_received <= '1';
          end if;
          if (m_axi_wready = '1') then -- write data received by interconnect?
            axi_wdat_received <= '1';
          end if;
        end if;
      end if;
    end if;
  end process axi_arbiter;


  -- read address channel --
  m_axi_araddr  <= std_logic_vector(xbus_adr_i);
  m_axi_arprot  <= std_logic_vector(xbus_tag_i);
  m_axi_arvalid <= std_logic(xbus_cyc_i and (not xbus_we_i) and (not axi_radr_received));

  -- read data channel --
  m_axi_rready  <= std_logic(xbus_cyc_i and (not xbus_we_i));
  xbus_dat_o    <= std_ulogic_vector(m_axi_rdata);

  -- write address channel --
  m_axi_awaddr  <= std_logic_vector(xbus_adr_i);
  m_axi_awprot  <= std_logic_vector(xbus_tag_i);
  m_axi_awvalid <= std_logic(xbus_cyc_i and xbus_we_i and (not axi_wadr_received));

  -- write data channel --
  m_axi_wdata   <= std_logic_vector(xbus_dat_i);
  m_axi_wstrb   <= std_logic_vector(xbus_sel_i);
  m_axi_wvalid  <= std_logic(xbus_cyc_i and xbus_we_i and (not axi_wdat_received));

  -- write response channel --
  m_axi_bready  <= std_logic(xbus_cyc_i and xbus_we_i);


  -- read/write response --
  axi_response: process(xbus_we_i, m_axi_bvalid, m_axi_bresp, m_axi_rvalid, m_axi_rresp)
  begin
    xbus_ack_o <= '0'; -- default
    xbus_err_o <= '0'; -- default
    if (xbus_we_i = '1') then -- write operation
      if (m_axi_bvalid = '1') then -- valid write response
        if (m_axi_bresp = "00") then -- status check
          xbus_ack_o <= '1'; -- OK
        else
          xbus_err_o <= '1'; -- ERROR
        end if;
      end if;
    else -- read operation
      if (m_axi_rvalid = '1') then -- valid read response
        if (m_axi_rresp = "00") then -- status check
          xbus_ack_o <= '1'; -- OK
        else
          xbus_err_o <= '1'; -- ERROR
        end if;
      end if;
    end if;
  end process axi_response;

end architecture xbus2axi4lite_bridge_rtl;
