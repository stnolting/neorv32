-- ================================================================================ --
-- NEORV32 SoC - XBUS to AHB3-Lite Bridge (single non-overlapping transfers only)   --
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

entity xbus2ahblite_bridge is
  port (
    -- global control --
    clk_i        : in  std_ulogic;                     -- global clock line
    rstn_i       : in  std_ulogic;                     -- global reset line, low-active, use as async
    -- xbus device interface --
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
    -- ahb-lite host interface --
    ahb_haddr_o  : out std_ulogic_vector(31 downto 0); -- address
    ahb_hwdata_o : out std_ulogic_vector(31 downto 0); -- write data
    ahb_hwrite_o : out std_ulogic;                     -- read/write
    ahb_hsize_o  : out std_ulogic_vector(2 downto 0);  -- transfer size
    ahb_hburst_o : out std_ulogic_vector(2 downto 0);  -- burst type
    ahb_hprot_o  : out std_ulogic_vector(3 downto 0);  -- protection control
    ahb_htrans_o : out std_ulogic_vector(1 downto 0);  -- transfer type
    ahb_hready_i : in  std_ulogic;                     -- transfer completed
    ahb_hresp_i  : in  std_ulogic;                     -- transfer response
    ahb_hrdata_i : in  std_ulogic_vector(31 downto 0)  -- read data
  );
end xbus2ahblite_bridge;

architecture xbus2ahblite_bridge_rtl of xbus2ahblite_bridge is

  -- arbiter --
  signal addr_ack_q : std_ulogic; -- address phase transfer completed
  signal pending_q  : std_ulogic; -- pending bus transaction (bus is in "data phase")

begin

  -- access arbiter --
  arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      addr_ack_q <= '0';
      pending_q  <= '0';
    elsif rising_edge(clk_i) then
      if (pending_q = '0') then -- idle (also AHB address phase)
        addr_ack_q <= ahb_hready_i; -- sample HREADY in address phase
        if (xbus_stb_i = '1') then
          pending_q <= '1';
        end if;
      else -- transfer in progress (AHB data phase)
        -- complete if HREADY has acknowledged address phase and is acknowledging data phase
        -- abort if core terminated the transfer by pulling CYC low
        if ((addr_ack_q = '1') and (ahb_hready_i = '1')) or (xbus_cyc_i = '0') then
          addr_ack_q <= '0';
          pending_q  <= '0';
        end if;
      end if;
    end if;
  end process arbiter;

  -- host response: evaluate in data phase --
  xbus_ack_o <= '1' when (addr_ack_q = '1') and (pending_q = '1') and (ahb_hready_i = '1') and (ahb_hresp_i = '0') else '0'; -- okay
  xbus_err_o <= '1' when (addr_ack_q = '1') and (pending_q = '1') and (ahb_hready_i = '1') and (ahb_hresp_i = '1') else '0'; -- error

  -- host request: NONSEQ during address phase, IDLE during data phase --
  ahb_htrans_o <= "10" when (xbus_stb_i = '1') else "00";

  -- protection control --
  ahb_hprot_o(3) <= '0'; -- non-cacheable
  ahb_hprot_o(2) <= '0'; -- non-bufferable
  ahb_hprot_o(1) <= xbus_tag_i(0); -- 0 = user-access, 1 = privileged access
  ahb_hprot_o(0) <= not xbus_tag_i(2); -- 0 = instruction fetch, 1 = data access

  -- burst control --
  ahb_hburst_o <= "000"; -- single burst

  -- read/write --
  ahb_hwrite_o <= xbus_we_i;

  -- address --
  ahb_haddr_o <= xbus_adr_i;

  -- data --
  ahb_hwdata_o <= xbus_dat_i;
  xbus_dat_o   <= ahb_hrdata_i;

  -- data quantity --
  with xbus_sel_i select ahb_hsize_o <=
    "000" when "1000" | "0100" | "0010" | "0001", -- byte
    "001" when "1100" | "0011", -- half-word (aligned only)
    "010" when others; -- word

end xbus2ahblite_bridge_rtl;
