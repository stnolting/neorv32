-- ================================================================================ --
-- NEORV32 SoC - Serial Memory Controller (SMC)                                     --
-- -------------------------------------------------------------------------------- --
-- Transparently maps up to two external SPI (PS)RAM/flash chips as linear memory   --
-- into the processor address space.                                                --
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

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_smc is
  generic (
    BURST_EN   : boolean;                       -- enable burst support
    BURST_SIZE : natural range 4 to 1024;       -- burst size in bytes, has to be a power of 2
    MEM_BASE   : std_ulogic_vector(31 downto 0) -- serial memory base address (256MB-aligned)
  );
  port (
    -- global control --
    clk_i      : in  std_ulogic;                    -- global clock line
    rstn_i     : in  std_ulogic;                    -- global reset line, low-active, async
    -- control and status register interface --
    ctrl_req_i : in  bus_req_t;                     -- bus request
    ctrl_rsp_o : out bus_rsp_t;                     -- bus response
    -- data interface --
    data_req_i : in  bus_req_t;                     -- bus request
    data_rsp_o : out bus_rsp_t;                     -- bus response
    -- memory interface --
    smc_ioen_o : out std_ulogic;                    -- SMC pin enable, can be used for IO multiplexing
    smc_sck_o  : out std_ulogic;                    -- clock
    smc_csn_o  : out std_ulogic_vector(1 downto 0); -- bank/chip select, low-active
    smc_sdo_o  : out std_ulogic;                    -- serial data output
    smc_sdi_i  : in  std_ulogic                     -- serial data input
  );
end neorv32_smc;

architecture neorv32_smc_rtl of neorv32_smc is

begin

  ctrl_rsp_o <= rsp_terminate_c;
  data_rsp_o <= rsp_terminate_c;
  smc_ioen_o <= '0';
  smc_sck_o  <= '0';
  smc_csn_o  <= "11";
  smc_sdo_o  <= '0';

end neorv32_smc_rtl;
