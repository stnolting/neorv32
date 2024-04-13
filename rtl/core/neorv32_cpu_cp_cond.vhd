-- ================================================================================ --
-- NEORV32 CPU - Co-Processor: RISC-V Cond. Operations ('Zicond') ISA Extension     --
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

entity neorv32_cpu_cp_cond is
  port (
    -- global control --
    clk_i   : in  std_ulogic; -- global clock, rising edge
    rstn_i  : in  std_ulogic; -- global reset, low-active, async
    ctrl_i  : in  ctrl_bus_t; -- main control bus
    start_i : in  std_ulogic; -- trigger operation
    -- data input --
    rs1_i   : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 1
    rs2_i   : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 2
    -- result and status --
    res_o   : out std_ulogic_vector(XLEN-1 downto 0); -- operation result
    valid_o : out std_ulogic -- data output valid
  );
end neorv32_cpu_cp_cond;

architecture neorv32_cpu_cp_cond_rtl of neorv32_cpu_cp_cond is

  signal rs2_zero, condition : std_ulogic;

begin

  -- conditional output --
  cond_out: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      res_o <= (others => '0');
    elsif rising_edge(clk_i) then
      if (start_i = '1') and (condition = '1') then -- unit triggered and condition true
        res_o <= rs1_i;
      else
        res_o <= (others => '0');
      end if;
    end if;
  end process cond_out;

  -- condition check --
  rs2_zero  <= '1' when (or_reduce_f(rs2_i) = '0') else '0';
  condition <= rs2_zero xnor ctrl_i.ir_funct3(1); -- equal zero / non equal zero

  -- processing done --
  valid_o <= start_i;


end neorv32_cpu_cp_cond_rtl;
