-- ================================================================================ --
-- NEORV32 - Test Setup Using The Internal IMEM To Run Pre-Installed Executables    --
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

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_test_setup_approm is
  generic (
    -- adapt these for your setup --
    CLOCK_FREQUENCY : natural := 100000000; -- clock frequency of clk_i in Hz
    IMEM_SIZE       : natural := 16*1024;   -- size of processor-internal instruction memory in bytes
    DMEM_SIZE       : natural := 8*1024     -- size of processor-internal data memory in bytes
  );
  port (
    -- Global control --
    clk_i  : in  std_ulogic; -- global clock, rising edge
    rstn_i : in  std_ulogic; -- global reset, low-active, async
    -- GPIO --
    gpio_o : out std_ulogic_vector(7 downto 0) -- parallel output
  );
end entity;

architecture neorv32_test_setup_approm_rtl of neorv32_test_setup_approm is

  signal con_gpio_out : std_ulogic_vector(31 downto 0);

begin

  -- The Core Of The Problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_top_inst: neorv32_top
  generic map (
    -- Clocking --
    CLOCK_FREQUENCY  => CLOCK_FREQUENCY, -- clock frequency of clk_i in Hz
    -- Boot Configuration --             
    BOOT_MODE_SELECT => 2,               -- boot from pre-initialized IMEM
    -- RISC-V CPU Extensions --          
    RISCV_ISA_C      => true,            -- implement compressed extension?
    RISCV_ISA_M      => true,            -- implement mul/div extension?
    RISCV_ISA_Zicntr => true,            -- implement base counters?
    -- Internal Instruction memory --    
    IMEM_EN          => true,            -- implement processor-internal instruction memory
    IMEM_SIZE        => IMEM_SIZE,       -- size of processor-internal instruction memory in bytes
    -- Internal Data memory --           
    DMEM_EN          => true,            -- implement processor-internal data memory
    DMEM_SIZE        => DMEM_SIZE,       -- size of processor-internal data memory in bytes
    -- Processor peripherals --          
    IO_GPIO_NUM      => 8,               -- number of GPIO input/output pairs (0..32)
    IO_CLINT_EN      => true             -- implement core local interruptor (CLINT)?
  )
  port map (
    -- Global control --
    clk_i  => clk_i,       -- global clock, rising edge
    rstn_i => rstn_i,      -- global reset, low-active, async
    -- GPIO (available if IO_GPIO_NUM > 0) --
    gpio_o => con_gpio_out -- parallel output
  );

  -- GPIO output --
  gpio_o <= con_gpio_out(7 downto 0);


end architecture;
