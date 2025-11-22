-- ================================================================================ --
-- NEORV32 Templates - Minimal generic setup with the bootloader enabled            --
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

entity neorv32_ProcessorTop_MinimalBoot is
  generic (
    -- Clocking --
    CLOCK_FREQUENCY : natural := 0;       -- clock frequency of clk_i in Hz
    -- Internal Instruction memory --
    IMEM_EN         : boolean := true;    -- implement processor-internal instruction memory
    IMEM_SIZE       : natural := 64*1024; -- size of processor-internal instruction memory in bytes
    -- Internal Data memory --
    DMEM_EN         : boolean := true;    -- implement processor-internal data memory
    DMEM_SIZE       : natural := 64*1024; -- size of processor-internal data memory in bytes
    -- Processor peripherals --
    IO_GPIO_NUM     : natural := 4;       -- number of GPIO input/output pairs (0..32)
    IO_PWM_NUM      : natural := 3        -- number of PWM channels to implement (0..32)
  );
  port (
    -- Global control --
    clk_i      : in  std_logic;
    rstn_i     : in  std_logic;
    -- GPIO (available if IO_GPIO_EN = true) --
    gpio_o     : out std_ulogic_vector(IO_GPIO_NUM-1 downto 0);
    -- primary UART0 (available if IO_UART0_EN = true) --
    uart_txd_o : out std_ulogic; -- UART0 send data
    uart_rxd_i : in  std_ulogic := '0'; -- UART0 receive data
    -- PWM (available if IO_PWM_NUM > 0) --
    pwm_o      : out std_ulogic_vector(IO_PWM_NUM-1 downto 0)
  );
end entity;

architecture neorv32_ProcessorTop_MinimalBoot_rtl of neorv32_ProcessorTop_MinimalBoot is

  -- internal IO connection --
  signal con_gpio_o, con_pwm_o : std_ulogic_vector(31 downto 0);

begin

  -- The core of the problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_inst: entity neorv32.neorv32_top
  generic map (
    -- Clocking --
    CLOCK_FREQUENCY  => CLOCK_FREQUENCY, -- clock frequency of clk_i in Hz
    -- Boot Configuration --
    BOOT_MODE_SELECT => 0,               -- boot via internal bootloader
    -- RISC-V CPU Extensions --
    RISCV_ISA_Zicntr => true,            -- implement base counters?
    -- Internal Instruction memory --
    IMEM_EN          => IMEM_EN,         -- implement processor-internal instruction memory
    IMEM_SIZE        => IMEM_SIZE,       -- size of processor-internal instruction memory in bytes
    -- Internal Data memory --
    DMEM_EN          => DMEM_EN,         -- implement processor-internal data memory
    DMEM_SIZE        => DMEM_SIZE,       -- size of processor-internal data memory in bytes
    -- Processor peripherals --
    IO_GPIO_NUM      => IO_GPIO_NUM,     -- number of GPIO input/output pairs (0..32)
    IO_CLINT_EN      => true,            -- implement core local interruptor (CLINT)?
    IO_UART0_EN      => true,            -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_PWM_NUM       => IO_PWM_NUM       -- number of PWM channels to implement (0..32); 0 = disabled
  )
  port map (
    -- Global control --
    clk_i       => clk_i,                        -- global clock, rising edge
    rstn_i      => rstn_i,                       -- global reset, low-active, async
    -- GPIO (available if IO_GPIO_NUM > 0) --
    gpio_o      => con_gpio_o,                   -- parallel output
    gpio_i      => (others => '0'),              -- parallel input
    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o => uart_txd_o,                   -- UART0 send data
    uart0_rxd_i => uart_rxd_i,                   -- UART0 receive data
    -- PWM (available if IO_PWM_NUM > 0) --
    pwm_o       => con_pwm_o                     -- pwm channels
  );

  -- GPIO --
  gpio_o <= con_gpio_o(IO_GPIO_NUM-1 downto 0);

  -- PWM --
  pwm_o <= con_pwm_o(IO_PWM_NUM-1 downto 0);


end architecture;
