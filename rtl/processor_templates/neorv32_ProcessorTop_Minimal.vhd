-- ================================================================================ --
-- NEORV32 Templates - Minimal setup without a bootloader                           --
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

entity neorv32_ProcessorTop_Minimal is
  generic (
    -- Clocking --
    CLOCK_FREQUENCY : natural := 0;       -- clock frequency of clk_i in Hz
    -- Internal Instruction memory --
    IMEM_EN         : boolean := true;    -- implement processor-internal instruction memory
    IMEM_SIZE       : natural := 8*1024;  -- size of processor-internal instruction memory in bytes
    -- Internal Data memory --
    DMEM_EN         : boolean := true;    -- implement processor-internal data memory
    DMEM_SIZE       : natural := 64*1024; -- size of processor-internal data memory in bytes
    -- Processor peripherals --
    IO_PWM_NUM      : natural := 3        -- number of PWM channels to implement (0..32)
  );
  port (
    -- Global control --
    clk_i  : in  std_logic;
    rstn_i : in  std_logic;
    -- PWM (available if IO_PWM_NUM > 0) --
    pwm_o  : out std_ulogic_vector(IO_PWM_NUM-1 downto 0)
  );
end entity;

architecture neorv32_ProcessorTop_Minimal_rtl of neorv32_ProcessorTop_Minimal is

  -- internal IO connection --
  signal con_pwm_o : std_ulogic_vector(31 downto 0);

begin

  -- The core of the problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_inst: entity neorv32.neorv32_top
  generic map (
    -- Clocking --
    CLOCK_FREQUENCY  => CLOCK_FREQUENCY, -- clock frequency of clk_i in Hz
    -- Boot Configuration --
    BOOT_MODE_SELECT => 2,               -- boot from pre-initialized internal IMEM
    -- RISC-V CPU Extensions --
    RISCV_ISA_Zicntr => true,            -- implement base counters?
    -- Internal Instruction memory --
    IMEM_EN          => IMEM_EN,         -- implement processor-internal instruction memory
    IMEM_SIZE        => IMEM_SIZE,       -- size of processor-internal instruction memory in bytes
    -- Internal Data memory --
    DMEM_EN          => DMEM_EN,         -- implement processor-internal data memory
    DMEM_SIZE        => DMEM_SIZE,       -- size of processor-internal data memory in bytes
    -- Processor peripherals --
    IO_CLINT_EN      => true,            -- implement core local interruptor (CLINT)?
    IO_PWM_NUM       => IO_PWM_NUM       -- number of PWM channels to implement (0..32); 0 = disabled
  )
  port map (
    -- Global control --
    clk_i  => clk_i,    -- global clock, rising edge
    rstn_i => rstn_i,   -- global reset, low-active, async
    -- PWM (available if IO_PWM_NUM > 0) --
    pwm_o  => con_pwm_o -- pwm channels
  );

  -- PWM --
  pwm_o <= con_pwm_o(IO_PWM_NUM-1 downto 0);


end architecture;
