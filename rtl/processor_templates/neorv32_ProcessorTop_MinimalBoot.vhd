-- #################################################################################################
-- # << NEORV32 - Minimal setup with the bootloader enabled >>                                     #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;

entity neorv32_ProcessorTop_MinimalBoot is
  generic (
    -- General --
    CLOCK_FREQUENCY   : natural := 0;       -- clock frequency of clk_i in Hz
    -- Internal Instruction memory --
    MEM_INT_IMEM_EN   : boolean := true;    -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE : natural := 64*1024; -- size of processor-internal instruction memory in bytes
    -- Internal Data memory --
    MEM_INT_DMEM_EN   : boolean := true;    -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE : natural := 64*1024; -- size of processor-internal data memory in bytes
    -- Processor peripherals --
    IO_GPIO_NUM       : natural := 0;       -- number of GPIO input/output pairs (0..64)
    IO_PWM_NUM_CH     : natural := 3        -- number of PWM channels to implement (0..12); 0 = disabled
  );
  port (
    -- Global control --
    clk_i      : in  std_logic;
    rstn_i     : in  std_logic;
    -- GPIO (available if IO_GPIO_EN = true) --
    gpio_o     : out std_ulogic_vector(3 downto 0);
    -- primary UART0 (available if IO_UART0_EN = true) --
    uart_txd_o : out std_ulogic; -- UART0 send data
    uart_rxd_i : in  std_ulogic := '0'; -- UART0 receive data
    -- PWM (available if IO_PWM_NUM_CH > 0) --
    pwm_o      : out std_ulogic_vector(IO_PWM_NUM_CH-1 downto 0)
  );
end entity;

architecture neorv32_ProcessorTop_MinimalBoot_rtl of neorv32_ProcessorTop_MinimalBoot is

  -- internal IO connection --
  signal con_gpio_o : std_ulogic_vector(63 downto 0);
  signal con_pwm_o  : std_ulogic_vector(11 downto 0);

begin

  -- The core of the problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_inst: entity neorv32.neorv32_top
  generic map (
    -- General --
    CLOCK_FREQUENCY   => CLOCK_FREQUENCY,   -- clock frequency of clk_i in Hz
    INT_BOOTLOADER_EN => true,              -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
    -- Internal Instruction memory --
    MEM_INT_IMEM_EN   => MEM_INT_IMEM_EN,   -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE => MEM_INT_IMEM_SIZE, -- size of processor-internal instruction memory in bytes
    -- Internal Data memory --
    MEM_INT_DMEM_EN   => MEM_INT_DMEM_EN,   -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE => MEM_INT_DMEM_SIZE, -- size of processor-internal data memory in bytes
    -- Processor peripherals --
    IO_GPIO_NUM       => IO_GPIO_NUM,       -- number of GPIO input/output pairs (0..64)
    IO_MTIME_EN       => true,              -- implement machine system timer (MTIME)?
    IO_UART0_EN       => true,              -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_PWM_NUM_CH     => IO_PWM_NUM_CH      -- number of PWM channels to implement (0..12); 0 = disabled
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
    -- PWM (available if IO_PWM_NUM_CH > 0) --
    pwm_o       => con_pwm_o                     -- pwm channels
  );

  -- GPIO --
  gpio_o <= con_gpio_o(3 downto 0);
  -- PWM --
  pwm_o <= con_pwm_o(IO_PWM_NUM_CH-1 downto 0);


end architecture;
