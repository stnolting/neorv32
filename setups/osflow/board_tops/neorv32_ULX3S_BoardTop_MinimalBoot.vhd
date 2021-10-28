-- #################################################################################################
-- # << NEORV32 - Example setup including the bootloader, for the ULX3S (c) Board >>               #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
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

library ECP5;
use ECP5.components.all; -- for device primitives and macros

entity neorv32_ULX3S_BoardTop_MinimalBoot is
  port (
    -- Clock and Reset inputs
    ULX3S_CLK : in std_logic;
    ULX3S_RST_N : in std_logic;
    -- LED outputs
    ULX3S_LED0 : out std_logic;
    ULX3S_LED1 : out std_logic;
    ULX3S_LED2 : out std_logic;
    ULX3S_LED3 : out std_logic;
    ULX3S_LED4 : out std_logic;
    ULX3S_LED5 : out std_logic;
    ULX3S_LED6 : out std_logic;
    ULX3S_LED7 : out std_logic;
    -- UART0
    ULX3S_RX : in  std_logic;
    ULX3S_TX : out std_logic
  );
end entity;

architecture neorv32_ULX3S_BoardTop_MinimalBoot_rtl of neorv32_ULX3S_BoardTop_MinimalBoot is

  -- configuration --
  constant f_clock_c : natural := 25000000; -- clock frequency in Hz

  -- internal IO connection --
  signal con_pwm    : std_logic_vector(2 downto 0);
  signal con_gpio_o : std_ulogic_vector(3 downto 0);

begin

  -- The core of the problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_inst: entity work.neorv32_ProcessorTop_MinimalBoot
  generic map (
    CLOCK_FREQUENCY   => f_clock_c, -- clock frequency of clk_i in Hz
    MEM_INT_IMEM_SIZE => 16*1024,
    MEM_INT_DMEM_SIZE => 8*1024
  )
  port map (
    -- Global control --
    clk_i      => std_ulogic(ULX3S_CLK),
    rstn_i     => std_ulogic(ULX3S_RST_N),
	
    -- GPIO --
    gpio_o     => con_gpio_o,

    -- primary UART --
    uart_txd_o => ULX3S_TX, -- UART0 send data
    uart_rxd_i => ULX3S_RX, -- UART0 receive data
    uart_rts_o => open, -- hw flow control: UART0.RX ready to receive ("RTR"), low-active, optional
    uart_cts_i => '0',  -- hw flow control: UART0.TX allowed to transmit, low-active, optional

    -- PWM (to on-board RGB LED) --
    pwm_o      => con_pwm
  );

  -- IO Connection --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ULX3S_LED0 <= con_gpio_o(0);
  ULX3S_LED1 <= con_gpio_o(1);
  ULX3S_LED2 <= con_gpio_o(2);
  ULX3S_LED3 <= con_gpio_o(3);
  ULX3S_LED4 <= '0'; -- unused
  ULX3S_LED5 <= con_pwm(0);
  ULX3S_LED6 <= con_pwm(1);
  ULX3S_LED7 <= con_pwm(2);

end architecture;
