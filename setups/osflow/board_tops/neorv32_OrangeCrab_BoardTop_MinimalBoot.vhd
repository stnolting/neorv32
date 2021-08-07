-- #################################################################################################
-- # << NEORV32 - Example setup including the bootloader, for the OrangeCrab (c) Board >>          #
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

entity neorv32_OrangeCrab_BoardTop_MinimalBoot is
  port (
    -- Clock and Reset inputs
    OrangeCrab_CLK : in std_logic;
    OrangeCrab_RST_N : in std_logic;
    -- LED outputs
    OrangeCrab_LED_RGB_R : out std_logic;
    OrangeCrab_LED_RGB_G : out std_logic;
    OrangeCrab_LED_RGB_B : out std_logic;
    -- UART0
    OrangeCrab_GPIO_0 : in  std_logic;
    OrangeCrab_GPIO_1 : out std_logic;
    OrangeCrab_GPIO_9 : out std_logic;
    -- USB Pins (which should be statically driven if not being used)
    OrangeCrab_USB_D_P   : out std_logic;
    OrangeCrab_USB_D_N   : out std_logic;
    OrangeCrab_USB_DP_PU : out std_logic
  );
end entity;

architecture neorv32_OrangeCrab_BoardTop_MinimalBoot_rtl of neorv32_OrangeCrab_BoardTop_MinimalBoot is

  -- configuration --
  constant f_clock_c : natural := 24000000; -- PLL output clock frequency in Hz

  -- Globals
  signal pll_clk: std_logic;

  -- internal IO connection --
  signal con_pwm    : std_logic_vector(2 downto 0);
  signal con_gpio_o : std_ulogic_vector(3 downto 0);

begin

  -- Assign USB pins to "0" so as to disconnect OrangeCrab from
  -- the host system.  Otherwise it would try to talk to
  -- us over USB, which wouldn't work since we have no stack.
  OrangeCrab_USB_D_P   <= '0';
  OrangeCrab_USB_D_N   <= '0';
  OrangeCrab_USB_DP_PU <= '0';

  -- System PLL -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  PLL_inst: EHXPLLL
  generic map (
    CLKI_DIV  =>  2, -- from `ecppll -i 48 -o 24`
    CLKFB_DIV =>  1,
    CLKOP_DIV =>  25
  )
  port map (
    CLKI    => OrangeCrab_CLK,
    CLKFB   => pll_clk,
    ENCLKOP => '1',
    CLKOP   => pll_clk,
    LOCK    => OrangeCrab_GPIO_9
  );

  -- The core of the problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  neorv32_inst: entity work.neorv32_ProcessorTop_MinimalBoot
  generic map (
    CLOCK_FREQUENCY => f_clock_c,  -- clock frequency of clk_i in Hz
    MEM_INT_IMEM_SIZE => 16*1024,
    MEM_INT_DMEM_SIZE => 8*1024
  )
  port map (
    -- Global control --
    clk_i      => std_ulogic(pll_clk),
    rstn_i     => std_ulogic(OrangeCrab_RST_N),

    -- GPIO --
    gpio_o     => con_gpio_o,

    -- primary UART --
    uart_txd_o => OrangeCrab_GPIO_1, -- UART0 send data
    uart_rxd_i => OrangeCrab_GPIO_0, -- UART0 receive data
    uart_rts_o => open, -- hw flow control: UART0.RX ready to receive ("RTR"), low-active, optional
    uart_cts_i => '0',  -- hw flow control: UART0.TX allowed to transmit, low-active, optional

    -- PWM (to on-board RGB LED) --
    pwm_o      => con_pwm
  );

  OrangeCrab_LED_RGB_R <= con_pwm(0) or not con_gpio_o(0);
  OrangeCrab_LED_RGB_G <= con_pwm(1);
  OrangeCrab_LED_RGB_B <= con_pwm(2);

end architecture;
