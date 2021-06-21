-- #################################################################################################
-- # << NEORV32 - Example setup including the bootloader, for the Fomu (c) Board >>                #
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

library iCE40;
use iCE40.components.all; -- for device primitives and macros

entity neorv32_Fomu_BoardTop_MixedLanguage is
  port (
    -- 48MHz Clock input
    clki : in std_logic;
    -- LED outputs
    rgb : out std_logic_vector(2 downto 0);
    -- USB Pins (which should be statically driven if not being used)
    usb_dp    : out std_logic;
    usb_dn    : out std_logic;
    usb_dp_pu : out std_logic
  );
end entity;

architecture neorv32_Fomu_BoardTop_MixedLanguage_rtl of neorv32_Fomu_BoardTop_MixedLanguage is

  -- configuration --
  constant f_clock_c : natural := 18000000; -- PLL output clock frequency in Hz

  component neorv32_Fomu_MixedLanguage_ClkGen
    port (
      clk_o  : out std_logic;
      rstn_o : out std_logic
    );
  end component;

  -- Globals
  signal pll_rstn : std_logic;
  signal pll_clk  : std_logic;

  -- internal IO connection --
  signal con_gpio_o : std_ulogic_vector(3 downto 0);
  signal con_pwm  : std_logic_vector(2 downto 0);

begin

  -- Assign USB pins to "0" so as to disconnect Fomu from
  -- the host system.  Otherwise it would try to talk to
  -- us over USB, which wouldn't work since we have no stack.
  usb_dp    <= '0';
  usb_dn    <= '0';
  usb_dp_pu <= '0';

  -- On-Chip HF Oscillator and System PLL -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  Clk_inst : neorv32_Fomu_MixedLanguage_ClkGen
  port map (
    clk_o  => pll_clk,
    rstn_o => pll_rstn
  );

  -- The core of the problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  neorv32_inst: entity work.neorv32_ProcessorTop_MinimalBoot
  generic map (
    CLOCK_FREQUENCY => f_clock_c  -- clock frequency of clk_i in Hz
  )
  port map (
    -- Global control --
    clk_i      => std_ulogic(pll_clk),
    rstn_i     => std_ulogic(pll_rstn),

    -- GPIO --
    gpio_o     => con_gpio_o,

    -- primary UART --
    uart_txd_o => open, -- UART0 send data
    uart_rxd_i => '0',  -- UART0 receive data
    uart_rts_o => open, -- hw flow control: UART0.RX ready to receive ("RTR"), low-active, optional
    uart_cts_i => '0',  -- hw flow control: UART0.TX allowed to transmit, low-active, optional

    -- PWM (to on-board RGB LED) --
    pwm_o      => con_pwm
  );

  -- IO Connection --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  RGB_inst: SB_RGBA_DRV
  generic map (
    CURRENT_MODE => "0b1",
    RGB0_CURRENT => "0b000011",
    RGB1_CURRENT => "0b000011",
    RGB2_CURRENT => "0b000011"
  )
  port map (
    CURREN   => '1',  -- I
    RGBLEDEN => '1',  -- I
    RGB2PWM  => con_pwm(2),                   -- I - blue  - pwm channel 2
    RGB1PWM  => con_pwm(1) or con_gpio_o(0),  -- I - red   - pwm channel 1 || BOOT blink
    RGB0PWM  => con_pwm(0),                   -- I - green - pwm channel 0
    RGB2     => rgb(2),  -- O - blue
    RGB1     => rgb(1),  -- O - red
    RGB0     => rgb(0)   -- O - green
  );

end architecture;
