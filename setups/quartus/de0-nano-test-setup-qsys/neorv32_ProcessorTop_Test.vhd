library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
library work;

entity neorv32_ProcessorTop_Test is port (
    clk_i                   : in std_logic;
    rstn_i                  : in std_logic;
    gpio_o                  : out std_logic_vector(7 downto 0);
    uart0_txd_o             : out std_logic;
    uart0_rxd_i             : in std_logic);
end neorv32_ProcessorTop_Test;

----------------------------------------------------------------------------------------------------
architecture rtl of neorv32_ProcessorTop_Test is
----------------------------------------------------------------------------------------------------

component neorv32_test_qsys is
port (
    clk_clk                 : in std_logic;
    perf_uart0_uart0_txd_o  : out std_logic;
    perf_uart0_uart0_rxd_i  : in std_logic;
    perf_gpio_gpio_o        : out std_logic_vector(31 downto 0);
    perf_gpio_gpio_i        : in std_logic_vector(31 downto 0);
    reset_reset_n           : in std_logic);
end component;

signal  perf_gpio_gpio_o    : std_logic_vector(31 downto 0);

begin

    gpio_o <= perf_gpio_gpio_o(7 downto 0);

    my_riscv_core : neorv32_test_qsys
    port map (
        clk_clk => clk_i,
        perf_gpio_gpio_o => perf_gpio_gpio_o,
        perf_gpio_gpio_i => (others => '0'),
        perf_uart0_uart0_txd_o => uart0_txd_o,
        perf_uart0_uart0_rxd_i => uart0_rxd_i,
        reset_reset_n => rstn_i);

end rtl;
