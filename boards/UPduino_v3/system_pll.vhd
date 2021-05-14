library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library iCE40UP;
use iCE40UP.components.all; -- for device primitives and macros

entity system_pll is
  port (
    ref_clk_i   : in  std_logic;
    rst_n_i     : in  std_logic;
    lock_o      : out std_logic;
    outcore_o   : out std_logic;
    outglobal_o : out std_logic
  );
end entity;


architecture rtl of system_pll is

begin

  Pll_inst : SB_PLL40_CORE
  port map (
    REFERENCECLK    => ref_clk_i,
    PLLOUTCORE      => outcore_o,
    PLLOUTGLOBAL    => outglobal_o,
    EXTFEEDBACK     => '0',
    DYNAMICDELAY    => x"00",
    LOCK            => lock_o,
    BYPASS          => '0',
    RESETB          => rst_n_i,
    LATCHINPUTVALUE => '0',
    SDO             => open,
    SDI             => '0',
    SCLK            => '0'
  );

end architecture;
