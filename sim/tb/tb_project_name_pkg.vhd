--==============================================================================
--  Package:       tb_project_name_pkg
--  Description:   Testbench utilities for module
--==============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package tb_project_name_pkg is

    ----------------------------------------------------------------------------
    -- Testbench Constants
    ----------------------------------------------------------------------------

    constant CLK_PERIOD : time := 10 ns;

    ----------------------------------------------------------------------------
    -- Procedures
    ----------------------------------------------------------------------------

    procedure clk_wait(signal clk : in std_logic; cycles : natural);

end package tb_project_name_pkg;


package body tb_project_name_pkg is

    ----------------------------------------------------------------------------
    -- Procedures
    ----------------------------------------------------------------------------

    procedure clk_wait(signal clk : in std_logic; cycles : natural) is
    begin
        for i in 1 to cycles loop
            wait until rising_edge(clk);
        end loop;
    end procedure;

end package body tb_project_name_pkg;