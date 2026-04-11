--==============================================================================
--  Testbench:     tb_project_name
--  Description:   Testbench for project_name
--==============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.project_name_pkg.all;
use work.tb_project_name_pkg.all;

entity tb_project_name is
end entity;

architecture tb of tb_project_name is

    ----------------------------------------------------------------------------
    -- DUT Signals
    ----------------------------------------------------------------------------

    signal tb_clk     : std_logic := '0';
    signal tb_rst     : std_logic := '1';

    signal tb_data_i  : std_logic_vector(DATA_WIDTH-1 downto 0) := (others => '0');
    signal tb_valid_i : std_logic := '0';

    signal tb_data_o  : std_logic_vector(DATA_WIDTH-1 downto 0);
    signal tb_valid_o : std_logic;

begin

    ----------------------------------------------------------------------------
    -- Clock generation
    ----------------------------------------------------------------------------

    tb_clk <= not tb_clk after CLK_PERIOD/2;

    ----------------------------------------------------------------------------
    -- DUT Instantiation
    ----------------------------------------------------------------------------

    dut : entity work.project_name
        generic map (
            G_DATA_WIDTH => DATA_WIDTH
        )
        port map (
            -- clk & rst
            clk     => tb_clk,
            rst     => tb_rst,
            -- input
            data_i  => tb_data_i,
            valid_i => tb_valid_i,
            -- output
            data_o  => tb_data_o,
            valid_o => tb_valid_o
        );

    ----------------------------------------------------------------------------
    -- Stimulus process
    ----------------------------------------------------------------------------

    p_stimulus : process
    begin

        ------------------------------------------------------------------------
        -- Reset
        ------------------------------------------------------------------------

        tb_rst <= '1';
        wait for 50 ns;
        tb_rst <= '0';

        ------------------------------------------------------------------------
        -- Test case 1
        ------------------------------------------------------------------------

        tb_data_i  <= x"00000001";
        tb_valid_i <= '1';

        wait for 300 ns;
        tb_valid_i <= '0';

        ------------------------------------------------------------------------
        -- End simulation
        ------------------------------------------------------------------------
        
        wait for 500 ns;
        assert false report "Simulation finished" severity failure;
        wait;

    end process;

end architecture tb;