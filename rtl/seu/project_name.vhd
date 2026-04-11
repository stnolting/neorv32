--==============================================================================
--  Module:        module
--  Project:       <project_name>
--  Author:        <author_name>
--  Created:       <date>
--  Last Modified: <date>
--
--  Description:
--  <Explain the purpose of the module>
--
--  Dependencies:
--      project_name.vhd
--
--==============================================================================

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY work;
USE work.project_name_pkg.ALL;

ENTITY project_name IS
    GENERIC (
        G_DATA_WIDTH : integer := DATA_WIDTH
    );
    PORT (
        ------------------------------------------------------------------------
        -- Clock / Reset
        ------------------------------------------------------------------------
        CLK : IN std_logic;
        RST : IN std_logic;
        ------------------------------------------------------------------------
        -- Inputs
        ------------------------------------------------------------------------
        DATA_I  : IN std_logic_vector(G_DATA_WIDTH - 1 DOWNTO 0);
        VALID_I : IN std_logic;
        ------------------------------------------------------------------------
        -- Outputs
        ------------------------------------------------------------------------
        DATA_O  : OUT std_logic_vector(G_DATA_WIDTH - 1 DOWNTO 0);
        VALID_O : OUT std_logic
    );
END ENTITY project_name;

ARCHITECTURE rtl OF project_name IS

    ----------------------------------------------------------------------------
    -- Constants
    ----------------------------------------------------------------------------

    CONSTANT C_INTERNAL_WIDTH : integer := G_DATA_WIDTH;

    ----------------------------------------------------------------------------
    -- Types
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Signals
    ----------------------------------------------------------------------------

    SIGNAL state      : t_state;
    SIGNAL next_state : t_state;
    SIGNAL data_reg   : std_logic_vector(G_DATA_WIDTH - 1 DOWNTO 0);
    SIGNAL valid_reg  : std_logic;

BEGIN

    ----------------------------------------------------------------------------
    -- Output assignments
    ----------------------------------------------------------------------------

    DATA_O  <= data_reg;
    VALID_O <= valid_reg;

    ----------------------------------------------------------------------------
    -- Main Combinatory process
    ----------------------------------------------------------------------------

    p_fsm : PROCESS (state, VALID_I, DATA_I)
    BEGIN
        -- Default assignments
        next_state <= state;
        data_reg   <= data_reg;
        valid_reg  <= valid_reg;

        CASE state IS
            WHEN IDLE =>
                valid_reg <= '0';
                IF VALID_I = '1' THEN
                    data_reg   <= DATA_I;
                    next_state <= RUN;
                END IF;

            WHEN RUN =>
                valid_reg  <= '1';
                next_state <= DONE;

            WHEN DONE =>
                valid_reg  <= '0';
                next_state <= IDLE;

            WHEN OTHERS =>
                valid_reg  <= '0';
                data_reg   <= (OTHERS => '0');
                next_state <= IDLE;

        END CASE;
    END PROCESS p_fsm;

    ----------------------------------------------------------------------------
    -- Main sequential process
    ----------------------------------------------------------------------------

    p_seq_fsm : PROCESS (clk, rst)
    BEGIN
        IF rst = '1' THEN
            state     <= IDLE;
            data_reg  <= (OTHERS => '0');
            valid_reg <= '0';
        ELSIF rising_edge(clk) THEN
            state <= next_state;
        END IF;
    END PROCESS p_seq_fsm;
END ARCHITECTURE rtl;