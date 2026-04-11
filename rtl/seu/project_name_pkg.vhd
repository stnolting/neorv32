--==============================================================================
--  Package:       project_name_pkg
--  Project:       <project_name>
--  Author:        <author_name>
--  Created:       <date>
--  Last Modified: <date>
--
--  Description:
--  Package containing types, constants, and component declarations
--  for the module.
--
--==============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package project_name_pkg is

    ----------------------------------------------------------------------------
    -- Version / Metadata
    ----------------------------------------------------------------------------
    constant MODULE_VERSION : string := "1.0";

    ----------------------------------------------------------------------------
    -- Constants
    ----------------------------------------------------------------------------

    constant DATA_WIDTH : integer := 32;
    constant ADDR_WIDTH : integer := 16;

    ----------------------------------------------------------------------------
    -- Types
    ----------------------------------------------------------------------------

    type t_state is (
        IDLE,
        RUN,
        DONE
    );

    ----------------------------------------------------------------------------
    -- Records
    ----------------------------------------------------------------------------

    type t_control is record
        enable : std_logic;
        mode   : std_logic_vector(1 downto 0);
    end record;

    ----------------------------------------------------------------------------
    -- Component Declaration
    ----------------------------------------------------------------------------

    component project_name
        generic (
            G_DATA_WIDTH : integer := DATA_WIDTH
        );
        port (
        ------------------------------------------------------------------------
        -- Clock / Reset
        ------------------------------------------------------------------------
        CLK : in std_logic;
        RST : in std_logic;
        ------------------------------------------------------------------------
        -- Inputs
        ------------------------------------------------------------------------
        DATA_I  : in std_logic_vector(G_DATA_WIDTH-1 downto 0);
        VALID_I : in std_logic;
        ------------------------------------------------------------------------
        -- Outputs
        ------------------------------------------------------------------------
        DATA_O  : out std_logic_vector(G_DATA_WIDTH-1 downto 0);
        VALID_O : out std_logic
        );
    end component;

end package project_name_pkg;


package body project_name_pkg is

    ----------------------------------------------------------------------------
    -- Package Body (functions/procedures if needed)
    ----------------------------------------------------------------------------

end package body project_name_pkg;