library ieee ;
use ieee.std_logic_1164.all;

package components is

  -- Yosys / IceCube wrapper components

  component SB_GB
  port(
    GLOBAL_BUFFER_OUTPUT         : out std_logic;
    USER_SIGNAL_TO_GLOBAL_BUFFER : in  std_logic
  );
  end component;

  component SB_HFOSC
  generic (
    CLKHF_DIV : string
  );
  port (
    CLKHFPU  : in  std_logic;
    CLKHFEN  : in  std_logic;
    CLKHF    : out std_logic
  );
  end component;

  component SB_PLL40_CORE is
  generic (
    FEEDBACK_PATH                  : string := "SIMPLE";
    DELAY_ADJUSTMENT_MODE_FEEDBACK : string := "FIXED";
    DELAY_ADJUSTMENT_MODE_RELATIVE : string := "FIXED";
    SHIFTREG_DIV_MODE              : std_logic := '0';
    FDA_FEEDBACK                   : std_logic_vector(3 downto 0) := x"0";
    FDA_RELATIVE                   : std_logic_vector(3 downto 0) := x"0";
    PLLOUT_SELECT                  : string := "GENCLK";
    DIVR                           : std_logic_vector(3 downto 0) := x"0";
    DIVF                           : std_logic_vector(6 downto 0) := "0000000";
    DIVQ                           : std_logic_vector(2 downto 0) := "000";
    FILTER_RANGE                   : std_logic_vector(2 downto 0) := "000";
    ENABLE_ICEGATE                 : bit := '0';
    TEST_MODE                      : bit := '0';
    EXTERNAL_DIVIDE_FACTOR         : integer := 1
  );
  port (
    REFERENCECLK    : in  std_logic;
    PLLOUTCORE      : out std_logic;
    PLLOUTGLOBAL    : out std_logic;
    EXTFEEDBACK     : in  std_logic;
    DYNAMICDELAY    : in  std_logic_vector(7 downto 0);
    LOCK            : out std_logic;
    BYPASS          : in  std_logic;
    RESETB          : in  std_logic;
    LATCHINPUTVALUE : in  std_logic;
    SDO             : out std_logic;
    SDI             : in  std_logic;
    SCLK            : in  std_logic
  );
  end component;

  component SB_PLL40_PAD
  generic (
    FEEDBACK_PATH                   : string := "SIMPLE";
    DELAY_ADJUSTMENT_MODE_FEEDBACK  : string := "FIXED";
    DELAY_ADJUSTMENT_MODE_RELATIVE  : string := "FIXED";
    SHIFTREG_DIV_MODE               : bit_vector(1 downto 0) := "00";
    FDA_FEEDBACK                    : bit_vector(3 downto 0) := "0000";
    FDA_RELATIVE                    : bit_vector(3 downto 0) := "0000";
    PLLOUT_SELECT                   : string := "GENCLK";
    DIVR                            : bit_vector(3 downto 0) := x"0";
    DIVF                            : bit_vector(6 downto 0) := "0000000";
    DIVQ                            : bit_vector(2 downto 0) := "000";
    FILTER_RANGE                    : bit_vector(2 downto 0) := "000";
    ENABLE_ICEGATE                  : bit := '0';
    TEST_MODE                       : bit := '0';
    EXTERNAL_DIVIDE_FACTOR          : integer := 1
  );
  port (
    PACKAGEPIN      : in  std_logic;
    PLLOUTCORE      : out std_logic;
    PLLOUTGLOBAL    : out std_logic;
    EXTFEEDBACK     : in  std_logic;
    DYNAMICDELAY    : in  std_logic_vector(7 downto 0);
    LOCK            : out std_logic;
    BYPASS          : in  std_logic;
    RESETB          : in  std_logic;
    LATCHINPUTVALUE : in  std_logic;
    SDO             : out std_logic;
    SDI             : in  std_logic;
    SCLK            : in  std_logic
  );
  end component;

  component SB_RGBA_DRV
  generic (
    CURRENT_MODE : string := "0b0";
    RGB0_CURRENT : string := "0b000000";
    RGB1_CURRENT : string := "0b000000";
    RGB2_CURRENT : string := "0b000000"
  );
  port (
    RGB0PWM  : in  std_logic;
    RGB1PWM  : in  std_logic;
    RGB2PWM  : in  std_logic;
    CURREN   : in  std_logic;
    RGBLEDEN : in  std_logic;
    RGB0     : out std_logic;
    RGB1     : out std_logic;
    RGB2     : out std_logic
  );
  end component;

  component SB_SPRAM256KA
  port (
    ADDRESS    : in std_logic_vector(13 downto 0);
    DATAIN     : in std_logic_vector(15 downto 0);
    MASKWREN   : in std_logic_vector(3 downto 0);
    WREN       : in std_logic;
    CHIPSELECT : in std_logic;
    CLOCK      : in std_logic;
    STANDBY    : in std_logic;
    SLEEP      : in std_logic;
    POWEROFF   : in std_logic;
    DATAOUT    : out std_logic_vector(15 downto 0)
  );
  end component;

end package;
