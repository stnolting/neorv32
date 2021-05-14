library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library iCE40UP;
use iCE40UP.components.all;

entity SP256K is
  port (
    AD       : in std_logic_vector(13 downto 0);
    DI       : in std_logic_vector(15 downto 0);
    MASKWE   : in std_logic_vector(3 downto 0);
    WE       : in std_logic;
    CS       : in std_logic;
    CK       : in std_logic;
    STDBY    : in std_logic;
    SLEEP    : in std_logic;
    PWROFF_N : in std_logic;
    DO       : out std_logic_vector(15 downto 0)
  );
end entity SP256K;


architecture rtl of SP256K is

begin

  ram_inst : SB_SPRAM256KA
  port map (
    ADDRESS    => AD,
    DATAIN     => DI,
    MASKWREN   => MASKWE,
    WREN       => WE,
    CHIPSELECT => CS,
    CLOCK      => CK,
    STANDBY    => STDBY,
    SLEEP      => SLEEP,
    POWEROFF   => PWROFF_N,
    DATAOUT    => DO
  );

end architecture rtl;


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library iCE40UP;
use iCE40UP.components.all;

entity HSOSC is
  generic (
    CLKHF_DIV : string
  );
  port (
    CLKHFPU : in  std_logic;
    CLKHFEN : in  std_logic;
    CLKHF   : out std_logic
  );
end entity HSOSC;

architecture rtl of HSOSC is

begin

  osc_inst : SB_HFOSC
  generic map (
    CLKHF_DIV => CLKHF_DIV
  )
  port map (
    CLKHFPU => CLKHFPU,
    CLKHFEN => CLKHFEN,
    CLKHF   => CLKHF
  );

end architecture rtl;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library iCE40UP;
use iCE40UP.components.all;

entity RGB is
  generic (
    CURRENT_MODE : string := "0b0";
    RGB0_CURRENT : string := "0b000000";
    RGB1_CURRENT : string := "0b000000";
    RGB2_CURRENT : string := "0b000000"
  );
  port (
    CURREN   : in  std_logic;
    RGBLEDEN : in  std_logic;
    RGB0PWM  : in  std_logic;
    RGB1PWM  : in  std_logic;
    RGB2PWM  : in  std_logic;
    RGB0     : out std_logic;
    RGB1     : out std_logic;
    RGB2     : out std_logic
  );
end entity RGB;

architecture rtl of RGB is

begin

  RGB_inst: SB_RGBA_DRV
  generic map (
    CURRENT_MODE => CURRENT_MODE,
    RGB0_CURRENT => RGB0_CURRENT,
    RGB1_CURRENT => RGB1_CURRENT,
    RGB2_CURRENT => RGB2_CURRENT
  )
  port map (
    CURREN   => CURREN,
    RGBLEDEN => RGBLEDEN,
    RGB0PWM  => RGB0PWM,
    RGB1PWM  => RGB1PWM,
    RGB2PWM  => RGB2PWM,
    RGB2     => RGB0,
    RGB1     => RGB1,
    RGB0     => RGB2
  );

end architecture rtl;
