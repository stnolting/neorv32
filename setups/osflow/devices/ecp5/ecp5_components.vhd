library ieee ;
use ieee.std_logic_1164.all;

package components is

  -- Yosys wrapper components

  component EHXPLLL
  generic
  (
    CLKI_DIV         : integer := 1;
    CLKFB_DIV        : integer := 1;
    CLKOP_DIV        : integer := 8;
    CLKOS_DIV        : integer := 8;
    CLKOS2_DIV       : integer := 8;
    CLKOS3_DIV       : integer := 8;
    CLKOP_ENABLE     : string  := "ENABLED";
    CLKOS_ENABLE     : string  := "DISABLED";
    CLKOS2_ENABLE    : string  := "DISABLED";
    CLKOS3_ENABLE    : string  := "DISABLED";
    CLKOP_CPHASE     : integer := 0;
    CLKOS_CPHASE     : integer := 0;
    CLKOS2_CPHASE    : integer := 0;
    CLKOS3_CPHASE    : integer := 0;
    CLKOP_FPHASE     : integer := 0;
    CLKOS_FPHASE     : integer := 0;
    CLKOS2_FPHASE    : integer := 0;
    CLKOS3_FPHASE    : integer := 0;
    FEEDBK_PATH      : string  := "CLKOP";
    CLKOP_TRIM_POL   : string  := "RISING";
    CLKOP_TRIM_DELAY : integer := 0;
    CLKOS_TRIM_POL   : string  := "RISING";
    CLKOS_TRIM_DELAY : integer := 0;
    OUTDIVIDER_MUXA  : string  := "DIVA";
    OUTDIVIDER_MUXB  : string  := "DIVB";
    OUTDIVIDER_MUXC  : string  := "DIVC";
    OUTDIVIDER_MUXD  : string  := "DIVD";
    PLL_LOCK_MODE    : integer := 0;
    PLL_LOCK_DELAY   : integer := 200;
    STDBY_ENABLE     : string  := "DISABLED";
    REFIN_RESET      : string  := "DISABLED";
    SYNC_ENABLE      : string  := "DISABLED";
    INT_LOCK_STICKY  : string  := "ENABLED";
    DPHASE_SOURCE    : string  := "DISABLED";
    PLLRST_ENA       : string  := "DISABLED";
    INTFB_WAKE       : string  := "DISABLED"
  );
  port
  (
    CLKI         : IN  std_logic := 'X';
    CLKFB        : IN  std_logic := 'X';
    RST          : IN  std_logic := 'X';
    STDBY        : IN  std_logic := 'X';
    PLLWAKESYNC  : IN  std_logic := 'X';
    PHASESEL1    : IN  std_logic := 'X';
    PHASESEL0    : IN  std_logic := 'X';
    PHASEDIR     : IN  std_logic := 'X';
    PHASESTEP    : IN  std_logic := 'X';
    PHASELOADREG : IN  std_logic := 'X';
    ENCLKOP      : IN  std_logic := 'X';
    ENCLKOS      : IN  std_logic := 'X';
    ENCLKOS2     : IN  std_logic := 'X';
    ENCLKOS3     : IN  std_logic := 'X';
    CLKOP        : OUT std_logic := 'X';
    CLKOS        : OUT std_logic := 'X';
    CLKOS2       : OUT std_logic := 'X';
    CLKOS3       : OUT std_logic := 'X';
    LOCK         : OUT std_logic := 'X';
    INTLOCK      : OUT std_logic := 'X';
    REFCLK       : OUT std_logic := 'X';
    CLKINTFB     : OUT std_logic := 'X'
  );
  end component;

end package;
