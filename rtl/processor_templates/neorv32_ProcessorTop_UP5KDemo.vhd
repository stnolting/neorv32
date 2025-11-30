-- ================================================================================ --
-- NEORV32 Templates - Example setup for boards with UP5K devices                   --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;

entity neorv32_ProcessorTop_UP5KDemo is
  generic (
    -- Clocking --
    CLOCK_FREQUENCY : natural := 0;       -- clock frequency of clk_i in Hz
    -- Internal Instruction memory --
    IMEM_EN         : boolean := true;    -- implement processor-internal instruction memory
    IMEM_SIZE       : natural := 64*1024; -- size of processor-internal instruction memory in bytes
    -- Internal Data memory --
    DMEM_EN         : boolean := true;    -- implement processor-internal data memory
    DMEM_SIZE       : natural := 64*1024; -- size of processor-internal data memory in bytes
    -- Processor peripherals --
    IO_GPIO_NUM     : natural := 32;      -- number of GPIO input/output pairs (0..32)
    IO_PWM_NUM      : natural := 3        -- number of PWM channels to implement (0..32)
  );
  port (
    -- Global control --
    clk_i       : in  std_logic;
    rstn_i      : in  std_logic;
    -- GPIO (available if IO_GPIO_NUM > 0) --
    gpio_i      : in  std_ulogic_vector(3 downto 0);
    gpio_o      : out std_ulogic_vector(3 downto 0);
    -- primary UART0 (available if IO_UART0_EN = true) --
    uart_txd_o  : out std_ulogic; -- UART0 send data
    uart_rxd_i  : in  std_ulogic := '0'; -- UART0 receive data
    -- SPI to on-board flash --
    flash_sck_o : out std_ulogic;
    flash_sdo_o : out std_ulogic;
    flash_sdi_i : in  std_ulogic;
    flash_csn_o : out std_ulogic; -- NEORV32.SPI_CS(0)
    -- SPI (available if IO_SPI_EN = true) --
    spi_sck_o   : out std_ulogic;
    spi_sdo_o   : out std_ulogic;
    spi_sdi_i   : in  std_ulogic;
    spi_csn_o   : out std_ulogic; -- NEORV32.SPI_CS(1)
    -- TWI (available if IO_TWI_EN = true) --
    twi_sda_io  : inout std_logic;
    twi_scl_io  : inout std_logic;
    -- PWM (available if IO_PWM_NUM > 0) --
    pwm_o       : out std_ulogic_vector(IO_PWM_NUM-1 downto 0)
  );
end entity;

architecture neorv32_ProcessorTop_UP5KDemo_rtl of neorv32_ProcessorTop_UP5KDemo is

  -- internal IO connection --
  signal con_gpio_o    : std_ulogic_vector(31 downto 0);
  signal con_gpio_i    : std_ulogic_vector(31 downto 0);
  signal con_pwm_o     : std_ulogic_vector(31 downto 0);
  signal con_spi_sck   : std_ulogic;
  signal con_spi_sdi   : std_ulogic;
  signal con_spi_sdo   : std_ulogic;
  signal con_spi_csn   : std_ulogic_vector(7 downto 0);
  signal con_twi_sda_i : std_ulogic;
  signal con_twi_sda_o : std_ulogic;
  signal con_twi_scl_i : std_ulogic;
  signal con_twi_scl_o : std_ulogic;

begin

  -- The core of the problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_inst: entity neorv32.neorv32_top
  generic map (
    -- Clocking --
    CLOCK_FREQUENCY => CLOCK_FREQUENCY, -- clock frequency of clk_i in Hz
    -- Boot Configuration --
    BOOT_MODE_SELECT => 0,              -- boot via internal bootloader
    -- RISC-V CPU Extensions --
    RISCV_ISA_M      => true,           -- implement mul/div extension?
    RISCV_ISA_U      => true,           -- implement user mode extension?
    RISCV_ISA_Zicntr => true,           -- implement base counters?
    -- Internal Instruction memory --
    IMEM_EN          => IMEM_EN,        -- implement processor-internal instruction memory
    IMEM_SIZE        => IMEM_SIZE,      -- size of processor-internal instruction memory in bytes
    -- Internal Data memory --
    DMEM_EN          => DMEM_EN,        -- implement processor-internal data memory
    DMEM_SIZE        => DMEM_SIZE,      -- size of processor-internal data memory in bytes
    -- Processor peripherals --
    IO_GPIO_NUM      => IO_GPIO_NUM,    -- number of GPIO input/output pairs (0..32)
    IO_CLINT_EN      => true,           -- implement core local interruptor (CLINT)?
    IO_UART0_EN      => true,           -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_SPI_EN        => true,           -- implement serial peripheral interface (SPI)?
    IO_TWI_EN        => true,           -- implement two-wire interface (TWI)?
    IO_PWM_NUM       => IO_PWM_NUM      -- number of PWM channels to implement (0..32); 0 = disabled
  )
  port map (
    -- Global control --
    clk_i       => clk_i,         -- global clock, rising edge
    rstn_i      => rstn_i,        -- global reset, low-active, async
    -- GPIO (available if IO_GPIO_NUM > 0) --
    gpio_o      => con_gpio_o,    -- parallel output
    gpio_i      => con_gpio_i,    -- parallel input
    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o => uart_txd_o,    -- UART0 send data
    uart0_rxd_i => uart_rxd_i,    -- UART0 receive data
    -- TWI (available if IO_TWI_EN = true) --
    twi_sda_i   => con_twi_sda_i, -- serial data line sense input
    twi_sda_o   => con_twi_sda_o, -- serial data line output (pull low only)
    twi_scl_i   => con_twi_scl_i, -- serial clock line sense input
    twi_scl_o   => con_twi_scl_o, -- serial clock line output (pull low only)
    -- PWM (available if IO_PWM_NUM > 0) --
    pwm_o       => con_pwm_o,      -- pwm channels
	-- SPI (available if IO_SPI_EN = true) --
    spi_clk_o   => con_spi_sck,	   -- SPI clock
    spi_dat_o   => con_spi_sdo,    -- SPI Data out (MOSI)
    spi_dat_i   => con_spi_sdi,    -- SPI Data in (MISO)
    spi_csn_o   => con_spi_csn     -- SPI CSn
  );

  -- SPI: on-board flash --
  flash_sck_o <= con_spi_sck;
  flash_sdo_o <= con_spi_sdo;
  flash_csn_o <= con_spi_csn(0);

  -- SPI: user port --
  spi_sck_o   <= con_spi_sck;
  spi_sdo_o   <= con_spi_sdo;
  spi_csn_o   <= con_spi_csn(1);

  con_spi_sdi <= flash_sdi_i when (con_spi_csn(0) = '0') else spi_sdi_i;

  -- GPIO --
  gpio_o <= con_gpio_o(3 downto 0);
  con_gpio_i(3 downto 0)  <= gpio_i;
  con_gpio_i(31 downto 4) <= (others => '0');

  -- PWM --
  pwm_o <= con_pwm_o(IO_PWM_NUM-1 downto 0);

  -- TWI tri-state driver --
  twi_sda_io    <= '0' when (con_twi_sda_o = '0') else 'Z'; -- module can only pull the line low actively
  twi_scl_io    <= '0' when (con_twi_scl_o = '0') else 'Z';
  con_twi_sda_i <= std_ulogic(twi_sda_io);
  con_twi_scl_i <= std_ulogic(twi_scl_io);


end architecture;
