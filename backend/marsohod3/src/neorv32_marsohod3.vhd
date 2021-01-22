--------------------------------------------------------------------------------
--  This file is a part of the NEORV32 project
--  Copyright (C) 2020, Susanin Crew
--
--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU General Public License for more details.
--
--  You should have received a copy of the GNU General Public License
--  along with this program; if not, write to the Free Software
--  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
-----------------------------------------------------------------------------
-- Entity: 	neorv32_marsohod3
-- File:	neorv32_marsohod3.vhd
-- Author:	Serge Knigovedov, hitche/at\yandex.com, Susanin Crew / ArtfulChips
-- Description:	Top level NEORV32 SoC for Marsohod3 board for
--   testing synthesis only yet
--------------------------------------------------------------------------------

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_marsohod3 is
  port (
    -- Buttons
    KEY0      : in std_logic;   -- reset
    KEY1      : in std_logic;   -- gpio_in(0)
    -- Quartz generator
    CLK100MHZ : in std_logic;
    -- 2th channel of FT2232H - /dev/ttyUSB1
    FTDI_BD0  : in std_logic;   -- TXD/TCK/SK
    FTDI_BD1  : out std_logic;  -- RXD/TDI/DO (MOSI)
    -- Inputs/outputs
    IO        : inout std_ulogic_vector(19 downto 0);
    -- LEDs
    LED       : out std_ulogic_vector(7 downto 0)
  );
end neorv32_marsohod3;

architecture rtl of neorv32_marsohod3 is

  signal clk      : std_logic;

  -- gpio
  signal gpio_out : std_ulogic_vector(31 downto 0);
  signal gpio_in  : std_ulogic_vector(31 downto 0);

begin

-- PLL                                                                        --
  my_pll: entity work.my_pll
	port map (
    areset  => '0',
		inclk0  => CLK100MHZ,
		c0      => clk
	);

-- SoC                                                                        --
  neorv32_top_inst: neorv32_top
    generic map (
      -- General --
      CLOCK_FREQUENCY               => 50000000,    -- clock frequency of clk_i in Hz
      BOOTLOADER_EN                 => true,        -- implement processor-internal bootloader?
      USER_CODE                     => x"600DBABE", -- custom user code
      HW_THREAD_ID                  => x"00000000", -- hardware thread id (hartid)
      -- RISC-V CPU Extensions --
      CPU_EXTENSION_RISCV_A         => true,        -- implement atomic extension?
      CPU_EXTENSION_RISCV_B         => true,        -- implement bit manipulation extensions?
      CPU_EXTENSION_RISCV_C         => true,        -- implement compressed extension?
      CPU_EXTENSION_RISCV_E         => false,       -- implement embedded RF extension?
      CPU_EXTENSION_RISCV_M         => true,        -- implement muld/div extension?
      CPU_EXTENSION_RISCV_U         => true,        -- implement user mode extension?
      CPU_EXTENSION_RISCV_Zicsr     => true,        -- implement CSR system?
      CPU_EXTENSION_RISCV_Zifencei  => true,        -- implement instruction stream sync.?
      -- Extension Options --
      FAST_MUL_EN                   => true,        -- use DSPs for M extension's multiplier
      FAST_SHIFT_EN                 => true,        -- use barrel shifter for shift operations
      -- Physical Memory Protection (PMP) --
      PMP_NUM_REGIONS               => 2,           -- number of regions (0..64)
      PMP_MIN_GRANULARITY           => 64*1024,     -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
      -- Hardware Performance Monitors (HPM) --
      HPM_NUM_CNTS                  => 2,           -- number of implemented HPM counters (0..29)
      -- Internal Instruction memory --
      MEM_INT_IMEM_EN               => true,        -- implement processor-internal instruction memory
      MEM_INT_IMEM_SIZE             => 16*1024,     -- size of processor-internal instruction memory in bytes
      MEM_INT_IMEM_ROM              => false,       -- implement processor-internal instruction memory as ROM
      -- Internal Data memory --
      MEM_INT_DMEM_EN               => true,        -- implement processor-internal data memory
      MEM_INT_DMEM_SIZE             => 8*1024,      -- size of processor-internal data memory in bytes
      -- Internal Cache memory --
      ICACHE_EN                     => false,       -- implement instruction cache
      ICACHE_NUM_BLOCKS             => 4,           -- i-cache: number of blocks (min 1), has to be a power of 2
      ICACHE_BLOCK_SIZE             => 64,          -- i-cache: block size in bytes (min 4), has to be a power of 2
      ICACHE_ASSOCIATIVITY          => 1,           -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2
      -- External memory interface --
      MEM_EXT_EN                    => true,        -- implement external memory bus interface?
      -- Processor peripherals --
      IO_GPIO_EN                    => true,        -- implement general purpose input/output port unit (GPIO)?
      IO_MTIME_EN                   => true,        -- implement machine system timer (MTIME)?
      IO_UART_EN                    => true,        -- implement universal asynchronous receiver/transmitter (UART)?
      IO_SPI_EN                     => true,        -- implement serial peripheral interface (SPI)?
      IO_TWI_EN                     => true,        -- implement two-wire interface (TWI)?
      IO_PWM_EN                     => true,        -- implement pulse-width modulation unit (PWM)?
      IO_WDT_EN                     => true,        -- implement watch dog timer (WDT)?
      IO_TRNG_EN                    => true,        -- implement true random number generator (TRNG)?
      IO_CFU0_EN                    => true,        -- implement custom functions unit 0 (CFU0)?
      IO_CFU1_EN                    => true         -- implement custom functions unit 1 (CFU1)?
    )
    port map (
      -- Global control --
      clk_i       => clk,             -- global clock, rising edge
      rstn_i      => KEY0,            -- global reset, low-active, async
      -- Wishbone bus interface --
      wb_tag_o    => open,            -- tag
      wb_adr_o    => open,            -- address
      wb_dat_i    => (others => '0'), -- read data
      wb_dat_o    => open,            -- write data
      wb_we_o     => open,            -- read/write
      wb_sel_o    => open,            -- byte enable
      wb_stb_o    => open,            -- strobe
      wb_cyc_o    => open,            -- valid cycle
      wb_lock_o   => open,            -- locked/exclusive bus access
      wb_ack_i    => '0',             -- transfer acknowledge
      wb_err_i    => '0',             -- transfer error
      -- Advanced memory control signals (available if MEM_EXT_EN = true) --
      fence_o     => IO(18),          -- indicates an executed FENCE operation
      fencei_o    => IO(17),          -- indicates an executed FENCEI operation
      -- GPIO --
      gpio_o      => gpio_out,        -- parallel output
      gpio_i      => gpio_in,         -- parallel input
      -- UART --
      uart_txd_o  => FTDI_BD1,        -- UART send data
      uart_rxd_i  => FTDI_BD0,        -- UART receive data
      -- SPI --
      spi_sck_o   => IO(16),          -- SPI serial clock
      spi_sdo_o   => IO(15),          -- controller data out, peripheral data in
      spi_sdi_i   => IO(14),          -- controller data in, peripheral data out
      spi_csn_o   => IO(13 downto 6), -- SPI CS
      -- TWI --
      twi_sda_io  => IO(5),           -- twi serial data line
      twi_scl_io  => IO(4),           -- twi serial clock line
      -- PWM --
      pwm_o       => IO(3 downto 0),  -- pwm channels
      -- system time input from external MTIME (available if IO_MTIME_EN = false) --
      mtime_i     => (others => '0'), -- current system time
      -- Interrupts --
      mtime_irq_i => '0',             -- machine timer interrupt, available if IO_MTIME_EN = false
      msw_irq_i   => '0',             -- machine software interrupt
      mext_irq_i  => IO(19)           -- machine external interrupt
    );

-- Input                                                                      --
  gpio_in(31 downto 1) <= (others => '0');
  gpio_in(0) <= KEY1;

-- Output                                                                     --
  LED <= gpio_out(7 downto 0);

end rtl;
