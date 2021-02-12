--------------------------------------------------------------------------------
--  This file is a part of the NEORV32 project
--  Copyleft (ɔ) 2021, Susanin Crew / ArtfulChips
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
--  ------------------------------------------------------------------------  --
--  Entity: 	   marsokhod3_neorv32
--  File:	       marsokhod3_neorv32.vhd
--  Author:	     Serge Knigovedov, hitche/at\yandex.com
--  Description: Top level NEORV32 SoC for Marsokhod3 board for
--               synthesis testing only yet
--------------------------------------------------------------------------------

library IEEE;
    use IEEE.STD_LOGIC_1164.all;
    use IEEE.NUMERIC_STD.all;

library neorv32;
    use neorv32.neorv32_package.all;

entity marsokhod3 is
  generic (
    DIV       : natural := 2                          -- Coefficient of division input clock 100 MHz
  );
  port (
    -- Buttons
    KEY0      : in std_ulogic;
    KEY1      : in std_ulogic;
    -- Quartz generator
    CLK100MHZ : in std_ulogic;
    -- FT2232H
    FTDI_AD   : inout std_ulogic_vector(7 downto 0);  -- 1th channel - /dev/ttyUSB0
                                                      -- 0: TXD/TCK/SK
                                                      -- 1: RXD/TDI/DO
    FTDI_AC   : inout std_ulogic_vector(7 downto 0);
    FTDI_BD   : inout std_ulogic_vector(3 downto 0);  -- 2th channel - /dev/ttyUSB1
                                                      -- 0: TXD/TCK/SK
                                                      -- 1: RXD/TDI/DO
    -- GPIO
    IO        : inout std_ulogic_vector(19 downto 0);
    -- LEDs
    LED       : out std_ulogic_vector(7 downto 0)
  );
end marsokhod3;

architecture rtl of marsokhod3 is

  signal clk      : std_ulogic;
  signal rs_n     : std_ulogic;
  signal gpio_out : std_ulogic_vector(31 downto 0);
  signal gpio_in  : std_ulogic_vector(31 downto 0);

begin

  rs_n <= KEY0;

-- PLL                                                                        --
  my_pll: entity work.my_pll
    generic map (
      clk0_divide_by  => DIV
    )
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
      PMP_NUM_REGIONS               => 0,           -- number of regions (0..64)
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
      IO_CFS_EN                     => true,        -- implement custom functions subsystem (CFS)?
      IO_CFS_CONFIG                 => (others => '0') -- custom CFS configuration generic
    )
    port map (
      -- Global control --
      clk_i       => clk,             -- global clock, rising edge
      rstn_i      => rs_n,            -- global reset, low-active, async
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
      uart_txd_o  => FTDI_BD(1),      -- UART send data
      uart_rxd_i  => FTDI_BD(0),      -- UART receive data
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
      -- Custom Functions Subsystem IO (available if IO_CFS_EN = true) --
      cfs_in_i    => (others => '0'), -- custom CFS inputs conduit
      cfs_out_o   => open,            -- custom CFS outputs conduit
      -- system time input from external MTIME (available if IO_MTIME_EN = false) --
      mtime_i     => (others => '0'), -- current system time
      -- Interrupts --
      soc_firq_i  => (others => '0'), -- fast interrupt channels
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
