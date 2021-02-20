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
--  Entity: 	   marsokhod3_neorv32_double_core
--  File:        marsokhod3_neorv32_double_core.vhd
--  Author:      Serge Knigovedov, hitche/at\yandex.com
--  Description: Top level double core NEORV32 SoC for Marsokhod3 board for
--               synthesis testing only yet
--------------------------------------------------------------------------------

library IEEE;
    use IEEE.STD_LOGIC_1164.all;
    use IEEE.NUMERIC_STD.all;
    use IEEE.MATH_REAL.all;

library neorv32;
    use neorv32.neorv32_package.all;

--use WORK.wishbone_package.all;

entity marsokhod3_double_core is
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
end marsokhod3_double_core;

architecture rtl of marsokhod3_double_core is

  signal clk        : std_ulogic;
  signal rs_n       : std_ulogic;
  signal gpio_out1  : std_ulogic_vector(31 downto 0);
  signal gpio_out2  : std_ulogic_vector(31 downto 0);
  signal gpio_in    : std_ulogic_vector(31 downto 0);

  -- Time label
  constant TL_DIV   : natural := 500_000; -- Frequency tl = 100 MHz / DIV / TL_DIV = 100 Hz
  signal tl_cntr    : unsigned(natural(ceil(log2(real(TL_DIV))))-1 downto 0);
  signal tl         : std_ulogic;

  -- Wishbone bus interface
  signal wb_a_tag   : std_ulogic_vector( 2 downto 0); -- tag
  signal wb_a_adr   : std_ulogic_vector(31 downto 0); -- address
  signal wb_a_dat_o : std_ulogic_vector(31 downto 0); -- read data
  signal wb_a_dat_i : std_ulogic_vector(31 downto 0); -- write data
  signal wb_a_we    : std_ulogic;                     -- read/write
  signal wb_a_sel   : std_ulogic_vector( 3 downto 0); -- byte enable
  signal wb_a_stb   : std_ulogic;                     -- strobe
  signal wb_a_cyc   : std_ulogic;                     -- valid cycle
  signal wb_a_lock  : std_ulogic;                     -- locked/exclusive bus access
  signal wb_a_ack   : std_ulogic;                     -- transfer acknowledge
  signal wb_a_err   : std_ulogic;                     -- transfer error
  signal wb_b_tag   : std_ulogic_vector( 2 downto 0); -- tag
  signal wb_b_adr   : std_ulogic_vector(31 downto 0); -- address
  signal wb_b_dat_o : std_ulogic_vector(31 downto 0); -- read data
  signal wb_b_dat_i : std_ulogic_vector(31 downto 0); -- write data
  signal wb_b_we    : std_ulogic;                     -- read/write
  signal wb_b_sel   : std_ulogic_vector( 3 downto 0); -- byte enable
  signal wb_b_stb   : std_ulogic;                     -- strobe
  signal wb_b_cyc   : std_ulogic;                     -- valid cycle
  signal wb_b_lock  : std_ulogic;                     -- locked/exclusive bus access
  signal wb_b_ack   : std_ulogic;                     -- transfer acknowledge
  signal wb_b_err   : std_ulogic;                     -- transfer error
  --signal wb_a_i     : wb_slv_in_type;
  --signal wb_a_o     : wb_slv_out_type;
  --signal wb_b_i     : wb_slv_in_type;
  --signal wb_b_o     : wb_slv_out_type;
  --signal wb_cpu1_i  : wb_mst_in_type;
  --signal wb_cpu1_o  : wb_mst_out_type;
  --signal wb_cpu2_i  : wb_mst_in_type;
  --signal wb_cpu2_o  : wb_mst_out_type;

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

-- Time label                                                                 --
  tl_p: process(rs_n, clk)
  begin
    if rs_n = '0' then
      tl_cntr <= (others => '0');
      tl <= '0';
    elsif rising_edge(clk) then
      tl_cntr <= tl_cntr + 1;

      if tl_cntr = TL_DIV - 1 then
        tl_cntr <= (others => '0');
        tl <= '1';
      else
        tl <= '0';
      end if;
    end if;
  end process;

-- Input                                                                      --
  gpio_in(31 downto 1) <= (others => '0');
  gpio_in(0) <= KEY1;

-- Output                                                                     --
  LED(7 downto 4) <= gpio_out1(3 downto 0);
  LED(3 downto 0) <= gpio_out2(3 downto 0);

-- Dual port RAM                                                              --
  dpr: entity work.wb_dp_ram
    generic map (
      DATA_WIDTH      => 32,
      MEM_BASE        => x"90000000",
      MEM_SIZE        => 32
    )
    port map (
      clk         => clk,
      -- Wishbone bus interface
      wb_a_tag    => wb_a_tag,
      wb_a_adr    => wb_a_adr,
      wb_a_dat_o  => wb_a_dat_o,
      wb_a_dat_i  => wb_a_dat_i,
      wb_a_we     => wb_a_we,
      wb_a_sel    => wb_a_sel,
      wb_a_stb    => wb_a_stb,
      wb_a_cyc    => wb_a_cyc,
      wb_a_lock   => wb_a_lock,
      wb_a_ack    => wb_a_ack,
      wb_a_err    => wb_a_err,
      wb_b_tag    => wb_b_tag,
      wb_b_adr    => wb_b_adr,
      wb_b_dat_o  => wb_b_dat_o,
      wb_b_dat_i  => wb_b_dat_i,
      wb_b_we     => wb_b_we,
      wb_b_sel    => wb_b_sel,
      wb_b_stb    => wb_b_stb,
      wb_b_cyc    => wb_b_cyc,
      wb_b_lock   => wb_b_lock,
      wb_b_ack    => wb_b_ack,
      wb_b_err    => wb_b_err
    );

-- Peripheral SoC                                                             --
  core_peripheral: neorv32_top
    generic map (
      -- General --
      CLOCK_FREQUENCY               => 50000000,    -- clock frequency of clk_i in Hz
      BOOTLOADER_EN                 => true,        -- implement processor-internal bootloader?
      USER_CODE                     => x"600DBABE", -- custom user code
      HW_THREAD_ID                  => 0,           -- hardware thread id (hartid)
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
      IO_UART0_EN                   => true,        -- implement primary universal asynchronous receiver/transmitter (UART0)?
      IO_UART1_EN                   => false,       -- implement secondary universal asynchronous receiver/transmitter (UART1)?
      IO_SPI_EN                     => true,        -- implement serial peripheral interface (SPI)?
      IO_TWI_EN                     => true,        -- implement two-wire interface (TWI)?
      IO_PWM_EN                     => true,        -- implement pulse-width modulation unit (PWM)?
      IO_WDT_EN                     => true,        -- implement watch dog timer (WDT)?
      IO_TRNG_EN                    => true,        -- implement true random number generator (TRNG)?
      IO_CFS_CONFIG                 => (others => '0'), -- custom CFS configuration generic
      IO_NCO_EN                     => true         -- implement numerically-controlled oscillator (NCO)?
    )
    port map (
      -- Global control --
      clk_i       => clk,             -- global clock, rising edge
      rstn_i      => rs_n,            -- global reset, low-active, async
      -- Wishbone bus interface --
      wb_tag_o    => wb_a_tag,        -- tag
      wb_adr_o    => wb_a_adr,        -- address
      wb_dat_i    => wb_a_dat_o,      -- read data
      wb_dat_o    => wb_a_dat_i,      -- write data
      wb_we_o     => wb_a_we,         -- read/write
      wb_sel_o    => wb_a_sel,        -- byte enable
      wb_stb_o    => wb_a_stb,        -- strobe
      wb_cyc_o    => wb_a_cyc,        -- valid cycle
      wb_lock_o   => wb_a_lock,       -- locked/exclusive bus access
      wb_ack_i    => wb_a_ack,        -- transfer acknowledge
      wb_err_i    => wb_a_err,        -- transfer error
      -- Advanced memory control signals (available if MEM_EXT_EN = true) --
      fence_o     => open,            -- indicates an executed FENCE operation
      fencei_o    => open,            -- indicates an executed FENCEI operation
      -- GPIO --
      gpio_o      => gpio_out1,       -- parallel output
      gpio_i      => gpio_in,         -- parallel input
      -- primary UART0 (available if IO_UART0_EN = true) --
      uart0_txd_o => FTDI_AD(1),      -- UART0 send data
      uart0_rxd_i => FTDI_AD(0),      -- UART0 receive data
      -- secondary UART1 (available if IO_UART1_EN = true) --
      uart1_txd_o => open,            -- UART1 send data
      uart1_rxd_i => '0',             -- UART1 receive data
      -- SPI --
      spi_sck_o   => open,            -- SPI serial clock
      spi_sdo_o   => open,            -- controller data out, peripheral data in
      spi_sdi_i   => '0',             -- controller data in, peripheral data out
      spi_csn_o   => open,            -- SPI CS
      -- TWI --
      twi_sda_io  => open,            -- twi serial data line
      twi_scl_io  => open,            -- twi serial clock line
      -- PWM --
      pwm_o       => open,            -- pwm channels
      -- Custom Functions Subsystem IO (available if IO_CFS_EN = true) --
      cfs_in_i    => (others => '0'), -- custom CFS inputs conduit
      cfs_out_o   => open,            -- custom CFS outputs conduit
      -- NCO output (available if IO_NCO_EN = true) --
      nco_o       => open,            -- numerically-controlled oscillator channels
      -- system time input from external MTIME (available if IO_MTIME_EN = false) --
      mtime_i     => (others => '0'), -- current system time
      -- Interrupts --
      soc_firq_i  => (others => '0'), -- fast interrupt channels
      mtime_irq_i => '0',             -- machine timer interrupt, available if IO_MTIME_EN = false
      msw_irq_i   => '0',             -- machine software interrupt
      mext_irq_i  => tl               -- machine external interrupt
    );

-- General SoC                                                                --
  core_general: neorv32_top
    generic map (
      -- General --
      CLOCK_FREQUENCY               => 50000000,    -- clock frequency of clk_i in Hz
      BOOTLOADER_EN                 => true,        -- implement processor-internal bootloader?
      USER_CODE                     => x"600DBABE", -- custom user code
      HW_THREAD_ID                  => 0,           -- hardware thread id (hartid)
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
      IO_UART0_EN                   => true,        -- implement primary universal asynchronous receiver/transmitter (UART0)?
      IO_UART1_EN                   => false,       -- implement secondary universal asynchronous receiver/transmitter (UART1)?
      IO_SPI_EN                     => true,        -- implement serial peripheral interface (SPI)?
      IO_TWI_EN                     => true,        -- implement two-wire interface (TWI)?
      IO_PWM_EN                     => true,        -- implement pulse-width modulation unit (PWM)?
      IO_WDT_EN                     => true,        -- implement watch dog timer (WDT)?
      IO_TRNG_EN                    => true,        -- implement true random number generator (TRNG)?
      IO_CFS_EN                     => true,        -- implement custom functions subsystem (CFS)?
      IO_CFS_CONFIG                 => (others => '0'), -- custom CFS configuration generic
      IO_NCO_EN                     => true         -- implement numerically-controlled oscillator (NCO)?
    )
    port map (
      -- Global control --
      clk_i       => clk,             -- global clock, rising edge
      rstn_i      => rs_n,            -- global reset, low-active, async
      -- Wishbone bus interface --
      wb_tag_o    => wb_b_tag,        -- tag
      wb_adr_o    => wb_b_adr,        -- address
      wb_dat_i    => wb_b_dat_o,      -- read data
      wb_dat_o    => wb_b_dat_i,      -- write data
      wb_we_o     => wb_b_we,         -- read/write
      wb_sel_o    => wb_b_sel,        -- byte enable
      wb_stb_o    => wb_b_stb,        -- strobe
      wb_cyc_o    => wb_b_cyc,        -- valid cycle
      wb_lock_o   => wb_b_lock,       -- locked/exclusive bus access
      wb_ack_i    => wb_b_ack,        -- transfer acknowledge
      wb_err_i    => wb_b_err,        -- transfer error
      -- Advanced memory control signals (available if MEM_EXT_EN = true) --
      fence_o     => open,            -- indicates an executed FENCE operation
      fencei_o    => open,            -- indicates an executed FENCEI operation
      -- GPIO --
      gpio_o      => gpio_out2,       -- parallel output
      gpio_i      => (others => '0'), -- parallel input
      -- primary UART0 (available if IO_UART0_EN = true) --
      uart0_txd_o => FTDI_BD(1),      -- UART0 send data
      uart0_rxd_i => FTDI_BD(0),      -- UART0 receive data
      -- secondary UART1 (available if IO_UART1_EN = true) --
      uart1_txd_o => open,            -- UART1 send data
      uart1_rxd_i => '0',             -- UART1 receive data
      -- SPI --
      spi_sck_o   => open,            -- SPI serial clock
      spi_sdo_o   => open,            -- controller data out, peripheral data in
      spi_sdi_i   => '0',             -- controller data in, peripheral data out
      spi_csn_o   => open,            -- SPI CS
      -- TWI --
      twi_sda_io  => open,            -- twi serial data line
      twi_scl_io  => open,            -- twi serial clock line
      -- PWM --
      pwm_o       => open,            -- pwm channels
      -- Custom Functions Subsystem IO (available if IO_CFS_EN = true) --
      cfs_in_i    => (others => '0'), -- custom CFS inputs conduit
      cfs_out_o   => open,            -- custom CFS outputs conduit
      -- NCO output (available if IO_NCO_EN = true) --
      nco_o       => open,            -- numerically-controlled oscillator channels
      -- system time input from external MTIME (available if IO_MTIME_EN = false) --
      mtime_i     => (others => '0'), -- current system time
      -- Interrupts --
      soc_firq_i  => (others => '0'), -- fast interrupt channels
      mtime_irq_i => '0',             -- machine timer interrupt, available if IO_MTIME_EN = false
      msw_irq_i   => '0',             -- machine software interrupt
      mext_irq_i  => tl               -- machine external interrupt
    );

end rtl;
