-- #################################################################################################
-- # << NEORV32 - Example setup for boards with UP5K devices >>                                    #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;

entity neorv32_ProcessorTop_UP5KDemo is
  generic (
    CLOCK_FREQUENCY              : natural := 0;      -- clock frequency of clk_i in Hz
    HW_THREAD_ID                 : natural := 0;      -- hardware thread id (32-bit)

    -- On-Chip Debugger (OCD) --
    ON_CHIP_DEBUGGER_EN          : boolean := false;  -- implement on-chip debugger?

    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_C        : boolean := true;   -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        : boolean := false;  -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        : boolean := true;   -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U        : boolean := false;  -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    : boolean := false;  -- implement 32-bit floating-point extension (using INT regs!)
    CPU_EXTENSION_RISCV_Zicsr    : boolean := true;   -- implement CSR system?
    CPU_EXTENSION_RISCV_Zifencei : boolean := false;  -- implement instruction stream sync.?

    -- Extension Options --
    FAST_MUL_EN                  : boolean := false;  -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                : boolean := false;  -- use barrel shifter for shift operations

    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS              : natural := 0;       -- number of regions (0..16)
    PMP_MIN_GRANULARITY          : natural := 4;       -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes

    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 : natural := 0;       -- number of implemented HPM counters (0..29)
    HPM_CNT_WIDTH                : natural := 40;      -- total size of HPM counters (0..64)

    -- Internal Instruction memory --
    MEM_INT_IMEM_EN              : boolean := true;    -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            : natural := 64*1024; -- size of processor-internal instruction memory in bytes

    -- Internal Data memory --
    MEM_INT_DMEM_EN              : boolean := true;    -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            : natural := 64*1024; -- size of processor-internal data memory in bytes

    -- Internal Cache memory --
    ICACHE_EN                    : boolean := false;  -- implement instruction cache
    ICACHE_NUM_BLOCKS            : natural := 4;      -- i-cache: number of blocks (min 1), has to be a power of 2
    ICACHE_BLOCK_SIZE            : natural := 64;     -- i-cache: block size in bytes (min 4), has to be a power of 2
    ICACHE_ASSOCIATIVITY         : natural := 1;      -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2

    -- Processor peripherals --
    IO_GPIO_NUM                  : natural := 64;     -- number of GPIO input/output pairs (0..64)
    IO_MTIME_EN                  : boolean := true;   -- implement machine system timer (MTIME)?
    IO_UART0_EN                  : boolean := true;   -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_SPI_EN                    : boolean := true;   -- implement serial peripheral interface (SPI)?
    IO_TWI_EN                    : boolean := true;   -- implement two-wire interface (TWI)?
    IO_PWM_NUM_CH                : natural := 3;      -- number of PWM channels to implement (0..12); 0 = disabled
    IO_WDT_EN                    : boolean := true    -- implement watch dog timer (WDT)?
  );
  port (
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

    -- PWM (available if IO_PWM_NUM_CH > 0) --
    pwm_o       : out std_ulogic_vector(IO_PWM_NUM_CH-1 downto 0)
  );
end entity;

architecture neorv32_ProcessorTop_UP5KDemo_rtl of neorv32_ProcessorTop_UP5KDemo is

  -- internal IO connection --
  signal con_gpio_o    : std_ulogic_vector(63 downto 0);
  signal con_gpio_i    : std_ulogic_vector(63 downto 0);
  signal con_pwm_o     : std_ulogic_vector(11 downto 0);
  signal con_spi_sck   : std_ulogic;
  signal con_spi_sdi   : std_ulogic;
  signal con_spi_sdo   : std_ulogic;
  signal con_spi_csn   : std_ulogic_vector(07 downto 0);
  signal con_twi_sda_i : std_ulogic;
  signal con_twi_sda_o : std_ulogic;
  signal con_twi_scl_i : std_ulogic;
  signal con_twi_scl_o : std_ulogic;

begin

  -- IO Connection --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

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
  con_gpio_i(03 downto 0) <= gpio_i;
  con_gpio_i(63 downto 4) <= (others => '0');

  -- PWM --
  pwm_o <= con_pwm_o(IO_PWM_NUM_CH-1 downto 0);

  -- TWI tri-state driver --
  twi_sda_io    <= '0' when (con_twi_sda_o = '0') else 'Z'; -- module can only pull the line low actively
  twi_scl_io    <= '0' when (con_twi_scl_o = '0') else 'Z';
  con_twi_sda_i <= std_ulogic(twi_sda_io);
  con_twi_scl_i <= std_ulogic(twi_scl_io);


  -- The core of the problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_inst: entity neorv32.neorv32_top
  generic map (
    -- General --
    CLOCK_FREQUENCY              => CLOCK_FREQUENCY,  -- clock frequency of clk_i in Hz
    INT_BOOTLOADER_EN            => true,             -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
    HART_ID                      => HW_THREAD_ID,     -- hardware thread id (32-bit)

    -- On-Chip Debugger (OCD) --
    ON_CHIP_DEBUGGER_EN          => ON_CHIP_DEBUGGER_EN,  -- implement on-chip debugger?

    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_C        => CPU_EXTENSION_RISCV_C,         -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        => CPU_EXTENSION_RISCV_E,         -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        => CPU_EXTENSION_RISCV_M,         -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U        => CPU_EXTENSION_RISCV_U,         -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    => CPU_EXTENSION_RISCV_Zfinx,     -- implement 32-bit floating-point extension (using INT regs!)
    CPU_EXTENSION_RISCV_Zicsr    => CPU_EXTENSION_RISCV_Zicsr,     -- implement CSR system?
    CPU_EXTENSION_RISCV_Zicntr   => true,                          -- implement base counters?
    CPU_EXTENSION_RISCV_Zifencei => CPU_EXTENSION_RISCV_Zifencei,  -- implement instruction stream sync.?

    -- Extension Options --
    FAST_MUL_EN                  => FAST_MUL_EN,    -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                => FAST_SHIFT_EN,  -- use barrel shifter for shift operations

    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS              => PMP_NUM_REGIONS,       -- number of regions (0..16)
    PMP_MIN_GRANULARITY          => PMP_MIN_GRANULARITY,   -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes

    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 => HPM_NUM_CNTS,          -- number of implemented HPM counters (0..29)
    HPM_CNT_WIDTH                => HPM_CNT_WIDTH,         -- total size of HPM counters (1..64)

    -- Internal Instruction memory --
    MEM_INT_IMEM_EN              => MEM_INT_IMEM_EN,       -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            => MEM_INT_IMEM_SIZE,     -- size of processor-internal instruction memory in bytes

    -- Internal Data memory --
    MEM_INT_DMEM_EN              => MEM_INT_DMEM_EN,       -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            => MEM_INT_DMEM_SIZE,     -- size of processor-internal data memory in bytes

    -- Internal Cache memory --
    ICACHE_EN                    => ICACHE_EN,             -- implement instruction cache
    ICACHE_NUM_BLOCKS            => ICACHE_NUM_BLOCKS,     -- i-cache: number of blocks (min 1), has to be a power of 2
    ICACHE_BLOCK_SIZE            => ICACHE_BLOCK_SIZE,     -- i-cache: block size in bytes (min 4), has to be a power of 2
    ICACHE_ASSOCIATIVITY         => ICACHE_ASSOCIATIVITY,  -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2

    -- Processor peripherals --
    IO_GPIO_NUM                  => IO_GPIO_NUM,    -- number of GPIO input/output pairs (0..64)
    IO_MTIME_EN                  => IO_MTIME_EN,    -- implement machine system timer (MTIME)?
    IO_UART0_EN                  => IO_UART0_EN,    -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_SPI_EN                    => IO_SPI_EN,      -- implement serial peripheral interface (SPI)?
    IO_TWI_EN                    => IO_TWI_EN,      -- implement two-wire interface (TWI)?
    IO_PWM_NUM_CH                => IO_PWM_NUM_CH,  -- number of PWM channels to implement (0..12); 0 = disabled
    IO_WDT_EN                    => IO_WDT_EN       -- implement watch dog timer (WDT)??
  )
  port map (
    -- Global control --
    clk_i       => clk_i,                        -- global clock, rising edge
    rstn_i      => rstn_i,                       -- global reset, low-active, async

    -- GPIO (available if IO_GPIO_NUM > 0) --
    gpio_o      => con_gpio_o,                   -- parallel output
    gpio_i      => con_gpio_i,                   -- parallel input

    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o => uart_txd_o,                   -- UART0 send data
    uart0_rxd_i => uart_rxd_i,                   -- UART0 receive data

    -- TWI (available if IO_TWI_EN = true) --
    twi_sda_i      => con_twi_sda_i,             -- serial data line sense input
    twi_sda_o      => con_twi_sda_o,             -- serial data line output (pull low only)
    twi_scl_i      => con_twi_scl_i,             -- serial clock line sense input
    twi_scl_o      => con_twi_scl_o,             -- serial clock line output (pull low only)

    -- PWM (available if IO_PWM_NUM_CH > 0) --
    pwm_o       => con_pwm_o                     -- pwm channels
  );


end architecture;
