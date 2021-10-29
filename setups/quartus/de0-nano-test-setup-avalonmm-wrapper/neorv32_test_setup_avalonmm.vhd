-- #################################################################################################
-- # << NEORV32 - Test Setup using the AvalonMM Interface >>                                       #
-- # ********************************************************************************************* #
-- # (c) "AvalonMM", "NIOS-2", "Qsys", "MegaWizard"  and "Platform Designer"                       # 
-- # are trademarks of Intel                                                                       #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
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
-- # The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32                           #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.ALL;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_test_setup_avalonmm is
  generic (
    -- adapt these for your setup --
    CLOCK_FREQUENCY   : natural := 50000000; -- clock frequency of clk_i in Hz
    MEM_INT_IMEM_SIZE : natural := 16*1024;   -- size of processor-internal instruction memory in bytes
    MEM_INT_DMEM_SIZE : natural := 8*1024     -- size of processor-internal data memory in bytes
  );
  port (
    -- Global control --
    clk_i       : in  std_ulogic; -- global clock, rising edge
    rstn_i      : in  std_ulogic; -- global reset, low-active, async
    -- GPIO --
    gpio_o      : out std_ulogic_vector(7 downto 0); -- parallel output
    -- UART0 --
    uart0_txd_o : out std_ulogic; -- UART0 send data
    uart0_rxd_i : in  std_ulogic  -- UART0 receive data
  );
end entity;

architecture neorv32_test_setup_avalonmm_rtl of neorv32_test_setup_avalonmm is

  component neorv32_top_avalonmm is
    generic (
      -- General --
      CLOCK_FREQUENCY              : natural;           -- clock frequency of clk_i in Hz
      HW_THREAD_ID                 : natural := 0;      -- hardware thread id (32-bit)
      INT_BOOTLOADER_EN            : boolean := false;  -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
  
      -- On-Chip Debugger (OCD) --
      ON_CHIP_DEBUGGER_EN          : boolean := false;  -- implement on-chip debugger
  
      -- RISC-V CPU Extensions --
      CPU_EXTENSION_RISCV_A        : boolean := false;  -- implement atomic extension?
      CPU_EXTENSION_RISCV_B        : boolean := false;  -- implement bit-manipulation extension?
      CPU_EXTENSION_RISCV_C        : boolean := false;  -- implement compressed extension?
      CPU_EXTENSION_RISCV_E        : boolean := false;  -- implement embedded RF extension?
      CPU_EXTENSION_RISCV_M        : boolean := false;  -- implement mul/div extension?
      CPU_EXTENSION_RISCV_U        : boolean := false;  -- implement user mode extension?
      CPU_EXTENSION_RISCV_Zfinx    : boolean := false;  -- implement 32-bit floating-point extension (using INT regs!)
      CPU_EXTENSION_RISCV_Zicsr    : boolean := true;   -- implement CSR system?
      CPU_EXTENSION_RISCV_Zifencei : boolean := false;  -- implement instruction stream sync.?
      CPU_EXTENSION_RISCV_Zmmul    : boolean := false;  -- implement multiply-only M sub-extension?
  
      -- Extension Options --
      FAST_MUL_EN                  : boolean := false;  -- use DSPs for M extension's multiplier
      FAST_SHIFT_EN                : boolean := false;  -- use barrel shifter for shift operations
      CPU_CNT_WIDTH                : natural := 64;     -- total width of CPU cycle and instret counters (0..64)
      CPU_IPB_ENTRIES              : natural := 2;      -- entries is instruction prefetch buffer, has to be a power of 2
  
      -- Physical Memory Protection (PMP) --
      PMP_NUM_REGIONS              : natural := 0;      -- number of regions (0..64)
      PMP_MIN_GRANULARITY          : natural := 64*1024; -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
  
      -- Hardware Performance Monitors (HPM) --
      HPM_NUM_CNTS                 : natural := 0;      -- number of implemented HPM counters (0..29)
      HPM_CNT_WIDTH                : natural := 40;     -- total size of HPM counters (0..64)
  
      -- Internal Instruction memory (IMEM) --
      MEM_INT_IMEM_EN              : boolean := false;  -- implement processor-internal instruction memory
      MEM_INT_IMEM_SIZE            : natural := 16*1024; -- size of processor-internal instruction memory in bytes
  
      -- Internal Data memory (DMEM) --
      MEM_INT_DMEM_EN              : boolean := false;  -- implement processor-internal data memory
      MEM_INT_DMEM_SIZE            : natural := 8*1024; -- size of processor-internal data memory in bytes
  
      -- Internal Cache memory (iCACHE) --
      ICACHE_EN                    : boolean := false;  -- implement instruction cache
      ICACHE_NUM_BLOCKS            : natural := 4;      -- i-cache: number of blocks (min 1), has to be a power of 2
      ICACHE_BLOCK_SIZE            : natural := 64;     -- i-cache: block size in bytes (min 4), has to be a power of 2
      ICACHE_ASSOCIATIVITY         : natural := 1;      -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2
  
      -- Stream link interface (SLINK) --
      SLINK_NUM_TX                 : natural := 0;      -- number of TX links (0..8)
      SLINK_NUM_RX                 : natural := 0;      -- number of TX links (0..8)
      SLINK_TX_FIFO                : natural := 1;      -- TX fifo depth, has to be a power of two
      SLINK_RX_FIFO                : natural := 1;      -- RX fifo depth, has to be a power of two
  
      -- External Interrupts Controller (XIRQ) --
      XIRQ_NUM_CH                  : natural := 0;      -- number of external IRQ channels (0..32)
      XIRQ_TRIGGER_TYPE            : std_ulogic_vector(31 downto 0) := x"ffffffff"; -- trigger type: 0=level, 1=edge
      XIRQ_TRIGGER_POLARITY        : std_ulogic_vector(31 downto 0) := x"ffffffff"; -- trigger polarity: 0=low-level/falling-edge, 1=high-level/rising-edge
  
      -- Processor peripherals --
      IO_GPIO_EN                   : boolean := false;  -- implement general purpose input/output port unit (GPIO)?
      IO_MTIME_EN                  : boolean := false;  -- implement machine system timer (MTIME)?
      IO_UART0_EN                  : boolean := false;  -- implement primary universal asynchronous receiver/transmitter (UART0)?
      IO_UART1_EN                  : boolean := false;  -- implement secondary universal asynchronous receiver/transmitter (UART1)?
      IO_SPI_EN                    : boolean := false;  -- implement serial peripheral interface (SPI)?
      IO_TWI_EN                    : boolean := false;  -- implement two-wire interface (TWI)?
      IO_PWM_NUM_CH                : natural := 0;      -- number of PWM channels to implement (0..60); 0 = disabled
      IO_WDT_EN                    : boolean := false;  -- implement watch dog timer (WDT)?
      IO_TRNG_EN                   : boolean := false;  -- implement true random number generator (TRNG)?
      IO_CFS_EN                    : boolean := false;  -- implement custom functions subsystem (CFS)?
      IO_CFS_CONFIG                : std_ulogic_vector(31 downto 0) := x"00000000"; -- custom CFS configuration generic
      IO_CFS_IN_SIZE               : positive := 32;    -- size of CFS input conduit in bits
      IO_CFS_OUT_SIZE              : positive := 32;    -- size of CFS output conduit in bits
      IO_NEOLED_EN                 : boolean := false;  -- implement NeoPixel-compatible smart LED interface (NEOLED)?
      IO_NEOLED_TX_FIFO            : natural := 1       -- NEOLED TX FIFO depth, 1..32k, has to be a power of two
    );
    port (
      -- Global control --
      clk_i          : in  std_ulogic; -- global clock, rising edge
      rstn_i         : in  std_ulogic; -- global reset, low-active, async
  
      -- JTAG on-chip debugger interface (available if ON_CHIP_DEBUGGER_EN = true) --
      jtag_trst_i    : in  std_ulogic := 'U'; -- low-active TAP reset (optional)
      jtag_tck_i     : in  std_ulogic := 'U'; -- serial clock
      jtag_tdi_i     : in  std_ulogic := 'U'; -- serial data input
      jtag_tdo_o     : out std_ulogic;        -- serial data output
      jtag_tms_i     : in  std_ulogic := 'U'; -- mode select
  
      -- AvalonMM interface
      read_o         : out std_logic;
      write_o        : out std_logic;
      waitrequest_i  : in std_logic := '0';
      byteenable_o   : out std_logic_vector(3 downto 0);
      address_o      : out std_logic_vector(31 downto 0);
      writedata_o    : out std_logic_vector(31 downto 0);
      readdata_i     : in std_logic_vector(31 downto 0) := (others => '0');

      -- Advanced memory control signals (available if MEM_EXT_EN = true) --
      fence_o        : out std_ulogic; -- indicates an executed FENCE operation
      fencei_o       : out std_ulogic; -- indicates an executed FENCEI operation
  
      -- TX stream interfaces (available if SLINK_NUM_TX > 0) --
      slink_tx_dat_o : out sdata_8x32_t; -- output data
      slink_tx_val_o : out std_ulogic_vector(7 downto 0); -- valid output
      slink_tx_rdy_i : in  std_ulogic_vector(7 downto 0) := (others => 'L'); -- ready to send
  
      -- RX stream interfaces (available if SLINK_NUM_RX > 0) --
      slink_rx_dat_i : in  sdata_8x32_t := (others => (others => 'U')); -- input data
      slink_rx_val_i : in  std_ulogic_vector(7 downto 0) := (others => 'L'); -- valid input
      slink_rx_rdy_o : out std_ulogic_vector(7 downto 0); -- ready to receive
  
      -- GPIO (available if IO_GPIO_EN = true) --
      gpio_o         : out std_ulogic_vector(63 downto 0); -- parallel output
      gpio_i         : in  std_ulogic_vector(63 downto 0) := (others => 'U'); -- parallel input
  
      -- primary UART0 (available if IO_UART0_EN = true) --
      uart0_txd_o    : out std_ulogic; -- UART0 send data
      uart0_rxd_i    : in  std_ulogic := 'U'; -- UART0 receive data
      uart0_rts_o    : out std_ulogic; -- hw flow control: UART0.RX ready to receive ("RTR"), low-active, optional
      uart0_cts_i    : in  std_ulogic := 'L'; -- hw flow control: UART0.TX allowed to transmit, low-active, optional
  
      -- secondary UART1 (available if IO_UART1_EN = true) --
      uart1_txd_o    : out std_ulogic; -- UART1 send data
      uart1_rxd_i    : in  std_ulogic := 'U'; -- UART1 receive data
      uart1_rts_o    : out std_ulogic; -- hw flow control: UART1.RX ready to receive ("RTR"), low-active, optional
      uart1_cts_i    : in  std_ulogic := 'L'; -- hw flow control: UART1.TX allowed to transmit, low-active, optional
  
      -- SPI (available if IO_SPI_EN = true) --
      spi_sck_o      : out std_ulogic; -- SPI serial clock
      spi_sdo_o      : out std_ulogic; -- controller data out, peripheral data in
      spi_sdi_i      : in  std_ulogic := 'U'; -- controller data in, peripheral data out
      spi_csn_o      : out std_ulogic_vector(07 downto 0); -- chip-select
  
      -- TWI (available if IO_TWI_EN = true) --
      twi_sda_io     : inout std_logic := 'U'; -- twi serial data line
      twi_scl_io     : inout std_logic := 'U'; -- twi serial clock line
  
      -- PWM (available if IO_PWM_NUM_CH > 0) --
      pwm_o          : out std_ulogic_vector(IO_PWM_NUM_CH-1 downto 0); -- pwm channels
  
      -- Custom Functions Subsystem IO (available if IO_CFS_EN = true) --
      cfs_in_i       : in  std_ulogic_vector(IO_CFS_IN_SIZE-1  downto 0) := (others => 'U'); -- custom CFS inputs conduit
      cfs_out_o      : out std_ulogic_vector(IO_CFS_OUT_SIZE-1 downto 0); -- custom CFS outputs conduit
  
      -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
      neoled_o       : out std_ulogic; -- async serial data line
  
      -- System time --
      mtime_i        : in  std_ulogic_vector(63 downto 0) := (others => 'U'); -- current system time from ext. MTIME (if IO_MTIME_EN = false)
      mtime_o        : out std_ulogic_vector(63 downto 0); -- current system time from int. MTIME (if IO_MTIME_EN = true)
  
      -- External platform interrupts (available if XIRQ_NUM_CH > 0) --
      xirq_i         : in  std_ulogic_vector(XIRQ_NUM_CH-1 downto 0) := (others => 'L'); -- IRQ channels
  
      -- CPU interrupts --
      mtime_irq_i    : in  std_ulogic := 'L'; -- machine timer interrupt, available if IO_MTIME_EN = false
      msw_irq_i      : in  std_ulogic := 'L'; -- machine software interrupt
      mext_irq_i     : in  std_ulogic := 'L'  -- machine external interrupt
    );
  end component neorv32_top_avalonmm;
  
  -- Intel/Altera RAM module created by MegaWizard
  COMPONENT dmem_ram IS
	PORT
	(
		address		: IN STD_LOGIC_VECTOR (10 DOWNTO 0);
		byteena		: IN STD_LOGIC_VECTOR (3 DOWNTO 0) :=  (OTHERS => '1');
		clock		: IN STD_LOGIC  := '1';
		data		: IN STD_LOGIC_VECTOR (31 DOWNTO 0);
		wren		: IN STD_LOGIC ;
		q		: OUT STD_LOGIC_VECTOR (31 DOWNTO 0)
	);
  END COMPONENT dmem_ram;

signal con_gpio_o     : std_ulogic_vector(63 downto 0);

signal read_o         : std_logic;
signal write_o        : std_logic;
signal waitrequest_i  : std_logic;
signal byteenable_o   : std_logic_vector(3 downto 0);
signal address_o      : std_logic_vector(31 downto 0);
signal writedata_o    : std_logic_vector(31 downto 0);
signal readdata_i     : std_logic_vector(31 downto 0);

signal read_wait_cnt  : std_logic_vector(1 downto 0);


begin

  -- The Core Of The Problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_top_inst: neorv32_top_avalonmm
  generic map (
    -- General --
    CLOCK_FREQUENCY              => CLOCK_FREQUENCY,   -- clock frequency of clk_i in Hz
    INT_BOOTLOADER_EN            => false,              -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_C        => true,              -- implement compressed extension?
    CPU_EXTENSION_RISCV_M        => true,              -- implement mul/div extension?
    CPU_EXTENSION_RISCV_Zicsr    => true,              -- implement CSR system?
    -- Internal Instruction memory --
    MEM_INT_IMEM_EN              => true,              -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            => MEM_INT_IMEM_SIZE, -- size of processor-internal instruction memory in bytes
    -- Internal Data memory --
    MEM_INT_DMEM_EN              => false,              -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            => 0,                  -- size of processor-internal data memory in bytes
    -- Processor peripherals --
    IO_GPIO_EN                   => true,              -- implement general purpose input/output port unit (GPIO)?
    IO_MTIME_EN                  => true,              -- implement machine system timer (MTIME)?
    IO_UART0_EN                  => true               -- implement primary universal asynchronous receiver/transmitter (UART0)?
  )
  port map (
    -- Global control --
    clk_i       => clk_i,       -- global clock, rising edge
    rstn_i      => rstn_i,      -- global reset, low-active, async
    -- GPIO (available if IO_GPIO_EN = true) --
    gpio_o      => con_gpio_o,  -- parallel output
    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o => uart0_txd_o, -- UART0 send data
    uart0_rxd_i => uart0_rxd_i,  -- UART0 receive data
    -- AvalonMM Interface
    read_o => read_o,
    write_o => write_o,
    waitrequest_i => waitrequest_i,
    byteenable_o => byteenable_o,
    address_o => address_o,
    writedata_o => writedata_o,
    readdata_i => readdata_i
  );

  -- Simple example hooking up RAM module to AvalonMM Interface
  -- and using this RAM as DMEM
  my_dmem_ram : dmem_ram
	port map(
		address => address_o(12 downto 2),
		byteena => byteenable_o,
		clock => clk_i,
		data => writedata_o,
		wren => write_o,
		q => readdata_i);


  -- Very simple AvalonMM control signals
  -- Write has 0 wait-states
  -- Read has 2 wait-states
  waitrequest_i <= '1' when (read_o = '1' and read_wait_cnt /= "10") else '0';
  process(clk_i, rstn_i)
  begin
    if rstn_i = '0' then
      read_wait_cnt <= "00";
    elsif rising_edge(clk_i) then
      if read_o = '0' then
        read_wait_cnt <= "00";
      else
        read_wait_cnt <= read_wait_cnt + '1';
      end if;
    end if;
  end process;

  -- GPIO output --
  gpio_o <= con_gpio_o(7 downto 0);


end architecture;
