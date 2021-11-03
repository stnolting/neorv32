-- #################################################################################################
-- # << NEORV32 - Processor Top Entity with Resolved Port Signals (std_logic/std_logic_vector) >>  #
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
-- # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_ProcessorTop_stdlogic is
  generic (
    -- General --
    CLOCK_FREQUENCY              : natural := 0;      -- clock frequency of clk_i in Hz
    INT_BOOTLOADER_EN            : boolean := true;   -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
    HW_THREAD_ID                 : natural := 0;      -- hardware thread id (32-bit)
    -- On-Chip Debugger (OCD) --
    ON_CHIP_DEBUGGER_EN          : boolean := false;  -- implement on-chip debugger
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        : boolean := false;  -- implement atomic extension?
    CPU_EXTENSION_RISCV_B        : boolean := false;  -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_C        : boolean := false;  -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        : boolean := false;  -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        : boolean := false;  -- implement muld/div extension?
    CPU_EXTENSION_RISCV_U        : boolean := false;  -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    : boolean := false;  -- implement 32-bit floating-point extension (using INT reg!)
    CPU_EXTENSION_RISCV_Zicsr    : boolean := true;   -- implement CSR system?
    CPU_EXTENSION_RISCV_Zicntr   : boolean := true;   -- implement base counters?
    CPU_EXTENSION_RISCV_Zihpm    : boolean := false;  -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zifencei : boolean := false;  -- implement instruction stream sync.?
    -- Extension Options --
    FAST_MUL_EN                  : boolean := false;  -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                : boolean := false;  -- use barrel shifter for shift operations
    CPU_CNT_WIDTH                : natural := 64;     -- total width of CPU cycle and instret counters (0..64)
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS              : natural := 0;      -- number of regions (0..64)
    PMP_MIN_GRANULARITY          : natural := 64*1024; -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 : natural := 0;      -- number of implemented HPM counters (0..29)
    HPM_CNT_WIDTH                : natural := 40;     -- total size of HPM counters (0..64)
    -- Internal Instruction memory --
    MEM_INT_IMEM_EN              : boolean := true;   -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            : natural := 16*1024; -- size of processor-internal instruction memory in bytes
    -- Internal Data memory --
    MEM_INT_DMEM_EN              : boolean := true;   -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            : natural := 8*1024; -- size of processor-internal data memory in bytes
    -- Internal Cache memory --
    ICACHE_EN                    : boolean := false;  -- implement instruction cache
    ICACHE_NUM_BLOCKS            : natural := 4;      -- i-cache: number of blocks (min 1), has to be a power of 2
    ICACHE_BLOCK_SIZE            : natural := 64;     -- i-cache: block size in bytes (min 4), has to be a power of 2
    ICACHE_ASSOCIATIVITY         : natural := 1;      -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2
    -- External memory interface --
    MEM_EXT_EN                   : boolean := false;  -- implement external memory bus interface?
    MEM_EXT_TIMEOUT              : natural := 255;    -- cycles after a pending bus access auto-terminates (0 = disabled)
    MEM_EXT_PIPE_MODE            : boolean := false;  -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
    MEM_EXT_BIG_ENDIAN           : boolean := false;  -- byte order: true=big-endian, false=little-endian
    MEM_EXT_ASYNC_RX             : boolean := false;  -- use register buffer for RX data when false
    -- Stream link interface --
    SLINK_NUM_TX                 : natural := 0;      -- number of TX links (0..8)
    SLINK_NUM_RX                 : natural := 0;      -- number of TX links (0..8)
    SLINK_TX_FIFO                : natural := 1;      -- TX fifo depth, has to be a power of two
    SLINK_RX_FIFO                : natural := 1;      -- RX fifo depth, has to be a power of two
    -- External Interrupts Controller (XIRQ) --
    XIRQ_NUM_CH                  : natural := 0;      -- number of external IRQ channels (0..32)
    XIRQ_TRIGGER_TYPE            : std_logic_vector(31 downto 0) := (others => '1'); -- trigger type: 0=level, 1=edge
    XIRQ_TRIGGER_POLARITY        : std_logic_vector(31 downto 0) := (others => '1'); -- trigger polarity: 0=low-level/falling-edge, 1=high-level/rising-edge
    -- Processor peripherals --
    IO_GPIO_EN                   : boolean := true;   -- implement general purpose input/output port unit (GPIO)?
    IO_MTIME_EN                  : boolean := true;   -- implement machine system timer (MTIME)?
    IO_UART0_EN                  : boolean := true;   -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_UART0_RX_FIFO             : natural := 1;      -- RX fifo depth, has to be a power of two, min 1
    IO_UART0_TX_FIFO             : natural := 1;      -- TX fifo depth, has to be a power of two, min 1
    IO_UART1_EN                  : boolean := true;   -- implement secondary universal asynchronous receiver/transmitter (UART1)?
    IO_UART1_RX_FIFO             : natural := 1;      -- RX fifo depth, has to be a power of two, min 1
    IO_UART1_TX_FIFO             : natural := 1;      -- TX fifo depth, has to be a power of two, min 1
    IO_SPI_EN                    : boolean := true;   -- implement serial peripheral interface (SPI)?
    IO_TWI_EN                    : boolean := true;   -- implement two-wire interface (TWI)?
    IO_PWM_NUM_CH                : natural := 4;      -- number of PWM channels to implement (0..60); 0 = disabled
    IO_WDT_EN                    : boolean := true;   -- implement watch dog timer (WDT)?
    IO_TRNG_EN                   : boolean := false;  -- implement true random number generator (TRNG)?
    IO_CFS_EN                    : boolean := false;  -- implement custom functions subsystem (CFS)?
    IO_CFS_CONFIG                : std_ulogic_vector(31 downto 0); -- custom CFS configuration generic
    IO_CFS_IN_SIZE               : positive := 32;    -- size of CFS input conduit in bits
    IO_CFS_OUT_SIZE              : positive := 32;    -- size of CFS output conduit in bits
    IO_NEOLED_EN                 : boolean := true;   -- implement NeoPixel-compatible smart LED interface (NEOLED)?
    IO_GPTMR_EN                  : boolean := false   -- implement general purpose timer (GPTMR)?
  );
  port (
    -- Global control --
    clk_i          : in  std_logic := '0'; -- global clock, rising edge
    rstn_i         : in  std_logic := '0'; -- global reset, low-active, async
    -- JTAG on-chip debugger interface (available if ON_CHIP_DEBUGGER_EN = true) --
    jtag_trst_i    : in  std_logic := '0'; -- low-active TAP reset (optional)
    jtag_tck_i     : in  std_logic := '0'; -- serial clock
    jtag_tdi_i     : in  std_logic := '0'; -- serial data input
    jtag_tdo_o     : out std_logic;        -- serial data output
    jtag_tms_i     : in  std_logic := '0'; -- mode select
    -- Wishbone bus interface (available if MEM_EXT_EN = true) --
    wb_tag_o       : out std_logic_vector(02 downto 0); -- tag
    wb_adr_o       : out std_logic_vector(31 downto 0); -- address
    wb_dat_i       : in  std_logic_vector(31 downto 0) := (others => '0'); -- read data
    wb_dat_o       : out std_logic_vector(31 downto 0); -- write data
    wb_we_o        : out std_logic; -- read/write
    wb_sel_o       : out std_logic_vector(03 downto 0); -- byte enable
    wb_stb_o       : out std_logic; -- strobe
    wb_cyc_o       : out std_logic; -- valid cycle
    wb_lock_o      : out std_logic; -- exclusive access request
    wb_ack_i       : in  std_logic := '0'; -- transfer acknowledge
    wb_err_i       : in  std_logic := '0'; -- transfer error
    -- Advanced memory control signals (available if MEM_EXT_EN = true) --
    fence_o        : out std_logic; -- indicates an executed FENCE operation
    fencei_o       : out std_logic; -- indicates an executed FENCEI operation
    -- TX stream interfaces (available if SLINK_NUM_TX > 0) --
    slink_tx_dat_o : out sdata_8x32r_t; -- output data
    slink_tx_val_o : out std_logic_vector(7 downto 0); -- valid output
    slink_tx_rdy_i : in  std_logic_vector(7 downto 0) := (others => '0'); -- ready to send
    -- RX stream interfaces (available if SLINK_NUM_RX > 0) --
    slink_rx_dat_i : in  sdata_8x32r_t := (others => (others => '0')); -- input data
    slink_rx_val_i : in  std_logic_vector(7 downto 0) := (others => '0'); -- valid input
    slink_rx_rdy_o : out std_logic_vector(7 downto 0); -- ready to receive
    -- GPIO (available if IO_GPIO_EN = true) --
    gpio_o         : out std_logic_vector(63 downto 0); -- parallel output
    gpio_i         : in  std_logic_vector(63 downto 0) := (others => '0'); -- parallel input
    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o    : out std_logic; -- UART0 send data
    uart0_rxd_i    : in  std_logic := '0'; -- UART0 receive data
    uart0_rts_o    : out std_logic; -- hw flow control: UART0.RX ready to receive ("RTR"), low-active, optional
    uart0_cts_i    : in  std_logic := '0'; -- hw flow control: UART0.TX allowed to transmit, low-active, optional
    -- secondary UART1 (available if IO_UART1_EN = true) --
    uart1_txd_o    : out std_logic; -- UART1 send data
    uart1_rxd_i    : in  std_logic := '0'; -- UART1 receive data
    uart1_rts_o    : out std_logic; -- hw flow control: UART1.RX ready to receive ("RTR"), low-active, optional
    uart1_cts_i    : in  std_logic := '0'; -- hw flow control: UART1.TX allowed to transmit, low-active, optional
    -- SPI (available if IO_SPI_EN = true) --
    spi_sck_o      : out std_logic; -- SPI serial clock
    spi_sdo_o      : out std_logic; -- controller data out, peripheral data in
    spi_sdi_i      : in  std_logic := '0'; -- controller data in, peripheral data out
    spi_csn_o      : out std_logic_vector(07 downto 0); -- SPI CS
    -- TWI (available if IO_TWI_EN = true) --
    twi_sda_io     : inout std_logic; -- twi serial data line
    twi_scl_io     : inout std_logic; -- twi serial clock line
    -- PWM (available if IO_PWM_NUM_CH > 0) --
    pwm_o          : out std_logic_vector(IO_PWM_NUM_CH-1 downto 0); -- pwm channels
    -- Custom Functions Subsystem IO (available if IO_CFS_EN = true) --
    cfs_in_i       : in  std_logic_vector(IO_CFS_IN_SIZE-1  downto 0); -- custom inputs
    cfs_out_o      : out std_logic_vector(IO_CFS_OUT_SIZE-1 downto 0); -- custom outputs
    -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
    neoled_o       : out std_logic; -- async serial data line
    -- System time --
    mtime_i        : in  std_logic_vector(63 downto 0) := (others => '0'); -- current system time from ext. MTIME (if IO_MTIME_EN = false)
    mtime_o        : out std_logic_vector(63 downto 0); -- current system time from int. MTIME (if IO_MTIME_EN = true)
    -- External platform interrupts (available if XIRQ_NUM_CH > 0) --
    xirq_i         : in  std_logic_vector(XIRQ_NUM_CH-1 downto 0) := (others => '0'); -- IRQ channels
    -- CPU Interrupts --
    mtime_irq_i    : in  std_logic := '0'; -- machine timer interrupt, available if IO_MTIME_EN = false
    msw_irq_i      : in  std_logic := '0'; -- machine software interrupt
    mext_irq_i     : in  std_logic := '0'  -- machine external interrupt
  );
end entity;

architecture neorv32_ProcessorTop_stdlogic_rtl of neorv32_ProcessorTop_stdlogic is

  -- type conversion --
  constant IO_CFS_CONFIG_INT         : std_ulogic_vector(31 downto 0) := std_ulogic_vector(IO_CFS_CONFIG);
  constant XIRQ_TRIGGER_TYPE_INT     : std_ulogic_vector(31 downto 0) := std_ulogic_vector(XIRQ_TRIGGER_TYPE);
  constant XIRQ_TRIGGER_POLARITY_INT : std_ulogic_vector(31 downto 0) := std_ulogic_vector(XIRQ_TRIGGER_POLARITY);
  --
  signal clk_i_int       : std_ulogic;
  signal rstn_i_int      : std_ulogic;
  --
  signal jtag_trst_i_int :std_ulogic;
  signal jtag_tck_i_int  :std_ulogic;
  signal jtag_tdi_i_int  :std_ulogic;
  signal jtag_tdo_o_int  :std_ulogic;
  signal jtag_tms_i_int  :std_ulogic;
  --
  signal wb_tag_o_int    : std_ulogic_vector(02 downto 0);
  signal wb_adr_o_int    : std_ulogic_vector(31 downto 0);
  signal wb_dat_i_int    : std_ulogic_vector(31 downto 0);
  signal wb_dat_o_int    : std_ulogic_vector(31 downto 0);
  signal wb_we_o_int     : std_ulogic;
  signal wb_sel_o_int    : std_ulogic_vector(03 downto 0);
  signal wb_stb_o_int    : std_ulogic;
  signal wb_cyc_o_int    : std_ulogic;
  signal wb_lock_o_int   : std_ulogic;
  signal wb_ack_i_int    : std_ulogic;
  signal wb_err_i_int    : std_ulogic;
  --
  signal fence_o_int     : std_ulogic;
  signal fencei_o_int    : std_ulogic;
  --
  signal slink_tx_dat_o_int : sdata_8x32_t;
  signal slink_tx_val_o_int : std_logic_vector(7 downto 0);
  signal slink_tx_rdy_i_int : std_logic_vector(7 downto 0);
  signal slink_rx_dat_i_int : sdata_8x32_t;
  signal slink_rx_val_i_int : std_logic_vector(7 downto 0);
  signal slink_rx_rdy_o_int : std_logic_vector(7 downto 0);
  --
  signal gpio_o_int      : std_ulogic_vector(63 downto 0);
  signal gpio_i_int      : std_ulogic_vector(63 downto 0);
  --
  signal uart0_txd_o_int : std_ulogic;
  signal uart0_rxd_i_int : std_ulogic;
  signal uart0_rts_o_int : std_ulogic;
  signal uart0_cts_i_int : std_ulogic;
  --
  signal uart1_txd_o_int : std_ulogic;
  signal uart1_rxd_i_int : std_ulogic;
  signal uart1_rts_o_int : std_ulogic;
  signal uart1_cts_i_int : std_ulogic;
  --
  signal spi_sck_o_int   : std_ulogic;
  signal spi_sdo_o_int   : std_ulogic;
  signal spi_sdi_i_int   : std_ulogic;
  signal spi_csn_o_int   : std_ulogic_vector(07 downto 0);
  --
  signal pwm_o_int       : std_ulogic_vector(IO_PWM_NUM_CH-1 downto 0);
  --
  signal cfs_in_i_int    : std_ulogic_vector(IO_CFS_IN_SIZE-1  downto 0);
  signal cfs_out_o_int   : std_ulogic_vector(IO_CFS_OUT_SIZE-1 downto 0);
  --
  signal neoled_o_int    : std_ulogic;
  --
  signal mtime_i_int     : std_ulogic_vector(63 downto 0);
  signal mtime_o_int     : std_ulogic_vector(63 downto 0);
  --
  signal xirq_i_int      : std_ulogic_vector(XIRQ_NUM_CH-1 downto 0);
  --
  signal mtime_irq_i_int : std_ulogic;
  signal msw_irq_i_int   : std_ulogic;
  signal mext_irq_i_int  : std_ulogic;

begin

  -- The Core Of The Problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_top_inst: neorv32_top
  generic map (
    -- General --
    CLOCK_FREQUENCY              => CLOCK_FREQUENCY,    -- clock frequency of clk_i in Hz
    INT_BOOTLOADER_EN            => INT_BOOTLOADER_EN,  -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
    HW_THREAD_ID                 => HW_THREAD_ID,       -- hardware thread id (hartid) (32-bit)
    -- On-Chip Debugger (OCD) --
    ON_CHIP_DEBUGGER_EN          => ON_CHIP_DEBUGGER_EN,          -- implement on-chip debugger
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        => CPU_EXTENSION_RISCV_A,        -- implement atomic extension?
    CPU_EXTENSION_RISCV_B        => CPU_EXTENSION_RISCV_B,        -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_C        => CPU_EXTENSION_RISCV_C,        -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        => CPU_EXTENSION_RISCV_E,        -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        => CPU_EXTENSION_RISCV_M,        -- implement muld/div extension?
    CPU_EXTENSION_RISCV_U        => CPU_EXTENSION_RISCV_U,        -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    => CPU_EXTENSION_RISCV_Zfinx,    -- implement 32-bit floating-point extension (using INT reg!)
    CPU_EXTENSION_RISCV_Zicsr    => CPU_EXTENSION_RISCV_Zicsr,    -- implement CSR system?
    CPU_EXTENSION_RISCV_Zicntr   => CPU_EXTENSION_RISCV_Zicntr,   -- implement base counters?
    CPU_EXTENSION_RISCV_Zihpm    => CPU_EXTENSION_RISCV_Zihpm,    -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zifencei => CPU_EXTENSION_RISCV_Zifencei, -- implement instruction stream sync.?
    -- Extension Options --
    FAST_MUL_EN                  => FAST_MUL_EN,        -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                => FAST_SHIFT_EN,      -- use barrel shifter for shift operations
    CPU_CNT_WIDTH                => CPU_CNT_WIDTH,      -- total width of CPU cycle and instret counters (0..64)
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS              => PMP_NUM_REGIONS,    -- number of regions (0..64)
    PMP_MIN_GRANULARITY          => PMP_MIN_GRANULARITY, -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 => HPM_NUM_CNTS,       -- number of implemented HPM counters (0..29)
    HPM_CNT_WIDTH                => HPM_CNT_WIDTH,      -- total size of HPM counters (0..64)
    -- Internal Instruction memory --
    MEM_INT_IMEM_EN              => MEM_INT_IMEM_EN,    -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            => MEM_INT_IMEM_SIZE,  -- size of processor-internal instruction memory in bytes
    -- Internal Data memory --
    MEM_INT_DMEM_EN              => MEM_INT_DMEM_EN,    -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            => MEM_INT_DMEM_SIZE,  -- size of processor-internal data memory in bytes
    -- Internal Cache memory --
    ICACHE_EN                    => ICACHE_EN,          -- implement instruction cache
    ICACHE_NUM_BLOCKS            => ICACHE_NUM_BLOCKS,  -- i-cache: number of blocks (min 1), has to be a power of 2
    ICACHE_BLOCK_SIZE            => ICACHE_BLOCK_SIZE,  -- i-cache: block size in bytes (min 4), has to be a power of 2
    ICACHE_ASSOCIATIVITY         => ICACHE_ASSOCIATIVITY, -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2
    -- External memory interface --
    MEM_EXT_EN                   => MEM_EXT_EN,         -- implement external memory bus interface?
    MEM_EXT_TIMEOUT              => MEM_EXT_TIMEOUT,    -- cycles after a pending bus access auto-terminates (0 = disabled)
    MEM_EXT_PIPE_MODE            => MEM_EXT_PIPE_MODE,  -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
    MEM_EXT_BIG_ENDIAN           => MEM_EXT_BIG_ENDIAN, -- byte order: true=big-endian, false=little-endian
    MEM_EXT_ASYNC_RX             => MEM_EXT_ASYNC_RX,   -- use register buffer for RX data when false
    -- Stream link interface --
    SLINK_NUM_TX                 => SLINK_NUM_TX,       -- number of TX links (0..8)
    SLINK_NUM_RX                 => SLINK_NUM_RX,       -- number of TX links (0..8)
    SLINK_TX_FIFO                => SLINK_TX_FIFO,      -- TX fifo depth, has to be a power of two
    SLINK_RX_FIFO                => SLINK_RX_FIFO,      -- RX fifo depth, has to be a power of two
    -- External Interrupts Controller (XIRQ) --
    XIRQ_NUM_CH                  => XIRQ_NUM_CH, -- number of external IRQ channels (0..32)
    XIRQ_TRIGGER_TYPE            => XIRQ_TRIGGER_TYPE_INT, -- trigger type: 0=level, 1=edge
    XIRQ_TRIGGER_POLARITY        => XIRQ_TRIGGER_POLARITY_INT, -- trigger polarity: 0=low-level/falling-edge, 1=high-level/rising-edge
    -- Processor peripherals --
    IO_GPIO_EN                   => IO_GPIO_EN,         -- implement general purpose input/output port unit (GPIO)?
    IO_MTIME_EN                  => IO_MTIME_EN,        -- implement machine system timer (MTIME)?
    IO_UART0_EN                  => IO_UART0_EN,        -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_UART0_RX_FIFO             => IO_UART0_RX_FIFO,   -- RX fifo depth, has to be a power of two, min 1
    IO_UART0_TX_FIFO             => IO_UART0_TX_FIFO,   -- TX fifo depth, has to be a power of two, min 1
    IO_UART1_EN                  => IO_UART1_EN,        -- implement secondary universal asynchronous receiver/transmitter (UART1)?
    IO_UART1_RX_FIFO             => IO_UART1_RX_FIFO,   -- RX fifo depth, has to be a power of two, min 1
    IO_UART1_TX_FIFO             => IO_UART1_TX_FIFO,   -- TX fifo depth, has to be a power of two, min 1
    IO_SPI_EN                    => IO_SPI_EN,          -- implement serial peripheral interface (SPI)?
    IO_TWI_EN                    => IO_TWI_EN,          -- implement two-wire interface (TWI)?
    IO_PWM_NUM_CH                => IO_PWM_NUM_CH,      -- number of PWM channels to implement (0..60); 0 = disabled
    IO_WDT_EN                    => IO_WDT_EN,          -- implement watch dog timer (WDT)?
    IO_TRNG_EN                   => IO_TRNG_EN,         -- implement true random number generator (TRNG)?
    IO_CFS_EN                    => IO_CFS_EN,          -- implement custom functions subsystem (CFS)?
    IO_CFS_CONFIG                => IO_CFS_CONFIG_INT,  -- custom CFS configuration generic
    IO_CFS_IN_SIZE               => IO_CFS_IN_SIZE,     -- size of CFS input conduit in bits
    IO_CFS_OUT_SIZE              => IO_CFS_OUT_SIZE,    -- size of CFS output conduit in bits
    IO_NEOLED_EN                 => IO_NEOLED_EN,       -- implement NeoPixel-compatible smart LED interface (NEOLED)?
    IO_GPTMR_EN                  => IO_GPTMR_EN         -- implement general purpose timer (GPTMR)?
  )
  port map (
    -- Global control --
    clk_i          => clk_i_int,       -- global clock, rising edge
    rstn_i         => rstn_i_int,      -- global reset, low-active, async
    -- JTAG on-chip debugger interface (available if ON_CHIP_DEBUGGER_EN = true) --
    jtag_trst_i    => jtag_trst_i_int, -- low-active TAP reset (optional)
    jtag_tck_i     => jtag_tck_i_int,  -- serial clock
    jtag_tdi_i     => jtag_tdi_i_int,  -- serial data input
    jtag_tdo_o     => jtag_tdo_o_int,  -- serial data output
    jtag_tms_i     => jtag_tms_i_int,  -- mode select
    -- Wishbone bus interface (available if MEM_EXT_EN = true) --
    wb_tag_o       => wb_tag_o_int,    -- tag
    wb_adr_o       => wb_adr_o_int,    -- address
    wb_dat_i       => wb_dat_i_int,    -- read data
    wb_dat_o       => wb_dat_o_int,    -- write data
    wb_we_o        => wb_we_o_int,     -- read/write
    wb_sel_o       => wb_sel_o_int,    -- byte enable
    wb_stb_o       => wb_stb_o_int,    -- strobe
    wb_cyc_o       => wb_cyc_o_int,    -- valid cycle
    wb_lock_o      => wb_lock_o_int,   -- exclusive access request
    wb_ack_i       => wb_ack_i_int,    -- transfer acknowledge
    wb_err_i       => wb_err_i_int,    -- transfer error
    -- Advanced memory control signals (available if MEM_EXT_EN = true) --
    fence_o        => fence_o_int,     -- indicates an executed FENCE operation
    fencei_o       => fencei_o_int,    -- indicates an executed FENCEI operation
    -- TX stream interfaces (available if SLINK_NUM_TX > 0) --
    slink_tx_dat_o => slink_tx_dat_o_int, -- output data
    slink_tx_val_o => slink_tx_val_o_int, -- valid output
    slink_tx_rdy_i => slink_tx_rdy_i_int, -- ready to send
    -- RX stream interfaces (available if SLINK_NUM_RX > 0) --
    slink_rx_dat_i => slink_rx_dat_i_int, -- input data
    slink_rx_val_i => slink_rx_val_i_int, -- valid input
    slink_rx_rdy_o => slink_rx_rdy_o_int, -- ready to receive
    -- GPIO (available if IO_GPIO_EN = true) --
    gpio_o         => gpio_o_int,      -- parallel output
    gpio_i         => gpio_i_int,      -- parallel input
    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o    => uart0_txd_o_int, -- UART0 send data
    uart0_rxd_i    => uart0_rxd_i_int, -- UART0 receive data
    uart0_rts_o    => uart0_rts_o_int, -- hw flow control: UART0.RX ready to receive ("RTR"), low-active, optional
    uart0_cts_i    => uart0_cts_i_int, -- hw flow control: UART0.TX allowed to transmit, low-active, optional
    -- secondary UART1 (available if IO_UART1_EN = true) --
    uart1_txd_o    => uart1_txd_o_int, -- UART1 send data
    uart1_rxd_i    => uart1_rxd_i_int, -- UART1 receive data
    uart1_rts_o    => uart1_rts_o_int, -- hw flow control: UART1.RX ready to receive ("RTR"), low-active, optional
    uart1_cts_i    => uart1_cts_i_int, -- hw flow control: UART1.TX allowed to transmit, low-active, optional
    -- SPI (available if IO_SPI_EN = true) --
    spi_sck_o      => spi_sck_o_int,   -- SPI serial clock
    spi_sdo_o      => spi_sdo_o_int,   -- controller data out, peripheral data in
    spi_sdi_i      => spi_sdi_i_int,   -- controller data in, peripheral data out
    spi_csn_o      => spi_csn_o_int,   -- SPI CS
    -- TWI (available if IO_TWI_EN = true) --
    twi_sda_io     => twi_sda_io,      -- twi serial data line
    twi_scl_io     => twi_scl_io,      -- twi serial clock line
    -- PWM (available if IO_PWM_NUM_CH > 0) --
    pwm_o          => pwm_o_int,       -- pwm channels
    -- Custom Functions Subsystem IO (available if IO_CFS_EN = true) --
    cfs_in_i       => cfs_in_i_int,    -- custom inputs
    cfs_out_o      => cfs_out_o_int,   -- custom outputs
    -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
    neoled_o       => neoled_o_int,    -- async serial data line
    -- System time --
    mtime_i        => mtime_i_int,     -- current system time from ext. MTIME (if IO_MTIME_EN = false)
    mtime_o        => mtime_o_int,     -- current system time from int. MTIME (if IO_MTIME_EN = true)
    -- External platform interrupts (available if XIRQ_NUM_CH > 0) --
    xirq_i         => xirq_i_int,      -- IRQ channels
    -- CPU Interrupts --
    mtime_irq_i    => mtime_irq_i_int, -- machine timer interrupt, available if IO_MTIME_EN = false
    msw_irq_i      => msw_irq_i_int,   -- machine software interrupt
    mext_irq_i     => mext_irq_i_int   -- machine external interrupt
  );

  -- type conversion --
  clk_i_int       <= std_ulogic(clk_i);
  rstn_i_int      <= std_ulogic(rstn_i);

  jtag_trst_i_int <= std_ulogic(jtag_trst_i);
  jtag_tck_i_int  <= std_ulogic(jtag_tck_i);
  jtag_tdi_i_int  <= std_ulogic(jtag_tdi_i);
  jtag_tdo_o      <= std_logic(jtag_tdo_o_int);
  jtag_tms_i_int  <= std_ulogic(jtag_tms_i);

  wb_tag_o        <= std_logic_vector(wb_tag_o_int);
  wb_adr_o        <= std_logic_vector(wb_adr_o_int);
  wb_dat_i_int    <= std_ulogic_vector(wb_dat_i);
  wb_dat_o        <= std_logic_vector(wb_dat_o_int);
  wb_we_o         <= std_logic(wb_we_o_int);
  wb_sel_o        <= std_logic_vector(wb_sel_o_int);
  wb_stb_o        <= std_logic(wb_stb_o_int);
  wb_cyc_o        <= std_logic(wb_cyc_o_int);
  wb_lock_o       <= std_logic(wb_lock_o_int);
  wb_ack_i_int    <= std_ulogic(wb_ack_i);
  wb_err_i_int    <= std_ulogic(wb_err_i);

  fence_o         <= std_logic(fence_o_int);
  fencei_o        <= std_logic(fencei_o_int);

  slink_tx_val_o     <= std_logic_vector(slink_tx_val_o_int);
  slink_tx_rdy_i_int <= std_ulogic_vector(slink_tx_rdy_i);
  slink_rx_val_i_int <= std_ulogic_vector(slink_rx_val_i);
  slink_rx_rdy_o     <= std_logic_vector(slink_rx_rdy_o_int);

  slink_conv:
  for i in 0 to 7 generate
    slink_tx_dat_o(i)     <= std_logic_vector(slink_tx_dat_o_int(i));
    slink_rx_dat_i_int(i) <= std_ulogic_vector(slink_rx_dat_i(i));
  end generate;

  gpio_o          <= std_logic_vector(gpio_o_int);
  gpio_i_int      <= std_ulogic_vector(gpio_i);

  uart0_txd_o     <= std_logic(uart0_txd_o_int);
  uart0_rxd_i_int <= std_ulogic(uart0_rxd_i);
  uart0_rts_o     <= std_logic(uart0_rts_o_int);
  uart0_cts_i_int <= std_ulogic(uart0_cts_i);
  uart1_txd_o     <= std_logic(uart1_txd_o_int);
  uart1_rxd_i_int <= std_ulogic(uart1_rxd_i);
  uart1_rts_o     <= std_logic(uart1_rts_o_int);
  uart1_cts_i_int <= std_ulogic(uart1_cts_i);

  spi_sck_o       <= std_logic(spi_sck_o_int);
  spi_sdo_o       <= std_logic(spi_sdo_o_int);
  spi_sdi_i_int   <= std_ulogic(spi_sdi_i);
  spi_csn_o       <= std_logic_vector(spi_csn_o_int);

  pwm_o           <= std_logic_vector(pwm_o_int);

  cfs_in_i_int    <= std_ulogic_vector(cfs_in_i);
  cfs_out_o       <= std_logic_vector(cfs_out_o_int);

  neoled_o        <= std_logic(neoled_o_int);

  mtime_i_int     <= std_ulogic_vector(mtime_i);
  mtime_o         <= std_logic_vector(mtime_o_int);

  xirq_i_int      <= std_ulogic_vector(xirq_i);

  msw_irq_i_int   <= std_ulogic(msw_irq_i);
  mext_irq_i_int  <= std_ulogic(mext_irq_i);


end architecture;
