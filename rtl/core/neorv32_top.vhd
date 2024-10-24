-- ================================================================================ --
-- NEORV32 SoC - Processor Top Entity                                               --
-- -------------------------------------------------------------------------------- --
-- HQ:           https://github.com/stnolting/neorv32                               --
-- Data Sheet:   https://stnolting.github.io/neorv32                                --
-- User Guide:   https://stnolting.github.io/neorv32/ug                             --
-- Software Ref: https://stnolting.github.io/neorv32/sw/files.html                  --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_top is
  generic (
    -- General --
    CLOCK_FREQUENCY            : natural;                                       -- clock frequency of clk_i in Hz
    CLOCK_GATING_EN            : boolean                        := false;       -- enable clock gating when in sleep mode
    HART_ID                    : std_ulogic_vector(31 downto 0) := x"00000000"; -- hardware thread ID
    JEDEC_ID                   : std_ulogic_vector(10 downto 0) := "00000000000"; -- JEDEC ID: continuation codes + vendor ID
    INT_BOOTLOADER_EN          : boolean                        := false;       -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM

    -- On-Chip Debugger (OCD) --
    ON_CHIP_DEBUGGER_EN        : boolean                        := false;       -- implement on-chip debugger
    DM_LEGACY_MODE             : boolean                        := false;       -- debug module spec version: false = v1.0, true = v0.13

    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A      : boolean                        := false;       -- implement atomic memory operations extension?
    CPU_EXTENSION_RISCV_B      : boolean                        := false;       -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_C      : boolean                        := false;       -- implement compressed extension?
    CPU_EXTENSION_RISCV_E      : boolean                        := false;       -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M      : boolean                        := false;       -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U      : boolean                        := false;       -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx  : boolean                        := false;       -- implement 32-bit floating-point extension (using INT regs!)
    CPU_EXTENSION_RISCV_Zicntr : boolean                        := true;        -- implement base counters?
    CPU_EXTENSION_RISCV_Zicond : boolean                        := false;       -- implement integer conditional operations?
    CPU_EXTENSION_RISCV_Zihpm  : boolean                        := false;       -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zmmul  : boolean                        := false;       -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zxcfu  : boolean                        := false;       -- implement custom (instr.) functions unit?

    -- Tuning Options --
    FAST_MUL_EN                : boolean                        := false;       -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN              : boolean                        := false;       -- use barrel shifter for shift operations
    REGFILE_HW_RST             : boolean                        := false;       -- implement full hardware reset for register file

    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS            : natural range 0 to 16          := 0;           -- number of regions (0..16)
    PMP_MIN_GRANULARITY        : natural                        := 4;           -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
    PMP_TOR_MODE_EN            : boolean                        := true;        -- implement TOR mode
    PMP_NAP_MODE_EN            : boolean                        := true;        -- implement NAPOT/NA4 modes

    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS               : natural range 0 to 13          := 0;           -- number of implemented HPM counters (0..13)
    HPM_CNT_WIDTH              : natural range 0 to 64          := 40;          -- total size of HPM counters (0..64)

    -- Internal Instruction memory (IMEM) --
    MEM_INT_IMEM_EN            : boolean                        := false;       -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE          : natural                        := 16*1024;     -- size of processor-internal instruction memory in bytes (use a power of 2)

    -- Internal Data memory (DMEM) --
    MEM_INT_DMEM_EN            : boolean                        := false;       -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE          : natural                        := 8*1024;      -- size of processor-internal data memory in bytes (use a power of 2)

    -- Internal Instruction Cache (iCACHE) --
    ICACHE_EN                  : boolean                        := false;       -- implement instruction cache
    ICACHE_NUM_BLOCKS          : natural range 1 to 256         := 4;           -- i-cache: number of blocks (min 1), has to be a power of 2
    ICACHE_BLOCK_SIZE          : natural range 4 to 2**16       := 64;          -- i-cache: block size in bytes (min 4), has to be a power of 2

    -- Internal Data Cache (dCACHE) --
    DCACHE_EN                  : boolean                        := false;       -- implement data cache
    DCACHE_NUM_BLOCKS          : natural range 1 to 256         := 4;           -- d-cache: number of blocks (min 1), has to be a power of 2
    DCACHE_BLOCK_SIZE          : natural range 4 to 2**16       := 64;          -- d-cache: block size in bytes (min 4), has to be a power of 2

    -- External bus interface (XBUS) --
    XBUS_EN                    : boolean                        := false;       -- implement external memory bus interface?
    XBUS_TIMEOUT               : natural                        := 255;         -- cycles after a pending bus access auto-terminates (0 = disabled)
    XBUS_REGSTAGE_EN           : boolean                        := false;       -- add XBUS register stage
    XBUS_CACHE_EN              : boolean                        := false;       -- enable external bus cache (x-cache)
    XBUS_CACHE_NUM_BLOCKS      : natural range 1 to 256         := 64;          -- x-cache: number of blocks (min 1), has to be a power of 2
    XBUS_CACHE_BLOCK_SIZE      : natural range 1 to 2**16       := 32;          -- x-cache: block size in bytes (min 4), has to be a power of 2

    -- Execute in-place module (XIP) --
    XIP_EN                     : boolean                        := false;       -- implement execute in place module (XIP)?
    XIP_CACHE_EN               : boolean                        := false;       -- implement XIP cache?
    XIP_CACHE_NUM_BLOCKS       : natural range 1 to 256         := 8;           -- number of blocks (min 1), has to be a power of 2
    XIP_CACHE_BLOCK_SIZE       : natural range 1 to 2**16       := 256;         -- block size in bytes (min 4), has to be a power of 2

    -- External Interrupts Controller (XIRQ) --
    XIRQ_NUM_CH                : natural range 0 to 32          := 0;           -- number of external IRQ channels (0..32)

    -- Processor peripherals --
    IO_DISABLE_SYSINFO         : boolean                        := false;       -- disable the SYSINFO module (for advanced users only)
    IO_GPIO_NUM                : natural range 0 to 64          := 0;           -- number of GPIO input/output pairs (0..64)
    IO_MTIME_EN                : boolean                        := false;       -- implement machine system timer (MTIME)?
    IO_UART0_EN                : boolean                        := false;       -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_UART0_RX_FIFO           : natural range 1 to 2**15       := 1;           -- RX fifo depth, has to be a power of two, min 1
    IO_UART0_TX_FIFO           : natural range 1 to 2**15       := 1;           -- TX fifo depth, has to be a power of two, min 1
    IO_UART1_EN                : boolean                        := false;       -- implement secondary universal asynchronous receiver/transmitter (UART1)?
    IO_UART1_RX_FIFO           : natural range 1 to 2**15       := 1;           -- RX fifo depth, has to be a power of two, min 1
    IO_UART1_TX_FIFO           : natural range 1 to 2**15       := 1;           -- TX fifo depth, has to be a power of two, min 1
    IO_SPI_EN                  : boolean                        := false;       -- implement serial peripheral interface (SPI)?
    IO_SPI_FIFO                : natural range 1 to 2**15       := 1;           -- RTX fifo depth, has to be a power of two, min 1
    IO_SDI_EN                  : boolean                        := false;       -- implement serial data interface (SDI)?
    IO_SDI_FIFO                : natural range 1 to 2**15       := 1;           -- RTX fifo depth, has to be zero or a power of two, min 1
    IO_TWI_EN                  : boolean                        := false;       -- implement two-wire interface (TWI)?
    IO_TWI_FIFO                : natural range 1 to 2**15       := 1;           -- RTX fifo depth, has to be zero or a power of two, min 1
    IO_PWM_NUM_CH              : natural range 0 to 12          := 0;           -- number of PWM channels to implement (0..12); 0 = disabled
    IO_WDT_EN                  : boolean                        := false;       -- implement watch dog timer (WDT)?
    IO_TRNG_EN                 : boolean                        := false;       -- implement true random number generator (TRNG)?
    IO_TRNG_FIFO               : natural range 1 to 2**15       := 1;           -- data fifo depth, has to be a power of two, min 1
    IO_CFS_EN                  : boolean                        := false;       -- implement custom functions subsystem (CFS)?
    IO_CFS_CONFIG              : std_ulogic_vector(31 downto 0) := x"00000000"; -- custom CFS configuration generic
    IO_CFS_IN_SIZE             : natural                        := 32;          -- size of CFS input conduit in bits
    IO_CFS_OUT_SIZE            : natural                        := 32;          -- size of CFS output conduit in bits
    IO_NEOLED_EN               : boolean                        := false;       -- implement NeoPixel-compatible smart LED interface (NEOLED)?
    IO_NEOLED_TX_FIFO          : natural range 1 to 2**15       := 1;           -- NEOLED FIFO depth, has to be a power of two, min 1
    IO_GPTMR_EN                : boolean                        := false;       -- implement general purpose timer (GPTMR)?
    IO_ONEWIRE_EN              : boolean                        := false;       -- implement 1-wire interface (ONEWIRE)?
    IO_DMA_EN                  : boolean                        := false;       -- implement direct memory access controller (DMA)?
    IO_SLINK_EN                : boolean                        := false;       -- implement stream link interface (SLINK)?
    IO_SLINK_RX_FIFO           : natural range 1 to 2**15       := 1;           -- RX fifo depth, has to be a power of two, min 1
    IO_SLINK_TX_FIFO           : natural range 1 to 2**15       := 1;           -- TX fifo depth, has to be a power of two, min 1
    IO_CRC_EN                  : boolean                        := false;        -- implement cyclic redundancy check unit (CRC)?
  
    SIGCOUNT_DEBOUNCE_LIMIT    : natural                        := 500000
  );
  port (
    -- Global control --
    clk_i          : in  std_ulogic;                                        -- global clock, rising edge
    rstn_i         : in  std_ulogic;                                        -- global reset, low-active, async

    -- JTAG on-chip debugger interface (available if ON_CHIP_DEBUGGER_EN = true) --
    jtag_tck_i     : in  std_ulogic := 'L';                                 -- serial clock
    jtag_tdi_i     : in  std_ulogic := 'L';                                 -- serial data input
    jtag_tdo_o     : out std_ulogic;                                        -- serial data output
    jtag_tms_i     : in  std_ulogic := 'L';                                 -- mode select

    -- External bus interface (available if XBUS_EN = true) --
    xbus_adr_o     : out std_ulogic_vector(31 downto 0);                    -- address
    xbus_dat_o     : out std_ulogic_vector(31 downto 0);                    -- write data
    xbus_tag_o     : out std_ulogic_vector(2 downto 0);                     -- access tag
    xbus_we_o      : out std_ulogic;                                        -- read/write
    xbus_sel_o     : out std_ulogic_vector(3 downto 0);                     -- byte enable
    xbus_stb_o     : out std_ulogic;                                        -- strobe
    xbus_cyc_o     : out std_ulogic;                                        -- valid cycle
    xbus_dat_i     : in  std_ulogic_vector(31 downto 0) := (others => 'L'); -- read data
    xbus_ack_i     : in  std_ulogic := 'L';                                 -- transfer acknowledge
    xbus_err_i     : in  std_ulogic := 'L';                                 -- transfer error

    -- Stream Link Interface (available if IO_SLINK_EN = true) --
    slink_rx_dat_i : in  std_ulogic_vector(31 downto 0) := (others => 'L'); -- RX input data
    slink_rx_src_i : in  std_ulogic_vector(3 downto 0)  := (others => 'L'); -- RX source routing information
    slink_rx_val_i : in  std_ulogic := 'L';                                 -- RX valid input
    slink_rx_lst_i : in  std_ulogic := 'L';                                 -- RX last element of stream
    slink_rx_rdy_o : out std_ulogic;                                        -- RX ready to receive
    slink_tx_dat_o : out std_ulogic_vector(31 downto 0);                    -- TX output data
    slink_tx_dst_o : out std_ulogic_vector(3 downto 0);                     -- TX destination routing information
    slink_tx_val_o : out std_ulogic;                                        -- TX valid output
    slink_tx_lst_o : out std_ulogic;                                        -- TX last element of stream
    slink_tx_rdy_i : in  std_ulogic := 'L';                                 -- TX ready to send

    -- XIP (execute in place via SPI) signals (available if XIP_EN = true) --
    xip_csn_o      : out std_ulogic;                                        -- chip-select, low-active
    xip_clk_o      : out std_ulogic;                                        -- serial clock
    xip_dat_i      : in  std_ulogic := 'L';                                 -- device data input
    xip_dat_o      : out std_ulogic;                                        -- controller data output

    -- GPIO (available if IO_GPIO_NUM > 0) --
    gpio_o         : out std_ulogic_vector(63 downto 0);                    -- parallel output
    gpio_i         : in  std_ulogic_vector(63 downto 0) := (others => 'L'); -- parallel input


    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o    : out std_ulogic;                                        -- UART0 send data
    uart0_rxd_i    : in  std_ulogic := 'L';                                 -- UART0 receive data
    uart0_rts_o    : out std_ulogic;                                        -- HW flow control: UART0.RX ready to receive ("RTR"), low-active, optional
    uart0_cts_i    : in  std_ulogic := 'L';                                 -- HW flow control: UART0.TX allowed to transmit, low-active, optional

    -- secondary UART1 (available if IO_UART1_EN = true) --
    uart1_txd_o    : out std_ulogic;                                        -- UART1 send data
    uart1_rxd_i    : in  std_ulogic := 'L';                                 -- UART1 receive data
    uart1_rts_o    : out std_ulogic;                                        -- HW flow control: UART1.RX ready to receive ("RTR"), low-active, optional
    uart1_cts_i    : in  std_ulogic := 'L';                                 -- HW flow control: UART1.TX allowed to transmit, low-active, optional

    -- SPI (available if IO_SPI_EN = true) --
    spi_clk_o      : out std_ulogic;                                        -- SPI serial clock
    spi_dat_o      : out std_ulogic;                                        -- controller data out, peripheral data in
    spi_dat_i      : in  std_ulogic := 'L';                                 -- controller data in, peripheral data out
    spi_csn_o      : out std_ulogic_vector(7 downto 0);                     -- chip-select, low-active

    -- SDI (available if IO_SDI_EN = true) --
    sdi_clk_i      : in  std_ulogic := 'L';                                 -- SDI serial clock
    sdi_dat_o      : out std_ulogic;                                        -- controller data out, peripheral data in
    sdi_dat_i      : in  std_ulogic := 'L';                                 -- controller data in, peripheral data out
    sdi_csn_i      : in  std_ulogic := 'H';                                 -- chip-select, low-active

    -- TWI (available if IO_TWI_EN = true) --
    twi_sda_i      : in  std_ulogic := 'H';                                 -- serial data line sense input
    twi_sda_o      : out std_ulogic;                                        -- serial data line output (pull low only)
    twi_scl_i      : in  std_ulogic := 'H';                                 -- serial clock line sense input
    twi_scl_o      : out std_ulogic;                                        -- serial clock line output (pull low only)

    -- 1-Wire Interface (available if IO_ONEWIRE_EN = true) --
    onewire_i      : in  std_ulogic := 'H';                                 -- 1-wire bus sense input
    onewire_o      : out std_ulogic;                                        -- 1-wire bus output (pull low only)

    -- PWM (available if IO_PWM_NUM_CH > 0) --
    pwm_o          : out std_ulogic_vector(11 downto 0);                    -- pwm channels

    -- Custom Functions Subsystem IO (available if IO_CFS_EN = true) --
    cfs_in_i       : in  std_ulogic_vector(IO_CFS_IN_SIZE-1 downto 0) := (others => 'L'); -- custom CFS inputs conduit
    cfs_out_o      : out std_ulogic_vector(IO_CFS_OUT_SIZE-1 downto 0);     -- custom CFS outputs conduit

    -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
    neoled_o       : out std_ulogic;                                        -- async serial data line

    -- Machine timer system time (available if IO_MTIME_EN = true) --
    mtime_time_o   : out std_ulogic_vector(63 downto 0);                    -- current system time

    -- External platform interrupts (available if XIRQ_NUM_CH > 0) --
    xirq_i         : in  std_ulogic_vector(31 downto 0) := (others => 'L'); -- IRQ channels

    -- CPU interrupts --
    mtime_irq_i    : in  std_ulogic := 'L';                                 -- machine timer interrupt, available if IO_MTIME_EN = false
    msw_irq_i      : in  std_ulogic := 'L';                                 -- machine software interrupt
    mext_irq_i     : in  std_ulogic := 'L';                                 -- machine external interrupt
  
    -- HALL input signal
    hall_signal_i  : in  std_ulogic;                                 -- HALL receive data
  );
end neorv32_top;

architecture neorv32_top_rtl of neorv32_top is

  -- auto-configuration --
  constant cpu_boot_addr_c : std_ulogic_vector(31 downto 0) := cond_sel_suv_f(INT_BOOTLOADER_EN, mem_boot_base_c, mem_imem_base_c);
  constant imem_as_rom_c   : boolean := not INT_BOOTLOADER_EN;
  constant io_gpio_en_c    : boolean := boolean(IO_GPIO_NUM > 0);
  constant io_xirq_en_c    : boolean := boolean(XIRQ_NUM_CH > 0);
  constant io_pwm_en_c     : boolean := boolean(IO_PWM_NUM_CH > 0);
  constant cpu_smpmp_c     : boolean := boolean(PMP_NUM_REGIONS > 0);
  constant io_sysinfo_en_c : boolean := not IO_DISABLE_SYSINFO;

  -- convert JEDEC ID to mvendor CSR --
  constant vendorid_c : std_ulogic_vector(31 downto 0) := x"00000" & "0" & JEDEC_ID;

  -- make sure physical memory sizes are a power of two --
  constant imem_size_c : natural := cond_sel_natural_f(is_power_of_two_f(MEM_INT_IMEM_SIZE), MEM_INT_IMEM_SIZE, 2**index_size_f(MEM_INT_IMEM_SIZE));
  constant dmem_size_c : natural := cond_sel_natural_f(is_power_of_two_f(MEM_INT_DMEM_SIZE), MEM_INT_DMEM_SIZE, 2**index_size_f(MEM_INT_DMEM_SIZE));

  -- reset generator --
  signal rstn_wdt                     : std_ulogic;
  signal rstn_sys_sreg, rstn_ext_sreg : std_ulogic_vector(3 downto 0);
  signal rstn_sys, rstn_ext           : std_ulogic;
  signal rst_cause                    : std_ulogic_vector(1 downto 0);

  -- clock generator --
  signal clk_cpu                   : std_ulogic; -- CPU core clock, can be switched off
  signal clk_div, clk_div_ff       : std_ulogic_vector(11 downto 0);
  signal clk_gen                   : std_ulogic_vector(7 downto 0);
  signal clk_gen_en, clk_gen_en_ff : std_ulogic;
  --
  type cg_en_enum_t is (
    CG_CFS, CG_UART0, CG_UART1, CG_SPI, CG_TWI, CG_PWM, CG_WDT, CG_NEOLED, CG_GPTMR, CG_XIP, CG_ONEWIRE
  );
  type cg_en_t is array (cg_en_enum_t) of std_ulogic;
  signal cg_en : cg_en_t;

  -- CPU status --
  signal cpu_debug, cpu_sleep : std_ulogic; -- cpu is in debug/sleep mode

  -- debug module interface (DMI) --
  signal dmi_req : dmi_req_t;
  signal dmi_rsp : dmi_rsp_t;

  -- debug core interface (DCI) --
  signal dci_ndmrstn, dci_halt_req : std_ulogic;

  -- bus: core complex (CPU + caches) and DMA --
  signal cpu_i_req, cpu_d_req, icache_req, dcache_req, core_req, main_req, main2_req, dma_req : bus_req_t;
  signal cpu_i_rsp, cpu_d_rsp, icache_rsp, dcache_rsp, core_rsp, main_rsp, main2_rsp, dma_rsp : bus_rsp_t;

  -- bus: main sections --
  signal imem_req, dmem_req, xipcache_req, xip_req, boot_req, io_req, xcache_req, xbus_req : bus_req_t;
  signal imem_rsp, dmem_rsp, xipcache_rsp, xip_rsp, boot_rsp, io_rsp, xcache_rsp, xbus_rsp : bus_rsp_t;

  -- bus: IO devices --
  type io_devices_enum_t is (
    IODEV_OCD, IODEV_SYSINFO, IODEV_NEOLED, IODEV_GPIO, IODEV_WDT, IODEV_TRNG, IODEV_TWI,
    IODEV_SPI, IODEV_SDI, IODEV_UART1, IODEV_UART0, IODEV_MTIME, IODEV_XIRQ, IODEV_ONEWIRE,
    IODEV_GPTMR, IODEV_PWM, IODEV_XIP, IODEV_CRC, IODEV_DMA, IODEV_SLINK, IODEV_CFS, IODEV_SIGCOUNT
  );
  type iodev_req_t is array (io_devices_enum_t) of bus_req_t;
  type iodev_rsp_t is array (io_devices_enum_t) of bus_rsp_t;
  signal iodev_req : iodev_req_t;
  signal iodev_rsp : iodev_rsp_t;

  -- IRQs --
  type firq_enum_t is (
    FIRQ_TRNG, FIRQ_UART0_RX, FIRQ_UART0_TX, FIRQ_UART1_RX, FIRQ_UART1_TX, FIRQ_SPI, FIRQ_SDI, FIRQ_TWI,
    FIRQ_CFS, FIRQ_NEOLED, FIRQ_XIRQ, FIRQ_GPTMR, FIRQ_ONEWIRE, FIRQ_DMA, FIRQ_SLINK_RX, FIRQ_SLINK_TX
  );
  type firq_t is array (firq_enum_t) of std_ulogic;
  signal firq      : firq_t;
  signal cpu_firq  : std_ulogic_vector(15 downto 0);
  signal mtime_irq : std_ulogic;

begin

  -- **************************************************************************************************************************
  -- Sanity Checks
  -- **************************************************************************************************************************
  sanity_checks:
  if (true) generate

    -- say hello --
    assert false report
      "[NEORV32] The NEORV32 RISC-V Processor " &
      "(v" & print_version_f(hw_version_c) & "), " &
      "github.com/stnolting/neorv32" severity note;

    -- show SoC configuration --
    assert false report
      "[NEORV32] Processor Configuration: " &
      cond_sel_string_f(MEM_INT_IMEM_EN,           "IMEM ",      "") &
      cond_sel_string_f(MEM_INT_DMEM_EN,           "DMEM ",      "") &
      cond_sel_string_f(INT_BOOTLOADER_EN,         "BOOTROM ",   "") &
      cond_sel_string_f(ICACHE_EN,                 "I-CACHE ",   "") &
      cond_sel_string_f(DCACHE_EN,                 "D-CACHE ",   "") &
      cond_sel_string_f(XBUS_EN,                   "XBUS ",      "") &
      cond_sel_string_f(XBUS_EN and XBUS_CACHE_EN, "X-CACHE ",   "") &
      cond_sel_string_f(XIP_EN,                    "XIP ",       "") &
      cond_sel_string_f(XIP_EN and XIP_CACHE_EN,   "XIP-CACHE ", "") &
      cond_sel_string_f(io_gpio_en_c,              "GPIO ",      "") &
      cond_sel_string_f(IO_MTIME_EN,               "MTIME ",     "") &
      cond_sel_string_f(IO_UART0_EN,               "UART0 ",     "") &
      cond_sel_string_f(IO_UART1_EN,               "UART1 ",     "") &
      cond_sel_string_f(IO_SPI_EN,                 "SPI ",       "") &
      cond_sel_string_f(IO_SDI_EN,                 "SDI ",       "") &
      cond_sel_string_f(IO_TWI_EN,                 "TWI ",       "") &
      cond_sel_string_f(io_pwm_en_c,               "PWM ",       "") &
      cond_sel_string_f(IO_WDT_EN,                 "WDT ",       "") &
      cond_sel_string_f(IO_TRNG_EN,                "TRNG ",      "") &
      cond_sel_string_f(IO_CFS_EN,                 "CFS ",       "") &
      cond_sel_string_f(IO_NEOLED_EN,              "NEOLED ",    "") &
      cond_sel_string_f(io_xirq_en_c,              "XIRQ ",      "") &
      cond_sel_string_f(IO_GPTMR_EN,               "GPTMR ",     "") &
      cond_sel_string_f(IO_ONEWIRE_EN,             "ONEWIRE ",   "") &
      cond_sel_string_f(IO_DMA_EN,                 "DMA ",       "") &
      cond_sel_string_f(IO_SLINK_EN,               "SLINK ",     "") &
      cond_sel_string_f(IO_CRC_EN,                 "CRC ",       "") &
      cond_sel_string_f(io_sysinfo_en_c,           "SYSINFO ",   "") &
      cond_sel_string_f(ON_CHIP_DEBUGGER_EN,       "OCD ",       "") &
      ""
      severity note;

    -- IMEM size was not a power of two --
    assert not ((MEM_INT_IMEM_SIZE /= imem_size_c) and (MEM_INT_IMEM_EN = true)) report
      "[NEORV32] Auto-adjusting invalid IMEM size configuration." severity warning;

    -- DMEM size was not a power of two --
    assert not ((MEM_INT_DMEM_SIZE /= dmem_size_c) and (MEM_INT_DMEM_EN = true)) report
      "[NEORV32] Auto-adjusting invalid DMEM size configuration." severity warning;

    -- SYSINFO disabled --
    assert not (io_sysinfo_en_c = false) report
      "[NEORV32] SYSINFO module disabled - some parts of the NEORV32 software framework will no longer work!" severity warning;

  end generate; -- /sanity_checks


  -- **************************************************************************************************************************
  -- Clock and Reset Generators
  -- **************************************************************************************************************************
  generators:
  if (true) generate

    -- Reset Generator ------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    reset_generator: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        rstn_ext_sreg <= (others => '0');
        rstn_ext      <= '0';
        rstn_sys_sreg <= (others => '0');
        rstn_sys      <= '0';
      elsif rising_edge(clk_i) then -- inverted clock to release reset _before_ all FFs trigger (rising edge)
        -- external reset --
        rstn_ext_sreg <= rstn_ext_sreg(rstn_ext_sreg'left-1 downto 0) & '1'; -- active for at least <rstn_ext_sreg'size> clock cycles
        rstn_ext      <= and_reduce_f(rstn_ext_sreg);
        -- internal reset --
        if (rstn_wdt = '0') or (dci_ndmrstn = '0') then -- sync reset sources
          rstn_sys_sreg <= (others => '0');
        else
          rstn_sys_sreg <= rstn_sys_sreg(rstn_sys_sreg'left-1 downto 0) & '1'; -- active for at least <rstn_sys_sreg'size> clock cycles
        end if;
        rstn_sys <= and_reduce_f(rstn_sys_sreg);
      end if;
    end process reset_generator;


    -- Reset Cause ----------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    reset_cause: process(rstn_ext, clk_i)
    begin
      if (rstn_ext = '0') then
        rst_cause <= "00"; -- reset from external pin
      elsif rising_edge(clk_i) then
        if (dci_ndmrstn = '0') then
          rst_cause <= "01"; -- reset from on-chip debugger
        elsif (rstn_wdt = '0') then
          rst_cause <= "10"; -- reset from watchdog timer
        end if;
      end if;
    end process reset_cause;


    -- Clock Generator ------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    clock_generator: process(rstn_sys, clk_i)
    begin
      if (rstn_sys = '0') then
        clk_gen_en_ff <= '0';
        clk_div       <= (others => '0');
        clk_div_ff    <= (others => '0');
      elsif rising_edge(clk_i) then
        clk_gen_en_ff <= clk_gen_en;
        if (clk_gen_en_ff = '1') then
          clk_div <= std_ulogic_vector(unsigned(clk_div) + 1);
        else -- reset if disabled
          clk_div <= (others => '0');
        end if;
        clk_div_ff <= clk_div;
      end if;
    end process clock_generator;

    -- clock enables: rising edge detectors --
    clk_gen(clk_div2_c)    <= clk_div(0)  and (not clk_div_ff(0));  -- clk/2
    clk_gen(clk_div4_c)    <= clk_div(1)  and (not clk_div_ff(1));  -- clk/4
    clk_gen(clk_div8_c)    <= clk_div(2)  and (not clk_div_ff(2));  -- clk/8
    clk_gen(clk_div64_c)   <= clk_div(5)  and (not clk_div_ff(5));  -- clk/64
    clk_gen(clk_div128_c)  <= clk_div(6)  and (not clk_div_ff(6));  -- clk/128
    clk_gen(clk_div1024_c) <= clk_div(9)  and (not clk_div_ff(9));  -- clk/1024
    clk_gen(clk_div2048_c) <= clk_div(10) and (not clk_div_ff(10)); -- clk/2048
    clk_gen(clk_div4096_c) <= clk_div(11) and (not clk_div_ff(11)); -- clk/4096

    -- fresh clocks anyone? --
    clk_gen_en <= cg_en(CG_WDT)   or cg_en(CG_UART0) or cg_en(CG_UART1) or cg_en(CG_SPI)    or
                  cg_en(CG_TWI)   or cg_en(CG_PWM)   or cg_en(CG_WDT)   or cg_en(CG_NEOLED) or
                  cg_en(CG_GPTMR) or cg_en(CG_XIP)   or cg_en(CG_ONEWIRE);

  end generate; -- /generators


  -- **************************************************************************************************************************
  -- Core Complex
  -- **************************************************************************************************************************
  core_complex:
  if (true) generate

    -- CPU Clock Gating -----------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_cpu_clockgate_inst_true:
    if CLOCK_GATING_EN generate
      neorv32_cpu_clockgate_inst: entity neorv32.neorv32_clockgate
      port map (
        clk_i  => clk_i,
        rstn_i => rstn_sys,
        halt_i => cpu_sleep,
        clk_o  => clk_cpu
      );
    end generate;

    neorv32_cpu_clockgate_inst_false:
    if not CLOCK_GATING_EN generate
      clk_cpu <= clk_i;
    end generate;


    -- CPU Core -------------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_cpu_inst: entity neorv32.neorv32_cpu
    generic map (
      -- General --
      HART_ID                    => HART_ID,
      VENDOR_ID                  => vendorid_c,
      CPU_BOOT_ADDR              => cpu_boot_addr_c,
      CPU_DEBUG_PARK_ADDR        => dm_park_entry_c,
      CPU_DEBUG_EXC_ADDR         => dm_exc_entry_c,
      -- RISC-V CPU Extensions --
      CPU_EXTENSION_RISCV_A      => CPU_EXTENSION_RISCV_A,
      CPU_EXTENSION_RISCV_B      => CPU_EXTENSION_RISCV_B,
      CPU_EXTENSION_RISCV_C      => CPU_EXTENSION_RISCV_C,
      CPU_EXTENSION_RISCV_E      => CPU_EXTENSION_RISCV_E,
      CPU_EXTENSION_RISCV_M      => CPU_EXTENSION_RISCV_M,
      CPU_EXTENSION_RISCV_U      => CPU_EXTENSION_RISCV_U,
      CPU_EXTENSION_RISCV_Zfinx  => CPU_EXTENSION_RISCV_Zfinx,
      CPU_EXTENSION_RISCV_Zicntr => CPU_EXTENSION_RISCV_Zicntr,
      CPU_EXTENSION_RISCV_Zicond => CPU_EXTENSION_RISCV_Zicond,
      CPU_EXTENSION_RISCV_Zihpm  => CPU_EXTENSION_RISCV_Zihpm,
      CPU_EXTENSION_RISCV_Zmmul  => CPU_EXTENSION_RISCV_Zmmul,
      CPU_EXTENSION_RISCV_Zxcfu  => CPU_EXTENSION_RISCV_Zxcfu,
      CPU_EXTENSION_RISCV_Sdext  => ON_CHIP_DEBUGGER_EN,
      CPU_EXTENSION_RISCV_Sdtrig => ON_CHIP_DEBUGGER_EN,
      CPU_EXTENSION_RISCV_Smpmp  => cpu_smpmp_c,
      -- Tuning Options --
      FAST_MUL_EN                => FAST_MUL_EN,
      FAST_SHIFT_EN              => FAST_SHIFT_EN,
      REGFILE_HW_RST             => REGFILE_HW_RST,
      -- Physical Memory Protection (PMP) --
      PMP_NUM_REGIONS            => PMP_NUM_REGIONS,
      PMP_MIN_GRANULARITY        => PMP_MIN_GRANULARITY,
      PMP_TOR_MODE_EN            => PMP_TOR_MODE_EN,
      PMP_NAP_MODE_EN            => PMP_NAP_MODE_EN,
      -- Hardware Performance Monitors (HPM) --
      HPM_NUM_CNTS               => HPM_NUM_CNTS,
      HPM_CNT_WIDTH              => HPM_CNT_WIDTH
    )
    port map (
      -- global control --
      clk_i      => clk_cpu, -- switchable clock
      clk_aux_i  => clk_i,
      rstn_i     => rstn_sys,
      sleep_o    => cpu_sleep,
      debug_o    => cpu_debug,
      -- interrupts --
      msi_i      => msw_irq_i,
      mei_i      => mext_irq_i,
      mti_i      => mtime_irq,
      firq_i     => cpu_firq,
      dbi_i      => dci_halt_req,
      -- instruction bus interface --
      ibus_req_o => cpu_i_req,
      ibus_rsp_i => cpu_i_rsp,
      -- data bus interface --
      dbus_req_o => cpu_d_req,
      dbus_rsp_i => cpu_d_rsp
    );

    -- fast interrupt requests (FIRQs) --
    cpu_firq(0)  <= firq(FIRQ_TRNG);
    cpu_firq(1)  <= firq(FIRQ_CFS);
    cpu_firq(2)  <= firq(FIRQ_UART0_RX);
    cpu_firq(3)  <= firq(FIRQ_UART0_TX);
    cpu_firq(4)  <= firq(FIRQ_UART1_RX);
    cpu_firq(5)  <= firq(FIRQ_UART1_TX);
    cpu_firq(6)  <= firq(FIRQ_SPI);
    cpu_firq(7)  <= firq(FIRQ_TWI);
    cpu_firq(8)  <= firq(FIRQ_XIRQ);
    cpu_firq(9)  <= firq(FIRQ_NEOLED);
    cpu_firq(10) <= firq(FIRQ_DMA);
    cpu_firq(11) <= firq(FIRQ_SDI);
    cpu_firq(12) <= firq(FIRQ_GPTMR);
    cpu_firq(13) <= firq(FIRQ_ONEWIRE);
    cpu_firq(14) <= firq(FIRQ_SLINK_RX);
    cpu_firq(15) <= firq(FIRQ_SLINK_TX);


    -- CPU Instruction Cache (I-Cache) --------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_icache_inst_true:
    if ICACHE_EN generate
      neorv32_icache_inst: entity neorv32.neorv32_cache
      generic map (
        NUM_BLOCKS => ICACHE_NUM_BLOCKS,
        BLOCK_SIZE => ICACHE_BLOCK_SIZE,
        UC_BEGIN   => uncached_begin_c(31 downto 28),
        UC_ENABLE  => true,
        READ_ONLY  => true
      )
      port map (
        clk_i      => clk_i,
        rstn_i     => rstn_sys,
        host_req_i => cpu_i_req,
        host_rsp_o => cpu_i_rsp,
        bus_req_o  => icache_req,
        bus_rsp_i  => icache_rsp
      );
    end generate;

    neorv32_icache_inst_false:
    if not ICACHE_EN generate
      icache_req <= cpu_i_req;
      cpu_i_rsp  <= icache_rsp;
    end generate;


    -- CPU Data Cache (D-Cache) ---------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_dcache_inst_true:
    if DCACHE_EN generate
      neorv32_dcache_inst: entity neorv32.neorv32_cache
      generic map (
        NUM_BLOCKS => DCACHE_NUM_BLOCKS,
        BLOCK_SIZE => DCACHE_BLOCK_SIZE,
        UC_BEGIN   => uncached_begin_c(31 downto 28),
        UC_ENABLE  => true,
        READ_ONLY  => false
      )
      port map (
        clk_i      => clk_i,
        rstn_i     => rstn_sys,
        host_req_i => cpu_d_req,
        host_rsp_o => cpu_d_rsp,
        bus_req_o  => dcache_req,
        bus_rsp_i  => dcache_rsp
      );
    end generate;

    neorv32_dcache_inst_false:
    if not DCACHE_EN generate
      dcache_req <= cpu_d_req;
      cpu_d_rsp  <= dcache_rsp;
    end generate;


    -- Core Complex Bus Switch ----------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_core_bus_switch_inst: entity neorv32.neorv32_bus_switch
    generic map (
      PORT_A_READ_ONLY => false,
      PORT_B_READ_ONLY => true -- i-fetch is read-only
    )
    port map (
      clk_i    => clk_i,
      rstn_i   => rstn_sys,
      a_lock_i => '0',        -- no exclusive accesses for port A
      a_req_i  => dcache_req, -- prioritized
      a_rsp_o  => dcache_rsp,
      b_req_i  => icache_req,
      b_rsp_o  => icache_rsp,
      x_req_o  => core_req,
      x_rsp_i  => core_rsp
    );

  end generate; -- /core_complex


  -- **************************************************************************************************************************
  -- Direct Memory Access Controller (DMA) Complex
  -- **************************************************************************************************************************
  neorv32_dma_complex_true:
  if IO_DMA_EN generate

    -- DMA Controller -------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_dma_inst: entity neorv32.neorv32_dma
    port map (
      clk_i     => clk_i,
      rstn_i    => rstn_sys,
      bus_req_i => iodev_req(IODEV_DMA),
      bus_rsp_o => iodev_rsp(IODEV_DMA),
      dma_req_o => dma_req,
      dma_rsp_i => dma_rsp,
      firq_i    => cpu_firq,
      irq_o     => firq(FIRQ_DMA)
    );


    -- DMA Bus Switch -------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_dma_bus_switch_inst: entity neorv32.neorv32_bus_switch
    generic map (
      PORT_A_READ_ONLY => false,
      PORT_B_READ_ONLY => false
    )
    port map (
      clk_i    => clk_i,
      rstn_i   => rstn_sys,
      a_lock_i => '0',      -- no exclusive accesses for port A
      a_req_i  => core_req, -- prioritized
      a_rsp_o  => core_rsp,
      b_req_i  => dma_req,
      b_rsp_o  => dma_rsp,
      x_req_o  => main_req,
      x_rsp_i  => main_rsp
    );

  end generate; -- /neorv32_dma_complex_true

  neorv32_dma_complex_false:
  if not IO_DMA_EN generate
    iodev_rsp(IODEV_DMA) <= rsp_terminate_c;
    main_req             <= core_req;
    core_rsp             <= main_rsp;
    firq(FIRQ_DMA)       <= '0';
  end generate;


  -- **************************************************************************************************************************
  -- Reservation Set Controller (for atomic LR/SC accesses)
  -- **************************************************************************************************************************
  neorv32_bus_reservation_set_true:
  if CPU_EXTENSION_RISCV_A generate
    neorv32_bus_reservation_set_inst: entity neorv32.neorv32_bus_reservation_set
    port map (
      clk_i       => clk_i,
      rstn_i      => rstn_sys,
      rvs_addr_o  => open, -- yet unused
      rvs_valid_o => open, -- yet unused
      rvs_clear_i => '0',  -- yet unused
      core_req_i  => main_req,
      core_rsp_o  => main_rsp,
      sys_req_o   => main2_req,
      sys_rsp_i   => main2_rsp
    );
  end generate;

  neorv32_bus_reservation_set_false:
  if not CPU_EXTENSION_RISCV_A generate
    main2_req <= main_req;
    main_rsp  <= main2_rsp;
  end generate;


  -- **************************************************************************************************************************
  -- Address Region Gateway
  -- **************************************************************************************************************************
  neorv32_bus_gateway_inst: entity neorv32.neorv32_bus_gateway
  generic map (
    TIMEOUT  => bus_timeout_c,
    -- port A: internal IMEM --
    A_ENABLE => MEM_INT_IMEM_EN,
    A_BASE   => mem_imem_base_c,
    A_SIZE   => imem_size_c,
    A_TMO_EN => true,
    A_PRIV   => false,
    -- port B: internal DMEM --
    B_ENABLE => MEM_INT_DMEM_EN,
    B_BASE   => mem_dmem_base_c,
    B_SIZE   => dmem_size_c,
    B_TMO_EN => true,
    B_PRIV   => false,
    -- port C: XIP --
    C_ENABLE => XIP_EN,
    C_BASE   => mem_xip_base_c,
    C_SIZE   => mem_xip_size_c,
    C_TMO_EN => false, -- no timeout for XIP accesses
    C_PRIV   => false,
    -- port D: BOOT ROM --
    D_ENABLE => INT_BOOTLOADER_EN,
    D_BASE   => mem_boot_base_c,
    D_SIZE   => mem_boot_size_c,
    D_TMO_EN => true,
    D_PRIV   => true, -- only privileged (M-mode) accesses are allowed
    -- port E: IO --
    E_ENABLE => true, -- always enabled (but will be trimmed if no IO devices are implemented)
    E_BASE   => mem_io_base_c,
    E_SIZE   => mem_io_size_c,
    E_TMO_EN => true,
    E_PRIV   => true, -- only privileged (M-mode) accesses are allowed
    -- port X (the void): XBUS --
    X_ENABLE => XBUS_EN,
    X_TMO_EN => false, -- timeout handled by XBUS gateway
    X_PRIV   => false
  )
  port map (
    -- global control --
    clk_i   => clk_i,
    rstn_i  => rstn_sys,
    -- host port --
    req_i   => main2_req,
    rsp_o   => main2_rsp,
    -- section ports --
    a_req_o => imem_req,
    a_rsp_i => imem_rsp,
    b_req_o => dmem_req,
    b_rsp_i => dmem_rsp,
    c_req_o => xip_req,
    c_rsp_i => xip_rsp,
    d_req_o => boot_req,
    d_rsp_i => boot_rsp,
    e_req_o => io_req,
    e_rsp_i => io_rsp,
    x_req_o => xbus_req,
    x_rsp_i => xbus_rsp
  );


  -- **************************************************************************************************************************
  -- Memory System
  -- **************************************************************************************************************************
  memory_system:
  if (true) generate

    -- Processor-Internal Instruction Memory (IMEM) -------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_int_imem_inst_true:
    if MEM_INT_IMEM_EN generate
      neorv32_int_imem_inst: entity neorv32.neorv32_imem
      generic map (
        IMEM_SIZE    => imem_size_c,
        IMEM_AS_IROM => imem_as_rom_c
      )
      port map (
        clk_i     => clk_i,
        rstn_i    => rstn_sys,
        bus_req_i => imem_req,
        bus_rsp_o => imem_rsp
      );
    end generate;

    neorv32_int_imem_inst_false:
    if not MEM_INT_IMEM_EN generate
      imem_rsp <= rsp_terminate_c;
    end generate;


    -- Processor-Internal Data Memory (DMEM) --------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_int_dmem_inst_true:
    if MEM_INT_DMEM_EN generate
      neorv32_int_dmem_inst: entity neorv32.neorv32_dmem
      generic map (
        DMEM_SIZE => dmem_size_c
      )
      port map (
        clk_i     => clk_i,
        rstn_i    => rstn_sys,
        bus_req_i => dmem_req,
        bus_rsp_o => dmem_rsp
      );
    end generate;

    neorv32_int_dmem_inst_false:
    if not MEM_INT_DMEM_EN generate
      dmem_rsp <= rsp_terminate_c;
    end generate;


    -- Processor-Internal Bootloader ROM (BOOTROM) --------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_boot_rom_inst_true:
    if INT_BOOTLOADER_EN generate
      neorv32_boot_rom_inst: entity neorv32.neorv32_boot_rom
      port map (
        clk_i     => clk_i,
        rstn_i    => rstn_sys,
        bus_req_i => boot_req,
        bus_rsp_o => boot_rsp
      );
    end generate;

    neorv32_boot_rom_inst_false:
    if not INT_BOOTLOADER_EN generate
      boot_rsp <= rsp_terminate_c;
    end generate;


    -- Execute In-Place Module (XIP) ----------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_xip_inst_true:
    if XIP_EN generate

      -- XIP interface --
      neorv32_xip_inst: entity neorv32.neorv32_xip
      generic map (
        XIP_CACHE_EN => XIP_CACHE_EN
      )
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => iodev_req(IODEV_XIP),
        bus_rsp_o   => iodev_rsp(IODEV_XIP),
        xip_req_i   => xipcache_req,
        xip_rsp_o   => xipcache_rsp,
        clkgen_en_o => cg_en(CG_XIP),
        clkgen_i    => clk_gen,
        spi_csn_o   => xip_csn_o,
        spi_clk_o   => xip_clk_o,
        spi_dat_i   => xip_dat_i,
        spi_dat_o   => xip_dat_o
      );

      -- XIP cache (XIP-CACHE) --
      neorv32_xipcache_inst_true:
      if XIP_CACHE_EN generate
        neorv32_xcache_inst: entity neorv32.neorv32_cache
        generic map (
          NUM_BLOCKS => XIP_CACHE_NUM_BLOCKS,
          BLOCK_SIZE => XIP_CACHE_BLOCK_SIZE,
          UC_BEGIN   => (others => '0'),
          UC_ENABLE  => false,
          READ_ONLY  => true
        )
        port map (
          clk_i      => clk_i,
          rstn_i     => rstn_sys,
          host_req_i => xip_req,
          host_rsp_o => xip_rsp,
          bus_req_o  => xipcache_req,
          bus_rsp_i  => xipcache_rsp
        );
      end generate;

      neorv32_xipcache_inst_false:
      if not XIP_CACHE_EN generate
        xipcache_req <= xip_req;
        xip_rsp      <= xipcache_rsp;
      end generate;

    end generate; -- /neorv32_xip_inst_true

    neorv32_xip_inst_false:
    if not XIP_EN generate
      iodev_rsp(IODEV_XIP) <= rsp_terminate_c;
      xip_rsp              <= rsp_terminate_c;
      cg_en(CG_XIP)        <= '0';
      xip_csn_o            <= '1';
      xip_clk_o            <= '0';
      xip_dat_o            <= '0';
    end generate;


    -- External Bus Interface (XBUS) ----------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_xbus_inst_true:
    if XBUS_EN generate

      -- external bus gateway (XBUS) --
      neorv32_xbus_inst: entity neorv32.neorv32_xbus
      generic map (
        TIMEOUT_VAL => XBUS_TIMEOUT,
        REGSTAGE_EN => XBUS_REGSTAGE_EN
      )
      port map (
        clk_i      => clk_i,
        rstn_i     => rstn_sys,
        bus_req_i  => xcache_req,
        bus_rsp_o  => xcache_rsp,
        --
        xbus_adr_o => xbus_adr_o,
        xbus_dat_i => xbus_dat_i,
        xbus_dat_o => xbus_dat_o,
        xbus_tag_o => xbus_tag_o,
        xbus_we_o  => xbus_we_o,
        xbus_sel_o => xbus_sel_o,
        xbus_stb_o => xbus_stb_o,
        xbus_cyc_o => xbus_cyc_o,
        xbus_ack_i => xbus_ack_i,
        xbus_err_i => xbus_err_i
      );

      -- external bus cache (X-CACHE) --
      neorv32_xcache_inst_true:
      if XBUS_CACHE_EN generate
        neorv32_xcache_inst: entity neorv32.neorv32_cache
        generic map (
          NUM_BLOCKS => XBUS_CACHE_NUM_BLOCKS,
          BLOCK_SIZE => XBUS_CACHE_BLOCK_SIZE,
          UC_BEGIN   => uncached_begin_c(31 downto 28),
          UC_ENABLE  => true,
          READ_ONLY  => false
        )
        port map (
          clk_i      => clk_i,
          rstn_i     => rstn_sys,
          host_req_i => xbus_req,
          host_rsp_o => xbus_rsp,
          bus_req_o  => xcache_req,
          bus_rsp_i  => xcache_rsp
        );
      end generate;

      neorv32_xcache_inst_false:
      if not XBUS_CACHE_EN generate
        xcache_req <= xbus_req;
        xbus_rsp   <= xcache_rsp;
      end generate;

    end generate; -- /neorv32_xbus_inst_true

    neorv32_xbus_inst_false:
    if not XBUS_EN generate
      xbus_rsp   <= rsp_terminate_c;
      xbus_adr_o <= (others => '0');
      xbus_dat_o <= (others => '0');
      xbus_tag_o <= (others => '0');
      xbus_we_o  <= '0';
      xbus_sel_o <= (others => '0');
      xbus_stb_o <= '0';
      xbus_cyc_o <= '0';
    end generate;

  end generate; -- /memory_system





  -- **************************************************************************************************************************
  -- IO/Peripheral Modules
  -- **************************************************************************************************************************
  io_system:
  if (true) generate

    -- IO Switch ------------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_bus_io_switch_inst: entity neorv32.neorv32_bus_io_switch
    generic map (
      DEV_SIZE  => iodev_size_c, -- size of a single IO device
      DEV_00_EN => ON_CHIP_DEBUGGER_EN, DEV_00_BASE => base_io_dm_c,
      DEV_01_EN => io_sysinfo_en_c,     DEV_01_BASE => base_io_sysinfo_c,
      DEV_02_EN => IO_NEOLED_EN,        DEV_02_BASE => base_io_neoled_c,
      DEV_03_EN => io_gpio_en_c,        DEV_03_BASE => base_io_gpio_c,
      DEV_04_EN => IO_WDT_EN,           DEV_04_BASE => base_io_wdt_c,
      DEV_05_EN => IO_TRNG_EN,          DEV_05_BASE => base_io_trng_c,
      DEV_06_EN => IO_TWI_EN,           DEV_06_BASE => base_io_twi_c,
      DEV_07_EN => IO_SPI_EN,           DEV_07_BASE => base_io_spi_c,
      DEV_08_EN => IO_SDI_EN,           DEV_08_BASE => base_io_sdi_c,
      DEV_09_EN => IO_UART1_EN,         DEV_09_BASE => base_io_uart1_c,
      DEV_10_EN => IO_UART0_EN,         DEV_10_BASE => base_io_uart0_c,
      DEV_11_EN => IO_MTIME_EN,         DEV_11_BASE => base_io_mtime_c,
      DEV_12_EN => io_xirq_en_c,        DEV_12_BASE => base_io_xirq_c,
      DEV_13_EN => IO_ONEWIRE_EN,       DEV_13_BASE => base_io_onewire_c,
      DEV_14_EN => IO_GPTMR_EN,         DEV_14_BASE => base_io_gptmr_c,
      DEV_15_EN => io_pwm_en_c,         DEV_15_BASE => base_io_pwm_c,
      DEV_16_EN => XIP_EN,              DEV_16_BASE => base_io_xip_c,
      DEV_17_EN => IO_CRC_EN,           DEV_17_BASE => base_io_crc_c,
      DEV_18_EN => IO_DMA_EN,           DEV_18_BASE => base_io_dma_c,
      DEV_19_EN => IO_SLINK_EN,         DEV_19_BASE => base_io_slink_c,
      DEV_20_EN => IO_CFS_EN,           DEV_20_BASE => base_io_cfs_c,

      DEV_21_EN => true,                DEV_21_BASE => base_io_sigcount_c, -- Always enable sigcount

      DEV_22_EN => false,               DEV_22_BASE => (others => '0'), -- reserved
      DEV_23_EN => false,               DEV_23_BASE => (others => '0'), -- reserved
      DEV_24_EN => false,               DEV_24_BASE => (others => '0'), -- reserved
      DEV_25_EN => false,               DEV_25_BASE => (others => '0'), -- reserved
      DEV_26_EN => false,               DEV_26_BASE => (others => '0'), -- reserved
      DEV_27_EN => false,               DEV_27_BASE => (others => '0'), -- reserved
      DEV_28_EN => false,               DEV_28_BASE => (others => '0'), -- reserved
      DEV_29_EN => false,               DEV_29_BASE => (others => '0'), -- reserved
      DEV_30_EN => false,               DEV_30_BASE => (others => '0'), -- reserved
      DEV_31_EN => false,               DEV_31_BASE => (others => '0')  -- reserved
    )
    port map (
      clk_i        => clk_i,
      rstn_i       => rstn_sys,
      main_req_i   => io_req,
      main_rsp_o   => io_rsp,
      dev_00_req_o => iodev_req(IODEV_OCD),     dev_00_rsp_i => iodev_rsp(IODEV_OCD),
      dev_01_req_o => iodev_req(IODEV_SYSINFO), dev_01_rsp_i => iodev_rsp(IODEV_SYSINFO),
      dev_02_req_o => iodev_req(IODEV_NEOLED),  dev_02_rsp_i => iodev_rsp(IODEV_NEOLED),
      dev_03_req_o => iodev_req(IODEV_GPIO),    dev_03_rsp_i => iodev_rsp(IODEV_GPIO),
      dev_04_req_o => iodev_req(IODEV_WDT),     dev_04_rsp_i => iodev_rsp(IODEV_WDT),
      dev_05_req_o => iodev_req(IODEV_TRNG),    dev_05_rsp_i => iodev_rsp(IODEV_TRNG),
      dev_06_req_o => iodev_req(IODEV_TWI),     dev_06_rsp_i => iodev_rsp(IODEV_TWI),
      dev_07_req_o => iodev_req(IODEV_SPI),     dev_07_rsp_i => iodev_rsp(IODEV_SPI),
      dev_08_req_o => iodev_req(IODEV_SDI),     dev_08_rsp_i => iodev_rsp(IODEV_SDI),
      dev_09_req_o => iodev_req(IODEV_UART1),   dev_09_rsp_i => iodev_rsp(IODEV_UART1),
      dev_10_req_o => iodev_req(IODEV_UART0),   dev_10_rsp_i => iodev_rsp(IODEV_UART0),
      dev_11_req_o => iodev_req(IODEV_MTIME),   dev_11_rsp_i => iodev_rsp(IODEV_MTIME),
      dev_12_req_o => iodev_req(IODEV_XIRQ),    dev_12_rsp_i => iodev_rsp(IODEV_XIRQ),
      dev_13_req_o => iodev_req(IODEV_ONEWIRE), dev_13_rsp_i => iodev_rsp(IODEV_ONEWIRE),
      dev_14_req_o => iodev_req(IODEV_GPTMR),   dev_14_rsp_i => iodev_rsp(IODEV_GPTMR),
      dev_15_req_o => iodev_req(IODEV_PWM),     dev_15_rsp_i => iodev_rsp(IODEV_PWM),
      dev_16_req_o => iodev_req(IODEV_XIP),     dev_16_rsp_i => iodev_rsp(IODEV_XIP),
      dev_17_req_o => iodev_req(IODEV_CRC),     dev_17_rsp_i => iodev_rsp(IODEV_CRC),
      dev_18_req_o => iodev_req(IODEV_DMA),     dev_18_rsp_i => iodev_rsp(IODEV_DMA),
      dev_19_req_o => iodev_req(IODEV_SLINK),   dev_19_rsp_i => iodev_rsp(IODEV_SLINK),
      dev_20_req_o => iodev_req(IODEV_CFS),     dev_20_rsp_i => iodev_rsp(IODEV_CFS),

      dev_21_req_o => iodev_req(IODEV_SIGCOUNT),dev_21_rsp_i => iodev_rsp(IODEV_SIGCOUNT),
  
      dev_22_req_o => open,                     dev_22_rsp_i => rsp_terminate_c, -- reserved
      dev_23_req_o => open,                     dev_23_rsp_i => rsp_terminate_c, -- reserved
      dev_24_req_o => open,                     dev_24_rsp_i => rsp_terminate_c, -- reserved
      dev_25_req_o => open,                     dev_25_rsp_i => rsp_terminate_c, -- reserved
      dev_26_req_o => open,                     dev_26_rsp_i => rsp_terminate_c, -- reserved
      dev_27_req_o => open,                     dev_27_rsp_i => rsp_terminate_c, -- reserved
      dev_28_req_o => open,                     dev_28_rsp_i => rsp_terminate_c, -- reserved
      dev_29_req_o => open,                     dev_29_rsp_i => rsp_terminate_c, -- reserved
      dev_30_req_o => open,                     dev_30_rsp_i => rsp_terminate_c, -- reserved
      dev_31_req_o => open,                     dev_31_rsp_i => rsp_terminate_c  -- reserved
    );


    -- Custom Functions Subsystem (CFS) -------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_cfs_inst_true:
    if IO_CFS_EN generate
      neorv32_cfs_inst: entity neorv32.neorv32_cfs
      generic map (
        CFS_CONFIG   => IO_CFS_CONFIG,
        CFS_IN_SIZE  => IO_CFS_IN_SIZE,
        CFS_OUT_SIZE => IO_CFS_OUT_SIZE
      )
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => iodev_req(IODEV_CFS),
        bus_rsp_o   => iodev_rsp(IODEV_CFS),
        clkgen_en_o => cg_en(CG_CFS),
        clkgen_i    => clk_gen,
        irq_o       => firq(FIRQ_CFS),
        cfs_in_i    => cfs_in_i,
        cfs_out_o   => cfs_out_o
      );
    end generate;

    neorv32_cfs_inst_false:
    if not IO_CFS_EN generate
      iodev_rsp(IODEV_CFS) <= rsp_terminate_c;
      cg_en(CG_CFS)        <= '0';
      firq(FIRQ_CFS)       <= '0';
      cfs_out_o            <= (others => '0');
    end generate;


    -- Serial Data Interface (SDI) ------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_sdi_inst_true:
    if IO_SDI_EN generate
      neorv32_sdi_inst: entity neorv32.neorv32_sdi
      generic map (
        RTX_FIFO => IO_SDI_FIFO
      )
      port map (
        clk_i     => clk_i,
        rstn_i    => rstn_sys,
        bus_req_i => iodev_req(IODEV_SDI),
        bus_rsp_o => iodev_rsp(IODEV_SDI),
        sdi_csn_i => sdi_csn_i,
        sdi_clk_i => sdi_clk_i,
        sdi_dat_i => sdi_dat_i,
        sdi_dat_o => sdi_dat_o,
        irq_o     => firq(FIRQ_SDI)
      );
    end generate;

    neorv32_sdi_inst_false:
    if not IO_SDI_EN generate
      iodev_rsp(IODEV_SDI) <= rsp_terminate_c;
      sdi_dat_o            <= '0';
      firq(FIRQ_SDI)       <= '0';
    end generate;


    -- Signal Counter -------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_sigcount_inst: entity neorv32.neorv32_sigcount
        generic map (
            DEBOUNCE_LIMIT => SIGCOUNT_DEBOUNCE_LIMIT  -- Defina o valor desejado para o limite de debounce
        )
        port map (
            clk_i        => clk_i,
            rstn_i       => rstn_sys,
            button_i     => hall_signal_i,          -- Sinal de entrada do botão
            bus_req_i    => iodev_req(IODEV_SIGCOUNT),    -- Conectar a requisição do barramento
            bus_rsp_o    => iodev_rsp(IODEV_SIGCOUNT)    -- Conectar a resposta do barramento
            -- counter_o    => counter_output_signal          -- Sinal de saída do contador FOR DEBUG
        );


    -- General Purpose Input/Output Port (GPIO) -----------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_gpio_inst_true:
    if io_gpio_en_c generate
      neorv32_gpio_inst: entity neorv32.neorv32_gpio
      generic map (
        GPIO_NUM => IO_GPIO_NUM
      )
      port map (
        clk_i     => clk_i,
        rstn_i    => rstn_sys,
        bus_req_i => iodev_req(IODEV_GPIO),
        bus_rsp_o => iodev_rsp(IODEV_GPIO),
        gpio_o    => gpio_o,
        gpio_i    => gpio_i
      );
    end generate;

    neorv32_gpio_inst_false:
    if not io_gpio_en_c generate
      iodev_rsp(IODEV_GPIO) <= rsp_terminate_c;
      gpio_o                <= (others => '0');
    end generate;


    -- Watch Dog Timer (WDT) ------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_wdt_inst_true:
    if IO_WDT_EN generate
      neorv32_wdt_inst: entity neorv32.neorv32_wdt
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        rst_cause_i => rst_cause,
        bus_req_i   => iodev_req(IODEV_WDT),
        bus_rsp_o   => iodev_rsp(IODEV_WDT),
        cpu_debug_i => cpu_debug,
        cpu_sleep_i => cpu_sleep,
        clkgen_en_o => cg_en(CG_WDT),
        clkgen_i    => clk_gen,
        rstn_o      => rstn_wdt
      );
    end generate;

    neorv32_wdt_inst_false:
    if not IO_WDT_EN generate
      iodev_rsp(IODEV_WDT) <= rsp_terminate_c;
      cg_en(CG_WDT)        <= '0';
      rstn_wdt             <= '1';
    end generate;


    -- Machine System Timer (MTIME) -----------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_mtime_inst_true:
    if IO_MTIME_EN generate
      neorv32_mtime_inst: entity neorv32.neorv32_mtime
      port map (
        clk_i     => clk_i,
        rstn_i    => rstn_sys,
        bus_req_i => iodev_req(IODEV_MTIME),
        bus_rsp_o => iodev_rsp(IODEV_MTIME),
        time_o    => mtime_time_o,
        irq_o     => mtime_irq
      );
    end generate;

    neorv32_mtime_inst_false:
    if not IO_MTIME_EN generate
      iodev_rsp(IODEV_MTIME) <= rsp_terminate_c;
      mtime_time_o           <= (others => '0');
      mtime_irq              <= mtime_irq_i;
    end generate;


    -- Primary Universal Asynchronous Receiver/Transmitter (UART0) ----------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_uart0_inst_true:
    if IO_UART0_EN generate
      neorv32_uart0_inst: entity neorv32.neorv32_uart
      generic map (
        SIM_LOG_FILE => "neorv32.uart0.sim_mode.text.out",
        UART_RX_FIFO => IO_UART0_RX_FIFO,
        UART_TX_FIFO => IO_UART0_TX_FIFO
      )
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => iodev_req(IODEV_UART0),
        bus_rsp_o   => iodev_rsp(IODEV_UART0),
        clkgen_en_o => cg_en(CG_UART0),
        clkgen_i    => clk_gen,
        uart_txd_o  => uart0_txd_o,
        uart_rxd_i  => uart0_rxd_i,
        uart_rts_o  => uart0_rts_o,
        uart_cts_i  => uart0_cts_i,
        irq_rx_o    => firq(FIRQ_UART0_RX),
        irq_tx_o    => firq(FIRQ_UART0_TX)
      );
    end generate;

    neorv32_uart0_inst_false:
    if not IO_UART0_EN generate
      iodev_rsp(IODEV_UART0) <= rsp_terminate_c;
      uart0_txd_o            <= '0';
      uart0_rts_o            <= '1';
      cg_en(CG_UART0)        <= '0';
      firq(FIRQ_UART0_RX)    <= '0';
      firq(FIRQ_UART0_TX)    <= '0';
    end generate;


    -- Secondary Universal Asynchronous Receiver/Transmitter (UART1) --------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_uart1_inst_true:
    if IO_UART1_EN generate
      neorv32_uart1_inst: entity neorv32.neorv32_uart
      generic map (
        SIM_LOG_FILE => "neorv32.uart1.sim_mode.text.out",
        UART_RX_FIFO => IO_UART1_RX_FIFO,
        UART_TX_FIFO => IO_UART1_TX_FIFO
      )
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => iodev_req(IODEV_UART1),
        bus_rsp_o   => iodev_rsp(IODEV_UART1),
        clkgen_en_o => cg_en(CG_UART1),
        clkgen_i    => clk_gen,
        uart_txd_o  => uart1_txd_o,
        uart_rxd_i  => uart1_rxd_i,
        uart_rts_o  => uart1_rts_o,
        uart_cts_i  => uart1_cts_i,
        irq_rx_o    => firq(FIRQ_UART1_RX),
        irq_tx_o    => firq(FIRQ_UART1_TX)
      );
    end generate;

    neorv32_uart1_inst_false:
    if not IO_UART1_EN generate
      iodev_rsp(IODEV_UART1) <= rsp_terminate_c;
      uart1_txd_o            <= '0';
      uart1_rts_o            <= '1';
      cg_en(CG_UART1)        <= '0';
      firq(FIRQ_UART1_RX)    <= '0';
      firq(FIRQ_UART1_TX)    <= '0';
    end generate;


    -- Serial Peripheral Interface (SPI) ------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_spi_inst_true:
    if IO_SPI_EN generate
      neorv32_spi_inst: entity neorv32.neorv32_spi
      generic map (
        IO_SPI_FIFO => IO_SPI_FIFO
      )
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => iodev_req(IODEV_SPI),
        bus_rsp_o   => iodev_rsp(IODEV_SPI),
        clkgen_en_o => cg_en(CG_SPI),
        clkgen_i    => clk_gen,
        spi_clk_o   => spi_clk_o,
        spi_dat_o   => spi_dat_o,
        spi_dat_i   => spi_dat_i,
        spi_csn_o   => spi_csn_o,
        irq_o       => firq(FIRQ_SPI)
      );
    end generate;

    neorv32_spi_inst_false:
    if not IO_SPI_EN generate
      iodev_rsp(IODEV_SPI) <= rsp_terminate_c;
      spi_clk_o            <= '0';
      spi_dat_o            <= '0';
      spi_csn_o            <= (others => '1');
      cg_en(CG_SPI)        <= '0';
      firq(FIRQ_SPI)       <= '0';
    end generate;


    -- Two-Wire Interface (TWI) ---------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_twi_inst_true:
    if IO_TWI_EN generate
      neorv32_twi_inst: entity neorv32.neorv32_twi
      generic map (
        IO_TWI_FIFO => IO_TWI_FIFO
      )
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => iodev_req(IODEV_TWI),
        bus_rsp_o   => iodev_rsp(IODEV_TWI),
        clkgen_en_o => cg_en(CG_TWI),
        clkgen_i    => clk_gen,
        twi_sda_i   => twi_sda_i,
        twi_sda_o   => twi_sda_o,
        twi_scl_i   => twi_scl_i,
        twi_scl_o   => twi_scl_o,
        irq_o       => firq(FIRQ_TWI)
      );
    end generate;

    neorv32_twi_inst_false:
    if not IO_TWI_EN generate
      iodev_rsp(IODEV_TWI) <= rsp_terminate_c;
      twi_sda_o            <= '1';
      twi_scl_o            <= '1';
      cg_en(CG_TWI)        <= '0';
      firq(FIRQ_TWI)       <= '0';
    end generate;


    -- Pulse-Width Modulation Controller (PWM) ------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_pwm_inst_true:
    if io_pwm_en_c generate
      neorv32_pwm_inst: entity neorv32.neorv32_pwm
      generic map (
        NUM_CHANNELS => IO_PWM_NUM_CH
      )
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => iodev_req(IODEV_PWM),
        bus_rsp_o   => iodev_rsp(IODEV_PWM),
        clkgen_en_o => cg_en(CG_PWM),
        clkgen_i    => clk_gen,
        pwm_o       => pwm_o
      );
    end generate;

    neorv32_pwm_inst_false:
    if not io_pwm_en_c generate
      iodev_rsp(IODEV_PWM) <= rsp_terminate_c;
      cg_en(CG_PWM)        <= '0';
      pwm_o                <= (others => '0');
    end generate;


    -- True Random Number Generator (TRNG) ----------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_trng_inst_true:
    if IO_TRNG_EN generate
      neorv32_trng_inst: entity neorv32.neorv32_trng
      generic map (
        IO_TRNG_FIFO => IO_TRNG_FIFO
      )
      port map (
        clk_i     => clk_i,
        rstn_i    => rstn_sys,
        bus_req_i => iodev_req(IODEV_TRNG),
        bus_rsp_o => iodev_rsp(IODEV_TRNG),
        irq_o     => firq(FIRQ_TRNG)
      );
    end generate;

    neorv32_trng_inst_false:
    if not IO_TRNG_EN generate
      iodev_rsp(IODEV_TRNG) <= rsp_terminate_c;
      firq(FIRQ_TRNG)       <= '0';
    end generate;


    -- Smart LED (WS2811/WS2812) Interface (NEOLED) -------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_neoled_inst_true:
    if IO_NEOLED_EN generate
      neorv32_neoled_inst: entity neorv32.neorv32_neoled
      generic map (
        FIFO_DEPTH => IO_NEOLED_TX_FIFO
      )
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => iodev_req(IODEV_NEOLED),
        bus_rsp_o   => iodev_rsp(IODEV_NEOLED),
        clkgen_en_o => cg_en(CG_NEOLED),
        clkgen_i    => clk_gen,
        irq_o       => firq(FIRQ_NEOLED),
        neoled_o    => neoled_o
      );
    end generate;

    neorv32_neoled_inst_false:
    if not IO_NEOLED_EN generate
      iodev_rsp(IODEV_NEOLED) <= rsp_terminate_c;
      cg_en(CG_NEOLED)        <= '0';
      firq(FIRQ_NEOLED)       <= '0';
      neoled_o                <= '0';
    end generate;


    -- External Interrupt Controller (XIRQ) ---------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_xirq_inst_true:
    if io_xirq_en_c generate
      neorv32_xirq_inst: entity neorv32.neorv32_xirq
      generic map (
        XIRQ_NUM_CH => XIRQ_NUM_CH
      )
      port map (
        clk_i     => clk_i,
        rstn_i    => rstn_sys,
        bus_req_i => iodev_req(IODEV_XIRQ),
        bus_rsp_o => iodev_rsp(IODEV_XIRQ),
        xirq_i    => xirq_i,
        cpu_irq_o => firq(FIRQ_XIRQ)
      );
    end generate;

    neorv32_xirq_inst_false:
    if not io_xirq_en_c generate
      iodev_rsp(IODEV_XIRQ) <= rsp_terminate_c;
      firq(FIRQ_XIRQ)       <= '0';
    end generate;


    -- General Purpose Timer (GPTMR) ----------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_gptmr_inst_true:
    if IO_GPTMR_EN generate
      neorv32_gptmr_inst: entity neorv32.neorv32_gptmr
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => iodev_req(IODEV_GPTMR),
        bus_rsp_o   => iodev_rsp(IODEV_GPTMR),
        clkgen_en_o => cg_en(CG_GPTMR),
        clkgen_i    => clk_gen,
        irq_o       => firq(FIRQ_GPTMR)
      );
    end generate;

    neorv32_gptmr_inst_false:
    if not IO_GPTMR_EN generate
      iodev_rsp(IODEV_GPTMR) <= rsp_terminate_c;
      cg_en(CG_GPTMR)        <= '0';
      firq(FIRQ_GPTMR)       <= '0';
    end generate;


    -- 1-Wire Interface Controller (ONEWIRE) --------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_onewire_inst_true:
    if IO_ONEWIRE_EN generate
      neorv32_onewire_inst: entity neorv32.neorv32_onewire
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => iodev_req(IODEV_ONEWIRE),
        bus_rsp_o   => iodev_rsp(IODEV_ONEWIRE),
        clkgen_en_o => cg_en(CG_ONEWIRE),
        clkgen_i    => clk_gen,
        onewire_i   => onewire_i,
        onewire_o   => onewire_o,
        irq_o       => firq(FIRQ_ONEWIRE)
      );
    end generate;

    neorv32_onewire_inst_false:
    if not IO_ONEWIRE_EN generate
      iodev_rsp(IODEV_ONEWIRE) <= rsp_terminate_c;
      onewire_o                <= '1';
      cg_en(CG_ONEWIRE)        <= '0';
      firq(FIRQ_ONEWIRE)       <= '0';
    end generate;


    -- Stream Link Interface (SLINK) ----------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_slink_inst_true:
    if IO_SLINK_EN generate
      neorv32_slink_inst: entity neorv32.neorv32_slink
      generic map (
        SLINK_RX_FIFO => IO_SLINK_RX_FIFO,
        SLINK_TX_FIFO => IO_SLINK_TX_FIFO
      )
      port map (
        clk_i            => clk_i,
        rstn_i           => rstn_sys,
        bus_req_i        => iodev_req(IODEV_SLINK),
        bus_rsp_o        => iodev_rsp(IODEV_SLINK),
        rx_irq_o         => firq(FIRQ_SLINK_RX),
        tx_irq_o         => firq(FIRQ_SLINK_TX),
        slink_rx_data_i  => slink_rx_dat_i,
        slink_rx_src_i   => slink_rx_src_i,
        slink_rx_valid_i => slink_rx_val_i,
        slink_rx_last_i  => slink_rx_lst_i,
        slink_rx_ready_o => slink_rx_rdy_o,
        slink_tx_data_o  => slink_tx_dat_o,
        slink_tx_dst_o   => slink_tx_dst_o,
        slink_tx_valid_o => slink_tx_val_o,
        slink_tx_last_o  => slink_tx_lst_o,
        slink_tx_ready_i => slink_tx_rdy_i
      );
    end generate;

    neorv32_slink_inst_false:
    if not IO_SLINK_EN generate
      iodev_rsp(IODEV_SLINK) <= rsp_terminate_c;
      firq(FIRQ_SLINK_RX)    <= '0';
      firq(FIRQ_SLINK_TX)    <= '0';
      slink_rx_rdy_o         <= '0';
      slink_tx_dat_o         <= (others => '0');
      slink_tx_dst_o         <= (others => '0');
      slink_tx_val_o         <= '0';
      slink_tx_lst_o         <= '0';
    end generate;


    -- Cyclic Redundancy Check Unit (CRC) -----------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_crc_inst_true:
    if IO_CRC_EN generate
      neorv32_crc_inst: entity neorv32.neorv32_crc
        port map (
        clk_i     => clk_i,
        rstn_i    => rstn_sys,
        bus_req_i => iodev_req(IODEV_CRC),
        bus_rsp_o => iodev_rsp(IODEV_CRC)
      );
    end generate;

    neorv32_crc_inst_false:
    if not IO_CRC_EN generate
      iodev_rsp(IODEV_CRC) <= rsp_terminate_c;
    end generate;


    -- System Configuration Information Memory (SYSINFO) --------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_sysinfo_inst_true:
    if io_sysinfo_en_c generate
      neorv32_sysinfo_inst: entity neorv32.neorv32_sysinfo
      generic map (
        CLOCK_FREQUENCY       => CLOCK_FREQUENCY,
        CLOCK_GATING_EN       => CLOCK_GATING_EN,
        INT_BOOTLOADER_EN     => INT_BOOTLOADER_EN,
        MEM_INT_IMEM_EN       => MEM_INT_IMEM_EN,
        MEM_INT_IMEM_SIZE     => imem_size_c,
        MEM_INT_DMEM_EN       => MEM_INT_DMEM_EN,
        MEM_INT_DMEM_SIZE     => dmem_size_c,
        ICACHE_EN             => ICACHE_EN,
        ICACHE_NUM_BLOCKS     => ICACHE_NUM_BLOCKS,
        ICACHE_BLOCK_SIZE     => ICACHE_BLOCK_SIZE,
        DCACHE_EN             => DCACHE_EN,
        DCACHE_NUM_BLOCKS     => DCACHE_NUM_BLOCKS,
        DCACHE_BLOCK_SIZE     => DCACHE_BLOCK_SIZE,
        XBUS_EN               => XBUS_EN,
        XBUS_CACHE_EN         => XBUS_CACHE_EN,
        XBUS_CACHE_NUM_BLOCKS => XBUS_CACHE_NUM_BLOCKS,
        XBUS_CACHE_BLOCK_SIZE => XBUS_CACHE_BLOCK_SIZE,
        XIP_EN                => XIP_EN,
        XIP_CACHE_EN          => XIP_CACHE_EN,
        XIP_CACHE_NUM_BLOCKS  => XIP_CACHE_NUM_BLOCKS,
        XIP_CACHE_BLOCK_SIZE  => XIP_CACHE_BLOCK_SIZE,
        ON_CHIP_DEBUGGER_EN   => ON_CHIP_DEBUGGER_EN,
        IO_GPIO_EN            => io_gpio_en_c,
        IO_MTIME_EN           => IO_MTIME_EN,
        IO_UART0_EN           => IO_UART0_EN,
        IO_UART1_EN           => IO_UART1_EN,
        IO_SPI_EN             => IO_SPI_EN,
        IO_SDI_EN             => IO_SDI_EN,
        IO_TWI_EN             => IO_TWI_EN,
        IO_PWM_EN             => io_pwm_en_c,
        IO_WDT_EN             => IO_WDT_EN,
        IO_TRNG_EN            => IO_TRNG_EN,
        IO_CFS_EN             => IO_CFS_EN,
        IO_NEOLED_EN          => IO_NEOLED_EN,
        IO_XIRQ_EN            => io_xirq_en_c,
        IO_GPTMR_EN           => IO_GPTMR_EN,
        IO_ONEWIRE_EN         => IO_ONEWIRE_EN,
        IO_DMA_EN             => IO_DMA_EN,
        IO_SLINK_EN           => IO_SLINK_EN,
        IO_CRC_EN             => IO_CRC_EN
      )
      port map (
        clk_i     => clk_i,
        rstn_i    => rstn_sys,
        bus_req_i => iodev_req(IODEV_SYSINFO),
        bus_rsp_o => iodev_rsp(IODEV_SYSINFO)
      );
    end generate;

    neorv32_sysinfo_inst_false:
    if not io_sysinfo_en_c generate
      iodev_rsp(IODEV_SYSINFO) <= rsp_terminate_c;
    end generate;


  end generate; -- /io_system


  -- **************************************************************************************************************************
  -- On-Chip Debugger Complex
  -- **************************************************************************************************************************
  neorv32_neorv32_ocd_inst_true:
  if ON_CHIP_DEBUGGER_EN generate

    -- On-Chip Debugger - Debug Transport Module (DTM) ----------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_debug_dtm_inst: entity neorv32.neorv32_debug_dtm
    generic map (
      IDCODE_VERSION => (others => '0'), -- yet unused
      IDCODE_PARTID  => (others => '0'), -- yet unused
      IDCODE_MANID   => JEDEC_ID
    )
    port map (
      clk_i      => clk_i,
      rstn_i     => rstn_ext,
      jtag_tck_i => jtag_tck_i,
      jtag_tdi_i => jtag_tdi_i,
      jtag_tdo_o => jtag_tdo_o,
      jtag_tms_i => jtag_tms_i,
      dmi_req_o  => dmi_req,
      dmi_rsp_i  => dmi_rsp
    );

    -- On-Chip Debugger - Debug Module (DM) ---------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_debug_dm_inst: entity neorv32.neorv32_debug_dm
    generic map (
      CPU_BASE_ADDR => base_io_dm_c,
      LEGACY_MODE   => DM_LEGACY_MODE
    )
    port map (
      clk_i          => clk_i,
      rstn_i         => rstn_ext,
      cpu_debug_i    => cpu_debug,
      dmi_req_i      => dmi_req,
      dmi_rsp_o      => dmi_rsp,
      bus_req_i      => iodev_req(IODEV_OCD),
      bus_rsp_o      => iodev_rsp(IODEV_OCD),
      cpu_ndmrstn_o  => dci_ndmrstn,
      cpu_halt_req_o => dci_halt_req
    );

  end generate;

  neorv32_debug_ocd_inst_false:
  if not ON_CHIP_DEBUGGER_EN generate
    iodev_rsp(IODEV_OCD) <= rsp_terminate_c;
    jtag_tdo_o           <= jtag_tdi_i; -- JTAG pass-through
    dci_ndmrstn          <= '1';
    dci_halt_req         <= '0';
  end generate;


end neorv32_top_rtl;
