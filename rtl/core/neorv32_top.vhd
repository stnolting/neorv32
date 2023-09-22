-- #################################################################################################
-- # << The NEORV32 RISC-V Processor - Top Entity >>                                               #
-- # ********************************************************************************************* #
-- # Check out the processor's online documentation for more information:                          #
-- #  HQ:         https://github.com/stnolting/neorv32                                             #
-- #  Data Sheet: https://stnolting.github.io/neorv32                                              #
-- #  User Guide: https://stnolting.github.io/neorv32/ug                                           #
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
use neorv32.neorv32_package.all;

entity neorv32_top is
  generic (
    -- General --
    CLOCK_FREQUENCY              : natural;                                       -- clock frequency of clk_i in Hz
    HART_ID                      : std_ulogic_vector(31 downto 0) := x"00000000"; -- hardware thread ID
    VENDOR_ID                    : std_ulogic_vector(31 downto 0) := x"00000000"; -- vendor's JEDEC ID
    INT_BOOTLOADER_EN            : boolean := false;                              -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM

    -- On-Chip Debugger (OCD) --
    ON_CHIP_DEBUGGER_EN          : boolean := false;                              -- implement on-chip debugger
    DM_LEGACY_MODE               : boolean := false;                              -- debug module spec version: false = v1.0, true = v0.13

    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        : boolean := false;                              -- implement atomic memory operations extension?
    CPU_EXTENSION_RISCV_B        : boolean := false;                              -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_C        : boolean := false;                              -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        : boolean := false;                              -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        : boolean := false;                              -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U        : boolean := false;                              -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    : boolean := false;                              -- implement 32-bit floating-point extension (using INT regs!)
    CPU_EXTENSION_RISCV_Zicntr   : boolean := true;                               -- implement base counters?
    CPU_EXTENSION_RISCV_Zihpm    : boolean := false;                              -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zifencei : boolean := false;                              -- implement instruction stream sync.?
    CPU_EXTENSION_RISCV_Zmmul    : boolean := false;                              -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zxcfu    : boolean := false;                              -- implement custom (instr.) functions unit?

    -- Tuning Options --
    FAST_MUL_EN                  : boolean := false;                              -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                : boolean := false;                              -- use barrel shifter for shift operations

    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS              : natural range 0 to 16 := 0;                    -- number of regions (0..16)
    PMP_MIN_GRANULARITY          : natural := 4;                                  -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes

    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 : natural range 0 to 13 := 0;                    -- number of implemented HPM counters (0..13)
    HPM_CNT_WIDTH                : natural range 0 to 64 := 40;                   -- total size of HPM counters (0..64)

    -- Atomic Memory Access - Reservation Set Granularity --
    AMO_RVS_GRANULARITY          : natural := 4;                                  -- size in bytes, has to be a power of 2, min 4

    -- Internal Instruction memory (IMEM) --
    MEM_INT_IMEM_EN              : boolean := false;                              -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            : natural := 16*1024;                            -- size of processor-internal instruction memory in bytes (use a power of 2)

    -- Internal Data memory (DMEM) --
    MEM_INT_DMEM_EN              : boolean := false;                              -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            : natural := 8*1024;                             -- size of processor-internal data memory in bytes (use a power of 2)

    -- Internal Instruction Cache (iCACHE) --
    ICACHE_EN                    : boolean                  := false;             -- implement instruction cache
    ICACHE_NUM_BLOCKS            : natural range 1 to 256   := 4;                 -- i-cache: number of blocks (min 1), has to be a power of 2
    ICACHE_BLOCK_SIZE            : natural range 4 to 2**16 := 64;                -- i-cache: block size in bytes (min 4), has to be a power of 2
    ICACHE_ASSOCIATIVITY         : natural range 1 to 2     := 1;                 -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2

    -- Internal Data Cache (dCACHE) --
    DCACHE_EN                    : boolean                  := false;             -- implement data cache
    DCACHE_NUM_BLOCKS            : natural range 1 to 256   := 4;                 -- d-cache: number of blocks (min 1), has to be a power of 2
    DCACHE_BLOCK_SIZE            : natural range 4 to 2**16 := 64;                -- d-cache: block size in bytes (min 4), has to be a power of 2

    -- External memory interface (WISHBONE) --
    MEM_EXT_EN                   : boolean := false;                              -- implement external memory bus interface?
    MEM_EXT_TIMEOUT              : natural := 255;                                -- cycles after a pending bus access auto-terminates (0 = disabled)
    MEM_EXT_PIPE_MODE            : boolean := false;                              -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
    MEM_EXT_BIG_ENDIAN           : boolean := false;                              -- byte order: true=big-endian, false=little-endian
    MEM_EXT_ASYNC_RX             : boolean := false;                              -- use register buffer for RX data when false
    MEM_EXT_ASYNC_TX             : boolean := false;                              -- use register buffer for TX data when false

    -- External Interrupts Controller (XIRQ) --
    XIRQ_NUM_CH                  : natural range 0 to 32          := 0;           -- number of external IRQ channels (0..32)
    XIRQ_TRIGGER_TYPE            : std_ulogic_vector(31 downto 0) := x"ffffffff"; -- trigger type: 0=level, 1=edge
    XIRQ_TRIGGER_POLARITY        : std_ulogic_vector(31 downto 0) := x"ffffffff"; -- trigger polarity: 0=low-level/falling-edge, 1=high-level/rising-edge

    -- Processor peripherals --
    IO_GPIO_NUM                  : natural range 0 to 64          := 0;           -- number of GPIO input/output pairs (0..64)
    IO_MTIME_EN                  : boolean                        := false;       -- implement machine system timer (MTIME)?
    IO_UART0_EN                  : boolean                        := false;       -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_UART0_RX_FIFO             : natural range 1 to 2**15       := 1;           -- RX fifo depth, has to be a power of two, min 1
    IO_UART0_TX_FIFO             : natural range 1 to 2**15       := 1;           -- TX fifo depth, has to be a power of two, min 1
    IO_UART1_EN                  : boolean                        := false;       -- implement secondary universal asynchronous receiver/transmitter (UART1)?
    IO_UART1_RX_FIFO             : natural range 1 to 2**15       := 1;           -- RX fifo depth, has to be a power of two, min 1
    IO_UART1_TX_FIFO             : natural range 1 to 2**15       := 1;           -- TX fifo depth, has to be a power of two, min 1
    IO_SPI_EN                    : boolean                        := false;       -- implement serial peripheral interface (SPI)?
    IO_SPI_FIFO                  : natural range 1 to 2**15       := 1;           -- RTX fifo depth, has to be a power of two, min 1
    IO_SDI_EN                    : boolean                        := false;       -- implement serial data interface (SDI)?
    IO_SDI_FIFO                  : natural range 1 to 2**15       := 1;           -- RTX fifo depth, has to be zero or a power of two, min 1
    IO_TWI_EN                    : boolean                        := false;       -- implement two-wire interface (TWI)?
    IO_PWM_NUM_CH                : natural range 0 to 12          := 0;           -- number of PWM channels to implement (0..12); 0 = disabled
    IO_WDT_EN                    : boolean                        := false;       -- implement watch dog timer (WDT)?
    IO_TRNG_EN                   : boolean                        := false;       -- implement true random number generator (TRNG)?
    IO_TRNG_FIFO                 : natural range 1 to 2**15       := 1;           -- data fifo depth, has to be a power of two, min 1
    IO_CFS_EN                    : boolean                        := false;       -- implement custom functions subsystem (CFS)?
    IO_CFS_CONFIG                : std_ulogic_vector(31 downto 0) := x"00000000"; -- custom CFS configuration generic
    IO_CFS_IN_SIZE               : natural                        := 32;          -- size of CFS input conduit in bits
    IO_CFS_OUT_SIZE              : natural                        := 32;          -- size of CFS output conduit in bits
    IO_NEOLED_EN                 : boolean                        := false;       -- implement NeoPixel-compatible smart LED interface (NEOLED)?
    IO_NEOLED_TX_FIFO            : natural range 1 to 2**15       := 1;           -- NEOLED FIFO depth, has to be a power of two, min 1
    IO_GPTMR_EN                  : boolean                        := false;       -- implement general purpose timer (GPTMR)?
    IO_XIP_EN                    : boolean                        := false;       -- implement execute in place module (XIP)?
    IO_ONEWIRE_EN                : boolean                        := false;       -- implement 1-wire interface (ONEWIRE)?
    IO_DMA_EN                    : boolean                        := false;       -- implement direct memory access controller (DMA)?
    IO_SLINK_EN                  : boolean                        := false;       -- implement stream link interface (SLINK)?
    IO_SLINK_RX_FIFO             : natural range 1 to 2**15       := 1;           -- RX fifo depth, has to be a power of two, min 1
    IO_SLINK_TX_FIFO             : natural range 1 to 2**15       := 1;           -- TX fifo depth, has to be a power of two, min 1
    IO_CRC_EN                    : boolean                        := false        -- implement cyclic redundancy check unit (CRC)?
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

    -- Wishbone bus interface (available if MEM_EXT_EN = true) --
    wb_tag_o       : out std_ulogic_vector(02 downto 0); -- request tag
    wb_adr_o       : out std_ulogic_vector(31 downto 0); -- address
    wb_dat_i       : in  std_ulogic_vector(31 downto 0) := (others => 'U'); -- read data
    wb_dat_o       : out std_ulogic_vector(31 downto 0); -- write data
    wb_we_o        : out std_ulogic; -- read/write
    wb_sel_o       : out std_ulogic_vector(03 downto 0); -- byte enable
    wb_stb_o       : out std_ulogic; -- strobe
    wb_cyc_o       : out std_ulogic; -- valid cycle
    wb_ack_i       : in  std_ulogic := 'L'; -- transfer acknowledge
    wb_err_i       : in  std_ulogic := 'L'; -- transfer error

    -- Stream Link Interface (available if IO_SLINK_EN = true) --
    slink_rx_dat_i : in  std_ulogic_vector(31 downto 0) := (others => 'U'); -- RX input data
    slink_rx_val_i : in  std_ulogic := 'L'; -- RX valid input
    slink_rx_rdy_o : out std_ulogic; -- RX ready to receive
    slink_tx_dat_o : out std_ulogic_vector(31 downto 0); -- TX output data
    slink_tx_val_o : out std_ulogic; -- TX valid output
    slink_tx_rdy_i : in  std_ulogic := 'L';  -- TX ready to send

    -- Advanced memory control signals --
    fence_o        : out std_ulogic; -- indicates an executed FENCE operation
    fencei_o       : out std_ulogic; -- indicates an executed FENCEI operation

    -- XIP (execute in place via SPI) signals (available if IO_XIP_EN = true) --
    xip_csn_o      : out std_ulogic; -- chip-select, low-active
    xip_clk_o      : out std_ulogic; -- serial clock
    xip_dat_i      : in  std_ulogic := 'L'; -- device data input
    xip_dat_o      : out std_ulogic; -- controller data output

    -- GPIO (available if IO_GPIO_NUM > 0) --
    gpio_o         : out std_ulogic_vector(63 downto 0); -- parallel output
    gpio_i         : in  std_ulogic_vector(63 downto 0) := (others => 'U'); -- parallel input

    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o    : out std_ulogic; -- UART0 send data
    uart0_rxd_i    : in  std_ulogic := 'U'; -- UART0 receive data
    uart0_rts_o    : out std_ulogic; -- HW flow control: UART0.RX ready to receive ("RTR"), low-active, optional
    uart0_cts_i    : in  std_ulogic := 'L'; -- HW flow control: UART0.TX allowed to transmit, low-active, optional

    -- secondary UART1 (available if IO_UART1_EN = true) --
    uart1_txd_o    : out std_ulogic; -- UART1 send data
    uart1_rxd_i    : in  std_ulogic := 'U'; -- UART1 receive data
    uart1_rts_o    : out std_ulogic; -- HW flow control: UART1.RX ready to receive ("RTR"), low-active, optional
    uart1_cts_i    : in  std_ulogic := 'L'; -- HW flow control: UART1.TX allowed to transmit, low-active, optional

    -- SPI (available if IO_SPI_EN = true) --
    spi_clk_o      : out std_ulogic; -- SPI serial clock
    spi_dat_o      : out std_ulogic; -- controller data out, peripheral data in
    spi_dat_i      : in  std_ulogic := 'U'; -- controller data in, peripheral data out
    spi_csn_o      : out std_ulogic_vector(07 downto 0); -- chip-select

    -- SDI (available if IO_SDI_EN = true) --
    sdi_clk_i      : in  std_ulogic := 'U'; -- SDI serial clock
    sdi_dat_o      : out std_ulogic; -- controller data out, peripheral data in
    sdi_dat_i      : in  std_ulogic := 'U'; -- controller data in, peripheral data out
    sdi_csn_i      : in  std_ulogic := 'H'; -- chip-select

    -- TWI (available if IO_TWI_EN = true) --
    twi_sda_i      : in  std_ulogic := 'H'; -- serial data line sense input
    twi_sda_o      : out std_ulogic; -- serial data line output (pull low only)
    twi_scl_i      : in  std_ulogic := 'H'; -- serial clock line sense input
    twi_scl_o      : out std_ulogic; -- serial clock line output (pull low only)

    -- 1-Wire Interface (available if IO_ONEWIRE_EN = true) --
    onewire_i      : in  std_ulogic := 'H'; -- 1-wire bus sense input
    onewire_o      : out std_ulogic; -- 1-wire bus output (pull low only)

    -- PWM (available if IO_PWM_NUM_CH > 0) --
    pwm_o          : out std_ulogic_vector(11 downto 0); -- pwm channels

    -- Custom Functions Subsystem IO (available if IO_CFS_EN = true) --
    cfs_in_i       : in  std_ulogic_vector(IO_CFS_IN_SIZE-1 downto 0) := (others => 'U'); -- custom CFS inputs conduit
    cfs_out_o      : out std_ulogic_vector(IO_CFS_OUT_SIZE-1 downto 0); -- custom CFS outputs conduit

    -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
    neoled_o       : out std_ulogic; -- async serial data line

    -- External platform interrupts (available if XIRQ_NUM_CH > 0) --
    xirq_i         : in  std_ulogic_vector(31 downto 0) := (others => 'L'); -- IRQ channels

    -- CPU interrupts --
    mtime_irq_i    : in  std_ulogic := 'L'; -- machine timer interrupt, available if IO_MTIME_EN = false
    msw_irq_i      : in  std_ulogic := 'L'; -- machine software interrupt
    mext_irq_i     : in  std_ulogic := 'L'  -- machine external interrupt
  );
end neorv32_top;

architecture neorv32_top_rtl of neorv32_top is

  -- auto-configuration --
  constant cpu_boot_addr_c : std_ulogic_vector(31 downto 0) := cond_sel_suv_f(INT_BOOTLOADER_EN, mem_boot_base_c, mem_imem_base_c);
  constant imem_as_rom_c   : boolean := not INT_BOOTLOADER_EN;
  constant io_gpio_en_c    : boolean := boolean(IO_GPIO_NUM > 0);
  constant io_xirq_en_c    : boolean := boolean(XIRQ_NUM_CH > 0);
  constant io_pwm_en_c     : boolean := boolean(IO_PWM_NUM_CH > 0);

  -- make sure physical memory sizes are a power of two --
  constant imem_size_valid_c : boolean := is_power_of_two_f(MEM_INT_IMEM_SIZE);
  constant imem_size_pow2_c  : natural := 2**index_size_f(MEM_INT_IMEM_SIZE);
  constant imem_size_c       : natural := cond_sel_natural_f(imem_size_valid_c, MEM_INT_IMEM_SIZE, imem_size_pow2_c);
  --
  constant dmem_size_valid_c : boolean := is_power_of_two_f(MEM_INT_DMEM_SIZE);
  constant dmem_size_pow2_c  : natural := 2**index_size_f(MEM_INT_DMEM_SIZE);
  constant dmem_size_c       : natural := cond_sel_natural_f(dmem_size_valid_c, MEM_INT_DMEM_SIZE, dmem_size_pow2_c);

  -- reset generator --
  signal rstn_wdt                     : std_ulogic;
  signal rstn_sys_sreg, rstn_ext_sreg : std_ulogic_vector(3 downto 0);
  signal rstn_sys, rstn_ext           : std_ulogic;
  signal rst_cause                    : std_ulogic_vector(1 downto 0);

  -- clock generator --
  signal clk_div, clk_div_ff       : std_ulogic_vector(11 downto 0);
  signal clk_gen                   : std_ulogic_vector(07 downto 0);
  signal clk_gen_en, clk_gen_en_ff : std_ulogic;
  --
  type cg_en_t is record
    wdt, uart0, uart1, spi, twi, pwm, cfs, neoled, gptmr, xip, onewire : std_ulogic;
  end record;
  signal cg_en : cg_en_t;

  -- CPU status --
  signal cpu_debug : std_ulogic; -- cpu is in debug mode
  signal cpu_sleep : std_ulogic; -- cpu is in sleep mode
  signal i_fence   : std_ulogic; -- instruction fence
  signal d_fence   : std_ulogic; -- data fence

  -- debug module interface (DMI) --
  signal dmi_req : dmi_req_t;
  signal dmi_rsp : dmi_rsp_t;

  -- debug core interface (DCI) --
  signal dci_ndmrstn, dci_halt_req : std_ulogic;

  -- bus: core complex --
  signal cpu_i_req,  cpu_d_req  : bus_req_t; -- CPU core
  signal cpu_i_rsp,  cpu_d_rsp  : bus_rsp_t; -- CPU core
  signal icache_req, dcache_req : bus_req_t; -- CPU caches
  signal icache_rsp, dcache_rsp : bus_rsp_t; -- CPU caches
  signal core_req               : bus_req_t; -- core complex (CPU + caches)
  signal core_rsp               : bus_rsp_t; -- core complex (CPU + caches)

  -- bus: core complex + DMA --
  signal main_req, main2_req, dma_req : bus_req_t; -- core complex (CPU + caches + DMA)
  signal main_rsp, main2_rsp, dma_rsp : bus_rsp_t; -- core complex (CPU + caches + DMA)

  -- bus: main sections --
  signal imem_req, dmem_req, xip_req, boot_req, io_req, xbus_req : bus_req_t;
  signal imem_rsp, dmem_rsp, xip_rsp, boot_rsp, io_rsp, xbus_rsp : bus_rsp_t;

  -- bus: IO devices --
  type io_devices_t is (
    IODEV_OCD, IODEV_SYSINFO, IODEV_NEOLED, IODEV_GPIO, IODEV_WDT, IODEV_TRNG, IODEV_TWI,
    IODEV_SPI, IODEV_SDI, IODEV_UART1, IODEV_UART0, IODEV_MTIME, IODEV_XIRQ, IODEV_ONEWIRE,
    IODEV_GPTMR, IODEV_PWM, IODEV_XIP, IODEV_CRC, IODEV_DMA, IODEV_SLINK, IODEV_CFS
  );
  type io_req_bus_t is array (io_devices_t) of bus_req_t;
  type io_rsp_bus_t is array (io_devices_t) of bus_rsp_t;
  signal io_dev_req : io_req_bus_t;
  signal io_dev_rsp : io_rsp_bus_t;

  -- IRQs --
  signal cpu_firq : std_ulogic_vector(15 downto 0);
  type irq_t is record
    wdt, uart0_rx, uart0_tx, uart1_rx, uart1_tx, spi, sdi, twi, cfs, neoled, xirq, gptmr, onewire, dma, trng, slink : std_ulogic;
  end record;
  signal firq      : irq_t;
  signal mtime_irq : std_ulogic;

begin

  -- **************************************************************************************************************************
  -- Sanity Checks
  -- **************************************************************************************************************************
  sanity_checks:
  if (true) generate

    assert false report
      "NEORV32 Processor Configuration: " &
      cond_sel_string_f(MEM_INT_IMEM_EN, "IMEM ", "") &
      cond_sel_string_f(MEM_INT_DMEM_EN, "DMEM ", "") &
      cond_sel_string_f(INT_BOOTLOADER_EN, "BOOTROM ", "") &
      cond_sel_string_f(ICACHE_EN, "I-CACHE ", "") &
      cond_sel_string_f(DCACHE_EN, "D-CACHE ", "") &
      cond_sel_string_f(MEM_EXT_EN, "WISHBONE ", "") &
      cond_sel_string_f(io_gpio_en_c, "GPIO ", "") &
      cond_sel_string_f(IO_MTIME_EN, "MTIME ", "") &
      cond_sel_string_f(IO_UART0_EN, "UART0 ", "") &
      cond_sel_string_f(IO_UART1_EN, "UART1 ", "") &
      cond_sel_string_f(IO_SPI_EN, "SPI ", "") &
      cond_sel_string_f(IO_SDI_EN, "SDI ", "") &
      cond_sel_string_f(IO_TWI_EN, "TWI ", "") &
      cond_sel_string_f(io_pwm_en_c, "PWM ", "") &
      cond_sel_string_f(IO_WDT_EN, "WDT ", "") &
      cond_sel_string_f(IO_TRNG_EN, "TRNG ", "") &
      cond_sel_string_f(IO_CFS_EN, "CFS ", "") &
      cond_sel_string_f(IO_NEOLED_EN, "NEOLED ", "") &
      cond_sel_string_f(io_xirq_en_c, "XIRQ ", "") &
      cond_sel_string_f(IO_GPTMR_EN, "GPTMR ", "") &
      cond_sel_string_f(IO_XIP_EN, "XIP ", "") &
      cond_sel_string_f(IO_ONEWIRE_EN, "ONEWIRE ", "") &
      cond_sel_string_f(IO_DMA_EN, "DMA ", "") &
      cond_sel_string_f(IO_SLINK_EN, "SLINK ", "") &
      cond_sel_string_f(IO_CRC_EN, "CRC ", "") &
      cond_sel_string_f(true, "SYSINFO ", "") &
      cond_sel_string_f(ON_CHIP_DEBUGGER_EN, "OCD ", "") &
      ""
      severity note;

    -- internal memory sizes --
    assert not ((imem_size_valid_c = false) and (MEM_INT_IMEM_EN = true)) report
      "NEORV32 PROCESSOR CONFIG WARNING: Configured internal IMEM size (" & natural'image(MEM_INT_IMEM_SIZE) & " bytes) is not a power of two. " &
      "Auto-adjusting memory size to the next power of two (" & natural'image(imem_size_c) & " bytes)" severity warning;
    assert not ((dmem_size_valid_c = false) and (MEM_INT_DMEM_EN = true)) report
      "NEORV32 PROCESSOR CONFIG WARNING: Configured internal DMEM size (" & natural'image(MEM_INT_DMEM_SIZE) & " bytes) is not a power of two. " &
      "Auto-adjusting memory size to the next power of two (" & natural'image(dmem_size_c) & " bytes)" severity warning;

    -- caches --
    assert not ((ICACHE_EN = true) and (CPU_EXTENSION_RISCV_Zifencei = false)) report
      "NEORV32 CPU CONFIG ERROR: <CPU_EXTENSION_RISCV_Zifencei> ISA extension is required to perform i-cache memory sync. operations." severity error;

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
      elsif falling_edge(clk_i) then -- inverted clock to release reset _before_ all FFs trigger (rising edge)
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
      elsif falling_edge(clk_i) then
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
    clk_gen(clk_div2_c)    <= clk_div(0)  and (not clk_div_ff(0));  -- CLK/2
    clk_gen(clk_div4_c)    <= clk_div(1)  and (not clk_div_ff(1));  -- CLK/4
    clk_gen(clk_div8_c)    <= clk_div(2)  and (not clk_div_ff(2));  -- CLK/8
    clk_gen(clk_div64_c)   <= clk_div(5)  and (not clk_div_ff(5));  -- CLK/64
    clk_gen(clk_div128_c)  <= clk_div(6)  and (not clk_div_ff(6));  -- CLK/128
    clk_gen(clk_div1024_c) <= clk_div(9)  and (not clk_div_ff(9));  -- CLK/1024
    clk_gen(clk_div2048_c) <= clk_div(10) and (not clk_div_ff(10)); -- CLK/2048
    clk_gen(clk_div4096_c) <= clk_div(11) and (not clk_div_ff(11)); -- CLK/4096

    -- fresh clocks anyone? --
    clk_gen_en <= cg_en.wdt or cg_en.uart0  or cg_en.uart1 or cg_en.spi or cg_en.twi or cg_en.pwm or
                  cg_en.cfs or cg_en.neoled or cg_en.gptmr or cg_en.xip or cg_en.onewire;

  end generate; -- /generators


  -- **************************************************************************************************************************
  -- Core Complex
  -- **************************************************************************************************************************
  core_complex:
  if (true) generate

    -- CPU Core -------------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_cpu_inst: entity neorv32.neorv32_cpu
    generic map (
      -- General --
      HART_ID                      => HART_ID,
      VENDOR_ID                    => VENDOR_ID,
      CPU_BOOT_ADDR                => cpu_boot_addr_c,
      CPU_DEBUG_PARK_ADDR          => dm_park_entry_c,
      CPU_DEBUG_EXC_ADDR           => dm_exc_entry_c,
      -- RISC-V CPU Extensions --
      CPU_EXTENSION_RISCV_A        => CPU_EXTENSION_RISCV_A,
      CPU_EXTENSION_RISCV_B        => CPU_EXTENSION_RISCV_B,
      CPU_EXTENSION_RISCV_C        => CPU_EXTENSION_RISCV_C,
      CPU_EXTENSION_RISCV_E        => CPU_EXTENSION_RISCV_E,
      CPU_EXTENSION_RISCV_M        => CPU_EXTENSION_RISCV_M,
      CPU_EXTENSION_RISCV_U        => CPU_EXTENSION_RISCV_U,
      CPU_EXTENSION_RISCV_Zfinx    => CPU_EXTENSION_RISCV_Zfinx,
      CPU_EXTENSION_RISCV_Zicntr   => CPU_EXTENSION_RISCV_Zicntr,
      CPU_EXTENSION_RISCV_Zihpm    => CPU_EXTENSION_RISCV_Zihpm,
      CPU_EXTENSION_RISCV_Zifencei => CPU_EXTENSION_RISCV_Zifencei,
      CPU_EXTENSION_RISCV_Zmmul    => CPU_EXTENSION_RISCV_Zmmul,
      CPU_EXTENSION_RISCV_Zxcfu    => CPU_EXTENSION_RISCV_Zxcfu,
      CPU_EXTENSION_RISCV_Sdext    => ON_CHIP_DEBUGGER_EN,
      CPU_EXTENSION_RISCV_Sdtrig   => ON_CHIP_DEBUGGER_EN,
      -- Tuning Options --
      FAST_MUL_EN                  => FAST_MUL_EN,
      FAST_SHIFT_EN                => FAST_SHIFT_EN,
      -- Physical Memory Protection (PMP) --
      PMP_NUM_REGIONS              => PMP_NUM_REGIONS,
      PMP_MIN_GRANULARITY          => PMP_MIN_GRANULARITY,
      -- Hardware Performance Monitors (HPM) --
      HPM_NUM_CNTS                 => HPM_NUM_CNTS,
      HPM_CNT_WIDTH                => HPM_CNT_WIDTH
    )
    port map (
      -- global control --
      clk_i      => clk_i,
      rstn_i     => rstn_sys,
      sleep_o    => cpu_sleep,
      debug_o    => cpu_debug,
      ifence_o   => i_fence,
      dfence_o   => d_fence,
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

    -- advanced memory control --
    fence_o  <= d_fence;
    fencei_o <= i_fence;

    -- fast interrupt requests (FIRQs) --
    cpu_firq(00) <= firq.wdt; -- highest priority
    cpu_firq(01) <= firq.cfs;
    cpu_firq(02) <= firq.uart0_rx;
    cpu_firq(03) <= firq.uart0_tx;
    cpu_firq(04) <= firq.uart1_rx;
    cpu_firq(05) <= firq.uart1_tx;
    cpu_firq(06) <= firq.spi;
    cpu_firq(07) <= firq.twi;
    cpu_firq(08) <= firq.xirq;
    cpu_firq(09) <= firq.neoled;
    cpu_firq(10) <= firq.dma;
    cpu_firq(11) <= firq.sdi;
    cpu_firq(12) <= firq.gptmr;
    cpu_firq(13) <= firq.onewire;
    cpu_firq(14) <= firq.slink;
    cpu_firq(15) <= firq.trng; -- lowest priority


    -- CPU Instruction Cache ------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_icache_inst_true:
    if (ICACHE_EN = true) generate
      neorv32_icache_inst: entity neorv32.neorv32_icache
      generic map (
        ICACHE_NUM_BLOCKS => ICACHE_NUM_BLOCKS,
        ICACHE_BLOCK_SIZE => ICACHE_BLOCK_SIZE,
        ICACHE_NUM_SETS   => ICACHE_ASSOCIATIVITY,
        ICACHE_UC_PBEGIN  => uncached_begin_c(31 downto 28)
      )
      port map (
        clk_i     => clk_i,
        rstn_i    => rstn_sys,
        clear_i   => i_fence,
        cpu_req_i => cpu_i_req,
        cpu_rsp_o => cpu_i_rsp,
        bus_req_o => icache_req,
        bus_rsp_i => icache_rsp
      );
    end generate;

    neorv32_icache_inst_false:
    if (ICACHE_EN = false) generate
      icache_req <= cpu_i_req;
      cpu_i_rsp  <= icache_rsp;
    end generate;


    -- CPU Data Cache -------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_dcache_inst_true:
    if (DCACHE_EN = true) generate
      neorv32_dcache_inst: entity neorv32.neorv32_dcache
      generic map (
        DCACHE_NUM_BLOCKS => DCACHE_NUM_BLOCKS,
        DCACHE_BLOCK_SIZE => DCACHE_BLOCK_SIZE,
        DCACHE_UC_PBEGIN  => uncached_begin_c(31 downto 28)
      )
      port map (
        clk_i     => clk_i,
        rstn_i    => rstn_sys,
        clear_i   => d_fence,
        cpu_req_i => cpu_d_req,
        cpu_rsp_o => cpu_d_rsp,
        bus_req_o => dcache_req,
        bus_rsp_i => dcache_rsp
      );
    end generate;

    neorv32_dcache_inst_false:
    if (DCACHE_EN = false) generate
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
      clk_i   => clk_i,
      rstn_i  => rstn_sys,
      a_req_i => dcache_req, -- prioritized
      a_rsp_o => dcache_rsp,
      b_req_i => icache_req,
      b_rsp_o => icache_rsp,
      x_req_o => core_req,
      x_rsp_i => core_rsp
    );

  end generate; -- /core_complex


  -- **************************************************************************************************************************
  -- Direct Memory Access Controller (DMA) Complex
  -- **************************************************************************************************************************
  neorv32_dma_complex_true:
  if (IO_DMA_EN = true) generate

    -- DMA Controller -------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_dma_inst: entity neorv32.neorv32_dma
    port map (
      clk_i     => clk_i,
      rstn_i    => rstn_sys,
      bus_req_i => io_dev_req(IODEV_DMA),
      bus_rsp_o => io_dev_rsp(IODEV_DMA),
      dma_req_o => dma_req,
      dma_rsp_i => dma_rsp,
      firq_i    => cpu_firq,
      irq_o     => firq.dma
    );


    -- DMA Bus Switch -------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_dma_bus_switch_inst: entity neorv32.neorv32_bus_switch
    generic map (
      PORT_A_READ_ONLY => false,
      PORT_B_READ_ONLY => false
    )
    port map (
      clk_i   => clk_i,
      rstn_i  => rstn_sys,
      a_req_i => core_req, -- prioritized
      a_rsp_o => core_rsp,
      b_req_i => dma_req,
      b_rsp_o => dma_rsp,
      x_req_o => main_req,
      x_rsp_i => main_rsp
    );

  end generate; -- /neorv32_dma_complex_true

  neorv32_dma_complex_false:
  if (IO_DMA_EN = false) generate
    io_dev_rsp(IODEV_DMA) <= rsp_terminate_c;
    main_req              <= core_req;
    core_rsp              <= main_rsp;
    firq.dma              <= '0';
  end generate;


  -- **************************************************************************************************************************
  -- Reservation Set Controller (for atomic LR/SC memory accesses)
  -- **************************************************************************************************************************
  neorv32_bus_reservation_set_true:
  if (CPU_EXTENSION_RISCV_A = true) generate
    neorv32_bus_reservation_set_inst: entity neorv32.neorv32_bus_reservation_set
    generic map (
      GRANULARITY => AMO_RVS_GRANULARITY
    )
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
  if (CPU_EXTENSION_RISCV_A = false) generate
    main2_req <= main_req;
    main_rsp  <= main2_rsp;
  end generate;


  -- **************************************************************************************************************************
  -- Address Region Gateway
  -- **************************************************************************************************************************
  neorv32_bus_gateway_inst: entity neorv32.neorv32_bus_gateway
  generic map (
    TIMEOUT     => bus_timeout_c,
    -- IMEM port --
    IMEM_ENABLE => MEM_INT_IMEM_EN,
    IMEM_BASE   => mem_imem_base_c,
    IMEM_SIZE   => imem_size_c,
    -- DMEM port --
    DMEM_ENABLE => MEM_INT_DMEM_EN,
    DMEM_BASE   => mem_dmem_base_c,
    DMEM_SIZE   => dmem_size_c,
    -- XIP port --
    XIP_ENABLE  => IO_XIP_EN,
    XIP_BASE    => mem_xip_base_c,
    XIP_SIZE    => mem_xip_size_c,
    -- BOOT ROM port --
    BOOT_ENABLE => INT_BOOTLOADER_EN,
    BOOT_BASE   => mem_boot_base_c,
    BOOT_SIZE   => mem_boot_size_c,
    -- IO port --
    IO_ENABLE   => true, -- always enabled (mandatory core module)
    IO_BASE     => mem_io_base_c,
    IO_SIZE     => mem_io_size_c,
    -- EXT port --
    EXT_ENABLE  => MEM_EXT_EN
  )
  port map (
    -- global control --
    clk_i      => clk_i,
    rstn_i     => rstn_sys,
    -- host port --
    main_req_i => main2_req,
    main_rsp_o => main2_rsp,
    -- section ports --
    imem_req_o => imem_req,
    imem_rsp_i => imem_rsp,
    dmem_req_o => dmem_req,
    dmem_rsp_i => dmem_rsp,
    xip_req_o  => xip_req,
    xip_rsp_i  => xip_rsp,
    boot_req_o => boot_req,
    boot_rsp_i => boot_rsp,
    io_req_o   => io_req,
    io_rsp_i   => io_rsp,
    ext_req_o  => xbus_req,
    ext_rsp_i  => xbus_rsp
  );


  -- **************************************************************************************************************************
  -- Memory System
  -- **************************************************************************************************************************
  memory_system:
  if (true) generate

    -- Processor-Internal Instruction Memory (IMEM) -------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_int_imem_inst_true:
    if (MEM_INT_IMEM_EN = true) and (imem_size_c > 0) generate
      neorv32_int_imem_inst: entity neorv32.neorv32_imem
      generic map (
        IMEM_SIZE    => imem_size_c,
        IMEM_AS_IROM => imem_as_rom_c
      )
      port map (
        clk_i     => clk_i,
        bus_req_i => imem_req,
        bus_rsp_o => imem_rsp
      );
    end generate;

    neorv32_int_imem_inst_false:
    if (MEM_INT_IMEM_EN = false) or (imem_size_c = 0) generate
      imem_rsp <= rsp_terminate_c;
    end generate;


    -- Processor-Internal Data Memory (DMEM) --------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_int_dmem_inst_true:
    if (MEM_INT_DMEM_EN = true) and (dmem_size_c > 0) generate
      neorv32_int_dmem_inst: entity neorv32.neorv32_dmem
      generic map (
        DMEM_SIZE => dmem_size_c
      )
      port map (
        clk_i     => clk_i,
        bus_req_i => dmem_req,
        bus_rsp_o => dmem_rsp
      );
    end generate;

    neorv32_int_dmem_inst_false:
    if (MEM_INT_DMEM_EN = false) or (dmem_size_c = 0) generate
      dmem_rsp <= rsp_terminate_c;
    end generate;


    -- Processor-Internal Bootloader ROM (BOOTROM) --------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_boot_rom_inst_true:
    if (INT_BOOTLOADER_EN = true) generate
      neorv32_boot_rom_inst: entity neorv32.neorv32_boot_rom
      port map (
        clk_i     => clk_i,
        bus_req_i => boot_req,
        bus_rsp_o => boot_rsp
      );
    end generate;

    neorv32_boot_rom_inst_false:
    if (INT_BOOTLOADER_EN = false) generate
      boot_rsp <= rsp_terminate_c;
    end generate;


    -- Execute In Place Module (XIP) ----------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_xip_inst_true:
    if (IO_XIP_EN = true) generate
      neorv32_xip_inst: entity neorv32.neorv32_xip
      port map (
        -- global control --
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => io_dev_req(IODEV_XIP),
        bus_rsp_o   => io_dev_rsp(IODEV_XIP),
        xip_req_i   => xip_req,
        xip_rsp_o   => xip_rsp,
        clkgen_en_o => cg_en.xip,
        clkgen_i    => clk_gen,
        spi_csn_o   => xip_csn_o,
        spi_clk_o   => xip_clk_o,
        spi_dat_i   => xip_dat_i,
        spi_dat_o   => xip_dat_o
      );
    end generate;

    neorv32_xip_inst_false:
    if (IO_XIP_EN = false) generate
      io_dev_rsp(IODEV_XIP) <= rsp_terminate_c;
      xip_rsp               <= rsp_terminate_c;
      cg_en.xip             <= '0';
      xip_csn_o             <= '1';
      xip_clk_o             <= '0';
      xip_dat_o             <= '0';
    end generate;


    -- External Wishbone Gateway (WISHBONE) ---------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_wishbone_inst_true:
    if (MEM_EXT_EN = true) generate
      neorv32_wishbone_inst: entity neorv32.neorv32_wishbone
      generic map (
        BUS_TIMEOUT => MEM_EXT_TIMEOUT,
        PIPE_MODE   => MEM_EXT_PIPE_MODE,
        BIG_ENDIAN  => MEM_EXT_BIG_ENDIAN,
        ASYNC_RX    => MEM_EXT_ASYNC_RX,
        ASYNC_TX    => MEM_EXT_ASYNC_TX
      )
      port map (
        clk_i     => clk_i,
        rstn_i    => rstn_sys,
        bus_req_i => xbus_req,
        bus_rsp_o => xbus_rsp,
        --
        wb_tag_o  => wb_tag_o,
        wb_adr_o  => wb_adr_o,
        wb_dat_i  => wb_dat_i,
        wb_dat_o  => wb_dat_o,
        wb_we_o   => wb_we_o,
        wb_sel_o  => wb_sel_o,
        wb_stb_o  => wb_stb_o,
        wb_cyc_o  => wb_cyc_o,
        wb_ack_i  => wb_ack_i,
        wb_err_i  => wb_err_i
      );
    end generate;

    neorv32_wishbone_inst_false:
    if (MEM_EXT_EN = false) generate
      xbus_rsp <= rsp_terminate_c;
      wb_adr_o <= (others => '0');
      wb_dat_o <= (others => '0');
      wb_we_o  <= '0';
      wb_sel_o <= (others => '0');
      wb_stb_o <= '0';
      wb_cyc_o <= '0';
      wb_tag_o <= (others => '0');
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
      -- device port enable and base address --
      DEV_00_EN => ON_CHIP_DEBUGGER_EN, DEV_00_BASE => base_io_dm_c,
      DEV_01_EN => true,                DEV_01_BASE => base_io_sysinfo_c, -- always enabled (mandatory core module)
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
      DEV_16_EN => IO_XIP_EN,           DEV_16_BASE => base_io_xip_c,
      DEV_17_EN => IO_CRC_EN,           DEV_17_BASE => base_io_crc_c,
      DEV_18_EN => IO_DMA_EN,           DEV_18_BASE => base_io_dma_c,
      DEV_19_EN => IO_SLINK_EN,         DEV_19_BASE => base_io_slink_c,
      DEV_20_EN => IO_CFS_EN,           DEV_20_BASE => base_io_cfs_c
    )
    port map (
      -- host port --
      main_req_i   => io_req,
      main_rsp_o   => io_rsp,
      -- device ports --
      dev_00_req_o => io_dev_req(IODEV_OCD),     dev_00_rsp_i => io_dev_rsp(IODEV_OCD),
      dev_01_req_o => io_dev_req(IODEV_SYSINFO), dev_01_rsp_i => io_dev_rsp(IODEV_SYSINFO),
      dev_02_req_o => io_dev_req(IODEV_NEOLED),  dev_02_rsp_i => io_dev_rsp(IODEV_NEOLED),
      dev_03_req_o => io_dev_req(IODEV_GPIO),    dev_03_rsp_i => io_dev_rsp(IODEV_GPIO),
      dev_04_req_o => io_dev_req(IODEV_WDT),     dev_04_rsp_i => io_dev_rsp(IODEV_WDT),
      dev_05_req_o => io_dev_req(IODEV_TRNG),    dev_05_rsp_i => io_dev_rsp(IODEV_TRNG),
      dev_06_req_o => io_dev_req(IODEV_TWI),     dev_06_rsp_i => io_dev_rsp(IODEV_TWI),
      dev_07_req_o => io_dev_req(IODEV_SPI),     dev_07_rsp_i => io_dev_rsp(IODEV_SPI),
      dev_08_req_o => io_dev_req(IODEV_SDI),     dev_08_rsp_i => io_dev_rsp(IODEV_SDI),
      dev_09_req_o => io_dev_req(IODEV_UART1),   dev_09_rsp_i => io_dev_rsp(IODEV_UART1),
      dev_10_req_o => io_dev_req(IODEV_UART0),   dev_10_rsp_i => io_dev_rsp(IODEV_UART0),
      dev_11_req_o => io_dev_req(IODEV_MTIME),   dev_11_rsp_i => io_dev_rsp(IODEV_MTIME),
      dev_12_req_o => io_dev_req(IODEV_XIRQ),    dev_12_rsp_i => io_dev_rsp(IODEV_XIRQ),
      dev_13_req_o => io_dev_req(IODEV_ONEWIRE), dev_13_rsp_i => io_dev_rsp(IODEV_ONEWIRE),
      dev_14_req_o => io_dev_req(IODEV_GPTMR),   dev_14_rsp_i => io_dev_rsp(IODEV_GPTMR),
      dev_15_req_o => io_dev_req(IODEV_PWM),     dev_15_rsp_i => io_dev_rsp(IODEV_PWM),
      dev_16_req_o => io_dev_req(IODEV_XIP),     dev_16_rsp_i => io_dev_rsp(IODEV_XIP),
      dev_17_req_o => io_dev_req(IODEV_CRC),     dev_17_rsp_i => io_dev_rsp(IODEV_CRC),
      dev_18_req_o => io_dev_req(IODEV_DMA),     dev_18_rsp_i => io_dev_rsp(IODEV_DMA),
      dev_19_req_o => io_dev_req(IODEV_SLINK),   dev_19_rsp_i => io_dev_rsp(IODEV_SLINK),
      dev_20_req_o => io_dev_req(IODEV_CFS),     dev_20_rsp_i => io_dev_rsp(IODEV_CFS)
    );


    -- Custom Functions Subsystem (CFS) -------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_cfs_inst_true:
    if (IO_CFS_EN = true) generate
      neorv32_cfs_inst: entity neorv32.neorv32_cfs
      generic map (
        CFS_CONFIG   => IO_CFS_CONFIG,
        CFS_IN_SIZE  => IO_CFS_IN_SIZE,
        CFS_OUT_SIZE => IO_CFS_OUT_SIZE
      )
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => io_dev_req(IODEV_CFS),
        bus_rsp_o   => io_dev_rsp(IODEV_CFS),
        clkgen_en_o => cg_en.cfs,
        clkgen_i    => clk_gen,
        irq_o       => firq.cfs,
        cfs_in_i    => cfs_in_i,
        cfs_out_o   => cfs_out_o
      );
    end generate;

    neorv32_cfs_inst_false:
    if (IO_CFS_EN = false) generate
      io_dev_rsp(IODEV_CFS) <= rsp_terminate_c;
      cg_en.cfs             <= '0';
      firq.cfs              <= '0';
      cfs_out_o             <= (others => '0');
    end generate;


    -- Serial Data Interface (SDI) ------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_sdi_inst_true:
    if (IO_SDI_EN = true) generate
      neorv32_SDI_inst: entity neorv32.neorv32_sdi
      generic map (
        RTX_FIFO => IO_SDI_FIFO
      )
      port map (
        clk_i     => clk_i,
        rstn_i    => rstn_sys,
        bus_req_i => io_dev_req(IODEV_SDI),
        bus_rsp_o => io_dev_rsp(IODEV_SDI),
        sdi_csn_i => sdi_csn_i,
        sdi_clk_i => sdi_clk_i,
        sdi_dat_i => sdi_dat_i,
        sdi_dat_o => sdi_dat_o,
        irq_o     => firq.sdi
      );
    end generate;

    neorv32_sdi_inst_false:
    if (IO_SDI_EN = false) generate
      io_dev_rsp(IODEV_SDI) <= rsp_terminate_c;
      sdi_dat_o             <= '0';
      firq.sdi              <= '0';
    end generate;


    -- General Purpose Input/Output Port (GPIO) -----------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_gpio_inst_true:
    if (io_gpio_en_c = true) generate
      neorv32_gpio_inst: entity neorv32.neorv32_gpio
      generic map (
        GPIO_NUM => IO_GPIO_NUM
      )
      port map (
        clk_i     => clk_i,
        rstn_i    => rstn_sys,
        bus_req_i => io_dev_req(IODEV_GPIO),
        bus_rsp_o => io_dev_rsp(IODEV_GPIO),
        gpio_o    => gpio_o,
        gpio_i    => gpio_i
      );
    end generate;

    neorv32_gpio_inst_false:
    if (io_gpio_en_c = false) generate
      io_dev_rsp(IODEV_GPIO) <= rsp_terminate_c;
      gpio_o                 <= (others => '0');
    end generate;


    -- Watch Dog Timer (WDT) ------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_wdt_inst_true:
    if (IO_WDT_EN = true) generate
      neorv32_wdt_inst: entity neorv32.neorv32_wdt
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        rst_cause_i => rst_cause,
        bus_req_i   => io_dev_req(IODEV_WDT),
        bus_rsp_o   => io_dev_rsp(IODEV_WDT),
        cpu_debug_i => cpu_debug,
        cpu_sleep_i => cpu_sleep,
        clkgen_en_o => cg_en.wdt,
        clkgen_i    => clk_gen,
        irq_o       => firq.wdt,
        rstn_o      => rstn_wdt
      );
    end generate;

    neorv32_wdt_inst_false:
    if (IO_WDT_EN = false) generate
      io_dev_rsp(IODEV_WDT) <= rsp_terminate_c;
      firq.wdt              <= '0';
      rstn_wdt              <= '1';
      cg_en.wdt             <= '0';
    end generate;


    -- Machine System Timer (MTIME) -----------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_mtime_inst_true:
    if (IO_MTIME_EN = true) generate
      neorv32_mtime_inst: entity neorv32.neorv32_mtime
      port map (
        clk_i     => clk_i,
        rstn_i    => rstn_sys,
        bus_req_i => io_dev_req(IODEV_MTIME),
        bus_rsp_o => io_dev_rsp(IODEV_MTIME),
        irq_o     => mtime_irq
      );
    end generate;

    neorv32_mtime_inst_false:
    if (IO_MTIME_EN = false) generate
      io_dev_rsp(IODEV_MTIME) <= rsp_terminate_c;
      mtime_irq               <= mtime_irq_i;
    end generate;


    -- Primary Universal Asynchronous Receiver/Transmitter (UART0) ----------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_uart0_inst_true:
    if (IO_UART0_EN = true) generate
      neorv32_uart0_inst: entity neorv32.neorv32_uart
      generic map (
        SIM_LOG_FILE => "neorv32.uart0.sim_mode.text.out",
        UART_RX_FIFO => IO_UART0_RX_FIFO,
        UART_TX_FIFO => IO_UART0_TX_FIFO
      )
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => io_dev_req(IODEV_UART0),
        bus_rsp_o   => io_dev_rsp(IODEV_UART0),
        clkgen_en_o => cg_en.uart0,
        clkgen_i    => clk_gen,
        uart_txd_o  => uart0_txd_o,
        uart_rxd_i  => uart0_rxd_i,
        uart_rts_o  => uart0_rts_o,
        uart_cts_i  => uart0_cts_i,
        irq_rx_o    => firq.uart0_rx,
        irq_tx_o    => firq.uart0_tx
      );
    end generate;

    neorv32_uart0_inst_false:
    if (IO_UART0_EN = false) generate
      io_dev_rsp(IODEV_UART0) <= rsp_terminate_c;
      uart0_txd_o             <= '0';
      uart0_rts_o             <= '1';
      cg_en.uart0             <= '0';
      firq.uart0_rx           <= '0';
      firq.uart0_tx           <= '0';
    end generate;


    -- Secondary Universal Asynchronous Receiver/Transmitter (UART1) --------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_uart1_inst_true:
    if (IO_UART1_EN = true) generate
      neorv32_uart1_inst: entity neorv32.neorv32_uart
      generic map (
        SIM_LOG_FILE => "neorv32.uart1.sim_mode.text.out",
        UART_RX_FIFO => IO_UART1_RX_FIFO,
        UART_TX_FIFO => IO_UART1_TX_FIFO
      )
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => io_dev_req(IODEV_UART1),
        bus_rsp_o   => io_dev_rsp(IODEV_UART1),
        clkgen_en_o => cg_en.uart1,
        clkgen_i    => clk_gen,
        uart_txd_o  => uart1_txd_o,
        uart_rxd_i  => uart1_rxd_i,
        uart_rts_o  => uart1_rts_o,
        uart_cts_i  => uart1_cts_i,
        irq_rx_o    => firq.uart1_rx,
        irq_tx_o    => firq.uart1_tx
      );
    end generate;

    neorv32_uart1_inst_false:
    if (IO_UART1_EN = false) generate
      io_dev_rsp(IODEV_UART1) <= rsp_terminate_c;
      uart1_txd_o             <= '0';
      uart1_rts_o             <= '1';
      cg_en.uart1             <= '0';
      firq.uart1_rx           <= '0';
      firq.uart1_tx           <= '0';
    end generate;


    -- Serial Peripheral Interface (SPI) ------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_spi_inst_true:
    if (IO_SPI_EN = true) generate
      neorv32_spi_inst: entity neorv32.neorv32_spi
      generic map (
        IO_SPI_FIFO => IO_SPI_FIFO
      )
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => io_dev_req(IODEV_SPI),
        bus_rsp_o   => io_dev_rsp(IODEV_SPI),
        clkgen_en_o => cg_en.spi,
        clkgen_i    => clk_gen,
        spi_clk_o   => spi_clk_o,
        spi_dat_o   => spi_dat_o,
        spi_dat_i   => spi_dat_i,
        spi_csn_o   => spi_csn_o,
        irq_o       => firq.spi
      );
    end generate;

    neorv32_spi_inst_false:
    if (IO_SPI_EN = false) generate
      io_dev_rsp(IODEV_SPI) <= rsp_terminate_c;
      spi_clk_o             <= '0';
      spi_dat_o             <= '0';
      spi_csn_o             <= (others => '1');
      cg_en.spi             <= '0';
      firq.spi              <= '0';
    end generate;


    -- Two-Wire Interface (TWI) ---------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_twi_inst_true:
    if (IO_TWI_EN = true) generate
      neorv32_twi_inst: entity neorv32.neorv32_twi
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => io_dev_req(IODEV_TWI),
        bus_rsp_o   => io_dev_rsp(IODEV_TWI),
        clkgen_en_o => cg_en.twi,
        clkgen_i    => clk_gen,
        twi_sda_i   => twi_sda_i,
        twi_sda_o   => twi_sda_o,
        twi_scl_i   => twi_scl_i,
        twi_scl_o   => twi_scl_o,
        irq_o       => firq.twi
      );
    end generate;

    neorv32_twi_inst_false:
    if (IO_TWI_EN = false) generate
      io_dev_rsp(IODEV_TWI) <= rsp_terminate_c;
      twi_sda_o             <= '1';
      twi_scl_o             <= '1';
      cg_en.twi             <= '0';
      firq.twi              <= '0';
    end generate;


    -- Pulse-Width Modulation Controller (PWM) ------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_pwm_inst_true:
    if (io_pwm_en_c = true) generate
      neorv32_pwm_inst: entity neorv32.neorv32_pwm
      generic map (
        NUM_CHANNELS => IO_PWM_NUM_CH
      )
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => io_dev_req(IODEV_PWM),
        bus_rsp_o   => io_dev_rsp(IODEV_PWM),
        clkgen_en_o => cg_en.pwm,
        clkgen_i    => clk_gen,
        pwm_o       => pwm_o
      );
    end generate;

    neorv32_pwm_inst_false:
    if (io_pwm_en_c = false) generate
      io_dev_rsp(IODEV_PWM) <= rsp_terminate_c;
      cg_en.pwm             <= '0';
      pwm_o                 <= (others => '0');
    end generate;


    -- True Random Number Generator (TRNG) ----------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_trng_inst_true:
    if (IO_TRNG_EN = true) generate
      neorv32_trng_inst: entity neorv32.neorv32_trng
      generic map (
        IO_TRNG_FIFO => IO_TRNG_FIFO
      )
      port map (
        clk_i     => clk_i,
        rstn_i    => rstn_sys,
        bus_req_i => io_dev_req(IODEV_TRNG),
        bus_rsp_o => io_dev_rsp(IODEV_TRNG),
        irq_o     => firq.trng
      );
    end generate;

    neorv32_trng_inst_false:
    if (IO_TRNG_EN = false) generate
      io_dev_rsp(IODEV_TRNG) <= rsp_terminate_c;
      firq.trng              <= '0';
    end generate;


    -- Smart LED (WS2811/WS2812) Interface (NEOLED) -------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_neoled_inst_true:
    if (IO_NEOLED_EN = true) generate
      neorv32_neoled_inst: entity neorv32.neorv32_neoled
      generic map (
        FIFO_DEPTH => IO_NEOLED_TX_FIFO
      )
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => io_dev_req(IODEV_NEOLED),
        bus_rsp_o   => io_dev_rsp(IODEV_NEOLED),
        clkgen_en_o => cg_en.neoled,
        clkgen_i    => clk_gen,
        irq_o       => firq.neoled,
        neoled_o    => neoled_o
      );
    end generate;

    neorv32_neoled_inst_false:
    if (IO_NEOLED_EN = false) generate
      io_dev_rsp(IODEV_NEOLED) <= rsp_terminate_c;
      cg_en.neoled             <= '0';
      firq.neoled              <= '0';
      neoled_o                 <= '0';
    end generate;


    -- External Interrupt Controller (XIRQ) ---------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_xirq_inst_true:
    if (io_xirq_en_c = true) generate
      neorv32_xirq_inst: entity neorv32.neorv32_xirq
      generic map (
        XIRQ_NUM_CH           => XIRQ_NUM_CH,
        XIRQ_TRIGGER_TYPE     => XIRQ_TRIGGER_TYPE,
        XIRQ_TRIGGER_POLARITY => XIRQ_TRIGGER_POLARITY
      )
      port map (
        -- host access --
        clk_i     => clk_i,
        rstn_i    => rstn_sys,
        bus_req_i => io_dev_req(IODEV_XIRQ),
        bus_rsp_o => io_dev_rsp(IODEV_XIRQ),
        xirq_i    => xirq_i,
        cpu_irq_o => firq.xirq
      );
    end generate;

    neorv32_xirq_inst_false:
    if (io_xirq_en_c = false) generate
      io_dev_rsp(IODEV_XIRQ) <= rsp_terminate_c;
      firq.xirq              <= '0';
    end generate;


    -- General Purpose Timer (GPTMR) ----------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_gptmr_inst_true:
    if (IO_GPTMR_EN = true) generate
      neorv32_gptmr_inst: entity neorv32.neorv32_gptmr
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => io_dev_req(IODEV_GPTMR),
        bus_rsp_o   => io_dev_rsp(IODEV_GPTMR),
        clkgen_en_o => cg_en.gptmr,
        clkgen_i    => clk_gen,
        irq_o       => firq.gptmr
      );
    end generate;

    neorv32_gptmr_inst_false:
    if (IO_GPTMR_EN = false) generate
      io_dev_rsp(IODEV_GPTMR) <= rsp_terminate_c;
      cg_en.gptmr             <= '0';
      firq.gptmr              <= '0';
    end generate;


    -- 1-Wire Interface Controller (ONEWIRE) --------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_onewire_inst_true:
    if (IO_ONEWIRE_EN = true) generate
      neorv32_onewire_inst: entity neorv32.neorv32_onewire
      port map (
        clk_i       => clk_i,
        rstn_i      => rstn_sys,
        bus_req_i   => io_dev_req(IODEV_ONEWIRE),
        bus_rsp_o   => io_dev_rsp(IODEV_ONEWIRE),
        clkgen_en_o => cg_en.onewire,
        clkgen_i    => clk_gen,
        onewire_i   => onewire_i,
        onewire_o   => onewire_o,
        irq_o       => firq.onewire
      );
    end generate;

    neorv32_onewire_inst_false:
    if (IO_ONEWIRE_EN = false) generate
      io_dev_rsp(IODEV_ONEWIRE) <= rsp_terminate_c;
      onewire_o                 <= '1';
      cg_en.onewire             <= '0';
      firq.onewire              <= '0';
    end generate;


    -- Stream Link Interface (SLINK) ----------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_slink_inst_true:
    if (IO_SLINK_EN = true) generate
      neorv32_slink_inst: entity neorv32.neorv32_slink
      generic map (
        SLINK_RX_FIFO => IO_SLINK_RX_FIFO,
        SLINK_TX_FIFO => IO_SLINK_TX_FIFO
      )
      port map (
        -- Host access --
        clk_i            => clk_i,
        rstn_i           => rstn_sys,
        bus_req_i        => io_dev_req(IODEV_SLINK),
        bus_rsp_o        => io_dev_rsp(IODEV_SLINK),
        irq_o            => firq.slink,
        -- RX stream interface --
        slink_rx_data_i  => slink_rx_dat_i,
        slink_rx_valid_i => slink_rx_val_i,
        slink_rx_ready_o => slink_rx_rdy_o,
        -- TX stream interface --
        slink_tx_data_o  => slink_tx_dat_o,
        slink_tx_valid_o => slink_tx_val_o,
        slink_tx_ready_i => slink_tx_rdy_i
      );
    end generate;

    neorv32_slink_inst_false:
    if (IO_SLINK_EN = false) generate
      io_dev_rsp(IODEV_SLINK) <= rsp_terminate_c;
      firq.slink              <= '0';
      slink_rx_rdy_o          <= '0';
      slink_tx_dat_o          <= (others => '0');
      slink_tx_val_o          <= '0';
    end generate;


    -- Cyclic Redundancy Check Unit (CRC) -----------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_crc_inst_true:
    if (IO_CRC_EN = true) generate
      neorv32_crc_inst: entity neorv32.neorv32_crc
        port map (
        clk_i     => clk_i,
        rstn_i    => rstn_sys,
        bus_req_i => io_dev_req(IODEV_CRC),
        bus_rsp_o => io_dev_rsp(IODEV_CRC)
      );
    end generate;

    neorv32_crc_inst_false:
    if (IO_CRC_EN = false) generate
      io_dev_rsp(IODEV_CRC) <= rsp_terminate_c;
    end generate;


    -- System Configuration Information Memory (SYSINFO) --------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_sysinfo_inst: entity neorv32.neorv32_sysinfo
    generic map (
      -- General --
      CLOCK_FREQUENCY      => CLOCK_FREQUENCY,
      INT_BOOTLOADER_EN    => INT_BOOTLOADER_EN,
      -- internal Instruction memory --
      MEM_INT_IMEM_EN      => MEM_INT_IMEM_EN,
      MEM_INT_IMEM_SIZE    => imem_size_c,
      -- Internal Data memory --
      MEM_INT_DMEM_EN      => MEM_INT_DMEM_EN,
      MEM_INT_DMEM_SIZE    => dmem_size_c,
      -- Reservation Set Granularity --
      AMO_RVS_GRANULARITY  => AMO_RVS_GRANULARITY,
      -- Instruction cache --
      ICACHE_EN            => ICACHE_EN,
      ICACHE_NUM_BLOCKS    => ICACHE_NUM_BLOCKS,
      ICACHE_BLOCK_SIZE    => ICACHE_BLOCK_SIZE,
      ICACHE_ASSOCIATIVITY => ICACHE_ASSOCIATIVITY,
      -- Data cache --
      DCACHE_EN            => DCACHE_EN,
      DCACHE_NUM_BLOCKS    => DCACHE_NUM_BLOCKS,
      DCACHE_BLOCK_SIZE    => DCACHE_BLOCK_SIZE,
      -- External memory interface --
      MEM_EXT_EN           => MEM_EXT_EN,
      MEM_EXT_BIG_ENDIAN   => MEM_EXT_BIG_ENDIAN,
      -- On-Chip Debugger --
      ON_CHIP_DEBUGGER_EN  => ON_CHIP_DEBUGGER_EN,
      -- Processor peripherals --
      IO_GPIO_EN           => io_gpio_en_c,
      IO_MTIME_EN          => IO_MTIME_EN,
      IO_UART0_EN          => IO_UART0_EN,
      IO_UART1_EN          => IO_UART1_EN,
      IO_SPI_EN            => IO_SPI_EN,
      IO_SDI_EN            => IO_SDI_EN,
      IO_TWI_EN            => IO_TWI_EN,
      IO_PWM_EN            => io_pwm_en_c,
      IO_WDT_EN            => IO_WDT_EN,
      IO_TRNG_EN           => IO_TRNG_EN,
      IO_CFS_EN            => IO_CFS_EN,
      IO_NEOLED_EN         => IO_NEOLED_EN,
      IO_XIRQ_EN           => io_xirq_en_c,
      IO_GPTMR_EN          => IO_GPTMR_EN,
      IO_XIP_EN            => IO_XIP_EN,
      IO_ONEWIRE_EN        => IO_ONEWIRE_EN,
      IO_DMA_EN            => IO_DMA_EN,
      IO_SLINK_EN          => IO_SLINK_EN,
      IO_CRC_EN            => IO_CRC_EN
    )
    port map (
      clk_i     => clk_i,
      bus_req_i => io_dev_req(IODEV_SYSINFO),
      bus_rsp_o => io_dev_rsp(IODEV_SYSINFO)
    );

  end generate; -- /io_system


  -- **************************************************************************************************************************
  -- On-Chip Debugger Complex
  -- **************************************************************************************************************************
  neorv32_neorv32_ocd_inst_true:
  if (ON_CHIP_DEBUGGER_EN = true) generate

    -- On-Chip Debugger - Debug Transport Module (DTM) ----------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_debug_dtm_inst: entity neorv32.neorv32_debug_dtm
    generic map (
      IDCODE_VERSION => (others => '0'),
      IDCODE_PARTID  => (others => '0'),
      IDCODE_MANID   => (others => '0')
    )
    port map (
      -- global control --
      clk_i        => clk_i,
      rstn_i       => rstn_ext,
      -- jtag connection --
      jtag_trst_i  => jtag_trst_i,
      jtag_tck_i   => jtag_tck_i,
      jtag_tdi_i   => jtag_tdi_i,
      jtag_tdo_o   => jtag_tdo_o,
      jtag_tms_i   => jtag_tms_i,
      -- debug module interface (DMI) --
      dmi_req_o    => dmi_req,
      dmi_rsp_i    => dmi_rsp
    );

    -- On-Chip Debugger - Debug Module (DM) ---------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_debug_dm_inst: entity neorv32.neorv32_debug_dm
    generic map (
      CPU_BASE_ADDR => base_io_dm_c,
      LEGACY_MODE   => DM_LEGACY_MODE
    )
    port map (
      -- global control --
      clk_i          => clk_i,
      rstn_i         => rstn_ext,
      cpu_debug_i    => cpu_debug,
      -- debug module interface (DMI) --
      dmi_req_i      => dmi_req,
      dmi_rsp_o      => dmi_rsp,
      -- CPU bus access --
      bus_req_i      => io_dev_req(IODEV_OCD),
      bus_rsp_o      => io_dev_rsp(IODEV_OCD),
      -- CPU control --
      cpu_ndmrstn_o  => dci_ndmrstn,
      cpu_halt_req_o => dci_halt_req
    );

  end generate;

  neorv32_debug_ocd_inst_false:
  if (ON_CHIP_DEBUGGER_EN = false) generate
    io_dev_rsp(IODEV_OCD) <= rsp_terminate_c;
    jtag_tdo_o            <= jtag_tdi_i; -- JTAG feed-through
    dci_ndmrstn           <= '1';
    dci_halt_req          <= '0';
  end generate;


end neorv32_top_rtl;
