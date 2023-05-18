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
    CLOCK_FREQUENCY              : natural;           -- clock frequency of clk_i in Hz
    HART_ID                      : std_ulogic_vector(31 downto 0) := x"00000000"; -- hardware thread ID
    VENDOR_ID                    : std_ulogic_vector(31 downto 0) := x"00000000"; -- vendor's JEDEC ID
    CUSTOM_ID                    : std_ulogic_vector(31 downto 0) := x"00000000"; -- custom user-defined ID
    INT_BOOTLOADER_EN            : boolean := false;  -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM

    -- On-Chip Debugger (OCD) --
    ON_CHIP_DEBUGGER_EN          : boolean := false;  -- implement on-chip debugger

    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_B        : boolean := false;  -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_C        : boolean := false;  -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        : boolean := false;  -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        : boolean := false;  -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U        : boolean := false;  -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    : boolean := false;  -- implement 32-bit floating-point extension (using INT regs!)
    CPU_EXTENSION_RISCV_Zicntr   : boolean := true;   -- implement base counters?
    CPU_EXTENSION_RISCV_Zicond   : boolean := false;  -- implement conditional operations extension?
    CPU_EXTENSION_RISCV_Zihpm    : boolean := false;  -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zifencei : boolean := false;  -- implement instruction stream sync.?
    CPU_EXTENSION_RISCV_Zmmul    : boolean := false;  -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zxcfu    : boolean := false;  -- implement custom (instr.) functions unit?

    -- Tuning Options --
    FAST_MUL_EN                  : boolean := false;  -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                : boolean := false;  -- use barrel shifter for shift operations
    CPU_IPB_ENTRIES              : natural := 1;      -- entries in instruction prefetch buffer, has to be a power of 2, min 1

    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS              : natural := 0;      -- number of regions (0..16)
    PMP_MIN_GRANULARITY          : natural := 4;      -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes

    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 : natural := 0;      -- number of implemented HPM counters (0..29)
    HPM_CNT_WIDTH                : natural := 40;     -- total size of HPM counters (0..64)

    -- Internal Instruction memory (IMEM) --
    MEM_INT_IMEM_EN              : boolean := false;  -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            : natural := 16*1024; -- size of processor-internal instruction memory in bytes

    -- Internal Data memory (DMEM) --
    MEM_INT_DMEM_EN              : boolean := false;  -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            : natural := 8*1024; -- size of processor-internal data memory in bytes

    -- Internal Instruction Cache (iCACHE) --
    ICACHE_EN                    : boolean := false;  -- implement instruction cache
    ICACHE_NUM_BLOCKS            : natural := 4;      -- i-cache: number of blocks (min 1), has to be a power of 2
    ICACHE_BLOCK_SIZE            : natural := 64;     -- i-cache: block size in bytes (min 4), has to be a power of 2
    ICACHE_ASSOCIATIVITY         : natural := 1;      -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2

    -- Internal Data Cache (dCACHE) --
    DCACHE_EN                    : boolean := false;  -- implement data cache
    DCACHE_NUM_BLOCKS            : natural := 4;      -- d-cache: number of blocks (min 1), has to be a power of 2
    DCACHE_BLOCK_SIZE            : natural := 64;     -- d-cache: block size in bytes (min 4), has to be a power of 2

    -- External memory interface (WISHBONE) --
    MEM_EXT_EN                   : boolean := false;  -- implement external memory bus interface?
    MEM_EXT_TIMEOUT              : natural := 255;    -- cycles after a pending bus access auto-terminates (0 = disabled)
    MEM_EXT_PIPE_MODE            : boolean := false;  -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
    MEM_EXT_BIG_ENDIAN           : boolean := false;  -- byte order: true=big-endian, false=little-endian
    MEM_EXT_ASYNC_RX             : boolean := false;  -- use register buffer for RX data when false
    MEM_EXT_ASYNC_TX             : boolean := false;  -- use register buffer for TX data when false

    -- External Interrupts Controller (XIRQ) --
    XIRQ_NUM_CH                  : natural := 0;      -- number of external IRQ channels (0..32)
    XIRQ_TRIGGER_TYPE            : std_ulogic_vector(31 downto 0) := x"ffffffff"; -- trigger type: 0=level, 1=edge
    XIRQ_TRIGGER_POLARITY        : std_ulogic_vector(31 downto 0) := x"ffffffff"; -- trigger polarity: 0=low-level/falling-edge, 1=high-level/rising-edge

    -- Processor peripherals --
    IO_GPIO_NUM                  : natural := 0;      -- number of GPIO input/output pairs (0..64)
    IO_MTIME_EN                  : boolean := false;  -- implement machine system timer (MTIME)?
    IO_UART0_EN                  : boolean := false;  -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_UART0_RX_FIFO             : natural := 1;      -- RX fifo depth, has to be a power of two, min 1
    IO_UART0_TX_FIFO             : natural := 1;      -- TX fifo depth, has to be a power of two, min 1
    IO_UART1_EN                  : boolean := false;  -- implement secondary universal asynchronous receiver/transmitter (UART1)?
    IO_UART1_RX_FIFO             : natural := 1;      -- RX fifo depth, has to be a power of two, min 1
    IO_UART1_TX_FIFO             : natural := 1;      -- TX fifo depth, has to be a power of two, min 1
    IO_SPI_EN                    : boolean := false;  -- implement serial peripheral interface (SPI)?
    IO_SPI_FIFO                  : natural := 1;      -- RTX fifo depth, has to be a power of two, min 1
    IO_SDI_EN                    : boolean := false;  -- implement serial data interface (SDI)?
    IO_SDI_FIFO                  : natural := 0;      -- RTX fifo depth, has to be zero or a power of two
    IO_TWI_EN                    : boolean := false;  -- implement two-wire interface (TWI)?
    IO_PWM_NUM_CH                : natural := 0;      -- number of PWM channels to implement (0..12); 0 = disabled
    IO_WDT_EN                    : boolean := false;  -- implement watch dog timer (WDT)?
    IO_TRNG_EN                   : boolean := false;  -- implement true random number generator (TRNG)?
    IO_TRNG_FIFO                 : natural := 1;      -- data fifo depth, has to be a power of two, min 1
    IO_CFS_EN                    : boolean := false;  -- implement custom functions subsystem (CFS)?
    IO_CFS_CONFIG                : std_ulogic_vector(31 downto 0) := x"00000000"; -- custom CFS configuration generic
    IO_CFS_IN_SIZE               : natural := 32;     -- size of CFS input conduit in bits
    IO_CFS_OUT_SIZE              : natural := 32;     -- size of CFS output conduit in bits
    IO_NEOLED_EN                 : boolean := false;  -- implement NeoPixel-compatible smart LED interface (NEOLED)?
    IO_NEOLED_TX_FIFO            : natural := 1;      -- NEOLED FIFO depth, has to be a power of two, min 1
    IO_GPTMR_EN                  : boolean := false;  -- implement general purpose timer (GPTMR)?
    IO_XIP_EN                    : boolean := false;  -- implement execute in place module (XIP)?
    IO_ONEWIRE_EN                : boolean := false;  -- implement 1-wire interface (ONEWIRE)?
    IO_DMA_EN                    : boolean := false   -- implement direct memory access controller (DMA)?
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

  -- CPU boot configuration --
  constant cpu_boot_addr_c : std_ulogic_vector(31 downto 0) := cond_sel_stdulogicvector_f(INT_BOOTLOADER_EN, boot_rom_base_c, ispace_base_c);

  -- reset generator --
  signal rstn_ext_sreg, rstn_int_sreg : std_ulogic_vector(3 downto 0);
  signal rstn_ext, rstn_int, rstn_wdt : std_ulogic;

  -- clock generator --
  signal clk_div       : std_ulogic_vector(11 downto 0);
  signal clk_div_ff    : std_ulogic_vector(11 downto 0);
  signal clk_gen       : std_ulogic_vector(07 downto 0);
  signal clk_gen_en    : std_ulogic_vector(10 downto 0);
  signal clk_gen_en_ff : std_ulogic;
  --
  signal wdt_cg_en     : std_ulogic;
  signal uart0_cg_en   : std_ulogic;
  signal uart1_cg_en   : std_ulogic;
  signal spi_cg_en     : std_ulogic;
  signal twi_cg_en     : std_ulogic;
  signal pwm_cg_en     : std_ulogic;
  signal cfs_cg_en     : std_ulogic;
  signal neoled_cg_en  : std_ulogic;
  signal gptmr_cg_en   : std_ulogic;
  signal xip_cg_en     : std_ulogic;
  signal onewire_cg_en : std_ulogic;

  -- CPU status --
  signal cpu_debug : std_ulogic; -- cpu is in debug mode
  signal cpu_sleep : std_ulogic; -- cpu is in sleep mode
  signal i_fence   : std_ulogic; -- instruction fence
  signal d_fence   : std_ulogic; -- data fence

  -- debug module interface (DMI) --
  type dmi_t is record
    req_valid   : std_ulogic;
    req_ready   : std_ulogic; -- DMI is allowed to make new requests when set
    req_address : std_ulogic_vector(05 downto 0);
    req_op      : std_ulogic_vector(01 downto 0);
    req_data    : std_ulogic_vector(31 downto 0);
    rsp_valid   : std_ulogic; -- response valid when set
    rsp_ready   : std_ulogic; -- ready to receive respond
    rsp_data    : std_ulogic_vector(31 downto 0);
    rsp_op      : std_ulogic_vector(01 downto 0);
  end record;
  signal dmi : dmi_t;

  -- debug core interface (DCI) --
  signal dci_ndmrstn  : std_ulogic;
  signal dci_halt_req : std_ulogic;

  -- internal bus system --
  type device_ids_t is (DEV_BUSKEEPER, DEV_IMEM, DEV_DMEM, DEV_BOOTROM, DEV_WISHBONE, DEV_GPIO, DEV_MTIME, DEV_UART0,
                       DEV_UART1, DEV_SPI, DEV_TWI, DEV_PWM, DEV_WDT, DEV_TRNG, DEV_CFS, DEV_NEOLED, DEV_SYSINFO,
                       DEV_OCD, DEV_XIRQ, DEV_GPTMR, DEV_XIP_CT, DEV_XIP_ACC, DEV_ONEWIRE, DEV_SDI, DEV_DMA);

  -- core complex --
  signal cpu_i_req,  cpu_d_req  : bus_req_t; -- CPU core
  signal cpu_i_rsp,  cpu_d_rsp  : bus_rsp_t; -- CPU core
  signal icache_req, dcache_req : bus_req_t; -- CPU caches
  signal icache_rsp, dcache_rsp : bus_rsp_t; -- CPU caches
  signal core_req               : bus_req_t; -- core complex (CPU + caches)
  signal core_rsp               : bus_rsp_t; -- core complex (CPU + caches)

  -- core complex + DMA --
  signal main_req, dma_req : bus_req_t; -- core complex (CPU + caches + DMA)
  signal main_rsp, dma_rsp : bus_rsp_t; -- core complex (CPU + caches + DMA)

  -- SoC bus --
  type response_bus_t is array (device_ids_t) of bus_rsp_t;
  signal soc_req   : bus_req_t; -- SoC request bus
  signal soc_rsp   : bus_rsp_t; -- SoC response bus
  signal io_req    : bus_req_t; -- request bus for internal IO/Peripheral devices only
  signal rsp_bus   : response_bus_t; -- global response bus
  signal bus_error : std_ulogic; -- global bus error

  -- IRQs --
  signal fast_irq     : std_ulogic_vector(15 downto 0);
  signal mtime_irq    : std_ulogic;
  signal wdt_irq      : std_ulogic;
  signal uart0_rx_irq : std_ulogic;
  signal uart0_tx_irq : std_ulogic;
  signal uart1_rx_irq : std_ulogic;
  signal uart1_tx_irq : std_ulogic;
  signal spi_irq      : std_ulogic;
  signal sdi_irq      : std_ulogic;
  signal twi_irq      : std_ulogic;
  signal cfs_irq      : std_ulogic;
  signal neoled_irq   : std_ulogic;
  signal xirq_irq     : std_ulogic;
  signal gptmr_irq    : std_ulogic;
  signal onewire_irq  : std_ulogic;
  signal dma_irq      : std_ulogic;
  signal trng_irq     : std_ulogic;

  -- misc --
  signal io_acc      : std_ulogic;
  signal ext_timeout : std_ulogic;
  signal ext_access  : std_ulogic;
  signal xip_access  : std_ulogic;
  signal xip_enable  : std_ulogic;
  signal xip_page    : std_ulogic_vector(3 downto 0);

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert false report
    "NEORV32 PROCESSOR CONFIGURATION: " &
    cond_sel_string_f(MEM_INT_IMEM_EN, "IMEM ", "") &
    cond_sel_string_f(MEM_INT_DMEM_EN, "DMEM ", "") &
    cond_sel_string_f(INT_BOOTLOADER_EN, "BOOTROM ", "") &
    cond_sel_string_f(ICACHE_EN, "I-CACHE ", "") &
    cond_sel_string_f(DCACHE_EN, "D-CACHE ", "") &
    cond_sel_string_f(MEM_EXT_EN, "WISHBONE ", "") &
    cond_sel_string_f(ON_CHIP_DEBUGGER_EN, "OCD ", "") &
    "+ " &
    cond_sel_string_f(boolean(IO_GPIO_NUM > 0), "GPIO ", "") &
    cond_sel_string_f(IO_MTIME_EN, "MTIME ", "") &
    cond_sel_string_f(IO_UART0_EN, "UART0 ", "") &
    cond_sel_string_f(IO_UART1_EN, "UART1 ", "") &
    cond_sel_string_f(IO_SPI_EN, "SPI ", "") &
    cond_sel_string_f(IO_SDI_EN, "SDI ", "") &
    cond_sel_string_f(IO_TWI_EN, "TWI ", "") &
    cond_sel_string_f(boolean(IO_PWM_NUM_CH > 0), "PWM ", "") &
    cond_sel_string_f(IO_WDT_EN, "WDT ", "") &
    cond_sel_string_f(IO_TRNG_EN, "TRNG ", "") &
    cond_sel_string_f(IO_CFS_EN, "CFS ", "") &
    cond_sel_string_f(IO_NEOLED_EN, "NEOLED ", "") &
    cond_sel_string_f(boolean(XIRQ_NUM_CH > 0), "XIRQ ", "") &
    cond_sel_string_f(IO_GPTMR_EN, "GPTMR ", "") &
    cond_sel_string_f(IO_XIP_EN, "XIP ", "") &
    cond_sel_string_f(IO_ONEWIRE_EN, "ONEWIRE ", "") &
    cond_sel_string_f(IO_DMA_EN, "DMA ", "") &
    ""
    severity note;

  -- boot configuration --
  assert not (INT_BOOTLOADER_EN = true) report
    "NEORV32 PROCESSOR CONFIG NOTE: Boot configuration: Indirect boot via bootloader (processor-internal BOOTROM)." severity note;
  assert not ((INT_BOOTLOADER_EN = false) and (MEM_INT_IMEM_EN = true)) report
    "NEORV32 PROCESSOR CONFIG NOTE: Boot configuration = direct boot from memory (processor-internal IMEM)." severity note;
  assert not ((INT_BOOTLOADER_EN = false) and (MEM_INT_IMEM_EN = false)) report
    "NEORV32 PROCESSOR CONFIG NOTE: Boot configuration = direct boot from memory (processor-external memory)." severity note;

  -- memory layout --
  assert not (ispace_base_c /= x"00000000") report
    "NEORV32 PROCESSOR CONFIG WARNING! Non-default base address for INSTRUCTION ADDRESS SPACE. Make sure this is sync with the software framework." severity warning;
  assert not (dspace_base_c /= x"80000000") report
    "NEORV32 PROCESSOR CONFIG WARNING! Non-default base address for DATA ADDRESS SPACE. Make sure this is sync with the software framework." severity warning;

  -- on-chip debugger --
  assert not (ON_CHIP_DEBUGGER_EN = true) report
    "NEORV32 PROCESSOR CONFIG NOTE: Implementing on-chip debugger (OCD)." severity note;

  -- caches --
  assert not ((ICACHE_EN = true) and (CPU_EXTENSION_RISCV_Zifencei = false)) report
    "NEORV32 CPU CONFIG WARNING! <CPU_EXTENSION_RISCV_Zifencei> ISA extension is required to perform i-cache memory sync operations." severity warning;


-- ****************************************************************************************************************************
-- Clock and Reset
-- ****************************************************************************************************************************

  -- Reset Generator ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  reset_generator: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      rstn_ext_sreg <= (others => '0');
      rstn_int_sreg <= (others => '0');
      rstn_ext      <= '0';
      rstn_int      <= '0';
    elsif falling_edge(clk_i) then -- inverted clock to release reset _before_ all FFs trigger (rising edge)
      -- external reset --
      rstn_ext_sreg <= rstn_ext_sreg(rstn_ext_sreg'left-1 downto 0) & '1'; -- active for at least <rstn_ext_sreg'size> clock cycles
      -- internal reset --
      if (rstn_wdt = '0') or (dci_ndmrstn = '0') then -- sync reset sources
        rstn_int_sreg <= (others => '0');
      else
        rstn_int_sreg <= rstn_int_sreg(rstn_int_sreg'left-1 downto 0) & '1'; -- active for at least <rstn_int_sreg'size> clock cycles
      end if;
      -- reset nets --
      rstn_ext <= and_reduce_f(rstn_ext_sreg); -- external reset (via reset pin)
      rstn_int <= and_reduce_f(rstn_int_sreg); -- internal reset (via reset pin, WDT or OCD)
    end if;
  end process reset_generator;


  -- Clock Generator ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  clock_generator: process(rstn_int, clk_i)
  begin
    if (rstn_int = '0') then
      clk_gen_en_ff <= '0';
      clk_div       <= (others => '0');
      clk_div_ff    <= (others => '0');
    elsif rising_edge(clk_i) then
      clk_gen_en_ff <= or_reduce_f(clk_gen_en);
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
  clk_gen_en(0)  <= wdt_cg_en;
  clk_gen_en(1)  <= uart0_cg_en;
  clk_gen_en(2)  <= uart1_cg_en;
  clk_gen_en(3)  <= spi_cg_en;
  clk_gen_en(4)  <= twi_cg_en;
  clk_gen_en(5)  <= pwm_cg_en;
  clk_gen_en(6)  <= cfs_cg_en;
  clk_gen_en(7)  <= neoled_cg_en;
  clk_gen_en(8)  <= gptmr_cg_en;
  clk_gen_en(9)  <= xip_cg_en;
  clk_gen_en(10) <= onewire_cg_en;


-- ****************************************************************************************************************************
-- Core Complex
-- ****************************************************************************************************************************

  -- CPU Core -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_inst: neorv32_cpu
  generic map (
    -- General --
    HART_ID                      => HART_ID,
    VENDOR_ID                    => VENDOR_ID,
    CPU_BOOT_ADDR                => cpu_boot_addr_c,
    CPU_DEBUG_PARK_ADDR          => dm_park_entry_c,
    CPU_DEBUG_EXC_ADDR           => dm_exc_entry_c,
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_B        => CPU_EXTENSION_RISCV_B,
    CPU_EXTENSION_RISCV_C        => CPU_EXTENSION_RISCV_C,
    CPU_EXTENSION_RISCV_E        => CPU_EXTENSION_RISCV_E,
    CPU_EXTENSION_RISCV_M        => CPU_EXTENSION_RISCV_M,
    CPU_EXTENSION_RISCV_U        => CPU_EXTENSION_RISCV_U,
    CPU_EXTENSION_RISCV_Zfinx    => CPU_EXTENSION_RISCV_Zfinx,
    CPU_EXTENSION_RISCV_Zicntr   => CPU_EXTENSION_RISCV_Zicntr,
    CPU_EXTENSION_RISCV_Zicond   => CPU_EXTENSION_RISCV_Zicond,
    CPU_EXTENSION_RISCV_Zihpm    => CPU_EXTENSION_RISCV_Zihpm,
    CPU_EXTENSION_RISCV_Zifencei => CPU_EXTENSION_RISCV_Zifencei,
    CPU_EXTENSION_RISCV_Zmmul    => CPU_EXTENSION_RISCV_Zmmul,
    CPU_EXTENSION_RISCV_Zxcfu    => CPU_EXTENSION_RISCV_Zxcfu,
    CPU_EXTENSION_RISCV_Sdext    => ON_CHIP_DEBUGGER_EN,
    CPU_EXTENSION_RISCV_Sdtrig   => ON_CHIP_DEBUGGER_EN,
    -- Extension Options --
    FAST_MUL_EN                  => FAST_MUL_EN,
    FAST_SHIFT_EN                => FAST_SHIFT_EN,
    CPU_IPB_ENTRIES              => CPU_IPB_ENTRIES,
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
    rstn_i     => rstn_int,
    sleep_o    => cpu_sleep,
    debug_o    => cpu_debug,
    ifence_o   => i_fence,
    dfence_o   => d_fence,
    -- interrupts --
    msi_i      => msw_irq_i,
    mei_i      => mext_irq_i,
    mti_i      => mtime_irq,
    firq_i     => fast_irq,
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
  fast_irq(00) <= wdt_irq; -- highest priority
  fast_irq(01) <= cfs_irq;
  fast_irq(02) <= uart0_rx_irq;
  fast_irq(03) <= uart0_tx_irq;
  fast_irq(04) <= uart1_rx_irq;
  fast_irq(05) <= uart1_tx_irq;
  fast_irq(06) <= spi_irq;
  fast_irq(07) <= twi_irq;
  fast_irq(08) <= xirq_irq;
  fast_irq(09) <= neoled_irq;
  fast_irq(10) <= dma_irq;
  fast_irq(11) <= sdi_irq;
  fast_irq(12) <= gptmr_irq;
  fast_irq(13) <= onewire_irq;
  fast_irq(14) <= '0';
  fast_irq(15) <= trng_irq; -- lowest priority


  -- CPU Instruction Cache ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_icache_inst_true:
  if (ICACHE_EN = true) generate
    neorv32_icache_inst: neorv32_icache
    generic map (
      ICACHE_NUM_BLOCKS => ICACHE_NUM_BLOCKS,
      ICACHE_BLOCK_SIZE => ICACHE_BLOCK_SIZE,
      ICACHE_NUM_SETS   => ICACHE_ASSOCIATIVITY
    )
    port map (
      clk_i     => clk_i,
      rstn_i    => rstn_int,
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
    neorv32_dcache_inst: neorv32_dcache
    generic map (
      DCACHE_NUM_BLOCKS => DCACHE_NUM_BLOCKS,
      DCACHE_BLOCK_SIZE => DCACHE_BLOCK_SIZE,
      DCACHE_UC_PBEGIN  => "1111"
    )
    port map (
      clk_i     => clk_i,
      rstn_i    => rstn_int,
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
  neorv32_core_busswitch_inst: neorv32_busswitch
  generic map (
    PORT_A_READ_ONLY => false,
    PORT_B_READ_ONLY => true -- i-fetch is read-only
  )
  port map (
    clk_i   => clk_i,
    rstn_i  => rstn_int,
    a_req_i => dcache_req,
    a_rsp_o => dcache_rsp,
    b_req_i => icache_req,
    b_rsp_o => icache_rsp,
    x_req_o => core_req,
    x_rsp_i => core_rsp
  );


-- ****************************************************************************************************************************
-- Direct Memory Access Controller (DMA) Complex
-- ****************************************************************************************************************************

  -- DMA controller and according bus switch --
  neorv32_dma_complex_true:
  if (IO_DMA_EN = true) generate

    -- DMA Controller -------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_dma_inst: neorv32_dma
    port map (
      clk_i     => clk_i,
      rstn_i    => rstn_int,
      bus_req_i => io_req,
      bus_rsp_o => rsp_bus(DEV_DMA),
      dma_req_o => dma_req,
      dma_rsp_i => dma_rsp,
      irq_o     => dma_irq
    );

    -- DMA Bus Switch -------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_dma_busswitch_inst: neorv32_busswitch
    generic map (
      PORT_A_READ_ONLY => false,
      PORT_B_READ_ONLY => false
    )
    port map (
      clk_i   => clk_i,
      rstn_i  => rstn_int,
      a_req_i => core_req,
      a_rsp_o => core_rsp,
      b_req_i => dma_req,
      b_rsp_o => dma_rsp,
      x_req_o => main_req,
      x_rsp_i => main_rsp
    );

  end generate;

  neorv32_dma_complex_false:
  if (IO_DMA_EN = false) generate
    rsp_bus(DEV_DMA) <= rsp_terminate_c;
    main_req <= core_req;
    core_rsp <= main_rsp;
    dma_irq  <= '0';
  end generate;


-- ****************************************************************************************************************************
-- Bus System
-- ****************************************************************************************************************************

  -- Bus Keeper (BUSKEEPER) -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_bus_keeper_inst: neorv32_bus_keeper
  port map (
    clk_i      => clk_i,
    rstn_i     => rstn_int,
    cpu_req_i  => io_req,
    cpu_rsp_o  => rsp_bus(DEV_BUSKEEPER),
    bus_req_i  => soc_req,
    bus_rsp_i  => soc_rsp,
    bus_err_o  => bus_error,
    bus_tmo_i  => ext_timeout,
    bus_ext_i  => ext_access,
    bus_xip_i  => xip_access
  );

  -- global bus response ---
  bus_response: process(rsp_bus)
    variable tmp_v : bus_rsp_t;
  begin
    tmp_v := rsp_terminate_c;
    for i in rsp_bus'range loop -- OR all response signals
      tmp_v.data := tmp_v.data or rsp_bus(i).data;
      tmp_v.ack  := tmp_v.ack  or rsp_bus(i).ack;
      tmp_v.err  := tmp_v.err  or rsp_bus(i).err;
    end loop;
    soc_rsp <= tmp_v;
  end process;

  -- central SoC bus --
  soc_req <= main_req;
  main_rsp.data <= soc_rsp.data;
  main_rsp.ack  <= soc_rsp.ack;
  main_rsp.err  <= bus_error; -- global bus error (buskeeper -> core)


-- ****************************************************************************************************************************
-- Memory System
-- ****************************************************************************************************************************

  -- Processor-Internal Instruction Memory (IMEM) -------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_int_imem_inst_true:
  if (MEM_INT_IMEM_EN = true) and (MEM_INT_IMEM_SIZE > 0) generate
    neorv32_int_imem_inst: neorv32_imem
    generic map (
      IMEM_BASE    => imem_base_c,
      IMEM_SIZE    => MEM_INT_IMEM_SIZE,
      IMEM_AS_IROM => not INT_BOOTLOADER_EN
    )
    port map (
      clk_i     => clk_i,
      bus_req_i => soc_req,
      bus_rsp_o => rsp_bus(DEV_IMEM)
    );
  end generate;

  neorv32_int_imem_inst_false:
  if (MEM_INT_IMEM_EN = false) or (MEM_INT_IMEM_SIZE = 0) generate
    rsp_bus(DEV_IMEM) <= rsp_terminate_c;
  end generate;

  -- Processor-Internal Data Memory (DMEM) --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_int_dmem_inst_true:
  if (MEM_INT_DMEM_EN = true) and (MEM_INT_DMEM_SIZE > 0) generate
    neorv32_int_dmem_inst: neorv32_dmem
    generic map (
      DMEM_BASE => dmem_base_c,
      DMEM_SIZE => MEM_INT_DMEM_SIZE
    )
    port map (
      clk_i     => clk_i,
      bus_req_i => soc_req,
      bus_rsp_o => rsp_bus(DEV_DMEM)
    );
  end generate;

  neorv32_int_dmem_inst_false:
  if (MEM_INT_DMEM_EN = false) or (MEM_INT_DMEM_SIZE = 0) generate
    rsp_bus(DEV_DMEM) <= rsp_terminate_c;
  end generate;


  -- Processor-Internal Bootloader ROM (BOOTROM) --------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_boot_rom_inst_true:
  if (INT_BOOTLOADER_EN = true) generate
    neorv32_boot_rom_inst: neorv32_boot_rom
    generic map (
      BOOTROM_BASE => boot_rom_base_c
    )
    port map (
      clk_i     => clk_i,
      bus_req_i => soc_req,
      bus_rsp_o => rsp_bus(DEV_BOOTROM)
    );
  end generate;

  neorv32_boot_rom_inst_false:
  if (INT_BOOTLOADER_EN = false) generate
    rsp_bus(DEV_BOOTROM) <= rsp_terminate_c;
  end generate;


  -- External Wishbone Gateway (WISHBONE) ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_wishbone_inst_true:
  if (MEM_EXT_EN = true) generate
    neorv32_wishbone_inst: neorv32_wishbone
    generic map (
      -- Internal instruction memory --
      MEM_INT_IMEM_EN   => MEM_INT_IMEM_EN,
      MEM_INT_IMEM_SIZE => MEM_INT_IMEM_SIZE,
      -- Internal data memory --
      MEM_INT_DMEM_EN   => MEM_INT_DMEM_EN,
      MEM_INT_DMEM_SIZE => MEM_INT_DMEM_SIZE,
      -- Interface Configuration --
      BUS_TIMEOUT       => MEM_EXT_TIMEOUT,
      PIPE_MODE         => MEM_EXT_PIPE_MODE,
      BIG_ENDIAN        => MEM_EXT_BIG_ENDIAN,
      ASYNC_RX          => MEM_EXT_ASYNC_RX,
      ASYNC_TX          => MEM_EXT_ASYNC_TX
    )
    port map (
      -- global control --
      clk_i      => clk_i,
      rstn_i     => rstn_int,
      bus_req_i  => soc_req,
      bus_rsp_o  => rsp_bus(DEV_WISHBONE),
      tmo_o      => ext_timeout,
      ext_o      => ext_access,
      xip_en_i   => xip_enable,
      xip_page_i => xip_page,
      --
      wb_tag_o   => wb_tag_o,
      wb_adr_o   => wb_adr_o,
      wb_dat_i   => wb_dat_i,
      wb_dat_o   => wb_dat_o,
      wb_we_o    => wb_we_o,
      wb_sel_o   => wb_sel_o,
      wb_stb_o   => wb_stb_o,
      wb_cyc_o   => wb_cyc_o,
      wb_ack_i   => wb_ack_i,
      wb_err_i   => wb_err_i
    );
  end generate;

  neorv32_wishbone_inst_false:
  if (MEM_EXT_EN = false) generate
    rsp_bus(DEV_WISHBONE) <= rsp_terminate_c;
    ext_timeout <= '0';
    ext_access  <= '0';
    wb_adr_o    <= (others => '0');
    wb_dat_o    <= (others => '0');
    wb_we_o     <= '0';
    wb_sel_o    <= (others => '0');
    wb_stb_o    <= '0';
    wb_cyc_o    <= '0';
    wb_tag_o    <= (others => '0');
  end generate;


  -- Execute In Place Module (XIP) ----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_xip_inst_true:
  if (IO_XIP_EN = true) generate
    neorv32_xip_inst: neorv32_xip
    port map (
      -- global control --
      clk_i       => clk_i,
      rstn_i      => rstn_int,
      bus_req_i   => io_req,
      bus_rsp_o   => rsp_bus(DEV_XIP_CT),
      xip_req_i   => soc_req,
      xip_rsp_o   => rsp_bus(DEV_XIP_ACC),
      xip_en_o    => xip_enable,
      xip_acc_o   => xip_access,
      xip_page_o  => xip_page,
      clkgen_en_o => xip_cg_en,
      clkgen_i    => clk_gen,
      spi_csn_o   => xip_csn_o,
      spi_clk_o   => xip_clk_o,
      spi_dat_i   => xip_dat_i,
      spi_dat_o   => xip_dat_o
    );
  end generate;

  neorv32_xip_inst_false:
  if (IO_XIP_EN = false) generate
    rsp_bus(DEV_XIP_CT)  <= rsp_terminate_c;
    rsp_bus(DEV_XIP_ACC) <= rsp_terminate_c;
    xip_enable <= '0';
    xip_access <= '0';
    xip_page   <= (others => '0');
    xip_cg_en  <= '0';
    xip_csn_o  <= '1';
    xip_clk_o  <= '0';
    xip_dat_o  <= '0';
  end generate;


-- ****************************************************************************************************************************
-- IO/Peripheral Modules
-- ****************************************************************************************************************************

  -- IO Gateway -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  io_gateway: process(soc_req, io_acc)
  begin
    io_req    <= soc_req;
    io_req.re <= io_acc and soc_req.re and (not soc_req.src); -- PMA: read access only from data interface
    io_req.we <= io_acc and soc_req.we and and_reduce_f(soc_req.ben); -- PMA: full-word write accesses only
  end process io_gateway;

  -- IO access? --
  io_acc <= '1' when (soc_req.addr(31 downto index_size_f(io_size_c)) = io_base_c(31 downto index_size_f(io_size_c))) else '0';


  -- Custom Functions Subsystem (CFS) -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cfs_inst_true:
  if (IO_CFS_EN = true) generate
    neorv32_cfs_inst: neorv32_cfs
    generic map (
      CFS_CONFIG   => IO_CFS_CONFIG,
      CFS_IN_SIZE  => IO_CFS_IN_SIZE,
      CFS_OUT_SIZE => IO_CFS_OUT_SIZE
    )
    port map (
      clk_i       => clk_i,
      rstn_i      => rstn_int,
      bus_req_i   => io_req,
      bus_rsp_o   => rsp_bus(DEV_CFS),
      clkgen_en_o => cfs_cg_en,
      clkgen_i    => clk_gen,
      irq_o       => cfs_irq,
      cfs_in_i    => cfs_in_i,
      cfs_out_o   => cfs_out_o
    );
  end generate;

  neorv32_cfs_inst_false:
  if (IO_CFS_EN = false) generate
    rsp_bus(DEV_CFS) <= rsp_terminate_c;
    cfs_cg_en <= '0';
    cfs_irq   <= '0';
    cfs_out_o <= (others => '0');
  end generate;


  -- Serial Data Interface (SDI) ------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_sdi_inst_true:
  if (IO_SDI_EN = true) generate
    neorv32_SDI_inst: neorv32_sdi
    generic map (
      RTX_FIFO => IO_SDI_FIFO
    )
    port map (
      clk_i     => clk_i,
      rstn_i    => rstn_int,
      bus_req_i => io_req,
      bus_rsp_o => rsp_bus(DEV_SDI),
      sdi_csn_i => sdi_csn_i,
      sdi_clk_i => sdi_clk_i,
      sdi_dat_i => sdi_dat_i,
      sdi_dat_o => sdi_dat_o,
      irq_o     => sdi_irq
    );
  end generate;

  neorv32_sdi_inst_false:
  if (IO_SDI_EN = false) generate
    rsp_bus(DEV_SDI) <= rsp_terminate_c;
    sdi_dat_o <= '0';
    sdi_irq   <= '0';
  end generate;


  -- General Purpose Input/Output Port (GPIO) -----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_gpio_inst_true:
  if (IO_GPIO_NUM > 0) generate
    neorv32_gpio_inst: neorv32_gpio
    generic map (
      GPIO_NUM => IO_GPIO_NUM
    )
    port map (
      -- host access --
      clk_i     => clk_i,
      rstn_i    => rstn_int,
      bus_req_i => io_req,
      bus_rsp_o => rsp_bus(DEV_GPIO),
      gpio_o    => gpio_o,
      gpio_i    => gpio_i
    );
  end generate;

  neorv32_gpio_inst_false:
  if (IO_GPIO_NUM = 0) generate
    rsp_bus(DEV_GPIO) <= rsp_terminate_c;
    gpio_o <= (others => '0');
  end generate;


  -- Watch Dog Timer (WDT) ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_wdt_inst_true:
  if (IO_WDT_EN = true) generate
    neorv32_wdt_inst: neorv32_wdt
    port map (
      clk_i       => clk_i,
      rstn_ext_i  => rstn_ext,
      rstn_int_i  => rstn_int,
      bus_req_i   => io_req,
      bus_rsp_o   => rsp_bus(DEV_WDT),
      cpu_debug_i => cpu_debug,
      cpu_sleep_i => cpu_sleep,
      clkgen_en_o => wdt_cg_en,
      clkgen_i    => clk_gen,
      irq_o       => wdt_irq,
      rstn_o      => rstn_wdt
    );
  end generate;

  neorv32_wdt_inst_false:
  if (IO_WDT_EN = false) generate
    rsp_bus(DEV_WDT) <= rsp_terminate_c;
    wdt_irq   <= '0';
    rstn_wdt  <= '1';
    wdt_cg_en <= '0';
  end generate;


  -- Machine System Timer (MTIME) -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_mtime_inst_true:
  if (IO_MTIME_EN = true) generate
    neorv32_mtime_inst: neorv32_mtime
    port map (
      clk_i     => clk_i,
      rstn_i    => rstn_int,
      bus_req_i => io_req,
      bus_rsp_o => rsp_bus(DEV_MTIME),
      irq_o     => mtime_irq
    );
  end generate;

  neorv32_mtime_inst_false:
  if (IO_MTIME_EN = false) generate
    rsp_bus(DEV_MTIME) <= rsp_terminate_c;
    mtime_irq <= mtime_irq_i;
  end generate;


  -- Primary Universal Asynchronous Receiver/Transmitter (UART0) ----------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_uart0_inst_true:
  if (IO_UART0_EN = true) generate
    neorv32_uart0_inst: neorv32_uart
    generic map (
      UART_PRIMARY => true,
      UART_RX_FIFO => IO_UART0_RX_FIFO,
      UART_TX_FIFO => IO_UART0_TX_FIFO
    )
    port map (
      clk_i       => clk_i,
      rstn_i      => rstn_int,
      bus_req_i   => io_req,
      bus_rsp_o   => rsp_bus(DEV_UART0),
      clkgen_en_o => uart0_cg_en,
      clkgen_i    => clk_gen,
      uart_txd_o  => uart0_txd_o,
      uart_rxd_i  => uart0_rxd_i,
      uart_rts_o  => uart0_rts_o,
      uart_cts_i  => uart0_cts_i,
      irq_rx_o    => uart0_rx_irq,
      irq_tx_o    => uart0_tx_irq
    );
  end generate;

  neorv32_uart0_inst_false:
  if (IO_UART0_EN = false) generate
    rsp_bus(DEV_UART0) <= rsp_terminate_c;
    uart0_txd_o  <= '0';
    uart0_rts_o  <= '1';
    uart0_cg_en  <= '0';
    uart0_rx_irq <= '0';
    uart0_tx_irq <= '0';
  end generate;


  -- Secondary Universal Asynchronous Receiver/Transmitter (UART1) --------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_uart1_inst_true:
  if (IO_UART1_EN = true) generate
    neorv32_uart1_inst: neorv32_uart
    generic map (
      UART_PRIMARY => false,
      UART_RX_FIFO => IO_UART1_RX_FIFO,
      UART_TX_FIFO => IO_UART1_TX_FIFO
    )
    port map (
      clk_i       => clk_i,
      rstn_i      => rstn_int,
      bus_req_i   => io_req,
      bus_rsp_o   => rsp_bus(DEV_UART1),
      clkgen_en_o => uart1_cg_en,
      clkgen_i    => clk_gen,
      uart_txd_o  => uart1_txd_o,
      uart_rxd_i  => uart1_rxd_i,
      uart_rts_o  => uart1_rts_o,
      uart_cts_i  => uart1_cts_i,
      irq_rx_o    => uart1_rx_irq,
      irq_tx_o    => uart1_tx_irq
    );
  end generate;

  neorv32_uart1_inst_false:
  if (IO_UART1_EN = false) generate
    rsp_bus(DEV_UART1) <= rsp_terminate_c;
    uart1_txd_o  <= '0';
    uart1_rts_o  <= '1';
    uart1_cg_en  <= '0';
    uart1_rx_irq <= '0';
    uart1_tx_irq <= '0';
  end generate;


  -- Serial Peripheral Interface (SPI) ------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_spi_inst_true:
  if (IO_SPI_EN = true) generate
    neorv32_spi_inst: neorv32_spi
    generic map (
      IO_SPI_FIFO => IO_SPI_FIFO
    )
    port map (
      clk_i       => clk_i,
      rstn_i      => rstn_int,
      bus_req_i   => io_req,
      bus_rsp_o   => rsp_bus(DEV_SPI),
      clkgen_en_o => spi_cg_en,
      clkgen_i    => clk_gen,
      spi_clk_o   => spi_clk_o,
      spi_dat_o   => spi_dat_o,
      spi_dat_i   => spi_dat_i,
      spi_csn_o   => spi_csn_o,
      irq_o       => spi_irq
    );
  end generate;

  neorv32_spi_inst_false:
  if (IO_SPI_EN = false) generate
    rsp_bus(DEV_SPI) <= rsp_terminate_c;
    spi_clk_o <= '0';
    spi_dat_o <= '0';
    spi_csn_o <= (others => '1');
    spi_cg_en <= '0';
    spi_irq   <= '0';
  end generate;


  -- Two-Wire Interface (TWI) ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_twi_inst_true:
  if (IO_TWI_EN = true) generate
    neorv32_twi_inst: neorv32_twi
    port map (
      clk_i       => clk_i,
      rstn_i      => rstn_int,
      bus_req_i   => io_req,
      bus_rsp_o   => rsp_bus(DEV_TWI),
      clkgen_en_o => twi_cg_en,
      clkgen_i    => clk_gen,
      twi_sda_i   => twi_sda_i,
      twi_sda_o   => twi_sda_o,
      twi_scl_i   => twi_scl_i,
      twi_scl_o   => twi_scl_o,
      irq_o       => twi_irq
    );
  end generate;

  neorv32_twi_inst_false:
  if (IO_TWI_EN = false) generate
    rsp_bus(DEV_TWI) <= rsp_terminate_c;
    twi_sda_o <= '1';
    twi_scl_o <= '1';
    twi_cg_en <= '0';
    twi_irq   <= '0';
  end generate;


  -- Pulse-Width Modulation Controller (PWM) ------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_pwm_inst_true:
  if (IO_PWM_NUM_CH > 0) generate
    neorv32_pwm_inst: neorv32_pwm
    generic map (
      NUM_CHANNELS => IO_PWM_NUM_CH
    )
    port map (
      clk_i       => clk_i,
      rstn_i      => rstn_int,
      bus_req_i   => io_req,
      bus_rsp_o   => rsp_bus(DEV_PWM),
      clkgen_en_o => pwm_cg_en,
      clkgen_i    => clk_gen,
      pwm_o       => pwm_o
    );
  end generate;

  neorv32_pwm_inst_false:
  if (IO_PWM_NUM_CH = 0) generate
    rsp_bus(DEV_PWM) <= rsp_terminate_c;
    pwm_cg_en <= '0';
    pwm_o     <= (others => '0');
  end generate;


  -- True Random Number Generator (TRNG) ----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_trng_inst_true:
  if (IO_TRNG_EN = true) generate
    neorv32_trng_inst: neorv32_trng
    generic map (
      IO_TRNG_FIFO => IO_TRNG_FIFO
    )
    port map (
      clk_i     => clk_i,
      rstn_i    => rstn_int,
      bus_req_i => io_req,
      bus_rsp_o => rsp_bus(DEV_TRNG),
      irq_o     => trng_irq
    );
  end generate;

  neorv32_trng_inst_false:
  if (IO_TRNG_EN = false) generate
    rsp_bus(DEV_TRNG) <= rsp_terminate_c;
    trng_irq          <= '0';
  end generate;


  -- Smart LED (WS2811/WS2812) Interface (NEOLED) -------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_neoled_inst_true:
  if (IO_NEOLED_EN = true) generate
    neorv32_neoled_inst: neorv32_neoled
    generic map (
      FIFO_DEPTH => IO_NEOLED_TX_FIFO
    )
    port map (
      clk_i       => clk_i,
      rstn_i      => rstn_int,
      bus_req_i   => io_req,
      bus_rsp_o   => rsp_bus(DEV_NEOLED),
      clkgen_en_o => neoled_cg_en,
      clkgen_i    => clk_gen,
      irq_o       => neoled_irq,
      neoled_o    => neoled_o
    );
  end generate;

  neorv32_neoled_inst_false:
  if (IO_NEOLED_EN = false) generate
    rsp_bus(DEV_NEOLED) <= rsp_terminate_c;
    neoled_cg_en <= '0';
    neoled_irq   <= '0';
    neoled_o     <= '0';
  end generate;


  -- External Interrupt Controller (XIRQ) ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_xirq_inst_true:
  if (XIRQ_NUM_CH > 0) generate
    neorv32_xirq_inst: neorv32_xirq
    generic map (
      XIRQ_NUM_CH           => XIRQ_NUM_CH,
      XIRQ_TRIGGER_TYPE     => XIRQ_TRIGGER_TYPE,
      XIRQ_TRIGGER_POLARITY => XIRQ_TRIGGER_POLARITY
    )
    port map (
      -- host access --
      clk_i     => clk_i,
      rstn_i    => rstn_int,
      bus_req_i => io_req,
      bus_rsp_o => rsp_bus(DEV_XIRQ),
      xirq_i    => xirq_i,
      cpu_irq_o => xirq_irq
    );
  end generate;

  neorv32_xirq_inst_false:
  if (XIRQ_NUM_CH = 0) generate
    rsp_bus(DEV_XIRQ) <= rsp_terminate_c;
    xirq_irq <= '0';
  end generate;


  -- General Purpose Timer (GPTMR) ----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_gptmr_inst_true:
  if (IO_GPTMR_EN = true) generate
    neorv32_gptmr_inst: neorv32_gptmr
    port map (
      clk_i       => clk_i,
      rstn_i      => rstn_int,
      bus_req_i   => io_req,
      bus_rsp_o   => rsp_bus(DEV_GPTMR),
      clkgen_en_o => gptmr_cg_en,
      clkgen_i    => clk_gen,
      irq_o       => gptmr_irq
    );
  end generate;

  neorv32_gptmr_inst_false:
  if (IO_GPTMR_EN = false) generate
    rsp_bus(DEV_GPTMR) <= rsp_terminate_c;
    gptmr_cg_en <= '0';
    gptmr_irq   <= '0';
  end generate;


  -- 1-Wire Interface Controller (ONEWIRE) --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_onewire_inst_true:
  if (IO_ONEWIRE_EN = true) generate
    neorv32_onewire_inst: neorv32_onewire
    port map (
      clk_i       => clk_i,
      rstn_i      => rstn_int,
      bus_req_i   => io_req,
      bus_rsp_o   => rsp_bus(DEV_ONEWIRE),
      clkgen_en_o => onewire_cg_en,
      clkgen_i    => clk_gen,
      onewire_i   => onewire_i,
      onewire_o   => onewire_o,
      irq_o       => onewire_irq
    );
  end generate;

  neorv32_onewire_inst_false:
  if (IO_ONEWIRE_EN = false) generate
    rsp_bus(DEV_ONEWIRE) <= rsp_terminate_c;
    onewire_o     <= '1';
    onewire_cg_en <= '0';
    onewire_irq   <= '0';
  end generate;


  -- System Configuration Information Memory (SYSINFO) --------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_sysinfo_inst: neorv32_sysinfo
  generic map (
    -- General --
    CLOCK_FREQUENCY      => CLOCK_FREQUENCY,
    CUSTOM_ID            => CUSTOM_ID,
    INT_BOOTLOADER_EN    => INT_BOOTLOADER_EN,
    -- Physical memory protection (PMP) --
    PMP_NUM_REGIONS      => PMP_NUM_REGIONS,
    -- internal Instruction memory --
    MEM_INT_IMEM_EN      => MEM_INT_IMEM_EN,
    MEM_INT_IMEM_SIZE    => MEM_INT_IMEM_SIZE,
    -- Internal Data memory --
    MEM_INT_DMEM_EN      => MEM_INT_DMEM_EN,
    MEM_INT_DMEM_SIZE    => MEM_INT_DMEM_SIZE,
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
    IO_GPIO_NUM          => IO_GPIO_NUM,
    IO_MTIME_EN          => IO_MTIME_EN,
    IO_UART0_EN          => IO_UART0_EN,
    IO_UART1_EN          => IO_UART1_EN,
    IO_SPI_EN            => IO_SPI_EN,
    IO_SDI_EN            => IO_SDI_EN,
    IO_TWI_EN            => IO_TWI_EN,
    IO_PWM_NUM_CH        => IO_PWM_NUM_CH,
    IO_WDT_EN            => IO_WDT_EN,
    IO_TRNG_EN           => IO_TRNG_EN,
    IO_CFS_EN            => IO_CFS_EN,
    IO_NEOLED_EN         => IO_NEOLED_EN,
    IO_XIRQ_NUM_CH       => XIRQ_NUM_CH,
    IO_GPTMR_EN          => IO_GPTMR_EN,
    IO_XIP_EN            => IO_XIP_EN,
    IO_ONEWIRE_EN        => IO_ONEWIRE_EN,
    IO_DMA_EN            => IO_DMA_EN
  )
  port map (
    clk_i     => clk_i,
    bus_req_i => io_req,
    bus_rsp_o => rsp_bus(DEV_SYSINFO)
  );


-- ****************************************************************************************************************************
-- On-Chip Debugger Complex
-- ****************************************************************************************************************************

  neorv32_neorv32_ocd_inst_true:
  if (ON_CHIP_DEBUGGER_EN = true) generate

    -- On-Chip Debugger - Debug Transport Module (DTM) ----------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_debug_dtm_inst: neorv32_debug_dtm
    generic map (
      IDCODE_VERSION => (others => '0'),
      IDCODE_PARTID  => (others => '0'),
      IDCODE_MANID   => (others => '0')
    )
    port map (
      -- global control --
      clk_i             => clk_i,
      rstn_i            => rstn_ext,
      -- jtag connection --
      jtag_trst_i       => jtag_trst_i,
      jtag_tck_i        => jtag_tck_i,
      jtag_tdi_i        => jtag_tdi_i,
      jtag_tdo_o        => jtag_tdo_o,
      jtag_tms_i        => jtag_tms_i,
      -- debug module interface (DMI) --
      dmi_req_valid_o   => dmi.req_valid,
      dmi_req_ready_i   => dmi.req_ready,
      dmi_req_address_o => dmi.req_address,
      dmi_req_data_o    => dmi.req_data,
      dmi_req_op_o      => dmi.req_op,
      dmi_rsp_valid_i   => dmi.rsp_valid,
      dmi_rsp_ready_o   => dmi.rsp_ready,
      dmi_rsp_data_i    => dmi.rsp_data,
      dmi_rsp_op_i      => dmi.rsp_op
    );

    -- On-Chip Debugger - Debug Module (DM) ---------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_debug_dm_inst: neorv32_debug_dm
    port map (
      -- global control --
      clk_i             => clk_i,
      rstn_i            => rstn_ext,
      cpu_debug_i       => cpu_debug,
      -- debug module interface (DMI) --
      dmi_req_valid_i   => dmi.req_valid,
      dmi_req_ready_o   => dmi.req_ready,
      dmi_req_address_i => dmi.req_address,
      dmi_req_data_i    => dmi.req_data,
      dmi_req_op_i      => dmi.req_op,
      dmi_rsp_valid_o   => dmi.rsp_valid,
      dmi_rsp_ready_i   => dmi.rsp_ready,
      dmi_rsp_data_o    => dmi.rsp_data,
      dmi_rsp_op_o      => dmi.rsp_op,
      -- CPU bus access --
      bus_req_i         => soc_req,
      bus_rsp_o         => rsp_bus(DEV_OCD),
      -- CPU control --
      cpu_ndmrstn_o     => dci_ndmrstn,
      cpu_halt_req_o    => dci_halt_req
    );

  end generate;

  neorv32_debug_ocd_inst_false:
  if (ON_CHIP_DEBUGGER_EN = false) generate
    rsp_bus(DEV_OCD) <= rsp_terminate_c;
    jtag_tdo_o   <= jtag_tdi_i; -- JTAG feed-through
    dci_ndmrstn  <= '1';
    dci_halt_req <= '0';
  end generate;


end neorv32_top_rtl;
