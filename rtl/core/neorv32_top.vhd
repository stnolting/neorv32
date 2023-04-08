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
    IO_SPI_FIFO                  : natural := 1;      -- SPI RTX fifo depth, has to be a power of two, min 1
    IO_SDI_EN                    : boolean := false;  -- implement serial data interface (SDI)?
    IO_SDI_FIFO                  : natural := 0;      -- SDI RTX fifo depth, has to be zero or a power of two
    IO_TWI_EN                    : boolean := false;  -- implement two-wire interface (TWI)?
    IO_PWM_NUM_CH                : natural := 0;      -- number of PWM channels to implement (0..12); 0 = disabled
    IO_WDT_EN                    : boolean := false;  -- implement watch dog timer (WDT)?
    IO_TRNG_EN                   : boolean := false;  -- implement true random number generator (TRNG)?
    IO_TRNG_FIFO                 : natural := 1;      -- TRNG fifo depth, has to be a power of two, min 1
    IO_CFS_EN                    : boolean := false;  -- implement custom functions subsystem (CFS)?
    IO_CFS_CONFIG                : std_ulogic_vector(31 downto 0) := x"00000000"; -- custom CFS configuration generic
    IO_CFS_IN_SIZE               : natural := 32;     -- size of CFS input conduit in bits
    IO_CFS_OUT_SIZE              : natural := 32;     -- size of CFS output conduit in bits
    IO_NEOLED_EN                 : boolean := false;  -- implement NeoPixel-compatible smart LED interface (NEOLED)?
    IO_NEOLED_TX_FIFO            : natural := 1;      -- NEOLED FIFO depth, has to be a power of two, min 1
    IO_GPTMR_EN                  : boolean := false;  -- implement general purpose timer (GPTMR)?
    IO_XIP_EN                    : boolean := false;  -- implement execute in place module (XIP)?
    IO_ONEWIRE_EN                : boolean := false   -- implement 1-wire interface (ONEWIRE)?
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
  signal rstn_ext_sreg : std_ulogic_vector(3 downto 0);
  signal rstn_int_sreg : std_ulogic_vector(3 downto 0);
  signal rstn_ext      : std_ulogic;
  signal rstn_int      : std_ulogic;
  signal rstn_wdt      : std_ulogic;

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
  type cpu_status_t is record
    debug : std_ulogic; -- set when in debug mode
    sleep : std_ulogic; -- set when in sleep mode
  end record;
  signal cpu_s : cpu_status_t;

  -- bus interface - instruction fetch --
  type bus_i_interface_t is record
    addr   : std_ulogic_vector(31 downto 0); -- bus access address
    rdata  : std_ulogic_vector(31 downto 0); -- bus read data
    re     : std_ulogic; -- read request
    ack    : std_ulogic; -- bus transfer acknowledge
    err    : std_ulogic; -- bus transfer error
    fence  : std_ulogic; -- fence.i instruction executed
    src    : std_ulogic; -- access source (1=instruction fetch, 0=data access)
    cached : std_ulogic; -- cached transfer
    priv   : std_ulogic; -- set when in privileged machine mode
  end record;
  signal cpu_i, i_cache : bus_i_interface_t;

  -- bus interface - data access --
  type bus_d_interface_t is record
    addr   : std_ulogic_vector(31 downto 0); -- bus access address
    rdata  : std_ulogic_vector(31 downto 0); -- bus read data
    wdata  : std_ulogic_vector(31 downto 0); -- bus write data
    ben    : std_ulogic_vector(03 downto 0); -- byte enable
    we     : std_ulogic; -- write request
    re     : std_ulogic; -- read request
    ack    : std_ulogic; -- bus transfer acknowledge
    err    : std_ulogic; -- bus transfer error
    fence  : std_ulogic; -- fence instruction executed
    src    : std_ulogic; -- access source (1=instruction fetch, 0=data access)
    cached : std_ulogic; -- cached transfer
    priv   : std_ulogic; -- set when in privileged machine mode
  end record;
  signal cpu_d, d_cache, p_bus : bus_d_interface_t;

  -- bus access error (from BUSKEEPER) --
  signal bus_error : std_ulogic;

  -- debug core interface (DCI) --
  signal dci_ndmrstn  : std_ulogic;
  signal dci_halt_req : std_ulogic;

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

  -- io space access --
  signal io_acc  : std_ulogic;
  signal io_rden : std_ulogic;
  signal io_wren : std_ulogic;

  -- module response bus - entry type --
  type resp_bus_entry_t is record
    rdata : std_ulogic_vector(31 downto 0);
    ack   : std_ulogic;
    err   : std_ulogic;
  end record;

  -- termination for unused/unimplemented bus endpoints --
  constant resp_bus_entry_terminate_c : resp_bus_entry_t := (rdata => (others => '0'), ack => '0', err => '0');

  -- module response bus - device ID --
  type resp_bus_id_t is (RESP_BUSKEEPER, RESP_IMEM, RESP_DMEM, RESP_BOOTROM, RESP_WISHBONE, RESP_GPIO,
                         RESP_MTIME, RESP_UART0, RESP_UART1, RESP_SPI, RESP_TWI, RESP_PWM, RESP_WDT,
                         RESP_TRNG, RESP_CFS, RESP_NEOLED, RESP_SYSINFO, RESP_OCD, RESP_XIRQ, RESP_GPTMR,
                         RESP_XIP_CT, RESP_XIP_ACC, RESP_ONEWIRE, RESP_SDI);

  -- module response bus --
  type resp_bus_t is array (resp_bus_id_t) of resp_bus_entry_t;
  signal resp_bus : resp_bus_t := (others => resp_bus_entry_terminate_c);

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

  -- misc --
  signal ext_timeout : std_ulogic;
  signal ext_access  : std_ulogic;
  signal xip_access  : std_ulogic;
  signal xip_enable  : std_ulogic;
  signal xip_page    : std_ulogic_vector(3 downto 0);

begin

  -- Processor IO/Peripherals Configuration -------------------------------------------------
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
    "- " &
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
    ""
    severity note;


  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
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
-- Clock and Reset System
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
-- CPU Core Complex
-- ****************************************************************************************************************************

  -- CPU Core -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_inst: neorv32_cpu
  generic map (
    -- General --
    HART_ID                      => HART_ID,                      -- hardware thread ID
    VENDOR_ID                    => VENDOR_ID,                    -- vendor's JEDEC ID
    CPU_BOOT_ADDR                => cpu_boot_addr_c,              -- cpu boot address
    CPU_DEBUG_PARK_ADDR          => dm_park_entry_c,              -- cpu debug mode parking loop entry address
    CPU_DEBUG_EXC_ADDR           => dm_exc_entry_c,               -- cpu debug mode exception entry address
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_B        => CPU_EXTENSION_RISCV_B,        -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_C        => CPU_EXTENSION_RISCV_C,        -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        => CPU_EXTENSION_RISCV_E,        -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        => CPU_EXTENSION_RISCV_M,        -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U        => CPU_EXTENSION_RISCV_U,        -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    => CPU_EXTENSION_RISCV_Zfinx,    -- implement 32-bit floating-point extension (using INT reg!)
    CPU_EXTENSION_RISCV_Zicntr   => CPU_EXTENSION_RISCV_Zicntr,   -- implement base counters?
    CPU_EXTENSION_RISCV_Zicond   => CPU_EXTENSION_RISCV_Zicond,   -- implement conditional operations extension?
    CPU_EXTENSION_RISCV_Zihpm    => CPU_EXTENSION_RISCV_Zihpm,    -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zifencei => CPU_EXTENSION_RISCV_Zifencei, -- implement instruction stream sync.?
    CPU_EXTENSION_RISCV_Zmmul    => CPU_EXTENSION_RISCV_Zmmul,    -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zxcfu    => CPU_EXTENSION_RISCV_Zxcfu,    -- implement custom (instr.) functions unit?
    CPU_EXTENSION_RISCV_Sdext    => ON_CHIP_DEBUGGER_EN,          -- implement external debug mode extension?
    CPU_EXTENSION_RISCV_Sdtrig   => ON_CHIP_DEBUGGER_EN,          -- implement debug mode trigger module extension?
    -- Extension Options --
    FAST_MUL_EN                  => FAST_MUL_EN,                  -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                => FAST_SHIFT_EN,                -- use barrel shifter for shift operations
    CPU_IPB_ENTRIES              => CPU_IPB_ENTRIES,              -- entries is instruction prefetch buffer, has to be a power of 1
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS              => PMP_NUM_REGIONS,              -- number of regions (0..16)
    PMP_MIN_GRANULARITY          => PMP_MIN_GRANULARITY,          -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 => HPM_NUM_CNTS,                 -- number of implemented HPM counters (0..29)
    HPM_CNT_WIDTH                => HPM_CNT_WIDTH                 -- total size of HPM counters (0..64)
  )
  port map (
    -- global control --
    clk_i         => clk_i,       -- global clock, rising edge
    rstn_i        => rstn_int,    -- global reset, low-active, async
    sleep_o       => cpu_s.sleep, -- cpu is in sleep mode when set
    debug_o       => cpu_s.debug, -- cpu is in debug mode when set
    -- instruction bus interface --
    i_bus_addr_o  => cpu_i.addr,  -- bus access address
    i_bus_rdata_i => cpu_i.rdata, -- bus read data
    i_bus_re_o    => cpu_i.re,    -- read request
    i_bus_ack_i   => cpu_i.ack,   -- bus transfer acknowledge
    i_bus_err_i   => cpu_i.err,   -- bus transfer error
    i_bus_fence_o => cpu_i.fence, -- executed FENCEI operation
    i_bus_priv_o  => cpu_i.priv,  -- current effective privilege level
    -- data bus interface --
    d_bus_addr_o  => cpu_d.addr,  -- bus access address
    d_bus_rdata_i => cpu_d.rdata, -- bus read data
    d_bus_wdata_o => cpu_d.wdata, -- bus write data
    d_bus_ben_o   => cpu_d.ben,   -- byte enable
    d_bus_we_o    => cpu_d.we,    -- write request
    d_bus_re_o    => cpu_d.re,    -- read request
    d_bus_ack_i   => cpu_d.ack,   -- bus transfer acknowledge
    d_bus_err_i   => cpu_d.err,   -- bus transfer error
    d_bus_fence_o => cpu_d.fence, -- executed FENCE operation
    d_bus_priv_o  => cpu_d.priv,  -- current effective privilege level
    -- non-maskable interrupt --
    msw_irq_i     => msw_irq_i,   -- machine software interrupt
    mext_irq_i    => mext_irq_i,  -- machine external interrupt request
    mtime_irq_i   => mtime_irq,   -- machine timer interrupt
    -- fast interrupts (custom) --
    firq_i        => fast_irq,    -- fast interrupt trigger
    -- debug mode (halt) request --
    db_halt_req_i => dci_halt_req
  );

  -- initialized but unused --
  cpu_i.src    <= '1';
  cpu_d.src    <= '0';
  cpu_i.cached <= '0';
  cpu_d.cached <= '0';

  -- advanced memory control --
  fence_o  <= cpu_d.fence; -- indicates an executed FENCE operation
  fencei_o <= cpu_i.fence; -- indicates an executed FENCE.I operation

  -- fast interrupt requests (FIRQs) - triggers are SINGLE-SHOT --
  fast_irq(00) <= wdt_irq;      -- HIGHEST PRIORITY - watchdog
  fast_irq(01) <= cfs_irq;      -- custom functions subsystem
  fast_irq(02) <= uart0_rx_irq; -- primary UART (UART0) RX
  fast_irq(03) <= uart0_tx_irq; -- primary UART (UART0) TX
  fast_irq(04) <= uart1_rx_irq; -- secondary UART (UART1) RX
  fast_irq(05) <= uart1_tx_irq; -- secondary UART (UART1) TX
  fast_irq(06) <= spi_irq;      -- SPI interrupt
  fast_irq(07) <= twi_irq;      -- TWI transfer done
  fast_irq(08) <= xirq_irq;     -- external interrupt controller
  fast_irq(09) <= neoled_irq;   -- NEOLED buffer IRQ
  fast_irq(10) <= '0';          -- reserved
  fast_irq(11) <= sdi_irq;      -- SDI interrupt
  fast_irq(12) <= gptmr_irq;    -- general purpose timer match
  fast_irq(13) <= onewire_irq;  -- ONEWIRE operation done
  fast_irq(14) <= '0';          -- reserved
  fast_irq(15) <= '0';          -- LOWEST PRIORITY - reserved


  -- CPU Instruction Cache ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_icache_inst_true:
  if (ICACHE_EN = true) generate
    neorv32_icache_inst: neorv32_icache
    generic map (
      ICACHE_NUM_BLOCKS => ICACHE_NUM_BLOCKS,   -- number of blocks (min 2), has to be a power of 2
      ICACHE_BLOCK_SIZE => ICACHE_BLOCK_SIZE,   -- block size in bytes (min 4), has to be a power of 2
      ICACHE_NUM_SETS   => ICACHE_ASSOCIATIVITY -- associativity / number of sets (1=direct_mapped), has to be a power of 2
    )
    port map (
      -- global control --
      clk_i        => clk_i,          -- global clock, rising edge
      rstn_i       => rstn_int,       -- global reset, low-active, async
      clear_i      => cpu_i.fence,    -- cache clear
      -- host controller interface --
      host_addr_i  => cpu_i.addr,     -- bus access address
      host_rdata_o => cpu_i.rdata,    -- bus read data
      host_re_i    => cpu_i.re,       -- read enable
      host_ack_o   => cpu_i.ack,      -- bus transfer acknowledge
      host_err_o   => cpu_i.err,      -- bus transfer error
      -- peripheral bus interface --
      bus_cached_o => i_cache.cached, -- set if cached (!) access in progress
      bus_addr_o   => i_cache.addr,   -- bus access address
      bus_rdata_i  => i_cache.rdata,  -- bus read data
      bus_re_o     => i_cache.re,     -- read enable
      bus_ack_i    => i_cache.ack,    -- bus transfer acknowledge
      bus_err_i    => i_cache.err     -- bus transfer error
    );
  end generate;

  neorv32_icache_inst_false:
  if (ICACHE_EN = false) generate
    i_cache.cached <= '0'; -- single transfer (uncached)
    i_cache.addr   <= cpu_i.addr;
    cpu_i.rdata    <= i_cache.rdata;
    i_cache.re     <= cpu_i.re;
    cpu_i.ack      <= i_cache.ack;
    cpu_i.err      <= i_cache.err;
  end generate;

  i_cache.priv  <= cpu_i.priv;
  i_cache.fence <= '0'; -- not used
  i_cache.src   <= '0'; -- not used


  -- CPU Data Cache -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_dcache_inst_true:
  if (DCACHE_EN = true) generate
    neorv32_dcache_inst: neorv32_dcache
    generic map (
      DCACHE_NUM_BLOCKS => DCACHE_NUM_BLOCKS, -- number of blocks (min 1), has to be a power of 2
      DCACHE_BLOCK_SIZE => DCACHE_BLOCK_SIZE, -- block size in bytes (min 4), has to be a power of 2
      DCACHE_UC_PBEGIN  => "1111"             -- begin of uncached address space (page number)
    )
    port map (
      -- global control --
      clk_i        => clk_i,          -- global clock, rising edge
      rstn_i       => rstn_int,       -- global reset, low-active, async
      clear_i      => cpu_d.fence,    -- cache clear
      -- host controller interface --
      host_addr_i  => cpu_d.addr,     -- bus access address
      host_rdata_o => cpu_d.rdata,    -- bus read data
      host_wdata_i => cpu_d.wdata,    -- bus write data
      host_ben_i   => cpu_d.ben,      -- byte enable
      host_we_i    => cpu_d.we,       -- write enable
      host_re_i    => cpu_d.re,       -- read enable
      host_ack_o   => cpu_d.ack,      -- bus transfer acknowledge
      host_err_o   => cpu_d.err,      -- bus transfer error
      -- peripheral bus interface --
      bus_cached_o => d_cache.cached, -- set if cached (!) access in progress
      bus_addr_o   => d_cache.addr,   -- bus access address
      bus_rdata_i  => d_cache.rdata,  -- bus read data
      bus_wdata_o  => d_cache.wdata,  -- bus write data
      bus_ben_o    => d_cache.ben,    -- byte enable
      bus_we_o     => d_cache.we,     -- write enable
      bus_re_o     => d_cache.re,     -- read enable
      bus_ack_i    => d_cache.ack,    -- bus transfer acknowledge
      bus_err_i    => d_cache.err     -- bus transfer error
    );
  end generate;

  neorv32_dcache_inst_false:
  if (DCACHE_EN = false) generate
    d_cache.cached <= '0'; -- single transfer (uncached)
    d_cache.addr   <= cpu_d.addr;
    cpu_d.rdata    <= d_cache.rdata;
    d_cache.wdata  <= cpu_d.wdata;
    d_cache.ben    <= cpu_d.ben;
    d_cache.we     <= cpu_d.we;
    d_cache.re     <= cpu_d.re;
    cpu_d.ack      <= d_cache.ack;
    cpu_d.err      <= d_cache.err;
  end generate;

  d_cache.priv  <= cpu_d.priv;
  d_cache.fence <= '0'; -- not used
  d_cache.src   <= '0'; -- not used


  -- CPU Bus Switch -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_busswitch_inst: neorv32_busswitch
  generic map (
    PORT_CA_READ_ONLY => false, -- set if controller port A is read-only
    PORT_CB_READ_ONLY => true   -- set if controller port B is read-only
  )
  port map (
    -- global control --
    clk_i           => clk_i,          -- global clock, rising edge
    rstn_i          => rstn_int,       -- global reset, low-active, async
    -- controller interface a --
    ca_bus_priv_i   => d_cache.priv,   -- current privilege level
    ca_bus_cached_i => d_cache.cached, -- set if cached transfer
    ca_bus_addr_i   => d_cache.addr,   -- bus access address
    ca_bus_rdata_o  => d_cache.rdata,  -- bus read data
    ca_bus_wdata_i  => d_cache.wdata,  -- bus write data
    ca_bus_ben_i    => d_cache.ben,    -- byte enable
    ca_bus_we_i     => d_cache.we,     -- write enable
    ca_bus_re_i     => d_cache.re,     -- read enable
    ca_bus_ack_o    => d_cache.ack,    -- bus transfer acknowledge
    ca_bus_err_o    => d_cache.err,    -- bus transfer error
    -- controller interface b --
    cb_bus_priv_i   => i_cache.priv,   -- current privilege level
    cb_bus_cached_i => i_cache.cached, -- set if cached transfer
    cb_bus_addr_i   => i_cache.addr,   -- bus access address
    cb_bus_rdata_o  => i_cache.rdata,  -- bus read data
    cb_bus_wdata_i  => (others => '0'),
    cb_bus_ben_i    => (others => '0'),
    cb_bus_we_i     => '0',
    cb_bus_re_i     => i_cache.re,     -- read enable
    cb_bus_ack_o    => i_cache.ack,    -- bus transfer acknowledge
    cb_bus_err_o    => i_cache.err,    -- bus transfer error
    -- peripheral bus --
    p_bus_priv_o    => p_bus.priv,     -- current privilege level
    p_bus_cached_o  => p_bus.cached,   -- set if cached transfer
    p_bus_src_o     => p_bus.src,      -- access source: 0 = A (data), 1 = B (instructions)
    p_bus_addr_o    => p_bus.addr,     -- bus access address
    p_bus_rdata_i   => p_bus.rdata,    -- bus read data
    p_bus_wdata_o   => p_bus.wdata,    -- bus write data
    p_bus_ben_o     => p_bus.ben,      -- byte enable
    p_bus_we_o      => p_bus.we,       -- write enable
    p_bus_re_o      => p_bus.re,       -- read enable
    p_bus_ack_i     => p_bus.ack,      -- bus transfer acknowledge
    p_bus_err_i     => bus_error       -- bus transfer error
  );

  -- any fence operation? --
  p_bus.fence <= cpu_i.fence or cpu_d.fence;

  -- bus response --
  bus_response: process(resp_bus)
    variable rdata_v : std_ulogic_vector(31 downto 0);
    variable ack_v   : std_ulogic;
    variable err_v   : std_ulogic;
  begin
    -- OR all response signals: only the module that has actually
    -- been accessed is allowed to *set* its bus output signals
    rdata_v := (others => '0');
    ack_v   := '0';
    err_v   := '0';
    for i in resp_bus'range loop
      rdata_v := rdata_v or resp_bus(i).rdata; -- read data
      ack_v   := ack_v   or resp_bus(i).ack;   -- acknowledge
      err_v   := err_v   or resp_bus(i).err;   -- error
    end loop; -- i
    p_bus.rdata <= rdata_v;
    p_bus.ack   <= ack_v;
    p_bus.err   <= err_v;
  end process;


  -- Bus Keeper (BUSKEEPER) -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_bus_keeper_inst: neorv32_bus_keeper
  port map (
    -- host access --
    clk_i      => clk_i,                          -- global clock line
    rstn_i     => rstn_int,                       -- global reset line, low-active, use as async
    addr_i     => p_bus.addr,                     -- address
    rden_i     => io_rden,                        -- read enable
    wren_i     => io_wren,                        -- byte write enable
    data_i     => p_bus.wdata,                    -- data in
    data_o     => resp_bus(RESP_BUSKEEPER).rdata, -- data out
    ack_o      => resp_bus(RESP_BUSKEEPER).ack,   -- transfer acknowledge
    err_o      => bus_error,                      -- transfer error
    -- bus monitoring --
    bus_addr_i => p_bus.addr,                     -- address
    bus_rden_i => p_bus.re,                       -- read enable
    bus_wren_i => p_bus.we,                       -- write enable
    bus_ack_i  => p_bus.ack,                      -- transfer acknowledge from bus system
    bus_err_i  => p_bus.err,                      -- transfer error from bus system
    bus_tmo_i  => ext_timeout,                    -- transfer timeout (external interface)
    bus_ext_i  => ext_access,                     -- external bus access
    bus_xip_i  => xip_access                      -- pending XIP access
  );

  -- unused, BUSKEEPER issues error to **directly** the CPU --
  resp_bus(RESP_BUSKEEPER).err <= '0';


-- ****************************************************************************************************************************
-- Memory System
-- ****************************************************************************************************************************

  -- Processor-Internal Instruction Memory (IMEM) -------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_int_imem_inst_true:
  if (MEM_INT_IMEM_EN = true) and (MEM_INT_IMEM_SIZE > 0) generate
    neorv32_int_imem_inst: neorv32_imem
    generic map (
      IMEM_BASE    => imem_base_c,          -- memory base address
      IMEM_SIZE    => MEM_INT_IMEM_SIZE,    -- processor-internal instruction memory size in bytes
      IMEM_AS_IROM => not INT_BOOTLOADER_EN -- implement IMEM as pre-initialized read-only memory?
    )
    port map (
      clk_i  => clk_i,                     -- global clock line
      rden_i => p_bus.re,                  -- read enable
      wren_i => p_bus.we,                  -- write enable
      ben_i  => p_bus.ben,                 -- byte write enable
      addr_i => p_bus.addr,                -- address
      data_i => p_bus.wdata,               -- data in
      data_o => resp_bus(RESP_IMEM).rdata, -- data out
      ack_o  => resp_bus(RESP_IMEM).ack,   -- transfer acknowledge
      err_o  => resp_bus(RESP_IMEM).err    -- transfer error
    );
  end generate;

  neorv32_int_imem_inst_false:
  if (MEM_INT_IMEM_EN = false) or (MEM_INT_IMEM_SIZE = 0) generate
    resp_bus(RESP_IMEM) <= resp_bus_entry_terminate_c;
  end generate;


  -- Processor-Internal Data Memory (DMEM) --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_int_dmem_inst_true:
  if (MEM_INT_DMEM_EN = true) and (MEM_INT_DMEM_SIZE > 0) generate
    neorv32_int_dmem_inst: neorv32_dmem
    generic map (
      DMEM_BASE => dmem_base_c,      -- memory base address
      DMEM_SIZE => MEM_INT_DMEM_SIZE -- processor-internal data memory size in bytes
    )
    port map (
      clk_i  => clk_i,                     -- global clock line
      rden_i => p_bus.re,                  -- read enable
      wren_i => p_bus.we,                  -- write enable
      ben_i  => p_bus.ben,                 -- byte write enable
      addr_i => p_bus.addr,                -- address
      data_i => p_bus.wdata,               -- data in
      data_o => resp_bus(RESP_DMEM).rdata, -- data out
      ack_o  => resp_bus(RESP_DMEM).ack    -- transfer acknowledge
    );
    resp_bus(RESP_DMEM).err <= '0'; -- no access error possible
  end generate;

  neorv32_int_dmem_inst_false:
  if (MEM_INT_DMEM_EN = false) or (MEM_INT_DMEM_SIZE = 0) generate
    resp_bus(RESP_DMEM) <= resp_bus_entry_terminate_c;
  end generate;


  -- Processor-Internal Bootloader ROM (BOOTROM) --------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_boot_rom_inst_true:
  if (INT_BOOTLOADER_EN = true) generate
    neorv32_boot_rom_inst: neorv32_boot_rom
    generic map (
      BOOTROM_BASE => boot_rom_base_c -- boot ROM base address
    )
    port map (
      clk_i  => clk_i,                        -- global clock line
      rden_i => p_bus.re,                     -- read enable
      wren_i => p_bus.we,                     -- write enable
      addr_i => p_bus.addr,                   -- address
      data_o => resp_bus(RESP_BOOTROM).rdata, -- data out
      ack_o  => resp_bus(RESP_BOOTROM).ack,   -- transfer acknowledge
      err_o  => resp_bus(RESP_BOOTROM).err    -- transfer error
    );
  end generate;

  neorv32_boot_rom_inst_false:
  if (INT_BOOTLOADER_EN = false) generate
    resp_bus(RESP_BOOTROM) <= resp_bus_entry_terminate_c;
  end generate;


  -- External Wishbone Gateway (WISHBONE) ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_wishbone_inst_true:
  if (MEM_EXT_EN = true) generate
    neorv32_wishbone_inst: neorv32_wishbone
    generic map (
      -- Internal instruction memory --
      MEM_INT_IMEM_EN   => MEM_INT_IMEM_EN,    -- implement processor-internal instruction memory
      MEM_INT_IMEM_SIZE => MEM_INT_IMEM_SIZE,  -- size of processor-internal instruction memory in bytes
      -- Internal data memory --
      MEM_INT_DMEM_EN   => MEM_INT_DMEM_EN,    -- implement processor-internal data memory
      MEM_INT_DMEM_SIZE => MEM_INT_DMEM_SIZE,  -- size of processor-internal data memory in bytes
      -- Interface Configuration --
      BUS_TIMEOUT       => MEM_EXT_TIMEOUT,    -- cycles after an UNACKNOWLEDGED bus access triggers a bus fault exception
      PIPE_MODE         => MEM_EXT_PIPE_MODE,  -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
      BIG_ENDIAN        => MEM_EXT_BIG_ENDIAN, -- byte order: true=big-endian, false=little-endian
      ASYNC_RX          => MEM_EXT_ASYNC_RX,   -- use register buffer for RX data when false
      ASYNC_TX          => MEM_EXT_ASYNC_TX    -- use register buffer for TX data when false
    )
    port map (
      -- global control --
      clk_i      => clk_i,                         -- global clock line
      rstn_i     => rstn_int,                      -- global reset line, low-active, async
      -- host access --
      src_i      => p_bus.src,                     -- access type (0: data, 1:instruction)
      addr_i     => p_bus.addr,                    -- address
      rden_i     => p_bus.re,                      -- read enable
      wren_i     => p_bus.we,                      -- write enable
      ben_i      => p_bus.ben,                     -- byte write enable
      data_i     => p_bus.wdata,                   -- data in
      data_o     => resp_bus(RESP_WISHBONE).rdata, -- data out
      ack_o      => resp_bus(RESP_WISHBONE).ack,   -- transfer acknowledge
      err_o      => resp_bus(RESP_WISHBONE).err,   -- transfer error
      tmo_o      => ext_timeout,                   -- transfer timeout
      priv_i     => p_bus.priv,                    -- current CPU privilege level
      ext_o      => ext_access,                    -- active external access
      -- xip configuration --
      xip_en_i   => xip_enable,                    -- XIP module enabled
      xip_page_i => xip_page,                      -- XIP memory page
      -- wishbone interface --
      wb_tag_o   => wb_tag_o,                      -- request tag
      wb_adr_o   => wb_adr_o,                      -- address
      wb_dat_i   => wb_dat_i,                      -- read data
      wb_dat_o   => wb_dat_o,                      -- write data
      wb_we_o    => wb_we_o,                       -- read/write
      wb_sel_o   => wb_sel_o,                      -- byte enable
      wb_stb_o   => wb_stb_o,                      -- strobe
      wb_cyc_o   => wb_cyc_o,                      -- valid cycle
      wb_ack_i   => wb_ack_i,                      -- transfer acknowledge
      wb_err_i   => wb_err_i                       -- transfer error
    );
  end generate;

  neorv32_wishbone_inst_false:
  if (MEM_EXT_EN = false) generate
    resp_bus(RESP_WISHBONE) <= resp_bus_entry_terminate_c;
    ext_timeout <= '0';
    ext_access  <= '0';
    --
    wb_adr_o <= (others => '0');
    wb_dat_o <= (others => '0');
    wb_we_o  <= '0';
    wb_sel_o <= (others => '0');
    wb_stb_o <= '0';
    wb_cyc_o <= '0';
    wb_tag_o <= (others => '0');
  end generate;


  -- Execute In Place Module (XIP) ----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_xip_inst_true:
  if (IO_XIP_EN = true) generate
    neorv32_xip_inst: neorv32_xip
    port map (
      -- global control --
      clk_i       => clk_i,                        -- global clock line
      rstn_i      => rstn_int,                     -- global reset line, low-active, async
      -- host access: control register access port --
      ct_addr_i   => p_bus.addr,                   -- address
      ct_rden_i   => io_rden,                      -- read enable
      ct_wren_i   => io_wren,                      -- write enable
      ct_data_i   => p_bus.wdata,                  -- data in
      ct_data_o   => resp_bus(RESP_XIP_CT).rdata,  -- data out
      ct_ack_o    => resp_bus(RESP_XIP_CT).ack,    -- transfer acknowledge
      -- host access: transparent SPI access port (read-only) --
      acc_addr_i  => p_bus.addr,                   -- address
      acc_rden_i  => p_bus.re,                     -- read enable
      acc_wren_i  => p_bus.we,                     -- write enable
      acc_data_o  => resp_bus(RESP_XIP_ACC).rdata, -- data out
      acc_ack_o   => resp_bus(RESP_XIP_ACC).ack,   -- transfer acknowledge
      acc_err_o   => resp_bus(RESP_XIP_ACC).err,   -- transfer error
      -- status --
      xip_en_o    => xip_enable,                   -- XIP enable
      xip_acc_o   => xip_access,                   -- pending XIP access
      xip_page_o  => xip_page,                     -- XIP page
      -- clock generator --
      clkgen_en_o => xip_cg_en,                    -- enable clock generator
      clkgen_i    => clk_gen,
      -- SPI device interface --
      spi_csn_o   => xip_csn_o,                    -- chip-select, low-active
      spi_clk_o   => xip_clk_o,                    -- serial clock
      spi_dat_i   => xip_dat_i,                    -- device data output
      spi_dat_o   => xip_dat_o                     -- controller data output
    );
    resp_bus(RESP_XIP_CT).err <= '0'; -- no access error possible
  end generate;

  neorv32_xip_inst_false:
  if (IO_XIP_EN = false) generate
    resp_bus(RESP_XIP_CT)  <= resp_bus_entry_terminate_c;
    resp_bus(RESP_XIP_ACC) <= resp_bus_entry_terminate_c;
    --
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

  -- IO Access? -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  io_acc  <= '1' when (p_bus.addr(31 downto index_size_f(io_size_c)) = io_base_c(31 downto index_size_f(io_size_c))) else '0';
  io_rden <= '1' when (io_acc = '1') and (p_bus.re = '1') and (p_bus.src = '0')    else '0'; -- PMA: read access only from data interface
  io_wren <= '1' when (io_acc = '1') and (p_bus.we = '1') and (p_bus.ben = "1111") else '0'; -- PMA: full-word write accesses only (reduces HW complexity)


  -- Custom Functions Subsystem (CFS) -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cfs_inst_true:
  if (IO_CFS_EN = true) generate
    neorv32_cfs_inst: neorv32_cfs
    generic map (
      CFS_CONFIG   => IO_CFS_CONFIG,  -- custom CFS configuration generic
      CFS_IN_SIZE  => IO_CFS_IN_SIZE, -- size of CFS input conduit in bits
      CFS_OUT_SIZE => IO_CFS_OUT_SIZE -- size of CFS output conduit in bits
    )
    port map (
      -- host access --
      clk_i       => clk_i,                    -- global clock line
      rstn_i      => rstn_int,                 -- global reset line, low-active, use as async
      priv_i      => p_bus.priv,               -- current CPU privilege mode
      addr_i      => p_bus.addr,               -- address
      rden_i      => io_rden,                  -- read enable
      wren_i      => io_wren,                  -- word write enable
      data_i      => p_bus.wdata,              -- data in
      data_o      => resp_bus(RESP_CFS).rdata, -- data out
      ack_o       => resp_bus(RESP_CFS).ack,   -- transfer acknowledge
      err_o       => resp_bus(RESP_CFS).err,   -- access error
      -- clock generator --
      clkgen_en_o => cfs_cg_en,                -- enable clock generator
      clkgen_i    => clk_gen,                  -- "clock" inputs
      -- interrupt --
      irq_o       => cfs_irq,                  -- interrupt request
      -- custom io (conduit) --
      cfs_in_i    => cfs_in_i,                 -- custom inputs
      cfs_out_o   => cfs_out_o                 -- custom outputs
    );
  end generate;

  neorv32_cfs_inst_false:
  if (IO_CFS_EN = false) generate
    resp_bus(RESP_CFS) <= resp_bus_entry_terminate_c;
    --
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
      RTX_FIFO => IO_SDI_FIFO -- RTX fifo depth, has to be a power of two, min 1
    )
    port map (
      -- host access --
      clk_i     => clk_i,                    -- global clock line
      rstn_i    => rstn_int,                 -- global reset line, low-active, async
      addr_i    => p_bus.addr,               -- address
      rden_i    => io_rden,                  -- read enable
      wren_i    => io_wren,                  -- write enable
      data_i    => p_bus.wdata,              -- data in
      data_o    => resp_bus(RESP_SDI).rdata, -- data out
      ack_o     => resp_bus(RESP_SDI).ack,   -- transfer acknowledge
      -- SDI receiver input --
      sdi_csn_i => sdi_csn_i,                -- low-active chip-select
      sdi_clk_i => sdi_clk_i,                -- serial clock
      sdi_dat_i => sdi_dat_i,                -- serial data input
      sdi_dat_o => sdi_dat_o,                -- serial data output
      -- interrupts --
      irq_o     => sdi_irq
    );
    resp_bus(RESP_SDI).err <= '0'; -- no access error possible
  end generate;

  neorv32_sdi_inst_false:
  if (IO_SDI_EN = false) generate
    resp_bus(RESP_SDI) <= resp_bus_entry_terminate_c;
    --
    sdi_dat_o <= '0';
    sdi_irq   <= '0';
  end generate;


  -- General Purpose Input/Output Port (GPIO) -----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_gpio_inst_true:
  if (IO_GPIO_NUM > 0) generate
    neorv32_gpio_inst: neorv32_gpio
    generic map (
      GPIO_NUM => IO_GPIO_NUM -- number of GPIO input/output pairs (0..64)
    )
    port map (
      -- host access --
      clk_i  => clk_i,                     -- global clock line
      rstn_i => rstn_int,                  -- global reset line, low-active, async
      addr_i => p_bus.addr,                -- address
      rden_i => io_rden,                   -- read enable
      wren_i => io_wren,                   -- write enable
      data_i => p_bus.wdata,               -- data in
      data_o => resp_bus(RESP_GPIO).rdata, -- data out
      ack_o  => resp_bus(RESP_GPIO).ack,   -- transfer acknowledge
      -- parallel io --
      gpio_o => gpio_o,
      gpio_i => gpio_i
    );
    resp_bus(RESP_GPIO).err <= '0'; -- no access error possible
  end generate;

  neorv32_gpio_inst_false:
  if (IO_GPIO_NUM = 0) generate
    resp_bus(RESP_GPIO) <= resp_bus_entry_terminate_c;
    --
    gpio_o <= (others => '0');
  end generate;


  -- Watch Dog Timer (WDT) ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_wdt_inst_true:
  if (IO_WDT_EN = true) generate
    neorv32_wdt_inst: neorv32_wdt
    port map (
      -- host access --
      clk_i       => clk_i,                    -- global clock line
      rstn_ext_i  => rstn_ext,                 -- external reset line, low-active, async
      rstn_int_i  => rstn_int,                 -- internal reset line, low-active, async
      rden_i      => io_rden,                  -- read enable
      wren_i      => io_wren,                  -- write enable
      addr_i      => p_bus.addr,               -- address
      data_i      => p_bus.wdata,              -- data in
      data_o      => resp_bus(RESP_WDT).rdata, -- data out
      ack_o       => resp_bus(RESP_WDT).ack,   -- transfer acknowledge
      -- CPU status --
      cpu_debug_i => cpu_s.debug,              -- CPU is in debug mode
      cpu_sleep_i => cpu_s.sleep,              -- CPU is in sleep mode
      -- clock generator --
      clkgen_en_o => wdt_cg_en,                -- enable clock generator
      clkgen_i    => clk_gen,
      -- timeout event --
      irq_o       => wdt_irq,                  -- timeout IRQ
      rstn_o      => rstn_wdt                  -- timeout reset, low_active, sync
    );
    resp_bus(RESP_WDT).err <= '0'; -- no access error possible
  end generate;

  neorv32_wdt_inst_false:
  if (IO_WDT_EN = false) generate
    resp_bus(RESP_WDT) <= resp_bus_entry_terminate_c;
    --
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
      -- host access --
      clk_i  => clk_i,                      -- global clock line
      rstn_i => rstn_int,                   -- global reset line, low-active, async
      addr_i => p_bus.addr,                 -- address
      rden_i => io_rden,                    -- read enable
      wren_i => io_wren,                    -- write enable
      data_i => p_bus.wdata,                -- data in
      data_o => resp_bus(RESP_MTIME).rdata, -- data out
      ack_o  => resp_bus(RESP_MTIME).ack,   -- transfer acknowledge
      -- interrupt --
      irq_o  => mtime_irq                   -- interrupt request
    );
    resp_bus(RESP_MTIME).err <= '0'; -- no access error possible
  end generate;

  neorv32_mtime_inst_false:
  if (IO_MTIME_EN = false) generate
    resp_bus(RESP_MTIME) <= resp_bus_entry_terminate_c;
    --
    mtime_irq <= mtime_irq_i; -- use external machine timer interrupt
  end generate;


  -- Primary Universal Asynchronous Receiver/Transmitter (UART0) ----------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_uart0_inst_true:
  if (IO_UART0_EN = true) generate
    neorv32_uart0_inst: neorv32_uart
    generic map (
      UART_PRIMARY => true,             -- true = primary UART (UART0), false = secondary UART (UART1)
      UART_RX_FIFO => IO_UART0_RX_FIFO, -- RX fifo depth, has to be a power of two, min 1
      UART_TX_FIFO => IO_UART0_TX_FIFO  -- TX fifo depth, has to be a power of two, min 1
    )
    port map (
      -- host access --
      clk_i       => clk_i,                      -- global clock line
      rstn_i      => rstn_int,                   -- global reset line, low-active, async
      addr_i      => p_bus.addr,                 -- address
      rden_i      => io_rden,                    -- read enable
      wren_i      => io_wren,                    -- write enable
      data_i      => p_bus.wdata,                -- data in
      data_o      => resp_bus(RESP_UART0).rdata, -- data out
      ack_o       => resp_bus(RESP_UART0).ack,   -- transfer acknowledge
      -- clock generator --
      clkgen_en_o => uart0_cg_en,                -- enable clock generator
      clkgen_i    => clk_gen,
      -- com lines --
      uart_txd_o  => uart0_txd_o,
      uart_rxd_i  => uart0_rxd_i,
      -- hardware flow control --
      uart_rts_o  => uart0_rts_o,                -- UART.RX ready to receive ("RTR"), low-active, optional
      uart_cts_i  => uart0_cts_i,                -- UART.TX allowed to transmit, low-active, optional
      -- interrupts --
      irq_rx_o    => uart0_rx_irq,               -- rx interrupt
      irq_tx_o    => uart0_tx_irq                -- tx interrupt
    );
    resp_bus(RESP_UART0).err <= '0'; -- no access error possible
  end generate;

  neorv32_uart0_inst_false:
  if (IO_UART0_EN = false) generate
    resp_bus(RESP_UART0) <= resp_bus_entry_terminate_c;
    --
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
      UART_PRIMARY => false,            -- true = primary UART (UART0), false = secondary UART (UART1)
      UART_RX_FIFO => IO_UART1_RX_FIFO, -- RX fifo depth, has to be a power of two, min 1
      UART_TX_FIFO => IO_UART1_TX_FIFO  -- TX fifo depth, has to be a power of two, min 1
    )
    port map (
      -- host access --
      clk_i       => clk_i,                      -- global clock line
      rstn_i      => rstn_int,                   -- global reset line, low-active, async
      addr_i      => p_bus.addr,                 -- address
      rden_i      => io_rden,                    -- read enable
      wren_i      => io_wren,                    -- write enable
      data_i      => p_bus.wdata,                -- data in
      data_o      => resp_bus(RESP_UART1).rdata, -- data out
      ack_o       => resp_bus(RESP_UART1).ack,   -- transfer acknowledge
      -- clock generator --
      clkgen_en_o => uart1_cg_en,                -- enable clock generator
      clkgen_i    => clk_gen,
      -- com lines --
      uart_txd_o  => uart1_txd_o,
      uart_rxd_i  => uart1_rxd_i,
      -- hardware flow control --
      uart_rts_o  => uart1_rts_o,                -- UART.RX ready to receive ("RTR"), low-active, optional
      uart_cts_i  => uart1_cts_i,                -- UART.TX allowed to transmit, low-active, optional
      -- interrupts --
      irq_rx_o    => uart1_rx_irq,               -- rx interrupt
      irq_tx_o    => uart1_tx_irq                -- tx interrupt
    );
    resp_bus(RESP_UART1).err <= '0'; -- no access error possible
  end generate;

  neorv32_uart1_inst_false:
  if (IO_UART1_EN = false) generate
    resp_bus(RESP_UART1) <= resp_bus_entry_terminate_c;
    --
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
      IO_SPI_FIFO => IO_SPI_FIFO -- SPI RTX fifo depth, has to be a power of two, min 1
    )
    port map (
      -- host access --
      clk_i       => clk_i,                    -- global clock line
      rstn_i      => rstn_int,                 -- global reset line, low-active, async
      addr_i      => p_bus.addr,               -- address
      rden_i      => io_rden,                  -- read enable
      wren_i      => io_wren,                  -- write enable
      data_i      => p_bus.wdata,              -- data in
      data_o      => resp_bus(RESP_SPI).rdata, -- data out
      ack_o       => resp_bus(RESP_SPI).ack,   -- transfer acknowledge
      -- clock generator --
      clkgen_en_o => spi_cg_en,                -- enable clock generator
      clkgen_i    => clk_gen,
      -- com lines --
      spi_clk_o   => spi_clk_o,                -- SPI serial clock
      spi_dat_o   => spi_dat_o,                -- controller data out, peripheral data in
      spi_dat_i   => spi_dat_i,                -- controller data in, peripheral data out
      spi_csn_o   => spi_csn_o,                -- SPI CS
      -- interrupt --
      irq_o       => spi_irq                   -- transmission done interrupt
    );
    resp_bus(RESP_SPI).err <= '0'; -- no access error possible
  end generate;

  neorv32_spi_inst_false:
  if (IO_SPI_EN = false) generate
    resp_bus(RESP_SPI) <= resp_bus_entry_terminate_c;
    --
    spi_clk_o <= '0';
    spi_dat_o <= '0';
    spi_csn_o <= (others => '1'); -- CS lines are low-active
    spi_cg_en <= '0';
    spi_irq   <= '0';
  end generate;


  -- Two-Wire Interface (TWI) ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_twi_inst_true:
  if (IO_TWI_EN = true) generate
    neorv32_twi_inst: neorv32_twi
    port map (
      -- host access --
      clk_i       => clk_i,                    -- global clock line
      rstn_i      => rstn_int,                 -- global reset line, low-active, async
      addr_i      => p_bus.addr,               -- address
      rden_i      => io_rden,                  -- read enable
      wren_i      => io_wren,                  -- write enable
      data_i      => p_bus.wdata,              -- data in
      data_o      => resp_bus(RESP_TWI).rdata, -- data out
      ack_o       => resp_bus(RESP_TWI).ack,   -- transfer acknowledge
      -- clock generator --
      clkgen_en_o => twi_cg_en,                -- enable clock generator
      clkgen_i    => clk_gen,
      -- com lines (require external tri-state drivers) --
      twi_sda_i   => twi_sda_i,                -- serial data line input
      twi_sda_o   => twi_sda_o,                -- serial data line output
      twi_scl_i   => twi_scl_i,                -- serial clock line input
      twi_scl_o   => twi_scl_o,                -- serial clock line output
      -- interrupt --
      irq_o       => twi_irq                   -- transfer done IRQ
    );
    resp_bus(RESP_TWI).err <= '0'; -- no access error possible
  end generate;

  neorv32_twi_inst_false:
  if (IO_TWI_EN = false) generate
    resp_bus(RESP_TWI) <= resp_bus_entry_terminate_c;
    --
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
      NUM_CHANNELS => IO_PWM_NUM_CH -- number of PWM channels (0..12)
    )
    port map (
      -- host access --
      clk_i       => clk_i,                    -- global clock line
      rstn_i      => rstn_int,                 -- global reset line, low-active, async
      addr_i      => p_bus.addr,               -- address
      rden_i      => io_rden,                  -- read enable
      wren_i      => io_wren,                  -- write enable
      data_i      => p_bus.wdata,              -- data in
      data_o      => resp_bus(RESP_PWM).rdata, -- data out
      ack_o       => resp_bus(RESP_PWM).ack,   -- transfer acknowledge
      -- clock generator --
      clkgen_en_o => pwm_cg_en,                -- enable clock generator
      clkgen_i    => clk_gen,
      -- pwm output channels --
      pwm_o       => pwm_o
    );
    resp_bus(RESP_PWM).err <= '0'; -- no access error possible
  end generate;

  neorv32_pwm_inst_false:
  if (IO_PWM_NUM_CH = 0) generate
    resp_bus(RESP_PWM) <= resp_bus_entry_terminate_c;
    --
    pwm_cg_en <= '0';
    pwm_o     <= (others => '0');
  end generate;


  -- True Random Number Generator (TRNG) ----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_trng_inst_true:
  if (IO_TRNG_EN = true) generate
    neorv32_trng_inst: neorv32_trng
    generic map (
      IO_TRNG_FIFO => IO_TRNG_FIFO -- RND fifo depth, has to be a power of two, min 1
    )
    port map (
      -- host access --
      clk_i  => clk_i,                     -- global clock line
      rstn_i => rstn_int,                  -- global reset line, low-active, async
      addr_i => p_bus.addr,                -- address
      rden_i => io_rden,                   -- read enable
      wren_i => io_wren,                   -- write enable
      data_i => p_bus.wdata,               -- data in
      data_o => resp_bus(RESP_TRNG).rdata, -- data out
      ack_o  => resp_bus(RESP_TRNG).ack    -- transfer acknowledge
    );
    resp_bus(RESP_TRNG).err <= '0'; -- no access error possible
  end generate;

  neorv32_trng_inst_false:
  if (IO_TRNG_EN = false) generate
    resp_bus(RESP_TRNG) <= resp_bus_entry_terminate_c;
  end generate;


  -- Smart LED (WS2811/WS2812) Interface (NEOLED) -------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_neoled_inst_true:
  if (IO_NEOLED_EN = true) generate
    neorv32_neoled_inst: neorv32_neoled
    generic map (
      FIFO_DEPTH => IO_NEOLED_TX_FIFO -- NEOLED FIFO depth, has to be a power of two, min 1
    )
    port map (
      -- host access --
      clk_i       => clk_i,                       -- global clock line
      rstn_i      => rstn_int,                    -- global reset line, low-active, async
      addr_i      => p_bus.addr,                  -- address
      rden_i      => io_rden,                     -- read enable
      wren_i      => io_wren,                     -- write enable
      data_i      => p_bus.wdata,                 -- data in
      data_o      => resp_bus(RESP_NEOLED).rdata, -- data out
      ack_o       => resp_bus(RESP_NEOLED).ack,   -- transfer acknowledge
      -- clock generator --
      clkgen_en_o => neoled_cg_en,                -- enable clock generator
      clkgen_i    => clk_gen,
      -- interrupt --
      irq_o       => neoled_irq,                  -- interrupt request
      -- NEOLED output --
      neoled_o    => neoled_o                     -- serial async data line
    );
    resp_bus(RESP_NEOLED).err <= '0'; -- no access error possible
  end generate;

  neorv32_neoled_inst_false:
  if (IO_NEOLED_EN = false) generate
    resp_bus(RESP_NEOLED) <= resp_bus_entry_terminate_c;
    --
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
      XIRQ_NUM_CH           => XIRQ_NUM_CH,          -- number of external IRQ channels (0..32)
      XIRQ_TRIGGER_TYPE     => XIRQ_TRIGGER_TYPE,    -- trigger type: 0=level, 1=edge
      XIRQ_TRIGGER_POLARITY => XIRQ_TRIGGER_POLARITY -- trigger polarity: 0=low-level/falling-edge, 1=high-level/rising-edge
    )
    port map (
      -- host access --
      clk_i     => clk_i,                     -- global clock line
      rstn_i    => rstn_int,                  -- global reset line, low-active, async
      addr_i    => p_bus.addr,                -- address
      rden_i    => io_rden,                   -- read enable
      wren_i    => io_wren,                   -- write enable
      data_i    => p_bus.wdata,               -- data in
      data_o    => resp_bus(RESP_XIRQ).rdata, -- data out
      ack_o     => resp_bus(RESP_XIRQ).ack,   -- transfer acknowledge
      -- external interrupt lines --
      xirq_i    => xirq_i,
      -- CPU interrupt --
      cpu_irq_o => xirq_irq
    );
    resp_bus(RESP_XIRQ).err <= '0'; -- no access error possible
  end generate;

  neorv32_xirq_inst_false:
  if (XIRQ_NUM_CH = 0) generate
    resp_bus(RESP_XIRQ) <= resp_bus_entry_terminate_c;
    --
    xirq_irq <= '0';
  end generate;


  -- General Purpose Timer (GPTMR) ----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_gptmr_inst_true:
  if (IO_GPTMR_EN = true) generate
    neorv32_gptmr_inst: neorv32_gptmr
    port map (
      -- host access --
      clk_i       => clk_i,                      -- global clock line
      rstn_i      => rstn_int,                   -- global reset line, low-active, async
      addr_i      => p_bus.addr,                 -- address
      rden_i      => io_rden,                    -- read enable
      wren_i      => io_wren,                    -- write enable
      data_i      => p_bus.wdata,                -- data in
      data_o      => resp_bus(RESP_GPTMR).rdata, -- data out
      ack_o       => resp_bus(RESP_GPTMR).ack,   -- transfer acknowledge
      -- clock generator --
      clkgen_en_o => gptmr_cg_en,                -- enable clock generator
      clkgen_i    => clk_gen,
      -- interrupt --
      irq_o       => gptmr_irq                   -- timer match interrupt
    );
    resp_bus(RESP_GPTMR).err <= '0'; -- no access error possible
  end generate;

  neorv32_gptmr_inst_false:
  if (IO_GPTMR_EN = false) generate
    resp_bus(RESP_GPTMR) <= resp_bus_entry_terminate_c;
    --
    gptmr_cg_en <= '0';
    gptmr_irq   <= '0';
  end generate;


  -- 1-Wire Interface Controller (ONEWIRE) --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_onewire_inst_true:
  if (IO_ONEWIRE_EN = true) generate
    neorv32_onewire_inst: neorv32_onewire
    port map (
    -- host access --
      clk_i       => clk_i,                        -- global clock line
      rstn_i      => rstn_int,                     -- global reset line, low-active, async
      addr_i      => p_bus.addr,                   -- address
      rden_i      => io_rden,                      -- read enable
      wren_i      => io_wren,                      -- write enable
      data_i      => p_bus.wdata,                  -- data in
      data_o      => resp_bus(RESP_ONEWIRE).rdata, -- data out
      ack_o       => resp_bus(RESP_ONEWIRE).ack,   -- transfer acknowledge
      -- clock generator --
      clkgen_en_o => onewire_cg_en,                -- enable clock generator
      clkgen_i    => clk_gen,
      -- com lines (require external tri-state drivers) --
      onewire_i   => onewire_i,                    -- 1-wire line state
      onewire_o   => onewire_o,                    -- 1-wire line pull-down
      -- interrupt --
      irq_o       => onewire_irq                   -- transfer done IRQ
    );
    resp_bus(RESP_ONEWIRE).err <= '0'; -- no access error possible
  end generate;

  neorv32_onewire_inst_false:
  if (IO_ONEWIRE_EN = false) generate
    resp_bus(RESP_ONEWIRE) <= resp_bus_entry_terminate_c;
    --
    onewire_o     <= '1';
    onewire_cg_en <= '0';
    onewire_irq   <= '0';
  end generate;


  -- System Configuration Information Memory (SYSINFO) --------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_sysinfo_inst: neorv32_sysinfo
  generic map (
    -- General --
    CLOCK_FREQUENCY      => CLOCK_FREQUENCY,      -- clock frequency of clk_i in Hz
    CUSTOM_ID            => CUSTOM_ID,            -- custom user-defined ID
    INT_BOOTLOADER_EN    => INT_BOOTLOADER_EN,    -- implement processor-internal bootloader?
    -- Physical memory protection (PMP) --
    PMP_NUM_REGIONS      => PMP_NUM_REGIONS,      -- number of regions (0..16)
    -- internal Instruction memory --
    MEM_INT_IMEM_EN      => MEM_INT_IMEM_EN,      -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE    => MEM_INT_IMEM_SIZE,    -- size of processor-internal instruction memory in bytes
    -- Internal Data memory --
    MEM_INT_DMEM_EN      => MEM_INT_DMEM_EN,      -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE    => MEM_INT_DMEM_SIZE,    -- size of processor-internal data memory in bytes
    -- Instruction cache --
    ICACHE_EN            => ICACHE_EN,            -- implement instruction cache
    ICACHE_NUM_BLOCKS    => ICACHE_NUM_BLOCKS,    -- i-cache: number of blocks (min 2), has to be a power of 2
    ICACHE_BLOCK_SIZE    => ICACHE_BLOCK_SIZE,    -- i-cache: block size in bytes (min 4), has to be a power of 2
    ICACHE_ASSOCIATIVITY => ICACHE_ASSOCIATIVITY, -- i-cache: associativity (min 1), has to be a power 2
    -- Data cache --
    DCACHE_EN            => DCACHE_EN,            -- implement data cache
    DCACHE_NUM_BLOCKS    => DCACHE_NUM_BLOCKS,    -- d-cache: number of blocks (min 2), has to be a power of 2
    DCACHE_BLOCK_SIZE    => DCACHE_BLOCK_SIZE,    -- d-cache: block size in bytes (min 4), has to be a power of 2
    -- External memory interface --
    MEM_EXT_EN           => MEM_EXT_EN,           -- implement external memory bus interface?
    MEM_EXT_BIG_ENDIAN   => MEM_EXT_BIG_ENDIAN,   -- byte order: true=big-endian, false=little-endian
    -- On-Chip Debugger --
    ON_CHIP_DEBUGGER_EN  => ON_CHIP_DEBUGGER_EN,  -- implement OCD?
    -- Processor peripherals --
    IO_GPIO_NUM          => IO_GPIO_NUM,          -- number of GPIO input/output pairs (0..64)
    IO_MTIME_EN          => IO_MTIME_EN,          -- implement machine system timer (MTIME)?
    IO_UART0_EN          => IO_UART0_EN,          -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_UART1_EN          => IO_UART1_EN,          -- implement secondary universal asynchronous receiver/transmitter (UART1)?
    IO_SPI_EN            => IO_SPI_EN,            -- implement serial peripheral interface (SPI)?
    IO_SDI_EN            => IO_SDI_EN,            -- implement serial data interface (SDI)?
    IO_TWI_EN            => IO_TWI_EN,            -- implement two-wire interface (TWI)?
    IO_PWM_NUM_CH        => IO_PWM_NUM_CH,        -- number of PWM channels to implement
    IO_WDT_EN            => IO_WDT_EN,            -- implement watch dog timer (WDT)?
    IO_TRNG_EN           => IO_TRNG_EN,           -- implement true random number generator (TRNG)?
    IO_CFS_EN            => IO_CFS_EN,            -- implement custom functions subsystem (CFS)?
    IO_NEOLED_EN         => IO_NEOLED_EN,         -- implement NeoPixel-compatible smart LED interface (NEOLED)?
    IO_XIRQ_NUM_CH       => XIRQ_NUM_CH,          -- number of external interrupt (XIRQ) channels to implement
    IO_GPTMR_EN          => IO_GPTMR_EN,          -- implement general purpose timer (GPTMR)?
    IO_XIP_EN            => IO_XIP_EN,            -- implement execute in place module (XIP)?
    IO_ONEWIRE_EN        => IO_ONEWIRE_EN         -- implement 1-wire interface (ONEWIRE)?
  )
  port map (
    -- host access --
    clk_i  => clk_i,                        -- global clock line
    addr_i => p_bus.addr,                   -- address
    rden_i => io_rden,                      -- read enable
    wren_i => io_wren,                      -- write enable
    data_o => resp_bus(RESP_SYSINFO).rdata, -- data out
    ack_o  => resp_bus(RESP_SYSINFO).ack,   -- transfer acknowledge
    err_o  => resp_bus(RESP_SYSINFO).err    -- transfer error
  );


-- ****************************************************************************************************************************
-- On-Chip Debugger Complex
-- ****************************************************************************************************************************

  neorv32_neorv32_ocd_inst_true:
  if (ON_CHIP_DEBUGGER_EN = true) generate

    -- On-Chip Debugger - Debug Module (DM) ---------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_debug_dm_inst: neorv32_debug_dm
    port map (
      -- global control --
      clk_i             => clk_i,                    -- global clock line
      rstn_i            => rstn_ext,                 -- external reset, low-active
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
      cpu_debug_i       => cpu_s.debug,              -- CPU is in debug mode
      cpu_addr_i        => p_bus.addr,               -- address
      cpu_rden_i        => p_bus.re,                 -- read enable
      cpu_wren_i        => p_bus.we,                 -- write enable
      cpu_ben_i         => p_bus.ben,                -- byte write enable
      cpu_data_i        => p_bus.wdata,              -- data in
      cpu_data_o        => resp_bus(RESP_OCD).rdata, -- data out
      cpu_ack_o         => resp_bus(RESP_OCD).ack,   -- transfer acknowledge
      -- CPU control --
      cpu_ndmrstn_o     => dci_ndmrstn,              -- soc reset
      cpu_halt_req_o    => dci_halt_req              -- request hart to halt (enter debug mode)
    );
    resp_bus(RESP_OCD).err <= '0'; -- no access error possible


    -- On-Chip Debugger - Debug Transport Module (DTM) ----------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_debug_dtm_inst: neorv32_debug_dtm
    generic map (
      IDCODE_VERSION => (others => '0'), -- version
      IDCODE_PARTID  => (others => '0'), -- part number
      IDCODE_MANID   => (others => '0')  -- manufacturer id
    )
    port map (
      -- global control --
      clk_i             => clk_i,    -- global clock line
      rstn_i            => rstn_ext, -- external reset, low-active
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

  end generate;

  neorv32_debug_ocd_inst_false:
  if (ON_CHIP_DEBUGGER_EN = false) generate
    jtag_tdo_o         <= jtag_tdi_i; -- JTAG feed-through
    resp_bus(RESP_OCD) <= resp_bus_entry_terminate_c;
    dci_ndmrstn        <= '1';
    dci_halt_req       <= '0';
  end generate;


end neorv32_top_rtl;
