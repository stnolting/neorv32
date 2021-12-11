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

entity neorv32_top is
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
    CPU_EXTENSION_RISCV_Zicntr   : boolean := true;   -- implement base counters?
    CPU_EXTENSION_RISCV_Zihpm    : boolean := false;  -- implement hardware performance monitors?
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

    -- External memory interface (WISHBONE) --
    MEM_EXT_EN                   : boolean := false;  -- implement external memory bus interface?
    MEM_EXT_TIMEOUT              : natural := 255;    -- cycles after a pending bus access auto-terminates (0 = disabled)
    MEM_EXT_PIPE_MODE            : boolean := false;  -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
    MEM_EXT_BIG_ENDIAN           : boolean := false;  -- byte order: true=big-endian, false=little-endian
    MEM_EXT_ASYNC_RX             : boolean := false;  -- use register buffer for RX data when false

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
    IO_UART0_RX_FIFO             : natural := 1;      -- RX fifo depth, has to be a power of two, min 1
    IO_UART0_TX_FIFO             : natural := 1;      -- TX fifo depth, has to be a power of two, min 1
    IO_UART1_EN                  : boolean := false;  -- implement secondary universal asynchronous receiver/transmitter (UART1)?
    IO_UART1_RX_FIFO             : natural := 1;      -- RX fifo depth, has to be a power of two, min 1
    IO_UART1_TX_FIFO             : natural := 1;      -- TX fifo depth, has to be a power of two, min 1
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
    IO_NEOLED_TX_FIFO            : natural := 1;      -- NEOLED TX FIFO depth, 1..32k, has to be a power of two
    IO_GPTMR_EN                  : boolean := false   -- implement general purpose timer (GPTMR)?
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
    wb_lock_o      : out std_ulogic; -- exclusive access request
    wb_ack_i       : in  std_ulogic := 'L'; -- transfer acknowledge
    wb_err_i       : in  std_ulogic := 'L'; -- transfer error

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
end neorv32_top;

architecture neorv32_top_rtl of neorv32_top is

  -- CPU boot configuration --
  constant cpu_boot_addr_c : std_ulogic_vector(31 downto 0) := cond_sel_stdulogicvector_f(INT_BOOTLOADER_EN, boot_rom_base_c, ispace_base_c);

  -- alignment check for internal memories --
  constant imem_align_check_c : std_ulogic_vector(index_size_f(MEM_INT_IMEM_SIZE)-1 downto 0) := (others => '0');
  constant dmem_align_check_c : std_ulogic_vector(index_size_f(MEM_INT_DMEM_SIZE)-1 downto 0) := (others => '0');

  -- helpers --
  constant io_slink_en_c : boolean := boolean(SLINK_NUM_RX > 0) or boolean(SLINK_NUM_TX > 0); -- implement slink at all?

  -- reset generator --
  signal rstn_gen : std_ulogic_vector(7 downto 0) := (others => '0'); -- initialize (=reset) via  (for FPGAs only)
  signal ext_rstn : std_ulogic;
  signal sys_rstn : std_ulogic;
  signal wdt_rstn : std_ulogic;

  -- clock generator --
  signal clk_div    : std_ulogic_vector(11 downto 0);
  signal clk_div_ff : std_ulogic_vector(11 downto 0);
  signal clk_gen    : std_ulogic_vector(07 downto 0);
  signal clk_gen_en : std_ulogic_vector(08 downto 0);
  --
  signal wdt_cg_en    : std_ulogic;
  signal uart0_cg_en  : std_ulogic;
  signal uart1_cg_en  : std_ulogic;
  signal spi_cg_en    : std_ulogic;
  signal twi_cg_en    : std_ulogic;
  signal pwm_cg_en    : std_ulogic;
  signal cfs_cg_en    : std_ulogic;
  signal neoled_cg_en : std_ulogic;
  signal gptmr_cg_en  : std_ulogic;

  -- bus interface --
  type bus_interface_t is record
    addr   : std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
    rdata  : std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
    wdata  : std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
    ben    : std_ulogic_vector(03 downto 0); -- byte enable
    we     : std_ulogic; -- write enable
    re     : std_ulogic; -- read enable
    ack    : std_ulogic; -- bus transfer acknowledge
    err    : std_ulogic; -- bus transfer error
    fence  : std_ulogic; -- fence(i) instruction executed
    priv   : std_ulogic_vector(1 downto 0); -- current privilege level
    src    : std_ulogic; -- access source (1=instruction fetch, 0=data access)
    lock   : std_ulogic; -- exclusive access request
  end record;
  signal cpu_i, i_cache, cpu_d, p_bus : bus_interface_t;

  -- bus access error (from BUSKEEPER) --
  signal bus_error : std_ulogic;

  -- debug core interface (DCI) --
  signal dci_ndmrstn  : std_ulogic;
  signal dci_halt_req : std_ulogic;

  -- debug module interface (DMI) --
  type dmi_t is record
    rstn       : std_ulogic;
    req_valid  : std_ulogic;
    req_ready  : std_ulogic; -- DMI is allowed to make new requests when set
    req_addr   : std_ulogic_vector(06 downto 0);
    req_op     : std_ulogic; -- 0=read, 1=write
    req_data   : std_ulogic_vector(31 downto 0);
    resp_valid : std_ulogic; -- response valid when set
    resp_ready : std_ulogic; -- ready to receive respond
    resp_data  : std_ulogic_vector(31 downto 0);
    resp_err   : std_ulogic; -- 0=ok, 1=error
  end record;
  signal dmi : dmi_t;

  -- io space access --
  signal io_acc  : std_ulogic;
  signal io_rden : std_ulogic;
  signal io_wren : std_ulogic;

  -- module response bus - entry type --
  type resp_bus_entry_t is record
    rdata : std_ulogic_vector(data_width_c-1 downto 0);
    ack   : std_ulogic;
    err   : std_ulogic;
  end record;
  constant resp_bus_entry_terminate_c : resp_bus_entry_t := (rdata => (others => '0'), ack => '0', err => '0');

  -- module response bus - device ID --
  type resp_bus_id_t is (RESP_BUSKEEPER, RESP_IMEM, RESP_DMEM, RESP_BOOTROM, RESP_WISHBONE, RESP_GPIO, RESP_MTIME, RESP_UART0, RESP_UART1, RESP_SPI,
                         RESP_TWI, RESP_PWM, RESP_WDT, RESP_TRNG, RESP_CFS, RESP_NEOLED, RESP_SYSINFO, RESP_OCD, RESP_SLINK, RESP_XIRQ, RESP_GPTMR);

  -- module response bus --
  type resp_bus_t is array (resp_bus_id_t) of resp_bus_entry_t;
  signal resp_bus : resp_bus_t := (others => resp_bus_entry_terminate_c);

  -- IRQs --
  signal fast_irq      : std_ulogic_vector(15 downto 0);
  signal mtime_irq     : std_ulogic;
  signal wdt_irq       : std_ulogic;
  signal uart0_rxd_irq : std_ulogic;
  signal uart0_txd_irq : std_ulogic;
  signal uart1_rxd_irq : std_ulogic;
  signal uart1_txd_irq : std_ulogic;
  signal spi_irq       : std_ulogic;
  signal twi_irq       : std_ulogic;
  signal cfs_irq       : std_ulogic;
  signal neoled_irq    : std_ulogic;
  signal slink_tx_irq  : std_ulogic;
  signal slink_rx_irq  : std_ulogic;
  signal xirq_irq      : std_ulogic;
  signal gptmr_irq     : std_ulogic;

  -- misc --
  signal mtime_time  : std_ulogic_vector(63 downto 0); -- current system time from MTIME
  signal ext_timeout : std_ulogic;
  signal ext_access  : std_ulogic;
  signal debug_mode  : std_ulogic;

begin

  -- Processor IO/Peripherals Configuration -------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert false report
  "NEORV32 PROCESSOR IO Configuration: " &
  cond_sel_string_f(IO_GPIO_EN, "GPIO ", "") &
  cond_sel_string_f(IO_MTIME_EN, "MTIME ", "") &
  cond_sel_string_f(IO_UART0_EN, "UART0 ", "") &
  cond_sel_string_f(IO_UART1_EN, "UART1 ", "") &
  cond_sel_string_f(IO_SPI_EN, "SPI ", "") &
  cond_sel_string_f(IO_TWI_EN, "TWI ", "") &
  cond_sel_string_f(boolean(IO_PWM_NUM_CH > 0), "PWM ", "") &
  cond_sel_string_f(IO_WDT_EN, "WDT ", "") &
  cond_sel_string_f(IO_TRNG_EN, "TRNG ", "") &
  cond_sel_string_f(IO_CFS_EN, "CFS ", "") &
  cond_sel_string_f(io_slink_en_c, "SLINK ", "") &
  cond_sel_string_f(IO_NEOLED_EN, "NEOLED ", "") &
  cond_sel_string_f(boolean(XIRQ_NUM_CH > 0), "XIRQ ", "") &
  cond_sel_string_f(IO_GPTMR_EN, "GPTMR ", "") &
  ""
  severity note;


  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- boot configuration --
  assert not (INT_BOOTLOADER_EN = true) report "NEORV32 PROCESSOR CONFIG NOTE: Boot configuration: Indirect boot via bootloader (processor-internal BOOTROM)." severity note;
  assert not ((INT_BOOTLOADER_EN = false) and (MEM_INT_IMEM_EN = true)) report "NEORV32 PROCESSOR CONFIG NOTE: Boot configuration: Direct boot from memory (processor-internal IMEM)." severity note;
  assert not ((INT_BOOTLOADER_EN = false) and (MEM_INT_IMEM_EN = false)) report "NEORV32 PROCESSOR CONFIG NOTE: Boot configuration: Direct boot from memory (processor-external (I)MEM)." severity note;
  --
  assert not ((MEM_EXT_EN = false) and (MEM_INT_DMEM_EN = false)) report "NEORV32 PROCESSOR CONFIG ERROR! Core cannot fetch data without external memory interface and internal IMEM." severity error;
  assert not ((MEM_EXT_EN = false) and (MEM_INT_IMEM_EN = false) and (INT_BOOTLOADER_EN = false)) report "NEORV32 PROCESSOR CONFIG ERROR! Core cannot fetch instructions without external memory interface, internal IMEM and bootloader." severity error;

  -- memory system - size --
  assert not ((MEM_INT_DMEM_EN = true) and (is_power_of_two_f(MEM_INT_IMEM_SIZE) = false)) report "NEORV32 PROCESSOR CONFIG WARNING! MEM_INT_IMEM_SIZE should be a power of 2 to allow optimal hardware mapping." severity warning;
  assert not ((MEM_INT_IMEM_EN = true) and (is_power_of_two_f(MEM_INT_DMEM_SIZE) = false)) report "NEORV32 PROCESSOR CONFIG WARNING! MEM_INT_DMEM_SIZE should be a power of 2 to allow optimal hardware mapping." severity warning;

  -- memory system - alignment --
  assert not (ispace_base_c(1 downto 0) /= "00") report "NEORV32 PROCESSOR CONFIG ERROR! Instruction memory space base address must be 4-byte-aligned." severity error;
  assert not (dspace_base_c(1 downto 0) /= "00") report "NEORV32 PROCESSOR CONFIG ERROR! Data memory space base address must be 4-byte-aligned." severity error;
  assert not ((ispace_base_c(index_size_f(MEM_INT_IMEM_SIZE)-1 downto 0) /= imem_align_check_c) and (MEM_INT_IMEM_EN = true)) report "NEORV32 PROCESSOR CONFIG ERROR! Instruction memory space base address has to be aligned to IMEM size." severity error;
  assert not ((dspace_base_c(index_size_f(MEM_INT_DMEM_SIZE)-1 downto 0) /= dmem_align_check_c) and (MEM_INT_DMEM_EN = true)) report "NEORV32 PROCESSOR CONFIG ERROR! Data memory space base address has to be aligned to DMEM size." severity error;

  -- memory system - layout warning --
  assert not (ispace_base_c /= x"00000000") report "NEORV32 PROCESSOR CONFIG WARNING! Non-default base address for instruction address space. Make sure this is sync with the software framework." severity warning;
  assert not (dspace_base_c /= x"80000000") report "NEORV32 PROCESSOR CONFIG WARNING! Non-default base address for data address space. Make sure this is sync with the software framework." severity warning;

  -- memory system - the i-cache is intended to accelerate instruction fetch via the external memory interface only --
  assert not ((ICACHE_EN = true) and (MEM_EXT_EN = false)) report "NEORV32 PROCESSOR CONFIG NOTE. Implementing i-cache without having the external memory interface implemented. The i-cache is intended to accelerate instruction fetch via the external memory interface." severity note;

  -- on-chip debugger --
  assert not (ON_CHIP_DEBUGGER_EN = true) report "NEORV32 PROCESSOR CONFIG NOTE: Implementing on-chip debugger (OCD)." severity note;


  -- Reset Generator ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  reset_generator: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      rstn_gen <= (others => '0');
      sys_rstn <= '0';
    elsif rising_edge(clk_i) then
      -- keep internal reset active for at least <rstn_gen'size> clock cycles --
      rstn_gen <= rstn_gen(rstn_gen'left-1 downto 0) & '1';
      -- system reset: can also be triggered by watchdog and debug module --
      sys_rstn <= ext_rstn and wdt_rstn and dci_ndmrstn;
    end if;
  end process reset_generator;

  -- beautified external reset signal --
  ext_rstn <= rstn_gen(rstn_gen'left);


  -- Clock Generator ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  clock_generator: process(sys_rstn, clk_i)
  begin
    if (sys_rstn = '0') then
      clk_gen_en <= (others => '-');
      clk_div    <= (others => '0');
      clk_div_ff <= (others => '-');
      clk_gen    <= (others => '-');
    elsif rising_edge(clk_i) then
      -- fresh clocks anyone? --
      clk_gen_en(0) <= wdt_cg_en;
      clk_gen_en(1) <= uart0_cg_en;
      clk_gen_en(2) <= uart1_cg_en;
      clk_gen_en(3) <= spi_cg_en;
      clk_gen_en(4) <= twi_cg_en;
      clk_gen_en(5) <= pwm_cg_en;
      clk_gen_en(6) <= cfs_cg_en;
      clk_gen_en(7) <= neoled_cg_en;
      clk_gen_en(8) <= gptmr_cg_en;
      -- actual clock generator --
      if (or_reduce_f(clk_gen_en) = '1') then
        clk_div <= std_ulogic_vector(unsigned(clk_div) + 1);
      end if;
      -- clock enables: rising edge detectors --
      clk_div_ff <= clk_div;
      clk_gen(clk_div2_c)    <= clk_div(0)  and (not clk_div_ff(0));  -- CLK/2
      clk_gen(clk_div4_c)    <= clk_div(1)  and (not clk_div_ff(1));  -- CLK/4
      clk_gen(clk_div8_c)    <= clk_div(2)  and (not clk_div_ff(2));  -- CLK/8
      clk_gen(clk_div64_c)   <= clk_div(5)  and (not clk_div_ff(5));  -- CLK/64
      clk_gen(clk_div128_c)  <= clk_div(6)  and (not clk_div_ff(6));  -- CLK/128
      clk_gen(clk_div1024_c) <= clk_div(9)  and (not clk_div_ff(9));  -- CLK/1024
      clk_gen(clk_div2048_c) <= clk_div(10) and (not clk_div_ff(10)); -- CLK/2048
      clk_gen(clk_div4096_c) <= clk_div(11) and (not clk_div_ff(11)); -- CLK/4096
    end if;
  end process clock_generator;


  -- CPU Core -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_inst: neorv32_cpu
  generic map (
    -- General --
    HW_THREAD_ID                 => HW_THREAD_ID,        -- hardware thread id
    CPU_BOOT_ADDR                => cpu_boot_addr_c,     -- cpu boot address
    CPU_DEBUG_ADDR               => dm_base_c,           -- cpu debug mode start address
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
    CPU_EXTENSION_RISCV_Zmmul    => CPU_EXTENSION_RISCV_Zmmul,    -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_DEBUG    => ON_CHIP_DEBUGGER_EN,          -- implement CPU debug mode?
    -- Extension Options --
    FAST_MUL_EN                  => FAST_MUL_EN,         -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                => FAST_SHIFT_EN,       -- use barrel shifter for shift operations
    CPU_CNT_WIDTH                => CPU_CNT_WIDTH,       -- total width of CPU cycle and instret counters (0..64)
    CPU_IPB_ENTRIES              => CPU_IPB_ENTRIES,     -- entries is instruction prefetch buffer, has to be a power of 2
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS              => PMP_NUM_REGIONS,     -- number of regions (0..64)
    PMP_MIN_GRANULARITY          => PMP_MIN_GRANULARITY, -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 => HPM_NUM_CNTS,        -- number of implemented HPM counters (0..29)
    HPM_CNT_WIDTH                => HPM_CNT_WIDTH        -- total size of HPM counters (0..64)
  )
  port map (
    -- global control --
    clk_i          => clk_i,        -- global clock, rising edge
    rstn_i         => sys_rstn,     -- global reset, low-active, async
    sleep_o        => open,         -- cpu is in sleep mode when set
    debug_o        => debug_mode,   -- cpu is in debug mode when set
    -- instruction bus interface --
    i_bus_addr_o   => cpu_i.addr,   -- bus access address
    i_bus_rdata_i  => cpu_i.rdata,  -- bus read data
    i_bus_wdata_o  => cpu_i.wdata,  -- bus write data
    i_bus_ben_o    => cpu_i.ben,    -- byte enable
    i_bus_we_o     => cpu_i.we,     -- write enable
    i_bus_re_o     => cpu_i.re,     -- read enable
    i_bus_lock_o   => cpu_i.lock,   -- exclusive access request
    i_bus_ack_i    => cpu_i.ack,    -- bus transfer acknowledge
    i_bus_err_i    => cpu_i.err,    -- bus transfer error
    i_bus_fence_o  => cpu_i.fence,  -- executed FENCEI operation
    i_bus_priv_o   => cpu_i.priv,   -- privilege level
    -- data bus interface --
    d_bus_addr_o   => cpu_d.addr,   -- bus access address
    d_bus_rdata_i  => cpu_d.rdata,  -- bus read data
    d_bus_wdata_o  => cpu_d.wdata,  -- bus write data
    d_bus_ben_o    => cpu_d.ben,    -- byte enable
    d_bus_we_o     => cpu_d.we,     -- write enable
    d_bus_re_o     => cpu_d.re,     -- read enable
    d_bus_lock_o   => cpu_d.lock,   -- exclusive access request
    d_bus_ack_i    => cpu_d.ack,    -- bus transfer acknowledge
    d_bus_err_i    => cpu_d.err,    -- bus transfer error
    d_bus_fence_o  => cpu_d.fence,  -- executed FENCE operation
    d_bus_priv_o   => cpu_d.priv,   -- privilege level
    -- system time input from MTIME --
    time_i         => mtime_time,   -- current system time
    -- non-maskable interrupt --
    msw_irq_i      => msw_irq_i,    -- machine software interrupt
    mext_irq_i     => mext_irq_i,   -- machine external interrupt request
    mtime_irq_i    => mtime_irq,    -- machine timer interrupt
    -- fast interrupts (custom) --
    firq_i         => fast_irq,     -- fast interrupt trigger
    -- debug mode (halt) request --
    db_halt_req_i  => dci_halt_req
  );

  -- misc --
  cpu_i.src <= '1'; -- initialized but unused
  cpu_d.src <= '0'; -- initialized but unused

  -- advanced memory control --
  fence_o  <= cpu_d.fence; -- indicates an executed FENCE operation
  fencei_o <= cpu_i.fence; -- indicates an executed FENCEI operation

  -- fast interrupt requests (FIRQs) --
  -- these signals are single-shot --
  fast_irq(00) <= wdt_irq;       -- HIGHEST PRIORITY - watchdog
  fast_irq(01) <= cfs_irq;       -- custom functions subsystem
  fast_irq(02) <= uart0_rxd_irq; -- primary UART (UART0) RX
  fast_irq(03) <= uart0_txd_irq; -- primary UART (UART0) TX
  fast_irq(04) <= uart1_rxd_irq; -- secondary UART (UART1) RX
  fast_irq(05) <= uart1_txd_irq; -- secondary UART (UART1) TX
  fast_irq(06) <= spi_irq;       -- SPI
  fast_irq(07) <= twi_irq;       -- TWI
  fast_irq(08) <= xirq_irq;      -- external interrupt controller
  fast_irq(09) <= neoled_irq;    -- NEOLED buffer free
  fast_irq(10) <= slink_rx_irq;  -- SLINK RX
  fast_irq(11) <= slink_tx_irq;  -- SLINK TX
  fast_irq(12) <= gptmr_irq;     -- general purpose timer
  --
  fast_irq(13) <= '0'; -- reserved
  fast_irq(14) <= '0'; -- reserved
  fast_irq(15) <= '0'; -- LOWEST PRIORITY - reserved


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
      clk_i         => clk_i,          -- global clock, rising edge
      rstn_i        => sys_rstn,       -- global reset, low-active, async
      clear_i       => cpu_i.fence,    -- cache clear
      -- host controller interface --
      host_addr_i   => cpu_i.addr,     -- bus access address
      host_rdata_o  => cpu_i.rdata,    -- bus read data
      host_wdata_i  => cpu_i.wdata,    -- bus write data
      host_ben_i    => cpu_i.ben,      -- byte enable
      host_we_i     => cpu_i.we,       -- write enable
      host_re_i     => cpu_i.re,       -- read enable
      host_ack_o    => cpu_i.ack,      -- bus transfer acknowledge
      host_err_o    => cpu_i.err,      -- bus transfer error
      -- peripheral bus interface --
      bus_addr_o    => i_cache.addr,   -- bus access address
      bus_rdata_i   => i_cache.rdata,  -- bus read data
      bus_wdata_o   => i_cache.wdata,  -- bus write data
      bus_ben_o     => i_cache.ben,    -- byte enable
      bus_we_o      => i_cache.we,     -- write enable
      bus_re_o      => i_cache.re,     -- read enable
      bus_ack_i     => i_cache.ack,    -- bus transfer acknowledge
      bus_err_i     => i_cache.err     -- bus transfer error
    );
  end generate;

  -- TODO: do not use LOCKED instruction fetch --
  i_cache.lock <= '0';

  neorv32_icache_inst_false:
  if (ICACHE_EN = false) generate
    i_cache.addr  <= cpu_i.addr;
    cpu_i.rdata   <= i_cache.rdata;
    i_cache.wdata <= cpu_i.wdata;
    i_cache.ben   <= cpu_i.ben;
    i_cache.we    <= cpu_i.we;
    i_cache.re    <= cpu_i.re;
    cpu_i.ack     <= i_cache.ack;
    cpu_i.err     <= i_cache.err;
  end generate;


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
    rstn_i          => sys_rstn,       -- global reset, low-active, async
    -- controller interface a --
    ca_bus_addr_i   => cpu_d.addr,     -- bus access address
    ca_bus_rdata_o  => cpu_d.rdata,    -- bus read data
    ca_bus_wdata_i  => cpu_d.wdata,    -- bus write data
    ca_bus_ben_i    => cpu_d.ben,      -- byte enable
    ca_bus_we_i     => cpu_d.we,       -- write enable
    ca_bus_re_i     => cpu_d.re,       -- read enable
    ca_bus_lock_i   => cpu_d.lock,     -- exclusive access request
    ca_bus_ack_o    => cpu_d.ack,      -- bus transfer acknowledge
    ca_bus_err_o    => cpu_d.err,      -- bus transfer error
    -- controller interface b --
    cb_bus_addr_i   => i_cache.addr,   -- bus access address
    cb_bus_rdata_o  => i_cache.rdata,  -- bus read data
    cb_bus_wdata_i  => i_cache.wdata,  -- bus write data
    cb_bus_ben_i    => i_cache.ben,    -- byte enable
    cb_bus_we_i     => i_cache.we,     -- write enable
    cb_bus_re_i     => i_cache.re,     -- read enable
    cb_bus_lock_i   => i_cache.lock,   -- exclusive access request
    cb_bus_ack_o    => i_cache.ack,    -- bus transfer acknowledge
    cb_bus_err_o    => i_cache.err,    -- bus transfer error
    -- peripheral bus --
    p_bus_src_o     => p_bus.src,      -- access source: 0 = A (data), 1 = B (instructions)
    p_bus_addr_o    => p_bus.addr,     -- bus access address
    p_bus_rdata_i   => p_bus.rdata,    -- bus read data
    p_bus_wdata_o   => p_bus.wdata,    -- bus write data
    p_bus_ben_o     => p_bus.ben,      -- byte enable
    p_bus_we_o      => p_bus.we,       -- write enable
    p_bus_re_o      => p_bus.re,       -- read enable
    p_bus_lock_o    => p_bus.lock,     -- exclusive access request
    p_bus_ack_i     => p_bus.ack,      -- bus transfer acknowledge
    p_bus_err_i     => bus_error       -- bus transfer error
  );

  -- current CPU privilege level --
  p_bus.priv <= cpu_i.priv; -- note: cpu_i.priv == cpu_d.priv

  -- fence operation (unused) --
  p_bus.fence <= cpu_d.fence or cpu_i.fence;

  -- bus response --
  bus_response: process(resp_bus)
    variable rdata_v : std_ulogic_vector(data_width_c-1 downto 0);
    variable ack_v   : std_ulogic;
    variable err_v   : std_ulogic;
  begin
    rdata_v := (others => '0');
    ack_v   := '0';
    err_v   := '0';
    for i in resp_bus'range loop
      rdata_v := rdata_v or resp_bus(i).rdata; -- read data
      ack_v   := ack_v   or resp_bus(i).ack;   -- acknowledge
      err_v   := err_v   or resp_bus(i).err;   -- error
    end loop; -- i
    p_bus.rdata <= rdata_v; -- processor bus: CPU transfer data input
    p_bus.ack   <= ack_v;   -- processor bus: CPU transfer ACK input
    p_bus.err   <= err_v;   -- processor bus: CPU transfer data bus error input
  end process;


  -- Bus Keeper (BUSKEEPER) -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_bus_keeper_inst: neorv32_bus_keeper
  port map (
    -- host access --
    clk_i      => clk_i,                          -- global clock line
    rstn_i     => sys_rstn,                       -- global reset line, low-active, use as async
    addr_i     => p_bus.addr,                     -- address
    rden_i     => io_rden,                        -- read enable
    wren_i     => io_wren,                        -- byte write enable
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
    bus_ext_i  => ext_access                      -- external bus access
  );

  -- unused, BUSKEEPER **directly** issues error to the CPU --
  resp_bus(RESP_BUSKEEPER).err <= '0';


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
      ack_o  => resp_bus(RESP_IMEM).ack    -- transfer acknowledge
    );
    resp_bus(RESP_IMEM).err <= '0'; -- no access error possible
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
      addr_i => p_bus.addr,                   -- address
      data_o => resp_bus(RESP_BOOTROM).rdata, -- data out
      ack_o  => resp_bus(RESP_BOOTROM).ack    -- transfer acknowledge
    );
    resp_bus(RESP_BOOTROM).err <= '0'; -- no access error possible
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
      ASYNC_RX          => MEM_EXT_ASYNC_RX    -- use register buffer for RX data when false
    )
    port map (
      -- global control --
      clk_i     => clk_i,                         -- global clock line
      rstn_i    => sys_rstn,                      -- global reset line, low-active
      -- host access --
      src_i     => p_bus.src,                     -- access type (0: data, 1:instruction)
      addr_i    => p_bus.addr,                    -- address
      rden_i    => p_bus.re,                      -- read enable
      wren_i    => p_bus.we,                      -- write enable
      ben_i     => p_bus.ben,                     -- byte write enable
      data_i    => p_bus.wdata,                   -- data in
      data_o    => resp_bus(RESP_WISHBONE).rdata, -- data out
      lock_i    => p_bus.lock,                    -- exclusive access request
      ack_o     => resp_bus(RESP_WISHBONE).ack,   -- transfer acknowledge
      err_o     => resp_bus(RESP_WISHBONE).err,   -- transfer error
      tmo_o     => ext_timeout,                   -- transfer timeout
      priv_i    => p_bus.priv,                    -- current CPU privilege level
      ext_o     => ext_access,                    -- active external access
      -- wishbone interface --
      wb_tag_o  => wb_tag_o,                      -- request tag
      wb_adr_o  => wb_adr_o,                      -- address
      wb_dat_i  => wb_dat_i,                      -- read data
      wb_dat_o  => wb_dat_o,                      -- write data
      wb_we_o   => wb_we_o,                       -- read/write
      wb_sel_o  => wb_sel_o,                      -- byte enable
      wb_stb_o  => wb_stb_o,                      -- strobe
      wb_cyc_o  => wb_cyc_o,                      -- valid cycle
      wb_lock_o => wb_lock_o,                     -- exclusive access request
      wb_ack_i  => wb_ack_i,                      -- transfer acknowledge
      wb_err_i  => wb_err_i                       -- transfer error
    );
  end generate;

  neorv32_wishbone_inst_false:
  if (MEM_EXT_EN = false) generate
    resp_bus(RESP_WISHBONE) <= resp_bus_entry_terminate_c;
    ext_timeout <= '0';
    ext_access  <= '0';
    --
    wb_adr_o  <= (others => '0');
    wb_dat_o  <= (others => '0');
    wb_we_o   <= '0';
    wb_sel_o  <= (others => '0');
    wb_stb_o  <= '0';
    wb_cyc_o  <= '0';
    wb_lock_o <= '0';
    wb_tag_o  <= (others => '0');
  end generate;


  -- IO Access? -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  io_acc  <= '1' when (p_bus.addr(data_width_c-1 downto index_size_f(io_size_c)) = io_base_c(data_width_c-1 downto index_size_f(io_size_c))) else '0';
  io_rden <= io_acc and p_bus.re and (not p_bus.src); -- PMA: no_execute for IO region
  -- the default NEORV32 peripheral/IO devices in the IO area can only be written in word mode (reduces HW complexity)
  io_wren <= io_acc and p_bus.we and and_reduce_f(p_bus.ben) and (not p_bus.src); -- PMA: write32 only, no_execute for IO region


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
      rstn_i      => sys_rstn,                 -- global reset line, low-active, use as async
      addr_i      => p_bus.addr,               -- address
      rden_i      => io_rden,                  -- read enable
      wren_i      => io_wren,                  -- byte write enable
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
    cfs_cg_en <= '0';
    cfs_irq   <= '0';
    cfs_out_o <= (others => '0');
  end generate;


  -- General Purpose Input/Output Port (GPIO) -----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_gpio_inst_true:
  if (IO_GPIO_EN = true) generate
    neorv32_gpio_inst: neorv32_gpio
    port map (
      -- host access --
      clk_i  => clk_i,                     -- global clock line
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
  if (IO_GPIO_EN = false) generate
    resp_bus(RESP_GPIO) <= resp_bus_entry_terminate_c;
    gpio_o <= (others => '0');
  end generate;


  -- Watch Dog Timer (WDT) ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_wdt_inst_true:
  if (IO_WDT_EN = true) generate
    neorv32_wdt_inst: neorv32_wdt
    generic map(
      DEBUG_EN => ON_CHIP_DEBUGGER_EN -- CPU debug mode implemented?
    )
    port map (
      -- host access --
      clk_i       => clk_i,                    -- global clock line
      rstn_i      => ext_rstn,                 -- global reset line, low-active
      rden_i      => io_rden,                  -- read enable
      wren_i      => io_wren,                  -- write enable
      addr_i      => p_bus.addr,               -- address
      data_i      => p_bus.wdata,              -- data in
      data_o      => resp_bus(RESP_WDT).rdata, -- data out
      ack_o       => resp_bus(RESP_WDT).ack,   -- transfer acknowledge
      -- CPU in debug mode? --
      cpu_debug_i => debug_mode,
      -- clock generator --
      clkgen_en_o => wdt_cg_en,                -- enable clock generator
      clkgen_i    => clk_gen,
      -- timeout event --
      irq_o       => wdt_irq,                  -- timeout IRQ
      rstn_o      => wdt_rstn                  -- timeout reset, low_active, use it as async!
    );
    resp_bus(RESP_WDT).err <= '0'; -- no access error possible
  end generate;

  neorv32_wdt_inst_false:
  if (IO_WDT_EN = false) generate
    resp_bus(RESP_WDT) <= resp_bus_entry_terminate_c;
    wdt_irq   <= '0';
    wdt_rstn  <= '1';
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
      addr_i => p_bus.addr,                 -- address
      rden_i => io_rden,                    -- read enable
      wren_i => io_wren,                    -- write enable
      data_i => p_bus.wdata,                -- data in
      data_o => resp_bus(RESP_MTIME).rdata, -- data out
      ack_o  => resp_bus(RESP_MTIME).ack,   -- transfer acknowledge
      -- time output for CPU --
      time_o => mtime_time,                 -- current system time
      -- interrupt --
      irq_o  => mtime_irq                   -- interrupt request
    );
    resp_bus(RESP_MTIME).err <= '0'; -- no access error possible
  end generate;

  neorv32_mtime_inst_false:
  if (IO_MTIME_EN = false) generate
    resp_bus(RESP_MTIME) <= resp_bus_entry_terminate_c;
    mtime_time <= mtime_i; -- use external machine timer time signal
    mtime_irq  <= mtime_irq_i; -- use external machine timer interrupt
  end generate;


  -- system time output LO --
  mtime_sync: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- buffer low word one clock cycle to compensate for MTIME's 1-cycle delay
      -- when overflowing from low-word to high-word -> only relevant for processor-external devices
      -- processor-internal devices (= the CPU) do not care about this delay offset as 64-bit MTIME.TIME
      -- cannot be accessed within a single cycle
      if (IO_MTIME_EN = true) then
        mtime_o(31 downto 0) <= mtime_time(31 downto 0);
      else
        mtime_o(31 downto 0) <= (others => '0');
      end if;
    end if;
  end process mtime_sync;

  -- system time output HI --
  mtime_o(63 downto 32) <= mtime_time(63 downto 32) when (IO_MTIME_EN = true) else (others => '0');


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
      irq_rxd_o   => uart0_rxd_irq,              -- uart data received interrupt
      irq_txd_o   => uart0_txd_irq               -- uart transmission done interrupt
    );
    resp_bus(RESP_UART0).err <= '0'; -- no access error possible
  end generate;

  neorv32_uart0_inst_false:
  if (IO_UART0_EN = false) generate
    resp_bus(RESP_UART0) <= resp_bus_entry_terminate_c;
    uart0_txd_o   <= '0';
    uart0_rts_o   <= '0';
    uart0_cg_en   <= '0';
    uart0_rxd_irq <= '0';
    uart0_txd_irq <= '0';
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
      irq_rxd_o   => uart1_rxd_irq,              -- uart data received interrupt
      irq_txd_o   => uart1_txd_irq               -- uart transmission done interrupt
    );
    resp_bus(RESP_UART1).err <= '0'; -- no access error possible
  end generate;

  neorv32_uart1_inst_false:
  if (IO_UART1_EN = false) generate
    resp_bus(RESP_UART1) <= resp_bus_entry_terminate_c;
    uart1_txd_o   <= '0';
    uart1_rts_o   <= '0';
    uart1_cg_en   <= '0';
    uart1_rxd_irq <= '0';
    uart1_txd_irq <= '0';
  end generate;


  -- Serial Peripheral Interface (SPI) ------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_spi_inst_true:
  if (IO_SPI_EN = true) generate
    neorv32_spi_inst: neorv32_spi
    port map (
      -- host access --
      clk_i       => clk_i,                    -- global clock line
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
      spi_sck_o   => spi_sck_o,                -- SPI serial clock
      spi_sdo_o   => spi_sdo_o,                -- controller data out, peripheral data in
      spi_sdi_i   => spi_sdi_i,                -- controller data in, peripheral data out
      spi_csn_o   => spi_csn_o,                -- SPI CS
      -- interrupt --
      irq_o       => spi_irq                   -- transmission done interrupt
    );
    resp_bus(RESP_SPI).err <= '0'; -- no access error possible
  end generate;

  neorv32_spi_inst_false:
  if (IO_SPI_EN = false) generate
    resp_bus(RESP_SPI) <= resp_bus_entry_terminate_c;
    spi_sck_o <= '0';
    spi_sdo_o <= '0';
    spi_csn_o <= (others => '1'); -- CSn lines are low-active
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
      addr_i      => p_bus.addr,               -- address
      rden_i      => io_rden,                  -- read enable
      wren_i      => io_wren,                  -- write enable
      data_i      => p_bus.wdata,              -- data in
      data_o      => resp_bus(RESP_TWI).rdata, -- data out
      ack_o       => resp_bus(RESP_TWI).ack,   -- transfer acknowledge
      -- clock generator --
      clkgen_en_o => twi_cg_en,                -- enable clock generator
      clkgen_i    => clk_gen,
      -- com lines --
      twi_sda_io  => twi_sda_io,               -- serial data line
      twi_scl_io  => twi_scl_io,               -- serial clock line
      -- interrupt --
      irq_o       => twi_irq                   -- transfer done IRQ
    );
    resp_bus(RESP_TWI).err <= '0'; -- no access error possible
  end generate;

  neorv32_twi_inst_false:
  if (IO_TWI_EN = false) generate
    resp_bus(RESP_TWI) <= resp_bus_entry_terminate_c;
    twi_sda_io <= 'Z';
    twi_scl_io <= 'Z';
    twi_cg_en  <= '0';
    twi_irq    <= '0';
  end generate;


  -- Pulse-Width Modulation Controller (PWM) ------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_pwm_inst_true:
  if (IO_PWM_NUM_CH > 0) generate
    neorv32_pwm_inst: neorv32_pwm
    generic map (
      NUM_CHANNELS => IO_PWM_NUM_CH -- number of PWM channels (0..60)
    )
    port map (
      -- host access --
      clk_i       => clk_i,                    -- global clock line
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
    pwm_cg_en <= '0';
    pwm_o     <= (others => '0');
  end generate;


  -- True Random Number Generator (TRNG) ----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_trng_inst_true:
  if (IO_TRNG_EN = true) generate
    neorv32_trng_inst: neorv32_trng
    port map (
      -- host access --
      clk_i  => clk_i,                     -- global clock line
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
      FIFO_DEPTH => IO_NEOLED_TX_FIFO -- TX FIFO depth (1..32k, power of two)
    )
    port map (
      -- host access --
      clk_i       => clk_i,                       -- global clock line
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
    neoled_cg_en <= '0';
    neoled_irq   <= '0';
    neoled_o     <= '0';
  end generate;


  -- Stream Link Interface (SLINK) ----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_slink_inst_true:
  if (io_slink_en_c = true) generate
    neorv32_slink_inst: neorv32_slink
    generic map (
      SLINK_NUM_TX  => SLINK_NUM_TX,  -- number of TX links (0..8)
      SLINK_NUM_RX  => SLINK_NUM_RX,  -- number of TX links (0..8)
      SLINK_TX_FIFO => SLINK_TX_FIFO, -- TX fifo depth, has to be a power of two
      SLINK_RX_FIFO => SLINK_RX_FIFO  -- RX fifo depth, has to be a power of two
    )
    port map (
      -- host access --
      clk_i          => clk_i,                      -- global clock line
      addr_i         => p_bus.addr,                 -- address
      rden_i         => io_rden,                    -- read enable
      wren_i         => io_wren,                    -- write enable
      data_i         => p_bus.wdata,                -- data in
      data_o         => resp_bus(RESP_SLINK).rdata, -- data out
      ack_o          => resp_bus(RESP_SLINK).ack,   -- transfer acknowledge
      -- interrupt --
      irq_tx_o       => slink_tx_irq,               -- transmission done
      irq_rx_o       => slink_rx_irq,               -- data received
      -- TX stream interfaces --
      slink_tx_dat_o => slink_tx_dat_o,             -- output data
      slink_tx_val_o => slink_tx_val_o,             -- valid output
      slink_tx_rdy_i => slink_tx_rdy_i,             -- ready to send
      -- RX stream interfaces --
      slink_rx_dat_i => slink_rx_dat_i,             -- input data
      slink_rx_val_i => slink_rx_val_i,             -- valid input
      slink_rx_rdy_o => slink_rx_rdy_o              -- ready to receive
    );
    resp_bus(RESP_SLINK).err <= '0'; -- no access error possible
  end generate;

  neorv32_slink_inst_false:
  if (io_slink_en_c = false) generate
    resp_bus(RESP_SLINK) <= resp_bus_entry_terminate_c;
    slink_tx_irq   <= '0';
    slink_rx_irq   <= '0';
    slink_tx_dat_o <= (others => (others => '0'));
    slink_tx_val_o <= (others => '0');
    slink_rx_rdy_o <= (others => '0');
  end generate;


  -- External Interrupt Controller (XIRQ) ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_xirq_inst_true:
  if (XIRQ_NUM_CH > 0) generate
    neorv32_slink_inst: neorv32_xirq
    generic map (
      XIRQ_NUM_CH           => XIRQ_NUM_CH,          -- number of external IRQ channels (0..32)
      XIRQ_TRIGGER_TYPE     => XIRQ_TRIGGER_TYPE,    -- trigger type: 0=level, 1=edge
      XIRQ_TRIGGER_POLARITY => XIRQ_TRIGGER_POLARITY -- trigger polarity: 0=low-level/falling-edge, 1=high-level/rising-edge
    )
    port map (
      -- host access --
      clk_i     => clk_i,                     -- global clock line
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
    xirq_irq <= '0';
  end generate;


  -- General Purpose Timer (GPTMR) ----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_gptmr_inst_true:
  if (IO_GPTMR_EN = true) generate
    neorv32_gptmr_inst: neorv32_gptmr
    port map (
      -- host access --
      clk_i     => clk_i,                      -- global clock line
      addr_i    => p_bus.addr,                 -- address
      rden_i    => io_rden,                    -- read enable
      wren_i    => io_wren,                    -- write enable
      data_i    => p_bus.wdata,                -- data in
      data_o    => resp_bus(RESP_GPTMR).rdata, -- data out
      ack_o     => resp_bus(RESP_GPTMR).ack,   -- transfer acknowledge
      -- clock generator --
      clkgen_en_o => gptmr_cg_en,              -- enable clock generator
      clkgen_i    => clk_gen,
      -- interrupt --
      irq_o       => gptmr_irq                 -- transmission done interrupt
    );
    resp_bus(RESP_GPTMR).err <= '0'; -- no access error possible
  end generate;

  neorv32_gptmr_inst_false:
  if (IO_GPTMR_EN = false) generate
    resp_bus(RESP_GPTMR) <= resp_bus_entry_terminate_c;
    gptmr_cg_en          <= '0';
    gptmr_irq            <= '0';
  end generate;


  -- System Configuration Information Memory (SYSINFO) --------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_sysinfo_inst: neorv32_sysinfo
  generic map (
    -- General --
    CLOCK_FREQUENCY              => CLOCK_FREQUENCY,      -- clock frequency of clk_i in Hz
    INT_BOOTLOADER_EN            => INT_BOOTLOADER_EN,    -- implement processor-internal bootloader?
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_Zfinx    => CPU_EXTENSION_RISCV_Zfinx,    -- implement 32-bit floating-point extension (using INT reg!)
    CPU_EXTENSION_RISCV_Zicsr    => CPU_EXTENSION_RISCV_Zicsr,    -- implement CSR system?
    CPU_EXTENSION_RISCV_Zicntr   => CPU_EXTENSION_RISCV_Zicntr,   -- implement base counters?
    CPU_EXTENSION_RISCV_Zihpm    => CPU_EXTENSION_RISCV_Zihpm,    -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zifencei => CPU_EXTENSION_RISCV_Zifencei, -- implement instruction stream sync.?
    CPU_EXTENSION_RISCV_Zmmul    => CPU_EXTENSION_RISCV_Zmmul,    -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_DEBUG    => ON_CHIP_DEBUGGER_EN,          -- implement CPU debug mode?
    -- Extension Options --
    FAST_MUL_EN                  => FAST_MUL_EN,          -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                => FAST_SHIFT_EN,        -- use barrel shifter for shift operations
    CPU_CNT_WIDTH                => CPU_CNT_WIDTH,        -- total width of CPU cycle and instret counters (0..64)
    -- Physical memory protection (PMP) --
    PMP_NUM_REGIONS              => PMP_NUM_REGIONS,      -- number of regions (0..64)
    -- internal Instruction memory --
    MEM_INT_IMEM_EN              => MEM_INT_IMEM_EN,      -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            => MEM_INT_IMEM_SIZE,    -- size of processor-internal instruction memory in bytes
    -- Internal Data memory --
    MEM_INT_DMEM_EN              => MEM_INT_DMEM_EN,      -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            => MEM_INT_DMEM_SIZE,    -- size of processor-internal data memory in bytes
    -- Internal Cache memory --
    ICACHE_EN                    => ICACHE_EN,            -- implement instruction cache
    ICACHE_NUM_BLOCKS            => ICACHE_NUM_BLOCKS,    -- i-cache: number of blocks (min 2), has to be a power of 2
    ICACHE_BLOCK_SIZE            => ICACHE_BLOCK_SIZE,    -- i-cache: block size in bytes (min 4), has to be a power of 2
    ICACHE_ASSOCIATIVITY         => ICACHE_ASSOCIATIVITY, -- i-cache: associativity (min 1), has to be a power 2
    -- External memory interface --
    MEM_EXT_EN                   => MEM_EXT_EN,           -- implement external memory bus interface?
    MEM_EXT_BIG_ENDIAN           => MEM_EXT_BIG_ENDIAN,   -- byte order: true=big-endian, false=little-endian
    -- On-Chip Debugger --
    ON_CHIP_DEBUGGER_EN          => ON_CHIP_DEBUGGER_EN,  -- implement OCD?
    -- Processor peripherals --
    IO_GPIO_EN                   => IO_GPIO_EN,           -- implement general purpose input/output port unit (GPIO)?
    IO_MTIME_EN                  => IO_MTIME_EN,          -- implement machine system timer (MTIME)?
    IO_UART0_EN                  => IO_UART0_EN,          -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_UART1_EN                  => IO_UART1_EN,          -- implement secondary universal asynchronous receiver/transmitter (UART1)?
    IO_SPI_EN                    => IO_SPI_EN,            -- implement serial peripheral interface (SPI)?
    IO_TWI_EN                    => IO_TWI_EN,            -- implement two-wire interface (TWI)?
    IO_PWM_NUM_CH                => IO_PWM_NUM_CH,        -- number of PWM channels to implement
    IO_WDT_EN                    => IO_WDT_EN,            -- implement watch dog timer (WDT)?
    IO_TRNG_EN                   => IO_TRNG_EN,           -- implement true random number generator (TRNG)?
    IO_CFS_EN                    => IO_CFS_EN,            -- implement custom functions subsystem (CFS)?
    IO_SLINK_EN                  => io_slink_en_c,        -- implement stream link interface?
    IO_NEOLED_EN                 => IO_NEOLED_EN,         -- implement NeoPixel-compatible smart LED interface (NEOLED)?
    IO_XIRQ_NUM_CH               => XIRQ_NUM_CH,          -- number of external interrupt (XIRQ) channels to implement
    IO_GPTMR_EN                  => IO_GPTMR_EN           -- implement general purpose timer (GPTMR)?
  )
  port map (
    -- host access --
    clk_i  => clk_i,                        -- global clock line
    addr_i => p_bus.addr,                   -- address
    rden_i => io_rden,                      -- read enable
    data_o => resp_bus(RESP_SYSINFO).rdata, -- data out
    ack_o  => resp_bus(RESP_SYSINFO).ack    -- transfer acknowledge
  );

  resp_bus(RESP_SYSINFO).err <= '0'; -- no access error possible


  -- **************************************************************************************************************************
  -- On-Chip Debugger Complex
  -- **************************************************************************************************************************


  -- On-Chip Debugger - Debug Module (DM) ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_neorv32_debug_dm_true:
  if (ON_CHIP_DEBUGGER_EN = true) generate
    neorv32_debug_dm_inst: neorv32_debug_dm
    port map (
      -- global control --
      clk_i            => clk_i,                    -- global clock line
      rstn_i           => ext_rstn,                 -- external reset, low-active
      -- debug module interface (DMI) --
      dmi_rstn_i       => dmi.rstn,
      dmi_req_valid_i  => dmi.req_valid,
      dmi_req_ready_o  => dmi.req_ready,
      dmi_req_addr_i   => dmi.req_addr,
      dmi_req_op_i     => dmi.req_op,
      dmi_req_data_i   => dmi.req_data,
      dmi_resp_valid_o => dmi.resp_valid,           -- response valid when set
      dmi_resp_ready_i => dmi.resp_ready,           -- ready to receive respond
      dmi_resp_data_o  => dmi.resp_data,
      dmi_resp_err_o   => dmi.resp_err,             -- 0=ok, 1=error
      -- CPU bus access --
      cpu_addr_i       => p_bus.addr,               -- address
      cpu_rden_i       => p_bus.re,                 -- read enable
      cpu_wren_i       => p_bus.we,                 -- write enable
      cpu_data_i       => p_bus.wdata,              -- data in
      cpu_data_o       => resp_bus(RESP_OCD).rdata, -- data out
      cpu_ack_o        => resp_bus(RESP_OCD).ack,   -- transfer acknowledge
      -- CPU control --
      cpu_ndmrstn_o    => dci_ndmrstn,              -- soc reset
      cpu_halt_req_o   => dci_halt_req              -- request hart to halt (enter debug mode)
    );
    resp_bus(RESP_OCD).err <= '0'; -- no access error possible
  end generate;

  neorv32_debug_dm_false:
  if (ON_CHIP_DEBUGGER_EN = false) generate
    dmi.req_ready  <= '0';
    dmi.resp_valid <= '0';
    dmi.resp_data  <= (others => '0');
    dmi.resp_err   <= '0';
    --
    resp_bus(RESP_OCD) <= resp_bus_entry_terminate_c;
    dci_ndmrstn  <= '1';
    dci_halt_req <= '0';
  end generate;


  -- On-Chip Debugger - Debug Transport Module (DTM) ----------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_neorv32_debug_dtm_true:
  if (ON_CHIP_DEBUGGER_EN = true) generate
    neorv32_debug_dtm_inst: neorv32_debug_dtm
    generic map (
      IDCODE_VERSION => jtag_tap_idcode_version_c, -- version
      IDCODE_PARTID  => jtag_tap_idcode_partid_c,  -- part number
      IDCODE_MANID   => jtag_tap_idcode_manid_c    -- manufacturer id
    )
    port map (
      -- global control --
      clk_i            => clk_i,          -- global clock line
      rstn_i           => ext_rstn,       -- external reset, low-active
      -- jtag connection --
      jtag_trst_i      => jtag_trst_i,
      jtag_tck_i       => jtag_tck_i,
      jtag_tdi_i       => jtag_tdi_i,
      jtag_tdo_o       => jtag_tdo_o,
      jtag_tms_i       => jtag_tms_i,
      -- debug module interface (DMI) --
      dmi_rstn_o       => dmi.rstn,
      dmi_req_valid_o  => dmi.req_valid,
      dmi_req_ready_i  => dmi.req_ready,  -- DMI is allowed to make new requests when set
      dmi_req_addr_o   => dmi.req_addr,
      dmi_req_op_o     => dmi.req_op,     -- 0=read, 1=write
      dmi_req_data_o   => dmi.req_data,
      dmi_resp_valid_i => dmi.resp_valid, -- response valid when set
      dmi_resp_ready_o => dmi.resp_ready, -- ready to receive respond
      dmi_resp_data_i  => dmi.resp_data,
      dmi_resp_err_i   => dmi.resp_err    -- 0=ok, 1=error
    );
  end generate;

  neorv32_debug_dtm_false:
  if (ON_CHIP_DEBUGGER_EN = false) generate
    jtag_tdo_o <= jtag_tdi_i; -- feed-through
    --
    dmi.rstn       <= '0';
    dmi.req_valid  <= '0';
    dmi.req_addr   <= (others => '0');
    dmi.req_op     <= '0';
    dmi.req_data   <= (others => '0');
    dmi.resp_ready <= '0';
  end generate;


end neorv32_top_rtl;
