-- #################################################################################################
-- # << NEORV32 - Processor Top Qsys component with AvalonMM Compatible Master Interface >>        #
-- # ********************************************************************************************* #
-- # (c) "NIOS-2", "Qsys", "Platform Designer" and "AvalonMM" are trademarks of Intel.                         #
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

entity neorv32_qsys is
  generic (
    GUI_CLOCK_FREQUENCY       : integer := 100000000;
    GUI_EMABLE_INTERNAL_IMEM  : integer := 1;
    GUI_IMEM_SIZE             : integer := 16;
    GUI_EMABLE_INTERNAL_DMEM  : integer := 1;
    GUI_DMEM_SIZE             : integer := 8;
    GUI_ENABLE_BOOTLOADER     : integer := 0;
    GUI_ENABLE_AVALONMM       : integer := 1;
    GUI_ENABLE_UART0          : integer := 1;
    GUI_ENABLE_UART1          : integer := 0;
    GUI_ENABLE_GPIO           : integer := 0
  );
  port (
    -- Global control --
    clk_i       : in  std_logic := '0'; -- global clock, rising edge
    rstn_i      : in  std_logic := '0'; -- global reset, low-active, async
    -- GPIO --
    gpio_o      : out std_logic_vector(63 downto 0); -- parallel output
    gpio_i      : in std_logic_vector(63 downto 0) := (others => '0'); -- parallel output
    -- UART0 --
    uart0_txd_o : out std_logic; -- UART0 send data
    uart0_rxd_i : in  std_logic := '0'; -- UART0 receive data

    -- UART1 --
    uart1_txd_o : out std_logic; -- UART0 send data
    uart1_rxd_i : in  std_logic := '0'; -- UART0 receive data
    
    -- AvalonMM interface
    read                    : out std_logic;
    write                   : out std_logic;
    waitrequest             : in std_logic := '0';
    byteenable              : out std_logic_vector(3 downto 0);
    address                 : out std_logic_vector(31 downto 0);
    writedata               : out std_logic_vector(31 downto 0);
    readdata                : in std_logic_vector(31 downto 0) := (others => '0')

  );
end entity;

architecture neorv32_qsys_rtl of neorv32_qsys is

signal  gpio_i_ulogic : std_ulogic_vector(63 downto 0);
signal  gpio_o_ulogic : std_ulogic_vector(63 downto 0);

-- Wishbone bus interface (available if MEM_EXT_EN = true) --
signal  wb_tag_o    : std_ulogic_vector(02 downto 0); -- request tag
signal  wb_adr_o    : std_ulogic_vector(31 downto 0); -- address
signal  wb_dat_i    : std_ulogic_vector(31 downto 0); -- read data
signal  wb_dat_o    : std_ulogic_vector(31 downto 0); -- write data
signal  wb_we_o     : std_ulogic; -- read/write
signal  wb_sel_o    : std_ulogic_vector(03 downto 0); -- byte enable
signal  wb_stb_o    : std_ulogic; -- strobe
signal  wb_cyc_o    : std_ulogic; -- valid cycle
signal  wb_lock_o   : std_ulogic; -- exclusive access request
signal  wb_ack_i    : std_ulogic; -- transfer acknowledge
signal  wb_err_i    : std_ulogic; -- transfer error

signal  reset       : std_logic;

function integer2bool(integer_value : integer := 0) return boolean is
begin
  if integer_value = 0 then
    return false;
  else
    return true;
  end if;
end function;

begin

  -- The Core Of The Problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_top_inst: neorv32_top
  generic map (
    -- General --
    CLOCK_FREQUENCY              => GUI_CLOCK_FREQUENCY,   -- clock frequency of clk_i in Hz
    INT_BOOTLOADER_EN            => integer2bool(GUI_ENABLE_BOOTLOADER),        -- implement processor-internal bootloader?
    HW_THREAD_ID                 => 0,           -- hardware thread id (hartid)
    -- On-Chip Debugger (OCD) --
    ON_CHIP_DEBUGGER_EN          => false,       -- implement on-chip debugger
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        => false,       -- implement atomic extension?
    CPU_EXTENSION_RISCV_C        => true,        -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        => false,       -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        => true,        -- implement muld/div extension?
    CPU_EXTENSION_RISCV_U        => true,        -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    => false,       -- implement 32-bit floating-point extension (using INT reg!)
    CPU_EXTENSION_RISCV_Zicsr    => true,        -- implement CSR system?
    CPU_EXTENSION_RISCV_Zifencei => false,       -- implement instruction stream sync.?
    CPU_EXTENSION_RISCV_Zmmul    => false,  -- implement multiply-only M sub-extension?
    -- Extension Options --
    FAST_MUL_EN                  => false,       -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                => false,       -- use barrel shifter for shift operations
    CPU_CNT_WIDTH                => 64,          -- total width of CPU cycle and instret counters (0..64)
    CPU_IPB_ENTRIES              => 2,           -- entries is instruction prefetch buffer, has to be a power of 2
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS              => 0,           -- number of regions (0..64)
    PMP_MIN_GRANULARITY          => 64*1024,     -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 => 4,           -- number of implemented HPM counters (0..29)
    HPM_CNT_WIDTH                => 40,          -- total size of HPM counters (0..64)
    -- Internal Instruction memory --
    MEM_INT_IMEM_EN              => integer2bool(GUI_EMABLE_INTERNAL_IMEM),        -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            => GUI_IMEM_SIZE*1024,     -- size of processor-internal instruction memory in bytes
    -- Internal Data memory --
    MEM_INT_DMEM_EN              => integer2bool(GUI_EMABLE_INTERNAL_DMEM),        -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            => GUI_DMEM_SIZE*1024,      -- size of processor-internal data memory in bytes
    -- Internal Cache memory --
    ICACHE_EN                    => false,       -- implement instruction cache
    ICACHE_NUM_BLOCKS            => 4,           -- i-cache: number of blocks (min 1), has to be a power of 2
    ICACHE_BLOCK_SIZE            => 64,          -- i-cache: block size in bytes (min 4), has to be a power of 2
    ICACHE_ASSOCIATIVITY         => 1,           -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2
    -- External memory interface --
    MEM_EXT_EN                   => integer2bool(GUI_ENABLE_AVALONMM),       -- implement external memory bus interface?
    MEM_EXT_TIMEOUT              => 0,           -- cycles after a pending bus access auto-terminates (0 = disabled)
    MEM_EXT_PIPE_MODE            => false,       -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
    MEM_EXT_BIG_ENDIAN           => false,       -- byte order: true=big-endian, false=little-endian
    MEM_EXT_ASYNC_RX             => false,       -- use register buffer for RX data when false
    -- Stream link interface (SLINK) --
    SLINK_NUM_TX                 => 0,           -- number of TX links (0..8)
    SLINK_NUM_RX                 => 0,           -- number of TX links (0..8)
    SLINK_TX_FIFO                => 1,           -- TX fifo depth, has to be a power of two
    SLINK_RX_FIFO                => 1,           -- RX fifo depth, has to be a power of two
    -- External Interrupts Controller (XIRQ) --
    XIRQ_NUM_CH                  => 0,      -- number of external IRQ channels (0..32)
    XIRQ_TRIGGER_TYPE            => (x"FFFFFFFF"), -- trigger type: 0=level, 1=edge
    XIRQ_TRIGGER_POLARITY        => (x"FFFFFFFF"), -- trigger polarity: 0=low-level/falling-edge, 1=high-level/rising-edge
    -- Processor peripherals --
    IO_GPIO_EN                   => integer2bool(GUI_ENABLE_GPIO),        -- implement general purpose input/output port unit (GPIO)?
    IO_MTIME_EN                  => true,        -- implement machine system timer (MTIME)?
    IO_UART0_EN                  => integer2bool(GUI_ENABLE_UART0),        -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_UART1_EN                  => integer2bool(GUI_ENABLE_UART1),        -- implement secondary universal asynchronous receiver/transmitter (UART1)?
    IO_SPI_EN                    => false,       -- implement serial peripheral interface (SPI)?
    IO_TWI_EN                    => false,       -- implement two-wire interface (TWI)?
    IO_PWM_NUM_CH                => 0,           -- number of PWM channels to implement (0..60); 0 = disabled
    IO_WDT_EN                    => true,        -- implement watch dog timer (WDT)?
    IO_TRNG_EN                   => false,       -- implement true random number generator (TRNG)?
    IO_CFS_EN                    => false,       -- implement custom functions subsystem (CFS)?
    IO_CFS_CONFIG                => x"00000000", -- custom CFS configuration generic
    IO_CFS_IN_SIZE               => 32,          -- size of CFS input conduit in bits
    IO_CFS_OUT_SIZE              => 32,          -- size of CFS output conduit in bits
    IO_NEOLED_EN                 => false,       -- implement NeoPixel-compatible smart LED interface (NEOLED)?
    IO_NEOLED_TX_FIFO            => 1            -- NEOLED TX FIFO depth, 1..32k, has to be a power of two
  )  
  port map (
    -- Global control --
    clk_i       => clk_i,           -- global clock, rising edge
    rstn_i      => rstn_i,          -- global reset, low-active, async
    -- JTAG on-chip debugger interface (available if ON_CHIP_DEBUGGER_EN = true) --
    jtag_trst_i => '0',             -- low-active TAP reset (optional)
    jtag_tck_i  => '0',             -- serial clock
    jtag_tdi_i  => '0',             -- serial data input
    jtag_tdo_o  => open,            -- serial data output
    jtag_tms_i  => '0',             -- mode select
    -- Wishbone bus interface (available if MEM_EXT_EN = true) --
    wb_tag_o    => wb_tag_o,        -- tag
    wb_adr_o    => wb_adr_o,        -- address
    wb_dat_i    => wb_dat_i,        -- read data
    wb_dat_o    => wb_dat_o,        -- write data
    wb_we_o     => wb_we_o,         -- read/write
    wb_sel_o    => wb_sel_o,        -- byte enable
    wb_stb_o    => wb_stb_o,        -- strobe
    wb_cyc_o    => wb_cyc_o,        -- valid cycle
    wb_lock_o   => wb_lock_o,       -- exclusive access request
    wb_ack_i    => wb_ack_i,        -- transfer acknowledge
    wb_err_i    => wb_err_i,        -- transfer error
    -- Advanced memory control signals (available if MEM_EXT_EN = true) --
    fence_o     => open,            -- indicates an executed FENCE operation
    fencei_o    => open,            -- indicates an executed FENCEI operation
    -- TX stream interfaces (available if SLINK_NUM_TX > 0) --
    slink_tx_dat_o => open,         -- output data
    slink_tx_val_o => open,         -- valid output
    slink_tx_rdy_i => (others => 'L'), -- ready to send
    -- RX stream interfaces (available if SLINK_NUM_RX > 0) --
    slink_rx_dat_i => (others => (others => 'U')), -- input data
    slink_rx_val_i => (others => 'L'), -- valid input
    slink_rx_rdy_o => open,         -- ready to receive

    -- GPIO (available if IO_GPIO_EN = true) --
    gpio_o      => gpio_o_ulogic,   -- parallel output
    gpio_i      => gpio_i_ulogic,   -- parallel input
    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o => uart0_txd_o,     -- UART0 send data
    uart0_rxd_i => uart0_rxd_i,     -- UART0 receive data
    uart0_rts_o => open,            -- hw flow control: UART0.RX ready to receive ("RTR"), low-active, optional
    uart0_cts_i => '0',             -- hw flow control: UART0.TX allowed to transmit, low-active, optional
    -- secondary UART1 (available if IO_UART1_EN = true) --
    uart1_txd_o => uart1_txd_o,     -- UART1 send data
    uart1_rxd_i => uart1_rxd_i,     -- UART1 receive data
    uart1_rts_o => open,            -- hw flow control: UART1.RX ready to receive ("RTR"), low-active, optional
    uart1_cts_i => '0',             -- hw flow control: UART1.TX allowed to transmit, low-active, optional
    -- SPI (available if IO_SPI_EN = true) --
    spi_sck_o   => open,            -- SPI serial clock
    spi_sdo_o   => open,            -- controller data out, peripheral data in
    spi_sdi_i   => '0',             -- controller data in, peripheral data out
    spi_csn_o   => open,            -- SPI CS
    -- TWI (available if IO_TWI_EN = true) --
    twi_sda_io  => open,            -- twi serial data line
    twi_scl_io  => open,            -- twi serial clock line
    -- PWM (available if IO_PWM_NUM_CH > 0) --
    pwm_o       => open,            -- pwm channels
    -- Custom Functions Subsystem IO --
    cfs_in_i    => (others => '0'), -- custom inputs
    cfs_out_o   => open,            -- custom outputs
    -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
    neoled_o    => open,            -- async serial data line
    -- System time --
    mtime_i     => (others => '0'), -- current system time from ext. MTIME (if IO_MTIME_EN = false)
    mtime_o     => open,            -- current system time from int. MTIME (if IO_MTIME_EN = true)
    -- External platform interrupts (available if XIRQ_NUM_CH > 0) --
    xirq_i      => (others => '0'), -- IRQ channels
    -- Interrupts --
    mtime_irq_i => '0',             -- machine timer interrupt, available if IO_MTIME_EN = false
    msw_irq_i   => '0',             -- machine software interrupt
    mext_irq_i  => '0'              -- machine external interrupt
  );

    -- Convert between std_logic / std_ulogic
  gpio_o <= std_logic_vector(gpio_o_ulogic);
  gpio_i_ulogic <= std_ulogic_vector(gpio_i);

  reset <= not(rstn_i);

  -- Wishbone to AvalonMM brdige
  read <= '1' when (wb_stb_o = '1' and wb_we_o = '0') else '0';
  write <= '1' when (wb_stb_o = '1' and wb_we_o = '1') else '0';
  address <= std_logic_vector(wb_adr_o);
  writedata <= std_logic_vector(wb_dat_o);
  byteenable <= std_logic_vector(wb_sel_o);

  wb_dat_i <= std_ulogic_vector(readdata);
  wb_ack_i <= not(waitrequest);
  wb_err_i <= '0';

end architecture;
