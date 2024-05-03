-- ================================================================================ --
-- NEORV32 - Processor Top Entity with AXI4-Lite & AXI4-Stream Compatible Interface --
-- -------------------------------------------------------------------------------- --
-- Dedicated for IP packaging/integration using AMD Vivado.                         --
-- Use the provided TCL script to automatically package this as IP module:          --
-- > source neorv32_vivado_ip.tcl                                                   --
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

entity neorv32_vivado_ip is
  generic (
    -- ------------------------------------------------------------
    -- Configuration Generics
    -- ------------------------------------------------------------
    -- AXI-Stream Interfaces --
    AXI4_STREAM_EN             : boolean                        := false;
    -- General --
    CLOCK_FREQUENCY            : natural                        := 0;
    HART_ID                    : std_ulogic_vector(31 downto 0) := x"00000000";
    JEDEC_ID                   : std_ulogic_vector(10 downto 0) := "00000000000";
    INT_BOOTLOADER_EN          : boolean                        := false;
    -- On-Chip Debugger (OCD) --
    ON_CHIP_DEBUGGER_EN        : boolean                        := false;
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A      : boolean                        := false;
    CPU_EXTENSION_RISCV_B      : boolean                        := false;
    CPU_EXTENSION_RISCV_C      : boolean                        := false;
    CPU_EXTENSION_RISCV_E      : boolean                        := false;
    CPU_EXTENSION_RISCV_M      : boolean                        := false;
    CPU_EXTENSION_RISCV_U      : boolean                        := false;
    CPU_EXTENSION_RISCV_Zfinx  : boolean                        := false;
    CPU_EXTENSION_RISCV_Zicntr : boolean                        := false;
    CPU_EXTENSION_RISCV_Zicond : boolean                        := false;
    CPU_EXTENSION_RISCV_Zihpm  : boolean                        := false;
    CPU_EXTENSION_RISCV_Zmmul  : boolean                        := false;
    CPU_EXTENSION_RISCV_Zxcfu  : boolean                        := false;
    -- Tuning Options --
    FAST_MUL_EN                : boolean                        := false;
    FAST_SHIFT_EN              : boolean                        := false;
    REGFILE_HW_RST             : boolean                        := false;
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS            : natural range 0 to 16          := 0;
    PMP_MIN_GRANULARITY        : natural                        := 4;
    PMP_TOR_MODE_EN            : boolean                        := false;
    PMP_NAP_MODE_EN            : boolean                        := false;
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS               : natural range 0 to 13          := 0;
    HPM_CNT_WIDTH              : natural range 0 to 64          := 40;
    -- Atomic Memory Access - Reservation Set Granularity --
    AMO_RVS_GRANULARITY        : natural                        := 4;
    -- Internal Instruction memory --
    MEM_INT_IMEM_EN            : boolean                        := false;
    MEM_INT_IMEM_SIZE          : natural                        := 16*1024;
    -- Internal Data memory --
    MEM_INT_DMEM_EN            : boolean                        := false;
    MEM_INT_DMEM_SIZE          : natural                        := 8*1024;
    -- Internal Cache memory --
    ICACHE_EN                  : boolean                        := false;
    ICACHE_NUM_BLOCKS          : natural range 1 to 256         := 4;
    ICACHE_BLOCK_SIZE          : natural range 4 to 2**16       := 64;
    -- Internal Data Cache (dCACHE) --
    DCACHE_EN                  : boolean                        := false;
    DCACHE_NUM_BLOCKS          : natural range 1 to 256         := 4;
    DCACHE_BLOCK_SIZE          : natural range 4 to 2**16       := 64;
    -- External Bus Interface --
    XBUS_TIMEOUT               : natural range 8 to 65536       := 64;
    XBUS_CACHE_EN              : boolean                        := false;
    XBUS_CACHE_NUM_BLOCKS      : natural range 1 to 256         := 8;
    XBUS_CACHE_BLOCK_SIZE      : natural range 1 to 2**16       := 256;
    -- Execute in-place module (XIP) --
    XIP_EN                     : boolean                        := false;
    XIP_CACHE_EN               : boolean                        := false;
    XIP_CACHE_NUM_BLOCKS       : natural range 1 to 256         := 8;
    XIP_CACHE_BLOCK_SIZE       : natural range 1 to 2**16       := 256;
    -- External Interrupts Controller (XIRQ) --
    XIRQ_NUM_CH                : natural                        := 0;
    XIRQ_TRIGGER_TYPE          : std_ulogic_vector(31 downto 0) := x"ffffffff";
    XIRQ_TRIGGER_POLARITY      : std_ulogic_vector(31 downto 0) := x"ffffffff";
    -- Processor peripherals --
    IO_GPIO_NUM                : natural range 0 to 64          := 0;
    IO_MTIME_EN                : boolean                        := false;
    IO_UART0_EN                : boolean                        := false;
    IO_UART0_RX_FIFO           : natural range 1 to 2**15       := 1;
    IO_UART0_TX_FIFO           : natural range 1 to 2**15       := 1;
    IO_UART1_EN                : boolean                        := false;
    IO_UART1_RX_FIFO           : natural range 1 to 2**15       := 1;
    IO_UART1_TX_FIFO           : natural range 1 to 2**15       := 1;
    IO_SPI_EN                  : boolean                        := false;
    IO_SPI_FIFO                : natural range 1 to 2**15       := 1;
    IO_SDI_EN                  : boolean                        := false;
    IO_SDI_FIFO                : natural range 1 to 2**15       := 1;
    IO_TWI_EN                  : boolean                        := false;
    IO_TWI_FIFO                : natural range 1 to 2**15       := 1;
    IO_PWM_NUM_CH              : natural range 0 to 12          := 0;
    IO_WDT_EN                  : boolean                        := false;
    IO_TRNG_EN                 : boolean                        := false;
    IO_TRNG_FIFO               : natural range 1 to 2**15       := 1;
    IO_CFS_EN                  : boolean                        := false;
    IO_CFS_CONFIG              : std_ulogic_vector(31 downto 0) := x"00000000";
    IO_CFS_IN_SIZE             : natural                        := 32;
    IO_CFS_OUT_SIZE            : natural                        := 32;
    IO_NEOLED_EN               : boolean                        := false;
    IO_NEOLED_TX_FIFO          : natural range 1 to 2**15       := 1;
    IO_GPTMR_EN                : boolean                        := false;
    IO_ONEWIRE_EN              : boolean                        := false;
    IO_DMA_EN                  : boolean                        := false;
    IO_SLINK_RX_FIFO           : natural range 1 to 2**15       := 1;
    IO_SLINK_TX_FIFO           : natural range 1 to 2**15       := 1;
    IO_CRC_EN                  : boolean                        := false
  );
  port (
    -- ------------------------------------------------------------
    -- Global Control
    -- ------------------------------------------------------------
    clk            : in  std_ulogic;
    resetn         : in  std_ulogic; -- low-active
    -- ------------------------------------------------------------
    -- AXI4-Lite-Compatible Host Interface (always available)
    -- ------------------------------------------------------------
    -- Clock and Reset --
--  m_axi_aclk     : in  std_ulogic := '0'; -- just to satisfy Vivado, but not actually used!
--  m_axi_aresetn  : in  std_ulogic := '0'; -- just to satisfy Vivado, but not actually used!
    -- Write Address Channel --
    m_axi_awaddr   : out std_ulogic_vector(31 downto 0);
    m_axi_awprot   : out std_ulogic_vector(2 downto 0);
    m_axi_awvalid  : out std_ulogic;
    m_axi_awready  : in  std_ulogic := '0';
    -- Write Data Channel --
    m_axi_wdata    : out std_ulogic_vector(31 downto 0);
    m_axi_wstrb    : out std_ulogic_vector(3 downto 0);
    m_axi_wvalid   : out std_ulogic;
    m_axi_wready   : in  std_ulogic := '0';
    -- Read Address Channel --
    m_axi_araddr   : out std_ulogic_vector(31 downto 0);
    m_axi_arprot   : out std_ulogic_vector(2 downto 0);
    m_axi_arvalid  : out std_ulogic;
    m_axi_arready  : in  std_ulogic := '0';
    -- Read Data Channel --
    m_axi_rdata    : in  std_ulogic_vector(31 downto 0) := x"00000000";
    m_axi_rresp    : in  std_ulogic_vector(1 downto 0) := "00";
    m_axi_rvalid   : in  std_ulogic := '0';
    m_axi_rready   : out std_ulogic;
    -- Write Response Channel --
    m_axi_bresp    : in  std_ulogic_vector(1 downto 0) := "00";
    m_axi_bvalid   : in  std_ulogic := '0';
    m_axi_bready   : out std_ulogic;
    -- ------------------------------------------------------------
    -- AXI4-Stream-Compatible Interfaces (available if AXI4_STREAM_EN = true)
    -- ------------------------------------------------------------
    -- Source --
--  s0_axis_aclk   : in  std_ulogic := '0'; -- just to satisfy Vivado, but not actually used!
    s0_axis_tvalid : out std_ulogic;
    s0_axis_tready : in  std_ulogic := '0';
    s0_axis_tdata  : out std_ulogic_vector(31 downto 0);
    s0_axis_tlast  : out std_ulogic;
    -- Sink --
--  s1_axis_aclk   : in  std_ulogic := '0'; -- just to satisfy Vivado, but not actually used!
    s1_axis_tvalid : in  std_ulogic := '0';
    s1_axis_tready : out std_ulogic;
    s1_axis_tdata  : in  std_ulogic_vector(31 downto 0) := x"00000000";
    s1_axis_tlast  : in  std_ulogic := '0';
    -- ------------------------------------------------------------
    -- JTAG on-chip debugger interface (available if ON_CHIP_DEBUGGER_EN = true)
    -- ------------------------------------------------------------
    jtag_trst_i    : in  std_ulogic := '0';
    jtag_tck_i     : in  std_ulogic := '0';
    jtag_tdi_i     : in  std_ulogic := '0';
    jtag_tdo_o     : out std_ulogic := '0';
    jtag_tms_i     : in  std_ulogic := '0';
    -- ------------------------------------------------------------
    -- Processor IO
    -- ------------------------------------------------------------
    -- XIP (execute in place via SPI) signals (available if IO_XIP_EN = true) --
    xip_csn_o      : out std_ulogic;
    xip_clk_o      : out std_ulogic;
    xip_dat_i      : in  std_ulogic := '0';
    xip_dat_o      : out std_ulogic;
    -- GPIO (available if IO_GPIO_NUM > 0) --
    gpio_o         : out std_ulogic_vector(63 downto 0);
    gpio_i         : in  std_ulogic_vector(63 downto 0) := x"0000000000000000";
    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o    : out std_ulogic;
    uart0_rxd_i    : in  std_ulogic := '0';
    uart0_rts_o    : out std_ulogic;
    uart0_cts_i    : in  std_ulogic := '0';
    -- secondary UART1 (available if IO_UART1_EN = true) --
    uart1_txd_o    : out std_ulogic;
    uart1_rxd_i    : in  std_ulogic := '0';
    uart1_rts_o    : out std_ulogic;
    uart1_cts_i    : in  std_ulogic := '0';
    -- SPI (available if IO_SPI_EN = true) --
    spi_clk_o      : out std_ulogic;
    spi_dat_o      : out std_ulogic;
    spi_dat_i      : in  std_ulogic := '0';
    spi_csn_o      : out std_ulogic_vector(7 downto 0); -- SPI CS
    -- SDI (available if IO_SDI_EN = true) --
    sdi_clk_i      : in  std_ulogic := '0';
    sdi_dat_o      : out std_ulogic;
    sdi_dat_i      : in  std_ulogic := '0';
    sdi_csn_i      : in  std_ulogic := '0';
    -- TWI (available if IO_TWI_EN = true) --
    twi_sda_i      : in  std_ulogic := '0';
    twi_sda_o      : out std_ulogic;
    twi_scl_i      : in  std_ulogic := '0';
    twi_scl_o      : out std_ulogic;
    -- 1-Wire Interface (available if IO_ONEWIRE_EN = true) --
    onewire_i      : in  std_ulogic := '0';
    onewire_o      : out std_ulogic;
    -- PWM (available if IO_PWM_NUM_CH > 0) --
    pwm_o          : out std_ulogic_vector(11 downto 0);
    -- Custom Functions Subsystem IO (available if IO_CFS_EN = true) --
    cfs_in_i       : in  std_ulogic_vector(IO_CFS_IN_SIZE-1  downto 0) := (others => '0');
    cfs_out_o      : out std_ulogic_vector(IO_CFS_OUT_SIZE-1 downto 0);
    -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
    neoled_o       : out std_ulogic;
    -- Machine timer system time (available if IO_MTIME_EN = true) --
    mtime_time_o   : out std_ulogic_vector(63 downto 0);
    -- GPTMR timer capture (available if IO_GPTMR_EN = true) --
    gptmr_trig_i   : in  std_ulogic := '0';
    -- External platform interrupts (available if XIRQ_NUM_CH > 0) --
    xirq_i         : in  std_ulogic_vector(31 downto 0) := x"00000000";
    -- CPU Interrupts --
    mtime_irq_i    : in  std_ulogic := '0';
    msw_irq_i      : in  std_ulogic := '0';
    mext_irq_i     : in  std_ulogic := '0'
  );
end entity;

architecture neorv32_vivado_ip_rtl of neorv32_vivado_ip is

  -- internal wishbone bus --
  type wb_bus_t is record
    adr : std_ulogic_vector(31 downto 0);
    di  : std_ulogic_vector(31 downto 0);
    do  : std_ulogic_vector(31 downto 0);
    we  : std_ulogic;
    sel : std_ulogic_vector(03 downto 0);
    cyc : std_ulogic;
    ack : std_ulogic;
    err : std_ulogic;
  end record;
  signal wb_core : wb_bus_t;

  -- AXI bridge control --
  type axi_ctrl_t is record
    radr_received, wadr_received, wdat_received : std_ulogic;
  end record;
  signal axi_ctrl : axi_ctrl_t;

begin

  -- The Core Of The Problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_top_inst: neorv32_top
  generic map (
    -- General --
    CLOCK_FREQUENCY            => CLOCK_FREQUENCY,
    CLOCK_GATING_EN            => false, -- clock gating is not supported here
    HART_ID                    => HART_ID,
    JEDEC_ID                   => JEDEC_ID,
    INT_BOOTLOADER_EN          => INT_BOOTLOADER_EN,
    -- On-Chip Debugger --
    ON_CHIP_DEBUGGER_EN        => ON_CHIP_DEBUGGER_EN,
    DM_LEGACY_MODE             => false,
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
    -- Extension Options --
    FAST_MUL_EN                => FAST_MUL_EN,
    FAST_SHIFT_EN              => FAST_SHIFT_EN,
    REGFILE_HW_RST             => REGFILE_HW_RST,
    -- Physical Memory Protection --
    PMP_NUM_REGIONS            => PMP_NUM_REGIONS,
    PMP_MIN_GRANULARITY        => PMP_MIN_GRANULARITY,
    PMP_TOR_MODE_EN            => PMP_TOR_MODE_EN,
    PMP_NAP_MODE_EN            => PMP_NAP_MODE_EN,
    -- Hardware Performance Monitors --
    HPM_NUM_CNTS               => HPM_NUM_CNTS,
    HPM_CNT_WIDTH              => HPM_CNT_WIDTH,
    -- Atomic Memory Access - Reservation Set Granularity --
    AMO_RVS_GRANULARITY        => AMO_RVS_GRANULARITY,
    -- Internal Instruction memory --
    MEM_INT_IMEM_EN            => MEM_INT_IMEM_EN,
    MEM_INT_IMEM_SIZE          => MEM_INT_IMEM_SIZE,
    -- Internal Data memory --
    MEM_INT_DMEM_EN            => MEM_INT_DMEM_EN,
    MEM_INT_DMEM_SIZE          => MEM_INT_DMEM_SIZE,
    -- Internal Cache memory --
    ICACHE_EN                  => ICACHE_EN,
    ICACHE_NUM_BLOCKS          => ICACHE_NUM_BLOCKS,
    ICACHE_BLOCK_SIZE          => ICACHE_BLOCK_SIZE,
    -- Internal Data Cache (dCACHE) --
    DCACHE_EN                  => DCACHE_EN,
    DCACHE_NUM_BLOCKS          => DCACHE_NUM_BLOCKS,
    DCACHE_BLOCK_SIZE          => DCACHE_BLOCK_SIZE,
    -- External bus interface --
    XBUS_EN                    => true,
    XBUS_TIMEOUT               => XBUS_TIMEOUT,
    XBUS_REGSTAGE_EN           => false,
    XBUS_CACHE_EN              => XBUS_CACHE_EN,
    XBUS_CACHE_NUM_BLOCKS      => XBUS_CACHE_NUM_BLOCKS,
    XBUS_CACHE_BLOCK_SIZE      => XBUS_CACHE_BLOCK_SIZE,
    -- Execute in-place module --
    XIP_EN                     => XIP_EN,
    XIP_CACHE_EN               => XIP_CACHE_EN,
    XIP_CACHE_NUM_BLOCKS       => XIP_CACHE_NUM_BLOCKS,
    XIP_CACHE_BLOCK_SIZE       => XIP_CACHE_BLOCK_SIZE,
    -- External Interrupts Controller --
    XIRQ_NUM_CH                => XIRQ_NUM_CH,
    XIRQ_TRIGGER_TYPE          => XIRQ_TRIGGER_TYPE,
    XIRQ_TRIGGER_POLARITY      => XIRQ_TRIGGER_POLARITY,
    -- Processor peripherals --
    IO_GPIO_NUM                => IO_GPIO_NUM,
    IO_MTIME_EN                => IO_MTIME_EN,
    IO_UART0_EN                => IO_UART0_EN,
    IO_UART0_RX_FIFO           => IO_UART0_RX_FIFO,
    IO_UART0_TX_FIFO           => IO_UART0_TX_FIFO,
    IO_UART1_EN                => IO_UART1_EN,
    IO_UART1_RX_FIFO           => IO_UART1_RX_FIFO,
    IO_UART1_TX_FIFO           => IO_UART1_TX_FIFO,
    IO_SPI_EN                  => IO_SPI_EN,
    IO_SPI_FIFO                => IO_SPI_FIFO,
    IO_SDI_EN                  => IO_SDI_EN,
    IO_SDI_FIFO                => IO_SDI_FIFO,
    IO_TWI_EN                  => IO_TWI_EN,
    IO_TWI_FIFO                => IO_TWI_FIFO,
    IO_PWM_NUM_CH              => IO_PWM_NUM_CH,
    IO_WDT_EN                  => IO_WDT_EN,
    IO_TRNG_EN                 => IO_TRNG_EN,
    IO_TRNG_FIFO               => IO_TRNG_FIFO,
    IO_CFS_EN                  => IO_CFS_EN,
    IO_CFS_CONFIG              => IO_CFS_CONFIG,
    IO_CFS_IN_SIZE             => IO_CFS_IN_SIZE,
    IO_CFS_OUT_SIZE            => IO_CFS_OUT_SIZE,
    IO_NEOLED_EN               => IO_NEOLED_EN,
    IO_NEOLED_TX_FIFO          => IO_NEOLED_TX_FIFO,
    IO_GPTMR_EN                => IO_GPTMR_EN,
    IO_ONEWIRE_EN              => IO_ONEWIRE_EN,
    IO_DMA_EN                  => IO_DMA_EN,
    IO_SLINK_EN                => AXI4_STREAM_EN,
    IO_SLINK_RX_FIFO           => IO_SLINK_RX_FIFO,
    IO_SLINK_TX_FIFO           => IO_SLINK_TX_FIFO,
    IO_CRC_EN                  => IO_CRC_EN
  )
  port map (
    -- Global control --
    clk_i          => clk,
    rstn_i         => resetn,
    -- JTAG on-chip debugger interface (available if ON_CHIP_DEBUGGER_EN = true) --
    jtag_trst_i    => jtag_trst_i,
    jtag_tck_i     => jtag_tck_i,
    jtag_tdi_i     => jtag_tdi_i,
    jtag_tdo_o     => jtag_tdo_o,
    jtag_tms_i     => jtag_tms_i,
    -- External bus interface (available if XBUS_EN = true) --
    xbus_adr_o     => wb_core.adr,
    xbus_dat_i     => wb_core.di,
    xbus_dat_o     => wb_core.do,
    xbus_we_o      => wb_core.we,
    xbus_sel_o     => wb_core.sel,
    xbus_stb_o     => open,
    xbus_cyc_o     => wb_core.cyc,
    xbus_ack_i     => wb_core.ack,
    xbus_err_i     => wb_core.err,
    -- Stream Link Interface (available if IO_SLINK_EN = true) --
    slink_rx_dat_i => s1_axis_tdata,
    slink_rx_val_i => s1_axis_tvalid,
    slink_rx_lst_i => s1_axis_tlast,
    slink_rx_rdy_o => s1_axis_tready,
    slink_tx_dat_o => s0_axis_tdata,
    slink_tx_val_o => s0_axis_tvalid,
    slink_tx_lst_o => s0_axis_tlast,
    slink_tx_rdy_i => s0_axis_tready,
    -- XIP (execute in place via SPI) signals (available if IO_XIP_EN = true) --
    xip_csn_o      => xip_csn_o,
    xip_clk_o      => xip_clk_o,
    xip_dat_i      => xip_dat_i,
    xip_dat_o      => xip_dat_o,
    -- GPIO (available if IO_GPIO_NUM > 0) --
    gpio_o         => gpio_o,
    gpio_i         => gpio_i,
    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o    => uart0_txd_o,
    uart0_rxd_i    => uart0_rxd_i,
    uart0_rts_o    => uart0_rts_o,
    uart0_cts_i    => uart0_cts_i,
    -- secondary UART1 (available if IO_UART1_EN = true) --
    uart1_txd_o    => uart1_txd_o,
    uart1_rxd_i    => uart1_rxd_i,
    uart1_rts_o    => uart1_rts_o,
    uart1_cts_i    => uart1_cts_i,
    -- SPI (available if IO_SPI_EN = true) --
    spi_clk_o      => spi_clk_o,
    spi_dat_o      => spi_dat_o,
    spi_dat_i      => spi_dat_i,
    spi_csn_o      => spi_csn_o,
    -- TWI (available if IO_TWI_EN = true) --
    twi_sda_i      => twi_sda_i,
    twi_sda_o      => twi_sda_o,
    twi_scl_i      => twi_scl_i,
    twi_scl_o      => twi_scl_o,
    -- 1-Wire Interface (available if IO_ONEWIRE_EN = true) --
    onewire_i      => onewire_i,
    onewire_o      => onewire_o,
    -- PWM available if IO_PWM_NUM_CH > 0) --
    pwm_o          => pwm_o,
    -- Custom Functions Subsystem IO (available if IO_CFS_EN = true) --
    cfs_in_i       => cfs_in_i,
    cfs_out_o      => cfs_out_o,
    -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
    neoled_o       => neoled_o,
    -- Machine timer system time (available if IO_MTIME_EN = true) --
    mtime_time_o   => mtime_time_o,
    -- GPTMR timer capture (available if IO_GPTMR_EN = true) --
    gptmr_trig_i   => gptmr_trig_i,
    -- External platform interrupts (available if XIRQ_NUM_CH > 0) --
    xirq_i         => xirq_i,
    -- CPU Interrupts --
    mtime_irq_i    => mtime_irq_i,
    msw_irq_i      => msw_irq_i,
    mext_irq_i     => mext_irq_i
  );


  -- Wishbone to AXI4-Lite Bridge -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  axi_arbiter: process(resetn, clk)
  begin
    if (resetn = '0') then
      axi_ctrl.radr_received <= '0';
      axi_ctrl.wadr_received <= '0';
      axi_ctrl.wdat_received <= '0';
    elsif rising_edge(clk) then
      if (wb_core.cyc = '0') then -- idle
        axi_ctrl.radr_received <= '0';
        axi_ctrl.wadr_received <= '0';
        axi_ctrl.wdat_received <= '0';
      else -- busy
        -- "read address received" flag --
        if (wb_core.we = '0') then -- pending READ
          if (m_axi_arready = '1') then -- read address received by interconnect?
            axi_ctrl.radr_received <= '1';
          end if;
        end if;
        -- "write address received" flag --
        if (wb_core.we = '1') then -- pending WRITE
          if (m_axi_awready = '1') then -- write address received by interconnect?
            axi_ctrl.wadr_received <= '1';
          end if;
        end if;
        -- "write data received" flag --
        if (wb_core.we = '1') then -- pending WRITE
          if (m_axi_wready = '1') then -- write data received by interconnect?
            axi_ctrl.wdat_received <= '1';
          end if;
        end if;
      end if;
    end if;
  end process axi_arbiter;


  -- read address channel --
  m_axi_araddr  <= wb_core.adr;
  m_axi_arvalid <= wb_core.cyc and (not wb_core.we) and (not axi_ctrl.radr_received);
  m_axi_arprot  <= "000";

  -- read data channel --
  m_axi_rready  <= wb_core.cyc and (not wb_core.we);
  wb_core.di    <= m_axi_rdata;

  -- write address channel --
  m_axi_awaddr  <= wb_core.adr;
  m_axi_awvalid <= wb_core.cyc and wb_core.we and (not axi_ctrl.wadr_received);
  m_axi_awprot  <= "000";

  -- write data channel --
  m_axi_wdata   <= wb_core.do;
  m_axi_wvalid  <= wb_core.cyc and wb_core.we and (not axi_ctrl.wdat_received);
  m_axi_wstrb   <= wb_core.sel;

  -- write response channel --
  m_axi_bready  <= wb_core.cyc and wb_core.we;


  -- read/write response --
  axi_response: process(wb_core, m_axi_bvalid, m_axi_bresp, m_axi_rvalid, m_axi_rresp)
  begin
    wb_core.ack <= '0'; -- default
    wb_core.err <= '0'; -- default
      if (wb_core.cyc = '1') then -- bus operation in progress
        if (wb_core.we = '1') then -- write operation
          if (m_axi_bvalid = '1') then -- valid response
            if (m_axi_bresp = "00") then -- status check
              wb_core.ack <= '1'; -- OK
            else
              wb_core.err <= '1'; -- ERROR!
            end if;
          end if;
        else -- read operation
          if (m_axi_rvalid = '1') then -- valid response
            if (m_axi_rresp = "00") then -- status check
              wb_core.ack <= '1'; -- OK
            else
              wb_core.err <= '1'; -- ERROR!
            end if;
          end if;
        end if;
      end if;
  end process axi_response;


end architecture neorv32_vivado_ip_rtl;
