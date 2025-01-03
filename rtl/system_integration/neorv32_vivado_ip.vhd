-- ================================================================================ --
-- NEORV32 - Processor Top Entity with AXI4-Lite & AXI4-Stream Compatible Interface --
-- -------------------------------------------------------------------------------- --
-- Dedicated for IP packaging/integration using AMD Vivado.                         --
-- Use the provided TCL script to automatically package this as IP module:          --
-- Vivado TCL console: > source neorv32_vivado_ip.tcl                               --
-- See the NEORV32 Datasheet and User Guide for more information.                   --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
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
    -- Clocking --
    CLOCK_FREQUENCY       : natural                       := 100_000_000;
    -- Identification --
    JEDEC_ID              : std_logic_vector(10 downto 0) := "00000000000";
    -- Boot Configuration --
    BOOT_MODE_SELECT      : natural range 0 to 2          := 0;
    BOOT_ADDR_CUSTOM      : std_ulogic_vector(31 downto 0) := x"00000000";
    -- On-Chip Debugger (OCD) --
    OCD_EN                : boolean                       := false;
    OCD_AUTHENTICATION    : boolean                       := false;
    -- RISC-V CPU Extensions --
    RISCV_ISA_C           : boolean                       := false;
    RISCV_ISA_E           : boolean                       := false;
    RISCV_ISA_M           : boolean                       := false;
    RISCV_ISA_U           : boolean                       := false;
    RISCV_ISA_Zaamo       : boolean                       := false;
    RISCV_ISA_Zba         : boolean                       := false;
    RISCV_ISA_Zbb         : boolean                       := false;
    RISCV_ISA_Zbkb        : boolean                       := false;
    RISCV_ISA_Zbkc        : boolean                       := false;
    RISCV_ISA_Zbkx        : boolean                       := false;
    RISCV_ISA_Zbs         : boolean                       := false;
    RISCV_ISA_Zfinx       : boolean                       := false;
    RISCV_ISA_Zicntr      : boolean                       := false;
    RISCV_ISA_Zicond      : boolean                       := false;
    RISCV_ISA_Zihpm       : boolean                       := false;
    RISCV_ISA_Zmmul       : boolean                       := false;
    RISCV_ISA_Zknd        : boolean                       := false;
    RISCV_ISA_Zkne        : boolean                       := false;
    RISCV_ISA_Zknh        : boolean                       := false;
    RISCV_ISA_Zksed       : boolean                       := false;
    RISCV_ISA_Zksh        : boolean                       := false;
    RISCV_ISA_Zxcfu       : boolean                       := false;
    -- Tuning Options --
    CPU_FAST_MUL_EN       : boolean                       := false;
    CPU_FAST_SHIFT_EN     : boolean                       := false;
    CPU_RF_HW_RST_EN      : boolean                       := false;
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS       : natural range 0 to 16         := 0;
    PMP_MIN_GRANULARITY   : natural                       := 4;
    PMP_TOR_MODE_EN       : boolean                       := false;
    PMP_NAP_MODE_EN       : boolean                       := false;
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS          : natural range 0 to 13         := 0;
    HPM_CNT_WIDTH         : natural range 0 to 64         := 40;
    -- Internal Instruction memory --
    MEM_INT_IMEM_EN       : boolean                       := false;
    MEM_INT_IMEM_SIZE     : natural                       := 16384;
    -- Internal Data memory --
    MEM_INT_DMEM_EN       : boolean                       := false;
    MEM_INT_DMEM_SIZE     : natural                       := 8192;
    -- Internal Cache memory --
    ICACHE_EN             : boolean                       := false;
    ICACHE_NUM_BLOCKS     : natural range 1 to 256        := 4;
    ICACHE_BLOCK_SIZE     : natural range 4 to 2**16      := 64;
    -- Internal Data Cache (dCACHE) --
    DCACHE_EN             : boolean                       := false;
    DCACHE_NUM_BLOCKS     : natural range 1 to 256        := 4;
    DCACHE_BLOCK_SIZE     : natural range 4 to 2**16      := 64;
    -- External Bus Interface --
    XBUS_EN               : boolean                       := true;
    XBUS_TIMEOUT          : natural range 8 to 65536      := 64;
    XBUS_REGSTAGE_EN      : boolean                       := false;
    XBUS_CACHE_EN         : boolean                       := false;
    XBUS_CACHE_NUM_BLOCKS : natural range 1 to 256        := 8;
    XBUS_CACHE_BLOCK_SIZE : natural range 1 to 2**16      := 256;
    -- Execute in-place module (XIP) --
    XIP_EN                : boolean                       := false;
    XIP_CACHE_EN          : boolean                       := false;
    XIP_CACHE_NUM_BLOCKS  : natural range 1 to 256        := 8;
    XIP_CACHE_BLOCK_SIZE  : natural range 1 to 2**16      := 256;
    -- External Interrupts Controller (XIRQ) --
    XIRQ_EN               : boolean                       := false;
    XIRQ_NUM_CH           : natural range 1 to 32         := 1; -- variable-sized ports must be at least 0 downto 0; #974
    -- Processor peripherals --
    IO_GPIO_EN            : boolean                       := false;
    IO_GPIO_IN_NUM        : natural range 1 to 64         := 1; -- variable-sized ports must be at least 0 downto 0; #974
    IO_GPIO_OUT_NUM       : natural range 1 to 64         := 1;
    IO_CLINT_EN           : boolean                       := false;
    IO_UART0_EN           : boolean                       := false;
    IO_UART0_RX_FIFO      : natural range 1 to 2**15      := 1;
    IO_UART0_TX_FIFO      : natural range 1 to 2**15      := 1;
    IO_UART1_EN           : boolean                       := false;
    IO_UART1_RX_FIFO      : natural range 1 to 2**15      := 1;
    IO_UART1_TX_FIFO      : natural range 1 to 2**15      := 1;
    IO_SPI_EN             : boolean                       := false;
    IO_SPI_FIFO           : natural range 1 to 2**15      := 1;
    IO_SDI_EN             : boolean                       := false;
    IO_SDI_FIFO           : natural range 1 to 2**15      := 1;
    IO_TWI_EN             : boolean                       := false;
    IO_TWI_FIFO           : natural range 1 to 2**15      := 1;
    IO_TWD_EN             : boolean                       := false;
    IO_TWD_FIFO           : natural range 1 to 2**15      := 1;
    IO_PWM_EN             : boolean                       := false;
    IO_PWM_NUM_CH         : natural range 1 to 16         := 1; -- variable-sized ports must be at least 0 downto 0; #974
    IO_WDT_EN             : boolean                       := false;
    IO_TRNG_EN            : boolean                       := false;
    IO_TRNG_FIFO          : natural range 1 to 2**15      := 1;
    IO_CFS_EN             : boolean                       := false;
    IO_CFS_CONFIG         : std_logic_vector(31 downto 0) := x"00000000";
    IO_CFS_IN_SIZE        : natural range 1 to 4096       := 32; -- variable-sized ports must be at least 0 downto 0; #974
    IO_CFS_OUT_SIZE       : natural range 1 to 4096       := 32; -- variable-sized ports must be at least 0 downto 0; #974
    IO_NEOLED_EN          : boolean                       := false;
    IO_NEOLED_TX_FIFO     : natural range 1 to 2**15      := 1;
    IO_GPTMR_EN           : boolean                       := false;
    IO_ONEWIRE_EN         : boolean                       := false;
    IO_DMA_EN             : boolean                       := false;
    IO_SLINK_EN           : boolean                        := false;
    IO_SLINK_RX_FIFO      : natural range 1 to 2**15      := 1;
    IO_SLINK_TX_FIFO      : natural range 1 to 2**15      := 1;
    IO_CRC_EN             : boolean                       := false
  );
  port (
    -- ------------------------------------------------------------
    -- Global Control
    -- ------------------------------------------------------------
    clk            : in  std_logic;
    resetn         : in  std_logic; -- low-active
    -- ------------------------------------------------------------
    -- AXI4-Lite Host Interface (available if XBUS_EN = true)
    -- ------------------------------------------------------------
    -- Clock and Reset --
--  m_axi_aclk     : in  std_logic := '0'; -- just to satisfy Vivado, but not actually used
--  m_axi_aresetn  : in  std_logic := '0'; -- just to satisfy Vivado, but not actually used
    -- Write Address Channel --
    m_axi_awaddr   : out std_logic_vector(31 downto 0);
    m_axi_awprot   : out std_logic_vector(2 downto 0);
    m_axi_awvalid  : out std_logic;
    m_axi_awready  : in  std_logic := '0';
    -- Write Data Channel --
    m_axi_wdata    : out std_logic_vector(31 downto 0);
    m_axi_wstrb    : out std_logic_vector(3 downto 0);
    m_axi_wvalid   : out std_logic;
    m_axi_wready   : in  std_logic := '0';
    -- Read Address Channel --
    m_axi_araddr   : out std_logic_vector(31 downto 0);
    m_axi_arprot   : out std_logic_vector(2 downto 0);
    m_axi_arvalid  : out std_logic;
    m_axi_arready  : in  std_logic := '0';
    -- Read Data Channel --
    m_axi_rdata    : in  std_logic_vector(31 downto 0) := x"00000000";
    m_axi_rresp    : in  std_logic_vector(1 downto 0); -- no default here (#1067)
    m_axi_rvalid   : in  std_logic := '0';
    m_axi_rready   : out std_logic;
    -- Write Response Channel --
    m_axi_bresp    : in  std_logic_vector(1 downto 0); -- no default here (#1067)
    m_axi_bvalid   : in  std_logic := '0';
    m_axi_bready   : out std_logic;
    -- ------------------------------------------------------------
    -- AXI4-Stream Interfaces (available if IO_SLINK_EN = true)
    -- ------------------------------------------------------------
    -- Source --
--  s0_axis_aclk   : in  std_logic := '0'; -- just to satisfy Vivado, but not actually used
    s0_axis_tdest  : out std_logic_vector(3 downto 0);
    s0_axis_tvalid : out std_logic;
    s0_axis_tready : in  std_logic := '0';
    s0_axis_tdata  : out std_logic_vector(31 downto 0);
    s0_axis_tlast  : out std_logic;
    -- Sink --
--  s1_axis_aclk   : in  std_logic := '0'; -- just to satisfy Vivado, but not actually used
    s1_axis_tid    : in  std_logic_vector(3 downto 0) := x"0";
    s1_axis_tvalid : in  std_logic := '0';
    s1_axis_tready : out std_logic;
    s1_axis_tdata  : in  std_logic_vector(31 downto 0) := x"00000000";
    s1_axis_tlast  : in  std_logic := '0';
    -- ------------------------------------------------------------
    -- JTAG on-chip debugger interface (available if OCD_EN = true)
    -- ------------------------------------------------------------
    jtag_tck_i     : in  std_logic := '0';
    jtag_tdi_i     : in  std_logic := '0';
    jtag_tdo_o     : out std_logic := '0';
    jtag_tms_i     : in  std_logic := '0';
    -- ------------------------------------------------------------
    -- Processor IO
    -- ------------------------------------------------------------
    -- XIP (execute in place via SPI) signals (available if IO_XIP_EN = true) --
    xip_csn_o      : out std_logic;
    xip_clk_o      : out std_logic;
    xip_dat_i      : in  std_logic := '0';
    xip_dat_o      : out std_logic;
    -- GPIO (available if IO_GPIO_IN/OUT_NUM > 0) --
    gpio_o         : out std_logic_vector(IO_GPIO_OUT_NUM-1 downto 0); -- variable-sized ports must be at least 0 downto 0; #974
    gpio_i         : in  std_logic_vector(IO_GPIO_IN_NUM-1 downto 0) := (others => '0'); -- variable-sized ports must be at least 0 downto 0; #974
    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o    : out std_logic;
    uart0_rxd_i    : in  std_logic := '0';
    uart0_rts_o    : out std_logic;
    uart0_cts_i    : in  std_logic := '0';
    -- secondary UART1 (available if IO_UART1_EN = true) --
    uart1_txd_o    : out std_logic;
    uart1_rxd_i    : in  std_logic := '0';
    uart1_rts_o    : out std_logic;
    uart1_cts_i    : in  std_logic := '0';
    -- SPI (available if IO_SPI_EN = true) --
    spi_clk_o      : out std_logic;
    spi_dat_o      : out std_logic;
    spi_dat_i      : in  std_logic := '0';
    spi_csn_o      : out std_logic_vector(7 downto 0); -- SPI CS
    -- SDI (available if IO_SDI_EN = true) --
    sdi_clk_i      : in  std_logic := '0';
    sdi_dat_o      : out std_logic;
    sdi_dat_i      : in  std_logic := '0';
    sdi_csn_i      : in  std_logic := '0';
    -- TWI (available if IO_TWI_EN = true) --
    twi_sda_i      : in  std_logic := '0';
    twi_sda_o      : out std_logic;
    twi_scl_i      : in  std_logic := '0';
    twi_scl_o      : out std_logic;
    -- TWD (available if IO_TWD_EN = true) --
    twd_sda_i      : in  std_logic := '0';
    twd_sda_o      : out std_logic;
    twd_scl_i      : in  std_logic := '0';
    twd_scl_o      : out std_logic;
    -- 1-Wire Interface (available if IO_ONEWIRE_EN = true) --
    onewire_i      : in  std_logic := '0';
    onewire_o      : out std_logic;
    -- PWM (available if IO_PWM_NUM_CH > 0) --
    pwm_o          : out std_logic_vector(IO_PWM_NUM_CH-1 downto 0); -- variable-sized ports must be at least 0 downto 0; #974
    -- Custom Functions Subsystem IO (available if IO_CFS_EN = true) --
    cfs_in_i       : in  std_logic_vector(IO_CFS_IN_SIZE-1 downto 0) := (others => '0'); -- variable-sized ports must be at least 0 downto 0; #974
    cfs_out_o      : out std_logic_vector(IO_CFS_OUT_SIZE-1 downto 0); -- variable-sized ports must be at least 0 downto 0; #974
    -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
    neoled_o       : out std_logic;
    -- Machine timer system time (available if IO_CLINT_EN = true) --
    mtime_time_o   : out std_logic_vector(63 downto 0);
    -- External platform interrupts (available if XIRQ_NUM_CH > 0) --
    xirq_i         : in  std_logic_vector(XIRQ_NUM_CH-1 downto 0) := (others => '0'); -- variable-sized ports must be at least 0 downto 0; #974
    -- CPU Interrupts --
    mtime_irq_i    : in  std_logic := '0';
    msw_irq_i      : in  std_logic := '0';
    mext_irq_i     : in  std_logic := '0'
  );
end entity;

architecture neorv32_vivado_ip_rtl of neorv32_vivado_ip is

  -- auto-configuration --
  constant num_gpio_c : natural := cond_sel_natural_f(IO_GPIO_EN, max_natural_f(IO_GPIO_IN_NUM, IO_GPIO_OUT_NUM), 0);
  constant num_xirq_c : natural := cond_sel_natural_f(XIRQ_EN, XIRQ_NUM_CH, 0);
  constant num_pwm_c  : natural := cond_sel_natural_f(IO_PWM_EN, IO_PWM_NUM_CH, 0);

  -- AXI4-Lite bridge --
  component xbus2axi4lite_bridge
    port (
      -- Global control
      clk           : in  std_logic;
      resetn        : in  std_logic;
      -- XBUS device interface --
      xbus_adr_i    : in  std_ulogic_vector(31 downto 0);
      xbus_dat_i    : in  std_ulogic_vector(31 downto 0);
      xbus_tag_i    : in  std_ulogic_vector(2 downto 0);
      xbus_we_i     : in  std_ulogic;
      xbus_sel_i    : in  std_ulogic_vector(3 downto 0);
      xbus_stb_i    : in  std_ulogic;
      xbus_cyc_i    : in  std_ulogic;
      xbus_ack_o    : out std_ulogic;
      xbus_err_o    : out std_ulogic;
      xbus_dat_o    : out std_ulogic_vector(31 downto 0);
      -- AXI4-Lite host write address channel --
      m_axi_awaddr  : out std_logic_vector(31 downto 0);
      m_axi_awprot  : out std_logic_vector(2 downto 0);
      m_axi_awvalid : out std_logic;
      m_axi_awready : in  std_logic;
      -- AXI4-Lite host write data channel --
      m_axi_wdata   : out std_logic_vector(31 downto 0);
      m_axi_wstrb   : out std_logic_vector(3 downto 0);
      m_axi_wvalid  : out std_logic;
      m_axi_wready  : in  std_logic;
      -- AXI4-Lite host read address channel --
      m_axi_araddr  : out std_logic_vector(31 downto 0);
      m_axi_arprot  : out std_logic_vector(2 downto 0);
      m_axi_arvalid : out std_logic;
      m_axi_arready : in  std_logic;
      -- AXI4-Lite host read data channel --
      m_axi_rdata   : in  std_logic_vector(31 downto 0);
      m_axi_rresp   : in  std_logic_vector(1 downto 0);
      m_axi_rvalid  : in  std_logic;
      m_axi_rready  : out std_logic;
      -- AXI4-Lite host write response channel --
      m_axi_bresp   : in  std_logic_vector(1 downto 0);
      m_axi_bvalid  : in  std_logic;
      m_axi_bready  : out std_logic
    );
  end component;

  -- type conversion --
  signal jtag_tdo_aux : std_ulogic;
  signal s0_axis_tdata_aux : std_ulogic_vector(31 downto 0);
  signal s0_axis_tdest_aux : std_ulogic_vector(3 downto 0);
  signal s1_axis_tready_aux, s0_axis_tvalid_aux, s0_axis_tlast_aux : std_ulogic;
  signal xip_csn_aux, xip_clk_aux, xip_do_aux : std_ulogic;
  signal uart0_txd_aux, uart0_rts_aux, uart1_txd_aux, uart1_rts_aux : std_ulogic;
  signal spi_clk_aux, spi_do_aux : std_ulogic;
  signal spi_csn_aux : std_ulogic_vector(7 downto 0);
  signal sdi_do_aux : std_ulogic;
  signal twi_sda_o_aux, twi_scl_o_aux : std_ulogic;
  signal twd_sda_o_aux, twd_scl_o_aux : std_ulogic;
  signal onewire_o_aux : std_ulogic;
  signal cfs_out_aux : std_ulogic_vector(IO_CFS_OUT_SIZE-1 downto 0);
  signal neoled_aux : std_ulogic;
  signal mtime_time_aux : std_ulogic_vector(63 downto 0);

  -- constrained size ports --
  signal gpio_o_aux : std_ulogic_vector(63 downto 0);
  signal gpio_i_aux : std_ulogic_vector(63 downto 0);
  signal pwm_o_aux  : std_ulogic_vector(15 downto 0);
  signal xirq_i_aux : std_ulogic_vector(31 downto 0);

  -- internal wishbone bus --
  signal xbus_adr : std_ulogic_vector(31 downto 0); -- address
  signal xbus_do  : std_ulogic_vector(31 downto 0); -- write data
  signal xbus_tag : std_ulogic_vector(2 downto 0);  -- access tag
  signal xbus_we  : std_ulogic;                     -- read/write
  signal xbus_sel : std_ulogic_vector(3 downto 0);  -- byte enable
  signal xbus_stb : std_ulogic;                     -- strobe
  signal xbus_cyc : std_ulogic;                     -- valid cycle
  signal xbus_di  : std_ulogic_vector(31 downto 0); -- read data
  signal xbus_ack : std_ulogic;                     -- transfer acknowledge
  signal xbus_err : std_ulogic;                     -- transfer error

begin

  -- The Core Of The Problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_top_inst: neorv32_top
  generic map (
    -- Clocking --
    CLOCK_FREQUENCY       => CLOCK_FREQUENCY,
    -- Identification --
    JEDEC_ID              => std_ulogic_vector(JEDEC_ID),
    -- Boot Configuration --
    BOOT_MODE_SELECT      => BOOT_MODE_SELECT,
    BOOT_ADDR_CUSTOM      => BOOT_ADDR_CUSTOM,
    -- On-Chip Debugger --
    OCD_EN                => OCD_EN,
    OCD_AUTHENTICATION    => OCD_AUTHENTICATION,
    -- RISC-V CPU Extensions --
    RISCV_ISA_C           => RISCV_ISA_C,
    RISCV_ISA_E           => RISCV_ISA_E,
    RISCV_ISA_M           => RISCV_ISA_M,
    RISCV_ISA_U           => RISCV_ISA_U,
    RISCV_ISA_Zaamo       => RISCV_ISA_Zaamo,
    RISCV_ISA_Zba         => RISCV_ISA_Zba,
    RISCV_ISA_Zbb         => RISCV_ISA_Zbb,
    RISCV_ISA_Zbkb        => RISCV_ISA_Zbkb,
    RISCV_ISA_Zbkc        => RISCV_ISA_Zbkc,
    RISCV_ISA_Zbkx        => RISCV_ISA_Zbkx,
    RISCV_ISA_Zbs         => RISCV_ISA_Zbs,
    RISCV_ISA_Zfinx       => RISCV_ISA_Zfinx,
    RISCV_ISA_Zicntr      => RISCV_ISA_Zicntr,
    RISCV_ISA_Zicond      => RISCV_ISA_Zicond,
    RISCV_ISA_Zihpm       => RISCV_ISA_Zihpm,
    RISCV_ISA_Zmmul       => RISCV_ISA_Zmmul,
    RISCV_ISA_Zknd        => RISCV_ISA_Zknd,
    RISCV_ISA_Zkne        => RISCV_ISA_Zkne,
    RISCV_ISA_Zknh        => RISCV_ISA_Zknh,
    RISCV_ISA_Zksed       => RISCV_ISA_Zksed,
    RISCV_ISA_Zksh        => RISCV_ISA_Zksh,
    RISCV_ISA_Zxcfu       => RISCV_ISA_Zxcfu,
    -- Extension Options --
    CPU_CLOCK_GATING_EN   => false, -- clock gating is not supported here
    CPU_FAST_MUL_EN       => CPU_FAST_MUL_EN,
    CPU_FAST_SHIFT_EN     => CPU_FAST_SHIFT_EN,
    CPU_RF_HW_RST_EN      => CPU_RF_HW_RST_EN,
    -- Physical Memory Protection --
    PMP_NUM_REGIONS       => PMP_NUM_REGIONS,
    PMP_MIN_GRANULARITY   => PMP_MIN_GRANULARITY,
    PMP_TOR_MODE_EN       => PMP_TOR_MODE_EN,
    PMP_NAP_MODE_EN       => PMP_NAP_MODE_EN,
    -- Hardware Performance Monitors --
    HPM_NUM_CNTS          => HPM_NUM_CNTS,
    HPM_CNT_WIDTH         => HPM_CNT_WIDTH,
    -- Internal Instruction memory --
    MEM_INT_IMEM_EN       => MEM_INT_IMEM_EN,
    MEM_INT_IMEM_SIZE     => MEM_INT_IMEM_SIZE,
    -- Internal Data memory --
    MEM_INT_DMEM_EN       => MEM_INT_DMEM_EN,
    MEM_INT_DMEM_SIZE     => MEM_INT_DMEM_SIZE,
    -- Internal Cache memory --
    ICACHE_EN             => ICACHE_EN,
    ICACHE_NUM_BLOCKS     => ICACHE_NUM_BLOCKS,
    ICACHE_BLOCK_SIZE     => ICACHE_BLOCK_SIZE,
    -- Internal Data Cache (dCACHE) --
    DCACHE_EN             => DCACHE_EN,
    DCACHE_NUM_BLOCKS     => DCACHE_NUM_BLOCKS,
    DCACHE_BLOCK_SIZE     => DCACHE_BLOCK_SIZE,
    -- External bus interface --
    XBUS_EN               => XBUS_EN,
    XBUS_TIMEOUT          => XBUS_TIMEOUT,
    XBUS_REGSTAGE_EN      => XBUS_REGSTAGE_EN,
    XBUS_CACHE_EN         => XBUS_CACHE_EN,
    XBUS_CACHE_NUM_BLOCKS => XBUS_CACHE_NUM_BLOCKS,
    XBUS_CACHE_BLOCK_SIZE => XBUS_CACHE_BLOCK_SIZE,
    -- Execute in-place module --
    XIP_EN                => XIP_EN,
    XIP_CACHE_EN          => XIP_CACHE_EN,
    XIP_CACHE_NUM_BLOCKS  => XIP_CACHE_NUM_BLOCKS,
    XIP_CACHE_BLOCK_SIZE  => XIP_CACHE_BLOCK_SIZE,
    -- External Interrupts Controller --
    XIRQ_NUM_CH           => num_xirq_c,
    -- Processor peripherals --
    IO_DISABLE_SYSINFO    => false,
    IO_GPIO_NUM           => num_gpio_c,
    IO_CLINT_EN           => IO_CLINT_EN,
    IO_UART0_EN           => IO_UART0_EN,
    IO_UART0_RX_FIFO      => IO_UART0_RX_FIFO,
    IO_UART0_TX_FIFO      => IO_UART0_TX_FIFO,
    IO_UART1_EN           => IO_UART1_EN,
    IO_UART1_RX_FIFO      => IO_UART1_RX_FIFO,
    IO_UART1_TX_FIFO      => IO_UART1_TX_FIFO,
    IO_SPI_EN             => IO_SPI_EN,
    IO_SPI_FIFO           => IO_SPI_FIFO,
    IO_SDI_EN             => IO_SDI_EN,
    IO_SDI_FIFO           => IO_SDI_FIFO,
    IO_TWI_EN             => IO_TWI_EN,
    IO_TWI_FIFO           => IO_TWI_FIFO,
    IO_TWD_EN             => IO_TWD_EN,
    IO_TWD_FIFO           => IO_TWD_FIFO,
    IO_PWM_NUM_CH         => num_pwm_c,
    IO_WDT_EN             => IO_WDT_EN,
    IO_TRNG_EN            => IO_TRNG_EN,
    IO_TRNG_FIFO          => IO_TRNG_FIFO,
    IO_CFS_EN             => IO_CFS_EN,
    IO_CFS_CONFIG         => std_ulogic_vector(IO_CFS_CONFIG),
    IO_CFS_IN_SIZE        => IO_CFS_IN_SIZE,
    IO_CFS_OUT_SIZE       => IO_CFS_OUT_SIZE,
    IO_NEOLED_EN          => IO_NEOLED_EN,
    IO_NEOLED_TX_FIFO     => IO_NEOLED_TX_FIFO,
    IO_GPTMR_EN           => IO_GPTMR_EN,
    IO_ONEWIRE_EN         => IO_ONEWIRE_EN,
    IO_DMA_EN             => IO_DMA_EN,
    IO_SLINK_EN           => IO_SLINK_EN,
    IO_SLINK_RX_FIFO      => IO_SLINK_RX_FIFO,
    IO_SLINK_TX_FIFO      => IO_SLINK_TX_FIFO,
    IO_CRC_EN             => IO_CRC_EN
  )
  port map (
    -- Global control --
    clk_i          => std_ulogic(clk),
    rstn_i         => std_ulogic(resetn),
    -- JTAG on-chip debugger interface (available if OCD_EN = true) --
    jtag_tck_i     => std_ulogic(jtag_tck_i),
    jtag_tdi_i     => std_ulogic(jtag_tdi_i),
    jtag_tdo_o     => jtag_tdo_aux,
    jtag_tms_i     => std_ulogic(jtag_tms_i),
    -- External bus interface (available if XBUS_EN = true) --
    xbus_adr_o     => xbus_adr,
    xbus_dat_o     => xbus_do,
    xbus_tag_o     => xbus_tag,
    xbus_we_o      => xbus_we,
    xbus_sel_o     => xbus_sel,
    xbus_stb_o     => xbus_stb,
    xbus_cyc_o     => xbus_cyc,
    xbus_dat_i     => xbus_di,
    xbus_ack_i     => xbus_ack,
    xbus_err_i     => xbus_err,
    -- Stream Link Interface (available if IO_SLINK_EN = true) --
    slink_rx_dat_i => std_ulogic_vector(s1_axis_tdata),
    slink_rx_src_i => std_ulogic_vector(s1_axis_tid),
    slink_rx_val_i => std_ulogic(s1_axis_tvalid),
    slink_rx_lst_i => std_ulogic(s1_axis_tlast),
    slink_rx_rdy_o => s1_axis_tready_aux,
    slink_tx_dat_o => s0_axis_tdata_aux,
    slink_tx_dst_o => s0_axis_tdest_aux,
    slink_tx_val_o => s0_axis_tvalid_aux,
    slink_tx_lst_o => s0_axis_tlast_aux,
    slink_tx_rdy_i => std_ulogic(s0_axis_tready),
    -- XIP (execute in place via SPI) signals (available if IO_XIP_EN = true) --
    xip_csn_o      => xip_csn_aux,
    xip_clk_o      => xip_clk_aux,
    xip_dat_i      => std_ulogic(xip_dat_i),
    xip_dat_o      => xip_do_aux,
    -- GPIO (available if IO_GPIO_NUM > 0) --
    gpio_o         => gpio_o_aux,
    gpio_i         => gpio_i_aux,
    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o    => uart0_txd_aux,
    uart0_rxd_i    => std_ulogic(uart0_rxd_i),
    uart0_rts_o    => uart0_rts_aux,
    uart0_cts_i    => std_ulogic(uart0_cts_i),
    -- secondary UART1 (available if IO_UART1_EN = true) --
    uart1_txd_o    => uart1_txd_aux,
    uart1_rxd_i    => std_ulogic(uart1_rxd_i),
    uart1_rts_o    => uart1_rts_aux,
    uart1_cts_i    => std_ulogic(uart1_cts_i),
    -- SPI (available if IO_SPI_EN = true) --
    spi_clk_o      => spi_clk_aux,
    spi_dat_o      => spi_do_aux,
    spi_dat_i      => std_ulogic(spi_dat_i),
    spi_csn_o      => spi_csn_aux,
    -- SDI (available if IO_SDI_EN = true) --
    sdi_clk_i      => std_ulogic(sdi_clk_i),
    sdi_dat_o      => sdi_do_aux,
    sdi_dat_i      => std_ulogic(sdi_dat_i),
    sdi_csn_i      => std_ulogic(sdi_csn_i),
    -- TWI (available if IO_TWI_EN = true) --
    twi_sda_i      => std_ulogic(twi_sda_i),
    twi_sda_o      => twi_sda_o_aux,
    twi_scl_i      => std_ulogic(twi_scl_i),
    twi_scl_o      => twi_scl_o_aux,
    -- TWD (available if IO_TWD_EN = true) --
    twd_sda_i      => std_ulogic(twd_sda_i),
    twd_sda_o      => twd_sda_o_aux,
    twd_scl_i      => std_ulogic(twd_scl_i),
    twd_scl_o      => twd_scl_o_aux,
    -- 1-Wire Interface (available if IO_ONEWIRE_EN = true) --
    onewire_i      => std_ulogic(onewire_i),
    onewire_o      => onewire_o_aux,
    -- PWM available if IO_PWM_NUM_CH > 0) --
    pwm_o          => pwm_o_aux,
    -- Custom Functions Subsystem IO (available if IO_CFS_EN = true) --
    cfs_in_i       => std_ulogic_vector(cfs_in_i),
    cfs_out_o      => cfs_out_aux,
    -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
    neoled_o       => neoled_aux,
    -- Machine timer system time (available if IO_MTIME_EN = true) --
    mtime_time_o   => mtime_time_aux,
    -- External platform interrupts (available if XIRQ_NUM_CH > 0) --
    xirq_i         => xirq_i_aux,
    -- CPU Interrupts --
    mtime_irq_i    => std_ulogic(mtime_irq_i),
    msw_irq_i      => std_ulogic(msw_irq_i),
    mext_irq_i     => std_ulogic(mext_irq_i)
  );


  -- Type Conversion (Outputs) --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  jtag_tdo_o     <= std_logic(jtag_tdo_aux);

  s1_axis_tready <= std_logic(s1_axis_tready_aux);
  s0_axis_tdata  <= std_logic_vector(s0_axis_tdata_aux);
  s0_axis_tdest  <= std_logic_vector(s0_axis_tdest_aux);
  s0_axis_tvalid <= std_logic(s0_axis_tvalid_aux);
  s0_axis_tlast  <= std_logic(s0_axis_tlast_aux);

  xip_csn_o      <= std_logic(xip_csn_aux);
  xip_clk_o      <= std_logic(xip_clk_aux);
  xip_dat_o      <= std_logic(xip_do_aux);

  uart0_txd_o    <= std_logic(uart0_txd_aux);
  uart0_rts_o    <= std_logic(uart0_rts_aux);
  uart1_txd_o    <= std_logic(uart1_txd_aux);
  uart1_rts_o    <= std_logic(uart1_rts_aux);

  spi_clk_o      <= std_logic(spi_clk_aux);
  spi_dat_o      <= std_logic(spi_do_aux);
  spi_csn_o      <= std_logic_vector(spi_csn_aux);

  sdi_dat_o      <= std_logic(sdi_do_aux);

  twi_sda_o      <= std_logic(twi_sda_o_aux);
  twi_scl_o      <= std_logic(twi_scl_o_aux);

  twd_sda_o      <= std_logic(twd_sda_o_aux);
  twd_scl_o      <= std_logic(twd_scl_o_aux);

  onewire_o      <= std_logic(onewire_o_aux);

  cfs_out_o      <= std_logic_vector(cfs_out_aux);

  neoled_o       <= std_logic(neoled_aux);

  mtime_time_o   <= std_logic_vector(mtime_time_aux);


  -- Type Conversion (Constrained Size Ports) -----------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- GPIO input --
  gpio_in_mapping: process(gpio_i)
  begin
    gpio_i_aux <= (others => '0');
    for i in 0 to IO_GPIO_IN_NUM-1 loop
      gpio_i_aux(i) <= std_ulogic(gpio_i(i));
    end loop;
  end process gpio_in_mapping;

  -- GPIO output --
  gpio_out_mapping:
  for i in 0 to IO_GPIO_OUT_NUM-1 generate
    gpio_o(i) <= std_logic(gpio_o_aux(i));
  end generate;

  -- PWM --
  pwm_mapping:
  for i in 0 to IO_PWM_NUM_CH-1 generate
    pwm_o(i) <= std_logic(pwm_o_aux(i));
  end generate;

  -- XIRQ --
  xirq_mapping: process(xirq_i)
  begin
    xirq_i_aux <= (others => '0');
    for i in 0 to XIRQ_NUM_CH-1 loop
      xirq_i_aux(i) <= std_ulogic(xirq_i(i));
    end loop;
  end process xirq_mapping;


  -- Wishbone-to-AXI4-Lite Bridge -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  axi4_bridge:
  if XBUS_EN generate
    axi4_bridge_inst: xbus2axi4lite_bridge
    port map (
      -- Global control --
      clk           => clk,
      resetn        => resetn,
      -- XBUS device interface --
      xbus_adr_i    => xbus_adr,
      xbus_dat_i    => xbus_do,
      xbus_tag_i    => xbus_tag,
      xbus_we_i     => xbus_we,
      xbus_sel_i    => xbus_sel,
      xbus_stb_i    => xbus_stb,
      xbus_cyc_i    => xbus_cyc,
      xbus_ack_o    => xbus_ack,
      xbus_err_o    => xbus_err,
      xbus_dat_o    => xbus_di,
      -- AXI4-Lite host write address channel --
      m_axi_awaddr  => m_axi_awaddr,
      m_axi_awprot  => m_axi_awprot,
      m_axi_awvalid => m_axi_awvalid,
      m_axi_awready => m_axi_awready,
      -- AXI4-Lite host write data channel --
      m_axi_wdata   => m_axi_wdata,
      m_axi_wstrb   => m_axi_wstrb,
      m_axi_wvalid  => m_axi_wvalid,
      m_axi_wready  => m_axi_wready,
      -- AXI4-Lite host read address channel --
      m_axi_araddr  => m_axi_araddr,
      m_axi_arprot  => m_axi_arprot,
      m_axi_arvalid => m_axi_arvalid,
      m_axi_arready => m_axi_arready,
      -- AXI4-Lite host read data channel --
      m_axi_rdata   => m_axi_rdata,
      m_axi_rresp   => m_axi_rresp,
      m_axi_rvalid  => m_axi_rvalid,
      m_axi_rready  => m_axi_rready,
      -- AXI4-Lite host write response channel --
      m_axi_bresp   => m_axi_bresp,
      m_axi_bvalid  => m_axi_bvalid,
      m_axi_bready  => m_axi_bready
    );
  end generate;

end architecture neorv32_vivado_ip_rtl;
