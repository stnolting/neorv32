-- ================================================================================ --
-- NEORV32 - Processor Wrapper with AXI4 & AXI4-Stream Compatible Interfaces        --
-- -------------------------------------------------------------------------------- --
-- Dedicated for IP packaging/integration using Microchip Libero.                   --
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

entity neorv32_libero_ip is
  generic (
    -- ------------------------------------------------------------
    -- Configuration Generics
    -- ------------------------------------------------------------
    -- Clocking --
    CLOCK_FREQUENCY        : natural                        := 100_000_000;
    -- Dual-Core Configuration --
    DUAL_CORE_EN_INT       : integer range 0 to 1           := 0;
    -- Boot Configuration --
    BOOT_MODE_SELECT       : natural range 0 to 2           := 1;
    BOOT_ADDR_CUSTOM_UPPER : natural range 0 to 65_535      := 53248;
    BOOT_ADDR_CUSTOM_LOWER : natural range 0 to 65_535      := 53248;
    -- On-Chip Debugger (OCD) --
    OCD_EN_INT             : integer range 0 to 1           := 0;
    OCD_NUM_HW_TRIGGERS    : integer range 0 to 16          := 0;
    OCD_AUTHENTICATION_INT : integer range 0 to 1           := 0;
    OCD_JEDEC_ID           : std_logic_vector(10 downto 0)  := "00000000000";
    -- RISC-V CPU Extensions --
    RISCV_ISA_C_INT        : integer range 0 to 1           := 0;
    RISCV_ISA_E_INT        : integer range 0 to 1           := 0;
    RISCV_ISA_M_INT        : integer range 0 to 1           := 0;
    RISCV_ISA_U_INT        : integer range 0 to 1           := 0;
    RISCV_ISA_Zaamo_INT    : integer range 0 to 1           := 0;
    RISCV_ISA_Zalrsc_INT   : integer range 0 to 1           := 0;
    RISCV_ISA_Zba_INT      : integer range 0 to 1           := 0;
    RISCV_ISA_Zbb_INT      : integer range 0 to 1           := 0;
    RISCV_ISA_Zbkb_INT     : integer range 0 to 1           := 0;
    RISCV_ISA_Zbkc_INT     : integer range 0 to 1           := 0;
    RISCV_ISA_Zbkx_INT     : integer range 0 to 1           := 0;
    RISCV_ISA_Zbs_INT      : integer range 0 to 1           := 0;
    RISCV_ISA_Zfinx_INT    : integer range 0 to 1           := 0;
    RISCV_ISA_Zicntr_INT   : integer range 0 to 1           := 0;
    RISCV_ISA_Zicond_INT   : integer range 0 to 1           := 0;
    RISCV_ISA_Zihpm_INT    : integer range 0 to 1           := 0;
    RISCV_ISA_Zmmul_INT    : integer range 0 to 1           := 0;
    RISCV_ISA_Zknd_INT     : integer range 0 to 1           := 0;
    RISCV_ISA_Zkne_INT     : integer range 0 to 1           := 0;
    RISCV_ISA_Zknh_INT     : integer range 0 to 1           := 0;
    RISCV_ISA_Zksed_INT    : integer range 0 to 1           := 0;
    RISCV_ISA_Zksh_INT     : integer range 0 to 1           := 0;
    RISCV_ISA_Zxcfu_INT    : integer range 0 to 1           := 0;
    -- Tuning Options --
    CPU_FAST_MUL_EN_INT    : integer range 0 to 1           := 0;
    CPU_FAST_SHIFT_EN_INT  : integer range 0 to 1           := 0;
    CPU_RF_HW_RST_EN_INT   : integer range 0 to 1           := 0;
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS        : natural range 0 to 16          := 0;
    PMP_MIN_GRANULARITY    : natural                        := 4;
    PMP_TOR_MODE_EN_INT    : integer range 0 to 1           := 0;
    PMP_NAP_MODE_EN_INT    : integer range 0 to 1           := 0;
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS           : natural range 0 to 13          := 0;
    HPM_CNT_WIDTH          : natural range 0 to 64          := 40;
    -- Internal Instruction memory --
    IMEM_EN_INT            : integer range 0 to 1           := 0;
    IMEM_SIZE              : natural                        := 16384;
    IMEM_OUTREG_EN_INT     : integer range 0 to 1           := 0;
    -- Internal Data memory --
    DMEM_EN_INT            : integer range 0 to 1           := 0;
    DMEM_SIZE              : natural                        := 8192;
    DMEM_OUTREG_EN_INT     : integer range 0 to 1           := 0;
    -- CPU Caches --
    ICACHE_EN_INT          : integer range 0 to 1           := 0;
    ICACHE_NUM_BLOCKS      : natural range 1 to 4096        := 4;
    DCACHE_EN_INT          : integer range 0 to 1           := 0;
    DCACHE_NUM_BLOCKS      : natural range 1 to 4096        := 4;
    CACHE_BLOCK_SIZE       : natural range 8 to 1024        := 64;
    -- External Bus Interface --
    XBUS_EN_INT            : integer range 0 to 1           := 1;
    XBUS_REGSTAGE_EN_INT   : integer range 0 to 1           := 1;
    -- Processor peripherals --
    IO_GPIO_EN_INT         : integer range 0 to 1           := 0;
    IO_GPIO_IN_NUM         : natural range 1 to 32          := 1;
    IO_GPIO_OUT_NUM        : natural range 1 to 32          := 1;
    IO_CLINT_EN_INT        : integer range 0 to 1           := 0;
    IO_UART0_EN_INT        : integer range 0 to 1           := 0;
    IO_UART0_RX_FIFO       : natural range 1 to 2**15       := 1;
    IO_UART0_TX_FIFO       : natural range 1 to 2**15       := 1;
    IO_UART1_EN_INT        : integer range 0 to 1           := 0;
    IO_UART1_RX_FIFO       : natural range 1 to 2**15       := 1;
    IO_UART1_TX_FIFO       : natural range 1 to 2**15       := 1;
    IO_SPI_EN_INT          : integer range 0 to 1           := 0;
    IO_SPI_FIFO            : natural range 1 to 2**15       := 1;
    IO_SDI_EN_INT          : integer range 0 to 1           := 0;
    IO_SDI_FIFO            : natural range 1 to 2**15       := 1;
    IO_TWI_EN_INT          : integer range 0 to 1           := 0;
    IO_TWI_FIFO            : natural range 1 to 2**15       := 1;
    IO_TWD_EN_INT          : integer range 0 to 1           := 0;
    IO_TWD_RX_FIFO         : natural range 1 to 2**15       := 1;
    IO_TWD_TX_FIFO         : natural range 1 to 2**15       := 1;
    IO_PWM_EN_INT          : integer range 0 to 1           := 0;
    IO_PWM_NUM_CH          : natural range 1 to 16          := 1;
    IO_WDT_EN_INT          : integer range 0 to 1           := 0;
    IO_TRNG_EN_INT         : integer range 0 to 1           := 0;
    IO_TRNG_FIFO           : natural range 1 to 2**15       := 1;
    IO_CFS_EN_INT          : integer range 0 to 1           := 0;
    IO_NEOLED_EN_INT       : integer range 0 to 1           := 0;
    IO_NEOLED_TX_FIFO      : natural range 1 to 2**15       := 1;
    IO_GPTMR_EN_INT        : integer range 0 to 1           := 0;
    IO_ONEWIRE_EN_INT      : integer range 0 to 1           := 0;
    IO_DMA_EN_INT          : integer range 0 to 1           := 0;
    IO_DMA_DSC_FIFO        : natural range 4 to 512         := 4;
    IO_SLINK_EN_INT        : integer range 0 to 1           := 0;
    IO_SLINK_RX_FIFO       : natural range 1 to 2**15       := 1;
    IO_SLINK_TX_FIFO       : natural range 1 to 2**15       := 1
  );
  port (
    -- ------------------------------------------------------------
    -- Global Control
    -- ------------------------------------------------------------
    clk            : in  std_logic;
    resetn         : in  std_logic; -- low-active
    ocd_resetn     : out std_logic; -- on-chip debugger reset output, low-active, sync
    wdt_resetn     : out std_logic; -- watchdog reset output, low-active, sync
    -- ------------------------------------------------------------
    -- AXI4 Host Interface (available if XBUS_EN_INT = 1)
    -- ------------------------------------------------------------
    -- Write Address Channel --
    m_axi_awaddr   : out std_logic_vector(31 downto 0);
    m_axi_awlen    : out std_logic_vector(7 downto 0);
    m_axi_awsize   : out std_logic_vector(2 downto 0);
    m_axi_awburst  : out std_logic_vector(1 downto 0);
    m_axi_awcache  : out std_logic_vector(3 downto 0);
    m_axi_awprot   : out std_logic_vector(2 downto 0);
    m_axi_awvalid  : out std_logic;
    m_axi_awready  : in  std_logic := '0';
    -- Write Data Channel --
    m_axi_wdata    : out std_logic_vector(31 downto 0);
    m_axi_wstrb    : out std_logic_vector(3 downto 0);
    m_axi_wlast    : out std_logic;
    m_axi_wvalid   : out std_logic;
    m_axi_wready   : in  std_logic := '0';
    -- Read Address Channel --
    m_axi_araddr   : out std_logic_vector(31 downto 0);
    m_axi_arlen    : out std_logic_vector(7 downto 0);
    m_axi_arsize   : out std_logic_vector(2 downto 0);
    m_axi_arburst  : out std_logic_vector(1 downto 0);
    m_axi_arcache  : out std_logic_vector(3 downto 0);
    m_axi_arprot   : out std_logic_vector(2 downto 0);
    m_axi_arvalid  : out std_logic;
    m_axi_arready  : in  std_logic := '0';
    -- Read Data Channel --
    m_axi_rdata    : in  std_logic_vector(31 downto 0) := x"00000000";
    m_axi_rresp    : in  std_logic_vector(1 downto 0); -- no default here (#1067)
    m_axi_rlast    : in  std_logic;
    m_axi_rvalid   : in  std_logic := '0';
    m_axi_rready   : out std_logic;
    -- Write Response Channel --
    m_axi_bresp    : in  std_logic_vector(1 downto 0); -- no default here (#1067)
    m_axi_bvalid   : in  std_logic := '0';
    m_axi_bready   : out std_logic;
    -- ------------------------------------------------------------
    -- AXI4-Stream Interfaces (available if IO_SLINK_EN_INT = 1)
    -- ------------------------------------------------------------
    -- Source --
    s0_axis_tdest  : out std_logic_vector(3 downto 0);
    s0_axis_tvalid : out std_logic;
    s0_axis_tready : in  std_logic := '0';
    s0_axis_tdata  : out std_logic_vector(31 downto 0);
    s0_axis_tlast  : out std_logic;
    -- Sink --
    s1_axis_tid    : in  std_logic_vector(3 downto 0) := x"0";
    s1_axis_tvalid : in  std_logic := '0';
    s1_axis_tready : out std_logic;
    s1_axis_tdata  : in  std_logic_vector(31 downto 0) := x"00000000";
    s1_axis_tlast  : in  std_logic := '0';
    -- ------------------------------------------------------------
    -- JTAG on-chip debugger interface (available if OCD_EN_INT = 1)
    -- ------------------------------------------------------------
    jtag_tck_i     : in  std_logic := '0';
    jtag_tdi_i     : in  std_logic := '0';
    jtag_tdo_o     : out std_logic := '0';
    jtag_tms_i     : in  std_logic := '0';
    -- ------------------------------------------------------------
    -- Processor IO
    -- ------------------------------------------------------------
    -- GPIO (available if IO_GPIO_IN/OUT_NUM > 0) --
    gpio_o         : out std_logic_vector(IO_GPIO_OUT_NUM-1 downto 0); -- variable-sized ports must be at least 0 downto 0; #974
    gpio_i         : in  std_logic_vector(IO_GPIO_IN_NUM-1 downto 0) := (others => '0'); -- variable-sized ports must be at least 0 downto 0; #974
    -- primary UART0 (available if IO_UART0_EN_INT = 1) --
    uart0_txd_o    : out std_logic;
    uart0_rxd_i    : in  std_logic := '0';
    uart0_rtsn_o   : out std_logic;
    uart0_ctsn_i   : in  std_logic := '0';
    -- secondary UART1 (available if IO_UART1_EN_INT = 1) --
    uart1_txd_o    : out std_logic;
    uart1_rxd_i    : in  std_logic := '0';
    uart1_rtsn_o   : out std_logic;
    uart1_ctsn_i   : in  std_logic := '0';
    -- SPI (available if IO_SPI_EN_INT = 1) --
    spi_clk_o      : out std_logic;
    spi_dat_o      : out std_logic;
    spi_dat_i      : in  std_logic := '0';
    spi_csn_o      : out std_logic_vector(7 downto 0); -- SPI CS
    -- SDI (available if IO_SDI_EN_INT = 1) --
    sdi_clk_i      : in  std_logic := '0';
    sdi_dat_o      : out std_logic;
    sdi_dat_i      : in  std_logic := '0';
    sdi_csn_i      : in  std_logic := '0';
    -- TWI (available if IO_TWI_EN_INT = 1) --
    twi_sda_i      : in  std_logic := '0';
    twi_sda_o      : out std_logic;
    twi_scl_i      : in  std_logic := '0';
    twi_scl_o      : out std_logic;
    -- TWD (available if IO_TWD_EN_INT = 1) --
    twd_sda_i      : in  std_logic := '0';
    twd_sda_o      : out std_logic;
    twd_scl_i      : in  std_logic := '0';
    twd_scl_o      : out std_logic;
    -- 1-Wire Interface (available if IO_ONEWIRE_EN_INT = 1) --
    onewire_i      : in  std_logic := '0';
    onewire_o      : out std_logic;
    -- PWM (available if IO_PWM_NUM_CH > 0) --
    pwm_o          : out std_logic_vector(IO_PWM_NUM_CH-1 downto 0); -- variable-sized ports must be at least 0 downto 0; #974
    -- Custom Functions Subsystem IO (available if IO_CFS_EN_INT = 1) --
    cfs_in_i       : in  std_logic_vector(255 downto 0) := (others => '0');
    cfs_out_o      : out std_logic_vector(255 downto 0);
    -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN_INT = 1) --
    neoled_o       : out std_logic;
    -- Machine timer system time (available if IO_CLINT_EN_INT = 1) --
    mtime_time_o   : out std_logic_vector(63 downto 0);
    -- CPU Interrupts --
    mtime_irq_i    : in  std_logic := '0';
    msw_irq_i      : in  std_logic := '0';
    mext_irq_i     : in  std_logic := '0'
  );
end entity;

architecture neorv32_libero_ip_rtl of neorv32_libero_ip is

  -- boolean conversions for generics --
  constant dual_core_en_c       : boolean := (DUAL_CORE_EN_INT = 1);
  constant ocd_en_c             : boolean := (OCD_EN_INT = 1);
  constant ocd_authentication_c : boolean := (OCD_AUTHENTICATION_INT = 1);
  constant riscv_isa_c_c        : boolean := (RISCV_ISA_C_INT = 1);
  constant riscv_isa_e_c        : boolean := (RISCV_ISA_E_INT = 1);
  constant riscv_isa_m_c        : boolean := (RISCV_ISA_M_INT = 1);
  constant riscv_isa_u_c        : boolean := (RISCV_ISA_U_INT = 1);
  constant riscv_isa_zaamo_c    : boolean := (RISCV_ISA_Zaamo_INT = 1);
  constant riscv_isa_zalrsc_c   : boolean := (RISCV_ISA_Zalrsc_INT = 1);
  constant riscv_isa_zba_c      : boolean := (RISCV_ISA_Zba_INT = 1);
  constant riscv_isa_zbb_c      : boolean := (RISCV_ISA_Zbb_INT = 1);
  constant riscv_isa_zbkb_c     : boolean := (RISCV_ISA_Zbkb_INT = 1);
  constant riscv_isa_zbkc_c     : boolean := (RISCV_ISA_Zbkc_INT = 1);
  constant riscv_isa_zbkx_c     : boolean := (RISCV_ISA_Zbkx_INT = 1);
  constant riscv_isa_zbs_c      : boolean := (RISCV_ISA_Zbs_INT = 1);
  constant riscv_isa_zfinx_c    : boolean := (RISCV_ISA_Zfinx_INT = 1);
  constant riscv_isa_zicntr_c   : boolean := (RISCV_ISA_Zicntr_INT = 1);
  constant riscv_isa_zicond_c   : boolean := (RISCV_ISA_Zicond_INT = 1);
  constant riscv_isa_zihpm_c    : boolean := (RISCV_ISA_Zihpm_INT = 1);
  constant riscv_isa_zmmul_c    : boolean := (RISCV_ISA_Zmmul_INT = 1);
  constant riscv_isa_zknd_c     : boolean := (RISCV_ISA_Zknd_INT = 1);
  constant riscv_isa_zkne_c     : boolean := (RISCV_ISA_Zkne_INT = 1);
  constant riscv_isa_zknh_c     : boolean := (RISCV_ISA_Zknh_INT = 1);
  constant riscv_isa_zksed_c    : boolean := (RISCV_ISA_Zksed_INT = 1);
  constant riscv_isa_zksh_c     : boolean := (RISCV_ISA_Zksh_INT = 1);
  constant riscv_isa_zxcfu_c    : boolean := (RISCV_ISA_Zxcfu_INT = 1);
  constant cpu_fast_mul_en_c    : boolean := (CPU_FAST_MUL_EN_INT = 1);
  constant cpu_fast_shift_en_c  : boolean := (CPU_FAST_SHIFT_EN_INT = 1);
  constant cpu_rf_hw_rst_en_c   : boolean := (CPU_RF_HW_RST_EN_INT = 1);
  constant pmp_tor_mode_en_c    : boolean := (PMP_TOR_MODE_EN_INT = 1);
  constant pmp_nap_mode_en_c    : boolean := (PMP_NAP_MODE_EN_INT = 1);
  constant imem_en_c            : boolean := (IMEM_EN_INT = 1);
  constant imem_outreg_en_c     : boolean := (IMEM_OUTREG_EN_INT = 1);
  constant dmem_en_c            : boolean := (DMEM_EN_INT = 1);
  constant dmem_outreg_en_c     : boolean := (DMEM_OUTREG_EN_INT = 1);
  constant icache_en_c          : boolean := (ICACHE_EN_INT = 1);
  constant dcache_en_c          : boolean := (DCACHE_EN_INT = 1);
  constant xbus_en_c            : boolean := (XBUS_EN_INT = 1);
  constant xbus_regstage_en_c   : boolean := (XBUS_REGSTAGE_EN_INT = 1);
  constant io_gpio_en_c         : boolean := (IO_GPIO_EN_INT = 1);
  constant io_clint_en_c        : boolean := (IO_CLINT_EN_INT = 1);
  constant io_uart0_en_c        : boolean := (IO_UART0_EN_INT = 1);
  constant io_uart1_en_c        : boolean := (IO_UART1_EN_INT = 1);
  constant io_spi_en_c          : boolean := (IO_SPI_EN_INT = 1);
  constant io_sdi_en_c          : boolean := (IO_SDI_EN_INT = 1);
  constant io_twi_en_c          : boolean := (IO_TWI_EN_INT = 1);
  constant io_twd_en_c          : boolean := (IO_TWD_EN_INT = 1);
  constant io_pwm_en_c          : boolean := (IO_PWM_EN_INT = 1);
  constant io_wdt_en_c          : boolean := (IO_WDT_EN_INT = 1);
  constant io_trng_en_c         : boolean := (IO_TRNG_EN_INT = 1);
  constant io_cfs_en_c          : boolean := (IO_CFS_EN_INT = 1);
  constant io_neoled_en_c       : boolean := (IO_NEOLED_EN_INT = 1);
  constant io_gptmr_en_c        : boolean := (IO_GPTMR_EN_INT = 1);
  constant io_onewire_en_c      : boolean := (IO_ONEWIRE_EN_INT = 1);
  constant io_dma_en_c          : boolean := (IO_DMA_EN_INT = 1);
  constant io_slink_en_c        : boolean := (IO_SLINK_EN_INT = 1);
  constant BOOT_ADDR_CUSTOM     : std_ulogic_vector(31 downto 0) := std_ulogic_vector(to_unsigned(BOOT_ADDR_CUSTOM_UPPER * 2**16 + BOOT_ADDR_CUSTOM_LOWER, 32));

  -- auto-configuration --
  constant num_gpio_c : natural := cond_sel_natural_f(io_gpio_en_c, max_natural_f(IO_GPIO_IN_NUM, IO_GPIO_OUT_NUM), 0);
  constant num_pwm_c  : natural := cond_sel_natural_f(io_pwm_en_c, IO_PWM_NUM_CH, 0);
  constant burst_en_c : boolean := icache_en_c or dcache_en_c; -- any cache bursts?

  -- AXI4 bridge --
  component xbus2axi4_bridge
  generic (
    BURST_EN  : boolean; -- enable burst transfers
    BURST_LEN : natural range 4 to 1024 -- bytes per burst, has to be a multiple of 4
  );
  port (
    -- Global control
    clk           : in  std_logic;
    resetn        : in  std_logic;
    -- XBUS device interface --
    xbus_adr_i    : in  std_ulogic_vector(31 downto 0);
    xbus_dat_i    : in  std_ulogic_vector(31 downto 0);
    xbus_cti_i    : in  std_ulogic_vector(2 downto 0);
    xbus_tag_i    : in  std_ulogic_vector(2 downto 0);
    xbus_we_i     : in  std_ulogic;
    xbus_sel_i    : in  std_ulogic_vector(3 downto 0);
    xbus_stb_i    : in  std_ulogic;
    xbus_ack_o    : out std_ulogic;
    xbus_err_o    : out std_ulogic;
    xbus_dat_o    : out std_ulogic_vector(31 downto 0);
    -- AXI4 host write address channel --
    m_axi_awaddr  : out std_logic_vector(31 downto 0);
    m_axi_awlen   : out std_logic_vector(7 downto 0);
    m_axi_awsize  : out std_logic_vector(2 downto 0);
    m_axi_awburst : out std_logic_vector(1 downto 0);
    m_axi_awcache : out std_logic_vector(3 downto 0);
    m_axi_awprot  : out std_logic_vector(2 downto 0);
    m_axi_awvalid : out std_logic;
    m_axi_awready : in  std_logic;
    -- AXI4 host write data channel --
    m_axi_wdata   : out std_logic_vector(31 downto 0);
    m_axi_wstrb   : out std_logic_vector(3 downto 0);
    m_axi_wlast   : out std_logic;
    m_axi_wvalid  : out std_logic;
    m_axi_wready  : in  std_logic;
    -- AXI4 host read address channel --
    m_axi_araddr  : out std_logic_vector(31 downto 0);
    m_axi_arlen   : out std_logic_vector(7 downto 0);
    m_axi_arsize  : out std_logic_vector(2 downto 0);
    m_axi_arburst : out std_logic_vector(1 downto 0);
    m_axi_arcache : out std_logic_vector(3 downto 0);
    m_axi_arprot  : out std_logic_vector(2 downto 0);
    m_axi_arvalid : out std_logic;
    m_axi_arready : in  std_logic;
    -- AXI4 host read data channel --
    m_axi_rdata   : in  std_logic_vector(31 downto 0);
    m_axi_rresp   : in  std_logic_vector(1 downto 0);
    m_axi_rlast   : in  std_logic;
    m_axi_rvalid  : in  std_logic;
    m_axi_rready  : out std_logic;
    -- AXI4 host write response channel --
    m_axi_bresp   : in  std_logic_vector(1 downto 0);
    m_axi_bvalid  : in  std_logic;
    m_axi_bready  : out std_logic
  );
  end component;

  -- type conversion --
  signal rstn_ocd, rstn_wdt : std_ulogic;
  signal jtag_tdo_aux : std_ulogic;
  signal s0_axis_tdata_aux : std_ulogic_vector(31 downto 0);
  signal s0_axis_tdest_aux : std_ulogic_vector(3 downto 0);
  signal s1_axis_tready_aux, s0_axis_tvalid_aux, s0_axis_tlast_aux : std_ulogic;
  signal uart0_txd_aux, uart0_rtsn_aux, uart1_txd_aux, uart1_rtsn_aux : std_ulogic;
  signal spi_clk_aux, spi_do_aux : std_ulogic;
  signal spi_csn_aux : std_ulogic_vector(7 downto 0);
  signal sdi_do_aux : std_ulogic;
  signal twi_sda_o_aux, twi_scl_o_aux : std_ulogic;
  signal twd_sda_o_aux, twd_scl_o_aux : std_ulogic;
  signal onewire_o_aux : std_ulogic;
  signal cfs_out_aux : std_ulogic_vector(255 downto 0);
  signal neoled_aux : std_ulogic;
  signal mtime_time_aux : std_ulogic_vector(63 downto 0);

  -- constrained size ports --
  signal gpio_o_aux : std_ulogic_vector(31 downto 0);
  signal gpio_i_aux : std_ulogic_vector(31 downto 0);
  signal pwm_o_aux  : std_ulogic_vector(15 downto 0);

  -- internal xbus --
  signal xbus_req : xbus_req_t;
  signal xbus_rsp : xbus_rsp_t;

begin

  -- The Core Of The Problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_top_inst: neorv32_top
  generic map (
    -- Clocking --
    CLOCK_FREQUENCY     => CLOCK_FREQUENCY,
    -- Dual-Core Configuration --
    DUAL_CORE_EN        => dual_core_en_c,
    -- Boot Configuration --
    BOOT_MODE_SELECT    => BOOT_MODE_SELECT,
    BOOT_ADDR_CUSTOM    => BOOT_ADDR_CUSTOM,
    -- On-Chip Debugger --
    OCD_EN              => ocd_en_c,
    OCD_NUM_HW_TRIGGERS => OCD_NUM_HW_TRIGGERS,
    OCD_AUTHENTICATION  => ocd_authentication_c,
    OCD_JEDEC_ID        => std_ulogic_vector(OCD_JEDEC_ID),
    -- RISC-V CPU Extensions --
    RISCV_ISA_C         => riscv_isa_c_c,
    RISCV_ISA_E         => riscv_isa_e_c,
    RISCV_ISA_M         => riscv_isa_m_c,
    RISCV_ISA_U         => riscv_isa_u_c,
    RISCV_ISA_Zaamo     => riscv_isa_zaamo_c,
    RISCV_ISA_Zalrsc    => riscv_isa_zalrsc_c,
    RISCV_ISA_Zba       => riscv_isa_zba_c,
    RISCV_ISA_Zbb       => riscv_isa_zbb_c,
    RISCV_ISA_Zbkb      => riscv_isa_zbkb_c,
    RISCV_ISA_Zbkc      => riscv_isa_zbkc_c,
    RISCV_ISA_Zbkx      => riscv_isa_zbkx_c,
    RISCV_ISA_Zbs       => riscv_isa_zbs_c,
    RISCV_ISA_Zfinx     => riscv_isa_zfinx_c,
    RISCV_ISA_Zicntr    => riscv_isa_zicntr_c,
    RISCV_ISA_Zicond    => riscv_isa_zicond_c,
    RISCV_ISA_Zihpm     => riscv_isa_zihpm_c,
    RISCV_ISA_Zmmul     => riscv_isa_zmmul_c,
    RISCV_ISA_Zknd      => riscv_isa_zknd_c,
    RISCV_ISA_Zkne      => riscv_isa_zkne_c,
    RISCV_ISA_Zknh      => riscv_isa_zknh_c,
    RISCV_ISA_Zksed     => riscv_isa_zksed_c,
    RISCV_ISA_Zksh      => riscv_isa_zksh_c,
    RISCV_ISA_Zxcfu     => riscv_isa_zxcfu_c,
    -- Extension Options --
    CPU_FAST_MUL_EN     => cpu_fast_mul_en_c,
    CPU_FAST_SHIFT_EN   => cpu_fast_shift_en_c,
    CPU_RF_HW_RST_EN    => cpu_rf_hw_rst_en_c,
    -- Physical Memory Protection --
    PMP_NUM_REGIONS     => PMP_NUM_REGIONS,
    PMP_MIN_GRANULARITY => PMP_MIN_GRANULARITY,
    PMP_TOR_MODE_EN     => pmp_tor_mode_en_c,
    PMP_NAP_MODE_EN     => pmp_nap_mode_en_c,
    -- Hardware Performance Monitors --
    HPM_NUM_CNTS        => HPM_NUM_CNTS,
    HPM_CNT_WIDTH       => HPM_CNT_WIDTH,
    -- Internal Instruction memory --
    IMEM_EN             => imem_en_c,
    IMEM_SIZE           => IMEM_SIZE,
    IMEM_OUTREG_EN      => imem_outreg_en_c,
    -- Internal Data memory --
    DMEM_EN             => dmem_en_c,
    DMEM_SIZE           => DMEM_SIZE,
    DMEM_OUTREG_EN      => dmem_outreg_en_c,
    -- CPU Caches --
    ICACHE_EN           => icache_en_c,
    ICACHE_NUM_BLOCKS   => ICACHE_NUM_BLOCKS,
    DCACHE_EN           => dcache_en_c,
    DCACHE_NUM_BLOCKS   => DCACHE_NUM_BLOCKS,
    CACHE_BLOCK_SIZE    => CACHE_BLOCK_SIZE,
    -- External bus interface --
    XBUS_EN             => xbus_en_c,
    XBUS_TIMEOUT        => 0, -- AXI does not allow any timeouts
    XBUS_REGSTAGE_EN    => xbus_regstage_en_c,
    -- Processor peripherals --
    IO_DISABLE_SYSINFO  => false,
    IO_GPIO_NUM         => num_gpio_c,
    IO_CLINT_EN         => io_clint_en_c,
    IO_UART0_EN         => io_uart0_en_c,
    IO_UART0_RX_FIFO    => IO_UART0_RX_FIFO,
    IO_UART0_TX_FIFO    => IO_UART0_TX_FIFO,
    IO_UART1_EN         => io_uart1_en_c,
    IO_UART1_RX_FIFO    => IO_UART1_RX_FIFO,
    IO_UART1_TX_FIFO    => IO_UART1_TX_FIFO,
    IO_SPI_EN           => io_spi_en_c,
    IO_SPI_FIFO         => IO_SPI_FIFO,
    IO_SDI_EN           => io_sdi_en_c,
    IO_SDI_FIFO         => IO_SDI_FIFO,
    IO_TWI_EN           => io_twi_en_c,
    IO_TWI_FIFO         => IO_TWI_FIFO,
    IO_TWD_EN           => io_twd_en_c,
    IO_TWD_RX_FIFO      => IO_TWD_RX_FIFO,
    IO_TWD_TX_FIFO      => IO_TWD_TX_FIFO,
    IO_PWM_NUM_CH       => num_pwm_c,
    IO_WDT_EN           => io_wdt_en_c,
    IO_TRNG_EN          => io_trng_en_c,
    IO_TRNG_FIFO        => IO_TRNG_FIFO,
    IO_CFS_EN           => io_cfs_en_c,
    IO_NEOLED_EN        => io_neoled_en_c,
    IO_NEOLED_TX_FIFO   => IO_NEOLED_TX_FIFO,
    IO_GPTMR_EN         => io_gptmr_en_c,
    IO_ONEWIRE_EN       => io_onewire_en_c,
    IO_DMA_EN           => io_dma_en_c,
    IO_DMA_DSC_FIFO     => IO_DMA_DSC_FIFO,
    IO_SLINK_EN         => io_slink_en_c,
    IO_SLINK_RX_FIFO    => IO_SLINK_RX_FIFO,
    IO_SLINK_TX_FIFO    => IO_SLINK_TX_FIFO
  )
  port map (
    -- Global control --
    clk_i          => std_ulogic(clk),
    rstn_i         => std_ulogic(resetn),
    rstn_ocd_o     => rstn_ocd,
    rstn_wdt_o     => rstn_wdt,
    -- JTAG on-chip debugger interface (available if OCD_EN_INT = 1) --
    jtag_tck_i     => std_ulogic(jtag_tck_i),
    jtag_tdi_i     => std_ulogic(jtag_tdi_i),
    jtag_tdo_o     => jtag_tdo_aux,
    jtag_tms_i     => std_ulogic(jtag_tms_i),
    -- External bus interface (available if XBUS_EN_INT = 1) --
    xbus_adr_o     => xbus_req.addr,
    xbus_dat_o     => xbus_req.data,
    xbus_cti_o     => xbus_req.cti,
    xbus_tag_o     => xbus_req.tag,
    xbus_we_o      => xbus_req.we,
    xbus_sel_o     => xbus_req.sel,
    xbus_stb_o     => xbus_req.stb,
    xbus_cyc_o     => xbus_req.cyc,
    xbus_dat_i     => xbus_rsp.data,
    xbus_ack_i     => xbus_rsp.ack,
    xbus_err_i     => xbus_rsp.err,
    -- Stream Link Interface (available if IO_SLINK_EN_INT = 1) --
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
    -- GPIO (available if IO_GPIO_NUM > 0) --
    gpio_o         => gpio_o_aux,
    gpio_i         => gpio_i_aux,
    -- primary UART0 (available if IO_UART0_EN_INT = 1) --
    uart0_txd_o    => uart0_txd_aux,
    uart0_rxd_i    => std_ulogic(uart0_rxd_i),
    uart0_rtsn_o   => uart0_rtsn_aux,
    uart0_ctsn_i   => std_ulogic(uart0_ctsn_i),
    -- secondary UART1 (available if IO_UART1_EN_INT = 1) --
    uart1_txd_o    => uart1_txd_aux,
    uart1_rxd_i    => std_ulogic(uart1_rxd_i),
    uart1_rtsn_o   => uart1_rtsn_aux,
    uart1_ctsn_i   => std_ulogic(uart1_ctsn_i),
    -- SPI (available if IO_SPI_EN_INT = 1) --
    spi_clk_o      => spi_clk_aux,
    spi_dat_o      => spi_do_aux,
    spi_dat_i      => std_ulogic(spi_dat_i),
    spi_csn_o      => spi_csn_aux,
    -- SDI (available if IO_SDI_EN_INT = 1) --
    sdi_clk_i      => std_ulogic(sdi_clk_i),
    sdi_dat_o      => sdi_do_aux,
    sdi_dat_i      => std_ulogic(sdi_dat_i),
    sdi_csn_i      => std_ulogic(sdi_csn_i),
    -- TWI (available if IO_TWI_EN_INT = 1) --
    twi_sda_i      => std_ulogic(twi_sda_i),
    twi_sda_o      => twi_sda_o_aux,
    twi_scl_i      => std_ulogic(twi_scl_i),
    twi_scl_o      => twi_scl_o_aux,
    -- TWD (available if IO_TWD_EN_INT = 1) --
    twd_sda_i      => std_ulogic(twd_sda_i),
    twd_sda_o      => twd_sda_o_aux,
    twd_scl_i      => std_ulogic(twd_scl_i),
    twd_scl_o      => twd_scl_o_aux,
    -- 1-Wire Interface (available if IO_ONEWIRE_EN_INT = 1) --
    onewire_i      => std_ulogic(onewire_i),
    onewire_o      => onewire_o_aux,
    -- PWM available if IO_PWM_NUM_CH > 0) --
    pwm_o          => pwm_o_aux,
    -- Custom Functions Subsystem IO (available if IO_CFS_EN_INT = 1) --
    cfs_in_i       => std_ulogic_vector(cfs_in_i),
    cfs_out_o      => cfs_out_aux,
    -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN_INT = 1) --
    neoled_o       => neoled_aux,
    -- Machine timer system time (available if IO_CLINT_EN_INT = 1) --
    mtime_time_o   => mtime_time_aux,
    -- CPU Interrupts --
    mtime_irq_i    => std_ulogic(mtime_irq_i),
    msw_irq_i      => std_ulogic(msw_irq_i),
    mext_irq_i     => std_ulogic(mext_irq_i)
  );


  -- Type Conversion (Outputs) --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ocd_resetn <= std_logic(rstn_ocd);
  wdt_resetn <= std_logic(rstn_wdt);

  jtag_tdo_o <= std_logic(jtag_tdo_aux);

  s1_axis_tready <= std_logic(s1_axis_tready_aux);
  s0_axis_tdata  <= std_logic_vector(s0_axis_tdata_aux);
  s0_axis_tdest  <= std_logic_vector(s0_axis_tdest_aux);
  s0_axis_tvalid <= std_logic(s0_axis_tvalid_aux);
  s0_axis_tlast  <= std_logic(s0_axis_tlast_aux);

  uart0_txd_o  <= std_logic(uart0_txd_aux);
  uart0_rtsn_o <= std_logic(uart0_rtsn_aux);

  uart1_txd_o  <= std_logic(uart1_txd_aux);
  uart1_rtsn_o <= std_logic(uart1_rtsn_aux);

  spi_clk_o <= std_logic(spi_clk_aux);
  spi_dat_o <= std_logic(spi_do_aux);
  spi_csn_o <= std_logic_vector(spi_csn_aux);

  sdi_dat_o <= std_logic(sdi_do_aux);

  twi_sda_o <= std_logic(twi_sda_o_aux);
  twi_scl_o <= std_logic(twi_scl_o_aux);

  twd_sda_o <= std_logic(twd_sda_o_aux);
  twd_scl_o <= std_logic(twd_scl_o_aux);

  onewire_o <= std_logic(onewire_o_aux);

  cfs_out_o <= std_logic_vector(cfs_out_aux);

  neoled_o <= std_logic(neoled_aux);

  mtime_time_o <= std_logic_vector(mtime_time_aux);


  -- Type Conversion (Constrained-Size Ports) -----------------------------------------------
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


  -- XBUS-to-AXI4 Bridge --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  axi4_bridge:
  if (XBUS_EN_INT = 1) generate
    axi4_bridge_inst: xbus2axi4_bridge
    generic map (
      BURST_EN  => burst_en_c,
      BURST_LEN => CACHE_BLOCK_SIZE
    )
    port map (
      -- Global control --
      clk           => clk,
      resetn        => resetn,
      -- XBUS device interface --
      xbus_adr_i    => xbus_req.addr,
      xbus_dat_i    => xbus_req.data,
      xbus_cti_i    => xbus_req.cti,
      xbus_tag_i    => xbus_req.tag,
      xbus_we_i     => xbus_req.we,
      xbus_sel_i    => xbus_req.sel,
      xbus_stb_i    => xbus_req.stb,
      xbus_ack_o    => xbus_rsp.ack,
      xbus_err_o    => xbus_rsp.err,
      xbus_dat_o    => xbus_rsp.data,
      -- AXI4 host write address channel --
      m_axi_awaddr  => m_axi_awaddr,
      m_axi_awlen   => m_axi_awlen ,
      m_axi_awsize  => m_axi_awsize,
      m_axi_awburst => m_axi_awburst,
      m_axi_awcache => m_axi_awcache,
      m_axi_awprot  => m_axi_awprot,
      m_axi_awvalid => m_axi_awvalid,
      m_axi_awready => m_axi_awready,
      -- AXI4 host write data channel --
      m_axi_wdata   => m_axi_wdata,
      m_axi_wstrb   => m_axi_wstrb,
      m_axi_wlast   => m_axi_wlast,
      m_axi_wvalid  => m_axi_wvalid,
      m_axi_wready  => m_axi_wready,
      -- AXI4 host read address channel --
      m_axi_araddr  => m_axi_araddr,
      m_axi_arlen   => m_axi_arlen,
      m_axi_arsize  => m_axi_arsize,
      m_axi_arburst => m_axi_arburst,
      m_axi_arcache => m_axi_arcache,
      m_axi_arprot  => m_axi_arprot,
      m_axi_arvalid => m_axi_arvalid,
      m_axi_arready => m_axi_arready,
      -- AXI4 host read data channel --
      m_axi_rdata   => m_axi_rdata,
      m_axi_rresp   => m_axi_rresp,
      m_axi_rlast   => m_axi_rlast,
      m_axi_rvalid  => m_axi_rvalid,
      m_axi_rready  => m_axi_rready,
      -- AXI4 host write response channel --
      m_axi_bresp   => m_axi_bresp,
      m_axi_bvalid  => m_axi_bvalid,
      m_axi_bready  => m_axi_bready
    );
  end generate;

end architecture neorv32_libero_ip_rtl;
