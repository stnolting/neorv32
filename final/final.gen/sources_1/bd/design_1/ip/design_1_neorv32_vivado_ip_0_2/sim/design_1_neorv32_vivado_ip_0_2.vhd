-- (c) Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- (c) Copyright 2022-2026 Advanced Micro Devices, Inc. All rights reserved.
-- 
-- This file contains confidential and proprietary information
-- of AMD and is protected under U.S. and international copyright
-- and other intellectual property laws.
-- 
-- DISCLAIMER
-- This disclaimer is not a license and does not grant any
-- rights to the materials distributed herewith. Except as
-- otherwise provided in a valid license issued to you by
-- AMD, and to the maximum extent permitted by applicable
-- law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
-- WITH ALL FAULTS, AND AMD HEREBY DISCLAIMS ALL WARRANTIES
-- AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
-- BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
-- INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
-- (2) AMD shall not be liable (whether in contract or tort,
-- including negligence, or under any other theory of
-- liability) for any loss or damage of any kind or nature
-- related to, arising under or in connection with these
-- materials, including for any direct, or any indirect,
-- special, incidental, or consequential loss or damage
-- (including loss of data, profits, goodwill, or any type of
-- loss or damage suffered as a result of any action brought
-- by a third party) even if such damage or loss was
-- reasonably foreseeable or AMD had been advised of the
-- possibility of the same.
-- 
-- CRITICAL APPLICATIONS
-- AMD products are not designed or intended to be fail-
-- safe, or for use in any application requiring fail-safe
-- performance, such as life-support or safety devices or
-- systems, Class III medical devices, nuclear facilities,
-- applications related to the deployment of airbags, or any
-- other applications that could lead to death, personal
-- injury, or severe property or environmental damage
-- (individually and collectively, "Critical
-- Applications"). Customer assumes the sole risk and
-- liability of any use of AMD products in Critical
-- Applications, subject only to applicable laws and
-- regulations governing limitations on product liability.
-- 
-- THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
-- PART OF THIS FILE AT ALL TIMES.
-- 
-- DO NOT MODIFY THIS FILE.

-- IP VLNV: NEORV32:user:neorv32_vivado_ip:1.0
-- IP Revision: 1

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY design_1_neorv32_vivado_ip_0_2 IS
  PORT (
    clk : IN STD_LOGIC;
    resetn : IN STD_LOGIC;
    gpio_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
    gpio_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
    uart0_txd_o : OUT STD_LOGIC;
    uart0_rxd_i : IN STD_LOGIC;
    uart0_rtsn_o : OUT STD_LOGIC;
    uart0_ctsn_i : IN STD_LOGIC;
    irq_msi_i : IN STD_LOGIC;
    irq_mti_i : IN STD_LOGIC;
    irq_mei_i : IN STD_LOGIC
  );
END design_1_neorv32_vivado_ip_0_2;

ARCHITECTURE design_1_neorv32_vivado_ip_0_2_arch OF design_1_neorv32_vivado_ip_0_2 IS
  ATTRIBUTE DowngradeIPIdentifiedWarnings : STRING;
  ATTRIBUTE DowngradeIPIdentifiedWarnings OF design_1_neorv32_vivado_ip_0_2_arch: ARCHITECTURE IS "yes";
  COMPONENT neorv32_vivado_ip IS
    GENERIC (
      CLOCK_FREQUENCY : INTEGER;
      DUAL_CORE_EN : BOOLEAN;
      BOOT_MODE_SELECT : INTEGER;
      BOOT_ADDR_CUSTOM : STD_ULOGIC_VECTOR(31 DOWNTO 0);
      OCD_EN : BOOLEAN;
      OCD_NUM_HW_TRIGGERS : INTEGER;
      OCD_AUTHENTICATION : BOOLEAN;
      OCD_JEDEC_ID : STD_LOGIC_VECTOR(10 DOWNTO 0);
      RISCV_ISA_C : BOOLEAN;
      RISCV_ISA_E : BOOLEAN;
      RISCV_ISA_M : BOOLEAN;
      RISCV_ISA_U : BOOLEAN;
      RISCV_ISA_Zaamo : BOOLEAN;
      RISCV_ISA_Zalrsc : BOOLEAN;
      RISCV_ISA_Zba : BOOLEAN;
      RISCV_ISA_Zbb : BOOLEAN;
      RISCV_ISA_Zbkb : BOOLEAN;
      RISCV_ISA_Zbkc : BOOLEAN;
      RISCV_ISA_Zbkx : BOOLEAN;
      RISCV_ISA_Zbs : BOOLEAN;
      RISCV_ISA_Zcb : BOOLEAN;
      RISCV_ISA_Zfinx : BOOLEAN;
      RISCV_ISA_Zibi : BOOLEAN;
      RISCV_ISA_Zicntr : BOOLEAN;
      RISCV_ISA_Zicond : BOOLEAN;
      RISCV_ISA_Zihpm : BOOLEAN;
      RISCV_ISA_Zimop : BOOLEAN;
      RISCV_ISA_Zmmul : BOOLEAN;
      RISCV_ISA_Zknd : BOOLEAN;
      RISCV_ISA_Zkne : BOOLEAN;
      RISCV_ISA_Zknh : BOOLEAN;
      RISCV_ISA_Zksed : BOOLEAN;
      RISCV_ISA_Zksh : BOOLEAN;
      RISCV_ISA_Smcntrpmf : BOOLEAN;
      RISCV_ISA_Xcfu : BOOLEAN;
      CPU_CONSTT_BR_EN : BOOLEAN;
      CPU_FAST_MUL_EN : BOOLEAN;
      CPU_FAST_SHIFT_EN : BOOLEAN;
      CPU_RF_ARCH_SEL : INTEGER;
      PMP_NUM_REGIONS : INTEGER;
      PMP_MIN_GRANULARITY : INTEGER;
      PMP_TOR_MODE_EN : BOOLEAN;
      PMP_NAP_MODE_EN : BOOLEAN;
      HPM_NUM_CNTS : INTEGER;
      HPM_CNT_WIDTH : INTEGER;
      IMEM_EN : BOOLEAN;
      IMEM_BASE : STD_ULOGIC_VECTOR(31 DOWNTO 0);
      IMEM_SIZE : INTEGER;
      IMEM_OUTREG_EN : BOOLEAN;
      DMEM_EN : BOOLEAN;
      DMEM_BASE : STD_ULOGIC_VECTOR(31 DOWNTO 0);
      DMEM_SIZE : INTEGER;
      DMEM_OUTREG_EN : BOOLEAN;
      ICACHE_EN : BOOLEAN;
      ICACHE_NUM_BLOCKS : INTEGER;
      DCACHE_EN : BOOLEAN;
      DCACHE_NUM_BLOCKS : INTEGER;
      CACHE_BLOCK_SIZE : INTEGER;
      CACHE_BURSTS_EN : BOOLEAN;
      XBUS_EN : BOOLEAN;
      XBUS_TIMEOUT : INTEGER;
      XBUS_REGSTAGE_EN : BOOLEAN;
      IO_GPIO_EN : BOOLEAN;
      IO_GPIO_IN_NUM : INTEGER;
      IO_GPIO_OUT_NUM : INTEGER;
      IO_GPIO_DIR_EN : BOOLEAN;
      IO_GPIO_DIR_NUM : INTEGER;
      IO_CLINT_EN : BOOLEAN;
      IO_UART0_EN : BOOLEAN;
      IO_UART0_RX_FIFO : INTEGER;
      IO_UART0_TX_FIFO : INTEGER;
      IO_UART1_EN : BOOLEAN;
      IO_UART1_RX_FIFO : INTEGER;
      IO_UART1_TX_FIFO : INTEGER;
      IO_SPI_EN : BOOLEAN;
      IO_SPI_FIFO : INTEGER;
      IO_SDI_EN : BOOLEAN;
      IO_SDI_FIFO : INTEGER;
      IO_TWI_EN : BOOLEAN;
      IO_TWI_FIFO : INTEGER;
      IO_TWD_EN : BOOLEAN;
      IO_TWD_RX_FIFO : INTEGER;
      IO_TWD_TX_FIFO : INTEGER;
      IO_PWM_EN : BOOLEAN;
      IO_PWM_NUM : INTEGER;
      IO_WDT_EN : BOOLEAN;
      IO_TRNG_EN : BOOLEAN;
      IO_TRNG_FIFO : INTEGER;
      IO_TRNG_NUM_RO : INTEGER;
      IO_TRNG_NUM_INV : INTEGER;
      IO_TRNG_NUM_RBIT : INTEGER;
      IO_CFS_EN : BOOLEAN;
      IO_NEOLED_EN : BOOLEAN;
      IO_NEOLED_TX_FIFO : INTEGER;
      IO_GPTMR_EN : BOOLEAN;
      IO_GPTMR_NUM : INTEGER;
      IO_ONEWIRE_EN : BOOLEAN;
      IO_DMA_EN : BOOLEAN;
      IO_DMA_DSC_FIFO : INTEGER;
      IO_SLINK_EN : BOOLEAN;
      IO_SLINK_RX_FIFO : INTEGER;
      IO_SLINK_TX_FIFO : INTEGER;
      IO_TRACER_EN : BOOLEAN;
      IO_TRACER_BUFFER : INTEGER;
      IO_TRACER_SIMLOG_EN : BOOLEAN
    );
    PORT (
      clk : IN STD_LOGIC;
      resetn : IN STD_LOGIC;
      ocd_resetn : OUT STD_LOGIC;
      wdt_resetn : OUT STD_LOGIC;
      m_axi_awaddr : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
      m_axi_awlen : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
      m_axi_awsize : OUT STD_LOGIC_VECTOR(2 DOWNTO 0);
      m_axi_awburst : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
      m_axi_awcache : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
      m_axi_awprot : OUT STD_LOGIC_VECTOR(2 DOWNTO 0);
      m_axi_awvalid : OUT STD_LOGIC;
      m_axi_awready : IN STD_LOGIC;
      m_axi_wdata : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
      m_axi_wstrb : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
      m_axi_wlast : OUT STD_LOGIC;
      m_axi_wvalid : OUT STD_LOGIC;
      m_axi_wready : IN STD_LOGIC;
      m_axi_araddr : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
      m_axi_arlen : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
      m_axi_arsize : OUT STD_LOGIC_VECTOR(2 DOWNTO 0);
      m_axi_arburst : OUT STD_LOGIC_VECTOR(1 DOWNTO 0);
      m_axi_arcache : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
      m_axi_arprot : OUT STD_LOGIC_VECTOR(2 DOWNTO 0);
      m_axi_arvalid : OUT STD_LOGIC;
      m_axi_arready : IN STD_LOGIC;
      m_axi_rdata : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
      m_axi_rresp : IN STD_LOGIC_VECTOR(1 DOWNTO 0);
      m_axi_rlast : IN STD_LOGIC;
      m_axi_rvalid : IN STD_LOGIC;
      m_axi_rready : OUT STD_LOGIC;
      m_axi_bresp : IN STD_LOGIC_VECTOR(1 DOWNTO 0);
      m_axi_bvalid : IN STD_LOGIC;
      m_axi_bready : OUT STD_LOGIC;
      s0_axis_tdest : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
      s0_axis_tvalid : OUT STD_LOGIC;
      s0_axis_tready : IN STD_LOGIC;
      s0_axis_tdata : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
      s0_axis_tlast : OUT STD_LOGIC;
      s1_axis_tid : IN STD_LOGIC_VECTOR(3 DOWNTO 0);
      s1_axis_tvalid : IN STD_LOGIC;
      s1_axis_tready : OUT STD_LOGIC;
      s1_axis_tdata : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
      s1_axis_tlast : IN STD_LOGIC;
      jtag_tck_i : IN STD_LOGIC;
      jtag_tdi_i : IN STD_LOGIC;
      jtag_tdo_o : OUT STD_LOGIC;
      jtag_tms_i : IN STD_LOGIC;
      gpio_dir_o : OUT STD_LOGIC_VECTOR(0 DOWNTO 0);
      gpio_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
      gpio_i : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
      uart0_txd_o : OUT STD_LOGIC;
      uart0_rxd_i : IN STD_LOGIC;
      uart0_rtsn_o : OUT STD_LOGIC;
      uart0_ctsn_i : IN STD_LOGIC;
      uart1_txd_o : OUT STD_LOGIC;
      uart1_rxd_i : IN STD_LOGIC;
      uart1_rtsn_o : OUT STD_LOGIC;
      uart1_ctsn_i : IN STD_LOGIC;
      spi_clk_o : OUT STD_LOGIC;
      spi_dat_o : OUT STD_LOGIC;
      spi_dat_i : IN STD_LOGIC;
      spi_csn_o : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
      sdi_clk_i : IN STD_LOGIC;
      sdi_dat_o : OUT STD_LOGIC;
      sdi_dat_i : IN STD_LOGIC;
      sdi_csn_i : IN STD_LOGIC;
      twi_sda_i : IN STD_LOGIC;
      twi_sda_o : OUT STD_LOGIC;
      twi_scl_i : IN STD_LOGIC;
      twi_scl_o : OUT STD_LOGIC;
      twd_sda_i : IN STD_LOGIC;
      twd_sda_o : OUT STD_LOGIC;
      twd_scl_i : IN STD_LOGIC;
      onewire_i : IN STD_LOGIC;
      onewire_o : OUT STD_LOGIC;
      pwm_o : OUT STD_LOGIC_VECTOR(0 DOWNTO 0);
      cfs_in_i : IN STD_LOGIC_VECTOR(255 DOWNTO 0);
      cfs_out_o : OUT STD_LOGIC_VECTOR(255 DOWNTO 0);
      neoled_o : OUT STD_LOGIC;
      mtime_time_o : OUT STD_LOGIC_VECTOR(63 DOWNTO 0);
      irq_msi_i : IN STD_LOGIC;
      irq_mti_i : IN STD_LOGIC;
      irq_mei_i : IN STD_LOGIC
    );
  END COMPONENT neorv32_vivado_ip;
  ATTRIBUTE X_INTERFACE_INFO : STRING;
  ATTRIBUTE X_INTERFACE_MODE : STRING;
  ATTRIBUTE X_INTERFACE_PARAMETER : STRING;
  ATTRIBUTE X_INTERFACE_INFO OF clk: SIGNAL IS "xilinx.com:signal:clock:1.0 clk CLK";
  ATTRIBUTE X_INTERFACE_MODE OF clk: SIGNAL IS "slave clk";
  ATTRIBUTE X_INTERFACE_PARAMETER OF clk: SIGNAL IS "XIL_INTERFACENAME clk, ASSOCIATED_BUSIF s0_axis:s1_axis:m_axi, ASSOCIATED_RESET resetn, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN /clk_wiz_0_clk_out1, INSERT_VIP 0";
  ATTRIBUTE X_INTERFACE_INFO OF resetn: SIGNAL IS "xilinx.com:signal:reset:1.0 resetn RST";
  ATTRIBUTE X_INTERFACE_MODE OF resetn: SIGNAL IS "slave resetn";
  ATTRIBUTE X_INTERFACE_PARAMETER OF resetn: SIGNAL IS "XIL_INTERFACENAME resetn, POLARITY ACTIVE_LOW, INSERT_VIP 0";
BEGIN
  U0 : neorv32_vivado_ip
    GENERIC MAP (
      CLOCK_FREQUENCY => 100000000,
      DUAL_CORE_EN => false,
      BOOT_MODE_SELECT => 0,
      BOOT_ADDR_CUSTOM => X"00000000",
      OCD_EN => false,
      OCD_NUM_HW_TRIGGERS => 0,
      OCD_AUTHENTICATION => false,
      OCD_JEDEC_ID => B"00000000000",
      RISCV_ISA_C => false,
      RISCV_ISA_E => false,
      RISCV_ISA_M => false,
      RISCV_ISA_U => false,
      RISCV_ISA_Zaamo => false,
      RISCV_ISA_Zalrsc => false,
      RISCV_ISA_Zba => false,
      RISCV_ISA_Zbb => false,
      RISCV_ISA_Zbkb => false,
      RISCV_ISA_Zbkc => false,
      RISCV_ISA_Zbkx => false,
      RISCV_ISA_Zbs => false,
      RISCV_ISA_Zcb => false,
      RISCV_ISA_Zfinx => false,
      RISCV_ISA_Zibi => false,
      RISCV_ISA_Zicntr => false,
      RISCV_ISA_Zicond => false,
      RISCV_ISA_Zihpm => false,
      RISCV_ISA_Zimop => false,
      RISCV_ISA_Zmmul => false,
      RISCV_ISA_Zknd => false,
      RISCV_ISA_Zkne => false,
      RISCV_ISA_Zknh => false,
      RISCV_ISA_Zksed => false,
      RISCV_ISA_Zksh => false,
      RISCV_ISA_Smcntrpmf => false,
      RISCV_ISA_Xcfu => false,
      CPU_CONSTT_BR_EN => false,
      CPU_FAST_MUL_EN => false,
      CPU_FAST_SHIFT_EN => false,
      CPU_RF_ARCH_SEL => 1,
      PMP_NUM_REGIONS => 0,
      PMP_MIN_GRANULARITY => 4,
      PMP_TOR_MODE_EN => false,
      PMP_NAP_MODE_EN => false,
      HPM_NUM_CNTS => 0,
      HPM_CNT_WIDTH => 40,
      IMEM_EN => true,
      IMEM_BASE => X"00000000",
      IMEM_SIZE => 16384,
      IMEM_OUTREG_EN => false,
      DMEM_EN => true,
      DMEM_BASE => X"80000000",
      DMEM_SIZE => 16384,
      DMEM_OUTREG_EN => false,
      ICACHE_EN => false,
      ICACHE_NUM_BLOCKS => 4,
      DCACHE_EN => false,
      DCACHE_NUM_BLOCKS => 4,
      CACHE_BLOCK_SIZE => 64,
      CACHE_BURSTS_EN => true,
      XBUS_EN => false,
      XBUS_TIMEOUT => 2048,
      XBUS_REGSTAGE_EN => false,
      IO_GPIO_EN => true,
      IO_GPIO_IN_NUM => 8,
      IO_GPIO_OUT_NUM => 8,
      IO_GPIO_DIR_EN => false,
      IO_GPIO_DIR_NUM => 1,
      IO_CLINT_EN => false,
      IO_UART0_EN => true,
      IO_UART0_RX_FIFO => 1,
      IO_UART0_TX_FIFO => 1,
      IO_UART1_EN => false,
      IO_UART1_RX_FIFO => 1,
      IO_UART1_TX_FIFO => 1,
      IO_SPI_EN => false,
      IO_SPI_FIFO => 1,
      IO_SDI_EN => false,
      IO_SDI_FIFO => 1,
      IO_TWI_EN => false,
      IO_TWI_FIFO => 1,
      IO_TWD_EN => false,
      IO_TWD_RX_FIFO => 1,
      IO_TWD_TX_FIFO => 1,
      IO_PWM_EN => false,
      IO_PWM_NUM => 1,
      IO_WDT_EN => false,
      IO_TRNG_EN => false,
      IO_TRNG_FIFO => 1,
      IO_TRNG_NUM_RO => 3,
      IO_TRNG_NUM_INV => 5,
      IO_TRNG_NUM_RBIT => 64,
      IO_CFS_EN => false,
      IO_NEOLED_EN => false,
      IO_NEOLED_TX_FIFO => 1,
      IO_GPTMR_EN => false,
      IO_GPTMR_NUM => 1,
      IO_ONEWIRE_EN => false,
      IO_DMA_EN => false,
      IO_DMA_DSC_FIFO => 4,
      IO_SLINK_EN => false,
      IO_SLINK_RX_FIFO => 1,
      IO_SLINK_TX_FIFO => 1,
      IO_TRACER_EN => false,
      IO_TRACER_BUFFER => 1,
      IO_TRACER_SIMLOG_EN => false
    )
    PORT MAP (
      clk => clk,
      resetn => resetn,
      m_axi_awready => '0',
      m_axi_wready => '0',
      m_axi_arready => '0',
      m_axi_rdata => X"00000000",
      m_axi_rresp => STD_LOGIC_VECTOR(TO_UNSIGNED(0, 2)),
      m_axi_rlast => '0',
      m_axi_rvalid => '0',
      m_axi_bresp => STD_LOGIC_VECTOR(TO_UNSIGNED(0, 2)),
      m_axi_bvalid => '0',
      s0_axis_tready => '0',
      s1_axis_tid => X"0",
      s1_axis_tvalid => '0',
      s1_axis_tdata => X"00000000",
      s1_axis_tlast => '0',
      jtag_tck_i => '0',
      jtag_tdi_i => '0',
      jtag_tms_i => '0',
      gpio_o => gpio_o,
      gpio_i => gpio_i,
      uart0_txd_o => uart0_txd_o,
      uart0_rxd_i => uart0_rxd_i,
      uart0_rtsn_o => uart0_rtsn_o,
      uart0_ctsn_i => uart0_ctsn_i,
      uart1_rxd_i => '0',
      uart1_ctsn_i => '0',
      spi_dat_i => '0',
      sdi_clk_i => '0',
      sdi_dat_i => '0',
      sdi_csn_i => '0',
      twi_sda_i => '0',
      twi_scl_i => '0',
      twd_sda_i => '0',
      twd_scl_i => '0',
      onewire_i => '0',
      cfs_in_i => STD_LOGIC_VECTOR(TO_UNSIGNED(0, 256)),
      irq_msi_i => irq_msi_i,
      irq_mti_i => irq_mti_i,
      irq_mei_i => irq_mei_i
    );
END design_1_neorv32_vivado_ip_0_2_arch;
