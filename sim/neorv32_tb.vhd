-- ================================================================================ --
-- NEORV32 - Default Processor Testbench                                            --
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
use ieee.math_real.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_tb is
  generic (
    -- processor --
    CLOCK_FREQUENCY     : natural                        := 100_000_000; -- clock frequency of clk_i in Hz
    DUAL_CORE_EN        : boolean                        := true;        -- enable dual-core homogeneous SMP
    BOOT_MODE_SELECT    : natural range 0 to 2           := 2;           -- boot from pre-initialized IMEM
    BOOT_ADDR_CUSTOM    : std_ulogic_vector(31 downto 0) := x"00000000"; -- custom CPU boot address (if boot_config = 1)
    RISCV_ISA_C         : boolean                        := true;        -- compressed extension
    RISCV_ISA_E         : boolean                        := false;       -- embedded RF extension
    RISCV_ISA_M         : boolean                        := true;        -- mul/div extension
    RISCV_ISA_U         : boolean                        := true;        -- user mode extension
    RISCV_ISA_Zaamo     : boolean                        := true;        -- atomic read-modify-write operations extension
    RISCV_ISA_Zalrsc    : boolean                        := true;        -- atomic reservation-set operations extension
    RISCV_ISA_Zcb       : boolean                        := false;       -- additional code size reduction instructions
    RISCV_ISA_Zba       : boolean                        := true;        -- shifted-add bit-manipulation extension
    RISCV_ISA_Zbb       : boolean                        := true;        -- basic bit-manipulation extension
    RISCV_ISA_Zbkb      : boolean                        := true;        -- bit-manipulation instructions for cryptography
    RISCV_ISA_Zbkc      : boolean                        := true;        -- carry-less multiplication instructions
    RISCV_ISA_Zbkx      : boolean                        := true;        -- cryptography crossbar permutation extension
    RISCV_ISA_Zbs       : boolean                        := true;        -- single-bit bit-manipulation extension
    RISCV_ISA_Zfinx     : boolean                        := true;        -- 32-bit floating-point extension
    RISCV_ISA_Zibi      : boolean                        := true;        -- branch with immediate
    RISCV_ISA_Zicntr    : boolean                        := true;        -- base counters
    RISCV_ISA_Zicond    : boolean                        := true;        -- integer conditional operations
    RISCV_ISA_Zihpm     : boolean                        := true;        -- hardware performance monitors
    RISCV_ISA_Zimop    : boolean                         := true;        -- may-be-operations
    RISCV_ISA_Zknd      : boolean                        := true;        -- cryptography NIST AES decryption extension
    RISCV_ISA_Zkne      : boolean                        := true;        -- cryptography NIST AES encryption extension
    RISCV_ISA_Zknh      : boolean                        := true;        -- cryptography NIST hash extension
    RISCV_ISA_Zksed     : boolean                        := true;        -- ShangMi block cipher extension
    RISCV_ISA_Zksh      : boolean                        := true;        -- ShangMi hash extension
    RISCV_ISA_Zmmul     : boolean                        := true;        -- multiply-only M sub-extension
    RISCV_ISA_Zxcfu     : boolean                        := true;        -- custom (instr.) functions unit
    CPU_CONSTT_BR_EN    : boolean                        := false;       -- constant-time branches
    CPU_FAST_MUL_EN     : boolean                        := true;        -- use DSPs for M extension's multiplier
    CPU_FAST_SHIFT_EN   : boolean                        := true;        -- use barrel shifter for shift operations
    CPU_RF_HW_RST_EN    : boolean                        := false;       -- implement full hardware reset for register file
    IMEM_EN             : boolean                        := true;        -- implement processor-internal instruction memory
    IMEM_SIZE           : natural                        := 32*1024;     -- size of processor-internal instruction memory in bytes (use a power of 2)
    DMEM_EN             : boolean                        := true;        -- implement processor-internal data memory
    DMEM_SIZE           : natural                        := 8*1024;      -- size of processor-internal data memory in bytes (use a power of 2)
    ICACHE_EN           : boolean                        := true;        -- implement instruction cache
    ICACHE_NUM_BLOCKS   : natural range 1 to 4096        := 64;          -- i-cache: number of blocks (min 1), has to be a power of 2
    DCACHE_EN           : boolean                        := true;        -- implement data cache
    DCACHE_NUM_BLOCKS   : natural range 1 to 4096        := 32;          -- d-cache: number of blocks (min 1), has to be a power of 2
    CACHE_BLOCK_SIZE    : natural range 8 to 1024        := 32;          -- i-cache/d-cache: block size in bytes (min 8), has to be a power of 2
    CACHE_BURSTS_EN     : boolean                        := true;        -- enable issuing of burst transfer for cache update
    TRACE_LOG_EN        : boolean                        := true;        -- write full trace log to file
    -- external memory A --
    EXT_MEM_A_EN        : boolean                        := false;       -- enable memory
    EXT_MEM_A_BASE      : std_ulogic_vector(31 downto 0) := x"00000000"; -- base address, has to be word-aligned
    EXT_MEM_A_SIZE      : natural                        := 64;          -- memory size in bytes, min 4
    EXT_MEM_A_LATE      : natural range 1 to 4096        := 20;          -- access latency cycles
    EXT_MEM_A_FILE      : string                         := "";          -- memory initialization file (plain HEX), no initialization if empty
    -- external memory B --
    EXT_MEM_B_EN        : boolean                        := false;       -- enable memory
    EXT_MEM_B_BASE      : std_ulogic_vector(31 downto 0) := x"80000000"; -- base address, has to be word-aligned
    EXT_MEM_B_SIZE      : natural                        := 64;          -- memory size in bytes, min 4
    EXT_MEM_B_LATE      : natural range 1 to 4096        := 40;          -- access latency cycles
    EXT_MEM_B_FILE      : string                         := ""           -- memory initialization file (plain HEX), no initialization if empty
  );
end neorv32_tb;

architecture neorv32_tb_rtl of neorv32_tb is

  -- generators --
  signal clk_en, clk_gen, rst_gen : std_ulogic;
  constant f_period_c : time := (1 sec) / CLOCK_FREQUENCY;

  -- IO connection --
  signal uart0_txd, uart0_ctsn, uart1_txd, uart1_ctsn : std_ulogic;
  signal gpio : std_ulogic_vector(31 downto 0);
  signal i2c_scl, i2c_sda : std_logic;
  signal twi_scl_i, twi_scl_o, twi_sda_i, twi_sda_o : std_ulogic;
  signal twd_scl_i, twd_scl_o, twd_sda_i, twd_sda_o : std_ulogic;
  signal onewire : std_logic;
  signal onewire_i, onewire_o : std_ulogic;
  signal spi_csn : std_ulogic_vector(7 downto 0);
  signal spi_di, spi_do, spi_clk : std_ulogic;
  signal sdi_di, sdi_do, sdi_clk, sdi_csn : std_ulogic;
  signal msi, mei, mti : std_ulogic;

  -- slink --
  type slink_t is record
    data  : std_ulogic_vector(31 downto 0); -- data
    addr  : std_ulogic_vector(3 downto 0); -- source/destination ID
    valid : std_ulogic; -- source valid
    last  : std_ulogic; -- last element of packet
    ready : std_ulogic; -- sink ready
  end record;
  signal slink_tx, slink_rx : slink_t;

  -- XBUS --
  signal xbus_core_req, xbus_ext_mem_a_req, xbus_ext_mem_b_req, xbus_mmio_req, xbus_trig_req : xbus_req_t;
  signal xbus_core_rsp, xbus_ext_mem_a_rsp, xbus_ext_mem_b_rsp, xbus_mmio_rsp, xbus_trig_rsp : xbus_rsp_t;
  signal xbus_fmem_data_req, xbus_fmem_tag_req : xbus_req_t;
  signal xbus_fmem_data_rsp, xbus_fmem_tag_rsp : xbus_rsp_t;

begin

  -- Clock & Reset Generators ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rst_gen <= '0', '1' after 30*(f_period_c/2);
  clk_en  <= '0', '1' after 60*(f_period_c/2);
  clk_gen <= (not clk_gen) and clk_en after (f_period_c/2);


  -- The Core of the Problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_top_inst: neorv32_top
  generic map (
    -- General --
    CLOCK_FREQUENCY     => CLOCK_FREQUENCY,
    TRACE_PORT_EN       => true,
    DUAL_CORE_EN        => DUAL_CORE_EN,
    -- Boot Configuration --
    BOOT_MODE_SELECT    => BOOT_MODE_SELECT,
    BOOT_ADDR_CUSTOM    => BOOT_ADDR_CUSTOM,
    -- On-Chip Debugger (OCD) --
    OCD_EN              => true,
    OCD_NUM_HW_TRIGGERS => 3,
    OCD_AUTHENTICATION  => true,
    -- RISC-V CPU Extensions --
    RISCV_ISA_C         => RISCV_ISA_C,
    RISCV_ISA_E         => RISCV_ISA_E,
    RISCV_ISA_M         => RISCV_ISA_M,
    RISCV_ISA_U         => RISCV_ISA_U,
    RISCV_ISA_Zaamo     => RISCV_ISA_Zaamo,
    RISCV_ISA_Zalrsc    => RISCV_ISA_Zalrsc,
    RISCV_ISA_Zcb       => RISCV_ISA_Zcb,
    RISCV_ISA_Zba       => RISCV_ISA_Zba,
    RISCV_ISA_Zbb       => RISCV_ISA_Zbb,
    RISCV_ISA_Zbkb      => RISCV_ISA_Zbkb,
    RISCV_ISA_Zbkc      => RISCV_ISA_Zbkc,
    RISCV_ISA_Zbkx      => RISCV_ISA_Zbkx,
    RISCV_ISA_Zbs       => RISCV_ISA_Zbs,
    RISCV_ISA_Zfinx     => RISCV_ISA_Zfinx,
    RISCV_ISA_Zibi      => RISCV_ISA_Zibi,
    RISCV_ISA_Zicntr    => RISCV_ISA_Zicntr,
    RISCV_ISA_Zicond    => RISCV_ISA_Zicond,
    RISCV_ISA_Zihpm     => RISCV_ISA_Zihpm,
    RISCV_ISA_Zimop     => RISCV_ISA_Zimop,
    RISCV_ISA_Zknd      => RISCV_ISA_Zknd,
    RISCV_ISA_Zkne      => RISCV_ISA_Zkne,
    RISCV_ISA_Zknh      => RISCV_ISA_Zknh,
    RISCV_ISA_Zksed     => RISCV_ISA_Zksed,
    RISCV_ISA_Zksh      => RISCV_ISA_Zksh,
    RISCV_ISA_Zmmul     => RISCV_ISA_Zmmul,
    RISCV_ISA_Zxcfu     => RISCV_ISA_Zxcfu,
    -- Extension Options --
    CPU_CONSTT_BR_EN    => CPU_CONSTT_BR_EN,
    CPU_FAST_MUL_EN     => CPU_FAST_MUL_EN,
    CPU_FAST_SHIFT_EN   => CPU_FAST_SHIFT_EN,
    CPU_RF_HW_RST_EN    => CPU_RF_HW_RST_EN,
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS     => 5,
    PMP_MIN_GRANULARITY => 4,
    PMP_TOR_MODE_EN     => true,
    PMP_NAP_MODE_EN     => true,
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS        => 12,
    HPM_CNT_WIDTH       => 40,
    -- Internal Instruction memory --
    IMEM_EN             => IMEM_EN,
    IMEM_SIZE           => IMEM_SIZE,
    IMEM_OUTREG_EN      => true,
    -- Internal Data memory --
    DMEM_EN             => DMEM_EN,
    DMEM_SIZE           => DMEM_SIZE,
    DMEM_OUTREG_EN      => true,
    -- CPU Caches --
    ICACHE_EN           => ICACHE_EN,
    ICACHE_NUM_BLOCKS   => ICACHE_NUM_BLOCKS,
    DCACHE_EN           => DCACHE_EN,
    DCACHE_NUM_BLOCKS   => DCACHE_NUM_BLOCKS,
    CACHE_BLOCK_SIZE    => CACHE_BLOCK_SIZE,
    CACHE_BURSTS_EN     => CACHE_BURSTS_EN,
    -- External bus interface --
    XBUS_EN             => true,
    XBUS_TIMEOUT        => 2048,
    XBUS_REGSTAGE_EN    => true,
    -- Processor peripherals --
    IO_GPIO_NUM         => 32,
    IO_CLINT_EN         => true,
    IO_UART0_EN         => true,
    IO_UART0_RX_FIFO    => 32,
    IO_UART0_TX_FIFO    => 32,
    IO_UART1_EN         => true,
    IO_UART1_RX_FIFO    => 1,
    IO_UART1_TX_FIFO    => 1,
    IO_SPI_EN           => true,
    IO_SPI_FIFO         => 4,
    IO_SDI_EN           => true,
    IO_SDI_FIFO         => 4,
    IO_TWI_EN           => true,
    IO_TWI_FIFO         => 4,
    IO_TWD_EN           => true,
    IO_TWD_RX_FIFO      => 4,
    IO_TWD_TX_FIFO      => 4,
    IO_PWM_NUM          => 8,
    IO_WDT_EN           => true,
    IO_TRNG_EN          => true,
    IO_TRNG_FIFO        => 4,
    IO_CFS_EN           => true,
    IO_NEOLED_EN        => true,
    IO_NEOLED_TX_FIFO   => 8,
    IO_GPTMR_NUM        => 4,
    IO_ONEWIRE_EN       => true,
    IO_ONEWIRE_FIFO     => 8,
    IO_DMA_EN           => true,
    IO_DMA_DSC_FIFO     => 8,
    IO_SLINK_EN         => true,
    IO_SLINK_RX_FIFO    => 4,
    IO_SLINK_TX_FIFO    => 1,
    IO_TRACER_EN        => true,
    IO_TRACER_BUFFER    => 32,
    IO_TRACER_SIMLOG_EN => TRACE_LOG_EN
  )
  port map (
    -- Global control --
    clk_i          => clk_gen,
    rstn_i         => rst_gen,
    rstn_ocd_o     => open,
    rstn_wdt_o     => open,
    -- Execution trace --
    trace_cpu0_o   => open,
    trace_cpu1_o   => open,
    -- JTAG on-chip debugger interface --
    jtag_tck_i     => '0',
    jtag_tdi_i     => '0',
    jtag_tdo_o     => open,
    jtag_tms_i     => '0',
    -- External bus interface --
    xbus_adr_o     => xbus_core_req.addr,
    xbus_dat_o     => xbus_core_req.data,
    xbus_cti_o     => xbus_core_req.cti,
    xbus_tag_o     => xbus_core_req.tag,
    xbus_we_o      => xbus_core_req.we,
    xbus_sel_o     => xbus_core_req.sel,
    xbus_stb_o     => xbus_core_req.stb,
    xbus_cyc_o     => xbus_core_req.cyc,
    xbus_dat_i     => xbus_core_rsp.data,
    xbus_ack_i     => xbus_core_rsp.ack,
    xbus_err_i     => xbus_core_rsp.err,
    -- Stream Link Interface --
    slink_rx_dat_i => slink_rx.data,
    slink_rx_src_i => slink_rx.addr,
    slink_rx_val_i => slink_rx.valid,
    slink_rx_lst_i => slink_rx.last,
    slink_rx_rdy_o => slink_rx.ready,
    slink_tx_dat_o => slink_tx.data,
    slink_tx_dst_o => slink_tx.addr,
    slink_tx_val_o => slink_tx.valid,
    slink_tx_lst_o => slink_tx.last,
    slink_tx_rdy_i => slink_tx.ready,
    -- GPIO --
    gpio_o         => gpio,
    gpio_i         => gpio,
    -- primary UART0 --
    uart0_txd_o    => uart0_txd,
    uart0_rxd_i    => uart1_txd,
    uart0_rtsn_o   => uart0_ctsn,
    uart0_ctsn_i   => uart1_ctsn,
    -- secondary UART1 --
    uart1_txd_o    => uart1_txd,
    uart1_rxd_i    => uart0_txd,
    uart1_rtsn_o   => uart1_ctsn,
    uart1_ctsn_i   => uart0_ctsn,
    -- SPI --
    spi_clk_o      => spi_clk,
    spi_dat_o      => spi_do,
    spi_dat_i      => spi_di,
    spi_csn_o      => spi_csn,
    -- SDI --
    sdi_clk_i      => sdi_clk,
    sdi_dat_o      => sdi_do,
    sdi_dat_i      => sdi_di,
    sdi_csn_i      => sdi_csn,
    -- TWI --
    twi_sda_i      => twi_sda_i,
    twi_sda_o      => twi_sda_o,
    twi_scl_i      => twi_scl_i,
    twi_scl_o      => twi_scl_o,
    -- TWD --
    twd_sda_i      => twd_sda_i,
    twd_sda_o      => twd_sda_o,
    twd_scl_i      => twd_scl_i,
    twd_scl_o      => twd_scl_o,
    -- 1-Wire Interface --
    onewire_i      => onewire_i,
    onewire_o      => onewire_o,
    -- PWM --
    pwm_o          => open,
    -- Custom Functions Subsystem IO --
    cfs_in_i       => (others => '0'),
    cfs_out_o      => open,
    -- NeoPixel-compatible smart LED interface --
    neoled_o       => open,
    -- Machine timer system time --
    mtime_time_o   => open,
    -- CPU Interrupts --
    mtime_irq_i    => mti,
    msw_irq_i      => msi,
    mext_irq_i     => mei
  );


  -- Two-Wire Bus - Tri-State Drivers (modules can only actively pull the signals low) ------
  -- -------------------------------------------------------------------------------------------
  i2c_sda   <= '0' when (twi_sda_o = '0') else 'Z';
  i2c_scl   <= '0' when (twi_scl_o = '0') else 'Z';
  twi_sda_i <= std_ulogic(i2c_sda); -- sense input
  twi_scl_i <= std_ulogic(i2c_scl); -- sense input

  i2c_sda   <= '0' when (twd_sda_o = '0') else 'Z';
  i2c_scl   <= '0' when (twd_scl_o = '0') else 'Z';
  twd_sda_i <= std_ulogic(i2c_sda); -- sense input
  twd_scl_i <= std_ulogic(i2c_scl); -- sense input

  -- I2C bus termination with weak pull-ups --
  i2c_scl <= 'H';
  i2c_sda <= 'H';


  -- One-Wire Bus - Tri-State Driver (module can only actively pull the signals low) --------
  -- -------------------------------------------------------------------------------------------
  onewire   <= '0' when (onewire_o = '0') else 'Z';
  onewire_i <= std_ulogic(onewire); -- sense input

  -- 1-Wire bus termination with weak pull-up --
  onewire <= 'H';


  -- SPI/SDI Loop-Back ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  sdi_clk <= spi_clk;
  sdi_csn <= spi_csn(7);
  sdi_di  <= spi_do;
  spi_di  <= sdi_do when (spi_csn(7) = '0') else spi_do;


  -- Stream-Link FIFO Buffer ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  slink_buffer: entity neorv32.neorv32_prim_fifo
  generic map (
    AWIDTH  => 0,
    DWIDTH  => 32+4+1,
    OUTGATE => true
  )
  port map (
    -- global control --
    clk_i                 => clk_gen,
    rstn_i                => rst_gen,
    clear_i               => '0',
    -- write port --
    wdata_i(31 downto  0) => slink_tx.data,
    wdata_i(35 downto 32) => slink_tx.addr,
    wdata_i(36)           => slink_tx.last,
    we_i                  => slink_tx.valid,
    free_o                => slink_tx.ready,
    -- read port --
    re_i                  => slink_rx.ready,
    rdata_o(31 downto  0) => slink_rx.data,
    rdata_o(35 downto 32) => slink_rx.addr,
    rdata_o(36)           => slink_rx.last,
    avail_o               => slink_rx.valid
  );


  -- UART Simulation Receivers --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  sim_rx_uart0: entity work.sim_uart_rx
  generic map (
    NAME => "tb.uart0_rx",
    FCLK => real(CLOCK_FREQUENCY),
    BAUD => real(19200)
  )
  port map (
    clk => clk_gen,
    rxd => uart0_txd
  );

  sim_rx_uart1: entity work.sim_uart_rx
  generic map (
    NAME => "tb.uart1_rx",
    FCLK => real(CLOCK_FREQUENCY),
    BAUD => real(19200)
  )
  port map (
    clk => clk_gen,
    rxd => uart1_txd
  );


  -- XBUS / Wishbone Interconnect -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  xbus_interconnect: entity work.xbus_gateway
  generic map (
    -- device address size in bytes and base address --
    DEV_0_EN => EXT_MEM_A_EN, DEV_0_SIZE => EXT_MEM_A_SIZE, DEV_0_BASE => EXT_MEM_A_BASE,
    DEV_1_EN => EXT_MEM_B_EN, DEV_1_SIZE => EXT_MEM_B_SIZE, DEV_1_BASE => EXT_MEM_B_BASE,
    DEV_2_EN => true,         DEV_2_SIZE =>              8, DEV_2_BASE => x"F0000000",
    DEV_3_EN => true,         DEV_3_SIZE =>              4, DEV_3_BASE => x"FF000000",
    DEV_4_EN => true,         DEV_4_SIZE =>             16, DEV_4_BASE => x"B0000000",
    DEV_5_EN => true,         DEV_5_SIZE =>             16, DEV_5_BASE => x"FF100000",
    DEV_6_EN => false,        DEV_6_SIZE =>              0, DEV_6_BASE => (others => '0'), -- unused
    DEV_7_EN => false,        DEV_7_SIZE =>              0, DEV_7_BASE => (others => '0')  -- unused
  )
  port map (
    -- host port --
    host_req_i  => xbus_core_req,
    host_rsp_o  => xbus_core_rsp,
    -- device ports --
    dev_0_req_o => xbus_ext_mem_a_req, dev_0_rsp_i => xbus_ext_mem_a_rsp,
    dev_1_req_o => xbus_ext_mem_b_req, dev_1_rsp_i => xbus_ext_mem_b_rsp,
    dev_2_req_o => xbus_mmio_req,      dev_2_rsp_i => xbus_mmio_rsp,
    dev_3_req_o => xbus_trig_req,      dev_3_rsp_i => xbus_trig_rsp,
    dev_4_req_o => xbus_fmem_data_req, dev_4_rsp_i => xbus_fmem_data_rsp,
    dev_5_req_o => xbus_fmem_tag_req,  dev_5_rsp_i => xbus_fmem_tag_rsp,
    dev_6_req_o => open,               dev_6_rsp_i => xbus_rsp_terminate_c, -- unused
    dev_7_req_o => open,               dev_7_rsp_i => xbus_rsp_terminate_c  -- unused
  );


  -- XBUS: External Memory A (cached) -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  xbus_external_memory_a_enable:
  if EXT_MEM_A_EN generate
    xbus_external_memory_a: entity work.xbus_memory
    generic map (
      MEM_SIZE => EXT_MEM_A_SIZE,
      MEM_LATE => EXT_MEM_A_LATE,
      MEM_FILE => EXT_MEM_A_FILE
    )
    port map (
      clk_i      => clk_gen,
      rstn_i     => rst_gen,
      xbus_req_i => xbus_ext_mem_a_req,
      xbus_rsp_o => xbus_ext_mem_a_rsp
    );
  end generate;

  xbus_external_memory_a_disable:
  if not EXT_MEM_A_EN generate
    xbus_ext_mem_a_rsp <= xbus_rsp_terminate_c;
  end generate;


  -- XBUS: External Memory B (cached) -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  xbus_external_memory_b_enable:
  if EXT_MEM_B_EN generate
    xbus_external_memory_b: entity work.xbus_memory
    generic map (
      MEM_SIZE => EXT_MEM_B_SIZE,
      MEM_LATE => EXT_MEM_B_LATE,
      MEM_FILE => EXT_MEM_B_FILE
    )
    port map (
      clk_i      => clk_gen,
      rstn_i     => rst_gen,
      xbus_req_i => xbus_ext_mem_b_req,
      xbus_rsp_o => xbus_ext_mem_b_rsp
    );
  end generate;

  xbus_external_memory_b_disable:
  if not EXT_MEM_B_EN generate
    xbus_ext_mem_b_rsp <= xbus_rsp_terminate_c;
  end generate;


  -- XBUS: External Memory-Mapped IO (uncached) ---------------------------------------------
  -- -------------------------------------------------------------------------------------------
  xbus_mmio: entity work.xbus_memory
  generic map (
    MEM_SIZE => 8,
    MEM_LATE => 16,
    MEM_FILE => "" -- no initialization
  )
  port map (
    clk_i      => clk_gen,
    rstn_i     => rst_gen,
    xbus_req_i => xbus_mmio_req,
    xbus_rsp_o => xbus_mmio_rsp
  );


  -- XBUS: External IRQ Trigger -------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  xbus_irq_trigger: process(rst_gen, clk_gen)
  begin
    if (rst_gen = '0') then
      msi <= '0';
      mti <= '0';
      mei <= '0';
    elsif rising_edge(clk_gen) then
      -- bus response defaults --
      xbus_trig_rsp.data <= (others => '0');
      xbus_trig_rsp.ack  <= '0';
      xbus_trig_rsp.err  <= '0';
      -- trigger RISC-V platform IRQs --
      if ((xbus_trig_req.cyc and xbus_trig_req.stb and xbus_trig_req.we and and_reduce_f(xbus_trig_req.sel)) = '1') then
        xbus_trig_rsp.ack <= '1';
        msi <= xbus_trig_req.data(03); -- machine software interrupt
        mti <= xbus_trig_req.data(07); -- machine timer interrupt
        mei <= xbus_trig_req.data(11); -- machine external interrupt
      end if;
    end if;
  end process xbus_irq_trigger;


  -- XBUS: Bus-Error-Test-Memory ------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  xbus_fmem_inst: entity work.xbus_fmem
  generic map (
    MEM_SIZE => 16
  )
  port map (
    clk_i     => clk_gen,
    rstn_i    => rst_gen,
    tag_req_i => xbus_fmem_tag_req,
    tag_rsp_o => xbus_fmem_tag_rsp,
    mem_req_i => xbus_fmem_data_req,
    mem_rsp_o => xbus_fmem_data_rsp
  );


end neorv32_tb_rtl;
