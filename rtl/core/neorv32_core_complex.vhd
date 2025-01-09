-- ================================================================================ --
-- NEORV32 SoC - Core Complex Top                                                   --
-- -------------------------------------------------------------------------------- --
-- CPU core + optional L1 I-cache + optional L1 D-cache + bus switch                --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_core_complex is
  generic (
    -- General --
    HART_ID             : natural range 0 to 3;
    NUM_HARTS           : natural range 1 to 4;
    VENDOR_ID           : std_ulogic_vector(31 downto 0);
    BOOT_ADDR           : std_ulogic_vector(31 downto 0);
    DEBUG_PARK_ADDR     : std_ulogic_vector(31 downto 0);
    DEBUG_EXC_ADDR      : std_ulogic_vector(31 downto 0);
    -- RISC-V ISA Extensions --
    RISCV_ISA_C         : boolean;
    RISCV_ISA_E         : boolean;
    RISCV_ISA_M         : boolean;
    RISCV_ISA_U         : boolean;
    RISCV_ISA_Zaamo     : boolean;
    RISCV_ISA_Zba       : boolean;
    RISCV_ISA_Zbb       : boolean;
    RISCV_ISA_Zbkb      : boolean;
    RISCV_ISA_Zbkc      : boolean;
    RISCV_ISA_Zbkx      : boolean;
    RISCV_ISA_Zbs       : boolean;
    RISCV_ISA_Zfinx     : boolean;
    RISCV_ISA_Zicntr    : boolean;
    RISCV_ISA_Zicond    : boolean;
    RISCV_ISA_Zihpm     : boolean;
    RISCV_ISA_Zknd      : boolean;
    RISCV_ISA_Zkne      : boolean;
    RISCV_ISA_Zknh      : boolean;
    RISCV_ISA_Zksed     : boolean;
    RISCV_ISA_Zksh      : boolean;
    RISCV_ISA_Zmmul     : boolean;
    RISCV_ISA_Zxcfu     : boolean;
    RISCV_ISA_Sdext     : boolean;
    RISCV_ISA_Sdtrig    : boolean;
    RISCV_ISA_Smpmp     : boolean;
    -- Tuning Options --
    CPU_CLOCK_GATING_EN : boolean;
    CPU_FAST_MUL_EN     : boolean;
    CPU_FAST_SHIFT_EN   : boolean;
    CPU_RF_HW_RST_EN    : boolean;
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS     : natural range 0 to 16;
    PMP_MIN_GRANULARITY : natural;
    PMP_TOR_MODE_EN     : boolean;
    PMP_NAP_MODE_EN     : boolean;
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS        : natural range 0 to 13;
    HPM_CNT_WIDTH       : natural range 0 to 64;
    -- Instruction Cache (iCACHE) --
    ICACHE_EN           : boolean;
    ICACHE_NUM_BLOCKS   : natural range 1 to 256;
    ICACHE_BLOCK_SIZE   : natural range 4 to 2**16;
    ICACHE_UC_BEGIN     : std_ulogic_vector(31 downto 0);
    -- Data Cache (dCACHE) --
    DCACHE_EN           : boolean;
    DCACHE_NUM_BLOCKS   : natural range 1 to 256;
    DCACHE_BLOCK_SIZE   : natural range 4 to 2**16;
    DCACHE_UC_BEGIN     : std_ulogic_vector(31 downto 0)
  );
  port (
    -- global control --
    clk_i     : in  std_ulogic;
    rstn_i    : in  std_ulogic;
    -- interrupts --
    msi_i     : in  std_ulogic;
    mei_i     : in  std_ulogic;
    mti_i     : in  std_ulogic;
    firq_i    : in  std_ulogic_vector(15 downto 0);
    dbi_i     : in  std_ulogic;
    -- inter-core communication links --
    icc_tx_o  : out icc_t; -- TX links
    icc_rx_i  : in  icc_t; -- RX links
    -- system bus interface --
    bus_req_o : out bus_req_t;
    bus_rsp_i : in  bus_rsp_t
  );
end neorv32_core_complex;

architecture neorv32_core_complex_rtl of neorv32_core_complex is

  -- bus system --
  signal cpu_i_req, cpu_d_req, icache_req, dcache_req : bus_req_t;
  signal cpu_i_rsp, cpu_d_rsp, icache_rsp, dcache_rsp : bus_rsp_t;

begin

  -- CPU Core -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_inst: entity neorv32.neorv32_cpu
  generic map (
    -- General --
    HART_ID             => HART_ID,
    NUM_HARTS           => NUM_HARTS,
    VENDOR_ID           => VENDOR_ID,
    BOOT_ADDR           => BOOT_ADDR,
    DEBUG_PARK_ADDR     => DEBUG_PARK_ADDR,
    DEBUG_EXC_ADDR      => DEBUG_EXC_ADDR,
    -- RISC-V ISA Extensions --
    RISCV_ISA_C         => RISCV_ISA_C,
    RISCV_ISA_E         => RISCV_ISA_E,
    RISCV_ISA_M         => RISCV_ISA_M,
    RISCV_ISA_U         => RISCV_ISA_U,
    RISCV_ISA_Zaamo     => RISCV_ISA_Zaamo,
    RISCV_ISA_Zba       => RISCV_ISA_Zba,
    RISCV_ISA_Zbb       => RISCV_ISA_Zbb,
    RISCV_ISA_Zbkb      => RISCV_ISA_Zbkb,
    RISCV_ISA_Zbkc      => RISCV_ISA_Zbkc,
    RISCV_ISA_Zbkx      => RISCV_ISA_Zbkx,
    RISCV_ISA_Zbs       => RISCV_ISA_Zbs,
    RISCV_ISA_Zfinx     => RISCV_ISA_Zfinx,
    RISCV_ISA_Zicntr    => RISCV_ISA_Zicntr,
    RISCV_ISA_Zicond    => RISCV_ISA_Zicond,
    RISCV_ISA_Zihpm     => RISCV_ISA_Zihpm,
    RISCV_ISA_Zknd      => RISCV_ISA_Zknd,
    RISCV_ISA_Zkne      => RISCV_ISA_Zkne,
    RISCV_ISA_Zknh      => RISCV_ISA_Zknh,
    RISCV_ISA_Zksed     => RISCV_ISA_Zksed,
    RISCV_ISA_Zksh      => RISCV_ISA_Zksh,
    RISCV_ISA_Zmmul     => RISCV_ISA_Zmmul,
    RISCV_ISA_Zxcfu     => RISCV_ISA_Zxcfu,
    RISCV_ISA_Sdext     => RISCV_ISA_Sdext,
    RISCV_ISA_Sdtrig    => RISCV_ISA_Sdtrig,
    RISCV_ISA_Smpmp     => RISCV_ISA_Smpmp,
    -- Tuning Options --
    CPU_CLOCK_GATING_EN => CPU_CLOCK_GATING_EN,
    CPU_FAST_MUL_EN     => CPU_FAST_MUL_EN,
    CPU_FAST_SHIFT_EN   => CPU_FAST_SHIFT_EN,
    CPU_RF_HW_RST_EN    => CPU_RF_HW_RST_EN,
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS     => PMP_NUM_REGIONS,
    PMP_MIN_GRANULARITY => PMP_MIN_GRANULARITY,
    PMP_TOR_MODE_EN     => PMP_TOR_MODE_EN,
    PMP_NAP_MODE_EN     => PMP_NAP_MODE_EN,
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS        => HPM_NUM_CNTS,
    HPM_CNT_WIDTH       => HPM_CNT_WIDTH
  )
  port map (
    -- global control --
    clk_i      => clk_i,
    rstn_i     => rstn_i,
    -- interrupts --
    msi_i      => msi_i,
    mei_i      => mei_i,
    mti_i      => mti_i,
    firq_i     => firq_i,
    dbi_i      => dbi_i,
    -- inter-core communication links --
    icc_tx_o   => icc_tx_o,
    icc_rx_i   => icc_rx_i,
    -- instruction bus interface --
    ibus_req_o => cpu_i_req,
    ibus_rsp_i => cpu_i_rsp,
    -- data bus interface --
    dbus_req_o => cpu_d_req,
    dbus_rsp_i => cpu_d_rsp
  );


  -- CPU L1 Instruction Cache (I-Cache) -----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_icache_enabled:
  if ICACHE_EN generate
    neorv32_icache_inst: entity neorv32.neorv32_cache
    generic map (
      NUM_BLOCKS => ICACHE_NUM_BLOCKS,
      BLOCK_SIZE => ICACHE_BLOCK_SIZE,
      UC_BEGIN   => ICACHE_UC_BEGIN(31 downto 28),
      UC_ENABLE  => true,
      READ_ONLY  => true
    )
    port map (
      clk_i      => clk_i,
      rstn_i     => rstn_i,
      host_req_i => cpu_i_req,
      host_rsp_o => cpu_i_rsp,
      bus_req_o  => icache_req,
      bus_rsp_i  => icache_rsp
    );
  end generate;

  neorv32_icache_disabled:
  if not ICACHE_EN generate
    icache_req <= cpu_i_req;
    cpu_i_rsp  <= icache_rsp;
  end generate;


  -- CPU L1 Data Cache (D-Cache) ------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_dcache_enabled:
  if DCACHE_EN generate
    neorv32_dcache_inst: entity neorv32.neorv32_cache
    generic map (
      NUM_BLOCKS => DCACHE_NUM_BLOCKS,
      BLOCK_SIZE => DCACHE_BLOCK_SIZE,
      UC_BEGIN   => DCACHE_UC_BEGIN(31 downto 28),
      UC_ENABLE  => true,
      READ_ONLY  => false
    )
    port map (
      clk_i      => clk_i,
      rstn_i     => rstn_i,
      host_req_i => cpu_d_req,
      host_rsp_o => cpu_d_rsp,
      bus_req_o  => dcache_req,
      bus_rsp_i  => dcache_rsp
    );
  end generate;

  neorv32_dcache_disabled:
  if not DCACHE_EN generate
    dcache_req <= cpu_d_req;
    cpu_d_rsp  <= dcache_rsp;
  end generate;


  -- Core Instruction/Data Bus Switch -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_core_bus_switch_inst: entity neorv32.neorv32_bus_switch
  generic map (
    ROUND_ROBIN_EN   => false, -- use prioritizing arbitration
    PORT_A_READ_ONLY => false,
    PORT_B_READ_ONLY => true   -- instruction fetch is read-only
  )
  port map (
    clk_i    => clk_i,
    rstn_i   => rstn_i,
    a_lock_i => '0',        -- no exclusive accesses
    a_req_i  => dcache_req, -- data accesses are prioritized
    a_rsp_o  => dcache_rsp,
    b_req_i  => icache_req,
    b_rsp_o  => icache_rsp,
    x_req_o  => bus_req_o,
    x_rsp_i  => bus_rsp_i
  );


end neorv32_core_complex_rtl;
