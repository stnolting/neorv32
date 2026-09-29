-- ================================================================================ --
-- NEORV32 SoC - CPU Core Complex (CPU + Caches + Bus Mux)                          --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  --
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
    HART_ID             : natural range 0 to 1           := 0;           -- hardware thread ID
    VENDOR_ID           : std_ulogic_vector(31 downto 0) := x"00000000"; -- vendor ID
    BOOT_ADDR           : std_ulogic_vector(31 downto 0) := x"00000000"; -- CPU boot address
    DEBUG_PARK_ADDR     : std_ulogic_vector(31 downto 0) := x"00000000"; -- CPU debug mode parking loop entry address
    DEBUG_EXC_ADDR      : std_ulogic_vector(31 downto 0) := x"00000000"; -- CPU debug mode exception entry address
    -- RISC-V ISA Extensions --
    RISCV_ISA_C         : boolean                        := false;       -- compressed extension
    RISCV_ISA_E         : boolean                        := false;       -- embedded RF extension
    RISCV_ISA_M         : boolean                        := false;       -- mul/div extension
    RISCV_ISA_U         : boolean                        := false;       -- user mode extension
    RISCV_ISA_Zaamo     : boolean                        := false;       -- atomic read-modify-write operations extension
    RISCV_ISA_Zalrsc    : boolean                        := false;       -- atomic reservation-set operations extension
    RISCV_ISA_Zba       : boolean                        := false;       -- shifted-add bit-manipulation extension
    RISCV_ISA_Zbb       : boolean                        := false;       -- basic bit-manipulation extension
    RISCV_ISA_Zbc       : boolean                        := false;       -- carry-less multiplication instructions
    RISCV_ISA_Zbkb      : boolean                        := false;       -- bit-manipulation instructions for cryptography
    RISCV_ISA_Zbkc      : boolean                        := false;       -- carry-less multiplication instructions
    RISCV_ISA_Zbkx      : boolean                        := false;       -- cryptography crossbar permutation extension
    RISCV_ISA_Zbs       : boolean                        := false;       -- single-bit bit-manipulation extension
    RISCV_ISA_Zcb       : boolean                        := false;       -- additional code size reduction instructions
    RISCV_ISA_Zcmop     : boolean                        := false;       -- compressed may-be-operations
    RISCV_ISA_Zfinx     : boolean                        := false;       -- 32-bit floating-point extension
    RISCV_ISA_Zibi      : boolean                        := false;       -- branch with immediate
    RISCV_ISA_Zicntr    : boolean                        := false;       -- base counters
    RISCV_ISA_Zicond    : boolean                        := false;       -- integer conditional operations
    RISCV_ISA_Zihpm     : boolean                        := false;       -- hardware performance monitors
    RISCV_ISA_Zimop     : boolean                        := false;       -- may-be-operations
    RISCV_ISA_Zknd      : boolean                        := false;       -- cryptography NIST AES decryption extension
    RISCV_ISA_Zkne      : boolean                        := false;       -- cryptography NIST AES encryption extension
    RISCV_ISA_Zknh      : boolean                        := false;       -- cryptography NIST hash extension
    RISCV_ISA_Zksed     : boolean                        := false;       -- ShangMi hash extension
    RISCV_ISA_Zksh      : boolean                        := false;       -- ShangMi block cipher extension
    RISCV_ISA_Zmmul     : boolean                        := false;       -- multiply-only M sub-extension
    RISCV_ISA_Sdext     : boolean                        := false;       -- external debug mode extension
    RISCV_ISA_Sdtrig    : boolean                        := false;       -- trigger module extension
    RISCV_ISA_Smcntrpmf : boolean                        := false;       -- counter privilege-mode filtering
    RISCV_ISA_Smpmp     : boolean                        := false;       -- physical memory protection
    RISCV_ISA_Xcfu      : boolean                        := false;       -- custom (instr.) functions unit
    -- Tuning Options --
    CPU_TRACE_EN        : boolean                        := false;       -- enable CPU execution trace generator
    CPU_CONSTT_BR_EN    : boolean                        := false;       -- constant-time branches
    CPU_FAST_MUL_EN     : boolean                        := false;       -- use DSPs for M extension's multiplier
    CPU_FAST_MUL_REGS   : natural range 1 to 3           := 1;           -- number of fast multiplier register stages (needs CPU_FAST_MUL_EN)
    CPU_FAST_SHIFT_EN   : boolean                        := false;       -- use barrel shifter for shift operations
    CPU_RF_ARCH_SEL     : natural range 0 to 3           := 0;           -- register file implementation style select
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS     : natural range 0 to 16          := 0;           -- number of regions
    PMP_MIN_GRANULARITY : natural                        := 4;           -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
    PMP_TOR_MODE_EN     : boolean                        := false;       -- enable TOR mode
    PMP_NAP_MODE_EN     : boolean                        := false;       -- enable NAPOT/NA4 modes
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS        : natural range 0 to 29          := 0;           -- number of implemented HPM counters
    HPM_CNT_WIDTH       : natural range 0 to 64          := 0;           -- total size of HPM counters
    -- Trigger Module (TM) --
    NUM_HW_TRIGGERS     : natural range 0 to 16          := 0;           -- number of hardware triggers
    -- CPU Caches --
    ICACHE_EN           : boolean                        := false;       -- implement instruction cache (i-cache)
    ICACHE_NUM_BLOCKS   : natural range 1 to 4096        := 4;           -- i-cache: number of blocks, has to be a power of 2
    DCACHE_EN           : boolean                        := false;       -- implement data cache (d-cache)
    DCACHE_NUM_BLOCKS   : natural range 1 to 4096        := 4;           -- d-cache: number of blocks, has to be a power of 2
    CACHE_BLOCK_SIZE    : natural range 4 to 1024        := 64;          -- i-cache/d-cache: block size in bytes, has to be a power of 2
    CACHE_BURSTS_EN     : boolean                        := true;        -- i-cache/d-cache: enable issuing of burst transfer for cache update
    CACHE_UC_BASE       : std_ulogic_vector(3 downto 0)  := x"F";        -- base address of uncached address space (256MB page)
    -- Misc --
    REGSTAGE_EN         : boolean                        := false        -- enable bus interface register stage
  );
  port (
    -- global control --
    clk_i     : in  std_ulogic;                     -- global clock, rising edge
    rstn_i    : in  std_ulogic;                     -- global reset, low-active, async
    -- status --
    mtime_i   : in  std_ulogic_vector(63 downto 0); -- system time input from CLINT/MTIME
    trace_o   : out trace_port_t;                   -- execution trace port (enabled when CPU_TRACE_EN = true)
    sleep_o   : out std_ulogic;                     -- CPU is in sleep mode
    -- interrupts --
    msi_i     : in  std_ulogic;                     -- RISC-V machine software interrupt
    mei_i     : in  std_ulogic;                     -- RISC-V machine external interrupt
    mti_i     : in  std_ulogic;                     -- RISC-V machine timer interrupt
    firq_i    : in  std_ulogic_vector(15 downto 0); -- custom fast interrupts
    dbi_i     : in  std_ulogic;                     -- RISC-V debug halt request interrupt
    -- bus interface --
    bus_req_o : out bus_req_t;                      -- request bus
    bus_rsp_i : in  bus_rsp_t                       -- response bus
  );
end entity;

architecture neorv32_core_complex_rtl of neorv32_core_complex is

  signal cpu_i_fence, cpu_d_fence, icache_sync, dcache_sync : std_ulogic;
  signal cpu_i_req, cpu_d_req, icache_req, dcache_req, bus_req : bus_req_t;
  signal cpu_i_rsp, cpu_d_rsp, icache_rsp, dcache_rsp, bus_rsp : bus_rsp_t;

begin

  -- CPU Core -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cpu_inst: entity neorv32.neorv32_cpu
  generic map (
    HART_ID             => HART_ID,
    VENDOR_ID           => VENDOR_ID,
    BOOT_ADDR           => BOOT_ADDR,
    DEBUG_PARK_ADDR     => DEBUG_PARK_ADDR,
    DEBUG_EXC_ADDR      => DEBUG_EXC_ADDR,
    RISCV_ISA_C         => RISCV_ISA_C,
    RISCV_ISA_E         => RISCV_ISA_E,
    RISCV_ISA_M         => RISCV_ISA_M,
    RISCV_ISA_U         => RISCV_ISA_U,
    RISCV_ISA_Zaamo     => RISCV_ISA_Zaamo,
    RISCV_ISA_Zalrsc    => RISCV_ISA_Zalrsc,
    RISCV_ISA_Zba       => RISCV_ISA_Zba,
    RISCV_ISA_Zbb       => RISCV_ISA_Zbb,
    RISCV_ISA_Zbc       => RISCV_ISA_Zbc,
    RISCV_ISA_Zbkb      => RISCV_ISA_Zbkb,
    RISCV_ISA_Zbkc      => RISCV_ISA_Zbkc,
    RISCV_ISA_Zbkx      => RISCV_ISA_Zbkx,
    RISCV_ISA_Zbs       => RISCV_ISA_Zbs,
    RISCV_ISA_Zcb       => RISCV_ISA_Zcb,
    RISCV_ISA_Zcmop     => RISCV_ISA_Zcmop,
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
    RISCV_ISA_Sdext     => RISCV_ISA_Sdext,
    RISCV_ISA_Sdtrig    => RISCV_ISA_Sdtrig,
    RISCV_ISA_Smcntrpmf => RISCV_ISA_Smcntrpmf,
    RISCV_ISA_Smpmp     => RISCV_ISA_Smpmp,
    RISCV_ISA_Xcfu      => RISCV_ISA_Xcfu,
    CPU_TRACE_EN        => CPU_TRACE_EN,
    CPU_CONSTT_BR_EN    => CPU_CONSTT_BR_EN,
    CPU_FAST_MUL_EN     => CPU_FAST_MUL_EN,
    CPU_FAST_MUL_REGS   => CPU_FAST_MUL_REGS,
    CPU_FAST_SHIFT_EN   => CPU_FAST_SHIFT_EN,
    CPU_RF_ARCH_SEL     => CPU_RF_ARCH_SEL,
    PMP_NUM_REGIONS     => PMP_NUM_REGIONS,
    PMP_MIN_GRANULARITY => PMP_MIN_GRANULARITY,
    PMP_TOR_MODE_EN     => PMP_TOR_MODE_EN,
    PMP_NAP_MODE_EN     => PMP_NAP_MODE_EN,
    HPM_NUM_CNTS        => HPM_NUM_CNTS,
    HPM_CNT_WIDTH       => HPM_CNT_WIDTH,
    NUM_HW_TRIGGERS     => NUM_HW_TRIGGERS
  )
  port map (
    clk_i      => clk_i,
    rstn_i     => rstn_i,
    mtime_i    => mtime_i,
    trace_o    => trace_o,
    sleep_o    => sleep_o,
    msi_i      => msi_i,
    mei_i      => mei_i,
    mti_i      => mti_i,
    firq_i     => firq_i,
    dbi_i      => dbi_i,
    ifence_o   => cpu_i_fence,
    ibus_req_o => cpu_i_req,
    ibus_rsp_i => cpu_i_rsp,
    dfence_o   => cpu_d_fence,
    dbus_req_o => cpu_d_req,
    dbus_rsp_i => cpu_d_rsp
  );

  -- Instruction Cache ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  icache_enabled:
  if ICACHE_EN generate
    icache_inst: entity neorv32.neorv32_cache
    generic map (
      NUM_BLOCKS => ICACHE_NUM_BLOCKS,
      BLOCK_SIZE => CACHE_BLOCK_SIZE,
      UC_BEGIN   => CACHE_UC_BASE,
      READ_ONLY  => true,
      BURSTS_EN  => CACHE_BURSTS_EN
    )
    port map (
      clk_i      => clk_i,
      rstn_i     => rstn_i,
      sync_i     => icache_sync,
      host_req_i => cpu_i_req,
      host_rsp_o => cpu_i_rsp,
      bus_req_o  => icache_req,
      bus_rsp_i  => icache_rsp
    );
    -- fence.i => clear I$
    icache_sync <= cpu_i_fence;
  end generate;

  icache_disabled:
  if not ICACHE_EN generate
    icache_sync <= '0';
    icache_req  <= cpu_i_req;
    cpu_i_rsp   <= icache_rsp;
  end generate;

  -- Data Cache -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  dcache_enabled:
  if DCACHE_EN generate
    dcache_inst: entity neorv32.neorv32_cache
    generic map (
      NUM_BLOCKS => DCACHE_NUM_BLOCKS,
      BLOCK_SIZE => CACHE_BLOCK_SIZE,
      UC_BEGIN   => CACHE_UC_BASE,
      READ_ONLY  => false,
      BURSTS_EN  => CACHE_BURSTS_EN
    )
    port map (
      clk_i      => clk_i,
      rstn_i     => rstn_i,
      sync_i     => dcache_sync,
      host_req_i => cpu_d_req,
      host_rsp_o => cpu_d_rsp,
      bus_req_o  => dcache_req,
      bus_rsp_i  => dcache_rsp
    );
    -- fence   => flush D$
    -- fence.i => clear I$ and flush D$ (so I$ gets updated data; #1540)
    dcache_sync <= cpu_d_fence or cpu_i_fence;
  end generate;

  dcache_disabled:
  if not DCACHE_EN generate
    dcache_sync <= '0';
    dcache_req  <= cpu_d_req;
    cpu_d_rsp   <= dcache_rsp;
  end generate;

  -- Instruction/Data Bus Switch ------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_switch_inst: entity neorv32.neorv32_bus_switch
  generic map (
    ROUND_ROBIN_EN => false, -- use prioritizing arbitration
    A_READ_ONLY    => false,
    B_READ_ONLY    => true -- instruction fetch is read-only
  )
  port map (
    clk_i   => clk_i,
    rstn_i  => rstn_i,
    a_req_i => dcache_req, -- data accesses are prioritized
    a_rsp_o => dcache_rsp,
    b_req_i => icache_req,
    b_rsp_o => icache_rsp,
    x_req_o => bus_req,
    x_rsp_i => bus_rsp
  );

  -- Optional Register Stage ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  reg_stage_enabled:
  if REGSTAGE_EN generate
    reg_stage_inst: entity neorv32.neorv32_bus_reg
    port map (
      clk_i        => clk_i,
      rstn_i       => rstn_i,
      host_req_i   => bus_req,
      host_rsp_o   => bus_rsp,
      device_req_o => bus_req_o,
      device_rsp_i => bus_rsp_i
    );
  end generate;

  -- pass-through --
  reg_stage_disabled:
  if not REGSTAGE_EN generate
    bus_req_o <= bus_req;
    bus_rsp   <= bus_rsp_i;
  end generate;

end architecture;
