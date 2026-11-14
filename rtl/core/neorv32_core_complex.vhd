-- ================================================================================ --
-- NEORV32 SoC - CPU Core Complex (CPU + Caches + Bus Mux)                          --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) Stephan Nolting. All rights reserved.                              --
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
    TRACE_PORT_EN       : boolean                        := false;       -- enable CPU trace port
    TRACE_SIMLOG_EN     : boolean                        := false;       -- enable simulation trace logging
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

  -- memory ordering arbiter --
  component neorv32_core_complex_ordering
  port (
    -- global control --
    rstn_i    : in  std_ulogic;
    clk_i     : in  std_ulogic;
    -- CPU interface --
    ifence_i  : in  std_ulogic;
    dfence_i  : in  std_ulogic;
    fenced_o  : out std_ulogic;
    -- Cache interface --
    ic_sync_o : out std_ulogic;
    ic_sync_i : in  std_ulogic;
    dc_sync_o : out std_ulogic;
    dc_sync_i : in  std_ulogic
  );
  end component;

  signal cpu_fence_r, cpu_fence_w, cpu_fence_x, cpu_ifence, cpu_dfence, cpu_fenced : std_ulogic;
  signal icache_sync, icache_syncd, dcache_sync, dcache_syncd : std_ulogic;
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
    TRACE_PORT_EN       => TRACE_PORT_EN,
    TRACE_SIMLOG_EN     => TRACE_SIMLOG_EN,
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
    fence_r_o  => cpu_fence_r,
    fence_w_o  => cpu_fence_w,
    fence_x_o  => cpu_fence_x,
    fenced_i   => cpu_fenced,
    ibus_req_o => cpu_i_req,
    ibus_rsp_i => cpu_i_rsp,
    dbus_req_o => cpu_d_req,
    dbus_rsp_i => cpu_d_rsp
  );

  -- Cache Synchronization Arbiter ----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ordering_arbiter:
  if ICACHE_EN or DCACHE_EN generate
    neorv32_core_complex_ordering_inst: neorv32_core_complex_ordering
    port map (
      rstn_i    => rstn_i,
      clk_i     => clk_i,
      ifence_i  => cpu_ifence,
      dfence_i  => cpu_dfence,
      fenced_o  => cpu_fenced,
      ic_sync_o => icache_sync,
      ic_sync_i => icache_syncd,
      dc_sync_o => dcache_sync,
      dc_sync_i => dcache_syncd
    );
  end generate;

  ordering_none:
  if not (ICACHE_EN or DCACHE_EN) generate
    icache_sync <= '0';
    dcache_sync <= '0';
    cpu_fenced  <= '1';
  end generate;

  -- TODO: implement fine-grained d-cache flush/invalidate --
  cpu_ifence <= cpu_fence_x;
  cpu_dfence <= cpu_fence_r or cpu_fence_w;

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
      sync_o     => icache_syncd,
      host_req_i => cpu_i_req,
      host_rsp_o => cpu_i_rsp,
      bus_req_o  => icache_req,
      bus_rsp_i  => icache_rsp
    );
  end generate;

  icache_disabled:
  if not ICACHE_EN generate
    icache_syncd <= '1';
    icache_req   <= cpu_i_req;
    cpu_i_rsp    <= icache_rsp;
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
      sync_o     => dcache_syncd,
      host_req_i => cpu_d_req,
      host_rsp_o => cpu_d_rsp,
      bus_req_o  => dcache_req,
      bus_rsp_i  => dcache_rsp
    );
  end generate;

  dcache_disabled:
  if not DCACHE_EN generate
    dcache_syncd <= '1';
    dcache_req   <= cpu_d_req;
    cpu_d_rsp    <= dcache_rsp;
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


-- ================================================================================ --
-- NEORV32 SoC - CPU Core Complex Memory Ordering Arbiter                           --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) Stephan Nolting. All rights reserved.                              --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity neorv32_core_complex_ordering is
  port (
    -- global control --
    rstn_i    : in  std_ulogic; -- reset, async, low-active
    clk_i     : in  std_ulogic; -- clock
    -- CPU interface --
    ifence_i  : in  std_ulogic; -- instruction fence, single-shot
    dfence_i  : in  std_ulogic; -- data fence, single-shot
    fenced_o  : out std_ulogic; -- operation(s) done, single-shot
    -- Cache interface --
    ic_sync_o : out std_ulogic; -- i-cache synchronization request, single-shot
    ic_sync_i : in  std_ulogic; -- i-cache synchronization completed
    dc_sync_o : out std_ulogic; -- d-cache synchronization request, single-shot
    dc_sync_i : in  std_ulogic  -- d-cache synchronization completed
  );
end entity;

architecture neorv32_core_complex_ordering_rtl of neorv32_core_complex_ordering is

  type state_t is (S_IDLE, S_DC_SYNC, S_DC_WAIT, S_IC_SYNC, S_IC_WAIT, S_DONE);
  signal state, state_nxt : state_t;
  signal ifence, ifence_nxt : std_ulogic;

begin

  -- arbiter sync --
  arbiter_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      state  <= S_IDLE;
      ifence <= '0';
    elsif rising_edge(clk_i) then
      state  <= state_nxt;
      ifence <= ifence_nxt;
    end if;
  end process;

  -- arbiter comb --
  arbiter_comb: process(state, ifence, ifence_i, dfence_i, ic_sync_i, dc_sync_i)
  begin
    -- defaults --
    state_nxt  <= state;
    ifence_nxt <= ifence;
    fenced_o   <= '0';
    ic_sync_o  <= '0';
    dc_sync_o  <= '0';
    -- fsm --
    case state is

      -- wait for requests ..
      when S_IDLE =>
        ifence_nxt <= ifence_i;
        if (ifence_i = '1') or (dfence_i = '1') then
          state_nxt <= S_DC_SYNC;
        end if;

      -- trigger d-cache flush --
      when S_DC_SYNC =>
        dc_sync_o <= '1';
        state_nxt <= S_DC_WAIT;

      -- wait for synchronization to complete --
      when S_DC_WAIT =>
        if (dc_sync_i = '1') then
          if (ifence = '1') then
            state_nxt <= S_IC_SYNC;
          else
            state_nxt <= S_DONE;
          end if;
        end if;

      -- trigger i-cache clear --
      when S_IC_SYNC =>
        ic_sync_o <= '1';
        state_nxt <= S_IC_WAIT;

      -- wait for synchronization to complete --
      when S_IC_WAIT =>
        if (ic_sync_i = '1') then
          state_nxt <= S_DONE;
        end if;

      -- signal "done" to CPU --
      when S_DONE =>
        fenced_o  <= '1';
        state_nxt <= S_IDLE;

      -- undefined --
      when others =>
        state_nxt <= S_IDLE;

    end case;
  end process;

end architecture;
