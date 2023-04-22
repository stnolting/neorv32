-- #################################################################################################
-- # << NEORV32 - CPU Top Entity >>                                                                #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
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
-- # The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32       (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cpu is
  generic (
    -- General --
    HART_ID                      : std_ulogic_vector(31 downto 0); -- hardware thread ID
    VENDOR_ID                    : std_ulogic_vector(31 downto 0); -- vendor's JEDEC ID
    CPU_BOOT_ADDR                : std_ulogic_vector(31 downto 0); -- cpu boot address
    CPU_DEBUG_PARK_ADDR          : std_ulogic_vector(31 downto 0); -- cpu debug mode parking loop entry address
    CPU_DEBUG_EXC_ADDR           : std_ulogic_vector(31 downto 0); -- cpu debug mode exception entry address
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_B        : boolean; -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_C        : boolean; -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        : boolean; -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        : boolean; -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U        : boolean; -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    : boolean; -- implement 32-bit floating-point extension (using INT reg!)
    CPU_EXTENSION_RISCV_Zicntr   : boolean; -- implement base counters?
    CPU_EXTENSION_RISCV_Zicond   : boolean; -- implement conditional operations extension?
    CPU_EXTENSION_RISCV_Zihpm    : boolean; -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zifencei : boolean; -- implement instruction stream sync.?
    CPU_EXTENSION_RISCV_Zmmul    : boolean; -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zxcfu    : boolean; -- implement custom (instr.) functions unit?
    CPU_EXTENSION_RISCV_Sdext    : boolean; -- implement external debug mode extension?
    CPU_EXTENSION_RISCV_Sdtrig   : boolean; -- implement trigger module extension?
    -- Extension Options --
    FAST_MUL_EN                  : boolean; -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                : boolean; -- use barrel shifter for shift operations
    CPU_IPB_ENTRIES              : natural; -- entries in instruction prefetch buffer, has to be a power of 2, min 1
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS              : natural; -- number of regions (0..16)
    PMP_MIN_GRANULARITY          : natural; -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 : natural; -- number of implemented HPM counters (0..29)
    HPM_CNT_WIDTH                : natural  -- total size of HPM counters (0..64)
  );
  port (
    -- global control --
    clk_i         : in  std_ulogic; -- global clock, rising edge
    rstn_i        : in  std_ulogic; -- global reset, low-active, async
    sleep_o       : out std_ulogic; -- cpu is in sleep mode when set
    debug_o       : out std_ulogic; -- cpu is in debug mode when set
    -- instruction bus interface --
    i_bus_addr_o  : out std_ulogic_vector(31 downto 0); -- bus access address
    i_bus_rdata_i : in  std_ulogic_vector(31 downto 0); -- bus read data
    i_bus_re_o    : out std_ulogic; -- read request
    i_bus_ack_i   : in  std_ulogic; -- bus transfer acknowledge
    i_bus_err_i   : in  std_ulogic; -- bus transfer error
    i_bus_fence_o : out std_ulogic; -- executed FENCEI operation
    i_bus_priv_o  : out std_ulogic; -- current effective privilege level
    -- data bus interface --
    d_bus_addr_o  : out std_ulogic_vector(31 downto 0); -- bus access address
    d_bus_rdata_i : in  std_ulogic_vector(31 downto 0); -- bus read data
    d_bus_wdata_o : out std_ulogic_vector(31 downto 0); -- bus write data
    d_bus_ben_o   : out std_ulogic_vector(3 downto 0); -- byte enable
    d_bus_we_o    : out std_ulogic; -- write request
    d_bus_re_o    : out std_ulogic; -- read request
    d_bus_ack_i   : in  std_ulogic; -- bus transfer acknowledge
    d_bus_err_i   : in  std_ulogic; -- bus transfer error
    d_bus_fence_o : out std_ulogic; -- executed FENCE operation
    d_bus_priv_o  : out std_ulogic; -- current effective privilege level
    -- interrupts --
    msw_irq_i     : in  std_ulogic; -- risc-v: machine software interrupt
    mext_irq_i    : in  std_ulogic; -- risc-v: machine external interrupt
    mtime_irq_i   : in  std_ulogic; -- risc-v: machine timer interrupt
    firq_i        : in  std_ulogic_vector(15 downto 0); -- custom: fast interrupts
    db_halt_req_i : in  std_ulogic  -- risc-v: halt request (debug mode)
  );
end neorv32_cpu;

architecture neorv32_cpu_rtl of neorv32_cpu is

  -- local constants: additional register file read ports --
  constant regfile_rs3_en_c : boolean := CPU_EXTENSION_RISCV_Zxcfu or CPU_EXTENSION_RISCV_Zfinx; -- 3rd register file read port (rs3)
  constant regfile_rs4_en_c : boolean := CPU_EXTENSION_RISCV_Zxcfu; -- 4th register file read port (rs4)

  -- local constant: instruction prefetch buffer depth --
  constant ipb_override_c : boolean := (CPU_EXTENSION_RISCV_C = true) and (CPU_IPB_ENTRIES < 2); -- override IPB size: set to 2?
  constant ipb_depth_c    : natural := cond_sel_natural_f(ipb_override_c, 2, CPU_IPB_ENTRIES);

  -- local signals --
  signal ctrl        : ctrl_bus_t; -- main control bus
  signal imm         : std_ulogic_vector(XLEN-1 downto 0); -- immediate
  signal rs1         : std_ulogic_vector(XLEN-1 downto 0); -- source register 1
  signal rs2         : std_ulogic_vector(XLEN-1 downto 0); -- source register 2
  signal rs3         : std_ulogic_vector(XLEN-1 downto 0); -- source register 3
  signal rs4         : std_ulogic_vector(XLEN-1 downto 0); -- source register 4
  signal alu_res     : std_ulogic_vector(XLEN-1 downto 0); -- alu result
  signal alu_add     : std_ulogic_vector(XLEN-1 downto 0); -- alu address result
  signal alu_cmp     : std_ulogic_vector(1 downto 0); -- comparator result
  signal mem_rdata   : std_ulogic_vector(XLEN-1 downto 0); -- memory read data
  signal cp_done     : std_ulogic; -- ALU co-processor operation done
  signal alu_exc     : std_ulogic; -- ALU exception
  signal bus_d_wait  : std_ulogic; -- wait for current bus data access
  signal csr_rdata   : std_ulogic_vector(XLEN-1 downto 0); -- csr read data
  signal mar         : std_ulogic_vector(XLEN-1 downto 0); -- memory address register
  signal ma_load     : std_ulogic; -- misaligned load data address
  signal ma_store    : std_ulogic; -- misaligned store data address
  signal be_load     : std_ulogic; -- bus error on load data access
  signal be_store    : std_ulogic; -- bus error on store data access
  signal fetch_pc    : std_ulogic_vector(XLEN-1 downto 0); -- pc for instruction fetch
  signal curr_pc     : std_ulogic_vector(XLEN-1 downto 0); -- current pc (for currently executed instruction)
  signal next_pc     : std_ulogic_vector(XLEN-1 downto 0); -- next pc (for next executed instruction)
  signal fpu_flags   : std_ulogic_vector(4 downto 0); -- FPU exception flags
  signal i_pmp_fault : std_ulogic; -- instruction fetch PMP fault

  -- pmp interface --
  signal pmp_addr : pmp_addr_if_t;
  signal pmp_ctrl : pmp_ctrl_if_t;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- say hello --
  assert false report
    "The NEORV32 RISC-V Processor (Version 0x" & to_hstring32_f(hw_version_c) & ") - github.com/stnolting/neorv32" severity note;

  -- CPU ISA configuration --
  assert false report
    "NEORV32 CPU CONFIG NOTE: Core ISA ('MARCH') = RV32" &
    cond_sel_string_f(CPU_EXTENSION_RISCV_E,        "E", "I") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_M,        "M", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_C,        "C", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_B,        "B", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_U,        "U", "") &
    cond_sel_string_f(true,                         "_Zicsr", "") & -- always enabled
    cond_sel_string_f(CPU_EXTENSION_RISCV_Zicntr,   "_Zicntr", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Zicond,   "_Zicond", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Zifencei, "_Zifencei", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Zfinx,    "_Zfinx", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Zihpm,    "_Zihpm", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Zmmul,    "_Zmmul", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Zxcfu,    "_Zxcfu", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Sdext,    "_Sdext", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Sdtrig,   "_Sdtrig", "") &
    ""
    severity note;

  -- simulation notifier --
  assert not (is_simulation_c = true)  report
    "NEORV32 CPU WARNING! Assuming this is a simulation." severity warning;
  assert not (is_simulation_c = false) report
    "NEORV32 CPU NOTE: Assuming this is real hardware." severity note;

  -- CPU boot address --
  assert not (CPU_BOOT_ADDR(1 downto 0) /= "00") report
    "NEORV32 CPU CONFIG ERROR! <CPU_BOOT_ADDR> has to be 32-bit aligned." severity error;
  assert false report
    "NEORV32 CPU CONFIG NOTE: Boot from address 0x" & to_hstring32_f(CPU_BOOT_ADDR) & "." severity note;

  -- Instruction prefetch buffer --
  assert not (is_power_of_two_f(CPU_IPB_ENTRIES) = false) report
    "NEORV32 CPU CONFIG ERROR! Number of entries in instruction prefetch buffer <CPU_IPB_ENTRIES> has to be a power of two." severity error;
  assert not (ipb_override_c = true) report
    "NEORV32 CPU CONFIG WARNING! Overriding <CPU_IPB_ENTRIES> configuration (setting =2) because C ISA extension is enabled." severity warning;

  -- PMP --
  assert not (PMP_NUM_REGIONS > 16) report
    "NEORV32 CPU CONFIG ERROR! Number of PMP regions <PMP_NUM_REGIONS> out of valid range (0..16)." severity error;
  assert not ((is_power_of_two_f(PMP_MIN_GRANULARITY) = false) and (PMP_NUM_REGIONS > 0)) report
    "NEORV32 CPU CONFIG ERROR! <PMP_MIN_GRANULARITY> has to be a power of two." severity error;
  assert not ((PMP_MIN_GRANULARITY < 4) and (PMP_NUM_REGIONS > 0)) report
    "NEORV32 CPU CONFIG ERROR! <PMP_MIN_GRANULARITY> has to be >= 4 bytes." severity error;

  -- HPM counters --
  assert not ((CPU_EXTENSION_RISCV_Zihpm = true) and (HPM_NUM_CNTS > 29)) report
    "NEORV32 CPU CONFIG ERROR! Number of HPM counters <HPM_NUM_CNTS> out of valid range (0..29)." severity error;
  assert not ((CPU_EXTENSION_RISCV_Zihpm = true) and ((HPM_CNT_WIDTH < 0) or (HPM_CNT_WIDTH > 64))) report
    "NEORV32 CPU CONFIG ERROR! HPM counter width <HPM_CNT_WIDTH> has to be 0..64 bit." severity error;

  -- Mul extension(s) --
  assert not ((CPU_EXTENSION_RISCV_Zmmul = true) and (CPU_EXTENSION_RISCV_M = true)) report
    "NEORV32 CPU CONFIG ERROR! <M> and <Zmmul> extensions cannot co-exist!" severity error;

  -- Debug mode --
  assert not ((CPU_EXTENSION_RISCV_Sdext = true) and (CPU_EXTENSION_RISCV_Zifencei = false)) report
    "NEORV32 CPU CONFIG ERROR! Debug mode requires <CPU_EXTENSION_RISCV_Zifencei> extension to be enabled." severity error;


  -- Control Unit ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_control_inst: neorv32_cpu_control
  generic map (
    -- General --
    HART_ID                      => HART_ID,                      -- hardware thread ID
    VENDOR_ID                    => VENDOR_ID,                    -- vendor's JEDEC ID
    CPU_BOOT_ADDR                => CPU_BOOT_ADDR,                -- cpu boot address
    CPU_DEBUG_PARK_ADDR          => CPU_DEBUG_PARK_ADDR,          -- cpu debug mode parking loop entry address
    CPU_DEBUG_EXC_ADDR           => CPU_DEBUG_EXC_ADDR,           -- cpu debug mode exception entry address
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_B        => CPU_EXTENSION_RISCV_B,        -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_C        => CPU_EXTENSION_RISCV_C,        -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        => CPU_EXTENSION_RISCV_E,        -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        => CPU_EXTENSION_RISCV_M,        -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U        => CPU_EXTENSION_RISCV_U,        -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    => CPU_EXTENSION_RISCV_Zfinx,    -- implement 32-bit floating-point extension (using INT reg!)
    CPU_EXTENSION_RISCV_Zicntr   => CPU_EXTENSION_RISCV_Zicntr,   -- implement base counters?
    CPU_EXTENSION_RISCV_Zicond   => CPU_EXTENSION_RISCV_Zicond,   -- implement conditional operations extension?
    CPU_EXTENSION_RISCV_Zihpm    => CPU_EXTENSION_RISCV_Zihpm,    -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zifencei => CPU_EXTENSION_RISCV_Zifencei, -- implement instruction stream sync.?
    CPU_EXTENSION_RISCV_Zmmul    => CPU_EXTENSION_RISCV_Zmmul,    -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zxcfu    => CPU_EXTENSION_RISCV_Zxcfu,    -- implement custom (instr.) functions unit?
    CPU_EXTENSION_RISCV_Sdext    => CPU_EXTENSION_RISCV_Sdext,    -- implement external debug mode extension?
    CPU_EXTENSION_RISCV_Sdtrig   => CPU_EXTENSION_RISCV_Sdtrig,   -- implement trigger module extension?
    -- Tuning Options --
    FAST_MUL_EN                  => FAST_MUL_EN,                  -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                => FAST_SHIFT_EN,                -- use barrel shifter for shift operations
    CPU_IPB_ENTRIES              => ipb_depth_c,                  -- entries is instruction prefetch buffer, has to be a power of 2, min 1
    -- Physical memory protection (PMP) --
    PMP_NUM_REGIONS              => PMP_NUM_REGIONS,              -- number of regions (0..16)
    PMP_MIN_GRANULARITY          => PMP_MIN_GRANULARITY,          -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 => HPM_NUM_CNTS,                 -- number of implemented HPM counters (0..29)
    HPM_CNT_WIDTH                => HPM_CNT_WIDTH                 -- total size of HPM counters
  )
  port map (
    -- global control --
    clk_i         => clk_i,         -- global clock, rising edge
    rstn_i        => rstn_i,        -- global reset, low-active, async
    ctrl_o        => ctrl,          -- main control bus
    -- instruction fetch interface --
    i_bus_addr_o  => fetch_pc,      -- bus access address
    i_bus_rdata_i => i_bus_rdata_i, -- bus read data
    i_bus_re_o    => i_bus_re_o,    -- read enable
    i_bus_ack_i   => i_bus_ack_i,   -- bus transfer acknowledge
    i_bus_err_i   => i_bus_err_i,   -- bus transfer error
    i_pmp_fault_i => i_pmp_fault,   -- instruction fetch pmp fault
    -- status input --
    alu_cp_done_i => cp_done,       -- ALU iterative operation done
    alu_exc_i     => alu_exc,       -- ALU exception
    bus_d_wait_i  => bus_d_wait,    -- wait for bus
    -- data input --
    cmp_i         => alu_cmp,       -- comparator status
    alu_add_i     => alu_add,       -- ALU address result
    rs1_i         => rs1,           -- rf source 1
    -- data output --
    imm_o         => imm,           -- immediate
    curr_pc_o     => curr_pc,       -- current PC (corresponding to current instruction)
    next_pc_o     => next_pc,       -- next PC (corresponding to next instruction)
    csr_rdata_o   => csr_rdata,     -- CSR read data
    -- FPU interface --
    fpu_flags_i   => fpu_flags,     -- exception flags
    -- debug mode (halt) request --
    db_halt_req_i => db_halt_req_i,
    -- interrupts (risc-v compliant) --
    msw_irq_i     => msw_irq_i,     -- machine software interrupt
    mext_irq_i    => mext_irq_i,    -- machine external interrupt
    mtime_irq_i   => mtime_irq_i,   -- machine timer interrupt
    -- fast interrupts (custom) --
    firq_i        => firq_i,        -- fast interrupt trigger
    -- physical memory protection --
    pmp_addr_o    => pmp_addr,      -- addresses
    pmp_ctrl_o    => pmp_ctrl,      -- configs
    -- bus access exceptions --
    mar_i         => mar,           -- memory address register
    ma_load_i     => ma_load,       -- misaligned load data address
    ma_store_i    => ma_store,      -- misaligned store data address
    be_load_i     => be_load,       -- bus error on load data access
    be_store_i    => be_store       -- bus error on store data access
  );

  -- CPU state --
  sleep_o <= ctrl.cpu_sleep; -- set when CPU is sleeping (after WFI)
  debug_o <= ctrl.cpu_debug; -- set when CPU is in debug mode

  -- instruction fetch interface --
  i_bus_addr_o  <= fetch_pc;
  i_bus_fence_o <= ctrl.bus_fencei;
  i_bus_priv_o  <= ctrl.cpu_priv;


  -- Register File --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_regfile_inst: neorv32_cpu_regfile
  generic map (
    RVE    => CPU_EXTENSION_RISCV_E, -- implement embedded RF extension?
    RS3_EN => regfile_rs3_en_c,      -- enable 3rd read port
    RS4_EN => regfile_rs4_en_c       -- enable 4th read port
  )
  port map (
    -- global control --
    clk_i  => clk_i,     -- global clock, rising edge
    ctrl_i => ctrl,      -- main control bus
    -- data input --
    alu_i  => alu_res,   -- ALU result
    mem_i  => mem_rdata, -- memory read data
    csr_i  => csr_rdata, -- CSR read data
    pc2_i  => next_pc,   -- next PC
    -- data output --
    rs1_o  => rs1,       -- operand 1
    rs2_o  => rs2,       -- operand 2
    rs3_o  => rs3,       -- operand 3
    rs4_o  => rs4        -- operand 4
  );


  -- ALU ------------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_alu_inst: neorv32_cpu_alu
  generic map (
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_B      => CPU_EXTENSION_RISCV_B,      -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_M      => CPU_EXTENSION_RISCV_M,      -- implement mul/div extension?
    CPU_EXTENSION_RISCV_Zmmul  => CPU_EXTENSION_RISCV_Zmmul,  -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zfinx  => CPU_EXTENSION_RISCV_Zfinx,  -- implement 32-bit floating-point extension (using INT reg!)
    CPU_EXTENSION_RISCV_Zxcfu  => CPU_EXTENSION_RISCV_Zxcfu,  -- implement custom (instr.) functions unit?
    CPU_EXTENSION_RISCV_Zicond => CPU_EXTENSION_RISCV_Zicond, -- implement conditional operations extension?
    -- Extension Options --
    FAST_MUL_EN                => FAST_MUL_EN,                -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN              => FAST_SHIFT_EN               -- use barrel shifter for shift operations
  )
  port map (
    -- global control --
    clk_i       => clk_i,     -- global clock, rising edge
    rstn_i      => rstn_i,    -- global reset, low-active, async
    ctrl_i      => ctrl,      -- main control bus
    -- data input --
    rs1_i       => rs1,       -- rf source 1
    rs2_i       => rs2,       -- rf source 2
    rs3_i       => rs3,       -- rf source 3
    rs4_i       => rs4,       -- rf source 4
    pc_i        => curr_pc,   -- current PC
    imm_i       => imm,       -- immediate
    -- data output --
    cmp_o       => alu_cmp,   -- comparator status
    res_o       => alu_res,   -- ALU result
    add_o       => alu_add,   -- address computation result
    fpu_flags_o => fpu_flags, -- FPU exception flags
    -- status --
    exc_o       => alu_exc,   -- ALU exception
    cp_done_o   => cp_done    -- iterative processing units done?
  );


  -- Bus Interface (Load/Store Unit) --------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_bus_inst: neorv32_cpu_bus
  generic map (
    PMP_NUM_REGIONS     => PMP_NUM_REGIONS,    -- number of regions (0..16)
    PMP_MIN_GRANULARITY => PMP_MIN_GRANULARITY -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
  )
  port map (
    -- global control --
    clk_i         => clk_i,         -- global clock, rising edge
    rstn_i        => rstn_i,        -- global reset, low-active, async
    ctrl_i        => ctrl,          -- main control bus
    -- cpu instruction fetch interface --
    fetch_pc_i    => fetch_pc,      -- PC for instruction fetch
    i_pmp_fault_o => i_pmp_fault,   -- instruction fetch pmp fault
    -- cpu data access interface --
    addr_i        => alu_add,       -- ALU.add result -> access address
    wdata_i       => rs2,           -- write data
    rdata_o       => mem_rdata,     -- read data
    mar_o         => mar,           -- current memory address register
    d_wait_o      => bus_d_wait,    -- wait for access to complete
    ma_load_o     => ma_load,       -- misaligned load data address
    ma_store_o    => ma_store,      -- misaligned store data address
    be_load_o     => be_load,       -- bus error on load data access
    be_store_o    => be_store,      -- bus error on store data access
    -- physical memory protection --
    pmp_addr_i    => pmp_addr,      -- addresses
    pmp_ctrl_i    => pmp_ctrl,      -- configurations
    -- data bus --
    d_bus_addr_o  => d_bus_addr_o,  -- bus access address
    d_bus_rdata_i => d_bus_rdata_i, -- bus read data
    d_bus_wdata_o => d_bus_wdata_o, -- bus write data
    d_bus_ben_o   => d_bus_ben_o,   -- byte enable
    d_bus_we_o    => d_bus_we_o,    -- write enable
    d_bus_re_o    => d_bus_re_o,    -- read enable
    d_bus_ack_i   => d_bus_ack_i,   -- bus transfer acknowledge
    d_bus_err_i   => d_bus_err_i,   -- bus transfer error
    d_bus_fence_o => d_bus_fence_o, -- fence operation
    d_bus_priv_o  => d_bus_priv_o   -- current effective privilege level
  );


end neorv32_cpu_rtl;
