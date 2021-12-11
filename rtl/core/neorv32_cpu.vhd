-- #################################################################################################
-- # << NEORV32 - CPU Top Entity >>                                                                #
-- # ********************************************************************************************* #
-- # NEORV32 CPU:                                                                                  #
-- # * neorv32_cpu.vhd                   - CPU top entity                                          #
-- #   * neorv32_cpu_alu.vhd             - Arithmetic/logic unit                                   #
-- #     * neorv32_cpu_cp_bitmanip.vhd   - Bit-manipulation co-processor                           #
-- #     * neorv32_cpu_cp_fpu.vhd        - Single-precision FPU co-processor                       #
-- #     * neorv32_cpu_cp_muldiv.vhd     - Integer multiplier/divider co-processor                 #
-- #     * neorv32_cpu_cp_shifter.vhd    - Base ISA shifter unit                                   #
-- #   * neorv32_cpu_bus.vhd             - Instruction and data bus interface unit                 #
-- #   * neorv32_cpu_control.vhd         - CPU control and CSR system                              #
-- #     * neorv32_cpu_decompressor.vhd  - Compressed instructions decoder                         #
-- #   * neorv32_cpu_regfile.vhd         - Data register file                                      #
-- # * neorv32_package.vhd               - Main CPU & Processor package file                       #
-- #                                                                                               #
-- # Check out the CPU's online documentation for more information:                                #
-- #  HQ:         https://github.com/stnolting/neorv32                                             #
-- #  Data Sheet: https://stnolting.github.io/neorv32                                              #
-- #  User Guide: https://stnolting.github.io/neorv32/ug                                           #
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

entity neorv32_cpu is
  generic (
    -- General --
    HW_THREAD_ID                 : natural; -- hardware thread id (32-bit)
    CPU_BOOT_ADDR                : std_ulogic_vector(31 downto 0); -- cpu boot address
    CPU_DEBUG_ADDR               : std_ulogic_vector(31 downto 0); -- cpu debug mode start address
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        : boolean; -- implement atomic extension?
    CPU_EXTENSION_RISCV_B        : boolean; -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_C        : boolean; -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        : boolean; -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        : boolean; -- implement muld/div extension?
    CPU_EXTENSION_RISCV_U        : boolean; -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    : boolean; -- implement 32-bit floating-point extension (using INT reg!)
    CPU_EXTENSION_RISCV_Zicsr    : boolean; -- implement CSR system?
    CPU_EXTENSION_RISCV_Zicntr   : boolean; -- implement base counters?
    CPU_EXTENSION_RISCV_Zihpm    : boolean; -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zifencei : boolean; -- implement instruction stream sync.?
    CPU_EXTENSION_RISCV_Zmmul    : boolean; -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_DEBUG    : boolean; -- implement CPU debug mode?
    -- Extension Options --
    FAST_MUL_EN                  : boolean; -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                : boolean; -- use barrel shifter for shift operations
    CPU_CNT_WIDTH                : natural; -- total width of CPU cycle and instret counters (0..64)
    CPU_IPB_ENTRIES              : natural; -- entries is instruction prefetch buffer, has to be a power of 2
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS              : natural; -- number of regions (0..64)
    PMP_MIN_GRANULARITY          : natural; -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 : natural; -- number of implemented HPM counters (0..29)
    HPM_CNT_WIDTH                : natural  -- total size of HPM counters (0..64)
  );
  port (
    -- global control --
    clk_i          : in  std_ulogic; -- global clock, rising edge
    rstn_i         : in  std_ulogic; -- global reset, low-active, async
    sleep_o        : out std_ulogic; -- cpu is in sleep mode when set
    debug_o        : out std_ulogic; -- cpu is in debug mode when set
    -- instruction bus interface --
    i_bus_addr_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
    i_bus_rdata_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
    i_bus_wdata_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
    i_bus_ben_o    : out std_ulogic_vector(03 downto 0); -- byte enable
    i_bus_we_o     : out std_ulogic; -- write enable
    i_bus_re_o     : out std_ulogic; -- read enable
    i_bus_lock_o   : out std_ulogic; -- exclusive access request
    i_bus_ack_i    : in  std_ulogic; -- bus transfer acknowledge
    i_bus_err_i    : in  std_ulogic; -- bus transfer error
    i_bus_fence_o  : out std_ulogic; -- executed FENCEI operation
    i_bus_priv_o   : out std_ulogic_vector(1 downto 0); -- privilege level
    -- data bus interface --
    d_bus_addr_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
    d_bus_rdata_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
    d_bus_wdata_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
    d_bus_ben_o    : out std_ulogic_vector(03 downto 0); -- byte enable
    d_bus_we_o     : out std_ulogic; -- write enable
    d_bus_re_o     : out std_ulogic; -- read enable
    d_bus_lock_o   : out std_ulogic; -- exclusive access request
    d_bus_ack_i    : in  std_ulogic; -- bus transfer acknowledge
    d_bus_err_i    : in  std_ulogic; -- bus transfer error
    d_bus_fence_o  : out std_ulogic; -- executed FENCE operation
    d_bus_priv_o   : out std_ulogic_vector(1 downto 0); -- privilege level
    -- system time input from MTIME --
    time_i         : in  std_ulogic_vector(63 downto 0); -- current system time
    -- interrupts (risc-v compliant) --
    msw_irq_i      : in  std_ulogic;-- machine software interrupt
    mext_irq_i     : in  std_ulogic;-- machine external interrupt
    mtime_irq_i    : in  std_ulogic;-- machine timer interrupt
    -- fast interrupts (custom) --
    firq_i         : in  std_ulogic_vector(15 downto 0);
    -- debug mode (halt) request --
    db_halt_req_i  : in  std_ulogic
  );
end neorv32_cpu;

architecture neorv32_cpu_rtl of neorv32_cpu is

  -- local signals --
  signal ctrl       : std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
  signal comparator : std_ulogic_vector(1 downto 0); -- comparator result
  signal imm        : std_ulogic_vector(data_width_c-1 downto 0); -- immediate
  signal instr      : std_ulogic_vector(data_width_c-1 downto 0); -- new instruction
  signal rs1, rs2   : std_ulogic_vector(data_width_c-1 downto 0); -- source registers
  signal alu_res    : std_ulogic_vector(data_width_c-1 downto 0); -- alu result
  signal alu_add    : std_ulogic_vector(data_width_c-1 downto 0); -- alu address result
  signal mem_rdata  : std_ulogic_vector(data_width_c-1 downto 0); -- memory read data
  signal alu_idone  : std_ulogic; -- iterative alu operation done
  signal bus_i_wait : std_ulogic; -- wait for current bus instruction fetch
  signal bus_d_wait : std_ulogic; -- wait for current bus data access
  signal csr_rdata  : std_ulogic_vector(data_width_c-1 downto 0); -- csr read data
  signal mar        : std_ulogic_vector(data_width_c-1 downto 0); -- current memory address register
  signal ma_instr   : std_ulogic; -- misaligned instruction address
  signal ma_load    : std_ulogic; -- misaligned load data address
  signal ma_store   : std_ulogic; -- misaligned store data address
  signal excl_state : std_ulogic; -- atomic/exclusive access lock status
  signal be_instr   : std_ulogic; -- bus error on instruction access
  signal be_load    : std_ulogic; -- bus error on load data access
  signal be_store   : std_ulogic; -- bus error on store data access
  signal fetch_pc   : std_ulogic_vector(data_width_c-1 downto 0); -- pc for instruction fetch
  signal curr_pc    : std_ulogic_vector(data_width_c-1 downto 0); -- current pc (for current executed instruction)
  signal next_pc    : std_ulogic_vector(data_width_c-1 downto 0); -- next pc (for next executed instruction)
  signal fpu_flags  : std_ulogic_vector(4 downto 0); -- FPU exception flags

  -- pmp interface --
  signal pmp_addr : pmp_addr_if_t;
  signal pmp_ctrl : pmp_ctrl_if_t;

begin

  -- CPU ISA Configuration ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert false report
  "NEORV32 CPU ISA Configuration (MARCH): " &
  cond_sel_string_f(CPU_EXTENSION_RISCV_E, "RV32E", "RV32I") &
  cond_sel_string_f(CPU_EXTENSION_RISCV_M, "M", "") &
  cond_sel_string_f(CPU_EXTENSION_RISCV_A, "A", "") &
  cond_sel_string_f(CPU_EXTENSION_RISCV_C, "C", "") &
  cond_sel_string_f(CPU_EXTENSION_RISCV_B, "B", "") &
  cond_sel_string_f(CPU_EXTENSION_RISCV_U, "U", "") &
  cond_sel_string_f(CPU_EXTENSION_RISCV_Zicsr, "_Zicsr", "") &
  cond_sel_string_f(CPU_EXTENSION_RISCV_Zicntr, "_Zicntr", "") &
  cond_sel_string_f(CPU_EXTENSION_RISCV_Zihpm, "_Zihpm", "") &
  cond_sel_string_f(CPU_EXTENSION_RISCV_Zifencei, "_Zifencei", "") &
  cond_sel_string_f(CPU_EXTENSION_RISCV_Zfinx, "_Zfinx", "") &
  cond_sel_string_f(CPU_EXTENSION_RISCV_Zmmul, "_Zmmul", "") &
  cond_sel_string_f(CPU_EXTENSION_RISCV_DEBUG, "_Debug", "") &
  ""
  severity note;


  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- hardware reset notifier --
  assert not (dedicated_reset_c = false) report "NEORV32 CPU CONFIG NOTE: Implementing NO dedicated hardware reset for uncritical registers (default, might reduce area). Set package constant <dedicated_reset_c> = TRUE to configure a DEFINED reset value for all CPU registers." severity note;
  assert not (dedicated_reset_c = true)  report "NEORV32 CPU CONFIG NOTE: Implementing defined hardware reset for uncritical registers (non-default, reset-to-zero, might increase area)." severity note;
  assert not ((def_rst_val_c /= '-') and (def_rst_val_c /= '0')) report "NEORV32 CPU CONFIG ERROR! Invalid configuration of package <def_rst_val_c> constant (has to be '-' or '0')." severity error;

  -- CSR system --
  assert not (CPU_EXTENSION_RISCV_Zicsr = false) report "NEORV32 CPU CONFIG WARNING! No exception/interrupt/trap/privileged features available when <CPU_EXTENSION_RISCV_Zicsr> = false." severity warning;

  -- CPU counters (cycle and instret) --
  assert not ((CPU_EXTENSION_RISCV_Zicntr = true) and ((CPU_CNT_WIDTH < 0) or (CPU_CNT_WIDTH > 64))) report "NEORV32 CPU CONFIG ERROR! Invalid <CPU_CNT_WIDTH> configuration. Has to be 0..64." severity error;
  assert not ((CPU_EXTENSION_RISCV_Zicntr = true) and (CPU_CNT_WIDTH < 64)) report "NEORV32 CPU CONFIG WARNING! Implementing CPU <cycle> and <instret> CSRs with reduced size (" & integer'image(CPU_CNT_WIDTH) & "-bit instead of 64-bit). This is not RISC-V compliant and might have unintended SW side effects." severity warning;

  -- U-extension requires Zicsr extension --
  assert not ((CPU_EXTENSION_RISCV_Zicsr = false) and (CPU_EXTENSION_RISCV_U = true)) report "NEORV32 CPU CONFIG ERROR! User mode requires <CPU_EXTENSION_RISCV_Zicsr> extension to be enabled." severity error;

  -- Instruction prefetch buffer size --
  assert not (is_power_of_two_f(CPU_IPB_ENTRIES) = false) report "NEORV32 CPU CONFIG ERROR! Number of entries in instruction prefetch buffer <CPU_IPB_ENTRIES> has to be a power of two." severity error;

  -- Co-processor timeout counter (for debugging only) --
  assert not (cp_timeout_en_c = true) report "NEORV32 CPU CONFIG WARNING! Co-processor timeout counter enabled. This should be used for debugging/simulation only." severity warning;

  -- PMP regions check --
  assert not (PMP_NUM_REGIONS > 64) report "NEORV32 CPU CONFIG ERROR! Number of PMP regions <PMP_NUM_REGIONS> out of valid range (0..64)." severity error;
  -- PMP granularity --
  assert not ((is_power_of_two_f(PMP_MIN_GRANULARITY) = false) and (PMP_NUM_REGIONS > 0)) report "NEORV32 CPU CONFIG ERROR! <PMP_MIN_GRANULARITY> has to be a power of two." severity error;
  assert not ((PMP_MIN_GRANULARITY < 8) and (PMP_NUM_REGIONS > 0)) report "NEORV32 CPU CONFIG ERROR! <PMP_MIN_GRANULARITY> has to be >= 8 bytes." severity error;
  assert not ((CPU_EXTENSION_RISCV_Zicsr = false) and (PMP_NUM_REGIONS > 0)) report "NEORV32 CPU CONFIG ERROR! Physical memory protection (PMP) requires <CPU_EXTENSION_RISCV_Zicsr> extension to be enabled." severity error;

  -- HPM counters check --
  assert not ((CPU_EXTENSION_RISCV_Zihpm = true) and (HPM_NUM_CNTS > 29)) report "NEORV32 CPU CONFIG ERROR! Number of HPM counters <HPM_NUM_CNTS> out of valid range (0..29)." severity error;
  assert not ((CPU_EXTENSION_RISCV_Zihpm = true) and ((HPM_CNT_WIDTH < 0) or (HPM_CNT_WIDTH > 64))) report "NEORV32 CPU CONFIG ERROR! HPM counter width <HPM_CNT_WIDTH> has to be 0..64 bit." severity error; 
  assert not ((CPU_EXTENSION_RISCV_Zicsr = false) and (CPU_EXTENSION_RISCV_Zihpm = true)) report "NEORV32 CPU CONFIG ERROR! Hardware performance monitors extension <CPU_EXTENSION_RISCV_Zihpm> requires <CPU_EXTENSION_RISCV_Zicsr> extension to be enabled." severity error;

  -- Mul-extension --
  assert not ((CPU_EXTENSION_RISCV_Zmmul = true) and (CPU_EXTENSION_RISCV_M = true)) report "NEORV32 CPU CONFIG ERROR! <M> and <Zmmul> extensions cannot co-exist!" severity error;

  -- Debug mode --
  assert not ((CPU_EXTENSION_RISCV_DEBUG = true) and (CPU_EXTENSION_RISCV_Zicsr = false)) report "NEORV32 CPU CONFIG ERROR! Debug mode requires <CPU_EXTENSION_RISCV_Zicsr> extension to be enabled." severity error;
  assert not ((CPU_EXTENSION_RISCV_DEBUG = true) and (CPU_EXTENSION_RISCV_Zifencei = false)) report "NEORV32 CPU CONFIG ERROR! Debug mode requires <CPU_EXTENSION_RISCV_Zifencei> extension to be enabled." severity error;

  -- fast multiplication option --
  assert not (FAST_MUL_EN = true) report "NEORV32 CPU CONFIG NOTE: <FAST_MUL_EN> set. Trying to use DSP blocks for base ISA multiplications." severity note;

  -- fast shift option --
  assert not (FAST_SHIFT_EN = true) report "NEORV32 CPU CONFIG NOTE: <FAST_SHIFT_EN> set. Implementing full-parallel logic / barrel shifters." severity note;


  -- Control Unit ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_control_inst: neorv32_cpu_control
  generic map (
    -- General --
    HW_THREAD_ID                 => HW_THREAD_ID,                 -- hardware thread id
    CPU_BOOT_ADDR                => CPU_BOOT_ADDR,                -- cpu boot address
    CPU_DEBUG_ADDR               => CPU_DEBUG_ADDR,               -- cpu debug mode start address
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        => CPU_EXTENSION_RISCV_A,        -- implement atomic extension?
    CPU_EXTENSION_RISCV_B        => CPU_EXTENSION_RISCV_B,        -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_C        => CPU_EXTENSION_RISCV_C,        -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        => CPU_EXTENSION_RISCV_E,        -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        => CPU_EXTENSION_RISCV_M,        -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U        => CPU_EXTENSION_RISCV_U,        -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    => CPU_EXTENSION_RISCV_Zfinx,    -- implement 32-bit floating-point extension (using INT reg!)
    CPU_EXTENSION_RISCV_Zicsr    => CPU_EXTENSION_RISCV_Zicsr,    -- implement CSR system?
    CPU_EXTENSION_RISCV_Zicntr   => CPU_EXTENSION_RISCV_Zicntr,   -- implement base counters?
    CPU_EXTENSION_RISCV_Zihpm    => CPU_EXTENSION_RISCV_Zihpm,    -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zifencei => CPU_EXTENSION_RISCV_Zifencei, -- implement instruction stream sync.?
    CPU_EXTENSION_RISCV_Zmmul    => CPU_EXTENSION_RISCV_Zmmul,    -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_DEBUG    => CPU_EXTENSION_RISCV_DEBUG,    -- implement CPU debug mode?
    -- Extension Options --
    CPU_CNT_WIDTH                => CPU_CNT_WIDTH,                -- total width of CPU cycle and instret counters (0..64)
    CPU_IPB_ENTRIES              => CPU_IPB_ENTRIES,              -- entries is instruction prefetch buffer, has to be a power of 2
    -- Physical memory protection (PMP) --
    PMP_NUM_REGIONS              => PMP_NUM_REGIONS,              -- number of regions (0..64)
    PMP_MIN_GRANULARITY          => PMP_MIN_GRANULARITY,          -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 => HPM_NUM_CNTS,                 -- number of implemented HPM counters (0..29)
    HPM_CNT_WIDTH                => HPM_CNT_WIDTH                 -- total size of HPM counters
  )
  port map (
    -- global control --
    clk_i         => clk_i,       -- global clock, rising edge
    rstn_i        => rstn_i,      -- global reset, low-active, async
    ctrl_o        => ctrl,        -- main control bus
    -- status input --
    alu_idone_i   => alu_idone,   -- ALU iterative operation done
    bus_i_wait_i  => bus_i_wait,  -- wait for bus
    bus_d_wait_i  => bus_d_wait,  -- wait for bus
    excl_state_i  => excl_state,  -- atomic/exclusive access lock status
    -- data input --
    instr_i       => instr,       -- instruction
    cmp_i         => comparator,  -- comparator status
    alu_add_i     => alu_add,     -- ALU address result
    rs1_i         => rs1,         -- rf source 1
    -- data output --
    imm_o         => imm,         -- immediate
    fetch_pc_o    => fetch_pc,    -- PC for instruction fetch
    curr_pc_o     => curr_pc,     -- current PC (corresponding to current instruction)
    next_pc_o     => next_pc,     -- next PC (corresponding to next instruction)
    csr_rdata_o   => csr_rdata,   -- CSR read data
    -- FPU interface --
    fpu_flags_i   => fpu_flags,   -- exception flags
    -- debug mode (halt) request --
    db_halt_req_i => db_halt_req_i,
    -- interrupts (risc-v compliant) --
    msw_irq_i     => msw_irq_i,   -- machine software interrupt
    mext_irq_i    => mext_irq_i,  -- machine external interrupt
    mtime_irq_i   => mtime_irq_i, -- machine timer interrupt
    -- fast interrupts (custom) --
    firq_i        => firq_i,      -- fast interrupt trigger
    -- system time input from MTIME --
    time_i        => time_i,      -- current system time
    -- physical memory protection --
    pmp_addr_o    => pmp_addr,    -- addresses
    pmp_ctrl_o    => pmp_ctrl,    -- configs
    -- bus access exceptions --
    mar_i         => mar,         -- memory address register
    ma_instr_i    => ma_instr,    -- misaligned instruction address
    ma_load_i     => ma_load,     -- misaligned load data address
    ma_store_i    => ma_store,    -- misaligned store data address
    be_instr_i    => be_instr,    -- bus error on instruction access
    be_load_i     => be_load,     -- bus error on load data access
    be_store_i    => be_store     -- bus error on store data access
  );

  -- CPU is sleeping? --
  sleep_o <= ctrl(ctrl_sleep_c); -- set when CPU is sleeping (after WFI)

  -- CPU is in debug mode? --
  debug_o <= ctrl(ctrl_debug_running_c);


  -- Register File --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_regfile_inst: neorv32_cpu_regfile
  generic map (
    CPU_EXTENSION_RISCV_E => CPU_EXTENSION_RISCV_E -- implement embedded RF extension?
  )
  port map (
    -- global control --
    clk_i  => clk_i,              -- global clock, rising edge
    ctrl_i => ctrl,               -- main control bus
    -- data input --
    mem_i  => mem_rdata,          -- memory read data
    alu_i  => alu_res,            -- ALU result
    -- data output --
    rs1_o  => rs1,                -- operand 1
    rs2_o  => rs2                 -- operand 2
  );


  -- ALU ------------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_alu_inst: neorv32_cpu_alu
  generic map (
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_B     => CPU_EXTENSION_RISCV_B,     -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_M     => CPU_EXTENSION_RISCV_M,     -- implement mul/div extension?
    CPU_EXTENSION_RISCV_Zmmul => CPU_EXTENSION_RISCV_Zmmul, -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zfinx => CPU_EXTENSION_RISCV_Zfinx, -- implement 32-bit floating-point extension (using INT reg!)
    -- Extension Options --
    FAST_MUL_EN               => FAST_MUL_EN,               -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN             => FAST_SHIFT_EN              -- use barrel shifter for shift operations
  )
  port map (
    -- global control --
    clk_i       => clk_i,         -- global clock, rising edge
    rstn_i      => rstn_i,        -- global reset, low-active, async
    ctrl_i      => ctrl,          -- main control bus
    -- data input --
    rs1_i       => rs1,           -- rf source 1
    rs2_i       => rs2,           -- rf source 2
    pc_i        => curr_pc,       -- current PC
    pc2_i       => next_pc,       -- next PC
    imm_i       => imm,           -- immediate
    csr_i       => csr_rdata,     -- CSR read data
    -- data output --
    cmp_o       => comparator,    -- comparator status
    res_o       => alu_res,       -- ALU result
    add_o       => alu_add,       -- address computation result
    fpu_flags_o => fpu_flags,     -- FPU exception flags
    -- status --
    idone_o     => alu_idone      -- iterative processing units done?
  );


  -- Bus Interface Unit ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_bus_inst: neorv32_cpu_bus
  generic map (
    CPU_EXTENSION_RISCV_A => CPU_EXTENSION_RISCV_A, -- implement atomic extension?
    CPU_EXTENSION_RISCV_C => CPU_EXTENSION_RISCV_C, -- implement compressed extension?
    -- Physical memory protection (PMP) --
    PMP_NUM_REGIONS       => PMP_NUM_REGIONS,       -- number of regions (0..64)
    PMP_MIN_GRANULARITY   => PMP_MIN_GRANULARITY    -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
  )
  port map (
    -- global control --
    clk_i          => clk_i,          -- global clock, rising edge
    rstn_i         => rstn_i,         -- global reset, low-active, async
    ctrl_i         => ctrl,           -- main control bus
    -- cpu instruction fetch interface --
    fetch_pc_i     => fetch_pc,       -- PC for instruction fetch
    instr_o        => instr,          -- instruction
    i_wait_o       => bus_i_wait,     -- wait for fetch to complete
    --
    ma_instr_o     => ma_instr,       -- misaligned instruction address
    be_instr_o     => be_instr,       -- bus error on instruction access
    -- cpu data access interface --
    addr_i         => alu_add,        -- ALU.add result -> access address
    wdata_i        => rs2,            -- write data
    rdata_o        => mem_rdata,      -- read data
    mar_o          => mar,            -- current memory address register
    d_wait_o       => bus_d_wait,     -- wait for access to complete
    --
    excl_state_o   => excl_state,     -- atomic/exclusive access status
    ma_load_o      => ma_load,        -- misaligned load data address
    ma_store_o     => ma_store,       -- misaligned store data address
    be_load_o      => be_load,        -- bus error on load data access
    be_store_o     => be_store,       -- bus error on store data access
    -- physical memory protection --
    pmp_addr_i     => pmp_addr,       -- addresses
    pmp_ctrl_i     => pmp_ctrl,       -- configurations
    -- instruction bus --
    i_bus_addr_o   => i_bus_addr_o,   -- bus access address
    i_bus_rdata_i  => i_bus_rdata_i,  -- bus read data
    i_bus_wdata_o  => i_bus_wdata_o,  -- bus write data
    i_bus_ben_o    => i_bus_ben_o,    -- byte enable
    i_bus_we_o     => i_bus_we_o,     -- write enable
    i_bus_re_o     => i_bus_re_o,     -- read enable
    i_bus_lock_o   => i_bus_lock_o,   -- exclusive access request
    i_bus_ack_i    => i_bus_ack_i,    -- bus transfer acknowledge
    i_bus_err_i    => i_bus_err_i,    -- bus transfer error
    i_bus_fence_o  => i_bus_fence_o,  -- fence operation
    -- data bus --
    d_bus_addr_o   => d_bus_addr_o,   -- bus access address
    d_bus_rdata_i  => d_bus_rdata_i,  -- bus read data
    d_bus_wdata_o  => d_bus_wdata_o,  -- bus write data
    d_bus_ben_o    => d_bus_ben_o,    -- byte enable
    d_bus_we_o     => d_bus_we_o,     -- write enable
    d_bus_re_o     => d_bus_re_o,     -- read enable
    d_bus_lock_o   => d_bus_lock_o,   -- exclusive access request
    d_bus_ack_i    => d_bus_ack_i,    -- bus transfer acknowledge
    d_bus_err_i    => d_bus_err_i,    -- bus transfer error
    d_bus_fence_o  => d_bus_fence_o   -- fence operation
  );

  -- current privilege level --
  i_bus_priv_o <= ctrl(ctrl_priv_lvl_msb_c downto ctrl_priv_lvl_lsb_c);
  d_bus_priv_o <= ctrl(ctrl_priv_lvl_msb_c downto ctrl_priv_lvl_lsb_c);


end neorv32_cpu_rtl;
