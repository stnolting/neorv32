-- #################################################################################################
-- # << NEORV32 - CPU Operations Control Unit >>                                                   #
-- # ********************************************************************************************* #
-- # CPU operations are controlled by several "engines" (modules). These engines operate in        #
-- # parallel to implement a simple pipeline:                                                      #
-- #  + Fetch engine:   Fetches 32-bit chunks of instruction words                                 #
-- #  + Issue engine:   Decodes compressed instructions, aligns and queues instruction words       #
-- #  + Execute engine: Multi-cycle execution of instructions (generate control signals)           #
-- #  + Trap engine:    Handles interrupts and exceptions                                          #
-- #  + CSR module:     Read/write access to control and status registers                          #
-- #  + Debug module:   CPU debug mode handling (on-chip debugger)                                 #
-- #  + Trigger module: Hardware-assisted breakpoints (on-chip debugger)                           #
-- #                                                                                               #
-- # NOTE: If <dedicated_reset_c> = true then <def_rst_val_c> evaluates to '-'. Registers that     #
-- #       reset to <def_rst_val_c> do NOT actually have a real reset by default and have to be    #
-- #       explicitly initialized by software! This is only used for "uncritical" registers. Many  #
-- #       of them will be initialized by the default crt0 start-up code.                          #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
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

entity neorv32_cpu_control is
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
    CPU_EXTENSION_RISCV_M        : boolean; -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U        : boolean; -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    : boolean; -- implement 32-bit floating-point extension (using INT reg!)
    CPU_EXTENSION_RISCV_Zicsr    : boolean; -- implement CSR system?
    CPU_EXTENSION_RISCV_Zicntr   : boolean; -- implement base counters?
    CPU_EXTENSION_RISCV_Zihpm    : boolean; -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zifencei : boolean; -- implement instruction stream sync.?
    CPU_EXTENSION_RISCV_Zmmul    : boolean; -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zxcfu    : boolean; -- implement custom (instr.) functions unit?
    CPU_EXTENSION_RISCV_DEBUG    : boolean; -- implement CPU debug mode?
    -- Tuning Options --
    FAST_MUL_EN                  : boolean; -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                : boolean; -- use barrel shifter for shift operations
    CPU_CNT_WIDTH                : natural; -- total width of CPU cycle and instret counters (0..64)
    CPU_IPB_ENTRIES              : natural; -- entries is instruction prefetch buffer, has to be a power of 2
    -- Physical memory protection (PMP) --
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
    ctrl_o        : out std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
    -- status input --
    alu_idone_i   : in  std_ulogic; -- ALU iterative operation done
    bus_i_wait_i  : in  std_ulogic; -- wait for bus
    bus_d_wait_i  : in  std_ulogic; -- wait for bus
    excl_state_i  : in  std_ulogic; -- atomic/exclusive access lock status
    -- data input --
    instr_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- instruction
    cmp_i         : in  std_ulogic_vector(1 downto 0); -- comparator status
    alu_add_i     : in  std_ulogic_vector(data_width_c-1 downto 0); -- ALU address result
    rs1_i         : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 1
    -- data output --
    imm_o         : out std_ulogic_vector(data_width_c-1 downto 0); -- immediate
    fetch_pc_o    : out std_ulogic_vector(data_width_c-1 downto 0); -- PC for instruction fetch
    curr_pc_o     : out std_ulogic_vector(data_width_c-1 downto 0); -- current PC (corresponding to current instruction)
    next_pc_o     : out std_ulogic_vector(data_width_c-1 downto 0); -- next PC (corresponding to next instruction)
    csr_rdata_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- CSR read data
    -- FPU interface --
    fpu_flags_i   : in  std_ulogic_vector(04 downto 0); -- exception flags
    -- debug mode (halt) request --
    db_halt_req_i : in  std_ulogic;
    -- interrupts (risc-v compliant) --
    msw_irq_i     : in  std_ulogic; -- machine software interrupt
    mext_irq_i    : in  std_ulogic; -- machine external interrupt
    mtime_irq_i   : in  std_ulogic; -- machine timer interrupt
    -- fast interrupts (custom) --
    firq_i        : in  std_ulogic_vector(15 downto 0);
    -- system time input from MTIME --
    time_i        : in  std_ulogic_vector(63 downto 0); -- current system time
    -- physical memory protection --
    pmp_addr_o    : out pmp_addr_if_t; -- addresses
    pmp_ctrl_o    : out pmp_ctrl_if_t; -- configs
    -- bus access exceptions --
    mar_i         : in  std_ulogic_vector(data_width_c-1 downto 0); -- memory address register
    ma_instr_i    : in  std_ulogic; -- misaligned instruction address
    ma_load_i     : in  std_ulogic; -- misaligned load data address
    ma_store_i    : in  std_ulogic; -- misaligned store data address
    be_instr_i    : in  std_ulogic; -- bus error on instruction access
    be_load_i     : in  std_ulogic; -- bus error on load data access
    be_store_i    : in  std_ulogic  -- bus error on store data access
  );
end neorv32_cpu_control;

architecture neorv32_cpu_control_rtl of neorv32_cpu_control is

  -- CPU core counter ([m]cycle, [m]instret) width - high/low parts --
  constant cpu_cnt_lo_width_c : natural := natural(cond_sel_int_f(boolean(CPU_CNT_WIDTH < 32), CPU_CNT_WIDTH, 32));
  constant cpu_cnt_hi_width_c : natural := natural(cond_sel_int_f(boolean(CPU_CNT_WIDTH > 32), CPU_CNT_WIDTH-32, 0));

  -- HPM counter width - high/low parts --
  constant hpm_cnt_lo_width_c : natural := natural(cond_sel_int_f(boolean(HPM_CNT_WIDTH < 32), HPM_CNT_WIDTH, 32));
  constant hpm_cnt_hi_width_c : natural := natural(cond_sel_int_f(boolean(HPM_CNT_WIDTH > 32), HPM_CNT_WIDTH-32, 0));

  -- instruction fetch engine --
  type fetch_engine_state_t is (IFETCH_REQUEST, IFETCH_ISSUE);
  type fetch_engine_t is record
    state       : fetch_engine_state_t;
    state_nxt   : fetch_engine_state_t;
    state_prev  : fetch_engine_state_t;
    restart     : std_ulogic;
    restart_nxt : std_ulogic;
    pc          : std_ulogic_vector(data_width_c-1 downto 0);
    pc_nxt      : std_ulogic_vector(data_width_c-1 downto 0);
    reset       : std_ulogic;
    bus_ir      : std_ulogic;
  end record;
  signal fetch_engine : fetch_engine_t;

  -- instruction prefetch buffer (FIFO) interface --
  type ipb_t is record
    wdata : std_ulogic_vector(2+31 downto 0); -- write status (bus_error, align_error) + 32-bit instruction data
    we    : std_ulogic; -- trigger write
    free  : std_ulogic; -- free entry available?
    clear : std_ulogic; -- clear all entries
    --
    rdata : std_ulogic_vector(2+31 downto 0); -- read data: status (bus_error, align_error) + 32-bit instruction data
    re    : std_ulogic; -- read enable
    avail : std_ulogic; -- data available?
  end record;
  signal ipb : ipb_t;

  -- pre-decoder --
  signal ci_instr16 : std_ulogic_vector(15 downto 0);
  signal ci_instr32 : std_ulogic_vector(31 downto 0);
  signal ci_illegal : std_ulogic;

  -- instruction issue engine --
  type issue_engine_t is record
    realign     : std_ulogic;
    realign_nxt : std_ulogic;
    align       : std_ulogic;
    align_nxt   : std_ulogic;
    buf         : std_ulogic_vector(2+15 downto 0);
    buf_nxt     : std_ulogic_vector(2+15 downto 0);
  end record;
  signal issue_engine : issue_engine_t;

  -- instruction issue interface --
  type cmd_issue_t is record
    data  : std_ulogic_vector(35 downto 0); -- 4-bit status + 32-bit instruction
    valid : std_ulogic; -- data word is valid when set
  end record;
  signal cmd_issue : cmd_issue_t;

  -- instruction decoding helper logic --
  type decode_aux_t is record
    is_a_lr  : std_ulogic;
    is_a_sc  : std_ulogic;
    is_f_op  : std_ulogic;
    is_m_mul : std_ulogic;
    is_m_div : std_ulogic;
    is_b_imm : std_ulogic;
    is_b_reg : std_ulogic;
    rs1_zero : std_ulogic;
    rd_zero  : std_ulogic;
  end record;
  signal decode_aux : decode_aux_t;

  -- instruction execution engine --
  type execute_engine_state_t is (DISPATCH, TRAP_ENTER, TRAP_EXIT, TRAP_EXECUTE, EXECUTE, ALU_WAIT,
                                  BRANCH, LOADSTORE_0, LOADSTORE_1, LOADSTORE_2, SYS_ENV, CSR_ACCESS);
  type execute_engine_t is record
    state        : execute_engine_state_t;
    state_nxt    : execute_engine_state_t;
    state_prev   : execute_engine_state_t;
    --
    i_reg        : std_ulogic_vector(31 downto 0);
    i_reg_nxt    : std_ulogic_vector(31 downto 0);
    i_reg_last   : std_ulogic_vector(31 downto 0); -- last executed instruction
    --
    is_ci        : std_ulogic; -- current instruction is de-compressed instruction
    is_ci_nxt    : std_ulogic;
    is_ici       : std_ulogic; -- current instruction is illegal de-compressed instruction
    is_ici_nxt   : std_ulogic;
    --
    branch_taken : std_ulogic; -- branch condition fulfilled
    pc           : std_ulogic_vector(data_width_c-1 downto 0); -- actual PC, corresponding to current executed instruction
    pc_mux_sel   : std_ulogic; -- source select for PC update
    pc_we        : std_ulogic; -- PC update enabled
    next_pc      : std_ulogic_vector(data_width_c-1 downto 0); -- next PC, corresponding to next instruction to be executed
    next_pc_inc  : std_ulogic_vector(data_width_c-1 downto 0); -- increment to get next PC
    pc_last      : std_ulogic_vector(data_width_c-1 downto 0); -- PC of last executed instruction
    --
    sleep        : std_ulogic; -- CPU in sleep mode
    sleep_nxt    : std_ulogic;
    branched     : std_ulogic; -- instruction fetch was reset
    branched_nxt : std_ulogic;
  end record;
  signal execute_engine : execute_engine_t;

  -- trap controller --
  type trap_ctrl_t is record
    exc_buf       : std_ulogic_vector(exc_width_c-1 downto 0);
    exc_fire      : std_ulogic; -- set if there is a valid source in the exception buffer
    irq_buf       : std_ulogic_vector(irq_width_c-1 downto 0);
    irq_fire      : std_ulogic; -- set if there is a valid source in the interrupt buffer
    cause         : std_ulogic_vector(6 downto 0); -- trap ID for mcause CSR
    cause_nxt     : std_ulogic_vector(6 downto 0);
    db_irq_fire   : std_ulogic; -- set if there is a valid IRQ source in the "enter debug mode" trap buffer
    db_irq_en     : std_ulogic; -- set if IRQs are allowed in debug mode
    --
    env_start     : std_ulogic; -- start trap handler env
    env_start_ack : std_ulogic; -- start of trap handler acknowledged
    env_end       : std_ulogic; -- end trap handler env
    --
    instr_be      : std_ulogic; -- instruction fetch bus error
    instr_ma      : std_ulogic; -- instruction fetch misaligned address
    instr_il      : std_ulogic; -- illegal instruction
    env_call      : std_ulogic; -- ecall instruction
    break_point   : std_ulogic; -- ebreak instruction
  end record;
  signal trap_ctrl : trap_ctrl_t;
  
  -- CPU main control bus --
  signal ctrl_nxt, ctrl : std_ulogic_vector(ctrl_width_c-1 downto 0);

  -- RISC-V control and status registers (CSRs) --
  type pmpcfg_t       is array (0 to 15) of std_ulogic_vector(7 downto 0);
  type pmpaddr_t      is array (0 to PMP_NUM_REGIONS-1) of std_ulogic_vector(data_width_c-3 downto index_size_f(PMP_MIN_GRANULARITY)-2);
  type mhpmevent_t    is array (0 to HPM_NUM_CNTS-1) of std_ulogic_vector(hpmcnt_event_size_c-1 downto 0);
  type mhpmevent_rd_t is array (0 to 28) of std_ulogic_vector(data_width_c-1 downto 0);
  type mhpmcnt_t      is array (0 to HPM_NUM_CNTS-1) of std_ulogic_vector(data_width_c-1 downto 0);
  type mhpmcnt_nxt_t  is array (0 to HPM_NUM_CNTS-1) of std_ulogic_vector(data_width_c downto 0);
  type mhpmcnt_ovfl_t is array (0 to HPM_NUM_CNTS-1) of std_ulogic_vector(0 downto 0);
  type mhpmcnt_rd_t   is array (0 to 29) of std_ulogic_vector(data_width_c-1 downto 0);
  type csr_t is record
    addr              : std_ulogic_vector(11 downto 0); -- csr address
    we                : std_ulogic; -- csr write enable
    we_nxt            : std_ulogic;
    wdata             : std_ulogic_vector(data_width_c-1 downto 0); -- csr write data
    rdata             : std_ulogic_vector(data_width_c-1 downto 0); -- csr read data
    --
    mstatus_mie       : std_ulogic; -- mstatus.MIE: global IRQ enable (R/W)
    mstatus_mpie      : std_ulogic; -- mstatus.MPIE: previous global IRQ enable (R/W)
    mstatus_mpp       : std_ulogic; -- mstatus.MPP: machine previous privilege mode
    mstatus_tw        : std_ulogic; -- mstatus.TW: do not allow user mode to execute WFI instruction when set
    --
    mie_msie          : std_ulogic; -- mie.MSIE: machine software interrupt enable (R/W)
    mie_meie          : std_ulogic; -- mie.MEIE: machine external interrupt enable (R/W)
    mie_mtie          : std_ulogic; -- mie.MEIE: machine timer interrupt enable (R/W)
    mie_firqe         : std_ulogic_vector(15 downto 0); -- mie.firq*e: fast interrupt enabled (R/W)
    --
    mip_firq_nclr     : std_ulogic_vector(15 downto 0); -- clear pending FIRQ (active-low)
    --
    mcounteren_cy     : std_ulogic; -- mcounteren.cy: allow cycle[h] access from user-mode
    mcounteren_tm     : std_ulogic; -- mcounteren.tm: allow time[h] access from user-mode
    mcounteren_ir     : std_ulogic; -- mcounteren.ir: allow instret[h] access from user-mode
    --
    mcountinhibit_cy  : std_ulogic; -- mcounterinhibit.cy: enable auto-increment for [m]cycle[h]
    mcountinhibit_ir  : std_ulogic; -- mcounterinhibit.ir: enable auto-increment for [m]instret[h]
    mcountinhibit_hpm : std_ulogic_vector(HPM_NUM_CNTS-1 downto 0); -- mcounterinhibit.hpm3: enable auto-increment for mhpmcounterx[h]
    --
    privilege         : std_ulogic; -- current privilege mode
    privilege_eff     : std_ulogic; -- current *effective* privilege mode
    --
    mepc              : std_ulogic_vector(data_width_c-1 downto 0); -- mepc: machine exception pc (R/W)
    mcause            : std_ulogic_vector(5 downto 0); -- mcause: machine trap cause (R/W)
    mtvec             : std_ulogic_vector(data_width_c-1 downto 0); -- mtvec: machine trap-handler base address (R/W), bit 1:0 == 00
    mtval             : std_ulogic_vector(data_width_c-1 downto 0); -- mtval: machine bad address or instruction (R/W)
    --
    mhpmevent         : mhpmevent_t; -- mhpmevent*: machine performance-monitoring event selector (R/W)
    mhpmevent_rd      : mhpmevent_rd_t; -- read data
    --
    mscratch          : std_ulogic_vector(data_width_c-1 downto 0); -- mscratch: scratch register (R/W)
    --
    mcycle            : std_ulogic_vector(data_width_c-1 downto 0); -- mcycle (R/W)
    mcycle_nxt        : std_ulogic_vector(data_width_c downto 0);
    mcycle_ovfl       : std_ulogic_vector(00 downto 0); -- counter low-to-high-word overflow
    mcycleh           : std_ulogic_vector(data_width_c-1 downto 0); -- mcycleh (R/W)
    minstret          : std_ulogic_vector(data_width_c-1 downto 0); -- minstret (R/W)
    minstret_nxt      : std_ulogic_vector(data_width_c downto 0);
    minstret_ovfl     : std_ulogic_vector(00 downto 0); -- counter low-to-high-word overflow
    minstreth         : std_ulogic_vector(data_width_c-1 downto 0); -- minstreth (R/W)
    --
    mhpmcounter       : mhpmcnt_t; -- mhpmcounter* (R/W), plus carry bit
    mhpmcounter_nxt   : mhpmcnt_nxt_t;
    mhpmcounter_ovfl  : mhpmcnt_ovfl_t; -- counter low-to-high-word overflow
    mhpmcounterh      : mhpmcnt_t; -- mhpmcounter*h (R/W)
    mhpmcounter_rd    : mhpmcnt_rd_t; -- mhpmcounter* (R/W): actual read data
    mhpmcounterh_rd   : mhpmcnt_rd_t; -- mhpmcounter*h (R/W): actual read data
    --
    pmpcfg            : pmpcfg_t; -- physical memory protection - configuration registers
    pmpaddr           : pmpaddr_t; -- physical memory protection - address registers (bits 33:2 of PHYSICAL address)
    --
    frm               : std_ulogic_vector(02 downto 0); -- frm (R/W): FPU rounding mode
    fflags            : std_ulogic_vector(04 downto 0); -- fflags (R/W): FPU exception flags
    --
    dcsr_ebreakm      : std_ulogic; -- dcsr.ebreakm (R/W): behavior of ebreak instruction on m-mode
    dcsr_ebreaku      : std_ulogic; -- dcsr.ebreaku (R/W): behavior of ebreak instruction on u-mode
    dcsr_step         : std_ulogic; -- dcsr.step (R/W): single-step mode
    dcsr_prv          : std_ulogic; -- dcsr.prv (R/W): current privilege level when entering debug mode
    dcsr_cause        : std_ulogic_vector(02 downto 0); -- dcsr.cause (R/-): why was debug mode entered
    dcsr_rd           : std_ulogic_vector(data_width_c-1 downto 0); -- dcsr (R/(W)): debug mode control and status register
    dpc               : std_ulogic_vector(data_width_c-1 downto 0); -- dpc (R/W): debug mode program counter
    dscratch0         : std_ulogic_vector(data_width_c-1 downto 0); -- dscratch0 (R/W): debug mode scratch register 0
    --
    tdata1_exe        : std_ulogic; -- enable (match) trigger
    tdata1_rd         : std_ulogic_vector(data_width_c-1 downto 0); -- tdata1 (R/(W)): trigger register read-back
    tdata2            : std_ulogic_vector(data_width_c-1 downto 0); -- tdata2 (R/W): address-match register
  end record;
  signal csr : csr_t;

  -- debug mode controller --
  type debug_ctrl_state_t is (DEBUG_OFFLINE, DEBUG_PENDING, DEBUG_ONLINE, DEBUG_EXIT);
  type debug_ctrl_t is record
    state        : debug_ctrl_state_t;
    -- decoded state --
    running      : std_ulogic; -- debug mode active
    pending      : std_ulogic; -- waiting to start debug mode
    -- entering triggers --
    trig_hw      : std_ulogic; -- hardware trigger
    trig_break   : std_ulogic; -- ebreak instruction
    trig_halt    : std_ulogic; -- external request
    trig_step    : std_ulogic; -- single-stepping mode
    -- leave debug mode --
    dret         : std_ulogic; -- executed DRET instruction
    -- misc --
    ext_halt_req : std_ulogic;
  end record;
  signal debug_ctrl : debug_ctrl_t;

  -- (hpm) counter events --
  signal cnt_event      : std_ulogic_vector(hpmcnt_event_size_c-1 downto 0);
  signal hpmcnt_trigger : std_ulogic_vector(HPM_NUM_CNTS-1 downto 0);

  -- illegal instruction check --
  signal illegal_opcode_lsbs : std_ulogic; -- opcode != rv32
  signal illegal_instruction : std_ulogic;
  signal illegal_register    : std_ulogic; -- illegal register (>x15) - E-extension
  signal illegal_compressed  : std_ulogic; -- illegal compressed instruction - C-extension

  -- access (privilege) check --
  signal csr_acc_valid : std_ulogic; -- valid CSR access (implemented and valid access rights)

  -- hardware trigger module --
  signal hw_trigger_fire : std_ulogic;

begin

-- ****************************************************************************************************************************
-- Instruction Fetch (always fetch 32-bit-aligned 32-bit chunks of data)
-- ****************************************************************************************************************************

  -- Fetch Engine FSM Sync ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  fetch_engine_fsm_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      fetch_engine.state      <= IFETCH_REQUEST;
      fetch_engine.state_prev <= IFETCH_REQUEST;
      fetch_engine.restart    <= '1';
      fetch_engine.pc         <= (others => def_rst_val_c);
    elsif rising_edge(clk_i) then
      fetch_engine.state      <= fetch_engine.state_nxt;
      fetch_engine.state_prev <= fetch_engine.state;
      fetch_engine.restart    <= fetch_engine.restart_nxt or fetch_engine.reset;
      fetch_engine.pc         <= fetch_engine.pc_nxt;
    end if;
  end process fetch_engine_fsm_sync;

  -- PC output --
  fetch_pc_o <= fetch_engine.pc(data_width_c-1 downto 1) & '0'; -- half-word aligned


  -- Fetch Engine FSM Comb ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  fetch_engine_fsm_comb: process(fetch_engine, execute_engine, ipb, instr_i, bus_i_wait_i, be_instr_i, ma_instr_i)
  begin
    -- arbiter defaults --
    fetch_engine.bus_ir      <= '0';
    fetch_engine.state_nxt   <= fetch_engine.state;
    fetch_engine.pc_nxt      <= fetch_engine.pc;
    fetch_engine.restart_nxt <= fetch_engine.restart;

    -- instruction prefetch buffer defaults --
    ipb.we    <= '0';
    ipb.wdata <= be_instr_i & ma_instr_i & instr_i(31 downto 0); -- store exception info and instruction word

    -- state machine --
    if (fetch_engine.state = IFETCH_REQUEST) then -- IFETCH_REQUEST: request new 32-bit-aligned instruction word
    -- ------------------------------------------------------------
      if (fetch_engine.restart = '1') then -- reset request
        fetch_engine.pc_nxt <= execute_engine.pc(data_width_c-1 downto 1) & '0'; -- initialize with "real" application PC
      elsif (ipb.free = '1') then -- free entry in buffer
        fetch_engine.bus_ir    <= '1'; -- instruction fetch request
        fetch_engine.state_nxt <= IFETCH_ISSUE;
      end if;
      fetch_engine.restart_nxt <= '0';

    else -- IFETCH_ISSUE: store instruction data to prefetch buffer
    -- ------------------------------------------------------------
      if (bus_i_wait_i = '0') or (be_instr_i = '1') or (ma_instr_i = '1') then -- wait for bus response
        fetch_engine.pc_nxt    <= std_ulogic_vector(unsigned(fetch_engine.pc) + 4);
        ipb.we                 <= not fetch_engine.restart; -- write to IPB if not being reset
        fetch_engine.state_nxt <= IFETCH_REQUEST;
      end if;

    end if;
  end process fetch_engine_fsm_comb;

  -- clear instruction prefetch buffer while being reset --
  ipb.clear <= fetch_engine.restart or fetch_engine.reset;


-- ****************************************************************************************************************************
-- Instruction Prefetch Buffer
-- ****************************************************************************************************************************

  -- Instruction Prefetch Buffer (FIFO) -----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  instr_prefetch_buffer: neorv32_fifo
  generic map (
    FIFO_DEPTH => CPU_IPB_ENTRIES,  -- number of fifo entries; has to be a power of two; min 1
    FIFO_WIDTH => ipb.wdata'length, -- size of data elements in fifo
    FIFO_RSYNC => false,            -- we NEED to read data asynchronously
    FIFO_SAFE  => false             -- no safe access required (ensured by FIFO-external control)
  )
  port map (
    -- control --
    clk_i   => clk_i,     -- clock, rising edge
    rstn_i  => '1',       -- async reset, low-active
    clear_i => ipb.clear, -- sync reset, high-active
    level_o => open,
    half_o  => open,
    -- write port --
    wdata_i => ipb.wdata, -- write data
    we_i    => ipb.we,    -- write enable
    free_o  => ipb.free,  -- at least one entry is free when set
    -- read port --
    re_i    => ipb.re,    -- read enable
    rdata_o => ipb.rdata, -- read data
    avail_o => ipb.avail  -- data available when set
  );


-- ****************************************************************************************************************************
-- Instruction Issue (recoding of compressed instructions and 32-bit instruction word construction)
-- ****************************************************************************************************************************

  -- Issue Engine FSM Sync ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  issue_engine_fsm_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then -- always start aligned after reset
      issue_engine.align   <= '0';
      issue_engine.realign <= '0';
      issue_engine.buf     <= (others => def_rst_val_c);
    elsif rising_edge(clk_i) then
      if (ipb.clear = '1') then
        if (CPU_EXTENSION_RISCV_C = true) and (execute_engine.pc(1) = '1') then -- branch to unaligned address?
          issue_engine.align   <= '1'; -- aligned on 16-bit boundary
          issue_engine.realign <= '1';
        else
          issue_engine.align   <= '0'; -- aligned on 32-bit boundary
          issue_engine.realign <= '0';
        end if;
      else
        issue_engine.align   <= issue_engine.align_nxt;
        issue_engine.realign <= issue_engine.realign_nxt;
      end if;
      issue_engine.buf <= issue_engine.buf_nxt;
    end if;
  end process issue_engine_fsm_sync;


  -- Issue Engine FSM Comb ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  issue_engine_fsm_comb: process(issue_engine, ipb, execute_engine, ci_illegal, ci_instr32)
  begin
    -- arbiter defaults --
    issue_engine.realign_nxt <= issue_engine.realign;
    issue_engine.align_nxt   <= issue_engine.align;
    issue_engine.buf_nxt     <= issue_engine.buf;

    -- instruction prefetch buffer interface defaults --
    ipb.re <= '0';

    -- instruction issue interface defaults --
    cmd_issue.valid <= '0';

    -- construct instruction data --
    -- cmd_issue.data = <illegal_compressed_instruction> & <bus_error & alignment_error> & <is_compressed_instrucion> & <32-bit_instruction_word>
    if (issue_engine.align = '0') or (CPU_EXTENSION_RISCV_C = false) then -- 32-bit aligned
      if (ipb.rdata(1 downto 0) = "11") or (CPU_EXTENSION_RISCV_C = false) then -- uncompressed
        cmd_issue.data <= '0' & ipb.rdata(33 downto 32) & '0' & ipb.rdata(31 downto 0);
      else -- compressed
        cmd_issue.data <= ci_illegal & ipb.rdata(33 downto 32) & '1' & ci_instr32;
      end if;
    else -- not 32-bit aligned
      if (issue_engine.buf(1 downto 0) = "11") then -- uncompressed
        cmd_issue.data <= '0' & (ipb.rdata(33 downto 32) or issue_engine.buf(17 downto 16)) & '0' & (ipb.rdata(15 downto 0) & issue_engine.buf(15 downto 0));
      else -- compressed
        cmd_issue.data <= ci_illegal & ipb.rdata(33 downto 32) & '1' & ci_instr32;
      end if;
    end if;

    -- store high half-word - we might need it for an unaligned uncompressed instruction --
    if (execute_engine.state = DISPATCH) and (ipb.avail = '1') and (CPU_EXTENSION_RISCV_C = true) then
      issue_engine.buf_nxt <= ipb.rdata(33 downto 32) & ipb.rdata(31 downto 16);
    end if;

    -- state machine --
    if (ipb.avail = '1') then -- instruction data available?

      if (issue_engine.realign = '0') then -- issue instruction if available
      -- ------------------------------------------------------------
        cmd_issue.valid <= '1';
        if (issue_engine.align = '0') or (CPU_EXTENSION_RISCV_C = false) then -- begin check in LOW instruction half-word
          if (execute_engine.state = DISPATCH) then -- ready to issue new command?
            ipb.re <= '1';
            if (ipb.rdata(1 downto 0) /= "11") and (CPU_EXTENSION_RISCV_C = true) then -- compressed
              issue_engine.align_nxt <= '1';
            end if;
          end if;
        else -- begin check in HIGH instruction half-word
          if (execute_engine.state = DISPATCH) then -- ready to issue new command?
            if (issue_engine.buf(1 downto 0) = "11") then -- uncompressed and unaligned
              ipb.re <= '1';
            else -- compressed - do not read from ipb here!
              issue_engine.align_nxt <= '0';
            end if;
          end if;
        end if;

      else -- re-align input fifo and half-word buffer after a branch to an unaligned address
      -- ------------------------------------------------------------
        ipb.re <= '1';
        issue_engine.realign_nxt <= '0';
      end if;

    end if;
  end process issue_engine_fsm_comb;

  -- 16-bit instructions: half-word select --
  ci_instr16 <= ipb.rdata(15 downto 0) when (issue_engine.align = '0') else issue_engine.buf(15 downto 0);


  -- Compressed Instructions Recoding -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_decompressor_inst_true:
  if (CPU_EXTENSION_RISCV_C = true) generate
    neorv32_cpu_decompressor_inst: neorv32_cpu_decompressor
    generic map (
      FPU_ENABLE => CPU_EXTENSION_RISCV_Zfinx -- floating-point instruction enabled
    )
    port map (
      -- instruction input --
      ci_instr16_i => ci_instr16, -- compressed instruction input
      -- instruction output --
      ci_illegal_o => ci_illegal, -- is an illegal compressed instruction
      ci_instr32_o => ci_instr32  -- 32-bit decompressed instruction
    );
  end generate;

  neorv32_cpu_decompressor_inst_false:
  if (CPU_EXTENSION_RISCV_C = false) generate
    ci_instr32 <= (others => '0');
    ci_illegal <= '0';
  end generate;


-- ****************************************************************************************************************************
-- Instruction Execution
-- ****************************************************************************************************************************

  -- Immediate Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  imm_gen: process(clk_i)
    variable opcode_v : std_ulogic_vector(6 downto 0);
  begin
    if rising_edge(clk_i) then
      -- default: I-immediate: ALU-immediate, loads, jump-and-link with register
      imm_o(31 downto 11) <= (others => execute_engine.i_reg(31)); -- sign extension
      imm_o(10 downto 05) <= execute_engine.i_reg(30 downto 25);
      imm_o(04 downto 01) <= execute_engine.i_reg(24 downto 21);
      imm_o(00)           <= execute_engine.i_reg(20);

      opcode_v := execute_engine.i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c+2) & "11";
      case opcode_v is -- save some bits here, the two LSBs are always "11" for rv32
        when opcode_store_c => -- S-immediate: store
          imm_o(31 downto 11) <= (others => execute_engine.i_reg(31)); -- sign extension
          imm_o(10 downto 05) <= execute_engine.i_reg(30 downto 25);
          imm_o(04 downto 01) <= execute_engine.i_reg(11 downto 08);
          imm_o(00)           <= execute_engine.i_reg(07);
        when opcode_branch_c => -- B-immediate: conditional branches
          imm_o(31 downto 12) <= (others => execute_engine.i_reg(31)); -- sign extension
          imm_o(11)           <= execute_engine.i_reg(07);
          imm_o(10 downto 05) <= execute_engine.i_reg(30 downto 25);
          imm_o(04 downto 01) <= execute_engine.i_reg(11 downto 08);
          imm_o(00)           <= '0';
        when opcode_lui_c | opcode_auipc_c => -- U-immediate: lui, auipc
          imm_o(31 downto 20) <= execute_engine.i_reg(31 downto 20);
          imm_o(19 downto 12) <= execute_engine.i_reg(19 downto 12);
          imm_o(11 downto 00) <= (others => '0');
        when opcode_jal_c => -- J-immediate: unconditional jumps
          imm_o(31 downto 20) <= (others => execute_engine.i_reg(31)); -- sign extension
          imm_o(19 downto 12) <= execute_engine.i_reg(19 downto 12);
          imm_o(11)           <= execute_engine.i_reg(20);
          imm_o(10 downto 05) <= execute_engine.i_reg(30 downto 25);
          imm_o(04 downto 01) <= execute_engine.i_reg(24 downto 21);
          imm_o(00)           <= '0';
        when opcode_atomic_c => -- atomic memory access and everything else
          if (CPU_EXTENSION_RISCV_A = true) then
            imm_o <= (others => '0'); -- effective address is addr = reg + 0 = reg
          else
            NULL; -- use default
          end if;
        when others => -- I-immediate
          NULL; -- use default
      end case;
    end if;
  end process imm_gen;


  -- Branch Condition Check -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  branch_check: process(execute_engine.i_reg, cmp_i)
  begin
    case execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) is
      when funct3_beq_c => -- branch if equal
        execute_engine.branch_taken <= cmp_i(cmp_equal_c);
      when funct3_bne_c => -- branch if not equal
        execute_engine.branch_taken <= not cmp_i(cmp_equal_c);
      when funct3_blt_c | funct3_bltu_c => -- branch if less (signed/unsigned)
        execute_engine.branch_taken <= cmp_i(cmp_less_c);
      when others => -- branch if greater or equal (signed/unsigned), invalid funct3 are checked by illegal instr. logic
        execute_engine.branch_taken <= not cmp_i(cmp_less_c);
    end case;
  end process branch_check;


  -- Execute Engine FSM Sync ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  execute_engine_fsm_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      -- no dedicated reset required --
      execute_engine.state_prev <= DISPATCH; -- actual reset value is not relevant
      execute_engine.i_reg      <= (others => def_rst_val_c);
      execute_engine.is_ci      <= def_rst_val_c;
      execute_engine.is_ici     <= def_rst_val_c;
      execute_engine.i_reg_last <= (others => def_rst_val_c);
      execute_engine.next_pc    <= (others => def_rst_val_c);
      ctrl                      <= (others => def_rst_val_c);
      -- registers that DO require a specific RESET state --
      execute_engine.pc         <= CPU_BOOT_ADDR(data_width_c-1 downto 2) & "00"; -- 32-bit aligned!
      execute_engine.pc_last    <= CPU_BOOT_ADDR(data_width_c-1 downto 2) & "00";
      execute_engine.state      <= DISPATCH;
      execute_engine.sleep      <= '0';
      execute_engine.branched   <= '1'; -- reset is a branch from "somewhere"
      ctrl(ctrl_bus_rd_c)       <= '0';
      ctrl(ctrl_bus_wr_c)       <= '0';
    elsif rising_edge(clk_i) then
      -- PC update --
      if (execute_engine.pc_we = '1') then
        if (execute_engine.pc_mux_sel = '0') then
          execute_engine.pc <= execute_engine.next_pc(data_width_c-1 downto 1) & '0'; -- normal (linear) increment OR trap enter/exit
        else
          execute_engine.pc <= alu_add_i(data_width_c-1 downto 1) & '0'; -- jump/taken_branch
        end if;
      end if;

      -- execute engine arbiter --
      execute_engine.state      <= execute_engine.state_nxt;
      execute_engine.state_prev <= execute_engine.state;
      execute_engine.sleep      <= execute_engine.sleep_nxt;
      execute_engine.branched   <= execute_engine.branched_nxt;
      execute_engine.i_reg      <= execute_engine.i_reg_nxt;
      execute_engine.is_ci      <= execute_engine.is_ci_nxt;
      execute_engine.is_ici     <= execute_engine.is_ici_nxt;

      -- PC & IR of "last executed" instruction for trap handling --
      if (execute_engine.state = EXECUTE) then
        execute_engine.pc_last    <= execute_engine.pc;
        execute_engine.i_reg_last <= execute_engine.i_reg;
      end if;

      -- next PC logic --
      case execute_engine.state is
        when TRAP_ENTER => -- ENTERING trap environment
          if (trap_ctrl.cause(5) = '1') and (CPU_EXTENSION_RISCV_DEBUG = true) then -- trap cause: debug mode (re-)entry
            execute_engine.next_pc <= CPU_DEBUG_ADDR; -- debug mode enter; start at "parking loop" <normal_entry>
          elsif (debug_ctrl.running = '1') and (CPU_EXTENSION_RISCV_DEBUG = true) then -- any other exception INSIDE debug mode
            execute_engine.next_pc <= std_ulogic_vector(unsigned(CPU_DEBUG_ADDR) + 4); -- start at "parking loop" <exception_entry>
          else -- normal trapping
            execute_engine.next_pc <= csr.mtvec(data_width_c-1 downto 1) & '0'; -- trap enter
          end if;
        when TRAP_EXIT => -- LEAVING trap environment
          if (CPU_EXTENSION_RISCV_DEBUG = false) or (debug_ctrl.running = '0') then -- normal end of trap
            execute_engine.next_pc <= csr.mepc(data_width_c-1 downto 1) & '0'; -- trap exit
          else -- DEBUG MODE exiting
            execute_engine.next_pc <= csr.dpc(data_width_c-1 downto 1) & '0'; -- debug mode exit
          end if;
        when EXECUTE => -- NORMAL pc increment
          execute_engine.next_pc <= std_ulogic_vector(unsigned(execute_engine.pc) + unsigned(execute_engine.next_pc_inc)); -- next linear PC
        when others =>
          NULL;
      end case;

      -- main control bus buffer --
      ctrl <= ctrl_nxt;
    end if;
  end process execute_engine_fsm_sync;


  -- PC increment for next linear instruction (+2 for compressed instr., +4 otherwise) --
  execute_engine.next_pc_inc(data_width_c-1 downto 4) <= (others => '0');
  execute_engine.next_pc_inc(3 downto 0) <= x"4" when ((execute_engine.is_ci = '0') or (CPU_EXTENSION_RISCV_C = false)) else x"2";

  -- PC output --
  curr_pc_o <= execute_engine.pc(data_width_c-1 downto 1) & '0'; -- current PC
  next_pc_o <= execute_engine.next_pc(data_width_c-1 downto 1) & '0'; -- next PC

  -- CSR access address --
  csr.addr <= execute_engine.i_reg(instr_csr_id_msb_c downto instr_csr_id_lsb_c);


  -- CPU Control Bus Output -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_output: process(ctrl, fetch_engine, trap_ctrl, execute_engine, csr, debug_ctrl)
  begin
    -- signals from execute engine --
    ctrl_o <= ctrl;
    -- prevent commits if illegal instruction --
    ctrl_o(ctrl_rf_wb_en_c) <= ctrl(ctrl_rf_wb_en_c) and (not trap_ctrl.exc_buf(exc_iillegal_c));
    ctrl_o(ctrl_bus_rd_c)   <= ctrl(ctrl_bus_rd_c)   and (not trap_ctrl.exc_buf(exc_iillegal_c));
    ctrl_o(ctrl_bus_wr_c)   <= ctrl(ctrl_bus_wr_c)   and (not trap_ctrl.exc_buf(exc_iillegal_c));
    -- current effective privilege level --
    ctrl_o(ctrl_priv_mode_c) <= csr.privilege_eff;
    -- register addresses --
    ctrl_o(ctrl_rf_rs1_adr4_c downto ctrl_rf_rs1_adr0_c) <= execute_engine.i_reg(instr_rs1_msb_c downto instr_rs1_lsb_c);
    ctrl_o(ctrl_rf_rs2_adr4_c downto ctrl_rf_rs2_adr0_c) <= execute_engine.i_reg(instr_rs2_msb_c downto instr_rs2_lsb_c);
    ctrl_o(ctrl_rf_rd_adr4_c  downto ctrl_rf_rd_adr0_c)  <= execute_engine.i_reg(instr_rd_msb_c  downto instr_rd_lsb_c);
    -- instruction fetch request --
    ctrl_o(ctrl_bus_if_c) <= fetch_engine.bus_ir;
    -- memory access size / sign --
    ctrl_o(ctrl_bus_unsigned_c) <= execute_engine.i_reg(instr_funct3_msb_c); -- unsigned LOAD (LBU, LHU)
    ctrl_o(ctrl_bus_size_msb_c downto ctrl_bus_size_lsb_c) <= execute_engine.i_reg(instr_funct3_lsb_c+1 downto instr_funct3_lsb_c); -- mem transfer size
    -- alu.shifter --
    ctrl_o(ctrl_alu_shift_dir_c) <= execute_engine.i_reg(instr_funct3_msb_c); -- shift direction (left/right)
    ctrl_o(ctrl_alu_shift_ar_c)  <= execute_engine.i_reg(30); -- is arithmetic shift
    -- instruction's function blocks (for co-processors) --
    ctrl_o(ctrl_ir_opcode7_6_c  downto ctrl_ir_opcode7_0_c) <= execute_engine.i_reg(instr_opcode_msb_c  downto instr_opcode_lsb_c);
    ctrl_o(ctrl_ir_funct12_11_c downto ctrl_ir_funct12_0_c) <= execute_engine.i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c);
    ctrl_o(ctrl_ir_funct3_2_c   downto ctrl_ir_funct3_0_c)  <= execute_engine.i_reg(instr_funct3_msb_c  downto instr_funct3_lsb_c);
    -- cpu status --
    ctrl_o(ctrl_sleep_c)         <= execute_engine.sleep; -- cpu is in sleep mode
    ctrl_o(ctrl_trap_c)          <= trap_ctrl.env_start_ack; -- cpu is starting a trap handler
    ctrl_o(ctrl_debug_running_c) <= debug_ctrl.running; -- cpu is currently in debug mode
    -- FPU rounding mode --
    ctrl_o(ctrl_alu_frm2_c downto ctrl_alu_frm0_c) <= csr.frm;
  end process ctrl_output;


  -- Decoding Helper Logic ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  decode_helper: process(execute_engine)
  begin
    -- defaults --
    decode_aux.is_a_lr  <= '0';
    decode_aux.is_a_sc  <= '0';
    decode_aux.is_f_op  <= '0';
    decode_aux.is_m_mul <= '0';
    decode_aux.is_m_div <= '0';
    decode_aux.is_b_imm <= '0';
    decode_aux.is_b_reg <= '0';
    decode_aux.rs1_zero <= '0';
    decode_aux.rd_zero  <= '0';

    -- is atomic load-reservate/store-conditional? --
    if (CPU_EXTENSION_RISCV_A = true) and (execute_engine.i_reg(instr_opcode_lsb_c+2) = '1') then -- valid atomic sub-opcode
      decode_aux.is_a_lr <= not execute_engine.i_reg(instr_funct5_lsb_c);
      decode_aux.is_a_sc <=     execute_engine.i_reg(instr_funct5_lsb_c);
    end if;

    -- is BITMANIP instruction? --
    -- pretty complex as we have to extract this from the ALU/ALUI instruction space --
    if (CPU_EXTENSION_RISCV_B = true) then -- BITMANIP implemented at all?
      -- immediate operation --
      if ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0110000") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001") and
           (
            (execute_engine.i_reg(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c) = "00000") or -- CLZ
            (execute_engine.i_reg(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c) = "00001") or -- CTZ
            (execute_engine.i_reg(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c) = "00010") or -- CPOP
            (execute_engine.i_reg(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c) = "00100") or -- SEXT.B
            (execute_engine.i_reg(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c) = "00101")    -- SEXT.H
           )
         ) or
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0110000") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "101")) or -- RORI
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0010100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "101") and (execute_engine.i_reg(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c) = "00111")) or -- ORCB
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0110100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "101")) or -- REV8
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0100100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001")) or -- BCLRI
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0100100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "101")) or -- BEXTI
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0110100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001")) or -- BINVI
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0010100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001")) then -- BSETI
        decode_aux.is_b_imm <= '1';
      end if;
      -- register operation --
      if ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0110000") and (execute_engine.i_reg(instr_funct3_msb_c-1 downto instr_funct3_lsb_c) = "01")) or -- ROR / ROL
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000101") and (execute_engine.i_reg(instr_funct3_msb_c) = '1')) or -- MIN[U] / MAX[U]
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "100")) or -- ZEXTH
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0100100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001")) or -- BCLR
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0100100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "101")) or -- BEXT
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0110100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001")) or -- BINV
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0010100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001")) or -- BSET
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000101") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001")) or -- CLMUL
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000101") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "011")) or -- CLMULH
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000101") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "010")) or -- CLMULR
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0100000") and
          (
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "111") or -- ANDN
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "110") or -- ORN
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "100")    -- XORN
          )
         ) or
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0010000") and
          (
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "010") or -- SH1ADD
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "100") or -- SH2ADD
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "110")    -- SH3ADD
          )
         ) then
        decode_aux.is_b_reg <= '1';
      end if;
    end if;

    -- floating-point operations (Zfinx) --
    if (CPU_EXTENSION_RISCV_Zfinx = true) then -- FPU implemented at all?
      if ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+3) = "0000")) or -- FADD.S / FSUB.S
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "00010")) or -- FMUL.S
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "11100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001")) or -- FCLASS.S
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "00100") and (execute_engine.i_reg(instr_funct3_msb_c) = '0')) or -- FSGNJ[N/X].S
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "00101") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_msb_c-1) = "00")) or -- FMIN.S / FMAX.S
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "10100") and (execute_engine.i_reg(instr_funct3_msb_c) = '0')) or -- FEQ.S / FLT.S / FLE.S
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "11010") and (execute_engine.i_reg(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c+1) = "0000")) or -- FCVT.S.W*
         ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "11000") and (execute_engine.i_reg(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c+1) = "0000")) then -- FCVT.W*.S
        decode_aux.is_f_op <= '1';
      end if;
    end if;

    -- integer MUL (M/Zmmul) / DIV (M) operation --
    if (execute_engine.i_reg(instr_opcode_lsb_c+5) = opcode_alu_c(5)) and (execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000001") then
      if ((CPU_EXTENSION_RISCV_M = true) or (CPU_EXTENSION_RISCV_Zmmul = true)) and (execute_engine.i_reg(instr_funct3_msb_c) = '0') then
        decode_aux.is_m_mul <= '1';
      end if;
      if (CPU_EXTENSION_RISCV_M = true) and (execute_engine.i_reg(instr_funct3_msb_c) = '1') then
        decode_aux.is_m_div <= '1';
      end if;
    end if;

    -- register/uimm5 checks --
    decode_aux.rs1_zero <= not or_reduce_f(execute_engine.i_reg(instr_rs1_msb_c downto instr_rs1_lsb_c));
    decode_aux.rd_zero  <= not or_reduce_f(execute_engine.i_reg(instr_rd_msb_c  downto instr_rd_lsb_c));
  end process decode_helper;


  -- Execute Engine FSM Comb ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  execute_engine_fsm_comb: process(execute_engine, debug_ctrl, trap_ctrl, decode_aux, fetch_engine, cmd_issue,
                                   csr, ctrl, alu_idone_i, bus_d_wait_i, excl_state_i)
    variable opcode_v : std_ulogic_vector(6 downto 0);
  begin
    -- arbiter defaults --
    execute_engine.state_nxt    <= execute_engine.state;
    execute_engine.i_reg_nxt    <= execute_engine.i_reg;
    execute_engine.is_ci_nxt    <= execute_engine.is_ci;
    execute_engine.is_ici_nxt   <= '0';
    execute_engine.sleep_nxt    <= execute_engine.sleep;
    execute_engine.branched_nxt <= execute_engine.branched;
    --
    execute_engine.pc_mux_sel   <= '0';
    execute_engine.pc_we        <= '0';

    -- instruction dispatch --
    fetch_engine.reset          <= '0';

    -- trap environment control --
    trap_ctrl.env_start_ack     <= '0';
    trap_ctrl.env_end           <= '0';

    -- leave debug mode --
    debug_ctrl.dret             <= '0';

    -- exception trigger --
    trap_ctrl.instr_be          <= '0';
    trap_ctrl.instr_ma          <= '0';
    trap_ctrl.env_call          <= '0';
    trap_ctrl.break_point       <= '0';

    -- CSR access --
    csr.we_nxt                  <= '0';

    -- CONTROL DEFAULTS --
    ctrl_nxt <= (others => '0'); -- default: all off
    ctrl_nxt(ctrl_alu_op2_c downto ctrl_alu_op0_c) <= alu_op_add_c; -- default ALU operation: ADD
    ctrl_nxt(ctrl_rf_mux1_c downto ctrl_rf_mux0_c) <= rf_mux_alu_c; -- default RF input: ALU
    -- ALU sign control --
    if (execute_engine.i_reg(instr_opcode_lsb_c+4) = '1') then -- ALU ops
      ctrl_nxt(ctrl_alu_unsigned_c) <= execute_engine.i_reg(instr_funct3_lsb_c+0); -- unsigned ALU operation? (SLTIU, SLTU)
    else -- branches
      ctrl_nxt(ctrl_alu_unsigned_c) <= execute_engine.i_reg(instr_funct3_lsb_c+1); -- unsigned branches? (BLTU, BGEU)
    end if;
    -- atomic store-conditional instruction (evaluate lock status) --
    ctrl_nxt(ctrl_bus_ch_lock_c) <= decode_aux.is_a_sc;


    -- state machine --
    case execute_engine.state is

      when DISPATCH => -- Get new command from instruction issue engine
      -- ------------------------------------------------------------
        -- PC update --
        execute_engine.pc_mux_sel <= '0'; -- linear next PC
        -- IR update --
        execute_engine.is_ci_nxt <= cmd_issue.data(32); -- flag to indicate a de-compressed instruction
        execute_engine.i_reg_nxt <= cmd_issue.data(31 downto 0);
        --
        if (cmd_issue.valid = '1') then -- instruction available?
          -- PC update --
          execute_engine.branched_nxt <= '0';
          execute_engine.pc_we        <= not execute_engine.branched; -- update PC with linear next_pc if there was no actual branch
          -- IR update - exceptions --
          if (CPU_EXTENSION_RISCV_C = false) then
            trap_ctrl.instr_ma <= cmd_issue.data(33); -- misaligned instruction fetch address, if C disabled
          end if;
          trap_ctrl.instr_be        <= cmd_issue.data(34); -- bus access fault during instruction fetch
          execute_engine.is_ici_nxt <= cmd_issue.data(35); -- invalid decompressed instruction
          -- any reason to go to trap state? --
          if (execute_engine.sleep = '1') or -- enter sleep state
             (trap_ctrl.exc_fire = '1') or -- exception during LAST instruction (e.g. illegal instruction)
             (trap_ctrl.env_start = '1') or -- pending trap (IRQ or exception)
             ((cmd_issue.data(33) = '1') and (CPU_EXTENSION_RISCV_C = false)) or -- misaligned instruction fetch address (if C disabled)
             (cmd_issue.data(34) = '1') then -- bus access fault during instruction fetch
            execute_engine.state_nxt <= TRAP_ENTER;
          else
            execute_engine.state_nxt <= EXECUTE;
          end if;
        end if;


      when TRAP_ENTER => -- Start trap environment - get trap vector (depc or epc), stay here for sleep mode
      -- ------------------------------------------------------------
        if (trap_ctrl.env_start = '1') then -- trap triggered?
          trap_ctrl.env_start_ack  <= '1';
          execute_engine.state_nxt <= TRAP_EXECUTE;
        end if;


      when TRAP_EXIT => -- Return from trap environment - get xEPC
      -- ------------------------------------------------------------
        trap_ctrl.env_end        <= '1';
        execute_engine.state_nxt <= TRAP_EXECUTE;


      when TRAP_EXECUTE => -- Process trap environment -> jump to xTVEC / return from trap environment -> jump to xEPC
      -- ------------------------------------------------------------
        execute_engine.pc_mux_sel <= '0'; -- next_PC
        fetch_engine.reset        <= '1';
        execute_engine.pc_we      <= '1';
        execute_engine.sleep_nxt  <= '0'; -- disable sleep mode
        execute_engine.state_nxt  <= DISPATCH;


      when EXECUTE => -- Decode and execute instruction (control has to be here for exactly 1 cycle in any case!)
      -- ------------------------------------------------------------
        opcode_v := execute_engine.i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c+2) & "11"; -- save some bits here, LSBs are always 11 for rv32
        case opcode_v is

          when opcode_alu_c | opcode_alui_c => -- (register/immediate) ALU operation
          -- ------------------------------------------------------------
            ctrl_nxt(ctrl_alu_opb_mux_c) <= not execute_engine.i_reg(instr_opcode_msb_c-1); -- use IMM as ALU.OPB for immediate operations

            -- ALU core operation --
            case execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) is -- actual ALU.logic operation (re-coding)
              when funct3_subadd_c => -- ADD(I)/SUB
                if ((execute_engine.i_reg(instr_opcode_msb_c-1) = '1') and (execute_engine.i_reg(instr_funct7_msb_c-1) = '1')) then -- not an immediate op and funct7.6 set => SUB
                  ctrl_nxt(ctrl_alu_op2_c downto ctrl_alu_op0_c) <= alu_op_sub_c;
                else
                  ctrl_nxt(ctrl_alu_op2_c downto ctrl_alu_op0_c) <= alu_op_add_c;
                end if;
              when funct3_slt_c | funct3_sltu_c => -- SLT(I), SLTU(I)
                ctrl_nxt(ctrl_alu_op2_c downto ctrl_alu_op0_c) <= alu_op_slt_c;
              when funct3_xor_c => -- XOR(I)
                ctrl_nxt(ctrl_alu_op2_c downto ctrl_alu_op0_c) <= alu_op_xor_c;
              when funct3_or_c => -- OR(I)
                ctrl_nxt(ctrl_alu_op2_c downto ctrl_alu_op0_c) <= alu_op_or_c;
              when others => -- AND(I), multi-cycle / co-processor operations
                ctrl_nxt(ctrl_alu_op2_c downto ctrl_alu_op0_c) <= alu_op_and_c;
            end case;

            -- co-processor MULDIV operation (multi-cycle) --
            if ((CPU_EXTENSION_RISCV_M = true) and ((decode_aux.is_m_mul = '1') or (decode_aux.is_m_div = '1'))) or -- MUL/DIV
               ((CPU_EXTENSION_RISCV_Zmmul = true) and (decode_aux.is_m_mul = '1')) then -- MUL
              ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_muldiv_c; -- trigger MULDIV CP
              execute_engine.state_nxt <= ALU_WAIT;
            -- co-processor BIT-MANIPULATION operation (multi-cycle) --
            elsif (CPU_EXTENSION_RISCV_B = true) and
                  (((execute_engine.i_reg(instr_opcode_lsb_c+5) = opcode_alu_c(5))  and (decode_aux.is_b_reg = '1')) or -- register operation
                   ((execute_engine.i_reg(instr_opcode_lsb_c+5) = opcode_alui_c(5)) and (decode_aux.is_b_imm = '1'))) then -- immediate operation
              ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_bitmanip_c; -- trigger BITMANIP CP
              execute_engine.state_nxt <= ALU_WAIT;
            -- co-processor SHIFT operation (multi-cycle) --
            elsif (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sll_c) or
                  (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sr_c) then
              ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_shifter_c; -- trigger SHIFTER CP
              execute_engine.state_nxt <= ALU_WAIT;
            -- ALU CORE operation (single-cycle) --
            else
              ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back
              execute_engine.state_nxt  <= DISPATCH;
            end if;


          when opcode_lui_c | opcode_auipc_c => -- load upper immediate / add upper immediate to PC
          -- ------------------------------------------------------------
            ctrl_nxt(ctrl_alu_opa_mux_c) <= '1'; -- ALU.OPA = PC (for AUIPC only)
            ctrl_nxt(ctrl_alu_opb_mux_c) <= '1'; -- use IMM as ALU.OPB
            if (execute_engine.i_reg(instr_opcode_lsb_c+5) = opcode_lui_c(5)) then -- LUI
              ctrl_nxt(ctrl_alu_op2_c downto ctrl_alu_op0_c) <= alu_op_movb_c; -- actual ALU operation = MOVB
            else -- AUIPC
              ctrl_nxt(ctrl_alu_op2_c downto ctrl_alu_op0_c) <= alu_op_add_c; -- actual ALU operation = ADD
            end if;
            ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back
            execute_engine.state_nxt  <= DISPATCH;


          when opcode_load_c | opcode_store_c | opcode_atomic_c => -- load/store / atomic memory access
          -- ------------------------------------------------------------
            ctrl_nxt(ctrl_alu_opb_mux_c) <= '1'; -- use IMM as ALU.OPB
            ctrl_nxt(ctrl_bus_mo_we_c)   <= '1'; -- write to MAR and MDO (MDO only relevant for store)
            execute_engine.state_nxt     <= LOADSTORE_0;


          when opcode_branch_c | opcode_jal_c | opcode_jalr_c => -- branch / jump and link (with register)
          -- ------------------------------------------------------------
            if (execute_engine.i_reg(instr_opcode_lsb_c+3 downto instr_opcode_lsb_c+2) = opcode_jalr_c(3 downto 2)) then -- JALR
              ctrl_nxt(ctrl_alu_opa_mux_c) <= '0'; -- use RS1 as ALU.OPA (branch target address base)
            else -- JAL
              ctrl_nxt(ctrl_alu_opa_mux_c) <= '1'; -- use PC as ALU.OPA (branch target address base)
            end if;
            ctrl_nxt(ctrl_alu_opb_mux_c) <= '1'; -- use IMM as ALU.OPB (branch target address offset)
            execute_engine.state_nxt     <= BRANCH;


          when opcode_fence_c => -- fence operations
          -- ------------------------------------------------------------
            if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_fence_c) then -- FENCE
              ctrl_nxt(ctrl_bus_fence_c)  <= '1';
              execute_engine.state_nxt    <= DISPATCH;
            elsif (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_fencei_c) and (CPU_EXTENSION_RISCV_Zifencei = true) then -- FENCE.I
              ctrl_nxt(ctrl_bus_fencei_c) <= '1';
              execute_engine.branched_nxt <= '1'; -- this is an actual branch
              execute_engine.state_nxt    <= TRAP_EXECUTE; -- use TRAP_EXECUTE to "modify" PC (PC <= PC)
            else -- illegal fence instruction
              execute_engine.state_nxt    <= DISPATCH;
            end if;


          when opcode_fop_c => -- floating-point operations
          -- ------------------------------------------------------------
            if (CPU_EXTENSION_RISCV_Zfinx = true) then
              ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_fpu_c; -- trigger FPU CP
              execute_engine.state_nxt <= ALU_WAIT;
            else
              execute_engine.state_nxt <= DISPATCH;
            end if;


          when opcode_cust0_c => -- CFU: custom RISC-V instructions (CUSTOM0 OPCODE space)
          -- ------------------------------------------------------------
            if (CPU_EXTENSION_RISCV_Zxcfu = true) then
              ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_cfu_c; -- trigger CFU CP
              execute_engine.state_nxt <= ALU_WAIT;
            else
              execute_engine.state_nxt <= DISPATCH;
            end if;


          when others => -- system/csr access OR illegal opcode
          -- ------------------------------------------------------------
            if (CPU_EXTENSION_RISCV_Zicsr = true) then
              if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_env_c) then -- system/environment
                execute_engine.state_nxt <= SYS_ENV;
              else -- CSR access
                execute_engine.state_nxt <= CSR_ACCESS;
              end if;
            else
              execute_engine.state_nxt <= DISPATCH;
            end if;

        end case;


      when SYS_ENV => -- system environment operation
      -- ------------------------------------------------------------
        -- MRET / DRET --
        if (execute_engine.i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = funct12_mret_c) then
          execute_engine.state_nxt <= TRAP_EXIT; -- mret
        elsif (execute_engine.i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = funct12_dret_c) and
              (CPU_EXTENSION_RISCV_DEBUG = true) then
          debug_ctrl.dret          <= '1';
          execute_engine.state_nxt <= TRAP_EXIT; -- dret
        else
          execute_engine.state_nxt <= DISPATCH; -- default
        end if;
        -- ECALL / EBREAK --
        if ((execute_engine.i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c+1) = funct12_ecall_c(11 downto 1))) then
          if (execute_engine.i_reg(instr_funct12_lsb_c) = funct12_ecall_c(0)) then
            trap_ctrl.env_call <= '1'; -- ecall
          else
            trap_ctrl.break_point <= '1'; -- ebreak
          end if;
        end if;
        -- WFI --
        if (execute_engine.i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = funct12_wfi_c) and
           ((CPU_EXTENSION_RISCV_DEBUG = false) or ((debug_ctrl.running = '0') and (csr.dcsr_step = '0'))) then
          execute_engine.sleep_nxt <= '1'; -- not executed (NOP) when in debug-mode or during single-stepping
        end if;


      when CSR_ACCESS => -- read & write status and control register (CSR) - no read/write if illegal instruction
      -- ------------------------------------------------------------
        -- CSR write access [invalid CSR instructions are already checked by the illegal instruction logic] --
        if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrw_c) or
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrwi_c) or -- CSRRW(I); always write CSR
           (decode_aux.rs1_zero = '0') then -- CSRRS(I) / CSRRC(I): write CSR if rs1/imm5 is NOT zero
          csr.we_nxt <= '1';
        end if;
        -- register file write back --
        ctrl_nxt(ctrl_rf_mux1_c downto ctrl_rf_mux0_c) <= rf_mux_csr_c;
        ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back
        execute_engine.state_nxt  <= DISPATCH;


      when ALU_WAIT => -- wait for multi-cycle ALU operation (ALU co-processor) to finish
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_alu_op2_c downto ctrl_alu_op0_c) <= alu_op_cp_c;
        -- wait for completion or abort on illegal instruction exception (the co-processor will also terminate operations)
        if (alu_idone_i = '1') or (trap_ctrl.exc_buf(exc_iillegal_c) = '1') then
          ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back (won't happen in case of an illegal instruction)
          execute_engine.state_nxt  <= DISPATCH;
        end if;


      when BRANCH => -- update PC for taken branches and jumps
      -- ------------------------------------------------------------
        -- get and store return address (only relevant for jump-and-link operations) --
        ctrl_nxt(ctrl_rf_mux1_c downto ctrl_rf_mux0_c) <= rf_mux_npc_c; -- next PC
        ctrl_nxt(ctrl_rf_wb_en_c) <= execute_engine.i_reg(instr_opcode_lsb_c+2); -- valid RF write-back? (is jump-and-link?)
        -- destination address --
        execute_engine.pc_mux_sel <= '1'; -- PC <= alu.add = branch/jump destination
        if (execute_engine.i_reg(instr_opcode_lsb_c+2) = '1') or (execute_engine.branch_taken = '1') then -- JAL/JALR or taken branch
          -- no need to check for illegal instructions here; the branch condition evaluation circuit will not set "branch_taken" if funct3 is invalid
          execute_engine.pc_we        <= '1'; -- update PC
          execute_engine.branched_nxt <= '1'; -- this is an actual branch
          fetch_engine.reset          <= '1'; -- trigger new instruction fetch from modified PC
        end if;
        execute_engine.state_nxt <= DISPATCH;


      when LOADSTORE_0 => -- trigger memory request
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_bus_lock_c) <= decode_aux.is_a_lr; -- atomic.LR: set lock
        if (execute_engine.i_reg(instr_opcode_msb_c-1) = '0') or (decode_aux.is_a_lr = '1') then -- normal load or atomic load-reservate
          ctrl_nxt(ctrl_bus_rd_c) <= '1'; -- read request
        else -- store
          if (decode_aux.is_a_sc = '0') or (CPU_EXTENSION_RISCV_A = false) then -- (normal) write request
            ctrl_nxt(ctrl_bus_wr_c) <= '1';
          else -- evaluate lock state
            ctrl_nxt(ctrl_bus_wr_c) <= excl_state_i; -- write request if lock is still ok
          end if;
        end if;
        execute_engine.state_nxt <= LOADSTORE_1;


      when LOADSTORE_1 => -- memory access latency
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_bus_mi_we_c) <= '1'; -- write input data to MDI (only relevant for LOADs)
        execute_engine.state_nxt   <= LOADSTORE_2;


      when LOADSTORE_2 => -- wait for bus transaction to finish
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_bus_mi_we_c) <= '1'; -- keep writing input data to MDI (only relevant for load (and SC.W) operations)
        ctrl_nxt(ctrl_rf_mux1_c downto ctrl_rf_mux0_c) <= rf_mux_mem_c; -- memory read data
        -- wait for memory response --
        if (trap_ctrl.env_start = '1') and (trap_ctrl.cause(6 downto 5) = "00") then -- abort if SYNC EXCEPTION (from bus or illegal cmd) / no IRQs and NOT DEBUG-MODE-related
          execute_engine.state_nxt <= DISPATCH;
        elsif (bus_d_wait_i = '0') then -- wait for bus to finish transaction
          -- data write-back --
          if (execute_engine.i_reg(instr_opcode_msb_c-1) = '0') or -- normal load
             (decode_aux.is_a_lr = '1') or -- atomic load-reservate
             (decode_aux.is_a_sc = '1') then -- atomic store-conditional
            ctrl_nxt(ctrl_rf_wb_en_c) <= '1';
          end if;
          -- remove atomic lock if this is NOT the LR.W instruction used to SET the lock --
          if (decode_aux.is_a_lr = '0') then -- execute and evaluate atomic store-conditional
            ctrl_nxt(ctrl_bus_de_lock_c) <= '1';
          end if;
          execute_engine.state_nxt <= DISPATCH;
        end if;


      when others => -- undefined
      -- ------------------------------------------------------------
        execute_engine.state_nxt <= DISPATCH;

    end case;
  end process execute_engine_fsm_comb;


-- ****************************************************************************************************************************
-- Illegal Instruction and CSR Access Check
-- ****************************************************************************************************************************

  -- CSR Access Check -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_access_check: process(execute_engine.i_reg, decode_aux, csr, debug_ctrl)
    variable csr_wacc_v : std_ulogic; -- actual CSR write
  begin
    -- is this CSR instruction really going to write to a CSR? --
    if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrw_c) or -- always write CSR
       (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrwi_c) or -- always write CSR
       (decode_aux.rs1_zero = '0') then -- clear/set: write CSR if rs1/imm5 is NOT zero
      csr_wacc_v := '1';
    else
      csr_wacc_v := '0';
    end if;

    -- check CSR access --
    csr_acc_valid <= '0'; -- default: invalid access
    case csr.addr is

      -- floating-point CSRs --
      when csr_fflags_c | csr_frm_c | csr_fcsr_c =>
        csr_acc_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zfinx); -- full access for everyone if FPU implemented

      -- machine trap setup/handling & counters --
      when csr_mstatus_c | csr_mstatush_c | csr_misa_c | csr_mie_c | csr_mtvec_c | csr_mscratch_c | csr_mepc_c | csr_mcause_c | csr_mip_c | csr_mtval_c |
           csr_mcycle_c | csr_mcycleh_c | csr_minstret_c | csr_minstreth_c | csr_mcountinhibit_c =>
        -- NOTE: MISA and MTVAL are read-only in the NEORV32 but we do not cause an exception here for compatibility.
        -- Machine-level code should read-back those CSRs after writing them to realize they are read-only.
        csr_acc_valid <= csr.privilege_eff; -- M-mode only 

      -- machine information registers & NEORV32-specific registers, read-only --
      when csr_mvendorid_c | csr_marchid_c | csr_mimpid_c | csr_mhartid_c | csr_mconfigptr_c | csr_mxisa_c =>
        csr_acc_valid <= (not csr_wacc_v) and csr.privilege_eff; -- M-mode only, read-only

      -- user-mode registers --
      when csr_mcounteren_c | csr_menvcfg_c | csr_menvcfgh_c =>
        csr_acc_valid <= csr.privilege_eff and bool_to_ulogic_f(CPU_EXTENSION_RISCV_U);

      -- physical memory protection (PMP) --
      when csr_pmpaddr0_c  | csr_pmpaddr1_c  | csr_pmpaddr2_c  | csr_pmpaddr3_c  | csr_pmpaddr4_c  | csr_pmpaddr5_c  | csr_pmpaddr6_c  | csr_pmpaddr7_c  | -- address
           csr_pmpaddr8_c  | csr_pmpaddr9_c  | csr_pmpaddr10_c | csr_pmpaddr11_c | csr_pmpaddr12_c | csr_pmpaddr13_c | csr_pmpaddr14_c | csr_pmpaddr15_c |
           csr_pmpcfg0_c   | csr_pmpcfg1_c   | csr_pmpcfg2_c   | csr_pmpcfg3_c => -- configuration
        csr_acc_valid <= csr.privilege_eff and bool_to_ulogic_f(boolean(PMP_NUM_REGIONS > 0)); -- M-mode only

      -- machine hardware performance monitors (MHPM) --
      when csr_mhpmcounter3_c   | csr_mhpmcounter4_c   | csr_mhpmcounter5_c   | csr_mhpmcounter6_c   | csr_mhpmcounter7_c   | csr_mhpmcounter8_c   | -- counter LOW
           csr_mhpmcounter9_c   | csr_mhpmcounter10_c  | csr_mhpmcounter11_c  | csr_mhpmcounter12_c  | csr_mhpmcounter13_c  | csr_mhpmcounter14_c  |
           csr_mhpmcounter15_c  | csr_mhpmcounter16_c  | csr_mhpmcounter17_c  | csr_mhpmcounter18_c  | csr_mhpmcounter19_c  | csr_mhpmcounter20_c  |
           csr_mhpmcounter21_c  | csr_mhpmcounter22_c  | csr_mhpmcounter23_c  | csr_mhpmcounter24_c  | csr_mhpmcounter25_c  | csr_mhpmcounter26_c  |
           csr_mhpmcounter27_c  | csr_mhpmcounter28_c  | csr_mhpmcounter29_c  | csr_mhpmcounter30_c  | csr_mhpmcounter31_c  |
           csr_mhpmcounter3h_c  | csr_mhpmcounter4h_c  | csr_mhpmcounter5h_c  | csr_mhpmcounter6h_c  | csr_mhpmcounter7h_c  | csr_mhpmcounter8h_c  | -- counter HIGH
           csr_mhpmcounter9h_c  | csr_mhpmcounter10h_c | csr_mhpmcounter11h_c | csr_mhpmcounter12h_c | csr_mhpmcounter13h_c | csr_mhpmcounter14h_c |
           csr_mhpmcounter15h_c | csr_mhpmcounter16h_c | csr_mhpmcounter17h_c | csr_mhpmcounter18h_c | csr_mhpmcounter19h_c | csr_mhpmcounter20h_c |
           csr_mhpmcounter21h_c | csr_mhpmcounter22h_c | csr_mhpmcounter23h_c | csr_mhpmcounter24h_c | csr_mhpmcounter25h_c | csr_mhpmcounter26h_c |
           csr_mhpmcounter27h_c | csr_mhpmcounter28h_c | csr_mhpmcounter29h_c | csr_mhpmcounter30h_c | csr_mhpmcounter31h_c |
           csr_mhpmevent3_c     | csr_mhpmevent4_c     | csr_mhpmevent5_c     | csr_mhpmevent6_c     | csr_mhpmevent7_c     | csr_mhpmevent8_c     | -- event configuration
           csr_mhpmevent9_c     | csr_mhpmevent10_c    | csr_mhpmevent11_c    | csr_mhpmevent12_c    | csr_mhpmevent13_c    | csr_mhpmevent14_c    |
           csr_mhpmevent15_c    | csr_mhpmevent16_c    | csr_mhpmevent17_c    | csr_mhpmevent18_c    | csr_mhpmevent19_c    | csr_mhpmevent20_c    |
           csr_mhpmevent21_c    | csr_mhpmevent22_c    | csr_mhpmevent23_c    | csr_mhpmevent24_c    | csr_mhpmevent25_c    | csr_mhpmevent26_c    |
           csr_mhpmevent27_c    | csr_mhpmevent28_c    | csr_mhpmevent29_c    | csr_mhpmevent30_c    | csr_mhpmevent31_c =>
        csr_acc_valid <= csr.privilege_eff and bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zihpm); -- M-mode only

--    -- user hardware performance monitors (HPM) --
--    when csr_hpmcounter3_c   | csr_hpmcounter4_c   | csr_hpmcounter5_c   | csr_hpmcounter6_c   | csr_hpmcounter7_c   | csr_hpmcounter8_c   | -- counter LOW
--         csr_hpmcounter9_c   | csr_hpmcounter10_c  | csr_hpmcounter11_c  | csr_hpmcounter12_c  | csr_hpmcounter13_c  | csr_hpmcounter14_c  |
--         csr_hpmcounter15_c  | csr_hpmcounter16_c  | csr_hpmcounter17_c  | csr_hpmcounter18_c  | csr_hpmcounter19_c  | csr_hpmcounter20_c  |
--         csr_hpmcounter21_c  | csr_hpmcounter22_c  | csr_hpmcounter23_c  | csr_hpmcounter24_c  | csr_hpmcounter25_c  | csr_hpmcounter26_c  |
--         csr_hpmcounter27_c  | csr_hpmcounter28_c  | csr_hpmcounter29_c  | csr_hpmcounter30_c  | csr_hpmcounter31_c  |
--         csr_hpmcounter3h_c  | csr_hpmcounter4h_c  | csr_hpmcounter5h_c  | csr_hpmcounter6h_c  | csr_hpmcounter7h_c  | csr_hpmcounter8h_c  | -- counter HIGH
--         csr_hpmcounter9h_c  | csr_hpmcounter10h_c | csr_hpmcounter11h_c | csr_hpmcounter12h_c | csr_hpmcounter13h_c | csr_hpmcounter14h_c |
--         csr_hpmcounter15h_c | csr_hpmcounter16h_c | csr_hpmcounter17h_c | csr_hpmcounter18h_c | csr_hpmcounter19h_c | csr_hpmcounter20h_c |
--         csr_hpmcounter21h_c | csr_hpmcounter22h_c | csr_hpmcounter23h_c | csr_hpmcounter24h_c | csr_hpmcounter25h_c | csr_hpmcounter26h_c |
--         csr_hpmcounter27h_c | csr_hpmcounter28h_c | csr_hpmcounter29h_c | csr_hpmcounter30h_c | csr_hpmcounter31h_c =>
--      csr_acc_valid <= '0'; -- >>> NOT IMPLEMENTED <<<

      -- user-level counters/timers (read-only) --
      when csr_cycle_c | csr_cycleh_c | csr_time_c | csr_timeh_c | csr_instret_c | csr_instreth_c =>
        case csr.addr(1 downto 0) is
          when "00"   => csr_acc_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicntr) and (not csr_wacc_v) and (csr.privilege_eff or csr.mcounteren_cy); -- cyle[h]: M-mode, U-mode if authorized, implemented at all, read-only
          when "01"   => csr_acc_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicntr) and (not csr_wacc_v) and (csr.privilege_eff or csr.mcounteren_tm); -- time[h]: M-mode, U-mode if authorized, implemented at all, read-only
          when "10"   => csr_acc_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicntr) and (not csr_wacc_v) and (csr.privilege_eff or csr.mcounteren_ir); -- instret[h]: M-mode, U-mode if authorized, implemented at all read-only
          when others => csr_acc_valid <= '0';
        end case;

      -- debug mode CSRs --
      when csr_dcsr_c | csr_dpc_c | csr_dscratch0_c =>
        csr_acc_valid <= debug_ctrl.running and bool_to_ulogic_f(CPU_EXTENSION_RISCV_DEBUG); -- access only in debug-mode

      -- trigger module CSRs --
      when csr_tselect_c | csr_tdata1_c | csr_tdata2_c | csr_tdata3_c | csr_tinfo_c | csr_tcontrol_c | csr_mcontext_c | csr_scontext_c =>
        -- access in debug-mode or M-mode (M-mode: writes to tdata* are ignored as DMODE is hardwired to 1)
        csr_acc_valid <= (debug_ctrl.running or csr.privilege_eff) and bool_to_ulogic_f(CPU_EXTENSION_RISCV_DEBUG);

      -- undefined / not implemented --
      when others =>
        csr_acc_valid <= '0'; -- invalid access
    end case;
  end process csr_access_check;


  -- Illegal Instruction Check --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  illegal_instruction_check: process(execute_engine, decode_aux, csr, csr_acc_valid, debug_ctrl)
    variable opcode_v : std_ulogic_vector(6 downto 0);
  begin
    -- defaults --
    illegal_instruction <= '0';
    illegal_register    <= '0';

    -- check opcode for rv32 --
    if (execute_engine.i_reg(instr_opcode_lsb_c+1 downto instr_opcode_lsb_c) = "11") then
      illegal_opcode_lsbs <= '0';
    else
      illegal_opcode_lsbs <= '1';
    end if;

    -- check for illegal compressed instruction --
    if (CPU_EXTENSION_RISCV_C = true) then
      illegal_compressed <= execute_engine.is_ici;
    else
      illegal_compressed <= '0';
    end if;

    -- check instructions --
    opcode_v := execute_engine.i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c+2) & "11"; -- save some bits here, LSBs are always 11 for rv32
    case opcode_v is

      when opcode_lui_c | opcode_auipc_c | opcode_jal_c => -- check sufficient LUI, UIPC, JAL (only check actual OPCODE)
      -- ------------------------------------------------------------
        illegal_instruction <= '0';
        -- illegal E-CPU register? --
        illegal_register <= execute_engine.i_reg(instr_rd_msb_c);

      when opcode_alu_c => -- check ALU.funct3 & ALU.funct7
      -- ------------------------------------------------------------
        if (((execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_subadd_c) or (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sr_c)) and
            (execute_engine.i_reg(instr_funct7_msb_c-2 downto instr_funct7_lsb_c) = "00000") and (execute_engine.i_reg(instr_funct7_msb_c) = '0')) or
           (((execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sll_c) or 
             (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_slt_c) or 
             (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sltu_c) or 
             (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_xor_c) or 
             (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_or_c) or 
             (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_and_c)) and
             (execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000000")) then -- valid base ALUI instruction?
          illegal_instruction <= '0';
        elsif ((CPU_EXTENSION_RISCV_M = true) or (CPU_EXTENSION_RISCV_Zmmul = false)) and (decode_aux.is_m_mul = '1') then -- valid MUL instruction?
          illegal_instruction <= '0';
        elsif (CPU_EXTENSION_RISCV_M = true) and (decode_aux.is_m_div = '1') then -- valid DIV instruction?
          illegal_instruction <= '0';
        elsif (CPU_EXTENSION_RISCV_B = true) and (decode_aux.is_b_reg = '1') then -- valid BITMANIP instruction?
          illegal_instruction <= '0';
        else
          illegal_instruction <= '1';
        end if;
        -- illegal E-CPU register? --
        illegal_register <= execute_engine.i_reg(instr_rd_msb_c) or execute_engine.i_reg(instr_rs1_msb_c) or execute_engine.i_reg(instr_rs2_msb_c);

      when opcode_alui_c => -- check ALUI.funct7
      -- ------------------------------------------------------------
        if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_subadd_c) or
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_slt_c) or
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sltu_c) or
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_xor_c) or
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_or_c) or
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_and_c) or
           ((execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sll_c) and -- shift logical left
            (execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000000")) or
           ((execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sr_c) and -- shift right
            ((execute_engine.i_reg(instr_funct7_msb_c-2 downto instr_funct7_lsb_c) = "00000") and (execute_engine.i_reg(instr_funct7_msb_c) = '0'))) then -- valid base ALUI instruction?
          illegal_instruction <= '0';
        elsif (CPU_EXTENSION_RISCV_B = true) and (decode_aux.is_b_imm = '1') then -- valid BITMANIP immediate instruction?
          illegal_instruction <= '0';
        else
          illegal_instruction <= '1';
        end if;
        -- illegal E-CPU register? --
        illegal_register <= execute_engine.i_reg(instr_rs1_msb_c) or execute_engine.i_reg(instr_rd_msb_c);

      when opcode_load_c => -- check LOAD.funct3
      -- ------------------------------------------------------------
        if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_lb_c) or
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_lh_c) or
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_lw_c) or
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_lbu_c) or
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_lhu_c) then
          illegal_instruction <= '0';
        else
          illegal_instruction <= '1';
        end if;
        -- illegal E-CPU register? --
        illegal_register <= execute_engine.i_reg(instr_rs1_msb_c) or execute_engine.i_reg(instr_rd_msb_c);

      when opcode_store_c => -- check STORE.funct3
      -- ------------------------------------------------------------
        if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sb_c) or
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sh_c) or
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sw_c) then
          illegal_instruction <= '0';
        else
          illegal_instruction <= '1';
        end if;
        -- illegal E-CPU register? --
        illegal_register <= execute_engine.i_reg(instr_rs2_msb_c) or execute_engine.i_reg(instr_rs1_msb_c);

      when opcode_atomic_c => -- atomic instructions
      -- ------------------------------------------------------------
        if (CPU_EXTENSION_RISCV_A = true) then
          if (execute_engine.i_reg(instr_funct5_msb_c downto instr_funct5_lsb_c) = "00010") then -- LR
            illegal_instruction <= '0';
            -- illegal E-CPU register? --
            illegal_register <= execute_engine.i_reg(instr_rs1_msb_c) or execute_engine.i_reg(instr_rd_msb_c);
          elsif (execute_engine.i_reg(instr_funct5_msb_c downto instr_funct5_lsb_c) = "00011") then -- SC
            illegal_instruction <= '0';
            -- illegal E-CPU register? --
            illegal_register <= execute_engine.i_reg(instr_rs1_msb_c) or execute_engine.i_reg(instr_rs2_msb_c) or execute_engine.i_reg(instr_rd_msb_c);
          else
            illegal_instruction <= '1';
          end if;
        else
          illegal_instruction <= '1';
        end if;

      when opcode_branch_c => -- check BRANCH.funct3
      -- ------------------------------------------------------------
        if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_beq_c) or
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_bne_c) or
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_blt_c) or
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_bge_c) or
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_bltu_c) or
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_bgeu_c) then
          illegal_instruction <= '0';
        else
          illegal_instruction <= '1';
        end if;
        -- illegal E-CPU register? --
        illegal_register <= execute_engine.i_reg(instr_rs2_msb_c) or execute_engine.i_reg(instr_rs1_msb_c);

      when opcode_jalr_c => -- check JALR.funct3
      -- ------------------------------------------------------------
        if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "000") then
          illegal_instruction <= '0';
        else
          illegal_instruction <= '1';
        end if;
        -- illegal E-CPU register? --
        illegal_register <= execute_engine.i_reg(instr_rs1_msb_c) or execute_engine.i_reg(instr_rd_msb_c);

      when opcode_fence_c => -- check FENCE.funct3
      -- ------------------------------------------------------------
        if ((execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_fencei_c) and (CPU_EXTENSION_RISCV_Zifencei = true)) or -- FENCE.I
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_fence_c) then -- FENCE
          illegal_instruction <= '0';
        else
          illegal_instruction <= '1';
        end if;
        -- NOTE: ignore all remaining bit fields here

      when opcode_syscsr_c => -- check system instructions
      -- ------------------------------------------------------------
        -- CSR access --
        if ((execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrw_c) or
            (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrs_c) or
            (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrc_c) or
            (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrwi_c) or
            (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrsi_c) or
            (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrci_c)) and
           (csr_acc_valid = '1') then -- valid CSR access?
          illegal_instruction <= '0';
          -- illegal E-CPU register? --
          if (execute_engine.i_reg(instr_funct3_msb_c) = '0') then -- reg-reg CSR
            illegal_register <= execute_engine.i_reg(instr_rs1_msb_c) or execute_engine.i_reg(instr_rd_msb_c);
          else -- reg-imm CSR
            illegal_register <= execute_engine.i_reg(instr_rd_msb_c);
          end if;
        -- system: ecall, ebreak, mret, wfi, dret --
        -- > WFI is always allowed to execute in M-mode; in U-mode it is allowed to execute if mstatus.TW = 0
        elsif (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "000") and
              (decode_aux.rs1_zero = '1') and (decode_aux.rd_zero = '1') and
              ((execute_engine.i_reg(instr_funct12_msb_c  downto instr_funct12_lsb_c) = funct12_ecall_c)  or -- ECALL
               (execute_engine.i_reg(instr_funct12_msb_c  downto instr_funct12_lsb_c) = funct12_ebreak_c) or -- EBREAK 
               ((execute_engine.i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = funct12_mret_c) and (csr.privilege = priv_mode_m_c)) or -- MRET (only allowed in ACTUAL M-mode)
               ((execute_engine.i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = funct12_dret_c) and (CPU_EXTENSION_RISCV_DEBUG = true) and (debug_ctrl.running = '1')) or -- DRET (only allowed in D-mode)
               ((execute_engine.i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = funct12_wfi_c) and ((csr.privilege = priv_mode_m_c) or (csr.mstatus_tw = '0')))) then -- WFI
          illegal_instruction <= '0';
        else
          illegal_instruction <= '1';
        end if;

      when opcode_fop_c => -- floating point operations - single/dual operands
      -- ------------------------------------------------------------
        if (CPU_EXTENSION_RISCV_Zfinx = true) and -- F extension implemented
           (execute_engine.i_reg(instr_funct7_lsb_c+1 downto instr_funct7_lsb_c) = float_single_c) and -- single-precision operations only
           (decode_aux.is_f_op = '1') then -- is correct/supported floating-point instruction
          illegal_instruction <= '0';
        else
          illegal_instruction <= '1';
        end if;
        -- illegal E-CPU register? --
        -- FIXME: rs2 is not checked!
        illegal_register <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zfinx) and (execute_engine.i_reg(instr_rs1_msb_c) or execute_engine.i_reg(instr_rd_msb_c));

      when opcode_cust0_c => -- CFU: custom instructions
      -- ------------------------------------------------------------
        if (CPU_EXTENSION_RISCV_Zxcfu = true) then -- CFU extension implemented
          illegal_instruction <= '0';
        else
          illegal_instruction <= '1';
        end if;
        -- illegal E-CPU register? --
        illegal_register <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zxcfu) and (execute_engine.i_reg(instr_rs2_msb_c) or execute_engine.i_reg(instr_rs1_msb_c) or execute_engine.i_reg(instr_rd_msb_c));

      when others => -- undefined instruction -> illegal!
      -- ------------------------------------------------------------
        illegal_instruction <= '1';

    end case;
  end process illegal_instruction_check;


  -- Illegal Operation Check ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- check in EXECUTE state: any illegal condition? --
  trap_ctrl.instr_il <= (illegal_opcode_lsbs or -- illegal opcode LSB bits - not rv32
                         illegal_instruction or -- illegal instruction
                         (bool_to_ulogic_f(CPU_EXTENSION_RISCV_E) and illegal_register) or -- illegal register access in E extension
                         illegal_compressed) -- illegal compressed instruction
                        when (execute_engine.state = EXECUTE) else '0';


-- ****************************************************************************************************************************
-- Exception and Interrupt (= Traps) Control
-- ****************************************************************************************************************************

  -- Trap Controller ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trap_controller: process(rstn_i, clk_i)
    variable mode_m_v, mode_u_v : std_ulogic;
  begin
    if (rstn_i = '0') then
      trap_ctrl.exc_buf   <= (others => '0');
      trap_ctrl.irq_buf   <= (others => '0');
      trap_ctrl.env_start <= '0';
      trap_ctrl.cause     <= (others => '0');
    elsif rising_edge(clk_i) then
      if (CPU_EXTENSION_RISCV_Zicsr = true) then
        -- > clear all queued exception triggers when starting the trap handling environment (trap_ctrl.env_start = 1)

        -- exception queue: misaligned load/store/instruction address --
        trap_ctrl.exc_buf(exc_lalign_c) <= (trap_ctrl.exc_buf(exc_lalign_c) or ma_load_i)          and (not trap_ctrl.env_start);
        trap_ctrl.exc_buf(exc_salign_c) <= (trap_ctrl.exc_buf(exc_salign_c) or ma_store_i)         and (not trap_ctrl.env_start);
        trap_ctrl.exc_buf(exc_ialign_c) <= (trap_ctrl.exc_buf(exc_ialign_c) or trap_ctrl.instr_ma) and (not trap_ctrl.env_start);

        -- exception queue: load/store/instruction bus access error --
        trap_ctrl.exc_buf(exc_laccess_c) <= (trap_ctrl.exc_buf(exc_laccess_c) or be_load_i)          and (not trap_ctrl.env_start);
        trap_ctrl.exc_buf(exc_saccess_c) <= (trap_ctrl.exc_buf(exc_saccess_c) or be_store_i)         and (not trap_ctrl.env_start);
        trap_ctrl.exc_buf(exc_iaccess_c) <= (trap_ctrl.exc_buf(exc_iaccess_c) or trap_ctrl.instr_be) and (not trap_ctrl.env_start);

        -- exception queue: illegal instruction / environment calls --
        trap_ctrl.exc_buf(exc_m_envcall_c) <= (trap_ctrl.exc_buf(exc_m_envcall_c) or (trap_ctrl.env_call and (    csr.privilege))) and (not trap_ctrl.env_start);
        trap_ctrl.exc_buf(exc_u_envcall_c) <= (trap_ctrl.exc_buf(exc_u_envcall_c) or (trap_ctrl.env_call and (not csr.privilege))) and (not trap_ctrl.env_start);
        trap_ctrl.exc_buf(exc_iillegal_c)  <= (trap_ctrl.exc_buf(exc_iillegal_c)  or trap_ctrl.instr_il)                           and (not trap_ctrl.env_start);

        -- exception queue: break point --
        if (CPU_EXTENSION_RISCV_DEBUG = true) then
          trap_ctrl.exc_buf(exc_break_c) <= (not trap_ctrl.env_start) and (trap_ctrl.exc_buf(exc_break_c) or 
            (trap_ctrl.break_point and (    csr.privilege) and (not csr.dcsr_ebreakm) and (not debug_ctrl.running)) or -- break to machine-trap-handler when in machine mode on "ebreak"
            (trap_ctrl.break_point and (not csr.privilege) and (not csr.dcsr_ebreaku) and (not debug_ctrl.running))); -- break to machine-trap-handler when in user mode on "ebreak"
        else
          trap_ctrl.exc_buf(exc_break_c) <= (trap_ctrl.exc_buf(exc_break_c) or trap_ctrl.break_point) and (not trap_ctrl.env_start);
        end if;

        -- exception queue / interrupt buffer: enter debug mode --
        if (CPU_EXTENSION_RISCV_DEBUG = true) then
          trap_ctrl.exc_buf(exc_db_break_c) <= (trap_ctrl.exc_buf(exc_db_break_c) or debug_ctrl.trig_break) and (not trap_ctrl.env_start);
          trap_ctrl.exc_buf(exc_db_hw_c)    <= (trap_ctrl.exc_buf(exc_db_hw_c)    or debug_ctrl.trig_hw)    and (not trap_ctrl.env_start);
          trap_ctrl.irq_buf(irq_db_halt_c)  <= debug_ctrl.trig_halt;
          trap_ctrl.irq_buf(irq_db_step_c)  <= debug_ctrl.trig_step;
        end if;

        -- interrupt buffer: machine software/external/timer interrupt --
        trap_ctrl.irq_buf(irq_msw_irq_c)   <= csr.mie_msie and msw_irq_i;
        trap_ctrl.irq_buf(irq_mext_irq_c)  <= csr.mie_meie and mext_irq_i;
        trap_ctrl.irq_buf(irq_mtime_irq_c) <= csr.mie_mtie and mtime_irq_i;

        -- interrupt *queue*: NEORV32-specific fast interrupts (FIRQ) - require manual ACK/clear --
        trap_ctrl.irq_buf(irq_firq_15_c downto irq_firq_0_c) <= (trap_ctrl.irq_buf(irq_firq_15_c downto irq_firq_0_c) or (csr.mie_firqe and firq_i)) and csr.mip_firq_nclr;

        -- trap environment control --
        if (trap_ctrl.env_start = '0') then -- no started trap handler
          if (trap_ctrl.exc_fire = '1') or ((trap_ctrl.irq_fire = '1') and -- exception triggered!
             ((execute_engine.state = EXECUTE) or (execute_engine.state = TRAP_ENTER))) then -- fire IRQs in EXECUTE or TRAP state only to continue execution even on permanent IRQ
            trap_ctrl.cause     <= trap_ctrl.cause_nxt; -- capture trap ID for mcause csr
            trap_ctrl.env_start <= '1';                 -- now execute engine can start trap handler
          end if;
        elsif (trap_ctrl.env_start_ack = '1') then -- start of trap handler acknowledged by execute engine
          trap_ctrl.env_start <= '0';
        end if;
      end if;
    end if;
  end process trap_controller;

  -- any exception/interrupt? --
  trap_ctrl.exc_fire <= or_reduce_f(trap_ctrl.exc_buf); -- sync. exceptions CANNOT be masked
  trap_ctrl.irq_fire <= (or_reduce_f(trap_ctrl.irq_buf) and csr.mstatus_mie and trap_ctrl.db_irq_en) or trap_ctrl.db_irq_fire; -- interrupts CAN be masked (but not the DEBUG halt IRQ)

  -- debug mode (entry) interrupts --
  trap_ctrl.db_irq_en   <= '0' when (CPU_EXTENSION_RISCV_DEBUG = true) and ((debug_ctrl.running = '1') or (csr.dcsr_step = '1')) else '1'; -- no interrupts when IN debug mode or IN single-step mode
  trap_ctrl.db_irq_fire <= (trap_ctrl.irq_buf(irq_db_step_c) or trap_ctrl.irq_buf(irq_db_halt_c)) when (CPU_EXTENSION_RISCV_DEBUG = true) else '0'; -- "NMI" for debug mode entry


  -- Trap Priority Encoder ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trap_priority: process(trap_ctrl)
  begin
    -- ----------------------------------------------------------------------------------------
    -- the following traps are caused by *synchronous* exceptions; we do not need a
    -- specific acknowledge mask since only _one_ exception (the one with highest priority)
    -- is allowed to kick in at once
    -- ----------------------------------------------------------------------------------------

    -- exception: 0.0 instruction address misaligned --
    if (trap_ctrl.exc_buf(exc_ialign_c) = '1') then
      trap_ctrl.cause_nxt <= trap_ima_c;

    -- exception: 0.1 instruction access fault --
    elsif (trap_ctrl.exc_buf(exc_iaccess_c) = '1') then
      trap_ctrl.cause_nxt <= trap_iba_c;

    -- exception: 0.2 illegal instruction --
    elsif (trap_ctrl.exc_buf(exc_iillegal_c) = '1') then
      trap_ctrl.cause_nxt <= trap_iil_c;


    -- exception: 0.11 environment call from M-mode --
    elsif (trap_ctrl.exc_buf(exc_m_envcall_c) = '1') then
      trap_ctrl.cause_nxt <= trap_menv_c;

    -- exception: 0.8 environment call from U-mode --
    elsif (trap_ctrl.exc_buf(exc_u_envcall_c) = '1') then
      trap_ctrl.cause_nxt <= trap_uenv_c;

    -- exception: 0.3 breakpoint --
    elsif (trap_ctrl.exc_buf(exc_break_c) = '1') then
      trap_ctrl.cause_nxt <= trap_brk_c;


    -- exception: 0.6 store address misaligned -
    elsif (trap_ctrl.exc_buf(exc_salign_c) = '1') then
      trap_ctrl.cause_nxt <= trap_sma_c;

    -- exception: 0.4 load address misaligned --
    elsif (trap_ctrl.exc_buf(exc_lalign_c) = '1') then
      trap_ctrl.cause_nxt <= trap_lma_c;

    -- exception: 0.7 store access fault --
    elsif (trap_ctrl.exc_buf(exc_saccess_c) = '1') then
      trap_ctrl.cause_nxt <= trap_sbe_c;

    -- exception: 0.5 load access fault --
    elsif (trap_ctrl.exc_buf(exc_laccess_c) = '1') then
      trap_ctrl.cause_nxt <= trap_lbe_c;

    -- ----------------------------------------------------------------------------------------
    -- (re-)enter debug mode requests: basically, these are standard traps that have some
    -- special handling - they have the highest INTERRUPT priority in order to go to debug when requested
    -- even if other IRQs are pending right now
    -- ----------------------------------------------------------------------------------------

    -- hardware trigger (sync) --
    elsif (trap_ctrl.exc_buf(exc_db_hw_c) = '1') then
      trap_ctrl.cause_nxt <= trap_db_hw_c;

    -- break instruction (sync) --
    elsif (trap_ctrl.exc_buf(exc_db_break_c) = '1') then
      trap_ctrl.cause_nxt <= trap_db_break_c;

    -- async. exceptions / interrupts

    -- external halt request (async) --
    elsif (trap_ctrl.irq_buf(irq_db_halt_c) = '1') then
      trap_ctrl.cause_nxt <= trap_db_halt_c;

    -- single stepping (async) --
    elsif (trap_ctrl.irq_buf(irq_db_step_c) = '1') then
      trap_ctrl.cause_nxt <= trap_db_step_c;

    -- ----------------------------------------------------------------------------------------
    -- custom FAST interrupts (*asynchronous* exceptions)
    -- ----------------------------------------------------------------------------------------

    -- interrupt: 1.16 fast interrupt channel 0 --
    elsif (trap_ctrl.irq_buf(irq_firq_0_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq0_c;

    -- interrupt: 1.17 fast interrupt channel 1 --
    elsif (trap_ctrl.irq_buf(irq_firq_1_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq1_c;

    -- interrupt: 1.18 fast interrupt channel 2 --
    elsif (trap_ctrl.irq_buf(irq_firq_2_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq2_c;

    -- interrupt: 1.19 fast interrupt channel 3 --
    elsif (trap_ctrl.irq_buf(irq_firq_3_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq3_c;

    -- interrupt: 1.20 fast interrupt channel 4 --
    elsif (trap_ctrl.irq_buf(irq_firq_4_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq4_c;

    -- interrupt: 1.21 fast interrupt channel 5 --
    elsif (trap_ctrl.irq_buf(irq_firq_5_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq5_c;

    -- interrupt: 1.22 fast interrupt channel 6 --
    elsif (trap_ctrl.irq_buf(irq_firq_6_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq6_c;

    -- interrupt: 1.23 fast interrupt channel 7 --
    elsif (trap_ctrl.irq_buf(irq_firq_7_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq7_c;

    -- interrupt: 1.24 fast interrupt channel 8 --
    elsif (trap_ctrl.irq_buf(irq_firq_8_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq8_c;

    -- interrupt: 1.25 fast interrupt channel 9 --
    elsif (trap_ctrl.irq_buf(irq_firq_9_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq9_c;

    -- interrupt: 1.26 fast interrupt channel 10 --
    elsif (trap_ctrl.irq_buf(irq_firq_10_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq10_c;

    -- interrupt: 1.27 fast interrupt channel 11 --
    elsif (trap_ctrl.irq_buf(irq_firq_11_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq11_c;

    -- interrupt: 1.28 fast interrupt channel 12 --
    elsif (trap_ctrl.irq_buf(irq_firq_12_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq12_c;

    -- interrupt: 1.29 fast interrupt channel 13 --
    elsif (trap_ctrl.irq_buf(irq_firq_13_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq13_c;

    -- interrupt: 1.30 fast interrupt channel 14 --
    elsif (trap_ctrl.irq_buf(irq_firq_14_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq14_c;

    -- interrupt: 1.31 fast interrupt channel 15 --
    elsif (trap_ctrl.irq_buf(irq_firq_15_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq15_c;

    -- ----------------------------------------------------------------------------------------
    -- standard RISC-V interrupts (*asynchronous* exceptions)
    -- ----------------------------------------------------------------------------------------

    -- interrupt: 1.11 machine external interrupt (MEI) --
    elsif (trap_ctrl.irq_buf(irq_mext_irq_c) = '1') then
      trap_ctrl.cause_nxt <= trap_mei_c;

    -- interrupt: 1.3 machine SW interrupt (MSI) --
    elsif (trap_ctrl.irq_buf(irq_msw_irq_c) = '1') then
      trap_ctrl.cause_nxt <= trap_msi_c;

    -- interrupt: 1.7 machine timer interrupt (MTI) --
    else--if (trap_ctrl.irq_buf(irq_mtime_irq_c) = '1') then -- last condition, so NO IF required
      trap_ctrl.cause_nxt <= trap_mti_c;

    end if;
  end process trap_priority;
  

-- ****************************************************************************************************************************
-- Control and Status Registers (CSRs)
-- ****************************************************************************************************************************

  -- Control and Status Registers - Write Data ----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_write_data: process(execute_engine.i_reg, csr.rdata, rs1_i)
    variable csr_imm_v : std_ulogic_vector(data_width_c-1 downto 0);
  begin
    -- tiny ALU to compute CSR write data --
    csr_imm_v := (others => '0');
    csr_imm_v(4 downto 0) := execute_engine.i_reg(19 downto 15); -- uimm5
    case execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) is
      when funct3_csrrw_c  => csr.wdata <= rs1_i;
      when funct3_csrrs_c  => csr.wdata <= csr.rdata or rs1_i;
      when funct3_csrrc_c  => csr.wdata <= csr.rdata and (not rs1_i);
      when funct3_csrrwi_c => csr.wdata <= csr_imm_v;
      when funct3_csrrsi_c => csr.wdata <= csr.rdata or csr_imm_v;
      when funct3_csrrci_c => csr.wdata <= csr.rdata and (not csr_imm_v);
      when others          => csr.wdata <= (others => '-'); -- undefined
    end case;
  end process csr_write_data;


  -- Control and Status Registers - Write Access --------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_write_access: process(rstn_i, clk_i)
    variable cause_v : std_ulogic_vector(6 downto 0);
  begin
    if (rstn_i = '0') then
      csr.we                <= '0';
      --
      csr.mstatus_mie       <= '0';
      csr.mstatus_mpie      <= '0';
      csr.mstatus_mpp       <= '0';
      csr.mstatus_tw        <= '0';
      csr.privilege         <= priv_mode_m_c; -- start in MACHINE mode
      csr.mie_msie          <= def_rst_val_c;
      csr.mie_meie          <= def_rst_val_c;
      csr.mie_mtie          <= def_rst_val_c;
      csr.mie_firqe         <= (others => def_rst_val_c);
      csr.mtvec             <= (others => def_rst_val_c);
      csr.mscratch          <= x"19880704";
      csr.mepc              <= (others => def_rst_val_c);
      csr.mcause            <= (others => def_rst_val_c);
      csr.mtval             <= (others => def_rst_val_c);
      csr.mip_firq_nclr     <= (others => def_rst_val_c);
      --
      csr.pmpcfg            <= (others => (others => '0'));
      csr.pmpaddr           <= (others => (others => def_rst_val_c));
      --
      csr.mhpmevent         <= (others => (others => def_rst_val_c));
      --
      csr.mcounteren_cy     <= def_rst_val_c;
      csr.mcounteren_tm     <= def_rst_val_c;
      csr.mcounteren_ir     <= def_rst_val_c;
      --
      csr.mcountinhibit_cy  <= def_rst_val_c;
      csr.mcountinhibit_ir  <= def_rst_val_c;
      csr.mcountinhibit_hpm <= (others => def_rst_val_c);
      --
      csr.fflags            <= (others => def_rst_val_c);
      csr.frm               <= (others => def_rst_val_c);
      --
      csr.dcsr_ebreakm      <= '0';
      csr.dcsr_ebreaku      <= '0';
      csr.dcsr_step         <= '0';
      csr.dcsr_prv          <= priv_mode_m_c;
      csr.dcsr_cause        <= (others => def_rst_val_c);
      csr.dpc               <= (others => def_rst_val_c);
      csr.dscratch0         <= (others => def_rst_val_c);
      --
      csr.tdata1_exe        <= '0';
      csr.tdata2            <= (others => def_rst_val_c);

    elsif rising_edge(clk_i) then
      -- write access? --
      csr.we <= csr.we_nxt;

      -- defaults --
      csr.mip_firq_nclr <= (others => '1'); -- active low

      if (CPU_EXTENSION_RISCV_Zicsr = true) then
        -- --------------------------------------------------------------------------------
        -- CSR access by application software
        -- --------------------------------------------------------------------------------
        if (csr.we = '1') and (trap_ctrl.exc_buf(exc_iillegal_c) = '0') then -- manual write access and not illegal instruction

          -- user floating-point CSRs --
          -- --------------------------------------------------------------------
          if (CPU_EXTENSION_RISCV_Zfinx = true) then -- floating point CSR class
            if (csr.addr(11 downto 2) = csr_class_float_c) then
              if (csr.addr(1 downto 0) = "01") then -- R/W: fflags - floating-point (FPU) exception flags
                csr.fflags <= csr.wdata(4 downto 0);
              elsif (csr.addr(1 downto 0) = "10") then -- R/W: frm - floating-point (FPU) rounding mode
                csr.frm    <= csr.wdata(2 downto 0);
              elsif (csr.addr(1 downto 0) = "11") then -- R/W: fcsr - floating-point (FPU) control/status (frm + fflags)
                csr.frm    <= csr.wdata(7 downto 5);
                csr.fflags <= csr.wdata(4 downto 0);
              end if;
            end if;
          end if;

          -- machine trap setup --
          -- --------------------------------------------------------------------
          if (csr.addr(11 downto 3) = csr_class_setup_c) then -- trap setup CSR class
            -- R/W: mstatus - machine status register --
            if (csr.addr(2 downto 0) = csr_mstatus_c(2 downto 0)) then
              csr.mstatus_mie  <= csr.wdata(03);
              csr.mstatus_mpie <= csr.wdata(07);
              if (CPU_EXTENSION_RISCV_U = true) then -- user mode implemented
                csr.mstatus_mpp <= csr.wdata(11) or csr.wdata(12); -- everything /= U will fall back to M
                csr.mstatus_tw  <= csr.wdata(21);
              end if;
            end if;
            -- R/W: mie - machine interrupt enable register --
            if (csr.addr(2 downto 0) = csr_mie_c(2 downto 0)) then
              csr.mie_msie  <= csr.wdata(03); -- machine SW IRQ enable
              csr.mie_mtie  <= csr.wdata(07); -- machine TIMER IRQ enable
              csr.mie_meie  <= csr.wdata(11); -- machine EXT IRQ enable
              csr.mie_firqe <= csr.wdata(31 downto 16); -- fast interrupt channels 0..15
            end if;
            -- R/W: mtvec - machine trap-handler base address (for ALL exceptions) --
            if (csr.addr(2 downto 0) = csr_mtvec_c(2 downto 0)) then
              csr.mtvec <= csr.wdata(data_width_c-1 downto 2) & "00"; -- mtvec.MODE=0
            end if;
            -- R/W: mcounteren - machine counter enable register --
            if (CPU_EXTENSION_RISCV_U = true) then -- this CSR is hardwired to zero if user mode is not implemented
              if (csr.addr(2 downto 0) = csr_mcounteren_c(2 downto 0)) then
                csr.mcounteren_cy <= csr.wdata(0); -- enable user-level access to cycle[h]
                csr.mcounteren_tm <= csr.wdata(1); -- enable user-level access to time[h]
                csr.mcounteren_ir <= csr.wdata(2); -- enable user-level access to instret[h]
              end if;
            end if;
          end if;

          -- machine trap handling --
          -- --------------------------------------------------------------------
          if (csr.addr(11 downto 4) = csr_class_trap_c) then -- machine trap handling CSR class
            -- R/W: mscratch - machine scratch register --
            if (csr.addr(3 downto 0) = csr_mscratch_c(3 downto 0)) then
              csr.mscratch <= csr.wdata;
            end if;
            -- R/W: mepc - machine exception program counter --
            if (csr.addr(3 downto 0) = csr_mepc_c(3 downto 0)) then
              csr.mepc <= csr.wdata;
            end if;
            -- R/W: mcause - machine trap cause --
            if (csr.addr(3 downto 0) = csr_mcause_c(3 downto 0)) then
              csr.mcause(csr.mcause'left) <= csr.wdata(31); -- 1: async/interrupt, 0: sync/exception
              csr.mcause(4 downto 0)      <= csr.wdata(4 downto 0); -- identifier
            end if;
            -- R/W: mip - machine interrupt pending --
            if (csr.addr(3 downto 0) = csr_mip_c(3 downto 0)) then
              csr.mip_firq_nclr <= csr.wdata(31 downto 16); -- set low to clear according bit (FIRQs only)
            end if;
          end if;

          -- physical memory protection: R/W: pmpcfg* - PMP configuration registers --
          -- --------------------------------------------------------------------
          if (PMP_NUM_REGIONS > 0) then
            if (csr.addr(11 downto 2) = csr_class_pmpcfg_c) then -- pmp configuration CSR class
              for i in 0 to 3 loop -- 3 pmpcfg CSRs
                if (csr.addr(1 downto 0) = std_ulogic_vector(to_unsigned(i, 2))) then
                  for j in 0 to 3 loop -- 4 entries per CSR
                    if (csr.pmpcfg(i*4+j)(7) = '0') then -- unlocked pmpcfg entry
                      csr.pmpcfg(i*4+j)(0) <= csr.wdata(j*8+0); -- R - read
                      csr.pmpcfg(i*4+j)(1) <= csr.wdata(j*8+1); -- W - write
                      csr.pmpcfg(i*4+j)(2) <= csr.wdata(j*8+2); -- X - execute
                      csr.pmpcfg(i*4+j)(3) <= csr.wdata(j*8+3); -- A_L - mode low [TOR-mode only!]
                      csr.pmpcfg(i*4+j)(4) <= '0'; -- A_H - mode high [TOR-mode only!]
                      csr.pmpcfg(i*4+j)(5) <= '0'; -- reserved
                      csr.pmpcfg(i*4+j)(6) <= '0'; -- reserved
                      csr.pmpcfg(i*4+j)(7) <= csr.wdata(j*8+7); -- L (locked / also enforce in machine-mode)
                    end if;
                  end loop; -- j (entry)
                end if;
              end loop; -- i (pmpcfg CSR)
            end if;
          end if;

          -- physical memory protection: R/W: pmpaddr* - PMP address registers --
          -- --------------------------------------------------------------------
          if (PMP_NUM_REGIONS > 0) then
            if (csr.addr(11 downto 4) = csr_class_pmpaddr_c) then 
              for i in 0 to PMP_NUM_REGIONS-1 loop
                if (csr.addr(3 downto 0) = std_ulogic_vector(to_unsigned(i, 4))) and (csr.pmpcfg(i)(7) = '0') then -- unlocked pmpaddr access
                  csr.pmpaddr(i) <= csr.wdata(data_width_c-3 downto index_size_f(PMP_MIN_GRANULARITY)-2);
                end if;
              end loop; -- i (PMP regions)
            end if;
          end if;

          -- machine counter setup --
          -- --------------------------------------------------------------------
          if (csr.addr(11 downto 5) = csr_cnt_setup_c) then -- counter configuration CSR class
            -- R/W: mcountinhibit - machine counter-inhibit register --
            if (csr.addr(4 downto 0) = csr_mcountinhibit_c(4 downto 0)) then
              csr.mcountinhibit_cy <= csr.wdata(0); -- enable auto-increment of [m]cycle[h] counter
              csr.mcountinhibit_ir <= csr.wdata(2); -- enable auto-increment of [m]instret[h] counter
              if (HPM_NUM_CNTS > 0) and (CPU_EXTENSION_RISCV_Zihpm = true) then -- any HPMs available?
                csr.mcountinhibit_hpm <= csr.wdata(csr.mcountinhibit_hpm'left+3 downto 3); -- enable auto-increment of [m]hpmcounter*[h] counter
              end if;
            end if;
            -- R/W: mhpmevent - machine performance-monitors event selector --
            if (HPM_NUM_CNTS > 0) and (CPU_EXTENSION_RISCV_Zihpm = true) then
              for i in 0 to HPM_NUM_CNTS-1 loop
                if (csr.addr(4 downto 0) = std_ulogic_vector(to_unsigned(i+3, 5))) then
                  csr.mhpmevent(i) <= csr.wdata(csr.mhpmevent(i)'left downto 0);
                end if;
                csr.mhpmevent(i)(hpmcnt_event_never_c) <= '0'; -- would be used for "TIME"
              end loop; -- i (CSRs)
            end if;
          end if;

          -- debug mode CSRs --
          -- --------------------------------------------------------------------
          if (CPU_EXTENSION_RISCV_DEBUG = true) then
            if (csr.addr(11 downto 2) = csr_class_debug_c) then -- debug CSR class
              -- R/W: dcsr - debug mode control and status register --
              if (csr.addr(1 downto 0) = csr_dcsr_c(1 downto 0)) then
                csr.dcsr_ebreakm <= csr.wdata(15);
                csr.dcsr_step    <= csr.wdata(2);
                if (CPU_EXTENSION_RISCV_U = true) then -- user mode implemented
                  csr.dcsr_ebreaku <= csr.wdata(12);
                  csr.dcsr_prv     <= csr.wdata(1) or csr.wdata(0); -- everything /= U will fall back to M
                end if;
              end if;
              -- R/W: dpc - debug mode program counter --
              if (csr.addr(1 downto 0) = csr_dpc_c(1 downto 0)) then
                csr.dpc <= csr.wdata(data_width_c-1 downto 1) & '0';
              end if;
              -- R/W: dscratch0 - debug mode scratch register 0 --
              if (csr.addr(1 downto 0) = csr_dscratch0_c(1 downto 0)) then
                csr.dscratch0 <= csr.wdata;
              end if;
            end if;
          end if;

          -- trigger module CSRs - only writable in DEBUG MODE (dmode == 1) --
          -- --------------------------------------------------------------------
          if (CPU_EXTENSION_RISCV_DEBUG = true) then
            if (csr.addr(11 downto 4) = csr_class_trigger_c) then -- trigger CSR class
              if (debug_ctrl.running = '1') then -- actual write only in debug mode
                -- R/W: tdata1 - match control --
                if (csr.addr(3 downto 0) = csr_tdata1_c(3 downto 0)) then
                  csr.tdata1_exe <= csr.wdata(2);
                end if;
                -- R/W: tdata2 - address compare --
                if (csr.addr(3 downto 0) = csr_tdata2_c(3 downto 0)) then
                  csr.tdata2 <= csr.wdata(data_width_c-1 downto 1) & '0';
                end if;
              end if;
            end if;
          end if;


        -- --------------------------------------------------------------------------------
        -- CSR access by hardware
        -- --------------------------------------------------------------------------------
        else

          -- --------------------------------------------------------------------
          -- floating-point (FPU) exception flags
          -- --------------------------------------------------------------------
          if (CPU_EXTENSION_RISCV_Zfinx = true) and (trap_ctrl.exc_buf(exc_iillegal_c) = '0') then -- no illegal instruction
            csr.fflags <= csr.fflags or fpu_flags_i; -- accumulate flags ("accrued exception flags")
          end if;

          -- --------------------------------------------------------------------
          -- TRAP ENTER: write machine trap cause, PC and trap value register
          -- --------------------------------------------------------------------
          if (trap_ctrl.env_start_ack = '1') then -- trap handler starting?

            -- normal trap entry: write mcause, mepc and mtval --
            -- --------------------------------------------------------------------
            if (CPU_EXTENSION_RISCV_DEBUG = false) or ((trap_ctrl.cause(5) = '0') and (debug_ctrl.running = '0')) then

              -- trap cause ID code --
              csr.mcause(csr.mcause'left) <= trap_ctrl.cause(trap_ctrl.cause'left); -- 1: interrupt, 0: exception
              csr.mcause(4 downto 0)      <= trap_ctrl.cause(4 downto 0); -- identifier

              -- trap PC --
              if (trap_ctrl.cause(trap_ctrl.cause'left) = '1') then -- for INTERRUPTS (async source)
                csr.mepc <= execute_engine.pc(data_width_c-1 downto 1) & '0'; -- this is the CURRENT pc = interrupted instruction
              else -- for sync. EXCEPTIONS (sync source)
                csr.mepc <= execute_engine.pc_last(data_width_c-1 downto 1) & '0'; -- this is the LAST pc = last executed instruction
              end if;

              -- trap value --
              cause_v := trap_ctrl.cause;
              cause_v(5) := '0'; -- bit 5 is always zero here (= normal trapping), so we do not need to check that again
              case cause_v is
                when trap_ima_c | trap_iba_c => -- misaligned instruction address OR instruction access error
                  csr.mtval <= execute_engine.pc(data_width_c-1 downto 1) & '0'; -- address of faulting instruction
                when trap_brk_c => -- breakpoint
                  csr.mtval <= execute_engine.pc_last(data_width_c-1 downto 1) & '0'; -- address of breakpoint instruction
                when trap_lma_c | trap_lbe_c | trap_sma_c | trap_sbe_c => -- misaligned load/store address OR load/store access error
                  csr.mtval <= mar_i; -- faulting data access address
                when trap_iil_c => -- illegal instruction
                  csr.mtval <= execute_engine.i_reg_last; -- faulting instruction itself
                when others => -- everything else including all interrupts
                  csr.mtval <= (others => '0');
              end case;

            end if;

            -- DEBUG MODE (trap) enter: write dpc and dcsr --
            -- --------------------------------------------------------------------
            if (CPU_EXTENSION_RISCV_DEBUG = true) and (trap_ctrl.cause(5) = '1') and (debug_ctrl.running = '0') then

              -- trap cause ID code --
              csr.dcsr_cause <= trap_ctrl.cause(2 downto 0); -- why did we enter debug mode?

              -- current privilege mode when debug mode was entered --
              csr.dcsr_prv <= csr.privilege;

              -- trap PC --
              if (trap_ctrl.cause(trap_ctrl.cause'left) = '1') then -- for INTERRUPTS (async source)
                csr.dpc <= execute_engine.pc(data_width_c-1 downto 1) & '0'; -- this is the CURRENT pc = interrupted instruction
              else -- for sync. EXCEPTIONS (sync source)
                csr.dpc <= execute_engine.pc_last(data_width_c-1 downto 1) & '0'; -- this is the LAST pc = last executed instruction
              end if;

            end if;

          end if;

          -- --------------------------------------------------------------------
          -- mstatus: context switch
          -- --------------------------------------------------------------------
          -- ENTER: trap handler starting
          if (trap_ctrl.env_start_ack = '1') then -- trap handler starting?
            if (CPU_EXTENSION_RISCV_DEBUG = false) or -- normal trapping (debug mode NOT implemented)
               ((debug_ctrl.running = '0') and (trap_ctrl.cause(5) = '0')) then -- not IN debug mode and not ENTERING debug mode
              csr.mstatus_mie  <= '0'; -- disable interrupts
              csr.mstatus_mpie <= csr.mstatus_mie; -- buffer previous mie state
              if (CPU_EXTENSION_RISCV_U = true) then -- user mode implemented
                csr.privilege   <= priv_mode_m_c; -- execute trap in machine mode
                csr.mstatus_mpp <= csr.privilege; -- backup previous privilege mode
              end if;
            end if;

          -- EXIT: return from trap
          elsif (trap_ctrl.env_end = '1') then
            if (CPU_EXTENSION_RISCV_DEBUG = true) and (debug_ctrl.running = '1') then -- return from debug mode
              if (CPU_EXTENSION_RISCV_U = true) then -- user mode implemented
                csr.privilege <= csr.dcsr_prv;
              end if;
            else -- return from "normal trap"
              csr.mstatus_mie  <= csr.mstatus_mpie; -- restore global IRQ enable flag
              csr.mstatus_mpie <= '1';
              if (CPU_EXTENSION_RISCV_U = true) then -- user mode implemented
                csr.privilege   <= csr.mstatus_mpp; -- go back to previous privilege mode
                csr.mstatus_mpp <= '0'; -- MRET has to clear mstatus.MPP
              end if;
            end if;
          end if;

        end if; -- /hardware csr access
      end if;
    end if;
  end process csr_write_access;

  -- effective privilege mode is M when in debug mode --
  csr.privilege_eff <= priv_mode_m_c when (CPU_EXTENSION_RISCV_DEBUG = true) and (debug_ctrl.running = '1') else csr.privilege;

  -- PMP output to bus unit --
  pmp_output:
  for i in 0 to PMP_NUM_REGIONS-1 generate
    pmp_addr_o(i)(data_width_c-1 downto index_size_f(PMP_MIN_GRANULARITY)) <= csr.pmpaddr(i); -- physical address
    pmp_ctrl_o(i) <= csr.pmpcfg(i);
  end generate;


  -- Control and Status Registers - Read Access ---------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_read_access: process(rstn_i, clk_i)
    variable csr_addr_v : std_ulogic_vector(11 downto 0);
  begin
    if rising_edge(clk_i) then
      csr.rdata <= (others => '0'); -- default output, unimplemented CSRs are hardwired to zero
      if (CPU_EXTENSION_RISCV_Zicsr = true) then
        csr_addr_v(11 downto 10) := csr.addr(11 downto 10);
        csr_addr_v(09 downto 08) := (others => csr.addr(8)); -- !!! WARNING: MACHINE (11) and USER (00) CSRs ONLY !!!
        csr_addr_v(07 downto 00) := csr.addr(07 downto 00);
        case csr_addr_v is

          -- floating-point CSRs --
          -- --------------------------------------------------------------------
          when csr_fflags_c => -- fflags (r/w): floating-point (FPU) exception flags
            if (CPU_EXTENSION_RISCV_Zfinx) then csr.rdata(4 downto 0) <= csr.fflags; else NULL; end if;
          when csr_frm_c => -- frm (r/w): floating-point (FPU) rounding mode
            if (CPU_EXTENSION_RISCV_Zfinx) then csr.rdata(2 downto 0) <= csr.frm; else NULL; end if;
          when csr_fcsr_c => -- fcsr (r/w): floating-point (FPU) control/status (frm + fflags)
            if (CPU_EXTENSION_RISCV_Zfinx) then csr.rdata(7 downto 0) <= csr.frm & csr.fflags; else NULL; end if;

          -- machine trap setup --
          -- --------------------------------------------------------------------
          when csr_mstatus_c => -- mstatus (r/w): machine status register
            csr.rdata(03) <= csr.mstatus_mie; -- MIE
            csr.rdata(07) <= csr.mstatus_mpie; -- MPIE
            csr.rdata(12 downto 11) <= (others => csr.mstatus_mpp); -- MPP: machine previous privilege mode
            csr.rdata(21) <= csr.mstatus_tw and bool_to_ulogic_f(CPU_EXTENSION_RISCV_U); -- TW
--        when csr_mstatush_c => -- mstatush (r/w): machine status register - high, implemented but always zero
--          csr.rdata <= (others => '0');
          when csr_misa_c => -- misa (r/-): ISA and extensions
            csr.rdata(00) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_A);     -- A CPU extension
            csr.rdata(01) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_B);     -- B CPU extension
            csr.rdata(02) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_C);     -- C CPU extension
            csr.rdata(04) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_E);     -- E CPU extension
            csr.rdata(08) <= not bool_to_ulogic_f(CPU_EXTENSION_RISCV_E); -- I CPU extension (if not E)
            csr.rdata(12) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_M);     -- M CPU extension
            csr.rdata(20) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_U);     -- U CPU extension
            csr.rdata(23) <= '1';                                         -- X CPU extension (non-standard extensions / NEORV32-specific)
            csr.rdata(30) <= '1'; -- 32-bit architecture (MXL lo)
            csr.rdata(31) <= '0'; -- 32-bit architecture (MXL hi)
          when csr_mie_c => -- mie (r/w): machine interrupt-enable register
            csr.rdata(03) <= csr.mie_msie; -- machine software IRQ enable
            csr.rdata(07) <= csr.mie_mtie; -- machine timer IRQ enable
            csr.rdata(11) <= csr.mie_meie; -- machine external IRQ enable
            csr.rdata(31 downto 16) <= csr.mie_firqe;
          when csr_mtvec_c => -- mtvec (r/w): machine trap-handler base address (for ALL exceptions)
            csr.rdata <= csr.mtvec(data_width_c-1 downto 2) & "00"; -- mtvec.MODE=0
          when csr_mcounteren_c => -- mcounteren (r/w): machine counter enable register,  hardwired to zero if user mode is not implemented
            csr.rdata(0) <= csr.mcounteren_cy and bool_to_ulogic_f(CPU_EXTENSION_RISCV_U); -- enable user-level access to cycle[h]
            csr.rdata(1) <= csr.mcounteren_tm and bool_to_ulogic_f(CPU_EXTENSION_RISCV_U); -- enable user-level access to time[h]
            csr.rdata(2) <= csr.mcounteren_ir and bool_to_ulogic_f(CPU_EXTENSION_RISCV_U); -- enable user-level access to instret[h]

          -- machine trap handling --
          -- --------------------------------------------------------------------
          when csr_mscratch_c => -- mscratch (r/w): machine scratch register
            csr.rdata <= csr.mscratch;
          when csr_mepc_c => -- mepc (r/w): machine exception program counter
            csr.rdata <= csr.mepc(data_width_c-1 downto 1) & '0';
          when csr_mcause_c => -- mcause (r/w): machine trap cause
            csr.rdata(31) <= csr.mcause(csr.mcause'left);
            csr.rdata(csr.mcause'left-1 downto 0) <= csr.mcause(csr.mcause'left-1 downto 0);
          when csr_mtval_c => -- mtval (r/-): machine bad address or instruction
            csr.rdata <= csr.mtval;
          when csr_mip_c => -- mip (r/w): machine interrupt pending
            csr.rdata(03) <= trap_ctrl.irq_buf(irq_msw_irq_c);
            csr.rdata(07) <= trap_ctrl.irq_buf(irq_mtime_irq_c);
            csr.rdata(11) <= trap_ctrl.irq_buf(irq_mext_irq_c);
            csr.rdata(31 downto 16) <= trap_ctrl.irq_buf(irq_firq_15_c downto irq_firq_0_c);

          -- physical memory protection - configuration (r/w) --
          -- --------------------------------------------------------------------
          when csr_pmpcfg0_c =>
            if (PMP_NUM_REGIONS > 0) then
              if (PMP_NUM_REGIONS > 00) then csr.rdata(07 downto 00) <= csr.pmpcfg(00); end if;
              if (PMP_NUM_REGIONS > 01) then csr.rdata(15 downto 08) <= csr.pmpcfg(01); end if;
              if (PMP_NUM_REGIONS > 02) then csr.rdata(23 downto 16) <= csr.pmpcfg(02); end if;
              if (PMP_NUM_REGIONS > 03) then csr.rdata(31 downto 24) <= csr.pmpcfg(03); end if;
            else NULL; end if;
          when csr_pmpcfg1_c =>
            if (PMP_NUM_REGIONS > 4) then
              if (PMP_NUM_REGIONS > 04) then csr.rdata(07 downto 00) <= csr.pmpcfg(04); end if;
              if (PMP_NUM_REGIONS > 05) then csr.rdata(15 downto 08) <= csr.pmpcfg(05); end if;
              if (PMP_NUM_REGIONS > 06) then csr.rdata(23 downto 16) <= csr.pmpcfg(06); end if;
              if (PMP_NUM_REGIONS > 07) then csr.rdata(31 downto 24) <= csr.pmpcfg(07); end if;
            else NULL; end if;
          when csr_pmpcfg2_c =>
            if (PMP_NUM_REGIONS > 8) then
              if (PMP_NUM_REGIONS > 08) then csr.rdata(07 downto 00) <= csr.pmpcfg(08); end if;
              if (PMP_NUM_REGIONS > 09) then csr.rdata(15 downto 08) <= csr.pmpcfg(09); end if;
              if (PMP_NUM_REGIONS > 10) then csr.rdata(23 downto 16) <= csr.pmpcfg(10); end if;
              if (PMP_NUM_REGIONS > 11) then csr.rdata(31 downto 24) <= csr.pmpcfg(11); end if;
            else NULL; end if;
          when csr_pmpcfg3_c =>
            if (PMP_NUM_REGIONS > 12) then
              if (PMP_NUM_REGIONS > 12) then csr.rdata(07 downto 00) <= csr.pmpcfg(12); end if;
              if (PMP_NUM_REGIONS > 13) then csr.rdata(15 downto 08) <= csr.pmpcfg(13); end if;
              if (PMP_NUM_REGIONS > 14) then csr.rdata(23 downto 16) <= csr.pmpcfg(14); end if;
              if (PMP_NUM_REGIONS > 15) then csr.rdata(31 downto 24) <= csr.pmpcfg(15); end if;
            else NULL; end if;

          -- physical memory protection - addresses (r/w) --
          -- --------------------------------------------------------------------
          when csr_pmpaddr0_c  => if (PMP_NUM_REGIONS > 00) then csr.rdata(data_width_c-3 downto index_size_f(PMP_MIN_GRANULARITY)-2) <= csr.pmpaddr(00); else NULL; end if;
          when csr_pmpaddr1_c  => if (PMP_NUM_REGIONS > 01) then csr.rdata(data_width_c-3 downto index_size_f(PMP_MIN_GRANULARITY)-2) <= csr.pmpaddr(01); else NULL; end if;
          when csr_pmpaddr2_c  => if (PMP_NUM_REGIONS > 02) then csr.rdata(data_width_c-3 downto index_size_f(PMP_MIN_GRANULARITY)-2) <= csr.pmpaddr(02); else NULL; end if;
          when csr_pmpaddr3_c  => if (PMP_NUM_REGIONS > 03) then csr.rdata(data_width_c-3 downto index_size_f(PMP_MIN_GRANULARITY)-2) <= csr.pmpaddr(03); else NULL; end if;
          when csr_pmpaddr4_c  => if (PMP_NUM_REGIONS > 04) then csr.rdata(data_width_c-3 downto index_size_f(PMP_MIN_GRANULARITY)-2) <= csr.pmpaddr(04); else NULL; end if;
          when csr_pmpaddr5_c  => if (PMP_NUM_REGIONS > 05) then csr.rdata(data_width_c-3 downto index_size_f(PMP_MIN_GRANULARITY)-2) <= csr.pmpaddr(05); else NULL; end if;
          when csr_pmpaddr6_c  => if (PMP_NUM_REGIONS > 06) then csr.rdata(data_width_c-3 downto index_size_f(PMP_MIN_GRANULARITY)-2) <= csr.pmpaddr(06); else NULL; end if;
          when csr_pmpaddr7_c  => if (PMP_NUM_REGIONS > 07) then csr.rdata(data_width_c-3 downto index_size_f(PMP_MIN_GRANULARITY)-2) <= csr.pmpaddr(07); else NULL; end if;
          when csr_pmpaddr8_c  => if (PMP_NUM_REGIONS > 08) then csr.rdata(data_width_c-3 downto index_size_f(PMP_MIN_GRANULARITY)-2) <= csr.pmpaddr(08); else NULL; end if;
          when csr_pmpaddr9_c  => if (PMP_NUM_REGIONS > 09) then csr.rdata(data_width_c-3 downto index_size_f(PMP_MIN_GRANULARITY)-2) <= csr.pmpaddr(09); else NULL; end if;
          when csr_pmpaddr10_c => if (PMP_NUM_REGIONS > 10) then csr.rdata(data_width_c-3 downto index_size_f(PMP_MIN_GRANULARITY)-2) <= csr.pmpaddr(10); else NULL; end if;
          when csr_pmpaddr11_c => if (PMP_NUM_REGIONS > 11) then csr.rdata(data_width_c-3 downto index_size_f(PMP_MIN_GRANULARITY)-2) <= csr.pmpaddr(11); else NULL; end if;
          when csr_pmpaddr12_c => if (PMP_NUM_REGIONS > 12) then csr.rdata(data_width_c-3 downto index_size_f(PMP_MIN_GRANULARITY)-2) <= csr.pmpaddr(12); else NULL; end if;
          when csr_pmpaddr13_c => if (PMP_NUM_REGIONS > 13) then csr.rdata(data_width_c-3 downto index_size_f(PMP_MIN_GRANULARITY)-2) <= csr.pmpaddr(13); else NULL; end if;
          when csr_pmpaddr14_c => if (PMP_NUM_REGIONS > 14) then csr.rdata(data_width_c-3 downto index_size_f(PMP_MIN_GRANULARITY)-2) <= csr.pmpaddr(14); else NULL; end if;
          when csr_pmpaddr15_c => if (PMP_NUM_REGIONS > 15) then csr.rdata(data_width_c-3 downto index_size_f(PMP_MIN_GRANULARITY)-2) <= csr.pmpaddr(15); else NULL; end if;

          -- machine counter setup --
          -- --------------------------------------------------------------------
          when csr_mcountinhibit_c => -- mcountinhibit (r/w): machine counter-inhibit register
            csr.rdata(0) <= csr.mcountinhibit_cy; -- enable auto-increment of [m]cycle[h] counter
            csr.rdata(2) <= csr.mcountinhibit_ir; -- enable auto-increment of [m]instret[h] counter
            if (HPM_NUM_CNTS > 0) and (CPU_EXTENSION_RISCV_Zihpm = true) then -- any HPMs available?
              csr.rdata(csr.mcountinhibit_hpm'left+3 downto 3) <= csr.mcountinhibit_hpm; -- enable auto-increment of [m]hpmcounterx[h] counter
            end if;

          -- machine performance-monitoring event selector (r/w) --
          -- --------------------------------------------------------------------
          when csr_mhpmevent3_c  => if (HPM_NUM_CNTS > 00) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(00); else NULL; end if;
          when csr_mhpmevent4_c  => if (HPM_NUM_CNTS > 01) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(01); else NULL; end if;
          when csr_mhpmevent5_c  => if (HPM_NUM_CNTS > 02) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(02); else NULL; end if;
          when csr_mhpmevent6_c  => if (HPM_NUM_CNTS > 03) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(03); else NULL; end if;
          when csr_mhpmevent7_c  => if (HPM_NUM_CNTS > 04) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(04); else NULL; end if;
          when csr_mhpmevent8_c  => if (HPM_NUM_CNTS > 05) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(05); else NULL; end if;
          when csr_mhpmevent9_c  => if (HPM_NUM_CNTS > 06) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(06); else NULL; end if;
          when csr_mhpmevent10_c => if (HPM_NUM_CNTS > 07) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(07); else NULL; end if;
          when csr_mhpmevent11_c => if (HPM_NUM_CNTS > 08) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(08); else NULL; end if;
          when csr_mhpmevent12_c => if (HPM_NUM_CNTS > 09) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(09); else NULL; end if;
          when csr_mhpmevent13_c => if (HPM_NUM_CNTS > 10) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(10); else NULL; end if;
          when csr_mhpmevent14_c => if (HPM_NUM_CNTS > 11) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(11); else NULL; end if;
          when csr_mhpmevent15_c => if (HPM_NUM_CNTS > 12) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(12); else NULL; end if;
          when csr_mhpmevent16_c => if (HPM_NUM_CNTS > 13) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(13); else NULL; end if;
          when csr_mhpmevent17_c => if (HPM_NUM_CNTS > 14) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(14); else NULL; end if;
          when csr_mhpmevent18_c => if (HPM_NUM_CNTS > 15) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(15); else NULL; end if;
          when csr_mhpmevent19_c => if (HPM_NUM_CNTS > 16) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(16); else NULL; end if;
          when csr_mhpmevent20_c => if (HPM_NUM_CNTS > 17) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(17); else NULL; end if;
          when csr_mhpmevent21_c => if (HPM_NUM_CNTS > 18) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(18); else NULL; end if;
          when csr_mhpmevent22_c => if (HPM_NUM_CNTS > 19) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(19); else NULL; end if;
          when csr_mhpmevent23_c => if (HPM_NUM_CNTS > 20) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(20); else NULL; end if;
          when csr_mhpmevent24_c => if (HPM_NUM_CNTS > 21) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(21); else NULL; end if;
          when csr_mhpmevent25_c => if (HPM_NUM_CNTS > 22) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(22); else NULL; end if;
          when csr_mhpmevent26_c => if (HPM_NUM_CNTS > 23) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(23); else NULL; end if;
          when csr_mhpmevent27_c => if (HPM_NUM_CNTS > 24) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(24); else NULL; end if;
          when csr_mhpmevent28_c => if (HPM_NUM_CNTS > 25) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(25); else NULL; end if;
          when csr_mhpmevent29_c => if (HPM_NUM_CNTS > 26) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(26); else NULL; end if;
          when csr_mhpmevent30_c => if (HPM_NUM_CNTS > 27) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(27); else NULL; end if;
          when csr_mhpmevent31_c => if (HPM_NUM_CNTS > 28) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmevent_rd(28); else NULL; end if;

          -- counters and timers --
          -- --------------------------------------------------------------------
          when csr_cycle_c | csr_mcycle_c => -- [m]cycle (r/w): Cycle counter LOW
            if (cpu_cnt_lo_width_c > 0) and (CPU_EXTENSION_RISCV_Zicntr) then csr.rdata(cpu_cnt_lo_width_c-1 downto 0) <= csr.mcycle(cpu_cnt_lo_width_c-1 downto 0); else NULL; end if;
          when csr_cycleh_c | csr_mcycleh_c => -- [m]cycleh (r/w): Cycle counter HIGH
            if (cpu_cnt_hi_width_c > 0) and (CPU_EXTENSION_RISCV_Zicntr) then csr.rdata(cpu_cnt_hi_width_c-1 downto 0) <= csr.mcycleh(cpu_cnt_hi_width_c-1 downto 0); else NULL; end if;

          when csr_instret_c | csr_minstret_c => -- [m]instret (r/w): Instructions-retired counter LOW
            if (cpu_cnt_lo_width_c > 0) and (CPU_EXTENSION_RISCV_Zicntr) then csr.rdata(cpu_cnt_lo_width_c-1 downto 0) <= csr.minstret(cpu_cnt_lo_width_c-1 downto 0); else NULL; end if;
          when csr_instreth_c | csr_minstreth_c => -- [m]instreth (r/w): Instructions-retired counter HIGH
            if (cpu_cnt_hi_width_c > 0) and (CPU_EXTENSION_RISCV_Zicntr) then csr.rdata(cpu_cnt_hi_width_c-1 downto 0) <= csr.minstreth(cpu_cnt_hi_width_c-1 downto 0); else NULL; end if;

          when csr_time_c => -- time (r/-): System time LOW (from MTIME unit)
            if (CPU_EXTENSION_RISCV_Zicntr) then csr.rdata <= time_i(31 downto 00); else NULL; end if; 
          when csr_timeh_c => -- timeh (r/-): System time HIGH (from MTIME unit)
            if (CPU_EXTENSION_RISCV_Zicntr) then csr.rdata <= time_i(63 downto 32); else NULL; end if; 

          -- hardware performance counters --
          -- --------------------------------------------------------------------
          -- low word (r/w) --
          when csr_mhpmcounter3_c  | csr_hpmcounter3_c  => if (HPM_NUM_CNTS > 00) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(00); else NULL; end if;
          when csr_mhpmcounter4_c  | csr_hpmcounter4_c  => if (HPM_NUM_CNTS > 01) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(01); else NULL; end if;
          when csr_mhpmcounter5_c  | csr_hpmcounter5_c  => if (HPM_NUM_CNTS > 02) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(02); else NULL; end if;
          when csr_mhpmcounter6_c  | csr_hpmcounter6_c  => if (HPM_NUM_CNTS > 03) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(03); else NULL; end if;
          when csr_mhpmcounter7_c  | csr_hpmcounter7_c  => if (HPM_NUM_CNTS > 04) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(04); else NULL; end if;
          when csr_mhpmcounter8_c  | csr_hpmcounter8_c  => if (HPM_NUM_CNTS > 05) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(05); else NULL; end if;
          when csr_mhpmcounter9_c  | csr_hpmcounter9_c  => if (HPM_NUM_CNTS > 06) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(06); else NULL; end if;
          when csr_mhpmcounter10_c | csr_hpmcounter10_c => if (HPM_NUM_CNTS > 07) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(07); else NULL; end if;
          when csr_mhpmcounter11_c | csr_hpmcounter11_c => if (HPM_NUM_CNTS > 08) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(08); else NULL; end if;
          when csr_mhpmcounter12_c | csr_hpmcounter12_c => if (HPM_NUM_CNTS > 09) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(09); else NULL; end if;
          when csr_mhpmcounter13_c | csr_hpmcounter13_c => if (HPM_NUM_CNTS > 10) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(10); else NULL; end if;
          when csr_mhpmcounter14_c | csr_hpmcounter14_c => if (HPM_NUM_CNTS > 11) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(11); else NULL; end if;
          when csr_mhpmcounter15_c | csr_hpmcounter15_c => if (HPM_NUM_CNTS > 12) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(12); else NULL; end if;
          when csr_mhpmcounter16_c | csr_hpmcounter16_c => if (HPM_NUM_CNTS > 13) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(13); else NULL; end if;
          when csr_mhpmcounter17_c | csr_hpmcounter17_c => if (HPM_NUM_CNTS > 14) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(14); else NULL; end if;
          when csr_mhpmcounter18_c | csr_hpmcounter18_c => if (HPM_NUM_CNTS > 15) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(15); else NULL; end if;
          when csr_mhpmcounter19_c | csr_hpmcounter19_c => if (HPM_NUM_CNTS > 16) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(16); else NULL; end if;
          when csr_mhpmcounter20_c | csr_hpmcounter20_c => if (HPM_NUM_CNTS > 17) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(17); else NULL; end if;
          when csr_mhpmcounter21_c | csr_hpmcounter21_c => if (HPM_NUM_CNTS > 18) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(18); else NULL; end if;
          when csr_mhpmcounter22_c | csr_hpmcounter22_c => if (HPM_NUM_CNTS > 19) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(19); else NULL; end if;
          when csr_mhpmcounter23_c | csr_hpmcounter23_c => if (HPM_NUM_CNTS > 20) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(20); else NULL; end if;
          when csr_mhpmcounter24_c | csr_hpmcounter24_c => if (HPM_NUM_CNTS > 21) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(21); else NULL; end if;
          when csr_mhpmcounter25_c | csr_hpmcounter25_c => if (HPM_NUM_CNTS > 22) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(22); else NULL; end if;
          when csr_mhpmcounter26_c | csr_hpmcounter26_c => if (HPM_NUM_CNTS > 23) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(23); else NULL; end if;
          when csr_mhpmcounter27_c | csr_hpmcounter27_c => if (HPM_NUM_CNTS > 24) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(24); else NULL; end if;
          when csr_mhpmcounter28_c | csr_hpmcounter28_c => if (HPM_NUM_CNTS > 25) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(25); else NULL; end if;
          when csr_mhpmcounter29_c | csr_hpmcounter29_c => if (HPM_NUM_CNTS > 26) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(26); else NULL; end if;
          when csr_mhpmcounter30_c | csr_hpmcounter30_c => if (HPM_NUM_CNTS > 27) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(27); else NULL; end if;
          when csr_mhpmcounter31_c | csr_hpmcounter31_c => if (HPM_NUM_CNTS > 28) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounter_rd(28); else NULL; end if;
          -- high word (r/w) --
          when csr_mhpmcounter3h_c  | csr_hpmcounter3h_c  => if (HPM_NUM_CNTS > 00) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(00); else NULL; end if;
          when csr_mhpmcounter4h_c  | csr_hpmcounter4h_c  => if (HPM_NUM_CNTS > 01) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(01); else NULL; end if;
          when csr_mhpmcounter5h_c  | csr_hpmcounter5h_c  => if (HPM_NUM_CNTS > 02) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(02); else NULL; end if;
          when csr_mhpmcounter6h_c  | csr_hpmcounter6h_c  => if (HPM_NUM_CNTS > 03) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(03); else NULL; end if;
          when csr_mhpmcounter7h_c  | csr_hpmcounter7h_c  => if (HPM_NUM_CNTS > 04) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(04); else NULL; end if;
          when csr_mhpmcounter8h_c  | csr_hpmcounter8h_c  => if (HPM_NUM_CNTS > 05) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(05); else NULL; end if;
          when csr_mhpmcounter9h_c  | csr_hpmcounter9h_c  => if (HPM_NUM_CNTS > 06) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(06); else NULL; end if;
          when csr_mhpmcounter10h_c | csr_hpmcounter10h_c => if (HPM_NUM_CNTS > 07) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(07); else NULL; end if;
          when csr_mhpmcounter11h_c | csr_hpmcounter11h_c => if (HPM_NUM_CNTS > 08) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(08); else NULL; end if;
          when csr_mhpmcounter12h_c | csr_hpmcounter12h_c => if (HPM_NUM_CNTS > 09) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(09); else NULL; end if;
          when csr_mhpmcounter13h_c | csr_hpmcounter13h_c => if (HPM_NUM_CNTS > 10) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(10); else NULL; end if;
          when csr_mhpmcounter14h_c | csr_hpmcounter14h_c => if (HPM_NUM_CNTS > 11) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(11); else NULL; end if;
          when csr_mhpmcounter15h_c | csr_hpmcounter15h_c => if (HPM_NUM_CNTS > 12) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(12); else NULL; end if;
          when csr_mhpmcounter16h_c | csr_hpmcounter16h_c => if (HPM_NUM_CNTS > 13) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(13); else NULL; end if;
          when csr_mhpmcounter17h_c | csr_hpmcounter17h_c => if (HPM_NUM_CNTS > 14) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(14); else NULL; end if;
          when csr_mhpmcounter18h_c | csr_hpmcounter18h_c => if (HPM_NUM_CNTS > 15) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(15); else NULL; end if;
          when csr_mhpmcounter19h_c | csr_hpmcounter19h_c => if (HPM_NUM_CNTS > 16) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(16); else NULL; end if;
          when csr_mhpmcounter20h_c | csr_hpmcounter20h_c => if (HPM_NUM_CNTS > 17) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(17); else NULL; end if;
          when csr_mhpmcounter21h_c | csr_hpmcounter21h_c => if (HPM_NUM_CNTS > 18) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(18); else NULL; end if;
          when csr_mhpmcounter22h_c | csr_hpmcounter22h_c => if (HPM_NUM_CNTS > 19) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(19); else NULL; end if;
          when csr_mhpmcounter23h_c | csr_hpmcounter23h_c => if (HPM_NUM_CNTS > 20) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(20); else NULL; end if;
          when csr_mhpmcounter24h_c | csr_hpmcounter24h_c => if (HPM_NUM_CNTS > 21) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(21); else NULL; end if;
          when csr_mhpmcounter25h_c | csr_hpmcounter25h_c => if (HPM_NUM_CNTS > 22) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(22); else NULL; end if;
          when csr_mhpmcounter26h_c | csr_hpmcounter26h_c => if (HPM_NUM_CNTS > 23) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(23); else NULL; end if;
          when csr_mhpmcounter27h_c | csr_hpmcounter27h_c => if (HPM_NUM_CNTS > 24) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(24); else NULL; end if;
          when csr_mhpmcounter28h_c | csr_hpmcounter28h_c => if (HPM_NUM_CNTS > 25) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(25); else NULL; end if;
          when csr_mhpmcounter29h_c | csr_hpmcounter29h_c => if (HPM_NUM_CNTS > 26) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(26); else NULL; end if;
          when csr_mhpmcounter30h_c | csr_hpmcounter30h_c => if (HPM_NUM_CNTS > 27) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(27); else NULL; end if;
          when csr_mhpmcounter31h_c | csr_hpmcounter31h_c => if (HPM_NUM_CNTS > 28) and (CPU_EXTENSION_RISCV_Zihpm) then csr.rdata <= csr.mhpmcounterh_rd(28); else NULL; end if;

          -- machine information registers --
          -- --------------------------------------------------------------------
--        when csr_mvendorid_c  => csr.rdata <= (others => '0'); -- mvendorid (r/-): vendor ID, implemented but always zero
          when csr_marchid_c    => csr.rdata(4 downto 0) <= "10011"; -- marchid (r/-): arch ID - official RISC-V open-source arch ID
          when csr_mimpid_c     => csr.rdata <= hw_version_c; -- mimpid (r/-): implementation ID -- NEORV32 hardware version
          when csr_mhartid_c    => csr.rdata <= std_ulogic_vector(to_unsigned(HW_THREAD_ID, 32)); -- mhartid (r/-): hardware thread ID
--        when csr_mconfigptr_c => csr.rdata <= (others => '0'); -- mconfigptr (r/-): machine configuration pointer register, implemented but always zero

          -- debug mode CSRs --
          -- --------------------------------------------------------------------
          when csr_dcsr_c      => if (CPU_EXTENSION_RISCV_DEBUG) then csr.rdata <= csr.dcsr_rd;   else NULL; end if; -- dcsr (r/w): debug mode control and status
          when csr_dpc_c       => if (CPU_EXTENSION_RISCV_DEBUG) then csr.rdata <= csr.dpc;       else NULL; end if; -- dpc (r/w): debug mode program counter
          when csr_dscratch0_c => if (CPU_EXTENSION_RISCV_DEBUG) then csr.rdata <= csr.dscratch0; else NULL; end if; -- dscratch0 (r/w): debug mode scratch register 0

          -- trigger module CSRs --
          -- --------------------------------------------------------------------
--        when csr_tselect_c  => if (CPU_EXTENSION_RISCV_DEBUG) then csr.rdata <= (others => '0'); else NULL; end if; -- tselect (r/w): always zero = only 1 trigger available
          when csr_tdata1_c   => if (CPU_EXTENSION_RISCV_DEBUG) then csr.rdata <= csr.tdata1_rd;   else NULL; end if; -- tdata1 (r/w): match control
          when csr_tdata2_c   => if (CPU_EXTENSION_RISCV_DEBUG) then csr.rdata <= csr.tdata2;      else NULL; end if; -- tdata2 (r/w): address-compare
--        when csr_tdata3_c   => if (CPU_EXTENSION_RISCV_DEBUG) then csr.rdata <= (others => '0'); else NULL; end if; -- tdata3 (r/w): implemented but always zero
          when csr_tinfo_c    => if (CPU_EXTENSION_RISCV_DEBUG) then csr.rdata <= x"00000004";     else NULL; end if; -- tinfo (r/w): address-match trigger only
--        when csr_tcontrol_c => if (CPU_EXTENSION_RISCV_DEBUG) then csr.rdata <= (others => '0'); else NULL; end if; -- tcontrol (r/w): implemented but always zero
--        when csr_mcontext_c => if (CPU_EXTENSION_RISCV_DEBUG) then csr.rdata <= (others => '0'); else NULL; end if; -- mcontext (r/w): implemented but always zero
--        when csr_scontext_c => if (CPU_EXTENSION_RISCV_DEBUG) then csr.rdata <= (others => '0'); else NULL; end if; -- scontext (r/w): implemented but always zero

          -- NEORV32-specific (RISC-V "custom") read-only CSRs --
          -- --------------------------------------------------------------------
          -- machine extended ISA extensions information --
          when csr_mxisa_c =>
            -- ISA extended (sub-)extensions --
            csr.rdata(00) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicsr);    -- Zicsr: privileged architecture (!!!)
            csr.rdata(01) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zifencei); -- Zifencei: instruction stream sync.
            csr.rdata(02) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zmmul);    -- Zmmul: mul/div
            csr.rdata(03) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zxcfu);    -- Zxcfu: custom RISC-V instructions
            csr.rdata(04) <= '0'; -- reserved
            csr.rdata(05) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zfinx);    -- Zfinx: FPU using x registers, "F-alternative"
            csr.rdata(06) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicntr) and
                             bool_to_ulogic_f(boolean(CPU_CNT_WIDTH /= 64)); -- Zxscnt: reduced-size CPU counters (from Zicntr)
            csr.rdata(07) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicntr);   -- Zicntr: base instructions, cycle and time CSRs
            csr.rdata(08) <= bool_to_ulogic_f(boolean(PMP_NUM_REGIONS > 0)); -- PMP: physical memory protection (Zspmp)
            csr.rdata(09) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zihpm);    -- Zihpm: hardware performance monitors
            csr.rdata(10) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_DEBUG);    -- RISC-V debug mode
            -- tuning options --
            csr.rdata(30) <= bool_to_ulogic_f(FAST_MUL_EN);                  -- DSP-based multiplication (M extensions only)
            csr.rdata(31) <= bool_to_ulogic_f(FAST_SHIFT_EN);                -- parallel logic for shifts (barrel shifters)

          -- undefined/unavailable --
          -- --------------------------------------------------------------------
          when others =>
            NULL; -- not implemented, read as zero

        end case;
      end if;
    end if;
  end process csr_read_access;

  -- CSR read data output --
  csr_rdata_o <= csr.rdata;
  

-- ****************************************************************************************************************************
-- CPU Counters / HPMs (CSRs)
-- ****************************************************************************************************************************

  -- Control and Status Registers - Counters ------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_counters: process(rstn_i, clk_i)
  begin
    -- Counter CSRs (each counter is split into two 32-bit counters - coupled via an MSB overflow FF)
    if (rstn_i = '0') then
      csr.mcycle           <= (others => def_rst_val_c);
      csr.mcycle_ovfl      <= (others => def_rst_val_c);
      csr.mcycleh          <= (others => def_rst_val_c);
      csr.minstret         <= (others => def_rst_val_c);
      csr.minstret_ovfl    <= (others => def_rst_val_c);
      csr.minstreth        <= (others => def_rst_val_c);
      csr.mhpmcounter      <= (others => (others => def_rst_val_c));
      csr.mhpmcounter_ovfl <= (others => (others => def_rst_val_c));
      csr.mhpmcounterh     <= (others => (others => def_rst_val_c));
    elsif rising_edge(clk_i) then

      -- [m]cycle --
      if (cpu_cnt_lo_width_c > 0) and (CPU_EXTENSION_RISCV_Zicntr = true) then
        csr.mcycle_ovfl(0) <= csr.mcycle_nxt(csr.mcycle_nxt'left) and (not csr.mcountinhibit_cy);
        if (csr.we = '1') and (csr.addr = csr_mcycle_c) then -- write access
          csr.mcycle(cpu_cnt_lo_width_c-1 downto 0) <= csr.wdata(cpu_cnt_lo_width_c-1 downto 0);
        elsif (csr.mcountinhibit_cy = '0') and (cnt_event(hpmcnt_event_cy_c) = '1') and (debug_ctrl.running = '0') then -- non-inhibited automatic update and not in debug mode
          csr.mcycle(cpu_cnt_lo_width_c-1 downto 0) <= csr.mcycle_nxt(cpu_cnt_lo_width_c-1 downto 0);
        end if;
      else
        csr.mcycle_ovfl <= (others => '-');
        csr.mcycle      <= (others => '-');
      end if;

      -- [m]cycleh --
      if (cpu_cnt_hi_width_c > 0) and (CPU_EXTENSION_RISCV_Zicntr = true) then
        if (csr.we = '1') and (csr.addr = csr_mcycleh_c) then -- write access
          csr.mcycleh(cpu_cnt_hi_width_c-1 downto 0) <= csr.wdata(cpu_cnt_hi_width_c-1 downto 0);
        else -- automatic update
          csr.mcycleh(cpu_cnt_hi_width_c-1 downto 0) <= std_ulogic_vector(unsigned(csr.mcycleh(cpu_cnt_hi_width_c-1 downto 0)) + unsigned(csr.mcycle_ovfl));
        end if;
      else
        csr.mcycleh <= (others => '-');
      end if;


      -- [m]instret --
      if (cpu_cnt_lo_width_c > 0) and (CPU_EXTENSION_RISCV_Zicntr = true) then
        csr.minstret_ovfl(0) <= csr.minstret_nxt(csr.minstret_nxt'left) and (not csr.mcountinhibit_ir);
        if (csr.we = '1') and (csr.addr = csr_minstret_c) then -- write access
          csr.minstret(cpu_cnt_lo_width_c-1 downto 0) <= csr.wdata(cpu_cnt_lo_width_c-1 downto 0);
        elsif (csr.mcountinhibit_ir = '0') and (cnt_event(hpmcnt_event_ir_c) = '1') and (debug_ctrl.running = '0') then -- non-inhibited automatic update and not in debug mode
          csr.minstret(cpu_cnt_lo_width_c-1 downto 0) <= csr.minstret_nxt(cpu_cnt_lo_width_c-1 downto 0);
        end if;
      else
        csr.minstret_ovfl <= (others => '-');
        csr.minstret      <= (others => '-');
      end if;

      -- [m]instreth --
      if (cpu_cnt_hi_width_c > 0) and (CPU_EXTENSION_RISCV_Zicntr = true) then
        if (csr.we = '1') and (csr.addr = csr_minstreth_c) then -- write access
          csr.minstreth(cpu_cnt_hi_width_c-1 downto 0) <= csr.wdata(cpu_cnt_hi_width_c-1 downto 0);
        else -- automatic update
          csr.minstreth(cpu_cnt_hi_width_c-1 downto 0) <= std_ulogic_vector(unsigned(csr.minstreth(cpu_cnt_hi_width_c-1 downto 0)) + unsigned(csr.minstret_ovfl));
        end if;
      else
        csr.minstreth <= (others => '-');
      end if;


      -- [machine] hardware performance monitors (counters) --
      for i in 0 to HPM_NUM_CNTS-1 loop

        -- [m]hpmcounter* --
        if (hpm_cnt_lo_width_c > 0) and (CPU_EXTENSION_RISCV_Zihpm = true) then
          csr.mhpmcounter_ovfl(i)(0) <= csr.mhpmcounter_nxt(i)(csr.mhpmcounter_nxt(i)'left) and (not csr.mcountinhibit_hpm(i));
          if (csr.we = '1') and (csr.addr = std_ulogic_vector(unsigned(csr_mhpmcounter3_c) + i)) then -- write access
            csr.mhpmcounter(i)(hpm_cnt_lo_width_c-1 downto 0) <= csr.wdata(hpm_cnt_lo_width_c-1 downto 0);
          elsif (csr.mcountinhibit_hpm(i) = '0') and (hpmcnt_trigger(i) = '1') then -- non-inhibited automatic update
            csr.mhpmcounter(i)(hpm_cnt_lo_width_c-1 downto 0) <= csr.mhpmcounter_nxt(i)(hpm_cnt_lo_width_c-1 downto 0);
          end if;
        else
          csr.mhpmcounter_ovfl(i) <= (others => '-');
          csr.mhpmcounter(i)      <= (others => '-');
        end if;

        -- [m]hpmcounter*h --
        if (hpm_cnt_hi_width_c > 0) and (CPU_EXTENSION_RISCV_Zihpm = true) then
          if (csr.we = '1') and (csr.addr = std_ulogic_vector(unsigned(csr_mhpmcounter3h_c) + i)) then -- write access
            csr.mhpmcounterh(i)(hpm_cnt_hi_width_c-1 downto 0) <= csr.wdata(hpm_cnt_hi_width_c-1 downto 0);
          else -- automatic update
            csr.mhpmcounterh(i)(hpm_cnt_hi_width_c-1 downto 0) <= std_ulogic_vector(unsigned(csr.mhpmcounterh(i)(hpm_cnt_hi_width_c-1 downto 0)) + unsigned(csr.mhpmcounter_ovfl(i)));
          end if;
        else
          csr.mhpmcounterh(i) <= (others => '-');
        end if;

      end loop; -- i

    end if;
  end process csr_counters;

  -- mcycle & minstret increment LOW --
  csr.mcycle_nxt   <= std_ulogic_vector(unsigned('0' & csr.mcycle)   + 1);
  csr.minstret_nxt <= std_ulogic_vector(unsigned('0' & csr.minstret) + 1);

  -- hpm counter increment LOW --
  hmp_cnt_lo_inc:
  for i in 0 to HPM_NUM_CNTS-1 generate
    csr.mhpmcounter_nxt(i) <= std_ulogic_vector(unsigned('0' & csr.mhpmcounter(i)) + 1);
  end generate;


  -- hpm counter read --
  hpm_rd_dummy: process(csr)
  begin
    csr.mhpmevent_rd    <= (others => (others => '0'));
    csr.mhpmcounter_rd  <= (others => (others => '0'));
    csr.mhpmcounterh_rd <= (others => (others => '0'));
    if (HPM_NUM_CNTS /= 0) and (CPU_EXTENSION_RISCV_Zihpm = true) then
      for i in 0 to HPM_NUM_CNTS-1 loop
        csr.mhpmevent_rd(i)(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(i);
        if (hpm_cnt_lo_width_c > 0) then
          csr.mhpmcounter_rd(i)(hpm_cnt_lo_width_c-1 downto 0) <= csr.mhpmcounter(i)(hpm_cnt_lo_width_c-1 downto 0);
        end if;
        if (hpm_cnt_hi_width_c > 0) then
          csr.mhpmcounterh_rd(i)(hpm_cnt_hi_width_c-1 downto 0) <= csr.mhpmcounterh(i)(hpm_cnt_hi_width_c-1 downto 0);
        end if;
      end loop; -- i
    end if;
  end process hpm_rd_dummy;


  -- Hardware Performance Monitor - Counter Event Control -----------------------------------
  -- -------------------------------------------------------------------------------------------
  hpmcnt_ctrl: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- enable selected triggers by ANDing actual events and according CSR configuration bits --
      -- OR everything to see if counter should increment --
      hpmcnt_trigger <= (others => '0'); -- default
      if (HPM_NUM_CNTS /= 0) then
        for i in 0 to HPM_NUM_CNTS-1 loop
          -- do not increment if CPU is in debug mode --
          hpmcnt_trigger(i) <= or_reduce_f(cnt_event and csr.mhpmevent(i)(cnt_event'left downto 0)) and (not debug_ctrl.running);
        end loop; -- i
      end if;
    end if;
  end process hpmcnt_ctrl;

  -- counter event trigger - RISC-V-specific --
  cnt_event(hpmcnt_event_cy_c)      <= not execute_engine.sleep; -- active cycle
  cnt_event(hpmcnt_event_never_c)   <= '0'; -- "never"
  cnt_event(hpmcnt_event_ir_c)      <= '1' when (execute_engine.state = EXECUTE) else '0'; -- (any) retired instruction

  -- counter event trigger - custom / NEORV32-specific --
  cnt_event(hpmcnt_event_cir_c)     <= '1' when (execute_engine.state = EXECUTE)      and (execute_engine.is_ci = '1')             else '0'; -- retired compressed instruction
  cnt_event(hpmcnt_event_wait_if_c) <= '1' when (fetch_engine.state   = IFETCH_ISSUE) and (fetch_engine.state_prev = IFETCH_ISSUE) else '0'; -- instruction fetch memory wait cycle
  cnt_event(hpmcnt_event_wait_ii_c) <= '1' when (execute_engine.state = DISPATCH)     and (execute_engine.state_prev = DISPATCH)   else '0'; -- instruction issue wait cycle
  cnt_event(hpmcnt_event_wait_mc_c) <= '1' when (execute_engine.state = ALU_WAIT)                                                  else '0'; -- multi-cycle alu-operation wait cycle

  cnt_event(hpmcnt_event_load_c)    <= '1' when                                          (ctrl(ctrl_bus_rd_c) = '1')               else '0'; -- load operation
  cnt_event(hpmcnt_event_store_c)   <= '1' when                                          (ctrl(ctrl_bus_wr_c) = '1')               else '0'; -- store operation
  cnt_event(hpmcnt_event_wait_ls_c) <= '1' when (execute_engine.state = LOADSTORE_2) and (execute_engine.state_prev = LOADSTORE_2) else '0'; -- load/store memory wait cycle

  cnt_event(hpmcnt_event_jump_c)    <= '1' when (execute_engine.state = BRANCH) and (execute_engine.i_reg(instr_opcode_lsb_c+2) = '1') else '0'; -- jump (unconditional)
  cnt_event(hpmcnt_event_branch_c)  <= '1' when (execute_engine.state = BRANCH) and (execute_engine.i_reg(instr_opcode_lsb_c+2) = '0') else '0'; -- branch (conditional, taken or not taken)
  cnt_event(hpmcnt_event_tbranch_c) <= '1' when (execute_engine.state = BRANCH) and (execute_engine.i_reg(instr_opcode_lsb_c+2) = '0') and (execute_engine.branch_taken = '1') else '0'; -- taken branch (conditional)

  cnt_event(hpmcnt_event_trap_c)    <= '1' when (trap_ctrl.env_start_ack = '1')                                    else '0'; -- entered trap
  cnt_event(hpmcnt_event_illegal_c) <= '1' when (trap_ctrl.env_start_ack = '1') and (trap_ctrl.cause = trap_iil_c) else '0'; -- illegal operation


-- ****************************************************************************************************************************
-- CPU Debug Mode (Part of the On-Chip Debugger)
-- ****************************************************************************************************************************

  -- Debug Control --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  debug_control: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      debug_ctrl.state <= DEBUG_OFFLINE;
      debug_ctrl.ext_halt_req <= '0';
    elsif rising_edge(clk_i) then
      if (CPU_EXTENSION_RISCV_DEBUG = true) then

        -- external halt request (from Debug Module) --
        debug_ctrl.ext_halt_req <= db_halt_req_i;

        -- state machine --
        case debug_ctrl.state is

          when DEBUG_OFFLINE => -- not in debug mode, waiting for entering request
            if (debug_ctrl.trig_halt = '1') or   -- external request (from DM)
               (debug_ctrl.trig_break = '1') or  -- ebreak instruction
               (debug_ctrl.trig_hw = '1') or     -- hardware trigger module
               (debug_ctrl.trig_step = '1') then -- single-stepping mode
              debug_ctrl.state <= DEBUG_PENDING;
            end if;

          when DEBUG_PENDING => -- waiting to start debug mode
            if (trap_ctrl.env_start_ack = '1') and (trap_ctrl.cause(5) = '1') then -- processing trap entry into debug mode
              debug_ctrl.state <= DEBUG_ONLINE;
            end if;

          when DEBUG_ONLINE => -- we are in debug mode
            if (debug_ctrl.dret = '1') then -- DRET instruction
              debug_ctrl.state <= DEBUG_EXIT;
            end if;

          when DEBUG_EXIT => -- leaving debug mode
            if (execute_engine.state = TRAP_EXECUTE) then -- processing trap exit
              debug_ctrl.state <= DEBUG_OFFLINE;
            end if;

          when others => -- undefined
            debug_ctrl.state <= DEBUG_OFFLINE;

        end case;
      else -- debug mode NOT implemented
        debug_ctrl.state <= DEBUG_OFFLINE;
        debug_ctrl.ext_halt_req <= '0';
      end if;
    end if;
  end process debug_control;

  -- state decoding --
  debug_ctrl.pending <= '1' when (debug_ctrl.state = DEBUG_PENDING) and (CPU_EXTENSION_RISCV_DEBUG = true) else '0';
  debug_ctrl.running <= '1' when ((debug_ctrl.state = DEBUG_ONLINE) or (debug_ctrl.state = DEBUG_EXIT)) and (CPU_EXTENSION_RISCV_DEBUG = true) else '0';

  -- entry debug mode triggers --
  debug_ctrl.trig_hw    <= hw_trigger_fire and (not debug_ctrl.running); -- enter debug mode by HW trigger module request
  debug_ctrl.trig_break <= trap_ctrl.break_point and (debug_ctrl.running or -- re-enter debug mode
                           ((    csr.privilege) and csr.dcsr_ebreakm) or -- enabled goto-debug-mode in machine mode on "ebreak"
                           ((not csr.privilege) and csr.dcsr_ebreaku));  -- enabled goto-debug-mode in user mode on "ebreak"
  debug_ctrl.trig_halt  <= debug_ctrl.ext_halt_req and (not debug_ctrl.running); -- external halt request (if not halted already)
  debug_ctrl.trig_step  <= csr.dcsr_step and (not debug_ctrl.running); -- single-step mode (trigger when NOT CURRENTLY in debug mode)


  -- Debug Control and Status Register (dcsr) - Read-Back -----------------------------------
  -- -------------------------------------------------------------------------------------------
  csr.dcsr_rd(31 downto 28) <= "0100"; -- xdebugver: external debug support compatible to spec
  csr.dcsr_rd(27 downto 16) <= (others => '0'); -- reserved
  csr.dcsr_rd(15)           <= csr.dcsr_ebreakm; -- ebreakm: what happens on ebreak in m-mode? (normal trap OR debug-enter)
  csr.dcsr_rd(14)           <= '0'; -- ebreakh: hypervisor mode not implemented
  csr.dcsr_rd(13)           <= '0'; -- ebreaks: supervisor mode not implemented
  csr.dcsr_rd(12)           <= csr.dcsr_ebreaku when (CPU_EXTENSION_RISCV_U = true) else '0'; -- ebreaku: what happens on ebreak in u-mode? (normal trap OR debug-enter)
  csr.dcsr_rd(11)           <= '0'; -- stepie: interrupts are disabled during single-stepping
  csr.dcsr_rd(10)           <= '1'; -- stopcount: standard counters and HPMs are stopped when in debug mode
  csr.dcsr_rd(09)           <= '0'; -- stoptime: timers increment as usual
  csr.dcsr_rd(08 downto 06) <= csr.dcsr_cause; -- debug mode entry cause
  csr.dcsr_rd(05)           <= '0'; -- reserved
  csr.dcsr_rd(04)           <= '0'; -- mprven: mstatus.mprv is ignored in debug mode
  csr.dcsr_rd(03)           <= '0'; -- nmip: no pending non-maskable interrupt
  csr.dcsr_rd(02)           <= csr.dcsr_step; -- step: single-step mode
  csr.dcsr_rd(01 downto 00) <= (others => csr.dcsr_prv); -- prv: privilege mode when debug mode was entered


-- ****************************************************************************************************************************
-- Hardware Trigger Module (Part of the On-Chip Debugger)
-- ****************************************************************************************************************************

  -- trigger to enter debug-mode: instruction address match (fire AFTER execution) --
  hw_trigger_fire <= '1' when (CPU_EXTENSION_RISCV_DEBUG = true) and (csr.tdata1_exe = '1') and
                              (csr.tdata2(data_width_c-1 downto 1) = execute_engine.pc(data_width_c-1 downto 1)) else '0';


  -- Match Control CSR (mcontrol @ tdata1) - Read-Back --------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr.tdata1_rd(31 downto 28) <= "0010"; -- type: address(/data) match trigger
  csr.tdata1_rd(27)           <= '1'; -- dmode: only debug-mode can write tdata* registers
  csr.tdata1_rd(26 downto 21) <= "000000"; -- maskmax: only exact values
  csr.tdata1_rd(20)           <= '0'; -- hit: feature not implemented
  csr.tdata1_rd(19)           <= '0'; -- select: fire on address match
  csr.tdata1_rd(18)           <= '1'; -- timing: trigger **after** executing the triggering instruction
  csr.tdata1_rd(17 downto 16) <= "00"; -- sizelo: match against an access of any size
  csr.tdata1_rd(15 downto 12) <= "0001"; -- action: enter debug mode on trigger
  csr.tdata1_rd(11)           <= '0'; -- chain: chaining not supported - there is only one trigger
  csr.tdata1_rd(10 downto 07) <= "0000"; -- match: only full-address-match
  csr.tdata1_rd(6)            <= '1'; -- m: trigger enabled when in machine mode
  csr.tdata1_rd(5)            <= '0'; -- h: hypervisor mode not supported
  csr.tdata1_rd(4)            <= '0'; -- s: supervisor mode not supported
  csr.tdata1_rd(3)            <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_U); -- u: trigger enabled when in user mode
  csr.tdata1_rd(2)            <= csr.tdata1_exe; -- execute: enable trigger
  csr.tdata1_rd(1)            <= '0'; -- store: store address or data matching not supported
  csr.tdata1_rd(0)            <= '0'; -- load: load address or data matching not supported


end neorv32_cpu_control_rtl;
