-- #################################################################################################
-- # << NEORV32 - CPU Control >>                                                                   #
-- # ********************************************************************************************* #
-- # CPU operation is split into a fetch engine (responsible for fetching instruction data), an    #
-- # issue engine (for recoding compressed instructions and for constructing 32-bit instruction    #
-- # words) and an execute engine (responsible for actually executing the instructions), a trap    #
-- # handling controller and the RISC-V status and control register set (CSRs) including the       #
-- # hardware performance monitor counters.                                                        #
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
    CPU_EXTENSION_RISCV_DEBUG    : boolean; -- implement CPU debug mode?
    -- Extension Options --
    CPU_CNT_WIDTH                : natural; -- total width of CPU cycle and instret counters (0..64)
    CPU_IPB_ENTRIES              : natural; -- entries is instruction prefetch buffer, has to be a power of 2
    -- Physical memory protection (PMP) --
    PMP_NUM_REGIONS              : natural; -- number of regions (0..64)
    PMP_MIN_GRANULARITY          : natural; -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
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
    bus_err_ack : std_ulogic;
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
  type issue_engine_state_t is (ISSUE_ACTIVE, ISSUE_REALIGN);
  type issue_engine_t is record
    state     : issue_engine_state_t;
    state_nxt : issue_engine_state_t;
    align     : std_ulogic;
    align_nxt : std_ulogic;
    buf       : std_ulogic_vector(2+15 downto 0);
    buf_nxt   : std_ulogic_vector(2+15 downto 0);
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
    is_atomic_lr    : std_ulogic;
    is_atomic_sc    : std_ulogic;
    is_float_op     : std_ulogic;
    sys_env_cmd     : std_ulogic_vector(11 downto 0);
    is_m_mul        : std_ulogic;
    is_m_div        : std_ulogic;
    is_bitmanip_imm : std_ulogic;
    is_bitmanip_reg : std_ulogic;
    rs1_zero        : std_ulogic;
    rs2_zero        : std_ulogic;
    rd_zero         : std_ulogic;
  end record;
  signal decode_aux : decode_aux_t;

  -- instruction execution engine --
  type execute_engine_state_t is (SYS_WAIT, DISPATCH, TRAP_ENTER, TRAP_EXIT, TRAP_EXECUTE, EXECUTE, ALU_WAIT,
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
    last_pc      : std_ulogic_vector(data_width_c-1 downto 0); -- PC of last executed instruction
    --
    sleep        : std_ulogic; -- CPU in sleep mode
    sleep_nxt    : std_ulogic;
    branched     : std_ulogic; -- instruction fetch was reset
    branched_nxt : std_ulogic;
  end record;
  signal execute_engine : execute_engine_t;

  -- trap controller --
  type trap_ctrl_t is record
    exc_buf       : std_ulogic_vector(exception_width_c-1 downto 0);
    exc_fire      : std_ulogic; -- set if there is a valid source in the exception buffer
    irq_buf       : std_ulogic_vector(interrupt_width_c-1 downto 0);
    irq_fire      : std_ulogic; -- set if there is a valid source in the interrupt buffer
    exc_ack       : std_ulogic; -- acknowledge all exceptions
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
    env_call      : std_ulogic;
    break_point   : std_ulogic;
  end record;
  signal trap_ctrl : trap_ctrl_t;
  
  -- CPU main control bus --
  signal ctrl_nxt, ctrl : std_ulogic_vector(ctrl_width_c-1 downto 0);

  -- fast instruction fetch access --
  signal bus_fast_ir : std_ulogic;

  -- RISC-V control and status registers (CSRs) --
  type pmp_ctrl_t     is array (0 to PMP_NUM_REGIONS-1) of std_ulogic_vector(7 downto 0);
  type pmp_addr_t     is array (0 to PMP_NUM_REGIONS-1) of std_ulogic_vector(data_width_c-1 downto 0);
  type pmp_ctrl_rd_t  is array (0 to 63) of std_ulogic_vector(7 downto 0);
  type mhpmevent_t    is array (0 to HPM_NUM_CNTS-1) of std_ulogic_vector(hpmcnt_event_size_c-1 downto 0);
  type mhpmcnt_t      is array (0 to HPM_NUM_CNTS-1) of std_ulogic_vector(31 downto 0);
  type mhpmcnt_nxt_t  is array (0 to HPM_NUM_CNTS-1) of std_ulogic_vector(32 downto 0);
  type mhpmcnt_ovfl_t is array (0 to HPM_NUM_CNTS-1) of std_ulogic_vector(0 downto 0);
  type mhpmcnt_rd_t   is array (0 to 29) of std_ulogic_vector(31 downto 0);
  type csr_t is record
    addr              : std_ulogic_vector(11 downto 0); -- csr address
    we                : std_ulogic; -- csr write enable
    we_nxt            : std_ulogic;
    wdata             : std_ulogic_vector(data_width_c-1 downto 0); -- csr write data
    rdata             : std_ulogic_vector(data_width_c-1 downto 0); -- csr read data
    --
    mstatus_mie       : std_ulogic; -- mstatus.MIE: global IRQ enable (R/W)
    mstatus_mpie      : std_ulogic; -- mstatus.MPIE: previous global IRQ enable (R/W)
    mstatus_mpp       : std_ulogic_vector(1 downto 0); -- mstatus.MPP: machine previous privilege mode
    --
    mie_msie          : std_ulogic; -- mie.MSIE: machine software interrupt enable (R/W)
    mie_meie          : std_ulogic; -- mie.MEIE: machine external interrupt enable (R/W)
    mie_mtie          : std_ulogic; -- mie.MEIE: machine timer interrupt enable (R/W)
    mie_firqe         : std_ulogic_vector(15 downto 0); -- mie.firq*e: fast interrupt enabled (R/W)
    --
    mip_clr           : std_ulogic_vector(15 downto 0); -- clear pending FIRQ
    --
    mcounteren_cy     : std_ulogic; -- mcounteren.cy: allow cycle[h] access from user-mode
    mcounteren_tm     : std_ulogic; -- mcounteren.tm: allow time[h] access from user-mode
    mcounteren_ir     : std_ulogic; -- mcounteren.ir: allow instret[h] access from user-mode
    --
    mcountinhibit_cy  : std_ulogic; -- mcounterinhibit.cy: enable auto-increment for [m]cycle[h]
    mcountinhibit_ir  : std_ulogic; -- mcounterinhibit.ir: enable auto-increment for [m]instret[h]
    mcountinhibit_hpm : std_ulogic_vector(HPM_NUM_CNTS-1 downto 0); -- mcounterinhibit.hpm3: enable auto-increment for mhpmcounterx[h]
    --
    privilege         : std_ulogic_vector(1 downto 0); -- hart's current privilege mode
    privilege_rd      : std_ulogic_vector(1 downto 0); -- hart's current privilege mode (effective)
    priv_m_mode       : std_ulogic; -- CPU in M-mode
    priv_u_mode       : std_ulogic; -- CPU in u-mode
    --
    mepc              : std_ulogic_vector(data_width_c-1 downto 0); -- mepc: machine exception pc (R/W)
    mcause            : std_ulogic_vector(5 downto 0); -- mcause: machine trap cause (R/W)
    mtvec             : std_ulogic_vector(data_width_c-1 downto 0); -- mtvec: machine trap-handler base address (R/W), bit 1:0 == 00
    mtval             : std_ulogic_vector(data_width_c-1 downto 0); -- mtval: machine bad address or instruction (R/W)
    --
    mhpmevent         : mhpmevent_t; -- mhpmevent*: machine performance-monitoring event selector (R/W)
    --
    mscratch          : std_ulogic_vector(data_width_c-1 downto 0); -- mscratch: scratch register (R/W)
    --
    mcycle            : std_ulogic_vector(31 downto 0); -- mcycle (R/W)
    mcycle_nxt        : std_ulogic_vector(32 downto 0);
    mcycle_ovfl       : std_ulogic_vector(00 downto 0); -- counter low-to-high-word overflow
    mcycleh           : std_ulogic_vector(31 downto 0); -- mcycleh (R/W)
    minstret          : std_ulogic_vector(31 downto 0); -- minstret (R/W)
    minstret_nxt      : std_ulogic_vector(32 downto 0);
    minstret_ovfl     : std_ulogic_vector(00 downto 0); -- counter low-to-high-word overflow
    minstreth         : std_ulogic_vector(31 downto 0); -- minstreth (R/W)
    --
    mhpmcounter       : mhpmcnt_t; -- mhpmcounter* (R/W), plus carry bit
    mhpmcounter_nxt   : mhpmcnt_nxt_t;
    mhpmcounter_ovfl  : mhpmcnt_ovfl_t; -- counter low-to-high-word overflow
    mhpmcounterh      : mhpmcnt_t; -- mhpmcounter*h (R/W)
    mhpmcounter_rd    : mhpmcnt_rd_t; -- mhpmcounter* (R/W): actual read data
    mhpmcounterh_rd   : mhpmcnt_rd_t; -- mhpmcounter*h (R/W): actual read data
    --
    pmpcfg            : pmp_ctrl_t; -- physical memory protection - configuration registers
    pmpcfg_rd         : pmp_ctrl_rd_t; -- physical memory protection - actual read data
    pmpaddr           : pmp_addr_t; -- physical memory protection - address registers
    --
    frm               : std_ulogic_vector(02 downto 0); -- frm (R/W): FPU rounding mode
    fflags            : std_ulogic_vector(04 downto 0); -- fflags (R/W): FPU exception flags
    --
    dcsr_ebreakm      : std_ulogic; -- dcsr.ebreakm (R/W): behavior of ebreak instruction on m-mode
    dcsr_ebreaku      : std_ulogic; -- dcsr.ebreaku (R/W): behavior of ebreak instruction on u-mode
    dcsr_step         : std_ulogic; -- dcsr.step (R/W): single-step mode
    dcsr_prv          : std_ulogic_vector(01 downto 0); -- dcsr.prv (R/W): current privilege level when entering debug mode
    dcsr_cause        : std_ulogic_vector(02 downto 0); -- dcsr.cause (R/-): why was debug mode entered
    dcsr_rd           : std_ulogic_vector(data_width_c-1 downto 0); -- dcsr (R/(W)): debug mode control and status register
    dpc               : std_ulogic_vector(data_width_c-1 downto 0); -- dpc (R/W): debug mode program counter
    dscratch0         : std_ulogic_vector(data_width_c-1 downto 0); -- dscratch0 (R/W): debug mode scratch register 0
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
      fetch_engine.restart    <= fetch_engine.restart_nxt;
      if (fetch_engine.restart = '1') then
        fetch_engine.pc <= execute_engine.pc(data_width_c-1 downto 1) & '0'; -- initialize with "real" application PC
      else
        fetch_engine.pc <= fetch_engine.pc_nxt;
      end if;
    end if;
  end process fetch_engine_fsm_sync;

  -- PC output --
  fetch_pc_o <= fetch_engine.pc(data_width_c-1 downto 1) & '0'; -- half-word aligned


  -- Fetch Engine FSM Comb ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  fetch_engine_fsm_comb: process(fetch_engine, execute_engine, ipb, instr_i, bus_i_wait_i, be_instr_i, ma_instr_i)
  begin
    -- arbiter defaults --
    bus_fast_ir              <= '0';
    fetch_engine.state_nxt   <= fetch_engine.state;
    fetch_engine.pc_nxt      <= fetch_engine.pc;
    fetch_engine.bus_err_ack <= '0';
    fetch_engine.restart_nxt <= fetch_engine.restart or fetch_engine.reset;

    -- instruction prefetch buffer interface --
    ipb.we    <= '0';
    ipb.wdata <= be_instr_i & ma_instr_i & instr_i(31 downto 0); -- store exception info and instruction word
    ipb.clear <= fetch_engine.restart;

    -- state machine --
    case fetch_engine.state is

      when IFETCH_REQUEST => -- request new 32-bit-aligned instruction word
      -- ------------------------------------------------------------
        if (ipb.free = '1') and (fetch_engine.restart = '0') then -- free entry in buffer AND no reset request?
          bus_fast_ir            <= '1'; -- fast instruction fetch request
          fetch_engine.state_nxt <= IFETCH_ISSUE;
        end if;
        if (fetch_engine.restart = '1') then -- reset request?
          fetch_engine.restart_nxt <= '0';
        end if;

      when IFETCH_ISSUE => -- store instruction data to prefetch buffer
      -- ------------------------------------------------------------
        fetch_engine.bus_err_ack <= be_instr_i or ma_instr_i; -- ACK bus/alignment errors
        if (bus_i_wait_i = '0') or (be_instr_i = '1') or (ma_instr_i = '1') then -- wait for bus response
          fetch_engine.pc_nxt <= std_ulogic_vector(unsigned(fetch_engine.pc) + 4);
          ipb.we              <= not fetch_engine.restart; -- write to IPB if not being reset
          if (fetch_engine.restart = '1') then -- reset request?
            fetch_engine.restart_nxt <= '0';
          end if;
          fetch_engine.state_nxt <= IFETCH_REQUEST;
        end if;

      when others => -- undefined
      -- ------------------------------------------------------------
        fetch_engine.state_nxt <= IFETCH_REQUEST;

    end case;
  end process fetch_engine_fsm_comb;


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
    if (rstn_i = '0') then
      issue_engine.state <= ISSUE_ACTIVE;
      issue_engine.align <= CPU_BOOT_ADDR(1); -- 32- or 16-bit boundary
      issue_engine.buf   <= (others => '0');
    elsif rising_edge(clk_i) then
      if (ipb.clear = '1') then
        if (CPU_EXTENSION_RISCV_C = true) and (execute_engine.pc(1) = '1') then -- branch to unaligned address?
          issue_engine.state <= ISSUE_REALIGN;
          issue_engine.align <= '1'; -- aligned on 16-bit boundary
        else
          issue_engine.state <= issue_engine.state_nxt;
          issue_engine.align <= '0'; -- always aligned on 32-bit boundaries
        end if;
      else
        issue_engine.state <= issue_engine.state_nxt;
        issue_engine.align <= issue_engine.align_nxt;
      end if;
      issue_engine.buf <= issue_engine.buf_nxt;
    end if;
  end process issue_engine_fsm_sync;


  -- Issue Engine FSM Comb ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  issue_engine_fsm_comb: process(issue_engine, ipb, execute_engine, ci_illegal, ci_instr32)
  begin
    -- arbiter defaults --
    issue_engine.state_nxt <= issue_engine.state;
    issue_engine.align_nxt <= issue_engine.align;
    issue_engine.buf_nxt   <= issue_engine.buf;

    -- instruction prefetch buffer interface defaults --
    ipb.re <= '0';

    -- instruction issue interface defaults --
    -- cmd_issue.data = <illegal_compressed_instruction> & <bus_error & alignment_error> & <is_compressed_instrucion> & <32-bit_instruction_word>
    cmd_issue.data  <= '0' & ipb.rdata(33 downto 32) & '0' & ipb.rdata(31 downto 0);
    cmd_issue.valid <= '0';

    -- state machine --
    case issue_engine.state is

      when ISSUE_ACTIVE => -- issue instruction if available
      -- ------------------------------------------------------------
        if (ipb.avail = '1') then -- instructions available?

          if (issue_engine.align = '0') or (CPU_EXTENSION_RISCV_C = false) then -- begin check in LOW instruction half-word
            if (execute_engine.state = DISPATCH) then -- ready to issue new command?
              cmd_issue.valid      <= '1';
              issue_engine.buf_nxt <= ipb.rdata(33 downto 32) & ipb.rdata(31 downto 16); -- store high half-word - we might need it for an unaligned uncompressed instruction
              if (ipb.rdata(1 downto 0) = "11") or (CPU_EXTENSION_RISCV_C = false) then -- uncompressed and "aligned"
                ipb.re <= '1';
                cmd_issue.data <= '0' & ipb.rdata(33 downto 32) & '0' & ipb.rdata(31 downto 0);
              else -- compressed
                ipb.re <= '1';
                cmd_issue.data <= ci_illegal & ipb.rdata(33 downto 32) & '1' & ci_instr32;
                issue_engine.align_nxt <= '1';
              end if;
            end if;

          else -- begin check in HIGH instruction half-word
            if (execute_engine.state = DISPATCH) then -- ready to issue new command?
              cmd_issue.valid      <= '1';
              issue_engine.buf_nxt <= ipb.rdata(33 downto 32) & ipb.rdata(31 downto 16); -- store high half-word - we might need it for an unaligned uncompressed instruction
              if (issue_engine.buf(1 downto 0) = "11") then -- uncompressed and "unaligned"
                ipb.re <= '1';
                cmd_issue.data <= '0' & (ipb.rdata(33 downto 32) or issue_engine.buf(17 downto 16)) & '0' & (ipb.rdata(15 downto 0) & issue_engine.buf(15 downto 0));
              else -- compressed
                -- do not read from ipb here!
                cmd_issue.data <= ci_illegal & ipb.rdata(33 downto 32) & '1' & ci_instr32;
                issue_engine.align_nxt <= '0';
              end if;
            end if;
          end if;
        end if;

      when ISSUE_REALIGN => -- re-align input fifos after a branch to an unaligned address
      -- ------------------------------------------------------------
        issue_engine.buf_nxt <= ipb.rdata(33 downto 32) & ipb.rdata(31 downto 16);
        if (ipb.avail = '1') then -- instructions available?
          ipb.re <= '1';
          issue_engine.state_nxt <= ISSUE_ACTIVE;
        end if;

      when others => -- undefined
      -- ------------------------------------------------------------
        issue_engine.state_nxt <= ISSUE_ACTIVE;

    end case;
  end process issue_engine_fsm_comb;

  -- 16-bit instructions: half-word select --
  ci_instr16 <= ipb.rdata(15 downto 0) when (issue_engine.align = '0') else issue_engine.buf(15 downto 0);


  -- Compressed Instructions Recoding -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_decompressor_inst_true:
  if (CPU_EXTENSION_RISCV_C = true) generate
    neorv32_cpu_decompressor_inst: neorv32_cpu_decompressor
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
  imm_gen: process(rstn_i, clk_i)
    variable opcode_v : std_ulogic_vector(6 downto 0);
  begin
    if (rstn_i = '0') then
      imm_o <= (others => def_rst_val_c);
    elsif rising_edge(clk_i) then
      -- default: I-immediate: ALU-immediate, loads, jump-and-link with registers
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
      when funct3_bge_c | funct3_bgeu_c => -- branch if greater or equal (signed/unsigned)
        execute_engine.branch_taken <= not cmp_i(cmp_less_c);
      when others => -- invalid
        execute_engine.branch_taken <= '0';
    end case;
  end process branch_check;


  -- Execute Engine FSM Sync ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  execute_engine_fsm_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      -- registers that DO require a specific reset state --
      execute_engine.pc         <= CPU_BOOT_ADDR(data_width_c-1 downto 2) & "00"; -- 32-bit aligned!
      execute_engine.state      <= SYS_WAIT;
      execute_engine.sleep      <= '0';
      execute_engine.branched   <= '1'; -- reset is a branch from "somewhere"
      -- no dedicated RESET required --
      execute_engine.state_prev <= SYS_WAIT; -- actual reset value is not relevant
      execute_engine.i_reg      <= (others => def_rst_val_c);
      execute_engine.is_ci      <= def_rst_val_c;
      execute_engine.is_ici     <= def_rst_val_c;
      execute_engine.last_pc    <= (others => def_rst_val_c);
      execute_engine.i_reg_last <= (others => def_rst_val_c);
      execute_engine.next_pc    <= (others => def_rst_val_c);
      ctrl                      <= (others => def_rst_val_c);
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

      execute_engine.state      <= execute_engine.state_nxt;
      execute_engine.state_prev <= execute_engine.state;
      execute_engine.sleep      <= execute_engine.sleep_nxt;
      execute_engine.branched   <= execute_engine.branched_nxt;
      execute_engine.i_reg      <= execute_engine.i_reg_nxt;
      execute_engine.is_ci      <= execute_engine.is_ci_nxt;
      execute_engine.is_ici     <= execute_engine.is_ici_nxt;

      -- PC & IR of "last executed" instruction for trap handling --
      if (execute_engine.state = EXECUTE) then
        execute_engine.last_pc    <= execute_engine.pc;
        execute_engine.i_reg_last <= execute_engine.i_reg;
      end if;

      -- next PC --
      case execute_engine.state is
        when TRAP_ENTER => -- ENTERING trap environment
          if (CPU_EXTENSION_RISCV_DEBUG = false) then -- normal trapping
            execute_engine.next_pc <= csr.mtvec(data_width_c-1 downto 1) & '0'; -- trap enter
          else -- DEBUG MODE enabled
            if (trap_ctrl.cause(5) = '1') then -- trap cause: debug mode (re-)entry
              execute_engine.next_pc <= CPU_DEBUG_ADDR; -- debug mode enter; start at "parking loop" <normal_entry>
            elsif (debug_ctrl.running = '1') then -- any other exception INSIDE debug mode
              execute_engine.next_pc <= std_ulogic_vector(unsigned(CPU_DEBUG_ADDR) + 4); -- execute at "parking loop" <exception_entry>
            else -- normal trapping
              execute_engine.next_pc <= csr.mtvec(data_width_c-1 downto 1) & '0'; -- trap enter
            end if;
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

      -- main control bus --
      ctrl <= ctrl_nxt;
    end if;
  end process execute_engine_fsm_sync;


  -- PC increment for next linear instruction (+2 for compressed instr., +4 otherwise) --
  execute_engine.next_pc_inc <= x"00000004" when ((execute_engine.is_ci = '0') or (CPU_EXTENSION_RISCV_C = false)) else x"00000002";

  -- PC output --
  curr_pc_o <= execute_engine.pc(data_width_c-1 downto 1)      & '0'; -- current PC for ALU ops
  next_pc_o <= execute_engine.next_pc(data_width_c-1 downto 1) & '0'; -- next PC for ALU ops

  -- CSR access address --
  csr.addr <= execute_engine.i_reg(instr_csr_id_msb_c downto instr_csr_id_lsb_c);


  -- CPU Control Bus Output -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_output: process(ctrl, fetch_engine, trap_ctrl, bus_fast_ir, execute_engine, csr, debug_ctrl)
  begin
    -- signals from execute engine --
    ctrl_o <= ctrl;
    -- prevent commits if illegal instruction --
    ctrl_o(ctrl_rf_wb_en_c) <= ctrl(ctrl_rf_wb_en_c) and (not trap_ctrl.exc_buf(exception_iillegal_c));
    ctrl_o(ctrl_bus_rd_c)   <= ctrl(ctrl_bus_rd_c)   and (not trap_ctrl.exc_buf(exception_iillegal_c));
    ctrl_o(ctrl_bus_wr_c)   <= ctrl(ctrl_bus_wr_c)   and (not trap_ctrl.exc_buf(exception_iillegal_c));
    -- current privilege level --
    ctrl_o(ctrl_priv_lvl_msb_c downto ctrl_priv_lvl_lsb_c) <= csr.privilege_rd;
    -- register addresses --
    ctrl_o(ctrl_rf_rs1_adr4_c downto ctrl_rf_rs1_adr0_c) <= execute_engine.i_reg(instr_rs1_msb_c downto instr_rs1_lsb_c);
    ctrl_o(ctrl_rf_rs2_adr4_c downto ctrl_rf_rs2_adr0_c) <= execute_engine.i_reg(instr_rs2_msb_c downto instr_rs2_lsb_c);
    ctrl_o(ctrl_rf_rd_adr4_c  downto ctrl_rf_rd_adr0_c)  <= execute_engine.i_reg(instr_rd_msb_c  downto instr_rd_lsb_c);
    -- fast bus access requests --
    ctrl_o(ctrl_bus_if_c) <= bus_fast_ir;
    -- bus error control --
    ctrl_o(ctrl_bus_ierr_ack_c) <= fetch_engine.bus_err_ack; -- instruction fetch bus access error ACK
    ctrl_o(ctrl_bus_derr_ack_c) <= trap_ctrl.env_start_ack; -- data access bus error access ACK
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
    variable sys_env_cmd_mask_v : std_ulogic_vector(11 downto 0);
  begin
    -- defaults --
    decode_aux.is_atomic_lr    <= '0';
    decode_aux.is_atomic_sc    <= '0';
    decode_aux.is_float_op     <= '0';
    decode_aux.is_m_mul        <= '0';
    decode_aux.is_m_div        <= '0';
    decode_aux.is_bitmanip_imm <= '0';
    decode_aux.is_bitmanip_reg <= '0';
    decode_aux.rs1_zero        <= '0';
    decode_aux.rs2_zero        <= '0';
    decode_aux.rd_zero         <= '0';

    -- is atomic load-reservate/store-conditional? --
    if (CPU_EXTENSION_RISCV_A = true) and (execute_engine.i_reg(instr_opcode_lsb_c+2) = '1') then -- valid atomic sub-opcode
      decode_aux.is_atomic_lr <= not execute_engine.i_reg(instr_funct5_lsb_c);
      decode_aux.is_atomic_sc <=     execute_engine.i_reg(instr_funct5_lsb_c);
    end if;

    -- is BITMANIP instruction? --
    -- pretty complex as we have to extract this from the ALU/ALUI instruction space --
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
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0110100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "101") and (execute_engine.i_reg(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c) = "11000")) then -- REV8
      decode_aux.is_bitmanip_imm <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_B); -- BITMANIP implemented at all?
    end if;
    -- register operation --
    if ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0110000") and (execute_engine.i_reg(instr_funct3_msb_c-1 downto instr_funct3_lsb_c) = "01")) or -- ROR / ROL
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000101") and (execute_engine.i_reg(instr_funct3_msb_c) = '1')) or -- MIN[U] / MAX[U]
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "100")) or -- ZEXTH
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
      decode_aux.is_bitmanip_reg <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_B); -- BITMANIP implemented at all?
    end if;

    -- floating-point operations (Zfinx) --
    if ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+3) = "0000")) or -- FADD.S / FSUB.S
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "00010")) or -- FMUL.S
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "11100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001")) or -- FCLASS.S
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "00100") and (execute_engine.i_reg(instr_funct3_msb_c) = '0')) or -- FSGNJ[N/X].S
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "00101") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_msb_c-1) = "00")) or -- FMIN.S / FMAX.S
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "10100") and (execute_engine.i_reg(instr_funct3_msb_c) = '0')) or -- FEQ.S / FLT.S / FLE.S
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "11010") and (execute_engine.i_reg(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c+1) = "0000")) or -- FCVT.S.W*
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "11000") and (execute_engine.i_reg(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c+1) = "0000")) then -- FCVT.W*.S
      decode_aux.is_float_op <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zfinx); -- FPU implemented at all?
    end if;

    -- system/environment instructions --
    sys_env_cmd_mask_v := funct12_ecall_c or funct12_ebreak_c or funct12_mret_c or funct12_wfi_c or funct12_dret_c; -- sum-up set bits
    decode_aux.sys_env_cmd <= execute_engine.i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) and sys_env_cmd_mask_v; -- set unused bits to always-zero

    -- integer MUL (M/Zmmul) / DIV (M) operation --
    if (execute_engine.i_reg(instr_opcode_lsb_c+5) = opcode_alu_c(5)) and
       (execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000001") then
      decode_aux.is_m_mul <= (not execute_engine.i_reg(instr_funct3_msb_c)) and (bool_to_ulogic_f(CPU_EXTENSION_RISCV_M) or bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zmmul));
      decode_aux.is_m_div <= execute_engine.i_reg(instr_funct3_msb_c) and bool_to_ulogic_f(CPU_EXTENSION_RISCV_M);
    end if;

    -- register address checks --
    decode_aux.rs1_zero <= not or_reduce_f(execute_engine.i_reg(instr_rs1_msb_c downto instr_rs1_lsb_c));
    decode_aux.rs2_zero <= not or_reduce_f(execute_engine.i_reg(instr_rs2_msb_c downto instr_rs2_lsb_c));
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
    -- ALU main control --
    ctrl_nxt(ctrl_alu_op2_c   downto ctrl_alu_op0_c)   <= alu_op_add_c;    -- default ALU operation: ADD
    ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_core_c; -- default ALU operation: ADD
    -- ALU sign control --
    if (execute_engine.i_reg(instr_opcode_lsb_c+4) = '1') then -- ALU ops
      ctrl_nxt(ctrl_alu_unsigned_c) <= execute_engine.i_reg(instr_funct3_lsb_c+0); -- unsigned ALU operation? (SLTIU, SLTU)
    else -- branches
      ctrl_nxt(ctrl_alu_unsigned_c) <= execute_engine.i_reg(instr_funct3_lsb_c+1); -- unsigned branches? (BLTU, BGEU)
    end if;
    -- atomic store-conditional instruction (evaluate lock status) --
    ctrl_nxt(ctrl_bus_ch_lock_c) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_A) and decode_aux.is_atomic_sc;


    -- state machine --
    case execute_engine.state is

      when SYS_WAIT => -- System delay cycle (to let side effects kick in)
      -- ------------------------------------------------------------
        execute_engine.state_nxt <= DISPATCH;


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
          trap_ctrl.instr_ma        <= cmd_issue.data(33) and (not bool_to_ulogic_f(CPU_EXTENSION_RISCV_C)); -- misaligned instruction fetch address, if C disabled
          trap_ctrl.instr_be        <= cmd_issue.data(34); -- bus access fault during instruction fetch
          execute_engine.is_ici_nxt <= cmd_issue.data(35); -- invalid decompressed instruction
          -- any reason to go to trap state? --
          if (execute_engine.sleep = '1') or -- enter sleep state
             (trap_ctrl.exc_fire = '1') or -- exception during LAST instruction (illegal instruction)
             (trap_ctrl.env_start = '1') or -- pending trap (IRQ or exception)
             ((cmd_issue.data(33) = '1') and (CPU_EXTENSION_RISCV_C = false)) or -- misaligned instruction fetch address, if C disabled
             (cmd_issue.data(34) = '1') then -- bus access fault during instruction fetch
            execute_engine.state_nxt <= TRAP_ENTER;
          else
            execute_engine.state_nxt <= EXECUTE;
          end if;
        end if;


      when TRAP_ENTER => -- Start trap environment - get xTVEC, stay here for sleep mode
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
        execute_engine.state_nxt  <= SYS_WAIT;


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

            -- Check if single-cycle or multi-cycle (co-processor) operation --
            -- co-processor MULDIV operation? --
            if ((CPU_EXTENSION_RISCV_M = true) and ((decode_aux.is_m_mul = '1') or (decode_aux.is_m_div = '1'))) or -- MUL/DIV
               ((CPU_EXTENSION_RISCV_Zmmul = true) and (decode_aux.is_m_mul = '1')) then -- MUL
              ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_muldiv_c; -- use MULDIV CP
              ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_copro_c;
              execute_engine.state_nxt                           <= ALU_WAIT;
            -- co-processor BIT-MANIPULATION operation? --
            elsif (CPU_EXTENSION_RISCV_B = true) and
                  (((execute_engine.i_reg(instr_opcode_lsb_c+5) = opcode_alu_c(5))  and (decode_aux.is_bitmanip_reg = '1')) or -- register operation
                   ((execute_engine.i_reg(instr_opcode_lsb_c+5) = opcode_alui_c(5)) and (decode_aux.is_bitmanip_imm = '1'))) then -- immediate operation
              ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_bitmanip_c; -- use BITMANIP CP
              ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_copro_c;
              execute_engine.state_nxt                           <= ALU_WAIT;
            -- co-processor SHIFT operation? --
            elsif (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sll_c) or
                  (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sr_c) then
              ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_shifter_c; -- use SHIFTER CP (only relevant for shift operations)
              ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_copro_c;
              execute_engine.state_nxt                           <= ALU_WAIT;
            -- ALU core operations (single-cycle) --
            else
              ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_core_c;
              ctrl_nxt(ctrl_rf_wb_en_c)                          <= '1'; -- valid RF write-back
              execute_engine.state_nxt                           <= DISPATCH;
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
              execute_engine.state_nxt    <= SYS_WAIT;
            elsif (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_fencei_c) and (CPU_EXTENSION_RISCV_Zifencei = true)  then -- FENCE.I
              ctrl_nxt(ctrl_bus_fencei_c) <= '1';
              execute_engine.branched_nxt <= '1'; -- this is an actual branch
              execute_engine.state_nxt    <= TRAP_EXECUTE; -- use TRAP_EXECUTE to "modify" PC (PC <= PC)
            else -- illegal fence instruction
              execute_engine.state_nxt    <= SYS_WAIT;
            end if;


          when opcode_syscsr_c => -- system/csr access
          -- ------------------------------------------------------------
            if (CPU_EXTENSION_RISCV_Zicsr = true) then
              if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_env_c) then -- system/environment
                execute_engine.state_nxt <= SYS_ENV;
              else -- CSR access
                execute_engine.state_nxt <= CSR_ACCESS;
              end if;
            else
              execute_engine.state_nxt <= SYS_WAIT;
            end if;


          when opcode_fop_c => -- floating-point operations
          -- ------------------------------------------------------------
            if (CPU_EXTENSION_RISCV_Zfinx = true) then
              ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_fpu_c; -- trigger FPU CP
              ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_copro_c;
              execute_engine.state_nxt <= ALU_WAIT;
            else
              execute_engine.state_nxt <= SYS_WAIT;
            end if;


          when others => -- illegal opcode
          -- ------------------------------------------------------------
            execute_engine.state_nxt <= SYS_WAIT;

        end case;


      when SYS_ENV => -- system environment operation - execution
      -- ------------------------------------------------------------
        execute_engine.state_nxt <= SYS_WAIT; -- default
        if (trap_ctrl.exc_buf(exception_iillegal_c) = '0') then -- no illegal instruction
          case decode_aux.sys_env_cmd is -- use a simplified input here (with hardwired zeros)
            when funct12_ecall_c  => trap_ctrl.env_call       <= '1'; -- ECALL
            when funct12_ebreak_c => trap_ctrl.break_point    <= '1'; -- EBREAK
            when funct12_mret_c   => execute_engine.state_nxt <= TRAP_EXIT; -- MRET
            when funct12_dret_c => -- DRET
              if (CPU_EXTENSION_RISCV_DEBUG = true) then
                execute_engine.state_nxt <= TRAP_EXIT;
                debug_ctrl.dret <= '1';
              else
                NULL; -- executed as NOP (and raise illegal instruction exception)
              end if;
            when funct12_wfi_c => -- WFI
              if (CPU_EXTENSION_RISCV_DEBUG = true) and
                ((debug_ctrl.running = '1') or (csr.dcsr_step = '1')) then -- act as NOP when in debug-mode or during single-stepping
                NULL; -- executed as NOP
              else
                execute_engine.sleep_nxt <= '1'; -- go to sleep mode
              end if;
            when others => NULL; -- undefined / execute as NOP
          end case;
        end if;


      when CSR_ACCESS => -- read & write status and control register (CSR)
      -- ------------------------------------------------------------
        -- CSR write access --
        if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrw_c) or
           (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrwi_c) then -- CSRRW(I)
          csr.we_nxt <= '1'; -- always write CSR
        else -- CSRRS(I) / CSRRC(I) [invalid CSR instruction are already checked by the illegal instruction logic]
          csr.we_nxt <= not decode_aux.rs1_zero; -- write CSR if rs1/imm is not zero
        end if;
        -- register file write back --
        ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_csrr_c;
        ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back
        execute_engine.state_nxt  <= DISPATCH;


      when ALU_WAIT => -- wait for multi-cycle ALU operation (co-processor) to finish
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_copro_c;
        if (alu_idone_i = '1') or (trap_ctrl.exc_buf(exception_iillegal_c) = '1') then -- completed or exception
          ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back
          execute_engine.state_nxt  <= DISPATCH;
        end if;


      when BRANCH => -- update PC for taken branches and jumps
      -- ------------------------------------------------------------
        -- get and store return address (only relevant for jump-and-link operations) --
        ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_nxpc_c; -- next PC
        ctrl_nxt(ctrl_rf_wb_en_c) <= execute_engine.i_reg(instr_opcode_lsb_c+2); -- valid RF write-back? (is jump-and-link?)
        -- destination address --
        execute_engine.pc_mux_sel <= '1'; -- PC <= alu.add = branch/jump destination
        if (execute_engine.i_reg(instr_opcode_lsb_c+2) = '1') or (execute_engine.branch_taken = '1') then -- JAL/JALR or taken branch
          -- no need to check for illegal instructions here; the branch condition evaluation circuit will not set "branch_taken" if funct3 is invalid
          execute_engine.pc_we        <= '1'; -- update PC
          execute_engine.branched_nxt <= '1'; -- this is an actual branch
          fetch_engine.reset          <= '1'; -- trigger new instruction fetch from modified PC
          execute_engine.state_nxt    <= SYS_WAIT;
        else
          execute_engine.state_nxt <= DISPATCH;
        end if;


      when LOADSTORE_0 => -- trigger memory request
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_bus_lock_c) <= decode_aux.is_atomic_lr; -- atomic.LR: set lock
        if (execute_engine.i_reg(instr_opcode_msb_c-1) = '0') or (decode_aux.is_atomic_lr = '1') then -- normal load or atomic load-reservate
          ctrl_nxt(ctrl_bus_rd_c) <= '1'; -- read request
        else -- store
          if (decode_aux.is_atomic_sc = '0') or (CPU_EXTENSION_RISCV_A = false) then -- (normal) write request
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
        ctrl_nxt(ctrl_rf_in_mux_c) <= '1'; -- RF input = memory input (only relevant for LOADs)
        -- wait for memory response --
        if (trap_ctrl.env_start = '1') and (trap_ctrl.cause(6 downto 5) = "00") then -- abort if SYNC EXCEPTION (from bus or illegal cmd) / no IRQs and NOT DEBUG-MODE-related
          execute_engine.state_nxt <= DISPATCH;
        elsif (bus_d_wait_i = '0') then -- wait for bus to finish transaction
          -- data write-back --
          if (execute_engine.i_reg(instr_opcode_msb_c-1) = '0') or -- normal load
             (decode_aux.is_atomic_lr = '1') or -- atomic load-reservate
             (decode_aux.is_atomic_sc = '1') then -- atomic store-conditional
            ctrl_nxt(ctrl_rf_wb_en_c) <= '1';
          end if;
          -- remove atomic lock if this is NOT the LR.W instruction used to SET the lock --
          if (decode_aux.is_atomic_lr = '0') then -- execute and evaluate atomic store-conditional
            ctrl_nxt(ctrl_bus_de_lock_c) <= '1';
          end if;
          execute_engine.state_nxt <= DISPATCH;
        end if;


      when others => -- undefined
      -- ------------------------------------------------------------
        execute_engine.state_nxt <= SYS_WAIT;

    end case;
  end process execute_engine_fsm_comb;


-- ****************************************************************************************************************************
-- Invalid Instruction / CSR access check
-- ****************************************************************************************************************************

  -- CSR Access Check -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_access_check: process(execute_engine.i_reg, decode_aux, csr, debug_ctrl)
    variable csr_wacc_v : std_ulogic; -- actual CSR write
--  variable csr_racc_v : std_ulogic; -- actual CSR read
  begin
    -- is this CSR instruction really going to write to a CSR? --
    if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrw_c) or
       (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrwi_c) then
      csr_wacc_v := '1'; -- always write CSR
--    csr_racc_v := or_reduce_f(execute_engine.i_reg(instr_rd_msb_c downto instr_rd_lsb_c)); -- read if rd != 0
    else -- clear/set
      csr_wacc_v := not decode_aux.rs1_zero; -- write if rs1/uimm5 != 0
--    csr_racc_v := '1'; -- always read CSR
    end if;

    -- check CSR access --
    case csr.addr is

      -- floating-point CSRs --
      when csr_fflags_c | csr_frm_c | csr_fcsr_c =>
        csr_acc_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zfinx); -- full access for everyone if FPU implemented

      -- machine trap setup/handling & counters --
      when csr_mstatus_c | csr_mstatush_c | csr_misa_c | csr_mie_c | csr_mtvec_c | csr_mscratch_c | csr_mepc_c | csr_mcause_c | csr_mip_c | csr_mtval_c |
           csr_mcycle_c | csr_mcycleh_c | csr_minstret_c | csr_minstreth_c | csr_mcountinhibit_c =>
        -- NOTE: MISA and MTVAL are read-only in the NEORV32 but we do not cause an exception here for compatibility.
        -- Machine-level code should read-back those CSRs after writing them to realize they are read-only.
        csr_acc_valid <= csr.priv_m_mode; -- M-mode only 

      -- machine information registers, read-only --
      when csr_mvendorid_c | csr_marchid_c | csr_mimpid_c | csr_mhartid_c | csr_mconfigptr_c =>
        csr_acc_valid <= (not csr_wacc_v) and csr.priv_m_mode; -- M-mode only, read-only

      -- user-mode registers --
      when csr_mcounteren_c | csr_menvcfg_c | csr_menvcfgh_c =>
        csr_acc_valid <= csr.priv_m_mode and bool_to_ulogic_f(CPU_EXTENSION_RISCV_U);

      -- physical memory protection (PMP) --
      when csr_pmpaddr0_c  | csr_pmpaddr1_c  | csr_pmpaddr2_c  | csr_pmpaddr3_c  | csr_pmpaddr4_c  | csr_pmpaddr5_c  | csr_pmpaddr6_c  | csr_pmpaddr7_c  | -- address
           csr_pmpaddr8_c  | csr_pmpaddr9_c  | csr_pmpaddr10_c | csr_pmpaddr11_c | csr_pmpaddr12_c | csr_pmpaddr13_c | csr_pmpaddr14_c | csr_pmpaddr15_c |
           csr_pmpaddr16_c | csr_pmpaddr17_c | csr_pmpaddr18_c | csr_pmpaddr19_c | csr_pmpaddr20_c | csr_pmpaddr21_c | csr_pmpaddr22_c | csr_pmpaddr23_c |
           csr_pmpaddr24_c | csr_pmpaddr25_c | csr_pmpaddr26_c | csr_pmpaddr27_c | csr_pmpaddr28_c | csr_pmpaddr29_c | csr_pmpaddr30_c | csr_pmpaddr31_c |
           csr_pmpaddr32_c | csr_pmpaddr33_c | csr_pmpaddr34_c | csr_pmpaddr35_c | csr_pmpaddr36_c | csr_pmpaddr37_c | csr_pmpaddr38_c | csr_pmpaddr39_c |
           csr_pmpaddr40_c | csr_pmpaddr41_c | csr_pmpaddr42_c | csr_pmpaddr43_c | csr_pmpaddr44_c | csr_pmpaddr45_c | csr_pmpaddr46_c | csr_pmpaddr47_c |
           csr_pmpaddr48_c | csr_pmpaddr49_c | csr_pmpaddr50_c | csr_pmpaddr51_c | csr_pmpaddr52_c | csr_pmpaddr53_c | csr_pmpaddr54_c | csr_pmpaddr55_c |
           csr_pmpaddr56_c | csr_pmpaddr57_c | csr_pmpaddr58_c | csr_pmpaddr59_c | csr_pmpaddr60_c | csr_pmpaddr61_c | csr_pmpaddr62_c | csr_pmpaddr63_c |
           csr_pmpcfg0_c   | csr_pmpcfg1_c   | csr_pmpcfg2_c   | csr_pmpcfg3_c   | csr_pmpcfg4_c   | csr_pmpcfg5_c   | csr_pmpcfg6_c   | csr_pmpcfg7_c   | -- configuration
           csr_pmpcfg8_c   | csr_pmpcfg9_c   | csr_pmpcfg10_c  | csr_pmpcfg11_c  | csr_pmpcfg12_c  | csr_pmpcfg13_c  | csr_pmpcfg14_c  | csr_pmpcfg15_c =>
        csr_acc_valid <= csr.priv_m_mode and bool_to_ulogic_f(boolean(PMP_NUM_REGIONS > 0)); -- M-mode only

      -- hardware performance monitors (HPM) --
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
        csr_acc_valid <= csr.priv_m_mode and bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zihpm); -- M-mode only

      -- user-level counters/timers (read-only) --
      when csr_cycle_c | csr_cycleh_c | csr_instret_c | csr_instreth_c | csr_time_c | csr_timeh_c =>
        case csr.addr(1 downto 0) is
          when "00"   => csr_acc_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicntr) and (not csr_wacc_v) and (csr.priv_m_mode or csr.mcounteren_cy); -- cyle[h]: M-mode, U-mode if authorized, implemented at all, read-only
          when "01"   => csr_acc_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicntr) and (not csr_wacc_v) and (csr.priv_m_mode or csr.mcounteren_tm); -- time[h]: M-mode, U-mode if authorized, implemented at all, read-only
          when "10"   => csr_acc_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicntr) and (not csr_wacc_v) and (csr.priv_m_mode or csr.mcounteren_ir); -- instret[h]: M-mode, U-mode if authorized, implemented at all read-only
          when others => csr_acc_valid <= '0';
        end case;

      -- debug mode CSRs --
      when csr_dcsr_c | csr_dpc_c | csr_dscratch0_c =>
        csr_acc_valid <= debug_ctrl.running and bool_to_ulogic_f(CPU_EXTENSION_RISCV_DEBUG); -- access only in debug-mode

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
    -- illegal instructions are checked in the EXECUTE state
    -- the execute engine should not commit any illegal instruction
    if (execute_engine.state = EXECUTE) then
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
          illegal_register <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_E) and execute_engine.i_reg(instr_rd_msb_c);

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
          elsif (CPU_EXTENSION_RISCV_B = true) and (decode_aux.is_bitmanip_reg = '1') then -- valid BITMANIP instruction?
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
          end if;
          -- illegal E-CPU register? --
          illegal_register <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_E) and (execute_engine.i_reg(instr_rd_msb_c) or execute_engine.i_reg(instr_rs1_msb_c) or execute_engine.i_reg(instr_rs2_msb_c));

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
          elsif (CPU_EXTENSION_RISCV_B = true) and (decode_aux.is_bitmanip_imm = '1') then -- valid BITMANIP immediate instruction?
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
          end if;
          -- illegal E-CPU register? --
          illegal_register <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_E) and (execute_engine.i_reg(instr_rs1_msb_c) or execute_engine.i_reg(instr_rd_msb_c));

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
          illegal_register <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_E) and (execute_engine.i_reg(instr_rs1_msb_c) or execute_engine.i_reg(instr_rd_msb_c));

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
          illegal_register <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_E) and (execute_engine.i_reg(instr_rs2_msb_c) or execute_engine.i_reg(instr_rs1_msb_c));

        when opcode_atomic_c => -- atomic instructions
        -- ------------------------------------------------------------
          if (CPU_EXTENSION_RISCV_A = true) then
            if (execute_engine.i_reg(instr_funct5_msb_c downto instr_funct5_lsb_c) = "00010") then -- LR
              illegal_instruction <= '0';
              -- illegal E-CPU register? --
              illegal_register <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_E) and (execute_engine.i_reg(instr_rs1_msb_c) or execute_engine.i_reg(instr_rd_msb_c));
            elsif (execute_engine.i_reg(instr_funct5_msb_c downto instr_funct5_lsb_c) = "00011") then -- SC
              illegal_instruction <= '0';
              -- illegal E-CPU register? --
              illegal_register <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_E) and (execute_engine.i_reg(instr_rs1_msb_c) or execute_engine.i_reg(instr_rs2_msb_c) or execute_engine.i_reg(instr_rd_msb_c));
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
          illegal_register <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_E) and (execute_engine.i_reg(instr_rs2_msb_c) or execute_engine.i_reg(instr_rs1_msb_c));

        when opcode_jalr_c => -- check JALR.funct3
        -- ------------------------------------------------------------
          if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "000") then
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
          end if;
          -- illegal E-CPU register? --
          illegal_register <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_E) and (execute_engine.i_reg(instr_rs1_msb_c) or execute_engine.i_reg(instr_rd_msb_c));

        when opcode_fence_c => -- check FENCE.funct3
        -- ------------------------------------------------------------
          if ((execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_fencei_c) and (CPU_EXTENSION_RISCV_Zifencei = true)) or -- FENCE.I
             (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_fence_c) then -- FENCE
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
          end if;
          -- illegal E-CPU register? --
          illegal_register <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_E) and (execute_engine.i_reg(instr_rs1_msb_c) or execute_engine.i_reg(instr_rd_msb_c));

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
            if (CPU_EXTENSION_RISCV_E = true) then
              if (execute_engine.i_reg(instr_funct3_msb_c) = '0') then -- reg-reg CSR
                illegal_register <= execute_engine.i_reg(instr_rs1_msb_c) or execute_engine.i_reg(instr_rd_msb_c);
              else -- reg-imm CSR
                illegal_register <= execute_engine.i_reg(instr_rd_msb_c);
              end if;
            end if;
          -- ecall, ebreak, mret, wfi, dret --
          elsif (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "000") and
                (decode_aux.rs1_zero = '1') and (decode_aux.rd_zero = '1') and
                ((execute_engine.i_reg(instr_funct12_msb_c  downto instr_funct12_lsb_c) = funct12_ecall_c)  or -- ECALL
                 (execute_engine.i_reg(instr_funct12_msb_c  downto instr_funct12_lsb_c) = funct12_ebreak_c) or -- EBREAK 
                 ((execute_engine.i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = funct12_mret_c) and (csr.priv_m_mode = '1')) or -- MRET (only allowed in M-mode)
                 ((execute_engine.i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = funct12_dret_c) and (CPU_EXTENSION_RISCV_DEBUG = true) and (debug_ctrl.running = '1')) or -- DRET (only allowed in D-mode)
                 (execute_engine.i_reg(instr_funct12_msb_c  downto instr_funct12_lsb_c) = funct12_wfi_c)) then -- WFI (always allowed to execute)
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
          end if;

        when opcode_fop_c => -- floating point operations - single/dual operands
        -- ------------------------------------------------------------
          if (CPU_EXTENSION_RISCV_Zfinx = true) and -- F extension implemented
             (execute_engine.i_reg(instr_funct7_lsb_c+1 downto instr_funct7_lsb_c) = float_single_c) and -- single-precision operations only
             (decode_aux.is_float_op = '1') then -- is correct/supported floating-point instruction
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
          end if;
          -- illegal E-CPU register? --
          -- FIXME: rs2 is not checked!
          illegal_register <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_E) and (execute_engine.i_reg(instr_rs1_msb_c) or execute_engine.i_reg(instr_rd_msb_c));

        when others => -- undefined instruction -> illegal!
        -- ------------------------------------------------------------
          illegal_instruction <= '1';

      end case;
    else
      illegal_opcode_lsbs <= '0';
      illegal_compressed  <= '0';
      illegal_instruction <= '0';
      illegal_register    <= '0';
    end if;
  end process illegal_instruction_check;

  -- any illegal condition? --
  trap_ctrl.instr_il <= illegal_instruction or illegal_opcode_lsbs or illegal_register or illegal_compressed;


-- ****************************************************************************************************************************
-- Exception and Interrupt (= Trap) Control
-- ****************************************************************************************************************************

  -- Trap Controller ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trap_controller: process(rstn_i, clk_i)
    variable mode_m_v, mode_u_v : std_ulogic;
  begin
    if (rstn_i = '0') then
      trap_ctrl.exc_buf   <= (others => '0');
      trap_ctrl.irq_buf   <= (others => '0');
      trap_ctrl.exc_ack   <= '0';
      trap_ctrl.env_start <= '0';
      trap_ctrl.cause     <= (others => '0');
    elsif rising_edge(clk_i) then
      if (CPU_EXTENSION_RISCV_Zicsr = true) then

        -- exception queue: misaligned load/store/instruction address --
        trap_ctrl.exc_buf(exception_lalign_c) <= (trap_ctrl.exc_buf(exception_lalign_c) or ma_load_i)          and (not trap_ctrl.exc_ack);
        trap_ctrl.exc_buf(exception_salign_c) <= (trap_ctrl.exc_buf(exception_salign_c) or ma_store_i)         and (not trap_ctrl.exc_ack);
        trap_ctrl.exc_buf(exception_ialign_c) <= (trap_ctrl.exc_buf(exception_ialign_c) or trap_ctrl.instr_ma) and (not trap_ctrl.exc_ack);

        -- exception queue: load/store/instruction bus access error --
        trap_ctrl.exc_buf(exception_laccess_c) <= (trap_ctrl.exc_buf(exception_laccess_c) or be_load_i)          and (not trap_ctrl.exc_ack);
        trap_ctrl.exc_buf(exception_saccess_c) <= (trap_ctrl.exc_buf(exception_saccess_c) or be_store_i)         and (not trap_ctrl.exc_ack);
        trap_ctrl.exc_buf(exception_iaccess_c) <= (trap_ctrl.exc_buf(exception_iaccess_c) or trap_ctrl.instr_be) and (not trap_ctrl.exc_ack);

        -- exception queue: illegal instruction / environment calls --
        trap_ctrl.exc_buf(exception_m_envcall_c) <= (trap_ctrl.exc_buf(exception_m_envcall_c) or (trap_ctrl.env_call and csr.priv_m_mode)) and (not trap_ctrl.exc_ack);
        trap_ctrl.exc_buf(exception_u_envcall_c) <= (trap_ctrl.exc_buf(exception_u_envcall_c) or (trap_ctrl.env_call and csr.priv_u_mode)) and (not trap_ctrl.exc_ack);
        trap_ctrl.exc_buf(exception_iillegal_c)  <= (trap_ctrl.exc_buf(exception_iillegal_c)  or trap_ctrl.instr_il)                       and (not trap_ctrl.exc_ack);

        -- exception queue: break point --
        if (CPU_EXTENSION_RISCV_DEBUG = true) then
          trap_ctrl.exc_buf(exception_break_c) <= (not trap_ctrl.exc_ack) and (trap_ctrl.exc_buf(exception_break_c) or 
            ((trap_ctrl.break_point and csr.priv_m_mode and (not csr.dcsr_ebreakm) and (not debug_ctrl.running)) or -- enable break to machine-trap-handler when in machine mode on "ebreak"
             (trap_ctrl.break_point and csr.priv_u_mode and (not csr.dcsr_ebreaku) and (not debug_ctrl.running)))); -- enable break to machine-trap-handler when in user mode on "ebreak"
        else
          trap_ctrl.exc_buf(exception_break_c) <= (trap_ctrl.exc_buf(exception_break_c) or trap_ctrl.break_point) and (not trap_ctrl.exc_ack);
        end if;

        -- exception buffer: enter debug mode --
        trap_ctrl.exc_buf(exception_db_break_c) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_DEBUG) and (trap_ctrl.exc_buf(exception_db_break_c) or debug_ctrl.trig_break) and (not trap_ctrl.exc_ack);
        trap_ctrl.irq_buf(interrupt_db_halt_c)  <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_DEBUG) and debug_ctrl.trig_halt;
        trap_ctrl.irq_buf(interrupt_db_step_c)  <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_DEBUG) and debug_ctrl.trig_step;

        -- interrupt buffer: machine software/external/timer interrupt --
        trap_ctrl.irq_buf(interrupt_msw_irq_c)   <= csr.mie_msie and msw_irq_i;
        trap_ctrl.irq_buf(interrupt_mext_irq_c)  <= csr.mie_meie and mext_irq_i;
        trap_ctrl.irq_buf(interrupt_mtime_irq_c) <= csr.mie_mtie and mtime_irq_i;

        -- interrupt queue: NEORV32-specific fast interrupts (FIRQ) --
        trap_ctrl.irq_buf(interrupt_firq_15_c downto interrupt_firq_0_c) <= (trap_ctrl.irq_buf(interrupt_firq_15_c downto interrupt_firq_0_c) or (csr.mie_firqe and firq_i)) and (not csr.mip_clr);

        -- trap environment control --
        if (trap_ctrl.env_start = '0') then -- no started trap handler
          if (trap_ctrl.exc_fire = '1') or ((trap_ctrl.irq_fire = '1') and -- exception triggered!
             ((execute_engine.state = EXECUTE) or (execute_engine.state = TRAP_ENTER))) then -- fire IRQs in EXECUTE or TRAP state only to continue execution even on permanent IRQ
            trap_ctrl.cause     <= trap_ctrl.cause_nxt; -- capture source ID for program (for mcause csr)
            trap_ctrl.exc_ack   <= '1';                 -- clear exceptions (no ack mask: these have highest priority and are always evaluated first!)
            trap_ctrl.env_start <= '1';                 -- now execute engine can start trap handler
          end if;
        else -- trap waiting to get started
          if (trap_ctrl.env_start_ack = '1') then -- start of trap handler acknowledged by execution engine
            trap_ctrl.exc_ack   <= '0';
            trap_ctrl.env_start <= '0';
          end if;
        end if;
      end if;
    end if;
  end process trap_controller;

  -- any exception/interrupt? --
  trap_ctrl.exc_fire <= or_reduce_f(trap_ctrl.exc_buf); -- exceptions/faults CANNOT be masked
  trap_ctrl.irq_fire <= (or_reduce_f(trap_ctrl.irq_buf) and csr.mstatus_mie and trap_ctrl.db_irq_en) or trap_ctrl.db_irq_fire; -- interrupts CAN be masked (but not the DEBUG halt IRQ)

  -- debug mode (entry) interrupts --
  trap_ctrl.db_irq_en   <= '0' when (CPU_EXTENSION_RISCV_DEBUG = true) and ((debug_ctrl.running = '1') or (csr.dcsr_step = '1')) else '1'; -- no interrupts when IN debug mode or IN single-step mode
  trap_ctrl.db_irq_fire <= (trap_ctrl.irq_buf(interrupt_db_step_c) or trap_ctrl.irq_buf(interrupt_db_halt_c)) when (CPU_EXTENSION_RISCV_DEBUG = true) else '0'; -- "NMI" for debug mode entry


  -- Trap Priority Encoder ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trap_priority: process(trap_ctrl)
  begin
    -- defaults --
    trap_ctrl.cause_nxt <= (others => '0');

    -- NOTE: Synchronous exceptions (from trap_ctrl.exc_buf) have higher priority than asynchronous
    -- exceptions (from trap_ctrl.irq_buf).

    -- ----------------------------------------------------------------------------------------
    -- the following traps are caused by *synchronous* exceptions; we do not need a
    -- specific acknowledge mask since only _one_ exception (the one with highest priority)
    -- is allowed to kick in at once
    -- ----------------------------------------------------------------------------------------

    -- exception: 0.0 instruction address misaligned --
    if (trap_ctrl.exc_buf(exception_ialign_c) = '1') then
      trap_ctrl.cause_nxt <= trap_ima_c;

    -- exception: 0.1 instruction access fault --
    elsif (trap_ctrl.exc_buf(exception_iaccess_c) = '1') then
      trap_ctrl.cause_nxt <= trap_iba_c;

    -- exception: 0.2 illegal instruction --
    elsif (trap_ctrl.exc_buf(exception_iillegal_c) = '1') then
      trap_ctrl.cause_nxt <= trap_iil_c;


    -- exception: 0.11 environment call from M-mode --
    elsif (trap_ctrl.exc_buf(exception_m_envcall_c) = '1') then
      trap_ctrl.cause_nxt <= trap_menv_c;

    -- exception: 0.8 environment call from U-mode --
    elsif (trap_ctrl.exc_buf(exception_u_envcall_c) = '1') then
      trap_ctrl.cause_nxt <= trap_uenv_c;

    -- exception: 0.3 breakpoint --
    elsif (trap_ctrl.exc_buf(exception_break_c) = '1') then
      trap_ctrl.cause_nxt <= trap_brk_c;


    -- exception: 0.6 store address misaligned -
    elsif (trap_ctrl.exc_buf(exception_salign_c) = '1') then
      trap_ctrl.cause_nxt <= trap_sma_c;

    -- exception: 0.4 load address misaligned --
    elsif (trap_ctrl.exc_buf(exception_lalign_c) = '1') then
      trap_ctrl.cause_nxt <= trap_lma_c;

    -- exception: 0.7 store access fault --
    elsif (trap_ctrl.exc_buf(exception_saccess_c) = '1') then
      trap_ctrl.cause_nxt <= trap_sbe_c;

    -- exception: 0.5 load access fault --
    elsif (trap_ctrl.exc_buf(exception_laccess_c) = '1') then
      trap_ctrl.cause_nxt <= trap_lbe_c;


    -- ----------------------------------------------------------------------------------------
    -- (re-)enter debug mode requests; basically, these are standard traps that have some
    -- special handling - they have the highest INTERRUPT priority in order to go to debug when requested
    -- even if other IRQs are pending right now
    -- ----------------------------------------------------------------------------------------

    -- break instruction --
    elsif (CPU_EXTENSION_RISCV_DEBUG = true) and (trap_ctrl.exc_buf(exception_db_break_c) = '1') then
      trap_ctrl.cause_nxt <= trap_db_break_c;

    -- external halt request --
    elsif (CPU_EXTENSION_RISCV_DEBUG = true) and (trap_ctrl.irq_buf(interrupt_db_halt_c) = '1') then
      trap_ctrl.cause_nxt <= trap_db_halt_c;

    -- single stepping --
    elsif (CPU_EXTENSION_RISCV_DEBUG = true) and (trap_ctrl.irq_buf(interrupt_db_step_c) = '1') then
      trap_ctrl.cause_nxt <= trap_db_step_c;


    -- ----------------------------------------------------------------------------------------
    -- the following traps are caused by *asynchronous* exceptions (= interrupts)
    -- ----------------------------------------------------------------------------------------

    -- custom FAST interrupt requests --

    -- interrupt: 1.16 fast interrupt channel 0 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_0_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq0_c;

    -- interrupt: 1.17 fast interrupt channel 1 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_1_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq1_c;

    -- interrupt: 1.18 fast interrupt channel 2 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_2_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq2_c;

    -- interrupt: 1.19 fast interrupt channel 3 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_3_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq3_c;

    -- interrupt: 1.20 fast interrupt channel 4 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_4_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq4_c;

    -- interrupt: 1.21 fast interrupt channel 5 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_5_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq5_c;

    -- interrupt: 1.22 fast interrupt channel 6 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_6_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq6_c;

    -- interrupt: 1.23 fast interrupt channel 7 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_7_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq7_c;

    -- interrupt: 1.24 fast interrupt channel 8 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_8_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq8_c;

    -- interrupt: 1.25 fast interrupt channel 9 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_9_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq9_c;

    -- interrupt: 1.26 fast interrupt channel 10 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_10_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq10_c;

    -- interrupt: 1.27 fast interrupt channel 11 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_11_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq11_c;

    -- interrupt: 1.28 fast interrupt channel 12 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_12_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq12_c;

    -- interrupt: 1.29 fast interrupt channel 13 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_13_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq13_c;

    -- interrupt: 1.30 fast interrupt channel 14 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_14_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq14_c;

    -- interrupt: 1.31 fast interrupt channel 15 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_15_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq15_c;


    -- standard RISC-V interrupts --

    -- interrupt: 1.11 machine external interrupt --
    elsif (trap_ctrl.irq_buf(interrupt_mext_irq_c) = '1') then
      trap_ctrl.cause_nxt <= trap_mei_c;

    -- interrupt: 1.3 machine SW interrupt --
    elsif (trap_ctrl.irq_buf(interrupt_msw_irq_c) = '1') then
      trap_ctrl.cause_nxt <= trap_msi_c;

    -- interrupt: 1.7 machine timer interrupt --
    elsif (trap_ctrl.irq_buf(interrupt_mtime_irq_c) = '1') then
      trap_ctrl.cause_nxt <= trap_mti_c;

    end if;
  end process trap_priority;
  

-- ****************************************************************************************************************************
-- Control and Status Registers (CSRs)
-- ****************************************************************************************************************************

  -- Control and Status Registers Write Data ------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_write_data: process(execute_engine.i_reg, csr.rdata, rs1_i)
    variable csr_operand_v : std_ulogic_vector(data_width_c-1 downto 0);
  begin
    -- CSR operand source --
    if (execute_engine.i_reg(instr_funct3_msb_c) = '1') then -- immediate
      csr_operand_v := (others => '0');
      csr_operand_v(4 downto 0) := execute_engine.i_reg(19 downto 15); -- uimm5
    else -- register
      csr_operand_v := rs1_i;
    end if;
    -- tiny ALU for CSR write operations --
    case execute_engine.i_reg(instr_funct3_lsb_c+1 downto instr_funct3_lsb_c) is
      when "10"   => csr.wdata <= csr.rdata or csr_operand_v; -- CSRRS(I)
      when "11"   => csr.wdata <= csr.rdata and (not csr_operand_v); -- CSRRC(I)
      when others => csr.wdata <= csr_operand_v; -- CSRRW(I)
    end case;
  end process csr_write_data;


  -- Control and Status Registers - Write Access --------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_write_access: process(rstn_i, clk_i)
    variable cause_v : std_ulogic_vector(6 downto 0);
  begin
    -- NOTE: If <dedicated_reset_c> = true then <def_rst_val_c> evaluates to '-'. Register that reset to <def_rst_val_c> do
    -- NOT actually have a real reset by default (def_rst_val_c = '-') and have to be explicitly initialized by software!
    -- see: https://forums.xilinx.com/t5/General-Technical-Discussion/quot-Don-t-care-quot-reset-value/td-p/412845
    if (rstn_i = '0') then
      csr.we                <= '0';
      --
      csr.mstatus_mie       <= '0';
      csr.mstatus_mpie      <= '0';
      csr.mstatus_mpp       <= (others => '0');
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
      csr.mip_clr           <= (others => def_rst_val_c);
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
      csr.dcsr_prv          <= (others => def_rst_val_c);
      csr.dcsr_cause        <= (others => def_rst_val_c);
      csr.dpc               <= (others => def_rst_val_c);
      csr.dscratch0         <= (others => def_rst_val_c);

    elsif rising_edge(clk_i) then
      -- write access? --
      csr.we <= csr.we_nxt;

      -- defaults --
      csr.mip_clr <= (others => '0');

      if (CPU_EXTENSION_RISCV_Zicsr = true) then
        -- --------------------------------------------------------------------------------
        -- CSR access by application software
        -- --------------------------------------------------------------------------------
        if (csr.we = '1') and (trap_ctrl.exc_buf(exception_iillegal_c) = '0') then -- manual write access and not illegal instruction

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
                csr.mstatus_mpp(0) <= csr.wdata(11) or csr.wdata(12);
                csr.mstatus_mpp(1) <= csr.wdata(11) or csr.wdata(12);
              end if;
            end if;
            -- R/W: mie - machine interrupt enable register --
            if (csr.addr(2 downto 0) = csr_mie_c(2 downto 0)) then
              csr.mie_msie <= csr.wdata(03); -- machine SW IRQ enable
              csr.mie_mtie <= csr.wdata(07); -- machine TIMER IRQ enable
              csr.mie_meie <= csr.wdata(11); -- machine EXT IRQ enable
              for i in 0 to 15 loop -- fast interrupt channels 0..15
                csr.mie_firqe(i) <= csr.wdata(16+i);
              end loop; -- i
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
            if (csr.addr(3 downto 0) =  csr_mip_c(3 downto 0)) then
              csr.mip_clr <= csr.wdata(31 downto 16);
            end if;
          end if;

          -- physical memory protection: R/W: pmpcfg* - PMP configuration registers --
          -- --------------------------------------------------------------------
          if (PMP_NUM_REGIONS > 0) then
            if (csr.addr(11 downto 4) = csr_class_pmpcfg_c) then -- pmp configuration CSR class
              for i in 0 to PMP_NUM_REGIONS-1 loop
                if (csr.addr(3 downto 0) = std_ulogic_vector(to_unsigned(i, 4))) then
                  if (csr.pmpcfg(i)(7) = '0') then -- unlocked pmpcfg access
                    csr.pmpcfg(i)(0) <= csr.wdata((i mod 4)*8+0); -- R (rights.read)
                    csr.pmpcfg(i)(1) <= csr.wdata((i mod 4)*8+1); -- W (rights.write)
                    csr.pmpcfg(i)(2) <= csr.wdata((i mod 4)*8+2); -- X (rights.execute)
                    csr.pmpcfg(i)(3) <= csr.wdata((i mod 4)*8+3) and csr.wdata((i mod 4)*8+4); -- A_L
                    csr.pmpcfg(i)(4) <= csr.wdata((i mod 4)*8+3) and csr.wdata((i mod 4)*8+4); -- A_H - NAPOT/OFF only
                    csr.pmpcfg(i)(5) <= '0'; -- reserved
                    csr.pmpcfg(i)(6) <= '0'; -- reserved
                    csr.pmpcfg(i)(7) <= csr.wdata((i mod 4)*8+7); -- L (locked / rights also enforced in m-mode)
                  end if;
                end if;
              end loop; -- i (PMP regions)
            end if;
          end if;

          -- physical memory protection: R/W: pmpaddr* - PMP address registers --
          -- --------------------------------------------------------------------
          if (PMP_NUM_REGIONS > 0) then
            if (csr.addr(11 downto 4) =  csr_pmpaddr0_c(11 downto 4)) or (csr.addr(11 downto 4) = csr_pmpaddr16_c(11 downto 4)) or
               (csr.addr(11 downto 4) = csr_pmpaddr32_c(11 downto 4)) or (csr.addr(11 downto 4) = csr_pmpaddr48_c(11 downto 4)) then 
              for i in 0 to PMP_NUM_REGIONS-1 loop
                if (csr.addr(6 downto 0) = std_ulogic_vector(unsigned(csr_pmpaddr0_c(6 downto 0)) + i)) and (csr.pmpcfg(i)(7) = '0') then -- unlocked pmpaddr access
                  csr.pmpaddr(i) <= csr.wdata;
                  csr.pmpaddr(i)(index_size_f(PMP_MIN_GRANULARITY)-4 downto 0) <= (others => '1');
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
              if (HPM_NUM_CNTS > 0) then -- any HPMs available?
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
                  csr.dcsr_prv(0)  <= csr.wdata(1) or csr.wdata(0);
                  csr.dcsr_prv(1)  <= csr.wdata(1) or csr.wdata(0);
                else -- only machine mode is available
                  csr.dcsr_prv <= priv_mode_m_c;
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


        -- --------------------------------------------------------------------------------
        -- CSR access by hardware
        -- --------------------------------------------------------------------------------
        else

          -- floating-point (FPU) exception flags --
          -- --------------------------------------------------------------------
          if (CPU_EXTENSION_RISCV_Zfinx = true) and (trap_ctrl.exc_buf(exception_iillegal_c) = '0') then -- no illegal instruction
            csr.fflags <= csr.fflags or fpu_flags_i; -- accumulate flags ("accrued exception flags")
          end if;

          -- mcause, mepc, mtval: write machine trap cause, PC and trap value register --
          -- --------------------------------------------------------------------
          if (trap_ctrl.env_start_ack = '1') then -- trap handler starting?

            if (CPU_EXTENSION_RISCV_DEBUG = false) or ((trap_ctrl.cause(5) = '0') and -- update mtval/mepc/mcause only when NOT ENTRY debug mode exception
                                                       (debug_ctrl.running = '0')) then -- and NOT IN debug mode

              -- trap cause ID code --
              csr.mcause(csr.mcause'left) <= trap_ctrl.cause(trap_ctrl.cause'left); -- 1: interrupt, 0: exception
              csr.mcause(4 downto 0)      <= trap_ctrl.cause(4 downto 0); -- identifier

              -- trap PC --
              if (trap_ctrl.cause(trap_ctrl.cause'left) = '1') then -- for INTERRUPTS (async source)
                csr.mepc <= execute_engine.pc(data_width_c-1 downto 1) & '0'; -- this is the CURRENT pc = interrupted instruction
              else -- for sync. EXCEPTIONS (sync source)
                csr.mepc <= execute_engine.last_pc(data_width_c-1 downto 1) & '0'; -- this is the LAST pc = last executed instruction
              end if;

              -- trap value --
              cause_v := trap_ctrl.cause;
              cause_v(5) := '0'; -- bit 5 is always zero here (= normal trapping), so we do not need to check that again
              case cause_v is
                when trap_ima_c | trap_iba_c => -- misaligned instruction address OR instruction access error
                  csr.mtval <= execute_engine.pc(data_width_c-1 downto 1) & '0'; -- address of faulting instruction
                when trap_brk_c => -- breakpoint
                  csr.mtval <= execute_engine.last_pc(data_width_c-1 downto 1) & '0'; -- address of breakpoint instruction
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
            if (CPU_EXTENSION_RISCV_DEBUG = true) and (trap_ctrl.cause(5) = '1') and (debug_ctrl.running = '0') then -- debug mode entry exception

              -- trap cause ID code --
              csr.dcsr_cause <= trap_ctrl.cause(2 downto 0); -- why did we enter debug mode?
              -- current privilege mode when debug mode was entered --
              csr.dcsr_prv <= csr.privilege;

              -- trap PC --
              if (trap_ctrl.cause(trap_ctrl.cause'left) = '1') then -- for INTERRUPTS (async source)
                csr.dpc <= execute_engine.pc(data_width_c-1 downto 1) & '0'; -- this is the CURRENT pc = interrupted instruction
              else -- for sync. EXCEPTIONS (sync source)
                csr.dpc <= execute_engine.last_pc(data_width_c-1 downto 1) & '0'; -- this is the LAST pc = last executed instruction
              end if;

            end if;

          end if;

          -- mstatus: context switch --
          -- --------------------------------------------------------------------
          -- ENTER: trap handling starting?
          if (trap_ctrl.env_start_ack = '1') then -- trap handler starting?

            if (CPU_EXTENSION_RISCV_DEBUG = false) or -- normal trapping (debug mode NOT implemented)
               ((debug_ctrl.running = '0') and (trap_ctrl.cause(5) = '0')) then -- not IN debug mode and not ENTERING debug mode
              csr.mstatus_mie  <= '0'; -- disable interrupts
              csr.mstatus_mpie <= csr.mstatus_mie; -- buffer previous mie state
              if (CPU_EXTENSION_RISCV_U = true) then -- implement user mode
                csr.privilege   <= priv_mode_m_c; -- execute trap in machine mode
                csr.mstatus_mpp <= csr.privilege; -- buffer previous privilege mode
              end if;
            end if;

          -- EXIT: return from exception
          elsif (trap_ctrl.env_end = '1') then
            if (CPU_EXTENSION_RISCV_DEBUG = true) and (debug_ctrl.running = '1') then -- return from debug mode
              if (CPU_EXTENSION_RISCV_U = true) then -- implement user mode
                csr.privilege <= csr.dcsr_prv;
              end if;
            else -- return from "normal trap"
              csr.mstatus_mie  <= csr.mstatus_mpie; -- restore global IRQ enable flag
              csr.mstatus_mpie <= '1';
              if (CPU_EXTENSION_RISCV_U = true) then -- implement user mode
                csr.privilege   <= csr.mstatus_mpp; -- go back to previous privilege mode
                csr.mstatus_mpp <= (others => '0');
              end if;
            end if;
          end if;

        end if; -- /hardware csr access
      end if;

      -- --------------------------------------------------------------------------------
      -- override write access for disabled functions
      -- --------------------------------------------------------------------------------

      -- user mode disabled --
      if (CPU_EXTENSION_RISCV_U = false) then
        csr.privilege     <= priv_mode_m_c;
        csr.mstatus_mpp   <= priv_mode_m_c;
        csr.mcounteren_cy <= '0';
        csr.mcounteren_tm <= '0';
        csr.mcounteren_ir <= '0';
        csr.dcsr_ebreaku  <= '0';
        csr.dcsr_prv      <= priv_mode_m_c;
      end if;

      -- pmp disabled --
      if (PMP_NUM_REGIONS = 0) then
        csr.pmpcfg  <= (others => (others => '0'));
        csr.pmpaddr <= (others => (others => '1'));
      end if;

      -- hpms disabled --
      if (HPM_NUM_CNTS = 0) then
        csr.mhpmevent         <= (others => (others => '0'));
        csr.mcountinhibit_hpm <= (others => '0');
      end if;

      -- cpu counters disabled --
      if (CPU_CNT_WIDTH = 0) then
        csr.mcounteren_cy    <= '0';
        csr.mcounteren_ir    <= '0';
        csr.mcountinhibit_cy <= '0';
        csr.mcountinhibit_ir <= '0';
      end if;

      -- floating-point extension disabled --
      if (CPU_EXTENSION_RISCV_Zfinx = false) then
        csr.fflags <= (others => '0');
        csr.frm    <= (others => '0');
      end if;

      -- debug mode disabled --
      if (CPU_EXTENSION_RISCV_DEBUG = false) then
        csr.dcsr_ebreakm <= '0';
        csr.dcsr_ebreaku <= '0';
        csr.dcsr_step    <= '0';
        csr.dcsr_cause   <= (others => '0');
        csr.dpc          <= (others => '0');
        csr.dscratch0    <= (others => '0');
      end if;

    end if;
  end process csr_write_access;

  -- decode current privilege mode --
  csr.privilege_rd <= priv_mode_m_c when (CPU_EXTENSION_RISCV_DEBUG = true) and (debug_ctrl.running = '1') else csr.privilege; -- effective privilege mode ("machine" when in debug mode)
  csr.priv_m_mode  <= '1' when (csr.privilege_rd = priv_mode_m_c) else '0';
  csr.priv_u_mode  <= '1' when (csr.privilege_rd = priv_mode_u_c) and (CPU_EXTENSION_RISCV_U = true) else '0';

  -- PMP configuration output to bus unit --
  pmp_output: process(csr)
  begin
    pmp_addr_o <= (others => (others => '0'));
    pmp_ctrl_o <= (others => (others => '0'));
    if (PMP_NUM_REGIONS /= 0) then
      for i in 0 to PMP_NUM_REGIONS-1 loop
        pmp_addr_o(i) <= csr.pmpaddr(i) & "11";
        pmp_addr_o(i)(index_size_f(PMP_MIN_GRANULARITY)-4 downto 0) <= (others => '1');
        pmp_ctrl_o(i) <= csr.pmpcfg(i);
      end loop; -- i
    end if;
  end process pmp_output;

  -- PMP config read dummy --
  pmp_rd_dummy: process(csr)
  begin
    csr.pmpcfg_rd  <= (others => (others => '0'));
    if (PMP_NUM_REGIONS /= 0) then
      for i in 0 to PMP_NUM_REGIONS-1 loop
        csr.pmpcfg_rd(i)  <= csr.pmpcfg(i);
      end loop; -- i
    end if;
  end process pmp_rd_dummy;


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
        elsif (csr.mcountinhibit_cy = '0') and (cnt_event(hpmcnt_event_cy_c) = '1') then -- non-inhibited automatic update
          csr.mcycle(cpu_cnt_lo_width_c-1 downto 0) <= csr.mcycle_nxt(cpu_cnt_lo_width_c-1 downto 0);
        end if;
      else
        csr.mcycle <= (others => '-');
        csr.mcycle_ovfl(0) <= '-';
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
        elsif (csr.mcountinhibit_ir = '0') and (cnt_event(hpmcnt_event_ir_c) = '1') then -- non-inhibited automatic update
          csr.minstret(cpu_cnt_lo_width_c-1 downto 0) <= csr.minstret_nxt(cpu_cnt_lo_width_c-1 downto 0);
        end if;
      else
        csr.minstret <= (others => '-');
        csr.minstret_ovfl(0) <= '-';
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
          csr.mhpmcounter(i) <= (others => '-');
          csr.mhpmcounter_ovfl(i)(0) <= '-';
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
    csr.mhpmcounter_rd  <= (others => (others => '0'));
    csr.mhpmcounterh_rd <= (others => (others => '0'));
    if (HPM_NUM_CNTS /= 0) and (CPU_EXTENSION_RISCV_Zihpm = true) then
      for i in 0 to HPM_NUM_CNTS-1 loop
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
  hpmcnt_ctrl: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      hpmcnt_trigger <= (others => def_rst_val_c);
    elsif rising_edge(clk_i) then
      -- enable selected triggers by ANDing actual events and according CSR configuration bits --
      -- OR everything to see if counter should increment --
      hpmcnt_trigger <= (others => '0'); -- default
      if (HPM_NUM_CNTS /= 0) then
        for i in 0 to HPM_NUM_CNTS-1 loop
          hpmcnt_trigger(i) <= or_reduce_f(cnt_event and csr.mhpmevent(i)(cnt_event'left downto 0));
        end loop; -- i
      end if;
    end if;
  end process hpmcnt_ctrl;

  -- counter event trigger - RISC-V-specific --
  cnt_event(hpmcnt_event_cy_c)      <= not execute_engine.sleep; -- active cycle
  cnt_event(hpmcnt_event_never_c)   <= '0'; -- undefined (never)
  cnt_event(hpmcnt_event_ir_c)      <= '1' when (execute_engine.state = EXECUTE) else '0'; -- retired instruction

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


  -- Control and Status Registers - Read Access ---------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_read_access: process(rstn_i, clk_i)
    variable csr_addr_v : std_ulogic_vector(11 downto 0);
  begin
    if rising_edge(clk_i) then
      csr.rdata <= (others => '0'); -- default output
      if (CPU_EXTENSION_RISCV_Zicsr = true) then
        csr_addr_v(11 downto 10) := csr.addr(11 downto 10);
        csr_addr_v(09 downto 08) := (others => csr.addr(8)); -- !!! WARNING: MACHINE (11) and USER (00) registers ONLY !!!
        csr_addr_v(07 downto 00) := csr.addr(07 downto 00);
        case csr_addr_v is

          -- floating-point CSRs --
          -- --------------------------------------------------------------------
          when csr_fflags_c => -- fflags (r/w): floating-point (FPU) exception flags
            if (CPU_EXTENSION_RISCV_Zfinx = true) then csr.rdata(4 downto 0) <= csr.fflags; else NULL; end if;
          when csr_frm_c => -- frm (r/w): floating-point (FPU) rounding mode
            if (CPU_EXTENSION_RISCV_Zfinx = true) then csr.rdata(2 downto 0) <= csr.frm; else NULL; end if;
          when csr_fcsr_c => -- fcsr (r/w): floating-point (FPU) control/status (frm + fflags)
            if (CPU_EXTENSION_RISCV_Zfinx = true) then csr.rdata(7 downto 5) <= csr.frm; csr.rdata(4 downto 0) <= csr.fflags; else NULL; end if;

          -- machine trap setup --
          -- --------------------------------------------------------------------
          when csr_mstatus_c => -- mstatus (r/w): machine status register
            csr.rdata(03) <= csr.mstatus_mie; -- MIE
            csr.rdata(07) <= csr.mstatus_mpie; -- MPIE
            csr.rdata(11) <= csr.mstatus_mpp(0); -- MPP: machine previous privilege mode low
            csr.rdata(12) <= csr.mstatus_mpp(1); -- MPP: machine previous privilege mode high
          when csr_misa_c => -- misa (r/-): ISA and extensions
            csr.rdata(00) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_A);     -- A CPU extension
            csr.rdata(01) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_B);     -- B CPU extension
            csr.rdata(02) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_C);     -- C CPU extension
            csr.rdata(04) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_E);     -- E CPU extension
            csr.rdata(08) <= not bool_to_ulogic_f(CPU_EXTENSION_RISCV_E); -- I CPU extension (if not E)
            csr.rdata(12) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_M);     -- M CPU extension
            csr.rdata(20) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_U);     -- U CPU extension
            csr.rdata(23) <= '1';                                         -- X CPU extension (non-std extensions)
            csr.rdata(30) <= '1'; -- 32-bit architecture (MXL lo)
            csr.rdata(31) <= '0'; -- 32-bit architecture (MXL hi)
          when csr_mie_c => -- mie (r/w): machine interrupt-enable register
            csr.rdata(03) <= csr.mie_msie; -- machine software IRQ enable
            csr.rdata(07) <= csr.mie_mtie; -- machine timer IRQ enable
            csr.rdata(11) <= csr.mie_meie; -- machine external IRQ enable
            for i in 0 to 15 loop -- fast interrupt channels 0..15 enable
              csr.rdata(16+i) <= csr.mie_firqe(i);
            end loop; -- i
          when csr_mtvec_c => -- mtvec (r/w): machine trap-handler base address (for ALL exceptions)
            csr.rdata <= csr.mtvec(data_width_c-1 downto 2) & "00"; -- mtvec.MODE=0
          when csr_mcounteren_c => -- mcounteren (r/w): machine counter enable register
            if (CPU_EXTENSION_RISCV_U = false) then -- this CSR is hardwired to zero if user mode is not implemented
              NULL;
            else
              csr.rdata(0) <= csr.mcounteren_cy; -- enable user-level access to cycle[h]
              csr.rdata(1) <= csr.mcounteren_tm; -- enable user-level access to time[h]
              csr.rdata(2) <= csr.mcounteren_ir; -- enable user-level access to instret[h]
            end if;

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
            csr.rdata(03) <= trap_ctrl.irq_buf(interrupt_msw_irq_c);
            csr.rdata(07) <= trap_ctrl.irq_buf(interrupt_mtime_irq_c);
            csr.rdata(11) <= trap_ctrl.irq_buf(interrupt_mext_irq_c);
            for i in 0 to 15 loop -- fast interrupt channels 0..15 pending
              csr.rdata(16+i) <= trap_ctrl.irq_buf(interrupt_firq_0_c+i);
            end loop; -- i

          -- physical memory protection - configuration (r/w) --
          -- --------------------------------------------------------------------
          when csr_pmpcfg0_c  => if (PMP_NUM_REGIONS > 00) then csr.rdata <= csr.pmpcfg_rd(03) & csr.pmpcfg_rd(02) & csr.pmpcfg_rd(01) & csr.pmpcfg_rd(00); else NULL; end if;
          when csr_pmpcfg1_c  => if (PMP_NUM_REGIONS > 03) then csr.rdata <= csr.pmpcfg_rd(07) & csr.pmpcfg_rd(06) & csr.pmpcfg_rd(05) & csr.pmpcfg_rd(04); else NULL; end if;
          when csr_pmpcfg2_c  => if (PMP_NUM_REGIONS > 07) then csr.rdata <= csr.pmpcfg_rd(11) & csr.pmpcfg_rd(10) & csr.pmpcfg_rd(09) & csr.pmpcfg_rd(08); else NULL; end if;
          when csr_pmpcfg3_c  => if (PMP_NUM_REGIONS > 11) then csr.rdata <= csr.pmpcfg_rd(15) & csr.pmpcfg_rd(14) & csr.pmpcfg_rd(13) & csr.pmpcfg_rd(12); else NULL; end if;
          when csr_pmpcfg4_c  => if (PMP_NUM_REGIONS > 15) then csr.rdata <= csr.pmpcfg_rd(19) & csr.pmpcfg_rd(18) & csr.pmpcfg_rd(17) & csr.pmpcfg_rd(16); else NULL; end if;
          when csr_pmpcfg5_c  => if (PMP_NUM_REGIONS > 19) then csr.rdata <= csr.pmpcfg_rd(23) & csr.pmpcfg_rd(22) & csr.pmpcfg_rd(21) & csr.pmpcfg_rd(20); else NULL; end if;
          when csr_pmpcfg6_c  => if (PMP_NUM_REGIONS > 23) then csr.rdata <= csr.pmpcfg_rd(27) & csr.pmpcfg_rd(26) & csr.pmpcfg_rd(25) & csr.pmpcfg_rd(24); else NULL; end if;
          when csr_pmpcfg7_c  => if (PMP_NUM_REGIONS > 27) then csr.rdata <= csr.pmpcfg_rd(31) & csr.pmpcfg_rd(30) & csr.pmpcfg_rd(29) & csr.pmpcfg_rd(28); else NULL; end if;
          when csr_pmpcfg8_c  => if (PMP_NUM_REGIONS > 31) then csr.rdata <= csr.pmpcfg_rd(35) & csr.pmpcfg_rd(34) & csr.pmpcfg_rd(33) & csr.pmpcfg_rd(32); else NULL; end if;
          when csr_pmpcfg9_c  => if (PMP_NUM_REGIONS > 35) then csr.rdata <= csr.pmpcfg_rd(39) & csr.pmpcfg_rd(38) & csr.pmpcfg_rd(37) & csr.pmpcfg_rd(36); else NULL; end if;
          when csr_pmpcfg10_c => if (PMP_NUM_REGIONS > 39) then csr.rdata <= csr.pmpcfg_rd(43) & csr.pmpcfg_rd(42) & csr.pmpcfg_rd(41) & csr.pmpcfg_rd(40); else NULL; end if;
          when csr_pmpcfg11_c => if (PMP_NUM_REGIONS > 43) then csr.rdata <= csr.pmpcfg_rd(47) & csr.pmpcfg_rd(46) & csr.pmpcfg_rd(45) & csr.pmpcfg_rd(44); else NULL; end if;
          when csr_pmpcfg12_c => if (PMP_NUM_REGIONS > 47) then csr.rdata <= csr.pmpcfg_rd(51) & csr.pmpcfg_rd(50) & csr.pmpcfg_rd(49) & csr.pmpcfg_rd(48); else NULL; end if;
          when csr_pmpcfg13_c => if (PMP_NUM_REGIONS > 51) then csr.rdata <= csr.pmpcfg_rd(55) & csr.pmpcfg_rd(54) & csr.pmpcfg_rd(53) & csr.pmpcfg_rd(52); else NULL; end if;
          when csr_pmpcfg14_c => if (PMP_NUM_REGIONS > 55) then csr.rdata <= csr.pmpcfg_rd(59) & csr.pmpcfg_rd(58) & csr.pmpcfg_rd(57) & csr.pmpcfg_rd(56); else NULL; end if;
          when csr_pmpcfg15_c => if (PMP_NUM_REGIONS > 59) then csr.rdata <= csr.pmpcfg_rd(63) & csr.pmpcfg_rd(62) & csr.pmpcfg_rd(61) & csr.pmpcfg_rd(60); else NULL; end if;

          -- physical memory protection - addresses (r/w) --
          -- --------------------------------------------------------------------
          when csr_pmpaddr0_c  => if (PMP_NUM_REGIONS > 00) then csr.rdata <= csr.pmpaddr(00); else NULL; end if;
          when csr_pmpaddr1_c  => if (PMP_NUM_REGIONS > 01) then csr.rdata <= csr.pmpaddr(01); else NULL; end if;
          when csr_pmpaddr2_c  => if (PMP_NUM_REGIONS > 02) then csr.rdata <= csr.pmpaddr(02); else NULL; end if;
          when csr_pmpaddr3_c  => if (PMP_NUM_REGIONS > 03) then csr.rdata <= csr.pmpaddr(03); else NULL; end if;
          when csr_pmpaddr4_c  => if (PMP_NUM_REGIONS > 04) then csr.rdata <= csr.pmpaddr(04); else NULL; end if;
          when csr_pmpaddr5_c  => if (PMP_NUM_REGIONS > 05) then csr.rdata <= csr.pmpaddr(05); else NULL; end if;
          when csr_pmpaddr6_c  => if (PMP_NUM_REGIONS > 06) then csr.rdata <= csr.pmpaddr(06); else NULL; end if;
          when csr_pmpaddr7_c  => if (PMP_NUM_REGIONS > 07) then csr.rdata <= csr.pmpaddr(07); else NULL; end if;
          when csr_pmpaddr8_c  => if (PMP_NUM_REGIONS > 08) then csr.rdata <= csr.pmpaddr(08); else NULL; end if;
          when csr_pmpaddr9_c  => if (PMP_NUM_REGIONS > 09) then csr.rdata <= csr.pmpaddr(09); else NULL; end if;
          when csr_pmpaddr10_c => if (PMP_NUM_REGIONS > 10) then csr.rdata <= csr.pmpaddr(10); else NULL; end if;
          when csr_pmpaddr11_c => if (PMP_NUM_REGIONS > 11) then csr.rdata <= csr.pmpaddr(11); else NULL; end if;
          when csr_pmpaddr12_c => if (PMP_NUM_REGIONS > 12) then csr.rdata <= csr.pmpaddr(12); else NULL; end if;
          when csr_pmpaddr13_c => if (PMP_NUM_REGIONS > 13) then csr.rdata <= csr.pmpaddr(13); else NULL; end if;
          when csr_pmpaddr14_c => if (PMP_NUM_REGIONS > 14) then csr.rdata <= csr.pmpaddr(14); else NULL; end if;
          when csr_pmpaddr15_c => if (PMP_NUM_REGIONS > 15) then csr.rdata <= csr.pmpaddr(15); else NULL; end if;
          when csr_pmpaddr16_c => if (PMP_NUM_REGIONS > 16) then csr.rdata <= csr.pmpaddr(16); else NULL; end if;
          when csr_pmpaddr17_c => if (PMP_NUM_REGIONS > 17) then csr.rdata <= csr.pmpaddr(17); else NULL; end if;
          when csr_pmpaddr18_c => if (PMP_NUM_REGIONS > 18) then csr.rdata <= csr.pmpaddr(18); else NULL; end if;
          when csr_pmpaddr19_c => if (PMP_NUM_REGIONS > 19) then csr.rdata <= csr.pmpaddr(19); else NULL; end if;
          when csr_pmpaddr20_c => if (PMP_NUM_REGIONS > 20) then csr.rdata <= csr.pmpaddr(20); else NULL; end if;
          when csr_pmpaddr21_c => if (PMP_NUM_REGIONS > 21) then csr.rdata <= csr.pmpaddr(21); else NULL; end if;
          when csr_pmpaddr22_c => if (PMP_NUM_REGIONS > 22) then csr.rdata <= csr.pmpaddr(22); else NULL; end if;
          when csr_pmpaddr23_c => if (PMP_NUM_REGIONS > 23) then csr.rdata <= csr.pmpaddr(23); else NULL; end if;
          when csr_pmpaddr24_c => if (PMP_NUM_REGIONS > 24) then csr.rdata <= csr.pmpaddr(24); else NULL; end if;
          when csr_pmpaddr25_c => if (PMP_NUM_REGIONS > 25) then csr.rdata <= csr.pmpaddr(25); else NULL; end if;
          when csr_pmpaddr26_c => if (PMP_NUM_REGIONS > 26) then csr.rdata <= csr.pmpaddr(26); else NULL; end if;
          when csr_pmpaddr27_c => if (PMP_NUM_REGIONS > 27) then csr.rdata <= csr.pmpaddr(27); else NULL; end if;
          when csr_pmpaddr28_c => if (PMP_NUM_REGIONS > 28) then csr.rdata <= csr.pmpaddr(28); else NULL; end if;
          when csr_pmpaddr29_c => if (PMP_NUM_REGIONS > 29) then csr.rdata <= csr.pmpaddr(29); else NULL; end if;
          when csr_pmpaddr30_c => if (PMP_NUM_REGIONS > 30) then csr.rdata <= csr.pmpaddr(30); else NULL; end if;
          when csr_pmpaddr31_c => if (PMP_NUM_REGIONS > 31) then csr.rdata <= csr.pmpaddr(31); else NULL; end if;
          when csr_pmpaddr32_c => if (PMP_NUM_REGIONS > 32) then csr.rdata <= csr.pmpaddr(32); else NULL; end if;
          when csr_pmpaddr33_c => if (PMP_NUM_REGIONS > 33) then csr.rdata <= csr.pmpaddr(33); else NULL; end if;
          when csr_pmpaddr34_c => if (PMP_NUM_REGIONS > 34) then csr.rdata <= csr.pmpaddr(34); else NULL; end if;
          when csr_pmpaddr35_c => if (PMP_NUM_REGIONS > 35) then csr.rdata <= csr.pmpaddr(35); else NULL; end if;
          when csr_pmpaddr36_c => if (PMP_NUM_REGIONS > 36) then csr.rdata <= csr.pmpaddr(36); else NULL; end if;
          when csr_pmpaddr37_c => if (PMP_NUM_REGIONS > 37) then csr.rdata <= csr.pmpaddr(37); else NULL; end if;
          when csr_pmpaddr38_c => if (PMP_NUM_REGIONS > 38) then csr.rdata <= csr.pmpaddr(38); else NULL; end if;
          when csr_pmpaddr39_c => if (PMP_NUM_REGIONS > 39) then csr.rdata <= csr.pmpaddr(39); else NULL; end if;
          when csr_pmpaddr40_c => if (PMP_NUM_REGIONS > 40) then csr.rdata <= csr.pmpaddr(40); else NULL; end if;
          when csr_pmpaddr41_c => if (PMP_NUM_REGIONS > 41) then csr.rdata <= csr.pmpaddr(41); else NULL; end if;
          when csr_pmpaddr42_c => if (PMP_NUM_REGIONS > 42) then csr.rdata <= csr.pmpaddr(42); else NULL; end if;
          when csr_pmpaddr43_c => if (PMP_NUM_REGIONS > 43) then csr.rdata <= csr.pmpaddr(43); else NULL; end if;
          when csr_pmpaddr44_c => if (PMP_NUM_REGIONS > 44) then csr.rdata <= csr.pmpaddr(44); else NULL; end if;
          when csr_pmpaddr45_c => if (PMP_NUM_REGIONS > 45) then csr.rdata <= csr.pmpaddr(45); else NULL; end if;
          when csr_pmpaddr46_c => if (PMP_NUM_REGIONS > 46) then csr.rdata <= csr.pmpaddr(46); else NULL; end if;
          when csr_pmpaddr47_c => if (PMP_NUM_REGIONS > 47) then csr.rdata <= csr.pmpaddr(47); else NULL; end if;
          when csr_pmpaddr48_c => if (PMP_NUM_REGIONS > 48) then csr.rdata <= csr.pmpaddr(48); else NULL; end if;
          when csr_pmpaddr49_c => if (PMP_NUM_REGIONS > 49) then csr.rdata <= csr.pmpaddr(49); else NULL; end if;
          when csr_pmpaddr50_c => if (PMP_NUM_REGIONS > 50) then csr.rdata <= csr.pmpaddr(50); else NULL; end if;
          when csr_pmpaddr51_c => if (PMP_NUM_REGIONS > 51) then csr.rdata <= csr.pmpaddr(51); else NULL; end if;
          when csr_pmpaddr52_c => if (PMP_NUM_REGIONS > 52) then csr.rdata <= csr.pmpaddr(52); else NULL; end if;
          when csr_pmpaddr53_c => if (PMP_NUM_REGIONS > 53) then csr.rdata <= csr.pmpaddr(53); else NULL; end if;
          when csr_pmpaddr54_c => if (PMP_NUM_REGIONS > 54) then csr.rdata <= csr.pmpaddr(54); else NULL; end if;
          when csr_pmpaddr55_c => if (PMP_NUM_REGIONS > 55) then csr.rdata <= csr.pmpaddr(55); else NULL; end if;
          when csr_pmpaddr56_c => if (PMP_NUM_REGIONS > 56) then csr.rdata <= csr.pmpaddr(56); else NULL; end if;
          when csr_pmpaddr57_c => if (PMP_NUM_REGIONS > 57) then csr.rdata <= csr.pmpaddr(57); else NULL; end if;
          when csr_pmpaddr58_c => if (PMP_NUM_REGIONS > 58) then csr.rdata <= csr.pmpaddr(58); else NULL; end if;
          when csr_pmpaddr59_c => if (PMP_NUM_REGIONS > 59) then csr.rdata <= csr.pmpaddr(59); else NULL; end if;
          when csr_pmpaddr60_c => if (PMP_NUM_REGIONS > 60) then csr.rdata <= csr.pmpaddr(60); else NULL; end if;
          when csr_pmpaddr61_c => if (PMP_NUM_REGIONS > 61) then csr.rdata <= csr.pmpaddr(61); else NULL; end if;
          when csr_pmpaddr62_c => if (PMP_NUM_REGIONS > 62) then csr.rdata <= csr.pmpaddr(62); else NULL; end if;
          when csr_pmpaddr63_c => if (PMP_NUM_REGIONS > 63) then csr.rdata <= csr.pmpaddr(63); else NULL; end if;

          -- machine counter setup --
          -- --------------------------------------------------------------------
          when csr_mcountinhibit_c => -- mcountinhibit (r/w): machine counter-inhibit register
            csr.rdata(0) <= csr.mcountinhibit_cy; -- enable auto-increment of [m]cycle[h] counter
            csr.rdata(2) <= csr.mcountinhibit_ir; -- enable auto-increment of [m]instret[h] counter
            if (HPM_NUM_CNTS > 0) then -- any HPMs available?
              csr.rdata(csr.mcountinhibit_hpm'left+3 downto 3) <= csr.mcountinhibit_hpm; -- enable auto-increment of [m]hpmcounterx[h] counter
            end if;

          -- machine performance-monitoring event selector (r/w) --
          -- --------------------------------------------------------------------
          when csr_mhpmevent3_c  => if (HPM_NUM_CNTS > 00) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(00); else NULL; end if;
          when csr_mhpmevent4_c  => if (HPM_NUM_CNTS > 01) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(01); else NULL; end if;
          when csr_mhpmevent5_c  => if (HPM_NUM_CNTS > 02) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(02); else NULL; end if;
          when csr_mhpmevent6_c  => if (HPM_NUM_CNTS > 03) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(03); else NULL; end if;
          when csr_mhpmevent7_c  => if (HPM_NUM_CNTS > 04) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(04); else NULL; end if;
          when csr_mhpmevent8_c  => if (HPM_NUM_CNTS > 05) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(05); else NULL; end if;
          when csr_mhpmevent9_c  => if (HPM_NUM_CNTS > 06) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(06); else NULL; end if;
          when csr_mhpmevent10_c => if (HPM_NUM_CNTS > 07) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(07); else NULL; end if;
          when csr_mhpmevent11_c => if (HPM_NUM_CNTS > 08) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(08); else NULL; end if;
          when csr_mhpmevent12_c => if (HPM_NUM_CNTS > 09) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(09); else NULL; end if;
          when csr_mhpmevent13_c => if (HPM_NUM_CNTS > 10) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(10); else NULL; end if;
          when csr_mhpmevent14_c => if (HPM_NUM_CNTS > 11) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(11); else NULL; end if;
          when csr_mhpmevent15_c => if (HPM_NUM_CNTS > 12) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(12); else NULL; end if;
          when csr_mhpmevent16_c => if (HPM_NUM_CNTS > 13) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(13); else NULL; end if;
          when csr_mhpmevent17_c => if (HPM_NUM_CNTS > 14) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(14); else NULL; end if;
          when csr_mhpmevent18_c => if (HPM_NUM_CNTS > 15) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(15); else NULL; end if;
          when csr_mhpmevent19_c => if (HPM_NUM_CNTS > 16) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(16); else NULL; end if;
          when csr_mhpmevent20_c => if (HPM_NUM_CNTS > 17) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(17); else NULL; end if;
          when csr_mhpmevent21_c => if (HPM_NUM_CNTS > 18) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(18); else NULL; end if;
          when csr_mhpmevent22_c => if (HPM_NUM_CNTS > 19) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(19); else NULL; end if;
          when csr_mhpmevent23_c => if (HPM_NUM_CNTS > 20) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(20); else NULL; end if;
          when csr_mhpmevent24_c => if (HPM_NUM_CNTS > 21) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(21); else NULL; end if;
          when csr_mhpmevent25_c => if (HPM_NUM_CNTS > 22) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(22); else NULL; end if;
          when csr_mhpmevent26_c => if (HPM_NUM_CNTS > 23) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(23); else NULL; end if;
          when csr_mhpmevent27_c => if (HPM_NUM_CNTS > 24) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(24); else NULL; end if;
          when csr_mhpmevent28_c => if (HPM_NUM_CNTS > 25) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(25); else NULL; end if;
          when csr_mhpmevent29_c => if (HPM_NUM_CNTS > 26) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(26); else NULL; end if;
          when csr_mhpmevent30_c => if (HPM_NUM_CNTS > 27) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(27); else NULL; end if;
          when csr_mhpmevent31_c => if (HPM_NUM_CNTS > 28) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata(hpmcnt_event_size_c-1 downto 0) <= csr.mhpmevent(28); else NULL; end if;

          -- counters and timers --
          -- --------------------------------------------------------------------
          when csr_cycle_c | csr_mcycle_c => -- [m]cycle (r/w): Cycle counter LOW
            if (cpu_cnt_lo_width_c > 0) and (CPU_EXTENSION_RISCV_Zicntr = true) then csr.rdata(cpu_cnt_lo_width_c-1 downto 0) <= csr.mcycle(cpu_cnt_lo_width_c-1 downto 0); else NULL; end if;
          when csr_cycleh_c | csr_mcycleh_c => -- [m]cycleh (r/w): Cycle counter HIGH
            if (cpu_cnt_hi_width_c > 0) and (CPU_EXTENSION_RISCV_Zicntr = true) then csr.rdata(cpu_cnt_hi_width_c-1 downto 0) <= csr.mcycleh(cpu_cnt_hi_width_c-1 downto 0); else NULL; end if;

          when csr_instret_c | csr_minstret_c => -- [m]instret (r/w): Instructions-retired counter LOW
            if (cpu_cnt_lo_width_c > 0) and (CPU_EXTENSION_RISCV_Zicntr = true) then csr.rdata(cpu_cnt_lo_width_c-1 downto 0) <= csr.minstret(cpu_cnt_lo_width_c-1 downto 0); else NULL; end if;
          when csr_instreth_c | csr_minstreth_c => -- [m]instreth (r/w): Instructions-retired counter HIGH
            if (cpu_cnt_hi_width_c > 0) and (CPU_EXTENSION_RISCV_Zicntr = true) then csr.rdata(cpu_cnt_hi_width_c-1 downto 0) <= csr.minstreth(cpu_cnt_hi_width_c-1 downto 0); else NULL; end if;

          when csr_time_c => -- time (r/-): System time LOW (from MTIME unit)
            if (CPU_EXTENSION_RISCV_Zicntr = true) then csr.rdata <= time_i(31 downto 00); else NULL; end if; 
          when csr_timeh_c => -- timeh (r/-): System time HIGH (from MTIME unit)
            if (CPU_EXTENSION_RISCV_Zicntr = true) then csr.rdata <= time_i(63 downto 32); else NULL; end if; 

          -- hardware performance counters --
          -- --------------------------------------------------------------------
          -- low word (r/w) --
          when csr_mhpmcounter3_c   => if (HPM_NUM_CNTS > 00) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(00); else NULL; end if;
          when csr_mhpmcounter4_c   => if (HPM_NUM_CNTS > 01) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(01); else NULL; end if;
          when csr_mhpmcounter5_c   => if (HPM_NUM_CNTS > 02) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(02); else NULL; end if;
          when csr_mhpmcounter6_c   => if (HPM_NUM_CNTS > 03) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(03); else NULL; end if;
          when csr_mhpmcounter7_c   => if (HPM_NUM_CNTS > 04) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(04); else NULL; end if;
          when csr_mhpmcounter8_c   => if (HPM_NUM_CNTS > 05) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(05); else NULL; end if;
          when csr_mhpmcounter9_c   => if (HPM_NUM_CNTS > 06) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(06); else NULL; end if;
          when csr_mhpmcounter10_c  => if (HPM_NUM_CNTS > 07) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(07); else NULL; end if;
          when csr_mhpmcounter11_c  => if (HPM_NUM_CNTS > 08) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(08); else NULL; end if;
          when csr_mhpmcounter12_c  => if (HPM_NUM_CNTS > 09) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(09); else NULL; end if;
          when csr_mhpmcounter13_c  => if (HPM_NUM_CNTS > 10) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(10); else NULL; end if;
          when csr_mhpmcounter14_c  => if (HPM_NUM_CNTS > 11) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(11); else NULL; end if;
          when csr_mhpmcounter15_c  => if (HPM_NUM_CNTS > 12) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(12); else NULL; end if;
          when csr_mhpmcounter16_c  => if (HPM_NUM_CNTS > 13) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(13); else NULL; end if;
          when csr_mhpmcounter17_c  => if (HPM_NUM_CNTS > 14) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(14); else NULL; end if;
          when csr_mhpmcounter18_c  => if (HPM_NUM_CNTS > 15) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(15); else NULL; end if;
          when csr_mhpmcounter19_c  => if (HPM_NUM_CNTS > 16) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(16); else NULL; end if;
          when csr_mhpmcounter20_c  => if (HPM_NUM_CNTS > 17) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(17); else NULL; end if;
          when csr_mhpmcounter21_c  => if (HPM_NUM_CNTS > 18) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(18); else NULL; end if;
          when csr_mhpmcounter22_c  => if (HPM_NUM_CNTS > 19) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(19); else NULL; end if;
          when csr_mhpmcounter23_c  => if (HPM_NUM_CNTS > 20) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(20); else NULL; end if;
          when csr_mhpmcounter24_c  => if (HPM_NUM_CNTS > 21) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(21); else NULL; end if;
          when csr_mhpmcounter25_c  => if (HPM_NUM_CNTS > 22) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(22); else NULL; end if;
          when csr_mhpmcounter26_c  => if (HPM_NUM_CNTS > 23) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(23); else NULL; end if;
          when csr_mhpmcounter27_c  => if (HPM_NUM_CNTS > 24) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(24); else NULL; end if;
          when csr_mhpmcounter28_c  => if (HPM_NUM_CNTS > 25) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(25); else NULL; end if;
          when csr_mhpmcounter29_c  => if (HPM_NUM_CNTS > 26) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(26); else NULL; end if;
          when csr_mhpmcounter30_c  => if (HPM_NUM_CNTS > 27) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(27); else NULL; end if;
          when csr_mhpmcounter31_c  => if (HPM_NUM_CNTS > 28) and (CPU_EXTENSION_RISCV_Zihpm = true) then csr.rdata <= csr.mhpmcounter_rd(28); else NULL; end if;
          -- high word (r/w) --
          when csr_mhpmcounter3h_c  => if (HPM_NUM_CNTS > 00) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(00); else NULL; end if;
          when csr_mhpmcounter4h_c  => if (HPM_NUM_CNTS > 01) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(01); else NULL; end if;
          when csr_mhpmcounter5h_c  => if (HPM_NUM_CNTS > 02) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(02); else NULL; end if;
          when csr_mhpmcounter6h_c  => if (HPM_NUM_CNTS > 03) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(03); else NULL; end if;
          when csr_mhpmcounter7h_c  => if (HPM_NUM_CNTS > 04) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(04); else NULL; end if;
          when csr_mhpmcounter8h_c  => if (HPM_NUM_CNTS > 05) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(05); else NULL; end if;
          when csr_mhpmcounter9h_c  => if (HPM_NUM_CNTS > 06) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(06); else NULL; end if;
          when csr_mhpmcounter10h_c => if (HPM_NUM_CNTS > 07) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(07); else NULL; end if;
          when csr_mhpmcounter11h_c => if (HPM_NUM_CNTS > 08) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(08); else NULL; end if;
          when csr_mhpmcounter12h_c => if (HPM_NUM_CNTS > 09) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(09); else NULL; end if;
          when csr_mhpmcounter13h_c => if (HPM_NUM_CNTS > 10) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(10); else NULL; end if;
          when csr_mhpmcounter14h_c => if (HPM_NUM_CNTS > 11) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(11); else NULL; end if;
          when csr_mhpmcounter15h_c => if (HPM_NUM_CNTS > 12) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(12); else NULL; end if;
          when csr_mhpmcounter16h_c => if (HPM_NUM_CNTS > 13) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(13); else NULL; end if;
          when csr_mhpmcounter17h_c => if (HPM_NUM_CNTS > 14) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(14); else NULL; end if;
          when csr_mhpmcounter18h_c => if (HPM_NUM_CNTS > 15) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(15); else NULL; end if;
          when csr_mhpmcounter19h_c => if (HPM_NUM_CNTS > 16) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(16); else NULL; end if;
          when csr_mhpmcounter20h_c => if (HPM_NUM_CNTS > 17) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(17); else NULL; end if;
          when csr_mhpmcounter21h_c => if (HPM_NUM_CNTS > 18) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(18); else NULL; end if;
          when csr_mhpmcounter22h_c => if (HPM_NUM_CNTS > 19) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(19); else NULL; end if;
          when csr_mhpmcounter23h_c => if (HPM_NUM_CNTS > 20) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(20); else NULL; end if;
          when csr_mhpmcounter24h_c => if (HPM_NUM_CNTS > 21) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(21); else NULL; end if;
          when csr_mhpmcounter25h_c => if (HPM_NUM_CNTS > 22) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(22); else NULL; end if;
          when csr_mhpmcounter26h_c => if (HPM_NUM_CNTS > 23) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(23); else NULL; end if;
          when csr_mhpmcounter27h_c => if (HPM_NUM_CNTS > 24) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(24); else NULL; end if;
          when csr_mhpmcounter28h_c => if (HPM_NUM_CNTS > 25) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(25); else NULL; end if;
          when csr_mhpmcounter29h_c => if (HPM_NUM_CNTS > 26) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(26); else NULL; end if;
          when csr_mhpmcounter30h_c => if (HPM_NUM_CNTS > 27) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(27); else NULL; end if;
          when csr_mhpmcounter31h_c => if (HPM_NUM_CNTS > 28) and (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_cnt_hi_width_c > 0) then csr.rdata <= csr.mhpmcounterh_rd(28); else NULL; end if;

          -- machine information registers --
          -- --------------------------------------------------------------------
--        when csr_mvendorid_c  => NULL; -- mvendorid (r/-): vendor ID, implemented but always zero
          when csr_marchid_c    => csr.rdata(4 downto 0) <= "10011"; -- marchid (r/-): arch ID - official RISC-V open-source arch ID
          when csr_mimpid_c     => csr.rdata <= hw_version_c; -- mimpid (r/-): implementation ID -- NEORV32 hardware version
          when csr_mhartid_c    => csr.rdata <= std_ulogic_vector(to_unsigned(HW_THREAD_ID, 32)); -- mhartid (r/-): hardware thread ID
--        when csr_mconfigptr_c => NULL; -- mconfigptr (r/-): machine configuration pointer register, implemented but always zero

          -- debug mode CSRs --
          -- --------------------------------------------------------------------
          when csr_dcsr_c      => if (CPU_EXTENSION_RISCV_DEBUG = true) then csr.rdata <= csr.dcsr_rd;   else NULL; end if; -- dcsr (r/w): debug mode control and status
          when csr_dpc_c       => if (CPU_EXTENSION_RISCV_DEBUG = true) then csr.rdata <= csr.dpc;       else NULL; end if; -- dpc (r/w): debug mode program counter
          when csr_dscratch0_c => if (CPU_EXTENSION_RISCV_DEBUG = true) then csr.rdata <= csr.dscratch0; else NULL; end if; -- dscratch0 (r/w): debug mode scratch register 0

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
            if (debug_ctrl.trig_halt = '1') or -- external request (from DM)
               (debug_ctrl.trig_break = '1') or -- ebreak instruction
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
  debug_ctrl.trig_break <= trap_ctrl.break_point and (debug_ctrl.running or -- we are in debug mode: re-enter debug mode
                           (csr.priv_m_mode and csr.dcsr_ebreakm and (not debug_ctrl.running)) or -- enabled goto-debug-mode in machine mode on "ebreak"
                           (csr.priv_u_mode and csr.dcsr_ebreaku and (not debug_ctrl.running))); -- enabled goto-debug-mode in user mode on "ebreak"
  debug_ctrl.trig_halt <= debug_ctrl.ext_halt_req and (not debug_ctrl.running); -- external halt request (if not halted already)
  debug_ctrl.trig_step <= csr.dcsr_step and (not debug_ctrl.running); -- single-step mode (trigger when NOT CURRENTLY in debug mode)


  -- Debug Control and Status Register (dcsr) - Read-Back -----------------------------------
  -- -------------------------------------------------------------------------------------------
  dcsr_readback_false:
  if (CPU_EXTENSION_RISCV_DEBUG = false) generate
    csr.dcsr_rd <= (others => '-');
  end generate;

  dcsr_readback_true:
  if (CPU_EXTENSION_RISCV_DEBUG = true) generate
    csr.dcsr_rd(31 downto 28) <= "0100"; -- xdebugver: external debug support compatible to spec
    csr.dcsr_rd(27 downto 16) <= (others => '0'); -- reserved
    csr.dcsr_rd(15) <= csr.dcsr_ebreakm; -- ebreakm: what happens on ebreak in m-mode? (normal trap OR debug-enter)
    csr.dcsr_rd(14) <= '0'; -- ebreakh: hypervisor mode not implemented
    csr.dcsr_rd(13) <= '0'; -- ebreaks: supervisor mode not implemented
    csr.dcsr_rd(12) <= csr.dcsr_ebreaku when (CPU_EXTENSION_RISCV_U = true) else '0'; -- ebreaku: what happens on ebreak in u-mode? (normal trap OR debug-enter)
    csr.dcsr_rd(11) <= '0'; -- stepie: interrupts are disabled during single-stepping
    csr.dcsr_rd(10) <= '0'; -- stopcount: counters increment as usual FIXME ???
    csr.dcsr_rd(09) <= '0'; -- stoptime: timers increment as usual
    csr.dcsr_rd(08 downto 06) <= csr.dcsr_cause; -- debug mode entry cause
    csr.dcsr_rd(05) <= '0'; -- reserved
    csr.dcsr_rd(04) <= '0'; -- mprven: mstatus.mprv is ignored in debug mode
    csr.dcsr_rd(03) <= '0'; -- nmip: pending non-maskable interrupt
    csr.dcsr_rd(02) <= csr.dcsr_step; -- step: single-step mode
    csr.dcsr_rd(01 downto 00) <= csr.dcsr_prv; -- prv: privilege mode when debug mode was entered
  end generate;


end neorv32_cpu_control_rtl;
