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
    HW_THREAD_ID                 : natural := 0;     -- hardware thread id (32-bit)
    CPU_BOOT_ADDR                : std_ulogic_vector(31 downto 0):= x"00000000"; -- cpu boot address
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        : boolean := false; -- implement atomic extension?
    CPU_EXTENSION_RISCV_B        : boolean := false; -- implement bit manipulation extensions?
    CPU_EXTENSION_RISCV_C        : boolean := false; -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        : boolean := false; -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_F        : boolean := false; -- implement 32-bit floating-point extension?
    CPU_EXTENSION_RISCV_M        : boolean := false; -- implement muld/div extension?
    CPU_EXTENSION_RISCV_U        : boolean := false; -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zicsr    : boolean := true;  -- implement CSR system?
    CPU_EXTENSION_RISCV_Zifencei : boolean := false; -- implement instruction stream sync.?
    -- Physical memory protection (PMP) --
    PMP_NUM_REGIONS              : natural := 0;       -- number of regions (0..64)
    PMP_MIN_GRANULARITY          : natural := 64*1024; -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 : natural := 0      -- number of implemented HPM counters (0..29)
  );
  port (
    -- global control --
    clk_i         : in  std_ulogic; -- global clock, rising edge
    rstn_i        : in  std_ulogic; -- global reset, low-active, async
    ctrl_o        : out std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
    -- status input --
    alu_wait_i    : in  std_ulogic; -- wait for ALU
    bus_i_wait_i  : in  std_ulogic; -- wait for bus
    bus_d_wait_i  : in  std_ulogic; -- wait for bus
    -- data input --
    instr_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- instruction
    cmp_i         : in  std_ulogic_vector(1 downto 0); -- comparator status
    alu_add_i     : in  std_ulogic_vector(data_width_c-1 downto 0); -- ALU address result
    rs1_i         : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 1
    -- data output --
    imm_o         : out std_ulogic_vector(data_width_c-1 downto 0); -- immediate
    fetch_pc_o    : out std_ulogic_vector(data_width_c-1 downto 0); -- PC for instruction fetch
    curr_pc_o     : out std_ulogic_vector(data_width_c-1 downto 0); -- current PC (corresponding to current instruction)
    csr_rdata_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- CSR read data
    -- FPU interface --
    fpu_rm_o      : out std_ulogic_vector(02 downto 0); -- rounding mode
    fpu_flags_i   : in  std_ulogic_vector(04 downto 0); -- exception flags
    -- interrupts (risc-v compliant) --
    msw_irq_i     : in  std_ulogic; -- machine software interrupt
    mext_irq_i    : in  std_ulogic; -- machine external interrupt
    mtime_irq_i   : in  std_ulogic; -- machine timer interrupt
    -- fast interrupts (custom) --
    firq_i        : in  std_ulogic_vector(15 downto 0);
    firq_ack_o    : out std_ulogic_vector(15 downto 0);
    -- system time input from MTIME --
    time_i        : in  std_ulogic_vector(63 downto 0); -- current system time
    -- physical memory protection --
    pmp_addr_o    : out pmp_addr_if_t; -- addresses
    pmp_ctrl_o    : out pmp_ctrl_if_t; -- configs
    -- bus access exceptions --
    mar_i         : in  std_ulogic_vector(data_width_c-1 downto 0);  -- memory address register
    ma_instr_i    : in  std_ulogic; -- misaligned instruction address
    ma_load_i     : in  std_ulogic; -- misaligned load data address
    ma_store_i    : in  std_ulogic; -- misaligned store data address
    be_instr_i    : in  std_ulogic; -- bus error on instruction access
    be_load_i     : in  std_ulogic; -- bus error on load data access
    be_store_i    : in  std_ulogic  -- bus error on store data access
  );
end neorv32_cpu_control;

architecture neorv32_cpu_control_rtl of neorv32_cpu_control is

  -- instruction fetch enginge --
  type fetch_engine_state_t is (IFETCH_RESET, IFETCH_REQUEST, IFETCH_ISSUE);
  type fetch_engine_t is record
    state       : fetch_engine_state_t;
    state_nxt   : fetch_engine_state_t;
    state_prev  : fetch_engine_state_t;
    pc          : std_ulogic_vector(data_width_c-1 downto 0);
    pc_nxt      : std_ulogic_vector(data_width_c-1 downto 0);
    reset       : std_ulogic;
    bus_err_ack : std_ulogic;
  end record;
  signal fetch_engine : fetch_engine_t;

  -- instrucion prefetch buffer (IPB, real FIFO) --
  type ipb_data_fifo_t is array (0 to ipb_entries_c-1) of std_ulogic_vector(2+31 downto 0);
  type ipb_t is record
    wdata : std_ulogic_vector(2+31 downto 0); -- write status (bus_error, align_error) + 32-bit instruction data
    we    : std_ulogic; -- trigger write
    free  : std_ulogic; -- free entry available?
    clear : std_ulogic; -- clear all entries
    --
    rdata : std_ulogic_vector(2+31 downto 0); -- read data: status (bus_error, align_error) + 32-bit instruction data
    re    : std_ulogic; -- read enable
    avail : std_ulogic; -- data available?
    --
    w_pnt : std_ulogic_vector(index_size_f(ipb_entries_c) downto 0); -- write pointer
    r_pnt : std_ulogic_vector(index_size_f(ipb_entries_c) downto 0); -- read pointer
    match : std_ulogic;
    empty : std_ulogic;
    full  : std_ulogic;
    --
    data  : ipb_data_fifo_t; -- fifo memory
  end record;
  signal ipb : ipb_t;

  -- pre-decoder --
  signal ci_instr16 : std_ulogic_vector(15 downto 0);
  signal ci_instr32 : std_ulogic_vector(31 downto 0);
  signal ci_illegal : std_ulogic;

  -- instruction issue enginge --
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
    alu_immediate   : std_ulogic;
    rs1_is_r0       : std_ulogic;
    is_atomic_lr    : std_ulogic;
    is_atomic_sc    : std_ulogic;
    is_bitmanip_imm : std_ulogic;
    is_bitmanip_reg : std_ulogic;
    is_float_f_reg  : std_ulogic;
    is_float_i_reg  : std_ulogic;
    sys_env_cmd     : std_ulogic_vector(11 downto 0);
  end record;
  signal decode_aux : decode_aux_t;

  -- instruction execution engine --
  type execute_engine_state_t is (SYS_WAIT, DISPATCH, TRAP_ENTER, TRAP_EXIT, TRAP_EXECUTE, EXECUTE, ALU_WAIT,
                                  BRANCH, FENCE_OP,LOADSTORE_0, LOADSTORE_1, LOADSTORE_2, SYS_ENV, CSR_ACCESS);
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
    is_cp_op     : std_ulogic; -- current instruction is a co-processor operation
    is_cp_op_nxt : std_ulogic;
    is_fp        : std_ulogic; -- floating-point operation - do not access to integer register file
    is_fp_nxt    : std_ulogic;
    --
    branch_taken : std_ulogic; -- branch condition fullfilled
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
    firq_sync     : std_ulogic_vector(15 downto 0);
    irq_fire      : std_ulogic; -- set if there is a valid source in the interrupt buffer
    exc_ack       : std_ulogic; -- acknowledge all exceptions
    irq_ack       : std_ulogic_vector(interrupt_width_c-1 downto 0); -- acknowledge specific interrupt
    irq_ack_nxt   : std_ulogic_vector(interrupt_width_c-1 downto 0);
    cause         : std_ulogic_vector(5 downto 0); -- trap ID for mcause CSR
    cause_nxt     : std_ulogic_vector(5 downto 0);
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

  -- atomic operations controller --
  type atomic_ctrl_t is record
    env_start  : std_ulogic; -- begin atomic operations
    env_end    : std_ulogic; -- end atomic operations
    env_end_ff : std_ulogic; -- end atomic operations dealyed
    env_abort  : std_ulogic; -- atomic operations abort (results in failure)
    lock       : std_ulogic; -- lock status
  end record;
  signal atomic_ctrl : atomic_ctrl_t;
  
  -- CPU main control bus --
  signal ctrl_nxt, ctrl : std_ulogic_vector(ctrl_width_c-1 downto 0);

  -- fast instruction fetch access --
  signal bus_fast_ir : std_ulogic;

  -- RISC-V control and status registers (CSRs) --
  type pmp_ctrl_t     is array (0 to PMP_NUM_REGIONS-1) of std_ulogic_vector(7 downto 0);
  type pmp_addr_t     is array (0 to PMP_NUM_REGIONS-1) of std_ulogic_vector(data_width_c-1 downto 0);
  type pmp_ctrl_rd_t  is array (0 to 63) of std_ulogic_vector(7 downto 0);
  type pmp_addr_rd_t  is array (0 to 63) of std_ulogic_vector(data_width_c-1 downto 0);
  type mhpmevent_t    is array (0 to HPM_NUM_CNTS-1) of std_ulogic_vector(hpmcnt_event_size_c-1 downto 0);
  type mhpmcnt_t      is array (0 to HPM_NUM_CNTS-1) of std_ulogic_vector(32 downto 0);
  type mhpmcnth_t     is array (0 to HPM_NUM_CNTS-1) of std_ulogic_vector(31 downto 0);
  type mhpmevent_rd_t is array (0 to 29) of std_ulogic_vector(hpmcnt_event_size_c-1 downto 0);
  type mhpmcnt_rd_t   is array (0 to 29) of std_ulogic_vector(32 downto 0);
  type mhpmcnth_rd_t  is array (0 to 29) of std_ulogic_vector(31 downto 0);
  type csr_t is record
    addr              : std_ulogic_vector(11 downto 0); -- csr address
    we                : std_ulogic; -- csr write enable
    we_nxt            : std_ulogic;
    re                : std_ulogic; -- csr read enable
    re_nxt            : std_ulogic;
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
    mcounteren_cy     : std_ulogic; -- mcounteren.cy: allow cycle[h] access from user-mode
    mcounteren_tm     : std_ulogic; -- mcounteren.tm: allow time[h] access from user-mode
    mcounteren_ir     : std_ulogic; -- mcounteren.ir: allow instret[h] access from user-mode
    mcounteren_hpm    : std_ulogic_vector(HPM_NUM_CNTS-1 downto 0); -- mcounteren.hpmx: allow mhpmcounterx[h] access from user-mode
    --
    mcountinhibit_cy  : std_ulogic; -- mcounterinhibit.cy: enable auto-increment for [m]cycle[h]
    mcountinhibit_ir  : std_ulogic; -- mcounterinhibit.ir: enable auto-increment for [m]instret[h]
    mcountinhibit_hpm : std_ulogic_vector(HPM_NUM_CNTS-1 downto 0); -- mcounterinhibit.hpm3: enable auto-increment for mhpmcounterx[h]
    --
    mip_status        : std_ulogic_vector(interrupt_width_c-1  downto 0); -- current buffered IRQs
    mip_clear         : std_ulogic_vector(interrupt_width_c-1  downto 0); -- set bits clear the according buffered IRQ
    --
    privilege         : std_ulogic_vector(1 downto 0); -- hart's current privilege mode
    priv_m_mode       : std_ulogic; -- CPU in M-mode
    priv_u_mode       : std_ulogic; -- CPU in u-mode
    --
    mepc              : std_ulogic_vector(data_width_c-1 downto 0); -- mepc: machine exception pc (R/W)
    mcause            : std_ulogic_vector(5 downto 0); -- mcause: machine trap cause (R/W)
    mtvec             : std_ulogic_vector(data_width_c-1 downto 0); -- mtvec: machine trap-handler base address (R/W), bit 1:0 == 00
    mtval             : std_ulogic_vector(data_width_c-1 downto 0); -- mtval: machine bad address or instruction (R/W)
    --
    mhpmevent         : mhpmevent_t; -- mhpmevent*: machine performance-monitoring event selector (R/W)
    mhpmevent_rd      : mhpmevent_rd_t; -- mhpmevent*: actual read data
    --
    mscratch          : std_ulogic_vector(data_width_c-1 downto 0); -- mscratch: scratch register (R/W)
    mcycle            : std_ulogic_vector(32 downto 0); -- mcycle (R/W), plus carry bit
    minstret          : std_ulogic_vector(32 downto 0); -- minstret (R/W), plus carry bit
    --
    mcycleh           : std_ulogic_vector(31 downto 0); -- mcycleh (R/W)
    minstreth         : std_ulogic_vector(31 downto 0); -- minstreth (R/W)
    --
    mhpmcounter       : mhpmcnt_t; -- mhpmcounter* (R/W), plus carry bit
    mhpmcounterh      : mhpmcnth_t; -- mhpmcounter*h (R/W)
    mhpmcounter_rd    : mhpmcnt_rd_t; -- mhpmcounter* (R/W): actual read data
    mhpmcounterh_rd   : mhpmcnth_rd_t; -- mhpmcounter*h (R/W): actual read data
    --
    pmpcfg            : pmp_ctrl_t; -- physical memory protection - configuration registers
    pmpcfg_rd         : pmp_ctrl_rd_t; -- physical memory protection - actual read data
    pmpaddr           : pmp_addr_t; -- physical memory protection - address registers
    pmpaddr_rd        : pmp_addr_rd_t; -- physical memory protection - actual read data
    --
    frm               : std_ulogic_vector(02 downto 0); -- frm (R/W): FPU rounding mode
    fflags            : std_ulogic_vector(04 downto 0); -- fflags (R/W): FPU exception flags
  end record;
  signal csr : csr_t;

  -- counter low-to-high-word carry --
  signal mcycle_msb      : std_ulogic;
  signal minstret_msb    : std_ulogic;
  signal mhpmcounter_msb : std_ulogic_vector(HPM_NUM_CNTS-1 downto 0);

  -- (hpm) counter events --
  signal cnt_event, cnt_event_nxt : std_ulogic_vector(hpmcnt_event_size_c-1 downto 0);
  signal hpmcnt_trigger           : std_ulogic_vector(HPM_NUM_CNTS-1 downto 0);

  -- illegal instruction check --
  signal illegal_opcode_lsbs : std_ulogic; -- if opcode != rv32
  signal illegal_instruction : std_ulogic;
  signal illegal_register    : std_ulogic; -- only for E-extension
  signal illegal_compressed  : std_ulogic; -- only fir C-extension

  -- access (privilege) check --
  signal csr_acc_valid : std_ulogic; -- valid CSR access (implemented and valid access rights)

begin

-- ****************************************************************************************************************************
-- Instruction Fetch (always fetches aligned 32-bit chunks of data)
-- ****************************************************************************************************************************

  -- Fetch Engine FSM Sync ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  fetch_engine_fsm_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      fetch_engine.state      <= IFETCH_RESET;
      fetch_engine.state_prev <= IFETCH_RESET;
      fetch_engine.pc         <= (others => '0');
    elsif rising_edge(clk_i) then
      if (fetch_engine.reset = '1') then
        fetch_engine.state <= IFETCH_RESET;
      else
        fetch_engine.state <= fetch_engine.state_nxt;
      end if;
      fetch_engine.state_prev <= fetch_engine.state;
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
    bus_fast_ir              <= '0';
    fetch_engine.state_nxt   <= fetch_engine.state;
    fetch_engine.pc_nxt      <= fetch_engine.pc;
    fetch_engine.bus_err_ack <= '0';

    -- instruction prefetch buffer interface --
    ipb.we    <= '0';
    ipb.wdata <= be_instr_i & ma_instr_i & instr_i(31 downto 0); -- store exception info and instruction word
    ipb.clear <= '0';

    -- state machine --
    case fetch_engine.state is

      when IFETCH_RESET => -- reset engine and prefetch buffer, get application PC
      -- ------------------------------------------------------------
        fetch_engine.bus_err_ack <= '1'; -- acknowledge any instruction bus errors, the execute engine has to take care of them / terminate current transfer
        fetch_engine.pc_nxt      <= execute_engine.pc(data_width_c-1 downto 1) & '0'; -- initialize with "real" application PC
        ipb.clear                <= '1'; -- clear prefetch buffer
        fetch_engine.state_nxt   <= IFETCH_REQUEST;

      when IFETCH_REQUEST => -- output current PC to bus system and request 32-bit (aligned!) instruction data
      -- ------------------------------------------------------------
        if (ipb.free = '1') then -- free entry in buffer?
          bus_fast_ir            <= '1'; -- fast instruction fetch request
          fetch_engine.state_nxt <= IFETCH_ISSUE;
        end if;

      when IFETCH_ISSUE => -- store instruction data to prefetch buffer
      -- ------------------------------------------------------------
        fetch_engine.bus_err_ack <= be_instr_i or ma_instr_i; -- ACK bus/alignment errors
        if (bus_i_wait_i = '0') or (be_instr_i = '1') or (ma_instr_i = '1') then -- wait for bus response
          fetch_engine.pc_nxt    <= std_ulogic_vector(unsigned(fetch_engine.pc) + 4);
          ipb.we                 <= '1';
          fetch_engine.state_nxt <= IFETCH_REQUEST;
        end if;

      when others => -- undefined
      -- ------------------------------------------------------------
        fetch_engine.state_nxt <= IFETCH_RESET;

    end case;
  end process fetch_engine_fsm_comb;


-- ****************************************************************************************************************************
-- Instruction Prefetch Buffer
-- ****************************************************************************************************************************


  -- Instruction Prefetch Buffer (FIFO) -----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  instr_prefetch_buffer: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- write port --
      if (ipb.clear = '1') then
        ipb.w_pnt <= (others => '0');
      elsif (ipb.we = '1') then
        ipb.w_pnt <= std_ulogic_vector(unsigned(ipb.w_pnt) + 1);
      end if;
      if (ipb.we = '1') then -- write data
        ipb.data(to_integer(unsigned(ipb.w_pnt(ipb.w_pnt'left-1 downto 0)))) <= ipb.wdata;
      end if;
      -- read port --
      if (ipb.clear = '1') then
        ipb.r_pnt <= (others => '0');
      elsif (ipb.re = '1') then
        ipb.r_pnt <= std_ulogic_vector(unsigned(ipb.r_pnt) + 1);
      end if;
    end if;
  end process instr_prefetch_buffer;

  -- async read --
  ipb.rdata <= ipb.data(to_integer(unsigned(ipb.r_pnt(ipb.r_pnt'left-1 downto 0))));

  -- status --
  ipb.match <= '1' when (ipb.r_pnt(ipb.r_pnt'left-1 downto 0) = ipb.w_pnt(ipb.w_pnt'left-1 downto 0))  else '0';
  ipb.full  <= '1' when (ipb.r_pnt(ipb.r_pnt'left) /= ipb.w_pnt(ipb.w_pnt'left)) and (ipb.match = '1') else '0';
  ipb.empty <= '1' when (ipb.r_pnt(ipb.r_pnt'left)  = ipb.w_pnt(ipb.w_pnt'left)) and (ipb.match = '1') else '0';
  ipb.free  <= not ipb.full;
  ipb.avail <= not ipb.empty;


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
        if (CPU_EXTENSION_RISCV_C = true) then
          if (execute_engine.pc(1) = '1') then -- branch to unaligned address?
            issue_engine.state <= ISSUE_REALIGN;
            issue_engine.align <= '1'; -- aligned on 16-bit boundary
          else
            issue_engine.state <= issue_engine.state_nxt;
            issue_engine.align <= '0'; -- aligned on 32-bit boundary
          end if;
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
                cmd_issue.data <= '0' & issue_engine.buf(17 downto 16) & '0' & (ipb.rdata(15 downto 0) & issue_engine.buf(15 downto 0));
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
  imm_gen: process(execute_engine.i_reg, clk_i)
    variable opcode_v : std_ulogic_vector(6 downto 0);
  begin
    opcode_v := execute_engine.i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c+2) & "11";
    if rising_edge(clk_i) then
      if (execute_engine.state = BRANCH) then -- next_PC as immediate for jump-and-link operations (=return address) via ALU.MOV_B
        imm_o <= execute_engine.next_pc;
      else -- "normal" immediate from instruction word
        case opcode_v is -- save some bits here, the two LSBs are always "11" for rv32
          when opcode_store_c | opcode_fsw_c => -- S-immediate
            imm_o(31 downto 11) <= (others => execute_engine.i_reg(31)); -- sign extension
            imm_o(10 downto 05) <= execute_engine.i_reg(30 downto 25);
            imm_o(04 downto 01) <= execute_engine.i_reg(11 downto 08);
            imm_o(00)           <= execute_engine.i_reg(07);
          when opcode_branch_c => -- B-immediate
            imm_o(31 downto 12) <= (others => execute_engine.i_reg(31)); -- sign extension
            imm_o(11)           <= execute_engine.i_reg(07);
            imm_o(10 downto 05) <= execute_engine.i_reg(30 downto 25);
            imm_o(04 downto 01) <= execute_engine.i_reg(11 downto 08);
            imm_o(00)           <= '0';
          when opcode_lui_c | opcode_auipc_c => -- U-immediate
            imm_o(31 downto 20) <= execute_engine.i_reg(31 downto 20);
            imm_o(19 downto 12) <= execute_engine.i_reg(19 downto 12);
            imm_o(11 downto 00) <= (others => '0');
          when opcode_jal_c => -- J-immediate
            imm_o(31 downto 20) <= (others => execute_engine.i_reg(31)); -- sign extension
            imm_o(19 downto 12) <= execute_engine.i_reg(19 downto 12);
            imm_o(11)           <= execute_engine.i_reg(20);
            imm_o(10 downto 05) <= execute_engine.i_reg(30 downto 25);
            imm_o(04 downto 01) <= execute_engine.i_reg(24 downto 21);
            imm_o(00)           <= '0';
          when opcode_atomic_c => -- atomic memory access
            imm_o               <= (others => '0'); -- effective address is addr = reg + 0 = reg
          when others => -- I-immediate
            imm_o(31 downto 11) <= (others => execute_engine.i_reg(31)); -- sign extension
            imm_o(10 downto 05) <= execute_engine.i_reg(30 downto 25);
            imm_o(04 downto 01) <= execute_engine.i_reg(24 downto 21);
            imm_o(00)           <= execute_engine.i_reg(20);
        end case;
      end if;
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
      when others => -- undefined
        execute_engine.branch_taken <= '0';
    end case;
  end process branch_check;


  -- Execute Engine FSM Sync ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- for registers that DO require a specific reset state --
  execute_engine_fsm_sync_rst: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      execute_engine.pc       <= CPU_BOOT_ADDR(data_width_c-1 downto 1) & '0';
      execute_engine.state    <= SYS_WAIT;
      execute_engine.sleep    <= '0';
      execute_engine.branched <= '1'; -- reset is a branch from "somewhere"
    elsif rising_edge(clk_i) then
      -- PC update --
      if (execute_engine.pc_we = '1') then
        if (execute_engine.pc_mux_sel = '0') then
          execute_engine.pc <= execute_engine.next_pc(data_width_c-1 downto 1) & '0'; -- normal (linear) increment
        else
          execute_engine.pc <= alu_add_i(data_width_c-1 downto 1) & '0'; -- jump/taken_branch
        end if;
      end if;
      --
      execute_engine.state    <= execute_engine.state_nxt;
      execute_engine.sleep    <= execute_engine.sleep_nxt;
      execute_engine.branched <= execute_engine.branched_nxt;
    end if;
  end process execute_engine_fsm_sync_rst;


  -- for registers that do NOT require a specific reset state --
  execute_engine_fsm_sync: process(clk_i)
  begin
    if rising_edge(clk_i) then
      execute_engine.state_prev <= execute_engine.state;
      execute_engine.i_reg      <= execute_engine.i_reg_nxt;
      execute_engine.is_ci      <= execute_engine.is_ci_nxt;
      execute_engine.is_cp_op   <= execute_engine.is_cp_op_nxt;
      execute_engine.is_fp      <= execute_engine.is_fp_nxt;
      -- PC & IR of "last executed" instruction --
      if (execute_engine.state = EXECUTE) then
        execute_engine.last_pc    <= execute_engine.pc;
        execute_engine.i_reg_last <= execute_engine.i_reg;
      end if;
      -- next PC --
      case execute_engine.state is
        when TRAP_ENTER => execute_engine.next_pc <= csr.mtvec(data_width_c-1 downto 1) & '0'; -- trap enter
        when TRAP_EXIT  => execute_engine.next_pc <= csr.mepc(data_width_c-1 downto 1) & '0'; -- trap exit
        when EXECUTE    => execute_engine.next_pc <= std_ulogic_vector(unsigned(execute_engine.pc) + unsigned(execute_engine.next_pc_inc)); -- next linear PC
        when others     => NULL;
      end case;
      -- main control bus --
      ctrl <= ctrl_nxt;
    end if;
  end process execute_engine_fsm_sync;

  -- PC increment for next linear instruction (+2 for compressed instr., +4 otherwise) --
  execute_engine.next_pc_inc <= x"00000004" when ((execute_engine.is_ci = '0') or (CPU_EXTENSION_RISCV_C = false)) else x"00000002";

  -- PC output --
  curr_pc_o <= execute_engine.pc(data_width_c-1 downto 1) & '0'; -- PC for ALU ops

  -- CSR access address --
  csr.addr <= execute_engine.i_reg(instr_csr_id_msb_c downto instr_csr_id_lsb_c);


  -- CPU Control Bus Output -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_output: process(ctrl, fetch_engine, trap_ctrl, atomic_ctrl, bus_fast_ir, execute_engine, csr)
  begin
    -- signals from execute engine --
    ctrl_o <= ctrl;
    -- current privilege level --
    ctrl_o(ctrl_priv_lvl_msb_c downto ctrl_priv_lvl_lsb_c) <= csr.privilege;
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
    -- locked bus operation (for atomic memory operations) --
    ctrl_o(ctrl_bus_lock_c) <= atomic_ctrl.lock; -- (bus) lock status
    -- cpu status --
    ctrl_o(ctrl_sleep_c) <= execute_engine.sleep; -- cpu is in sleep mode
  end process ctrl_output;


  -- Decoding Helper Logic ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  decode_helper: process(execute_engine)
    variable sys_env_cmd_mask_v : std_ulogic_vector(11 downto 0);
  begin
    -- defaults --
    decode_aux.alu_immediate   <= '0';
    decode_aux.rs1_is_r0       <= '0';
    decode_aux.is_atomic_lr    <= '0';
    decode_aux.is_atomic_sc    <= '0';
    decode_aux.is_bitmanip_imm <= '0';
    decode_aux.is_bitmanip_reg <= '0';
    decode_aux.is_float_f_reg  <= '0';
    decode_aux.is_float_i_reg  <= '0';

    -- is immediate ALU operation? --
    decode_aux.alu_immediate <= not execute_engine.i_reg(instr_opcode_msb_c-1);

    -- is rs1 == r0? --
    decode_aux.rs1_is_r0 <= not or_all_f(execute_engine.i_reg(instr_rs1_msb_c downto instr_rs1_lsb_c));

    -- is atomic load-reservate/store-conditional? --
    if (CPU_EXTENSION_RISCV_A = true) and (execute_engine.i_reg(instr_opcode_lsb_c+3 downto instr_opcode_lsb_c+2) = "11") then -- valid atomic sub-opcode
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
          (execute_engine.i_reg(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c) = "00010") or -- PCNT
          (execute_engine.i_reg(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c) = "00100") or -- SEXT.B
          (execute_engine.i_reg(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c) = "00101")    -- SEXT.H
         )
       ) or
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "01001") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001")) or -- SBCLRI
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "00101") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001")) or -- SBSETI
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "01101") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001")) or -- SBINVI
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "01001") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "101")) or -- SBEXTI
       --
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "01100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "101")) or -- RORI
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "00101") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "101") and (execute_engine.i_reg(instr_imm12_lsb_c+6 downto instr_imm12_lsb_c) = "0000111")) or -- GORCI.b 7 (orc.b)
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "01101") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "101") and (execute_engine.i_reg(instr_imm12_lsb_c+6 downto instr_imm12_lsb_c) = "0011000")) then -- GREVI.-8 (rev8)
      decode_aux.is_bitmanip_imm <= '1';
    end if;
    -- register operation --
    if ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0110000") and (execute_engine.i_reg(instr_funct3_msb_c-1 downto instr_funct3_lsb_c) = "01")) or -- ROR / ROL
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000101") and (execute_engine.i_reg(instr_funct3_msb_c) = '1')) or -- MIN[U] / MAX[U]
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "100")) or -- PACK
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
        ) or
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0100100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001")) or -- SBCLR
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0010100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001")) or -- SBSET
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0110100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001")) or -- SBINV
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0100100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "101")) then -- SBSEXT
      decode_aux.is_bitmanip_reg <= '1';
    end if;

    -- floating-point FLOAT_register operations --
    if ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "11110")) or -- FMV.W.X
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "00000")) or -- FADD.S
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "00001")) or -- FSUB.S
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "00010")) or -- FMUL.S
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "00011")) or -- FDIV.S
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "01011") and (execute_engine.i_reg(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c) = "00000")) or -- FSQRT.S
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "00100") and (execute_engine.i_reg(instr_funct3_msb_c) = '0')) or -- FSGNJ[N/X].S
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "00101") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_msb_c-1) = "00")) or -- FMIN.S / FMAX.S
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "11010") and (execute_engine.i_reg(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c+1) = "0000")) then -- FCVT.S.W*
      decode_aux.is_float_f_reg <= '1';
    end if;
    -- floating-point INTEGER_register operations --
    if ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "11100") and (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_msb_c-1) = "00")) or -- FMV.X.W / FCLASS.S
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "10100") and (execute_engine.i_reg(instr_funct3_msb_c) = '0')) or -- FEQ.S / FLT.S / FLE.S
       ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "11000") and (execute_engine.i_reg(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c+1) = "0000")) then -- FCVT.W*.S
      decode_aux.is_float_i_reg <= '1';
    end if;

    -- system/environment instructions --
    sys_env_cmd_mask_v := funct12_ecall_c or funct12_ebreak_c or funct12_mret_c or funct12_wfi_c; -- sum-up set bits
    decode_aux.sys_env_cmd(11 downto 0) <= execute_engine.i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) and sys_env_cmd_mask_v; -- set unsued bits to always-zero
  end process decode_helper;


  -- Execute Engine FSM Comb ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  execute_engine_fsm_comb: process(execute_engine, decode_aux, fetch_engine, cmd_issue, trap_ctrl, csr, ctrl, csr_acc_valid,
                                   alu_wait_i, bus_d_wait_i, ma_load_i, be_load_i, ma_store_i, be_store_i)
    variable opcode_v : std_ulogic_vector(6 downto 0);
  begin
    -- arbiter defaults --
    execute_engine.state_nxt    <= execute_engine.state;
    execute_engine.i_reg_nxt    <= execute_engine.i_reg;
    execute_engine.is_cp_op_nxt <= execute_engine.is_cp_op;
    execute_engine.is_ci_nxt    <= execute_engine.is_ci;
    execute_engine.is_fp_nxt    <= execute_engine.is_fp;
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

    -- exception trigger --
    trap_ctrl.instr_be          <= '0';
    trap_ctrl.instr_ma          <= '0';
    trap_ctrl.env_call          <= '0';
    trap_ctrl.break_point       <= '0';
    illegal_compressed          <= '0';

    -- CSR access --
    csr.we_nxt                  <= '0';
    csr.re_nxt                  <= '0';

    -- atomic operations control --
    atomic_ctrl.env_start       <= '0';
    atomic_ctrl.env_end         <= '0';
    atomic_ctrl.env_abort       <= '0';

    -- CONTROL DEFAULTS --
    ctrl_nxt <= (others => '0'); -- default: all off
    -- ALU main control --
    ctrl_nxt(ctrl_alu_addsub_c) <= '0'; -- ADD(I)
    ctrl_nxt(ctrl_alu_func1_c  downto ctrl_alu_func0_c) <= alu_func_cmd_arith_c; -- default ALU function select: arithmetic
    ctrl_nxt(ctrl_alu_arith_c) <= alu_arith_cmd_addsub_c; -- default ALU arithmetic operation: ADDSUB
    -- ALU sign control --
    if (execute_engine.i_reg(instr_opcode_lsb_c+4) = '1') then -- ALU ops
      ctrl_nxt(ctrl_alu_unsigned_c) <= execute_engine.i_reg(instr_funct3_lsb_c+0); -- unsigned ALU operation? (SLTIU, SLTU)
    else -- branches
      ctrl_nxt(ctrl_alu_unsigned_c) <= execute_engine.i_reg(instr_funct3_lsb_c+1); -- unsigned branches? (BLTU, BGEU)
    end if;


    -- state machine --
    case execute_engine.state is

      when SYS_WAIT => -- System delay cycle (used to wait for side effects to kick in) [and to init r0 with zero if it is a physical register]
      -- ------------------------------------------------------------
        -- set reg_file's r0 to zero --
        if (rf_r0_is_reg_c = true) then -- is r0 implemented as physical register, which has to be set to zero?
          ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_cmd_copro_c; -- hacky! CSR read-access CP selected without a valid CSR-read -> results zero
          ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_csr_rd_c; -- use CSR-READ CP
          ctrl_nxt(ctrl_rf_r0_we_c)                          <= '1'; -- force RF write access and force rd=r0
        end if;
        --
        execute_engine.state_nxt <= DISPATCH;


      when DISPATCH => -- Get new command from instruction issue engine
      -- ------------------------------------------------------------
        -- housekeeping --
        execute_engine.is_cp_op_nxt <= '0'; -- init
        execute_engine.is_fp_nxt    <= '0'; -- init
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
          trap_ctrl.instr_ma <= cmd_issue.data(33); -- misaligned instruction fetch address
          trap_ctrl.instr_be <= cmd_issue.data(34); -- bus access fault during instruction fetch
          illegal_compressed <= cmd_issue.data(35); -- invalid decompressed instruction
          -- any reason to go to trap state? --
          if (execute_engine.sleep = '1') or (trap_ctrl.env_start = '1') or (trap_ctrl.exc_fire = '1') or ((cmd_issue.data(33) or cmd_issue.data(34)) = '1') then
            execute_engine.state_nxt <= TRAP_ENTER;
          else
            execute_engine.state_nxt <= EXECUTE;
          end if;
        end if;


      when TRAP_ENTER => -- Start trap environment - get MTVEC, stay here for sleep mode
      -- ------------------------------------------------------------
        if (trap_ctrl.env_start = '1') then -- trap triggered?
          trap_ctrl.env_start_ack   <= '1';
          execute_engine.state_nxt  <= TRAP_EXECUTE;
        end if;

      when TRAP_EXIT => -- Return from trap environment - get MEPC
      -- ------------------------------------------------------------
        trap_ctrl.env_end        <= '1';
        execute_engine.state_nxt <= TRAP_EXECUTE;

      when TRAP_EXECUTE => -- Start trap environment - jump to MTVEC / return from trap environment - jump to MEPC
      -- ------------------------------------------------------------
        execute_engine.pc_mux_sel <= '0'; -- next PC (csr.mtvec)
        fetch_engine.reset        <= '1';
        execute_engine.pc_we      <= '1';
        execute_engine.sleep_nxt  <= '0'; -- disable sleep mode
        execute_engine.state_nxt  <= SYS_WAIT;


      when EXECUTE => -- Decode and execute instruction (control has to be here for excatly 1 cyle in any case!)
      -- ------------------------------------------------------------
        opcode_v := execute_engine.i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c+2) & "11"; -- save some bits here, LSBs are always 11 for rv32
        case opcode_v is

          when opcode_alu_c | opcode_alui_c => -- (immediate) ALU operation
          -- ------------------------------------------------------------
            ctrl_nxt(ctrl_alu_opa_mux_c) <= '0'; -- use RS1 as ALU.OPA
            ctrl_nxt(ctrl_alu_opb_mux_c) <= decode_aux.alu_immediate; -- use IMM as ALU.OPB for immediate operations
            ctrl_nxt(ctrl_rf_in_mux_c)   <= '0'; -- RF input = ALU result

            -- ALU arithmetic operation type and ADD/SUB --
            if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_slt_c) or
               (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sltu_c) then
              ctrl_nxt(ctrl_alu_arith_c) <= alu_arith_cmd_slt_c;
            else
              ctrl_nxt(ctrl_alu_arith_c) <= alu_arith_cmd_addsub_c;
            end if;

            -- ADD/SUB --
            if ((decode_aux.alu_immediate = '0') and (execute_engine.i_reg(instr_funct7_msb_c-1) = '1')) or -- not an immediate op and funct7.6 set => SUB
               (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_slt_c) or -- SLT operation
               (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sltu_c) then -- SLTU operation
              ctrl_nxt(ctrl_alu_addsub_c) <= '1'; -- SUB/SLT
            else
              ctrl_nxt(ctrl_alu_addsub_c) <= '0'; -- ADD(I)
            end if;

            -- ALU logic operation --
            case execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) is -- actual ALU.logic operation (re-coding)
              when funct3_xor_c => ctrl_nxt(ctrl_alu_logic1_c downto ctrl_alu_logic0_c) <= alu_logic_cmd_xor_c; -- XOR(I)
              when funct3_or_c  => ctrl_nxt(ctrl_alu_logic1_c downto ctrl_alu_logic0_c) <= alu_logic_cmd_or_c;  -- OR(I)
              when others       => ctrl_nxt(ctrl_alu_logic1_c downto ctrl_alu_logic0_c) <= alu_logic_cmd_and_c; -- AND(I)
            end case;

            -- co-processor MULDIV operation? --
            if (CPU_EXTENSION_RISCV_M = true) and (execute_engine.i_reg(instr_opcode_lsb_c+5) = opcode_alu_c(5)) and (execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000001") then -- MULDIV CP op?
              ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_muldiv_c; -- use MULDIV CP
              execute_engine.is_cp_op_nxt                        <= '1'; -- this is a CP operation
              ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_cmd_copro_c;
            -- co-processor bit manipulation operation? --
            elsif (CPU_EXTENSION_RISCV_B = true) and
              (((execute_engine.i_reg(instr_opcode_lsb_c+5) = opcode_alu_c(5))  and (decode_aux.is_bitmanip_reg = '1')) or -- register operation
               ((execute_engine.i_reg(instr_opcode_lsb_c+5) = opcode_alui_c(5)) and (decode_aux.is_bitmanip_imm = '1'))) then -- immediate operation
              ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_bitmanip_c; -- use BITMANIP CP
              execute_engine.is_cp_op_nxt                        <= '1'; -- this is a CP operation
              ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_cmd_copro_c;
            -- ALU operation, function select --
            else
              execute_engine.is_cp_op_nxt <= '0'; -- no CP operation
              case execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) is -- actual ALU.func operation (re-coding)
                when funct3_xor_c | funct3_or_c | funct3_and_c => ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_cmd_logic_c;
                when funct3_sll_c | funct3_sr_c                => ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_cmd_shift_c;
                when others                                    => ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_cmd_arith_c;
              end case;
            end if;

            -- multi cycle alu operation? --
            if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sll_c) or -- SLL shift operation?
               (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sr_c) or -- SR shift operation?
               ((CPU_EXTENSION_RISCV_M = true) and (execute_engine.i_reg(instr_opcode_lsb_c+5) = opcode_alu_c(5)) and (execute_engine.i_reg(instr_funct7_lsb_c) = '1')) or -- MULDIV CP op?
               ((CPU_EXTENSION_RISCV_B = true) and (
                 ((execute_engine.i_reg(instr_opcode_lsb_c+5) = opcode_alu_c(5))  and (decode_aux.is_bitmanip_reg = '1')) or -- BITMANIP CP register operation?
                 ((execute_engine.i_reg(instr_opcode_lsb_c+5) = opcode_alui_c(5)) and (decode_aux.is_bitmanip_imm = '1'))) ) then -- BITMANIP CP immediate operation?
              execute_engine.state_nxt <= ALU_WAIT;
            else -- single cycle ALU operation
              ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back
              execute_engine.state_nxt <= DISPATCH;
            end if;

          when opcode_lui_c | opcode_auipc_c => -- load upper immediate / add upper immediate to PC
          -- ------------------------------------------------------------
            ctrl_nxt(ctrl_alu_opa_mux_c) <= '1'; -- ALU.OPA = PC (for AUIPC only)
            ctrl_nxt(ctrl_alu_opb_mux_c) <= '1'; -- use IMM as ALU.OPB
            ctrl_nxt(ctrl_alu_arith_c)   <= alu_arith_cmd_addsub_c; -- actual ALU operation = ADD
            ctrl_nxt(ctrl_alu_logic1_c downto ctrl_alu_logic0_c) <= alu_logic_cmd_movb_c; -- MOVB
            if (execute_engine.i_reg(instr_opcode_lsb_c+5) = opcode_lui_c(5)) then -- LUI
              ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_cmd_logic_c; -- actual ALU operation = MOVB
            else -- AUIPC
              ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_cmd_arith_c; -- actual ALU operation = ADD
            end if;
            ctrl_nxt(ctrl_rf_in_mux_c) <= '0'; -- RF input = ALU result
            ctrl_nxt(ctrl_rf_wb_en_c)  <= '1'; -- valid RF write-back
            execute_engine.state_nxt   <= DISPATCH;

          when opcode_load_c | opcode_store_c | opcode_atomic_c | opcode_flw_c | opcode_fsw_c => -- load/store / atomic memory access / floating-point load/store 
          -- ------------------------------------------------------------
            ctrl_nxt(ctrl_alu_opa_mux_c) <= '0'; -- use RS1 as ALU.OPA
            ctrl_nxt(ctrl_alu_opb_mux_c) <= '1'; -- use IMM as ALU.OPB
            ctrl_nxt(ctrl_bus_mo_we_c)   <= '1'; -- write to MAR and MDO (MDO only relevant for store)
            if (CPU_EXTENSION_RISCV_F = true) and (execute_engine.i_reg(instr_opcode_lsb_c+3 downto instr_opcode_lsb_c+2) = "01") then -- floating-point load/store
              execute_engine.is_fp_nxt    <= decode_aux.is_float_f_reg; -- no integer register file write back for FPU internal operations
              ctrl_nxt(ctrl_bus_wd_sel_c) <= '1'; -- use memory-write-data from FPU co-processor (only relevant for float STORE)
            end if;
            --
            if (CPU_EXTENSION_RISCV_A = false) or -- atomic extension disabled
               (execute_engine.i_reg(instr_opcode_lsb_c+3 downto instr_opcode_lsb_c+2) = "00") or  -- normal integerload/store
               ((CPU_EXTENSION_RISCV_F = true) and (execute_engine.i_reg(instr_opcode_lsb_c+3 downto instr_opcode_lsb_c+2) = "01")) then -- floating-point load/store
              execute_engine.state_nxt <= LOADSTORE_0;
            else -- atomic operation
              atomic_ctrl.env_start <= not execute_engine.i_reg(instr_funct5_lsb_c); -- LR: start LOCKED memory access environment
              if (execute_engine.i_reg(instr_funct5_msb_c downto instr_funct5_lsb_c) = funct5_a_sc_c) or -- store-conditional
                 (execute_engine.i_reg(instr_funct5_msb_c downto instr_funct5_lsb_c) = funct5_a_lr_c) then -- load-reservate
                execute_engine.state_nxt <= LOADSTORE_0;
              else -- unimplemented (atomic) instruction
                execute_engine.state_nxt <= SYS_WAIT;
              end if;
            end if;

          when opcode_branch_c | opcode_jal_c | opcode_jalr_c => -- branch / jump and link (with register)
          -- ------------------------------------------------------------
            -- target address (ALU.ADD) operands --
            if (execute_engine.i_reg(instr_opcode_lsb_c+3 downto instr_opcode_lsb_c+2) = opcode_jalr_c(3 downto 2)) then -- JALR
              ctrl_nxt(ctrl_alu_opa_mux_c) <= '0'; -- use RS1 as ALU.OPA (branch target address base)
            else -- JAL
              ctrl_nxt(ctrl_alu_opa_mux_c) <= '1'; -- use PC as ALU.OPA (branch target address base)
            end if;
            ctrl_nxt(ctrl_alu_opb_mux_c) <= '1'; -- use IMM as ALU.OPB (branch target address offset)
            execute_engine.state_nxt     <= BRANCH;

          when opcode_fence_c => -- fence operations
          -- ------------------------------------------------------------
            execute_engine.state_nxt <= FENCE_OP;

          when opcode_syscsr_c => -- system/csr access
          -- ------------------------------------------------------------
            if (CPU_EXTENSION_RISCV_Zicsr = true) then
              csr.re_nxt <= csr_acc_valid; -- always read CSR if valid access, only relevant for CSR-instructions
              ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_cmd_copro_c; -- only relevant for CSR-instructions
              ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_csr_rd_c; -- use CSR-READ CP, only relevant for CSR-instructions
              if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_env_c) then -- system/environment
                execute_engine.state_nxt <= SYS_ENV;
              else -- CSR access
                execute_engine.state_nxt <= CSR_ACCESS;
              end if;
            else
              execute_engine.state_nxt <= SYS_WAIT;
            end if;

          when opcode_fop_c => -- floating-point operations (1 or 2 operands)
          -- ------------------------------------------------------------
            execute_engine.state_nxt <= SYS_WAIT;
            if (CPU_EXTENSION_RISCV_F = true) then
              execute_engine.is_fp_nxt                           <= decode_aux.is_float_f_reg; -- no integer register file write back for FPU internal operations
              ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_fpu_c; -- use FPU CP
              execute_engine.is_cp_op_nxt                        <= '1'; -- this is a CP operation
              ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_cmd_copro_c;
              execute_engine.state_nxt                           <= ALU_WAIT;
            end if;

          when others => -- undefined
          -- ------------------------------------------------------------
            execute_engine.state_nxt <= SYS_WAIT;

        end case;


      when SYS_ENV => -- system environment operation - execution
      -- ------------------------------------------------------------
        execute_engine.state_nxt <= SYS_WAIT;
        case decode_aux.sys_env_cmd is -- use a simplified input here (with permanent zeros)
          when funct12_ecall_c  => trap_ctrl.env_call       <= '1'; -- ECALL
          when funct12_ebreak_c => trap_ctrl.break_point    <= '1'; -- EBREAK
          when funct12_mret_c   => execute_engine.state_nxt <= TRAP_EXIT; -- MRET
          when funct12_wfi_c    => execute_engine.sleep_nxt <= '1'; -- WFI
          when others           => NULL;-- undefined
        end case;


      when CSR_ACCESS => -- read & write status and control register (CSR)
      -- ------------------------------------------------------------
        -- CSR write access --
        case execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          when funct3_csrrw_c | funct3_csrrwi_c => -- CSRRW(I)
            csr.we_nxt <= csr_acc_valid; -- always write CSR if valid access
          when funct3_csrrs_c | funct3_csrrsi_c | funct3_csrrc_c | funct3_csrrci_c => -- CSRRS(I) / CSRRC(I)
            csr.we_nxt <= (not decode_aux.rs1_is_r0) and csr_acc_valid; -- write CSR if rs1/imm is not zero and if valid access
          when others => -- invalid
            csr.we_nxt <= '0';
        end case;
        -- register file write back --
        ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_cmd_copro_c;
        ctrl_nxt(ctrl_rf_in_mux_c)                         <= '0'; -- RF input = ALU result
        ctrl_nxt(ctrl_rf_wb_en_c)                          <= '1'; -- valid RF write-back
        execute_engine.state_nxt                           <= DISPATCH;


      when ALU_WAIT => -- wait for multi-cycle ALU operation (shifter or CP) to finish
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_rf_in_mux_c) <= '0'; -- RF input = ALU result
        if (CPU_EXTENSION_RISCV_F = false) then
          ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back (permanent write-back)
        else
          ctrl_nxt(ctrl_rf_wb_en_c) <= not execute_engine.is_fp; -- allow write back if NOT <FPU-internal operation>
        end if;
        -- cp access or alu.shift? --
        if (execute_engine.is_cp_op = '1') then
          ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_cmd_copro_c;
        else
          ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_cmd_shift_c;
        end if;
        -- wait for result --
        if (alu_wait_i = '0') then
          execute_engine.state_nxt <= DISPATCH;
        end if;


      when BRANCH => -- update PC for taken branches and jumps
      -- ------------------------------------------------------------
        -- get and store return address (only relevant for jump-and-link operations) --
        ctrl_nxt(ctrl_alu_opb_mux_c)                         <= '1'; -- use IMM as ALU.OPB (next_pc from immediate generator = return address)
        ctrl_nxt(ctrl_alu_logic1_c downto ctrl_alu_logic0_c) <= alu_logic_cmd_movb_c; -- MOVB
        ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c)   <= alu_func_cmd_logic_c; -- actual ALU operation = MOVB
        ctrl_nxt(ctrl_rf_in_mux_c)                           <= '0'; -- RF input = ALU result
        ctrl_nxt(ctrl_rf_wb_en_c)                            <= execute_engine.i_reg(instr_opcode_lsb_c+2); -- valid RF write-back? (is jump-and-link?)
        -- destination address --
        execute_engine.pc_mux_sel <= '1'; -- alu.add = branch/jump destination
        if (execute_engine.i_reg(instr_opcode_lsb_c+2) = '1') or (execute_engine.branch_taken = '1') then -- JAL/JALR or taken branch
          execute_engine.pc_we        <= '1'; -- update PC
          execute_engine.branched_nxt <= '1'; -- this is an actual branch
          fetch_engine.reset          <= '1'; -- trigger new instruction fetch from modified PC
          execute_engine.state_nxt    <= SYS_WAIT;
        else
          execute_engine.state_nxt <= DISPATCH;
        end if;


      when FENCE_OP => -- fence operations - execution
      -- ------------------------------------------------------------
        execute_engine.state_nxt <= SYS_WAIT;
        -- FENCE.I --
        if (CPU_EXTENSION_RISCV_Zifencei = true) then
          execute_engine.pc_mux_sel <= '0'; -- linear next PC = start *new* instruction fetch with next instruction (only relevant for fence.i)
          if (execute_engine.i_reg(instr_funct3_lsb_c) = funct3_fencei_c(0)) then
            execute_engine.pc_we        <= '1'; -- update PC
            execute_engine.branched_nxt <= '1'; -- this is an actual branch
            fetch_engine.reset          <= '1'; -- trigger new instruction fetch from modified PC
            ctrl_nxt(ctrl_bus_fencei_c) <= '1';
          end if;
        end if;
        -- FENCE --
        if (execute_engine.i_reg(instr_funct3_lsb_c) = funct3_fence_c(0)) then
          ctrl_nxt(ctrl_bus_fence_c) <= '1';
        end if;


      when LOADSTORE_0 => -- trigger memory request
      -- ------------------------------------------------------------
        if (execute_engine.i_reg(instr_opcode_msb_c-1) = '0') or (decode_aux.is_atomic_lr = '1') then -- normal load or atomic load-reservate
          ctrl_nxt(ctrl_bus_rd_c) <= '1'; -- read request
        else -- store
          ctrl_nxt(ctrl_bus_wr_c) <= '1'; -- write request
        end if;
        execute_engine.state_nxt <= LOADSTORE_1;


      when LOADSTORE_1 => -- memory latency
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_bus_mi_we_c) <= '1'; -- write input data to MDI (only relevant for LOAD)
        execute_engine.state_nxt   <= LOADSTORE_2;


      when LOADSTORE_2 => -- wait for bus transaction to finish
      -- ------------------------------------------------------------
        -- ALU control (only relevant for atomic memory operations) --
        if (CPU_EXTENSION_RISCV_A = true) then
          ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_atomic_c; -- atomic.SC: result comes from "atomic co-processor"
          ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_cmd_copro_c;
        end if;
        -- register file write-back --
        if (decode_aux.is_atomic_sc = '1') then
          ctrl_nxt(ctrl_rf_in_mux_c) <= '0'; -- RF input = ALU.res (only relevant for atomic.SC)
        else
          ctrl_nxt(ctrl_rf_in_mux_c) <= '1'; -- RF input = memory input (only relevant for LOADs)
        end if;
        --
        ctrl_nxt(ctrl_bus_mi_we_c) <= '1'; -- keep writing input data to MDI (only relevant for load operations)
        -- wait for memory response --
        if ((ma_load_i or be_load_i or ma_store_i or be_store_i) = '1') then -- abort if exception
          atomic_ctrl.env_abort     <= '1'; -- LOCKED (atomic) memory access environment failed (forces SC result to be non-zero => failure)
          ctrl_nxt(ctrl_rf_wb_en_c) <= decode_aux.is_atomic_sc; -- store-conditional failes: allow rf write back of non-zero result
          execute_engine.state_nxt  <= DISPATCH;
        elsif (bus_d_wait_i = '0') then -- wait for bus to finish transaction
          if (execute_engine.i_reg(instr_opcode_msb_c-1) = '0') or (decode_aux.is_atomic_lr = '1') or (decode_aux.is_atomic_sc = '1') then -- load / load-reservate / store conditional
            ctrl_nxt(ctrl_rf_wb_en_c) <= not execute_engine.is_fp; -- allow write back if NOT <FPU-internal operation>
          end if;
          if (CPU_EXTENSION_RISCV_F = true) and (execute_engine.i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c+2) = opcode_flw_c(6 downto 2)) then -- floating-point LOAD.word
            ctrl_nxt(ctrl_cp_fpu_mem_we_c) <= '1'; -- co-processor register file write-back
          end if;
          atomic_ctrl.env_end      <= not decode_aux.is_atomic_lr; -- normal end of LOCKED (atomic) memory access environment - if we are not starting it via LR instruction
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
  csr_access_check: process(execute_engine.i_reg, csr)
    variable csr_wacc_v           : std_ulogic; -- to check access to read-only CSRs
--  variable csr_racc_v           : std_ulogic; -- to check access to write-only CSRs
    variable csr_mcounteren_hpm_v : std_ulogic_vector(28 downto 0); -- max 29 HPM counters
  begin
    -- is this CSR instruction really going to write/read to/from a CSR? --
    if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrw_c) or
       (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrwi_c) then
      csr_wacc_v := '1'; -- always write CSR
--    csr_racc_v := or_all_f(execute_engine.i_reg(instr_rd_msb_c downto instr_rd_lsb_c)); -- read allowed if rd != 0
    else
      csr_wacc_v := or_all_f(execute_engine.i_reg(instr_rs1_msb_c downto instr_rs1_lsb_c)); -- write allowed if rs1/uimm5 != 0
--    csr_racc_v := '1'; -- always read CSR
    end if;

    -- low privilege level access to hpm counters? --
    csr_mcounteren_hpm_v := (others => '0');
    if (CPU_EXTENSION_RISCV_U = true) then
      csr_mcounteren_hpm_v(HPM_NUM_CNTS-1 downto 0) := csr.mcounteren_hpm(HPM_NUM_CNTS-1 downto 0);
    else -- 'mcounteren' CSR is hardwired to zero if user mode is not implemented
      csr_mcounteren_hpm_v := (others => '0');
    end if;

    -- check CSR access --
    case csr.addr is
      -- standard read/write CSRs --
      when csr_fflags_c | csr_frm_c | csr_fcsr_c => csr_acc_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_F); -- full access for everyone if F extension is enabled
      --
      when csr_mstatus_c       => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_mstatush_c      => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_misa_c          => csr_acc_valid <= csr.priv_m_mode;-- and (not csr_wacc_v); -- M-mode only, MISA is read-only in the NEORV32 but we do not cause an exception here for compatibility
      when csr_mie_c           => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_mtvec_c         => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_mscratch_c      => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_mepc_c          => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_mcause_c        => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_mcounteren_c    => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_mtval_c         => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_mip_c           => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      --
      when csr_pmpcfg0_c | csr_pmpcfg1_c | csr_pmpcfg2_c  | csr_pmpcfg3_c  | csr_pmpcfg4_c  | csr_pmpcfg5_c  | csr_pmpcfg6_c  | csr_pmpcfg7_c |
           csr_pmpcfg8_c | csr_pmpcfg9_c | csr_pmpcfg10_c | csr_pmpcfg11_c | csr_pmpcfg12_c | csr_pmpcfg13_c | csr_pmpcfg14_c | csr_pmpcfg15_c =>
        csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      --
      when csr_pmpaddr0_c  | csr_pmpaddr1_c  | csr_pmpaddr2_c  | csr_pmpaddr3_c  | csr_pmpaddr4_c  | csr_pmpaddr5_c  | csr_pmpaddr6_c  | csr_pmpaddr7_c  |
           csr_pmpaddr8_c  | csr_pmpaddr9_c  | csr_pmpaddr10_c | csr_pmpaddr11_c | csr_pmpaddr12_c | csr_pmpaddr13_c | csr_pmpaddr14_c | csr_pmpaddr15_c |
           csr_pmpaddr16_c | csr_pmpaddr17_c | csr_pmpaddr18_c | csr_pmpaddr19_c | csr_pmpaddr20_c | csr_pmpaddr21_c | csr_pmpaddr22_c | csr_pmpaddr23_c |
           csr_pmpaddr24_c | csr_pmpaddr25_c | csr_pmpaddr26_c | csr_pmpaddr27_c | csr_pmpaddr28_c | csr_pmpaddr29_c | csr_pmpaddr30_c | csr_pmpaddr31_c |
           csr_pmpaddr32_c | csr_pmpaddr33_c | csr_pmpaddr34_c | csr_pmpaddr35_c | csr_pmpaddr36_c | csr_pmpaddr37_c | csr_pmpaddr38_c | csr_pmpaddr39_c |
           csr_pmpaddr40_c | csr_pmpaddr41_c | csr_pmpaddr42_c | csr_pmpaddr43_c | csr_pmpaddr44_c | csr_pmpaddr45_c | csr_pmpaddr46_c | csr_pmpaddr47_c |
           csr_pmpaddr48_c | csr_pmpaddr49_c | csr_pmpaddr50_c | csr_pmpaddr51_c | csr_pmpaddr52_c | csr_pmpaddr53_c | csr_pmpaddr54_c | csr_pmpaddr55_c |
           csr_pmpaddr56_c | csr_pmpaddr57_c | csr_pmpaddr58_c | csr_pmpaddr59_c | csr_pmpaddr60_c | csr_pmpaddr61_c | csr_pmpaddr62_c | csr_pmpaddr63_c =>
        csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      --
      when csr_mcountinhibit_c => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      --
      when csr_mhpmevent3_c  | csr_mhpmevent4_c  | csr_mhpmevent5_c  | csr_mhpmevent6_c  | csr_mhpmevent7_c  | csr_mhpmevent8_c  |
           csr_mhpmevent9_c  | csr_mhpmevent10_c | csr_mhpmevent11_c | csr_mhpmevent12_c | csr_mhpmevent13_c | csr_mhpmevent14_c |
           csr_mhpmevent15_c | csr_mhpmevent16_c | csr_mhpmevent17_c | csr_mhpmevent18_c | csr_mhpmevent19_c | csr_mhpmevent20_c |
           csr_mhpmevent21_c | csr_mhpmevent22_c | csr_mhpmevent23_c | csr_mhpmevent24_c | csr_mhpmevent25_c | csr_mhpmevent26_c |
           csr_mhpmevent27_c | csr_mhpmevent28_c | csr_mhpmevent29_c | csr_mhpmevent30_c | csr_mhpmevent31_c =>
        csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      --
      when csr_mcycle_c        => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_minstret_c      => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      --
      when csr_mhpmcounter3_c  | csr_mhpmcounter4_c  | csr_mhpmcounter5_c  | csr_mhpmcounter6_c  | csr_mhpmcounter7_c  | csr_mhpmcounter8_c  |
           csr_mhpmcounter9_c  | csr_mhpmcounter10_c | csr_mhpmcounter11_c | csr_mhpmcounter12_c | csr_mhpmcounter13_c | csr_mhpmcounter14_c |
           csr_mhpmcounter15_c | csr_mhpmcounter16_c | csr_mhpmcounter17_c | csr_mhpmcounter18_c | csr_mhpmcounter19_c | csr_mhpmcounter20_c |
           csr_mhpmcounter21_c | csr_mhpmcounter22_c | csr_mhpmcounter23_c | csr_mhpmcounter24_c | csr_mhpmcounter25_c | csr_mhpmcounter26_c |
           csr_mhpmcounter27_c | csr_mhpmcounter28_c | csr_mhpmcounter29_c | csr_mhpmcounter30_c | csr_mhpmcounter31_c =>
        csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      --
      when csr_mcycleh_c       => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_minstreth_c     => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      --
      when csr_mhpmcounter3h_c  | csr_mhpmcounter4h_c  | csr_mhpmcounter5h_c  | csr_mhpmcounter6h_c  | csr_mhpmcounter7h_c  | csr_mhpmcounter8h_c  |
           csr_mhpmcounter9h_c  | csr_mhpmcounter10h_c | csr_mhpmcounter11h_c | csr_mhpmcounter12h_c | csr_mhpmcounter13h_c | csr_mhpmcounter14h_c |
           csr_mhpmcounter15h_c | csr_mhpmcounter16h_c | csr_mhpmcounter17h_c | csr_mhpmcounter18h_c | csr_mhpmcounter19h_c | csr_mhpmcounter20h_c |
           csr_mhpmcounter21h_c | csr_mhpmcounter22h_c | csr_mhpmcounter23h_c | csr_mhpmcounter24h_c | csr_mhpmcounter25h_c | csr_mhpmcounter26h_c |
           csr_mhpmcounter27h_c | csr_mhpmcounter28h_c | csr_mhpmcounter29h_c | csr_mhpmcounter30h_c | csr_mhpmcounter31h_c =>
        csr_acc_valid <= csr.priv_m_mode; -- M-mode only

      -- standard read-only CSRs --
      when csr_cycle_c         => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr.mcounteren_cy); -- M-mode, U-mode if authorized, read-only
      when csr_time_c          => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr.mcounteren_tm); -- M-mode, U-mode if authorized, read-only
      when csr_instret_c       => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr.mcounteren_ir); -- M-mode, U-mode if authorized, read-only
      --
      when csr_hpmcounter3_c   => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(00)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter4_c   => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(01)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter5_c   => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(02)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter6_c   => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(03)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter7_c   => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(04)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter8_c   => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(05)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter9_c   => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(06)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter10_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(07)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter11_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(08)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter12_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(09)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter13_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(10)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter14_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(11)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter15_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(12)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter16_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(13)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter17_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(14)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter18_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(15)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter19_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(16)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter20_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(17)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter21_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(18)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter22_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(19)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter23_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(20)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter24_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(21)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter25_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(22)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter26_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(23)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter27_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(24)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter28_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(25)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter29_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(26)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter30_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(27)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter31_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(28)); -- M-mode, U-mode if authorized, read-only
      --
      when csr_cycleh_c        => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr.mcounteren_cy); -- M-mode, U-mode if authorized, read-only
      when csr_timeh_c         => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr.mcounteren_tm); -- M-mode, U-mode if authorized, read-only
      when csr_instreth_c      => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr.mcounteren_ir); -- M-mode, U-mode if authorized, read-only
      --
      when csr_hpmcounter3h_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(00)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter4h_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(01)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter5h_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(02)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter6h_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(03)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter7h_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(04)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter8h_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(05)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter9h_c  => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(06)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter10h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(07)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter11h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(08)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter12h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(09)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter13h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(10)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter14h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(11)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter15h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(12)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter16h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(13)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter17h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(14)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter18h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(15)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter19h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(16)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter20h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(17)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter21h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(18)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter22h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(19)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter23h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(20)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter24h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(21)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter25h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(22)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter26h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(23)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter27h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(24)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter28h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(25)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter29h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(26)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter30h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(27)); -- M-mode, U-mode if authorized, read-only
      when csr_hpmcounter31h_c => csr_acc_valid <= (not csr_wacc_v) and (csr.priv_m_mode or csr_mcounteren_hpm_v(28)); -- M-mode, U-mode if authorized, read-only
      --
      when csr_mvendorid_c     => csr_acc_valid <= (not csr_wacc_v) and csr.priv_m_mode; -- M-mode only, read-only
      when csr_marchid_c       => csr_acc_valid <= (not csr_wacc_v) and csr.priv_m_mode; -- M-mode only, read-only
      when csr_mimpid_c        => csr_acc_valid <= (not csr_wacc_v) and csr.priv_m_mode; -- M-mode only, read-only
      when csr_mhartid_c       => csr_acc_valid <= (not csr_wacc_v) and csr.priv_m_mode; -- M-mode only, read-only
      -- custom read-only CSRs --
      when csr_mzext_c         => csr_acc_valid <= (not csr_wacc_v) and csr.priv_m_mode; -- M-mode only, read-only
      --
      when others              => csr_acc_valid <= '0'; -- invalid access
    end case;
  end process csr_access_check;


  -- Illegal Instruction Check --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  illegal_instruction_check: process(execute_engine, decode_aux, csr_acc_valid)
    variable opcode_v : std_ulogic_vector(6 downto 0);
  begin
    -- illegal instructions are checked in the EXECUTE stage
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

      -- check instructions --
      opcode_v := execute_engine.i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c+2) & "11";
      case opcode_v is

        -- check sufficient LUI, UIPC, JAL (only check actual OPCODE) --
        -- ------------------------------------------------------------
        when opcode_lui_c | opcode_auipc_c | opcode_jal_c =>
          illegal_instruction <= '0';
          -- illegal E-CPU register? --
          if (CPU_EXTENSION_RISCV_E = true) and (execute_engine.i_reg(instr_rd_msb_c) = '1') then
            illegal_register <= '1';
          end if;

        when opcode_alu_c => -- check ALU.funct3 & ALU.funct7
        -- ------------------------------------------------------------
          if (execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000001") then -- MULDIV
            if (CPU_EXTENSION_RISCV_M = false) then -- not implemented
              illegal_instruction <= '1';
            end if;
          elsif (decode_aux.is_bitmanip_reg = '1') then -- bit manipulation
            if (CPU_EXTENSION_RISCV_B = false) then -- not implemented
              illegal_instruction <= '1';
            end if;
          elsif ((execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_subadd_c) or
                 (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sr_c)) and -- ADD/SUB or SRA/SRL check
                ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) /= "0000000") and
                 (execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) /= "0100000")) then -- ADD/SUB or SRA/SRL select
            illegal_instruction <= '1';
          else
            illegal_instruction <= '0';
          end if;
          -- illegal E-CPU register? --
          if (CPU_EXTENSION_RISCV_E = true) and
             ((execute_engine.i_reg(instr_rs2_msb_c) = '1') or (execute_engine.i_reg(instr_rs1_msb_c) = '1') or (execute_engine.i_reg(instr_rd_msb_c) = '1')) then
            illegal_register <= '1';
          end if;

        when opcode_alui_c => -- check ALUI.funct7
        -- ------------------------------------------------------------
          if (decode_aux.is_bitmanip_imm = '1') then -- bit manipulation
            if (CPU_EXTENSION_RISCV_B = false) then -- not implemented
              illegal_instruction <= '1';
            end if;
          elsif ((execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sll_c) and
              (execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) /= "0000000")) or -- shift logical left
             ((execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sr_c) and
              ((execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) /= "0000000") and
               (execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) /= "0100000"))) then -- shift right
            illegal_instruction <= '1';
          else
            illegal_instruction <= '0';
          end if;
          -- illegal E-CPU register? --
          if (CPU_EXTENSION_RISCV_E = true) and ((execute_engine.i_reg(instr_rs1_msb_c) = '1') or (execute_engine.i_reg(instr_rd_msb_c) = '1')) then
            illegal_register <= '1';
          end if;

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
          if (CPU_EXTENSION_RISCV_E = true) and ((execute_engine.i_reg(instr_rs1_msb_c) = '1') or (execute_engine.i_reg(instr_rd_msb_c) = '1')) then
            illegal_register <= '1';
          end if;

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
          if (CPU_EXTENSION_RISCV_E = true) and ((execute_engine.i_reg(instr_rs2_msb_c) = '1') or (execute_engine.i_reg(instr_rs1_msb_c) = '1')) then
            illegal_register <= '1';
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
          if (CPU_EXTENSION_RISCV_E = true) and ((execute_engine.i_reg(instr_rs2_msb_c) = '1') or (execute_engine.i_reg(instr_rs1_msb_c) = '1')) then
            illegal_register <= '1';
          end if;

        when opcode_jalr_c => -- check JALR.funct3
        -- ------------------------------------------------------------
          if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "000") then
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
          end if;
          -- illegal E-CPU register? --
          if (CPU_EXTENSION_RISCV_E = true) and ((execute_engine.i_reg(instr_rs1_msb_c) = '1') or (execute_engine.i_reg(instr_rd_msb_c) = '1')) then
            illegal_register <= '1';
          end if;

        when opcode_fence_c => -- fence instructions
        -- ------------------------------------------------------------
          if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_fencei_c) and (CPU_EXTENSION_RISCV_Zifencei = true) then -- FENCE.I
            illegal_instruction <= '0';
          elsif (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_fence_c) then -- FENCE
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
          end if;

        when opcode_syscsr_c => -- check system instructions
        -- ------------------------------------------------------------
          -- CSR access --
          if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrw_c) or
             (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrs_c) or
             (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrc_c) or
             (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrwi_c) or
             (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrsi_c) or
             (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrci_c) then
            -- valid CSR access? --
            if (csr_acc_valid = '1') then
              illegal_instruction <= '0';
            else
              illegal_instruction <= '1';
            end if;
            -- illegal E-CPU register? --
            if (CPU_EXTENSION_RISCV_E = true) then
              if (execute_engine.i_reg(instr_funct3_msb_c) = '0') then -- reg-reg CSR
                illegal_register <= execute_engine.i_reg(instr_rs1_msb_c) or execute_engine.i_reg(instr_rd_msb_c);
              else -- reg-imm CSR
                illegal_register <= execute_engine.i_reg(instr_rd_msb_c);
              end if;
            end if;

          -- ecall, ebreak, mret, wfi --
          elsif (execute_engine.i_reg(instr_rd_msb_c  downto instr_rd_lsb_c)  = "00000") and
                (execute_engine.i_reg(instr_rs1_msb_c downto instr_rs1_lsb_c) = "00000") then
            if (execute_engine.i_reg(instr_funct12_msb_c  downto instr_funct12_lsb_c) = funct12_ecall_c)  or -- ECALL
               (execute_engine.i_reg(instr_funct12_msb_c  downto instr_funct12_lsb_c) = funct12_ebreak_c) or -- EBREAK 
               (execute_engine.i_reg(instr_funct12_msb_c  downto instr_funct12_lsb_c) = funct12_mret_c)   or -- MRET
               (execute_engine.i_reg(instr_funct12_msb_c  downto instr_funct12_lsb_c) = funct12_wfi_c) then  -- WFI
              illegal_instruction <= '0';
            else
              illegal_instruction <= '1';
            end if;
          else
            illegal_instruction <= '1';
          end if;

        when opcode_atomic_c => -- atomic instructions
        -- ------------------------------------------------------------
          if (CPU_EXTENSION_RISCV_A = true) and -- atomic memory operations (A extension) enabled
             ((execute_engine.i_reg(instr_funct5_msb_c downto instr_funct5_lsb_c) = funct5_a_lr_c) or -- LR
              (execute_engine.i_reg(instr_funct5_msb_c downto instr_funct5_lsb_c) = funct5_a_sc_c)) then -- SC
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
          end if;

        when opcode_fop_c => -- floating point operations (dual-operand)
        -- ------------------------------------------------------------
          if (CPU_EXTENSION_RISCV_F = true) and -- F extension enabled
             (execute_engine.i_reg(instr_funct7_lsb_c+1 downto instr_funct7_lsb_c) = float_single_c) and -- single-precision operations
             ((decode_aux.is_float_f_reg = '1') or (decode_aux.is_float_i_reg = '1')) then -- float_reg or int_reg operations
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
          end if;

        when opcode_flw_c | opcode_fsw_c => -- floating point load/store word
        -- ------------------------------------------------------------
          if (CPU_EXTENSION_RISCV_F = true) and -- F extension enabled
             (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "010") then -- 32-bit transfer size
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
          end if;

        when others => -- undefined instruction -> illegal!
        -- ------------------------------------------------------------
          illegal_instruction <= '1';

      end case;
    else
      illegal_opcode_lsbs <= '0';
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
      trap_ctrl.irq_ack   <= (others => '0');
      trap_ctrl.env_start <= '0';
      trap_ctrl.cause     <= trap_reset_c;
      trap_ctrl.firq_sync <= (others => '0');
    elsif rising_edge(clk_i) then
      if (CPU_EXTENSION_RISCV_Zicsr = true) then
        -- exception buffer: misaligned load/store/instruction address
        trap_ctrl.exc_buf(exception_lalign_c)    <= (trap_ctrl.exc_buf(exception_lalign_c)    or ma_load_i)          and (not trap_ctrl.exc_ack);
        trap_ctrl.exc_buf(exception_salign_c)    <= (trap_ctrl.exc_buf(exception_salign_c)    or ma_store_i)         and (not trap_ctrl.exc_ack);
        trap_ctrl.exc_buf(exception_ialign_c)    <= (trap_ctrl.exc_buf(exception_ialign_c)    or trap_ctrl.instr_ma) and (not trap_ctrl.exc_ack);
        -- exception buffer: load/store/instruction bus access error
        trap_ctrl.exc_buf(exception_laccess_c)   <= (trap_ctrl.exc_buf(exception_laccess_c)   or be_load_i)          and (not trap_ctrl.exc_ack);
        trap_ctrl.exc_buf(exception_saccess_c)   <= (trap_ctrl.exc_buf(exception_saccess_c)   or be_store_i)         and (not trap_ctrl.exc_ack);
        trap_ctrl.exc_buf(exception_iaccess_c)   <= (trap_ctrl.exc_buf(exception_iaccess_c)   or trap_ctrl.instr_be) and (not trap_ctrl.exc_ack);
        -- exception buffer: illegal instruction / env call / break point
        trap_ctrl.exc_buf(exception_m_envcall_c) <= (trap_ctrl.exc_buf(exception_m_envcall_c) or (trap_ctrl.env_call and csr.priv_m_mode)) and (not trap_ctrl.exc_ack);
        trap_ctrl.exc_buf(exception_u_envcall_c) <= (trap_ctrl.exc_buf(exception_u_envcall_c) or (trap_ctrl.env_call and csr.priv_u_mode)) and (not trap_ctrl.exc_ack);
        trap_ctrl.exc_buf(exception_break_c)     <= (trap_ctrl.exc_buf(exception_break_c)     or trap_ctrl.break_point)                    and (not trap_ctrl.exc_ack);
        trap_ctrl.exc_buf(exception_iillegal_c)  <= (trap_ctrl.exc_buf(exception_iillegal_c)  or trap_ctrl.instr_il)                       and (not trap_ctrl.exc_ack);
        -- interrupt buffer: machine software/external/timer interrupt
        trap_ctrl.irq_buf(interrupt_msw_irq_c)   <= csr.mie_msie and (trap_ctrl.irq_buf(interrupt_msw_irq_c)   or msw_irq_i)   and (not (trap_ctrl.irq_ack(interrupt_msw_irq_c)   or csr.mip_clear(interrupt_msw_irq_c)));
        trap_ctrl.irq_buf(interrupt_mext_irq_c)  <= csr.mie_meie and (trap_ctrl.irq_buf(interrupt_mext_irq_c)  or mext_irq_i)  and (not (trap_ctrl.irq_ack(interrupt_mext_irq_c)  or csr.mip_clear(interrupt_mext_irq_c)));
        trap_ctrl.irq_buf(interrupt_mtime_irq_c) <= csr.mie_mtie and (trap_ctrl.irq_buf(interrupt_mtime_irq_c) or mtime_irq_i) and (not (trap_ctrl.irq_ack(interrupt_mtime_irq_c) or csr.mip_clear(interrupt_mtime_irq_c)));
        -- interrupt buffer: NEORV32-specific fast interrupts
        trap_ctrl.firq_sync <= firq_i;
        for i in 0 to 15 loop
          trap_ctrl.irq_buf(interrupt_firq_0_c+i) <= csr.mie_firqe(i) and (trap_ctrl.irq_buf(interrupt_firq_0_c+i) or trap_ctrl.firq_sync(i)) and (not (trap_ctrl.irq_ack(interrupt_firq_0_c+i) or csr.mip_clear(interrupt_firq_0_c+i)));
        end loop;
        -- trap control --
        if (trap_ctrl.env_start = '0') then -- no started trap handler
          if (trap_ctrl.exc_fire = '1') or ((trap_ctrl.irq_fire = '1') and -- trap triggered!
             ((execute_engine.state = EXECUTE) or (execute_engine.state = TRAP_ENTER))) then -- fire IRQs in EXECUTE or TRAP state only to continue execution even on permanent IRQ
            trap_ctrl.cause     <= trap_ctrl.cause_nxt;   -- capture source ID for program (for mcause csr)
            trap_ctrl.exc_ack   <= '1';                   -- clear execption
            trap_ctrl.irq_ack   <= trap_ctrl.irq_ack_nxt; -- clear interrupt with interrupt ACK mask
            trap_ctrl.env_start <= '1';                   -- now execute engine can start trap handler
          end if;
        else -- trap waiting to get started
          if (trap_ctrl.env_start_ack = '1') then -- start of trap handler acknowledged by execution engine
            trap_ctrl.exc_ack   <= '0';
            trap_ctrl.irq_ack   <= (others => '0');
            trap_ctrl.env_start <= '0';
          end if;
        end if;
      end if;
    end if;
  end process trap_controller;

  -- any exception/interrupt? --
  trap_ctrl.exc_fire <= or_all_f(trap_ctrl.exc_buf); -- exceptions/faults CANNOT be masked
  trap_ctrl.irq_fire <= or_all_f(trap_ctrl.irq_buf) and csr.mstatus_mie; -- interrupts CAN be masked

  -- current pending interrupts (for CSR.MIP register) --
  csr.mip_status <= trap_ctrl.irq_buf;

  -- acknowledge mask output --
  firq_ack_o <= trap_ctrl.irq_ack(interrupt_firq_15_c downto interrupt_firq_0_c);


  -- Trap Priority Encoder ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trap_priority: process(trap_ctrl)
  begin
    -- defaults --
    trap_ctrl.cause_nxt   <= (others => '0');
    trap_ctrl.irq_ack_nxt <= (others => '0');

    -- the following traps are caused by *asynchronous* exceptions (= interrupts)
    -- here we do need a specific acknowledge mask since several sources can trigger at once

    -- interrupt: 1.11 machine external interrupt --
    if (trap_ctrl.irq_buf(interrupt_mext_irq_c) = '1') then
      trap_ctrl.cause_nxt <= trap_mei_c;
      trap_ctrl.irq_ack_nxt(interrupt_mext_irq_c) <= '1';

    -- interrupt: 1.3 machine SW interrupt --
    elsif (trap_ctrl.irq_buf(interrupt_msw_irq_c) = '1') then
      trap_ctrl.cause_nxt <= trap_msi_c;
      trap_ctrl.irq_ack_nxt(interrupt_msw_irq_c) <= '1';

    -- interrupt: 1.7 machine timer interrupt --
    elsif (trap_ctrl.irq_buf(interrupt_mtime_irq_c) = '1') then
      trap_ctrl.cause_nxt <= trap_mti_c;
      trap_ctrl.irq_ack_nxt(interrupt_mtime_irq_c) <= '1';


    -- interrupt: 1.16 fast interrupt channel 0 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_0_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq0_c;
      trap_ctrl.irq_ack_nxt(interrupt_firq_0_c) <= '1';

    -- interrupt: 1.17 fast interrupt channel 1 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_1_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq1_c;
      trap_ctrl.irq_ack_nxt(interrupt_firq_1_c) <= '1';

    -- interrupt: 1.18 fast interrupt channel 2 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_2_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq2_c;
      trap_ctrl.irq_ack_nxt(interrupt_firq_2_c) <= '1';

    -- interrupt: 1.19 fast interrupt channel 3 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_3_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq3_c;
      trap_ctrl.irq_ack_nxt(interrupt_firq_3_c) <= '1';

    -- interrupt: 1.20 fast interrupt channel 4 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_4_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq4_c;
      trap_ctrl.irq_ack_nxt(interrupt_firq_4_c) <= '1';

    -- interrupt: 1.21 fast interrupt channel 5 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_5_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq5_c;
      trap_ctrl.irq_ack_nxt(interrupt_firq_5_c) <= '1';

    -- interrupt: 1.22 fast interrupt channel 6 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_6_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq6_c;
      trap_ctrl.irq_ack_nxt(interrupt_firq_6_c) <= '1';

    -- interrupt: 1.23 fast interrupt channel 7 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_7_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq7_c;
      trap_ctrl.irq_ack_nxt(interrupt_firq_7_c) <= '1';

    -- interrupt: 1.24 fast interrupt channel 8 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_8_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq8_c;
      trap_ctrl.irq_ack_nxt(interrupt_firq_8_c) <= '1';

    -- interrupt: 1.25 fast interrupt channel 9 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_9_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq9_c;
      trap_ctrl.irq_ack_nxt(interrupt_firq_9_c) <= '1';

    -- interrupt: 1.26 fast interrupt channel 10 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_10_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq10_c;
      trap_ctrl.irq_ack_nxt(interrupt_firq_10_c) <= '1';

    -- interrupt: 1.27 fast interrupt channel 11 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_11_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq11_c;
      trap_ctrl.irq_ack_nxt(interrupt_firq_11_c) <= '1';

    -- interrupt: 1.28 fast interrupt channel 12 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_12_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq12_c;
      trap_ctrl.irq_ack_nxt(interrupt_firq_12_c) <= '1';

    -- interrupt: 1.29 fast interrupt channel 13 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_13_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq13_c;
      trap_ctrl.irq_ack_nxt(interrupt_firq_13_c) <= '1';

    -- interrupt: 1.30 fast interrupt channel 14 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_14_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq14_c;
      trap_ctrl.irq_ack_nxt(interrupt_firq_14_c) <= '1';

    -- interrupt: 1.31 fast interrupt channel 15 --
    elsif (trap_ctrl.irq_buf(interrupt_firq_15_c) = '1') then
      trap_ctrl.cause_nxt <= trap_firq15_c;
      trap_ctrl.irq_ack_nxt(interrupt_firq_15_c) <= '1';


    -- the following traps are caused by *synchronous* exceptions (= 'classic' exceptions)
    -- here we do not need a specific acknowledge mask since only one exception (the one
    -- with highest priority) is evaluated at once

    -- exception: 0.1 instruction access fault --
    elsif (trap_ctrl.exc_buf(exception_iaccess_c) = '1') then
      trap_ctrl.cause_nxt <= trap_iba_c;

    -- exception: 0.2 illegal instruction --
    elsif (trap_ctrl.exc_buf(exception_iillegal_c) = '1') then
      trap_ctrl.cause_nxt <= trap_iil_c;

    -- exception: 0.0 instruction address misaligned --
    elsif (trap_ctrl.exc_buf(exception_ialign_c) = '1') then
      trap_ctrl.cause_nxt <= trap_ima_c;


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

    -- not implemented --
    else
      trap_ctrl.cause_nxt   <= (others => '0');
      trap_ctrl.irq_ack_nxt <= (others => '0');
    end if;
  end process trap_priority;


  -- Atomic Memory Access - Status Controller -----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  atomic_memacc_controller: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      atomic_ctrl.lock       <= '0';
      atomic_ctrl.env_end_ff <= '0';
    elsif rising_edge(clk_i) then
      if (CPU_EXTENSION_RISCV_A = true) then
        if (atomic_ctrl.env_end_ff = '1') or -- normal termination
           (atomic_ctrl.env_abort = '1') or  -- fast termination (error)
           (trap_ctrl.env_start = '1') then  -- triggered trap -> failure
          atomic_ctrl.lock <= '0';
        elsif (atomic_ctrl.env_start = '1') then
          atomic_ctrl.lock <= '1';
        end if;
        atomic_ctrl.env_end_ff <= atomic_ctrl.env_end;
      else
        atomic_ctrl.lock       <= '0';
        atomic_ctrl.env_end_ff <= '0';
      end if;
    end if;
  end process atomic_memacc_controller;
  

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
  begin
    if (rstn_i = '0') then
      csr.we           <= '0';
      --
      csr.mstatus_mie  <= '0';
      csr.mstatus_mpie <= '0';
      csr.mstatus_mpp  <= priv_mode_m_c; -- start in MACHINE mode
      csr.privilege    <= priv_mode_m_c; -- start in MACHINE mode
      csr.mie_msie     <= '0';
      csr.mie_meie     <= '0';
      csr.mie_mtie     <= '0';
      csr.mie_firqe    <= (others => '0');
      csr.mtvec        <= (others => '0');
      csr.mscratch     <= x"19880704"; -- :)
      csr.mepc         <= (others => '0');
      csr.mcause       <= trap_reset_c; -- mcause = TRAP_CODE_RESET (hardware reset, "non-maskable interrupt")
      csr.mtval        <= (others => '0');
      csr.mip_clear    <= (others => '0');
      --
      csr.pmpcfg  <= (others => (others => '0'));
      csr.pmpaddr <= (others => (others => '1'));
      --
      csr.mhpmevent <= (others => (others => '0'));
      --
      csr.mcounteren_cy  <= '0';
      csr.mcounteren_tm  <= '0';
      csr.mcounteren_ir  <= '0';
      csr.mcounteren_hpm <= (others => '0');
      --
      csr.mcountinhibit_cy  <= '0';
      csr.mcountinhibit_ir  <= '0';
      csr.mcountinhibit_hpm <= (others => '0');
      --
      csr.fflags <= (others => '0');
      csr.frm    <= (others => '0');

    elsif rising_edge(clk_i) then
      -- write access? --
      csr.we <= csr.we_nxt;
      if (CPU_EXTENSION_RISCV_Zicsr = true) then

        -- defaults --
        csr.mip_clear <= (others => '0');

        -- --------------------------------------------------------------------------------
        -- CSR access by application software
        -- --------------------------------------------------------------------------------
        if (csr.we = '1') then -- manual update

          -- user floating-point CSRs --
          -- --------------------------------------------------------------------
          if (csr.addr(11 downto 4) = csr_class_float_c) then -- floating point CSR class
            -- R/W: fflags - floating-point (FPU) exception flags --
            if (csr.addr(3 downto 0) = csr_fflags_c(3 downto 0)) and (CPU_EXTENSION_RISCV_F = true) then
              csr.fflags <= csr.wdata(4 downto 0);
            end if;
            -- R/W: frm - floating-point (FPU) rounding mode --
            if (csr.addr(3 downto 0) = csr_frm_c(3 downto 0)) and (CPU_EXTENSION_RISCV_F = true) then
              csr.frm <= csr.wdata(2 downto 0);
            end if;
            -- R/W: fflags - floating-point (FPU) control/status (frm + fflags) --
            if (csr.addr(3 downto 0) = csr_fcsr_c(3 downto 0)) and (CPU_EXTENSION_RISCV_F = true) then
              csr.frm    <= csr.wdata(7 downto 5);
              csr.fflags <= csr.wdata(4 downto 0);
            end if;
          end if;

          -- machine trap setup --
          -- --------------------------------------------------------------------
          if (csr.addr(11 downto 4) = csr_setup_c) then -- ftrap setup CSR class
            -- R/W: mstatus - machine status register --
            if (csr.addr(3 downto 0) = csr_mstatus_c(3 downto 0)) then
              csr.mstatus_mie  <= csr.wdata(03);
              csr.mstatus_mpie <= csr.wdata(07);
              if (CPU_EXTENSION_RISCV_U = true) then -- user mode implemented
                csr.mstatus_mpp(0) <= csr.wdata(11) or csr.wdata(12);
                csr.mstatus_mpp(1) <= csr.wdata(11) or csr.wdata(12);
              else -- only machine mode is available
                csr.mstatus_mpp <= priv_mode_m_c;
              end if;
            end if;
            -- R/W: mie - machine interrupt enable register --
            if (csr.addr(3 downto 0) = csr_mie_c(3 downto 0)) then
              csr.mie_msie <= csr.wdata(03); -- machine SW IRQ enable
              csr.mie_mtie <= csr.wdata(07); -- machine TIMER IRQ enable
              csr.mie_meie <= csr.wdata(11); -- machine EXT IRQ enable
              for i in 0 to 15 loop -- fast interrupt channels 0..15
                csr.mie_firqe(i) <= csr.wdata(16+i);
              end loop; -- i
            end if;
            -- R/W: mtvec - machine trap-handler base address (for ALL exceptions) --
            if (csr.addr(3 downto 0) = csr_mtvec_c(3 downto 0)) then
              csr.mtvec <= csr.wdata(data_width_c-1 downto 2) & "00"; -- mtvec.MODE=0
            end if;
            -- R/W: machine counter enable register --
            if (csr.addr(3 downto 0) = csr_mcounteren_c(3 downto 0)) then
              if (CPU_EXTENSION_RISCV_U = true) then -- this CSR is hardwired to zero if user mode is not implemented
                csr.mcounteren_cy  <= csr.wdata(0); -- enable user-level access to cycle[h]
                csr.mcounteren_tm  <= csr.wdata(1); -- enable user-level access to time[h]
                csr.mcounteren_ir  <= csr.wdata(2); -- enable user-level access to instret[h]
                csr.mcounteren_hpm <= csr.wdata(csr.mcounteren_hpm'left+3 downto 3); -- enable user-level access to hpmcounterx[h]
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
              csr.mepc <= csr.wdata(data_width_c-1 downto 1) & '0';
            end if;
            -- R/W: mcause - machine trap cause --
            if (csr.addr(3 downto 0) = csr_mcause_c(3 downto 0)) then
              csr.mcause(csr.mcause'left) <= csr.wdata(31); -- 1: interrupt, 0: exception
              csr.mcause(4 downto 0)      <= csr.wdata(4 downto 0); -- identifier
            end if;
            -- R/W: mtval - machine bad address/instruction --
            if (csr.addr(3 downto 0) = csr_mtval_c(3 downto 0)) then
              csr.mtval <= csr.wdata;
            end if;
            -- R/W: mip - machine interrupt pending --
            if (csr.addr(3 downto 0) = csr_mip_c(3 downto 0)) then
              csr.mip_clear(interrupt_msw_irq_c)   <= not csr.wdata(03);
              csr.mip_clear(interrupt_mtime_irq_c) <= not csr.wdata(07);
              csr.mip_clear(interrupt_mext_irq_c)  <= not csr.wdata(11);
              for i in 0 to 15 loop -- fast interrupt channels 0..15
                csr.mip_clear(interrupt_firq_0_c+i) <= not csr.wdata(16+i);
              end loop; -- i
            end if;
          end if;

          -- physical memory protection: R/W: pmpcfg* - PMP configuration registers --
          -- --------------------------------------------------------------------
          if (csr.addr(11 downto 4) = csr_class_pmpcfg_c) then -- pmp configuration CSR class
            if (PMP_NUM_REGIONS > 0) then
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
          if (csr.addr(11 downto 4) =  csr_pmpaddr0_c(11 downto 4)) or (csr.addr(11 downto 4) = csr_pmpaddr16_c(11 downto 4)) or
             (csr.addr(11 downto 4) = csr_pmpaddr32_c(11 downto 4)) or (csr.addr(11 downto 4) = csr_pmpaddr48_c(11 downto 4)) then 
            if (PMP_NUM_REGIONS > 0) then
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
          -- R/W: mcountinhibit - machine counter-inhibit register --
          if (csr.addr = csr_mcountinhibit_c) then
            csr.mcountinhibit_cy  <= csr.wdata(0); -- enable auto-increment of [m]cycle[h] counter
            csr.mcountinhibit_ir  <= csr.wdata(2); -- enable auto-increment of [m]instret[h] counter
            csr.mcountinhibit_hpm <= csr.wdata(csr.mcountinhibit_hpm'left+3 downto 3); -- enable auto-increment of [m]hpmcounter*[h] counter
          end if;

          -- machine performance-monitoring event selector --
          -- --------------------------------------------------------------------
          if (unsigned(csr.addr) >= unsigned(csr_mhpmevent3_c)) and (unsigned(csr.addr) <= unsigned(csr_mhpmevent31_c)) then
            if (HPM_NUM_CNTS > 0) then
              for i in 0 to HPM_NUM_CNTS-1 loop
                if (csr.addr(4 downto 0) = std_ulogic_vector(to_unsigned(i+3, 5))) then
                  csr.mhpmevent(i) <= csr.wdata(csr.mhpmevent(i)'left downto 0);
                  csr.mhpmevent(i)(1) <= '0'; -- would be used for "TIME"
                end if;
              end loop; -- i (CSRs)
            end if;
          end if;


        -- --------------------------------------------------------------------------------
        -- CSR access by hardware
        -- --------------------------------------------------------------------------------
        else

          -- floating-point (FPU) exception flags --
          -- --------------------------------------------------------------------
          if (CPU_EXTENSION_RISCV_F = true) and (execute_engine.state = ALU_WAIT) then -- FIXME?
            csr.fflags <= csr.fflags or fpu_flags_i; -- accumulate flags ("accrued exception flags")
          end if;

          -- mcause, mepc, mtval: machine trap cause, PC and value register --
          -- --------------------------------------------------------------------
          if (trap_ctrl.env_start_ack = '1') then -- trap handler starting?
            -- trap cause ID code --
            csr.mcause(csr.mcause'left) <= trap_ctrl.cause(trap_ctrl.cause'left); -- 1: interrupt, 0: exception
            csr.mcause(4 downto 0)      <= trap_ctrl.cause(4 downto 0); -- identifier
            -- trap PC --
            if (trap_ctrl.cause(trap_ctrl.cause'left) = '1') then -- for INTERRUPTS
              csr.mepc <= execute_engine.pc(data_width_c-1 downto 1) & '0'; -- this is the CURRENT pc = interrupted instruction
            else -- for EXCEPTIONS
              csr.mepc <= execute_engine.last_pc(data_width_c-1 downto 1) & '0'; -- this is the LAST pc = last executed instruction
            end if;
            -- trap value --
            case trap_ctrl.cause is
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

          -- mstatus: context switch --
          -- --------------------------------------------------------------------
          if (trap_ctrl.env_start_ack = '1') then -- ENTER: trap handler starting?
            csr.mstatus_mie  <= '0'; -- disable interrupts
            csr.mstatus_mpie <= csr.mstatus_mie; -- buffer previous mie state
            if (CPU_EXTENSION_RISCV_U = true) then -- implement user mode
              csr.privilege   <= priv_mode_m_c; -- execute trap in machine mode
              csr.mstatus_mpp <= csr.privilege; -- buffer previous privilege mode
            end if;
          elsif (trap_ctrl.env_end = '1') then -- EXIT: return from exception
            csr.mstatus_mie  <= csr.mstatus_mpie; -- restore global IRQ enable flag
            csr.mstatus_mpie <= '1';
            if (CPU_EXTENSION_RISCV_U = true) then -- implement user mode
              csr.privilege   <= csr.mstatus_mpp; -- go back to previous privilege mode
              csr.mstatus_mpp <= priv_mode_m_c;
            end if;
          end if;
          -- user mode NOT implemented --
          if (CPU_EXTENSION_RISCV_U = false) then
            csr.privilege   <= priv_mode_m_c;
            csr.mstatus_mpp <= priv_mode_m_c;
          end if;

        end if; -- /hardware csr access
      end if;

      -- --------------------------------------------------------------------------------
      -- override write access for disabled functions
      -- --------------------------------------------------------------------------------

      -- user mode disabled --
      if (CPU_EXTENSION_RISCV_U = false) then
        csr.privilege      <= priv_mode_m_c;
        csr.mstatus_mpp    <= priv_mode_m_c;
        csr.mcounteren_cy  <= '0';
        csr.mcounteren_tm  <= '0';
        csr.mcounteren_ir  <= '0';
        csr.mcounteren_hpm <= (others => '0');
      end if;

      -- pmp disabled --
      if (PMP_NUM_REGIONS = 0) then
        csr.pmpcfg  <= (others => (others => '0'));
        csr.pmpaddr <= (others => (others => '1'));
      end if;

      -- hpms disabled --
      if (HPM_NUM_CNTS = 0) then
        csr.mhpmevent         <= (others => (others => '0'));
        csr.mcounteren_hpm    <= (others => '0');
        csr.mcountinhibit_hpm <= (others => '0');
      end if;

      -- floating-point extension disabled --
      if (CPU_EXTENSION_RISCV_F = false) then
        csr.fflags <= (others => '0');
        csr.frm    <= (others => '0');
      end if;

    end if;
  end process csr_write_access;

  -- decode privilege mode --
  csr.priv_m_mode <= '1' when (csr.privilege = priv_mode_m_c) else '0';
  csr.priv_u_mode <= '1' when (csr.privilege = priv_mode_u_c) else '0';

  -- PMP configuration output to bus unit --
  pmp_output: process(csr)
  begin
    pmp_addr_o <= (others => (others => '0'));
    pmp_ctrl_o <= (others => (others => '0'));
    for i in 0 to PMP_NUM_REGIONS-1 loop
      pmp_addr_o(i) <= csr.pmpaddr(i) & "11";
      pmp_addr_o(i)(index_size_f(PMP_MIN_GRANULARITY)-4 downto 0) <= (others => '1');
      pmp_ctrl_o(i) <= csr.pmpcfg(i);
    end loop; -- i
  end process pmp_output;

  -- PMP read dummy --
  pmp_rd_dummy: process(csr)
  begin
    csr.pmpcfg_rd  <= (others => (others => '0'));
    csr.pmpaddr_rd <= (others => (others => '0'));
    for i in 0 to PMP_NUM_REGIONS-1 loop
      csr.pmpcfg_rd(i)  <= csr.pmpcfg(i);
      csr.pmpaddr_rd(i) <= csr.pmpaddr(i);
      if (csr.pmpcfg(i)(4 downto 3) = "00") then -- mode = off
        csr.pmpaddr_rd(i)(index_size_f(PMP_MIN_GRANULARITY)-3 downto 0) <= (others => '0'); -- required for granularity check by SW
      end if;
    end loop; -- i
  end process pmp_rd_dummy;

  -- FPU rounding mode --
  fpu_rm_o <= csr.frm;


  -- Control and Status Registers - Counters ------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_counters: process(clk_i)
  begin
    -- Counter CSRs (each counter is split into two 32-bit counters)
    if rising_edge(clk_i) then

      -- [m]cycle --
      if (csr.we = '1') and (csr.addr = csr_mcycle_c) then -- write access
        csr.mcycle <= '0' & csr.wdata;
        mcycle_msb <= '0';
      elsif (csr.mcountinhibit_cy = '0') and (cnt_event(hpmcnt_event_cy_c) = '1') then -- non-inhibited automatic update
        csr.mcycle <= std_ulogic_vector(unsigned(csr.mcycle) + 1);
        mcycle_msb <= csr.mcycle(csr.mcycle'left);
      end if;

      -- [m]cycleh --
      if (csr.we = '1') and (csr.addr = csr_mcycleh_c) then -- write access
        csr.mcycleh <= csr.wdata;
      elsif ((mcycle_msb xor csr.mcycle(csr.mcycle'left)) = '1') then -- automatic update (continued)
        csr.mcycleh <= std_ulogic_vector(unsigned(csr.mcycleh) + 1);
      end if;

      -- [m]instret --
      if (csr.we = '1') and (csr.addr = csr_minstret_c) then -- write access
        csr.minstret <= '0' & csr.wdata;
        minstret_msb <= '0';
      elsif (csr.mcountinhibit_ir = '0') and (cnt_event(hpmcnt_event_ir_c) = '1') and (cnt_event(hpmcnt_event_cy_c) = '1') then -- non-inhibited automatic update
        csr.minstret <= std_ulogic_vector(unsigned(csr.minstret) + 1);
        minstret_msb <= csr.minstret(csr.minstret'left);
      end if;

      -- [m]instreth --
      if (csr.we = '1') and (csr.addr = csr_minstreth_c) then -- write access
        csr.minstreth <= csr.wdata;
      elsif ((minstret_msb xor csr.minstret(csr.minstret'left)) = '1') then -- automatic update (continued)
        csr.minstreth <= std_ulogic_vector(unsigned(csr.minstreth) + 1);
      end if;

      -- [machine] hardware performance monitors (counters) --
      for i in 0 to HPM_NUM_CNTS-1 loop
        -- [m]hpmcounter* --
        if (csr.we = '1') and (csr.addr = std_ulogic_vector(unsigned(csr_mhpmcounter3_c) + i)) then -- write access
          csr.mhpmcounter(i) <= '0' & csr.wdata;
          mhpmcounter_msb(i) <= '0';
        elsif (csr.mcountinhibit_hpm(i) = '0') and (hpmcnt_trigger(i) = '1') then -- non-inhibited automatic update
          csr.mhpmcounter(i) <= std_ulogic_vector(unsigned(csr.mhpmcounter(i)) + 1);
          mhpmcounter_msb(i) <= csr.mhpmcounter(i)(csr.mhpmcounter(i)'left);
        end if;

        -- [m]hpmcounter*h --
        if (csr.we = '1') and (csr.addr = std_ulogic_vector(unsigned(csr_mhpmcounter3h_c) + i)) then -- write access
          csr.mhpmcounterh(i) <= csr.wdata;
        elsif ((mhpmcounter_msb(i) xor csr.mhpmcounter(i)(csr.mhpmcounter(i)'left)) = '1') then -- automatic update (continued)
          csr.mhpmcounterh(i) <= std_ulogic_vector(unsigned(csr.mhpmcounterh(i)) + 1);
        end if;
      end loop; -- i

    end if;
  end process csr_counters;

  -- hpm read dummy --
  hpm_rd_dummy: process(csr)
  begin
    csr.mhpmevent_rd    <= (others => (others => '0'));
    csr.mhpmcounter_rd  <= (others => (others => '0'));
    csr.mhpmcounterh_rd <= (others => (others => '0'));
    for i in 0 to HPM_NUM_CNTS-1 loop
      csr.mhpmevent_rd(i)    <= csr.mhpmevent(i);
      csr.mhpmcounter_rd(i)  <= csr.mhpmcounter(i);
      csr.mhpmcounterh_rd(i) <= csr.mhpmcounterh(i);
    end loop; -- i
  end process hpm_rd_dummy;


  -- (HPM) Counter Event Control ------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  hpmcnt_ctrl: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- buffer event sources --
      cnt_event <= cnt_event_nxt;
      -- enable selected triggers by ANDing actual events and according CSR configuration bits --
      -- OR everything to see if counter should increment --
      hpmcnt_trigger <= (others => '0'); -- default
      for i in 0 to HPM_NUM_CNTS-1 loop
        hpmcnt_trigger(i) <= or_all_f(cnt_event and csr.mhpmevent(i)(cnt_event'left downto 0));
      end loop; -- i
    end if;
  end process hpmcnt_ctrl;

  -- counter event trigger - RISC-V specific --
  cnt_event_nxt(hpmcnt_event_cy_c)    <= not execute_engine.sleep; -- active cycle
  cnt_event_nxt(hpmcnt_event_never_c) <= '0'; -- undefined (never)
  cnt_event_nxt(hpmcnt_event_ir_c)    <= '1' when (execute_engine.state = EXECUTE) else '0'; -- retired instruction

  -- counter event trigger - custom / NEORV32-specific --
  cnt_event_nxt(hpmcnt_event_cir_c)     <= '1' when (execute_engine.state = EXECUTE)      and (execute_engine.is_ci = '1')             else '0'; -- retired compressed instruction
  cnt_event_nxt(hpmcnt_event_wait_if_c) <= '1' when (fetch_engine.state   = IFETCH_ISSUE) and (fetch_engine.state_prev = IFETCH_ISSUE) else '0'; -- instruction fetch memory wait cycle
  cnt_event_nxt(hpmcnt_event_wait_ii_c) <= '1' when (execute_engine.state = DISPATCH)     and (execute_engine.state_prev = DISPATCH)   else '0'; -- instruction issue wait cycle
  cnt_event_nxt(hpmcnt_event_wait_mc_c) <= '1' when (execute_engine.state = ALU_WAIT)     and (execute_engine.state_prev = ALU_WAIT)   else '0'; -- multi-cycle alu-operation wait cycle

  cnt_event_nxt(hpmcnt_event_load_c)    <= '1' when (execute_engine.state = LOADSTORE_1) and (ctrl(ctrl_bus_rd_c) = '1')               else '0'; -- load operation
  cnt_event_nxt(hpmcnt_event_store_c)   <= '1' when (execute_engine.state = LOADSTORE_1) and (ctrl(ctrl_bus_wr_c) = '1')               else '0'; -- store operation
  cnt_event_nxt(hpmcnt_event_wait_ls_c) <= '1' when (execute_engine.state = LOADSTORE_2) and (execute_engine.state_prev = LOADSTORE_2) else '0'; -- load/store memory wait cycle

  cnt_event_nxt(hpmcnt_event_jump_c)    <= '1' when (execute_engine.state = BRANCH) and (execute_engine.i_reg(instr_opcode_lsb_c+2) = '1') else '0'; -- jump (unconditional)
  cnt_event_nxt(hpmcnt_event_branch_c)  <= '1' when (execute_engine.state = BRANCH) and (execute_engine.i_reg(instr_opcode_lsb_c+2) = '0') else '0'; -- branch (conditional, taken or not taken)
  cnt_event_nxt(hpmcnt_event_tbranch_c) <= '1' when (execute_engine.state = BRANCH) and (execute_engine.i_reg(instr_opcode_lsb_c+2) = '0') and (execute_engine.branch_taken = '1') else '0'; -- taken branch (conditional)

  cnt_event_nxt(hpmcnt_event_trap_c)    <= '1' when (trap_ctrl.env_start_ack = '1')                                    else '0'; -- entered trap
  cnt_event_nxt(hpmcnt_event_illegal_c) <= '1' when (trap_ctrl.env_start_ack = '1') and (trap_ctrl.cause = trap_iil_c) else '0'; -- illegal operation


  -- Control and Status Registers - Read Access ---------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      csr.re    <= csr.re_nxt; -- read access?
      csr.rdata <= (others => '0'); -- default output
      if (CPU_EXTENSION_RISCV_Zicsr = true) and (csr.re = '1') then
        case csr.addr is

          -- user floating-point CSRs --
          -- --------------------------------------------------------------------
          when csr_fflags_c => -- R/W: fflags - floating-point (FPU) exception flags
            csr.rdata <= (others => '0');
            if (CPU_EXTENSION_RISCV_F = true) then -- FPU implemented
              csr.rdata(4 downto 0) <= csr.fflags;
            end if;
          when csr_frm_c => -- R/W: frm - floating-point (FPU) rounding mode
            csr.rdata <= (others => '0');
            if (CPU_EXTENSION_RISCV_F = true) then -- FPU implemented
              csr.rdata(2 downto 0) <= csr.frm;
            end if;
          when csr_fcsr_c => -- R/W: fflags - floating-point (FPU) control/status (frm + fflags)
            csr.rdata <= (others => '0');
            if (CPU_EXTENSION_RISCV_F = true) then -- FPU implemented
              csr.rdata(7 downto 5) <= csr.frm;
              csr.rdata(4 downto 0) <= csr.fflags;
            end if;

          -- machine trap setup --
          when csr_mstatus_c => -- R/W: mstatus - machine status register
            csr.rdata(03) <= csr.mstatus_mie; -- MIE
            csr.rdata(06) <= '1' and bool_to_ulogic_f(CPU_EXTENSION_RISCV_U); -- UBE: CPU/Processor is BIG-ENDIAN (in user-mode)
            csr.rdata(07) <= csr.mstatus_mpie; -- MPIE
            csr.rdata(11) <= csr.mstatus_mpp(0); -- MPP: machine previous privilege mode low
            csr.rdata(12) <= csr.mstatus_mpp(1); -- MPP: machine previous privilege mode high
          when csr_mstatush_c => -- R/-: mstatush - machine status register - high part
            csr.rdata(05) <= '1'; -- MBE: CPU/Processor is BIG-ENDIAN (in machine-mode)
          when csr_misa_c => -- R/-: misa - ISA and extensions
            csr.rdata(00) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_A);     -- A CPU extension
            csr.rdata(01) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_B);     -- B CPU extension
            csr.rdata(02) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_C);     -- C CPU extension
            csr.rdata(04) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_E);     -- E CPU extension
            csr.rdata(05) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_F);     -- F CPU extension
            csr.rdata(08) <= not bool_to_ulogic_f(CPU_EXTENSION_RISCV_E); -- I CPU extension (if not E)
            csr.rdata(12) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_M);     -- M CPU extension
            csr.rdata(20) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_U);     -- U CPU extension
            csr.rdata(23) <= '1';                                         -- X CPU extension (non-std extensions)
            csr.rdata(30) <= '1'; -- 32-bit architecture (MXL lo)
            csr.rdata(31) <= '0'; -- 32-bit architecture (MXL hi)
          when csr_mie_c => -- R/W: mie - machine interrupt-enable register
            csr.rdata(03) <= csr.mie_msie; -- machine software IRQ enable
            csr.rdata(07) <= csr.mie_mtie; -- machine timer IRQ enable
            csr.rdata(11) <= csr.mie_meie; -- machine external IRQ enable
            for i in 0 to 15 loop -- fast interrupt channels 0..15 enable
              csr.rdata(16+i) <= csr.mie_firqe(i);
            end loop; -- i
          when csr_mtvec_c => -- R/W: mtvec - machine trap-handler base address (for ALL exceptions)
            csr.rdata <= csr.mtvec(data_width_c-1 downto 2) & "00"; -- mtvec.MODE=0
          when csr_mcounteren_c => -- R/W: machine counter enable register
            if (CPU_EXTENSION_RISCV_U = true) then -- this CSR is hardwired to zero if user mode is not implemented
              csr.rdata(0) <= csr.mcounteren_cy; -- enable user-level access to cycle[h]
              csr.rdata(1) <= csr.mcounteren_tm; -- enable user-level access to time[h]
              csr.rdata(2) <= csr.mcounteren_ir; -- enable user-level access to instret[h]
              csr.rdata(csr.mcounteren_hpm'left+3 downto 3) <= csr.mcounteren_hpm; -- enable user-level access to hpmcounterx[h]
            else
              csr.rdata <= (others => '0');
            end if;

          -- machine trap handling --
          when csr_mscratch_c => -- R/W: mscratch - machine scratch register
            csr.rdata <= csr.mscratch;
          when csr_mepc_c => -- R/W: mepc - machine exception program counter
            csr.rdata <= csr.mepc(data_width_c-1 downto 1) & '0';
          when csr_mcause_c => -- R/W: mcause - machine trap cause
            csr.rdata(31) <= csr.mcause(csr.mcause'left);
            csr.rdata(csr.mcause'left-1 downto 0) <= csr.mcause(csr.mcause'left-1 downto 0);
          when csr_mtval_c => -- R/W: mtval - machine bad address or instruction
            csr.rdata <= csr.mtval;
          when csr_mip_c => -- R/W: mip - machine interrupt pending
            csr.rdata(03) <= csr.mip_status(interrupt_msw_irq_c);
            csr.rdata(07) <= csr.mip_status(interrupt_mtime_irq_c);
            csr.rdata(11) <= csr.mip_status(interrupt_mext_irq_c);
            for i in 0 to 15 loop -- fast interrupt channels 0..15 pending
              csr.rdata(16+i) <= csr.mip_status(interrupt_firq_0_c+i);
            end loop; -- i

          -- physical memory protection - configuration --
          when csr_pmpcfg0_c  => csr.rdata <= csr.pmpcfg_rd(03) & csr.pmpcfg_rd(02) & csr.pmpcfg_rd(01) & csr.pmpcfg_rd(00); -- R/W: pmpcfg0
          when csr_pmpcfg1_c  => csr.rdata <= csr.pmpcfg_rd(07) & csr.pmpcfg_rd(06) & csr.pmpcfg_rd(05) & csr.pmpcfg_rd(04); -- R/W: pmpcfg1
          when csr_pmpcfg2_c  => csr.rdata <= csr.pmpcfg_rd(11) & csr.pmpcfg_rd(10) & csr.pmpcfg_rd(09) & csr.pmpcfg_rd(08); -- R/W: pmpcfg2
          when csr_pmpcfg3_c  => csr.rdata <= csr.pmpcfg_rd(15) & csr.pmpcfg_rd(14) & csr.pmpcfg_rd(13) & csr.pmpcfg_rd(12); -- R/W: pmpcfg3
          when csr_pmpcfg4_c  => csr.rdata <= csr.pmpcfg_rd(19) & csr.pmpcfg_rd(18) & csr.pmpcfg_rd(17) & csr.pmpcfg_rd(16); -- R/W: pmpcfg4
          when csr_pmpcfg5_c  => csr.rdata <= csr.pmpcfg_rd(23) & csr.pmpcfg_rd(22) & csr.pmpcfg_rd(21) & csr.pmpcfg_rd(20); -- R/W: pmpcfg5
          when csr_pmpcfg6_c  => csr.rdata <= csr.pmpcfg_rd(27) & csr.pmpcfg_rd(26) & csr.pmpcfg_rd(25) & csr.pmpcfg_rd(24); -- R/W: pmpcfg6
          when csr_pmpcfg7_c  => csr.rdata <= csr.pmpcfg_rd(31) & csr.pmpcfg_rd(30) & csr.pmpcfg_rd(29) & csr.pmpcfg_rd(28); -- R/W: pmpcfg7
          when csr_pmpcfg8_c  => csr.rdata <= csr.pmpcfg_rd(35) & csr.pmpcfg_rd(34) & csr.pmpcfg_rd(33) & csr.pmpcfg_rd(32); -- R/W: pmpcfg8
          when csr_pmpcfg9_c  => csr.rdata <= csr.pmpcfg_rd(39) & csr.pmpcfg_rd(38) & csr.pmpcfg_rd(37) & csr.pmpcfg_rd(36); -- R/W: pmpcfg9
          when csr_pmpcfg10_c => csr.rdata <= csr.pmpcfg_rd(43) & csr.pmpcfg_rd(42) & csr.pmpcfg_rd(41) & csr.pmpcfg_rd(40); -- R/W: pmpcfg10
          when csr_pmpcfg11_c => csr.rdata <= csr.pmpcfg_rd(47) & csr.pmpcfg_rd(46) & csr.pmpcfg_rd(45) & csr.pmpcfg_rd(44); -- R/W: pmpcfg11
          when csr_pmpcfg12_c => csr.rdata <= csr.pmpcfg_rd(51) & csr.pmpcfg_rd(50) & csr.pmpcfg_rd(49) & csr.pmpcfg_rd(48); -- R/W: pmpcfg12
          when csr_pmpcfg13_c => csr.rdata <= csr.pmpcfg_rd(55) & csr.pmpcfg_rd(54) & csr.pmpcfg_rd(53) & csr.pmpcfg_rd(52); -- R/W: pmpcfg13
          when csr_pmpcfg14_c => csr.rdata <= csr.pmpcfg_rd(59) & csr.pmpcfg_rd(58) & csr.pmpcfg_rd(57) & csr.pmpcfg_rd(56); -- R/W: pmpcfg14
          when csr_pmpcfg15_c => csr.rdata <= csr.pmpcfg_rd(63) & csr.pmpcfg_rd(62) & csr.pmpcfg_rd(61) & csr.pmpcfg_rd(60); -- R/W: pmpcfg15

          -- physical memory protection - addresses --
          when csr_pmpaddr0_c  => csr.rdata <= csr.pmpaddr_rd(00); -- R/W: pmpaddr0
          when csr_pmpaddr1_c  => csr.rdata <= csr.pmpaddr_rd(01); -- R/W: pmpaddr1
          when csr_pmpaddr2_c  => csr.rdata <= csr.pmpaddr_rd(02); -- R/W: pmpaddr2
          when csr_pmpaddr3_c  => csr.rdata <= csr.pmpaddr_rd(03); -- R/W: pmpaddr3
          when csr_pmpaddr4_c  => csr.rdata <= csr.pmpaddr_rd(04); -- R/W: pmpaddr4
          when csr_pmpaddr5_c  => csr.rdata <= csr.pmpaddr_rd(05); -- R/W: pmpaddr5
          when csr_pmpaddr6_c  => csr.rdata <= csr.pmpaddr_rd(06); -- R/W: pmpaddr6
          when csr_pmpaddr7_c  => csr.rdata <= csr.pmpaddr_rd(07); -- R/W: pmpaddr7
          when csr_pmpaddr8_c  => csr.rdata <= csr.pmpaddr_rd(08); -- R/W: pmpaddr8
          when csr_pmpaddr9_c  => csr.rdata <= csr.pmpaddr_rd(09); -- R/W: pmpaddr9
          when csr_pmpaddr10_c => csr.rdata <= csr.pmpaddr_rd(10); -- R/W: pmpaddr10
          when csr_pmpaddr11_c => csr.rdata <= csr.pmpaddr_rd(11); -- R/W: pmpaddr11
          when csr_pmpaddr12_c => csr.rdata <= csr.pmpaddr_rd(12); -- R/W: pmpaddr12
          when csr_pmpaddr13_c => csr.rdata <= csr.pmpaddr_rd(13); -- R/W: pmpaddr13
          when csr_pmpaddr14_c => csr.rdata <= csr.pmpaddr_rd(14); -- R/W: pmpaddr14
          when csr_pmpaddr15_c => csr.rdata <= csr.pmpaddr_rd(15); -- R/W: pmpaddr15
          when csr_pmpaddr16_c => csr.rdata <= csr.pmpaddr_rd(16); -- R/W: pmpaddr16
          when csr_pmpaddr17_c => csr.rdata <= csr.pmpaddr_rd(17); -- R/W: pmpaddr17
          when csr_pmpaddr18_c => csr.rdata <= csr.pmpaddr_rd(18); -- R/W: pmpaddr18
          when csr_pmpaddr19_c => csr.rdata <= csr.pmpaddr_rd(19); -- R/W: pmpaddr19
          when csr_pmpaddr20_c => csr.rdata <= csr.pmpaddr_rd(20); -- R/W: pmpaddr20
          when csr_pmpaddr21_c => csr.rdata <= csr.pmpaddr_rd(21); -- R/W: pmpaddr21
          when csr_pmpaddr22_c => csr.rdata <= csr.pmpaddr_rd(22); -- R/W: pmpaddr22
          when csr_pmpaddr23_c => csr.rdata <= csr.pmpaddr_rd(23); -- R/W: pmpaddr23
          when csr_pmpaddr24_c => csr.rdata <= csr.pmpaddr_rd(24); -- R/W: pmpaddr24
          when csr_pmpaddr25_c => csr.rdata <= csr.pmpaddr_rd(25); -- R/W: pmpaddr25
          when csr_pmpaddr26_c => csr.rdata <= csr.pmpaddr_rd(26); -- R/W: pmpaddr26
          when csr_pmpaddr27_c => csr.rdata <= csr.pmpaddr_rd(27); -- R/W: pmpaddr27
          when csr_pmpaddr28_c => csr.rdata <= csr.pmpaddr_rd(28); -- R/W: pmpaddr28
          when csr_pmpaddr29_c => csr.rdata <= csr.pmpaddr_rd(29); -- R/W: pmpaddr29
          when csr_pmpaddr30_c => csr.rdata <= csr.pmpaddr_rd(30); -- R/W: pmpaddr30
          when csr_pmpaddr31_c => csr.rdata <= csr.pmpaddr_rd(31); -- R/W: pmpaddr31
          when csr_pmpaddr32_c => csr.rdata <= csr.pmpaddr_rd(32); -- R/W: pmpaddr32
          when csr_pmpaddr33_c => csr.rdata <= csr.pmpaddr_rd(33); -- R/W: pmpaddr33
          when csr_pmpaddr34_c => csr.rdata <= csr.pmpaddr_rd(34); -- R/W: pmpaddr34
          when csr_pmpaddr35_c => csr.rdata <= csr.pmpaddr_rd(35); -- R/W: pmpaddr35
          when csr_pmpaddr36_c => csr.rdata <= csr.pmpaddr_rd(36); -- R/W: pmpaddr36
          when csr_pmpaddr37_c => csr.rdata <= csr.pmpaddr_rd(37); -- R/W: pmpaddr37
          when csr_pmpaddr38_c => csr.rdata <= csr.pmpaddr_rd(38); -- R/W: pmpaddr38
          when csr_pmpaddr39_c => csr.rdata <= csr.pmpaddr_rd(39); -- R/W: pmpaddr39
          when csr_pmpaddr40_c => csr.rdata <= csr.pmpaddr_rd(40); -- R/W: pmpaddr40
          when csr_pmpaddr41_c => csr.rdata <= csr.pmpaddr_rd(41); -- R/W: pmpaddr41
          when csr_pmpaddr42_c => csr.rdata <= csr.pmpaddr_rd(42); -- R/W: pmpaddr42
          when csr_pmpaddr43_c => csr.rdata <= csr.pmpaddr_rd(43); -- R/W: pmpaddr43
          when csr_pmpaddr44_c => csr.rdata <= csr.pmpaddr_rd(44); -- R/W: pmpaddr44
          when csr_pmpaddr45_c => csr.rdata <= csr.pmpaddr_rd(45); -- R/W: pmpaddr45
          when csr_pmpaddr46_c => csr.rdata <= csr.pmpaddr_rd(46); -- R/W: pmpaddr46
          when csr_pmpaddr47_c => csr.rdata <= csr.pmpaddr_rd(47); -- R/W: pmpaddr47
          when csr_pmpaddr48_c => csr.rdata <= csr.pmpaddr_rd(48); -- R/W: pmpaddr48
          when csr_pmpaddr49_c => csr.rdata <= csr.pmpaddr_rd(49); -- R/W: pmpaddr49
          when csr_pmpaddr50_c => csr.rdata <= csr.pmpaddr_rd(50); -- R/W: pmpaddr50
          when csr_pmpaddr51_c => csr.rdata <= csr.pmpaddr_rd(51); -- R/W: pmpaddr51
          when csr_pmpaddr52_c => csr.rdata <= csr.pmpaddr_rd(52); -- R/W: pmpaddr52
          when csr_pmpaddr53_c => csr.rdata <= csr.pmpaddr_rd(53); -- R/W: pmpaddr53
          when csr_pmpaddr54_c => csr.rdata <= csr.pmpaddr_rd(54); -- R/W: pmpaddr54
          when csr_pmpaddr55_c => csr.rdata <= csr.pmpaddr_rd(55); -- R/W: pmpaddr55
          when csr_pmpaddr56_c => csr.rdata <= csr.pmpaddr_rd(56); -- R/W: pmpaddr56
          when csr_pmpaddr57_c => csr.rdata <= csr.pmpaddr_rd(57); -- R/W: pmpaddr57
          when csr_pmpaddr58_c => csr.rdata <= csr.pmpaddr_rd(58); -- R/W: pmpaddr58
          when csr_pmpaddr59_c => csr.rdata <= csr.pmpaddr_rd(59); -- R/W: pmpaddr59
          when csr_pmpaddr60_c => csr.rdata <= csr.pmpaddr_rd(60); -- R/W: pmpaddr60
          when csr_pmpaddr61_c => csr.rdata <= csr.pmpaddr_rd(61); -- R/W: pmpaddr61
          when csr_pmpaddr62_c => csr.rdata <= csr.pmpaddr_rd(62); -- R/W: pmpaddr62
          when csr_pmpaddr63_c => csr.rdata <= csr.pmpaddr_rd(63); -- R/W: pmpaddr63

          -- machine counter setup --
          -- --------------------------------------------------------------------
          when csr_mcountinhibit_c => -- R/W: mcountinhibit - machine counter-inhibit register
            csr.rdata(0) <= csr.mcountinhibit_cy; -- enable auto-increment of [m]cycle[h] counter
            csr.rdata(2) <= csr.mcountinhibit_ir; -- enable auto-increment of [m]instret[h] counter
            csr.rdata(csr.mcountinhibit_hpm'left+3 downto 3) <= csr.mcountinhibit_hpm; -- enable auto-increment of [m]hpmcounterx[h] counter

          -- machine performance-monitoring event selector --
          when csr_mhpmevent3_c  => csr.rdata(csr.mhpmevent_rd(00)'left downto 0) <= csr.mhpmevent_rd(00); -- R/W: mhpmevent3
          when csr_mhpmevent4_c  => csr.rdata(csr.mhpmevent_rd(01)'left downto 0) <= csr.mhpmevent_rd(01); -- R/W: mhpmevent4
          when csr_mhpmevent5_c  => csr.rdata(csr.mhpmevent_rd(02)'left downto 0) <= csr.mhpmevent_rd(02); -- R/W: mhpmevent5
          when csr_mhpmevent6_c  => csr.rdata(csr.mhpmevent_rd(03)'left downto 0) <= csr.mhpmevent_rd(03); -- R/W: mhpmevent6
          when csr_mhpmevent7_c  => csr.rdata(csr.mhpmevent_rd(04)'left downto 0) <= csr.mhpmevent_rd(04); -- R/W: mhpmevent7
          when csr_mhpmevent8_c  => csr.rdata(csr.mhpmevent_rd(05)'left downto 0) <= csr.mhpmevent_rd(05); -- R/W: mhpmevent8
          when csr_mhpmevent9_c  => csr.rdata(csr.mhpmevent_rd(06)'left downto 0) <= csr.mhpmevent_rd(06); -- R/W: mhpmevent9
          when csr_mhpmevent10_c => csr.rdata(csr.mhpmevent_rd(07)'left downto 0) <= csr.mhpmevent_rd(07); -- R/W: mhpmevent10
          when csr_mhpmevent11_c => csr.rdata(csr.mhpmevent_rd(08)'left downto 0) <= csr.mhpmevent_rd(08); -- R/W: mhpmevent11
          when csr_mhpmevent12_c => csr.rdata(csr.mhpmevent_rd(09)'left downto 0) <= csr.mhpmevent_rd(09); -- R/W: mhpmevent12
          when csr_mhpmevent13_c => csr.rdata(csr.mhpmevent_rd(10)'left downto 0) <= csr.mhpmevent_rd(10); -- R/W: mhpmevent13
          when csr_mhpmevent14_c => csr.rdata(csr.mhpmevent_rd(11)'left downto 0) <= csr.mhpmevent_rd(11); -- R/W: mhpmevent14
          when csr_mhpmevent15_c => csr.rdata(csr.mhpmevent_rd(12)'left downto 0) <= csr.mhpmevent_rd(12); -- R/W: mhpmevent15
          when csr_mhpmevent16_c => csr.rdata(csr.mhpmevent_rd(13)'left downto 0) <= csr.mhpmevent_rd(13); -- R/W: mhpmevent16
          when csr_mhpmevent17_c => csr.rdata(csr.mhpmevent_rd(14)'left downto 0) <= csr.mhpmevent_rd(14); -- R/W: mhpmevent17
          when csr_mhpmevent18_c => csr.rdata(csr.mhpmevent_rd(15)'left downto 0) <= csr.mhpmevent_rd(15); -- R/W: mhpmevent18
          when csr_mhpmevent19_c => csr.rdata(csr.mhpmevent_rd(16)'left downto 0) <= csr.mhpmevent_rd(16); -- R/W: mhpmevent19
          when csr_mhpmevent20_c => csr.rdata(csr.mhpmevent_rd(17)'left downto 0) <= csr.mhpmevent_rd(17); -- R/W: mhpmevent20
          when csr_mhpmevent21_c => csr.rdata(csr.mhpmevent_rd(18)'left downto 0) <= csr.mhpmevent_rd(18); -- R/W: mhpmevent21
          when csr_mhpmevent22_c => csr.rdata(csr.mhpmevent_rd(19)'left downto 0) <= csr.mhpmevent_rd(19); -- R/W: mhpmevent22
          when csr_mhpmevent23_c => csr.rdata(csr.mhpmevent_rd(20)'left downto 0) <= csr.mhpmevent_rd(20); -- R/W: mhpmevent23
          when csr_mhpmevent24_c => csr.rdata(csr.mhpmevent_rd(21)'left downto 0) <= csr.mhpmevent_rd(21); -- R/W: mhpmevent24
          when csr_mhpmevent25_c => csr.rdata(csr.mhpmevent_rd(22)'left downto 0) <= csr.mhpmevent_rd(22); -- R/W: mhpmevent25
          when csr_mhpmevent26_c => csr.rdata(csr.mhpmevent_rd(23)'left downto 0) <= csr.mhpmevent_rd(23); -- R/W: mhpmevent26
          when csr_mhpmevent27_c => csr.rdata(csr.mhpmevent_rd(24)'left downto 0) <= csr.mhpmevent_rd(24); -- R/W: mhpmevent27
          when csr_mhpmevent28_c => csr.rdata(csr.mhpmevent_rd(25)'left downto 0) <= csr.mhpmevent_rd(25); -- R/W: mhpmevent28
          when csr_mhpmevent29_c => csr.rdata(csr.mhpmevent_rd(26)'left downto 0) <= csr.mhpmevent_rd(26); -- R/W: mhpmevent29
          when csr_mhpmevent30_c => csr.rdata(csr.mhpmevent_rd(27)'left downto 0) <= csr.mhpmevent_rd(27); -- R/W: mhpmevent30
          when csr_mhpmevent31_c => csr.rdata(csr.mhpmevent_rd(28)'left downto 0) <= csr.mhpmevent_rd(28); -- R/W: mhpmevent31

          -- counters and timers --
          when csr_cycle_c | csr_mcycle_c => -- (R)/(W): [m]cycle: Cycle counter LOW
            csr.rdata <= csr.mcycle(31 downto 0);
          when csr_time_c => -- (R)/-: time: System time LOW (from MTIME unit)
            csr.rdata <= time_i(31 downto 0);
          when csr_instret_c | csr_minstret_c => -- (R)/(W): [m]instret: Instructions-retired counter LOW
            csr.rdata <= csr.minstret(31 downto 0);
          when csr_cycleh_c | csr_mcycleh_c => -- (R)/(W): [m]cycleh: Cycle counter HIGH
            csr.rdata <= csr.mcycleh(31 downto 0);
          when csr_timeh_c => -- (R)/-: timeh: System time HIGH (from MTIME unit)
            csr.rdata <= time_i(63 downto 32);
          when csr_instreth_c | csr_minstreth_c => -- (R)/(W): [m]instreth: Instructions-retired counter HIGH
            csr.rdata <= csr.minstreth(31 downto 0);

          -- hardware performance counters --
          when csr_hpmcounter3_c   | csr_mhpmcounter3_c   => csr.rdata <= csr.mhpmcounter_rd(00)(31 downto 0); -- (R)/(W): [m]hpmcounter3 - low
          when csr_hpmcounter4_c   | csr_mhpmcounter4_c   => csr.rdata <= csr.mhpmcounter_rd(01)(31 downto 0); -- (R)/(W): [m]hpmcounter4 - low
          when csr_hpmcounter5_c   | csr_mhpmcounter5_c   => csr.rdata <= csr.mhpmcounter_rd(02)(31 downto 0); -- (R)/(W): [m]hpmcounter5 - low
          when csr_hpmcounter6_c   | csr_mhpmcounter6_c   => csr.rdata <= csr.mhpmcounter_rd(03)(31 downto 0); -- (R)/(W): [m]hpmcounter6 - low
          when csr_hpmcounter7_c   | csr_mhpmcounter7_c   => csr.rdata <= csr.mhpmcounter_rd(04)(31 downto 0); -- (R)/(W): [m]hpmcounter7 - low
          when csr_hpmcounter8_c   | csr_mhpmcounter8_c   => csr.rdata <= csr.mhpmcounter_rd(05)(31 downto 0); -- (R)/(W): [m]hpmcounter8 - low
          when csr_hpmcounter9_c   | csr_mhpmcounter9_c   => csr.rdata <= csr.mhpmcounter_rd(06)(31 downto 0); -- (R)/(W): [m]hpmcounter9 - low
          when csr_hpmcounter10_c  | csr_mhpmcounter10_c  => csr.rdata <= csr.mhpmcounter_rd(07)(31 downto 0); -- (R)/(W): [m]hpmcounter10 - low
          when csr_hpmcounter11_c  | csr_mhpmcounter11_c  => csr.rdata <= csr.mhpmcounter_rd(08)(31 downto 0); -- (R)/(W): [m]hpmcounter11 - low
          when csr_hpmcounter12_c  | csr_mhpmcounter12_c  => csr.rdata <= csr.mhpmcounter_rd(09)(31 downto 0); -- (R)/(W): [m]hpmcounter12 - low
          when csr_hpmcounter13_c  | csr_mhpmcounter13_c  => csr.rdata <= csr.mhpmcounter_rd(10)(31 downto 0); -- (R)/(W): [m]hpmcounter13 - low
          when csr_hpmcounter14_c  | csr_mhpmcounter14_c  => csr.rdata <= csr.mhpmcounter_rd(11)(31 downto 0); -- (R)/(W): [m]hpmcounter14 - low
          when csr_hpmcounter15_c  | csr_mhpmcounter15_c  => csr.rdata <= csr.mhpmcounter_rd(12)(31 downto 0); -- (R)/(W): [m]hpmcounter15 - low
          when csr_hpmcounter16_c  | csr_mhpmcounter16_c  => csr.rdata <= csr.mhpmcounter_rd(13)(31 downto 0); -- (R)/(W): [m]hpmcounter16 - low
          when csr_hpmcounter17_c  | csr_mhpmcounter17_c  => csr.rdata <= csr.mhpmcounter_rd(14)(31 downto 0); -- (R)/(W): [m]hpmcounter17 - low
          when csr_hpmcounter18_c  | csr_mhpmcounter18_c  => csr.rdata <= csr.mhpmcounter_rd(15)(31 downto 0); -- (R)/(W): [m]hpmcounter18 - low
          when csr_hpmcounter19_c  | csr_mhpmcounter19_c  => csr.rdata <= csr.mhpmcounter_rd(16)(31 downto 0); -- (R)/(W): [m]hpmcounter19 - low
          when csr_hpmcounter20_c  | csr_mhpmcounter20_c  => csr.rdata <= csr.mhpmcounter_rd(17)(31 downto 0); -- (R)/(W): [m]hpmcounter20 - low
          when csr_hpmcounter21_c  | csr_mhpmcounter21_c  => csr.rdata <= csr.mhpmcounter_rd(18)(31 downto 0); -- (R)/(W): [m]hpmcounter21 - low
          when csr_hpmcounter22_c  | csr_mhpmcounter22_c  => csr.rdata <= csr.mhpmcounter_rd(19)(31 downto 0); -- (R)/(W): [m]hpmcounter22 - low
          when csr_hpmcounter23_c  | csr_mhpmcounter23_c  => csr.rdata <= csr.mhpmcounter_rd(20)(31 downto 0); -- (R)/(W): [m]hpmcounter23 - low
          when csr_hpmcounter24_c  | csr_mhpmcounter24_c  => csr.rdata <= csr.mhpmcounter_rd(21)(31 downto 0); -- (R)/(W): [m]hpmcounter24 - low
          when csr_hpmcounter25_c  | csr_mhpmcounter25_c  => csr.rdata <= csr.mhpmcounter_rd(22)(31 downto 0); -- (R)/(W): [m]hpmcounter25 - low
          when csr_hpmcounter26_c  | csr_mhpmcounter26_c  => csr.rdata <= csr.mhpmcounter_rd(23)(31 downto 0); -- (R)/(W): [m]hpmcounter26 - low
          when csr_hpmcounter27_c  | csr_mhpmcounter27_c  => csr.rdata <= csr.mhpmcounter_rd(24)(31 downto 0); -- (R)/(W): [m]hpmcounter27 - low
          when csr_hpmcounter28_c  | csr_mhpmcounter28_c  => csr.rdata <= csr.mhpmcounter_rd(25)(31 downto 0); -- (R)/(W): [m]hpmcounter28 - low
          when csr_hpmcounter29_c  | csr_mhpmcounter29_c  => csr.rdata <= csr.mhpmcounter_rd(26)(31 downto 0); -- (R)/(W): [m]hpmcounter29 - low
          when csr_hpmcounter30_c  | csr_mhpmcounter30_c  => csr.rdata <= csr.mhpmcounter_rd(27)(31 downto 0); -- (R)/(W): [m]hpmcounter30 - low
          when csr_hpmcounter31_c  | csr_mhpmcounter31_c  => csr.rdata <= csr.mhpmcounter_rd(28)(31 downto 0); -- (R)/(W): [m]hpmcounter31 - low

          when csr_hpmcounter3h_c  | csr_mhpmcounter3h_c  => csr.rdata <= csr.mhpmcounterh_rd(00)(31 downto 0); -- (R)/(W): [m]hpmcounter3h - high
          when csr_hpmcounter4h_c  | csr_mhpmcounter4h_c  => csr.rdata <= csr.mhpmcounterh_rd(01)(31 downto 0); -- (R)/(W): [m]hpmcounter4h - high
          when csr_hpmcounter5h_c  | csr_mhpmcounter5h_c  => csr.rdata <= csr.mhpmcounterh_rd(02)(31 downto 0); -- (R)/(W): [m]hpmcounter5h - high
          when csr_hpmcounter6h_c  | csr_mhpmcounter6h_c  => csr.rdata <= csr.mhpmcounterh_rd(03)(31 downto 0); -- (R)/(W): [m]hpmcounter6h - high
          when csr_hpmcounter7h_c  | csr_mhpmcounter7h_c  => csr.rdata <= csr.mhpmcounterh_rd(04)(31 downto 0); -- (R)/(W): [m]hpmcounter7h - high
          when csr_hpmcounter8h_c  | csr_mhpmcounter8h_c  => csr.rdata <= csr.mhpmcounterh_rd(05)(31 downto 0); -- (R)/(W): [m]hpmcounter8h - high
          when csr_hpmcounter9h_c  | csr_mhpmcounter9h_c  => csr.rdata <= csr.mhpmcounterh_rd(06)(31 downto 0); -- (R)/(W): [m]hpmcounter9h - high
          when csr_hpmcounter10h_c | csr_mhpmcounter10h_c => csr.rdata <= csr.mhpmcounterh_rd(07)(31 downto 0); -- (R)/(W): [m]hpmcounter10h - high
          when csr_hpmcounter11h_c | csr_mhpmcounter11h_c => csr.rdata <= csr.mhpmcounterh_rd(08)(31 downto 0); -- (R)/(W): [m]hpmcounter11h - high
          when csr_hpmcounter12h_c | csr_mhpmcounter12h_c => csr.rdata <= csr.mhpmcounterh_rd(09)(31 downto 0); -- (R)/(W): [m]hpmcounter12h - high
          when csr_hpmcounter13h_c | csr_mhpmcounter13h_c => csr.rdata <= csr.mhpmcounterh_rd(10)(31 downto 0); -- (R)/(W): [m]hpmcounter13h - high
          when csr_hpmcounter14h_c | csr_mhpmcounter14h_c => csr.rdata <= csr.mhpmcounterh_rd(11)(31 downto 0); -- (R)/(W): [m]hpmcounter14h - high
          when csr_hpmcounter15h_c | csr_mhpmcounter15h_c => csr.rdata <= csr.mhpmcounterh_rd(12)(31 downto 0); -- (R)/(W): [m]hpmcounter15h - high
          when csr_hpmcounter16h_c | csr_mhpmcounter16h_c => csr.rdata <= csr.mhpmcounterh_rd(13)(31 downto 0); -- (R)/(W): [m]hpmcounter16h - high
          when csr_hpmcounter17h_c | csr_mhpmcounter17h_c => csr.rdata <= csr.mhpmcounterh_rd(14)(31 downto 0); -- (R)/(W): [m]hpmcounter17h - high
          when csr_hpmcounter18h_c | csr_mhpmcounter18h_c => csr.rdata <= csr.mhpmcounterh_rd(15)(31 downto 0); -- (R)/(W): [m]hpmcounter18h - high
          when csr_hpmcounter19h_c | csr_mhpmcounter19h_c => csr.rdata <= csr.mhpmcounterh_rd(16)(31 downto 0); -- (R)/(W): [m]hpmcounter19h - high
          when csr_hpmcounter20h_c | csr_mhpmcounter20h_c => csr.rdata <= csr.mhpmcounterh_rd(17)(31 downto 0); -- (R)/(W): [m]hpmcounter20h - high
          when csr_hpmcounter21h_c | csr_mhpmcounter21h_c => csr.rdata <= csr.mhpmcounterh_rd(18)(31 downto 0); -- (R)/(W): [m]hpmcounter21h - high
          when csr_hpmcounter22h_c | csr_mhpmcounter22h_c => csr.rdata <= csr.mhpmcounterh_rd(19)(31 downto 0); -- (R)/(W): [m]hpmcounter22h - high
          when csr_hpmcounter23h_c | csr_mhpmcounter23h_c => csr.rdata <= csr.mhpmcounterh_rd(20)(31 downto 0); -- (R)/(W): [m]hpmcounter23h - high
          when csr_hpmcounter24h_c | csr_mhpmcounter24h_c => csr.rdata <= csr.mhpmcounterh_rd(21)(31 downto 0); -- (R)/(W): [m]hpmcounter24h - high
          when csr_hpmcounter25h_c | csr_mhpmcounter25h_c => csr.rdata <= csr.mhpmcounterh_rd(22)(31 downto 0); -- (R)/(W): [m]hpmcounter25h - high
          when csr_hpmcounter26h_c | csr_mhpmcounter26h_c => csr.rdata <= csr.mhpmcounterh_rd(23)(31 downto 0); -- (R)/(W): [m]hpmcounter26h - high
          when csr_hpmcounter27h_c | csr_mhpmcounter27h_c => csr.rdata <= csr.mhpmcounterh_rd(24)(31 downto 0); -- (R)/(W): [m]hpmcounter27h - high
          when csr_hpmcounter28h_c | csr_mhpmcounter28h_c => csr.rdata <= csr.mhpmcounterh_rd(25)(31 downto 0); -- (R)/(W): [m]hpmcounter28h - high
          when csr_hpmcounter29h_c | csr_mhpmcounter29h_c => csr.rdata <= csr.mhpmcounterh_rd(26)(31 downto 0); -- (R)/(W): [m]hpmcounter29h - high
          when csr_hpmcounter30h_c | csr_mhpmcounter30h_c => csr.rdata <= csr.mhpmcounterh_rd(27)(31 downto 0); -- (R)/(W): [m]hpmcounter30h - high
          when csr_hpmcounter31h_c | csr_mhpmcounter31h_c => csr.rdata <= csr.mhpmcounterh_rd(28)(31 downto 0); -- (R)/(W): [m]hpmcounter31h - high

          -- machine information registers --
          when csr_mvendorid_c => -- R/-: mvendorid - vendor ID
            csr.rdata <= (others => '0');
          when csr_marchid_c => -- R/-: marchid - arch ID
            csr.rdata(4 downto 0) <= "10011"; -- official RISC-V open-source arch ID
          when csr_mimpid_c => -- R/-: mimpid - implementation ID
            csr.rdata <= hw_version_c; -- NEORV32 hardware version
          when csr_mhartid_c => -- R/-: mhartid - hardware thread ID
            csr.rdata <= std_ulogic_vector(to_unsigned(HW_THREAD_ID, 32));

          -- custom machine read-only CSRs --
          when csr_mzext_c => -- R/-: mzext - available RISC-V Z* sub-extensions
            csr.rdata(0) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicsr);    -- Zicsr
            csr.rdata(1) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zifencei); -- Zifencei
            csr.rdata(2) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_B);        -- Zbb (B)
            csr.rdata(3) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_B);        -- Zbs (B)
            csr.rdata(4) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_B);        -- Zba (B)

          -- undefined/unavailable --
          when others =>
            csr.rdata <= (others => '0'); -- not implemented

        end case;
      end if;
    end if;
  end process csr_read_access;

  -- CSR read data output --
  csr_rdata_o <= csr.rdata;


end neorv32_cpu_control_rtl;
