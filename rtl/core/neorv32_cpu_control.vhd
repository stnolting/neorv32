-- #################################################################################################
-- # << NEORV32 - CPU Operations Control Unit >>                                                   #
-- # ********************************************************************************************* #
-- # CPU operations are controlled by several "engines" (modules). These engines operate in        #
-- # parallel to implement a simple pipeline:                                                      #
-- #  + Fetch engine:    Fetches 32-bit chunks of instruction words                                #
-- #  + Issue engine:    Decodes compressed instructions, aligns and queues instruction words      #
-- #  + Execute engine:  Multi-cycle execution of instructions (generate control signals)          #
-- #  + Trap controller: Handles interrupts and exceptions                                         #
-- #  + CSR module:      Read/write access to control and status registers                         #
-- #  + Debug module:    CPU debug mode handling (on-chip debugger)                                #
-- #  + Trigger module:  Hardware-assisted breakpoints (on-chip debugger)                          #
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
    CPU_EXTENSION_RISCV_Zihpm    : boolean; -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zifencei : boolean; -- implement instruction stream sync.?
    CPU_EXTENSION_RISCV_Zmmul    : boolean; -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zxcfu    : boolean; -- implement custom (instr.) functions unit?
    CPU_EXTENSION_RISCV_Zicond   : boolean; -- implement conditional operations extension?
    CPU_EXTENSION_RISCV_Sdext    : boolean; -- implement external debug mode extension?
    CPU_EXTENSION_RISCV_Sdtrig   : boolean; -- implement trigger module extension?
    -- Tuning Options --
    FAST_MUL_EN                  : boolean; -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                : boolean; -- use barrel shifter for shift operations
    CPU_IPB_ENTRIES              : natural; -- entries in instruction prefetch buffer, has to be a power of 2, min 1
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
    ctrl_o        : out ctrl_bus_t; -- main control bus
    -- instruction fetch interface --
    i_bus_addr_o  : out std_ulogic_vector(XLEN-1 downto 0); -- bus access address
    i_bus_rdata_i : in  std_ulogic_vector(31 downto 0); -- bus read data
    i_bus_re_o    : out std_ulogic; -- read enable
    i_bus_ack_i   : in  std_ulogic; -- bus transfer acknowledge
    i_bus_err_i   : in  std_ulogic; -- bus transfer error
    i_pmp_fault_i : in  std_ulogic; -- instruction fetch pmp fault
    -- status input --
    alu_cp_done_i : in  std_ulogic; -- ALU iterative operation done
    alu_exc_i     : in  std_ulogic; -- ALU exception
    bus_d_wait_i  : in  std_ulogic; -- wait for bus
    -- data input --
    cmp_i         : in  std_ulogic_vector(1 downto 0); -- comparator status
    alu_add_i     : in  std_ulogic_vector(XLEN-1 downto 0); -- ALU address result
    rs1_i         : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 1
    -- data output --
    imm_o         : out std_ulogic_vector(XLEN-1 downto 0); -- immediate
    curr_pc_o     : out std_ulogic_vector(XLEN-1 downto 0); -- current PC (corresponding to current instruction)
    next_pc_o     : out std_ulogic_vector(XLEN-1 downto 0); -- next PC (corresponding to next instruction)
    csr_rdata_o   : out std_ulogic_vector(XLEN-1 downto 0); -- CSR read data
    -- FPU interface --
    fpu_flags_i   : in  std_ulogic_vector(4 downto 0); -- exception flags
    -- debug mode (halt) request --
    db_halt_req_i : in  std_ulogic;
    -- interrupts (risc-v compliant) --
    msw_irq_i     : in  std_ulogic; -- machine software interrupt
    mext_irq_i    : in  std_ulogic; -- machine external interrupt
    mtime_irq_i   : in  std_ulogic; -- machine timer interrupt
    -- fast interrupts (custom) --
    firq_i        : in  std_ulogic_vector(15 downto 0);
    -- physical memory protection --
    pmp_addr_o    : out pmp_addr_if_t; -- addresses
    pmp_ctrl_o    : out pmp_ctrl_if_t; -- configs
    -- bus access exceptions --
    mar_i         : in  std_ulogic_vector(XLEN-1 downto 0); -- memory address register
    ma_load_i     : in  std_ulogic; -- misaligned load data address
    ma_store_i    : in  std_ulogic; -- misaligned store data address
    be_load_i     : in  std_ulogic; -- bus error on load data access
    be_store_i    : in  std_ulogic  -- bus error on store data access
  );
end neorv32_cpu_control;

architecture neorv32_cpu_control_rtl of neorv32_cpu_control is

  -- HPM counter width - high/low parts --
  constant hpm_cnt_lo_width_c : natural := natural(cond_sel_int_f(boolean(HPM_CNT_WIDTH < 32), HPM_CNT_WIDTH, 32));
  constant hpm_cnt_hi_width_c : natural := natural(cond_sel_int_f(boolean(HPM_CNT_WIDTH > 32), HPM_CNT_WIDTH-32, 0));

  -- instruction fetch engine --
  type fetch_engine_state_t is (IF_RESTART, IF_REQUEST, IF_PENDING, IF_WAIT);
  type fetch_engine_t is record
    state      : fetch_engine_state_t;
    state_prev : fetch_engine_state_t;
    restart    : std_ulogic;
    unaligned  : std_ulogic;
    pc         : std_ulogic_vector(XLEN-1 downto 0);
    reset      : std_ulogic;
    resp       : std_ulogic; -- bus response
    a_err      : std_ulogic; -- alignment error
    pmp_err    : std_ulogic; -- PMP error
  end record;
  signal fetch_engine : fetch_engine_t;

  -- instruction prefetch buffer (FIFO) interface --
  type ipb_data_t is array (0 to 1) of std_ulogic_vector((2+16)-1 downto 0); -- status (bus_error, align_error) & 16-bit instruction
  type ipb_t is record
    wdata : ipb_data_t;
    we    : std_ulogic_vector(1 downto 0); -- trigger write
    free  : std_ulogic_vector(1 downto 0); -- free entry available?
    rdata : ipb_data_t;
    re    : std_ulogic_vector(1 downto 0); -- read enable
    avail : std_ulogic_vector(1 downto 0); -- data available?
  end record;
  signal ipb : ipb_t;

  -- instruction issue engine --
  type issue_engine_t is record
    align     : std_ulogic;
    align_set : std_ulogic;
    align_clr : std_ulogic;
    ci_i16    : std_ulogic_vector(15 downto 0);
    ci_i32    : std_ulogic_vector(31 downto 0);
    data      : std_ulogic_vector((3+32)-1 downto 0); -- 3-bit status & 32-bit instruction
    valid     : std_ulogic_vector(1 downto 0); -- data word is valid when != 0
    ack       : std_ulogic;
  end record;
  signal issue_engine : issue_engine_t;

  -- instruction decoding helper logic --
  type decode_aux_t is record
    is_f_op   : std_ulogic;
    is_m_mul  : std_ulogic;
    is_m_div  : std_ulogic;
    is_b_imm  : std_ulogic;
    is_b_reg  : std_ulogic;
    is_zicond : std_ulogic;
    rs1_zero  : std_ulogic;
    rd_zero   : std_ulogic;
  end record;
  signal decode_aux : decode_aux_t;

  -- instruction execution engine --
  -- make sure reset state is the first item in the list (discussion #415)
  type execute_engine_state_t is (DISPATCH, TRAP_ENTER, TRAP_EXIT, TRAP_EXECUTE, CPU_SLEEP,
                                  EXECUTE, ALU_WAIT, BRANCH, BRANCHED, SYSTEM, MEM_REQ, MEM_WAIT);
  type execute_engine_t is record
    state        : execute_engine_state_t;
    state_nxt    : execute_engine_state_t;
    state_prev   : execute_engine_state_t;
    state_prev2  : execute_engine_state_t;
    ir           : std_ulogic_vector(31 downto 0);
    ir_nxt       : std_ulogic_vector(31 downto 0);
    is_ci        : std_ulogic; -- current instruction is de-compressed instruction
    is_ci_nxt    : std_ulogic;
    branch_taken : std_ulogic; -- branch condition fulfilled
    pc           : std_ulogic_vector(XLEN-1 downto 0); -- actual PC, corresponding to current executed instruction
    pc_mux_sel   : std_ulogic; -- source select for PC update
    pc_we        : std_ulogic; -- PC update enabled
    next_pc      : std_ulogic_vector(XLEN-1 downto 0); -- next PC, corresponding to next instruction to be executed
    next_pc_inc  : std_ulogic_vector(XLEN-1 downto 0); -- increment to get next PC
    branched     : std_ulogic; -- instruction fetch was reset
    branched_nxt : std_ulogic;
  end record;
  signal execute_engine : execute_engine_t;

  -- trap controller --
  type trap_ctrl_t is record
    exc_buf       : std_ulogic_vector(exc_width_c-1 downto 0); -- synchronous exception buffer (one bit per exception)
    exc_fire      : std_ulogic; -- set if there is a valid source in the exception buffer
    irq_pnd       : std_ulogic_vector(irq_width_c-1 downto 0); -- pending interrupt
    irq_buf       : std_ulogic_vector(irq_width_c-1 downto 0); -- asynchronous exception/interrupt buffer (one bit per interrupt source)
    irq_fire      : std_ulogic; -- set if an interrupt is actually kicking in
    cause         : std_ulogic_vector(6 downto 0); -- trap ID for mcause CSR & debug-mode entry identifier
    epc           : std_ulogic_vector(XLEN-1 downto 0); -- exception program counter
    --
    env_start     : std_ulogic; -- start trap handler env
    env_start_ack : std_ulogic; -- start of trap handler acknowledged
    env_end       : std_ulogic; -- end trap handler env
    wakeup        : std_ulogic; -- wakeup from sleep due to an enabled pending IRQ
    --
    instr_be      : std_ulogic; -- instruction fetch bus error
    instr_ma      : std_ulogic; -- instruction fetch misaligned address
    instr_il      : std_ulogic; -- illegal instruction
    env_call      : std_ulogic; -- ecall instruction
    break_point   : std_ulogic; -- ebreak instruction
  end record;
  signal trap_ctrl : trap_ctrl_t;

  -- CPU main control bus --
  signal ctrl, ctrl_nxt : ctrl_bus_t;

  -- RISC-V control and status registers (CSRs) --
  type csr_t is record
    addr          : std_ulogic_vector(11 downto 0); -- csr address
    we            : std_ulogic; -- csr write enable
    we_nxt        : std_ulogic;
    re            : std_ulogic; -- csr read enable
    re_nxt        : std_ulogic;
    wdata         : std_ulogic_vector(XLEN-1 downto 0); -- csr write data
    rdata         : std_ulogic_vector(XLEN-1 downto 0); -- csr read data
    --
    mstatus_mie   : std_ulogic; -- global IRQ enable
    mstatus_mpie  : std_ulogic; -- previous global IRQ enable
    mstatus_mpp   : std_ulogic; -- machine previous privilege mode
    mstatus_mprv  : std_ulogic; -- effective privilege level for machine-mode load/stores
    mstatus_tw    : std_ulogic; -- do not allow user mode to execute WFI instruction when set
    --
    mie_msi       : std_ulogic; -- machine software interrupt enable
    mie_mei       : std_ulogic; -- machine external interrupt enable
    mie_mti       : std_ulogic; -- machine timer interrupt enable
    mie_firq      : std_ulogic_vector(15 downto 0); -- fast interrupt enable
    mip_firq_nclr : std_ulogic_vector(15 downto 0); -- clear pending FIRQ (active-low)
    --
    privilege     : std_ulogic; -- current privilege mode
    privilege_eff : std_ulogic; -- current *effective* privilege mode
    --
    mepc          : std_ulogic_vector(XLEN-1 downto 0); -- machine exception PC
    mcause        : std_ulogic_vector(5 downto 0); -- machine trap cause
    mtvec         : std_ulogic_vector(XLEN-1 downto 0); -- machine trap-handler base address
    mtval         : std_ulogic_vector(XLEN-1 downto 0); -- machine bad address or instruction
    mscratch      : std_ulogic_vector(XLEN-1 downto 0); -- machine scratch register
    mcounteren    : std_ulogic_vector(XLEN-1 downto 0); -- machine counter access enable
    mcountinhibit : std_ulogic_vector(XLEN-1 downto 0); -- inhibit counter auto-increment
    --
    frm           : std_ulogic_vector(2 downto 0); -- FPU rounding mode
    fflags        : std_ulogic_vector(4 downto 0); -- FPU exception flags
    --
    dcsr_ebreakm  : std_ulogic; -- behavior of ebreak instruction in m-mode
    dcsr_ebreaku  : std_ulogic; -- behavior of ebreak instruction in u-mode
    dcsr_step     : std_ulogic; -- single-step mode
    dcsr_prv      : std_ulogic; -- current privilege level when entering debug mode
    dcsr_cause    : std_ulogic_vector(2 downto 0); -- why was debug mode entered
    dcsr_rd       : std_ulogic_vector(XLEN-1 downto 0); -- debug mode control and status register
    dpc           : std_ulogic_vector(XLEN-1 downto 0); -- mode program counter
    dscratch0     : std_ulogic_vector(XLEN-1 downto 0); -- debug mode scratch register 0
    --
    tdata1_exe    : std_ulogic; -- enable (match) trigger
    tdata1_action : std_ulogic; -- enter debug mode / ebreak exception when trigger fires
    tdata1_dmode  : std_ulogic; -- set to ignore tdata* CSR access from machine-mode
    tdata1_rd     : std_ulogic_vector(XLEN-1 downto 0); -- trigger register read-back
    tdata2        : std_ulogic_vector(XLEN-1 downto 0); -- address-match register
  end record;
  signal csr : csr_t;

  -- hpm event configuration CSRs (first 3 entries are just dummies) --
  type hpmevent_cfg_t is array (0 to HPM_NUM_CNTS-1) of std_ulogic_vector(hpmcnt_event_size_c-1 downto 0);
  type hpmevent_rd_t  is array (0 to 31) of std_ulogic_vector(XLEN-1 downto 0);
  type hpmevent_t is record
    we  : std_ulogic_vector(31 downto 0);
    cfg : hpmevent_cfg_t;
  end record;
  signal hpmevent    : hpmevent_t;
  signal hpmevent_rd : hpmevent_rd_t;

  -- physical memory protection CSRs --
  type pmp_cfg_t     is array (0 to PMP_NUM_REGIONS-1) of std_ulogic_vector(7 downto 0);
  type pmp_addr_t    is array (0 to PMP_NUM_REGIONS-1) of std_ulogic_vector(XLEN-1 downto 0);
  type pmp_cfg_rd_t  is array (0 to 03) of std_ulogic_vector(XLEN-1 downto 0);
  type pmp_addr_rd_t is array (0 to 15) of std_ulogic_vector(XLEN-1 downto 0);
  type pmp_t is record
    we_cfg  : std_ulogic_vector(03 downto 0);
    we_addr : std_ulogic_vector(15 downto 0);
    cfg     : pmp_cfg_t;
    addr    : pmp_addr_t;
  end record;
  signal pmp         : pmp_t;
  signal pmp_cfg_rd  : pmp_cfg_rd_t;
  signal pmp_addr_rd : pmp_addr_rd_t;

  -- counter CSRs --
  type cnt_dat_t is array (0 to 31) of std_ulogic_vector(XLEN-1 downto 0);
  type cnt_nxt_t is array (0 to 31) of std_ulogic_vector(XLEN downto 0);
  type cnt_ovf_t is array (0 to 31) of std_ulogic_vector(0 downto 0);
  type cnt_t is record
    we_lo : std_ulogic_vector(31 downto 0);
    we_hi : std_ulogic_vector(31 downto 0);
    inc   : std_ulogic_vector(31 downto 0);
    lo    : cnt_dat_t; -- counter word low
    hi    : cnt_dat_t; -- counter word high
    nxt   : cnt_nxt_t; -- increment, including carry bit
    ovf   : cnt_ovf_t; -- counter low-to-high-word overflow
  end record;
  signal cnt       : cnt_t;
  signal cnt_lo_rd : cnt_dat_t;
  signal cnt_hi_rd : cnt_dat_t;

  -- counter events --
  signal cnt_event : std_ulogic_vector(hpmcnt_event_size_c-1 downto 0);

  -- debug mode controller --
  type debug_ctrl_state_t is (DEBUG_OFFLINE, DEBUG_ONLINE, DEBUG_LEAVING);
  type debug_ctrl_t is record
    state        : debug_ctrl_state_t;
    running      : std_ulogic; -- CPU is in debug mode
    trig_hw      : std_ulogic; -- hardware trigger
    trig_break   : std_ulogic; -- ebreak instruction trigger
    trig_halt    : std_ulogic; -- external request trigger
    trig_step    : std_ulogic; -- single-stepping mode trigger
    dret         : std_ulogic; -- executed DRET instruction
    ext_halt_req : std_ulogic; -- external halt request buffer
  end record;
  signal debug_ctrl : debug_ctrl_t;

  -- illegal instruction check --
  signal illegal_cmd : std_ulogic;

  -- CSR access/privilege and r/w check --
  signal csr_reg_valid  : std_ulogic; -- valid CSR access (implemented at all)
  signal csr_rw_valid   : std_ulogic; -- valid CSR access (valid r/w access rights)
  signal csr_priv_valid : std_ulogic; -- valid CSR access (valid access privilege)

  -- hardware trigger module --
  signal hw_trigger_fire : std_ulogic;

  -- misc --
  signal imm_opcode : std_ulogic_vector(06 downto 0); -- simplified opcode for immediate generator
  signal csr_raddr  : std_ulogic_vector(11 downto 0); -- CSR read address (AND-gated)

begin

-- ****************************************************************************************************************************
-- Instruction Fetch (always fetch 32-bit-aligned 32-bit chunks of data)
-- ****************************************************************************************************************************

  -- Fetch Engine FSM -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  fetch_engine_fsm: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      fetch_engine.state      <= IF_RESTART;
      fetch_engine.state_prev <= IF_RESTART;
      fetch_engine.restart    <= '1'; -- set to reset IPB
      fetch_engine.unaligned  <= '0'; -- always start at aligned address after reset
      fetch_engine.pc         <= (others => '0');
      fetch_engine.pmp_err    <= '0';
    elsif rising_edge(clk_i) then
      -- previous state (for HPM) --
      fetch_engine.state_prev <= fetch_engine.state;

      -- restart request buffer --
      if (fetch_engine.state = IF_RESTART) then -- restart done
        fetch_engine.restart <= '0';
      else -- buffer request
        fetch_engine.restart <= fetch_engine.restart or fetch_engine.reset;
      end if;

      -- register PMP fault --
      fetch_engine.pmp_err <= i_pmp_fault_i;

      -- fsm --
      case fetch_engine.state is

        when IF_RESTART => -- set new fetch start address
        -- ------------------------------------------------------------
          fetch_engine.pc        <= execute_engine.pc(XLEN-1 downto 2) & "00"; -- initialize with "real" PC, 32-bit aligned
          fetch_engine.unaligned <= execute_engine.pc(1);
          fetch_engine.state     <= IF_REQUEST;

        when IF_REQUEST => -- request new 32-bit-aligned instruction word
        -- ------------------------------------------------------------
          if (ipb.free = "11") then -- wait for free IPB space
            fetch_engine.state <= IF_PENDING;
          end if;

        when IF_PENDING => -- wait for bus response and write instruction data to prefetch buffer
        -- ------------------------------------------------------------
          if (fetch_engine.resp = '1') then -- wait for bus response
            fetch_engine.pc        <= std_ulogic_vector(unsigned(fetch_engine.pc) + 4);
            fetch_engine.unaligned <= '0';
            if (fetch_engine.restart = '1') or (fetch_engine.reset = '1') then -- restart request (fast)
              fetch_engine.state <= IF_RESTART;
            -- do not trigger new instruction fetch when a branch instruction is being executed (wait for branch destination)
            elsif ((execute_engine.ir(instr_opcode_msb_c downto instr_opcode_lsb_c+2) = opcode_branch_c(6 downto 2)) and
                   (execute_engine.ir(31) = '1')) or -- predict: taken if branching backwards
                  (execute_engine.ir(instr_opcode_msb_c downto instr_opcode_lsb_c+2) = opcode_jal_c(6 downto 2)) or    -- always taken
                  (execute_engine.ir(instr_opcode_msb_c downto instr_opcode_lsb_c+2) = opcode_jalr_c(6 downto 2)) then -- always taken
              fetch_engine.state <= IF_WAIT;
            else -- request next instruction word
              fetch_engine.state <= IF_REQUEST;
            end if;
          end if;

        when IF_WAIT => -- wait for branch instruction
        -- ------------------------------------------------------------
          if (fetch_engine.restart = '1') or (fetch_engine.reset = '1') then -- restart request (fast) if taken branch
            fetch_engine.state <= IF_RESTART;
          else
            fetch_engine.state <= IF_REQUEST;
          end if;

        when others => -- undefined
        -- ------------------------------------------------------------
          fetch_engine.state <= IF_RESTART;

      end case;
    end if;
  end process fetch_engine_fsm;

  -- PC output for instruction fetch --
  i_bus_addr_o <= fetch_engine.pc(XLEN-1 downto 2) & "00"; -- 32-bit aligned

  -- instruction fetch (read) request if IPB not full --
  i_bus_re_o <= '1' when (fetch_engine.state = IF_REQUEST) and (ipb.free = "11") else '0';

  -- unaligned access error (no alignment exceptions possible when using C-extension) --
  fetch_engine.a_err <= '1' when (fetch_engine.unaligned = '1') and (CPU_EXTENSION_RISCV_C = false) else '0';

  -- instruction bus response --
  -- [NOTE] PMP and alignment-error will keep pending until the actually triggered bus access completes (or fails)
  fetch_engine.resp <= '1' when (i_bus_ack_i = '1') or (i_bus_err_i = '1') else '0';

  -- IPB instruction data and status --
  ipb.wdata(0) <= (i_bus_err_i or fetch_engine.pmp_err) & fetch_engine.a_err & i_bus_rdata_i(15 downto 00);
  ipb.wdata(1) <= (i_bus_err_i or fetch_engine.pmp_err) & fetch_engine.a_err & i_bus_rdata_i(31 downto 16);

  -- IPB write enable --
  ipb.we(0) <= '1' when (fetch_engine.state = IF_PENDING) and (fetch_engine.resp = '1') and
                        ((fetch_engine.unaligned = '0') or (CPU_EXTENSION_RISCV_C = false)) else '0';
  ipb.we(1) <= '1' when (fetch_engine.state = IF_PENDING) and (fetch_engine.resp = '1') else '0';


  -- Instruction Prefetch Buffer (FIFO) -----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  prefetch_buffer:
  for i in 0 to 1 generate -- low half-word and high half-word (+status)
    prefetch_buffer_inst: neorv32_fifo
    generic map (
      FIFO_DEPTH => CPU_IPB_ENTRIES,     -- number of fifo entries; has to be a power of two; min 1
      FIFO_WIDTH => ipb.wdata(i)'length, -- size of data elements in fifo
      FIFO_RSYNC => false,               -- we NEED to read data asynchronously
      FIFO_SAFE  => false                -- no safe access required (ensured by FIFO-external control)
    )
    port map (
      -- control --
      clk_i   => clk_i,                -- clock, rising edge
      rstn_i  => rstn_i,               -- async reset, low-active
      clear_i => fetch_engine.restart, -- sync reset, high-active
      half_o  => open,                 -- at least half full
      -- write port --
      wdata_i => ipb.wdata(i),         -- write data
      we_i    => ipb.we(i),            -- write enable
      free_o  => ipb.free(i),          -- at least one entry is free when set
      -- read port --
      re_i    => ipb.re(i),            -- read enable
      rdata_o => ipb.rdata(i),         -- read data
      avail_o => ipb.avail(i)          -- data available when set
    );
  end generate;


-- ****************************************************************************************************************************
-- Instruction Issue (decompress 16-bit instructions and assemble a 32-bit instruction word)
-- ****************************************************************************************************************************

  -- Issue Engine FSM (required only if C extension is enabled) -----------------------------
  -- -------------------------------------------------------------------------------------------
  issue_engine_enabled:
  if (CPU_EXTENSION_RISCV_C = true) generate

    issue_engine_fsm_sync: process(clk_i)
    begin
      if rising_edge(clk_i) then
        if (fetch_engine.restart = '1') then
          issue_engine.align <= execute_engine.pc(1); -- branch to unaligned address?
        elsif (execute_engine.state = DISPATCH) then
          issue_engine.align <= (issue_engine.align and (not issue_engine.align_clr)) or issue_engine.align_set; -- "RS" flip-flop
        end if;
      end if;
    end process issue_engine_fsm_sync;

    issue_engine_fsm_comb: process(issue_engine, ipb)
    begin
      -- defaults --
      issue_engine.align_set <= '0';
      issue_engine.align_clr <= '0';
      issue_engine.valid     <= "00";

      -- start with LOW half-word --
      if (issue_engine.align = '0')  then
        if (ipb.rdata(0)(1 downto 0) /= "11") then -- compressed
          issue_engine.align_set <= ipb.avail(0); -- start of next instruction word is NOT 32-bit-aligned
          issue_engine.valid(0)  <= ipb.avail(0);
          issue_engine.data      <= ipb.rdata(0)(17 downto 16) & '1' & issue_engine.ci_i32;
        else -- aligned uncompressed
          issue_engine.valid <= (others => (ipb.avail(0) and ipb.avail(1)));
          issue_engine.data  <= (ipb.rdata(1)(17 downto 16) or ipb.rdata(0)(17 downto 16)) & '0' &
                                (ipb.rdata(1)(15 downto 00)  & ipb.rdata(0)(15 downto 00));
        end if;
      -- start with HIGH half-word --
      else
        if (ipb.rdata(1)(1 downto 0) /= "11") then -- compressed
          issue_engine.align_clr <= ipb.avail(1); -- start of next instruction word IS 32-bit-aligned again
          issue_engine.valid(1)  <= ipb.avail(1);
          issue_engine.data      <= ipb.rdata(1)(17 downto 16) & '1' & issue_engine.ci_i32;
        else -- unaligned uncompressed
          issue_engine.valid <= (others => (ipb.avail(0) and ipb.avail(1)));
          issue_engine.data  <= (ipb.rdata(0)(17 downto 16) or ipb.rdata(1)(17 downto 16)) & '0' &
                                (ipb.rdata(0)(15 downto 00)  & ipb.rdata(1)(15 downto 00));
        end if;
      end if;
    end process issue_engine_fsm_comb;

  end generate; -- /issue_engine_enabled

  issue_engine_disabled:
  if (CPU_EXTENSION_RISCV_C = false) generate

    issue_engine.valid <= (others => ipb.avail(0)); -- only use status flags from IPB[0]
    issue_engine.data  <= ipb.rdata(0)(17 downto 16) & '0' & (ipb.rdata(1)(15 downto 0) & ipb.rdata(0)(15 downto 0));

  end generate; -- /issue_engine_disabled

  -- update IPB FIFOs (ready-for-next)? --
  ipb.re(0) <= '1' when (issue_engine.valid(0) = '1') and (issue_engine.ack = '1') else '0';
  ipb.re(1) <= '1' when (issue_engine.valid(1) = '1') and (issue_engine.ack = '1') else '0';


  -- Compressed Instructions Decoding -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_decompressor_inst_true:
  if (CPU_EXTENSION_RISCV_C = true) generate
    neorv32_cpu_decompressor_inst: neorv32_cpu_decompressor
    generic map (
      FPU_ENABLE => CPU_EXTENSION_RISCV_Zfinx -- floating-point instructions enabled
    )
    port map (
      ci_instr16_i => issue_engine.ci_i16, -- compressed instruction
      ci_instr32_o => issue_engine.ci_i32  -- decompressed instruction
    );
  end generate;

  neorv32_cpu_decompressor_inst_false:
  if (CPU_EXTENSION_RISCV_C = false) generate
    issue_engine.ci_i32 <= (others => '0');
  end generate;

  -- 16-bit instructions: half-word select --
  issue_engine.ci_i16 <= ipb.rdata(0)(15 downto 0) when (issue_engine.align = '0') else ipb.rdata(1)(15 downto 0);


-- ****************************************************************************************************************************
-- Instruction Execution
-- ****************************************************************************************************************************

  -- Immediate Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  imm_gen: process(clk_i)
  begin
    if rising_edge(clk_i) then
      case imm_opcode is
        when opcode_store_c => -- S-immediate: store
          imm_o(XLEN-1 downto 11) <= (others => execute_engine.ir(31)); -- sign extension
          imm_o(10 downto 05)     <= execute_engine.ir(30 downto 25);
          imm_o(04 downto 00)     <= execute_engine.ir(11 downto 07);
        when opcode_branch_c => -- B-immediate: conditional branches
          imm_o(XLEN-1 downto 12) <= (others => execute_engine.ir(31)); -- sign extension
          imm_o(11)               <= execute_engine.ir(07);
          imm_o(10 downto 05)     <= execute_engine.ir(30 downto 25);
          imm_o(04 downto 01)     <= execute_engine.ir(11 downto 08);
          imm_o(00)               <= '0';
        when opcode_lui_c | opcode_auipc_c => -- U-immediate: lui, auipc
          imm_o(XLEN-1 downto 12) <= execute_engine.ir(31 downto 12);
          imm_o(11 downto 00)     <= (others => '0');
        when opcode_jal_c => -- J-immediate: unconditional jumps
          imm_o(XLEN-1 downto 20) <= (others => execute_engine.ir(31)); -- sign extension
          imm_o(19 downto 12)     <= execute_engine.ir(19 downto 12);
          imm_o(11)               <= execute_engine.ir(20);
          imm_o(10 downto 01)     <= execute_engine.ir(30 downto 21);
          imm_o(00)               <= '0';
        when others => -- I-immediate: ALU-immediate, loads, jump-and-link with register
          imm_o(XLEN-1 downto 11) <= (others => execute_engine.ir(31)); -- sign extension
          imm_o(10 downto 01)     <= execute_engine.ir(30 downto 21);
          imm_o(00)               <= execute_engine.ir(20);
      end case;
    end if;
  end process imm_gen;

  -- save some bits here - the two LSBs are always "11" for 32-bit instructions --
  imm_opcode <= execute_engine.ir(instr_opcode_msb_c downto instr_opcode_lsb_c+2) & "11";


  -- Branch Condition Check -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  branch_check: process(execute_engine.ir, cmp_i)
  begin -- this is hacky!
    if (execute_engine.ir(instr_funct3_msb_c) = '0') then -- beq / bne
      execute_engine.branch_taken <= cmp_i(cmp_equal_c) xor execute_engine.ir(instr_funct3_lsb_c);
    else -- blt(u) / bge(u)
      execute_engine.branch_taken <= cmp_i(cmp_less_c)  xor execute_engine.ir(instr_funct3_lsb_c);
    end if;
  end process branch_check;


  -- Execute Engine FSM Sync ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  execute_engine_fsm_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl                       <= ctrl_bus_zero_c;
      execute_engine.state       <= BRANCHED; -- reset is a branch from "somewhere"
      execute_engine.state_prev  <= BRANCHED;
      execute_engine.state_prev2 <= BRANCHED;
      execute_engine.branched    <= '0';
      execute_engine.ir          <= (others => '0');
      execute_engine.is_ci       <= '0';
      execute_engine.pc          <= CPU_BOOT_ADDR(XLEN-1 downto 2) & "00"; -- 32-bit aligned boot address
      execute_engine.next_pc     <= (others => '0');
    elsif rising_edge(clk_i) then
      -- control bus --
      ctrl <= ctrl_nxt;

      -- execute engine arbiter --
      execute_engine.state       <= execute_engine.state_nxt;
      execute_engine.state_prev  <= execute_engine.state; -- for HPMs only
      execute_engine.state_prev2 <= execute_engine.state_prev; -- for HPMs only
      execute_engine.branched    <= execute_engine.branched_nxt;
      execute_engine.ir          <= execute_engine.ir_nxt;
      execute_engine.is_ci       <= execute_engine.is_ci_nxt;

      -- PC update --
      if (execute_engine.pc_we = '1') then
        if (execute_engine.pc_mux_sel = '0') then
          execute_engine.pc <= execute_engine.next_pc(XLEN-1 downto 1) & '0'; -- next instruction address
        else
          execute_engine.pc <= alu_add_i(XLEN-1 downto 1) & '0'; -- jump/taken-branch
        end if;
      end if;

      -- next PC logic --
      case execute_engine.state is
        when TRAP_ENTER => -- starting trap environment
          if (trap_ctrl.cause(5) = '1') and (CPU_EXTENSION_RISCV_Sdext = true) then -- trap cause: debug mode (re-)entry
            execute_engine.next_pc <= CPU_DEBUG_PARK_ADDR; -- debug mode enter; start at "parking loop" <normal_entry>
          elsif (debug_ctrl.running = '1') and (CPU_EXTENSION_RISCV_Sdext = true) then -- any other exception INSIDE debug mode
            execute_engine.next_pc <= CPU_DEBUG_EXC_ADDR; -- debug mode enter: start at "parking loop" <exception_entry>
          else -- normal start of trap
            execute_engine.next_pc <= csr.mtvec(XLEN-1 downto 2) & "00"; -- trap enter
          end if;
        when TRAP_EXIT => -- leaving trap environment
          if (debug_ctrl.running = '1') and (CPU_EXTENSION_RISCV_Sdext = true) then -- debug mode exit
            execute_engine.next_pc <= csr.dpc(XLEN-1 downto 1) & '0'; -- debug mode exit
          else -- normal end of trap
            execute_engine.next_pc <= csr.mepc(XLEN-1 downto 1) & '0'; -- trap exit
          end if;
        when EXECUTE => -- normal increment
          execute_engine.next_pc <= std_ulogic_vector(unsigned(execute_engine.pc) + unsigned(execute_engine.next_pc_inc)); -- next linear PC
        when BRANCHED => -- control flow transfer
          execute_engine.next_pc <= execute_engine.pc(XLEN-1 downto 1) & '0'; -- get updated PC
        when others =>
          NULL;
      end case;
    end if;
  end process execute_engine_fsm_sync;

  -- PC increment for next linear instruction (+2 for compressed instr., +4 otherwise) --
  execute_engine.next_pc_inc(XLEN-1 downto 4) <= (others => '0');
  execute_engine.next_pc_inc(3 downto 0) <= x"4" when ((execute_engine.is_ci = '0') or (CPU_EXTENSION_RISCV_C = false)) else x"2";

  -- PC output --
  curr_pc_o <= execute_engine.pc(XLEN-1 downto 1) & '0'; -- current PC
  next_pc_o <= execute_engine.next_pc(XLEN-1 downto 1) & '0'; -- next PC


  -- Decoding Helper Logic ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  decode_helper: process(execute_engine)
  begin
    -- defaults --
    decode_aux.is_f_op   <= '0';
    decode_aux.is_m_mul  <= '0';
    decode_aux.is_m_div  <= '0';
    decode_aux.is_b_imm  <= '0';
    decode_aux.is_b_reg  <= '0';
    decode_aux.is_zicond <= '0';
    decode_aux.rs1_zero  <= '0';
    decode_aux.rd_zero   <= '0';

    -- is BITMANIP instruction? --
    -- pretty complex as we have to check the already-crowded ALU/ALUI instruction space --
    if (CPU_EXTENSION_RISCV_B = true) then -- BITMANIP implemented at all?
      -- register-immediate operation --
      if ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0110000") and (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001") and (
           (execute_engine.ir(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c) = "00000") or -- CLZ
           (execute_engine.ir(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c) = "00001") or -- CTZ
           (execute_engine.ir(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c) = "00010") or -- CPOP
           (execute_engine.ir(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c) = "00100") or -- SEXT.B
           (execute_engine.ir(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c) = "00101")    -- SEXT.H
          )) or
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0110000") and (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = "101")) or -- RORI
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0010100") and (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = "101") and
                                                                                               (execute_engine.ir(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c) = "00111")) or -- ORCB
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0100100") and (execute_engine.ir(instr_funct3_msb_c-1 downto instr_funct3_lsb_c) = "01")) or -- BCLRI / BEXTI
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0110100") and (execute_engine.ir(instr_funct3_msb_c-1 downto instr_funct3_lsb_c) = "01")) or -- REV8 / BINVI
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0010100") and (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001")) then -- BSETI
        decode_aux.is_b_imm <= '1';
      end if;
      -- register-register operation --
      if ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0110000") and (execute_engine.ir(instr_funct3_msb_c-1 downto instr_funct3_lsb_c) = "01")) or -- ROR / ROL
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000101") and (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) /= "000")) or -- MIN[U] / MAX[U] / CMUL[H/R]
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000100") and (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = "100")) or -- ZEXTH
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0100100") and (execute_engine.ir(instr_funct3_msb_c-1 downto instr_funct3_lsb_c) = "01")) or -- BCLR / BEXT
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0110100") and (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001")) or -- BINV
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0010100") and (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001")) or -- BSET
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0100000") and (
           (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = "111") or -- ANDN
           (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = "110") or -- ORN
           (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = "100")    -- XORN
          )) or
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0010000") and (
           (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = "010") or -- SH1ADD
           (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = "100") or -- SH2ADD
           (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = "110")    -- SH3ADD
          )
         ) then
        decode_aux.is_b_reg <= '1';
      end if;
    end if;

    -- floating-point operations (Zfinx) --
    if (CPU_EXTENSION_RISCV_Zfinx = true) then -- FPU implemented at all?
      if ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c+3) = "0000")) or -- FADD.S / FSUB.S
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "00010")) or -- FMUL.S
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "11100") and (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = "001")) or -- FCLASS.S
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "00100") and (execute_engine.ir(instr_funct3_msb_c) = '0')) or -- FSGNJ[N/X].S
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "00101") and (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_msb_c-1) = "00")) or -- FMIN.S / FMAX.S
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "10100") and (execute_engine.ir(instr_funct3_msb_c) = '0')) or -- FEQ.S / FLT.S / FLE.S
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "11010") and (execute_engine.ir(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c+1) = "0000")) or -- FCVT.S.W*
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c+2) = "11000") and (execute_engine.ir(instr_funct12_lsb_c+4 downto instr_funct12_lsb_c+1) = "0000")) then -- FCVT.W*.S
        if (execute_engine.ir(instr_funct7_lsb_c+1 downto instr_funct7_lsb_c) = float_single_c) then -- single-precision operations only
          decode_aux.is_f_op <= '1';
        end if;
      end if;
    end if;

    -- integer MUL (M/Zmmul) / DIV (M) operation --
    if (execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000001") then
      if ((CPU_EXTENSION_RISCV_M = true) or (CPU_EXTENSION_RISCV_Zmmul = true)) and (execute_engine.ir(instr_funct3_msb_c) = '0') then
        decode_aux.is_m_mul <= '1';
      end if;
      if (CPU_EXTENSION_RISCV_M = true) and (execute_engine.ir(instr_funct3_msb_c) = '1') then
        decode_aux.is_m_div <= '1';
      end if;
    end if;

    -- conditional operations (Zicond) --
    if (CPU_EXTENSION_RISCV_Zicond = true) and (execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000111") and
       (execute_engine.ir(instr_funct3_msb_c) = '1') and (execute_engine.ir(instr_funct3_lsb_c) = '1') then
      decode_aux.is_zicond <= '1';
    end if;

    -- register/uimm5 checks --
    if (execute_engine.ir(instr_rs1_msb_c downto instr_rs1_lsb_c) = "00000") then
      decode_aux.rs1_zero <= '1';
    end if;
    if (execute_engine.ir(instr_rd_msb_c downto instr_rd_lsb_c) = "00000") then
      decode_aux.rd_zero <= '1';
    end if;
  end process decode_helper;

  -- CSR access address --
  csr.addr <= execute_engine.ir(instr_imm12_msb_c downto instr_imm12_lsb_c);


  -- Execute Engine FSM Comb ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  execute_engine_fsm_comb: process(execute_engine, debug_ctrl, trap_ctrl, decode_aux, fetch_engine, issue_engine, csr, alu_cp_done_i, bus_d_wait_i)
  begin
    -- arbiter defaults --
    execute_engine.state_nxt    <= execute_engine.state;
    execute_engine.ir_nxt       <= execute_engine.ir;
    execute_engine.is_ci_nxt    <= execute_engine.is_ci;
    execute_engine.branched_nxt <= execute_engine.branched;
    execute_engine.pc_mux_sel   <= '0';
    execute_engine.pc_we        <= '0';

    -- issue engine defaults --
    issue_engine.ack <= '0';

    -- instruction dispatch defaults --
    fetch_engine.reset <= '0';

    -- trap environment control defaults --
    trap_ctrl.env_start_ack <= '0';
    trap_ctrl.env_end       <= '0';
    trap_ctrl.instr_be      <= '0';
    trap_ctrl.instr_ma      <= '0';
    trap_ctrl.env_call      <= '0';
    trap_ctrl.break_point   <= '0';
    debug_ctrl.dret         <= '0';

    -- CSR access defaults --
    csr.we_nxt <= '0';
    csr.re_nxt <= '0';

    -- Control defaults --
    ctrl_nxt        <= ctrl_bus_zero_c; -- all zero/off by default
    ctrl_nxt.alu_op <= alu_op_add_c; -- default ALU operation: ADD
    ctrl_nxt.rf_mux <= rf_mux_alu_c; -- default RF input: ALU
    -- ALU sign control --
    if (execute_engine.ir(instr_opcode_lsb_c+4) = '1') then -- ALU ops
      ctrl_nxt.alu_unsigned <= execute_engine.ir(instr_funct3_lsb_c+0); -- unsigned ALU operation? (SLTIU, SLTU)
    else -- branches
      ctrl_nxt.alu_unsigned <= execute_engine.ir(instr_funct3_lsb_c+1); -- unsigned branches? (BLTU, BGEU)
    end if;

    -- state machine --
    case execute_engine.state is

      when DISPATCH => -- Wait for ISSUE engine to become ready
      -- ------------------------------------------------------------
        if (issue_engine.valid(0) = '1') or (issue_engine.valid(1) = '1') then -- new instruction available / IFETCH and ISSUE ready again?
          if (trap_ctrl.env_start = '1') or (trap_ctrl.exc_fire = '1') then -- pending trap
            execute_engine.state_nxt <= TRAP_ENTER;
          else -- normal execution
            issue_engine.ack          <= '1';
            trap_ctrl.instr_be        <= issue_engine.data(34); -- bus access fault during instruction fetch
            trap_ctrl.instr_ma        <= issue_engine.data(33) and (not bool_to_ulogic_f(CPU_EXTENSION_RISCV_C)); -- misaligned instruction fetch (if C disabled)
            execute_engine.is_ci_nxt  <= issue_engine.data(32); -- this is a de-compressed instruction
            execute_engine.ir_nxt     <= issue_engine.data(31 downto 0);
            execute_engine.pc_we      <= not execute_engine.branched; -- update PC with next_pc if there was no actual branch
            execute_engine.state_nxt  <= EXECUTE;
          end if;
        end if;


      when TRAP_ENTER => -- Start trap environment and get trap vector
      -- ------------------------------------------------------------
        if (trap_ctrl.env_start = '1') then
          trap_ctrl.env_start_ack  <= '1';
          execute_engine.state_nxt <= TRAP_EXECUTE;
        end if;


      when TRAP_EXIT => -- Return from trap environment and get xEPC
      -- ------------------------------------------------------------
        trap_ctrl.env_end        <= '1';
        execute_engine.state_nxt <= TRAP_EXECUTE;


      when TRAP_EXECUTE => -- Process trap environment
      -- ------------------------------------------------------------
        execute_engine.pc_mux_sel <= '0'; -- next_PC (xEPC or trap vector)
        fetch_engine.reset        <= '1';
        execute_engine.pc_we      <= '1';
        execute_engine.state_nxt  <= BRANCHED;


      when EXECUTE => -- Decode and execute instruction (control has to be here for exactly 1 cycle in any case!)
      -- [NOTE] register file is read in this stage; due to the sync read, data will be available in the _next_ state
      -- ------------------------------------------------------------
        -- clear branch flipflop --
        execute_engine.branched_nxt <= '0';

        -- decode instruction class --
        case execute_engine.ir(instr_opcode_msb_c downto instr_opcode_lsb_c) is

          when opcode_alu_c | opcode_alui_c => -- register/immediate ALU operation
          -- ------------------------------------------------------------
            -- register-immediate ALU operation --
            if (execute_engine.ir(instr_opcode_msb_c-1) = '0') then
              ctrl_nxt.alu_opb_mux <= '1'; -- use IMM as ALU.OPB
            end if;

            -- ALU core operation --
            case execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) is -- actual ALU.logic operation (re-coding)
              when funct3_subadd_c => -- ADD(I), SUB
                if ((execute_engine.ir(instr_opcode_msb_c-1) = '1') and (execute_engine.ir(instr_funct7_msb_c-1) = '1')) then
                  ctrl_nxt.alu_op <= alu_op_sub_c; -- SUB if not an immediate op and funct7.6 set
                else
                  ctrl_nxt.alu_op <= alu_op_add_c;
                end if;
              when funct3_slt_c | funct3_sltu_c => -- SLT(I), SLTU(I)
                ctrl_nxt.alu_op <= alu_op_slt_c;
              when funct3_xor_c => -- XOR(I)
                ctrl_nxt.alu_op <= alu_op_xor_c;
              when funct3_or_c => -- OR(I)
                ctrl_nxt.alu_op <= alu_op_or_c;
              when others => -- AND(I) or multi-cycle / co-processor operation
                ctrl_nxt.alu_op <= alu_op_and_c;
            end case;

            -- EXT: co-processor MULDIV operation (multi-cycle) --
            if ((CPU_EXTENSION_RISCV_M = true) and (execute_engine.ir(instr_opcode_lsb_c+5) = opcode_alu_c(5)) and
                ((decode_aux.is_m_mul = '1') or (decode_aux.is_m_div = '1'))) or -- MUL/DIV
               ((CPU_EXTENSION_RISCV_Zmmul = true) and (execute_engine.ir(instr_opcode_lsb_c+5) = opcode_alu_c(5)) and
                (decode_aux.is_m_mul = '1')) then -- MUL
              ctrl_nxt.alu_cp_trig(cp_sel_muldiv_c) <= '1'; -- trigger MULDIV CP
              execute_engine.state_nxt              <= ALU_WAIT;
            -- EXT: co-processor BIT-MANIPULATION operation (multi-cycle) --
            elsif (CPU_EXTENSION_RISCV_B = true) and
                  (((execute_engine.ir(instr_opcode_lsb_c+5) = opcode_alu_c(5))  and (decode_aux.is_b_reg = '1')) or -- register operation
                   ((execute_engine.ir(instr_opcode_lsb_c+5) = opcode_alui_c(5)) and (decode_aux.is_b_imm = '1'))) then -- immediate operation
              ctrl_nxt.alu_cp_trig(cp_sel_bitmanip_c) <= '1'; -- trigger BITMANIP CP
              execute_engine.state_nxt                <= ALU_WAIT;
            -- EXT: co-processor CONDITIONAL operations (multi-cycle) --
            elsif (CPU_EXTENSION_RISCV_Zicond = true) and (decode_aux.is_zicond = '1') and
                  (execute_engine.ir(instr_opcode_lsb_c+5) = opcode_alu_c(5)) then
              ctrl_nxt.alu_cp_trig(cp_sel_cond_c) <= '1'; -- trigger COND CP
              execute_engine.state_nxt            <= ALU_WAIT;
            -- BASE: co-processor SHIFT operation (multi-cycle) --
            elsif (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sll_c) or
                  (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sr_c) then
              ctrl_nxt.alu_cp_trig(cp_sel_shifter_c) <= '1'; -- trigger SHIFTER CP
              execute_engine.state_nxt               <= ALU_WAIT;
            -- BASE: ALU CORE operation (single-cycle) --
            else
              ctrl_nxt.rf_wb_en        <= '1'; -- valid RF write-back
              execute_engine.state_nxt <= DISPATCH;
            end if;


          when opcode_lui_c | opcode_auipc_c => -- load upper immediate / add upper immediate to PC
          -- ------------------------------------------------------------
            ctrl_nxt.alu_opa_mux <= '1'; -- ALU.OPA = PC (for AUIPC only)
            ctrl_nxt.alu_opb_mux <= '1'; -- use IMM as ALU.OPB
            if (execute_engine.ir(instr_opcode_lsb_c+5) = opcode_lui_c(5)) then -- LUI
              ctrl_nxt.alu_op <= alu_op_movb_c; -- pass immediate
            else -- AUIPC
              ctrl_nxt.alu_op <= alu_op_add_c; -- add PC and immediate
            end if;
            ctrl_nxt.rf_wb_en        <= '1'; -- valid RF write-back
            execute_engine.state_nxt <= DISPATCH;


          when opcode_load_c | opcode_store_c => -- load/store
          -- ------------------------------------------------------------
            ctrl_nxt.alu_opb_mux     <= '1'; -- use IMM as ALU.OPB
            ctrl_nxt.bus_mo_we       <= '1'; -- write memory output registers (data & address)
            execute_engine.state_nxt <= MEM_REQ;


          when opcode_branch_c | opcode_jal_c | opcode_jalr_c => -- branch / jump and link (with register)
          -- ------------------------------------------------------------
            ctrl_nxt.alu_opb_mux <= '1'; -- use IMM as ALU.OPB (branch target address offset)
            if (execute_engine.ir(instr_opcode_lsb_c+3 downto instr_opcode_lsb_c+2) = opcode_jalr_c(3 downto 2)) then -- JALR
              ctrl_nxt.alu_opa_mux <= '0'; -- use RS1 as ALU.OPA (branch target address base)
            else -- JAL
              ctrl_nxt.alu_opa_mux <= '1'; -- use PC as ALU.OPA (branch target address base)
            end if;
            execute_engine.state_nxt <= BRANCH;


          when opcode_fence_c => -- fence operations
          -- ------------------------------------------------------------
            if (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_fencei_c) and (CPU_EXTENSION_RISCV_Zifencei = true) then
              ctrl_nxt.bus_fencei      <= '1'; -- FENCE.I
              execute_engine.state_nxt <= TRAP_EXECUTE; -- use TRAP_EXECUTE to "modify" PC (PC <= PC)
            else
              execute_engine.state_nxt <= DISPATCH;
              if (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_fence_c) then
                ctrl_nxt.bus_fence <= '1'; -- FENCE
              end if;
            end if;


          when opcode_fop_c => -- floating-point operations
          -- ------------------------------------------------------------
            if (CPU_EXTENSION_RISCV_Zfinx = true) then
              ctrl_nxt.alu_cp_trig(cp_sel_fpu_c) <= '1'; -- trigger FPU CP
              execute_engine.state_nxt <= ALU_WAIT;
            else
              execute_engine.state_nxt <= DISPATCH;
            end if;


          when opcode_cust0_c | opcode_cust1_c | opcode_cust2_c | opcode_cust3_c => -- CFU: custom RISC-V instructions
          -- ------------------------------------------------------------
            if (CPU_EXTENSION_RISCV_Zxcfu = true) then
              ctrl_nxt.alu_cp_trig(cp_sel_cfu_c) <= '1'; -- trigger CFU CP
              execute_engine.state_nxt <= ALU_WAIT;
            else
              execute_engine.state_nxt <= DISPATCH;
            end if;


          when others => -- environment/CSR operation or ILLEGAL opcode
          -- ------------------------------------------------------------
            csr.re_nxt               <= '1';
            execute_engine.state_nxt <= SYSTEM; -- no state change if illegal opcode

        end case; -- /EXECUTE


      when ALU_WAIT => -- wait for multi-cycle ALU operation (ALU co-processor) to finish
      -- ------------------------------------------------------------
        ctrl_nxt.alu_op <= alu_op_cp_c;
        -- wait for completion or abort on illegal instruction exception (the co-processor will also terminate operations)
        if (alu_cp_done_i = '1') or (trap_ctrl.exc_buf(exc_iillegal_c) = '1') then
          ctrl_nxt.rf_wb_en        <= '1'; -- valid RF write-back (won't happen in case of an illegal instruction)
          execute_engine.state_nxt <= DISPATCH;
        end if;


      when BRANCH => -- update PC on taken branches and jumps
      -- ------------------------------------------------------------
        ctrl_nxt.rf_mux           <= rf_mux_npc_c; -- return addree = next PC
        execute_engine.pc_mux_sel <= '1'; -- PC <= alu.add = branch/jump destination
        execute_engine.pc_we      <= '1'; -- update PC with destination; will be overridden again in DISPATCH if branch not taken
        if (execute_engine.ir(instr_opcode_lsb_c+2) = '1') or (execute_engine.branch_taken = '1') then -- JAL/JALR or taken branch
          fetch_engine.reset       <= '1'; -- reset instruction fetch starting at modified PC
          execute_engine.state_nxt <= BRANCHED;
        else
          execute_engine.state_nxt <= DISPATCH;
        end if;
        -- valid RF write-back? --
        if (execute_engine.ir(instr_opcode_lsb_c+2) = '1') then -- is jump-and-link?
          ctrl_nxt.rf_wb_en <= '1';
        end if;


      when BRANCHED => -- delay cycle to wait for reset of pipeline front-end (instruction fetch)
      -- ------------------------------------------------------------
        execute_engine.branched_nxt <= '1'; -- this is an actual branch
        execute_engine.state_nxt    <= DISPATCH;
        -- housekeeping: use this state also to (re-)initialize the register file's x0/zero register --
        if (reset_x0_c = true) then -- if x0 is a "real" register that has to be initialized to zero
          ctrl_nxt.rf_mux     <= rf_mux_csr_c; -- this will return 0 since csr.re_nxt has not been set
          ctrl_nxt.rf_zero_we <= '1'; -- allow/force write access to x0
        end if;


      when MEM_REQ => -- trigger memory request
      -- ------------------------------------------------------------
        if (trap_ctrl.exc_buf(exc_iillegal_c) = '0') then -- not an illegal instruction
          ctrl_nxt.bus_req <= '1'; -- trigger memory request
        end if;
        execute_engine.state_nxt <= MEM_WAIT;


      when MEM_WAIT => -- wait for bus transaction to finish
      -- ------------------------------------------------------------
        ctrl_nxt.rf_mux <= rf_mux_mem_c; -- memory read data
        -- wait for memory response --
        if ((trap_ctrl.exc_buf(exc_laccess_c) or trap_ctrl.exc_buf(exc_saccess_c) or -- bus access error
             trap_ctrl.exc_buf(exc_lalign_c)  or trap_ctrl.exc_buf(exc_salign_c)  or -- alignment error
             trap_ctrl.exc_buf(exc_iillegal_c)) = '1') then -- illegal instruction
          execute_engine.state_nxt <= DISPATCH; -- abort!
        elsif (bus_d_wait_i = '0') then -- wait for bus to finish transaction
          if (execute_engine.ir(instr_opcode_msb_c-1) = '0') then -- load
            ctrl_nxt.rf_wb_en <= '1'; -- data write-back
          end if;
          execute_engine.state_nxt <= DISPATCH;
        end if;


      when CPU_SLEEP => -- Sleep mode; execute as NOP during debugging; wakeup on pending IRQ
      -- ------------------------------------------------------------
        if (debug_ctrl.running = '1') or (csr.dcsr_step = '1') or (trap_ctrl.wakeup = '1') then
          execute_engine.state_nxt <= DISPATCH;
        end if;


      when others => -- SYSTEM - system environment operation; no state change if illegal instruction
      -- ------------------------------------------------------------
        execute_engine.state_nxt <= DISPATCH;
        ctrl_nxt.rf_mux          <= rf_mux_csr_c; -- CSR read data
        if (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_env_c) and -- ENVIRONMENT
           (trap_ctrl.exc_buf(exc_iillegal_c) = '0') then -- and NOT already identified as illegal instruction
          case execute_engine.ir(instr_funct12_msb_c downto instr_funct12_lsb_c) is
            when funct12_ecall_c  => trap_ctrl.env_call       <= '1'; -- ecall
            when funct12_ebreak_c => trap_ctrl.break_point    <= '1'; -- ebreak
            when funct12_mret_c   => execute_engine.state_nxt <= TRAP_EXIT; -- mret
            when funct12_dret_c   => execute_engine.state_nxt <= TRAP_EXIT; debug_ctrl.dret <= '1'; -- dret
            when others           => execute_engine.state_nxt <= CPU_SLEEP; -- "funct12_wfi_c" - wfi/sleep or illegal
          end case;
        else -- CSR ACCESS - no CSR will be altered if illegal instruction
          if (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrw_c) or  -- CSRRW:  always write CSR
             (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrwi_c) or -- CSRRWI: always write CSR
             (decode_aux.rs1_zero = '0') then -- CSRR(S/C)(I): write CSR if rs1/imm5 is NOT zero
            csr.we_nxt <= '1';
          end if;
          ctrl_nxt.rf_wb_en <= '1'; -- valid RF write-back
        end if;

    end case;
  end process execute_engine_fsm_comb;


  -- CPU Control Bus Output -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- register file --
  ctrl_o.rf_wb_en     <= ctrl.rf_wb_en and (not trap_ctrl.exc_buf(exc_iillegal_c)); -- no write if illegal instruction
  ctrl_o.rf_rs1       <= execute_engine.ir(instr_rs1_msb_c downto instr_rs1_lsb_c);
  ctrl_o.rf_rs2       <= execute_engine.ir(instr_rs2_msb_c downto instr_rs2_lsb_c);
  ctrl_o.rf_rs3       <= execute_engine.ir(instr_rs3_msb_c downto instr_rs3_lsb_c);
  ctrl_o.rf_rd        <= execute_engine.ir(instr_rd_msb_c  downto instr_rd_lsb_c);
  ctrl_o.rf_mux       <= ctrl.rf_mux;
  ctrl_o.rf_zero_we   <= ctrl.rf_zero_we;

  -- alu --
  ctrl_o.alu_op       <= ctrl.alu_op;
  ctrl_o.alu_opa_mux  <= ctrl.alu_opa_mux;
  ctrl_o.alu_opb_mux  <= ctrl.alu_opb_mux;
  ctrl_o.alu_unsigned <= ctrl.alu_unsigned;
  ctrl_o.alu_frm      <= csr.frm;
  ctrl_o.alu_cp_trig  <= ctrl.alu_cp_trig;

  -- bus interface --
  ctrl_o.bus_req      <= ctrl.bus_req;
  ctrl_o.bus_mo_we    <= ctrl.bus_mo_we;
  ctrl_o.bus_fence    <= ctrl.bus_fence;
  ctrl_o.bus_fencei   <= ctrl.bus_fencei;
  ctrl_o.bus_priv     <= csr.mstatus_mpp when (csr.mstatus_mprv = '1') else csr.privilege_eff; -- effective privilege level for loads/stores in M-mode

  -- instruction word bit fields --
  ctrl_o.ir_funct3    <= execute_engine.ir(instr_funct3_msb_c  downto instr_funct3_lsb_c);
  ctrl_o.ir_funct12   <= execute_engine.ir(instr_funct12_msb_c downto instr_funct12_lsb_c);
  ctrl_o.ir_opcode    <= execute_engine.ir(instr_opcode_msb_c  downto instr_opcode_lsb_c);

  -- cpu status --
  ctrl_o.cpu_priv     <= csr.privilege_eff;
  ctrl_o.cpu_sleep    <= '1' when (execute_engine.state = CPU_SLEEP) else '0';
  ctrl_o.cpu_trap     <= trap_ctrl.env_start_ack;
  ctrl_o.cpu_debug    <= debug_ctrl.running;


-- ****************************************************************************************************************************
-- Illegal Instruction and CSR Access Check
-- ****************************************************************************************************************************

  -- CSR Access Check: Available at All -----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_avail_check: process(csr.addr)
  begin
    csr_reg_valid <= '0'; -- default: invalid access
    case csr.addr is

      -- floating-point CSRs --
      when csr_fflags_c | csr_frm_c | csr_fcsr_c =>
        csr_reg_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zfinx); -- valid if FPU implemented

      -- machine trap setup/handling, counters, environment & information registers, etc. --
      when csr_mstatus_c       | csr_mstatush_c   | csr_misa_c    | csr_mie_c        | csr_mtvec_c     |
           csr_mscratch_c      | csr_mepc_c       | csr_mcause_c  | csr_mip_c        | csr_mtval_c     |
           csr_mcountinhibit_c | csr_mcounteren_c | csr_menvcfg_c | csr_menvcfgh_c   | csr_mvendorid_c |
           csr_marchid_c       | csr_mimpid_c     | csr_mhartid_c | csr_mconfigptr_c | csr_mxisa_c =>
        csr_reg_valid <= '1';

      -- physical memory protection (PMP) --
      when csr_pmpcfg0_c   | csr_pmpcfg1_c   | csr_pmpcfg2_c   | csr_pmpcfg3_c   | csr_pmpcfg4_c   | csr_pmpcfg5_c   | csr_pmpcfg6_c   | csr_pmpcfg7_c   | -- configuration
           csr_pmpcfg8_c   | csr_pmpcfg9_c   | csr_pmpcfg10_c  | csr_pmpcfg11_c  | csr_pmpcfg12_c  | csr_pmpcfg13_c  | csr_pmpcfg14_c  | csr_pmpcfg15_c  |
           csr_pmpaddr0_c  | csr_pmpaddr1_c  | csr_pmpaddr2_c  | csr_pmpaddr3_c  | csr_pmpaddr4_c  | csr_pmpaddr5_c  | csr_pmpaddr6_c  | csr_pmpaddr7_c  | -- address
           csr_pmpaddr8_c  | csr_pmpaddr9_c  | csr_pmpaddr10_c | csr_pmpaddr11_c | csr_pmpaddr12_c | csr_pmpaddr13_c | csr_pmpaddr14_c | csr_pmpaddr15_c |
           csr_pmpaddr16_c | csr_pmpaddr17_c | csr_pmpaddr18_c | csr_pmpaddr19_c | csr_pmpaddr20_c | csr_pmpaddr21_c | csr_pmpaddr22_c | csr_pmpaddr23_c |
           csr_pmpaddr24_c | csr_pmpaddr25_c | csr_pmpaddr26_c | csr_pmpaddr27_c | csr_pmpaddr28_c | csr_pmpaddr29_c | csr_pmpaddr30_c | csr_pmpaddr31_c |
           csr_pmpaddr32_c | csr_pmpaddr33_c | csr_pmpaddr34_c | csr_pmpaddr35_c | csr_pmpaddr36_c | csr_pmpaddr37_c | csr_pmpaddr38_c | csr_pmpaddr39_c |
           csr_pmpaddr40_c | csr_pmpaddr41_c | csr_pmpaddr42_c | csr_pmpaddr43_c | csr_pmpaddr44_c | csr_pmpaddr45_c | csr_pmpaddr46_c | csr_pmpaddr47_c |
           csr_pmpaddr48_c | csr_pmpaddr49_c | csr_pmpaddr50_c | csr_pmpaddr51_c | csr_pmpaddr52_c | csr_pmpaddr53_c | csr_pmpaddr54_c | csr_pmpaddr55_c |
           csr_pmpaddr56_c | csr_pmpaddr57_c | csr_pmpaddr58_c | csr_pmpaddr59_c | csr_pmpaddr60_c | csr_pmpaddr61_c | csr_pmpaddr62_c | csr_pmpaddr63_c =>
        csr_reg_valid <= bool_to_ulogic_f(boolean(PMP_NUM_REGIONS > 0)); -- valid if PMP implemented

      -- hardware performance monitors (HPM) --
      when csr_hpmcounter3_c    | csr_hpmcounter4_c    | csr_hpmcounter5_c    | csr_hpmcounter6_c    | csr_hpmcounter7_c    | csr_hpmcounter8_c    | -- user counters LOW
           csr_hpmcounter9_c    | csr_hpmcounter10_c   | csr_hpmcounter11_c   | csr_hpmcounter12_c   | csr_hpmcounter13_c   | csr_hpmcounter14_c   |
           csr_hpmcounter15_c   | csr_hpmcounter16_c   | csr_hpmcounter17_c   | csr_hpmcounter18_c   | csr_hpmcounter19_c   | csr_hpmcounter20_c   |
           csr_hpmcounter21_c   | csr_hpmcounter22_c   | csr_hpmcounter23_c   | csr_hpmcounter24_c   | csr_hpmcounter25_c   | csr_hpmcounter26_c   |
           csr_hpmcounter27_c   | csr_hpmcounter28_c   | csr_hpmcounter29_c   | csr_hpmcounter30_c   | csr_hpmcounter31_c   |
           csr_hpmcounter3h_c   | csr_hpmcounter4h_c   | csr_hpmcounter5h_c   | csr_hpmcounter6h_c   | csr_hpmcounter7h_c   | csr_hpmcounter8h_c   | -- user counters HIGH
           csr_hpmcounter9h_c   | csr_hpmcounter10h_c  | csr_hpmcounter11h_c  | csr_hpmcounter12h_c  | csr_hpmcounter13h_c  | csr_hpmcounter14h_c  |
           csr_hpmcounter15h_c  | csr_hpmcounter16h_c  | csr_hpmcounter17h_c  | csr_hpmcounter18h_c  | csr_hpmcounter19h_c  | csr_hpmcounter20h_c  |
           csr_hpmcounter21h_c  | csr_hpmcounter22h_c  | csr_hpmcounter23h_c  | csr_hpmcounter24h_c  | csr_hpmcounter25h_c  | csr_hpmcounter26h_c  |
           csr_hpmcounter27h_c  | csr_hpmcounter28h_c  | csr_hpmcounter29h_c  | csr_hpmcounter30h_c  | csr_hpmcounter31h_c  |
           csr_mhpmcounter3_c   | csr_mhpmcounter4_c   | csr_mhpmcounter5_c   | csr_mhpmcounter6_c   | csr_mhpmcounter7_c   | csr_mhpmcounter8_c   | -- machine counters LOW
           csr_mhpmcounter9_c   | csr_mhpmcounter10_c  | csr_mhpmcounter11_c  | csr_mhpmcounter12_c  | csr_mhpmcounter13_c  | csr_mhpmcounter14_c  |
           csr_mhpmcounter15_c  | csr_mhpmcounter16_c  | csr_mhpmcounter17_c  | csr_mhpmcounter18_c  | csr_mhpmcounter19_c  | csr_mhpmcounter20_c  |
           csr_mhpmcounter21_c  | csr_mhpmcounter22_c  | csr_mhpmcounter23_c  | csr_mhpmcounter24_c  | csr_mhpmcounter25_c  | csr_mhpmcounter26_c  |
           csr_mhpmcounter27_c  | csr_mhpmcounter28_c  | csr_mhpmcounter29_c  | csr_mhpmcounter30_c  | csr_mhpmcounter31_c  |
           csr_mhpmcounter3h_c  | csr_mhpmcounter4h_c  | csr_mhpmcounter5h_c  | csr_mhpmcounter6h_c  | csr_mhpmcounter7h_c  | csr_mhpmcounter8h_c  | -- machine counters HIGH
           csr_mhpmcounter9h_c  | csr_mhpmcounter10h_c | csr_mhpmcounter11h_c | csr_mhpmcounter12h_c | csr_mhpmcounter13h_c | csr_mhpmcounter14h_c |
           csr_mhpmcounter15h_c | csr_mhpmcounter16h_c | csr_mhpmcounter17h_c | csr_mhpmcounter18h_c | csr_mhpmcounter19h_c | csr_mhpmcounter20h_c |
           csr_mhpmcounter21h_c | csr_mhpmcounter22h_c | csr_mhpmcounter23h_c | csr_mhpmcounter24h_c | csr_mhpmcounter25h_c | csr_mhpmcounter26h_c |
           csr_mhpmcounter27h_c | csr_mhpmcounter28h_c | csr_mhpmcounter29h_c | csr_mhpmcounter30h_c | csr_mhpmcounter31h_c |
           csr_mhpmevent3_c     | csr_mhpmevent4_c     | csr_mhpmevent5_c     | csr_mhpmevent6_c     | csr_mhpmevent7_c     | csr_mhpmevent8_c     | -- event configuration
           csr_mhpmevent9_c     | csr_mhpmevent10_c    | csr_mhpmevent11_c    | csr_mhpmevent12_c    | csr_mhpmevent13_c    | csr_mhpmevent14_c    |
           csr_mhpmevent15_c    | csr_mhpmevent16_c    | csr_mhpmevent17_c    | csr_mhpmevent18_c    | csr_mhpmevent19_c    | csr_mhpmevent20_c    |
           csr_mhpmevent21_c    | csr_mhpmevent22_c    | csr_mhpmevent23_c    | csr_mhpmevent24_c    | csr_mhpmevent25_c    | csr_mhpmevent26_c    |
           csr_mhpmevent27_c    | csr_mhpmevent28_c    | csr_mhpmevent29_c    | csr_mhpmevent30_c    | csr_mhpmevent31_c =>
        csr_reg_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zihpm); -- valid if Zihpm implemented

      -- counter and timer CSRs --
      when csr_cycle_c  | csr_mcycle_c  | csr_instret_c  | csr_minstret_c  |
           csr_cycleh_c | csr_mcycleh_c | csr_instreth_c | csr_minstreth_c =>
        csr_reg_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicntr); -- valid if Zicntr implemented

      -- debug-mode CSRs --
      when csr_dcsr_c | csr_dpc_c | csr_dscratch0_c =>
        csr_reg_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Sdext); -- valid if debug-mode implemented

      -- trigger module CSRs --
      when csr_tselect_c | csr_tdata1_c | csr_tdata2_c | csr_tdata3_c | csr_tinfo_c | csr_tcontrol_c | csr_mcontext_c | csr_scontext_c =>
        csr_reg_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Sdtrig); -- valid if trigger module implemented

      -- undefined / not implemented --
      when others =>
        csr_reg_valid <= '0'; -- invalid access

    end case;
  end process csr_avail_check;


  -- CSR Access Check: R/W Capabilities -----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_rw_check: process(csr.addr, execute_engine.ir, decode_aux.rs1_zero)
  begin
    if (csr.addr(11 downto 10) = "11") and -- CSR is read-only
       -- is this CSR instruction really going to write to the CSR? --
       ((execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrw_c) or -- always write CSR
        (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrwi_c) or -- always write CSR
        (decode_aux.rs1_zero = '0')) then -- clear/set: write CSR if rs1/imm5 is NOT zero
      csr_rw_valid <= '0'; -- invalid access
    else
      csr_rw_valid <= '1'; -- access granted
    end if;
  end process csr_rw_check;


  -- CSR Access Check: Privilege Level ------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_priv_check: process(csr, debug_ctrl)
  begin
    if ((csr.addr = csr_dcsr_c) or (csr.addr = csr_dpc_c) or (csr.addr = csr_dscratch0_c)) and -- debug-mode-only CSR?
       (CPU_EXTENSION_RISCV_Sdext = true) and (debug_ctrl.running = '0') then -- debug-mode implemented and not running?
      csr_priv_valid <= '0'; -- invalid access
    elsif (csr.addr(11 downto 8) = csr_cycle_c(11 downto 8)) and -- user counter access
          ((CPU_EXTENSION_RISCV_Zicntr = true) or (CPU_EXTENSION_RISCV_Zihpm = true)) and -- any counters available?
          (CPU_EXTENSION_RISCV_U = true) and (csr.privilege_eff = '0') and -- user mode enabled and active
          (csr.mcounteren(to_integer(unsigned(csr.addr(4 downto 0)))) = '0') then -- access not allowed?
      csr_priv_valid <= '0'; -- invalid access
    elsif (csr.addr(9 downto 8) /= "00") and (csr.privilege_eff = '0') then
      csr_priv_valid <= '0'; -- invalid access
    else
      csr_priv_valid <= '1'; -- access granted
    end if;
  end process csr_priv_check;


  -- Illegal Instruction Check --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  illegal_instruction_check: process(execute_engine, decode_aux, csr, csr_reg_valid, csr_rw_valid, csr_priv_valid, debug_ctrl)
  begin
    -- defaults --
    illegal_cmd <= '0';

    -- check instruction word encoding and side effects --
    case execute_engine.ir(instr_opcode_msb_c downto instr_opcode_lsb_c) is

      when opcode_lui_c | opcode_auipc_c | opcode_jal_c => -- LUI, UIPC, JAL (only check actual OPCODE)
      -- ------------------------------------------------------------
        illegal_cmd <= '0';

      when opcode_jalr_c => -- check JALR.funct3
      -- ------------------------------------------------------------
        case execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          when "000"  => illegal_cmd <= '0';
          when others => illegal_cmd <= '1';
        end case;

      when opcode_branch_c => -- check BRANCH.funct3
      -- ------------------------------------------------------------
        case execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          when funct3_beq_c | funct3_bne_c | funct3_blt_c | funct3_bge_c | funct3_bltu_c | funct3_bgeu_c => illegal_cmd <= '0';
          when others => illegal_cmd <= '1';
        end case;

      when opcode_load_c => -- check LOAD.funct3
      -- ------------------------------------------------------------
        case execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          when funct3_lb_c | funct3_lh_c | funct3_lw_c | funct3_lbu_c | funct3_lhu_c => illegal_cmd <= '0';
          when others => illegal_cmd <= '1';
        end case;

      when opcode_store_c => -- check STORE.funct3
      -- ------------------------------------------------------------
        case execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          when funct3_sb_c | funct3_sh_c | funct3_sw_c => illegal_cmd <= '0';
          when others => illegal_cmd <= '1';
        end case;

      when opcode_alu_c => -- check ALU.funct3 & ALU.funct7
      -- ------------------------------------------------------------
        if ((((execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_subadd_c) or (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sr_c)) and
             (execute_engine.ir(instr_funct7_msb_c-2 downto instr_funct7_lsb_c) = "00000") and (execute_engine.ir(instr_funct7_msb_c) = '0')) or
            (((execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sll_c) or
              (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_slt_c) or
              (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sltu_c) or
              (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_xor_c) or
              (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_or_c) or
              (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_and_c)) and
              (execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000000"))) or -- valid base ALU instruction?
           (((CPU_EXTENSION_RISCV_M = true) or (CPU_EXTENSION_RISCV_Zmmul = true)) and (decode_aux.is_m_mul = '1')) or -- valid MUL instruction?
           ((CPU_EXTENSION_RISCV_M = true) and (decode_aux.is_m_div = '1')) or -- valid DIV instruction?
           ((CPU_EXTENSION_RISCV_B = true) and (decode_aux.is_b_reg = '1')) or -- valid BITMANIP register instruction?
           ((CPU_EXTENSION_RISCV_Zicond = true) and (decode_aux.is_zicond = '1')) then -- valid CONDITIONAL instruction?
          illegal_cmd <= '0';
        else
          illegal_cmd <= '1';
        end if;

      when opcode_alui_c => -- check ALU.funct3 & ALU.funct7
      -- ------------------------------------------------------------
        if ((execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_subadd_c) or
            (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_slt_c) or
            (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sltu_c) or
            (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_xor_c) or
            (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_or_c) or
            (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_and_c) or
            ((execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sll_c) and
             (execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000000")) or
            ((execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sr_c) and
             ((execute_engine.ir(instr_funct7_msb_c-2 downto instr_funct7_lsb_c) = "00000") and (execute_engine.ir(instr_funct7_msb_c) = '0')))) or -- valid base ALUI instruction?
           ((CPU_EXTENSION_RISCV_B = true) and (decode_aux.is_b_imm = '1')) then -- valid BITMANIP immediate instruction?
          illegal_cmd <= '0';
        else
          illegal_cmd <= '1';
        end if;

      when opcode_fence_c => -- check FENCE.funct3, ignore all remaining bit-fields
      -- ------------------------------------------------------------
        case execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          when funct3_fence_c  => illegal_cmd <= '0'; -- FENCE
          when funct3_fencei_c => illegal_cmd <= not bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zifencei); -- FENCE.I
          when others          => illegal_cmd <= '1';
        end case;

      when opcode_system_c => -- check system instructions
      -- ------------------------------------------------------------
        if (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_env_c) then -- system environment
          if (decode_aux.rs1_zero = '1') and (decode_aux.rd_zero = '1') then
            case execute_engine.ir(instr_funct12_msb_c downto instr_funct12_lsb_c) is
              when funct12_ecall_c | funct12_ebreak_c => illegal_cmd <= '0'; -- ECALL, EBREAK
              when funct12_mret_c                     => illegal_cmd <= not csr.privilege; -- MRET (only allowed in ACTUAL M-mode)
              when funct12_wfi_c                      => illegal_cmd <= (not csr.privilege) and csr.mstatus_tw; -- WFI (only allowed in ACTUAL M-mode or if mstatus.TW = 0)
              when funct12_dret_c                     => illegal_cmd <= not debug_ctrl.running; -- DRET (only allowed in D-mode)
              when others => illegal_cmd <= '1';
            end case;
          else
            illegal_cmd <= '1';
          end if;
        elsif (csr_reg_valid = '0') or (csr_rw_valid = '0') or (csr_priv_valid = '0') or -- invalid CSR access?
              (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csril_c) then -- invalid CSR access instruction?
          illegal_cmd <= '1';
        else
          illegal_cmd <= '0';
        end if;

      when opcode_fop_c => -- floating point operations - single/dual operands
      -- ------------------------------------------------------------
        illegal_cmd <= (not bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zfinx)) or (not decode_aux.is_f_op);

      when opcode_cust0_c | opcode_cust1_c | opcode_cust2_c | opcode_cust3_c => -- custom instructions (CFU)
      -- ------------------------------------------------------------
        illegal_cmd <= not bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zxcfu); -- CFU extension implemented?

      when others => -- undefined/illegal opcode
      -- ------------------------------------------------------------
        illegal_cmd <= '1';

    end case;
  end process illegal_instruction_check;


  -- Illegal Operation Check ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- check in EXECUTE states: any illegal instruction trigger? --
  trap_ctrl.instr_il <= (illegal_cmd or alu_exc_i) when ((execute_engine.state = EXECUTE) or (execute_engine.state = ALU_WAIT)) else '0';


-- ****************************************************************************************************************************
-- Trap Controller for Interrupts and Exceptions
-- ****************************************************************************************************************************

  -- Trap Buffer ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trap_buffer: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      trap_ctrl.exc_buf <= (others => '0');
      trap_ctrl.irq_pnd <= (others => '0');
      trap_ctrl.irq_buf <= (others => '0');
    elsif rising_edge(clk_i) then

      -- Exception Buffer -----------------------------------------------------
      -- If several exception sources trigger at once, all the requests will
      -- stay active until the trap environment is started. Only the exception
      -- with highest priority will be used to update the MCAUSE CSR. The
      -- remaining ones will be discarded.
      -- ----------------------------------------------------------------------

      -- misaligned load/store/instruction address --
      trap_ctrl.exc_buf(exc_lalign_c) <= (trap_ctrl.exc_buf(exc_lalign_c) or ma_load_i)          and (not trap_ctrl.env_start_ack);
      trap_ctrl.exc_buf(exc_salign_c) <= (trap_ctrl.exc_buf(exc_salign_c) or ma_store_i)         and (not trap_ctrl.env_start_ack);
      trap_ctrl.exc_buf(exc_ialign_c) <= (trap_ctrl.exc_buf(exc_ialign_c) or trap_ctrl.instr_ma) and (not trap_ctrl.env_start_ack);

      -- load/store/instruction bus access fault --
      trap_ctrl.exc_buf(exc_laccess_c) <= (trap_ctrl.exc_buf(exc_laccess_c) or be_load_i)          and (not trap_ctrl.env_start_ack);
      trap_ctrl.exc_buf(exc_saccess_c) <= (trap_ctrl.exc_buf(exc_saccess_c) or be_store_i)         and (not trap_ctrl.env_start_ack);
      trap_ctrl.exc_buf(exc_iaccess_c) <= (trap_ctrl.exc_buf(exc_iaccess_c) or trap_ctrl.instr_be) and (not trap_ctrl.env_start_ack);

      -- illegal instruction & environment call --
      trap_ctrl.exc_buf(exc_ecall_c)    <= (trap_ctrl.exc_buf(exc_ecall_c)    or trap_ctrl.env_call) and (not trap_ctrl.env_start_ack);
      trap_ctrl.exc_buf(exc_iillegal_c) <= (trap_ctrl.exc_buf(exc_iillegal_c) or trap_ctrl.instr_il) and (not trap_ctrl.env_start_ack);

      -- break point --
      if (CPU_EXTENSION_RISCV_Sdext = true) then
        trap_ctrl.exc_buf(exc_ebreak_c) <= (not trap_ctrl.env_start_ack) and (trap_ctrl.exc_buf(exc_ebreak_c) or
          (hw_trigger_fire and (not csr.tdata1_action)) or -- trigger module fires and enter-debug is disabled
          (trap_ctrl.break_point and (    csr.privilege) and (not csr.dcsr_ebreakm) and (not debug_ctrl.running)) or -- enter M-mode handler on ebreak in M-mode
          (trap_ctrl.break_point and (not csr.privilege) and (not csr.dcsr_ebreaku) and (not debug_ctrl.running))); -- enter M-mode handler on ebreak in U-mode
      else
        trap_ctrl.exc_buf(exc_ebreak_c) <= (trap_ctrl.exc_buf(exc_ebreak_c) or trap_ctrl.break_point or hw_trigger_fire) and (not trap_ctrl.env_start_ack);
      end if;

      -- debug-mode entry --
      if (CPU_EXTENSION_RISCV_Sdext = true) then
        trap_ctrl.exc_buf(exc_db_break_c) <= (trap_ctrl.exc_buf(exc_db_break_c) or debug_ctrl.trig_break) and (not trap_ctrl.env_start_ack);
        trap_ctrl.exc_buf(exc_db_hw_c)    <= (trap_ctrl.exc_buf(exc_db_hw_c)    or debug_ctrl.trig_hw)    and (not trap_ctrl.env_start_ack);
      else
        trap_ctrl.exc_buf(exc_db_break_c) <= '0';
        trap_ctrl.exc_buf(exc_db_hw_c)    <= '0';
      end if;


      -- Interrupt Pending Buffer ---------------------------------------------
      -- Once triggered, the fast interrupt requests stay active until
      -- explicitly cleared via the MIP CSR.
      -- ----------------------------------------------------------------------

      -- RISC-V machine interrupts --
      trap_ctrl.irq_pnd(irq_msi_irq_c) <= msw_irq_i;
      trap_ctrl.irq_pnd(irq_mei_irq_c) <= mext_irq_i;
      trap_ctrl.irq_pnd(irq_mti_irq_c) <= mtime_irq_i;

      -- NEORV32-specific fast interrupts --
      for i in 0 to 15 loop
        trap_ctrl.irq_pnd(irq_firq_0_c+i) <= (trap_ctrl.irq_pnd(irq_firq_0_c+i) and csr.mip_firq_nclr(i)) or firq_i(i);
      end loop;

      -- debug-mode entry --
      trap_ctrl.irq_pnd(irq_db_halt_c) <= '0'; -- unused
      trap_ctrl.irq_pnd(irq_db_step_c) <= '0'; -- unused


      -- Interrupt Masking Buffer ---------------------------------------------
      -- Masking of interrupt request lines. Furthermore, this buffer ensures
      -- that an *active* interrupt request line *stays* active (even if
      -- disabled via MIE) if the trap environment is *currently* starting.
      -- ----------------------------------------------------------------------

      -- RISC-V machine interrupts --
      trap_ctrl.irq_buf(irq_msi_irq_c) <= (trap_ctrl.irq_pnd(irq_msi_irq_c) and csr.mie_msi) or (trap_ctrl.env_start and trap_ctrl.irq_buf(irq_msi_irq_c));
      trap_ctrl.irq_buf(irq_mei_irq_c) <= (trap_ctrl.irq_pnd(irq_mei_irq_c) and csr.mie_mei) or (trap_ctrl.env_start and trap_ctrl.irq_buf(irq_mei_irq_c));
      trap_ctrl.irq_buf(irq_mti_irq_c) <= (trap_ctrl.irq_pnd(irq_mti_irq_c) and csr.mie_mti) or (trap_ctrl.env_start and trap_ctrl.irq_buf(irq_mti_irq_c));

      -- NEORV32-specific fast interrupts --
      for i in 0 to 15 loop
        trap_ctrl.irq_buf(irq_firq_0_c+i) <= (trap_ctrl.irq_pnd(irq_firq_0_c+i) and csr.mie_firq(i)) or (trap_ctrl.env_start and trap_ctrl.irq_buf(irq_firq_0_c+i));
      end loop;

      -- debug-mode entry --
      if (CPU_EXTENSION_RISCV_Sdext = true) then
        trap_ctrl.irq_buf(irq_db_halt_c) <= debug_ctrl.trig_halt or (trap_ctrl.env_start and trap_ctrl.irq_buf(irq_db_halt_c));
        trap_ctrl.irq_buf(irq_db_step_c) <= debug_ctrl.trig_step or (trap_ctrl.env_start and trap_ctrl.irq_buf(irq_db_step_c));
      else
        trap_ctrl.irq_buf(irq_db_halt_c) <= '0';
        trap_ctrl.irq_buf(irq_db_step_c) <= '0';
      end if;

    end if;
  end process trap_buffer;


  -- Trap Controller ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trap_controller: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      trap_ctrl.wakeup    <= '0';
      trap_ctrl.env_start <= '0';
    elsif rising_edge(clk_i) then
      trap_ctrl.wakeup <= or_reduce_f(trap_ctrl.irq_buf); -- wakeup from sleep due to any pending IRQ (including debug IRQs!)
      if (trap_ctrl.env_start = '0') then -- no started trap handler yet
        -- trigger IRQ only in EXECUTE state to continue execution even on permanent IRQ
        if (trap_ctrl.exc_fire = '1') or ((trap_ctrl.irq_fire = '1') and (execute_engine.state = EXECUTE)) then
          trap_ctrl.env_start <= '1'; -- now execute engine can start trap handler
        end if;
      else -- trap environment ready to start
        if (trap_ctrl.env_start_ack = '1') then -- start of trap handler acknowledged by execute engine
          trap_ctrl.env_start <= '0';
        end if;
      end if;
    end if;
  end process trap_controller;

  -- any exception? --
  trap_ctrl.exc_fire <= '1' when (or_reduce_f(trap_ctrl.exc_buf) = '1') else '0'; -- sync. exceptions CANNOT be masked

  -- valid interrupt request? --
  trap_ctrl.irq_fire <= '1' when
    (
     (or_reduce_f(trap_ctrl.irq_buf(irq_firq_15_c downto irq_msi_irq_c)) = '1') and -- pending machine IRQ
     ((csr.mstatus_mie = '1') or (csr.privilege = priv_mode_u_c)) and -- take IRQ when in M-mode and MIE=1 OR when in U-mode
     (debug_ctrl.running = '0') and (csr.dcsr_step = '0') -- no IRQs when in single-stepping mode or during single-stepping
    ) or
    (trap_ctrl.irq_buf(irq_db_step_c) = '1') or -- debug-mode single-step IRQ
    (trap_ctrl.irq_buf(irq_db_halt_c) = '1') else '0'; -- debug-mode halt IRQ

  -- exception program counter (for updating xPC CSRs) --
  trap_ctrl.epc <= execute_engine.next_pc when (trap_ctrl.cause(trap_ctrl.cause'left) = '1') else execute_engine.pc;


  -- Trap Priority Encoder ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trap_encoder: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- standard RISC-V exceptions --
      if    (trap_ctrl.exc_buf(exc_ialign_c)   = '1') then trap_ctrl.cause <= trap_ima_c;  -- instruction address misaligned
      elsif (trap_ctrl.exc_buf(exc_iaccess_c)  = '1') then trap_ctrl.cause <= trap_iaf_c;  -- instruction access fault
      elsif (trap_ctrl.exc_buf(exc_iillegal_c) = '1') then trap_ctrl.cause <= trap_iil_c;  -- illegal instruction
      elsif (trap_ctrl.exc_buf(exc_ecall_c)    = '1') then trap_ctrl.cause <= trap_env_c(6 downto 2) & csr.privilege & csr.privilege; -- environment call (U/M)
      elsif (trap_ctrl.exc_buf(exc_ebreak_c)   = '1') then trap_ctrl.cause <= trap_brk_c;  -- breakpoint
      elsif (trap_ctrl.exc_buf(exc_salign_c)   = '1') then trap_ctrl.cause <= trap_sma_c;  -- store address misaligned
      elsif (trap_ctrl.exc_buf(exc_lalign_c)   = '1') then trap_ctrl.cause <= trap_lma_c;  -- load address misaligned
      elsif (trap_ctrl.exc_buf(exc_saccess_c)  = '1') then trap_ctrl.cause <= trap_saf_c;  -- store access fault
      elsif (trap_ctrl.exc_buf(exc_laccess_c)  = '1') then trap_ctrl.cause <= trap_laf_c;  -- load access fault
      -- debug mode exceptions and interrupts --
      elsif (trap_ctrl.irq_buf(irq_db_halt_c)  = '1') then trap_ctrl.cause <= trap_db_halt_c;  -- external halt request (async)
      elsif (trap_ctrl.exc_buf(exc_db_hw_c)    = '1') then trap_ctrl.cause <= trap_db_trig_c;  -- hardware trigger (sync)
      elsif (trap_ctrl.exc_buf(exc_db_break_c) = '1') then trap_ctrl.cause <= trap_db_break_c; -- break instruction (sync)
      elsif (trap_ctrl.irq_buf(irq_db_step_c)  = '1') then trap_ctrl.cause <= trap_db_step_c;  -- single stepping (async)
      -- NEORV32-specific fast interrupts --
      elsif (trap_ctrl.irq_buf(irq_firq_0_c)   = '1') then trap_ctrl.cause <= trap_firq0_c;  -- fast interrupt channel 0
      elsif (trap_ctrl.irq_buf(irq_firq_1_c)   = '1') then trap_ctrl.cause <= trap_firq1_c;  -- fast interrupt channel 1
      elsif (trap_ctrl.irq_buf(irq_firq_2_c)   = '1') then trap_ctrl.cause <= trap_firq2_c;  -- fast interrupt channel 2
      elsif (trap_ctrl.irq_buf(irq_firq_3_c)   = '1') then trap_ctrl.cause <= trap_firq3_c;  -- fast interrupt channel 3
      elsif (trap_ctrl.irq_buf(irq_firq_4_c)   = '1') then trap_ctrl.cause <= trap_firq4_c;  -- fast interrupt channel 4
      elsif (trap_ctrl.irq_buf(irq_firq_5_c)   = '1') then trap_ctrl.cause <= trap_firq5_c;  -- fast interrupt channel 5
      elsif (trap_ctrl.irq_buf(irq_firq_6_c)   = '1') then trap_ctrl.cause <= trap_firq6_c;  -- fast interrupt channel 6
      elsif (trap_ctrl.irq_buf(irq_firq_7_c)   = '1') then trap_ctrl.cause <= trap_firq7_c;  -- fast interrupt channel 7
      elsif (trap_ctrl.irq_buf(irq_firq_8_c)   = '1') then trap_ctrl.cause <= trap_firq8_c;  -- fast interrupt channel 8
      elsif (trap_ctrl.irq_buf(irq_firq_9_c)   = '1') then trap_ctrl.cause <= trap_firq9_c;  -- fast interrupt channel 9
      elsif (trap_ctrl.irq_buf(irq_firq_10_c)  = '1') then trap_ctrl.cause <= trap_firq10_c; -- fast interrupt channel 10
      elsif (trap_ctrl.irq_buf(irq_firq_11_c)  = '1') then trap_ctrl.cause <= trap_firq11_c; -- fast interrupt channel 11
      elsif (trap_ctrl.irq_buf(irq_firq_12_c)  = '1') then trap_ctrl.cause <= trap_firq12_c; -- fast interrupt channel 12
      elsif (trap_ctrl.irq_buf(irq_firq_13_c)  = '1') then trap_ctrl.cause <= trap_firq13_c; -- fast interrupt channel 13
      elsif (trap_ctrl.irq_buf(irq_firq_14_c)  = '1') then trap_ctrl.cause <= trap_firq14_c; -- fast interrupt channel 14
      elsif (trap_ctrl.irq_buf(irq_firq_15_c)  = '1') then trap_ctrl.cause <= trap_firq15_c; -- fast interrupt channel 15
      -- standard RISC-V interrupts --
      elsif (trap_ctrl.irq_buf(irq_mei_irq_c)  = '1') then trap_ctrl.cause <= trap_mei_c; -- machine external interrupt (MEI)
      elsif (trap_ctrl.irq_buf(irq_msi_irq_c)  = '1') then trap_ctrl.cause <= trap_msi_c; -- machine SW interrupt (MSI)
      elsif (trap_ctrl.irq_buf(irq_mti_irq_c)  = '1') then trap_ctrl.cause <= trap_mti_c; -- machine timer interrupt (MTI)
      else trap_ctrl.cause <= trap_mti_c; end if; -- don't care
    end if;
  end process trap_encoder;


-- ****************************************************************************************************************************
-- Control and Status Registers (CSRs)
-- ****************************************************************************************************************************

  -- Control and Status Registers - Write Data ----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_write_data: process(execute_engine.ir, csr.rdata, rs1_i)
    variable tmp_v : std_ulogic_vector(XLEN-1 downto 0);
  begin
    -- immediate/register operand --
    if (execute_engine.ir(instr_funct3_msb_c) = '1') then
      tmp_v := (others => '0');
      tmp_v(4 downto 0) := execute_engine.ir(19 downto 15); -- uimm5
    else
      tmp_v := rs1_i;
    end if;
    -- tiny ALU to compute CSR write data --
    case execute_engine.ir(instr_funct3_msb_c-1 downto instr_funct3_lsb_c) is
      when "10"   => csr.wdata <= csr.rdata or tmp_v; -- set
      when "11"   => csr.wdata <= csr.rdata and (not tmp_v); -- clear
      when others => csr.wdata <= tmp_v; -- write
    end case;
  end process csr_write_data;


  -- Control and Status Registers - Write Access --------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      csr.we            <= '0';
      csr.privilege     <= priv_mode_m_c; -- start in MACHINE mode
      csr.mstatus_mie   <= '0';
      csr.mstatus_mpie  <= '0';
      csr.mstatus_mpp   <= '0';
      csr.mstatus_mprv  <= '0';
      csr.mstatus_tw    <= '0';
      csr.mie_msi       <= '0';
      csr.mie_mei       <= '0';
      csr.mie_mti       <= '0';
      csr.mie_firq      <= (others => '0');
      csr.mtvec         <= (others => '0');
      csr.mscratch      <= x"19880704";
      csr.mepc          <= (others => '0');
      csr.mcause        <= (others => '0');
      csr.mtval         <= (others => '0');
      csr.mcounteren    <= (others => '0');
      csr.mcountinhibit <= (others => '0');
      csr.mip_firq_nclr <= (others => '0');
      csr.fflags        <= (others => '0');
      csr.frm           <= (others => '0');
      csr.dcsr_ebreakm  <= '0';
      csr.dcsr_ebreaku  <= '0';
      csr.dcsr_step     <= '0';
      csr.dcsr_prv      <= priv_mode_m_c;
      csr.dcsr_cause    <= (others => '0');
      csr.dpc           <= (others => '0');
      csr.dscratch0     <= (others => '0');
      csr.tdata1_exe    <= '0';
      csr.tdata1_action <= '0';
      csr.tdata1_dmode  <= '0';
      csr.tdata2        <= (others => '0');
    elsif rising_edge(clk_i) then

      -- write access? --
      csr.we <= csr.we_nxt and (not trap_ctrl.exc_buf(exc_iillegal_c)); -- write if not illegal instruction

      -- defaults --
      csr.mip_firq_nclr <= (others => '1'); -- active low

      -- ********************************************************************************
      -- Manual CSR access by application software
      -- ********************************************************************************
      if (csr.we = '1') then -- manual write access and not illegal instruction
        case csr.addr is

          -- user floating-point CSRs --
          -- --------------------------------------------------------------------
          when csr_fflags_c => -- floating-point exception flags
            if (CPU_EXTENSION_RISCV_Zfinx = true) then
              csr.fflags <= csr.wdata(4 downto 0);
            end if;

          when csr_frm_c => -- floating-point rounding mode
            if (CPU_EXTENSION_RISCV_Zfinx = true) then
              csr.frm <= csr.wdata(2 downto 0);
            end if;

          when csr_fcsr_c => -- floating-point control/status (frm & fflags)
            if (CPU_EXTENSION_RISCV_Zfinx = true) then
              csr.frm    <= csr.wdata(7 downto 5);
              csr.fflags <= csr.wdata(4 downto 0);
            end if;

          -- machine trap setup --
          -- --------------------------------------------------------------------
          when csr_mstatus_c => -- machine status register
            csr.mstatus_mie  <= csr.wdata(03);
            csr.mstatus_mpie <= csr.wdata(07);
            if (CPU_EXTENSION_RISCV_U = true) then
              csr.mstatus_mpp  <= csr.wdata(11) or csr.wdata(12); -- everything /= U will fall back to M
              csr.mstatus_mprv <= csr.wdata(17);
              csr.mstatus_tw   <= csr.wdata(21);
            end if;

          when csr_mie_c => -- machine interrupt enable register
            csr.mie_msi  <= csr.wdata(03); -- machine SW IRQ enable
            csr.mie_mti  <= csr.wdata(07); -- machine TIMER IRQ enable
            csr.mie_mei  <= csr.wdata(11); -- machine EXT IRQ enable
            csr.mie_firq <= csr.wdata(31 downto 16); -- fast interrupt channels 0..15

          when csr_mtvec_c => -- machine trap-handler base address
            csr.mtvec <= csr.wdata(XLEN-1 downto 2) & "00"; -- mtvec.MODE=0

          when csr_mcounteren_c => -- machine counter access enable
            if (CPU_EXTENSION_RISCV_U = true) then
              if (CPU_EXTENSION_RISCV_Zicntr = true) then
                csr.mcounteren(0) <= csr.wdata(0);
                csr.mcounteren(2) <= csr.wdata(2);
              end if;
              if (CPU_EXTENSION_RISCV_Zihpm = true) then -- any HPMs available?
                csr.mcounteren(XLEN-1 downto 3) <= csr.wdata(XLEN-1 downto 3);
              end if;
            end if;

          -- machine trap handling --
          -- --------------------------------------------------------------------
          when csr_mscratch_c => -- machine scratch register
            csr.mscratch <= csr.wdata;

          when csr_mepc_c => -- machine exception program counter
            csr.mepc <= csr.wdata;

          when csr_mcause_c => -- machine trap cause
            csr.mcause <= csr.wdata(31) & csr.wdata(4 downto 0); -- type (exception/interrupt) & identifier

          when csr_mip_c => -- machine interrupt pending
            csr.mip_firq_nclr <= csr.wdata(31 downto 16); -- set low to clear according bit (FIRQs only)

          -- machine counter setup --
          -- --------------------------------------------------------------------
          when csr_mcountinhibit_c => -- machine counter-inhibit register
            if (CPU_EXTENSION_RISCV_Zicntr = true) then
              csr.mcountinhibit(0) <= csr.wdata(0); -- inhibit auto-increment of [m]cycle[h] counter
              csr.mcountinhibit(2) <= csr.wdata(2); -- inhibit auto-increment of [m]instret[h] counter
            end if;
            if (CPU_EXTENSION_RISCV_Zihpm = true) then -- any HPMs available?
              csr.mcountinhibit(XLEN-1 downto 3) <= csr.wdata(XLEN-1 downto 3); -- inhibit auto-increment of [m]hpmcounter*[h] counter
            end if;

          -- debug mode CSRs --
          -- --------------------------------------------------------------------
          when csr_dcsr_c => -- debug mode control and status register
            if (CPU_EXTENSION_RISCV_Sdext = true) then
              csr.dcsr_ebreakm <= csr.wdata(15);
              csr.dcsr_step    <= csr.wdata(2);
              if (CPU_EXTENSION_RISCV_U = true) then -- user mode implemented
                csr.dcsr_ebreaku <= csr.wdata(12);
                csr.dcsr_prv     <= csr.wdata(1) or csr.wdata(0); -- everything /= U will fall back to M
              end if;
            end if;

          when csr_dpc_c => -- debug mode program counter
            if (CPU_EXTENSION_RISCV_Sdext = true) then
            csr.dpc <= csr.wdata(XLEN-1 downto 1) & '0';
              end if;

          when csr_dscratch0_c => -- debug mode scratch register 0
            if (CPU_EXTENSION_RISCV_Sdext = true) then
              csr.dscratch0 <= csr.wdata;
            end if;

          -- trigger module CSRs --
          -- --------------------------------------------------------------------
          when csr_tdata1_c => -- match control
            if (CPU_EXTENSION_RISCV_Sdtrig = true) then
              csr.tdata1_exe    <= csr.wdata(2);
              csr.tdata1_action <= csr.wdata(12);
              csr.tdata1_dmode  <= csr.wdata(27);
            end if;

          when csr_tdata2_c => -- address compare
            if (CPU_EXTENSION_RISCV_Sdtrig = true) then
              csr.tdata2 <= csr.wdata(XLEN-1 downto 1) & '0';
            end if;

          -- not implemented (or coded somewhere else) --
          -- --------------------------------------------------------------------
          when others => NULL;

        end case;


      -- ********************************************************************************
      -- Automatic CSR access by hardware
      -- ********************************************************************************
      else

        -- --------------------------------------------------------------------
        -- floating-point (FPU) exception flags
        -- --------------------------------------------------------------------
        if (CPU_EXTENSION_RISCV_Zfinx = true) and (trap_ctrl.exc_buf(exc_iillegal_c) = '0') then -- no illegal instruction
          csr.fflags <= csr.fflags or fpu_flags_i; -- accumulate flags ("accrued exception flags")
        end if;

        -- --------------------------------------------------------------------
        -- TRAP ENTER
        -- --------------------------------------------------------------------
        if (trap_ctrl.env_start_ack = '1') then -- trap handler starting?

          -- NORMAL trap entry: write mcause, mepc and mtval - no update when in debug-mode! --
          -- --------------------------------------------------------------------
          if (CPU_EXTENSION_RISCV_Sdext = false) or ((trap_ctrl.cause(5) = '0') and (debug_ctrl.running = '0')) then
            -- trap cause --
            csr.mcause <= trap_ctrl.cause(trap_ctrl.cause'left) & trap_ctrl.cause(4 downto 0); -- type & identifier
            -- trap PC --
            csr.mepc <= trap_ctrl.epc(XLEN-1 downto 1) & '0';
            -- trap value --
            case trap_ctrl.cause is
              when trap_lma_c | trap_laf_c | trap_sma_c | trap_saf_c => -- misaligned load/store address or load/store access error
                csr.mtval <= mar_i; -- faulting data access address
              when trap_iil_c => -- illegal instruction
                csr.mtval <= execute_engine.ir; -- faulting instruction word
              when others => -- everything else including all interrupts
                csr.mtval <= (others => '0');
            end case;
            -- update privilege level and interrupt enable stack --
            csr.privilege    <= priv_mode_m_c; -- execute trap in machine mode
            csr.mstatus_mie  <= '0'; -- disable interrupts
            csr.mstatus_mpie <= csr.mstatus_mie; -- backup previous mie state
            csr.mstatus_mpp  <= csr.privilege; -- backup previous privilege mode
          end if;

          -- DEBUG MODE entry: write dpc and dcsr - no update when already in debug-mode! --
          -- --------------------------------------------------------------------
          if (CPU_EXTENSION_RISCV_Sdext = true) and (trap_ctrl.cause(5) = '1') and (debug_ctrl.running = '0') then
            -- trap cause --
            csr.dcsr_cause <= trap_ctrl.cause(2 downto 0); -- why did we enter debug mode?
            -- current privilege mode when debug mode was entered --
            csr.dcsr_prv <= csr.privilege;
            -- trap PC --
            csr.dpc <= trap_ctrl.epc(XLEN-1 downto 1) & '0';
          end if;

        -- --------------------------------------------------------------------
        -- TRAP EXIT
        -- --------------------------------------------------------------------
        elsif (trap_ctrl.env_end = '1') then

          -- return from debug mode --
          if (CPU_EXTENSION_RISCV_Sdext = true) and (debug_ctrl.running = '1') then
            if (CPU_EXTENSION_RISCV_U = true) then
              csr.privilege <= csr.dcsr_prv;
              if (csr.dcsr_prv /= priv_mode_m_c) then
                csr.mstatus_mprv <= '0'; -- clear if return priv. mode is less than M
              end if;
            end if;

          -- return from "normal trap" --
          else
            if (CPU_EXTENSION_RISCV_U = true) then
              csr.privilege   <= csr.mstatus_mpp; -- restore previous privilege mode
              csr.mstatus_mpp <= priv_mode_u_c; -- set to least-privileged mode that is supported
              if (csr.mstatus_mpp /= priv_mode_m_c) then
                csr.mstatus_mprv <= '0'; -- clear if return priv. mode is less than M
              end if;
            end if;
            csr.mstatus_mie  <= csr.mstatus_mpie; -- restore global IRQ enable flag
            csr.mstatus_mpie <= '1';
          end if;

        end if; -- /trap exit

      end if; -- /hardware csr access


      -- ********************************************************************************
      -- Override - hardwire unimplemented registers to all-zero
      -- ********************************************************************************

      -- hardwired bits --
      csr.mcounteren(1)    <= '0'; -- time[h] not implemented
      csr.mcountinhibit(1) <= '0'; -- time[h] not implemented

      -- no FPU --
      if (CPU_EXTENSION_RISCV_Zfinx = false) then
        csr.frm    <= (others => '0');
        csr.fflags <= (others => '0');
      end if;

      -- no base counters --
      if (CPU_EXTENSION_RISCV_Zicntr = false) then
        csr.mcounteren(2 downto 0)    <= (others => '0');
        csr.mcountinhibit(2 downto 0) <= (others => '0');
      end if;

      -- no hardware performance monitors --
      if (CPU_EXTENSION_RISCV_Zihpm = false) then
        csr.mcounteren(XLEN-1 downto 3)    <= (others => '0');
        csr.mcountinhibit(XLEN-1 downto 3) <= (others => '0');
      end if;

      -- no user mode --
      if (CPU_EXTENSION_RISCV_U = false) then
        csr.privilege     <= priv_mode_m_c;
        csr.mstatus_mprv  <= '0';
        csr.mstatus_tw    <= '0';
        csr.mstatus_mpp   <= priv_mode_m_c;
        csr.dcsr_ebreaku  <= '0';
        csr.dcsr_prv      <= '0';
        csr.mcounteren    <= (others => '0');
      end if;

      -- no debug mode --
      if (CPU_EXTENSION_RISCV_Sdext = false) then
        csr.dcsr_ebreakm <= '0';
        csr.dcsr_step    <= '0';
        csr.dcsr_ebreaku <= '0';
        csr.dcsr_prv     <= priv_mode_m_c;
        csr.dcsr_cause   <= (others => '0');
        csr.dpc          <= (others => '0');
        csr.dscratch0    <= (others => '0');
      end if;

      -- no trigger module--
      if (CPU_EXTENSION_RISCV_Sdtrig = false) then
        csr.tdata1_exe    <= '0';
        csr.tdata1_action <= '0';
        csr.tdata1_dmode  <= '0';
        csr.tdata2        <= (others => '0');
      end if;

    end if;
  end process csr_write_access;

  -- effective privilege mode is MACHINE when in debug mode --
  csr.privilege_eff <= priv_mode_m_c when (CPU_EXTENSION_RISCV_Sdext = true) and (debug_ctrl.running = '1') else csr.privilege;


  -- Physical Memory Protection (PMP) CSRs --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- write enable decoder --
  pmp_we: process(csr)
  begin
    -- Configuration registers --
    pmp.we_cfg <= (others => '0');
    if (csr.addr(11 downto 2) = csr_pmpcfg0_c(11 downto 2)) and (csr.we = '1') then
      pmp.we_cfg(to_integer(unsigned(csr.addr(1 downto 0)))) <= '1';
    end if;
    -- Address registers --
    pmp.we_addr <= (others => '0');
    if (csr.addr(11 downto 4) = csr_pmpaddr0_c(11 downto 4)) and (csr.we = '1') then
      pmp.we_addr(to_integer(unsigned(csr.addr(3 downto 0)))) <= '1';
    end if;
  end process pmp_we;

  -- PMP registers --
  pmp_reg_gen:
  for i in 0 to PMP_NUM_REGIONS-1 generate
    pmp_reg: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        pmp.cfg(i)  <= (others => '0');
        pmp.addr(i) <= (others => '0');
      elsif rising_edge(clk_i) then
        -- configuration --
        if (pmp.we_cfg(i/4) = '1') and (pmp.cfg(i)(7) = '0') then -- unlocked write access
          pmp.cfg(i)(2 downto 0) <= csr.wdata((i mod 4)*8+2 downto (i mod 4)*8+0); -- X (execute), W (write), R (read)
          if (PMP_MIN_GRANULARITY > 4) and (csr.wdata((i mod 4)*8+4 downto (i mod 4)*8+3) = pmp_mode_na4_c) then
            pmp.cfg(i)(4 downto 3) <= pmp_mode_off_c; -- NA4 not available, fall back to OFF
          else
            pmp.cfg(i)(4 downto 3) <= csr.wdata((i mod 4)*8+4 downto (i mod 4)*8+3); -- A (mode)
          end if;
          pmp.cfg(i)(6 downto 5) <= "00"; -- reserved
          pmp.cfg(i)(7) <= csr.wdata((i mod 4)*8+7); -- L (locked)
        end if;
        -- address --
        if (pmp.we_addr(i) = '1') and (pmp.cfg(i)(7) = '0') then -- unlocked write access
          if (i < PMP_NUM_REGIONS-1) then
            if (pmp.cfg(i+1)(7) = '0') or (pmp.cfg(i+1)(4 downto 3) /= pmp_mode_tor_c) then -- cfg(i+1) not "LOCKED TOR"
              pmp.addr(i) <= "00" & csr.wdata(XLEN-3 downto 0);
            end if;
          else -- very last entry
            pmp.addr(i) <= "00" & csr.wdata(XLEN-3 downto 0);
          end if;
        end if;
      end if;
    end process pmp_reg;
  end generate;

  -- PMP output to bus unit and CSR read-back --
  pmp_connect: process(pmp)
  begin
    pmp_ctrl_o  <= (others => (others => '0'));
    pmp_addr_o  <= (others => (others => '0'));
    pmp_cfg_rd  <= (others => (others => '0'));
    pmp_addr_rd <= (others => (others => '0'));
    for i in 0 to PMP_NUM_REGIONS-1 loop
      pmp_ctrl_o(i) <= pmp.cfg(i);
      pmp_addr_o(i) <= pmp.addr(i) & "00"; -- word aligned address
      pmp_cfg_rd(i/4)(8*(i mod 4)+7 downto 8*(i mod 4)+0) <= pmp.cfg(i);
      pmp_addr_rd(i)(XLEN-1 downto index_size_f(PMP_MIN_GRANULARITY)-2) <= pmp.addr(i)(XLEN-1 downto index_size_f(PMP_MIN_GRANULARITY)-2);
      if (PMP_MIN_GRANULARITY = 8) then -- bit [G-1] reads as zero in TOR or OFF mode
        if (pmp.cfg(i)(4) = '0') then -- TOR/OFF
          pmp_addr_rd(i)(index_size_f(PMP_MIN_GRANULARITY)-1) <= '0';
        end if;
      elsif (PMP_MIN_GRANULARITY > 8) then
        -- in NAPOT mode, bits [G-2:0] must read as one
        pmp_addr_rd(i)(index_size_f(PMP_MIN_GRANULARITY)-2 downto 0) <= (others => '1');
        -- in TOR or OFF mode, bits [G-1:0] must read as zero
        if (pmp.cfg(i)(4) = '0') then -- TOR/OFF
          pmp_addr_rd(i)(index_size_f(PMP_MIN_GRANULARITY)-1 downto 0) <= (others => '0');
        end if;
      end if;
    end loop;
  end process pmp_connect;


  -- Hardware Performance Monitors (HMP) Counter Event Configuration CSRs -------------------
  -- -------------------------------------------------------------------------------------------
  -- write enable decoder --
  hpmevent_we: process(csr)
  begin
    hpmevent.we <= (others => '0');
    if (csr.addr(11 downto 5) = csr_cnt_setup_c) and (csr.we = '1') then
      hpmevent.we(to_integer(unsigned(csr.addr(4 downto 0)))) <= '1';
    end if;
  end process hpmevent_we;

  -- HPM event registers --
  hpmevent_reg_gen:
  for i in 0 to HPM_NUM_CNTS-1 generate
    hpmevent_reg: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        hpmevent.cfg(i) <= (others => '0');
      elsif rising_edge(clk_i) then
        if (hpmevent.we(3+i) = '1') then
          hpmevent.cfg(i) <= csr.wdata(hpmcnt_event_size_c-1 downto 0);
        end if;
      end if;
    end process hpmevent_reg;
  end generate;

  -- HPM event CSR read-back --
  hpm_event_connect: process(hpmevent)
  begin
    hpmevent_rd <= (others => (others => '0'));
    for i in 3 to (HPM_NUM_CNTS+3)-1 loop
      hpmevent_rd(i)(hpmcnt_event_size_c-1 downto 0) <= hpmevent.cfg(i-3);
      hpmevent_rd(i)(hpmcnt_event_tm_c)              <= '0'; -- time, unused/reserved
    end loop;
  end process hpm_event_connect;


  -- Control and Status Registers - Read Access ---------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      csr.re    <= csr.re_nxt; -- read access?
      csr.rdata <= (others => '0'); -- default output, unimplemented CSRs/CSR bits read as zero
      case csr_raddr is

        -- hardware-only CSRs --
        -- --------------------------------------------------------------------
--      when csr_zero_c => -- zero: always returns zero, only relevant for hardware-access, not visible to ISA
--        csr.rdata <= (others => '0');

        -- floating-point CSRs --
        -- --------------------------------------------------------------------
        when csr_fflags_c => -- floating-point (FPU) exception flags
          if (CPU_EXTENSION_RISCV_Zfinx) then csr.rdata(4 downto 0) <= csr.fflags; end if;

        when csr_frm_c => -- floating-point (FPU) rounding mode
          if (CPU_EXTENSION_RISCV_Zfinx) then csr.rdata(2 downto 0) <= csr.frm; end if;

        when csr_fcsr_c => -- floating-point (FPU) control/status (frm & fflags)
          if (CPU_EXTENSION_RISCV_Zfinx) then csr.rdata(7 downto 0) <= csr.frm & csr.fflags; end if;

        -- machine trap setup --
        -- --------------------------------------------------------------------
        when csr_mstatus_c => -- machine status register - low word
          csr.rdata(03) <= csr.mstatus_mie; -- MIE
          csr.rdata(07) <= csr.mstatus_mpie; -- MPIE
          csr.rdata(12 downto 11) <= (others => csr.mstatus_mpp); -- MPP: machine previous privilege mode
          csr.rdata(17) <= csr.mstatus_mprv;
          csr.rdata(21) <= csr.mstatus_tw and bool_to_ulogic_f(CPU_EXTENSION_RISCV_U); -- TW

--      when csr_mstatush_c => -- machine status register - high word, implemented but always zero
--        csr.rdata <= (others => '0');

        when csr_misa_c => -- ISA and extensions
          csr.rdata(01) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_B);     -- B CPU extension
          csr.rdata(02) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_C);     -- C CPU extension
          csr.rdata(04) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_E);     -- E CPU extension
          csr.rdata(08) <= not bool_to_ulogic_f(CPU_EXTENSION_RISCV_E); -- I CPU extension (if not E)
          csr.rdata(12) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_M);     -- M CPU extension
          csr.rdata(20) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_U);     -- U CPU extension
          csr.rdata(23) <= '1';                                         -- X CPU extension (non-standard extensions / NEORV32-specific)
          csr.rdata(30) <= '1'; -- 32-bit architecture (MXL lo)
          csr.rdata(31) <= '0'; -- 32-bit architecture (MXL hi)

        when csr_mie_c => -- machine interrupt-enable register
          csr.rdata(03) <= csr.mie_msi; -- machine software IRQ enable
          csr.rdata(07) <= csr.mie_mti; -- machine timer IRQ enable
          csr.rdata(11) <= csr.mie_mei; -- machine external IRQ enable
          csr.rdata(31 downto 16) <= csr.mie_firq;

        when csr_mtvec_c => --machine trap-handler base address (for ALL exceptions)
          csr.rdata <= csr.mtvec(XLEN-1 downto 2) & "00"; -- mtvec.MODE=0

        when csr_mcounteren_c => -- machine counter enable register
          if (CPU_EXTENSION_RISCV_U = true) then
            csr.rdata(0) <= csr.mcounteren(0); -- allow user-level access to cycle[h]
            csr.rdata(2) <= csr.mcounteren(2); -- allow user-level access to instret[h]
            if (CPU_EXTENSION_RISCV_Zihpm = true) and (HPM_NUM_CNTS > 0) then -- any HPMs implemented?
              csr.rdata((HPM_NUM_CNTS+3)-1 downto 3) <= csr.mcounteren((HPM_NUM_CNTS+3)-1 downto 3); -- allow user-level access to all available hpmcounter*[h] CSRs
            end if;
          end if;

        -- machine configuration --
        -- --------------------------------------------------------------------
--      when csr_menvcfg_c  => csr.rdata <= (others => '0'); -- hardwired to zero
--      when csr_menvcfgh_c => csr.rdata <= (others => '0'); -- hardwired to zero

        -- machine trap handling --
        -- --------------------------------------------------------------------
        when csr_mscratch_c => -- machine scratch register
          csr.rdata <= csr.mscratch;

        when csr_mepc_c => -- machine exception program counter
          csr.rdata <= csr.mepc(XLEN-1 downto 1) & '0';

        when csr_mcause_c => -- machine trap cause
          csr.rdata(31)         <= csr.mcause(5);
          csr.rdata(4 downto 0) <= csr.mcause(4 downto 0);

        when csr_mtval_c => -- machine bad address or instruction
          csr.rdata <= csr.mtval;

        when csr_mip_c => -- machine interrupt pending
          csr.rdata(03)           <= trap_ctrl.irq_pnd(irq_msi_irq_c);
          csr.rdata(07)           <= trap_ctrl.irq_pnd(irq_mti_irq_c);
          csr.rdata(11)           <= trap_ctrl.irq_pnd(irq_mei_irq_c);
          csr.rdata(31 downto 16) <= trap_ctrl.irq_pnd(irq_firq_15_c downto irq_firq_0_c);

        -- physical memory protection --
        -- --------------------------------------------------------------------
        -- region configuration --
        when csr_pmpcfg0_c | csr_pmpcfg1_c | csr_pmpcfg2_c | csr_pmpcfg3_c =>
          if (PMP_NUM_REGIONS > 0) then csr.rdata <= pmp_cfg_rd(to_integer(unsigned(csr.addr(1 downto 0)))); end if;

        -- region address --
        when csr_pmpaddr0_c  | csr_pmpaddr1_c  | csr_pmpaddr2_c  | csr_pmpaddr3_c  |
             csr_pmpaddr4_c  | csr_pmpaddr5_c  | csr_pmpaddr6_c  | csr_pmpaddr7_c  |
             csr_pmpaddr8_c  | csr_pmpaddr9_c  | csr_pmpaddr10_c | csr_pmpaddr11_c |
             csr_pmpaddr12_c | csr_pmpaddr13_c | csr_pmpaddr14_c | csr_pmpaddr15_c =>
          if (PMP_NUM_REGIONS > 0) then csr.rdata <= pmp_addr_rd(to_integer(unsigned(csr.addr(3 downto 0)))); end if;

        -- machine counter setup --
        -- --------------------------------------------------------------------
        when csr_mcountinhibit_c => -- machine counter-inhibit register
          if (CPU_EXTENSION_RISCV_Zicntr = true) then
            csr.rdata(0) <= csr.mcountinhibit(0); -- inhibit [m]cycle[h] counter
            csr.rdata(2) <= csr.mcountinhibit(2); -- inhibit [m]instret[h] counter
          end if;
          if (CPU_EXTENSION_RISCV_Zihpm = true) and (HPM_NUM_CNTS > 0) then -- any HPMs implemented?
            csr.rdata((HPM_NUM_CNTS+3)-1 downto 3) <= csr.mcountinhibit((HPM_NUM_CNTS+3)-1 downto 3); -- inhibit [m]hpmcounter*[h] counter
          end if;

        -- HPM event select --
        when csr_mhpmevent3_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 00) then csr.rdata <= hpmevent_rd(03); end if;
        when csr_mhpmevent4_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 01) then csr.rdata <= hpmevent_rd(04); end if;
        when csr_mhpmevent5_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 02) then csr.rdata <= hpmevent_rd(05); end if;
        when csr_mhpmevent6_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 03) then csr.rdata <= hpmevent_rd(06); end if;
        when csr_mhpmevent7_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 04) then csr.rdata <= hpmevent_rd(07); end if;
        when csr_mhpmevent8_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 05) then csr.rdata <= hpmevent_rd(08); end if;
        when csr_mhpmevent9_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 06) then csr.rdata <= hpmevent_rd(09); end if;
        when csr_mhpmevent10_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 07) then csr.rdata <= hpmevent_rd(10); end if;
        when csr_mhpmevent11_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 08) then csr.rdata <= hpmevent_rd(11); end if;
        when csr_mhpmevent12_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 09) then csr.rdata <= hpmevent_rd(12); end if;
        when csr_mhpmevent13_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 10) then csr.rdata <= hpmevent_rd(13); end if;
        when csr_mhpmevent14_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 11) then csr.rdata <= hpmevent_rd(14); end if;
        when csr_mhpmevent15_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 12) then csr.rdata <= hpmevent_rd(15); end if;
        when csr_mhpmevent16_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 13) then csr.rdata <= hpmevent_rd(16); end if;
        when csr_mhpmevent17_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 14) then csr.rdata <= hpmevent_rd(17); end if;
        when csr_mhpmevent18_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 15) then csr.rdata <= hpmevent_rd(18); end if;
        when csr_mhpmevent19_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 16) then csr.rdata <= hpmevent_rd(19); end if;
        when csr_mhpmevent20_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 17) then csr.rdata <= hpmevent_rd(20); end if;
        when csr_mhpmevent21_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 18) then csr.rdata <= hpmevent_rd(21); end if;
        when csr_mhpmevent22_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 19) then csr.rdata <= hpmevent_rd(22); end if;
        when csr_mhpmevent23_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 20) then csr.rdata <= hpmevent_rd(23); end if;
        when csr_mhpmevent24_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 21) then csr.rdata <= hpmevent_rd(24); end if;
        when csr_mhpmevent25_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 22) then csr.rdata <= hpmevent_rd(25); end if;
        when csr_mhpmevent26_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 23) then csr.rdata <= hpmevent_rd(26); end if;
        when csr_mhpmevent27_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 24) then csr.rdata <= hpmevent_rd(27); end if;
        when csr_mhpmevent28_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 25) then csr.rdata <= hpmevent_rd(28); end if;
        when csr_mhpmevent29_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 26) then csr.rdata <= hpmevent_rd(29); end if;
        when csr_mhpmevent30_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 27) then csr.rdata <= hpmevent_rd(30); end if;
        when csr_mhpmevent31_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 28) then csr.rdata <= hpmevent_rd(31); end if;

        -- counters and timers --
        -- --------------------------------------------------------------------
        -- low word --
        when csr_mcycle_c        | csr_cycle_c        => if (CPU_EXTENSION_RISCV_Zicntr) then csr.rdata <= cnt_lo_rd(00); end if;
        when csr_minstret_c      | csr_instret_c      => if (CPU_EXTENSION_RISCV_Zicntr) then csr.rdata <= cnt_lo_rd(02); end if;
        when csr_mhpmcounter3_c  | csr_hpmcounter3_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 00) then csr.rdata <= cnt_lo_rd(03); end if;
        when csr_mhpmcounter4_c  | csr_hpmcounter4_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 01) then csr.rdata <= cnt_lo_rd(04); end if;
        when csr_mhpmcounter5_c  | csr_hpmcounter5_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 02) then csr.rdata <= cnt_lo_rd(05); end if;
        when csr_mhpmcounter6_c  | csr_hpmcounter6_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 03) then csr.rdata <= cnt_lo_rd(06); end if;
        when csr_mhpmcounter7_c  | csr_hpmcounter7_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 04) then csr.rdata <= cnt_lo_rd(07); end if;
        when csr_mhpmcounter8_c  | csr_hpmcounter8_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 05) then csr.rdata <= cnt_lo_rd(08); end if;
        when csr_mhpmcounter9_c  | csr_hpmcounter9_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 06) then csr.rdata <= cnt_lo_rd(09); end if;
        when csr_mhpmcounter10_c | csr_hpmcounter10_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 07) then csr.rdata <= cnt_lo_rd(10); end if;
        when csr_mhpmcounter11_c | csr_hpmcounter11_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 08) then csr.rdata <= cnt_lo_rd(11); end if;
        when csr_mhpmcounter12_c | csr_hpmcounter12_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 09) then csr.rdata <= cnt_lo_rd(12); end if;
        when csr_mhpmcounter13_c | csr_hpmcounter13_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 10) then csr.rdata <= cnt_lo_rd(13); end if;
        when csr_mhpmcounter14_c | csr_hpmcounter14_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 11) then csr.rdata <= cnt_lo_rd(14); end if;
        when csr_mhpmcounter15_c | csr_hpmcounter15_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 12) then csr.rdata <= cnt_lo_rd(15); end if;
        when csr_mhpmcounter16_c | csr_hpmcounter16_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 13) then csr.rdata <= cnt_lo_rd(16); end if;
        when csr_mhpmcounter17_c | csr_hpmcounter17_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 14) then csr.rdata <= cnt_lo_rd(17); end if;
        when csr_mhpmcounter18_c | csr_hpmcounter18_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 15) then csr.rdata <= cnt_lo_rd(18); end if;
        when csr_mhpmcounter19_c | csr_hpmcounter19_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 16) then csr.rdata <= cnt_lo_rd(19); end if;
        when csr_mhpmcounter20_c | csr_hpmcounter20_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 17) then csr.rdata <= cnt_lo_rd(20); end if;
        when csr_mhpmcounter21_c | csr_hpmcounter21_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 18) then csr.rdata <= cnt_lo_rd(21); end if;
        when csr_mhpmcounter22_c | csr_hpmcounter22_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 19) then csr.rdata <= cnt_lo_rd(22); end if;
        when csr_mhpmcounter23_c | csr_hpmcounter23_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 20) then csr.rdata <= cnt_lo_rd(23); end if;
        when csr_mhpmcounter24_c | csr_hpmcounter24_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 21) then csr.rdata <= cnt_lo_rd(24); end if;
        when csr_mhpmcounter25_c | csr_hpmcounter25_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 22) then csr.rdata <= cnt_lo_rd(25); end if;
        when csr_mhpmcounter26_c | csr_hpmcounter26_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 23) then csr.rdata <= cnt_lo_rd(26); end if;
        when csr_mhpmcounter27_c | csr_hpmcounter27_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 24) then csr.rdata <= cnt_lo_rd(27); end if;
        when csr_mhpmcounter28_c | csr_hpmcounter28_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 25) then csr.rdata <= cnt_lo_rd(28); end if;
        when csr_mhpmcounter29_c | csr_hpmcounter29_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 26) then csr.rdata <= cnt_lo_rd(29); end if;
        when csr_mhpmcounter30_c | csr_hpmcounter30_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 27) then csr.rdata <= cnt_lo_rd(30); end if;
        when csr_mhpmcounter31_c | csr_hpmcounter31_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 28) then csr.rdata <= cnt_lo_rd(31); end if;

        -- high word --
        when csr_mcycleh_c        | csr_cycleh_c        => if (CPU_EXTENSION_RISCV_Zicntr) then csr.rdata <= cnt_hi_rd(00); end if;
        when csr_minstreth_c      | csr_instreth_c      => if (CPU_EXTENSION_RISCV_Zicntr) then csr.rdata <= cnt_hi_rd(02); end if;
        when csr_mhpmcounter3h_c  | csr_hpmcounter3h_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 00) then csr.rdata <= cnt_hi_rd(03); end if;
        when csr_mhpmcounter4h_c  | csr_hpmcounter4h_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 01) then csr.rdata <= cnt_hi_rd(04); end if;
        when csr_mhpmcounter5h_c  | csr_hpmcounter5h_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 02) then csr.rdata <= cnt_hi_rd(05); end if;
        when csr_mhpmcounter6h_c  | csr_hpmcounter6h_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 03) then csr.rdata <= cnt_hi_rd(06); end if;
        when csr_mhpmcounter7h_c  | csr_hpmcounter7h_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 04) then csr.rdata <= cnt_hi_rd(07); end if;
        when csr_mhpmcounter8h_c  | csr_hpmcounter8h_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 05) then csr.rdata <= cnt_hi_rd(08); end if;
        when csr_mhpmcounter9h_c  | csr_hpmcounter9h_c  => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 06) then csr.rdata <= cnt_hi_rd(09); end if;
        when csr_mhpmcounter10h_c | csr_hpmcounter10h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 07) then csr.rdata <= cnt_hi_rd(10); end if;
        when csr_mhpmcounter11h_c | csr_hpmcounter11h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 08) then csr.rdata <= cnt_hi_rd(11); end if;
        when csr_mhpmcounter12h_c | csr_hpmcounter12h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 09) then csr.rdata <= cnt_hi_rd(12); end if;
        when csr_mhpmcounter13h_c | csr_hpmcounter13h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 10) then csr.rdata <= cnt_hi_rd(13); end if;
        when csr_mhpmcounter14h_c | csr_hpmcounter14h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 11) then csr.rdata <= cnt_hi_rd(14); end if;
        when csr_mhpmcounter15h_c | csr_hpmcounter15h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 12) then csr.rdata <= cnt_hi_rd(15); end if;
        when csr_mhpmcounter16h_c | csr_hpmcounter16h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 13) then csr.rdata <= cnt_hi_rd(16); end if;
        when csr_mhpmcounter17h_c | csr_hpmcounter17h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 14) then csr.rdata <= cnt_hi_rd(17); end if;
        when csr_mhpmcounter18h_c | csr_hpmcounter18h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 15) then csr.rdata <= cnt_hi_rd(18); end if;
        when csr_mhpmcounter19h_c | csr_hpmcounter19h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 16) then csr.rdata <= cnt_hi_rd(19); end if;
        when csr_mhpmcounter20h_c | csr_hpmcounter20h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 17) then csr.rdata <= cnt_hi_rd(20); end if;
        when csr_mhpmcounter21h_c | csr_hpmcounter21h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 18) then csr.rdata <= cnt_hi_rd(21); end if;
        when csr_mhpmcounter22h_c | csr_hpmcounter22h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 19) then csr.rdata <= cnt_hi_rd(22); end if;
        when csr_mhpmcounter23h_c | csr_hpmcounter23h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 20) then csr.rdata <= cnt_hi_rd(23); end if;
        when csr_mhpmcounter24h_c | csr_hpmcounter24h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 21) then csr.rdata <= cnt_hi_rd(24); end if;
        when csr_mhpmcounter25h_c | csr_hpmcounter25h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 22) then csr.rdata <= cnt_hi_rd(25); end if;
        when csr_mhpmcounter26h_c | csr_hpmcounter26h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 23) then csr.rdata <= cnt_hi_rd(26); end if;
        when csr_mhpmcounter27h_c | csr_hpmcounter27h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 24) then csr.rdata <= cnt_hi_rd(27); end if;
        when csr_mhpmcounter28h_c | csr_hpmcounter28h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 25) then csr.rdata <= cnt_hi_rd(28); end if;
        when csr_mhpmcounter29h_c | csr_hpmcounter29h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 26) then csr.rdata <= cnt_hi_rd(29); end if;
        when csr_mhpmcounter30h_c | csr_hpmcounter30h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 27) then csr.rdata <= cnt_hi_rd(30); end if;
        when csr_mhpmcounter31h_c | csr_hpmcounter31h_c => if (CPU_EXTENSION_RISCV_Zihpm) and (HPM_NUM_CNTS > 28) then csr.rdata <= cnt_hi_rd(31); end if;

        -- machine information registers --
        -- --------------------------------------------------------------------
        when csr_mvendorid_c  => csr.rdata <= VENDOR_ID; -- vendor's JEDEC ID
        when csr_marchid_c    => csr.rdata(4 downto 0) <= "10011"; -- architecture ID - official RISC-V open-source arch ID
        when csr_mimpid_c     => csr.rdata <= hw_version_c; -- implementation ID -- NEORV32 hardware version
        when csr_mhartid_c    => csr.rdata <= HART_ID; -- hardware thread ID
--      when csr_mconfigptr_c => csr.rdata <= (others => '0'); -- machine configuration pointer register, implemented but always zero

        -- debug mode CSRs --
        -- --------------------------------------------------------------------
        when csr_dcsr_c      => if (CPU_EXTENSION_RISCV_Sdext) then csr.rdata <= csr.dcsr_rd;   end if; -- debug mode control and status
        when csr_dpc_c       => if (CPU_EXTENSION_RISCV_Sdext) then csr.rdata <= csr.dpc;       end if; -- debug mode program counter
        when csr_dscratch0_c => if (CPU_EXTENSION_RISCV_Sdext) then csr.rdata <= csr.dscratch0; end if; -- debug mode scratch register 0

        -- trigger module CSRs --
        -- --------------------------------------------------------------------
--      when csr_tselect_c  => if (CPU_EXTENSION_RISCV_Sdtrig) then csr.rdata <= (others => '0'); end if; -- always zero = only 1 trigger available
        when csr_tdata1_c   => if (CPU_EXTENSION_RISCV_Sdtrig) then csr.rdata <= csr.tdata1_rd;   end if; -- match control
        when csr_tdata2_c   => if (CPU_EXTENSION_RISCV_Sdtrig) then csr.rdata <= csr.tdata2;      end if; -- address-compare
--      when csr_tdata3_c   => if (CPU_EXTENSION_RISCV_Sdtrig) then csr.rdata <= (others => '0'); end if; -- implemented but always zero
        when csr_tinfo_c    => if (CPU_EXTENSION_RISCV_Sdtrig) then csr.rdata <= x"00000004";     end if; -- address-match trigger only
--      when csr_tcontrol_c => if (CPU_EXTENSION_RISCV_Sdtrig) then csr.rdata <= (others => '0'); end if; -- implemented but always zero
--      when csr_mcontext_c => if (CPU_EXTENSION_RISCV_Sdtrig) then csr.rdata <= (others => '0'); end if; -- implemented but always zero
--      when csr_scontext_c => if (CPU_EXTENSION_RISCV_Sdtrig) then csr.rdata <= (others => '0'); end if; -- implemented but always zero

        -- NEORV32-specific (RISC-V "custom") read-only CSRs --
        -- --------------------------------------------------------------------
        -- machine extended ISA extensions information --
        when csr_mxisa_c =>
          -- extended ISA (sub-)extensions --
          csr.rdata(00) <= '1';                                            -- Zicsr: CSR access (always enabled)
          csr.rdata(01) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zifencei); -- Zifencei: instruction stream sync.
          csr.rdata(02) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zmmul);    -- Zmmul: mul/div
          csr.rdata(03) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zxcfu);    -- Zxcfu: custom RISC-V instructions
          csr.rdata(04) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicond);   -- Zicond: conditional operations
          csr.rdata(05) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zfinx);    -- Zfinx: FPU using x registers
--        csr.rdata(06) <= '0'; -- reserved
          csr.rdata(07) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicntr);   -- Zicntr: base counters
          csr.rdata(08) <= bool_to_ulogic_f(boolean(PMP_NUM_REGIONS > 0)); -- PMP: physical memory protection (Zspmp)
          csr.rdata(09) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zihpm);    -- Zihpm: hardware performance monitors
          csr.rdata(10) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Sdext);    -- Sdext: RISC-V (external) debug mode
          csr.rdata(11) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Sdtrig);   -- Sdtrig: trigger module
          -- misc --
          csr.rdata(20) <= bool_to_ulogic_f(is_simulation_c);              -- is this a simulation?
          -- tuning options --
          csr.rdata(30) <= bool_to_ulogic_f(FAST_MUL_EN);                  -- DSP-based multiplication (M extensions only)
          csr.rdata(31) <= bool_to_ulogic_f(FAST_SHIFT_EN);                -- parallel logic for shifts (barrel shifters)

        -- undefined/unavailable --
        -- --------------------------------------------------------------------
        when others => NULL; -- not implemented, read as zero

      end case;
    end if;
  end process csr_read_access;

  -- AND-gate CSR read address: csr.rdata is zero if csr.re is not set --
  -- [WARNING] M-mode (9:8 = 11) and U-mode (9:8 = 00) CSRs only!
  csr_raddr <= (csr.addr(11 downto 10) & csr.addr(8) & csr.addr(8) & csr.addr(7 downto 0)) when (csr.re = '1') else (others => '0');

  -- CSR read data output --
  csr_rdata_o <= csr.rdata;


-- ****************************************************************************************************************************
-- CPU Counters (Standard Counters and Hardware Performance Monitors)
-- ****************************************************************************************************************************

  -- Counter CSRs ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- write enable decoder --
  cnt_we: process(csr)
  begin
    cnt.we_lo <= (others => '0');
    cnt.we_hi <= (others => '0');
    -- no need to check bits 6:5 of the address as they're always zero (checked by illegal CSR logic)
    if (csr.we = '1') and (csr.addr(11 downto 8) = csr_mcycle_c(11 downto 8)) then
      if (csr.addr(7) = '0') then -- low word
        cnt.we_lo(to_integer(unsigned(csr.addr(4 downto 0)))) <= '1';
      else -- high word
        cnt.we_hi(to_integer(unsigned(csr.addr(4 downto 0)))) <= '1';
      end if;
    end if;
  end process cnt_we;

  -- hardware counters --
  cpu_counter_gen:
  for i in 0 to 31 generate
    -- counter CSRs --
    cnt_regs: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        cnt.lo(i)  <= (others => '0');
        cnt.ovf(i) <= (others => '0');
        cnt.hi(i)  <= (others => '0');
      elsif rising_edge(clk_i) then
        -- low word --
        if (cnt.we_lo(i) = '1') then
          cnt.lo(i) <= csr.wdata;
        else
          cnt.lo(i) <= cnt.nxt(i)(XLEN-1 downto 0);
        end if;
        cnt.ovf(i)(0) <= cnt.nxt(i)(XLEN);
        -- high word --
        if (cnt.we_hi(i) = '1') then
          cnt.hi(i) <= csr.wdata;
        else
          cnt.hi(i) <= std_ulogic_vector(unsigned(cnt.hi(i)) + unsigned(cnt.ovf(i)));
        end if;
      end if;
    end process cnt_regs;

    -- low-word increment --
    cnt.nxt(i) <= std_ulogic_vector(unsigned('0' & cnt.lo(i)) + 1) when (cnt.inc(i) = '1') else
                  std_ulogic_vector(unsigned('0' & cnt.lo(i)) + 0);
  end generate;

  -- counter CSR read-back --
  cnt_connect: process(cnt)
  begin
    cnt_lo_rd <= (others => (others => '0'));
    cnt_hi_rd <= (others => (others => '0'));
    -- basic counters --
    if (CPU_EXTENSION_RISCV_Zicntr = true) then
      cnt_lo_rd(0) <= cnt.lo(0); -- cycle
      cnt_hi_rd(0) <= cnt.hi(0); -- cycleh
      cnt_lo_rd(2) <= cnt.lo(2); -- instret
      cnt_hi_rd(2) <= cnt.hi(2); -- instreth
    end if;
    -- hpm counters --
    if (CPU_EXTENSION_RISCV_Zihpm = true) then
      for i in 0 to HPM_NUM_CNTS-1 loop
        if (hpm_cnt_lo_width_c > 0) then
          cnt_lo_rd(3+i)(hpm_cnt_lo_width_c-1 downto 0) <= cnt.lo(3+i)(hpm_cnt_lo_width_c-1 downto 0);
        end if;
        if (hpm_cnt_hi_width_c > 0) then
          cnt_hi_rd(3+i)(hpm_cnt_hi_width_c-1 downto 0) <= cnt.hi(3+i)(hpm_cnt_hi_width_c-1 downto 0);
        end if;
      end loop;
    end if;
  end process cnt_connect;


  -- Counter Increment Control (Trigger Events) ---------------------------------------------
  -- -------------------------------------------------------------------------------------------
  counter_event: process(clk_i)
  begin
    -- increment if any enabled event fires, do not increment if CPU is in debug mode or if counter is inhibited
    if rising_edge(clk_i) then
      cnt.inc <= (others => '0'); -- default
      -- base counters --
      cnt.inc(0) <= cnt_event(hpmcnt_event_cy_c) and (not csr.mcountinhibit(0)) and (not debug_ctrl.running); -- cycle
      cnt.inc(2) <= cnt_event(hpmcnt_event_ir_c) and (not csr.mcountinhibit(2)) and (not debug_ctrl.running); -- instret
      -- hpm counters --
      for i in 0 to HPM_NUM_CNTS-1 loop
        cnt.inc(3+i) <= or_reduce_f(cnt_event and hpmevent.cfg(i)) and (not csr.mcountinhibit(3+i)) and (not debug_ctrl.running);
      end loop;
    end if;
  end process counter_event;

  -- RISC-V-specific basic counter events (for HPM and base counters) --
  cnt_event(hpmcnt_event_cy_c) <= '0' when (execute_engine.state = CPU_SLEEP) else '1'; -- active cycle
  cnt_event(hpmcnt_event_tm_c) <= '0'; -- time (unused/reserved)
  cnt_event(hpmcnt_event_ir_c) <= '1' when (execute_engine.state = EXECUTE) else '0'; -- retired instruction

  -- NEORV32-specific counter events (for HPM counters only) --
  cnt_event(hpmcnt_event_cir_c)     <= '1' when (execute_engine.state = EXECUTE)    and (execute_engine.is_ci      = '1')        else '0'; -- executed compressed instruction
  cnt_event(hpmcnt_event_wait_if_c) <= '1' when (fetch_engine.state   = IF_PENDING) and (fetch_engine.state_prev   = IF_PENDING) else '0'; -- instruction fetch memory wait cycle
  cnt_event(hpmcnt_event_wait_ii_c) <= '1' when (execute_engine.state = DISPATCH)   and (execute_engine.state_prev = DISPATCH)   else '0'; -- instruction issue wait cycle
  cnt_event(hpmcnt_event_wait_mc_c) <= '1' when (execute_engine.state = ALU_WAIT)                                                else '0'; -- multi-cycle alu-operation wait cycle

  cnt_event(hpmcnt_event_load_c)    <= '1' when (ctrl.bus_req = '1')              and (execute_engine.ir(instr_opcode_msb_c-1) = '0') else '0'; -- load operation
  cnt_event(hpmcnt_event_store_c)   <= '1' when (ctrl.bus_req = '1')              and (execute_engine.ir(instr_opcode_msb_c-1) = '1') else '0'; -- store operation
  cnt_event(hpmcnt_event_wait_ls_c) <= '1' when (execute_engine.state = MEM_WAIT) and (execute_engine.state_prev2 = MEM_WAIT)         else '0'; -- load/store memory wait cycle

  cnt_event(hpmcnt_event_jump_c)    <= '1' when (execute_engine.state = BRANCH)   and (execute_engine.ir(instr_opcode_lsb_c+2) = '1') else '0'; -- jump (unconditional)
  cnt_event(hpmcnt_event_branch_c)  <= '1' when (execute_engine.state = BRANCH)   and (execute_engine.ir(instr_opcode_lsb_c+2) = '0') else '0'; -- branch (conditional, taken or not taken)
  cnt_event(hpmcnt_event_tbranch_c) <= '1' when (execute_engine.state = BRANCHED) and (execute_engine.state_prev = BRANCH) and
                                                                                      (execute_engine.ir(instr_opcode_lsb_c+2) = '0') else '0'; -- taken branch (conditional)

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
      debug_ctrl.ext_halt_req <= '0';
      debug_ctrl.state        <= DEBUG_OFFLINE;
    elsif rising_edge(clk_i) then
      if (CPU_EXTENSION_RISCV_Sdext = true) then
        debug_ctrl.ext_halt_req <= db_halt_req_i; -- external halt request (from Debug Module)
        case debug_ctrl.state is -- state machine

          when DEBUG_OFFLINE => -- waiting to start debug mode
            if (trap_ctrl.env_start_ack = '1') and (trap_ctrl.cause(5) = '1') then -- processing trap entry into debug mode
              debug_ctrl.state <= DEBUG_ONLINE;
            end if;

          when DEBUG_ONLINE => -- we are in debug mode
            if (debug_ctrl.dret = '1') then -- DRET instruction
              debug_ctrl.state <= DEBUG_LEAVING;
            end if;

          when DEBUG_LEAVING => -- leaving debug mode
            if (execute_engine.state = TRAP_EXECUTE) then -- processing trap exit (updating PC and status registers)
              debug_ctrl.state <= DEBUG_OFFLINE;
            end if;

          when others => -- undefined
            debug_ctrl.state <= DEBUG_OFFLINE;

        end case;
      else -- debug mode not implemented
        debug_ctrl.ext_halt_req <= '0';
        debug_ctrl.state        <= DEBUG_OFFLINE;
      end if;
    end if;
  end process debug_control;

  -- CPU is in debug mode --
  debug_ctrl.running <= '0' when (CPU_EXTENSION_RISCV_Sdext = false) or (debug_ctrl.state = DEBUG_OFFLINE) else '1';

  -- debug mode entry triggers --
  debug_ctrl.trig_hw    <= hw_trigger_fire and (not debug_ctrl.running) and csr.tdata1_action and csr.tdata1_dmode; -- enter debug mode by HW trigger module request (only valid if dmode = 1)
  debug_ctrl.trig_break <= trap_ctrl.break_point and (debug_ctrl.running or -- re-enter debug mode
                           ((    csr.privilege) and csr.dcsr_ebreakm) or    -- enabled goto-debug-mode in machine mode on "ebreak"
                           ((not csr.privilege) and csr.dcsr_ebreaku));     -- enabled goto-debug-mode in user mode on "ebreak"
  debug_ctrl.trig_halt  <= debug_ctrl.ext_halt_req and (not debug_ctrl.running); -- external halt request (if not halted already)
  debug_ctrl.trig_step  <= csr.dcsr_step and (not debug_ctrl.running); -- single-step mode (trigger when NOT CURRENTLY in debug mode)


  -- Debug Control and Status Register (dcsr) - Read-Back -----------------------------------
  -- -------------------------------------------------------------------------------------------
  csr.dcsr_rd(31 downto 28) <= "0100"; -- xdebugver: external debug support compatible to spec. version 1.0
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
  csr.dcsr_rd(04)           <= '1'; -- mprven: mstatus.mprv is also evaluated in debug mode
  csr.dcsr_rd(03)           <= '0'; -- nmip: no pending non-maskable interrupt
  csr.dcsr_rd(02)           <= csr.dcsr_step; -- step: single-step mode
  csr.dcsr_rd(01 downto 00) <= (others => csr.dcsr_prv); -- prv: privilege mode when debug mode was entered


-- ****************************************************************************************************************************
-- Hardware Trigger Module (Part of the On-Chip Debugger)
-- ****************************************************************************************************************************

  -- trigger to enter debug-mode: instruction address match (fire AFTER execution) --
  hw_trigger_fire <= '1' when (CPU_EXTENSION_RISCV_Sdtrig = true) and (csr.tdata1_exe = '1') and
                              (csr.tdata2(XLEN-1 downto 1) = execute_engine.pc(XLEN-1 downto 1)) and
                              (execute_engine.state = EXECUTE) else '0';


  -- Match Control CSR (mcontrol @ tdata1) - Read-Back --------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr.tdata1_rd(31 downto 28) <= "0010"; -- type: address(/data) match trigger
  csr.tdata1_rd(27)           <= csr.tdata1_dmode; -- dmode: set to ignore machine-mode access to tdata* CSRs
  csr.tdata1_rd(26 downto 21) <= "000000"; -- maskmax: only exact values
  csr.tdata1_rd(20)           <= '0'; -- hit: feature not implemented
  csr.tdata1_rd(19)           <= '0'; -- select: fire on address match
  csr.tdata1_rd(18)           <= '1'; -- timing: trigger **after** executing the triggering instruction
  csr.tdata1_rd(17 downto 16) <= "00"; -- sizelo: match against an access of any size
  csr.tdata1_rd(15 downto 12) <= "000" & csr.tdata1_action; -- action: 1: enter debug mode on trigger, 0: ebreak exception on trigger
  csr.tdata1_rd(11)           <= '0'; -- chain: chaining not supported - there is only one trigger
  csr.tdata1_rd(10 downto 07) <= "0000"; -- match: only full-address-match
  csr.tdata1_rd(6)            <= '1'; -- m: trigger enabled when in machine mode
  csr.tdata1_rd(5)            <= '0'; -- h: hypervisor mode not supported
  csr.tdata1_rd(4)            <= '0'; -- s: supervisor mode not supported
  csr.tdata1_rd(3)            <= '1' when (CPU_EXTENSION_RISCV_U = true) else '0'; -- u: trigger enabled when in user mode
  csr.tdata1_rd(2)            <= csr.tdata1_exe; -- execute: enable trigger
  csr.tdata1_rd(1)            <= '0'; -- store: store address or data matching not supported
  csr.tdata1_rd(0)            <= '0'; -- load: load address or data matching not supported


end neorv32_cpu_control_rtl;
