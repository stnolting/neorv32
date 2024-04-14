-- ================================================================================ --
-- NEORV32 CPU - Central Control Unit                                               --
-- -------------------------------------------------------------------------------- --
-- CPU operations are controlled by several "engines" (modules). These engines      --
-- operate in parallel to implement a tiny 2-stage pipeline:                        --
-- + Fetch engine:    Fetches 32-bit chunks of instruction words (pipeline stage 1) --
-- + Issue engine:    Decodes RVC instructions, aligns & queues instruction words   --
-- + Execute engine:  Multi-cycle execution of instructions (pipeline stage 2)      --
-- + Trap controller: Handles interrupts and exceptions                             --
-- + CSR module:      Read/write access to control and status registers             --
-- + CPU counters:    Base and HPM counters                                         --
-- + Debug module:    CPU debug mode handling (on-chip debugger)                    --
-- + Trigger module:  Hardware-assisted breakpoints (on-chip debugger)              --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cpu_control is
  generic (
    -- General --
    HART_ID                    : std_ulogic_vector(31 downto 0); -- hardware thread ID
    VENDOR_ID                  : std_ulogic_vector(31 downto 0); -- vendor's JEDEC ID
    CPU_BOOT_ADDR              : std_ulogic_vector(31 downto 0); -- cpu boot address
    CPU_DEBUG_PARK_ADDR        : std_ulogic_vector(31 downto 0); -- cpu debug mode parking loop entry address, 4-byte aligned
    CPU_DEBUG_EXC_ADDR         : std_ulogic_vector(31 downto 0); -- cpu debug mode exception entry address, 4-byte aligned
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A      : boolean; -- implement atomic memory operations extension?
    CPU_EXTENSION_RISCV_B      : boolean; -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_C      : boolean; -- implement compressed extension?
    CPU_EXTENSION_RISCV_E      : boolean; -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M      : boolean; -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U      : boolean; -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx  : boolean; -- implement 32-bit floating-point extension (using INT regs)
    CPU_EXTENSION_RISCV_Zicntr : boolean; -- implement base counters?
    CPU_EXTENSION_RISCV_Zicond : boolean; -- implement integer conditional operations?
    CPU_EXTENSION_RISCV_Zihpm  : boolean; -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zmmul  : boolean; -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zxcfu  : boolean; -- implement custom (instr.) functions unit?
    CPU_EXTENSION_RISCV_Sdext  : boolean; -- implement external debug mode extension?
    CPU_EXTENSION_RISCV_Sdtrig : boolean; -- implement trigger module extension?
    CPU_EXTENSION_RISCV_Smpmp  : boolean; -- implement physical memory protection?
    -- Tuning Options --
    FAST_MUL_EN                : boolean; -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN              : boolean; -- use barrel shifter for shift operations
    REGFILE_HW_RST             : boolean; -- implement full hardware reset for register file
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS               : natural range 0 to 13; -- number of implemented HPM counters (0..13)
    HPM_CNT_WIDTH              : natural range 0 to 64  -- total size of HPM counters (0..64)
  );
  port (
    -- global control --
    clk_i         : in  std_ulogic; -- global clock, rising edge
    clk_aux_i     : in  std_ulogic; -- always-on clock, rising edge
    rstn_i        : in  std_ulogic; -- global reset, low-active, async
    ctrl_o        : out ctrl_bus_t; -- main control bus
    -- instruction fetch interface --
    i_pmp_fault_i : in  std_ulogic; -- instruction fetch pmp fault
    bus_req_o     : out bus_req_t;  -- request
    bus_rsp_i     : in  bus_rsp_t;  -- response
    -- data path interface --
    alu_cp_done_i : in  std_ulogic; -- ALU iterative operation done
    cmp_i         : in  std_ulogic_vector(1 downto 0); -- comparator status
    alu_add_i     : in  std_ulogic_vector(XLEN-1 downto 0); -- ALU address result
    rs1_i         : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 1
    imm_o         : out std_ulogic_vector(XLEN-1 downto 0); -- immediate
    fetch_pc_o    : out std_ulogic_vector(XLEN-1 downto 0); -- instruction fetch address
    curr_pc_o     : out std_ulogic_vector(XLEN-1 downto 0); -- current PC (corresponding to current instruction)
    link_pc_o     : out std_ulogic_vector(XLEN-1 downto 0); -- link PC (return address)
    csr_rdata_o   : out std_ulogic_vector(XLEN-1 downto 0); -- CSR read data
    -- external CSR interface --
    xcsr_we_o     : out std_ulogic; -- global write enable
    xcsr_addr_o   : out std_ulogic_vector(11 downto 0); -- address
    xcsr_wdata_o  : out std_ulogic_vector(XLEN-1 downto 0); -- write data
    xcsr_rdata_i  : in  std_ulogic_vector(XLEN-1 downto 0); -- read data
    -- interrupts --
    db_halt_req_i : in  std_ulogic; -- debug mode (halt) request
    msi_i         : in  std_ulogic; -- machine software interrupt
    mei_i         : in  std_ulogic; -- machine external interrupt
    mti_i         : in  std_ulogic; -- machine timer interrupt
    firq_i        : in  std_ulogic_vector(15 downto 0); -- fast interrupts
    -- data access interface --
    lsu_wait_i    : in  std_ulogic; -- wait for data bus
    mar_i         : in  std_ulogic_vector(XLEN-1 downto 0); -- memory address register
    ma_load_i     : in  std_ulogic; -- misaligned load data address
    ma_store_i    : in  std_ulogic; -- misaligned store data address
    be_load_i     : in  std_ulogic; -- bus error on load data access
    be_store_i    : in  std_ulogic  -- bus error on store data access
  );
end neorv32_cpu_control;

architecture neorv32_cpu_control_rtl of neorv32_cpu_control is

  -- HPM counter auto-configuration --
  constant hpm_num_c          : natural := cond_sel_natural_f(CPU_EXTENSION_RISCV_Zihpm, HPM_NUM_CNTS, 0);
  constant hpm_cnt_lo_width_c : natural := cond_sel_natural_f(boolean(HPM_CNT_WIDTH < 32), HPM_CNT_WIDTH, 32); -- width low word
  constant hpm_cnt_hi_width_c : natural := natural(cond_sel_int_f(boolean(HPM_CNT_WIDTH > 32), HPM_CNT_WIDTH-32, 0)); -- width high word

  -- instruction fetch engine --
  type fetch_engine_state_t is (IF_RESTART, IF_REQUEST, IF_PENDING);
  type fetch_engine_t is record
    state   : fetch_engine_state_t;
    restart : std_ulogic; -- buffered restart request (after branch)
    pc      : std_ulogic_vector(XLEN-1 downto 0);
    reset   : std_ulogic; -- restart request (after branch)
    resp    : std_ulogic; -- bus response
    priv    : std_ulogic; -- fetch privilege level
  end record;
  signal fetch_engine : fetch_engine_t;

  -- instruction prefetch buffer (FIFO) interface --
  type ipb_data_t is array (0 to 1) of std_ulogic_vector(16 downto 0); -- bus_error & 16-bit instruction
  type ipb_t is record
    wdata, rdata : ipb_data_t;
    we,    re    : std_ulogic_vector(1 downto 0);
    free,  avail : std_ulogic_vector(1 downto 0);
  end record;
  signal ipb : ipb_t;

  -- instruction issue engine --
  type issue_engine_t is record
    align     : std_ulogic;
    align_set : std_ulogic;
    align_clr : std_ulogic;
    ci_i16    : std_ulogic_vector(15 downto 0);
    ci_i32    : std_ulogic_vector(31 downto 0);
    data      : std_ulogic_vector((2+32)-1 downto 0); -- is_compressed & bus_error & 32-bit instruction
    valid     : std_ulogic_vector(1 downto 0); -- data word is valid
    ack       : std_ulogic;
  end record;
  signal issue_engine : issue_engine_t;

  -- instruction decoding helper logic --
  type decode_aux_t is record
    opcode    : std_ulogic_vector(6 downto 0);
    is_a_lr   : std_ulogic;
    is_a_sc   : std_ulogic;
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
  type execute_engine_state_t is (DISPATCH, TRAP_ENTER, TRAP_EXIT, RESTART, FENCE, SLEEP,
                                  EXECUTE, ALU_WAIT, BRANCH, BRANCHED, SYSTEM, MEM_REQ, MEM_WAIT);
  type execute_engine_t is record
    state        : execute_engine_state_t;
    state_nxt    : execute_engine_state_t;
    ir           : std_ulogic_vector(31 downto 0);
    ir_nxt       : std_ulogic_vector(31 downto 0);
    is_ci        : std_ulogic; -- current instruction is de-compressed instruction
    is_ci_nxt    : std_ulogic;
    branch_taken : std_ulogic; -- branch condition fulfilled
    pc           : std_ulogic_vector(XLEN-1 downto 0); -- actual PC, corresponding to current executed instruction
    pc_we        : std_ulogic; -- PC update enabled
    next_pc      : std_ulogic_vector(XLEN-1 downto 0); -- next PC, corresponding to next instruction to be executed
    next_pc_inc  : std_ulogic_vector(XLEN-1 downto 0); -- increment to get next PC
    link_pc      : std_ulogic_vector(XLEN-1 downto 0); -- next PC for linking (return address)
  end record;
  signal execute_engine : execute_engine_t;

  -- execution monitor --
  type monitor_t is record
    cnt     : std_ulogic_vector(monitor_mc_tmo_c downto 0);
    cnt_add : std_ulogic_vector(monitor_mc_tmo_c downto 0);
    exc     : std_ulogic;
  end record;
  signal monitor : monitor_t;

  -- CPU sleep-mode --
  signal sleep_mode : std_ulogic;

  -- trap controller --
  type trap_ctrl_t is record
    exc_buf     : std_ulogic_vector(exc_width_c-1 downto 0); -- synchronous exception buffer (one bit per exception)
    exc_fire    : std_ulogic; -- set if there is a valid source in the exception buffer
    irq_pnd     : std_ulogic_vector(irq_width_c-1 downto 0); -- pending interrupt
    irq_buf     : std_ulogic_vector(irq_width_c-1 downto 0); -- asynchronous exception/interrupt buffer (one bit per interrupt source)
    irq_fire    : std_ulogic; -- set if an interrupt is actually kicking in
    cause       : std_ulogic_vector(6 downto 0); -- trap ID for mcause CSR & debug-mode entry identifier
    epc         : std_ulogic_vector(XLEN-1 downto 0); -- exception program counter
    --
    env_pending : std_ulogic; -- start of trap environment if pending
    env_enter   : std_ulogic; -- enter trap environment
    env_exit    : std_ulogic; -- leave trap environment
    wakeup      : std_ulogic; -- wakeup from sleep due to an enabled pending IRQ
    --
    instr_be    : std_ulogic; -- instruction fetch bus error
    instr_ma    : std_ulogic; -- instruction fetch misaligned address
    instr_il    : std_ulogic; -- illegal instruction
    ecall       : std_ulogic; -- ecall instruction
    ebreak      : std_ulogic; -- ebreak instruction
    hwtrig      : std_ulogic; -- hardware trigger module
  end record;
  signal trap_ctrl : trap_ctrl_t;

  -- CPU main control bus --
  signal ctrl, ctrl_nxt : ctrl_bus_t;

  -- control and status registers (CSRs) --
  type csr_t is record
    addr           : std_ulogic_vector(11 downto 0); -- csr address
    raddr          : std_ulogic_vector(11 downto 0); -- simplified csr read address
    we, we_nxt     : std_ulogic; -- csr write enable
    re, re_nxt     : std_ulogic; -- csr read enable
    wdata, rdata   : std_ulogic_vector(XLEN-1 downto 0); -- csr write/read data
    --
    mstatus_mie    : std_ulogic; -- machine-mode IRQ enable
    mstatus_mpie   : std_ulogic; -- previous machine-mode IRQ enable
    mstatus_mpp    : std_ulogic; -- machine previous privilege mode
    mstatus_mprv   : std_ulogic; -- effective privilege level for load/stores
    mstatus_tw     : std_ulogic; -- do not allow user mode to execute WFI instruction when set
    --
    mie_msi        : std_ulogic; -- machine software interrupt enable
    mie_mei        : std_ulogic; -- machine external interrupt enable
    mie_mti        : std_ulogic; -- machine timer interrupt enable
    mie_firq       : std_ulogic_vector(15 downto 0); -- fast interrupt enable
    --
    privilege      : std_ulogic; -- current privilege mode
    privilege_eff  : std_ulogic; -- current *effective* privilege mode
    --
    mepc           : std_ulogic_vector(XLEN-1 downto 0); -- machine exception PC
    mcause         : std_ulogic_vector(5 downto 0); -- machine trap cause
    mtvec          : std_ulogic_vector(XLEN-1 downto 0); -- machine trap-handler base address
    mtval          : std_ulogic_vector(XLEN-1 downto 0); -- machine bad address or instruction
    mtinst         : std_ulogic_vector(XLEN-1 downto 0); -- machine trap instruction
    mscratch       : std_ulogic_vector(XLEN-1 downto 0); -- machine scratch register
    mcounteren     : std_ulogic; -- machine counter access enable (from user-mode) for ALL counters
    mcountinhibit  : std_ulogic_vector(15 downto 0); -- inhibit counter auto-increment
    --
    dcsr_ebreakm   : std_ulogic; -- behavior of ebreak instruction in m-mode
    dcsr_ebreaku   : std_ulogic; -- behavior of ebreak instruction in u-mode
    dcsr_step      : std_ulogic; -- single-step mode
    dcsr_prv       : std_ulogic; -- current privilege level when entering debug mode
    dcsr_cause     : std_ulogic_vector(2 downto 0); -- why was debug mode entered
    dcsr_rd        : std_ulogic_vector(XLEN-1 downto 0); -- debug mode control and status register
    dpc            : std_ulogic_vector(XLEN-1 downto 0); -- mode program counter
    dscratch0      : std_ulogic_vector(XLEN-1 downto 0); -- debug mode scratch register 0
    --
    tdata1_execute : std_ulogic; -- enable instruction address match trigger
    tdata1_action  : std_ulogic; -- enter debug mode / ebreak exception when trigger fires
    tdata1_dmode   : std_ulogic; -- set to ignore tdata* CSR access from machine-mode
    tdata1_rd      : std_ulogic_vector(XLEN-1 downto 0); -- trigger register read-back
    tdata2         : std_ulogic_vector(XLEN-1 downto 0); -- address-match register
  end record;
  signal csr : csr_t;

  -- hpm event configuration CSRs --
  type hpmevent_cfg_t is array (3 to 15) of std_ulogic_vector(hpmcnt_event_size_c-1 downto 0);
  type hpmevent_rd_t  is array (3 to 15) of std_ulogic_vector(XLEN-1 downto 0);
  signal hpmevent_cfg : hpmevent_cfg_t;
  signal hpmevent_rd  : hpmevent_rd_t;
  signal hpmevent_we  : std_ulogic_vector(15 downto 0);

  -- counter CSRs --
  type cnt_dat_t is array (0 to 2+hpm_num_c) of std_ulogic_vector(XLEN-1 downto 0);
  type cnt_nxt_t is array (0 to 2+hpm_num_c) of std_ulogic_vector(XLEN downto 0);
  type cnt_ovf_t is array (0 to 2+hpm_num_c) of std_ulogic_vector(0 downto 0);
  type cnt_t is record
    we_lo : std_ulogic_vector(15 downto 0);
    we_hi : std_ulogic_vector(15 downto 0);
    inc   : std_ulogic_vector(15 downto 0);
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
  type debug_ctrl_t is record
    running    : std_ulogic; -- CPU is in debug mode
    trig_hw    : std_ulogic; -- hardware trigger
    trig_break : std_ulogic; -- ebreak instruction trigger
    trig_halt  : std_ulogic; -- external request trigger
    trig_step  : std_ulogic; -- single-stepping mode trigger
  end record;
  signal debug_ctrl : debug_ctrl_t;

  -- illegal instruction check --
  signal illegal_cmd : std_ulogic;

  -- CSR access/privilege/read-write check --
  signal csr_reg_valid  : std_ulogic; -- CSR implemented at all
  signal csr_rw_valid   : std_ulogic; -- valid r/w access rights
  signal csr_priv_valid : std_ulogic; -- valid access privilege

  -- hardware trigger module --
  signal hw_trigger_match, hw_trigger_fired : std_ulogic;

  -- CSR read-back helpers --
  signal csr_rdata, xcsr_rdata : std_ulogic_vector(XLEN-1 downto 0);

begin

-- ****************************************************************************************************************************
-- Instruction Fetch (always fetch 32-bit-aligned 32-bit chunks of data)
-- ****************************************************************************************************************************

  -- Fetch Engine FSM -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  fetch_engine_fsm: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      fetch_engine.state   <= IF_RESTART;
      fetch_engine.restart <= '1'; -- set to reset IPB
      fetch_engine.pc      <= CPU_BOOT_ADDR(XLEN-1 downto 2) & "00"; -- 32-bit aligned boot address
      fetch_engine.priv    <= priv_mode_m_c; -- start in machine mode
    elsif rising_edge(clk_i) then
      -- restart request --
      if (fetch_engine.state = IF_RESTART) then -- restart done
        fetch_engine.restart <= '0';
      else -- buffer request
        fetch_engine.restart <= fetch_engine.restart or fetch_engine.reset;
      end if;

      -- fsm --
      case fetch_engine.state is

        when IF_REQUEST => -- request next 32-bit-aligned instruction word
        -- ------------------------------------------------------------
          if (ipb.free = "11") then -- free IPB space?
            fetch_engine.state <= IF_PENDING;
          elsif (fetch_engine.restart = '1') or (fetch_engine.reset = '1') then -- restart because of branch
            fetch_engine.state <= IF_RESTART;
          end if;

        when IF_PENDING => -- wait for bus response and write instruction data to prefetch buffer
        -- ------------------------------------------------------------
          if (fetch_engine.resp = '1') then -- wait for bus response
            fetch_engine.pc    <= std_ulogic_vector(unsigned(fetch_engine.pc) + 4); -- next word
            fetch_engine.pc(1) <= '0'; -- (re-)align to 32-bit
            if (fetch_engine.restart = '1') or (fetch_engine.reset = '1') then -- restart request due to branch
              fetch_engine.state <= IF_RESTART;
            else -- request next linear instruction word
              fetch_engine.state <= IF_REQUEST;
            end if;
          end if;

        when others => -- IF_RESTART: set new start address
        -- ------------------------------------------------------------
          fetch_engine.pc    <= execute_engine.next_pc(XLEN-1 downto 1) & '0'; -- initialize from PC incl. 16-bit-alignment bit
          fetch_engine.priv  <= csr.privilege_eff; -- set new privilege level
          fetch_engine.state <= IF_REQUEST;

      end case;
    end if;
  end process fetch_engine_fsm;

  -- PC output for instruction fetch --
  bus_req_o.addr <= fetch_engine.pc(XLEN-1 downto 2) & "00"; -- word aligned
  fetch_pc_o     <= fetch_engine.pc(XLEN-1 downto 2) & "00"; -- word aligned

  -- instruction fetch (read) request if IPB not full --
  bus_req_o.stb <= '1' when (fetch_engine.state = IF_REQUEST) and (ipb.free = "11") else '0';

  -- instruction bus response --
  fetch_engine.resp <= bus_rsp_i.ack or bus_rsp_i.err;

  -- IPB instruction data and status --
  ipb.wdata(0) <= (bus_rsp_i.err or i_pmp_fault_i) & bus_rsp_i.data(15 downto 00);
  ipb.wdata(1) <= (bus_rsp_i.err or i_pmp_fault_i) & bus_rsp_i.data(31 downto 16);

  -- IPB write enable --
  ipb.we(0) <= '1' when (fetch_engine.state = IF_PENDING) and (fetch_engine.resp = '1') and
                        ((fetch_engine.pc(1) = '0') or (CPU_EXTENSION_RISCV_C = false)) else '0';
  ipb.we(1) <= '1' when (fetch_engine.state = IF_PENDING) and (fetch_engine.resp = '1') else '0';

  -- bus access type --
  bus_req_o.priv  <= fetch_engine.priv; -- current effective privilege level
  bus_req_o.data  <= (others => '0'); -- read-only
  bus_req_o.ben   <= (others => '0'); -- read-only
  bus_req_o.rw    <= '0'; -- read-only
  bus_req_o.src   <= '1'; -- source = instruction fetch
  bus_req_o.rvso  <= '0'; -- cannot be a reservation set operation
  bus_req_o.fence <= ctrl.lsu_fence; -- fence(.i) operation, valid without STB being set


  -- Instruction Prefetch Buffer (FIFO) -----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  prefetch_buffer:
  for i in 0 to 1 generate -- low half-word + high half-word (incl. status bits)
    prefetch_buffer_inst: entity neorv32.neorv32_fifo
    generic map (
      FIFO_DEPTH => 2,                   -- number of fifo entries; has to be a power of two, min 2
      FIFO_WIDTH => ipb.wdata(i)'length, -- size of data elements in fifo
      FIFO_RSYNC => false,               -- we NEED to read data asynchronously
      FIFO_SAFE  => false,               -- no safe access required (ensured by FIFO-external logic)
      FULL_RESET => false                -- no HW reset, try to infer BRAM
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

  -- Compressed Instructions Decoder --------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_decompressor_inst_true:
  if CPU_EXTENSION_RISCV_C generate
    neorv32_cpu_decompressor_inst: entity neorv32.neorv32_cpu_decompressor
    port map (
      ci_instr16_i => issue_engine.ci_i16,
      ci_instr32_o => issue_engine.ci_i32
    );
  end generate;

  neorv32_cpu_decompressor_inst_false:
  if not CPU_EXTENSION_RISCV_C generate
    issue_engine.ci_i32 <= (others => '0');
  end generate;

  -- half-word select --
  issue_engine.ci_i16 <= ipb.rdata(0)(15 downto 0) when (issue_engine.align = '0') else ipb.rdata(1)(15 downto 0);


  -- Issue Engine FSM (required if C extension is enabled) ----------------------------------
  -- -------------------------------------------------------------------------------------------
  issue_engine_enabled:
  if CPU_EXTENSION_RISCV_C generate

    issue_engine_fsm_sync: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        issue_engine.align <= '0'; -- start aligned after reset
      elsif rising_edge(clk_i) then
        if (fetch_engine.restart = '1') then
          issue_engine.align <= execute_engine.next_pc(1); -- branch to unaligned address?
        elsif (issue_engine.ack = '1') then
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
        if (ipb.rdata(0)(1 downto 0) /= "11") then -- compressed, use IPB(0) entry
          issue_engine.align_set <= ipb.avail(0); -- start of next instruction word is NOT 32-bit-aligned
          issue_engine.valid(0)  <= ipb.avail(0);
          issue_engine.data      <= '1' & ipb.rdata(0)(16) & issue_engine.ci_i32;
        else -- aligned uncompressed; use IPB(0) status flags only
          issue_engine.valid <= (others => (ipb.avail(0) and ipb.avail(1)));
          issue_engine.data  <= '0' & ipb.rdata(0)(16) & ipb.rdata(1)(15 downto 0) & ipb.rdata(0)(15 downto 0);
        end if;
      -- start with HIGH half-word --
      else
        if (ipb.rdata(1)(1 downto 0) /= "11") then -- compressed, use IPB(1) entry
          issue_engine.align_clr <= ipb.avail(1); -- start of next instruction word is 32-bit-aligned again
          issue_engine.valid(1)  <= ipb.avail(1);
          issue_engine.data      <= '1' & ipb.rdata(1)(16) & issue_engine.ci_i32;
        else -- unaligned uncompressed; use IPB(0) status flags only
          issue_engine.valid <= (others => (ipb.avail(0) and ipb.avail(1)));
          issue_engine.data  <= '0' & ipb.rdata(0)(16) & ipb.rdata(0)(15 downto 0) & ipb.rdata(1)(15 downto 0);
        end if;
      end if;
    end process issue_engine_fsm_comb;

  end generate; -- /issue_engine_enabled

  issue_engine_disabled: -- use IPB(0) status flags only
  if not CPU_EXTENSION_RISCV_C generate
    issue_engine.valid <= (others => ipb.avail(0));
    issue_engine.data  <= '0' & ipb.rdata(0)(16) & (ipb.rdata(1)(15 downto 0) & ipb.rdata(0)(15 downto 0));
  end generate; -- /issue_engine_disabled

  -- update IPB FIFOs --
  ipb.re(0) <= issue_engine.valid(0) and issue_engine.ack;
  ipb.re(1) <= issue_engine.valid(1) and issue_engine.ack;


-- ****************************************************************************************************************************
-- Instruction Execution
-- ****************************************************************************************************************************

  -- Immediate Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  imm_gen: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      imm_o <= (others => '0');
    elsif rising_edge(clk_i) then
      -- default I-immediate: ALU-immediate, load, jump-and-link with register --
      imm_o(XLEN-1 downto 11) <= (others => execute_engine.ir(31)); -- sign extension
      imm_o(10 downto 01)     <= execute_engine.ir(30 downto 21);
      imm_o(00)               <= execute_engine.ir(20);
      --
      case decode_aux.opcode is
        when opcode_store_c => -- S-immediate: store
          imm_o(XLEN-1 downto 11) <= (others => execute_engine.ir(31)); -- sign extension
          imm_o(10 downto 05)     <= execute_engine.ir(30 downto 25);
          imm_o(04 downto 00)     <= execute_engine.ir(11 downto 07);
        when opcode_branch_c => -- B-immediate: conditional branch
          imm_o(XLEN-1 downto 12) <= (others => execute_engine.ir(31)); -- sign extension
          imm_o(11)               <= execute_engine.ir(07);
          imm_o(10 downto 05)     <= execute_engine.ir(30 downto 25);
          imm_o(04 downto 01)     <= execute_engine.ir(11 downto 08);
          imm_o(00)               <= '0';
        when opcode_lui_c | opcode_auipc_c => -- U-immediate: lui, auipc
          imm_o(XLEN-1 downto 12) <= execute_engine.ir(31 downto 12);
          imm_o(11 downto 00)     <= (others => '0');
        when opcode_jal_c => -- J-immediate: unconditional jump
          imm_o(XLEN-1 downto 20) <= (others => execute_engine.ir(31)); -- sign extension
          imm_o(19 downto 12)     <= execute_engine.ir(19 downto 12);
          imm_o(11)               <= execute_engine.ir(20);
          imm_o(10 downto 01)     <= execute_engine.ir(30 downto 21);
          imm_o(00)               <= '0';
        when opcode_amo_c => -- atomic memory access
          if (CPU_EXTENSION_RISCV_A = true) then
            imm_o <= (others => '0');
          else
            NULL;
          end if;
        when others =>
          NULL;
      end case;
    end if;
  end process imm_gen;


  -- Branch Condition Check -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  branch_check: process(execute_engine.ir, cmp_i)
  begin
    if (execute_engine.ir(instr_opcode_lsb_c+2) = '0') then -- conditional branch
      if (execute_engine.ir(instr_funct3_msb_c) = '0') then -- beq / bne
        execute_engine.branch_taken <= cmp_i(cmp_equal_c) xor execute_engine.ir(instr_funct3_lsb_c);
      else -- blt(u) / bge(u)
        execute_engine.branch_taken <= cmp_i(cmp_less_c) xor execute_engine.ir(instr_funct3_lsb_c);
      end if;
    else -- unconditional branch
      execute_engine.branch_taken <= '1';
    end if;
  end process branch_check;


  -- Execute Engine FSM Sync ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  execute_engine_fsm_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl                   <= ctrl_bus_zero_c;
      execute_engine.state   <= RESTART;
      execute_engine.ir      <= (others => '0');
      execute_engine.is_ci   <= '0';
      execute_engine.pc      <= CPU_BOOT_ADDR(XLEN-1 downto 2) & "00"; -- 32-bit aligned boot address
      execute_engine.next_pc <= CPU_BOOT_ADDR(XLEN-1 downto 2) & "00"; -- 32-bit aligned boot address
      execute_engine.link_pc <= CPU_BOOT_ADDR(XLEN-1 downto 2) & "00"; -- 32-bit aligned boot address
    elsif rising_edge(clk_i) then
      -- control bus --
      ctrl <= ctrl_nxt;

      -- execute engine arbiter --
      execute_engine.state <= execute_engine.state_nxt;
      execute_engine.ir    <= execute_engine.ir_nxt;
      execute_engine.is_ci <= execute_engine.is_ci_nxt;

      -- current PC: address of instruction being executed --
      if (execute_engine.pc_we = '1') then
        execute_engine.pc <= execute_engine.next_pc(XLEN-1 downto 1) & '0';
      end if;

      -- link PC: return address --
      if (execute_engine.state = BRANCH) then
        execute_engine.link_pc <= execute_engine.next_pc(XLEN-1 downto 1) & '0';
      end if;

      -- next PC: address of next instruction --
      case execute_engine.state is

        when TRAP_ENTER => -- starting trap environment
          if (trap_ctrl.cause(5) = '1') and (CPU_EXTENSION_RISCV_Sdext = true) then -- debug mode (re-)entry
            execute_engine.next_pc <= CPU_DEBUG_PARK_ADDR(XLEN-1 downto 2) & "00"; -- debug mode enter; start at "parking loop" <normal_entry>
          elsif (debug_ctrl.running = '1') and (CPU_EXTENSION_RISCV_Sdext = true) then -- any other trap INSIDE debug mode
            execute_engine.next_pc <= CPU_DEBUG_EXC_ADDR(XLEN-1 downto 2) & "00"; -- debug mode enter: start at "parking loop" <exception_entry>
          else -- normal start of trap
            if (csr.mtvec(1 downto 0) = "01") and (trap_ctrl.cause(6) = '1') then -- vectored mode + interrupt
              execute_engine.next_pc <= csr.mtvec(XLEN-1 downto 7) & trap_ctrl.cause(4 downto 0) & "00"; -- pc = mtvec + 4 * mcause
            else
              execute_engine.next_pc <= csr.mtvec(XLEN-1 downto 2) & "00"; -- pc = mtvec
            end if;
          end if;

        when TRAP_EXIT => -- leaving trap environment
          if (debug_ctrl.running = '1') and (CPU_EXTENSION_RISCV_Sdext = true) then -- debug mode exit
            execute_engine.next_pc <= csr.dpc(XLEN-1 downto 1) & '0';
          else -- normal end of trap
            execute_engine.next_pc <= csr.mepc(XLEN-1 downto 1) & '0';
          end if;

        when BRANCH => -- control flow transfer
          if (trap_ctrl.exc_buf(exc_illegal_c) = '0') and (execute_engine.branch_taken = '1') then -- valid taken branch
            execute_engine.next_pc <= alu_add_i(XLEN-1 downto 1) & '0';
          end if;

        when EXECUTE => -- linear increment
          execute_engine.next_pc <= std_ulogic_vector(unsigned(execute_engine.pc) + unsigned(execute_engine.next_pc_inc));

        when others => -- no update
          NULL;

      end case;
    end if;
  end process execute_engine_fsm_sync;

  -- check if branch destination is misaligned --
  trap_ctrl.instr_ma <= '1' when (execute_engine.state = BRANCH) and (trap_ctrl.exc_buf(exc_illegal_c) = '0') and -- valid branch instruction
                                 (execute_engine.branch_taken = '1') and -- branch is taken
                                 (alu_add_i(1) = '1') and (CPU_EXTENSION_RISCV_C = false) else '0'; -- misaligned destination

  -- PC increment for next LINEAR instruction (+2 for compressed instr., +4 otherwise) --
  execute_engine.next_pc_inc(XLEN-1 downto 4) <= (others => '0');
  execute_engine.next_pc_inc(3 downto 0) <= x"4" when ((execute_engine.is_ci = '0') or (CPU_EXTENSION_RISCV_C = false)) else x"2";

  -- PC output --
  curr_pc_o <= execute_engine.pc(XLEN-1 downto 1) & '0'; -- current PC
  link_pc_o <= (execute_engine.link_pc(XLEN-1 downto 1) & '0') when (execute_engine.state = BRANCHED) else (others => '0'); -- return address


  -- Decoding Helper Logic ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  decode_helper: process(execute_engine)
  begin
    -- defaults --
    decode_aux.is_f_op   <= '0';
    decode_aux.is_a_lr   <= '0';
    decode_aux.is_a_sc   <= '0';
    decode_aux.is_m_mul  <= '0';
    decode_aux.is_m_div  <= '0';
    decode_aux.is_b_imm  <= '0';
    decode_aux.is_b_reg  <= '0';
    decode_aux.is_zicond <= '0';

    -- ATOMIC instructions --
    if (CPU_EXTENSION_RISCV_A = true) and -- implemented at all?
       (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = "010") and
       (execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c+3) = "0001") then
      decode_aux.is_a_lr <= not execute_engine.ir(instr_funct7_lsb_c+2); -- LR.W
      decode_aux.is_a_sc <=     execute_engine.ir(instr_funct7_lsb_c+2); -- SC.W
    end if;

    -- BITMANIP instruction --
    if (CPU_EXTENSION_RISCV_B = true) then -- implemented at all?
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
         ((execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000101") and (execute_engine.ir(instr_funct3_msb_c) = '1')) or -- MIN[U] / MAX[U]
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

    -- FLOATING-POINT instructions (Zfinx) --
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

    -- integer MUL (M/Zmmul) / DIV (M) instruction --
    if (execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000001") then
      if ((CPU_EXTENSION_RISCV_M = true) or (CPU_EXTENSION_RISCV_Zmmul = true)) and (execute_engine.ir(instr_funct3_msb_c) = '0') then
        decode_aux.is_m_mul <= '1';
      end if;
      if (CPU_EXTENSION_RISCV_M = true) and (execute_engine.ir(instr_funct3_msb_c) = '1') then
        decode_aux.is_m_div <= '1';
      end if;
    end if;

    -- CONDITIONAL instruction (Zicond) --
    if (CPU_EXTENSION_RISCV_Zicond = true) and (execute_engine.ir(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000111") and
       (execute_engine.ir(instr_funct3_msb_c) = '1') and (execute_engine.ir(instr_funct3_lsb_c) = '1') then
      decode_aux.is_zicond <= '1';
    end if;
  end process decode_helper;

  -- register/uimm5 checks --
  decode_aux.rs1_zero <= '1' when (execute_engine.ir(instr_rs1_msb_c downto instr_rs1_lsb_c) = "00000") else '0';
  decode_aux.rd_zero  <= '1' when (execute_engine.ir(instr_rd_msb_c  downto instr_rd_lsb_c ) = "00000") else '0';

  -- simplified rv32 opcode --
  decode_aux.opcode <= execute_engine.ir(instr_opcode_msb_c downto instr_opcode_lsb_c+2) & "11";


  -- Execute Engine FSM Comb ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  execute_engine_fsm_comb: process(execute_engine, debug_ctrl, trap_ctrl, hw_trigger_match, decode_aux, issue_engine, csr, alu_cp_done_i, lsu_wait_i)
  begin
    -- arbiter defaults --
    execute_engine.state_nxt <= execute_engine.state;
    execute_engine.ir_nxt    <= execute_engine.ir;
    execute_engine.is_ci_nxt <= execute_engine.is_ci;
    execute_engine.pc_we     <= '0';
    --
    issue_engine.ack         <= '0';
    --
    fetch_engine.reset       <= '0';
    --
    trap_ctrl.env_enter      <= '0';
    trap_ctrl.env_exit       <= '0';
    trap_ctrl.instr_be       <= '0';
    trap_ctrl.ecall          <= '0';
    trap_ctrl.ebreak         <= '0';
    trap_ctrl.hwtrig         <= '0';
    --
    csr.we_nxt               <= '0';
    csr.re_nxt               <= '0';
    --
    ctrl_nxt                 <= ctrl_bus_zero_c; -- all zero/off by default (default ALU operation = ZERO, adder.out = ADD)

    -- ALU sign control --
    if (execute_engine.ir(instr_opcode_lsb_c+4) = '1') then -- ALU ops
      ctrl_nxt.alu_unsigned <= execute_engine.ir(instr_funct3_lsb_c+0); -- unsigned ALU operation? (SLTIU, SLTU)
    else -- branches
      ctrl_nxt.alu_unsigned <= execute_engine.ir(instr_funct3_lsb_c+1); -- unsigned branches? (BLTU, BGEU)
    end if;

    -- ALU operand A: is PC? --
    case decode_aux.opcode is
      when opcode_auipc_c | opcode_jal_c | opcode_branch_c =>
        ctrl_nxt.alu_opa_mux <= '1';
      when others =>
        ctrl_nxt.alu_opa_mux <= '0';
    end case;

    -- ALU operand B: is immediate? --
    case decode_aux.opcode is
      when opcode_alui_c | opcode_lui_c | opcode_auipc_c | opcode_load_c | opcode_store_c | opcode_amo_c | opcode_branch_c | opcode_jal_c | opcode_jalr_c =>
        ctrl_nxt.alu_opb_mux <= '1';
      when others =>
        ctrl_nxt.alu_opb_mux <= '0';
    end case;

    -- memory read/write access --
    if (CPU_EXTENSION_RISCV_A = true) and (decode_aux.opcode(2) = opcode_amo_c(2)) then -- lr/sc
      ctrl_nxt.lsu_rw <= execute_engine.ir(instr_funct7_lsb_c+2);
    else -- normal load/store
      ctrl_nxt.lsu_rw <= execute_engine.ir(5);
    end if;

    -- state machine --
    case execute_engine.state is

      when DISPATCH => -- wait for ISSUE ENGINE to emit a valid instruction word
      -- ------------------------------------------------------------
        if (trap_ctrl.env_pending = '1') or (trap_ctrl.exc_fire = '1') then -- pending trap or pending exception (fast)
          execute_engine.state_nxt <= TRAP_ENTER;
        elsif (CPU_EXTENSION_RISCV_Sdtrig = true) and (hw_trigger_match = '1') then -- hardware breakpoint
          execute_engine.pc_we     <= '1'; -- pc <= next_pc
          trap_ctrl.hwtrig         <= '1';
          execute_engine.state_nxt <= DISPATCH; -- stay here another round until trap_ctrl.hwtrig arrives in trap_ctrl.env_pending
        elsif (issue_engine.valid(0) = '1') or (issue_engine.valid(1) = '1') then -- new instruction word available
          issue_engine.ack         <= '1';
          trap_ctrl.instr_be       <= issue_engine.data(32); -- access fault during instruction fetch
          execute_engine.is_ci_nxt <= issue_engine.data(33); -- this is a de-compressed instruction
          execute_engine.ir_nxt    <= issue_engine.data(31 downto 0); -- instruction word
          execute_engine.pc_we     <= '1'; -- pc <= next_pc
          execute_engine.state_nxt <= EXECUTE;
        end if;

      when TRAP_ENTER => -- enter trap environment and jump to trap vector
      -- ------------------------------------------------------------
        if (trap_ctrl.env_pending = '1') then -- wait for sync. exceptions to become pending
          trap_ctrl.env_enter      <= '1';
          execute_engine.state_nxt <= RESTART;
        end if;

      when TRAP_EXIT => -- return from trap environment and jump to xEPC
      -- ------------------------------------------------------------
        trap_ctrl.env_exit       <= '1';
        execute_engine.state_nxt <= RESTART;

      when RESTART => -- reset and restart instruction fetch at <next_pc>
      -- ------------------------------------------------------------
        fetch_engine.reset       <= '1';
        execute_engine.state_nxt <= BRANCHED;

      when EXECUTE => -- decode and execute instruction (control will be here for exactly 1 cycle in any case)
      -- [NOTE] register file is read in this stage; due to the sync read, data will be available in the _next_ state
      -- ------------------------------------------------------------
        case decode_aux.opcode is

          -- register/immediate ALU operation --
          when opcode_alu_c | opcode_alui_c =>

            -- ALU core operation --
            case execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) is
              when funct3_subadd_c              => ctrl_nxt.alu_op <= alu_op_add_c; -- ADD(I), SUB
              when funct3_slt_c | funct3_sltu_c => ctrl_nxt.alu_op <= alu_op_slt_c; -- SLT(I), SLTU(I)
              when funct3_xor_c                 => ctrl_nxt.alu_op <= alu_op_xor_c; -- XOR(I)
              when funct3_or_c                  => ctrl_nxt.alu_op <= alu_op_or_c;  -- OR(I)
              when others                       => ctrl_nxt.alu_op <= alu_op_and_c; -- AND(I)
            end case;

            -- addition/subtraction control --
            if (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c+1) = funct3_slt_c(2 downto 1)) or -- SLT(I), SLTU(I)
               ((execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_subadd_c) and
                (execute_engine.ir(instr_opcode_msb_c-1) = '1') and (execute_engine.ir(instr_funct7_msb_c-1) = '1')) then -- SUB
              ctrl_nxt.alu_sub <= '1';
            end if;

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
            -- EXT: co-processor CONDITIONAL operation (multi-cycle) --
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

          -- load upper immediate / add upper immediate to PC --
          when opcode_lui_c | opcode_auipc_c =>
            if (execute_engine.ir(instr_opcode_lsb_c+5) = opcode_lui_c(5)) then -- LUI
              ctrl_nxt.alu_op <= alu_op_movb_c; -- pass immediate
            else -- AUIPC
              ctrl_nxt.alu_op <= alu_op_add_c; -- add PC and immediate
            end if;
            ctrl_nxt.rf_wb_en        <= '1'; -- valid RF write-back
            execute_engine.state_nxt <= DISPATCH;

          -- memory access --
          when opcode_load_c | opcode_store_c | opcode_amo_c =>
            execute_engine.state_nxt <= MEM_REQ;

          -- branch / jump and link / with register --
          when opcode_branch_c | opcode_jal_c | opcode_jalr_c =>
            execute_engine.state_nxt <= BRANCH;

          -- memory fence operations --
          when opcode_fence_c =>
            execute_engine.state_nxt <= FENCE;

          -- FPU: floating-point operations --
          when opcode_fop_c =>
            ctrl_nxt.alu_cp_trig(cp_sel_fpu_c) <= '1'; -- trigger FPU co-processor
            execute_engine.state_nxt           <= ALU_WAIT; -- will be aborted via monitor exception if FPU not implemented

          -- CFU: custom RISC-V instructions --
          when opcode_cust0_c | opcode_cust1_c | opcode_cust2_c | opcode_cust3_c =>
            ctrl_nxt.alu_cp_trig(cp_sel_cfu_c) <= '1'; -- trigger CFU co-processor
            execute_engine.state_nxt           <= ALU_WAIT; -- will be aborted via monitor exception if CFU not implemented

          -- environment/CSR operation or ILLEGAL opcode --
          when others =>
            csr.re_nxt               <= '1';
            execute_engine.state_nxt <= SYSTEM;

        end case; -- /EXECUTE

      when ALU_WAIT => -- wait for multi-cycle ALU co-processor operation to finish/trap
      -- ------------------------------------------------------------
        ctrl_nxt.alu_op <= alu_op_cp_c;
        if (alu_cp_done_i = '1') or (trap_ctrl.exc_buf(exc_illegal_c) = '1') then
          ctrl_nxt.rf_wb_en        <= '1'; -- valid RF write-back (won't happen in case of an illegal instruction)
          execute_engine.state_nxt <= DISPATCH;
        end if;

      when FENCE => -- memory fence
      -- ------------------------------------------------------------
        if (trap_ctrl.exc_buf(exc_illegal_c) = '1') then -- abort if illegal instruction
          execute_engine.state_nxt <= DISPATCH;
        else
          ctrl_nxt.lsu_fence       <= '1'; -- NOTE: fence == fence.i
          execute_engine.state_nxt <= RESTART; -- reset instruction fetch + IPB (actually only required for fence.i)
        end if;

      when BRANCH => -- update next_PC on taken branches and jumps
      -- ------------------------------------------------------------
        ctrl_nxt.rf_wb_en <= execute_engine.ir(instr_opcode_lsb_c+2); -- save return address if link operation (will not happen if misaligned)
        if (trap_ctrl.exc_buf(exc_illegal_c) = '0') and (execute_engine.branch_taken = '1') then -- valid taken branch
          fetch_engine.reset       <= '1'; -- reset instruction fetch to restart at modified PC
          execute_engine.state_nxt <= BRANCHED; -- shortcut (faster than going to RESTART)
        else
          execute_engine.state_nxt <= DISPATCH;
        end if;

      when BRANCHED => -- delay cycle to wait for reset of pipeline front-end (instruction fetch)
      -- ------------------------------------------------------------
        ctrl_nxt.rf_zero_we      <= not bool_to_ulogic_f(REGFILE_HW_RST); -- house keeping: force writing zero to x0 if it's a phys. register
        execute_engine.state_nxt <= DISPATCH;

      when MEM_REQ => -- trigger memory request
      -- ------------------------------------------------------------
        if (trap_ctrl.exc_buf(exc_illegal_c) = '0') then -- memory request only if not an illegal instruction
          ctrl_nxt.lsu_req <= '1'; -- memory access request
        end if;
        execute_engine.state_nxt <= MEM_WAIT;

      when MEM_WAIT => -- wait for bus transaction to finish
      -- ------------------------------------------------------------
        if (lsu_wait_i = '0') or -- bus system has completed the transaction
           (trap_ctrl.exc_buf(exc_saccess_c) = '1') or (trap_ctrl.exc_buf(exc_laccess_c) = '1') or -- access exception
           (trap_ctrl.exc_buf(exc_salign_c)  = '1') or (trap_ctrl.exc_buf(exc_lalign_c)  = '1') or -- alignment exception
           (trap_ctrl.exc_buf(exc_illegal_c) = '1') then -- illegal instruction exception
          if ((CPU_EXTENSION_RISCV_A = true) and (decode_aux.opcode(2) = opcode_amo_c(2))) or -- atomic operation
             (execute_engine.ir(instr_opcode_msb_c-1) = '0') then -- normal load
            ctrl_nxt.rf_wb_en <= '1'; -- allow write-back to register file (won't happen in case of exception)
          end if;
          execute_engine.state_nxt <= DISPATCH;
        end if;

      when SLEEP => -- sleep mode
      -- ------------------------------------------------------------
        if (trap_ctrl.wakeup = '1') then
          execute_engine.state_nxt <= DISPATCH;
        end if;

      when others => -- SYSTEM - system environment operation; no effect if illegal instruction
      -- ------------------------------------------------------------
        if (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_env_c) and -- ENVIRONMENT
           (trap_ctrl.exc_buf(exc_illegal_c) = '0') then -- not an illegal instruction
          -- three LSBs are sufficient to distinguish environment instructions --
          if (execute_engine.ir(instr_funct12_lsb_c+2 downto instr_funct12_lsb_c) = "000") then
            trap_ctrl.ecall <= '1'; -- ecall
          end if;
          if (execute_engine.ir(instr_funct12_lsb_c+2 downto instr_funct12_lsb_c) = "001") then
            trap_ctrl.ebreak <= '1'; -- ebreak
          end if;
          if (execute_engine.ir(instr_funct12_lsb_c+2 downto instr_funct12_lsb_c) = "010") then
            execute_engine.state_nxt <= TRAP_EXIT; -- xret
          elsif (execute_engine.ir(instr_funct12_lsb_c+2 downto instr_funct12_lsb_c) = "101") then
            execute_engine.state_nxt <= SLEEP; -- wfi
          else
            execute_engine.state_nxt <= DISPATCH; -- default
          end if;
        else -- CSR ACCESS - no state change if illegal instruction
          if (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrw_c) or  -- CSRRW:  always write CSR
             (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrwi_c) or -- CSRRWI: always write CSR
             (decode_aux.rs1_zero = '0') then -- CSRR(S/C)(I): write CSR if rs1/imm5 is NOT zero
            csr.we_nxt <= '1';
          end if;
          ctrl_nxt.rf_wb_en        <= '1'; -- valid RF write-back
          execute_engine.state_nxt <= DISPATCH;
        end if;

    end case;
  end process execute_engine_fsm_comb;


  -- CPU Control Bus Output -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- register file --
  ctrl_o.rf_wb_en     <= ctrl.rf_wb_en and -- inhibit write-back only for rd-updating exceptions that must not commit
                         (not trap_ctrl.exc_buf(exc_illegal_c)) and
                         (not trap_ctrl.exc_buf(exc_ialign_c))  and (not trap_ctrl.exc_buf(exc_salign_c))  and (not trap_ctrl.exc_buf(exc_lalign_c)) and
                         (not trap_ctrl.exc_buf(exc_iaccess_c)) and (not trap_ctrl.exc_buf(exc_saccess_c)) and (not trap_ctrl.exc_buf(exc_laccess_c));
  ctrl_o.rf_rs1       <= execute_engine.ir(instr_rs1_msb_c downto instr_rs1_lsb_c);
  ctrl_o.rf_rs2       <= execute_engine.ir(instr_rs2_msb_c downto instr_rs2_lsb_c);
  ctrl_o.rf_rd        <= execute_engine.ir(instr_rd_msb_c downto instr_rd_lsb_c);
  ctrl_o.rf_zero_we   <= ctrl.rf_zero_we;

  -- alu --
  ctrl_o.alu_op       <= ctrl.alu_op;
  ctrl_o.alu_sub      <= ctrl.alu_sub;
  ctrl_o.alu_opa_mux  <= ctrl.alu_opa_mux;
  ctrl_o.alu_opb_mux  <= ctrl.alu_opb_mux;
  ctrl_o.alu_unsigned <= ctrl.alu_unsigned;
  ctrl_o.alu_cp_trig  <= ctrl.alu_cp_trig;

  -- data bus interface --
  ctrl_o.lsu_req      <= ctrl.lsu_req;
  ctrl_o.lsu_rw       <= ctrl.lsu_rw;
  ctrl_o.lsu_mo_we    <= '1' when (execute_engine.state = MEM_REQ) else '0'; -- write memory output registers (data & address)
  ctrl_o.lsu_fence    <= ctrl.lsu_fence; -- fence(.i)
  ctrl_o.lsu_priv     <= csr.mstatus_mpp when (csr.mstatus_mprv = '1') else csr.privilege_eff; -- effective privilege level for loads/stores in M-mode

  -- instruction word bit fields --
  ctrl_o.ir_funct3    <= execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c);
  ctrl_o.ir_funct12   <= execute_engine.ir(instr_funct12_msb_c downto instr_funct12_lsb_c);
  ctrl_o.ir_opcode    <= execute_engine.ir(instr_opcode_msb_c downto instr_opcode_lsb_c);

  -- cpu status --
  ctrl_o.cpu_priv     <= csr.privilege_eff;
  ctrl_o.cpu_sleep    <= sleep_mode;
  ctrl_o.cpu_trap     <= trap_ctrl.env_enter;
  ctrl_o.cpu_debug    <= debug_ctrl.running;


-- ****************************************************************************************************************************
-- Illegal Instruction Detection
-- ****************************************************************************************************************************

  -- Instruction Execution Monitor (trap if multi-cycle instruction does not complete) ------
  -- -------------------------------------------------------------------------------------------
  multi_cycle_monitor: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      monitor.cnt <= (others => '0');
    elsif rising_edge(clk_i) then
      monitor.cnt <= std_ulogic_vector(unsigned(monitor.cnt_add) + 1);
    end if;
  end process multi_cycle_monitor;

  -- timeout counter (allow mapping of entire logic into the LUTs in front of the carry-chain) --
  monitor.cnt_add <= monitor.cnt when (execute_engine.state = ALU_WAIT) else (others => '0');

  -- raise illegal instruction exception if a multi-cycle instruction takes longer than a bound amount of time --
  monitor.exc <= monitor.cnt(monitor.cnt'left);


  -- CSR Access Check: Available at All -----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_avail_check: process(csr.addr)
  begin
    case csr.addr is

      -- user-defined U-mode CFU CSRs --
      when csr_cfureg0_c | csr_cfureg1_c | csr_cfureg2_c | csr_cfureg3_c =>
        csr_reg_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zxcfu); -- available if CFU implemented

      -- floating-point CSRs --
      when csr_fflags_c | csr_frm_c | csr_fcsr_c =>
        csr_reg_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zfinx); -- available if FPU implemented

      -- machine trap setup/handling, environment/information registers, etc. --
      when csr_mstatus_c  | csr_mstatush_c      | csr_misa_c      | csr_mie_c     | csr_mtvec_c  |
           csr_mscratch_c | csr_mepc_c          | csr_mcause_c    | csr_mip_c     | csr_mtval_c  |
           csr_mtinst_c   | csr_mcountinhibit_c | csr_mvendorid_c | csr_marchid_c | csr_mimpid_c |
           csr_mhartid_c  | csr_mconfigptr_c    | csr_mxisa_c =>
        csr_reg_valid <= '1'; -- always implemented

      -- machine-controlled user-mode CSRs --
      when csr_mcounteren_c | csr_menvcfg_c | csr_menvcfgh_c =>
        csr_reg_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_U); -- available if U-mode implemented

      -- physical memory protection (PMP) --
      when csr_pmpcfg0_c   | csr_pmpcfg1_c   | csr_pmpcfg2_c   | csr_pmpcfg3_c   | -- configuration
           csr_pmpaddr0_c  | csr_pmpaddr1_c  | csr_pmpaddr2_c  | csr_pmpaddr3_c  |
           csr_pmpaddr4_c  | csr_pmpaddr5_c  | csr_pmpaddr6_c  | csr_pmpaddr7_c  | -- address
           csr_pmpaddr8_c  | csr_pmpaddr9_c  | csr_pmpaddr10_c | csr_pmpaddr11_c |
           csr_pmpaddr12_c | csr_pmpaddr13_c | csr_pmpaddr14_c | csr_pmpaddr15_c =>
        csr_reg_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Smpmp); -- available if PMP implemented

      -- hardware performance monitors (HPM) --
      when csr_hpmcounter3_c    | csr_hpmcounter4_c    | csr_hpmcounter5_c    | csr_hpmcounter6_c    | csr_hpmcounter7_c    |
           csr_hpmcounter8_c    | csr_hpmcounter9_c    | csr_hpmcounter10_c   | csr_hpmcounter11_c   | csr_hpmcounter12_c   |
           csr_hpmcounter13_c   | csr_hpmcounter14_c   | csr_hpmcounter15_c   | -- user counters LOW
           csr_hpmcounter3h_c   | csr_hpmcounter4h_c   | csr_hpmcounter5h_c   | csr_hpmcounter6h_c   | csr_hpmcounter7h_c   |
           csr_hpmcounter8h_c   | csr_hpmcounter9h_c   | csr_hpmcounter10h_c  | csr_hpmcounter11h_c  | csr_hpmcounter12h_c  |
           csr_hpmcounter13h_c  | csr_hpmcounter14h_c  | csr_hpmcounter15h_c  | -- user counters HIGH
           csr_mhpmcounter3_c   | csr_mhpmcounter4_c   | csr_mhpmcounter5_c   | csr_mhpmcounter6_c   | csr_mhpmcounter7_c   |
           csr_mhpmcounter8_c   | csr_mhpmcounter9_c   | csr_mhpmcounter10_c  | csr_mhpmcounter11_c  | csr_mhpmcounter12_c  |
           csr_mhpmcounter13_c  | csr_mhpmcounter14_c  | csr_mhpmcounter15_c  | -- machine counters LOW
           csr_mhpmcounter3h_c  | csr_mhpmcounter4h_c  | csr_mhpmcounter5h_c  | csr_mhpmcounter6h_c  | csr_mhpmcounter7h_c  |
           csr_mhpmcounter8h_c  | csr_mhpmcounter9h_c  | csr_mhpmcounter10h_c | csr_mhpmcounter11h_c | csr_mhpmcounter12h_c |
           csr_mhpmcounter13h_c | csr_mhpmcounter14h_c | csr_mhpmcounter15h_c | -- machine counters HIGH
           csr_mhpmevent3_c     | csr_mhpmevent4_c     | csr_mhpmevent5_c     | csr_mhpmevent6_c     | csr_mhpmevent7_c     |
           csr_mhpmevent8_c     | csr_mhpmevent9_c     | csr_mhpmevent10_c    | csr_mhpmevent11_c    | csr_mhpmevent12_c    |
           csr_mhpmevent13_c    | csr_mhpmevent14_c    | csr_mhpmevent15_c => -- machine event configuration
        csr_reg_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zihpm); -- available if Zihpm implemented

      -- counter and timer CSRs --
      when csr_cycle_c  | csr_mcycle_c  | csr_instret_c  | csr_minstret_c |
           csr_cycleh_c | csr_mcycleh_c | csr_instreth_c | csr_minstreth_c =>
        csr_reg_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicntr); -- available if Zicntr implemented

      -- debug-mode CSRs --
      when csr_dcsr_c | csr_dpc_c | csr_dscratch0_c =>
        csr_reg_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Sdext); -- available if debug-mode implemented

      -- trigger module CSRs --
      when csr_tselect_c | csr_tdata1_c | csr_tdata2_c | csr_tinfo_c =>
        csr_reg_valid <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Sdtrig); -- available if trigger module implemented

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
       ((execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrw_c)  or -- will always write to CSR
        (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrwi_c) or -- will always write to CSR
        (decode_aux.rs1_zero = '0')) then -- clear/set instructions: write to CSR only if rs1/imm5 is NOT zero
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
    elsif (csr.addr(11 downto 8) = csr_cycle_c(11 downto 8)) and -- user-mode counter access
          ((CPU_EXTENSION_RISCV_Zicntr = true) or (CPU_EXTENSION_RISCV_Zihpm = true)) and -- any counters available?
          (CPU_EXTENSION_RISCV_U = true) and (csr.privilege_eff = '0') and (csr.mcounteren = '0') then -- user mode enabled, active and access not allowed?
      csr_priv_valid <= '0'; -- invalid access
    elsif (csr.addr(9 downto 8) /= "00") and (csr.privilege_eff = '0') then -- invalid privilege level
      csr_priv_valid <= '0'; -- invalid access
    else
      csr_priv_valid <= '1'; -- access granted
    end if;
  end process csr_priv_check;


  -- Illegal Instruction Check --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  illegal_check: process(execute_engine, decode_aux, csr, csr_reg_valid, csr_rw_valid, csr_priv_valid, debug_ctrl)
  begin
    illegal_cmd <= '0'; -- default
    case decode_aux.opcode is

      when opcode_lui_c | opcode_auipc_c | opcode_jal_c =>
        illegal_cmd <= '0'; -- all encodings are valid

      when opcode_jalr_c =>
        case execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          when "000"  => illegal_cmd <= '0';
          when others => illegal_cmd <= '1';
        end case;

      when opcode_branch_c =>
        case execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          when funct3_beq_c | funct3_bne_c | funct3_blt_c | funct3_bge_c | funct3_bltu_c | funct3_bgeu_c => illegal_cmd <= '0';
          when others => illegal_cmd <= '1';
        end case;

      when opcode_load_c =>
        case execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          when funct3_lb_c | funct3_lh_c | funct3_lw_c | funct3_lbu_c | funct3_lhu_c => illegal_cmd <= '0';
          when others => illegal_cmd <= '1';
        end case;

      when opcode_store_c =>
        case execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          when funct3_sb_c | funct3_sh_c | funct3_sw_c => illegal_cmd <= '0';
          when others => illegal_cmd <= '1';
        end case;

      when opcode_amo_c =>
        if (CPU_EXTENSION_RISCV_A = true) and ((decode_aux.is_a_lr = '1') or (decode_aux.is_a_sc = '1')) then -- LR.W/SC.W
          illegal_cmd <= '0';
        else
          illegal_cmd <= '1';
        end if;

      when opcode_alu_c =>
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

      when opcode_alui_c =>
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

      when opcode_fence_c =>
        case execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          when funct3_fence_c | funct3_fencei_c => illegal_cmd <= '0'; -- fence[.i]
          when others                           => illegal_cmd <= '1';
        end case;

      when opcode_system_c =>
        if (execute_engine.ir(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_env_c) then -- system environment
          if (decode_aux.rs1_zero = '1') and (decode_aux.rd_zero = '1') then
            case execute_engine.ir(instr_funct12_msb_c downto instr_funct12_lsb_c) is
              when funct12_ecall_c | funct12_ebreak_c => illegal_cmd <= '0'; -- ecall, ebreak
              when funct12_mret_c                     => illegal_cmd <= (not csr.privilege) or debug_ctrl.running; -- mret allowed in (real/non-debug) M-mode only
              when funct12_dret_c                     => illegal_cmd <= not debug_ctrl.running; -- dret allowed in debug mode only
              when funct12_wfi_c                      => illegal_cmd <= (not csr.privilege) and csr.mstatus_tw; -- wfi allowed in M-mode or if TW is zero
              when others                             => illegal_cmd <= '1';
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

      when opcode_fop_c =>
        illegal_cmd <= (not bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zfinx)) or (not decode_aux.is_f_op);

      when opcode_cust0_c | opcode_cust1_c | opcode_cust2_c | opcode_cust3_c =>
        illegal_cmd <= not bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zxcfu); -- all encodings valid if CFU enable

      when others =>
        illegal_cmd <= '1'; -- undefined/illegal opcode

    end case;
  end process illegal_check;


  -- Illegal Operation Check ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trap_ctrl.instr_il <= '1' when ((execute_engine.state = EXECUTE) or (execute_engine.state = ALU_WAIT)) and -- check in execution states only
                                 ((monitor.exc = '1') or -- execution monitor exception (multi-cycle instruction timeout)
                                  (illegal_cmd = '1') or -- illegal instruction?
                                  (execute_engine.ir(instr_opcode_lsb_c+1 downto instr_opcode_lsb_c) /= "11")) else '0'; -- illegal opcode LSBs?


-- ****************************************************************************************************************************
-- Trap Controller
-- ****************************************************************************************************************************

  -- Exception Buffer -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  exception_buffer: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      trap_ctrl.exc_buf <= (others => '0');
    elsif rising_edge(clk_i) then

      -- Exception Buffer -----------------------------------------------------
      -- If several exception sources trigger at once, all the requests will
      -- stay active until the trap environment is started. Only the exception
      -- with highest priority will be used to update the MCAUSE CSR. All
      -- remaining ones will be discarded.
      -- ----------------------------------------------------------------------

      -- misaligned load/store/instruction address --
      trap_ctrl.exc_buf(exc_lalign_c) <= (trap_ctrl.exc_buf(exc_lalign_c) or ma_load_i)          and (not trap_ctrl.env_enter);
      trap_ctrl.exc_buf(exc_salign_c) <= (trap_ctrl.exc_buf(exc_salign_c) or ma_store_i)         and (not trap_ctrl.env_enter);
      trap_ctrl.exc_buf(exc_ialign_c) <= (trap_ctrl.exc_buf(exc_ialign_c) or trap_ctrl.instr_ma) and (not trap_ctrl.env_enter);

      -- load/store/instruction access fault --
      trap_ctrl.exc_buf(exc_laccess_c) <= (trap_ctrl.exc_buf(exc_laccess_c) or be_load_i)          and (not trap_ctrl.env_enter);
      trap_ctrl.exc_buf(exc_saccess_c) <= (trap_ctrl.exc_buf(exc_saccess_c) or be_store_i)         and (not trap_ctrl.env_enter);
      trap_ctrl.exc_buf(exc_iaccess_c) <= (trap_ctrl.exc_buf(exc_iaccess_c) or trap_ctrl.instr_be) and (not trap_ctrl.env_enter);

      -- illegal instruction & environment call --
      trap_ctrl.exc_buf(exc_ecall_c)   <= (trap_ctrl.exc_buf(exc_ecall_c)   or trap_ctrl.ecall)    and (not trap_ctrl.env_enter);
      trap_ctrl.exc_buf(exc_illegal_c) <= (trap_ctrl.exc_buf(exc_illegal_c) or trap_ctrl.instr_il) and (not trap_ctrl.env_enter);

      -- break point --
      if (CPU_EXTENSION_RISCV_Sdext = true) then
        trap_ctrl.exc_buf(exc_ebreak_c) <= (not trap_ctrl.env_enter) and (trap_ctrl.exc_buf(exc_ebreak_c) or
          (trap_ctrl.hwtrig and (not csr.tdata1_action)) or -- trigger module fires and enter-debug-action is disabled
          (trap_ctrl.ebreak and (    csr.privilege) and (not csr.dcsr_ebreakm) and (not debug_ctrl.running)) or -- enter M-mode handler on ebreak in M-mode
          (trap_ctrl.ebreak and (not csr.privilege) and (not csr.dcsr_ebreaku) and (not debug_ctrl.running)));  -- enter M-mode handler on ebreak in U-mode
      else
        trap_ctrl.exc_buf(exc_ebreak_c) <= (trap_ctrl.exc_buf(exc_ebreak_c) or trap_ctrl.ebreak or (trap_ctrl.hwtrig and (not csr.tdata1_action))) and (not trap_ctrl.env_enter);
      end if;

      -- debug-mode entry --
      if (CPU_EXTENSION_RISCV_Sdext = true) then
        trap_ctrl.exc_buf(exc_db_break_c) <= (trap_ctrl.exc_buf(exc_db_break_c) or debug_ctrl.trig_break) and (not trap_ctrl.env_enter);
        trap_ctrl.exc_buf(exc_db_hw_c)    <= (trap_ctrl.exc_buf(exc_db_hw_c)    or debug_ctrl.trig_hw)    and (not trap_ctrl.env_enter);
      else
        trap_ctrl.exc_buf(exc_db_break_c) <= '0';
        trap_ctrl.exc_buf(exc_db_hw_c)    <= '0';
      end if;
    end if;
  end process exception_buffer;


  -- Interrupt Buffer -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  interrupt_buffer: process(rstn_i, clk_aux_i) -- always-on clock domain
  begin
    if (rstn_i = '0') then
      trap_ctrl.irq_pnd <= (others => '0');
      trap_ctrl.irq_buf <= (others => '0');
    elsif rising_edge(clk_aux_i) then

      -- Interrupt-Pending Buffer ---------------------------------------------
      -- Once triggered the interrupt line should stay active until explicitly
      -- cleared by a mechanism specific to the interrupt-causing module.
      -- ----------------------------------------------------------------------

      -- RISC-V machine interrupts --
      trap_ctrl.irq_pnd(irq_msi_irq_c) <= msi_i;
      trap_ctrl.irq_pnd(irq_mei_irq_c) <= mei_i;
      trap_ctrl.irq_pnd(irq_mti_irq_c) <= mti_i;

      -- NEORV32-specific fast interrupts --
      trap_ctrl.irq_pnd(irq_firq_15_c downto irq_firq_0_c) <= firq_i(15 downto 0);

      -- debug-mode entry --
      trap_ctrl.irq_pnd(irq_db_halt_c) <= '0'; -- unused
      trap_ctrl.irq_pnd(irq_db_step_c) <= '0'; -- unused

      -- Interrupt-Masking Buffer ---------------------------------------------
      -- Masking of interrupt request lines. Additionally, this buffer ensures
      -- that an active interrupt request line stays active (even when
      -- disabled via MIE) if the trap environment is already starting.
      -- ----------------------------------------------------------------------

      -- RISC-V machine interrupts --
      trap_ctrl.irq_buf(irq_msi_irq_c) <= (trap_ctrl.irq_pnd(irq_msi_irq_c) and csr.mie_msi) or (trap_ctrl.env_pending and trap_ctrl.irq_buf(irq_msi_irq_c));
      trap_ctrl.irq_buf(irq_mei_irq_c) <= (trap_ctrl.irq_pnd(irq_mei_irq_c) and csr.mie_mei) or (trap_ctrl.env_pending and trap_ctrl.irq_buf(irq_mei_irq_c));
      trap_ctrl.irq_buf(irq_mti_irq_c) <= (trap_ctrl.irq_pnd(irq_mti_irq_c) and csr.mie_mti) or (trap_ctrl.env_pending and trap_ctrl.irq_buf(irq_mti_irq_c));

      -- NEORV32-specific fast interrupts --
      for i in 0 to 15 loop
        trap_ctrl.irq_buf(irq_firq_0_c+i) <= (trap_ctrl.irq_pnd(irq_firq_0_c+i) and csr.mie_firq(i)) or (trap_ctrl.env_pending and trap_ctrl.irq_buf(irq_firq_0_c+i));
      end loop;

      -- debug-mode entry --
      if (CPU_EXTENSION_RISCV_Sdext = true) then
        trap_ctrl.irq_buf(irq_db_halt_c) <= debug_ctrl.trig_halt or (trap_ctrl.env_pending and trap_ctrl.irq_buf(irq_db_halt_c));
        trap_ctrl.irq_buf(irq_db_step_c) <= debug_ctrl.trig_step or (trap_ctrl.env_pending and trap_ctrl.irq_buf(irq_db_step_c));
      else
        trap_ctrl.irq_buf(irq_db_halt_c) <= '0';
        trap_ctrl.irq_buf(irq_db_step_c) <= '0';
      end if;
    end if;
  end process interrupt_buffer;


  -- Trap Priority Logic --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trap_priority: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      trap_ctrl.cause <= (others => '0');
    elsif rising_edge(clk_i) then
      -- standard RISC-V exceptions --
      if    (trap_ctrl.exc_buf(exc_iaccess_c)  = '1') then trap_ctrl.cause <= trap_iaf_c; -- instruction access fault
      elsif (trap_ctrl.exc_buf(exc_illegal_c)  = '1') then trap_ctrl.cause <= trap_iil_c; -- illegal instruction
      elsif (trap_ctrl.exc_buf(exc_ialign_c)   = '1') then trap_ctrl.cause <= trap_ima_c; -- instruction address misaligned
      elsif (trap_ctrl.exc_buf(exc_ecall_c)    = '1') then trap_ctrl.cause <= trap_env_c(6 downto 2) & csr.privilege & csr.privilege; -- environment call (U/M)
      elsif (trap_ctrl.exc_buf(exc_ebreak_c)   = '1') then trap_ctrl.cause <= trap_brk_c; -- environment breakpoint
      elsif (trap_ctrl.exc_buf(exc_salign_c)   = '1') then trap_ctrl.cause <= trap_sma_c; -- store address misaligned
      elsif (trap_ctrl.exc_buf(exc_lalign_c)   = '1') then trap_ctrl.cause <= trap_lma_c; -- load address misaligned
      elsif (trap_ctrl.exc_buf(exc_saccess_c)  = '1') then trap_ctrl.cause <= trap_saf_c; -- store access fault
      elsif (trap_ctrl.exc_buf(exc_laccess_c)  = '1') then trap_ctrl.cause <= trap_laf_c; -- load access fault
      -- standard RISC-V debug mode exceptions and interrupts --
      elsif (trap_ctrl.irq_buf(irq_db_halt_c)  = '1') then trap_ctrl.cause <= trap_db_halt_c;  -- external halt request (async)
      elsif (trap_ctrl.exc_buf(exc_db_hw_c)    = '1') then trap_ctrl.cause <= trap_db_trig_c;  -- hardware trigger (sync)
      elsif (trap_ctrl.exc_buf(exc_db_break_c) = '1') then trap_ctrl.cause <= trap_db_break_c; -- breakpoint (sync)
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
      elsif (trap_ctrl.irq_buf(irq_msi_irq_c)  = '1') then trap_ctrl.cause <= trap_msi_c; -- machine software interrupt (MSI)
      elsif (trap_ctrl.irq_buf(irq_mti_irq_c)  = '1') then trap_ctrl.cause <= trap_mti_c; -- machine timer interrupt (MTI)
      else trap_ctrl.cause <= (others => '0'); end if;
    end if;
  end process trap_priority;


  -- Trap Controller ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trap_controller: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      trap_ctrl.env_pending <= '0';
    elsif rising_edge(clk_i) then
      if (trap_ctrl.env_pending = '0') then -- no pending trap environment yet
        if (trap_ctrl.exc_fire = '1') or ((trap_ctrl.irq_fire = '1') and (execute_engine.state = EXECUTE)) then -- trigger IRQ only in EXECUTE state
          trap_ctrl.env_pending <= '1'; -- now execute engine can start trap handling
        end if;
      elsif (trap_ctrl.env_enter = '1') then -- start of trap environment acknowledged by execute engine
        trap_ctrl.env_pending <= '0';
      end if;
    end if;
  end process trap_controller;

  -- wake-up from / do not enter sleep mode: during debugging or on pending IRQ --
  trap_ctrl.wakeup <= or_reduce_f(trap_ctrl.irq_buf) or debug_ctrl.running or csr.dcsr_step;

  -- any exception? --
  trap_ctrl.exc_fire <= '1' when (or_reduce_f(trap_ctrl.exc_buf) = '1') else '0'; -- sync. exceptions CANNOT be masked

  -- any interrupt? --
  trap_ctrl.irq_fire <= '1' when
    (
     (or_reduce_f(trap_ctrl.irq_buf(irq_firq_15_c downto irq_msi_irq_c)) = '1') and -- pending IRQ
     ((csr.mstatus_mie = '1') or (csr.privilege = priv_mode_u_c)) and -- take IRQ when in M-mode and MIE=1 OR when in U-mode
     (debug_ctrl.running = '0') and (csr.dcsr_step = '0') -- no IRQs when in debug-mode or during debug single-stepping
    ) or
    (trap_ctrl.irq_buf(irq_db_step_c) = '1') or -- debug-mode single-step IRQ
    (trap_ctrl.irq_buf(irq_db_halt_c) = '1') else '0'; -- debug-mode halt IRQ

  -- exception program counter (for updating xPC CSRs) --
  trap_ctrl.epc <= execute_engine.next_pc when (trap_ctrl.cause(trap_ctrl.cause'left) = '1') else execute_engine.pc;


  -- CPU Sleep Mode Control -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  sleep_control: process(rstn_i, clk_aux_i) -- always-on clock domain
  begin
    if (rstn_i = '0') then
      sleep_mode <= '0';
    elsif rising_edge(clk_aux_i) then
      if (execute_engine.state = SLEEP) and -- instruction execution has halted
         (ipb.free /= "11") and -- instruction fetch has halted
         (trap_ctrl.wakeup = '0') then -- no wake-up request
        sleep_mode <= '1';
      else
        sleep_mode <= '0';
      end if;
    end if;
  end process sleep_control;


-- ****************************************************************************************************************************
-- Control and Status Registers (CSRs)
-- ****************************************************************************************************************************

  -- CSR Access Address ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr.addr  <= execute_engine.ir(instr_imm12_msb_c downto instr_imm12_lsb_c);
  -- simplified CSR read address - [WARNING] M-mode (9:8 = 11) and U-mode (9:8 = 00) CSRs only! --
  csr.raddr <= csr.addr(11 downto 10) & csr.addr(8) & csr.addr(8) & csr.addr(7 downto 0);


  -- CSR Write Data -------------------------------------------------------------------------
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


  -- External CSR Interface -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  xcsr_we_o    <= csr.we;
  xcsr_addr_o  <= csr.addr;
  xcsr_wdata_o <= csr.wdata;
  xcsr_rdata   <= xcsr_rdata_i;


  -- CSR Write Access -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      csr.we             <= '0';
      csr.privilege      <= priv_mode_m_c;
      csr.mstatus_mie    <= '0';
      csr.mstatus_mpie   <= '0';
      csr.mstatus_mpp    <= priv_mode_m_c;
      csr.mstatus_mprv   <= '0';
      csr.mstatus_tw     <= '0';
      csr.mie_msi        <= '0';
      csr.mie_mei        <= '0';
      csr.mie_mti        <= '0';
      csr.mie_firq       <= (others => '0');
      csr.mtvec          <= CPU_BOOT_ADDR(XLEN-1 downto 2) & "00"; -- 32-bit aligned boot address
      csr.mscratch       <= x"19880704";
      csr.mepc           <= CPU_BOOT_ADDR(XLEN-1 downto 2) & "00"; -- 32-bit aligned boot address
      csr.mcause         <= (others => '0');
      csr.mtval          <= (others => '0');
      csr.mtinst         <= (others => '0');
      csr.mcounteren     <= '0';
      csr.mcountinhibit  <= (others => '0');
      csr.dcsr_ebreakm   <= '0';
      csr.dcsr_ebreaku   <= '0';
      csr.dcsr_step      <= '0';
      csr.dcsr_prv       <= priv_mode_m_c;
      csr.dcsr_cause     <= (others => '0');
      csr.dpc            <= CPU_BOOT_ADDR(XLEN-1 downto 2) & "00"; -- 32-bit aligned boot address
      csr.dscratch0      <= (others => '0');
      csr.tdata1_execute <= '0';
      csr.tdata1_action  <= '0';
      csr.tdata1_dmode   <= '0';
      csr.tdata2         <= (others => '0');
    elsif rising_edge(clk_i) then

      -- write if not an illegal instruction --
      csr.we <= csr.we_nxt and (not trap_ctrl.exc_buf(exc_illegal_c));

      -- ********************************************************************************
      -- CSR access by application software
      -- ********************************************************************************
      if (csr.we = '1') then
        case csr.addr is

          -- --------------------------------------------------------------------
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
            csr.mie_msi  <= csr.wdata(03);
            csr.mie_mti  <= csr.wdata(07);
            csr.mie_mei  <= csr.wdata(11);
            csr.mie_firq <= csr.wdata(31 downto 16);

          when csr_mtvec_c => -- machine trap-handler base address
            if (csr.wdata(1 downto 0) = "01") then
              csr.mtvec <= csr.wdata(XLEN-1 downto 7) & "00000" & "01"; -- mtvec.MODE=1 (vectored)
            else
              csr.mtvec <= csr.wdata(XLEN-1 downto 2) & "00"; -- mtvec.MODE=0 (direct)
            end if;

          when csr_mcounteren_c => -- machine counter access enable
            if (CPU_EXTENSION_RISCV_U = true) then
              if (CPU_EXTENSION_RISCV_Zicntr = true) and (CPU_EXTENSION_RISCV_Zihpm = true) then
                csr.mcounteren <= or_reduce_f(csr.wdata(15 downto 3)) or csr.wdata(2) or csr.wdata(0); -- hpms, instret, cycle
              elsif (CPU_EXTENSION_RISCV_Zicntr = true) then
                csr.mcounteren <= csr.wdata(2) or csr.wdata(0); -- instret, cycle
              elsif (CPU_EXTENSION_RISCV_Zihpm = true) then
                csr.mcounteren <= or_reduce_f(csr.wdata(15 downto 3)); -- hpms
              end if;
            end if;

          -- --------------------------------------------------------------------
          -- machine trap handling --
          -- --------------------------------------------------------------------
          when csr_mscratch_c => -- machine scratch register
            csr.mscratch <= csr.wdata;

          when csr_mepc_c => -- machine exception program counter
            csr.mepc <= csr.wdata(XLEN-1 downto 1) & '0';
            if (CPU_EXTENSION_RISCV_C = false) then -- RISC-V priv. spec.: MEPC[1] is masked when IALIGN = 32
              csr.mepc(1) <= '0';
            end if;

          when csr_mcause_c => -- machine trap cause
            csr.mcause <= csr.wdata(31) & csr.wdata(4 downto 0); -- type (exception/interrupt) & identifier

          -- --------------------------------------------------------------------
          -- machine counter setup --
          -- --------------------------------------------------------------------
          when csr_mcountinhibit_c => -- machine counter-inhibit register
            if (CPU_EXTENSION_RISCV_Zicntr = true) then
              csr.mcountinhibit(0) <= csr.wdata(0);
              csr.mcountinhibit(2) <= csr.wdata(2);
            end if;
            if (CPU_EXTENSION_RISCV_Zihpm = true) then
              csr.mcountinhibit(15 downto 3) <= csr.wdata(15 downto 3);
            end if;

          -- --------------------------------------------------------------------
          -- debug mode CSRs --
          -- --------------------------------------------------------------------
          when csr_dcsr_c => -- debug mode control and status register
            if (CPU_EXTENSION_RISCV_Sdext = true) then
              csr.dcsr_ebreakm <= csr.wdata(15);
              csr.dcsr_step    <= csr.wdata(2);
              if (CPU_EXTENSION_RISCV_U = true) then
                csr.dcsr_ebreaku <= csr.wdata(12);
                csr.dcsr_prv     <= csr.wdata(1) or csr.wdata(0); -- everything /= U will fall back to M
              end if;
            end if;

          when csr_dpc_c => -- debug mode program counter
            if (CPU_EXTENSION_RISCV_Sdext = true) then
              csr.dpc <= csr.wdata(XLEN-1 downto 1) & '0';
              if (CPU_EXTENSION_RISCV_C = false) then -- RISC-V priv. spec.: DPC[1] is masked when IALIGN = 32
                csr.dpc(1) <= '0';
              end if;
            end if;

          when csr_dscratch0_c => -- debug mode scratch register 0
            if (CPU_EXTENSION_RISCV_Sdext = true) then
              csr.dscratch0 <= csr.wdata;
            end if;

          -- --------------------------------------------------------------------
          -- trigger module CSRs --
          -- --------------------------------------------------------------------
          when csr_tdata1_c => -- match control
            if (CPU_EXTENSION_RISCV_Sdtrig = true) then
              if (csr.tdata1_dmode = '0') or (debug_ctrl.running = '1') then -- write access from debug-mode only?
                csr.tdata1_execute <= csr.wdata(2);
                csr.tdata1_action  <= csr.wdata(12);
              end if;
              if (debug_ctrl.running = '1') then -- writable from debug-mode only
                csr.tdata1_dmode <= csr.wdata(27);
              end if;
            end if;

          when csr_tdata2_c => -- address compare
            if (CPU_EXTENSION_RISCV_Sdtrig = true) then
              if (csr.tdata1_dmode = '0') or (debug_ctrl.running = '1') then -- write access from debug-mode only?
                csr.tdata2 <= csr.wdata(XLEN-1 downto 1) & '0';
              end if;
            end if;

          -- --------------------------------------------------------------------
          -- not implemented (or implemented externally) --
          -- --------------------------------------------------------------------
          when others => NULL;

        end case;

      -- ********************************************************************************
      -- Hardware CSR access: TRAP ENTER
      -- ********************************************************************************
      elsif (trap_ctrl.env_enter = '1') then

        -- NORMAL trap entry - no CSR update when in debug-mode! --
        if (CPU_EXTENSION_RISCV_Sdext = false) or ((trap_ctrl.cause(5) = '0') and (debug_ctrl.running = '0')) then
          csr.mcause <= trap_ctrl.cause(trap_ctrl.cause'left) & trap_ctrl.cause(4 downto 0); -- trap type & identifier
          csr.mepc   <= trap_ctrl.epc(XLEN-1 downto 1) & '0'; -- trap PC
          -- trap value --
          if (trap_ctrl.cause(6) = '0') and (trap_ctrl.cause(2) = '1') then -- load/store misaligned/access faults [hacky!]
            csr.mtval <= mar_i; -- faulting data access address
          else -- everything else including all interrupts
            csr.mtval <= (others => '0');
          end if;
          -- trap instruction --
          if (trap_ctrl.cause(6) = '0') then -- exception
            csr.mtinst <= execute_engine.ir;
            if (execute_engine.is_ci = '1') and (CPU_EXTENSION_RISCV_C = true) then
              csr.mtinst(1) <= '0'; -- RISC-V priv. spec: clear bit 1 if compressed instruction
            end if;
          else -- interrupt
            csr.mtinst <= (others => '0');
          end if;
          -- update privilege level and interrupt-enable stack --
          csr.privilege    <= priv_mode_m_c; -- execute trap in machine mode
          csr.mstatus_mie  <= '0'; -- disable interrupts
          csr.mstatus_mpie <= csr.mstatus_mie; -- backup previous mie state
          csr.mstatus_mpp  <= csr.privilege; -- backup previous privilege mode
        end if;

        -- DEBUG MODE entry - no CSR update when already in debug-mode! --
        if (CPU_EXTENSION_RISCV_Sdext = true) and (trap_ctrl.cause(5) = '1') and (debug_ctrl.running = '0') then
          -- trap cause --
          csr.dcsr_cause <= trap_ctrl.cause(2 downto 0); -- why did we enter debug mode?
          -- current privilege mode when debug mode was entered --
          csr.dcsr_prv <= csr.privilege;
          -- trap PC --
          csr.dpc <= trap_ctrl.epc(XLEN-1 downto 1) & '0';
        end if;

      -- ********************************************************************************
      -- Hardware CSR access: TRAP EXIT
      -- ********************************************************************************
      elsif (trap_ctrl.env_exit = '1') then

        -- return from debug mode --
        if (CPU_EXTENSION_RISCV_Sdext = true) and (debug_ctrl.running = '1') then
          if (CPU_EXTENSION_RISCV_U = true) then
            csr.privilege <= csr.dcsr_prv;
            if (csr.dcsr_prv /= priv_mode_m_c) then
              csr.mstatus_mprv <= '0'; -- clear if return to priv. mode less than M
            end if;
          end if;

        -- return from normal trap --
        else
          if (CPU_EXTENSION_RISCV_U = true) then
            csr.privilege   <= csr.mstatus_mpp; -- restore previous privilege mode
            csr.mstatus_mpp <= priv_mode_u_c; -- set to least-privileged mode that is supported
            if (csr.mstatus_mpp /= priv_mode_m_c) then
              csr.mstatus_mprv <= '0'; -- clear if return to priv. mode less than M
            end if;
          end if;
          csr.mstatus_mie  <= csr.mstatus_mpie; -- restore machine-mode IRQ enable flag
          csr.mstatus_mpie <= '1';
        end if;

      end if;

      -- ********************************************************************************
      -- Override - hardwire/terminate unimplemented registers/bits
      -- ********************************************************************************

      -- hardwired bits --
      csr.mcountinhibit(1) <= '0'; -- time[h] not implemented

      -- no base counters --
      if (CPU_EXTENSION_RISCV_Zicntr = false) then
        csr.mcountinhibit(2 downto 0) <= (others => '0');
      end if;

      -- no hardware performance monitors --
      if (CPU_EXTENSION_RISCV_Zihpm = false) then
        csr.mcountinhibit(15 downto 3) <= (others => '0');
      end if;

      -- no counters at all --
      if (CPU_EXTENSION_RISCV_Zicntr = false) and (CPU_EXTENSION_RISCV_Zihpm = false) then
        csr.mcounteren <= '0';
      end if;

      -- no user mode --
      if (CPU_EXTENSION_RISCV_U = false) then
        csr.privilege    <= priv_mode_m_c;
        csr.mstatus_mpp  <= priv_mode_m_c;
        csr.mstatus_mprv <= '0';
        csr.mstatus_tw   <= '0';
        csr.dcsr_ebreaku <= '0';
        csr.dcsr_prv     <= '0';
        csr.mcounteren   <= '0';
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

      -- no trigger module --
      if (CPU_EXTENSION_RISCV_Sdtrig = false) then
        csr.tdata1_execute <= '0';
        csr.tdata1_action  <= '0';
        csr.tdata1_dmode   <= '0';
        csr.tdata2         <= (others => '0');
      end if;

    end if;
  end process csr_write_access;

  -- effective privilege mode is MACHINE when in debug mode --
  csr.privilege_eff <= priv_mode_m_c when (debug_ctrl.running = '1') else csr.privilege;


  -- CSR Read Access ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_read_access: process(csr, trap_ctrl.irq_pnd, hpmevent_rd, cnt_lo_rd, cnt_hi_rd)
  begin
    csr_rdata <= (others => '0'); -- default
    case csr.raddr is

      -- --------------------------------------------------------------------
      -- machine trap setup --
      -- --------------------------------------------------------------------
      when csr_mstatus_c => -- machine status register - low word
        csr_rdata(03) <= csr.mstatus_mie;
        csr_rdata(07) <= csr.mstatus_mpie;
        csr_rdata(12 downto 11) <= (others => csr.mstatus_mpp);
        csr_rdata(17) <= csr.mstatus_mprv;
        csr_rdata(21) <= csr.mstatus_tw and bool_to_ulogic_f(CPU_EXTENSION_RISCV_U);

--    when csr_mstatush_c => csr_rdata <= (others => '0'); -- machine status register - high word - hardwired to zero

      when csr_misa_c => -- ISA and extensions
        csr_rdata(00) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_A);     -- A CPU extension
        csr_rdata(01) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_B);     -- B CPU extension
        csr_rdata(02) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_C);     -- C CPU extension
        csr_rdata(04) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_E);     -- E CPU extension
        csr_rdata(08) <= bool_to_ulogic_f(not CPU_EXTENSION_RISCV_E); -- I CPU extension (if not E)
        csr_rdata(12) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_M);     -- M CPU extension
        csr_rdata(20) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_U);     -- U CPU extension
        csr_rdata(23) <= '1';                                         -- X CPU extension (non-standard extensions / NEORV32-specific)
        csr_rdata(31 downto 30) <= "01"; -- MXL = 32

      when csr_mie_c => -- machine interrupt-enable register
        csr_rdata(03) <= csr.mie_msi;
        csr_rdata(07) <= csr.mie_mti;
        csr_rdata(11) <= csr.mie_mei;
        csr_rdata(31 downto 16) <= csr.mie_firq;

      when csr_mtvec_c => -- machine trap-handler base address
        csr_rdata <= csr.mtvec;

      when csr_mcounteren_c => -- machine counter enable register
        if (CPU_EXTENSION_RISCV_U = true) then
          if (CPU_EXTENSION_RISCV_Zicntr = true) then
            csr_rdata(0) <= csr.mcounteren; -- cycle
            csr_rdata(2) <= csr.mcounteren; -- instret
          end if;
          if (CPU_EXTENSION_RISCV_Zihpm = true) then
            csr_rdata(15 downto 3) <= (others => csr.mcounteren); -- hpmcounter
          end if;
        end if;

      -- --------------------------------------------------------------------
      -- machine configuration --
      -- --------------------------------------------------------------------
--    when csr_menvcfg_c  => csr_rdata <= (others => '0'); -- hardwired to zero
--    when csr_menvcfgh_c => csr_rdata <= (others => '0'); -- hardwired to zero

      -- --------------------------------------------------------------------
      -- machine trap handling --
      -- --------------------------------------------------------------------
      when csr_mscratch_c => -- machine scratch register
        csr_rdata <= csr.mscratch;

      when csr_mepc_c => -- machine exception program counter
        csr_rdata <= csr.mepc(XLEN-1 downto 1) & '0';

      when csr_mcause_c => -- machine trap cause
        csr_rdata(31)         <= csr.mcause(5);
        csr_rdata(4 downto 0) <= csr.mcause(4 downto 0);

      when csr_mtval_c => -- machine trap value
        csr_rdata <= csr.mtval;

      when csr_mip_c => -- machine interrupt pending
        csr_rdata(03)           <= trap_ctrl.irq_pnd(irq_msi_irq_c);
        csr_rdata(07)           <= trap_ctrl.irq_pnd(irq_mti_irq_c);
        csr_rdata(11)           <= trap_ctrl.irq_pnd(irq_mei_irq_c);
        csr_rdata(31 downto 16) <= trap_ctrl.irq_pnd(irq_firq_15_c downto irq_firq_0_c);

      when csr_mtinst_c => -- machine trap instruction
        csr_rdata <= csr.mtinst;

      -- --------------------------------------------------------------------
      -- machine counter setup --
      -- --------------------------------------------------------------------
      when csr_mcountinhibit_c => -- machine counter-inhibit register
        if (CPU_EXTENSION_RISCV_Zicntr = true) then
          csr_rdata(0) <= csr.mcountinhibit(0); -- [m]cycle[h]
          csr_rdata(2) <= csr.mcountinhibit(2); -- [m]instret[h]
        end if;
        if (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_num_c > 0) then
          for i in 3 to (hpm_num_c+3)-1 loop
            csr_rdata(i) <= csr.mcountinhibit(i); -- [m]hpmcounter*[h]
          end loop;
        end if;

      -- HPM event configuration --
      when csr_mhpmevent3_c  => if (hpm_num_c > 00) then csr_rdata <= hpmevent_rd(03); end if;
      when csr_mhpmevent4_c  => if (hpm_num_c > 01) then csr_rdata <= hpmevent_rd(04); end if;
      when csr_mhpmevent5_c  => if (hpm_num_c > 02) then csr_rdata <= hpmevent_rd(05); end if;
      when csr_mhpmevent6_c  => if (hpm_num_c > 03) then csr_rdata <= hpmevent_rd(06); end if;
      when csr_mhpmevent7_c  => if (hpm_num_c > 04) then csr_rdata <= hpmevent_rd(07); end if;
      when csr_mhpmevent8_c  => if (hpm_num_c > 05) then csr_rdata <= hpmevent_rd(08); end if;
      when csr_mhpmevent9_c  => if (hpm_num_c > 06) then csr_rdata <= hpmevent_rd(09); end if;
      when csr_mhpmevent10_c => if (hpm_num_c > 07) then csr_rdata <= hpmevent_rd(10); end if;
      when csr_mhpmevent11_c => if (hpm_num_c > 08) then csr_rdata <= hpmevent_rd(11); end if;
      when csr_mhpmevent12_c => if (hpm_num_c > 09) then csr_rdata <= hpmevent_rd(12); end if;
      when csr_mhpmevent13_c => if (hpm_num_c > 10) then csr_rdata <= hpmevent_rd(13); end if;
      when csr_mhpmevent14_c => if (hpm_num_c > 11) then csr_rdata <= hpmevent_rd(14); end if;
      when csr_mhpmevent15_c => if (hpm_num_c > 12) then csr_rdata <= hpmevent_rd(15); end if;

      -- --------------------------------------------------------------------
      -- counters and timers --
      -- --------------------------------------------------------------------
      -- low word --
      when csr_mcycle_c        | csr_cycle_c        => if (CPU_EXTENSION_RISCV_Zicntr) then csr_rdata <= cnt_lo_rd(00); end if;
      when csr_minstret_c      | csr_instret_c      => if (CPU_EXTENSION_RISCV_Zicntr) then csr_rdata <= cnt_lo_rd(02); end if;
      when csr_mhpmcounter3_c  | csr_hpmcounter3_c  => if (hpm_num_c > 00) then csr_rdata <= cnt_lo_rd(03); end if;
      when csr_mhpmcounter4_c  | csr_hpmcounter4_c  => if (hpm_num_c > 01) then csr_rdata <= cnt_lo_rd(04); end if;
      when csr_mhpmcounter5_c  | csr_hpmcounter5_c  => if (hpm_num_c > 02) then csr_rdata <= cnt_lo_rd(05); end if;
      when csr_mhpmcounter6_c  | csr_hpmcounter6_c  => if (hpm_num_c > 03) then csr_rdata <= cnt_lo_rd(06); end if;
      when csr_mhpmcounter7_c  | csr_hpmcounter7_c  => if (hpm_num_c > 04) then csr_rdata <= cnt_lo_rd(07); end if;
      when csr_mhpmcounter8_c  | csr_hpmcounter8_c  => if (hpm_num_c > 05) then csr_rdata <= cnt_lo_rd(08); end if;
      when csr_mhpmcounter9_c  | csr_hpmcounter9_c  => if (hpm_num_c > 06) then csr_rdata <= cnt_lo_rd(09); end if;
      when csr_mhpmcounter10_c | csr_hpmcounter10_c => if (hpm_num_c > 07) then csr_rdata <= cnt_lo_rd(10); end if;
      when csr_mhpmcounter11_c | csr_hpmcounter11_c => if (hpm_num_c > 08) then csr_rdata <= cnt_lo_rd(11); end if;
      when csr_mhpmcounter12_c | csr_hpmcounter12_c => if (hpm_num_c > 09) then csr_rdata <= cnt_lo_rd(12); end if;
      when csr_mhpmcounter13_c | csr_hpmcounter13_c => if (hpm_num_c > 10) then csr_rdata <= cnt_lo_rd(13); end if;
      when csr_mhpmcounter14_c | csr_hpmcounter14_c => if (hpm_num_c > 11) then csr_rdata <= cnt_lo_rd(14); end if;
      when csr_mhpmcounter15_c | csr_hpmcounter15_c => if (hpm_num_c > 12) then csr_rdata <= cnt_lo_rd(15); end if;

      -- high word --
      when csr_mcycleh_c        | csr_cycleh_c        => if (CPU_EXTENSION_RISCV_Zicntr) then csr_rdata <= cnt_hi_rd(00); end if;
      when csr_minstreth_c      | csr_instreth_c      => if (CPU_EXTENSION_RISCV_Zicntr) then csr_rdata <= cnt_hi_rd(02); end if;
      when csr_mhpmcounter3h_c  | csr_hpmcounter3h_c  => if (hpm_num_c > 00) then csr_rdata <= cnt_hi_rd(03); end if;
      when csr_mhpmcounter4h_c  | csr_hpmcounter4h_c  => if (hpm_num_c > 01) then csr_rdata <= cnt_hi_rd(04); end if;
      when csr_mhpmcounter5h_c  | csr_hpmcounter5h_c  => if (hpm_num_c > 02) then csr_rdata <= cnt_hi_rd(05); end if;
      when csr_mhpmcounter6h_c  | csr_hpmcounter6h_c  => if (hpm_num_c > 03) then csr_rdata <= cnt_hi_rd(06); end if;
      when csr_mhpmcounter7h_c  | csr_hpmcounter7h_c  => if (hpm_num_c > 04) then csr_rdata <= cnt_hi_rd(07); end if;
      when csr_mhpmcounter8h_c  | csr_hpmcounter8h_c  => if (hpm_num_c > 05) then csr_rdata <= cnt_hi_rd(08); end if;
      when csr_mhpmcounter9h_c  | csr_hpmcounter9h_c  => if (hpm_num_c > 06) then csr_rdata <= cnt_hi_rd(09); end if;
      when csr_mhpmcounter10h_c | csr_hpmcounter10h_c => if (hpm_num_c > 07) then csr_rdata <= cnt_hi_rd(10); end if;
      when csr_mhpmcounter11h_c | csr_hpmcounter11h_c => if (hpm_num_c > 08) then csr_rdata <= cnt_hi_rd(11); end if;
      when csr_mhpmcounter12h_c | csr_hpmcounter12h_c => if (hpm_num_c > 09) then csr_rdata <= cnt_hi_rd(12); end if;
      when csr_mhpmcounter13h_c | csr_hpmcounter13h_c => if (hpm_num_c > 10) then csr_rdata <= cnt_hi_rd(13); end if;
      when csr_mhpmcounter14h_c | csr_hpmcounter14h_c => if (hpm_num_c > 11) then csr_rdata <= cnt_hi_rd(14); end if;
      when csr_mhpmcounter15h_c | csr_hpmcounter15h_c => if (hpm_num_c > 12) then csr_rdata <= cnt_hi_rd(15); end if;

      -- --------------------------------------------------------------------
      -- machine information registers --
      -- --------------------------------------------------------------------
      when csr_mvendorid_c  => csr_rdata <= VENDOR_ID; -- vendor's JEDEC ID
      when csr_marchid_c    => csr_rdata(4 downto 0) <= "10011"; -- architecture ID - official RISC-V open-source arch ID
      when csr_mimpid_c     => csr_rdata <= hw_version_c; -- implementation ID -- NEORV32 hardware version
      when csr_mhartid_c    => csr_rdata <= HART_ID; -- hardware thread ID
--    when csr_mconfigptr_c => csr_rdata <= (others => '0'); -- machine configuration pointer register - hardwired to zero

      -- --------------------------------------------------------------------
      -- debug mode CSRs --
      -- --------------------------------------------------------------------
      when csr_dcsr_c      => if (CPU_EXTENSION_RISCV_Sdext) then csr_rdata <= csr.dcsr_rd;   end if; -- debug mode control and status
      when csr_dpc_c       => if (CPU_EXTENSION_RISCV_Sdext) then csr_rdata <= csr.dpc;       end if; -- debug mode program counter
      when csr_dscratch0_c => if (CPU_EXTENSION_RISCV_Sdext) then csr_rdata <= csr.dscratch0; end if; -- debug mode scratch register 0

      -- --------------------------------------------------------------------
      -- trigger module CSRs --
      -- --------------------------------------------------------------------
--    when csr_tselect_c => if (CPU_EXTENSION_RISCV_Sdtrig) then csr_rdata <= (others => '0'); end if; -- hardwired to zero = only 1 trigger available
      when csr_tdata1_c  => if (CPU_EXTENSION_RISCV_Sdtrig) then csr_rdata <= csr.tdata1_rd;   end if; -- match control
      when csr_tdata2_c  => if (CPU_EXTENSION_RISCV_Sdtrig) then csr_rdata <= csr.tdata2;      end if; -- address-compare
      when csr_tinfo_c => -- trigger information
        if (CPU_EXTENSION_RISCV_Sdtrig) then
          csr_rdata(31 downto 24) <= x"01"; -- Sdtrig ISA spec. version 1.0
          csr_rdata(15 downto 00) <= x"0006"; -- mcontrol6 type trigger only
        end if;

      -- --------------------------------------------------------------------
      -- NEORV32-specific (RISC-V "custom") read-only CSRs --
      -- --------------------------------------------------------------------
      -- machine extended ISA extensions information --
      when csr_mxisa_c =>
        -- extended ISA (sub-)extensions --
        csr_rdata(00) <= '1';                                          -- Zicsr: CSR access (always enabled)
        csr_rdata(01) <= '1';                                          -- Zifencei: instruction stream sync. (always enabled)
        csr_rdata(02) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zmmul);  -- Zmmul: mul/div
        csr_rdata(03) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zxcfu);  -- Zxcfu: custom RISC-V instructions
        csr_rdata(04) <= '0';                                          -- reserved
        csr_rdata(05) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zfinx);  -- Zfinx: FPU using x registers
        csr_rdata(06) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicond); -- Zicond: integer conditional operations
        csr_rdata(07) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicntr); -- Zicntr: base counters
        csr_rdata(08) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Smpmp);  -- Smpmp: physical memory protection
        csr_rdata(09) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zihpm);  -- Zihpm: hardware performance monitors
        csr_rdata(10) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Sdext);  -- Sdext: RISC-V (external) debug mode
        csr_rdata(11) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Sdtrig); -- Sdtrig: trigger module
        -- misc --
        csr_rdata(20) <= bool_to_ulogic_f(is_simulation_c);            -- is this a simulation?
        -- tuning options --
        csr_rdata(29) <= bool_to_ulogic_f(REGFILE_HW_RST);             -- full hardware reset of register file
        csr_rdata(30) <= bool_to_ulogic_f(FAST_MUL_EN);                -- DSP-based multiplication (M extensions only)
        csr_rdata(31) <= bool_to_ulogic_f(FAST_SHIFT_EN);              -- parallel logic for shifts (barrel shifters)

      -- --------------------------------------------------------------------
      -- undefined/unavailable (or implemented externally) --
      -- --------------------------------------------------------------------
      when others => NULL; -- use default: read as zero

    end case;
  end process csr_read_access;


  -- CSR read-data gate --
  csr_read_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      csr.re    <= '0';
      csr.rdata <= (others => '0');
    elsif rising_edge(clk_i) then
      csr.re <= csr.re_nxt;
      if (csr.re = '1') then
        csr.rdata <= csr_rdata or xcsr_rdata;
      else
        csr.rdata <= (others => '0'); -- output zero if no valid CSR read access operation
      end if;
    end if;
  end process csr_read_reg;

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
    -- [NOTE] no need to check bits 6:4 of the address as they're always zero (checked by illegal CSR logic)
    if (csr.we = '1') and (csr.addr(11 downto 8) = csr_mcycle_c(11 downto 8)) then
      if (csr.addr(7) = '0') then -- low word
        cnt.we_lo(to_integer(unsigned(csr.addr(3 downto 0)))) <= '1';
      else -- high word
        cnt.we_hi(to_integer(unsigned(csr.addr(3 downto 0)))) <= '1';
      end if;
    end if;
  end process cnt_we;


  -- hardware counters --
  cpu_counter_gen:
  for i in 0 to 2+hpm_num_c generate

    -- counter CSRs --
    cnt_reg: process(rstn_i, clk_i)
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
    end process cnt_reg;

    -- low-word increment --
    cnt.nxt(i) <= std_ulogic_vector(unsigned('0' & cnt.lo(i)) + 1) when (cnt.inc(i) = '1') else
                  std_ulogic_vector(unsigned('0' & cnt.lo(i)) + 0);

  end generate;


  -- read-back --
  cnt_connect: process(cnt)
  begin
    cnt_lo_rd <= (others => (others => '0'));
    cnt_hi_rd <= (others => (others => '0'));
    -- base counters --
    if (CPU_EXTENSION_RISCV_Zicntr = true) then
      cnt_lo_rd(0) <= cnt.lo(0); -- cycle
      cnt_hi_rd(0) <= cnt.hi(0); -- cycleh
      cnt_lo_rd(2) <= cnt.lo(2); -- instret
      cnt_hi_rd(2) <= cnt.hi(2); -- instreth
    end if;
    -- hpm counters --
    if (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_num_c > 0) then
      for i in 3 to (hpm_num_c+3)-1 loop
        if (hpm_cnt_lo_width_c > 0) then -- constrain low word size
          cnt_lo_rd(i)(hpm_cnt_lo_width_c-1 downto 0) <= cnt.lo(i)(hpm_cnt_lo_width_c-1 downto 0);
        end if;
        if (hpm_cnt_hi_width_c > 0) then -- constrain high word size
          cnt_hi_rd(i)(hpm_cnt_hi_width_c-1 downto 0) <= cnt.hi(i)(hpm_cnt_hi_width_c-1 downto 0);
        end if;
      end loop;
    end if;
  end process cnt_connect;


  -- Hardware Performance Monitors (HPM) - Counter Event Configuration CSRs -----------------
  -- -------------------------------------------------------------------------------------------
  hpmevent_gen_enable:
  if CPU_EXTENSION_RISCV_Zihpm and (hpm_num_c > 0) generate

    -- write enable decoder --
    hpmevent_write: process(csr)
    begin
      hpmevent_we <= (others => '0');
      -- [NOTE] no need to check bit 4 of the address as it's always zero (checked by illegal CSR logic)
      if (csr.addr(11 downto 5) = csr_mcountinhibit_c(11 downto 5)) and (csr.we = '1') then
        hpmevent_we(to_integer(unsigned(csr.addr(3 downto 0)))) <= '1';
      end if;
    end process hpmevent_write;

    -- event registers --
    hpmevent_reg_gen:
    for i in 3 to (hpm_num_c+3)-1 generate
      hpmevent_reg: process(rstn_i, clk_i)
      begin
        if (rstn_i = '0') then
          hpmevent_cfg(i) <= (others => '0');
        elsif rising_edge(clk_i) then
          if (hpmevent_we(i) = '1') then
            hpmevent_cfg(i) <= csr.wdata(hpmcnt_event_size_c-1 downto 0);
          end if;
          hpmevent_cfg(i)(hpmcnt_event_tm_c) <= '0'; -- time, unused/reserved
        end if;
      end process hpmevent_reg;
      -- read-back --
      hpmevent_rd(i)(XLEN-1 downto hpmcnt_event_size_c) <= (others => '0');
      hpmevent_rd(i)(hpmcnt_event_size_c-1 downto 0)    <= hpmevent_cfg(i);
    end generate;

    -- terminate unused entries --
    hpmevent_terminate_gen:
    for i in hpm_num_c+3 to 15 generate
      hpmevent_cfg(i) <= (others => '0');
      hpmevent_rd(i)  <= (others => '0');
    end generate;

  end generate;


  -- no HPMs implemented --
  hpmevent_gen_disable:
  if (not CPU_EXTENSION_RISCV_Zihpm) or (hpm_num_c = 0) generate
    hpmevent_cfg <= (others => (others => '0'));
    hpmevent_rd  <= (others => (others => '0'));
  end generate;


  -- Counter Increment Control (Trigger Events) ---------------------------------------------
  -- -------------------------------------------------------------------------------------------
  counter_event: process(rstn_i, clk_i)
  begin -- increment if an enabled event fires; do not increment if CPU is in debug mode or if counter is inhibited
    if (rstn_i = '0') then
      cnt.inc <= (others => '0');
    elsif rising_edge(clk_i) then
      cnt.inc <= (others => '0'); -- default
      -- base counters --
      if (CPU_EXTENSION_RISCV_Zicntr = true) then
        cnt.inc(0) <= cnt_event(hpmcnt_event_cy_c) and (not csr.mcountinhibit(0)) and (not debug_ctrl.running);
        cnt.inc(2) <= cnt_event(hpmcnt_event_ir_c) and (not csr.mcountinhibit(2)) and (not debug_ctrl.running);
      end if;
      -- hpm counters --
      if (CPU_EXTENSION_RISCV_Zihpm = true) and (hpm_num_c > 0) then
        for i in 3 to (hpm_num_c+3)-1 loop
          cnt.inc(i) <= or_reduce_f(cnt_event and hpmevent_cfg(i)) and (not csr.mcountinhibit(i)) and (not debug_ctrl.running);
        end loop;
      end if;
    end if;
  end process counter_event;

  -- RISC-V-specific base counter events (for HPM and base counters) --
  cnt_event(hpmcnt_event_cy_c) <= '1' when (sleep_mode = '0')               else '0'; -- cycle: active cycle
  cnt_event(hpmcnt_event_tm_c) <=                                                '0'; -- time: unused/reserved
  cnt_event(hpmcnt_event_ir_c) <= '1' when (execute_engine.state = EXECUTE) else '0'; -- instret: retired (==executed) instruction

  -- NEORV32-specific counter events (for HPM counters only) --
  cnt_event(hpmcnt_event_compr_c)    <= '1' when (execute_engine.state = EXECUTE)  and (execute_engine.is_ci = '1')  else '0'; -- executed compressed instruction
  cnt_event(hpmcnt_event_wait_dis_c) <= '1' when (execute_engine.state = DISPATCH) and (issue_engine.valid   = "00") else '0'; -- instruction dispatch wait cycle
  cnt_event(hpmcnt_event_wait_alu_c) <= '1' when (execute_engine.state = ALU_WAIT)                                   else '0'; -- multi-cycle ALU co-processor wait cycle
  cnt_event(hpmcnt_event_branch_c)   <= '1' when (execute_engine.state = BRANCH)                                     else '0'; -- executed branch instruction
  cnt_event(hpmcnt_event_branched_c) <= '1' when (execute_engine.state = BRANCHED)                                   else '0'; -- control flow transfer
  cnt_event(hpmcnt_event_load_c)     <= '1' when (ctrl.lsu_req = '1') and (ctrl.lsu_rw = '0')                        else '0'; -- executed load operation
  cnt_event(hpmcnt_event_store_c)    <= '1' when (ctrl.lsu_req = '1') and (ctrl.lsu_rw = '1')                        else '0'; -- executed store operation
  cnt_event(hpmcnt_event_wait_lsu_c) <= '1' when (ctrl.lsu_req = '0') and (execute_engine.state = MEM_WAIT)          else '0'; -- load/store unit memory wait cycle
  cnt_event(hpmcnt_event_trap_c)     <= '1' when (trap_ctrl.env_enter = '1')                                         else '0'; -- entered trap


-- ****************************************************************************************************************************
-- CPU Debug Mode (Part of the On-Chip Debugger)
-- ****************************************************************************************************************************

  -- Debug Control --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  debug_mode_enable:
  if CPU_EXTENSION_RISCV_Sdext generate

    debug_control: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        debug_ctrl.running <= '0';
      elsif rising_edge(clk_i) then
        if (debug_ctrl.running = '0') then -- debug mode OFFLINE
          if (trap_ctrl.env_enter = '1') and (trap_ctrl.cause(5) = '1') then -- waiting for entry event
            debug_ctrl.running <= '1';
          end if;
        else -- debug mode ONLINE
          if (trap_ctrl.env_exit = '1') then -- waiting for exit event
            debug_ctrl.running <= '0';
          end if;
        end if;
      end if;
    end process debug_control;

    -- debug mode entry triggers --
    debug_ctrl.trig_hw    <= trap_ctrl.hwtrig and (not debug_ctrl.running) and csr.tdata1_action and csr.tdata1_dmode; -- enter debug mode by HW trigger module request (only valid if dmode = 1)
    debug_ctrl.trig_break <= trap_ctrl.ebreak and (debug_ctrl.running or   -- re-enter debug mode
                             ((    csr.privilege) and csr.dcsr_ebreakm) or -- enabled goto-debug-mode in machine mode on "ebreak"
                             ((not csr.privilege) and csr.dcsr_ebreaku));  -- enabled goto-debug-mode in user mode on "ebreak"
    debug_ctrl.trig_halt  <= db_halt_req_i and (not debug_ctrl.running); -- external halt request (if not halted already)
    debug_ctrl.trig_step  <= csr.dcsr_step and (not debug_ctrl.running); -- single-step mode (trigger when NOT CURRENTLY in debug mode)

  end generate;

  -- Sdext ISA extension not enabled --
  debug_mode_disable:
  if not CPU_EXTENSION_RISCV_Sdext generate
    debug_ctrl.running    <= '0';
    debug_ctrl.trig_hw    <= '0';
    debug_ctrl.trig_break <= '0';
    debug_ctrl.trig_halt  <= '0';
    debug_ctrl.trig_step  <= '0';
  end generate;


  -- Debug Control and Status Register (dcsr) - Read-Back -----------------------------------
  -- -------------------------------------------------------------------------------------------
  csr.dcsr_rd(31 downto 28) <= "0100"; -- xdebugver: external debug support compatible to spec. version 1.0
  csr.dcsr_rd(27 downto 16) <= (others => '0'); -- reserved
  csr.dcsr_rd(15)           <= csr.dcsr_ebreakm; -- ebreakm: what happens on ebreak in m-mode? (normal trap OR debug-enter)
  csr.dcsr_rd(14)           <= '0'; -- reserved
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
  trigger_module_enable:
  if CPU_EXTENSION_RISCV_Sdtrig generate

    -- trigger on instruction address match (trigger right BEFORE execution) --
    hw_trigger_match <= '1' when (csr.tdata1_execute = '1') and -- trigger enabled to match on instruction address
                                 (hw_trigger_fired = '0') and -- trigger has not fired yet
                                 (csr.tdata2(XLEN-1 downto 1) = execute_engine.next_pc(XLEN-1 downto 1)) -- address match
                                 else '0';

    -- status flag - set when trigger has fired --
    hw_trigger_exception: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        hw_trigger_fired <= '0';
      elsif rising_edge(clk_i) then
        if (hw_trigger_fired = '0') then
          hw_trigger_fired <= hw_trigger_match and trap_ctrl.exc_buf(exc_ebreak_c); -- trigger has fired and breakpoint exception is pending
        elsif (csr.we = '1') and (csr.addr = csr_tdata1_c) and (csr.wdata(22) = '0') then -- tdata1 write access
          hw_trigger_fired <= '0';
        end if;
      end if;
    end process hw_trigger_exception;

  end generate;

  -- Sdtrig ISA extension not enabled --
  trigger_module_disable:
  if not CPU_EXTENSION_RISCV_Sdtrig generate
    hw_trigger_match <= '0';
    hw_trigger_fired <= '0';
  end generate;


  -- Match Control CSR (mcontrol6 @ tdata1) - Read-Back -------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr.tdata1_rd(31 downto 28) <= x"6"; -- type: address match trigger (mcontrol6)
  csr.tdata1_rd(27)           <= csr.tdata1_dmode; -- dmode: set to ignore machine-mode write access to tdata* CSRs
  csr.tdata1_rd(26)           <= '0'; -- uncertain: trigger satisfies the configured conditions
  csr.tdata1_rd(25)           <= '0'; -- hit1: hardwired to zero
  csr.tdata1_rd(24)           <= '0'; -- vs: VS-mode not implemented
  csr.tdata1_rd(23)           <= '0'; -- vu: VU-mode not implemented
  csr.tdata1_rd(22)           <= hw_trigger_fired; -- hit0: set when trigger has fired
  csr.tdata1_rd(21)           <= '0'; -- select: only address matching is supported
  csr.tdata1_rd(20 downto 19) <= "00"; -- reserved
  csr.tdata1_rd(18 downto 16) <= "000"; -- size: match accesses of any size
  csr.tdata1_rd(15 downto 12) <= "000" & csr.tdata1_action; -- action = 1: enter debug mode on trigger, action = 0: ebreak exception on trigger
  csr.tdata1_rd(11)           <= '0'; -- chain: chaining not supported - there is only one trigger
  csr.tdata1_rd(10 downto 07) <= "0000"; -- match: equal-match only
  csr.tdata1_rd(6)            <= '1'; -- m: trigger always enabled when in machine-mode
  csr.tdata1_rd(5)            <= '0'; -- uncertainen: hardwired to zero
  csr.tdata1_rd(4)            <= '0'; -- s: supervisor-mode not supported
  csr.tdata1_rd(3)            <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_U); -- u: trigger always enabled when in user-mode (if implemented)
  csr.tdata1_rd(2)            <= csr.tdata1_execute; -- execute: enable trigger on instruction match
  csr.tdata1_rd(1)            <= '0'; -- store: store address or data matching not supported
  csr.tdata1_rd(0)            <= '0'; -- load: load address or data matching not supported


end neorv32_cpu_control_rtl;
