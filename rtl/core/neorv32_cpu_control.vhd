-- #################################################################################################
-- # << NEORV32 - CPU Control >>                                                                   #
-- # ********************************************************************************************* #
-- # CPU operation is split into a fetch engine (responsible for fetching instruction data), an    #
-- # issue engine (for recoding compressed instructions and for constructing 32-bit instruction    #
-- # words) and an execute engine (responsible for actually executing the instructions), a trap    #
-- # handling controller and the RISC-V status and control register set (CSRs).                    #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2020, Stephan Nolting. All rights reserved.                                     #
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
    HW_THREAD_ID                 : std_ulogic_vector(31 downto 0):= x"00000000"; -- hardware thread id
    CPU_BOOT_ADDR                : std_ulogic_vector(31 downto 0):= x"00000000"; -- cpu boot address
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        : boolean := false; -- implement atomic extension?
    CPU_EXTENSION_RISCV_C        : boolean := false; -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        : boolean := false; -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        : boolean := false; -- implement muld/div extension?
    CPU_EXTENSION_RISCV_U        : boolean := false; -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zicsr    : boolean := true;  -- implement CSR system?
    CPU_EXTENSION_RISCV_Zifencei : boolean := true;  -- implement instruction stream sync.?
    -- Physical memory protection (PMP) --
    PMP_USE                      : boolean := false  -- implement physical memory protection?
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
    -- interrupts (risc-v compliant) --
    msw_irq_i     : in  std_ulogic; -- machine software interrupt
    mext_irq_i    : in  std_ulogic; -- machine external interrupt
    mtime_irq_i   : in  std_ulogic; -- machine timer interrupt
    -- fast interrupts (custom) --
    firq_i        : in  std_ulogic_vector(3 downto 0);
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

  -- instruction execution engine --
  type execute_engine_state_t is (SYS_WAIT, DISPATCH, TRAP, EXECUTE, ALU_WAIT, BRANCH, FENCE_OP, LOADSTORE_0, LOADSTORE_1, LOADSTORE_2, SYS_ENV, CSR_ACCESS);
  type execute_engine_t is record
    state        : execute_engine_state_t;
    state_nxt    : execute_engine_state_t;
    --
    i_reg        : std_ulogic_vector(31 downto 0);
    i_reg_nxt    : std_ulogic_vector(31 downto 0);
    i_reg_last   : std_ulogic_vector(31 downto 0); -- last executed instruction
    --
    is_ci        : std_ulogic; -- current instruction is de-compressed instruction
    is_ci_nxt    : std_ulogic;
    is_cp_op     : std_ulogic; -- current instruction is a co-processor operation
    is_cp_op_nxt : std_ulogic;
    --
    branch_taken : std_ulogic; -- branch condition fullfilled
    pc           : std_ulogic_vector(data_width_c-1 downto 0); -- actual PC, corresponding to current executed instruction
    pc_mux_sel   : std_ulogic_vector(1 downto 0); -- source select for PC update
    pc_we        : std_ulogic; -- PC update enabled
    next_pc      : std_ulogic_vector(data_width_c-1 downto 0); -- next PC, corresponding to next instruction to be executed
    last_pc      : std_ulogic_vector(data_width_c-1 downto 0); -- PC of last executed instruction
    --
    sleep        : std_ulogic; -- CPU in sleep mode
    sleep_nxt    : std_ulogic;
    if_rst       : std_ulogic; -- instruction fetch was reset
    if_rst_nxt   : std_ulogic;
  end record;
  signal execute_engine : execute_engine_t;

  -- trap controller --
  type trap_ctrl_t is record
    exc_buf       : std_ulogic_vector(exception_width_c-1 downto 0);
    exc_fire      : std_ulogic; -- set if there is a valid source in the exception buffer
    irq_buf       : std_ulogic_vector(interrupt_width_c-1 downto 0);
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
  type pmp_ctrl_t is array (0 to pmp_max_r_c-1) of std_ulogic_vector(7 downto 0);
  type pmp_addr_t is array (0 to pmp_max_r_c-1) of std_ulogic_vector(data_width_c-1 downto 0);
  type csr_t is record
    we           : std_ulogic; -- csr write enable
    we_nxt       : std_ulogic;
    re           : std_ulogic; -- csr read enable
    re_nxt       : std_ulogic;
    wdata        : std_ulogic_vector(data_width_c-1 downto 0); -- csr write data
    rdata        : std_ulogic_vector(data_width_c-1 downto 0); -- csr read data
    --
    mstatus_mie  : std_ulogic; -- mstatus.MIE: global IRQ enable (R/W)
    mstatus_mpie : std_ulogic; -- mstatus.MPIE: previous global IRQ enable (R/W)
    mstatus_mpp  : std_ulogic_vector(1 downto 0); -- mstatus.MPP: machine previous privilege mode
    --
    mie_msie     : std_ulogic; -- mie.MSIE: machine software interrupt enable (R/W)
    mie_meie     : std_ulogic; -- mie.MEIE: machine external interrupt enable (R/W)
    mie_mtie     : std_ulogic; -- mie.MEIE: machine timer interrupt enable (R/W)
    mie_firqe    : std_ulogic_vector(3 downto 0); -- mie.firq*e: fast interrupt enabled (R/W)
    --
    privilege    : std_ulogic_vector(1 downto 0); -- hart's current privilege mode
    priv_m_mode  : std_ulogic; -- CPU in M-mode
    priv_u_mode  : std_ulogic; -- CPU in u-mode
    --
    mepc         : std_ulogic_vector(data_width_c-1 downto 0); -- mepc: machine exception pc (R/W)
    mcause       : std_ulogic_vector(data_width_c-1 downto 0); -- mcause: machine trap cause (R/W)
    mtvec        : std_ulogic_vector(data_width_c-1 downto 0); -- mtvec: machine trap-handler base address (R/W), bit 1:0 == 00
    mtval        : std_ulogic_vector(data_width_c-1 downto 0); -- mtval: machine bad address or isntruction (R/W)
    mscratch     : std_ulogic_vector(data_width_c-1 downto 0); -- mscratch: scratch register (R/W)
    mcycle       : std_ulogic_vector(32 downto 0); -- mcycle (R/W), plus carry bit
    minstret     : std_ulogic_vector(32 downto 0); -- minstret (R/W), plus carry bit
    mcycleh      : std_ulogic_vector(31 downto 0); -- mcycleh (R/W)
    minstreth    : std_ulogic_vector(31 downto 0); -- minstreth (R/W)
    pmpcfg       : pmp_ctrl_t; -- physical memory protection - configuration registers
    pmpaddr      : pmp_addr_t; -- physical memory protection - address registers
  end record;
  signal csr : csr_t;

  signal mcycle_msb   : std_ulogic;
  signal minstret_msb : std_ulogic;

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
      fetch_engine.state <= IFETCH_RESET;
      fetch_engine.pc    <= (others => '0');
    elsif rising_edge(clk_i) then
      if (fetch_engine.reset = '1') then
        fetch_engine.state <= IFETCH_RESET;
      else
        fetch_engine.state <= fetch_engine.state_nxt;
      end if;
      fetch_engine.pc <= fetch_engine.pc_nxt;
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

      when IFETCH_RESET => -- reset engine and prefetch buffer, get appilcation PC
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
            if (execute_engine.state = DISPATCH) then
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
            if (execute_engine.state = DISPATCH) then
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

  -- 16-bit instruction: half-word select --
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
      if (execute_engine.state = BRANCH) then -- next_PC as immediate for jump-and-link operations (=return address)
        imm_o <= execute_engine.next_pc;
      else -- "normal" immediate from instruction
        case opcode_v is -- save some bits here, LSBs are always 11 for rv32
          when opcode_store_c => -- S-immediate
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
        execute_engine.branch_taken <= cmp_i(alu_cmp_equal_c);
      when funct3_bne_c => -- branch if not equal
        execute_engine.branch_taken <= not cmp_i(alu_cmp_equal_c);
      when funct3_blt_c | funct3_bltu_c => -- branch if less (signed/unsigned)
        execute_engine.branch_taken <= cmp_i(alu_cmp_less_c);
      when funct3_bge_c | funct3_bgeu_c => -- branch if greater or equal (signed/unsigned)
        execute_engine.branch_taken <= not cmp_i(alu_cmp_less_c);
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
      execute_engine.pc     <= CPU_BOOT_ADDR(data_width_c-1 downto 1) & '0';
      execute_engine.state  <= SYS_WAIT;
      execute_engine.sleep  <= '0';
      execute_engine.if_rst <= '1'; -- instruction fetch is reset after system reset
    elsif rising_edge(clk_i) then
      -- PC update --
      if (execute_engine.pc_we = '1') then
        case execute_engine.pc_mux_sel is
          when "00"   => execute_engine.pc <= execute_engine.next_pc(data_width_c-1 downto 1) & '0'; -- normal (linear) increment
          when "01"   => execute_engine.pc <= alu_add_i(data_width_c-1 downto 1) & '0'; -- jump/branch
          when "10"   => execute_engine.pc <= csr.mtvec(data_width_c-1 downto 1) & '0'; -- trap enter
          when others => execute_engine.pc <= csr.mepc(data_width_c-1 downto 1) & '0'; -- trap return
        end case;
      end if;
      --
      execute_engine.state  <= execute_engine.state_nxt;
      execute_engine.sleep  <= execute_engine.sleep_nxt;
      execute_engine.if_rst <= execute_engine.if_rst_nxt;
    end if;
  end process execute_engine_fsm_sync_rst;


  -- for registers that do NOT require a specific reset state --
  execute_engine_fsm_sync: process(clk_i)
  begin
    if rising_edge(clk_i) then
      execute_engine.i_reg    <= execute_engine.i_reg_nxt;
      execute_engine.is_ci    <= execute_engine.is_ci_nxt;
      execute_engine.is_cp_op <= execute_engine.is_cp_op_nxt;
      -- next PC (next linear instruction) --
      if (execute_engine.state = EXECUTE) then
        if (execute_engine.is_ci = '1') then -- compressed instruction?
          execute_engine.next_pc <= std_ulogic_vector(unsigned(execute_engine.pc) + 2);
        else
          execute_engine.next_pc <= std_ulogic_vector(unsigned(execute_engine.pc) + 4);
        end if;
      end if;
      -- PC & IR of last "executed" instruction --
      if (execute_engine.state = EXECUTE) then
        execute_engine.last_pc    <= execute_engine.pc;
        execute_engine.i_reg_last <= execute_engine.i_reg;
      end if;
      -- main control bus --
      ctrl <= ctrl_nxt;
    end if;
  end process execute_engine_fsm_sync;

  -- PC output --
  curr_pc_o <= execute_engine.pc(data_width_c-1 downto 1) & '0'; -- PC for ALU ops


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
    ctrl_o(ctrl_bus_ierr_ack_c) <= fetch_engine.bus_err_ack;
    ctrl_o(ctrl_bus_derr_ack_c) <= trap_ctrl.env_start_ack;
    -- instruction's function blocks (for co-processors) --
    ctrl_o(ctrl_ir_funct12_11_c downto ctrl_ir_funct12_0_c) <= execute_engine.i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c);
    ctrl_o(ctrl_ir_funct3_2_c   downto ctrl_ir_funct3_0_c)  <= execute_engine.i_reg(instr_funct3_msb_c  downto instr_funct3_lsb_c);
    -- locked bus operation (for atomica memory operations) --
    ctrl_o(ctrl_bus_lock_c) <= atomic_ctrl.lock; -- (bus) lock status
  end process ctrl_output;


  -- Execute Engine FSM Comb ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  execute_engine_fsm_comb: process(execute_engine, fetch_engine, cmd_issue, trap_ctrl, csr, ctrl, csr_acc_valid,
                                   alu_wait_i, bus_d_wait_i, ma_load_i, be_load_i, ma_store_i, be_store_i)
    variable alu_immediate_v : std_ulogic;
    variable rs1_is_r0_v     : std_ulogic;
    variable opcode_v        : std_ulogic_vector(6 downto 0);
    variable is_atomic_lr_v  : std_ulogic;
    variable is_atomic_sc_v  : std_ulogic;
  begin
    -- arbiter defaults --
    execute_engine.state_nxt    <= execute_engine.state;
    execute_engine.i_reg_nxt    <= execute_engine.i_reg;
    execute_engine.is_cp_op_nxt <= execute_engine.is_cp_op;
    execute_engine.is_ci_nxt    <= execute_engine.is_ci;
    execute_engine.sleep_nxt    <= execute_engine.sleep;
    execute_engine.if_rst_nxt   <= execute_engine.if_rst;
    --
    execute_engine.pc_mux_sel   <= (others => '0');
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
    if (execute_engine.i_reg(instr_opcode_lsb_c+4) = '1') then -- ALU ops
      ctrl_nxt(ctrl_alu_unsigned_c) <= execute_engine.i_reg(instr_funct3_lsb_c+0); -- unsigned ALU operation? (SLTIU, SLTU)
    else -- branches
      ctrl_nxt(ctrl_alu_unsigned_c) <= execute_engine.i_reg(instr_funct3_lsb_c+1); -- unsigned branches? (BLTU, BGEU)
    end if;
    -- memory access --
    ctrl_nxt(ctrl_bus_unsigned_c)                            <= execute_engine.i_reg(instr_funct3_msb_c); -- unsigned LOAD (LBU, LHU)
    ctrl_nxt(ctrl_bus_size_msb_c downto ctrl_bus_size_lsb_c) <= execute_engine.i_reg(instr_funct3_lsb_c+1 downto instr_funct3_lsb_c); -- mem transfer size
    -- alu.shifter --
    ctrl_nxt(ctrl_alu_shift_dir_c) <= execute_engine.i_reg(instr_funct3_msb_c); -- shift direction (left/right)
    ctrl_nxt(ctrl_alu_shift_ar_c)  <= execute_engine.i_reg(30); -- is arithmetic shift
    -- ALU main control --
    ctrl_nxt(ctrl_alu_addsub_c)                         <= '0'; -- ADD(I)
    ctrl_nxt(ctrl_alu_func1_c  downto ctrl_alu_func0_c) <= alu_func_cmd_arith_c; -- default ALU function select: arithmetic
    ctrl_nxt(ctrl_alu_arith_c)                          <= alu_arith_cmd_addsub_c; -- default ALU arithmetic operation: ADDSUB

    -- is immediate ALU operation? --
    alu_immediate_v := not execute_engine.i_reg(instr_opcode_msb_c-1);

    -- is rs1 == r0? --
    rs1_is_r0_v := not or_all_f(execute_engine.i_reg(instr_rs1_msb_c downto instr_rs1_lsb_c));

    -- is atomic load-reservate/store-conditional? --
    if (CPU_EXTENSION_RISCV_A = true) and (execute_engine.i_reg(instr_opcode_lsb_c+2) = '1') then -- valid atomic sub-opcode
      is_atomic_lr_v := not execute_engine.i_reg(instr_funct5_lsb_c);
      is_atomic_sc_v :=     execute_engine.i_reg(instr_funct5_lsb_c);
    else
      is_atomic_lr_v := '0';
      is_atomic_sc_v := '0';
    end if;


    -- state machine --
    case execute_engine.state is

      when SYS_WAIT => -- System delay cycle (used to wait for side effects to kick in) ((and to init r0 with zero if it is a physical register))
      -- ------------------------------------------------------------
        -- set reg_file's r0 to zero --
        if (rf_r0_is_reg_c = true) then -- is r0 implemented as physical register, which has to be set to zero?
          ctrl_nxt(ctrl_rf_in_mux_msb_c downto ctrl_rf_in_mux_lsb_c) <= "11"; -- RF input = CSR output (hacky! results zero since there is no valid CSR_read)
          ctrl_nxt(ctrl_rf_r0_we_c) <= '1'; -- force RF write access and force rd=r0
        end if;
        --
        execute_engine.state_nxt <= DISPATCH;


      when DISPATCH => -- Get new command from instruction issue engine
      -- ------------------------------------------------------------
        execute_engine.pc_mux_sel <= "00"; -- linear next PC
        -- IR update --
        execute_engine.is_ci_nxt <= cmd_issue.data(32); -- flag to indicate a de-compressed instruction beeing executed
        execute_engine.i_reg_nxt <= cmd_issue.data(31 downto 0);
        --
        if (cmd_issue.valid = '1') then -- instruction available?
          -- IR update - exceptions --
          trap_ctrl.instr_ma <= cmd_issue.data(33); -- misaligned instruction fetch address
          trap_ctrl.instr_be <= cmd_issue.data(34); -- bus access fault during instruction fetch
          illegal_compressed <= cmd_issue.data(35); -- invalid decompressed instruction
          -- PC update --
          execute_engine.if_rst_nxt <= '0';
          execute_engine.pc_we      <= not execute_engine.if_rst; -- update PC with linear next_pc if there was NO non-linear PC modification
          -- any reason to go to trap state? --
          if (execute_engine.sleep = '1') or (trap_ctrl.env_start = '1') or (trap_ctrl.exc_fire = '1') or ((cmd_issue.data(33) or cmd_issue.data(34)) = '1') then
            execute_engine.state_nxt <= TRAP;
          else
            execute_engine.state_nxt <= EXECUTE;
          end if;
        end if;


      when TRAP => -- Start trap environment (also used as cpu sleep state)
      -- ------------------------------------------------------------
        execute_engine.pc_mux_sel <= "10"; -- csr.mtvec (trap)
        fetch_engine.reset        <= '1';
        execute_engine.if_rst_nxt <= '1'; -- this will be a non-linear PC modification
        if (trap_ctrl.env_start = '1') then -- trap triggered?
          trap_ctrl.env_start_ack   <= '1';
          execute_engine.pc_we      <= '1';
          execute_engine.sleep_nxt  <= '0'; -- waky waky
          execute_engine.state_nxt  <= SYS_WAIT;
        end if;


      when EXECUTE => -- Decode and execute instruction (control has to be here for excatly 1 cyle in any case!)
      -- ------------------------------------------------------------
        opcode_v := execute_engine.i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c+2) & "11"; -- save some bits here, LSBs are always 11 for rv32
        case opcode_v is

          when opcode_alu_c | opcode_alui_c => -- (immediate) ALU operation
          -- ------------------------------------------------------------
            ctrl_nxt(ctrl_alu_opa_mux_c)   <= '0'; -- use RS1 as ALU.OPA
            ctrl_nxt(ctrl_alu_opb_mux_c)   <= alu_immediate_v; -- use IMM as ALU.OPB for immediate operations
            ctrl_nxt(ctrl_rf_in_mux_msb_c) <= '0'; -- RF input = ALU result

            -- ALU arithmetic operation type and ADD/SUB --
            if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_slt_c) or
               (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sltu_c) then
              ctrl_nxt(ctrl_alu_arith_c) <= alu_arith_cmd_slt_c;
            else
              ctrl_nxt(ctrl_alu_arith_c) <= alu_arith_cmd_addsub_c;
            end if;

            -- ADD/SUB --
            if ((alu_immediate_v = '0') and (execute_engine.i_reg(instr_funct7_msb_c-1) = '1')) or -- not an immediate op and funct7.6 set => SUB
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

            -- cp access? --
            ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_muldiv_c; -- just in case a mul/div operation
            if (CPU_EXTENSION_RISCV_M = true) and (execute_engine.i_reg(instr_opcode_lsb_c+5) = opcode_alu_c(5)) and (execute_engine.i_reg(instr_funct7_lsb_c) = '1') then -- MULDIV CP op?
              execute_engine.is_cp_op_nxt                        <= '1'; -- this is a CP operation
              ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_cmd_copro_c;
            -- ALU operation - function select --
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
               ((CPU_EXTENSION_RISCV_M = true) and (execute_engine.i_reg(instr_opcode_lsb_c+5) = opcode_alu_c(5)) and (execute_engine.i_reg(instr_funct7_lsb_c) = '1')) then -- MULDIV CP op?
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
            ctrl_nxt(ctrl_rf_in_mux_msb_c) <= '0'; -- RF input = ALU result
            ctrl_nxt(ctrl_rf_wb_en_c)      <= '1'; -- valid RF write-back
            execute_engine.state_nxt       <= DISPATCH;

          when opcode_load_c | opcode_store_c | opcode_atomic_c => -- load/store / atomic memory access
          -- ------------------------------------------------------------
            ctrl_nxt(ctrl_alu_opa_mux_c) <= '0'; -- use RS1 as ALU.OPA
            ctrl_nxt(ctrl_alu_opb_mux_c) <= '1'; -- use IMM as ALU.OPB
            ctrl_nxt(ctrl_bus_mo_we_c)   <= '1'; -- write to MAR and MDO (MDO only relevant for store)
            --
            if (CPU_EXTENSION_RISCV_A = false) or (execute_engine.i_reg(instr_opcode_lsb_c+2) = '0') then -- atomic (A) extension disabled or normal load/store
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
            -- compute target address --
            ctrl_nxt(ctrl_alu_arith_c) <= alu_arith_cmd_addsub_c; -- actual ALU operation = ADD
            ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_cmd_arith_c; -- actual ALU operation = ADD
            if (execute_engine.i_reg(instr_opcode_lsb_c+3 downto instr_opcode_lsb_c+2) = opcode_jalr_c(3 downto 2)) then -- JALR
              ctrl_nxt(ctrl_alu_opa_mux_c) <= '0'; -- use RS1 as ALU.OPA (branch target address base)
            else -- JAL / branch
              ctrl_nxt(ctrl_alu_opa_mux_c) <= '1'; -- use PC as ALU.OPA (branch target address base)
            end if;
            ctrl_nxt(ctrl_alu_opb_mux_c) <= '1'; -- use IMM as ALU.OPB (branch target address offset)
            --
            execute_engine.state_nxt <= BRANCH;

          when opcode_fence_c => -- fence operations
          -- ------------------------------------------------------------
            execute_engine.state_nxt <= FENCE_OP;

          when opcode_syscsr_c => -- system/csr access
          -- ------------------------------------------------------------
            csr.re_nxt <= csr_acc_valid; -- always read CSR if valid access, only relevant for CSR-instructions
            if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_env_c) then -- system/environment
              execute_engine.state_nxt <= SYS_ENV;
            else -- CSR access
              execute_engine.state_nxt <= CSR_ACCESS;
            end if;

          when others => -- undefined
          -- ------------------------------------------------------------
            execute_engine.state_nxt <= SYS_WAIT;

        end case;


      when SYS_ENV => -- system environment operation - execution
      -- ------------------------------------------------------------
        execute_engine.pc_mux_sel <= "11"; -- csr.mepc (only relevant for MRET)
        case execute_engine.i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) is
          when funct12_ecall_c => -- ECALL
            trap_ctrl.env_call <= '1';
          when funct12_ebreak_c => -- EBREAK
            trap_ctrl.break_point <= '1';
          when funct12_mret_c => -- MRET
            trap_ctrl.env_end    <= '1';
            execute_engine.pc_we <= '1'; -- update PC from MEPC
            fetch_engine.reset   <= '1';
            execute_engine.if_rst_nxt <= '1'; -- this is a non-linear PC modification
          when funct12_wfi_c => -- WFI
            execute_engine.sleep_nxt <= '1'; -- good night
          when others => -- undefined
            NULL;
        end case;
        execute_engine.state_nxt <= SYS_WAIT;


      when CSR_ACCESS => -- read & write status and control register (CSR)
      -- ------------------------------------------------------------
        -- CSR write access --
        case execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          when funct3_csrrw_c | funct3_csrrwi_c => -- CSRRW(I)
            csr.we_nxt <= csr_acc_valid; -- always write CSR if valid access
          when funct3_csrrs_c | funct3_csrrsi_c | funct3_csrrc_c | funct3_csrrci_c => -- CSRRS(I) / CSRRC(I)
            csr.we_nxt <= (not rs1_is_r0_v) and csr_acc_valid; -- write CSR if rs1/imm is not zero and if valid access
          when others => -- invalid
            csr.we_nxt <= '0';
        end case;
        -- register file write back --
        ctrl_nxt(ctrl_rf_in_mux_msb_c downto ctrl_rf_in_mux_lsb_c) <= "11"; -- RF input = CSR output
        ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back
        execute_engine.state_nxt  <= DISPATCH;


      when ALU_WAIT => -- wait for multi-cycle ALU operation (shifter or CP) to finish
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_rf_in_mux_msb_c) <= '0'; -- RF input = ALU result
        ctrl_nxt(ctrl_rf_wb_en_c)      <= '1'; -- valid RF write-back (permanent write-back)
        -- cp access or alu shift? --
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
        ctrl_nxt(ctrl_rf_in_mux_msb_c)                       <= '0'; -- RF input = ALU result
        ctrl_nxt(ctrl_rf_wb_en_c)                            <= execute_engine.i_reg(instr_opcode_lsb_c+2); -- valid RF write-back? (is jump-and-link?)
        -- destination address --
        execute_engine.pc_mux_sel <= "01"; -- alu.add = branch/jump destination
        if (execute_engine.i_reg(instr_opcode_lsb_c+2) = '1') or (execute_engine.branch_taken = '1') then -- JAL/JALR or taken branch
          execute_engine.pc_we      <= '1'; -- update PC
          fetch_engine.reset        <= '1'; -- trigger new instruction fetch from modified PC
          execute_engine.if_rst_nxt <= '1'; -- this is a non-linear PC modification
          execute_engine.state_nxt  <= SYS_WAIT;
        else
          execute_engine.state_nxt <= DISPATCH;
        end if;


      when FENCE_OP => -- fence operations - execution
      -- ------------------------------------------------------------
        execute_engine.state_nxt  <= SYS_WAIT;
        execute_engine.pc_mux_sel <= "00"; -- linear next PC = "refetch" next instruction (only relevant for fence.i)
        -- FENCE.I --
        if (execute_engine.i_reg(instr_funct3_lsb_c) = funct3_fencei_c(0)) and (CPU_EXTENSION_RISCV_Zifencei = true) then
          execute_engine.pc_we        <= '1';
          execute_engine.if_rst_nxt   <= '1'; -- this is a non-linear PC modification
          fetch_engine.reset          <= '1';
          ctrl_nxt(ctrl_bus_fencei_c) <= '1';
        end if;
        -- FENCE --
        if (execute_engine.i_reg(instr_funct3_lsb_c) = funct3_fence_c(0)) then
          ctrl_nxt(ctrl_bus_fence_c) <= '1';
        end if;


      when LOADSTORE_0 => -- trigger memory request
      -- ------------------------------------------------------------
        if (execute_engine.i_reg(instr_opcode_msb_c-1) = '0') or (is_atomic_lr_v = '1') then -- normal load or atomic load-reservate
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
        if (CPU_EXTENSION_RISCV_A = true) then -- only relevant for atomic operations
          ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_atomic_c; -- SC: result comes from "atomic co-processor"
          ctrl_nxt(ctrl_alu_func1_c downto ctrl_alu_func0_c) <= alu_func_cmd_copro_c;
        end if;
        --
        ctrl_nxt(ctrl_rf_in_mux_lsb_c) <= '0'; -- RF input = ALU.res or MEM
        if (is_atomic_sc_v = '1') then
          ctrl_nxt(ctrl_rf_in_mux_msb_c) <= '0'; -- RF input = ALU.res
        else
          ctrl_nxt(ctrl_rf_in_mux_msb_c) <= '1'; -- RF input = memory input (only relevant for LOAD)
        end if;
        --
        ctrl_nxt(ctrl_bus_mi_we_c) <= '1'; -- keep writing input data to MDI (only relevant for load operations)
        -- wait for memory response --
        if ((ma_load_i or be_load_i or ma_store_i or be_store_i) = '1') then -- abort if exception
          atomic_ctrl.env_abort     <= '1'; -- LOCKED (atomic) memory access environment failed (forces SC result to be non-zero => failure)
          ctrl_nxt(ctrl_rf_wb_en_c) <= is_atomic_sc_v; -- SC failes: allow write back of non-zero result
          execute_engine.state_nxt  <= DISPATCH;
        elsif (bus_d_wait_i = '0') then -- wait for bus to finish transaction
          if (execute_engine.i_reg(instr_opcode_msb_c-1) = '0') or (is_atomic_lr_v = '1') or (is_atomic_sc_v = '1') then -- load / load-reservate / store conditional
            ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back
          end if;
          atomic_ctrl.env_end      <= '1'; -- normal end of LOCKED (atomic) memory access environment
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


  -- Illegal CSR Access Check ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  invalid_csr_access_check: process(execute_engine.i_reg, csr)
    variable csr_wacc_v : std_ulogic; -- to check access to read-only CSRs
--  variable csr_racc_v : std_ulogic; -- to check access to write-only CSRs
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

    -- check CSR access --
    case execute_engine.i_reg(instr_csr_id_msb_c downto instr_csr_id_lsb_c) is
      when csr_mstatus_c   => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_mstatush_c  => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_misa_c      => csr_acc_valid <= csr.priv_m_mode;-- and (not csr_wacc_v); -- M-mode only, MISA is read-only in the NEORV32 but we don't cause an exception here for compatibility
      when csr_mie_c       => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_mtvec_c     => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_mscratch_c  => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_mepc_c      => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_mcause_c    => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_mtval_c     => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_mip_c       => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      --
      when csr_pmpcfg0_c   => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(pmp_num_regions_c >= 1)) and csr.priv_m_mode; -- M-mode only
      when csr_pmpcfg1_c   => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(pmp_num_regions_c >= 5)) and csr.priv_m_mode; -- M-mode only
      --
      when csr_pmpaddr0_c  => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(pmp_num_regions_c >= 1)) and csr.priv_m_mode; -- M-mode only
      when csr_pmpaddr1_c  => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(pmp_num_regions_c >= 2)) and csr.priv_m_mode; -- M-mode only
      when csr_pmpaddr2_c  => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(pmp_num_regions_c >= 3)) and csr.priv_m_mode; -- M-mode only
      when csr_pmpaddr3_c  => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(pmp_num_regions_c >= 4)) and csr.priv_m_mode; -- M-mode only
      when csr_pmpaddr4_c  => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(pmp_num_regions_c >= 5)) and csr.priv_m_mode; -- M-mode only
      when csr_pmpaddr5_c  => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(pmp_num_regions_c >= 6)) and csr.priv_m_mode; -- M-mode only
      when csr_pmpaddr6_c  => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(pmp_num_regions_c >= 7)) and csr.priv_m_mode; -- M-mode only
      when csr_pmpaddr7_c  => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(pmp_num_regions_c >= 8)) and csr.priv_m_mode; -- M-mode only
      --
      when csr_mcycle_c    => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_minstret_c  => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      --
      when csr_mcycleh_c   => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      when csr_minstreth_c => csr_acc_valid <= csr.priv_m_mode; -- M-mode only
      --
      when csr_cycle_c     => csr_acc_valid <= (not csr_wacc_v); -- all modes, read-only
      when csr_time_c      => csr_acc_valid <= (not csr_wacc_v); -- all modes, read-only
      when csr_instret_c   => csr_acc_valid <= (not csr_wacc_v); -- all modes, read-only
      --
      when csr_cycleh_c    => csr_acc_valid <= (not csr_wacc_v); -- all modes, read-only
      when csr_timeh_c     => csr_acc_valid <= (not csr_wacc_v); -- all modes, read-only
      when csr_instreth_c  => csr_acc_valid <= (not csr_wacc_v); -- all modes, read-only
      --
      when csr_mvendorid_c => csr_acc_valid <= csr.priv_m_mode and (not csr_wacc_v); -- M-mode only, read-only
      when csr_marchid_c   => csr_acc_valid <= csr.priv_m_mode and (not csr_wacc_v); -- M-mode only, read-only
      when csr_mimpid_c    => csr_acc_valid <= csr.priv_m_mode and (not csr_wacc_v); -- M-mode only, read-only
      when csr_mhartid_c   => csr_acc_valid <= csr.priv_m_mode and (not csr_wacc_v); -- M-mode only, read-only
      --
      when csr_mzext_c     => csr_acc_valid <= csr.priv_m_mode and (not csr_wacc_v); -- M-mode only, read-only
      --
      when others => csr_acc_valid <= '0'; -- undefined, invalid access
    end case;
  end process invalid_csr_access_check;


  -- Illegal Instruction Check --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  illegal_instruction_check: process(execute_engine, csr_acc_valid)
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

        -- OPCODE check sufficient: LUI, UIPC, JAL --
        when opcode_lui_c | opcode_auipc_c | opcode_jal_c =>
          illegal_instruction <= '0';
          -- illegal E-CPU register? --
          if (CPU_EXTENSION_RISCV_E = true) and (execute_engine.i_reg(instr_rd_msb_c) = '1') then
            illegal_register <= '1';
          end if;

        when opcode_alui_c => -- check ALUI funct7
          if ((execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sll_c) and
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

        when opcode_load_c => -- check LOAD funct3
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

        when opcode_store_c => -- check STORE funct3
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

        when opcode_branch_c => -- check BRANCH funct3
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

        when opcode_jalr_c => -- check JALR funct3
          if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "000") then
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
          end if;
          -- illegal E-CPU register? --
          if (CPU_EXTENSION_RISCV_E = true) and ((execute_engine.i_reg(instr_rs1_msb_c) = '1') or (execute_engine.i_reg(instr_rd_msb_c) = '1')) then
            illegal_register <= '1';
          end if;

        when opcode_alu_c => -- check ALU funct3 & funct7
          if (execute_engine.i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000001") then -- MULDIV
            if (CPU_EXTENSION_RISCV_M = false) then -- not implemented
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

        when opcode_fence_c => -- fence instructions --
          if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_fencei_c) and (CPU_EXTENSION_RISCV_Zifencei = true) then -- FENCE.I
            illegal_instruction <= '0';
          elsif (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_fence_c) then -- FENCE
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
          end if;

        when opcode_syscsr_c => -- check system instructions --
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

        when opcode_atomic_c => -- atomic instructions --
          if (CPU_EXTENSION_RISCV_A = true) and -- atomic memory operations (A extension) enabled
             ((execute_engine.i_reg(instr_funct5_msb_c downto instr_funct5_lsb_c) = funct5_a_lr_c) or -- LR
              (execute_engine.i_reg(instr_funct5_msb_c downto instr_funct5_lsb_c) = funct5_a_sc_c)) then -- SC
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
          end if;

        when others => -- undefined instruction -> illegal!
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
      trap_ctrl.cause     <= trap_reset_c;
      trap_ctrl.env_start <= '0';
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
        trap_ctrl.irq_buf(interrupt_msw_irq_c)   <= csr.mie_msie and (trap_ctrl.irq_buf(interrupt_msw_irq_c)   or msw_irq_i)   and (not trap_ctrl.irq_ack(interrupt_msw_irq_c));
        trap_ctrl.irq_buf(interrupt_mext_irq_c)  <= csr.mie_meie and (trap_ctrl.irq_buf(interrupt_mext_irq_c)  or mext_irq_i)  and (not trap_ctrl.irq_ack(interrupt_mext_irq_c));
        trap_ctrl.irq_buf(interrupt_mtime_irq_c) <= csr.mie_mtie and (trap_ctrl.irq_buf(interrupt_mtime_irq_c) or mtime_irq_i) and (not trap_ctrl.irq_ack(interrupt_mtime_irq_c));
        -- interrupt buffer: custom fast interrupts
        trap_ctrl.irq_buf(interrupt_firq_0_c)    <= csr.mie_firqe(0) and (trap_ctrl.irq_buf(interrupt_firq_0_c) or firq_i(0)) and (not trap_ctrl.irq_ack(interrupt_firq_0_c));
        trap_ctrl.irq_buf(interrupt_firq_1_c)    <= csr.mie_firqe(1) and (trap_ctrl.irq_buf(interrupt_firq_1_c) or firq_i(1)) and (not trap_ctrl.irq_ack(interrupt_firq_1_c));
        trap_ctrl.irq_buf(interrupt_firq_2_c)    <= csr.mie_firqe(2) and (trap_ctrl.irq_buf(interrupt_firq_2_c) or firq_i(2)) and (not trap_ctrl.irq_ack(interrupt_firq_2_c));
        trap_ctrl.irq_buf(interrupt_firq_3_c)    <= csr.mie_firqe(3) and (trap_ctrl.irq_buf(interrupt_firq_3_c) or firq_i(3)) and (not trap_ctrl.irq_ack(interrupt_firq_3_c));
        -- trap control --
        if (trap_ctrl.env_start = '0') then -- no started trap handler
          if (trap_ctrl.exc_fire = '1') or ((trap_ctrl.irq_fire = '1') and -- exception/IRQ detected!
             ((execute_engine.state = EXECUTE) or (execute_engine.state = TRAP))) then -- sample IRQs in EXECUTE or TRAP state only to continue execution even if permanent IRQ
            trap_ctrl.cause     <= trap_ctrl.cause_nxt;   -- capture source ID for program (for mcause csr)
            trap_ctrl.exc_ack   <= '1';                   -- clear execption
            trap_ctrl.irq_ack   <= trap_ctrl.irq_ack_nxt; -- capture and clear with interrupt ACK mask
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


  -- Trap Priority Detector -----------------------------------------------------------------
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

    -- interrupt: 1.7 machine timer interrupt --
    elsif (trap_ctrl.irq_buf(interrupt_mtime_irq_c) = '1') then
      trap_ctrl.cause_nxt <= trap_mti_c;
      trap_ctrl.irq_ack_nxt(interrupt_mtime_irq_c) <= '1';

    -- interrupt: 1.3 machine SW interrupt --
    elsif (trap_ctrl.irq_buf(interrupt_msw_irq_c) = '1') then
      trap_ctrl.cause_nxt <= trap_msi_c;
      trap_ctrl.irq_ack_nxt(interrupt_msw_irq_c) <= '1';


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


    -- the following traps are caused by *synchronous* exceptions (= classic exceptions)
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

    -- undefined / not implemented --
    else
      trap_ctrl.cause_nxt   <= (others => '0');
      trap_ctrl.irq_ack_nxt <= (others => '0');
    end if;
  end process trap_priority;


  -- Atomic Operation Controller ------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  atomics_controller: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      atomic_ctrl.lock       <= '0';
      atomic_ctrl.env_end_ff <= '0';
    elsif rising_edge(clk_i) then
      if (CPU_EXTENSION_RISCV_A = true) then
        if (atomic_ctrl.env_end_ff = '1') or -- normal termination
           (atomic_ctrl.env_abort = '1') or  -- fast temrination (error)
           (trap_ctrl.env_start = '1') then -- triggered trap -> failure
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
  end process atomics_controller;
  

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


  -- Control and Status Registers Write Access ----------------------------------------------
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
      csr.mcause       <= (others => '0');
      csr.mtval        <= (others => '0');
      csr.pmpcfg       <= (others => (others => '0'));
      csr.pmpaddr      <= (others => (others => '1'));
      --
      csr.mcycle       <= (others => '0');
      csr.minstret     <= (others => '0');
      csr.mcycleh      <= (others => '0');
      csr.minstreth    <= (others => '0');
      mcycle_msb       <= '0';
      minstret_msb     <= '0';
    elsif rising_edge(clk_i) then
      -- write access? --
      csr.we <= csr.we_nxt;
      if (CPU_EXTENSION_RISCV_Zicsr = true) then

        -- --------------------------------------------------------------------------------
        -- CSR access by application software
        -- --------------------------------------------------------------------------------
        if (csr.we = '1') then -- manual update
          case execute_engine.i_reg(instr_csr_id_msb_c downto instr_csr_id_lsb_c) is
          
            -- machine trap setup --
            -- --------------------------------------------------------------------
            when csr_mstatus_c => -- R/W: mstatus - machine status register
              csr.mstatus_mie  <= csr.wdata(03);
              csr.mstatus_mpie <= csr.wdata(07);
              if (CPU_EXTENSION_RISCV_U = true) then -- user mode implemented
                csr.mstatus_mpp(0) <= csr.wdata(11) or csr.wdata(12);
                csr.mstatus_mpp(1) <= csr.wdata(11) or csr.wdata(12);
              else -- only machine mode is available
                csr.mstatus_mpp <= priv_mode_m_c;
              end if;
            when csr_mie_c => -- R/W: mie - machine interrupt-enable register
              csr.mie_msie <= csr.wdata(03); -- machine SW IRQ enable
              csr.mie_mtie <= csr.wdata(07); -- machine TIMER IRQ enable
              csr.mie_meie <= csr.wdata(11); -- machine EXT IRQ enable
              --
              csr.mie_firqe(0) <= csr.wdata(16); -- fast interrupt channel 0
              csr.mie_firqe(1) <= csr.wdata(17); -- fast interrupt channel 1
              csr.mie_firqe(2) <= csr.wdata(18); -- fast interrupt channel 2
              csr.mie_firqe(3) <= csr.wdata(19); -- fast interrupt channel 3
            when csr_mtvec_c => -- R/W: mtvec - machine trap-handler base address (for ALL exceptions)
              csr.mtvec <= csr.wdata(data_width_c-1 downto 2) & "00"; -- mtvec.MODE=0

            -- machine trap handling --
            -- --------------------------------------------------------------------
            when csr_mscratch_c =>  -- R/W: mscratch - machine scratch register
              csr.mscratch <= csr.wdata;
            when csr_mepc_c => -- R/W: mepc - machine exception program counter
              csr.mepc <= csr.wdata(data_width_c-1 downto 1) & '0';
            when csr_mcause_c => -- R/W: mcause - machine trap cause
              csr.mcause <= (others => '0');
              csr.mcause(csr.mcause'left) <= csr.wdata(31); -- 1: interrupt, 0: exception
              csr.mcause(4 downto 0)      <= csr.wdata(4 downto 0); -- identifier
            when csr_mtval_c => -- R/W: mtval - machine bad address/instruction
              csr.mtval <= csr.wdata;

            -- physical memory protection - configuration --
            -- --------------------------------------------------------------------
            when csr_pmpcfg0_c => -- R/W: pmpcfg0 - PMP configuration register 0
              if (PMP_USE = true) and (pmp_num_regions_c >= 1) then
                for j in 0 to 3 loop -- bytes in pmpcfg CSR
                  if ((j+1) <= pmp_num_regions_c) then
                    if (csr.pmpcfg(0+j)(7) = '0') then -- unlocked pmpcfg access
                      csr.pmpcfg(0+j)(0) <= csr.wdata(j*8+0); -- R (rights.read)
                      csr.pmpcfg(0+j)(1) <= csr.wdata(j*8+1); -- W (rights.write)
                      csr.pmpcfg(0+j)(2) <= csr.wdata(j*8+2); -- X (rights.execute)
                      csr.pmpcfg(0+j)(3) <= csr.wdata(j*8+3) and csr.wdata(j*8+4); -- A_L
                      csr.pmpcfg(0+j)(4) <= csr.wdata(j*8+3) and csr.wdata(j*8+4); -- A_H - NAPOT/OFF only
                      csr.pmpcfg(0+j)(5) <= '0'; -- reserved
                      csr.pmpcfg(0+j)(6) <= '0'; -- reserved
                      csr.pmpcfg(0+j)(7) <= csr.wdata(j*8+7); -- L (locked / rights also enforced in m-mode)
                    end if;
                  end if;
                end loop; -- j (bytes in CSR)
              end if;
            when csr_pmpcfg1_c => -- R/W: pmpcfg1 - PMP configuration register 1
              if (PMP_USE = true) and (pmp_num_regions_c >= 5) then
                for j in 0 to 3 loop -- bytes in pmpcfg CSR
                  if ((j+1+4) <= pmp_num_regions_c) then
                    if (csr.pmpcfg(4+j)(7) = '0') then -- unlocked pmpcfg access
                      csr.pmpcfg(4+j)(0) <= csr.wdata(j*8+0); -- R (rights.read)
                      csr.pmpcfg(4+j)(1) <= csr.wdata(j*8+1); -- W (rights.write)
                      csr.pmpcfg(4+j)(2) <= csr.wdata(j*8+2); -- X (rights.execute)
                      csr.pmpcfg(4+j)(3) <= csr.wdata(j*8+3) and csr.wdata(j*8+4); -- A_L
                      csr.pmpcfg(4+j)(4) <= csr.wdata(j*8+3) and csr.wdata(j*8+4); -- A_H - NAPOT/OFF only
                      csr.pmpcfg(4+j)(5) <= '0'; -- reserved
                      csr.pmpcfg(4+j)(6) <= '0'; -- reserved
                      csr.pmpcfg(4+j)(7) <= csr.wdata(j*8+7); -- L (locked / rights also enforced in m-mode)
                    end if;
                  end if;
                end loop; -- j (bytes in CSR)
              end if;

            -- physical memory protection - addresses --
            -- --------------------------------------------------------------------
            when csr_pmpaddr0_c | csr_pmpaddr1_c | csr_pmpaddr2_c | csr_pmpaddr3_c |
                 csr_pmpaddr4_c | csr_pmpaddr5_c | csr_pmpaddr6_c | csr_pmpaddr7_c => -- R/W: pmpaddr0..7 - PMP address register 0..7
              if (PMP_USE = true) then
                for i in 0 to pmp_num_regions_c-1 loop
                  if (execute_engine.i_reg(23 downto 20) = std_ulogic_vector(to_unsigned(i, 4))) and (csr.pmpcfg(i)(7) = '0') then -- unlocked pmpaddr access
                    csr.pmpaddr(i) <= csr.wdata;
                    csr.pmpaddr(i)(index_size_f(pmp_min_granularity_c)-4 downto 0) <= (others => '1');
                  end if;
                end loop; -- i (CSRs)
              end if;

            -- undefined --
            -- --------------------------------------------------------------------
            when others =>
              NULL;

          end case;

        -- --------------------------------------------------------------------------------
        -- CSR access by hardware
        -- --------------------------------------------------------------------------------
        else

          -- mepc & mtval: machine exception PC & machine trap value register --
          -- --------------------------------------------------------------------
          if (trap_ctrl.env_start_ack = '1') then -- trap handler starting?
            -- trap ID code --
            csr.mcause <= (others => '0');
            csr.mcause(csr.mcause'left) <= trap_ctrl.cause(trap_ctrl.cause'left); -- 1: interrupt, 0: exception
            csr.mcause(4 downto 0)      <= trap_ctrl.cause(4 downto 0); -- identifier
            if (trap_ctrl.cause(trap_ctrl.cause'left) = '1') then -- for INTERRUPTS
              csr.mepc  <= execute_engine.pc(data_width_c-1 downto 1) & '0'; -- this is the CURRENT pc = interrupted instruction
              csr.mtval <= (others => '0'); -- mtval is zero for interrupts
            else -- for EXCEPTIONS (according to their priority)
              csr.mepc <= execute_engine.last_pc(data_width_c-1 downto 1) & '0'; -- this is the LAST pc = last executed instruction
              if (trap_ctrl.cause(4 downto 0) = trap_iba_c(4 downto 0)) or -- instruction access error OR
                 (trap_ctrl.cause(4 downto 0) = trap_ima_c(4 downto 0)) or -- misaligned instruction address OR
                 (trap_ctrl.cause(4 downto 0) = trap_brk_c(4 downto 0)) or -- breakpoint OR
                 (trap_ctrl.cause(4 downto 0) = trap_menv_c(4 downto 0)) then -- environment call
                csr.mtval <= execute_engine.pc(data_width_c-1 downto 1) & '0'; -- address of faulting instruction
              elsif (trap_ctrl.cause(4 downto 0) = trap_iil_c(4 downto 0)) then -- illegal instruction
                csr.mtval <= execute_engine.i_reg_last; -- faulting instruction itself
              else -- load/store misalign/access errors
                csr.mtval <= mar_i; -- faulting data access address
              end if;
            end if;
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

        end if; -- hardware csr access

      -- --------------------------------------------------------------------------------
      -- Counter CSRs
      -- --------------------------------------------------------------------------------

        -- [m]cycle --
        if (csr.we = '1') and (execute_engine.i_reg(instr_csr_id_msb_c downto instr_csr_id_lsb_c) = csr_mcycle_c) then -- write access
          csr.mcycle <= '0' & csr.wdata;
          mcycle_msb <= '0';
        elsif (execute_engine.sleep = '0') then -- automatic update (if CPU is not in sleep mode)
          csr.mcycle <= std_ulogic_vector(unsigned(csr.mcycle) + 1);
          mcycle_msb <= csr.mcycle(csr.mcycle'left);
        end if;

        -- [m]cycleh --
        if (csr.we = '1') and (execute_engine.i_reg(instr_csr_id_msb_c downto instr_csr_id_lsb_c) = csr_mcycleh_c) then -- write access
          csr.mcycleh <= csr.wdata(csr.mcycleh'left downto 0);
        elsif ((mcycle_msb xor csr.mcycle(csr.mcycle'left)) = '1') then -- automatic update
          csr.mcycleh <= std_ulogic_vector(unsigned(csr.mcycleh) + 1);
        end if;

        -- [m]instret --
        if (csr.we = '1') and (execute_engine.i_reg(instr_csr_id_msb_c downto instr_csr_id_lsb_c) = csr_minstret_c) then -- write access
          csr.minstret <= '0' & csr.wdata;
          minstret_msb <= '0';
        elsif (execute_engine.state = EXECUTE) then -- automatic update (if CPU commits an instruction)
          csr.minstret <= std_ulogic_vector(unsigned(csr.minstret) + 1);
          minstret_msb <= csr.minstret(csr.minstret'left);
        end if;

        -- [m]instreth --
        if (csr.we = '1') and (execute_engine.i_reg(instr_csr_id_msb_c downto instr_csr_id_lsb_c) = csr_minstreth_c) then -- write access
          csr.minstreth <= csr.wdata(csr.minstreth'left downto 0);
        elsif ((minstret_msb xor csr.minstret(csr.minstret'left)) = '1') then -- automatic update
          csr.minstreth <= std_ulogic_vector(unsigned(csr.minstreth) + 1);
        end if;

      end if;
    end if;
  end process csr_write_access;

  -- decode privilege mode --
  csr.priv_m_mode <= '1' when (csr.privilege = priv_mode_m_c)  or (CPU_EXTENSION_RISCV_U = false) else '0';
  csr.priv_u_mode <= '1' when (csr.privilege = priv_mode_u_c) and (CPU_EXTENSION_RISCV_U = true)  else '0';

  -- PMP configuration output to bus unit --
  pmp_output: process(csr)
  begin
    pmp_addr_o <= (others => (others => '0'));
    pmp_ctrl_o <= (others => (others => '0'));
    if (PMP_USE = true) then
      for i in 0 to pmp_num_regions_c-1 loop
        pmp_addr_o(i) <= csr.pmpaddr(i) & "11";
        pmp_addr_o(i)(index_size_f(pmp_min_granularity_c)-4 downto 0) <= (others => '1');
        pmp_ctrl_o(i) <= csr.pmpcfg(i);
      end loop; -- i
    end if;
  end process pmp_output;


  -- Control and Status Registers Read Access -----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      csr.re    <= csr.re_nxt; -- read access?
      csr.rdata <= (others => '0'); -- default output
      if (CPU_EXTENSION_RISCV_Zicsr = true) and (csr.re = '1') then
        case execute_engine.i_reg(instr_csr_id_msb_c downto instr_csr_id_lsb_c) is

          -- machine trap setup --
          when csr_mstatus_c => -- R/W: mstatus - machine status register
            csr.rdata(03) <= csr.mstatus_mie;  -- MIE
            csr.rdata(07) <= csr.mstatus_mpie; -- MPIE
            csr.rdata(11) <= csr.mstatus_mpp(0); -- MPP: machine previous privilege mode low
            csr.rdata(12) <= csr.mstatus_mpp(1); -- MPP: machine previous privilege mode high
          when csr_mstatush_c => -- R/-: mstatush - machine status register - high part
            csr.rdata(05) <= '1'; -- MBE: CPU/Processor is BIG-ENDIAN
          when csr_misa_c => -- R/-: misa - ISA and extensions
            csr.rdata(00) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_A);     -- A CPU extension
            csr.rdata(02) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_C);     -- C CPU extension
            csr.rdata(04) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_E);     -- E CPU extension
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
            --
            csr.rdata(16) <= csr.mie_firqe(0); -- fast interrupt channel 0
            csr.rdata(17) <= csr.mie_firqe(1); -- fast interrupt channel 1
            csr.rdata(18) <= csr.mie_firqe(2); -- fast interrupt channel 2
            csr.rdata(19) <= csr.mie_firqe(3); -- fast interrupt channel 3
          when csr_mtvec_c => -- R/W: mtvec - machine trap-handler base address (for ALL exceptions)
            csr.rdata <= csr.mtvec(data_width_c-1 downto 2) & "00"; -- mtvec.MODE=0

          -- machine trap handling --
          when csr_mscratch_c => -- R/W: mscratch - machine scratch register
            csr.rdata <= csr.mscratch;
          when csr_mepc_c => -- R/W: mepc - machine exception program counter
            csr.rdata <= csr.mepc(data_width_c-1 downto 1) & '0';
          when csr_mcause_c => -- R/W: mcause - machine trap cause
            csr.rdata <= csr.mcause;
          when csr_mtval_c => -- R/W: mtval - machine bad address or instruction
            csr.rdata <= csr.mtval;
          when csr_mip_c => -- R/W: mip - machine interrupt pending
            csr.rdata(03) <= trap_ctrl.irq_buf(interrupt_msw_irq_c);
            csr.rdata(07) <= trap_ctrl.irq_buf(interrupt_mtime_irq_c);
            csr.rdata(11) <= trap_ctrl.irq_buf(interrupt_mext_irq_c);
            --
            csr.rdata(16) <= trap_ctrl.irq_buf(interrupt_firq_0_c);
            csr.rdata(17) <= trap_ctrl.irq_buf(interrupt_firq_1_c);
            csr.rdata(18) <= trap_ctrl.irq_buf(interrupt_firq_2_c);
            csr.rdata(19) <= trap_ctrl.irq_buf(interrupt_firq_3_c);

          -- physical memory protection - configuration --
          when csr_pmpcfg0_c => -- R/W: pmpcfg0 - physical memory protection configuration register 0
            if (PMP_USE = true) then
              if (pmp_num_regions_c >= 1) then
                csr.rdata(07 downto 00) <= csr.pmpcfg(0);
              end if;
              if (pmp_num_regions_c >= 2) then
                csr.rdata(15 downto 08) <= csr.pmpcfg(1);
              end if;
              if (pmp_num_regions_c >= 3) then
                csr.rdata(23 downto 16) <= csr.pmpcfg(2);
              end if;
              if (pmp_num_regions_c >= 4) then
                csr.rdata(31 downto 24) <= csr.pmpcfg(3);
              end if;
            end if;
          when csr_pmpcfg1_c => -- R/W: pmpcfg1 - physical memory protection configuration register 1
            if (PMP_USE = true) then
              if (pmp_num_regions_c >= 5) then
                csr.rdata(07 downto 00) <= csr.pmpcfg(4);
              end if;
              if (pmp_num_regions_c >= 6) then
                csr.rdata(15 downto 08) <= csr.pmpcfg(5);
              end if;
              if (pmp_num_regions_c >= 7) then
                csr.rdata(23 downto 16) <= csr.pmpcfg(6);
              end if;
              if (pmp_num_regions_c >= 8) then
                csr.rdata(31 downto 24) <= csr.pmpcfg(7);
              end if;
            end if;

          -- physical memory protection - addresses --
          when csr_pmpaddr0_c => -- R/W: pmpaddr0 - physical memory protection address register 0
            if (PMP_USE = true) and (pmp_num_regions_c >= 1) then
              csr.rdata <= csr.pmpaddr(0);
              if (csr.pmpcfg(0)(4 downto 3) = "00") then -- mode = off
                csr.rdata(index_size_f(pmp_min_granularity_c)-3 downto 0) <= (others => '0'); -- required for granularity check by SW
              end if;
            end if;
          when csr_pmpaddr1_c => -- R/W: pmpaddr1 - physical memory protection address register 1
            if (PMP_USE = true) and (pmp_num_regions_c >= 2) then
              csr.rdata <= csr.pmpaddr(1);
              if (csr.pmpcfg(1)(4 downto 3) = "00") then -- mode = off
                csr.rdata(index_size_f(pmp_min_granularity_c)-3 downto 0) <= (others => '0'); -- required for granularity check by SW
              end if;
            end if;
          when csr_pmpaddr2_c => -- R/W: pmpaddr2 - physical memory protection address register 2
            if (PMP_USE = true) and (pmp_num_regions_c >= 3) then
              csr.rdata <= csr.pmpaddr(2);
              if (csr.pmpcfg(2)(4 downto 3) = "00") then -- mode = off
                csr.rdata(index_size_f(pmp_min_granularity_c)-3 downto 0) <= (others => '0'); -- required for granularity check by SW
              end if;
            end if;
          when csr_pmpaddr3_c => -- R/W: pmpaddr3 - physical memory protection address register 3
            if (PMP_USE = true) and (pmp_num_regions_c >= 4) then
              csr.rdata <= csr.pmpaddr(3);
              if (csr.pmpcfg(3)(4 downto 3) = "00") then -- mode = off
                csr.rdata(index_size_f(pmp_min_granularity_c)-3 downto 0) <= (others => '0'); -- required for granularity check by SW
              end if;
            end if;
          when csr_pmpaddr4_c => -- R/W: pmpaddr4 - physical memory protection address register 4
            if (PMP_USE = true) and (pmp_num_regions_c >= 5) then
              csr.rdata <= csr.pmpaddr(4);
              if (csr.pmpcfg(4)(4 downto 3) = "00") then -- mode = off
                csr.rdata(index_size_f(pmp_min_granularity_c)-3 downto 0) <= (others => '0'); -- required for granularity check by SW
              end if;
            end if;
          when csr_pmpaddr5_c => -- R/W: pmpaddr5 - physical memory protection address register 5
            if (PMP_USE = true) and (pmp_num_regions_c >= 6) then
              csr.rdata <= csr.pmpaddr(5);
              if (csr.pmpcfg(5)(4 downto 3) = "00") then -- mode = off
                csr.rdata(index_size_f(pmp_min_granularity_c)-3 downto 0) <= (others => '0'); -- required for granularity check by SW
              end if;
            end if;
          when csr_pmpaddr6_c => -- R/W: pmpaddr6 - physical memory protection address register 6
            if (PMP_USE = true) and (pmp_num_regions_c >= 7) then
              csr.rdata <= csr.pmpaddr(6);
              if (csr.pmpcfg(6)(4 downto 3) = "00") then -- mode = off
                csr.rdata(index_size_f(pmp_min_granularity_c)-3 downto 0) <= (others => '0'); -- required for granularity check by SW
              end if;
            end if;
          when csr_pmpaddr7_c => -- R/W: pmpaddr7 - physical memory protection address register 7
            if (PMP_USE = true) and (pmp_num_regions_c >= 8) then
              csr.rdata <= csr.pmpaddr(7);
              if (csr.pmpcfg(7)(4 downto 3) = "00") then -- mode = off
                csr.rdata(index_size_f(pmp_min_granularity_c)-3 downto 0) <= (others => '0'); -- required for granularity check by SW
              end if;
            end if;

          -- counters and timers --
          when csr_cycle_c | csr_mcycle_c => -- R/(W): [m]cycle: Cycle counter LOW
            csr.rdata <= csr.mcycle(31 downto 0);
          when csr_time_c => -- R/-: time: System time LOW (from MTIME unit)
            csr.rdata <= time_i(31 downto 0);
          when csr_instret_c | csr_minstret_c => -- R/(W): [m]instret: Instructions-retired counter LOW
            csr.rdata <= csr.minstret(31 downto 0);
          when csr_cycleh_c | csr_mcycleh_c => -- R/(W): [m]cycleh: Cycle counter HIGH
            csr.rdata <= csr.mcycleh(31 downto 0);
          when csr_timeh_c => -- R/-: timeh: System time HIGH (from MTIME unit)
            csr.rdata <= time_i(63 downto 32);
          when csr_instreth_c | csr_minstreth_c => -- R/(W): [m]instreth: Instructions-retired counter HIGH
            csr.rdata <= csr.minstreth(31 downto 0);

          -- machine information registers --
          when csr_mvendorid_c => -- R/-: mvendorid - vendor ID
            csr.rdata <= (others => '0');
          when csr_marchid_c => -- R/-: marchid - arch ID
            csr.rdata(4 downto 0) <= "10011"; -- official RISC-V open-source arch ID
          when csr_mimpid_c => -- R/-: mimpid - implementation ID
            csr.rdata <= hw_version_c; -- NEORV32 hardware version
          when csr_mhartid_c => -- R/-: mhartid - hardware thread ID
            csr.rdata <= HW_THREAD_ID;

          -- custom machine read-only CSRs --
          when csr_mzext_c => -- R/-: mzext - available Z* extensions
            csr.rdata(0) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicsr);    -- RISC-V.Zicsr CPU extension
            csr.rdata(1) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zifencei); -- RISC-V.Zifencei CPU extension
            csr.rdata(2) <= bool_to_ulogic_f(PMP_USE);                      -- RISC-V physical memory protection

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
