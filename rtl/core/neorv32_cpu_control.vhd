-- #################################################################################################
-- # << NEORV32 - CPU Control >>                                                                   #
-- # ********************************************************************************************* #
-- # CPU operation is split into a fetch engine (responsible for fetching an decompressing instr-  #
-- # uctions), an execute engine (responsible for actually executing the instructions), an inter-  #
-- # rupt and exception handling controller and the RISC-V status and control registers (CSRs).    #
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
    CSR_COUNTERS_USE             : boolean := true;  -- implement RISC-V perf. counters ([m]instret[h], [m]cycle[h], time[h])?
    HW_THREAD_ID                 : std_ulogic_vector(31 downto 0):= x"00000000"; -- hardware thread id
    CPU_BOOT_ADDR                : std_ulogic_vector(31 downto 0):= x"00000000"; -- cpu boot address
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_C        : boolean := false; -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        : boolean := false; -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        : boolean := false; -- implement muld/div extension?
    CPU_EXTENSION_RISCV_U        : boolean := false; -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zicsr    : boolean := true;  -- implement CSR system?
    CPU_EXTENSION_RISCV_Zifencei : boolean := true;  -- implement instruction stream sync.?
    -- Physical memory protection (PMP) --
    PMP_USE                      : boolean := false; -- implement physical memory protection?
    PMP_NUM_REGIONS              : natural := 4; -- number of regions (1..4)
    PMP_GRANULARITY              : natural := 0  -- granularity (0=none, 1=8B, 2=16B, 3=32B, ...)
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
    alu_add_i     : in  std_ulogic_vector(data_width_c-1 downto 0); -- ALU.add result
    -- data output --
    imm_o         : out std_ulogic_vector(data_width_c-1 downto 0); -- immediate
    fetch_pc_o    : out std_ulogic_vector(data_width_c-1 downto 0); -- PC for instruction fetch
    curr_pc_o     : out std_ulogic_vector(data_width_c-1 downto 0); -- current PC (corresponding to current instruction)
    next_pc_o     : out std_ulogic_vector(data_width_c-1 downto 0); -- next PC (corresponding to current instruction)
    -- csr data interface --
    csr_wdata_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- CSR write data
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
    pmp_addr_o     : out pmp_addr_if_t; -- addresses
    pmp_ctrl_o     : out pmp_ctrl_if_t; -- configs
    priv_mode_o    : out std_ulogic_vector(1 downto 0); -- current CPU privilege level
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
  type fetch_engine_state_t is (IFETCH_RESET, IFETCH_0, IFETCH_1, IFETCH_2);
  type fetch_engine_t is record
    state           : fetch_engine_state_t;
    state_nxt       : fetch_engine_state_t;
    i_buf           : std_ulogic_vector(33 downto 0);
    i_buf_nxt       : std_ulogic_vector(33 downto 0);
    i_buf2          : std_ulogic_vector(33 downto 0);
    i_buf2_nxt      : std_ulogic_vector(33 downto 0);
    ci_input        : std_ulogic_vector(15 downto 0); -- input to compressed instr. decoder
    i_buf_state     : std_ulogic_vector(01 downto 0);
    i_buf_state_nxt : std_ulogic_vector(01 downto 0);
    pc_real         : std_ulogic_vector(data_width_c-1 downto 0);
    pc_real_add     : std_ulogic_vector(data_width_c-1 downto 0);
    pc_fetch        : std_ulogic_vector(data_width_c-1 downto 0);
    pc_fetch_add    : std_ulogic_vector(data_width_c-1 downto 0);
    reset           : std_ulogic;
    bus_err_ack     : std_ulogic;
  end record;
  signal fetch_engine : fetch_engine_t;

  -- pre-decoder --
  signal ci_instr32 : std_ulogic_vector(31 downto 0);
  signal ci_illegal : std_ulogic;

  -- instrucion prefetch buffer (IPB) --
  type ipb_t is record
    wdata  : std_ulogic_vector(35 downto 0);
    rdata  : std_ulogic_vector(35 downto 0);
    waddr  : std_ulogic_vector(31 downto 0);
    raddr  : std_ulogic_vector(31 downto 0);
    status : std_ulogic;
    free   : std_ulogic;
    avail  : std_ulogic;
    we     : std_ulogic;
    re     : std_ulogic;
    clear  : std_ulogic;
  end record;
  signal ipb : ipb_t;

  -- instruction execution engine --
  type execute_engine_state_t is (SYS_WAIT, DISPATCH, TRAP, EXECUTE, ALU_WAIT, BRANCH, LOADSTORE_0, LOADSTORE_1, LOADSTORE_2, CSR_ACCESS);
  type execute_engine_t is record
    state        : execute_engine_state_t;
    state_prev   : execute_engine_state_t;
    state_nxt    : execute_engine_state_t;
    i_reg        : std_ulogic_vector(31 downto 0);
    i_reg_nxt    : std_ulogic_vector(31 downto 0);
    is_ci        : std_ulogic; -- current instruction is de-compressed instruction
    is_ci_nxt    : std_ulogic;
    is_jump      : std_ulogic; -- current instruction is jump instruction
    is_jump_nxt  : std_ulogic;
    branch_taken : std_ulogic; -- branch condition fullfilled
    pc           : std_ulogic_vector(data_width_c-1 downto 0); -- actual PC, corresponding to current executed instruction
    pc_nxt       : std_ulogic_vector(data_width_c-1 downto 0);
    next_pc      : std_ulogic_vector(data_width_c-1 downto 0); -- next PC, corresponding to next instruction to be executed
    last_pc      : std_ulogic_vector(data_width_c-1 downto 0); -- PC of last executed instruction
    sleep        : std_ulogic; -- CPU in sleep mode
    sleep_nxt    : std_ulogic; -- CPU in sleep mode
  end record;
  signal execute_engine : execute_engine_t;

  signal next_pc_tmp : std_ulogic_vector(data_width_c-1 downto 0);

  -- trap controller --
  type trap_ctrl_t is record
    exc_buf       : std_ulogic_vector(exception_width_c-1 downto 0);
    exc_fire      : std_ulogic; -- set if there is a valid source in the exception buffer
    irq_buf       : std_ulogic_vector(interrupt_width_c-1 downto 0);
    irq_fire      : std_ulogic; -- set if there is a valid source in the interrupt buffer
    exc_ack       : std_ulogic; -- acknowledge all exceptions
    irq_ack       : std_ulogic_vector(interrupt_width_c-1 downto 0); -- acknowledge specific interrupt
    irq_ack_nxt   : std_ulogic_vector(interrupt_width_c-1 downto 0);
    cause         : std_ulogic_vector(5 downto 0); -- trap ID (for "mcause"), only for hw
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
  
  -- CPU control signals --
  signal ctrl_nxt, ctrl : std_ulogic_vector(ctrl_width_c-1 downto 0);

  -- fast bus access --
  signal bus_fast_ir : std_ulogic;

  -- RISC-V control and status registers (CSRs) --
  type pmp_ctrl_t is array (0 to PMP_NUM_REGIONS-1) of std_ulogic_vector(7 downto 0);
  type pmp_addr_t is array (0 to PMP_NUM_REGIONS-1) of std_ulogic_vector(data_width_c-1 downto 0);
  type csr_t is record
    we           : std_ulogic; -- write enable
    we_nxt       : std_ulogic;
    re           : std_ulogic; -- read enable
    re_nxt       : std_ulogic;
    mstatus_mie  : std_ulogic; -- mstatus.MIE: global IRQ enable (R/W)
    mstatus_mpie : std_ulogic; -- mstatus.MPIE: previous global IRQ enable (R/-)
    mie_msie     : std_ulogic; -- mie.MSIE: machine software interrupt enable (R/W)
    mie_meie     : std_ulogic; -- mie.MEIE: machine external interrupt enable (R/W)
    mie_mtie     : std_ulogic; -- mie.MEIE: machine timer interrupt enable (R/W
    mie_firqe    : std_ulogic_vector(3 downto 0); -- mie.firq*e: fast interrupt enabled (R/W)
    mpp          : std_ulogic_vector(1 downto 0); -- machine previous privilege mode
    privilege    : std_ulogic_vector(1 downto 0); -- hart's current previous privilege mode
    mepc         : std_ulogic_vector(data_width_c-1 downto 0); -- mepc: machine exception pc (R/W)
    mcause       : std_ulogic_vector(data_width_c-1 downto 0); -- mcause: machine trap cause (R/-)
    mtvec        : std_ulogic_vector(data_width_c-1 downto 0); -- mtvec: machine trap-handler base address (R/W), bit 1:0 == 00
    mtval        : std_ulogic_vector(data_width_c-1 downto 0); -- mtval: machine bad address or isntruction (R/W)
    mscratch     : std_ulogic_vector(data_width_c-1 downto 0); -- mscratch: scratch register (R/W)
    mcycle       : std_ulogic_vector(32 downto 0); -- mcycle (R/W), plus carry bit
    minstret     : std_ulogic_vector(32 downto 0); -- minstret (R/W), plus carry bit
    mcycleh      : std_ulogic_vector(19 downto 0); -- mcycleh (R/W) - REDUCED BIT-WIDTH!
    minstreth    : std_ulogic_vector(19 downto 0); -- minstreth (R/W) - REDUCED BIT-WIDTH!
    pmpcfg       : pmp_ctrl_t; -- physical memory protection - configuration registers
    pmpaddr      : pmp_addr_t; -- physical memory protection - address registers
  end record;
  signal csr : csr_t;

  signal mcycle_msb   : std_ulogic;
  signal minstret_msb : std_ulogic;
  signal systime      : std_ulogic_vector(63 downto 0);

  -- illegal instruction check --
  signal illegal_instruction : std_ulogic;
  signal illegal_register    : std_ulogic; -- only for E-extension
  signal illegal_compressed  : std_ulogic; -- only fir C-extension

  -- access (privilege) check --
  signal csr_acc_valid : std_ulogic; -- valid CSR access (implemented and valid access rights)

begin

-- ****************************************************************************************************************************
-- Instruction Fetch
-- ****************************************************************************************************************************

  -- Compressed Instructions Recoding -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_decompressor_inst_true:
  if (CPU_EXTENSION_RISCV_C = true) generate
    neorv32_cpu_decompressor_inst: neorv32_cpu_decompressor
    port map (
      -- instruction input --
      ci_instr16_i => fetch_engine.ci_input, -- compressed instruction input
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


  -- Fetch Engine FSM Sync ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- for registers that require a specific reset state --
  fetch_engine_fsm_sync_rst: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      fetch_engine.state <= IFETCH_RESET;
    elsif rising_edge(clk_i) then
      if (fetch_engine.reset = '1') then
        fetch_engine.state <= IFETCH_RESET;
      else
        fetch_engine.state <= fetch_engine.state_nxt;
      end if;
    end if;
  end process fetch_engine_fsm_sync_rst;


  -- for registers that DO NOT require a specific reset state --
  fetch_engine_fsm_sync: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (fetch_engine.state = IFETCH_RESET) then
        fetch_engine.pc_fetch  <= execute_engine.pc(data_width_c-1 downto 1) & '0'; -- initialize with "real" application PC
        fetch_engine.pc_real   <= execute_engine.pc(data_width_c-1 downto 1) & '0'; -- initialize with "real" application PC
      else
        fetch_engine.pc_real   <= std_ulogic_vector(unsigned(fetch_engine.pc_real(data_width_c-1 downto 1) & '0')  + unsigned(fetch_engine.pc_real_add(data_width_c-1 downto 1) & '0'));
        fetch_engine.pc_fetch  <= std_ulogic_vector(unsigned(fetch_engine.pc_fetch(data_width_c-1 downto 1) & '0') + unsigned(fetch_engine.pc_fetch_add(data_width_c-1 downto 1) & '0'));
      end if;
      --
      fetch_engine.i_buf       <= fetch_engine.i_buf_nxt;
      fetch_engine.i_buf2      <= fetch_engine.i_buf2_nxt;
      fetch_engine.i_buf_state <= fetch_engine.i_buf_state_nxt;
    end if;
  end process fetch_engine_fsm_sync;

  -- PC output --
  fetch_pc_o <= fetch_engine.pc_fetch(data_width_c-1 downto 1) & '0';


  -- Fetch Engine FSM Comb ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  fetch_engine_fsm_comb: process(fetch_engine, csr, ipb, instr_i, bus_i_wait_i, ci_instr32, ci_illegal, be_instr_i, ma_instr_i)
  begin
    -- arbiter defaults --
    bus_fast_ir                  <= '0';
    fetch_engine.state_nxt       <= fetch_engine.state;
    fetch_engine.pc_fetch_add    <= (others => '0');
    fetch_engine.pc_real_add     <= (others => '0');
    fetch_engine.i_buf_nxt       <= fetch_engine.i_buf;
    fetch_engine.i_buf2_nxt      <= fetch_engine.i_buf2;
    fetch_engine.i_buf_state_nxt <= fetch_engine.i_buf_state;
    fetch_engine.ci_input        <= fetch_engine.i_buf2(15 downto 00);
    fetch_engine.bus_err_ack     <= '0';

    -- instruction prefetch buffer interface --
    ipb.we    <= '0';
    ipb.clear <= '0';
    ipb.wdata <= (others => '0');
    ipb.waddr <= fetch_engine.pc_real(data_width_c-1 downto 1) & '0';

    -- state machine --
    case fetch_engine.state is

      when IFETCH_RESET => -- reset engine, prefetch buffer, get appilcation PC
      -- ------------------------------------------------------------
        fetch_engine.i_buf_state_nxt <= (others => '0');
        ipb.clear                    <= '1'; -- clear instruction prefetch buffer
        fetch_engine.state_nxt       <= IFETCH_0;

      when IFETCH_0 => -- output current PC to bus system, request 32-bit word
      -- ------------------------------------------------------------
        bus_fast_ir            <= '1'; -- fast instruction fetch request
        fetch_engine.state_nxt <= IFETCH_1;

      when IFETCH_1 => -- store data from memory to buffer(s)
      -- ------------------------------------------------------------
        if (bus_i_wait_i = '0') or (be_instr_i = '1') or (ma_instr_i = '1') then -- wait for bus response
          fetch_engine.i_buf_nxt       <= be_instr_i & ma_instr_i & instr_i(31 downto 0); -- store data word and exception info
          fetch_engine.i_buf2_nxt      <= fetch_engine.i_buf;
          fetch_engine.i_buf_state_nxt <= fetch_engine.i_buf_state(0) & '1';
          fetch_engine.bus_err_ack     <= '1'; -- acknowledge any instruction bus errors, the execute engine has to take care of them
          if (fetch_engine.i_buf_state(0) = '1') then -- buffer filled?
            fetch_engine.state_nxt <= IFETCH_2;
          else
            fetch_engine.pc_fetch_add <= std_ulogic_vector(to_unsigned(4, data_width_c));
            fetch_engine.state_nxt    <= IFETCH_0; -- get another instruction word
          end if;
        end if;

      when IFETCH_2 => -- construct instruction word and issue
      -- ------------------------------------------------------------
        if (fetch_engine.pc_fetch(1) = '0') or (CPU_EXTENSION_RISCV_C = false) then -- 32-bit aligned
          fetch_engine.ci_input <= fetch_engine.i_buf2(15 downto 00);

          if (ipb.free = '1') then -- free entry in buffer?
            ipb.we <= '1';
            if (fetch_engine.i_buf2(01 downto 00) = "11") or (CPU_EXTENSION_RISCV_C = false) then -- uncompressed
              ipb.wdata                 <= '0' & fetch_engine.i_buf2(33 downto 32) & '0' & fetch_engine.i_buf2(31 downto 0);
              fetch_engine.pc_real_add  <= std_ulogic_vector(to_unsigned(4, data_width_c));
              fetch_engine.pc_fetch_add <= std_ulogic_vector(to_unsigned(4, data_width_c));
              fetch_engine.state_nxt    <= IFETCH_0;
            else -- compressed
              ipb.wdata                 <= ci_illegal & fetch_engine.i_buf2(33 downto 32) & '1' & ci_instr32;
              fetch_engine.pc_fetch_add <= std_ulogic_vector(to_unsigned(2, data_width_c));
              fetch_engine.pc_real_add  <= std_ulogic_vector(to_unsigned(2, data_width_c));
              fetch_engine.state_nxt    <= IFETCH_2; -- try to get another 16-bit instruction word in next round
            end if;
          end if;

        else -- 16-bit aligned
          fetch_engine.ci_input <= fetch_engine.i_buf2(31 downto 16);

          if (ipb.free = '1') then -- free entry in buffer?
            ipb.we <= '1';
            if (fetch_engine.i_buf2(17 downto 16) = "11") then -- uncompressed
              ipb.wdata                 <= '0' & fetch_engine.i_buf(33 downto 32) & '0' & fetch_engine.i_buf(15 downto 00) & fetch_engine.i_buf2(31 downto 16);
              fetch_engine.pc_real_add  <= std_ulogic_vector(to_unsigned(4, data_width_c));
              fetch_engine.pc_fetch_add <= std_ulogic_vector(to_unsigned(4, data_width_c));
              fetch_engine.state_nxt    <= IFETCH_0;
            else -- compressed
              ipb.wdata                 <= ci_illegal & fetch_engine.i_buf(33 downto 32) & '1' & ci_instr32;
              fetch_engine.pc_fetch_add <= std_ulogic_vector(to_unsigned(2, data_width_c));
              fetch_engine.pc_real_add  <= std_ulogic_vector(to_unsigned(2, data_width_c));
              fetch_engine.state_nxt    <= IFETCH_0;
            end if;
          end if;
       end if;

      when others => -- undefined
      -- ------------------------------------------------------------
        fetch_engine.state_nxt <= IFETCH_RESET;

    end case;
  end process fetch_engine_fsm_comb;


-- ****************************************************************************************************************************
-- Instruction Prefetch Buffer
-- ****************************************************************************************************************************


  -- Instruction Prefetch Buffer Stage ------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  instr_prefetch_buffer: process(rstn_i, clk_i) -- once upon a time, this was a fifo with 8 entries
  begin
    if (rstn_i = '0') then
      ipb.status <= '0';
      ipb.rdata  <= (others => '0');
      ipb.raddr  <= (others => '0');
    elsif rising_edge(clk_i) then
      if (ipb.clear = '1') then
        ipb.status <= '0';
      elsif (ipb.we = '1') then
        ipb.status <= '1';
      elsif (ipb.re = '1') then
        ipb.status <= '0';
      end if;
      if (ipb.we = '1') then
        ipb.rdata <= ipb.wdata;
        ipb.raddr <= ipb.waddr;
      end if;
    end if;
  end process instr_prefetch_buffer;

  -- status --
  ipb.free  <= not ipb.status;
  ipb.avail <= ipb.status;


-- ****************************************************************************************************************************
-- Instruction Execution
-- ****************************************************************************************************************************


  -- Immediate Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  imm_gen: process(clk_i)
  begin
    if rising_edge(clk_i) then
      case execute_engine.i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c) is
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
        when opcode_syscsr_c => -- CSR-immediate
          imm_o(31 downto 05) <= (others => '0');
          imm_o(04 downto 00) <= execute_engine.i_reg(19 downto 15);
        when others => -- I-immediate
          imm_o(31 downto 11) <= (others => execute_engine.i_reg(31)); -- sign extension
          imm_o(10 downto 05) <= execute_engine.i_reg(30 downto 25);
          imm_o(04 downto 01) <= execute_engine.i_reg(24 downto 21);
          imm_o(00)           <= execute_engine.i_reg(20);
      end case;
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
      execute_engine.pc      <= CPU_BOOT_ADDR(data_width_c-1 downto 1) & '0';
      execute_engine.last_pc <= CPU_BOOT_ADDR(data_width_c-1 downto 1) & '0';
      execute_engine.state   <= SYS_WAIT;
      execute_engine.sleep   <= '0';
    elsif rising_edge(clk_i) then
      execute_engine.pc <= execute_engine.pc_nxt(data_width_c-1 downto 1) & '0';
      if (execute_engine.state = EXECUTE) then
        execute_engine.last_pc <= execute_engine.pc(data_width_c-1 downto 1) & '0';
      end if;
      execute_engine.state <= execute_engine.state_nxt;
      execute_engine.sleep <= execute_engine.sleep_nxt;
    end if;
  end process execute_engine_fsm_sync_rst;


  -- for registers that do NOT require a specific reset state --
  execute_engine_fsm_sync: process(clk_i)
  begin
    if rising_edge(clk_i) then
      execute_engine.state_prev <= execute_engine.state;
      execute_engine.i_reg      <= execute_engine.i_reg_nxt;
      execute_engine.is_ci      <= execute_engine.is_ci_nxt;
      execute_engine.is_jump    <= execute_engine.is_jump_nxt;
      --
      ctrl <= ctrl_nxt;
    end if;
  end process execute_engine_fsm_sync;

  -- PC output --
  curr_pc_o              <= execute_engine.pc(data_width_c-1 downto 1) & '0';
  next_pc_tmp            <= std_ulogic_vector(unsigned(execute_engine.pc) + 2) when (execute_engine.is_ci = '1') else std_ulogic_vector(unsigned(execute_engine.pc) + 4);
  execute_engine.next_pc <= next_pc_tmp(data_width_c-1 downto 1) & '0';
  next_pc_o              <= next_pc_tmp(data_width_c-1 downto 1) & '0';


  -- CPU Control Bus Output -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_output: process(ctrl, execute_engine, fetch_engine, trap_ctrl, csr, bus_fast_ir)
  begin
    ctrl_o <= ctrl;
    -- direct output of register addresses --
    ctrl_o(ctrl_rf_rd_adr4_c  downto ctrl_rf_rd_adr0_c)  <= execute_engine.i_reg(instr_rd_msb_c  downto instr_rd_lsb_c);
    ctrl_o(ctrl_rf_rs1_adr4_c downto ctrl_rf_rs1_adr0_c) <= execute_engine.i_reg(instr_rs1_msb_c downto instr_rs1_lsb_c);
    ctrl_o(ctrl_rf_rs2_adr4_c downto ctrl_rf_rs2_adr0_c) <= execute_engine.i_reg(instr_rs2_msb_c downto instr_rs2_lsb_c);
    -- fast bus access requests --
    ctrl_o(ctrl_bus_if_c) <= ctrl(ctrl_bus_if_c) or bus_fast_ir;
    -- bus error control --
    ctrl_o(ctrl_bus_ierr_ack_c) <= fetch_engine.bus_err_ack;
    ctrl_o(ctrl_bus_derr_ack_c) <= trap_ctrl.env_start_ack;
  end process ctrl_output;


  -- Execute Engine FSM Comb ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  execute_engine_fsm_comb: process(execute_engine, fetch_engine, ipb, trap_ctrl, csr, ctrl, csr_acc_valid,
                                   alu_add_i, alu_wait_i, bus_d_wait_i, ma_load_i, be_load_i, ma_store_i, be_store_i)
    variable alu_immediate_v : std_ulogic;
    variable alu_operation_v : std_ulogic_vector(2 downto 0);
    variable rs1_is_r0_v     : std_ulogic;
  begin
    -- arbiter defaults --
    execute_engine.state_nxt   <= execute_engine.state;
    execute_engine.i_reg_nxt   <= execute_engine.i_reg;
    execute_engine.is_jump_nxt <= '0';
    execute_engine.is_ci_nxt   <= execute_engine.is_ci;
    execute_engine.pc_nxt      <= execute_engine.pc;
    execute_engine.sleep_nxt   <= execute_engine.sleep;

    -- instruction dispatch --
    fetch_engine.reset         <= '0';
    ipb.re                     <= '0';

    -- trap environment control --
    trap_ctrl.env_start_ack    <= '0';
    trap_ctrl.env_end          <= '0';

    -- exception trigger --
    trap_ctrl.instr_be         <= '0';
    trap_ctrl.instr_ma         <= '0';
    trap_ctrl.env_call         <= '0';
    trap_ctrl.break_point      <= '0';
    illegal_compressed         <= '0';

    -- CSR access --
    csr.we_nxt                 <= '0';
    csr.re_nxt                 <= '0';

    -- control defaults --
    ctrl_nxt <= (others => '0'); -- all off at first
    if (execute_engine.i_reg(instr_opcode_lsb_c+4) = '1') then -- ALU ops
      ctrl_nxt(ctrl_alu_unsigned_c) <= execute_engine.i_reg(instr_funct3_lsb_c+0); -- unsigned ALU operation (SLTIU, SLTU)
    else -- branches
      ctrl_nxt(ctrl_alu_unsigned_c) <= execute_engine.i_reg(instr_funct3_lsb_c+1); -- unsigned branches (BLTU, BGEU)
    end if;
    ctrl_nxt(ctrl_bus_unsigned_c)  <= execute_engine.i_reg(instr_funct3_msb_c); -- unsigned LOAD (LBU, LHU)
    ctrl_nxt(ctrl_alu_shift_dir_c) <= execute_engine.i_reg(instr_funct3_msb_c); -- shift direction (left/right)
    ctrl_nxt(ctrl_alu_shift_ar_c)  <= execute_engine.i_reg(30); -- is arithmetic shift
    ctrl_nxt(ctrl_bus_size_lsb_c)  <= execute_engine.i_reg(instr_funct3_lsb_c+0); -- transfer size lsb (00=byte, 01=half-word)
    ctrl_nxt(ctrl_bus_size_msb_c)  <= execute_engine.i_reg(instr_funct3_lsb_c+1); -- transfer size msb (10=word, 11=?)
    ctrl_nxt(ctrl_cp_cmd2_c   downto ctrl_cp_cmd0_c)   <= execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c); -- CP operation
    ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_muldiv_c; -- only CP0 (MULDIV) implemented yet

    -- is immediate operation? --
    alu_immediate_v := '0';
    if (execute_engine.i_reg(instr_opcode_msb_c-1) = '0') then
      alu_immediate_v := '1';
    end if;

    -- alu operation re-coding --
    case execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) is
      when funct3_subadd_c => -- SUB / ADD(I)
        if (alu_immediate_v = '0') and (execute_engine.i_reg(instr_funct7_msb_c-1) = '1') then -- not immediate and funct7 = SUB
          alu_operation_v := alu_cmd_sub_c;
        else
          alu_operation_v := alu_cmd_add_c;
        end if;
      when funct3_sll_c  => alu_operation_v := alu_cmd_shift_c; -- SLL(I)
      when funct3_slt_c  => alu_operation_v := alu_cmd_slt_c;   -- SLT(I)
      when funct3_sltu_c => alu_operation_v := alu_cmd_slt_c;   -- SLTU(I)
      when funct3_xor_c  => alu_operation_v := alu_cmd_xor_c;   -- XOR(I)
      when funct3_sr_c   => alu_operation_v := alu_cmd_shift_c; -- SRL(I) / SRA(I)
      when funct3_or_c   => alu_operation_v := alu_cmd_or_c;    -- OR(I)
      when funct3_and_c  => alu_operation_v := alu_cmd_and_c;   -- AND(I)
      when others        => alu_operation_v := (others => '0'); -- undefined
    end case;

    -- is rs1 = r0? --
    rs1_is_r0_v := '0';
    if (execute_engine.i_reg(instr_rs1_msb_c downto instr_rs1_lsb_c) = "00000") then
      rs1_is_r0_v := '1';
    end if;

    -- state machine --
    case execute_engine.state is

      when SYS_WAIT => -- Delay cycle (used to wait for side effects to kick in)
      -- ------------------------------------------------------------
        execute_engine.state_nxt <= DISPATCH;

       when DISPATCH => -- Get new command from instruction prefetch buffer (IPB)
       -- ------------------------------------------------------------
        if (ipb.avail = '1') then -- instruction available?
          ipb.re <= '1';
          trap_ctrl.instr_ma <= ipb.rdata(33); -- misaligned instruction fetch address
          trap_ctrl.instr_be <= ipb.rdata(34); -- bus access fault druing instrucion fetch
          illegal_compressed <= ipb.rdata(35); -- invalid decompressed instruction
          execute_engine.is_ci_nxt <= ipb.rdata(32); -- flag to indicate this is a compressed instruction beeing executed
          execute_engine.i_reg_nxt <= ipb.rdata(31 downto 0);
          execute_engine.pc_nxt    <= ipb.raddr; -- the PC according to the current instruction
          -- ipb.rdata(35) is not immediately checked here!
          if (execute_engine.sleep = '1') or (trap_ctrl.env_start = '1') or ((ipb.rdata(33) or ipb.rdata(34)) = '1') then
            execute_engine.state_nxt <= TRAP;
          else
            execute_engine.state_nxt <= EXECUTE;
          end if;
        end if;

      when TRAP => -- Start trap environment (also used as cpu sleep state)
      -- ------------------------------------------------------------
        fetch_engine.reset <= '1';
        if (trap_ctrl.env_start = '1') then -- check here again if we came directly from DISPATCH
          trap_ctrl.env_start_ack  <= '1';
          execute_engine.pc_nxt    <= csr.mtvec;
          execute_engine.sleep_nxt <= '0'; -- waky waky
          execute_engine.state_nxt <= SYS_WAIT;
        end if;

      when EXECUTE => -- Decode and execute instruction
      -- ------------------------------------------------------------
        case execute_engine.i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c) is

          when opcode_alu_c | opcode_alui_c => -- ALU operation
          -- ------------------------------------------------------------
            ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '0'; -- use RS1 as ALU.OPA
            ctrl_nxt(ctrl_alu_opb_mux_lsb_c) <= alu_immediate_v; -- use IMM as ALU.OPB for immediate operations
            ctrl_nxt(ctrl_alu_opc_mux_c)     <= not alu_immediate_v;
            ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_operation_v; -- actual ALU operation
            ctrl_nxt(ctrl_rf_in_mux_msb_c downto ctrl_rf_in_mux_lsb_c) <= "00"; -- RF input = ALU result
            -- multi cycle alu operation? --
            if (alu_operation_v = alu_cmd_shift_c) or -- shift operation?
               ((CPU_EXTENSION_RISCV_M = true) and (execute_engine.i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c) = opcode_alu_c) and
                (execute_engine.i_reg(instr_funct7_lsb_c) = '1')) then -- MULDIV?
              execute_engine.state_nxt <= ALU_WAIT;
            else
              ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back
              execute_engine.state_nxt <= DISPATCH;
            end if;
            -- cp access? --
            if (CPU_EXTENSION_RISCV_M = true) and (execute_engine.i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c) = opcode_alu_c) and
               (execute_engine.i_reg(instr_funct7_lsb_c) = '1') then -- MULDIV?
              ctrl_nxt(ctrl_cp_use_c) <= '1'; -- use CP
            end if;

          when opcode_lui_c | opcode_auipc_c => -- load upper immediate (add to PC)
          -- ------------------------------------------------------------
            ctrl_nxt(ctrl_rf_clear_rs1_c) <= '1'; -- force RS1 = r0 (only relevant for LUI)
            if (execute_engine.i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c) = opcode_auipc_c) then -- AUIPC
              ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '1'; -- use PC as ALU.OPA
            else -- LUI
              ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '0'; -- use RS1 as ALU.OPA
            end if;
            ctrl_nxt(ctrl_alu_opb_mux_lsb_c) <= '1'; -- use IMM as ALU.OPB
            ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_add_c; -- actual ALU operation
            ctrl_nxt(ctrl_rf_in_mux_msb_c downto ctrl_rf_in_mux_lsb_c) <= "00"; -- RF input = ALU result
            ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back
            execute_engine.state_nxt <= DISPATCH;

          when opcode_load_c | opcode_store_c => -- load/store
          -- ------------------------------------------------------------
            ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '0'; -- use RS1 as ALU.OPA
            ctrl_nxt(ctrl_alu_opb_mux_lsb_c) <= '1'; -- use IMM as ALU.OPB
            ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_add_c; -- actual ALU operation
            ctrl_nxt(ctrl_bus_mar_we_c) <= '1'; -- write to MAR
            ctrl_nxt(ctrl_bus_mdo_we_c) <= '1'; -- write to MDO (only relevant for stores)
            execute_engine.state_nxt    <= LOADSTORE_0;

          when opcode_branch_c => -- branch instruction
          -- ------------------------------------------------------------
            ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '1'; -- use PC as ALU.OPA
            ctrl_nxt(ctrl_alu_opb_mux_lsb_c) <= '1'; -- use IMM as ALU.OPB
            ctrl_nxt(ctrl_alu_opc_mux_c)     <= '1'; -- use RS2 as ALU.OPC
            execute_engine.state_nxt         <= BRANCH;

          when opcode_jal_c | opcode_jalr_c => -- jump and link (with register)
          -- ------------------------------------------------------------
            -- compute target address --
            if (execute_engine.i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c) = opcode_jal_c) then -- JAL
              ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '1'; -- use PC as ALU.OPA
            else -- JALR
              ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '0'; -- use RS1 as ALU.OPA
            end if;
            ctrl_nxt(ctrl_alu_opb_mux_lsb_c) <= '1'; -- use IMM as ALU.OPB
            -- save return address --
            ctrl_nxt(ctrl_rf_in_mux_msb_c downto ctrl_rf_in_mux_lsb_c) <= "10"; -- RF input = next PC (save return address)
            ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back
            --
            execute_engine.is_jump_nxt <= '1'; -- this is a jump operation
            execute_engine.state_nxt   <= BRANCH;

          when opcode_fence_c => -- fence operations
          -- ------------------------------------------------------------
            execute_engine.pc_nxt <= execute_engine.next_pc; -- "refetch" next instruction (only relevant for fencei)
            if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_fencei_c) and (CPU_EXTENSION_RISCV_Zifencei = true) then -- FENCEI
              fetch_engine.reset          <= '1';
              ctrl_nxt(ctrl_bus_fencei_c) <= '1';
            end if;
            if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_fence_c) then -- FENCE
              ctrl_nxt(ctrl_bus_fence_c) <= '1';
            end if;
            execute_engine.state_nxt <= SYS_WAIT;

          when opcode_syscsr_c => -- system/csr access
          -- ------------------------------------------------------------
            csr.re_nxt <= csr_acc_valid; -- read CSR if valid access
            if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_env_c) then -- system
              case execute_engine.i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) is
                when funct12_ecall_c => -- ECALL
                  trap_ctrl.env_call <= '1';
                when funct12_ebreak_c => -- EBREAK
                  trap_ctrl.break_point <= '1';
                when funct12_mret_c => -- MRET
                  trap_ctrl.env_end     <= '1';
                  execute_engine.pc_nxt <= csr.mepc;
                  fetch_engine.reset    <= '1';
                when funct12_wfi_c => -- WFI = "CPU sleep"
                  execute_engine.sleep_nxt <= '1'; -- good night
                when others => -- undefined
                  NULL;
              end case;
              execute_engine.state_nxt <= SYS_WAIT;
            else -- CSR access
              execute_engine.state_nxt <= CSR_ACCESS;
            end if;

          when others => -- undefined
          -- ------------------------------------------------------------
            execute_engine.state_nxt <= DISPATCH;

        end case;

      when CSR_ACCESS => -- write CSR data to RF, write ALU.res to CSR
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_alu_opa_mux_msb_c) <= '0'; -- default
        ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '0'; -- default
        ctrl_nxt(ctrl_alu_opb_mux_msb_c) <= '0'; -- default
        ctrl_nxt(ctrl_alu_opb_mux_lsb_c) <= '0'; -- default
        ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_or_c; -- default ALU operation = OR
        case execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          -- register operations --
          when funct3_csrrw_c => -- CSRRW
            ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '0'; -- OPA = rs1
            ctrl_nxt(ctrl_alu_opb_mux_lsb_c) <= '0'; -- OPB = rs2
            ctrl_nxt(ctrl_rf_clear_rs2_c)    <= '1'; -- rs2 = 0
            ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_or_c; -- actual ALU operation = OR
            csr.we_nxt <= csr_acc_valid; -- always write CSR if valid access
          when funct3_csrrs_c => -- CSRRS
            ctrl_nxt(ctrl_alu_opa_mux_msb_c) <= '1'; -- OPA = csr
            ctrl_nxt(ctrl_alu_opb_mux_msb_c) <= '1'; -- OPB = rs1
            ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_or_c; -- actual ALU operation = OR
            csr.we_nxt <= (not rs1_is_r0_v) and csr_acc_valid; -- write CSR if rs1 is not zero_reg and if valid access
          when funct3_csrrc_c => -- CSRRC
            ctrl_nxt(ctrl_alu_opa_mux_msb_c) <= '1'; -- OPA = csr
            ctrl_nxt(ctrl_alu_opb_mux_msb_c) <= '1'; -- OPB = rs1
            ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_bitc_c; -- actual ALU operation = bit clear
            csr.we_nxt <= (not rs1_is_r0_v) and csr_acc_valid; -- write CSR if rs1 is not zero_reg and if valid access
          -- immediate operations --
          when funct3_csrrwi_c => -- CSRRWI
            ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '0'; -- OPA = rs1
            ctrl_nxt(ctrl_rf_clear_rs1_c)    <= '1'; -- rs1 = 0
            ctrl_nxt(ctrl_alu_opb_mux_lsb_c) <= '1'; -- OPB = immediate
            ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_or_c; -- actual ALU operation = OR
            csr.we_nxt <= csr_acc_valid; -- always write CSR if valid access
          when funct3_csrrsi_c => -- CSRRSI
            ctrl_nxt(ctrl_alu_opa_mux_msb_c) <= '1'; -- OPA = csr
            ctrl_nxt(ctrl_alu_opb_mux_lsb_c) <= '1'; -- OPB = immediate
            ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_or_c; -- actual ALU operation = OR
            csr.we_nxt <= (not rs1_is_r0_v) and csr_acc_valid; -- write CSR if UIMM5 is not zero (bits from rs1 filed) and if valid access
          when funct3_csrrci_c => -- CSRRCI
            ctrl_nxt(ctrl_alu_opa_mux_msb_c) <= '1'; -- OPA = csr
            ctrl_nxt(ctrl_alu_opb_mux_lsb_c) <= '1'; -- OPB = immediate
            ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_bitc_c; -- actual ALU operation = bit clear
            csr.we_nxt <= (not rs1_is_r0_v) and csr_acc_valid; -- write CSR if UIMM5 is not zero (bits from rs1 filed) and if valid access
          when others => -- undefined
            NULL;
        end case;
        -- RF write back --
        ctrl_nxt(ctrl_rf_in_mux_msb_c downto ctrl_rf_in_mux_lsb_c) <= "11"; -- RF input = CSR output
        ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back
        execute_engine.state_nxt  <= DISPATCH; -- FIXME should be SYS_WAIT? have another cycle to let side-effects kick in

      when ALU_WAIT => -- wait for multi-cycle ALU operation (shifter or CP) to finish
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_shift_c;
        ctrl_nxt(ctrl_rf_in_mux_msb_c downto ctrl_rf_in_mux_lsb_c) <= "00"; -- RF input = ALU result
        ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back (permanent write-back)
        -- cp access? --
        if (CPU_EXTENSION_RISCV_M = true) and (execute_engine.i_reg(instr_funct7_lsb_c) = '1') then -- MULDIV?
          ctrl_nxt(ctrl_cp_use_c) <= '1'; -- use CP
        end if;
        -- wait for result --
        if (alu_wait_i = '0') then
          execute_engine.state_nxt  <= DISPATCH;
        end if;

      when BRANCH => -- update PC for taken branches and jumps
      -- ------------------------------------------------------------
        execute_engine.pc_nxt <= alu_add_i; -- branch/jump destination
        if (execute_engine.is_jump = '1') or (execute_engine.branch_taken = '1') then
          fetch_engine.reset       <= '1'; -- trigger new instruction fetch from modified PC
          execute_engine.state_nxt <= SYS_WAIT;
        else
          execute_engine.state_nxt <= DISPATCH;
        end if;

      when LOADSTORE_0 => -- trigger memory request
      -- ------------------------------------------------------------
        if (execute_engine.i_reg(instr_opcode_msb_c-1) = '0') then -- LOAD
          ctrl_nxt(ctrl_bus_rd_c) <= '1'; -- read request
        else -- STORE
          ctrl_nxt(ctrl_bus_wr_c) <= '1'; -- write request
        end if;
        execute_engine.state_nxt <= LOADSTORE_1;

      when LOADSTORE_1 => -- memory latency
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_bus_mdi_we_c) <= '1'; -- write input data to MDI (only relevant for LOAD)
        execute_engine.state_nxt <= LOADSTORE_2;

      when LOADSTORE_2 => -- wait for bus transaction to finish
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_bus_mdi_we_c) <= '1'; -- keep writing input data to MDI (only relevant for LOAD)
        ctrl_nxt(ctrl_rf_in_mux_msb_c downto ctrl_rf_in_mux_lsb_c) <= "01"; -- RF input = memory input (only relevant for LOAD)
        if (ma_load_i = '1') or (be_load_i = '1') or (ma_store_i = '1') or (be_store_i = '1') then -- abort if exception
          execute_engine.state_nxt <= SYS_WAIT;
        elsif (bus_d_wait_i = '0') then -- wait here for bus to finish transaction
          if (execute_engine.i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c) = opcode_load_c) then -- LOAD?
            ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back
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


  -- Illegal CSR Access Check ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  invalid_csr_access_check: process(execute_engine, csr)
    variable is_m_mode_v : std_ulogic;
  begin
    -- are we in machine mode? --
    is_m_mode_v := '0';
    if (csr.privilege = m_priv_mode_c) then
      is_m_mode_v := '1';
    end if;

    -- check CSR access --
    csr_acc_valid <= '0'; -- default
    case execute_engine.i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) is
      when x"300" => csr_acc_valid <= is_m_mode_v; -- mstatus
      when x"301" => csr_acc_valid <= is_m_mode_v; -- misa
      when x"304" => csr_acc_valid <= is_m_mode_v; -- mie
      when x"305" => csr_acc_valid <= is_m_mode_v; -- mtvev
      when x"340" => csr_acc_valid <= is_m_mode_v; -- mscratch
      when x"341" => csr_acc_valid <= is_m_mode_v; -- mepc
      when x"342" => csr_acc_valid <= is_m_mode_v; -- mcause
      when x"343" => csr_acc_valid <= is_m_mode_v; -- mtval
      when x"344" => csr_acc_valid <= is_m_mode_v; -- mip
      --
      when x"3a0" => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(PMP_NUM_REGIONS >=  1)) and is_m_mode_v; -- pmpacfg0
      when x"3a1" => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(PMP_NUM_REGIONS >=  5)) and is_m_mode_v; -- pmpacfg1
      --
      when x"3b0" => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(PMP_NUM_REGIONS >=  1)) and is_m_mode_v; -- pmpaddr0
      when x"3b1" => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(PMP_NUM_REGIONS >=  2)) and is_m_mode_v; -- pmpaddr1
      when x"3b2" => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(PMP_NUM_REGIONS >=  3)) and is_m_mode_v; -- pmpaddr2
      when x"3b3" => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(PMP_NUM_REGIONS >=  4)) and is_m_mode_v; -- pmpaddr3
      when x"3b4" => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(PMP_NUM_REGIONS >=  5)) and is_m_mode_v; -- pmpaddr4
      when x"3b5" => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(PMP_NUM_REGIONS >=  6)) and is_m_mode_v; -- pmpaddr5
      when x"3b6" => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(PMP_NUM_REGIONS >=  7)) and is_m_mode_v; -- pmpaddr6
      when x"3b7" => csr_acc_valid <= bool_to_ulogic_f(PMP_USE) and bool_to_ulogic_f(boolean(PMP_NUM_REGIONS >=  8)) and is_m_mode_v; -- pmpaddr7
      --
      when x"c00" => csr_acc_valid <= bool_to_ulogic_f(CSR_COUNTERS_USE); -- cycle
      when x"c01" => csr_acc_valid <= bool_to_ulogic_f(CSR_COUNTERS_USE); -- time
      when x"c02" => csr_acc_valid <= bool_to_ulogic_f(CSR_COUNTERS_USE); -- instret
      when x"c80" => csr_acc_valid <= bool_to_ulogic_f(CSR_COUNTERS_USE); -- cycleh
      when x"c81" => csr_acc_valid <= bool_to_ulogic_f(CSR_COUNTERS_USE); -- timeh
      when x"c82" => csr_acc_valid <= bool_to_ulogic_f(CSR_COUNTERS_USE); -- instreth
      --
      when x"b00" => csr_acc_valid <= bool_to_ulogic_f(CSR_COUNTERS_USE) and is_m_mode_v; -- mcycle
      when x"b02" => csr_acc_valid <= bool_to_ulogic_f(CSR_COUNTERS_USE) and is_m_mode_v; -- minstret
      when x"b80" => csr_acc_valid <= bool_to_ulogic_f(CSR_COUNTERS_USE) and is_m_mode_v; -- mcycleh
      when x"b82" => csr_acc_valid <= bool_to_ulogic_f(CSR_COUNTERS_USE) and is_m_mode_v; -- minstreth
      --
      when x"f11" => csr_acc_valid <= is_m_mode_v; -- mvendorid
      when x"f12" => csr_acc_valid <= is_m_mode_v; -- marchid
      when x"f13" => csr_acc_valid <= is_m_mode_v; -- mimpid
      when x"f14" => csr_acc_valid <= is_m_mode_v; -- mhartid
      --
      when others => csr_acc_valid <= '0'; -- undefined
    end case;
  end process invalid_csr_access_check;


  -- Illegal Instruction Check --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  illegal_instruction_check: process(execute_engine, csr, ctrl_nxt, csr_acc_valid)
  begin
    -- illegal instructions are checked in the EXECUTE stage
    -- the execute engine will only commit valid instructions
    if (execute_engine.state = EXECUTE) then
      -- defaults --
      illegal_instruction <= '0';
      illegal_register    <= '0';

      -- check if using reg >= 16 for E-CPUs --
      --if (CPU_EXTENSION_RISCV_E = true) then
      --  illegal_register <= ????? FIXME
      --else
      --  illegal_register <= '0';
      --end if;

      -- check instructions --
      case execute_engine.i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c) is

        -- OPCODE check sufficient: LUI, UIPC, JAL --
        when opcode_lui_c | opcode_auipc_c | opcode_jal_c =>
          illegal_instruction <= '0';

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
      
        when opcode_store_c => -- check STORE funct3
          if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sb_c) or
             (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sh_c) or
             (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sw_c) then
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
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

        when opcode_jalr_c => -- check JALR funct3
          if (execute_engine.i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "000") then
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
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

        when others => -- compressed instruction or undefined instruction
          if (execute_engine.i_reg(1 downto 0) = "11") then -- undefined/unimplemented opcode
            illegal_instruction <= '1';
          end if;

      end case;
    else
      illegal_instruction <= '0';
      illegal_register    <= '0';
    end if;
  end process illegal_instruction_check;

  -- any illegal condition? --
  trap_ctrl.instr_il <= illegal_instruction or illegal_register or illegal_compressed;


-- ****************************************************************************************************************************
-- Exception and Interrupt Control
-- ****************************************************************************************************************************


  -- Trap Controller ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trap_controller: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      trap_ctrl.exc_buf   <= (others => '0');
      trap_ctrl.irq_buf   <= (others => '0');
      trap_ctrl.exc_ack   <= '0';
      trap_ctrl.irq_ack   <= (others => '0');
      trap_ctrl.cause     <= (others => '0');
      trap_ctrl.env_start <= '0';
    elsif rising_edge(clk_i) then
      if (CPU_EXTENSION_RISCV_Zicsr = true) then
        -- exception buffer: misaligned load/store/instruction address
        trap_ctrl.exc_buf(exception_lalign_c)    <= (trap_ctrl.exc_buf(exception_lalign_c)    or ma_load_i)             and (not trap_ctrl.exc_ack);
        trap_ctrl.exc_buf(exception_salign_c)    <= (trap_ctrl.exc_buf(exception_salign_c)    or ma_store_i)            and (not trap_ctrl.exc_ack);
        trap_ctrl.exc_buf(exception_ialign_c)    <= (trap_ctrl.exc_buf(exception_ialign_c)    or trap_ctrl.instr_ma)    and (not trap_ctrl.exc_ack);
        -- exception buffer: load/store/instruction bus access error
        trap_ctrl.exc_buf(exception_laccess_c)   <= (trap_ctrl.exc_buf(exception_laccess_c)   or be_load_i)             and (not trap_ctrl.exc_ack);
        trap_ctrl.exc_buf(exception_saccess_c)   <= (trap_ctrl.exc_buf(exception_saccess_c)   or be_store_i)            and (not trap_ctrl.exc_ack);
        trap_ctrl.exc_buf(exception_iaccess_c)   <= (trap_ctrl.exc_buf(exception_iaccess_c)   or trap_ctrl.instr_be)    and (not trap_ctrl.exc_ack);
        -- exception buffer: illegal instruction / env call / break point
        trap_ctrl.exc_buf(exception_m_envcall_c) <= (trap_ctrl.exc_buf(exception_m_envcall_c) or trap_ctrl.env_call)    and (not trap_ctrl.exc_ack);
        trap_ctrl.exc_buf(exception_break_c)     <= (trap_ctrl.exc_buf(exception_break_c)     or trap_ctrl.break_point) and (not trap_ctrl.exc_ack);
        trap_ctrl.exc_buf(exception_iillegal_c)  <= (trap_ctrl.exc_buf(exception_iillegal_c)  or trap_ctrl.instr_il)    and (not trap_ctrl.exc_ack);
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
             ((execute_engine.state = EXECUTE) or (execute_engine.state = TRAP))) then -- sample IRQs in EXECUTE or TRAP state only -> continue execution even if permanent IRQ
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
  trap_ctrl.exc_fire <= or_all_f(trap_ctrl.exc_buf); -- exceptions/faults cannot be masked
  trap_ctrl.irq_fire <= or_all_f(trap_ctrl.irq_buf) and csr.mstatus_mie; -- interrupts can be masked


  -- Trap Priority Detector -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trap_priority: process(trap_ctrl)
  begin
    -- defaults --
    trap_ctrl.cause_nxt   <= (others => '0');
    trap_ctrl.irq_ack_nxt <= (others => '0');

    -- the following traps are caused by asynchronous exceptions (-> interrupts)
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


    -- the following traps are caused by synchronous exceptions
    -- here we do not need a specific acknowledge mask since only one exception (the one
    -- with highest priority) can trigger at once

    -- trap/fault: 0.1 instruction access fault --
    elsif (trap_ctrl.exc_buf(exception_iaccess_c) = '1') then
      trap_ctrl.cause_nxt <= trap_iba_c;

    -- trap/fault: 0.2 illegal instruction --
    elsif (trap_ctrl.exc_buf(exception_iillegal_c) = '1') then
      trap_ctrl.cause_nxt <= trap_iil_c;

    -- trap/fault: 0.0 instruction address misaligned --
    elsif (trap_ctrl.exc_buf(exception_ialign_c) = '1') then
      trap_ctrl.cause_nxt <= trap_ima_c;


    -- trap/fault: 0.11 environment call from M-mode --
    elsif (trap_ctrl.exc_buf(exception_m_envcall_c) = '1') then
      trap_ctrl.cause_nxt <= trap_menv_c;

    -- trap/fault: 0.3 breakpoint --
    elsif (trap_ctrl.exc_buf(exception_break_c) = '1') then
      trap_ctrl.cause_nxt <= trap_brk_c;


    -- trap/fault: 0.6 store address misaligned -
    elsif (trap_ctrl.exc_buf(exception_salign_c) = '1') then
      trap_ctrl.cause_nxt <= trap_sma_c;

    -- trap/fault: 0.4 load address misaligned --
    elsif (trap_ctrl.exc_buf(exception_lalign_c) = '1') then
      trap_ctrl.cause_nxt <= trap_lma_c;

    -- trap/fault: 0.7 store access fault --
    elsif (trap_ctrl.exc_buf(exception_saccess_c) = '1') then
      trap_ctrl.cause_nxt <= trap_sbe_c;

    -- trap/fault: 0.5 load access fault --
    elsif (trap_ctrl.exc_buf(exception_laccess_c) = '1') then
      trap_ctrl.cause_nxt <= trap_lbe_c;

    -- undefined / not implemented --
    else
      trap_ctrl.cause_nxt   <= (others => '0');
      trap_ctrl.irq_ack_nxt <= (others => '0');
    end if;
  end process trap_priority;
  

-- ****************************************************************************************************************************
-- Control and Status Registers (CSRs)
-- ****************************************************************************************************************************

  -- Control and Status Registers Write Access ----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      csr.we <= '0';
      csr.re <= '0';
      --
      csr.mstatus_mie  <= '0';
      csr.mstatus_mpie <= '0';
      csr.mie_msie     <= '0';
      csr.mie_meie     <= '0';
      csr.mie_mtie     <= '0';
      csr.mie_firqe    <= (others => '0');
      csr.mtvec        <= (others => '0');
      csr.mscratch     <= (others => '0');
      csr.mepc         <= (others => '0');
      csr.mcause       <= (others => '0');
      csr.mtval        <= (others => '0');
      csr.mpp          <= m_priv_mode_c; -- start in MACHINE mode
      csr.privilege    <= m_priv_mode_c; -- start in MACHINE mode
      csr.pmpcfg       <= (others => (others => '0'));
      csr.pmpaddr      <= (others => (others => '0'));
    elsif rising_edge(clk_i) then
      if (CPU_EXTENSION_RISCV_Zicsr = true) then
        -- access --
        csr.we <= csr.we_nxt;
        csr.re <= csr.re_nxt;

        -- registers that can be modified by user --
        if (csr.we = '1') then -- manual update

          -- Machine CSRs --
          if (execute_engine.i_reg(31 downto 28) = x"3") then
            -- machine trap setup --
            if (execute_engine.i_reg(27 downto 24) = x"0") then
              case execute_engine.i_reg(23 downto 20) is
                when x"0" => -- R/W: mstatus - machine status register
                  csr.mstatus_mie  <= csr_wdata_i(03);
                  csr.mstatus_mpie <= csr_wdata_i(07);
                  --
                  if (CPU_EXTENSION_RISCV_U = true) then -- user mode implemented
                    csr.mpp(0) <= csr_wdata_i(11) and csr_wdata_i(12);
                    csr.mpp(1) <= csr_wdata_i(11) and csr_wdata_i(12);
                  end if;
                when x"4" => -- R/W: mie - machine interrupt-enable register
                  csr.mie_msie <= csr_wdata_i(03); -- machine SW IRQ enable
                  csr.mie_mtie <= csr_wdata_i(07); -- machine TIMER IRQ enable
                  csr.mie_meie <= csr_wdata_i(11); -- machine EXT IRQ enable
                  --
                  csr.mie_firqe(0) <= csr_wdata_i(16); -- fast interrupt channel 0
                  csr.mie_firqe(1) <= csr_wdata_i(17); -- fast interrupt channel 1
                  csr.mie_firqe(2) <= csr_wdata_i(18); -- fast interrupt channel 2
                  csr.mie_firqe(3) <= csr_wdata_i(19); -- fast interrupt channel 3
                when x"5" => -- R/W: mtvec - machine trap-handler base address (for ALL exceptions)
                  csr.mtvec <= csr_wdata_i(data_width_c-1 downto 2) & "00"; -- mtvec.MODE=0
                when others =>
                  NULL;
              end case;
            end if;
            -- machine trap handling --
            if (execute_engine.i_reg(27 downto 24) = x"4") then
              case execute_engine.i_reg(23 downto 20) is
                when x"0" => -- R/W: mscratch - machine scratch register
                  csr.mscratch <= csr_wdata_i;
                when x"1" => -- R/W: mepc - machine exception program counter
                  csr.mepc <= csr_wdata_i(data_width_c-1 downto 1) & '0';
                when x"3" => -- R/W: mtval - machine bad address or instruction
                  csr.mtval <= csr_wdata_i;
                when others =>
                  NULL;
              end case;
            end if;
            -- machine physical memory protection (pmp) --
            if (PMP_USE = true) then
              -- pmpcfg --
              if (execute_engine.i_reg(27 downto 24) = x"a") then
                if (PMP_NUM_REGIONS >= 1) then
                  if (execute_engine.i_reg(23 downto 20) = x"0") then -- pmpcfg0
                    for j in 0 to 3 loop -- bytes in pmpcfg CSR
                      if ((j+1) <= PMP_NUM_REGIONS) then
                        if (csr.pmpcfg(0+j)(7) = '0') then -- unlocked pmpcfg access
                          csr.pmpcfg(0+j)(0) <= csr_wdata_i(j*8+0); -- R
                          csr.pmpcfg(0+j)(1) <= csr_wdata_i(j*8+1); -- W
                          csr.pmpcfg(0+j)(2) <= csr_wdata_i(j*8+2); -- X
                          csr.pmpcfg(0+j)(3) <= csr_wdata_i(j*8+3) and csr_wdata_i(j*8+4); -- A_L
                          csr.pmpcfg(0+j)(4) <= csr_wdata_i(j*8+3) and csr_wdata_i(j*8+4); -- A_H - NAPOT/OFF only
                          csr.pmpcfg(0+j)(5) <= '0'; -- reserved
                          csr.pmpcfg(0+j)(6) <= '0'; -- reserved
                          csr.pmpcfg(0+j)(7) <= csr_wdata_i(j*8+7); -- L
                        end if;
                      end if;
                    end loop; -- j (bytes in CSR)
                  end if;
                end if;
                if (PMP_NUM_REGIONS >= 5) then
                  if (execute_engine.i_reg(23 downto 20) = x"1") then -- pmpcfg1
                    for j in 0 to 3 loop -- bytes in pmpcfg CSR
                      if ((j+1+4) <= PMP_NUM_REGIONS) then
                        if (csr.pmpcfg(4+j)(7) = '0') then -- unlocked pmpcfg access
                          csr.pmpcfg(4+j)(0) <= csr_wdata_i(j*8+0); -- R
                          csr.pmpcfg(4+j)(1) <= csr_wdata_i(j*8+1); -- W
                          csr.pmpcfg(4+j)(2) <= csr_wdata_i(j*8+2); -- X
                          csr.pmpcfg(4+j)(3) <= csr_wdata_i(j*8+3) and csr_wdata_i(j*8+4); -- A_L
                          csr.pmpcfg(4+j)(4) <= csr_wdata_i(j*8+3) and csr_wdata_i(j*8+4); -- A_H - NAPOT/OFF only
                          csr.pmpcfg(4+j)(5) <= '0'; -- reserved
                          csr.pmpcfg(4+j)(6) <= '0'; -- reserved
                          csr.pmpcfg(4+j)(7) <= csr_wdata_i(j*8+7); -- L
                        end if;
                      end if;
                    end loop; -- j (bytes in CSR)
                  end if;
                end if;
              end if;
              -- pmpaddr --
              if (execute_engine.i_reg(27 downto 24) = x"b") then
                for i in 0 to PMP_NUM_REGIONS-1 loop
                  if (execute_engine.i_reg(23 downto 20) = std_ulogic_vector(to_unsigned(i, 4))) and (csr.pmpcfg(i)(7) = '0') then -- unlocked pmpaddr access
                    csr.pmpaddr(i) <= csr_wdata_i;
                  end if;
                end loop; -- i (CSRs)
              end if;
            end if; -- implement PMP at all?
          end if;

        -- automatic update by hardware --
        else

          -- machine exception PC & machine trap value register --
          if (trap_ctrl.env_start_ack = '1') then -- trap handler starting?
            csr.mcause <= trap_ctrl.cause(trap_ctrl.cause'left) & "000" & x"00000" & "000" & trap_ctrl.cause(4 downto 0);
            if (trap_ctrl.cause(trap_ctrl.cause'left) = '1') then -- for INTERRUPTS only (is mcause(31))
              csr.mepc  <= execute_engine.pc(data_width_c-1 downto 1) & '0'; -- this is the CURRENT pc = interrupted instruction
              csr.mtval <= (others => '0'); -- mtval is zero for interrupts
            else -- for EXCEPTIONS (according to their priority)
              csr.mepc <= execute_engine.last_pc(data_width_c-1 downto 1) & '0'; -- this is the LAST pc = last executed instruction
              if (trap_ctrl.cause(4 downto 0) = trap_iba_c(4 downto 0)) or -- instr access error OR
                 (trap_ctrl.cause(4 downto 0) = trap_ima_c(4 downto 0)) or -- misaligned instruction OR
                 (trap_ctrl.cause(4 downto 0) = trap_brk_c(4 downto 0)) or -- breakpoint OR
                 (trap_ctrl.cause(4 downto 0) = trap_menv_c(4 downto 0)) then -- env call OR
                csr.mtval <= execute_engine.pc(data_width_c-1 downto 1) & '0'; -- address of faulting instruction
              elsif (trap_ctrl.cause(4 downto 0) = trap_iil_c(4 downto 0)) then -- illegal instruction
                csr.mtval <= execute_engine.i_reg; -- faulting instruction itself
              else -- load/store misalignments/access errors
                csr.mtval <= mar_i; -- faulting data access address
              end if;
            end if;
          end if;

          -- context switch in mstatus --
          if (trap_ctrl.env_start_ack = '1') then -- ENTER: trap handler starting?
            csr.mstatus_mie  <= '0'; -- disable interrupts
            csr.mstatus_mpie <= csr.mstatus_mie; -- buffer previous mie state
            if (CPU_EXTENSION_RISCV_U = true) then -- implement user mode
              csr.privilege <= m_priv_mode_c; -- execute trap in machine mode
              csr.mpp       <= csr.privilege; -- buffer previous privilege mode
            end if;
          elsif (trap_ctrl.env_end = '1') then -- EXIT: return from exception
            csr.mstatus_mie  <= csr.mstatus_mpie; -- restore global IRQ enable flag
            csr.mstatus_mpie <= '1';
            if (CPU_EXTENSION_RISCV_U = true) then -- implement user mode
              csr.privilege <= csr.mpp; -- go back to previous privilege mode
              csr.mpp       <= u_priv_mode_c;
            end if;
          end if;

          -- user mode NOT implemented --
          if (CPU_EXTENSION_RISCV_U = false) then -- implement user mode
            csr.privilege <= m_priv_mode_c;
            csr.mpp       <= m_priv_mode_c;
          end if;
        end if;
      end if;
    end if;
  end process csr_write_access;


  -- Control and Status Registers Read Access -----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      csr_rdata_o <= (others => '0'); -- default
      if (CPU_EXTENSION_RISCV_Zicsr = true) and (csr.re = '1') then
        case execute_engine.i_reg(31 downto 20) is

          -- machine trap setup --
          when x"300" => -- R/W: mstatus - machine status register
            csr_rdata_o(03) <= csr.mstatus_mie;  -- MIE
            csr_rdata_o(07) <= csr.mstatus_mpie; -- MPIE
            csr_rdata_o(11) <= csr.mpp(0); -- MPP: machine previous privilege mode low
            csr_rdata_o(12) <= csr.mpp(1); -- MPP: machine previous privilege mode high
          when x"301" => -- R/-: misa - ISA and extensions
            csr_rdata_o(02) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_C);     -- C CPU extension
            csr_rdata_o(04) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_E);     -- E CPU extension
            csr_rdata_o(08) <= not bool_to_ulogic_f(CPU_EXTENSION_RISCV_E); -- I CPU extension (if not E)
            csr_rdata_o(12) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_M);     -- M CPU extension
            csr_rdata_o(20) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_U);     -- U CPU extension
            csr_rdata_o(23) <= '1';                                         -- X CPU extension (non-std extensions)
            csr_rdata_o(30) <= '1'; -- 32-bit architecture (MXL lo)
            csr_rdata_o(31) <= '0'; -- 32-bit architecture (MXL hi)
          when x"304" => -- R/W: mie - machine interrupt-enable register
            csr_rdata_o(03) <= csr.mie_msie; -- machine software IRQ enable
            csr_rdata_o(07) <= csr.mie_mtie; -- machine timer IRQ enable
            csr_rdata_o(11) <= csr.mie_meie; -- machine external IRQ enable
            --
            csr_rdata_o(16) <= csr.mie_firqe(0); -- fast interrupt channel 0
            csr_rdata_o(17) <= csr.mie_firqe(1); -- fast interrupt channel 1
            csr_rdata_o(18) <= csr.mie_firqe(2); -- fast interrupt channel 2
            csr_rdata_o(19) <= csr.mie_firqe(3); -- fast interrupt channel 3
          when x"305" => -- R/W: mtvec - machine trap-handler base address (for ALL exceptions)
            csr_rdata_o <= csr.mtvec(data_width_c-1 downto 2) & "00"; -- mtvec.MODE=0

          -- machine trap handling --
          when x"340" => -- R/W: mscratch - machine scratch register
            csr_rdata_o <= csr.mscratch;
          when x"341" => -- R/W: mepc - machine exception program counter
            csr_rdata_o <= csr.mepc(data_width_c-1 downto 1) & '0';
          when x"342" => -- R/-: mcause - machine trap cause
            csr_rdata_o <= csr.mcause;
          when x"343" => -- R/W: mtval - machine bad address or instruction
            csr_rdata_o <= csr.mtval;
          when x"344" => -- R/W: mip - machine interrupt pending
            csr_rdata_o(03) <= trap_ctrl.irq_buf(interrupt_msw_irq_c);
            csr_rdata_o(07) <= trap_ctrl.irq_buf(interrupt_mtime_irq_c);
            csr_rdata_o(11) <= trap_ctrl.irq_buf(interrupt_mext_irq_c);
            --
            csr_rdata_o(16) <= trap_ctrl.irq_buf(interrupt_firq_0_c);
            csr_rdata_o(17) <= trap_ctrl.irq_buf(interrupt_firq_1_c);
            csr_rdata_o(18) <= trap_ctrl.irq_buf(interrupt_firq_2_c);
            csr_rdata_o(19) <= trap_ctrl.irq_buf(interrupt_firq_3_c);

          -- physical memory protection --
          when x"3a0" => -- R/W: pmpcfg0 - physical memory protection configuration register 0
            if (PMP_USE = true) then
              if (PMP_NUM_REGIONS >= 1) then
                csr_rdata_o(07 downto 00) <= csr.pmpcfg(0);
              end if;
              if (PMP_NUM_REGIONS >= 2) then
                csr_rdata_o(15 downto 08) <= csr.pmpcfg(1);
              end if;
              if (PMP_NUM_REGIONS >= 3) then
                csr_rdata_o(23 downto 16) <= csr.pmpcfg(2);
              end if;
              if (PMP_NUM_REGIONS >= 4) then
                csr_rdata_o(31 downto 24) <= csr.pmpcfg(3);
              end if;
            end if;
          when x"3a1" => -- R/W: pmpcfg1 - physical memory protection configuration register 1
            if (PMP_USE = true) then
              if (PMP_NUM_REGIONS >= 5) then
                csr_rdata_o(07 downto 00) <= csr.pmpcfg(4);
              end if;
              if (PMP_NUM_REGIONS >= 6) then
                csr_rdata_o(15 downto 08) <= csr.pmpcfg(5);
              end if;
              if (PMP_NUM_REGIONS >= 7) then
                csr_rdata_o(23 downto 16) <= csr.pmpcfg(6);
              end if;
              if (PMP_NUM_REGIONS >= 8) then
                csr_rdata_o(31 downto 24) <= csr.pmpcfg(7);
              end if;
            end if;

          when x"3b0" => -- R/W: pmpaddr0 - physical memory protection address register 0
            if (PMP_USE = true) and (PMP_NUM_REGIONS >= 1) then
              csr_rdata_o <= csr.pmpaddr(0);
              if (csr.pmpcfg(0)(4 downto 3) = "00") then -- mode = off
                csr_rdata_o(PMP_GRANULARITY-1 downto 0) <= (others => '0'); -- required for granularity check by SW
              else -- mode = NAPOT
                csr_rdata_o(PMP_GRANULARITY-2 downto 0) <= (others => '1');
              end if;
            end if;
          when x"3b1" => -- R/W: pmpaddr1 - physical memory protection address register 1
            if (PMP_USE = true) and (PMP_NUM_REGIONS >= 2) then
              csr_rdata_o <= csr.pmpaddr(1);
              if (csr.pmpcfg(1)(4 downto 3) = "00") then -- mode = off
                csr_rdata_o(PMP_GRANULARITY-1 downto 0) <= (others => '0'); -- required for granularity check by SW
              else -- mode = NAPOT
                csr_rdata_o(PMP_GRANULARITY-2 downto 0) <= (others => '1');
              end if;
            end if;
          when x"3b2" => -- R/W: pmpaddr2 - physical memory protection address register 2
            if (PMP_USE = true) and (PMP_NUM_REGIONS >= 3) then
              csr_rdata_o <= csr.pmpaddr(2);
              if (csr.pmpcfg(2)(4 downto 3) = "00") then -- mode = off
                csr_rdata_o(PMP_GRANULARITY-1 downto 0) <= (others => '0'); -- required for granularity check by SW
              else -- mode = NAPOT
                csr_rdata_o(PMP_GRANULARITY-2 downto 0) <= (others => '1');
              end if;
            end if;
          when x"3b3" => -- R/W: pmpaddr3 - physical memory protection address register 3
            if (PMP_USE = true) and (PMP_NUM_REGIONS >= 4) then
              csr_rdata_o <= csr.pmpaddr(3);
              if (csr.pmpcfg(3)(4 downto 3) = "00") then -- mode = off
                csr_rdata_o(PMP_GRANULARITY-1 downto 0) <= (others => '0'); -- required for granularity check by SW
              else -- mode = NAPOT
                csr_rdata_o(PMP_GRANULARITY-2 downto 0) <= (others => '1');
              end if;
            end if;
          when x"3b4" => -- R/W: pmpaddr4 - physical memory protection address register 4
            if (PMP_USE = true) and (PMP_NUM_REGIONS >= 5) then
              csr_rdata_o <= csr.pmpaddr(4);
              if (csr.pmpcfg(4)(4 downto 3) = "00") then -- mode = off
                csr_rdata_o(PMP_GRANULARITY-1 downto 0) <= (others => '0'); -- required for granularity check by SW
              else -- mode = NAPOT
                csr_rdata_o(PMP_GRANULARITY-2 downto 0) <= (others => '1');
              end if;
            end if;
          when x"3b5" => -- R/W: pmpaddr5 - physical memory protection address register 5
            if (PMP_USE = true) and (PMP_NUM_REGIONS >= 6) then
              csr_rdata_o <= csr.pmpaddr(5);
              if (csr.pmpcfg(5)(4 downto 3) = "00") then -- mode = off
                csr_rdata_o(PMP_GRANULARITY-1 downto 0) <= (others => '0'); -- required for granularity check by SW
              else -- mode = NAPOT
                csr_rdata_o(PMP_GRANULARITY-2 downto 0) <= (others => '1');
              end if;
            end if;
          when x"3b6" => -- R/W: pmpaddr6 - physical memory protection address register 6
            if (PMP_USE = true) and (PMP_NUM_REGIONS >= 7) then
              csr_rdata_o <= csr.pmpaddr(6);
              if (csr.pmpcfg(6)(4 downto 3) = "00") then -- mode = off
                csr_rdata_o(PMP_GRANULARITY-1 downto 0) <= (others => '0'); -- required for granularity check by SW
              else -- mode = NAPOT
                csr_rdata_o(PMP_GRANULARITY-2 downto 0) <= (others => '1');
              end if;
            end if;
          when x"3b7" => -- R/W: pmpaddr7 - physical memory protection address register 7
            if (PMP_USE = true) and (PMP_NUM_REGIONS >= 8) then
              csr_rdata_o <= csr.pmpaddr(7);
              if (csr.pmpcfg(7)(4 downto 3) = "00") then -- mode = off
                csr_rdata_o(PMP_GRANULARITY-1 downto 0) <= (others => '0'); -- required for granularity check by SW
              else -- mode = NAPOT
                csr_rdata_o(PMP_GRANULARITY-2 downto 0) <= (others => '1');
              end if;
            end if;

          -- counter and timers --
          when x"c00" | x"b00" => -- R/(W): cycle/mcycle: Cycle counter LOW
            csr_rdata_o <= csr.mcycle(31 downto 0);
          when x"c01" => -- R/-: time: System time LOW (from MTIME unit)
            csr_rdata_o <= systime(31 downto 0);
          when x"c02" | x"b02" => -- R/(W): instret/minstret: Instructions-retired counter LOW
            csr_rdata_o <= csr.minstret(31 downto 0);
          when x"c80" | x"b80" => -- R/(W): cycleh/mcycleh: Cycle counter HIGH
            csr_rdata_o <= x"000" & csr.mcycleh(19 downto 0); -- only the lowest 20 bit!
          when x"c81" => -- R/-: timeh: System time HIGH (from MTIME unit)
            csr_rdata_o <= systime(63 downto 32);
          when x"c82" | x"b82" => -- R/(W): instreth/minstreth: Instructions-retired counter HIGH
            csr_rdata_o <= x"000" & csr.minstreth(19 downto 0); -- only the lowest 20 bit!

          -- machine information registers --
          when x"f11" => -- R/-: mvendorid
            csr_rdata_o <= (others => '0'); -- not available for NEORV32
          when x"f12" => -- R/-: marchid
            csr_rdata_o <= (others => '0'); -- not available for NEORV32
          when x"f13" => -- R/-: mimpid - implementation ID / NEORV32: version
            csr_rdata_o <= hw_version_c;
          when x"f14" => -- R/-: mhartid - hardware thread ID
            csr_rdata_o <= HW_THREAD_ID;

          -- undefined/unavailable --
          when others =>
            csr_rdata_o <= (others => '0'); -- not implemented

        end case;
      else
        csr_rdata_o <= (others => '0');
      end if;
    end if;
  end process csr_read_access;

  -- time[h] CSR --
  systime <= time_i when (CSR_COUNTERS_USE = true) else (others => '0');

  -- CPU's current privilege level --
  priv_mode_o <= csr.privilege;

  -- PMP output --
  pmp_output: process(csr)
  begin
    pmp_addr_o <= (others => (others => '0'));
    pmp_ctrl_o <= (others => (others => '0'));
    if (PMP_USE = true) then
      for i in 0 to PMP_NUM_REGIONS-1 loop
        pmp_addr_o(i) <= csr.pmpaddr(i) & "00";
        pmp_ctrl_o(i) <= csr.pmpcfg(i);
      end loop; -- i
    end if;
  end process pmp_output;


  -- RISC-V Counter CSRs --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_counters: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      csr.mcycle    <= (others => '0');
      csr.minstret  <= (others => '0');
      csr.mcycleh   <= (others => '0');
      csr.minstreth <= (others => '0');
      mcycle_msb    <= '0';
      minstret_msb  <= '0';
    elsif rising_edge(clk_i) then
      if (CSR_COUNTERS_USE = true) then

        -- mcycle (cycle) --
        mcycle_msb <= csr.mcycle(csr.mcycle'left);
        if (csr.we = '1') and (execute_engine.i_reg(31 downto 20) = x"b00") then -- write access
          csr.mcycle(31 downto 0) <= csr_wdata_i;
          csr.mcycle(32) <= '0';
        elsif (execute_engine.sleep = '0') then -- automatic update
          csr.mcycle <= std_ulogic_vector(unsigned(csr.mcycle) + 1);
        end if;

        -- mcycleh (cycleh) --
        if (csr.we = '1') and (execute_engine.i_reg(31 downto 20) = x"b80") then -- write access
          csr.mcycleh <= csr_wdata_i(19 downto 0);
        elsif ((mcycle_msb xor csr.mcycle(csr.mcycle'left)) = '1') then -- automatic update
          csr.mcycleh <= std_ulogic_vector(unsigned(csr.mcycleh) + 1);
        end if;

        -- minstret (instret) --
        minstret_msb <= csr.minstret(csr.minstret'left);
        if (csr.we = '1') and (execute_engine.i_reg(31 downto 20) = x"b02") then -- write access
          csr.minstret(31 downto 0) <= csr_wdata_i;
          csr.minstret(32) <= '0';
        elsif (execute_engine.state_prev /= EXECUTE) and (execute_engine.state = EXECUTE) then -- automatic update
          csr.minstret <= std_ulogic_vector(unsigned(csr.minstret) + 1);
        end if;

        -- minstreth (instreth) --
        if (csr.we = '1') and (execute_engine.i_reg(31 downto 20) = x"b82") then -- write access
          csr.minstreth <= csr_wdata_i(19 downto 0);
        elsif ((minstret_msb xor csr.minstret(csr.minstret'left)) = '1') then -- automatic update
          csr.minstreth <= std_ulogic_vector(unsigned(csr.minstreth) + 1);
        end if;

      else -- if not implemented
        csr.mcycle    <= (others => '0');
        csr.minstret  <= (others => '0');
        csr.mcycleh   <= (others => '0');
        csr.minstreth <= (others => '0');
        mcycle_msb    <= '0';
        minstret_msb  <= '0';
      end if;
    end if;
  end process csr_counters;


end neorv32_cpu_control_rtl;
