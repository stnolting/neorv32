-- #################################################################################################
-- # << NEORV32 - CPU Control >>                                                                   #
-- # ********************************************************************************************* #
-- # FSM to control CPU operations. This unit also includes the control and status registers (CSR) #
-- # and the interrupt and exception controller.                                                   #
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
    CLOCK_FREQUENCY           : natural := 0; -- clock frequency of clk_i in Hz
    HART_ID                   : std_ulogic_vector(31 downto 0) := x"00000000"; -- custom hardware thread ID
    BOOTLOADER_USE            : boolean := true;   -- implement processor-internal bootloader?
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_C     : boolean := false;  -- implement compressed extension?
    CPU_EXTENSION_RISCV_E     : boolean := false;  -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M     : boolean := false;  -- implement muld/div extension?
    CPU_EXTENSION_RISCV_Zicsr : boolean := true;   -- implement CSR system?
    -- Memory configuration: Instruction memory --
    MEM_ISPACE_BASE           : std_ulogic_vector(31 downto 0) := x"00000000"; -- base address of instruction memory space
    MEM_ISPACE_SIZE           : natural := 8*1024; -- total size of instruction memory space in byte
    MEM_INT_IMEM_USE          : boolean := true;   -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE         : natural := 8*1024; -- size of processor-internal instruction memory in bytes
    MEM_INT_IMEM_ROM          : boolean := false;  -- implement processor-internal instruction memory as ROM
    -- Memory configuration: Data memory --
    MEM_DSPACE_BASE           : std_ulogic_vector(31 downto 0) := x"80000000"; -- base address of data memory space
    MEM_DSPACE_SIZE           : natural := 4*1024; -- total size of data memory space in byte
    MEM_INT_DMEM_USE          : boolean := true;   -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE         : natural := 4*1024; -- size of processor-internal data memory in bytes
    -- Memory configuration: External memory interface --
    MEM_EXT_USE               : boolean := false;  -- implement external memory bus interface?
    -- Processor peripherals --
    IO_GPIO_USE               : boolean := true;   -- implement general purpose input/output port unit (GPIO)?
    IO_MTIME_USE              : boolean := true;   -- implement machine system timer (MTIME)?
    IO_UART_USE               : boolean := true;   -- implement universal asynchronous receiver/transmitter (UART)?
    IO_SPI_USE                : boolean := true;   -- implement serial peripheral interface (SPI)?
    IO_TWI_USE                : boolean := true;   -- implement two-wire interface (TWI)?
    IO_PWM_USE                : boolean := true;   -- implement pulse-width modulation unit (PWM)?
    IO_WDT_USE                : boolean := true;   -- implement watch dog timer (WDT)?
    IO_CLIC_USE               : boolean := true;   -- implement core local interrupt controller (CLIC)?
    IO_TRNG_USE               : boolean := true    -- implement true random number generator (TRNG)?
  );
  port (
    -- global control --
    clk_i         : in  std_ulogic; -- global clock, rising edge
    rstn_i        : in  std_ulogic; -- global reset, low-active, async
    ctrl_o        : out std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
    -- status input --
    alu_wait_i    : in  std_ulogic; -- wait for ALU
    bus_wait_i    : in  std_ulogic; -- wait for bus
    -- data input --
    instr_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- instruction
    cmp_i         : in  std_ulogic_vector(1 downto 0); -- comparator status
    alu_add_i     : in  std_ulogic_vector(data_width_c-1 downto 0); -- ALU.add result
    -- data output --
    imm_o         : out std_ulogic_vector(data_width_c-1 downto 0); -- immediate
    pc_o          : out std_ulogic_vector(data_width_c-1 downto 0); -- current PC
    alu_pc_o      : out std_ulogic_vector(data_width_c-1 downto 0); -- delayed PC for ALU
    -- csr data interface --
    csr_wdata_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- CSR write data
    csr_rdata_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- CSR read data
    -- external interrupt --
    clic_irq_i    : in  std_ulogic; -- CLIC interrupt request
    mtime_irq_i   : in  std_ulogic; -- machine timer interrupt
    -- bus access exceptions --
    mar_i         : in  std_ulogic_vector(data_width_c-1 downto 0);  -- memory address register
    ma_instr_i    : in  std_ulogic; -- misaligned instruction address
    ma_load_i     : in  std_ulogic; -- misaligned load data address
    ma_store_i    : in  std_ulogic; -- misaligned store data address
    be_instr_i    : in  std_ulogic; -- bus error on instruction access
    be_load_i     : in  std_ulogic; -- bus error on load data access
    be_store_i    : in  std_ulogic; -- bus error on store data access
    bus_exc_ack_o : out std_ulogic  -- bus exception error acknowledge
  );
end neorv32_cpu_control;

architecture neorv32_cpu_control_rtl of neorv32_cpu_control is

  -- state machine --
  type state_t is (IFETCH_0, IFETCH_1, IFETCH_2, IFETCH_3, IFETCH_4, IFETCH_5, IFETCH_6, EXECUTE,
                   ALU_WAIT, STORE_0, LOAD_0, LOADSTORE_0, LOADSTORE_1, CSR_ACCESS, SLEEP);
  signal state, state_nxt : state_t;
  signal ctrl_nxt, ctrl   : std_ulogic_vector(ctrl_width_c-1 downto 0);
  signal hw_control       : std_ulogic_vector(data_width_c-1 downto 0);

  -- pre-decoder --
  signal ci_instr32 : std_ulogic_vector(31 downto 0);
  signal ci_valid   : std_ulogic;
  signal ci_illegal : std_ulogic;

  -- instruction register --
  signal i_reg,  i_reg_nxt  : std_ulogic_vector(31 downto 0);
  signal i_buf,  i_buf_nxt  : std_ulogic_vector(15 downto 0);
  signal ci_reg, ci_reg_nxt : std_ulogic_vector(15 downto 0);
  signal iavail, iavail_nxt : std_ulogic;
  signal is_ci,  is_ci_nxt  : std_ulogic; -- current instruction is COMPRESSED instruction flag

  -- immediates --
  signal imm_reg : std_ulogic_vector(data_width_c-1 downto 0);

  -- branch system --
  signal is_branch     : std_ulogic;
  signal is_branch_nxt : std_ulogic;
  signal branch_taken  : std_ulogic;

  -- program counter --
  signal pc_reg         : std_ulogic_vector(data_width_c-1 downto 0); -- actual PC
  signal pc_backup_reg  : std_ulogic_vector(data_width_c-1 downto 0); -- delayed PC (for ALU operations)
  signal pc_backup2_reg : std_ulogic_vector(data_width_c-1 downto 0); -- delayed delayed PC (for exception handling)
  signal mepc           : std_ulogic_vector(data_width_c-1 downto 0); -- exception PC

  -- irq controller --
  signal exc_buf       : std_ulogic_vector(exception_width_c-1 downto 0);
  signal exc_ack       : std_ulogic_vector(exception_width_c-1 downto 0);
  signal exc_ack_nxt   : std_ulogic_vector(exception_width_c-1 downto 0);
  signal exc_src       : std_ulogic_vector(exception_width_c-1 downto 0);
  signal exc_fire      : std_ulogic;
  signal irq_buf       : std_ulogic_vector(interrupt_width_c-1 downto 0);
  signal irq_ack       : std_ulogic_vector(interrupt_width_c-1 downto 0);
  signal irq_ack_nxt   : std_ulogic_vector(interrupt_width_c-1 downto 0);
  signal irq_fire      : std_ulogic;
  signal exc_cpu_start : std_ulogic; -- starting exception env
  signal exc_cpu_ack   : std_ulogic; -- starting of exception env acknowledge
  signal exc_cpu_end   : std_ulogic; -- exiting eception env
  signal exc_cause     : std_ulogic_vector(data_width_c-1 downto 0);
  signal exc_cause_nxt : std_ulogic_vector(data_width_c-1 downto 0);

  -- RISC-V CSRs --
  signal mstatus_mie    : std_ulogic; -- mstatus.MIE: global IRQ enable (R/W)
  signal mstatus_mpie   : std_ulogic; -- mstatus.MPIE: previous global IRQ enable (R/-)
  signal mip_msip       : std_ulogic; -- mip.MSIP: machine software interrupt pending (R/W)
  signal mie_msie       : std_ulogic; -- mie.MSIE: machine software interrupt enable (R/W)
  signal mie_meie       : std_ulogic; -- mie.MEIE: machine external interrupt enable (R/W)
  signal mie_mtie       : std_ulogic; -- mie.MEIE: machine timer interrupt enable (R/W)
  signal mtvec          : std_ulogic_vector(data_width_c-1 downto 0); -- mtvec: machine trap-handler base address (R/W)
  signal mtval          : std_ulogic_vector(data_width_c-1 downto 0); -- mtval: machine bad address or isntruction (R/-)
  signal mscratch       : std_ulogic_vector(data_width_c-1 downto 0); -- mscratch: scratch register (R/W)
  signal mtinst         : std_ulogic_vector(data_width_c-1 downto 0); -- mtinst: machine trap instruction (transformed) (R/-)
  signal cycle_lo       : std_ulogic_vector(32 downto 0); -- cycle, mtime (R/-)
  signal instret_lo     : std_ulogic_vector(32 downto 0); -- instret (R/-)
  signal cycle_hi       : std_ulogic_vector(15 downto 0); -- cycleh, mtimeh (R/-) - only 16-bit wide
  signal instret_hi     : std_ulogic_vector(15 downto 0); -- instreth (R/-) - only 16-bit wide
  signal cycle_lo_msb   : std_ulogic;
  signal instret_lo_msb : std_ulogic;

  -- illegal instruction check ..
  signal illegal_instruction : std_ulogic;
  signal illegal_register    : std_ulogic; -- only for E-extension
  signal illegal_compressed  : std_ulogic; -- only fir C-extension

  -- synchronous exceptions trigger --
  signal illegal_instr_exc : std_ulogic;
  signal env_call          : std_ulogic;
  signal break_point       : std_ulogic;

begin

  -- Compressed Instructions Recoding -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_decompressor_inst_true:
  if (CPU_EXTENSION_RISCV_C = true) generate
    neorv32_cpu_decompressor_inst: neorv32_cpu_decompressor
    port map (
      -- instruction input --
      ci_instr16_i => ci_reg,     -- compressed instruction input
      -- instruction output --
      ci_valid_o   => ci_valid,   -- is a compressed instruction
      ci_illegal_o => ci_illegal, -- is an illegal compressed instruction
      ci_instr32_o => ci_instr32  -- 32-bit decompressed instruction
    );
  end generate;

  neorv32_cpu_decompressor_inst_false:
  if (CPU_EXTENSION_RISCV_C = false) generate
    ci_instr32 <= instr_i;
    ci_valid   <= '0';
    ci_illegal <= '0';
  end generate;


  -- Immediate Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  imm_gen: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- default: I-immediate --
      imm_reg(31 downto 11) <= (others => i_reg(31)); -- sign extension
      imm_reg(10 downto 05) <= i_reg(30 downto 25);
      imm_reg(04 downto 01) <= i_reg(24 downto 21);
      imm_reg(00)           <= i_reg(20);
      case i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c) is
        when opcode_store_c => -- S-immediate
          imm_reg(31 downto 11) <= (others => i_reg(31)); -- sign extension
          imm_reg(10 downto 05) <= i_reg(30 downto 25);
          imm_reg(04 downto 01) <= i_reg(11 downto 08);
          imm_reg(00)           <= i_reg(07);
        when opcode_branch_c => -- B-immediate
          imm_reg(31 downto 12) <= (others => i_reg(31)); -- sign extension
          imm_reg(11)           <= i_reg(07);
          imm_reg(10 downto 05) <= i_reg(30 downto 25);
          imm_reg(04 downto 01) <= i_reg(11 downto 08);
          imm_reg(00)           <= '0';
        when opcode_lui_c | opcode_auipc_c => -- U-immediate
          imm_reg(31 downto 20) <= i_reg(31 downto 20);
          imm_reg(19 downto 12) <= i_reg(19 downto 12);
          imm_reg(11 downto 00) <= (others => '0');
        when opcode_jal_c => -- J-immediate
          imm_reg(31 downto 20) <= (others => i_reg(31)); -- sign extension
          imm_reg(19 downto 12) <= i_reg(19 downto 12);
          imm_reg(11)           <= i_reg(20);
          imm_reg(10 downto 05) <= i_reg(30 downto 25);
          imm_reg(04 downto 01) <= i_reg(24 downto 21);
          imm_reg(00)           <= '0';
        when opcode_syscsr_c => -- CSR-immediate
          imm_reg(31 downto 05) <= (others => '0');
          imm_reg(04 downto 00) <= i_reg(19 downto 15);
        when others => -- I-immediate
          imm_reg(31 downto 11) <= (others => i_reg(31)); -- sign extension
          imm_reg(10 downto 05) <= i_reg(30 downto 25);
          imm_reg(04 downto 01) <= i_reg(24 downto 21);
          imm_reg(00)           <= i_reg(20);
      end case;
    end if;
  end process imm_gen;

  -- output --
  imm_o <= imm_reg;


  -- Branch Condition Check -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  branch_check: process(i_reg, cmp_i)
  begin
    case i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) is
      when funct3_beq_c => -- branch if equal
        branch_taken <= cmp_i(alu_cmp_equal_c);
      when funct3_bne_c => -- branch if not equal
        branch_taken <= not cmp_i(alu_cmp_equal_c);
      when funct3_blt_c | funct3_bltu_c => -- branch if less (signed/unsigned)
        branch_taken <= cmp_i(alu_cmp_less_c);
      when funct3_bge_c | funct3_bgeu_c => -- branch if greater or equal (signed/unsigned)
        branch_taken <= not cmp_i(alu_cmp_less_c);
      when others => -- undefined
        branch_taken <= '0';
    end case;
  end process branch_check;


  -- Arbiter State Machine Sync -------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  arbiter_sync_rst: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      -- these registers REQUIRE a specific reset state
      state <= IFETCH_0;
    elsif rising_edge(clk_i) then
      state <= state_nxt;
    end if;
  end process arbiter_sync_rst;

  arbiter_sync: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- these registers do not need a specific reset state
      ctrl      <= ctrl_nxt;
      i_reg     <= i_reg_nxt;
      i_buf     <= i_buf_nxt;
      ci_reg    <= ci_reg_nxt;
      iavail    <= iavail_nxt;
      is_ci     <= is_ci_nxt;
      is_branch <= is_branch_nxt;
    end if;
  end process arbiter_sync;

  -- control bus output --
  ctrl_outpu: process(ctrl, i_reg)
  begin
    ctrl_o <= ctrl;
    -- direct output of register addresses --
    ctrl_o(ctrl_rf_rd_adr4_c  downto ctrl_rf_rd_adr0_c)  <= i_reg(instr_rd_msb_c  downto instr_rd_lsb_c);
    ctrl_o(ctrl_rf_rs1_adr4_c downto ctrl_rf_rs1_adr0_c) <= i_reg(instr_rs1_msb_c downto instr_rs1_lsb_c);
    ctrl_o(ctrl_rf_rs2_adr4_c downto ctrl_rf_rs2_adr0_c) <= i_reg(instr_rs2_msb_c downto instr_rs2_lsb_c);
  end process ctrl_outpu;


  -- Arbiter State Machine Comb -----------------------------------------------
  -- -----------------------------------------------------------------------------
  arbiter_comb: process(state, ctrl, i_reg, alu_wait_i, bus_wait_i, exc_cpu_start, ma_load_i, be_load_i, ma_store_i, be_store_i,
                        i_reg, ci_reg, i_buf, instr_i, is_ci, iavail, pc_backup_reg, ci_valid, ci_instr32)
    variable alu_immediate_v : std_ulogic;
    variable alu_operation_v : std_ulogic_vector(2 downto 0);
    variable rs1_is_r0_v     : std_ulogic;
    variable rd_is_r0_v      : std_ulogic;
  begin
    -- arbiter defaults --
    state_nxt     <= state;
    is_branch_nxt <= '0';
    exc_cpu_ack   <= '0';
    exc_cpu_end   <= '0';

    i_reg_nxt  <= i_reg;
    i_buf_nxt  <= i_buf;
    ci_reg_nxt <= ci_reg;
    iavail_nxt <= iavail;
    is_ci_nxt  <= is_ci;

    -- exception trigger --
    env_call    <= '0';
    break_point <= '0';

    -- control defaults --
    ctrl_nxt <= (others => '0'); -- all off at first
    ctrl_nxt(ctrl_bus_unsigned_c)   <= i_reg(instr_funct3_msb_c); -- unsigned LOAD (LBU, LHU)
    if (i_reg(instr_opcode_lsb_c+4) = '1') then -- ALU ops
      ctrl_nxt(ctrl_alu_unsigned_c) <= i_reg(instr_funct3_lsb_c+0); -- unsigned ALU operation (SLTIU, SLTU)
    else -- branches
      ctrl_nxt(ctrl_alu_unsigned_c) <= i_reg(instr_funct3_lsb_c+1); -- unsigned branches (BLTU, BGEU)
    end if;
    ctrl_nxt(ctrl_alu_shift_dir_c)  <= i_reg(instr_funct3_msb_c); -- shift direction
    ctrl_nxt(ctrl_alu_shift_ar_c)   <= i_reg(30); -- arithmetic shift
    ctrl_nxt(ctrl_bus_size_lsb_c)   <= i_reg(instr_funct3_lsb_c+0); -- transfer size lsb (00=byte, 01=half-word)
    ctrl_nxt(ctrl_bus_size_msb_c)   <= i_reg(instr_funct3_lsb_c+1); -- transfer size msb (10=word, 11=?)
    ctrl_nxt(ctrl_alu_cmd2_c  downto ctrl_alu_cmd0_c)  <= alu_cmd_add_c; -- actual ALU operation = add
    ctrl_nxt(ctrl_cp_cmd2_c   downto ctrl_cp_cmd0_c)   <= i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c); -- CP operation
    ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_muldiv_c; -- only CP0 implemented yet

    -- is immediate operation? --
    alu_immediate_v := '0';
    if (i_reg(instr_opcode_msb_c-1) = '0') then
      alu_immediate_v := '1';
    end if;

    -- hardware branch operation control --
    hw_control(31 downto 24) <= ('0' & funct3_bne_c) & ('1' & funct3_bne_c);
    hw_control(23 downto 16) <= funct3_xor_c & funct3_slt_c & "00";
    hw_control(15 downto 08) <= funct3_beq_c & funct3_bne_c & "11";
    hw_control(07 downto 00) <= funct3_beq_c & funct3_bne_c & "00";

    -- alu operation --
    case i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) is
      when funct3_subadd_c => -- SUB / ADD(I)
        if (alu_immediate_v = '0') and (i_reg(instr_funct7_msb_c-1) = '1') then -- not immediate and funct7 = SUB
          alu_operation_v := alu_cmd_sub_c;
        else
          alu_operation_v := alu_cmd_add_c;
        end if;
      when funct3_sll_c  => alu_operation_v := alu_cmd_shift_c; -- SLL(I)
      when funct3_slt_c  => alu_operation_v := alu_cmd_slt_c; -- SLT(I)
      when funct3_sltu_c => alu_operation_v := alu_cmd_slt_c; -- SLTU(I)
      when funct3_xor_c  => alu_operation_v := alu_cmd_xor_c; -- XOR(I)
      when funct3_sr_c   => alu_operation_v := alu_cmd_shift_c; -- SRL(I) / SRA(I)
      when funct3_or_c   => alu_operation_v := alu_cmd_or_c; -- OR(I)
      when funct3_and_c  => alu_operation_v := alu_cmd_and_c; -- AND(I)
      when others        => alu_operation_v := (others => '-'); -- undefined
    end case;

    -- is rs1 = r0? --
    rs1_is_r0_v := '0';
    if (i_reg(instr_rs1_msb_c downto instr_rs1_lsb_c) = "00000") then
      rs1_is_r0_v := '1';
    end if;

    -- is rd = r0? --
    rd_is_r0_v := '0';
    if (i_reg(instr_rd_msb_c downto instr_rd_lsb_c) = "00000") then
      rd_is_r0_v := '1';
    end if;


    -- state machine: instruction fetch and execution --
    case state is

      when IFETCH_0 => -- output current PC to bus system
      -- ------------------------------------------------------------
      ctrl_nxt(ctrl_bus_if_c) <= '1'; -- instruction fetch request (output PC to bus address)
      iavail_nxt <= '0'; -- absolutely no instruction available yet
      state_nxt  <= IFETCH_1;

      when IFETCH_1 => -- memory latency, update PC
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_alu_opa_mux_msb_c downto ctrl_alu_opa_mux_lsb_c) <= "11"; -- opa = current PC
        ctrl_nxt(ctrl_alu_opb_mux_msb_c downto ctrl_alu_opb_mux_lsb_c) <= "11"; -- opb = PC_increment
        ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_add_c; -- actual ALU operation = ADD
        ctrl_nxt(ctrl_csr_pc_we_c) <= '1'; -- update PC
        state_nxt <= IFETCH_2;

      when IFETCH_2 => -- update instruction buffers
      -- ------------------------------------------------------------
        if (exc_cpu_start = '1') then -- exception detected!
          exc_cpu_ack <= '1';
          state_nxt <= IFETCH_0; -- start new instruction fetch
        else -- normal operation
          -- instruction register update --
          if (CPU_EXTENSION_RISCV_C = true) then -- compressed AND uncompressed instructions possible
            if (pc_backup_reg(1) = '0') then
              i_buf_nxt  <= instr_i(31 downto 16);
              ci_reg_nxt <= instr_i(15 downto 00);
            else
              i_buf_nxt  <= instr_i(15 downto 00);
              ci_reg_nxt <= instr_i(31 downto 16);
            end if;
          else -- only uncompressed instructions
            i_reg_nxt <= instr_i(31 downto 0);
          end if;
          -- next state --
          if (bus_wait_i = '0') then -- wait for bus response
            if (CPU_EXTENSION_RISCV_C = true) then -- compressed AND uncompressed instructions possible
              state_nxt <= IFETCH_3;
            else -- only uncompressed instructions
              state_nxt <= EXECUTE;
            end if;
          end if;
        end if;


      when IFETCH_3 => -- check for exception, start instruction execution (only available for C-extension)
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_alu_opa_mux_msb_c downto ctrl_alu_opa_mux_lsb_c) <= "11"; -- opa = current PC
        ctrl_nxt(ctrl_alu_opb_mux_msb_c downto ctrl_alu_opb_mux_lsb_c) <= "11"; -- opb = PC_increment
        ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_add_c; -- actual ALU operation = ADD
        --
        if (exc_cpu_start = '1') then -- exception detected!
          exc_cpu_ack <= '1';
          state_nxt   <= IFETCH_0; -- start new instruction fetch
        else -- normal operation
          if (ci_valid = '1') then -- directly execute decoded compressed instruction
            i_reg_nxt <= ci_instr32;
            state_nxt <= EXECUTE;
          elsif (pc_backup_reg(1) = '0') or (iavail = '1') then -- 32-bit aligned uncompressed instruction
            i_reg_nxt <= i_buf & ci_reg;
            state_nxt <= EXECUTE;
            ctrl_nxt(ctrl_csr_pc_we_c) <= not pc_backup_reg(1); -- update PC again when on 32b-aligned address
          else
            i_reg_nxt <= i_buf & ci_reg;
            state_nxt <= IFETCH_4;
          end if;
        end if;

      when IFETCH_4 => -- get missing instruction parts: output current PC to bus system (only available for C-extension)
      -- ------------------------------------------------------------
      ctrl_nxt(ctrl_bus_if_c) <= '1'; -- instruction fetch request (output PC to bus address)
      state_nxt <= IFETCH_5;

      when IFETCH_5 => -- memory latency, update PC (only available for C-extension)
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_alu_opa_mux_msb_c downto ctrl_alu_opa_mux_lsb_c) <= "11"; -- opa = current PC
        ctrl_nxt(ctrl_alu_opb_mux_msb_c downto ctrl_alu_opb_mux_lsb_c) <= "11"; -- opb = PC_increment
        ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_add_c; -- actual ALU operation = ADD
        ctrl_nxt(ctrl_csr_pc_we_c) <= '1'; -- update PC
        state_nxt <= IFETCH_6;

      when IFETCH_6 => -- update missing instruction buffer parts (only available for C-extension)
      -- ------------------------------------------------------------
        if (bus_wait_i = '0') then -- wait for bus response
          i_buf_nxt  <= instr_i(15 downto 00);
          iavail_nxt <= '1';
          state_nxt  <= IFETCH_3;
        end if;


      when EXECUTE => -- decode and execute instruction
      -- ------------------------------------------------------------
        is_ci_nxt <= ci_valid; -- flag to indicate this is a compressed instruction beeing executed (ci_valid is zero if not C-ext is not implemented)
        case i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c) is

          when opcode_alu_c | opcode_alui_c => -- ALU operation
          -- ------------------------------------------------------------
            ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '0'; -- use RS1 as ALU.OPA
            ctrl_nxt(ctrl_alu_opb_mux_lsb_c) <= alu_immediate_v; -- use IMM as ALU.OPB for immediate operations
            ctrl_nxt(ctrl_alu_opc_mux_c)     <= not alu_immediate_v;
            ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_operation_v; -- actual ALU operation
            ctrl_nxt(ctrl_rf_in_mux_msb_c downto ctrl_rf_in_mux_lsb_c) <= "00"; -- RF input = ALU result
            if (CPU_EXTENSION_RISCV_M = true) and (i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c) = opcode_alu_c) and
               (i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000001") then -- MULDIV?
              ctrl_nxt(ctrl_cp_use_c) <= '1'; -- use CP
              ctrl_nxt(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) <= cp_sel_muldiv_c; -- muldiv CP
              state_nxt <= ALU_WAIT;
            elsif (alu_operation_v = alu_cmd_shift_c) then -- multi-cycle shift operation?
              state_nxt <= ALU_WAIT;
            else
              ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back
              state_nxt <= IFETCH_0;
            end if;

          when opcode_lui_c | opcode_auipc_c => -- load upper immediate (add to PC)
          -- ------------------------------------------------------------
            ctrl_nxt(ctrl_rf_clear_rs1_c) <= '1'; -- force RS1 = r0
            if (i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c) = opcode_auipc_c) then -- AUIPC
              ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '1'; -- use PC as ALU.OPA
            else -- LUI
              ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '0'; -- use RS1 as ALU.OPA
            end if;
            ctrl_nxt(ctrl_alu_opb_mux_lsb_c)  <= '1'; -- use IMM as ALU.OPB
            ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_add_c; -- actual ALU operation
            ctrl_nxt(ctrl_rf_in_mux_msb_c downto ctrl_rf_in_mux_lsb_c) <= "00"; -- RF input = ALU result
            ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back
            state_nxt <= IFETCH_0;

          when opcode_load_c | opcode_store_c => -- load/store
          -- ------------------------------------------------------------
            ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '0'; -- use RS1 as ALU.OPA
            ctrl_nxt(ctrl_alu_opb_mux_lsb_c) <= '1'; -- use IMM as ALU.OPB
            ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_add_c; -- actual ALU operation
            ctrl_nxt(ctrl_bus_mar_we_c)  <= '1'; -- write to MAR
            ctrl_nxt(ctrl_bus_mdo_we_c)  <= '1'; -- write to MDO (only relevant for stores)
            if (i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c) = opcode_load_c) then -- LOAD
              state_nxt <= LOAD_0;
            else -- STORE
              state_nxt <= STORE_0;
            end if;

          when opcode_branch_c => -- branch instruction
          -- ------------------------------------------------------------
            ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '1'; -- use PC as ALU.OPA
            ctrl_nxt(ctrl_alu_opb_mux_lsb_c) <= '1'; -- use IMM as ALU.OPB
            ctrl_nxt(ctrl_alu_opc_mux_c) <= '1'; -- use RS2 as ALU.OPC
            is_branch_nxt <= '1';
            state_nxt <= IFETCH_0;

          when opcode_jal_c | opcode_jalr_c => -- jump and link (with register)
          -- ------------------------------------------------------------
            -- compute target address --
            if (i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c) = opcode_jal_c) then -- JAL
              ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '1'; -- use PC as ALU.OPA
            else -- JALR
              ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '0'; -- use RS1 as ALU.OPA
            end if;
            ctrl_nxt(ctrl_alu_opb_mux_lsb_c) <= '1'; -- use IMM as ALU.OPB
            -- save return address --
            ctrl_nxt(ctrl_rf_in_mux_msb_c downto ctrl_rf_in_mux_lsb_c) <= "10"; -- RF input = current PC
            ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back
            -- update PC --
            ctrl_nxt(ctrl_csr_pc_we_c)  <= '1'; -- update PC
            state_nxt <= IFETCH_0;

          when opcode_syscsr_c => -- system/csr access
          -- ------------------------------------------------------------
            ctrl_nxt(ctrl_csr_re_c) <= '1'; -- ALWAYS READ CSR!!! (OLD: not rd_is_r0_v; -- valid CSR read if rd is not r0)
            if (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_env_c) then -- system
              case i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) is
                when x"000" => env_call    <= '1'; state_nxt <= IFETCH_0; -- ECALL
                when x"001" => break_point <= '1'; state_nxt <= IFETCH_0; -- EBREAK
                when x"302" => exc_cpu_end <= '1'; state_nxt <= IFETCH_0; -- MRET
                when x"105" =>                     state_nxt <= SLEEP;    -- WFI
                when others => NULL; -- undefined
              end case;
            elsif (CPU_EXTENSION_RISCV_Zicsr = true) then -- CSR access
              state_nxt <= CSR_ACCESS;
            else
              state_nxt <= IFETCH_0;
            end if;

          when others => -- undefined
          -- ------------------------------------------------------------
            state_nxt <= IFETCH_0;

        end case;

      when ALU_WAIT => -- wait for multi-cycle ALU operation to finish
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_operation_v; -- actual ALU operation
        ctrl_nxt(ctrl_rf_in_mux_msb_c downto ctrl_rf_in_mux_lsb_c) <= "00"; -- RF input = ALU result
        ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back (write back all the time)
        if (alu_wait_i = '0') then
          state_nxt <= IFETCH_0;
        end if;

      when LOAD_0 => -- trigger memory read request
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_bus_rd_c) <= '1'; -- read request
        state_nxt <= LOADSTORE_0;

      when STORE_0 => -- trigger memory write request
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_bus_wr_c) <= '1'; -- write request
        state_nxt <= LOADSTORE_0;

      when LOADSTORE_0 => -- memory latency
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_bus_mdi_we_c) <= '1'; -- write input data to MDI (only relevant for LOAD)
        state_nxt <= LOADSTORE_1;

      when LOADSTORE_1 => -- wait for bus transaction to finish
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_bus_mdi_we_c) <= '1'; -- keep writing input data to MDI (only relevant for LOAD)
        ctrl_nxt(ctrl_rf_in_mux_msb_c downto ctrl_rf_in_mux_lsb_c) <= "01"; -- RF input = memory input (only relevant for LOAD)
        if (ma_load_i = '1') or (be_load_i = '1') or (ma_store_i = '1') or (be_store_i = '1') then -- abort if exception
          state_nxt <= IFETCH_0;
        elsif (bus_wait_i = '0') then -- wait here for bus to finish transaction
          if (i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c) = opcode_load_c) then -- LOAD?
            ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back
          end if;
          state_nxt <= IFETCH_0;
        end if;

      when CSR_ACCESS => -- write CSR data to RF, write ALU.res to CSR
      -- ------------------------------------------------------------
        ctrl_nxt(ctrl_alu_opa_mux_msb_c) <= '0'; -- default
        ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '0'; -- default
        ctrl_nxt(ctrl_alu_opb_mux_msb_c) <= '0'; -- default
        ctrl_nxt(ctrl_alu_opb_mux_lsb_c) <= '0'; -- default
        ctrl_nxt(ctrl_csr_we_c) <= not rs1_is_r0_v; -- valid CSR write if rs1 is not r0 (or not imm5 = 0)
        case i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          when funct3_csrrw_c => -- CSSRW
            ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '0'; -- OPA = rs1
            ctrl_nxt(ctrl_alu_opb_mux_lsb_c) <= '0'; -- OPB = rs2
            ctrl_nxt(ctrl_rf_clear_rs2_c)    <= '1'; -- rs2 = 0
            ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_add_c; -- actual ALU operation = ADD
          when funct3_csrrs_c => -- CSSRS
            ctrl_nxt(ctrl_alu_opa_mux_msb_c) <= '1'; -- OPA = csr
            ctrl_nxt(ctrl_alu_opb_mux_msb_c) <= '1'; -- OPB = crs1
            ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_or_c; -- actual ALU operation = OR
          when funct3_csrrc_c => -- CSSRC
            ctrl_nxt(ctrl_alu_opa_mux_msb_c) <= '1'; -- OPA = csr
            ctrl_nxt(ctrl_alu_opb_mux_msb_c) <= '1'; -- OPB = rs1
            ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_bitc_c; -- actual ALU operation = bit clear
          when funct3_csrrwi_c => -- CSSRWI
            ctrl_nxt(ctrl_alu_opa_mux_lsb_c) <= '0'; -- OPA = rs1
            ctrl_nxt(ctrl_rf_clear_rs1_c)    <= '1'; -- rs1 = 0
            ctrl_nxt(ctrl_alu_opb_mux_lsb_c) <= '1'; -- OPB = immediate
            ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_add_c; -- actual ALU operation = ADD
          when funct3_csrrsi_c => -- CSSRSI
            ctrl_nxt(ctrl_alu_opa_mux_msb_c) <= '1'; -- OPA = csr
            ctrl_nxt(ctrl_alu_opb_mux_lsb_c) <= '1'; -- OPB = immediate
            ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_or_c; -- actual ALU operation = OR
          when funct3_csrrci_c => -- CSSRCI
            ctrl_nxt(ctrl_alu_opa_mux_msb_c) <= '1'; -- OPA = csr
            ctrl_nxt(ctrl_alu_opb_mux_lsb_c) <= '1'; -- OPB = immediate
            ctrl_nxt(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) <= alu_cmd_bitc_c; -- actual ALU operation = bit clear
          when others => -- undefined
            NULL;
        end case;
        -- RF write back --
        ctrl_nxt(ctrl_rf_in_mux_msb_c downto ctrl_rf_in_mux_lsb_c) <= "11"; -- RF input = CSR output register
        ctrl_nxt(ctrl_rf_wb_en_c) <= '1'; -- valid RF write-back
        state_nxt <= IFETCH_0;

      when SLEEP => -- stall and wait for interrupt (WFI)
      -- ------------------------------------------------------------
        if (exc_cpu_start = '1') then -- exception detected!
          exc_cpu_ack <= '1';
          state_nxt   <= IFETCH_0; -- start new instruction fetch
        end if;

      when others => -- undefined
      -- ------------------------------------------------------------
        state_nxt <= IFETCH_0;

    end case;
  end process arbiter_comb;


  -- Illegal Instruction Check --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  illegal_instruction_check: process(i_reg, state, ctrl_nxt)
  begin
    if (state = EXECUTE) then
      -- defaults --
      illegal_instruction <= '0';
      illegal_register    <= '0';
      illegal_compressed  <= '0';

      -- check if using reg >= 16 for E-CPUs --
      if (CPU_EXTENSION_RISCV_E = true) then
        illegal_register <= ctrl_nxt(ctrl_rf_rd_adr4_c) or ctrl_nxt(ctrl_rf_rs2_adr4_c) or ctrl_nxt(ctrl_rf_rs1_adr4_c);
      else
        illegal_register <= '0';
      end if;

      -- check instructions --
      case i_reg(instr_opcode_msb_c downto instr_opcode_lsb_c) is

        -- OPCODE check sufficient: LUI, UIPC, JAL --
        when opcode_lui_c | opcode_auipc_c | opcode_jal_c =>
          illegal_instruction <= '0';

        when opcode_alui_c => -- check ALUI funct7
          if ((i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sll_c) and
              (i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) /= "0000000")) or -- shift logical left
             ((i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sr_c) and
              ((i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) /= "0000000") and
               (i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) /= "0100000"))) then -- shift right
            illegal_instruction <= '1';
          else
            illegal_instruction <= '0';
          end if;
      
        when opcode_load_c => -- check LOAD funct3
          if (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_lb_c) or
             (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_lh_c) or
             (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_lw_c) or
             (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_lbu_c) or
             (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_lhu_c) then
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
          end if;
      
        when opcode_store_c => -- check STORE funct3
          if (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sb_c) or
             (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sh_c) or
             (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sw_c) then
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
          end if;

        when opcode_branch_c => -- check BRANCH funct3
          if (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_beq_c) or
             (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_bne_c) or
             (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_blt_c) or
             (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_bge_c) or
             (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_bltu_c) or
             (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_bgeu_c) then
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
          end if;

        when opcode_jalr_c => -- check JALR funct3
          if (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = "000") then
            illegal_instruction <= '0';
          else
            illegal_instruction <= '1';
          end if;

        when opcode_alu_c => -- check ALU funct3 & funct7
          if (i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000001") then -- MULDIV
            if (CPU_EXTENSION_RISCV_M = false) then
              illegal_instruction <= '1';
            end if;
          elsif ((i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_subadd_c) or
                 (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_sr_c)) and -- ADD/SUB or SRA/SRL check
                ((i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) /= "0000000") and
                 (i_reg(instr_funct7_msb_c downto instr_funct7_lsb_c) /= "0100000")) then -- ADD/SUB or SRA/SRL select
            illegal_instruction <= '1';
          else
            illegal_instruction <= '0';
          end if;

        when opcode_syscsr_c => -- check system instructions --
          -- CSR access --
          if (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrw_c) or
             (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrs_c) or
             (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrc_c) or
             (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrwi_c) or
             (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrsi_c) or
             (i_reg(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csrrci_c) then
            -- valid CSR? --
            if (i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"300") or -- mstatus
               (i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"301") or -- misa
               (i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"304") or -- mie
               (i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"305") or -- mtvev
               (i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"340") or -- mscratch
               (i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"341") or -- mepc
               (i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"342") or -- mcause
               (i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"343") or -- mtval
               (i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"344") or -- mip
               (i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"34a") or -- mtinst
               --
               ((i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"c00") and (CPU_EXTENSION_RISCV_E = false)) or -- cycle
               ((i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"c01") and (CPU_EXTENSION_RISCV_E = false)) or -- time
               ((i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"c02") and (CPU_EXTENSION_RISCV_E = false)) or -- instret
               ((i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"c80") and (CPU_EXTENSION_RISCV_E = false)) or -- cycleh
               ((i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"c81") and (CPU_EXTENSION_RISCV_E = false)) or -- timeh
               ((i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"c82") and (CPU_EXTENSION_RISCV_E = false)) or -- instreth
               --
               ((i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"b00") and (CPU_EXTENSION_RISCV_E = false)) or -- mcycle
               ((i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"b02") and (CPU_EXTENSION_RISCV_E = false)) or -- minstret
               ((i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"b80") and (CPU_EXTENSION_RISCV_E = false)) or -- mcycleh
               ((i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"b82") and (CPU_EXTENSION_RISCV_E = false)) or -- minstreth
               --
               (i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"f13") or -- mimpid
               (i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"f14") or -- mhartid
               (i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"fff") or -- mhwctrl
               --
               (i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"fc0") or -- mfeatures
               (i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"fc1") or -- mclock
               (i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"fc4") or -- mispacebase
               (i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"fc5") or -- mispacesize
               (i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"fc6") or -- mdspacebase
               (i_reg(instr_funct12_msb_c downto instr_funct12_lsb_c) = x"fc7") then -- mdspacesize
              illegal_instruction <= '0';
            else
              illegal_instruction <= '1';
            end if;

          -- ecall, ebreak, mret, wfi --
          elsif (i_reg(instr_rd_msb_c  downto instr_rd_lsb_c)  = "00000") and
                (i_reg(instr_rs1_msb_c downto instr_rs1_lsb_c) = "00000") then
            if (i_reg(instr_funct12_msb_c  downto instr_funct12_lsb_c) = "000000000000") or -- ECALL
               (i_reg(instr_funct12_msb_c  downto instr_funct12_lsb_c) = "000000000001") or -- EBREAK 
               (i_reg(instr_funct12_msb_c  downto instr_funct12_lsb_c) = "001100000010") or -- MRET
               (i_reg(instr_funct12_msb_c  downto instr_funct12_lsb_c) = "000100000101") then -- WFI
              illegal_instruction <= '0';
            else
              illegal_instruction <= '1';
            end if;
          else
            illegal_instruction <= '1';
          end if;

        when others => -- compressed instruction or undefined instruction
          if (i_reg(1 downto 0) = "11") then -- undefined/unimplemented opcode
            illegal_instruction <= '1';
          else -- compressed instruction
            illegal_compressed <= ci_valid and ci_illegal;
          end if;

      end case;
    else
      illegal_instruction <= '0';
      illegal_register    <= '0';
      illegal_compressed  <= '0';
    end if;
  end process illegal_instruction_check;

  -- any illegal condition? --
  illegal_instr_exc <= illegal_instruction or illegal_register or illegal_compressed;


  -- Program Counter ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  program_counter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      if (BOOTLOADER_USE = true) then -- boot from bootloader ROM
        pc_reg         <= boot_base_c;
        pc_backup_reg  <= boot_base_c;
        pc_backup2_reg <= boot_base_c;
      else -- boot from IMEM
        pc_reg         <= MEM_ISPACE_BASE;
        pc_backup_reg  <= MEM_ISPACE_BASE;
        pc_backup2_reg <= MEM_ISPACE_BASE;
      end if;
    elsif rising_edge(clk_i) then
      -- actual PC --
      if (exc_cpu_ack = '1') then -- exception start?
        pc_reg <= mtvec;
      elsif (exc_cpu_end = '1') then -- return from exception
        pc_reg <= mepc;
      elsif (ctrl(ctrl_csr_pc_we_c) = '1') or ((is_branch and branch_taken) = '1') then -- manual update or taken branch
        pc_reg <= alu_add_i;
      end if;
      -- delayed PC --
      if (state = IFETCH_1) then
        pc_backup_reg <= pc_reg; -- PC for ALU address computations
      end if;
      if (state = EXECUTE) then
        pc_backup2_reg <= pc_backup_reg; -- PC backup for exceptions
      end if;
    end if;
  end process program_counter;

  -- output --
  pc_o     <= pc_reg;
  alu_pc_o <= pc_backup_reg;


  -- Exception Controller -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  exception_controller: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      exc_buf       <= (others => '0');
      irq_buf       <= (others => '0');
      exc_ack       <= (others => '0');
      irq_ack       <= (others => '0');
      exc_src       <= (others => '-');
      exc_cpu_start <= '0';
      exc_cause     <= (others => '0');
      mtinst        <= (others => '-');
    elsif rising_edge(clk_i) then
      if (CPU_EXTENSION_RISCV_Zicsr = true) then
        -- exception buffer: misaligned load/store/instruction address
        exc_buf(exception_lalign_c) <= (exc_buf(exception_lalign_c) or ma_load_i)  and (not exc_ack(exception_lalign_c));
        exc_buf(exception_salign_c) <= (exc_buf(exception_salign_c) or ma_store_i) and (not exc_ack(exception_salign_c));
        exc_buf(exception_ialign_c) <= (exc_buf(exception_ialign_c) or ma_instr_i) and (not exc_ack(exception_ialign_c));
        -- exception buffer: load/store/instruction bus access error
        exc_buf(exception_laccess_c) <= (exc_buf(exception_laccess_c) or be_load_i)  and (not exc_ack(exception_laccess_c));
        exc_buf(exception_saccess_c) <= (exc_buf(exception_saccess_c) or be_store_i) and (not exc_ack(exception_saccess_c));
        exc_buf(exception_iaccess_c) <= (exc_buf(exception_iaccess_c) or be_instr_i) and (not exc_ack(exception_iaccess_c));
        -- exception buffer: illegal instruction / env call / break point
        exc_buf(exception_iillegal_c)  <= (exc_buf(exception_iillegal_c)  or illegal_instr_exc) and (not exc_ack(exception_iillegal_c));
        exc_buf(exception_m_envcall_c) <= (exc_buf(exception_m_envcall_c) or env_call)          and (not exc_ack(exception_m_envcall_c));
        exc_buf(exception_break_c)     <= (exc_buf(exception_break_c)     or break_point)       and (not exc_ack(exception_break_c));
        -- interrupt buffer: machine software/external/timer interrupt
        irq_buf(interrupt_msw_irq_c) <= mie_msie and (irq_buf(interrupt_msw_irq_c) or mip_msip) and (not irq_ack(interrupt_msw_irq_c));
        if (IO_CLIC_USE = true) then
          irq_buf(interrupt_mext_irq_c) <= mie_meie and (irq_buf(interrupt_mext_irq_c) or clic_irq_i) and (not irq_ack(interrupt_mext_irq_c));
        else
          irq_buf(interrupt_mext_irq_c) <= '0';
        end if;
        if (IO_MTIME_USE = true) then
          irq_buf(interrupt_mtime_irq_c) <= mie_mtie and (irq_buf(interrupt_mtime_irq_c) or mtime_irq_i) and (not irq_ack(interrupt_mtime_irq_c));
        else
          irq_buf(interrupt_mtime_irq_c) <= '0';
        end if;

        -- exception control --
        if (exc_cpu_start = '0') then -- 
           -- exception/interrupt triggered, waiting for IRQ in EXECUTE (make sure at least 1 instr. is executed even for a continous IRQ)
          if (exc_fire = '1') or ((irq_fire = '1') and ((state = EXECUTE) or (state = SLEEP))) then
            exc_cause     <= exc_cause_nxt; -- capture source for program
            mtinst        <= i_reg; -- MTINST NOT FOULLY IMPLEMENTED YET! FIXME
            mtinst(1)     <= not is_ci; -- bit is set for uncompressed instruction
            exc_src       <= exc_buf; -- capture source for hardware
            exc_ack       <= exc_ack_nxt; -- capture and clear with exception ACK mask
            irq_ack       <= irq_ack_nxt; -- capture and clear with interrupt ACK mask
            exc_cpu_start <= '1';
          end if;
        else -- waiting for exception handler to get started
          if (exc_cpu_ack = '1') then -- handler started?
            exc_ack <= (others => '0');
            irq_ack <= (others => '0');
            exc_cpu_start <= '0';
          end if;
        end if;
      else -- (CPU_EXTENSION_RISCV_Zicsr = false)
        exc_buf       <= (others => '0');
        irq_buf       <= (others => '0');
        exc_ack       <= (others => '0');
        irq_ack       <= (others => '0');
        exc_src       <= (others => '0');
        exc_cpu_start <= '0';
        exc_cause     <= (others => '0');
        mtinst        <= (others => '0');
      end if;
    end if;
  end process exception_controller;

  -- any exception/interrupt? --
  exc_fire <= or_all_f(exc_buf); -- classic exceptions (faults/traps) cannot be masked
  irq_fire <= or_all_f(irq_buf) and mstatus_mie; -- classic interrupts can be enabled/disabled

  -- exception acknowledge for bus unit --
  bus_exc_ack_o <= exc_cpu_ack;

  -- exception priority encoder --
  exc_priority: process(exc_buf, irq_buf)
  begin
    -- defaults --
    exc_cause_nxt <= (others => '0');
    exc_ack_nxt   <= (others => '0');
    irq_ack_nxt   <= (others => '0');

    -- interrupt: 1.11 machine external interrupt --
    if (irq_buf(interrupt_mext_irq_c) = '1') then
      exc_cause_nxt(exc_cause_nxt'left) <= '1';
      exc_cause_nxt(3 downto 0) <= "1011";
      irq_ack_nxt(interrupt_mext_irq_c) <= '1';

    -- interrupt: 1.7 machine timer interrupt --
    elsif (irq_buf(interrupt_mtime_irq_c) = '1') then
      exc_cause_nxt(exc_cause_nxt'left) <= '1';
      exc_cause_nxt(3 downto 0) <= "0111";
      irq_ack_nxt(interrupt_mtime_irq_c) <= '1';

    -- interrupt: 1.3 machine SW interrupt --
    elsif (irq_buf(interrupt_msw_irq_c) = '1') then
      exc_cause_nxt(exc_cause_nxt'left) <= '1';
      exc_cause_nxt(3 downto 0) <= "0011";
      irq_ack_nxt(interrupt_msw_irq_c) <= '1';


    -- trap/fault: 0.0 instruction address misaligned --
    elsif (exc_buf(exception_ialign_c) = '1') then
      exc_cause_nxt(exc_cause_nxt'left) <= '0';
      exc_cause_nxt(3 downto 0) <= "0000";
      exc_ack_nxt(exception_ialign_c)   <= '1';
      exc_ack_nxt(exception_iaccess_c)  <= '1';
      exc_ack_nxt(exception_iillegal_c) <= '1';

    -- trap/fault: 0.1 instruction access fault --
    elsif (exc_buf(exception_iaccess_c) = '1') then
      exc_cause_nxt(exc_cause_nxt'left) <= '0';
      exc_cause_nxt(3 downto 0) <= "0001";
      exc_ack_nxt(exception_ialign_c)   <= '1';
      exc_ack_nxt(exception_iaccess_c)  <= '1';
      exc_ack_nxt(exception_iillegal_c) <= '1';

    -- trap/fault: 0.2 illegal instruction --
    elsif (exc_buf(exception_iillegal_c) = '1') then
      exc_cause_nxt(exc_cause_nxt'left) <= '0';
      exc_cause_nxt(3 downto 0) <= "0010";
      exc_ack_nxt(exception_ialign_c)   <= '1';
      exc_ack_nxt(exception_iaccess_c)  <= '1';
      exc_ack_nxt(exception_iillegal_c) <= '1';


    -- trap/fault: 0.11 environment call from M-mode --
    elsif (exc_buf(exception_m_envcall_c) = '1') then
      exc_cause_nxt(exc_cause_nxt'left) <= '0';
      exc_cause_nxt(3 downto 0) <= "1011";
      exc_ack_nxt(exception_m_envcall_c) <= '1';

    -- trap/fault: 0.3 breakpoint --
    elsif (exc_buf(exception_break_c) = '1') then
      exc_cause_nxt(exc_cause_nxt'left) <= '0';
      exc_cause_nxt(3 downto 0) <= "0011";
      exc_ack_nxt(exception_break_c) <= '1';


    -- trap/fault: 0.6 store address misaligned -
    elsif (exc_buf(exception_salign_c) = '1') then
      exc_cause_nxt(exc_cause_nxt'left) <= '0';
      exc_cause_nxt(3 downto 0) <= "0110";
      exc_ack_nxt(exception_salign_c)  <= '1';
      exc_ack_nxt(exception_lalign_c)  <= '1';
      exc_ack_nxt(exception_saccess_c) <= '1';
      exc_ack_nxt(exception_laccess_c) <= '1';

    -- trap/fault: 0.4 load address misaligned --
    elsif (exc_buf(exception_lalign_c) = '1') then
      exc_cause_nxt(exc_cause_nxt'left) <= '0';
      exc_cause_nxt(3 downto 0) <= "0100";
      exc_ack_nxt(exception_salign_c)  <= '1';
      exc_ack_nxt(exception_lalign_c)  <= '1';
      exc_ack_nxt(exception_saccess_c) <= '1';
      exc_ack_nxt(exception_laccess_c) <= '1';

    -- trap/fault: 0.7 store access fault --
    elsif (exc_buf(exception_saccess_c) = '1') then
      exc_cause_nxt(exc_cause_nxt'left) <= '0';
      exc_cause_nxt(3 downto 0) <= "0111";
      exc_ack_nxt(exception_salign_c)  <= '1';
      exc_ack_nxt(exception_lalign_c)  <= '1';
      exc_ack_nxt(exception_saccess_c) <= '1';
      exc_ack_nxt(exception_laccess_c) <= '1';

    -- trap/fault: 0.5 load access fault --
    elsif (exc_buf(exception_laccess_c) = '1') then
      exc_cause_nxt(exc_cause_nxt'left) <= '0';
      exc_cause_nxt(3 downto 0) <= "0101";
      exc_ack_nxt(exception_salign_c)  <= '1';
      exc_ack_nxt(exception_lalign_c)  <= '1';
      exc_ack_nxt(exception_saccess_c) <= '1';
      exc_ack_nxt(exception_laccess_c) <= '1';

    -- undefined / not implemented --
    else
      exc_cause_nxt <= (others => '0'); -- default
      exc_ack_nxt   <= (others => '0'); -- default
    end if;
  end process exc_priority;


  -- Control and Status Registers Write Access ----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      mstatus_mie  <= '0';
      mstatus_mpie <= '0';
      mie_msie     <= '0';
      mie_meie     <= '0';
      mie_mtie     <= '0';
      mtvec        <= (others => '-');
      mtval        <= (others => '-');
      mepc         <= (others => '-');
      mip_msip     <= '-';
    elsif rising_edge(clk_i) then
      if (CPU_EXTENSION_RISCV_Zicsr = true) then
        mip_msip <= '0';
        -- register that can be modified by user --
        if (ctrl(ctrl_csr_we_c) = '1') then -- manual update
          case i_reg(31 downto 20) is
            -- machine trap setup --
            when x"300" => -- R/W: mstatus - machine status register
              mstatus_mie  <= csr_wdata_i(03);
              mstatus_mpie <= csr_wdata_i(07);
            when x"304" => -- R/W: mie - machine interrupt-enable register
              mie_msie <= csr_wdata_i(03); -- SW IRQ enable
              if (IO_MTIME_USE = true) then
                mie_mtie <= csr_wdata_i(07); -- TIMER IRQ enable
              end if;
              if (IO_CLIC_USE = true) then
                mie_meie <= csr_wdata_i(11); -- EXT IRQ enable
              end if;
            when x"305" => -- R/W: mtvec - machine trap-handler base address (for ALL exceptions)
              mtvec <= csr_wdata_i;
            -- machine trap handling --
            when x"340" => -- R/W: mscratch - machine scratch register
              mscratch <= csr_wdata_i;
            when x"344" => -- R/W: mip - machine interrupt pending
              mip_msip <= csr_wdata_i(03); -- manual SW IRQ trigger
            -- machine trap handling --
            when x"341" => -- R/W: mepc - machine exception program counter
              mepc <= csr_wdata_i;
            -- undefined/unavailable --
            when others =>
              NULL;
          end case;

        else -- automatic update by hardware
          -- machine exception PC & exception value register --
          if (exc_cpu_ack = '1') then -- exception start?
            if (exc_cause(exc_cause_nxt'left) = '1') then -- for INTERRUPTs: mepc = address of next (unclompeted) instruction
              mepc  <= pc_reg;
              mtval <= (others => '-'); -- not specified
            else -- for EXCEPTIONs: mepc = address of next (unclompeted) instruction
              mepc <= pc_backup2_reg;
              if ((exc_src(exception_iaccess_c) or exc_src(exception_ialign_c)) = '1') then -- instruction access error OR misaligned instruction
                mtval <= pc_backup_reg;
              elsif (exc_src(exception_iillegal_c) = '1') then -- illegal instruction
                mtval <= i_reg;
              elsif ((exc_src(exception_lalign_c)  or exc_src(exception_salign_c) or
                      exc_src(exception_laccess_c) or exc_src(exception_saccess_c)) = '1') then -- load/store misaligned / access error
                mtval <= mar_i;
              end if;
            end if;
          end if;

          -- context switch in mstatus --
          if (exc_cpu_ack = '1') then -- actually entering trap
            mstatus_mie <= '0';
            if (mstatus_mpie = '0') then -- FIXME: prevent loosing the prev MIE state after several traps
              mstatus_mpie <= mstatus_mie;
            end if;
          elsif (exc_cpu_end = '1') then -- return from exception
            mstatus_mie <= mstatus_mpie;
          end if;
        end if;

      else -- CPU_EXTENSION_RISCV_Zicsr = false
        mstatus_mie  <= '0';
        mstatus_mpie <= '0';
        mie_msie     <= '0';
        mie_meie     <= '0';
        mie_mtie     <= '0';
        mtvec        <= (others => '0');
        mtval        <= (others => '0');
        mepc         <= (others => '0');
        mip_msip     <= '0';
      end if;
    end if;
  end process csr_write_access;


  -- Control and Status Registers Read Access -----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      csr_rdata_o <= (others => '0'); -- default
      if (CPU_EXTENSION_RISCV_Zicsr = true) then -- implement CSR access at all?
        if (ctrl(ctrl_csr_re_c) = '1') then
          case i_reg(31 downto 20) is
            -- machine trap setup --
            when x"300" => -- R/W: mstatus - machine status register
              csr_rdata_o(03) <= mstatus_mie; -- MIE
              csr_rdata_o(07) <= mstatus_mpie; -- MPIE
              csr_rdata_o(11) <= '1'; -- MPP low
              csr_rdata_o(12) <= '1'; -- MPP high
            when x"301" => -- R/-: misa - ISA and extensions
              csr_rdata_o(02) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_C);     -- C CPU extension
              csr_rdata_o(04) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_E);     -- E CPU extension
              csr_rdata_o(08) <= not bool_to_ulogic_f(CPU_EXTENSION_RISCV_E); -- I CPU extension (if not E)
              csr_rdata_o(12) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_M);     -- M CPU extension
              csr_rdata_o(23) <= '1';                                         -- X CPU extension: non-standard extensions
              csr_rdata_o(25) <= bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicsr); -- Z CPU extension
              csr_rdata_o(30) <= '1'; -- 32-bit architecture (MXL lo)
              csr_rdata_o(31) <= '0'; -- 32-bit architecture (MXL hi)
            when x"304" => -- R/W: mie - machine interrupt-enable register
              csr_rdata_o(03) <= mie_msie; -- software IRQ enable
              csr_rdata_o(07) <= mie_mtie; -- timer IRQ enable
              csr_rdata_o(11) <= mie_meie; -- external IRQ enable
            when x"305" => -- R/W: mtvec - machine trap-handler base address (for ALL exceptions)
              csr_rdata_o <= mtvec;
            -- machine trap handling --
            when x"340" => -- R/W: mscratch - machine scratch register
              csr_rdata_o <= mscratch;
            when x"341" => -- R/W: mepc - machine exception program counter
              csr_rdata_o <= mepc;
            when x"342" => -- R/-: mcause - machine trap cause
              csr_rdata_o <= exc_cause;
            when x"343" => -- R/-: mtval - machine bad address or instruction
              csr_rdata_o <= mtval;
            when x"344" => -- R/W: mip - machine interrupt pending
              csr_rdata_o(03) <= irq_buf(interrupt_msw_irq_c);
              csr_rdata_o(07) <= irq_buf(interrupt_mtime_irq_c);
              csr_rdata_o(11) <= irq_buf(interrupt_mext_irq_c);
            when x"34a" => -- R/-: mtinst - machine trap instruction (transformed)
              csr_rdata_o <= mtinst;
            -- counter and timers --
            when x"c00" | x"c01" | x"b00" => -- R/-: cycle/time/mcycle: Cycle counter LOW / Timer LOW
              csr_rdata_o <= cycle_lo(31 downto 0);
            when x"c02" | x"b02" => -- R/-: instret/minstret: Instructions-retired counter LOW
              csr_rdata_o <= instret_lo(31 downto 0);
            when x"c80" | x"c81" | x"b80" => -- R/-: cycleh/timeh/mcycleh: Cycle counter HIGH / Timer HIGH
              csr_rdata_o(15 downto 0) <= cycle_hi; -- counter is only 16-bit wide!
            when x"c82" | x"b82" => -- R/-: instreth/minstreth: Instructions-retired counter HIGH
              csr_rdata_o(15 downto 0) <= instret_hi; -- counter is only 16-bit wide!
            -- machine information registers --
            when x"f13" => -- R/-: mimpid - implementation ID / version
              csr_rdata_o <= hw_version_c;
            when x"f14" => -- R/-: mhartid - hardware thread ID
              csr_rdata_o <= HART_ID;
            when x"fff" => -- R/-: mhwctrl - hardware controller
              csr_rdata_o <= hw_control;
            -- CUSTOM read-only machine CSRs --
            when x"fc0" => -- R/-: mfeatures - implemented processor devices/features
              csr_rdata_o(00) <= bool_to_ulogic_f(BOOTLOADER_USE);   -- implement processor-internal bootloader?
              csr_rdata_o(01) <= bool_to_ulogic_f(MEM_EXT_USE);      -- implement external memory bus interface?
              csr_rdata_o(02) <= bool_to_ulogic_f(MEM_INT_IMEM_USE); -- implement processor-internal instruction memory
              csr_rdata_o(03) <= bool_to_ulogic_f(MEM_INT_IMEM_ROM); -- implement processor-internal instruction memory as ROM
              csr_rdata_o(04) <= bool_to_ulogic_f(MEM_INT_DMEM_USE); -- implement processor-internal data memory
              --
              csr_rdata_o(16) <= bool_to_ulogic_f(IO_GPIO_USE);  -- implement general purpose input/output port unit (GPIO)?
              csr_rdata_o(17) <= bool_to_ulogic_f(IO_MTIME_USE); -- implement machine system timer (MTIME)?
              csr_rdata_o(18) <= bool_to_ulogic_f(IO_UART_USE);  -- implement universal asynchronous receiver/transmitter (UART)?
              csr_rdata_o(19) <= bool_to_ulogic_f(IO_SPI_USE);   -- implement serial peripheral interface (SPI)?
              csr_rdata_o(20) <= bool_to_ulogic_f(IO_TWI_USE);   -- implement two-wire interface (TWI)?
              csr_rdata_o(21) <= bool_to_ulogic_f(IO_PWM_USE);   -- implement pulse-width modulation unit (PWM)?
              csr_rdata_o(22) <= bool_to_ulogic_f(IO_WDT_USE);   -- implement watch dog timer (WDT)?
              csr_rdata_o(23) <= bool_to_ulogic_f(IO_CLIC_USE);  -- implement core local interrupt controller (CLIC)?
              csr_rdata_o(24) <= bool_to_ulogic_f(IO_TRNG_USE);  -- implement true random number generator (TRNG)?
            when x"fc1" => -- R/-: mclock - processor clock speed
              csr_rdata_o <= std_ulogic_vector(to_unsigned(CLOCK_FREQUENCY, 32));
            when x"fc4" => -- R/-: mispacebase - Base address of instruction memory space
              csr_rdata_o <= MEM_ISPACE_BASE;
            when x"fc5" => -- R/-: mdspacebase - Base address of data memory space
              csr_rdata_o <= MEM_DSPACE_BASE;
            when x"fc6" => -- R/-: mispacesize - Total size of instruction memory space in byte
              csr_rdata_o <= std_ulogic_vector(to_unsigned(MEM_ISPACE_SIZE, 32));
            when x"fc7" => -- R/-: mdspacesize - Total size of data memory space in byte
              csr_rdata_o <= std_ulogic_vector(to_unsigned(MEM_DSPACE_SIZE, 32));
            -- undefined/unavailable --
            when others =>
              csr_rdata_o <= (others => '0'); -- not implemented (yet)
          end case;
        end if;
      end if;
    end if;
  end process csr_read_access;


  -- Optional RISC-V CSRs: Counters ---------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_counters: process(rstn_i, clk_i)
  begin
    if rising_edge(clk_i) then
      if (rstn_i = '0') then
        cycle_lo       <= (others => '0');
        instret_lo     <= (others => '0');
        cycle_hi       <= (others => '0');
        instret_hi     <= (others => '0');
        cycle_lo_msb   <= '0';
        instret_lo_msb <= '0';
      elsif (CPU_EXTENSION_RISCV_E = false) then
        -- low word overflow buffers --
        cycle_lo_msb   <= cycle_lo(cycle_lo'left);
        instret_lo_msb <= instret_lo(instret_lo'left);
        -- low word counters --
        cycle_lo <= std_ulogic_vector(unsigned(cycle_lo) + 1);
        if (state = EXECUTE) then
          instret_lo <= std_ulogic_vector(unsigned(instret_lo) + 1);
        end if;
        -- high word counters --
        if ((cycle_lo_msb xor cycle_lo(cycle_lo'left)) = '1') then
          cycle_hi <= std_ulogic_vector(unsigned(cycle_hi) + 1);
        end if;
        if ((instret_lo_msb xor instret_lo(instret_lo'left)) = '1') then
          instret_hi <= std_ulogic_vector(unsigned(instret_hi) + 1);
        end if;
      else -- counters are not available in embedded mode
        cycle_lo       <= (others => '0');
        instret_lo     <= (others => '0');
        cycle_hi       <= (others => '0');
        instret_hi     <= (others => '0');
        cycle_lo_msb   <= '0';
        instret_lo_msb <= '0';
      end if;
    end if;
  end process csr_counters;


end neorv32_cpu_control_rtl;
