-- ================================================================================ --
-- NEORV32 CPU - Compressed Instructions Decoder (RISC-V 'C' ISA Extensions)        --
-- -------------------------------------------------------------------------------- --
-- Only the non floating-point 'Zca' ISA subset is supported by default.            --
-- The optional 'Zcb' sub-extension can emit 32-bit instructions that depend        --
-- on the 'M'/'Zmmul' and 'B'/'Zbb' ISA extensions.                                 --
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

entity neorv32_cpu_zcmp is
  port (
    clk_i : in std_logic;
    rstn_i : in std_logic;
    zcmp_detect : in std_logic;
    ctrl_i : in ctrl_bus_t;
    fetch_restart : in std_logic;  
    zcmp_instr_reg: in std_ulogic_vector(15 downto 0);
    ipb_avail : in std_ulogic_vector(1 downto 0);
    zcmp_is_push : in std_ulogic;
    zcmp_is_popret : in std_ulogic; -- instruction is popret
    zcmp_is_popretz : in std_ulogic; -- instruction is popretz
    zcmp_is_mvsa01 : in std_ulogic;
    zcmp_is_mva01s : in std_ulogic;
    frontend_bus_zcmp : out if_bus_t;
    zcmp_in_uop_seq : out std_logic
  );
end neorv32_cpu_zcmp;

architecture arch of neorv32_cpu_zcmp is

  type uop_state_type is (S_IDLE, S_ZCMP_UOP_SEQ, S_POPRET, S_POPRETZ, S_ZCMP_DOUBLE_MOVE_1, S_ZCMP_DOUBLE_MOVE_2, S_ZCMP_ABORT);
  signal uop_state_reg, uop_state_nxt : uop_state_type;
  
  signal uop_ctr, uop_ctr_next, uop_ctr_nxt_in_seq : integer range 0 to 15;

  signal zcmp_stack_sw_offset, zcmp_stack_lw_offset : signed(11 downto 0);
  signal zcmp_reg_list : std_ulogic_vector(3 downto 0);
  signal zcmp_ls_reg : std_ulogic_vector(4 downto 0);
  signal zcmp_num_regs : integer range 0 to 15;
  signal zcmp_stack_adj_base : integer range 0 to 127;
  signal zcmp_stack_adj : integer range 0 to 255;
  -- signal zcmp_in_uop_seq : std_ulogic; -- TODO needed?

  signal zcmp_instr, zcmp_sw_instr, zcmp_lw_instr, zcmp_jalr_instr : std_ulogic_vector(31 downto 0);
  constant zcmp_sw_instr_opcode : std_ulogic_vector(6 downto 0) := "0100011";
  constant zcmp_lw_instr_opcode : std_ulogic_vector(6 downto 0) := "0000011";
  constant zcmp_jalr_instr_opcode : std_ulogic_vector(6 downto 0) := "1100111";

  constant zcmp_instr_funct3 : std_ulogic_vector(2 downto 0) := "010";
  constant zcmp_instr_rs1_sp : std_ulogic_vector(4 downto 0) := "00010"; -- stack pointer 
  constant zcmp_instr_rs1_ra : std_ulogic_vector(4 downto 0) := "00001"; -- return address 

  signal zcmp_stack_adj_instr, zcmp_push_stack_adj_instr, zcmp_pop_stack_adj_instr, zcmp_li_a0_instr : std_ulogic_vector(31 downto 0);
  constant zcmp_addi_instr_opcode : std_ulogic_vector(6 downto 0) := "0010011";
  constant zcmp_addi_instr_funct3 : std_ulogic_vector(2 downto 0) := "000";
  constant zcmp_addi_rs1_sp : std_ulogic_vector(4 downto 0) := "00010"; -- stack pointer 

  constant zcmp_zero_a0_instr : std_ulogic_vector(31 downto 0) := x"00000513"; -- addi x10, x0, 0 == li a0, 0

  signal zcmp_sa01_r1s, zcmp_sa01_r2s : std_ulogic_vector(4 downto 0);

begin


  zcmp_reg_list <= zcmp_instr_reg(7 downto 4);

  -- number of registers which are pushed/popped from/to the stack
  zcmp_num_regs <= 13 when to_integer(unsigned(zcmp_reg_list)) = 15 else
                   0 when to_integer(unsigned(zcmp_reg_list)) < 4 else
                   to_integer(unsigned(zcmp_reg_list)) - 3;

  -- minimum value by which the stack pointer has to be adjusted
  zcmp_stack_adj_base <= 64 when to_integer(unsigned(zcmp_reg_list)) = 15 else
                         48 when to_integer(unsigned(zcmp_reg_list)) >= 12 else
                         32 when to_integer(unsigned(zcmp_reg_list)) >= 8 else
                         16;

  -- total stack pointer adjustmend value = base + immediate
  zcmp_stack_adj <= zcmp_stack_adj_base + ((to_integer(unsigned(zcmp_instr_reg(3 downto 2))) * 16));

  -- the stack pointer update is at the end of a push/pop sequence. Therefore we need to use offset values based on the current uop counter value when accessing the stack
  zcmp_stack_sw_offset <= to_signed(-((zcmp_num_regs - uop_ctr) * 4), zcmp_stack_sw_offset'length);
  zcmp_stack_lw_offset <= to_signed(-((zcmp_num_regs - uop_ctr) * 4) + zcmp_stack_adj, zcmp_stack_lw_offset'length);

  -- register selection for push/pop uop sequence
  zcmp_ls_reg <= "00001" when uop_ctr = 0 else -- ra
                 "01000" when uop_ctr = 1 else -- s0
                 "01001" when uop_ctr = 2 else -- s1
                 std_ulogic_vector(to_unsigned(uop_ctr + 15, zcmp_ls_reg'length)); -- s2-s11 (s2 == x18)

  -- the instruction opcodes for the stores and loads to/from stack during a push/pop sequence
  zcmp_sw_instr <= std_ulogic_vector(zcmp_stack_sw_offset(11 downto 5)) & zcmp_ls_reg & zcmp_instr_rs1_sp & zcmp_instr_funct3 & std_ulogic_vector(zcmp_stack_sw_offset(4 downto 0)) & zcmp_sw_instr_opcode;
  zcmp_lw_instr <= std_ulogic_vector(zcmp_stack_lw_offset) & zcmp_instr_rs1_sp & zcmp_instr_funct3 & zcmp_ls_reg & zcmp_lw_instr_opcode;

  -- the 32-bit addi instruction that adjusts the stack pointer after a push sequence. The immediate value is generated with the zcmp_stack_adj signal
  zcmp_push_stack_adj_instr <= std_ulogic_vector(-to_signed(zcmp_stack_adj, 12)) &
                               zcmp_addi_rs1_sp & -- rs1 = sp 
                               zcmp_addi_instr_funct3 &
                               zcmp_addi_rs1_sp & -- rd = rs1 = sp 
                               zcmp_addi_instr_opcode; -- addi 

  -- the 32-bit addi instruction that adjusts the stack pointer after a pop sequence. The immediate value is generated with the zcmp_stack_adj signal
  zcmp_pop_stack_adj_instr <= std_ulogic_vector(to_signed(zcmp_stack_adj, 12)) &
                              zcmp_addi_rs1_sp & -- rs1 = sp 
                              zcmp_addi_instr_funct3 &
                              zcmp_addi_rs1_sp & -- rd = rs1 = sp 
                              zcmp_addi_instr_opcode; -- addi 

  -- use either the addi instruction with negative offset (push) or positive (pop)
  zcmp_stack_adj_instr <= zcmp_push_stack_adj_instr when zcmp_is_push = '1' else
                          zcmp_pop_stack_adj_instr;

  -- stack instruction is either a load or store depending on push/pop sequence
  zcmp_instr <= zcmp_sw_instr when zcmp_is_push = '1' else
                zcmp_lw_instr;

  -- li a0, 0 opcode                              
  zcmp_li_a0_instr <= "00000000000000000000" & "01010" & zcmp_addi_instr_opcode;
  zcmp_jalr_instr <= "000000000000" & zcmp_instr_rs1_ra & "00000000" & zcmp_jalr_instr_opcode;

  uop_ctr_next <= uop_ctr_nxt_in_seq;

  -- cm.mv* instructions use sreg number specifiers, these two signals map them to xreg specifiers
  zcmp_sa01_r1s <= (zcmp_instr_reg(9) or zcmp_instr_reg(8)) &
                   (not (zcmp_instr_reg(9) or zcmp_instr_reg(8))) &
                   zcmp_instr_reg(9 downto 7);

  zcmp_sa01_r2s <= (zcmp_instr_reg(4) or zcmp_instr_reg(3)) &
                   (not (zcmp_instr_reg(4) or zcmp_instr_reg(3))) &
                   zcmp_instr_reg(4 downto 2);

  uop_fsm_sync : process (rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      uop_ctr <= 0;
      uop_state_reg <= S_IDLE;
    elsif rising_edge(clk_i) then
      uop_ctr <= uop_ctr_next;
      uop_state_reg <= uop_state_nxt;
    end if;
  end process uop_fsm_sync;

  uop_fsm_comb : process (uop_state_reg, zcmp_jalr_instr, zcmp_sa01_r1s, zcmp_sa01_r2s, zcmp_is_mvsa01, zcmp_is_mva01s, uop_ctr, fetch_restart, ipb_avail, zcmp_in_uop_seq, zcmp_is_popret, zcmp_is_popretz, ctrl_i, zcmp_detect, zcmp_num_regs, zcmp_instr, zcmp_stack_adj_instr)
  begin

    -- defaults
    uop_ctr_nxt_in_seq <= uop_ctr;
    uop_state_nxt <= uop_state_reg;
    zcmp_in_uop_seq <= '0';
    frontend_bus_zcmp.valid <= '0';
    frontend_bus_zcmp.compr <= '1';
    frontend_bus_zcmp.fault <= '0';
    frontend_bus_zcmp.instr <= (others => '0');
    frontend_bus_zcmp.zcmp_in_uop_seq <= zcmp_in_uop_seq;
    frontend_bus_zcmp.zcmp_atomic_tail <= '0';
    frontend_bus_zcmp.zcmp_start <= '0';

    case uop_state_reg is
      when S_IDLE =>
        if (zcmp_detect = '1') then -- 
          if (zcmp_is_mvsa01 = '1' or zcmp_is_mva01s = '1') then
            uop_state_nxt <= S_ZCMP_DOUBLE_MOVE_1;
          else
            uop_state_nxt <= S_ZCMP_UOP_SEQ;
          end if;
        end if;

      when S_ZCMP_UOP_SEQ =>
        zcmp_in_uop_seq <= '1';
        if (uop_ctr = 15) then -- last instruction
          frontend_bus_zcmp.instr <= zcmp_stack_adj_instr; -- issue stack pointer adjustment instruction
          frontend_bus_zcmp.valid <= '1';

          if (ctrl_i.if_ready = '1') then -- only advance if the uop has been acknowledged by control unit 
            uop_ctr_nxt_in_seq <= 0;

            if (zcmp_is_popret = '1') then
              frontend_bus_zcmp.zcmp_atomic_tail <= '1';
              uop_state_nxt <= S_POPRET; -- issue return instruction
            elsif (zcmp_is_popretz = '1') then
              frontend_bus_zcmp.zcmp_atomic_tail <= '1';
              uop_state_nxt <= S_POPRETZ; -- zero a0 before returning  
            else
              uop_state_nxt <= S_IDLE; -- cm.push is finished after stack adjustment
            end if;
          end if;

        else

          if (ctrl_i.if_ready = '1') then -- advance to next uop if control unit has acknowledged the current one
            uop_ctr_nxt_in_seq <= uop_ctr + 1;
          end if;

          frontend_bus_zcmp.instr <= zcmp_instr;
          frontend_bus_zcmp.valid <= '1';

          if (uop_ctr + 1 = zcmp_num_regs and ctrl_i.if_ready = '1') then -- is next register the last one?
            uop_ctr_nxt_in_seq <= 15;
          end if;

        end if;

        -- during the issuing of load or store instructions of a push/pop sequence, the sequence can be aborted if a trap occurs
        -- because cm.push is often the first instruction after a branch instruction, this fsm begins to run before the frontend receives the restart signal. We abort on fetch.restart  
        if (fetch_restart = '1' or ctrl_i.cpu_trap = '1') then
          uop_state_nxt <= S_ZCMP_ABORT;
          zcmp_in_uop_seq <= '0';
          uop_ctr_nxt_in_seq <= 0;
          frontend_bus_zcmp.valid <= '0';
          frontend_bus_zcmp.instr <= (others => '0');
        end if;

      when S_POPRET =>
        frontend_bus_zcmp.zcmp_atomic_tail <= '1'; -- no pending traps will be handled
        zcmp_in_uop_seq <= '1';
        frontend_bus_zcmp.instr <= zcmp_jalr_instr;
        frontend_bus_zcmp.valid <= '1';

        if (ctrl_i.if_ready = '1') then
          uop_state_nxt <= S_IDLE;
        end if;

      when S_POPRETZ =>
        frontend_bus_zcmp.zcmp_atomic_tail <= '1';
        zcmp_in_uop_seq <= '1';
        frontend_bus_zcmp.instr <= zcmp_zero_a0_instr; --zero a0
        frontend_bus_zcmp.valid <= '1';

        if (ctrl_i.if_ready = '1') then
          uop_state_nxt <= S_POPRET; -- issue ret instruction
        end if;

        -- double move instruction, first uop
      when S_ZCMP_DOUBLE_MOVE_1 =>
        frontend_bus_zcmp.zcmp_atomic_tail <= '1';
        zcmp_in_uop_seq <= '1';

        -- abort on previous branch
        if (fetch_restart = '1') then
          uop_state_nxt <= S_ZCMP_ABORT;
          frontend_bus_zcmp.zcmp_atomic_tail <= '0';
          zcmp_in_uop_seq <= '0';
          uop_ctr_nxt_in_seq <= 0;
          frontend_bus_zcmp.valid <= '0';
          frontend_bus_zcmp.instr <= (others => '0');
        end if;

        if (zcmp_is_mva01s = '1') then
          frontend_bus_zcmp.instr <= x"000" & zcmp_sa01_r1s & zcmp_addi_instr_funct3 & "01010" & zcmp_addi_instr_opcode;
        else
          frontend_bus_zcmp.instr <= x"000" & "01010" & zcmp_addi_instr_funct3 & zcmp_sa01_r1s & zcmp_addi_instr_opcode;
        end if;

        frontend_bus_zcmp.valid <= '1';

        if (ctrl_i.if_ready = '1') then
          uop_state_nxt <= S_ZCMP_DOUBLE_MOVE_2;
        end if;

        -- double move instruction, second uop
      when S_ZCMP_DOUBLE_MOVE_2 =>
        frontend_bus_zcmp.zcmp_atomic_tail <= '1';
        zcmp_in_uop_seq <= '1';

        -- abort on previous branch
        if (fetch_restart = '1') then
          uop_state_nxt <= S_ZCMP_ABORT;
          frontend_bus_zcmp.zcmp_atomic_tail <= '0';
          zcmp_in_uop_seq <= '0';
          uop_ctr_nxt_in_seq <= 0;
          frontend_bus_zcmp.valid <= '0';
          frontend_bus_zcmp.instr <= (others => '0');
        end if;

        if (zcmp_is_mva01s = '1') then
          frontend_bus_zcmp.instr <= x"000" & zcmp_sa01_r2s & zcmp_addi_instr_funct3 & "01011" & zcmp_addi_instr_opcode;
        else
          frontend_bus_zcmp.instr <= x"000" & "01011" & zcmp_addi_instr_funct3 & zcmp_sa01_r2s & zcmp_addi_instr_opcode;
        end if;

        frontend_bus_zcmp.valid <= '1';

        if (ctrl_i.if_ready = '1') then
          uop_state_nxt <= S_IDLE;
        end if;

      when S_ZCMP_ABORT =>
        if (ipb_avail /= "00") then
          uop_state_nxt <= S_IDLE;

          if (zcmp_detect = '1') then -- it is possible that the next instruction in one of the ipbs is from the zcmp extension 
            if (zcmp_is_mvsa01 = '1' or zcmp_is_mva01s = '1') then
              uop_state_nxt <= S_ZCMP_DOUBLE_MOVE_1;
            else
              uop_state_nxt <= S_ZCMP_UOP_SEQ;
            end if;
          end if;
        end if;

    end case;
  end process;

end arch;