-- #################################################################################################
-- # << NEORV32 - Arithmetical/Logical Unit >>                                                     #
-- # ********************************************************************************************* #
-- # Main data/address ALU and ALU co-processor (= multi-cycle function units).                    #
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

entity neorv32_cpu_alu is
  generic (
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_B     : boolean; -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_M     : boolean; -- implement mul/div extension?
    CPU_EXTENSION_RISCV_Zmmul : boolean; -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zfinx : boolean; -- implement 32-bit floating-point extension (using INT reg!)
    CPU_EXTENSION_RISCV_Zxcfu : boolean; -- implement custom (instr.) functions unit?
    -- Extension Options --
    FAST_MUL_EN               : boolean; -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN             : boolean  -- use barrel shifter for shift operations
  );
  port (
    -- global control --
    clk_i       : in  std_ulogic; -- global clock, rising edge
    rstn_i      : in  std_ulogic; -- global reset, low-active, async
    ctrl_i      : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
    -- data input --
    rs1_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 1
    rs2_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 2
    pc_i        : in  std_ulogic_vector(data_width_c-1 downto 0); -- current PC
    imm_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- immediate
    -- data output --
    cmp_o       : out std_ulogic_vector(1 downto 0); -- comparator status
    res_o       : out std_ulogic_vector(data_width_c-1 downto 0); -- ALU result
    add_o       : out std_ulogic_vector(data_width_c-1 downto 0); -- address computation result
    fpu_flags_o : out std_ulogic_vector(4 downto 0); -- FPU exception flags
    -- status --
    idone_o     : out std_ulogic -- iterative processing units done?
  );
end neorv32_cpu_alu;

architecture neorv32_cpu_cpu_rtl of neorv32_cpu_alu is

  -- comparator --
  signal cmp_opx : std_ulogic_vector(data_width_c downto 0);
  signal cmp_opy : std_ulogic_vector(data_width_c downto 0);
  signal cmp     : std_ulogic_vector(1 downto 0); -- comparator status

  -- operands --
  signal opa, opb : std_ulogic_vector(data_width_c-1 downto 0);

  -- results --
  signal addsub_res : std_ulogic_vector(data_width_c downto 0);
  signal cp_res     : std_ulogic_vector(data_width_c-1 downto 0);

  -- co-processor interface --
  type cp_data_if_t  is array (0 to 7)  of std_ulogic_vector(data_width_c-1 downto 0);
  signal cp_result : cp_data_if_t; -- co-processor i result
  signal cp_start  : std_ulogic_vector(7 downto 0); -- trigger co-processor i
  signal cp_valid  : std_ulogic_vector(7 downto 0); -- co-processor i done

begin

  -- Comparator Unit (for conditional branches) ---------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cmp_opx <= (rs1_i(rs1_i'left) and (not ctrl_i(ctrl_alu_unsigned_c))) & rs1_i;
  cmp_opy <= (rs2_i(rs2_i'left) and (not ctrl_i(ctrl_alu_unsigned_c))) & rs2_i;

  cmp(cmp_equal_c) <= '1' when (rs1_i = rs2_i) else '0';
  cmp(cmp_less_c)  <= '1' when (signed(cmp_opx) < signed(cmp_opy)) else '0';
  cmp_o            <= cmp;


  -- ALU Input Operand Mux ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  opa <= pc_i  when (ctrl_i(ctrl_alu_opa_mux_c) = '1') else rs1_i; -- operand a (first ALU input operand), only required for arithmetic ops
  opb <= imm_i when (ctrl_i(ctrl_alu_opb_mux_c) = '1') else rs2_i; -- operand b (second ALU input operand)


  -- Binary Adder/Subtracter ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  binary_arithmetic_core: process(ctrl_i, opa, opb)
    variable cin_v  : std_ulogic_vector(0 downto 0);
    variable op_a_v : std_ulogic_vector(data_width_c downto 0);
    variable op_b_v : std_ulogic_vector(data_width_c downto 0);
    variable op_y_v : std_ulogic_vector(data_width_c downto 0);
    variable res_v  : std_ulogic_vector(data_width_c downto 0);
  begin
    -- operand sign-extension --
    op_a_v := (opa(opa'left) and (not ctrl_i(ctrl_alu_unsigned_c))) & opa;
    op_b_v := (opb(opb'left) and (not ctrl_i(ctrl_alu_unsigned_c))) & opb;
    -- add/sub(slt) select --
    if (ctrl_i(ctrl_alu_op0_c) = '1') then -- subtraction
      op_y_v   := not op_b_v;
      cin_v(0) := '1';
    else -- addition
      op_y_v   := op_b_v;
      cin_v(0) := '0';
    end if;
    -- adder core --
    addsub_res <= std_ulogic_vector(unsigned(op_a_v) + unsigned(op_y_v) + unsigned(cin_v(0 downto 0)));
  end process binary_arithmetic_core;

  -- direct output of adder result --
  add_o <= addsub_res(data_width_c-1 downto 0);


  -- ALU Operation Select -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  alu_core: process(ctrl_i, addsub_res, cp_res, rs1_i, opb)
  begin
    case ctrl_i(ctrl_alu_op2_c downto ctrl_alu_op0_c) is
      when alu_op_add_c  => res_o <= addsub_res(data_width_c-1 downto 0); -- default
      when alu_op_sub_c  => res_o <= addsub_res(data_width_c-1 downto 0);
      when alu_op_cp_c   => res_o <= cp_res;
      when alu_op_slt_c  => res_o <= (others => '0'); res_o(0) <= addsub_res(addsub_res'left); -- carry/borrow
      when alu_op_movb_c => res_o <= opb;
      when alu_op_xor_c  => res_o <= rs1_i xor opb; -- only rs1 required for logic ops (opa would also contain pc)
      when alu_op_or_c   => res_o <= rs1_i or  opb;
      when alu_op_and_c  => res_o <= rs1_i and opb;
      when others        => res_o <= addsub_res(data_width_c-1 downto 0);
    end case;
  end process alu_core;


  -- **************************************************************************************************************************
  -- ALU Co-Processors
  -- **************************************************************************************************************************

  -- co-processor select / start trigger --
  -- > "cp_start" is high for one cycle to trigger operation of the according co-processor
  cp_start(7 downto 0) <= ctrl_i(ctrl_cp_trig7_c downto ctrl_cp_trig0_c);

  -- co-processor operation done? --
  -- > "cp_valid" signal has to be set (for one cycle) one cycle before output data (cp_result) is valid
  idone_o <= cp_valid(0) or cp_valid(1) or cp_valid(2) or cp_valid(3) or
             cp_valid(4) or cp_valid(5) or cp_valid(6) or cp_valid(7);

  -- co-processor result --
  -- > "cp_result" data has to be always zero unless co-processor was actually triggered
  cp_res <= cp_result(0) or cp_result(1) or cp_result(2) or cp_result(3) or
            cp_result(4) or cp_result(5) or cp_result(6) or cp_result(7);


  -- Co-Processor 0: Shifter Unit (CPU Base ISA) --------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_cp_shifter_inst: neorv32_cpu_cp_shifter
  generic map (
    FAST_SHIFT_EN => FAST_SHIFT_EN -- use barrel shifter for shift operations
  )
  port map (
    -- global control --
    clk_i   => clk_i,        -- global clock, rising edge
    rstn_i  => rstn_i,       -- global reset, low-active, async
    ctrl_i  => ctrl_i,       -- main control bus
    start_i => cp_start(0),  -- trigger operation
    -- data input --
    rs1_i   => rs1_i,        -- rf source 1
    shamt_i => opb(index_size_f(data_width_c)-1 downto 0), -- shift amount
    -- result and status --
    res_o   => cp_result(0), -- operation result
    valid_o => cp_valid(0)   -- data output valid
  );


  -- Co-Processor 1: Integer Multiplication/Division Unit ('M' Extension) -------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_cp_muldiv_inst_true:
  if (CPU_EXTENSION_RISCV_M = true) or (CPU_EXTENSION_RISCV_Zmmul = true) generate
    neorv32_cpu_cp_muldiv_inst: neorv32_cpu_cp_muldiv
    generic map (
      FAST_MUL_EN => FAST_MUL_EN,          -- use DSPs for faster multiplication
      DIVISION_EN => CPU_EXTENSION_RISCV_M -- implement divider hardware
    )
    port map (
      -- global control --
      clk_i   => clk_i,        -- global clock, rising edge
      rstn_i  => rstn_i,       -- global reset, low-active, async
      ctrl_i  => ctrl_i,       -- main control bus
      start_i => cp_start(1),  -- trigger operation
      -- data input --
      rs1_i   => rs1_i,        -- rf source 1
      rs2_i   => rs2_i,        -- rf source 2
      -- result and status --
      res_o   => cp_result(1), -- operation result
      valid_o => cp_valid(1)   -- data output valid
    );
  end generate;

  neorv32_cpu_cp_muldiv_inst_false:
  if (CPU_EXTENSION_RISCV_M = false) and (CPU_EXTENSION_RISCV_Zmmul = false) generate
    cp_result(1) <= (others => '0');
    cp_valid(1)  <= '0';
  end generate;


  -- Co-Processor 2: Bit-Manipulation Unit ('B' Extension) ----------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_cp_bitmanip_inst_true:
  if (CPU_EXTENSION_RISCV_B = true) generate
    neorv32_cpu_cp_bitmanip_inst: neorv32_cpu_cp_bitmanip
    generic map (
      FAST_SHIFT_EN => FAST_SHIFT_EN -- use barrel shifter for shift operations
    )
    port map (
      -- global control --
      clk_i   => clk_i,        -- global clock, rising edge
      rstn_i  => rstn_i,       -- global reset, low-active, async
      ctrl_i  => ctrl_i,       -- main control bus
      start_i => cp_start(2),  -- trigger operation
      -- data input --
      cmp_i   => cmp,          -- comparator status
      rs1_i   => rs1_i,        -- rf source 1
      rs2_i   => rs2_i,        -- rf source 2
      shamt_i => opb(index_size_f(data_width_c)-1 downto 0), -- shift amount
      -- result and status --
      res_o   => cp_result(2), -- operation result
      valid_o => cp_valid(2)   -- data output valid
    );
  end generate;

  neorv32_cpu_cp_bitmanip_inst_false:
  if (CPU_EXTENSION_RISCV_B = false) generate
    cp_result(2) <= (others => '0');
    cp_valid(2)  <= '0';
  end generate;


  -- Co-Processor 3: Single-Precision Floating-Point Unit ('Zfinx' Extension) ---------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_cp_fpu_inst_true:
  if (CPU_EXTENSION_RISCV_Zfinx = true) generate
    neorv32_cpu_cp_fpu_inst: neorv32_cpu_cp_fpu
    port map (
      -- global control --
      clk_i    => clk_i,        -- global clock, rising edge  
      rstn_i   => rstn_i,       -- global reset, low-active, async
      ctrl_i   => ctrl_i,       -- main control bus
      start_i  => cp_start(3),  -- trigger operation
      -- data input --
      cmp_i    => cmp,          -- comparator status
      rs1_i    => rs1_i,        -- rf source 1
      rs2_i    => rs2_i,        -- rf source 2
      -- result and status --
      res_o    => cp_result(3), -- operation result
      fflags_o => fpu_flags_o,  -- exception flags
      valid_o  => cp_valid(3)   -- data output valid
    );
  end generate;

  neorv32_cpu_cp_fpu_inst_false:
  if (CPU_EXTENSION_RISCV_Zfinx = false) generate
    cp_result(3) <= (others => '0');
    fpu_flags_o  <= (others => '0');
    cp_valid(3)  <= '0';
  end generate;


  -- Co-Processor 4: Custom (Instructions) Functions Unit ('Zxcfu' Extension) ---------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_cp_cfu_inst_true:
  if (CPU_EXTENSION_RISCV_Zxcfu = true) generate
    neorv32_cpu_cp_cfu_inst: neorv32_cpu_cp_cfu
    port map (
      -- global control --
      clk_i   => clk_i,        -- global clock, rising edge
      rstn_i  => rstn_i,       -- global reset, low-active, async
      ctrl_i  => ctrl_i,       -- main control bus
      start_i => cp_start(4),  -- trigger operation
      -- data input --
      rs1_i   => rs1_i,        -- rf source 1
      rs2_i   => rs2_i,        -- rf source 2
      -- result and status --
      res_o   => cp_result(4), -- operation result
      valid_o => cp_valid(4)   -- data output valid
    );
  end generate;

  neorv32_cpu_cp_cfu_inst_false:
  if (CPU_EXTENSION_RISCV_Zxcfu = false) generate
    cp_result(4) <= (others => '0');
    cp_valid(4)  <= '0';
  end generate;


  -- Co-Processor 5: Reserved ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cp_result(5) <= (others => '0');
  cp_valid(5)  <= '0';


  -- Co-Processor 6: Reserved ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cp_result(6) <= (others => '0');
  cp_valid(6)  <= '0';


  -- Co-Processor 7: Reserved ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cp_result(7) <= (others => '0');
  cp_valid(7)  <= '0';


end neorv32_cpu_cpu_rtl;
