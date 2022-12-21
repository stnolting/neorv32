-- #################################################################################################
-- # << NEORV32 - Arithmetical/Logical Unit >>                                                     #
-- # ********************************************************************************************* #
-- # Main data/address ALU and ALU co-processors (= multi-cycle function units).                   #
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
    XLEN                      : natural; -- data path width
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
    rs1_i       : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 1
    rs2_i       : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 2
    rs3_i       : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 3
    rs4_i       : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 4
    pc_i        : in  std_ulogic_vector(XLEN-1 downto 0); -- current PC
    imm_i       : in  std_ulogic_vector(XLEN-1 downto 0); -- immediate
    -- data output --
    cmp_o       : out std_ulogic_vector(1 downto 0); -- comparator status
    res_o       : out std_ulogic_vector(XLEN-1 downto 0); -- ALU result
    add_o       : out std_ulogic_vector(XLEN-1 downto 0); -- address computation result
    fpu_flags_o : out std_ulogic_vector(4 downto 0); -- FPU exception flags
    -- status --
    idone_o     : out std_ulogic -- iterative processing units done?
  );
end neorv32_cpu_alu;

architecture neorv32_cpu_cpu_rtl of neorv32_cpu_alu is

  -- comparator --
  signal cmp_rs1 : std_ulogic_vector(XLEN downto 0);
  signal cmp_rs2 : std_ulogic_vector(XLEN downto 0);
  signal cmp     : std_ulogic_vector(1 downto 0); -- comparator status

  -- operands --
  signal opa, opb : std_ulogic_vector(XLEN-1 downto 0);

  -- intermediate results --
  signal addsub_res : std_ulogic_vector(XLEN downto 0);
  signal cp_res     : std_ulogic_vector(XLEN-1 downto 0);

  -- co-processor interface --
  type cp_data_if_t  is array (0 to 4) of std_ulogic_vector(XLEN-1 downto 0);
  signal cp_result : cp_data_if_t; -- co-processor result
  signal cp_start  : std_ulogic_vector(4 downto 0); -- trigger co-processor
  signal cp_valid  : std_ulogic_vector(4 downto 0); -- co-processor done

begin

  -- Comparator Unit (for conditional branches) ---------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cmp_rs1 <= (rs1_i(rs1_i'left) and (not ctrl_i(ctrl_alu_unsigned_c))) & rs1_i; -- optional sign-extension
  cmp_rs2 <= (rs2_i(rs2_i'left) and (not ctrl_i(ctrl_alu_unsigned_c))) & rs2_i; -- optional sign-extension

  cmp(cmp_equal_c) <= '1' when (rs1_i = rs2_i) else '0';
  cmp(cmp_less_c)  <= '1' when (signed(cmp_rs1) < signed(cmp_rs2)) else '0'; -- signed or unsigned comparison
  cmp_o            <= cmp;


  -- ALU Input Operand Select ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  opa <= pc_i  when (ctrl_i(ctrl_alu_opa_mux_c) = '1') else rs1_i;
  opb <= imm_i when (ctrl_i(ctrl_alu_opb_mux_c) = '1') else rs2_i;


  -- Adder/Subtracter Core ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  arithmetic_core: process(ctrl_i, opa, opb)
    variable opa_v, opb_v : std_ulogic_vector(XLEN downto 0);
  begin
    -- operand sign-extension --
    opa_v := (opa(opa'left) and (not ctrl_i(ctrl_alu_unsigned_c))) & opa;
    opb_v := (opb(opb'left) and (not ctrl_i(ctrl_alu_unsigned_c))) & opb;
    -- add/sub(slt) select --
    if (ctrl_i(ctrl_alu_op0_c) = '1') then
      addsub_res <= std_ulogic_vector(unsigned(opa_v) - unsigned(opb_v));
    else
      addsub_res <= std_ulogic_vector(unsigned(opa_v) + unsigned(opb_v));
    end if;
  end process arithmetic_core;

  -- direct output of adder result --
  add_o <= addsub_res(XLEN-1 downto 0);


  -- ALU Operation Select -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  alu_core: process(ctrl_i, addsub_res, cp_res, rs1_i, opb)
  begin
    case ctrl_i(ctrl_alu_op2_c downto ctrl_alu_op0_c) is
      when alu_op_add_c  => res_o <= addsub_res(XLEN-1 downto 0); -- default
      when alu_op_sub_c  => res_o <= addsub_res(XLEN-1 downto 0);
      when alu_op_cp_c   => res_o <= cp_res;
      when alu_op_slt_c  => res_o <= (others => '0'); res_o(0) <= addsub_res(addsub_res'left); -- carry/borrow
      when alu_op_movb_c => res_o <= opb;
      when alu_op_xor_c  => res_o <= rs1_i xor opb; -- only rs1 required for logic ops (opa would also contain pc)
      when alu_op_or_c   => res_o <= rs1_i or  opb;
      when alu_op_and_c  => res_o <= rs1_i and opb;
      when others        => res_o <= addsub_res(XLEN-1 downto 0); -- don't care
    end case;
  end process alu_core;


  -- **************************************************************************************************************************
  -- ALU Co-Processors
  -- **************************************************************************************************************************

  -- co-processor select / start trigger --
  -- > "cp_start" is high for one cycle to trigger operation of the according co-processor
  cp_start(4 downto 0) <= ctrl_i(ctrl_cp_trig4_c downto ctrl_cp_trig0_c);

  -- (iterative) co-processor operation done? --
  -- > "cp_valid" signal has to be set (for one cycle) one cycle before CP output data (cp_result) is valid
  idone_o <= cp_valid(0) or cp_valid(1) or cp_valid(2) or cp_valid(3) or cp_valid(4);

  -- co-processor result --
  -- > "cp_result" data has to be always zero unless the specific co-processor has been actually triggered
  cp_res <= cp_result(0) or cp_result(1) or cp_result(2) or cp_result(3) or cp_result(4);


  -- Co-Processor 0: Shifter Unit ('I'/'E' Base ISA) ----------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_cp_shifter_inst: neorv32_cpu_cp_shifter
  generic map (
    XLEN          => XLEN,         -- data path width
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
    shamt_i => opb(index_size_f(XLEN)-1 downto 0), -- shift amount
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
      XLEN        => XLEN,                 -- data path width
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
      XLEN          => XLEN,         -- data path width
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
      shamt_i => opb(index_size_f(XLEN)-1 downto 0), -- shift amount
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
    generic map (
      XLEN => XLEN -- data path width
    )
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
      rs3_i    => rs3_i,        -- rf source 3
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
    generic map (
      XLEN => XLEN -- data path width
    )
    port map (
      -- global control --
      clk_i   => clk_i,        -- global clock, rising edge
      rstn_i  => rstn_i,       -- global reset, low-active, async
      ctrl_i  => ctrl_i,       -- main control bus
      start_i => cp_start(4),  -- trigger operation
      -- data input --
      rs1_i   => rs1_i,        -- rf source 1
      rs2_i   => rs2_i,        -- rf source 2
      rs3_i   => rs3_i,        -- rf source 3
      rs4_i   => rs4_i,        -- rf source 4
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


end neorv32_cpu_cpu_rtl;
