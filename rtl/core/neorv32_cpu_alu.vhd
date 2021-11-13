-- #################################################################################################
-- # << NEORV32 - Arithmetical/Logical Unit >>                                                     #
-- # ********************************************************************************************* #
-- # Main data and address ALU and co-processor interface/arbiter.                                 #
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

entity neorv32_cpu_alu is
  generic (
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_B     : boolean; -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_M     : boolean; -- implement mul/div extension?
    CPU_EXTENSION_RISCV_Zmmul : boolean; -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zfinx : boolean; -- implement 32-bit floating-point extension (using INT reg!)
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
    pc2_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- next PC
    imm_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- immediate
    csr_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- CSR read data
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
  signal alu_res    : std_ulogic_vector(data_width_c-1 downto 0);
  signal cp_res     : std_ulogic_vector(data_width_c-1 downto 0);

  -- co-processor arbiter and interface --
  type cp_ctrl_t is record
    cmd     : std_ulogic;
    cmd_ff  : std_ulogic;
    start   : std_ulogic;
    busy    : std_ulogic;
    timeout : std_ulogic_vector(9 downto 0);
  end record;
  signal cp_ctrl : cp_ctrl_t;

  -- co-processor interface --
  signal cp_start  : std_ulogic_vector(3 downto 0); -- trigger co-processor i
  signal cp_valid  : std_ulogic_vector(3 downto 0); -- co-processor i done
  signal cp_result : cp_data_if_t; -- co-processor result

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
  alu_core: process(ctrl_i, addsub_res, rs1_i, opb)
  begin
    case ctrl_i(ctrl_alu_op2_c downto ctrl_alu_op0_c) is
      when alu_op_add_c  => alu_res <= addsub_res(data_width_c-1 downto 0); -- (default)
      when alu_op_sub_c  => alu_res <= addsub_res(data_width_c-1 downto 0);
--    when alu_op_mova_c => alu_res <= rs1_i; -- FIXME
      when alu_op_slt_c  => alu_res <= (others => '0'); alu_res(0) <= addsub_res(addsub_res'left); -- => carry/borrow
      when alu_op_movb_c => alu_res <= opb;
      when alu_op_xor_c  => alu_res <= rs1_i xor opb; -- only rs1 required for logic ops (opa would also contain pc)
      when alu_op_or_c   => alu_res <= rs1_i or  opb;
      when alu_op_and_c  => alu_res <= rs1_i and opb;
      when others        => alu_res <= addsub_res(data_width_c-1 downto 0);
    end case;
  end process alu_core;

  -- ALU Function Select --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  alu_function_mux: process(ctrl_i, alu_res, pc2_i, csr_i, cp_res)
  begin
    case ctrl_i(ctrl_alu_func1_c downto ctrl_alu_func0_c) is
      when alu_func_core_c  => res_o <= alu_res; -- (default)
      when alu_func_nxpc_c  => res_o <= pc2_i;
      when alu_func_csrr_c  => res_o <= csr_i;
      when alu_func_copro_c => res_o <= cp_res;
      when others           => res_o <= alu_res; -- undefined
    end case;
  end process alu_function_mux;


  -- **************************************************************************************************************************
  -- Co-Processors
  -- **************************************************************************************************************************

  -- Co-Processor Arbiter -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- Interface:
  -- Co-processor "valid" signal has to be asserted (for one cycle) one cycle before asserting output data
  -- Co-processor "output data" has to be always zero unless co-processor was explicitly triggered
  cp_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      cp_ctrl.cmd_ff  <= '0';
      cp_ctrl.busy    <= '0';
      cp_ctrl.timeout <= (others => '0');
    elsif rising_edge(clk_i) then
      cp_ctrl.cmd_ff <= cp_ctrl.cmd;
      -- timeout counter --
      if (cp_ctrl.start = '1') then
        cp_ctrl.busy <= '1';
      elsif (or_reduce_f(cp_valid) = '1') then
        cp_ctrl.busy <= '0';
      end if;
      if (cp_ctrl.busy = '1') and (cp_timeout_en_c = true) then
        cp_ctrl.timeout <= std_ulogic_vector(unsigned(cp_ctrl.timeout) + 1);
      else
        cp_ctrl.timeout <= (others => '0');
      end if;
      if (cp_ctrl.timeout(cp_ctrl.timeout'left) = '1') and (cp_timeout_en_c = true) then -- timeout
        assert false report "NEORV32 CPU CO-PROCESSOR TIMEOUT ERROR!" severity warning;
      end if;
    end if;
  end process cp_arbiter;

  -- is co-processor operation? --
  cp_ctrl.cmd   <= '1' when (ctrl_i(ctrl_alu_func1_c downto ctrl_alu_func0_c) = alu_func_copro_c) else '0';
  cp_ctrl.start <= '1' when (cp_ctrl.cmd = '1') and (cp_ctrl.cmd_ff = '0') else '0';

  -- co-processor select / star trigger --
  cp_start(0) <= '1' when (cp_ctrl.start = '1') and (ctrl_i(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) = "00") else '0';
  cp_start(1) <= '1' when (cp_ctrl.start = '1') and (ctrl_i(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) = "01") else '0';
  cp_start(2) <= '1' when (cp_ctrl.start = '1') and (ctrl_i(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) = "10") else '0';
  cp_start(3) <= '1' when (cp_ctrl.start = '1') and (ctrl_i(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) = "11") else '0';

  -- co-processor operation done? --
  idone_o <= or_reduce_f(cp_valid);

  -- co-processor result - only the *actually selected* co-processor may output data != 0 --
  cp_res <= cp_result(0) or cp_result(1) or cp_result(2) or cp_result(3);


  -- Co-Processor 0: Shifter (CPU Core ISA) --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_cp_shifter_inst: neorv32_cpu_cp_shifter
  generic map (
    FAST_SHIFT_EN => FAST_SHIFT_EN -- use barrel shifter for shift operations
  )
  port map (
    -- global control --
    clk_i   => clk_i,           -- global clock, rising edge
    rstn_i  => rstn_i,          -- global reset, low-active, async
    ctrl_i  => ctrl_i,          -- main control bus
    start_i => cp_start(0),     -- trigger operation
    -- data input --
    rs1_i   => rs1_i,           -- rf source 1
    shamt_i => opb(index_size_f(data_width_c)-1 downto 0), -- shift amount
    -- result and status --
    res_o   => cp_result(0),    -- operation result
    valid_o => cp_valid(0)      -- data output valid
  );


  -- Co-Processor 1: Integer Multiplication/Division ('M' Extension) ------------------------
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
      clk_i   => clk_i,         -- global clock, rising edge
      rstn_i  => rstn_i,        -- global reset, low-active, async
      ctrl_i  => ctrl_i,        -- main control bus
      start_i => cp_start(1),   -- trigger operation
      -- data input --
      rs1_i   => rs1_i,         -- rf source 1
      rs2_i   => rs2_i,         -- rf source 2
      -- result and status --
      res_o   => cp_result(1),  -- operation result
      valid_o => cp_valid(1)    -- data output valid
    );
  end generate;

  neorv32_cpu_cp_muldiv_inst_false:
  if (CPU_EXTENSION_RISCV_M = false) and (CPU_EXTENSION_RISCV_Zmmul = false) generate
    cp_result(1) <= (others => '0');
    cp_valid(1)  <= cp_start(1); -- to make sure CPU does not get stalled if there is an accidental access
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
    cp_valid(2)  <= cp_start(2); -- to make sure CPU does not get stalled if there is an accidental access
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
    cp_valid(3)  <= cp_start(3); -- to make sure CPU does not get stalled if there is an accidental access
  end generate;


end neorv32_cpu_cpu_rtl;
