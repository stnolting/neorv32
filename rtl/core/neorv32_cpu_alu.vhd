-- #################################################################################################
-- # << NEORV32 CPU - Arithmetic/Logic Unit >>                                                     #
-- # ********************************************************************************************* #
-- # Main data/address ALU and ALU co-processors (= multi-cycle function units).                   #
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
-- # The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32       (c) Stephan Nolting #
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
    -- Tuning Options --
    FAST_MUL_EN               : boolean; -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN             : boolean  -- use barrel shifter for shift operations
  );
  port (
    -- global control --
    clk_i       : in  std_ulogic; -- global clock, rising edge
    rstn_i      : in  std_ulogic; -- global reset, low-active, async
    ctrl_i      : in  ctrl_bus_t; -- main control bus
    -- CSR interface --
    csr_we_i    : in  std_ulogic; -- global write enable
    csr_addr_i  : in  std_ulogic_vector(11 downto 0); -- address
    csr_wdata_i : in  std_ulogic_vector(XLEN-1 downto 0); -- write data
    csr_rdata_o : out std_ulogic_vector(XLEN-1 downto 0); -- read data
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
    -- status --
    cp_done_o   : out std_ulogic  -- co-processor operation done?
  );
end neorv32_cpu_alu;

architecture neorv32_cpu_cpu_rtl of neorv32_cpu_alu is

  -- comparator --
  signal cmp_rs1 : std_ulogic_vector(XLEN downto 0);
  signal cmp_rs2 : std_ulogic_vector(XLEN downto 0);
  signal cmp     : std_ulogic_vector(1 downto 0); -- comparator status

  -- operands --
  signal opa,   opb   : std_ulogic_vector(XLEN-1 downto 0);
  signal opa_x, opb_x : std_ulogic_vector(XLEN downto 0);

  -- intermediate results --
  signal addsub_res : std_ulogic_vector(XLEN downto 0);
  signal cp_res     : std_ulogic_vector(XLEN-1 downto 0);

  -- co-processor interface --
  type cp_data_t  is array (0 to 4) of std_ulogic_vector(XLEN-1 downto 0);
  signal cp_result : cp_data_t; -- co-processor result
  signal cp_start  : std_ulogic_vector(4 downto 0); -- co-processor trigger
  signal cp_valid  : std_ulogic_vector(4 downto 0); -- co-processor done

  -- CSR read-backs --
  signal csr_rdata_fpu, csr_rdata_cfu : std_ulogic_vector(XLEN-1 downto 0);

begin

  -- Comparator Unit (for conditional branches) ---------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cmp_rs1 <= (rs1_i(rs1_i'left) and (not ctrl_i.alu_unsigned)) & rs1_i; -- sign-extend
  cmp_rs2 <= (rs2_i(rs2_i'left) and (not ctrl_i.alu_unsigned)) & rs2_i; -- sign-extend

  cmp(cmp_equal_c) <= '1' when (rs1_i = rs2_i) else '0';
  cmp(cmp_less_c)  <= '1' when (signed(cmp_rs1) < signed(cmp_rs2)) else '0'; -- signed or unsigned comparison
  cmp_o            <= cmp;


  -- ALU Input Operand Select ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  opa <= pc_i  when (ctrl_i.alu_opa_mux = '1') else rs1_i;
  opb <= imm_i when (ctrl_i.alu_opb_mux = '1') else rs2_i;


  -- Adder/Subtracter Core ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  opa_x <= (opa(opa'left) and (not ctrl_i.alu_unsigned)) & opa; -- sign-extend
  opb_x <= (opb(opb'left) and (not ctrl_i.alu_unsigned)) & opb; -- sign-extend

  addsub_res <= std_ulogic_vector(unsigned(opa_x) - unsigned(opb_x)) when (ctrl_i.alu_op(0) = '1') else
                std_ulogic_vector(unsigned(opa_x) + unsigned(opb_x));

  add_o <= addsub_res(XLEN-1 downto 0); -- direct output of adder result


  -- ALU Operation Select -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  alu_core: process(ctrl_i, addsub_res, cp_res, rs1_i, opb)
  begin
    case ctrl_i.alu_op is
      when alu_op_add_c  => res_o <= addsub_res(XLEN-1 downto 0);
      when alu_op_sub_c  => res_o <= addsub_res(XLEN-1 downto 0);
      when alu_op_cp_c   => res_o <= cp_res;
      when alu_op_slt_c  => res_o(XLEN-1 downto 1) <= (others => '0');
                            res_o(0) <= addsub_res(addsub_res'left); -- carry/borrow
      when alu_op_movb_c => res_o <= opb;
      when alu_op_xor_c  => res_o <= rs1_i xor opb; -- only rs1 is required for logic ops (opa would also contain pc)
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
  cp_start <= ctrl_i.alu_cp_trig;

  -- multi-cycle co-processor operation done? --
  -- > "cp_valid" signal has to be set (for one cycle) one cycle before CP output data (cp_result) is valid
  cp_done_o <= cp_valid(0) or cp_valid(1) or cp_valid(2) or cp_valid(3) or cp_valid(4);

  -- co-processor result --
  -- > "cp_result" data has to be always zero unless the specific co-processor has been actually triggered
  cp_res <= cp_result(0) or cp_result(1) or cp_result(2) or cp_result(3) or cp_result(4);

  -- co-processor CSR read-back --
  -- > "csr_rdata_*" data has to be always zero unless the specific co-processor is actually being accessed
  csr_rdata_o <= csr_rdata_fpu or csr_rdata_cfu;


  -- Co-Processor 0: Shifter Unit (Base ISA) ------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_cp_shifter_inst: entity neorv32.neorv32_cpu_cp_shifter
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
    shamt_i => opb(index_size_f(XLEN)-1 downto 0), -- shift amount
    -- result and status --
    res_o   => cp_result(0), -- operation result
    valid_o => cp_valid(0)   -- data output valid
  );


  -- Co-Processor 1: Integer Multiplication/Division Unit ('M' ISA Extension) ---------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_cp_muldiv_inst_true:
  if (CPU_EXTENSION_RISCV_M = true) or (CPU_EXTENSION_RISCV_Zmmul = true) generate
    neorv32_cpu_cp_muldiv_inst: entity neorv32.neorv32_cpu_cp_muldiv
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


  -- Co-Processor 2: Bit-Manipulation Unit ('B' ISA Extension) ------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_cp_bitmanip_inst_true:
  if (CPU_EXTENSION_RISCV_B = true) generate
    neorv32_cpu_cp_bitmanip_inst: entity neorv32.neorv32_cpu_cp_bitmanip
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


  -- Co-Processor 3: Single-Precision Floating-Point Unit ('Zfinx' ISA Extension) -----------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_cp_fpu_inst_true:
  if (CPU_EXTENSION_RISCV_Zfinx = true) generate
    neorv32_cpu_cp_fpu_inst: entity neorv32.neorv32_cpu_cp_fpu
    port map (
      -- global control --
      clk_i       => clk_i,         -- global clock, rising edge
      rstn_i      => rstn_i,        -- global reset, low-active, async
      ctrl_i      => ctrl_i,        -- main control bus
      start_i     => cp_start(3),   -- trigger operation
      -- CSR interface --
      csr_we_i    => csr_we_i,      -- global write enable
      csr_addr_i  => csr_addr_i,    -- address
      csr_wdata_i => csr_wdata_i,   -- write data
      csr_rdata_o => csr_rdata_fpu, -- read data
      -- data input --
      cmp_i       => cmp,           -- comparator status
      rs1_i       => rs1_i,         -- rf source 1
      rs2_i       => rs2_i,         -- rf source 2
      rs3_i       => rs3_i,         -- rf source 3
      -- result and status --
      res_o       => cp_result(3),  -- operation result
      valid_o     => cp_valid(3)    -- data output valid
    );
  end generate;

  neorv32_cpu_cp_fpu_inst_false:
  if (CPU_EXTENSION_RISCV_Zfinx = false) generate
    csr_rdata_fpu <= (others => '0');
    cp_result(3)  <= (others => '0');
    cp_valid(3)   <= '0';
  end generate;


  -- Co-Processor 4: Custom (Instructions) Functions Unit ('Zxcfu' ISA Extension) -----------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_cp_cfu_inst_true:
  if (CPU_EXTENSION_RISCV_Zxcfu = true) generate
    neorv32_cpu_cp_cfu_inst: entity neorv32.neorv32_cpu_cp_cfu
    port map (
      -- global control --
      clk_i   => clk_i,             -- global clock, rising edge
      rstn_i  => rstn_i,            -- global reset, low-active, async
      ctrl_i  => ctrl_i,            -- main control bus
      start_i => cp_start(4),       -- trigger operation
      -- CSR interface --
      csr_we_i    => csr_we_i,      -- global write enable
      csr_addr_i  => csr_addr_i,    -- address
      csr_wdata_i => csr_wdata_i,   -- write data
      csr_rdata_o => csr_rdata_cfu, -- read data
      -- data input --
      rs1_i   => rs1_i,             -- rf source 1
      rs2_i   => rs2_i,             -- rf source 2
      rs3_i   => rs3_i,             -- rf source 3
      rs4_i   => rs4_i,             -- rf source 4
      -- result and status --
      res_o   => cp_result(4),      -- operation result
      valid_o => cp_valid(4)        -- data output valid
    );
  end generate;

  neorv32_cpu_cp_cfu_inst_false:
  if (CPU_EXTENSION_RISCV_Zxcfu = false) generate
    csr_rdata_cfu <= (others => '0');
    cp_result(4)  <= (others => '0');
    cp_valid(4)   <= '0';
  end generate;


end neorv32_cpu_cpu_rtl;
