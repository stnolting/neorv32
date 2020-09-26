-- #################################################################################################
-- # << NEORV32 - Arithmetical/Logical Unit >>                                                     #
-- # ********************************************************************************************* #
-- # Main data and address ALU. Includes comparator unit and co-processor interface/arbiter.       #
-- # The shifter sub-unit uses an iterative approach.                                              #
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

entity neorv32_cpu_alu is
  generic (
    CPU_EXTENSION_RISCV_M : boolean := true -- implement muld/div extension?
  );
  port (
    -- global control --
    clk_i       : in  std_ulogic; -- global clock, rising edge
    rstn_i      : in  std_ulogic; -- global reset, low-active, async
    ctrl_i      : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
    -- data input --
    rs1_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 1
    rs2_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 2
    pc2_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- delayed PC
    imm_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- immediate
    csr_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- csr read data
    -- data output --
    cmp_o       : out std_ulogic_vector(1 downto 0); -- comparator status
    add_o       : out std_ulogic_vector(data_width_c-1 downto 0); -- OPA + OPB
    res_o       : out std_ulogic_vector(data_width_c-1 downto 0); -- ALU result
    -- co-processor interface --
    cp_opa_o    : out std_ulogic_vector(data_width_c-1 downto 0); -- co-processor operand a
    cp_opb_o    : out std_ulogic_vector(data_width_c-1 downto 0); -- co-processor operand b
    cp0_start_o : out std_ulogic; -- trigger co-processor 0
    cp0_data_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- co-processor 0 result
    cp0_valid_i : in  std_ulogic; -- co-processor 0 result valid
    cp1_start_o : out std_ulogic; -- trigger co-processor 1
    cp1_data_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- co-processor 1 result
    cp1_valid_i : in  std_ulogic; -- co-processor 1 result valid
    -- status --
    wait_o      : out std_ulogic -- busy due to iterative processing units
  );
end neorv32_cpu_alu;

architecture neorv32_cpu_cpu_rtl of neorv32_cpu_alu is

  -- operands --
  signal opa, opb, opc : std_ulogic_vector(data_width_c-1 downto 0);

  -- results --
  signal add_res : std_ulogic_vector(data_width_c-1 downto 0);
  signal alu_res : std_ulogic_vector(data_width_c-1 downto 0);
  signal cp_res  : std_ulogic_vector(data_width_c-1 downto 0);

  -- comparator --
  signal cmp_opx   : std_ulogic_vector(data_width_c downto 0);
  signal cmp_opy   : std_ulogic_vector(data_width_c downto 0);
  signal cmp_sub   : std_ulogic_vector(data_width_c downto 0);
  signal sub_res   : std_ulogic_vector(data_width_c-1 downto 0);
  signal cmp_equal : std_ulogic;
  signal cmp_less  : std_ulogic;

  -- shifter --
  type shifter_t is record
    cmd    : std_ulogic;
    cmd_ff : std_ulogic;
    start  : std_ulogic;
    run    : std_ulogic;
    halt   : std_ulogic;
    cnt    : std_ulogic_vector(4 downto 0);
    sreg   : std_ulogic_vector(data_width_c-1 downto 0);
  end record;
  signal shifter : shifter_t;

  -- co-processor arbiter and interface --
  type cp_ctrl_t is record
    cmd_ff : std_ulogic;
    busy   : std_ulogic;
    start  : std_ulogic;
    halt   : std_ulogic;
  end record;
  signal cp_ctrl : cp_ctrl_t;

begin

  -- Operand Mux ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  input_op_mux: process(ctrl_i, csr_i, pc2_i, rs1_i, rs2_i, imm_i)
  begin
    -- opa (first ALU input operand) --
    case ctrl_i(ctrl_alu_opa_mux_msb_c downto ctrl_alu_opa_mux_lsb_c) is
      when "00"   => opa <= rs1_i;
      when "01"   => opa <= pc2_i;
      when "10"   => opa <= csr_i;
      when others => opa <= (others => '0');
    end case;
    -- opb (second ALU input operand) --
    if (ctrl_i(ctrl_alu_opb_mux_c) = '0') then
      opb <= rs2_i;
    else
      opb <= imm_i;
    end if;
    -- opc (second operand for comparison and SUB) --
    if (ctrl_i(ctrl_alu_opc_mux_c) = '0') then
      opc <= imm_i;
    else
      opc <= rs2_i;
    end if;
  end process input_op_mux;


  -- Comparator Unit ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- less than (x < y) --
  cmp_opx  <= (rs1_i(rs1_i'left) and (not ctrl_i(ctrl_alu_unsigned_c))) & rs1_i;
  cmp_opy  <= (opc(opc'left)     and (not ctrl_i(ctrl_alu_unsigned_c))) & opc;
  cmp_sub  <= std_ulogic_vector(signed(cmp_opx) - signed(cmp_opy));
  cmp_less <= cmp_sub(cmp_sub'left); -- carry (borrow) indicates a "less"
  sub_res  <= cmp_sub(data_width_c-1 downto 0); -- use the less-comparator also for SUB operations

  -- equal (x = y) --
  cmp_equal <= '1' when (rs1_i = opc) else '0';

  -- output for branch condition evaluation -
  cmp_o(alu_cmp_equal_c) <= cmp_equal;
  cmp_o(alu_cmp_less_c)  <= cmp_less;


  -- Binary Adder ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  add_res <= std_ulogic_vector(unsigned(opa) + unsigned(opb));
  add_o   <= add_res; -- direct output


  -- Iterative Shifter Unit -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  shifter_unit: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      shifter.sreg   <= (others => '0');
      shifter.cnt    <= (others => '0');
      shifter.cmd_ff <= '0';
    elsif rising_edge(clk_i) then
      shifter.cmd_ff <= shifter.cmd;
      if (shifter.start = '1') then -- trigger new shift
        shifter.sreg <= opa; -- shift operand
        shifter.cnt  <= opb(index_size_f(data_width_c)-1 downto 0); -- shift amount
      elsif (shifter.run = '1') then -- running shift
        -- coarse shift: multiples of 4 --
        if (or_all_f(shifter.cnt(shifter.cnt'left downto 2)) = '1') then -- shift amount >= 4
          shifter.cnt <= std_ulogic_vector(unsigned(shifter.cnt) - 4);
          if (ctrl_i(ctrl_alu_shift_dir_c) = '0') then -- SLL: shift left logical
            shifter.sreg <= shifter.sreg(shifter.sreg'left-4 downto 0) & "0000";
          else -- SRL: shift right logical / SRA: shift right arithmetical
            shifter.sreg <= (shifter.sreg(shifter.sreg'left) and ctrl_i(ctrl_alu_shift_ar_c)) &
                            (shifter.sreg(shifter.sreg'left) and ctrl_i(ctrl_alu_shift_ar_c)) &
                            (shifter.sreg(shifter.sreg'left) and ctrl_i(ctrl_alu_shift_ar_c)) &
                            (shifter.sreg(shifter.sreg'left) and ctrl_i(ctrl_alu_shift_ar_c)) & shifter.sreg(shifter.sreg'left downto 4);
          end if;
        -- fine shift: single shifts, 0..3 times --
        else
          shifter.cnt <= std_ulogic_vector(unsigned(shifter.cnt) - 1);
          if (ctrl_i(ctrl_alu_shift_dir_c) = '0') then -- SLL: shift left logical
            shifter.sreg <= shifter.sreg(shifter.sreg'left-1 downto 0) & '0';
          else -- SRL: shift right logical / SRA: shift right arithmetical
            shifter.sreg <= (shifter.sreg(shifter.sreg'left) and ctrl_i(ctrl_alu_shift_ar_c)) & shifter.sreg(shifter.sreg'left downto 1);
          end if;
        end if;
      end if;
    end if;
  end process shifter_unit;

  -- is shift operation? --
  shifter.cmd   <= '1' when (ctrl_i(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) = alu_cmd_shift_c) and (ctrl_i(ctrl_cp_use_c) = '0') else '0';
  shifter.start <= '1' when (shifter.cmd = '1') and (shifter.cmd_ff = '0') else '0';

  -- shift operation running? --
  shifter.run  <= '1' when (or_all_f(shifter.cnt) = '1') or (shifter.start = '1') else '0';
  shifter.halt <= '1' when (or_all_f(shifter.cnt(shifter.cnt'left downto 1)) = '1') or (shifter.start = '1') else '0';


  -- Coprocessor Arbiter --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cp_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      cp_ctrl.cmd_ff <= '0';
      cp_ctrl.busy   <= '0';
    elsif rising_edge(clk_i) then
      if (CPU_EXTENSION_RISCV_M = true) then
        cp_ctrl.cmd_ff <= ctrl_i(ctrl_cp_use_c);
        if (cp_ctrl.start = '1') then
          cp_ctrl.busy <= '1';
        elsif ((cp0_valid_i or cp1_valid_i) = '1') then -- cp computation done?
          cp_ctrl.busy <= '0';
        end if;
      else -- no co-processor(s) implemented
        cp_ctrl.cmd_ff <= '0';
        cp_ctrl.busy   <= '0';
      end if;
    end if;
  end process cp_arbiter;

  -- is co-processor operation? --
  cp_ctrl.start <= '1' when (ctrl_i(ctrl_cp_use_c) = '1') and (cp_ctrl.cmd_ff = '0') else '0';
  cp0_start_o   <= '1' when (cp_ctrl.start = '1') and (ctrl_i(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) = cp_sel_muldiv_c) else '0'; -- MULDIV CP
  cp1_start_o   <= '0'; -- not yet implemented

  -- co-processor operation running? --
  cp_ctrl.halt <= cp_ctrl.busy or cp_ctrl.start;

  -- co-processor operands --
  cp_opa_o <= opa;
  cp_opb_o <= opb;

  -- co-processor result --
  cp_res <= cp0_data_i or cp1_data_i; -- only the selcted cp may output data != 0


  -- ALU Function Select --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  alu_function_mux: process(ctrl_i, opa, opb, add_res, sub_res, cmp_less, shifter.sreg)
  begin
    case ctrl_i(ctrl_alu_cmd2_c downto ctrl_alu_cmd0_c) is
      when alu_cmd_xor_c   => alu_res <= opa xor opb;
      when alu_cmd_or_c    => alu_res <= opa or  opb;
      when alu_cmd_and_c   => alu_res <= opa and opb;
      when alu_cmd_bclr_c  => alu_res <= opa and (not opb);
      when alu_cmd_sub_c   => alu_res <= sub_res;
      when alu_cmd_add_c   => alu_res <= add_res;
      when alu_cmd_shift_c => alu_res <= shifter.sreg;
      when alu_cmd_slt_c   => alu_res <= (others => '0'); alu_res(0) <= cmp_less;
      when others          => alu_res <= (others => '0'); -- undefined
    end case;
  end process alu_function_mux;


  -- ALU Result -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  wait_o <= shifter.halt or cp_ctrl.halt; -- wait until iterative units have completed
  res_o  <= cp_res when (ctrl_i(ctrl_cp_use_c) = '1') else alu_res; -- FIXME?


end neorv32_cpu_cpu_rtl;
