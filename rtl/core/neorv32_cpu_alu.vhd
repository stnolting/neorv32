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
    CPU_EXTENSION_RISCV_M : boolean := true; -- implement muld/div extension?
    FAST_SHIFT_EN         : boolean := false -- use barrel shifter for shift operations
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
    -- data output --
    res_o       : out std_ulogic_vector(data_width_c-1 downto 0); -- ALU result
    add_o       : out std_ulogic_vector(data_width_c-1 downto 0); -- address computation result
    -- co-processor interface --
    cp_start_o  : out std_ulogic_vector(7 downto 0); -- trigger co-processor i
    cp_valid_i  : in  std_ulogic_vector(7 downto 0); -- co-processor i done
    cp_result_i : in  cp_data_if_t; -- co-processor result
    -- status --
    wait_o      : out std_ulogic -- busy due to iterative processing units
  );
end neorv32_cpu_alu;

architecture neorv32_cpu_cpu_rtl of neorv32_cpu_alu is

  -- operands --
  signal opa, opb : std_ulogic_vector(data_width_c-1 downto 0);

  -- results --
  signal addsub_res : std_ulogic_vector(data_width_c downto 0);
  --
  signal cp_res     : std_ulogic_vector(data_width_c-1 downto 0);
  signal arith_res  : std_ulogic_vector(data_width_c-1 downto 0);
  signal logic_res  : std_ulogic_vector(data_width_c-1 downto 0);

  -- shifter --
  type shifter_t is record
    cmd     : std_ulogic;
    cmd_ff  : std_ulogic;
    start   : std_ulogic;
    run     : std_ulogic;
    halt    : std_ulogic;
    cnt     : std_ulogic_vector(4 downto 0);
    sreg    : std_ulogic_vector(data_width_c-1 downto 0);
    -- for barrel shifter only --
    bs_a_in : std_ulogic_vector(4 downto 0);
    bs_d_in : std_ulogic_vector(data_width_c-1 downto 0);
  end record;
  signal shifter : shifter_t;

  -- co-processor arbiter and interface --
  type cp_ctrl_t is record
    cmd    : std_ulogic;
    cmd_ff : std_ulogic;
    busy   : std_ulogic;
    start  : std_ulogic;
    halt   : std_ulogic;
  end record;
  signal cp_ctrl : cp_ctrl_t;

begin

  -- Operand Mux ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  opa <= pc2_i when (ctrl_i(ctrl_alu_opa_mux_c) = '1') else rs1_i; -- operand a (first ALU input operand), only required for arithmetic ops
  opb <= imm_i when (ctrl_i(ctrl_alu_opb_mux_c) = '1') else rs2_i; -- operand b (second ALU input operand)


  -- Binary Adder/Subtractor ----------------------------------------------------------------
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
    if (ctrl_i(ctrl_alu_addsub_c) = '1') then -- subtraction
      op_y_v   := not op_b_v;
      cin_v(0) := '1';
    else -- addition
      op_y_v   := op_b_v;
      cin_v(0) := '0';
    end if;
    -- adder core (result + carry/borrow) --
    addsub_res <= std_ulogic_vector(unsigned(op_a_v) + unsigned(op_y_v) + unsigned(cin_v(0 downto 0)));
  end process binary_arithmetic_core;

  -- direct output of address result --
  add_o <= addsub_res(data_width_c-1 downto 0);

  -- ALU arithmetic logic core --
  arithmetic_core: process(ctrl_i, addsub_res)
  begin
    if (ctrl_i(ctrl_alu_arith_c) = alu_arith_cmd_addsub_c) then -- ADD/SUB
      arith_res <= addsub_res(data_width_c-1 downto 0);
    else -- SLT
      arith_res <= (others => '0');
      arith_res(0) <= addsub_res(addsub_res'left); -- => carry/borrow
    end if;
  end process arithmetic_core;


  -- Shifter Unit ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  shifter_unit: process(clk_i)
    variable bs_input_v   : std_ulogic_vector(data_width_c-1 downto 0);
    variable bs_level_4_v : std_ulogic_vector(data_width_c-1 downto 0);
    variable bs_level_3_v : std_ulogic_vector(data_width_c-1 downto 0);
    variable bs_level_2_v : std_ulogic_vector(data_width_c-1 downto 0);
    variable bs_level_1_v : std_ulogic_vector(data_width_c-1 downto 0);
    variable bs_level_0_v : std_ulogic_vector(data_width_c-1 downto 0);
  begin
    if rising_edge(clk_i) then
      shifter.cmd_ff <= shifter.cmd;

      -- --------------------------------------------------------------------------------
      -- Iterative shifter (small but slow) (default)
      -- --------------------------------------------------------------------------------
      if (FAST_SHIFT_EN = false) then

        if (shifter.start = '1') then -- trigger new shift
          shifter.sreg <= rs1_i; -- shift operand (can only be rs1; opa would also contain pc)
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

      -- --------------------------------------------------------------------------------
      -- Barrel shifter (huge but fast)
      -- --------------------------------------------------------------------------------
      else
        -- operands and cycle control --
        if (shifter.start = '1') then -- trigger new shift
          shifter.bs_d_in <= rs1_i; -- shift operand (can only be rs1; opa would also contain pc)
          shifter.bs_a_in <= opb(index_size_f(data_width_c)-1 downto 0); -- shift amount
          shifter.cnt     <= (others => '0');
        end if;

        -- convert left shifts to right shifts --
        if (ctrl_i(ctrl_alu_shift_dir_c) = '0') then -- is left shift?
          bs_input_v := bit_rev_f(shifter.bs_d_in); -- reverse bit order of input operand
        else
          bs_input_v := shifter.bs_d_in;
        end if;
        -- shift >> 16 --
        if (shifter.bs_a_in(4) = '1') then
          bs_level_4_v(31 downto 16) := (others => (bs_input_v(bs_input_v'left) and ctrl_i(ctrl_alu_shift_ar_c)));
          bs_level_4_v(15 downto 00) := (bs_input_v(31 downto 16));
        else
          bs_level_4_v := bs_input_v;
        end if;
        -- shift >> 8 --
        if (shifter.bs_a_in(3) = '1') then
          bs_level_3_v(31 downto 24) := (others => (bs_input_v(bs_input_v'left) and ctrl_i(ctrl_alu_shift_ar_c)));
          bs_level_3_v(23 downto 00) := (bs_level_4_v(31 downto 8));
        else
          bs_level_3_v := bs_level_4_v;
        end if;
        -- shift >> 4 --
        if (shifter.bs_a_in(2) = '1') then
          bs_level_2_v(31 downto 28) := (others => (bs_input_v(bs_input_v'left) and ctrl_i(ctrl_alu_shift_ar_c)));
          bs_level_2_v(27 downto 00) := (bs_level_3_v(31 downto 4));
        else
          bs_level_2_v := bs_level_3_v;
        end if;
        -- shift >> 2 --
        if (shifter.bs_a_in(1) = '1') then
          bs_level_1_v(31 downto 30) := (others => (bs_input_v(bs_input_v'left) and ctrl_i(ctrl_alu_shift_ar_c)));
          bs_level_1_v(29 downto 00) := (bs_level_2_v(31 downto 2));
        else
          bs_level_1_v := bs_level_2_v;
        end if;
        -- shift >> 1 --
        if (shifter.bs_a_in(0) = '1') then
          bs_level_0_v(31 downto 31) := (others => (bs_input_v(bs_input_v'left) and ctrl_i(ctrl_alu_shift_ar_c)));
          bs_level_0_v(30 downto 00) := (bs_level_1_v(31 downto 1));
        else
          bs_level_0_v := bs_level_1_v;
        end if;
        -- re-convert original left shifts --
        if (ctrl_i(ctrl_alu_shift_dir_c) = '0') then
          shifter.sreg <= bit_rev_f(bs_level_0_v);
        else
          shifter.sreg <= bs_level_0_v;
        end if;
      end if;
    end if;
  end process shifter_unit;

  -- is shift operation? --
  shifter.cmd   <= '1' when (ctrl_i(ctrl_alu_func1_c downto ctrl_alu_func0_c) = alu_func_cmd_shift_c) else '0';
  shifter.start <= '1' when (shifter.cmd = '1') and (shifter.cmd_ff = '0') else '0';

  -- shift operation running? --
  shifter.run  <= '1' when (or_all_f(shifter.cnt) = '1') or (shifter.start = '1') else '0';
  shifter.halt <= '1' when (or_all_f(shifter.cnt(shifter.cnt'left downto 1)) = '1') or (shifter.start = '1') else '0';


  -- Co-Processor Arbiter -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- Interface:
  -- Co-processor "valid" signal has to be asserted (for one cycle) one cycle before asserting output data
  -- Co-processor "output data" has to be always zero unless co-processor was explicitly triggered
  cp_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      cp_ctrl.cmd_ff <= '0';
      cp_ctrl.busy   <= '0';
    elsif rising_edge(clk_i) then
      cp_ctrl.cmd_ff <= cp_ctrl.cmd;
      if (or_all_f(cp_valid_i) = '1') then -- cp computation done?
        cp_ctrl.busy <= '0';
      elsif (cp_ctrl.start = '1') then
        cp_ctrl.busy <= '1';
      end if;
    end if;
  end process cp_arbiter;

  -- is co-processor operation? --
  cp_ctrl.cmd   <= '1' when (ctrl_i(ctrl_alu_func1_c downto ctrl_alu_func0_c) = alu_func_cmd_copro_c) else '0';
  cp_ctrl.start <= '1' when (cp_ctrl.cmd = '1') and (cp_ctrl.cmd_ff = '0') else '0';

  -- co-processor select --
  cp_operation_trigger: process(cp_ctrl, ctrl_i)
  begin
    for i in 0 to 7 loop
      if (cp_ctrl.start = '1') and (ctrl_i(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) = std_ulogic_vector(to_unsigned(i, 3))) then
        cp_start_o(i) <= '1';
      else
        cp_start_o(i) <= '0';
      end if;
    end loop; -- i
  end process;

  -- co-processor operation (still) running? --
  cp_ctrl.halt <= (cp_ctrl.busy and (not or_all_f(cp_valid_i))) or cp_ctrl.start;

  -- co-processor result - only the *actually selected* co-processor may output data != 0 --
  cp_res <= cp_result_i(0) or cp_result_i(1) or cp_result_i(2) or cp_result_i(3) or
            cp_result_i(4) or cp_result_i(5) or cp_result_i(6) or cp_result_i(7);


  -- ALU Logic Core -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  alu_logic_core: process(ctrl_i, rs1_i, opb)
  begin
    case ctrl_i(ctrl_alu_logic1_c downto ctrl_alu_logic0_c) is
      when alu_logic_cmd_movb_c => logic_res <= opb; -- (default)
      when alu_logic_cmd_xor_c  => logic_res <= rs1_i xor opb; -- only rs1 required for logic ops (opa would also contain pc)
      when alu_logic_cmd_or_c   => logic_res <= rs1_i or  opb;
      when alu_logic_cmd_and_c  => logic_res <= rs1_i and opb;
      when others               => logic_res <= opb; -- undefined
    end case;
  end process alu_logic_core;


  -- ALU Function Select --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  alu_function_mux: process(ctrl_i, arith_res, logic_res, shifter.sreg, cp_res)
  begin
    case ctrl_i(ctrl_alu_func1_c downto ctrl_alu_func0_c) is
      when alu_func_cmd_arith_c => res_o <= arith_res; -- (default)
      when alu_func_cmd_logic_c => res_o <= logic_res;
      when alu_func_cmd_shift_c => res_o <= shifter.sreg;
      when alu_func_cmd_copro_c => res_o <= cp_res;
      when others               => res_o <= arith_res; -- undefined
    end case;
  end process alu_function_mux;


  -- ALU Busy -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  wait_o <= shifter.halt or cp_ctrl.halt; -- wait until iterative units have completed


end neorv32_cpu_cpu_rtl;
