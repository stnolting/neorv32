-- #################################################################################################
-- # << NEORV32 - CPU Co-Processor: Integer Multiplier/Divider Unit (RISC-V "M" Extension) >>      #
-- # ********************************************************************************************* #
-- # Multiplier core (signed/unsigned) uses serial add-and-shift algorithm. Multiplications can be #
-- # mapped to DSP blocks (faster!) when FAST_MUL_EN = true. Divider core (unsigned-only; pre and  #
-- # post sign-compensation logic) uses serial restoring serial algorithm.                         #
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

entity neorv32_cpu_cp_muldiv is
  generic (
    FAST_MUL_EN : boolean; -- use DSPs for faster multiplication
    DIVISION_EN : boolean  -- implement divider hardware
  );
  port (
    -- global control --
    clk_i   : in  std_ulogic; -- global clock, rising edge
    rstn_i  : in  std_ulogic; -- global reset, low-active, async
    ctrl_i  : in  ctrl_bus_t; -- main control bus
    start_i : in  std_ulogic; -- trigger operation
    -- data input --
    rs1_i   : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 1
    rs2_i   : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 2
    -- result and status --
    res_o   : out std_ulogic_vector(XLEN-1 downto 0); -- operation result
    valid_o : out std_ulogic -- data output valid
  );
end neorv32_cpu_cp_muldiv;

architecture neorv32_cpu_cp_muldiv_rtl of neorv32_cpu_cp_muldiv is

  -- operations --
  constant cp_op_mul_c    : std_ulogic_vector(2 downto 0) := "000"; -- mul
  constant cp_op_mulh_c   : std_ulogic_vector(2 downto 0) := "001"; -- mulh
  constant cp_op_mulhsu_c : std_ulogic_vector(2 downto 0) := "010"; -- mulhsu
  constant cp_op_mulhu_c  : std_ulogic_vector(2 downto 0) := "011"; -- mulhu
  constant cp_op_div_c    : std_ulogic_vector(2 downto 0) := "100"; -- div
  constant cp_op_divu_c   : std_ulogic_vector(2 downto 0) := "101"; -- divu
  constant cp_op_rem_c    : std_ulogic_vector(2 downto 0) := "110"; -- rem
  constant cp_op_remu_c   : std_ulogic_vector(2 downto 0) := "111"; -- remu

  -- controller --
  type state_t is (S_IDLE, S_BUSY, S_DONE);
  type ctrl_t is record
    state         : state_t;
    cnt           : std_ulogic_vector(index_size_f(XLEN)-1 downto 0); -- iteration counter
    cp_op         : std_ulogic_vector(2 downto 0); -- operation to execute
    cp_op_ff      : std_ulogic_vector(2 downto 0);
    op            : std_ulogic; -- 0 = mul, 1 = div
    rs1_is_signed : std_ulogic;
    rs2_is_signed : std_ulogic;
    out_en        : std_ulogic;
    rs2_abs       : std_ulogic_vector(XLEN-1 downto 0);
  end record;
  signal ctrl : ctrl_t;

  -- divider core --
  type div_t is record
    start     : std_ulogic; -- start new division
    sign_mod  : std_ulogic; -- result sign correction
    remainder : std_ulogic_vector(XLEN-1 downto 0);
    quotient  : std_ulogic_vector(XLEN-1 downto 0);
    sub       : std_ulogic_vector(XLEN   downto 0); -- try subtraction (and restore if underflow)
    res_u     : std_ulogic_vector(XLEN-1 downto 0); -- unsigned result
    res       : std_ulogic_vector(XLEN-1 downto 0);
  end record;
  signal div : div_t;

  -- multiplier core --
  type mul_t is record
    start  : std_ulogic; -- start new multiplication
    prod   : std_ulogic_vector((2*XLEN)-1 downto 0); -- product
    add    : std_ulogic_vector(XLEN downto 0); -- addition step
    p_sext : std_ulogic; -- product sign-extension
    dsp_x  : signed(XLEN downto 0); -- input for using DSPs
    dsp_y  : signed(XLEN downto 0); -- input for using DSPs
    dsp_z  : signed(2*XLEN+1 downto 0);
  end record;
  signal mul : mul_t;

begin

  -- Co-Processor Controller ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  coprocessor_ctrl: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl.state    <= S_IDLE;
      ctrl.rs2_abs  <= (others => '0');
      ctrl.cnt      <= (others => '0');
      ctrl.cp_op_ff <= (others => '0');
      ctrl.out_en   <= '0';
      div.sign_mod  <= '0';
    elsif rising_edge(clk_i) then
      -- defaults --
      ctrl.out_en <= '0';

      -- FSM --
      case ctrl.state is

        when S_IDLE => -- wait for start signal
          -- arbitration
          ctrl.cp_op_ff <= ctrl.cp_op;
          ctrl.cnt      <= std_ulogic_vector(to_unsigned(XLEN-2, index_size_f(XLEN))); -- iterative cycle counter
          if (start_i = '1') then -- trigger new operation
            if (DIVISION_EN = true) then
              -- DIV: check relevant input signs for result sign compensation --
              if (ctrl.cp_op(1 downto 0) = cp_op_div_c(1 downto 0)) then -- signed div operation
                div.sign_mod <= (rs1_i(rs1_i'left) xor rs2_i(rs2_i'left)) and or_reduce_f(rs2_i); -- different signs AND divisor not zero
              elsif (ctrl.cp_op(1 downto 0) = cp_op_rem_c(1 downto 0)) then -- signed rem operation
                div.sign_mod <= rs1_i(rs1_i'left);
              else
                div.sign_mod <= '0';
              end if;
              -- DIV: abs(rs2) --
              if ((rs2_i(rs2_i'left) and ctrl.rs2_is_signed) = '1') then -- signed division?
                ctrl.rs2_abs <= std_ulogic_vector(0 - unsigned(rs2_i)); -- make positive
              else
                ctrl.rs2_abs <= rs2_i;
              end if;
            end if;
            -- is fast multiplication?--
            if (ctrl.op = '0') and (FAST_MUL_EN = true) then
              ctrl.state <= S_DONE;
            else -- serial division or serial multiplication
              ctrl.state <= S_BUSY;
            end if;
          end if;

        when S_BUSY => -- processing
          ctrl.cnt <= std_ulogic_vector(unsigned(ctrl.cnt) - 1);
          if (or_reduce_f(ctrl.cnt) = '0') or (ctrl_i.cpu_trap = '1') then -- abort on trap
            ctrl.state <= S_DONE;
          end if;

        when S_DONE => -- final step / enable output for one cycle
          ctrl.out_en <= '1';
          ctrl.state  <= S_IDLE;

        when others => -- undefined
          ctrl.state <= S_IDLE;
      end case;
    end if;
  end process coprocessor_ctrl;

  -- done? assert one cycle before actual data output --
  valid_o <= '1' when (ctrl.state = S_DONE) else '0';

  -- co-processor operation --
  ctrl.cp_op <= ctrl_i.ir_funct3;
  ctrl.op    <= '1' when (ctrl_i.ir_funct3(2) = '1') else '0';

  -- input operands treated as signed? --
  ctrl.rs1_is_signed <= '1' when (ctrl.cp_op = cp_op_mulh_c) or (ctrl.cp_op = cp_op_mulhsu_c) or
                                 (ctrl.cp_op = cp_op_div_c) or (ctrl.cp_op = cp_op_rem_c) else '0';
  ctrl.rs2_is_signed <= '1' when (ctrl.cp_op = cp_op_mulh_c) or
                                 (ctrl.cp_op = cp_op_div_c) or (ctrl.cp_op = cp_op_rem_c) else '0';

  -- start operation (do it fast!) --
  mul.start <= '1' when (start_i = '1') and (ctrl.op = '0') else '0';
  div.start <= '1' when (start_i = '1') and (ctrl.op = '1') else '0';


  -- Multiplier Core (signed/unsigned) - Full Parallel --------------------------------------
  -- -------------------------------------------------------------------------------------------
  multiplier_core_parallel:
  if (FAST_MUL_EN = true) generate

    -- direct approach --
    multiplier_core: process(clk_i)
    begin
      if rising_edge(clk_i) then
        if (mul.start = '1') then
          mul.dsp_x <= signed((rs1_i(rs1_i'left) and ctrl.rs1_is_signed) & rs1_i);
          mul.dsp_y <= signed((rs2_i(rs2_i'left) and ctrl.rs2_is_signed) & rs2_i);
        end if;
        mul.prod <= std_ulogic_vector(mul.dsp_z(63 downto 0));
      end if;
    end process multiplier_core;

    -- actual multiplication --
    mul.dsp_z <= mul.dsp_x * mul.dsp_y;

  end generate; --/multiplier_core_parallel

  -- no parallel multiplier --
  multiplier_core_parallel_none:
  if (FAST_MUL_EN = false) generate
    mul.dsp_x <= (others => '0');
    mul.dsp_y <= (others => '0');
    mul.dsp_z <= (others => '0');
  end generate;


  -- Multiplier Core (signed/unsigned) - Iterative ------------------------------------------
  -- -------------------------------------------------------------------------------------------
  multiplier_core_serial:
  if (FAST_MUL_EN = false) generate

    -- shift-and-add algorithm --
    multiplier_core: process(clk_i)
    begin
      if rising_edge(clk_i) then
        if (mul.start = '1') then -- start new multiplication
          mul.prod(63 downto 32) <= (others => '0');
          mul.prod(31 downto 00) <= rs1_i;
        elsif (ctrl.state = S_BUSY) or (ctrl.state = S_DONE) then -- processing step or sign-finalization step
          mul.prod(63 downto 31) <= mul.add(32 downto 0);
          mul.prod(30 downto 00) <= mul.prod(31 downto 1);
        end if;
      end if;
    end process multiplier_core;

    -- multiply with 0/1 via addition --
    mul_update: process(mul, ctrl, rs2_i)
    begin
      if (mul.prod(0) = '1') then -- multiply with 1
        if (ctrl.state = S_DONE) and (ctrl.rs1_is_signed = '1') then -- for signed operations only: take care of negative weighted MSB -> multiply with -1
          mul.add <= std_ulogic_vector(unsigned(mul.p_sext & mul.prod(63 downto 32)) - unsigned((rs2_i(rs2_i'left) and ctrl.rs2_is_signed) & rs2_i));
        else -- multiply with +1
          mul.add <= std_ulogic_vector(unsigned(mul.p_sext & mul.prod(63 downto 32)) + unsigned((rs2_i(rs2_i'left) and ctrl.rs2_is_signed) & rs2_i));
        end if;
      else -- multiply with 0
        mul.add <= mul.p_sext & mul.prod(63 downto 32);
      end if;
    end process mul_update;

    -- product sign extension bit --
    mul.p_sext <= mul.prod(mul.prod'left) and ctrl.rs2_is_signed;

  end generate; -- /multiplier_core_serial

  -- no serial multiplier --
  multiplier_core_serial_none:
  if (FAST_MUL_EN = true) generate
    mul.add    <= (others => '0');
    mul.p_sext <= '0';
  end generate;


  -- Divider Core (unsigned) - Iterative ----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  divider_core_serial:
  if (DIVISION_EN = true) generate

    -- restoring division algorithm --
    divider_core: process(clk_i)
    begin
      if rising_edge(clk_i) then
        if (div.start = '1') then -- start new division
          if ((rs1_i(rs1_i'left) and ctrl.rs1_is_signed) = '1') then -- signed division?
            div.quotient <= std_ulogic_vector(0 - unsigned(rs1_i)); -- make positive
          else
            div.quotient <= rs1_i;
          end if;
          div.remainder <= (others => '0');
        elsif (ctrl.state = S_BUSY) or (ctrl.state = S_DONE) then -- running?
          div.quotient <= div.quotient(30 downto 0) & (not div.sub(32));
          if (div.sub(32) = '0') then -- implicit shift
            div.remainder <= div.sub(31 downto 0);
          else -- underflow: restore and explicit shift
            div.remainder <= div.remainder(30 downto 0) & div.quotient(31);
          end if;
        end if;
      end if;
    end process divider_core;

    -- try another subtraction (and shift) --
    div.sub <= std_ulogic_vector(unsigned('0' & div.remainder(30 downto 0) & div.quotient(31)) - unsigned('0' & ctrl.rs2_abs));

    -- result and sign compensation --
    div.res_u <= div.quotient when (ctrl.cp_op = cp_op_div_c) or (ctrl.cp_op = cp_op_divu_c) else div.remainder;
    div.res   <= std_ulogic_vector(0 - unsigned(div.res_u)) when (div.sign_mod = '1') else div.res_u;

  end generate; -- /divider_core_serial

  -- no divider --
  divider_core_serial_none:
  if (DIVISION_EN = false) generate
    div.remainder <= (others => '0');
    div.quotient  <= (others => '0');
    div.sub       <= (others => '0');
    div.res_u     <= (others => '0');
    div.res       <= (others => '0');
  end generate;


  -- Data Output ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  operation_result: process(ctrl, mul, div)
  begin
    res_o <= (others => '0'); -- default
    if (ctrl.out_en = '1') then
      case ctrl.cp_op_ff is
        when cp_op_mul_c =>
          res_o <= mul.prod(31 downto 00);
        when cp_op_mulh_c | cp_op_mulhsu_c | cp_op_mulhu_c =>
          res_o <= mul.prod(63 downto 32);
        when others => -- cp_op_div_c | cp_op_rem_c | cp_op_divu_c | cp_op_remu_c
          res_o <= div.res;
      end case;
    end if;
  end process operation_result;


end neorv32_cpu_cp_muldiv_rtl;
