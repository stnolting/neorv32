-- #################################################################################################
-- # << NEORV32 - CPU Co-Processor: Integer Multiplier/Divider Unit (RISC-V "M" Extension) >>      #
-- # ********************************************************************************************* #
-- # Multiplier and Divider unit. Implements the RISC-V M CPU extension.                           #
-- #                                                                                               #
-- # Multiplier core (signed/unsigned) uses classical serial algorithm. Unit latency: 31+3 cycles  #
-- # Divider core (unsigned) uses classical serial algorithm. Unit latency: 32+4 cycles            #
-- #                                                                                               #
-- # Multiplications can be mapped to DSP blocks (faster!) when FAST_MUL_EN = true.                #
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

entity neorv32_cpu_cp_muldiv is
  generic (
    FAST_MUL_EN : boolean; -- use DSPs for faster multiplication
    DIVISION_EN : boolean  -- implement divider hardware
  );
  port (
    -- global control --
    clk_i   : in  std_ulogic; -- global clock, rising edge
    rstn_i  : in  std_ulogic; -- global reset, low-active, async
    ctrl_i  : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
    start_i : in  std_ulogic; -- trigger operation
    -- data input --
    rs1_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 1
    rs2_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 2
    -- result and status --
    res_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- operation result
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
  type state_t is (IDLE, DIV_PREPROCESS, PROCESSING, FINALIZE);
  signal state         : state_t;
  signal cnt           : std_ulogic_vector(4 downto 0);
  signal cp_op         : std_ulogic_vector(2 downto 0); -- operation to execute
  signal cp_op_ff      : std_ulogic_vector(2 downto 0); -- operation that was executed
  signal start_div     : std_ulogic;
  signal start_mul     : std_ulogic;
  signal operation     : std_ulogic;
  signal div_opy       : std_ulogic_vector(data_width_c-1 downto 0);
  signal rs1_is_signed : std_ulogic;
  signal rs2_is_signed : std_ulogic;
  signal opy_is_zero   : std_ulogic;
  signal div_res_corr  : std_ulogic;
  signal out_en        : std_ulogic;

  -- divider core --
  signal remainder        : std_ulogic_vector(data_width_c-1 downto 0);
  signal quotient         : std_ulogic_vector(data_width_c-1 downto 0);
  signal div_sub          : std_ulogic_vector(data_width_c   downto 0);
  signal div_sign_comp_in : std_ulogic_vector(data_width_c-1 downto 0);
  signal div_sign_comp    : std_ulogic_vector(data_width_c-1 downto 0);
  signal div_res          : std_ulogic_vector(data_width_c-1 downto 0);

  -- multiplier core --
  signal mul_product    : std_ulogic_vector(63 downto 0);
  signal mul_do_add     : std_ulogic_vector(data_width_c downto 0);
  signal mul_sign_cycle : std_ulogic;
  signal mul_p_sext     : std_ulogic;
  signal mul_op_x       : signed(32 downto 0); -- for using DSPs
  signal mul_op_y       : signed(32 downto 0); -- for using DSPs

begin

  -- Co-Processor Controller ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  coprocessor_ctrl: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      state        <= IDLE;
      div_opy      <= (others => def_rst_val_c);
      cnt          <= (others => def_rst_val_c);
      cp_op_ff     <= (others => def_rst_val_c);
      start_div    <= '0';
      out_en       <= '0';
      valid_o      <= '0';
      div_res_corr <= def_rst_val_c;
      opy_is_zero  <= def_rst_val_c;
    elsif rising_edge(clk_i) then
      -- defaults --
      start_div <= '0';
      out_en    <= '0';
      valid_o   <= '0';

      -- FSM --
      case state is

        when IDLE =>
          cp_op_ff <= cp_op;
          cnt      <= "11110";
          if (start_i = '1') then
            if (operation = '1') and (DIVISION_EN = true) then -- division
              start_div <= '1';
              state     <= DIV_PREPROCESS;
            else -- multiplication
              if (FAST_MUL_EN = true) then
                valid_o <= '1';
                state   <= FINALIZE;
              else
                state <= PROCESSING;
              end if;
            end if;
          end if;

        when DIV_PREPROCESS =>
          -- check relevant input signs --
          if (cp_op = cp_op_div_c) then -- result sign compensation for div?
            div_res_corr <= rs1_i(rs1_i'left) xor rs2_i(rs2_i'left);
          elsif (cp_op = cp_op_rem_c) then -- result sign compensation for rem?
            div_res_corr <= rs1_i(rs1_i'left);
          else
            div_res_corr <= '0';
          end if;
          -- divide by zero? --
          opy_is_zero <= not or_reduce_f(rs2_i); -- set if rs2 = 0
          -- abs(rs2) --
          if ((rs2_i(rs2_i'left) and rs2_is_signed) = '1') then -- signed division?
            div_opy <= std_ulogic_vector(0 - unsigned(rs2_i)); -- make positive
          else
            div_opy <= rs2_i;
          end if;
          --
          state <= PROCESSING;

        when PROCESSING =>
          cnt <= std_ulogic_vector(unsigned(cnt) - 1);
          if (cnt = "00000") then
            valid_o <= '1';
            state   <= FINALIZE;
          end if;

        when FINALIZE =>
          out_en <= '1';
          state  <= IDLE;

        when others =>
          state <= IDLE;
      end case;
    end if;
  end process coprocessor_ctrl;

  -- co-processor command --
  cp_op <= ctrl_i(ctrl_ir_funct3_2_c downto ctrl_ir_funct3_0_c);

  -- operation: 0=mul, 1=div --
  operation <= '1' when (cp_op(2) = '1') else '0';

  -- opx (rs1) signed? --
  rs1_is_signed <= '1' when (cp_op = cp_op_mulh_c) or (cp_op = cp_op_mulhsu_c) or (cp_op = cp_op_div_c) or (cp_op = cp_op_rem_c) else '0';

  -- opy (rs2) signed? --
  rs2_is_signed <= '1' when (cp_op = cp_op_mulh_c) or (cp_op = cp_op_div_c) or (cp_op = cp_op_rem_c) else '0';

  -- start MUL operation (do it fast!) --
  start_mul <= '1' when (state = IDLE) and (start_i = '1') and (operation = '0') else '0';


  -- Multiplier Core (signed/unsigned) ------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- iterative multiplication (bit-serial) --
  multiplier_core_serial:
  if (FAST_MUL_EN = false) generate
    multiplier_core: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        mul_product <= (others => def_rst_val_c);
      elsif rising_edge(clk_i) then
        if (start_mul = '1') then -- start new multiplication
          mul_product(63 downto 32) <= (others => '0');
          mul_product(31 downto 00) <= rs2_i;
        elsif (state = PROCESSING) or (state = FINALIZE) then -- processing step or sign-finalization step
          mul_product(63 downto 31) <= mul_do_add(32 downto 0);
          mul_product(30 downto 00) <= mul_product(31 downto 1);
        end if;
      end if;
    end process multiplier_core;
  end generate;

  -- parallel multiplication (using DSP blocks) --
  multiplier_core_dsp:
  if (FAST_MUL_EN = true) generate
    multiplier_core: process(clk_i)
      variable tmp_v : signed(65 downto 0);
    begin
      if rising_edge(clk_i) then
        if (start_mul = '1') then
          mul_op_x <= signed((rs1_i(rs1_i'left) and rs1_is_signed) & rs1_i);
          mul_op_y <= signed((rs2_i(rs2_i'left) and rs2_is_signed) & rs2_i);
        end if;
        tmp_v := mul_op_x * mul_op_y;
        mul_product <= std_ulogic_vector(tmp_v(63 downto 0));
        --mul_buf_ff  <= mul_op_x * mul_op_y;
        --mul_product <= std_ulogic_vector(mul_buf_ff(63 downto 0)); -- let the register balancing do the magic here
      end if;
    end process multiplier_core;
  end generate;

  -- do another addition (bit-serial) --
  mul_update: process(mul_product, mul_sign_cycle, mul_p_sext, rs1_is_signed, rs1_i)
  begin
    -- current bit of rs2_i to take care of --
    if (mul_product(0) = '1') then -- multiply with 1
      if (mul_sign_cycle = '1') then -- for signed operations only: take care of negative weighted MSB -> multiply with -1
        mul_do_add <= std_ulogic_vector(unsigned(mul_p_sext & mul_product(63 downto 32)) - unsigned((rs1_i(rs1_i'left) and rs1_is_signed) & rs1_i));
      else -- multiply with +1
        mul_do_add <= std_ulogic_vector(unsigned(mul_p_sext & mul_product(63 downto 32)) + unsigned((rs1_i(rs1_i'left) and rs1_is_signed) & rs1_i));
      end if;
    else -- multiply with 0
      mul_do_add <= mul_p_sext & mul_product(63 downto 32);
    end if;
  end process mul_update;

  -- sign control --
  mul_sign_cycle <= rs2_is_signed when (state = FINALIZE) else '0';
  mul_p_sext     <= mul_product(mul_product'left) and rs1_is_signed;


  -- Divider Core (unsigned) ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  divider_core_serial:
  if (DIVISION_EN = true) generate
    divider_core: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        quotient  <= (others => def_rst_val_c);
        remainder <= (others => def_rst_val_c);
      elsif rising_edge(clk_i) then
        if (start_div = '1') then -- start new division
          if ((rs1_i(rs1_i'left) and rs1_is_signed) = '1') then -- signed division?
            quotient <= std_ulogic_vector(0 - unsigned(rs1_i)); -- make positive
          else
            quotient <= rs1_i;
          end if;
          remainder <= (others => '0');
        elsif (state = PROCESSING) or (state = FINALIZE) then -- running?
          quotient <= quotient(30 downto 0) & (not div_sub(32));
          if (div_sub(32) = '0') then -- still overflowing
            remainder <= div_sub(31 downto 0);
          else -- underflow
            remainder <= remainder(30 downto 0) & quotient(31);
          end if;
        end if;
      end if;
    end process divider_core;

    -- try another subtraction --
    div_sub <= std_ulogic_vector(unsigned('0' & remainder(30 downto 0) & quotient(31)) - unsigned('0' & div_opy));

    -- result sign compensation --
    div_sign_comp_in <= quotient when (cp_op = cp_op_div_c) else remainder;
    div_sign_comp    <= std_ulogic_vector(0 - unsigned(div_sign_comp_in));
    div_res          <= div_sign_comp when (div_res_corr = '1') and (opy_is_zero = '0') else div_sign_comp_in;
  end generate;

  -- no divider --
  divider_core_serial_none:
  if (DIVISION_EN = false) generate
    remainder <= (others => '0');
    quotient  <= (others => '0');
    div_res   <= (others => '0');
  end generate;


  -- Data Output ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  operation_result: process(out_en, cp_op_ff, mul_product, div_res, quotient, opy_is_zero, rs1_i, remainder)
  begin
    if (out_en = '1') then
      case cp_op_ff is
        when cp_op_mul_c =>
          res_o <= mul_product(31 downto 00);
        when cp_op_mulh_c | cp_op_mulhsu_c | cp_op_mulhu_c =>
          res_o <= mul_product(63 downto 32);
        when cp_op_div_c =>
          res_o <= div_res;
        when cp_op_divu_c =>
          res_o <= quotient;
        when cp_op_rem_c =>
          if (opy_is_zero = '0') then
            res_o <= div_res;
          else
            res_o <= rs1_i;
          end if;
        when others => -- cp_op_remu_c
          res_o <= remainder;
      end case;
    else
      res_o <= (others => '0');
    end if;
  end process operation_result;


end neorv32_cpu_cp_muldiv_rtl;
