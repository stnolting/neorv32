-- #################################################################################################
-- # << NEORV32 - CPU Co-Processor: MULDIV unit >>                                                 #
-- # ********************************************************************************************* #
-- # Multiplier and Divider unit. Implements the RISC-V RV32-M CPU extension.                      #
-- # Multiplier core (signed/unsigned) uses serial algorithm. -> 32+8 cycles latency               #
-- # Divider core (unsigned) uses serial algorithm. -> 32+8 cycles latency                         #
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

entity neorv32_cpu_cp_muldiv is
  port (
    -- global control --
    clk_i   : in  std_ulogic; -- global clock, rising edge
    rstn_i  : in  std_ulogic; -- global reset, low-active, async
    ctrl_i  : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
    -- data input --
    rs1_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 1
    rs2_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 2
    -- result and status --
    res_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- operation result
    valid_o : out std_ulogic -- data output valid
  );
end neorv32_cpu_cp_muldiv;

architecture neorv32_cpu_cp_muldiv_rtl of neorv32_cpu_cp_muldiv is

  -- controller --
  type state_t is (IDLE, DECODE, INIT_OPX, INIT_OPY, PROCESSING, FINALIZE, COMPLETED);
  signal state         : state_t;
  signal cnt           : std_ulogic_vector(4 downto 0);
  signal cp_op         : std_ulogic_vector(2 downto 0); -- operation to execute
  signal start         : std_ulogic;
  signal operation     : std_ulogic;
  signal opx, opy      : std_ulogic_vector(data_width_c-1 downto 0); -- input operands
  signal opx_is_signed : std_ulogic;
  signal opy_is_signed : std_ulogic;
  signal div_res_corr  : std_ulogic;

  -- divider core --
  signal remainder        : std_ulogic_vector(data_width_c-1 downto 0);
  signal quotient         : std_ulogic_vector(data_width_c-1 downto 0);
  signal div_sub          : std_ulogic_vector(data_width_c   downto 0);
  signal div_sign_comp_in : std_ulogic_vector(data_width_c-1 downto 0);
  signal div_sign_comp    : std_ulogic_vector(data_width_c-1 downto 0);
  signal div_res          : std_ulogic_vector(data_width_c-1 downto 0);

  -- multiplier core --
  signal mul_product    : std_ulogic_vector(63 downto 0);
  signal mul_do_add     : std_ulogic_vector(32 downto 0);
  signal mul_sign_cycle : std_ulogic;
  signal mul_p_sext     : std_ulogic;

begin

  -- Co-Processor Controller ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  coprocessor_ctrl: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      state        <= IDLE;
      cp_op        <= (others => '0');
      opx          <= (others => '0');
      opy          <= (others => '0');
      cnt          <= (others => '0');
      start        <= '0';
      valid_o      <= '0';
      div_res_corr <= '0';
    elsif rising_edge(clk_i) then
      -- defaults --
      start   <= '0';
      valid_o <= '0';

      -- FSM --
      case state is
        when IDLE =>
          opx   <= rs1_i;
          opy   <= rs2_i;
          if (ctrl_i(ctrl_cp_use_c) = '1') and (ctrl_i(ctrl_cp_id_msb_c downto ctrl_cp_id_lsb_c) = cp_sel_muldiv_c) then
            cp_op <= ctrl_i(ctrl_cp_cmd2_c downto ctrl_cp_cmd0_c);
            state <= DECODE;
          end if;

        when DECODE =>
          if (cp_op = cp_op_div_c) or (cp_op = cp_op_rem_c) then -- result sign compensation for div?
            div_res_corr <= opx(opx'left) xor opy(opy'left);
          else
            div_res_corr <= '0';
          end if;
          cnt <= "11111";
          if (operation = '1') then -- division
            state <= INIT_OPX;
          else -- multiplication
            start <= '1';
            state <= PROCESSING;
          end if;

        when INIT_OPX =>
          if ((opx(opx'left) and opx_is_signed) = '1') then -- signed division?
            opx <= div_sign_comp; -- make positive
          end if;
          state <= INIT_OPY;

        when INIT_OPY =>
          start <= '1';
          if ((opy(opy'left) and opy_is_signed) = '1') then -- signed division?
            opy <= div_sign_comp; -- make positive
          end if;
          state <= PROCESSING;

        when PROCESSING =>
          cnt <= std_ulogic_vector(unsigned(cnt) - 1);
          if (cnt = "00000") then
            state <= FINALIZE;
          end if;

        when FINALIZE =>
          state <= COMPLETED;

        when COMPLETED =>
          valid_o <= '1';
          state   <= IDLE;
      end case;
    end if;
  end process coprocessor_ctrl;

  -- operation --
  operation <= '1' when (cp_op = cp_op_div_c) or (cp_op = cp_op_divu_c) or (cp_op = cp_op_rem_c) or (cp_op = cp_op_remu_c) else '0';

  -- opx (rs1) signed? --
  opx_is_signed <= '1' when (cp_op = cp_op_mulh_c) or (cp_op = cp_op_mulhsu_c) or (cp_op = cp_op_div_c) or (cp_op = cp_op_rem_c) else '0';

  -- opy (rs2) signed? --
  opy_is_signed <= '1' when (cp_op = cp_op_mulh_c) or (cp_op = cp_op_div_c) or (cp_op = cp_op_rem_c) else '0';


  -- Multiplier Core ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  multiplier_core: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (start = '1') then -- start new multiplication
        mul_product(63 downto 32) <= (others => (opy(opy'left) and opy_is_signed)); -- sign extension
        mul_product(31 downto 00) <= opy;
      elsif ((state = PROCESSING) or (state = FINALIZE)) and (operation = '0') then
        mul_product(63 downto 31) <= mul_do_add(32 downto 0);
        mul_product(30 downto 00) <= mul_product(31 downto 1);
      end if;
    end if;
  end process multiplier_core;

  -- MUL: do another addition --
  mul_update: process(mul_product, mul_sign_cycle, mul_p_sext, opy_is_signed, opx)
  begin
    if (mul_product(0) = '1') then
      if (mul_sign_cycle = '1') then -- for signed operation only: take care of negative weighted MSB
        mul_do_add <= std_ulogic_vector(unsigned(mul_p_sext & mul_product(63 downto 32)) - unsigned((opx(opy'left) and opx_is_signed) & opx));
      else
        mul_do_add <= std_ulogic_vector(unsigned(mul_p_sext & mul_product(63 downto 32)) + unsigned((opx(opy'left) and opx_is_signed) & opx));
      end if;
    else
      mul_do_add <= mul_p_sext & mul_product(63 downto 32);
    end if;
  end process mul_update;

  -- sign control --
  mul_sign_cycle <= opy_is_signed when (state = FINALIZE) else '0';
  mul_p_sext     <= mul_product(mul_product'left) and opx_is_signed;


  -- Divider Core ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  divider_core: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (start = '1') then -- start new division
        quotient  <= opx;
        remainder <= (others => '0');
      elsif ((state = PROCESSING) or (state = FINALIZE)) and (operation = '1') then -- running?
        quotient <= quotient(30 downto 0) & (not div_sub(32));
        if (div_sub(32) = '0') then -- still overflowing
          remainder <= div_sub(31 downto 0);
        else -- underflow
          remainder <= remainder(30 downto 0) & quotient(31);
        end if;
      end if;
    end if;
  end process divider_core;

  -- DIV: try another subtraction --
  div_sub <= std_ulogic_vector(unsigned('0' & remainder(30 downto 0) & quotient(31)) - unsigned('0' & opy));

  -- Div sign compensation --
  div_sign_comp_in <= opx when (state = INIT_OPX) else
                      opy when (state = INIT_OPY) else
                      quotient when ((cp_op = cp_op_div_c) or (cp_op = cp_op_divu_c)) else remainder;
  div_sign_comp <= std_ulogic_vector(0 - unsigned(div_sign_comp_in));

  -- result sign correction --
  div_res <= div_sign_comp when (div_res_corr = '1') else div_sign_comp_in;


  -- Data Output ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  operation_result: process(clk_i)
  begin
    if rising_edge(clk_i) then
      case cp_op is
        when cp_op_mul_c =>
          res_o <= mul_product(31 downto 00);
        when cp_op_mulh_c | cp_op_mulhsu_c | cp_op_mulhu_c =>
          res_o <= mul_product(63 downto 32);
        when cp_op_div_c =>
          res_o <= div_res;
        when cp_op_divu_c =>
          res_o <= quotient;
        when cp_op_rem_c =>
          res_o <= div_res;
        when cp_op_remu_c =>
          res_o <= remainder;
        when others => -- undefined
          res_o <= (others => '0');
      end case;
    end if;
  end process operation_result;


end neorv32_cpu_cp_muldiv_rtl;
