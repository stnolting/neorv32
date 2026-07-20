-- ================================================================================ --
-- NEORV32 CPU - ALU Integer Multiply/Divide Unit (RISC-V M/Zmmul ISA Extensions)   --
-- -------------------------------------------------------------------------------- --
-- Multiplier core (signed/unsigned) uses serial add-and-shift algorithm.           --
-- Multiplications can be mapped to DSP blocks (faster!) when FAST_MUL_EN = true.   --
-- Divider core (unsigned-only; pre and post sign-compensation logic) uses serial   --
-- restoring serial algorithm.                                                      --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cpu_alu_muldiv is
  generic (
    FAST_MUL_EN : boolean; -- use DSPs for faster multiplication
    DIVISION_EN : boolean  -- implement divider hardware
  );
  port (
    -- global control --
    clk_i   : in  std_ulogic; -- global clock, rising edge
    rstn_i  : in  std_ulogic; -- global reset, low-active, async
    ctrl_i  : in  ctrl_bus_t; -- main control bus
    -- data input --
    rs1_i   : in  std_ulogic_vector(31 downto 0); -- rf source 1
    rs2_i   : in  std_ulogic_vector(31 downto 0); -- rf source 2
    -- result and status --
    res_o   : out std_ulogic_vector(31 downto 0); -- operation result
    valid_o : out std_ulogic                      -- data output valid
  );
end entity;

architecture neorv32_cpu_alu_muldiv_rtl of neorv32_cpu_alu_muldiv is

  -- absolute value --
  function abs32_f(input: std_ulogic_vector; is_signed: std_ulogic) return std_ulogic_vector is
    variable res_v : std_ulogic_vector(31 downto 0);
  begin
    if (input(input'left) = '1') and (is_signed = '1') then
      res_v := std_ulogic_vector(0 - unsigned(input));
    else
      res_v := input;
    end if;
    return res_v;
  end function;

  -- operations --
  constant op_mul_c    : std_ulogic_vector(2 downto 0) := "000";
  constant op_mulh_c   : std_ulogic_vector(2 downto 0) := "001";
  constant op_mulhsu_c : std_ulogic_vector(2 downto 0) := "010";
  constant op_mulhu_c  : std_ulogic_vector(2 downto 0) := "011";
  constant op_div_c    : std_ulogic_vector(2 downto 0) := "100";
  constant op_divu_c   : std_ulogic_vector(2 downto 0) := "101";
  constant op_rem_c    : std_ulogic_vector(2 downto 0) := "110";
  constant op_remu_c   : std_ulogic_vector(2 downto 0) := "111";

  -- instruction decode --
  signal valid_cmd : std_ulogic;

  -- controller --
  type state_t is (S_IDLE, S_BUSY, S_DONE);
  type ctrl_t is record
    state  : state_t;
    cnt    : std_ulogic_vector(4 downto 0);
    out_en : std_ulogic;
  end record;
  signal ctrl : ctrl_t; -- FSM
  signal rs1_signed, rs2_signed : std_ulogic;

  -- divider core --
  signal div_start : std_ulogic;
  signal div_divi  : std_ulogic_vector(31 downto 0);
  signal div_quot  : std_ulogic_vector(31 downto 0);
  signal div_rema  : std_ulogic_vector(31 downto 0);
  signal div_sign  : std_ulogic;
  signal div_sub   : std_ulogic_vector(32 downto 0);
  signal div_res_u : std_ulogic_vector(31 downto 0);
  signal div_res   : std_ulogic_vector(31 downto 0);

  -- multiplier core --
  signal mul_start : std_ulogic;
  signal mul_res   : std_ulogic_vector(63 downto 0);
  signal mul_add   : std_ulogic_vector(32 downto 0);

begin

  -- Valid Instruction? ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  valid_cmd <= '1' when (ctrl_i.alu_cp_alu = '1') and
    (ctrl_i.ir_opcode(5) = '1') and (ctrl_i.ir_funct12(11 downto 5) = "0000001") and
    ((ctrl_i.ir_funct3(2) = '0') or ((ctrl_i.ir_funct3(2) = '1') and DIVISION_EN)) else '0';


  -- Co-Processor Controller ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  control: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl.state  <= S_IDLE;
      ctrl.cnt    <= (others => '0');
      ctrl.out_en <= '0';
    elsif rising_edge(clk_i) then
      -- defaults --
      ctrl.out_en <= '0';
      ctrl.cnt    <= std_ulogic_vector(to_unsigned(30, 5)); -- cycle counter initialization

      -- fsm --
      case ctrl.state is

        when S_BUSY => -- processing
        -- ------------------------------------------------------------
          ctrl.cnt <= std_ulogic_vector(unsigned(ctrl.cnt) - 1);
          if (ctrl_i.cpu_trap = '1') then -- abort on trap
            ctrl.state <= S_IDLE;
          elsif (or_reduce_f(ctrl.cnt) = '0') then -- processing done
            ctrl.state <= S_DONE;
          end if;

        when S_DONE => -- S_DONE: final step / enable output for one cycle
        -- ------------------------------------------------------------
          ctrl.out_en <= '1';
          ctrl.state  <= S_IDLE;

        when others => -- S_IDLE: wait for start signal
        -- ------------------------------------------------------------
          if (valid_cmd = '1') then -- trigger new operation
            if (ctrl_i.ir_funct3(2) = '0') and FAST_MUL_EN then -- is fast multiplication?
              ctrl.state <= S_DONE;
            else -- serial division or serial multiplication
              ctrl.state <= S_BUSY;
            end if;
          end if;

      end case;
    end if;
  end process;

  -- done? assert one cycle before actual data output --
  valid_o <= '1' when (ctrl.state = S_DONE) else '0';

  -- input operands treated as signed? --
  rs1_signed <= '1' when (ctrl_i.ir_funct3 = op_mulh_c) or (ctrl_i.ir_funct3 = op_mulhsu_c) or
                         (ctrl_i.ir_funct3 = op_div_c)  or (ctrl_i.ir_funct3 = op_rem_c) else '0';
  rs2_signed <= '1' when (ctrl_i.ir_funct3 = op_mulh_c) or
                         (ctrl_i.ir_funct3 = op_div_c)  or (ctrl_i.ir_funct3 = op_rem_c) else '0';

  -- operation trigger --
  mul_start <= '1' when (valid_cmd = '1') and (ctrl_i.ir_funct3(2) = '0') else '0';
  div_start <= '1' when (valid_cmd = '1') and (ctrl_i.ir_funct3(2) = '1') else '0';


  -- Multiplier Core - Full Parallel --------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  multiplier_core_parallel:
  if FAST_MUL_EN generate
    multiplier_inst: entity neorv32.neorv32_prim_mul
    generic map (
      DWIDTH => 32
    )
    port map (
      clk_i    => clk_i,
      rstn_i   => rstn_i,
      en_i     => mul_start,
      opa_i    => rs1_i,
      opa_sn_i => rs1_signed,
      opb_i    => rs2_i,
      opb_sn_i => rs2_signed,
      res_o    => mul_res
    );
    mul_add <= (others => '0'); -- unused
  end generate;


  -- Multiplier Core - Iterative ------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  multiplier_core_serial:
  if not FAST_MUL_EN generate

    -- shift-and-add algorithm --
    multiplier_core: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        mul_res <= (others => '0');
      elsif rising_edge(clk_i) then
        if (mul_start = '1') then -- start new multiplication
          mul_res(63 downto 32) <= (others => '0');
          mul_res(31 downto 0)  <= rs1_i;
        elsif (ctrl.state /= S_IDLE) and (ctrl_i.ir_funct3(2) = '0') then -- MUL processing steps or sign-finalization step
          mul_res(63 downto 31) <= mul_add(32 downto 0);
          mul_res(30 downto 0)  <= mul_res(31 downto 1);
        end if;
      end if;
    end process;

    -- multiply by 0/-1/+1 via shift and subtraction/addition --
    mul_update: process(mul_res, ctrl, rs2_i, rs1_signed, rs2_signed)
      variable sign_v : std_ulogic;
      variable opb_v  : unsigned(32 downto 0);
    begin
      sign_v := mul_res(mul_res'left) and rs2_signed; -- product sign extension bit
      opb_v  := unsigned((rs2_i(rs2_i'left) and rs2_signed) & rs2_i);
      if (mul_res(0) = '1') then -- multiply by 1
        if (ctrl.state = S_DONE) and (rs1_signed = '1') then -- take care of negative weighted MSB -> multiply by -1
          mul_add <= std_ulogic_vector(unsigned(sign_v & mul_res(63 downto 32)) - opb_v);
        else -- multiply by +1
          mul_add <= std_ulogic_vector(unsigned(sign_v & mul_res(63 downto 32)) + opb_v);
        end if;
      else -- multiply by 0
        mul_add <= sign_v & mul_res(63 downto 32);
      end if;
    end process;

  end generate;


  -- Divider Core - Iterative ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  divider_core_serial:
  if DIVISION_EN generate

    -- unsigned restoring division algorithm --
    divider_core: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        div_divi <= (others => '0');
        div_quot <= (others => '0');
        div_rema <= (others => '0');
        div_sign <= '0';
      elsif rising_edge(clk_i) then
        if (div_start = '1') then -- start new division
          div_divi <= abs32_f(rs2_i, rs2_signed);
          div_quot <= abs32_f(rs1_i, rs1_signed);
          div_rema <= (others => '0');
          case ctrl_i.ir_funct3(1 downto 0) is -- check for result's sign compensation
            when "00"   => div_sign <= or_reduce_f(rs2_i) and (rs1_i(rs1_i'left) xor rs2_i(rs2_i'left)); -- signed div
            when "10"   => div_sign <= rs1_i(rs1_i'left); -- signed rem
            when others => div_sign <= '0';
          end case;
        elsif (ctrl.state = S_BUSY) or (ctrl.state = S_DONE) then -- running?
          div_quot <= div_quot(30 downto 0) & (not div_sub(32));
          if (div_sub(32) = '0') then
            div_rema <= div_sub(31 downto 0);
          else -- underflow: restore
            div_rema <= div_rema(30 downto 0) & div_quot(31);
          end if;
        end if;
      end if;
    end process;

    -- do another subtraction (and shift) --
    div_sub <= std_ulogic_vector(unsigned('0' & div_rema(30 downto 0) & div_quot(31)) - unsigned('0' & div_divi));

    -- result select and sign compensation --
    div_res_u <= div_quot when (ctrl_i.ir_funct3(2 downto 1) = op_div_c(2 downto 1)) else div_rema;
    div_res   <= std_ulogic_vector(0 - unsigned(div_res_u)) when (div_sign = '1') else div_res_u;

  end generate;

  -- no divider --
  divider_core_serial_none:
  if not DIVISION_EN generate
    div_divi  <= (others => '0');
    div_quot  <= (others => '0');
    div_rema  <= (others => '0');
    div_sign  <= '0';
    div_sub   <= (others => '0');
    div_res_u <= (others => '0');
    div_res   <= (others => '0');
  end generate;


  -- Data Output ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  operation_result: process(ctrl, ctrl_i.ir_funct3, mul_res, div_res)
  begin
    res_o <= (others => '0');
    if (ctrl.out_en = '1') then
      case ctrl_i.ir_funct3 is
        when op_mul_c =>
          res_o <= mul_res(31 downto 0);
        when op_mulh_c | op_mulhsu_c | op_mulhu_c =>
          res_o <= mul_res(63 downto 32);
        when others =>
          res_o <= div_res;
      end case;
    end if;
  end process;

end architecture;
