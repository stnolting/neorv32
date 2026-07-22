-- ================================================================================ --
-- NEORV32 CPU - ALU Shifter (RISC-V Base ISA)                                      --
-- -------------------------------------------------------------------------------- --
-- FAST_SHIFT_EN = false -> Use bit-serial shifter architecture (small but slow)    --
-- FAST_SHIFT_EN = true  -> Use barrel shifter architecture (large but fast)        --
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

entity neorv32_cpu_alu_shifter is
  generic (
    FAST_SHIFT_EN : boolean -- implement fast but large barrel shifter
  );
  port (
    -- global control --
    clk_i   : in  std_ulogic; -- global clock, rising edge
    rstn_i  : in  std_ulogic; -- global reset, low-active, async
    ctrl_i  : in  ctrl_bus_t; -- main control bus
    -- data input --
    rs1_i   : in  std_ulogic_vector(31 downto 0); -- rf source 1
    shamt_i : in  std_ulogic_vector(4 downto 0);  -- shift amount
    -- result and status --
    res_o   : out std_ulogic_vector(31 downto 0); -- operation result
    valid_o : out std_ulogic                      -- data output valid
  );
end entity;

architecture neorv32_cpu_alu_shifter_rtl of neorv32_cpu_alu_shifter is

  -- instruction decode --
  signal valid_cmd : std_ulogic;

  -- shifter --
  signal busy, sgn, done, oe : std_ulogic;
  signal cnt  : std_ulogic_vector(4 downto 0);
  signal sreg : std_ulogic_vector(31 downto 0);

  -- barrel shifter layers --
  type level_t is array (5 downto 0) of std_ulogic_vector(31 downto 0);
  signal lvl : level_t;

begin

  -- Valid Instruction? ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  valid_cmd <= '1' when (ctrl_i.alu_cp_alu = '1') and (
    ((ctrl_i.ir_funct3 = funct3_sll_c) and (ctrl_i.ir_funct12(11 downto 5) = "0000000")) or -- SLL[I]
    ((ctrl_i.ir_funct3 = funct3_sr_c)  and (ctrl_i.ir_funct12(11 downto 5) = "0000000")) or -- SRL[I]
    ((ctrl_i.ir_funct3 = funct3_sr_c)  and (ctrl_i.ir_funct12(11 downto 5) = "0100000"))) else '0'; -- SRA[I]

  -- Serial Shifter (small but slow) --------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  serial_shifter:
  if not FAST_SHIFT_EN generate

    shifter: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        busy <= '0';
        oe   <= '0';
        cnt  <= (others => '0');
        sreg <= (others => '0');
      elsif rising_edge(clk_i) then
        -- arbitration --
        if (valid_cmd = '1') then
          busy <= '1';
        elsif (done = '1') or (ctrl_i.cpu_trap = '1') then -- abort on trap
          busy <= '0';
        end if;
        oe <= busy and done;
        -- shift register --
        if (valid_cmd = '1') then -- trigger new operation
          cnt  <= shamt_i;
          sreg <= rs1_i;
        elsif (or_reduce_f(cnt) = '1') then -- operation in progress
          cnt <= std_ulogic_vector(unsigned(cnt) - 1);
          if (ctrl_i.ir_funct3(2) = '0') then -- shift left logical
            sreg <= sreg(sreg'left-1 downto 0) & '0';
          else -- shift right (arithmetical)
            sreg <= (sreg(sreg'left) and ctrl_i.ir_funct12(10)) & sreg(sreg'left downto 1);
          end if;
        end if;
      end if;
    end process;

    -- shift control --
    done    <= not or_reduce_f(cnt(cnt'left downto 1));
    valid_o <= busy and done;
    res_o   <= sreg when (oe = '1') else (others => '0');

    -- unused --
    lvl <= (others => (others => '0'));
    sgn <= '0';

  end generate;

  -- Barrel Shifter (fast but large) --------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  barrel_shifter:
  if FAST_SHIFT_EN generate

    -- input layer: convert left shifts to right shifts using bit-reversal --
    lvl(0) <= bit_rev_f(rs1_i) when (ctrl_i.ir_funct3(2) = '0') else rs1_i;
    sgn    <= rs1_i(31) and ctrl_i.ir_funct12(10); -- sign (extension) for arithmetic shifts

    -- mux layers: right-shifts only --
    barrel_shifter_gen:
    for i in 0 to 4 generate
      lvl(i+1)(31 downto 32-2**i) <= (others => sgn)        when (shamt_i(i) = '1') else lvl(i)(31 downto 32-2**i);
      lvl(i+1)(31-2**i downto 0)  <= lvl(i)(31 downto 2**i) when (shamt_i(i) = '1') else lvl(i)(31-2**i downto 0);
    end generate;

    -- register layer --
    pipe_reg: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        oe   <= '0';
        sreg <= (others => '0');
      elsif rising_edge(clk_i) then
        oe   <= valid_cmd;
        sreg <= lvl(5);
      end if;
    end process;

    -- output layer: re-convert original left shifts and result gate --
    res_o   <= (others => '0') when (oe = '0') else bit_rev_f(sreg) when (ctrl_i.ir_funct3(2) = '0') else sreg;
    valid_o <= valid_cmd;

    -- unused --
    busy <= '0';
    done <= '0';
    cnt  <= (others => '0');

  end generate;

end architecture;
