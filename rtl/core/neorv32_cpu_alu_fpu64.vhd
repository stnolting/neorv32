-- ================================================================================ --
-- NEORV32 CPU - ALU Single and Double-Precision FPU (RISC-V FD ISA Extensions)     --
-- -------------------------------------------------------------------------------- --
-- It's a top level entity that should be used to connect an FPU entity             --
-- not ascociated with neorv32 library.                                             --
-- Implimenting FPU this way allows the end user to make their own choice on        --
-- what comes first: size or performance.                                           --
--                                                                                  --
-- Included FPU is expected to be fully compatible with RISC-V FD instructions      --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) Stephan Nolting. All rights reserved.                              --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cpu_alu_fpu64 is
  port (
    -- global control --
    clk_i       : in  std_ulogic;                     -- global clock, rising edge
    rstn_i      : in  std_ulogic;                     -- global reset, low-active, async
    ctrl_i      : in  ctrl_bus_t;                     -- main control bus
    -- CSR interface --
    csr_we_i    : in  std_ulogic;                     -- write enable
    csr_addr_i  : in  std_ulogic_vector(1 downto 0);  -- address
    csr_wdata_i : in  std_ulogic_vector(31 downto 0); -- write data
    csr_rdata_o : out std_ulogic_vector(31 downto 0); -- read data
    -- data input --
    rs1_i       : in  std_ulogic_vector(63 downto 0); -- rf source 1
    rs2_i       : in  std_ulogic_vector(63 downto 0); -- rf source 2
    rs3_i       : in  std_ulogic_vector(63 downto 0); -- rf source 3
    -- result and status --
    res_o       : out std_ulogic_vector(63 downto 0); -- operation result
    valid_o     : out std_ulogic                      -- data output valid
  );
end entity;

architecture neorv32_cpu_alu_fpu64_rtl of neorv32_cpu_alu_fpu64 is

  -- FPU CSRs --
  signal csr_frm    : std_ulogic_vector(2 downto 0);
  signal csr_fflags : std_ulogic_vector(4 downto 0);
  signal fflags     : std_ulogic_vector(4 downto 0);
  signal fflag_hold : std_ulogic_vector(4 downto 0);

  signal fpu_op       : std_ulogic;
  signal op_ready     : std_ulogic;
  signal clear        : std_ulogic;
  signal res_i        : std_ulogic_vector(63 downto 0);
  signal res_hold     : std_ulogic_vector(63 downto 0);
  
  signal dec_fmadd    : std_ulogic;
  signal dec_fmsub    : std_ulogic;
  signal dec_fnmadd   : std_ulogic;
  signal dec_fnmsub   : std_ulogic;
  signal dec_fadd     : std_ulogic;
  signal dec_fsub     : std_ulogic;
  signal dec_fmul     : std_ulogic;
  signal dec_fdiv     : std_ulogic;
  signal dec_fsqrt    : std_ulogic;
  signal dec_fsgnj    : std_ulogic;
  signal dec_fcmp     : std_ulogic;
  signal dec_fmax     : std_ulogic;
  signal dec_fclass   : std_ulogic;
  signal dec_fmv_i2f  : std_ulogic;
  signal dec_fmv_f2i  : std_ulogic;
  signal dec_fcvt_f2f : std_ulogic;
  signal dec_fcvt_i2f : std_ulogic;
  signal dec_fcvt_f2i : std_ulogic;
  signal dec_fcvt_op  : std_ulogic_vector(1 downto 0);
  
  signal dec_fmt      : std_ulogic_vector(1 downto 0);
  signal dec_rm       : std_ulogic_vector(2 downto 0);

begin

  -- write access --
  csr_write: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      csr_frm    <= (others => '0');
      csr_fflags <= (others => '0');
    elsif rising_edge(clk_i) then
      if (csr_we_i = '1') then
        -- exception flags --
        if (csr_addr_i = csr_fflags_c(1 downto 0)) then
          csr_fflags <= csr_wdata_i(4 downto 0);
        end if;
        -- rounding mode --
        if (csr_addr_i = csr_frm_c(1 downto 0)) then
          csr_frm <= csr_wdata_i(2 downto 0);
        end if;
        -- control/status (frm & fflags) --
        if (csr_addr_i = csr_fcsr_c(1 downto 0)) then
          csr_frm    <= csr_wdata_i(7 downto 5);
          csr_fflags <= csr_wdata_i(4 downto 0);
        end if;
      elsif fpu_op = '1' then -- auto-update ("accumulate" flags)
        fflags     <= fflag_hold;
        --csr_fflags <= fflag_hold;
        csr_fflags <= csr_fflags or fflag_hold;
      end if;
    end if;
  end process;

  -- read access --
  csr_read: process(csr_addr_i, csr_fflags, csr_frm)
  begin
    csr_rdata_o <= (others => '0'); -- default
    case csr_addr_i is
      when "01"   => csr_rdata_o(4 downto 0) <= csr_fflags; -- exception flags
      when "10"   => csr_rdata_o(2 downto 0) <= csr_frm; -- rounding mode
      when "11"   => csr_rdata_o(7 downto 0) <= csr_frm & csr_fflags; -- control/status (frm & fflags)
      when others => null;
    end case;
  end process;

  fpu_op    <= '1' when ctrl_i.rf_d_cvt = '1' or ctrl_i.rf_d_alu = '1' or ctrl_i.rf_d_int = '1' else
               '0' when ctrl_i.rf_wb_en = '1' else
               '0' when rstn_i = '0' else fpu_op;
  clear     <= not fpu_op;

  valid_o   <= op_ready when fpu_op = '1' else '0';
  
  process(clk_i, rstn_i)
  begin
    if rstn_i = '0' then
      res_hold    <= (others => '0');
--      fflags      <= (others => '0');
    elsif rising_edge(clk_i) then
      if fpu_op = '1' then
        res_hold  <= res_i;
--        fflags    <= fflag_hold;
      else
        res_hold  <= (others => '0');
      end if;
    end if;
  end process;

  --res_hold  <= res_i when op_ready = '1' and fpu_op = '1' else (others => '0');

  res_o <= res_hold;

  dec_fmadd   <= '1' when ctrl_i.ir_opcode = "1000011" and
                          fpu_op = '1' else '0';
  dec_fmsub   <= '1' when ctrl_i.ir_opcode = "1000111" and
                          fpu_op = '1' else '0';
  dec_fnmsub  <= '1' when ctrl_i.ir_opcode = "1001011" and
                          fpu_op = '1' else '0';
  dec_fnmadd  <= '1' when ctrl_i.ir_opcode = "1001111" and
                          fpu_op = '1' else '0';
  dec_fadd  <= '1' when ctrl_i.ir_opcode = "1010011" and
                        ctrl_i.ir_funct12(11 downto 6) = "000000" and
                        fpu_op = '1' else '0';
  dec_fsub  <= '1' when ctrl_i.ir_opcode = "1010011" and
                        ctrl_i.ir_funct12(11 downto 6) = "000010" and
                        fpu_op = '1' else '0';
  dec_fmul  <= '1' when ctrl_i.ir_opcode = "1010011" and
                        ctrl_i.ir_funct12(11 downto 6) = "000100" and
                        fpu_op = '1' else '0';
  dec_fdiv  <= '1' when ctrl_i.ir_opcode = "1010011" and
                        ctrl_i.ir_funct12(11 downto 6) = "000110" and
                        fpu_op = '1' else '0';
  dec_fsqrt <= '1' when ctrl_i.ir_opcode = "1010011" and
                        ctrl_i.ir_funct12(11 downto 6) = "010110" and
                        fpu_op = '1' else '0';
  dec_fsgnj <= '1' when ctrl_i.ir_opcode = "1010011" and
                        ctrl_i.ir_funct12(11 downto 6) = "001000" and
                        fpu_op = '1' else '0';
  dec_fcmp  <= '1' when ctrl_i.ir_opcode = "1010011" and
                        ctrl_i.ir_funct12(11 downto 6) = "101000" and
                        fpu_op = '1' else '0';
  dec_fmax  <= '1' when ctrl_i.ir_opcode = "1010011" and
                        ctrl_i.ir_funct12(11 downto 6) = "001010" and
                        fpu_op = '1' else '0';
  dec_fclass<= '1' when ctrl_i.ir_opcode = "1010011" and
                        ctrl_i.ir_funct12(11 downto 6) = "111000" and
                        ctrl_i.ir_funct3(0) = '1' and
                        fpu_op = '1' else '0';
  dec_fmv_i2f   <= '1' when ctrl_i.ir_opcode = "1010011" and
                            ctrl_i.ir_funct12(11 downto 6) = "111100" and
                            ctrl_i.ir_funct3(0) = '0' and
                            fpu_op = '1' else '0';
  dec_fmv_f2i   <= '1' when ctrl_i.ir_opcode = "1010011" and
                            ctrl_i.ir_funct12(11 downto 6) = "111000" and
                            fpu_op = '1' else '0';
  dec_fcvt_f2f  <= '1' when ctrl_i.ir_opcode = "1010011" and
                            ctrl_i.ir_funct12(11 downto 6) = "010000" and
                            fpu_op = '1' else '0';
  dec_fcvt_i2f  <= '1' when ctrl_i.ir_opcode = "1010011" and
                            ctrl_i.ir_funct12(11 downto 6) = "110100" and
                            fpu_op = '1' else '0';
  dec_fcvt_f2i  <= '1' when ctrl_i.ir_opcode = "1010011" and
                            ctrl_i.ir_funct12(11 downto 6) = "110000" and
                            fpu_op = '1' else '0';

  dec_fcvt_op   <= ctrl_i.rf_rs2(1 downto 0);

  dec_fmt(1)<= '0';
  dec_fmt(0)<= ctrl_i.ir_funct12(5);
  
  dec_rm    <= ctrl_i.ir_funct3 when ctrl_i.ir_opcode = "1010011" and
                                     (ctrl_i.ir_funct12(11 downto 6) = "001000" or
                                     ctrl_i.ir_funct12(11 downto 6) = "001010" or
                                     ctrl_i.ir_funct12(11 downto 6) = "101000" or
                                     ctrl_i.ir_funct12(11 downto 6) = "111000" or
                                     ctrl_i.ir_funct12(11 downto 6) = "111100") else csr_frm; 

  neorv32_cpu_alu_fpu64_inst: entity work.fpu_unit
  port map(
    rstn_i    => rstn_i,
    clk_i     => clk_i,
    rs1       => rs1_i,
    rs2       => rs2_i,
    rs3       => rs3_i,
    fmadd     => dec_fmadd,
    fmsub     => dec_fmsub,
    fnmadd    => dec_fnmadd,
    fnmsub    => dec_fnmsub,
    fadd      => dec_fadd,
    fsub      => dec_fsub,
    fmul      => dec_fmul,
    fdiv      => dec_fdiv,
    fsqrt     => dec_fsqrt,
    fsgnj     => dec_fsgnj,
    fcmp      => dec_fcmp,
    fmax      => dec_fmax,
    fclass    => dec_fclass,
    fmv_i2f   => dec_fmv_i2f,
    fmv_f2i   => dec_fmv_f2i,
    fcvt_f2f  => dec_fcvt_f2f,
    fcvt_i2f  => dec_fcvt_i2f,
    fcvt_f2i  => dec_fcvt_f2i,
    fcvt_op   => dec_fcvt_op,
    fmt       => dec_fmt,
    rm        => dec_rm,
    res       => res_i,
    flags     => fflag_hold,
    ready     => op_ready,
    clear     => '0'
  );

end architecture;
