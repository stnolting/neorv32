-- #################################################################################################
-- # << NEORV32 - CPU Data Register File >>                                                        #
-- # ********************************************************************************************* #
-- # General purpose data register file. 32 entries for normal mode (I), 16 entries for embedded   #
-- # mode (E) when RISC-V "E" extension is enabled. Register zero (r0) is a normal physical        #
-- # registers, that has to be initialized to zero by the CPU control system. For normal           #
-- # operations r0 cannot be written. The register file uses synchronous reads. Hence it can be    #
-- # mapped to FPGA block RAM.                                                                     #
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

entity neorv32_cpu_regfile is
  generic (
    CPU_EXTENSION_RISCV_E : boolean := false -- implement embedded RF extension?
  );
  port (
    -- global control --
    clk_i  : in  std_ulogic; -- global clock, rising edge
    ctrl_i : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
    -- data input --
    mem_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- memory read data
    alu_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- ALU result
    csr_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- CSR read data
    pc_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- current pc
    -- data output --
    rs1_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- operand 1
    rs2_o  : out std_ulogic_vector(data_width_c-1 downto 0)  -- operand 2
  );
end neorv32_cpu_regfile;

architecture neorv32_cpu_regfile_rtl of neorv32_cpu_regfile is

  -- register file --
  type   reg_file_t is array (31 downto 0) of std_ulogic_vector(data_width_c-1 downto 0);
  type   reg_file_emb_t is array (15 downto 0) of std_ulogic_vector(data_width_c-1 downto 0);
  signal reg_file      : reg_file_t;
  signal reg_file_emb  : reg_file_emb_t;
  signal rf_write_data : std_ulogic_vector(data_width_c-1 downto 0); -- actual write-back data
  signal rd_is_r0      : std_ulogic; -- writing to r0?
  signal rf_we         : std_ulogic;
  signal dst_addr      : std_ulogic_vector(4 downto 0); -- destination address

begin

  -- Input mux ------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  input_mux: process(ctrl_i, mem_i, alu_i, pc_i, csr_i)
  begin
    case ctrl_i(ctrl_rf_in_mux_msb_c downto ctrl_rf_in_mux_lsb_c) is
      when "00"   => rf_write_data <= alu_i;
      when "01"   => rf_write_data <= mem_i;
      when "10"   => rf_write_data <= pc_i;
      when others => rf_write_data <= csr_i;
    end case;
  end process input_mux;

  -- check if we are writing to x0 --
  rd_is_r0 <= not or_all_f(ctrl_i(ctrl_rf_rd_adr4_c downto ctrl_rf_rd_adr0_c)) when (CPU_EXTENSION_RISCV_E = false) else
              not or_all_f(ctrl_i(ctrl_rf_rd_adr3_c downto ctrl_rf_rd_adr0_c));

  -- valid RF write access --
  rf_we <= (ctrl_i(ctrl_rf_wb_en_c) and (not rd_is_r0)) or ctrl_i(ctrl_rf_r0_we_c);

  -- destination address --
  dst_addr <= ctrl_i(ctrl_rf_rd_adr4_c downto ctrl_rf_rd_adr0_c) when (ctrl_i(ctrl_rf_r0_we_c) = '0') else (others => '0'); -- force dst=r0?


  -- Register file read/write access --------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rf_access: process(clk_i)
  begin
    if rising_edge(clk_i) then -- sync read and write
      if (CPU_EXTENSION_RISCV_E = false) then -- normal register file with 32 entries
        if (rf_we = '1') then
          reg_file(to_integer(unsigned(dst_addr(4 downto 0)))) <= rf_write_data;
        else -- read
          rs1_o <= reg_file(to_integer(unsigned(ctrl_i(ctrl_rf_rs1_adr4_c downto ctrl_rf_rs1_adr0_c))));
          rs2_o <= reg_file(to_integer(unsigned(ctrl_i(ctrl_rf_rs2_adr4_c downto ctrl_rf_rs2_adr0_c))));
        end if;
      else -- embedded register file with 16 entries
        if (rf_we = '1') then
          reg_file_emb(to_integer(unsigned(dst_addr(3 downto 0)))) <= rf_write_data;
        else -- read
          rs1_o <= reg_file_emb(to_integer(unsigned(ctrl_i(ctrl_rf_rs1_adr3_c downto ctrl_rf_rs1_adr0_c))));
          rs2_o <= reg_file_emb(to_integer(unsigned(ctrl_i(ctrl_rf_rs2_adr3_c downto ctrl_rf_rs2_adr0_c))));
        end if;
      end if;
    end if;
  end process rf_access;


end neorv32_cpu_regfile_rtl;
