-- #################################################################################################
-- # << NEORV32 - CPU General Purpose Data Register File >>                                        #
-- # ********************************************************************************************* #
-- # Data register file. 32 entries (= 1024 bit) for RV32I ISA (default), 16 entries (= 512 bit)   #
-- # for RV32E ISA (when RISC-V "E" extension is enabled).                                         #
-- #                                                                                               #
-- # Register zero (x0) is a "normal" physical register that is set to zero by the CPU control     #
-- # hardware. This is not required for non-BRAM-based register files where x0 is hardwired to     #
-- # zero. Set <reset_x0_c> to 'false' in this case.                                               #
-- #                                                                                               #
-- # The register file uses synchronous read accesses and a *single* (multiplexed) address port    #
-- # for writing and reading rd/rs1 and a single read-only port for rs2. Therefore, the whole      #
-- # register file can be mapped to a single true-dual-port block RAM.                             #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
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
    CPU_EXTENSION_RISCV_E : boolean -- implement embedded RF extension?
  );
  port (
    -- global control --
    clk_i  : in  std_ulogic; -- global clock, rising edge
    ctrl_i : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
    -- data input --
    alu_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- ALU result
    mem_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- memory read data
    csr_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- CSR read data
    pc2_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- next PC
    -- data output --
    rs1_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- operand 1
    rs2_o  : out std_ulogic_vector(data_width_c-1 downto 0)  -- operand 2
  );
end neorv32_cpu_regfile;

architecture neorv32_cpu_regfile_rtl of neorv32_cpu_regfile is

  -- register file --
  type   reg_file_t is array (31 downto 0) of std_ulogic_vector(data_width_c-1 downto 0);
  type   reg_file_emb_t is array (15 downto 0) of std_ulogic_vector(data_width_c-1 downto 0);
  signal reg_file     : reg_file_t;
  signal reg_file_emb : reg_file_emb_t;

  -- access --
  signal rf_wdata : std_ulogic_vector(data_width_c-1 downto 0); -- actual write-back data
  signal rf_we    : std_ulogic; -- write enable
  signal rd_zero  : std_ulogic; -- writing to x0?
  signal opa_addr : std_ulogic_vector(4 downto 0); -- rs1/dst address
  signal opb_addr : std_ulogic_vector(4 downto 0); -- rs2 address

begin

  -- Data Write-Back Select -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  wb_select: process(ctrl_i, alu_i, mem_i, csr_i, pc2_i)
  begin
    case ctrl_i(ctrl_rf_mux1_c downto ctrl_rf_mux0_c) is
      when rf_mux_alu_c => rf_wdata <= alu_i; -- ALU result
      when rf_mux_mem_c => rf_wdata <= mem_i; -- memory read data
      when rf_mux_csr_c => rf_wdata <= csr_i; -- CSR read data
      when rf_mux_npc_c => rf_wdata <= pc2_i; -- next PC (branch return/link address)
      when others       => rf_wdata <= alu_i; -- don't care
    end case;
  end process wb_select;


  -- Register File Access -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- "normal" (RV32I) register file with 32 registers --
  reg_file_rv32i:
  if (CPU_EXTENSION_RISCV_E = false) generate
    rf_access: process(clk_i)
    begin
      if rising_edge(clk_i) then -- sync read and write
        if (rf_we = '1') then
          reg_file(to_integer(unsigned(opa_addr(4 downto 0)))) <= rf_wdata;
        end if;
        rs1_o <= reg_file(to_integer(unsigned(opa_addr(4 downto 0))));
        rs2_o <= reg_file(to_integer(unsigned(opb_addr(4 downto 0))));
      end if;
    end process rf_access;
  end generate;

  -- "embedded" (RV32E) register file with 16 registers --
  reg_file_rv32e:
  if (CPU_EXTENSION_RISCV_E = true) generate
    rf_access: process(clk_i)
    begin
      if rising_edge(clk_i) then -- sync read and write
        if (rf_we = '1') then
          reg_file_emb(to_integer(unsigned(opa_addr(3 downto 0)))) <= rf_wdata;
        end if;
        rs1_o <= reg_file_emb(to_integer(unsigned(opa_addr(3 downto 0))));
        rs2_o <= reg_file_emb(to_integer(unsigned(opb_addr(3 downto 0))));
      end if;
    end process rf_access;
  end generate;

  -- write enable --
  rd_zero <= '1' when (ctrl_i(ctrl_rf_rd_adr4_c downto ctrl_rf_rd_adr0_c) = "00000") else '0';
  rf_we   <= (ctrl_i(ctrl_rf_wb_en_c) and (not rd_zero)) or ctrl_i(ctrl_rf_zero_we_c); -- do not write to x0 unless explicitly forced

  -- access addresses --
  opa_addr <= "00000" when (ctrl_i(ctrl_rf_zero_we_c) = '1') else -- force rd = zero
              ctrl_i(ctrl_rf_rd_adr4_c downto ctrl_rf_rd_adr0_c) when (ctrl_i(ctrl_rf_wb_en_c) = '1') else -- rd
              ctrl_i(ctrl_rf_rs1_adr4_c downto ctrl_rf_rs1_adr0_c); -- rs1
  opb_addr <= ctrl_i(ctrl_rf_rs2_adr4_c downto ctrl_rf_rs2_adr0_c); -- rs2


end neorv32_cpu_regfile_rtl;
