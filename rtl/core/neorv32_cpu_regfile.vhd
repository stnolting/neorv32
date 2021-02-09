-- #################################################################################################
-- # << NEORV32 - CPU General Purpose Data Register File >>                                        #
-- # ********************************************************************************************* #
-- # General purpose data register file. 32 entries (= 1024 bit) for normal mode (RV32I),          #
-- # 16 entries (= 512 bit) for embedded mode (RV32E) when RISC-V "E" extension is enabled.        #
-- #                                                                                               #
-- # Register zero (r0/x0) is a "normal" physical reg that has to be initialized to zero by the    #
-- # CPU control system. For normal operations register zero cannot be written.                    #
-- #                                                                                               #
-- # The register file uses synchronous read accesses and a *single* (multiplexed) address port    #
-- # for writing and reading rs1 and a single read-only port for rs2. Therefore, the whole         #
-- # register file can be mapped to a single true dual-port block RAM.                             #
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
    -- data output --
    rs1_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- operand 1
    rs2_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- operand 2
    cmp_o  : out std_ulogic_vector(1 downto 0) -- comparator status
  );
end neorv32_cpu_regfile;

architecture neorv32_cpu_regfile_rtl of neorv32_cpu_regfile is

  -- register file --
  type   reg_file_t is array (31 downto 0) of std_ulogic_vector(data_width_c-1 downto 0);
  type   reg_file_emb_t is array (15 downto 0) of std_ulogic_vector(data_width_c-1 downto 0);
  signal reg_file     : reg_file_t;
  signal reg_file_emb : reg_file_emb_t;
  signal rf_wdata     : std_ulogic_vector(data_width_c-1 downto 0); -- actual write-back data
  signal rd_is_r0     : std_ulogic; -- writing to r0?
  signal rf_we        : std_ulogic;
  signal dst_addr     : std_ulogic_vector(4 downto 0); -- destination address
  signal opa_addr     : std_ulogic_vector(4 downto 0); -- rs1/dst address
  signal opb_addr     : std_ulogic_vector(4 downto 0); -- rs2 address
  signal rs1, rs2     : std_ulogic_vector(data_width_c-1 downto 0);

  -- comparator --
  signal cmp_opx : std_ulogic_vector(data_width_c downto 0);
  signal cmp_opy : std_ulogic_vector(data_width_c downto 0);

begin

  -- Data Input Mux -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rf_wdata <= alu_i when (ctrl_i(ctrl_rf_in_mux_c) = '0') else mem_i;


  -- Register File Access -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rf_access: process(clk_i)
  begin
    if rising_edge(clk_i) then -- sync read and write
      if (CPU_EXTENSION_RISCV_E = false) then -- normal register file with 32 entries
        if (rf_we = '1') then
          reg_file(to_integer(unsigned(opa_addr(4 downto 0)))) <= rf_wdata;
        end if;
        rs1 <= reg_file(to_integer(unsigned(opa_addr(4 downto 0))));
        rs2 <= reg_file(to_integer(unsigned(opb_addr(4 downto 0))));
      else -- embedded register file with 16 entries
        if (rf_we = '1') then
          reg_file_emb(to_integer(unsigned(opa_addr(3 downto 0)))) <= rf_wdata;
        end if;
        rs1 <= reg_file_emb(to_integer(unsigned(opa_addr(3 downto 0))));
        rs2 <= reg_file_emb(to_integer(unsigned(opb_addr(3 downto 0))));
      end if;
    end if;
  end process rf_access;

  -- check if we are writing to x0 --
  rd_is_r0 <= not or_all_f(ctrl_i(ctrl_rf_rd_adr4_c downto ctrl_rf_rd_adr0_c)) when (CPU_EXTENSION_RISCV_E = false) else
              not or_all_f(ctrl_i(ctrl_rf_rd_adr3_c downto ctrl_rf_rd_adr0_c));

  -- valid RF write access? --
  rf_we <= (ctrl_i(ctrl_rf_wb_en_c) and (not rd_is_r0)) or ctrl_i(ctrl_rf_r0_we_c);

  -- destination address --
  dst_addr <= ctrl_i(ctrl_rf_rd_adr4_c downto ctrl_rf_rd_adr0_c) when (ctrl_i(ctrl_rf_r0_we_c) = '0') else (others => '0'); -- force dst=r0?

  -- access addresses --
  opa_addr <= dst_addr when (rf_we = '1') else ctrl_i(ctrl_rf_rs1_adr4_c downto ctrl_rf_rs1_adr0_c); -- rd/rs1
  opb_addr <= ctrl_i(ctrl_rf_rs2_adr4_c downto ctrl_rf_rs2_adr0_c); -- rs2

  -- data output --
  rs1_o <= rs1;
  rs2_o <= rs2;


  -- Comparator Unit (for conditional branches) ---------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cmp_opx <= (rs1(rs1'left) and (not ctrl_i(ctrl_alu_unsigned_c))) & rs1;
  cmp_opy <= (rs2(rs2'left) and (not ctrl_i(ctrl_alu_unsigned_c))) & rs2;

  cmp_o(cmp_equal_c) <= '1' when (rs1 = rs2) else '0';
  cmp_o(cmp_less_c)  <= '1' when (signed(cmp_opx) < signed(cmp_opy)) else '0';


end neorv32_cpu_regfile_rtl;
