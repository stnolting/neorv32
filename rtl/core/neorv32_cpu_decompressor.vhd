-- #################################################################################################
-- # << NEORV32 - CPU: Compressed Instructions Decoder (RISC-V "C" Extension) >>                   #
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

entity neorv32_cpu_decompressor is
  port (
    -- instruction input --
    ci_instr16_i : in  std_ulogic_vector(15 downto 0); -- compressed instruction input
    -- instruction output --
    ci_illegal_o : out std_ulogic; -- is an illegal compressed instruction
    ci_instr32_o : out std_ulogic_vector(31 downto 0)  -- 32-bit decompressed instruction
  );
end neorv32_cpu_decompressor;

architecture neorv32_cpu_decompressor_rtl of neorv32_cpu_decompressor is

  -- compressed instruction layout --
  constant ci_opcode_lsb_c : natural :=  0;
  constant ci_opcode_msb_c : natural :=  1;
  constant ci_rd_3_lsb_c   : natural :=  2;
  constant ci_rd_3_msb_c   : natural :=  4;
  constant ci_rd_5_lsb_c   : natural :=  7;
  constant ci_rd_5_msb_c   : natural := 11;
  constant ci_rs1_3_lsb_c  : natural :=  7;
  constant ci_rs1_3_msb_c  : natural :=  9;
  constant ci_rs1_5_lsb_c  : natural :=  7;
  constant ci_rs1_5_msb_c  : natural := 11;
  constant ci_rs2_3_lsb_c  : natural :=  2;
  constant ci_rs2_3_msb_c  : natural :=  4;
  constant ci_rs2_5_lsb_c  : natural :=  2;
  constant ci_rs2_5_msb_c  : natural :=  6;
  constant ci_funct3_lsb_c : natural := 13;
  constant ci_funct3_msb_c : natural := 15;

begin

  -- Compressed Instruction Decoder ---------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  decompressor: process(ci_instr16_i)
    variable imm20_v : std_ulogic_vector(20 downto 0);
    variable imm12_v : std_ulogic_vector(12 downto 0);
  begin
    -- defaults --
    ci_illegal_o <= '0';
    ci_instr32_o <= (others => '0');

    -- helper: 22-bit sign-extended immediate for J/JAL --
    imm20_v := (others => ci_instr16_i(12)); -- sign extension
    imm20_v(00):= '0';
    imm20_v(01):= ci_instr16_i(3);
    imm20_v(02):= ci_instr16_i(4);
    imm20_v(03):= ci_instr16_i(5);
    imm20_v(04):= ci_instr16_i(11);
    imm20_v(05):= ci_instr16_i(2);
    imm20_v(06):= ci_instr16_i(7);
    imm20_v(07):= ci_instr16_i(6);
    imm20_v(08):= ci_instr16_i(9);
    imm20_v(09):= ci_instr16_i(10);
    imm20_v(10):= ci_instr16_i(8);
    imm20_v(11):= ci_instr16_i(12);

    -- helper: 12-bit sign-extended immediate for branches --
    imm12_v := (others => ci_instr16_i(12)); -- sign extension
    imm12_v(00):= '0';
    imm12_v(01):= ci_instr16_i(3);
    imm12_v(02):= ci_instr16_i(4);
    imm12_v(03):= ci_instr16_i(10);
    imm12_v(04):= ci_instr16_i(11);
    imm12_v(05):= ci_instr16_i(2);
    imm12_v(06):= ci_instr16_i(5);
    imm12_v(07):= ci_instr16_i(6);
    imm12_v(08):= ci_instr16_i(12);

    -- actual decoder --
    case ci_instr16_i(ci_opcode_msb_c downto ci_opcode_lsb_c) is
    
      when "00" => -- C0: Register-Based Loads and Stores
        case ci_instr16_i(ci_funct3_msb_c downto ci_funct3_lsb_c) is

          when "000" => -- Illegal_instruction, C.ADDI4SPN
          -- ----------------------------------------------------------------------------------------------------------
            -- C.ADDI4SPN
            ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alui_c;
            ci_instr32_o(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "00010"; -- stack pointer
            ci_instr32_o(instr_rd_msb_c downto instr_rd_lsb_c)         <= "01" & ci_instr16_i(ci_rd_3_msb_c downto ci_rd_3_lsb_c);
            ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_subadd_c;
            ci_instr32_o(instr_imm12_msb_c downto instr_imm12_lsb_c)   <= (others => '0'); -- zero extend
            ci_instr32_o(instr_imm12_lsb_c + 0)                        <= '0';
            ci_instr32_o(instr_imm12_lsb_c + 1)                        <= '0';
            ci_instr32_o(instr_imm12_lsb_c + 2)                        <= ci_instr16_i(6);
            ci_instr32_o(instr_imm12_lsb_c + 3)                        <= ci_instr16_i(5);
            ci_instr32_o(instr_imm12_lsb_c + 4)                        <= ci_instr16_i(11);
            ci_instr32_o(instr_imm12_lsb_c + 5)                        <= ci_instr16_i(12);
            ci_instr32_o(instr_imm12_lsb_c + 6)                        <= ci_instr16_i(7);
            ci_instr32_o(instr_imm12_lsb_c + 7)                        <= ci_instr16_i(8);
            ci_instr32_o(instr_imm12_lsb_c + 8)                        <= ci_instr16_i(9);
            ci_instr32_o(instr_imm12_lsb_c + 9)                        <= ci_instr16_i(10);
            --
            ci_illegal_o <= not or_reduce_f(ci_instr16_i(12 downto 2)); -- 12:2 = "00000000000" is official illegal instruction

          when "010" | "011" => -- C.LW / C.FLW
          -- ----------------------------------------------------------------------------------------------------------
            ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_load_c;
            ci_instr32_o(21 downto 20)                                 <= "00";
            ci_instr32_o(22)                                           <= ci_instr16_i(6);
            ci_instr32_o(23)                                           <= ci_instr16_i(10);
            ci_instr32_o(24)                                           <= ci_instr16_i(11);
            ci_instr32_o(25)                                           <= ci_instr16_i(12);
            ci_instr32_o(26)                                           <= ci_instr16_i(5);
            ci_instr32_o(31 downto 27)                                 <= (others => '0');
            ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_lw_c;
            ci_instr32_o(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "01" & ci_instr16_i(ci_rs1_3_msb_c downto ci_rs1_3_lsb_c); -- x8 - x15
            ci_instr32_o(instr_rd_msb_c downto instr_rd_lsb_c)         <= "01" & ci_instr16_i(ci_rd_3_msb_c downto ci_rd_3_lsb_c);   -- x8 - x15
            if (ci_instr16_i(ci_funct3_lsb_c) = '1') then -- C.FLW
              ci_illegal_o <= '1';
            end if;

          when "110" | "111" => -- C.SW / C.FSW
          -- ----------------------------------------------------------------------------------------------------------
            ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_store_c;
            ci_instr32_o(08 downto 07)                                 <= "00";
            ci_instr32_o(09)                                           <= ci_instr16_i(6);
            ci_instr32_o(10)                                           <= ci_instr16_i(10);
            ci_instr32_o(11)                                           <= ci_instr16_i(11);
            ci_instr32_o(25)                                           <= ci_instr16_i(12);
            ci_instr32_o(26)                                           <= ci_instr16_i(5);
            ci_instr32_o(31 downto 27)                                 <= (others => '0');
            ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_sw_c;
            ci_instr32_o(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "01" & ci_instr16_i(ci_rs1_3_msb_c downto ci_rs1_3_lsb_c); -- x8 - x15
            ci_instr32_o(instr_rs2_msb_c downto instr_rs2_lsb_c)       <= "01" & ci_instr16_i(ci_rs2_3_msb_c downto ci_rs2_3_lsb_c); -- x8 - x15
            if (ci_instr16_i(ci_funct3_lsb_c) = '1') then -- C.FSW
              ci_illegal_o <= '1';
            end if;

          when others => -- undefined
          -- ----------------------------------------------------------------------------------------------------------
            ci_instr32_o <= (others => '-');
            ci_illegal_o <= '1';

        end case;

      when "01" => -- C1: Control Transfer Instructions, Integer Constant-Generation Instructions

        case ci_instr16_i(ci_funct3_msb_c downto ci_funct3_lsb_c) is
          when "101" => -- C.J
          -- ----------------------------------------------------------------------------------------------------------
            ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_jal_c;
            ci_instr32_o(instr_rd_msb_c downto instr_rd_lsb_c)         <= "00000"; -- discard return address
            ci_instr32_o(19 downto 12)                                 <= imm20_v(19 downto 12);
            ci_instr32_o(20)                                           <= imm20_v(11);
            ci_instr32_o(30 downto 21)                                 <= imm20_v(10 downto 01);
            ci_instr32_o(31)                                           <= imm20_v(20);

          when "001" => -- C.JAL
          -- ----------------------------------------------------------------------------------------------------------
            ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_jal_c;
            ci_instr32_o(instr_rd_msb_c downto instr_rd_lsb_c)         <= "00001"; -- save return address to link register
            ci_instr32_o(19 downto 12)                                 <= imm20_v(19 downto 12);
            ci_instr32_o(20)                                           <= imm20_v(11);
            ci_instr32_o(30 downto 21)                                 <= imm20_v(10 downto 01);
            ci_instr32_o(31)                                           <= imm20_v(20);

          when "110" => -- C.BEQ
          -- ----------------------------------------------------------------------------------------------------------
            ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_branch_c;
            ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_beq_c;
            ci_instr32_o(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "01" & ci_instr16_i(ci_rs1_3_msb_c downto ci_rs1_3_lsb_c);
            ci_instr32_o(instr_rs2_msb_c downto instr_rs2_lsb_c)       <= "00000"; -- x0
            ci_instr32_o(07)                                           <= imm12_v(11);
            ci_instr32_o(11 downto 08)                                 <= imm12_v(04 downto 01);
            ci_instr32_o(30 downto 25)                                 <= imm12_v(10 downto 05);
            ci_instr32_o(31)                                           <= imm12_v(12);

          when "111" => -- C.BNEZ
          -- ----------------------------------------------------------------------------------------------------------
            ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_branch_c;
            ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_bne_c;
            ci_instr32_o(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "01" & ci_instr16_i(ci_rs1_3_msb_c downto ci_rs1_3_lsb_c);
            ci_instr32_o(instr_rs2_msb_c downto instr_rs2_lsb_c)       <= "00000"; -- x0
            ci_instr32_o(07)                                           <= imm12_v(11);
            ci_instr32_o(11 downto 08)                                 <= imm12_v(04 downto 01);
            ci_instr32_o(30 downto 25)                                 <= imm12_v(10 downto 05);
            ci_instr32_o(31)                                           <= imm12_v(12);

          when "010" => -- C.LI
          -- ----------------------------------------------------------------------------------------------------------
            ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alui_c;
            ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_subadd_c;
            ci_instr32_o(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "00000"; -- x0
            ci_instr32_o(instr_rd_msb_c downto instr_rd_lsb_c)         <= ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c);
            ci_instr32_o(instr_imm12_msb_c downto instr_imm12_lsb_c)   <= (others => ci_instr16_i(12)); -- sign extend
            ci_instr32_o(instr_imm12_lsb_c + 0)                        <= ci_instr16_i(2);
            ci_instr32_o(instr_imm12_lsb_c + 1)                        <= ci_instr16_i(3);
            ci_instr32_o(instr_imm12_lsb_c + 2)                        <= ci_instr16_i(4);
            ci_instr32_o(instr_imm12_lsb_c + 3)                        <= ci_instr16_i(5);
            ci_instr32_o(instr_imm12_lsb_c + 4)                        <= ci_instr16_i(6);
            ci_instr32_o(instr_imm12_lsb_c + 5)                        <= ci_instr16_i(12);

          when "011" => -- C.LUI / C.ADDI16SP
          -- ----------------------------------------------------------------------------------------------------------
            if (ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c) = "00010") then -- C.ADDI16SP
              ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alui_c;
              ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_subadd_c;
              ci_instr32_o(instr_rd_msb_c downto instr_rd_lsb_c)         <= ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c);
              ci_instr32_o(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "00010"; -- stack pointer
              ci_instr32_o(instr_rd_msb_c downto instr_rd_lsb_c)         <= "00010"; -- stack pointer
              ci_instr32_o(instr_imm12_msb_c downto instr_imm12_lsb_c)   <= (others => ci_instr16_i(12)); -- sign extend
              ci_instr32_o(instr_imm12_lsb_c + 0)                        <= '0';
              ci_instr32_o(instr_imm12_lsb_c + 1)                        <= '0';
              ci_instr32_o(instr_imm12_lsb_c + 2)                        <= '0';
              ci_instr32_o(instr_imm12_lsb_c + 3)                        <= '0';
              ci_instr32_o(instr_imm12_lsb_c + 4)                        <= ci_instr16_i(6);
              ci_instr32_o(instr_imm12_lsb_c + 5)                        <= ci_instr16_i(2);
              ci_instr32_o(instr_imm12_lsb_c + 6)                        <= ci_instr16_i(5);
              ci_instr32_o(instr_imm12_lsb_c + 7)                        <= ci_instr16_i(3);
              ci_instr32_o(instr_imm12_lsb_c + 8)                        <= ci_instr16_i(4);
              ci_instr32_o(instr_imm12_lsb_c + 9)                        <= ci_instr16_i(12);

            else -- C.LUI
              ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_lui_c;
              ci_instr32_o(instr_rd_msb_c downto instr_rd_lsb_c)         <= ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c);
              ci_instr32_o(instr_imm20_msb_c downto instr_imm20_lsb_c)   <= (others => ci_instr16_i(12)); -- sign extend
              ci_instr32_o(instr_imm20_lsb_c + 0)                        <= ci_instr16_i(2);
              ci_instr32_o(instr_imm20_lsb_c + 1)                        <= ci_instr16_i(3);
              ci_instr32_o(instr_imm20_lsb_c + 2)                        <= ci_instr16_i(4);
              ci_instr32_o(instr_imm20_lsb_c + 3)                        <= ci_instr16_i(5);
              ci_instr32_o(instr_imm20_lsb_c + 4)                        <= ci_instr16_i(6);
              ci_instr32_o(instr_imm20_lsb_c + 5)                        <= ci_instr16_i(12);
            end if;
            if (ci_instr16_i(6 downto 2) = "00000") and (ci_instr16_i(12) = '0') then -- reserved
              ci_illegal_o <= '1';
            end if;

          when "000" => -- C.NOP (rd=0) / C.ADDI
          -- ----------------------------------------------------------------------------------------------------------
            ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alui_c;
            ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_subadd_c;
            ci_instr32_o(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= ci_instr16_i(ci_rs1_5_msb_c downto ci_rs1_5_lsb_c);
            ci_instr32_o(instr_rd_msb_c downto instr_rd_lsb_c)         <= ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c);
            ci_instr32_o(instr_imm12_msb_c downto instr_imm12_lsb_c)   <= (others => ci_instr16_i(12)); -- sign extend
            ci_instr32_o(instr_imm12_lsb_c + 0)                        <= ci_instr16_i(2);
            ci_instr32_o(instr_imm12_lsb_c + 1)                        <= ci_instr16_i(3);
            ci_instr32_o(instr_imm12_lsb_c + 2)                        <= ci_instr16_i(4);
            ci_instr32_o(instr_imm12_lsb_c + 3)                        <= ci_instr16_i(5);
            ci_instr32_o(instr_imm12_lsb_c + 4)                        <= ci_instr16_i(6);
            ci_instr32_o(instr_imm12_lsb_c + 5)                        <= ci_instr16_i(12);

          when "100" => -- C.SRLI, C.SRAI, C.ANDI, C.SUB, C.XOR, C.OR, C.AND, reserved
          -- ----------------------------------------------------------------------------------------------------------
            ci_instr32_o(instr_rs1_msb_c downto instr_rs1_lsb_c) <= "01" & ci_instr16_i(ci_rs1_3_msb_c downto ci_rs1_3_lsb_c);
            ci_instr32_o(instr_rd_msb_c downto instr_rd_lsb_c)   <= "01" & ci_instr16_i(ci_rs1_3_msb_c downto ci_rs1_3_lsb_c);
            if (ci_instr16_i(11 downto 10) = "11") then -- register-register operation
              ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alu_c;
              ci_instr32_o(instr_rs2_msb_c downto instr_rs2_lsb_c) <= "01" & ci_instr16_i(ci_rs2_3_msb_c downto ci_rs2_3_lsb_c);
              case ci_instr16_i(6 downto 5) is
                when "00" => -- C.SUB
                  ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_subadd_c;
                  ci_instr32_o(instr_funct7_msb_c downto instr_funct7_lsb_c) <= "0100000";
                when "01" => -- C.XOR
                  ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_xor_c;
                  ci_instr32_o(instr_funct7_msb_c downto instr_funct7_lsb_c) <= "0000000";
                when "10" => -- C.OR
                  ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_or_c;
                  ci_instr32_o(instr_funct7_msb_c downto instr_funct7_lsb_c) <= "0000000";
                when others => -- C.AND
                  ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_and_c;
                  ci_instr32_o(instr_funct7_msb_c downto instr_funct7_lsb_c) <= "0000000";
              end case;
            else -- register-immediate operation
              ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alui_c;
              case ci_instr16_i(11 downto 10) is
                when "00" => -- C.SRLI
                  ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_sr_c;
                  ci_instr32_o(instr_funct7_msb_c downto instr_funct7_lsb_c) <= "0000000";
                  ci_instr32_o(instr_imm12_lsb_c + 0)                        <= ci_instr16_i(2);
                  ci_instr32_o(instr_imm12_lsb_c + 1)                        <= ci_instr16_i(3);
                  ci_instr32_o(instr_imm12_lsb_c + 2)                        <= ci_instr16_i(4);
                  ci_instr32_o(instr_imm12_lsb_c + 3)                        <= ci_instr16_i(5);
                  ci_instr32_o(instr_imm12_lsb_c + 4)                        <= ci_instr16_i(6);
                  ci_illegal_o <= ci_instr16_i(12);
                when "01" => -- C.SRAI
                  ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_sr_c;
                  ci_instr32_o(instr_funct7_msb_c downto instr_funct7_lsb_c) <= "0100000";
                  ci_instr32_o(instr_imm12_lsb_c + 0)                        <= ci_instr16_i(2);
                  ci_instr32_o(instr_imm12_lsb_c + 1)                        <= ci_instr16_i(3);
                  ci_instr32_o(instr_imm12_lsb_c + 2)                        <= ci_instr16_i(4);
                  ci_instr32_o(instr_imm12_lsb_c + 3)                        <= ci_instr16_i(5);
                  ci_instr32_o(instr_imm12_lsb_c + 4)                        <= ci_instr16_i(6);
                  ci_illegal_o <= ci_instr16_i(12);
                when "10" => -- C.ANDI
                  ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_and_c;
                  ci_instr32_o(instr_imm12_msb_c downto instr_imm12_lsb_c)   <= (others => ci_instr16_i(12)); -- sign extend
                  ci_instr32_o(instr_imm12_lsb_c + 0)                        <= ci_instr16_i(2);
                  ci_instr32_o(instr_imm12_lsb_c + 1)                        <= ci_instr16_i(3);
                  ci_instr32_o(instr_imm12_lsb_c + 2)                        <= ci_instr16_i(4);
                  ci_instr32_o(instr_imm12_lsb_c + 3)                        <= ci_instr16_i(5);
                  ci_instr32_o(instr_imm12_lsb_c + 4)                        <= ci_instr16_i(6);
                  ci_instr32_o(instr_imm12_lsb_c + 5)                        <= ci_instr16_i(12);
                when others => -- register-register operation
                  NULL;
              end case;
            end if;
            if (ci_instr16_i(12 downto 10) = "111") then -- reserved / undefined
              ci_illegal_o <= '1';
            end if;

          when others => -- undefined
          -- ----------------------------------------------------------------------------------------------------------
            ci_instr32_o <= (others => '-');
            ci_illegal_o <= '1';

        end case;

      when "10" => -- C2: Stack-Pointer-Based Loads and Stores, Control Transfer Instructions
        case ci_instr16_i(ci_funct3_msb_c downto ci_funct3_lsb_c) is

          when "000" => -- C.SLLI
          -- ----------------------------------------------------------------------------------------------------------
            ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alui_c;
            ci_instr32_o(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= ci_instr16_i(ci_rs1_5_msb_c downto ci_rs1_5_lsb_c);
            ci_instr32_o(instr_rd_msb_c downto instr_rd_lsb_c)         <= ci_instr16_i(ci_rs1_5_msb_c downto ci_rs1_5_lsb_c);
            ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_sll_c;
            ci_instr32_o(instr_funct7_msb_c downto instr_funct7_lsb_c) <= "0000000";
            ci_instr32_o(instr_imm12_lsb_c + 0)                        <= ci_instr16_i(2);
            ci_instr32_o(instr_imm12_lsb_c + 1)                        <= ci_instr16_i(3);
            ci_instr32_o(instr_imm12_lsb_c + 2)                        <= ci_instr16_i(4);
            ci_instr32_o(instr_imm12_lsb_c + 3)                        <= ci_instr16_i(5);
            ci_instr32_o(instr_imm12_lsb_c + 4)                        <= ci_instr16_i(6);
            ci_illegal_o <= ci_instr16_i(12);

          when "010" | "011" => -- C.LWSP / C.FLWSP
          -- ----------------------------------------------------------------------------------------------------------
            ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_load_c;
            ci_instr32_o(21 downto 20)                                 <= "00";
            ci_instr32_o(22)                                           <= ci_instr16_i(4);
            ci_instr32_o(23)                                           <= ci_instr16_i(5);
            ci_instr32_o(24)                                           <= ci_instr16_i(6);
            ci_instr32_o(25)                                           <= ci_instr16_i(12);
            ci_instr32_o(26)                                           <= ci_instr16_i(2);
            ci_instr32_o(27)                                           <= ci_instr16_i(3);
            ci_instr32_o(31 downto 28)                                 <= (others => '0');
            ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_lw_c;
            ci_instr32_o(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "00010"; -- stack pointer
            ci_instr32_o(instr_rd_msb_c downto instr_rd_lsb_c)         <= ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c);
            if (ci_instr16_i(ci_funct3_lsb_c) = '1') then -- C.FLWSP
              ci_illegal_o <= '1';
            end if;

          when "110" | "111" => -- C.SWSP / C.FSWSP
          -- ----------------------------------------------------------------------------------------------------------
            ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_store_c;
            ci_instr32_o(08 downto 07)                                 <= "00";
            ci_instr32_o(09)                                           <= ci_instr16_i(9);
            ci_instr32_o(10)                                           <= ci_instr16_i(10);
            ci_instr32_o(11)                                           <= ci_instr16_i(11);
            ci_instr32_o(25)                                           <= ci_instr16_i(12);
            ci_instr32_o(26)                                           <= ci_instr16_i(7);
            ci_instr32_o(27)                                           <= ci_instr16_i(8);
            ci_instr32_o(31 downto 28)                                 <= (others => '0');
            ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_sw_c;
            ci_instr32_o(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "00010"; -- stack pointer
            ci_instr32_o(instr_rs2_msb_c downto instr_rs2_lsb_c)       <= ci_instr16_i(ci_rs2_5_msb_c downto ci_rs2_5_lsb_c);
            if (ci_instr16_i(ci_funct3_lsb_c) = '1') then -- C.FSWSP
              ci_illegal_o <= '1';
            end if;

          when "100" => -- C.JR, C.JALR, C.MV, C.EBREAK, C.ADD
          -- ----------------------------------------------------------------------------------------------------------
            if (ci_instr16_i(12) = '0') then -- C.JR, C.MV
              if (ci_instr16_i(6 downto 2) = "00000") then -- C.JR
                ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_jalr_c;
                ci_instr32_o(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= ci_instr16_i(ci_rs1_5_msb_c downto ci_rs1_5_lsb_c);
                ci_instr32_o(instr_rd_msb_c downto instr_rd_lsb_c)         <= "00000"; -- discard return address
              else -- C.MV
                ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alu_c;
                ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= "000";
                ci_instr32_o(instr_rd_msb_c downto instr_rd_lsb_c)         <= ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c);
                ci_instr32_o(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "00000"; -- x0
                ci_instr32_o(instr_rs2_msb_c downto instr_rs2_lsb_c)       <= ci_instr16_i(ci_rs2_5_msb_c downto ci_rs2_5_lsb_c);
              end if;
            else -- C.EBREAK, C.JALR, C.ADD
              if (ci_instr16_i(6 downto 2) = "00000") then -- C.EBREAK, C.JALR
                if (ci_instr16_i(11 downto 7) = "00000") then -- C.EBREAK
                  ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c)   <= opcode_syscsr_c;
                  ci_instr32_o(instr_funct12_msb_c downto instr_funct12_lsb_c) <= "000000000001";
                else -- C.JALR
                  ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_jalr_c;
                  ci_instr32_o(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= ci_instr16_i(ci_rs1_5_msb_c downto ci_rs1_5_lsb_c);
                  ci_instr32_o(instr_rd_msb_c downto instr_rd_lsb_c)         <= "00001"; -- save return address to link register
                end if;
              else -- C.ADD
                ci_instr32_o(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alu_c;
                ci_instr32_o(instr_funct3_msb_c downto instr_funct3_lsb_c) <= "000";
                ci_instr32_o(instr_rd_msb_c downto instr_rd_lsb_c)         <= ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c);
                ci_instr32_o(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c);
                ci_instr32_o(instr_rs2_msb_c downto instr_rs2_lsb_c)       <= ci_instr16_i(ci_rs2_5_msb_c downto ci_rs2_5_lsb_c);
              end if;
            end if;

          when others => -- undefined
          -- ----------------------------------------------------------------------------------------------------------
            ci_instr32_o <= (others => '-');
            ci_illegal_o <= '1';

        end case;

      when others => -- not a compressed instruction
      -- ----------------------------------------------------------------------------------------------------------
        ci_instr32_o <= (others => '-');
        ci_illegal_o <= '0';

    end case;
  end process decompressor;


end neorv32_cpu_decompressor_rtl;
