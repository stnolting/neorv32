-- #################################################################################################
-- # << NEORV32 - CPU: Compressed Instructions Decoder (RISC-V "C" Extension) >>                   #
-- # ********************************************************************************************* #
-- # Compressed instructions decoder compatible to the RISC-V C ISA extensions. Illegal compressed #
-- # instructions are output "as-is".                                                              #
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

entity neorv32_cpu_decompressor is
  generic (
    FPU_ENABLE : boolean -- floating-point instruction enabled
  );
  port (
    ci_instr16_i : in  std_ulogic_vector(15 downto 0); -- compressed instruction
    ci_instr32_o : out std_ulogic_vector(31 downto 0)  -- decompressed instruction
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

  -- immediates --
  signal imm20 : std_ulogic_vector(20 downto 0);
  signal imm12 : std_ulogic_vector(12 downto 0);

  -- intermediates --
  signal illegal : std_ulogic;
  signal decoded : std_ulogic_vector(31 downto 0);

begin

  -- Large Immediates -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- 22-bit sign-extended immediate for J/JAL --
  imm20(00) <= '0';
  imm20(01) <= ci_instr16_i(3);
  imm20(02) <= ci_instr16_i(4);
  imm20(03) <= ci_instr16_i(5);
  imm20(04) <= ci_instr16_i(11);
  imm20(05) <= ci_instr16_i(2);
  imm20(06) <= ci_instr16_i(7);
  imm20(07) <= ci_instr16_i(6);
  imm20(08) <= ci_instr16_i(9);
  imm20(09) <= ci_instr16_i(10);
  imm20(10) <= ci_instr16_i(8);
  imm20(20 downto 11) <= (others => ci_instr16_i(12)); -- sign extension

  -- 12-bit sign-extended immediate for branches --
  imm12(00) <= '0';
  imm12(01) <= ci_instr16_i(3);
  imm12(02) <= ci_instr16_i(4);
  imm12(03) <= ci_instr16_i(10);
  imm12(04) <= ci_instr16_i(11);
  imm12(05) <= ci_instr16_i(2);
  imm12(06) <= ci_instr16_i(5);
  imm12(07) <= ci_instr16_i(6);
  imm12(12 downto 08) <= (others => ci_instr16_i(12)); -- sign extension


  -- Compressed Instruction Decoder ---------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  decompressor: process(ci_instr16_i, imm20, imm12)
  begin
    -- defaults --
    illegal <= '0';
    decoded <= (others => '0');

    -- actual decoder --
    case ci_instr16_i(ci_opcode_msb_c downto ci_opcode_lsb_c) is

      when "00" => -- C0: Register-Based Loads and Stores
        case ci_instr16_i(ci_funct3_msb_c downto ci_funct3_lsb_c) is

          when "010" | "011" => -- C.LW / C.FLW (integer and float are identical as the FPU implements the Zfinx ISA extension)
          -- ----------------------------------------------------------------------------------------------------------
            decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_load_c;
            decoded(21 downto 20)                                 <= "00";
            decoded(22)                                           <= ci_instr16_i(6);
            decoded(23)                                           <= ci_instr16_i(10);
            decoded(24)                                           <= ci_instr16_i(11);
            decoded(25)                                           <= ci_instr16_i(12);
            decoded(26)                                           <= ci_instr16_i(5);
            decoded(31 downto 27)                                 <= (others => '0');
            decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_lw_c;
            decoded(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "01" & ci_instr16_i(ci_rs1_3_msb_c downto ci_rs1_3_lsb_c); -- x8 - x15
            decoded(instr_rd_msb_c downto instr_rd_lsb_c)         <= "01" & ci_instr16_i(ci_rd_3_msb_c downto ci_rd_3_lsb_c);   -- x8 - x15
            if (ci_instr16_i(ci_funct3_lsb_c) = '1') and (FPU_ENABLE = false) then -- C.FLW
              illegal <= '1';
            end if;

          when "110" | "111" => -- C.SW / C.FSW (integer and float are identical as the FPU implements the Zfinx ISA extension)
          -- ----------------------------------------------------------------------------------------------------------
            decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_store_c;
            decoded(08 downto 07)                                 <= "00";
            decoded(09)                                           <= ci_instr16_i(6);
            decoded(10)                                           <= ci_instr16_i(10);
            decoded(11)                                           <= ci_instr16_i(11);
            decoded(25)                                           <= ci_instr16_i(12);
            decoded(26)                                           <= ci_instr16_i(5);
            decoded(31 downto 27)                                 <= (others => '0');
            decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_sw_c;
            decoded(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "01" & ci_instr16_i(ci_rs1_3_msb_c downto ci_rs1_3_lsb_c); -- x8 - x15
            decoded(instr_rs2_msb_c downto instr_rs2_lsb_c)       <= "01" & ci_instr16_i(ci_rs2_3_msb_c downto ci_rs2_3_lsb_c); -- x8 - x15
            if (ci_instr16_i(ci_funct3_lsb_c) = '1') and (FPU_ENABLE = false) then -- C.FSW
              illegal <= '1';
            end if;

          when others => -- "000": Illegal_instruction, C.ADDI4SPN; others: illegal
          -- ----------------------------------------------------------------------------------------------------------
            decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alui_c;
            decoded(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "00010"; -- stack pointer
            decoded(instr_rd_msb_c downto instr_rd_lsb_c)         <= "01" & ci_instr16_i(ci_rd_3_msb_c downto ci_rd_3_lsb_c);
            decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_subadd_c;
            decoded(instr_imm12_msb_c downto instr_imm12_lsb_c)   <= (others => '0'); -- zero extend
            decoded(instr_imm12_lsb_c + 0)                        <= '0';
            decoded(instr_imm12_lsb_c + 1)                        <= '0';
            decoded(instr_imm12_lsb_c + 2)                        <= ci_instr16_i(6);
            decoded(instr_imm12_lsb_c + 3)                        <= ci_instr16_i(5);
            decoded(instr_imm12_lsb_c + 4)                        <= ci_instr16_i(11);
            decoded(instr_imm12_lsb_c + 5)                        <= ci_instr16_i(12);
            decoded(instr_imm12_lsb_c + 6)                        <= ci_instr16_i(7);
            decoded(instr_imm12_lsb_c + 7)                        <= ci_instr16_i(8);
            decoded(instr_imm12_lsb_c + 8)                        <= ci_instr16_i(9);
            decoded(instr_imm12_lsb_c + 9)                        <= ci_instr16_i(10);
            --
            if (ci_instr16_i(12 downto 5) = "00000000") or -- canonical illegal C instruction or C.ADDI4SPN with nzuimm = 0
               (ci_instr16_i(ci_funct3_msb_c downto ci_funct3_lsb_c) = "001") or -- C.FLS / C.LQ
               (ci_instr16_i(ci_funct3_msb_c downto ci_funct3_lsb_c) = "100") or -- reserved
               (ci_instr16_i(ci_funct3_msb_c downto ci_funct3_lsb_c) = "101") then -- C.C.FSD / C.SQ
              illegal <= '1';
            end if;

        end case;

      when "01" => -- C1: Control Transfer Instructions, Integer Constant-Generation Instructions

        case ci_instr16_i(ci_funct3_msb_c downto ci_funct3_lsb_c) is
          when "101" | "001" => -- C.J, C.JAL
          -- ----------------------------------------------------------------------------------------------------------
            if (ci_instr16_i(ci_funct3_msb_c) = '1') then -- C.J
              decoded(instr_rd_msb_c downto instr_rd_lsb_c) <= "00000"; -- discard return address
            else -- C.JAL
              decoded(instr_rd_msb_c downto instr_rd_lsb_c) <= "00001"; -- save return address to link register
            end if;
            decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_jal_c;
            decoded(19 downto 12)                                 <= imm20(19 downto 12);
            decoded(20)                                           <= imm20(11);
            decoded(30 downto 21)                                 <= imm20(10 downto 01);
            decoded(31)                                           <= imm20(20);

          when "110" | "111" => -- C.BEQ, C.BNEZ
          -- ----------------------------------------------------------------------------------------------------------
            if (ci_instr16_i(ci_funct3_lsb_c) = '0') then -- C.BEQ
              decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_beq_c;
            else -- C.BNEZ
              decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_bne_c;
            end if;
            decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_branch_c;
            decoded(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "01" & ci_instr16_i(ci_rs1_3_msb_c downto ci_rs1_3_lsb_c);
            decoded(instr_rs2_msb_c downto instr_rs2_lsb_c)       <= "00000"; -- x0
            decoded(07)                                           <= imm12(11);
            decoded(11 downto 08)                                 <= imm12(04 downto 01);
            decoded(30 downto 25)                                 <= imm12(10 downto 05);
            decoded(31)                                           <= imm12(12);

          when "010" => -- C.LI
          -- ----------------------------------------------------------------------------------------------------------
            decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alui_c;
            decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_subadd_c;
            decoded(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "00000"; -- x0
            decoded(instr_rd_msb_c downto instr_rd_lsb_c)         <= ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c);
            decoded(instr_imm12_msb_c downto instr_imm12_lsb_c)   <= (others => ci_instr16_i(12)); -- sign extend
            decoded(instr_imm12_lsb_c + 0)                        <= ci_instr16_i(2);
            decoded(instr_imm12_lsb_c + 1)                        <= ci_instr16_i(3);
            decoded(instr_imm12_lsb_c + 2)                        <= ci_instr16_i(4);
            decoded(instr_imm12_lsb_c + 3)                        <= ci_instr16_i(5);
            decoded(instr_imm12_lsb_c + 4)                        <= ci_instr16_i(6);
            decoded(instr_imm12_lsb_c + 5)                        <= ci_instr16_i(12);

          when "011" => -- C.LUI / C.ADDI16SP
          -- ----------------------------------------------------------------------------------------------------------
            if (ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c) = "00010") then -- C.ADDI16SP
              decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alui_c;
              decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_subadd_c;
              decoded(instr_rd_msb_c downto instr_rd_lsb_c)         <= ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c);
              decoded(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "00010"; -- stack pointer
              decoded(instr_rd_msb_c downto instr_rd_lsb_c)         <= "00010"; -- stack pointer
              decoded(instr_imm12_msb_c downto instr_imm12_lsb_c)   <= (others => ci_instr16_i(12)); -- sign extend
              decoded(instr_imm12_lsb_c + 0)                        <= '0';
              decoded(instr_imm12_lsb_c + 1)                        <= '0';
              decoded(instr_imm12_lsb_c + 2)                        <= '0';
              decoded(instr_imm12_lsb_c + 3)                        <= '0';
              decoded(instr_imm12_lsb_c + 4)                        <= ci_instr16_i(6);
              decoded(instr_imm12_lsb_c + 5)                        <= ci_instr16_i(2);
              decoded(instr_imm12_lsb_c + 6)                        <= ci_instr16_i(5);
              decoded(instr_imm12_lsb_c + 7)                        <= ci_instr16_i(3);
              decoded(instr_imm12_lsb_c + 8)                        <= ci_instr16_i(4);
              decoded(instr_imm12_lsb_c + 9)                        <= ci_instr16_i(12);
            else -- C.LUI
              decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_lui_c;
              decoded(instr_rd_msb_c downto instr_rd_lsb_c)         <= ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c);
              decoded(instr_imm20_msb_c downto instr_imm20_lsb_c)   <= (others => ci_instr16_i(12)); -- sign extend
              decoded(instr_imm20_lsb_c + 0)                        <= ci_instr16_i(2);
              decoded(instr_imm20_lsb_c + 1)                        <= ci_instr16_i(3);
              decoded(instr_imm20_lsb_c + 2)                        <= ci_instr16_i(4);
              decoded(instr_imm20_lsb_c + 3)                        <= ci_instr16_i(5);
              decoded(instr_imm20_lsb_c + 4)                        <= ci_instr16_i(6);
              decoded(instr_imm20_lsb_c + 5)                        <= ci_instr16_i(12);
            end if;
            if (ci_instr16_i(6 downto 2) = "00000") and (ci_instr16_i(12) = '0') then -- reserved if nzimm = 0
              illegal <= '1';
            end if;

          when "000" => -- C.NOP (rd=0) / C.ADDI
          -- ----------------------------------------------------------------------------------------------------------
            decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alui_c;
            decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_subadd_c;
            decoded(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= ci_instr16_i(ci_rs1_5_msb_c downto ci_rs1_5_lsb_c);
            decoded(instr_rd_msb_c downto instr_rd_lsb_c)         <= ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c);
            decoded(instr_imm12_msb_c downto instr_imm12_lsb_c)   <= (others => ci_instr16_i(12)); -- sign extend
            decoded(instr_imm12_lsb_c + 0)                        <= ci_instr16_i(2);
            decoded(instr_imm12_lsb_c + 1)                        <= ci_instr16_i(3);
            decoded(instr_imm12_lsb_c + 2)                        <= ci_instr16_i(4);
            decoded(instr_imm12_lsb_c + 3)                        <= ci_instr16_i(5);
            decoded(instr_imm12_lsb_c + 4)                        <= ci_instr16_i(6);
            decoded(instr_imm12_lsb_c + 5)                        <= ci_instr16_i(12);

          when others => -- 100: C.SRLI, C.SRAI, C.ANDI, C.SUB, C.XOR, C.OR, C.AND, reserved
          -- ----------------------------------------------------------------------------------------------------------
            decoded(instr_rd_msb_c downto instr_rd_lsb_c)   <= "01" & ci_instr16_i(ci_rs1_3_msb_c downto ci_rs1_3_lsb_c);
            decoded(instr_rs1_msb_c downto instr_rs1_lsb_c) <= "01" & ci_instr16_i(ci_rs1_3_msb_c downto ci_rs1_3_lsb_c);
            decoded(instr_rs2_msb_c downto instr_rs2_lsb_c) <= "01" & ci_instr16_i(ci_rs2_3_msb_c downto ci_rs2_3_lsb_c);
            case ci_instr16_i(11 downto 10) is
              when "00" | "01" => -- C.SRLI, C.SRAI
                if (ci_instr16_i(10) = '0') then -- C.SRLI
                  decoded(instr_funct7_msb_c downto instr_funct7_lsb_c) <= "0000000";
                else -- C.SRAI
                  decoded(instr_funct7_msb_c downto instr_funct7_lsb_c) <= "0100000";
                end if;
                decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alui_c;
                decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_sr_c;
                decoded(instr_imm12_lsb_c + 0)                        <= ci_instr16_i(2);
                decoded(instr_imm12_lsb_c + 1)                        <= ci_instr16_i(3);
                decoded(instr_imm12_lsb_c + 2)                        <= ci_instr16_i(4);
                decoded(instr_imm12_lsb_c + 3)                        <= ci_instr16_i(5);
                decoded(instr_imm12_lsb_c + 4)                        <= ci_instr16_i(6);
                if (ci_instr16_i(12) = '1') then -- nzuimm[5] = 1 -> RV32 custom
                  illegal <= '1';
                end if;
              when "10" => -- C.ANDI
                decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alui_c;
                decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_and_c;
                decoded(instr_imm12_msb_c downto instr_imm12_lsb_c)   <= (others => ci_instr16_i(12)); -- sign extend
                decoded(instr_imm12_lsb_c + 0)                        <= ci_instr16_i(2);
                decoded(instr_imm12_lsb_c + 1)                        <= ci_instr16_i(3);
                decoded(instr_imm12_lsb_c + 2)                        <= ci_instr16_i(4);
                decoded(instr_imm12_lsb_c + 3)                        <= ci_instr16_i(5);
                decoded(instr_imm12_lsb_c + 4)                        <= ci_instr16_i(6);
                decoded(instr_imm12_lsb_c + 5)                        <= ci_instr16_i(12);
              when others => -- "11" = register-register operation
                decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alu_c;
                case ci_instr16_i(6 downto 5) is
                  when "00" => -- C.SUB
                    decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_subadd_c;
                    decoded(instr_funct7_msb_c downto instr_funct7_lsb_c) <= "0100000";
                  when "01" => -- C.XOR
                    decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_xor_c;
                    decoded(instr_funct7_msb_c downto instr_funct7_lsb_c) <= "0000000";
                  when "10" => -- C.OR
                    decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_or_c;
                    decoded(instr_funct7_msb_c downto instr_funct7_lsb_c) <= "0000000";
                  when others => -- C.AND
                    decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_and_c;
                    decoded(instr_funct7_msb_c downto instr_funct7_lsb_c) <= "0000000";
                end case;
            end case;

        end case;

      when others => -- C2: Stack-Pointer-Based Loads and Stores, Control Transfer Instructions (or C3, which is not a RVC instruction)
        case ci_instr16_i(ci_funct3_msb_c downto ci_funct3_lsb_c) is

          when "000" => -- C.SLLI
          -- ----------------------------------------------------------------------------------------------------------
            decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alui_c;
            decoded(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= ci_instr16_i(ci_rs1_5_msb_c downto ci_rs1_5_lsb_c);
            decoded(instr_rd_msb_c downto instr_rd_lsb_c)         <= ci_instr16_i(ci_rs1_5_msb_c downto ci_rs1_5_lsb_c);
            decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_sll_c;
            decoded(instr_funct7_msb_c downto instr_funct7_lsb_c) <= "0000000";
            decoded(instr_imm12_lsb_c + 0)                        <= ci_instr16_i(2);
            decoded(instr_imm12_lsb_c + 1)                        <= ci_instr16_i(3);
            decoded(instr_imm12_lsb_c + 2)                        <= ci_instr16_i(4);
            decoded(instr_imm12_lsb_c + 3)                        <= ci_instr16_i(5);
            decoded(instr_imm12_lsb_c + 4)                        <= ci_instr16_i(6);
            if (ci_instr16_i(12) = '1') then -- nzuimm[5] = 1 -> RV32 custom
              illegal <= '1';
            end if;

          when "010" | "011" => -- C.LWSP / C.FLWSP (integer and float are identical as the FPU implements the Zfinx ISA extension)
          -- ----------------------------------------------------------------------------------------------------------
            decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_load_c;
            decoded(21 downto 20)                                 <= "00";
            decoded(22)                                           <= ci_instr16_i(4);
            decoded(23)                                           <= ci_instr16_i(5);
            decoded(24)                                           <= ci_instr16_i(6);
            decoded(25)                                           <= ci_instr16_i(12);
            decoded(26)                                           <= ci_instr16_i(2);
            decoded(27)                                           <= ci_instr16_i(3);
            decoded(31 downto 28)                                 <= (others => '0');
            decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_lw_c;
            decoded(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "00010"; -- stack pointer
            decoded(instr_rd_msb_c downto instr_rd_lsb_c)         <= ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c);
            if (ci_instr16_i(ci_funct3_lsb_c) = '1') and (FPU_ENABLE = false) then -- C.FLWSP
              illegal <= '1';
            end if;

          when "110" | "111" => -- C.SWSP / C.FSWSP (integer and float are identical as the FPU implements the Zfinx ISA extension)
          -- ----------------------------------------------------------------------------------------------------------
            decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_store_c;
            decoded(08 downto 07)                                 <= "00";
            decoded(09)                                           <= ci_instr16_i(9);
            decoded(10)                                           <= ci_instr16_i(10);
            decoded(11)                                           <= ci_instr16_i(11);
            decoded(25)                                           <= ci_instr16_i(12);
            decoded(26)                                           <= ci_instr16_i(7);
            decoded(27)                                           <= ci_instr16_i(8);
            decoded(31 downto 28)                                 <= (others => '0');
            decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= funct3_sw_c;
            decoded(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "00010"; -- stack pointer
            decoded(instr_rs2_msb_c downto instr_rs2_lsb_c)       <= ci_instr16_i(ci_rs2_5_msb_c downto ci_rs2_5_lsb_c);
            if (ci_instr16_i(ci_funct3_lsb_c) = '1') and (FPU_ENABLE = false) then -- C.FSWSP
              illegal <= '1';
            end if;

          when others => -- "100": C.JR, C.JALR, C.MV, C.EBREAK, C.ADD; others: undefined
          -- ----------------------------------------------------------------------------------------------------------
            if (ci_instr16_i(12) = '0') then -- C.JR, C.MV
              if (ci_instr16_i(6 downto 2) = "00000") then -- C.JR
                decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_jalr_c;
                decoded(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= ci_instr16_i(ci_rs1_5_msb_c downto ci_rs1_5_lsb_c);
                decoded(instr_rd_msb_c downto instr_rd_lsb_c)         <= "00000"; -- discard return address
              else -- C.MV
                decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alu_c;
                decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= "000";
                decoded(instr_rd_msb_c downto instr_rd_lsb_c)         <= ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c);
                decoded(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "00000"; -- x0
                decoded(instr_rs2_msb_c downto instr_rs2_lsb_c)       <= ci_instr16_i(ci_rs2_5_msb_c downto ci_rs2_5_lsb_c);
              end if;
            else -- C.EBREAK, C.JALR, C.ADD
              if (ci_instr16_i(6 downto 2) = "00000") then -- C.EBREAK, C.JALR
                if (ci_instr16_i(11 downto 7) = "00000") then -- C.EBREAK
                  decoded(instr_opcode_msb_c downto instr_opcode_lsb_c)   <= opcode_system_c;
                  decoded(instr_funct12_msb_c downto instr_funct12_lsb_c) <= funct12_ebreak_c;
                else -- C.JALR
                  decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_jalr_c;
                  decoded(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= ci_instr16_i(ci_rs1_5_msb_c downto ci_rs1_5_lsb_c);
                  decoded(instr_rd_msb_c downto instr_rd_lsb_c)         <= "00001"; -- save return address to link register
                end if;
              else -- C.ADD
                decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alu_c;
                decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= "000";
                decoded(instr_rd_msb_c downto instr_rd_lsb_c)         <= ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c);
                decoded(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c);
                decoded(instr_rs2_msb_c downto instr_rs2_lsb_c)       <= ci_instr16_i(ci_rs2_5_msb_c downto ci_rs2_5_lsb_c);
              end if;
            end if;
            --
            if (ci_instr16_i(ci_funct3_msb_c downto ci_funct3_lsb_c) = "001") or -- C.FLDSP / C.LQSP
               (ci_instr16_i(ci_funct3_msb_c downto ci_funct3_lsb_c) = "101") then -- C.FSDSP / C.SQSP
              illegal <= '1';
            end if;

        end case;

    end case;
  end process decompressor;

  -- output original 16-bit instruction word if illegal instruction --
  ci_instr32_o <= (x"0000" & ci_instr16_i) when (illegal = '1') else decoded;


end neorv32_cpu_decompressor_rtl;
