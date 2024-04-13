-- ================================================================================ --
-- NEORV32 CPU - Compressed Instructions Decoder (RISC-V "C" Extension)             --
-- -------------------------------------------------------------------------------- --
-- Compressed instructions decoder compatible to the RISC-V C ISA extension.        --
-- Illegal compressed instructions are output "as-is" but zero-extended.            --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cpu_decompressor is
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

          when "000" => -- Illegal_instruction, C.ADDI4SPN
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
            if (ci_instr16_i(12 downto 5) = "00000000") then -- canonical illegal C instruction or C.ADDI4SPN with nzuimm = 0
              illegal <= '1';
            end if;

          when "010" => -- C.LW
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

          when "110" => -- C.SW
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

          when others => -- "011": C.FLW, "111": C.FSW, "001": C.FLS / C.LQ, "100": reserved, "101": C.FSD / C.SQ
          -- ----------------------------------------------------------------------------------------------------------
            illegal <= '1';

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
                if (ci_instr16_i(12) = '1') then -- nzuimm[5] = 1 -> RV32 custom / illegal
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
                if (ci_instr16_i(12) = '1') then -- reserved instruction space.
                  illegal <= '1';
                end if;
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

          when "010" | "011" => -- C.LWSP / C.FLWSP
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
            if (ci_instr16_i(ci_funct3_lsb_c) = '1') or -- C.FLWSP -> illegal
               (ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c) = "00000") then -- rd = 0 -> reserved
              illegal <= '1';
            end if;

          when "110" | "111" => -- C.SWSP / C.FSWSP
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
            if (ci_instr16_i(ci_funct3_lsb_c) = '1') then -- C.FSWSP -> illegal
              illegal <= '1';
            end if;

          when "100" => -- "100": C.JR, C.JALR, C.MV, C.EBREAK, C.ADD
          -- ----------------------------------------------------------------------------------------------------------
            if (ci_instr16_i(12) = '0') then -- C.JR, C.MV
              if (ci_instr16_i(6 downto 2) = "00000") then -- C.JR
                decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_jalr_c;
                decoded(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= ci_instr16_i(ci_rs1_5_msb_c downto ci_rs1_5_lsb_c);
                decoded(instr_rd_msb_c downto instr_rd_lsb_c)         <= "00000"; -- discard return address
                if (ci_instr16_i(ci_rs1_5_msb_c downto ci_rs1_5_lsb_c) = "00000") or -- rs1 = 0 -> reserved
                   (ci_instr16_i(ci_rs2_5_msb_c downto ci_rs2_5_lsb_c) /= "00000") then -- rs2 != 0 -> illegal
                  illegal <= '1';
                end if;
              else -- C.MV
                decoded(instr_opcode_msb_c downto instr_opcode_lsb_c) <= opcode_alu_c;
                decoded(instr_funct3_msb_c downto instr_funct3_lsb_c) <= "000";
                decoded(instr_rd_msb_c downto instr_rd_lsb_c)         <= ci_instr16_i(ci_rd_5_msb_c downto ci_rd_5_lsb_c);
                decoded(instr_rs1_msb_c downto instr_rs1_lsb_c)       <= "00000"; -- x0
                decoded(instr_rs2_msb_c downto instr_rs2_lsb_c)       <= ci_instr16_i(ci_rs2_5_msb_c downto ci_rs2_5_lsb_c);
                if (ci_instr16_i(ci_rs2_5_msb_c downto ci_rs2_5_lsb_c) = "00000") then -- rs2 = 0 -> reserved
                  illegal <= '1';
                end if;
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

          when others => -- "001"/"101": C.FLDSP / C.LQSP, C.FSDSP / C.SQSP -> illegal
          -- ----------------------------------------------------------------------------------------------------------
            illegal <= '1';

        end case;

    end case;
  end process decompressor;

  -- output original 16-bit instruction word if illegal instruction --
  ci_instr32_o <= (x"0000" & ci_instr16_i) when (illegal = '1') else decoded;


end neorv32_cpu_decompressor_rtl;
