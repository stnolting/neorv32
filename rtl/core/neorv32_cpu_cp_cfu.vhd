-- ================================================================================ --
-- NEORV32 CPU - Co-Processor: Custom (RISC-V Instructions) Functions Unit (CFU)    --
-- -------------------------------------------------------------------------------- --
-- For custom/user-defined RISC-V instructions. See the CPU's documentation for     --
-- more information. Also take a look at the "software-counterpart" of this default --
-- CFU hardware example in 'sw/example/demo_cfu'.                                   --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity neorv32_cpu_cp_cfu is
  port (
    -- global control --
    clk_i    : in  std_ulogic; -- global clock, rising edge
    rstn_i   : in  std_ulogic; -- global reset, low-active, async
    -- operation trigger --
    start_i  : in  std_ulogic; -- start trigger, single-shot
    -- operands --
    type_i   : in  std_ulogic; -- instruction type (0 = R-type, 1 = I-type)
    funct3_i : in  std_ulogic_vector(2 downto 0);  -- "funct3" bit-field from instruction word
    funct7_i : in  std_ulogic_vector(6 downto 0);  -- "funct7" bit-field from instruction word (R-type only)
    imm12_i  : in  std_ulogic_vector(11 downto 0); -- "imm12" bit-field from instruction word (I-type only)
    rs1_i    : in  std_ulogic_vector(31 downto 0); -- rf source 1 via "rs1" bit-field from instruction word
    rs2_i    : in  std_ulogic_vector(31 downto 0); -- rf source 2 via "rs2" bit-field from instruction word
    -- result and status --
    result_o : out std_ulogic_vector(31 downto 0); -- operation result
    valid_o  : out std_ulogic -- result valid, operation done; set one cycle before result_o is valid
  );
end neorv32_cpu_cp_cfu;

  -- **************************************************************************************************************************
  -- CFU Interface Documentation
  -- **************************************************************************************************************************

  -- ----------------------------------------------------------------------------------------
  -- Input Operands
  -- ----------------------------------------------------------------------------------------
  -- > type_i   (input,  1-bit): instruction R-type (R-type or I-type); driven by the instruction word's OPCODE bit 5
  -- > funct3_i (input,  3-bit): 3-bit function select / immediate value; driven by instruction word's <funct3> bit-field
  -- > funct7_i (input,  7-bit): 7-bit function select / immediate value; driven by instruction word's <funct7> bit-field, R-type only
  -- > imm12_i  (input, 12-bit): 12-bit immediate value; driven by instruction word's <imm12> bit-field, I-type only
  -- > rs1_i    (input, 32-bit): source register 1; selected by instruction word's <rs1> bit-field
  -- > rs2_i    (input, 32-bit): source register 2; selected by instruction word's <rs2> bit-field, R-type only
  --
  -- The general instruction type is identified by the <type_i> input.
  -- r_type_c (= 0): R-type instruction (RISC-V custom-0 opcode); 'rs1', 'rs2', 'funct7' and 'funct3'
  -- i_type_c (= 1): I-type instruction (RISC-V custom-1 opcode); 'rs1', 'imm12' and 'funct3'
  --
  -- The signals <rs1_i> and <rs2_i> provide the actual source operand data read from the CPU's register
  -- file. These register operands are addressed by the custom instruction word's 'rs1' and 'rs2' bit-fields.
  -- The actual CFU operation can be defined by using the <funct3_i> and/or <funct7_i> signals.
  -- Both signals are driven by the according bit-fields of the custom instruction word.
  --
  -- [NOTE] All input operands/signals remain stable until the CFU operation has completed.

  -- ----------------------------------------------------------------------------------------
  -- Operation Control, Result and Status
  -- ----------------------------------------------------------------------------------------
  -- > start_i  (input,   1-bit): operation trigger (start CFU operation, high for one cycle)
  -- > result_o (output, 32-bit): processing result
  -- > valid_o  (output,  1-bit): set high (for one cycle) when processing is done
  --
  -- The start of a new CFU operation is indicated by <start_i> being high for exactly one cycle. When the CFU has completed
  -- processing, the data returned via <result_o> will be written to the CPU's register file (indexed by the 'rd' bit-field).
  --
  -- The <valid_o> signal is used to signal the completion of the CFU operation. For pure-combinatorial instructions
  -- (completing within 1 clock cycle) <valid_o> can be hardwired to 1. If the CFU requires several clock cycles for
  -- completion the <valid_o> signal has to be set when <result_o> is valid. Otherwise, <valid_o> should be cleared.
  --
  -- Example interface timing for a multi-cycle CFU operation:
  -- clk_i    ____/----\____/----\____/----\____/----\____
  -- start_i  ____/---------\_____________________________ trigger is high for one cycle
  -- valid_o  ________________________/---------\_________ set for one cycle when operation is done / result is valid
  -- result_o ........................|DDDDDDDDD|......... don't care ('.') except for valid-output phase ('D')
  --
  -- [NOTE] If the <valid_o> signal is not set within a bound time after the CFU has been triggered via <start_i>
  --        (default = 512 cycles; see "monitor_mc_tmo_c" constant in the main NEORV32 package file) the CFU operation
  --        is automatically terminated by the hardware and an illegal instruction exception is raised.


  -- **************************************************************************************************************************
  -- Actual CFU User Logic Example: XTEA - Extended Tiny Encryption Algorithm (replace this with your custom logic)
  -- **************************************************************************************************************************

  -- This CFU example implements the Extended Tiny Encryption Algorithm (XTEA).
  -- The CFU provides five r3-type instructions for encryption and decryption.
  -- Furthermore, four r4-type instruction are used for reading/writing the XTEA key registers.

  -- The RTL code was implemented according to an open-source software C reference:
  -- https://de.wikipedia.org/wiki/Extended_Tiny_Encryption_Algorithm

architecture neorv32_cpu_cp_cfu_rtl of neorv32_cpu_cp_cfu is

  -- CFU instruction type formats --
  constant r_type_c : std_ulogic := '0'; -- R-type CFU instructions (custom-0 opcode)
  constant i_type_c : std_ulogic := '1'; -- I-type CFU instructions (custom-1 opcode)

  -- instruction identifiers (funct3 bit-field) --
  constant xtea_enc_v0_c : std_ulogic_vector(2 downto 0) := "000";
  constant xtea_enc_v1_c : std_ulogic_vector(2 downto 0) := "001";
  constant xtea_dec_v0_c : std_ulogic_vector(2 downto 0) := "010";
  constant xtea_dec_v1_c : std_ulogic_vector(2 downto 0) := "011";
  constant xtea_init_c   : std_ulogic_vector(2 downto 0) := "100";

  -- round-key update --
  constant xtea_delta_c : std_ulogic_vector(31 downto 0) := x"9e3779b9";

  -- key storage (accessed via special r4-type instructions) --
  type key_mem_t is array (0 to 3) of std_ulogic_vector(31 downto 0);
  signal key_mem : key_mem_t;

  -- processing logic --
  type xtea_t is record
    done : std_ulogic_vector(1 downto 0); -- multi-cycle done shift register; 2 stages = 2 cyles latency
    opa  : std_ulogic_vector(31 downto 0); -- input operand a
    opb  : std_ulogic_vector(31 downto 0); -- input operand b
    sum  : std_ulogic_vector(31 downto 0); -- round key buffer
    res  : std_ulogic_vector(31 downto 0); -- operation results
  end record;
  signal xtea : xtea_t;

  -- helpers --
  signal tmp_a, tmp_b, tmp_x, tmp_y, tmp_z, tmp_r : std_ulogic_vector(31 downto 0);

begin

  -- XTEA Processing Core ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  xtea_core: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      xtea.done <= (others => '0');
      xtea.opa  <= (others => '0');
      xtea.opb  <= (others => '0');
      xtea.sum  <= (others => '0');
      xtea.res  <= (others => '0');
      key_mem   <= (others => (others => '0'));
    elsif rising_edge(clk_i) then
      -- "operation-done" shift register: module has 2 cycles latency --
      xtea.done(0) <= '0'; -- default: no operation trigger
      xtea.done(1) <= xtea.done(0); -- arbitration shift register

      -- trigger new operation --
      if (start_i = '1') then
        if (type_i = r_type_c) then -- R-type for computational instructions
          xtea.opa     <= rs1_i; -- buffer input operand rs1
          xtea.opb     <= rs2_i; -- buffer input operand rs2
          xtea.done(0) <= '1'; -- start data processing
        else -- I-type is used for key access instructions
          if (funct3_i(0) = '1') then -- key write-enable
            key_mem(to_integer(unsigned(imm12_i(1 downto 0)))) <= rs1_i; -- write key data at imm12(1:0)
          end if;
        end if;
      end if;

      -- data processing --
      if (xtea.done(0) = '1') then -- second-stage execution trigger
        -- update "sum" round key --
        if (funct3_i(2) = '1') then -- initialize
          xtea.sum <= xtea.opa; -- set initial round key
        elsif (funct3_i(1 downto 0) = xtea_enc_v0_c(1 downto 0)) then -- encrypt v0
          xtea.sum <= std_ulogic_vector(unsigned(xtea.sum) + unsigned(xtea_delta_c));
        elsif (funct3_i(1 downto 0) = xtea_dec_v1_c(1 downto 0)) then -- decrypt v1
          xtea.sum <= std_ulogic_vector(unsigned(xtea.sum) - unsigned(xtea_delta_c));
        end if;
        -- process "v" operands --
        if (funct3_i(1) = '0') then -- encrypt
          xtea.res <= std_ulogic_vector(unsigned(tmp_b) + unsigned(tmp_r));
        else -- decrypt
          xtea.res <= std_ulogic_vector(unsigned(tmp_b) - unsigned(tmp_r));
        end if;
      end if;

    end if;
  end process xtea_core;

  -- helpers --
  tmp_a <= xtea.opb when (funct3_i(0) = '0') else xtea.opa; -- v1 / v0 select
  tmp_b <= xtea.opa when (funct3_i(0) = '0') else xtea.opb; -- v0 / v1 select
  tmp_x <= xtea.opb(27 downto 0) & "0000"  when (funct3_i(0) = '0') else xtea.opa(27 downto 0) & "0000";  -- v << 4
  tmp_y <= "00000" & xtea.opb(31 downto 5) when (funct3_i(0) = '0') else "00000" & xtea.opa(31 downto 5); -- v >> 5
  tmp_z <= key_mem(to_integer(unsigned(xtea.sum(1 downto 0)))) when (funct3_i(0) = '0') else -- key[sum & 3]
           key_mem(to_integer(unsigned(xtea.sum(12 downto 11)))); -- key[(sum >> 11) & 3]
  tmp_r <= std_ulogic_vector(unsigned(tmp_x xor tmp_y) + unsigned(tmp_a)) xor std_ulogic_vector(unsigned(xtea.sum) + unsigned(tmp_z));


  -- Function Result Select -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  result_select: process(type_i, funct3_i, imm12_i, xtea, key_mem)
  begin
    if (type_i = r_type_c) then -- R-type instructions; function select via "funct3" and ""funct7
    -- ----------------------------------------------------------------------
      case funct3_i is -- just check "funct3" here; "funct7" bit-field is ignored in this example
        when xtea_enc_v0_c | xtea_enc_v1_c | xtea_dec_v0_c | xtea_dec_v1_c => -- encryption/decryption
          result_o <= xtea.res; -- processing result
          valid_o  <= xtea.done(1); -- multi-cycle processing done when set
        when xtea_init_c => -- xtea initialization
          result_o <= (others => '0'); -- just output zero
          valid_o  <= '1'; -- pure-combinatorial, so we are done "immediately"
        when others => -- all unspecified operations
          result_o <= (others => '0'); -- no logic implemented
          valid_o  <= '0'; -- this will cause an illegal instruction exception
      end case;

    else -- I-type instructions; used for key access
    -- ----------------------------------------------------------------------
      result_o <= key_mem(to_integer(unsigned(imm12_i(1 downto 0))));
      valid_o  <= '1'; -- pure-combinatorial, so we are done "immediately"

    end if;
  end process result_select;


end neorv32_cpu_cp_cfu_rtl;
