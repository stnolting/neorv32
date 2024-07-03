-- ================================================================================ --
-- NEORV32 CPU - Co-Processor: Custom (RISC-V Instructions) Functions Unit (CFU)    --
-- -------------------------------------------------------------------------------- --
-- For custom/user-defined RISC-V instructions See the  CPU's documentation for     --
-- more information. Also take a look at the "software-counterpart" this default    --
-- CFU hardware in 'sw/example/demo_cfu'.                                           --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

  -- **************************************************************************************************************************
  -- CFU Interface Documentation
  -- **************************************************************************************************************************

  -- ----------------------------------------------------------------------------------------
  -- Input Operands
  -- ----------------------------------------------------------------------------------------
  -- rs1_i    (input, 32-bit): source register 1; selected by instruction word's <rs1> bit-field
  -- rs2_i    (input, 32-bit): source register 2; selected by instruction word's <rs2> bit-field
  -- rs3_i    (input, 32-bit): source register 3; selected by instruction word's <rs3> bit-field
  -- rs4_i    (input, 32-bit): source register 4; selected by instruction word's <rs4> bit-field
  -- rtype_i  (input,  2-bit): instruction R-type; driven by instruction word's OPCODE
  -- funct3_i (input,  3-bit): 3-bit function select / immediate value; driven by instruction word's <funct3> bit-field
  -- funct7_i (input,  7-bit): 7-bit function select / immediate value; driven by instruction word's <funct7> bit-field
  --
  -- The general instruction type is identified by the <rtype_i> input.
  -- r3type_c  (= 00) - R3-type instructions  (custom-0 opcode): 'rs1', 'rs2' and 'funct7' and 'funct3'
  -- r4type_c  (= 01) - R4-type instructions  (custom-1 opcode): 'rs1', 'rs2', 'rs3' and 'funct3'
  -- r5typeA_c (= 10) - R5-type instruction A (custom-2 opcode): 'rs1', 'rs2', 'rs3', 'rs4', no immediates
  -- r5typeB_c (= 11) - R5-type instruction B (custom-3 opcode): 'rs1', 'rs2', 'rs3', 'rs4', no immediates
  --
  -- The four signals <rs1_i>, <rs2_i>, <rs3_i> and <rs4_i> provide the source operand data read from the CPU's register
  -- file. The source registers are adressed by the custom instruction word's <rs1>, <rs2>, <rs3> and <rs4> bit-fields.
  --
  -- [TIP] <rs1_i>, <rs2_i>, <rs3_i> and <rs4_i> are directly driven by the register file (e.g. block RAM). For complex CFU
  --       designs it is recommended to buffer these signals using CFU-internal registers before actually using them.
  --
  -- [NOTE] The R4-type instructions and R5-type instruction provide additional source register. When used, this will
  --        increase the hardware requirements of the register file.
  --
  -- The actual CFU operation can be defined by using the <funct3_i> and/or <funct7_i> signals (depending on the R-type).
  -- Both signals are driven by the according bit-fields of the custom instruction word. These immediates can be used to
  -- select the actual function or to provide small literals for certain operations (like shift amounts, offsets, ...).
  --
  -- [NOTE] All input operand signals remain stable during CFU operation.

  -- ----------------------------------------------------------------------------------------
  -- Processing Interface
  -- ----------------------------------------------------------------------------------------
  -- rstn_i   (input,   1-bit): asynchronous reset, low-active
  -- clk_i    (input,   1-bit): main clock, interface signals updated on the rising edge
  -- start_i  (input,   1-bit): operation trigger (start processing, high for one cycle)
  -- active_i (input,   1-bit): operation in progress while (optional signal)
  -- result_o (output, 32-bit): processing result
  -- valid_o  (output,  1-bit): set high when processing is done
  --
  -- The start of a new CFU operation is indicated by <start_i> being high for exactly one cycle. The CFU may operate while
  -- <active_i> is high and should stop all internal operations when it clears again. However, using this signal is optional.
  --
  -- When the CFU has completed computation, the data send via the <result_o> signal will be written to the CPU's register
  -- file (indexed by the "rd" register). The CPU pipeline samples this signal exactly one cycle after <valid_o> has been set.
  --
  -- [TIP] For complex CFU designs it is highly recommended to register <result_o> in order to keep the CPU's critical
  --       path as short as possible.
  --
  -- The <valid_o> signal is used to signal the completion of the CFU operation. For pure-combinatorial instructions
  -- (completing within 1 clock cycle) <valid_o> can be tied to 1. If the CFU requires several clock cycles for completion
  -- the <valid_o> signal has to be set high for one cycle EXACTLY ONE CYCLE before <result_o> is valid.
  --
  -- Example interface timing for a multi-cycle CFU operation ("D" represents the processing result in the output phase):
  -- clk_i    ____/----\____/----\____/----\____/----\____/----\____
  -- start_i  ____/---------\_______________________________________ trigger is high for one cycle
  -- active_i ____/-----------------------------\___________________ cease processing when low
  -- valid_o  ________________________/---------\___________________ set one cycle before output phase, zero otherwise
  -- result_o dddddddddddddddddddddddddddddddddd|DDDDDDDDD|ddddddddd don't care except for output phase
  --
  -- [NOTE] If the <valid_o> signal is not set within a bound time window (default = 512 cycles; see "monitor_mc_tmo_c"
  --        constant in the main NEORV32 package file) the CFU operation is automatically terminated by the hardware
  --        (clearing <active_i>) and an illegal instruction exception is raised.

  -- ----------------------------------------------------------------------------------------
  -- CFU-Internal Control and Status Registers (CFU-CSRs)
  -- ----------------------------------------------------------------------------------------
  -- csr_we_i    (input,   1-bit): set to indicate a valid CFU CSR write access, high for one cycle
  -- csr_addr_i  (input,   2-bit): CSR address
  -- csr_wdata_i (input,  32-bit): CSR write data
  -- csr_rdata_i (output, 32-bit): CSR read data
  --
  -- The NEORV32 provides four directly accessible CSRs for custom use inside the CFU. These registers can be used to pass
  -- further operands, to check the unit's status or to configure operation modes.
  --
  -- [TIP] If more than four CFU-internal CSRs are required the designer can implement an "indirect access mechanism" based
  --       on just two of the default CSRs: one CSR is used to configure the index while the other is used as an alias to
  --       exchange data with the indexed CFU-internal CSR.

  -- **************************************************************************************************************************
  -- Actual CFU User Logic Example: XTEA - Extended Tiny Encryption Algorithm (replace this with your custom logic)
  -- **************************************************************************************************************************

  -- This CFU example implements the Extended Tiny Encryption Algorithm (XTEA).
  -- The CFU provides 5 custom instructions to accelerate encryption and decryption using dedicated hardware.
  -- The RTL code is not optimized (not for area, not for clock speed, not for performance) and was
  -- implemented according to an open-source software C reference:
  -- https://de.wikipedia.org/wiki/Extended_Tiny_Encryption_Algorithm

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity neorv32_cpu_cp_cfu is
  port (
    -- global control --
    clk_i       : in  std_ulogic; -- global clock, rising edge
    rstn_i      : in  std_ulogic; -- global reset, low-active, async
    -- operation control --
    start_i     : in std_ulogic; -- operation trigger/strobe
    active_i    : in std_ulogic; -- operation in progress
    rtype_i     : in std_ulogic_vector(1 downto 0); -- instruction type, see constants below
    funct3_i    : in std_ulogic_vector(2 downto 0); -- "funct3" bit-field from custom instruction word
    funct7_i    : in std_ulogic_vector(6 downto 0); -- "funct7" bit-field from custom instruction word
    -- CSR interface --
    csr_we_i    : in  std_ulogic; -- write enable
    csr_addr_i  : in  std_ulogic_vector(1 downto 0); -- address
    csr_wdata_i : in  std_ulogic_vector(31 downto 0); -- write data
    csr_rdata_o : out std_ulogic_vector(31 downto 0) := (others => '0'); -- read data
    -- operands --
    rs1_i       : in  std_ulogic_vector(31 downto 0); -- rf source 1
    rs2_i       : in  std_ulogic_vector(31 downto 0); -- rf source 2
    rs3_i       : in  std_ulogic_vector(31 downto 0); -- rf source 3
    rs4_i       : in  std_ulogic_vector(31 downto 0); -- rf source 4
    -- result and status --
    result_o    : out std_ulogic_vector(31 downto 0) := (others => '0'); -- operation result
    valid_o     : out std_ulogic := '0' -- data output valid (one cycle ahead); operation done
  );
end neorv32_cpu_cp_cfu;

architecture neorv32_cpu_cp_cfu_rtl of neorv32_cpu_cp_cfu is

  -- instruction format types --
  constant r3type_c  : std_ulogic_vector(1 downto 0) := "00"; -- R3-type instructions (custom-0 opcode)
  constant r4type_c  : std_ulogic_vector(1 downto 0) := "01"; -- R4-type instructions (custom-1 opcode)
  constant r5typeA_c : std_ulogic_vector(1 downto 0) := "10"; -- R5-type instruction A (custom-2 opcode)
  constant r5typeB_c : std_ulogic_vector(1 downto 0) := "11"; -- R5-type instruction B (custom-3 opcode)

  -- instruction identifiers (funct3 bit-field) --
  constant xtea_enc_v0_c : std_ulogic_vector(2 downto 0) := "000";
  constant xtea_enc_v1_c : std_ulogic_vector(2 downto 0) := "001";
  constant xtea_dec_v0_c : std_ulogic_vector(2 downto 0) := "010";
  constant xtea_dec_v1_c : std_ulogic_vector(2 downto 0) := "011";
  constant xtea_init_c   : std_ulogic_vector(2 downto 0) := "100";

  -- round-key update --
  constant xtea_delta_c : std_ulogic_vector(31 downto 0) := x"9e3779b9";

  -- key storage (accessed via CFU CSRs) --
  type key_mem_t is array (0 to 3) of std_ulogic_vector(31 downto 0);
  signal key_mem : key_mem_t;

  -- processing logic --
  type xtea_t is record
    done : std_ulogic; -- multi-cycle done shift register
    opa  : std_ulogic_vector(31 downto 0); -- input operand a
    opb  : std_ulogic_vector(31 downto 0); -- input operand b
    sum  : std_ulogic_vector(31 downto 0); -- round key buffer
    res  : std_ulogic_vector(31 downto 0); -- operation results
  end record;
  signal xtea : xtea_t;

  -- helpers --
  signal tmp_a, tmp_b, tmp_x, tmp_y, tmp_z, tmp_r : std_ulogic_vector(31 downto 0);

begin

  -- CFU-Internal Control and Status Registers (CFU-CSRs): 128-Bit Key Storage --------------
  -- -------------------------------------------------------------------------------------------
  -- synchronous write access --
  csr_write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      key_mem <= (others => (others => '0'));
    elsif rising_edge(clk_i) then
      if (csr_we_i = '1') then
        key_mem(to_integer(unsigned(csr_addr_i))) <= csr_wdata_i;
      end if;
    end if;
  end process csr_write_access;

  -- asynchronous read access --
  csr_rdata_o <= key_mem(to_integer(unsigned(csr_addr_i)));


  -- XTEA Processing Core ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  xtea_core: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      xtea.done <= '0';
      xtea.opa  <= (others => '0');
      xtea.opb  <= (others => '0');
      xtea.sum  <= (others => '0');
      xtea.res  <= (others => '0');
    elsif rising_edge(clk_i) then

      -- trigger new operation --
      xtea.done <= '0'; -- default
      if (start_i = '1') and (rtype_i = r3type_c) then -- execution trigger and correct instruction type
        xtea.opa  <= rs1_i; -- buffer input operand rs1 (for improved physical timing)
        xtea.opb  <= rs2_i; -- buffer input operand rs2 (for improved physical timing)
        xtea.done <= '1'; -- result is available in the 2nd cycle
      end if;

      -- data processing --
      if (xtea.done = '1') then -- second-stage execution trigger
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
  result_select: process(rtype_i, funct3_i, xtea)
  begin
    case rtype_i is -- check instruction type

      when r3type_c => -- R3-type instructions; function select via "funct3" and ""funct7
      -- ----------------------------------------------------------------------
        case funct3_i is -- Just check "funct3" here; "funct7" bit-field is ignored
          when xtea_enc_v0_c | xtea_enc_v1_c | xtea_dec_v0_c | xtea_dec_v1_c => -- encryption/decryption
            result_o <= xtea.res; -- processing result
            valid_o  <= xtea.done; -- multi-cycle processing done when set
          when xtea_init_c => -- initialization
            result_o <= (others => '0'); -- just output zero
            valid_o  <= '1'; -- pure-combinatorial, so we are done "immediately"
          when others => -- all unspecified operations
            result_o <= (others => '0'); -- no logic implemented
            valid_o  <= '0'; -- this will cause an illegal instruction exception after timeout
        end case;

      when r4type_c => -- R4-type instructions; function select via "funct3"
      -- ----------------------------------------------------------------------
        result_o <= (others => '0'); -- no logic implemented
        valid_o  <= '0'; -- this will cause an illegal instruction exception after timeout

      when r5typeA_c => -- R5-type instruction A; only one function
      -- ----------------------------------------------------------------------
        result_o <= (others => '0'); -- no logic implemented
        valid_o  <= '0'; -- this will cause an illegal instruction exception after timeout

      when r5typeB_c => -- R5-type instruction B; only one function
      -- ----------------------------------------------------------------------
        result_o <= (others => '0'); -- no logic implemented
        valid_o  <= '0'; -- this will cause an illegal instruction exception after timeout

      when others => -- undefined
      -- ----------------------------------------------------------------------
        result_o <= (others => '0'); -- no logic implemented
        valid_o  <= '0'; -- this will cause an illegal instruction exception after timeout

    end case;
  end process result_select;


end neorv32_cpu_cp_cfu_rtl;
