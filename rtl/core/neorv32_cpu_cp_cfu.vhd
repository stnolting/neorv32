-- ================================================================================ --
-- NEORV32 CPU - Co-Processor: Custom (RISC-V Instructions) Functions Unit (CFU)    --
-- -------------------------------------------------------------------------------- --
-- For custom/user-defined RISC-V instructions (R3-type, R4-type and R5-type        --
-- formats). See the  CPU's documentation for more information. Also take a look at --
-- the "software-counterpart" this default CFU hardware in 'sw/example/demo_cfu'.   --
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

entity neorv32_cpu_cp_cfu is
  port (
    -- global control --
    clk_i       : in  std_ulogic; -- global clock, rising edge
    rstn_i      : in  std_ulogic; -- global reset, low-active, async
    ctrl_i      : in  ctrl_bus_t; -- main control bus
    start_i     : in  std_ulogic; -- trigger operation
    -- CSR interface --
    csr_we_i    : in  std_ulogic; -- write enable
    csr_addr_i  : in  std_ulogic_vector(1 downto 0); -- address
    csr_wdata_i : in  std_ulogic_vector(XLEN-1 downto 0); -- write data
    csr_rdata_o : out std_ulogic_vector(XLEN-1 downto 0) := (others => '0'); -- read data
    -- data input --
    rs1_i       : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 1
    rs2_i       : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 2
    rs3_i       : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 3
    rs4_i       : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 4
    -- result and status --
    res_o       : out std_ulogic_vector(XLEN-1 downto 0) := (others => '0'); -- operation result
    valid_o     : out std_ulogic := '0' -- data output valid
  );
end neorv32_cpu_cp_cfu;

architecture neorv32_cpu_cp_cfu_rtl of neorv32_cpu_cp_cfu is

  -- CFU Control ---------------------------------------------
  -- ------------------------------------------------------------
  type control_t is record
    busy   : std_ulogic; -- CFU is busy
    done   : std_ulogic; -- set to '1' when processing is done
    result : std_ulogic_vector(XLEN-1 downto 0); -- CFU processing result (for write-back to register file)
    rtype  : std_ulogic_vector(1 downto 0); -- instruction type, see constants below
    funct3 : std_ulogic_vector(2 downto 0); -- "funct3" bit-field from custom instruction word
    funct7 : std_ulogic_vector(6 downto 0); -- "funct7" bit-field from custom instruction word
  end record;
  signal control : control_t;

  -- instruction format types --
  constant r3type_c  : std_ulogic_vector(1 downto 0) := "00"; -- R3-type instructions (custom-0 opcode)
  constant r4type_c  : std_ulogic_vector(1 downto 0) := "01"; -- R4-type instructions (custom-1 opcode)
  constant r5typeA_c : std_ulogic_vector(1 downto 0) := "10"; -- R5-type instruction A (custom-2 opcode)
  constant r5typeB_c : std_ulogic_vector(1 downto 0) := "11"; -- R5-type instruction B (custom-3 opcode)

  -- User-Defined Logic --------------------------------------
  -- ------------------------------------------------------------
  -- xtea instructions (funct3 bit-field) --
  constant xtea_enc_v0_c : std_ulogic_vector(2 downto 0) := "000";
  constant xtea_enc_v1_c : std_ulogic_vector(2 downto 0) := "001";
  constant xtea_dec_v0_c : std_ulogic_vector(2 downto 0) := "010";
  constant xtea_dec_v1_c : std_ulogic_vector(2 downto 0) := "011";
  constant xtea_init_c   : std_ulogic_vector(2 downto 0) := "100";

  -- xtea round-key adjusting --
  constant xtea_delta_c : std_ulogic_vector(31 downto 0) := x"9e3779b9";

  -- xtea key storage (accessed via CFU CSRs) --
  type key_mem_t is array (0 to 3) of std_ulogic_vector(31 downto 0);
  signal key_mem : key_mem_t;

  -- xtea processing logic --
  type xtea_t is record
    done : std_ulogic_vector(1 downto 0);  -- multi-cycle operation SREG
    opa  : std_ulogic_vector(31 downto 0); -- input operand a
    opb  : std_ulogic_vector(31 downto 0); -- input operand b
    sum  : std_ulogic_vector(31 downto 0); -- round key buffer
    res  : std_ulogic_vector(31 downto 0); -- operation results
  end record;
  signal xtea : xtea_t;

  -- xtea helper --
  signal tmp_a, tmp_b, tmp_x, tmp_y, tmp_z, tmp_r : std_ulogic_vector(31 downto 0);

begin

  -- **************************************************************************************************************************
  -- This controller is required to handle the CFU-CPU interface.
  -- **************************************************************************************************************************

  -- CFU Controller -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- The <control> record acts as proxy logic that ensures correct communication with the
  -- CPU pipeline. However, this control instance adds one additional cycle of latency.
  -- Advanced users can remove this default control instance to obtain maximum throughput.
  cfu_control: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      res_o        <= (others => '0');
      control.busy <= '0';
    elsif rising_edge(clk_i) then
      res_o <= (others => '0'); -- default; all CPU co-processor outputs are logically OR-ed
      if (control.busy = '0') then -- CFU is idle
        control.busy <= start_i; -- trigger new CFU operation
      else -- CFU operation in progress
        res_o <= control.result; -- output result only if CFU is processing; has to be all-zero otherwise
        if (control.done = '1') or (ctrl_i.cpu_trap = '1') then -- operation done or abort if trap (exception)
          control.busy <= '0';
        end if;
      end if;
    end if;
  end process cfu_control;

  -- CPU feedback --
  valid_o <= control.busy and control.done; -- set one cycle before result data

  -- pack user-defined instruction type/function bits --
  control.rtype  <= ctrl_i.ir_opcode(6 downto 5);
  control.funct3 <= ctrl_i.ir_funct3;
  control.funct7 <= ctrl_i.ir_funct12(11 downto 5);


  -- **************************************************************************************************************************
  -- CFU Hardware Documentation
  -- **************************************************************************************************************************

  -- ----------------------------------------------------------------------------------------
  -- CFU Instruction Formats
  -- ----------------------------------------------------------------------------------------
  -- The CFU supports three instruction types:
  --
  -- Up to 1024 RISC-V R3-Type Instructions (RISC-V standard):
  -- This format consists of two source registers ('rs1', 'rs2'), a destination register ('rd') and two "immediate" bit-fields
  -- ('funct7' and 'funct3').
  --
  -- Up to 8 RISC-V R4-Type Instructions (RISC-V standard):
  -- This format consists of three source registers ('rs1', 'rs2', 'rs3'), a destination register ('rd') and one "immediate"
  -- bit-field ('funct3').
  --
  -- Two individual RISC-V R5-Type Instructions (NEORV32-specific):
  -- This format consists of four source registers ('rs1', 'rs2', 'rs3', 'rs4') and a destination register ('rd'). There are
  -- no immediate fields.

  -- ----------------------------------------------------------------------------------------
  -- Input Operands
  -- ----------------------------------------------------------------------------------------
  -- > rs1_i          (input, 32-bit): source register 1; selected by 'rs1' bit-field
  -- > rs2_i          (input, 32-bit): source register 2; selected by 'rs2' bit-field
  -- > rs3_i          (input, 32-bit): source register 3; selected by 'rs3' bit-field
  -- > rs4_i          (input, 32-bit): source register 4; selected by 'rs4' bit-field
  -- > control.rtype  (input,  2-bit): defining the R-type; driven by OPCODE
  -- > control.funct3 (input,  3-bit): 3-bit function select / immediate value; driven by instruction word's 'funct3' bit-field
  -- > control.funct7 (input,  7-bit): 7-bit function select / immediate value; driven by instruction word's 'funct7' bit-field
  --
  -- [NOTE] The set of usable signals depends on the actual R-type of the instruction.
  --
  -- The general instruction type is identified by the <control.rtype>.
  -- > r3type_c  - R3-type instructions (custom-0 opcode)
  -- > r4type_c  - R4-type instructions (custom-1 opcode)
  -- > r5typeA_c - R5-type instruction A (custom-2 opcode)
  -- > r5typeB_c - R5-type instruction B (custom-3 opcode)
  --
  -- The four signals <rs1_i>, <rs2_i>, <rs3_i> and <rs4_i> provide the source operand data read from the CPU's register file.
  -- The source registers are adressed by the custom instruction word's 'rs1', 'rs2', 'rs3' and 'rs4' bit-fields.
  --
  -- The actual CFU operation can be defined by using the <control.funct3> and/or <control.funct7> signals (if available for a
  -- certain R-type instruction). Both signals are directly driven by the according bit-fields of the custom instruction word.
  -- These immediates can be used to select the actual function or to provide small literals for certain operations (like shift
  -- amounts, offsets, multiplication factors, ...).
  --
  -- [NOTE] <rs1_i>, <rs2_i>, <rs3_i> and <rs4_i> are directly driven by the register file (e.g. block RAM). For complex CFU
  --        designs it is recommended to buffer these signals using CFU-internal registers before actually using them.
  --
  -- [NOTE] The R4-type instructions and R5-type instruction provide additional source register. When used, this will increase
  --        the hardware requirements of the register file.

  -- ----------------------------------------------------------------------------------------
  -- Result Output
  -- ----------------------------------------------------------------------------------------
  -- > control.result (output, 32-bit): processing result
  --
  -- When the CFU has completed computations, the data send via the <control.result> signal will be written to the CPU's register
  -- file. The destination register is addressed by the <rd> bit-field in the instruction word. The CFU result output is registered
  -- in the CFU controller (see above) - so do not worry too much about increasing the CPU's critical path with your custom
  -- logic.

  -- ----------------------------------------------------------------------------------------
  -- Processing Control
  -- ----------------------------------------------------------------------------------------
  -- > rstn_i       (input,  1-bit): asynchronous reset, low-active
  -- > clk_i        (input,  1-bit): main clock, triggering on rising edge
  -- > start_i      (input,  1-bit): operation trigger (start processing, high for one cycle)
  -- > control.done (output, 1-bit): set high when processing is done
  --
  -- For pure-combinatorial instructions (completing within 1 clock cycle) <control.done> can be tied to 1. If the CFU requires
  -- several clock cycles for internal processing, the <start_i> signal can be used to *start* a new iterative operation. As soon
  -- as all internal computations have completed, the <control.done> signal has to be set to indicate completion. This will
  -- complete CFU instruction operation and will also write the processing result <control.result> back to the CPU register file.

  -- ----------------------------------------------------------------------------------------
  -- CFU Exception
  -- ----------------------------------------------------------------------------------------
  -- The CFU does not provide a dedicated exception mechanism. However, if the <control.done> signal is not set within a bound
  -- time window (default = 512 cycles; see "monitor_mc_tmo_c" constant in the main NEORV32 package file) the CFU operation is
  -- automatically terminated by the hardware and an **illegal instruction exception** is raised. This default mechanism combined
  -- with according software handling can be used to "emulate" dedicated CFU exceptions.

  -- ----------------------------------------------------------------------------------------
  -- CFU-Internal Control and Status Registers (CFU-CSRs)
  -- ----------------------------------------------------------------------------------------
  -- > csr_we_i    (input,   1-bit): set to indicate a valid CFU CSR write access
  -- > csr_addr_i  (input,   2-bit): CSR address
  -- > csr_wdata_i (input,  32-bit): CSR write data
  -- > csr_rdata_i (output, 32-bit): CSR read data
  --
  -- The NEORV32 provides four directly accessible CSRs for custom use inside the CFU. These registers can be used to pass
  -- further operands, to check the unit's status or to configure operation modes. For instance, a 128-bit wide key could be
  -- passed to an encryption system.
  --
  -- If more than four CFU-internal CSRs are required the designer can implement an "indirect access mechanism" based on just
  -- two of the default CSRs: one CSR is used to configure the index while the other is used as an alias to exchange data with
  -- the indexed CFU-internal CSR - this concept is similar to the RISC-V Indirect CSR Access Extension Specification (Smcsrind).


  -- **************************************************************************************************************************
  -- Actual CFU User Logic Example: XTEA - Extended Tiny Encryption Algorithm (replace this with your custom logic)
  -- **************************************************************************************************************************

  -- This CFU example implements the Extended Tiny Encryption Algorithm (XTEA).
  -- The CFU provides 5 custom instructions to accelerate encryption and decryption using dedicated hardware.
  -- The RTL code is not optimized (not for area, not for clock speed, not for performance) and was
  -- implemented according to a software C reference (https://de.wikipedia.org/wiki/Extended_Tiny_Encryption_Algorithm).


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
      xtea.done <= (others => '0');
      xtea.opa  <= (others => '0');
      xtea.opb  <= (others => '0');
      xtea.sum  <= (others => '0');
      xtea.res  <= (others => '0');
    elsif rising_edge(clk_i) then

      -- shift register for computation delay --
      xtea.done(0) <= '0'; -- default
      xtea.done(1) <= xtea.done(0);

      -- trigger new operation --
      if (start_i = '1') and (control.rtype = r3type_c) then -- execution trigger and correct instruction type
        xtea.opa     <= rs1_i; -- buffer input operand rs1 (for improved physical timing)
        xtea.opb     <= rs2_i; -- buffer input operand rs2 (for improved physical timing)
        xtea.done(0) <= '1';   -- result is available in the 2nd cycle
      end if;

      -- data processing --
      if (xtea.done(0) = '1') then -- second-stage execution trigger
        -- update "sum" round key --
        if (control.funct3(2) = '1') then -- initialize
          xtea.sum <= xtea.opa; -- set initial round key
        elsif (control.funct3(1 downto 0) = xtea_enc_v0_c(1 downto 0)) then -- encrypt v0
          xtea.sum <= std_ulogic_vector(unsigned(xtea.sum) + unsigned(xtea_delta_c));
        elsif (control.funct3(1 downto 0) = xtea_dec_v1_c(1 downto 0)) then -- decrypt v1
          xtea.sum <= std_ulogic_vector(unsigned(xtea.sum) - unsigned(xtea_delta_c));
        end if;
        -- process "v" operands --
        if (control.funct3(1) = '0') then -- encrypt
          xtea.res <= std_ulogic_vector(unsigned(tmp_b) + unsigned(tmp_r));
        else -- decrypt
          xtea.res <= std_ulogic_vector(unsigned(tmp_b) - unsigned(tmp_r));
        end if;
      end if;

    end if;
  end process xtea_core;

  -- helpers --
  tmp_a <= xtea.opb when (control.funct3(0) = '0') else xtea.opa; -- v1 / v0 select
  tmp_b <= xtea.opa when (control.funct3(0) = '0') else xtea.opb; -- v0 / v1 select
  tmp_x <= xtea.opb(27 downto 0) & "0000"  when (control.funct3(0) = '0') else xtea.opa(27 downto 0) & "0000";  -- v << 4
  tmp_y <= "00000" & xtea.opb(31 downto 5) when (control.funct3(0) = '0') else "00000" & xtea.opa(31 downto 5); -- v >> 5
  tmp_z <= key_mem(to_integer(unsigned(xtea.sum(1 downto 0)))) when (control.funct3(0) = '0') else -- key[sum & 3]
           key_mem(to_integer(unsigned(xtea.sum(12 downto 11)))); -- key[(sum >> 11) & 3]
  tmp_r <= std_ulogic_vector(unsigned(tmp_x xor tmp_y) + unsigned(tmp_a)) xor std_ulogic_vector(unsigned(xtea.sum) + unsigned(tmp_z));


  -- Function Result Select -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  result_select: process(control, xtea)
  begin
    case control.rtype is

      when r3type_c => -- R3-type instructions; function select via "funct7" and "funct3"
      -- ----------------------------------------------------------------------
        case control.funct3 is -- Just check "funct3" here; "funct7" bit-field is ignored
          when xtea_enc_v0_c | xtea_enc_v1_c | xtea_dec_v0_c | xtea_dec_v1_c => -- encryption/decryption
            control.result <= xtea.res; -- processing result
            control.done   <= xtea.done(xtea.done'left); -- multi-cycle processing done when set
          when xtea_init_c => -- initialization
            control.result <= (others => '0'); -- just output zero
            control.done   <= '1'; -- pure-combinatorial, so we are done "immediately"
          when others => -- all unspecified operations
            control.result <= (others => '0'); -- no logic implemented
            control.done   <= '0'; -- this will cause an illegal instruction exception after timeout
        end case;

      when r4type_c => -- R4-type instructions; function select via "funct3"
      -- ----------------------------------------------------------------------
        control.result <= (others => '0'); -- no logic implemented
        control.done   <= '0'; -- this will cause an illegal instruction exception after timeout

      when r5typeA_c => -- R5-type instruction A
      -- ----------------------------------------------------------------------
        -- No function/immediate bit-fields are available for this instruction type.
        -- Hence, there is just one operation that can be implemented.
        control.result <= (others => '0'); -- no logic implemented
        control.done   <= '0'; -- this will cause an illegal instruction exception after timeout

      when r5typeB_c => -- R5-type instruction B
      -- ----------------------------------------------------------------------
        -- No function/immediate bit-fields are available for this instruction type.
        -- Hence, there is just one operation that can be implemented.
        control.result <= (others => '0'); -- no logic implemented
        control.done   <= '0'; -- this will cause an illegal instruction exception after timeout

      when others => -- undefined
      -- ----------------------------------------------------------------------
        control.result <= (others => '0'); -- no logic implemented
        control.done   <= '0'; -- this will cause an illegal instruction exception after timeout

    end case;
  end process result_select;


end neorv32_cpu_cp_cfu_rtl;
