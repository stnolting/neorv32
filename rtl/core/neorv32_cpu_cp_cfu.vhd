-- #################################################################################################
-- # << NEORV32 - CPU Co-Processor: Custom (Instructions) Functions Unit >>                        #
-- # ********************************************************************************************* #
-- # For user-defined custom RISC-V instructions (R3-type, R4-type and R5-type formats).           #
-- # See the CPU's documentation for more information.                                             #
-- #                                                                                               #
-- # NOTE: Take a look at the "software-counterpart" of this CFU example in 'sw/example/demo_cfu'. #
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

entity neorv32_cpu_cp_cfu is
  port (
    -- global control --
    clk_i   : in  std_ulogic; -- global clock, rising edge
    rstn_i  : in  std_ulogic; -- global reset, low-active, async
    ctrl_i  : in  ctrl_bus_t; -- main control bus
    start_i : in  std_ulogic; -- trigger operation
    -- data input --
    rs1_i   : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 1
    rs2_i   : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 2
    rs3_i   : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 3
    rs4_i   : in  std_ulogic_vector(XLEN-1 downto 0); -- rf source 4
    -- result and status --
    res_o   : out std_ulogic_vector(XLEN-1 downto 0); -- operation result
    valid_o : out std_ulogic -- data output valid
  );
end neorv32_cpu_cp_cfu;

architecture neorv32_cpu_cp_cfu_rtl of neorv32_cpu_cp_cfu is

-- CFU controll - do not modify! ---------------------------
-- ------------------------------------------------------------

  type control_t is record
    busy   : std_ulogic; -- CFU is busy
    done   : std_ulogic; -- set to '1' when processing is done
    result : std_ulogic_vector(XLEN-1 downto 0); -- user's processing result (for write-back to register file)
    rtype  : std_ulogic_vector(1 downto 0); -- instruction type, see constants below
    funct3 : std_ulogic_vector(2 downto 0); -- "funct3" bit-field from custom instruction
    funct7 : std_ulogic_vector(6 downto 0); -- "funct7" bit-field from custom instruction
  end record;
  signal control : control_t;

  -- instruction format types --
  constant r3type_c  : std_ulogic_vector(1 downto 0) := "00"; -- R3-type instructions (custom-0 opcode)
  constant r4type_c  : std_ulogic_vector(1 downto 0) := "01"; -- R4-type instructions (custom-1 opcode)
  constant r5typeA_c : std_ulogic_vector(1 downto 0) := "10"; -- R5-type instruction A (custom-2 opcode)
  constant r5typeB_c : std_ulogic_vector(1 downto 0) := "11"; -- R5-type instruction B (custom-3 opcode)

-- User Logic ----------------------------------------------
-- ------------------------------------------------------------

  -- multiply-add unit (r4-type instruction example) --
  type madd_t is record
    sreg : std_ulogic_vector(2 downto 0); -- 3 cycles latency = 3 bits in arbitration shift register
    done : std_ulogic;
    --
    opa  : std_ulogic_vector(XLEN-1 downto 0);
    opb  : std_ulogic_vector(XLEN-1 downto 0);
    opc  : std_ulogic_vector(XLEN-1 downto 0);
    mul  : std_ulogic_vector(2*XLEN-1 downto 0);
    res  : std_ulogic_vector(2*XLEN-1 downto 0);
  end record;
  signal madd : madd_t;

begin

-- ****************************************************************************************************************************
-- This controller is required to handle the CPU/pipeline interface. Do not modify!
-- ****************************************************************************************************************************

  -- CFU Controller -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cfu_control: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      res_o        <= (others => '0');
      control.busy <= '0';
    elsif rising_edge(clk_i) then
      res_o <= (others => '0'); -- default; all CPU co-processor outputs are logically OR-ed
      if (control.busy = '0') then -- idle
        if (start_i = '1') then
          control.busy <= '1';
        end if;
      else -- busy
        if (control.done = '1') or (ctrl_i.cpu_trap = '1') then -- processing done? abort if trap (exception)
          res_o        <= control.result; -- output result for just one cycle, CFU output has to be all-zero otherwise
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


-- ****************************************************************************************************************************
-- CFU Hardware Documentation and Implementation Notes
-- ****************************************************************************************************************************

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
  -- bit-field ('funct7').
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
  --
  -- [NOTE] The CFU cannot cause any kind of exception at all (yet; this feature is planned for the future).


  -- ----------------------------------------------------------------------------------------
  -- Result Output
  -- ----------------------------------------------------------------------------------------
  -- > control.result (output, 32-bit): processing result ("data")
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
  --
  -- [NOTE] If the <control.done> signal is not set within a bound time window (default = 128 cycles) the CFU operation is
  --        automatically terminated by the hardware and an illegal instruction exception is raised. This feature can also be
  --        be used to implement custom CFU exceptions.


  -- ----------------------------------------------------------------------------------------
  -- Final Notes
  -- ----------------------------------------------------------------------------------------
  -- The <control> record provides something like a "keeper" that ensures correct functionality and that also provides a
  -- simple-to-use interface hardware designers can start with. However, the control instance adds one additional cycle of
  -- latency. Advanced users can remove this default control instance to obtain maximum throughput.


-- ****************************************************************************************************************************
-- Actual CFU User Logic Example - replace this with your custom logic
-- ****************************************************************************************************************************

  -- Iterative Multiply-Add Unit - Iteration Control ----------------------------------------
  -- -------------------------------------------------------------------------------------------
  madd_control: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      madd.sreg <= (others => '0');
    elsif rising_edge(clk_i) then
      -- operation trigger --
      if (control.busy = '0') and -- CFU is idle (ready for next operation)
         (start_i = '1') and -- CFU is actually triggered by a custom instruction word
         (control.rtype = r4type_c) and -- this is a R4-type instruction
         (control.funct3(2 downto 1) = "00") then -- trigger only for specific funct3 values
        madd.sreg(0) <= '1';
      else
        madd.sreg(0) <= '0';
      end if;
      -- simple shift register for tracking operation --
      madd.sreg(madd.sreg'left downto 1) <= madd.sreg(madd.sreg'left-1 downto 0); -- shift left
    end if;
  end process madd_control;

  -- processing has reached last stage (=done) when sreg's MSB is set --
  madd.done <= madd.sreg(madd.sreg'left);


  -- Iterative Multiply-Add Unit - Arithmetic Core ------------------------------------------
  -- -------------------------------------------------------------------------------------------
  madd_core: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- stage 0: buffer input operands --
      madd.opa <= rs1_i;
      madd.opb <= rs2_i;
      madd.opc <= rs3_i;
      -- stage 1: multiply rs1 and rs2 --
      madd.mul <= std_ulogic_vector(unsigned(madd.opa) * unsigned(madd.opb));
      -- stage 2: add rs3 to multiplication result --
      madd.res <= std_ulogic_vector(unsigned(madd.mul) + unsigned(madd.opc));
    end if;
  end process madd_core;


  -- Output select --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  out_select: process(control, rs1_i, rs2_i, rs3_i, rs4_i, madd)
  begin
    case control.rtype is

      -- --------------------------------------------------------
      when r3type_c => -- R3-type instructions
      -- --------------------------------------------------------

        -- This is a simple ALU that implements four pure-combinatorial instructions.
        -- The actual function is selected by the "funct3" bit-field of the custom instruction.
        case control.funct3 is
          when "000" => -- funct3 = "000": bit-reversal of rs1
            control.result <= bit_rev_f(rs1_i);
            control.done   <= '1'; -- pure-combinatorial, so we are done "immediately"
          when "001" => -- funct3 = "001": XNOR input operands
            control.result <= rs1_i xnor rs2_i;
            control.done   <= '1'; -- pure-combinatorial, so we are done "immediately"
          when others => -- not implemented
            control.result <= (others => '0');
            control.done   <= '0'; -- this will cause an illegal instruction exception after timeout
        end case;

      -- --------------------------------------------------------
      when r4type_c => -- R4-type instructions
      -- --------------------------------------------------------

        -- This is an iterative multiply-and-add unit that requires several cycles for processing.
        -- The actual function is selected by the lowest bit of the "funct3" bit-field.
        case control.funct3 is
          when "000" => -- funct3 = "000": multiply-add low-part result: rs1*rs2+r3 [31:0]
            control.result <= madd.res(31 downto 0);
            control.done   <= madd.done; -- iterative, wait for unit to finish
          when "001" => -- funct3 = "001": multiply-add high-part result: rs1*rs2+r3 [63:32]
            control.result <= madd.res(63 downto 32);
            control.done   <= madd.done; -- iterative, wait for unit to finish
          when others => -- not implemented
            control.result <= (others => '0');
            control.done   <= '0'; -- this will cause an illegal instruction exception after timeout
        end case;

      -- --------------------------------------------------------
      when r5typeA_c => -- R5-type instruction A
      -- --------------------------------------------------------

        -- No function/immediate bit-fields are available for this instruction type.
        -- Hence, there is just one operation that can be implemented.
        control.result <= rs1_i and rs2_i and rs3_i and rs4_i; -- AND-all
        control.done   <= '1'; -- pure-combinatorial, so we are done "immediately"

      -- --------------------------------------------------------
      when r5typeB_c => -- R5-type instruction B
      -- --------------------------------------------------------

        -- No function/immediate bit-fields are available for this instruction type.
        -- Hence, there is just one operation that can be implemented.
        control.result <= rs1_i xor rs2_i xor rs3_i xor rs4_i; -- XOR-all
        control.done   <= '1'; -- set high to prevent permanent CPU stall

      -- --------------------------------------------------------
      when others => -- undefined
      -- --------------------------------------------------------

        control.result <= (others => '0');
        control.done   <= '0';

    end case;
  end process out_select;


end neorv32_cpu_cp_cfu_rtl;
