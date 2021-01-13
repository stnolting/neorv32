-- #################################################################################################
-- # << NEORV32 - CPU Co-Processor: Bit manipulation unit (RISC-V "B" Extension) >>                #
-- # ********************************************************************************************* #
-- # The bit manipulation unit is implemted as co-processor that has a processing latency of at    #
-- # least 3 cycles. Only the "base" bit manipulation subset ('Zbb') is supported yet.             #
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

entity neorv32_cpu_cp_bitmanip is
  port (
    -- global control --
    clk_i   : in  std_ulogic; -- global clock, rising edge
    rstn_i  : in  std_ulogic; -- global reset, low-active, async
    ctrl_i  : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
    start_i : in  std_ulogic; -- trigger operation
    -- data input --
    cmp_i   : in  std_ulogic_vector(1 downto 0); -- comparator status
    rs1_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 1
    rs2_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 2
    -- result and status --
    res_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- operation result
    valid_o : out std_ulogic -- data output valid
  );
end neorv32_cpu_cp_bitmanip;

architecture neorv32_cpu_cp_bitmanip_rtl of neorv32_cpu_cp_bitmanip is

  -- commands --
  constant op_clz_c   : natural := 0;
  constant op_ctz_c   : natural := 1;
  constant op_cpop_c  : natural := 2;
  constant op_min_c   : natural := 3;
  constant op_max_c   : natural := 4;
  constant op_sextb_c : natural := 5;
  constant op_sexth_c : natural := 6;
  constant op_andn_c  : natural := 7;
  constant op_orn_c   : natural := 8;
  constant op_xnor_c  : natural := 9;
  constant op_pack_c  : natural := 10;
  constant op_ror_c   : natural := 11;
  constant op_rol_c   : natural := 12;
  constant op_rev8_c  : natural := 13;
  constant op_orcb_c  : natural := 14;
  --
  constant op_width_c : natural := 15;

  -- controller --
  type ctrl_state_t is (S_IDLE, S_START_SHIFT, S_BUSY_SHIFT, S_BUSY_LOGIC);
  signal ctrl_state   : ctrl_state_t;
  signal cmd, cmd_buf : std_ulogic_vector(op_width_c-1 downto 0);

  -- operand buffers --
  signal rs1_reg : std_ulogic_vector(data_width_c-1 downto 0);
  signal rs2_reg : std_ulogic_vector(data_width_c-1 downto 0);
  signal less_ff : std_ulogic;

  -- shift amount (immediate or register) --
  signal shamt : std_ulogic_vector(index_size_f(data_width_c)-1 downto 0);

  -- shifter --
  type shifter_t is record
    start   : std_ulogic;
    run     : std_ulogic;
    bcnt    : std_ulogic_vector(index_size_f(data_width_c) downto 0); -- bit counter
    cnt     : std_ulogic_vector(index_size_f(data_width_c) downto 0); -- iteration counter
    cnt_max : std_ulogic_vector(index_size_f(data_width_c) downto 0);
    sreg    : std_ulogic_vector(data_width_c-1 downto 0);
  end record;
  signal shifter : shifter_t;

  -- operation results --
  type res_t is array (0 to op_width_c-1) of std_ulogic_vector(data_width_c-1 downto 0);
  signal res_int, res_out : res_t;

begin

  -- Instruction Decoding (One-Hot) ---------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- a minimal decoding logic is used here -> just to distinguish between B.zbb instructions
  -- a more specific decoding and instruction check is done by the CPU control unit

  -- Zbb - Base Instructions --
  cmd(op_clz_c)   <= '1' when (ctrl_i(ctrl_ir_opcode7_5_c) = '0') and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "11") and (ctrl_i(ctrl_ir_funct3_2_c downto ctrl_ir_funct3_0_c) = "001") and (ctrl_i(ctrl_ir_funct12_2_c downto ctrl_ir_funct12_0_c) = "000") else '0';
  cmd(op_ctz_c)   <= '1' when (ctrl_i(ctrl_ir_opcode7_5_c) = '0') and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "11") and (ctrl_i(ctrl_ir_funct3_2_c downto ctrl_ir_funct3_0_c) = "001") and (ctrl_i(ctrl_ir_funct12_2_c downto ctrl_ir_funct12_0_c) = "001") else '0';
  cmd(op_cpop_c)  <= '1' when (ctrl_i(ctrl_ir_opcode7_5_c) = '0') and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "11") and (ctrl_i(ctrl_ir_funct3_2_c downto ctrl_ir_funct3_0_c) = "001") and (ctrl_i(ctrl_ir_funct12_2_c downto ctrl_ir_funct12_0_c) = "010") else '0';
  cmd(op_sextb_c) <= '1' when (ctrl_i(ctrl_ir_opcode7_5_c) = '0') and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "11") and (ctrl_i(ctrl_ir_funct3_2_c downto ctrl_ir_funct3_0_c) = "001") and (ctrl_i(ctrl_ir_funct12_2_c downto ctrl_ir_funct12_0_c) = "100") else '0';
  cmd(op_sexth_c) <= '1' when (ctrl_i(ctrl_ir_opcode7_5_c) = '0') and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "11") and (ctrl_i(ctrl_ir_funct3_2_c downto ctrl_ir_funct3_0_c) = "001") and (ctrl_i(ctrl_ir_funct12_2_c downto ctrl_ir_funct12_0_c) = "101") else '0';
  --
  cmd(op_ror_c)   <= '1' when                                         (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_7_c) = "1100") and (ctrl_i(ctrl_ir_funct3_2_c downto ctrl_ir_funct3_0_c) = "101") else '0';
  cmd(op_rol_c)   <= '1' when (ctrl_i(ctrl_ir_opcode7_5_c) = '1') and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_7_c) = "1100") and (ctrl_i(ctrl_ir_funct3_2_c downto ctrl_ir_funct3_0_c) = "001") else '0';
  --
  cmd(op_rev8_c)  <= '1' when (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_7_c) = "1101") else '0';
  --
  cmd(op_orcb_c)  <= '1' when (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "01") else '0';
  --
  cmd(op_min_c)   <= '1' when (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "00") and (ctrl_i(ctrl_ir_funct12_7_c downto ctrl_ir_funct12_5_c) = "101") and (ctrl_i(ctrl_ir_funct3_1_c) = '0') else '0';
  cmd(op_max_c)   <= '1' when (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "00") and (ctrl_i(ctrl_ir_funct12_7_c downto ctrl_ir_funct12_5_c) = "101") and (ctrl_i(ctrl_ir_funct3_1_c) = '1') else '0';
  --
  cmd(op_andn_c)  <= '1' when (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "10") and (ctrl_i(ctrl_ir_funct3_1_c downto ctrl_ir_funct3_0_c) = "11") else '0';
  cmd(op_orn_c)   <= '1' when (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "10") and (ctrl_i(ctrl_ir_funct3_1_c downto ctrl_ir_funct3_0_c) = "10") else '0';
  cmd(op_xnor_c)  <= '1' when (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "10") and (ctrl_i(ctrl_ir_funct3_1_c downto ctrl_ir_funct3_0_c) = "00") else '0';
  --
  cmd(op_pack_c)  <= '1' when (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_5_c) = "000100") else '0';


  -- Co-Processor Controller ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  coprocessor_ctrl: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl_state    <= S_IDLE;
      cmd_buf       <= (others => '0');
      rs1_reg       <= (others => '0');
      rs2_reg       <= (others => '0');
      less_ff       <= '0';
      shifter.start <= '0';
      valid_o       <= '0';
    elsif rising_edge(clk_i) then
      -- defaults --
      shifter.start <= '0';
      valid_o       <= '0';

      -- fsm --
      case ctrl_state is

        when S_IDLE => -- wait for operation trigger
        -- ------------------------------------------------------------
          less_ff <= cmp_i(alu_cmp_less_c);
          if (start_i = '1') then
            cmd_buf <= cmd;
            rs1_reg <= rs1_i;
            rs2_reg <= rs2_i;
            if ((cmd(op_clz_c) or cmd(op_ctz_c) or cmd(op_cpop_c) or cmd(op_ror_c) or cmd(op_rol_c)) = '1') then -- multi-cycle shift operation
              shifter.start <= '1';
              ctrl_state <= S_START_SHIFT;
            else
              ctrl_state <= S_BUSY_LOGIC;
            end if;
          end if;

        when S_START_SHIFT => -- one cycle delay to start shift operation
        -- ------------------------------------------------------------
          ctrl_state <= S_BUSY_SHIFT;

        when S_BUSY_SHIFT => -- wait for multi-cycle shift operation to finish
        -- ------------------------------------------------------------
          if (shifter.run = '0') then
            ctrl_state <= S_BUSY_LOGIC;
          end if;

        when S_BUSY_LOGIC => -- single-cycle logic operation (and output)
        -- ------------------------------------------------------------
          valid_o    <= '1';
          ctrl_state <= S_IDLE;

        when others => -- undefined
        -- ------------------------------------------------------------
          ctrl_state <= S_IDLE;

      end case;
    end if;
  end process coprocessor_ctrl;


  -- Shift Amount ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- we could also use ALU's internal operand B - but we are having a local version here in order to allow
  -- better logic combination inside the ALU (since that is the critical path of the CPU)
  shamt <= ctrl_i(shamt'left+ctrl_ir_funct12_0_c downto ctrl_ir_funct12_0_c) when (ctrl_i(ctrl_ir_opcode7_5_c) = '0') else rs2_reg(shamt'left downto 0);


  -- Shifter Function Core ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  shifter_unit: process(clk_i)
    variable new_bit_v : std_ulogic;
  begin
    if rising_edge(clk_i) then
      if (shifter.start = '1') then -- trigger new shift
        shifter.cnt <= (others => '0');
        -- shift operand --
        if (cmd_buf(op_clz_c) = '1') or (cmd_buf(op_rol_c) = '1') then -- count LEADING zeros / rotate LEFT
          shifter.sreg <= bit_rev_f(rs1_reg); -- reverse - we can only do right shifts here
        else -- ctz, cpop, ror
          shifter.sreg <= rs1_reg;
        end if;
        -- max shift amount --
        if (cmd_buf(op_cpop_c) = '1') then -- population count
          shifter.cnt_max <= (others => '0');
          shifter.cnt_max(shifter.cnt_max'left) <= '1';
        else
          shifter.cnt_max <= '0' & shamt;
        end if;
        shifter.bcnt <= (others => '0');
      elsif (shifter.run = '1') then -- right shifts only
        new_bit_v := ((cmd_buf(op_ror_c) or cmd_buf(op_rol_c)) and shifter.sreg(0)) or (cmd_buf(op_clz_c) or cmd_buf(op_ctz_c));
        shifter.sreg <= new_bit_v & shifter.sreg(shifter.sreg'left downto 1); -- ro[r/l]/lsr(for counting)
        shifter.cnt  <= std_ulogic_vector(unsigned(shifter.cnt) + 1); -- iteration counter
        if (shifter.sreg(0) = '1') then
          shifter.bcnt <= std_ulogic_vector(unsigned(shifter.bcnt) + 1); -- bit counter
        end if;
      end if;
    end if;
  end process shifter_unit;

  shifter_unit_ctrl: process(cmd_buf, shifter)
  begin
    -- keep shifting until ... --
    if (cmd_buf(op_clz_c) = '1') or (cmd_buf(op_ctz_c) = '1') then -- count leading/trailing zeros
      shifter.run <= not shifter.sreg(0);
    else -- population count / rotate
      if (shifter.cnt = shifter.cnt_max) then
        shifter.run <= '0';
      else
        shifter.run <= '1';
      end if;
    end if;
  end process shifter_unit_ctrl;


  -- Base ('Zbb') Logic Function Core -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- count leading/trailing zeros --
  res_int(op_clz_c)(data_width_c-1 downto shifter.cnt'left+1) <= (others => '0');
  res_int(op_clz_c)(shifter.cnt'left downto 0) <= shifter.cnt;
  res_int(op_ctz_c)(data_width_c-1 downto shifter.cnt'left+1) <= (others => '0');
  res_int(op_ctz_c)(shifter.cnt'left downto 0) <= shifter.cnt;

  -- count set bits --
  res_int(op_cpop_c)(data_width_c-1 downto shifter.bcnt'left+1) <= (others => '0');
  res_int(op_cpop_c)(shifter.bcnt'left downto 0) <= shifter.bcnt;

  -- min/max select --
  res_int(op_min_c) <= rs1_reg when (less_ff = '1') else rs2_reg;
  res_int(op_max_c) <= rs2_reg when (less_ff = '1') else rs1_reg;

  -- sign-extension --
  res_int(op_sextb_c)(data_width_c-1 downto 8) <= (others => rs1_reg(7));
  res_int(op_sextb_c)(7 downto 0) <= rs1_reg(7 downto 0); -- sign-extend byte
  res_int(op_sexth_c)(data_width_c-1 downto 16) <= (others => rs1_reg(15));
  res_int(op_sexth_c)(15 downto 0) <= rs1_reg(15 downto 0); -- sign-extend half-word

  -- logic with negate --
  res_int(op_andn_c) <= rs1_reg and (not rs2_reg); -- logical and-not
  res_int(op_orn_c)  <= rs1_reg or  (not rs2_reg); -- logical or-not
  res_int(op_xnor_c) <= rs1_reg xor (not rs2_reg); -- logical xor-not

  -- pack two words in one register --
  res_int(op_pack_c) <= rs2_reg((data_width_c/2)-1 downto 0) & rs1_reg((data_width_c/2)-1 downto 0); -- pack lower halves

  -- rotate right/left --
  res_int(op_ror_c) <= shifter.sreg;
  res_int(op_rol_c) <= bit_rev_f(shifter.sreg);

  -- reversal.8 (byte swap) --
  res_int(op_rev8_c) <= bswap32_f(rs1_reg);

  -- or-combine.byte --
  or_combine_byte_gen:
  for i in 0 to (data_width_c/8)-1 generate
    res_int(op_orcb_c)(i*8+7 downto i*8) <= (others => or_all_f(rs1_reg(i*8+7 downto i*8)));
  end generate; -- i


  -- Output Selector ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  res_out(op_clz_c)   <= res_int(op_clz_c)   when (cmd_buf(op_clz_c)   = '1') else (others => '0');
  res_out(op_ctz_c)   <= res_int(op_ctz_c)   when (cmd_buf(op_ctz_c)   = '1') else (others => '0');
  res_out(op_cpop_c)  <= res_int(op_cpop_c)  when (cmd_buf(op_cpop_c)  = '1') else (others => '0');
  res_out(op_min_c)   <= res_int(op_min_c)   when (cmd_buf(op_min_c)   = '1') else (others => '0');
  res_out(op_max_c)   <= res_int(op_max_c)   when (cmd_buf(op_max_c)   = '1') else (others => '0');
  res_out(op_sextb_c) <= res_int(op_sextb_c) when (cmd_buf(op_sextb_c) = '1') else (others => '0');
  res_out(op_sexth_c) <= res_int(op_sexth_c) when (cmd_buf(op_sexth_c) = '1') else (others => '0');
  res_out(op_andn_c)  <= res_int(op_andn_c)  when (cmd_buf(op_andn_c)  = '1') else (others => '0');
  res_out(op_orn_c)   <= res_int(op_orn_c)   when (cmd_buf(op_orn_c)   = '1') else (others => '0');
  res_out(op_xnor_c)  <= res_int(op_xnor_c)  when (cmd_buf(op_xnor_c)  = '1') else (others => '0');
  res_out(op_pack_c)  <= res_int(op_pack_c)  when (cmd_buf(op_pack_c)  = '1') else (others => '0');
  res_out(op_ror_c)   <= res_int(op_ror_c)   when (cmd_buf(op_ror_c)   = '1') else (others => '0');
  res_out(op_rol_c)   <= res_int(op_rol_c)   when (cmd_buf(op_rol_c)   = '1') else (others => '0');
  res_out(op_rev8_c)  <= res_int(op_rev8_c)  when (cmd_buf(op_rev8_c)  = '1') else (others => '0');
  res_out(op_orcb_c)  <= res_int(op_orcb_c)  when (cmd_buf(op_orcb_c)  = '1') else (others => '0');


  -- Output Gate ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  output_gate: process(clk_i)
  begin
    if rising_edge(clk_i) then
      res_o <= (others => '0');
      if (ctrl_state = S_BUSY_LOGIC) then
        res_o <= res_out(op_clz_c)   or res_out(op_ctz_c)   or res_out(op_cpop_c) or
                 res_out(op_min_c)   or res_out(op_max_c)   or
                 res_out(op_sextb_c) or res_out(op_sexth_c) or
                 res_out(op_andn_c)  or res_out(op_orn_c)   or res_out(op_xnor_c) or
                 res_out(op_pack_c)  or
                 res_out(op_ror_c)   or res_out(op_rol_c)   or
                 res_out(op_rev8_c)  or
                 res_out(op_orcb_c);
      end if;
    end if;
  end process output_gate;


end neorv32_cpu_cp_bitmanip_rtl;
