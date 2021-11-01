-- #################################################################################################
-- # << NEORV32 - CPU Co-Processor: Bit-Manipulation Co-Processor Unit (RISC-V "B" Extension) >>   #
-- # ********************************************************************************************* #
-- # The bit manipulation unit is implemented as co-processor that has a processing latency of 1   #
-- # cycle for logic/arithmetic operations and 3+shamt (=shift amount) cycles for shift(-related)  #
-- # operations. Use the FAST_SHIFT_EN option to reduce shift-related instruction's latency to a   #
-- # fixed value of 3 cycles latency (using barrel shifters).                                      #
-- #                                                                                               #
-- # Supported sub-extensions (Zb*):                                                               #
-- # - Zba: Address generation instructions                                                        #
-- # - Zbb: Basic bit-manipulation instructions                                                    #
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
  generic (
    FAST_SHIFT_EN : boolean -- use barrel shifter for shift operations
  );
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
    shamt_i : in  std_ulogic_vector(index_size_f(data_width_c)-1 downto 0); -- shift amount
    -- result and status --
    res_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- operation result
    valid_o : out std_ulogic -- data output valid
  );
end neorv32_cpu_cp_bitmanip;

architecture neorv32_cpu_cp_bitmanip_rtl of neorv32_cpu_cp_bitmanip is

  -- Sub-extension configuration --
  constant zbb_en_c : boolean := true;
  constant zba_en_c : boolean := true;
  -- --------------------------- --

  -- commands: Zbb - logic with negate --
  constant op_andn_c    : natural := 0;
  constant op_orn_c     : natural := 1;
  constant op_xnor_c    : natural := 2;
  -- commands: Zbb - count leading/trailing zero bits --
  constant op_clz_c     : natural := 3;
  constant op_ctz_c     : natural := 4;
  -- commands: Zbb - count population --
  constant op_cpop_c    : natural := 5;
  -- commands: Zbb - integer minimum/maximum --
  constant op_max_c     : natural := 6; -- signed/unsigned
  constant op_min_c     : natural := 7; -- signed/unsigned
  -- commands: Zbb - sign- and zero-extension --
  constant op_sextb_c   : natural := 8;
  constant op_sexth_c   : natural := 9;
  constant op_zexth_c   : natural := 10;
  -- commands: Zbb - bitwise rotation --
  constant op_rol_c     : natural := 11;
  constant op_ror_c     : natural := 12; -- rori
  -- commands: Zbb - or-combine --
  constant op_orcb_c    : natural := 13;
  -- commands: Zbb - byte-reverse --
  constant op_rev8_c    : natural := 14;
  -- commands: Zba - shifted add --
  constant op_sh1add_c  : natural := 15;
  constant op_sh2add_c  : natural := 16;
  constant op_sh3add_c  : natural := 17;
  --
  constant op_width_c   : natural := 18;

  -- controller --
  type ctrl_state_t is (S_IDLE, S_START_SHIFT, S_BUSY_SHIFT);
  signal ctrl_state   : ctrl_state_t;
  signal cmd, cmd_buf : std_ulogic_vector(op_width_c-1 downto 0);
  signal valid        : std_ulogic;

  -- operand buffers --
  signal rs1_reg : std_ulogic_vector(data_width_c-1 downto 0);
  signal rs2_reg : std_ulogic_vector(data_width_c-1 downto 0);
  signal sha_reg : std_ulogic_vector(index_size_f(data_width_c)-1 downto 0);
  signal less_ff : std_ulogic;

  -- serial shifter --
  type shifter_t is record
    start   : std_ulogic;
    run     : std_ulogic;
    bcnt    : std_ulogic_vector(index_size_f(data_width_c) downto 0); -- bit counter
    cnt     : std_ulogic_vector(index_size_f(data_width_c) downto 0); -- iteration counter
    cnt_max : std_ulogic_vector(index_size_f(data_width_c) downto 0);
    sreg    : std_ulogic_vector(data_width_c-1 downto 0);
  end record;
  signal shifter : shifter_t;

  -- barrel shifter --
  type bs_level_t is array (index_size_f(data_width_c) downto 0) of std_ulogic_vector(data_width_c-1 downto 0);
  signal bs_level : bs_level_t;

  -- operation results --
  type res_t is array (0 to op_width_c-1) of std_ulogic_vector(data_width_c-1 downto 0);
  signal res_int, res_out : res_t;

  -- shifted-add unit --
  signal adder_core : std_ulogic_vector(data_width_c-1 downto 0);

begin

  -- Sub-Extension Configuration ------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert false report
  "Implementing bit-manipulation (B) sub-extensions: " &
  cond_sel_string_f(zbb_en_c, "Zbb", "") &
  cond_sel_string_f(zba_en_c, "Zba", "") &
  ""
  severity note;


  -- Instruction Decoding (One-Hot) ---------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- a minimal decoding logic is used here -> just to distinguish between B.Zbb instructions
  -- a more specific decoding and instruction check is done by the CPU control unit

  -- Zbb - Basic bit-manipulation instructions --
  cmd(op_andn_c)   <= '1' when (zbb_en_c = true) and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "10") and (ctrl_i(ctrl_ir_funct3_1_c downto ctrl_ir_funct3_0_c) = "11") else '0';
  cmd(op_orn_c)    <= '1' when (zbb_en_c = true) and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "10") and (ctrl_i(ctrl_ir_funct3_1_c downto ctrl_ir_funct3_0_c) = "10") else '0';
  cmd(op_xnor_c)   <= '1' when (zbb_en_c = true) and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "10") and (ctrl_i(ctrl_ir_funct3_1_c downto ctrl_ir_funct3_0_c) = "00") else '0';
  --
  cmd(op_max_c)    <= '1' when (zbb_en_c = true) and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "00") and (ctrl_i(ctrl_ir_funct12_5_c) = '1') and (ctrl_i(ctrl_ir_funct3_1_c) = '1') else '0';
  cmd(op_min_c)    <= '1' when (zbb_en_c = true) and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "00") and (ctrl_i(ctrl_ir_funct12_5_c) = '1') and (ctrl_i(ctrl_ir_funct3_1_c) = '0') else '0';
  cmd(op_zexth_c)  <= '1' when (zbb_en_c = true) and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "00") and (ctrl_i(ctrl_ir_funct12_5_c) = '0') else '0';
  --
  cmd(op_orcb_c)   <= '1' when (zbb_en_c = true) and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "01") and (ctrl_i(ctrl_ir_funct12_7_c) = '1') else '0';
  --
  cmd(op_clz_c)    <= '1' when (zbb_en_c = true) and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "11") and (ctrl_i(ctrl_ir_funct12_7_c) = '0') and (ctrl_i(ctrl_ir_funct12_2_c downto ctrl_ir_funct12_0_c) = "000") else '0';
  cmd(op_ctz_c)    <= '1' when (zbb_en_c = true) and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "11") and (ctrl_i(ctrl_ir_funct12_7_c) = '0') and (ctrl_i(ctrl_ir_funct12_2_c downto ctrl_ir_funct12_0_c) = "001") else '0';
  cmd(op_cpop_c)   <= '1' when (zbb_en_c = true) and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "11") and (ctrl_i(ctrl_ir_funct12_7_c) = '0') and (ctrl_i(ctrl_ir_funct12_2_c downto ctrl_ir_funct12_0_c) = "010") else '0';
  cmd(op_sextb_c)  <= '1' when (zbb_en_c = true) and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "11") and (ctrl_i(ctrl_ir_funct12_7_c) = '0') and (ctrl_i(ctrl_ir_funct3_2_c) = '0') and (ctrl_i(ctrl_ir_funct12_2_c downto ctrl_ir_funct12_0_c) = "100") else '0';
  cmd(op_sexth_c)  <= '1' when (zbb_en_c = true) and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "11") and (ctrl_i(ctrl_ir_funct12_7_c) = '0') and (ctrl_i(ctrl_ir_funct3_2_c) = '0') and (ctrl_i(ctrl_ir_funct12_2_c downto ctrl_ir_funct12_0_c) = "101") else '0';
  cmd(op_rol_c)    <= '1' when (zbb_en_c = true) and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "11") and (ctrl_i(ctrl_ir_funct12_7_c) = '0') and (ctrl_i(ctrl_ir_funct3_2_c downto ctrl_ir_funct3_0_c) = "001") and (ctrl_i(ctrl_ir_opcode7_5_c) = '1') else '0';
  cmd(op_ror_c)    <= '1' when (zbb_en_c = true) and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "11") and (ctrl_i(ctrl_ir_funct12_7_c) = '0') and (ctrl_i(ctrl_ir_funct3_2_c downto ctrl_ir_funct3_0_c) = "101") else '0';
  cmd(op_rev8_c)   <= '1' when (zbb_en_c = true) and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "11") and (ctrl_i(ctrl_ir_funct12_7_c) = '1') else '0';

  -- Zba - Address generation instructions --
  cmd(op_sh1add_c) <= '1' when (zba_en_c = true) and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "01") and (ctrl_i(ctrl_ir_funct12_7_c) = '0') and (ctrl_i(ctrl_ir_funct3_2_c downto ctrl_ir_funct3_1_c) = "01") else '0';
  cmd(op_sh2add_c) <= '1' when (zba_en_c = true) and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "01") and (ctrl_i(ctrl_ir_funct12_7_c) = '0') and (ctrl_i(ctrl_ir_funct3_2_c downto ctrl_ir_funct3_1_c) = "10") else '0';
  cmd(op_sh3add_c) <= '1' when (zba_en_c = true) and (ctrl_i(ctrl_ir_funct12_10_c downto ctrl_ir_funct12_9_c) = "01") and (ctrl_i(ctrl_ir_funct12_7_c) = '0') and (ctrl_i(ctrl_ir_funct3_2_c downto ctrl_ir_funct3_1_c) = "11") else '0';


  -- Co-Processor Controller ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  coprocessor_ctrl: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl_state    <= S_IDLE;
      cmd_buf       <= (others => def_rst_val_c);
      rs1_reg       <= (others => def_rst_val_c);
      rs2_reg       <= (others => def_rst_val_c);
      sha_reg       <= (others => def_rst_val_c);
      less_ff       <= def_rst_val_c;
      shifter.start <= '0';
      valid         <= '0';
    elsif rising_edge(clk_i) then
      -- defaults --
      shifter.start <= '0';
      valid         <= '0';

      -- fsm --
      case ctrl_state is

        when S_IDLE => -- wait for operation trigger
        -- ------------------------------------------------------------
          if (start_i = '1') then
            less_ff <= cmp_i(cmp_less_c);
            cmd_buf <= cmd;
            rs1_reg <= rs1_i;
            rs2_reg <= rs2_i;
            sha_reg <= shamt_i;
            if ((cmd(op_clz_c) or cmd(op_ctz_c) or cmd(op_cpop_c) or cmd(op_ror_c) or cmd(op_rol_c)) = '1') then -- multi-cycle shift operation
              if (FAST_SHIFT_EN = false) then -- default: iterative computation
                shifter.start <= '1';
                ctrl_state <= S_START_SHIFT;
              else -- full-parallel computation
                ctrl_state <= S_BUSY_SHIFT;
              end if;
            else
              valid      <= '1';
              ctrl_state <= S_IDLE;
            end if;
          end if;

        when S_START_SHIFT => -- one cycle delay to start shift operation
        -- ------------------------------------------------------------
          ctrl_state <= S_BUSY_SHIFT;

        when S_BUSY_SHIFT => -- wait for multi-cycle shift operation to finish
        -- ------------------------------------------------------------
          if (shifter.run = '0') then
            valid      <= '1';
            ctrl_state <= S_IDLE;
          end if;

        when others => -- undefined
        -- ------------------------------------------------------------
          ctrl_state <= S_IDLE;

      end case;
    end if;
  end process coprocessor_ctrl;


  -- Shifter Function Core (iterative: small but slow) --------------------------------------
  -- -------------------------------------------------------------------------------------------
  serial_shifter:
  if (FAST_SHIFT_EN = false) generate
    shifter_unit: process(rstn_i, clk_i)
      variable new_bit_v : std_ulogic;
    begin
      if (rstn_i = '0') then
        shifter.cnt     <= (others => def_rst_val_c);
        shifter.sreg    <= (others => def_rst_val_c);
        shifter.cnt_max <= (others => def_rst_val_c);
        shifter.bcnt    <= (others => def_rst_val_c);
      elsif rising_edge(clk_i) then
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
            shifter.cnt_max <= '0' & sha_reg;
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
  end generate;

  -- run control --
  serial_shifter_ctrl:
  if (FAST_SHIFT_EN = false) generate
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
  end generate;


  -- Shifter Function Core (parallel: fast but large) ---------------------------------------
  -- -------------------------------------------------------------------------------------------
  barrel_shifter_async_sync:
  if (FAST_SHIFT_EN = true) generate
    shifter_unit_fast: process(rstn_i, clk_i)
      variable new_bit_v : std_ulogic;
    begin
      if (rstn_i = '0') then
        shifter.cnt     <= (others => def_rst_val_c);
        shifter.sreg    <= (others => def_rst_val_c);
        shifter.bcnt    <= (others => def_rst_val_c);
      elsif rising_edge(clk_i) then
        -- population count --
        shifter.bcnt <= std_ulogic_vector(to_unsigned(popcount_f(rs1_reg), shifter.bcnt'length));
        -- count leading/trailing zeros --
        if cmd_buf(op_clz_c) = '1' then -- leading
          shifter.cnt <= std_ulogic_vector(to_unsigned(leading_zeros_f(rs1_reg), shifter.cnt'length));
        else -- trailing
          shifter.cnt <= std_ulogic_vector(to_unsigned(leading_zeros_f(bit_rev_f(rs1_reg)), shifter.cnt'length));
        end if;
        -- barrel shifter --
        shifter.sreg <= bs_level(0); -- rol/ror[i]
      end if;
    end process shifter_unit_fast;
    shifter.run <= '0'; -- we are done already!
  end generate;

  -- barrel shifter array --
  barrel_shifter_async:
  if (FAST_SHIFT_EN = true) generate
    shifter_unit_async: process(rs1_reg, sha_reg, cmd_buf, bs_level)
    begin
      -- input level: convert left shifts to right shifts --
      if (cmd_buf(op_rol_c) = '1') then -- is left shift?
        bs_level(index_size_f(data_width_c)) <= bit_rev_f(rs1_reg); -- reverse bit order of input operand
      else
        bs_level(index_size_f(data_width_c)) <= rs1_reg;
      end if;

      -- shifter array --
      for i in index_size_f(data_width_c)-1 downto 0 loop
        if (sha_reg(i) = '1') then
          bs_level(i)(data_width_c-1 downto data_width_c-(2**i)) <= bs_level(i+1)((2**i)-1 downto 0);
          bs_level(i)((data_width_c-(2**i))-1 downto 0) <= bs_level(i+1)(data_width_c-1 downto 2**i);
        else
          bs_level(i) <= bs_level(i+1);
        end if;
      end loop;
    end process shifter_unit_async;
  end generate;


  -- Shifted-Add Core -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  shift_adder: process(rs1_reg, rs2_reg, ctrl_i)
    variable opb_v : std_ulogic_vector(data_width_c-1 downto 0);
  begin
    case ctrl_i(ctrl_ir_funct3_2_c downto ctrl_ir_funct3_1_c) is
      when "01"   => opb_v := rs1_reg(rs1_reg'left-1 downto 0) & '0';   -- << 1
      when "10"   => opb_v := rs1_reg(rs1_reg'left-2 downto 0) & "00";  -- << 2
      when "11"   => opb_v := rs1_reg(rs1_reg'left-3 downto 0) & "000"; -- << 3
      when others => opb_v := rs1_reg(rs1_reg'left-1 downto 0) & '0';   -- undefined
    end case;
    adder_core <= std_ulogic_vector(unsigned(rs2_reg) + unsigned(opb_v));
  end process shift_adder;


  -- Operation Results ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- logic with negate --
  res_int(op_andn_c) <= rs1_reg and (not rs2_reg); -- logical and-not
  res_int(op_orn_c)  <= rs1_reg or  (not rs2_reg); -- logical or-not
  res_int(op_xnor_c) <= rs1_reg xor (not rs2_reg); -- logical xor-not

  -- count leading/trailing zeros --
  res_int(op_clz_c)(data_width_c-1 downto shifter.cnt'left+1) <= (others => '0');
  res_int(op_clz_c)(shifter.cnt'left downto 0) <= shifter.cnt;
  res_int(op_ctz_c) <= (others => '0'); -- unused/redundant

  -- count set bits --
  res_int(op_cpop_c)(data_width_c-1 downto shifter.bcnt'left+1) <= (others => '0');
  res_int(op_cpop_c)(shifter.bcnt'left downto 0) <= shifter.bcnt;

  -- min/max select --
  res_int(op_min_c) <= rs1_reg when ((less_ff xor cmd_buf(op_max_c)) = '1') else rs2_reg;
  res_int(op_max_c) <= (others => '0'); -- unused/redundant

  -- sign-extension --
  res_int(op_sextb_c)(data_width_c-1 downto 8) <= (others => rs1_reg(7));
  res_int(op_sextb_c)(7 downto 0) <= rs1_reg(7 downto 0); -- sign-extend byte
  res_int(op_sexth_c)(data_width_c-1 downto 16) <= (others => rs1_reg(15));
  res_int(op_sexth_c)(15 downto 0) <= rs1_reg(15 downto 0); -- sign-extend half-word
  res_int(op_zexth_c)(data_width_c-1 downto 16) <= (others => '0');
  res_int(op_zexth_c)(15 downto 0) <= rs1_reg(15 downto 0); -- zero-extend half-word

  -- rotate right/left --
  res_int(op_ror_c) <= shifter.sreg;
  res_int(op_rol_c) <= bit_rev_f(shifter.sreg); -- reverse to compensate internal right-only shifts

  -- or-combine.byte --
  or_combine_gen:
  for i in 0 to (data_width_c/8)-1 generate -- sub-byte loop
    res_int(op_orcb_c)(i*8+7 downto i*8) <= (others => or_reduce_f(rs1_reg(i*8+7 downto i*8)));
  end generate; -- i

  -- reversal.8 (byte swap) --
  res_int(op_rev8_c) <= bswap32_f(rs1_reg);

  -- address generation instructions --
  res_int(op_sh1add_c) <= adder_core;
  res_int(op_sh2add_c) <= (others => '0'); -- unused/redundant
  res_int(op_sh3add_c) <= (others => '0'); -- unused/redundant


  -- Output Selector ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  res_out(op_andn_c)  <= res_int(op_andn_c)  when (cmd_buf(op_andn_c)  = '1') else (others => '0');
  res_out(op_orn_c)   <= res_int(op_orn_c)   when (cmd_buf(op_orn_c)   = '1') else (others => '0');
  res_out(op_xnor_c)  <= res_int(op_xnor_c)  when (cmd_buf(op_xnor_c)  = '1') else (others => '0');
  res_out(op_clz_c)   <= res_int(op_clz_c)   when ((cmd_buf(op_clz_c) or cmd_buf(op_ctz_c)) = '1') else (others => '0');
  res_out(op_ctz_c)   <= (others => '0'); -- unused/redundant
  res_out(op_cpop_c)  <= res_int(op_cpop_c)  when (cmd_buf(op_cpop_c)  = '1') else (others => '0');
  res_out(op_min_c)   <= res_int(op_min_c)   when ((cmd_buf(op_min_c) or cmd_buf(op_max_c)) = '1') else (others => '0');
  res_out(op_max_c)   <= (others => '0'); -- unused/redundant
  res_out(op_sextb_c) <= res_int(op_sextb_c) when (cmd_buf(op_sextb_c) = '1') else (others => '0');
  res_out(op_sexth_c) <= res_int(op_sexth_c) when (cmd_buf(op_sexth_c) = '1') else (others => '0');
  res_out(op_zexth_c) <= res_int(op_zexth_c) when (cmd_buf(op_zexth_c) = '1') else (others => '0');
  res_out(op_ror_c)   <= res_int(op_ror_c)   when (cmd_buf(op_ror_c)   = '1') else (others => '0');
  res_out(op_rol_c)   <= res_int(op_rol_c)   when (cmd_buf(op_rol_c)   = '1') else (others => '0');
  res_out(op_orcb_c)  <= res_int(op_orcb_c)  when (cmd_buf(op_orcb_c)  = '1') else (others => '0');
  res_out(op_rev8_c)  <= res_int(op_rev8_c)  when (cmd_buf(op_rev8_c)  = '1') else (others => '0');
  --
  res_out(op_sh1add_c) <= res_int(op_sh1add_c) when ((cmd_buf(op_sh1add_c) or cmd_buf(op_sh2add_c) or cmd_buf(op_sh3add_c))  = '1') else (others => '0');
  res_out(op_sh2add_c) <= (others => '0'); -- unused/redundant
  res_out(op_sh3add_c) <= (others => '0'); -- unused/redundant


  -- Output Gate ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  output_gate: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      res_o <= (others => def_rst_val_c);
    elsif rising_edge(clk_i) then
      res_o <= (others => '0');
      if (valid = '1') then
        res_o <= res_out(op_andn_c)  or res_out(op_orn_c)   or res_out(op_xnor_c) or
                 res_out(op_clz_c)   or res_out(op_cpop_c)  or -- res_out(op_ctz_c) is unused here
                 res_out(op_min_c)   or -- res_out(op_max_c) is unused here
                 res_out(op_sextb_c) or res_out(op_sexth_c) or res_out(op_zexth_c) or
                 res_out(op_ror_c)   or res_out(op_rol_c)   or
                 res_out(op_orcb_c)  or res_out(op_rev8_c)  or
                 res_out(op_sh1add_c); -- res_out(op_sh2add_c) and res_out(op_sh3add_c) are unused here
      end if;
    end if;
  end process output_gate;

  -- valid output --
  valid_o <= valid;


end neorv32_cpu_cp_bitmanip_rtl;
