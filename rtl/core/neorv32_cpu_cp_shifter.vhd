-- #################################################################################################
-- # << NEORV32 - CPU Co-Processor: Shifter (CPU Core ISA) >>                                      #
-- # ********************************************************************************************* #
-- # Bit-shift unit for base ISA.                                                                  #
-- # FAST_SHIFT_EN = false (default): Use bit-serial shifter architecture (small but slow)         #
-- # FAST_SHIFT_EN = true: Use barrel shifter architecture (large but fast)                        #
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

entity neorv32_cpu_cp_shifter is
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
    rs1_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 1
    shamt_i : in  std_ulogic_vector(index_size_f(data_width_c)-1 downto 0); -- shift amount
    -- result and status --
    res_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- operation result
    valid_o : out std_ulogic -- data output valid
  );
end neorv32_cpu_cp_shifter;

architecture neorv32_cpu_cp_shifter_rtl of neorv32_cpu_cp_shifter is

  -- serial shifter --
  type shifter_t is record
    busy    : std_ulogic;
    busy_ff : std_ulogic;
    done    : std_ulogic;
    cnt     : std_ulogic_vector(index_size_f(data_width_c)-1 downto 0);
    sreg    : std_ulogic_vector(data_width_c-1 downto 0);
    res     : std_ulogic_vector(data_width_c-1 downto 0);
  end record;
  signal shifter : shifter_t;

  -- barrel shifter --
  type bs_level_t is array (index_size_f(data_width_c) downto 0) of std_ulogic_vector(data_width_c-1 downto 0);
  signal bs_level  : bs_level_t;
  signal bs_result : std_ulogic_vector(data_width_c-1 downto 0);

begin

  -- Iterative Shifter Core (small but slow) ------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  serial_shifter_sync:
  if (FAST_SHIFT_EN = false) generate
    shifter_unit_sync: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        shifter.busy    <= '0';
        shifter.busy_ff <= def_rst_val_c;
        shifter.sreg    <= (others => def_rst_val_c);
        shifter.cnt     <= (others => def_rst_val_c);
      elsif rising_edge(clk_i) then
        shifter.busy_ff <= shifter.busy;
        if (start_i = '1') then
          shifter.busy <= '1';
        elsif (shifter.done = '1') then
          shifter.busy <= '0';
        end if;
        --
        if (start_i = '1') then -- trigger new shift
          shifter.sreg <= rs1_i; -- shift operand
          shifter.cnt  <= shamt_i; -- shift amount
        elsif (or_reduce_f(shifter.cnt) = '1') then -- running shift (cnt != 0)
          shifter.cnt <= std_ulogic_vector(unsigned(shifter.cnt) - 1);
          if (ctrl_i(ctrl_alu_shift_dir_c) = '0') then -- SLL: shift left logical
            shifter.sreg <= shifter.sreg(shifter.sreg'left-1 downto 0) & '0';
          else -- SRL: shift right logical / SRA: shift right arithmetical
            shifter.sreg <= (shifter.sreg(shifter.sreg'left) and ctrl_i(ctrl_alu_shift_ar_c)) & shifter.sreg(shifter.sreg'left downto 1);
          end if;
        end if;
      end if;
    end process shifter_unit_sync;
  end generate;

  -- shift control/output --
  serial_shifter_ctrl:
  if (FAST_SHIFT_EN = false) generate
    shifter.done <= not or_reduce_f(shifter.cnt(shifter.cnt'left downto 1));
    valid_o      <= shifter.busy and shifter.done;
    res_o        <= shifter.sreg when (shifter.busy = '0') and (shifter.busy_ff = '1') else (others => '0');
  end generate;


  -- Barrel Shifter Core (fast but large) ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  barrel_shifter_async:
  if (FAST_SHIFT_EN = true) generate
    shifter_unit_async: process(rs1_i, shamt_i, ctrl_i, bs_level)
    begin
      -- input level: convert left shifts to right shifts --
      if (ctrl_i(ctrl_alu_shift_dir_c) = '0') then -- is left shift?
        bs_level(index_size_f(data_width_c)) <= bit_rev_f(rs1_i); -- reverse bit order of input operand
      else
        bs_level(index_size_f(data_width_c)) <= rs1_i;
      end if;

      -- shifter array --
      for i in index_size_f(data_width_c)-1 downto 0 loop
        if (shamt_i(i) = '1') then
          bs_level(i)(data_width_c-1 downto data_width_c-(2**i)) <= (others => (bs_level(i+1)(data_width_c-1) and ctrl_i(ctrl_alu_shift_ar_c)));
          bs_level(i)((data_width_c-(2**i))-1 downto 0) <= bs_level(i+1)(data_width_c-1 downto 2**i);
        else
          bs_level(i) <= bs_level(i+1);
        end if;
      end loop;

      -- re-convert original left shifts --
      if (ctrl_i(ctrl_alu_shift_dir_c) = '0') then
        bs_result <= bit_rev_f(bs_level(0));
      else
        bs_result <= bs_level(0);
      end if;
    end process shifter_unit_async;
  end generate;

  -- output register --
  barrel_shifter_sync:
  if (FAST_SHIFT_EN = true) generate
    shifter_unit_sync: process(clk_i)
    begin
      if rising_edge(clk_i) then
        res_o <= (others => '0');
        if (start_i = '1') then
          res_o <= bs_result;
        end if;
      end if;
    end process shifter_unit_sync;
  end generate;

  -- shift control/output --
  barrel_shifter_ctrl:
  if (FAST_SHIFT_EN = true) generate
    valid_o <= start_i;
  end generate;


end neorv32_cpu_cp_shifter_rtl;
