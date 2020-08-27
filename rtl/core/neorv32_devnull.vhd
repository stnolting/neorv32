-- #################################################################################################
-- # << NEORV32 - /DEV/NULL (DEVNULL) Dummy Device with Simulation Output >>                       #
-- # ********************************************************************************************* #
-- # In simulation:    This unit will output the lowest 8 bit of the written data as ASCII chars   #
-- #                   to the simulator console and to a text file ("neorv32.devnull.out").        #
-- #                   The complete data 32-bit data word is dumped to "neorv32.devnull.data.out". #
-- # In real hardware: This unit implements a "/dev/null" device.                                  #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2020, Stephan Nolting. All rights reserved.                                     #
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
use std.textio.all;

entity neorv32_devnull is
  port (
    -- host access --
    clk_i  : in  std_ulogic; -- global clock line
    addr_i : in  std_ulogic_vector(31 downto 0); -- address
    rden_i : in  std_ulogic; -- read enable
    wren_i : in  std_ulogic; -- write enable
    data_i : in  std_ulogic_vector(31 downto 0); -- data in
    data_o : out std_ulogic_vector(31 downto 0); -- data out
    ack_o  : out std_ulogic  -- transfer acknowledge
  );
end neorv32_devnull;

architecture neorv32_devnull_rtl of neorv32_devnull is

  -- configuration --
  constant sim_text_output_en_c : boolean := true; -- output lowest byte as char to simulator and file when enabled
  constant sim_data_output_en_c : boolean := true; -- dump 32-word to file when enabled

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(devnull_size_c); -- low address boundary bit

  -- access control --
  signal acc_en : std_ulogic; -- module access enable

begin

  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = devnull_base_c(hi_abb_c downto lo_abb_c)) else '0';


  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rw_access: process(clk_i)
    file file_devnull_text_out : text open write_mode is "neorv32.devnull.out";
    file file_devnull_data_out : text open write_mode is "neorv32.devnull.data.out";
    variable i : integer;
    variable la, lb, lc : line; -- we need several variables here since "writeline" seems to flush the source variable
  begin
    if rising_edge(clk_i) then
      ack_o <= acc_en and (wren_i or rden_i);
      if (acc_en = '1') and (wren_i = '1') then
        if (sim_text_output_en_c = true) then
          -- print lowest byte as ASCII to console --
          i := to_integer(unsigned(data_i(7 downto 0)));
          if (i >= 128) then -- out of range?
            i := 0;
          end if;
          if (i /= 10) and (i /= 13) then -- skip line breaks - they are issued via "writeline"
            write(la, character'val(i));
            write(lb, character'val(i));
          end if;
          if (i = 10) then -- line break: write to screen and file
            writeline(output, la);
            writeline(file_devnull_text_out, lb);
          end if;
        end if;
        if (sim_data_output_en_c = true) then
          -- dump raw data
          for x in 7 downto 0 loop
            write(lc, to_hexchar_f(data_i(3+x*4 downto 0+x*4))); -- write in hex form
          end loop; -- x
          writeline(file_devnull_data_out, lc);
        end if;
      end if;
    end if;
  end process rw_access;

  -- output --
  data_o <= (others => '0');


end neorv32_devnull_rtl;
