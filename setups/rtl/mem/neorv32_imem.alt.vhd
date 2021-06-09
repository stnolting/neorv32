-- #################################################################################################
-- # << NEORV32 - Processor-Internal instruction memory (IMEM) >>                                  #
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
use neorv32.neorv32_application_image.all;

architecture neorv32_imem_rtl of neorv32_imem is

  type ram_t is array(0 to IMEM_SIZE/4-1) of std_ulogic_vector(data_o'range);

  impure function init_imem(init : application_init_image_t) return ram_t is
    variable mem_v : ram_t := (others => (others => '0'));
  begin
    for i in init'range loop
      mem_v(i) := init(i);
    end loop;
    return mem_v;
  end function init_imem;

  signal rden : std_ulogic;
  signal data : std_ulogic_vector(data_o'range);
  signal addr : std_ulogic_vector(index_size_f(IMEM_SIZE/4)-1 downto 0);

  subtype acc_slice is integer range 31 downto index_size_f(IMEM_SIZE);

  signal acc_en : std_ulogic;
  signal acc_rd : std_ulogic;
  signal acc_wr : std_ulogic;

begin

  addr <= addr_i(index_size_f(IMEM_SIZE/4)+1 downto 2);

  acc_en <= '1' when addr_i(acc_slice) = IMEM_BASE(acc_slice) else '0';
  acc_rd <= acc_en and rden_i;
  acc_wr <= acc_en and wren_i;

  process(clk_i)
    variable memory : ram_t := init_imem(application_init_image);
  begin
    if rising_edge(clk_i) then
      rden <= acc_rd;
      ack_o <= acc_rd or acc_wr;
      if acc_rd then
        data <= memory(to_integer(unsigned(addr)));
      end if;
      if acc_wr then
        for x in 0 to 3 loop
          if ben_i(x) then
            memory(to_integer(unsigned(addr)))((x+1)*8-1 downto x*8) := data_i((x+1)*8-1 downto x*8);
          end if;
        end loop;
      end if;
    end if;
  end process;

  data_o <= data when rden else (others => '0');

end architecture;
