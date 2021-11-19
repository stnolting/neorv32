-- #################################################################################################
-- # << NEORV32 - Processor-internal data memory (DMEM) >>                                         #
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

architecture neorv32_dmem_rtl of neorv32_dmem is

  -- IO space: module base address --
  constant hi_abb_c : natural := 31; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(DMEM_SIZE); -- low address boundary bit

  -- local signals --
  signal acc_en  : std_ulogic;
  signal rdata   : std_ulogic_vector(31 downto 0);
  signal rden    : std_ulogic;
  signal addr    : std_ulogic_vector(index_size_f(DMEM_SIZE/4)-1 downto 0);
  signal addr_ff : std_ulogic_vector(index_size_f(DMEM_SIZE/4)-1 downto 0);

  -- -------------------------------------------------------------------------------------------------------------- --
  -- The memory (RAM) is built from 4 individual byte-wide memories b0..b3, since some synthesis tools have         --
  -- problems with 32-bit memories that provide dedicated byte-enable signals AND/OR with multi-dimensional arrays. --
  -- -------------------------------------------------------------------------------------------------------------- --

  -- RAM - not initialized at all --
  signal mem_ram_b0 : mem8_t(0 to DMEM_SIZE/4-1);
  signal mem_ram_b1 : mem8_t(0 to DMEM_SIZE/4-1);
  signal mem_ram_b2 : mem8_t(0 to DMEM_SIZE/4-1);
  signal mem_ram_b3 : mem8_t(0 to DMEM_SIZE/4-1);

  -- read data --
  signal mem_ram_b0_rd, mem_ram_b1_rd, mem_ram_b2_rd, mem_ram_b3_rd : std_ulogic_vector(7 downto 0);

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert false report "NEORV32 PROCESSOR CONFIG NOTE: Using CYCLONE-2-optimized HDL style DMEM." severity note;
  assert false report "NEORV32 PROCESSOR CONFIG NOTE: Implementing processor-internal DMEM (RAM, " & natural'image(DMEM_SIZE) & " bytes)." severity note;


  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = DMEM_BASE(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= addr_i(index_size_f(DMEM_SIZE/4)+1 downto 2); -- word aligned


  -- Memory Access --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      addr_ff <= addr;
      if (acc_en = '1') then -- reduce switching activity when not accessed
        if (wren_i = '1') and (ben_i(0) = '1') then -- byte 0
          mem_ram_b0(to_integer(unsigned(addr))) <= data_i(07 downto 00);
        end if;
        if (wren_i = '1') and (ben_i(1) = '1') then -- byte 1
          mem_ram_b1(to_integer(unsigned(addr))) <= data_i(15 downto 08);
        end if;
        if (wren_i = '1') and (ben_i(2) = '1') then -- byte 2
          mem_ram_b2(to_integer(unsigned(addr))) <= data_i(23 downto 16);
        end if;
        if (wren_i = '1') and (ben_i(3) = '1') then -- byte 3
          mem_ram_b3(to_integer(unsigned(addr))) <= data_i(31 downto 24);
        end if;
      end if;
    end if;
  end process mem_access;

  -- sync(!) read - alternative HDL style --
  mem_ram_b0_rd <= mem_ram_b0(to_integer(unsigned(addr_ff)));
  mem_ram_b1_rd <= mem_ram_b1(to_integer(unsigned(addr_ff)));
  mem_ram_b2_rd <= mem_ram_b2(to_integer(unsigned(addr_ff)));
  mem_ram_b3_rd <= mem_ram_b3(to_integer(unsigned(addr_ff)));


  -- Bus Feedback ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_feedback: process(clk_i)
  begin
    if rising_edge(clk_i) then
      rden  <= acc_en and rden_i;
      ack_o <= acc_en and (rden_i or wren_i);
    end if;
  end process bus_feedback;

  -- pack --
  rdata <= mem_ram_b3_rd & mem_ram_b2_rd & mem_ram_b1_rd & mem_ram_b0_rd;

  -- output gate --
  data_o <= rdata when (rden = '1') else (others => '0');


end neorv32_dmem_rtl;
