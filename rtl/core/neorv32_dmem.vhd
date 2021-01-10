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

entity neorv32_dmem is
  generic (
    DMEM_BASE : std_ulogic_vector(31 downto 0) := x"80000000"; -- memory base address
    DMEM_SIZE : natural := 4*1024  -- processor-internal instruction memory size in bytes
  );
  port (
    clk_i  : in  std_ulogic; -- global clock line
    rden_i : in  std_ulogic; -- read enable
    wren_i : in  std_ulogic; -- write enable
    ben_i  : in  std_ulogic_vector(03 downto 0); -- byte write enable
    addr_i : in  std_ulogic_vector(31 downto 0); -- address
    data_i : in  std_ulogic_vector(31 downto 0); -- data in
    data_o : out std_ulogic_vector(31 downto 0); -- data out
    ack_o  : out std_ulogic -- transfer acknowledge
  );
end neorv32_dmem;

architecture neorv32_dmem_rtl of neorv32_dmem is

  -- IO space: module base address --
  constant hi_abb_c : natural := 31; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(DMEM_SIZE); -- low address boundary bit

  -- local signals --
  signal acc_en : std_ulogic;
  signal rdata  : std_ulogic_vector(31 downto 0);
  signal rden   : std_ulogic;
  signal addr   : std_ulogic_vector(index_size_f(DMEM_SIZE/4)-1 downto 0);

  -- RAM --
  -- The memory is built from 4x byte-wide memories defined as unique signals, since many synthesis tools
  -- have problems with 32-bit memories with byte-enable signals or with multi-dimensional arrays.
  type dmem_file_t is array (0 to DMEM_SIZE/4-1) of std_ulogic_vector(7 downto 0);
  signal dmem_file_ll : dmem_file_t;
  signal dmem_file_lh : dmem_file_t;
  signal dmem_file_hl : dmem_file_t;
  signal dmem_file_hh : dmem_file_t;


  -- -------------------------------------------------------------------------------- --
  -- attributes - these are *NOT mandatory*; just for footprint / timing optimization --
  -- -------------------------------------------------------------------------------- --

  -- lattice radiant --
  attribute syn_ramstyle : string;
  attribute syn_ramstyle of dmem_file_ll : signal is "no_rw_check";
  attribute syn_ramstyle of dmem_file_lh : signal is "no_rw_check";
  attribute syn_ramstyle of dmem_file_hl : signal is "no_rw_check";
  attribute syn_ramstyle of dmem_file_hh : signal is "no_rw_check";

  -- intel quartus prime --
  attribute ramstyle : string;
  attribute ramstyle of dmem_file_ll : signal is "no_rw_check";
  attribute ramstyle of dmem_file_lh : signal is "no_rw_check";
  attribute ramstyle of dmem_file_hl : signal is "no_rw_check";
  attribute ramstyle of dmem_file_hh : signal is "no_rw_check";

begin

  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = DMEM_BASE(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= addr_i(index_size_f(DMEM_SIZE/4)+1 downto 2); -- word aligned


  -- Memory Access --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  dmem_file_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      rden  <= rden_i and acc_en;
      ack_o <= acc_en and (rden_i or wren_i);
      if (acc_en = '1') then -- reduce switching activity when not accessed
        -- write --
        if (wren_i = '1') then
          if (ben_i(0) = '1') then
            dmem_file_ll(to_integer(unsigned(addr))) <= data_i(07 downto 00);
          end if;
          if (ben_i(1) = '1') then
            dmem_file_lh(to_integer(unsigned(addr))) <= data_i(15 downto 08);
          end if;
          if (ben_i(2) = '1') then
            dmem_file_hl(to_integer(unsigned(addr))) <= data_i(23 downto 16);
          end if;
          if (ben_i(3) = '1') then
            dmem_file_hh(to_integer(unsigned(addr))) <= data_i(31 downto 24);
          end if;
        end if;
        -- read --
        rdata(07 downto 00) <= dmem_file_ll(to_integer(unsigned(addr)));
        rdata(15 downto 08) <= dmem_file_lh(to_integer(unsigned(addr)));
        rdata(23 downto 16) <= dmem_file_hl(to_integer(unsigned(addr)));
        rdata(31 downto 24) <= dmem_file_hh(to_integer(unsigned(addr)));
      end if;
    end if;
  end process dmem_file_access;

  -- output gate --
  data_o <= rdata when (rden = '1') else (others => '0');


end neorv32_dmem_rtl;
