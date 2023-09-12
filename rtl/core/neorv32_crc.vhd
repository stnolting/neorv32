-- #################################################################################################
-- # << NEORV32 - Cyclic Redundancy Check Unit (CRC) >>                                            #
-- # ********************************************************************************************* #
-- # Bit-serial / iterative CRC computation module with programmable polynomial and operating mode #
-- # (CRC8, CRC16, CRC32). The write access ACK signal is DELAYED to ensure that the current CRC   #
-- # has completed before new data can be written.                                                 #
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
-- # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_crc is
  port (
    clk_i     : in  std_ulogic; -- global clock line
    rstn_i    : in  std_ulogic; -- global reset line, low-active
    bus_req_i : in  bus_req_t;  -- bus request
    bus_rsp_o : out bus_rsp_t   -- bus response
  );
end neorv32_crc;

architecture neorv32_crc_rtl of neorv32_crc is

  -- interface register addresses --
  constant mode_addr_c : std_ulogic_vector(1 downto 0) := "00"; -- r/w: mode register
  constant poly_addr_c : std_ulogic_vector(1 downto 0) := "01"; -- r/w: polynomial register
  constant data_addr_c : std_ulogic_vector(1 downto 0) := "10"; -- -/w: data register
  constant sreg_addr_c : std_ulogic_vector(1 downto 0) := "11"; -- r/w: CRC shift register

  -- CRC core --
  type crc_t is record
    mode : std_ulogic_vector(01 downto 0);
    poly : std_ulogic_vector(31 downto 0);
    data : std_ulogic_vector(07 downto 0);
    sreg : std_ulogic_vector(31 downto 0);
    --
    cnt  : std_ulogic_vector(03 downto 0);
    msb  : std_ulogic;
  end record;
  signal crc : crc_t;

  -- delayed ACK on write access --
  signal we_ack : std_ulogic_vector(5 downto 0); -- to wait for serial CRC processing

begin

  -- Host Access ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- write access --
  write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      crc.mode <= (others => '0');
      crc.poly <= (others => '0');
      crc.data <= (others => '0');
      we_ack   <= (others => '0');
    elsif rising_edge(clk_i) then
      if (bus_req_i.we = '1') then
        if (bus_req_i.addr(3 downto 2) = mode_addr_c) then -- mode select
          crc.mode <= bus_req_i.data(01 downto 0);
        end if;
        if (bus_req_i.addr(3 downto 2) = poly_addr_c) then -- polynomial
          crc.poly <= bus_req_i.data(31 downto 0);
        end if;
        if (bus_req_i.addr(3 downto 2) = data_addr_c) then -- data
          crc.data <= bus_req_i.data(07 downto 0);
        end if;
      end if;
      -- delayed write ACK --
      we_ack <= we_ack(we_ack'left-1 downto 0) & bus_req_i.we;
    end if;
  end process write_access;

  -- read access --
  read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      bus_rsp_o.data <= (others => '0');
      bus_rsp_o.ack  <= we_ack(we_ack'left) or bus_req_i.re;
      if (bus_req_i.re = '1') then
        case bus_req_i.addr(3 downto 2) is
          when mode_addr_c => bus_rsp_o.data(01 downto 0) <= crc.mode; -- mode select
          when poly_addr_c => bus_rsp_o.data(31 downto 0) <= crc.poly; -- polynomial
          when others      => bus_rsp_o.data(31 downto 0) <= crc.sreg; -- CRC result
        end case;
      end if;
    end if;
  end process read_access;

  -- no access error possible --
  bus_rsp_o.err <= '0';


  -- Bit-Serial CRC Core --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  crc_core: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      crc.cnt  <= (others => '1');
      crc.sreg <= (others => '0');
    elsif rising_edge(clk_i) then
      -- arbitration --
      if (bus_req_i.we = '1') and (bus_req_i.addr(3 downto 2) = data_addr_c) then -- writing new data
        crc.cnt <= "0111"; -- start with MSB
      elsif (crc.cnt(3) = '0') then -- not done yet?
        crc.cnt <= std_ulogic_vector(unsigned(crc.cnt) - 1);
      end if;
      -- computation --
      if (bus_req_i.we = '1') and (bus_req_i.addr(3 downto 2) = sreg_addr_c) then -- set start value
        crc.sreg <= bus_req_i.data;
      elsif (crc.cnt(3) = '0') then
        if (crc.msb = crc.data(to_integer(unsigned(crc.cnt(2 downto 0))))) then
          crc.sreg <= (crc.sreg(30 downto 0) & '0');
        else
          crc.sreg <= (crc.sreg(30 downto 0) & '0') xor crc.poly;
        end if;
      end if;
    end if;
  end process crc_core;

  -- operation mode --
	with crc.mode select crc.msb <=
		crc.sreg(07) when "00",   -- crc8
		crc.sreg(15) when "01",   -- crc16
		crc.sreg(31) when others; -- crc32


end neorv32_crc_rtl;
