-- #################################################################################################
-- # << NEORV32 - NEORV32 bus protocol PSL model >>                                                #
-- # ********************************************************************************************* #
-- # Implements a simple model of the NEORV32 bus protocol using PSL. Use ASSUMES generic to       #
-- # between assert (verifying behavior) and assume (constraining behavior) directives.            #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Torsten Meissner. All rights reserved.                                    #
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


entity neorv32_bus_protocol is
  generic (
    ASSUMES : boolean := false
  );
  port (
    clk_i  : in  std_ulogic;
    rstn_i : in  std_ulogic;
    addr_i : in  std_ulogic_vector(31 downto 0); -- address
    rden_i : in  std_ulogic; -- read enable
    wren_i : in  std_ulogic; -- write enable
    data_i : in  std_ulogic_vector(31 downto 0); -- data in
    data_o : in  std_ulogic_vector(31 downto 0); -- data out
    ack_o  : in  std_ulogic; -- transfer acknowledge
    err_o  : in  std_ulogic  -- transfer error
  );
end entity;


architecture formal of neorv32_bus_protocol is

begin

  -- All is sensitive to rising edge of clk
  default clock is rising_edge(clk_i);

  -- define properties for reuse

  -- Some assumptions about the bus interface
  -- These assumptions are based on the bus interface specification
  -- in section '3.12.12. Bus Interface' of the neorv32 documentation.

  -- No read / write request during reset
  property no_read_write_during_reset is (
    always (not rstn_i -> not wren_i and not rden_i)
  );

  -- Read / write requests last one cycle only
  -- There are no new requests until peripheral responds
  property no_read_write_until_write_resp is (
    always (wren_i -> next (not wren_i and not rden_i until_ ack_o or err_o))
  );
  property no_read_write_until_read_resp is (
    always (rden_i -> next (not rden_i and not wren_i until_ ack_o or err_o))
  );
  
  -- Never write and read request at the same time
  property onehot0_read_write is (
    never (wren_i and rden_i)
  );

  -- Signal stability during write bus cycle
  property addr_stable_until_write_resp is (
    always (wren_i -> next (stable(addr_i) until_ (ack_o or err_o)))
  );
  property data_stable_until_write_resp is (
    always (wren_i -> next (stable(data_o) until_ (ack_o or err_o)))
  );

  -- Signal stability during read bus cycle
  property addr_stable_until_read_resp is (
    always (rden_i -> next (stable(addr_i) until_ (ack_o or err_o)))
  );


  ASSUMES : if ASSUMES generate

    assume no_read_write_during_reset;
    assume no_read_write_until_write_resp;
    assume no_read_write_until_read_resp;
    assume onehot0_read_write;
    assume addr_stable_until_write_resp;
    assume data_stable_until_write_resp;
    assume addr_stable_until_read_resp;

  else generate

    assert no_read_write_during_reset;
    assert no_read_write_until_write_resp;
    assert no_read_write_until_read_resp;
    assert onehot0_read_write;
    assert addr_stable_until_write_resp;
    assert data_stable_until_write_resp;
    assert addr_stable_until_read_resp;

  end generate;

end architecture;
