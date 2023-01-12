-- #################################################################################################
-- # << NEORV32 - GPIO PSL vunit  file >>                                                          #
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


vunit neorv32_gpio_vu (neorv32_gpio(neorv32_gpio_rtl)) {

  signal gpio_sel_vu : std_logic;
  signal addr_vu     : std_logic_vector(addr_i'range);

  -- GPIO address space is selected
  gpio_sel_vu <= '1' when unsigned(addr_i) >= unsigned(gpio_base_c) and
                          unsigned(addr_i) <= (unsigned(gpio_base_c) + gpio_size_c - 1) else
                 '0';

  -- dword aligned address
  addr_vu <= addr_i(addr_i'left downto 2) & "00";

  -- All is sensitive to rising edge of clk
  default clock is rising_edge(clk_i);


  -- Initial reset of 3 cycles
  restrict {not rstn_i[*3]; rstn_i[+]}[*1];


  -- Some assumptions about the bus interface
  -- These assumptions are based on the bus interface specification
  -- in section '3.12.12. Bus Interface' of the neorv32 documentation.
  -- They should later moved into a separate module for reusability.

  -- No read / write request during reset
  assume always (not rstn_i -> not wren_i and not rden_i);

  -- Read / write requests last one cycle only
  -- There are no new requests until peripheral responds
  assume always (wren_i -> next (not wren_i and not rden_i until_ ack_o or err_o));
  assume always (rden_i -> next (not rden_i and not wren_i until_ ack_o or err_o));
  assume never (wren_i and rden_i);

  -- Signal stability during write bus cycle
  assume always (wren_i -> next (stable(addr_i) until_ (ack_o or err_o)));
  assume always (wren_i -> next (stable(data_o) until_ (ack_o or err_o)));

  -- Signal stability during read bus cycle
  assume always (rden_i -> next (stable(addr_i) until_ (ack_o or err_o)));

  -- @TODO: Remove address restriction
  assume always wren_i or rden_i -> addr_i(addr_i'left downto 8) = x"FFFFFF";


  -- Async reset of GPIO outs
  process (all) is
  begin
    if (not rstn_i) then
      GPIO_RESET : assert gpio_o = 64x"0";
    end if;
  end process;

  -- Never write gpio outs without write enable
  GPIO_STABLE :  assert always (
    not wren_i -> next stable(gpio_o)
  );

  -- Never write lower gpio outs without gpio out low address
  NO_LO_WRITE : assert always (
    addr_vu /= gpio_out_lo_addr_c -> next stable (gpio_o(31 downto 0))
  );

  -- Never write upper gpio outs without gpio out high address
  NO_HI_WRITE : assert always (
    addr /= gpio_out_hi_addr_c -> next stable (gpio_o(63 downto 32))
  );

  -- Always ack or err response in next cycle
  REQ_RESP : assert always (
    (wren_i or rden_i) and gpio_sel_vu -> next (ack_o or err_o)
  );

  -- Never ack & err at same cycle
  RESP_ONEHOT0 : assert always (
    not (ack_o and err_o)
  ) abort not rstn_i;

  -- Responses active one cycle only
  RESP_ONE_CYCLE : assert always (
    ack_o or err_o -> next not ack_o and not err_o
  );

  -- Always ack response if write / read at valid addresses
  ACK_RESP : assert always (
    (wren_i and (addr_vu = gpio_out_lo_addr_c or addr_vu = gpio_out_hi_addr_c)) or (rden_i and gpio_sel_vu) ->
    next ack_o
  );

  -- Always err response if write at invalid addresses
  ERR_RESP : assert always (
    (wren_i and gpio_sel_vu and addr_vu /= gpio_out_lo_addr_c and addr_vu /= gpio_out_hi_addr_c) ->
    next err_o
  );

  -- Always write lower gpios at next cycle if wren and gpio out low address
  WRITE_OUT_LO : assert always (
    wren_i and addr_vu = gpio_out_lo_addr_c -> next gpio_o(31 downto 0) = prev(data_i)
  );

  -- Always write upper gpios at next cycle if wren and gpio out high address
  WRITE_OUT_HI : assert always (
    wren_i and addr_vu = gpio_out_hi_addr_c -> next gpio_o(63 downto 32) = prev(data_i)
  );

  -- Always set data out to state of gpio inputs 31:0 two cycles before if read at gpio in low address
  READ_IN_LO : assert always (
    rden_i and addr_vu = gpio_in_lo_addr_c -> next data_o = prev(gpio_i(31 downto 0), 2)
  );

  -- Always set data out to state of gpio inputs 63:32 two cycles before if read at gpio in high address
  READ_IN_HI : assert always (
    rden_i and addr_vu = gpio_in_hi_addr_c -> next data_o = prev(gpio_i(63 downto 32), 2)
  );

  -- Always set data out to state of gpio outputs 31:0 one cycle before if read at gpio out low address
  READ_OUT_LO : assert always (
    rden_i and addr_vu = gpio_out_lo_addr_c -> next data_o = prev(gpio_o(31 downto 0), 1)
  );

  -- Always set data out to state of gpio outputs 63:32 one cycle before if read at gpio out high address
  READ_OUT_HI : assert always (
    rden_i and addr_vu = gpio_out_hi_addr_c -> next data_o = prev(gpio_o(63 downto 32), 1)
  );

}
