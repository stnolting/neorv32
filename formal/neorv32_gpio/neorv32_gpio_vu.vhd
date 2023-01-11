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
  gpio_sel_vu <= '1' when addr_i >= gpio_base_c and
                          unsigned(addr_i) <= (unsigned(gpio_base_c) + gpio_size_c - 1) else
                 '0';
  -- dword aligned address
  addr_vu <= addr_i(addr_i'left downto 2) & "00";

  -- All is sensitive to rising edge of clk
  default clock is rising_edge(clk_i);

  -- Initial reset of 3 cycles
  restrict {not rstn_i[*3]; rstn_i[+]}[*1];

  -- Some assumptions about the bus interface
  assume always (not rstn_i -> not wren_i and not rden_i);
  assume always (rstn_i -> not (wren_i and rden_i));
  assume always (wren_i or rden_i) -> addr_i(31 downto 8) = x"ffffff";


  -- Async reset of GPIO outs
  process (all) is
  begin
    if (not rstn_i) then
      assert gpio_o = 64x"0";
    end if;
  end process;

  -- Never write gpio outs without write enable
  assert always (
    not wren_i -> next stable(gpio_o)
  );

  -- Never write lower gpio outs without gpio out low address
  assert always (
    addr_vu /= gpio_out_lo_addr_c -> next stable (gpio_o(31 downto 0))
  );

  -- Never write upper gpio outs without gpio out high address
  assert always (
    addr /= gpio_out_hi_addr_c -> next stable (gpio_o(63 downto 32))
  );

  -- Always write lower gpios at next cycle if wren and gpio out low address
  assert always (
    wren_i and addr_vu = gpio_out_lo_addr_c -> next gpio_o(31 downto 0) = prev(data_i)
  ) abort not rstn_i;

  -- Always write upper gpios at next cycle if wren and gpio out high address
  assert always (
    wren_i and addr_vu = gpio_out_hi_addr_c -> next gpio_o(63 downto 32) = prev(data_i)
  ) abort not rstn_i;

  -- Always ack or err response in next cycle
  assert always (
    gpio_sel_vu and (wren_i or rden_i) -> next (ack_o or err_o)
  ) abort not rstn_i;

  -- Never ack & err at same cycle
  assert always (
    not (ack_o and err_o)
  ) abort not rstn_i;

}