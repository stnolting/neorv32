-- #################################################################################################
-- # << NEORV32 - Generic Clock Gating Switch >>                                                   #
-- # ********************************************************************************************* #
-- # This is a generic clock switch that allows to shut down the clock of certain processor        #
-- # modules in order to reduce power consumption.                                                 #
-- #                                                                                               #
-- # [NOTE] Especially for FPGA setups, it is highly recommended to replace this default module    #
-- #        by a technology-/platform-specific macro or primitive (e.g. a clock mux) wrapper.      #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # The NEORV32 RISC-V Processor, https://github.com/stnolting/neorv32                            #
-- # Copyright (c) 2024, Stephan Nolting. All rights reserved.                                     #
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
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;

entity neorv32_clockgate is
  port (
    clk_i  : in  std_ulogic; -- global clock line, always-on
    rstn_i : in  std_ulogic; -- global reset line, low-active, async
    halt_i : in  std_ulogic; -- shut down clock output when set
    clk_o  : out std_ulogic  -- switched clock output
  );
end neorv32_clockgate;

architecture neorv32_clockgate_rtl of neorv32_clockgate is

  signal enable : std_ulogic;

begin

  -- Warn about Clock Gating ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert false report "[NEORV32] Clock gating enabled (using generic clock switch)." severity warning;


  -- Clock Switch ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  clock_switch: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      enable <= '1';
    elsif falling_edge(clk_i) then -- update on falling edge to avoid glitches on 'clk_o'
      enable <= not halt_i;
    end if;
  end process clock_switch;

  -- for FPGA designs better replace this by a technology-specific primitive or macro --
  clk_o <= clk_i when (enable = '1') else '0';


end neorv32_clockgate_rtl;
