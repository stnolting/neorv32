-- #################################################################################################
-- # << NEORV32 - CPU Co-Processor: Single-Precision Floating Point Unit (RISC-V "F" Extension) >> #
-- # ********************************************************************************************* #
-- #                                                                                               #
-- #                            !!!        WORK-IN-PROGRESS         !!!                            #
-- #                            !!! THIS UNIT IS NOT FUNCTIONAL YET !!!                            #
-- #                                                                                               #
-- # ********************************************************************************************* #
-- # !!! Enabling the F extension does not has an effect on the CPU. If F is enabled, there    !!! #
-- # !!! will be no traps when trying to execute floating-point instructions, since the main   !!! #
-- # !!! CPU control unit allready provides all necessary F-extension infrastructure.          !!! #
-- # !!! However, all F instructions will always return zero.                                  !!! #
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

entity neorv32_cpu_cp_fpu is
  port (
    -- global control --
    clk_i     : in  std_ulogic; -- global clock, rising edge
    rstn_i    : in  std_ulogic; -- global reset, low-active, async
    ctrl_i    : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
    start_i   : in  std_ulogic; -- trigger operation
    -- data input --
    frm_i     : in  std_ulogic_vector(2 downto 0); -- rounding mode
    reg_i     : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source (rs1)
    mem_i     : in  std_ulogic_vector(data_width_c-1 downto 0); -- memory read-data
    -- result and status --
    fflags_o  : out std_ulogic_vector(4 downto 0); -- exception flags
    mem_o     : out std_ulogic_vector(data_width_c-1 downto 0); -- memory write-data
    res_o     : out std_ulogic_vector(data_width_c-1 downto 0); -- operation result
    valid_o   : out std_ulogic -- data output valid
  );
end neorv32_cpu_cp_fpu;

architecture neorv32_cpu_cp_fpu_rtl of neorv32_cpu_cp_fpu is

begin

  -- There is nothing to see here yet -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  fflags_o <= (others => '0');
  mem_o    <= (others => '0');
  res_o    <= (others => '0');
  valid_o  <= start_i;


end neorv32_cpu_cp_fpu_rtl;
