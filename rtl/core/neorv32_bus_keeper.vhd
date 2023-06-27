-- #################################################################################################
-- # << NEORV32 - Bus Keeper (BUSKEEPER) >>                                                        #
-- # ********************************************************************************************* #
-- # This unit monitors the processor-internal bus. If the accessed module does not respond within #
-- # the defined number of cycles (VHDL package: max_proc_int_response_time_c) or issues an ERROR  #
-- # condition, the BUS KEEPER asserts the error signal to inform the CPU.                         #
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

entity neorv32_bus_keeper is
  port (
    clk_i     : in  std_ulogic; -- global clock line
    rstn_i    : in  std_ulogic; -- global reset, low-active, async
    bus_req_i : in  bus_req_t;  -- monitor request bus
    bus_rsp_i : in  bus_rsp_t;  -- monitor response bus
    bus_err_o : out std_ulogic; -- signal bus error to CPU
    bus_tmo_i : in  std_ulogic; -- transfer timeout (external interface)
    bus_ext_i : in  std_ulogic; -- external bus access
    bus_xip_i : in  std_ulogic  -- pending XIP access
  );
end neorv32_bus_keeper;

architecture neorv32_bus_keeper_rtl of neorv32_bus_keeper is

  -- timeout counter size --
  constant cnt_width_c : natural := index_size_f(max_proc_int_response_time_c);

  -- controller --
  type ctrl_t is record
    pending : std_ulogic;
    timeout : std_ulogic_vector(cnt_width_c-1 downto 0);
    bus_err : std_ulogic;
    ignore  : std_ulogic;
  end record;
  signal ctrl : ctrl_t;

begin

  -- Sanity Check --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (max_proc_int_response_time_c < 2)
    report "NEORV32 PROCESSOR CONFIG ERROR! Processor-internal bus timeout <max_proc_int_response_time_c> has to >= 2." severity error;


  -- Monitor --------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  keeper_control: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl.pending <= '0';
      ctrl.bus_err <= '0';
      ctrl.timeout <= (others => '0');
      ctrl.ignore  <= '0';
    elsif rising_edge(clk_i) then
      -- defaults --
      ctrl.bus_err <= '0';
      -- bus idle --
      if (ctrl.pending = '0') then
        ctrl.timeout <= std_ulogic_vector(to_unsigned(max_proc_int_response_time_c-1, cnt_width_c));
        ctrl.ignore  <= '0';
        if (bus_req_i.re = '1') or (bus_req_i.we = '1') then
          ctrl.pending <= '1';
        end if;
      -- bus access pending --
      else
        -- countdown timer --
        ctrl.timeout <= std_ulogic_vector(unsigned(ctrl.timeout) - 1);
        -- bus keeper shall ignore internal timeout during this access (because it's "external") --
        ctrl.ignore <= ctrl.ignore or (bus_ext_i or bus_xip_i);
        -- response check --
        if (bus_rsp_i.err = '1') or -- error termination by bus system
           (bus_tmo_i = '1') or -- EXTERNAL access timeout
           ((or_reduce_f(ctrl.timeout) = '0') and (ctrl.ignore = '0')) then -- INTERNAL access timeout
          ctrl.bus_err <= '1';
          ctrl.pending <= '0';
        elsif (bus_rsp_i.ack = '1') then -- normal termination by bus system
          ctrl.bus_err <= '0';
          ctrl.pending <= '0';
        end if;
      end if;
    end if;
  end process keeper_control;

  -- signal bus error to CPU --
  bus_err_o <= ctrl.bus_err;


end neorv32_bus_keeper_rtl;
