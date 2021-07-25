-- #################################################################################################
-- # << NEORV32 - Bus Keeper (BUSKEEPER) >>                                                        #
-- # ********************************************************************************************* #
-- # This unit monitors the processor-internal bus. If the accessed INTERNAL (IMEM if enabled,     #
-- # DMEM if enabled, BOOTROM + IO region) module does not respond within the defined number of    #
-- # cycles (VHDL package: max_proc_int_response_time_c) the BUS KEEPER asserts the error signal   #
-- # to inform the CPU / bus driver.                                                               #
-- #                                                                                               #
-- # WARNING: The bus keeper timeout does not track accesses via the processor-external bus        #
-- #          interface! If the timeout-function of the Wishbone interface is not used, the CPU    #
-- #          might be permanently stalled by an an unacknowledged transfer! If the external bus   #
-- #          interface is disabled, ALL accesses by the CPU are internal.                         #
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

entity neorv32_bus_keeper is
  generic (
    -- External memory interface --
    MEM_EXT_EN        : boolean; -- implement external memory bus interface?
    -- Internal instruction memory --
    MEM_INT_IMEM_EN   : boolean; -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE : natural; -- size of processor-internal instruction memory in bytes
    -- Internal data memory --
    MEM_INT_DMEM_EN   : boolean; -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE : natural  -- size of processor-internal data memory in bytes
  );
  port (
    -- host access --
    clk_i  : in  std_ulogic; -- global clock line
    rstn_i : in  std_ulogic; -- global reset line, low-active
    addr_i : in  std_ulogic_vector(31 downto 0); -- address
    rden_i : in  std_ulogic; -- read enable
    wren_i : in  std_ulogic; -- write enable
    ack_i  : in  std_ulogic; -- transfer acknowledge from bus system
    err_i  : in  std_ulogic; -- transfer error from bus system
    err_o  : out std_ulogic  -- bus error
  );
end neorv32_bus_keeper;

architecture neorv32_bus_keeper_rtl of neorv32_bus_keeper is

  -- access check --
  type access_check_t is record
    int_imem       : std_ulogic;
    int_dmem       : std_ulogic;
    int_bootrom_io : std_ulogic;
    valid          : std_ulogic;
  end record;
  signal access_check : access_check_t;

  -- controller --
  type control_t is record
    pending : std_ulogic;
    timeout : std_ulogic_vector(index_size_f(max_proc_int_response_time_c)-1 downto 0);
    bus_err : std_ulogic;
  end record;
  signal control : control_t;

begin

  -- Sanity Check --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (max_proc_int_response_time_c < 2) report "NEORV32 PROCESSOR CONFIG ERROR! Processor-internal bus timeout <max_proc_int_response_time_c> has to >= 2." severity error;


  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- access to processor-internal IMEM or DMEM? --
  access_check.int_imem <= '1' when (addr_i(31 downto index_size_f(MEM_INT_IMEM_SIZE)) = imem_base_c(31 downto index_size_f(MEM_INT_IMEM_SIZE))) and (MEM_INT_IMEM_EN = true) else '0';
  access_check.int_dmem <= '1' when (addr_i(31 downto index_size_f(MEM_INT_DMEM_SIZE)) = dmem_base_c(31 downto index_size_f(MEM_INT_DMEM_SIZE))) and (MEM_INT_DMEM_EN = true) else '0';
  -- access to processor-internal BOOTROM or IO devices? --
  access_check.int_bootrom_io <= '1' when (addr_i(31 downto 16) = boot_rom_base_c(31 downto 16)) else '0'; -- hacky!
  -- actual internal bus access? --
  access_check.valid <= access_check.int_imem or access_check.int_dmem or access_check.int_bootrom_io;


  -- Keeper ---------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  keeper_control: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      control.pending <= '0';
      control.bus_err <= '0';
      control.timeout <= (others => def_rst_val_c);
    elsif rising_edge(clk_i) then

      -- pending access? --
      control.bus_err <= '0';
      if (control.pending = '0') then -- idle
        if ((rden_i or wren_i) = '1') and ((access_check.valid = '1') or (MEM_EXT_EN = false)) then -- valid INTERNAL access
          control.pending <= '1';
        end if;
      else -- pending
        if (ack_i = '1') or (err_i = '1') then -- termination by bus system
          control.pending <= '0';
        elsif (or_reduce_f(control.timeout) = '0') then -- timeout! terminate bus transfer
          control.pending <= '0';
          control.bus_err <= '1';
        end if;
      end if;

      -- timeout counter --
      if (control.pending = '0') then
        control.timeout <= std_ulogic_vector(to_unsigned(max_proc_int_response_time_c, index_size_f(max_proc_int_response_time_c)));
      else
        control.timeout <= std_ulogic_vector(unsigned(control.timeout) - 1); -- countdown timer
      end if;
    end if;
  end process keeper_control;

  err_o <= control.bus_err;


end neorv32_bus_keeper_rtl;
