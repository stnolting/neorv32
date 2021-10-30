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
    clk_i      : in  std_ulogic; -- global clock line
    rstn_i     : in  std_ulogic; -- global reset, low-active, async
    addr_i     : in  std_ulogic_vector(31 downto 0); -- address
    rden_i     : in  std_ulogic; -- read enable
    wren_i     : in  std_ulogic; -- write enable
    data_o     : out std_ulogic_vector(31 downto 0); -- data out
    ack_o      : out std_ulogic; -- transfer acknowledge
    err_o      : out std_ulogic; -- transfer error
    -- bus monitoring --
    bus_addr_i : in  std_ulogic_vector(31 downto 0); -- address
    bus_rden_i : in  std_ulogic; -- read enable
    bus_wren_i : in  std_ulogic; -- write enable
    bus_ack_i  : in  std_ulogic; -- transfer acknowledge from bus system
    bus_err_i  : in  std_ulogic  -- transfer error from bus system
  );
end neorv32_bus_keeper;

architecture neorv32_bus_keeper_rtl of neorv32_bus_keeper is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(buskeeper_size_c); -- low address boundary bit

  -- Control register --
  constant ctrl_err_type_c : natural :=  0; -- r/-: error type: 0=device error, 1=access timeout
  constant ctrl_err_src_c  : natural :=  1; -- r/-: error source: 0=processor-external, 1=processor-internal
  constant ctrl_err_flag_c : natural := 31; -- r/c: bus error encountered, sticky; cleared by writing zero

  -- sticky error flag --
  signal err_flag : std_ulogic;

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- bus access check --
  type access_check_t is record
    int_imem       : std_ulogic;
    int_dmem       : std_ulogic;
    int_bootrom_io : std_ulogic;
    valid          : std_ulogic;
  end record;
  signal access_check : access_check_t;

  -- controller --
  type control_t is record
    pending  : std_ulogic;
    timeout  : std_ulogic_vector(index_size_f(max_proc_int_response_time_c)-1 downto 0);
    err_type : std_ulogic;
    int_ext  : std_ulogic;
    bus_err  : std_ulogic;
  end record;
  signal control : control_t;

begin

  -- Sanity Check --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (max_proc_int_response_time_c < 2) report "NEORV32 PROCESSOR CONFIG ERROR! Processor-internal bus timeout <max_proc_int_response_time_c> has to >= 2." severity error;


  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = buskeeper_base_c(hi_abb_c downto lo_abb_c)) else '0';
  wren   <= acc_en and wren_i;
  rden   <= acc_en and rden_i;


  -- Bus Access Check -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- access to processor-internal IMEM or DMEM? --
  access_check.int_imem <= '1' when (bus_addr_i(31 downto index_size_f(MEM_INT_IMEM_SIZE)) = imem_base_c(31 downto index_size_f(MEM_INT_IMEM_SIZE))) and (MEM_INT_IMEM_EN = true) else '0';
  access_check.int_dmem <= '1' when (bus_addr_i(31 downto index_size_f(MEM_INT_DMEM_SIZE)) = dmem_base_c(31 downto index_size_f(MEM_INT_DMEM_SIZE))) and (MEM_INT_DMEM_EN = true) else '0';
  -- access to processor-internal BOOTROM or IO devices? --
  access_check.int_bootrom_io <= '1' when (bus_addr_i(31 downto 16) = boot_rom_base_c(31 downto 16)) else '0'; -- hacky!
  -- actual internal bus access? --
  access_check.valid <= access_check.int_imem or access_check.int_dmem or access_check.int_bootrom_io;


  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rw_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- bus handshake --
      ack_o <= wren or rden;

      -- read access --
      data_o <= (others => '0');
      if (rden = '1') then
        data_o(ctrl_err_type_c) <= control.err_type;
        data_o(ctrl_err_src_c)  <= control.int_ext;
        data_o(ctrl_err_flag_c) <= err_flag;
      end if;
      --
      if (control.bus_err = '1') then
        err_flag <= '1'; -- sticky error flag
      elsif (rden = '1') then -- clear on read
        err_flag <= '0';
      end if;
    end if;
  end process rw_access;


  -- Keeper ---------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  keeper_control: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      control.pending  <= '0';
      control.bus_err  <= '0';
      control.err_type <= def_rst_val_c;
      control.int_ext  <= def_rst_val_c;
      control.timeout  <= (others => def_rst_val_c);
      err_o            <= '0';
    elsif rising_edge(clk_i) then
      -- defaults --
      control.bus_err <= '0';

      -- access monitor: IDLE --
      if (control.pending = '0') then
        control.timeout <= std_ulogic_vector(to_unsigned(max_proc_int_response_time_c, index_size_f(max_proc_int_response_time_c)));
        if (bus_rden_i = '1') or (bus_wren_i = '1') then
          if (access_check.valid = '1') or (MEM_EXT_EN = false) then
            control.int_ext <= '1'; -- processor-internal access
          else
            control.int_ext <= '0'; -- processor-external access
          end if;
          control.pending <= '1';
        end if;

      -- access monitor: PENDING --
      else
        control.timeout <= std_ulogic_vector(unsigned(control.timeout) - 1); -- countdown timer
        if (bus_ack_i = '1') then -- normal termination by bus system
          control.err_type <= '0'; -- don't care
          control.bus_err  <= '0';
          control.pending  <= '0';
        elsif (bus_err_i = '1') then -- error termination by bus system
          control.err_type <= '0'; -- device error
          control.bus_err  <= '1';
          control.pending  <= '0';
        elsif (or_reduce_f(control.timeout) = '0') and (control.int_ext = '1') then -- timeout! terminate bus transfer (internal accesses only!)
          control.err_type <= '1'; -- timeout error
          control.bus_err  <= '1';
          control.pending  <= '0';
        end if;
      end if;

    -- only output timeout errors here - device errors are already propagated by the bus system --
    err_o <= control.bus_err and control.err_type;
    end if;
  end process keeper_control;


end neorv32_bus_keeper_rtl;
