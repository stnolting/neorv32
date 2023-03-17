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
    -- host access --
    clk_i      : in  std_ulogic; -- global clock line
    rstn_i     : in  std_ulogic; -- global reset, low-active, async
    addr_i     : in  std_ulogic_vector(31 downto 0); -- address
    rden_i     : in  std_ulogic; -- read enable
    wren_i     : in  std_ulogic; -- write enable
    data_i     : in  std_ulogic_vector(31 downto 0); -- data in
    data_o     : out std_ulogic_vector(31 downto 0); -- data out
    ack_o      : out std_ulogic; -- transfer acknowledge
    err_o      : out std_ulogic; -- transfer error
    -- bus monitoring --
    bus_addr_i : in  std_ulogic_vector(31 downto 0); -- address
    bus_rden_i : in  std_ulogic; -- read enable
    bus_wren_i : in  std_ulogic; -- write enable
    bus_ack_i  : in  std_ulogic; -- transfer acknowledge from bus system
    bus_err_i  : in  std_ulogic; -- transfer error from bus system
    bus_tmo_i  : in  std_ulogic; -- transfer timeout (external interface)
    bus_ext_i  : in  std_ulogic; -- external bus access
    bus_xip_i  : in  std_ulogic  -- pending XIP access
  );
end neorv32_bus_keeper;

architecture neorv32_bus_keeper_rtl of neorv32_bus_keeper is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(buskeeper_size_c); -- low address boundary bit

  -- Control register --
  constant ctrl_err_type_c : natural :=  0; -- r/-: error type: 0=device error, 1=access timeout
  constant ctrl_err_flag_c : natural := 31; -- r/c: bus error encountered, sticky

  -- error codes --
  constant err_device_c  : std_ulogic := '0'; -- device access error
  constant err_timeout_c : std_ulogic := '1'; -- timeout error

  -- sticky error flags --
  signal err_flag : std_ulogic;
  signal err_type : std_ulogic;

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- timeout counter size --
  constant cnt_width_c : natural := index_size_f(max_proc_int_response_time_c);

  -- controller --
  type control_t is record
    pending  : std_ulogic;
    timeout  : std_ulogic_vector(cnt_width_c-1 downto 0);
    err_type : std_ulogic;
    bus_err  : std_ulogic;
    ignore   : std_ulogic;
    expired  : std_ulogic;
  end record;
  signal control : control_t;

begin

  -- Sanity Check --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (max_proc_int_response_time_c < 2)
    report "NEORV32 PROCESSOR CONFIG ERROR! Processor-internal bus timeout <max_proc_int_response_time_c> has to >= 2." severity error;


  -- Host Access ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- access control --
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = buskeeper_base_c(hi_abb_c downto lo_abb_c)) else '0';
  wren   <= acc_en and wren_i;
  rden   <= acc_en and rden_i;

  -- write access --
  write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      err_flag <= '0';
      err_type <= '0';
    elsif rising_edge(clk_i) then
      if (control.bus_err = '1') then -- sticky error flag
        err_flag <= '1';
        err_type <= control.err_type;
      elsif (wren = '1') or (rden = '1') then -- clear on read or write access
        err_flag <= '0';
      end if;
    end if;
  end process write_access;

  -- read access --
  read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      ack_o  <= wren or rden; -- bus handshake
      data_o <= (others => '0');
      if (rden = '1') then
        data_o(ctrl_err_type_c) <= err_type;
        data_o(ctrl_err_flag_c) <= err_flag;
      end if;
    end if;
  end process read_access;


  -- Monitor --------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  keeper_control: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      control.pending  <= '0';
      control.bus_err  <= '0';
      control.err_type <= '0';
      control.timeout  <= (others => '0');
      control.ignore   <= '0';
    elsif rising_edge(clk_i) then
      -- defaults --
      control.bus_err <= '0';

      -- IDLE --
      if (control.pending = '0') then
        control.timeout <= std_ulogic_vector(to_unsigned(max_proc_int_response_time_c-1, cnt_width_c));
        control.ignore  <= '0';
        if (bus_rden_i = '1') or (bus_wren_i = '1') then
          control.pending <= '1';
        end if;
      -- PENDING --
      else
        -- countdown timer --
        if (control.expired = '0') then
          control.timeout <= std_ulogic_vector(unsigned(control.timeout) - 1);
        end if;
        -- bus keeper shall ignore internal timeout during this access (because it's "external") --
        control.ignore <= control.ignore or (bus_ext_i or bus_xip_i);
        -- response handling --
        if (bus_err_i = '1') then -- error termination by bus system
          control.err_type <= err_device_c; -- device error
          control.bus_err  <= '1';
          control.pending  <= '0';
        elsif ((control.expired = '1') and (control.ignore = '0')) or -- valid INTERNAL access timeout
              (bus_tmo_i = '1') then -- EXTERNAL access timeout
          control.err_type <= err_timeout_c; -- timeout error
          control.bus_err  <= '1';
          control.pending  <= '0';
        elsif (bus_ack_i = '1') then -- normal termination by bus system
          control.err_type <= '0'; -- don't care
          control.bus_err  <= '0';
          control.pending  <= '0';
        end if;
      end if;
    end if;
  end process keeper_control;

  -- timeout counter expired? --
  control.expired <= '1' when (or_reduce_f(control.timeout) = '0') else '0';

  -- signal bus error to CPU --
  err_o <= control.bus_err;


end neorv32_bus_keeper_rtl;
