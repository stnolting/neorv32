-- #################################################################################################
-- # << NEORV32 - External Bus Interface (WISHBONE) >>                                             #
-- # ********************************************************************************************* #
-- # The interface provides registers for all outgoing signals. If the host cancels a running      #
-- # transfer, the Wishbone arbiter still waits some time for the bus system to ACK to transfer.   #
-- # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
-- # All bus accesses from the CPU, which do not target the internal IO region, the internal boot- #
-- # loader or the internal instruction or data memories (if implemented), are delegated via this  #
-- # Wishbone gateway to the external bus interface.                                               #
-- # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
-- # This interface supports classic/standard Wishbone transactions (WB_PIPELINED_MODE = false)    #
-- # and also pipelined transactions (WB_PIPELINED_MODE = true).                                   #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2020, Stephan Nolting. All rights reserved.                                     #
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

entity neorv32_wishbone is
  generic (
    WB_PIPELINED_MODE : boolean := false; -- false: classic/standard wishbone mode, true: pipelined wishbone mode
    -- Internal instruction memory --
    MEM_INT_IMEM_USE  : boolean := true;   -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE : natural := 8*1024; -- size of processor-internal instruction memory in bytes
    -- Internal data memory --
    MEM_INT_DMEM_USE  : boolean := true;   -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE : natural := 4*1024  -- size of processor-internal data memory in bytes
  );
  port (
    -- global control --
    clk_i    : in  std_ulogic; -- global clock line
    rstn_i   : in  std_ulogic; -- global reset line, low-active
    -- host access --
    addr_i   : in  std_ulogic_vector(31 downto 0); -- address
    rden_i   : in  std_ulogic; -- read enable
    wren_i   : in  std_ulogic; -- write enable
    ben_i    : in  std_ulogic_vector(03 downto 0); -- byte write enable
    data_i   : in  std_ulogic_vector(31 downto 0); -- data in
    data_o   : out std_ulogic_vector(31 downto 0); -- data out
    cancel_i : in  std_ulogic; -- cancel current bus transaction
    ack_o    : out std_ulogic; -- transfer acknowledge
    err_o    : out std_ulogic; -- transfer error
    -- wishbone interface --
    wb_adr_o : out std_ulogic_vector(31 downto 0); -- address
    wb_dat_i : in  std_ulogic_vector(31 downto 0); -- read data
    wb_dat_o : out std_ulogic_vector(31 downto 0); -- write data
    wb_we_o  : out std_ulogic; -- read/write
    wb_sel_o : out std_ulogic_vector(03 downto 0); -- byte enable
    wb_stb_o : out std_ulogic; -- strobe
    wb_cyc_o : out std_ulogic; -- valid cycle
    wb_ack_i : in  std_ulogic; -- transfer acknowledge
    wb_err_i : in  std_ulogic  -- transfer error
  );
end neorv32_wishbone;

architecture neorv32_wishbone_rtl of neorv32_wishbone is

  -- constants --
  constant wb_timeout_c : natural := bus_timeout_c/2;

  -- access control --
  signal int_imem_acc, int_imem_acc_real : std_ulogic;
  signal int_dmem_acc, int_dmem_acc_real : std_ulogic;
  signal int_boot_acc                    : std_ulogic;
  signal wb_access                       : std_ulogic;

  -- bus arbiter
  type ctrl_state_t is (IDLE, BUSY, CANCELED);
  type ctrl_t is record
    state      : ctrl_state_t;
    state_prev : ctrl_state_t;
    we         : std_ulogic;
    rd_req     : std_ulogic;
    wr_req     : std_ulogic;
    adr        : std_ulogic_vector(31 downto 0);
    wdat       : std_ulogic_vector(31 downto 0);
    rdat       : std_ulogic_vector(31 downto 0);
    sel        : std_ulogic_vector(3 downto 0);
    ack        : std_ulogic;
    err        : std_ulogic;
    timeout    : std_ulogic_vector(index_size_f(wb_timeout_c)-1 downto 0);
  end record;
  signal ctrl : ctrl_t;

  signal stb_int, cyc_int : std_ulogic;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (bus_timeout_c <= 15) report "NEORV32 PROCESSOR CONFIG ERROR: Bus timeout (bus_timeout_c) should be >16 for interfacing external modules." severity error;


  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- access to internal IMEM or DMEM? --
  int_imem_acc <= '1' when (addr_i(31 downto index_size_f(MEM_INT_IMEM_SIZE)) = imem_base_c(31 downto index_size_f(MEM_INT_IMEM_SIZE))) else '0';
  int_dmem_acc <= '1' when (addr_i(31 downto index_size_f(MEM_INT_DMEM_SIZE)) = dmem_base_c(31 downto index_size_f(MEM_INT_DMEM_SIZE))) else '0';
  int_imem_acc_real <= int_imem_acc when (MEM_INT_IMEM_USE = true) else '0';
  int_dmem_acc_real <= int_dmem_acc when (MEM_INT_DMEM_USE = true) else '0';

  -- access to internal BOOTROM or IO devices? --
  int_boot_acc <= '1' when (addr_i >= boot_rom_base_c) else '0'; -- this also covers access to the IO space
--int_boot_acc <= '1' when (addr_i(31 downto index_size_f(2*boot_rom_max_size_c)) = boot_rom_base_c(31 downto index_size_f(2*boot_rom_max_size_c))) else '0'; -- this also covers access to the IO space
--int_io_acc   <= '1' when (addr_i >= io_base_c) else '0';

  -- actual external bus access? --
  wb_access <= (not int_imem_acc_real) and (not int_dmem_acc_real) and (not int_boot_acc);

  -- Bus Arbiter -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl.state      <= IDLE;
      ctrl.state_prev <= IDLE;
      ctrl.we         <= '0';
      ctrl.rd_req     <= '0';
      ctrl.wr_req     <= '0';
      ctrl.adr        <= (others => '0');
      ctrl.wdat       <= (others => '0');
      ctrl.rdat       <= (others => '0');
      ctrl.sel        <= (others => '0');
      ctrl.timeout    <= (others => '0');
      ctrl.ack        <= '0';
      ctrl.err        <= '0';
    elsif rising_edge(clk_i) then
      -- defaults --
      ctrl.state_prev <= ctrl.state;
      ctrl.rdat       <= (others => '0');
      ctrl.ack        <= '0';
      ctrl.err        <= '0';
      ctrl.timeout    <= std_ulogic_vector(to_unsigned(wb_timeout_c, index_size_f(wb_timeout_c)));

      -- state machine --
      case ctrl.state is

        when IDLE => -- waiting for host request
        -- ------------------------------------------------------------
          ctrl.rd_req <= '0';
          ctrl.wr_req <= '0';
          -- buffer all outgoing signals --
          ctrl.we   <= wren_i;
          ctrl.adr  <= addr_i;
          ctrl.wdat <= data_i;
          ctrl.sel  <= ben_i;
          -- valid read/write access --
          if ((wb_access and (wren_i or ctrl.wr_req or rden_i or ctrl.rd_req)) = '1') then
            ctrl.state <= BUSY;
          end if;

        when BUSY => -- transfer in progress
        -- ------------------------------------------------------------
          ctrl.rdat <= wb_dat_i;
          if (cancel_i = '1') then -- transfer canceled by host
            ctrl.state <= CANCELED;
          elsif (wb_err_i = '1') then -- abnormal bus termination
            ctrl.err   <= '1';
            ctrl.state <= CANCELED;
          elsif (wb_ack_i = '1') then -- normal bus termination
            ctrl.ack   <= '1';
            ctrl.state <= IDLE;
          end if;

        when CANCELED => -- 
        -- ------------------------------------------------------------
          ctrl.wr_req <= ctrl.wr_req or wren_i; -- buffer new request
          ctrl.rd_req <= ctrl.rd_req or rden_i; -- buffer new request
          -- wait for bus.peripheral to ACK transfer (as "aborted" but still somehow "completed")
          -- or wait for a timeout and force termination
          ctrl.timeout <= std_ulogic_vector(unsigned(ctrl.timeout) - 1); -- timeout counter
          if (wb_ack_i = '1') or (or_all_f(ctrl.timeout) = '0') then
            ctrl.state <= IDLE;
          end if;

        when others => -- undefined
        -- ------------------------------------------------------------
          ctrl.state <= IDLE;

      end case;
    end if;
  end process bus_arbiter;


  -- host access --
  data_o   <= ctrl.rdat;
  ack_o    <= ctrl.ack;
  err_o    <= ctrl.err;

  -- wishbone interface --
  wb_adr_o <= ctrl.adr;
  wb_dat_o <= ctrl.wdat;
  wb_we_o  <= ctrl.we;
  wb_sel_o <= ctrl.sel;
  wb_stb_o <= stb_int when (WB_PIPELINED_MODE = true) else cyc_int;
  wb_cyc_o <= cyc_int;

  stb_int  <= '1' when ((ctrl.state = BUSY) and (ctrl.state_prev = IDLE)) else '0';
  cyc_int  <= '0' when (ctrl.state = IDLE) else '1';


end neorv32_wishbone_rtl;
