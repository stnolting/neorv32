-- #################################################################################################
-- # << NEORV32 - External Bus Interface (WISHBONE) >>                                             #
-- # ********************************************************************************************* #
-- # The interface is either unregistered (INTERFACE_REG_STAGES = 0), only outgoing signals are    #
-- # registered (INTERFACE_REG_STAGES = 1) or incoming and outgoing signals are registered         #
-- # (INTERFACE_REG_STAGES = 2).                                                                   #
-- # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
-- # All bus accesses from the CPU, which do not target the internal IO region, the internal boot- #
-- # loader or the internal instruction or data memories (if implemented), are delegated via this  #
-- # Wishbone gateway to the external bus interface.                                               #
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
    INTERFACE_REG_STAGES : natural := 2; -- number of interface register stages (0,1,2)
    -- Memory configuration: Instruction memory --
    MEM_ISPACE_BASE      : std_ulogic_vector(31 downto 0) := x"00000000"; -- base address of instruction memory space
    MEM_ISPACE_SIZE      : natural := 8*1024; -- total size of instruction memory space in byte
    MEM_INT_IMEM_USE     : boolean := true;   -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE    : natural := 8*1024; -- size of processor-internal instruction memory in bytes
    -- Memory configuration: Data memory --
    MEM_DSPACE_BASE      : std_ulogic_vector(31 downto 0) := x"80000000"; -- base address of data memory space
    MEM_DSPACE_SIZE      : natural := 4*1024; -- total size of data memory space in byte
    MEM_INT_DMEM_USE     : boolean := true;   -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE    : natural := 4*1024  -- size of processor-internal data memory in bytes
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

  -- access control --
  signal int_imem_acc, int_imem_acc_real : std_ulogic;
  signal int_dmem_acc, int_dmem_acc_real : std_ulogic;
  signal int_boot_acc, int_io_acc        : std_ulogic;
  signal wb_access                       : std_ulogic;
  signal wb_access_ff, wb_access_ff_ff   : std_ulogic;
  signal rb_en                           : std_ulogic;

  -- bus arbiter --
  signal wb_stb_ff0 : std_ulogic;
  signal wb_stb_ff1 : std_ulogic;
  signal wb_cyc_ff  : std_ulogic;
  signal wb_ack_ff  : std_ulogic;
  signal wb_err_ff  : std_ulogic;

  -- data read-back --
  signal wb_rdata : std_ulogic_vector(31 downto 0);

begin

  -- Sanity Check ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  sanity_check: process(rstn_i)
  begin
    if rising_edge(rstn_i) then -- no worries - this won't be synthesized
      if (INTERFACE_REG_STAGES > 2) then
        assert false report "NEORV32 CONFIG ERROR! Number of external memory interface buffer stages must be 0, 1 or 2." severity error;
      end if;
      if (INTERFACE_REG_STAGES = 0) then
        assert false report "NEORV32 CONFIG WARNING! External memory interface without register stages is still experimental for peripherals with more than 1 cycle latency." severity warning;
      end if;
    end if;
  end process sanity_check;


  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- access to internal IMEM or DMEM? --
  int_imem_acc <= '1' when (addr_i >= MEM_ISPACE_BASE) and (addr_i < std_ulogic_vector(unsigned(MEM_ISPACE_BASE) + MEM_INT_IMEM_SIZE)) else '0';
  int_dmem_acc <= '1' when (addr_i >= MEM_DSPACE_BASE) and (addr_i < std_ulogic_vector(unsigned(MEM_DSPACE_BASE) + MEM_INT_DMEM_SIZE)) else '0';
  int_imem_acc_real <= int_imem_acc when (MEM_INT_IMEM_USE = true) else '0';
  int_dmem_acc_real <= int_dmem_acc when (MEM_INT_DMEM_USE = true) else '0';
  int_boot_acc <= '1' when (addr_i >= boot_base_c) else '0';
  int_io_acc   <= '1' when (addr_i >= io_base_c)   else '0';

  -- actual external bus access? --
  wb_access <= (not int_imem_acc_real) and (not int_dmem_acc_real) and (not int_boot_acc) and (not int_io_acc) and (wren_i or rden_i);


  -- Bus Arbiter -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      wb_cyc_ff       <= '0';
      wb_stb_ff1      <= '0';
      wb_stb_ff0      <= '0';
      wb_ack_ff       <= '0';
      wb_err_ff       <= '0';
      wb_access_ff    <= '0';
      wb_access_ff_ff <= '0';
    elsif rising_edge(clk_i) then
      -- bus cycle --
      if (INTERFACE_REG_STAGES = 0) then
        wb_cyc_ff <= '0'; -- unused
      else
        wb_cyc_ff <= (wb_cyc_ff or wb_access) and (not wb_ack_i) and (not wb_err_i) and (not cancel_i);
      end if;
      -- bus strobe --
      wb_stb_ff1 <= wb_stb_ff0;
      wb_stb_ff0 <= wb_access;
      -- bus ack --
      wb_ack_ff <= wb_ack_i;
      -- bus err --
      wb_err_ff <= wb_err_i;
      -- access still active? --
      wb_access_ff_ff <= wb_access_ff;
      if (wb_access = '1') then
        wb_access_ff <= '1';
      elsif ((wb_ack_i or wb_err_i or cancel_i) = '1') then
        wb_access_ff <= '0';
      end if;
    end if;
  end process bus_arbiter;

  -- bus cycle --
  wb_cyc_o <= wb_access when (INTERFACE_REG_STAGES = 0) else wb_cyc_ff;

  -- bus_strobe: rising edge detector --
  wb_stb_o <= (wb_access and (not wb_stb_ff0)) when (INTERFACE_REG_STAGES = 0) else (wb_stb_ff0 and (not wb_stb_ff1));

  -- cpu ack --
  ack_o <= wb_ack_ff when (INTERFACE_REG_STAGES = 2) else wb_ack_i;

  -- cpu err --
  err_o <= wb_err_ff when (INTERFACE_REG_STAGES = 2) else wb_err_i;

  -- cpu read-data --
  rb_en  <= wb_access_ff_ff when (INTERFACE_REG_STAGES = 2) else wb_access_ff;
  data_o <= wb_rdata when (rb_en = '1') else (others => '0');


  -- Bus Buffer -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  interface_reg_level_zero:
  if (INTERFACE_REG_STAGES = 0) generate -- 0 register levels: direct connection
    wb_rdata <= wb_dat_i;
    wb_adr_o <= addr_i;
    wb_dat_o <= data_i;
    wb_sel_o <= ben_i;
    wb_we_o  <= wren_i;
  end generate;

  interface_reg_level_one:
  if (INTERFACE_REG_STAGES = 1) generate -- 1 register levels: buffer outgoing signals
    buffer_stages_one: process(clk_i)
    begin
      if rising_edge(clk_i) then
        if (wb_cyc_ff = '0') then
          wb_adr_o <= addr_i;
          wb_dat_o <= data_i;
          wb_sel_o <= ben_i;
          wb_we_o  <= wren_i;
        end if;
      end if;
    end process buffer_stages_one;
    wb_rdata <= wb_dat_i;
  end generate;

  interface_reg_level_two:
  if (INTERFACE_REG_STAGES = 2) generate -- 2 register levels: buffer incoming and outgoing signals
    buffer_stages_two: process(clk_i)
    begin
      if rising_edge(clk_i) then
        if (wb_cyc_ff = '0') then
          wb_adr_o <= addr_i;
          wb_dat_o <= data_i;
          wb_sel_o <= ben_i;
          wb_we_o  <= wren_i;
        end if;
        if (wb_ack_i = '1') then
          wb_rdata <= wb_dat_i;
        end if;
      end if;
    end process buffer_stages_two;
  end generate;


end neorv32_wishbone_rtl;
