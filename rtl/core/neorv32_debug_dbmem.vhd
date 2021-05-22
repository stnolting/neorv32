-- #################################################################################################
-- # << NEORV32 - On-Chip Debugger - Debug Memory (DBMEM) >>                                       #
-- # ********************************************************************************************* #
-- # This unit contains:                                                                           #
-- # * code ROM for the "park loop" code                                                           #
-- # * memory-mapped registers for communicating with the DM                                       #
-- #   * data buffer for accessing DM.data0                                                        #
-- #   * program buffer for accessing progbuf0/1 (+ virtual load/store entry & ebreak entry)       #
-- #   * status and control register to check for REQs from DM and to send ACKs to DM              #
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

entity neorv32_debug_dbmem is
  port (
    -- global control --
    clk_i               : in  std_ulogic; -- global clock line
    -- CPU bus access --
    bus_addr_i          : in  std_ulogic_vector(31 downto 0); -- address
    bus_rden_i          : in  std_ulogic; -- read enable
    bus_wren_i          : in  std_ulogic; -- write enable
    bus_data_i          : in  std_ulogic_vector(31 downto 0); -- data in
    bus_data_o          : out std_ulogic_vector(31 downto 0); -- data out
    bus_ack_o           : out std_ulogic; -- transfer acknowledge
    -- Debug core interface --
    dci_halt_ack_o      : out std_ulogic; -- CPU (re-)entered HALT state (single-shot)
    dci_resume_req_i    : in  std_ulogic; -- DM wants the CPU to resume when set
    dci_resume_ack_o    : out std_ulogic; -- CPU starts resuming when set (single-shot)
    dci_execute_req_i   : in  std_ulogic; -- DM wants CPU to execute program buffer when set
    dci_execute_ack_o   : out std_ulogic; -- CPU starts executing program buffer when set (single-shot)
    dci_exception_ack_o : out std_ulogic; -- CPU has detected an exception (single-shot)
    dci_progbuf_i       : in  std_ulogic_vector(255 downto 0); -- program buffer, 4 32-bit entries
    dci_data_we_i       : in  std_ulogic; -- write abstract data
    dci_data_i          : in  std_ulogic_vector(31 downto 0); -- abstract write data
    dci_data_o          : out std_ulogic_vector(31 downto 0)  -- abstract read data
  );
end neorv32_debug_dbmem;

architecture neorv32_debug_dbmem_rtl of neorv32_debug_dbmem is

  -- debug code ROM for "park loop" --
  type code_rom_file_t is array (0 to 31) of std_ulogic_vector(31 downto 0);
  constant code_rom_file : code_rom_file_t := (
    00000000 => x"0180006f",
    00000001 => x"7b241073",
    00000002 => x"02000413",
    00000003 => x"98802023",
    00000004 => x"7b202473",
    00000005 => x"00100073",
    00000006 => x"7b241073",
    00000007 => x"00100413",
    00000008 => x"98802023",
    00000009 => x"98002403",
    00000010 => x"00847413",
    00000011 => x"02041263",
    00000012 => x"98002403",
    00000013 => x"00247413",
    00000014 => x"00041463",
    00000015 => x"fe9ff06f",
    00000016 => x"00400413",
    00000017 => x"98802023",
    00000018 => x"7b202473",
    00000019 => x"7b200073",
    00000020 => x"01000413",
    00000021 => x"98802023",
    00000022 => x"7b202473",
    00000023 => x"88000067",
    others   => x"00100073"  -- ebreak
  );

  -- IO space: module base address --
  constant hi_abb_c : natural := 31; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(dbmem_size_c); -- low address boundary bit

  -- global access control --
  signal acc_en : std_ulogic;
  signal rden   : std_ulogic;
  signal wren   : std_ulogic;

  -- debug code ROM --
  type code_rom_t is record
    acc   : std_ulogic;
    rden  : std_ulogic;
    addr  : std_ulogic_vector(04 downto 0);
    rdata : std_ulogic_vector(31 downto 0);
  end record;
  signal code_rom : code_rom_t;

  -- program buffer --
  type prom_t is array (0 to 3) of std_ulogic_vector(31 downto 0);
  type prog_buf_t is record
    acc   : std_ulogic;
    rden  : std_ulogic;
    addr  : std_ulogic_vector(01 downto 0);
    rdata : std_ulogic_vector(31 downto 0);
    prom  : prom_t;
  end record;
  signal prog_buf : prog_buf_t;

  -- data buffer --
  type data_buf_t is record
    acc  : std_ulogic;
    mem  : std_ulogic_vector(31 downto 0);
    rden : std_ulogic;
    wren : std_ulogic;
  end record;
  signal data_buf : data_buf_t;

  -- status register --
  type sreg_t is record
    acc   : std_ulogic;
    rden  : std_ulogic;
    wren  : std_ulogic;
    rdata : std_ulogic_vector(31 downto 0);
  end record;
  signal sreg : sreg_t;

  constant sreg_halt_ack_c      : natural := 0; -- -/w: CPU is halted in debug mode and waits in park loop
  constant sreg_resume_req_c    : natural := 1; -- r/-: DM requests CPU to resume
  constant sreg_resume_ack_c    : natural := 2; -- -/w: CPU starts resuming
  constant sreg_execute_req_c   : natural := 3; -- r/-: DM requests to execute program buffer
  constant sreg_execute_ack_c   : natural := 4; -- -/w: CPU starts to execute program buffer
  constant sreg_exception_ack_c : natural := 5; -- -/w: CPU has detected an exception

begin

  -- Access Control ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (bus_addr_i(hi_abb_c downto lo_abb_c) = dbmem_base_c(hi_abb_c downto lo_abb_c)) else '0';
  rden   <= acc_en and bus_rden_i;
  wren   <= acc_en and bus_wren_i;

  -- code rom --
  code_rom.acc  <= '1' when (bus_addr_i(lo_abb_c-1 downto lo_abb_c-2) = dbmem_code_base_c(lo_abb_c-1 downto lo_abb_c-2)) else '0';
  code_rom.addr <= bus_addr_i(6 downto 2); -- word aligned

  -- program buffer --
  prog_buf.acc  <= '1' when (bus_addr_i(lo_abb_c-1 downto lo_abb_c-2) = dbmem_pbuf_base_c(lo_abb_c-1 downto lo_abb_c-2)) else '0';
  prog_buf.addr <= bus_addr_i(3 downto 2); -- word aligned

  -- data buffer --
  data_buf.acc <= '1' when (bus_addr_i(lo_abb_c-1 downto lo_abb_c-2) = dbmem_data_base_c(lo_abb_c-1 downto lo_abb_c-2)) else '0';

  -- status register --
  sreg.acc <= '1' when (bus_addr_i(lo_abb_c-1 downto lo_abb_c-2) = dbmem_sreg_base_c(lo_abb_c-1 downto lo_abb_c-2)) else '0';


  -- Debug Code ROM -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rom_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      code_rom.rden <= code_rom.acc and rden;
      if ((code_rom.acc and rden) = '1') then
        code_rom.rdata <= code_rom_file(to_integer(unsigned(code_rom.addr)));
      end if;
    end if;
  end process rom_access;


  -- Program Buffer -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  progbuf_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      prog_buf.rden <= prog_buf.acc and rden;
      if ((prog_buf.acc and rden) = '1') then
        prog_buf.rdata <= prog_buf.prom(to_integer(unsigned(prog_buf.addr)));
      end if;
    end if;
  end process progbuf_access;

  -- build ROM array --
  create_prog_mem:
  for i in 0 to 3 generate
    prog_buf.prom(i) <= dci_progbuf_i((i*32+32)-1 downto i*32);
  end generate; -- i


  -- Data Access ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  data_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (dci_data_we_i = '1') then -- DM write access
        data_buf.mem <= dci_data_i;
      elsif (data_buf.acc = '1') and (wren = '1') then -- BUS write access
        data_buf.mem <= bus_data_i;
      end if;
    end if;
  end process data_access;

  data_buf.rden <= data_buf.acc and rden;
  data_buf.wren <= data_buf.acc and wren;

  -- DM read access --
  dci_data_o <= data_buf.mem;


  -- Status/Control Register ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  sreg_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- bus write access - all flags auto clear --
      dci_halt_ack_o      <= '0';
      dci_resume_ack_o    <= '0';
      dci_execute_ack_o   <= '0';
      dci_exception_ack_o <= '0';
      if (sreg.acc = '1') and (wren = '1') then
        dci_halt_ack_o      <= bus_data_i(sreg_halt_ack_c);
        dci_resume_ack_o    <= bus_data_i(sreg_resume_ack_c);
        dci_execute_ack_o   <= bus_data_i(sreg_execute_ack_c);
        dci_exception_ack_o <= bus_data_i(sreg_exception_ack_c);
      end if;
    end if;
  end process sreg_access;

  sreg.rden <= sreg.acc and rden;
  sreg.wren <= sreg.acc and wren;

  -- bus read access --
  sreg_rdback: process(dci_resume_req_i, dci_execute_req_i)
  begin
    sreg.rdata <= (others => '0');
    sreg.rdata(sreg_resume_req_c)  <= dci_resume_req_i;
    sreg.rdata(sreg_execute_req_c) <= dci_execute_req_i;
  end process;


  -- Output to CPU Bus ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_feedback: process(clk_i)
  begin
    if rising_edge(clk_i) then
      bus_ack_o <= code_rom.rden or prog_buf.rden or (data_buf.rden or data_buf.wren) or (sreg.rden or sreg.wren);
      -- read data select & output gate --
      if (code_rom.rden = '1') then
        bus_data_o <= code_rom.rdata;
      elsif (prog_buf.rden = '1') then
        bus_data_o <= prog_buf.rdata;
      elsif (data_buf.rden = '1') then
        bus_data_o <= data_buf.mem;
      elsif (sreg.rden = '1') then
        bus_data_o <= sreg.rdata;
      else
        bus_data_o <= (others => '0');
      end if;
    end if;
  end process bus_feedback;


end neorv32_debug_dbmem_rtl;
