-- ================================================================================ --
-- NEORV32 SoC - Processor-Internal Data Memory (DMEM)                              --
-- -------------------------------------------------------------------------------- --
-- [TIP] This file can be replaced by a technology-specific implementation to       --
--       optimize timing, area, energy, etc.                                        --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_dmem is
  generic (
    DMEM_SIZE : natural; -- memory size in bytes, has to be a power of 2, min 4
    OUTREG_EN : boolean  -- implement output register stage
  );
  port (
    clk_i     : in  std_ulogic; -- global clock line
    rstn_i    : in  std_ulogic; -- async reset, low-active
    bus_req_i : in  bus_req_t;  -- bus request
    bus_rsp_o : out bus_rsp_t   -- bus response
  );
end neorv32_dmem;

architecture neorv32_dmem_rtl of neorv32_dmem is

  -- highest address bit --
  constant addr_hi_c : natural := index_size_f(DMEM_SIZE/4)+1;

  -- local signals --
  signal rdata : std_ulogic_vector(31 downto 0);
  signal dout  : std_ulogic_vector(31 downto 0);
  signal wack  : std_ulogic;
  signal rden  : std_ulogic_vector(1 downto 0);
  signal addr  : unsigned(index_size_f(DMEM_SIZE/4)-1 downto 0);

  -- [NOTE] The memory (RAM) is built from 4 individual byte-wide memories as some synthesis tools
  --        have issues inferring 32-bit memories with individual byte-enable signals.
  -- [NOTE] Read-during-write behavior is irrelevant
  --        as read and write accesses are mutually exclusive (ensured by bus protocol).
  signal mem_ram_b0, mem_ram_b1, mem_ram_b2, mem_ram_b3 : mem8_t(0 to DMEM_SIZE/4-1);

begin

  -- Memory Core ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (bus_req_i.stb = '1') then
        if (bus_req_i.rw = '1') then -- write access
          if (bus_req_i.ben(0) = '1') then -- byte 0
            mem_ram_b0(to_integer(addr)) <= bus_req_i.data(7 downto 0);
          end if;
          if (bus_req_i.ben(1) = '1') then -- byte 1
            mem_ram_b1(to_integer(addr)) <= bus_req_i.data(15 downto 8);
          end if;
          if (bus_req_i.ben(2) = '1') then -- byte 2
            mem_ram_b2(to_integer(addr)) <= bus_req_i.data(23 downto 16);
          end if;
          if (bus_req_i.ben(3) = '1') then -- byte 3
            mem_ram_b3(to_integer(addr)) <= bus_req_i.data(31 downto 24);
          end if;
        else -- read access
          rdata(7  downto 0)  <= mem_ram_b0(to_integer(addr));
          rdata(15 downto 8)  <= mem_ram_b1(to_integer(addr));
          rdata(23 downto 16) <= mem_ram_b2(to_integer(addr));
          rdata(31 downto 24) <= mem_ram_b3(to_integer(addr));
        end if;
      end if;
    end if;
  end process mem_access;

  -- word access address --
  addr <= unsigned(bus_req_i.addr(addr_hi_c downto 2));


  -- Bus Handshake --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_handshake: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      wack <= '0';
      rden <= (others => '0');
    elsif rising_edge(clk_i) then
      wack <= bus_req_i.stb and bus_req_i.rw;
      rden <= rden(0) & (bus_req_i.stb and (not bus_req_i.rw));
    end if;
  end process bus_handshake;


  -- Output Register Stage ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  output_register_enabled:
  if OUTREG_EN generate -- might improve FPGA mapping/timing results
    ram_outreg: process(clk_i)
    begin
      if rising_edge(clk_i) then -- no reset required due to output gate
        dout <= rdata;
      end if;
    end process ram_outreg;
    bus_rsp_o.data <= dout when (rden(1) = '1') else (others => '0'); -- output gate
    bus_rsp_o.err  <= '0'; -- no access error possible
    bus_rsp_o.ack  <= rden(1) or wack;
  end generate;

  -- no output register stage --
  output_register_disabled:
  if not OUTREG_EN generate
    dout           <= rdata;
    bus_rsp_o.data <= dout when (rden(0) = '1') else (others => '0'); -- output gate
    bus_rsp_o.err  <= '0'; -- no access error possible
    bus_rsp_o.ack  <= rden(0) or wack;
  end generate;


end neorv32_dmem_rtl;
