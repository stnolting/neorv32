-- ================================================================================ --
-- NEORV32 - Simple XBUS / Wishbone Memory (meant for simulation only)              --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity xbus_memory is
  generic (
    MEM_SIZE : natural := 1024; -- memory size in bytes; should be a power of two
    MEM_LATE : natural := 1 -- number of latency cycles (min 1)
  );
  port (
    clk_i      : in  std_ulogic;
    rstn_i     : in  std_ulogic;
    xbus_req_i : in  xbus_req_t;
    xbus_rsp_o : out xbus_rsp_t
  );
end xbus_memory;

architecture xbus_memory_rtl of xbus_memory is

  -- memory type --
  type mem8_bv_t is array (natural range <>) of bit_vector(7 downto 0); -- bit_vector type for optimized system storage

  -- memory access --
  signal addr  : integer range 0 to (MEM_SIZE/4)-1;
  signal rdata : std_ulogic_vector(31 downto 0);
  signal ack   : std_ulogic;

  -- latency generator --
  type late_data_t is array (MEM_LATE downto 0) of std_ulogic_vector(31 downto 0);
  signal late_data : late_data_t;
  signal late_ack  : std_ulogic_vector(MEM_LATE downto 0);

begin

  -- word-aligned read/write address --
  addr <= to_integer(unsigned(xbus_req_i.addr(index_size_f(MEM_SIZE/4)+1 downto 2)));


  -- Non-Initialized Memory Core ------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  memory_core: process(clk_i)
    variable mem8_bv_b0_v, mem8_bv_b1_v, mem8_bv_b2_v, mem8_bv_b3_v : mem8_bv_t(0 to (MEM_SIZE/4)-1);
  begin
    if rising_edge(clk_i) then
      ack <= '0';
      if (xbus_req_i.cyc = '1') and (xbus_req_i.stb = '1') then
        ack <= '1';
        if (xbus_req_i.we = '1') then
          if (xbus_req_i.sel(0) = '1') then mem8_bv_b0_v(addr) := to_bitvector(xbus_req_i.data(07 downto 00)); end if;
          if (xbus_req_i.sel(1) = '1') then mem8_bv_b1_v(addr) := to_bitvector(xbus_req_i.data(15 downto 08)); end if;
          if (xbus_req_i.sel(2) = '1') then mem8_bv_b2_v(addr) := to_bitvector(xbus_req_i.data(23 downto 16)); end if;
          if (xbus_req_i.sel(3) = '1') then mem8_bv_b3_v(addr) := to_bitvector(xbus_req_i.data(31 downto 24)); end if;
        else
          rdata(07 downto 00) <= to_stdulogicvector(mem8_bv_b0_v(addr));
          rdata(15 downto 08) <= to_stdulogicvector(mem8_bv_b1_v(addr));
          rdata(23 downto 16) <= to_stdulogicvector(mem8_bv_b2_v(addr));
          rdata(31 downto 24) <= to_stdulogicvector(mem8_bv_b3_v(addr));
        end if;
      end if;
    end if;
  end process memory_core;


  -- Latency Generator ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  latency_gen: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      late_data <= (others => (others => '0'));
      late_ack  <= (others => '0');
    elsif rising_edge(clk_i) then
      late_data(0) <= rdata;
      late_ack(0)  <= ack;
      for i in 0 to MEM_LATE-1 loop
        late_data(i+1) <= late_data(i);
        late_ack(i+1)  <= late_ack(i);
      end loop;
    end if;
  end process latency_gen;

  -- bus response --
  xbus_rsp_o.data <= rdata when (MEM_LATE = 1) else late_data(MEM_LATE-1);
  xbus_rsp_o.ack  <= ack   when (MEM_LATE = 1) else late_ack(MEM_LATE-1);
  xbus_rsp_o.err  <= '0';


end xbus_memory_rtl;
