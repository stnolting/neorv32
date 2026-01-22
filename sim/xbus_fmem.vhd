-- ================================================================================ --
-- NEORV32 - XBUS / Wishbone fine-grained memory error test module                  --
-- -------------------------------------------------------------------------------- --
-- This is a generic RAM that can be accessed by the mem_req port. Each memory word --
-- provides an additional TAG (accessible by the tag_req port). When set, a bus     --
-- error will be generated when accessing the according memory word via mem_req.    --
-- This module is used to simulate unaligned bus access errors.                     --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity xbus_fmem is
  generic (
    MEM_SIZE : natural := 4 -- memory size in bytes; min 4; should be a power of two
  );
  port (
    clk_i     : in  std_ulogic;
    rstn_i    : in  std_ulogic;
    tag_req_i : in  xbus_req_t;
    tag_rsp_o : out xbus_rsp_t;
    mem_req_i : in  xbus_req_t;
    mem_rsp_o : out xbus_rsp_t
  );
end xbus_fmem;

architecture xbus_fmem_rtl of xbus_fmem is

  -- address width --
  constant awidth_c : natural := index_size_f(MEM_SIZE/4);

  -- data memory --
  type data_mem_t is array ((2**awidth_c)-1 downto 0) of std_ulogic_vector(7 downto 0);
  signal data_mem_0, data_mem_1, data_mem_2, data_mem_3 : data_mem_t;
  signal data_req : xbus_req_t;
  signal data_mem_en : std_ulogic_vector(3 downto 0);
  signal data_mem_rd : std_ulogic_vector(31 downto 0);

  -- tag memory --
  signal tag_mem : std_ulogic_vector((2**awidth_c)-1 downto 0);
  signal tag_req : xbus_req_t;
  signal tag_mem_en, tag_mem_rd, tag_err : std_ulogic;

begin

  -- Tag Memory -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tag_memory: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- port A: read/write --
      if (tag_mem_en = '1') then
        if (tag_req_i.we = '1') then
          tag_mem(to_integer(unsigned(tag_req_i.addr(awidth_c+1 downto 2)))) <= tag_req_i.data(0);
        else
          tag_mem_rd <= tag_mem(to_integer(unsigned(tag_req_i.addr(awidth_c+1 downto 2))));
        end if;
      end if;
      -- port B: read-only --
      if (mem_req_i.cyc = '1') then
        tag_err <= tag_mem(to_integer(unsigned(mem_req_i.addr(awidth_c+1 downto 2))));
      end if;
    end if;
  end process tag_memory;

  -- access enable --
  tag_mem_en <= tag_req_i.sel(0) when (tag_req_i.cyc = '1') and (tag_req_i.stb = '1') else '0';

  -- bus handshake --
  tag_mem_handshake: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      tag_req   <= xbus_req_terminate_c;
      tag_rsp_o <= xbus_rsp_terminate_c;
    elsif rising_edge(clk_i) then
      tag_req   <= tag_req_i; -- delay by one cycle to compensate RAM (read) access latency
      tag_rsp_o <= xbus_rsp_terminate_c;
      if (tag_req.cyc = '1') and (tag_req.stb = '1') then
        if (mem_req_i.we = '1') then -- write
          tag_rsp_o.data <= (others => '0');
        else -- read
          tag_rsp_o.data <= x"0000000" & "000" & tag_mem_rd;
        end if;
        tag_rsp_o.ack  <= '1';
        tag_rsp_o.err  <= '0';
      end if;
    end if;
  end process tag_mem_handshake;


  -- Data Memory ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  data_memory: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- byte 0 --
      if (data_mem_en(0) = '1') then
        if (mem_req_i.we = '1') then
          data_mem_0(to_integer(unsigned(mem_req_i.addr(awidth_c+1 downto 2)))) <= mem_req_i.data(7 downto 0);
        else
          data_mem_rd(7 downto 0) <= data_mem_0(to_integer(unsigned(mem_req_i.addr(awidth_c+1 downto 2))));
        end if;
      end if;
      -- byte 1 --
      if (data_mem_en(1) = '1') then
        if (mem_req_i.we = '1') then
          data_mem_1(to_integer(unsigned(mem_req_i.addr(awidth_c+1 downto 2)))) <= mem_req_i.data(15 downto 8);
        else
          data_mem_rd(15 downto 8) <= data_mem_1(to_integer(unsigned(mem_req_i.addr(awidth_c+1 downto 2))));
        end if;
      end if;
      -- byte 2 --
      if (data_mem_en(2) = '1') then
        if (mem_req_i.we = '1') then
          data_mem_2(to_integer(unsigned(mem_req_i.addr(awidth_c+1 downto 2)))) <= mem_req_i.data(23 downto 16);
        else
          data_mem_rd(23 downto 16) <= data_mem_2(to_integer(unsigned(mem_req_i.addr(awidth_c+1 downto 2))));
        end if;
      end if;
      -- byte 3 --
      if (data_mem_en(3) = '1') then
        if (mem_req_i.we = '1') then
          data_mem_3(to_integer(unsigned(mem_req_i.addr(awidth_c+1 downto 2)))) <= mem_req_i.data(31 downto 24);
        else
          data_mem_rd(31 downto 24) <= data_mem_3(to_integer(unsigned(mem_req_i.addr(awidth_c+1 downto 2))));
        end if;
      end if;
    end if;
  end process data_memory;

  -- byte-wise access enable --
  data_mem_en <= mem_req_i.sel when (mem_req_i.cyc = '1') and (mem_req_i.stb = '1') else (others => '0');

  -- bus handshake --
  data_mem_handshake: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      data_req  <= xbus_req_terminate_c;
      mem_rsp_o <= xbus_rsp_terminate_c;
    elsif rising_edge(clk_i) then
      data_req  <= mem_req_i; -- delay by one cycle to compensate RAM (read) access latency
      mem_rsp_o <= xbus_rsp_terminate_c;
      if (data_req.cyc = '1') and (data_req.stb = '1') then
        if (data_req.we = '1') then -- write
          mem_rsp_o.data <= (others => '0');
          mem_rsp_o.ack  <= '1';
          mem_rsp_o.err  <= '0';
        else -- read
          mem_rsp_o.data <= data_mem_rd;
          mem_rsp_o.ack  <= not tag_err;
          mem_rsp_o.err  <= tag_err;
        end if;
      end if;
    end if;
  end process data_mem_handshake;

end xbus_fmem_rtl;
