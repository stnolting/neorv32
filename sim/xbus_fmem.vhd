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
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
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

  -- data memory access --
  signal data_req    : xbus_req_t;
  signal data_mem_en : std_ulogic_vector(3 downto 0);
  signal data_mem_rd : std_ulogic_vector(31 downto 0);

  -- tag memory access --
  signal tag_req    : xbus_req_t;
  signal tag_mem_en : std_ulogic;
  signal tag_mem_rd : std_ulogic_vector(0 downto 0);
  signal tag_err    : std_ulogic_vector(0 downto 0);

begin

  -- Tag Memory -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tag_mem_inst: entity neorv32.neorv32_prim_sdpram
  generic map (
    AWIDTH => awidth_c,
    DWIDTH => 1,
    OUTREG => false
  )
  port map (
    -- global control --
    clk_i    => clk_i,
    -- write port --
    a_en_i   => tag_mem_en,
    a_rw_i   => tag_req_i.we,
    a_addr_i => tag_req_i.addr(awidth_c+1 downto 2),
    a_data_i => tag_req_i.data(0 downto 0),
    a_data_o => tag_mem_rd,
    -- read port --
    b_en_i   => mem_req_i.cyc,
    b_addr_i => mem_req_i.addr(awidth_c+1 downto 2),
    b_data_o => tag_err
  );

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
          tag_rsp_o.data <= x"0000000" & "000" & tag_mem_rd(0);
        end if;
        tag_rsp_o.ack  <= '1';
        tag_rsp_o.err  <= '0';
      end if;
    end if;
  end process tag_mem_handshake;


  -- Data Memory ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  data_mem_gen:
  for i in 0 to 3 generate -- four individual byte-wide RAMs
    data_mem_inst: entity neorv32.neorv32_prim_spram
    generic map (
      AWIDTH => awidth_c,
      DWIDTH => 8,
      OUTREG => false
    )
    port map (
      clk_i  => clk_i,
      en_i   => data_mem_en(i),
      rw_i   => mem_req_i.we,
      addr_i => mem_req_i.addr(awidth_c+1 downto 2),
      data_i => mem_req_i.data(i*8+7 downto i*8),
      data_o => data_mem_rd(i*8+7 downto i*8)
    );
  end generate;

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
          mem_rsp_o.ack  <= not tag_err(0);
          mem_rsp_o.err  <= tag_err(0);
        end if;
      end if;
    end if;
  end process data_mem_handshake;


end xbus_fmem_rtl;
