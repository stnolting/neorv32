-- ================================================================================ --
-- NEORV32 SoC - Data Memory (DMEM)                                                 --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_dmem is
  generic (
    MEM_SIZE : natural; -- memory size in bytes, has to be a power of 2, min 4
    OUTREG   : boolean  -- implement output register stage
  );
  port (
    clk_i     : in  std_ulogic; -- global clock line
    rstn_i    : in  std_ulogic; -- async reset, low-active
    bus_req_i : in  bus_req_t;  -- bus request
    bus_rsp_o : out bus_rsp_t   -- bus response
  );
end neorv32_dmem;

architecture neorv32_dmem_rtl of neorv32_dmem is

  -- DMEM RAM wrapper --
  -- [NOTE] We use component instantiation here to allow easy black-box instantiation for
  -- late component binding (e.g. when using the VHDL-to-Verilog flow with Verilog memory IP).
  component neorv32_dmem_ram
  generic (
    AWIDTH : natural;
    OUTREG : natural
  );
  port (
    clk_i  : in  std_ulogic;
    en_i   : in  std_ulogic_vector(3 downto 0);
    rw_i   : in  std_ulogic;
    addr_i : in  std_ulogic_vector(31 downto 0);
    data_i : in  std_ulogic_vector(31 downto 0);
    data_o : out std_ulogic_vector(31 downto 0)
  );
  end component;

  -- auto-configuration --
  constant awidth_c : natural := index_size_f(MEM_SIZE); -- address width (byte-addressing)
  constant outreg_c : natural := cond_sel_natural_f(OUTREG, 1, 0); -- add output register?

  -- local signals --
  signal rdata : std_ulogic_vector(31 downto 0);
  signal wren  : std_ulogic;
  signal rden  : std_ulogic_vector(1 downto 0);
  signal ben   : std_ulogic_vector(3 downto 0);

begin

  -- Data Memory (Wrapper) ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  dmem_ram_inst: neorv32_dmem_ram
  generic map (
    AWIDTH => awidth_c,
    OUTREG => outreg_c
  )
  port map (
    clk_i  => clk_i,
    en_i   => ben,
    rw_i   => bus_req_i.rw,
    addr_i => bus_req_i.addr,
    data_i => bus_req_i.data,
    data_o => rdata
  );

  -- byte-wise enable --
  ben <= bus_req_i.ben when (bus_req_i.stb = '1') else (others => '0');

  -- Bus Handshake --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_handshake: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      wren <= '0';
      rden <= (others => '0');
    elsif rising_edge(clk_i) then
      wren <= bus_req_i.stb and bus_req_i.rw;
      rden <= rden(0) & (bus_req_i.stb and (not bus_req_i.rw));
    end if;
  end process bus_handshake;

  bus_rsp_o.data <= rdata when (rden(outreg_c) = '1') else (others => '0'); -- output gate
  bus_rsp_o.err  <= '0';
  bus_rsp_o.ack  <= rden(outreg_c) or wren;

end neorv32_dmem_rtl;
