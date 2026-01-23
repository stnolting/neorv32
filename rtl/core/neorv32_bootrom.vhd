-- ================================================================================ --
-- NEORV32 SoC - Bootloader ROM (BOOTROM)                                           --
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

entity neorv32_bootrom is
  port (
    clk_i     : in  std_ulogic; -- global clock line
    rstn_i    : in  std_ulogic; -- async reset, low-active
    bus_req_i : in  bus_req_t;  -- bus request
    bus_rsp_o : out bus_rsp_t   -- bus response
  );
end neorv32_bootrom;

architecture neorv32_bootrom_rtl of neorv32_bootrom is

  -- BOOTROM ROM wrapper --
  -- [NOTE] We use component instantiation here to allow easy black-box instantiation for
  -- late component binding (e.g. when using the VHDL-to-Verilog flow with Verilog memory IP).
  component neorv32_bootrom_rom
  generic (
    AWIDTH : natural
  );
  port (
    clk_i  : in  std_ulogic;
    en_i   : in  std_ulogic;
    addr_i : in  std_ulogic_vector(31 downto 0);
    data_o : out std_ulogic_vector(31 downto 0)
  );
  end component;

  -- auto-configuration --
  constant awidth_c : natural := index_size_f(iodev_size_c); -- max address width (byte-addressing)

  -- local signals --
  signal rden  : std_ulogic;
  signal rdata : std_ulogic_vector(31 downto 0);

begin

  -- Pre-initialized Bootloader ROM (Wrapper) -----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bootrom_rom_inst: neorv32_bootrom_rom
  generic map (
    AWIDTH => awidth_c
  )
  port map (
    clk_i  => clk_i,
    en_i   => bus_req_i.stb,
    addr_i => bus_req_i.addr,
    data_o => rdata
  );

  -- Bus Handshake --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_handshake: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      rden <= '0';
    elsif rising_edge(clk_i) then
      rden <= bus_req_i.stb and (not bus_req_i.rw); -- read-only
    end if;
  end process bus_handshake;

  -- output gate --
  bus_rsp_o.data <= rdata when (rden = '1') else (others => '0');
  bus_rsp_o.ack  <= rden;
  bus_rsp_o.err  <= '0';

end neorv32_bootrom_rtl;
