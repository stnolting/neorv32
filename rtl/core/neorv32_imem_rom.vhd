-- ================================================================================ --
-- NEORV32 SoC - Instruction Memory (IMEM) - ROM Primitive Wrapper                  --
-- -------------------------------------------------------------------------------- --
-- Replace this file by a more efficient technology-specific IP wrapper.            --
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
use neorv32.neorv32_imem_image.all;

entity neorv32_imem_rom is
  generic (
    AWIDTH : natural; -- address width (byte address)
    OUTREG : natural  -- add output register stage when 1
  );
  port (
    clk_i  : in  std_ulogic;                     -- clock, rising-edge
    en_i   : in  std_ulogic;                     -- access-enable
    addr_i : in  std_ulogic_vector(31 downto 0); -- full byte address
    data_o : out std_ulogic_vector(31 downto 0)  -- read data, sync
  );
end neorv32_imem_rom;

architecture neorv32_imem_rom_rtl of neorv32_imem_rom is

  constant awidth_c : natural := index_size_f(image_size_c); -- byte address width
  signal rdata : std_ulogic_vector(31 downto 0);

begin

  -- notifier --
  assert false report
    "[NEORV32] Using default IMEM ROM component (" &
    natural'image(2**awidth_c) & " bytes)." severity warning;

  -- size check --
  assert (image_size_c <= 2**AWIDTH) report
    "[NEORV32] Application image (" & natural'image(image_size_c) & " bytes) " &
    "overflows processor-internal IMEM (" & natural'image(2**AWIDTH) & " bytes)!" severity error;

  -- ROM --
  rom_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (en_i = '1') then
        rdata <= image_data_c(to_integer(unsigned(addr_i(awidth_c-1 downto 2))));
      end if;
    end if;
  end process rom_access;

  -- output register stage --
  rom_output_register_enabled:
  if (OUTREG = 1) generate
    rom_outreg: process(clk_i)
    begin
      if rising_edge(clk_i) then
        data_o <= rdata;
      end if;
    end process rom_outreg;
  end generate;

  -- no output register stage --
  rom_output_register_disabled:
  if (OUTREG = 0) generate
    data_o <= rdata;
  end generate;

end neorv32_imem_rom_rtl;
