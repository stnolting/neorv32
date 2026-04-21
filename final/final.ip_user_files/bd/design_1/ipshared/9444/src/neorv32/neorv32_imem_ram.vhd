-- ================================================================================ --
-- NEORV32 SoC - Instruction Memory (IMEM) - RAM Primitive Wrapper                  --
-- -------------------------------------------------------------------------------- --
-- Replace this file by a more efficient technology-specific IP wrapper. The read-  --
-- during-write behavior is irrelevant as read/write accesses are mutual exclusive. --
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

entity neorv32_imem_ram is
  generic (
    AWIDTH : natural; -- address width (byte address)
    OUTREG : natural  -- add output register stage when 1
  );
  port (
    clk_i  : in  std_ulogic;                     -- clock, rising-edge
    en_i   : in  std_ulogic_vector(3 downto 0);  -- byte-wise access-enable
    rw_i   : in  std_ulogic;                     -- 0=read, 1=write
    addr_i : in  std_ulogic_vector(31 downto 0); -- full byte address
    data_i : in  std_ulogic_vector(31 downto 0); -- write data
    data_o : out std_ulogic_vector(31 downto 0)  -- read data, sync
  );
end neorv32_imem_ram;

architecture neorv32_imem_ram_rtl of neorv32_imem_ram is

begin

  -- notifier --
  assert false report "[NEORV32] Using default IMEM RAM component." severity note;

  -- 4x byte-wide RAMs --
  ram_gen:
  for i in 0 to 3 generate
    ram_inst: entity neorv32.neorv32_prim_spram
    generic map (
      AWIDTH => AWIDTH-2,
      DWIDTH => 8,
      OUTREG => OUTREG
    )
    port map (
      clk_i  => clk_i,
      en_i   => en_i(i),
      rw_i   => rw_i,
      addr_i => addr_i(AWIDTH-1 downto 2),
      data_i => data_i(i*8+7 downto i*8),
      data_o => data_o(i*8+7 downto i*8)
    );
  end generate;

end neorv32_imem_ram_rtl;
