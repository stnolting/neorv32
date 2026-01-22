-- ================================================================================ --
-- NEORV32 SoC - Generic Cache - Data and Tag RAM Primitive Wrapper                 --
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
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cache_ram is
  generic (
    TAG_WIDTH : natural; -- tag width
    IDX_WIDTH : natural; -- index width
    OFS_WIDTH : natural  -- offset width
  );
  port (
    clk_i     : in  std_ulogic;                     -- global clock, rising edge
    addr_i    : in  std_ulogic_vector(31 downto 0); -- full byte address
    tag_we_i  : in  std_ulogic;                     -- tag write-enable
    tag_o     : out std_ulogic_vector(31 downto 0); -- zero-extended read tag
    data_we_i : in  std_ulogic_vector(3 downto 0);  -- byte-wise data write-enable
    data_i    : in  std_ulogic_vector(31 downto 0); -- write data
    data_o    : out std_ulogic_vector(31 downto 0)  -- read data
  );
end neorv32_cache_ram;

architecture neorv32_cache_ram_rtl of neorv32_cache_ram is

  signal tag_rd : std_ulogic_vector(TAG_WIDTH-1 downto 0);

begin

  -- notifier --
  assert false report "[NEORV32] Using default CACHE RAM component." severity warning;

  -- tag RAM --
  tag_memory_inst: entity neorv32.neorv32_prim_spram
  generic map (
    AWIDTH => IDX_WIDTH,
    DWIDTH => TAG_WIDTH,
    OUTREG => 0
  )
  port map (
    clk_i  => clk_i,
    en_i   => '1',
    rw_i   => tag_we_i,
    addr_i => addr_i(31-TAG_WIDTH downto 2+OFS_WIDTH), -- index
    data_i => addr_i(31 downto 31-(TAG_WIDTH-1)), -- tag
    data_o => tag_rd
  );

  -- zero-extend tag output --
  tag_o <= std_ulogic_vector(resize(unsigned(tag_rd), 32));

  -- data RAM --
  data_memory_gen:
  for i in 0 to 3 generate
    data_memory_inst: entity neorv32.neorv32_prim_spram
    generic map (
      AWIDTH => IDX_WIDTH + OFS_WIDTH,
      DWIDTH => 8,
      OUTREG => 0
    )
    port map (
      clk_i  => clk_i,
      en_i   => '1',
      rw_i   => data_we_i(i),
      addr_i => addr_i(IDX_WIDTH+OFS_WIDTH+1 downto 2), -- index & word-offset
      data_i => data_i(i*8+7 downto i*8),
      data_o => data_o(i*8+7 downto i*8)
    );
  end generate;

end neorv32_cache_ram_rtl;
