-- ================================================================================ --
-- NEORV32 SoC - Generic Cache - Status and Data RAM Primitive Wrapper              --
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

entity neorv32_cache_ram is
  generic (
    TAG_WIDTH : natural; -- tag width
    IDX_WIDTH : natural; -- index width
    OFS_WIDTH : natural  -- offset width
  );
  port (
    -- global control --
    clk_i     : in  std_ulogic;                     -- global clock, rising edge
    addr_i    : in  std_ulogic_vector(31 downto 0); -- full byte address
    -- status memory --
    sta_we_i  : in  std_ulogic;                     -- status memory write-enable
    vld_i     : in  std_ulogic;                     -- valid-flag input
    vld_o     : out std_ulogic;                     -- valid-flag output
    tag_o     : out std_ulogic_vector(31 downto 0); -- zero-extended tag output
    -- data memory --
    data_we_i : in  std_ulogic_vector(3 downto 0);  -- byte-wise data write-enable
    data_i    : in  std_ulogic_vector(31 downto 0); -- write data
    data_o    : out std_ulogic_vector(31 downto 0)  -- read data
  );
end neorv32_cache_ram;

architecture neorv32_cache_ram_rtl of neorv32_cache_ram is

  signal sta_wr, sta_rd : std_ulogic_vector(TAG_WIDTH downto 0);

begin

  -- notifier --
  assert false report "[NEORV32] Using default CACHE RAM component." severity note;

  -- status RAM (valid-flag + tag) --
  status_ram_inst: entity neorv32.neorv32_prim_spram
  generic map (
    AWIDTH => IDX_WIDTH,
    DWIDTH => TAG_WIDTH + 1,
    OUTREG => 0
  )
  port map (
    clk_i  => clk_i,
    en_i   => '1',
    rw_i   => sta_we_i,
    addr_i => addr_i(31-TAG_WIDTH downto 2+OFS_WIDTH), -- index
    data_i => sta_wr, -- valid-flag & tag
    data_o => sta_rd
  );

  sta_wr <= vld_i & addr_i(31 downto 31-(TAG_WIDTH-1));
  vld_o  <= sta_rd(TAG_WIDTH); -- MSB = valid-flag
  tag_o(TAG_WIDTH-1 downto 0) <= sta_rd(TAG_WIDTH-1 downto 0); -- actual tag
  tag_o(31 downto TAG_WIDTH)  <= (others => '0'); -- zero-extend

  -- data RAM (four individual byte RAMs per word) --
  data_ram_gen:
  for i in 0 to 3 generate
    data_ram_inst: entity neorv32.neorv32_prim_spram
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
