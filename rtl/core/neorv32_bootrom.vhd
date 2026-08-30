-- ================================================================================ --
-- NEORV32 SoC - Bootloader ROM (BOOTROM)                                           --
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
use neorv32.neorv32_bootrom_image.all;

entity neorv32_bootrom is
  port (
    -- global control --
    clk_i      : in  std_ulogic;                     -- clock, trigger on rising edge
    rstn_i     : in  std_ulogic;                     -- async reset, low-active
    -- bus request --
    req_addr_i : in  std_ulogic_vector(15 downto 0); -- access address (byte-addressing)
    req_ben_i  : in  std_ulogic_vector(3 downto 0);  -- byte enable
    req_stb_i  : in  std_ulogic;                     -- request strobe
    req_rw_i   : in  std_ulogic;                     -- 0 = read, 1 = write
    -- bus response --
    rsp_data_o : out std_ulogic_vector(31 downto 0); -- read data
    rsp_ack_o  : out std_ulogic;                     -- access acknowledge
    rsp_err_o  : out std_ulogic                      -- access error
  );
end entity;

architecture neorv32_bootrom_rtl of neorv32_bootrom is

  constant awidth_c : natural := index_size_f(image_size_c); -- physical byte address width

  signal rdack : std_ulogic;
  signal rdata : std_ulogic_vector(31 downto 0);

begin

  -- memory read access --
  rom_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (req_stb_i = '1') then
        rdata <= image_data_c(to_integer(unsigned(req_addr_i(awidth_c-1 downto 2))));
      end if;
    end if;
  end process;

  -- bus handshake --
  bus_handshake: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      rdack <= '0';
    elsif rising_edge(clk_i) then
      rdack <= req_stb_i and (not req_rw_i); -- read-only
    end if;
  end process;

  -- output gate --
  rsp_data_o <= rdata when (rdack = '1') else (others => '0');
  rsp_ack_o  <= rdack;
  rsp_err_o  <= '0'; -- no access errors supported (could be used for ECC / parity checks)

end architecture;
