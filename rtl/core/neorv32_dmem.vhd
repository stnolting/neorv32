-- ================================================================================ --
-- NEORV32 SoC - Data Memory (DMEM)                                                 --
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

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_dmem is
  generic (
    AWIDTH : natural; -- memory address width (byte-addressing)
    OUTREG : boolean  -- add output register stage
  );
  port (
    -- global control --
    clk_i      : in  std_ulogic;                     -- clock, trigger on rising edge
    rstn_i     : in  std_ulogic;                     -- async reset, low-active
    -- bus request --
    req_addr_i : in  std_ulogic_vector(31 downto 0); -- access address (byte-addressing)
    req_data_i : in  std_ulogic_vector(31 downto 0); -- write data
    req_ben_i  : in  std_ulogic_vector(3 downto 0);  -- byte enable
    req_stb_i  : in  std_ulogic;                     -- request strobe
    req_rw_i   : in  std_ulogic;                     -- 0 = read, 1 = write
    -- bus response --
    rsp_data_o : out std_ulogic_vector(31 downto 0); -- read data
    rsp_ack_o  : out std_ulogic;                     -- access acknowledge
    rsp_err_o  : out std_ulogic                      -- access error
  );
end entity;

architecture neorv32_dmem_rtl of neorv32_dmem is

  constant outreg_c : natural := sel_natural_f(OUTREG, 1, 0); -- add output register?

  signal en    : std_ulogic_vector(3 downto 0);
  signal rdata : std_ulogic_vector(31 downto 0);
  signal wrack : std_ulogic;
  signal rdack : std_ulogic_vector(1 downto 0);

begin

  -- memory core --
  dmem_ram_gen:
  for i in 0 to 3 generate -- 4x byte-wide RAMs
    dmem_ram: entity neorv32.neorv32_prim_spram
    generic map (
      AWIDTH => AWIDTH-2,
      DWIDTH => 8,
      OUTREG => OUTREG
    )
    port map (
      clk_i  => clk_i,
      en_i   => en(i),
      rw_i   => req_rw_i,
      addr_i => req_addr_i(AWIDTH-1 downto 2),
      data_i => req_data_i(i*8+7 downto i*8),
      data_o => rdata(i*8+7 downto i*8)
    );
    en(i) <= req_ben_i(i) and req_stb_i; -- byte-wise enable
  end generate;

  -- bus handshake --
  bus_handshake: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      wrack <= '0';
      rdack <= (others => '0');
    elsif rising_edge(clk_i) then
      wrack <= req_stb_i and req_rw_i;
      rdack <= rdack(0) & (req_stb_i and (not req_rw_i));
    end if;
  end process;

  -- output gate --
  rsp_data_o <= rdata when (rdack(outreg_c) = '1') else (others => '0');
  rsp_ack_o  <= wrack or rdack(outreg_c);
  rsp_err_o  <= '0'; -- no access errors supported (could be used for ECC / parity checks)

end architecture;
