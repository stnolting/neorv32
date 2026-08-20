-- ================================================================================ --
-- NEORV32 SoC - Instruction Memory (IMEM)                                          --
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

entity neorv32_imem is
  generic (
    AWIDTH  : natural; -- memory address width (byte-addressing)
    INITROM : boolean; -- implement IMEM as pre-initialized read-only memory?
    OUTREG  : boolean  -- add output register stage
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

architecture neorv32_imem_rtl of neorv32_imem is

  component neorv32_imem_rom -- IMEM ROM wrapper
  generic (
    AWIDTH : natural;
    OUTREG : boolean
  );
  port (
    clk_i  : in  std_ulogic;
    en_i   : in  std_ulogic;
    addr_i : in  std_ulogic_vector(31 downto 0);
    data_o : out std_ulogic_vector(31 downto 0)
  );
  end component;

  constant outreg_c : natural := sel_natural_f(OUTREG, 1, 0); -- add output register?

  signal en    : std_ulogic_vector(3 downto 0);
  signal rdata : std_ulogic_vector(31 downto 0);
  signal wrack : std_ulogic;
  signal rdack : std_ulogic_vector(1 downto 0);

begin

  -- IMEM as pre-initialized ROM --
  imem_rom:
  if INITROM generate
    imem_rom_inst: neorv32_imem_rom
    generic map (
      AWIDTH => AWIDTH,
      OUTREG => OUTREG
    )
    port map (
      clk_i  => clk_i,
      en_i   => req_stb_i,
      addr_i => req_addr_i,
      data_o => rdata
    );
  end generate;

  -- IMEM as plain RAM --
  imem_ram:
  if not INITROM generate
    imem_ram_gen:
    for i in 0 to 3 generate -- 4x byte-wide RAMs
      imem_ram_inst: entity neorv32.neorv32_prim_spram
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

  rsp_data_o <= rdata when (rdack(outreg_c) = '1') else (others => '0');
  rsp_ack_o  <= rdack(outreg_c) when INITROM else (rdack(outreg_c) or wrack);
  rsp_err_o  <= '0'; -- no access errors supported (could be used for ECC / parity checks)

end architecture;

-- ================================================================================ --
-- NEORV32 SoC - Instruction Memory (IMEM) - ROM Primitive Wrapper                  --
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
    OUTREG : boolean  -- add output register stage when true
  );
  port (
    clk_i  : in  std_ulogic;                     -- clock, rising-edge
    en_i   : in  std_ulogic;                     -- access-enable
    addr_i : in  std_ulogic_vector(31 downto 0); -- full byte address
    data_o : out std_ulogic_vector(31 downto 0)  -- read data, sync
  );
end entity;

architecture neorv32_imem_rom_rtl of neorv32_imem_rom is

  constant awidth_c : natural := index_size_f(image_size_c); -- physical byte address width
  signal rdata : std_ulogic_vector(31 downto 0);

begin

  -- size check --
  assert (image_size_c <= 2**AWIDTH) report
    "[NEORV32] IMEM image (" & natural'image(image_size_c) & " bytes) " &
    "overflows IMEM size (" & natural'image(2**AWIDTH) & " bytes)!" severity error;

  -- ROM --
  rom_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (en_i = '1') then
        rdata <= image_data_c(to_integer(unsigned(addr_i(awidth_c-1 downto 2))));
      end if;
    end if;
  end process;

  -- output register stage --
  rom_output_register_enabled:
  if OUTREG generate
    rom_outreg: process(clk_i)
    begin
      if rising_edge(clk_i) then
        data_o <= rdata;
      end if;
    end process;
  end generate;

  -- no output register stage --
  rom_output_register_disabled:
  if not OUTREG generate
    data_o <= rdata;
  end generate;

end architecture;
