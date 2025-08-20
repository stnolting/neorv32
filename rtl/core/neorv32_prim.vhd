-- ================================================================================ --
-- NEORV32 - Primitives - Generic Single-Port RAM (SPRAM)                           --
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

entity neorv32_prim_spram is
  generic (
    AWIDTH : natural; -- address width (number of bits)
    DWIDTH : natural; -- data width (number of bits)
    OUTREG : boolean  -- add output register stage
  );
  port (
    -- global control --
    clk_i  : in  std_ulogic;                           -- global clock
    -- read/write port --
    en_i   : in  std_ulogic;                           -- access enable
    rw_i   : in  std_ulogic;                           -- 0=read, 1=write
    addr_i : in  std_ulogic_vector(AWIDTH-1 downto 0); -- address
    data_i : in  std_ulogic_vector(DWIDTH-1 downto 0); -- write data
    data_o : out std_ulogic_vector(DWIDTH-1 downto 0)  -- read data
  );
end neorv32_prim_spram;

architecture neorv32_prim_spram_rtl of neorv32_prim_spram is

  type ram_t is array (0 to (2**AWIDTH)-1) of std_ulogic_vector(DWIDTH-1 downto 0);
  signal ram : ram_t;
  signal rdata : std_ulogic_vector(DWIDTH-1 downto 0);

begin

  -- Memory Core ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  memory_core: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (en_i = '1') then
        if (rw_i = '1') then
          ram(to_integer(unsigned(addr_i))) <= data_i;
        else
          rdata <= ram(to_integer(unsigned(addr_i)));
        end if;
      end if;
    end if;
  end process memory_core;

  -- Output Register ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  output_register_enabled: -- might improve FPGA mapping and/or timing results
  if OUTREG generate
    read_outreg: process(clk_i)
    begin
      if rising_edge(clk_i) then
        data_o <= rdata;
      end if;
    end process read_outreg;
  end generate;

  -- no output register --
  output_register_disabled:
  if not OUTREG generate
    data_o <= rdata;
  end generate;

end neorv32_prim_spram_rtl;


-- ================================================================================ --
-- NEORV32 - Primitives - Generic Simple Dual-Port RAM (SDPRAM)                     --
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

entity neorv32_prim_sdpram is
  generic (
    AWIDTH : natural; -- address width (number of bits)
    DWIDTH : natural; -- data width (number of bits)
    OUTREG : boolean  -- add output register stage
  );
  port (
    -- global control --
    clk_i    : in  std_ulogic;                           -- global clock
    -- read/write port --
    a_en_i   : in  std_ulogic;                           -- access enable
    a_rw_i   : in  std_ulogic;                           -- 0=read, 1=write
    a_addr_i : in  std_ulogic_vector(AWIDTH-1 downto 0); -- read/write address
    a_data_i : in  std_ulogic_vector(DWIDTH-1 downto 0); -- write data
    a_data_o : out std_ulogic_vector(DWIDTH-1 downto 0); -- read data
    -- read port --
    b_en_i   : in  std_ulogic;                           -- access enable
    b_addr_i : in  std_ulogic_vector(AWIDTH-1 downto 0); -- read address
    b_data_o : out std_ulogic_vector(DWIDTH-1 downto 0)  -- read data
  );
end neorv32_prim_sdpram;

architecture neorv32_prim_sdpram_rtl of neorv32_prim_sdpram is

  type ram_t is array (0 to (2**AWIDTH)-1) of std_ulogic_vector(DWIDTH-1 downto 0);
  signal ram : ram_t;
  signal a_rdata, b_rdata : std_ulogic_vector(DWIDTH-1 downto 0);

begin

  -- Memory Core ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  memory_core: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- port A --
      if (a_en_i = '1') then
        if (a_rw_i = '1') then
          ram(to_integer(unsigned(a_addr_i))) <= a_data_i;
        else
          a_rdata <= ram(to_integer(unsigned(a_addr_i)));
        end if;
      end if;
      -- port B --
      if (b_en_i = '1') then
        b_rdata <= ram(to_integer(unsigned(b_addr_i)));
      end if;
    end if;
  end process memory_core;

  -- Output Register ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  output_register_enabled: -- might improve FPGA mapping and/or timing results
  if OUTREG generate
    read_outreg: process(clk_i)
    begin
      if rising_edge(clk_i) then
        a_data_o <= a_rdata;
        b_data_o <= b_rdata;
      end if;
    end process read_outreg;
  end generate;

  -- no output register --
  output_register_disabled:
  if not OUTREG generate
    a_data_o <= a_rdata;
    b_data_o <= b_rdata;
  end generate;

end neorv32_prim_sdpram_rtl;
