--------------------------------------------------------------------------------
--  This file is a part of the NEORV32 project
--  Copyleft (ɔ) 2021, Susanin Crew / ArtfulChips
--
--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU General Public License for more details.
--
--  You should have received a copy of the GNU General Public License
--  along with this program; if not, write to the Free Software
--  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
--  ------------------------------------------------------------------------  --
--  Entity: 	   wb_dp_ram
--  File:        wb_dp_ram.vhd
--  Author:	     Serge Knigovedov, hitche/at\yandex.com
--  Description: Wishbone True Dual-Port RAM with single clock.
--               Read-during-write on port A or B returns newly written data
--------------------------------------------------------------------------------

library IEEE;
    use IEEE.STD_LOGIC_1164.all;
    use IEEE.NUMERIC_STD.all;

library neorv32;
    use neorv32.neorv32_package.all;

--use WORK.wishbone_package.all;

entity wb_dp_ram is
  generic (
    DATA_WIDTH  : natural := 32;
    MEM_BASE    : std_ulogic_vector(31 downto 0) := x"90000000"; -- memory base address
    MEM_SIZE    : natural := 32
  );
  port (
    clk         : in  std_ulogic;
    -- Wishbone bus interface
    wb_a_tag    : in  std_ulogic_vector( 2 downto 0); -- tag
    wb_a_adr    : in  std_ulogic_vector(31 downto 0); -- address
    wb_a_dat_o  : out std_ulogic_vector(31 downto 0); -- read data
    wb_a_dat_i  : in  std_ulogic_vector(31 downto 0); -- write data
    wb_a_we     : in  std_ulogic;                     -- read/write
    wb_a_sel    : in  std_ulogic_vector( 3 downto 0); -- byte enable
    wb_a_stb    : in  std_ulogic;                     -- strobe
    wb_a_cyc    : in  std_ulogic;                     -- valid cycle
    wb_a_lock   : in  std_ulogic;                     -- locked/exclusive bus access
    wb_a_ack    : out std_ulogic;                     -- transfer acknowledge
    wb_a_err    : out std_ulogic;                     -- transfer error
    wb_b_tag    : in  std_ulogic_vector( 2 downto 0); -- tag
    wb_b_adr    : in  std_ulogic_vector(31 downto 0); -- address
    wb_b_dat_o  : out std_ulogic_vector(31 downto 0); -- read data
    wb_b_dat_i  : in  std_ulogic_vector(31 downto 0); -- write data
    wb_b_we     : in  std_ulogic;                     -- read/write
    wb_b_sel    : in  std_ulogic_vector( 3 downto 0); -- byte enable
    wb_b_stb    : in  std_ulogic;                     -- strobe
    wb_b_cyc    : in  std_ulogic;                     -- valid cycle
    wb_b_lock   : in  std_ulogic;                     -- locked/exclusive bus access
    wb_b_ack    : out std_ulogic;                     -- transfer acknowledge
    wb_b_err    : out std_ulogic                      -- transfer error
    --wb_a_i  : in  wb_slv_in_type;
    --wb_a_o  : out wb_slv_out_type;
    --wb_b_i  : in  wb_slv_in_type;
    --wb_b_o  : out wb_slv_out_type
  );
end wb_dp_ram;

architecture rtl of wb_dp_ram is

  -- Build a 2-D array type for the RAM
  subtype word_type is std_ulogic_vector((DATA_WIDTH-1) downto 0);
  type memory_type is array(MEM_SIZE-1 downto 0) of word_type;

  -- Declare the RAM
  shared variable ram : memory_type;

  signal stb_a  : std_ulogic;
  signal stb_b  : std_ulogic;
  signal addr_a : std_ulogic_vector(index_size_f(MEM_SIZE)-1 downto 0);
  signal addr_b : std_ulogic_vector(index_size_f(MEM_SIZE)-1 downto 0);

begin

-- Interface A                                                                --
  stb_a     <= wb_a_stb when (wb_a_adr >= MEM_BASE) and (wb_a_adr < std_ulogic_vector(unsigned(MEM_BASE) + MEM_SIZE)) else '0';
  addr_a    <= wb_a_adr(index_size_f(MEM_SIZE)+1 downto 2); -- word aligned
  wb_a_err  <= '0';

  ram_a: process(clk)
  begin
    if rising_edge(clk) then
      if (stb_a and wb_a_cyc and wb_a_we) = '1' then
        ram(to_integer(unsigned(addr_a))) := wb_a_dat_i;
      end if;
      wb_a_dat_o <= ram(to_integer(unsigned(addr_a)));
      wb_a_ack <= stb_a; -- Synchronous slave
    end if;
  end process;

-- Interface B                                                                --
  stb_b     <= wb_b_stb when (wb_b_adr >= MEM_BASE) and (wb_b_adr < std_ulogic_vector(unsigned(MEM_BASE) + MEM_SIZE)) else '0';
  addr_b    <= wb_b_adr(index_size_f(MEM_SIZE)+1 downto 2); -- word aligned
  wb_b_err  <= '0';

  ram_b: process(clk)
  begin
    if rising_edge(clk) then
      if (stb_b and wb_b_cyc and wb_b_we) = '1' then
        ram(to_integer(unsigned(addr_b))) := wb_b_dat_i;
      end if;
      wb_b_dat_o <= ram(to_integer(unsigned(addr_b)));
      wb_b_ack <= stb_b; -- Synchronous slave
    end if;
  end process;

end rtl;
