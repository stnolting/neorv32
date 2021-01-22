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
--  Package:     wishbone_package
--  File:        wishbone_package.vhd
--  Author:      Serge Knigovedov, hitche/at\yandex.com
--  Description: Wishbone interface type declarations
--------------------------------------------------------------------------------
library IEEE;
    use IEEE.STD_LOGIC_1164.all;

package wishbone_package is

  -- Wishbone master inputs
  type wb_mst_in_type is record
    dat   : std_ulogic_vector(31 downto 0); -- read data
    ack   : std_ulogic;                     -- transfer acknowledge
    err   : std_ulogic;                     -- transfer error
  end record;

  -- Wishbone master outputs
  type wb_mst_out_type is record
    tag   : std_ulogic_vector( 2 downto 0); -- tag
    adr   : std_ulogic_vector(31 downto 0); -- address
    dat   : std_ulogic_vector(31 downto 0); -- write data
    we    : std_ulogic;                     -- read/write
    sel   : std_ulogic_vector( 3 downto 0); -- byte enable
    stb   : std_ulogic;                     -- strobe
    cyc   : std_ulogic;                     -- valid cycle
    lock  : std_ulogic;                     -- locked/exclusive bus access
  end record;

  -- Wishbone slave inputs
  type wb_slv_in_type is record
    tag   : std_ulogic_vector( 2 downto 0); -- tag
    adr   : std_ulogic_vector(31 downto 0); -- address
    dat   : std_ulogic_vector(31 downto 0); -- write data
    we    : std_ulogic;                     -- read/write
    sel   : std_ulogic_vector( 3 downto 0); -- byte enable
    stb   : std_ulogic;                     -- strobe
    cyc   : std_ulogic;                     -- valid cycle
    lock  : std_ulogic;                     -- locked/exclusive bus access
  end record;

  -- Wishbone slave outputs
  type wb_slv_out_type is record
    dat   : std_ulogic_vector(31 downto 0); -- read data
    ack   : std_ulogic;                     -- transfer acknowledge
    err   : std_ulogic;                     -- transfer error
  end record;

end wishbone_package;
