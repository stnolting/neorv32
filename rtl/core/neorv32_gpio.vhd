-- ================================================================================ --
-- NEORV32 SoC - General Purpose Parallel Input/Output Port (GPIO)                  --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_gpio is
  generic (
    GPIO_NUM : natural range 0 to 64 -- number of GPIO input/output pairs (0..64)
  );
  port (
    clk_i     : in  std_ulogic; -- global clock line
    rstn_i    : in  std_ulogic; -- global reset line, low-active, async
    bus_req_i : in  bus_req_t;  -- bus request
    bus_rsp_o : out bus_rsp_t;  -- bus response
    gpio_o    : out std_ulogic_vector(63 downto 0); -- parallel output
    gpio_i    : in  std_ulogic_vector(63 downto 0)  -- parallel input
  );
end neorv32_gpio;

architecture neorv32_gpio_rtl of neorv32_gpio is

  signal din, din_rd, dout, dout_rd : std_ulogic_vector(63 downto 0);

begin

  -- Bus Access -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bus_rsp_o <= rsp_terminate_c;
      dout      <= (others => '0');
    elsif rising_edge(clk_i) then
      -- bus handshake --
      bus_rsp_o.ack  <= bus_req_i.stb;
      bus_rsp_o.err  <= '0';
      bus_rsp_o.data <= (others => '0');
      if (bus_req_i.stb = '1') then
        if (bus_req_i.rw = '1') then -- write access
          if (bus_req_i.addr(3 downto 2) = "10") then
            dout(31 downto 00) <= bus_req_i.data;
          end if;
          if (bus_req_i.addr(3 downto 2) = "11") then
            dout(63 downto 32) <= bus_req_i.data;
          end if;
        else -- read access
          case bus_req_i.addr(3 downto 2) is
            when "00"   => bus_rsp_o.data <= din_rd(31 downto 00);
            when "01"   => bus_rsp_o.data <= din_rd(63 downto 32);
            when "10"   => bus_rsp_o.data <= dout_rd(31 downto 00);
            when others => bus_rsp_o.data <= dout_rd(63 downto 32);
          end case;
        end if;

      end if;
    end if;
  end process bus_access;


  -- Physical Pin Mapping -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  pin_mapping: process(din, dout)
  begin
    din_rd  <= (others => '0');
    dout_rd <= (others => '0');
    for i in 0 to GPIO_NUM-1 loop
      din_rd(i)  <= din(i);
      dout_rd(i) <= dout(i);
    end loop;
  end process pin_mapping;

  -- output --
  gpio_o <= dout_rd;

  -- synchronize input --
  input_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      din <= (others => '0');
    elsif rising_edge(clk_i) then
      din <= gpio_i;
    end if;
  end process input_sync;


end neorv32_gpio_rtl;
