-- ================================================================================ --
-- NEORV32 SoC - 32 Binary Hardware Spinlocks (HWSPINLOCK)                          --
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

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_hwspinlock is
  port (
    clk_i     : in  std_ulogic; -- global clock line
    rstn_i    : in  std_ulogic; -- global reset line, low-active, async
    bus_req_i : in  bus_req_t;  -- bus request
    bus_rsp_o : out bus_rsp_t   -- bus response
  );
end neorv32_hwspinlock;

architecture neorv32_hwspinlock_rtl of neorv32_hwspinlock is

  signal lock_q, sel : std_ulogic_vector(31 downto 0);

begin

  -- Spinlocks ------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  spinlock_gen:
  for i in 0 to 31 generate

    -- binary spinlock --
    spinlock: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        lock_q(i) <= '0';
      elsif rising_edge(clk_i) then
        if (bus_req_i.stb = '1') and (bus_req_i.addr(7) = '0') and (sel(i) = '1') then
          lock_q(i) <= not bus_req_i.rw; -- claim on read, release on write
        end if;
      end if;
    end process spinlock;

    -- spinlock select --
    sel(i) <= '1' when (bus_req_i.addr(6 downto 2) = std_ulogic_vector(to_unsigned(i, 5))) else '0';

  end generate; -- /spinlock_gen


  -- Bus Response ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_response: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bus_rsp_o <= rsp_terminate_c;
    elsif rising_edge(clk_i) then
      bus_rsp_o.ack  <= bus_req_i.stb;
      bus_rsp_o.err  <= '0';
      bus_rsp_o.data <= (others => '0');
      if (bus_req_i.stb = '1') and (bus_req_i.rw = '0') then
        if (bus_req_i.addr(7) = '0') then -- individual lock state
          bus_rsp_o.data <= lock_q and sel;
        else -- state of all locks
          bus_rsp_o.data <= lock_q;
        end if;
      end if;
    end if;
  end process bus_response;

end neorv32_hwspinlock_rtl;
