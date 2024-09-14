-- ================================================================================ --
-- NEORV32 SoC - RISC-V-Compatible Machine Timer (MTIME)                            --
-- -------------------------------------------------------------------------------- --
-- The 64-bit counter core is split into two decoupled 32-bit registers to provide  --
-- a shorter carry chain (improving timing).                                        --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_mtime is
  port (
    clk_i     : in  std_ulogic; -- global clock line
    rstn_i    : in  std_ulogic; -- global reset line, low-active, async
    bus_req_i : in  bus_req_t;  -- bus request
    bus_rsp_o : out bus_rsp_t;  -- bus response
    time_o    : out std_ulogic_vector(63 downto 0); -- current system time
    irq_o     : out std_ulogic  -- interrupt request
  );
end neorv32_mtime;

architecture neorv32_mtime_rtl of neorv32_mtime is

  -- time write access buffer --
  signal mtime_we : std_ulogic_vector(1 downto 0);

  -- counter core --
  signal mtimecmp, mtime : std_ulogic_vector(63 downto 0);
  signal buf_lo          : std_ulogic_vector(31 downto 0);
  signal inc_lo, inc_hi  : std_ulogic_vector(32 downto 0);
  signal carry           : std_ulogic_vector(0 downto 0);

  -- comparator --
  signal cmp_lo_eq, cmp_lo_gt, cmp_lo_ge, cmp_hi_eq, cmp_hi_gt : std_ulogic;

begin

  -- Bus Access -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      mtimecmp  <= (others => '0');
      mtime_we  <= (others => '0');
      bus_rsp_o <= rsp_terminate_c;
    elsif rising_edge(clk_i) then
      -- mtimecmp write access --
      if (bus_req_i.stb = '1') and (bus_req_i.rw = '1') and (bus_req_i.addr(3) = '1') then
        if (bus_req_i.addr(2) = '0') then
          mtimecmp(31 downto 0) <= bus_req_i.data;
        else
          mtimecmp(63 downto 32) <= bus_req_i.data;
        end if;
      end if;
      -- mtime write access buffer --
      mtime_we(0) <= bus_req_i.stb and bus_req_i.rw and (not bus_req_i.addr(3)) and (not bus_req_i.addr(2));
      mtime_we(1) <= bus_req_i.stb and bus_req_i.rw and (not bus_req_i.addr(3)) and (    bus_req_i.addr(2));
      -- read access --
      bus_rsp_o.ack  <= bus_req_i.stb; -- bus handshake
      bus_rsp_o.err  <= '0'; -- no access errors
      bus_rsp_o.data <= (others => '0'); -- default
      if (bus_req_i.stb = '1') and (bus_req_i.rw = '0') then
        case bus_req_i.addr(3 downto 2) is
          when "00"   => bus_rsp_o.data <= mtime(31 downto 0);
          when "01"   => bus_rsp_o.data <= mtime(63 downto 32);
          when "10"   => bus_rsp_o.data <= mtimecmp(31 downto 0);
          when others => bus_rsp_o.data <= mtimecmp(63 downto 32);
        end case;
      end if;
    end if;
  end process bus_access;


  -- 64-Bit Counter Core (split into two 32-bit registers) ----------------------------------
  -- -------------------------------------------------------------------------------------------
  counter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      mtime  <= (others => '0');
      buf_lo <= (others => '0');
      carry  <= (others => '0');
    elsif rising_edge(clk_i) then
      -- low-word --
      if (mtime_we(0) = '1') then -- write access (data is stable for at least one cycle after STB becomes low)
        mtime(31 downto 0) <= bus_req_i.data;
      else -- auto increment
        mtime(31 downto 0) <= inc_lo(31 downto 0);
      end if;
      buf_lo   <= mtime(31 downto 0); -- delay low-word by one cycle for system time output
      carry(0) <= inc_lo(32); -- low-to-high carry
      -- high-word --
      if (mtime_we(1) = '1') then -- write access (data is stable for at least one cycle after STB becomes low)
        mtime(63 downto 32) <= bus_req_i.data;
      else -- auto increment
        mtime(63 downto 32) <= inc_hi(31 downto 0);
      end if;
    end if;
  end process counter;

  -- increments --
  inc_lo <= std_ulogic_vector(unsigned('0' & mtime(31 downto  0)) + 1);
  inc_hi <= std_ulogic_vector(unsigned('0' & mtime(63 downto 32)) + unsigned(carry));

  -- system time output --
  time_o <= mtime(63 downto 32) & buf_lo;


  -- Interrupt Generator (comparator) -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_gen: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      cmp_lo_ge <= '0';
      irq_o     <= '0';
    elsif rising_edge(clk_i) then
      cmp_lo_ge <= cmp_lo_gt or cmp_lo_eq; -- low word greater-than or equal
      irq_o     <= cmp_hi_gt or (cmp_hi_eq and cmp_lo_ge);
    end if;
  end process irq_gen;

  -- sub-word comparators; there is one cycle delay between low (earlier) and high (later) word --
  cmp_lo_eq <= '1' when (unsigned(mtime(31 downto  0)) = unsigned(mtimecmp(31 downto  0))) else '0';
  cmp_lo_gt <= '1' when (unsigned(mtime(31 downto  0)) > unsigned(mtimecmp(31 downto  0))) else '0';
  cmp_hi_eq <= '1' when (unsigned(mtime(63 downto 32)) = unsigned(mtimecmp(63 downto 32))) else '0';
  cmp_hi_gt <= '1' when (unsigned(mtime(63 downto 32)) > unsigned(mtimecmp(63 downto 32))) else '0';


end neorv32_mtime_rtl;
