-- ================================================================================ --
-- NEORV32 SoC - RISC-V-Compatible Machine System Timer (MTIME)                     --
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

  -- mtime.time write access buffer --
  signal mtime_we : std_ulogic_vector(1 downto 0);

  -- accessible regs --
  signal mtimecmp_lo  : std_ulogic_vector(31 downto 0);
  signal mtimecmp_hi  : std_ulogic_vector(31 downto 0);
  signal mtime_lo     : std_ulogic_vector(31 downto 0);
  signal mtime_lo_q   : std_ulogic_vector(31 downto 0);
  signal mtime_hi     : std_ulogic_vector(31 downto 0);
  signal mtime_lo_inc : std_ulogic_vector(32 downto 0);
  signal carry        : std_ulogic_vector( 0 downto 0);
  signal mtime_hi_inc : std_ulogic_vector(31 downto 0);

  -- comparator --
  signal cmp_lo_eq, cmp_lo_gt, cmp_lo_ge, cmp_hi_eq, cmp_hi_gt : std_ulogic;

begin

  -- Bus Access -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      mtimecmp_lo <= (others => '0');
      mtimecmp_hi <= (others => '0');
      mtime_we    <= (others => '0');
      bus_rsp_o   <= rsp_terminate_c;
    elsif rising_edge(clk_i) then
      -- MTIMECMP write access --
      if (bus_req_i.stb = '1') and (bus_req_i.rw = '1') and (bus_req_i.addr(3) = '1') then
        if (bus_req_i.addr(2) = '0') then
          mtimecmp_lo <= bus_req_i.data;
        else
          mtimecmp_hi <= bus_req_i.data;
        end if;
      end if;
      -- MTIME write access buffer --
      mtime_we(0) <= bus_req_i.stb and bus_req_i.rw and (not bus_req_i.addr(3)) and (not bus_req_i.addr(2));
      mtime_we(1) <= bus_req_i.stb and bus_req_i.rw and (not bus_req_i.addr(3)) and (    bus_req_i.addr(2));
      -- read access --
      bus_rsp_o.ack  <= bus_req_i.stb; -- bus handshake
      bus_rsp_o.err  <= '0'; -- no access errors
      bus_rsp_o.data <= (others => '0'); -- default
      if (bus_req_i.stb = '1') and (bus_req_i.rw = '0') then
        case bus_req_i.addr(3 downto 2) is
          when "00"   => bus_rsp_o.data <= mtime_lo;
          when "01"   => bus_rsp_o.data <= mtime_hi;
          when "10"   => bus_rsp_o.data <= mtimecmp_lo;
          when others => bus_rsp_o.data <= mtimecmp_hi;
        end case;
      end if;
    end if;
  end process bus_access;


  -- 64-Bit MTIME Counter -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  counter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      mtime_lo <= (others => '0');
      carry    <= (others => '0');
      mtime_hi <= (others => '0');
    elsif rising_edge(clk_i) then
      -- low-word --
      if (mtime_we(0) = '1') then -- write access
        mtime_lo <= bus_req_i.data; -- write data is stable for at least one cycle after STB becomes low
        carry(0) <= '0';
      else -- auto increment
        mtime_lo <= mtime_lo_inc(31 downto 0);
        carry(0) <= mtime_lo_inc(32);
      end if;
      -- high-word --
      if (mtime_we(1) = '1') then -- write access
        mtime_hi <= bus_req_i.data; -- write data is stable for at least one cycle after STB becomes low
      else -- auto increment
        mtime_hi <= mtime_hi_inc;
      end if;
    end if;
  end process counter;

  -- time increment --
  mtime_lo_inc <= std_ulogic_vector(unsigned('0' & mtime_lo) + 1);
  mtime_hi_inc <= std_ulogic_vector(unsigned(mtime_hi) + unsigned(carry));


  -- Synchronize Output Words ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  out_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      mtime_lo_q <= (others => '0');
    elsif rising_edge(clk_i) then
      mtime_lo_q <= mtime_lo;
    end if;
  end process out_sync;

  -- delay low-word by one cycle --
  time_o <= mtime_hi & mtime_lo_q;


  -- Comparator (Interrupt Generator) -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_gen: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      cmp_lo_ge <= '0';
      irq_o     <= '0';
    elsif rising_edge(clk_i) then
      cmp_lo_ge <= cmp_lo_gt or cmp_lo_eq; -- low word greater than or equal
      irq_o     <= cmp_hi_gt or (cmp_hi_eq and cmp_lo_ge);
    end if;
  end process irq_gen;

  -- sub-word comparators; there is one cycle delay between low (earlier) and high (later) word --
  cmp_lo_eq <= '1' when (unsigned(mtime_lo) = unsigned(mtimecmp_lo)) else '0'; -- low-word equal
  cmp_lo_gt <= '1' when (unsigned(mtime_lo) > unsigned(mtimecmp_lo)) else '0'; -- low-word greater than
  cmp_hi_eq <= '1' when (unsigned(mtime_hi) = unsigned(mtimecmp_hi)) else '0'; -- high-word equal
  cmp_hi_gt <= '1' when (unsigned(mtime_hi) > unsigned(mtimecmp_hi)) else '0'; -- high-word greater than


end neorv32_mtime_rtl;
