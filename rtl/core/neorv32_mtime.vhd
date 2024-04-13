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
  signal mtime_lo_nxt : std_ulogic_vector(32 downto 0);
  signal mtime_lo_cry : std_ulogic_vector(00 downto 0);
  signal mtime_hi     : std_ulogic_vector(31 downto 0);

  -- comparators --
  signal cmp_lo_ge, cmp_lo_ge_ff, cmp_hi_eq, cmp_hi_gt : std_ulogic;

begin

  -- Bus Access -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      mtimecmp_lo    <= (others => '0');
      mtimecmp_hi    <= (others => '0');
      mtime_we       <= (others => '0');
      mtime_lo       <= (others => '0');
      mtime_lo_cry   <= (others => '0');
      mtime_hi       <= (others => '0');
      --
      bus_rsp_o.ack  <= '0';
      bus_rsp_o.err  <= '0';
      bus_rsp_o.data <= (others => '0');
    elsif rising_edge(clk_i) then
      -- mtimecmp --
      if (bus_req_i.stb = '1') and (bus_req_i.rw = '1') and (bus_req_i.addr(3) = '1') then
        if (bus_req_i.addr(2) = '0') then
          mtimecmp_lo <= bus_req_i.data;
        else
          mtimecmp_hi <= bus_req_i.data;
        end if;
      end if;

      -- mtime write access buffer --
      mtime_we(0) <= bus_req_i.stb and bus_req_i.rw and (not bus_req_i.addr(3)) and (not bus_req_i.addr(2));
      mtime_we(1) <= bus_req_i.stb and bus_req_i.rw and (not bus_req_i.addr(3)) and (    bus_req_i.addr(2));

      -- mtime.low --
      if (mtime_we(0) = '1') then -- write access
        mtime_lo <= bus_req_i.data;
      else -- auto increment
        mtime_lo <= mtime_lo_nxt(31 downto 0);
      end if;

      -- low-to-high carry --
      mtime_lo_cry(0) <= mtime_lo_nxt(32);

      -- mtime.high --
      if (mtime_we(1) = '1') then -- write access
        mtime_hi <= bus_req_i.data;
      else -- auto increment (if mtime.low overflows)
        mtime_hi <= std_ulogic_vector(unsigned(mtime_hi) + unsigned(mtime_lo_cry));
      end if;

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

  -- mtime.time_LO increment --
  mtime_lo_nxt <= std_ulogic_vector(unsigned('0' & mtime_lo) + 1);

  -- system time output --
  time_o <= mtime_hi & mtime_lo; -- NOTE: low and high words are not synchronized here!


  -- Comparator -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cmp_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      cmp_lo_ge_ff <= '0';
      irq_o        <= '0';
    elsif rising_edge(clk_i) then
      cmp_lo_ge_ff <= cmp_lo_ge; -- there is one cycle delay between low (earlier) and high (later) word
      irq_o        <= cmp_hi_gt or (cmp_hi_eq and cmp_lo_ge_ff);
    end if;
  end process cmp_sync;

  -- sub-word comparators --
  cmp_lo_ge <= '1' when (unsigned(mtime_lo) >= unsigned(mtimecmp_lo)) else '0'; -- low-word: greater than or equal
  cmp_hi_eq <= '1' when (unsigned(mtime_hi) =  unsigned(mtimecmp_hi)) else '0'; -- high-word: equal
  cmp_hi_gt <= '1' when (unsigned(mtime_hi) >  unsigned(mtimecmp_hi)) else '0'; -- high-word: greater than


end neorv32_mtime_rtl;
