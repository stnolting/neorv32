-- ================================================================================ --
-- NEORV32 CPU - Load/Store Unit                                                    --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cpu_lsu is
  generic (
    HART_ID : natural; -- hardware thread ID
    AMO_EN  : boolean  -- enable atomic memory accesses
  );
  port (
    -- global control --
    clk_i       : in  std_ulogic; -- global clock, rising edge
    rstn_i      : in  std_ulogic; -- global reset, low-active, async
    ctrl_i      : in  ctrl_bus_t; -- main control bus
    -- memory data access interface --
    addr_i      : in  std_ulogic_vector(31 downto 0); -- access address
    wdata_i     : in  std_ulogic_vector(31 downto 0); -- write data
    rdata_o     : out std_ulogic_vector(31 downto 0); -- read data
    mar_o       : out std_ulogic_vector(31 downto 0); -- current memory address register
    wait_o      : out std_ulogic;                     -- wait for access to complete
    err_o       : out std_ulogic_vector(3 downto 0);  -- alignment/access errors
    pmp_fault_i : in  std_ulogic;                     -- PMP read/write access fault
    -- data bus --
    dbus_req_o  : out bus_req_t; -- request
    dbus_rsp_i  : in  bus_rsp_t  -- response
  );
end neorv32_cpu_lsu;

architecture neorv32_cpu_lsu_rtl of neorv32_cpu_lsu is

  signal req : bus_req_t;
  signal misalign : std_ulogic;

begin

  -- Atomic Memory Access -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  amo_enabled:
  if AMO_EN generate
    amo_reg: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        req.amo   <= '0';
        req.amoop <= (others => '0');
        req.lock  <= '0';
      elsif rising_edge(clk_i) then
        if (ctrl_i.lsu_mo_en = '1') then
          -- atomic memory access operation --
          req.amo <= ctrl_i.ir_opcode(2);
          case ctrl_i.ir_funct12(11 downto 7) is
            when "00010" => req.amoop <= "1000"; -- Zalrsc.LR
            when "00011" => req.amoop <= "1001"; -- Zalrsc.SC
            when "00000" => req.amoop <= "0001"; -- Zaamo.ADD
            when "00100" => req.amoop <= "0010"; -- Zaamo.XOR
            when "01100" => req.amoop <= "0011"; -- Zaamo.AND
            when "01000" => req.amoop <= "0100"; -- Zaamo.OR
            when "10000" => req.amoop <= "1110"; -- Zaamo.MIN
            when "10100" => req.amoop <= "1111"; -- Zaamo.MAX
            when "11000" => req.amoop <= "0110"; -- Zaamo.MINU
            when "11100" => req.amoop <= "0111"; -- Zaamo.MAXU
            when others  => req.amoop <= "0000"; -- Zaamo.SWAP
          end case;
        end if;
        -- bus locking for read-modify-write operations --
        if (ctrl_i.lsu_mo_en = '1') and (ctrl_i.ir_opcode(2) = '1') and (ctrl_i.ir_funct12(8) = '0') then
          req.lock <= '1'; -- set if atomic read-modify-write instruction
        elsif (dbus_rsp_i.ack = '1') or (ctrl_i.cpu_trap = '1') then
          req.lock <= '0'; -- clear at the end of the bus access
        end if;
      end if;
    end process amo_reg;
  end generate;

  -- no atomic memory operations --
  amo_disabled:
  if not AMO_EN generate
    req.amo   <= '0';
    req.amoop <= (others => '0');
    req.lock  <= '0';
  end generate;


  -- Request --------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_do_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      req.meta <= (others => '0');
      req.addr <= (others => '0');
      req.data <= (others => '0');
      req.ben  <= (others => '0');
      req.rw   <= '0';
      misalign <= '0';
    elsif rising_edge(clk_i) then
      if (ctrl_i.lsu_mo_en = '1') then
        req.meta <= std_ulogic_vector(to_unsigned(HART_ID, 2)) & ctrl_i.cpu_debug & ctrl_i.lsu_priv & '0';
        req.addr <= addr_i; -- memory address register
        case ctrl_i.ir_funct3(1 downto 0) is -- alignment + byte-enable
          when "00" => -- byte
            req.data   <= wdata_i(7 downto 0) & wdata_i(7 downto 0) & wdata_i(7 downto 0) & wdata_i(7 downto 0);
            req.ben(0) <= (not addr_i(1)) and (not addr_i(0));
            req.ben(1) <= (not addr_i(1)) and (    addr_i(0));
            req.ben(2) <= (    addr_i(1)) and (not addr_i(0));
            req.ben(3) <= (    addr_i(1)) and (    addr_i(0));
            misalign   <= '0';
          when "01" => -- half-word
            req.data <= wdata_i(15 downto 0) & wdata_i(15 downto 0);
            req.ben  <= addr_i(1) & addr_i(1) & (not addr_i(1)) & (not addr_i(1));
            misalign <= addr_i(0);
          when others => -- word
            req.data <= wdata_i;
            req.ben  <= (others => '1');
            misalign <= addr_i(1) or addr_i(0);
        end case;
        if AMO_EN and (ctrl_i.ir_opcode(2) = '1') and (ctrl_i.ir_funct12(8) = '0') then
          req.rw <= '0'; -- atomic read-modify-write operations are modified load requests
        else
          req.rw <= ctrl_i.lsu_wr;
        end if;
      end if;
    end if;
  end process mem_do_reg;

  -- direct output --
  req.burst <= '0'; -- only non-burst/single-accesses
  req.fence <= ctrl_i.lsu_fence;

  -- access request (all source signals are driven by registers) --
  req.stb <= ctrl_i.lsu_req and (not misalign) and (not pmp_fault_i);

  -- output bus request --
  dbus_req_o <= req;

  -- address feedback for MTVAL CSR --
  mar_o <= req.addr;


  -- Response -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_di_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      rdata_o <= (others => '0');
    elsif rising_edge(clk_i) then
      rdata_o <= (others => '0'); -- output zero if there is no pending memory request
      if (ctrl_i.lsu_mi_en = '1') then
        case ctrl_i.ir_funct3(1 downto 0) is
          when "00" => -- byte
            case req.addr(1 downto 0) is
              when "00"   => rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(7),  24) & dbus_rsp_i.data(7 downto 0);
              when "01"   => rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(15), 24) & dbus_rsp_i.data(15 downto 8);
              when "10"   => rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(23), 24) & dbus_rsp_i.data(23 downto 16);
              when others => rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(31), 24) & dbus_rsp_i.data(31 downto 24);
            end case;
          when "01" => -- half-word
            if (req.addr(1) = '0') then
              rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(15), 16) & dbus_rsp_i.data(15 downto 0);
            else
              rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(31), 16) & dbus_rsp_i.data(31 downto 16);
            end if;
          when others => -- word
            rdata_o <= dbus_rsp_i.data;
        end case;
      end if;
    end if;
  end process mem_di_reg;

  -- wait for bus response --
  wait_o <= not dbus_rsp_i.ack;

  -- access/alignment errors --
  -- [NOTE] AMOs will report load AND store exceptions. However, only the store exception will be reported due to its higher priority.
  -- [NOTE] ACK is ignored for the error response to shorten the bus system's critical path.
  err_o(0) <= ctrl_i.lsu_mi_en and ctrl_i.lsu_rd and misalign; -- misaligned load
  err_o(1) <= ctrl_i.lsu_mi_en and ctrl_i.lsu_rd and (dbus_rsp_i.err or pmp_fault_i); -- load access error
  err_o(2) <= ctrl_i.lsu_mi_en and ctrl_i.lsu_wr and misalign; -- misaligned store
  err_o(3) <= ctrl_i.lsu_mi_en and ctrl_i.lsu_wr and (dbus_rsp_i.err or pmp_fault_i); -- store access error

end neorv32_cpu_lsu_rtl;
