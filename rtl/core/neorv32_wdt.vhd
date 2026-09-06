-- ================================================================================ --
-- NEORV32 SoC - Watch Dog Timer (WDT)                                              --
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

entity neorv32_wdt is
  port (
    clk_i      : in  std_ulogic;                    -- global clock line
    rstn_ext_i : in  std_ulogic;                    -- external reset, low-active
    rstn_dbg_i : in  std_ulogic;                    -- debugger reset, low-active
    rstn_sys_i : in  std_ulogic;                    -- system reset, low-active
    bus_req_i  : in  bus_req_t;                     -- bus request
    bus_rsp_o  : out bus_rsp_t;                     -- bus response
    clkgen_i   : in  std_ulogic_vector(7 downto 0); -- prescaled clock enables
    rstn_o     : out std_ulogic                     -- timeout reset, low_active, sync
  );
end entity;

architecture neorv32_wdt_rtl of neorv32_wdt is

  -- Reset password --
  constant reset_pwd_c : std_ulogic_vector(31 downto 0) := x"709d1ab3";

  -- Control register bits --
  constant ctrl_enable_c      : natural :=  0; -- r/w: WDT enable
  constant ctrl_lock_c        : natural :=  1; -- r/w: lock write access to control register when set
  constant ctrl_rcause_lsb_c  : natural :=  2; -- r/-: cause of last system reset, bit 0, LSB
  constant ctrl_rcause_msb_c  : natural :=  3; -- r/-: cause of last system reset, bit 1, MSB
  constant ctrl_timeout_lsb_c : natural :=  8; -- r/w: timeout value, bit 0, LSB
  constant ctrl_timeout_msb_c : natural := 31; -- r/w: timeout value, bit 23, MSB

  -- control register --
  type ctrl_t is record
    enable   : std_ulogic;
    lock     : std_ulogic;
    timeout  : std_ulogic_vector(23 downto 0);
    rst_time : std_ulogic; -- reset timeout counter
    rst_trig : std_ulogic; -- trigger system reset
  end record;
  signal ctrl : ctrl_t; -- register set

  -- misc --
  signal bus_ack        : std_ulogic; -- access acknowledge
  signal bus_rden       : std_ulogic; -- bus read access
  signal bus_rdata      : std_ulogic_vector(31 downto 0); -- bus read data
  signal prsc_tick      : std_ulogic; -- prescaler clock generator
  signal cen            : std_ulogic; -- counter enable
  signal cnt            : std_ulogic_vector(23 downto 0); -- timeout counter
  signal hw_rst_timeout : std_ulogic; -- trigger reset because of timeout
  signal hw_rst_access  : std_ulogic; -- trigger reset because of illegal access if locked
  signal reset_cause    : std_ulogic_vector(1 downto 0); -- cause of last reset

begin

  -- Bus Access -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_handshake: process(rstn_sys_i, clk_i)
  begin
    if (rstn_sys_i = '0') then
      bus_ack  <= '0';
      bus_rden <= '0';
    elsif rising_edge(clk_i) then
      bus_ack  <= bus_req_i.stb;
      bus_rden <= bus_req_i.stb and (not bus_req_i.rw);
    end if;
  end process;

  -- write access --
  bus_write: process(rstn_sys_i, clk_i)
  begin
    if (rstn_sys_i = '0') then
      ctrl.enable   <= '0'; -- disable WDT after reset
      ctrl.lock     <= '0'; -- unlock after reset
      ctrl.timeout  <= (others => '0');
      ctrl.rst_time <= '0';
      ctrl.rst_trig <= '0';
    elsif rising_edge(clk_i) then
      ctrl.rst_time <= '0'; -- default
      ctrl.rst_trig <= '0'; -- default
      if (bus_req_i.stb = '1') and (bus_req_i.rw = '1') then
        if (bus_req_i.addr(2) = '0') then -- control register
          if (ctrl.lock = '0') then -- update configuration only if not locked
            ctrl.enable  <= bus_req_i.data(ctrl_enable_c);
            ctrl.lock    <= bus_req_i.data(ctrl_lock_c);
            ctrl.timeout <= bus_req_i.data(ctrl_timeout_msb_c downto ctrl_timeout_lsb_c);
          else -- write access attempt to locked CTRL register
            ctrl.rst_trig <= '1';
          end if;
        else -- reset timeout counter - password check
          if (bus_req_i.data(31 downto 0) = reset_pwd_c) then
            ctrl.rst_time <= '1'; -- correct: reset timer
          else
            ctrl.rst_trig <= '1'; -- incorrect: trigger system reset
          end if;
        end if;
      end if;
    end if;
  end process;

  -- read access (asynchronous) --
  bus_read: process(bus_rden, ctrl, reset_cause)
  begin
    bus_rdata <= (others => '0');
    if (bus_rden = '1') then -- output gating
      bus_rdata(ctrl_enable_c)                                <= ctrl.enable;
      bus_rdata(ctrl_lock_c)                                  <= ctrl.lock;
      bus_rdata(ctrl_rcause_msb_c  downto ctrl_rcause_lsb_c)  <= reset_cause;
      bus_rdata(ctrl_timeout_msb_c downto ctrl_timeout_lsb_c) <= ctrl.timeout;
    end if;
  end process;

  -- bus response --
  bus_rsp_o.ack  <= bus_ack;
  bus_rsp_o.err  <= '0'; -- no access errors supported
  bus_rsp_o.data <= bus_rdata;

  -- Timeout Counter ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  wdt_counter: process(rstn_sys_i, clk_i)
  begin
    if (rstn_sys_i = '0') then
      cen <= '0';
      cnt <= (others => '0');
    elsif rising_edge(clk_i) then
      cen <= ctrl.enable; -- delay start by 1 cycle to make sure cnt is initialized by ctrl.timeout
      if (cen = '0') or (ctrl.enable = '0') or (ctrl.rst_time = '1') then -- watchdog disabled or reset
        cnt <= ctrl.timeout;
      elsif (prsc_tick = '1') then
        cnt <= std_ulogic_vector(unsigned(cnt) - 1);
      end if;
    end if;
  end process;

  -- countdown timer tick --
  prsc_tick <= clkgen_i(clk_div4096_c); -- clock-enable tick at fixed clock rate

  -- Reset Generator ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  reset_generator: process(rstn_sys_i, clk_i)
  begin
    if (rstn_sys_i = '0') then
      hw_rst_timeout <= '0';
      hw_rst_access  <= '0';
      rstn_o         <= '1';
    elsif rising_edge(clk_i) then -- reset triggers are sticky until system reset
      hw_rst_timeout <= hw_rst_timeout or (ctrl.enable and cen and prsc_tick and (not or_reduce_f(cnt))); -- timeout
      hw_rst_access  <= hw_rst_access  or (ctrl.enable and ctrl.lock and ctrl.rst_trig); -- locked and incorrect password
      rstn_o         <= not (hw_rst_timeout or hw_rst_access); -- system-wide reset
    end if;
  end process;

  -- Reset-Cause Indicator ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  reset_identifier: process(rstn_ext_i, clk_i)
  begin
    if (rstn_ext_i = '0') then
      reset_cause <= "00"; -- reset from external hardware signal
    elsif rising_edge(clk_i) then
      if (hw_rst_timeout = '1') then
        reset_cause <= "10"; -- reset from watchdog timer
      elsif (hw_rst_access = '1') then
        reset_cause <= "11"; -- reset from invalid watchdog access (locked or incorrect password)
      elsif (rstn_dbg_i = '0') then
        reset_cause <= "01"; -- reset from on-chip debugger
      end if;
    end if;
  end process;

end architecture;
