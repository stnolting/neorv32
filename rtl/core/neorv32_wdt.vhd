-- #################################################################################################
-- # << NEORV32 - Watch Dog Timer (WDT) >>                                                         #
-- # ********************************************************************************************* #
-- # "Bark and bite" Watchdog. The WDt will trigger a CPU interrupt when the internal 24-bit       #
-- # reaches half of the programmed timeout value ("bark") before generating a system-wide         #
-- # hardware reset  when it finally reaches the full timeout value ("bite"). The internal counter #
-- # increments at 1/4096 of the processor's main clock.                                           #
-- #                                                                                               #
-- # Access to the control register can be permanently inhibited by setting the lock bit. This bit #
-- # can only be cleared by a hardware reset.                                                      #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_wdt is
  port (
    clk_i       : in  std_ulogic; -- global clock line
    rstn_i      : in  std_ulogic; -- system reset, low-active, async
    rst_cause_i : in  std_ulogic_vector(1 downto 0); -- reset cause
    bus_req_i   : in  bus_req_t;  -- bus request
    bus_rsp_o   : out bus_rsp_t;  -- bus response
    cpu_debug_i : in  std_ulogic; -- CPU is in debug mode
    cpu_sleep_i : in  std_ulogic; -- CPU is in sleep mode
    clkgen_en_o : out std_ulogic; -- enable clock generator
    clkgen_i    : in  std_ulogic_vector(7 downto 0);
    irq_o       : out std_ulogic; -- timeout IRQ
    rstn_o      : out std_ulogic  -- timeout reset, low_active, sync
  );
end neorv32_wdt;

architecture neorv32_wdt_rtl of neorv32_wdt is

  -- Reset password --
  constant reset_pwd_c : std_ulogic_vector(31 downto 0) := x"709d1ab3";

  -- Control register bits --
  constant ctrl_enable_c      : natural :=  0; -- r/w: WDT enable
  constant ctrl_lock_c        : natural :=  1; -- r/w: lock write access to control register when set
  constant ctrl_dben_c        : natural :=  2; -- r/w: allow WDT to continue operation even when CPU is in debug mode
  constant ctrl_sen_c         : natural :=  3; -- r/w: allow WDT to continue operation even when CPU is in sleep mode
  constant ctrl_strict_c      : natural :=  4; -- r/w: force hardware reset if reset password is incorrect
  constant ctrl_rcause_lo_c   : natural :=  5; -- r/-: cause of last system reset - low
  constant ctrl_rcause_hi_c   : natural :=  6; -- r/-: cause of last system reset - high
  --
  constant ctrl_timeout_lsb_c : natural :=  8; -- r/w: timeout value LSB
  constant ctrl_timeout_msb_c : natural := 31; -- r/w: timeout value MSB

  -- control register --
  type ctrl_t is record
    enable  : std_ulogic;
    lock    : std_ulogic;
    dben    : std_ulogic;
    sen     : std_ulogic;
    strict  : std_ulogic;
    timeout : std_ulogic_vector(23 downto 0);
  end record;
  signal ctrl : ctrl_t;

  -- prescaler clock generator --
  signal prsc_tick : std_ulogic;

  -- timeout counter --
  signal cnt                 : std_ulogic_vector(23 downto 0); -- timeout counter
  signal cnt_started         : std_ulogic;
  signal cnt_inc, cnt_inc_ff : std_ulogic; -- increment counter when set
  signal timeout_rst         : std_ulogic;
  signal timeout_irq         : std_ulogic;

  -- misc --
  signal hw_rst      : std_ulogic;
  signal reset_wdt   : std_ulogic;
  signal reset_force : std_ulogic;

begin

  -- Host Access ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- write access --
  write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl.enable  <= '0'; -- disable WDT after reset
      ctrl.lock    <= '0'; -- unlock after reset
      ctrl.dben    <= '0';
      ctrl.sen     <= '0';
      ctrl.strict  <= '0';
      ctrl.timeout <= (others => '0');
      reset_wdt    <= '0';
      reset_force  <= '0';
    elsif rising_edge(clk_i) then
      -- defaults --
      reset_wdt   <= '0';
      reset_force <= '0';
      -- bus access --
      if (bus_req_i.we = '1') then
        if (bus_req_i.addr(2) = '0') then -- control register
          if (ctrl.lock = '0') then -- update configuration only if not locked
            ctrl.enable  <= bus_req_i.data(ctrl_enable_c);
            ctrl.lock    <= bus_req_i.data(ctrl_lock_c) and ctrl.enable; -- lock only if already enabled
            ctrl.dben    <= bus_req_i.data(ctrl_dben_c);
            ctrl.sen     <= bus_req_i.data(ctrl_sen_c);
            ctrl.strict  <= bus_req_i.data(ctrl_strict_c);
            ctrl.timeout <= bus_req_i.data(ctrl_timeout_msb_c downto ctrl_timeout_lsb_c);
          else -- write access attempt to locked CTRL register
            reset_force <= '1';
          end if;
        else -- reset timeout counter - password check
          if (bus_req_i.data(31 downto 0) = reset_pwd_c) then
            reset_wdt <= '1'; -- password correct
          else
            reset_force <= '1'; -- password incorrect
          end if;
        end if;
      end if;
    end if;
  end process write_access;

  -- read access --
  read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      bus_rsp_o.ack  <= bus_req_i.re or bus_req_i.we;
      bus_rsp_o.data <= (others => '0');
      if (bus_req_i.re = '1') then
        bus_rsp_o.data(ctrl_enable_c)                                <= ctrl.enable;
        bus_rsp_o.data(ctrl_lock_c)                                  <= ctrl.lock;
        bus_rsp_o.data(ctrl_dben_c)                                  <= ctrl.dben;
        bus_rsp_o.data(ctrl_sen_c)                                   <= ctrl.sen;
        bus_rsp_o.data(ctrl_rcause_hi_c downto ctrl_rcause_lo_c)     <= rst_cause_i;
        bus_rsp_o.data(ctrl_strict_c)                                <= ctrl.strict;
        bus_rsp_o.data(ctrl_timeout_msb_c downto ctrl_timeout_lsb_c) <= ctrl.timeout;
      end if;
    end if;
  end process read_access;

  -- no access error possible --
  bus_rsp_o.err <= '0';


  -- Timeout Counter ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  wdt_counter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      cnt_inc_ff  <= '0';
      cnt_started <= '0';
      cnt         <= (others => '0');
    elsif rising_edge(clk_i) then
      cnt_inc_ff  <= cnt_inc;
      cnt_started <= ctrl.enable and (cnt_started or prsc_tick); -- start with next clock tick
      if (ctrl.enable = '0') or (reset_wdt = '1') then -- watchdog disabled or reset with correct password
        cnt <= (others => '0');
      elsif (cnt_inc_ff = '1') then
        cnt <= std_ulogic_vector(unsigned(cnt) + 1);
      end if;
    end if;
  end process wdt_counter;

  -- clock generator --
  clkgen_en_o <= ctrl.enable; -- enable clock generator
  prsc_tick   <= clkgen_i(clk_div4096_c); -- clock enable tick

  -- valid counter increment? --
  cnt_inc <= '1' when ((prsc_tick = '1') and (cnt_started = '1')) and -- clock tick and started
                      ((cpu_debug_i = '0') or (ctrl.dben = '1'))  and -- not in debug mode or allowed to run in debug mode
                      ((cpu_sleep_i = '0') or (ctrl.sen = '1')) else '0'; -- not in sleep mode or allowed to run in sleep mode

  -- timeout detection --
  timeout_irq <= '1' when (cnt_started = '1') and (cnt = ('0' & ctrl.timeout(23 downto 1))) else '0'; -- half timeout value
  timeout_rst <= '1' when (cnt_started = '1') and (cnt =        ctrl.timeout(23 downto 0))  else '0'; -- full timeout value


  -- Event Generators -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  event_generator: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      irq_o  <= '0';
      hw_rst <= '0';
    elsif rising_edge(clk_i) then
      -- defaults --
      irq_o  <= '0';
      hw_rst <= '0';
      if (ctrl.enable = '1') then
        -- interrupt --
        if (timeout_irq = '1') and (prsc_tick = '1') then
          irq_o <= '1';
        end if;
        -- hardware reset --
        if ((timeout_rst = '1') and (prsc_tick = '1')) or -- timeout
           ((ctrl.strict = '1') and (reset_force = '1')) then -- strict mode and incorrect password / locked CTRL write attempt
          hw_rst <= '1';
        end if;
      end if;
    end if;
  end process event_generator;

  -- system-wide reset --
  rstn_o <= not hw_rst;


end neorv32_wdt_rtl;
