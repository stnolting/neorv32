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
    -- host access --
    clk_i       : in  std_ulogic; -- global clock line
    rstn_ext_i  : in  std_ulogic; -- external reset line, low-active, async
    rstn_int_i  : in  std_ulogic; -- internal reset line, low-active, async
    addr_i      : in  std_ulogic_vector(31 downto 0); -- address
    rden_i      : in  std_ulogic; -- read enable
    wren_i      : in  std_ulogic; -- write enable
    data_i      : in  std_ulogic_vector(31 downto 0); -- data in
    data_o      : out std_ulogic_vector(31 downto 0); -- data out
    ack_o       : out std_ulogic; -- transfer acknowledge
    -- CPU status --
    cpu_debug_i : in  std_ulogic; -- CPU is in debug mode
    cpu_sleep_i : in  std_ulogic; -- CPU is in sleep mode
    -- clock generator --
    clkgen_en_o : out std_ulogic; -- enable clock generator
    clkgen_i    : in  std_ulogic_vector(07 downto 0);
    -- timeout event --
    irq_o       : out std_ulogic; -- timeout IRQ
    rstn_o      : out std_ulogic  -- timeout reset, low_active, sync
  );
end neorv32_wdt;

architecture neorv32_wdt_rtl of neorv32_wdt is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(wdt_size_c); -- low address boundary bit

  -- Control register bits --
  constant ctrl_enable_c      : natural :=  0; -- r/w: WDT enable
  constant ctrl_lock_c        : natural :=  1; -- r/w: lock write access to control register when set
  constant ctrl_dben_c        : natural :=  2; -- r/w: allow WDT to continue operation even when CPU is in debug mode
  constant ctrl_sen_c         : natural :=  3; -- r/w: allow WDT to continue operation even when CPU is in sleep mode
  constant ctrl_reset_c       : natural :=  4; -- -/w: reset WDT if set ("feed" watchdog)
  constant ctrl_rcause_c      : natural :=  5; -- r/-: cause of last system reset: 0=external reset, 1=watchdog timeout
  --
  constant ctrl_timeout_lsb_c : natural :=  8; -- r/w: timeout value LSB
  constant ctrl_timeout_msb_c : natural := 31; -- r/w: timeout value MSB

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal wren   : std_ulogic;
  signal rden   : std_ulogic;

  -- control register --
  type ctrl_t is record
    enable  : std_ulogic; -- WDT enable
    lock    : std_ulogic; -- lock write access to control register when set
    dben    : std_ulogic; -- allow WDT to continue operation even when CPU is in debug mode
    sen     : std_ulogic; -- allow WDT to continue operation even when CPU is in sleep mode
    reset   : std_ulogic; -- reset WDT if set ("feed" watchdog)
    rcause  : std_ulogic; -- cause of last system reset: 0=external reset, 1=watchdog timeout
    timeout : std_ulogic_vector(23 downto 0); -- timeout value
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

  -- interrupt & reset generators --
  signal irq_gen_buf, hw_rstn : std_ulogic;

begin

  -- Host Access ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- access control --
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = wdt_base_c(hi_abb_c downto lo_abb_c)) else '0';
  wren   <= acc_en and wren_i;
  rden   <= acc_en and rden_i;

  -- write access --
  write_access: process(rstn_int_i, clk_i)
  begin
    if (rstn_int_i = '0') then
      ctrl.enable  <= '0'; -- disable WDT after reset
      ctrl.lock    <= '0'; -- unlock after reset
      ctrl.dben    <= '0';
      ctrl.sen     <= '0';
      ctrl.reset   <= '0';
      ctrl.timeout <= (others => '0');
    elsif rising_edge(clk_i) then
      ctrl.reset <= '0'; -- default
      if (wren = '1') then
        ctrl.reset <= data_i(ctrl_reset_c);
        if (ctrl.lock = '0') then -- update configuration only if not locked
          ctrl.enable  <= data_i(ctrl_enable_c);
          ctrl.lock    <= data_i(ctrl_lock_c) and ctrl.enable; -- lock only if already enabled
          ctrl.dben    <= data_i(ctrl_dben_c);
          ctrl.sen     <= data_i(ctrl_sen_c);
          ctrl.timeout <= data_i(ctrl_timeout_msb_c downto ctrl_timeout_lsb_c);
        end if;
      end if;
    end if;
  end process write_access;

  -- read access --
  read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      ack_o  <= rden or wren;
      data_o <= (others => '0');
      if (rden = '1') then
        data_o(ctrl_enable_c)                                <= ctrl.enable;
        data_o(ctrl_lock_c)                                  <= ctrl.lock;
        data_o(ctrl_dben_c)                                  <= ctrl.dben;
        data_o(ctrl_sen_c)                                   <= ctrl.sen;
        data_o(ctrl_rcause_c)                                <= ctrl.rcause;
        data_o(ctrl_timeout_msb_c downto ctrl_timeout_lsb_c) <= ctrl.timeout;
      end if;
    end if;
  end process read_access;

  -- reset cause indicator --
  reset_cause: process(rstn_ext_i, clk_i)
  begin
    if (rstn_ext_i = '0') then
      ctrl.rcause <= '0';
    elsif rising_edge(clk_i) then
      ctrl.rcause <= ctrl.rcause or (not hw_rstn); -- sticky-set on WDT timeout/force
    end if;
  end process reset_cause;


  -- Timeout Counter ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  wdt_counter: process(clk_i)
  begin
    if rising_edge(clk_i) then
      cnt_inc_ff  <= cnt_inc;
      cnt_started <= ctrl.enable and (cnt_started or prsc_tick); -- set with next clock tick
      if (ctrl.enable = '0') or (ctrl.reset = '1') then -- watchdog disabled or reset
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
  cnt_inc <= '1' when (prsc_tick = '1') and (cnt_started = '1') and -- clock tick and started
                      ((cpu_debug_i = '0') or (ctrl.dben = '1')) and -- not in debug mode or allowed to run in debug mode
                      ((cpu_sleep_i = '0') or (ctrl.sen = '1')) else '0'; -- not in sleep mode or allowed to run in sleep mode

  -- timeout detection --
  timeout_irq <= '1' when (cnt_started = '1') and (cnt = ('0' & ctrl.timeout(23 downto 1))) else '0'; -- half timeout value
  timeout_rst <= '1' when (cnt_started = '1') and (cnt = ctrl.timeout(23 downto 0)) else '0'; -- full timeout value


  -- Event Generators -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- interrupt --
  irq_trigger: process(clk_i)
  begin
    if rising_edge(clk_i) then
      irq_gen_buf <= timeout_irq;
      if (irq_gen_buf = '0') and (timeout_irq = '1') and -- rising edge detector
         (ctrl.enable = '1') and (timeout_rst = '0') then -- enabled and not a HW reset
        irq_o <= '1';
      else
        irq_o <= '0';
      end if;
    end if;
  end process irq_trigger;

  -- hardware reset --
  rst_trigger: process(rstn_int_i, clk_i)
  begin
    if (rstn_int_i = '0') then
      hw_rstn <= '1';
    elsif rising_edge(clk_i) then
      if (ctrl.enable = '1') and (timeout_rst = '1') then
        hw_rstn <= '0';
      else
        hw_rstn <= '1';
      end if;
    end if;
  end process rst_trigger;

  -- system wide reset --
  rstn_o <= hw_rstn;


end neorv32_wdt_rtl;
