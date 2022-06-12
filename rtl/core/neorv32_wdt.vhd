-- #################################################################################################
-- # << NEORV32 - Watch Dog Timer (WDT) >>                                                         #
-- # ********************************************************************************************* #
-- # Watchdog counter to trigger an action if the CPU gets stuck. The internal timeout counter is  #
-- # 20-bit wide. If this counter overflows one of two possible actions is triggered: generate an  #
-- # IRQ or force a hardware reset of the system. A WDT action can also be triggered manually at   #
-- # any time by setting the FORCE bit.                                                            #
-- #                                                                                               #
-- # When writing to the control register the 16 MSB-aligned bits have to contain the "watchdog    #
-- # access password". Otherwise all write accesses are ignored.                                   #
-- #                                                                                               #
-- # Access to the control register can be permanently locked by setting the lock bit. This bit    #
-- # can only be cleared by a system reset.                                                        #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
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
  constant ctrl_enable_c  : natural :=  0; -- r/w: WDT enable
  constant ctrl_clksel0_c : natural :=  1; -- r/w: prescaler select bit 0
  constant ctrl_clksel1_c : natural :=  2; -- r/w: prescaler select bit 1
  constant ctrl_clksel2_c : natural :=  3; -- r/w: prescaler select bit 2
  constant ctrl_mode_c    : natural :=  4; -- r/w: 0: WDT timeout triggers interrupt, 1: WDT timeout triggers hard reset
  constant ctrl_rcause_c  : natural :=  5; -- r/-: cause of last action (reset/IRQ): 0=external reset, 1=watchdog overflow
  constant ctrl_reset_c   : natural :=  6; -- -/w: reset WDT if set
  constant ctrl_force_c   : natural :=  7; -- -/w: force WDT action
  constant ctrl_lock_c    : natural :=  8; -- r/w: lock access to control register when set
  constant ctrl_dben_c    : natural :=  9; -- r/w: allow WDT to continue operation even when CPU is in debug mode
  constant ctrl_half_c    : natural := 10; -- r/-: set if at least half of the max. timeout counter value has been reached
  constant ctrl_pause_c   : natural := 11; -- r/w: pause WDT when CPU is in sleep mode
  --
  constant ctrl_pwd_lsb_c : natural := 16; -- -/w: access password LSB
  constant ctrl_pwd_msb_c : natural := 31; -- -/w: access password MSB

  -- access password --
  constant pwd_c : std_ulogic_vector(15 downto 0) := x"CA36";

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal wren   : std_ulogic;
  signal rden   : std_ulogic;

  -- control register --
  type ctrl_t is record
    enable  : std_ulogic; -- 1=WDT enabled
    clk_sel : std_ulogic_vector(2 downto 0);
    mode    : std_ulogic; -- 0=trigger IRQ on overflow; 1=trigger hard reset on overflow
    rcause  : std_ulogic; -- cause of last system reset: '0' = external, '1' = watchdog
    reset   : std_ulogic; -- reset WDT
    enforce : std_ulogic; -- force action
    lock    : std_ulogic; -- lock control register
    dben    : std_ulogic; -- allow operation also if CPU is in  debug mode
    pause   : std_ulogic; -- allow operation even if CPU is in sleep mode
  end record;
  signal ctrl : ctrl_t;

  -- prescaler clock generator --
  signal prsc_tick : std_ulogic;

  -- password check --
  signal pwd_ok : std_ulogic;

  -- WDT core --
  signal wdt_cnt   : std_ulogic_vector(20 downto 0);
  signal hw_rst    : std_ulogic;
  signal cnt_en    : std_ulogic;
  signal cnt_en_ff : std_ulogic;

begin

  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = wdt_base_c(hi_abb_c downto lo_abb_c)) else '0';
  wren   <= acc_en and wren_i;
  rden   <= acc_en and rden_i;


  -- Write Access ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  write_access: process(rstn_int_i, clk_i)
  begin
    if (rstn_int_i = '0') then
      ctrl.reset   <= '0';
      ctrl.enforce <= '0';
      ctrl.enable  <= '0'; -- disable WDT
      ctrl.mode    <= '0';
      ctrl.clk_sel <= (others => '0');
      ctrl.lock    <= '0';
      ctrl.dben    <= '0';
      ctrl.pause   <= '0';
    elsif rising_edge(clk_i) then
      -- auto-clear reset and force flags --
      ctrl.reset   <= '0';
      ctrl.enforce <= '0';
      -- write access - only if password is OK --
      if (wren = '1') and (pwd_ok = '1') then
        ctrl.reset   <= data_i(ctrl_reset_c);
        ctrl.enforce <= data_i(ctrl_force_c);
        if (ctrl.lock = '0') then -- update configuration only if not locked
          ctrl.enable  <= data_i(ctrl_enable_c);
          ctrl.mode    <= data_i(ctrl_mode_c);
          ctrl.clk_sel <= data_i(ctrl_clksel2_c downto ctrl_clksel0_c);
          ctrl.lock    <= data_i(ctrl_lock_c);
          ctrl.dben    <= data_i(ctrl_dben_c);
          ctrl.pause   <= data_i(ctrl_pause_c);
        end if;
      end if;
    end if;
  end process write_access;

  -- password check --
  pwd_ok <= '1' when (data_i(ctrl_pwd_msb_c downto ctrl_pwd_lsb_c) = pwd_c) else '0';

  -- clock generator --
  clkgen_en_o <= ctrl.enable; -- enable clock generator
  prsc_tick   <= clkgen_i(to_integer(unsigned(ctrl.clk_sel))); -- clock enable tick


  -- Watchdog Timeout -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  wdt_counter: process(rstn_int_i, clk_i)
  begin
    if (rstn_int_i = '0') then
      cnt_en_ff <= '0';
      wdt_cnt   <= (others => '0');
      rstn_o    <= '1'; -- do NOT fire on reset!
    elsif rising_edge(clk_i) then
      cnt_en_ff <= cnt_en;
      if (ctrl.reset = '1') then -- watchdog reset
        wdt_cnt <= (others => '0');
      elsif (cnt_en_ff = '1') then
        wdt_cnt <= std_ulogic_vector(unsigned('0' & wdt_cnt(wdt_cnt'left-1 downto 0)) + 1);
      end if;
      rstn_o <= not hw_rst;
    end if;
  end process wdt_counter;

  -- WDT counter allowed to run? --
  cnt_en <= ctrl.enable and -- watchdog enabled
            prsc_tick and -- counter increment tick
            ((not cpu_debug_i) or ctrl.dben) and -- CPU not in debug mode or allowed to also run when in debug mode
            ((not cpu_sleep_i) or (not ctrl.pause)); -- pause watchdog when CPU is in sleep mode

  -- action triggers --
  irq_o  <= ctrl.enable and (wdt_cnt(wdt_cnt'left) or ctrl.enforce) and (not ctrl.mode); -- mode 0: IRQ
  hw_rst <= ctrl.enable and (wdt_cnt(wdt_cnt'left) or ctrl.enforce) and (    ctrl.mode); -- mode 1: RESET


  -- Reset Cause Indicator ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  reset_generator: process(rstn_ext_i, clk_i)
  begin
    if (rstn_ext_i = '0') then
      ctrl.rcause <= '0';
    elsif rising_edge(clk_i) then
      ctrl.rcause <= ctrl.rcause or hw_rst; -- sticky-set on WDT timeout/force
    end if;
  end process reset_generator;


  -- Read Access ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      ack_o  <= rden or (wren and pwd_ok); -- bus access (timeout) exception if write access with incorrect password
      data_o <= (others => '0');
      if (rden = '1') then
        data_o(ctrl_enable_c) <= ctrl.enable;
        data_o(ctrl_mode_c)   <= ctrl.mode;
        data_o(ctrl_rcause_c) <= ctrl.rcause;
        data_o(ctrl_clksel2_c downto ctrl_clksel0_c) <= ctrl.clk_sel;
        data_o(ctrl_lock_c)   <= ctrl.lock;
        data_o(ctrl_dben_c)   <= ctrl.dben;
        data_o(ctrl_half_c)   <= wdt_cnt(wdt_cnt'left-1);
        data_o(ctrl_pause_c)  <= ctrl.pause;
      end if;
    end if;
  end process read_access;


end neorv32_wdt_rtl;
