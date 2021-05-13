-- #################################################################################################
-- # << NEORV32 - Watch Dog Timer (WDT) >>                                                         #
-- # ********************************************************************************************* #
-- # Watchdog counter to trigger an action if the CPU gets stuck.                                  #
-- # The internal counter is 20-bit wide. If this counter overflows one of two possible actions is #
-- # triggered: Generate an IRQ or force a hardware reset of the system.                           #
-- # A WDT action can also be triggered manually at any time by setting the FORCE bit.             #
-- #                                                                                               #
-- # Access to the control register can be permanently locked by setting the lock bit. This bit    #
-- # can only be cleared by a hardware reset (external or caused by the watchdog itself).          #
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
    rstn_i      : in  std_ulogic; -- global reset line, low-active
    addr_i      : in  std_ulogic_vector(31 downto 0); -- address
    rden_i      : in  std_ulogic; -- read enable
    wren_i      : in  std_ulogic; -- write enable
    data_i      : in  std_ulogic_vector(31 downto 0); -- data in
    data_o      : out std_ulogic_vector(31 downto 0); -- data out
    ack_o       : out std_ulogic; -- transfer acknowledge
    -- clock generator --
    clkgen_en_o : out std_ulogic; -- enable clock generator
    clkgen_i    : in  std_ulogic_vector(07 downto 0);
    -- timeout event --
    irq_o       : out std_ulogic; -- timeout IRQ
    rstn_o      : out std_ulogic  -- timeout reset, low_active, use as async
  );
end neorv32_wdt;

architecture neorv32_wdt_rtl of neorv32_wdt is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(wdt_size_c); -- low address boundary bit

  -- Control register bits --
  constant ctrl_enable_c  : natural := 0; -- r/w: WDT enable
  constant ctrl_clksel0_c : natural := 1; -- r/w: prescaler select bit 0
  constant ctrl_clksel1_c : natural := 2; -- r/w: prescaler select bit 1
  constant ctrl_clksel2_c : natural := 3; -- r/w: prescaler select bit 2
  constant ctrl_mode_c    : natural := 4; -- r/w: 0: WDT timeout triggers interrupt, 1: WDT timeout triggers hard reset
  constant ctrl_rcause_c  : natural := 5; -- r/-: cause of last action (reset/IRQ): 0=external reset, 1=watchdog overflow
  constant ctrl_reset_c   : natural := 6; -- -/w: reset WDT if set
  constant ctrl_force_c   : natural := 7; -- -/w: force WDT action
  constant ctrl_lock_c    : natural := 8; -- r/w: lock access to control register when set

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal wren   : std_ulogic;
  signal rden   : std_ulogic;

  -- control register --
  type ctrl_reg_t is record
    enable  : std_ulogic; -- 1=WDT enabled
    clk_sel : std_ulogic_vector(2 downto 0);
    mode    : std_ulogic; -- 0=trigger IRQ on overflow; 1=trigger hard reset on overflow
    rcause  : std_ulogic; -- cause of last system reset: '0' = external, '1' = watchdog
    reset   : std_ulogic; -- reset WDT
    enforce : std_ulogic; -- force action
    lock    : std_ulogic; -- lock control register
  end record;
  signal ctrl_reg : ctrl_reg_t;

  -- prescaler clock generator --
  signal prsc_tick : std_ulogic;

  -- WDT core --
  signal wdt_cnt : std_ulogic_vector(20 downto 0);
  signal hw_rst  : std_ulogic;
  signal rst_gen : std_ulogic_vector(03 downto 0);

  -- internal reset (sync, low-active) --
  signal rstn_sync : std_ulogic;

begin

  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = wdt_base_c(hi_abb_c downto lo_abb_c)) else '0';
  wren   <= acc_en and wren_i;
  rden   <= acc_en and rden_i;


  -- Write Access ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl_reg.reset   <= '0';
      ctrl_reg.enforce <= '0';
      ctrl_reg.enable  <= '0'; -- disable WDT
      ctrl_reg.mode    <= '0'; -- trigger interrupt on WDT overflow
      ctrl_reg.clk_sel <= (others => '1'); -- slowest clock source
      ctrl_reg.lock    <= '0';
    elsif rising_edge(clk_i) then
      if (rstn_sync = '0') then -- internal reset
        ctrl_reg.reset   <= '0';
        ctrl_reg.enforce <= '0';
        ctrl_reg.enable  <= '0'; -- disable WDT
        ctrl_reg.mode    <= '0'; -- trigger interrupt on WDT overflow
        ctrl_reg.clk_sel <= (others => '1'); -- slowest clock source
        ctrl_reg.lock    <= '0';
      else
        -- auto-clear WDT reset and WDT force flags --
        ctrl_reg.reset   <= '0';
        ctrl_reg.enforce <= '0';
        -- actual write access --
        if (wren = '1') then
          ctrl_reg.reset   <= data_i(ctrl_reset_c);
          ctrl_reg.enforce <= data_i(ctrl_force_c);
          if (ctrl_reg.lock = '0') then -- update configuration only if unlocked
            ctrl_reg.enable  <= data_i(ctrl_enable_c);
            ctrl_reg.mode    <= data_i(ctrl_mode_c);
            ctrl_reg.clk_sel <= data_i(ctrl_clksel2_c downto ctrl_clksel0_c);
            ctrl_reg.lock    <= data_i(ctrl_lock_c);
          end if;
        end if;
      end if;
    end if;
  end process write_access;

  -- clock generator --
  clkgen_en_o <= ctrl_reg.enable; -- enable clock generator
  prsc_tick   <= clkgen_i(to_integer(unsigned(ctrl_reg.clk_sel))); -- clock enable tick


  -- Watchdog Counter -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  wdt_counter: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (ctrl_reg.reset = '1') then -- watchdog reset
        wdt_cnt <= (others => '0');
      elsif (ctrl_reg.enable = '1') and (prsc_tick = '1') then
        wdt_cnt <= std_ulogic_vector(unsigned(wdt_cnt) + 1);
      end if;
    end if;
  end process wdt_counter;

  -- action trigger --
  irq_o  <= ctrl_reg.enable and (wdt_cnt(wdt_cnt'left) or ctrl_reg.enforce) and (not ctrl_reg.mode); -- mode 0: IRQ
  hw_rst <= ctrl_reg.enable and (wdt_cnt(wdt_cnt'left) or ctrl_reg.enforce) and (    ctrl_reg.mode); -- mode 1: RESET


  -- Reset Generator & Action Cause Indicator -----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  reset_generator: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl_reg.rcause <= '0';
      rst_gen         <= (others => '1'); -- do NOT fire on reset!
      rstn_sync       <= '1';
    elsif rising_edge(clk_i) then
      ctrl_reg.rcause <= ctrl_reg.rcause or hw_rst; -- sticky-set on WDT timeout/force
      if (hw_rst = '1') then
        rst_gen <= (others => '0');
      else
        rst_gen <= rst_gen(rst_gen'left-1 downto 0) & '1';
      end if;
      rstn_sync <= rst_gen(rst_gen'left);
    end if;
  end process reset_generator;

  -- system reset --
  rstn_o <= rst_gen(rst_gen'left);


  -- Read Access ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      ack_o  <= rden or wren;
      if (rden = '1') then
        data_o(ctrl_enable_c) <= ctrl_reg.enable;
        data_o(ctrl_mode_c)   <= ctrl_reg.mode;
        data_o(ctrl_rcause_c) <= ctrl_reg.rcause;
        data_o(ctrl_clksel2_c downto ctrl_clksel0_c) <= ctrl_reg.clk_sel;
        data_o(ctrl_lock_c)   <= ctrl_reg.lock;
      else
        data_o <= (others => '0');
      end if;
    end if;
  end process read_access;


end neorv32_wdt_rtl;
