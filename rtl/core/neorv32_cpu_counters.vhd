-- ================================================================================ --
-- NEORV32 CPU - Hardware Counters                                                  --
-- -------------------------------------------------------------------------------- --
-- Implementing hardware counters for the following RISC-V ISA extensions:          --
-- + Zicntr: Base Counters -> [m]cycle[h] + [m]instret[h] CSRs                      --
-- + Zihpm:  Hardware Performance Monitors -> [m]hpmcntx[h] + mhpmeventx CRSs       --
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

entity neorv32_cpu_counters is
  generic (
    ZICNTR_EN : boolean; -- implement base counters
    ZIHPM_EN  : boolean; -- implement hardware performance monitors
    HPM_NUM   : natural range 0 to 13; -- number of implemented HPM counters (0..13)
    HPM_WIDTH : natural range 0 to 64  -- total size of HPM counters (0..64)
  );
  port (
    -- global control --
    clk_i   : in  std_ulogic; -- global clock, rising edge
    rstn_i  : in  std_ulogic; -- global reset, low-active, async
    ctrl_i  : in  ctrl_bus_t; -- main control bus
    -- read back --
    rdata_o : out std_ulogic_vector(XLEN-1 downto 0) -- read data
  );
end neorv32_cpu_counters;

architecture neorv32_cpu_counters_rtl of neorv32_cpu_counters is

  -- global access decoder --
  type cnt_we_t is array (0 to 15) of std_ulogic_vector(1 downto 0);
  signal cnt_we : cnt_we_t;
  signal cnt_acc, cfg_acc, inh_acc : std_ulogic;
  signal sel, cnt_re, cfg_we, cfg_re : std_ulogic_vector(15 downto 0);

  -- counter increment control --
  signal inhibit, cnt_en : std_ulogic_vector(15 downto 0);

  -- individual HPM read-backs --
  type hpmevent_t is array (3 to 15) of std_ulogic_vector(11 downto 0);
  type hpmcnt_t   is array (3 to 15) of std_ulogic_vector(63 downto 0);
  signal hpmevent, hpmevent_rd : hpmevent_t;
  signal hpmcnt_rd : hpmcnt_t;

  -- read-backs --
  signal rdata, cycle_rd, time_rd, instret_rd, hpm_rd, inhibit_rd : std_ulogic_vector(63 downto 0);

begin

  -- Access Decoder -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_gen:
  for i in 0 to 15 generate
    sel(i)    <= '1' when (ctrl_i.csr_addr(3 downto 0) = std_ulogic_vector(to_unsigned(i, 4))) else '0';
    cnt_we(i)(0) <= cnt_acc and sel(i) and ctrl_i.csr_we and (not ctrl_i.csr_addr(7));
    cnt_we(i)(1) <= cnt_acc and sel(i) and ctrl_i.csr_we and (    ctrl_i.csr_addr(7));
    cnt_re(i)    <= cnt_acc and sel(i) and ctrl_i.csr_re;
    cfg_we(i)    <= cfg_acc and sel(i) and ctrl_i.csr_we;
    cfg_re(i)    <= cfg_acc and sel(i) and ctrl_i.csr_re;
  end generate;

  -- CSR access --
  cnt_acc <= '1' when ZICNTR_EN and ((ctrl_i.csr_addr(11 downto 5) = csr_cycle_c(11 downto 5)) or
                                     (ctrl_i.csr_addr(11 downto 5) = csr_mcycle_c(11 downto 5)) or
                                     (ctrl_i.csr_addr(11 downto 5) = csr_cycleh_c(11 downto 5)) or
                                     (ctrl_i.csr_addr(11 downto 5) = csr_mcycleh_c(11 downto 5))) else '0';
  cfg_acc <= '1' when ZIHPM_EN and (ctrl_i.csr_addr(11 downto 5) = csr_mhpmevent3_c(11 downto 5)) else '0';
  inh_acc <= '1' when (ctrl_i.csr_addr = csr_mcountinhibit_c) else '0';

  -- global data read-back and subword select --
  rdata   <= cycle_rd or time_rd or instret_rd or hpm_rd or inhibit_rd;
  rdata_o <= rdata(63 downto 32) when (ctrl_i.csr_addr(7) = '1') else rdata(31 downto 0);


  -- Counter Increment Control --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cnt_control: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      inhibit <= (others => '0');
      cnt_en  <= (others => '0');
    elsif rising_edge(clk_i) then
      -- mcountinhibit CSR write access --
      if (inh_acc = '1') and (ctrl_i.csr_we = '1') then
        if ZICNTR_EN then
          inhibit(0) <= ctrl_i.csr_wdata(0); -- [m]cycle[h]
          inhibit(2) <= ctrl_i.csr_wdata(2); -- [m]instret[h]
        end if;
        if ZIHPM_EN then
          inhibit((3+HPM_NUM)-1 downto 3) <= ctrl_i.csr_wdata((3+HPM_NUM)-1 downto 3); -- [m]hpmcounter*[h]
        end if;
      end if;
      -- counter (increment) enable --
      -- increment if an (enabled) event fires; do not increment if CPU is in debug mode or if counter is inhibited
      cnt_en(0) <= ctrl_i.cnt_event(cnt_event_cy_c) and (not ctrl_i.cpu_debug) and (not inhibit(0));
      cnt_en(1) <= '0'; -- time: not available
      cnt_en(2) <= ctrl_i.cnt_event(cnt_event_ir_c) and (not ctrl_i.cpu_debug) and (not inhibit(2));
      for i in 3 to 15 loop
        cnt_en(i) <= or_reduce_f(ctrl_i.cnt_event and hpmevent(i)) and (not ctrl_i.cpu_debug) and (not inhibit(i));
      end loop;
    end if;
  end process cnt_control;

  -- mcountinhibit read-back --
  inhibit_rd(63 downto 16) <= (others => '0');
  inhibit_rd(15 downto 0)  <= inhibit when (inh_acc = '1') and (ctrl_i.csr_re = '1') else (others => '0');


  -- Base Counters (Zicntr) -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  base_enabled:
  if ZICNTR_EN generate

    -- [m]cycle[h] --
    cycle_inst: entity neorv32.neorv32_prim_cnt
    generic map (
      CWIDTH => 64
    )
    port map (
      clk_i  => clk_i,
      rstn_i => rstn_i,
      inc_i  => cnt_en(0),
      we_i   => cnt_we(0),
      data_i => ctrl_i.csr_wdata,
      oe_i   => cnt_re(0),
      cnt_o  => cycle_rd
    );

    -- [m]time[h] --
    time_rd <= (others => '0'); -- not implemented (yet?)

    -- [m]instret[h] --
    instret_inst: entity neorv32.neorv32_prim_cnt
    generic map (
      CWIDTH => 64
    )
    port map (
      clk_i  => clk_i,
      rstn_i => rstn_i,
      inc_i  => cnt_en(2),
      we_i   => cnt_we(2),
      data_i => ctrl_i.csr_wdata,
      oe_i   => cnt_re(2),
      cnt_o  => instret_rd
    );

  end generate;

  -- base counters disabled --
  base_disabled:
  if not ZICNTR_EN generate
    cycle_rd   <= (others => '0');
    time_rd    <= (others => '0');
    instret_rd <= (others => '0');
  end generate;


  -- Hardware Performance Monitors (Zihpm) --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  hpm_enabled:
  if ZIHPM_EN and (HPM_NUM > 0) generate

    hpm_gen:
    for i in 3 to (HPM_NUM+3)-1 generate

      -- [m]hpmcntx[h] --
      hpmcnt_inst: entity neorv32.neorv32_prim_cnt
      generic map (
        CWIDTH => HPM_WIDTH
      )
      port map (
        clk_i  => clk_i,
        rstn_i => rstn_i,
        inc_i  => cnt_en(i),
        we_i   => cnt_we(i),
        data_i => ctrl_i.csr_wdata,
        oe_i   => cnt_re(i),
        cnt_o  => hpmcnt_rd(i)
      );

      -- mhpmeventx --
      hpmevent_reg: process(rstn_i, clk_i)
      begin
        if (rstn_i = '0') then
          hpmevent(i) <= (others => '0');
        elsif rising_edge(clk_i) then
          if (cfg_we(i) = '1') then
            hpmevent(i) <= ctrl_i.csr_wdata(11 downto 0);
          end if;
          hpmevent(i)(cnt_event_tm_c) <= '0'; -- time: not available
        end if;
      end process hpmevent_reg;
      hpmevent_rd(i) <= hpmevent(i) when (cfg_re(i) = '1') else (others => '0');

    end generate;

    -- terminate unused entries --
    hpm_terminate_gen:
    for i in HPM_NUM+3 to 15 generate
      hpmcnt_rd(i)   <= (others => '0');
      hpmevent(i)    <= (others => '0');
      hpmevent_rd(i) <= (others => '0');
    end generate;

    -- read-back --
    hpm_read_back: process(hpmcnt_rd, hpmevent_rd)
      variable cnt_v, cfg_v : std_ulogic_vector(63 downto 0);
    begin
      cnt_v := (others => '0');
      cfg_v := (others => '0');
      for i in 3 to 15 loop
        cnt_v := cnt_v or hpmcnt_rd(i);
        cfg_v := cfg_v or std_ulogic_vector(resize(unsigned(hpmevent_rd(i)), 64));
      end loop;
      hpm_rd <= cnt_v or cfg_v;
    end process hpm_read_back;

  end generate;

  -- HPMs disabled --
  hpm_disabled:
  if (not ZIHPM_EN) or (HPM_NUM = 0) generate
    hpmcnt_rd   <= (others => (others => '0'));
    hpmevent    <= (others => (others => '0'));
    hpmevent_rd <= (others => (others => '0'));
    hpm_rd      <= (others => '0');
  end generate;

end neorv32_cpu_counters_rtl;
