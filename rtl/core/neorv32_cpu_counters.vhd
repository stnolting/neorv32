-- ================================================================================ --
-- NEORV32 CPU - Hardware Counters                                                  --
-- -------------------------------------------------------------------------------- --
-- Implementing hardware counters/control for the following RISC-V ISA extensions:  --
-- + Zicntr:    Base Counters           -> [m]cycle[h]         + [m]instret[h]      --
-- + Zihpm:     Hardware Perf. Monitors -> [m]hpmcnt[3..15][h] + mhpmevent[3..15]   --
-- + Smcntrpmf: Counter Priv. Filtering -> mcyclecfg[h]        + minstretcfg[h]     --
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
    ZICNTR_EN    : boolean;               -- base counters
    ZIHPM_EN     : boolean;               -- hardware performance monitors
    SMCNTRPMF_EN : boolean;               -- counter privilege-mode filtering
    UMODE_EN     : boolean;               -- user-mode
    HPM_NUM      : natural range 0 to 13; -- number of implemented HPM counters (0..13)
    HPM_WIDTH    : natural range 0 to 64  -- total size of HPM counters (0..64)
  );
  port (
    -- global control --
    clk_i   : in  std_ulogic; -- global clock, rising edge
    rstn_i  : in  std_ulogic; -- global reset, low-active, async
    ctrl_i  : in  ctrl_bus_t; -- main control bus
    -- read back --
    rdata_o : out std_ulogic_vector(31 downto 0) -- read data
  );
end neorv32_cpu_counters;

architecture neorv32_cpu_counters_rtl of neorv32_cpu_counters is

  -- global access decoder --
  type cnt_we_t is array (0 to 15) of std_ulogic_vector(1 downto 0);
  signal cnt_we : cnt_we_t;
  signal cnt_acc, cfg_acc, inh_acc, pmf_acc : std_ulogic;
  signal sel, cnt_re, cfg_we, cfg_re : std_ulogic_vector(15 downto 0);

  -- counter increment control --
  signal inhibit, cnt_inc : std_ulogic_vector(15 downto 0);
  signal pmf_cy, pmf_ir, pmf_inh : std_ulogic_vector(1 downto 0);

  -- HPM read-backs --
  type hpmevent_t is array (3 to 15) of std_ulogic_vector(10 downto 0);
  type hpmcnt_t   is array (3 to 15) of std_ulogic_vector(63 downto 0);
  signal hpmevent, hpmevent_rd : hpmevent_t;
  signal hpmcnt_rd : hpmcnt_t;

  -- global CSR read-backs --
  signal rdata64, cycle_rd, time_rd, instret_rd, hpm_rd, inhibit_rd, pmf_rd : std_ulogic_vector(63 downto 0);

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
  pmf_acc <= '1' when SMCNTRPMF_EN and (ctrl_i.csr_addr(11 downto 2) = csr_minstretcfgh_c(11 downto 2)) else '0';

  -- global CSR read-back and subword select --
  rdata64 <= cycle_rd or time_rd or instret_rd or hpm_rd or inhibit_rd or pmf_rd;
  rdata_o <= rdata64(63 downto 32) when (ctrl_i.csr_addr(7) = '1') else rdata64(31 downto 0);


  -- Counter-Inhibit CSR --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cnt_inhibit: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      inhibit <= (others => '0');
    elsif rising_edge(clk_i) then
      -- base counters --
      if not ZICNTR_EN then
        inhibit(0) <= '0';
        inhibit(2) <= '0';
      elsif (inh_acc = '1') and (ctrl_i.csr_we = '1') then
        inhibit(0) <= ctrl_i.csr_wdata(0); -- [m]cycle[h]
        inhibit(2) <= ctrl_i.csr_wdata(2); -- [m]instret[h]
      end if;
      -- hardware performance monitors --
      if not ZIHPM_EN then
        inhibit(15 downto 3) <= (others => '0');
      elsif (inh_acc = '1') and (ctrl_i.csr_we = '1') then
        inhibit(15 downto 3) <= ctrl_i.csr_wdata(15 downto 3); -- [m]hpmcounter*[h]
      end if;
      -- unused --
      inhibit(1) <= '0'; -- [m]time[h] not implemented
    end if;
  end process cnt_inhibit;

  -- mcountinhibit read-back --
  inhibit_rd(15 downto 0)  <= inhibit when (inh_acc = '1') and (ctrl_i.csr_re = '1') else (others => '0');
  inhibit_rd(63 downto 16) <= (others => '0');


  -- Privilege-Mode Filtering ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  filtering_enabled:
  if SMCNTRPMF_EN and ZICNTR_EN generate
    pmf_reg: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        pmf_cy <= (others => '0');
        pmf_ir <= (others => '0');
      elsif rising_edge(clk_i) then
        if (pmf_acc = '1') and (ctrl_i.csr_we = '1') then
          if (ctrl_i.csr_addr(1 downto 0) = csr_mcyclecfgh_c(1 downto 0)) then
            pmf_cy <= ctrl_i.csr_wdata(30) & ctrl_i.csr_wdata(28);
          end if;
          if (ctrl_i.csr_addr(1 downto 0) = csr_minstretcfgh_c(1 downto 0)) then
            pmf_ir <= ctrl_i.csr_wdata(30) & ctrl_i.csr_wdata(28);
          end if;
        end if;
        -- no-user-mode override --
        if not UMODE_EN then
          pmf_cy(0) <= '0';
          pmf_ir(0) <= '0';
        end if;
      end if;
    end process pmf_reg;

    -- CSR read-back --
    pmf_read_back: process(pmf_acc, ctrl_i, pmf_cy, pmf_ir)
    begin
      pmf_rd <= (others => '0');
      if (pmf_acc = '1') and (ctrl_i.csr_re = '1') then
        if (ctrl_i.csr_addr(0) = '1') then -- mcyclecfgh
          pmf_rd(30) <= pmf_cy(1);
          pmf_rd(28) <= pmf_cy(0);
        else -- minstretcfgh
          pmf_rd(30) <= pmf_ir(1);
          pmf_rd(28) <= pmf_ir(0);
        end if;
      end if;
    end process pmf_read_back;

    -- counter-inhibit according to current privilege-mode --
    pmf_inh(0) <= pmf_cy(1) when (ctrl_i.cpu_priv = priv_mode_m_c) or (UMODE_EN = false) else pmf_cy(0);
    pmf_inh(1) <= pmf_ir(1) when (ctrl_i.cpu_priv = priv_mode_m_c) or (UMODE_EN = false) else pmf_ir(0);
  end generate;

  -- filtering disabled --
  filtering_disabled:
  if not (SMCNTRPMF_EN and ZICNTR_EN) generate
    pmf_cy  <= (others => '0');
    pmf_ir  <= (others => '0');
    pmf_rd  <= (others => '0');
    pmf_inh <= (others => '0');
  end generate;


  -- Counter Increment (no increment during debugging or if inhibited/filtered) -------------
  -- -------------------------------------------------------------------------------------------
  -- RISC-V base counter events --
  cnt_inc(0) <= ctrl_i.cnt_event(cnt_event_cy_c) and (not ctrl_i.cpu_debug) and (not inhibit(0)) and (not pmf_inh(0));
  cnt_inc(1) <= '0'; -- undefined
  cnt_inc(2) <= ctrl_i.cnt_event(cnt_event_ir_c) and (not ctrl_i.cpu_debug) and (not inhibit(2)) and (not pmf_inh(1));
  -- NEORV32-specific HPM events --
  event_gen:
  for i in 3 to 15 generate
    cnt_inc(i) <= or_reduce_f(ctrl_i.cnt_event and hpmevent(i)) and (not ctrl_i.cpu_debug) and (not inhibit(i));
  end generate;


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
      inc_i  => cnt_inc(0),
      we_i   => cnt_we(0),
      data_i => ctrl_i.csr_wdata,
      oe_i   => cnt_re(0),
      cnt_o  => cycle_rd
    );

    -- [m]time[h] --
    time_rd <= (others => '0'); -- not implemented

    -- [m]instret[h] --
    instret_inst: entity neorv32.neorv32_prim_cnt
    generic map (
      CWIDTH => 64
    )
    port map (
      clk_i  => clk_i,
      rstn_i => rstn_i,
      inc_i  => cnt_inc(2),
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

      -- [m]hpmcnt[3..15][h] --
      hpmcnt_inst: entity neorv32.neorv32_prim_cnt
      generic map (
        CWIDTH => HPM_WIDTH
      )
      port map (
        clk_i  => clk_i,
        rstn_i => rstn_i,
        inc_i  => cnt_inc(i),
        we_i   => cnt_we(i),
        data_i => ctrl_i.csr_wdata,
        oe_i   => cnt_re(i),
        cnt_o  => hpmcnt_rd(i)
      );

      -- mhpmevent[3..15] --
      hpmevent_reg: process(rstn_i, clk_i)
      begin
        if (rstn_i = '0') then
          hpmevent(i) <= (others => '0');
        elsif rising_edge(clk_i) then
          if (cfg_we(i) = '1') then
            hpmevent(i) <= ctrl_i.csr_wdata(10 downto 0);
          end if;
          hpmevent(i)(cnt_event_tm_c) <= '0'; -- time: not available
        end if;
      end process hpmevent_reg;
      hpmevent_rd(i) <= hpmevent(i) when (cfg_re(i) = '1') else (others => '0');

    end generate;

    -- terminate unused HPM slices --
    hpm_terminate_gen:
    for i in HPM_NUM+3 to 15 generate
      hpmcnt_rd(i)   <= (others => '0');
      hpmevent(i)    <= (others => '0');
      hpmevent_rd(i) <= (others => '0');
    end generate;

    -- read-back --
    hpm_read_back: process(hpmcnt_rd, hpmevent_rd)
      variable tmp_v : std_ulogic_vector(63 downto 0);
    begin
      tmp_v := (others => '0');
      for i in 3 to 15 loop
        tmp_v := tmp_v or hpmcnt_rd(i) or std_ulogic_vector(resize(unsigned(hpmevent_rd(i)), 64));
      end loop;
      hpm_rd <= tmp_v;
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
