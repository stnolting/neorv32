-- ================================================================================ --
-- NEORV32 SoC - General Purpose Timer (GPTMR)                                      --
-- -------------------------------------------------------------------------------- --
-- Can operate in interval timer mode and/or in input capture mode.                 --
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

entity neorv32_gptmr is
  port (
    clk_i       : in  std_ulogic; -- global clock line
    rstn_i      : in  std_ulogic; -- global reset line, low-active
    bus_req_i   : in  bus_req_t;  -- bus request
    bus_rsp_o   : out bus_rsp_t;  -- bus response
    clkgen_en_o : out std_ulogic; -- enable clock generator
    clkgen_i    : in  std_ulogic_vector(7 downto 0);
    irq_o       : out std_ulogic; -- timer match interrupt
    capture_i   : in  std_ulogic  -- capture input
  );
end neorv32_gptmr;

architecture neorv32_gptmr_rtl of neorv32_gptmr is

  -- control register --
  constant ctrl_en_c     : natural := 0; -- r/w: global enable
  constant ctrl_prsc0_c  : natural := 1; -- r/w: clock prescaler select bit 0
  constant ctrl_prsc1_c  : natural := 2; -- r/w: clock prescaler select bit 1
  constant ctrl_prsc2_c  : natural := 3; -- r/w: clock prescaler select bit 2
  constant ctrl_irqm_c   : natural := 4; -- r/w: enable interrupt on timer match
  constant ctrl_irqc_c   : natural := 5; -- r/w: enable interrupt on capture trigger
  constant ctrl_rise_c   : natural := 6; -- r/w: capture on rising edge
  constant ctrl_fall_c   : natural := 7; -- r/w: capture on falling edge
  constant ctrl_filter_c : natural := 8; -- r/w: filter capture input signal
  --
  constant ctrl_trigm_c  : natural := 30; -- r/c: timer-match has fired, cleared by writing 0
  constant ctrl_trigc_c  : natural := 31; -- r/c: capture-trigger has fired, cleared by writing 0
  --
  signal ctrl : std_ulogic_vector(8 downto 0);

  -- trigger flags --
  signal trig_match, trig_capture : std_ulogic;

  -- timer core --
  type timer_t is record
    count    : std_ulogic_vector(31 downto 0); -- counter register
    thres    : std_ulogic_vector(31 downto 0); -- threshold register
    tick     : std_ulogic; -- clock generator tick
    match    : std_ulogic; -- count == thres
    match_ff : std_ulogic;
    trigger  : std_ulogic; -- match trigger (single-shot)
    cnt_we   : std_ulogic; -- write access to count register
  end record;
  signal timer : timer_t;

  -- sampling and capture logic --
  type capture_t is record
    sync    : std_ulogic_vector(1 downto 0); -- synchronizer (prevent metastability)
    sreg    : std_ulogic_vector(3 downto 0); -- input sampling
    state   : std_ulogic_vector(1 downto 0); -- state buffer to detect edges
    count   : std_ulogic_vector(31 downto 0); -- capture register
    trigger : std_ulogic; -- edge detector has been triggered
    rising  : std_ulogic; -- rising-edge detector
    falling : std_ulogic; -- falling-edge detector
  end record;
  signal capture : capture_t;

begin

  -- Bus Access -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bus_rsp_o.ack  <= '0';
      bus_rsp_o.err  <= '0';
      bus_rsp_o.data <= (others => '0');
      ctrl           <= (others => '0');
      trig_match     <= '0';
      trig_capture   <= '0';
      timer.cnt_we   <= '0';
      timer.thres    <= (others => '0');
    elsif rising_edge(clk_i) then
      -- defaults --
      bus_rsp_o.ack  <= bus_req_i.stb;
      bus_rsp_o.err  <= '0'; -- no access error possible
      bus_rsp_o.data <= (others => '0');
      timer.cnt_we   <= '0';

      -- IRQ trigger --
      trig_match   <= ctrl(ctrl_en_c) and ctrl(ctrl_irqm_c) and (trig_match or timer.trigger);
      trig_capture <= ctrl(ctrl_en_c) and ctrl(ctrl_irqc_c) and (trig_capture or capture.trigger);

      -- actual bus access --
      if (bus_req_i.stb = '1') then
        if (bus_req_i.rw = '1') then -- write access
          if (bus_req_i.addr(3 downto 2) = "00") then -- control register
            ctrl(ctrl_en_c)     <= bus_req_i.data(ctrl_en_c);
            ctrl(ctrl_prsc0_c)  <= bus_req_i.data(ctrl_prsc0_c);
            ctrl(ctrl_prsc1_c)  <= bus_req_i.data(ctrl_prsc1_c);
            ctrl(ctrl_prsc2_c)  <= bus_req_i.data(ctrl_prsc2_c);
            ctrl(ctrl_irqm_c)   <= bus_req_i.data(ctrl_irqm_c);
            ctrl(ctrl_irqc_c)   <= bus_req_i.data(ctrl_irqc_c);
            ctrl(ctrl_rise_c)   <= bus_req_i.data(ctrl_rise_c);
            ctrl(ctrl_fall_c)   <= bus_req_i.data(ctrl_fall_c);
            ctrl(ctrl_filter_c) <= bus_req_i.data(ctrl_filter_c);
            if (bus_req_i.data(ctrl_trigm_c) = '0') then -- clear by writing zero
              trig_match <= '0';
            end if;
            if (bus_req_i.data(ctrl_trigc_c) = '0') then -- clear by writing zero
              trig_capture <= '0';
            end if;
          end if;
          if (bus_req_i.addr(3 downto 2) = "01") then -- threshold register
            timer.thres <= bus_req_i.data;
          end if;
          if (bus_req_i.addr(3 downto 2) = "10") then -- counter register
            timer.cnt_we <= '1';
          end if;
        else -- read access
          case bus_req_i.addr(3 downto 2) is
            when "00" => -- control register
              bus_rsp_o.data(ctrl_en_c)     <= ctrl(ctrl_en_c);
              bus_rsp_o.data(ctrl_prsc0_c)  <= ctrl(ctrl_prsc0_c);
              bus_rsp_o.data(ctrl_prsc1_c)  <= ctrl(ctrl_prsc1_c);
              bus_rsp_o.data(ctrl_prsc2_c)  <= ctrl(ctrl_prsc2_c);
              bus_rsp_o.data(ctrl_irqm_c)   <= ctrl(ctrl_irqm_c);
              bus_rsp_o.data(ctrl_irqc_c)   <= ctrl(ctrl_irqc_c);
              bus_rsp_o.data(ctrl_rise_c)   <= ctrl(ctrl_rise_c);
              bus_rsp_o.data(ctrl_fall_c)   <= ctrl(ctrl_fall_c);
              bus_rsp_o.data(ctrl_filter_c) <= ctrl(ctrl_filter_c);
              --
              bus_rsp_o.data(ctrl_trigm_c)  <= trig_match;
              bus_rsp_o.data(ctrl_trigc_c)  <= trig_capture;
            when "01" => -- threshold register
              bus_rsp_o.data <= timer.thres;
            when "10" => -- counter register
              bus_rsp_o.data <= timer.count;
            when others => -- capture register
              bus_rsp_o.data <= capture.count;
          end case;
        end if;
      end if;
    end if;
  end process bus_access;

  -- interrupt request --
  irq_o <= trig_match or trig_capture;


  -- Timer Core -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  counter_core: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      timer.count    <= (others => '0');
      timer.match_ff <= '0';
    elsif rising_edge(clk_i) then
      if (timer.cnt_we = '1') then -- write access
        timer.count <= bus_req_i.data; -- data_i will remain stable for at least 1 cycle after WREN has returned to low
      elsif (ctrl(ctrl_en_c) = '1') and (timer.tick = '1') then -- enabled and clock tick
        if (timer.match = '1') then -- timer-match
          timer.count <= (others => '0');
        else
          timer.count <= std_ulogic_vector(unsigned(timer.count) + 1);
        end if;
      end if;
      timer.match_ff <= timer.match;
    end if;
  end process counter_core;

  -- counter = threshold? --
  timer.match <= '1' when (timer.count = timer.thres) and (ctrl(ctrl_en_c) = '1') else '0';

  -- match edge detector --
  timer.trigger <= '1' when (timer.match_ff = '0') and (timer.match = '1') else '0';

  -- clock select --
  clock_select: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      timer.tick <= '0';
    elsif rising_edge(clk_i) then
      timer.tick <= clkgen_i(to_integer(unsigned(ctrl(ctrl_prsc2_c downto ctrl_prsc0_c))));
    end if;
  end process clock_select;

  -- clock generator enable --
  clkgen_en_o <= ctrl(ctrl_en_c);


  -- Capture Control ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  signal_capture: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      capture.sync  <= (others => '0');
      capture.sreg  <= (others => '0');
      capture.state <= (others => '0');
      capture.count <= (others => '0');
    elsif rising_edge(clk_i) then
      -- synchronizer - prevent metastability --
      capture.sync <= capture.sync(0) & capture_i;

      -- sample shift register, running at reduced sample rate --
      if (clkgen_i(clk_div4_c) = '1') then
        capture.sreg <= capture.sreg(2 downto 0) & capture.sync(1);
      end if;

      -- sample state buffer --
      capture.state(1) <= capture.state(0);
      if (ctrl(ctrl_filter_c) = '0') then -- no filter, use synchronized input
        capture.state(0) <= capture.sync(1);
      else -- active filter: change state only if input is stable for 4 sample clocks
        if (capture.sreg = "1111") then
          capture.state(0) <= '1';
        elsif (capture.sreg = "0000") then
          capture.state(0) <= '0';
        else
          capture.state(0) <= capture.state(0);
        end if;
      end if;

      -- timer capture --
      if (ctrl(ctrl_en_c) = '1') and (capture.trigger = '1') then
        capture.count <= timer.count;
      end if;
    end if;
  end process signal_capture;

  -- edge detectors --
  capture.rising  <= '1' when (capture.state = "01") else '0';
  capture.falling <= '1' when (capture.state = "10") else '0';

  -- capture trigger --
  capture.trigger <= (ctrl(ctrl_rise_c) and capture.rising) or -- rising-edge trigger
                     (ctrl(ctrl_fall_c) and capture.falling); -- falling-edge trigger


end neorv32_gptmr_rtl;
