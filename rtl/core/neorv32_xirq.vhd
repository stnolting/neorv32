-- ================================================================================ --
-- NEORV32 SoC - External Interrupt Controller (XIRQ)                               --
-- -------------------------------------------------------------------------------- --
-- Simple interrupt controller for platform (processor-external) interrupts. Up to  --
-- 32 channels are supported that get (optionally) prioritized into a single CPU    --
-- interrupt. Trigger type is programmable per channel by configuration registers.  --
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

entity neorv32_xirq is
  generic (
    XIRQ_NUM_CH : natural range 0 to 32 -- number of IRQ channels
  );
  port (
    clk_i     : in  std_ulogic; -- global clock line
    rstn_i    : in  std_ulogic; -- global reset line, low-active
    bus_req_i : in  bus_req_t;  -- bus request
    bus_rsp_o : out bus_rsp_t;  -- bus response
    xirq_i    : in  std_ulogic_vector(31 downto 0); -- external IRQ channels
    cpu_irq_o : out std_ulogic  -- CPU interrupt
  );
end neorv32_xirq;

architecture neorv32_xirq_rtl of neorv32_xirq is

  -- register addresses --
  constant addr_enable_c    : std_ulogic_vector(2 downto 0) := "000"; -- r/w: channel enable
  constant addr_pending_c   : std_ulogic_vector(2 downto 0) := "001"; -- r/w: pending IRQs
  constant addr_source_c    : std_ulogic_vector(2 downto 0) := "010"; -- r/w: source IRQ, ACK on write
  constant addr_ttype_c     : std_ulogic_vector(2 downto 0) := "011"; -- r/w: trigger type (level/edge)
  constant addr_tpolarity_c : std_ulogic_vector(2 downto 0) := "100"; -- r/w: trigger polarity (high/low or rising/falling)

  -- interface registers --
  signal irq_enable, nclr_pending, irq_type, irq_polarity : std_ulogic_vector(XIRQ_NUM_CH-1 downto 0);
  signal irq_source : std_ulogic_vector(4 downto 0);

  -- interrupt trigger --
  signal irq_sync, irq_sync2, irq_trig : std_ulogic_vector(XIRQ_NUM_CH-1 downto 0);

  -- interrupt buffer --
  signal irq_pending, irq_raw    : std_ulogic_vector(XIRQ_NUM_CH-1 downto 0);
  signal irq_fire,    irq_active : std_ulogic;

  -- priority encoder --
  type prio_enc_t is array (0 to XIRQ_NUM_CH-1) of std_ulogic_vector(4 downto 0);
  signal prio_enc : prio_enc_t;

begin

  -- Bus Access -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bus_rsp_o    <= rsp_terminate_c;
      nclr_pending <= (others => '0');
      irq_type     <= (others => '0');
      irq_polarity <= (others => '0');
      irq_enable   <= (others => '0');
    elsif rising_edge(clk_i) then
      -- defaults --
      bus_rsp_o.ack  <= bus_req_i.stb;
      bus_rsp_o.err  <= '0';
      bus_rsp_o.data <= (others => '0');
      nclr_pending   <= (others => '1');
      -- bus access --
      if (bus_req_i.stb = '1') then
        if (bus_req_i.rw = '1') then -- write access
          if (bus_req_i.addr(4 downto 2) = addr_enable_c) then -- channel-enable
            irq_enable <= bus_req_i.data(XIRQ_NUM_CH-1 downto 0);
          end if;
          if (bus_req_i.addr(4 downto 2) = addr_pending_c) then -- clear pending IRQs
            nclr_pending <= bus_req_i.data(XIRQ_NUM_CH-1 downto 0); -- set zero to clear pending IRQ
          end if;
          if (bus_req_i.addr(4 downto 2) = addr_ttype_c) then -- trigger type
            irq_type <= bus_req_i.data(XIRQ_NUM_CH-1 downto 0);
          end if;
          if (bus_req_i.addr(4 downto 2) = addr_tpolarity_c) then -- trigger polarity
            irq_polarity <= bus_req_i.data(XIRQ_NUM_CH-1 downto 0);
          end if;
        else -- read access
          case bus_req_i.addr(4 downto 2) is
            when addr_enable_c  => bus_rsp_o.data(XIRQ_NUM_CH-1 downto 0) <= irq_enable;   -- channel-enable
            when addr_pending_c => bus_rsp_o.data(XIRQ_NUM_CH-1 downto 0) <= irq_pending;  -- pending IRQs
            when addr_source_c  => bus_rsp_o.data(4 downto 0)             <= irq_source;   -- IRQ source
            when addr_ttype_c   => bus_rsp_o.data(XIRQ_NUM_CH-1 downto 0) <= irq_type;     -- trigger type
            when others         => bus_rsp_o.data(XIRQ_NUM_CH-1 downto 0) <= irq_polarity; -- trigger polarity
          end case;
        end if;
      end if;
    end if;
  end process bus_access;


  -- IRQ Trigger --------------------------------------------------------------
  -- -----------------------------------------------------------------------------
  synchronizer: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      irq_sync  <= (others => '0');
      irq_sync2 <= (others => '0');
    elsif rising_edge(clk_i) then
      irq_sync  <= xirq_i(XIRQ_NUM_CH-1 downto 0);
      irq_sync2 <= irq_sync;
    end if;
  end process synchronizer;

  -- trigger type select --
  irq_trigger_gen:
  for i in 0 to XIRQ_NUM_CH-1 generate
    irq_trigger: process(irq_sync, irq_sync2, irq_type, irq_polarity)
      variable sel_v : std_ulogic_vector(1 downto 0);
    begin
      sel_v := irq_type(i) & irq_polarity(i);
      case sel_v is
        when "00"   => irq_trig(i) <= not irq_sync(i); -- low-level
        when "01"   => irq_trig(i) <= irq_sync(i); -- high-level
        when "10"   => irq_trig(i) <= (not irq_sync(i)) and irq_sync2(i); -- falling-edge
        when "11"   => irq_trig(i) <= irq_sync(i) and (not irq_sync2(i)); -- rising-edge
        when others => irq_trig(i) <= '0';
      end case;
    end process irq_trigger;
  end generate;


  -- IRQ Buffer ---------------------------------------------------------------
  -- -----------------------------------------------------------------------------
  irq_buffer: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      irq_pending <= (others => '0');
    elsif rising_edge(clk_i) then
      irq_pending <= (irq_pending and nclr_pending) or irq_trig;
    end if;
  end process irq_buffer;

  -- filter enabled channels --
  irq_raw <= irq_pending and irq_enable;

  -- anyone firing? --
  irq_fire <= or_reduce_f(irq_raw);

  -- encode highest-priority source (structural code: mux-chain) --
  priority_encoder_gen:
  for i in 0 to XIRQ_NUM_CH-1 generate -- start with highest priority
    priority_encoder_gen_chain: -- inside chain
    if i < XIRQ_NUM_CH-1 generate
      prio_enc(i) <= std_ulogic_vector(to_unsigned(i, 5)) when (irq_raw(i) = '1') else prio_enc(i+1);
    end generate;
    priority_encoder_gen_last: -- end of chain
    if i = XIRQ_NUM_CH-1 generate
      prio_enc(XIRQ_NUM_CH-1) <= std_ulogic_vector(to_unsigned(XIRQ_NUM_CH-1, 5)); -- lowest priority
    end generate;
  end generate;


  -- IRQ Arbiter --------------------------------------------------------------
  -- -----------------------------------------------------------------------------
  irq_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      irq_active <= '0';
      irq_source <= (others => '0');
    elsif rising_edge(clk_i) then
      if (irq_active = '0') then -- no active IRQ
        irq_source <= prio_enc(0); -- get IRQ source
        if (irq_fire = '1') then
          irq_active <= '1';
        end if;
      elsif (bus_req_i.stb = '1') and (bus_req_i.rw = '1') and
            (bus_req_i.addr(4 downto 2) = addr_source_c) then -- acknowledge on write access
        irq_active <= '0';
      end if;
    end if;
  end process irq_arbiter;

  -- CPU interrupt --
  cpu_irq_o <= irq_active;


end neorv32_xirq_rtl;
