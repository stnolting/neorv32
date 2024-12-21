-- ================================================================================ --
-- NEORV32 SoC - External Interrupt Controller (XIRQ)                               --
-- -------------------------------------------------------------------------------- --
-- Simple interrupt controller for platform (processor-external) interrupts. Up to  --
-- 32 channels are supported that get prioritized into a single CPU interrupt.      --
-- Trigger type is programmable per-channel by configuration registers.             --
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
    NUM_CH : natural range 0 to 32 -- number of IRQ channels
  );
  port (
    clk_i     : in  std_ulogic; -- global clock line
    rstn_i    : in  std_ulogic; -- global reset line, low-active
    bus_req_i : in  bus_req_t;  -- bus request
    bus_rsp_o : out bus_rsp_t;  -- bus response
    xirq_i    : in  std_ulogic_vector(NUM_CH-1 downto 0); -- external IRQ channels
    cpu_irq_o : out std_ulogic  -- CPU interrupt
  );
end neorv32_xirq;

architecture neorv32_xirq_rtl of neorv32_xirq is

  -- register addresses --
  constant addr_eie_c  : std_ulogic_vector(1 downto 0) := "00"; -- r/w: channel enable
  constant addr_esc_c  : std_ulogic_vector(1 downto 0) := "01"; -- r/w: source IRQ, ACK on write
  constant addr_ttyp_c : std_ulogic_vector(1 downto 0) := "10"; -- r/w: trigger type (level/edge)
  constant addr_tpol_c : std_ulogic_vector(1 downto 0) := "11"; -- r/w: trigger polarity (high/low or rising/falling)

  -- configuration registers --
  signal irq_enable, irq_type, irq_polarity : std_ulogic_vector(NUM_CH-1 downto 0);

  -- interrupt trigger --
  signal irq_sync1, irq_sync2, irq_trig : std_ulogic_vector(NUM_CH-1 downto 0);

  -- pending interrupt(s) --
  signal irq_pending : std_ulogic_vector(NUM_CH-1 downto 0);

  -- priority encoder --
  type prio_enc_t is array (0 to NUM_CH-1) of std_ulogic_vector(4 downto 0);
  signal prio_enc : prio_enc_t;

  -- interrupt arbiter --
  signal irq_state  : std_ulogic_vector(1 downto 0);
  signal irq_source : std_ulogic_vector(4 downto 0);
  signal irq_clear  : std_ulogic_vector(31 downto 0);

begin

  -- Bus Access -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bus_rsp_o    <= rsp_terminate_c;
      irq_type     <= (others => '0');
      irq_polarity <= (others => '0');
      irq_enable   <= (others => '0');
    elsif rising_edge(clk_i) then
      -- defaults --
      bus_rsp_o.ack  <= bus_req_i.stb;
      bus_rsp_o.err  <= '0';
      bus_rsp_o.data <= (others => '0');
      -- bus access --
      if (bus_req_i.stb = '1') then
        if (bus_req_i.rw = '1') then -- write access
          if (bus_req_i.addr(3 downto 2) = addr_eie_c) then -- channel-enable
            irq_enable <= bus_req_i.data(NUM_CH-1 downto 0);
          end if;
          if (bus_req_i.addr(3 downto 2) = addr_ttyp_c) then -- trigger type
            irq_type <= bus_req_i.data(NUM_CH-1 downto 0);
          end if;
          if (bus_req_i.addr(3 downto 2) = addr_tpol_c) then -- trigger polarity
            irq_polarity <= bus_req_i.data(NUM_CH-1 downto 0);
          end if;
        else -- read access
          case bus_req_i.addr(3 downto 2) is
            when addr_eie_c => -- channel-enable
              bus_rsp_o.data(NUM_CH-1 downto 0) <= irq_enable;
            when addr_esc_c =>
              bus_rsp_o.data(31)         <= irq_state(1); -- active interrupt waiting for ACK
              bus_rsp_o.data(4 downto 0) <= irq_source; -- interrupt source (channel number)
            when addr_ttyp_c => -- trigger type
              bus_rsp_o.data(NUM_CH-1 downto 0) <= irq_type;
            when others => -- trigger polarity
              bus_rsp_o.data(NUM_CH-1 downto 0) <= irq_polarity;
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
      irq_sync1 <= (others => '0');
      irq_sync2 <= (others => '0');
    elsif rising_edge(clk_i) then
      irq_sync1 <= xirq_i(NUM_CH-1 downto 0);
      irq_sync2 <= irq_sync1;
    end if;
  end process synchronizer;

  -- trigger type select --
  irq_trigger_gen:
  for i in 0 to NUM_CH-1 generate
    irq_trigger: process(irq_sync1, irq_sync2, irq_type, irq_polarity)
      variable sel_v : std_ulogic_vector(1 downto 0);
    begin
      sel_v := irq_type(i) & irq_polarity(i);
      case sel_v is
        when "00"   => irq_trig(i) <= not irq_sync1(i); -- low-level
        when "01"   => irq_trig(i) <= irq_sync1(i); -- high-level
        when "10"   => irq_trig(i) <= (not irq_sync1(i)) and irq_sync2(i); -- falling-edge
        when "11"   => irq_trig(i) <= irq_sync1(i) and (not irq_sync2(i)); -- rising-edge
        when others => irq_trig(i) <= '0';
      end case;
    end process irq_trigger;
  end generate;


  -- Interrupt-Pending Buffer -------------------------------------------------
  -- -----------------------------------------------------------------------------
  irq_buffer: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      irq_pending <= (others => '0');
    elsif rising_edge(clk_i) then
      irq_pending <= irq_enable and ((irq_pending and (not irq_clear(NUM_CH-1 downto 0))) or irq_trig);
    end if;
  end process irq_buffer;


  -- Priority Encoder (structural code: mux-chain) ----------------------------
  -- -----------------------------------------------------------------------------
  priority_encoder_gen:
  for i in 0 to NUM_CH-1 generate -- start with highest priority (=0)
    priority_encoder_gen_chain: -- inside chain
    if i < NUM_CH-1 generate
      prio_enc(i) <= std_ulogic_vector(to_unsigned(i, 5)) when (irq_pending(i) = '1') else prio_enc(i+1);
    end generate;
    priority_encoder_gen_last: -- end of chain
    if i = NUM_CH-1 generate
      prio_enc(NUM_CH-1) <= std_ulogic_vector(to_unsigned(NUM_CH-1, 5)); -- lowest priority
    end generate;
  end generate;


  -- IRQ Arbiter --------------------------------------------------------------
  -- -----------------------------------------------------------------------------
  irq_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      irq_clear  <= (others => '0');
      irq_source <= (others => '0');
      irq_state  <= (others => '0');
    elsif rising_edge(clk_i) then
      irq_clear <= (others => '0'); -- default
      case irq_state is

        when "00" => -- wait for pending interrupt
          irq_source <= prio_enc(0); -- highest-priority channel
          if (or_reduce_f(irq_pending) = '1') then
            irq_state <= "01";
          end if;

        when "01" => -- clear triggering channel
          irq_clear(to_integer(unsigned(irq_source))) <= '1'; -- ACK/clear according pending bit
          irq_state <= "11";

        when others => -- wait for CPU acknowledge
          if (bus_req_i.stb = '1') and (bus_req_i.rw = '1') and (bus_req_i.addr(3 downto 2) = addr_esc_c) then -- acknowledge on write access
            irq_state <= "00";
          end if;

      end case;
    end if;
  end process irq_arbiter;

  -- CPU interrupt --
  cpu_irq_o <= irq_state(0);


end neorv32_xirq_rtl;
