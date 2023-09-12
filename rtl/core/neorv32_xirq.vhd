-- #################################################################################################
-- # << NEORV32 - External Interrupt Controller (XIRQ) >>                                          #
-- # ********************************************************************************************* #
-- # Simple interrupt controller for platform (processor-external) interrupts. Up to 32 channels   #
-- # are supported that get (optionally) prioritized into a single CPU interrupt.                  #
-- #                                                                                               #
-- # The actual trigger configuration has to be done BEFORE synthesis using the XIRQ_TRIGGER_TYPE  #
-- # and XIRQ_TRIGGER_POLARITY generics. These allow to configure channel-independent low-level,   #
-- # high-level, falling-edge and rising-edge triggers.                                            #
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

entity neorv32_xirq is
  generic (
    XIRQ_NUM_CH           : natural range 0 to 32; -- number of external IRQ channels
    XIRQ_TRIGGER_TYPE     : std_ulogic_vector(31 downto 0); -- trigger type: 0=level, 1=edge
    XIRQ_TRIGGER_POLARITY : std_ulogic_vector(31 downto 0)  -- trigger polarity: 0=low-level/falling-edge, 1=high-level/rising-edge
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

  -- interface registers --
  signal irq_enable   : std_ulogic_vector(XIRQ_NUM_CH-1 downto 0); -- r/w: channel enable
  signal nclr_pending : std_ulogic_vector(XIRQ_NUM_CH-1 downto 0); -- r/w: pending IRQs
  signal irq_source   : std_ulogic_vector(4 downto 0); -- r/w: source IRQ, ACK on write

  -- interrupt trigger --
  signal irq_sync  : std_ulogic_vector(XIRQ_NUM_CH-1 downto 0);
  signal irq_sync2 : std_ulogic_vector(XIRQ_NUM_CH-1 downto 0);
  signal irq_trig  : std_ulogic_vector(XIRQ_NUM_CH-1 downto 0);

  -- interrupt buffer --
  signal irq_pending : std_ulogic_vector(XIRQ_NUM_CH-1 downto 0);
  signal irq_raw     : std_ulogic_vector(XIRQ_NUM_CH-1 downto 0);
  signal irq_fire    : std_ulogic;

  -- interrupt arbiter --
  signal irq_source_nxt : std_ulogic_vector(4 downto 0);
  signal irq_active     : std_ulogic;

begin

  -- Host Access ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- write access --
  write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      nclr_pending <= (others => '0'); -- clear all pending interrupts on reset
      irq_enable   <= (others => '0');
    elsif rising_edge(clk_i) then
      nclr_pending <= (others => '1');
      if (bus_req_i.we = '1') then
        if (bus_req_i.addr(3 downto 2) = "00") then -- channel-enable
          irq_enable <= bus_req_i.data(XIRQ_NUM_CH-1 downto 0);
        end if;
        if (bus_req_i.addr(3 downto 2) = "01") then -- clear pending IRQs
          nclr_pending <= bus_req_i.data(XIRQ_NUM_CH-1 downto 0); -- set zero to clear pending IRQ
        end if;
      end if;
    end if;
  end process write_access;

  -- read access --
  read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      bus_rsp_o.ack  <= bus_req_i.re or bus_req_i.we; -- bus handshake
      bus_rsp_o.data <= (others => '0');
      if (bus_req_i.re = '1') then
        case bus_req_i.addr(3 downto 2) is
          when "00"   => bus_rsp_o.data(XIRQ_NUM_CH-1 downto 0) <= irq_enable; -- channel-enable
          when "01"   => bus_rsp_o.data(XIRQ_NUM_CH-1 downto 0) <= irq_pending; -- pending IRQs
          when others => bus_rsp_o.data(4 downto 0)             <= irq_source; -- IRQ source
        end case;
      end if;
    end if;
  end process read_access;

  -- no access error possible --
  bus_rsp_o.err <= '0';


  -- IRQ Trigger --------------------------------------------------------------
  -- -----------------------------------------------------------------------------
  synchronizer: process(clk_i)
  begin
    if rising_edge(clk_i) then
      irq_sync  <= xirq_i(XIRQ_NUM_CH-1 downto 0);
      irq_sync2 <= irq_sync;
    end if;
  end process synchronizer;

  -- trigger type select --
  irq_trigger_gen:
  for i in 0 to XIRQ_NUM_CH-1 generate
    irq_trigger: process(irq_sync, irq_sync2)
      variable sel_v : std_ulogic_vector(1 downto 0);
    begin
      sel_v := XIRQ_TRIGGER_TYPE(i) & XIRQ_TRIGGER_POLARITY(i);
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
  irq_buffer: process(clk_i)
  begin
    if rising_edge(clk_i) then
      irq_pending <= (irq_pending and nclr_pending) or irq_trig;
    end if;
  end process irq_buffer;

  -- filter enabled channels --
  irq_raw <= irq_pending and irq_enable;

  -- encode current IRQ's priority --
  priority_encoder: process(irq_raw)
  begin
    irq_source_nxt <= (others => '0');
    if (XIRQ_NUM_CH > 1) then
      for i in 0 to XIRQ_NUM_CH-1 loop
        if (irq_raw(i) = '1') then
          irq_source_nxt <= std_ulogic_vector(to_unsigned(i, irq_source_nxt'length));
          exit;
        end if;
      end loop;
    end if;
  end process priority_encoder;

  -- anyone firing? --
  irq_fire <= '1' when (or_reduce_f(irq_raw) = '1') else '0';


  -- IRQ Arbiter --------------------------------------------------------------
  -- -----------------------------------------------------------------------------
  irq_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      cpu_irq_o  <= '0';
      irq_active <= '0';
      irq_source <= (others => '0');
    elsif rising_edge(clk_i) then
      cpu_irq_o <= '0';
      if (irq_active = '0') then -- no active IRQ
        irq_source <= irq_source_nxt; -- get IRQ source that has highest priority
        if (irq_fire = '1') then
          cpu_irq_o  <= '1';
          irq_active <= '1';
        end if;
      elsif (bus_req_i.we = '1') and (bus_req_i.addr(3 downto 2) = "10") then -- acknowledge on write access
        irq_active <= '0';
      end if;
    end if;
  end process irq_arbiter;


end neorv32_xirq_rtl;
