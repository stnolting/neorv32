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

entity neorv32_xirq is
  generic (
    XIRQ_NUM_CH           : natural; -- number of external IRQ channels (0..32)
    XIRQ_TRIGGER_TYPE     : std_ulogic_vector(31 downto 0); -- trigger type: 0=level, 1=edge
    XIRQ_TRIGGER_POLARITY : std_ulogic_vector(31 downto 0)  -- trigger polarity: 0=low-level/falling-edge, 1=high-level/rising-edge
  );
  port (
    -- host access --
    clk_i     : in  std_ulogic; -- global clock line
    addr_i    : in  std_ulogic_vector(31 downto 0); -- address
    rden_i    : in  std_ulogic; -- read enable
    wren_i    : in  std_ulogic; -- write enable
    data_i    : in  std_ulogic_vector(31 downto 0); -- data in
    data_o    : out std_ulogic_vector(31 downto 0); -- data out
    ack_o     : out std_ulogic; -- transfer acknowledge
    -- external interrupt lines --
    xirq_i    : in  std_ulogic_vector(XIRQ_NUM_CH-1 downto 0);
    -- CPU interrupt --
    cpu_irq_o : out std_ulogic
  );
end neorv32_xirq;

architecture neorv32_xirq_rtl of neorv32_xirq is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(xirq_size_c); -- low address boundary bit

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- control registers --
  signal irq_enable  : std_ulogic_vector(XIRQ_NUM_CH-1 downto 0); -- r/w: interrupt enable
  signal clr_pending : std_ulogic_vector(XIRQ_NUM_CH-1 downto 0); -- r/w: clear pending IRQs
  signal irq_src     : std_ulogic_vector(4 downto 0); -- r/w: source IRQ, ACK on any write

  -- interrupt trigger --
  signal irq_sync  : std_ulogic_vector(XIRQ_NUM_CH-1 downto 0);
  signal irq_sync2 : std_ulogic_vector(XIRQ_NUM_CH-1 downto 0);
  signal irq_trig  : std_ulogic_vector(XIRQ_NUM_CH-1 downto 0);

  -- interrupt buffer --
  signal irq_buf  : std_ulogic_vector(XIRQ_NUM_CH-1 downto 0);
  signal irq_fire : std_ulogic;

  -- interrupt source --
  signal irq_src_nxt : std_ulogic_vector(4 downto 0);

  -- arbiter --
  signal irq_run : std_ulogic;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not ((XIRQ_NUM_CH < 0) or (XIRQ_NUM_CH > 32)) report "NEORV32 PROCESSOR CONFIG ERROR: Number of XIRQ inputs <XIRQ_NUM_CH> has to be 0..32." severity error;


  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = xirq_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= xirq_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
  wren   <= acc_en and wren_i;
  rden   <= acc_en and rden_i;


  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rw_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- bus handshake --
      ack_o <= rden or wren;

      -- write access --
      clr_pending <= (others => '1');
      if (wren = '1') then
        -- channel-enable --
        if (addr = xirq_enable_addr_c) then
          irq_enable <= data_i(XIRQ_NUM_CH-1 downto 0);
        end if;
        -- clear pending IRQs --
        if (addr = xirq_pending_addr_c) then
          clr_pending <= data_i(XIRQ_NUM_CH-1 downto 0); -- set zero to clear pending IRQ
        end if;
      end if;

      -- read access --
      data_o <= (others => '0');
      if (rden = '1') then
        case addr is
          when xirq_enable_addr_c  => data_o(XIRQ_NUM_CH-1 downto 0) <= irq_enable; -- channel-enable
          when xirq_pending_addr_c => data_o(XIRQ_NUM_CH-1 downto 0) <= irq_buf; -- pending IRQs
          when xirq_source_addr_c  => data_o(4 downto 0) <= irq_src; -- source IRQ
          when others => NULL;
        end case;
      end if;
    end if;
  end process rw_access;


  -- IRQ Trigger --------------------------------------------------------------
  -- -----------------------------------------------------------------------------
  irq_trigger: process(clk_i)
  begin
    if rising_edge(clk_i) then
      irq_sync  <= xirq_i;
      irq_sync2 <= irq_sync;
    end if;
  end process irq_trigger;

  irq_trigger_comb: process(irq_sync, irq_sync2)
    variable sel_v : std_ulogic_vector(1 downto 0);
  begin
    for i in 0 to XIRQ_NUM_CH-1 loop
      sel_v := XIRQ_TRIGGER_TYPE(i) & XIRQ_TRIGGER_POLARITY(i);
      case sel_v is
        when "00"   => irq_trig(i) <= not irq_sync(i); -- low-level
        when "01"   => irq_trig(i) <= irq_sync(i); -- high-level
        when "10"   => irq_trig(i) <= (not irq_sync(i)) and irq_sync2(i); -- falling-edge
        when "11"   => irq_trig(i) <= irq_sync(i) and (not irq_sync2(i)); -- rising-edge
        when others => irq_trig(i) <= '0';
      end case;
    end loop;
  end process irq_trigger_comb;


  -- IRQ Buffer ---------------------------------------------------------------
  -- -----------------------------------------------------------------------------
  irq_buffer: process(clk_i)
  begin
    if rising_edge(clk_i) then
      irq_buf <= (irq_buf or (irq_trig and irq_enable)) and clr_pending;
    end if;
  end process irq_buffer;

  -- anyone firing? --
  irq_fire <= or_reduce_f(irq_buf);


  -- IRQ Priority Encoder -----------------------------------------------------
  -- -----------------------------------------------------------------------------
  irq_priority: process(irq_buf)
  begin
    irq_src_nxt <= (others => '0');
    if (XIRQ_NUM_CH > 1) then
      for i in 0 to XIRQ_NUM_CH-1 loop
        if (irq_buf(i) = '1') then
          irq_src_nxt(index_size_f(XIRQ_NUM_CH)-1 downto 0) <= std_ulogic_vector(to_unsigned(i, index_size_f(XIRQ_NUM_CH)));
          exit;
        end if;
      end loop;
    end if;
  end process irq_priority;


  -- IRQ Arbiter --------------------------------------------------------------
  -- -----------------------------------------------------------------------------
  irq_arbiter: process(clk_i)
  begin
    if rising_edge(clk_i) then
      cpu_irq_o <= '0';
      if (irq_run = '0') then -- no active IRQ
        if (irq_fire = '1') then
          cpu_irq_o <= '1';
          irq_run   <= '1';
          irq_src   <= irq_src_nxt;
        end if;
      else -- active IRQ, wait for CPU to acknowledge
        if (wren = '1') and (addr = xirq_source_addr_c) then -- write _any_ value to acknowledge
          irq_run <= '0';
        end if;
      end if;
    end if;
  end process irq_arbiter;


end neorv32_xirq_rtl;
