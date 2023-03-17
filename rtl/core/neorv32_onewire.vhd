-- #################################################################################################
-- # << NEORV32 - 1-Wire Interface Host Controller (ONEWIRE) >>                                    #
-- # ********************************************************************************************* #
-- # Single-wire bus controller, compatible to the "Dallas 1-Wire Bus System".                     #
-- # Provides three basic operations:                                                              #
-- # * generate reset pulse and check for device presence                                          #
-- # * transfer single bit (read-while-write)                                                      #
-- # * transfer full byte (read-while-write)                                                       #
-- # After completing any of the operations the interrupt signal is triggered.                     #
-- # The base time for bus interactions is configured using a 2-bit clock prescaler and a 8-bit    #
-- # clock divider. All bus operations are timed using (hardwired) multiples of this base time.    #
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

entity neorv32_onewire is
  port (
    -- host access --
    clk_i       : in  std_ulogic; -- global clock line
    rstn_i      : in  std_ulogic; -- global reset line, low-active, async
    addr_i      : in  std_ulogic_vector(31 downto 0); -- address
    rden_i      : in  std_ulogic; -- read enable
    wren_i      : in  std_ulogic; -- write enable
    data_i      : in  std_ulogic_vector(31 downto 0); -- data in
    data_o      : out std_ulogic_vector(31 downto 0); -- data out
    ack_o       : out std_ulogic; -- transfer acknowledge
    -- clock generator --
    clkgen_en_o : out std_ulogic; -- enable clock generator
    clkgen_i    : in  std_ulogic_vector(07 downto 0);
    -- com lines (require external tri-state drivers) --
    onewire_i   : in  std_ulogic; -- 1-wire line state
    onewire_o   : out std_ulogic; -- 1-wire line pull-down
    -- interrupt --
    irq_o       : out std_ulogic -- transfer done IRQ
  );
end neorv32_onewire;

architecture neorv32_onewire_rtl of neorv32_onewire is

  -- timing configuration (absolute time in multiples of the base tick time t_base) --
  constant t_write_one_c       : unsigned(6 downto 0) := to_unsigned( 1, 7); -- t0
  constant t_read_sample_c     : unsigned(6 downto 0) := to_unsigned( 2, 7); -- t1
  constant t_slot_end_c        : unsigned(6 downto 0) := to_unsigned( 7, 7); -- t2
  constant t_pause_end_c       : unsigned(6 downto 0) := to_unsigned( 9, 7); -- t3
  constant t_reset_end_c       : unsigned(6 downto 0) := to_unsigned(48, 7); -- t4
  constant t_presence_sample_c : unsigned(6 downto 0) := to_unsigned(55, 7); -- t5
  constant t_presence_end_c    : unsigned(6 downto 0) := to_unsigned(96, 7); -- t6
  -- -> see data sheet for more information about the t* timing values --

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(twi_size_c); -- low address boundary bit

  -- control register --
  constant ctrl_en_c        : natural :=  0; -- r/w: TWI enable
  constant ctrl_prsc0_c     : natural :=  1; -- r/w: prescaler select bit 0
  constant ctrl_prsc1_c     : natural :=  2; -- r/w: prescaler select bit 1
  constant ctrl_clkdiv0_c   : natural :=  3; -- r/w: clock divider bit 0
  constant ctrl_clkdiv1_c   : natural :=  4; -- r/w: clock divider bit 1
  constant ctrl_clkdiv2_c   : natural :=  5; -- r/w: clock divider bit 2
  constant ctrl_clkdiv3_c   : natural :=  6; -- r/w: clock divider bit 3
  constant ctrl_clkdiv4_c   : natural :=  7; -- r/w: clock divider bit 4
  constant ctrl_clkdiv5_c   : natural :=  8; -- r/w: clock divider bit 5
  constant ctrl_clkdiv6_c   : natural :=  9; -- r/w: clock divider bit 6
  constant ctrl_clkdiv7_c   : natural := 10; -- r/w: clock divider bit 7
  constant ctrl_trig_rst_c  : natural := 11; -- -/w: trigger reset pulse, auto-clears
  constant ctrl_trig_bit_c  : natural := 12; -- -/w: trigger single-bit transmission, auto-clears
  constant ctrl_trig_byte_c : natural := 13; -- -/w: trigger full-byte transmission, auto-clears
  --
  constant ctrl_sense_c     : natural := 29; -- r/-: current state of the bus line
  constant ctrl_presence_c  : natural := 30; -- r/-: bus presence detected
  constant ctrl_busy_c      : natural := 31; -- r/-: set while operation in progress

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- control register --
  type ctrl_t is record
    enable    : std_ulogic;
    clk_prsc  : std_ulogic_vector(1 downto 0);
    clk_div   : std_ulogic_vector(7 downto 0);
    trig_rst  : std_ulogic;
    trig_bit  : std_ulogic;
    trig_byte : std_ulogic;
  end record;
  signal ctrl : ctrl_t;

  -- write data --
  signal tx_data  : std_ulogic_vector(7 downto 0);

  -- clock generator --
  signal clk_sel  : std_ulogic_vector(3 downto 0);
  signal clk_tick : std_ulogic;
  signal clk_cnt  : unsigned(7 downto 0);

  -- serial engine --
  type serial_t is record
    state    : std_ulogic_vector(2 downto 0);
    busy     : std_ulogic;
    bit_cnt  : unsigned(2 downto 0);
    tick_cnt : unsigned(6 downto 0);
    tick     : std_ulogic;
    tick_ff  : std_ulogic;
    sreg     : std_ulogic_vector(7 downto 0);
    done     : std_ulogic;
    wire_in  : std_ulogic_vector(1 downto 0);
    wire_lo  : std_ulogic;
    wire_hi  : std_ulogic;
    sample   : std_ulogic;
    presence : std_ulogic;
  end record;
  signal serial : serial_t;

begin

  -- Write Access ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- access control --
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = onewire_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= onewire_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
  wren   <= acc_en and wren_i;
  rden   <= acc_en and rden_i;

  -- write access --
  write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl.enable    <= '0';
      ctrl.clk_prsc  <= (others => '0');
      ctrl.clk_div   <= (others => '0');
      ctrl.trig_rst  <= '0';
      ctrl.trig_bit  <= '0';
      ctrl.trig_byte <= '0';
      tx_data        <= (others => '0');
    elsif rising_edge(clk_i) then
      -- write access --
      if (wren = '1') then
        -- control register --
        if (addr = onewire_ctrl_addr_c) then
          ctrl.enable   <= data_i(ctrl_en_c);
          ctrl.clk_prsc <= data_i(ctrl_prsc1_c downto ctrl_prsc0_c);
          ctrl.clk_div  <= data_i(ctrl_clkdiv7_c downto ctrl_clkdiv0_c);
        end if;
        -- data register --
        if (addr = onewire_data_addr_c) then
          tx_data <= data_i(7 downto 0);
        end if;
      end if;

      -- operation triggers --
      if (wren = '1') and (addr = onewire_ctrl_addr_c) then -- set by host
        ctrl.trig_rst  <= data_i(ctrl_trig_rst_c);
        ctrl.trig_bit  <= data_i(ctrl_trig_bit_c);
        ctrl.trig_byte <= data_i(ctrl_trig_byte_c);
      elsif (ctrl.enable = '0') or (serial.state(1) = '1') then -- cleared when disabled or when in RTX/RESET state
        ctrl.trig_rst  <= '0';
        ctrl.trig_bit  <= '0';
        ctrl.trig_byte <= '0';
      end if;
    end if;
  end process write_access;

  -- read access --
  read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      ack_o  <= rden or wren; -- bus handshake
      data_o <= (others => '0');
      if (rden = '1') then
        -- control register --
        if (addr = onewire_ctrl_addr_c) then
          data_o(ctrl_en_c)                            <= ctrl.enable;
          data_o(ctrl_prsc1_c downto ctrl_prsc0_c)     <= ctrl.clk_prsc;
          data_o(ctrl_clkdiv7_c downto ctrl_clkdiv0_c) <= ctrl.clk_div;
          --
          data_o(ctrl_sense_c)                         <= serial.wire_in(1);
          data_o(ctrl_presence_c)                      <= serial.presence;
          data_o(ctrl_busy_c)                          <= serial.busy;
        -- data register --
        else -- if (addr = onewire_data_addr_c) then
          data_o(7 downto 0) <= serial.sreg;
        end if;
      end if;
    end if;
  end process read_access;


  -- Tick Generator -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tick_generator: process(clk_i)
  begin
    if rising_edge(clk_i) then
      clk_tick    <= clk_sel(to_integer(unsigned(ctrl.clk_prsc)));
      serial.tick <= '0'; -- default
      if (ctrl.enable = '0') then
        clk_cnt <= (others => '0');
      elsif (clk_tick = '1') then
        if (clk_cnt = unsigned(ctrl.clk_div)) then
          clk_cnt     <= (others => '0');
          serial.tick <= '1'; -- signal is high for 1 clk_i cycle every 't_base'
        else
          clk_cnt <= clk_cnt + 1;
        end if;
      end if;
      serial.tick_ff <= serial.tick; -- tick delayed by one clock cycle (for precise bus state sampling)
    end if;
  end process tick_generator;

  -- enable SoC clock generator --
  clkgen_en_o <= ctrl.enable;

  -- only use the lowest 4 clocks of the system clock generator --
  clk_sel <= clkgen_i(3 downto 0);


  -- Serial Engine --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  serial_engine: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- input synchronizer --
      serial.wire_in <= serial.wire_in(0) & to_stdulogic(to_bit(onewire_i)); -- "to_bit" to avoid hardware-vs-simulation mismatch

      -- bus control --
      if (serial.busy = '0') or (serial.wire_hi = '1') then -- disabled/idle or active tristate request
        onewire_o <= '1'; -- release bus (tristate), high (by pull-up resistor) or actively pulled low by device(s)
      elsif (serial.wire_lo = '1') then
        onewire_o <= '0'; -- pull bus actively low
      end if;

      -- defaults --
      serial.done    <= '0';
      serial.wire_lo <= '0';
      serial.wire_hi <= '0';

      -- FSM --
      serial.state(2) <= ctrl.enable; -- module enabled? force reset state otherwise
      case serial.state is

        when "100" => -- enabled, but IDLE: wait for new request
        -- ------------------------------------------------------------
          serial.tick_cnt <= (others => '0');
          -- transmission size --
          if (ctrl.trig_bit = '1') then
            serial.bit_cnt <= "000"; -- single bit
          else
            serial.bit_cnt <= "111"; -- full-byte
          end if;
          -- any operation request? --
          if (ctrl.trig_rst = '1') or (ctrl.trig_bit = '1') or (ctrl.trig_byte = '1') then
            serial.state(1 downto 0) <= "01"; -- SYNC
          end if;

        when "101" => -- SYNC: start operation with next base tick
        -- ------------------------------------------------------------
          serial.sreg <= tx_data;
          if (serial.tick = '1') then -- synchronize
            serial.wire_lo <= '1'; -- force bus to low
            if (ctrl.trig_rst = '1') then
              serial.state(1 downto 0) <= "11"; -- RESET
            else
              serial.state(1 downto 0) <= "10"; -- RTX
            end if;
          end if;

        when "110" => -- RTX: read/write 'serial.bit_cnt-1' bits
        -- ------------------------------------------------------------
          -- go high to write 1 or to read OR time slot completed --
          if ((serial.tick_cnt = t_write_one_c) and (serial.sreg(0) = '1')) or (serial.tick_cnt = t_slot_end_c) then
            serial.wire_hi <= '1'; -- release bus
          end if;
          -- sample input (precisely / just once!) --
          if (serial.tick_cnt = t_read_sample_c) and (serial.tick_ff = '1') then
            serial.sample <= serial.wire_in(1);
          end if;
          -- inter-slot pause (end of bit) & iteration control --
          if (serial.tick_cnt = t_pause_end_c) then -- bit done
            serial.tick_cnt <= (others => '0');
            serial.sreg     <= serial.sample & serial.sreg(7 downto 1); -- new bit; LSB first
            serial.bit_cnt  <= serial.bit_cnt - 1;
            if (serial.bit_cnt = "000") then -- all done
              serial.done              <= '1'; -- operation done
              serial.state(1 downto 0) <= "00"; -- go back to IDLE
            else -- next bit
              serial.wire_lo <= '1'; -- force bus to low again
            end if;
          elsif (serial.tick = '1') then
            serial.tick_cnt <= serial.tick_cnt + 1;
          end if;

        when "111" => -- RESET: generate reset pulse and check for bus presence
        -- ------------------------------------------------------------
          if (serial.tick = '1') then
            serial.tick_cnt <= serial.tick_cnt + 1;
          end if;
          -- end of reset pulse --
          if (serial.tick_cnt = t_reset_end_c) then
            serial.wire_hi <= '1'; -- release bus
          end if;
          -- sample device presence (precisely / just once!) --
          if (serial.tick_cnt = t_presence_sample_c) and (serial.tick_ff = '1') then
            serial.presence <= not serial.wire_in(1); -- set if bus is pulled low by any device
          end if;
          -- end of presence phase --
          if (serial.tick_cnt = t_presence_end_c) then
            serial.done              <= '1'; -- operation done
            serial.state(1 downto 0) <= "00"; -- go back to IDLE
          end if;

        when others => -- "0--" OFFLINE: deactivated, reset externally-readable signals
        -- ------------------------------------------------------------
          serial.sreg     <= (others => '0');
          serial.presence <= '0';
          serial.state(1 downto 0) <= "00"; -- stay here, go to IDLE when module is enabled

      end case;
    end if;
  end process serial_engine;

  -- serial engine busy? --
  serial.busy <= '0' when (serial.state(1 downto 0) = "00") else '1';

  -- operation done interrupt --
  irq_o <= serial.done;


end neorv32_onewire_rtl;
