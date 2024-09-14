-- ================================================================================ --
-- NEORV32 SoC - 1-Wire Interface Host Controller (ONEWIRE)                         --
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

entity neorv32_onewire is
  port (
    clk_i       : in  std_ulogic; -- global clock line
    rstn_i      : in  std_ulogic; -- global reset line, low-active
    bus_req_i   : in  bus_req_t;  -- bus request
    bus_rsp_o   : out bus_rsp_t;  -- bus response
    clkgen_en_o : out std_ulogic; -- enable clock generator
    clkgen_i    : in  std_ulogic_vector(7 downto 0);
    onewire_i   : in  std_ulogic; -- 1-wire line state
    onewire_o   : out std_ulogic; -- 1-wire line pull-down
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

  -- control register --
  constant ctrl_en_c        : natural :=  0; -- r/w: TWI enable
  constant ctrl_prsc0_c     : natural :=  1; -- r/w: prescaler select bit 0
  constant ctrl_prsc1_c     : natural :=  2; -- r/w: prescaler select bit 1
  constant ctrl_clkdiv0_c   : natural :=  3; -- r/w: clock divider bit 0
  constant ctrl_clkdiv7_c   : natural := 10; -- r/w: clock divider bit 7
  constant ctrl_trig_rst_c  : natural := 11; -- -/w: trigger reset pulse, auto-clears
  constant ctrl_trig_bit_c  : natural := 12; -- -/w: trigger single-bit transmission, auto-clears
  constant ctrl_trig_byte_c : natural := 13; -- -/w: trigger full-byte transmission, auto-clears
  --
  constant ctrl_sense_c     : natural := 29; -- r/-: current state of the bus line
  constant ctrl_presence_c  : natural := 30; -- r/-: bus presence detected
  constant ctrl_busy_c      : natural := 31; -- r/-: set while operation in progress

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
    wire_in  : std_ulogic_vector(1 downto 0);
    wire_lo  : std_ulogic;
    wire_hi  : std_ulogic;
    sample   : std_ulogic;
    presence : std_ulogic;
  end record;
  signal serial : serial_t;

begin

  -- Bus Access ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bus_rsp_o      <= rsp_terminate_c;
      ctrl.enable    <= '0';
      ctrl.clk_prsc  <= (others => '0');
      ctrl.clk_div   <= (others => '0');
      ctrl.trig_rst  <= '0';
      ctrl.trig_bit  <= '0';
      ctrl.trig_byte <= '0';
      tx_data        <= (others => '0');
    elsif rising_edge(clk_i) then
      -- bus handshake --
      bus_rsp_o.ack  <= bus_req_i.stb;
      bus_rsp_o.err  <= '0';
      bus_rsp_o.data <= (others => '0');
      -- write access --
      if (bus_req_i.stb = '1') and (bus_req_i.rw = '1') then
        if (bus_req_i.addr(2) = '0') then -- control register
          ctrl.enable   <= bus_req_i.data(ctrl_en_c);
          ctrl.clk_prsc <= bus_req_i.data(ctrl_prsc1_c downto ctrl_prsc0_c);
          ctrl.clk_div  <= bus_req_i.data(ctrl_clkdiv7_c downto ctrl_clkdiv0_c);
        else -- data register
          tx_data <= bus_req_i.data(7 downto 0);
        end if;
      end if;
      -- operation triggers --
      if (bus_req_i.stb = '1') and (bus_req_i.rw = '1') and (bus_req_i.addr(2) = '0') then -- set by host
        ctrl.trig_rst  <= bus_req_i.data(ctrl_trig_rst_c);
        ctrl.trig_bit  <= bus_req_i.data(ctrl_trig_bit_c);
        ctrl.trig_byte <= bus_req_i.data(ctrl_trig_byte_c);
      elsif (ctrl.enable = '0') or (serial.state(1) = '1') then -- cleared when disabled or when in RTX/RESET state
        ctrl.trig_rst  <= '0';
        ctrl.trig_bit  <= '0';
        ctrl.trig_byte <= '0';
      end if;
      -- read access --
      if (bus_req_i.stb = '1') and (bus_req_i.rw = '0') then
        if (bus_req_i.addr(2) = '0') then -- control register
          bus_rsp_o.data(ctrl_en_c)                            <= ctrl.enable;
          bus_rsp_o.data(ctrl_prsc1_c downto ctrl_prsc0_c)     <= ctrl.clk_prsc;
          bus_rsp_o.data(ctrl_clkdiv7_c downto ctrl_clkdiv0_c) <= ctrl.clk_div;
          --
          bus_rsp_o.data(ctrl_sense_c)                         <= serial.wire_in(1);
          bus_rsp_o.data(ctrl_presence_c)                      <= serial.presence;
        else -- data register
          bus_rsp_o.data(7 downto 0) <= serial.sreg;
        end if;
      end if;

    end if;
  end process bus_access;


  -- Tick Generator -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tick_generator: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      clk_tick       <= '0';
      clk_cnt        <= (others => '0');
      serial.tick    <= '0';
      serial.tick_ff <= '0';
    elsif rising_edge(clk_i) then
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
  serial_engine: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      serial.wire_in  <= (others => '0');
      serial.wire_lo  <= '0';
      serial.wire_hi  <= '0';
      serial.state    <= (others => '0');
      serial.tick_cnt <= (others => '0');
      serial.bit_cnt  <= (others => '0');
      serial.sreg     <= (others => '0');
      serial.sample   <= '0';
      onewire_o       <= '0';
    elsif rising_edge(clk_i) then
      -- input synchronizer --
      serial.wire_in <= serial.wire_in(0) & to_stdulogic(to_bit(onewire_i)); -- "to_bit" to avoid hardware-vs-simulation mismatch

      -- bus control --
      if (serial.busy = '0') or (serial.wire_hi = '1') then -- disabled/idle or active tristate request
        onewire_o <= '1'; -- release bus (tristate), high (by pull-up resistor) or actively pulled low by device(s)
      elsif (serial.wire_lo = '1') then
        onewire_o <= '0'; -- pull bus actively low
      end if;

      -- defaults --
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
            serial.state(1 downto 0) <= "00"; -- go back to IDLE
          end if;

        when others => -- "0--" OFFLINE: deactivated, reset externally-readable signals
        -- ------------------------------------------------------------
          serial.sreg              <= (others => '0');
          serial.presence          <= '0';
          serial.state(1 downto 0) <= "00"; -- stay here, go to IDLE when module is enabled

      end case;
    end if;
  end process serial_engine;

  -- serial engine busy? --
  serial.busy <= '0' when (serial.state(1 downto 0) = "00") else '1';


  -- Interrupt Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_generator: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      irq_o <= '0';
    elsif rising_edge(clk_i) then
      if (serial.state = "100") then -- enabled and in idle state
        irq_o <= '1';
      else
        irq_o <= '0';
      end if;
    end if;
  end process irq_generator;


end neorv32_onewire_rtl;
