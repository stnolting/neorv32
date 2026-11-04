-- ================================================================================ --
-- NEORV32 SoC - Two-Wire Interface Host Controller (TWI)                           --
-- -------------------------------------------------------------------------------- --
-- For a single-host bus setup only / no bus arbitration available (yet).           --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) Stephan Nolting. All rights reserved.                              --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_twi is
  generic (
    FIFO_DEPTH : natural range 1 to 32768 -- data/command FIFO depth, has to be a power of two, min 1
  );
  port (
    clk_i     : in  std_ulogic;                    -- global clock line
    rstn_i    : in  std_ulogic;                    -- global reset line, low-active, async
    bus_req_i : in  bus_req_t;                     -- bus request
    bus_rsp_o : out bus_rsp_t;                     -- bus response
    clkgen_i  : in  std_ulogic_vector(7 downto 0); -- prescaled clock enables
    twi_sda_i : in  std_ulogic;                    -- serial data line input
    twi_sda_o : out std_ulogic;                    -- serial data line output (0=GND, 1=high-Z)
    twi_scl_i : in  std_ulogic;                    -- serial clock line input
    twi_scl_o : out std_ulogic;                    -- serial clock line output (0=GND, 1=high-Z)
    irq_o     : out std_ulogic                     -- interrupt
  );
end entity;

architecture neorv32_twi_rtl of neorv32_twi is

  -- control register layout --
  constant ctrl_en_c        : natural :=  0; -- r/w: Module enable (reset when zero)
  constant ctrl_prsc0_c     : natural :=  1; -- r/w: Clock prescaler bit 0
  constant ctrl_prsc2_c     : natural :=  3; -- r/w: Clock prescaler bit 2
  constant ctrl_cdiv0_c     : natural :=  4; -- r/w: Clock divider bit 0
  constant ctrl_cdiv3_c     : natural :=  7; -- r/w: Clock divider bit 3
  --
  constant ctrl_fifo0_c     : natural := 15; -- r/-: log2(FIFO size), bit 0 (LSB)
  constant ctrl_fifo3_c     : natural := 18; -- r/-: log2(FIFO size), bit 3 (MSB)
  --
  constant ctrl_nack_c      : natural := 23; -- r/-: TX-NACK received
  constant ctrl_nack_clr_c  : natural := 24; -- -/w: Clear NACK flag
  constant ctrl_irq_clr_c   : natural := 25; -- -/w: Clear interrupt request
  constant ctrl_claimed_c   : natural := 26; -- r/-: Bus is claimed by TWI controller
  constant ctrl_sense_scl_c : natural := 27; -- r/-: Current state of the SCL bus line
  constant ctrl_sense_sda_c : natural := 28; -- r/-: Current state of the SDA bus line
  constant ctrl_tx_full_c   : natural := 29; -- r/-: TX FIFO full
  constant ctrl_rx_avail_c  : natural := 30; -- r/-: RX FIFO data available
  constant ctrl_busy_c      : natural := 31; -- r/-: Set if TWI unit is busy

  -- helpers --
  constant log2_fifo_size_c : natural := index_size_f(FIFO_DEPTH);

  -- bus interface --
  signal bus_ack, bus_rden : std_ulogic;
  signal bus_rdata : std_ulogic_vector(31 downto 0);

  -- control register --
  type ctrl_t is record
    enable : std_ulogic;
    prsc   : std_ulogic_vector(2 downto 0);
    cdiv   : std_ulogic_vector(3 downto 0);
  end record;
  signal ctrl : ctrl_t; -- register set
  signal ctrl_nack_clr, ctrl_irq_clr : std_ulogic; -- write-only control register bits

  -- TX FIFO interface --
  type tx_fifo_t is record
    clr   : std_ulogic; -- sync reset, high-active
    we    : std_ulogic; -- write enable
    rdata : std_ulogic_vector(8 downto 0); -- transmit data/command
    avail : std_ulogic; -- data available
    free  : std_ulogic; -- free entry available
  end record;
  signal tx_fifo : tx_fifo_t;

  -- RX FIFO interface --
  type rx_fifo_t is record
    clr   : std_ulogic; -- sync reset, high-active
    re    : std_ulogic; -- read enable
    rdata : std_ulogic_vector(7 downto 0); -- received data
    avail : std_ulogic; -- data available
  end record;
  signal rx_fifo : rx_fifo_t;

  -- timing generator --
  signal clkgen_cnt  : std_ulogic_vector(3 downto 0); -- clock divider
  signal clkgen_tick : std_ulogic; -- actual TWI clock tick
  signal clkgen_halt : std_ulogic; -- halt clock during clock stretching
  signal phase_sreg  : std_ulogic_vector(3 downto 0); -- clock phase generator
  signal phase       : std_ulogic_vector(3 downto 0);

  -- serial engine --
  type state_t is (S_OFFLINE, S_IDLE, S_START, S_STOP, S_DATA);
  type engine_t is record
    state : state_t; -- FSM state
    bcnt  : std_ulogic_vector(3 downto 0); -- bit counter
    sreg  : std_ulogic_vector(8 downto 0); -- shift register
    rw    : std_ulogic; -- 0 = RX, 1 = TX
    pop   : std_ulogic; -- pop TX data from FIFO
    push  : std_ulogic; -- push RX data to FIFO
    nack  : std_ulogic; -- NACK received
    claim : std_ulogic; -- bus is claimed by TWI controller
    scl   : std_ulogic; -- SCL driver
    sda   : std_ulogic; -- SDA driver
  end record;
  signal engine : engine_t;
  signal fsm_state : state_t; -- state override
  signal busy, busy_ff : std_ulogic; -- operation in progress

  -- input synchronizer --
  signal sda_sync, scl_sync : std_ulogic_vector(1 downto 0);

begin

  -- Bus Access -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_handshake: process(clk_i)
  begin
    if rising_edge(clk_i) then
      bus_ack  <= bus_req_i.stb;
      bus_rden <= bus_req_i.stb and (not bus_req_i.rw);
    end if;
  end process;

  -- write access --
  bus_write: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl.enable <= '0';
      ctrl.prsc   <= (others => '0');
      ctrl.cdiv   <= (others => '0');
    elsif rising_edge(clk_i) then
      if (bus_req_i.stb = '1') and (bus_req_i.rw = '1') and (bus_req_i.addr(2) = '0') then
        ctrl.enable <= bus_req_i.data(ctrl_en_c);
        ctrl.prsc   <= bus_req_i.data(ctrl_prsc2_c downto ctrl_prsc0_c);
        ctrl.cdiv   <= bus_req_i.data(ctrl_cdiv3_c downto ctrl_cdiv0_c);
      end if;
    end if;
  end process;

  -- write-only flags --
  wo_flags: process(bus_req_i)
  begin
    ctrl_nack_clr <= '0';
    ctrl_irq_clr  <= '0';
    if (bus_req_i.stb = '1') and (bus_req_i.rw = '1') and (bus_req_i.addr(2) = '0') then
      ctrl_nack_clr <= bus_req_i.data(ctrl_nack_clr_c);
      ctrl_irq_clr  <= bus_req_i.data(ctrl_irq_clr_c);
    end if;
  end process;

  -- read access (asynchronous) --
  bus_read: process(bus_rden, bus_req_i, ctrl, engine, scl_sync, sda_sync, tx_fifo, rx_fifo, busy)
  begin
    bus_rdata <= (others => '0');
    if (bus_rden = '1') then -- output gating
      if (bus_req_i.addr(2) = '0') then -- control register
        bus_rdata(ctrl_en_c)                        <= ctrl.enable;
        bus_rdata(ctrl_prsc2_c downto ctrl_prsc0_c) <= ctrl.prsc;
        bus_rdata(ctrl_cdiv3_c downto ctrl_cdiv0_c) <= ctrl.cdiv;
        bus_rdata(ctrl_fifo3_c downto ctrl_fifo0_c) <= std_ulogic_vector(to_unsigned(log2_fifo_size_c, 4));
        bus_rdata(ctrl_nack_c)                      <= engine.nack;
        bus_rdata(ctrl_claimed_c)                   <= engine.claim;
        bus_rdata(ctrl_sense_scl_c)                 <= scl_sync(1);
        bus_rdata(ctrl_sense_sda_c)                 <= sda_sync(1);
        bus_rdata(ctrl_tx_full_c)                   <= not tx_fifo.free;
        bus_rdata(ctrl_rx_avail_c)                  <= rx_fifo.avail;
        bus_rdata(ctrl_busy_c)                      <= busy;
      else -- RX data
        bus_rdata(7 downto 0) <= rx_fifo.rdata;
      end if;
    end if;
  end process;

  -- bus response --
  bus_rsp_o.ack  <= bus_ack;
  bus_rsp_o.err  <= '0'; -- no access errors supported
  bus_rsp_o.data <= bus_rdata;


  -- Data/Command FIFOs ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tx_fifo.clr <= (not ctrl.enable) or engine.nack; -- flush on TX-NACK
  tx_fifo.we  <= bus_req_i.stb and bus_req_i.rw and bus_req_i.addr(2);
  rx_fifo.clr <= not ctrl.enable;
  rx_fifo.re  <= bus_rden and bus_req_i.addr(2);

  -- TX-Data/Command --
  tx_fifo_inst: entity neorv32.neorv32_prim_fifo
  generic map (
    AWIDTH  => log2_fifo_size_c,
    DWIDTH  => 9,
    OUTGATE => false,
    ASYNCRD => false
  )
  port map (
    -- global control --
    clk_i   => clk_i,
    rstn_i  => rstn_i,
    clear_i => tx_fifo.clr,
    level_o => open,
    -- write port --
    wdata_i => bus_req_i.data(8 downto 0),
    we_i    => tx_fifo.we,
    free_o  => tx_fifo.free,
    -- read port --
    re_i    => engine.pop,
    rdata_o => tx_fifo.rdata,
    avail_o => tx_fifo.avail
  );

  -- RX-Data --
  rx_fifo_inst: entity neorv32.neorv32_prim_fifo
  generic map (
    AWIDTH  => log2_fifo_size_c,
    DWIDTH  => 8,
    OUTGATE => false,
    ASYNCRD => false
  )
  port map (
    -- global control --
    clk_i   => clk_i,
    rstn_i  => rstn_i,
    clear_i => rx_fifo.clr,
    level_o => open,
    -- write port --
    wdata_i => engine.sreg(8 downto 1),
    we_i    => engine.push,
    free_o  => open,
    -- read port --
    re_i    => rx_fifo.re,
    rdata_o => rx_fifo.rdata,
    avail_o => rx_fifo.avail
  );


  -- IRQ Generator --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_gen: process(clk_i)
  begin
    if rising_edge(clk_i) then
      busy_ff <= busy and ctrl.enable;
      if (ctrl.enable = '0') then
        irq_o <= '0';
      elsif (busy_ff = '1') and (busy = '0') then
        irq_o <= '1'; -- set if unit goes idle
      elsif (ctrl_irq_clr = '1') then
        irq_o <= '0'; -- clear on ACK
      end if;
    end if;
  end process;


  -- Timing Generator -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  timing_generator: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (ctrl.enable = '0') or (engine.state = S_IDLE) then -- stop clock when disabled or idle
        clkgen_tick <= '0';
        clkgen_cnt  <= (others => '0');
        phase_sreg  <= "0001"; -- start with new phase cycle
      else
        -- clock generator --
        clkgen_tick <= '0'; -- default
        if (clkgen_halt = '0') then -- not halted by clock stretching
          if (clkgen_i(to_integer(unsigned(ctrl.prsc))) = '1') then -- pre-scaled clock tick
            if (clkgen_cnt = ctrl.cdiv) then -- clock divider for fine-tuning
              clkgen_tick <= '1';
              clkgen_cnt  <= (others => '0');
            else
              clkgen_cnt <= std_ulogic_vector(unsigned(clkgen_cnt) + 1);
            end if;
          end if;
        end if;
        -- phase generator --
        if (clkgen_tick = '1') then -- clock tick
          phase_sreg <= phase_sreg(2 downto 0) & phase_sreg(3); -- rotate left
        end if;
      end if;
    end if;
  end process;

  -- TWI signals are set/sampled using 4 non-overlapping clock phase ticks --
  phase(0) <= clkgen_tick and phase_sreg(0); -- first step
  phase(1) <= clkgen_tick and phase_sreg(1);
  phase(2) <= clkgen_tick and phase_sreg(2);
  phase(3) <= clkgen_tick and phase_sreg(3); -- last step

  -- clock stretching: controller wants to drive SCL high, but SCL is still pulled low by peripheral --
  clkgen_halt <= '1' when (engine.scl = '1') and (scl_sync(1) = '0') else '0';


  -- Serial Engine --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  serial_engine: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      engine.state <= S_OFFLINE;
      engine.bcnt  <= (others => '0');
      engine.sreg  <= (others => '0');
      engine.rw    <= '0';
      engine.pop   <= '0';
      engine.push  <= '0';
      engine.nack  <= '0';
      engine.claim <= '0';
      engine.scl   <= '1';
      engine.sda   <= '1';
    elsif rising_edge(clk_i) then
      engine.pop  <= '0'; -- default
      engine.push <= '0'; -- default
      case fsm_state is

        when S_IDLE => -- idle, check for new command
        -- ------------------------------------------------------------
          engine.bcnt <= (others => '0');
          -- process next command --
          if (tx_fifo.avail = '1') then -- command available
            engine.pop <= '1'; -- consume command
            engine.rw  <= not tx_fifo.rdata(8); -- RX/TX data
            if (tx_fifo.rdata(8) = '0') then -- transmit data and check ACK/NACK
              engine.sreg  <= tx_fifo.rdata(7 downto 0) & '1';
              engine.state <= S_DATA;
            elsif (tx_fifo.rdata(1) = '1') then -- receive data and send ACK/NACK
              engine.sreg  <= x"ff" & tx_fifo.rdata(0);
              engine.state <= S_DATA;
            elsif (tx_fifo.rdata(0) = '0') then -- generate START
              engine.state <= S_START;
            elsif (engine.claim = '1') then -- generate STOP only if bus is claimed
              engine.state <= S_STOP;
            end if;
          end if;
          -- clear NACK flag --
          if (ctrl_nack_clr = '1') then
            engine.nack <= '0';
          end if;

        when S_START => -- generate (repeated)START condition
        -- ------------------------------------------------------------
          engine.claim <= '1';
          if (phase(0) = '1') then
            engine.sda <= '1';
          elsif (phase(2) = '1') then
            engine.sda <= '0';
          end if;
          if (phase(1) = '1') then
            engine.scl <= '1';
          elsif (phase(3) = '1') then
            engine.scl <= '0';
          end if;
          if (phase(3) = '1') then -- operation done
            engine.state <= S_IDLE;
          end if;

        when S_STOP => -- generate STOP condition
        -- ------------------------------------------------------------
          if (phase(1) = '1') then
            engine.scl <= '1';
          end if;
          if (phase(0) = '1') then
            engine.sda <= '0';
          elsif (phase(2) = '1') then
            engine.sda <= '1';
          end if;
          if (phase(3) = '1') then -- operation done
            engine.claim <= '0';
            engine.state <= S_IDLE;
          end if;

        when S_DATA => -- transmit/receive data
        -- ------------------------------------------------------------
          if (phase(3) = '1') and (engine.bcnt = "1001") then -- operation done
            if (engine.rw = '1') then -- transmitting
              if (engine.sreg(0) = '1') then -- NACK
                engine.nack  <= '1';
                engine.state <= S_STOP; -- terminate: generate STOP condition
              else -- ACK
                engine.state <= S_IDLE;
              end if;
            else -- receiving
              engine.push  <= '1'; -- push RX data to FIFO
              engine.state <= S_IDLE;
            end if;
          end if;
          -- SDA update --
          if (phase(0) = '1') then
            engine.sda <= engine.sreg(8);
          end if;
          -- sample --
          if (phase(2) = '1') then
            engine.sreg <= engine.sreg(7 downto 0) & sda_sync(1);
            engine.bcnt <= std_ulogic_vector(unsigned(engine.bcnt) + 1);
          end if;
          -- SCL drive --
          if (phase(1) = '1') then
            engine.scl <= '1';
          elsif (phase(3) = '1') then
            engine.scl <= '0'; -- [NOTE] SCL remains low after transmission to keep bus claimed
          end if;

        when others => -- S_OFFLINE: module deactivated, bus released
        -- ------------------------------------------------------------
          engine.scl   <= '1';
          engine.sda   <= '1';
          engine.nack  <= '0';
          engine.claim <= '0';
          engine.state <= S_IDLE;

      end case;
    end if;
  end process;

  -- FSM state override --
  fsm_state <= engine.state when (ctrl.enable = '1') else S_OFFLINE;

  -- operation in progress --
  busy <= '0' when (engine.state = S_OFFLINE) or ((engine.state = S_IDLE) and (tx_fifo.avail = '0')) else '1';

  -- input synchronizer --
  input_sync: process(clk_i)
  begin
    if rising_edge(clk_i) then
      sda_sync <= sda_sync(0) & to_stdulogic(to_bit(twi_sda_i)); -- "to_bit" to avoid HW-vs-sim mismatch
      scl_sync <= scl_sync(0) & to_stdulogic(to_bit(twi_scl_i));
    end if;
  end process;

  -- tri-state control: 0 = drive GND, 1 = tri-state / driven by pull-up --
  twi_sda_o <= engine.sda;
  twi_scl_o <= engine.scl;

end architecture;
