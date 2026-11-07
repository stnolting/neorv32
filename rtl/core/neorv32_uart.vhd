-- ================================================================================ --
-- NEORV32 SoC - Universal Asynchronous Receiver and Transmitter (UART)             --
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

-- pragma translate_off
-- RTL_SYNTHESIS OFF
use std.textio.all;
-- RTL_SYNTHESIS ON
-- pragma translate_on

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_uart is
  generic (
    RX_FIFO_DEPTH : natural range 1 to 32768; -- RX FIFO depth, has to be a power of two, min 1
    TX_FIFO_DEPTH : natural range 1 to 32768  -- TX FIFO depth, has to be a power of two, min 1
  );
  port (
    clk_i       : in  std_ulogic;                    -- global clock line
    rstn_i      : in  std_ulogic;                    -- global reset line, low-active, async
    bus_req_i   : in  bus_req_t;                     -- bus request
    bus_rsp_o   : out bus_rsp_t;                     -- bus response
    clkgen_i    : in  std_ulogic_vector(7 downto 0); -- prescaled clock enables
    uart_txd_o  : out std_ulogic;                    -- serial TX line
    uart_rxd_i  : in  std_ulogic;                    -- serial RX line
    uart_rtsn_o : out std_ulogic;                    -- ready to receive ("RTR"), low-active
    uart_ctsn_i : in  std_ulogic;                    -- allowed to transmit, low-active
    irq_o       : out std_ulogic                     -- interrupt
  );
end entity;

architecture neorv32_uart_rtl of neorv32_uart is

  -- control register bits --
  constant ctrl_rx_en_c       : natural :=  0; -- r/w: RX enable
  constant ctrl_tx_en_c       : natural :=  1; -- r/w: TX enable
  constant ctrl_tx_sim_en_c   : natural :=  2; -- r/w: TX simulation mode enable
  constant ctrl_rts_en_c      : natural :=  3; -- r/w: RX hardware flow-control enable (RTS)
  constant ctrl_cts_en_c      : natural :=  4; -- r/w: TX hardware flow-control enable (CTS)
  --
  constant ctrl_rx_empty_c    : natural :=  8; -- r/-: RX FIFO empty
  constant ctrl_rx_half_c     : natural :=  9; -- r/-: RX FIFO at least half full
  constant ctrl_rx_full_c     : natural := 10; -- r/-: RX FIFO full
  constant ctrl_rx_over_c     : natural := 11; -- r/-: RX FIFO overflow
  constant ctrl_tx_empty_c    : natural := 12; -- r/-: TX FIFO empty
  constant ctrl_tx_half_c     : natural := 13; -- r/-: TX FIFO at least half full
  constant ctrl_tx_full_c     : natural := 14; -- r/-: TX FIFO full
  constant ctrl_tx_busy_c     : natural := 15; -- r/-: TX FIFO not empty or transmitter busy
  constant ctrl_rx_irq_sel0_c : natural := 16; -- r/w: RX interrupt select, bit 0 (LSB)
  constant ctrl_rx_irq_sel1_c : natural := 17; -- r/w: RX interrupt select, bit 1 (MSB)
  constant ctrl_tx_irq_sel0_c : natural := 18; -- r/w: TX interrupt select, bit 0 (LSB)
  constant ctrl_tx_irq_sel1_c : natural := 19; -- r/w: TX interrupt select, bit 1 (MSB)
  constant ctrl_rx_irq_pend_c : natural := 20; -- r/-: pending RX interrupt
  constant ctrl_tx_irq_pend_c : natural := 21; -- r/-: pending TX interrupt
  --
  constant ctrl_rx_fifo0_c    : natural := 24; -- r/-: log2(RX_FIFO_DEPTH), bit 0 (LSB)
  constant ctrl_rx_fifo3_c    : natural := 27; -- r/-: log2(RX_FIFO_DEPTH), bit 3 (MSB)
  constant ctrl_tx_fifo0_c    : natural := 28; -- r/-: log2(TX_FIFO_DEPTH), bit 0 (LSB)
  constant ctrl_tx_fifo3_c    : natural := 31; -- r/-: log2(TX_FIFO_DEPTH), bit 3 (MSB)

  -- FIFO address widths --
  constant rx_awidth_c : natural := index_size_f(RX_FIFO_DEPTH);
  constant tx_awidth_c : natural := index_size_f(TX_FIFO_DEPTH);

  -- bus interface --
  signal bus_ack, bus_rden : std_ulogic;
  signal bus_rdata : std_ulogic_vector(31 downto 0);

  -- control register --
  type ctrl_t is record
    rx_en      : std_ulogic;
    tx_en      : std_ulogic;
    tx_sim_en  : std_ulogic;
    rts_en     : std_ulogic;
    cts_en     : std_ulogic;
    rx_irq_sel : std_ulogic_vector(1 downto 0);
    tx_irq_sel : std_ulogic_vector(1 downto 0);
  end record;
  signal ctrl : ctrl_t;

  -- baud rate configuration --
  signal baud_div : std_ulogic_vector(9 downto 0);
  signal clk_prsc : std_ulogic_vector(2 downto 0);

  -- serial engines --
  type engine_t is record
    busy    : std_ulogic; -- operation in progress
    commit  : std_ulogic; -- commit data
    sync    : std_ulogic_vector(2 downto 0); -- input synchronizer
    sreg    : std_ulogic_vector(8 downto 0); -- shift register
    bitcnt  : std_ulogic_vector(3 downto 0); -- bit counter
    baudcnt : std_ulogic_vector(9 downto 0); -- baud counter
  end record;
  signal tx, rx : engine_t;

  -- FIFO interfaces --
  type fifo_t is record
    clear, we, re     : std_ulogic;
    wdata, rdata      : std_ulogic_vector(7 downto 0);
    free, avail, half : std_ulogic;
  end record;
  signal rx_fifo, tx_fifo : fifo_t;
  signal rx_level : std_ulogic_vector(rx_awidth_c downto 0);
  signal tx_level : std_ulogic_vector(tx_awidth_c downto 0);

  -- misc --
  signal uart_clk, rx_irq, tx_irq, rx_over : std_ulogic;

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

  -- register write access --
  bus_write: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl.rx_en      <= '0';
      ctrl.tx_en      <= '0';
      ctrl.tx_sim_en  <= '0';
      ctrl.rts_en     <= '0';
      ctrl.cts_en     <= '0';
      ctrl.rx_irq_sel <= (others => '0');
      ctrl.tx_irq_sel <= (others => '0');
      baud_div        <= (others => '0');
      clk_prsc        <= (others => '0');
    elsif rising_edge(clk_i) then
      if (bus_req_i.stb = '1') and (bus_req_i.rw = '1') then
         -- control register --
        if (bus_req_i.addr(3 downto 2) = "00") then
          ctrl.rx_en      <= bus_req_i.data(ctrl_rx_en_c);
          ctrl.tx_en      <= bus_req_i.data(ctrl_tx_en_c);
          ctrl.tx_sim_en  <= bus_req_i.data(ctrl_tx_sim_en_c) and bool_to_ulogic_f(is_simulation_c);
          ctrl.rts_en     <= bus_req_i.data(ctrl_rts_en_c);
          ctrl.cts_en     <= bus_req_i.data(ctrl_cts_en_c);
          ctrl.rx_irq_sel <= bus_req_i.data(ctrl_rx_irq_sel1_c downto ctrl_rx_irq_sel0_c);
          ctrl.tx_irq_sel <= bus_req_i.data(ctrl_tx_irq_sel1_c downto ctrl_tx_irq_sel0_c);
        end if;
        -- baud rate configuration --
        if (bus_req_i.addr(3 downto 2) = "01") then
          baud_div <= bus_req_i.data(9  downto 0);
          clk_prsc <= bus_req_i.data(18 downto 16);
        end if;
      end if;
    end if;
  end process;

  -- read access (asynchronous) --
  bus_read: process(bus_rden, bus_req_i, ctrl, rx_fifo, rx_over, tx_fifo, tx, rx_irq, tx_irq, baud_div, clk_prsc)
  begin
    bus_rdata <= (others => '0');
    if (bus_rden = '1') then -- output gating
      case bus_req_i.addr(3 downto 2) is
        when "00" => -- control register
          bus_rdata(ctrl_rx_en_c)     <= ctrl.rx_en;
          bus_rdata(ctrl_tx_en_c)     <= ctrl.tx_en;
          bus_rdata(ctrl_tx_sim_en_c) <= ctrl.tx_sim_en and bool_to_ulogic_f(is_simulation_c);
          bus_rdata(ctrl_rts_en_c)    <= ctrl.rts_en;
          bus_rdata(ctrl_cts_en_c)    <= ctrl.cts_en;
          --
          bus_rdata(ctrl_rx_empty_c) <= not rx_fifo.avail;
          bus_rdata(ctrl_rx_half_c)  <= rx_fifo.half;
          bus_rdata(ctrl_rx_full_c)  <= not rx_fifo.free;
          bus_rdata(ctrl_rx_over_c)  <= rx_over;
          bus_rdata(ctrl_tx_empty_c) <= not tx_fifo.avail;
          bus_rdata(ctrl_tx_half_c)  <= tx_fifo.half;
          bus_rdata(ctrl_tx_full_c)  <= not tx_fifo.free;
          bus_rdata(ctrl_tx_busy_c)  <= tx.busy or tx_fifo.avail;
          --
          bus_rdata(ctrl_rx_irq_sel1_c downto ctrl_rx_irq_sel0_c) <= ctrl.rx_irq_sel;
          bus_rdata(ctrl_tx_irq_sel1_c downto ctrl_tx_irq_sel0_c) <= ctrl.tx_irq_sel;
          bus_rdata(ctrl_rx_irq_pend_c)                           <= rx_irq;
          bus_rdata(ctrl_tx_irq_pend_c)                           <= tx_irq;
          --
          bus_rdata(ctrl_rx_fifo3_c downto ctrl_rx_fifo0_c) <= std_ulogic_vector(to_unsigned(rx_awidth_c, 4));
          bus_rdata(ctrl_tx_fifo3_c downto ctrl_tx_fifo0_c) <= std_ulogic_vector(to_unsigned(tx_awidth_c, 4));
        when "01" => -- baud rate configuration
          bus_rdata(9  downto 0)  <= baud_div;
          bus_rdata(18 downto 16) <= clk_prsc;
        when others => -- data register
          bus_rdata(7 downto 0) <= rx_fifo.rdata(7 downto 0);
      end case;
    end if;
  end process;

  -- bus response --
  bus_rsp_o.ack  <= bus_ack;
  bus_rsp_o.err  <= '0'; -- no access errors supported
  bus_rsp_o.data <= bus_rdata;

  -- UART clock tick --
  uart_clk <= clkgen_i(to_integer(unsigned(clk_prsc)));

  -- CPU interrupt --
  irq_gen: process(clk_i)
  begin
    if rising_edge(clk_i) then
      irq_o <= (ctrl.rx_en and rx_irq) or (ctrl.tx_en and tx_irq);
    end if;
  end process;


  -- RX FIFO --------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rx_fifo_inst: entity neorv32.neorv32_prim_fifo
  generic map (
    AWIDTH  => rx_awidth_c,
    DWIDTH  => 8,
    OUTGATE => false,
    ASYNCRD => false
  )
  port map (
    -- global control --
    clk_i   => clk_i,
    rstn_i  => '1', -- not required
    clear_i => rx_fifo.clear,
    level_o => rx_level,
    -- write port --
    wdata_i => rx_fifo.wdata,
    we_i    => rx_fifo.we,
    free_o  => rx_fifo.free,
    -- read port --
    re_i    => rx_fifo.re,
    rdata_o => rx_fifo.rdata,
    avail_o => rx_fifo.avail
  );

  rx_fifo.clear <= not ctrl.rx_en;
  rx_fifo.wdata <= rx.sreg(7 downto 0);
  rx_fifo.we    <= rx.commit;
  rx_fifo.re    <= bus_rden and bus_req_i.addr(3);

  rx_fifo_small:
  if (rx_awidth_c = 0) generate
    rx_fifo.half <= rx_fifo.avail;
  end generate;

  rx_fifo_large:
  if (rx_awidth_c > 0) generate
    rx_fifo.half <= rx_level(rx_awidth_c) or rx_level(rx_awidth_c-1);
  end generate;

  -- RX overflow error (sticky) --
  rx_overflow: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (ctrl.rx_en = '0') or (rx_fifo.free = '1') then -- clear: disabled or FIFO space available
        rx_over <= '0';
      elsif (rx_fifo.we = '1') and (rx_fifo.free = '0') then -- set: writing to full FIFO
        rx_over <= '1';
      end if;
    end if;
  end process;

  -- interrupt trigger --
  rx_irq_gen: process(ctrl.rx_irq_sel, rx_fifo)
  begin
    case ctrl.rx_irq_sel is
      when "01"   => rx_irq <= rx_fifo.avail;    -- FIFO data available
      when "10"   => rx_irq <= rx_fifo.half;     -- FIFO at least half full
      when "11"   => rx_irq <= not rx_fifo.free; -- FIFO full
      when others => rx_irq <= '0';              -- disabled
    end case;
  end process;


  -- TX FIFO --------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tx_fifo_inst: entity neorv32.neorv32_prim_fifo
  generic map (
    AWIDTH  => tx_awidth_c,
    DWIDTH  => 8,
    OUTGATE => false,
    ASYNCRD => false
  )
  port map (
    -- global control --
    clk_i   => clk_i,
    rstn_i  => '1', -- not required
    clear_i => tx_fifo.clear,
    level_o => tx_level,
    -- write port --
    wdata_i => tx_fifo.wdata,
    we_i    => tx_fifo.we,
    free_o  => tx_fifo.free,
    -- read port --
    re_i    => tx_fifo.re,
    rdata_o => tx_fifo.rdata,
    avail_o => tx_fifo.avail
  );

  tx_fifo.clear <= (not ctrl.tx_en) or ctrl.tx_sim_en;
  tx_fifo.wdata <= bus_req_i.data(7 downto 0);
  tx_fifo.we    <= bus_req_i.stb and bus_req_i.rw and bus_req_i.addr(3);
  tx_fifo.re    <= tx.commit;

  tx_fifo_small:
  if (tx_awidth_c = 0) generate
    tx_fifo.half <= tx_fifo.avail;
  end generate;

  tx_fifo_large:
  if (tx_awidth_c > 0) generate
    tx_fifo.half <= tx_level(tx_awidth_c) or tx_level(tx_awidth_c-1);
  end generate;

  -- interrupt trigger --
  tx_irq_gen: process(ctrl.tx_irq_sel, tx_fifo, tx.busy)
  begin
    case ctrl.tx_irq_sel is
      when "01"   => tx_irq <= not (tx.busy or tx_fifo.avail); -- bus engine idle and FIFO empty
      when "10"   => tx_irq <= not tx_fifo.half;               -- FIFO less than half full
      when "11"   => tx_irq <= tx_fifo.free;                   -- FIFO not full
      when others => tx_irq <= '0';                            -- disabled
    end case;
  end process;


  -- Receiver (RX) --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  receiver: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- input synchronizer --
      if (uart_clk = '1') then
        rx.sync <= rx.sync(1 downto 0) & uart_rxd_i;
      end if;
      -- fsm --
      rx.commit <= '0'; -- default
      if (ctrl.rx_en = '0') then -- disabled
        rx.busy <= '0';
      elsif (rx.busy = '0') then -- idle
        rx.baudcnt <= '0' & baud_div(9 downto 1); -- half-baud to sample at center of each bit
        rx.bitcnt  <= "1010"; -- 1 start-bit + 8 data-bits + 1 stop-bit
        if (uart_clk = '1') and (rx.sync(2 downto 1) = "10") then -- start-bit detected
          rx.busy <= '1';
        end if;
      elsif (uart_clk = '1') then -- operation in progress
        if (rx.baudcnt = "0000000000") then -- bit done
          rx.baudcnt <= baud_div;
          rx.bitcnt  <= std_ulogic_vector(unsigned(rx.bitcnt) - 1);
          rx.sreg    <= rx.sync(2) & rx.sreg(rx.sreg'left downto 1);
          if (rx.bitcnt = "1010") and (rx.sync(2) = '1') then -- start bit no longer valid, abort
            rx.busy <= '0';
          elsif (rx.bitcnt = "0001") then
            rx.commit <= rx.sync(2); -- push only if stop bit is valid
            rx.busy   <= '0';
          end if;
        else
          rx.baudcnt <= std_ulogic_vector(unsigned(rx.baudcnt) - 1);
        end if;
      end if;
    end if;
  end process;

  -- RX hardware flow control --
  rx_flow: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      uart_rtsn_o <= '1';
    elsif rising_edge(clk_i) then
      if (ctrl.rx_en = '0') then -- receiver disabled - not allowed to send
        uart_rtsn_o <= '1';
      elsif (ctrl.rts_en = '0') then -- flow control disabled - allowed to send
        uart_rtsn_o <= '0';
      elsif (rx_fifo.half = '0') then -- FIFO less than half full - allowed to send
        uart_rtsn_o <= '0';
      else -- FIFO more than half full - not allowed to send
        uart_rtsn_o <= '1';
      end if;
    end if;
  end process;


  -- Transmitter (TX) -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  transmitter: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- input synchronizer --
      if (uart_clk = '1') then
        tx.sync <= tx.sync(1 downto 0) & uart_ctsn_i;
      end if;
      -- fsm --
      tx.commit <= '0'; -- default
      if (ctrl.tx_en = '0') then -- disabled
        tx.busy <= '0';
      elsif (tx.busy = '0') then -- idle
        tx.baudcnt <= baud_div;
        tx.bitcnt  <= "1010"; -- 1 start-bit + 8 data-bits + 1 stop-bit
        tx.sreg    <= tx_fifo.rdata & '0'; -- data & start-bit
        if (uart_clk = '1') and (tx_fifo.avail = '1') then -- data available
          if (tx.sync(2 downto 1) = "00") or (ctrl.cts_en = '0') then -- allowed to send OR no flow-control
            tx.commit <= '1';
            tx.busy   <= '1';
          end if;
        end if;
      elsif (uart_clk = '1') then -- operation in progress
        if (tx.baudcnt = "0000000000") then -- bit done
          tx.baudcnt <= baud_div;
          tx.bitcnt  <= std_ulogic_vector(unsigned(tx.bitcnt) - 1);
          tx.sreg    <= '1' & tx.sreg(tx.sreg'left downto 1);
          if (tx.bitcnt = "0001") then
            tx.busy <= '0';
          end if;
        else
          tx.baudcnt <= std_ulogic_vector(unsigned(tx.baudcnt) - 1);
        end if;
      end if;
    end if;
  end process;

  -- TX pin control --
  tx_output: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then -- reset for primary output
      uart_txd_o <= '1';
    elsif rising_edge(clk_i) then
      if (ctrl.tx_en = '0') or (tx.busy = '0') then -- disabled or idle
        uart_txd_o <= '1';
      else
        uart_txd_o <= tx.sreg(0);
      end if;
    end if;
  end process;


  -- Transmitter (TX) - Simulation-Mode: Print TX data to simulator console -----------------
  -- -------------------------------------------------------------------------------------------
-- pragma translate_off
-- RTL_SYNTHESIS OFF
  tx_sim_enabled:
  if is_simulation_c generate
    tx_sim: process(clk_i)
      variable char_v : integer;
      variable line_v : line;
    begin
      if rising_edge(clk_i) then
        if (ctrl.tx_sim_en = '1') and (tx_fifo.we = '1') then
          char_v := to_integer(unsigned(tx_fifo.wdata));
          if (char_v >= 128) then -- non-printable?
            char_v := 0;
          end if;
          if (char_v /= 10) and (char_v /= 13) then -- skip line breaks
            write(line_v, character'val(char_v));
          elsif (char_v = 10) then -- flush line to screen
            writeline(output, line_v);
          end if;
        end if;
      end if;
    end process;
  end generate;
-- RTL_SYNTHESIS ON
-- pragma translate_on

end architecture;
