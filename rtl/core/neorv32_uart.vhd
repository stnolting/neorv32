-- #################################################################################################
-- # << NEORV32 - Universal Asynchronous Receiver and Transmitter (UART0/1) >>                     #
-- # ********************************************************************************************* #
-- # Frame configuration: 1 start bit, 8 bit data, parity bit (none/even/odd), 1 stop bit,         #
-- # programmable BAUD rate via clock pre-scaler and 12-bit BAUD value configuration register,     #
-- # optional configurable RX and TX FIFOs.                                                        #
-- #                                                                                               #
-- # Interrupts: Configurable RX and TX interrupt (both triggered by specific FIFO fill-levels)    #
-- #                                                                                               #
-- # Support for RTS("RTR")/CTS hardware flow control:                                             #
-- # * uart_rts_o = 0: RX is ready to receive a new char, enabled via CTRL.ctrl_rts_en_c           #
-- # * uart_cts_i = 0: TX is allowed to send a new char, enabled via CTRL.ctrl_cts_en_c            #
-- #                                                                                               #
-- # UART0 / UART1:                                                                                #
-- # This module is used for implementing UART0 and UART1. The UART_PRIMARY generic configures the #
-- # interface register addresses and simulation outputs for UART0 (UART_PRIMARY = true) or UART1  #
-- # (UART_PRIMARY = false).                                                                       #
-- #                                                                                               #
-- # SIMULATION MODE:                                                                              #
-- # When the simulation mode is enabled (setting the ctrl.ctrl_sim_en_c bit) any write            #
-- # access to the TX register will not trigger any UART activity. Instead, the written data is    #
-- # output to the simulation environment. The lowest 8 bits of the written data are printed as    #
-- # ASCII char to the simulator console.                                                          #
-- # This char is also stored to the file "neorv32.uartX.sim_mode.text.out" (where X = 0 for UART0 #
-- # and X = 1 for UART1). The full 32-bit write data is also stored as 8-digit hexadecimal value  #
-- # to the file "neorv32.uartX.sim_mode.data.out" (where X = 0 for UART0 and X = 1 for UART1).    #
-- # No interrupts are triggered when in SIMULATION MODE.                                          #
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
use std.textio.all;

entity neorv32_uart is
  generic (
    UART_PRIMARY : boolean; -- true = primary UART (UART0), false = secondary UART (UART1)
    UART_RX_FIFO : natural; -- RX fifo depth, has to be a power of two, min 1
    UART_TX_FIFO : natural  -- TX fifo depth, has to be a power of two, min 1
  );
  port (
    -- host access --
    clk_i       : in  std_ulogic; -- global clock line
    addr_i      : in  std_ulogic_vector(31 downto 0); -- address
    rden_i      : in  std_ulogic; -- read enable
    wren_i      : in  std_ulogic; -- write enable
    data_i      : in  std_ulogic_vector(31 downto 0); -- data in
    data_o      : out std_ulogic_vector(31 downto 0); -- data out
    ack_o       : out std_ulogic; -- transfer acknowledge
    -- clock generator --
    clkgen_en_o : out std_ulogic; -- enable clock generator
    clkgen_i    : in  std_ulogic_vector(07 downto 0);
    -- com lines --
    uart_txd_o  : out std_ulogic;
    uart_rxd_i  : in  std_ulogic;
    -- hardware flow control --
    uart_rts_o  : out std_ulogic; -- UART.RX ready to receive ("RTR"), low-active, optional
    uart_cts_i  : in  std_ulogic; -- UART.TX allowed to transmit, low-active, optional
    -- interrupts --
    irq_rxd_o   : out std_ulogic; -- uart data received interrupt
    irq_txd_o   : out std_ulogic  -- uart transmission done interrupt
  );
end neorv32_uart;

architecture neorv32_uart_rtl of neorv32_uart is

  -- interface configuration for UART0 / UART1 --
  constant uart_id_base_c      : std_ulogic_vector(data_width_c-1 downto 0) := cond_sel_stdulogicvector_f(UART_PRIMARY, uart0_base_c,      uart1_base_c);
  constant uart_id_size_c      : natural                                    := cond_sel_natural_f(        UART_PRIMARY, uart0_size_c,      uart1_size_c);
  constant uart_id_ctrl_addr_c : std_ulogic_vector(data_width_c-1 downto 0) := cond_sel_stdulogicvector_f(UART_PRIMARY, uart0_ctrl_addr_c, uart1_ctrl_addr_c);
  constant uart_id_rtx_addr_c  : std_ulogic_vector(data_width_c-1 downto 0) := cond_sel_stdulogicvector_f(UART_PRIMARY, uart0_rtx_addr_c,  uart1_rtx_addr_c);

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(uart_id_size_c); -- low address boundary bit

  -- simulation output configuration --
  constant sim_screen_output_en_c : boolean := true; -- output lowest byte as char to simulator console when enabled
  constant sim_text_output_en_c   : boolean := true; -- output lowest byte as char to text file when enabled
  constant sim_data_output_en_c   : boolean := true; -- dump 32-bit TX word to file when enabled
  constant sim_uart_text_file_c   : string  := cond_sel_string_f(UART_PRIMARY, "neorv32.uart0.sim_mode.text.out", "neorv32.uart1.sim_mode.text.out");
  constant sim_uart_data_file_c   : string  := cond_sel_string_f(UART_PRIMARY, "neorv32.uart0.sim_mode.data.out", "neorv32.uart1.sim_mode.data.out");

  -- control register --
  signal ctrl : std_ulogic_vector(31 downto 0);

  -- control register bits --
  constant ctrl_baud00_c   : natural :=  0; -- r/w: baud config bit 0
  constant ctrl_baud01_c   : natural :=  1; -- r/w: baud config bit 1
  constant ctrl_baud02_c   : natural :=  2; -- r/w: baud config bit 2
  constant ctrl_baud03_c   : natural :=  3; -- r/w: baud config bit 3
  constant ctrl_baud04_c   : natural :=  4; -- r/w: baud config bit 4
  constant ctrl_baud05_c   : natural :=  5; -- r/w: baud config bit 5
  constant ctrl_baud06_c   : natural :=  6; -- r/w: baud config bit 6
  constant ctrl_baud07_c   : natural :=  7; -- r/w: baud config bit 7
  constant ctrl_baud08_c   : natural :=  8; -- r/w: baud config bit 8
  constant ctrl_baud09_c   : natural :=  9; -- r/w: baud config bit 9
  constant ctrl_baud10_c   : natural := 10; -- r/w: baud config bit 10
  constant ctrl_baud11_c   : natural := 11; -- r/w: baud config bit 11
  constant ctrl_sim_en_c   : natural := 12; -- r/w: UART <<SIMULATION MODE>> enable
  constant ctrl_rx_empty_c : natural := 13; -- r/-: RX FIFO is empty
  constant ctrl_rx_half_c  : natural := 14; -- r/-: RX FIFO is at least half-full
  constant ctrl_rx_full_c  : natural := 15; -- r/-: RX FIFO is full
  constant ctrl_tx_empty_c : natural := 16; -- r/-: TX FIFO is empty
  constant ctrl_tx_half_c  : natural := 17; -- r/-: TX FIFO is at least half-full
  constant ctrl_tx_full_c  : natural := 18; -- r/-: TX FIFO is full
  -- ...
  constant ctrl_rts_en_c   : natural := 20; -- r/w: enable hardware flow control: assert rts_o if ready to receive
  constant ctrl_cts_en_c   : natural := 21; -- r/w: enable hardware flow control: send only if cts_i is asserted
  constant ctrl_pmode0_c   : natural := 22; -- r/w: Parity config (0=even; 1=odd)
  constant ctrl_pmode1_c   : natural := 23; -- r/w: Enable parity bit
  constant ctrl_prsc0_c    : natural := 24; -- r/w: baud prsc bit 0
  constant ctrl_prsc1_c    : natural := 25; -- r/w: baud prsc bit 1
  constant ctrl_prsc2_c    : natural := 26; -- r/w: baud prsc bit 2
  constant ctrl_cts_c      : natural := 27; -- r/-: current state of CTS input
  constant ctrl_en_c       : natural := 28; -- r/w: UART enable
  constant ctrl_rx_irq_c   : natural := 29; -- r/w: RX IRQ mode: 1=FIFO at least half-full; 0=FIFO not empty
  constant ctrl_tx_irq_c   : natural := 30; -- r/w: TX IRQ mode: 1=FIFO less than half-full; 0=FIFO not full
  constant ctrl_tx_busy_c  : natural := 31; -- r/-: UART transmitter is busy

  -- data register flags --
  constant data_lsb_c      : natural :=  0; -- r/-: received char LSB
  constant data_msb_c      : natural :=  7; -- r/-: received char MSB
  -- ...
  constant data_rx_perr_c  : natural := 28; -- r/-: RX parity error
  constant data_rx_ferr_c  : natural := 29; -- r/-: RX frame error
  constant data_rx_overr_c : natural := 30; -- r/-: RX data overrun
  constant data_rx_avail_c : natural := 31; -- r/-: RX data available

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- clock generator --
  signal uart_clk : std_ulogic;

  -- numbers of bits in transmission frame --
  signal num_bits : std_ulogic_vector(3 downto 0);

  -- hardware flow-control IO buffer --
  signal uart_cts_ff : std_ulogic_vector(1 downto 0);
  signal uart_rts    : std_ulogic;

  -- UART transmitter --
  type tx_state_t is (S_TX_IDLE, S_TX_GET, S_TX_CHECK, S_TX_TRANSMIT, S_TX_SIM);
  type tx_engine_t is record
    state    : tx_state_t;
    busy     : std_ulogic;
    done     : std_ulogic;
    bitcnt   : std_ulogic_vector(03 downto 0);
    sreg     : std_ulogic_vector(10 downto 0);
    baud_cnt : std_ulogic_vector(11 downto 0);
    cts      : std_ulogic; -- allow new transmission when 1
  end record;
  signal tx_engine : tx_engine_t;

  -- UART receiver --
  type rx_state_t is (S_RX_IDLE, S_RX_RECEIVE);
  type rx_engine_t is record
    state    : rx_state_t;
    done     : std_ulogic;
    sync     : std_ulogic_vector(04 downto 0);
    bitcnt   : std_ulogic_vector(03 downto 0);
    sreg     : std_ulogic_vector(09 downto 0);
    baud_cnt : std_ulogic_vector(11 downto 0);
    overr    : std_ulogic;
    rtr      : std_ulogic; -- ready to receive when 1
  end record;
  signal rx_engine : rx_engine_t;

  -- TX FIFO --
  type tx_buffer_t is record
    we    : std_ulogic; -- write enable
    re    : std_ulogic; -- read enable
    clear : std_ulogic; -- sync reset, high-active
    wdata : std_ulogic_vector(31 downto 0); -- write data
    rdata : std_ulogic_vector(31 downto 0); -- read data
    avail : std_ulogic; -- data available?
    free  : std_ulogic; -- free entry available?
    half  : std_ulogic; -- half full
  end record;
  signal tx_buffer : tx_buffer_t;

  -- RX FIFO --
  type rx_buffer_t is record
    we    : std_ulogic; -- write enable
    re    : std_ulogic; -- read enable
    clear : std_ulogic; -- sync reset, high-active
    wdata : std_ulogic_vector(9 downto 0); -- write data
    rdata : std_ulogic_vector(9 downto 0); -- read data
    avail : std_ulogic; -- data available?
    free  : std_ulogic; -- free entry available?
    half  : std_ulogic; -- half full
  end record;
  signal rx_buffer : rx_buffer_t;

  -- interrupt generator --
  type irq_t is record
    set : std_ulogic;
    buf : std_ulogic_vector(1 downto 0);
  end record;
  signal rx_irq, tx_irq : irq_t;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (is_power_of_two_f(UART_RX_FIFO) = false) report "NEORV32 PROCESSOR CONFIG ERROR: UART" &
  cond_sel_string_f(UART_PRIMARY, "0", "1") & " <UART_RX_FIFO> has to be a power of two." severity error;
  assert not (is_power_of_two_f(UART_TX_FIFO) = false) report "NEORV32 PROCESSOR CONFIG ERROR: UART" &
  cond_sel_string_f(UART_PRIMARY, "0", "1") & " <UART_TX_FIFO> has to be a power of two." severity error;


  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = uart_id_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= uart_id_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
  wren   <= acc_en and wren_i;
  rden   <= acc_en and rden_i;


  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rw_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- bus access acknowledge --
      ack_o <= wren or rden;

      -- write access --
      if (wren = '1') then
        if (addr = uart_id_ctrl_addr_c) then
          ctrl <= (others => '0');
          ctrl(ctrl_baud11_c downto ctrl_baud00_c) <= data_i(ctrl_baud11_c downto ctrl_baud00_c);
          ctrl(ctrl_sim_en_c)                      <= data_i(ctrl_sim_en_c);
          ctrl(ctrl_pmode1_c downto ctrl_pmode0_c) <= data_i(ctrl_pmode1_c downto ctrl_pmode0_c);
          ctrl(ctrl_prsc2_c  downto ctrl_prsc0_c)  <= data_i(ctrl_prsc2_c  downto ctrl_prsc0_c);
          ctrl(ctrl_rts_en_c)                      <= data_i(ctrl_rts_en_c);
          ctrl(ctrl_cts_en_c)                      <= data_i(ctrl_cts_en_c);
          ctrl(ctrl_rx_irq_c)                      <= data_i(ctrl_rx_irq_c);
          ctrl(ctrl_tx_irq_c)                      <= data_i(ctrl_tx_irq_c);
          ctrl(ctrl_en_c)                          <= data_i(ctrl_en_c);
        end if;
      end if;

      -- read access --
      data_o <= (others => '0');
      if (rden = '1') then
        if (addr = uart_id_ctrl_addr_c) then
          data_o(ctrl_baud11_c downto ctrl_baud00_c) <= ctrl(ctrl_baud11_c downto ctrl_baud00_c);
          data_o(ctrl_sim_en_c)                      <= ctrl(ctrl_sim_en_c);
          data_o(ctrl_pmode1_c downto ctrl_pmode0_c) <= ctrl(ctrl_pmode1_c downto ctrl_pmode0_c);
          data_o(ctrl_prsc2_c  downto ctrl_prsc0_c)  <= ctrl(ctrl_prsc2_c  downto ctrl_prsc0_c);
          data_o(ctrl_rts_en_c)                      <= ctrl(ctrl_rts_en_c);
          data_o(ctrl_cts_en_c)                      <= ctrl(ctrl_cts_en_c);
          data_o(ctrl_rx_empty_c)                    <= not rx_buffer.avail;
          data_o(ctrl_rx_half_c)                     <= rx_buffer.half;
          data_o(ctrl_rx_full_c)                     <= not rx_buffer.free;
          data_o(ctrl_tx_empty_c)                    <= not tx_buffer.avail;
          data_o(ctrl_tx_half_c)                     <= tx_buffer.half;
          data_o(ctrl_tx_full_c)                     <= not tx_buffer.free;
          data_o(ctrl_en_c)                          <= ctrl(ctrl_en_c);
          data_o(ctrl_rx_irq_c)                      <= ctrl(ctrl_rx_irq_c) and bool_to_ulogic_f(boolean(UART_RX_FIFO > 1)); -- tie to zero if UART_RX_FIFO = 1
          data_o(ctrl_tx_irq_c)                      <= ctrl(ctrl_tx_irq_c) and bool_to_ulogic_f(boolean(UART_TX_FIFO > 1)); -- tie to zero if UART_TX_FIFO = 1
          data_o(ctrl_tx_busy_c)                     <= tx_engine.busy;
          data_o(ctrl_cts_c)                         <= uart_cts_ff(1);
        else -- uart_id_rtx_addr_c
          data_o(data_msb_c downto data_lsb_c) <= rx_buffer.rdata(7 downto 0);
          data_o(data_rx_perr_c)               <= rx_buffer.rdata(8);
          data_o(data_rx_ferr_c)               <= rx_buffer.rdata(9);
          data_o(data_rx_overr_c)              <= rx_engine.overr;
          data_o(data_rx_avail_c)              <= rx_buffer.avail; -- data available (valid?)
        end if;
      end if;
    end if;
  end process rw_access;

  -- number of bits to be sampled --
  -- if parity flag is ENABLED:  11 bit -> "1011" (1 start bit + 8 data bits + 1 parity bit + 1 stop bit)
  -- if parity flag is DISABLED: 10 bit -> "1010" (1 start bit + 8 data bits + 1 stop bit)
  num_bits <= "1011" when (ctrl(ctrl_pmode1_c) = '1') else "1010";


  -- Clock Selection ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- clock enable --
  clkgen_en_o <= ctrl(ctrl_en_c);

  -- uart clock select --
  uart_clk <= clkgen_i(to_integer(unsigned(ctrl(ctrl_prsc2_c downto ctrl_prsc0_c))));


  -- TX FIFO --------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tx_engine_fifo_inst: neorv32_fifo
  generic map (
    FIFO_DEPTH => UART_TX_FIFO, -- number of fifo entries; has to be a power of two; min 1
    FIFO_WIDTH => 32,           -- size of data elements in fifo (32-bit only for simulation)
    FIFO_RSYNC => false,        -- async read
    FIFO_SAFE  => true          -- safe access
  )
  port map (
    -- control --
    clk_i   => clk_i,           -- clock, rising edge
    rstn_i  => '1',             -- async reset, low-active
    clear_i => tx_buffer.clear, -- sync reset, high-active
    level_o => open,
    half_o  => tx_buffer.half,  -- FIFO at least half-full
    -- write port --
    wdata_i => tx_buffer.wdata, -- write data
    we_i    => tx_buffer.we,    -- write enable
    free_o  => tx_buffer.free,  -- at least one entry is free when set
    -- read port --
    re_i    => tx_buffer.re,    -- read enable
    rdata_o => tx_buffer.rdata, -- read data
    avail_o => tx_buffer.avail  -- data available when set
  );

  -- control --
  tx_buffer.clear <= not ctrl(ctrl_en_c);

  -- write access --
  tx_buffer.we    <= '1' when (wren = '1') and (addr = uart_id_rtx_addr_c) else '0';
  tx_buffer.wdata <= data_i;


  -- UART Transmitter Engine ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  uart_tx_engine: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- defaults --
      uart_txd_o     <= '1'; -- keep TX line idle (=high) if waiting for permission to start sending (->CTS)
      tx_buffer.re   <= '0';
      tx_engine.done <= '0';

      -- FSM --
      if (ctrl(ctrl_en_c) = '0') then -- disabled
        tx_engine.state <= S_TX_IDLE;
      else
        case tx_engine.state is

          when S_TX_IDLE => -- wait for new data to send
          -- ------------------------------------------------------------
            if (tx_buffer.avail = '1') then -- new data available
              if (ctrl(ctrl_sim_en_c) = '0') then -- normal mode
                tx_engine.state <= S_TX_GET;
              else -- simulation mode
                tx_engine.state <= S_TX_SIM;
              end if;
              tx_buffer.re <= '1';
            end if;

          when S_TX_GET => -- get new data from buffer and prepare transmission
          -- ------------------------------------------------------------
            tx_engine.baud_cnt <= ctrl(ctrl_baud11_c downto ctrl_baud00_c);
            tx_engine.bitcnt   <= num_bits;
            if (ctrl(ctrl_pmode1_c) = '1') then -- add parity flag
              -- stop bit & parity bit & data (8-bit) & start bit
              tx_engine.sreg <= '1' & (xor_reduce_f(tx_buffer.rdata(7 downto 0)) xor ctrl(ctrl_pmode0_c)) & tx_buffer.rdata(7 downto 0) & '0';
            else
              -- (dummy fill-bit &) stop bit & data (8-bit) & start bit
              tx_engine.sreg <= '1' & '1' & tx_buffer.rdata(7 downto 0) & '0';
            end if;
            tx_engine.state <= S_TX_CHECK;

          when S_TX_CHECK => -- check if allowed to send
          -- ------------------------------------------------------------
            if (tx_engine.cts = '1') then -- clear to send
              tx_engine.state <= S_TX_TRANSMIT;
            end if;

          when S_TX_TRANSMIT => -- transmit data
          -- ------------------------------------------------------------
            if (uart_clk = '1') then
              if (or_reduce_f(tx_engine.baud_cnt) = '0') then -- bit done?
                tx_engine.baud_cnt <= ctrl(ctrl_baud11_c downto ctrl_baud00_c);
                tx_engine.bitcnt   <= std_ulogic_vector(unsigned(tx_engine.bitcnt) - 1);
                tx_engine.sreg     <= '1' & tx_engine.sreg(tx_engine.sreg'left downto 1);
              else
                tx_engine.baud_cnt <= std_ulogic_vector(unsigned(tx_engine.baud_cnt) - 1);
              end if;
            end if;
            uart_txd_o <= tx_engine.sreg(0);
            if (or_reduce_f(tx_engine.bitcnt) = '0') then -- all bits send?
              tx_engine.done  <= '1'; -- sending done
              tx_engine.state <= S_TX_IDLE;
            end if;

          when S_TX_SIM => -- simulation mode output
          -- ------------------------------------------------------------
            tx_engine.state <= S_TX_IDLE;

          when others => -- undefined
          -- ------------------------------------------------------------
            tx_engine.state <= S_TX_IDLE;

        end case;
      end if;
    end if;
  end process uart_tx_engine;

  -- transmitter busy --
  tx_engine.busy <= '0' when (tx_engine.state = S_TX_IDLE) else '1';


  -- UART Receiver Engine -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  uart_rx_engine: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- input synchronizer --
      rx_engine.sync <= uart_rxd_i & rx_engine.sync(rx_engine.sync'left downto 1);

      -- default --
      rx_engine.done <= '0';

      -- FSM --
      if (ctrl(ctrl_en_c) = '0') then -- disabled
        rx_engine.overr <= '0';
        rx_engine.state <= S_RX_IDLE;
      else
        case rx_engine.state is

          when S_RX_IDLE => -- idle; prepare receive
          -- ------------------------------------------------------------
            rx_engine.baud_cnt <= '0' & ctrl(ctrl_baud11_c downto ctrl_baud01_c); -- half baud delay at the beginning to sample in the middle of each bit
            rx_engine.bitcnt   <= num_bits;
            if (rx_engine.sync(3 downto 0) = "0011") then -- start bit? (falling edge)
              rx_engine.state <= S_RX_RECEIVE;
            end if;

          when S_RX_RECEIVE => -- receive data
          -- ------------------------------------------------------------
            if (uart_clk = '1') then
              if (or_reduce_f(rx_engine.baud_cnt) = '0') then -- bit done
                rx_engine.baud_cnt <= ctrl(ctrl_baud11_c downto ctrl_baud00_c);
                rx_engine.bitcnt   <= std_ulogic_vector(unsigned(rx_engine.bitcnt) - 1);
                rx_engine.sreg     <= rx_engine.sync(2) & rx_engine.sreg(rx_engine.sreg'left downto 1);
              else
                rx_engine.baud_cnt <= std_ulogic_vector(unsigned(rx_engine.baud_cnt) - 1);
              end if;
            end if;
            if (or_reduce_f(rx_engine.bitcnt) = '0') then -- all bits received?
              rx_engine.done  <= '1'; -- receiving done
              rx_engine.state <= S_RX_IDLE;
            end if;

          when others => -- undefined
          -- ------------------------------------------------------------
            rx_engine.state <= S_RX_IDLE;

        end case;

        -- overrun flag --
        if (rden = '1') and (addr = uart_id_rtx_addr_c) then -- clear when reading data register
          rx_engine.overr <= '0';
        elsif (rx_buffer.we = '1') and (rx_buffer.free = '0') then -- write to full FIFO
          rx_engine.overr <= '1';
        end if;
      end if;
    end if;
  end process uart_rx_engine;

  -- RX engine ready for a new char? --
  rx_engine.rtr <= '1' when (rx_engine.state = S_RX_IDLE) and (ctrl(ctrl_en_c) = '1') else '0';


  -- RX FIFO --------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rx_engine_fifo_inst: neorv32_fifo
  generic map (
    FIFO_DEPTH => UART_RX_FIFO, -- number of fifo entries; has to be a power of two; min 1
    FIFO_WIDTH => 10,           -- size of data elements in fifo
    FIFO_RSYNC => false,        -- async read
    FIFO_SAFE  => true          -- safe access
  )
  port map (
    -- control --
    clk_i   => clk_i,           -- clock, rising edge
    rstn_i  => '1',             -- async reset, low-active
    clear_i => rx_buffer.clear, -- sync reset, high-active
    level_o => open,
    half_o  => rx_buffer.half,  -- FIFO at least half-full
    -- write port --
    wdata_i => rx_buffer.wdata, -- write data
    we_i    => rx_buffer.we,    -- write enable
    free_o  => rx_buffer.free,  -- at least one entry is free when set
    -- read port --
    re_i    => rx_buffer.re,    -- read enable
    rdata_o => rx_buffer.rdata, -- read data
    avail_o => rx_buffer.avail  -- data available when set
  );

  -- control --
  rx_buffer.clear <= not ctrl(ctrl_en_c);

  -- read/write access --
  rx_buffer.wdata(7 downto 0) <= rx_engine.sreg(7 downto 0) when (ctrl(ctrl_pmode1_c) = '1') else rx_engine.sreg(8 downto 1); -- RX data
  rx_buffer.wdata(8) <= ctrl(ctrl_pmode1_c) and (xor_reduce_f(rx_engine.sreg(8 downto 0)) xor ctrl(ctrl_pmode0_c)); -- parity error flag
  rx_buffer.wdata(9) <= not rx_engine.sreg(9); -- frame error flag: check stop bit (error if not set)
  rx_buffer.we <= '1' when (rx_engine.bitcnt = "0000") and (rx_engine.state = S_RX_RECEIVE) else '0'; -- RX complete
  rx_buffer.re <= '1' when (rden = '1') and (addr = uart_id_rtx_addr_c) else '0';


  -- Hardware Flow Control ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tx_engine.cts <= (not uart_cts_ff(1)) when (ctrl(ctrl_cts_en_c) = '1') else '1'; -- input is low-active, internal signal is high-active
  uart_rts      <= (not rx_engine.rtr)  when (ctrl(ctrl_rts_en_c) = '1') else '0'; -- output is low-active

  -- flow-control input/output synchronizer --
  flow_control_buffer: process(clk_i)
  begin
    if rising_edge(clk_i) then -- should be mapped to IOBs
      uart_cts_ff <= uart_cts_ff(0) & uart_cts_i;
      uart_rts_o  <= uart_rts;
    end if;
  end process flow_control_buffer;


  -- Interrupt Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_type: process(ctrl, tx_buffer, rx_buffer, tx_engine.done)
  begin
    -- TX interrupt --
    if (UART_TX_FIFO = 1) or (ctrl(ctrl_tx_irq_c) = '0') then
      tx_irq.set <= tx_buffer.free and tx_engine.done; -- fire IRQ if FIFO is not full
    else
      tx_irq.set <= (not tx_buffer.half) and tx_engine.done; -- fire IRQ if FIFO is less than half-full
    end if;
    -- RX interrupt --
    if (UART_RX_FIFO = 1) or (ctrl(ctrl_rx_irq_c) = '0') then
      rx_irq.set <= rx_buffer.avail; -- fire IRQ if FIFO is not empty
    else
      rx_irq.set <= rx_buffer.half; -- fire IRQ if FIFO is at least half-full
    end if;
  end process irq_type;

  -- interrupt edge detector --
  irq_detect: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (ctrl(ctrl_en_c) = '0') then
        tx_irq.buf <= "00";
        rx_irq.buf <= "00";
      else
        tx_irq.buf <= tx_irq.buf(0) & tx_irq.set;
        rx_irq.buf <= rx_irq.buf(0) & rx_irq.set;
      end if;
    end if;
  end process irq_detect;

  -- IRQ requests to CPU --
  irq_txd_o <= '1' when (tx_irq.buf = "01") else '0';
  irq_rxd_o <= '1' when (rx_irq.buf = "01") else '0';


  -- SIMULATION Transmitter -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
-- pragma translate_off
-- synthesis translate_off
-- RTL_SYNTHESIS OFF
  sim_output: process(clk_i) -- for SIMULATION ONLY!
    file file_uart_text_out : text open write_mode is sim_uart_text_file_c;
    file file_uart_data_out : text open write_mode is sim_uart_data_file_c;
    variable char_v         : integer;
    variable line_screen_v  : line; -- we need several line variables here since "writeline" seems to flush the source variable
    variable line_text_v    : line;
    variable line_data_v    : line;
  begin
    if rising_edge(clk_i) then
      if (tx_engine.state = S_TX_SIM) then -- UART simulation mode
        
        -- print lowest byte as ASCII char --
        char_v := to_integer(unsigned(tx_buffer.rdata(7 downto 0)));
        if (char_v >= 128) then -- out of range?
          char_v := 0;
        end if;

        if (char_v /= 10) and (char_v /= 13) then -- skip line breaks - they are issued via "writeline"
          if (sim_screen_output_en_c = true) then
            write(line_screen_v, character'val(char_v));
          end if;
          if (sim_text_output_en_c = true) then
            write(line_text_v, character'val(char_v));
          end if;
        end if;

        if (char_v = 10) then -- line break: write to screen and text file
          if (sim_screen_output_en_c = true) then
            writeline(output, line_screen_v);
          end if;
          if (sim_text_output_en_c = true) then
            writeline(file_uart_text_out, line_text_v);
          end if;
        end if;

        -- dump raw data as 8 hex chars to file --
        if (sim_data_output_en_c = true) then
          for x in 7 downto 0 loop
            write(line_data_v, to_hexchar_f(tx_buffer.rdata(3+x*4 downto 0+x*4))); -- write in hex form
          end loop; -- x
          writeline(file_uart_data_out, line_data_v);
        end if;

      end if;
    end if;
  end process sim_output;
-- RTL_SYNTHESIS ON
-- synthesis translate_on
-- pragma translate_on

end neorv32_uart_rtl;
