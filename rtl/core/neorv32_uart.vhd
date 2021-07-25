-- #################################################################################################
-- # << NEORV32 - Universal Asynchronous Receiver and Transmitter (UART0/1) >>                     #
-- # ********************************************************************************************* #
-- # Frame configuration: 1 start bit, 8 bit data, parity bit (none/even/odd), 1 stop bit,         #
-- # programmable BAUD rate via clock pre-scaler and 12-bit BAUD value config register. RX engine  #
-- # a simple 2-entry data buffer (for double-buffering).                                          #
-- # Interrupts: UART_RX_available, UART_TX_done                                                   #
-- #                                                                                               #
-- # Support for RTS("RTR")/CTS hardware flow control:                                             #
-- # * uart_rts_o = 0: RX is ready to receive a new char, enabled via CTRL.ctrl_uart_rts_en_c      #
-- # * uart_cts_i = 0: TX is allowed to send a new char, enabled via CTRL.ctrl_uart_cts_en_c       #
-- #                                                                                               #
-- # UART0 / UART1:                                                                                #
-- # This module is used for implementing UART0 and UART1. The UART_PRIMARY generic configures the #
-- # interface register addresses and simulation output setting for UART0 (UART_PRIMARY = true)    #
-- # or UART1 (UART_PRIMARY = false).                                                              #
-- #                                                                                               #
-- # SIMULATION MODE:                                                                              #
-- # When the simulation mode is enabled (setting the ctrl.ctrl_uart_sim_en_c bit) any write       #
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
use std.textio.all; -- obviously only for simulation

entity neorv32_uart is
  generic (
    UART_PRIMARY : boolean -- true = primary UART (UART0), false = secondary UART (UART1)
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
  constant sim_data_output_en_c   : boolean := true; -- dump 32-word to file when enabled

  -- simulation output file configuration --
  constant sim_uart_text_file_c : string := cond_sel_string_f(UART_PRIMARY, "neorv32.uart0.sim_mode.text.out", "neorv32.uart1.sim_mode.text.out");
  constant sim_uart_data_file_c : string := cond_sel_string_f(UART_PRIMARY, "neorv32.uart0.sim_mode.data.out", "neorv32.uart1.sim_mode.data.out");

  -- control register --
  signal ctrl : std_ulogic_vector(31 downto 0);

  -- control register bits --
  constant ctrl_uart_baud00_c  : natural :=  0; -- r/w: UART baud config bit 0
  constant ctrl_uart_baud01_c  : natural :=  1; -- r/w: UART baud config bit 1
  constant ctrl_uart_baud02_c  : natural :=  2; -- r/w: UART baud config bit 2
  constant ctrl_uart_baud03_c  : natural :=  3; -- r/w: UART baud config bit 3
  constant ctrl_uart_baud04_c  : natural :=  4; -- r/w: UART baud config bit 4
  constant ctrl_uart_baud05_c  : natural :=  5; -- r/w: UART baud config bit 5
  constant ctrl_uart_baud06_c  : natural :=  6; -- r/w: UART baud config bit 6
  constant ctrl_uart_baud07_c  : natural :=  7; -- r/w: UART baud config bit 7
  constant ctrl_uart_baud08_c  : natural :=  8; -- r/w: UART baud config bit 8
  constant ctrl_uart_baud09_c  : natural :=  9; -- r/w: UART baud config bit 9
  constant ctrl_uart_baud10_c  : natural := 10; -- r/w: UART baud config bit 10
  constant ctrl_uart_baud11_c  : natural := 11; -- r/w: UART baud config bit 11
  constant ctrl_uart_sim_en_c  : natural := 12; -- r/w: UART <<SIMULATION MODE>> enable
  -- ...
  constant ctrl_uart_rts_en_c  : natural := 20; -- r/w: enable hardware flow control: assert rts_o if ready to receive
  constant ctrl_uart_cts_en_c  : natural := 21; -- r/w: enable hardware flow control: send only if cts_i is asserted
  constant ctrl_uart_pmode0_c  : natural := 22; -- r/w: Parity config (0=even; 1=odd)
  constant ctrl_uart_pmode1_c  : natural := 23; -- r/w: Enable parity bit
  constant ctrl_uart_prsc0_c   : natural := 24; -- r/w: UART baud prsc bit 0
  constant ctrl_uart_prsc1_c   : natural := 25; -- r/w: UART baud prsc bit 1
  constant ctrl_uart_prsc2_c   : natural := 26; -- r/w: UART baud prsc bit 2
  constant ctrl_uart_cts_c     : natural := 27; -- r/-: current state of CTS input
  constant ctrl_uart_en_c      : natural := 28; -- r/w: UART enable
  -- ...
  constant ctrl_uart_tx_busy_c : natural := 31; -- r/-: UART transmitter is busy

  -- data register flags --
  constant data_rx_perr_c  : natural := 28; -- r/-: Rx parity error
  constant data_rx_ferr_c  : natural := 29; -- r/-: Rx frame error
  constant data_rx_overr_c : natural := 30; -- r/-: Rx data overrun
  constant data_rx_avail_c : natural := 31; -- r/-: Rx data available

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wr_en  : std_ulogic; -- word write enable
  signal rd_en  : std_ulogic; -- read enable

  -- clock generator --
  signal uart_clk : std_ulogic;

  -- numbers of bits in transmission frame --
  signal num_bits : std_ulogic_vector(03 downto 0);

  -- hardware flow-control IO buffer --
  signal uart_cts_ff : std_ulogic_vector(1 downto 0);
  signal uart_rts    : std_ulogic;

  -- uart tx unit --
  type uart_tx_t is record
    busy       : std_ulogic;
    done       : std_ulogic;
    bitcnt     : std_ulogic_vector(03 downto 0);
    sreg       : std_ulogic_vector(10 downto 0);
    baud_cnt   : std_ulogic_vector(11 downto 0);
    tx_granted : std_ulogic; -- allowed to start sending when 1
    cts        : std_ulogic; -- allow new transmission when 1
  end record;
  signal uart_tx : uart_tx_t;

  -- uart rx unit --
  type ry_data_buf_t is array (0 to 1) of std_ulogic_vector(07 downto 0);
  type uart_rx_t is record
    sync     : std_ulogic_vector(04 downto 0);
    busy     : std_ulogic;
    busy_ff  : std_ulogic;
    bitcnt   : std_ulogic_vector(03 downto 0);
    sreg     : std_ulogic_vector(09 downto 0);
    baud_cnt : std_ulogic_vector(11 downto 0);
    rtr      : std_ulogic; -- ready to receive when 1
    --
    avail    : std_ulogic_vector(02 downto 0);
    data     : ry_data_buf_t;
    data_rd  : std_ulogic_vector(07 downto 0);
    ferr     : std_ulogic_vector(01 downto 0); -- frame error (stop bit not set)
    ferr_rd  : std_ulogic;
    perr     : std_ulogic_vector(01 downto 0); -- parity error
    perr_rd  : std_ulogic;
  end record;
  signal uart_rx : uart_rx_t;

begin

  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = uart_id_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= uart_id_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
  wr_en  <= acc_en and wren_i;
  rd_en  <= acc_en and rden_i;


  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rw_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      ack_o <= acc_en and (rden_i or wren_i);
      -- write access --
      if (wr_en = '1') then
        if (addr = uart_id_ctrl_addr_c) then
          ctrl <= (others => '0');
          ctrl(ctrl_uart_baud11_c downto ctrl_uart_baud00_c) <= data_i(ctrl_uart_baud11_c downto ctrl_uart_baud00_c);
          ctrl(ctrl_uart_sim_en_c)                           <= data_i(ctrl_uart_sim_en_c);
          ctrl(ctrl_uart_pmode1_c downto ctrl_uart_pmode0_c) <= data_i(ctrl_uart_pmode1_c downto ctrl_uart_pmode0_c);
          ctrl(ctrl_uart_prsc2_c  downto ctrl_uart_prsc0_c)  <= data_i(ctrl_uart_prsc2_c  downto ctrl_uart_prsc0_c);
          ctrl(ctrl_uart_rts_en_c)                           <= data_i(ctrl_uart_rts_en_c);
          ctrl(ctrl_uart_cts_en_c)                           <= data_i(ctrl_uart_cts_en_c);
          ctrl(ctrl_uart_en_c)                               <= data_i(ctrl_uart_en_c);
        end if;
      end if;
      -- read access --
      data_o <= (others => '0');
      if (rd_en = '1') then
        if (addr = uart_id_ctrl_addr_c) then
          data_o(ctrl_uart_baud11_c downto ctrl_uart_baud00_c) <= ctrl(ctrl_uart_baud11_c downto ctrl_uart_baud00_c);
          data_o(ctrl_uart_sim_en_c)                           <= ctrl(ctrl_uart_sim_en_c);
          data_o(ctrl_uart_pmode1_c downto ctrl_uart_pmode0_c) <= ctrl(ctrl_uart_pmode1_c downto ctrl_uart_pmode0_c);
          data_o(ctrl_uart_prsc2_c  downto ctrl_uart_prsc0_c)  <= ctrl(ctrl_uart_prsc2_c  downto ctrl_uart_prsc0_c);
          data_o(ctrl_uart_rts_en_c)                           <= ctrl(ctrl_uart_rts_en_c);
          data_o(ctrl_uart_cts_en_c)                           <= ctrl(ctrl_uart_cts_en_c);
          data_o(ctrl_uart_en_c)                               <= ctrl(ctrl_uart_en_c);
          data_o(ctrl_uart_tx_busy_c)                          <= uart_tx.busy;
          data_o(ctrl_uart_cts_c)                              <= uart_cts_ff(1);
        else -- uart_id_rtx_addr_c
          data_o(data_rx_avail_c) <= or_reduce_f(uart_rx.avail);
          data_o(data_rx_overr_c) <= and_reduce_f(uart_rx.avail);
          data_o(data_rx_ferr_c)  <= uart_rx.ferr_rd;
          data_o(data_rx_perr_c)  <= uart_rx.perr_rd;
          data_o(7 downto 0)      <= uart_rx.data_rd;
        end if;
      end if;
    end if;
  end process rw_access;

  -- number of bits to be sampled --
  -- if parity flag is ENABLED:  11 bit -> "1011" (1 start bit + 8 data bits + 1 parity bit + 1 stop bit)
  -- if parity flag is DISABLED: 10 bit -> "1010" (1 start bit + 8 data bits + 1 stop bit)
  num_bits <= "1011" when (ctrl(ctrl_uart_pmode1_c) = '1') else "1010";


  -- Clock Selection ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- clock enable --
  clkgen_en_o <= ctrl(ctrl_uart_en_c);

  -- uart clock select --
  uart_clk <= clkgen_i(to_integer(unsigned(ctrl(ctrl_uart_prsc2_c downto ctrl_uart_prsc0_c))));


  -- UART Transmitter -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  uart_tx_unit: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- serial engine --
      uart_tx.done <= '0';
      if (uart_tx.busy = '0') or (ctrl(ctrl_uart_en_c) = '0') or (ctrl(ctrl_uart_sim_en_c) = '1') then -- idle or disabled or in SIM mode
        uart_tx.busy     <= '0';
        uart_tx.baud_cnt <= ctrl(ctrl_uart_baud11_c downto ctrl_uart_baud00_c);
        uart_tx.bitcnt   <= num_bits;
        uart_tx.sreg(0)  <= '1';
        if (wr_en = '1') and (ctrl(ctrl_uart_en_c) = '1') and (addr = uart_id_rtx_addr_c) and (ctrl(ctrl_uart_sim_en_c) = '0') then -- write trigger and not in SIM mode
          if (ctrl(ctrl_uart_pmode1_c) = '1') then -- add parity flag
            uart_tx.sreg <= '1' & (xor_reduce_f(data_i(7 downto 0)) xor ctrl(ctrl_uart_pmode0_c)) & data_i(7 downto 0) & '0'; -- stopbit & parity bit & data & startbit
          else
            uart_tx.sreg <= '1' & '1' & data_i(7 downto 0) & '0'; -- (dummy fill-bit &) stopbit & data & startbit
          end if;
          uart_tx.busy <= '1';
        end if;
      elsif (uart_clk = '1') and (uart_tx.tx_granted = '1') then
        if (uart_tx.baud_cnt = x"000") then
          uart_tx.baud_cnt <= ctrl(ctrl_uart_baud11_c downto ctrl_uart_baud00_c);
          uart_tx.bitcnt   <= std_ulogic_vector(unsigned(uart_tx.bitcnt) - 1);
          uart_tx.sreg     <= '1' & uart_tx.sreg(uart_tx.sreg'left downto 1);
        else
          uart_tx.baud_cnt <= std_ulogic_vector(unsigned(uart_tx.baud_cnt) - 1);
        end if;
        if (uart_tx.bitcnt = "0000") then
          uart_tx.busy <= '0'; -- done
          uart_tx.done <= '1';
        end if;
      end if;
      -- transmission granted --
      if (ctrl(ctrl_uart_en_c) = '0') then -- disabled
        uart_tx.tx_granted <= '0';
      elsif (uart_tx.done = '1') then
        uart_tx.tx_granted <= '0';
      elsif (uart_tx.cts = '1') then
        uart_tx.tx_granted <= '1';
      end if;
      -- transmitter output --
      uart_txd_o <= uart_tx.sreg(0) or (not uart_tx.tx_granted); -- keep TX line idle (=high) if waiting for permission to start sending (->CTS)
    end if;
  end process uart_tx_unit;


  -- UART Receiver --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  uart_rx_unit: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- input synchronizer --
      uart_rx.sync <= uart_rxd_i & uart_rx.sync(4 downto 1);

      -- serial engine --
      if (uart_rx.busy = '0') or (ctrl(ctrl_uart_en_c) = '0') or (ctrl(ctrl_uart_sim_en_c) = '1') then -- idle or disabled or in SIM mode
        uart_rx.busy     <= '0';
        uart_rx.baud_cnt <= '0' & ctrl(ctrl_uart_baud11_c downto ctrl_uart_baud01_c); -- half baud delay at the beginning to sample in the middle of each bit
        uart_rx.bitcnt   <= num_bits;
        if (ctrl(ctrl_uart_en_c) = '0') then -- to ensure defined state when reading
          uart_rx.perr <= (others => '0');
          uart_rx.ferr <= (others => '0');
          uart_rx.data <= (others => (others => '0'));
        elsif (uart_rx.sync(2 downto 0) = "001") then -- start bit? (falling edge)
          uart_rx.busy <= '1';
        end if;
      elsif (uart_clk = '1') then
        if (uart_rx.baud_cnt = x"000") then
          uart_rx.baud_cnt <= ctrl(ctrl_uart_baud11_c downto ctrl_uart_baud00_c);
          uart_rx.bitcnt   <= std_ulogic_vector(unsigned(uart_rx.bitcnt) - 1);
          uart_rx.sreg     <= uart_rx.sync(0) & uart_rx.sreg(uart_rx.sreg'left downto 1);
        else
          uart_rx.baud_cnt <= std_ulogic_vector(unsigned(uart_rx.baud_cnt) - 1);
        end if;
        if (uart_rx.bitcnt = "0000") then
          uart_rx.busy <= '0'; -- done
          -- data buffer (double buffering) --
          uart_rx.perr(0) <= ctrl(ctrl_uart_pmode1_c) and (xor_reduce_f(uart_rx.sreg(8 downto 0)) xor ctrl(ctrl_uart_pmode0_c));
          uart_rx.ferr(0) <= not uart_rx.sreg(9); -- check stop bit (error if not set)
          if (ctrl(ctrl_uart_pmode1_c) = '1') then -- add parity flag
            uart_rx.data(0) <= uart_rx.sreg(7 downto 0);
          else
            uart_rx.data(0) <= uart_rx.sreg(8 downto 1);
          end if;
          uart_rx.perr(1) <= uart_rx.perr(0);
          uart_rx.ferr(1) <= uart_rx.ferr(0);
          uart_rx.data(1) <= uart_rx.data(0);
        end if;
      end if;

      -- RX available flag --
      uart_rx.busy_ff <= uart_rx.busy;
      if (ctrl(ctrl_uart_en_c) = '0') then -- disabled
        uart_rx.avail <= "000";
      elsif ((uart_rx.avail(0) = '1') or (uart_rx.avail(1) = '1')) and (rd_en = '1') and (addr = uart_id_rtx_addr_c) then -- RX read access
        uart_rx.avail <= '0' & '0' & uart_rx.avail(1);
      elsif (uart_rx.busy_ff = '1') and (uart_rx.busy = '0') then -- RX done
        uart_rx.avail <= uart_rx.avail(1 downto 0) & '1';
      end if;
    end if;
  end process uart_rx_unit;

  -- Receiver double-buffering - buffer read --
  uart_rx.perr_rd <= uart_rx.perr(1) when (uart_rx.avail(1) = '1') else uart_rx.perr(0);
  uart_rx.ferr_rd <= uart_rx.ferr(1) when (uart_rx.avail(1) = '1') else uart_rx.ferr(0);
  uart_rx.data_rd <= uart_rx.data(1) when (uart_rx.avail(1) = '1') else uart_rx.data(0);

  -- RX engine ready for a new char? --
  uart_rx.rtr <= '1' when (uart_rx.avail(2 downto 0) = "000") and (uart_rx.busy = '0') and (uart_rx.busy_ff = '0') and (ctrl(ctrl_uart_en_c) = '1') else '0';


  -- Hardware Flow Control ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  uart_tx.cts <= (not uart_cts_ff(1)) when (ctrl(ctrl_uart_cts_en_c) = '1') else '1'; -- input is low-active, internal signal is high-active
  uart_rts    <= (not uart_rx.rtr)    when (ctrl(ctrl_uart_rts_en_c) = '1') else '0'; -- output is low-active

  -- flow-control input/output synchronizer --
  flow_control_buffer: process(clk_i)
  begin
    if rising_edge(clk_i) then -- should be mapped to IOBs
      uart_cts_ff <= uart_cts_ff(0) & uart_cts_i;
      uart_rts_o  <= uart_rts;
    end if;
  end process flow_control_buffer;


  -- Interrupts -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- UART RX data available
  irq_rxd_o <= uart_rx.busy_ff and (not uart_rx.busy);
  -- UART TX complete
  irq_txd_o <= uart_tx.done;


  -- SIMULATION Output ----------------------------------------------------------------------
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
      if (ctrl(ctrl_uart_en_c) = '1') and (ctrl(ctrl_uart_sim_en_c) = '1') then -- UART enabled and simulation output selected?
        if (wr_en = '1') and (addr = uart_id_rtx_addr_c) then -- write access to tx register
        
          -- print lowest byte to ASCII char --
          char_v := to_integer(unsigned(data_i(7 downto 0)));
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

          -- dump raw data as 8 hex char text to file --
          if (sim_data_output_en_c = true) then
            for x in 7 downto 0 loop
              write(line_data_v, to_hexchar_f(data_i(3+x*4 downto 0+x*4))); -- write in hex form
            end loop; -- x
            writeline(file_uart_data_out, line_data_v);
          end if;

        end if;
      end if;
    end if;
  end process sim_output;
-- RTL_SYNTHESIS ON
-- synthesis translate_on
-- pragma translate_on

end neorv32_uart_rtl;
