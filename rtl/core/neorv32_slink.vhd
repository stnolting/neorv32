-- #################################################################################################
-- # << NEORV32 - Stream Link Interface (SLINK) >>                                                 #
-- # ********************************************************************************************* #
-- # Up to 8 input (RX) and up to 8 output (TX) stream links are supported. Each stream direction  #
-- # provides a global interrupt to indicate that a RX link has received new data or that a TX     #
-- # has finished sending data. Each link is provides an internal FIFO for buffering.              #
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

entity neorv32_slink is
  generic (
    SLINK_NUM_TX  : natural := 8; -- number of TX links (0..8)
    SLINK_NUM_RX  : natural := 8; -- number of TX links (0..8)
    SLINK_TX_FIFO : natural := 1; -- TX fifo depth, has to be a power of two
    SLINK_RX_FIFO : natural := 1  -- RX fifo depth, has to be a power of two
  );
  port (
    -- host access --
    clk_i          : in  std_ulogic; -- global clock line
    addr_i         : in  std_ulogic_vector(31 downto 0); -- address
    rden_i         : in  std_ulogic; -- read enable
    wren_i         : in  std_ulogic; -- write enable
    data_i         : in  std_ulogic_vector(31 downto 0); -- data in
    data_o         : out std_ulogic_vector(31 downto 0); -- data out
    ack_o          : out std_ulogic; -- transfer acknowledge
    -- interrupt --
    irq_tx_o       : out std_ulogic; -- transmission done
    irq_rx_o       : out std_ulogic; -- data received
    -- TX stream interfaces --
    slink_tx_dat_o : out sdata_8x32_t; -- output data
    slink_tx_val_o : out std_ulogic_vector(7 downto 0); -- valid output
    slink_tx_rdy_i : in  std_ulogic_vector(7 downto 0); -- ready to send
    -- RX stream interfaces --
    slink_rx_dat_i : in  sdata_8x32_t; -- input data
    slink_rx_val_i : in  std_ulogic_vector(7 downto 0); -- valid input
    slink_rx_rdy_o : out std_ulogic_vector(7 downto 0)  -- ready to receive
  );
end neorv32_slink;

architecture neorv32_slink_rtl of neorv32_slink is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(slink_size_c); -- low address boundary bit

  -- control reg bits --
  constant ctrl_rx0_avail_c : natural :=  0; -- r/-: set if TX link 0 is ready to send
  constant ctrl_rx1_avail_c : natural :=  1; -- r/-: set if TX link 1 is ready to send
  constant ctrl_rx2_avail_c : natural :=  2; -- r/-: set if TX link 2 is ready to send
  constant ctrl_rx3_avail_c : natural :=  3; -- r/-: set if TX link 3 is ready to send
  constant ctrl_rx4_avail_c : natural :=  4; -- r/-: set if TX link 4 is ready to send
  constant ctrl_rx5_avail_c : natural :=  5; -- r/-: set if TX link 5 is ready to send
  constant ctrl_rx6_avail_c : natural :=  6; -- r/-: set if TX link 6 is ready to send
  constant ctrl_rx7_avail_c : natural :=  7; -- r/-: set if TX link 7 is ready to send
  --
  constant ctrl_tx0_free_c  : natural :=  8; -- r/-: set if RX link 0 data available
  constant ctrl_tx1_free_c  : natural :=  9; -- r/-: set if RX link 1 data available
  constant ctrl_tx2_free_c  : natural := 10; -- r/-: set if RX link 2 data available
  constant ctrl_tx3_free_c  : natural := 11; -- r/-: set if RX link 3 data available
  constant ctrl_tx4_free_c  : natural := 12; -- r/-: set if RX link 4 data available
  constant ctrl_tx5_free_c  : natural := 13; -- r/-: set if RX link 5 data available
  constant ctrl_tx6_free_c  : natural := 14; -- r/-: set if RX link 6 data available
  constant ctrl_tx7_free_c  : natural := 15; -- r/-: set if RX link 7 data available
  --
  constant ctrl_rx_num0_c   : natural := 16; -- r/-: number of implemented RX links -1 bit 0
  constant ctrl_rx_num1_c   : natural := 17; -- r/-: number of implemented RX links -1 bit 1
  constant ctrl_rx_num2_c   : natural := 18; -- r/-: number of implemented RX links -1 bit 2
  constant ctrl_tx_num0_c   : natural := 19; -- r/-: number of implemented TX links -1 bit 0
  constant ctrl_tx_num1_c   : natural := 20; -- r/-: number of implemented TX links -1 bit 1
  constant ctrl_tx_num2_c   : natural := 21; -- r/-: number of implemented TX links -1 bit 2
  --
  constant ctrl_rx_size0_c  : natural := 22; -- r/-: log2(RX FIFO size) bit 0
  constant ctrl_rx_size1_c  : natural := 23; -- r/-: log2(RX FIFO size) bit 1
  constant ctrl_rx_size2_c  : natural := 24; -- r/-: log2(RX FIFO size) bit 2
  constant ctrl_rx_size3_c  : natural := 25; -- r/-: log2(RX FIFO size) bit 3
  constant ctrl_tx_size0_c  : natural := 26; -- r/-: log2(TX FIFO size) bit 0
  constant ctrl_tx_size1_c  : natural := 27; -- r/-: log2(TX FIFO size) bit 1
  constant ctrl_tx_size2_c  : natural := 28; -- r/-: log2(TX FIFO size) bit 2
  constant ctrl_tx_size3_c  : natural := 29; -- r/-: log2(TX FIFO size) bit 3
  --
  constant ctrl_en_c        : natural := 31; -- r/w: global enable

  -- bus access control --
  signal ack_read  : std_ulogic;
  signal ack_write : std_ulogic;
  signal acc_en    : std_ulogic;
  signal addr      : std_ulogic_vector(31 downto 0);

  -- control register --
  signal enable : std_ulogic; -- global enable

  -- interrupt generator --
  signal tx_fifo_free_buf  : std_ulogic_vector(7 downto 0);
  signal rx_fifo_avail_buf : std_ulogic_vector(7 downto 0);

  -- stream link fifo interface --
  type fifo_data_t is array (0 to 7) of std_ulogic_vector(31 downto 0);
  signal rx_fifo_rdata             : fifo_data_t;
  signal fifo_clear                : std_ulogic;
  signal link_sel                  : std_ulogic_vector(7 downto 0);
  signal tx_fifo_we, tx_fifo_free  : std_ulogic_vector(7 downto 0);
  signal rx_fifo_re, rx_fifo_avail : std_ulogic_vector(7 downto 0);

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (is_power_of_two_f(SLINK_TX_FIFO) = false) report "NEORV32 PROCESSOR CONFIG ERROR: SLINK <SLINK_TX_FIFO> has to be a power of two." severity error;
  assert not (SLINK_TX_FIFO > 2**15) report "NEORV32 PROCESSOR CONFIG ERROR: SLINK <SLINK_TX_FIFO> has to be 1..32768." severity error;
  --
  assert not (is_power_of_two_f(SLINK_RX_FIFO) = false) report "NEORV32 PROCESSOR CONFIG ERROR: SLINK <SLINK_RX_FIFO> has to be a power of two." severity error;
  assert not (SLINK_RX_FIFO > 2**15) report "NEORV32 PROCESSOR CONFIG ERROR: SLINK <SLINK_RX_FIFO> has to be 1..32768." severity error;
  --
  assert not (SLINK_NUM_RX > 8) report "NEORV32 PROCESSOR CONFIG ERROR: SLINK <SLINK_NUM_RX> has to be 0..8." severity error;
  assert not (SLINK_NUM_TX > 8) report "NEORV32 PROCESSOR CONFIG ERROR: SLINK <SLINK_NUM_TX> has to be 0..8." severity error;
  --
  assert false report "NEORV32 PROCESSOR CONFIG NOTE: Implementing " & integer'image(SLINK_NUM_RX) & " RX and " &
  integer'image(SLINK_NUM_TX) & " TX stream links." severity note;


  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = slink_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= slink_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned


  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rw_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- write access --
      ack_write <= '0';
      if (acc_en = '1') and (wren_i = '1') then
        if (addr(5) = '0') then -- control register
          enable    <= data_i(ctrl_en_c);
          ack_write <= '1';
        else -- TX links
          ack_write <= or_reduce_f(link_sel and tx_fifo_free);
        end if;
      end if;

      -- read access --
      data_o   <= (others => '0');
      ack_read <= '0';
      if (acc_en = '1') and (rden_i = '1') then
        if (addr(5) = '0') then -- control register
          data_o(ctrl_rx7_avail_c downto ctrl_rx0_avail_c) <= rx_fifo_avail;
          data_o(ctrl_tx7_free_c  downto ctrl_tx0_free_c)  <= tx_fifo_free;
          data_o(ctrl_rx_num2_c   downto ctrl_rx_num0_c)   <= std_ulogic_vector(to_unsigned(SLINK_NUM_RX-1, 3));
          data_o(ctrl_tx_num2_c   downto ctrl_tx_num0_c)   <= std_ulogic_vector(to_unsigned(SLINK_NUM_TX-1, 3));
          data_o(ctrl_rx_size3_c  downto ctrl_rx_size0_c)  <= std_ulogic_vector(to_unsigned(index_size_f(SLINK_RX_FIFO), 4));
          data_o(ctrl_tx_size3_c  downto ctrl_tx_size0_c)  <= std_ulogic_vector(to_unsigned(index_size_f(SLINK_TX_FIFO), 4));
          data_o(ctrl_en_c)                                <= enable;
          ack_read <= '1';
        else -- RX links
          data_o   <= rx_fifo_rdata(to_integer(unsigned(addr(4 downto 2))));
          ack_read <= or_reduce_f(link_sel and rx_fifo_avail);
        end if;
      end if;
    end if;
  end process rw_access;

  -- bus access acknowledge --
  ack_o <= ack_write or ack_read;

  -- link fifo reset (sync) --
  fifo_clear <= not enable;


  -- Interrupt Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_generator: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- buffer status --
      tx_fifo_free_buf  <= tx_fifo_free;
      rx_fifo_avail_buf <= rx_fifo_avail;
      -- rising edge detector --
      irq_tx_o <= enable and or_reduce_f(tx_fifo_free  and (not tx_fifo_free_buf));
      irq_rx_o <= enable and or_reduce_f(rx_fifo_avail and (not rx_fifo_avail_buf));
    end if;
  end process irq_generator;


  -- Link Select ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  link_select: process(addr)
  begin
    case addr(5 downto 2) is -- MSB = data fifo access at all?
      when "1000" => link_sel <= "00000001";
      when "1001" => link_sel <= "00000010";
      when "1010" => link_sel <= "00000100";
      when "1011" => link_sel <= "00001000";
      when "1100" => link_sel <= "00010000";
      when "1101" => link_sel <= "00100000";
      when "1110" => link_sel <= "01000000";
      when "1111" => link_sel <= "10000000";
      when others => link_sel <= "00000000";
    end case;
  end process link_select;

  fifo_access_gen:
  for i in 0 to 7 generate
    tx_fifo_we(i) <= link_sel(i) and acc_en and wren_i;
    rx_fifo_re(i) <= link_sel(i) and acc_en and rden_i;
  end generate;


  -- TX Link FIFOs --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  transmit_fifo_gen:
  for i in 0 to SLINK_NUM_TX-1 generate
    transmit_fifo_inst: neorv32_fifo
    generic map (
      FIFO_DEPTH => SLINK_TX_FIFO, -- number of fifo entries; has to be a power of two; min 1
      FIFO_WIDTH => 32,            -- size of data elements in fifo
      FIFO_RSYNC => false,         -- false = async read; true = sync read
      FIFO_SAFE  => true           -- true = allow read/write only if data available
    )
    port map (
      -- control --
      clk_i   => clk_i,             -- clock, rising edge
      rstn_i  => '1',               -- async reset, low-active
      clear_i => fifo_clear,        -- sync reset, high-active
      -- write port --
      wdata_i => data_i,            -- write data
      we_i    => tx_fifo_we(i),     -- write enable
      free_o  => tx_fifo_free(i),   -- at least one entry is free when set
      -- read port --
      re_i    => slink_tx_rdy_i(i), -- read enable
      rdata_o => slink_tx_dat_o(i), -- read data
      avail_o => slink_tx_val_o(i)  -- data available when set
    );
  end generate;

  -- terminate unimplemented links --
  transmit_fifo_gen_terminate:
  for i in SLINK_NUM_TX to 7 generate
    tx_fifo_free(i)   <= '0';
    slink_tx_dat_o(i) <= (others => '0');
    slink_tx_val_o(i) <= '0';
  end generate;


  -- RX Link FIFOs --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  receive_fifo_gen:
  for i in 0 to SLINK_NUM_RX-1 generate
    receive_fifo_inst: neorv32_fifo
    generic map (
      FIFO_DEPTH => SLINK_RX_FIFO, -- number of fifo entries; has to be a power of two; min 1
      FIFO_WIDTH => 32,            -- size of data elements in fifo
      FIFO_RSYNC => false,         -- false = async read; true = sync read
      FIFO_SAFE  => true           -- true = allow read/write only if data available
    )
    port map (
      -- control --
      clk_i   => clk_i,             -- clock, rising edge
      rstn_i  => '1',               -- async reset, low-active
      clear_i => fifo_clear,        -- sync reset, high-active
      -- write port --
      wdata_i => slink_rx_dat_i(i), -- write data
      we_i    => slink_rx_val_i(i), -- write enable
      free_o  => slink_rx_rdy_o(i), -- at least one entry is free when set
      -- read port --
      re_i    => rx_fifo_re(i),     -- read enable
      rdata_o => rx_fifo_rdata(i),  -- read data
      avail_o => rx_fifo_avail(i)   -- data available when set
    );
  end generate;

  -- terminate unimplemented links --
  receive_fifo_gen_terminate:
  for i in SLINK_NUM_RX to 7 generate
    rx_fifo_avail(i)  <= '0';
    slink_rx_rdy_o(i) <= '0';
    rx_fifo_rdata(i)  <= (others => '0');
  end generate;


end neorv32_slink_rtl;
