-- #################################################################################################
-- # << NEORV32 - Stream Link Interface (SLINK) >>                                                 #
-- # ********************************************************************************************* #
-- # Up to 8 input (RX) and up to 8 output (TX) stream links are supported. Each link provides an  #
-- # internal FIFO for buffering. Each stream direction provides a global interrupt to indicate    #
-- # that a RX link has received new data or that a TX link has finished sending data              #
-- # (if FIFO_DEPTH = 1) OR if RX/TX link FIFO has become half full (if FIFO_DEPTH > 1).           #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
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
    SLINK_NUM_TX  : natural; -- number of TX links (0..8)
    SLINK_NUM_RX  : natural; -- number of TX links (0..8)
    SLINK_TX_FIFO : natural; -- TX fifo depth, has to be a power of two
    SLINK_RX_FIFO : natural  -- RX fifo depth, has to be a power of two
  );
  port (
    -- host access --
    clk_i          : in  std_ulogic; -- global clock line
    rstn_i         : in  std_ulogic; -- global reset line, low-active
    addr_i         : in  std_ulogic_vector(31 downto 0); -- address
    rden_i         : in  std_ulogic; -- read enable
    wren_i         : in  std_ulogic; -- write enable
    data_i         : in  std_ulogic_vector(31 downto 0); -- data in
    data_o         : out std_ulogic_vector(31 downto 0); -- data out
    ack_o          : out std_ulogic; -- transfer acknowledge
    -- interrupt --
    irq_tx_o       : out std_ulogic;
    irq_rx_o       : out std_ulogic;
    -- TX stream interfaces --
    slink_tx_dat_o : out sdata_8x32_t; -- output data
    slink_tx_val_o : out std_ulogic_vector(7 downto 0); -- valid output
    slink_tx_rdy_i : in  std_ulogic_vector(7 downto 0); -- ready to send
    slink_tx_lst_o : out std_ulogic_vector(7 downto 0); -- last data of packet
    -- RX stream interfaces --
    slink_rx_dat_i : in  sdata_8x32_t; -- input data
    slink_rx_val_i : in  std_ulogic_vector(7 downto 0); -- valid input
    slink_rx_rdy_o : out std_ulogic_vector(7 downto 0); -- ready to receive
    slink_rx_lst_i : in  std_ulogic_vector(7 downto 0)  -- last data of packet
  );
end neorv32_slink;

architecture neorv32_slink_rtl of neorv32_slink is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(slink_size_c); -- low address boundary bit

  -- control register bits --
  constant ctrl_en_c            : natural :=  0; -- r/w: global enable/reset
  --
  constant ctrl_rx_size_lsb_c   : natural :=  8; -- r/-: log2(RX FIFO size)
  constant ctrl_rx_size_msb_c   : natural := 11;
  constant ctrl_tx_size_lsb_c   : natural := 12; -- r/-: log2(TX FIFO size)
  constant ctrl_tx_size_msb_c   : natural := 15;
  constant ctrl_rx_irq_en_lsb_c : natural := 16; -- r/w: enable RX interrupt for link 0..7
  constant ctrl_rx_irq_en_msb_c : natural := 23;
  constant ctrl_tx_irq_en_lsb_c : natural := 24; -- r/w: enable TX interrupt for link 0..7
  constant ctrl_tx_irq_en_msb_c : natural := 31;

  -- status register bits --
  constant status_rx_avail_lsb_c : natural :=  0; -- r/-: set if RX link 0..7 FIFO is NOT empty
  constant status_rx_avail_msb_c : natural :=  7;
  constant status_tx_free_lsb_c  : natural :=  8; -- r/-: set if TX link 0..7 FIFO is NOT full
  constant status_tx_free_msb_c  : natural := 15;
  constant status_rx_last_lsb_c  : natural := 16; -- r/-: indicates RX packet end of link 0..7
  constant status_rx_last_msb_c  : natural := 23;
  constant status_tx_last_lsb_c  : natural := 24; -- r/w: set TX packet end of lin 0..7
  constant status_tx_last_msb_c  : natural := 31;

  -- bus access control --
  signal acc_en : std_ulogic;
  signal addr   : std_ulogic_vector(31 downto 0);
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- control/status register --
  signal enable      : std_ulogic; -- global enable
  signal irq_rx_en   : std_ulogic_vector(SLINK_NUM_RX-1 downto 0);
  signal irq_tx_en   : std_ulogic_vector(SLINK_NUM_TX-1 downto 0);
  signal tx_fifo_lst : std_ulogic_vector(SLINK_NUM_TX-1 downto 0);

  -- stream link fifo interface --
  type fifo_data_t is array (0 to 7) of std_ulogic_vector(31 downto 0);
  signal rx_fifo_rdata : fifo_data_t;
  signal fifo_clear    : std_ulogic;
  signal link_sel      : std_ulogic_vector(7 downto 0);
  signal tx_fifo_we    : std_ulogic_vector(7 downto 0);
  signal slink_tx_lst  : std_ulogic_vector(7 downto 0);
  signal slink_tx_val  : std_ulogic_vector(7 downto 0);
  signal rx_fifo_re    : std_ulogic_vector(7 downto 0);
  signal rx_fifo_avail : std_ulogic_vector(7 downto 0);
  signal tx_fifo_free  : std_ulogic_vector(7 downto 0);
  signal rx_fifo_lst   : std_ulogic_vector(7 downto 0);

  -- interrupt generator --
  type detect_t is array (0 to 7) of std_ulogic_vector(1 downto 0);
  type irq_t is record
    detect : detect_t; -- rising-edge detector
    fire   : std_ulogic_vector(7 downto 0);
  end record;
  signal rx_irq, tx_irq : irq_t;

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


  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = slink_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= slink_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
  wren   <= acc_en and wren_i;
  rden   <= acc_en and rden_i;


  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rw_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      enable      <= '0';
      irq_rx_en   <= (others => '0');
      irq_tx_en   <= (others => '0');
      tx_fifo_lst <= (others => '0');
      ack_o       <= '-';
      data_o      <= (others => '-');
    elsif rising_edge(clk_i) then

      -- bus access acknowledge --
      ack_o <= rden or wren;

      -- write access --
      if (wren = '1') then
        if (addr(5) = '0') then -- control/status register
          if (addr(4) = '0') then -- control register
            enable <= data_i(ctrl_en_c);
            for i in 0 to SLINK_NUM_RX-1 loop
              irq_rx_en(i) <= data_i(ctrl_rx_irq_en_lsb_c + i);
            end loop;
            for i in 0 to SLINK_NUM_TX-1 loop
              irq_tx_en(i) <= data_i(ctrl_tx_irq_en_lsb_c + i);
            end loop;
          else -- status register
            for i in 0 to SLINK_NUM_TX-1 loop
              tx_fifo_lst(i) <= data_i(status_tx_last_lsb_c + i);
            end loop;
          end if;
        end if;
      end if;

      -- read access --
      data_o <= (others => '0');
      if (rden = '1') then
        if (addr(5) = '0') then -- control/status register
          if (addr(4) = '0') then -- control register
            data_o(ctrl_en_c) <= enable;
            data_o(ctrl_rx_size_msb_c downto ctrl_rx_size_lsb_c) <= std_ulogic_vector(to_unsigned(index_size_f(SLINK_RX_FIFO), 4));
            data_o(ctrl_tx_size_msb_c downto ctrl_tx_size_lsb_c) <= std_ulogic_vector(to_unsigned(index_size_f(SLINK_TX_FIFO), 4));
            for i in 0 to SLINK_NUM_RX-1 loop
              data_o(ctrl_rx_irq_en_lsb_c + i) <= irq_rx_en(i);
            end loop;
            for i in 0 to SLINK_NUM_TX-1 loop
              data_o(ctrl_tx_irq_en_lsb_c + i) <= irq_tx_en(i);
            end loop;
          else -- status register
            for i in 0 to SLINK_NUM_RX-1 loop
              data_o(status_rx_avail_lsb_c + i) <= rx_fifo_avail(i);
              data_o(status_rx_last_lsb_c + i)  <= rx_fifo_lst(i) and rx_fifo_avail(i);
            end loop;
            for i in 0 to SLINK_NUM_TX-1 loop
              data_o(status_tx_free_lsb_c + i) <= tx_fifo_free(i);
              data_o(status_tx_last_lsb_c + i) <= tx_fifo_lst(i);
            end loop;
          end if;
        else -- RX links
          case addr(4 downto 2) is
            when "000"  => data_o <= rx_fifo_rdata(0);
            when "001"  => data_o <= rx_fifo_rdata(1);
            when "010"  => data_o <= rx_fifo_rdata(2);
            when "011"  => data_o <= rx_fifo_rdata(3);
            when "100"  => data_o <= rx_fifo_rdata(4);
            when "101"  => data_o <= rx_fifo_rdata(5);
            when "110"  => data_o <= rx_fifo_rdata(6);
            when "111"  => data_o <= rx_fifo_rdata(7);
            when others => data_o <= (others => '0');
          end case;
        end if;
      end if;

    end if;
  end process rw_access;

  -- link fifo reset (sync) --
  fifo_clear <= not enable;


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
    tx_fifo_we(i) <= link_sel(i) and wren;
    rx_fifo_re(i) <= link_sel(i) and rden;
  end generate;


  -- TX Link FIFOs --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  transmit_fifo_gen:
  for i in 0 to SLINK_NUM_TX-1 generate
    transmit_fifo_inst: neorv32_fifo
    generic map (
      FIFO_DEPTH => SLINK_TX_FIFO, -- number of fifo entries; has to be a power of two; min 1
      FIFO_WIDTH => 32+1,          -- size of data elements in fifo
      FIFO_RSYNC => false,         -- async read
      FIFO_SAFE  => true           -- safe access
    )
    port map (
      -- control --
      clk_i                => clk_i,             -- clock, rising edge
      rstn_i               => '1',               -- async reset, low-active
      clear_i              => fifo_clear,        -- sync reset, high-active
      level_o              => open,              -- fill level
      half_o               => open,              -- FIFO is at least half full
      -- write port --
      wdata_i(31 downto 0) => data_i,            -- write data
      wdata_i(32)          => tx_fifo_lst(i),    -- end of packet
      we_i                 => tx_fifo_we(i),     -- write enable
      free_o               => tx_fifo_free(i),   -- at least one entry is free when set
      -- read port --
      re_i                 => slink_tx_rdy_i(i), -- read enable
      rdata_o(31 downto 0) => slink_tx_dat_o(i), -- read data
      rdata_o(32)          => slink_tx_lst(i),   -- end of packet
      avail_o              => slink_tx_val(i)    -- data available when set
    );
    slink_tx_lst_o(i) <= slink_tx_lst(i) and slink_tx_val(i);
    slink_tx_val_o(i) <= slink_tx_val(i);
  end generate;

  -- terminate unimplemented links --
  transmit_fifo_gen_terminate:
  for i in SLINK_NUM_TX to 7 generate
    tx_fifo_free(i)   <= '0';
    slink_tx_dat_o(i) <= (others => '0');
    slink_tx_val_o(i) <= '0';
    slink_tx_lst_o(i) <= '0';
  end generate;


  -- RX Link FIFOs --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  receive_fifo_gen:
  for i in 0 to SLINK_NUM_RX-1 generate
    receive_fifo_inst: neorv32_fifo
    generic map (
      FIFO_DEPTH => SLINK_RX_FIFO, -- number of fifo entries; has to be a power of two; min 1
      FIFO_WIDTH => 32+1,          -- size of data elements in fifo
      FIFO_RSYNC => false,         -- async read
      FIFO_SAFE  => true           -- safe access
    )
    port map (
      -- control --
      clk_i                => clk_i,             -- clock, rising edge
      rstn_i               => '1',               -- async reset, low-active
      clear_i              => fifo_clear,        -- sync reset, high-active
      level_o              => open,              -- fill level
      half_o               => open,              -- FIFO is at least half full
      -- write port --
      wdata_i(31 downto 0) => slink_rx_dat_i(i), -- write data
      wdata_i(32)          => slink_rx_lst_i(i), -- end of packet
      we_i                 => slink_rx_val_i(i), -- write enable
      free_o               => slink_rx_rdy_o(i), -- at least one entry is free when set
      -- read port --
      re_i                 => rx_fifo_re(i),     -- read enable
      rdata_o(31 downto 0) => rx_fifo_rdata(i),  -- read data
      rdata_o(32)          => rx_fifo_lst(i),    -- end of packet
      avail_o              => rx_fifo_avail(i)   -- data available when set
    );
  end generate;

  -- terminate unimplemented links --
  receive_fifo_gen_terminate:
  for i in SLINK_NUM_RX to 7 generate
    rx_fifo_avail(i)  <= '0';
    slink_rx_rdy_o(i) <= '0';
    rx_fifo_lst(i)    <= '0';
    rx_fifo_rdata(i)  <= (others => '0');
  end generate;


  -- Interrupt Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- edge detector - sync --
  irq_edge_detect_sync: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- RX --
      rx_irq.detect <= (others => (others => '0'));
      for i in 0 to SLINK_NUM_RX-1 loop
        if (enable = '1') and  (irq_rx_en(i) = '1') then
          rx_irq.detect(i) <= rx_irq.detect(i)(0) & rx_fifo_avail(i);
        else
          rx_irq.detect(i) <= "00";
        end if;
      end loop;
      -- TX --
      tx_irq.detect <= (others => (others => '0'));
      for i in 0 to SLINK_NUM_TX-1 loop
        if (enable = '1') and (irq_tx_en(i) = '1') then
          tx_irq.detect(i) <= tx_irq.detect(i)(0) & tx_fifo_free(i);
        else
          tx_irq.detect(i) <= "00";
        end if;
      end loop;

      -- interrupt trigger --
      if (enable = '0') then
        irq_rx_o <= '0';
        irq_tx_o <= '0';
      else
        irq_rx_o <= or_reduce_f(rx_irq.fire);
        irq_tx_o <= or_reduce_f(tx_irq.fire);
      end if;
    end if;
  end process irq_edge_detect_sync;

  -- edge detector - comb --
  irq_edge_detect_comb: process(rx_irq.detect, tx_irq.detect)
  begin
    -- RX --
    rx_irq.fire <= (others => '0');
    for i in 0 to SLINK_NUM_RX-1 loop
      if (rx_irq.detect(i) = "01") then
        rx_irq.fire(i) <= '1';
      end if;
    end loop;
    -- TX --
    tx_irq.fire <= (others => '0');
    for i in 0 to SLINK_NUM_TX-1 loop
      if (tx_irq.detect(i) = "01") then
        tx_irq.fire(i) <= '1';
      end if;
    end loop;
  end process irq_edge_detect_comb;


end neorv32_slink_rtl;
