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
    SLINK_NUM_TX  : natural; -- number of TX links (0..8)
    SLINK_NUM_RX  : natural; -- number of TX links (0..8)
    SLINK_TX_FIFO : natural; -- TX fifo depth, has to be a power of two
    SLINK_RX_FIFO : natural  -- RX fifo depth, has to be a power of two
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

  -- control register bits --
  constant ctrl_rx_num_lsb_c  : natural :=  0; -- r/-: number of implemented RX links
  constant ctrl_rx_num_msb_c  : natural :=  3;
  --
  constant ctrl_tx_num_lsb_c  : natural :=  4; -- r/-: number of implemented TX links
  constant ctrl_tx_num_msb_c  : natural :=  7;
  --
  constant ctrl_rx_size_lsb_c : natural :=  8; -- r/-: log2(RX FIFO size)
  constant ctrl_rx_size_msb_c : natural := 11;
  --
  constant ctrl_tx_size_lsb_c : natural := 12; -- r/-: log2(TX FIFO size)
  constant ctrl_tx_size_msb_c : natural := 15;
  --
  constant ctrl_en_c          : natural := 31; -- r/w: global enable

  -- status register bits --
  constant status_rx_avail_lsb_c : natural :=  0; -- r/-: set if TX link 0..7 is ready to send
  constant status_rx_avail_msb_c : natural :=  7;
  --
  constant status_tx_free_lsb_c  : natural :=  8; -- r/-: set if RX link 0..7 data available
  constant status_tx_free_msb_c  : natural := 15;
  --
  constant status_rx_half_lsb_c  : natural := 16; -- r/-: set if TX link 0..7 FIFO fill-level is >= half-full
  constant status_rx_half_msb_c  : natural := 23;
  --
  constant status_tx_half_lsb_c  : natural := 24; -- r/-: set if RX link 0..7 FIFO fill-level is > half-full
  constant status_tx_half_msb_c  : natural := 31;

  -- bus access control --
  signal ack_read  : std_ulogic;
  signal ack_write : std_ulogic;
  signal acc_en    : std_ulogic;
  signal addr      : std_ulogic_vector(31 downto 0);

  -- control register --
  signal enable : std_ulogic; -- global enable

  -- stream link fifo interface --
  type fifo_data_t is array (0 to 7) of std_ulogic_vector(31 downto 0);
  type fifo_rx_level_t is array (0 to 7) of std_ulogic_vector(index_size_f(SLINK_RX_FIFO) downto 0);
  type fifo_tx_level_t is array (0 to 7) of std_ulogic_vector(index_size_f(SLINK_TX_FIFO) downto 0);
  signal rx_fifo_rdata                   : fifo_data_t;
  signal rx_fifo_level                   : fifo_rx_level_t;
  signal tx_fifo_level                   : fifo_tx_level_t;
  signal fifo_clear                      : std_ulogic;
  signal link_sel                        : std_ulogic_vector(7 downto 0);
  signal tx_fifo_we,    rx_fifo_re       : std_ulogic_vector(7 downto 0);
  signal rx_fifo_avail, rx_fifo_avail_ff : std_ulogic_vector(7 downto 0);
  signal tx_fifo_free,  tx_fifo_free_ff  : std_ulogic_vector(7 downto 0);
  signal rx_fifo_half,  rx_fifo_half_ff  : std_ulogic_vector(7 downto 0);
  signal tx_fifo_half,  tx_fifo_half_ff  : std_ulogic_vector(7 downto 0);

  -- interrupt controller --
  type irq_t is record
    rx_pending    : std_ulogic;
    rx_pending_ff : std_ulogic;
    rx_fire       : std_ulogic;
    tx_pending    : std_ulogic;
    tx_pending_ff : std_ulogic;
    tx_fire       : std_ulogic;
    wr_ack        : std_ulogic;
    rd_ack        : std_ulogic;
  end record;
  signal irq : irq_t;

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
      irq.wr_ack <= '0';
      ack_write  <= '0';
      if (acc_en = '1') and (wren_i = '1') then
        if (addr(5) = '0') then -- control/status 
          if (addr(4) = '0') then -- control register
            enable <= data_i(ctrl_en_c);
          else -- status register
            irq.wr_ack <= '1';
          end if;
          ack_write <= '1';
        else -- TX links
          ack_write <= or_reduce_f(link_sel and tx_fifo_free);
        end if;
      end if;

      -- read access --
      irq.rd_ack <= '0';
      data_o     <= (others => '0');
      ack_read   <= '0';
      if (acc_en = '1') and (rden_i = '1') then
        if (addr(5) = '0') then -- control/status registers
          ack_read <= '1';
          if (addr(4) = '0') then -- control register
            data_o(ctrl_rx_num_msb_c  downto ctrl_rx_num_lsb_c)  <= std_ulogic_vector(to_unsigned(SLINK_NUM_RX, 4));
            data_o(ctrl_tx_num_msb_c  downto ctrl_tx_num_lsb_c)  <= std_ulogic_vector(to_unsigned(SLINK_NUM_TX, 4));
            data_o(ctrl_rx_size_msb_c downto ctrl_rx_size_lsb_c) <= std_ulogic_vector(to_unsigned(index_size_f(SLINK_RX_FIFO), 4));
            data_o(ctrl_tx_size_msb_c downto ctrl_tx_size_lsb_c) <= std_ulogic_vector(to_unsigned(index_size_f(SLINK_TX_FIFO), 4));
            data_o(ctrl_en_c)                                    <= enable;
          else -- fifo status register
            data_o(status_rx_avail_msb_c downto status_rx_avail_lsb_c) <= rx_fifo_avail;
            data_o(status_tx_free_msb_c  downto status_tx_free_lsb_c)  <= tx_fifo_free;
            data_o(status_rx_half_msb_c  downto status_rx_half_lsb_c)  <= rx_fifo_half;
            data_o(status_tx_half_msb_c  downto status_tx_half_lsb_c)  <= tx_fifo_half;
            irq.rd_ack <= '1';
          end if;
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


  -- FIFO Level Monitoring ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  level_monitor: process(rx_fifo_level, tx_fifo_level)
  begin
    -- RX FIFO --
    rx_fifo_half <= (others => '0');
    for i in 0 to SLINK_NUM_RX-1 loop
      if (unsigned(rx_fifo_level(i)) >= to_unsigned(cond_sel_natural_f(boolean(SLINK_RX_FIFO > 1), SLINK_RX_FIFO/2, 1), rx_fifo_level(i)'length)) then
        rx_fifo_half(i) <= '1';
      end if;
    end loop;
    -- TX FIFO --
    tx_fifo_half <= (others => '0');
    for i in 0 to SLINK_NUM_TX-1 loop
      if (unsigned(tx_fifo_level(i)) >= to_unsigned(cond_sel_natural_f(boolean(SLINK_TX_FIFO > 1), SLINK_TX_FIFO/2, 1), tx_fifo_level(i)'length)) then
        tx_fifo_half(i) <= '1';
      end if;
    end loop;
  end process level_monitor;


  -- Interrupt Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_arbiter: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (enable = '0') then
        irq.rx_pending <= '0';
        irq.tx_pending <= '0';
      else
        -- RX IRQ --
        if (irq.rx_pending = '0') then
          irq.rx_pending <= irq.rx_fire;
        elsif (irq.rd_ack = '1') or (irq.wr_ack = '1') then
          irq.rx_pending <= '0';
        end if;
        -- TX IRQ --
        if (irq.tx_pending = '0') then
          irq.tx_pending <= irq.tx_fire;
        elsif (irq.rd_ack = '1') or (irq.wr_ack = '1') then
          irq.tx_pending <= '0';
        end if;
      end if;
      -- CPU IRQs --
      irq.rx_pending_ff <= irq.rx_pending;
      irq.tx_pending_ff <= irq.tx_pending;
      irq_rx_o <= irq.rx_pending and (not irq.rx_pending_ff);
      irq_tx_o <= irq.tx_pending and (not irq.tx_pending_ff);
    end if;
  end process irq_arbiter;

  -- status buffer --
  irq_generator_sync: process(clk_i)
  begin
    if rising_edge(clk_i) then
      rx_fifo_avail_ff <= rx_fifo_avail;
      rx_fifo_half_ff  <= rx_fifo_half;
      tx_fifo_free_ff  <= tx_fifo_free;
      tx_fifo_half_ff  <= tx_fifo_half;
    end if;
  end process irq_generator_sync;

  -- IRQ event detector --
  -- RX interrupt: fire if any RX_FIFO gets full / fire if any RX_FIFO.level becomes half-full
  irq.rx_fire <= or_reduce_f(rx_fifo_avail and (not rx_fifo_avail_ff)) when (SLINK_RX_FIFO = 1) else or_reduce_f(rx_fifo_half and (not rx_fifo_half_ff));
  -- TX interrupt: fire if any TX_FIFO gets empty / fire if any TX_FIFO.level falls below half-full level
  irq.tx_fire <= or_reduce_f(tx_fifo_free and (not tx_fifo_free_ff)) when (SLINK_TX_FIFO = 1) else or_reduce_f((not tx_fifo_half) and tx_fifo_half_ff);


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
      FIFO_SAFE  => true           -- true = allow read/write only if entry available
    )
    port map (
      -- control --
      clk_i   => clk_i,             -- clock, rising edge
      rstn_i  => '1',               -- async reset, low-active
      clear_i => fifo_clear,        -- sync reset, high-active
      level_o => tx_fifo_level(i),  -- fill level
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
    tx_fifo_level(i)  <= (others => '0');
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
      FIFO_SAFE  => true           -- true = allow read/write only if entry available
    )
    port map (
      -- control --
      clk_i   => clk_i,             -- clock, rising edge
      rstn_i  => '1',               -- async reset, low-active
      clear_i => fifo_clear,        -- sync reset, high-active
      level_o => rx_fifo_level(i),  -- fill level
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
    rx_fifo_level(i)  <= (others => '0');
  end generate;


end neorv32_slink_rtl;
