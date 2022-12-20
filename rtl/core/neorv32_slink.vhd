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
    rstn_i         : in  std_ulogic; -- global reset line, low-active, async
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
  constant ctrl_en_c          : natural :=  0; -- r/w: global enable/reset
  constant ctrl_rx_num_lsb_c  : natural := 16; -- r/-: log2(RX FIFO size)
  constant ctrl_rx_num_msb_c  : natural := 19;
  constant ctrl_tx_num_lsb_c  : natural := 20; -- r/-: log2(TX FIFO size)
  constant ctrl_tx_num_msb_c  : natural := 23;
  constant ctrl_rx_size_lsb_c : natural := 24; -- r/-: log2(RX FIFO size)
  constant ctrl_rx_size_msb_c : natural := 27;
  constant ctrl_tx_size_lsb_c : natural := 28; -- r/-: log2(TX FIFO size)
  constant ctrl_tx_size_msb_c : natural := 31;

  -- interrupt register bits --
  constant irq_rx_mode_lsb_c : natural :=  0;
  constant irq_rx_mode_msb_c : natural := 15;
  constant irq_tx_mode_lsb_c : natural := 16;
  constant irq_tx_mode_msb_c : natural := 31;

  -- status register bits --
  constant status_empty_lsb_c : natural :=  0; -- r/-: set if RX/TX link 0..7 FIFO is empty
  constant status_empty_msb_c : natural :=  7;
  constant status_half_lsb_c  : natural :=  8; -- r/-: set if RX/TX link 0..7 FIFO is half full
  constant status_half_msb_c  : natural := 15;
  constant status_full_lsb_c  : natural := 16; -- r/-: set if RX/TX link 0..7 FIFO is full
  constant status_full_msb_c  : natural := 23;
  constant status_last_lsb_c  : natural := 24; -- r/(w): set TX packet end of link 0..7
  constant status_last_msb_c  : natural := 31;

  -- bus access control --
  signal acc_en : std_ulogic;
  signal addr   : std_ulogic_vector(31 downto 0);
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- control registers --
  signal enable       : std_ulogic; -- global enable
  signal irq_rx_mode  : std_ulogic_vector(15 downto 0);
  signal irq_tx_mode  : std_ulogic_vector(15 downto 0);
  signal tx_fifo_last : std_ulogic_vector(07 downto 0);

  -- FIFO interface --
  type fifo_data_t is array (0 to 7) of std_ulogic_vector(31 downto 0);
  type fifo_t is record
    avail : std_ulogic_vector(7 downto 0);
    half  : std_ulogic_vector(7 downto 0);
    free  : std_ulogic_vector(7 downto 0);
    we    : std_ulogic_vector(7 downto 0);
    re    : std_ulogic_vector(7 downto 0);
    wlast : std_ulogic_vector(7 downto 0);
    rlast : std_ulogic_vector(7 downto 0);
    clr   : std_ulogic_vector(7 downto 0);
    rdata : fifo_data_t;
    wdata : fifo_data_t;
  end record;
  signal rx_fifo, tx_fifo: fifo_t; 

  -- link select --
  signal link_sel : std_ulogic_vector(7 downto 0);

  -- interrupt generator --
  type trigger_t is array (0 to 7) of std_ulogic_vector(1 downto 0);
  type irq_t is record
    trigger : trigger_t; -- rising-edge detector
    fire    : std_ulogic_vector(7 downto 0);
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


  -- Write Access ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      enable       <= '0';
      irq_rx_mode  <= (others => '0');
      irq_tx_mode  <= (others => '0');
      tx_fifo_last <= (others => '0');
    elsif rising_edge(clk_i) then
      if (wren = '1') then
        if (addr = slink_ctrl_c) then -- control register
          enable <= data_i(ctrl_en_c);
        end if;
        if (addr = slink_irq_c) then -- interrupt configuration register
          irq_rx_mode <= data_i(irq_rx_mode_msb_c downto irq_rx_mode_lsb_c);
          irq_tx_mode <= data_i(irq_tx_mode_msb_c downto irq_tx_mode_lsb_c);
        end if;
        if (addr = slink_tx_status_c) then -- TX link status (end-of-packet)
          tx_fifo_last <= data_i(status_last_msb_c downto status_last_lsb_c);
        end if;
      end if;
    end if;
  end process write_access;


  -- Read Access ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      ack_o  <= rden or wren; -- bus access acknowledge
      data_o <= (others => '0');
      if (rden = '1') then
        case addr is
          when slink_ctrl_c => -- control register
            data_o(ctrl_en_c) <= enable;
            data_o(ctrl_rx_num_msb_c  downto ctrl_rx_num_lsb_c)  <= std_ulogic_vector(to_unsigned(SLINK_NUM_RX, 4));
            data_o(ctrl_tx_num_msb_c  downto ctrl_tx_num_lsb_c)  <= std_ulogic_vector(to_unsigned(SLINK_NUM_TX, 4));
            data_o(ctrl_rx_size_msb_c downto ctrl_rx_size_lsb_c) <= std_ulogic_vector(to_unsigned(index_size_f(SLINK_RX_FIFO), 4));
            data_o(ctrl_tx_size_msb_c downto ctrl_tx_size_lsb_c) <= std_ulogic_vector(to_unsigned(index_size_f(SLINK_TX_FIFO), 4));
          when slink_irq_c => -- interrupt configuration register
            data_o((SLINK_NUM_RX+irq_rx_mode_lsb_c)-1 downto irq_rx_mode_lsb_c) <= irq_rx_mode(SLINK_NUM_RX-1 downto 0);
            data_o((SLINK_NUM_TX+irq_tx_mode_lsb_c)-1 downto irq_tx_mode_lsb_c) <= irq_tx_mode(SLINK_NUM_TX-1 downto 0);
          when slink_rx_status_c => -- RX status register
             for i in 0 to SLINK_NUM_RX-1 loop
              data_o(status_empty_lsb_c + i) <= not rx_fifo.avail(i);
              data_o(status_half_lsb_c  + i) <= rx_fifo.half(i);
              data_o(status_full_lsb_c  + i) <= not rx_fifo.free(i);
              data_o(status_last_lsb_c  + i) <= rx_fifo.rlast(i);
            end loop;
          when slink_tx_status_c => -- TX link status register
            for i in 0 to SLINK_NUM_TX-1 loop
              data_o(status_empty_lsb_c + i) <= not tx_fifo.avail(i);
              data_o(status_half_lsb_c  + i) <= tx_fifo.half(i);
              data_o(status_full_lsb_c  + i) <= not tx_fifo.free(i);
              data_o(status_last_lsb_c  + i) <= tx_fifo_last(i); -- from register!
            end loop;
          when slink_link0_c => data_o <= rx_fifo.rdata(0); -- RX link 0 data
          when slink_link1_c => data_o <= rx_fifo.rdata(1); -- RX link 1 data
          when slink_link2_c => data_o <= rx_fifo.rdata(2); -- RX link 2 data
          when slink_link3_c => data_o <= rx_fifo.rdata(3); -- RX link 3 data
          when slink_link4_c => data_o <= rx_fifo.rdata(4); -- RX link 4 data
          when slink_link5_c => data_o <= rx_fifo.rdata(5); -- RX link 5 data
          when slink_link6_c => data_o <= rx_fifo.rdata(6); -- RX link 6 data
          when slink_link7_c => data_o <= rx_fifo.rdata(7); -- RX link 7 data
          when others        => data_o <= (others => '0');
        end case;
      end if;
    end if;
  end process read_access;


  -- Link Control ---------------------------------------------------------------------------
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

  -- global FIFO control --
  fifo_control:
  for i in 0 to 7 generate
    tx_fifo.clr(i)   <= not enable;
    tx_fifo.we(i)    <= link_sel(i) and wren;
    tx_fifo.wlast(i) <= tx_fifo_last(i);
    tx_fifo.wdata(i) <= data_i;
    --
    rx_fifo.clr(i) <= not enable;
    rx_fifo.re(i)  <= link_sel(i) and rden;
  end generate;


  -- TX Links -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tx_gen:
  for i in 0 to SLINK_NUM_TX-1 generate
    -- data FIFO --
    tx_fifo_inst: neorv32_fifo
    generic map (
      FIFO_DEPTH => SLINK_TX_FIFO, -- number of fifo entries; has to be a power of two; min 1
      FIFO_WIDTH => 32+1,          -- size of data elements in fifo
      FIFO_RSYNC => false,         -- async read
      FIFO_SAFE  => true,          -- safe access
      FIFO_GATE  => false          -- no output gate required
    )
    port map (
      clk_i                => clk_i,            -- clock, rising edge
      rstn_i               => '1',              -- async reset, low-active
      clear_i              => tx_fifo.clr(i),   -- sync reset, high-active
      half_o               => tx_fifo.half(i),  -- FIFO is at least half full
      wdata_i(31 downto 0) => tx_fifo.wdata(i), -- write data
      wdata_i(32)          => tx_fifo.wlast(i), -- end of packet
      we_i                 => tx_fifo.we(i),    -- write enable
      free_o               => tx_fifo.free(i),  -- at least one entry is free when set
      re_i                 => tx_fifo.re(i),    -- read enable
      rdata_o(31 downto 0) => tx_fifo.rdata(i), -- read data
      rdata_o(32)          => tx_fifo.rlast(i), -- end of packet
      avail_o              => tx_fifo.avail(i)  -- data available when set
    );

    -- stream link interface --
    tx_fifo.re(i)     <= slink_tx_rdy_i(i);
    slink_tx_dat_o(i) <= tx_fifo.rdata(i);
    slink_tx_val_o(i) <= tx_fifo.avail(i);
    slink_tx_lst_o(i) <= tx_fifo.rlast(i) and tx_fifo.avail(i);
  end generate;

  -- terminate unimplemented links --
  tx_terminate:
  for i in SLINK_NUM_TX to 7 generate
    tx_fifo.half(i)  <= '0';
    tx_fifo.free(i)  <= '0';
    tx_fifo.re(i)    <= '0';
    tx_fifo.rdata(i) <= (others => '0');
    tx_fifo.rlast(i) <= '0';
    tx_fifo.avail(i) <= '0';
    --
    slink_tx_dat_o(i) <= (others => '0');
    slink_tx_val_o(i) <= '0';
    slink_tx_lst_o(i) <= '0';
  end generate;


  -- RX Links -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rx_gen:
  for i in 0 to SLINK_NUM_RX-1 generate
    -- data FIFO --
    rx_fifo_inst: neorv32_fifo
    generic map (
      FIFO_DEPTH => SLINK_RX_FIFO, -- number of fifo entries; has to be a power of two; min 1
      FIFO_WIDTH => 32+1,          -- size of data elements in fifo
      FIFO_RSYNC => false,         -- async read
      FIFO_SAFE  => true,          -- safe access
      FIFO_GATE  => false          -- no output gate required
    )
    port map (
      clk_i                => clk_i,            -- clock, rising edge
      rstn_i               => '1',              -- async reset, low-active
      clear_i              => rx_fifo.clr(i),   -- sync reset, high-active
      half_o               => rx_fifo.half(i),  -- FIFO is at least half full
      wdata_i(31 downto 0) => rx_fifo.wdata(i), -- write data
      wdata_i(32)          => rx_fifo.wlast(i), -- end of packet
      we_i                 => rx_fifo.we(i),    -- write enable
      free_o               => rx_fifo.free(i),  -- at least one entry is free when set
      re_i                 => rx_fifo.re(i),    -- read enable
      rdata_o(31 downto 0) => rx_fifo.rdata(i), -- read data
      rdata_o(32)          => rx_fifo.rlast(i), -- end of packet
      avail_o              => rx_fifo.avail(i)  -- data available when set
    );
  
    -- stream link interface --
    rx_fifo.wdata(i)  <= slink_rx_dat_i(i);
    rx_fifo.we(i)     <= slink_rx_val_i(i);
    rx_fifo.wlast(i)  <= slink_rx_lst_i(i);
    slink_rx_rdy_o(i) <= rx_fifo.free(i);
  end generate;

  -- terminate unimplemented links --
  rx_terminate:
  for i in SLINK_NUM_RX to 7 generate
    rx_fifo.half(i)  <= '0';
    rx_fifo.wdata(i) <= (others => '0');
    rx_fifo.wlast(i) <= '0';
    rx_fifo.we(i)    <= '0';
    rx_fifo.free(i)  <= '0';
    rx_fifo.rdata(i) <= (others => '0');
    rx_fifo.rlast(i) <= '0';
    rx_fifo.avail(i) <= '0';
    --
    slink_rx_rdy_o(i) <= '0';
  end generate;


  -- Interrupt Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_generator: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- RX --
      rx_irq.trigger <= (others => (others => '0'));
      for i in 0 to SLINK_NUM_RX-1 loop
        if (irq_rx_mode(2*i+1) = '1') then -- enabled
          if (irq_rx_mode(2*i+0) = '0') then -- type
            rx_irq.trigger(i) <= rx_irq.trigger(i)(0) & (rx_fifo.avail(i)); -- FIFO becomes not empty
          else
            rx_irq.trigger(i) <= rx_irq.trigger(i)(0) & (rx_fifo.half(i)); -- FIFO becomes at least half full
          end if;
        end if;
      end loop; -- i

      -- TX --
      tx_irq.trigger <= (others => (others => '0'));
      for i in 0 to SLINK_NUM_TX-1 loop
        if (irq_tx_mode(2*i+1) = '1') then -- enabled
          if (irq_tx_mode(2*i+0) = '0') then -- type
            tx_irq.trigger(i) <= tx_irq.trigger(i)(0) & (not tx_fifo.avail(i)); -- FIFO becomes empty
          else
            tx_irq.trigger(i) <= tx_irq.trigger(i)(0) & (not tx_fifo.half(i)); -- FIFO becomes less than half full
          end if;
        end if;
      end loop; -- i

      -- IRQ to CPU --
      irq_rx_o <= enable and or_reduce_f(rx_irq.fire);
      irq_tx_o <= enable and or_reduce_f(tx_irq.fire);
    end if;
  end process irq_generator;
  
  -- edge detector --
  irq_detect: process(rx_irq.trigger, tx_irq.trigger)
  begin
    -- RX --
    rx_irq.fire <= (others => '0');
    for i in 0 to SLINK_NUM_RX-1 loop
      if (rx_irq.trigger(i) = "01") then
        rx_irq.fire(i) <= '1';
      end if;
    end loop;
    -- TX --
    tx_irq.fire <= (others => '0');
    for i in 0 to SLINK_NUM_TX-1 loop
      if (tx_irq.trigger(i) = "01") then
        tx_irq.fire(i) <= '1';
      end if;
    end loop;
  end process irq_detect;


end neorv32_slink_rtl;
