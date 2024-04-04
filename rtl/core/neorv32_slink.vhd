-- #################################################################################################
-- # << NEORV32 - Stream Link Interface (SLINK) >>                                                 #
-- # ********************************************************************************************* #
-- # Two independent stream links for RX and TX each equipped with a configurable FIFO and         #
-- # individually programmable interrupt conditions (based on the FIFO status flags).              #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # The NEORV32 RISC-V Processor, https://github.com/stnolting/neorv32                            #
-- # Copyright (c) 2024, Stephan Nolting. All rights reserved.                                     #
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
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_slink is
  generic (
    SLINK_RX_FIFO : natural range 1 to 2**15; -- RX fifo depth, has to be a power of two, min 1
    SLINK_TX_FIFO : natural range 1 to 2**15  -- TX fifo depth, has to be a power of two, min 1
  );
  port (
    -- Host access --
    clk_i            : in  std_ulogic; -- global clock line
    rstn_i           : in  std_ulogic; -- global reset line, low-active, async
    bus_req_i        : in  bus_req_t;  -- bus request
    bus_rsp_o        : out bus_rsp_t;  -- bus response
    rx_irq_o         : out std_ulogic; -- RX interrupt
    tx_irq_o         : out std_ulogic; -- TX interrupt
    -- RX stream interface --
    slink_rx_data_i  : in  std_ulogic_vector(31 downto 0); -- input data
    slink_rx_valid_i : in  std_ulogic; -- valid input
    slink_rx_last_i  : in  std_ulogic; -- end of stream
    slink_rx_ready_o : out std_ulogic; -- ready to receive
    -- TX stream interface --
    slink_tx_data_o  : out std_ulogic_vector(31 downto 0); -- output data
    slink_tx_valid_o : out std_ulogic; -- valid output
    slink_tx_last_o  : out std_ulogic; -- end of stream
    slink_tx_ready_i : in  std_ulogic  -- ready to send
  );
end neorv32_slink;

architecture neorv32_slink_rtl of neorv32_slink is

  -- memory-mapped interface registers --
  constant addr_ctrl_c    : std_ulogic_vector(1 downto 0) := "00"; -- control register
  constant addr_rx_c      : std_ulogic_vector(1 downto 0) := "01"; -- RX data
  constant addr_tx_c      : std_ulogic_vector(1 downto 0) := "10"; -- TX data
  constant addr_tx_last_c : std_ulogic_vector(1 downto 0) := "11"; -- TX data + last-delimiter

  -- control register --
  constant ctrl_en_c            : natural :=  0; -- r/w: Global module enable
  constant ctrl_rx_clr_c        : natural :=  1; -- -/w: Clear RX FIFO, auto-clears
  constant ctrl_tx_clr_c        : natural :=  2; -- -/w: Clear TX FIFO, auto-clears
  --
  constant ctrl_rx_last_c       : natural :=  4; -- r/-: RX end-of-stream (according to prev. read RX data)
  --
  constant ctrl_rx_empty_c      : natural :=  8; -- r/-: RX FIFO empty
  constant ctrl_rx_half_c       : natural :=  9; -- r/-: RX FIFO at least half full
  constant ctrl_rx_full_c       : natural := 10; -- r/-: RX FIFO full
  constant ctrl_tx_empty_c      : natural := 11; -- r/-: TX FIFO empty
  constant ctrl_tx_half_c       : natural := 12; -- r/-: TX FIFO at least half full
  constant ctrl_tx_full_c       : natural := 13; -- r/-: TX FIFO full
  --
  constant ctrl_irq_rx_nempty_c : natural := 16; -- r/w: RX interrupt if RX FIFO not empty
  constant ctrl_irq_rx_half_c   : natural := 17; -- r/w: RX interrupt if RX FIFO at least half full
  constant ctrl_irq_rx_full_c   : natural := 18; -- r/w: RX interrupt if RX FIFO full
  constant ctrl_irq_tx_empty_c  : natural := 19; -- r/w: TX interrupt if TX FIFO empty
  constant ctrl_irq_tx_nhalf_c  : natural := 20; -- r/w: TX interrupt if TX FIFO not at least half full
  constant ctrl_irq_tx_nfull_c  : natural := 21; -- r/w: TX interrupt if TX FIFO not full
  --
  constant ctrl_rx_fifo_size0_c : natural := 24; -- r/-: log2(RX fifo size), bit 0 (lsb)
  constant ctrl_rx_fifo_size1_c : natural := 25; -- r/-: log2(RX fifo size), bit 1
  constant ctrl_rx_fifo_size2_c : natural := 26; -- r/-: log2(RX fifo size), bit 2
  constant ctrl_rx_fifo_size3_c : natural := 27; -- r/-: log2(RX fifo size), bit 3 (msb)
  constant ctrl_tx_fifo_size0_c : natural := 28; -- r/-: log2(TX fifo size), bit 0 (lsb)
  constant ctrl_tx_fifo_size1_c : natural := 29; -- r/-: log2(TX fifo size), bit 1
  constant ctrl_tx_fifo_size2_c : natural := 30; -- r/-: log2(TX fifo size), bit 2
  constant ctrl_tx_fifo_size3_c : natural := 31; -- r/-: log2(TX fifo size), bit 3 (msb)

  -- control register --
  type ctrl_t is record
    enable                      : std_ulogic;
    rx_clr,        tx_clr       : std_ulogic;
    irq_rx_nempty, irq_tx_empty : std_ulogic;
    irq_rx_half,   irq_tx_nhalf : std_ulogic;
    irq_rx_full,   irq_tx_nfull : std_ulogic;
  end record;
  signal ctrl : ctrl_t;

  -- RX end-of-stream indicator --
  signal rx_last : std_ulogic;

  -- FIFO interface --
  type fifo_t is record
    we    : std_ulogic; -- write enable
    re    : std_ulogic; -- read enable
    clear : std_ulogic; -- sync reset, high-active
    wdata : std_ulogic_vector(32 downto 0); -- write data + last-flag
    rdata : std_ulogic_vector(32 downto 0); -- read data + last-flag
    avail : std_ulogic; -- data available?
    free  : std_ulogic; -- free entry available?
    half  : std_ulogic; -- half full
  end record;
  signal tx_fifo, rx_fifo : fifo_t;

begin

  -- Bus Access -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bus_rsp_o.ack      <= '0';
      bus_rsp_o.err      <= '0';
      bus_rsp_o.data     <= (others => '0');
      ctrl.enable        <= '0';
      ctrl.rx_clr        <= '0';
      ctrl.tx_clr        <= '0';
      ctrl.irq_rx_nempty <= '0';
      ctrl.irq_rx_half   <= '0';
      ctrl.irq_rx_full   <= '0';
      ctrl.irq_tx_empty  <= '0';
      ctrl.irq_tx_nhalf  <= '0';
      ctrl.irq_tx_nfull  <= '0';
    elsif rising_edge(clk_i) then
      -- bus handshake --
      bus_rsp_o.ack  <= bus_req_i.stb;
      bus_rsp_o.err  <= '0';
      bus_rsp_o.data <= (others => '0');

      -- defaults --
      ctrl.rx_clr <= '0'; -- auto-clear
      ctrl.tx_clr <= '0'; -- auto-clear

      if (bus_req_i.stb = '1') then
        -- write access --
        if (bus_req_i.rw = '1') then
          if (bus_req_i.addr(3 downto 2) = addr_ctrl_c) then -- control register
            ctrl.enable <= bus_req_i.data(ctrl_en_c);
            ctrl.rx_clr <= bus_req_i.data(ctrl_rx_clr_c);
            ctrl.tx_clr <= bus_req_i.data(ctrl_tx_clr_c);
            --
            ctrl.irq_rx_nempty <= bus_req_i.data(ctrl_irq_rx_nempty_c);
            ctrl.irq_rx_half   <= bus_req_i.data(ctrl_irq_rx_half_c);
            ctrl.irq_rx_full   <= bus_req_i.data(ctrl_irq_rx_full_c);
            ctrl.irq_tx_empty  <= bus_req_i.data(ctrl_irq_tx_empty_c);
            ctrl.irq_tx_nhalf  <= bus_req_i.data(ctrl_irq_tx_nhalf_c);
            ctrl.irq_tx_nfull  <= bus_req_i.data(ctrl_irq_tx_nfull_c);
          end if;
        -- read access --
        else
          if (bus_req_i.addr(3 downto 2) = addr_ctrl_c) then -- control register
            bus_rsp_o.data(ctrl_en_c) <= ctrl.enable;
            --
            bus_rsp_o.data(ctrl_rx_last_c) <= rx_last;
            --
            bus_rsp_o.data(ctrl_rx_empty_c) <= not rx_fifo.avail;
            bus_rsp_o.data(ctrl_rx_half_c)  <= rx_fifo.half;
            bus_rsp_o.data(ctrl_rx_full_c)  <= not rx_fifo.free;
            bus_rsp_o.data(ctrl_tx_empty_c) <= not tx_fifo.avail;
            bus_rsp_o.data(ctrl_tx_half_c)  <= tx_fifo.half;
            bus_rsp_o.data(ctrl_tx_full_c)  <= not tx_fifo.free;
            --
            bus_rsp_o.data(ctrl_irq_rx_nempty_c) <= ctrl.irq_rx_nempty;
            bus_rsp_o.data(ctrl_irq_rx_half_c)   <= ctrl.irq_rx_half;
            bus_rsp_o.data(ctrl_irq_rx_full_c)   <= ctrl.irq_rx_full;
            bus_rsp_o.data(ctrl_irq_tx_empty_c)  <= ctrl.irq_tx_empty;
            bus_rsp_o.data(ctrl_irq_tx_nhalf_c)  <= ctrl.irq_tx_nhalf;
            bus_rsp_o.data(ctrl_irq_tx_nfull_c)  <= ctrl.irq_tx_nfull;
            --
            bus_rsp_o.data(ctrl_rx_fifo_size3_c downto ctrl_rx_fifo_size0_c) <= std_ulogic_vector(to_unsigned(index_size_f(SLINK_RX_FIFO), 4));
            bus_rsp_o.data(ctrl_tx_fifo_size3_c downto ctrl_tx_fifo_size0_c) <= std_ulogic_vector(to_unsigned(index_size_f(SLINK_TX_FIFO), 4));
          else -- RX (TX) data register
            bus_rsp_o.data <= rx_fifo.rdata(31 downto 0);
          end if;
        end if;
      end if;
    end if;
  end process bus_access;


  -- RX Data FIFO ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rx_fifo_inst: entity neorv32.neorv32_fifo
  generic map (
    FIFO_DEPTH => SLINK_RX_FIFO,
    FIFO_WIDTH => 32+1,  -- data + last-flag
    FIFO_RSYNC => false, -- "async" read - update FIFO status RIGHT after write access (for slink_rx_ready_o)
    FIFO_SAFE  => true,  -- safe access
    FULL_RESET => false  -- no HW reset, try to infer BRAM
  )
  port map (
    -- control --
    clk_i   => clk_i,
    rstn_i  => rstn_i,
    clear_i => rx_fifo.clear,
    half_o  => rx_fifo.half,
    -- write port --
    wdata_i => rx_fifo.wdata,
    we_i    => rx_fifo.we,
    free_o  => rx_fifo.free,
    -- read port --
    re_i    => rx_fifo.re,
    rdata_o => rx_fifo.rdata,
    avail_o => rx_fifo.avail
  );

  rx_fifo.clear <= (not ctrl.enable) or ctrl.rx_clr;
  rx_fifo.re    <= '1' when (bus_req_i.stb = '1') and (bus_req_i.rw = '0') and (bus_req_i.addr(3 downto 2) = addr_rx_c) else '0';

  rx_fifo.we                 <= slink_rx_valid_i;
  rx_fifo.wdata(31 downto 0) <= slink_rx_data_i;
  rx_fifo.wdata(32)          <= slink_rx_last_i;
  slink_rx_ready_o           <= rx_fifo.free;

  -- backup current RX last indicator for current access --
  rx_last_flag: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      rx_last <= '0';
    elsif rising_edge(clk_i) then
      if (rx_fifo.clear = '1') then
        rx_last <= '0';
      elsif (rx_fifo.re = '1') then
        rx_last <= rx_fifo.rdata(32);
      end if;
    end if;
  end process rx_last_flag;

  -- interrupt generator --
  rx_interrupt: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      rx_irq_o <= '0';
    elsif rising_edge(clk_i) then
      rx_irq_o <= ctrl.enable and ( -- IRQ if enabled and ...
                  (ctrl.irq_rx_nempty and (    rx_fifo.avail)) or -- RX FIFO is not empty
                  (ctrl.irq_rx_half   and (    rx_fifo.half))  or -- RX FIFO is at least half full
                  (ctrl.irq_rx_full   and (not rx_fifo.free)));   -- RX FIFO is full
    end if;
  end process rx_interrupt;


  -- TX Data FIFO ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tx_fifo_inst: entity neorv32.neorv32_fifo
  generic map (
    FIFO_DEPTH => SLINK_TX_FIFO,
    FIFO_WIDTH => 32+1,  -- data + last-flag
    FIFO_RSYNC => false, -- "async" read - update FIFO status RIGHT after read access (for slink_tx_valid_o)
    FIFO_SAFE  => true,  -- safe access
    FULL_RESET => false  -- no HW reset, try to infer BRAM
  )
  port map (
    -- control --
    clk_i   => clk_i,
    rstn_i  => rstn_i,
    clear_i => tx_fifo.clear,
    half_o  => tx_fifo.half,
    -- write port --
    wdata_i => tx_fifo.wdata,
    we_i    => tx_fifo.we,
    free_o  => tx_fifo.free,
    -- read port --
    re_i    => tx_fifo.re,
    rdata_o => tx_fifo.rdata,
    avail_o => tx_fifo.avail
  );

  tx_fifo.clear <= (not ctrl.enable) or ctrl.tx_clr;
  tx_fifo.we    <= '1' when (bus_req_i.stb = '1') and (bus_req_i.rw = '1') and (bus_req_i.addr(3) = '1') else '0';
  tx_fifo.wdata <= bus_req_i.addr(2) & bus_req_i.data; -- last-flag is set implicitly via access address (TX/TX_LAST register)

  tx_fifo.re       <= slink_tx_ready_i;
  slink_tx_data_o  <= tx_fifo.rdata(31 downto 0);
  slink_tx_last_o  <= tx_fifo.rdata(32) and tx_fifo.avail;
  slink_tx_valid_o <= tx_fifo.avail;

  -- interrupt generator --
  tx_interrupt: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      tx_irq_o <= '0';
    elsif rising_edge(clk_i) then
      tx_irq_o <= ctrl.enable and ( -- IRQ if enabled and ...
                  (ctrl.irq_tx_empty  and (not tx_fifo.avail)) or -- TX FIFO is empty
                  (ctrl.irq_tx_nhalf  and (not tx_fifo.half))  or -- TX FIFO is not at least half full
                  (ctrl.irq_tx_nfull  and (    tx_fifo.free)));   -- TX FIFO is not full
    end if;
  end process tx_interrupt;


end neorv32_slink_rtl;
