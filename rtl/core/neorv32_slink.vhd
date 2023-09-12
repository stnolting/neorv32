-- #################################################################################################
-- # << NEORV32 - Stream Link Interface (SLINK) >>                                                 #
-- # ********************************************************************************************* #
-- # Two independent stream links for RX and TX each equipped with a configurable FIFO and         #
-- # providing programmable interrupt conditions.                                                  #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
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
    SLINK_RX_FIFO : natural range 1 to 2**15; -- RX fifo depth, has to be a power of two
    SLINK_TX_FIFO : natural range 1 to 2**15  -- TX fifo depth, has to be a power of two
  );
  port (
    -- Host access --
    clk_i            : in  std_ulogic; -- global clock line
    rstn_i           : in  std_ulogic; -- global reset line, low-active, async
    bus_req_i        : in  bus_req_t;  -- bus request
    bus_rsp_o        : out bus_rsp_t;  -- bus response
    irq_o            : out std_ulogic; -- CPU interrupt
    -- RX stream interface --
    slink_rx_data_i  : in  std_ulogic_vector(31 downto 0); -- input data
    slink_rx_valid_i : in  std_ulogic; -- valid input
    slink_rx_ready_o : out std_ulogic; -- ready to receive
    -- TX stream interface --
    slink_tx_data_o  : out std_ulogic_vector(31 downto 0); -- output data
    slink_tx_valid_o : out std_ulogic; -- valid output
    slink_tx_ready_i : in  std_ulogic  -- ready to send
  );
end neorv32_slink;

architecture neorv32_slink_rtl of neorv32_slink is

  -- control register --
  constant ctrl_en_c            : natural :=  0; -- r/w: Global module enable
  constant ctrl_rx_clr_c        : natural :=  1; -- -/w: Clear RX FIFO, auto-clears
  constant ctrl_tx_clr_c        : natural :=  2; -- -/w: Clear TX FIFO, auto-clears
  --
  constant ctrl_rx_empty_c      : natural :=  8; -- r/-: RX FIFO empty
  constant ctrl_rx_half_c       : natural :=  9; -- r/-: RX FIFO at least half full
  constant ctrl_rx_full_c       : natural := 10; -- r/-: RX FIFO full
  constant ctrl_tx_empty_c      : natural := 11; -- r/-: TX FIFO empty
  constant ctrl_tx_half_c       : natural := 12; -- r/-: TX FIFO at least half full
  constant ctrl_tx_full_c       : natural := 13; -- r/-: TX FIFO full
  --
  constant ctrl_irq_rx_nempty_c : natural := 16; -- r/w: IRQ if RX FIFO not empty
  constant ctrl_irq_rx_half_c   : natural := 17; -- r/w: IRQ if RX FIFO at least half full
  constant ctrl_irq_rx_full_c   : natural := 18; -- r/w: IRQ if RX FIFO full
  constant ctrl_irq_tx_empty_c  : natural := 19; -- r/w: IRQ if TX FIFO empty
  constant ctrl_irq_tx_nhalf_c  : natural := 20; -- r/w: IRQ if TX FIFO not at least half full
  constant ctrl_irq_tx_nfull_c  : natural := 21; -- r/w: IRQ if TX FIFO not full
  --
  constant ctrl_rx_fifo_size0_c : natural := 24; -- r/-: log2(RX fifo size), bit 0 (lsb)
  constant ctrl_rx_fifo_size1_c : natural := 25; -- r/-: log2(RX fifo size), bit 1
  constant ctrl_rx_fifo_size2_c : natural := 26; -- r/-: log2(RX fifo size), bit 2
  constant ctrl_rx_fifo_size3_c : natural := 27; -- r/-: log2(RX fifo size), bit 3 (msb)
  constant ctrl_tx_fifo_size0_c : natural := 28; -- r/-: log2(TX fifo size), bit 0 (lsb)
  constant ctrl_tx_fifo_size1_c : natural := 29; -- r/-: log2(TX fifo size), bit 1
  constant ctrl_tx_fifo_size2_c : natural := 30; -- r/-: log2(TX fifo size), bit 2
  constant ctrl_tx_fifo_size3_c : natural := 31; -- r/-: log2(TX fifo size), bit 3 (msb)

  -- control register (see bit definitions above) --
  type ctrl_t is record
    enable        : std_ulogic;
    rx_clr        : std_ulogic;
    tx_clr        : std_ulogic;
    irq_rx_nempty : std_ulogic;
    irq_rx_half   : std_ulogic;
    irq_rx_full   : std_ulogic;
    irq_tx_empty  : std_ulogic;
    irq_tx_nhalf  : std_ulogic;
    irq_tx_nfull  : std_ulogic;
  end record;
  signal ctrl : ctrl_t;

  -- FIFO interface --
  type fifo_t is record
    we    : std_ulogic; -- write enable
    re    : std_ulogic; -- read enable
    clear : std_ulogic; -- sync reset, high-active
    wdata : std_ulogic_vector(31 downto 0); -- write data
    rdata : std_ulogic_vector(31 downto 0); -- read data
    avail : std_ulogic; -- data available?
    free  : std_ulogic; -- free entry available?
    half  : std_ulogic; -- half full
  end record;
  signal tx_fifo, rx_fifo : fifo_t;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not ((is_power_of_two_f(SLINK_TX_FIFO) = false) or (is_power_of_two_f(SLINK_RX_FIFO) = false)) report
    "NEORV32 PROCESSOR CONFIG ERROR: SLINK FIFO sizes have to be a power of two." severity error;


  -- Host Access ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- write access --
  write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
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
      ctrl.rx_clr <= '0'; -- auto-clear
      ctrl.tx_clr <= '0'; -- auto-clear
      if (bus_req_i.we = '1') then
        if (bus_req_i.addr(2) = '0') then -- control register
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
      end if;
    end if;
  end process write_access;

  -- read access --
  read_aceess: process(clk_i)
  begin
    if rising_edge(clk_i) then
      bus_rsp_o.ack  <= bus_req_i.re or bus_req_i.we; -- bus access acknowledge
      bus_rsp_o.data <= (others => '0');
      if (bus_req_i.re = '1') then
        if (bus_req_i.addr(2) = '0') then -- control register
          bus_rsp_o.data(ctrl_en_c) <= ctrl.enable;
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
        else -- RX/TX data register
          bus_rsp_o.data <= rx_fifo.rdata(31 downto 0);
        end if;
      end if;
    end if;
  end process read_aceess;

  -- no access error possible --
  bus_rsp_o.err <= '0';


  -- RX Data FIFO ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rx_fifo_inst: entity neorv32.neorv32_fifo
  generic map (
    FIFO_DEPTH => SLINK_RX_FIFO,
    FIFO_WIDTH => 32,
    FIFO_RSYNC => true, -- sync read
    FIFO_SAFE  => true  -- safe access
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
  rx_fifo.re    <= '1' when (bus_req_i.re = '1') and (bus_req_i.addr(2) = '1') else '0';

  rx_fifo.we       <= slink_rx_valid_i;
  rx_fifo.wdata    <= slink_rx_data_i;
  slink_rx_ready_o <= rx_fifo.free;


  -- TX Data FIFO ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tx_fifo_inst: entity neorv32.neorv32_fifo
  generic map (
    FIFO_DEPTH => SLINK_TX_FIFO,
    FIFO_WIDTH => 32,
    FIFO_RSYNC => true, -- sync read
    FIFO_SAFE  => true  -- safe access
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
  tx_fifo.we    <= '1' when (bus_req_i.we = '1') and (bus_req_i.addr(2) = '1') else '0';
  tx_fifo.wdata <= bus_req_i.data;

  tx_fifo.re       <= slink_tx_ready_i;
  slink_tx_data_o  <= tx_fifo.rdata;
  slink_tx_valid_o <= tx_fifo.avail;


  -- Interrupt Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_generator: process(clk_i)
  begin
    if rising_edge(clk_i) then
      irq_o <= ctrl.enable and ( -- IRQ if enabled and ...
              (ctrl.irq_tx_empty  and (not tx_fifo.avail)) or -- TX FIFO is empty
              (ctrl.irq_tx_nhalf  and (not tx_fifo.half))  or -- TX FIFO is not at least half full
              (ctrl.irq_tx_nfull  and (    tx_fifo.free))  or -- TX FIFO is not full
              (ctrl.irq_rx_nempty and (    rx_fifo.avail)) or -- RX FIFO is not empty
              (ctrl.irq_rx_half   and (    rx_fifo.half))  or -- RX FIFO is at least half full
              (ctrl.irq_rx_full   and (not rx_fifo.free)));   -- RX FIFO is full
    end if;
  end process irq_generator;


end neorv32_slink_rtl;
