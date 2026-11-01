-- ================================================================================ --
-- NEORV32 SoC - Stream Link Interface (SLINK)                                      --
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

entity neorv32_slink is
  generic (
    RX_FIFO_DEPTH : natural range 1 to 32768; -- RX FIFO depth, has to be a power of two, min 1
    TX_FIFO_DEPTH : natural range 1 to 32768  -- TX FIFO depth, has to be a power of two, min 1
  );
  port (
    -- Host access --
    clk_i            : in  std_ulogic;                     -- global clock line
    rstn_i           : in  std_ulogic;                     -- global reset line, low-active, async
    bus_req_i        : in  bus_req_t;                      -- bus request
    bus_rsp_o        : out bus_rsp_t;                      -- bus response
    irq_o            : out std_ulogic;                     -- interrupt
    -- RX stream interface --
    slink_rx_data_i  : in  std_ulogic_vector(31 downto 0); -- input data
    slink_rx_valid_i : in  std_ulogic;                     -- valid input
    slink_rx_last_i  : in  std_ulogic;                     -- end of stream
    slink_rx_ready_o : out std_ulogic;                     -- ready to receive
    -- TX stream interface --
    slink_tx_data_o  : out std_ulogic_vector(31 downto 0); -- output data
    slink_tx_valid_o : out std_ulogic;                     -- valid output
    slink_tx_last_o  : out std_ulogic;                     -- end of stream
    slink_tx_ready_i : in  std_ulogic                      -- ready to send
  );
end entity;

architecture neorv32_slink_rtl of neorv32_slink is

  -- control register --
  constant ctrl_rx_en_c       : natural :=  0; -- r/w: enable receiver
  constant ctrl_tx_en_c       : natural :=  1; -- r/w: enable transmitter
  --
  constant ctrl_rx_last_c     : natural :=  4; -- r/-: RX TOP word is last word of package
  --
  constant ctrl_rx_empty_c    : natural :=  8; -- r/-: RX FIFO empty
  constant ctrl_rx_half_c     : natural :=  9; -- r/-: RX FIFO at least half full
  constant ctrl_rx_full_c     : natural := 10; -- r/-: RX FIFO full
  constant ctrl_tx_empty_c    : natural := 11; -- r/-: TX FIFO empty
  constant ctrl_tx_half_c     : natural := 12; -- r/-: TX FIFO at least half full
  constant ctrl_tx_full_c     : natural := 13; -- r/-: TX FIFO full
  --
  constant ctrl_rx_irq_sel0_c : natural := 16; -- r/w: RX interrupt select, bit 0 (LSB)
  constant ctrl_rx_irq_sel1_c : natural := 17; -- r/w: RX interrupt select, bit 1 (MSB)
  constant ctrl_tx_irq_sel0_c : natural := 18; -- r/w: TX interrupt select, bit 0 (LSB)
  constant ctrl_tx_irq_sel1_c : natural := 19; -- r/w: TX interrupt select, bit 1 (MSB)
  constant ctrl_rx_irq_pend_c : natural := 20; -- r/-: pending RX interrupt
  constant ctrl_tx_irq_pend_c : natural := 21; -- r/-: pending TX interrupt
  --
  constant ctrl_rx_fifo0_c    : natural := 24; -- r/-: log2(RX FIFO size), bit 0 (LSB)
  constant ctrl_rx_fifo3_c    : natural := 27; -- r/-: log2(RX FIFO size), bit 3 (MSB)
  constant ctrl_tx_fifo0_c    : natural := 28; -- r/-: log2(TX FIFO size), bit 0 (LSB)
  constant ctrl_tx_fifo3_c    : natural := 31; -- r/-: log2(TX FIFO size), bit 3 (MSB)

  -- helpers --
  constant rx_awidth_c : natural := index_size_f(RX_FIFO_DEPTH);
  constant tx_awidth_c : natural := index_size_f(TX_FIFO_DEPTH);

  -- bus interface --
  signal bus_ack, bus_rden : std_ulogic;
  signal bus_rdata : std_ulogic_vector(31 downto 0);

  -- control register set --
  type ctrl_t is record
    rx_enable  : std_ulogic;
    tx_enable  : std_ulogic;
    rx_irq_sel : std_ulogic_vector(1 downto 0);
    tx_irq_sel : std_ulogic_vector(1 downto 0);
  end record;
  signal ctrl : ctrl_t;

  -- FIFO interfaces --
  type fifo_t is record
    clear, we, re     : std_ulogic;
    wdata, rdata      : std_ulogic_vector(32 downto 0);
    avail, free, half : std_ulogic;
  end record;
  signal tx_fifo, rx_fifo : fifo_t;
  signal rx_level : std_ulogic_vector(rx_awidth_c downto 0);
  signal tx_level : std_ulogic_vector(tx_awidth_c downto 0);

  -- interrupts --
  signal rx_irq, tx_irq : std_ulogic;

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
      ctrl.rx_enable  <= '0';
      ctrl.tx_enable  <= '0';
      ctrl.rx_irq_sel <= (others => '0');
      ctrl.tx_irq_sel <= (others => '0');
    elsif rising_edge(clk_i) then
      if (bus_req_i.stb = '1') and (bus_req_i.rw = '1') then
        if (bus_req_i.addr(3 downto 2) = "00") then -- control register
          ctrl.rx_enable  <= bus_req_i.data(ctrl_rx_en_c);
          ctrl.tx_enable  <= bus_req_i.data(ctrl_tx_en_c);
          ctrl.rx_irq_sel <= bus_req_i.data(ctrl_rx_irq_sel1_c downto ctrl_rx_irq_sel0_c);
          ctrl.tx_irq_sel <= bus_req_i.data(ctrl_tx_irq_sel1_c downto ctrl_tx_irq_sel0_c);
        end if;
      end if;
    end if;
  end process;

  -- read access (asynchronous) --
  bus_read: process(bus_rden, bus_req_i.addr, ctrl, rx_fifo, tx_fifo, rx_irq, tx_irq)
  begin
    bus_rdata <= (others => '0');
    if (bus_rden = '1') then -- output gating
      if (bus_req_i.addr(3 downto 2) = "00") then -- control register
        bus_rdata(ctrl_rx_en_c) <= ctrl.rx_enable;
        bus_rdata(ctrl_tx_en_c) <= ctrl.tx_enable;
        --
        bus_rdata(ctrl_rx_last_c) <= rx_fifo.rdata(32);
        --
        bus_rdata(ctrl_rx_empty_c) <= not rx_fifo.avail;
        bus_rdata(ctrl_rx_half_c)  <= rx_fifo.half;
        bus_rdata(ctrl_rx_full_c)  <= not rx_fifo.free;
        bus_rdata(ctrl_tx_empty_c) <= not tx_fifo.avail;
        bus_rdata(ctrl_tx_half_c)  <= tx_fifo.half;
        bus_rdata(ctrl_tx_full_c)  <= not tx_fifo.free;
        --
        bus_rdata(ctrl_rx_irq_sel1_c downto ctrl_rx_irq_sel0_c) <= ctrl.rx_irq_sel;
        bus_rdata(ctrl_tx_irq_sel1_c downto ctrl_tx_irq_sel0_c) <= ctrl.tx_irq_sel;
        bus_rdata(ctrl_rx_irq_pend_c)                           <= rx_irq;
        bus_rdata(ctrl_tx_irq_pend_c)                           <= tx_irq;
        --
        bus_rdata(ctrl_rx_fifo3_c downto ctrl_rx_fifo0_c) <= std_ulogic_vector(to_unsigned(rx_awidth_c, 4));
        bus_rdata(ctrl_tx_fifo3_c downto ctrl_tx_fifo0_c) <= std_ulogic_vector(to_unsigned(tx_awidth_c, 4));
      else -- RX data
        bus_rdata <= rx_fifo.rdata(31 downto 0);
      end if;
    end if;
  end process;

  -- bus response --
  bus_rsp_o.ack  <= bus_ack;
  bus_rsp_o.err  <= '0'; -- no access errors supported
  bus_rsp_o.data <= bus_rdata;

  -- CPU interrupt --
  irq_gen: process(clk_i)
  begin
    if rising_edge(clk_i) then
      irq_o <= (ctrl.rx_enable and rx_irq) or (ctrl.tx_enable and tx_irq);
    end if;
  end process;


  -- Receiver (RX) --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rx_fifo_inst: entity neorv32.neorv32_prim_fifo
  generic map (
    AWIDTH  => rx_awidth_c,
    DWIDTH  => 1+32, -- last + data
    OUTGATE => true, -- to prevent getting X in simulation
    ASYNCRD => false
  )
  port map (
    -- global control --
    clk_i   => clk_i,
    rstn_i  => rstn_i,
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

  rx_fifo.clear    <= not ctrl.rx_enable;
  rx_fifo.re       <= bus_rden and bus_req_i.addr(3);
  rx_fifo.we       <= slink_rx_valid_i;
  rx_fifo.wdata    <= slink_rx_last_i & slink_rx_data_i;
  slink_rx_ready_o <= rx_fifo.free and ctrl.rx_enable;

  rx_fifo_small:
  if (rx_awidth_c = 0) generate
    rx_fifo.half <= rx_fifo.avail;
  end generate;

  rx_fifo_large:
  if (rx_awidth_c > 0) generate
    rx_fifo.half <= rx_level(rx_awidth_c) or rx_level(rx_awidth_c-1);
  end generate;

  -- interrupt generator --
  rx_irq_trigger: process(ctrl.rx_enable, rx_fifo)
  begin
    case ctrl.rx_irq_sel is
      when "01"   => rx_irq <= ctrl.rx_enable and      rx_fifo.avail; -- FIFO not empty (data available)
      when "10"   => rx_irq <= ctrl.rx_enable and      rx_fifo.half;  -- FIFO at least half full
      when "11"   => rx_irq <= ctrl.rx_enable and (not rx_fifo.free); -- FIFO full
      when others => rx_irq <= '0'; -- disabled
    end case;
  end process;


  -- Transmitter (TX) -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tx_fifo_inst: entity neorv32.neorv32_prim_fifo
  generic map (
    AWIDTH  => tx_awidth_c,
    DWIDTH  => 1+32, -- last + data
    OUTGATE => false,
    ASYNCRD => false
  )
  port map (
    -- global control --
    clk_i   => clk_i,
    rstn_i  => rstn_i,
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

  tx_fifo.clear    <= not ctrl.tx_enable;
  tx_fifo.we       <= bus_req_i.stb and bus_req_i.rw and bus_req_i.addr(3);
  tx_fifo.wdata    <= bus_req_i.addr(2) & bus_req_i.data; -- last-flag is set via access address
  tx_fifo.re       <= slink_tx_ready_i;
  slink_tx_data_o  <= tx_fifo.rdata(31 downto 0);
  slink_tx_last_o  <= tx_fifo.rdata(32);
  slink_tx_valid_o <= tx_fifo.avail;

  tx_fifo_small:
  if (tx_awidth_c = 0) generate
    tx_fifo.half <= tx_fifo.avail;
  end generate;

  tx_fifo_large:
  if (tx_awidth_c > 0) generate
    tx_fifo.half <= tx_level(tx_awidth_c) or tx_level(tx_awidth_c-1);
  end generate;

  -- interrupt generator --
  tx_irq_trigger: process(ctrl.tx_enable, tx_fifo)
  begin
    case ctrl.tx_irq_sel is
      when "01"   => tx_irq <= ctrl.tx_enable and (not tx_fifo.avail); -- FIFO empty
      when "10"   => tx_irq <= ctrl.tx_enable and (not tx_fifo.half);  -- FIFO less than half full
      when "11"   => tx_irq <= ctrl.tx_enable and      tx_fifo.free;   -- FIFO not full (free space available)
      when others => tx_irq <= '0'; -- disabled
    end case;
  end process;

end architecture;
