-- ================================================================================ --
-- NEORV32 SoC - Two-Wire Device (TWD)                                              --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_twd is
  generic (
    TWD_RX_FIFO : natural range 1 to 2**15; -- receive FIFO depth, has to be a power of two, min 1
    TWD_TX_FIFO : natural range 1 to 2**15  -- transmit FIFO depth, has to be a power of two, min 1
  );
  port (
    clk_i     : in  std_ulogic;                    -- global clock line
    rstn_i    : in  std_ulogic;                    -- global reset line, low-active, async
    bus_req_i : in  bus_req_t;                     -- bus request
    bus_rsp_o : out bus_rsp_t;                     -- bus response
    clkgen_i  : in  std_ulogic_vector(7 downto 0); -- prescaled clock enables
    twd_sda_i : in  std_ulogic;                    -- serial data line input
    twd_sda_o : out std_ulogic;                    -- serial data line output
    twd_scl_i : in  std_ulogic;                    -- serial clock line input
    twd_scl_o : out std_ulogic;                    -- serial clock line output
    irq_o     : out std_ulogic                     -- interrupt
  );
end neorv32_twd;

architecture neorv32_twd_rtl of neorv32_twd is

  -- control and status register --
  constant ctrl_en_c           : natural :=  0; -- r/w: module enable (reset when zero)
  constant ctrl_clr_rx_c       : natural :=  1; -- -/w: clear RX FIFO (flag auto-clears)
  constant ctrl_clr_tx_c       : natural :=  2; -- -/w: clear TX FIFO (flag auto-clears)
  constant ctrl_fsel_c         : natural :=  3; -- r/w: input filter / sample clock select
  constant ctrl_addr0_c        : natural :=  4; -- r/w: device address, bit 0 (LSB)
  constant ctrl_addr6_c        : natural := 10; -- r/w: device address, bit 6 (MSB)
  constant ctrl_irq_rx_avail_c : natural := 11; -- r/w: IRQ if RX FIFO not empty
  constant ctrl_irq_rx_full_c  : natural := 12; -- r/w: IRQ if RX FIFO full
  constant ctrl_irq_tx_empty_c : natural := 13; -- r/w: IRQ if TX FIFO empty
  constant ctrl_irq_tx_nfull_c : natural := 14; -- r/w: IRQ if TX FIFO not full
  constant ctrl_irq_com_beg_c  : natural := 15; -- r/w: IRQ if begin of communication
  constant ctrl_irq_com_end_c  : natural := 16; -- r/w: IRQ if end of communication
  constant ctrl_rx_size0_c     : natural := 17; -- r/-: log2(RX_FIFO size), bit 0 (LSB)
  constant ctrl_rx_size3_c     : natural := 20; -- r/-: log2(RX_FIFO size), bit 3 (MSB)
  constant ctrl_tx_size0_c     : natural := 21; -- r/-: log2(TX_FIFO size), bit 0 (LSB)
  constant ctrl_tx_size3_c     : natural := 24; -- r/-: log2(TX_FIFO size), bit 3 (MSB)
  constant ctrl_rx_avail_c     : natural := 25; -- r/-: RX FIFO data available
  constant ctrl_rx_full_c      : natural := 26; -- r/-: RX FIFO full
  constant ctrl_tx_empty_c     : natural := 27; -- r/-: TX FIFO empty
  constant ctrl_tx_full_c      : natural := 28; -- r/-: TX FIFO full
  constant ctrl_com_beg_c      : natural := 29; -- r/c: communication started, clear by writing 1
  constant ctrl_com_end_c      : natural := 30; -- r/c: communication ended, clear by writing 1
  constant ctrl_com_c          : natural := 31; -- r/-: active communication

  -- configuration constants --
  constant rx_size_c : natural := index_size_f(TWD_RX_FIFO);
  constant tx_size_c : natural := index_size_f(TWD_TX_FIFO);

  -- access helpers --
  signal acc_we, acc_re : std_ulogic;

  -- control register --
  type ctrl_t is record
    enable       : std_ulogic;
    fsel         : std_ulogic;
    device_addr  : std_ulogic_vector(6 downto 0);
    irq_rx_avail : std_ulogic;
    irq_rx_full  : std_ulogic;
    irq_tx_empty : std_ulogic;
    irq_tx_nfull : std_ulogic;
    irq_com_beg  : std_ulogic;
    irq_com_end  : std_ulogic;
  end record;
  signal ctrl : ctrl_t;

  -- bus sampling logic --
  type smp_t is record
    clk_en   : std_ulogic; -- sample clock
    valid    : std_ulogic; -- valid sample
    sda_sreg : std_ulogic_vector(2 downto 0); -- SDA synchronizer
    scl_sreg : std_ulogic_vector(2 downto 0); -- SCL synchronizer
    sda      : std_ulogic; -- current SDA state
    scl      : std_ulogic; -- current SCL state
    scl_rise : std_ulogic; -- SCL rising edge
    scl_fall : std_ulogic; -- SCL falling edge
    start    : std_ulogic; -- start condition
    stop     : std_ulogic; -- stop condition
  end record;
  signal smp : smp_t;

  -- FIFO interface --
  type fifo_t is record
    clr   : std_ulogic; -- sync reset, high-active
    we    : std_ulogic; -- write enable
    re    : std_ulogic; -- read enable
    wdata : std_ulogic_vector(7 downto 0); -- write data
    rdata : std_ulogic_vector(7 downto 0); -- read data
    avail : std_ulogic; -- data available?
    free  : std_ulogic; -- free entry available?
  end record;
  signal rx_fifo, tx_fifo : fifo_t;

  -- bus engine --
  type state_t is (S_IDLE, S_INIT, S_ADDR, S_RESP, S_PREP, S_RTX, S_ACK);
  type engine_t is record
    state : state_t; -- FSM state
    cnt   : unsigned(3 downto 0); -- bit counter
    sreg  : std_ulogic_vector(7 downto 0); -- shift register
    cmd   : std_ulogic; -- 0 = write, 1 = read
    sda   : std_ulogic; -- SDA line drive
    rx_we : std_ulogic; -- write write-enable
    tx_re : std_ulogic; -- read read-enable
    com   : std_ulogic; -- active communication
  end record;
  signal engine : engine_t;

  -- communication state monitor --
  signal com_beg, com_end : std_ulogic;

begin

  -- Bus Access -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bus_rsp_o         <= rsp_terminate_c;
      ctrl.enable       <= '0';
      ctrl.fsel         <= '0';
      ctrl.device_addr  <= (others => '0');
      ctrl.irq_rx_avail <= '0';
      ctrl.irq_rx_full  <= '0';
      ctrl.irq_tx_empty <= '0';
      ctrl.irq_tx_nfull <= '0';
      ctrl.irq_com_beg  <= '0';
      ctrl.irq_com_end  <= '0';
    elsif rising_edge(clk_i) then
      -- bus defaults --
      bus_rsp_o.ack  <= bus_req_i.stb;
      bus_rsp_o.err  <= '0';
      bus_rsp_o.data <= (others => '0');
      -- write access --
      if (acc_we = '1') and (bus_req_i.addr(2) = '0') then -- control register
        ctrl.enable       <= bus_req_i.data(ctrl_en_c);
        ctrl.fsel         <= bus_req_i.data(ctrl_fsel_c);
        ctrl.device_addr  <= bus_req_i.data(ctrl_addr6_c downto ctrl_addr0_c);
        ctrl.irq_rx_avail <= bus_req_i.data(ctrl_irq_rx_avail_c);
        ctrl.irq_rx_full  <= bus_req_i.data(ctrl_irq_rx_full_c);
        ctrl.irq_tx_empty <= bus_req_i.data(ctrl_irq_tx_empty_c);
        ctrl.irq_tx_nfull <= bus_req_i.data(ctrl_irq_tx_nfull_c);
        ctrl.irq_com_beg  <= bus_req_i.data(ctrl_irq_com_beg_c);
        ctrl.irq_com_end  <= bus_req_i.data(ctrl_irq_com_end_c);
      end if;
      -- read access --
      if (acc_re = '1') then
        if (bus_req_i.addr(2) = '0') then -- control register
          bus_rsp_o.data(ctrl_en_c)                              <= ctrl.enable;
          bus_rsp_o.data(ctrl_fsel_c)                            <= ctrl.fsel;
          bus_rsp_o.data(ctrl_addr6_c downto ctrl_addr0_c)       <= ctrl.device_addr;
          bus_rsp_o.data(ctrl_irq_rx_avail_c)                    <= ctrl.irq_rx_avail;
          bus_rsp_o.data(ctrl_irq_rx_full_c)                     <= ctrl.irq_rx_full;
          bus_rsp_o.data(ctrl_irq_tx_empty_c)                    <= ctrl.irq_tx_empty;
          bus_rsp_o.data(ctrl_irq_tx_nfull_c)                    <= ctrl.irq_tx_nfull;
          bus_rsp_o.data(ctrl_irq_com_beg_c)                     <= ctrl.irq_com_beg;
          bus_rsp_o.data(ctrl_irq_com_end_c)                     <= ctrl.irq_com_end;
          bus_rsp_o.data(ctrl_rx_size3_c downto ctrl_rx_size0_c) <= std_ulogic_vector(to_unsigned(rx_size_c, 4));
          bus_rsp_o.data(ctrl_tx_size3_c downto ctrl_tx_size0_c) <= std_ulogic_vector(to_unsigned(tx_size_c, 4));
          bus_rsp_o.data(ctrl_rx_avail_c)                        <= rx_fifo.avail;
          bus_rsp_o.data(ctrl_rx_full_c)                         <= not rx_fifo.free;
          bus_rsp_o.data(ctrl_tx_empty_c)                        <= not tx_fifo.avail;
          bus_rsp_o.data(ctrl_tx_full_c)                         <= not tx_fifo.free;
          bus_rsp_o.data(ctrl_com_beg_c)                         <= com_beg;
          bus_rsp_o.data(ctrl_com_end_c)                         <= com_end;
          bus_rsp_o.data(ctrl_com_c)                             <= engine.com;
        else -- RX data FIFO
          bus_rsp_o.data(7 downto 0) <= rx_fifo.rdata;
        end if;
      end if;
    end if;
  end process bus_access;

  -- access helpers --
  acc_we <= '1' when (bus_req_i.stb = '1') and (bus_req_i.rw = '1') else '0';
  acc_re <= '1' when (bus_req_i.stb = '1') and (bus_req_i.rw = '0') else '0';


  -- Data FIFOs -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- TX --
  tx_fifo_inst: entity neorv32.neorv32_prim_fifo
  generic map (
    AWIDTH  => tx_size_c,
    DWIDTH  => 8,
    OUTGATE => false
  )
  port map (
    -- global control --
    clk_i   => clk_i,
    rstn_i  => rstn_i,
    clear_i => tx_fifo.clr,
    -- write port --
    wdata_i => tx_fifo.wdata,
    we_i    => tx_fifo.we,
    free_o  => tx_fifo.free,
    -- read port --
    re_i    => tx_fifo.re,
    rdata_o => tx_fifo.rdata,
    avail_o => tx_fifo.avail
  );

  tx_fifo.clr   <= (not ctrl.enable) or (acc_we and (not bus_req_i.addr(2)) and bus_req_i.data(ctrl_clr_tx_c));
  tx_fifo.wdata <= bus_req_i.data(7 downto 0);
  tx_fifo.we    <= acc_we and bus_req_i.addr(2);
  tx_fifo.re    <= engine.tx_re;

  -- RX --
  rx_fifo_inst: entity neorv32.neorv32_prim_fifo
  generic map (
    AWIDTH  => rx_size_c,
    DWIDTH  => 8,
    OUTGATE => false
  )
  port map (
    -- global control --
    clk_i   => clk_i,
    rstn_i  => rstn_i,
    clear_i => rx_fifo.clr,
    -- write port --
    wdata_i => rx_fifo.wdata,
    we_i    => rx_fifo.we,
    free_o  => rx_fifo.free,
    -- read port --
    re_i    => rx_fifo.re,
    rdata_o => rx_fifo.rdata,
    avail_o => rx_fifo.avail
  );

  rx_fifo.clr   <= (not ctrl.enable) or (acc_we and (not bus_req_i.addr(2)) and bus_req_i.data(ctrl_clr_rx_c));
  rx_fifo.wdata <= engine.sreg;
  rx_fifo.we    <= engine.rx_we;
  rx_fifo.re    <= acc_re and bus_req_i.addr(2);


  -- Interrupt Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_gen: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      irq_o <= '0';
    elsif rising_edge(clk_i) then
      irq_o <= ctrl.enable and (
               (ctrl.irq_rx_avail and      rx_fifo.avail)  or -- RX FIFO not empty
               (ctrl.irq_rx_full  and (not rx_fifo.free))  or -- RX FIFO full
               (ctrl.irq_tx_empty and (not tx_fifo.avail)) or -- TX FIFO empty
               (ctrl.irq_tx_nfull and      tx_fifo.free)   or -- TX FIFO not full
               (ctrl.irq_com_beg  and      com_beg)        or -- begin of communication
               (ctrl.irq_com_end  and      com_end));         -- end of communication
    end if;
  end process irq_gen;


  -- Bus Sampling Logic ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  synchronizer: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      smp.valid    <= '0';
      smp.sda_sreg <= (others => '0');
      smp.scl_sreg <= (others => '0');
    elsif rising_edge(clk_i) then
      -- input register --
      smp.sda_sreg(0) <= to_stdulogic(to_bit(twd_sda_i)); -- "to_bit" to avoid hardware-vs-simulation mismatch
      smp.scl_sreg(0) <= to_stdulogic(to_bit(twd_scl_i));
      -- sample register --
      smp.valid <= ctrl.enable and smp.clk_en; -- valid sample
      if (smp.clk_en = '1') then
        if (ctrl.enable = '1') then
          smp.sda_sreg(2 downto 1) <= smp.sda_sreg(1 downto 0);
          smp.scl_sreg(2 downto 1) <= smp.scl_sreg(1 downto 0);
        else
          smp.sda_sreg(2 downto 1) <= (others => '1');
          smp.scl_sreg(2 downto 1) <= (others => '1');
        end if;
      end if;
    end if;
  end process synchronizer;

  -- sample clock for input "filtering" --
  smp.clk_en <= clkgen_i(clk_div64_c) when (ctrl.fsel = '1') else clkgen_i(clk_div8_c);

  -- bus event detectors (event signals are "single-shot") --
  smp.sda      <= smp.sda_sreg(1);
  smp.scl      <= smp.sda_sreg(1);
  smp.scl_rise <= smp.valid and (not smp.scl_sreg(2)) and (    smp.scl_sreg(1));
  smp.scl_fall <= smp.valid and (    smp.scl_sreg(2)) and (not smp.scl_sreg(1));
  smp.start    <= smp.valid and smp.scl_sreg(2) and smp.scl_sreg(1) and (    smp.sda_sreg(2)) and (not smp.sda_sreg(1));
  smp.stop     <= smp.valid and smp.scl_sreg(2) and smp.scl_sreg(1) and (not smp.sda_sreg(2)) and (    smp.sda_sreg(1));


  -- Bus Engine -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_engine: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      engine.state <= S_IDLE;
      engine.cnt   <= (others => '0');
      engine.sreg  <= (others => '1');
      engine.cmd   <= '0';
      engine.sda   <= '1';
      engine.rx_we <= '0';
      engine.tx_re <= '0';
      engine.com   <= '0';
    elsif rising_edge(clk_i) then
      engine.rx_we <= '0';
      engine.tx_re <= '0';
      case engine.state is

        when S_IDLE => -- idle, wait for start condition
        -- ------------------------------------------------------------
          engine.sda <= '1'; -- idle
          engine.com <= '0'; -- no active communication yet/anymore
          if (ctrl.enable = '1') and (smp.start = '1') then
            engine.state <= S_INIT;
          end if;

        when S_INIT => -- (re-)initialize
        -- ------------------------------------------------------------
          engine.cnt   <= (others => '0');
          engine.state <= S_ADDR;

        when S_ADDR => -- sample address + R/W bit and check if address match and data is available
        -- ------------------------------------------------------------
          if (ctrl.enable = '0') or (smp.stop = '1') then -- disabled or stop-condition received?
            engine.state <= S_IDLE;
          elsif (smp.start = '1') then -- start-condition received?
            engine.state <= S_INIT;
          elsif (engine.cnt(3) = '1') and (smp.scl_fall = '1') then -- 8 bits received?
            if (ctrl.device_addr = engine.sreg(7 downto 1)) then -- address match?
              engine.state <= S_RESP;
            else -- no access, go back to idle
              engine.state <= S_IDLE;
            end if;
          end if;
          -- sample bus on rising edge --
          if (smp.scl_rise = '1') then
            engine.sreg <= engine.sreg(6 downto 0) & smp.sda;
            engine.cnt  <= engine.cnt + 1;
          end if;

        when S_RESP => -- send device address-match ACK
        -- ------------------------------------------------------------
          engine.sda <= '0'; -- ACK
          engine.com <= '1'; -- communication started
          engine.cmd <= engine.sreg(0); -- READ/WRITE operation request
          if (ctrl.enable = '0') then -- disabled?
            engine.state <= S_IDLE;
          elsif (smp.scl_fall = '1') then -- end of bit slot
            engine.state <= S_PREP;
          end if;

        when S_PREP => -- prepare data transmission
        -- ------------------------------------------------------------
          if (tx_fifo.avail = '1') and (engine.cmd = '1') then -- data available for read?
            engine.sreg <= tx_fifo.rdata;
            engine.sda  <= tx_fifo.rdata(7);
          else -- no TX data available or write operation
            engine.sreg <= (others => '1');
            engine.sda  <= '1';
          end if;
          engine.cnt   <= (others => '0');
          engine.state <= S_RTX;

        when S_RTX => -- receive/transmit 8 data bits
        -- ------------------------------------------------------------
          if (ctrl.enable = '0') or (smp.stop = '1') then -- disabled or stop-condition
            engine.state <= S_IDLE;
          elsif (smp.start = '1') then -- start-condition
            engine.state <= S_INIT;
          elsif (engine.cnt(3) = '1') and (smp.scl_fall = '1') then -- 8 bits received?
            engine.state <= S_ACK;
          end if;
          -- sample bus on rising edge --
          if (smp.scl_rise = '1') then
            engine.sreg <= engine.sreg(6 downto 0) & smp.sda;
            engine.cnt  <= engine.cnt + 1;
          end if;
          -- update bus at falling edge --
          if (smp.scl_fall = '1') then -- end of bit slot
            engine.sda <= engine.sreg(7);
          end if;

        when S_ACK => -- receive/transmit ACK/NACK
        -- ------------------------------------------------------------
          if (ctrl.enable = '0') or (smp.stop = '1') then -- disabled or stop-condition
            engine.state <= S_IDLE;
          else
            if (engine.cmd = '0') then -- WRITE operation
              engine.sda   <= not rx_fifo.free; -- ACK if RX FIFO is not full; NACK if RX FIFO is full
              engine.rx_we <= smp.scl_fall; -- push to RX FIFO at end of bit slot (if RX FIFO not full)
            else -- READ operation
              engine.sda   <= '1'; -- keep high-Z so we can sample the ACK/NACK from the host
              engine.tx_re <= smp.scl_rise and (not smp.sda); -- pop from RX FIFO if ACK at sample point
            end if;
            if (smp.scl_fall = '1') then -- end of bit slot
              engine.state <= S_PREP;
            end if;
          end if;

        when others => -- undefined
        -- ------------------------------------------------------------
          engine.state <= S_IDLE;

      end case;
    end if;
  end process bus_engine;

  -- TWI lines --
  twd_scl_o <= '1'; -- used as input only
  twd_sda_o <= engine.sda;


  -- Communication State Monitor ------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  com_state_monitor: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      com_beg <= '0';
      com_end <= '0';
    elsif rising_edge(clk_i) then
      if (ctrl.enable = '0') then
        com_beg <= '0';
        com_end <= '0';
      else
        -- begin of communication --
        if (engine.state = S_RESP) and (smp.scl_fall = '1') then
          com_beg <= '1';
        elsif (acc_we = '1') and (bus_req_i.addr(2) = '0') and (bus_req_i.data(ctrl_com_beg_c) = '1') then
          com_beg <= '0';
        end if;
        -- end of communication --
        if (engine.state = S_IDLE) and (engine.com = '1') then
          com_end <= '1';
        elsif (acc_we = '1') and (bus_req_i.addr(2) = '0') and (bus_req_i.data(ctrl_com_end_c) = '1') then
          com_end <= '0';
        end if;
      end if;
    end if;
  end process com_state_monitor;

end neorv32_twd_rtl;
