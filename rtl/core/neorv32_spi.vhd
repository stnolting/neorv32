-- #################################################################################################
-- # << NEORV32 - Serial Peripheral Interface Controller (SPI) >>                                  #
-- # ********************************************************************************************* #
-- # Frame format: 8/16/24/32-bit receive/transmit data, always MSB first, 2 clock modes,          #
-- # 8 pre-scaled clocks (derived from system clock), 8 dedicated chip-select lines (low-active).  #
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

entity neorv32_spi is
  generic (
    IO_SPI_FIFO : natural -- SPI RTX fifo depth, has to be zero or a power of two
  );
  port (
    -- host access --
    clk_i       : in  std_ulogic; -- global clock line
    rstn_i      : in  std_ulogic; -- global reset line, low-active
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
    spi_sck_o   : out std_ulogic; -- SPI serial clock
    spi_sdo_o   : out std_ulogic; -- controller data out, peripheral data in
    spi_sdi_i   : in  std_ulogic; -- controller data in, peripheral data out
    spi_csn_o   : out std_ulogic_vector(07 downto 0); -- SPI CS
    -- interrupt --
    irq_o       : out std_ulogic -- transmission done interrupt
  );
end neorv32_spi;

architecture neorv32_spi_rtl of neorv32_spi is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(spi_size_c); -- low address boundary bit

  -- control register --
  constant ctrl_cs0_c        : natural :=  0; -- r/w: spi CS 0
  constant ctrl_cs1_c        : natural :=  1; -- r/w: spi CS 1
  constant ctrl_cs2_c        : natural :=  2; -- r/w: spi CS 2
  constant ctrl_cs3_c        : natural :=  3; -- r/w: spi CS 3
  constant ctrl_cs4_c        : natural :=  4; -- r/w: spi CS 4
  constant ctrl_cs5_c        : natural :=  5; -- r/w: spi CS 5
  constant ctrl_cs6_c        : natural :=  6; -- r/w: spi CS 6
  constant ctrl_cs7_c        : natural :=  7; -- r/w: spi CS 7
  constant ctrl_en_c         : natural :=  8; -- r/w: spi enable
  constant ctrl_cpha_c       : natural :=  9; -- r/w: spi clock phase
  constant ctrl_prsc0_c      : natural := 10; -- r/w: spi prescaler select bit 0
  constant ctrl_prsc1_c      : natural := 11; -- r/w: spi prescaler select bit 1
  constant ctrl_prsc2_c      : natural := 12; -- r/w: spi prescaler select bit 2
  constant ctrl_size0_c      : natural := 13; -- r/w: data size lsb (00:  8-bit, 01: 16-bit)
  constant ctrl_size1_c      : natural := 14; -- r/w: data size msb (10: 24-bit, 11: 32-bit)
  constant ctrl_cpol_c       : natural := 15; -- r/w: spi clock polarity
  constant ctrl_highspeed_c  : natural := 16; -- r/w: spi high-speed mode enable (ignoring ctrl_prsc)
  constant ctrl_irq0_c       : natural := 17; -- r/w: interrupt mode lsb (0-: PHY going idle)
  constant ctrl_irq1_c       : natural := 18; -- r/w: interrupt mode msb (10: TX fifo less than half full, 11: TX fifo empty)
  --
  constant ctrl_fifo_size0_c : natural := 19; -- r/-: log2(FIFO size), bit 0 (lsb)
  constant ctrl_fifo_size1_c : natural := 20; -- r/-: log2(FIFO size), bit 1
  constant ctrl_fifo_size2_c : natural := 21; -- r/-: log2(FIFO size), bit 2
  constant ctrl_fifo_size3_c : natural := 22; -- r/-: log2(FIFO size), bit 3 (msb)
  --
  constant ctrl_rx_avail_c   : natural := 27; -- r/-: RX FIFO data available (RX FIFO not empty)
  constant ctrl_tx_empty_c   : natural := 28; -- r/-: TX FIFO empty
  constant ctrl_tx_half_c    : natural := 29; -- r/-: TX FIFO at least half full
  constant ctrl_tx_full_c    : natural := 30; -- r/-: TX FIFO full
  constant ctrl_busy_c       : natural := 31; -- r/-: spi PHY busy or TX FIFO not empty yet
  --
  signal ctrl : std_ulogic_vector(18 downto 0);

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- clock generator --
  signal spi_clk_en : std_ulogic;

  -- interrupt generator --
  signal irq_gen : std_ulogic_vector(1 downto 0);

  -- spi transceiver --
  type rtx_engine_t is record
    state    : std_ulogic_vector(02 downto 0);
    busy     : std_ulogic;
    start    : std_ulogic;
    sreg     : std_ulogic_vector(31 downto 0);
    bitcnt   : std_ulogic_vector(05 downto 0);
    bytecnt  : std_ulogic_vector(02 downto 0);
    sdi_sync : std_ulogic;
    done     : std_ulogic;
  end record;
  signal rtx_engine : rtx_engine_t;

  -- RX/TX FIFO interface --
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
  assert not ((is_power_of_two_f(IO_SPI_FIFO) = false) and (IO_SPI_FIFO /= 0))
  report "NEORV32 PROCESSOR CONFIG ERROR: SPI <IO_SPI_FIFO> has to be a power of two." severity error;
  assert not (IO_SPI_FIFO > 2**15)
  report "NEORV32 PROCESSOR CONFIG ERROR: SPI <IO_SPI_FIFO> has to be 1..32768." severity error;


  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = spi_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= spi_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
  wren   <= acc_en and wren_i;
  rden   <= acc_en and rden_i;


  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rw_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl   <= (others => '0');
      ack_o  <= '-';
      data_o <= (others => '-');
    elsif rising_edge(clk_i) then
      -- bus access acknowledge --
      ack_o <= rden or wren;

      -- write access --
      if (wren = '1') then
        if (addr = spi_ctrl_addr_c) then -- control register
          ctrl(ctrl_cs7_c downto ctrl_cs0_c)     <= data_i(ctrl_cs7_c downto ctrl_cs0_c);
          ctrl(ctrl_en_c)                        <= data_i(ctrl_en_c);
          ctrl(ctrl_cpha_c)                      <= data_i(ctrl_cpha_c);
          ctrl(ctrl_prsc2_c downto ctrl_prsc0_c) <= data_i(ctrl_prsc2_c downto ctrl_prsc0_c);
          ctrl(ctrl_size1_c downto ctrl_size0_c) <= data_i(ctrl_size1_c downto ctrl_size0_c);
          ctrl(ctrl_cpol_c)                      <= data_i(ctrl_cpol_c);
          ctrl(ctrl_highspeed_c)                 <= data_i(ctrl_highspeed_c);
          if (IO_SPI_FIFO > 0) then -- FIFO implemented: all IRQ options available
            ctrl(ctrl_irq1_c downto ctrl_irq0_c) <= data_i(ctrl_irq1_c downto ctrl_irq0_c);
          else -- fall back: only the busy state of the SPI PHY can be used as IRQ source if no FIFO is implemented
            ctrl(ctrl_irq1_c downto ctrl_irq0_c) <= "00";
          end if;
        end if;
      end if;

      -- read access --
      data_o <= (others => '0');
      if (rden = '1') then
        if (addr = spi_ctrl_addr_c) then -- control register
          data_o(ctrl_cs7_c downto ctrl_cs0_c)     <= ctrl(ctrl_cs7_c downto ctrl_cs0_c);
          data_o(ctrl_en_c)                        <= ctrl(ctrl_en_c);
          data_o(ctrl_cpha_c)                      <= ctrl(ctrl_cpha_c);
          data_o(ctrl_prsc2_c downto ctrl_prsc0_c) <= ctrl(ctrl_prsc2_c downto ctrl_prsc0_c);
          data_o(ctrl_size1_c downto ctrl_size0_c) <= ctrl(ctrl_size1_c downto ctrl_size0_c);
          data_o(ctrl_cpol_c)                      <= ctrl(ctrl_cpol_c);
          data_o(ctrl_highspeed_c)                 <= ctrl(ctrl_highspeed_c);
          data_o(ctrl_irq1_c downto ctrl_irq0_c)   <= ctrl(ctrl_irq1_c downto ctrl_irq0_c);
          --
          data_o(ctrl_fifo_size3_c downto ctrl_fifo_size0_c) <= std_ulogic_vector(to_unsigned(index_size_f(IO_SPI_FIFO), 4));
          --
          if (IO_SPI_FIFO > 0) then
            data_o(ctrl_rx_avail_c) <= rx_fifo.avail;
            data_o(ctrl_tx_empty_c) <= not tx_fifo.avail;
            data_o(ctrl_tx_half_c)  <= tx_fifo.half;
            data_o(ctrl_tx_full_c)  <= not tx_fifo.free;
            data_o(ctrl_busy_c)     <= rtx_engine.busy or tx_fifo.avail; -- PHY busy or TX FIFO not empty yet
          else
            data_o(ctrl_rx_avail_c) <= '0';
            data_o(ctrl_tx_empty_c) <= '0';
            data_o(ctrl_tx_half_c)  <= '0';
            data_o(ctrl_tx_full_c)  <= '0';
            data_o(ctrl_busy_c)     <= rtx_engine.busy;
          end if;
        else -- data register (spi_rtx_addr_c)
          data_o <= rx_fifo.rdata;
        end if;
      end if;
    end if;
  end process rw_access;


  -- RTX FIFO ("Ring Buffer") ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rxt_fifo_gen:
  if (IO_SPI_FIFO > 0) generate

    -- TX FIFO ------------------------------------------------
    tx_fifo_inst: neorv32_fifo
    generic map (
      FIFO_DEPTH => IO_SPI_FIFO, -- number of fifo entries; has to be a power of two; min 1
      FIFO_WIDTH => 32,          -- size of data elements in fifo (32-bit only for simulation)
      FIFO_RSYNC => false,       -- async read
      FIFO_SAFE  => true         -- safe access
    )
    port map (
      -- control --
      clk_i   => clk_i,         -- clock, rising edge
      rstn_i  => '1',           -- async reset, low-active
      clear_i => tx_fifo.clear, -- sync reset, high-active
      half_o  => tx_fifo.half,  -- FIFO at least half-full
      -- write port --
      wdata_i => tx_fifo.wdata, -- write data
      we_i    => tx_fifo.we,    -- write enable
      free_o  => tx_fifo.free,  -- at least one entry is free when set
      -- read port --
      re_i    => tx_fifo.re,    -- read enable
      rdata_o => tx_fifo.rdata, -- read data
      avail_o => tx_fifo.avail  -- data available when set
    );

    -- reset --
    tx_fifo.clear <= not ctrl(ctrl_en_c);

    -- write access --
    tx_fifo.we    <= '1' when (wren = '1') and (addr = spi_rtx_addr_c) else '0';
    tx_fifo.wdata <= data_i;

    -- read access --
    tx_fifo.re       <= '1' when (rtx_engine.state = "100") and (tx_fifo.avail = '1') else '0';
    rtx_engine.start <= tx_fifo.re;


    -- RX FIFO ------------------------------------------------
    rx_fifo_inst: neorv32_fifo
    generic map (
      FIFO_DEPTH => IO_SPI_FIFO, -- number of fifo entries; has to be a power of two; min 1
      FIFO_WIDTH => 32,          -- size of data elements in fifo (32-bit only for simulation)
      FIFO_RSYNC => false,       -- async read
      FIFO_SAFE  => true         -- safe access
    )
    port map (
      -- control --
      clk_i   => clk_i,         -- clock, rising edge
      rstn_i  => '1',           -- async reset, low-active
      clear_i => rx_fifo.clear, -- sync reset, high-active
      half_o  => rx_fifo.half,  -- FIFO at least half-full
      -- write port --
      wdata_i => rx_fifo.wdata, -- write data
      we_i    => rx_fifo.we,    -- write enable
      free_o  => rx_fifo.free,  -- at least one entry is free when set
      -- read port --
      re_i    => rx_fifo.re,    -- read enable
      rdata_o => rx_fifo.rdata, -- read data
      avail_o => rx_fifo.avail  -- data available when set
    );

    -- reset --
    rx_fifo.clear <= not ctrl(ctrl_en_c);

    -- write access --
    rx_fifo.wdata <= rtx_engine.sreg;
    rx_fifo.we    <= rtx_engine.done;

    -- read access --
    rx_fifo.re <= '1' when (rden = '1') and (addr = spi_rtx_addr_c) else '0';

  end generate;


  -- no FIFO at all -----------------------------------------
  rtx_fifo_terminate:
  if (IO_SPI_FIFO = 0) generate
    -- TX --
    tx_fifo.clear <= '0';
    tx_fifo.half  <= '0';
    tx_fifo.wdata <= (others => '0');
    tx_fifo.we    <= '0';
    tx_fifo.free  <= '0';
    tx_fifo.re    <= '0';
    tx_fifo.rdata <= data_i;
    tx_fifo.avail <= '0';
    -- RX --
    rx_fifo.clear <= '0';
    rx_fifo.half  <= '0';
    rx_fifo.wdata <= (others => '0');
    rx_fifo.we    <= '0';
    rx_fifo.free  <= '0';
    rx_fifo.re    <= '0';
    rx_fifo.rdata <= rtx_engine.sreg;
    rx_fifo.avail <= '0';
    -- SPI PHY trigger --
    rtx_engine.start <= '1' when (wren = '1') and (addr = spi_rtx_addr_c) else '0'; -- trigger new SPI transmission
  end generate;


  -- SPI Transceiver ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  spi_rtx_unit: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- defaults --
      rtx_engine.done <= '0';

      -- serial engine --
      rtx_engine.state(2) <= ctrl(ctrl_en_c);
      case rtx_engine.state is

        when "100" => -- enabled but idle, waiting for new transmission trigger
        -- ------------------------------------------------------------
          spi_sck_o <= ctrl(ctrl_cpol_c);
          rtx_engine.bitcnt <= (others => '0');
          if (rtx_engine.start = '1') then -- trigger new transmission
            rtx_engine.sreg <= tx_fifo.rdata;
            rtx_engine.state(1 downto 0) <= "01";
          end if;

        when "101" => -- start with next new clock pulse
        -- ------------------------------------------------------------
          if (spi_clk_en = '1') then
            if (ctrl(ctrl_cpha_c) = '1') then -- clock phase shift
              spi_sck_o <= not ctrl(ctrl_cpol_c);
            end if;
            rtx_engine.state(1 downto 0) <= "10";
          end if;

        when "110" => -- first half of bit transmission
        -- ------------------------------------------------------------
          if (spi_clk_en = '1') then
            spi_sck_o <= not (ctrl(ctrl_cpha_c) xor ctrl(ctrl_cpol_c));
            rtx_engine.sdi_sync <= spi_sdi_i; -- sample data input
            rtx_engine.bitcnt <= std_ulogic_vector(unsigned(rtx_engine.bitcnt) + 1);
            rtx_engine.state(1 downto 0) <= "11";
          end if;

        when "111" => -- second half of bit transmission
        -- ------------------------------------------------------------
          if (spi_clk_en = '1') then
            rtx_engine.sreg <= rtx_engine.sreg(30 downto 0) & rtx_engine.sdi_sync; -- shift and set output
            if (rtx_engine.bitcnt(5 downto 3) = rtx_engine.bytecnt) then -- all bits transferred?
              spi_sck_o <= ctrl(ctrl_cpol_c);
              rtx_engine.done              <= '1'; -- done!
              rtx_engine.state(1 downto 0) <= "00"; -- transmission done
            else
              spi_sck_o <= ctrl(ctrl_cpha_c) xor ctrl(ctrl_cpol_c);
              rtx_engine.state(1 downto 0) <= "10";
            end if;
          end if;

        when others => -- "0--": SPI deactivated
        -- ------------------------------------------------------------
          spi_sck_o <= ctrl(ctrl_cpol_c);
          rtx_engine.sreg <= (others => '0');
          rtx_engine.state(1 downto 0) <= "00";

      end case;
    end if;
  end process spi_rtx_unit;

  -- clock generator --
  clkgen_en_o <= ctrl(ctrl_en_c); -- clock generator enable
  spi_clk_en  <= clkgen_i(to_integer(unsigned(ctrl(ctrl_prsc2_c downto ctrl_prsc0_c)))) or ctrl(ctrl_highspeed_c); -- clock select

  -- PHY busy flag --
  rtx_engine.busy <= '0' when (rtx_engine.state(1 downto 0) = "00") else '1';

  -- transmission size --
  data_size: process(ctrl, rtx_engine)
  begin
    case ctrl(ctrl_size1_c downto ctrl_size0_c) is
      when "00"   => rtx_engine.bytecnt <= "001"; spi_sdo_o <= rtx_engine.sreg(07); -- 8-bit mode
      when "01"   => rtx_engine.bytecnt <= "010"; spi_sdo_o <= rtx_engine.sreg(15); -- 16-bit mode
      when "10"   => rtx_engine.bytecnt <= "011"; spi_sdo_o <= rtx_engine.sreg(23); -- 24-bit mode
      when others => rtx_engine.bytecnt <= "100"; spi_sdo_o <= rtx_engine.sreg(31); -- 32-bit mode
    end case;
  end process data_size;

  -- direct chip-select; low-active --  
  spi_csn_o(7 downto 0) <= not ctrl(ctrl_cs7_c downto ctrl_cs0_c);


  -- Interrupt Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_generator: process(clk_i)
  begin
    if rising_edge(clk_i) then
      case ctrl(ctrl_irq1_c downto ctrl_irq0_c) is
        when "10"   => irq_gen <= irq_gen(0) & (not tx_fifo.half);  -- TX FIFO _becomes_ less than half full
        when "11"   => irq_gen <= irq_gen(0) & (not tx_fifo.avail); -- TX FIFO _becomes_ empty
        when others => irq_gen <= irq_gen(0) & rtx_engine.done;     -- current SPI transmission done (default)
      end case;
    end if;
  end process irq_generator;

  -- CPU interrupt --
  irq_o <= '1' when (ctrl(ctrl_en_c) = '1') and (irq_gen = "01") else '0'; -- rising edge detector


end neorv32_spi_rtl;
