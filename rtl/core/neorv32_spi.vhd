-- #################################################################################################
-- # << NEORV32 - Serial Peripheral Interface Controller (SPI) >>                                  #
-- # ********************************************************************************************* #
-- # Frame format: 8/16/24/32-bit receive/transmit data, always MSB first, 2 clock modes, 8 clock  #
-- # pre-scalers (derived from system clock) + 4-bit clock divider for bus clock configuration, 8  #
-- # dedicated chip-select lines (low-active).                                                     #
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
    rstn_i      : in  std_ulogic; -- global reset line, low-active, async
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
  constant ctrl_en_c         : natural :=  0; -- r/w: spi enable
  constant ctrl_cpha_c       : natural :=  1; -- r/w: spi clock phase
  constant ctrl_cpol_c       : natural :=  2; -- r/w: spi clock polarity
  constant ctrl_size0_c      : natural :=  3; -- r/w: data size lsb (00:  8-bit, 01: 16-bit)
  constant ctrl_size1_c      : natural :=  4; -- r/w: data size msb (10: 24-bit, 11: 32-bit)
  constant ctrl_cs_sel0_c    : natural :=  5; -- r/w: spi CS select bit 0
  constant ctrl_cs_sel1_c    : natural :=  6; -- r/w: spi CS select bit 1
  constant ctrl_cs_sel2_c    : natural :=  7; -- r/w: spi CS select bit 2
  constant ctrl_cs_en_c      : natural :=  8; -- r/w: enable selected CS line (set bit -> clear line)
  constant ctrl_prsc0_c      : natural :=  9; -- r/w: spi prescaler select bit 0
  constant ctrl_prsc1_c      : natural := 10; -- r/w: spi prescaler select bit 1
  constant ctrl_prsc2_c      : natural := 11; -- r/w: spi prescaler select bit 2
  constant ctrl_cdiv0_c      : natural := 12; -- r/w: clock divider bit 0
  constant ctrl_cdiv1_c      : natural := 13; -- r/w: clock divider bit 1
  constant ctrl_cdiv2_c      : natural := 14; -- r/w: clock divider bit 2
  constant ctrl_cdiv3_c      : natural := 15; -- r/w: clock divider bit 3
  constant ctrl_irq0_c       : natural := 16; -- r/w: interrupt mode lsb (0-: PHY going idle)
  constant ctrl_irq1_c       : natural := 17; -- r/w: interrupt mode msb (10: TX fifo less than half full, 11: TX fifo empty)
  --
  constant ctrl_fifo_size0_c : natural := 23; -- r/-: log2(FIFO size), bit 0 (lsb)
  constant ctrl_fifo_size1_c : natural := 24; -- r/-: log2(FIFO size), bit 1
  constant ctrl_fifo_size2_c : natural := 25; -- r/-: log2(FIFO size), bit 2
  constant ctrl_fifo_size3_c : natural := 26; -- r/-: log2(FIFO size), bit 3 (msb)
  constant ctrl_rx_avail_c   : natural := 27; -- r/-: RX FIFO data available (RX FIFO not empty)
  constant ctrl_tx_empty_c   : natural := 28; -- r/-: TX FIFO empty
  constant ctrl_tx_half_c    : natural := 29; -- r/-: TX FIFO at least half full
  constant ctrl_tx_full_c    : natural := 30; -- r/-: TX FIFO full
  constant ctrl_busy_c       : natural := 31; -- r/-: spi PHY busy or TX FIFO not empty yet

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- control register - see bit definitions above --
  type ctrl_t is record
    enable   : std_ulogic;
    cpha     : std_ulogic;
    cpol     : std_ulogic;
    rtx_size : std_ulogic_vector(1 downto 0);
    cs_sel   : std_ulogic_vector(2 downto 0);
    cs_en    : std_ulogic;
    prsc     : std_ulogic_vector(2 downto 0);
    cdiv     : std_ulogic_vector(3 downto 0);
    irq_mode : std_ulogic_vector(1 downto 0);
  end record;
  signal ctrl : ctrl_t;

  -- clock generator --
  signal cdiv_cnt   : std_ulogic_vector(3 downto 0); -- clock divider
  signal spi_clk_en : std_ulogic; -- actual SPI "clock"

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


  -- Write Access ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl.enable    <= '0';
      ctrl.cpha      <= '0';
      ctrl.cpol      <= '0';
      ctrl.rtx_size  <= (others => '0');
      ctrl.cs_sel    <= (others => '0');
      ctrl.cs_en     <= '0';
      ctrl.prsc      <= (others => '0');
      ctrl.cdiv      <= (others => '0');
      ctrl.irq_mode  <= (others => '0');
    elsif rising_edge(clk_i) then
      if (wren = '1') then
        if (addr = spi_ctrl_addr_c) then -- control register
          ctrl.enable    <= data_i(ctrl_en_c);
          ctrl.cpha      <= data_i(ctrl_cpha_c);
          ctrl.cpol      <= data_i(ctrl_cpol_c);
          ctrl.rtx_size  <= data_i(ctrl_size1_c downto ctrl_size0_c);
          ctrl.cs_sel    <= data_i(ctrl_cs_sel2_c downto ctrl_cs_sel0_c);
          ctrl.cs_en     <= data_i(ctrl_cs_en_c);
          ctrl.prsc      <= data_i(ctrl_prsc2_c downto ctrl_prsc0_c);
          ctrl.cdiv      <= data_i(ctrl_cdiv3_c downto ctrl_cdiv0_c);
          if (IO_SPI_FIFO > 0) then -- FIFO implemented: all IRQ options available
            ctrl.irq_mode <= data_i(ctrl_irq1_c downto ctrl_irq0_c);
          else -- fall back: only the busy state of the SPI PHY can be used as IRQ source if no FIFO is implemented
            ctrl.irq_mode <= "00";
          end if;
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
        if (addr = spi_ctrl_addr_c) then -- control register
          data_o(ctrl_en_c)                            <= ctrl.enable;
          data_o(ctrl_cpha_c)                          <= ctrl.cpha;
          data_o(ctrl_cpol_c)                          <= ctrl.cpol;
          data_o(ctrl_size1_c downto ctrl_size0_c)     <= ctrl.rtx_size;
          data_o(ctrl_cs_sel2_c downto ctrl_cs_sel0_c) <= ctrl.cs_sel;
          data_o(ctrl_cs_en_c)                         <= ctrl.cs_en;
          data_o(ctrl_prsc2_c downto ctrl_prsc0_c)     <= ctrl.prsc;
          data_o(ctrl_cdiv3_c downto ctrl_cdiv0_c)     <= ctrl.cdiv;
          data_o(ctrl_irq1_c downto ctrl_irq0_c)       <= ctrl.irq_mode;
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
  end process read_access;


  -- Direct Chip-Select (low-active) --------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  direct_cs: process(ctrl)
  begin
    spi_csn_o <= (others => '1'); -- default: all disabled
    if (ctrl.cs_en = '1') and (ctrl.enable = '1') then
      spi_csn_o(to_integer(unsigned(ctrl.cs_sel))) <= '0';
    end if;
  end process direct_cs;


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
      FIFO_SAFE  => true,        -- safe access
      FIFO_GATE  => false        -- no output gate required
    )
    port map (
      -- control --
      clk_i   => clk_i,         -- clock, rising edge
      rstn_i  => rstn_i,        -- async reset, low-active
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
    tx_fifo.clear <= not ctrl.enable;

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
      FIFO_SAFE  => true,        -- safe access
      FIFO_GATE  => false        -- no output gate required
    )
    port map (
      -- control --
      clk_i   => clk_i,         -- clock, rising edge
      rstn_i  => rstn_i,        -- async reset, low-active
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
    rx_fifo.clear <= not ctrl.enable;

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


  -- Clock Generator ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  clock_generator: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (ctrl.enable = '0') then -- reset/disabled
        spi_clk_en <= '0';
        cdiv_cnt   <= (others => '0');
      else
        spi_clk_en <= '0'; -- default
        if (clkgen_i(to_integer(unsigned(ctrl.prsc))) = '1') then -- pre-scaled clock
          if (cdiv_cnt = ctrl.cdiv) then -- clock divider for fine-tuning
            spi_clk_en <= '1';
            cdiv_cnt   <= (others => '0');
          else
            cdiv_cnt <= std_ulogic_vector(unsigned(cdiv_cnt) + 1);
          end if;
        end if;
      end if;
    end if;
  end process clock_generator;

  -- clock generator enable --
  clkgen_en_o <= ctrl.enable;


  -- SPI Transceiver ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  spi_rtx_unit: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- defaults --
      rtx_engine.done <= '0';

      -- serial engine --
      rtx_engine.state(2) <= ctrl.enable;
      case rtx_engine.state is

        when "100" => -- enabled but idle, waiting for new transmission trigger
        -- ------------------------------------------------------------
          spi_sck_o <= ctrl.cpol;
          rtx_engine.bitcnt <= (others => '0');
          if (rtx_engine.start = '1') then -- trigger new transmission
            rtx_engine.sreg <= tx_fifo.rdata;
            rtx_engine.state(1 downto 0) <= "01";
          end if;

        when "101" => -- start with next new clock pulse
        -- ------------------------------------------------------------
          if (spi_clk_en = '1') then
            if (ctrl.cpha = '1') then -- clock phase shift
              spi_sck_o <= not ctrl.cpol;
            end if;
            rtx_engine.state(1 downto 0) <= "10";
          end if;

        when "110" => -- first half of bit transmission
        -- ------------------------------------------------------------
          if (spi_clk_en = '1') then
            spi_sck_o                    <= not (ctrl.cpha xor ctrl.cpol);
            rtx_engine.sdi_sync          <= spi_sdi_i; -- sample data input
            rtx_engine.bitcnt            <= std_ulogic_vector(unsigned(rtx_engine.bitcnt) + 1);
            rtx_engine.state(1 downto 0) <= "11";
          end if;

        when "111" => -- second half of bit transmission
        -- ------------------------------------------------------------
          if (spi_clk_en = '1') then
            rtx_engine.sreg <= rtx_engine.sreg(30 downto 0) & rtx_engine.sdi_sync; -- shift and set output
            if (rtx_engine.bitcnt(5 downto 3) = rtx_engine.bytecnt) then -- all bits transferred?
              spi_sck_o                    <= ctrl.cpol;
              rtx_engine.done              <= '1'; -- done!
              rtx_engine.state(1 downto 0) <= "00"; -- transmission done
            else
              spi_sck_o                    <= ctrl.cpha xor ctrl.cpol;
              rtx_engine.state(1 downto 0) <= "10";
            end if;
          end if;

        when others => -- "0--": SPI deactivated
        -- ------------------------------------------------------------
          spi_sck_o                    <= ctrl.cpol;
          rtx_engine.sreg              <= (others => '0');
          rtx_engine.state(1 downto 0) <= "00";

      end case;
    end if;
  end process spi_rtx_unit;

  -- PHY busy flag --
  rtx_engine.busy <= '0' when (rtx_engine.state(1 downto 0) = "00") else '1';

  -- transmission size --
  data_size: process(ctrl, rtx_engine)
  begin
    case ctrl.rtx_size is
      when "00"   => rtx_engine.bytecnt <= "001"; spi_sdo_o <= rtx_engine.sreg(07); -- 8-bit mode
      when "01"   => rtx_engine.bytecnt <= "010"; spi_sdo_o <= rtx_engine.sreg(15); -- 16-bit mode
      when "10"   => rtx_engine.bytecnt <= "011"; spi_sdo_o <= rtx_engine.sreg(23); -- 24-bit mode
      when others => rtx_engine.bytecnt <= "100"; spi_sdo_o <= rtx_engine.sreg(31); -- 32-bit mode
    end case;
  end process data_size;


  -- Interrupt Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_generator: process(clk_i)
  begin
    if rising_edge(clk_i) then
      case ctrl.irq_mode is
        when "10"   => irq_gen <= irq_gen(0) & (not tx_fifo.half);  -- TX FIFO _becomes_ less than half full
        when "11"   => irq_gen <= irq_gen(0) & (not tx_fifo.avail); -- TX FIFO _becomes_ empty
        when others => irq_gen <= irq_gen(0) & rtx_engine.done;     -- current SPI transmission done (default)
      end case;
    end if;
  end process irq_generator;

  -- CPU interrupt --
  irq_o <= '1' when (ctrl.enable = '1') and (irq_gen = "01") else '0'; -- rising edge detector


end neorv32_spi_rtl;
