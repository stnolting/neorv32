-- #################################################################################################
-- # << NEORV32 - Smart LED (WS2811/WS2812) Interface (NEOLED) >>                                  #
-- # ********************************************************************************************* #
-- # Hardware interface for direct control of "smart LEDs" using an asynchronous serial data       #
-- # line. Compatible with the WS2811 and WS2812 LEDs.                                             #
-- #                                                                                               #
-- # NeoPixel-compatible, RGB (24-bit) and RGBW (32-bit) modes supported (in "parallel")           #
-- # (TM) "NeoPixel" is a trademark of Adafruit Industries.                                        #
-- #                                                                                               #
-- # The interface uses a programmable carrier frequency (800 KHz for the WS2812 LEDs)             #
-- # configurable via the control register's clock prescaler bits (ctrl_clksel*_c) and the period  #
-- # length configuration bits (ctrl_t_tot_*_c). "high-times" for sending a ZERO or a ONE bit are  #
-- # configured using the ctrl_t_0h_*_c and ctrl_t_1h_*_c bits, respectively. 32-bit transfers     #
-- # (for RGBW modules) and 24-bit transfers (for RGB modules) are supported via ctrl_mode__c.     #
-- #                                                                                               #
-- # The device features a TX buffer (FIFO) with <FIFO_DEPTH> entries with configurable interrupt. #
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

entity neorv32_neoled is
  generic (
    FIFO_DEPTH : natural -- NEOLED FIFO depth, has to be a power of two, min 1
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
    -- interrupt --
    irq_o       : out std_ulogic; -- interrupt request
    -- NEOLED output --
    neoled_o    : out std_ulogic -- serial async data line
  );
end neorv32_neoled;

architecture neorv32_neoled_rtl of neorv32_neoled is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(neoled_size_c); -- low address boundary bit

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- Control register bits --
  constant ctrl_en_c       : natural :=  0; -- r/w: module enable
  constant ctrl_mode_c     : natural :=  1; -- r/w: 0 = 24-bit RGB mode, 1 = 32-bit RGBW mode
  constant ctrl_strobe_c   : natural :=  2; -- r/w: 0 = send normal data, 1 = send LED strobe command (RESET) on data write
  --
  constant ctrl_clksel0_c  : natural :=  3; -- r/w: prescaler select bit 0
  constant ctrl_clksel1_c  : natural :=  4; -- r/w: prescaler select bit 1
  constant ctrl_clksel2_c  : natural :=  5; -- r/w: prescaler select bit 2
  --
  constant ctrl_bufs_0_c   : natural :=  6; -- r/-: log2(FIFO_DEPTH) bit 0
  constant ctrl_bufs_1_c   : natural :=  7; -- r/-: log2(FIFO_DEPTH) bit 1
  constant ctrl_bufs_2_c   : natural :=  8; -- r/-: log2(FIFO_DEPTH) bit 2
  constant ctrl_bufs_3_c   : natural :=  9; -- r/-: log2(FIFO_DEPTH) bit 3
  --
  constant ctrl_t_tot_0_c  : natural := 10; -- r/w: pulse-clock ticks per total period bit 0
  constant ctrl_t_tot_1_c  : natural := 11; -- r/w: pulse-clock ticks per total period bit 1
  constant ctrl_t_tot_2_c  : natural := 12; -- r/w: pulse-clock ticks per total period bit 2
  constant ctrl_t_tot_3_c  : natural := 13; -- r/w: pulse-clock ticks per total period bit 3
  constant ctrl_t_tot_4_c  : natural := 14; -- r/w: pulse-clock ticks per total period bit 4
  --
  constant ctrl_t_0h_0_c   : natural := 15; -- r/w: pulse-clock ticks per ZERO high-time bit 0
  constant ctrl_t_0h_1_c   : natural := 16; -- r/w: pulse-clock ticks per ZERO high-time bit 1
  constant ctrl_t_0h_2_c   : natural := 17; -- r/w: pulse-clock ticks per ZERO high-time bit 2
  constant ctrl_t_0h_3_c   : natural := 18; -- r/w: pulse-clock ticks per ZERO high-time bit 3
  constant ctrl_t_0h_4_c   : natural := 19; -- r/w: pulse-clock ticks per ZERO high-time bit 4
  --
  constant ctrl_t_1h_0_c   : natural := 20; -- r/w: pulse-clock ticks per ONE high-time bit 0
  constant ctrl_t_1h_1_c   : natural := 21; -- r/w: pulse-clock ticks per ONE high-time bit 1
  constant ctrl_t_1h_2_c   : natural := 22; -- r/w: pulse-clock ticks per ONE high-time bit 2
  constant ctrl_t_1h_3_c   : natural := 23; -- r/w: pulse-clock ticks per ONE high-time bit 3
  constant ctrl_t_1h_4_c   : natural := 24; -- r/w: pulse-clock ticks per ONE high-time bit 4
  --
  constant ctrl_irq_conf_c : natural := 27; -- r/w: interrupt condition: 0=IRQ when buffer less than half full, 1=IRQ when buffer is empty
  constant ctrl_tx_empty_c : natural := 28; -- r/-: TX FIFO is empty
  constant ctrl_tx_half_c  : natural := 29; -- r/-: TX FIFO is at least half-full
  constant ctrl_tx_full_c  : natural := 30; -- r/-: TX FIFO is full
  constant ctrl_tx_busy_c  : natural := 31; -- r/-: serial TX engine busy when set

  -- control register --
  type ctrl_t is record
    enable   : std_ulogic;
    mode     : std_ulogic;
    strobe   : std_ulogic;
    clk_prsc : std_ulogic_vector(2 downto 0);
    irq_conf : std_ulogic;
    t_total  : std_ulogic_vector(4 downto 0);
    t0_high  : std_ulogic_vector(4 downto 0);
    t1_high  : std_ulogic_vector(4 downto 0);
  end record;
  signal ctrl : ctrl_t;

  -- transmission buffer --
  type tx_fifo_t is record
    we    : std_ulogic; -- write enable
    re    : std_ulogic; -- read enable
    clear : std_ulogic; -- sync reset, high-active
    wdata : std_ulogic_vector(31+2 downto 0); -- write data (excluding mode)
    rdata : std_ulogic_vector(31+2 downto 0); -- read data (including mode)
    avail : std_ulogic; -- data available?
    free  : std_ulogic; -- free entry available?
    half  : std_ulogic; -- half full
  end record;
  signal tx_fifo : tx_fifo_t;

  -- serial transmission engine --
  type serial_t is record
    -- state control --
    state      : std_ulogic_vector(2 downto 0);
    mode       : std_ulogic;
    done       : std_ulogic;
    busy       : std_ulogic;
    bit_cnt    : std_ulogic_vector(5 downto 0);
    -- shift register --
    sreg       : std_ulogic_vector(31 downto 0);
    next_bit   : std_ulogic; -- next bit to send
    -- pulse generator --
    pulse_clk  : std_ulogic; -- pulse cycle "clock"
    pulse_cnt  : std_ulogic_vector(4 downto 0);
    t_high     : std_ulogic_vector(4 downto 0);
    strobe_cnt : std_ulogic_vector(6 downto 0);
  end record;
  signal serial : serial_t;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not ((is_power_of_two_f(FIFO_DEPTH) = false) or (FIFO_DEPTH < 1) or (FIFO_DEPTH > 32768))
    report "NEORV32 PROCESSOR CONFIG ERROR! Invalid NEOLED FIFO size configuration (1..32k)." severity error;


  -- Host Access ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- access control --
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = neoled_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= neoled_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
  wren   <= acc_en and wren_i;
  rden   <= acc_en and rden_i;

  -- write access --
  write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl.enable   <= '0';
      ctrl.mode     <= '0';
      ctrl.strobe   <= '0';
      ctrl.clk_prsc <= (others => '0');
      ctrl.irq_conf <= '0';
      ctrl.t_total  <= (others => '0');
      ctrl.t0_high  <= (others => '0');
      ctrl.t1_high  <= (others => '0');
    elsif rising_edge(clk_i) then
      if (wren = '1') and (addr = neoled_ctrl_addr_c) then
        ctrl.enable   <= data_i(ctrl_en_c);
        ctrl.mode     <= data_i(ctrl_mode_c);
        ctrl.strobe   <= data_i(ctrl_strobe_c);
        ctrl.clk_prsc <= data_i(ctrl_clksel2_c downto ctrl_clksel0_c);
        ctrl.irq_conf <= data_i(ctrl_irq_conf_c);
        ctrl.t_total  <= data_i(ctrl_t_tot_4_c downto ctrl_t_tot_0_c);
        ctrl.t0_high  <= data_i(ctrl_t_0h_4_c  downto ctrl_t_0h_0_c);
        ctrl.t1_high  <= data_i(ctrl_t_1h_4_c  downto ctrl_t_1h_0_c);
      end if;
    end if;
  end process write_access;

  -- read access --
  read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      ack_o  <= wren or rden; -- access acknowledge
      data_o <= (others => '0');
      if (rden = '1') then -- and (addr = neoled_ctrl_addr_c) then
        data_o(ctrl_en_c)                            <= ctrl.enable;
        data_o(ctrl_mode_c)                          <= ctrl.mode;
        data_o(ctrl_strobe_c)                        <= ctrl.strobe;
        data_o(ctrl_clksel2_c downto ctrl_clksel0_c) <= ctrl.clk_prsc;
        data_o(ctrl_irq_conf_c)                      <= ctrl.irq_conf or bool_to_ulogic_f(boolean(FIFO_DEPTH = 1)); -- tie to one if FIFO_DEPTH is 1
        data_o(ctrl_bufs_3_c  downto ctrl_bufs_0_c)  <= std_ulogic_vector(to_unsigned(index_size_f(FIFO_DEPTH), 4));
        data_o(ctrl_t_tot_4_c downto ctrl_t_tot_0_c) <= ctrl.t_total;
        data_o(ctrl_t_0h_4_c  downto ctrl_t_0h_0_c)  <= ctrl.t0_high;
        data_o(ctrl_t_1h_4_c  downto ctrl_t_1h_0_c)  <= ctrl.t1_high;
        --
        data_o(ctrl_tx_empty_c)                      <= not tx_fifo.avail;
        data_o(ctrl_tx_half_c)                       <= tx_fifo.half;
        data_o(ctrl_tx_full_c)                       <= not tx_fifo.free;
        data_o(ctrl_tx_busy_c)                       <= serial.busy;
      end if;
    end if;
  end process read_access;

  -- enable external clock generator --
  clkgen_en_o <= ctrl.enable;


  -- TX Buffer (FIFO) -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  data_buffer: neorv32_fifo
  generic map (
    FIFO_DEPTH => FIFO_DEPTH, -- number of fifo entries; has to be a power of two; min 1
    FIFO_WIDTH => 32+2,       -- size of data elements in fifo
    FIFO_RSYNC => true,       -- sync read
    FIFO_SAFE  => true        -- safe access
  )
  port map (
    -- control --
    clk_i   => clk_i,         -- clock, rising edge
    rstn_i  => rstn_i,        -- async reset, low-active
    clear_i => tx_fifo.clear, -- sync reset, high-active
    half_o  => tx_fifo.half,  -- FIFO is at least half full
    -- write port --
    wdata_i => tx_fifo.wdata, -- write data
    we_i    => tx_fifo.we,    -- write enable
    free_o  => tx_fifo.free,  -- at least one entry is free when set
    -- read port --
    re_i    => tx_fifo.re,    -- read enable
    rdata_o => tx_fifo.rdata, -- read data
    avail_o => tx_fifo.avail  -- data available when set
  );

  tx_fifo.re    <= '1' when (serial.state = "100") else '0';
  tx_fifo.we    <= '1' when (wren = '1') and (addr = neoled_data_addr_c) else '0';
  tx_fifo.wdata <= ctrl.strobe & ctrl.mode & data_i;
  tx_fifo.clear <= not ctrl.enable;

  -- IRQ generator --
  irq_generator: process(clk_i)
  begin
    if rising_edge(clk_i) then
      irq_o <= ctrl.enable and (
               ((not ctrl.irq_conf) and (not tx_fifo.avail)) or -- fire IRQ if FIFO is empty
               ((    ctrl.irq_conf) and (not tx_fifo.half)));   -- fire IRQ if FIFO is less than half full
    end if;
  end process irq_generator;


  -- Serial TX Engine -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  serial_engine: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- clock generator --
      serial.pulse_clk <= clkgen_i(to_integer(unsigned(ctrl.clk_prsc)));

      -- defaults --
      serial.done <= '0';

      -- FSM --
      serial.state(2) <= ctrl.enable;
      case serial.state is

        when "100" => -- IDLE: waiting for new TX data, prepare transmission
        -- ------------------------------------------------------------
          neoled_o          <= '0';
          serial.pulse_cnt  <= (others => '0');
          serial.strobe_cnt <= (others => '0');
          serial.sreg       <= tx_fifo.rdata(31 downto 0);
          if (tx_fifo.rdata(32) = '0') then -- "RGB" mode
            serial.mode    <= '0';
            serial.bit_cnt <= "011000"; -- total number of bits to send: 3x8=24
          else -- "RGBW" mode
            serial.mode    <= '1';
            serial.bit_cnt <= "100000"; -- total number of bits to send: 4x8=32
          end if;
          if (tx_fifo.avail = '1') then
            if (tx_fifo.rdata(33) = '0') then -- send data
              serial.state(1 downto 0) <= "01";
            else -- send RESET command
              serial.state(1 downto 0) <= "11";
            end if;
          end if;

        when "101" => -- GETBIT: get next TX bit
        -- ------------------------------------------------------------
          serial.sreg      <= serial.sreg(serial.sreg'left-1 downto 0) & '0'; -- shift left by one position (MSB-first)
          serial.bit_cnt   <= std_ulogic_vector(unsigned(serial.bit_cnt) - 1);
          serial.pulse_cnt <= (others => '0');
          if (serial.next_bit = '0') then -- send zero-bit
            serial.t_high <= ctrl.t0_high;
          else -- send one-bit
            serial.t_high <= ctrl.t1_high;
          end if;
          if (serial.bit_cnt = "000000") then -- all done?
            neoled_o                 <= '0';
            serial.done              <= '1'; -- done sending data
            serial.state(1 downto 0) <= "00";
          else -- send current data MSB
            neoled_o                 <= '1';
            serial.state(1 downto 0) <= "10"; -- transmit single pulse
          end if;

        when "110" => -- PULSE: send pulse with specific duty cycle
        -- ------------------------------------------------------------
          -- total pulse length = ctrl.t_total
          -- pulse high time    = serial.t_high
          if (serial.pulse_clk = '1') then
            serial.pulse_cnt <= std_ulogic_vector(unsigned(serial.pulse_cnt) + 1);
            -- T_high reached? --
            if (serial.pulse_cnt = serial.t_high) then
              neoled_o <= '0';
            end if;
            -- T_total reached? --
            if (serial.pulse_cnt = ctrl.t_total) then
              serial.state(1 downto 0) <= "01"; -- get next bit to send
            end if;
          end if;

        when "111" => -- STROBE: strobe LED data ("RESET" command)
        -- ------------------------------------------------------------
          -- wait for 127 * ctrl.t_total to ensure RESET
          if (serial.pulse_clk = '1') then
            -- T_total reached? --
            if (serial.pulse_cnt = ctrl.t_total) then
              serial.pulse_cnt  <= (others => '0');
              serial.strobe_cnt <= std_ulogic_vector(unsigned(serial.strobe_cnt) + 1);
            else
              serial.pulse_cnt <= std_ulogic_vector(unsigned(serial.pulse_cnt) + 1);
            end if;
          end if;
          -- number of LOW periods reached for RESET? --
          if (and_reduce_f(serial.strobe_cnt) = '1') then
            serial.done              <= '1'; -- done sending RESET
            serial.state(1 downto 0) <= "00";
          end if;

        when others => -- "0--": disabled
        -- ------------------------------------------------------------
          serial.state(1 downto 0) <= "00";

      end case;
    end if;
  end process serial_engine;

  -- SREG's TX data: bit 23 for RGB mode (24-bit), bit 31 for RGBW mode (32-bit) --
  serial.next_bit <= serial.sreg(23) when (serial.mode = '0') else serial.sreg(31);

  -- TX engine status --
  serial.busy <= '0' when (serial.state(1 downto 0) = "00") else '1';


end neorv32_neoled_rtl;
