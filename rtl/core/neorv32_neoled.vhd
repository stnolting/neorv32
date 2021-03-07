-- #################################################################################################
-- # << NEORV32 - Smart LED (WS2811/WS2812) Interface (NEOLED) >>                                  #
-- # ********************************************************************************************* #
-- # Hardware interface for direct control of "smart LEDs" using an asynchronouse serial data      #
-- # line. Compatible with the WS2811 and WS2812 LEDs.                                             #
-- #                                                                                               #
-- # NeoPixel-compatible, RGB (24-bit) and RGBW (32-bit)                                           #
-- # (c) "NeoPixel" is a trademark of Adafruit Industries.                                         #
-- #                                                                                               #
-- # The interface uses a programmable carries frequency (800 KHz for the WS2812 LEDs)             #
-- # configurable via the control register's clock prescaler bits (ctrl_clksel*_c) and the period  #
-- # length configuration bits (ctrl_t_tot_*_c). "high-times" for sending a ZERO or a ONE bit are  #
-- # configured using the ctrl_t_0h_*_c and ctrl_t_1h_*_c bits, respectively. 32-bit transfers     #
-- # (for RGBW modules) and 24-bit transfers (for RGB modules) are supported via ctrl_mode__c.     #
-- #                                                                                               #
-- # The device features a TX buffer with <tx_buffer_entries_c> entries. The devices busy flag and #
-- # IRQ generator can be programmed to either clear the busy flag / send an IRQ when AT LEAST ONE #
-- # FREE BUFFER ENTRY is available (ctrl_bscon_c = 0) or when the WHOLE BUFFER IS EMPTY           #
-- # (ctrl_bscon_c = 1).                                                                           #
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

entity neorv32_neoled is
  port (
    -- host access --
    clk_i       : in  std_ulogic; -- global clock line
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

  -- TX buffer size configuration --
  constant tx_buffer_entries_c : natural := 4; -- number of entries in TX buffer, has to be a power of two, min=0

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(neoled_size_c); -- low address boundary bit

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- Control register bits --
  constant ctrl_enable_c    : natural :=  0; -- r/w: module enable
  constant ctrl_mode_c      : natural :=  1; -- r/w: 0 = 24-bit RGB mode, 1 = 32-bit RGBW mode
  constant ctrl_bscon_c     : natural :=  2; -- r/w: buffer status configuration -> busy_flag/IRQ config
  constant ctrl_clksel0_c   : natural :=  3; -- r/w: prescaler select bit 0
  constant ctrl_clksel1_c   : natural :=  4; -- r/w: prescaler select bit 1
  constant ctrl_clksel2_c   : natural :=  5; -- r/w: prescaler select bit 2
  --
  constant ctrl_bufs_0_c    : natural :=  6; -- r/-: log2(tx_buffer_entries_c) bit 0
  constant ctrl_bufs_1_c    : natural :=  7; -- r/-: log2(tx_buffer_entries_c) bit 1
  constant ctrl_bufs_2_c    : natural :=  8; -- r/-: log2(tx_buffer_entries_c) bit 2
  constant ctrl_bufs_3_c    : natural :=  9; -- r/-: log2(tx_buffer_entries_c) bit 3
  --
  constant ctrl_t_tot_0_c   : natural := 10; -- r/w: pulse-clock ticks per total period bit 0
  constant ctrl_t_tot_1_c   : natural := 11; -- r/w: pulse-clock ticks per total period bit 1
  constant ctrl_t_tot_2_c   : natural := 12; -- r/w: pulse-clock ticks per total period bit 2
  constant ctrl_t_tot_3_c   : natural := 13; -- r/w: pulse-clock ticks per total period bit 3
  constant ctrl_t_tot_4_c   : natural := 14; -- r/w: pulse-clock ticks per total period bit 4
  --
  constant ctrl_t_0h_0_c    : natural := 15; -- r/w: pulse-clock ticks per ZERO high-time bit 0
  constant ctrl_t_0h_1_c    : natural := 16; -- r/w: pulse-clock ticks per ZERO high-time bit 1
  constant ctrl_t_0h_2_c    : natural := 17; -- r/w: pulse-clock ticks per ZERO high-time bit 2
  constant ctrl_t_0h_3_c    : natural := 18; -- r/w: pulse-clock ticks per ZERO high-time bit 3
  constant ctrl_t_0h_4_c    : natural := 19; -- r/w: pulse-clock ticks per ZERO high-time bit 4
  --
  constant ctrl_t_1h_0_c    : natural := 20; -- r/w: pulse-clock ticks per ONE high-time bit 0
  constant ctrl_t_1h_1_c    : natural := 21; -- r/w: pulse-clock ticks per ONE high-time bit 1
  constant ctrl_t_1h_2_c    : natural := 22; -- r/w: pulse-clock ticks per ONE high-time bit 2
  constant ctrl_t_1h_3_c    : natural := 23; -- r/w: pulse-clock ticks per ONE high-time bit 3
  constant ctrl_t_1h_4_c    : natural := 24; -- r/w: pulse-clock ticks per ONE high-time bit 4
  --
  constant ctrl_tx_status_c : natural := 30; -- r/-: serial TX engine busy when set
  constant ctrl_busy_c      : natural := 31; -- r/-: busy / buffer status flag (configured via ctrl_bscon_c)

  -- control register --
  type ctrl_t is record
    enable   : std_ulogic;
    bscon    : std_ulogic; -- buffer/busy status flag configuration
    mode     : std_ulogic;
    clk_prsc : std_ulogic_vector(2 downto 0);
    ready    : std_ulogic; -- buffer ready to accept new data
    -- pulse config --
    t_total  : std_ulogic_vector(4 downto 0);
    t0_high  : std_ulogic_vector(4 downto 0);
    t1_high  : std_ulogic_vector(4 downto 0);
  end record;
  signal ctrl : ctrl_t;

  -- transmission buffer --
  type tx_fifo_t is array (0 to tx_buffer_entries_c-1) of std_ulogic_vector(31+1 downto 0);
  type tx_buffer_t is record
    we       : std_ulogic; -- write enable
    re       : std_ulogic; -- read enable
    wdata    : std_ulogic_vector(31 downto 0); -- write data (excluding excluding)
    rdata    : std_ulogic_vector(31+1 downto 0); -- read data (including mode)
    --
    w_pnt    : std_ulogic_vector(index_size_f(tx_buffer_entries_c) downto 0); -- write pointer
    r_pnt    : std_ulogic_vector(index_size_f(tx_buffer_entries_c) downto 0); -- read pointer
    match    : std_ulogic;
    empty    : std_ulogic;
    empty_ff : std_ulogic;
    full     : std_ulogic;
    avail    : std_ulogic; -- data available?
    free     : std_ulogic; -- free entry available?
    free_ff  : std_ulogic;
    --
    data  : tx_fifo_t; -- fifo memory
  end record;
  signal tx_buffer : tx_buffer_t;

  -- serial transmission engine --
  type serial_state_t is (S_IDLE, S_INIT, S_GETBIT, S_PULSE);
  type serial_t is record
    -- state control --
    state     : serial_state_t;
    mode      : std_ulogic;
    busy      : std_ulogic;
    bit_cnt   : std_ulogic_vector(5 downto 0);
    -- shift register --
    sreg      : std_ulogic_vector(31 downto 0);
    next_bit  : std_ulogic; -- next bit to send
    -- pulse generator --
    pulse_clk : std_ulogic; -- pulse cycle "clock"
    pulse_cnt : std_ulogic_vector(4 downto 0);
    t_high    : std_ulogic_vector(4 downto 0);
    output    : std_ulogic;
  end record;
  signal serial : serial_t;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not ((is_power_of_two_f(tx_buffer_entries_c) = false) or (tx_buffer_entries_c > 32768)) report "NEORV32 PROCESSOR CONFIG ERROR! Invalid <IO.NEOPIX> buffer size configuration!" severity error;


  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = neoled_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= neoled_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
  wren   <= acc_en and wren_i;
  rden   <= acc_en and rden_i;


  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rw_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- access acknowledge --
      ack_o <= wren or rden;

      -- write access --
      tx_buffer.we <= '0';
      if (wren = '1') then
        -- control register --
        if (addr = neoled_ctrl_addr_c) then
          ctrl.enable   <= data_i(ctrl_enable_c);
          ctrl.mode     <= data_i(ctrl_mode_c);
          ctrl.bscon    <= data_i(ctrl_bscon_c);
          ctrl.clk_prsc <= data_i(ctrl_clksel2_c downto ctrl_clksel0_c);
          ctrl.t_total  <= data_i(ctrl_t_tot_4_c downto ctrl_t_tot_0_c);
          ctrl.t0_high  <= data_i(ctrl_t_0h_4_c  downto ctrl_t_0h_0_c);
          ctrl.t1_high  <= data_i(ctrl_t_1h_4_c  downto ctrl_t_1h_0_c);
        end if;
        -- tx data register (FIFO) --
        if (addr = neoled_data_addr_c) then
          tx_buffer.wdata <= data_i;
          tx_buffer.we    <= tx_buffer.free; -- only write new data if there is at least one free entry left
        end if;
      end if;

      -- read access: control register --
      data_o <= (others => '0');
      if (rden = '1') and (addr = neoled_ctrl_addr_c) then
        data_o(ctrl_enable_c)                        <= ctrl.enable;
        data_o(ctrl_mode_c)                          <= ctrl.mode;
        data_o(ctrl_bscon_c)                         <= ctrl.bscon;
        data_o(ctrl_clksel2_c downto ctrl_clksel0_c) <= ctrl.clk_prsc;
        data_o(ctrl_bufs_3_c  downto ctrl_bufs_0_c)  <= std_ulogic_vector(to_unsigned(index_size_f(tx_buffer_entries_c), 4));
        data_o(ctrl_t_tot_4_c downto ctrl_t_tot_0_c) <= ctrl.t_total;
        data_o(ctrl_t_0h_4_c  downto ctrl_t_0h_0_c)  <= ctrl.t0_high;
        data_o(ctrl_t_1h_4_c  downto ctrl_t_1h_0_c)  <= ctrl.t1_high;
        data_o(ctrl_tx_status_c)                     <= serial.busy;
        data_o(ctrl_busy_c)                          <= not ctrl.ready;
      end if;
    end if;
  end process rw_access;

  -- enable external clock generator --
  clkgen_en_o <= ctrl.enable;


  -- TX Buffer (FIFO) -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  instr_prefetch_buffer: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- write port --
      if (ctrl.enable = '0') then
        tx_buffer.w_pnt <= (others => '0');
      elsif (tx_buffer.we = '1') then
        tx_buffer.w_pnt <= std_ulogic_vector(unsigned(tx_buffer.w_pnt) + 1);
      end if;
      if (tx_buffer.we = '1') then -- write data
        tx_buffer.data(to_integer(unsigned(tx_buffer.w_pnt(tx_buffer.w_pnt'left-1 downto 0)))) <=  ctrl.mode & tx_buffer.wdata;
      end if;
      -- read port --
      if (ctrl.enable = '0') then
        tx_buffer.r_pnt <= (others => '0');
      elsif (tx_buffer.re = '1') then
        tx_buffer.r_pnt <= std_ulogic_vector(unsigned(tx_buffer.r_pnt) + 1);
      end if;
      tx_buffer.rdata <= tx_buffer.data(to_integer(unsigned(tx_buffer.r_pnt(tx_buffer.r_pnt'left-1 downto 0)))); -- sync read
      -- status buffer --
      tx_buffer.empty_ff <= tx_buffer.empty;
      tx_buffer.free_ff  <= tx_buffer.free;
    end if;
  end process instr_prefetch_buffer;

  -- status --
  tx_buffer.match <= '1' when (tx_buffer.r_pnt(tx_buffer.r_pnt'left-1 downto 0) = tx_buffer.w_pnt(tx_buffer.w_pnt'left-1 downto 0)) else '0';
  tx_buffer.full  <= '1' when (tx_buffer.r_pnt(tx_buffer.r_pnt'left) /= tx_buffer.w_pnt(tx_buffer.w_pnt'left)) and (tx_buffer.match = '1') else '0';
  tx_buffer.empty <= '1' when (tx_buffer.r_pnt(tx_buffer.r_pnt'left)  = tx_buffer.w_pnt(tx_buffer.w_pnt'left)) and (tx_buffer.match = '1') else '0';
  tx_buffer.free  <= not tx_buffer.full;
  tx_buffer.avail <= not tx_buffer.empty;


  -- Buffer Status Flag and IRQ Generator ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- ctrl.bscon = 0: clear buffer/busy status flag and send IRQ if -> there is at least one free entry in buffer
  -- ctrl.bscon = 1: clear buffer/busy status flag and send IRQ if -> the complete buffer is empty
  irq_generator: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (ctrl.enable = '1') then
        if (ctrl.bscon = '0') then -- one entry is becoming free
          irq_o <= (not tx_buffer.free_ff) and tx_buffer.free;
        else -- buffer is becoming empty
          irq_o <= (not tx_buffer.empty_ff) and tx_buffer.empty;
        end if;
      else
        irq_o <= '0';
      end if;
    end if;
  end process irq_generator;

  -- ready flag --
  ctrl.ready <= tx_buffer.free when (ctrl.bscon = '0') else tx_buffer.empty;


  -- Serial TX Engine -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  serial_engine: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- defaults --
      serial.pulse_clk <= clkgen_i(to_integer(unsigned(ctrl.clk_prsc)));

      -- disabled --
      if (ctrl.enable = '0') then -- disabled
        serial.output <= '0';
        serial.state  <= S_IDLE;
      else
        case serial.state is

          when S_IDLE => -- waiting for new TX data
          -- ------------------------------------------------------------
            serial.output    <= '0';
            serial.pulse_cnt <= (others => '0');
            if (tx_buffer.avail = '1') then
              serial.state <= S_INIT;
            end if;

          when S_INIT => -- initialize TX shift engine
          -- ------------------------------------------------------------
            if (tx_buffer.rdata(32) = '0') then -- mode = "RGB" 
              serial.mode    <= '0';
              serial.bit_cnt <= "011000"; -- total number of bits to send: 3x8=24
            else -- mode = "RGBW"
              serial.mode    <= '1';
              serial.bit_cnt <= "100000"; -- total number of bits to send: 4x8=32
            end if;
            serial.sreg  <= tx_buffer.rdata(31 downto 00);
            serial.state <= S_GETBIT;

          when S_GETBIT => -- get next TX bit
          -- ------------------------------------------------------------
            serial.sreg      <= serial.sreg(serial.sreg'left-1 downto 0) & '0'; -- shift left by one position (MSB-first)
            serial.bit_cnt   <= std_ulogic_vector(unsigned(serial.bit_cnt) - 1);
            serial.pulse_cnt <= (others => '0');
            if (serial.bit_cnt = "000000") then -- all done?
              serial.state <= S_IDLE;
            else -- check current data MSB
              if (serial.next_bit = '0') then -- send zero-bit
                serial.t_high <= ctrl.t0_high;
              else -- send one-bit
                serial.t_high <= ctrl.t1_high;
              end if;
              serial.state  <= S_PULSE; -- transmit single pulse
              serial.output <= '1';
            end if;

          when S_PULSE => -- send pulse with specific duty cycle
          -- ------------------------------------------------------------
            -- total pulse length = ctrl.t_total
            -- pulse high time    = serial.t_high
            if (serial.pulse_clk = '1') then
              serial.pulse_cnt <= std_ulogic_vector(unsigned(serial.pulse_cnt) + 1);
              -- T_high reached? --
              if (serial.pulse_cnt = serial.t_high) then
                serial.output <= '0';
              end if;
              -- T_total reached? --
              if (serial.pulse_cnt = ctrl.t_total) then
                serial.state <= S_GETBIT; -- get next bit to send
              end if;
            end if;

          when others => -- undefined
          -- ------------------------------------------------------------
            serial.state <= S_IDLE;

        end case;
      end if;
      -- serial data output --
      neoled_o <= serial.output; -- IOB.FF
    end if;
  end process serial_engine;

  -- SREG's TX data: bit 23 for RGB mode (24-bit), bit 31 for RGBW mode (32-bit) --
  serial.next_bit <= serial.sreg(23) when (serial.mode = '0') else serial.sreg(31);

  -- get new TX data --
  tx_buffer.re <= '1' when (serial.state = S_IDLE) and (tx_buffer.avail = '1') else '0';

  -- TX engine status --
  serial.busy <= '0' when (serial.state = S_IDLE) or (ctrl.enable = '0') else '1';


end neorv32_neoled_rtl;
