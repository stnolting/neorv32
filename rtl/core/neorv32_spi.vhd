-- #################################################################################################
-- # << NEORV32 - Serial Peripheral Interface Controller (SPI) >>                                  #
-- # ********************************************************************************************* #
-- # Frame format: 8/16/24/32-bit receive/transmit data, always MSB first, 2 clock modes,          #
-- # 8 pre-scaled clocks (derived from system clock), 8 dedicated chip-select lines (low-active).  #
-- # Interrupt: SPI_transfer_done                                                                  #
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

entity neorv32_spi is
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

  -- control reg bits --
  constant ctrl_spi_cs0_c    : natural :=  0; -- r/w: spi CS 0
  constant ctrl_spi_cs1_c    : natural :=  1; -- r/w: spi CS 1
  constant ctrl_spi_cs2_c    : natural :=  2; -- r/w: spi CS 2
  constant ctrl_spi_cs3_c    : natural :=  3; -- r/w: spi CS 3
  constant ctrl_spi_cs4_c    : natural :=  4; -- r/w: spi CS 4
  constant ctrl_spi_cs5_c    : natural :=  5; -- r/w: spi CS 5
  constant ctrl_spi_cs6_c    : natural :=  6; -- r/w: spi CS 6
  constant ctrl_spi_cs7_c    : natural :=  7; -- r/w: spi CS 7
  --
  constant ctrl_spi_en_c     : natural :=  8; -- r/w: spi enable
  constant ctrl_spi_cpha_c   : natural :=  9; -- r/w: spi clock phase
  constant ctrl_spi_prsc0_c  : natural := 10; -- r/w: spi prescaler select bit 0
  constant ctrl_spi_prsc1_c  : natural := 11; -- r/w: spi prescaler select bit 1
  constant ctrl_spi_prsc2_c  : natural := 12; -- r/w: spi prescaler select bit 2
  constant ctrl_spi_size0_c  : natural := 13; -- r/w: data size (00:  8-bit, 01: 16-bit)
  constant ctrl_spi_size1_c  : natural := 14; -- r/w: data size (10: 24-bit, 11: 32-bit)
  --
  constant ctrl_spi_busy_c   : natural := 31; -- r/-: spi transceiver is busy

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- accessible regs --
  signal ctrl        : std_ulogic_vector(14 downto 0);
  signal tx_data_reg : std_ulogic_vector(31 downto 0);
  signal rx_data     : std_ulogic_vector(31 downto 0);

  -- clock generator --
  signal spi_clk : std_ulogic;

  -- spi transceiver --
  signal spi_start      : std_ulogic;
  signal spi_busy       : std_ulogic;
  signal spi_state0     : std_ulogic;
  signal spi_state1     : std_ulogic;
  signal spi_rtx_sreg   : std_ulogic_vector(31 downto 0);
  signal spi_rx_data    : std_ulogic_vector(31 downto 0);
  signal spi_bitcnt     : std_ulogic_vector(05 downto 0);
  signal spi_bitcnt_max : std_ulogic_vector(05 downto 0);
  signal spi_sdi_ff0    : std_ulogic;
  signal spi_sdi_ff1    : std_ulogic;

begin

  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = spi_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= spi_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
  wren   <= acc_en and wren_i;
  rden   <= acc_en and rden_i;


  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rw_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      ack_o <= acc_en and (rden_i or wren_i);
      -- write access --
      spi_start <= '0';
      if (wren = '1') then
        if (addr = spi_ctrl_addr_c) then -- control
          ctrl <= data_i(ctrl'left downto 0);
        end if;
        if (addr = spi_rtx_addr_c) then -- tx data
          tx_data_reg <= data_i;
          spi_start   <= '1';
        end if;
      end if;
      -- read access --
      data_o <= (others => '0');
      if (rden = '1') then
        if (addr = spi_ctrl_addr_c) then
          data_o(ctrl_spi_cs0_c)    <= ctrl(ctrl_spi_cs0_c);
          data_o(ctrl_spi_cs1_c)    <= ctrl(ctrl_spi_cs1_c);
          data_o(ctrl_spi_cs2_c)    <= ctrl(ctrl_spi_cs2_c);
          data_o(ctrl_spi_cs3_c)    <= ctrl(ctrl_spi_cs3_c);
          data_o(ctrl_spi_cs4_c)    <= ctrl(ctrl_spi_cs4_c);
          data_o(ctrl_spi_cs5_c)    <= ctrl(ctrl_spi_cs5_c);
          data_o(ctrl_spi_cs6_c)    <= ctrl(ctrl_spi_cs6_c);
          data_o(ctrl_spi_cs7_c)    <= ctrl(ctrl_spi_cs7_c);
          --
          data_o(ctrl_spi_en_c)     <= ctrl(ctrl_spi_en_c);
          data_o(ctrl_spi_cpha_c)   <= ctrl(ctrl_spi_cpha_c);
          data_o(ctrl_spi_prsc0_c)  <= ctrl(ctrl_spi_prsc0_c);
          data_o(ctrl_spi_prsc1_c)  <= ctrl(ctrl_spi_prsc1_c);
          data_o(ctrl_spi_prsc2_c)  <= ctrl(ctrl_spi_prsc2_c);
          data_o(ctrl_spi_size0_c)  <= ctrl(ctrl_spi_size0_c);
          data_o(ctrl_spi_size1_c)  <= ctrl(ctrl_spi_size1_c);
          --
          data_o(ctrl_spi_busy_c)   <= spi_busy;
        else -- spi_rtx_addr_c
          data_o <= rx_data;
        end if;
      end if;
    end if;
  end process rw_access;

  -- direct chip-select (CS) (output is low-active) --  
  spi_csn_o(7 downto 0) <= not ctrl(ctrl_spi_cs7_c downto ctrl_spi_cs0_c);


  -- Clock Selection ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- clock generator enable --
  clkgen_en_o <= ctrl(ctrl_spi_en_c);

  -- spi clock select --
  spi_clk <= clkgen_i(to_integer(unsigned(ctrl(ctrl_spi_prsc2_c downto ctrl_spi_prsc0_c))));


  -- SPI Transceiver ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  spi_rtx_unit: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- input (sdi) synchronizer --
      spi_sdi_ff0 <= spi_sdi_i;
      spi_sdi_ff1 <= spi_sdi_ff0;

      -- serial engine --
      irq_o <= '0';
      if (spi_state0 = '0') or (ctrl(ctrl_spi_en_c) = '0') then -- idle or disabled
      -- --------------------------------------------------------------
        spi_bitcnt <= (others => '0');
        spi_state1 <= '0';
        spi_sdo_o  <= '0';
        spi_sck_o  <= '0';
        if (ctrl(ctrl_spi_en_c) = '0') then -- disabled
          spi_busy <= '0';
        elsif (spi_start = '1') then -- start new transmission
          spi_rtx_sreg <= tx_data_reg;
          spi_busy     <= '1';
        end if;
        spi_state0 <= spi_busy and spi_clk; -- start with next new clock pulse

      else -- transmission in progress
      -- --------------------------------------------------------------
        if (spi_state1 = '0') then -- first half of transmission
        -- --------------------------------------------------------------
          spi_sck_o <= ctrl(ctrl_spi_cpha_c);

          case ctrl(ctrl_spi_size1_c downto ctrl_spi_size0_c) is
            when "00"   => spi_sdo_o <= spi_rtx_sreg(07); -- 8-bit mode
            when "01"   => spi_sdo_o <= spi_rtx_sreg(15); -- 16-bit mode
            when "10"   => spi_sdo_o <= spi_rtx_sreg(23); -- 24-bit mode
            when others => spi_sdo_o <= spi_rtx_sreg(31); -- 32-bit mode
          end case;

          if (spi_clk = '1') then
            spi_state1 <= '1';
            if (ctrl(ctrl_spi_cpha_c) = '0') then
              spi_rtx_sreg <= spi_rtx_sreg(30 downto 0) & spi_sdi_ff1;
            end if;
            spi_bitcnt <= std_ulogic_vector(unsigned(spi_bitcnt) + 1);
          end if;

        else -- second half of transmission
        -- --------------------------------------------------------------
          spi_sck_o <= not ctrl(ctrl_spi_cpha_c);

          if (spi_clk = '1') then
            spi_state1 <= '0';
            if (ctrl(ctrl_spi_cpha_c) = '1') then
              spi_rtx_sreg <= spi_rtx_sreg(30 downto 0) & spi_sdi_ff1;
            end if;
            if (spi_bitcnt = spi_bitcnt_max) then
              spi_state0 <= '0';
              spi_busy   <= '0';
              irq_o      <= '1';
            end if;
          end if;
        end if;
      end if;
    end if;
  end process spi_rtx_unit;


  -- RTX Data size ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  data_size: process(ctrl)
  begin
    case ctrl(ctrl_spi_size1_c downto ctrl_spi_size0_c) is
      when "00"   => spi_bitcnt_max <= "001000"; -- 8-bit mode
      when "01"   => spi_bitcnt_max <= "010000"; -- 16-bit mode
      when "10"   => spi_bitcnt_max <= "011000"; -- 24-bit mode
      when others => spi_bitcnt_max <= "100000"; -- 32-bit mode
    end case;
  end process data_size;


  -- RX-Data Masking ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rx_mapping: process(ctrl, spi_rtx_sreg)
  begin
    case ctrl(ctrl_spi_size1_c downto ctrl_spi_size0_c) is
      when "00"   => rx_data <= x"000000" & spi_rtx_sreg(07 downto 0); -- 8-bit mode
      when "01"   => rx_data <= x"0000"   & spi_rtx_sreg(15 downto 0); -- 16-bit mode
      when "10"   => rx_data <= x"00"     & spi_rtx_sreg(23 downto 0); -- 24-bit mode
      when others => rx_data <=             spi_rtx_sreg(31 downto 0); -- 32-bit mode
    end case;
  end process rx_mapping;


end neorv32_spi_rtl;
