-- #################################################################################################
-- # << NEORV32 - Serial Peripheral Interface Controller (SPI) >>                                  #
-- # ********************************************************************************************* #
-- # Frame format: 8/16/24/32-bit receive/transmit data, always MSB first, 2 clock modes,          #
-- # 8 pre-scaled clocks (derived from system clock), 8 dedicated chip-select lines (low-active).  #
-- # Interrupt: "transfer done"                                                                    #
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

  -- control register --
  constant ctrl_cs0_c   : natural :=  0; -- r/w: spi CS 0
  constant ctrl_cs1_c   : natural :=  1; -- r/w: spi CS 1
  constant ctrl_cs2_c   : natural :=  2; -- r/w: spi CS 2
  constant ctrl_cs3_c   : natural :=  3; -- r/w: spi CS 3
  constant ctrl_cs4_c   : natural :=  4; -- r/w: spi CS 4
  constant ctrl_cs5_c   : natural :=  5; -- r/w: spi CS 5
  constant ctrl_cs6_c   : natural :=  6; -- r/w: spi CS 6
  constant ctrl_cs7_c   : natural :=  7; -- r/w: spi CS 7
  --
  constant ctrl_en_c    : natural :=  8; -- r/w: spi enable
  constant ctrl_cpha_c  : natural :=  9; -- r/w: spi clock phase
  constant ctrl_prsc0_c : natural := 10; -- r/w: spi prescaler select bit 0
  constant ctrl_prsc1_c : natural := 11; -- r/w: spi prescaler select bit 1
  constant ctrl_prsc2_c : natural := 12; -- r/w: spi prescaler select bit 2
  constant ctrl_size0_c : natural := 13; -- r/w: data size lsb (00:  8-bit, 01: 16-bit)
  constant ctrl_size1_c : natural := 14; -- r/w: data size msb (10: 24-bit, 11: 32-bit)
  constant ctrl_cpol_c  : natural := 15; -- r/w: spi clock polarity
  --
  constant ctrl_busy_c  : natural := 31; -- r/-: spi transceiver is busy
  --
  signal ctrl : std_ulogic_vector(15 downto 0);

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- clock generator --
  signal spi_clk_en : std_ulogic;

  -- spi transceiver --
  type rtx_engine_t is record
    state    : std_ulogic_vector(02 downto 0);
    busy     : std_ulogic;
    start    : std_ulogic;
    sreg     : std_ulogic_vector(31 downto 0);
    bitcnt   : std_ulogic_vector(05 downto 0);
    bytecnt  : std_ulogic_vector(02 downto 0);
    sdi_sync : std_ulogic_vector(01 downto 0);
  end record;
  signal rtx_engine : rtx_engine_t;

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
      -- bus access acknowledge --
      ack_o <= rden or wren;

      -- write access --
      if (wren = '1') then
        if (addr = spi_ctrl_addr_c) then -- control register
          ctrl(ctrl_cs0_c)   <= data_i(ctrl_cs0_c);
          ctrl(ctrl_cs1_c)   <= data_i(ctrl_cs1_c);
          ctrl(ctrl_cs2_c)   <= data_i(ctrl_cs2_c);
          ctrl(ctrl_cs3_c)   <= data_i(ctrl_cs3_c);
          ctrl(ctrl_cs4_c)   <= data_i(ctrl_cs4_c);
          ctrl(ctrl_cs5_c)   <= data_i(ctrl_cs5_c);
          ctrl(ctrl_cs6_c)   <= data_i(ctrl_cs6_c);
          ctrl(ctrl_cs7_c)   <= data_i(ctrl_cs7_c);
          --
          ctrl(ctrl_en_c)    <= data_i(ctrl_en_c);
          ctrl(ctrl_cpha_c)  <= data_i(ctrl_cpha_c);
          ctrl(ctrl_prsc0_c) <= data_i(ctrl_prsc0_c);
          ctrl(ctrl_prsc1_c) <= data_i(ctrl_prsc1_c);
          ctrl(ctrl_prsc2_c) <= data_i(ctrl_prsc2_c);
          ctrl(ctrl_size0_c) <= data_i(ctrl_size0_c);
          ctrl(ctrl_size1_c) <= data_i(ctrl_size1_c);
          ctrl(ctrl_cpol_c)  <= data_i(ctrl_cpol_c);
        end if;
      end if;

      -- read access --
      data_o <= (others => '0');
      if (rden = '1') then
        if (addr = spi_ctrl_addr_c) then -- control register
          data_o(ctrl_cs0_c)   <= ctrl(ctrl_cs0_c);
          data_o(ctrl_cs1_c)   <= ctrl(ctrl_cs1_c);
          data_o(ctrl_cs2_c)   <= ctrl(ctrl_cs2_c);
          data_o(ctrl_cs3_c)   <= ctrl(ctrl_cs3_c);
          data_o(ctrl_cs4_c)   <= ctrl(ctrl_cs4_c);
          data_o(ctrl_cs5_c)   <= ctrl(ctrl_cs5_c);
          data_o(ctrl_cs6_c)   <= ctrl(ctrl_cs6_c);
          data_o(ctrl_cs7_c)   <= ctrl(ctrl_cs7_c);
          --
          data_o(ctrl_en_c)    <= ctrl(ctrl_en_c);
          data_o(ctrl_cpha_c)  <= ctrl(ctrl_cpha_c);
          data_o(ctrl_prsc0_c) <= ctrl(ctrl_prsc0_c);
          data_o(ctrl_prsc1_c) <= ctrl(ctrl_prsc1_c);
          data_o(ctrl_prsc2_c) <= ctrl(ctrl_prsc2_c);
          data_o(ctrl_size0_c) <= ctrl(ctrl_size0_c);
          data_o(ctrl_size1_c) <= ctrl(ctrl_size1_c);
          data_o(ctrl_cpol_c)  <= ctrl(ctrl_cpol_c);
          --
          data_o(ctrl_busy_c)  <= rtx_engine.busy;
        else -- data register (spi_rtx_addr_c)
          data_o <= rtx_engine.sreg;
        end if;
      end if;
    end if;
  end process rw_access;

  -- direct chip-select (CS), output is low-active --  
  spi_csn_o(7 downto 0) <= not ctrl(ctrl_cs7_c downto ctrl_cs0_c);

  -- trigger new SPI transmission --
  rtx_engine.start <= '1' when (wren = '1') and (addr = spi_rtx_addr_c) else '0';


  -- Clock Selection ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  clkgen_en_o <= ctrl(ctrl_en_c); -- clock generator enable
  spi_clk_en  <= clkgen_i(to_integer(unsigned(ctrl(ctrl_prsc2_c downto ctrl_prsc0_c)))); -- clock select


  -- Transmission Data Size -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  data_size: process(ctrl)
  begin
    case ctrl(ctrl_size1_c downto ctrl_size0_c) is
      when "00"   => rtx_engine.bytecnt <= "001"; -- 1-byte mode
      when "01"   => rtx_engine.bytecnt <= "010"; -- 2-byte mode
      when "10"   => rtx_engine.bytecnt <= "011"; -- 3-byte mode
      when others => rtx_engine.bytecnt <= "100"; -- 4-byte mode
    end case;
  end process data_size;


  -- SPI Transceiver ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  spi_rtx_unit: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- input (sdi) synchronizer --
      rtx_engine.sdi_sync <= rtx_engine.sdi_sync(0) & spi_sdi_i;

      -- output (sdo) buffer --
      case ctrl(ctrl_size1_c downto ctrl_size0_c) is
        when "00"   => spi_sdo_o <= rtx_engine.sreg(07); -- 8-bit mode
        when "01"   => spi_sdo_o <= rtx_engine.sreg(15); -- 16-bit mode
        when "10"   => spi_sdo_o <= rtx_engine.sreg(23); -- 24-bit mode
        when others => spi_sdo_o <= rtx_engine.sreg(31); -- 32-bit mode
      end case;

      -- defaults --
      spi_sck_o <= ctrl(ctrl_cpol_c);
      irq_o     <= '0';

      -- serial engine --
      rtx_engine.state(2) <= ctrl(ctrl_en_c);
      case rtx_engine.state is

        when "100" => -- enabled but idle, waiting for new transmission trigger
        -- ------------------------------------------------------------
          rtx_engine.bitcnt <= (others => '0');
          if (rtx_engine.start = '1') then -- trigger new transmission
            rtx_engine.sreg <= data_i;
            rtx_engine.state(1 downto 0) <= "01";
          end if;

        when "101" => -- start with next new clock pulse
        -- ------------------------------------------------------------
          if (spi_clk_en = '1') then
            rtx_engine.state(1 downto 0) <= "10";
          end if;

        when "110" => -- first half of bit transmission
        -- ------------------------------------------------------------
          spi_sck_o <= ctrl(ctrl_cpha_c) xor ctrl(ctrl_cpol_c);
          if (spi_clk_en = '1') then
            rtx_engine.bitcnt <= std_ulogic_vector(unsigned(rtx_engine.bitcnt) + 1);
            rtx_engine.state(1 downto 0) <= "11";
          end if;

        when "111" => -- second half of bit transmission
        -- ------------------------------------------------------------
          spi_sck_o <= ctrl(ctrl_cpha_c) xnor ctrl(ctrl_cpol_c);
          if (spi_clk_en = '1') then
            rtx_engine.sreg <= rtx_engine.sreg(30 downto 0) & rtx_engine.sdi_sync(rtx_engine.sdi_sync'left);
            if (rtx_engine.bitcnt(5 downto 3) = rtx_engine.bytecnt) then -- all bits transferred?
              irq_o <= '1'; -- interrupt!
              rtx_engine.state(1 downto 0) <= "00"; -- transmission done
            else
              rtx_engine.state(1 downto 0) <= "10";
            end if;
          end if;

        when others => -- "0--": SPI deactivated
        -- ------------------------------------------------------------
          rtx_engine.state(1 downto 0) <= "00";

      end case;
    end if;
  end process spi_rtx_unit;

  -- busy flag --
  rtx_engine.busy <= '0' when (rtx_engine.state(1 downto 0) = "00") else '1';


end neorv32_spi_rtl;
