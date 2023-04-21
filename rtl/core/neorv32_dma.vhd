-- #################################################################################################
-- # << NEORV32 - Direct Memory Access (DMA) Controller >>                                         #
-- # ********************************************************************************************* #
-- # Simple single-channel scatter/gather DMA controller that is also capable of transforming data #
-- # while moving it from source to destination.                                                   #
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

entity neorv32_dma is
  port (
    -- global control --
    clk_i          : in  std_ulogic; -- global clock line
    rstn_i         : in  std_ulogic; -- global reset line, low-active, async
    -- peripheral port: configuration and status --
    addr_i         : in  std_ulogic_vector(31 downto 0); -- address
    rden_i         : in  std_ulogic; -- read enable
    wren_i         : in  std_ulogic; -- write enable
    data_i         : in  std_ulogic_vector(31 downto 0); -- data in
    data_o         : out std_ulogic_vector(31 downto 0); -- data out
    ack_o          : out std_ulogic; -- transfer acknowledge
    -- host port: bus access --
    bus_bus_priv_o : out std_ulogic; -- current privilege level
    bus_cached_o   : out std_ulogic; -- set if cached (!) access in progress
    bus_src_o      : out std_ulogic; -- access source
    bus_addr_o     : out std_ulogic_vector(31 downto 0); -- bus access address
    bus_rdata_i    : in  std_ulogic_vector(31 downto 0); -- bus read data
    bus_wdata_o    : out std_ulogic_vector(31 downto 0); -- bus write data
    bus_ben_o      : out std_ulogic_vector(03 downto 0); -- byte enable
    bus_we_o       : out std_ulogic; -- write enable
    bus_re_o       : out std_ulogic; -- read enable
    bus_ack_i      : in  std_ulogic; -- bus transfer acknowledge
    bus_err_i      : in  std_ulogic; -- bus transfer error
    -- interrupt --
    irq_o          : out std_ulogic
  );
end neorv32_dma;

architecture neorv32_dma_rtl of neorv32_dma is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(dma_size_c); -- low address boundary bit

  -- control access control --
  signal acc_en : std_ulogic; -- module access enable
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- transfer type register bits --
  constant type_num_lo_c      : natural :=  0; -- r/w: Number of elements to transfer, LSB
  constant type_num_hi_c      : natural := 23; -- r/w: Number of elements to transfer, MSB
  constant type_src_qsel_lo_c : natural := 24; -- r/w: SRC data quantity, LSB, 00=byte, 01=half-word
  constant type_src_qsel_hi_c : natural := 25; -- r/w: SRC data quantity, MSB, 10=word, 11=word
  constant type_dst_qsel_lo_c : natural := 26; -- r/w: DST data quantity, LSB, 00=byte, 01=half-word
  constant type_dst_qsel_hi_c : natural := 27; -- r/w: DST data quantity, MSB, 10=word, 11=word
  constant type_src_inc_c     : natural := 28; -- r/w: SRC constant (0) or incrementing (1) address
  constant type_dst_inc_c     : natural := 29; -- r/w: DST constant (0) or incrementing (1) address
  constant type_sext_c        : natural := 30; -- r/w: Sign-extend sub-words when set
  constant type_endian_c      : natural := 31; -- r/w: Convert Endianness when set

  -- control and status register bits --
  constant ctrl_en_c       : natural :=  0; -- r/w: DMA enable
  constant ctrl_error_rd_c : natural := 29; -- r/-: error during read transfer
  constant ctrl_error_wr_c : natural := 30; -- r/-: error during write transfer
  constant ctrl_busy_c     : natural := 31; -- r/-: DMA transfer in progress

begin

  -- ----------------------- --
  -- TODO - WORK-IN-PROGRESS --
  -- ----------------------- --

  data_o         <= (others => '0');
  ack_o          <= '0';

  bus_bus_priv_o <= '0';
  bus_cached_o   <= '0';
  bus_src_o      <= '0';
  bus_addr_o     <= (others => '0');
  bus_wdata_o    <= (others => '0');
  bus_ben_o      <= (others => '0');
  bus_we_o       <= '0';
  bus_re_o       <= '0';

  irq_o          <= '0';

end neorv32_dma_rtl;
