-- #################################################################################################
-- # << NEORV32 - Processor-Internal IMEM for Lattice iCE40 UltraPlus >>                           #
-- # ********************************************************************************************* #
-- # Memory has a physical size of 64kb (2 x SPRAMs).                                              #
-- # Logical size IMEM_SIZE must be less or equal.                                                 #
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

library iCE40UP;
use iCE40UP.components.all; -- for device primitives

architecture neorv32_imem_rtl of neorv32_imem is

  -- advanced configuration --------------------------------------------------------------------------------
  constant spram_sleep_mode_en_c : boolean := false; -- put IMEM into sleep mode when idle (for low power)
  -- -------------------------------------------------------------------------------------------------------

  -- IO space: module base address --
  constant hi_abb_c : natural := 31; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(64*1024); -- low address boundary bit

  -- local signals --
  signal acc_en : std_ulogic;
  signal mem_cs : std_ulogic;
  signal rdata  : std_ulogic_vector(31 downto 0);
  signal rden   : std_ulogic;

  -- SPRAM signals --
  signal spram_clk   : std_logic;
  signal spram_addr  : std_logic_vector(13 downto 0);
  signal spram_di_lo : std_logic_vector(15 downto 0);
  signal spram_di_hi : std_logic_vector(15 downto 0);
  signal spram_do_lo : std_logic_vector(15 downto 0);
  signal spram_do_hi : std_logic_vector(15 downto 0);
  signal spram_be_lo : std_logic_vector(03 downto 0);
  signal spram_be_hi : std_logic_vector(03 downto 0);
  signal spram_we    : std_logic;
  signal spram_pwr_n : std_logic;
  signal spram_cs    : std_logic;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert false report "NEORV32 PROCESSOR CONFIG NOTE: Using iCE40up SPRAM-based IMEM." severity note;
  assert not (IMEM_AS_IROM = true) report "NEORV32 PROCESSOR CONFIG ERROR: ICE40 Ultra Plus SPRAM cannot be initialized by bitstream!" severity failure;
  assert not (IMEM_SIZE > 64*1024) report "NEORV32 PROCESSOR CONFIG ERROR: IMEM has a fixed physical size of 64kB. Logical size must be less or equal." severity error;


  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = IMEM_BASE(hi_abb_c downto lo_abb_c)) else '0';
  mem_cs <= acc_en and (rden_i or wren_i);


  -- Memory Access --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  imem_spram_lo_inst : SP256K
  port map (
    AD       => spram_addr,  -- I
    DI       => spram_di_lo, -- I
    MASKWE   => spram_be_lo, -- I
    WE       => spram_we,    -- I
    CS       => spram_cs,    -- I
    CK       => spram_clk,   -- I
    STDBY    => '0',         -- I
    SLEEP    => spram_pwr_n, -- I
    PWROFF_N => '1',         -- I
    DO       => spram_do_lo  -- O
  );

  imem_spram_hi_inst : SP256K
  port map (
    AD       => spram_addr,  -- I
    DI       => spram_di_hi, -- I
    MASKWE   => spram_be_hi, -- I
    WE       => spram_we,    -- I
    CS       => spram_cs,    -- I
    CK       => spram_clk,   -- I
    STDBY    => '0',         -- I
    SLEEP    => spram_pwr_n, -- I
    PWROFF_N => '1',         -- I
    DO       => spram_do_hi  -- O
  );

  -- access logic and signal type conversion --
  spram_clk   <= std_logic(clk_i);
  spram_addr  <= std_logic_vector(addr_i(13+2 downto 0+2));
  spram_di_lo <= std_logic_vector(data_i(15 downto 00));
  spram_di_hi <= std_logic_vector(data_i(31 downto 16));
  spram_we    <= '1' when ((acc_en and wren_i) = '1') else '0'; -- global write enable
  spram_cs    <= std_logic(mem_cs);
  spram_be_lo <= std_logic(ben_i(1)) & std_logic(ben_i(1)) & std_logic(ben_i(0)) & std_logic(ben_i(0)); -- low byte write enable
  spram_be_hi <= std_logic(ben_i(3)) & std_logic(ben_i(3)) & std_logic(ben_i(2)) & std_logic(ben_i(2)); -- high byte write enable
  spram_pwr_n <= '0' when ((spram_sleep_mode_en_c = false) or (mem_cs = '1')) else '1'; -- LP mode disabled or IMEM selected
  rdata       <= std_ulogic_vector(spram_do_hi) & std_ulogic_vector(spram_do_lo);

  buffer_ff: process(clk_i)
  begin
    if rising_edge(clk_i) then
      ack_o <= mem_cs;
      rden  <= acc_en and rden_i;
    end if;
  end process buffer_ff;

  -- output gate --
  data_o <= rdata when (rden = '1') else (others => '0');


end neorv32_imem_rtl;
