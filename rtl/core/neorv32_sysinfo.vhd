-- #################################################################################################
-- # << NEORV32 - System/Processor Configuration Information Memory (SYSINFO) >>                   #
-- # ********************************************************************************************* #
-- # This unit provides information regarding the NEORV32 processor system configuration -         #
-- # mostly derived from the top's configuration generics.                                         #
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

entity neorv32_sysinfo is
  generic (
    -- General --
    CLOCK_FREQUENCY      : natural := 0;      -- clock frequency of clk_i in Hz
    BOOTLOADER_EN        : boolean := true;   -- implement processor-internal bootloader?
    USER_CODE            : std_ulogic_vector(31 downto 0) := x"00000000"; -- custom user code
    -- Internal Instruction memory --
    MEM_INT_IMEM_EN      : boolean := true;   -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE    : natural := 8*1024; -- size of processor-internal instruction memory in bytes
    MEM_INT_IMEM_ROM     : boolean := false;  -- implement processor-internal instruction memory as ROM
    -- Internal Data memory --
    MEM_INT_DMEM_EN      : boolean := true;   -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE    : natural := 4*1024; -- size of processor-internal data memory in bytes
    -- Internal Cache memory --
    ICACHE_EN            : boolean := true;   -- implement instruction cache
    ICACHE_NUM_BLOCKS    : natural := 4;      -- i-cache: number of blocks (min 2), has to be a power of 2
    ICACHE_BLOCK_SIZE    : natural := 64;     -- i-cache: block size in bytes (min 4), has to be a power of 2
    ICACHE_ASSOCIATIVITY : natural := 1;      -- i-cache: associativity (min 1), has to be a power 2
    -- External memory interface --
    MEM_EXT_EN           : boolean := false;  -- implement external memory bus interface?
    -- Processor peripherals --
    IO_GPIO_EN           : boolean := true;   -- implement general purpose input/output port unit (GPIO)?
    IO_MTIME_EN          : boolean := true;   -- implement machine system timer (MTIME)?
    IO_UART0_EN          : boolean := true;   -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_UART1_EN          : boolean := true;   -- implement secondary universal asynchronous receiver/transmitter (UART1)?
    IO_SPI_EN            : boolean := true;   -- implement serial peripheral interface (SPI)?
    IO_TWI_EN            : boolean := true;   -- implement two-wire interface (TWI)?
    IO_PWM_EN            : boolean := true;   -- implement pulse-width modulation unit (PWM)?
    IO_WDT_EN            : boolean := true;   -- implement watch dog timer (WDT)?
    IO_TRNG_EN           : boolean := true;   -- implement true random number generator (TRNG)?
    IO_CFS_EN            : boolean := true;   -- implement custom functions subsystem (CFS)?
    IO_NCO_EN            : boolean := true;   -- implement numerically-controlled oscillator (NCO)?
    IO_NEOLED_EN         : boolean := true    -- implement NeoPixel-compatible smart LED interface (NEOLED)?
  );
  port (
    -- host access --
    clk_i  : in  std_ulogic; -- global clock line
    addr_i : in  std_ulogic_vector(31 downto 0); -- address
    rden_i : in  std_ulogic; -- read enable
    data_o : out std_ulogic_vector(31 downto 0); -- data out
    ack_o  : out std_ulogic  -- transfer acknowledge
  );
end neorv32_sysinfo;

architecture neorv32_sysinfo_rtl of neorv32_sysinfo is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(sysinfo_size_c); -- low address boundary bit

  -- access control --
  signal acc_en    : std_ulogic; -- module access enable
  signal addr      : std_ulogic_vector(31 downto 0);
  signal rden      : std_ulogic;
  signal info_addr : std_ulogic_vector(02 downto 0);

  -- system information ROM --
  type info_mem_t is array (0 to 7) of std_ulogic_vector(31 downto 0);
  signal sysinfo_mem : info_mem_t;

begin

  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en    <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = sysinfo_base_c(hi_abb_c downto lo_abb_c)) else '0';
  rden      <= acc_en and rden_i; -- valid read access
  addr      <= sysinfo_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
  info_addr <= addr(index_size_f(sysinfo_size_c)-1 downto 2);


  -- Construct Info ROM ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- SYSINFO(0): Processor (primary) clock frequency --
  sysinfo_mem(0) <= std_ulogic_vector(to_unsigned(CLOCK_FREQUENCY, 32));

  -- SYSINFO(1): Custom user code/ID --
  sysinfo_mem(1) <= USER_CODE;

  -- SYSINFO(2): Implemented processor devices/features --
  -- Memory --
  sysinfo_mem(2)(00) <= bool_to_ulogic_f(BOOTLOADER_EN);     -- processor-internal bootloader implemented?
  sysinfo_mem(2)(01) <= bool_to_ulogic_f(MEM_EXT_EN);        -- external memory bus interface implemented?
  sysinfo_mem(2)(02) <= bool_to_ulogic_f(MEM_INT_IMEM_EN);   -- processor-internal instruction memory implemented?
  sysinfo_mem(2)(03) <= bool_to_ulogic_f(MEM_INT_IMEM_ROM);  -- processor-internal instruction memory implemented as ROM?
  sysinfo_mem(2)(04) <= bool_to_ulogic_f(MEM_INT_DMEM_EN);   -- processor-internal data memory implemented?
  sysinfo_mem(2)(05) <= bool_to_ulogic_f(xbus_big_endian_c); -- is external memory bus interface using BIG-endian byte-order?
  sysinfo_mem(2)(06) <= bool_to_ulogic_f(ICACHE_EN);         -- processor-internal instruction cache implemented?
  --
  sysinfo_mem(2)(15 downto 07) <= (others => '0'); -- reserved
  -- IO --
  sysinfo_mem(2)(16) <= bool_to_ulogic_f(IO_GPIO_EN);   -- general purpose input/output port unit (GPIO) implemented?
  sysinfo_mem(2)(17) <= bool_to_ulogic_f(IO_MTIME_EN);  -- machine system timer (MTIME) implemented?
  sysinfo_mem(2)(18) <= bool_to_ulogic_f(IO_UART0_EN);  -- primary universal asynchronous receiver/transmitter (UART0) implemented?
  sysinfo_mem(2)(19) <= bool_to_ulogic_f(IO_SPI_EN);    -- serial peripheral interface (SPI) implemented?
  sysinfo_mem(2)(20) <= bool_to_ulogic_f(IO_TWI_EN);    -- two-wire interface (TWI) implemented?
  sysinfo_mem(2)(21) <= bool_to_ulogic_f(IO_PWM_EN);    -- pulse-width modulation unit (PWM) implemented?
  sysinfo_mem(2)(22) <= bool_to_ulogic_f(IO_WDT_EN);    -- watch dog timer (WDT) implemented?
  sysinfo_mem(2)(23) <= bool_to_ulogic_f(IO_CFS_EN);    -- custom functions subsystem (CFS) implemented?
  sysinfo_mem(2)(24) <= bool_to_ulogic_f(IO_TRNG_EN);   -- true random number generator (TRNG) implemented?
  sysinfo_mem(2)(25) <= bool_to_ulogic_f(IO_NCO_EN);    -- numerically-controlled oscillator (NCO) implemented?
  sysinfo_mem(2)(26) <= bool_to_ulogic_f(IO_UART1_EN);  -- secondary universal asynchronous receiver/transmitter (UART1) implemented?
  sysinfo_mem(2)(27) <= bool_to_ulogic_f(IO_NEOLED_EN); -- NeoPixel-compatible smart LED interface (NEOLED) implemented?
  --
  sysinfo_mem(2)(31 downto 28) <= (others => '0'); -- reserved

  -- SYSINFO(3): Cache configuration --
  sysinfo_mem(3)(03 downto 00) <= std_ulogic_vector(to_unsigned(index_size_f(ICACHE_BLOCK_SIZE),    4)) when (ICACHE_EN = true) else (others => '0'); -- i-cache: log2(block_size_in_bytes)
  sysinfo_mem(3)(07 downto 04) <= std_ulogic_vector(to_unsigned(index_size_f(ICACHE_NUM_BLOCKS),    4)) when (ICACHE_EN = true) else (others => '0'); -- i-cache: log2(number_of_block)
  sysinfo_mem(3)(11 downto 08) <= std_ulogic_vector(to_unsigned(index_size_f(ICACHE_ASSOCIATIVITY), 4)) when (ICACHE_EN = true) else (others => '0'); -- i-cache: log2(associativity)
  sysinfo_mem(3)(15 downto 12) <= "0001" when (ICACHE_ASSOCIATIVITY > 1) and (ICACHE_EN = true) else (others => '0'); -- i-cache: replacement strategy (LRU only (yet))
  --
  sysinfo_mem(3)(19 downto 16) <= (others => '0'); -- reserved - d-cache: log2(block_size)
  sysinfo_mem(3)(23 downto 20) <= (others => '0'); -- reserved - d-cache: log2(num_blocks)
  sysinfo_mem(3)(27 downto 24) <= (others => '0'); -- reserved - d-cache: log2(associativity)
  sysinfo_mem(3)(31 downto 28) <= (others => '0'); -- reserved - d-cache: replacement strategy

  -- SYSINFO(4): Base address of instruction memory space --
  sysinfo_mem(4) <= ispace_base_c; -- defined in neorv32_package.vhd file

  -- SYSINFO(5): Base address of data memory space --
  sysinfo_mem(5) <= dspace_base_c; -- defined in neorv32_package.vhd file

  -- SYSINFO(6): Size of IMEM in bytes --
  sysinfo_mem(6) <= std_ulogic_vector(to_unsigned(MEM_INT_IMEM_SIZE, 32)) when (MEM_INT_IMEM_EN = true) else (others => '0');

  -- SYSINFO(7): Size of DMEM in bytes --
  sysinfo_mem(7) <= std_ulogic_vector(to_unsigned(MEM_INT_DMEM_SIZE, 32)) when (MEM_INT_DMEM_EN = true) else (others => '0');


  -- Read Access ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      ack_o  <= rden;
      data_o <= (others => '0');
      if (rden = '1') then
        data_o <= sysinfo_mem(to_integer(unsigned(info_addr)));
      end if;
    end if;
  end process read_access;


end neorv32_sysinfo_rtl;
