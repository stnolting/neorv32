-- #################################################################################################
-- # << NEORV32 - System/Processor Configuration Information Memory (SYSINFO) >>                   #
-- # ********************************************************************************************* #
-- # This unit provides information regarding the NEORV32 processor system configuration -         #
-- # mostly derived from the top's configuration generics.                                         #
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

entity neorv32_sysinfo is
  generic (
    -- General --
    CLOCK_FREQUENCY      : natural; -- clock frequency of clk_i in Hz
    INT_BOOTLOADER_EN    : boolean; -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
    -- Internal instruction memory --
    MEM_INT_IMEM_EN      : boolean; -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE    : natural; -- size of processor-internal instruction memory in bytes
    -- Internal data memory --
    MEM_INT_DMEM_EN      : boolean; -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE    : natural; -- size of processor-internal data memory in bytes
    -- Reservation Set Granularity --
    AMO_RVS_GRANULARITY  : natural; -- size in bytes, has to be a power of 2, min 4
    -- Instruction cache --
    ICACHE_EN            : boolean; -- implement instruction cache
    ICACHE_NUM_BLOCKS    : natural; -- i-cache: number of blocks (min 2), has to be a power of 2
    ICACHE_BLOCK_SIZE    : natural; -- i-cache: block size in bytes (min 4), has to be a power of 2
    ICACHE_ASSOCIATIVITY : natural; -- i-cache: associativity (min 1), has to be a power 2
    -- Data cache --
    DCACHE_EN            : boolean; -- implement data cache
    DCACHE_NUM_BLOCKS    : natural; -- d-cache: number of blocks (min 2), has to be a power of 2
    DCACHE_BLOCK_SIZE    : natural; -- d-cache: block size in bytes (min 4), has to be a power of 2
    -- External memory interface --
    MEM_EXT_EN           : boolean; -- implement external memory bus interface?
    MEM_EXT_BIG_ENDIAN   : boolean; -- byte order: true=big-endian, false=little-endian
    -- On-chip debugger --
    ON_CHIP_DEBUGGER_EN  : boolean; -- implement OCD?
    -- Processor peripherals --
    IO_GPIO_EN           : boolean; -- implement general purpose IO port (GPIO)?
    IO_MTIME_EN          : boolean; -- implement machine system timer (MTIME)?
    IO_UART0_EN          : boolean; -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_UART1_EN          : boolean; -- implement secondary universal asynchronous receiver/transmitter (UART1)?
    IO_SPI_EN            : boolean; -- implement serial peripheral interface (SPI)?
    IO_SDI_EN            : boolean; -- implement serial data interface (SDI)?
    IO_TWI_EN            : boolean; -- implement two-wire interface (TWI)?
    IO_PWM_EN            : boolean; -- implement pulse-width modulation controller (PWM)?
    IO_WDT_EN            : boolean; -- implement watch dog timer (WDT)?
    IO_TRNG_EN           : boolean; -- implement true random number generator (TRNG)?
    IO_CFS_EN            : boolean; -- implement custom functions subsystem (CFS)?
    IO_NEOLED_EN         : boolean; -- implement NeoPixel-compatible smart LED interface (NEOLED)?
    IO_XIRQ_EN           : boolean; -- implement external interrupts controller (XIRQ)?
    IO_GPTMR_EN          : boolean; -- implement general purpose timer (GPTMR)?
    IO_XIP_EN            : boolean; -- implement execute in place module (XIP)?
    IO_ONEWIRE_EN        : boolean; -- implement 1-wire interface (ONEWIRE)?
    IO_DMA_EN            : boolean; -- implement direct memory access controller (DMA)?
    IO_SLINK_EN          : boolean; -- implement stream link interface (SLINK)?
    IO_CRC_EN            : boolean  -- implement cyclic redundancy check unit (CRC)?
  );
  port (
    clk_i     : in  std_ulogic; -- global clock line
    bus_req_i : in  bus_req_t;  -- bus request
    bus_rsp_o : out bus_rsp_t   -- bus response
  );
end neorv32_sysinfo;

architecture neorv32_sysinfo_rtl of neorv32_sysinfo is

  -- helpers --
  constant int_imem_en_c : boolean := MEM_INT_IMEM_EN and boolean(MEM_INT_IMEM_SIZE > 0);
  constant int_dmem_en_c : boolean := MEM_INT_DMEM_EN and boolean(MEM_INT_DMEM_SIZE > 0);

  -- system information ROM --
  type sysinfo_t is array (0 to 3) of std_ulogic_vector(31 downto 0);
  signal sysinfo : sysinfo_t;

begin

  -- Construct Info ROM ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- SYSINFO(0): Processor Clock Frequency --
  sysinfo(0) <= std_ulogic_vector(to_unsigned(CLOCK_FREQUENCY, 32));

  -- SYSINFO(1): Internal Memory Configuration (sizes)
  sysinfo(1)(07 downto 00) <= std_ulogic_vector(to_unsigned(index_size_f(MEM_INT_IMEM_SIZE), 8)); -- log2(IMEM size)
  sysinfo(1)(15 downto 08) <= std_ulogic_vector(to_unsigned(index_size_f(MEM_INT_DMEM_SIZE), 8)); -- log2(DMEM size)
  sysinfo(1)(23 downto 16) <= (others => '0'); -- reserved
  sysinfo(1)(31 downto 24) <= std_ulogic_vector(to_unsigned(index_size_f(AMO_RVS_GRANULARITY), 8)); -- log2(reservation set granularity)

  -- SYSINFO(2): SoC Configuration --
  -- Memory System --
  sysinfo(2)(00) <= bool_to_ulogic_f(INT_BOOTLOADER_EN);   -- processor-internal bootloader implemented?
  sysinfo(2)(01) <= bool_to_ulogic_f(MEM_EXT_EN);          -- external memory bus interface implemented?
  sysinfo(2)(02) <= bool_to_ulogic_f(int_imem_en_c);       -- processor-internal instruction memory implemented?
  sysinfo(2)(03) <= bool_to_ulogic_f(int_dmem_en_c);       -- processor-internal data memory implemented?
  sysinfo(2)(04) <= bool_to_ulogic_f(MEM_EXT_BIG_ENDIAN);  -- is external memory bus interface using BIG-endian byte-order?
  sysinfo(2)(05) <= bool_to_ulogic_f(ICACHE_EN);           -- processor-internal instruction cache implemented?
  sysinfo(2)(06) <= bool_to_ulogic_f(DCACHE_EN);           -- processor-internal data cache implemented?
  -- reserved --
  sysinfo(2)(07) <= '0'; -- reserved
  sysinfo(2)(08) <= '0'; -- reserved
  sysinfo(2)(09) <= '0'; -- reserved
  sysinfo(2)(10) <= '0'; -- reserved
  sysinfo(2)(11) <= '0'; -- reserved
  -- Peripherals/IO --
  sysinfo(2)(12) <= bool_to_ulogic_f(IO_CRC_EN);           -- cyclic redundancy check unit (CRC) implemented?
  sysinfo(2)(13) <= bool_to_ulogic_f(IO_SLINK_EN);         -- stream link interface (SLINK) implemented?
  sysinfo(2)(14) <= bool_to_ulogic_f(IO_DMA_EN);           -- direct memory access controller (DMA) implemented?
  sysinfo(2)(15) <= bool_to_ulogic_f(IO_GPIO_EN);          -- general purpose input/output port unit (GPIO) implemented?
  sysinfo(2)(16) <= bool_to_ulogic_f(IO_MTIME_EN);         -- machine system timer (MTIME) implemented?
  sysinfo(2)(17) <= bool_to_ulogic_f(IO_UART0_EN);         -- primary universal asynchronous receiver/transmitter (UART0) implemented?
  sysinfo(2)(18) <= bool_to_ulogic_f(IO_SPI_EN);           -- serial peripheral interface (SPI) implemented?
  sysinfo(2)(19) <= bool_to_ulogic_f(IO_TWI_EN);           -- two-wire interface (TWI) implemented?
  sysinfo(2)(20) <= bool_to_ulogic_f(IO_PWM_EN);           -- pulse-width modulation unit (PWM) implemented?
  sysinfo(2)(21) <= bool_to_ulogic_f(IO_WDT_EN);           -- watch dog timer (WDT) implemented?
  sysinfo(2)(22) <= bool_to_ulogic_f(IO_CFS_EN);           -- custom functions subsystem (CFS) implemented?
  sysinfo(2)(23) <= bool_to_ulogic_f(IO_TRNG_EN);          -- true random number generator (TRNG) implemented?
  sysinfo(2)(24) <= bool_to_ulogic_f(IO_SDI_EN);           -- serial data interface (SDI) implemented?
  sysinfo(2)(25) <= bool_to_ulogic_f(IO_UART1_EN);         -- secondary universal asynchronous receiver/transmitter (UART1) implemented?
  sysinfo(2)(26) <= bool_to_ulogic_f(IO_NEOLED_EN);        -- NeoPixel-compatible smart LED interface (NEOLED) implemented?
  sysinfo(2)(27) <= bool_to_ulogic_f(IO_XIRQ_EN);          -- external interrupt controller (XIRQ) implemented?
  sysinfo(2)(28) <= bool_to_ulogic_f(IO_GPTMR_EN);         -- general purpose timer (GPTMR) implemented?
  sysinfo(2)(29) <= bool_to_ulogic_f(IO_XIP_EN);           -- execute in place module (XIP) implemented?
  sysinfo(2)(30) <= bool_to_ulogic_f(IO_ONEWIRE_EN);       -- 1-wire interface (ONEWIRE) implemented?
  sysinfo(2)(31) <= bool_to_ulogic_f(ON_CHIP_DEBUGGER_EN); -- on-chip debugger implemented?

  -- SYSINFO(3): Cache Configuration --
  sysinfo(3)(03 downto 00) <= std_ulogic_vector(to_unsigned(index_size_f(ICACHE_BLOCK_SIZE),    4)) when (ICACHE_EN = true) else (others => '0'); -- i-cache: log2(block_size_in_bytes)
  sysinfo(3)(07 downto 04) <= std_ulogic_vector(to_unsigned(index_size_f(ICACHE_NUM_BLOCKS),    4)) when (ICACHE_EN = true) else (others => '0'); -- i-cache: log2(number_of_block)
  sysinfo(3)(11 downto 08) <= std_ulogic_vector(to_unsigned(index_size_f(ICACHE_ASSOCIATIVITY), 4)) when (ICACHE_EN = true) else (others => '0'); -- i-cache: log2(associativity)
  sysinfo(3)(15 downto 12) <= "0001" when (ICACHE_ASSOCIATIVITY > 1) and (ICACHE_EN = true) else (others => '0'); -- i-cache: replacement strategy (LRU only (yet))
  --
  sysinfo(3)(19 downto 16) <= std_ulogic_vector(to_unsigned(index_size_f(DCACHE_BLOCK_SIZE), 4)) when (DCACHE_EN = true) else (others => '0'); -- d-cache: log2(block_size)
  sysinfo(3)(23 downto 20) <= std_ulogic_vector(to_unsigned(index_size_f(DCACHE_NUM_BLOCKS), 4)) when (DCACHE_EN = true) else (others => '0'); -- d-cache: log2(num_blocks)
  sysinfo(3)(27 downto 24) <= (others => '0'); -- d-cache: log2(associativity)
  sysinfo(3)(31 downto 28) <= (others => '0'); -- d-cache: replacement strategy


  -- Read Access ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      bus_rsp_o.ack  <= bus_req_i.re; -- read-only!
      bus_rsp_o.data <= (others => '0');
      if (bus_req_i.re = '1') then
        bus_rsp_o.data <= sysinfo(to_integer(unsigned(bus_req_i.addr(3 downto 2))));
      end if;
    end if;
  end process read_access;

  -- no access error possible --
  bus_rsp_o.err <= '0';


end neorv32_sysinfo_rtl;
