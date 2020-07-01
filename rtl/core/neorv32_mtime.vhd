-- #################################################################################################
-- # << NEORV32 - Machine System Timer (MTIME) >>                                                  #
-- # ********************************************************************************************* #
-- # Compatible to RISC-V spec's mtime & mtimecmp.                                                 #
-- # Write mtime.LO first when updating the system time. System time should be written only at     #
-- # system start. RISC-V spec. exception: The MTIME interrupt is ACKed by the processor itself.   #
-- # However, the  achine time cannot issue a new interrupt until the mtimecmp.HI register is      #
-- # written again.                                                                                #
-- # Note: The 64-bit time and compare system is broken and de-coupled into two 32-bit systems.    #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2020, Stephan Nolting. All rights reserved.                                     #
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

entity neorv32_mtime is
  port (
    -- host access --
    clk_i     : in  std_ulogic; -- global clock line
    rstn_i    : in  std_ulogic := '0'; -- global reset, low-active, async
    addr_i    : in  std_ulogic_vector(31 downto 0); -- address
    rden_i    : in  std_ulogic; -- read enable
    wren_i    : in  std_ulogic; -- write enable
    ben_i     : in  std_ulogic_vector(03 downto 0); -- byte write enable
    data_i    : in  std_ulogic_vector(31 downto 0); -- data in
    data_o    : out std_ulogic_vector(31 downto 0); -- data out
    ack_o     : out std_ulogic; -- transfer acknowledge
    -- interrupt --
    irq_o     : out std_ulogic  -- interrupt request
  );
end neorv32_mtime;

architecture neorv32_mtime_rtl of neorv32_mtime is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(mtime_size_c); -- low address boundary bit

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wren   : std_ulogic; -- module access enable

  -- accessible regs --
  signal mtimecmp        : std_ulogic_vector(63 downto 0);
  signal mtime_lo        : std_ulogic_vector(32 downto 0);
  signal mtime_lo_msb_ff : std_ulogic;
  signal mtime_hi        : std_ulogic_vector(31 downto 0);

  -- irq control --
  signal cmp_lo       : std_ulogic;
  signal cmp_lo_ff    : std_ulogic;
  signal cmp_hi       : std_ulogic;
  signal cmp_match_ff : std_ulogic;
  signal irq_flag     : std_ulogic;
  signal irq_flag_ff  : std_ulogic;

begin

  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = mtime_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= mtime_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
  wren   <= acc_en and wren_i;


  -- System Time Update ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  system_time: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (rstn_i = '0') then
        mtime_lo <= (others => '0');
        mtime_hi <= (others => '0');
      else
        -- mtime low --
        mtime_lo <= std_ulogic_vector(unsigned(mtime_lo) + 1);
        mtime_lo_msb_ff <= mtime_lo(mtime_lo'left);
        -- mtime high --
        if ((mtime_lo_msb_ff xor mtime_lo(mtime_lo'left)) = '1') then -- mtime_lo carry?
          mtime_hi <= std_ulogic_vector(unsigned(mtime_hi) + 1);
        end if;
      end if;
    end if;
  end process system_time;


  -- Write Access ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  wr_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      ack_o <= acc_en and (rden_i or wren_i);
      -- mtimecmp low --
      if (wren = '1') and (addr = mtime_cmp_lo_addr_c) then
        for i in 0 to 3 loop
          if (ben_i(i) = '1') then
            mtimecmp(00+7+i*8 downto 00+0+i*8) <= data_i(7+i*8 downto 0+i*8);
          end if;
        end loop; -- byte enable
      end if;

      -- mtimecmp high --
      if (wren = '1') and (addr = mtime_cmp_hi_addr_c) then
        for i in 0 to 3 loop
          if (ben_i(i) = '1') then
            mtimecmp(32+7+i*8 downto 32+0+i*8) <= data_i(7+i*8 downto 0+i*8);
          end if;
        end loop; -- byte enable
      end if;
    end if;
  end process wr_access;


  -- Read Access ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rd_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      data_o <= (others => '0'); -- default
      if (rden_i = '1') and (acc_en = '1') then
        if (addr = mtime_time_lo_addr_c) then -- mtime LOW
          data_o <= mtime_lo(31 downto 00);
        elsif (addr = mtime_time_hi_addr_c) then -- mtime HIGH
          data_o <= mtime_hi;
        elsif (addr = mtime_cmp_lo_addr_c) then -- mtimecmp LOW
          data_o <= mtimecmp(31 downto 00);
        else -- (addr = mtime_cmp_hi_addr_c) then -- mtimecmp HIGH
          data_o <= mtimecmp(63 downto 32);
        end if;
      end if;
    end if;
  end process rd_access;


  -- Comparator -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cmp_sync: process(clk_i)
  begin
    if rising_edge(clk_i) then
      cmp_lo_ff    <= cmp_lo;
      cmp_match_ff <= cmp_lo_ff and cmp_hi;
    end if;
  end process cmp_sync;

  -- test words --
  cmp_lo <= '1' when (unsigned(mtime_lo(31 downto 00)) >= unsigned(mtimecmp(31 downto 00))) else '0';
  cmp_hi <= '1' when (unsigned(mtime_hi(31 downto 00)) >= unsigned(mtimecmp(63 downto 32))) else '0';


  -- Interrupt Logic ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_ctrl: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (rstn_i = '0') then
        irq_flag_ff <= '0';
        irq_flag    <= '0';
      else
        irq_flag_ff  <= irq_flag;
        if (irq_flag = '0') then -- idle
          irq_flag <= '0';
          if (cmp_match_ff = '1') then
            irq_flag <= '1';
          end if;
        elsif (wren = '1') and (addr = mtime_cmp_hi_addr_c) then -- ACK
          irq_flag <= '0';
        end if;
      end if;
    end if;
  end process irq_ctrl;

  -- irq output to CPU --
  irq_o <= irq_flag and (not irq_flag_ff); -- rising edge detector


end neorv32_mtime_rtl;
