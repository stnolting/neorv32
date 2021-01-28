-- #################################################################################################
-- # << NEORV32 - General Purpose Parallel Input/Output Port (GPIO) >>                             #
-- # ********************************************************************************************* #
-- # 32-bit parallel input & output unit. Any pin change (HI->LO or LO->HI) of an enabled input    #
-- # pin (via irq_en register) triggers an IRQ.                                                    #
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

entity neorv32_gpio is
  port (
    -- host access --
    clk_i  : in  std_ulogic; -- global clock line
    addr_i : in  std_ulogic_vector(31 downto 0); -- address
    rden_i : in  std_ulogic; -- read enable
    wren_i : in  std_ulogic; -- write enable
    data_i : in  std_ulogic_vector(31 downto 0); -- data in
    data_o : out std_ulogic_vector(31 downto 0); -- data out
    ack_o  : out std_ulogic; -- transfer acknowledge
    -- parallel io --
    gpio_o : out std_ulogic_vector(31 downto 0);
    gpio_i : in  std_ulogic_vector(31 downto 0);
    -- interrupt --
    irq_o  : out std_ulogic
  );
end neorv32_gpio;

architecture neorv32_gpio_rtl of neorv32_gpio is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(gpio_size_c); -- low address boundary bit

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address

  -- accessible regs --
  signal din    : std_ulogic_vector(31 downto 0); -- r/-
  signal dout   : std_ulogic_vector(31 downto 0); -- r/w
  signal irq_en : std_ulogic_vector(31 downto 0); -- -/w, uses the same address as data_in

  -- misc --
  signal in_buf : std_ulogic_vector(31 downto 0);

begin

  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = gpio_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= gpio_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned


  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rw_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      ack_o <= acc_en and (rden_i or wren_i);
      -- write access --
      if ((acc_en and wren_i) = '1') then
        if (addr = gpio_in_addr_c) then
          irq_en <= data_i; -- pin change IRQ enable
        else -- gpio_out_addr_c
          dout <= data_i; -- data output port
        end if;
      end if;
      -- read access --
      data_o <= (others => '0');
      if ((acc_en and rden_i) = '1') then
        if (addr = gpio_in_addr_c) then
          data_o <= din; -- data input port
        else -- gpio_out_addr_c
          data_o <= dout; -- data output port
        end if;
      end if;
    end if;
  end process rw_access;

  -- output --
  gpio_o <= dout;


  -- IRQ Detector ------------------------------------------------------------
  -- -----------------------------------------------------------------------------
  irq_detector: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- input synchronizer --
      in_buf <= gpio_i;
      din    <= in_buf;
      -- IRQ --
      irq_o <= or_all_f((in_buf xor din) and irq_en); -- any enabled pin transition triggers an interrupt
    end if;
  end process irq_detector;


end neorv32_gpio_rtl;
