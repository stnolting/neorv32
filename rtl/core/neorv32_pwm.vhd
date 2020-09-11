-- #################################################################################################
-- # << NEORV32 - Pulse Width Modulation Controller (PWM) >>                                       #
-- # ********************************************************************************************* #
-- # Simple 4-channel PWM controller with 8 bit resolution for the duty cycle and programmable     #
-- # clock.                                                                                        #
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

entity neorv32_pwm is
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
    -- pwm output channels --
    pwm_o       : out std_ulogic_vector(03 downto 0)
  );
end neorv32_pwm;

architecture neorv32_pwm_rtl of neorv32_pwm is

  -- internal configuration --
  constant num_pwm_channels_c : natural := 4; -- number of PWM channels, fixed!

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(pwm_size_c); -- low address boundary bit

  -- Control register bits --
  constant ctrl_enable_c    : natural := 0; -- r/w: PWM enable
  constant ctrl_prsc0_bit_c : natural := 1; -- r/w: prescaler select bit 0
  constant ctrl_prsc1_bit_c : natural := 2; -- r/w: prescaler select bit 1
  constant ctrl_prsc2_bit_c : natural := 3; -- r/w: prescaler select bit 2

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wren   : std_ulogic; -- write enable
  signal rden   : std_ulogic; -- read enable

  -- accessible regs --
  type pwm_ch_t is array (0 to num_pwm_channels_c-1) of std_ulogic_vector(7 downto 0);
  signal pwm_ch : pwm_ch_t; -- duty cycle (r/w)
  signal enable : std_ulogic; -- enable unit (r/w)
  signal prsc   : std_ulogic_vector(2 downto 0); -- clock prescaler (r/w)

  -- prescaler clock generator --
  signal prsc_tick : std_ulogic;

  -- pwm core counter --
  signal pwm_cnt : std_ulogic_vector(7 downto 0);

begin

  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = pwm_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= pwm_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
  rden   <= acc_en and rden_i;
  wren   <= acc_en and wren_i;


  -- Write access ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  wr_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      ack_o <= acc_en and (rden_i or wren_i);
      -- write access --
      if (wren = '1') then
        if (addr = pwm_ctrl_addr_c) then -- control register
          enable <= data_i(ctrl_enable_c);
          prsc   <= data_i(ctrl_prsc2_bit_c downto ctrl_prsc0_bit_c);
        end if;
        if (addr = pwm_duty_addr_c) then -- duty cycle register
          for i in 0 to 3 loop
            pwm_ch(i) <= data_i(7+i*8 downto 0+i*8);
          end loop;
        end if;
      end if;
      -- read access --
      data_o <= (others => '0');
      if (rden = '1') then
        if (addr = pwm_ctrl_addr_c) then
          data_o(ctrl_enable_c) <= enable;
          data_o(ctrl_prsc2_bit_c downto ctrl_prsc0_bit_c) <= prsc;
        else -- pwm_duty_addr_c
          data_o(07 downto 00) <= pwm_ch(0);
          data_o(15 downto 08) <= pwm_ch(1);
          data_o(23 downto 16) <= pwm_ch(2);
          data_o(31 downto 24) <= pwm_ch(3);
        end if;
      end if;
    end if;
  end process wr_access;

  -- PWM clock select --
  clkgen_en_o <= enable; -- enable clock generator
  prsc_tick   <= clkgen_i(to_integer(unsigned(prsc)));


  -- PWM Core -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  pwm_core: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- pwm counter --
      if (enable = '0') then 
        pwm_cnt <= (others => '0');
      elsif (prsc_tick = '1') then
        pwm_cnt <= std_ulogic_vector(unsigned(pwm_cnt) + 1);
      end if;
      -- channels --
      for i in 0 to num_pwm_channels_c-1 loop
        if (unsigned(pwm_cnt) >= unsigned(pwm_ch(i))) or (enable = '0') then
          pwm_o(i) <= '0';
        else
          pwm_o(i) <= '1';
        end if;
      end loop; -- i, pwm channel
    end if;
  end process pwm_core;


end neorv32_pwm_rtl;
