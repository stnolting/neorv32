-- #################################################################################################
-- # << NEORV32 - Pulse Width Modulation Controller (PWM) >>                                       #
-- # ********************************************************************************************* #
-- # Simple PWM controller with 8 bit resolution for the duty cycle and programmable base          #
-- # frequency. The controller supports up to 60 PWM channels.                                     #
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

entity neorv32_pwm is
  generic (
    NUM_CHANNELS : natural -- number of PWM channels (0..60)
  );
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
    pwm_o       : out std_ulogic_vector(NUM_CHANNELS-1 downto 0)
  );
end neorv32_pwm;

architecture neorv32_pwm_rtl of neorv32_pwm is

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
  type pwm_ch_t is array (0 to NUM_CHANNELS-1) of std_ulogic_vector(7 downto 0);
  signal pwm_ch : pwm_ch_t; -- duty cycle (r/w)
  signal enable : std_ulogic; -- enable unit (r/w)
  signal prsc   : std_ulogic_vector(2 downto 0); -- clock prescaler (r/w)

  type pwm_ch_rd_t is array (0 to 60-1) of std_ulogic_vector(7 downto 0);
  signal pwm_ch_rd : pwm_ch_rd_t; -- duty cycle read-back

  -- prescaler clock generator --
  signal prsc_tick : std_ulogic;

  -- pwm core counter --
  signal pwm_cnt : std_ulogic_vector(7 downto 0);

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (NUM_CHANNELS > 60) report "NEORV32 PROCESSOR CONFIG ERROR! <IO.PWM> invalid number of channels! Has to be 0..60.!" severity error;


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
      ack_o <= rden or wren;

      -- write access --
      if (wren = '1') then
        -- control register --
        if (addr = pwm_ctrl_addr_c) then
          enable <= data_i(ctrl_enable_c);
          prsc   <= data_i(ctrl_prsc2_bit_c downto ctrl_prsc0_bit_c);
        end if;
        -- duty cycle registers --
        for i in 0 to NUM_CHANNELS-1 loop -- channel loop
          if (addr(5 downto 2) = std_ulogic_vector(to_unsigned((i/4)+1, 4))) then -- 4 channels per register; add ctrl reg offset
            pwm_ch(i) <= data_i((i mod 4)*8+7 downto (i mod 4)*8+0);
          end if;
        end loop;
      end if;

      -- read access --
      data_o <= (others => '0');
      if (rden = '1') then
        case addr(5 downto 2) is
          when x"0"   => data_o(ctrl_enable_c) <= enable; data_o(ctrl_prsc2_bit_c downto ctrl_prsc0_bit_c) <= prsc;
          when x"1"   => if (NUM_CHANNELS > 0) then data_o <= pwm_ch_rd(3)  & pwm_ch_rd(2)  & pwm_ch_rd(1)  & pwm_ch_rd(0);  else NULL; end if;
          when x"2"   => if (NUM_CHANNELS > 0) then data_o <= pwm_ch_rd(7)  & pwm_ch_rd(6)  & pwm_ch_rd(5)  & pwm_ch_rd(4);  else NULL; end if;
          when x"3"   => if (NUM_CHANNELS > 0) then data_o <= pwm_ch_rd(11) & pwm_ch_rd(10) & pwm_ch_rd(9)  & pwm_ch_rd(8);  else NULL; end if;
          when x"4"   => if (NUM_CHANNELS > 0) then data_o <= pwm_ch_rd(15) & pwm_ch_rd(14) & pwm_ch_rd(13) & pwm_ch_rd(12); else NULL; end if;
          when x"5"   => if (NUM_CHANNELS > 0) then data_o <= pwm_ch_rd(19) & pwm_ch_rd(18) & pwm_ch_rd(17) & pwm_ch_rd(16); else NULL; end if;
          when x"6"   => if (NUM_CHANNELS > 0) then data_o <= pwm_ch_rd(23) & pwm_ch_rd(22) & pwm_ch_rd(21) & pwm_ch_rd(20); else NULL; end if;
          when x"7"   => if (NUM_CHANNELS > 0) then data_o <= pwm_ch_rd(27) & pwm_ch_rd(26) & pwm_ch_rd(25) & pwm_ch_rd(24); else NULL; end if;
          when x"8"   => if (NUM_CHANNELS > 0) then data_o <= pwm_ch_rd(31) & pwm_ch_rd(30) & pwm_ch_rd(29) & pwm_ch_rd(28); else NULL; end if;
          when x"9"   => if (NUM_CHANNELS > 0) then data_o <= pwm_ch_rd(35) & pwm_ch_rd(34) & pwm_ch_rd(33) & pwm_ch_rd(32); else NULL; end if;
          when x"a"   => if (NUM_CHANNELS > 0) then data_o <= pwm_ch_rd(39) & pwm_ch_rd(38) & pwm_ch_rd(37) & pwm_ch_rd(36); else NULL; end if;
          when x"b"   => if (NUM_CHANNELS > 0) then data_o <= pwm_ch_rd(43) & pwm_ch_rd(42) & pwm_ch_rd(41) & pwm_ch_rd(40); else NULL; end if;
          when x"c"   => if (NUM_CHANNELS > 0) then data_o <= pwm_ch_rd(47) & pwm_ch_rd(46) & pwm_ch_rd(45) & pwm_ch_rd(44); else NULL; end if;
          when x"d"   => if (NUM_CHANNELS > 0) then data_o <= pwm_ch_rd(51) & pwm_ch_rd(50) & pwm_ch_rd(49) & pwm_ch_rd(48); else NULL; end if;
          when x"e"   => if (NUM_CHANNELS > 0) then data_o <= pwm_ch_rd(55) & pwm_ch_rd(54) & pwm_ch_rd(53) & pwm_ch_rd(52); else NULL; end if;
          when x"f"   => if (NUM_CHANNELS > 0) then data_o <= pwm_ch_rd(59) & pwm_ch_rd(58) & pwm_ch_rd(57) & pwm_ch_rd(56); else NULL; end if;
          when others => NULL;
        end case;
      end if;
    end if;
  end process wr_access;

  -- duty cycle read-back --
  pwm_dc_rd_gen: process(pwm_ch)
  begin
    pwm_ch_rd <= (others => (others => '0'));
    for i in 0 to NUM_CHANNELS-1 loop
      pwm_ch_rd(i) <= pwm_ch(i);
    end loop;
  end process pwm_dc_rd_gen;

  -- PWM clock select --
  clkgen_en_o <= enable; -- enable clock generator
  prsc_tick   <= clkgen_i(to_integer(unsigned(prsc)));


  -- PWM Core -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  pwm_core: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- pwm base counter --
      if (enable = '0') then 
        pwm_cnt <= (others => '0');
      elsif (prsc_tick = '1') then
        pwm_cnt <= std_ulogic_vector(unsigned(pwm_cnt) + 1);
      end if;

      -- channels --
      for i in 0 to NUM_CHANNELS-1 loop
        if (unsigned(pwm_cnt) >= unsigned(pwm_ch(i))) or (enable = '0') then
          pwm_o(i) <= '0';
        else
          pwm_o(i) <= '1';
        end if;
      end loop;
    end if;
  end process pwm_core;


end neorv32_pwm_rtl;
