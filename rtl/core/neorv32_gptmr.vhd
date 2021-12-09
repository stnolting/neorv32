-- #################################################################################################
-- # << NEORV32 - General Purpose Timer (GPTMR) >>                                                 #
-- # ********************************************************************************************* #
-- # 32-bit timer with configurable clock prescaler. The timer fires an interrupt whenever the     #
-- # counter register value reaches the programmed threshold value. The timer can operate in       #
-- # single-shot mode (count until it reaches THRESHOLD and stop) or in continuous mode (count     #
-- # until it reaches THRESHOLD and auto-reset).                                                   #
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

entity neorv32_gptmr is
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
    -- interrupt --
    irq_o       : out std_ulogic -- transmission done interrupt
  );
end neorv32_gptmr;

architecture neorv32_gptmr_rtl of neorv32_gptmr is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(gptmr_size_c); -- low address boundary bit

  -- control register --
  constant ctrl_en_c    : natural := 0; -- r/w: timer enable
  constant ctrl_prsc0_c : natural := 1; -- r/w: clock prescaler select bit 0
  constant ctrl_prsc1_c : natural := 2; -- r/w: clock prescaler select bit 1
  constant ctrl_prsc2_c : natural := 3; -- r/w: clock prescaler select bit 2
  constant ctrl_mode_c  : natural := 4; -- r/w: mode (0=single-shot, 1=continuous)
  --
  signal ctrl : std_ulogic_vector(4 downto 0);

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- clock generator --
  signal gptmr_clk_en : std_ulogic;

  -- timer core --
  type timer_t is record
    count  : std_ulogic_vector(31 downto 0); -- counter register
    thres  : std_ulogic_vector(31 downto 0); -- threshold value
    match  : std_ulogic; -- count == thres
    cnt_we : std_ulogic; -- write access to count
  end record;
  signal timer : timer_t;

  -- interrupt detector --
  signal irq_detect : std_ulogic_vector(1 downto 0);

begin

  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = gptmr_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= gptmr_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
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
      timer.cnt_we <= '0';
      if (wren = '1') then
        if (addr = gptmr_ctrl_addr_c) then -- control register
          ctrl(ctrl_en_c)    <= data_i(ctrl_en_c);
          ctrl(ctrl_prsc0_c) <= data_i(ctrl_prsc0_c);
          ctrl(ctrl_prsc1_c) <= data_i(ctrl_prsc1_c);
          ctrl(ctrl_prsc2_c) <= data_i(ctrl_prsc2_c);
          ctrl(ctrl_mode_c)  <= data_i(ctrl_mode_c);
        end if;
        if (addr = gptmr_thres_addr_c) then -- threshold register
          timer.thres <= data_i;
        end if;
        if (addr = gptmr_count_addr_c) then -- counter register
          timer.cnt_we <= '1';
        end if;
      end if;

      -- read access --
      data_o <= (others => '0');
      if (rden = '1') then
        case addr(3 downto 2) is
          when "00" => -- control register
            data_o(ctrl_en_c)    <= ctrl(ctrl_en_c);
            data_o(ctrl_prsc0_c) <= ctrl(ctrl_prsc0_c);
            data_o(ctrl_prsc1_c) <= ctrl(ctrl_prsc1_c);
            data_o(ctrl_prsc2_c) <= ctrl(ctrl_prsc2_c);
            data_o(ctrl_mode_c)  <= ctrl(ctrl_mode_c);
          when "01" => -- threshold register
            data_o <= timer.thres;
          when others => -- counter register
            data_o <= timer.count;
        end case;
      end if;
    end if;
  end process rw_access;

  -- clock generator enable --
  clkgen_en_o <= ctrl(ctrl_en_c);

  -- clock select --
  gptmr_clk_en <= clkgen_i(to_integer(unsigned(ctrl(ctrl_prsc2_c downto ctrl_prsc0_c))));


  -- Timer Core -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  timer_core: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (timer.cnt_we = '1') then -- write access
        timer.count <= data_i;
      elsif (ctrl(ctrl_en_c) = '1') and (gptmr_clk_en = '1') then -- enabled and clock tick
        if (timer.match = '1') then
          if (ctrl(ctrl_mode_c) = '1') then -- reset counter if continuous mode
            timer.count <= (others => '0');
          end if;
        else
          timer.count <= std_ulogic_vector(unsigned(timer.count) + 1);
        end if;
      end if;
    end if;
  end process timer_core;

  -- counter = threshold? --
  timer.match <= '1' when (timer.count = timer.thres) else '0';


  -- Interrupt Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_generator: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (ctrl(ctrl_en_c) = '0') then
        irq_detect <= "00";
      else
        irq_detect <= irq_detect(0) & timer.match;
      end if;
    end if;
  end process irq_generator;

  -- IRQ request to CPU --
  irq_o <= '1' when (irq_detect = "01") else '0';


end neorv32_gptmr_rtl;
