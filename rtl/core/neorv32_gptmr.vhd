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

entity neorv32_gptmr is
  port (
    clk_i       : in  std_ulogic; -- global clock line
    rstn_i      : in  std_ulogic; -- global reset line, low-active
    bus_req_i   : in  bus_req_t;  -- bus request
    bus_rsp_o   : out bus_rsp_t;  -- bus response
    clkgen_en_o : out std_ulogic; -- enable clock generator
    clkgen_i    : in  std_ulogic_vector(7 downto 0);
    irq_o       : out std_ulogic -- timer match interrupt
  );
end neorv32_gptmr;

architecture neorv32_gptmr_rtl of neorv32_gptmr is

  -- control register --
  constant ctrl_en_c    : natural := 0; -- r/w: timer enable
  constant ctrl_prsc0_c : natural := 1; -- r/w: clock prescaler select bit 0
  constant ctrl_prsc1_c : natural := 2; -- r/w: clock prescaler select bit 1
  constant ctrl_prsc2_c : natural := 3; -- r/w: clock prescaler select bit 2
  constant ctrl_mode_c  : natural := 4; -- r/w: mode (0=single-shot, 1=continuous)
  --
  signal ctrl : std_ulogic_vector(4 downto 0);

  -- timer core --
  type timer_t is record
    count  : std_ulogic_vector(31 downto 0); -- counter register
    thres  : std_ulogic_vector(31 downto 0); -- threshold value
    tick   : std_ulogic; -- clock generator tick
    match  : std_ulogic; -- count == thres
    cnt_we : std_ulogic; -- write access to count
  end record;
  signal timer : timer_t;

begin

  -- Host Access ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- write access --
  write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      timer.cnt_we <= '0';
      ctrl         <= (others => '0');
      timer.thres  <= (others => '0');
    elsif rising_edge(clk_i) then
      timer.cnt_we <= '0'; -- default
      if (bus_req_i.we = '1') then
        if (bus_req_i.addr(3 downto 2) = "00") then -- control register
          ctrl(ctrl_en_c)    <= bus_req_i.data(ctrl_en_c);
          ctrl(ctrl_prsc0_c) <= bus_req_i.data(ctrl_prsc0_c);
          ctrl(ctrl_prsc1_c) <= bus_req_i.data(ctrl_prsc1_c);
          ctrl(ctrl_prsc2_c) <= bus_req_i.data(ctrl_prsc2_c);
          ctrl(ctrl_mode_c)  <= bus_req_i.data(ctrl_mode_c);
        end if;
        if (bus_req_i.addr(3 downto 2) = "01") then -- threshold register
          timer.thres <= bus_req_i.data;
        end if;
        if (bus_req_i.addr(3 downto 2) = "10") then -- counter register
          timer.cnt_we <= '1';
        end if;
      end if;
    end if;
  end process write_access;

  -- read access --
  read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      bus_rsp_o.ack  <= bus_req_i.re or bus_req_i.we; -- bus access acknowledge
      bus_rsp_o.data <= (others => '0');
      if (bus_req_i.re = '1') then
        case bus_req_i.addr(3 downto 2) is
          when "00" => -- control register
            bus_rsp_o.data(ctrl_en_c)    <= ctrl(ctrl_en_c);
            bus_rsp_o.data(ctrl_prsc0_c) <= ctrl(ctrl_prsc0_c);
            bus_rsp_o.data(ctrl_prsc1_c) <= ctrl(ctrl_prsc1_c);
            bus_rsp_o.data(ctrl_prsc2_c) <= ctrl(ctrl_prsc2_c);
            bus_rsp_o.data(ctrl_mode_c)  <= ctrl(ctrl_mode_c);
          when "01" => -- threshold register
            bus_rsp_o.data <= timer.thres;
          when others => -- counter register
            bus_rsp_o.data <= timer.count;
        end case;
      end if;
    end if;
  end process read_access;

  -- no access error possible --
  bus_rsp_o.err <= '0';


  -- Timer Core -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  counter_core: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      timer.count <= (others => '0');
    elsif rising_edge(clk_i) then
      if (timer.cnt_we = '1') then -- write access
        timer.count <= bus_req_i.data; -- data_i will stay unchanged for min. 1 cycle after WREN has returned to low again
      elsif (ctrl(ctrl_en_c) = '1') and (timer.tick = '1') then -- enabled and clock tick
        if (timer.match = '1') then
          if (ctrl(ctrl_mode_c) = '1') then -- reset counter if continuous mode
            timer.count <= (others => '0');
          end if;
        else
          timer.count <= std_ulogic_vector(unsigned(timer.count) + 1);
        end if;
      end if;
    end if;
  end process counter_core;

  -- counter = threshold? --
  timer.match <= '1' when (timer.count = timer.thres) else '0';

  -- clock generator enable --
  clkgen_en_o <= ctrl(ctrl_en_c);

  -- clock select --
  clock_select: process(clk_i)
  begin
    if rising_edge(clk_i) then
      timer.tick <= clkgen_i(to_integer(unsigned(ctrl(ctrl_prsc2_c downto ctrl_prsc0_c))));
    end if;
  end process clock_select;

  -- interrupt --
  irq_generator: process(clk_i)
  begin
    if rising_edge(clk_i) then
      irq_o <= ctrl(ctrl_en_c) and timer.match;
    end if;
  end process irq_generator;


end neorv32_gptmr_rtl;
