-- ================================================================================ --
-- NEORV32 SoC - Pulse Width Modulation Controller (PWM)                            --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_pwm is
  generic (
    NUM_CHANNELS : natural range 0 to 12 -- number of PWM channels (0..12)
  );
  port (
    clk_i       : in  std_ulogic; -- global clock line
    rstn_i      : in  std_ulogic; -- global reset line, low-active, async
    bus_req_i   : in  bus_req_t;  -- bus request
    bus_rsp_o   : out bus_rsp_t;  -- bus response
    clkgen_en_o : out std_ulogic; -- enable clock generator
    clkgen_i    : in  std_ulogic_vector(7 downto 0); -- clock divider input
    pwm_o       : out std_ulogic_vector(11 downto 0)  -- PWM output
  );
end neorv32_pwm;

architecture neorv32_pwm_rtl of neorv32_pwm is

  -- Control register bits --
  constant ctrl_enable_c    : natural := 0; -- r/w: PWM enable
  constant ctrl_prsc0_bit_c : natural := 1; -- r/w: prescaler select bit 0
  constant ctrl_prsc1_bit_c : natural := 2; -- r/w: prescaler select bit 1
  constant ctrl_prsc2_bit_c : natural := 3; -- r/w: prescaler select bit 2

  -- accessible regs --
  type pwm_ch_t is array (0 to 11) of std_ulogic_vector(7 downto 0);
  signal pwm_ch : pwm_ch_t; -- duty cycle (r/w)
  signal enable : std_ulogic; -- enable unit (r/w)
  signal prsc   : std_ulogic_vector(2 downto 0); -- clock prescaler (r/w)

  type pwm_ch_rd_t is array (0 to 11) of std_ulogic_vector(7 downto 0);
  signal pwm_ch_rd : pwm_ch_rd_t; -- duty cycle read-back

  -- prescaler clock generator --
  signal prsc_tick : std_ulogic;

  -- pwm core counter --
  signal pwm_cnt : std_ulogic_vector(7 downto 0);

begin

  -- Bus Access -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bus_rsp_o <= rsp_terminate_c;
      enable    <= '0';
      prsc      <= (others => '0');
      pwm_ch    <= (others => (others => '0'));
    elsif rising_edge(clk_i) then
      -- bus handshake --
      bus_rsp_o.ack  <= bus_req_i.stb;
      bus_rsp_o.err  <= '0';
      bus_rsp_o.data <= (others => '0');
      if (bus_req_i.stb = '1') then

        -- write access --
        if (bus_req_i.rw = '1') then
          -- control register --
          if (bus_req_i.addr(3 downto 2) = "00") then
            enable <= bus_req_i.data(ctrl_enable_c);
            prsc   <= bus_req_i.data(ctrl_prsc2_bit_c downto ctrl_prsc0_bit_c);
          end if;
          -- duty cycle register 0 --
          if (bus_req_i.addr(3 downto 2) = "01") then
            pwm_ch(00) <= bus_req_i.data(07 downto 00);
            pwm_ch(01) <= bus_req_i.data(15 downto 08);
            pwm_ch(02) <= bus_req_i.data(23 downto 16);
            pwm_ch(03) <= bus_req_i.data(31 downto 24);
          end if;
          -- duty cycle register 1 --
          if (bus_req_i.addr(3 downto 2) = "10") then
            pwm_ch(04) <= bus_req_i.data(07 downto 00);
            pwm_ch(05) <= bus_req_i.data(15 downto 08);
            pwm_ch(06) <= bus_req_i.data(23 downto 16);
            pwm_ch(07) <= bus_req_i.data(31 downto 24);
          end if;
          -- duty cycle register 2 --
          if (bus_req_i.addr(3 downto 2) = "11") then
            pwm_ch(08) <= bus_req_i.data(07 downto 00);
            pwm_ch(09) <= bus_req_i.data(15 downto 08);
            pwm_ch(10) <= bus_req_i.data(23 downto 16);
            pwm_ch(11) <= bus_req_i.data(31 downto 24);
          end if;

        -- read access --
        else
          case bus_req_i.addr(3 downto 2) is
            when "00"   => bus_rsp_o.data(ctrl_enable_c) <= enable; bus_rsp_o.data(ctrl_prsc2_bit_c downto ctrl_prsc0_bit_c) <= prsc;
            when "01"   => bus_rsp_o.data <= pwm_ch_rd(03) & pwm_ch_rd(02) & pwm_ch_rd(01) & pwm_ch_rd(00);
            when "10"   => bus_rsp_o.data <= pwm_ch_rd(07) & pwm_ch_rd(06) & pwm_ch_rd(05) & pwm_ch_rd(04);
            when "11"   => bus_rsp_o.data <= pwm_ch_rd(11) & pwm_ch_rd(10) & pwm_ch_rd(09) & pwm_ch_rd(08);
            when others => bus_rsp_o.data <= (others => '0');
          end case;
        end if;

      end if;
    end if;
  end process bus_access;

  -- duty cycle read-back --
  pwm_dc_rd_gen: process(pwm_ch)
  begin
    pwm_ch_rd <= (others => (others => '0'));
    for i in 0 to NUM_CHANNELS-1 loop -- only implement the actually configured number of channel register
      pwm_ch_rd(i) <= pwm_ch(i);
    end loop;
  end process pwm_dc_rd_gen;


  -- PWM Core -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  pwm_core: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      pwm_cnt <= (others => '0');
      pwm_o   <= (others => '0');
    elsif rising_edge(clk_i) then
      -- pwm base counter --
      if (enable = '0') then
        pwm_cnt <= (others => '0');
      elsif (prsc_tick = '1') then
        pwm_cnt <= std_ulogic_vector(unsigned(pwm_cnt) + 1);
      end if;
      -- channels --
      pwm_o <= (others => '0');
      for i in 0 to NUM_CHANNELS-1 loop
        if (unsigned(pwm_cnt) >= unsigned(pwm_ch(i))) or (enable = '0') then
          pwm_o(i) <= '0';
        else
          pwm_o(i) <= '1';
        end if;
      end loop;
    end if;
  end process pwm_core;

  -- PWM clock select --
  clkgen_en_o <= enable; -- enable clock generator
  prsc_tick   <= clkgen_i(to_integer(unsigned(prsc)));


end neorv32_pwm_rtl;
