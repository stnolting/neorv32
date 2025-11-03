-- ================================================================================ --
-- NEORV32 SoC - Pulse Width Modulation Controller (PWM)                            --
-- -------------------------------------------------------------------------------- --
-- Provides up to 16 PWM channels; each channel features an individual enable flag, --
-- an 8-bit duty-cycle configuration and a 3-bit clock prescaler + a 16-bit clock   --
-- for programming the channel's sample rate / carrier frequency.                   --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
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
    NUM_CHANNELS : natural range 0 to 16 -- number of PWM channels (0..16)
  );
  port (
    clk_i     : in  std_ulogic; -- global clock line
    rstn_i    : in  std_ulogic; -- global reset line, low-active, async
    bus_req_i : in  bus_req_t;  -- bus request
    bus_rsp_o : out bus_rsp_t;  -- bus response
    clkgen_i  : in  std_ulogic_vector(7 downto 0); -- clock divider input
    pwm_o     : out std_ulogic_vector(15 downto 0) -- PWM output
  );
end neorv32_pwm;

architecture neorv32_pwm_rtl of neorv32_pwm is

  -- pwm channel controller --
  component neorv32_pwm_channel
  port (
    clk_i    : in  std_ulogic; -- global clock line
    rstn_i   : in  std_ulogic; -- global reset line, low-active, async
    en_i     : in  std_ulogic; -- access enable
    rw_i     : in  std_ulogic; -- read/write access
    wdata_i  : in  std_ulogic_vector(31 downto 0); -- write data
    rdata_o  : out std_ulogic_vector(31 downto 0); -- read data
    clkgen_i : in  std_ulogic_vector(7 downto 0); -- clock divider input
    pwm_o    : out std_ulogic -- PWM output
  );
  end component;

  -- wiring --
  type rdata_t is array (0 to NUM_CHANNELS-1) of std_ulogic_vector(31 downto 0);
  signal rdata     : rdata_t;
  signal rdata_sum : std_ulogic_vector(31 downto 0);
  signal sel, pwm  : std_ulogic_vector(NUM_CHANNELS-1 downto 0);

begin

  -- Bus Access -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bus_rsp_o <= rsp_terminate_c;
    elsif rising_edge(clk_i) then
      bus_rsp_o <= rsp_terminate_c;
      if (bus_req_i.stb = '1') then
        bus_rsp_o.data <= rdata_sum;
        bus_rsp_o.ack  <= '1';
      end if;
    end if;
  end process bus_access;

  -- data read-back (large OR) --
  read_back: process(rdata)
    variable tmp_v : std_ulogic_vector(31 downto 0);
  begin
    tmp_v := (others => '0');
    for i in 0 to NUM_CHANNELS-1 loop
      tmp_v := tmp_v or rdata(i);
    end loop;
    rdata_sum <= tmp_v;
  end process read_back;

  -- Channel Controllers --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  pwm_channel_gen:
  for i in 0 to NUM_CHANNELS-1 generate
    neorv32_pwm_channel_inst: neorv32_pwm_channel
    port map (
      clk_i    => clk_i,
      rstn_i   => rstn_i,
      en_i     => sel(i),
      rw_i     => bus_req_i.rw,
      wdata_i  => bus_req_i.data,
      rdata_o  => rdata(i),
      clkgen_i => clkgen_i,
      pwm_o    => pwm(i)
    );
    sel(i) <= bus_req_i.stb when (bus_req_i.addr(5 downto 2) = std_ulogic_vector(to_unsigned(i, 4))) else '0';
  end generate;

  channel_output: process(pwm)
  begin
    pwm_o <= (others => '0');
    pwm_o(NUM_CHANNELS-1 downto 0) <= pwm;
  end process channel_output;

end neorv32_pwm_rtl;


-- ================================================================================ --
-- NEORV32 SoC - PWM - Channel Controller                                           --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity neorv32_pwm_channel is
  port (
    clk_i    : in  std_ulogic; -- global clock line
    rstn_i   : in  std_ulogic; -- global reset line, low-active, async
    en_i     : in  std_ulogic; -- access enable
    rw_i     : in  std_ulogic; -- read/write access
    wdata_i  : in  std_ulogic_vector(31 downto 0); -- write data
    rdata_o  : out std_ulogic_vector(31 downto 0); -- read data
    clkgen_i : in  std_ulogic_vector(7 downto 0); -- clock divider input
    pwm_o    : out std_ulogic -- PWM output
  );
end neorv32_pwm_channel;

architecture neorv32_pwm_channel_rtl of neorv32_pwm_channel is

  -- configuration register --
  signal cfg_en   : std_ulogic; -- channel enable
  signal cfg_prsc : std_ulogic_vector(2 downto 0); -- (course) clock prescaler select
  signal cfg_pol  : std_ulogic; -- channel polarity
  signal cfg_cdiv : std_ulogic_vector(9 downto 0); -- (fine) clock divider
  signal cfg_duty : std_ulogic_vector(7 downto 0); -- duty cycle

  -- pwm core --
  signal cnt_cdiv : std_ulogic_vector(9 downto 0);
  signal cnt_tick : std_ulogic;
  signal cnt_duty : std_ulogic_vector(7 downto 0);

begin

  -- Configuration --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  config_write: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      cfg_en   <= '0';
      cfg_prsc <= (others => '0');
      cfg_pol <= '0';
      cfg_cdiv <= (others => '0');
      cfg_duty <= (others => '0');
    elsif rising_edge(clk_i) then
      if (en_i = '1') and (rw_i = '1') then
        cfg_en   <= wdata_i(31);
        cfg_prsc <= wdata_i(30 downto 28);
        cfg_pol  <= wdata_i(27);
        cfg_cdiv <= wdata_i(17 downto 8);
        cfg_duty <= wdata_i(7 downto 0);
      end if;
    end if;
  end process config_write;

  -- read access --
  rdata_o <= cfg_en & cfg_prsc & cfg_pol & "000000000" & cfg_cdiv & cfg_duty when (en_i = '1') else (others => '0');

  -- PWM Core -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  pwm_core: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      cnt_tick <= '0';
      cnt_cdiv <= (others => '0');
      cnt_duty <= (others => '0');
      pwm_o    <= '0';
    elsif rising_edge(clk_i) then

      -- clock divider --
      cnt_tick <= '0';
      if (cfg_en = '0') then
        cnt_cdiv <= (others => '0');
      elsif (clkgen_i(to_integer(unsigned(cfg_prsc))) = '1') then -- pre-scaled clock
        if (cnt_cdiv = cfg_cdiv) then -- fine-tuned clock
          cnt_cdiv <= (others => '0');
          cnt_tick <= '1'; -- single-shot
        else
          cnt_cdiv <= std_ulogic_vector(unsigned(cnt_cdiv) + 1);
        end if;
      end if;

      -- duty cycle counter --
      if (cfg_en = '0') then
        cnt_duty <= (others => '0');
      elsif (cnt_tick = '1') then
        cnt_duty <= std_ulogic_vector(unsigned(cnt_duty) + 1);
      end if;

      -- pwm output --
      if (cfg_en = '0') or (unsigned(cnt_duty) >= unsigned(cfg_duty)) then
        pwm_o <= cfg_pol;
      else
        pwm_o <= not cfg_pol;
      end if;

    end if;
  end process pwm_core;

end neorv32_pwm_channel_rtl;
