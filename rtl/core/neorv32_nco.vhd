-- #################################################################################################
-- # << NEORV32 - Number-Controlled Oscillator (NCO) >>                                            #
-- # ********************************************************************************************* #
-- # Arbitrary frequency generator based on a number-controlled oscillator (NCO) core with three   #
-- # independent channels. The phase accumulators and the tuning words are 20-bit wide (+1 bit for #
-- # the accumulator to detect overflows). See data sheet for more information.                    #
-- #                                                                                               #
-- # Output frequency for channel i:                                                               #
-- # f_out(i) =  (f_cpu / clk_prsc(i)) * (tuning_word(i) / 2^21) * 0.5                             #
-- # f_cpu       := CPU/processors primary clock                                                   #
-- # clk_prsc    := 3-bit clock prescaler                                                          #
-- # tuning_word := channel's 20-bit tuning word                                                   #
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

entity neorv32_nco is
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
    -- NCO output --
    nco_o       : out std_ulogic_vector(02 downto 0)
  );
end neorv32_nco;

architecture neorv32_nco_rtl of neorv32_nco is

  -- NCO configuration --
  constant phase_accu_width_c : natural := 20; -- bits, min=1, max=as much as you like, default=20
  constant num_channels_c     : natural :=  3; -- NCO channels, max=3

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(nco_size_c); -- low address boundary bit

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wren   : std_ulogic; -- word write access
  signal rden   : std_ulogic; -- read access

  -- control register bits --
  constant ctrl_en_c           : natural :=  0; -- r/w: global NCO enable
  --
  constant ctrl_ch0_mode_c     : natural :=  1; -- r/w: output mode (0=fixed 50% duty cycle; 1=pulse mode)
  constant ctrl_ch0_idle_pol_c : natural :=  2; -- r/w: output idle polarity (0=low, 1=high)
  constant ctrl_ch0_oe_c       : natural :=  3; -- r/w: enable processor output pin
  constant ctrl_ch0_output_c   : natural :=  4; -- r/-: current channel output state
  constant ctrl_ch0_prsc0_c    : natural :=  5; -- r/w: clock prescaler select bit 0
  constant ctrl_ch0_prsc1_c    : natural :=  6; -- r/w: clock prescaler select bit 1
  constant ctrl_ch0_prsc2_c    : natural :=  7; -- r/w: clock prescaler select bit 2
  constant ctrl_ch0_pulse0_c   : natural :=  8; -- r/w: pulse length select bit 0
  constant ctrl_ch0_pulse1_c   : natural :=  9; -- r/w: pulse length select bit 1
  constant ctrl_ch0_pulse2_c   : natural := 10; -- r/w: pulse length select bit 2
  --
  constant ctrl_ch1_mode_c     : natural := 21; -- r/w: output mode (0=fixed 50% duty cycle; 1=pulse mode)
  constant ctrl_ch1_idle_pol_c : natural := 22; -- r/w: output idle polarity (0=low, 1=high)
  constant ctrl_ch1_oe_c       : natural := 23; -- r/w: enable processor output pin
  constant ctrl_ch1_output_c   : natural := 24; -- r/-: current channel output state
  constant ctrl_ch1_prsc0_c    : natural := 25; -- r/w: clock prescaler select bit 0
  constant ctrl_ch1_prsc1_c    : natural := 26; -- r/w: clock prescaler select bit 1
  constant ctrl_ch1_prsc2_c    : natural := 27; -- r/w: clock prescaler select bit 2
  constant ctrl_ch1_pulse0_c   : natural := 28; -- r/w: pulse length select bit 0
  constant ctrl_ch1_pulse1_c   : natural := 29; -- r/w: pulse length select bit 1
  constant ctrl_ch1_pulse2_c   : natural := 20; -- r/w: pulse length select bit 2
  --
  constant ctrl_ch2_mode_c     : natural := 21; -- r/w: output mode (0=fixed 50% duty cycle; 1=pulse mode)
  constant ctrl_ch2_idle_pol_c : natural := 22; -- r/w: output idle polarity (0=low, 1=high)
  constant ctrl_ch2_oe_c       : natural := 23; -- r/w: enable processor output pin
  constant ctrl_ch2_output_c   : natural := 24; -- r/-: current channel output state
  constant ctrl_ch2_prsc0_c    : natural := 25; -- r/w: clock prescaler select bit 0
  constant ctrl_ch2_prsc1_c    : natural := 26; -- r/w: clock prescaler select bit 1
  constant ctrl_ch2_prsc2_c    : natural := 27; -- r/w: clock prescaler select bit 2
  constant ctrl_ch2_pulse0_c   : natural := 28; -- r/w: pulse length select bit 0
  constant ctrl_ch2_pulse1_c   : natural := 29; -- r/w: pulse length select bit 1
  constant ctrl_ch2_pulse2_c   : natural := 30; -- r/w: pulse length select bit 2
  --
  constant ctrl_ch_offset_c    : natural := 10; -- number of bits for each channel
  constant ctrl_size_c         : natural := num_channels_c*ctrl_ch_offset_c+1; -- number of bits in primary control register

  -- accessible regs --
  type tuning_word_t is array (0 to num_channels_c-1) of std_ulogic_vector(phase_accu_width_c-1 downto 0);
  signal tuning_word : tuning_word_t; -- r/w: tuning word channel i
  signal ctrl        : std_ulogic_vector(ctrl_size_c-1 downto 0); -- r/w: control register

  -- nco core --
  type nco_sel_t is array (0 to num_channels_c-1) of std_ulogic_vector(2 downto 0);
  type pulse_cnt_t is array (0 to num_channels_c-1) of std_ulogic_vector(7 downto 0);
  type accu_t is array (0 to num_channels_c-1) of std_ulogic_vector(phase_accu_width_c downto 0); -- +1 bit for overflow detection
  --
  type nco_core_t is record
    -- control --
    enable        : std_ulogic; -- global enable
    prsc_sel      : nco_sel_t;
    pulse_sel     : nco_sel_t;
    idle_pol      : std_ulogic_vector(num_channels_c-1 downto 0);
    output_en     : std_ulogic_vector(num_channels_c-1 downto 0);
    mode          : std_ulogic_vector(num_channels_c-1 downto 0);
    -- NCO core --
    phase_accu    : accu_t;
    clk_tick      : std_ulogic_vector(num_channels_c-1 downto 0);
    ovfl_buf      : std_ulogic_vector(num_channels_c-1 downto 0);
    overflow      : std_ulogic_vector(num_channels_c-1 downto 0); -- phase accu overflow
    trigger       : std_ulogic_vector(num_channels_c-1 downto 0); -- current NCO output level
    -- pulse generator --
    pulse_tick    : std_ulogic_vector(num_channels_c-1 downto 0);
    pulse_trig    : std_ulogic_vector(num_channels_c-1 downto 0);
    pulse_trig_ff : std_ulogic_vector(num_channels_c-1 downto 0);
    pulse_cnt     : pulse_cnt_t;
    pulse_out     : std_ulogic_vector(num_channels_c-1 downto 0);
    -- status --
    output_nxt    : std_ulogic_vector(num_channels_c-1 downto 0);
    output        : std_ulogic_vector(num_channels_c-1 downto 0); -- current NCO output level
  end record;
  signal nco : nco_core_t;

begin

  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = nco_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= nco_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
  wren   <= acc_en and wren_i;
  rden   <= acc_en and rden_i;


  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rw_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      ack_o <= wren or rden;

      -- write access --
      if (wren = '1') then
        -- control register --
        if (addr = nco_ctrl_addr_c) then
          ctrl <= data_i(ctrl'left downto 0);
        end if;
        -- tuning words --
        if (addr = nco_ch0_addr_c) then -- channel 0
          tuning_word(0) <= data_i(phase_accu_width_c-1 downto 0);
        end if;
        if (addr = nco_ch1_addr_c) then -- channel 1
          tuning_word(1) <= data_i(phase_accu_width_c-1 downto 0);
        end if;
        if (addr = nco_ch2_addr_c) then -- channel 2
          tuning_word(2) <= data_i(phase_accu_width_c-1 downto 0);
        end if;
      end if;

      -- read access --
      data_o <= (others => '0');
      if (rden = '1') then
        case addr is
          when nco_ctrl_addr_c =>
            data_o(ctrl'left downto 0) <= ctrl;
            for i in 0 to num_channels_c-1 loop
              data_o(ctrl_ch0_output_c + i*ctrl_ch_offset_c) <= nco.output(i);
            end loop;
          when nco_ch0_addr_c =>
            data_o(phase_accu_width_c-1 downto 0) <= tuning_word(0);
          when nco_ch1_addr_c =>
            data_o(phase_accu_width_c-1 downto 0) <= tuning_word(1);
          when nco_ch2_addr_c =>
            data_o(phase_accu_width_c-1 downto 0) <= tuning_word(2);
          when others =>
            data_o <= (others => '0');
        end case;
      end if;
    end if;
  end process rw_access;

  -- enable external clock generator --
  clkgen_en_o <= nco.enable;

  -- control register --
  primary_control_register:
  for i in 0 to num_channels_c-1 generate
    nco.mode(i)      <= ctrl(ctrl_ch0_mode_c + i*ctrl_ch_offset_c);
    nco.idle_pol(i)  <= ctrl(ctrl_ch0_idle_pol_c + i*ctrl_ch_offset_c);
    nco.output_en(i) <= ctrl(ctrl_ch0_oe_c + i*ctrl_ch_offset_c);
    nco.prsc_sel(i)  <= ctrl(ctrl_ch0_prsc2_c + i*ctrl_ch_offset_c downto ctrl_ch0_prsc0_c + i*ctrl_ch_offset_c);
    nco.pulse_sel(i) <= ctrl(ctrl_ch0_pulse2_c + i*ctrl_ch_offset_c downto ctrl_ch0_pulse0_c + i*ctrl_ch_offset_c);
  end generate; -- i
  nco.enable <= ctrl(ctrl_en_c);

  -- NCO Core -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  nco_core: process(clk_i)
  begin
    if rising_edge(clk_i) then
      for i in 0 to num_channels_c-1 loop
        -- clock select --
        nco.clk_tick(i) <= clkgen_i(to_integer(unsigned(nco.prsc_sel(i))));

        -- phase accu & output trigger --
        if (nco.enable = '0') then -- disabled
          nco.phase_accu(i) <= (others => '0');
          nco.trigger(i)    <= '0';
        else
          if (nco.clk_tick(i) = '1') then -- wait for clock enable tick
            nco.phase_accu(i) <= std_ulogic_vector(unsigned(nco.phase_accu(i)) + unsigned('0' & tuning_word(i)));
          end if;
          if (nco.overflow(i) = '1') then -- toggle NCO output trigger on overflow
            nco.trigger(i) <= not nco.trigger(i);
          end if;
        end if;

        -- buffer for overflow check (edge detector) --
        nco.ovfl_buf(i) <= nco.phase_accu(i)(phase_accu_width_c);
      end loop; -- i
    end if;
  end process nco_core;

  -- phase accu overflow detector --
  overflow_detect:
  for i in 0 to num_channels_c-1 generate
    nco.overflow(i) <= nco.phase_accu(i)(phase_accu_width_c) and (not nco.ovfl_buf(i));
  end generate; -- i


  -- Pulse Generator ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  pulse_generator: process(clk_i)
  begin
    if rising_edge(clk_i) then
      for i in 0 to num_channels_c-1 loop
        -- phase accu trigger -> edge detector --
        nco.pulse_trig_ff(i) <= nco.trigger(i);

        -- pulse counter --
        if (nco.enable = '0') or (nco.mode(i) = '0') or (nco.pulse_trig(i) = '1') then -- disabled or reset
          nco.pulse_cnt(i) <= (others => '0');
        elsif (nco.clk_tick(i) = '1') then 
          nco.pulse_cnt(i) <= std_ulogic_vector(unsigned(nco.pulse_cnt(i)) + 1);
        end if;

        -- pulse generator --
        if (nco.enable = '0') or (nco.mode(i) = '0') then
          nco.pulse_out(i) <= '0';
        elsif (nco.pulse_trig(i) = '1') then -- set on phase accu's trigger (rising edge)
          nco.pulse_out(i) <= '1';
        elsif (nco.pulse_tick(i) = '1') then -- clear after "timeout" from pulse length counter
          nco.pulse_out(i) <= '0';
        end if;
      end loop; -- i
    end if;
  end process pulse_generator;

  -- pulse length select --
  pulse_length_sel:
  for i in 0 to num_channels_c-1 generate
    nco.pulse_tick(i) <= nco.pulse_cnt(i)(to_integer(unsigned(nco.pulse_sel(i))));
  end generate; -- i

  -- pulse-set edge detector --
  trigger_detect:
  for i in 0 to num_channels_c-1 generate
    nco.pulse_trig(i) <= nco.trigger(i) and (not nco.pulse_trig_ff(i));
  end generate; -- i


  -- Output Configuration -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  output_generator: process(clk_i)
  begin
    if rising_edge(clk_i) then
      for i in 0 to num_channels_c-1 loop
        -- polarity configuration --
        if (nco.enable = '1') then -- channel enabled?
          nco.output(i) <= nco.output_nxt(i) xor nco.idle_pol(i); -- apply polarity configuration
        else
          nco.output(i) <= nco.idle_pol(i); -- use *inactive* polarity configuration when disabled
        end if;

        -- output to physical pin --
        nco_o(i) <= nco.output(i) and nco.output_en(i);
      end loop; -- i
    end if;
  end process output_generator;

  -- NCO output mode select --
  nco_output_mode:
  for i in 0 to num_channels_c-1 generate
    nco.output_nxt(i) <= nco.trigger(i) when (nco.mode(i) = '0') else nco.pulse_out(i);
  end generate; -- i


end neorv32_nco_rtl;
