-- #################################################################################################
-- # << NEORV32 - Quadrature Decoder (QDEC) >>                                                     #
-- # ********************************************************************************************* #
-- # Quadrature decoder to sample rotary encoders without any software overhead. The decoder       #
-- # supports up to 6 independent channels. Each channel provides it's own 16-bit position counter #
-- # that is increment/decremented according to which input edge is leading (A or B).              #
-- #                                                                                               #
-- # The decoder's sample rate is programmable to adjust to manually operated (slow) or            #
-- # automatically operated (fast) rotary encoders.                                                #
-- #                                                                                               #
-- # Each channel provides an optional state-change interrupt that is triggered if there is any    #
-- # valid movement detected. Furthermore, each channel provides an optional decoder error         #
-- # interrupt that is triggered if an illegal state transition (i.e. both phase signals change at #
-- # the same time) is detected.                                                                   #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
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

entity neorv32_qdec is
  generic (
    QDEC_NUM_CH : natural -- number of channels (0..6)
  );
  port (
    -- host access --
    clk_i       : in  std_ulogic; -- global clock line
    rstn_i      : in  std_ulogic; -- global reset line, low-active
    addr_i      : in  std_ulogic_vector(31 downto 0); -- address
    rden_i      : in  std_ulogic; -- read enable
    wren_i      : in  std_ulogic; -- write enable
    data_i      : in  std_ulogic_vector(31 downto 0); -- data in
    data_o      : out std_ulogic_vector(31 downto 0); -- data out
    ack_o       : out std_ulogic; -- transfer acknowledge
    -- clock generator --
    clkgen_en_o : out std_ulogic; -- enable clock generator
    clkgen_i    : in  std_ulogic_vector(07 downto 0);
    -- quadrature encoder input --
    enc_a_i     : in  std_ulogic_vector(5 downto 0); -- rotary encoder phase A
    enc_b_i     : in  std_ulogic_vector(5 downto 0); -- rotary encoder phase B
    -- state change interrupt --
    irq_o       : out std_ulogic
  );
end neorv32_qdec;

architecture neorv32_qdec_rtl of neorv32_qdec is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(qdec_size_c); -- low address boundary bit

  -- control register --
  constant ctrl_en_c    : natural :=  0; -- r/w: timer enable
  constant ctrl_prsc0_c : natural :=  1; -- r/w: sample clock prescaler select bit 0
  constant ctrl_prsc1_c : natural :=  2; -- r/w: sample clock prescaler select bit 1
  constant ctrl_prsc2_c : natural :=  3; -- r/w: sample clock prescaler select bit 2
  constant ctrl_cirq0_c : natural :=  4; -- r/w: enable state change interrupt for channel 0
  constant ctrl_cirq1_c : natural :=  5; -- r/w: enable state change interrupt for channel 1
  constant ctrl_cirq2_c : natural :=  6; -- r/w: enable state change interrupt for channel 2
  constant ctrl_cirq3_c : natural :=  7; -- r/w: enable state change interrupt for channel 3
  constant ctrl_cirq4_c : natural :=  8; -- r/w: enable state change interrupt for channel 4
  constant ctrl_cirq5_c : natural :=  9; -- r/w: enable state change interrupt for channel 5
  constant ctrl_eirq0_c : natural := 10; -- r/w: enable decoder error interrupt for channel 0
  constant ctrl_eirq1_c : natural := 11; -- r/w: enable decoder error interrupt for channel 1
  constant ctrl_eirq2_c : natural := 12; -- r/w: enable decoder error interrupt for channel 2
  constant ctrl_eirq3_c : natural := 13; -- r/w: enable decoder error interrupt for channel 3
  constant ctrl_eirq4_c : natural := 14; -- r/w: enable decoder error interrupt for channel 4
  constant ctrl_eirq5_c : natural := 15; -- r/w: enable decoder error interrupt for channel 5
  --
  constant ctrl_err0_c  : natural := 26; -- r/c: channel 0 decoder error, sticky, clear-only
  constant ctrl_err1_c  : natural := 27; -- r/c: channel 1 decoder error, sticky, clear-only
  constant ctrl_err2_c  : natural := 28; -- r/c: channel 2 decoder error, sticky, clear-only
  constant ctrl_err3_c  : natural := 29; -- r/c: channel 3 decoder error, sticky, clear-only
  constant ctrl_err4_c  : natural := 30; -- r/c: channel 4 decoder error, sticky, clear-only
  constant ctrl_err5_c  : natural := 31; -- r/c: channel 5 decoder error, sticky, clear-only
  --
  signal enable  : std_ulogic;
  signal prsc    : std_ulogic_vector(2 downto 0);
  signal cirq_en : std_ulogic_vector(QDEC_NUM_CH-1 downto 0);
  signal eirq_en : std_ulogic_vector(QDEC_NUM_CH-1 downto 0);

  -- start-up control --
  signal ready_sreg : std_ulogic_vector(1 downto 0);
  signal ready      : std_ulogic;

  -- decoder error flag --
  signal err      : std_ulogic_vector(QDEC_NUM_CH-1 downto 0);
  signal err_nclr : std_ulogic_vector(QDEC_NUM_CH-1 downto 0);

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- sample clock generator --
  signal prsc_tick      : std_ulogic;
  signal sample_cnt     : std_ulogic_vector(5+1 downto 0);
  signal sample_cnt_msb : std_ulogic;
  signal sample_tick    : std_ulogic; -- final sample clock enable

  -- movement detector --
  type sync_t is array (0 to QDEC_NUM_CH-1) of std_ulogic_vector(1 downto 0);
  type move_t is record
    sync_a    : sync_t;
    sync_b    : sync_t;
    sample_a  : sync_t;
    sample_b  : sync_t;
    valid     : std_ulogic_vector(QDEC_NUM_CH-1 downto 0);
    direction : std_ulogic_vector(QDEC_NUM_CH-1 downto 0);
    error     : std_ulogic_vector(QDEC_NUM_CH-1 downto 0);
  end record;
  signal move : move_t;

  -- position counters --
  type cnt_t is array (0 to QDEC_NUM_CH-1) of std_ulogic_vector(15 downto 0);
  signal cnt : cnt_t;

  -- IRQ generator --
  signal irq_gen : std_ulogic_vector(1 downto 0);

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not ((QDEC_NUM_CH < 0) or (QDEC_NUM_CH > 6)) report
  "NEORV32 PROCESSOR CONFIG ERROR: Number of QDEC input channels <QDEC_NUM_CH> has to be 0..6." severity error;


  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = qdec_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= qdec_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
  wren   <= acc_en and wren_i;
  rden   <= acc_en and rden_i;


  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rw_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      enable   <= '0';
      prsc     <= (others => '0');
      cirq_en  <= (others => '0');
      eirq_en  <= (others => '0');
      --
      err_nclr <= (others => '-');
      data_o   <= (others => '-');
      ack_o    <= '-';
    elsif rising_edge(clk_i) then

      -- write access --
      err_nclr <= (others => '1');
      if (wren = '1') then
        if (addr = qdec_ctrl_addr_c) then
          enable <= data_i(ctrl_en_c);
          prsc   <= data_i(ctrl_prsc2_c downto ctrl_prsc0_c);
          for i in 0 to QDEC_NUM_CH-1 loop
            cirq_en(i)  <= data_i(ctrl_cirq0_c + i);
            eirq_en(i)  <= data_i(ctrl_eirq0_c + i);
            err_nclr(i) <= data_i(ctrl_err0_c  + i); -- clear by writing zero
          end loop; -- i
        end if;
      end if;

      -- read access --
      data_o <= (others => '0');
      if (rden = '1') then
        case addr is
          when qdec_ctrl_addr_c => -- control register
            data_o(ctrl_en_c)                        <= enable;
            data_o(ctrl_prsc2_c downto ctrl_prsc0_c) <= prsc;
            for i in 0 to QDEC_NUM_CH-1 loop
              data_o(ctrl_cirq0_c + i) <= cirq_en(i);
              data_o(ctrl_eirq0_c + i) <= eirq_en(i);
              data_o(ctrl_err0_c  + i) <= err(i);
            end loop; -- i
          when qdec_cnt0_addr_c => -- position counter channels 0 & 1
            if (QDEC_NUM_CH > 0) then data_o(15 downto 00) <= cnt(0); else NULL; end if;
            if (QDEC_NUM_CH > 1) then data_o(31 downto 16) <= cnt(1); else NULL; end if;
          when qdec_cnt1_addr_c => -- position counter channels 2 & 3
            if (QDEC_NUM_CH > 2) then data_o(15 downto 00) <= cnt(2); else NULL; end if;
            if (QDEC_NUM_CH > 3) then data_o(31 downto 16) <= cnt(3); else NULL; end if;
          when qdec_cnt2_addr_c => -- position counter channels 4 & 5
            if (QDEC_NUM_CH > 4) then data_o(15 downto 00) <= cnt(4); else NULL; end if;
            if (QDEC_NUM_CH > 5) then data_o(31 downto 16) <= cnt(5); else NULL; end if;
          when others => -- undefined
            NULL;
        end case;
      end if;

      -- bus handshake --
      ack_o <= wren or rden;
 
    end if;
  end process rw_access;


  -- Sample Clock Generator -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  sample_clock: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- scaler --
      if (enable = '0') then
        sample_cnt <= (others => '0');
      elsif (prsc_tick = '1') then
        sample_cnt <= std_ulogic_vector(unsigned(sample_cnt) + 1);
      end if;
      -- tick generator --
      sample_cnt_msb <= sample_cnt(sample_cnt'left);
      sample_tick    <= sample_cnt(sample_cnt'left) and (not sample_cnt_msb); -- rising edge detect
    end if;
  end process sample_clock;

  -- clock generator enable --
  clkgen_en_o <= enable;

  -- clock generator tick --
  prsc_tick <= clkgen_i(to_integer(unsigned(prsc)));


  -- Movement Detector ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  move_detect_sync: process(clk_i)
  begin
    if rising_edge(clk_i) then
      for i in 0 to QDEC_NUM_CH-1 loop
        -- input synchronizer to prevent metastability --
        move.sync_a(i) <= move.sync_a(i)(0) & enc_a_i(i);
        move.sync_b(i) <= move.sync_b(i)(0) & enc_b_i(i);
        -- sampling --
        move.sample_a(i)(1) <= move.sample_a(i)(0);
        move.sample_b(i)(1) <= move.sample_b(i)(0);
        if (enable = '0') then
          move.sample_a(i)(0) <= '0';
          move.sample_a(i)(0) <= '0';
        elsif (sample_tick = '1') then
          move.sample_a(i)(0) <= move.sync_a(i)(1);
          move.sample_b(i)(0) <= move.sync_b(i)(1);
        end if;
      end loop; -- i
    end if;
  end process move_detect_sync;

  -- state decoding --
  move_detect_comb: process(move)
    variable tmp_v : std_ulogic_vector(3 downto 0);
  begin
    for i in 0 to QDEC_NUM_CH-1 loop
      tmp_v := move.sample_a(i)(0) & move.sample_b(i)(0) & move.sample_a(i)(1) & move.sample_b(i)(1);
      case tmp_v is -- AB_current | AB_previous - gray encoding!
        when "0000" => move.direction(i) <= '0'; move.valid(i) <= '0'; move.error(i) <= '0';
        when "0001" => move.direction(i) <= '1'; move.valid(i) <= '1'; move.error(i) <= '0';
        when "0010" => move.direction(i) <= '0'; move.valid(i) <= '1'; move.error(i) <= '0';
        when "0011" => move.direction(i) <= '0'; move.valid(i) <= '0'; move.error(i) <= '1';
        when "0100" => move.direction(i) <= '0'; move.valid(i) <= '1'; move.error(i) <= '0';
        when "0101" => move.direction(i) <= '0'; move.valid(i) <= '0'; move.error(i) <= '0';
        when "0110" => move.direction(i) <= '0'; move.valid(i) <= '0'; move.error(i) <= '1';
        when "0111" => move.direction(i) <= '1'; move.valid(i) <= '1'; move.error(i) <= '0';
        when "1000" => move.direction(i) <= '1'; move.valid(i) <= '1'; move.error(i) <= '0';
        when "1001" => move.direction(i) <= '0'; move.valid(i) <= '0'; move.error(i) <= '1';
        when "1010" => move.direction(i) <= '0'; move.valid(i) <= '0'; move.error(i) <= '0';
        when "1011" => move.direction(i) <= '0'; move.valid(i) <= '1'; move.error(i) <= '0';
        when "1100" => move.direction(i) <= '0'; move.valid(i) <= '0'; move.error(i) <= '1';
        when "1101" => move.direction(i) <= '0'; move.valid(i) <= '1'; move.error(i) <= '0';
        when "1110" => move.direction(i) <= '1'; move.valid(i) <= '1'; move.error(i) <= '0';
        when "1111" => move.direction(i) <= '0'; move.valid(i) <= '0'; move.error(i) <= '0';
        when others => move.direction(i) <= '0'; move.valid(i) <= '0'; move.error(i) <= '0';
      end case;
    end loop; -- i
  end process move_detect_comb;


  -- Start-Up -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  start_up: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- wait for two sample ticks before we start "real" operation
      -- this will prevent false interrupts, positions changes and errors
      if (enable = '0') then
        ready_sreg <= "00";
      elsif (sample_tick = '1') then
        ready_sreg <= ready_sreg(0) & '1';
      end if;
    end if;
  end process start_up;

  -- system ready? --
  ready <= '1' when (ready_sreg = "11") else '0';


  -- Position Update ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  position_update: process(clk_i)
  begin
    if rising_edge(clk_i) then
      for i in 0 to QDEC_NUM_CH-1 loop
        if (ready = '0') then
          cnt(i) <= (others => '0');
          err(i) <= '0';
        else
          -- channel position counter --
          if (move.valid(i) = '1') then
            if (move.direction(i) = '1') then -- increment
              cnt(i) <= std_ulogic_vector(unsigned(cnt(i)) + 1);
            else -- decrement
              cnt(i) <= std_ulogic_vector(unsigned(cnt(i)) - 1);
            end if;
          end if;
          -- channel decoder error flag (sticky) --
          err(i) <= (err(i) or move.error(i)) and err_nclr(i);
        end if;
      end loop; -- i
    end if;
  end process position_update;


  -- Interrupt Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_generator: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (ready = '0') then
        irq_gen <= "00";
      else
        irq_gen(1) <= irq_gen(0);
        irq_gen(0) <= or_reduce_f(cirq_en and move.valid) or -- state-change interrupt
                      or_reduce_f(eirq_en and move.error); -- decoder error interrupt
      end if;
    end if;
  end process irq_generator;

  -- CPU interrupt request --
  irq_o <= '1' when (irq_gen = "01") else '0';


end neorv32_qdec_rtl;
