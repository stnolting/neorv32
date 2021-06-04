-- #################################################################################################
-- # << NEORV32 - True Random Number Generator (TRNG) >>                                           #
-- # ********************************************************************************************* #
-- # This unit implements a *true* random number generator which uses several ring oscillators as  #
-- # entropy source. The outputs of all chains are XORed and de-biased using a John von Neumann    #
-- # randomness extractor. The de-biased signal is further processed by a simple LFSR for improved #
-- # whitening.                                                                                    #
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

entity neorv32_trng is
  port (
    -- host access --
    clk_i  : in  std_ulogic; -- global clock line
    addr_i : in  std_ulogic_vector(31 downto 0); -- address
    rden_i : in  std_ulogic; -- read enable
    wren_i : in  std_ulogic; -- write enable
    data_i : in  std_ulogic_vector(31 downto 0); -- data in
    data_o : out std_ulogic_vector(31 downto 0); -- data out
    ack_o  : out std_ulogic  -- transfer acknowledge
  );
end neorv32_trng;

architecture neorv32_trng_rtl of neorv32_trng is

  -- Advanced Configuration --------------------------------------------------------------------------------
  constant num_roscs_c     : natural := 4; -- total number of ring oscillators
  constant num_inv_start_c : natural := 5; -- number of inverters in FIRST ring oscillator (has to be odd)
  constant num_inv_inc_c   : natural := 2; -- number of inverters increment for each next ring oscillator (has to be even)
  constant lfsr_en_c       : boolean := true; -- use LFSR-based post-processing
  constant lfsr_taps_c     : std_ulogic_vector(7 downto 0) := "10111000"; -- Fibonacci post-processing LFSR feedback taps
  -- -------------------------------------------------------------------------------------------------------

  -- control register bits --
  constant ctrl_data_lsb_c : natural :=  0; -- r/-: Random data byte LSB
  constant ctrl_data_msb_c : natural :=  7; -- r/-: Random data byte MSB
  --
  constant ctrl_en_c       : natural := 30; -- r/w: TRNG enable
  constant ctrl_valid_c    : natural := 31; -- r/-: Output data valid

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(trng_size_c); -- low address boundary bit

  -- Component: Ring-Oscillator --
  component neorv32_trng_ring_osc
    generic (
      NUM_INV : natural := 16 -- number of inverters in chain
    );
    port (
      clk_i    : in  std_ulogic;
      enable_i : in  std_ulogic; -- enable chain input
      enable_o : out std_ulogic; -- enable chain output
      data_o   : out std_ulogic  -- sync random bit
    );
  end component;

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal wren   : std_ulogic; -- full word write enable
  signal rden   : std_ulogic; -- read enable

  -- ring-oscillator array --
  signal osc_array_en_in  : std_ulogic_vector(num_roscs_c-1 downto 0);
  signal osc_array_en_out : std_ulogic_vector(num_roscs_c-1 downto 0);
  signal osc_array_data   : std_ulogic_vector(num_roscs_c-1 downto 0);

  -- von-Neumann de-biasing --
  type debiasing_t is record
    sreg  : std_ulogic_vector(1 downto 0);
    state : std_ulogic; -- process de-biasing every second cycle
    valid : std_ulogic; -- de-biased data
    data  : std_ulogic; -- de-biased data valid
  end record;
  signal debiasing : debiasing_t;

  -- (post-)processing core --
  type processing_t is record
    enable : std_ulogic; -- TRNG enable flag
    cnt    : std_ulogic_vector(3 downto 0); -- bit counter
    sreg   : std_ulogic_vector(7 downto 0); -- data shift register
    output : std_ulogic_vector(7 downto 0); -- output register
    valid  : std_ulogic; -- data output valid flag
  end record;
  signal processing : processing_t;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (num_roscs_c = 0) report "NEORV32 PROCESSOR CONFIG ERROR: TRNG - Total number of ring-oscillators has to be >0." severity error;
  assert not ((num_inv_start_c mod 2)  = 0) report "NEORV32 PROCESSOR CONFIG ERROR: TRNG - Number of inverters in fisrt ring has to be odd." severity error;
  assert not ((num_inv_inc_c   mod 2) /= 0) report "NEORV32 PROCESSOR CONFIG ERROR: TRNG - Number of inverters increment for each next ring has to be even." severity error;


  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = trng_base_c(hi_abb_c downto lo_abb_c)) else '0';
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
        processing.enable <= data_i(ctrl_en_c);
      end if;
      -- read access --
      data_o <= (others => '0');
      if (rden = '1') then
        data_o(ctrl_data_msb_c downto ctrl_data_lsb_c) <= processing.output;
        data_o(ctrl_en_c)                              <= processing.enable;
        data_o(ctrl_valid_c)                           <= processing.valid;
      end if;
    end if;
  end process rw_access;


  -- Entropy Source -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_trng_ring_osc_inst:
  for i in 0 to num_roscs_c-1 generate
    neorv32_trng_ring_osc_inst_i: neorv32_trng_ring_osc
    generic map (
      NUM_INV => num_inv_start_c + (i*num_inv_inc_c) -- number of inverters in chain
    )
    port map (
      clk_i    => clk_i,
      enable_i => osc_array_en_in(i),
      enable_o => osc_array_en_out(i),
      data_o   => osc_array_data(i)
    );
  end generate;

  -- RO enable chain --
  array_intercon: process(processing.enable, osc_array_en_out)
  begin
    for i in 0 to num_roscs_c-1 loop
      if (i = 0) then -- start of enable chain
        osc_array_en_in(i) <= processing.enable;
      else
        osc_array_en_in(i) <= osc_array_en_out(i-1);
      end if;
    end loop; -- i
  end process array_intercon;


  -- John von Neumann De-Biasing ------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neumann_debiasing_sync: process(clk_i)
  begin
    if rising_edge(clk_i) then
      debiasing.sreg  <= debiasing.sreg(debiasing.sreg'left-1 downto 0) & xor_reduce_f(osc_array_data);
      debiasing.state <= (not debiasing.state) and osc_array_en_out(num_roscs_c-1); -- start toggling when last RO is enabled -> process in every second cycle
    end if;
  end process neumann_debiasing_sync;

  -- Edge detector --
  neumann_debiasing_comb: process(debiasing)
    variable tmp_v : std_ulogic_vector(2 downto 0);
  begin
    -- check groups of two non-overlapping bits from the input stream
    tmp_v := debiasing.state & debiasing.sreg;
    case tmp_v is
      when "101"  => debiasing.valid <= '1'; debiasing.data <= '1'; -- rising edge  -> '1'
      when "110"  => debiasing.valid <= '1'; debiasing.data <= '0'; -- falling edge -> '0'
      when others => debiasing.valid <= '0'; debiasing.data <= '0'; -- no valid data
    end case;
  end process neumann_debiasing_comb;


  -- Processing Core ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  processing_core: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- sample random data bit and apply post-processing --
      if (processing.enable = '0') then
        processing.cnt  <= (others => '0');
        processing.sreg <= (others => '0');
      elsif (debiasing.valid = '1') then -- valid random sample?
        if (processing.cnt = "1000") then
          processing.cnt <= (others => '0');
        else
          processing.cnt <= std_ulogic_vector(unsigned(processing.cnt) + 1);
        end if;
        if (lfsr_en_c = true) then -- LFSR post-processing
          processing.sreg <= processing.sreg(processing.sreg'left-1 downto 0) & ((not xor_reduce_f(processing.sreg and lfsr_taps_c)) xnor debiasing.data);
        else -- NO post-processing
          processing.sreg <= processing.sreg(processing.sreg'left-1 downto 0) & debiasing.data;
        end if;
      end if;

      -- data output register --
      if (processing.cnt = "1000") then
        processing.output <= processing.sreg;
      end if;

      -- data ready/valid flag --
      if (processing.cnt = "1000") then -- new sample ready?
        processing.valid <= '1';
      elsif (processing.enable = '0') or (rden = '1') then -- clear when deactivated or on data read
        processing.valid <= '0';
      end if;
    end if;
  end process processing_core;


end neorv32_trng_rtl;


-- ############################################################################################################################
-- ############################################################################################################################


-- #################################################################################################
-- # << NEORV32 - True Random Number Generator (TRNG) - Ring-Oscillator-Based Entropy Source >>    #
-- # ********************************************************************************************* #
-- # An inverter chain (ring oscillator) is used as entropy source.                                #
-- # The inverter chain is constructed as an "asynchronous" LFSR. The single inverters are         #
-- # connected via latches that are used to enable/disable the TRNG. Also, these latches are used  #
-- # as additional delay element. By using unique enable signals for each latch, the synthesis     #
-- # tool cannot "optimize" (=remove) any of the inverters out of the design. Furthermore, the     #
-- # latches prevent the synthesis tool from detecting combinatorial loops.                        #
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

entity neorv32_trng_ring_osc is
  generic (
    NUM_INV : natural := 15 -- number of inverters in chain
  );
  port (
    clk_i    : in  std_ulogic;
    enable_i : in  std_ulogic; -- enable chain input
    enable_o : out std_ulogic; -- enable chain output
    data_o   : out std_ulogic  -- sync random bit
  );
end neorv32_trng_ring_osc;

architecture neorv32_trng_ring_osc_rtl of neorv32_trng_ring_osc is

  signal inv_chain   : std_ulogic_vector(NUM_INV-1 downto 0); -- oscillator chain
  signal enable_sreg : std_ulogic_vector(NUM_INV-1 downto 0); -- enable shift register
  signal sync_ff     : std_ulogic_vector(1 downto 0); -- output signal synchronizer

begin

  -- Ring Oscillator ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ring_osc: process(enable_i, enable_sreg, inv_chain)
  begin
    -- Using individual enable signals for each inverter - derived from a shift register - to prevent the synthesis tool
    -- from removing all but one inverter (since they implement "logical identical functions").
    -- This also allows to make the TRNG platform independent.
    for i in 0 to NUM_INV-1 loop -- inverters in chain
      if (enable_i = '0') then -- start with a defined state (latch reset)
        inv_chain(i) <= '0';
      elsif (enable_sreg(i) = '1') then
        -- here we have the inverter chain --
        if (i = NUM_INV-1) then -- left-most inverter?
          inv_chain(i) <= not inv_chain(0);
        else
          inv_chain(i) <= not inv_chain(i+1);
        end if;
      end if;
    end loop; -- i
  end process ring_osc;


  -- Control --------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_unit: process(clk_i)
  begin
    if rising_edge(clk_i) then
      enable_sreg <= enable_sreg(enable_sreg'left-1 downto 0) & enable_i; -- activate right-most inverter first
      sync_ff     <= sync_ff(0) & inv_chain(0); -- synchronize to prevent metastability 
    end if;
  end process ctrl_unit;

  -- output for "enable chain" --
  enable_o <= enable_sreg(enable_sreg'left);

  -- rnd output --
  data_o <= sync_ff(1);


end neorv32_trng_ring_osc_rtl;
