-- #################################################################################################
-- # << NEORV32 - True Random Number Generator (TRNG) >>                                           #
-- # ********************************************************************************************* #
-- # This unit implements a true random number generator which uses several GARO chain as entropy  #
-- # source. The outputs of all chains are XORed and de-biased using a John von Neumann randomness #
-- # extractor. The output is further post-processed by a simple LFSR for improved whitening.      #
-- #                                                                                               #
-- # Sources:                                                                                      #
-- #  - Von Neumann De-Biasing: "Iterating Von Neumann's Post-Processing under Hardware            #
-- #    Constraints" by Vladimir Rozic, Bohan Yang, Wim Dehaene and Ingrid Verbauwhede, 2016       #
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

  -- advanced configuration --------------------------------------------------------------------------------
  constant num_inv_c   : natural := 15; -- length of GARO inverter chain (default=15, has to be odd)
  constant num_garos_c : natural := 2; -- number of GARO elements (default=2)
  constant lfsr_taps_c : std_ulogic_vector(7 downto 0) := "10111000"; -- Fibonacci post-processing LFSR feedback taps
  constant lfsr_en_c   : boolean := true; -- use LFSR-based post-processing
  type tap_mask_t is array (0 to num_garos_c-1) of std_ulogic_vector(num_inv_c-2 downto 0);
  constant tap_mask : tap_mask_t := ( -- GARO tap mask(s), number of set bits has to be even
    "11110000000000", -- "slow" osc
    "11001100110000"  -- "fast" osc
  );
  -- -------------------------------------------------------------------------------------------------------

  -- control register bits --
  constant ctrl_data_lsb_c   : natural :=  0; -- r/-: Random data bit LSB
  constant ctrl_data_msb_c   : natural :=  7; -- r/-: Random data bit MSB
  constant ctrl_data_valid_c : natural := 15; -- r/-: Output data valid
  constant ctrl_err_zero_c   : natural := 16; -- r/-: stuck at 0 error
  constant ctrl_err_one_c    : natural := 17; -- r/-: stuck at 1 error
  constant ctrl_en_c         : natural := 31; -- r/w: TRNG enable

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(trng_size_c); -- low address boundary bit

  -- Component: GARO Element --
  component neorv32_trng_garo_element
    generic (
      NUM_INV : natural := 16 -- number of inverters in chain
    );
    port (
      clk_i    : in  std_ulogic;
      enable_i : in  std_ulogic;
      enable_o : out std_ulogic;
      mask_i   : in  std_ulogic_vector(NUM_INV-2 downto 0);
      data_o   : out std_ulogic;
      error0_o : out std_ulogic;
      error1_o : out std_ulogic
    );
  end component;

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wren   : std_ulogic; -- full word write enable
  signal rden   : std_ulogic; -- read enable

  -- garo array --
  signal garo_en_in    : std_ulogic_vector(num_garos_c-1 downto 0);
  signal garo_en_out   : std_ulogic_vector(num_garos_c-1 downto 0);
  signal garo_data     : std_ulogic_vector(num_garos_c-1 downto 0);
  signal garo_err_zero : std_ulogic_vector(num_garos_c-1 downto 0);
  signal garo_err_one  : std_ulogic_vector(num_garos_c-1 downto 0);
  signal garo_res      : std_ulogic;
  signal garo_err0     : std_ulogic;
  signal garo_err1     : std_ulogic;

  -- de-biasing --
  signal db_data     : std_ulogic_vector(2 downto 0);
  signal db_state    : std_ulogic; -- process de-biasing every second cycle
  signal rnd_valid   : std_ulogic;
  signal rnd_data    : std_ulogic;

  -- processing core --
  signal rnd_enable : std_ulogic;
  signal rnd_cnt    : std_ulogic_vector(3 downto 0);
  signal rnd_sreg   : std_ulogic_vector(7 downto 0);
  signal rnd_output : std_ulogic_vector(7 downto 0);
  signal rnd_ready  : std_ulogic;

  -- health check --
  signal rnd_error_zero : std_ulogic; -- stuck at zero
  signal rnd_error_one  : std_ulogic; -- stuck at one

begin

  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = trng_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= trng_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
  wren   <= acc_en and wren_i;
  rden   <= acc_en and rden_i;


  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rw_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      ack_o <= acc_en and (rden_i or wren_i);
      -- write access --
      if (wren = '1') and (addr = trng_ctrl_addr_c) then
        rnd_enable <= data_i(ctrl_en_c);
      end if;
      -- read access --
      data_o <= (others => '0');
      if (rden = '1') and (addr = trng_ctrl_addr_c) then
        data_o(ctrl_data_msb_c downto ctrl_data_lsb_c) <= rnd_output;
        data_o(ctrl_data_valid_c) <= rnd_ready;
        data_o(ctrl_err_zero_c)   <= rnd_error_zero;
        data_o(ctrl_err_one_c)    <= rnd_error_one;
        data_o(ctrl_en_c)         <= rnd_enable;
      end if;
    end if;
  end process rw_access;


  -- Entropy Source -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_trng_garo_element_inst:
  for i in 0 to num_garos_c-1 generate
    neorv32_trng_garo_element_inst_i: neorv32_trng_garo_element
    generic map (
      NUM_INV => num_inv_c -- number of inverters in chain
    )
    port map (
      clk_i    => clk_i,
      enable_i => garo_en_in(i),
      enable_o => garo_en_out(i),
      mask_i   => tap_mask(i),
      data_o   => garo_data(i),
      error0_o => garo_err_zero(i),
      error1_o => garo_err_one(i)
    );
  end generate;

  -- GARO element connection --
  garo_intercon: process(rnd_enable, garo_en_out, garo_data, garo_err_zero, garo_err_one)
    variable data_v : std_ulogic;
    variable err0_v : std_ulogic;
    variable err1_v : std_ulogic;
  begin
    -- enable chain --
    for i in 0 to num_garos_c-1 loop
      if (i = 0) then
        garo_en_in(i) <= rnd_enable;
      else
        garo_en_in(i) <= garo_en_out(i-1);
      end if;
    end loop; -- i
    -- data & status --
    data_v := garo_data(0);
    err0_v := garo_err_zero(0);
    err1_v := garo_err_one(0);
    for i in 1 to num_garos_c-1 loop
      data_v := data_v xor garo_data(i);
      err0_v := err0_v or garo_err_zero(i);
      err1_v := err1_v or garo_err_one(i);
    end loop; -- i
    garo_res  <= data_v;
    garo_err0 <= err0_v;
    garo_err1 <= err1_v;
  end process garo_intercon;


  -- De-Biasing -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  jvn_debiasing_sync: process(clk_i)
  begin
    if rising_edge(clk_i) then
      db_data  <= db_data(db_data'left-1 downto 0) & garo_res;
      db_state <= (not db_state) and rnd_enable; -- just toggle when enabled -> process in every second cycle
    end if;
  end process jvn_debiasing_sync;


  -- John von Neumann De-Biasing --
  jvn_debiasing: process(db_state, db_data)
    variable tmp_v : std_ulogic_vector(2 downto 0);
  begin
    -- check groups of two non-overlapping bits from the input stream
    tmp_v := db_state & db_data(db_data'left downto db_data'left-1);
    case tmp_v is
      when "101"  => rnd_valid <= '1'; rnd_data <= '1'; -- rising edge -> '1'
      when "110"  => rnd_valid <= '1'; rnd_data <= '0'; -- falling edge -> '0'
      when others => rnd_valid <= '0'; rnd_data <= '-'; -- invalid
    end case;
  end process jvn_debiasing;


  -- Processing Core ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  processing_core: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- sample random data and apply post-processing --
      if (rnd_enable = '0') then
        rnd_cnt  <= (others => '0');
        rnd_sreg <= (others => '0');
      elsif (rnd_valid = '1') and (garo_en_out(garo_en_out'left) = '1') then -- valid random sample and GAROs ready?
        if (rnd_cnt = "1000") then
          rnd_cnt <= (others => '0');
        else
          rnd_cnt <= std_ulogic_vector(unsigned(rnd_cnt) + 1);
        end if;
        if (lfsr_en_c = true) then -- LFSR post-processing
          rnd_sreg <= rnd_sreg(rnd_sreg'left-1 downto 0) & (xnor_all_f(rnd_sreg and lfsr_taps_c) xnor rnd_data);
        else -- NO post-processing
          rnd_sreg <= rnd_sreg(rnd_sreg'left-1 downto 0) & rnd_data;
        end if;
      end if;

      -- data output register --
      if (rnd_cnt = "1000") then
        rnd_output <= rnd_sreg;
      end if;

      -- health check error --
      if (rnd_enable = '0') then
        rnd_error_zero <= '0';
        rnd_error_one  <= '0';
      else
        rnd_error_zero <= rnd_error_zero or garo_err0;
        rnd_error_one  <= rnd_error_one  or garo_err1;
      end if;

      -- data ready flag --
      if (rnd_cnt = "1000") then -- new sample ready?
        rnd_ready <= '1';
      elsif (rnd_enable = '0') or (rden = '1') then -- clear when deactivated or on data read
        rnd_ready <= '0';
      end if;
    end if;
  end process processing_core;


end neorv32_trng_rtl;


-- ############################################################################################################################
-- ############################################################################################################################


-- #################################################################################################
-- # << NEORV32 - True Random Number Generator (TRNG) - GARO Chain-Based Entropy Source >>         #
-- # ********************************************************************************************* #
-- # An inverter chain (ring oscillator) is used as entropy source. The inverter chain is          #
-- # constructed as GARO (Galois Ring Oscillator) TRNG, which is an "asynchronous" LFSR. The       #
-- # single inverters are connected via latches that are used to enbale/disable the TRNG. Also,    #
-- # these latches are used as additional delay element. By using unique enable signals for each   #
-- # latch, the synthesis tool cannot "optimize" (=remove) any of the inverters out of the design. #
-- # Furthermore, the latches prevent the synthesis tool from detecting combinatorial loops.       #
-- #                                                                                               #
-- # Sources:                                                                                      #
-- #  - GARO: "Experimental Assessment of FIRO- and GARO-based Noise Sources for Digital TRNG      #
-- #    Designs on FPGAs" by Martin Schramm, Reiner Dojen and Michael Heigly, 2017                 #
-- #  - Latches for platform independence: "Extended Abstract: The Butterfly PUF Protecting IP     #
-- #    on every FPGA" by Sandeep S. Kumar, Jorge Guajardo, Roel Maesyz, Geert-Jan Schrijen and    #
-- #    Pim Tuyls, Philips Research Europe, 2008                                                   #
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

entity neorv32_trng_garo_element is
  generic (
    NUM_INV : natural := 15 -- number of inverters in chain
  );
  port (
    clk_i    : in  std_ulogic;
    enable_i : in  std_ulogic;
    enable_o : out std_ulogic;
    mask_i   : in  std_ulogic_vector(NUM_INV-2 downto 0);
    data_o   : out std_ulogic;
    error0_o : out std_ulogic;
    error1_o : out std_ulogic
  );
end neorv32_trng_garo_element;

architecture neorv32_trng_garo_element_rtl of neorv32_trng_garo_element is

  -- debugging --
  constant is_sim_c : boolean := false;

  signal inv_chain   : std_ulogic_vector(NUM_INV-1 downto 0); -- oscillator chain
  signal enable_sreg : std_ulogic_vector(NUM_INV-1 downto 0); -- enable shift register
  signal sync_ff     : std_ulogic_vector(2 downto 0); -- synchronizer

  signal cnt_zero, cnt_one : std_ulogic_vector(5 downto 0); -- stuck-at-0/1 counters

begin

  -- Sanity Check ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert ((NUM_INV mod 2) /= 0) report "NEORV32 TRNG.GARO_element: NUM_INV has to be odd." severity error;


  -- Entropy Source -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  garo_chain: process(clk_i, enable_i, enable_sreg, mask_i, inv_chain)
  begin
    if (is_sim_c = false) then
      for i in 0 to NUM_INV-1 loop -- inverters in chain
        if (enable_i = '0') then -- start with a defined state (latch reset)
          inv_chain(i) <= '0';
        -- Using individual enable signals for each inverter - derived from a shift register - to prevent the synthesis tool
        -- from removing all but one inverter (since they implement "logical identical functions").
        -- This also allows to make the TRNG platform independent.
        elsif (enable_sreg(i) = '1') then
          -- here we have the inverter chain --
          if (i = NUM_INV-1) then -- left-most inverter?
            inv_chain(i) <= not inv_chain(0); -- direct input of right most inverter (= output signal)
          else
            -- if tap switch is ON:  use final output XORed with previous inverter's output
            -- if tap switch is OFF: just use previous inverter's output
            inv_chain(i) <= not (inv_chain(i+1) xor (inv_chain(0) and mask_i(i)));
          end if;
        end if;
      end loop; -- i
    else -- simulate as simple LFSR
      if rising_edge(clk_i) then
        if (enable_i = '0') then
          inv_chain <= (others => '0');
        else
          inv_chain(NUM_INV-1 downto 0) <= inv_chain(inv_chain'left-1 downto 0) & xnor_all_f(inv_chain(NUM_INV-2 downto 0) and mask_i);
        end if;
      end if;
    end if;
  end process garo_chain;


  -- Control --------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_unit: process(clk_i)
  begin
    if rising_edge(clk_i) then
      enable_sreg <= enable_sreg(enable_sreg'left-1 downto 0) & enable_i; -- activate right-most inverter first
      sync_ff     <= sync_ff(sync_ff'left-1 downto 0) & inv_chain(0); -- synchronize to prevent metastability 
    end if;
  end process ctrl_unit;

  -- output for "enable chain" --
  enable_o <= enable_sreg(enable_sreg'left);

  -- rnd output --
  data_o <= sync_ff(sync_ff'left);


  -- Health Check ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  health_check: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (enable_sreg(enable_sreg'left) = '0') then
        cnt_zero <= (others => '0');
        cnt_one  <= (others => '0');
      else
        -- stuck-at-zero --
        if (and_all_f(cnt_zero) = '0') then -- max not reached yet
          error0_o <= '0';
          if (sync_ff(sync_ff'left) = '0') then
            cnt_zero <= std_ulogic_vector(unsigned(cnt_zero) + 1);
          else
            cnt_zero <= (others => '0');
          end if;
        else
          error0_o <= '1';
        end if;
        -- stuck-at-one --
        if (and_all_f(cnt_one) = '0') then -- max not reached yet
          error1_o <= '0';
          if (sync_ff(sync_ff'left) = '1') then
            cnt_one <= std_ulogic_vector(unsigned(cnt_one) + 1);
          else
            cnt_one <= (others => '0');
          end if;
        else
          error1_o <= '1';
        end if;
      end if;
    end if;
  end process health_check;


end neorv32_trng_garo_element_rtl;
