-- #################################################################################################
-- # << NEORV32 - True Random Number Generator (TRNG) >>                                           #
-- # ********************************************************************************************* #
-- # This unit implements a true random number generator which uses an inverter chain as entropy   #
-- # source. The inverter chain is constructed as GARO (Galois Ring Oscillator) TRNG. The single   #
-- # inverters are connected via simple latches that are used to enbale/disable the TRNG. Also,    #
-- # these latches are used as additional delay element. By using unique enable signals for each   #
-- # latch, the synthesis tool cannot "optimize" one of the inverters out of the design. Further-  #
-- # more, the latches prevent the synthesis tool from detecting combinatorial loops.              #
-- # The output of the GARO is de-biased by a simple von Neuman random extractor and is further    #
-- # post-processed by a 16-bit LFSR for improved whitening.                                       #
-- #                                                                                               #
-- # Sources:                                                                                      #
-- #  - GARO: "Experimental Assessment of FIRO- and GARO-based Noise Sources for Digital TRNG      #
-- #    Designs on FPGAs" by Martin Schramm, Reiner Dojen and Michael Heigly, 2017                 #
-- #  - Latches for platform independence: "Extended Abstract: The Butterfly PUF Protecting IP     #
-- #    on every FPGA" by Sandeep S. Kumar, Jorge Guajardo, Roel Maesyz, Geert-Jan Schrijen and    #
-- #    Pim Tuyls, Philips Research Europe, 2008                                                   #
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
  constant num_inv_c   : natural := 16; -- length of GARO inverter chain (default=16, max=16)
  constant lfsr_taps_c : std_ulogic_vector(15 downto 0) := "1101000000001000"; -- Fibonacci LFSR feedback taps
  -- -------------------------------------------------------------------------------------------------------

  -- control register bits --
  constant ctrl_taps_lsb_c  : natural :=  0; -- -/w: TAP 0 enable
  constant ctrl_taps_msb_c  : natural := 15; -- -/w: TAP 15 enable
  constant ctrl_en_c        : natural := 31; -- r/w: TRNG enable

  -- data register bits --
  constant ctrl_data_lsb_c  : natural :=  0; -- r/-: Random data bit 0
  constant ctrl_data_msb_c  : natural := 15; -- r/-: Random data bit 15
  constant ctrl_rnd_valid_c : natural := 31; -- r/-: Output byte valid

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(trng_size_c); -- low address boundary bit

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wren   : std_ulogic; -- full word write enable
  signal rden   : std_ulogic; -- read enable

  -- random number generator --
  signal rnd_inv         : std_ulogic_vector(num_inv_c-1 downto 0); -- inverter chain
  signal rnd_enable_sreg : std_ulogic_vector(num_inv_c-1 downto 0); -- enable shift register
  signal rnd_enable      : std_ulogic;
  signal tap_config      : std_ulogic_vector(15 downto 0);
  signal rnd_sync        : std_ulogic_vector(02 downto 0); -- metastability filter & de-biasing
  signal ready_ff        : std_ulogic; -- new random data available
  signal rnd_sreg        : std_ulogic_vector(15 downto 0); -- sample shift reg
  signal rnd_cnt         : std_ulogic_vector(04 downto 0);
  signal new_sample      : std_ulogic; -- new output byte ready
  signal rnd_data        : std_ulogic_vector(15 downto 0); -- random data register (read-only)

  -- Randomness extractor (von Neumann De-Biasing) --
  signal db_state  : std_ulogic;
  signal db_enable : std_ulogic; -- valid data from de-biasing
  signal db_data   : std_ulogic; -- actual data from de-biasing

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
      if (wren = '1') then
        if (addr = trng_ctrl_addr_c) then
          tap_config <= data_i(tap_config'left downto 0);
          rnd_enable <= data_i(ctrl_en_c);
        end if;
      end if;
      -- read access --
      data_o <= (others => '0');
      if (rden = '1') then
        if (addr = trng_ctrl_addr_c) then
          data_o(ctrl_taps_msb_c downto ctrl_taps_lsb_c) <= tap_config;
          data_o(ctrl_en_c) <= rnd_enable;
        else -- trng_data_addr_c
          data_o(ctrl_data_msb_c downto ctrl_data_lsb_c) <= rnd_data;
          data_o(ctrl_rnd_valid_c) <= ready_ff;
        end if;
      end if;
    end if;
  end process rw_access;


  -- True Random Generator ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  entropy_source: process(rnd_enable_sreg, rnd_enable, rnd_inv, tap_config)
  begin
    for i in 0 to num_inv_c-1 loop
      if (rnd_enable = '0') then -- start with a defined state (latch reset)
        rnd_inv(i) <= '0';
      -- uniquely enable latches to prevent synthesis from removing chain elements
      elsif (rnd_enable_sreg(i) = '1') then -- latch enable
        -- here we have the inverter chain --
        if (i = num_inv_c-1) then -- left most inverter?
          if (tap_config(i) = '1') then
            rnd_inv(i) <= not rnd_inv(0); -- direct input of right most inverter (= output signal)
          else
            rnd_inv(i) <= '0';
          end if;
        else
          if (tap_config(i) = '1') then
            rnd_inv(i) <= not (rnd_inv(i+1) xor rnd_inv(0)); -- use final output as feedback
          else
            rnd_inv(i) <= not rnd_inv(i+1); -- normal chain: use previous inverter's output as input
          end if;
        end if;
      end if;
    end loop; -- i
  end process entropy_source;

  -- unique enable signals for each inverter latch --
  inv_enable: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- using individual enable signals for each inverter - derived from a shift register - to prevent the synthesis tool
      -- from removing all but one inverter (since they implement "logical identical functions")
      -- this also allows to make the trng platform independent
      rnd_enable_sreg <= rnd_enable_sreg(num_inv_c-2 downto 0) & rnd_enable; -- activate right most inverter first
    end if;
  end process inv_enable;


  -- Processing Core ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  processing_core: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- synchronize output of GARO --
      rnd_sync <= rnd_sync(1 downto 0) & rnd_inv(0); -- no more metastability

      -- von Neumann De-Biasing state --
      db_state <= (not db_state) and rnd_enable; -- just toggle -> process in every second cycle

      -- sample random data & post-processing --
      if (rnd_enable = '0') then
        rnd_cnt  <= (others => '0');
        rnd_sreg <= (others => '0');
      elsif (db_enable = '1') then -- valid de-biased output?
        if (rnd_cnt = "10000") then
          rnd_cnt <= (others => '0');
        else
          rnd_cnt <= std_ulogic_vector(unsigned(rnd_cnt) + 1);
        end if;
        rnd_sreg <= rnd_sreg(rnd_sreg'left-1 downto 0) & (xnor_all_f(rnd_sreg and lfsr_taps_c) xor db_data); -- LFSR post-processing
--      rnd_sreg <= rnd_sreg(rnd_sreg'left-1 downto 0) & db_data; -- LFSR post-processing
      end if;

      -- data output register --
      if (new_sample = '1') then
        rnd_data <= rnd_sreg;
      end if;

      -- data ready flag --
      if (rnd_enable = '0') or (rden = '1') then -- clear when deactivated or on data read
        ready_ff <= '0';
      elsif (new_sample = '1') then
        ready_ff <= '1';
      end if;
    end if;
  end process processing_core;

  -- John von Neumann De-Biasing --
  debiasing: process(db_state, rnd_sync)
    variable tmp_v : std_ulogic_vector(2 downto 0);
  begin
    -- check groups of two non-overlapping bits from the input stream
    tmp_v := db_state & rnd_sync(2 downto 1);
    case tmp_v is
      when "101"  => db_enable <= '1'; db_data <= '1'; -- rising edge  -> '1'
      when "110"  => db_enable <= '1'; db_data <= '0'; -- falling edge -> '0'
      when others => db_enable <= '0'; db_data <= '0'; -- invalid
    end case;
  end process debiasing;

  -- new valid byte available? --
  new_sample <= '1' when (rnd_cnt = "10000") and (rnd_enable = '1') and (db_enable = '1') else '0';


end neorv32_trng_rtl;
