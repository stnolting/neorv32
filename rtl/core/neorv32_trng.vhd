-- #################################################################################################
-- # << NEORV32 - True Random Number Generator (TRNG) >>                                           #
-- # ********************************************************************************************* #
-- # This processor module instantiates the "neoTRNG" true random number generator. An optional    #
-- # "random pool" FIFO can be configured using the TRNG_FIFO generic.                             #
-- # See the neoTRNG documentation for more information: https://github.com/stnolting/neoTRNG      #
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

entity neorv32_trng is
  generic (
    IO_TRNG_FIFO : natural range 1 to 2**15 -- RND fifo depth, has to be a power of two, min 1
  );
  port (
    clk_i     : in  std_ulogic; -- global clock line
    rstn_i    : in  std_ulogic; -- global reset line, low-active, async
    bus_req_i : in  bus_req_t;  -- bus request
    bus_rsp_o : out bus_rsp_t;  -- bus response
    irq_o     : out std_ulogic  -- CPU interrupt
  );
end neorv32_trng;

architecture neorv32_trng_rtl of neorv32_trng is

  -- neoTRNG Configuration ------------------------------------------------------------------------
  constant num_cells_c     : natural := 3; -- total number of ring-oscillator cells
  constant num_inv_start_c : natural := 5; -- number of inverters in first cell, has to be odd
  -- ----------------------------------------------------------------------------------------------

  -- use simulation mode (pseudo-RNG)? --
  constant sim_mode_c : boolean := is_simulation_c; -- is this a simulation?

  -- control register bits --
  constant ctrl_data_lsb_c      : natural :=  0; -- r/-: Random data byte LSB
  constant ctrl_data_msb_c      : natural :=  7; -- r/-: Random data byte MSB
  --
  constant ctrl_fifo_size0_c    : natural := 16; -- r/-: log2(FIFO size) bit 0
  constant ctrl_fifo_size1_c    : natural := 17; -- r/-: log2(FIFO size) bit 1
  constant ctrl_fifo_size2_c    : natural := 18; -- r/-: log2(FIFO size) bit 2
  constant ctrl_fifo_size3_c    : natural := 19; -- r/-: log2(FIFO size) bit 3
  --
  constant ctrl_irq_fifo_nempty : natural := 25; -- r/w: IRQ if fifo is not empty
  constant ctrl_irq_fifo_half   : natural := 26; -- r/w: IRQ if fifo is at least half-full
  constant ctrl_irq_fifo_full   : natural := 27; -- r/w: IRQ if fifo is full
  constant ctrl_fifo_clr_c      : natural := 28; -- -/w: Clear data FIFO (auto clears)
  constant ctrl_sim_mode_c      : natural := 29; -- r/-: TRNG implemented in pseudo-RNG simulation mode
  constant ctrl_en_c            : natural := 30; -- r/w: TRNG enable
  constant ctrl_valid_c         : natural := 31; -- r/-: Output data valid

  -- Component: neoTRNG true random number generator --
  component neoTRNG
    generic (
      NUM_CELLS     : natural := 3;    -- number of ring-oscillator cells
      NUM_INV_START : natural := 5;    -- number of inverters in first cell, has to be odd
      SIM_MODE      : boolean := false -- enable simulation mode (use pseudo-RNG)
    );
    port (
      clk_i    : in  std_ulogic; -- module clock
      rstn_i   : in  std_ulogic; -- module reset, low-active, async, optional
      enable_i : in  std_ulogic; -- module enable (high-active)
      data_o   : out std_ulogic_vector(7 downto 0); -- random data byte output
      valid_o  : out std_ulogic  -- data_o is valid when set
    );
  end component;

  -- control --
  signal enable          : std_ulogic;
  signal fifo_clr        : std_ulogic;
  signal irq_fifo_nempty : std_ulogic;
  signal irq_fifo_half   : std_ulogic;
  signal irq_fifo_full   : std_ulogic;

  -- data FIFO --
  type fifo_t is record
    we    : std_ulogic; -- write enable
    re    : std_ulogic; -- read enable
    clear : std_ulogic; -- sync reset, high-active
    wdata : std_ulogic_vector(7 downto 0); -- write data
    rdata : std_ulogic_vector(7 downto 0); -- read data
    avail : std_ulogic; -- data available?
    half  : std_ulogic; -- at least half full?
    free  : std_ulogic; -- space left?
  end record;
  signal fifo : fifo_t;

begin

  -- Bus Access ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bus_rsp_o.ack   <= '0';
      bus_rsp_o.err   <= '0';
      bus_rsp_o.data  <= (others => '0');
      enable          <= '0';
      fifo_clr        <= '0';
      irq_fifo_nempty <= '0';
      irq_fifo_half   <= '0';
      irq_fifo_full   <= '0';
    elsif rising_edge(clk_i) then
      -- bus handshake --
      bus_rsp_o.ack  <= bus_req_i.stb;
      bus_rsp_o.err  <= '0';
      bus_rsp_o.data <= (others => '0');

      -- defaults --
      fifo_clr <= '0'; -- auto-clear

      if (bus_req_i.stb = '1') then

        -- write access --
        if (bus_req_i.rw = '1') then
          enable          <= bus_req_i.data(ctrl_en_c);
          fifo_clr        <= bus_req_i.data(ctrl_fifo_clr_c);
          irq_fifo_nempty <= bus_req_i.data(ctrl_irq_fifo_nempty);
          irq_fifo_half   <= bus_req_i.data(ctrl_irq_fifo_half);
          irq_fifo_full   <= bus_req_i.data(ctrl_irq_fifo_full);
        -- read access --

        else
          bus_rsp_o.data(ctrl_data_msb_c downto ctrl_data_lsb_c) <= fifo.rdata;
          --
          bus_rsp_o.data(ctrl_fifo_size3_c downto ctrl_fifo_size0_c) <= std_ulogic_vector(to_unsigned(index_size_f(IO_TRNG_FIFO), 4));
          --
          bus_rsp_o.data(ctrl_irq_fifo_nempty) <= irq_fifo_nempty;
          bus_rsp_o.data(ctrl_irq_fifo_half)   <= irq_fifo_half;
          bus_rsp_o.data(ctrl_irq_fifo_full)   <= irq_fifo_full;
          bus_rsp_o.data(ctrl_sim_mode_c)      <= bool_to_ulogic_f(sim_mode_c);
          bus_rsp_o.data(ctrl_en_c)            <= enable;
          bus_rsp_o.data(ctrl_valid_c)         <= fifo.avail;
        end if;

      end if;
    end if;
  end process bus_access;


  -- neoTRNG True Random Number Generator ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neoTRNG_inst: neoTRNG
    generic map (
      NUM_CELLS     => num_cells_c,
      NUM_INV_START => num_inv_start_c,
      SIM_MODE      => sim_mode_c
    )
    port map (
      clk_i    => clk_i,
      rstn_i   => rstn_i,
      enable_i => enable,
      data_o   => fifo.wdata,
      valid_o  => fifo.we
    );


  -- Data FIFO ("Random Pool") --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rnd_pool_fifo_inst: entity neorv32.neorv32_fifo
  generic map (
    FIFO_DEPTH => IO_TRNG_FIFO, -- number of fifo entries; has to be a power of two; min 1
    FIFO_WIDTH => 8,            -- size of data elements in fifo
    FIFO_RSYNC => true,         -- sync read
    FIFO_SAFE  => true          -- safe access
  )
  port map (
    -- control --
    clk_i   => clk_i,      -- clock, rising edge
    rstn_i  => rstn_i,     -- async reset, low-active
    clear_i => fifo.clear, -- sync reset, high-active
    half_o  => fifo.half,  -- at least half full
    -- write port --
    wdata_i => fifo.wdata, -- write data
    we_i    => fifo.we,    -- write enable
    free_o  => fifo.free,  -- at least one entry is free when set
    -- read port --
    re_i    => fifo.re,    -- read enable
    rdata_o => fifo.rdata, -- read data
    avail_o => fifo.avail  -- data available when set
  );

  fifo.clear <= '1' when (enable = '0') or (fifo_clr = '1') else '0';
  fifo.re    <= '1' when (bus_req_i.stb = '1') and (bus_req_i.rw = '0') else '0';

  -- FIFO-level interrupt generator --
 irq_generator: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      irq_o <= '0';
    elsif rising_edge(clk_i) then
      irq_o <= enable and (
               (irq_fifo_nempty and fifo.avail) or     -- IRQ if FIFO not empty
               (irq_fifo_half   and fifo.half)  or     -- IRQ if FIFO at least half full
               (irq_fifo_full   and (not fifo.free))); -- IRQ if FIFO full
    end if;
  end process irq_generator;


end neorv32_trng_rtl;


-- ############################################################################################################################
-- ############################################################################################################################


-- #################################################################################################
-- # << neoTRNG V3 - A Tiny and Platform-Independent True Random Number Generator >>               #
-- # ********************************************************************************************* #
-- # The neoTNG true-random generator uses free-running ring-oscillators to generate "phase noise" #
-- # that is used as entropy source. The ring-oscillators are based on plain inverter chains that  #
-- # are decoupled using individually-enabled latches in order to prevent the synthesis from       #
-- # trimming parts of the logic. Hence, the TRNG provides a platform-agnostic architecture that   #
-- # can be implemented for any FPGA without requiring primitive instantiation or technology-      #
-- # specific attributes or synthesis options.                                                     #
-- #                                                                                               #
-- # The random output from each entropy cells is synchronized and XOR-ed with the other cell's    #
-- # outputs before it is and fed into a simple 2-bit "von Neumann randomness extractor"           #
-- # (extracting edges). 64 de-biased bits are "combined" using a LFSR-style shift register (in    #
-- # order to improve spectral distribution) to provide one final random data byte.                #
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
-- # neoTRNG - https://github.com/stnolting/neoTRNG                            (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity neoTRNG is
  generic (
    NUM_CELLS     : natural := 3;    -- number of ring-oscillator cells
    NUM_INV_START : natural := 5;    -- number of inverters in first cell, has to be odd
    SIM_MODE      : boolean := false -- enable simulation mode (use pseudo-RNG)
  );
  port (
    clk_i    : in  std_ulogic; -- module clock
    rstn_i   : in  std_ulogic; -- module reset, low-active, async, optional
    enable_i : in  std_ulogic; -- module enable (high-active)
    data_o   : out std_ulogic_vector(7 downto 0); -- random data byte output
    valid_o  : out std_ulogic  -- data_o is valid when set
  );
end neoTRNG;

architecture neoTRNG_rtl of neoTRNG is

  -- entropy generator cell --
  component neoTRNG_cell
    generic (
      NUM_INV  : natural; -- number of inverters, has to be odd
      SIM_MODE : boolean  -- use LFSR instead of physical entropy source
    );
    port (
      clk_i  : in  std_ulogic; -- clock
      rstn_i : in  std_ulogic; -- reset, low-active, async, optional
      en_i   : in  std_ulogic; -- enable chain input
      en_o   : out std_ulogic; -- enable chain output
      rnd_o  : out std_ulogic  -- random data (sync)
    );
  end component;

  -- entropy cell interconnect --
  signal cell_en_in  : std_ulogic_vector(NUM_CELLS-1 downto 0); -- enable sreg input
  signal cell_en_out : std_ulogic_vector(NUM_CELLS-1 downto 0); -- enable sreg output
  signal cell_rnd    : std_ulogic_vector(NUM_CELLS-1 downto 0); -- cell random output
  signal rnd_raw     : std_ulogic; -- combined raw random data

  -- de-biasing --
  signal debias_sreg  : std_ulogic_vector(1 downto 0); -- sample buffer
  signal debias_state : std_ulogic; -- process de-biasing every second cycle
  signal debias_valid : std_ulogic; -- result bit valid
  signal debias_data  : std_ulogic; -- result bit

  -- sampling control --
  signal sample_en   : std_ulogic; -- global enable
  signal sample_sreg : std_ulogic_vector(7 downto 0); -- shift register / de-serializer
  signal sample_cnt  : std_ulogic_vector(6 downto 0); -- bits-per-sample (64) counter

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert false report
    "[neoTRNG NOTE] << neoTRNG V3 - A Tiny and Platform-Independent True Random Number Generator >>" severity note;
  assert ((NUM_INV_START mod 2) /= 0) report
    "[neoTRNG ERROR] Number of inverters in first cell <NUM_INV_START> has to be odd!" severity error;


  -- Entropy Source -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  entropy_source:
  for i in 0 to NUM_CELLS-1 generate
    neoTRNG_cell_inst: neoTRNG_cell
    generic map (
      NUM_INV  => NUM_INV_START + 2*i, -- increasing cell length
      SIM_MODE => SIM_MODE
    )
    port map (
      clk_i  => clk_i,
      rstn_i => rstn_i,
      en_i   => cell_en_in(i),
      en_o   => cell_en_out(i),
      rnd_o  => cell_rnd(i)
    );
  end generate;

  -- enable shift register chain --
  cell_en_in(0) <= sample_en;
  cell_en_in(NUM_CELLS-1 downto 1) <= cell_en_out(NUM_CELLS-2 downto 0);

  -- combine cell outputs --
  combine: process(cell_rnd)
    variable tmp_v : std_ulogic;
  begin
    tmp_v := '0';
    for i in 0 to NUM_CELLS-1 loop
      tmp_v := tmp_v xor cell_rnd(i);
    end loop;
    rnd_raw <= tmp_v;
  end process combine;


  -- John von Neumann Randomness Extractor (De-Biasing) -------------------------------------
  -- -------------------------------------------------------------------------------------------
  debiasing_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      debias_sreg  <= (others => '0');
      debias_state <= '0';
    elsif rising_edge(clk_i) then
      debias_sreg <= debias_sreg(0) & rnd_raw;
      -- start operation when last cell is enabled and process in every second cycle --
      debias_state <= (not debias_state) and cell_en_out(NUM_CELLS-1);
    end if;
  end process debiasing_sync;

  -- edge detector - check groups of two non-overlapping bits from the random stream --
  debiasing_comb: process(debias_state, debias_sreg)
    variable tmp_v : std_ulogic_vector(2 downto 0);
  begin
    tmp_v := debias_state & debias_sreg(1 downto 0);
    case tmp_v is
      when "101"  => debias_valid <= '1'; -- rising edge
      when "110"  => debias_valid <= '1'; -- falling edge
      when others => debias_valid <= '0'; -- no valid data
    end case;
  end process debiasing_comb;

  -- edge data --
  debias_data <= debias_sreg(0);


  -- Sampling Control -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  sampling_control: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      sample_en   <= '0';
      sample_cnt  <= (others => '0');
      sample_sreg <= (others => '0');
    elsif rising_edge(clk_i) then
      sample_en <= enable_i;
      if (sample_en = '0') or (sample_cnt(sample_cnt'left) = '1') then -- start new iteration
        sample_cnt  <= (others => '0');
        sample_sreg <= (others => '0');
      elsif (debias_valid = '1') then -- LFSR-style sample shift register to inter-mix random stream
        sample_cnt  <= std_ulogic_vector(unsigned(sample_cnt) + 1);
        sample_sreg <= sample_sreg(6 downto 0) & (sample_sreg(7) xor debias_data);
      end if;
    end if;
  end process sampling_control;

  -- TRNG output stream --
  data_o  <= sample_sreg;
  valid_o <= sample_cnt(sample_cnt'left);


end neoTRNG_rtl;


-- ############################################################################################################################
-- ############################################################################################################################


-- #################################################################################################
-- # << neoTRNG V3 - A Tiny and Platform-Independent True Random Number Generator >>               #
-- # ********************************************************************************************* #
-- # neoTRNG entropy source cell, based on a simple ring-oscillator constructed from an odd number #
-- # of inverter. The inverters are decoupled using individually-enabled latches to prevent the    #
-- # synthesis from removing parts of the oscillator chain - hardware hack! ;)                     #
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
-- # neoTRNG - https://github.com/stnolting/neoTRNG                            (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;

entity neoTRNG_cell is
  generic (
    NUM_INV  : natural; -- number of inverters, has to be odd
    SIM_MODE : boolean  -- use LFSR instead of physical entropy source
  );
  port (
    clk_i  : in  std_ulogic; -- clock
    rstn_i : in  std_ulogic; -- reset, low-active, async, optional
    en_i   : in  std_ulogic; -- enable chain input
    en_o   : out std_ulogic; -- enable chain output
    rnd_o  : out std_ulogic  -- random data (sync)
  );
end neoTRNG_cell;

architecture neoTRNG_cell_rtl of neoTRNG_cell is

  signal rosc : std_ulogic_vector(NUM_INV-1 downto 0); -- ring oscillator element: inverter + latch
  signal sreg : std_ulogic_vector(NUM_INV-1 downto 0); -- enable shift register
  signal sync : std_ulogic_vector(1 downto 0); -- output synchronizer

begin

  -- Physical Entropy Source: Ring Oscillator -----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- Each cell is based on a simple ring oscillator with an odd number of inverters. Each
  -- inverter is followed by a latch that provides a reset (to start in a defined state) and
  -- a latch-enable to make the latch transparent. Switching to transparent mode is done one by
  -- one by the enable shift register (see notes below).

  sim_mode_false:
  if SIM_MODE = false generate

    -- ring oscillator --
    ring_osc:
    for i in 0 to NUM_INV-1 generate

      ring_osc_start:
      if (i = 0) generate -- inverting latch
        rosc(i) <= '0' when (en_i = '0') else (not rosc(NUM_INV-1)) when (sreg(i) = '1') else rosc(i);
      end generate;

      ring_osc_chain:
      if (i > 0) generate -- inverting latch
        rosc(i) <= '0' when (en_i = '0') else (not rosc(i-1)) when (sreg(i) = '1') else rosc(i);
      end generate;

    end generate;

  end generate;


  -- Simulation-Only Entropy Source: Pseudo-RNG ---------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- The pseudo-RNG is meant for functional rtl simulation only. It is based on a simple LFSR.
  -- Do not use this option for "real" implementations!

  sim_mode_true:
  if SIM_MODE = true generate

    assert false report
      "[neoTRNG WARNING] Implementing non-physical pseudo-RNG!" severity warning;

    sim_lfsr: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        rosc <= (others => '0');
      elsif rising_edge(clk_i) then
        if (en_i = '0') then
          rosc <= (others => '0');
        else -- sequence might NOT be maximum-length!
          rosc(rosc'left downto 1) <= rosc(rosc'left-1 downto 0);
          rosc(0) <= not (rosc(NUM_INV-1) xor rosc(0));
        end if;
      end if;
    end process sim_lfsr;

  end generate;


  -- Output Synchronizer --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- Sample the actual entropy source (= phase noise) and move it to the system's clock domain.

  synchronizer: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      sync <= (others => '0');
    elsif rising_edge(clk_i) then
      sync <= sync(0) & rosc(NUM_INV-1);
    end if;
  end process synchronizer;

  -- cell output --
  rnd_o <= sync(1);


  -- Enable Shift-Register ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- Using individual enable signals from a shift register for each inverter in order to prevent
  -- the synthesis tool from removing all but one inverter (since they implement "logical
  -- identical functions"). This makes the TRNG platform independent as we do not require tool-/
  -- technology-specific primitives, attributes or other options.

  en_shift_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      sreg <= (others => '0');
    elsif rising_edge(clk_i) then
      sreg <= sreg(sreg'left-1 downto 0) & en_i;
    end if;
  end process en_shift_reg;

  -- output for global enable chain --
  en_o <= sreg(sreg'left);


end neoTRNG_cell_rtl;
