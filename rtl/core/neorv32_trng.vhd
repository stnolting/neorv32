-- ================================================================================ --
-- NEORV32 SoC - True Random Number Generator (TRNG)                                --
-- -------------------------------------------------------------------------------- --
-- Based on the neoTRNG: https://github.com/stnolting/neoTRNG                       --
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

entity neorv32_trng is
  generic (
    IO_TRNG_FIFO : natural range 1 to 2**15 -- RND fifo depth, has to be a power of two, min 1
  );
  port (
    clk_i     : in  std_ulogic; -- global clock line
    rstn_i    : in  std_ulogic; -- global reset line, low-active, async
    bus_req_i : in  bus_req_t;  -- bus request
    bus_rsp_o : out bus_rsp_t;  -- bus response
    irq_o     : out std_ulogic  -- data-available interrupt
  );
end neorv32_trng;

architecture neorv32_trng_rtl of neorv32_trng is

  -- neoTRNG Configuration --------------------------------------------------------------------------
  constant num_cells_c     : natural := 3; -- total number of ring-oscillator cells
  constant num_inv_start_c : natural := 5; -- number of inverters in first cell, has to be odd, min 3
  -- ------------------------------------------------------------------------------------------------

  -- control register bits --
  constant ctrl_data_lsb_c   : natural :=  0; -- r/-: Random data byte LSB
  constant ctrl_data_msb_c   : natural :=  7; -- r/-: Random data byte MSB
  --
  constant ctrl_fifo_size0_c : natural := 16; -- r/-: log2(FIFO size) bit 0
  constant ctrl_fifo_size3_c : natural := 19; -- r/-: log2(FIFO size) bit 3
  --
  constant ctrl_irq_sel_c    : natural := 27; -- r/w: interrupt select (0 = data available, 1 = FIFO full)
  constant ctrl_fifo_clr_c   : natural := 28; -- -/w: Clear data FIFO (auto clears)
  constant ctrl_sim_mode_c   : natural := 29; -- r/-: TRNG implemented in pseudo-RNG simulation mode
  constant ctrl_en_c         : natural := 30; -- r/w: TRNG enable
  constant ctrl_valid_c      : natural := 31; -- r/-: Output data valid

  -- neoTRNG true random number generator --
  component neoTRNG
    generic (
      NUM_CELLS     : natural := 3;    -- number of ring-oscillator cells
      NUM_INV_START : natural := 5;    -- number of inverters in first cell, has to be odd, min 3
      SIM_MODE      : boolean := false -- enable simulation mode (adding explicit propagation delay)
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
  signal enable, irq_sel, fifo_clr : std_ulogic;

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
      bus_rsp_o <= rsp_terminate_c;
      fifo_clr  <= '0';
      irq_sel   <= '0';
      enable    <= '0';
    elsif rising_edge(clk_i) then
      -- defaults --
      bus_rsp_o.ack  <= bus_req_i.stb;
      bus_rsp_o.err  <= '0';
      bus_rsp_o.data <= (others => '0');
      fifo_clr       <= '0'; -- auto-clear
      -- host access --
      if (bus_req_i.stb = '1') then
        if (bus_req_i.rw = '1') then -- write access
          irq_sel  <= bus_req_i.data(ctrl_irq_sel_c);
          fifo_clr <= bus_req_i.data(ctrl_fifo_clr_c);
          enable   <= bus_req_i.data(ctrl_en_c);
        else -- read access
          bus_rsp_o.data(ctrl_data_msb_c downto ctrl_data_lsb_c) <= fifo.rdata;
          --
          bus_rsp_o.data(ctrl_fifo_size3_c downto ctrl_fifo_size0_c) <= std_ulogic_vector(to_unsigned(index_size_f(IO_TRNG_FIFO), 4));
          --
          bus_rsp_o.data(ctrl_irq_sel_c)  <= irq_sel;
          bus_rsp_o.data(ctrl_sim_mode_c) <= bool_to_ulogic_f(is_simulation_c);
          bus_rsp_o.data(ctrl_en_c)       <= enable;
          bus_rsp_o.data(ctrl_valid_c)    <= fifo.avail;
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
      SIM_MODE      => is_simulation_c
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
    FIFO_SAFE  => true,         -- safe access
    FULL_RESET => false         -- no HW reset, try to infer BRAM
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

  -- IRQ generator --
  irq_generator: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      irq_o <= '0';
    elsif rising_edge(clk_i) then
      if (enable = '1') then
        if (irq_sel = '0') then -- fire IRQ if any data is available
          irq_o <= fifo.avail;
        else -- fire IRQ if data FIFO is full
          irq_o <= not fifo.free;
        end if;
      else
        irq_o <= '0';
      end if;
    end if;
  end process irq_generator;


end neorv32_trng_rtl;


-- #################################################################################################
-- # neoTRNG - A Tiny and Platform-Independent True Random Number Generator (version 3.1)          #
-- # https://github.com/stnolting/neoTRNG                                                          #
-- # ********************************************************************************************* #
-- # The neoTNG true-random number generator samples free-running ring-oscillators (combinatorial  #
-- # loops) to obtain *phase noise* hat is used as entropy source. The individual ring-oscillators #
-- # are based on plain inverter chains that are decoupled using individually-enabled latches in   #
-- # order to prevent the synthesis tool from trimming parts of the logic.                         #
-- #                                                                                               #
-- # Hence, the TRNG provides a platform- agnostic architecture that can be implemented for any    #
-- # FPGA/ASIC without requiring primitive instantiation or technology-specific attributes or      #
-- # platform-specific synthesis options.                                                          #
-- #                                                                                               #
-- # The random output from each entropy cells is synchronized and XOR-ed with the other cell's    #
-- # outputs before it is and fed into a simple 2-bit "John von Neumann randomness extractor"      #
-- # (extracting edges). A total of 64 de-biased bits are combined using a LFSR-style shift        #
-- # register (for improved spectral distribution) to provide one final random data byte.          #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2024, Stephan Nolting. All rights reserved.                                     #
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
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity neoTRNG is
  generic (
    NUM_CELLS     : natural := 3;    -- number of ring-oscillator cells
    NUM_INV_START : natural := 5;    -- number of inverters in first ring-oscillator cell, has to be odd
    SIM_MODE      : boolean := false -- enable simulation mode (adding explicit propagation delay)
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
      NUM_INV  : natural := 3;    -- number of inverters, has to be odd, min 3
      SIM_MODE : boolean := false -- enable simulation mode (adding explicit propagation delay)
    );
    port (
      clk_i  : in  std_ulogic; -- clock
      rstn_i : in  std_ulogic; -- reset, low-active, async, optional
      en_i   : in  std_ulogic; -- enable-chain input
      en_o   : out std_ulogic; -- enable-chain output
      rnd_o  : out std_ulogic  -- random data (sync)
    );
  end component;

  -- entropy cell interconnect --
  signal cell_en_in   : std_ulogic_vector(NUM_CELLS-1 downto 0); -- enable-sreg input
  signal cell_en_out  : std_ulogic_vector(NUM_CELLS-1 downto 0); -- enable-sreg output
  signal cell_rnd     : std_ulogic_vector(NUM_CELLS-1 downto 0); -- cell random output
  signal rnd_raw      : std_ulogic; -- combined raw random data

  -- de-biasing --
  signal debias_sreg  : std_ulogic_vector(1 downto 0); -- sample buffer
  signal debias_state : std_ulogic; -- process de-biasing every second cycle
  signal debias_valid : std_ulogic; -- result bit valid
  signal debias_data  : std_ulogic; -- result bit

  -- sampling control --
  signal sample_en    : std_ulogic; -- global enable
  signal sample_sreg  : std_ulogic_vector(7 downto 0); -- shift-register / de-serializer
  signal sample_cnt   : std_ulogic_vector(6 downto 0); -- bits-per-sample (64) counter

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert false report
    "[neoTRNG] The neoTRNG (v3.1) - A Tiny and Platform-Independent True Random Number Generator, " &
    "https://github.com/stnolting/neoTRNG" severity note;

  assert (NUM_INV_START mod 2) /= 0 report
    "[neoTRNG] Number of inverters in first cell <NUM_INV_START> has to be odd!" severity error;

  assert NUM_INV_START >= 3 report
    "[neoTRNG] Number of inverters in first cell <NUM_INV_START> has to be at least 3!" severity error;

  assert not SIM_MODE report
    "[neoTRNG] Simulation-mode enabled!" severity warning;


  -- Entropy Source(s) ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  entropy_cell_gen:
  for i in 0 to NUM_CELLS-1 generate
    neoTRNG_cell_inst: neoTRNG_cell
    generic map (
      NUM_INV  => NUM_INV_START + (2 * i), -- increasing (odd) ring-oscillator length
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

  -- enable-shift-register chain --
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
  debiasing: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      debias_sreg  <= (others => '0');
      debias_state <= '0';
    elsif rising_edge(clk_i) then
      debias_sreg <= debias_sreg(0) & rnd_raw;
      -- start operation when last cell is enabled and process in every second cycle --
      debias_state <= (not debias_state) and cell_en_out(cell_en_out'left);
    end if;
  end process debiasing;

  -- check groups of two non-overlapping bits from the random stream for edges --
  debias_valid <= debias_state and (debias_sreg(1) xor debias_sreg(0));
  debias_data  <= debias_sreg(0);


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
      elsif (debias_valid = '1') then -- LFSR-style sampling shift-register to scramble and mix random stream
        sample_cnt  <= std_ulogic_vector(unsigned(sample_cnt) + 1);
        sample_sreg <= sample_sreg(6 downto 0) & (sample_sreg(7) xor debias_data);
      end if;
    end if;
  end process sampling_control;

  -- TRNG output stream --
  data_o  <= sample_sreg;
  valid_o <= sample_cnt(sample_cnt'left);

end neoTRNG_rtl;


-- **********************************************************************************************************
-- neoTRNG entropy source cell, based on a simple ring-oscillator constructed from an odd number
-- of inverter. The inverters are decoupled using individually-enabled latches to prevent synthesis
-- from removing parts of the oscillator chain.
-- **********************************************************************************************************

library ieee;
use ieee.std_logic_1164.all;

entity neoTRNG_cell is
  generic (
    NUM_INV  : natural := 3;    -- number of inverters, has to be odd, min 3
    SIM_MODE : boolean := false -- enable simulation mode (adding explicit propagation delay)
  );
  port (
    clk_i  : in  std_ulogic; -- clock
    rstn_i : in  std_ulogic; -- reset, low-active, async, optional
    en_i   : in  std_ulogic; -- enable-chain input
    en_o   : out std_ulogic; -- enable-chain output
    rnd_o  : out std_ulogic  -- random data (sync)
  );
end neoTRNG_cell;

architecture neoTRNG_cell_rtl of neoTRNG_cell is

  signal sreg    : std_ulogic_vector(NUM_INV-1 downto 0); -- enable-shift-register
  signal latch   : std_ulogic_vector(NUM_INV-1 downto 0); -- ring oscillator: latches
  signal inv_in  : std_ulogic_vector(NUM_INV-1 downto 0); -- ring oscillator: inverter inputs
  signal inv_out : std_ulogic_vector(NUM_INV-1 downto 0); -- ring oscillator: inverter outputs
  signal sync    : std_ulogic_vector(1 downto 0); -- output synchronizer

begin

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


  -- Physical Entropy Source: Ring Oscillator -----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- Each cell is based on a simple ring oscillator with an odd number of inverters. Each
  -- inverter is followed by a latch that provides a reset (to start in a defined state) and
  -- a latch-enable to make the latch transparent. Switching to transparent mode is done one by
  -- one by the enable shift register.

  ring_osc_gen:
  for i in 0 to NUM_INV-1 generate

    -- latch with global reset and individual enable --
    latch(i) <= '0' when (en_i = '0') else latch(i) when (sreg(i) = '0') else inv_out(i);

    -- inverter with simulated propagation delay --
    inverter_sim:
    if SIM_MODE generate
      inv_out(i) <= (not inv_in(i)) after 2 ns; -- for SIMULATION ONLY
    end generate;

    -- inverter for actual synthesis --
    inverter_phy:
    if not SIM_MODE generate
      inv_out(i) <= (not inv_in(i));
    end generate;

  end generate;

  -- interconnect --
  inv_in(0) <= latch(NUM_INV-1); -- beginning/end of chain
  inv_in(NUM_INV-1 downto 1) <= latch(NUM_INV-2 downto 0); -- inside chain


  -- Output Synchronizer --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- Sample the actual entropy source (= phase noise) and move it to the system's clock domain.

  synchronizer: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      sync <= (others => '0');
    elsif rising_edge(clk_i) then
      sync <= sync(0) & latch(latch'left);
    end if;
  end process synchronizer;

  -- cell output --
  rnd_o <= sync(1);

end neoTRNG_cell_rtl;
