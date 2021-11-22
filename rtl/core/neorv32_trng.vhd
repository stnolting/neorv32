-- #################################################################################################
-- # << NEORV32 - True Random Number Generator (TRNG) >>                                           #
-- # ********************************************************************************************* #
-- # This processor module instantiates the "neoTRNG" true random number generator.                #
-- # See the neoTRNG's documentation for more information: https://github.com/stnolting/neoTRNG    #
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

  -- neoTRNG Configuration -------------------------------------------------------------------------------------------
  constant num_cells_c     : natural := 3; -- total number of ring-oscillator cells
  constant num_inv_start_c : natural := 3; -- number of inverters in first cell (short path), has to be odd
  constant num_inv_inc_c   : natural := 2; -- number of additional inverters in next cell (short path), has to be even
  constant num_inv_delay_c : natural := 2; -- additional inverters to form cell's long path, has to be even
  -- -----------------------------------------------------------------------------------------------------------------

  -- control register bits --
  constant ctrl_data_lsb_c : natural :=  0; -- r/-: Random data byte LSB
  constant ctrl_data_msb_c : natural :=  7; -- r/-: Random data byte MSB
  constant ctrl_en_c       : natural := 30; -- r/w: TRNG enable
  constant ctrl_valid_c    : natural := 31; -- r/-: Output data valid

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(trng_size_c); -- low address boundary bit

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal wren   : std_ulogic; -- full word write enable
  signal rden   : std_ulogic; -- read enable

  -- Component: neoTRNG true random number generator --
  component neoTRNG
    generic (
      NUM_CELLS     : natural; -- total number of ring-oscillator cells
      NUM_INV_START : natural; -- number of inverters in first cell (short path), has to be odd
      NUM_INV_INC   : natural; -- number of additional inverters in next cell (short path), has to be even
      NUM_INV_DELAY : natural  -- additional inverters to form cell's long path, has to be even
    );
    port (
      clk_i    : in  std_ulogic; -- global clock line
      enable_i : in  std_ulogic; -- unit enable (high-active), reset unit when low
      data_o   : out std_ulogic_vector(7 downto 0); -- random data byte output
      valid_o  : out std_ulogic  -- data_o is valid when set
    );
  end component;

  -- TRNG interface --
  signal trng_data  : std_ulogic_vector(7 downto 0);
  signal trng_valid : std_ulogic;

  -- arbiter --
  signal enable  : std_ulogic;
  signal valid   : std_ulogic;
  signal rnd_reg : std_ulogic_vector(7 downto 0);

begin

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
      -- host bus acknowledge --
      ack_o <= wren or rden;

      -- write access --
      if (wren = '1') then
        enable <= data_i(ctrl_en_c);
      end if;

      -- read access --
      data_o <= (others => '0');
      if (rden = '1') then
        data_o(ctrl_data_msb_c downto ctrl_data_lsb_c) <= rnd_reg;
        data_o(ctrl_en_c)    <= enable;
        data_o(ctrl_valid_c) <= valid;
      end if;

      -- sample --
      if (trng_valid = '1') then
        rnd_reg <= trng_data;
      end if;

      -- data valid? --
      if (enable = '0') then -- disabled
        valid <= '0';
      else
        if (trng_valid = '1') then
          valid <= '1';
        elsif (rden = '1') then
          valid <= '0';
        end if;
      end if;
    end if;
  end process rw_access;


  -- neoTRNG --------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neoTRNG_inst: neoTRNG
    generic map (
      NUM_CELLS     => num_cells_c,
      NUM_INV_START => num_inv_start_c,
      NUM_INV_INC   => num_inv_inc_c,
      NUM_INV_DELAY => num_inv_delay_c
    )
    port map (
      clk_i    => clk_i,
      enable_i => enable,
      data_o   => trng_data,
      valid_o  => trng_valid
    );


end neorv32_trng_rtl;


-- ############################################################################################################################
-- ############################################################################################################################


-- #################################################################################################
-- # << neoTRNG - A Tiny and Platform-Independent True Random Number Generator for any FPGA >>     #
-- # ********************************************************************************************* #
-- # This generator is based on entropy cells, which implement simple ring-oscillators. Each ring- #
-- # oscillator features a short and a long delay path that is dynamically selected defining the   #
-- # primary oscillation frequency. The cells are cascaded so that the random data output of a     #
-- # cell controls the delay path of the next cell (which has the next-larger inverter chain).     #
-- #                                                                                               #
-- # The random data outputs of all cells are XOR-ed and de-biased using a von Neumann randomness  #
-- # extractor (converting edges into bits). The resulting bit is sampled in chunks of 8 bits to   #
-- # provide the final random data output. No further internal post-processing is applied. Hence,  #
-- # the TRNG produces simple de-biased *RAW* data.                                                #
-- #                                                                                               #
-- # The entropy cell architecture uses individually-controlled latches and inverters to create    #
-- # the inverter chain in a platform-agnostic style that can be implemented for any FPGA without  #
-- # requiring primitive instantiation or technology-specific attributes.                          #
-- #                                                                                               #
-- # See the neoTRNG's documentation for more information: https://github.com/stnolting/neoTRNG    #
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
-- # neoTRNG - https://github.com/stnolting/neoTRNG                            (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity neoTRNG is
  generic (
    NUM_CELLS     : natural; -- total number of ring-oscillator cells
    NUM_INV_START : natural; -- number of inverters in first cell (short path), has to be odd
    NUM_INV_INC   : natural; -- number of additional inverters in next cell (short path), has to be even
    NUM_INV_DELAY : natural  -- additional inverters to form cell's long path, has to be even
  );
  port (
    clk_i    : in  std_ulogic; -- global clock line
    enable_i : in  std_ulogic; -- unit enable (high-active), reset unit when low
    data_o   : out std_ulogic_vector(7 downto 0); -- random data byte output
    valid_o  : out std_ulogic  -- data_o is valid when set
  );
end neoTRNG;

architecture neoTRNG_rtl of neoTRNG is

  -- Component: neoTRNG entropy cell --
  component neoTRNG_cell
    generic (
      NUM_INV_S : natural; -- number of inverters in short path
      NUM_INV_L : natural  -- number of inverters in long path
    );
    port (
      clk_i    : in  std_ulogic; -- system clock
      select_i : in  std_ulogic; -- delay select
      enable_i : in  std_ulogic; -- enable chain input
      enable_o : out std_ulogic; -- enable chain output
      data_o   : out std_ulogic  -- sync random bit
    );
  end component;

  -- ring-oscillator array interconnect --
  type cell_array_t is record
    en_in  : std_ulogic_vector(NUM_CELLS-1 downto 0);
    en_out : std_ulogic_vector(NUM_CELLS-1 downto 0);
    rnd    : std_ulogic_vector(NUM_CELLS-1 downto 0);
    sel    : std_ulogic_vector(NUM_CELLS-1 downto 0);
  end record;
  signal cell_array : cell_array_t;

  -- global cell-XOR --
  signal rnd_bit : std_ulogic;

  -- von-Neumann de-biasing --
  type debiasing_t is record
    sreg  : std_ulogic_vector(1 downto 0);
    state : std_ulogic; -- process de-biasing every second cycle
    valid : std_ulogic; -- de-biased data
    data  : std_ulogic; -- de-biased data valid
  end record;
  signal deb : debiasing_t;

  -- control unit --
  type ctrl_t is record
    enable : std_ulogic;
    run    : std_ulogic;
    cnt    : std_ulogic_vector(2 downto 0); -- bit counter
    sreg   : std_ulogic_vector(7 downto 0); -- data shift register
  end record;
  signal ctrl : ctrl_t;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (NUM_CELLS < 2) report "neoTRNG config ERROR: Total number of ring-oscillator cells <NUM_CELLS> has to be >= 2." severity error;
  assert not ((NUM_INV_START mod 2)  = 0) report "neoTRNG config ERROR: Number of inverters in first cell <NUM_INV_START> has to be odd." severity error;
  assert not ((NUM_INV_INC   mod 2) /= 0) report "neoTRNG config ERROR: Inverter increment for each next cell <NUM_INV_INC> has to be even." severity error;
  assert not ((NUM_INV_DELAY mod 2) /= 0) report "neoTRNG config ERROR: Inverter increment to form long path <NUM_INV_DELAY> has to be even." severity error;


  -- Entropy Source -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neoTRNG_cell_inst:
  for i in 0 to NUM_CELLS-1 generate
    neoTRNG_cell_inst_i: neoTRNG_cell
    generic map (
      NUM_INV_S => NUM_INV_START + (i*NUM_INV_INC), -- number of inverters in short chain
      NUM_INV_L => NUM_INV_START + (i*NUM_INV_INC) + NUM_INV_DELAY -- number of inverters in long chain
    )
    port map (
      clk_i    => clk_i,
      select_i => cell_array.sel(i),
      enable_i => cell_array.en_in(i),
      enable_o => cell_array.en_out(i),
      data_o   => cell_array.rnd(i) -- SYNC data output
    );
  end generate;

  -- path select chain --
  cell_array.sel(0) <= cell_array.rnd(NUM_CELLS-1); -- use output of last cell to select path of first cell
  cell_array.sel(NUM_CELLS-1 downto 1) <= cell_array.rnd(NUM_CELLS-2 downto 0); -- i+1 <= i

  -- enable chain --
  cell_array.en_in(0) <= ctrl.enable; -- start of chain
  cell_array.en_in(NUM_CELLS-1 downto 1) <=cell_array.en_out(NUM_CELLS-2 downto 0); -- i+1 <= i


  -- XOR All Cell's Outputs -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cell_xor: process(cell_array.rnd)
    variable tmp_v : std_ulogic;
  begin
    tmp_v := '0';
    for i in 0 to NUM_CELLS-1 loop
      tmp_v := tmp_v xor cell_array.rnd(i);
    end loop; -- i
    rnd_bit <= tmp_v;
  end process cell_xor;


  -- John von Neumann Randomness Extractor --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  debiasing_sync: process(clk_i)
  begin
    if rising_edge(clk_i) then
      deb.sreg <= deb.sreg(0) & rnd_bit;
      -- start operation when last cell is enabled and process in every second cycle --
      deb.state <= (not deb.state) and cell_array.en_out(NUM_CELLS-1);
    end if;
  end process debiasing_sync;

  -- edge detector --
  debiasing_comb: process(deb)
    variable tmp_v : std_ulogic_vector(2 downto 0);
  begin
    tmp_v := deb.state & deb.sreg(1 downto 0); -- check groups of two non-overlapping bits from the input stream
    case tmp_v is
      when "101"  => deb.valid <= '1'; deb.data <= '0'; -- rising edge = '0'
      when "110"  => deb.valid <= '1'; deb.data <= '1'; -- falling edge = '1'
      when others => deb.valid <= '0'; deb.data <= '-'; -- no valid data
    end case;
  end process debiasing_comb;


  -- Control Unit ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  control_unit: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- make sure enable is sync --
      ctrl.enable <= enable_i;

      -- sample chunks of 8 bit --
      if (ctrl.enable = '0') then
        ctrl.cnt <= (others => '0');
        ctrl.run <= '0';
      elsif (deb.valid = '1') then -- valid random sample?
        ctrl.cnt <= std_ulogic_vector(unsigned(ctrl.cnt) + 1);
        ctrl.run <= '1';
      end if;

      -- sample shift register --
      if (deb.valid = '1') then
        ctrl.sreg <= ctrl.sreg(ctrl.sreg'left-1 downto 0) & deb.data;
      end if;

    end if;
  end process control_unit;

  -- random byte output --
  data_o <= ctrl.sreg;

  -- data valid? --
  valid_o <= '1' when (ctrl.cnt = "000") and (ctrl.run = '1') else '0';


end neoTRNG_rtl;


-- ############################################################################################################################
-- ############################################################################################################################


-- #################################################################################################
-- # << neoTRNG - A Tiny and Platform-Independent True Random Number Generator for any FPGA >>     #
-- # ********************************************************************************************* #
-- # neoTRNG Entropy Cell                                                                          #
-- #                                                                                               #
-- # The cell consists of two ring-oscillators build from inverter chains. The short chain uses    #
-- # NUM_INV_S inverters and oscillates at a "high" frequency and the long chain uses NUM_INV_L    #
-- # inverters and oscillates at a "low" frequency. The select_i input selects which chain is      #
-- # actually used.                                                                                #
-- #                                                                                               #
-- # Each inverter chain is constructed as an "asynchronous" shift register. The single inverters  #
-- # are connected via latches that are used to enable/disable the TRNG. Also, these latches are   #
-- # used as additional delay element. By using unique enable signals for each latch, the          #
-- # synthesis tool cannot "optimize" (=remove) any of the inverters out of the design making the  #
-- # design platform-agnostic.                                                                     #
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
-- # neoTRNG - https://github.com/stnolting/neoTRNG                            (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;

entity neoTRNG_cell is
  generic (
    NUM_INV_S : natural; -- number of inverters in short path
    NUM_INV_L : natural  -- number of inverters in long path
  );
  port (
    clk_i    : in  std_ulogic; -- system clock
    select_i : in  std_ulogic; -- delay select
    enable_i : in  std_ulogic; -- enable chain input
    enable_o : out std_ulogic; -- enable chain output
    data_o   : out std_ulogic  -- sync random bit
  );
end neoTRNG_cell;

architecture neoTRNG_cell_rtl of neoTRNG_cell is

  signal inv_chain_s   : std_ulogic_vector(NUM_INV_S-1 downto 0); -- short oscillator chain
  signal inv_chain_l   : std_ulogic_vector(NUM_INV_L-1 downto 0); -- long oscillator chain
  signal feedback      : std_ulogic; -- cell feedback/output
  signal enable_sreg_s : std_ulogic_vector(NUM_INV_S-1 downto 0); -- enable shift register for short chain
  signal enable_sreg_l : std_ulogic_vector(NUM_INV_L-1 downto 0); -- enable shift register for long chain
  signal sync_ff       : std_ulogic_vector(1 downto 0); -- output signal synchronizer

begin

  -- Ring Oscillators -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- Each cell provides a short inverter chain (high frequency) and a long oscillator chain (low frequency).
  -- The select_i signals defines which chain is enabled.
  -- NOTE: All signals that control a inverter-latch element have to be registered to ensure a single element
  -- is mapped to a single LUT (or LUT + FF(latch-mode)).

  -- short oscillator chain --
  ring_osc_short: process(enable_i, enable_sreg_s, feedback, inv_chain_s)
  begin
    for i in 0 to NUM_INV_S-1 loop -- inverters in short chain
      if (enable_i = '0') then -- start with a defined state (latch reset)
        inv_chain_s(i) <= '0';
      elsif (enable_sreg_s(i) = '1') then
        if (i = NUM_INV_S-1) then -- left-most inverter?
          inv_chain_s(i) <= not feedback;
        else
          inv_chain_s(i) <= not inv_chain_s(i+1);
        end if;
      end if;
    end loop; -- i
  end process ring_osc_short;

  -- long oscillator chain --
  ring_osc_long: process(enable_i, enable_sreg_l, feedback, inv_chain_l)
  begin
    for i in 0 to NUM_INV_L-1 loop -- inverters in long chain
      if (enable_i = '0') then -- start with a defined state (latch reset)
        inv_chain_l(i) <= '0';
      elsif (enable_sreg_l(i) = '1') then
        if (i = NUM_INV_L-1) then -- left-most inverter?
          inv_chain_l(i) <= not feedback;
        else
          inv_chain_l(i) <= not inv_chain_l(i+1);
        end if;
      end if;
    end loop; -- i
  end process ring_osc_long;

  -- length select --
  feedback <= inv_chain_l(0) when (select_i = '0') else inv_chain_s(0);


  -- Control --------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- Using individual enable signals for each inverter from a shift register to prevent the synthesis tool
  -- from removing all but one inverter (since they implement "logical identical functions" (='toggle')).
  -- This makes the TRNG platform independent (since we do not need to use primitives to ensure a correct architecture).
  ctrl_unit: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- enable sreg --
      enable_sreg_s <= enable_sreg_s(enable_sreg_s'left-1 downto 0) & enable_i;
      enable_sreg_l <= enable_sreg_l(enable_sreg_l'left-1 downto 0) & enable_sreg_s(enable_sreg_s'left);
      -- data output sync - no metastability beyond this point --
      sync_ff <= sync_ff(0) & feedback;
    end if;
  end process ctrl_unit;

  -- output for "enable chain" --
  enable_o <= enable_sreg_l(enable_sreg_l'left);

  -- random data output --
  data_o <= sync_ff(1);


end neoTRNG_cell_rtl;
