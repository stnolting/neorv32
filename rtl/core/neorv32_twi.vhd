-- #################################################################################################
-- # << NEORV32 - Two-Wire Interface Controller (TWI) >>                                           #
-- # ********************************************************************************************* #
-- # Supports START and STOP conditions, 8 bit data + ACK/NACK transfers and clock stretching.     #
-- # Supports ACKs by the constroller. No multi-controller support and no peripheral mode support  #
-- # yet. Interrupt: TWI_transfer_done                                                             #
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

entity neorv32_twi is
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
    -- com lines --
    twi_sda_io  : inout std_logic; -- serial data line
    twi_scl_io  : inout std_logic; -- serial clock line
    -- interrupt --
    irq_o       : out std_ulogic -- transfer done IRQ
  );
end neorv32_twi;

architecture neorv32_twi_rtl of neorv32_twi is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(twi_size_c); -- low address boundary bit

  -- control reg bits --
  constant ctrl_twi_en_c     : natural := 0; -- r/w: TWI enable
  constant ctrl_twi_start_c  : natural := 1; -- -/w: Generate START condition
  constant ctrl_twi_stop_c   : natural := 2; -- -/w: Generate STOP condition
  constant ctrl_twi_prsc0_c  : natural := 3; -- r/w: CLK prsc bit 0
  constant ctrl_twi_prsc1_c  : natural := 4; -- r/w: CLK prsc bit 1
  constant ctrl_twi_prsc2_c  : natural := 5; -- r/w: CLK prsc bit 2
  constant ctrl_twi_mack_c   : natural := 6; -- r/w: generate ACK by controller for transmission
  constant ctrl_twi_cksten_c : natural := 7; -- r/w: enable clock stretching by peripheral
  --
  constant ctrl_twi_ack_c    : natural := 30; -- r/-: Set if ACK received
  constant ctrl_twi_busy_c   : natural := 31; -- r/-: Set if TWI unit is busy

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wr_en  : std_ulogic; -- word write enable
  signal rd_en  : std_ulogic; -- read enable

  -- twi clocking --
  signal twi_clk        : std_ulogic;
  signal twi_phase_gen  : std_ulogic_vector(3 downto 0);
  signal twi_clk_phase  : std_ulogic_vector(3 downto 0);

  -- twi clock stretching --
  signal twi_clk_halt : std_ulogic;

  -- twi transceiver core --
  signal ctrl         : std_ulogic_vector(7 downto 0); -- unit's control register
  signal arbiter      : std_ulogic_vector(2 downto 0);
  signal twi_bitcnt   : std_ulogic_vector(3 downto 0);
  signal twi_rtx_sreg : std_ulogic_vector(8 downto 0); -- main rx/tx shift reg

  -- tri-state I/O --
  signal twi_sda_i_ff0, twi_sda_i_ff1 : std_ulogic; -- sda input sync
  signal twi_scl_i_ff0, twi_scl_i_ff1 : std_ulogic; -- sda input sync
  signal twi_sda_i,     twi_sda_o     : std_ulogic;
  signal twi_scl_i,     twi_scl_o     : std_ulogic;

begin

  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = twi_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= twi_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
  wr_en  <= acc_en and wren_i;
  rd_en  <= acc_en and rden_i;


  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rw_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      ack_o <= acc_en and (rden_i or wren_i);
      -- write access --
      if (wr_en = '1') then
        if (addr = twi_ctrl_addr_c) then
          ctrl <= data_i(ctrl'left downto 0);
        end if;
      end if;
      -- read access --
      data_o <= (others => '0');
      if (rd_en = '1') then
        if (addr = twi_ctrl_addr_c) then
          data_o(ctrl_twi_en_c)     <= ctrl(ctrl_twi_en_c);
          data_o(ctrl_twi_prsc0_c)  <= ctrl(ctrl_twi_prsc0_c);
          data_o(ctrl_twi_prsc1_c)  <= ctrl(ctrl_twi_prsc1_c);
          data_o(ctrl_twi_prsc2_c)  <= ctrl(ctrl_twi_prsc2_c);
          data_o(ctrl_twi_mack_c)   <= ctrl(ctrl_twi_mack_c);
          data_o(ctrl_twi_cksten_c) <= ctrl(ctrl_twi_cksten_c);
          --
          data_o(ctrl_twi_ack_c)    <= not twi_rtx_sreg(0);
          data_o(ctrl_twi_busy_c)   <= arbiter(1) or arbiter(0);
        else -- twi_rtx_addr_c =>
          data_o(7 downto 0)        <= twi_rtx_sreg(8 downto 1);
          
        end if;
      end if;
    end if;
  end process rw_access;


  -- Clock Generation -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- clock generator enable --
  clkgen_en_o <= ctrl(ctrl_twi_en_c);

  -- main twi clock select --
  twi_clk <= clkgen_i(to_integer(unsigned(ctrl(ctrl_twi_prsc2_c downto ctrl_twi_prsc0_c))));

  -- generate four non-overlapping clock ticks at twi_clk/4 --
  clock_phase_gen: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (arbiter(2) = '0') or (arbiter = "100") then -- offline or idle
        twi_phase_gen <= "0001"; -- make sure to start with a new phase, 0,1,2,3 stepping
      elsif (twi_clk = '1') and (twi_clk_halt = '0') then -- enabled and no clock stretching detected
        twi_phase_gen <= twi_phase_gen(2 downto 0) & twi_phase_gen(3); -- shift left
      end if;
    end if;
  end process clock_phase_gen;

  twi_clk_phase(0) <= twi_phase_gen(0) and twi_clk; -- first step
  twi_clk_phase(1) <= twi_phase_gen(1) and twi_clk;
  twi_clk_phase(2) <= twi_phase_gen(2) and twi_clk;
  twi_clk_phase(3) <= twi_phase_gen(3) and twi_clk; -- last step


  -- TWI Transceiver ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  twi_rtx_unit: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- input synchronizer & sampler --
      twi_sda_i_ff0 <= twi_sda_i;
      twi_sda_i_ff1 <= twi_sda_i_ff0;
      twi_scl_i_ff0 <= twi_scl_i;
      twi_scl_i_ff1 <= twi_scl_i_ff0;

      -- defaults --
      irq_o      <= '0';
      arbiter(2) <= ctrl(ctrl_twi_en_c); -- still activated?

      -- serial engine --
      -- TWI bus signals are set/sampled using 4 clock phases
      case arbiter is

        when "100" => -- IDLE: waiting for requests, bus might be still claimed by this controller if no STOP condition was generated
          twi_bitcnt <= (others => '0');
          if (wr_en = '1') then
            if (addr = twi_ctrl_addr_c) then
              if (data_i(ctrl_twi_start_c) = '1') then -- issue START condition
                arbiter(1 downto 0) <= "01";
              elsif (data_i(ctrl_twi_stop_c) = '1') then  -- issue STOP condition
                arbiter(1 downto 0) <= "10";
              end if;
            elsif (addr = twi_rtx_addr_c) then -- start a data transmission
              -- one bit extra for ack, issued by controller if ctrl_twi_mack_c is set,
              -- sampled from peripheral if ctrl_twi_mack_c is cleared
              twi_rtx_sreg <= data_i(7 downto 0) & (not ctrl(ctrl_twi_mack_c));
              arbiter(1 downto 0) <= "11";
            end if;
          end if;

        when "101" => -- START: generate START condition
          if (twi_clk_phase(0) = '1') then
            twi_sda_o <= '1';
          elsif (twi_clk_phase(1) = '1') then
            twi_sda_o <= '0';
          end if;

          if (twi_clk_phase(0) = '1') then
            twi_scl_o <= '1';
          elsif (twi_clk_phase(3) = '1') then
            twi_scl_o <= '0';
            arbiter(1 downto 0) <= "00"; -- go back to IDLE
          end if;

        when "110" => -- STOP: generate STOP condition
          if (twi_clk_phase(0) = '1') then
            twi_sda_o <= '0';
          elsif (twi_clk_phase(3) = '1') then
            twi_sda_o <= '1';
            arbiter(1 downto 0) <= "00"; -- go back to IDLE
          end if;
          
          if (twi_clk_phase(0) = '1') then
            twi_scl_o <= '0';
          elsif (twi_clk_phase(1) = '1') then
            twi_scl_o <= '1';
          end if;

        when "111" => -- TRANSMISSION: transmission in progress
          if (twi_clk_phase(0) = '1') then
            twi_bitcnt   <= std_ulogic_vector(unsigned(twi_bitcnt) + 1);
            twi_scl_o    <= '0';
            twi_sda_o    <= twi_rtx_sreg(8); -- MSB first
          elsif (twi_clk_phase(1) = '1') then -- first half + second half of valid data strobe
            twi_scl_o    <= '1';
          elsif (twi_clk_phase(3) = '1') then
            twi_rtx_sreg <= twi_rtx_sreg(7 downto 0) & twi_sda_i_ff1; -- sample and shift left
            twi_scl_o    <= '0';
          end if;

          if (twi_bitcnt = "1010") then -- 8 data bits + 1 bit for ACK + 1 tick delay
            arbiter(1 downto 0) <= "00"; -- go back to IDLE
            irq_o <= '1'; -- fire IRQ
          end if;

        when others => -- "0--" OFFLINE: TWI deactivated
          twi_sda_o <= '1';
          twi_scl_o <= '1';
          arbiter   <= ctrl(ctrl_twi_en_c) & "00"; -- stay here, go to idle when activated

      end case;
    end if;
  end process twi_rtx_unit;


  -- Clock Stretching Detector --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  clock_stretching: process(ctrl, arbiter, twi_scl_o, twi_scl_i_ff1)
  begin
    -- clock stretching by the peripheral can happen at "any time"
    if (arbiter(2) = '1') and              -- module enabled
       (ctrl(ctrl_twi_cksten_c) = '1') and -- clock stretching enabled
       (twi_scl_o = '1') and               -- controller wants to pull scl high
       (twi_scl_i_ff1 = '0') then          -- but scl is pulled low by peripheral
      twi_clk_halt <= '1';
    else
      twi_clk_halt <= '0';
    end if;
  end process clock_stretching;


  -- Tri-State Driver -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- SDA and SCL need to be of type std_logic to be correctly resolved in simulation
  twi_sda_io <= '0' when (twi_sda_o = '0') else 'Z';
  twi_scl_io <= '0' when (twi_scl_o = '0') else 'Z';

  -- read-back --
  twi_sda_i <= std_ulogic(twi_sda_io);
  twi_scl_i <= std_ulogic(twi_scl_io);


end neorv32_twi_rtl;
