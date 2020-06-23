-- #################################################################################################
-- # << NEORV32 - Core Local Interrupt Controller (CLIC) >>                                        #
-- # ********************************************************************************************* #
-- # This unit provides 8 maskable interrupt lines with according ACK lines. The IRQ               #
-- # triggers on a high level (use external edge detectors if required). Each line has a unique    #
-- # enable bit. The acknowledge output is set high for one clock cycle to confirm the             #
-- # interrupt has been sampled and has also been cpatured by the according handler function.      #
-- # All external interrupt requests are forwarded to the CPU's MACHINE EXTERNAL IRQ trigger.      #
-- # If several IRQs occur at the same time, the one with highest priority is executed while the   #
-- # others are kept in a buffer. A pending IRQ can be deleted by cvlearing the IRQs IRQ enable    #
-- # bit. ext_irq_i(0) has highest priority while ext_irq_i(7) has the lowest priority.            #
-- # An IRQ is acknowledged when the CPU reads the CLIC control register.                          #
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

entity neorv32_clic is
  port (
    -- host access --
    clk_i     : in  std_ulogic; -- global clock line
    rden_i    : in  std_ulogic; -- read enable
    wren_i    : in  std_ulogic; -- write enable
    ben_i     : in  std_ulogic_vector(03 downto 0); -- byte write enable
    addr_i    : in  std_ulogic_vector(31 downto 0); -- address
    data_i    : in  std_ulogic_vector(31 downto 0); -- data in
    data_o    : out std_ulogic_vector(31 downto 0); -- data out
    ack_o     : out std_ulogic; -- transfer acknowledge
    -- cpu interrupt --
    cpu_irq_o : out std_ulogic; -- trigger CPU's external IRQ
    -- external interrupt lines --
    ext_irq_i : in  std_ulogic_vector(07 downto 0); -- IRQ, triggering on HIGH level
    ext_ack_o : out std_ulogic_vector(07 downto 0)  -- acknowledge
  );
end neorv32_clic;

architecture neorv32_clic_rtl of neorv32_clic is

  -- control register bits --
  constant ctrl_irq_src0_c    : natural :=  0; -- r/-: IRQ source bit 0
  constant ctrl_irq_src1_c    : natural :=  1; -- r/-: IRQ source bit 1
  constant ctrl_irq_src2_c    : natural :=  2; -- r/-: IRQ source bit 2
  constant ctrl_ack_c         : natural :=  3; -- -/w: ACK IRQ, auto-clears
  constant ctrl_en_c          : natural :=  4; -- r/w: unit enable
  --
  constant ctrl_en_irq0_c     : natural :=  8; -- r/w: IRQ channel 0 enable
  constant ctrl_en_irq1_c     : natural :=  9; -- r/w: IRQ channel 1 enable
  constant ctrl_en_irq2_c     : natural := 10; -- r/w: IRQ channel 2 enable
  constant ctrl_en_irq3_c     : natural := 11; -- r/w: IRQ channel 3 enable
  constant ctrl_en_irq4_c     : natural := 12; -- r/w: IRQ channel 4 enable
  constant ctrl_en_irq5_c     : natural := 13; -- r/w: IRQ channel 5 enable
  constant ctrl_en_irq6_c     : natural := 14; -- r/w: IRQ channel 6 enable
  constant ctrl_en_irq7_c     : natural := 15; -- r/w: IRQ channel 7 enable
  --
  constant ctrl_sw_irq_src0_c : natural := 16; -- -/w: IRQ (SW) trigger select bit 0, auto-clears
  constant ctrl_sw_irq_src1_c : natural := 17; -- -/w: IRQ (SW) trigger select bit 1, auto-clears
  constant ctrl_sw_irq_src2_c : natural := 18; -- -/w: IRQ (SW) trigger select bit 2, auto-clears
  constant ctrl_sw_irq_en_c   : natural := 19; -- -/w: IRQ (SW) trigger enable, auto-clears

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(clic_size_c); -- low address boundary bit

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal wren   : std_ulogic; -- full word write enable
  signal rden   : std_ulogic; -- read enable

  -- r/w accessible registers --
  signal irq_enable  : std_ulogic_vector(7 downto 0); -- r/w: IRQ channel enable
  signal irq_src_reg : std_ulogic_vector(2 downto 0); -- r/-: currently triggered IRQ
  signal enable      : std_ulogic; -- r/w: unit enable

  -- sw irq trigger --
  signal sw_trig_src : std_ulogic_vector(2 downto 0); -- -/w: IRQ channel to be triggered by software
  signal sw_trig_en  : std_ulogic; -- -/w: software IRQ trigger enable

  -- irq input / ack output system --
  signal irq_raw, irq_sw, irq_valid, ack_mask : std_ulogic_vector(7 downto 0);

  -- controller core --
  signal irq_buf  : std_ulogic_vector(7 downto 0);
  signal irq_src  : std_ulogic_vector(2 downto 0);
  signal irq_fire : std_ulogic;
  signal cpu_ack  : std_ulogic;
  signal state    : std_ulogic;

begin

  -- Access control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = clic_base_c(hi_abb_c downto lo_abb_c)) else '0';
  wren   <= acc_en and wren_i;
  rden   <= acc_en and rden_i;


  -- Read/Write access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rw_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      ack_o <= acc_en and (rden_i or wren_i);
      -- write access --
      sw_trig_en  <= '0';
      cpu_ack     <= '0';
      if (wren = '1') then
        if (ben_i(0) = '1') then
          cpu_ack <= data_i(ctrl_ack_c);
          enable  <= data_i(ctrl_en_c);
        end if;
        if (ben_i(1) = '1') then
          irq_enable <= data_i(ctrl_en_irq7_c downto ctrl_en_irq0_c);
        end if;
        if (ben_i(2) = '1') then
          sw_trig_src <= data_i(ctrl_sw_irq_src2_c downto ctrl_sw_irq_src0_c);
          sw_trig_en  <= data_i(ctrl_sw_irq_en_c);
        end if;
      end if;
      -- read access --
      data_o <= (others => '0');
      if (rden = '1') then
        data_o(ctrl_en_c) <= enable;
        data_o(ctrl_irq_src2_c downto ctrl_irq_src0_c) <= irq_src_reg;
        data_o(ctrl_en_irq7_c downto ctrl_en_irq0_c) <= irq_enable;
      end if;
    end if;
  end process rw_access;


  -- SW IRQ Trigger ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  sw_irq_trigger: process(sw_trig_en, sw_trig_src)
    variable sw_irq_sel_v : std_ulogic_vector(3 downto 0);
  begin
    sw_irq_sel_v := sw_trig_en & sw_trig_src;
    case sw_irq_sel_v is
      when "1000" => irq_sw <= "00000001";
      when "1001" => irq_sw <= "00000010";
      when "1010" => irq_sw <= "00000100";
      when "1011" => irq_sw <= "00001000";
      when "1100" => irq_sw <= "00010000";
      when "1101" => irq_sw <= "00100000";
      when "1110" => irq_sw <= "01000000";
      when "1111" => irq_sw <= "10000000";
      when others => irq_sw <= "00000000";
    end case;
  end process sw_irq_trigger;


  -- Get external/software interrupt request ------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ext_irq_source_sync: process(clk_i)
  begin
    if rising_edge(clk_i) then
      irq_raw <= ext_irq_i;
    end if;
  end process ext_irq_source_sync;

  -- actual IRQ --
  irq_valid <= irq_raw or irq_sw;


  -- IRQ controller core --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_core: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- ack output --
      ext_ack_o <= ack_mask;
      -- irq buffer --
      for i in 0 to 7 loop
        -- keep requests until they are acknowledged, clear request when channel is disabled
        irq_buf(i) <= (irq_buf(i) or irq_valid(i)) and (not ack_mask(i)) and irq_enable(i);
      end loop; -- i
      -- mini state FSM --
      cpu_irq_o <= '0';
      if (state = '0') or (enable = '0') then -- idle or all deactivated
        state       <= '0';
        irq_src_reg <= (others => '0');
        if (irq_fire = '1') then -- valid active IRQ
          irq_src_reg <= irq_src; -- capture source
          cpu_irq_o   <= '1'; -- trigger CPU interrupt
          state       <= '1'; -- go to active IRQ state
        end if;
      else -- active interrupt request
        if (cpu_ack = '1') then -- wait for ack from cpu
          state <= '0';
        end if;
      end if;
    end if;
  end process irq_core;

  -- anybody firing? --
  irq_fire <= or_all_f(irq_buf);

  -- get interrupt priority --
  irq_src <= "000" when (irq_buf(0) = '1') else
             "001" when (irq_buf(1) = '1') else
             "010" when (irq_buf(2) = '1') else
             "011" when (irq_buf(3) = '1') else
             "100" when (irq_buf(4) = '1') else
             "101" when (irq_buf(5) = '1') else
             "110" when (irq_buf(6) = '1') else
             "111";-- when (irq_buf(7) = '1') else "---";


  -- ACK priority decoder -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ack_priority_dec: process(state, cpu_ack, irq_src_reg)
    variable irq_ack_v : std_ulogic_vector(3 downto 0);
  begin
    irq_ack_v := (cpu_ack and state) & irq_src_reg;
    case irq_ack_v is
      when "1000" => ack_mask <= "00000001";
      when "1001" => ack_mask <= "00000010";
      when "1010" => ack_mask <= "00000100";
      when "1011" => ack_mask <= "00001000";
      when "1100" => ack_mask <= "00010000";
      when "1101" => ack_mask <= "00100000";
      when "1110" => ack_mask <= "01000000";
      when "1111" => ack_mask <= "10000000";
      when others => ack_mask <= "00000000";
    end case;
  end process ack_priority_dec;


end neorv32_clic_rtl;
