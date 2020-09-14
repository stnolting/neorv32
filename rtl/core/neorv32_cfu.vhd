-- #################################################################################################
-- # << NEORV32 - Custom Function Unit (CFU) >>                                                    #
-- # ********************************************************************************************* #
-- # For tightly-coupled custom co-processors. Provides four memory mapped interface registers.    #
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

entity neorv32_cfu is
  port (
    -- host access --
    clk_i       : in  std_ulogic; -- global clock line
    rstn_i      : in  std_ulogic; -- global reset line, low-active, use as async
    addr_i      : in  std_ulogic_vector(31 downto 0); -- address
    rden_i      : in  std_ulogic; -- read enable
    wren_i      : in  std_ulogic; -- write enable
    data_i      : in  std_ulogic_vector(31 downto 0); -- data in
    data_o      : out std_ulogic_vector(31 downto 0); -- data out
    ack_o       : out std_ulogic; -- transfer acknowledge
    -- clock generator --
    clkgen_en_o : out std_ulogic; -- enable clock generator
    clkgen_i    : in  std_ulogic_vector(07 downto 0); -- "clock" inputs
    -- interrupt --
    irq_o       : out std_ulogic
    -- custom io --
    -- ...
  );
end neorv32_cfu;

architecture neorv32_cfu_rtl of neorv32_cfu is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(cfu_size_c); -- low address boundary bit

  -- access control --
  signal acc_en : std_ulogic; -- module access enable
  signal addr   : std_ulogic_vector(31 downto 0); -- access address
  signal wr_en  : std_ulogic; -- word write enable
  signal rd_en  : std_ulogic; -- read enable

  -- default CFU interface registers --
  type cfu_regs_t is array (0 to 3) of std_ulogic_vector(31 downto 0);
  signal cfu_reg_in  : cfu_regs_t; -- interface registers for WRITE
  signal cfu_reg_out : cfu_regs_t; -- interface registers for READ

begin

  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- These assignments are required to check if the CFU is accessed at all.
  -- Do NOT modify this for your custom application (unless you really know what you are doing).

  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = cfu_base_c(hi_abb_c downto lo_abb_c)) else '0';
  addr   <= cfu_base_c(31 downto lo_abb_c) & addr_i(lo_abb_c-1 downto 2) & "00"; -- word aligned
  wr_en  <= acc_en and wren_i;
  rd_en  <= acc_en and rden_i;


  -- Clock System ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- The top unit implements a clock generator providing 8 "derived clocks"
  -- Actually, these signals must not be used as direct clock signals, but as clock enable signals.

  -- The following clock divider rates are available:
  -- clkgen_i(clk_div2_c)    -> MAIN_CLK/2
  -- clkgen_i(clk_div4_c)    -> MAIN_CLK/4
  -- clkgen_i(clk_div8_c)    -> MAIN_CLK/8
  -- clkgen_i(clk_div64_c)   -> MAIN_CLK/64
  -- clkgen_i(clk_div128_c)  -> MAIN_CLK/128
  -- clkgen_i(clk_div1024_c) -> MAIN_CLK/1024
  -- clkgen_i(clk_div2048_c) -> MAIN_CLK/2048
  -- clkgen_i(clk_div4096_c) -> MAIN_CLK/4096

  -- For instance, if you want to drive a system at MAIN_CLK/8, you can use the following construct:

  -- if (rstn_i = '0') then -- async and low-active reset
  -- ...
  -- elsif rising_edge(clk_i) then -- Always use the main clock for all clock processes!
  --   if (clkgen_i(clk_div8_c) = '1') then -- the div8 "clock" is actually a clock enable
  --     ...
  --   end if;
  -- end if;

  -- The clkgen_i input clocks are available when at least one IO/peripheral device requires the clocks generated by the
  -- clock generator. The CFU can enable the clock generator via the clkgen_en_o signal.
  -- Make sure to deactivate clkgen_en_o if no clocks are required to reduce power consumption.

  clkgen_en_o <= '0'; -- not used for this minimal example


  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- Here we are reading/writing from/to the interface registers of the module. Please note that the peripheral/IO
  -- modules of the NEORV32 can only be written in full word mode (32-bit). Any other write access (half-word or byte)
  -- will trigger a store access fault exception.
  --
  -- All register of every unit are cleared during the processor boot sequence by the default crt0.S code.
  -- Make cfu_reg0_addr_c the CFU's control register. This register is cleared first during booting.
  -- If the control register is cleared, no actions should be taken when writing to other CFU registers.
  --
  -- The CFU provides 4 memory-mapped interface register. For instance, these could be used to provide
  -- a <status register> for status information, a <data register< for reading/writing from/to a data FIFO, a <command register>
  -- for issueing commands and a <control register> for global control of the unit.
  --
  rw_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- transfer ack --
      ack_o <= wr_en or rd_en;

      -- write access --
      if (wr_en = '1') then
        if (addr = cfu_reg0_addr_c) then -- this should be the control register
          cfu_reg_in(0) <= data_i; -- for example: control register
        end if;
        if (addr = cfu_reg1_addr_c) then
          cfu_reg_in(1) <= data_i; -- for example: data in/out fifo
        end if;
        if (addr = cfu_reg2_addr_c) then
          cfu_reg_in(2) <= data_i; -- for example: command fifo
        end if;
        if (addr = cfu_reg3_addr_c) then
          cfu_reg_in(3) <= data_i; -- for example: status register
        end if;
      end if;

      -- read access --
      data_o <= (others => '0'); -- make sure the output is zero if there is no actual read access
      if (rd_en = '1') then
        if (addr = cfu_reg0_addr_c) then
          data_o <= cfu_reg_out(0);
        elsif (addr = cfu_reg1_addr_c) then
          data_o <= cfu_reg_out(1);
        elsif (addr = cfu_reg2_addr_c) then
          data_o <= cfu_reg_out(2);
        else -- addr = cfu_reg3_addr_c
          data_o <= cfu_reg_out(3);
        end if;
      end if;
    end if;
  end process rw_access;


  -- CFU Interrupt --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- The CFU provides a single interrupt request signal, which is forwarded to the CPU's fast interrupt channel 1.
  -- This channel is shared with the GPIO unit - so both unit can trigger the same interrupt.
  --
  -- An interrupt request is generated when the irq_o signal is high for one cycle. If the signal is high for more than one cycle
  -- more than one interrupt request might be generated.
  --
  -- There is no interrupt acknowledge signal. If required: The interrupt handler can write to a specific control reister bit within the
  -- CFU to acknowledge an interrupt.

  irq_o <= '0'; -- not used for this minimal example


  -- CFU Core -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- This is where the actual functionality can be implemented. In this example we are just doing some pointless operations.

  cfu_reg_out(0) <= cfu_reg_in(0) and cfu_reg_in(1);
  cfu_reg_out(1) <= cfu_reg_in(2) and cfu_reg_in(3);
  cfu_reg_out(2) <= x"00000001";
  cfu_reg_out(3) <= (others => '0');


end neorv32_cfu_rtl;
