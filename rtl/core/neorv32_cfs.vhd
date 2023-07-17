-- #################################################################################################
-- # << NEORV32 - Custom Functions Subsystem (CFS) >>                                              #
-- # ********************************************************************************************* #
-- # Intended for tightly-coupled, application-specific custom co-processors. This module provides #
-- # 64x 32-bit memory-mapped interface registers, one interrupt request signal and custom IO      #
-- # conduits for processor-external or chip-external interface.                                   #
-- #                                                                                               #
-- # NOTE: This is just an example/illustration template. Modify/replace this file to implement    #
-- #       your own custom design logic.                                                           #
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

entity neorv32_cfs is
  generic (
    CFS_CONFIG   : std_ulogic_vector(31 downto 0); -- custom CFS configuration generic
    CFS_IN_SIZE  : natural; -- size of CFS input conduit in bits
    CFS_OUT_SIZE : natural  -- size of CFS output conduit in bits
  );
  port (
    clk_i       : in  std_ulogic; -- global clock line
    rstn_i      : in  std_ulogic; -- global reset line, low-active, use as async
    bus_req_i   : in  bus_req_t;  -- bus request
    bus_rsp_o   : out bus_rsp_t;  -- bus response
    clkgen_en_o : out std_ulogic; -- enable clock generator
    clkgen_i    : in  std_ulogic_vector(07 downto 0); -- "clock" inputs
    irq_o       : out std_ulogic; -- interrupt request
    cfs_in_i    : in  std_ulogic_vector(CFS_IN_SIZE-1 downto 0); -- custom inputs
    cfs_out_o   : out std_ulogic_vector(CFS_OUT_SIZE-1 downto 0) -- custom outputs
  );
end neorv32_cfs;

architecture neorv32_cfs_rtl of neorv32_cfs is

  -- default CFS interface registers --
  type cfs_regs_t is array (0 to 3) of std_ulogic_vector(31 downto 0); -- just implement 4 registers for this example
  signal cfs_reg_wr : cfs_regs_t; -- interface registers for WRITE accesses
  signal cfs_reg_rd : cfs_regs_t; -- interface registers for READ accesses

begin

  -- CFS Generics ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- In it's default version the CFS provides three configuration generics:
  -- > CFS_IN_SIZE  - configures the size (in bits) of the CFS input conduit cfs_in_i
  -- > CFS_OUT_SIZE - configures the size (in bits) of the CFS output conduit cfs_out_o
  -- > CFS_CONFIG   - is a blank 32-bit generic. It is intended as a "generic conduit" to propagate
  --                  custom configuration flags from the top entity down to this module.


  -- CFS IOs --------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- By default, the CFS provides two IO signals (cfs_in_i and cfs_out_o) that are available at the processor's top entity.
  -- These are intended as "conduits" to propagate custom signals from this module and the processor top entity.

  cfs_out_o <= (others => '0'); -- not used for this minimal example


  -- Reset System ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- The CFS can be reset using the global rstn_i signal. This signal should be used as asynchronous reset and is active-low.
  -- Note that rstn_i can be asserted by a processor-external reset, the on-chip debugger and also by the watchdog.
  --
  -- Most default peripheral devices of the NEORV32 do NOT use a dedicated hardware reset at all. Instead, these units are
  -- reset by writing ZERO to a specific "control register" located right at the beginning of the device's address space
  -- (so this register is cleared at first). The crt0 start-up code writes ZERO to every single address in the processor's
  -- IO space - including the CFS. Make sure that this initial clearing does not cause any unintended CFS actions.


  -- Clock System ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- The processor top unit implements a clock generator providing 8 "derived clocks".
  -- Actually, these signals should not be used as direct clock signals, but as *clock enable* signals.
  -- clkgen_i is always synchronous to the main system clock (clk_i).
  --
  -- The following clock dividers are available:
  -- > clkgen_i(clk_div2_c)    -> MAIN_CLK/2
  -- > clkgen_i(clk_div4_c)    -> MAIN_CLK/4
  -- > clkgen_i(clk_div8_c)    -> MAIN_CLK/8
  -- > clkgen_i(clk_div64_c)   -> MAIN_CLK/64
  -- > clkgen_i(clk_div128_c)  -> MAIN_CLK/128
  -- > clkgen_i(clk_div1024_c) -> MAIN_CLK/1024
  -- > clkgen_i(clk_div2048_c) -> MAIN_CLK/2048
  -- > clkgen_i(clk_div4096_c) -> MAIN_CLK/4096
  --
  -- For instance, if you want to drive a clock process at MAIN_CLK/8 clock speed you can use the following construct:
  --
  --   if (rstn_i = '0') then -- async and low-active reset (if required at all)
  --   ...
  --   elsif rising_edge(clk_i) then -- always use the main clock for all clock processes
  --     if (clkgen_i(clk_div8_c) = '1') then -- the div8 "clock" is actually a clock enable
  --       ...
  --     end if;
  --   end if;
  --
  -- The clkgen_i input clocks are available when at least one IO/peripheral device (for example UART0) requires the clocks
  -- generated by the clock generator. The CFS can enable the clock generator by itself by setting the clkgen_en_o signal high.
  -- The CFS cannot ensure to deactivate the clock generator by setting the clkgen_en_o signal low as other peripherals might
  -- still keep the generator activated. Make sure to deactivate the CFS's clkgen_en_o if no clocks are required in here to
  -- reduce dynamic power consumption.

  clkgen_en_o <= '0'; -- not used for this minimal example


  -- Interrupt ------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- The CFS features a single interrupt signal, which is connected to the CPU's "fast interrupt" channel 1 (FIRQ1).
  -- The interrupt is triggered by a one-cycle high-level. After triggering, the interrupt appears as "pending" in the CPU's
  -- mip CSR ready to trigger execution of the according interrupt handler. It is the task of the application to programmer
  -- to enable/clear the CFS interrupt using the CPU's mie and mip registers when required.

  irq_o <= '0'; -- not used for this minimal example


  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- Here we are reading/writing from/to the interface registers of the module and generate the CPU access handshake (bus response).
  --
  -- The CFS provides up to 64 memory-mapped 32-bit interface registers. For instance, these could be used to provide a
  -- <control register> for global control of the unit, a <data register> for reading/writing from/to a data FIFO, a
  -- <command register> for issuing commands and a <status register> for status information.
  --
  -- Following the interface protocol, each read or write access has to be acknowledged in the following cycle using the ack_o
  -- signal (or even later if the module needs additional time). If no ACK is generated at all, the bus access will time out
  -- and cause a bus access fault exception. The current CPU privilege level is available via the 'priv_i' signal (0 = user mode,
  -- 1 = machine mode), which can be used to constrain access to certain registers or features to privileged software only.
  --
  -- This module also provides an optional ERROR signal to indicate a faulty access operation (for example when accessing an
  -- unused, read-only or "locked" CFS register address). This signal may only be set when the module is actually accessed
  -- and is set INSTEAD of the ACK signal. Setting the ERR signal will raise a bus access exception with a "Device Error" qualifier
  -- that can be handled by the application software. Note that the current privilege level should not be exposed to software to
  -- maintain full virtualization. Hence, CFS-based "privilege escalation" should trigger a bus access exception (e.g. by setting 'err_o').

  bus_rsp_o.err <= '0'; -- Tie to zero if not explicitly used.


  -- Host access example: Read and write access to the interface registers + bus transfer acknowledge. This example only
  -- implements four physical r/w register (the four lowest CFS registers). The remaining addresses of the CFS are not associated
  -- with any physical registers - any access to those is simply ignored but still acknowledged. Only full-word write accesses are
  -- supported (and acknowledged) by this example. Sub-word write access will not alter any CFS register state and will cause
  -- a "bus store access" exception (with a "Device Timeout" qualifier as not ACK is generated in that case).

  host_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      cfs_reg_wr(0) <= (others => '0');
      cfs_reg_wr(1) <= (others => '0');
      cfs_reg_wr(2) <= (others => '0');
      cfs_reg_wr(3) <= (others => '0');
      --
      bus_rsp_o.ack  <= '0';
      bus_rsp_o.data <= (others => '0');
    elsif rising_edge(clk_i) then -- synchronous interface for read and write accesses
      -- transfer/access acknowledge --
      -- default: required for the CPU to check the CFS is answering a bus read OR write request;
      -- all read and write accesses (to any cfs_reg, even if there is no according physical register implemented) will succeed.
      bus_rsp_o.ack <= bus_req_i.re or bus_req_i.we;

      -- write access --
      if (bus_req_i.we = '1') then -- full-word write access, high for one cycle if there is an actual write access
        if (bus_req_i.addr(7 downto 2) = "000000") then -- make sure to use the internal "addr" signal for the read/write interface
          cfs_reg_wr(0) <= bus_req_i.data; -- some physical register, for example: control register
        end if;
        if (bus_req_i.addr(7 downto 2) = "000001") then
          cfs_reg_wr(1) <= bus_req_i.data; -- some physical register, for example: data in/out fifo
        end if;
        if (bus_req_i.addr(7 downto 2) = "000010") then
          cfs_reg_wr(2) <= bus_req_i.data; -- some physical register, for example: command fifo
        end if;
        if (bus_req_i.addr(7 downto 2) = "000011") then
          cfs_reg_wr(3) <= bus_req_i.data; -- some physical register, for example: status register
        end if;
      end if;

      -- read access --
      bus_rsp_o.data <= (others => '0'); -- the output HAS TO BE ZERO if there is no actual read access
      if (bus_req_i.re = '1') then -- the read access is always 32-bit wide, high for one cycle if there is an actual read access
        case bus_req_i.addr(7 downto 2) is -- make sure to use the internal 'addr' signal for the read/write interface
          when "000000" => bus_rsp_o.data <= cfs_reg_rd(0);
          when "000001" => bus_rsp_o.data <= cfs_reg_rd(1);
          when "000010" => bus_rsp_o.data <= cfs_reg_rd(2);
          when "000011" => bus_rsp_o.data <= cfs_reg_rd(3);
          when others   => bus_rsp_o.data <= (others => '0'); -- the remaining registers are not implemented and will read as zero
        end case;
      end if;
    end if;
  end process host_access;


  -- CFS Function Core ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- This is where the actual functionality can be implemented.
  -- The logic below is just a very simple example that transforms data
  -- from an input register into data in an output register.

  cfs_reg_rd(0) <= bin_to_gray_f(cfs_reg_wr(0)); -- convert binary to gray code
  cfs_reg_rd(1) <= gray_to_bin_f(cfs_reg_wr(1)); -- convert gray to binary code
  cfs_reg_rd(2) <= bit_rev_f(cfs_reg_wr(2)); -- bit reversal
  cfs_reg_rd(3) <= bswap32_f(cfs_reg_wr(3)); -- byte swap (endianness conversion)


end neorv32_cfs_rtl;
