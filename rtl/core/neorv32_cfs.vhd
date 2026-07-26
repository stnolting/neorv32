-- ================================================================================ --
-- NEORV32 SoC - Custom Functions Subsystem (CFS)                                   --
-- -------------------------------------------------------------------------------- --
-- Use this module to implement custom memory-mapped co-processors or interfaces.   --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cfs is
  port (
    -- global control --
    clk_i      : in  std_ulogic;                      -- global clock
    rstn_i     : in  std_ulogic;                      -- global reset, low-active, async
    -- CPU request --
    req_addr_i : in  std_ulogic_vector(15 downto 0);  -- byte address (64kB address space)
    req_data_i : in  std_ulogic_vector(31 downto 0);  -- write data
    req_ben_i  : in  std_ulogic_vector(3 downto 0);   -- byte enable
    req_stb_i  : in  std_ulogic;                      -- access request strobe
    req_rw_i   : in  std_ulogic;                      -- 0=read, 1=write
    -- CPU response --
    rsp_data_o : out std_ulogic_vector(31 downto 0);  -- read data
    rsp_ack_o  : out std_ulogic;                      -- access acknowledge
    -- CPU interrupt --
    irq_o      : out std_ulogic;                      -- interrupt request, high-active
    -- external IO --
    cfs_in_i   : in  std_ulogic_vector(255 downto 0); -- custom inputs conduit
    cfs_out_o  : out std_ulogic_vector(255 downto 0)  -- custom outputs conduit
  );
end entity;

architecture neorv32_cfs_rtl of neorv32_cfs is

  -- exemplary CFS interface registers --
  type cfs_regs_t is array (0 to 3) of std_ulogic_vector(31 downto 0); -- implement 4 read/write registers
  signal cfs_reg_wr : cfs_regs_t; -- for WRITE accesses
  signal cfs_reg_rd : cfs_regs_t; -- for READ accesses

begin

  -- CFS IOs --------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- By default, the CFS provides two IO ports (cfs_in_i and cfs_out_o) that are available at the processor's top entity.
  -- These are intended as "conduits" to propagate custom CFS signals between the CFS and the processor top entity.

  cfs_out_o <= (others => '0'); -- not used for this minimal example

  -- Interrupt ------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- The CFS features a single interrupt signal, which is connected to the CPU's "fast interrupt" channel 1 (FIRQ1).
  -- The according CPU interrupt becomes pending as long as <irq_o> is high.

  irq_o <= '0'; -- not used for this minimal example

  -- Read/Write Access ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- The CFS provides up to 64kB of memory-mapped address space (16 address bits, byte-addressing) that can be used
  -- for custom memories and interface registers. According to the CPU's bus protocol, each read or write access has
  -- to be acknowledged in the following cycle using the <rsp_ack_o> signal (or even later if the module needs
  -- additional time to complete the access). If no ACK is generated, the bus access will time out causing a bus access
  -- fault exception (this can also be done intentionally to explicitly throw an access exception).
  --
  -- [EXAMPLE] Read and write access to the interface registers and bus transfer acknowledge. This example only four
  -- physical 32-bit read/write register (using the four lowest CFS address bits). The remaining addresses of the CFS
  -- are not associated with any physical registers - any access to those is simply ignored but still acknowledged;
  -- read accesses will return zero. Only full-word write accesses are supported by this example.
  -- Sub-word write accesses are ignored but still acknowledged.

  host_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      cfs_reg_wr <= (others => (others => '0'));
      rsp_data_o <= (others => '0');
      rsp_ack_o  <= '0';
    elsif rising_edge(clk_i) then -- synchronous interface for read and write accesses
      -- transfer/access acknowledge --
      rsp_ack_o <= req_stb_i; -- send ACK right after the access request

      -- bus access --
      rsp_data_o <= (others => '0'); -- the output HAS TO BE ZERO if there is no actual (read) access
      if (req_stb_i = '1') then -- valid access cycle, STB is high for exactly one cycle

        -- write access (word-wise) --
        if (req_rw_i = '1') then
          if (req_addr_i(15 downto 2) = "00000000000000") then -- 16-bit byte address = 14-bit word address
            cfs_reg_wr(0) <= req_data_i;
          end if;
          if (req_addr_i(15 downto 2) = "00000000000001") then
            cfs_reg_wr(1) <= req_data_i;
          end if;
          if (req_addr_i(15 downto 2) = "00000000000010") then
            cfs_reg_wr(2) <= req_data_i;
          end if;
          if (req_addr_i(15 downto 2) = "00000000000011") then
            cfs_reg_wr(3) <= req_data_i;
          end if;

        -- read access (word-wise) --
        else
          case req_addr_i(15 downto 2) is -- 16-bit byte-address = 14-bit word-address
            when "00000000000000" => rsp_data_o <= cfs_reg_rd(0);
            when "00000000000001" => rsp_data_o <= cfs_reg_rd(1);
            when "00000000000010" => rsp_data_o <= cfs_reg_rd(2);
            when "00000000000011" => rsp_data_o <= cfs_reg_rd(3);
            when others           => rsp_data_o <= (others => '0');
          end case;
        end if;

      end if;
    end if;
  end process;

  -- CFS Function Core ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- This is where the actual functionality can be implemented. The logic below is just a very
  -- simple example that transforms data from an input register into data in an output register.

  cfs_reg_rd(0) <= x"0000000" & "000" & or_reduce_f(cfs_reg_wr(0));  -- OR all bits
  cfs_reg_rd(1) <= x"0000000" & "000" & xor_reduce_f(cfs_reg_wr(1)); -- XOR all bits
  cfs_reg_rd(2) <= bit_rev_f(cfs_reg_wr(2));                         -- bit reversal
  cfs_reg_rd(3) <= cfs_reg_wr(3);                                    -- pass-through

end architecture;
