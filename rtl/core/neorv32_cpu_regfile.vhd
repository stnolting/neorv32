-- ================================================================================ --
-- NEORV32 CPU - Data Register File                                                 --
-- -------------------------------------------------------------------------------- --
-- The architecture style of the register file is selected by the ARCHSEL generic:  --
-- 0: Register-based SRAM with sync. read (e.g. to map to FPGA block RAM)           --
-- 1: Register-based SRAM with async. read (e.g. to map to FPGA distributed RAM)    --
-- 2: Register-based with full hardware reset                                       --
-- 3: Latch-based (e.g. for ASIC implementation)                                    --
--                                                                                  --
-- [NOTE] Read-during-write behavior of the register file's memory core is          --
--        irrelevant as read and write accesses are mutually exclusive.             --
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

entity neorv32_cpu_regfile is
  generic (
    AWIDTH  : natural range 4 to 5; -- address width
    ARCHSEL : natural range 0 to 3  -- architecture style select
  );
  port (
    -- global control --
    clk_i      : in  std_ulogic; -- global clock, rising edge
    rstn_i     : in  std_ulogic; -- global reset, low-active, async
    zero_i     : in  std_ulogic; -- force write to x0 (for SRAM memory only)
    -- write port (rd) --
    rd_we_i    : in  std_ulogic;                     -- write-enable
    rd_addr_i  : in  std_ulogic_vector(4 downto 0);  -- address
    rd_data_i  : in  std_ulogic_vector(31 downto 0); -- write data
    -- read port 1 (rs1) --
    rs1_addr_i : in  std_ulogic_vector(4 downto 0);  -- address
    rs1_data_o : out std_ulogic_vector(31 downto 0); -- read data
    -- read port 1 (rs2) --
    rs2_addr_i : in  std_ulogic_vector(4 downto 0);  -- address
    rs2_data_o : out std_ulogic_vector(31 downto 0)  -- read data
  );
end entity;

architecture neorv32_cpu_regfile_rtl of neorv32_cpu_regfile is

  -- access logic --
  signal rf_we  : std_ulogic;
  signal addr   : std_ulogic_vector(4 downto 0);
  signal wdata  : std_ulogic_vector(31 downto 0);
  signal onehot : std_ulogic_vector((2**AWIDTH)-1 downto 1);

  -- memory core --
  type   regfile_t is array ((2**AWIDTH)-1 downto 0) of std_ulogic_vector(31 downto 0);
  signal regfile : regfile_t;

begin

  -- Architecture Style 0: Register-Based SRAM with Synchronous Read ------------------------
  -- -------------------------------------------------------------------------------------------
  arch_sram_sync:
  if (ARCHSEL = 0) generate

    -- Register zero (x0) is just another physical register that has to be initialized by the CPU control.
    -- Writes to x0 are inhibited unless the control forces a write (writing zero) to re-initialize x0.
    rf_we <= (rd_we_i and or_reduce_f(rd_addr_i(AWIDTH-1 downto 0))) or zero_i;
    addr  <= (others => '0') when (zero_i  = '1') else -- force rd = zero
             rd_addr_i       when (rd_we_i = '1') else rs1_addr_i; -- multiplexed rd/rs1

    -- synchronous write & read (SDPRAM) --
    rf_access: process(clk_i)
    begin
      if rising_edge(clk_i) then
        if (rf_we = '1') then
          regfile(to_integer(unsigned(addr(AWIDTH-1 downto 0)))) <= rd_data_i;
        end if;
        rs1_data_o <= regfile(to_integer(unsigned(addr(AWIDTH-1 downto 0))));
        rs2_data_o <= regfile(to_integer(unsigned(rs2_addr_i(AWIDTH-1 downto 0))));
      end if;
    end process;

    -- unused --
    wdata  <= (others => '0');
    onehot <= (others => '0');

  end generate;


  -- Architecture Style 1: Register-Based SRAM with Asynchronous Read -----------------------
  -- -------------------------------------------------------------------------------------------
  arch_sram_async:
  if (ARCHSEL = 1) generate

    -- multiplexed rd/rs1 address to map to SDPRAM --
    addr <= rd_addr_i when (rd_we_i = '1') else rs1_addr_i;

    -- synchronous write --
    rf_write: process(clk_i)
    begin
      if rising_edge(clk_i) then
        if (rd_we_i = '1') then
          regfile(to_integer(unsigned(addr(AWIDTH-1 downto 0)))) <= rd_data_i;
        end if;
      end if;
    end process;

    -- asynchronous read + zero-insertion + output-register --
    rf_read: process(clk_i)
    begin
      if rising_edge(clk_i) then
        if (rs1_addr_i = "00000") then -- reading x0
          rs1_data_o <= (others => '0');
        else
          rs1_data_o <= regfile(to_integer(unsigned(addr(AWIDTH-1 downto 0))));
        end if;
        if (rs2_addr_i = "00000") then -- reading x0
          rs2_data_o <= (others => '0');
        else
          rs2_data_o <= regfile(to_integer(unsigned(rs2_addr_i(AWIDTH-1 downto 0))));
        end if;
      end if;
    end process;

    -- unused --
    rf_we  <= '0';
    wdata  <= (others => '0');
    onehot <= (others => '0');

  end generate;


  -- Architecture Style 2: Register-Based with Hardware Reset -------------------------------
  -- -------------------------------------------------------------------------------------------
  arch_reg:
  if (ARCHSEL = 2) generate

    -- write select --
    onehot_gen:
    for i in 1 to (2**AWIDTH)-1 generate
      onehot(i) <= rd_we_i when (unsigned(rd_addr_i(AWIDTH-1 downto 0)) = to_unsigned(i, AWIDTH)) else '0';
    end generate;

    -- individual registers with reset --
    regfile_gen:
    for i in 1 to (2**AWIDTH)-1 generate
      rf_write: process(rstn_i, clk_i)
      begin
        if (rstn_i = '0') then
          regfile(i) <= (others => '0');
        elsif rising_edge(clk_i) then
          if (onehot(i) = '1') then
            regfile(i) <= rd_data_i;
          end if;
        end if;
      end process;
    end generate;
    regfile(0) <= (others => '0'); -- x0 is hardwired to zero

    -- synchronous read --
    rf_read: process(clk_i)
    begin
      if rising_edge(clk_i) then
        rs1_data_o <= regfile(to_integer(unsigned(rs1_addr_i(AWIDTH-1 downto 0))));
        rs2_data_o <= regfile(to_integer(unsigned(rs2_addr_i(AWIDTH-1 downto 0))));
      end if;
    end process;

    -- unused --
    rf_we <= '0';
    addr  <= (others => '0');
    wdata <= (others => '0');

  end generate;


  -- Architecture Style 3: Latch-Based ------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  arch_latch:
  if (ARCHSEL = 3) generate

    -- write buffer --
    rf_write_buf: process(clk_i)
    begin
      if rising_edge(clk_i) then
        -- data input register --
        if (rd_we_i = '1') then
          wdata <= rd_data_i;
        end if;
        -- write-address buffer --
        if (rd_we_i = '1') then
          addr <= rd_addr_i;
        else
          addr <= (others => '0');
        end if;
      end if;
    end process;

    -- write select --
    onehot_gen:
    for i in 1 to (2**AWIDTH)-1 generate
      onehot(i) <= '1' when (unsigned(addr(AWIDTH-1 downto 0)) = to_unsigned(i, AWIDTH)) else '0';
    end generate;

    -- individual latches (transparent when clock is LOW) --
    regfile_gen:
    for i in 1 to (2**AWIDTH)-1 generate
      rf_write: process(clk_i, onehot, wdata)
      begin
        if (clk_i = '0') and (onehot(i) = '1') then
          regfile(i) <= wdata;
        end if;
      end process;
    end generate;
    regfile(0) <= (others => '0'); -- x0 is hardwired to zero

    -- synchronous read --
    rf_read: process(clk_i)
    begin
      if rising_edge(clk_i) then
        rs1_data_o <= regfile(to_integer(unsigned(rs1_addr_i(AWIDTH-1 downto 0))));
        rs2_data_o <= regfile(to_integer(unsigned(rs2_addr_i(AWIDTH-1 downto 0))));
      end if;
    end process;

    -- unused --
    rf_we <= '0';

  end generate;

end architecture;
