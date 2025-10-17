-- ================================================================================ --
-- NEORV32 CPU - Data Register File                                                 --
-- -------------------------------------------------------------------------------- --
-- Data register file. 32 entries (= 1024 bit) for RV32I ISA (default), 16 entries  --
-- (= 512 bit) for RV32E ISA (when RISC-V "E" extension is enabled via "RVE_EN").   --
--                                                                                  --
-- By default the register file is coded to infer block RAM (for FPGAs), that does  --
-- not provide a dedicated hardware reset. For ASIC implementation or setups that   --
-- do require a dedicated hardware reset a flip-flop-based architecture can be      --
-- enabled via "RST_EN".                                                            --
--                                                                                  --
-- [NOTE] Read-during-write behavior of the register file's memory core is          --
--        irrelevant as read and write accesses are mutually exclusive and will     --
--        never happen at the same time.                                            --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
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
    RST_EN : boolean; -- implement dedicated hardware reset ("ASIC style")
    RVE_EN : boolean  -- implement embedded RF extension
  );
  port (
    -- global control --
    clk_i  : in  std_ulogic; -- global clock, rising edge
    rstn_i : in  std_ulogic; -- global reset, low-active, async
    ctrl_i : in  ctrl_bus_t; -- main control bus
    -- operands --
    rd_i   : in  std_ulogic_vector(XLEN-1 downto 0); -- destination data rd
    rs1_o  : out std_ulogic_vector(XLEN-1 downto 0); -- source data rs1
    rs2_o  : out std_ulogic_vector(XLEN-1 downto 0)  -- source data rs2
  );
end neorv32_cpu_regfile;

architecture neorv32_cpu_regfile_rtl of neorv32_cpu_regfile is

  -- auto-configuration --
  constant awidth_c : natural := cond_sel_natural_f(RVE_EN, 4, 5); -- address width

  -- register file --
  type   reg_file_t is array (0 to (2**awidth_c)-1) of std_ulogic_vector(XLEN-1 downto 0);
  signal reg_file : reg_file_t;

  -- access logic --
  signal rf_we, rd_zero : std_ulogic;
  signal opa_addr : std_ulogic_vector(4 downto 0);

begin

  -- FPGA-Style Register File Access Logic --------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- Register zero (x0) is a "normal" physical register that is set to zero by the CPU control
  -- hardware. The register file uses synchronous read accesses and a *single* multiplexed
  -- address port for writing and reading rd/rs1 and a single read-only port for reading rs2.
  -- Therefore, the whole register file can be mapped to a single true-dual-port RAM.

  rd_zero  <= '1' when (ctrl_i.rf_rd = "00000") else '0';
  rf_we    <= (ctrl_i.rf_wb_en and (not rd_zero)) or ctrl_i.rf_zero_we; -- never write to x0 unless forced
  opa_addr <= "00000" when (ctrl_i.rf_zero_we = '1') else -- force rd = zero
              ctrl_i.rf_rd when (ctrl_i.rf_wb_en = '1') else -- rd
              ctrl_i.rf_rs1; -- rs1


  -- FPGA-Style Register File (BlockRAM, no hardware reset at all) --------------------------
  -- -------------------------------------------------------------------------------------------
  register_file_fpga:
  if not RST_EN generate
    reg_file_inst: entity neorv32.neorv32_prim_sdpram
    generic map (
      AWIDTH => awidth_c,
      DWIDTH => XLEN,
      OUTREG => false
    )
    port map (
      -- global control --
      clk_i    => clk_i,
      -- write port --
      a_en_i   => '1',
      a_rw_i   => rf_we,
      a_addr_i => opa_addr(awidth_c-1 downto 0),
      a_data_i => rd_i,
      a_data_o => rs1_o,
      -- read port --
      b_en_i   => '1',
      b_addr_i => ctrl_i.rf_rs2(awidth_c-1 downto 0),
      b_data_o => rs2_o
    );
    reg_file <= (others => (others => '0')); -- unused
  end generate;


  -- ASIC-Style Register File (individual FFs, full hardware reset) -------------------------
  -- -------------------------------------------------------------------------------------------
  register_file_asic:
  if RST_EN generate

    -- x0 is hardwired to zero --
    reg_file(0) <= (others => '0');

    -- individual registers --
    reg_gen:
    for i in 1 to (2**awidth_c)-1 generate
      register_file: process(rstn_i, clk_i)
      begin
        if (rstn_i = '0') then
          reg_file(i) <= (others => '0'); -- full hardware reset
        elsif rising_edge(clk_i) then
          if (unsigned(ctrl_i.rf_rd(awidth_c-1 downto 0)) = to_unsigned(i, awidth_c)) and (ctrl_i.rf_wb_en = '1') then
            reg_file(i) <= rd_i;
          end if;
        end if;
      end process register_file;
    end generate;

    -- synchronous read --
    rf_read: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        rs1_o <= (others => '0');
        rs2_o <= (others => '0');
      elsif rising_edge(clk_i) then
        rs1_o <= reg_file(to_integer(unsigned(ctrl_i.rf_rs1(awidth_c-1 downto 0))));
        rs2_o <= reg_file(to_integer(unsigned(ctrl_i.rf_rs2(awidth_c-1 downto 0))));
      end if;
    end process rf_read;

  end generate;

end neorv32_cpu_regfile_rtl;
