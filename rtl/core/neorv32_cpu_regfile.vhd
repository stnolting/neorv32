-- #################################################################################################
-- # << NEORV32 CPU - General Purpose Data Register File >>                                        #
-- # ********************************************************************************************* #
-- # Data register file. 32 entries (= 1024 bit) for RV32I ISA (default), 16 entries (= 512 bit)   #
-- # for RV32E ISA (when RISC-V "E" extension is enabled via "RVE_EN").                            #
-- #                                                                                               #
-- # By default the register file is coded to infer block RAM (for FPGAs), that do no provide a    #
-- # dedicated hardware reset. For ASIC implementation or setup requiring a dedicated hardware     #
-- # reset a single-register-based architecture can be enabled via "RST_EN".                       #
-- #                                                                                               #
-- # A third and a fourth read port can be optionally enabled ("RS3_EN", "RS4_EN").                #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # The NEORV32 RISC-V Processor, https://github.com/stnolting/neorv32                            #
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

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cpu_regfile is
  generic (
    RST_EN : boolean; -- enable dedicated hardware reset ("ASIC style")
    RVE_EN : boolean; -- implement embedded RF extension
    RS3_EN : boolean; -- enable 3rd read port
    RS4_EN : boolean  -- enable 4th read port
  );
  port (
    -- global control --
    clk_i  : in  std_ulogic; -- global clock, rising edge
    rstn_i : in  std_ulogic; -- global reset, low-active, async
    ctrl_i : in  ctrl_bus_t; -- main control bus
    -- data input --
    alu_i  : in  std_ulogic_vector(XLEN-1 downto 0); -- ALU result
    mem_i  : in  std_ulogic_vector(XLEN-1 downto 0); -- memory read data
    csr_i  : in  std_ulogic_vector(XLEN-1 downto 0); -- CSR read data
    ret_i  : in  std_ulogic_vector(XLEN-1 downto 0); -- link PC
    -- data output --
    rs1_o  : out std_ulogic_vector(XLEN-1 downto 0); -- rs1
    rs2_o  : out std_ulogic_vector(XLEN-1 downto 0); -- rs2
    rs3_o  : out std_ulogic_vector(XLEN-1 downto 0); -- rs3
    rs4_o  : out std_ulogic_vector(XLEN-1 downto 0)  -- rs4
  );
end neorv32_cpu_regfile;

architecture neorv32_cpu_regfile_rtl of neorv32_cpu_regfile is

  -- auto-configuration --
  constant addr_bits_c : natural := cond_sel_natural_f(RVE_EN, 4, 5); -- address width

  -- register file --
  type   reg_file_t is array ((2**addr_bits_c)-1 downto 0) of std_ulogic_vector(XLEN-1 downto 0);
  signal reg_file : reg_file_t;

  -- access --
  signal rf_wdata  : std_ulogic_vector(XLEN-1 downto 0); -- write-back data
  signal rf_we     : std_ulogic; -- write enable
  signal rf_we_sel : std_ulogic_vector((2**addr_bits_c)-1 downto 0); -- one-hot write enable
  signal rd_zero   : std_ulogic; -- writing to x0?
  signal opa_addr  : std_ulogic_vector(4 downto 0); -- rs1/rd address
  signal rs4_addr  : std_ulogic_vector(4 downto 0); -- rs4 address

begin

  -- Data Write-Back Select -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  wb_select: process(ctrl_i, alu_i, mem_i, csr_i, ret_i)
  begin
    case ctrl_i.rf_mux is
      when rf_mux_alu_c => rf_wdata <= alu_i; -- ALU result
      when rf_mux_mem_c => rf_wdata <= mem_i; -- memory read data
      when rf_mux_csr_c => rf_wdata <= csr_i; -- CSR read data
      when rf_mux_ret_c => rf_wdata <= ret_i; -- link PC (return address)
      when others       => rf_wdata <= alu_i; -- don't care
    end case;
  end process wb_select;


  -- FPGA Register File (no hardware reset) -------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  register_file_fpga:
  if not RST_EN generate

    -- Register zero (x0) is a "normal" physical register that is set to zero by the CPU control
    -- hardware. The register file uses synchronous read accesses and a *single* multiplexed
    -- address port for writing and reading rd/rs1 and a single read-only port for rs2. Therefore,
    -- the whole register file can be mapped to a single true-dual-port block RAM.

    rd_zero  <= '1' when (ctrl_i.rf_rd = "00000") else '0';
    rf_we    <= (ctrl_i.rf_wb_en and (not rd_zero)) or ctrl_i.rf_zero_we; -- never write to x0 unless explicitly forced
    opa_addr <= "00000" when (ctrl_i.rf_zero_we = '1') else -- force rd = zero
                ctrl_i.rf_rd when (ctrl_i.rf_wb_en = '1') else -- rd
                ctrl_i.rf_rs1; -- rs1

    register_file: process(clk_i)
    begin
      if rising_edge(clk_i) then
        if (rf_we = '1') then
          reg_file(to_integer(unsigned(opa_addr(addr_bits_c-1 downto 0)))) <= rf_wdata;
        end if;
        rs1_o <= reg_file(to_integer(unsigned(opa_addr(addr_bits_c-1 downto 0))));
        rs2_o <= reg_file(to_integer(unsigned(ctrl_i.rf_rs2(addr_bits_c-1 downto 0))));
      end if;
    end process register_file;

  end generate;


  -- ASIC Register File (full hardware reset) -----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  register_file_asic:
  if RST_EN generate

    -- write enable decoder --
    we_decode: process(ctrl_i)
    begin
      rf_we_sel <= (others => '0');
      if (ctrl_i.rf_wb_en = '1') then
        rf_we_sel(to_integer(unsigned(ctrl_i.rf_rd(addr_bits_c-1 downto 0)))) <= '1';
      end if;
    end process we_decode;

    -- individual registers --
    reg_gen:
    for i in 1 to (2**addr_bits_c)-1 generate
      register_file: process(rstn_i, clk_i)
      begin
        if (rstn_i = '0') then
          reg_file(i) <= (others => '0');
        elsif rising_edge(clk_i) then
          if (rf_we_sel(i) = '1') then
            reg_file(i) <= rf_wdata;
          end if;
        end if;
      end process register_file;
    end generate;

    reg_file(0) <= (others => '0'); -- x0 is hardwired to zero

    rf_read: process(clk_i)
    begin
      if rising_edge(clk_i) then
        rs1_o <= reg_file(to_integer(unsigned(ctrl_i.rf_rs1(addr_bits_c-1 downto 0))));
        rs2_o <= reg_file(to_integer(unsigned(ctrl_i.rf_rs2(addr_bits_c-1 downto 0))));
      end if;
    end process rf_read;

  end generate;


  -- Optional 3rd Read Port (rs3) -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rs3_enable:
  if RS3_EN generate
    rs3_read: process(clk_i)
    begin
      if rising_edge(clk_i) then
        rs3_o <= reg_file(to_integer(unsigned(ctrl_i.rf_rs3(addr_bits_c-1 downto 0))));
      end if;
    end process rs3_read;
  end generate;

  rs3_disable:
  if not RS3_EN generate
    rs3_o <= (others => '0');
  end generate;


  -- Optional 4th Read Port (rs4) -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  rs4_enable:
  if RS4_EN generate
    rs4_read: process(clk_i)
    begin
      if rising_edge(clk_i) then
        rs4_o <= reg_file(to_integer(unsigned(rs4_addr(addr_bits_c-1 downto 0))));
      end if;
    end process rs4_read;
    rs4_addr <= ctrl_i.ir_funct12(6 downto 5) & ctrl_i.ir_funct3; -- rs4 = [26:25] & [14:12]; not RISC-V-standard!
  end generate;

  rs4_disable:
  if not RS4_EN generate
    rs4_o <= (others => '0');
  end generate;


end neorv32_cpu_regfile_rtl;
