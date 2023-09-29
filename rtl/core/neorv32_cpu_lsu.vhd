-- #################################################################################################
-- # << NEORV32 CPU - Load/Store Unit >>                                                           #
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
-- # The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32       (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cpu_lsu is
  generic (
    AMO_LRSC_ENABLE : boolean -- enable atomic LR/SC operations
  );
  port (
    -- global control --
    clk_i         : in  std_ulogic; -- global clock, rising edge
    rstn_i        : in  std_ulogic := '0'; -- global reset, low-active, async
    ctrl_i        : in  ctrl_bus_t; -- main control bus
    -- cpu data access interface --
    addr_i        : in  std_ulogic_vector(XLEN-1 downto 0); -- ALU result -> access address
    wdata_i       : in  std_ulogic_vector(XLEN-1 downto 0); -- write data
    rdata_o       : out std_ulogic_vector(XLEN-1 downto 0); -- read data
    mar_o         : out std_ulogic_vector(XLEN-1 downto 0); -- current memory address register
    wait_o        : out std_ulogic; -- wait for access to complete
    ma_load_o     : out std_ulogic; -- misaligned load data address
    ma_store_o    : out std_ulogic; -- misaligned store data address
    be_load_o     : out std_ulogic; -- bus error on load data access
    be_store_o    : out std_ulogic; -- bus error on store data access
    pmp_r_fault_i : in  std_ulogic; -- PMP read fault
    pmp_w_fault_i : in  std_ulogic; -- PMP write fault
    -- data bus --
    bus_req_o     : out bus_req_t;  -- request
    bus_rsp_i     : in  bus_rsp_t   -- response
  );
end neorv32_cpu_lsu;

architecture neorv32_cpu_lsu_rtl of neorv32_cpu_lsu is

  -- bus arbiter --
  type bus_arbiter_t is record
    pend_rd : std_ulogic; -- pending bus read access
    pend_wr : std_ulogic; -- pending bus write access
    bus_err : std_ulogic; -- bus access error
  end record;
  signal arbiter : bus_arbiter_t;

  -- misc --
  signal mar        : std_ulogic_vector(XLEN-1 downto 0); -- data memory address register
  signal misaligned : std_ulogic; -- misaligned address

begin

  -- Access Address -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_adr_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      mar        <= (others => '0');
      misaligned <= '0';
    elsif rising_edge(clk_i) then
      if (ctrl_i.lsu_mo_we = '1') then
        mar <= addr_i; -- memory address register
        case ctrl_i.ir_funct3(1 downto 0) is -- alignment check
          when "00"   => misaligned <= '0'; -- byte
          when "01"   => misaligned <= addr_i(0); -- half-word
          when "10"   => misaligned <= addr_i(1) or addr_i(0); -- word
          when others => misaligned <= '0'; -- undefined
        end case;
      end if;
    end if;
  end process mem_adr_reg;

  -- address output --
  bus_req_o.addr <= mar;
  mar_o          <= mar; -- for MTVAL CSR


  -- Access Type ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_type_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bus_req_o.priv <= '0';
      bus_req_o.rvso <= '0';
    elsif rising_edge(clk_i) then
      if (ctrl_i.lsu_mo_we = '1') then
        -- privilege level --
        bus_req_o.priv <= ctrl_i.lsu_priv;
        -- reservation set operation --
        if (AMO_LRSC_ENABLE = true) and (ctrl_i.ir_opcode(2) = opcode_amo_c(2)) then
          bus_req_o.rvso <= '1';
        else
          bus_req_o.rvso <= '0';
        end if;
      end if;
    end if;
  end process mem_type_reg;

  -- source identifier --
  bus_req_o.src <= '0'; -- 0 = data access


  -- Write Data: Alignment and Byte Enable --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_do_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bus_req_o.data <= (others => '0');
      bus_req_o.ben  <= (others => '0');
    elsif rising_edge(clk_i) then
      if (ctrl_i.lsu_mo_we = '1') then
        case ctrl_i.ir_funct3(1 downto 0) is
          when "00" => -- byte
            bus_req_o.data(07 downto 00) <= wdata_i(7 downto 0);
            bus_req_o.data(15 downto 08) <= wdata_i(7 downto 0);
            bus_req_o.data(23 downto 16) <= wdata_i(7 downto 0);
            bus_req_o.data(31 downto 24) <= wdata_i(7 downto 0);
            bus_req_o.ben <= (others => '0');
            bus_req_o.ben(to_integer(unsigned(addr_i(1 downto 0)))) <= '1';
          when "01" => -- half-word
            bus_req_o.data(15 downto 00) <= wdata_i(15 downto 0);
            bus_req_o.data(31 downto 16) <= wdata_i(15 downto 0);
            if (addr_i(1) = '0') then
              bus_req_o.ben <= "0011"; -- low half-word
            else
              bus_req_o.ben <= "1100"; -- high half-word
            end if;
          when others => -- word
            bus_req_o.data <= wdata_i;
            bus_req_o.ben  <= "1111";
        end case;
      end if;
    end if;
  end process mem_do_reg;


  -- Read Data: Alignment and Sign-Extension ------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_di_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      rdata_o <= (others => '0');
    elsif rising_edge(clk_i) then
      if (arbiter.pend_rd = '1') or ((AMO_LRSC_ENABLE = true) and (arbiter.pend_wr = '1')) then -- also update on write access for atomic sc.w
        case ctrl_i.ir_funct3(1 downto 0) is
          when "00" => -- byte
            case mar(1 downto 0) is
              when "00" => -- byte 0
                rdata_o(7 downto 0) <= bus_rsp_i.data(07 downto 00);
                rdata_o(XLEN-1 downto 8) <= (others => ((not ctrl_i.ir_funct3(2)) and bus_rsp_i.data(07))); -- sign-ext
              when "01" => -- byte 1
                rdata_o(7 downto 0) <= bus_rsp_i.data(15 downto 08);
                rdata_o(XLEN-1 downto 8) <= (others => ((not ctrl_i.ir_funct3(2)) and bus_rsp_i.data(15))); -- sign-ext
              when "10" => -- byte 2
                rdata_o(7 downto 0) <= bus_rsp_i.data(23 downto 16);
                rdata_o(XLEN-1 downto 8) <= (others => ((not ctrl_i.ir_funct3(2)) and bus_rsp_i.data(23))); -- sign-ext
              when others => -- byte 3
                rdata_o(7 downto 0) <= bus_rsp_i.data(31 downto 24);
                rdata_o(XLEN-1 downto 8) <= (others => ((not ctrl_i.ir_funct3(2)) and bus_rsp_i.data(31))); -- sign-ext
            end case;
          when "01" => -- half-word
            if (mar(1) = '0') then
              rdata_o(15 downto 0) <= bus_rsp_i.data(15 downto 00); -- low half-word
              rdata_o(XLEN-1 downto 16) <= (others => ((not ctrl_i.ir_funct3(2)) and bus_rsp_i.data(15))); -- sign-ext
            else
              rdata_o(15 downto 0) <= bus_rsp_i.data(31 downto 16); -- high half-word
              rdata_o(XLEN-1 downto 16) <= (others => ((not ctrl_i.ir_funct3(2)) and bus_rsp_i.data(31))); -- sign-ext
            end if;
          when others => -- word
            rdata_o(XLEN-1 downto 0) <= bus_rsp_i.data(XLEN-1 downto 0); -- full word
        end case;
      end if;
    end if;
  end process mem_di_reg;


  -- Access Arbiter -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  access_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      arbiter.bus_err <= '0';
      arbiter.pend_rd <= '0';
      arbiter.pend_wr <= '0';
    elsif rising_edge(clk_i) then
      arbiter.bus_err <= bus_rsp_i.err and (arbiter.pend_rd or arbiter.pend_wr); -- bus error during access
      if (arbiter.pend_rd = '0') and (arbiter.pend_wr = '0') then -- idle
        arbiter.pend_rd <= ctrl_i.lsu_req_rd;
        arbiter.pend_wr <= ctrl_i.lsu_req_wr;
      elsif (bus_rsp_i.ack = '1') or (ctrl_i.cpu_trap = '1') then -- normal termination or start of trap handling
        arbiter.pend_rd <= '0';
        arbiter.pend_wr <= '0';
      end if;
    end if;
  end process access_arbiter;

  -- wait for bus response --
  wait_o <= not bus_rsp_i.ack;

  -- output data access/alignment errors to control unit --
  ma_load_o  <= arbiter.pend_rd and misaligned;
  be_load_o  <= arbiter.pend_rd and (arbiter.bus_err or pmp_r_fault_i);
  ma_store_o <= arbiter.pend_wr and misaligned;
  be_store_o <= arbiter.pend_wr and (arbiter.bus_err or pmp_w_fault_i);

  -- access requests (all source signals are driven by registers!) --
  bus_req_o.re <= ctrl_i.lsu_req_rd and (not misaligned) and (not pmp_r_fault_i);
  bus_req_o.we <= ctrl_i.lsu_req_wr and (not misaligned) and (not pmp_w_fault_i);


end neorv32_cpu_lsu_rtl;
