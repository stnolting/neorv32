-- #################################################################################################
-- # << NEORV32 - (Data) Bus Interface Unit >>                                                     #
-- # ********************************************************************************************* #
-- # Data bus interface (load/store unit) and physical memory protection (PMP).                    #
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

entity neorv32_cpu_bus is
  generic (
    PMP_NUM_REGIONS     : natural; -- number of regions (0..16)
    PMP_MIN_GRANULARITY : natural  -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
  );
  port (
    -- global control --
    clk_i         : in  std_ulogic; -- global clock, rising edge
    rstn_i        : in  std_ulogic := '0'; -- global reset, low-active, async
    ctrl_i        : in  ctrl_bus_t; -- main control bus
    -- cpu instruction fetch interface --
    fetch_pc_i    : in  std_ulogic_vector(XLEN-1 downto 0); -- PC for instruction fetch
    i_pmp_fault_o : out std_ulogic; -- instruction fetch pmp fault
    -- cpu data access interface --
    addr_i        : in  std_ulogic_vector(XLEN-1 downto 0); -- ALU result -> access address
    wdata_i       : in  std_ulogic_vector(XLEN-1 downto 0); -- write data
    rdata_o       : out std_ulogic_vector(XLEN-1 downto 0); -- read data
    mar_o         : out std_ulogic_vector(XLEN-1 downto 0); -- current memory address register
    d_wait_o      : out std_ulogic; -- wait for access to complete
    ma_load_o     : out std_ulogic; -- misaligned load data address
    ma_store_o    : out std_ulogic; -- misaligned store data address
    be_load_o     : out std_ulogic; -- bus error on load data access
    be_store_o    : out std_ulogic; -- bus error on store data access
    -- physical memory protection --
    pmp_addr_i    : in  pmp_addr_if_t; -- addresses
    pmp_ctrl_i    : in  pmp_ctrl_if_t; -- configs
    -- data bus --
    d_bus_addr_o  : out std_ulogic_vector(XLEN-1 downto 0); -- bus access address
    d_bus_rdata_i : in  std_ulogic_vector(XLEN-1 downto 0); -- bus read data
    d_bus_wdata_o : out std_ulogic_vector(XLEN-1 downto 0); -- bus write data
    d_bus_ben_o   : out std_ulogic_vector((XLEN/8)-1 downto 0); -- byte enable
    d_bus_we_o    : out std_ulogic; -- write enable
    d_bus_re_o    : out std_ulogic; -- read enable
    d_bus_ack_i   : in  std_ulogic; -- bus transfer acknowledge
    d_bus_err_i   : in  std_ulogic; -- bus transfer error
    d_bus_fence_o : out std_ulogic; -- fence operation
    d_bus_priv_o  : out std_ulogic  -- current effective privilege level
  );
end neorv32_cpu_bus;

architecture neorv32_cpu_bus_rtl of neorv32_cpu_bus is

  -- PMP configuration register bits --
  constant pmp_cfg_r_c  : natural := 0; -- read permit
  constant pmp_cfg_w_c  : natural := 1; -- write permit
  constant pmp_cfg_x_c  : natural := 2; -- execute permit
  constant pmp_cfg_al_c : natural := 3; -- mode bit low
  constant pmp_cfg_ah_c : natural := 4; -- mode bit high
  constant pmp_cfg_l_c  : natural := 7; -- locked entry

  -- PMP helpers --
  constant pmp_lsb_c  : natural := index_size_f(PMP_MIN_GRANULARITY); -- min = 2
  constant pmp_zero_c : std_ulogic_vector(XLEN-1 downto pmp_lsb_c) := (others => '0');

  -- misc --
  signal data_sign  : std_ulogic; -- signed load
  signal mar        : std_ulogic_vector(XLEN-1 downto 0); -- data memory address register
  signal misaligned : std_ulogic; -- misaligned address

  -- bus arbiter --
  type bus_arbiter_t is record
    pend      : std_ulogic; -- pending bus access
    err       : std_ulogic; -- bus access error
    pmp_r_err : std_ulogic; -- pmp load fault
    pmp_w_err : std_ulogic; -- pmp store fault
  end record;
  signal arbiter : bus_arbiter_t;

  -- physical memory protection --
  type pmp_mask_t is array (0 to PMP_NUM_REGIONS-1) of std_ulogic_vector(XLEN-1 downto pmp_lsb_c);
  type pmp_t is record
    i_cmp_mm : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    i_cmp_ge : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    i_cmp_lt : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    d_cmp_mm : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    d_cmp_ge : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    d_cmp_lt : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    i_match  : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    d_match  : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    perm_ex  : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    perm_rd  : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    perm_wr  : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    fail_ex  : std_ulogic_vector(PMP_NUM_REGIONS   downto 0);
    fail_rd  : std_ulogic_vector(PMP_NUM_REGIONS   downto 0);
    fail_wr  : std_ulogic_vector(PMP_NUM_REGIONS   downto 0);
  end record;
  signal pmp_mask : pmp_mask_t;
  signal pmp      : pmp_t;

  -- pmp faults --
  signal if_pmp_fault : std_ulogic; -- pmp instruction access fault
  signal ld_pmp_fault : std_ulogic; -- pmp load access fault
  signal st_pmp_fault : std_ulogic; -- pmp store access fault

begin

  -- Access Address -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_adr_reg: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (ctrl_i.bus_mo_we = '1') then
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
  d_bus_addr_o <= mar;
  mar_o        <= mar; -- for MTVAL CSR


  -- Write Data: Byte Enable and Alignment --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_do_reg: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (ctrl_i.bus_mo_we = '1') then
        d_bus_ben_o <= (others => '0'); -- default
        case ctrl_i.ir_funct3(1 downto 0) is
          when "00" => -- byte
            d_bus_wdata_o(07 downto 00) <= wdata_i(7 downto 0);
            d_bus_wdata_o(15 downto 08) <= wdata_i(7 downto 0);
            d_bus_wdata_o(23 downto 16) <= wdata_i(7 downto 0);
            d_bus_wdata_o(31 downto 24) <= wdata_i(7 downto 0);
            d_bus_ben_o(to_integer(unsigned(addr_i(1 downto 0)))) <= '1';
          when "01" => -- half-word
            d_bus_wdata_o(15 downto 00) <= wdata_i(15 downto 0);
            d_bus_wdata_o(31 downto 16) <= wdata_i(15 downto 0);
            if (addr_i(1) = '0') then
              d_bus_ben_o <= "0011"; -- low half-word
            else
              d_bus_ben_o <= "1100"; -- high half-word
            end if;
          when others => -- word
            d_bus_wdata_o <= wdata_i;
            d_bus_ben_o   <= "1111";
        end case;
      end if;
    end if;
  end process mem_do_reg;


  -- Read Data: Alignment and Sign-Extension ------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_di_reg: process(clk_i)
  begin
    if rising_edge(clk_i) then
      case ctrl_i.ir_funct3(1 downto 0) is
        when "00" => -- byte
          case mar(1 downto 0) is
            when "00" => -- byte 0
              rdata_o(7 downto 0) <= d_bus_rdata_i(07 downto 00);
              rdata_o(XLEN-1 downto 8) <= (others => (data_sign and d_bus_rdata_i(07))); -- sign extension
            when "01" => -- byte 1
              rdata_o(7 downto 0) <= d_bus_rdata_i(15 downto 08);
              rdata_o(XLEN-1 downto 8) <= (others => (data_sign and d_bus_rdata_i(15))); -- sign extension
            when "10" => -- byte 2
              rdata_o(7 downto 0) <= d_bus_rdata_i(23 downto 16);
              rdata_o(XLEN-1 downto 8) <= (others => (data_sign and d_bus_rdata_i(23))); -- sign extension
            when others => -- byte 3
              rdata_o(7 downto 0) <= d_bus_rdata_i(31 downto 24);
              rdata_o(XLEN-1 downto 8) <= (others => (data_sign and d_bus_rdata_i(31))); -- sign extension
          end case;
        when "01" => -- half-word
          if (mar(1) = '0') then
            rdata_o(15 downto 0) <= d_bus_rdata_i(15 downto 00); -- low half-word
            rdata_o(XLEN-1 downto 16) <= (others => (data_sign and d_bus_rdata_i(15))); -- sign extension
          else
            rdata_o(15 downto 0) <= d_bus_rdata_i(31 downto 16); -- high half-word
            rdata_o(XLEN-1 downto 16) <= (others => (data_sign and d_bus_rdata_i(31))); -- sign extension
          end if;
        when others => -- word
          rdata_o(XLEN-1 downto 0) <= d_bus_rdata_i(XLEN-1 downto 0); -- full word
      end case;
    end if;
  end process mem_di_reg;

  -- sign extension --
  data_sign <= not ctrl_i.ir_funct3(2); -- NOT unsigned LOAD (LBU, LHU)


  -- Access Arbiter -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  data_access_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      arbiter.pend      <= '0';
      arbiter.err       <= '0';
      arbiter.pmp_r_err <= '0';
      arbiter.pmp_w_err <= '0';
    elsif rising_edge(clk_i) then
      -- arbiter --
      if (arbiter.pend = '0') then -- idle
        if (ctrl_i.bus_req = '1') then -- start bus access
          arbiter.pend <= '1';
        end if;
        arbiter.err <= '0';
      else -- bus access in progress
        -- accumulate bus errors --
        if (d_bus_err_i = '1') or -- bus error
           ((ctrl_i.ir_opcode(5) = '1') and (arbiter.pmp_w_err = '1')) or -- PMP store fault
           ((ctrl_i.ir_opcode(5) = '0') and (arbiter.pmp_r_err = '1')) then -- PMP load fault
          arbiter.err <= '1';
        end if;
        -- wait for normal termination or start of trap handling --
        if (d_bus_ack_i = '1') or (ctrl_i.cpu_trap = '1') then
          arbiter.pend <= '0';
        end if;
      end if;
      -- PMP error --
      if (ctrl_i.bus_mo_we = '1') then -- sample PMP errors only once
        arbiter.pmp_r_err <= ld_pmp_fault;
        arbiter.pmp_w_err <= st_pmp_fault;
      end if;
    end if;
  end process data_access_arbiter;

  -- wait for bus response --
  d_wait_o <= not d_bus_ack_i;

  -- output data access error to control unit --
  ma_load_o  <= '1' when (arbiter.pend = '1') and (ctrl_i.ir_opcode(5) = '0') and (misaligned  = '1') else '0';
  be_load_o  <= '1' when (arbiter.pend = '1') and (ctrl_i.ir_opcode(5) = '0') and (arbiter.err = '1') else '0';
  ma_store_o <= '1' when (arbiter.pend = '1') and (ctrl_i.ir_opcode(5) = '1') and (misaligned  = '1') else '0';
  be_store_o <= '1' when (arbiter.pend = '1') and (ctrl_i.ir_opcode(5) = '1') and (arbiter.err = '1') else '0';

  -- data bus control interface (all source signals are driven by registers) --
  d_bus_we_o    <= ctrl_i.bus_req and (    ctrl_i.ir_opcode(5)) and (not misaligned) and (not arbiter.pmp_w_err);
  d_bus_re_o    <= ctrl_i.bus_req and (not ctrl_i.ir_opcode(5)) and (not misaligned) and (not arbiter.pmp_r_err);
  d_bus_fence_o <= ctrl_i.bus_fence;
  d_bus_priv_o  <= ctrl_i.bus_priv;


  -- RISC-V Physical Memory Protection (PMP) ------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- compute address masks for NAPOT modes (iterative!) --
  pmp_masking_gen:
  for r in 0 to PMP_NUM_REGIONS-1 generate
    pmp_masking: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        pmp_mask(r) <= (others => '0');
      elsif rising_edge(clk_i) then -- address mask computation has a latency of max 32 cycles
        if (pmp_ctrl_i(r)(pmp_cfg_al_c) = '1') then -- NAPOT (or TOR, but that's irrelevant here)
          pmp_mask(r)(pmp_lsb_c) <= '0';
          for i in pmp_lsb_c+1 to XLEN-1 loop
            pmp_mask(r)(i) <= pmp_mask(r)(i-1) or (not pmp_addr_i(r)(i-1)); -- skip address byte offset
          end loop; -- i
        else -- NA4
          pmp_mask(r) <= (others => '1');
        end if;
      end if;
    end process pmp_masking;
  end generate;


  -- check address --
  pmp_check_address:
  for r in 0 to PMP_NUM_REGIONS-1 generate
    -- NA4 and NAPOT --
    pmp.i_cmp_mm(r) <= '1' when ((fetch_pc_i(XLEN-1 downto pmp_lsb_c) and pmp_mask(r)) = (pmp_addr_i(r)(XLEN-1 downto pmp_lsb_c) and pmp_mask(r))) else '0';
    pmp.d_cmp_mm(r) <= '1' when ((    addr_i(XLEN-1 downto pmp_lsb_c) and pmp_mask(r)) = (pmp_addr_i(r)(XLEN-1 downto pmp_lsb_c) and pmp_mask(r))) else '0';
    -- TOR region 0 --
    pmp_check_address_r0:
    if (r = 0) generate -- first entry: use ZERO as base and current entry as bound
      pmp.i_cmp_ge(r) <= '1'; -- address is always greater than or equal to zero
      pmp.i_cmp_lt(r) <= '0'; -- unused
      pmp.d_cmp_ge(r) <= '1'; -- address is always greater than or equal to zero
      pmp.d_cmp_lt(r) <= '0'; -- unused
    end generate;
    -- TOR region any --
    pmp_check_address_rany:
    if (r > 0) generate -- use previous entry as base and current entry as bound
      pmp.i_cmp_ge(r) <= '1' when (unsigned(fetch_pc_i(XLEN-1 downto pmp_lsb_c)) >= unsigned(pmp_addr_i(r-1)(XLEN-1 downto pmp_lsb_c))) else '0';
      pmp.i_cmp_lt(r) <= '1' when (unsigned(fetch_pc_i(XLEN-1 downto pmp_lsb_c)) <  unsigned(pmp_addr_i(r  )(XLEN-1 downto pmp_lsb_c))) else '0';
      pmp.d_cmp_ge(r) <= '1' when (unsigned(    addr_i(XLEN-1 downto pmp_lsb_c)) >= unsigned(pmp_addr_i(r-1)(XLEN-1 downto pmp_lsb_c))) else '0';
      pmp.d_cmp_lt(r) <= '1' when (unsigned(    addr_i(XLEN-1 downto pmp_lsb_c)) <  unsigned(pmp_addr_i(r  )(XLEN-1 downto pmp_lsb_c))) else '0';
    end generate;
  end generate;


  -- check region matching according to configured mode --
  pmp_check_match_gen:
  for r in 0 to PMP_NUM_REGIONS-1 generate
    pmp_check_match: process(pmp_ctrl_i, pmp)
    begin
      case pmp_ctrl_i(r)(pmp_cfg_ah_c downto pmp_cfg_al_c) is
        when pmp_mode_off_c => -- entry disabled
          pmp.i_match(r) <= '0';
          pmp.d_match(r) <= '0';
        when pmp_mode_tor_c => -- top of region
          if (r = (PMP_NUM_REGIONS-1)) then -- very last entry
            pmp.i_match(r) <= pmp.i_cmp_ge(r) and pmp.i_cmp_lt(r);
            pmp.d_match(r) <= pmp.d_cmp_ge(r) and pmp.d_cmp_lt(r);
          else -- this saves a LOT of comparators
            pmp.i_match(r) <= pmp.i_cmp_ge(r) and (not pmp.i_cmp_ge(r+1));
            pmp.d_match(r) <= pmp.d_cmp_ge(r) and (not pmp.d_cmp_ge(r+1));
          end if;
        when others => -- naturally-aligned region
          pmp.i_match(r) <= pmp.i_cmp_mm(r);
          pmp.d_match(r) <= pmp.d_cmp_mm(r);
        end case;
    end process pmp_check_match;
  end generate;


  -- generate permission bits --
  -- M mode: always allow if lock bit not set, otherwise check permission
  pmp_permission_gen:
  for r in 0 to PMP_NUM_REGIONS-1 generate
    pmp.perm_ex(r) <= pmp_ctrl_i(r)(pmp_cfg_x_c) or (not pmp_ctrl_i(r)(pmp_cfg_l_c)) when (ctrl_i.cpu_priv = priv_mode_m_c) else pmp_ctrl_i(r)(pmp_cfg_x_c);
    pmp.perm_rd(r) <= pmp_ctrl_i(r)(pmp_cfg_r_c) or (not pmp_ctrl_i(r)(pmp_cfg_l_c)) when (ctrl_i.bus_priv = priv_mode_m_c) else pmp_ctrl_i(r)(pmp_cfg_r_c);
    pmp.perm_wr(r) <= pmp_ctrl_i(r)(pmp_cfg_w_c) or (not pmp_ctrl_i(r)(pmp_cfg_l_c)) when (ctrl_i.bus_priv = priv_mode_m_c) else pmp_ctrl_i(r)(pmp_cfg_w_c);
  end generate;


  -- check for access fault (using static prioritization) --
  -- default: fault if not M-mode --
  pmp.fail_ex(PMP_NUM_REGIONS) <= '1' when (ctrl_i.cpu_priv /= priv_mode_m_c) else '0';
  pmp.fail_rd(PMP_NUM_REGIONS) <= '1' when (ctrl_i.bus_priv /= priv_mode_m_c) else '0';
  pmp.fail_wr(PMP_NUM_REGIONS) <= '1' when (ctrl_i.bus_priv /= priv_mode_m_c) else '0';
  -- this is a *structural* description of a prioritization logic implemented as a multiplexer chain --
  pmp_chech_fault:
  for r in PMP_NUM_REGIONS-1 downto 0 generate -- start with lowest priority
    pmp.fail_ex(r) <= not pmp.perm_ex(r) when (pmp.i_match(r) = '1') else pmp.fail_ex(r+1);
    pmp.fail_rd(r) <= not pmp.perm_rd(r) when (pmp.d_match(r) = '1') else pmp.fail_rd(r+1);
    pmp.fail_wr(r) <= not pmp.perm_wr(r) when (pmp.d_match(r) = '1') else pmp.fail_wr(r+1);
  end generate;


  -- final PMP access fault signals (ignore PMP rules when in debug mode) --
  if_pmp_fault <= '1' when (pmp.fail_ex(0) = '1') and (PMP_NUM_REGIONS > 0) and (ctrl_i.cpu_debug = '0') else '0';
  ld_pmp_fault <= '1' when (pmp.fail_rd(0) = '1') and (PMP_NUM_REGIONS > 0) and (ctrl_i.cpu_debug = '0') else '0';
  st_pmp_fault <= '1' when (pmp.fail_wr(0) = '1') and (PMP_NUM_REGIONS > 0) and (ctrl_i.cpu_debug = '0') else '0';

  -- instruction fetch PMP fault --
  i_pmp_fault_o <= if_pmp_fault;


end neorv32_cpu_bus_rtl;
