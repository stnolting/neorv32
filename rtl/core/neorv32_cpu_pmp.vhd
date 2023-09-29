-- #################################################################################################
-- # << NEORV32 CPU - Physical Memory Protection Unit >>                                           #
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

entity neorv32_cpu_pmp is
  generic (
    NUM_REGIONS : natural range 0 to 16; -- number of regions (0..16)
    GRANULARITY : natural  -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
  );
  port (
    -- global control --
    clk_i       : in  std_ulogic; -- global clock, rising edge
    rstn_i      : in  std_ulogic; -- global reset, low-active, async
    ctrl_i      : in  ctrl_bus_t; -- main control bus
    -- CSR interface --
    csr_we_i    : in  std_ulogic; -- global write enable
    csr_addr_i  : in  std_ulogic_vector(11 downto 0); -- address
    csr_wdata_i : in  std_ulogic_vector(XLEN-1 downto 0); -- write data
    csr_rdata_o : out std_ulogic_vector(XLEN-1 downto 0); -- read data
    -- address input --
    addr_if_i   : in  std_ulogic_vector(XLEN-1 downto 0); -- instruction fetch address
    addr_ls_i   : in  std_ulogic_vector(XLEN-1 downto 0); -- load/store address
    -- faults --
    fault_ex_o  : out std_ulogic; -- instruction fetch fault
    fault_rd_o  : out std_ulogic; -- data load fault
    fault_wr_o  : out std_ulogic  -- data store fault
  );
end neorv32_cpu_pmp;

architecture neorv32_cpu_pmp_rtl of neorv32_cpu_pmp is

  -- PMP configuration register bits --
  constant cfg_r_c  : natural := 0; -- read permit
  constant cfg_w_c  : natural := 1; -- write permit
  constant cfg_x_c  : natural := 2; -- execute permit
  constant cfg_al_c : natural := 3; -- mode bit low
  constant cfg_ah_c : natural := 4; -- mode bit high
  constant cfg_rl_c : natural := 5; -- reserved
  constant cfg_rh_c : natural := 6; -- reserved
  constant cfg_l_c  : natural := 7; -- locked entry

  -- PMP modes --
  constant mode_off_c   : std_ulogic_vector(1 downto 0) := "00"; -- null region (disabled)
  constant mode_tor_c   : std_ulogic_vector(1 downto 0) := "01"; -- top of range
  constant mode_na4_c   : std_ulogic_vector(1 downto 0) := "10"; -- naturally aligned four-byte region
  constant mode_napot_c : std_ulogic_vector(1 downto 0) := "11"; -- naturally aligned power-of-two region (>= 8 bytes)

  -- PMP helpers --
  constant pmp_lsb_c : natural := index_size_f(GRANULARITY); -- min = 2

  -- PMP CSRs --
  type csr_cfg_t      is array (0 to NUM_REGIONS-1) of std_ulogic_vector(7 downto 0);
  type csr_addr_t     is array (0 to NUM_REGIONS-1) of std_ulogic_vector(XLEN-1 downto 0);
  type csr_cfg_rd_t   is array (0 to 15) of std_ulogic_vector(7 downto 0);
  type csr_cfg_rd32_t is array (0 to 03) of std_ulogic_vector(XLEN-1 downto 0);
  type csr_addr_rd_t  is array (0 to 15) of std_ulogic_vector(XLEN-1 downto 0);
  type csr_t is record
    we_cfg  : std_ulogic_vector(03 downto 0);
    we_addr : std_ulogic_vector(15 downto 0);
    cfg     : csr_cfg_t;
    addr    : csr_addr_t;
  end record;
  signal csr      : csr_t;
  signal cfg_rd   : csr_cfg_rd_t;
  signal cfg_rd32 : csr_cfg_rd32_t;
  signal addr_rd  : csr_addr_rd_t;

  -- PMP address extension to 34 bit --
  type xaddr_t is array (0 to NUM_REGIONS-1) of std_ulogic_vector(XLEN+1 downto 0);
  signal xaddr : xaddr_t;

  -- access permission check --
  type addr_mask_t is array (0 to NUM_REGIONS-1) of std_ulogic_vector(XLEN-1 downto pmp_lsb_c);
  signal addr_mask_napot, addr_mask : addr_mask_t;
  type check_t is record
    i_cmp_mm : std_ulogic_vector(NUM_REGIONS-1 downto 0);
    i_cmp_ge : std_ulogic_vector(NUM_REGIONS-1 downto 0);
    i_cmp_lt : std_ulogic_vector(NUM_REGIONS-1 downto 0);
    d_cmp_mm : std_ulogic_vector(NUM_REGIONS-1 downto 0);
    d_cmp_ge : std_ulogic_vector(NUM_REGIONS-1 downto 0);
    d_cmp_lt : std_ulogic_vector(NUM_REGIONS-1 downto 0);
    i_match  : std_ulogic_vector(NUM_REGIONS-1 downto 0);
    d_match  : std_ulogic_vector(NUM_REGIONS-1 downto 0);
    perm_ex  : std_ulogic_vector(NUM_REGIONS-1 downto 0);
    perm_rw  : std_ulogic_vector(NUM_REGIONS-1 downto 0);
    fail_ex  : std_ulogic_vector(NUM_REGIONS   downto 0);
    fail_rw  : std_ulogic_vector(NUM_REGIONS   downto 0);
  end record;
  signal check : check_t;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (is_power_of_two_f(GRANULARITY) = false) report
    "NEORV32 CPU CONFIG ERROR! PMP granularity has to be a power of two." severity error;
  assert not (GRANULARITY < 4) report
    "NEORV32 CPU CONFIG ERROR! PMP granularity has to be at least 4 bytes." severity error;


  -- CSR Write Access -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_we: process(csr_we_i, csr_addr_i) -- write enable decoder
  begin
    -- Configuration registers --
    csr.we_cfg <= (others => '0');
    if (csr_addr_i(11 downto 2) = csr_pmpcfg0_c(11 downto 2)) and (csr_we_i = '1') then
      csr.we_cfg(to_integer(unsigned(csr_addr_i(1 downto 0)))) <= '1';
    end if;
    -- Address registers --
    csr.we_addr <= (others => '0');
    if (csr_addr_i(11 downto 4) = csr_pmpaddr0_c(11 downto 4)) and (csr_we_i = '1') then
      csr.we_addr(to_integer(unsigned(csr_addr_i(3 downto 0)))) <= '1';
    end if;
  end process csr_we;

  -- PMP CSR registers --
  csr_reg_gen:
  for i in 0 to NUM_REGIONS-1 generate
    csr_reg: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        csr.cfg(i)  <= (others => '0');
        csr.addr(i) <= (others => '0');
      elsif rising_edge(clk_i) then

        -- configuration --
        if (csr.we_cfg(i/4) = '1') and (csr.cfg(i)(7) = '0') then -- unlocked write access
          csr.cfg(i)(cfg_r_c) <= csr_wdata_i((i mod 4)*8+0); -- R (read)
          csr.cfg(i)(cfg_w_c) <= csr_wdata_i((i mod 4)*8+1); -- W (write)
          csr.cfg(i)(cfg_x_c) <= csr_wdata_i((i mod 4)*8+2); -- X (execute)
          if (GRANULARITY > 4) and (csr_wdata_i((i mod 4)*8+4 downto (i mod 4)*8+3) = mode_na4_c) then
            csr.cfg(i)(cfg_ah_c downto cfg_al_c) <= mode_off_c; -- NA4 not available, fall back to OFF
          else
            csr.cfg(i)(cfg_ah_c downto cfg_al_c) <= csr_wdata_i((i mod 4)*8+4 downto (i mod 4)*8+3); -- A (mode)
          end if;
          csr.cfg(i)(cfg_rl_c) <= '0'; -- reserved
          csr.cfg(i)(cfg_rh_c) <= '0'; -- reserved
          csr.cfg(i)(cfg_l_c)  <= csr_wdata_i((i mod 4)*8+7); -- L (locked)
        end if;

        -- address --
        if (csr.we_addr(i) = '1') and (csr.cfg(i)(cfg_l_c) = '0') then -- unlocked write access
          if (i < NUM_REGIONS-1) then
            if (csr.cfg(i+1)(cfg_l_c) = '0') or (csr.cfg(i+1)(cfg_ah_c downto cfg_al_c) /= mode_tor_c) then -- cfg(i+1) not "LOCKED TOR"
              csr.addr(i) <= "00" & csr_wdata_i(XLEN-3 downto 0);
            end if;
          else -- very last entry
            csr.addr(i) <= "00" & csr_wdata_i(XLEN-3 downto 0);
          end if;
        end if;

      end if;
    end process csr_reg;
  end generate;


  -- CSR Read Access ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_read_access: process(csr_addr_i, cfg_rd32, addr_rd)
  begin
    if (csr_addr_i(11 downto 5) = csr_pmpcfg0_c(11 downto 5)) then -- PMP CSR
      if (csr_addr_i(4) = '0') then -- PMP configuration CSR
        csr_rdata_o <= cfg_rd32(to_integer(unsigned(csr_addr_i(1 downto 0))));
      else -- PMP address CSR
        csr_rdata_o <= addr_rd(to_integer(unsigned(csr_addr_i(3 downto 0))));
      end if;
    else
      csr_rdata_o <= (others => '0');
    end if;
  end process csr_read_access;

  -- CSR read-back --
  csr_read_back_gen:
  for i in 0 to NUM_REGIONS-1 generate
    -- configuration --
    cfg_rd(i) <= csr.cfg(i);
    -- address --
    address_read_back: process(csr)
    begin
      addr_rd(i) <= (others => '0');
      addr_rd(i)(XLEN-1 downto pmp_lsb_c-2) <= csr.addr(i)(XLEN-1 downto pmp_lsb_c-2);
      if (GRANULARITY = 8) then -- bit G-1 reads as zero in TOR or OFF mode
        if (csr.cfg(i)(cfg_ah_c) = '0') then -- TOR/OFF mode
          addr_rd(i)(pmp_lsb_c) <= '0';
        end if;
      elsif (GRANULARITY > 8) then
        addr_rd(i)(pmp_lsb_c-2 downto 0) <= (others => '1'); -- in NAPOT mode bits G-2:0 must read as one
        if (csr.cfg(i)(cfg_ah_c) = '0') then -- TOR/OFF mode
          addr_rd(i)(pmp_lsb_c-1 downto 0) <= (others => '0'); -- in TOR or OFF mode bits G-1:0 must read as zero
        end if;
      end if;
    end process address_read_back;
  end generate;

  -- terminate unused CSR read-backs --
  csr_read_back_terminate:
  for i in NUM_REGIONS to 15 generate
    cfg_rd(i)  <= (others => '0');
    addr_rd(i) <= (others => '0');
  end generate;

  -- pack configuration read-back --
  cfg_rd32(0) <= cfg_rd(03) & cfg_rd(02) & cfg_rd(01) & cfg_rd(00);
  cfg_rd32(1) <= cfg_rd(07) & cfg_rd(06) & cfg_rd(05) & cfg_rd(04);
  cfg_rd32(2) <= cfg_rd(11) & cfg_rd(10) & cfg_rd(09) & cfg_rd(08);
  cfg_rd32(3) <= cfg_rd(15) & cfg_rd(14) & cfg_rd(13) & cfg_rd(12);


  -- Access Check Logic ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  region_gen:
  for r in 0 to NUM_REGIONS-1 generate

    -- extend region addresses to 34-bit --
    xaddr(r) <= csr.addr(r) & "00"; -- mask byte offset


    -- compute address masks for NAPOT mode --
    addr_mask_napot(r)(pmp_lsb_c) <= '0';
    addr_mask_napot_gen:
    for i in pmp_lsb_c+1 to XLEN-1 generate
      addr_mask_napot(r)(i) <= addr_mask_napot(r)(i-1) or (not xaddr(r)(i-1));
    end generate;

    -- address mask select --
    addr_masking: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        addr_mask(r) <= (others => '0');
      elsif rising_edge(clk_i) then
        if (csr.cfg(r)(cfg_al_c) = '1') then -- NAPOT
          addr_mask(r) <= addr_mask_napot(r);
        else -- NA4
          addr_mask(r) <= (others => '1');
        end if;
      end if;
    end process addr_masking;


    -- check region address match --
    -- NA4 and NAPOT --
    check.i_cmp_mm(r) <= '1' when ((addr_if_i(XLEN-1 downto pmp_lsb_c) and addr_mask(r)) = (xaddr(r)(XLEN-1 downto pmp_lsb_c) and addr_mask(r))) else '0';
    check.d_cmp_mm(r) <= '1' when ((addr_ls_i(XLEN-1 downto pmp_lsb_c) and addr_mask(r)) = (xaddr(r)(XLEN-1 downto pmp_lsb_c) and addr_mask(r))) else '0';
    -- TOR region 0 --
    addr_check_r0_gen:
    if (r = 0) generate -- first entry: use ZERO as base and current entry as bound
      check.i_cmp_ge(r) <= '1'; -- address is always greater than or equal to zero
      check.i_cmp_lt(r) <= '0'; -- unused
      check.d_cmp_ge(r) <= '1'; -- address is always greater than or equal to zero
      check.d_cmp_lt(r) <= '0'; -- unused
    end generate;
    -- TOR region any --
    addr_check_rx_gen:
    if (r > 0) generate -- use previous entry as base and current entry as bound
      check.i_cmp_ge(r) <= '1' when (unsigned(addr_if_i(XLEN-1 downto pmp_lsb_c)) >= unsigned(xaddr(r-1)(XLEN-1 downto pmp_lsb_c))) else '0';
      check.i_cmp_lt(r) <= '1' when (unsigned(addr_if_i(XLEN-1 downto pmp_lsb_c)) <  unsigned(xaddr(r  )(XLEN-1 downto pmp_lsb_c))) else '0';
      check.d_cmp_ge(r) <= '1' when (unsigned(addr_ls_i(XLEN-1 downto pmp_lsb_c)) >= unsigned(xaddr(r-1)(XLEN-1 downto pmp_lsb_c))) else '0';
      check.d_cmp_lt(r) <= '1' when (unsigned(addr_ls_i(XLEN-1 downto pmp_lsb_c)) <  unsigned(xaddr(r  )(XLEN-1 downto pmp_lsb_c))) else '0';
    end generate;


    -- check region match according to configured mode --
    match_check: process(csr, check)
    begin
      case csr.cfg(r)(cfg_ah_c downto cfg_al_c) is
        when mode_off_c => -- entry disabled
          check.i_match(r) <= '0';
          check.d_match(r) <= '0';
        when mode_tor_c => -- top of region
          if (r = (NUM_REGIONS-1)) then -- very last entry
            check.i_match(r) <= check.i_cmp_ge(r) and check.i_cmp_lt(r);
            check.d_match(r) <= check.d_cmp_ge(r) and check.d_cmp_lt(r);
          else -- this saves a LOT of comparators
            check.i_match(r) <= check.i_cmp_ge(r) and (not check.i_cmp_ge(r+1));
            check.d_match(r) <= check.d_cmp_ge(r) and (not check.d_cmp_ge(r+1));
          end if;
        when others => -- naturally-aligned region
          check.i_match(r) <= check.i_cmp_mm(r);
          check.d_match(r) <= check.d_cmp_mm(r);
        end case;
    end process match_check;


    -- compute region permission --
    perm_check: process(csr.cfg, ctrl_i)
    begin
      -- execute (X) --
      if (ctrl_i.cpu_priv = priv_mode_m_c) then -- M mode: always allow if lock bit
        check.perm_ex(r) <= csr.cfg(r)(cfg_x_c) or (not csr.cfg(r)(cfg_l_c));
      else -- U mode: check actual permission
        check.perm_ex(r) <= csr.cfg(r)(cfg_x_c);
      end if;
      -- read (R) --
      if (ctrl_i.lsu_rw = '0') then
        if (ctrl_i.lsu_priv = priv_mode_m_c) then -- M mode: always allow if lock bit
          check.perm_rw(r) <= csr.cfg(r)(cfg_r_c) or (not csr.cfg(r)(cfg_l_c));
        else -- U mode: check actual permission
          check.perm_rw(r) <= csr.cfg(r)(cfg_r_c);
        end if;
      -- write (W) --
      else
        if (ctrl_i.lsu_priv = priv_mode_m_c) then -- M mode: always allow if lock bit
          check.perm_rw(r) <= csr.cfg(r)(cfg_w_c) or (not csr.cfg(r)(cfg_l_c));
        else -- U mode: check actual permission
          check.perm_rw(r) <= csr.cfg(r)(cfg_w_c);
        end if;
      end if;
    end process perm_check;

  end generate;


  -- check for access fault (using static prioritization) --
  check.fail_ex(NUM_REGIONS) <= '1' when (ctrl_i.cpu_priv /= priv_mode_m_c) else '0'; -- default (if not match): fault if not M-mode
  check.fail_rw(NUM_REGIONS) <= '1' when (ctrl_i.lsu_priv /= priv_mode_m_c) else '0'; -- default (if not match): fault if not M-mode
  -- this is a *structural* description of a prioritization logic implemented as a multiplexer chain --
  fault_check_gen:
  for r in NUM_REGIONS-1 downto 0 generate -- start with lowest priority
    check.fail_ex(r) <= not check.perm_ex(r) when (check.i_match(r) = '1') else check.fail_ex(r+1);
    check.fail_rw(r) <= not check.perm_rw(r) when (check.d_match(r) = '1') else check.fail_rw(r+1);
  end generate;


  -- final PMP access fault signals (ignore PMP rules when in debug mode) --
  fault_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      fault_ex_o <= '0';
      fault_rd_o <= '0';
      fault_wr_o <= '0';
    elsif rising_edge(clk_i) then
      fault_ex_o <= (not ctrl_i.cpu_debug) and check.fail_ex(0);
      fault_rd_o <= (not ctrl_i.cpu_debug) and check.fail_rw(0) and (not ctrl_i.lsu_rw);
      fault_wr_o <= (not ctrl_i.cpu_debug) and check.fail_rw(0) and (    ctrl_i.lsu_rw);
    end if;
  end process fault_reg;


end neorv32_cpu_pmp_rtl;
