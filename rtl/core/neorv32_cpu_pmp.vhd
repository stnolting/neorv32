-- ================================================================================ --
-- NEORV32 CPU - Physical Memory Protection Unit (RISC-V "Smpmp" Extension)         --
-- -------------------------------------------------------------------------------- --
-- Compatible to the RISC-V PMP privilege architecture specifications. Granularity  --
-- and supported modes can be constrained via generics to reduce area requirements. --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cpu_pmp is
  generic (
    NUM_REGIONS : natural range 0 to 16; -- number of regions (0..16)
    GRANULARITY : natural; -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
    TOR_EN      : boolean; -- implement TOR mode
    NAP_EN      : boolean  -- implement NAPOT/NA4 modes
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
    fault_rw_o  : out std_ulogic  -- read/write access fault
  );
end neorv32_cpu_pmp;

architecture neorv32_cpu_pmp_rtl of neorv32_cpu_pmp is

  -- auto-configuration --
  constant granularity_c : natural := cond_sel_natural_f(boolean(GRANULARITY < 4), 4, 2**index_size_f(GRANULARITY));

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
  constant pmp_lsb_c : natural := index_size_f(granularity_c); -- min = 2

  -- PMP CSRs --
  type csr_cfg_t      is array (0 to NUM_REGIONS-1) of std_ulogic_vector(7 downto 0);
  type csr_addr_t     is array (0 to NUM_REGIONS-1) of std_ulogic_vector(XLEN-1 downto 0);
  type csr_cfg_rd_t   is array (0 to 15) of std_ulogic_vector(7 downto 0);
  type csr_cfg_rd32_t is array (0 to 03) of std_ulogic_vector(XLEN-1 downto 0);
  type csr_addr_rd_t  is array (0 to 15) of std_ulogic_vector(XLEN-1 downto 0);
  type csr_t is record
    we_cfg  : std_ulogic_vector(3 downto 0);
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

  -- region access logic --
  type addr_mask_t is array (0 to NUM_REGIONS-1) of std_ulogic_vector(XLEN-1 downto pmp_lsb_c);
  signal addr_mask_napot, addr_mask : addr_mask_t;
  type region_t is record
    i_cmp_mm, d_cmp_mm : std_ulogic_vector(NUM_REGIONS-1 downto 0); -- masked match
    i_cmp_ge, d_cmp_ge : std_ulogic_vector(NUM_REGIONS-1 downto 0); -- greater or equal
    i_cmp_lt, d_cmp_lt : std_ulogic_vector(NUM_REGIONS-1 downto 0); -- less than
    i_match,  d_match  : std_ulogic_vector(NUM_REGIONS-1 downto 0); -- region address match
    perm_ex,  perm_rw  : std_ulogic_vector(NUM_REGIONS-1 downto 0); -- region's permission
  end record;
  signal region : region_t;

  -- permission check violation --
  signal fail_ex, fail_rw : std_ulogic_vector(NUM_REGIONS downto 0);

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert (GRANULARITY = granularity_c) report
    "[NEORV32] Auto-adjusting invalid PMP granularity configuration." severity warning;


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
      variable mode_v : std_ulogic_vector(1 downto 0);
    begin
      if (rstn_i = '0') then
        csr.cfg(i)  <= (others => '0');
        csr.addr(i) <= (others => '0');
      elsif rising_edge(clk_i) then

        -- configuration --
        if (csr.we_cfg(i/4) = '1') and (csr.cfg(i)(cfg_l_c) = '0') then -- unlocked write access
          csr.cfg(i)(cfg_r_c) <= csr_wdata_i((i mod 4)*8+cfg_r_c); -- R (read)
          csr.cfg(i)(cfg_w_c) <= csr_wdata_i((i mod 4)*8+cfg_w_c); -- W (write)
          csr.cfg(i)(cfg_x_c) <= csr_wdata_i((i mod 4)*8+cfg_x_c); -- X (execute)
          -- A (mode) --
          mode_v := csr_wdata_i((i mod 4)*8+cfg_ah_c downto (i mod 4)*8+cfg_al_c);
          if ((mode_v = mode_tor_c)   and (not TOR_EN)) or -- TOR mode not implemented
             ((mode_v = mode_na4_c)   and (not NAP_EN)) or -- NA4 mode not implemented
             ((mode_v = mode_napot_c) and (not NAP_EN)) or -- NAPOT mode not implemented
             ((mode_v = mode_na4_c)   and (granularity_c > 4)) then -- NA4 not available
            csr.cfg(i)(cfg_ah_c downto cfg_al_c) <= mode_off_c;
          else -- valid configuration
            csr.cfg(i)(cfg_ah_c downto cfg_al_c) <= mode_v;
          end if;
          --
          csr.cfg(i)(cfg_rl_c) <= '0'; -- reserved
          csr.cfg(i)(cfg_rh_c) <= '0'; -- reserved
          csr.cfg(i)(cfg_l_c)  <= csr_wdata_i((i mod 4)*8+cfg_l_c); -- L (locked)
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
      if (granularity_c = 8) and TOR_EN then -- bit G-1 reads as zero in TOR or OFF mode
        if (csr.cfg(i)(cfg_ah_c) = '0') then -- TOR/OFF mode
          addr_rd(i)(pmp_lsb_c) <= '0';
        end if;
      elsif (granularity_c > 8) then
        if NAP_EN then
          addr_rd(i)(pmp_lsb_c-2 downto 0) <= (others => '1'); -- in NAPOT mode bits G-2:0 must read as one
        end if;
        if TOR_EN then
          if (csr.cfg(i)(cfg_ah_c) = '0') then -- TOR/OFF mode
            addr_rd(i)(pmp_lsb_c-1 downto 0) <= (others => '0'); -- in TOR or OFF mode bits G-1:0 must read as zero
          end if;
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
  csr_read_back_pack:
  for i in 0 to 3 generate
    cfg_rd32(i) <= cfg_rd(i*4+3) & cfg_rd(i*4+2) & cfg_rd(i*4+1) & cfg_rd(i*4+0);
  end generate;


  -- Region Access Logic --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  region_gen:
  for r in 0 to NUM_REGIONS-1 generate

    -- extend region addresses to 34-bit --
    xaddr(r) <= csr.addr(r) & "00"; -- mask byte offset


    nap_mode_enable:
    if NAP_EN generate

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

    end generate; -- /nap_mode_enable


    -- check region address match --
    -- NA4 and NAPOT --
    region.i_cmp_mm(r) <= '1' when ((addr_if_i(XLEN-1 downto pmp_lsb_c) and addr_mask(r)) = (xaddr(r)(XLEN-1 downto pmp_lsb_c) and addr_mask(r))) and NAP_EN else '0';
    region.d_cmp_mm(r) <= '1' when ((addr_ls_i(XLEN-1 downto pmp_lsb_c) and addr_mask(r)) = (xaddr(r)(XLEN-1 downto pmp_lsb_c) and addr_mask(r))) and NAP_EN else '0';
    -- TOR region 0 --
    addr_match_r0_gen:
    if (r = 0) generate -- first entry: use ZERO as base and current entry as bound
      region.i_cmp_ge(r) <= '1' when TOR_EN else '0'; -- address is always greater than or equal to zero (and TOR mode enabled)
      region.i_cmp_lt(r) <= '0'; -- unused
      region.d_cmp_ge(r) <= '1' when TOR_EN else '0'; -- address is always greater than or equal to zero (and TOR mode enabled)
      region.d_cmp_lt(r) <= '0'; -- unused
    end generate;
    -- TOR region any --
    addr_match_rx_gen:
    if (r > 0) generate -- use previous entry as base and current entry as bound
      region.i_cmp_ge(r) <= '1' when (unsigned(addr_if_i(XLEN-1 downto pmp_lsb_c)) >= unsigned(xaddr(r-1)(XLEN-1 downto pmp_lsb_c))) and TOR_EN else '0';
      region.i_cmp_lt(r) <= '1' when (unsigned(addr_if_i(XLEN-1 downto pmp_lsb_c)) <  unsigned(xaddr(r  )(XLEN-1 downto pmp_lsb_c))) and TOR_EN else '0';
      region.d_cmp_ge(r) <= '1' when (unsigned(addr_ls_i(XLEN-1 downto pmp_lsb_c)) >= unsigned(xaddr(r-1)(XLEN-1 downto pmp_lsb_c))) and TOR_EN else '0';
      region.d_cmp_lt(r) <= '1' when (unsigned(addr_ls_i(XLEN-1 downto pmp_lsb_c)) <  unsigned(xaddr(r  )(XLEN-1 downto pmp_lsb_c))) and TOR_EN else '0';
    end generate;


    -- check region match according to configured mode --
    match_gen: process(csr, region)
      variable tmp_v : std_ulogic_vector(1 downto 0);
    begin
      tmp_v := csr.cfg(r)(cfg_ah_c downto cfg_al_c);
      case tmp_v is -- VHDL/GHDL issue: "object type is not locally static"
        when mode_off_c => -- entry disabled
          region.i_match(r) <= '0';
          region.d_match(r) <= '0';
        when mode_tor_c => -- top of region
          if TOR_EN then -- TOR mode implemented?
            if (r = (NUM_REGIONS-1)) then -- very last entry
              region.i_match(r) <= region.i_cmp_ge(r) and region.i_cmp_lt(r);
              region.d_match(r) <= region.d_cmp_ge(r) and region.d_cmp_lt(r);
            else -- this saves a LOT of comparators
              region.i_match(r) <= region.i_cmp_ge(r) and (not region.i_cmp_ge(r+1));
              region.d_match(r) <= region.d_cmp_ge(r) and (not region.d_cmp_ge(r+1));
            end if;
          else
            region.i_match(r) <= '0';
            region.d_match(r) <= '0';
          end if;
        when others => -- naturally-aligned region
          if NAP_EN then -- NAPOT/NA4 modes implemented?
            region.i_match(r) <= region.i_cmp_mm(r);
            region.d_match(r) <= region.d_cmp_mm(r);
          else
            region.i_match(r) <= '0';
            region.d_match(r) <= '0';
          end if;
        end case;
    end process match_gen;


    -- compute region permissions --
    perm_gen: process(csr.cfg, ctrl_i)
    begin
      -- execute (X) --
      if (ctrl_i.cpu_priv = priv_mode_m_c) then
        region.perm_ex(r) <= csr.cfg(r)(cfg_x_c) or (not csr.cfg(r)(cfg_l_c)); -- M mode: always allow if not locked
      else
        region.perm_ex(r) <= csr.cfg(r)(cfg_x_c);
      end if;
      -- read (R) --
      if (ctrl_i.lsu_rw = '0') then
        if (ctrl_i.lsu_priv = priv_mode_m_c) then
          region.perm_rw(r) <= csr.cfg(r)(cfg_r_c) or (not csr.cfg(r)(cfg_l_c)); -- M mode: always allow if not locked
        else
          region.perm_rw(r) <= csr.cfg(r)(cfg_r_c);
        end if;
      -- write (W) --
      else
        if (ctrl_i.lsu_priv = priv_mode_m_c) then
          region.perm_rw(r) <= csr.cfg(r)(cfg_w_c) or (not csr.cfg(r)(cfg_l_c)); -- M mode: always allow if not locked
        else
          region.perm_rw(r) <= csr.cfg(r)(cfg_w_c);
        end if;
      end if;
    end process perm_gen;

  end generate;


  -- Access Permission Check ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- check for access fault (using static prioritization) --
  fail_ex(NUM_REGIONS) <= '1' when (ctrl_i.cpu_priv /= priv_mode_m_c) else '0'; -- default (if not match): fault if not M-mode
  fail_rw(NUM_REGIONS) <= '1' when (ctrl_i.lsu_priv /= priv_mode_m_c) else '0'; -- default (if not match): fault if not M-mode
  -- this is a *structural* description of a prioritization logic implemented as a multiplexer chain --
  fault_check_gen:
  for r in NUM_REGIONS-1 downto 0 generate -- start with lowest priority
    fail_ex(r) <= not region.perm_ex(r) when (region.i_match(r) = '1') else fail_ex(r+1);
    fail_rw(r) <= not region.perm_rw(r) when (region.d_match(r) = '1') else fail_rw(r+1);
  end generate;


  -- final access check --
  access_check: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      fault_ex_o <= '0';
      fault_rw_o <= '0';
    elsif rising_edge(clk_i) then
      fault_ex_o <= (not ctrl_i.cpu_debug) and fail_ex(0); -- ignore PMP rules when in debug mode
      fault_rw_o <= (not ctrl_i.cpu_debug) and fail_rw(0);
    end if;
  end process access_check;


end neorv32_cpu_pmp_rtl;
