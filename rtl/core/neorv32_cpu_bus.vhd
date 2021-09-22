-- #################################################################################################
-- # << NEORV32 - Bus Interface Unit >>                                                            #
-- # ********************************************************************************************* #
-- # Instruction and data bus interfaces and physical memory protection (PMP).                     #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
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

entity neorv32_cpu_bus is
  generic (
    CPU_EXTENSION_RISCV_A : boolean; -- implement atomic extension?
    CPU_EXTENSION_RISCV_C : boolean; -- implement compressed extension?
    -- Physical memory protection (PMP) --
    PMP_NUM_REGIONS       : natural; -- number of regions (0..64)
    PMP_MIN_GRANULARITY   : natural  -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
  );
  port (
    -- global control --
    clk_i          : in  std_ulogic; -- global clock, rising edge
    rstn_i         : in  std_ulogic := '0'; -- global reset, low-active, async
    ctrl_i         : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
    -- cpu instruction fetch interface --
    fetch_pc_i     : in  std_ulogic_vector(data_width_c-1 downto 0); -- PC for instruction fetch
    instr_o        : out std_ulogic_vector(data_width_c-1 downto 0); -- instruction
    i_wait_o       : out std_ulogic; -- wait for fetch to complete
    --
    ma_instr_o     : out std_ulogic; -- misaligned instruction address
    be_instr_o     : out std_ulogic; -- bus error on instruction access
    -- cpu data access interface --
    addr_i         : in  std_ulogic_vector(data_width_c-1 downto 0); -- ALU result -> access address
    wdata_i        : in  std_ulogic_vector(data_width_c-1 downto 0); -- write data
    rdata_o        : out std_ulogic_vector(data_width_c-1 downto 0); -- read data
    mar_o          : out std_ulogic_vector(data_width_c-1 downto 0); -- current memory address register
    d_wait_o       : out std_ulogic; -- wait for access to complete
    --
    excl_state_o   : out std_ulogic; -- atomic/exclusive access status
    ma_load_o      : out std_ulogic; -- misaligned load data address
    ma_store_o     : out std_ulogic; -- misaligned store data address
    be_load_o      : out std_ulogic; -- bus error on load data access
    be_store_o     : out std_ulogic; -- bus error on store data access
    -- physical memory protection --
    pmp_addr_i     : in  pmp_addr_if_t; -- addresses
    pmp_ctrl_i     : in  pmp_ctrl_if_t; -- configs
    -- instruction bus --
    i_bus_addr_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
    i_bus_rdata_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
    i_bus_wdata_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
    i_bus_ben_o    : out std_ulogic_vector(03 downto 0); -- byte enable
    i_bus_we_o     : out std_ulogic; -- write enable
    i_bus_re_o     : out std_ulogic; -- read enable
    i_bus_lock_o   : out std_ulogic; -- exclusive access request
    i_bus_ack_i    : in  std_ulogic; -- bus transfer acknowledge
    i_bus_err_i    : in  std_ulogic; -- bus transfer error
    i_bus_fence_o  : out std_ulogic; -- fence operation
    -- data bus --
    d_bus_addr_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
    d_bus_rdata_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
    d_bus_wdata_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
    d_bus_ben_o    : out std_ulogic_vector(03 downto 0); -- byte enable
    d_bus_we_o     : out std_ulogic; -- write enable
    d_bus_re_o     : out std_ulogic; -- read enable
    d_bus_lock_o   : out std_ulogic; -- exclusive access request
    d_bus_ack_i    : in  std_ulogic; -- bus transfer acknowledge
    d_bus_err_i    : in  std_ulogic; -- bus transfer error
    d_bus_fence_o  : out std_ulogic  -- fence operation
  );
end neorv32_cpu_bus;

architecture neorv32_cpu_bus_rtl of neorv32_cpu_bus is

  -- PMP modes --
  constant pmp_off_mode_c   : std_ulogic_vector(1 downto 0) := "00"; -- null region (disabled)
--constant pmp_tor_mode_c   : std_ulogic_vector(1 downto 0) := "01"; -- top of range
--constant pmp_na4_mode_c   : std_ulogic_vector(1 downto 0) := "10"; -- naturally aligned four-byte region
  constant pmp_napot_mode_c : std_ulogic_vector(1 downto 0) := "11"; -- naturally aligned power-of-two region (>= 8 bytes)

  -- PMP granularity --
  constant pmp_g_c : natural := index_size_f(PMP_MIN_GRANULARITY);

  -- PMP configuration register bits --
  constant pmp_cfg_r_c  : natural := 0; -- read permit
  constant pmp_cfg_w_c  : natural := 1; -- write permit
  constant pmp_cfg_x_c  : natural := 2; -- execute permit
  constant pmp_cfg_al_c : natural := 3; -- mode bit low
  constant pmp_cfg_ah_c : natural := 4; -- mode bit high
  --
  constant pmp_cfg_l_c  : natural := 7; -- locked entry

  -- data interface registers --
  signal mar, mdo, mdi : std_ulogic_vector(data_width_c-1 downto 0);

  -- data access --
  signal d_bus_wdata : std_ulogic_vector(data_width_c-1 downto 0); -- write data
  signal d_bus_rdata : std_ulogic_vector(data_width_c-1 downto 0); -- read data
  signal rdata_align : std_ulogic_vector(data_width_c-1 downto 0); -- read-data alignment
  signal d_bus_ben   : std_ulogic_vector(3 downto 0); -- write data byte enable

  -- misaligned access? --
  signal d_misaligned, i_misaligned : std_ulogic;

  -- bus arbiter --
  type bus_arbiter_t is record
    rd_req    : std_ulogic; -- read access in progress
    wr_req    : std_ulogic; -- write access in progress
    err_align : std_ulogic; -- alignment error
    err_bus   : std_ulogic; -- bus access error
  end record;
  signal i_arbiter, d_arbiter : bus_arbiter_t;

  -- atomic/exclusive access - reservation controller --
  signal exclusive_lock        : std_ulogic;
  signal exclusive_lock_status : std_ulogic_vector(data_width_c-1 downto 0); -- read data

  -- physical memory protection --
  type pmp_addr_t is array (0 to PMP_NUM_REGIONS-1) of std_ulogic_vector(data_width_c-1 downto 0);
  type pmp_t is record
    addr_mask     : pmp_addr_t;
    region_base   : pmp_addr_t; -- region config base address
    region_i_addr : pmp_addr_t; -- masked instruction access base address for comparator
    region_d_addr : pmp_addr_t; -- masked data access base address for comparator
    i_match       : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0); -- region match for instruction interface
    d_match       : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0); -- region match for data interface
    if_fault      : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0); -- region access fault for fetch operation
    ld_fault      : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0); -- region access fault for load operation
    st_fault      : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0); -- region access fault for store operation
  end record;
  signal pmp : pmp_t;

  -- memory control signal buffer (when using PMP) --
  signal d_bus_we, d_bus_we_buf : std_ulogic;
  signal d_bus_re, d_bus_re_buf : std_ulogic;
  signal i_bus_re, i_bus_re_buf : std_ulogic;

  -- pmp faults anyone? --
  signal if_pmp_fault : std_ulogic; -- pmp instruction access fault
  signal ld_pmp_fault : std_ulogic; -- pmp load access fault
  signal st_pmp_fault : std_ulogic; -- pmp store access fault

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (PMP_NUM_REGIONS > pmp_num_regions_critical_c) report "NEORV32 CPU CONFIG WARNING! Number of implemented PMP regions (PMP_NUM_REGIONS = " &
  integer'image(PMP_NUM_REGIONS) & ") beyond critical limit (pmp_num_regions_critical_c = " & integer'image(pmp_num_regions_critical_c) &
  "). Inserting another register stage (that will increase memory latency by +1 cycle)." severity warning;


  -- Data Interface: Access Address ---------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_adr_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      mar <= (others => def_rst_val_c);
    elsif rising_edge(clk_i) then
      if (ctrl_i(ctrl_bus_mo_we_c) = '1') then
        mar <= addr_i;
      end if;
    end if;
  end process mem_adr_reg;

  -- read-back for exception controller --
  mar_o <= mar;

  -- alignment check --
  misaligned_d_check: process(mar, ctrl_i)
  begin
    -- check data access --
    d_misaligned <= '0'; -- default
    case ctrl_i(ctrl_bus_size_msb_c downto ctrl_bus_size_lsb_c) is -- data size
      when "00" => -- byte
        d_misaligned <= '0';
      when "01" => -- half-word
        if (mar(0) /= '0') then
          d_misaligned <= '1';
        end if;
      when others => -- word
        if (mar(1 downto 0) /= "00") then
          d_misaligned <= '1';
        end if;
    end case;
  end process misaligned_d_check;


  -- Data Interface: Write Data -------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_do_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      mdo <= (others => def_rst_val_c);
    elsif rising_edge(clk_i) then
      if (ctrl_i(ctrl_bus_mo_we_c) = '1') then
        mdo <= wdata_i; -- memory data output register (MDO)
      end if;
    end if;
  end process mem_do_reg;

  -- byte enable and output data alignment --
  byte_enable: process(mar, mdo, ctrl_i)
  begin
    case ctrl_i(ctrl_bus_size_msb_c downto ctrl_bus_size_lsb_c) is -- data size
      when "00" => -- byte
        d_bus_wdata(07 downto 00) <= mdo(07 downto 00);
        d_bus_wdata(15 downto 08) <= mdo(07 downto 00);
        d_bus_wdata(23 downto 16) <= mdo(07 downto 00);
        d_bus_wdata(31 downto 24) <= mdo(07 downto 00);
        case mar(1 downto 0) is
          when "00"   => d_bus_ben <= "0001";
          when "01"   => d_bus_ben <= "0010";
          when "10"   => d_bus_ben <= "0100";
          when others => d_bus_ben <= "1000";
        end case;
      when "01" => -- half-word
        d_bus_wdata(31 downto 16) <= mdo(15 downto 00);
        d_bus_wdata(15 downto 00) <= mdo(15 downto 00);
        if (mar(1) = '0') then
          d_bus_ben <= "0011"; -- low half-word
        else
          d_bus_ben <= "1100"; -- high half-word
        end if;
      when others => -- word
        d_bus_wdata <= mdo;
        d_bus_ben   <= "1111"; -- full word
    end case;
  end process byte_enable;


  -- Data Interface: Read Data --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_di_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      mdi <= (others => def_rst_val_c);
    elsif rising_edge(clk_i) then
      if (ctrl_i(ctrl_bus_mi_we_c) = '1') then
        mdi <= d_bus_rdata; -- memory data input register (MDI)
      end if;
    end if;
  end process mem_di_reg;

  -- input data alignment and sign extension --
  read_align: process(mdi, mar, ctrl_i)
    variable byte_in_v  : std_ulogic_vector(07 downto 0); 
    variable hword_in_v : std_ulogic_vector(15 downto 0);
  begin
    -- sub-word input --
    case mar(1 downto 0) is
      when "00"   => byte_in_v := mdi(07 downto 00); hword_in_v := mdi(15 downto 00); -- byte 0 / half-word 0
      when "01"   => byte_in_v := mdi(15 downto 08); hword_in_v := mdi(15 downto 00); -- byte 1 / half-word 0
      when "10"   => byte_in_v := mdi(23 downto 16); hword_in_v := mdi(31 downto 16); -- byte 2 / half-word 1
      when others => byte_in_v := mdi(31 downto 24); hword_in_v := mdi(31 downto 16); -- byte 3 / half-word 1
    end case;
    -- actual data size --
    case ctrl_i(ctrl_bus_size_msb_c downto ctrl_bus_size_lsb_c) is
      when "00" => -- byte
        rdata_align(31 downto 08) <= (others => ((not ctrl_i(ctrl_bus_unsigned_c)) and byte_in_v(7))); -- sign extension
        rdata_align(07 downto 00) <= byte_in_v;
      when "01" => -- half-word
        rdata_align(31 downto 16) <= (others => ((not ctrl_i(ctrl_bus_unsigned_c)) and hword_in_v(15))); -- sign extension
        rdata_align(15 downto 00) <= hword_in_v; -- high half-word
      when others => -- word
        rdata_align <= mdi; -- full word
    end case;
  end process read_align;

  -- insert exclusive lock status for SC operations only --
  rdata_o <= exclusive_lock_status when (CPU_EXTENSION_RISCV_A = true) and (ctrl_i(ctrl_bus_ch_lock_c) = '1') else rdata_align;


  -- Data Access Arbiter --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  data_access_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      d_arbiter.wr_req    <= '0';
      d_arbiter.rd_req    <= '0';
      d_arbiter.err_align <= '0';
      d_arbiter.err_bus   <= '0';
    elsif rising_edge(clk_i) then
      -- data access request --
      if (d_arbiter.wr_req = '0') and (d_arbiter.rd_req = '0') then -- idle
        d_arbiter.wr_req    <= ctrl_i(ctrl_bus_wr_c);
        d_arbiter.rd_req    <= ctrl_i(ctrl_bus_rd_c);
        d_arbiter.err_align <= d_misaligned;
        d_arbiter.err_bus   <= '0';
      else -- in progress
        d_arbiter.err_align <= (d_arbiter.err_align or d_misaligned) and (not ctrl_i(ctrl_bus_derr_ack_c));
        d_arbiter.err_bus   <= (d_arbiter.err_bus or d_bus_err_i or (st_pmp_fault and d_arbiter.wr_req) or (ld_pmp_fault and d_arbiter.rd_req)) and
                               (not ctrl_i(ctrl_bus_derr_ack_c));
        if (d_bus_ack_i = '1') or (ctrl_i(ctrl_bus_derr_ack_c) = '1') then -- wait for normal termination / CPU abort
          d_arbiter.wr_req <= '0';
          d_arbiter.rd_req <= '0';
        end if;
      end if;
    end if;
  end process data_access_arbiter;

  -- wait for bus transaction to finish --
  d_wait_o <= (d_arbiter.wr_req or d_arbiter.rd_req) and (not d_bus_ack_i);

  -- output data access error to controller --
  ma_load_o  <= d_arbiter.rd_req and d_arbiter.err_align;
  be_load_o  <= d_arbiter.rd_req and d_arbiter.err_bus;
  ma_store_o <= d_arbiter.wr_req and d_arbiter.err_align;
  be_store_o <= d_arbiter.wr_req and d_arbiter.err_bus;

  -- data bus (read/write)--
  d_bus_addr_o  <= mar;
  d_bus_wdata_o <= d_bus_wdata;
  d_bus_ben_o   <= d_bus_ben;
  d_bus_we      <= ctrl_i(ctrl_bus_wr_c) and (not d_misaligned) and (not st_pmp_fault); -- no actual write when misaligned or PMP fault
  d_bus_re      <= ctrl_i(ctrl_bus_rd_c) and (not d_misaligned) and (not ld_pmp_fault); -- no actual read when misaligned or PMP fault
  d_bus_we_o    <= d_bus_we_buf when (PMP_NUM_REGIONS > pmp_num_regions_critical_c) else d_bus_we;
  d_bus_re_o    <= d_bus_re_buf when (PMP_NUM_REGIONS > pmp_num_regions_critical_c) else d_bus_re;
  d_bus_fence_o <= ctrl_i(ctrl_bus_fence_c);
  d_bus_rdata   <= d_bus_rdata_i;

  -- additional register stage for control signals if using PMP_NUM_REGIONS > pmp_num_regions_critical_c --
  pmp_dbus_buffer: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      d_bus_we_buf <= '0';
      d_bus_re_buf <= '0';
    elsif rising_edge(clk_i) then
      d_bus_we_buf <= d_bus_we;
      d_bus_re_buf <= d_bus_re;
    end if;
  end process pmp_dbus_buffer;


  -- Reservation Controller (LR/SC [A extension]) -------------------------------------------
  -- -------------------------------------------------------------------------------------------
  exclusive_access_controller: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      exclusive_lock <= '0';
    elsif rising_edge(clk_i) then
      if (CPU_EXTENSION_RISCV_A = true) then
        if (ctrl_i(ctrl_trap_c) = '1') or (ctrl_i(ctrl_bus_de_lock_c) = '1') then -- remove lock if entering a trap or executing a non-load-reservate memory access
          exclusive_lock <= '0';
        elsif (ctrl_i(ctrl_bus_lock_c) = '1') then -- set new lock
          exclusive_lock <= '1';
        end if;
      else
        exclusive_lock <= '0';
      end if;
    end if;
  end process exclusive_access_controller;

  -- lock status for SC operation --
  exclusive_lock_status(data_width_c-1 downto 1) <= (others => '0');
  exclusive_lock_status(0) <= not exclusive_lock;

  -- output reservation status to control unit (to check if SC should write at all) --
  excl_state_o <= exclusive_lock;

  -- output to memory system --
  i_bus_lock_o <= '0'; -- instruction fetches cannot be locked
  d_bus_lock_o <= exclusive_lock;


  -- Instruction Fetch Arbiter --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ifetch_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      i_arbiter.rd_req    <= '0';
      i_arbiter.err_align <= '0';
      i_arbiter.err_bus   <= '0';
    elsif rising_edge(clk_i) then
      -- instruction fetch request --
      if (i_arbiter.rd_req = '0') then -- idle
        i_arbiter.rd_req    <= ctrl_i(ctrl_bus_if_c);
        i_arbiter.err_align <= i_misaligned;
        i_arbiter.err_bus   <= '0';
      else -- in progres
        i_arbiter.err_align <= (i_arbiter.err_align or i_misaligned) and (not ctrl_i(ctrl_bus_ierr_ack_c));
        i_arbiter.err_bus   <= (i_arbiter.err_bus or i_bus_err_i or if_pmp_fault) and (not ctrl_i(ctrl_bus_ierr_ack_c));
        if (i_bus_ack_i = '1') or (ctrl_i(ctrl_bus_ierr_ack_c) = '1') then -- wait for normal termination / CPU abort
          i_arbiter.rd_req <= '0';
        end if;
      end if;
    end if;
  end process ifetch_arbiter;

  i_arbiter.wr_req <= '0'; -- instruction fetch is read-only

  -- wait for bus transaction to finish --
  i_wait_o <= i_arbiter.rd_req and (not i_bus_ack_i);

  -- output instruction fetch error to controller --
  ma_instr_o <= i_arbiter.err_align;
  be_instr_o <= i_arbiter.err_bus;

  -- instruction bus (read-only) --
  i_bus_addr_o  <= fetch_pc_i(data_width_c-1 downto 2) & "00"; -- instruction access is always 4-byte aligned (even for compressed instructions)
  i_bus_wdata_o <= (others => '0'); -- instruction fetch is read-only
  i_bus_ben_o   <= (others => '0');
  i_bus_we_o    <= '0';
  i_bus_re      <= ctrl_i(ctrl_bus_if_c) and (not i_misaligned) and (not if_pmp_fault); -- no actual read when misaligned or PMP fault
  i_bus_re_o    <= i_bus_re_buf when (PMP_NUM_REGIONS > pmp_num_regions_critical_c) else i_bus_re;
  i_bus_fence_o <= ctrl_i(ctrl_bus_fencei_c);
  instr_o       <= i_bus_rdata_i;

  -- check instruction access address alignment --
  i_misaligned <= '0' when (CPU_EXTENSION_RISCV_C = true) else -- no alignment exceptions possible when using C-extension
                  '1' when (fetch_pc_i(1) = '1') else '0'; -- 32-bit accesses only

  -- additional register stage for control signals if using PMP_NUM_REGIONS > pmp_num_regions_critical_c --
  pmp_ibus_buffer: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      i_bus_re_buf <= '0';
    elsif rising_edge(clk_i) then
      i_bus_re_buf <= i_bus_re;
    end if;
  end process pmp_ibus_buffer;


  -- Physical Memory Protection (PMP) -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- compute address masks (ITERATIVE!!!) --
  pmp_masks: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      pmp.addr_mask <= (others => (others => def_rst_val_c));
    elsif rising_edge(clk_i) then -- address mask computation (not the actual address check!) has a latency of max +32 cycles
      for r in 0 to PMP_NUM_REGIONS-1 loop -- iterate over all regions
        pmp.addr_mask(r) <= (others => '0');
        for i in pmp_g_c to data_width_c-1 loop
          pmp.addr_mask(r)(i) <= pmp.addr_mask(r)(i-1) or (not pmp_addr_i(r)(i-1));
        end loop; -- i
      end loop; -- r
    end if;
  end process pmp_masks;


  -- address access check --
  pmp_address_check:
  for r in 0 to PMP_NUM_REGIONS-1 generate -- iterate over all regions
    pmp.region_i_addr(r) <= fetch_pc_i                             and pmp.addr_mask(r);
    pmp.region_d_addr(r) <= mar                                    and pmp.addr_mask(r);
    pmp.region_base(r)   <= pmp_addr_i(r)(data_width_c+1 downto 2) and pmp.addr_mask(r);
    --
    pmp.i_match(r) <= '1' when (pmp.region_i_addr(r)(data_width_c-1 downto pmp_g_c) = pmp.region_base(r)(data_width_c-1 downto pmp_g_c)) else '0';
    pmp.d_match(r) <= '1' when (pmp.region_d_addr(r)(data_width_c-1 downto pmp_g_c) = pmp.region_base(r)(data_width_c-1 downto pmp_g_c)) else '0';
  end generate; -- r


  -- check access type and region's permissions --
  pmp_check_permission: process(pmp, pmp_ctrl_i, ctrl_i)
  begin
    for r in 0 to PMP_NUM_REGIONS-1 loop -- iterate over all regions
      if ((ctrl_i(ctrl_priv_lvl_msb_c downto ctrl_priv_lvl_lsb_c) = priv_mode_u_c) or (pmp_ctrl_i(r)(pmp_cfg_l_c) = '1')) and -- user privilege level or locked pmp entry -> enforce permissions also for machine mode
         (pmp_ctrl_i(r)(pmp_cfg_ah_c downto pmp_cfg_al_c) /= pmp_off_mode_c) and -- active entry
         (ctrl_i(ctrl_debug_running_c) = '0') then -- disable PMP checks when in debug mode
        pmp.if_fault(r) <= pmp.i_match(r) and (not pmp_ctrl_i(r)(pmp_cfg_x_c)); -- fetch access match no execute permission
        pmp.ld_fault(r) <= pmp.d_match(r) and (not pmp_ctrl_i(r)(pmp_cfg_r_c)); -- load access match no read permission
        pmp.st_fault(r) <= pmp.d_match(r) and (not pmp_ctrl_i(r)(pmp_cfg_w_c)); -- store access match no write permission
      else
        pmp.if_fault(r) <= '0';
        pmp.ld_fault(r) <= '0';
        pmp.st_fault(r) <= '0';
      end if;
    end loop; -- r
  end process pmp_check_permission;


  -- final PMP access fault signals --
  if_pmp_fault <= or_reduce_f(pmp.if_fault) when (PMP_NUM_REGIONS > 0) else '0';
  ld_pmp_fault <= or_reduce_f(pmp.ld_fault) when (PMP_NUM_REGIONS > 0) else '0';
  st_pmp_fault <= or_reduce_f(pmp.st_fault) when (PMP_NUM_REGIONS > 0) else '0';


end neorv32_cpu_bus_rtl;
