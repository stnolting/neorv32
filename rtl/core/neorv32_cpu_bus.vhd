-- #################################################################################################
-- # << NEORV32 - (Data) Bus Interface Unit >>                                                     #
-- # ********************************************************************************************* #
-- # Data bus interface (load/store unit) and physical memory protection (PMP).                    #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
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
    PMP_NUM_REGIONS     : natural; -- number of regions (0..16)
    PMP_MIN_GRANULARITY : natural  -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
  );
  port (
    -- global control --
    clk_i         : in  std_ulogic; -- global clock, rising edge
    rstn_i        : in  std_ulogic := '0'; -- global reset, low-active, async
    ctrl_i        : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
    -- cpu instruction fetch interface --
    fetch_pc_i    : in  std_ulogic_vector(data_width_c-1 downto 0); -- PC for instruction fetch
    i_pmp_fault_o : out std_ulogic; -- instruction fetch pmp fault
    -- cpu data access interface --
    addr_i        : in  std_ulogic_vector(data_width_c-1 downto 0); -- ALU result -> access address
    wdata_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- write data
    rdata_o       : out std_ulogic_vector(data_width_c-1 downto 0); -- read data
    mar_o         : out std_ulogic_vector(data_width_c-1 downto 0); -- current memory address register
    d_wait_o      : out std_ulogic; -- wait for access to complete
    ma_load_o     : out std_ulogic; -- misaligned load data address
    ma_store_o    : out std_ulogic; -- misaligned store data address
    be_load_o     : out std_ulogic; -- bus error on load data access
    be_store_o    : out std_ulogic; -- bus error on store data access
    -- physical memory protection --
    pmp_addr_i    : in  pmp_addr_if_t; -- addresses
    pmp_ctrl_i    : in  pmp_ctrl_if_t; -- configs
    -- data bus --
    d_bus_addr_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
    d_bus_rdata_i : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
    d_bus_wdata_o : out std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
    d_bus_ben_o   : out std_ulogic_vector(03 downto 0); -- byte enable
    d_bus_we_o    : out std_ulogic; -- write enable
    d_bus_re_o    : out std_ulogic; -- read enable
    d_bus_ack_i   : in  std_ulogic; -- bus transfer acknowledge
    d_bus_err_i   : in  std_ulogic; -- bus transfer error
    d_bus_fence_o : out std_ulogic  -- fence operation
  );
end neorv32_cpu_bus;

architecture neorv32_cpu_bus_rtl of neorv32_cpu_bus is

  -- PMP modes --
  constant pmp_off_mode_c   : std_ulogic_vector(1 downto 0) := "00"; -- null region (disabled)
  constant pmp_tor_mode_c   : std_ulogic_vector(1 downto 0) := "01"; -- top of range
--constant pmp_na4_mode_c   : std_ulogic_vector(1 downto 0) := "10"; -- naturally aligned four-byte region
--constant pmp_napot_mode_c : std_ulogic_vector(1 downto 0) := "11"; -- naturally aligned power-of-two region (>= 8 bytes)

  -- PMP configuration register bits --
  constant pmp_cfg_r_c  : natural := 0; -- read permit
  constant pmp_cfg_w_c  : natural := 1; -- write permit
  constant pmp_cfg_x_c  : natural := 2; -- execute permit
  constant pmp_cfg_al_c : natural := 3; -- mode bit low
  constant pmp_cfg_ah_c : natural := 4; -- mode bit high
  constant pmp_cfg_l_c  : natural := 7; -- locked entry

  -- PMP minimal granularity --
  constant pmp_lsb_c : natural := index_size_f(PMP_MIN_GRANULARITY);

  -- misc --
  signal data_size  : std_ulogic_vector(1 downto 0); -- transfer size
  signal data_sign  : std_ulogic; -- signed load
  signal mar        : std_ulogic_vector(data_width_c-1 downto 0); -- data memory address register
  signal misaligned : std_ulogic; -- misaligned address

  -- bus arbiter --
  type bus_arbiter_t is record
    busy : std_ulogic; -- pending bus access
    rw   : std_ulogic; -- read/write access
    err  : std_ulogic; -- bus access error
  end record;
  signal arbiter : bus_arbiter_t;

  -- memory control signal buffer (when using PMP) --
  signal d_bus_we, d_bus_we_buf : std_ulogic;
  signal d_bus_re, d_bus_re_buf : std_ulogic;

  -- physical memory protection --
  type pmp_t is record
    i_match  : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0); -- region match for instruction interface
    d_match  : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0); -- region match for data interface
    if_fault : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0); -- region access fault for fetch operation
    ld_fault : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0); -- region access fault for load operation
    st_fault : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0); -- region access fault for store operation
  end record;
  signal pmp : pmp_t;

  -- pmp faults --
  signal if_pmp_fault : std_ulogic; -- pmp instruction access fault
  signal ld_pmp_fault : std_ulogic; -- pmp load access fault
  signal st_pmp_fault : std_ulogic; -- pmp store access fault

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (PMP_NUM_REGIONS > pmp_num_regions_critical_c) report "NEORV32 CPU CONFIG WARNING! Number of implemented PMP regions (PMP_NUM_REGIONS = " &
  integer'image(PMP_NUM_REGIONS) & ") beyond critical limit (pmp_num_regions_critical_c = " & integer'image(pmp_num_regions_critical_c) &
  "). Inserting another register stage (increasing DATA memory access latency by +1 cycle)." severity warning;


  -- Transfer Configuration -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  data_size <= ctrl_i(ctrl_ir_funct3_1_c downto ctrl_ir_funct3_0_c); -- transfer size
  data_sign <= not ctrl_i(ctrl_ir_funct3_2_c); -- NOT unsigned LOAD (LBU, LHU)


  -- Access Address -------------------------------------------------------------------------
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

  -- address output --
  d_bus_addr_o <= mar;
  mar_o        <= mar; -- for mtval csr (exceptions)

  -- data access address alignment check --
  misaligned_d_check: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      misaligned <= def_rst_val_c;
    elsif rising_edge(clk_i) then
      if (ctrl_i(ctrl_bus_mo_we_c) = '1') then
        case data_size is -- data size
          when "00"   => misaligned <= '0'; -- byte
          when "01"   => misaligned <= addr_i(0); -- half-word
          when others => misaligned <= addr_i(1) or addr_i(0); -- word
        end case;
      end if;
    end if;
  end process misaligned_d_check;


  -- Write Data -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_do_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      d_bus_wdata_o <= (others => def_rst_val_c);
      d_bus_ben_o   <= (others => def_rst_val_c);
    elsif rising_edge(clk_i) then
      if (ctrl_i(ctrl_bus_mo_we_c) = '1') then
        -- byte enable and data alignment --
        case data_size is -- data size
          when "00" => -- byte
            d_bus_wdata_o(07 downto 00) <= wdata_i(7 downto 0);
            d_bus_wdata_o(15 downto 08) <= wdata_i(7 downto 0);
            d_bus_wdata_o(23 downto 16) <= wdata_i(7 downto 0);
            d_bus_wdata_o(31 downto 24) <= wdata_i(7 downto 0);
            case addr_i(1 downto 0) is
              when "00"   => d_bus_ben_o <= "0001";
              when "01"   => d_bus_ben_o <= "0010";
              when "10"   => d_bus_ben_o <= "0100";
              when others => d_bus_ben_o <= "1000";
            end case;
          when "01" => -- half-word
            d_bus_wdata_o(31 downto 16) <= wdata_i(15 downto 0);
            d_bus_wdata_o(15 downto 00) <= wdata_i(15 downto 0);
            if (addr_i(1) = '0') then
              d_bus_ben_o <= "0011"; -- low half-word
            else
              d_bus_ben_o <= "1100"; -- high half-word
            end if;
          when others => -- word
            d_bus_wdata_o <= wdata_i;
            d_bus_ben_o   <= "1111"; -- full word
        end case;
      end if;
    end if;
  end process mem_do_reg;


  -- Read Data ------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  read_align: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      rdata_o <= (others => def_rst_val_c);
    elsif rising_edge(clk_i) then
      -- input data alignment and sign extension --
      case data_size is
        when "00" => -- byte
          case mar(1 downto 0) is
            when "00" => -- byte 0
              rdata_o(07 downto 00) <= d_bus_rdata_i(07 downto 00);
              rdata_o(31 downto 08) <= (others => (data_sign and d_bus_rdata_i(07))); -- sign extension
            when "01" => -- byte 1
              rdata_o(07 downto 00) <= d_bus_rdata_i(15 downto 08);
              rdata_o(31 downto 08) <= (others => (data_sign and d_bus_rdata_i(15))); -- sign extension
            when "10" => -- byte 2
              rdata_o(07 downto 00) <= d_bus_rdata_i(23 downto 16);
              rdata_o(31 downto 08) <= (others => (data_sign and d_bus_rdata_i(23))); -- sign extension
            when others => -- byte 3
              rdata_o(07 downto 00) <= d_bus_rdata_i(31 downto 24);
              rdata_o(31 downto 08) <= (others => (data_sign and d_bus_rdata_i(31))); -- sign extension
          end case;
        when "01" => -- half-word
          if (mar(1) = '0') then
            rdata_o(15 downto 00) <= d_bus_rdata_i(15 downto 00); -- low half-word
            rdata_o(31 downto 16) <= (others => (data_sign and d_bus_rdata_i(15))); -- sign extension
          else
            rdata_o(15 downto 00) <= d_bus_rdata_i(31 downto 16); -- high half-word
            rdata_o(31 downto 16) <= (others => (data_sign and d_bus_rdata_i(31))); -- sign extension
          end if;
        when others => -- word
          rdata_o <= d_bus_rdata_i; -- full word
      end case;
    end if;
  end process read_align;


  -- Access Arbiter -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  data_access_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      arbiter.busy <= '0';
      arbiter.rw   <= def_rst_val_c;
      arbiter.err  <= def_rst_val_c;
    elsif rising_edge(clk_i) then
      if (arbiter.busy = '0') then -- idle
        arbiter.busy <= ctrl_i(ctrl_bus_wr_c) or ctrl_i(ctrl_bus_rd_c); -- start bus access
        arbiter.rw   <= ctrl_i(ctrl_bus_wr_c); -- set if write access
        arbiter.err  <= '0';
      else -- bus access in progress
        -- accumulate bus errors --
        if (d_bus_err_i = '1') or -- bus error
           ((arbiter.rw = '1') and (st_pmp_fault = '1')) or -- PMP store fault
           ((arbiter.rw = '0') and (ld_pmp_fault = '1')) then -- PMP load fault
          arbiter.err <= '1';
        end if;
        -- do not abort directly when an error has been detected - wait until the trap environment
        -- has started (ctrl_i(ctrl_trap_c)) to make sure the error signals are evaluated BEFORE d_wait_o clears
        if (d_bus_ack_i = '1') or (ctrl_i(ctrl_trap_c) = '1') then
          arbiter.busy <= '0';
        end if;
      end if;
    end if;
  end process data_access_arbiter;

  -- wait for bus transaction to finish --
  d_wait_o <= '1' when (arbiter.busy = '1') and (d_bus_ack_i = '0') else '0';

  -- output data access error to controller --
  ma_load_o  <= '1' when (arbiter.busy = '1') and (arbiter.rw = '0') and (misaligned  = '1') else '0';
  be_load_o  <= '1' when (arbiter.busy = '1') and (arbiter.rw = '0') and (arbiter.err = '1') else '0';
  ma_store_o <= '1' when (arbiter.busy = '1') and (arbiter.rw = '1') and (misaligned  = '1') else '0';
  be_store_o <= '1' when (arbiter.busy = '1') and (arbiter.rw = '1') and (arbiter.err = '1') else '0';

  -- data bus interface (read/write)--
  d_bus_we      <= ctrl_i(ctrl_bus_wr_c) and (not misaligned) and (not st_pmp_fault); -- no actual write when misaligned or PMP fault
  d_bus_re      <= ctrl_i(ctrl_bus_rd_c) and (not misaligned) and (not ld_pmp_fault); -- no actual read when misaligned or PMP fault
  d_bus_fence_o <= ctrl_i(ctrl_bus_fence_c);

  -- additional register stage for control signals if PMP_NUM_REGIONS > pmp_num_regions_critical_c --
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

  d_bus_we_o <= d_bus_we_buf when (PMP_NUM_REGIONS > pmp_num_regions_critical_c) else d_bus_we;
  d_bus_re_o <= d_bus_re_buf when (PMP_NUM_REGIONS > pmp_num_regions_critical_c) else d_bus_re;


  -- Physical Memory Protection (PMP) -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- check address region --
  pmp_check_address: process(pmp_addr_i, fetch_pc_i, mar)
  begin
    for i in 0 to PMP_NUM_REGIONS-1 loop -- iterate over all regions
      if (i = 0) then -- use ZERO as bottom boundary and current entry as top boundary for first entry
        pmp.i_match(i) <= bool_to_ulogic_f(unsigned(fetch_pc_i(data_width_c-1 downto pmp_lsb_c)) < unsigned(pmp_addr_i(0)(data_width_c-1 downto pmp_lsb_c)));
        pmp.d_match(i) <= bool_to_ulogic_f(unsigned(mar(data_width_c-1 downto pmp_lsb_c))        < unsigned(pmp_addr_i(0)(data_width_c-1 downto pmp_lsb_c)));
      else -- use previous entry as bottom boundary and current entry as top boundary
        pmp.i_match(i) <= bool_to_ulogic_f((unsigned(pmp_addr_i(i-1)(data_width_c-1 downto pmp_lsb_c)) <= unsigned(fetch_pc_i(data_width_c-1 downto pmp_lsb_c))) and
                                           (unsigned(fetch_pc_i(data_width_c-1 downto pmp_lsb_c))      <  unsigned(pmp_addr_i(i)(data_width_c-1 downto pmp_lsb_c))));
        pmp.d_match(i) <= bool_to_ulogic_f((unsigned(pmp_addr_i(i-1)(data_width_c-1 downto pmp_lsb_c)) <= unsigned(mar(data_width_c-1 downto pmp_lsb_c))) and
                                           (unsigned(mar(data_width_c-1 downto pmp_lsb_c))             <  unsigned(pmp_addr_i(i)(data_width_c-1 downto pmp_lsb_c))));
      end if;
    end loop; -- i
  end process pmp_check_address;

  -- check access type and permissions --
  pmp_check_permission: process(pmp, pmp_ctrl_i, ctrl_i)
  begin
    pmp.if_fault <= (others => '0');
    pmp.ld_fault <= (others => '0');
    pmp.st_fault <= (others => '0');
    for i in 0 to PMP_NUM_REGIONS-1 loop -- iterate over all regions
      if ((ctrl_i(ctrl_priv_mode_c) = priv_mode_u_c) or (pmp_ctrl_i(i)(pmp_cfg_l_c) = '1')) and -- enforce if USER-mode or LOCKED
         (pmp_ctrl_i(i)(pmp_cfg_ah_c downto pmp_cfg_al_c) = pmp_tor_mode_c) and -- active entry
         (ctrl_i(ctrl_debug_running_c) = '0') then -- disable PMP checks when in debug mode
        pmp.if_fault(i) <= pmp.i_match(i) and (not pmp_ctrl_i(i)(pmp_cfg_x_c)); -- fetch access match no execute permission
        pmp.ld_fault(i) <= pmp.d_match(i) and (not pmp_ctrl_i(i)(pmp_cfg_r_c)); -- load access match no read permission
        pmp.st_fault(i) <= pmp.d_match(i) and (not pmp_ctrl_i(i)(pmp_cfg_w_c)); -- store access match no write permission
      end if;
    end loop; -- i
  end process pmp_check_permission;

  -- final PMP access fault signals --
  if_pmp_fault <= '1' when (or_reduce_f(pmp.if_fault) = '1') and (PMP_NUM_REGIONS > 0) else '0';
  ld_pmp_fault <= '1' when (or_reduce_f(pmp.ld_fault) = '1') and (PMP_NUM_REGIONS > 0) else '0';
  st_pmp_fault <= '1' when (or_reduce_f(pmp.st_fault) = '1') and (PMP_NUM_REGIONS > 0) else '0';

  -- instruction fetch PMP fault --
  i_pmp_fault_o <= if_pmp_fault;


end neorv32_cpu_bus_rtl;
