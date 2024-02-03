-- #################################################################################################
-- # << NEORV32 CPU - Memory Management Unit >>                                                    #
-- # ********************************************************************************************* #
-- # Providing *minimal* memory protection, memory virtualization and memory paging support. The   #
-- # page table entry layout is compatible to the RISC-V "Sv32" specification. The PTE table       #
-- # (translation look-aside buffer, TLB) uses a cache-like direct-mapped architecture.            #
-- # Note that this MMU does NOT support any kind of hardware page walking.                        #
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

entity neorv32_cpu_mmu is
  generic (
    MMU_TLB_SIZE : natural range 2 to 16 -- number of TLB entries, has to be a power of 2
  );
  port (
    -- global control --
    clk_i       : in  std_ulogic; -- global clock, rising edge
    rstn_i      : in  std_ulogic; -- global reset, low-active, async
    -- CSR interface --
    csr_we_i    : in  std_ulogic; -- global write enable
    csr_addr_i  : in  std_ulogic_vector(11 downto 0); -- address
    csr_wdata_i : in  std_ulogic_vector(XLEN-1 downto 0); -- write data
    csr_rdata_o : out std_ulogic_vector(XLEN-1 downto 0); -- read data
    -- bus interfaces --
    ibus_req_i  : in  bus_req_t; -- virtual instruction access request
    dbus_req_i  : in  bus_req_t; -- virtual data access request
    ibus_req_o  : out bus_req_t; -- physical instruction access request
    dbus_req_o  : out bus_req_t; -- physical data access request
    -- page faults --
    fault_i_o   : out std_ulogic; -- instruction page fault
    fault_l_o   : out std_ulogic; -- load page fault
    fault_s_o   : out std_ulogic  -- store page fault
  );
end neorv32_cpu_mmu;

architecture neorv32_cpu_mmu_rtl of neorv32_cpu_mmu is

  -- auto-configuration --
  constant num_ptes_c     : natural := cond_sel_natural_f(is_power_of_two_f(MMU_TLB_SIZE), MMU_TLB_SIZE, 2**index_size_f(MMU_TLB_SIZE));
  constant mmu_vtg_size_c : natural := 20 - index_size_f(num_ptes_c); -- virtual page number tag size
  constant csr_vtg_lsb_c  : natural := 10 + index_size_f(num_ptes_c); -- LSB of virtual page number tag in VPN CSR
  constant adr_vtg_lsb_c  : natural := 12 + index_size_f(num_ptes_c); -- LSB of virtual page number tag in virtual address

  -- table attributes --
  constant att_v_c : natural := 0; -- entry is valid
  constant att_r_c : natural := 1; -- read permission
  constant att_w_c : natural := 2; -- write permission
  constant att_x_c : natural := 3; -- execute permission
  constant att_u_c : natural := 4; -- enable translation in user-mode
  constant att_g_c : natural := 5; -- globally mapped
  constant att_a_c : natural := 6; -- entry has been accessed
  constant att_d_c : natural := 7; -- entry is dirty

  -- translation table --
  type pte_vtg_t is array (0 to num_ptes_c-1) of std_ulogic_vector(mmu_vtg_size_c-1 downto 0);
  type pte_ppn_t is array (0 to num_ptes_c-1) of std_ulogic_vector(19 downto 0);
  type pte_att_t is array (0 to num_ptes_c-1) of std_ulogic_vector(7 downto 0);
  signal pte_vtg : pte_vtg_t; -- PTE virtual page number tag
  signal pte_ppn : pte_ppn_t; -- PTE physical page number
  signal pte_att : pte_att_t; -- PTE attributes/flags

  -- MMU enable --
  signal atp_en : std_ulogic;

  -- MMU CSR interface --
  signal csr_en     : std_ulogic; -- access to MMU CSRs
  signal csr_wdata  : std_ulogic_vector(XLEN-1 downto 0); -- write data buffer
  signal mmu_we     : std_ulogic_vector(1 downto 0); -- MMU table write enable
  signal mmu_index  : std_ulogic_vector(index_size_f(num_ptes_c)-1 downto 0); -- PTE select
  signal mmu_vpn_rd : std_ulogic_vector(XLEN-1 downto 0); -- VPN read-back
  signal mmu_pte_rd : std_ulogic_vector(XLEN-1 downto 0); -- PTE read-back

  -- table lookup --
  type lookup_t is record
    idx : std_ulogic_vector(index_size_f(num_ptes_c)-1 downto 0); -- virtual page number index (PTE select)
    vtg : std_ulogic_vector(mmu_vtg_size_c-1 downto 0); -- virtual page number tag
    ppn : std_ulogic_vector(19 downto 0); -- physical page number
    att : std_ulogic_vector(7 downto 0); -- page attributes / flags
    hit : std_ulogic; -- PTE hit
  end record;
  signal i_lookup, d_lookup : lookup_t;

  -- page faults --
  signal i_fault, l_fault, s_fault : std_ulogic;

  -- bus buffer --
  signal i_ppn_ff, d_ppn_ff : std_ulogic_vector(19 downto 0);
  signal i_stb_ff, d_stb_ff : std_ulogic;

begin

  -- CSR Interface --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- write access --
  csr_write: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      atp_en    <= '0';
      mmu_we    <= (others => '0');
      csr_wdata <= (others => '0');
      mmu_index <= (others => '0');
    elsif rising_edge(clk_i) then
      mmu_we <= (others => '0');
      if (csr_we_i = '1') and (csr_en = '1') then
        csr_wdata <= csr_wdata_i; -- buffer write data for indirect PTE access
        if (csr_addr_i(1 downto 0) = csr_mxmmuatp_c(1 downto 0)) then -- address translation and protection
          atp_en <= csr_wdata_i(31);
        end if;
        if (csr_addr_i(1 downto 0) = csr_mxmmusel_c(1 downto 0)) then -- page table entry select
          mmu_index <= csr_wdata_i(mmu_index'range);
        end if;
        if (csr_addr_i(1 downto 0) = csr_mxmmuvpn_c(1 downto 0)) then -- virtual page number
          mmu_we(0) <= '1';
        end if;
        if (csr_addr_i(1 downto 0) = csr_mxmmupte_c(1 downto 0)) then -- page table entry
          mmu_we(1) <= '1';
        end if;
      end if;
    end if;
  end process csr_write;

  -- read access --
  csr_read: process(csr_en, csr_addr_i, atp_en, mmu_index, mmu_vpn_rd, mmu_pte_rd)
  begin
    csr_rdata_o <= (others => '0'); -- default
    if (csr_en = '1') then
      case csr_addr_i(1 downto 0) is
        when "00" => -- address translation and protection
          csr_rdata_o(31) <= atp_en;
        when "01" => -- virtual page number
          csr_rdata_o(mmu_index'range) <= mmu_index;
        when "10" => -- virtual page number
          csr_rdata_o <= mmu_vpn_rd;
        when "11" => -- page table entry
          csr_rdata_o <= mmu_pte_rd;
        when others => -- undefined
          csr_rdata_o <= (others => '0');
      end case;
    end if;
  end process csr_read;

  -- valid access to MMU CSRs? --
  csr_en <= '1' when (csr_addr_i(11 downto 2) = csr_mxmmuatp_c(11 downto 2)) else '0';


  -- MMU Table (TLB Storage) ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mmu_update: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      pte_vtg <= (others => (others => '0'));
      pte_ppn <= (others => (others => '0'));
      pte_att <= (others => (others => '0'));
    elsif rising_edge(clk_i) then
      -- update virtual page number --
      if (mmu_we(0) = '1') then
        pte_vtg(to_integer(unsigned(mmu_index))) <= csr_wdata(29 downto csr_vtg_lsb_c);
      end if;
      -- update actual TLB entry --
      if (mmu_we(1) = '1') then
        pte_ppn(to_integer(unsigned(mmu_index))) <= csr_wdata(29 downto 10);
        pte_att(to_integer(unsigned(mmu_index))) <= csr_wdata(07 downto 00);
      end if;
    end if;
  end process mmu_update;

  -- read-back --
  mmu_readback: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      mmu_vpn_rd <= (others => '0');
      mmu_pte_rd <= (others => '0');
    elsif rising_edge(clk_i) then
      -- virtual page number --
      mmu_vpn_rd(XLEN-1 downto 30)          <= (others => '0'); -- 32-bit physical address space only
      mmu_vpn_rd(29 downto csr_vtg_lsb_c)   <= pte_vtg(to_integer(unsigned(mmu_index))); -- page number MSBs (tag)
      mmu_vpn_rd(csr_vtg_lsb_c-1 downto 10) <= mmu_index; -- page number LSBs (index)
      mmu_vpn_rd(9 downto 0)                <= (others => '0'); -- unused
      -- page table entry --
      mmu_pte_rd(XLEN-1 downto 30) <= (others => '0'); -- 32-bit physical address space only
      mmu_pte_rd(29 downto 10)     <= pte_ppn(to_integer(unsigned(mmu_index))); -- physical page number
      mmu_pte_rd(9 downto 8)       <= "00"; -- RSW: reserved
      mmu_pte_rd(7 downto 0)       <= pte_att(to_integer(unsigned(mmu_index)));
    end if;
  end process mmu_readback;


  -- TLB Lookup -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- instruction fetch lookup --
  i_lookup.idx <= ibus_req_i.addr(adr_vtg_lsb_c-1 downto 12); -- virtual page number index
  i_lookup.vtg <= ibus_req_i.addr(31 downto adr_vtg_lsb_c); -- virtual page number tag
  i_lookup.ppn <= pte_ppn(to_integer(unsigned(i_lookup.idx))); -- translated physical page number
  i_lookup.att <= pte_att(to_integer(unsigned(i_lookup.idx))); -- PTE attributes
  i_lookup.hit <= '1' when (pte_vtg(to_integer(unsigned(i_lookup.idx))) = i_lookup.vtg) else '0'; -- PTE available?

  -- data access lookup --
  d_lookup.idx <= dbus_req_i.addr(adr_vtg_lsb_c-1 downto 12); -- virtual page number index
  d_lookup.vtg <= dbus_req_i.addr(31 downto adr_vtg_lsb_c); -- virtual page number tag
  d_lookup.ppn <= pte_ppn(to_integer(unsigned(d_lookup.idx))); -- translated physical page number
  d_lookup.att <= pte_att(to_integer(unsigned(d_lookup.idx))); -- PTE attributes
  d_lookup.hit <= '1' when (pte_vtg(to_integer(unsigned(d_lookup.idx))) = d_lookup.vtg) else '0'; -- PTE available?

  -- instruction page fault --
  i_fault <= '1' when (ibus_req_i.priv /= priv_mode_m_c) and -- non-machine-mode access
                      (
                       (i_lookup.hit          = '0') or -- page miss
                       (i_lookup.att(att_v_c) = '0') or -- entry is invalid
                       (i_lookup.att(att_x_c) = '0') or -- no execute permission
                       (i_lookup.att(att_a_c) = '0') or -- page not accessed yet
                       (i_lookup.att(att_u_c) = '0')    -- not allowed in user-mode
                      ) else '0';

  -- load page fault --
  l_fault <= '1' when (dbus_req_i.priv /= priv_mode_m_c) and -- non-machine-mode access
                      (dbus_req_i.rw = '0') and -- is read access
                      (
                       (d_lookup.hit          = '0') or -- page miss
                       (d_lookup.att(att_v_c) = '0') or -- entry is invalid
                       (d_lookup.att(att_r_c) = '0') or -- no read permission
                       (d_lookup.att(att_a_c) = '0') or -- page not accessed yet
                       (d_lookup.att(att_u_c) = '0')    -- not allowed in user-mode
                      ) else '0';

  -- store page fault --
  s_fault <= '1' when (dbus_req_i.priv /= priv_mode_m_c) and -- non-machine-mode access
                      (dbus_req_i.rw = '1') and -- is write access
                      (
                       (d_lookup.hit          = '0') or -- page miss
                       (d_lookup.att(att_v_c) = '0') or -- entry is invalid
                       (d_lookup.att(att_w_c) = '0') or -- no write permission
                       (d_lookup.att(att_d_c) = '0') or -- page not dirty yet
                       (d_lookup.att(att_a_c) = '0') or -- page not accessed yet
                       (d_lookup.att(att_u_c) = '0')    -- not allowed in user-mode
                      ) else '0';


  -- Bus Output -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- register stage --
  output_buffer: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      i_ppn_ff  <= (others => '0');
      i_stb_ff  <= '0';
      d_ppn_ff  <= (others => '0');
      d_stb_ff  <= '0';
      fault_i_o <= '0';
      fault_l_o <= '0';
      fault_s_o <= '0';
    elsif rising_edge(clk_i) then
      i_ppn_ff  <= i_lookup.ppn;
      i_stb_ff  <= ibus_req_i.stb and (not i_fault); -- request only if no page fault
      d_ppn_ff  <= d_lookup.ppn;
      d_stb_ff  <= ibus_req_i.stb and (not l_fault) and (not s_fault); -- request only if no page fault
      fault_i_o <= atp_en and ibus_req_i.stb and i_fault; -- valid instruction page fault
      fault_l_o <= atp_en and dbus_req_i.stb and l_fault; -- valid load page fault
      fault_s_o <= atp_en and dbus_req_i.stb and s_fault; -- valid store page fault
    end if;
  end process output_buffer;

  -- override bus requests --
  bus_output: process(ibus_req_i, dbus_req_i, i_ppn_ff, i_stb_ff, d_ppn_ff, d_stb_ff)
  begin
    -- default: non-translated / pass-through --
    ibus_req_o <= ibus_req_i;
    dbus_req_o <= dbus_req_i;
    if (atp_en = '1') then -- address translation and protection enabled
      -- instruction fetch --
      if (ibus_req_i.priv /= priv_mode_m_c) then -- translate only when not in machine-mode
        ibus_req_o.addr(31 downto 12) <= i_ppn_ff;
        ibus_req_o.stb                <= i_stb_ff;
      end if;
      -- data access --
      if (dbus_req_i.priv /= priv_mode_m_c) then -- translate only when not in machine-mode
        dbus_req_o.addr(31 downto 12) <= d_ppn_ff;
        dbus_req_o.stb                <= d_stb_ff;
      end if;
    end if;
  end process bus_output;


end neorv32_cpu_mmu_rtl;
