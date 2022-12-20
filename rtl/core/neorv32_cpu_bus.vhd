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
    XLEN                : natural; -- data path width
    PMP_NUM_REGIONS     : natural; -- number of regions (0..16)
    PMP_MIN_GRANULARITY : natural  -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
  );
  port (
    -- global control --
    clk_i         : in  std_ulogic; -- global clock, rising edge
    rstn_i        : in  std_ulogic := '0'; -- global reset, low-active, async
    ctrl_i        : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
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

  -- PMP minimal granularity --
  constant pmp_lsb_c : natural := index_size_f(PMP_MIN_GRANULARITY); -- min = 2

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
  type pmp_t is record
    i_cmp_ge : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    i_cmp_lt : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    d_cmp_ge : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    d_cmp_lt : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    i_match  : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    d_match  : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    perm_ex  : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    perm_rd  : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    perm_wr  : std_ulogic_vector(PMP_NUM_REGIONS-1 downto 0);
    if_fault : std_ulogic;
    ld_fault : std_ulogic;
    st_fault : std_ulogic;
  end record;
  signal pmp : pmp_t;

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
      if (ctrl_i(ctrl_bus_mo_we_c) = '1') then
        mar <= addr_i; -- memory address register
        case ctrl_i(ctrl_ir_funct3_1_c downto ctrl_ir_funct3_0_c) is -- alignment check
          when "00"   => misaligned <= '0'; -- byte
          when "01"   => misaligned <= addr_i(0); -- half-word
          when "10"   => misaligned <= addr_i(1) or addr_i(0); -- word
          when others => -- double-word
            if (XLEN = 32) then -- RV32
              misaligned <= '0';
            else -- RV64
              misaligned <= addr_i(2) or addr_i(1) or addr_i(0);
            end if;
        end case;
      end if;
    end if;
  end process mem_adr_reg;

  -- address output --
  d_bus_addr_o <= mar;
  mar_o        <= mar; -- for MTVAL CSR


  -- Write Data: Byte Enable and Alignment --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_do_reg_rv32:
  if (XLEN = 32) generate
    mem_do_reg: process(clk_i)
    begin
      if rising_edge(clk_i) then
        if (ctrl_i(ctrl_bus_mo_we_c) = '1') then
          d_bus_ben_o <= (others => '0'); -- default
          case ctrl_i(ctrl_ir_funct3_1_c downto ctrl_ir_funct3_0_c) is -- data size

            when "00" => -- byte
              for i in 0 to (XLEN/8)-1 loop
                d_bus_wdata_o(i*8+7 downto i*8) <= wdata_i(7 downto 0);
              end loop;
              d_bus_ben_o(to_integer(unsigned(addr_i(1 downto 0)))) <= '1';

            when "01" => -- half-word
              for i in 0 to (XLEN/16)-1 loop
                d_bus_wdata_o(i*16+15 downto i*16) <= wdata_i(15 downto 0);
              end loop;
              if (addr_i(1) = '0') then
                d_bus_ben_o <= "0011"; -- low half-word
              else
                d_bus_ben_o <= "1100"; -- high half-word
              end if;

            when others => -- word
              for i in 0 to (XLEN/32)-1 loop
                d_bus_wdata_o(i*32+31 downto i*32) <= wdata_i(31 downto 0);
              end loop;
              d_bus_ben_o <= (others => '1'); -- full word

          end case;
        end if;
      end if;
    end process mem_do_reg;
  end generate; -- /mem_do_reg_rv32

  mem_do_reg_rv64:
  if (XLEN = 64) generate
    mem_do_reg: process(clk_i)
      variable tmp_v : std_ulogic_vector(1 downto 0);
    begin
      if rising_edge(clk_i) then
        if (ctrl_i(ctrl_bus_mo_we_c) = '1') then
          d_bus_ben_o <= (others => '0'); -- default
          case ctrl_i(ctrl_ir_funct3_1_c downto ctrl_ir_funct3_0_c) is -- data size

            when "00" => -- byte
              for i in 0 to (XLEN/8)-1 loop
                d_bus_wdata_o(i*8+7 downto i*8) <= wdata_i(7 downto 0);
              end loop;
              d_bus_ben_o(to_integer(unsigned(addr_i(2 downto 0)))) <= '1';

            when "01" => -- half-word
              for i in 0 to (XLEN/16)-1 loop
                d_bus_wdata_o(i*16+15 downto i*16) <= wdata_i(15 downto 0);
              end loop;
              tmp_v := addr_i(1 downto 0);
              case tmp_v is
                when "00"   => d_bus_ben_o <= "00000011"; -- half-word 0
                when "01"   => d_bus_ben_o <= "00001100"; -- half-word 1
                when "10"   => d_bus_ben_o <= "00110000"; -- half-word 2
                when others => d_bus_ben_o <= "11000000"; -- half-word 3
              end case;

            when "10" => -- word
              for i in 0 to (XLEN/32)-1 loop
                d_bus_wdata_o(i*32+31 downto i*32) <= wdata_i(31 downto 0);
              end loop;
              if (addr_i(2) = '0') then
                d_bus_ben_o <= "00001111"; -- word 0
              else
                d_bus_ben_o <= "11110000"; -- word 1
              end if;

            when others => -- double-word
              for i in 0 to (XLEN/64)-1 loop
                d_bus_wdata_o(i*64+63 downto i*64) <= wdata_i(63 downto 0);
              end loop;
              d_bus_ben_o <= (others => '1'); -- full double-word

          end case;
        end if;
      end if;
    end process mem_do_reg;
  end generate; -- /mem_do_reg_rv64


  -- Read Data: Alignment and Sign-Extension ------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_di_reg_rv32:
  if (XLEN = 32) generate
    mem_di_reg: process(clk_i)
      variable tmp_v : std_ulogic_vector(1 downto 0);
    begin
      if rising_edge(clk_i) then
        case ctrl_i(ctrl_ir_funct3_1_c downto ctrl_ir_funct3_0_c) is

          when "00" => -- byte
            tmp_v := mar(1 downto 0);
            case tmp_v is
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
  end generate; -- /mem_di_reg_rv32

  mem_di_reg_rv64:
  if (XLEN = 64) generate
    mem_di_reg: process(clk_i)
      variable tmp3_v : std_ulogic_vector(2 downto 0);
      variable tmp2_v : std_ulogic_vector(1 downto 0);
    begin
      if rising_edge(clk_i) then
        case ctrl_i(ctrl_ir_funct3_1_c downto ctrl_ir_funct3_0_c) is

          when "00" => -- byte
            tmp3_v := mar(2 downto 0);
            case tmp3_v is
              when "000" => -- byte 0
                rdata_o(7 downto 0) <= d_bus_rdata_i(07 downto 00);
                rdata_o(XLEN-1 downto 8) <= (others => (data_sign and d_bus_rdata_i(07))); -- sign extension
              when "001" => -- byte 1
                rdata_o(7 downto 0) <= d_bus_rdata_i(15 downto 08);
                rdata_o(XLEN-1 downto 8) <= (others => (data_sign and d_bus_rdata_i(15))); -- sign extension
              when "010" => -- byte 2
                rdata_o(7 downto 0) <= d_bus_rdata_i(23 downto 16);
                rdata_o(XLEN-1 downto 8) <= (others => (data_sign and d_bus_rdata_i(23))); -- sign extension
              when "011" => -- byte 3
                rdata_o(7 downto 0) <= d_bus_rdata_i(31 downto 24);
                rdata_o(XLEN-1 downto 8) <= (others => (data_sign and d_bus_rdata_i(31))); -- sign extension
              when "100" => -- byte 4
                rdata_o(7 downto 0) <= d_bus_rdata_i(39 downto 32);
                rdata_o(XLEN-1 downto 8) <= (others => (data_sign and d_bus_rdata_i(39))); -- sign extension
              when "101" => -- byte 5
                rdata_o(7 downto 0) <= d_bus_rdata_i(47 downto 40);
                rdata_o(XLEN-1 downto 8) <= (others => (data_sign and d_bus_rdata_i(47))); -- sign extension
              when "110" => -- byte 6
                rdata_o(7 downto 0) <= d_bus_rdata_i(55 downto 48);
                rdata_o(XLEN-1 downto 8) <= (others => (data_sign and d_bus_rdata_i(55))); -- sign extension
              when others => -- byte 7
                rdata_o(7 downto 0) <= d_bus_rdata_i(63 downto 56);
                rdata_o(XLEN-1 downto 8) <= (others => (data_sign and d_bus_rdata_i(63))); -- sign extension
            end case;

          when "01" => -- half-word
            tmp2_v := mar(1 downto 0);
            case tmp2_v is
              when "00" => -- half word 0
                rdata_o(15 downto 0) <= d_bus_rdata_i(15 downto 00); -- low half-word
                rdata_o(XLEN-1 downto 16) <= (others => (data_sign and d_bus_rdata_i(15))); -- sign extension
              when "01" => -- half word 1
                rdata_o(15 downto 0) <= d_bus_rdata_i(31 downto 16); -- low half-word
                rdata_o(XLEN-1 downto 16) <= (others => (data_sign and d_bus_rdata_i(31))); -- sign extension
              when "10" => -- half word 2
                rdata_o(15 downto 0) <= d_bus_rdata_i(47 downto 32); -- low half-word
                rdata_o(XLEN-1 downto 16) <= (others => (data_sign and d_bus_rdata_i(47))); -- sign extension
              when others => -- half word 3
                rdata_o(15 downto 0) <= d_bus_rdata_i(63 downto 48); -- low half-word
                rdata_o(XLEN-1 downto 16) <= (others => (data_sign and d_bus_rdata_i(63))); -- sign extension
            end case;

          when "10" => -- word
            if (mar(2) = '0') then
              rdata_o(31 downto 0) <= d_bus_rdata_i(31 downto 00); -- low word
              rdata_o(XLEN-1 downto 16) <= (others => (data_sign and d_bus_rdata_i(31))); -- sign extension
            else
              rdata_o(31 downto 0) <= d_bus_rdata_i(63 downto 32); -- high word
              rdata_o(XLEN-1 downto 16) <= (others => (data_sign and d_bus_rdata_i(63))); -- sign extension
            end if;

          when others => -- double-word
            rdata_o(XLEN-1 downto 0) <= d_bus_rdata_i(XLEN-1 downto 0); -- full double word

        end case;
      end if;
    end process mem_di_reg;
  end generate; -- /mem_di_reg_rv64

  -- sign extension --
  data_sign <= not ctrl_i(ctrl_ir_funct3_2_c); -- NOT unsigned LOAD (LBU, LHU)


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
      arbiter.pmp_r_err <= ld_pmp_fault;
      arbiter.pmp_w_err <= st_pmp_fault;
      if (arbiter.pend = '0') then -- idle
        if (ctrl_i(ctrl_bus_req_c) = '1') then -- start bus access
          arbiter.pend <= '1';
        end if;
        arbiter.err <= '0';
      else -- bus access in progress
        -- accumulate bus errors --
        if (d_bus_err_i = '1') or -- bus error
           ((ctrl_i(ctrl_ir_opcode7_5_c) = '1') and (arbiter.pmp_w_err = '1')) or -- PMP store fault
           ((ctrl_i(ctrl_ir_opcode7_5_c) = '0') and (arbiter.pmp_r_err = '1')) then -- PMP load fault
          arbiter.err <= '1';
        end if;
        -- wait for normal termination or start of trap handling --
        if (d_bus_ack_i = '1') or (ctrl_i(ctrl_trap_c) = '1') then
          arbiter.pend <= '0';
        end if;
      end if;
    end if;
  end process data_access_arbiter;

  -- wait for bus response --
  d_wait_o <= not d_bus_ack_i;

  -- output data access error to controller --
  ma_load_o  <= '1' when (arbiter.pend = '1') and (ctrl_i(ctrl_ir_opcode7_5_c) = '0') and (misaligned  = '1') else '0';
  be_load_o  <= '1' when (arbiter.pend = '1') and (ctrl_i(ctrl_ir_opcode7_5_c) = '0') and (arbiter.err = '1') else '0';
  ma_store_o <= '1' when (arbiter.pend = '1') and (ctrl_i(ctrl_ir_opcode7_5_c) = '1') and (misaligned  = '1') else '0';
  be_store_o <= '1' when (arbiter.pend = '1') and (ctrl_i(ctrl_ir_opcode7_5_c) = '1') and (arbiter.err = '1') else '0';

  -- data bus control interface (all source signals are driven by registers) --
  d_bus_we_o    <= ctrl_i(ctrl_bus_req_c) and (    ctrl_i(ctrl_ir_opcode7_5_c)) and (not misaligned) and (not arbiter.pmp_w_err);
  d_bus_re_o    <= ctrl_i(ctrl_bus_req_c) and (not ctrl_i(ctrl_ir_opcode7_5_c)) and (not misaligned) and (not arbiter.pmp_r_err);
  d_bus_fence_o <= ctrl_i(ctrl_bus_fence_c);
  d_bus_priv_o  <= ctrl_i(ctrl_bus_priv_c);


  -- RISC-V Physical Memory Protection (PMP) ------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- check address --
  pmp_check_address: process(fetch_pc_i, addr_i, pmp_addr_i)
  begin
    for r in 0 to PMP_NUM_REGIONS-1 loop
      if (r = 0) then -- first entry: use ZERO as base and current entry as bound
        pmp.i_cmp_ge(r) <= '1'; -- address is always greater than or equal to zero
        pmp.i_cmp_lt(r) <= '0'; -- unused
        pmp.d_cmp_ge(r) <= '1'; -- address is always greater than or equal to zero
        pmp.d_cmp_lt(r) <= '0'; -- unused
      else -- use previous entry as base and current entry as bound
        pmp.i_cmp_ge(r) <= bool_to_ulogic_f(unsigned(fetch_pc_i(XLEN-1 downto pmp_lsb_c)) >= unsigned(pmp_addr_i(r-1)(XLEN-1 downto pmp_lsb_c)));
        pmp.i_cmp_lt(r) <= bool_to_ulogic_f(unsigned(fetch_pc_i(XLEN-1 downto pmp_lsb_c)) <  unsigned(pmp_addr_i(r-0)(XLEN-1 downto pmp_lsb_c)));
        pmp.d_cmp_ge(r) <= bool_to_ulogic_f(unsigned(    addr_i(XLEN-1 downto pmp_lsb_c)) >= unsigned(pmp_addr_i(r-1)(XLEN-1 downto pmp_lsb_c)));
        pmp.d_cmp_lt(r) <= bool_to_ulogic_f(unsigned(    addr_i(XLEN-1 downto pmp_lsb_c)) <  unsigned(pmp_addr_i(r-0)(XLEN-1 downto pmp_lsb_c)));
      end if;
    end loop; -- r
  end process pmp_check_address;


  -- check mode --
  pmp_check_mode: process(pmp_ctrl_i, pmp)
  begin
    for r in 0 to PMP_NUM_REGIONS-1 loop
      if (pmp_ctrl_i(r)(pmp_cfg_ah_c downto pmp_cfg_al_c) = pmp_mode_tor_c) then -- TOR mode
        if (r < (PMP_NUM_REGIONS-1)) then
          -- this saves a LOT of comparators --
          pmp.i_match(r) <= pmp.i_cmp_ge(r) and (not pmp.i_cmp_ge(r+1));
          pmp.d_match(r) <= pmp.d_cmp_ge(r) and (not pmp.d_cmp_ge(r+1));
        else -- very last entry
          pmp.i_match(r) <= pmp.i_cmp_ge(r) and pmp.i_cmp_lt(r);
          pmp.d_match(r) <= pmp.d_cmp_ge(r) and pmp.d_cmp_lt(r);
        end if;
      else -- entry disabled
        pmp.i_match(r) <= '0';
        pmp.d_match(r) <= '0';
      end if;
    end loop; -- r
  end process pmp_check_mode;


  -- check permission --
  pmp_check_permission: process(ctrl_i, pmp_ctrl_i)
  begin
    for r in 0 to PMP_NUM_REGIONS-1 loop

      -- instruction fetch access --
      if (ctrl_i(ctrl_priv_mode_c) = priv_mode_m_c) then -- M mode: always allow if lock bit not set, otherwise check permission
        pmp.perm_ex(r) <= (not pmp_ctrl_i(r)(pmp_cfg_l_c)) or pmp_ctrl_i(r)(pmp_cfg_x_c);
      else -- U mode: always check permission
        pmp.perm_ex(r) <= pmp_ctrl_i(r)(pmp_cfg_x_c);
      end if;
 
      -- load/store accesses from M mod (can also use U mode's permissions if MSTATUS.MPRV is set) --
      if (ctrl_i(ctrl_bus_priv_c) = priv_mode_m_c) then -- M mode: always allow if lock bit not set, otherwise check permission
        pmp.perm_rd(r) <= (not pmp_ctrl_i(r)(pmp_cfg_l_c)) or pmp_ctrl_i(r)(pmp_cfg_r_c);
        pmp.perm_wr(r) <= (not pmp_ctrl_i(r)(pmp_cfg_l_c)) or pmp_ctrl_i(r)(pmp_cfg_w_c);
      else -- U mode: always check permission
        pmp.perm_rd(r) <= pmp_ctrl_i(r)(pmp_cfg_r_c);
        pmp.perm_wr(r) <= pmp_ctrl_i(r)(pmp_cfg_w_c);
      end if;
 
    end loop; -- r
  end process pmp_check_permission;


  -- check for access fault (using static prioritization) --
  pmp_check_fault: process(ctrl_i, pmp)
    variable tmp_if_v, tmp_ld_v, tmp_st_v : std_ulogic_vector(PMP_NUM_REGIONS downto 0);
  begin
    -- > This is a *structural* description of a prioritization logic (a multiplexer chain).
    -- > I prefer this style as I do not like using a loop with 'exit' - and I also think this style might be smaller
    -- > and faster (could use the carry chain?!) as the synthesizer has less freedom doing what *I* want. ;)
    tmp_if_v(PMP_NUM_REGIONS) := bool_to_ulogic_f(ctrl_i(ctrl_priv_mode_c) /= priv_mode_m_c); -- default: fault if U mode
    tmp_ld_v(PMP_NUM_REGIONS) := bool_to_ulogic_f(ctrl_i(ctrl_bus_priv_c)  /= priv_mode_m_c); -- default: fault if U mode
    tmp_st_v(PMP_NUM_REGIONS) := bool_to_ulogic_f(ctrl_i(ctrl_bus_priv_c)  /= priv_mode_m_c); -- default: fault if U mode
 
    for r in PMP_NUM_REGIONS-1 downto 0 loop -- start with lowest priority
      -- instruction fetch access --
      if (pmp.i_match(r) = '1') then -- address matches region r
        tmp_if_v(r) := not pmp.perm_ex(r); -- fault if no execute permission
      else
        tmp_if_v(r) := tmp_if_v(r+1);
      end if;
      -- data load/store access --
      if (pmp.d_match(r) = '1') then -- address matches region r
        tmp_ld_v(r) := not pmp.perm_rd(r); -- fault if no read permission
        tmp_st_v(r) := not pmp.perm_wr(r); -- fault if no write permission
      else
        tmp_ld_v(r) := tmp_ld_v(r+1);
        tmp_st_v(r) := tmp_st_v(r+1);
      end if;
    end loop; -- r
    pmp.if_fault <= tmp_if_v(0);
    pmp.ld_fault <= tmp_ld_v(0);
    pmp.st_fault <= tmp_st_v(0);

    -- > this is the behavioral version of the code above (instruction fetch access)
--  pmp.if_fault <= bool_to_ulogic_f(ctrl_i(ctrl_priv_mode_c) /= priv_mode_m_c); -- default: fault if U mode
--  for r in 0 to PMP_NUM_REGIONS-1 loop
--    if (pmp.i_match(r) = '1') then
--      pmp.if_fault <= not pmp.perm_ex(r); -- fault if no execute permission
--      exit;
--    end if;
--  end loop; -- r
  end process pmp_check_fault;

  -- final PMP access fault signals (ignored when in debug mode) --
  if_pmp_fault <= '1' when (pmp.if_fault = '1') and (PMP_NUM_REGIONS > 0) and (ctrl_i(ctrl_debug_running_c) = '0') else '0';
  ld_pmp_fault <= '1' when (pmp.ld_fault = '1') and (PMP_NUM_REGIONS > 0) and (ctrl_i(ctrl_debug_running_c) = '0') else '0';
  st_pmp_fault <= '1' when (pmp.st_fault = '1') and (PMP_NUM_REGIONS > 0) and (ctrl_i(ctrl_debug_running_c) = '0') else '0';

  -- instruction fetch PMP fault --
  i_pmp_fault_o <= if_pmp_fault;


end neorv32_cpu_bus_rtl;
