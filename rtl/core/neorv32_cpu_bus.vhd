-- #################################################################################################
-- # << NEORV32 - Bus Interface Unit >>                                                            #
-- # ********************************************************************************************* #
-- # This unit connects the CPU to the memory/IO system.                                           #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2020, Stephan Nolting. All rights reserved.                                     #
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
    CPU_EXTENSION_RISCV_C : boolean := true; -- implement compressed extension?
    MEM_EXT_TIMEOUT       : natural := 15 -- cycles after which a valid bus access will timeout
  );
  port (
    -- global control --
    clk_i        : in  std_ulogic; -- global clock, rising edge
    rstn_i       : in  std_ulogic; -- global reset, low-active, async
    ctrl_i       : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
    -- data input --
    wdata_i      : in  std_ulogic_vector(data_width_c-1 downto 0); -- write data
    pc_i         : in  std_ulogic_vector(data_width_c-1 downto 0); -- current PC
    alu_i        : in  std_ulogic_vector(data_width_c-1 downto 0); -- ALU result
    -- data output --
    instr_o      : out std_ulogic_vector(data_width_c-1 downto 0); -- instruction
    rdata_o      : out std_ulogic_vector(data_width_c-1 downto 0); -- read data
    -- status --
    mar_o        : out std_ulogic_vector(data_width_c-1 downto 0); -- current memory address register
    ma_instr_o   : out std_ulogic; -- misaligned instruction address
    ma_load_o    : out std_ulogic; -- misaligned load data address
    ma_store_o   : out std_ulogic; -- misaligned store data address
    be_instr_o   : out std_ulogic; -- bus error on instruction access
    be_load_o    : out std_ulogic; -- bus error on load data access
    be_store_o   : out std_ulogic; -- bus error on store data access
    bus_wait_o   : out std_ulogic; -- wait for bus operation to finish
    bus_busy_o   : out std_ulogic; -- bus unit is busy
    -- bus system --
    bus_addr_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
    bus_rdata_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
    bus_wdata_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
    bus_ben_o    : out std_ulogic_vector(03 downto 0); -- byte enable
    bus_we_o     : out std_ulogic; -- write enable
    bus_re_o     : out std_ulogic; -- read enable
    bus_cancel_o : out std_ulogic; -- cancel current bus transaction
    bus_ack_i    : in  std_ulogic; -- bus transfer acknowledge
    bus_err_i    : in  std_ulogic  -- bus transfer error
  );
end neorv32_cpu_bus;

architecture neorv32_cpu_bus_rtl of neorv32_cpu_bus is

  -- interface registers --
  signal mar, mdo, mdi : std_ulogic_vector(data_width_c-1 downto 0);

  -- bus request controller --
  signal bus_busy    : std_ulogic;
  signal bus_if_req  : std_ulogic;
  signal bus_rd_req  : std_ulogic;
  signal bus_wr_req  : std_ulogic;
  signal access_err  : std_ulogic;
  signal align_err   : std_ulogic;
  signal bus_timeout : std_ulogic_vector(index_size_f(MEM_EXT_TIMEOUT)-1 downto 0);

  -- misaligned access? --
  signal misaligned_data, misaligned_instr : std_ulogic;

begin

  -- Address and Control --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_adr_reg: process(rstn_i, clk_i)
  begin
    if rising_edge(clk_i) then
      if (ctrl_i(ctrl_bus_mar_we_c) = '1') then
        mar <= alu_i;
      end if;
    end if;
  end process mem_adr_reg;

  -- address output --
  bus_addr_o <= pc_i when ((bus_if_req or ctrl_i(ctrl_bus_if_c)) = '1') else mar; -- is instruction fetch? keep output at PC as long as IF request is active
  mar_o      <= mar;

  -- write request output --
  bus_we_o <= ctrl_i(ctrl_bus_wr_c) and (not misaligned_data);

  -- read request output (also used for instruction fetch) --
  bus_re_o <= (ctrl_i(ctrl_bus_rd_c) and (not misaligned_data)) or (ctrl_i(ctrl_bus_if_c) and (not misaligned_instr)); -- FIXME i_reg and misaligned


  -- Write Data -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_do_reg: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (ctrl_i(ctrl_bus_mdo_we_c) = '1') then
        mdo <= wdata_i;
      end if;
    end if;
  end process mem_do_reg;

  -- byte enable and output data alignment --
  byte_enable: process(mar, mdo, ctrl_i)
  begin
    case ctrl_i(ctrl_bus_size_msb_c downto ctrl_bus_size_lsb_c) is -- data size
      when "00" => -- byte
        bus_wdata_o(07 downto 00) <= mdo(07 downto 00);
        bus_wdata_o(15 downto 08) <= mdo(07 downto 00);
        bus_wdata_o(23 downto 16) <= mdo(07 downto 00);
        bus_wdata_o(31 downto 24) <= mdo(07 downto 00);
        bus_ben_o <= (others => '0');
        bus_ben_o(to_integer(unsigned(mar(1 downto 0)))) <= '1';
      when "01" => -- half-word
        bus_wdata_o(31 downto 16) <= mdo(15 downto 00);
        bus_wdata_o(15 downto 00) <= mdo(15 downto 00);
        if (mar(1) = '0') then
          bus_ben_o <= "0011"; -- low half-word
        else
          bus_ben_o <= "1100"; -- high half-word
        end if;
      when others => -- word
        bus_wdata_o <= mdo;
        bus_ben_o <= "1111"; -- full word
    end case;
  end process byte_enable;


  -- Read Data ------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_out_buf: process(clk_i)
  begin
    if rising_edge(clk_i) then
      -- memory data in register (MDI) --
      if (ctrl_i(ctrl_bus_mdi_we_c) = '1') then
        mdi <= bus_rdata_i;
      end if;
    end if;
  end process mem_out_buf;

  -- instruction output --
  instr_o <= bus_rdata_i;

  -- input data align and sign extension --
  read_align: process(mdi, mar, ctrl_i)
    variable signed_v : std_ulogic;
  begin
    signed_v := not ctrl_i(ctrl_bus_unsigned_c);
    case ctrl_i(ctrl_bus_size_msb_c downto ctrl_bus_size_lsb_c) is -- data size
      when "00" => -- byte
        case mar(1 downto 0) is
          when "00" =>
            rdata_o(31 downto 08) <= (others => (signed_v and mdi(07)));
            rdata_o(07 downto 00) <= mdi(07 downto 00); -- byte 0
          when "01" =>
            rdata_o(31 downto 08) <= (others => (signed_v and mdi(15)));
            rdata_o(07 downto 00) <= mdi(15 downto 08); -- byte 1
          when "10" =>
            rdata_o(31 downto 08) <= (others => (signed_v and mdi(23)));
            rdata_o(07 downto 00) <= mdi(23 downto 16); -- byte 2
          when others =>
            rdata_o(31 downto 08) <= (others => (signed_v and mdi(31)));
            rdata_o(07 downto 00) <= mdi(31 downto 24); -- byte 3
        end case;
      when "01" => -- half-word
        if (mar(1) = '0') then
          rdata_o(31 downto 16) <= (others => (signed_v and mdi(15)));
          rdata_o(15 downto 00) <= mdi(15 downto 00); -- low half-word
        else
          rdata_o(31 downto 16) <= (others => (signed_v and mdi(31)));
          rdata_o(15 downto 00) <= mdi(31 downto 16); -- high half-word
        end if;
      when others => -- word
        rdata_o <= mdi; -- full word
    end case;
  end process read_align;


  -- Bus Status Controller ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_ctrl: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bus_busy    <= '0';
      bus_if_req  <= '0';
      bus_rd_req  <= '0';
      bus_wr_req  <= '0';
      access_err  <= '0';
      align_err   <= '0';
      bus_timeout <= (others => '0');
    elsif rising_edge(clk_i) then
      if (bus_busy = '0') or (ctrl_i(ctrl_bus_reset_c) = '1') then -- wait for new request or reset
        bus_busy      <= ctrl_i(ctrl_bus_if_c) or ctrl_i(ctrl_bus_rd_c) or ctrl_i(ctrl_bus_wr_c); -- any request at all?
        bus_if_req    <= ctrl_i(ctrl_bus_if_c); -- instruction fetch
        bus_rd_req    <= ctrl_i(ctrl_bus_rd_c); -- store access
        bus_wr_req    <= ctrl_i(ctrl_bus_wr_c); -- load access
        bus_timeout   <= std_ulogic_vector(to_unsigned(MEM_EXT_TIMEOUT, index_size_f(MEM_EXT_TIMEOUT)));
        access_err    <= '0';
        align_err     <= '0';
      else -- bus transfer in progress
        bus_timeout <= std_ulogic_vector(unsigned(bus_timeout) - 1);
        align_err   <= (align_err or misaligned_data or misaligned_instr) and (not ctrl_i(ctrl_bus_exc_ack_c));
        access_err  <= (access_err or (not or_all_f(bus_timeout)) or bus_err_i) and (not ctrl_i(ctrl_bus_exc_ack_c));
        if (align_err = '1') or (access_err = '1') then
          if (ctrl_i(ctrl_bus_exc_ack_c) = '1') then -- wait for controller to ack exception
            bus_if_req <= '0';
            bus_rd_req <= '0';
            bus_wr_req <= '0';
            bus_busy   <= '0';
          end if;
        elsif (bus_ack_i = '1') then -- normal termination
          bus_if_req <= '0';
          bus_rd_req <= '0';
          bus_wr_req <= '0';
          bus_busy   <= '0';
        end if;
      end if;
    end if;
  end process bus_ctrl;

  -- output bus access error to controller --
  be_instr_o <= bus_if_req and access_err;
  be_load_o  <= bus_rd_req and access_err;
  be_store_o <= bus_wr_req and access_err;

  -- output alignment error to controller --
  ma_instr_o <= bus_if_req and align_err;
  ma_load_o  <= bus_rd_req and align_err;
  ma_store_o <= bus_wr_req and align_err;

  -- terminate bus access --
  bus_cancel_o <= (bus_busy and (align_err or access_err)) or ctrl_i(ctrl_bus_reset_c);

  -- wait for bus --
  bus_busy_o <= bus_busy;
  bus_wait_o <= bus_busy and (not bus_ack_i); -- FIXME: 'async' ack


  -- Check for Misaligned Access ------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  misaligned_d_check: process(mar, ctrl_i)
  begin
    -- check data access --
    misaligned_data <= '0'; -- default
    case ctrl_i(ctrl_bus_size_msb_c downto ctrl_bus_size_lsb_c) is -- data size
      when "00" => -- byte
        misaligned_data <= '0';
      when "01" => -- half-word
        if (mar(0) /= '0') then
          misaligned_data <= '1';
        end if;
      when others => -- word
        if (mar(1 downto 0) /= "00") then
          misaligned_data <= '1';
        end if;
    end case;
  end process misaligned_d_check;

  misaligned_i_check: process(ctrl_i, pc_i)
  begin
    -- check instruction access --
    misaligned_instr <= '0'; -- default
    if (CPU_EXTENSION_RISCV_C = true) then -- 16-bit and 32-bit instruction accesses
      misaligned_instr <= '0'; -- no alignment exceptions possible
    else -- 32-bit instruction accesses only
      if (pc_i(1) = '1') then -- PC(0) is always zero
        misaligned_instr <= '1';
      end if; 
    end if;
  end process misaligned_i_check;


end neorv32_cpu_bus_rtl;
