-- ================================================================================ --
-- NEORV32 CPU - Load/Store Unit: Unaligned Memory Access Handler                   --
-- -------------------------------------------------------------------------------- --
-- Sits between the LSU and the data bus. Translates a potentially-misaligned       --
-- bus_req_t into one or two word-aligned sub-transactions, then reassembles the    --
-- responses. Aligned and AMO accesses are passed through as a single transaction.  --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cpu_lsu_unaligned is
  port (
    -- global control --
    clk_i       : in  std_ulogic; -- global clock, rising edge
    rstn_i      : in  std_ulogic; -- global reset, low-active, async
    -- CPU control bus --
    ctrl_i      : in  ctrl_bus_t;
    -- PMP access fault --
    pmp_fault_i : in  std_ulogic;
    -- raw access operands (combinational, valid when ctrl_i.lsu_mo_en = '1') --
    addr_i      : in  std_ulogic_vector(31 downto 0); -- access address
    wdata_i     : in  std_ulogic_vector(31 downto 0); -- write data
    -- LSU interface --
    req_i       : in  bus_req_t; -- request (possibly unaligned)
    rsp_o       : out bus_rsp_t; -- response
    -- BUS interface --
    req_o       : out bus_req_t; -- request (aligned)
    rsp_i       : in  bus_rsp_t  -- response
  );
end neorv32_cpu_lsu_unaligned;

architecture neorv32_cpu_lsu_unaligned_rtl of neorv32_cpu_lsu_unaligned is

  -- registered access parameters --
  signal acc_addr   : std_ulogic_vector(31 downto 0); -- original (possibly unaligned) address
  signal acc_funct3 : std_ulogic_vector(2 downto 0);  -- access type and sign control

  -- sub-access parameters --
  signal r1_stb  : std_ulogic;
  signal r1_addr : std_ulogic_vector(31 downto 0);
  signal r1_data : std_ulogic_vector(31 downto 0);
  signal r1_ben  : std_ulogic_vector(3 downto 0);
  signal r2_stb  : std_ulogic;
  signal r2_data : std_ulogic_vector(31 downto 0);
  signal r2_ben  : std_ulogic_vector(3 downto 0);

  -- access classification --
  type state_t is (S_IDLE, S_UNALIGNED_W, S_UNALIGNED_HW);
  signal state      : state_t;
  signal req2_split : boolean; -- true when access straddles a word boundary

  -- split-transaction phase --
  type phase_t is (REQ1, REQ2);
  signal req_phase : phase_t;
  signal rsp1_data : std_ulogic_vector(31 downto 0); -- captured first sub-access response

begin

  -- Split Parameter Computation -------------------------------------------------------------
  -- Computes word-aligned sub-access parameters from the raw (possibly unaligned) address and write data.
  -- Aligned and AMO accesses fall into S_IDLE (single transaction).
  -- Misaligned non-AMO half-words and words use S_UNALIGNED_HW and S_UNALIGNED_W respectively.
  -- -----------------------------------------------------------------------------------------
  param_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      acc_addr   <= (others => '0');
      acc_funct3 <= (others => '0');
      r1_addr    <= (others => '0');
      r1_data    <= (others => '0');
      r1_ben     <= (others => '0');
      r2_data    <= (others => '0');
      r2_ben     <= (others => '0');
      state      <= S_IDLE;
    elsif rising_edge(clk_i) then
      if (ctrl_i.lsu_mo_en = '1') then
        acc_addr   <= addr_i;
        acc_funct3 <= ctrl_i.ir_funct3;
        r1_addr    <= addr_i(31 downto 2) & "00"; -- align down to word boundary
        r2_data    <= (others => '0');
        r2_ben     <= (others => '0');

        case ctrl_i.ir_funct3(1 downto 0) is
          when "00" => -- byte: always naturally aligned, single transaction
            state      <= S_IDLE;
            r1_data    <= wdata_i(7 downto 0) & wdata_i(7 downto 0) & wdata_i(7 downto 0) & wdata_i(7 downto 0);
            r1_ben(0)  <= (not addr_i(1)) and (not addr_i(0));
            r1_ben(1)  <= (not addr_i(1)) and (    addr_i(0));
            r1_ben(2)  <= (    addr_i(1)) and (not addr_i(0));
            r1_ben(3)  <= (    addr_i(1)) and (    addr_i(0));

          when "01" => -- half-word
            if (addr_i(0) = '1') and (ctrl_i.ir_opcode(2) = '0') then -- misaligned, non-AMO
              state <= S_UNALIGNED_HW;
              if (addr_i(1) = '0') then
                -- offset "01": both bytes lie within the same aligned word
                r1_ben  <= "0110";
                r1_data <= x"00" & wdata_i(15 downto 0) & x"00";
                -- no second access
              else
                -- offset "11": straddles word boundary
                r1_ben  <= "1000";
                r1_data <= wdata_i(7 downto 0) & wdata_i(7 downto 0) & wdata_i(7 downto 0) & wdata_i(7 downto 0);
                r2_ben  <= "0001";
                r2_data <= wdata_i(15 downto 8) & wdata_i(15 downto 8) & wdata_i(15 downto 8) & wdata_i(15 downto 8);
              end if;
            else -- aligned half-word or AMO: single transaction
              state   <= S_IDLE;
              r1_ben  <= addr_i(1) & addr_i(1) & (not addr_i(1)) & (not addr_i(1));
              r1_data <= wdata_i(15 downto 0) & wdata_i(15 downto 0);
            end if;

          when others => -- word
            if ((addr_i(1) or addr_i(0)) = '1') and (ctrl_i.ir_opcode(2) = '0') then -- misaligned, non-AMO
              state <= S_UNALIGNED_W;
              case addr_i(1 downto 0) is
                when "01" => -- 3 bytes in word1 (lanes 1,2,3), 1 byte in word2 (lane 0)
                  r1_ben  <= "1110"; r1_data <= wdata_i(23 downto 0) & x"00";
                  r2_ben  <= "0001"; r2_data <= wdata_i(31 downto 24) & wdata_i(31 downto 24) & wdata_i(31 downto 24) & wdata_i(31 downto 24);
                when "10" => -- 2 bytes in word1 (lanes 2,3), 2 bytes in word2 (lanes 0,1)
                  r1_ben  <= "1100"; r1_data <= wdata_i(15 downto 0) & x"0000";
                  r2_ben  <= "0011"; r2_data <= wdata_i(31 downto 16) & wdata_i(31 downto 16);
                when others => -- "11": 1 byte in word1 (lane 3), 3 bytes in word2 (lanes 0,1,2)
                  r1_ben  <= "1000"; r1_data <= wdata_i(7 downto 0) & wdata_i(7 downto 0) & wdata_i(7 downto 0) & wdata_i(7 downto 0);
                  r2_ben  <= "0111"; r2_data <= x"00" & wdata_i(31 downto 8);
              end case;
            else -- aligned word or AMO: single transaction
              state   <= S_IDLE;
              r1_ben  <= (others => '1');
              r1_data <= wdata_i;
            end if;

        end case;
      end if;
    end if;
  end process param_reg;

  -- forward request unless it's a misaligned AMO transaction
  r1_stb <= ctrl_i.lsu_req and (not (req_i.amo and (acc_addr(1) or acc_addr(0))));
  -- second word required when the access straddles a word boundary --
  req2_split <= (state = S_UNALIGNED_W) or (state = S_UNALIGNED_HW and acc_addr(1) = '1');

  unaligned_fsm: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      r2_stb    <= '0';
      req_phase <= REQ1;
      rsp1_data <= (others => '0');
    elsif rising_edge(clk_i) then
      r2_stb <= '0';
      if (ctrl_i.cpu_trap = '1') then
        req_phase <= REQ1; -- abort any in-progress split on exception
      elsif req2_split then
        -- only issue the second request if there is no error
        if (req_phase = REQ1) and (rsp_i.ack = '1') and (rsp_i.err = '0') then
          req_phase <= REQ2;
          rsp1_data <= rsp_i.data; -- capture first word response
          r2_stb <= '1';
        elsif (req_phase = REQ2) and (rsp_i.ack = '1') then
          req_phase <= REQ1; -- split complete
        end if;
      end if;
    end if;
  end process unaligned_fsm;


  -- Bus Output and Response Mux -----------------------------------------------------------
  -- -----------------------------------------------------------------------------------------
  bus_output: process(req_i, r1_stb, r1_addr, r1_data, r1_ben, r2_stb, r2_data, r2_ben,
                      acc_addr, acc_funct3, state, req2_split,
                      req_phase, rsp1_data, pmp_fault_i, rsp_i)
  begin
    req_o       <= req_i;
    req_o.addr  <= r1_addr;
    req_o.data  <= r1_data;
    req_o.ben   <= r1_ben;
    req_o.stb   <= r1_stb and (not pmp_fault_i);
    req_o.burst <= '0';

    rsp_o <= rsp_i; -- Default or first sub access

    -- Second sub-access
    if req2_split then
      if (req_phase = REQ2) then
        req_o.addr <= std_ulogic_vector(unsigned(r1_addr) + 4);
        req_o.data <= r2_data;
        req_o.ben  <= r2_ben;
        req_o.stb  <= r2_stb and (not pmp_fault_i);
      end if;
      -- Wait for second ack
      if (req_phase = REQ1) then
        rsp_o.ack <= '0';
      else
        rsp_o.ack <= rsp_i.ack;
      end if;
    end if;

    -- Read data reconstruction
    if (req_i.rw = '0') then
      if (state = S_IDLE) then
        -- aligned response: extract bytes and sign-extend
        case acc_funct3(1 downto 0) is
          when "00" => -- byte
            case acc_addr(1 downto 0) is
              when "00"   => rsp_o.data <= replicate_f((not acc_funct3(2)) and rsp_i.data(7),  24) & rsp_i.data(7  downto 0);
              when "01"   => rsp_o.data <= replicate_f((not acc_funct3(2)) and rsp_i.data(15), 24) & rsp_i.data(15 downto 8);
              when "10"   => rsp_o.data <= replicate_f((not acc_funct3(2)) and rsp_i.data(23), 24) & rsp_i.data(23 downto 16);
              when others => rsp_o.data <= replicate_f((not acc_funct3(2)) and rsp_i.data(31), 24) & rsp_i.data(31 downto 24);
            end case;
          when "01" => -- half-word (aligned)
            if (acc_addr(1) = '0') then
              rsp_o.data <= replicate_f((not acc_funct3(2)) and rsp_i.data(15), 16) & rsp_i.data(15 downto 0);
            else
              rsp_o.data <= replicate_f((not acc_funct3(2)) and rsp_i.data(31), 16) & rsp_i.data(31 downto 16);
            end if;
          when others => -- word
            rsp_o.data <= rsp_i.data;
        end case;
      elsif (state = S_UNALIGNED_HW) then
        if (acc_addr(1) = '0') then
          -- offset "01": bytes 1-2 within a single aligned word
          rsp_o.data <= replicate_f((not acc_funct3(2)) and rsp_i.data(23), 16) & rsp_i.data(23 downto 8);
        elsif (req_phase = REQ2) then
          -- offset "11": byte3/word1 (captured) + byte0/word2 (current)
          rsp_o.data <= replicate_f((not acc_funct3(2)) and rsp_i.data(7), 16) & rsp_i.data(7 downto 0) & rsp1_data(31 downto 24);
        end if;
      elsif (req_phase = REQ2) then -- S_UNALIGNED_W
        -- reassemble 32-bit word from the two sub-access responses
        case acc_addr(1 downto 0) is
          when "01"   => rsp_o.data <= rsp_i.data(7  downto 0) & rsp1_data(31 downto 8);
          when "10"   => rsp_o.data <= rsp_i.data(15 downto 0) & rsp1_data(31 downto 16);
          when others => rsp_o.data <= rsp_i.data(23 downto 0) & rsp1_data(31 downto 24);
        end case;
      end if;
    end if;
  end process bus_output;

end neorv32_cpu_lsu_unaligned_rtl;
