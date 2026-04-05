-- ================================================================================ --
-- NEORV32 CPU - Load/Store Unit                                                    --
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

entity neorv32_cpu_lsu is
  generic (
    HART_ID      : natural; -- hardware thread ID
    AMO_EN       : boolean; -- enable atomic memory accesses
    UNALIGNED_EN : boolean  -- enable hardware-supported unaligned loads/stores (Zicclsm)
  );
  port (
    -- global control --
    clk_i       : in  std_ulogic; -- global clock, rising edge
    rstn_i      : in  std_ulogic; -- global reset, low-active, async
    ctrl_i      : in  ctrl_bus_t; -- main control bus
    -- memory data access interface --
    addr_i      : in  std_ulogic_vector(31 downto 0); -- access address
    wdata_i     : in  std_ulogic_vector(31 downto 0); -- write data
    rdata_o     : out std_ulogic_vector(31 downto 0); -- read data
    mar_o       : out std_ulogic_vector(31 downto 0); -- current memory address register
    wait_o      : out std_ulogic;                     -- wait for access to complete
    err_o       : out std_ulogic_vector(3 downto 0);  -- alignment/access errors
    pmp_fault_i : in  std_ulogic;                     -- PMP read/write access fault
    -- data bus --
    dbus_req_o  : out bus_req_t; -- request
    dbus_rsp_i  : in  bus_rsp_t  -- response
  );
end neorv32_cpu_lsu;

architecture neorv32_cpu_lsu_rtl of neorv32_cpu_lsu is

  signal req : bus_req_t;
  signal misalign : std_ulogic;

  -- unaligned split-transaction support --
  -- Set by mem_do_reg on lsu_mo_en; held stable until next transaction.
  type state_t is (
    S_INACTIVE, S_UNALIGNED_W, S_UNALIGNED_HW
  );
  signal state            : state_t;
  signal req2_needed      : boolean;                        -- access straddles a word boundary and requires a 2nd bus transaction
  signal req2_wdata       : std_ulogic_vector(31 downto 0); -- 2nd sub-access write data

  -- Set/cleared by unaligned_fsm.
  type phase_t is (
    REQ1, REQ2
  );
  signal req_phase   : phase_t; -- REQ1=waiting for 1st ack, REQ2=waiting for 2nd ack
  signal req_phase_d : phase_t; -- registered delay of req_phase (for one-cycle stb pulse)
  signal req1_data   : std_ulogic_vector(31 downto 0); -- captured 1st response word

begin

  -- Atomic Memory Access -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  amo_enabled:
  if AMO_EN generate
    amo_reg: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        req.amo   <= '0';
        req.amoop <= (others => '0');
        req.lock  <= '0';
      elsif rising_edge(clk_i) then
        if (ctrl_i.lsu_mo_en = '1') then
          -- atomic memory access operation --
          req.amo <= ctrl_i.ir_opcode(2);
          case ctrl_i.ir_funct12(11 downto 7) is
            when "00010" => req.amoop <= "1000"; -- Zalrsc.LR
            when "00011" => req.amoop <= "1001"; -- Zalrsc.SC
            when "00000" => req.amoop <= "0001"; -- Zaamo.ADD
            when "00100" => req.amoop <= "0010"; -- Zaamo.XOR
            when "01100" => req.amoop <= "0011"; -- Zaamo.AND
            when "01000" => req.amoop <= "0100"; -- Zaamo.OR
            when "10000" => req.amoop <= "1110"; -- Zaamo.MIN
            when "10100" => req.amoop <= "1111"; -- Zaamo.MAX
            when "11000" => req.amoop <= "0110"; -- Zaamo.MINU
            when "11100" => req.amoop <= "0111"; -- Zaamo.MAXU
            when others  => req.amoop <= "0000"; -- Zaamo.SWAP
          end case;
        end if;
        -- bus locking for read-modify-write operations --
        if (ctrl_i.lsu_mo_en = '1') and (ctrl_i.ir_opcode(2) = '1') and (ctrl_i.ir_funct12(8) = '0') then
          req.lock <= '1'; -- set if atomic read-modify-write instruction
        elsif (dbus_rsp_i.ack = '1') or (ctrl_i.cpu_trap = '1') then
          req.lock <= '0'; -- clear at the end of the bus access
        end if;
      end if;
    end process amo_reg;
  end generate;

  -- no atomic memory operations --
  amo_disabled:
  if not AMO_EN generate
    req.amo   <= '0';
    req.amoop <= (others => '0');
    req.lock  <= '0';
  end generate;


  -- Request --------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- This process captures address, data, byte-enables and misalignment for each transaction.
  -- When UNALIGNED_EN is true and a non-AMO misaligned access is detected, the address is
  -- aligned down, byte-enables and data are adjusted for the first sub-access, and split
  -- parameters for the second sub-access are registered.
  mem_do_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      req.meta           <= (others => '0');
      req.addr           <= (others => '0');
      req.data           <= (others => '0');
      req.ben            <= (others => '0');
      req.rw             <= '0';
      misalign           <= '0';
      state              <= S_INACTIVE;
      req2_wdata         <= (others => '0');
    elsif rising_edge(clk_i) then
      if (ctrl_i.lsu_mo_en = '1') then
        req.meta <= std_ulogic_vector(to_unsigned(HART_ID, 2)) & ctrl_i.cpu_debug & ctrl_i.lsu_priv & '0';
        if AMO_EN and (ctrl_i.ir_opcode(2) = '1') and (ctrl_i.ir_funct12(8) = '0') then
          req.rw <= '0'; -- atomic read-modify-write operations are modified load requests
        else
          req.rw <= ctrl_i.lsu_wr;
        end if;
        case ctrl_i.ir_funct3(1 downto 0) is
          when "00" => -- byte (always aligned)
            req.addr           <= addr_i;
            req.data           <= wdata_i(7 downto 0) & wdata_i(7 downto 0) & wdata_i(7 downto 0) & wdata_i(7 downto 0);
            req.ben(0)         <= (not addr_i(1)) and (not addr_i(0));
            req.ben(1)         <= (not addr_i(1)) and (    addr_i(0));
            req.ben(2)         <= (    addr_i(1)) and (not addr_i(0));
            req.ben(3)         <= (    addr_i(1)) and (    addr_i(0));
            misalign           <= '0';
            state              <= S_INACTIVE;

          when "01" => -- half-word
            if UNALIGNED_EN and (addr_i(0) = '1') and (ctrl_i.ir_opcode(2) = '0') then
              -- UNALIGNED_EN: handle misaligned half-word
              misalign         <= '0';
              state            <= S_UNALIGNED_HW;
              req.addr         <= addr_i;
              if (addr_i(1) = '0') then
                -- offset "01": bytes 1-2 both within the same aligned word, no straddle
                -- lane1=bits[15:8]=wdata[7:0], lane2=bits[23:16]=wdata[15:8]
                req.ben        <= "0110";
                req.data       <= x"00" & wdata_i(15 downto 0) & x"00";
              else
                -- offset "11": byte 3 of word1, byte 0 of word2 - straddles boundary
                req.ben        <= "1000";
                req.data       <= wdata_i(7 downto 0) & wdata_i(7 downto 0) & wdata_i(7 downto 0) & wdata_i(7 downto 0);
                req2_wdata     <= wdata_i(15 downto 8) & wdata_i(15 downto 8) & wdata_i(15 downto 8) & wdata_i(15 downto 8);
              end if;
            else
              -- aligned half-word or AMO
              req.addr         <= addr_i;
              req.data         <= wdata_i(15 downto 0) & wdata_i(15 downto 0);
              req.ben          <= addr_i(1) & addr_i(1) & (not addr_i(1)) & (not addr_i(1));
              misalign         <= addr_i(0);
              state            <= S_INACTIVE;
            end if;

          when others => -- word
            if UNALIGNED_EN and ((addr_i(1) or addr_i(0)) = '1') and (ctrl_i.ir_opcode(2) = '0') then
              -- handle misaligned word (all non-zero offsets straddle a word boundary)
              misalign         <= '0';
              state            <= S_UNALIGNED_W;
              req.addr         <= addr_i;
              case addr_i(1 downto 0) is
                when "01" => -- 3 bytes in word1 (lanes 1,2,3), 1 byte in word2 (lane 0)
                  req.ben    <= "1110";
                  req.data   <= wdata_i(23 downto 0) & x"00";
                  req2_wdata <= wdata_i(31 downto 24) & wdata_i(31 downto 24) & wdata_i(31 downto 24) & wdata_i(31 downto 24);
                when "10" => -- 2 bytes in word1 (lanes 2,3), 2 bytes in word2 (lanes 0,1)
                  req.ben    <= "1100";
                  req.data   <= wdata_i(15 downto 0) & x"0000";
                  req2_wdata <= wdata_i(31 downto 16) & wdata_i(31 downto 16);
                when others => -- "11": 1 byte in word1 (lane 3), 3 bytes in word2 (lanes 0,1,2)
                  req.ben    <= "1000";
                  req.data   <= wdata_i(7 downto 0) & wdata_i(7 downto 0) & wdata_i(7 downto 0) & wdata_i(7 downto 0);
                  req2_wdata <= x"00" & wdata_i(31 downto 8);
              end case;
            else
              -- aligned word or AMO
              req.addr       <= addr_i;
              req.data       <= wdata_i;
              req.ben        <= (others => '1');
              misalign       <= addr_i(1) or addr_i(0);
              state          <= S_INACTIVE;
            end if;

        end case;
      end if;
    end if;
  end process mem_do_reg;

  -- direct output --
  req.burst <= '0'; -- only non-burst/single-accesses
  req.fence <= ctrl_i.lsu_fence;

  -- req2_needed: true when the access straddles a word boundary (all word misalignments; HW only at offset "11")
  req2_needed <= (state = S_UNALIGNED_W) or (state = S_UNALIGNED_HW and req.addr(1) = '1');

  -- Unaligned Split-Transaction FSM -----------------------------------------------------------
  -- Manages req_phase (which sub-access is in flight) and captures first response word.
  -- -------------------------------------------------------------------------------------------
  unaligned_fsm: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      req_phase   <= REQ1;
      req_phase_d <= REQ1;
      req1_data     <= (others => '0');
    elsif rising_edge(clk_i) then
      req_phase_d <= req_phase; -- one-cycle delay for rising-edge detection
      if UNALIGNED_EN then
        if (ctrl_i.cpu_trap = '1') then
          -- exception: abort any in-progress split
          req_phase <= REQ1;
        elsif (state /= S_INACTIVE) and (req2_needed) then
          if (req_phase = REQ1) and (dbus_rsp_i.ack = '1') then
            -- first sub-access complete: save word1, move to phase 1
            req_phase <= REQ2;
            req1_data   <= dbus_rsp_i.data;
          elsif (req_phase = REQ2) and (dbus_rsp_i.ack = '1') then
            -- second sub-access complete: transaction done
            req_phase <= REQ1;
          end if;
        end if;
      end if;
    end if;
  end process unaligned_fsm;


  -- Bus Output Mux ----------------------------------------------------------------------------
  -- Generate a second transaction if the unaligned accesses are allowed and the request requires it
  -- -------------------------------------------------------------------------------------------
  unaligned_output:
  if UNALIGNED_EN generate
    req.stb <= '0'; -- stb is not used in this path; dbus_req_o.stb is driven by the process below
    mar_o   <= req.addr; -- unaligned address reported to MTVAL CSR on fault

    bus_output: process(req, req2_wdata, req2_needed,
                        req_phase, req_phase_d, misalign, pmp_fault_i,
                        state, ctrl_i, dbus_rsp_i)
    begin
      -- defaults: first sub-access / aligned access
      -- req.addr holds the original unaligned address; mask it for the bus
      dbus_req_o      <= req;
      dbus_req_o.addr <= req.addr(31 downto 2) & "00";
      dbus_req_o.stb  <= ctrl_i.lsu_req and (not misalign) and (not pmp_fault_i);

      if req2_needed then
        if req_phase = REQ2 then
          -- second sub-access: switch to second word address/data/ben
          -- ben bits 2:0 = number of bytes in the second word;
          dbus_req_o.addr <= std_ulogic_vector(unsigned(req.addr(31 downto 2) & "00") + 4);
          dbus_req_o.data <= req2_wdata;
          if (state = S_UNALIGNED_W) then
            dbus_req_o.ben(3) <= '0';
            dbus_req_o.ben(2) <= req.addr(1) and req.addr(0);
            dbus_req_o.ben(1) <= req.addr(1);
            dbus_req_o.ben(0) <= '1';
          else
            dbus_req_o.ben <= "0001";
          end if;
          -- one-cycle strobe pulse on the rising edge of req_phase
          if (req_phase_d = REQ1) then
            dbus_req_o.stb <= not pmp_fault_i;
          else
            dbus_req_o.stb <= '0';
          end if;
        end if;

        -- hold CPU in S_MEM_RSP until the final ack
        if (req_phase = REQ1) then
          wait_o <= '1';
        elsif dbus_rsp_i.ack = '1' then
          wait_o <= '0';
        else
          wait_o <= not dbus_rsp_i.ack;
        end if;
      else
        wait_o <= not dbus_rsp_i.ack;
      end if;
    end process bus_output;
  end generate;

  no_unaligned_output:
  if not UNALIGNED_EN generate
    -- access request (all source signals are driven by registers) --
    req.stb    <= ctrl_i.lsu_req and (not misalign) and (not pmp_fault_i);
    -- output bus request --
    dbus_req_o <= req;
    -- address feedback for MTVAL CSR --
    mar_o      <= req.addr;
    -- wait for bus response --
    wait_o     <= not dbus_rsp_i.ack;
    -- req_phase/req_phase_d/req1_data are driven by unaligned_fsm (always present);
    -- when UNALIGNED_EN=false they stay at reset ('0')
    -- unaligned_active/straddle/is_hw and split_* registers are driven by mem_do_reg reset
    -- and never updated when UNALIGNED_EN=false, so no extra assignments needed here.
  end generate;


  -- Response -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_di_reg: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      rdata_o <= (others => '0');
    elsif rising_edge(clk_i) then
      rdata_o <= (others => '0'); -- output zero if there is no pending memory request
      if (ctrl_i.lsu_mi_en = '1') then
        if UNALIGNED_EN and (state /= S_INACTIVE) then
          if (state = S_UNALIGNED_HW) then
            if (req.addr(1) = '0') then
              -- Half-word at offset "01": bytes 1-2 within the single aligned word
              -- Extract bits[23:8]; sign-extend if LH (funct3(2)='0')
              rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(23), 16) & dbus_rsp_i.data(23 downto 8);
            elsif (req_phase = REQ2) then
              -- half-word at offset "11": byte3/word1 + byte0/word2
              rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(7), 16) & dbus_rsp_i.data(7 downto 0) & req1_data(31 downto 24);
            end if;
          elsif (req_phase = REQ2) then
            -- Straddling access: combine req1_data (first word) with current response (second word)
            if (req.addr(1 downto 0) = "01") then -- word at offset "01": bytes 1-3 from word1, byte 0 from word2
              rdata_o <= dbus_rsp_i.data(7 downto 0) & req1_data(31 downto 8);
            elsif (req.addr(1 downto 0) = "10") then -- word at offset "10": bytes 2-3 from word1, bytes 0-1 from word2
              rdata_o <= dbus_rsp_i.data(15 downto 0) & req1_data(31 downto 16);
            else -- word at offset "11": byte 3 from word1, bytes 0-2 from word2
              rdata_o <= dbus_rsp_i.data(23 downto 0) & req1_data(31 downto 24);
            end if;
          end if;
          -- else: straddle phase 0 (first ack received but second not yet started):
          -- rdata_o is set to zero (default above); CPU is still in S_MEM_RSP (wait_o='1')
          -- and will not read rdata_o until the second ack clears wait_o.
        else
          -- standard aligned response path
          case ctrl_i.ir_funct3(1 downto 0) is
            when "00" => -- byte
              case req.addr(1 downto 0) is
                when "00"   => rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(7),  24) & dbus_rsp_i.data(7 downto 0);
                when "01"   => rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(15), 24) & dbus_rsp_i.data(15 downto 8);
                when "10"   => rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(23), 24) & dbus_rsp_i.data(23 downto 16);
                when others => rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(31), 24) & dbus_rsp_i.data(31 downto 24);
              end case;
            when "01" => -- half-word
              if (req.addr(1) = '0') then
                rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(15), 16) & dbus_rsp_i.data(15 downto 0);
              else
                rdata_o <= replicate_f((not ctrl_i.ir_funct3(2)) and dbus_rsp_i.data(31), 16) & dbus_rsp_i.data(31 downto 16);
              end if;
            when others => -- word
              rdata_o <= dbus_rsp_i.data;
          end case;
        end if;
      end if;
    end if;
  end process mem_di_reg;

  -- access/alignment errors --
  -- [NOTE] AMOs will report load AND store exceptions. However, only the store exception will be reported due to its higher priority.
  -- [NOTE] ACK is ignored for the error response to shorten the bus system's critical path.
  -- [NOTE] For UNALIGNED_EN, misalign is forced to '0' for non-AMO accesses in mem_do_reg,
  --        so misaligned regular loads/stores will not raise exceptions here.
  err_o(0) <= ctrl_i.lsu_mi_en and ctrl_i.lsu_rd and misalign; -- misaligned load
  err_o(1) <= ctrl_i.lsu_mi_en and ctrl_i.lsu_rd and (dbus_rsp_i.err or pmp_fault_i); -- load access error
  err_o(2) <= ctrl_i.lsu_mi_en and ctrl_i.lsu_wr and misalign; -- misaligned store
  err_o(3) <= ctrl_i.lsu_mi_en and ctrl_i.lsu_wr and (dbus_rsp_i.err or pmp_fault_i); -- store access error

end neorv32_cpu_lsu_rtl;
