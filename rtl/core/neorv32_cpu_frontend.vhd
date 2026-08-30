-- ================================================================================ --
-- NEORV32 CPU - Front End (Instruction Fetch)                                      --
-- -------------------------------------------------------------------------------- --
-- + Fetch engine:    Fetches aligned 32-bit chunks of instruction words            --
-- + Prefetch buffer: Buffers pre-fetched 32-bit instruction data                   --
-- + Issue engine:    Decodes RVC instructions, aligns & issues instruction words   --
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

entity neorv32_cpu_frontend is
  generic (
    HART_ID     : natural; -- hardware thread ID
    RISCV_C     : boolean; -- implement C ISA extension
    RISCV_ZCB   : boolean; -- implement Zcb ISA sub-extension
    RISCV_ZCMOP : boolean; -- implement Zcmop ISA sub-extension
    RISCV_ZCMP  : boolean  -- implement Zcmp ISA sub-extension
  );
  port (
    -- global control --
    clk_i      : in  std_ulogic; -- global clock, rising edge
    rstn_i     : in  std_ulogic; -- global reset, low-active, async
    ctrl_i     : in  ctrl_bus_t; -- main control bus
    -- instruction fetch interface --
    ibus_req_o : out bus_req_t; -- request
    ibus_rsp_i : in  bus_rsp_t; -- response
    -- PMP interface --
    pmp_addr_o : out std_ulogic_vector(31 downto 0); -- access address
    pmp_priv_o : out std_ulogic; -- access privilege level
    pmp_err_i  : in  std_ulogic; -- PMP access fault
    -- back-end interface --
    frontend_o : out if_bus_t -- fetch data and status
  );
end entity;

architecture neorv32_cpu_frontend_rtl of neorv32_cpu_frontend is

  -- heart ID --
  constant hid_c : std_ulogic_vector(1 downto 0) := std_ulogic_vector(to_unsigned(HART_ID, 2));

  -- instruction prefetch buffer --
  component neorv32_cpu_frontend_ipb
  port (
    clk_i   : in  std_ulogic;
    clear_i : in  std_ulogic;
    wdata_i : in  std_ulogic_vector(16 downto 0);
    we_i    : in  std_ulogic;
    free_o  : out std_ulogic;
    re_i    : in  std_ulogic;
    rdata_o : out std_ulogic_vector(16 downto 0);
    avail_o : out std_ulogic
  );
  end component;

  -- instruction fetch engine --
  type state_t is (S_RESTART, S_REQUEST, S_PENDING);
  type fetch_t is record
    state : state_t;
    reset : std_ulogic; -- buffered restart request (after branch)
    addr  : std_ulogic_vector(31 downto 0); -- fetch address
    priv  : std_ulogic; -- fetch privilege level
    debug : std_ulogic; -- debug-mode access
  end record;
  signal fetch : fetch_t; -- FSM

  -- reset instruction fetch after branch --
  signal restart : std_ulogic;

  -- instruction prefetch buffer (FIFO) interface --
  type ipb_data_t is array (1 downto 0) of std_ulogic_vector(16 downto 0); -- bus_error & 16-bit instruction
  signal ipb_wdata, ipb_rdata : ipb_data_t;
  signal ipb_we,    ipb_re    : std_ulogic_vector(1 downto 0);
  signal ipb_free,  ipb_avail : std_ulogic_vector(1 downto 0);

  -- instruction issue engine --
  signal align_q, align_set, align_clr : std_ulogic;
  signal issue_valid : std_ulogic_vector(1 downto 0);
  signal cmd16 : std_ulogic_vector(15 downto 0);
  signal cmd32 : std_ulogic_vector(31 downto 0);

  -- Zcmp micro-op issue engine --
  type issue_state_t is (S_ISSUE, S_ZCMP);
  signal issue_state_reg, issue_state_nxt : issue_state_t;
  signal frontend_bus_zcmp, frontend_bus_issue : if_bus_t; -- front-end bus sources
  signal zcmp_instr_reg, zcmp_instr_nxt : std_ulogic_vector(15 downto 0); -- latched Zcmp instruction word
  signal zcmp_detect : std_ulogic; -- zcmp instruction detected, micro-op sequence starts next cycle
  signal zcmp_in_uop_seq : std_ulogic; -- micro-op sequence running
  signal issue_valid_zcmp : std_ulogic_vector(1 downto 0); -- IPB acknowledge at end of micro-op sequence
  signal instr_is_zcmp : std_ulogic; -- decompressor: instruction is a Zcmp instruction
  signal zcmp_op : zcmp_op_t; -- decompressor: Zcmp operation type

begin

  -- ******************************************************************************************************************
  -- Instruction Fetch (always fetch 32-bit-aligned 32-bit chunks of data)
  -- ******************************************************************************************************************

  -- Fetch Engine FSM -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  fetch_fsm: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      fetch.state <= S_RESTART;
      fetch.reset <= '1'; -- reset IPB and issue engine
      fetch.addr  <= (others => '0');
      fetch.priv  <= priv_mode_m_c;
      fetch.debug <= '0';
    elsif rising_edge(clk_i) then
      case fetch.state is

        when S_RESTART => -- set new start address
        -- ------------------------------------------------------------
          fetch.reset <= '0'; -- restart done
          fetch.addr  <= ctrl_i.pc_nxt; -- initialize from PC
          fetch.priv  <= ctrl_i.cpu_priv; -- set new privilege level
          fetch.debug <= ctrl_i.cpu_debug; -- access from debug-mode
          fetch.state <= S_REQUEST;

        when S_REQUEST => -- request next 32-bit-aligned instruction word
        -- ------------------------------------------------------------
          fetch.reset <= restart; -- buffer restart request
          if (ipb_free = "11") then -- free IPB space?
            fetch.state <= S_PENDING;
          elsif (restart = '1') then -- restart request due to branch
            fetch.state <= S_RESTART;
          end if;

        when S_PENDING => -- wait for bus response and write instruction data to prefetch buffer
        -- ------------------------------------------------------------
          fetch.reset <= restart; -- buffer restart request
          if (ibus_rsp_i.ack = '1') then -- wait for bus response
            fetch.addr <= std_ulogic_vector(unsigned(fetch.addr(31 downto 2)) + 1) & "00"; -- next (re-)aligned word
            if (restart = '1') then -- restart request due to branch
              fetch.state <= S_RESTART;
            else -- request next linear instruction word
              fetch.state <= S_REQUEST;
            end if;
          end if;

      end case;
    end if;
  end process;

  -- reset instruction fetch after branch --
  restart <= fetch.reset or ctrl_i.if_reset;

  -- PMP interface --
  pmp_addr_o <= fetch.addr(31 downto 2) & "00"; -- word aligned
  pmp_priv_o <= fetch.priv;

  -- instruction bus request --
  ibus_req_o.meta  <= hid_c & fetch.debug & fetch.priv & '1';
  ibus_req_o.addr  <= fetch.addr(31 downto 2) & "00"; -- word aligned
  ibus_req_o.stb   <= '1' when (fetch.state = S_REQUEST) and (ipb_free = "11") else '0';
  ibus_req_o.data  <= (others => '0'); -- read-only
  ibus_req_o.ben   <= (others => '1'); -- always full-word access
  ibus_req_o.rw    <= '0'; -- read-only
  ibus_req_o.amo   <= '0'; -- cannot be an atomic memory operation
  ibus_req_o.amoop <= (others => '0'); -- cannot be an atomic memory operation
  ibus_req_o.burst <= '0'; -- only single-access
  ibus_req_o.lock  <= '0'; -- always unlocked access

  -- IPB instruction data and status --
  ipb_wdata(0) <= (ibus_rsp_i.err or pmp_err_i) & ibus_rsp_i.data(15 downto 0);
  ipb_wdata(1) <= (ibus_rsp_i.err or pmp_err_i) & ibus_rsp_i.data(31 downto 16);

  -- IPB write enable --
  ipb_we(0) <= '1' when (fetch.state = S_PENDING) and (ibus_rsp_i.ack = '1') and ((fetch.addr(1) = '0') or (not RISCV_C)) else '0';
  ipb_we(1) <= '1' when (fetch.state = S_PENDING) and (ibus_rsp_i.ack = '1') else '0';

  -- Instruction Prefetch Buffer ------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  prefetch_buffer:
  for i in 0 to 1 generate
    ipb_inst: neorv32_cpu_frontend_ipb
    port map (
      -- global control --
      clk_i   => clk_i,
      clear_i => restart,
      -- write port --
      wdata_i => ipb_wdata(i),
      we_i    => ipb_we(i),
      free_o  => ipb_free(i),
      -- read port --
      re_i    => ipb_re(i),
      rdata_o => ipb_rdata(i),
      avail_o => ipb_avail(i)
    );
  end generate;

  -- ******************************************************************************************************************
  -- Instruction Issue (decompress 16-bit instruction and/or assemble a 32-bit instruction word)
  -- ******************************************************************************************************************

  issue_enabled:
  if RISCV_C generate

    -- Compressed Instructions Decoder --------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_cpu_decompressor_inst: entity neorv32.neorv32_cpu_decompressor
    generic map (
      ZCB_EN   => RISCV_ZCB,
      ZCMOP_EN => RISCV_ZCMOP,
      ZCMP_EN  => RISCV_ZCMP
    )
    port map (
      instr_i       => cmd16,
      instr_o       => cmd32,
      instr_is_zcmp => instr_is_zcmp,
      zcmp_op       => zcmp_op
    );

    -- half-word select --
    cmd16 <= ipb_rdata(0)(15 downto 0) when (align_q = '0') else ipb_rdata(1)(15 downto 0);
    frontend_bus_issue.i16 <= cmd16; -- original 16-bit instruction

    -- Issue Engine FSM -----------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    issue_fsm_sync: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        issue_state_reg <= S_ISSUE;
        zcmp_instr_reg  <= (others => '0');
      elsif rising_edge(clk_i) then
        issue_state_reg <= issue_state_nxt;
        zcmp_instr_reg  <= zcmp_instr_nxt;
        if (fetch.reset = '1') then
          align_q <= ctrl_i.pc_nxt(1); -- restart at unaligned address?
        elsif (ipb_re /= "00") then
          align_q <= (align_q and (not align_clr)) or align_set; -- alignment "RS flip-flop"
        end if;
      end if;
    end process;

    issue_fsm_comb: process(align_q, fetch, ipb_avail, ipb_rdata, cmd32, zcmp_instr_reg, instr_is_zcmp, issue_state_reg, zcmp_in_uop_seq)
    begin
      -- defaults --
      align_set <= '0';
      align_clr <= '0';
      issue_state_nxt <= issue_state_reg;
      issue_valid <= "00";
      issue_valid_zcmp <= "00";
      zcmp_instr_nxt <= zcmp_instr_reg;
      zcmp_detect <= '0';
      frontend_bus_issue.i32   <= (others => '0');
      frontend_bus_issue.compr <= '0';
      frontend_bus_issue.fault <= '0';
      frontend_bus_issue.zcmp_in_uop_seq  <= '0';
      frontend_bus_issue.zcmp_atomic_tail <= '0';

      case issue_state_reg is

        when S_ISSUE => -- regular instruction issue
        -- ------------------------------------------------------------
          -- start at LOW half-word --
          if (align_q = '0') then
            if (ipb_rdata(0)(1 downto 0) /= "11") and (ipb_avail(0) = '1') then -- compressed, consume IPB(0) entry
              if (instr_is_zcmp = '1') and (ipb_rdata(0)(16) = '0') then -- Zcmp instruction without fetch fault (faulted words use the regular path to raise an access fault)
                zcmp_instr_nxt  <= ipb_rdata(0)(15 downto 0); -- save Zcmp instruction
                issue_state_nxt <= S_ZCMP;
                zcmp_detect     <= '1'; -- Zcmp micro-op sequence is about to start
              else
                align_set <= ipb_avail(0); -- start of next instruction word is NOT 32-bit-aligned
                issue_valid <= '0' & ipb_avail(0);
                frontend_bus_issue.fault <= ipb_rdata(0)(16);
                frontend_bus_issue.i32   <= cmd32;
                frontend_bus_issue.compr <= '1';
              end if;
            elsif (ipb_avail = "11") then -- aligned uncompressed, consume both IPB entries
              issue_valid <= (others => (ipb_avail(1) and ipb_avail(0)));
              frontend_bus_issue.fault <= ipb_rdata(1)(16) or ipb_rdata(0)(16);
              frontend_bus_issue.i32   <= ipb_rdata(1)(15 downto 0) & ipb_rdata(0)(15 downto 0);
              frontend_bus_issue.compr <= '0';
            end if;
          -- start at HIGH half-word --
          elsif (ipb_avail(1) = '1') then
            if (ipb_rdata(1)(1 downto 0) /= "11") then -- compressed, consume IPB(1) entry
              if (instr_is_zcmp = '1') and (ipb_rdata(1)(16) = '0') then -- Zcmp instruction without fetch fault (faulted words use the regular path to raise an access fault)
                zcmp_instr_nxt  <= ipb_rdata(1)(15 downto 0); -- save Zcmp instruction
                issue_state_nxt <= S_ZCMP;
                zcmp_detect     <= '1'; -- Zcmp micro-op sequence is about to start
              else
                align_clr <= ipb_avail(1); -- start of next instruction word IS 32-bit-aligned again
                issue_valid <= ipb_avail(1) & '0';
                frontend_bus_issue.fault <= ipb_rdata(1)(16);
                frontend_bus_issue.i32   <= cmd32;
                frontend_bus_issue.compr <= '1';
              end if;
            elsif (ipb_avail = "11") then -- unaligned uncompressed, consume both IPB entries
              issue_valid <= (others => (ipb_avail(0) and ipb_avail(1)));
              frontend_bus_issue.fault <= ipb_rdata(0)(16) or ipb_rdata(1)(16);
              frontend_bus_issue.i32   <= ipb_rdata(0)(15 downto 0) & ipb_rdata(1)(15 downto 0);
              frontend_bus_issue.compr <= '0';
            end if;
          end if;

        when S_ZCMP => -- Zcmp micro-op sequence in progress; the sequencer drives the front-end bus
        -- ------------------------------------------------------------
          if (zcmp_in_uop_seq = '0') then -- sequence has completed
            issue_state_nxt <= S_ISSUE;
            zcmp_instr_nxt  <= (others => '0');
            if (align_q = '0') then
              align_set <= ipb_avail(0); -- start of next instruction word is NOT 32-bit-aligned
              issue_valid_zcmp <= "01"; -- consume the Zcmp instruction's IPB entry
            else
              align_clr <= ipb_avail(1); -- start of next instruction word IS 32-bit-aligned again
              issue_valid_zcmp <= "10"; -- consume the Zcmp instruction's IPB entry
            end if;
          end if;
          if (fetch.reset = '1') then -- on branch the IPBs must not be acknowledged as they contain outdated instructions
            issue_valid_zcmp <= "00";
            issue_state_nxt  <= S_ISSUE;
          end if;

      end case;
    end process;

    -- issue valid instruction word to execution stage --
    frontend_bus_issue.valid <= issue_valid(1) or issue_valid(0);
    frontend_bus_issue.zcmp_start <= zcmp_detect; -- Zcmp micro-op sequence is about to start

    -- bus switch: while a Zcmp micro-op sequence is being issued the sequencer drives the front-end bus --
    frontend_o <= frontend_bus_zcmp when (zcmp_in_uop_seq = '1') else frontend_bus_issue;

    -- IPB read access --
    ipb_re(0) <= (issue_valid(0) and ctrl_i.if_ready) or issue_valid_zcmp(0);
    ipb_re(1) <= (issue_valid(1) and ctrl_i.if_ready) or issue_valid_zcmp(1);

    -- Zcmp Micro-Op Sequencer ----------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    zcmp_enabled:
    if RISCV_ZCMP generate
      neorv32_cpu_zcmp_inst: entity neorv32.neorv32_cpu_zcmp
      port map (
        clk_i             => clk_i,
        rstn_i            => rstn_i,
        ctrl_i            => ctrl_i,
        zcmp_detect       => zcmp_detect,
        fetch_restart     => fetch.reset,
        ipb_avail         => ipb_avail,
        zcmp_instr_reg    => zcmp_instr_reg,
        zcmp_op           => zcmp_op,
        frontend_bus_zcmp => frontend_bus_zcmp,
        zcmp_in_uop_seq   => zcmp_in_uop_seq
      );
    end generate;

    zcmp_disabled:
    if not RISCV_ZCMP generate
      zcmp_in_uop_seq <= '0';
    end generate;

  end generate; -- /issue_enabled

  -- issue engine disabled --
  issue_disabled:
  if not RISCV_C generate
    align_q          <= '0';
    align_set        <= '0';
    align_clr        <= '0';
    issue_valid      <= (others => '0');
    cmd16            <= (others => '0');
    cmd32            <= (others => '0');
    ipb_re           <= (others => (ctrl_i.if_ready and ipb_avail(0)));
    frontend_o.valid <= ipb_avail(0);
    frontend_o.i32   <= ipb_rdata(1)(15 downto 0) & ipb_rdata(0)(15 downto 0);
    frontend_o.i16   <= (others => '0');
    frontend_o.compr <= '0';
    frontend_o.fault <= ipb_rdata(0)(16);
    frontend_o.zcmp_in_uop_seq  <= '0';
    frontend_o.zcmp_start       <= '0';
    frontend_o.zcmp_atomic_tail <= '0';
  end generate;

end architecture;


-- ================================================================================ --
-- NEORV32 CPU - Instruction Prefetch Buffer (FIFO)                                 --
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

entity neorv32_cpu_frontend_ipb is
  port (
    -- global control --
    clk_i   : in  std_ulogic; -- clock, rising edge
    clear_i : in  std_ulogic; -- sync reset, high-active
    -- write port --
    wdata_i : in  std_ulogic_vector(16 downto 0); -- write data
    we_i    : in  std_ulogic; -- write enable
    free_o  : out std_ulogic; -- at least one entry is free when set
    -- read port --
    re_i    : in  std_ulogic; -- read enable
    rdata_o : out std_ulogic_vector(16 downto 0); -- read data
    avail_o : out std_ulogic  -- data available when set
  );
end entity;

architecture neorv32_cpu_frontend_ipb_rtl of neorv32_cpu_frontend_ipb is

  -- IPB depth --
  constant awidth_c : natural := 1; -- 1 address bit = 2 entries

  -- pointers and status --
  signal w_pnt, r_pnt : std_ulogic_vector(awidth_c downto 0);
  signal match : std_ulogic;

  -- memory core --
  type ipb_t is array (0 to (2**awidth_c)-1) of std_ulogic_vector(16 downto 0);
  signal ipb : ipb_t;

begin

  -- Pointers -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  pointer_reg: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (clear_i = '1') then
        w_pnt <= (others => '0');
        r_pnt <= (others => '0');
      else
        if (we_i = '1') then
          w_pnt <= std_ulogic_vector(unsigned(w_pnt) + 1);
        end if;
        if (re_i = '1') then
          r_pnt <= std_ulogic_vector(unsigned(r_pnt) + 1);
        end if;
      end if;
    end if;
  end process;

  -- status --
  match   <= '1' when (r_pnt(awidth_c-1 downto 0) = w_pnt(awidth_c-1 downto 0)) else '0';
  free_o  <= '0' when (r_pnt(awidth_c) /= w_pnt(awidth_c)) and (match = '1') else '1';
  avail_o <= '0' when (r_pnt(awidth_c)  = w_pnt(awidth_c)) and (match = '1') else '1';

  -- Memory Core ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_write: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (we_i = '1') then
        ipb(to_integer(unsigned(w_pnt(awidth_c-1 downto 0)))) <= wdata_i;
      end if;
    end if;
  end process;

  -- asynchronous read --
  rdata_o <= ipb(to_integer(unsigned(r_pnt(awidth_c-1 downto 0))));

end architecture;
