-- ================================================================================ --
-- NEORV32 CPU - Front End (Instruction Fetch)                                      --
-- -------------------------------------------------------------------------------- --
-- + Fetch engine:    Fetches aligned 32-bit chunks of instruction words            --
-- + Prefetch buffer: Buffers pre-fetched 32-bit instruction data                   --
-- + Issue engine:    Decodes RVC instructions, aligns & issues instruction words   --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
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
    RISCV_C : boolean; -- implement C ISA extension
    RISCV_ZCB : boolean; -- implement Zcb ISA sub-extension
    RISCV_ZCMP : boolean -- implement Zcb ISA sub-extension
  );
  port (
    -- global control --
    clk_i : in std_ulogic; -- global clock, rising edge
    rstn_i : in std_ulogic; -- global reset, low-active, async
    ctrl_i : in ctrl_bus_t; -- main control bus
    -- instruction fetch interface --
    ibus_req_o : out bus_req_t;
    ibus_rsp_i : in bus_rsp_t;
    -- back-end interface --
    frontend_o : out if_bus_t
  );
end neorv32_cpu_frontend;

architecture neorv32_cpu_frontend_rtl of neorv32_cpu_frontend is

  -- instruction prefetch buffer --
  component neorv32_cpu_frontend_ipb
    generic (
      AWIDTH : natural;
      DWIDTH : natural
    );
    port (
      clk_i : in std_ulogic;
      rstn_i : in std_ulogic;
      clear_i : in std_ulogic;
      wdata_i : in std_ulogic_vector(DWIDTH - 1 downto 0);
      we_i : in std_ulogic;
      free_o : out std_ulogic;
      re_i : in std_ulogic;
      rdata_o : out std_ulogic_vector(DWIDTH - 1 downto 0);
      avail_o : out std_ulogic
    );
  end component;

  -- instruction fetch engine --
  type state_t is (S_RESTART, S_REQUEST, S_PENDING);
  type fetch_t is record
    state : state_t;
    restart : std_ulogic; -- buffered restart request (after branch)
    pc : std_ulogic_vector(XLEN - 1 downto 0);
    priv : std_ulogic; -- fetch privilege level
  end record;
  signal fetch : fetch_t;

  -- instruction prefetch buffer (FIFO) interface --
  type ipb_data_t is array (0 to 1) of std_ulogic_vector(16 downto 0); -- bus_error & 16-bit instruction
  type ipb_t is record
    wdata, rdata : ipb_data_t;
    we, re : std_ulogic_vector(1 downto 0);
    free, avail : std_ulogic_vector(1 downto 0);
  end record;
  signal ipb : ipb_t;

  -- instruction issue engine --
  signal align_q, align_set, align_clr : std_ulogic;
  signal issue_valid : std_ulogic_vector(1 downto 0);
  signal cmd16 : std_ulogic_vector(15 downto 0);
  signal cmd32 : std_ulogic_vector(31 downto 0);

  type issue_state_type is (S_ISSUE, S_ZCMP);
  signal issue_state_reg, issue_state_nxt : issue_state_type;

  signal frontend_bus_zcmp, frontend_bus_issue : if_bus_t;

  type uop_state_type is (S_IDLE, S_ZCMP_UOP_SEQ, S_POPRET, S_POPRETZ, S_ZCMP_BRANCH_ABORT);
  signal uop_state_reg, uop_state_nxt : uop_state_type;

  signal zcmp_instr_reg, zcmp_instr_nxt : std_ulogic_vector(15 downto 0) := (others => '0');

  signal uop_ctr, uop_ctr_next, uop_ctr_nxt_in_seq : integer range 0 to 15;
  signal uop_ctr_clr : std_ulogic;

  signal zcmp_stack_sw_offset, zcmp_stack_lw_offset : signed(11 downto 0);
  signal zcmp_reg_list : std_ulogic_vector(3 downto 0);
  signal zcmp_ls_reg : std_ulogic_vector(4 downto 0);
  signal zcmp_num_regs : integer range 0 to 15;
  signal zcmp_stack_adj_base : integer range 0 to 127;
  signal zcmp_stack_adj : integer range 0 to 255;
  signal zcmp_detect : std_ulogic;
  signal zcmp_in_uop_seq : std_ulogic;

  signal issue_valid_zcmp : std_ulogic_vector(1 downto 0);

  -- decompressor signal for Zcmp
  signal instr_is_zcmp : std_ulogic;
  signal zcmp_is_popret : std_ulogic;
  signal zcmp_is_popretz : std_ulogic;

  signal zcmp_instr, zcmp_sw_instr, zcmp_lw_instr, zcmp_jalr_instr : std_ulogic_vector(31 downto 0);
  constant zcmp_sw_instr_opcode : std_ulogic_vector(6 downto 0) := "0100011";
  constant zcmp_lw_instr_opcode : std_ulogic_vector(6 downto 0) := "0000011";
  constant zcmp_jalr_instr_opcode : std_ulogic_vector(6 downto 0) := "1100111";

  constant zcmp_instr_funct3 : std_ulogic_vector(2 downto 0) := "010";
  constant zcmp_instr_rs1_sp : std_ulogic_vector(4 downto 0) := "00010"; -- stack pointer 
  constant zcmp_instr_rs1_ra : std_ulogic_vector(4 downto 0) := "00001"; -- return address 

  signal zcmp_stack_adj_instr, zcmp_push_stack_adj_instr, zcmp_pop_stack_adj_instr, zcmp_li_a0_instr : std_ulogic_vector(31 downto 0);
  constant zcmp_addi_instr_opcode : std_ulogic_vector(6 downto 0) := "0010011";
  constant zcmp_addi_instr_funct3 : std_ulogic_vector(2 downto 0) := "000";
  constant zcmp_addi_rs1_sp : std_ulogic_vector(4 downto 0) := "00010"; -- stack pointer 

  constant zcmp_zero_a0_instr : std_ulogic_vector(31 downto 0) := x"00000513";

begin

  -- ******************************************************************************************************************
  -- Instruction Fetch (always fetch 32-bit-aligned 32-bit chunks of data)
  -- ******************************************************************************************************************

  -- Fetch Engine FSM -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  fetch_fsm : process (rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      fetch.state <= S_RESTART;
      fetch.restart <= '1'; -- reset IPB and issue engine
      fetch.pc <= (others => '0');
      fetch.priv <= priv_mode_m_c;
    elsif rising_edge(clk_i) then
      case fetch.state is

        when S_RESTART => -- set new start address
          -- ------------------------------------------------------------
          fetch.restart <= '0'; -- restart done
          fetch.pc <= ctrl_i.pc_nxt; -- initialize from PC
          fetch.priv <= ctrl_i.cpu_priv; -- set new privilege level
          fetch.state <= S_REQUEST;

        when S_REQUEST => -- request next 32-bit-aligned instruction word
          -- ------------------------------------------------------------
          fetch.restart <= fetch.restart or ctrl_i.if_reset; -- buffer restart request
          if (ipb.free = "11") then -- free IPB space?
            fetch.state <= S_PENDING;
          elsif (fetch.restart = '1') or (ctrl_i.if_reset = '1') then -- restart because of branch
            fetch.state <= S_RESTART;
          end if;

        when S_PENDING => -- wait for bus response and write instruction data to prefetch buffer
          -- ------------------------------------------------------------
          fetch.restart <= fetch.restart or ctrl_i.if_reset; -- buffer restart request
          if (ibus_rsp_i.ack = '1') then -- wait for bus response
            fetch.pc <= std_ulogic_vector(unsigned(fetch.pc) + 4); -- next word
            fetch.pc(1) <= '0'; -- (re-)align to 32-bit
            if (fetch.restart = '1') or (ctrl_i.if_reset = '1') then -- restart request due to branch
              fetch.state <= S_RESTART;
            else -- request next linear instruction word
              fetch.state <= S_REQUEST;
            end if;
          end if;

        when others => -- undefined
          -- ------------------------------------------------------------
          fetch.state <= S_RESTART;

      end case;
    end if;
  end process fetch_fsm;

  -- instruction bus request --
  ibus_req_o.addr <= fetch.pc(XLEN - 1 downto 2) & "00"; -- word aligned
  ibus_req_o.stb <= '1' when (fetch.state = S_REQUEST) and (ipb.free = "11") else
                    '0';
  ibus_req_o.data <= (others => '0'); -- read-only
  ibus_req_o.ben <= (others => '1'); -- always full-word access
  ibus_req_o.rw <= '0'; -- read-only
  ibus_req_o.src <= '1'; -- always "instruction fetch" access
  ibus_req_o.priv <= fetch.priv; -- current effective privilege level
  ibus_req_o.debug <= ctrl_i.cpu_debug; -- CPU is in debug mode
  ibus_req_o.amo <= '0'; -- cannot be an atomic memory operation
  ibus_req_o.amoop <= (others => '0'); -- cannot be an atomic memory operation
  ibus_req_o.burst <= '0'; -- only single-access
  ibus_req_o.lock <= '0'; -- always unlocked access
  ibus_req_o.fence <= ctrl_i.if_fence; -- fence request, valid without STB being set ("out-of-band" signal)

  -- IPB instruction data and status --
  ipb.wdata(0) <= ibus_rsp_i.err & ibus_rsp_i.data(15 downto 0);
  ipb.wdata(1) <= ibus_rsp_i.err & ibus_rsp_i.data(31 downto 16);

  -- IPB write enable --
  ipb.we(0) <= '1' when (fetch.state = S_PENDING) and (ibus_rsp_i.ack = '1') and ((fetch.pc(1) = '0') or (not RISCV_C)) else
               '0';
  ipb.we(1) <= '1' when (fetch.state = S_PENDING) and (ibus_rsp_i.ack = '1') else
               '0';

  -- Instruction Prefetch Buffer (FIFO) -----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  prefetch_buffer :
  for i in 0 to 1 generate
    ipb_inst : neorv32_cpu_frontend_ipb
    generic map(
      AWIDTH => 1, -- 1 address bit = 2 entries
      DWIDTH => 17 -- error status & instruction half-word data
    )
    port map(
      -- global control --
      clk_i => clk_i, -- clock, rising edge
      rstn_i => rstn_i, -- async reset, low-active
      clear_i => fetch.restart, -- sync reset, high-active
      -- write port --
      wdata_i => ipb.wdata(i), -- write data
      we_i => ipb.we(i), -- write enable
      free_o => ipb.free(i), -- at least one entry is free when set
      -- read port --
      re_i => ipb.re(i), -- read enable
      rdata_o => ipb.rdata(i), -- read data
      avail_o => ipb.avail(i) -- data available when set
    );
  end generate;

  -- ******************************************************************************************************************
  -- Instruction Issue (decompress 16-bit instruction and/or assemble a 32-bit instruction word)
  -- ******************************************************************************************************************

  issue_enabled :
  if RISCV_C generate

    -- Compressed Instructions Decoder --------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_cpu_decompressor_inst : entity neorv32.neorv32_cpu_decompressor
      generic map(
        ZCB_EN => RISCV_ZCB,
        ZCMP_EN => RISCV_ZCMP
      )
      port map(
        instr_i => cmd16,
        instr_o => cmd32,
        instr_is_zcmp => instr_is_zcmp,
        zcmp_is_popret => zcmp_is_popret,
        zcmp_is_popretz => zcmp_is_popretz
      );

    -- half-word select --
    cmd16 <= ipb.rdata(0)(15 downto 0) when (align_q = '0') else
             ipb.rdata(1)(15 downto 0);

    -- Issue Engine FSM -----------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    issue_fsm_sync : process (rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        align_q <= '0'; -- start aligned after reset
      elsif rising_edge(clk_i) then
        zcmp_instr_reg <= zcmp_instr_nxt;
        issue_state_reg <= issue_state_nxt;
        if (fetch.restart = '1') then
          align_q <= ctrl_i.pc_nxt(1); -- branch to unaligned address?
        elsif (ipb.re(0) = '1') or (ipb.re(1) = '1') or (issue_valid_zcmp /= "00") then
          align_q <= (align_q and (not align_clr)) or align_set; -- alignment "RS flip-flop"
        end if;
      end if;
    end process issue_fsm_sync;

    issue_fsm_comb : process (align_q, ipb, cmd32, zcmp_instr_reg, instr_is_zcmp, issue_state_reg, zcmp_in_uop_seq)
    begin
      -- defaults --
      align_set <= '0';
      align_clr <= '0';

      issue_state_nxt <= issue_state_reg;

      issue_valid_zcmp <= "00";
      issue_valid <= "00";
      frontend_bus_issue.instr <= (others => '0');
      frontend_bus_issue.compr <= '0';
      frontend_bus_issue.fault <= '0';

      frontend_bus_issue.zcmp_in_uop_seq <= '0';

      zcmp_instr_nxt <= zcmp_instr_reg;
      zcmp_detect <= '0';

      case issue_state_reg is
        when S_ISSUE =>
          -- start at LOW half-word --
          if (align_q = '0') then
            if (ipb.rdata(0)(1 downto 0) /= "11") and (ipb.avail(0) = '1') then -- compressed, consume IPB(0) entry

              if (instr_is_zcmp = '1') then
                zcmp_instr_nxt <= ipb.rdata(0)(15 downto 0);
                issue_state_nxt <= S_ZCMP;
                zcmp_detect <= '1';
              else
                align_set <= ipb.avail(0); -- start of next instruction word is NOT 32-bit-aligned
                issue_valid(0) <= ipb.avail(0);
                issue_valid(1) <= '0';
                frontend_bus_issue.fault <= ipb.rdata(0)(16);
                frontend_bus_issue.instr <= cmd32;
                frontend_bus_issue.compr <= '1';
              end if;
            elsif (ipb.avail = "11") then -- aligned uncompressed, consume both IPB entries
              issue_valid(0) <= ipb.avail(1) and ipb.avail(0);
              issue_valid(1) <= ipb.avail(1) and ipb.avail(0);
              frontend_bus_issue.fault <= ipb.rdata(1)(16) or ipb.rdata(0)(16);
              frontend_bus_issue.instr <= ipb.rdata(1)(15 downto 0) & ipb.rdata(0)(15 downto 0);
              frontend_bus_issue.compr <= '0';
            end if;
            -- start at HIGH half-word --
          elsif (ipb.avail(1) = '1') then
            if (ipb.rdata(1)(1 downto 0) /= "11") then -- compressed, consume IPB(1) entry

              if (instr_is_zcmp = '1') then
                zcmp_instr_nxt <= ipb.rdata(1)(15 downto 0);
                issue_state_nxt <= S_ZCMP;
                zcmp_detect <= '1';
              else
                align_clr <= ipb.avail(1); -- start of next instruction word is 32-bit-aligned again
                issue_valid(0) <= '0';
                issue_valid(1) <= ipb.avail(1);
                frontend_bus_issue.fault <= ipb.rdata(1)(16);
                frontend_bus_issue.instr <= cmd32;
                frontend_bus_issue.compr <= '1';
              end if;
            elsif (ipb.avail = "11") then -- unaligned uncompressed, consume both IPB entries
              issue_valid(0) <= ipb.avail(0) and ipb.avail(1);
              issue_valid(1) <= ipb.avail(0) and ipb.avail(1);
              frontend_bus_issue.fault <= ipb.rdata(0)(16) or ipb.rdata(1)(16);
              frontend_bus_issue.instr <= ipb.rdata(0)(15 downto 0) & ipb.rdata(1)(15 downto 0);
              frontend_bus_issue.compr <= '0';
            end if;
          end if;
        when S_ZCMP =>

          if not zcmp_in_uop_seq = '1' then
            issue_state_nxt <= S_ISSUE;
            zcmp_instr_nxt <= (others => '0');
            if (align_q = '0') then
              align_set <= ipb.avail(0); -- start of next instruction word is NOT 32-bit-aligned
              issue_valid_zcmp <= "01";
            else
              align_clr <= ipb.avail(1); -- start of next instruction word is 32-bit-aligned again
              issue_valid_zcmp <= "10";
            end if;
          end if;

          if (fetch.restart = '1') then -- on branch ipb's must not be acknowledged as they contain old instructions 
            issue_valid_zcmp <= "00";
            issue_state_nxt <= S_ISSUE;
          end if;
      end case;
    end process issue_fsm_comb;

    -- issue valid instruction word to execution stage --
    frontend_bus_issue.valid <= issue_valid(1) or issue_valid(0);

    frontend_o <= frontend_bus_zcmp when zcmp_in_uop_seq = '1' else
                  frontend_bus_issue;

    -- IPB read access --
    ipb.re(0) <= (issue_valid(0) and ctrl_i.if_ready) or (issue_valid_zcmp(0));
    ipb.re(1) <= (issue_valid(1) and ctrl_i.if_ready) or (issue_valid_zcmp(1));

    zcmp_enabled :
    if RISCV_ZCMP generate
      zcmp_reg_list <= zcmp_instr_reg(7 downto 4);
      zcmp_num_regs <= 13 when to_integer(unsigned(zcmp_reg_list)) = 15 else
                       0 when to_integer(unsigned(zcmp_reg_list)) < 4 else
                       to_integer(unsigned(zcmp_reg_list)) - 3;

      zcmp_stack_adj_base <= 64 when to_integer(unsigned(zcmp_reg_list)) = 15 else
                             48 when to_integer(unsigned(zcmp_reg_list)) >= 12 else
                             32 when to_integer(unsigned(zcmp_reg_list)) >= 8 else
                             16;

      zcmp_stack_adj <= zcmp_stack_adj_base + ((to_integer(unsigned(zcmp_instr_reg(3 downto 2))) * 16));

      zcmp_stack_sw_offset <= to_signed(-((zcmp_num_regs - uop_ctr) * 4), zcmp_stack_sw_offset'length);
      zcmp_stack_lw_offset <= to_signed(-((zcmp_num_regs - uop_ctr) * 4) + zcmp_stack_adj, zcmp_stack_lw_offset'length);

      zcmp_ls_reg <= "00001" when uop_ctr = 0 else -- ra
                     "01000" when uop_ctr = 1 else -- s0
                     "01001" when uop_ctr = 2 else -- s1
                     std_ulogic_vector(to_unsigned(uop_ctr + 15, zcmp_ls_reg'length)); -- s2-s11 (s2 == x18)

      zcmp_sw_instr <= std_ulogic_vector(zcmp_stack_sw_offset(11 downto 5)) & zcmp_ls_reg & zcmp_instr_rs1_sp & zcmp_instr_funct3 & std_ulogic_vector(zcmp_stack_sw_offset(4 downto 0)) & zcmp_sw_instr_opcode;
      -- zcmp_lw_instr <= std_ulogic_vector(zcmp_stack_lw_offset(11 downto 5)) & zcmp_ls_reg & zcmp_instr_rs1 & zcmp_instr_funct3 & std_ulogic_vector(zcmp_stack_lw_offset(4 downto 0)) & zcmp_lw_instr_opcode;

      zcmp_lw_instr <= std_ulogic_vector(zcmp_stack_lw_offset) & zcmp_instr_rs1_sp & zcmp_instr_funct3 & zcmp_ls_reg & zcmp_lw_instr_opcode;

      zcmp_instr <= zcmp_sw_instr when zcmp_instr_reg(9) = '0' else
                    zcmp_lw_instr;

      zcmp_push_stack_adj_instr <= std_ulogic_vector(-to_signed(zcmp_stack_adj, 12)) &
                                   zcmp_addi_rs1_sp & -- rs1 = sp 
                                   zcmp_addi_instr_funct3 &
                                   zcmp_addi_rs1_sp & -- rd = rs1 = sp 
                                   zcmp_addi_instr_opcode; -- addi 

      zcmp_pop_stack_adj_instr <= std_ulogic_vector(to_signed(zcmp_stack_adj, 12)) &
                                  zcmp_addi_rs1_sp & -- rs1 = sp 
                                  zcmp_addi_instr_funct3 &
                                  zcmp_addi_rs1_sp & -- rd = rs1 = sp 
                                  zcmp_addi_instr_opcode; -- addi 

      zcmp_stack_adj_instr <= zcmp_push_stack_adj_instr when zcmp_instr_reg(9) = '0' else
                              zcmp_pop_stack_adj_instr;

      zcmp_li_a0_instr <= (31 downto 12 => '0',
                          11 downto 7 => "01010",
                          6 downto 0 => zcmp_addi_instr_opcode);

      zcmp_jalr_instr <= (31 downto 20 => '0',
                         19 downto 15 => zcmp_instr_rs1_ra,
                         14 downto 7 => '0',
                         6 downto 0 => zcmp_jalr_instr_opcode);

      uop_ctr_clr <= '0';

      uop_ctr_next <= 0 when uop_ctr_clr = '1' else
                      uop_ctr_nxt_in_seq;

      uop_fsm_sync : process (rstn_i, clk_i)
      begin
        if (rstn_i = '0') then
          uop_ctr <= 0;
          uop_state_reg <= S_IDLE;
        elsif rising_edge(clk_i) then
          uop_ctr <= uop_ctr_next;
          uop_state_reg <= uop_state_nxt;
        end if;
      end process uop_fsm_sync;

      uop_fsm_comb : process (uop_state_reg, zcmp_jalr_instr, uop_ctr, fetch, ipb, zcmp_in_uop_seq, zcmp_is_popret, zcmp_is_popretz, ctrl_i, zcmp_detect, zcmp_num_regs, zcmp_instr, zcmp_stack_adj_instr)
      begin
        uop_ctr_nxt_in_seq <= uop_ctr;
        uop_state_nxt <= uop_state_reg;
        zcmp_in_uop_seq <= '0';
        frontend_bus_zcmp.valid <= '0';
        frontend_bus_zcmp.compr <= '0';
        frontend_bus_zcmp.fault <= '0';
        frontend_bus_zcmp.instr <= (others => '0');
        frontend_bus_zcmp.zcmp_in_uop_seq <= zcmp_in_uop_seq;

        case uop_state_reg is
          when S_IDLE =>
            if (zcmp_detect = '1') then
              uop_state_nxt <= S_ZCMP_UOP_SEQ;
            end if;

          when S_ZCMP_UOP_SEQ =>
            zcmp_in_uop_seq <= '1';
            if (uop_ctr = 15) then --last instruction
              frontend_bus_zcmp.instr <= zcmp_stack_adj_instr;
              frontend_bus_zcmp.valid <= '1';

              if (ctrl_i.if_ready = '1') then
                uop_ctr_nxt_in_seq <= 0;

                if (zcmp_is_popret = '1') then
                  uop_state_nxt <= S_POPRET;
                elsif (zcmp_is_popretz = '1') then
                  uop_state_nxt <= S_POPRETZ;
                else
                  uop_state_nxt <= S_IDLE;
                end if;
              end if;

            else

              if (ctrl_i.if_ready = '1') then
                uop_ctr_nxt_in_seq <= uop_ctr + 1;
              end if;

              frontend_bus_zcmp.instr <= zcmp_instr;
              frontend_bus_zcmp.valid <= '1';

              if (uop_ctr + 1 = zcmp_num_regs) then
                uop_ctr_nxt_in_seq <= 15;
              end if;

            end if;

            if (fetch.restart = '1') then
              uop_state_nxt <= S_ZCMP_BRANCH_ABORT;
              zcmp_in_uop_seq <= '0';
              uop_ctr_nxt_in_seq <= 0;
              frontend_bus_zcmp.valid <= '0';
              frontend_bus_zcmp.instr <= (others => '0');
            end if;

          when S_POPRET =>
            zcmp_in_uop_seq <= '1';
            frontend_bus_zcmp.instr <= zcmp_jalr_instr;
            frontend_bus_zcmp.valid <= '1';

            if (ctrl_i.if_ready = '1') then
              uop_state_nxt <= S_IDLE;
            end if;

          when S_POPRETZ =>
            zcmp_in_uop_seq <= '1';
            frontend_bus_zcmp.instr <= zcmp_zero_a0_instr; --zero a0
            frontend_bus_zcmp.valid <= '1';

            if (ctrl_i.if_ready = '1') then
              uop_state_nxt <= S_POPRET; -- issue ret instruction
            end if;

          when S_ZCMP_BRANCH_ABORT =>
            if (ipb.avail /= "00") then
              uop_state_nxt <= S_IDLE;
            end if;

        end case;
      end process;
    end generate; -- /zcmp_enabled

    zcmp_disabled :
    if not RISCV_ZCMP generate
      zcmp_reg_list <= (others => '0');
      zcmp_num_regs <= 0;
      zcmp_stack_adj_base <= 0;
      zcmp_stack_adj <= 0;
      zcmp_stack_sw_offset <= (others => '0');
      zcmp_ls_reg <= (others => '0');
      zcmp_sw_instr <= (others => '0');
      zcmp_push_stack_adj_instr <= (others => '0');
      uop_ctr_clr <= '0';
      zcmp_in_uop_seq <= '0';
    end generate; -- /zcmp_disabled

  end generate; -- /issue_enabled

  -- issue engine disabled --
  issue_disabled :
  if not RISCV_C generate
    align_q <= '0';
    align_set <= '0';
    align_clr <= '0';
    issue_valid <= (others => '0');
    cmd16 <= (others => '0');
    cmd32 <= (others => '0');
    ipb.re <= (others => (ctrl_i.if_ready and ipb.avail(0)));
    frontend_o.valid <= ipb.avail(0);
    frontend_o.instr <= ipb.rdata(1)(15 downto 0) & ipb.rdata(0)(15 downto 0);
    frontend_o.compr <= '0';
    frontend_o.fault <= ipb.rdata(0)(16);
  end generate;

end neorv32_cpu_frontend_rtl;

-- ================================================================================ --
-- NEORV32 CPU - Instruction Prefetch Buffer                                        --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cpu_frontend_ipb is
  generic (
    AWIDTH : natural; -- address width
    DWIDTH : natural -- data width
  );
  port (
    -- global control --
    clk_i : in std_ulogic; -- clock, rising edge
    rstn_i : in std_ulogic; -- async reset, low-active
    clear_i : in std_ulogic; -- sync reset, high-active
    -- write port --
    wdata_i : in std_ulogic_vector(DWIDTH - 1 downto 0); -- write data
    we_i : in std_ulogic; -- write enable
    free_o : out std_ulogic; -- at least one entry is free when set
    -- read port --
    re_i : in std_ulogic; -- read enable
    rdata_o : out std_ulogic_vector(DWIDTH - 1 downto 0); -- read data
    avail_o : out std_ulogic -- data available when set
  );
end neorv32_cpu_frontend_ipb;

architecture neorv32_cpu_frontend_ipb_rtl of neorv32_cpu_frontend_ipb is

  -- pointers and status --
  signal w_pnt, r_pnt : std_ulogic_vector(AWIDTH downto 0);
  signal match, empty, full : std_ulogic;

  -- memory core --
  type ipb_t is array (0 to (2 ** AWIDTH) - 1) of std_ulogic_vector(DWIDTH - 1 downto 0);
  signal ipb : ipb_t;

begin

  -- Pointers -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  pointer_reg : process (rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      w_pnt <= (others => '0');
      r_pnt <= (others => '0');
    elsif rising_edge(clk_i) then
      if (clear_i = '1') then
        w_pnt <= (others => '0');
      elsif (we_i = '1') then
        w_pnt <= std_ulogic_vector(unsigned(w_pnt) + 1);
      end if;
      if (clear_i = '1') then
        r_pnt <= (others => '0');
      elsif (re_i = '1') then
        r_pnt <= std_ulogic_vector(unsigned(r_pnt) + 1);
      end if;
    end if;
  end process pointer_reg;

  -- status --
  match <= '1' when (r_pnt(AWIDTH - 1 downto 0) = w_pnt(AWIDTH - 1 downto 0)) else
           '0';
  full <= '1' when (r_pnt(AWIDTH) /= w_pnt(AWIDTH)) and (match = '1') else
          '0';
  empty <= '1' when (r_pnt(AWIDTH) = w_pnt(AWIDTH)) and (match = '1') else
           '0';
  free_o <= not full;
  avail_o <= not empty;

  -- Memory Core ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_write : process (clk_i)
  begin
    if rising_edge(clk_i) then
      if (we_i = '1') then
        ipb(to_integer(unsigned(w_pnt(AWIDTH - 1 downto 0)))) <= wdata_i;
      end if;
    end if;
  end process mem_write;

  -- asynchronous(!) read --
  rdata_o <= ipb(to_integer(unsigned(r_pnt(AWIDTH - 1 downto 0))));

end neorv32_cpu_frontend_ipb_rtl;