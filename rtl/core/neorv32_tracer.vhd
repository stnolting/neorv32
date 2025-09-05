-- ================================================================================ --
-- NEORV32 SoC - Execution Tracer                                                   --
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

entity neorv32_tracer is
  generic (
    TRACE_DEPTH   : natural range 1 to 2**15; -- trace buffer depth (has to be a power of two)
    DUAL_CORE_EN  : boolean; -- trace the dual-core configuration
    SIM_LOG_EN    : boolean; -- enable simulation trace logging
    SIM_LOG_FILE0 : string := "";  -- trace log file CPU 0
    SIM_LOG_FILE1 : string := ""   -- trace log file CPU 1
  );
  port (
    clk_i     : in  std_ulogic;   -- global clock line
    rstn_i    : in  std_ulogic;   -- global reset line, low-active, async
    trace0_i  : in  trace_port_t; -- CPU 0 trace port
    trace1_i  : in  trace_port_t; -- CPU 1 trace port
    bus_req_i : in  bus_req_t;    -- bus request
    bus_rsp_o : out bus_rsp_t;    -- bus response
    irq_o     : out std_ulogic    -- tracing-done interrupt
  );
end neorv32_tracer;

architecture neorv32_tracer_rtl of neorv32_tracer is

  -- control register bits --
  constant ctrl_enable_c  : natural :=  0; -- r/w: module enable; reset module if 0
  constant ctrl_hsel_c    : natural :=  1; -- r/w: selected hart for tracing
  constant ctrl_start_c   : natural :=  2; -- r/w: start tracing; flag always reads as zero
  constant ctrl_stop_c    : natural :=  3; -- r/w: stop tracing; flag always reads as zero
  constant ctrl_run_c     : natural :=  4; -- r/-: tracing is running when set
  constant ctrl_avail_c   : natural :=  5; -- r/-: trace data available
  constant ctrl_irq_clr_c : natural :=  6; -- r/w: clear pending interrupt by writing one
  constant data_tbm_lsb_c : natural :=  7; -- r/-: log2(RX FIFO size) LSB
  constant data_tbm_msb_c : natural := 10; -- r/-: log2(RX FIFO size) MSB

  -- helpers --
  constant log2_fifo_size_c : natural := index_size_f(TRACE_DEPTH);

  -- simulation trace logger --
  component neorv32_tracer_simlog
    generic (
      LOG_FILE : string -- trace log file
    );
    port (
      clk_i   : in std_ulogic;  -- global clock line
      rstn_i  : in std_ulogic;  -- global reset line, low-active, async
      trace_i : in trace_port_t -- CPU trace port
    );
  end component;

  -- control registers --
  signal ctrl_en, ctrl_hsel, ctrl_start, ctrl_stop, ctrl_iclr : std_ulogic;
  signal stop_addr : std_ulogic_vector(30 downto 0);

  -- trace arbiter --
  type state_t is (S_OFFLINE, S_GET_SRC, S_GET_DST);
  type arbiter_t is record
    state : state_t; -- FSM state
    astop : std_ulogic; -- auto-stop tracing at given address
    run   : std_ulogic; -- tracing in progress
    src   : std_ulogic_vector(31 downto 0); -- source address
    dst   : std_ulogic_vector(31 downto 0); -- destination address
    trap  : std_ulogic; -- trap entry
    first : std_ulogic; -- first trap packet
    push  : std_ulogic; -- push SRC + DST to trace buffer
  end record;
  signal arbiter : arbiter_t;

  -- trace buffer interface --
  type fifo_t is record
    we,    re    : std_ulogic; -- write/read enable
    wdata, rdata : std_ulogic_vector(63 downto 0); -- write/read data
    avail, free  : std_ulogic; -- FIFO status
    clear        : std_ulogic; -- sync clear
  end record;
  signal fifo : fifo_t;

  -- misc --
  signal over_check : std_ulogic; -- FIFO overflow checker
  signal over_trash : std_ulogic; -- discard data from trace buffer
  signal trace_src  : trace_port_t; -- trace input stream

begin

  -- Bus Access -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bus_rsp_o  <= rsp_terminate_c;
      ctrl_en    <= '0';
      ctrl_hsel  <= '0';
      ctrl_start <= '0';
      ctrl_stop  <= '0';
      ctrl_iclr  <= '0';
      stop_addr  <= (others => '0');
    elsif rising_edge(clk_i) then
      -- bus handshake --
      bus_rsp_o.ack <= bus_req_i.stb;
      bus_rsp_o.err <= '0';
      -- write access --
      ctrl_start <= '0';
      ctrl_stop  <= '0';
      ctrl_iclr  <= '0';
      if (bus_req_i.stb = '1') and (bus_req_i.rw = '1') then
        if (bus_req_i.addr(3 downto 2) = "00") then -- control register
          ctrl_en    <= bus_req_i.data(ctrl_enable_c);
          ctrl_hsel  <= bus_req_i.data(ctrl_hsel_c) and bool_to_ulogic_f(DUAL_CORE_EN);
          ctrl_start <= bus_req_i.data(ctrl_start_c);
          ctrl_stop  <= bus_req_i.data(ctrl_stop_c);
          ctrl_iclr  <= bus_req_i.data(ctrl_irq_clr_c);
        end if;
        if (bus_req_i.addr(3 downto 2) = "01") then -- stop-address register
          stop_addr <= bus_req_i.data(31 downto 1);
        end if;
      end if;
      -- read access --
      bus_rsp_o.data <= (others => '0');
      if (bus_req_i.stb = '1') and (bus_req_i.rw = '0') then
        case bus_req_i.addr(3 downto 2) is
          when "00" => -- control register
            bus_rsp_o.data(ctrl_enable_c) <= ctrl_en;
            bus_rsp_o.data(ctrl_hsel_c)   <= ctrl_hsel and bool_to_ulogic_f(DUAL_CORE_EN);
            bus_rsp_o.data(ctrl_run_c)    <= arbiter.run;
            bus_rsp_o.data(ctrl_avail_c)  <= fifo.avail;
            bus_rsp_o.data(data_tbm_msb_c downto data_tbm_lsb_c) <= std_ulogic_vector(to_unsigned(log2_fifo_size_c, 4));
          when "01" => -- stop-address register
            bus_rsp_o.data <= stop_addr & '0';
          when "10" => -- trace data: source
            bus_rsp_o.data <= fifo.rdata(31 downto 0);
          when others => -- trace data: destination
            bus_rsp_o.data <= fifo.rdata(63 downto 32);
        end case;
      end if;
    end if;
  end process bus_access;


  -- Trace Control Arbiter ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trace_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      arbiter.state <= S_OFFLINE;
      arbiter.astop <= '0';
      arbiter.src   <= (others => '0');
      arbiter.dst   <= (others => '0');
      arbiter.trap  <= '0';
      arbiter.first <= '0';
      arbiter.push  <= '0';
    elsif rising_edge(clk_i) then
      -- defaults --
      arbiter.push <= '0';

      -- stop tracing at address --
      if (ctrl_en = '0') or (arbiter.run = '0') then
        arbiter.astop <= '0';
      elsif (trace_src.valid = '1') and (trace_src.pc(31 downto 1) = stop_addr) then
        arbiter.astop <= '1';
      end if;

      -- fsm --
      case arbiter.state is

        when S_OFFLINE => -- tracing disabled
        -- ------------------------------------------------------------
          arbiter.trap  <= '0'; -- no trap yet
          arbiter.first <= '1'; -- this will be the first trace packet
          if (ctrl_en = '1') and (ctrl_start = '1') then
            arbiter.state <= S_GET_SRC;
          end if;

        when S_GET_SRC => -- get delta source address
        -- ------------------------------------------------------------
          if (ctrl_en = '0') or (ctrl_stop = '1') or (arbiter.astop = '1') then
            arbiter.state <= S_OFFLINE;
          elsif (trace_src.mode(1) = '0') then -- halt tracing when we are in debug-mode
            arbiter.trap <= arbiter.trap or trace_src.trap;
            if (trace_src.valid = '1') or (trace_src.trap = '1') then
              arbiter.src(31 downto 1) <= trace_src.pc(31 downto 1);
            end if;
            if (trace_src.delta = '1') then -- non-linear PC change
              arbiter.state <= S_GET_DST;
            end if;
          end if;

        when S_GET_DST => -- get delta destination address
        -- ------------------------------------------------------------
          arbiter.src(0) <= arbiter.trap;
          arbiter.dst    <= trace_src.pc(31 downto 1) & arbiter.first;
          if (ctrl_en = '0') or (ctrl_stop = '1') or (arbiter.astop = '1') then
            arbiter.state <= S_OFFLINE;
          elsif (trace_src.mode(1) = '1') then -- discard this packet if we have entered debug-mode
            arbiter.state <= S_GET_SRC;
          elsif (trace_src.valid = '1') then -- first instruction of branch destination
            arbiter.push  <= '1';
            arbiter.trap  <= '0';
            arbiter.first <= '0';
            arbiter.state <= S_GET_SRC;
          end if;

        when others => -- undefined
        -- ------------------------------------------------------------
          arbiter.state <= S_OFFLINE;

      end case;
    end if;
  end process trace_arbiter;

  -- tracing in process --
  arbiter.run <= '0' when (arbiter.state = S_OFFLINE) else '1';

  -- trace source (CPU0 or CPU1) --
  trace_src <= trace0_i when (ctrl_hsel = '0') or (DUAL_CORE_EN = false) else trace1_i;


  -- Interrupt Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_generator: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      irq_o <= '0';
    elsif rising_edge(clk_i) then
      if (ctrl_en = '0') then
        irq_o <= '0';
      elsif (arbiter.astop = '1') then
        irq_o <= '1';
      elsif (ctrl_iclr = '1') then
        irq_o <= '0';
      end if;
    end if;
  end process irq_generator;


  -- Trace Buffer (implemented as FIFO) -----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trace_buffer_inst: entity neorv32.neorv32_prim_fifo
  generic map (
    AWIDTH  => log2_fifo_size_c,
    DWIDTH  => 2*32,
    OUTGATE => false
  )
  port map (
    -- global control --
    clk_i   => clk_i,
    rstn_i  => rstn_i,
    clear_i => fifo.clear,
    -- write port --
    wdata_i => fifo.wdata,
    we_i    => fifo.we,
    free_o  => fifo.free,
    -- read port --
    re_i    => fifo.re,
    rdata_o => fifo.rdata,
    avail_o => fifo.avail
  );

  -- FIFO access --
  fifo.clear <= not ctrl_en;
  fifo.we    <= arbiter.push;
  fifo.wdata <= arbiter.dst & arbiter.src;
  fifo.re    <= '1' when (over_trash = '1') or
                         ((bus_req_i.stb = '1') and (bus_req_i.rw = '0') and (bus_req_i.addr(3 downto 2) = "11")) else '0';

  -- discard oldest entry if overflowing --
  discard: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      over_check <= '0';
      over_trash <= '0';
    elsif rising_edge(clk_i) then
      if (over_check = '0') or (ctrl_en = '0') or (arbiter.run = '0') then
        over_check <= not fifo.free;
        over_trash <= '0';
      else
        over_check <= '0';
        over_trash <= '1';
      end if;
    end if;
  end process discard;


  -- Simulation Trace Logging ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
-- pragma translate_off
-- RTL_SYNTHESIS OFF

  -- CPU 0 --
  sim_trace0_enabled:
  if SIM_LOG_EN generate
    assert false report "[NEORV32] CPU 0 trace logging enabled: " & SIM_LOG_FILE0 severity note;
    neorv32_tracer_simlog0_inst: neorv32_tracer_simlog
    generic map (
      LOG_FILE => SIM_LOG_FILE0
    )
    port map (
      clk_i   => clk_i,
      rstn_i  => rstn_i,
      trace_i => trace0_i
    );
  end generate;

  -- CPU 1 --
  sim_trace1_enabled:
  if SIM_LOG_EN and DUAL_CORE_EN generate
    assert false report "[NEORV32] CPU 1 trace logging enabled: " & SIM_LOG_FILE1 severity note;
    neorv32_tracer_simlog1_inst: neorv32_tracer_simlog
    generic map (
      LOG_FILE => SIM_LOG_FILE1
    )
    port map (
      clk_i   => clk_i,
      rstn_i  => rstn_i,
      trace_i => trace1_i
    );
  end generate;

-- RTL_SYNTHESIS ON
-- pragma translate_on

end neorv32_tracer_rtl;


-- ================================================================================ --
-- NEORV32 SoC - Simulation-Only Trace Logger                                       --
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

-- pragma translate_off
-- RTL_SYNTHESIS OFF
use std.textio.all;
-- RTL_SYNTHESIS ON
-- pragma translate_on

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_tracer_simlog is
  generic (
    LOG_FILE : string -- trace log file
  );
  port (
    clk_i   : in std_ulogic;  -- global clock line
    rstn_i  : in std_ulogic;  -- global reset line, low-active, async
    trace_i : in trace_port_t -- CPU trace port
  );
end neorv32_tracer_simlog;

architecture neorv32_tracer_simlog_rtl of neorv32_tracer_simlog is

  -- decode instruction mnemonic --
  function decode_mnemonic_f(inst : std_ulogic_vector(31 downto 0)) return string is
  begin
    case inst(instr_opcode_msb_c downto instr_opcode_lsb_c) is
      -- ALU: register with immediate --
      when opcode_alui_c =>
        case inst(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          when "000"  => return "addi";
          when "001"  =>
            if (inst(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000000") then
              return "slli";
            else
              return "ALUI?";
            end if;
          when "010" => return "slti";
          when "011" => return "sltiu";
          when "100" => return "xori";
          when "101" =>
            if (inst(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000000") then
              return "srli";
            elsif (inst(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0100000") then
              return "srai";
            else
              return "ALUI?";
            end if;
          when "110"  => return "ori";
          when others => return "andi";
        end case;
      -- ALU: register with register --
      when opcode_alu_c =>
        if (inst(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000000") then -- base ISA
          case inst(instr_funct3_msb_c downto instr_funct3_lsb_c) is
            when "000"  => return "add";
            when "001"  => return "sll";
            when "010"  => return "slt";
            when "011"  => return "sltu";
            when "100"  => return "xor";
            when "101"  => return "srl";
            when "110"  => return "or";
            when others => return "and";
          end case;
        elsif (inst(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0100000") then -- base ISA
          case inst(instr_funct3_msb_c downto instr_funct3_lsb_c) is
            when "000"  => return "sub";
            when "101"  => return "sra";
            when others => return "ALU?";
          end case;
        elsif (inst(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000111") then -- Zicond
          case inst(instr_funct3_msb_c downto instr_funct3_lsb_c) is
            when "101"  => return "czero.eqz";
            when "111"  => return "czero.nez";
            when others => return "CZERO?";
          end case;
        elsif (inst(instr_funct7_msb_c downto instr_funct7_lsb_c) = "0000001") then -- M/Zmmul
          case inst(instr_funct3_msb_c downto instr_funct3_lsb_c) is
            when "000"  => return "mul";
            when "001"  => return "mulh";
            when "010"  => return "mulhsu";
            when "011"  => return "mulhu";
            when "100"  => return "div";
            when "101"  => return "divu";
            when "110"  => return "rem";
            when others => return "remu";
          end case;
        else
          return "ALU?";
        end if;
      -- upper-immediates --
      when opcode_lui_c   => return "lui";
      when opcode_auipc_c => return "auipc";
      -- jump-and-link --
      when opcode_jal_c  => return "jal";
      when opcode_jalr_c => return "jalr";
      -- conditional branches --
      when opcode_branch_c =>
        case inst(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          when "000"  => return "beq";
          when "001"  => return "bne";
          when "100"  => return "blt";
          when "101"  => return "bge";
          when "110"  => return "bltu";
          when "111"  => return "bgeu";
          when others => return "BRANCH?";
        end case;
      -- memory load --
      when opcode_load_c =>
        case inst(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          when "000"  => return "lb";
          when "001"  => return "lh";
          when "010"  => return "lw";
          when "100"  => return "lbu";
          when "101"  => return "lhu";
          when others => return "LOAD?";
        end case;
      -- memory store --
      when opcode_store_c =>
        case inst(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          when "000"  => return "sb";
          when "001"  => return "sh";
          when "010"  => return "sw";
          when others => return "STORE?";
        end case;
      -- atomic memory operations --
      when opcode_amo_c =>
        if (inst(instr_funct3_msb_c downto instr_funct3_lsb_c) = "010") then
          case inst(instr_funct5_msb_c downto instr_funct5_lsb_c) is
            when "00010" => return "lr.w";
            when "00011" => return "sc.w";
            when "00001" => return "amoswap.w ";
            when "00000" => return "amoadd.w";
            when "00100" => return "amoxor.w";
            when "01100" => return "amoand.w";
            when "01000" => return "amoor.w";
            when "10000" => return "amomin.w";
            when "10100" => return "amomax.w";
            when "11000" => return "amominu.w";
            when "11100" => return "amomaxu.w";
            when others  => return "AMO.W?";
          end case;
        else
          return "AMO?";
        end if;
      -- fences --
      when opcode_fence_c =>
        case inst(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          when "000"  => return "fence";
          when "001"  => return "fence.i";
          when others => return "FENCE?";
        end case;
      -- system / environment --
      when opcode_system_c =>
        case inst(instr_funct3_msb_c downto instr_funct3_lsb_c) is
          when "000" =>
            case inst(instr_imm12_msb_c downto instr_imm12_lsb_c) is
              when x"000" => return "ecall";
              when x"001" => return "ebreak";
              when x"105" => return "wfi";
              when x"302" => return "mret";
              when x"7b2" => return "dret";
              when others => return "ENV?";
            end case;
          when "001"  => return "csrrw";
          when "010"  => return "csrrs";
          when "011"  => return "csrrc";
          when "101"  => return "csrrwi";
          when "110"  => return "csrrsi";
          when "111"  => return "csrrci";
          when others => return "CSR?";
        end case;
      -- floating-point --
      when opcode_fpu_c => return "FPU?";
      -- custom instructions --
      when opcode_cust0_c => return "custom0";
      when opcode_cust1_c => return "custom1";
      when opcode_cust2_c => return "custom2";
      when opcode_cust3_c => return "custom3";
      -- undefined --
      when others => return "UNKNOWN?";
    end case;
  end function decode_mnemonic_f;

  -- decode CSR name --
  function decode_csr_f(addr : std_ulogic_vector(11 downto 0)) return string is
  begin
    case addr is
      -- user floating-point CSRs --
      when csr_fflags_c         => return "fflags";
      when csr_frm_c            => return "frm";
      when csr_fcsr_c           => return "fcsr";
      -- machine trap setup --
      when csr_mstatus_c        => return "mstatus";
      when csr_misa_c           => return "misa";
      when csr_mie_c            => return "mie";
      when csr_mtvec_c          => return "mtvec";
      when csr_mcounteren_c     => return "mcounteren";
      when csr_mstatush_c       => return "mstatush";
      -- machine configuration --
      when csr_menvcfg_c        => return "menvcfg";
      when csr_menvcfgh_c       => return "menvcfgh";
      -- machine counter setup --
      when csr_mcountinhibit_c  => return "mcountinhibit";
      when csr_mhpmevent3_c     => return "mhpmevent3";
      when csr_mhpmevent4_c     => return "mhpmevent4";
      when csr_mhpmevent5_c     => return "mhpmevent5";
      when csr_mhpmevent6_c     => return "mhpmevent6";
      when csr_mhpmevent7_c     => return "mhpmevent7";
      when csr_mhpmevent8_c     => return "mhpmevent8";
      when csr_mhpmevent9_c     => return "mhpmevent9";
      when csr_mhpmevent10_c    => return "mhpmevent10";
      when csr_mhpmevent11_c    => return "mhpmevent11";
      when csr_mhpmevent12_c    => return "mhpmevent12";
      when csr_mhpmevent13_c    => return "mhpmevent13";
      when csr_mhpmevent14_c    => return "mhpmevent14";
      when csr_mhpmevent15_c    => return "mhpmevent15";
      -- machine trap handling --
      when csr_mscratch_c       => return "mscratch";
      when csr_mepc_c           => return "mepc";
      when csr_mcause_c         => return "mcause";
      when csr_mtval_c          => return "mtval";
      when csr_mip_c            => return "mip";
      when csr_mtinst_c         => return "mtinst";
      -- physical memory protection - configuration --
      when csr_pmpcfg0_c        => return "pmpcfg0";
      when csr_pmpcfg1_c        => return "pmpcfg1";
      when csr_pmpcfg2_c        => return "pmpcfg2";
      when csr_pmpcfg3_c        => return "pmpcfg3";
      -- physical memory protection - address --
      when csr_pmpaddr0_c       => return "pmpaddr0";
      when csr_pmpaddr1_c       => return "pmpaddr1";
      when csr_pmpaddr2_c       => return "pmpaddr2";
      when csr_pmpaddr3_c       => return "pmpaddr3";
      when csr_pmpaddr4_c       => return "pmpaddr4";
      when csr_pmpaddr5_c       => return "pmpaddr5";
      when csr_pmpaddr6_c       => return "pmpaddr6";
      when csr_pmpaddr7_c       => return "pmpaddr7";
      when csr_pmpaddr8_c       => return "pmpaddr8";
      when csr_pmpaddr9_c       => return "pmpaddr9";
      when csr_pmpaddr10_c      => return "pmpaddr10";
      when csr_pmpaddr11_c      => return "pmpaddr11";
      when csr_pmpaddr12_c      => return "pmpaddr12";
      when csr_pmpaddr13_c      => return "pmpaddr13";
      when csr_pmpaddr14_c      => return "pmpaddr14";
      when csr_pmpaddr15_c      => return "pmpaddr15";
      -- trigger module registers --
      when csr_tselect_c        => return "tselect";
      when csr_tdata1_c         => return "tdata1";
      when csr_tdata2_c         => return "tdata2";
      when csr_tinfo_c          => return "tinfo";
      -- debug registers --
      when csr_dcsr_c           => return "dcsr";
      when csr_dpc_c            => return "dpc";
      when csr_dscratch0_c      => return "dscratch0";
      -- NEORV32-specific read/write user registers --
      when csr_cfureg0_c        => return "cfureg0";
      when csr_cfureg1_c        => return "cfureg1";
      when csr_cfureg2_c        => return "cfureg2";
      when csr_cfureg3_c        => return "cfureg3";
      -- machine counters/timers --
      when csr_mcycle_c         => return "mcycle";
      when csr_mtime_c          => return "mtime";
      when csr_minstret_c       => return "minstret";
      when csr_mhpmcounter3_c   => return "mhpmcounter3_";
      when csr_mhpmcounter4_c   => return "mhpmcounter4_";
      when csr_mhpmcounter5_c   => return "mhpmcounter5_";
      when csr_mhpmcounter6_c   => return "mhpmcounter6_";
      when csr_mhpmcounter7_c   => return "mhpmcounter7_";
      when csr_mhpmcounter8_c   => return "mhpmcounter8_";
      when csr_mhpmcounter9_c   => return "mhpmcounter9_";
      when csr_mhpmcounter10_c  => return "mhpmcounter10";
      when csr_mhpmcounter11_c  => return "mhpmcounter11";
      when csr_mhpmcounter12_c  => return "mhpmcounter12";
      when csr_mhpmcounter13_c  => return "mhpmcounter13";
      when csr_mhpmcounter14_c  => return "mhpmcounter14";
      when csr_mhpmcounter15_c  => return "mhpmcounter15";
      when csr_mcycleh_c        => return "mcycleh";
      when csr_mtimeh_c         => return "mtimeh";
      when csr_minstreth_c      => return "minstreth";
      when csr_mhpmcounter3h_c  => return "mhpmcounter3h";
      when csr_mhpmcounter4h_c  => return "mhpmcounter4h";
      when csr_mhpmcounter5h_c  => return "mhpmcounter5h";
      when csr_mhpmcounter6h_c  => return "mhpmcounter6h";
      when csr_mhpmcounter7h_c  => return "mhpmcounter7h";
      when csr_mhpmcounter8h_c  => return "mhpmcounter8h";
      when csr_mhpmcounter9h_c  => return "mhpmcounter9h";
      when csr_mhpmcounter10h_c => return "mhpmcounter10h";
      when csr_mhpmcounter11h_c => return "mhpmcounter11h";
      when csr_mhpmcounter12h_c => return "mhpmcounter12h";
      when csr_mhpmcounter13h_c => return "mhpmcounter13h";
      when csr_mhpmcounter14h_c => return "mhpmcounter14h";
      when csr_mhpmcounter15h_c => return "mhpmcounter15h";
      -- user counters/timers --
      when csr_cycle_c          => return "cycle";
      when csr_time_c           => return "time";
      when csr_instret_c        => return "instret";
      when csr_hpmcounter3_c    => return "hpmcounter3";
      when csr_hpmcounter4_c    => return "hpmcounter4";
      when csr_hpmcounter5_c    => return "hpmcounter5";
      when csr_hpmcounter6_c    => return "hpmcounter6";
      when csr_hpmcounter7_c    => return "hpmcounter7";
      when csr_hpmcounter8_c    => return "hpmcounter8";
      when csr_hpmcounter9_c    => return "hpmcounter9";
      when csr_hpmcounter10_c   => return "hpmcounter10";
      when csr_hpmcounter11_c   => return "hpmcounter11";
      when csr_hpmcounter12_c   => return "hpmcounter12";
      when csr_hpmcounter13_c   => return "hpmcounter13";
      when csr_hpmcounter14_c   => return "hpmcounter14";
      when csr_hpmcounter15_c   => return "hpmcounter15";
      when csr_cycleh_c         => return "cycleh";
      when csr_timeh_c          => return "timeh";
      when csr_instreth_c       => return "instreth";
      when csr_hpmcounter3h_c   => return "hpmcounter3h";
      when csr_hpmcounter4h_c   => return "hpmcounter4h";
      when csr_hpmcounter5h_c   => return "hpmcounter5h";
      when csr_hpmcounter6h_c   => return "hpmcounter6h";
      when csr_hpmcounter7h_c   => return "hpmcounter7h";
      when csr_hpmcounter8h_c   => return "hpmcounter8h";
      when csr_hpmcounter9h_c   => return "hpmcounter9h";
      when csr_hpmcounter10h_c  => return "hpmcounter10h";
      when csr_hpmcounter11h_c  => return "hpmcounter11h";
      when csr_hpmcounter12h_c  => return "hpmcounter12h";
      when csr_hpmcounter13h_c  => return "hpmcounter13h";
      when csr_hpmcounter14h_c  => return "hpmcounter14h";
      when csr_hpmcounter15h_c  => return "hpmcounter15h";
      -- machine information registers --
      when csr_mvendorid_c      => return "mvendorid";
      when csr_marchid_c        => return "marchid";
      when csr_mimpid_c         => return "mimpid";
      when csr_mhartid_c        => return "mhartid";
      when csr_mconfigptr_c     => return "mconfigptr";
      -- NEORV32-specific machine registers --
      when csr_mxcsr_c          => return "mxcsr";
      when csr_mxisa_c          => return "mxisa";
      -- unknown; just print address --
      when others               => return "x" & print_hex_f(addr);
    end case;
  end function decode_csr_f;

  -- decode instruction operands --
  function decode_operands_f(inst : std_ulogic_vector(31 downto 0)) return string is
    variable rs1_iv, rs2_iv, rd_iv : integer;
    variable is_v, ib_v, iu_v, ij_v, ii_v : std_ulogic_vector(31 downto 0);
    variable is_iv, ib_iv, iu_iv, ij_iv, ii_iv : integer;
  begin
    -- registers --
    rs1_iv := to_integer(unsigned(inst(instr_rs1_msb_c downto instr_rs1_lsb_c)));
    rs2_iv := to_integer(unsigned(inst(instr_rs2_msb_c downto instr_rs2_lsb_c)));
    rd_iv  := to_integer(unsigned(inst(instr_rd_msb_c downto instr_rd_lsb_c)));
    -- immediates --
    is_v := replicate_f(inst(31), 21) & inst(30 downto 25) & inst(11 downto 7);
    ib_v := replicate_f(inst(31), 20) & inst(7) & inst(30 downto 25) & inst(11 downto 8) & '0';
    iu_v := inst(31 downto 12) & x"000";
    ij_v := replicate_f(inst(31), 12) & inst(19 downto 12) & inst(20) & inst(30 downto 21) & '0';
    ii_v := replicate_f(inst(31), 21) & inst(30 downto 21) & inst(20);
    is_iv := to_integer(signed(is_v));
    ib_iv := to_integer(signed(ib_v));
    iu_iv := to_integer(signed(iu_v));
    ij_iv := to_integer(signed(ij_v));
    ii_iv := to_integer(signed(ii_v));
    -- instruction types --
    case inst(instr_opcode_msb_c downto instr_opcode_lsb_c) is
      when opcode_alui_c   => return "x" & integer'image(rd_iv)  & ", x"  & integer'image(rs2_iv) & ", "  & integer'image(ii_iv);
      when opcode_alu_c    => return "x" & integer'image(rd_iv)  & ", x"  & integer'image(rs2_iv) & ", x" & integer'image(rs1_iv);
      when opcode_lui_c    => return "x" & integer'image(rd_iv)  & ", 0x" & print_hex_f(iu_v);
      when opcode_auipc_c  => return "x" & integer'image(rd_iv)  & ", 0x" & print_hex_f(iu_v);
      when opcode_jal_c    => return "x" & integer'image(rd_iv)  & ", "   & integer'image(ij_iv);
      when opcode_jalr_c   => return "x" & integer'image(rd_iv)  & ", x"  & integer'image(rs2_iv) & ", "  & integer'image(ii_iv);
      when opcode_branch_c => return "x" & integer'image(rs2_iv) & ", x"  & integer'image(rs1_iv) & ", "  & integer'image(is_iv);
      when opcode_load_c   => return "x" & integer'image(rd_iv)  & ", "   & integer'image(is_iv)  & "(x"  & integer'image(rs1_iv) & ")";
      when opcode_store_c  => return "x" & integer'image(rs1_iv) & ", "   & integer'image(is_iv)  & "(x"  & integer'image(rs2_iv) & ")";
      when opcode_fence_c  => return "";
      when opcode_amo_c    =>
        if (inst(28) = '1') then -- zalrsc
          return "x" & integer'image(rd_iv) & ", (x" & integer'image(rs1_iv) & ")";
        else -- zaamo
          return "x" & integer'image(rd_iv) & ", x" & integer'image(rs2_iv) & ", (x" & integer'image(rs1_iv) & ")";
        end if;
      when opcode_system_c =>
        if (inst(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_env_c) or
           (inst(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_csril_c) then -- environment
          return "";
        elsif (inst(instr_funct3_msb_c) = '0') then -- csr-register
          return "x" & integer'image(rd_iv) & ", " & decode_csr_f(inst(31 downto 20)) & ", x" & integer'image(rs1_iv);
        else -- csr-immediate
          return "x" & integer'image(rd_iv) & ", " & decode_csr_f(inst(31 downto 20)) & ", " & integer'image(rs1_iv);
        end if;
      when others => return "";
    end case;
  end function decode_operands_f;

  signal trap_q : std_ulogic; -- trap entry
  signal cycle_q, order_q : unsigned(31 downto 0); -- cycle and index counters

begin

-- pragma translate_off
-- RTL_SYNTHESIS OFF

  -- Write Trace to Log File (SIMULATION ONLY) ----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  sim_trace_gen:
  if is_simulation_c generate
    sim_trace: process(rstn_i, clk_i)
      file     file_v : text open write_mode is LOG_FILE;
      variable line_v : line;
    begin
      if (rstn_i = '0') then
        trap_q  <= '0';
        cycle_q <= (others => '0');
        order_q <= (others => '0');
      elsif rising_edge(clk_i) then
        trap_q  <= trap_q or trace_i.trap;
        cycle_q <= cycle_q + 1;
        if (trace_i.valid = '1') then
          order_q <= order_q + 1;
          -- index --
          write(line_v, integer'(to_integer(order_q)));
          write(line_v, string'(" "));
          -- timestamp --
          write(line_v, integer'(to_integer(cycle_q)));
          write(line_v, string'(" "));
          -- instruction address --
          write(line_v, string'("0x"));
          write(line_v, string'(print_hex_f(trace_i.pc)));
          write(line_v, string'(" "));
          -- instruction word --
          write(line_v, string'("0x"));
          write(line_v, string'(print_hex_f(trace_i.inst)));
          write(line_v, string'(" "));
          -- privilege level --
          if (trace_i.mode(1) = '1') then
            write(line_v, string'("D "));
          elsif (trace_i.mode(0) = '1') then
            write(line_v, string'("M "));
          else
            write(line_v, string'("U "));
          end if;
          -- decoded instruction --
          if (trace_i.rvc = '1') then -- de-compressed instruction?
            write(line_v, string'("c."));
          end if;
          write(line_v, string'(decode_mnemonic_f(trace_i.inst)));
          write(line_v, string'(" "));
          write(line_v, string'(decode_operands_f(trace_i.inst)));
          -- trap entry --
          if (trap_q = '1') then
            trap_q <= '0';
            write(line_v, string'(" <TRAP_ENTRY>"));
          end if;
          --
          writeline(file_v, line_v);
        end if;
      end if;
    end process sim_trace;
  end generate;

-- RTL_SYNTHESIS ON
-- pragma translate_on

end neorv32_tracer_simlog_rtl;
