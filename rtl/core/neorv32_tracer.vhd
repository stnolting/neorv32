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
    DUAL_CORE_EN  : boolean;                  -- trace the dual-core configuration
    SIM_LOG_EN    : boolean;                  -- enable simulation trace logging
    SIM_LOG_FILE0 : string := "";             -- trace log file CPU 0
    SIM_LOG_FILE1 : string := ""              -- trace log file CPU 1
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
  type state_t is (S_OFFLINE, S_TRACING);
  type arbiter_t is record
    state : state_t; -- FSM state
    first : std_ulogic; -- first trace entry
    valid : std_ulogic_vector(1 downto 0); -- valid-sample shift register
    compr : std_ulogic_vector(1 downto 0); -- is-decompressed shift register
    src   : std_ulogic_vector(31 downto 0); -- source address
    dst   : std_ulogic_vector(31 downto 0); -- destination address
    add   : std_ulogic_vector(31 downto 0); -- offset for next linear address
    nxt   : std_ulogic_vector(31 downto 0); -- next linear address
    push  : std_ulogic; -- push SRC + DST to trace buffer
    astop : std_ulogic; -- auto-stop tracing
    run   : std_ulogic; -- tracing in progress
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

  -- trace source select (CPU0 or CPU1) --
  trace_src <= trace0_i when (ctrl_hsel = '0') or (DUAL_CORE_EN = false) else trace1_i;


  -- Trace Control Arbiter ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trace_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      arbiter.state <= S_OFFLINE;
      arbiter.first <= '0';
      arbiter.valid <= (others => '0');
      arbiter.compr <= (others => '0');
      arbiter.src   <= (others => '0');
      arbiter.dst   <= (others => '0');
    elsif rising_edge(clk_i) then
      case arbiter.state is

        when S_OFFLINE => -- tracing disabled
        -- ------------------------------------------------------------
          arbiter.first <= '1'; -- this will be the first trace packet
          arbiter.valid <= (others => '0'); -- no valid data sampled yet
          if (ctrl_en = '1') and (ctrl_start = '1') then
            arbiter.state <= S_TRACING;
          end if;

        when S_TRACING => -- tracing in progress
        -- ------------------------------------------------------------
          arbiter.valid(0) <= '0'; -- default
          if (ctrl_en = '0') or (ctrl_stop = '1') or (arbiter.astop = '1') then -- tracing still running
            arbiter.state <= S_OFFLINE;
          elsif (trace_src.valid = '1') and (trace_src.debug = '0') then -- valid trace packet and not in debug-mode
            arbiter.valid(0) <= '1';
            arbiter.compr(0) <= trace_src.compr;
            arbiter.dst      <= trace_src.pc_rdata(31 downto 1) & trace_src.intr;
          end if;
          -- sample shift register --
          if (arbiter.valid(0) = '1') then
            arbiter.valid(1) <= '1';
            arbiter.compr(1) <= arbiter.compr(0);
            arbiter.src      <= arbiter.dst(31 downto 1) & arbiter.first;
          end if;
          -- clear first-packet flag on first push
          if (arbiter.push = '1') then
            arbiter.first <= '0';
          end if;

        when others => -- undefined
        -- ------------------------------------------------------------
          arbiter.state <= S_OFFLINE;

      end case;
    end if;
  end process trace_arbiter;

  -- compute next linear address --
  arbiter.add <= x"00000002" when (arbiter.compr(1) = '1') else x"00000004";
  arbiter.nxt <= std_ulogic_vector(unsigned(arbiter.src) + unsigned(arbiter.add));

  -- push to trace buffer if address delta (new PC != next linear address) --
  arbiter.push <= '1' when (arbiter.dst(31 downto 1) /= arbiter.nxt(31 downto 1)) and (arbiter.valid = "11") else '0';

  -- automatic stop if reaching stop address --
  arbiter.astop <= '1' when (arbiter.src(31 downto 1) = stop_addr) and (arbiter.valid(0) = '1') else '0';

  -- tracing in process --
  arbiter.run <= '0' when (arbiter.state = S_OFFLINE) else '1';


  -- Interrupt Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_generator: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      irq_o <= '0';
    elsif rising_edge(clk_i) then
      if (ctrl_en = '0') then
        irq_o <= '0';
      elsif (arbiter.astop = '1') then -- trigger interrupt when reaching auto-stop-address
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

-- pragma translate_off
-- RTL_SYNTHESIS OFF

  -- list of all currently supported instructions --
  type inst_touple_c is record
    machine  : std_ulogic_vector(31 downto 0); -- instruction word
    mnemonic : string(1 to 11); -- according assembly mnemonic
  end record;
  type inst_t is array (0 to 191) of inst_touple_c;
  constant inst_c : inst_t := (
    ("-------------------------0110111", "lui        "), -- base ISA
    ("-------------------------0010111", "auipc      "),
    ("-------------------------1101111", "jal        "),
    ("-----------------000-----1100111", "jalr       "),
    ("-----------------000-----1100011", "beq        "),
    ("-----------------001-----1100011", "bne        "),
    ("-----------------100-----1100011", "blt        "),
    ("-----------------101-----1100011", "bge        "),
    ("-----------------110-----1100011", "bltu       "),
    ("-----------------111-----1100011", "bgeu       "),
    ("-----------------000-----0000011", "lb         "),
    ("-----------------001-----0000011", "lh         "),
    ("-----------------010-----0000011", "lw         "),
    ("-----------------100-----0000011", "lbu        "),
    ("-----------------101-----0000011", "lhu        "),
    ("-----------------000-----0100011", "sb         "),
    ("-----------------001-----0100011", "sh         "),
    ("-----------------010-----0100011", "sw         "),
    ("-----------------000-----0010011", "addi       "),
    ("-----------------010-----0010011", "slti       "),
    ("-----------------011-----0010011", "sltiu      "),
    ("-----------------100-----0010011", "xori       "),
    ("-----------------110-----0010011", "ori        "),
    ("-----------------111-----0010011", "andi       "),
    ("0000000----------001-----0010011", "slli       "),
    ("0000000----------101-----0010011", "srli       "),
    ("0100000----------101-----0010011", "srai       "),
    ("0000000----------000-----0110011", "add        "),
    ("0100000----------000-----0110011", "sub        "),
    ("0000000----------001-----0110011", "sll        "),
    ("0000000----------010-----0110011", "slt        "),
    ("0000000----------011-----0110011", "sltu       "),
    ("0000000----------100-----0110011", "xor        "),
    ("0000000----------101-----0110011", "srl        "),
    ("0100000----------101-----0110011", "sra        "),
    ("0000000----------110-----0110011", "or         "),
    ("0000000----------111-----0110011", "and        "),
    ("-----------------000-----0001111", "fence      "),
    ("00000000000000000000000001110011", "ecall      "),
    ("00000000000100000000000001110011", "ebreak     "),
    ("00010000010100000000000001110011", "wfi        "),
    ("00110000001000000000000001110011", "mret       "),
    ("01111011001000000000000001110011", "dret       "),
    ("-----------------010-----1100011", "beqi       "), -- Zibi
    ("-----------------011-----1100011", "bnei       "),
    ("-------------------------0001011", "custom0    "), -- custom instructions
    ("-------------------------0101011", "custom1    "),
    ("-------------------------1011011", "custom2    "),
    ("-------------------------1111011", "custom3    "),
    ("-----------------001-----0001111", "fence.i    "), -- Zifencei
    ("-----------------001-----1110011", "csrrw      "), -- Zicsr
    ("-----------------010-----1110011", "csrrs      "),
    ("-----------------011-----1110011", "csrrc      "),
    ("-----------------101-----1110011", "csrrwi     "),
    ("-----------------110-----1110011", "csrrsi     "),
    ("-----------------111-----1110011", "csrrci     "),
    ("100000011100-----100-----1110011", "mop.r.0    "), -- Zimop
    ("100000011101-----100-----1110011", "mop.r.1    "),
    ("100000011110-----100-----1110011", "mop.r.2    "),
    ("100000011111-----100-----1110011", "mop.r.3    "),
    ("100001011100-----100-----1110011", "mop.r.4    "),
    ("100001011101-----100-----1110011", "mop.r.5    "),
    ("100001011110-----100-----1110011", "mop.r.6    "),
    ("100001011111-----100-----1110011", "mop.r.7    "),
    ("100010011100-----100-----1110011", "mop.r.8    "),
    ("100010011101-----100-----1110011", "mop.r.9    "),
    ("100010011110-----100-----1110011", "mop.r.10   "),
    ("100010011111-----100-----1110011", "mop.r.11   "),
    ("100011011100-----100-----1110011", "mop.r.12   "),
    ("100011011101-----100-----1110011", "mop.r.13   "),
    ("100011011110-----100-----1110011", "mop.r.14   "),
    ("100011011111-----100-----1110011", "mop.r.15   "),
    ("110000011100-----100-----1110011", "mop.r.16   "),
    ("110000011101-----100-----1110011", "mop.r.17   "),
    ("110000011110-----100-----1110011", "mop.r.18   "),
    ("110000011111-----100-----1110011", "mop.r.19   "),
    ("110001011100-----100-----1110011", "mop.r.20   "),
    ("110001011101-----100-----1110011", "mop.r.21   "),
    ("110001011110-----100-----1110011", "mop.r.22   "),
    ("110001011111-----100-----1110011", "mop.r.23   "),
    ("110010011100-----100-----1110011", "mop.r.24   "),
    ("110010011101-----100-----1110011", "mop.r.25   "),
    ("110010011110-----100-----1110011", "mop.r.26   "),
    ("110010011111-----100-----1110011", "mop.r.27   "),
    ("110011011100-----100-----1110011", "mop.r.28   "),
    ("110011011101-----100-----1110011", "mop.r.29   "),
    ("110011011110-----100-----1110011", "mop.r.30   "),
    ("110011011111-----100-----1110011", "mop.r.31   "),
    ("1000001----------100-----1110011", "mop.rr.0   "),
    ("1000011----------100-----1110011", "mop.rr.1   "),
    ("1000101----------100-----1110011", "mop.rr.2   "),
    ("1000111----------100-----1110011", "mop.rr.3   "),
    ("1100001----------100-----1110011", "mop.rr.4   "),
    ("1100011----------100-----1110011", "mop.rr.5   "),
    ("1100101----------100-----1110011", "mop.rr.6   "),
    ("1100111----------100-----1110011", "mop.rr.7   "),
    ("0000111----------101-----0110011", "czero.eqz  "), -- Zicond
    ("0000111----------111-----0110011", "czero.nez  "),
    ("0000001----------000-----0110011", "mul        "), -- M / Zm*
    ("0000001----------001-----0110011", "mulh       "),
    ("0000001----------010-----0110011", "mulhsu     "),
    ("0000001----------011-----0110011", "mulh       "),
    ("0000001----------100-----0110011", "div        "),
    ("0000001----------101-----0110011", "divu       "),
    ("0000001----------110-----0110011", "rem        "),
    ("0000001----------111-----0110011", "remu       "),
    ("00010--00000-----010-----0101111", "lr.w       "), -- A / Za*
    ("00011------------010-----0101111", "sc.w       "),
    ("00001------------010-----0101111", "amoswap.w  "),
    ("00000------------010-----0101111", "amoadd.w   "),
    ("00100------------010-----0101111", "amoxor.w   "),
    ("01100------------010-----0101111", "amoand.w   "),
    ("01000------------010-----0101111", "amoor.w    "),
    ("10000------------010-----0101111", "amomin.w   "),
    ("10100------------010-----0101111", "amomax.w   "),
    ("11000------------010-----0101111", "amominu.w  "),
    ("11100------------010-----0101111", "amomaxu.w  "),
    ("0100000----------111-----0110011", "andn       "), -- B / Zb*
    ("011000000000-----001-----0010011", "clz        "),
    ("011000000010-----001-----0010011", "cpop       "),
    ("011000000001-----001-----0010011", "ctz        "),
    ("0000101----------110-----0110011", "max        "),
    ("0000101----------111-----0110011", "maxu       "),
    ("0000101----------100-----0110011", "min        "),
    ("0000101----------101-----0110011", "minu       "),
    ("001010000111-----101-----0010011", "orc.b      "),
    ("0100000----------110-----0110011", "orn        "),
    ("0000100----------100-----0110011", "pack       "),
    ("0000100----------111-----0110011", "packh      "),
    ("011010011000-----101-----0010011", "rev8       "),
    ("011010000111-----101-----0010011", "brev8      "),
    ("0110000----------001-----0110011", "rol        "),
    ("0110000----------101-----0110011", "ror        "),
    ("0110000----------101-----0010011", "rori       "),
    ("011000000100-----001-----0010011", "sext.b     "),
    ("011000000101-----001-----0010011", "sext.j     "),
    ("0010000----------010-----0110011", "sh1add     "),
    ("0010000----------100-----0110011", "sh2add     "),
    ("0010000----------110-----0110011", "sh3add     "),
    ("000010001111-----101-----0010011", "unzip      "),
    ("0100000----------100-----0110011", "xnor       "),
    ("000010000000-----100-----0110011", "zext.h     "),
    ("000010001111-----001-----0010011", "zip        "),
    ("0100100----------001-----0110011", "bclr       "),
    ("0100100----------001-----0010011", "bclri      "),
    ("0100100----------101-----0110011", "bext       "),
    ("0100100----------101-----0010011", "bexti      "),
    ("0110100----------001-----0110011", "binv       "),
    ("0110100----------001-----0010011", "binvi      "),
    ("0010100----------001-----0110011", "bset       "),
    ("0010100----------001-----0010011", "bseti      "),
    ("0000101----------001-----0110011", "clmul      "),
    ("0000101----------011-----0110011", "clmulh     "),
    ("0000101----------010-----0110011", "clmulr     "),
    ("--10101----------000-----0110011", "aes32dsi   "), -- Zk*
    ("--10111----------000-----0110011", "aes32dsmi  "),
    ("--10001----------000-----0110011", "aes32esi   "),
    ("--10011----------000-----0110011", "aes32esmi  "),
    ("000100000010-----001-----0010011", "sha256sig0 "),
    ("000100000011-----001-----0010011", "sha256sig1 "),
    ("000100000000-----001-----0010011", "sha256sum0 "),
    ("000100000001-----001-----0010011", "sha256sum1 "),
    ("0101110----------000-----0110011", "sha512sig0h"),
    ("0101010----------000-----0110011", "sha512sig0l"),
    ("0101111----------000-----0110011", "sha512sig1h"),
    ("0101011----------000-----0110011", "sha512sig1l"),
    ("0101000----------000-----0110011", "sha512sum0r"),
    ("0101001----------000-----0110011", "sha512sum1r"),
    ("000100001000-----001-----0010011", "sm3p0      "),
    ("000100001001-----001-----0010011", "sm3p1      "),
    ("--11000----------000-----0110011", "sm4ed      "),
    ("--11010----------000-----0110011", "sm4ks      "),
    ("0010100----------100-----0110011", "xperm8     "),
    ("0010100----------010-----0110011", "xperm4     "),
    ("0000000------------------1010011", "fadd.s     "), -- Zfinx
    ("0000100------------------1010011", "fsub.s     "),
    ("0001000------------------1010011", "fmul.s     "),
    ("0001100------------------1010011", "fdiv.s     "),
    ("010110000000-------------1010011", "fsqrt.s    "),
    ("0010000----------000-----1010011", "fsgnj.s    "),
    ("0010000----------001-----1010011", "fsgnjn.s   "),
    ("0010000----------010-----1010011", "fsgnjx.s   "),
    ("0010100----------000-----1010011", "fmin.s     "),
    ("0010100----------001-----1010011", "fmax.s     "),
    ("110000000000-------------1010011", "fcvt.w.s   "),
    ("110000000001-------------1010011", "fcvt.wu.s  "),
    ("1010000----------010-----1010011", "feq.s      "),
    ("1010000----------001-----1010011", "flt.s      "),
    ("1010000----------000-----1010011", "fle.s      "),
    ("111000000000-----001-----1010011", "fclass.s   "),
    ("110100000000-------------1010011", "fcvt.s.w   "),
    ("110100000001-------------1010011", "fcvt.s.wu  ")
  );

  -- decode instruction mnemonic --
  function decode_mnemonic_f(inst : std_ulogic_vector(31 downto 0)) return string is
  begin
    for i in inst_c'range loop
      if match_f(inst, inst_c(i).machine) then
        return inst_c(i).mnemonic;
      end if;
    end loop;
    return "INVALID";
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
      when others               => return "0x" & to_hexstring_f(addr);
    end case;
  end function decode_csr_f;

  -- decode instruction operands --
  function decode_operands_f(inst : std_ulogic_vector(31 downto 0)) return string is
    variable is_v, ib_v, iu_v, ij_v, ii_v : std_ulogic_vector(31 downto 0);
    variable rs1_iv, rs2_iv, rd_iv, is_iv, ib_iv, iu_iv, ij_iv, ii_iv : integer;
  begin
    -- registers --
    rs1_iv := to_integer(unsigned(inst(instr_rs1_msb_c downto instr_rs1_lsb_c)));
    rs2_iv := to_integer(unsigned(inst(instr_rs2_msb_c downto instr_rs2_lsb_c)));
    rd_iv  := to_integer(unsigned(inst(instr_rd_msb_c downto instr_rd_lsb_c)));
    -- immediates --
    is_v  := replicate_f(inst(31), 21) & inst(30 downto 25) & inst(11 downto 7);
    ib_v  := replicate_f(inst(31), 20) & inst(7) & inst(30 downto 25) & inst(11 downto 8) & '0';
    iu_v  := inst(31 downto 12) & x"000";
    ij_v  := replicate_f(inst(31), 12) & inst(19 downto 12) & inst(20) & inst(30 downto 21) & '0';
    ii_v  := replicate_f(inst(31), 21) & inst(30 downto 21) & inst(20);
    is_iv := to_integer(signed(is_v));
    ib_iv := to_integer(signed(ib_v));
    iu_iv := to_integer(signed(iu_v));
    ij_iv := to_integer(signed(ij_v));
    ii_iv := to_integer(signed(ii_v));
    -- instruction types --
    case inst(instr_opcode_msb_c downto instr_opcode_lsb_c) is
      when opcode_alui_c   => return "x" & integer'image(rd_iv)  & ", x"  & integer'image(rs1_iv) & ", "  & integer'image(ii_iv);
      when opcode_alu_c    => return "x" & integer'image(rd_iv)  & ", x"  & integer'image(rs1_iv) & ", x" & integer'image(rs2_iv);
      when opcode_lui_c    => return "x" & integer'image(rd_iv)  & ", 0x" & to_hexstring_f(iu_v);
      when opcode_auipc_c  => return "x" & integer'image(rd_iv)  & ", 0x" & to_hexstring_f(iu_v);
      when opcode_jal_c    => return "x" & integer'image(rd_iv)  & ", "   & integer'image(ij_iv);
      when opcode_jalr_c   => return "x" & integer'image(rd_iv)  & ", "   & integer'image(ii_iv)  & "(x"  & integer'image(rs1_iv) & ")";
      when opcode_branch_c => return "x" & integer'image(rs1_iv) & ", x"  & integer'image(rs2_iv) & ", "  & integer'image(is_iv);
      when opcode_load_c   => return "x" & integer'image(rd_iv)  & ", "   & integer'image(is_iv)  & "(x"  & integer'image(rs1_iv) & ")";
      when opcode_store_c  => return "x" & integer'image(rs2_iv) & ", "   & integer'image(is_iv)  & "(x"  & integer'image(rs1_iv) & ")";
      when opcode_amo_c    =>
        if (inst(28 downto 27) = "10") then -- zalrsc LR
          return "x" & integer'image(rd_iv) & ", (x" & integer'image(rs1_iv) & ")";
        elsif (inst(28 downto 27) = "11") then -- zalrsc SC
          return "x" & integer'image(rd_iv) & ", x" & integer'image(rs2_iv) & ", (x" & integer'image(rs1_iv) & ")";
        else -- zaamo
          return "x" & integer'image(rd_iv) & ", x" & integer'image(rs2_iv) & ", (x" & integer'image(rs1_iv) & ")";
        end if;
      when opcode_system_c =>
        if (inst(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_env_c) then -- environment
          return "";
        elsif (inst(instr_funct3_msb_c downto instr_funct3_lsb_c) = funct3_zimop_c) then -- zimop may-be-operations
          if (inst(25) = '0') then -- mop.r
            return "x" & integer'image(rd_iv) & ", x" & integer'image(rs1_iv);
          else -- mop.rr
            return "x" & integer'image(rd_iv) & ", x" & integer'image(rs1_iv) & ", x" & integer'image(rs2_iv);
          end if;
        elsif (inst(instr_funct3_msb_c) = '0') then -- csr-register
          return "x" & integer'image(rd_iv) & ", " & decode_csr_f(inst(31 downto 20)) & ", x" & integer'image(rs1_iv);
        else -- csr-immediate
          return "x" & integer'image(rd_iv) & ", " & decode_csr_f(inst(31 downto 20)) & ", " & integer'image(rs1_iv);
        end if;
      when others => return "";
    end case;
  end function decode_operands_f;

  -- time stamp counter --
  signal cycle_cnt : std_ulogic_vector(31 downto 0);

-- RTL_SYNTHESIS ON
-- pragma translate_on

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
        cycle_cnt <= (others => '0');
      elsif rising_edge(clk_i) then
        cycle_cnt <= std_ulogic_vector(unsigned(cycle_cnt) + 1);
        if (trace_i.valid = '1') then
          -- index --
          write(line_v, integer'(to_integer(unsigned(trace_i.order))));
          write(line_v, string'(" "));
          -- timestamp --
          write(line_v, integer'(to_integer(unsigned(cycle_cnt))));
          write(line_v, string'(" "));
          -- instruction address --
          write(line_v, string'("0x"));
          write(line_v, string'(to_hexstring_f(trace_i.pc_rdata)));
          write(line_v, string'(" "));
          -- instruction word --
          write(line_v, string'("0x"));
          write(line_v, string'(to_hexstring_f(trace_i.insn)));
          write(line_v, string'(" "));
          -- privilege level --
          if (trace_i.debug = '1') then
            write(line_v, string'("D "));
          elsif (trace_i.mode = "11") then
            write(line_v, string'("M "));
          elsif (trace_i.mode = "00") then
            write(line_v, string'("U "));
          else
            write(line_v, string'("? "));
          end if;
          -- decoded instruction --
          if (trace_i.compr = '1') then -- de-compressed instruction
            write(line_v, string'("c."));
          else
            write(line_v, string'("  "));
          end if;
          write(line_v, string'(decode_mnemonic_f(trace_i.insn)));
          write(line_v, string'(decode_operands_f(trace_i.insn)));
          -- trap entry --
          if (trace_i.intr = '1') then
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
