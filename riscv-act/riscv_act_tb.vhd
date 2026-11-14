-- ================================================================================ --
-- RISC-V Architectural Certification Tests (ACTs) Testbench                        --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) Stephan Nolting. All rights reserved.                              --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library std;
use std.textio.all;
use std.env.finish;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity riscv_act_tb is
  generic (
    TEST_BIN  : string  := "";       -- path + file of test memory image (flattened ELF binary)
    TRACE_EN  : boolean := false;    -- enable trace logging (disabled by default)
    TRACE_LOG : string  := "";       -- path + file of trace log; no logging when empty
    MEM_SIZE  : natural := 1024*1024 -- main memory size in bytes (max 4MB)
  );
end;

architecture riscv_act_tb_rtl of riscv_act_tb is

  -- main memory (bit_vector type for optimized system storage) --
  type mem_t is array (0 to MEM_SIZE-1) of bit_vector(7 downto 0);
  constant mem_base_c  : std_ulogic_vector(31 downto 0) := x"80000000";
  constant mem_abits_c : natural := index_size_f(MEM_SIZE);

  -- initialize mem_t array from binary file file (max MEM_SIZE) --
  impure function mem_init_f(file_name : string) return mem_t is
    type     file_t is file of character;
    file     file_f : file_t;
    variable mem_v  : mem_t;
    variable idx_v  : natural;
    variable tmp_v  : character;
  begin
    mem_v := (others => (others => '0'));
    idx_v := 0;
    if (file_name /= "") then
      file_open(file_f, file_name, READ_MODE);
      while (not endfile(file_f)) loop
        read(file_f, tmp_v);
        mem_v(idx_v) := to_bitvector(std_ulogic_vector(to_unsigned(character'pos(tmp_v), 8)));
        idx_v := idx_v + 1;
      end loop;
      file_close(file_f);
    end if;
    return mem_v;
  end function mem_init_f;

  -- generators --
  signal clk_gen, rst_gen : std_ulogic := '0';

  -- memory/IO system --
  signal xbus_req : xbus_req_t;
  signal xbus_rsp : xbus_rsp_t;
  signal mem_addr : integer range 0 to MEM_SIZE-1;
  signal cpu_mei  : std_ulogic;

  -- misc --
  signal trace_cpu0 : trace_port_t;

begin

  -- Clock and Reset Generators -------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  clk_gen <= not clk_gen after 5 ns;
  rst_gen <= '0', '1' after 100 ns;

  -- The Core of the Problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_top_inst: neorv32_top
  generic map (
    -- Processor Clocking --
    CLOCK_FREQUENCY     => 100_000_000,
    TRACE_PORT_EN       => true,
    -- Boot Configuration --
    BOOT_MODE_SELECT    => 1, -- boot from BOOT_ADDR_CUSTOM
    BOOT_ADDR_CUSTOM    => mem_base_c,
    -- RISC-V CPU Extensions --
    RISCV_ISA_C         => true,
    RISCV_ISA_M         => true,
    RISCV_ISA_U         => true,
    RISCV_ISA_Zaamo     => true,
    RISCV_ISA_Zalrsc    => true,
    RISCV_ISA_Zba       => true,
    RISCV_ISA_Zbb       => true,
    RISCV_ISA_Zbc       => true,
    RISCV_ISA_Zbkb      => true,
    RISCV_ISA_Zbkc      => true,
    RISCV_ISA_Zbkx      => true,
    RISCV_ISA_Zbs       => true,
    RISCV_ISA_Zcb       => true,
    RISCV_ISA_Zcmop     => true,
    RISCV_ISA_Zibi      => true,
    RISCV_ISA_Zicntr    => true,
    RISCV_ISA_Zicond    => true,
    RISCV_ISA_Zihpm     => true,
    RISCV_ISA_Zimop     => true,
    RISCV_ISA_Zknd      => true,
    RISCV_ISA_Zkne      => true,
    RISCV_ISA_Zknh      => true,
    RISCV_ISA_Zksed     => true,
    RISCV_ISA_Zksh      => true,
    RISCV_ISA_Smcntrpmf => true,
    -- Tuning Options --
    CPU_FAST_MUL_EN     => true,
    CPU_FAST_SHIFT_EN   => true,
    -- Physical Memory Protection --
    PMP_NUM_REGIONS     => 16,
    PMP_MIN_GRANULARITY => 4,
    PMP_TOR_MODE_EN     => true,
    PMP_NAP_MODE_EN     => true,
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS        => 29,
    HPM_CNT_WIDTH       => 64,
    -- Internal memories --
    IMEM_EN             => false,
    DMEM_EN             => false,
    -- External bus interface --
    XBUS_EN             => true,
    XBUS_REGSTAGE_EN    => false,
    -- Processor peripherals --
    IO_CLINT_EN         => true
  )
  port map (
    -- Global control --
    clk_i        => clk_gen,
    rstn_i       => rst_gen,
    -- Execution trace --
    trace_cpu0_o => trace_cpu0,
    -- External bus interface --
    xbus_adr_o   => xbus_req.addr,
    xbus_dat_o   => xbus_req.data,
    xbus_cti_o   => xbus_req.cti,
    xbus_tag_o   => xbus_req.tag,
    xbus_we_o    => xbus_req.we,
    xbus_sel_o   => xbus_req.sel,
    xbus_stb_o   => xbus_req.stb,
    xbus_cyc_o   => xbus_req.cyc,
    xbus_dat_i   => xbus_rsp.data,
    xbus_ack_i   => xbus_rsp.ack,
    xbus_err_i   => xbus_rsp.err,
    -- CPU Interrupts --
    irq_msi_i    => '0',
    irq_mti_i    => '0',
    irq_mei_i    => cpu_mei
  );

  -- Trace Logger ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  trace_enable:
  if TRACE_EN and (TRACE_LOG /= "") generate
    neorv32_cpu_trace_simlog_inst: entity neorv32.neorv32_cpu_trace_simlog
    generic map (
      LOG_FILE => TRACE_LOG
    )
    port map (
      clk_i   => clk_gen,
      rstn_i  => rst_gen,
      trace_i => trace_cpu0
    );
  end generate;

  -- Memory/IO System -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  subsystem: process(rst_gen, clk_gen)
    variable mem_v  : mem_t := mem_init_f(TEST_BIN);
    variable char_v : integer;
    variable line_v : line;
  begin
    if (rst_gen = '0') then
      xbus_rsp <= xbus_rsp_terminate_c;
      cpu_mei  <= '0';
    elsif rising_edge(clk_gen) then
      xbus_rsp <= xbus_rsp_terminate_c; -- default bus response
      if (xbus_req.cyc = '1') and (xbus_req.stb = '1') then
        -- ----------------------------------------
        -- Main memory access
        -- ----------------------------------------
        if (xbus_req.addr(31 downto 28) = mem_base_c(31 downto 28)) then
          xbus_rsp.ack <= '1';
          for i in 0 to 3 loop
            if (xbus_req.sel(i) = '1') then
              if (xbus_req.we = '1') then
                mem_v(mem_addr+i) := to_bitvector(xbus_req.data(i*8+7 downto i*8));
              else
                xbus_rsp.data(i*8+7 downto i*8) <= to_stdulogicvector(mem_v(mem_addr+i));
              end if;
            end if;
          end loop;
        -- ----------------------------------------
        -- Simulation control: halt
        -- ----------------------------------------
        elsif (xbus_req.addr = x"F0000000") then
          xbus_rsp.ack <= '1';
          if (xbus_req.we = '1') then
            if (xbus_req.data = x"12345678") then
              report "TEST PASS!" severity note;
            else
              report "TEST FAIL!" severity error;
            end if;
            finish;
          end if;
        -- ----------------------------------------
        -- Console output
        -- ----------------------------------------
        elsif (xbus_req.addr = x"F0000004") then
          xbus_rsp.ack <= '1';
          if (xbus_req.we = '1') then
            char_v := to_integer(unsigned(xbus_req.data(7 downto 0)));
            if (char_v >= 128) then -- printable?
              char_v := 0;
            end if;
            if (char_v /= 10) and (char_v /= 13) then -- skip line breaks
              write(line_v, character'val(char_v));
            elsif (char_v = 10) then -- flush line to screen on LF
              writeline(output, line_v);
            end if;
          end if;
        -- ----------------------------------------
        -- External machine interrupt
        -- ----------------------------------------
        elsif (xbus_req.addr = x"F0000008") then
          xbus_rsp.ack <= '1';
          if (xbus_req.we = '1') then
            cpu_mei <= xbus_req.data(0);
          end if;
        end if;
      end if;
    end if;
  end process subsystem;

  -- memory access address --
  mem_addr <= to_integer(unsigned(xbus_req.addr(mem_abits_c-1 downto 2) & "00"));

end;
