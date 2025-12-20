-- ================================================================================ --
-- NEORV32 testbench for running the RISC-V architecture test framework             --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
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

entity rvtest_tb is
  generic (
    TEST_DIR : string  := "";          -- current test directory
    MEM_SIZE : natural := 4*1024*1024; -- main memory size in bytes
    TRACE_EN : boolean := false        -- enable trace logging
  );
end rvtest_tb;

architecture rvtest_tb_rtl of rvtest_tb is

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

  -- dump mem_t array cutout to ASCII HEX file --
  procedure mem_dump_f(mem : mem_t; file_name : string; offs_beg : natural; offs_end : natural) is
    file     file_f : text;
    variable idx_v  : natural;
    variable line_v : line;
  begin
    if (file_name /= "") then
      file_open(file_f, file_name, WRITE_MODE);
      idx_v := offs_beg;
      while (idx_v < offs_end) loop
        write(line_v, to_hexstring_f(to_stdulogicvector(mem(idx_v+3) & mem(idx_v+2) & mem(idx_v+1) & mem(idx_v+0))));
        writeline(file_f, line_v);
        idx_v := idx_v + 4;
      end loop;
      file_close(file_f);
    end if;
  end procedure mem_dump_f;

  -- generators --
  signal clk_gen, rst_gen : std_ulogic := '0';

  -- memory system --
  signal xbus_req : xbus_req_t;
  signal xbus_rsp : xbus_rsp_t;
  signal mem_addr : integer range 0 to MEM_SIZE-1;
  signal sig_beg, sig_end : std_ulogic_vector(31 downto 0);
  signal cpu_mei : std_ulogic;

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
    RISCV_ISA_Zcb       => true,
    RISCV_ISA_Zba       => true,
    RISCV_ISA_Zbb       => true,
    RISCV_ISA_Zbkb      => true,
    RISCV_ISA_Zbkc      => true,
    RISCV_ISA_Zbkx      => true,
    RISCV_ISA_Zbs       => true,
    RISCV_ISA_Zicntr    => true,
    RISCV_ISA_Zicond    => true,
    RISCV_ISA_Zimop     => true,
    RISCV_ISA_Zknd      => true,
    RISCV_ISA_Zkne      => true,
    RISCV_ISA_Zknh      => true,
    RISCV_ISA_Zksed     => true,
    RISCV_ISA_Zksh      => true,
    -- Tuning Options --
    CPU_FAST_MUL_EN     => true,
    CPU_FAST_SHIFT_EN   => true,
    -- Physical Memory Protection --
    PMP_NUM_REGIONS     => 16,
    PMP_MIN_GRANULARITY => 4,
    PMP_TOR_MODE_EN     => true,
    PMP_NAP_MODE_EN     => true,
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
    xbus_dat_i   => xbus_rsp.data,
    xbus_dat_o   => xbus_req.data,
    xbus_we_o    => xbus_req.we,
    xbus_sel_o   => xbus_req.sel,
    xbus_stb_o   => xbus_req.stb,
    xbus_cyc_o   => xbus_req.cyc,
    xbus_ack_i   => xbus_rsp.ack,
    xbus_err_i   => xbus_rsp.err,
    -- CPU Interrupts --
    irq_msi_i    => '0',
    irw_mti_i    => '0',
    irq_mei_i    => cpu_mei
  );


  -- Trace Log Generator --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  sim_trace_enabled:
  if TRACE_EN generate
    neorv32_cpu_trace_simlog_inst: entity neorv32.neorv32_cpu_trace_simlog
    generic map (
      LOG_FILE => TEST_DIR & "trace.log"
    )
    port map (
      clk_i   => clk_gen,
      rstn_i  => rst_gen,
      trace_i => trace_cpu0
    );
  end generate;


  -- Memory System --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  memory_system: process(rst_gen, clk_gen)
    variable mem_v : mem_t := mem_init_f(TEST_DIR & "main.bin");
  begin
    if (rst_gen = '0') then
      xbus_rsp <= xbus_rsp_terminate_c;
      sig_beg  <= (others => '0');
      sig_end  <= (others => '0');
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
        -- Signature layout
        -- ----------------------------------------
        elsif (xbus_req.addr = x"F0000000") then -- start address
          if (xbus_req.we = '1') then
            xbus_rsp.ack <= '1';
            sig_beg <= xbus_req.data;
          end if;
        elsif (xbus_req.addr = x"F0000004") then -- end address
          if (xbus_req.we = '1') then
            xbus_rsp.ack <= '1';
            sig_end <= xbus_req.data;
          end if;
        -- ----------------------------------------
        -- Dump signature and terminate simulation
        -- ----------------------------------------
        elsif (xbus_req.addr = x"F0000008") then
          if (xbus_req.we = '1') then
            xbus_rsp.ack <= '1';
            mem_dump_f(
              mem_v,
              TEST_DIR & "DUT-neorv32.signature",
              to_integer(unsigned(sig_beg(mem_abits_c-1 downto 0))),
              to_integer(unsigned(sig_end(mem_abits_c-1 downto 0)))
            );
            finish;
          end if;
        -- ----------------------------------------
        -- External machine interrupt trigger
        -- ----------------------------------------
        elsif (xbus_req.addr = x"F000000C") then
          if (xbus_req.we = '1') then
            xbus_rsp.ack <= '1';
            cpu_mei <= xbus_req.data(11);
          end if;
        end if;
      end if;
    end if;
  end process memory_system;

  -- memory access address --
  mem_addr <= to_integer(unsigned(xbus_req.addr(mem_abits_c-1 downto 2) & "00"));

end rvtest_tb_rtl;
