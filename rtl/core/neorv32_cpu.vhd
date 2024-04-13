-- ================================================================================ --
-- NEORV32 CPU - CPU Top Entity                                                     --
-- -------------------------------------------------------------------------------- --
-- Check out the processor's online documentation for more information:             --
--  HQ:         https://github.com/stnolting/neorv32                                --
--  Data Sheet: https://stnolting.github.io/neorv32                                 --
--  User Guide: https://stnolting.github.io/neorv32/ug                              --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cpu is
  generic (
    -- General --
    HART_ID                    : std_ulogic_vector(31 downto 0); -- hardware thread ID
    VENDOR_ID                  : std_ulogic_vector(31 downto 0); -- vendor's JEDEC ID
    CPU_BOOT_ADDR              : std_ulogic_vector(31 downto 0); -- cpu boot address
    CPU_DEBUG_PARK_ADDR        : std_ulogic_vector(31 downto 0); -- cpu debug mode parking loop entry address
    CPU_DEBUG_EXC_ADDR         : std_ulogic_vector(31 downto 0); -- cpu debug mode exception entry address
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A      : boolean; -- implement atomic memory operations extension?
    CPU_EXTENSION_RISCV_B      : boolean; -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_C      : boolean; -- implement compressed extension?
    CPU_EXTENSION_RISCV_E      : boolean; -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M      : boolean; -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U      : boolean; -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx  : boolean; -- implement 32-bit floating-point extension (using INT reg!)
    CPU_EXTENSION_RISCV_Zicntr : boolean; -- implement base counters?
    CPU_EXTENSION_RISCV_Zicond : boolean; -- implement integer conditional operations?
    CPU_EXTENSION_RISCV_Zihpm  : boolean; -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zmmul  : boolean; -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zxcfu  : boolean; -- implement custom (instr.) functions unit?
    CPU_EXTENSION_RISCV_Sdext  : boolean; -- implement external debug mode extension?
    CPU_EXTENSION_RISCV_Sdtrig : boolean; -- implement trigger module extension?
    CPU_EXTENSION_RISCV_Smpmp  : boolean; -- implement physical memory protection?
    -- Tuning Options --
    FAST_MUL_EN                : boolean; -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN              : boolean; -- use barrel shifter for shift operations
    REGFILE_HW_RST             : boolean; -- implement full hardware reset for register file
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS            : natural range 0 to 16; -- number of regions (0..16)
    PMP_MIN_GRANULARITY        : natural; -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
    PMP_TOR_MODE_EN            : boolean; -- implement TOR mode
    PMP_NAP_MODE_EN            : boolean; -- implement NAPOT/NA4 modes
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS               : natural range 0 to 13; -- number of implemented HPM counters (0..13)
    HPM_CNT_WIDTH              : natural range 0 to 64  -- total size of HPM counters (0..64)
  );
  port (
    -- global control --
    clk_i      : in  std_ulogic; -- switchable global clock, rising edge
    clk_aux_i  : in  std_ulogic; -- always-on clock, rising edge
    rstn_i     : in  std_ulogic; -- global reset, low-active, async
    sleep_o    : out std_ulogic; -- cpu is in sleep mode when set
    debug_o    : out std_ulogic; -- cpu is in debug mode when set
    -- interrupts --
    msi_i      : in  std_ulogic; -- risc-v machine software interrupt
    mei_i      : in  std_ulogic; -- risc-v machine external interrupt
    mti_i      : in  std_ulogic; -- risc-v machine timer interrupt
    firq_i     : in  std_ulogic_vector(15 downto 0); -- custom fast interrupts
    dbi_i      : in  std_ulogic; -- risc-v debug halt request interrupt
    -- instruction bus interface --
    ibus_req_o : out bus_req_t; -- request bus
    ibus_rsp_i : in  bus_rsp_t; -- response bus
    -- data bus interface --
    dbus_req_o : out bus_req_t; -- request bus
    dbus_rsp_i : in  bus_rsp_t  -- response bus
  );
end neorv32_cpu;

architecture neorv32_cpu_rtl of neorv32_cpu is

  -- auto-configuration --
  constant regfile_rs3_en_c : boolean := CPU_EXTENSION_RISCV_Zxcfu or CPU_EXTENSION_RISCV_Zfinx; -- 3rd register file read port (rs3)
  constant regfile_rs4_en_c : boolean := CPU_EXTENSION_RISCV_Zxcfu; -- 4th register file read port (rs4)

  -- control-unit-external CSR interface --
  signal xcsr_we        : std_ulogic;
  signal xcsr_addr      : std_ulogic_vector(11 downto 0);
  signal xcsr_wdata     : std_ulogic_vector(XLEN-1 downto 0);
  signal xcsr_rdata_pmp : std_ulogic_vector(XLEN-1 downto 0);
  signal xcsr_rdata_alu : std_ulogic_vector(XLEN-1 downto 0);
  signal xcsr_rdata_res : std_ulogic_vector(XLEN-1 downto 0);

  -- local signals --
  signal ctrl         : ctrl_bus_t; -- main control bus
  signal imm          : std_ulogic_vector(XLEN-1 downto 0); -- immediate
  signal rf_wdata     : std_ulogic_vector(XLEN-1 downto 0); -- register file write data
  signal rs1, rs2     : std_ulogic_vector(XLEN-1 downto 0); -- source registers 1 and 2
  signal rs3, rs4     : std_ulogic_vector(XLEN-1 downto 0); -- source registers 3 and 4 (optional)
  signal alu_res      : std_ulogic_vector(XLEN-1 downto 0); -- alu result
  signal alu_add      : std_ulogic_vector(XLEN-1 downto 0); -- alu address result
  signal alu_cmp      : std_ulogic_vector(1 downto 0); -- comparator result
  signal mem_rdata    : std_ulogic_vector(XLEN-1 downto 0); -- memory read data
  signal cp_done      : std_ulogic; -- ALU co-processor operation done
  signal lsu_wait     : std_ulogic; -- wait for current data bus access
  signal csr_rdata    : std_ulogic_vector(XLEN-1 downto 0); -- csr read data
  signal mar          : std_ulogic_vector(XLEN-1 downto 0); -- memory address register
  signal ma_load      : std_ulogic; -- misaligned load data address
  signal ma_store     : std_ulogic; -- misaligned store data address
  signal be_load      : std_ulogic; -- bus error on load data access
  signal be_store     : std_ulogic; -- bus error on store data access
  signal fetch_pc     : std_ulogic_vector(XLEN-1 downto 0); -- pc for instruction fetch
  signal curr_pc      : std_ulogic_vector(XLEN-1 downto 0); -- current pc (for currently executed instruction)
  signal link_pc      : std_ulogic_vector(XLEN-1 downto 0); -- link pc (return address)
  signal pmp_ex_fault : std_ulogic; -- PMP instruction fetch fault
  signal pmp_rw_fault : std_ulogic; -- PMP read/write access fault

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- CPU ISA configuration --
  assert false report "[NEORV32] CPU ISA: rv32" &
    cond_sel_string_f(CPU_EXTENSION_RISCV_E,      "e",         "i") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_M,      "m",         "" ) &
    cond_sel_string_f(CPU_EXTENSION_RISCV_A,      "a",         "" ) &
    cond_sel_string_f(CPU_EXTENSION_RISCV_C,      "c",         "" ) &
    cond_sel_string_f(CPU_EXTENSION_RISCV_B,      "b",         "" ) &
    cond_sel_string_f(CPU_EXTENSION_RISCV_U,      "u",         "" ) &
    cond_sel_string_f(true,                       "_zicsr",    "" ) & -- always enabled
    cond_sel_string_f(CPU_EXTENSION_RISCV_Zicntr, "_zicntr",   "" ) &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Zicond, "_zicond",   "" ) &
    cond_sel_string_f(true,                       "_zifencei", "" ) & -- always enabled
    cond_sel_string_f(CPU_EXTENSION_RISCV_Zfinx,  "_zfinx",    "" ) &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Zihpm,  "_zihpm",    "" ) &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Zmmul,  "_zmmul",    "" ) &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Zxcfu,  "_zxcfu",    "" ) &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Sdext,  "_sdext",    "" ) &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Sdtrig, "_sdtrig",   "" ) &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Smpmp,  "_smpmp",    "" )
    severity note;

  -- CPU tuning options --
  assert false report "[NEORV32] CPU tuning options: " &
    cond_sel_string_f(FAST_MUL_EN,    "fast_mul ",   "") &
    cond_sel_string_f(FAST_SHIFT_EN,  "fast_shift ", "") &
    cond_sel_string_f(REGFILE_HW_RST, "rf_hw_rst ",  "")
    severity note;

  -- simulation notifier --
  assert not (is_simulation_c = true) report "[NEORV32] Assuming this is a simulation." severity warning;


  -- Control Unit ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_control_inst: entity neorv32.neorv32_cpu_control
  generic map (
    -- General --
    HART_ID                    => HART_ID,                    -- hardware thread ID
    VENDOR_ID                  => VENDOR_ID,                  -- vendor's JEDEC ID
    CPU_BOOT_ADDR              => CPU_BOOT_ADDR,              -- cpu boot address
    CPU_DEBUG_PARK_ADDR        => CPU_DEBUG_PARK_ADDR,        -- cpu debug mode parking loop entry address
    CPU_DEBUG_EXC_ADDR         => CPU_DEBUG_EXC_ADDR,         -- cpu debug mode exception entry address
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A      => CPU_EXTENSION_RISCV_A,      -- implement atomic memory operations extension?
    CPU_EXTENSION_RISCV_B      => CPU_EXTENSION_RISCV_B,      -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_C      => CPU_EXTENSION_RISCV_C,      -- implement compressed extension?
    CPU_EXTENSION_RISCV_E      => CPU_EXTENSION_RISCV_E,      -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M      => CPU_EXTENSION_RISCV_M,      -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U      => CPU_EXTENSION_RISCV_U,      -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx  => CPU_EXTENSION_RISCV_Zfinx,  -- implement 32-bit floating-point extension (using INT reg!)
    CPU_EXTENSION_RISCV_Zicntr => CPU_EXTENSION_RISCV_Zicntr, -- implement base counters?
    CPU_EXTENSION_RISCV_Zicond => CPU_EXTENSION_RISCV_Zicond, -- implement integer conditional operations?
    CPU_EXTENSION_RISCV_Zihpm  => CPU_EXTENSION_RISCV_Zihpm,  -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zmmul  => CPU_EXTENSION_RISCV_Zmmul,  -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zxcfu  => CPU_EXTENSION_RISCV_Zxcfu,  -- implement custom (instr.) functions unit?
    CPU_EXTENSION_RISCV_Sdext  => CPU_EXTENSION_RISCV_Sdext,  -- implement external debug mode extension?
    CPU_EXTENSION_RISCV_Sdtrig => CPU_EXTENSION_RISCV_Sdtrig, -- implement trigger module extension?
    CPU_EXTENSION_RISCV_Smpmp  => CPU_EXTENSION_RISCV_Smpmp,  -- implement physical memory protection?
    -- Tuning Options --
    FAST_MUL_EN                => FAST_MUL_EN,                -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN              => FAST_SHIFT_EN,              -- use barrel shifter for shift operations
    REGFILE_HW_RST             => REGFILE_HW_RST,             -- implement full hardware reset for register file
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS               => HPM_NUM_CNTS,               -- number of implemented HPM counters (0..13)
    HPM_CNT_WIDTH              => HPM_CNT_WIDTH               -- total size of HPM counters
  )
  port map (
    -- global control --
    clk_i         => clk_i,          -- global clock, rising edge
    clk_aux_i     => clk_aux_i,      -- always-on clock, rising edge
    rstn_i        => rstn_i,         -- global reset, low-active, async
    ctrl_o        => ctrl,           -- main control bus
    -- instruction fetch interface --
    i_pmp_fault_i => pmp_ex_fault,   -- instruction fetch pmp fault
    bus_req_o     => ibus_req_o,     -- request
    bus_rsp_i     => ibus_rsp_i,     -- response
    -- data path interface --
    alu_cp_done_i => cp_done,        -- ALU iterative operation done
    cmp_i         => alu_cmp,        -- comparator status
    alu_add_i     => alu_add,        -- ALU address result
    rs1_i         => rs1,            -- rf source 1
    imm_o         => imm,            -- immediate
    fetch_pc_o    => fetch_pc,       -- instruction fetch address
    curr_pc_o     => curr_pc,        -- current PC (corresponding to current instruction)
    link_pc_o     => link_pc,        -- link PC (return address)
    csr_rdata_o   => csr_rdata,      -- CSR read data
    -- external CSR interface --
    xcsr_we_o     => xcsr_we,        -- global write enable
    xcsr_addr_o   => xcsr_addr,      -- address
    xcsr_wdata_o  => xcsr_wdata,     -- write data
    xcsr_rdata_i  => xcsr_rdata_res, -- read data
    -- interrupts --
    db_halt_req_i => dbi_i,          -- debug mode (halt) request
    msi_i         => msi_i,          -- machine software interrupt
    mei_i         => mei_i,          -- machine external interrupt
    mti_i         => mti_i,          -- machine timer interrupt
    firq_i        => firq_i,         -- fast interrupts
    -- data access interface --
    lsu_wait_i    => lsu_wait,       -- wait for data bus
    mar_i         => mar,            -- memory address register
    ma_load_i     => ma_load,        -- misaligned load data address
    ma_store_i    => ma_store,       -- misaligned store data address
    be_load_i     => be_load,        -- bus error on load data access
    be_store_i    => be_store        -- bus error on store data access
  );

  -- external CSR read-back --
  xcsr_rdata_res <= xcsr_rdata_pmp or xcsr_rdata_alu;

  -- CPU state --
  sleep_o <= ctrl.cpu_sleep; -- set when CPU is sleeping (after WFI)
  debug_o <= ctrl.cpu_debug; -- set when CPU is in debug mode


  -- Register File --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_regfile_inst: entity neorv32.neorv32_cpu_regfile
  generic map (
    RST_EN => REGFILE_HW_RST,        -- enable dedicated hardware reset ("ASIC style")
    RVE_EN => CPU_EXTENSION_RISCV_E, -- implement embedded RF extension
    RS3_EN => regfile_rs3_en_c,      -- enable 3rd read port
    RS4_EN => regfile_rs4_en_c       -- enable 4th read port
  )
  port map (
    -- global control --
    clk_i  => clk_i,     -- global clock, rising edge
    rstn_i => rstn_i,    -- global reset, low-active, async
    ctrl_i => ctrl,      -- main control bus
    -- operands --
    rd_i   => rf_wdata,  -- destination operand rd
    rs1_o  => rs1,       -- source operand rs1
    rs2_o  => rs2,       -- source operand rs2
    rs3_o  => rs3,       -- source operand rs3
    rs4_o  => rs4        -- source operand rs4
  );

  -- all buses are zero unless there is an according operation --
  rf_wdata <= alu_res or mem_rdata or csr_rdata or link_pc;


  -- ALU (Arithmetic/Logic Unit) and ALU Co-Processors --------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_alu_inst: entity neorv32.neorv32_cpu_alu
  generic map (
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_B      => CPU_EXTENSION_RISCV_B,      -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_M      => CPU_EXTENSION_RISCV_M,      -- implement mul/div extension?
    CPU_EXTENSION_RISCV_Zicond => CPU_EXTENSION_RISCV_Zicond, -- implement integer conditional operations?
    CPU_EXTENSION_RISCV_Zmmul  => CPU_EXTENSION_RISCV_Zmmul,  -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zfinx  => CPU_EXTENSION_RISCV_Zfinx,  -- implement 32-bit floating-point extension (using INT reg!)
    CPU_EXTENSION_RISCV_Zxcfu  => CPU_EXTENSION_RISCV_Zxcfu,  -- implement custom (instr.) functions unit?
    -- Tuning Options --
    FAST_MUL_EN                => FAST_MUL_EN,                -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN              => FAST_SHIFT_EN               -- use barrel shifter for shift operations
  )
  port map (
    -- global control --
    clk_i       => clk_i,          -- global clock, rising edge
    rstn_i      => rstn_i,         -- global reset, low-active, async
    ctrl_i      => ctrl,           -- main control bus
    -- CSR interface --
    csr_we_i    => xcsr_we,        -- global write enable
    csr_addr_i  => xcsr_addr,      -- address
    csr_wdata_i => xcsr_wdata,     -- write data
    csr_rdata_o => xcsr_rdata_alu, -- read data
    -- data input --
    rs1_i       => rs1,            -- rf source 1
    rs2_i       => rs2,            -- rf source 2
    rs3_i       => rs3,            -- rf source 3
    rs4_i       => rs4,            -- rf source 4
    pc_i        => curr_pc,        -- current PC
    imm_i       => imm,            -- immediate
    -- data output --
    cmp_o       => alu_cmp,        -- comparator status
    res_o       => alu_res,        -- ALU result
    add_o       => alu_add,        -- address computation result
    -- status --
    cp_done_o   => cp_done         -- iterative processing units done?
  );


  -- Load/Store Unit ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_lsu_inst: entity neorv32.neorv32_cpu_lsu
  generic map (
    AMO_LRSC_ENABLE => CPU_EXTENSION_RISCV_A -- enable atomic LR/SC operations
  )
  port map (
    -- global control --
    clk_i       => clk_i,        -- global clock, rising edge
    rstn_i      => rstn_i,       -- global reset, low-active, async
    ctrl_i      => ctrl,         -- main control bus
    -- cpu data access interface --
    addr_i      => alu_add,      -- access address
    wdata_i     => rs2,          -- write data
    rdata_o     => mem_rdata,    -- read data
    mar_o       => mar,          -- memory address register
    wait_o      => lsu_wait,     -- wait for access to complete
    ma_load_o   => ma_load,      -- misaligned load data address
    ma_store_o  => ma_store,     -- misaligned store data address
    be_load_o   => be_load,      -- bus error on load data access
    be_store_o  => be_store,     -- bus error on store data access
    pmp_fault_i => pmp_rw_fault, -- PMP read/write access fault
    -- data bus --
    bus_req_o   => dbus_req_o,   -- request
    bus_rsp_i   => dbus_rsp_i    -- response
  );


  -- Physical Memory Protection -------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  pmp_inst_true:
  if CPU_EXTENSION_RISCV_Smpmp generate
    neorv32_cpu_pmp_inst: entity neorv32.neorv32_cpu_pmp
    generic map (
      NUM_REGIONS => PMP_NUM_REGIONS,     -- number of regions (0..16)
      GRANULARITY => PMP_MIN_GRANULARITY, -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
      TOR_EN      => PMP_TOR_MODE_EN,     -- implement TOR mode
      NAP_EN      => PMP_NAP_MODE_EN      -- implement NAPOT/NA4 modes
    )
    port map (
      -- global control --
      clk_i       => clk_i,          -- global clock, rising edge
      rstn_i      => rstn_i,         -- global reset, low-active, async
      ctrl_i      => ctrl,           -- main control bus
      -- CSR interface --
      csr_we_i    => xcsr_we,        -- global write enable
      csr_addr_i  => xcsr_addr,      -- address
      csr_wdata_i => xcsr_wdata,     -- write data
      csr_rdata_o => xcsr_rdata_pmp, -- read data
      -- address input --
      addr_if_i   => fetch_pc,       -- instruction fetch address
      addr_ls_i   => alu_add,        -- load/store address
      -- faults --
      fault_ex_o  => pmp_ex_fault,   -- instruction fetch fault
      fault_rw_o  => pmp_rw_fault    -- read/write access fault
    );
  end generate;

  pmp_inst_false:
  if not CPU_EXTENSION_RISCV_Smpmp generate
    xcsr_rdata_pmp <= (others => '0');
    pmp_ex_fault   <= '0';
    pmp_rw_fault   <= '0';
  end generate;


end neorv32_cpu_rtl;
