-- #################################################################################################
-- # << NEORV32 - CPU Top Entity >>                                                                #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
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
-- # The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32       (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cpu is
  generic (
    -- General --
    HART_ID                      : std_ulogic_vector(31 downto 0); -- hardware thread ID
    VENDOR_ID                    : std_ulogic_vector(31 downto 0); -- vendor's JEDEC ID
    CPU_BOOT_ADDR                : std_ulogic_vector(31 downto 0); -- cpu boot address
    CPU_DEBUG_PARK_ADDR          : std_ulogic_vector(31 downto 0); -- cpu debug mode parking loop entry address
    CPU_DEBUG_EXC_ADDR           : std_ulogic_vector(31 downto 0); -- cpu debug mode exception entry address
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        : boolean; -- implement atomic memory operations extension?
    CPU_EXTENSION_RISCV_B        : boolean; -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_C        : boolean; -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        : boolean; -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        : boolean; -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U        : boolean; -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    : boolean; -- implement 32-bit floating-point extension (using INT reg!)
    CPU_EXTENSION_RISCV_Zicntr   : boolean; -- implement base counters?
    CPU_EXTENSION_RISCV_Zihpm    : boolean; -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zifencei : boolean; -- implement instruction stream sync.?
    CPU_EXTENSION_RISCV_Zmmul    : boolean; -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zxcfu    : boolean; -- implement custom (instr.) functions unit?
    CPU_EXTENSION_RISCV_Sdext    : boolean; -- implement external debug mode extension?
    CPU_EXTENSION_RISCV_Sdtrig   : boolean; -- implement trigger module extension?
    -- Tuning Options --
    FAST_MUL_EN                  : boolean; -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                : boolean; -- use barrel shifter for shift operations
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS              : natural range 0 to 16; -- number of regions (0..16)
    PMP_MIN_GRANULARITY          : natural; -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 : natural range 0 to 13; -- number of implemented HPM counters (0..13)
    HPM_CNT_WIDTH                : natural range 0 to 64  -- total size of HPM counters (0..64)
  );
  port (
    -- global control --
    clk_i      : in  std_ulogic; -- global clock, rising edge
    rstn_i     : in  std_ulogic; -- global reset, low-active, async
    sleep_o    : out std_ulogic; -- cpu is in sleep mode when set
    debug_o    : out std_ulogic; -- cpu is in debug mode when set
    ifence_o   : out std_ulogic; -- instruction fence
    dfence_o   : out std_ulogic; -- data fence
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
  constant pmp_enable_c     : boolean := boolean(PMP_NUM_REGIONS > 0);

  -- external CSR interface --
  signal xcsr_we        : std_ulogic;
  signal xcsr_addr      : std_ulogic_vector(11 downto 0);
  signal xcsr_wdata     : std_ulogic_vector(XLEN-1 downto 0);
  signal xcsr_rdata_pmp : std_ulogic_vector(XLEN-1 downto 0);
  signal xcsr_rdata_alu : std_ulogic_vector(XLEN-1 downto 0);
  signal xcsr_rdata_res : std_ulogic_vector(XLEN-1 downto 0);

  -- local signals --
  signal ctrl         : ctrl_bus_t; -- main control bus
  signal imm          : std_ulogic_vector(XLEN-1 downto 0); -- immediate
  signal rs1          : std_ulogic_vector(XLEN-1 downto 0); -- source register 1
  signal rs2          : std_ulogic_vector(XLEN-1 downto 0); -- source register 2
  signal rs3          : std_ulogic_vector(XLEN-1 downto 0); -- source register 3
  signal rs4          : std_ulogic_vector(XLEN-1 downto 0); -- source register 4
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
  signal next_pc      : std_ulogic_vector(XLEN-1 downto 0); -- next pc (for next executed instruction)
  signal pmp_ex_fault : std_ulogic; -- PMP instruction fetch fault
  signal pmp_rd_fault : std_ulogic; -- PMP read fault
  signal pmp_wr_fault : std_ulogic; -- PMP write fault

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- say hello --
  assert false report
    "The NEORV32 RISC-V Processor, Version 0x" & to_hstring32_f(hw_version_c) & " - github.com/stnolting/neorv32" severity note;

  -- CPU ISA configuration --
  assert false report
    "NEORV32 CPU Configuration: RV32" &
    cond_sel_string_f(CPU_EXTENSION_RISCV_E,        "E", "I") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_M,        "M", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_A,        "A", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_C,        "C", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_B,        "B", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_U,        "U", "") &
    cond_sel_string_f(true,                         "_Zicsr", "") & -- always enabled
    cond_sel_string_f(CPU_EXTENSION_RISCV_Zicntr,   "_Zicntr", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Zifencei, "_Zifencei", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Zfinx,    "_Zfinx", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Zihpm,    "_Zihpm", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Zmmul,    "_Zmmul", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Zxcfu,    "_Zxcfu", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Sdext,    "_Sdext", "") &
    cond_sel_string_f(CPU_EXTENSION_RISCV_Sdtrig,   "_Sdtrig", "") &
    cond_sel_string_f(pmp_enable_c,                 "_Smpmp", "")
    severity note;

  -- simulation notifier --
  assert not (is_simulation_c = true) report
    "NEORV32 CPU WARNING! Assuming this is a simulation." severity warning;

  -- CPU boot address --
  assert not (CPU_BOOT_ADDR(1 downto 0) /= "00") report
    "NEORV32 CPU CONFIG ERROR! <CPU_BOOT_ADDR> has to be 32-bit aligned." severity error;

  -- Hardware multiplier extensions --
  assert not ((CPU_EXTENSION_RISCV_Zmmul = true) and (CPU_EXTENSION_RISCV_M = true)) report
    "NEORV32 CPU CONFIG ERROR! <M> and <Zmmul> extensions cannot co-exist!" severity error;

  -- Debug mode --
  assert not ((CPU_EXTENSION_RISCV_Sdext = true) and (CPU_EXTENSION_RISCV_Zifencei = false)) report
    "NEORV32 CPU CONFIG ERROR! Debug mode requires <CPU_EXTENSION_RISCV_Zifencei> extension to be enabled." severity error;


  -- Control Unit ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_control_inst: entity neorv32.neorv32_cpu_control
  generic map (
    -- General --
    HART_ID                      => HART_ID,                      -- hardware thread ID
    VENDOR_ID                    => VENDOR_ID,                    -- vendor's JEDEC ID
    CPU_BOOT_ADDR                => CPU_BOOT_ADDR,                -- cpu boot address
    CPU_DEBUG_PARK_ADDR          => CPU_DEBUG_PARK_ADDR,          -- cpu debug mode parking loop entry address
    CPU_DEBUG_EXC_ADDR           => CPU_DEBUG_EXC_ADDR,           -- cpu debug mode exception entry address
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        => CPU_EXTENSION_RISCV_A,        -- implement atomic memory operations extension?
    CPU_EXTENSION_RISCV_B        => CPU_EXTENSION_RISCV_B,        -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_C        => CPU_EXTENSION_RISCV_C,        -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        => CPU_EXTENSION_RISCV_E,        -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        => CPU_EXTENSION_RISCV_M,        -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U        => CPU_EXTENSION_RISCV_U,        -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    => CPU_EXTENSION_RISCV_Zfinx,    -- implement 32-bit floating-point extension (using INT reg!)
    CPU_EXTENSION_RISCV_Zicntr   => CPU_EXTENSION_RISCV_Zicntr,   -- implement base counters?
    CPU_EXTENSION_RISCV_Zihpm    => CPU_EXTENSION_RISCV_Zihpm,    -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zifencei => CPU_EXTENSION_RISCV_Zifencei, -- implement instruction stream sync.?
    CPU_EXTENSION_RISCV_Zmmul    => CPU_EXTENSION_RISCV_Zmmul,    -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zxcfu    => CPU_EXTENSION_RISCV_Zxcfu,    -- implement custom (instr.) functions unit?
    CPU_EXTENSION_RISCV_Sdext    => CPU_EXTENSION_RISCV_Sdext,    -- implement external debug mode extension?
    CPU_EXTENSION_RISCV_Sdtrig   => CPU_EXTENSION_RISCV_Sdtrig,   -- implement trigger module extension?
    -- Tuning Options --
    FAST_MUL_EN                  => FAST_MUL_EN,                  -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                => FAST_SHIFT_EN,                -- use barrel shifter for shift operations
    -- Physical memory protection (PMP) --
    PMP_EN                       => pmp_enable_c,                 -- physical memory protection enabled
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 => HPM_NUM_CNTS,                 -- number of implemented HPM counters (0..13)
    HPM_CNT_WIDTH                => HPM_CNT_WIDTH                 -- total size of HPM counters
  )
  port map (
    -- global control --
    clk_i         => clk_i,          -- global clock, rising edge
    rstn_i        => rstn_i,         -- global reset, low-active, async
    ctrl_o        => ctrl,           -- main control bus
    -- instruction fetch interface --
    bus_req_o     => ibus_req_o,     -- request
    bus_rsp_i     => ibus_rsp_i,     -- response
    -- status input --
    i_pmp_fault_i => pmp_ex_fault,   -- instruction fetch pmp fault
    alu_cp_done_i => cp_done,        -- ALU iterative operation done
    lsu_wait_i    => lsu_wait,       -- wait for data bus
    cmp_i         => alu_cmp,        -- comparator status
    -- data input --
    alu_add_i     => alu_add,        -- ALU address result
    rs1_i         => rs1,            -- rf source 1
    -- data output --
    imm_o         => imm,            -- immediate
    fetch_pc_o    => fetch_pc,       -- instruction fetch address
    curr_pc_o     => curr_pc,        -- current PC (corresponding to current instruction)
    next_pc_o     => next_pc,        -- next PC (corresponding to next instruction)
    csr_rdata_o   => csr_rdata,      -- CSR read data
    -- external CSR interface --
    xcsr_we_o     => xcsr_we,        -- global write enable
    xcsr_addr_o   => xcsr_addr,      -- address
    xcsr_wdata_o  => xcsr_wdata,     -- write data
    xcsr_rdata_i  => xcsr_rdata_res, -- read data
    -- debug mode (halt) request --
    db_halt_req_i => dbi_i,
    -- interrupts (risc-v compliant) --
    msi_i         => msi_i,          -- machine software interrupt
    mei_i         => mei_i,          -- machine external interrupt
    mti_i         => mti_i,          -- machine timer interrupt
    -- fast interrupts (custom) --
    firq_i        => firq_i,         -- fast interrupt trigger
    -- bus access exceptions --
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

  -- instruction/data fence --
  ifence_o <= ctrl.lsu_fencei;
  dfence_o <= ctrl.lsu_fence;


  -- Register File --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_regfile_inst: entity neorv32.neorv32_cpu_regfile
  generic map (
    RVE    => CPU_EXTENSION_RISCV_E, -- implement embedded RF extension?
    RS3_EN => regfile_rs3_en_c,      -- enable 3rd read port
    RS4_EN => regfile_rs4_en_c       -- enable 4th read port
  )
  port map (
    -- global control --
    clk_i  => clk_i,     -- global clock, rising edge
    ctrl_i => ctrl,      -- main control bus
    -- data input --
    alu_i  => alu_res,   -- ALU result
    mem_i  => mem_rdata, -- memory read data
    csr_i  => csr_rdata, -- CSR read data
    pc2_i  => next_pc,   -- next PC
    -- data output --
    rs1_o  => rs1,       -- rs1
    rs2_o  => rs2,       -- rs2
    rs3_o  => rs3,       -- rs3
    rs4_o  => rs4        -- rs4
  );


  -- ALU (Arithmetic/Logic Unit) and ALU Co-Processors --------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_alu_inst: entity neorv32.neorv32_cpu_alu
  generic map (
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_B     => CPU_EXTENSION_RISCV_B,     -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_M     => CPU_EXTENSION_RISCV_M,     -- implement mul/div extension?
    CPU_EXTENSION_RISCV_Zmmul => CPU_EXTENSION_RISCV_Zmmul, -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zfinx => CPU_EXTENSION_RISCV_Zfinx, -- implement 32-bit floating-point extension (using INT reg!)
    CPU_EXTENSION_RISCV_Zxcfu => CPU_EXTENSION_RISCV_Zxcfu, -- implement custom (instr.) functions unit?
    -- Tuning Options --
    FAST_MUL_EN               => FAST_MUL_EN,               -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN             => FAST_SHIFT_EN              -- use barrel shifter for shift operations
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
    clk_i         => clk_i,        -- global clock, rising edge
    rstn_i        => rstn_i,       -- global reset, low-active, async
    ctrl_i        => ctrl,         -- main control bus
    -- cpu data access interface --
    addr_i        => alu_add,      -- access address
    wdata_i       => rs2,          -- write data
    rdata_o       => mem_rdata,    -- read data
    mar_o         => mar,          -- memory address register
    wait_o        => lsu_wait,     -- wait for access to complete
    ma_load_o     => ma_load,      -- misaligned load data address
    ma_store_o    => ma_store,     -- misaligned store data address
    be_load_o     => be_load,      -- bus error on load data access
    be_store_o    => be_store,     -- bus error on store data access
    pmp_r_fault_i => pmp_rd_fault, -- PMP read fault
    pmp_w_fault_i => pmp_wr_fault, -- PMP write fault
    -- data bus --
    bus_req_o     => dbus_req_o,   -- request
    bus_rsp_i     => dbus_rsp_i    -- response
  );


  -- Physical Memory Protection -------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  pmp_inst_true:
  if (pmp_enable_c = true) generate
    neorv32_cpu_pmp_inst: entity neorv32.neorv32_cpu_pmp
    generic map (
      NUM_REGIONS => PMP_NUM_REGIONS,    -- number of regions (0..16)
      GRANULARITY => PMP_MIN_GRANULARITY -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
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
      fault_rd_o  => pmp_rd_fault,   -- data load fault
      fault_wr_o  => pmp_wr_fault    -- data store fault
    );
  end generate;

  pmp_inst_false:
  if (pmp_enable_c = false) generate
    xcsr_rdata_pmp <= (others => '0');
    pmp_ex_fault    <= '0';
    pmp_rd_fault    <= '0';
    pmp_wr_fault    <= '0';
  end generate;


end neorv32_cpu_rtl;
