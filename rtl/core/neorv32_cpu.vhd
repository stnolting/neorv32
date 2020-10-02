-- #################################################################################################
-- # << NEORV32 - CPU Top Entity >>                                                                #
-- # ********************************************************************************************* #
-- # NEORV32 CPU:                                                                                  #
-- # * neorv32_cpu.vhd                  : CPU top entity                                           #
-- #   * neorv32_cpu_alu.vhd            : Arithmetic/logic unit                                    #
-- #   * neorv32_cpu_bus.vhd            : Instruction and data bus interface unit                  #
-- #   * neorv32_cpu_cp_muldiv.vhd      : MULDIV co-processor                                      #
-- #   * neorv32_cpu_ctrl.vhd           : CPU control and CSR system                               #
-- #     * neorv32_cpu_decompressor.vhd : Compressed instructions decoder                          #
-- #   * neorv32_cpu_regfile.vhd        : Data register file                                       #
-- #                                                                                               #
-- # Check the processor's documentary for more information: docs/NEORV32.pdf                      #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2020, Stephan Nolting. All rights reserved.                                     #
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
-- # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cpu is
  generic (
    -- General --
    HW_THREAD_ID                 : std_ulogic_vector(31 downto 0):= (others => '0'); -- hardware thread id
    CPU_BOOT_ADDR                : std_ulogic_vector(31 downto 0):= (others => '0'); -- cpu boot address
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_C        : boolean := false; -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        : boolean := false; -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        : boolean := false; -- implement muld/div extension?
    CPU_EXTENSION_RISCV_U        : boolean := false; -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zicsr    : boolean := true;  -- implement CSR system?
    CPU_EXTENSION_RISCV_Zifencei : boolean := true;  -- implement instruction stream sync.?
    -- Extension Options --
    FAST_MUL_EN                  : boolean := false; -- use DSPs for M extension's multiplier
    -- Physical Memory Protection (PMP) --
    PMP_USE                      : boolean := false; -- implement PMP?
    PMP_NUM_REGIONS              : natural := 4;     -- number of regions (max 8)
    PMP_GRANULARITY              : natural := 14;    -- minimal region granularity (1=8B, 2=16B, 3=32B, ...) default is 64k
    -- Bus Interface --
    BUS_TIMEOUT                  : natural := 15     -- cycles after which a valid bus access will timeout
  );
  port (
    -- global control --
    clk_i          : in  std_ulogic := '0'; -- global clock, rising edge
    rstn_i         : in  std_ulogic := '0'; -- global reset, low-active, async
    -- instruction bus interface --
    i_bus_addr_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
    i_bus_rdata_i  : in  std_ulogic_vector(data_width_c-1 downto 0) := (others => '0'); -- bus read data
    i_bus_wdata_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
    i_bus_ben_o    : out std_ulogic_vector(03 downto 0); -- byte enable
    i_bus_we_o     : out std_ulogic; -- write enable
    i_bus_re_o     : out std_ulogic; -- read enable
    i_bus_cancel_o : out std_ulogic; -- cancel current bus transaction
    i_bus_ack_i    : in  std_ulogic := '0'; -- bus transfer acknowledge
    i_bus_err_i    : in  std_ulogic := '0'; -- bus transfer error
    i_bus_fence_o  : out std_ulogic; -- executed FENCEI operation
    -- data bus interface --
    d_bus_addr_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
    d_bus_rdata_i  : in  std_ulogic_vector(data_width_c-1 downto 0) := (others => '0'); -- bus read data
    d_bus_wdata_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
    d_bus_ben_o    : out std_ulogic_vector(03 downto 0); -- byte enable
    d_bus_we_o     : out std_ulogic; -- write enable
    d_bus_re_o     : out std_ulogic; -- read enable
    d_bus_cancel_o : out std_ulogic; -- cancel current bus transaction
    d_bus_ack_i    : in  std_ulogic := '0'; -- bus transfer acknowledge
    d_bus_err_i    : in  std_ulogic := '0'; -- bus transfer error
    d_bus_fence_o  : out std_ulogic; -- executed FENCE operation
    -- system time input from MTIME --
    time_i         : in  std_ulogic_vector(63 downto 0) := (others => '0'); -- current system time
    -- interrupts (risc-v compliant) --
    msw_irq_i      : in  std_ulogic := '0'; -- machine software interrupt
    mext_irq_i     : in  std_ulogic := '0'; -- machine external interrupt
    mtime_irq_i    : in  std_ulogic := '0'; -- machine timer interrupt
    -- fast interrupts (custom) --
    firq_i         : in  std_ulogic_vector(3 downto 0) := (others => '0')
  );
end neorv32_cpu;

architecture neorv32_cpu_rtl of neorv32_cpu is

  -- local signals --
  signal ctrl       : std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
  signal alu_cmp    : std_ulogic_vector(1 downto 0); -- alu comparator result
  signal imm        : std_ulogic_vector(data_width_c-1 downto 0); -- immediate
  signal instr      : std_ulogic_vector(data_width_c-1 downto 0); -- new instruction
  signal rs1, rs2   : std_ulogic_vector(data_width_c-1 downto 0); -- source registers
  signal alu_res    : std_ulogic_vector(data_width_c-1 downto 0); -- alu result
  signal alu_add    : std_ulogic_vector(data_width_c-1 downto 0); -- alu adder result
  signal rdata      : std_ulogic_vector(data_width_c-1 downto 0); -- memory read data
  signal alu_wait   : std_ulogic; -- alu is busy due to iterative unit
  signal bus_i_wait : std_ulogic; -- wait for current bus instruction fetch
  signal bus_d_wait : std_ulogic; -- wait for current bus data access
  signal csr_rdata  : std_ulogic_vector(data_width_c-1 downto 0); -- csr read data
  signal mar        : std_ulogic_vector(data_width_c-1 downto 0); -- current memory address register
  signal ma_instr   : std_ulogic; -- misaligned instruction address
  signal ma_load    : std_ulogic; -- misaligned load data address
  signal ma_store   : std_ulogic; -- misaligned store data address
  signal be_instr   : std_ulogic; -- bus error on instruction access
  signal be_load    : std_ulogic; -- bus error on load data access
  signal be_store   : std_ulogic; -- bus error on store data access
  signal fetch_pc   : std_ulogic_vector(data_width_c-1 downto 0); -- pc for instruction fetch
  signal curr_pc    : std_ulogic_vector(data_width_c-1 downto 0); -- current pc (for current executed instruction)
  signal next_pc    : std_ulogic_vector(data_width_c-1 downto 0); -- next pc (for next to-be-executed instruction)

  -- co-processor interface --
  signal cp0_data,  cp1_data  : std_ulogic_vector(data_width_c-1 downto 0);
  signal cp0_valid, cp1_valid : std_ulogic;
  signal cp0_start, cp1_start : std_ulogic;

  -- pmp interface --
  signal pmp_addr  : pmp_addr_if_t;
  signal pmp_ctrl  : pmp_ctrl_if_t;
  signal priv_mode : std_ulogic_vector(1 downto 0); -- current CPU privilege level

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- CSR system --
  assert not (CPU_EXTENSION_RISCV_Zicsr = false) report "NEORV32 CPU CONFIG WARNING! No exception/interrupt/machine features available when CPU_EXTENSION_RISCV_Zicsr = false." severity warning;
  -- U-extension requires Zicsr extension --
  assert not ((CPU_EXTENSION_RISCV_Zicsr = false) and (CPU_EXTENSION_RISCV_U = true)) report "NEORV32 CPU CONFIG ERROR! User mode requires CPU_EXTENSION_RISCV_Zicsr extension." severity error;
  -- PMP requires Zicsr extension --
  assert not ((CPU_EXTENSION_RISCV_Zicsr = false) and (PMP_USE = true)) report "NEORV32 CPU CONFIG ERROR! Physical memory protection (PMP) requires CPU_EXTENSION_RISCV_Zicsr extension." severity error;
  -- PMP regions --
  assert not ((PMP_NUM_REGIONS > pmp_max_r_c) and (PMP_USE = true)) report "NEORV32 CPU CONFIG ERROR! Number of PMP regions out of valid range." severity error;
  -- PMP granulartiy --
  assert not (((PMP_GRANULARITY < 1) or (PMP_GRANULARITY > 32)) and (PMP_USE = true)) report "NEORV32 CPU CONFIG ERROR! Invalid PMP granulartiy (0 < G < 33)." severity error;
  -- Bus timeout --
  assert not (BUS_TIMEOUT < 1) report "NEORV32 CPU CONFIG ERROR! Invalid bus timeout - must be at least 1 cycle." severity error;


  -- Control Unit ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_control_inst: neorv32_cpu_control
  generic map (
    -- General --
    HW_THREAD_ID                 => HW_THREAD_ID,    -- hardware thread id
    CPU_BOOT_ADDR                => CPU_BOOT_ADDR,   -- cpu boot address
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_C        => CPU_EXTENSION_RISCV_C,        -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        => CPU_EXTENSION_RISCV_E,        -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        => CPU_EXTENSION_RISCV_M,        -- implement muld/div extension?
    CPU_EXTENSION_RISCV_U        => CPU_EXTENSION_RISCV_U,        -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zicsr    => CPU_EXTENSION_RISCV_Zicsr,    -- implement CSR system?
    CPU_EXTENSION_RISCV_Zifencei => CPU_EXTENSION_RISCV_Zifencei, -- implement instruction stream sync.?
    -- Physical memory protection (PMP) --
    PMP_USE                      => PMP_USE,         -- implement physical memory protection?
    PMP_NUM_REGIONS              => PMP_NUM_REGIONS, -- number of regions (1..4)
    PMP_GRANULARITY              => PMP_GRANULARITY  -- granularity (0=none, 1=8B, 2=16B, 3=32B, ...)
  )
  port map (
    -- global control --
    clk_i         => clk_i,       -- global clock, rising edge
    rstn_i        => rstn_i,      -- global reset, low-active, async
    ctrl_o        => ctrl,        -- main control bus
    -- status input --
    alu_wait_i    => alu_wait,    -- wait for ALU
    bus_i_wait_i  => bus_i_wait,  -- wait for bus
    bus_d_wait_i  => bus_d_wait,  -- wait for bus
    -- data input --
    instr_i       => instr,       -- instruction
    cmp_i         => alu_cmp,     -- comparator status
    alu_add_i     => alu_add,     -- ALU.add result
    alu_res_i     => alu_res,     -- ALU processing result
    -- data output --
    imm_o         => imm,         -- immediate
    fetch_pc_o    => fetch_pc,    -- PC for instruction fetch
    curr_pc_o     => curr_pc,     -- current PC (corresponding to current instruction)
    next_pc_o     => next_pc,     -- next PC (corresponding to current instruction
    csr_rdata_o   => csr_rdata,   -- CSR read data
    -- interrupts (risc-v compliant) --
    msw_irq_i     => msw_irq_i,   -- machine software interrupt
    mext_irq_i    => mext_irq_i,  -- machine external interrupt
    mtime_irq_i   => mtime_irq_i, -- machine timer interrupt
    -- fast interrupts (custom) --
    firq_i        => firq_i,
    -- system time input from MTIME --
    time_i        => time_i,      -- current system time
    -- physical memory protection --
    pmp_addr_o    => pmp_addr,    -- addresses
    pmp_ctrl_o    => pmp_ctrl,    -- configs
    priv_mode_o   => priv_mode,   -- current CPU privilege level
    -- bus access exceptions --
    mar_i         => mar,         -- memory address register
    ma_instr_i    => ma_instr,    -- misaligned instruction address
    ma_load_i     => ma_load,     -- misaligned load data address
    ma_store_i    => ma_store,    -- misaligned store data address
    be_instr_i    => be_instr,    -- bus error on instruction access
    be_load_i     => be_load,     -- bus error on load data access
    be_store_i    => be_store     -- bus error on store data access
  );


  -- Register File --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_regfile_inst: neorv32_cpu_regfile
  generic map (
    CPU_EXTENSION_RISCV_E => CPU_EXTENSION_RISCV_E -- implement embedded RF extension?
  )
  port map (
    -- global control --
    clk_i  => clk_i,              -- global clock, rising edge
    ctrl_i => ctrl,               -- main control bus
    -- data input --
    mem_i  => rdata,              -- memory read data
    alu_i  => alu_res,            -- ALU result
    csr_i  => csr_rdata,          -- CSR read data
    pc_i   => next_pc,            -- next pc (for linking)
    -- data output --
    rs1_o  => rs1,                -- operand 1
    rs2_o  => rs2                 -- operand 2
  );


  -- ALU ------------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_alu_inst: neorv32_cpu_alu
  generic map (
    CPU_EXTENSION_RISCV_M => CPU_EXTENSION_RISCV_M -- implement muld/div extension?
  )
  port map (
    -- global control --
    clk_i       => clk_i,         -- global clock, rising edge
    rstn_i      => rstn_i,        -- global reset, low-active, async
    ctrl_i      => ctrl,          -- main control bus
    -- data input --
    rs1_i       => rs1,           -- rf source 1
    rs2_i       => rs2,           -- rf source 2
    pc2_i       => curr_pc,       -- delayed PC
    imm_i       => imm,           -- immediate
    -- data output --
    cmp_o       => alu_cmp,       -- comparator status
    add_o       => alu_add,       -- OPA + OPB
    res_o       => alu_res,       -- ALU result
    -- co-processor interface --
    cp0_start_o => cp0_start,     -- trigger co-processor 0
    cp0_data_i  => cp0_data,      -- co-processor 0 result
    cp0_valid_i => cp0_valid,     -- co-processor 0 result valid
    cp1_start_o => cp1_start,     -- trigger co-processor 1
    cp1_data_i  => cp1_data,      -- co-processor 1 result
    cp1_valid_i => cp1_valid,     -- co-processor 1 result valid
    -- status --
    wait_o      => alu_wait       -- busy due to iterative processing units
  );


  -- Co-Processor 0: MULDIV Unit ------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_cp_muldiv_inst_true:
  if (CPU_EXTENSION_RISCV_M = true) generate
    neorv32_cpu_cp_muldiv_inst: neorv32_cpu_cp_muldiv
    generic map (
      FAST_MUL_EN => FAST_MUL_EN -- use DSPs for faster multiplication
    )
    port map (
      -- global control --
      clk_i   => clk_i,           -- global clock, rising edge
      rstn_i  => rstn_i,          -- global reset, low-active, async
      ctrl_i  => ctrl,            -- main control bus
      -- data input --
      start_i => cp0_start,       -- trigger operation
      rs1_i   => rs1,             -- rf source 1
      rs2_i   => rs2,             -- rf source 2
      -- result and status --
      res_o   => cp0_data,        -- operation result
      valid_o => cp0_valid        -- data output valid
    );
  end generate;

  neorv32_cpu_cp_muldiv_inst_false:
  if (CPU_EXTENSION_RISCV_M = false) generate
    cp0_data  <= (others => '0');
    cp0_valid <= '0';
  end generate;


  -- Co-Processor 1: Not Implemented Yet ----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cp1_data  <= (others => '0');
  cp1_valid <= '0';


  -- Bus Interface Unit ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_bus_inst: neorv32_cpu_bus
  generic map (
    CPU_EXTENSION_RISCV_C => CPU_EXTENSION_RISCV_C, -- implement compressed extension?
    BUS_TIMEOUT           => BUS_TIMEOUT,           -- cycles after which a valid bus access will timeout
    -- Physical memory protection (PMP) --
    PMP_USE               => PMP_USE,               -- implement physical memory protection?
    PMP_NUM_REGIONS       => PMP_NUM_REGIONS,       -- number of regions (1..4)
    PMP_GRANULARITY       => PMP_GRANULARITY        -- granularity (0=none, 1=8B, 2=16B, 3=32B, ...)
  )
  port map (
    -- global control --
    clk_i          => clk_i,          -- global clock, rising edge
    rstn_i         => rstn_i,         -- global reset, low-active, async
    ctrl_i         => ctrl,           -- main control bus
    -- cpu instruction fetch interface --
    fetch_pc_i     => fetch_pc,       -- PC for instruction fetch
    instr_o        => instr,          -- instruction
    i_wait_o       => bus_i_wait,     -- wait for fetch to complete
    --
    ma_instr_o     => ma_instr,       -- misaligned instruction address
    be_instr_o     => be_instr,       -- bus error on instruction access
    -- cpu data access interface --
    addr_i         => alu_add,        -- ALU.add result -> access address
    wdata_i        => rs2,            -- write data
    rdata_o        => rdata,          -- read data
    mar_o          => mar,            -- current memory address register
    d_wait_o       => bus_d_wait,     -- wait for access to complete
    --
    ma_load_o      => ma_load,        -- misaligned load data address
    ma_store_o     => ma_store,       -- misaligned store data address
    be_load_o      => be_load,        -- bus error on load data access
    be_store_o     => be_store,       -- bus error on store data access
    -- physical memory protection --
    pmp_addr_i     => pmp_addr,       -- addresses
    pmp_ctrl_i     => pmp_ctrl,       -- configs
    priv_mode_i    => priv_mode,      -- current CPU privilege level
    -- instruction bus --
    i_bus_addr_o   => i_bus_addr_o,   -- bus access address
    i_bus_rdata_i  => i_bus_rdata_i,  -- bus read data
    i_bus_wdata_o  => i_bus_wdata_o,  -- bus write data
    i_bus_ben_o    => i_bus_ben_o,    -- byte enable
    i_bus_we_o     => i_bus_we_o,     -- write enable
    i_bus_re_o     => i_bus_re_o,     -- read enable
    i_bus_cancel_o => i_bus_cancel_o, -- cancel current bus transaction
    i_bus_ack_i    => i_bus_ack_i,    -- bus transfer acknowledge
    i_bus_err_i    => i_bus_err_i,    -- bus transfer error
    i_bus_fence_o  => i_bus_fence_o,  -- fence operation
    -- data bus --
    d_bus_addr_o   => d_bus_addr_o,   -- bus access address
    d_bus_rdata_i  => d_bus_rdata_i,  -- bus read data
    d_bus_wdata_o  => d_bus_wdata_o,  -- bus write data
    d_bus_ben_o    => d_bus_ben_o,    -- byte enable
    d_bus_we_o     => d_bus_we_o,     -- write enable
    d_bus_re_o     => d_bus_re_o,     -- read enable
    d_bus_cancel_o => d_bus_cancel_o, -- cancel current bus transaction
    d_bus_ack_i    => d_bus_ack_i,    -- bus transfer acknowledge
    d_bus_err_i    => d_bus_err_i,    -- bus transfer error
    d_bus_fence_o  => d_bus_fence_o   -- fence operation
  );


end neorv32_cpu_rtl;
