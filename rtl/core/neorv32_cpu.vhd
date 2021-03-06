-- #################################################################################################
-- # << NEORV32 - CPU Top Entity >>                                                                #
-- # ********************************************************************************************* #
-- # NEORV32 CPU:                                                                                  #
-- # * neorv32_cpu.vhd                   - CPU top entity                                          #
-- #   * neorv32_cpu_alu.vhd             - Arithmetic/logic unit                                   #
-- #   * neorv32_cpu_bus.vhd             - Instruction and data bus interface unit                 #
-- #   * neorv32_cpu_cp_bitmanip.vhd     - Bit-manipulation co-processor ('B')                     #
-- #   * neorv32_cpu_cp_fpu.vhd          - Single-precision FPU co-processor ('F')                 #
-- #   * neorv32_cpu_cp_muldiv.vhd       - Integer multiplier/divider co-processor ('M')           #
-- #   * neorv32_cpu_ctrl.vhd            - CPU control and CSR system                              #
-- #     * neorv32_cpu_decompressor.vhd  - Compressed instructions decoder                         #
-- #   * neorv32_cpu_regfile.vhd         - Data register file                                      #
-- # * neorv32_package.vhd               - Main CPU/processor package file                         #
-- #                                                                                               #
-- # Check out the processor's data sheet for more information: docs/NEORV32.pdf                   #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
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
    HW_THREAD_ID                 : natural := 0;     -- hardware thread id (32-bit)
    CPU_BOOT_ADDR                : std_ulogic_vector(31 downto 0):= x"00000000"; -- cpu boot address
    BUS_TIMEOUT                  : natural := 63;    -- cycles after an UNACKNOWLEDGED bus access triggers a bus fault exception
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        : boolean := false; -- implement atomic extension?
    CPU_EXTENSION_RISCV_B        : boolean := false; -- implement bit manipulation extensions?
    CPU_EXTENSION_RISCV_C        : boolean := false; -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        : boolean := false; -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_F        : boolean := false; -- implement 32-bit floating-point extension?
    CPU_EXTENSION_RISCV_M        : boolean := false; -- implement muld/div extension?
    CPU_EXTENSION_RISCV_U        : boolean := false; -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zicsr    : boolean := true;  -- implement CSR system?
    CPU_EXTENSION_RISCV_Zifencei : boolean := false; -- implement instruction stream sync.?
    -- Extension Options --
    FAST_MUL_EN                  : boolean := false; -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                : boolean := false; -- use barrel shifter for shift operations
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS              : natural := 0; -- number of regions (0..64)
    PMP_MIN_GRANULARITY          : natural := 64*1024; -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 : natural := 0      -- number of implemented HPM counters (0..29)
  );
  port (
    -- global control --
    clk_i          : in  std_ulogic := '0'; -- global clock, rising edge
    rstn_i         : in  std_ulogic := '0'; -- global reset, low-active, async
    sleep_o        : out std_ulogic; -- cpu is in sleep mode when set
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
    i_bus_priv_o   : out std_ulogic_vector(1 downto 0); -- privilege level
    i_bus_lock_o   : out std_ulogic; -- locked/exclusive access
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
    d_bus_priv_o   : out std_ulogic_vector(1 downto 0); -- privilege level
    d_bus_lock_o   : out std_ulogic; -- locked/exclusive access
    -- system time input from MTIME --
    time_i         : in  std_ulogic_vector(63 downto 0) := (others => '0'); -- current system time
    -- interrupts (risc-v compliant) --
    msw_irq_i      : in  std_ulogic := '0'; -- machine software interrupt
    mext_irq_i     : in  std_ulogic := '0'; -- machine external interrupt
    mtime_irq_i    : in  std_ulogic := '0'; -- machine timer interrupt
    -- fast interrupts (custom) --
    firq_i         : in  std_ulogic_vector(15 downto 0) := (others => '0');
    firq_ack_o     : out std_ulogic_vector(15 downto 0)
  );
end neorv32_cpu;

architecture neorv32_cpu_rtl of neorv32_cpu is

  -- local signals --
  signal ctrl          : std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
  signal comparator    : std_ulogic_vector(1 downto 0); -- comparator result
  signal imm           : std_ulogic_vector(data_width_c-1 downto 0); -- immediate
  signal instr         : std_ulogic_vector(data_width_c-1 downto 0); -- new instruction
  signal rs1, rs2      : std_ulogic_vector(data_width_c-1 downto 0); -- source registers
  signal alu_res       : std_ulogic_vector(data_width_c-1 downto 0); -- alu result
  signal alu_add       : std_ulogic_vector(data_width_c-1 downto 0); -- alu address result
  signal mem_rdata     : std_ulogic_vector(data_width_c-1 downto 0); -- memory read data
  signal mem_wdata     : std_ulogic_vector(data_width_c-1 downto 0); -- memory write-data
  signal alu_wait      : std_ulogic; -- alu is busy due to iterative unit
  signal bus_i_wait    : std_ulogic; -- wait for current bus instruction fetch
  signal bus_d_wait    : std_ulogic; -- wait for current bus data access
  signal csr_rdata     : std_ulogic_vector(data_width_c-1 downto 0); -- csr read data
  signal mar           : std_ulogic_vector(data_width_c-1 downto 0); -- current memory address register
  signal ma_instr      : std_ulogic; -- misaligned instruction address
  signal ma_load       : std_ulogic; -- misaligned load data address
  signal ma_store      : std_ulogic; -- misaligned store data address
  signal be_instr      : std_ulogic; -- bus error on instruction access
  signal be_load       : std_ulogic; -- bus error on load data access
  signal be_store      : std_ulogic; -- bus error on store data access
  signal fetch_pc      : std_ulogic_vector(data_width_c-1 downto 0); -- pc for instruction fetch
  signal curr_pc       : std_ulogic_vector(data_width_c-1 downto 0); -- current pc (for current executed instruction)
  signal fpu_mem_wdata : std_ulogic_vector(data_width_c-1 downto 0); -- memory write-data form FPU
  signal fpu_rm        : std_ulogic_vector(2 downto 0); -- FPU rounding mode
  signal fpu_flags     : std_ulogic_vector(4 downto 0); -- FPU exception flags

  -- co-processor interface --
  signal cp_start  : std_ulogic_vector(7 downto 0); -- trigger co-processor i
  signal cp_valid  : std_ulogic_vector(7 downto 0); -- co-processor i done
  signal cp_result : cp_data_if_t; -- co-processor result

  -- pmp interface --
  signal pmp_addr  : pmp_addr_if_t;
  signal pmp_ctrl  : pmp_ctrl_if_t;

  -- atomic memory access - success? --
  signal atomic_sc_res: std_ulogic;

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- CSR system --
  assert not (CPU_EXTENSION_RISCV_Zicsr = false) report "NEORV32 CPU CONFIG WARNING! No exception/interrupt/trap/privileged features available when CPU_EXTENSION_RISCV_Zicsr = false." severity warning;
  -- U-extension requires Zicsr extension --
  assert not ((CPU_EXTENSION_RISCV_Zicsr = false) and (CPU_EXTENSION_RISCV_U = true)) report "NEORV32 CPU CONFIG ERROR! User mode requires CPU_EXTENSION_RISCV_Zicsr extension." severity error;
  -- PMP requires Zicsr extension --
  assert not ((CPU_EXTENSION_RISCV_Zicsr = false) and (PMP_NUM_REGIONS > 0)) report "NEORV32 CPU CONFIG ERROR! Physical memory protection (PMP) requires CPU_EXTENSION_RISCV_Zicsr extension." severity error;

  -- Bus timeout --
  assert not (BUS_TIMEOUT < 2) report "NEORV32 CPU CONFIG ERROR! Invalid bus access timeout value <BUS_TIMEOUT>. Has to be >= 2." severity error;

  -- Instruction prefetch buffer size --
  assert not (is_power_of_two_f(ipb_entries_c) = false) report "NEORV32 CPU CONFIG ERROR! Number of entries in instruction prefetch buffer <ipb_entries_c> has to be a power of two." severity error;
  -- A extension - only lr.w and sc.w are supported yet --
  assert not (CPU_EXTENSION_RISCV_A = true) report "NEORV32 CPU CONFIG WARNING! Atomic operations extension (A) only supports <lr.w> and <sc.w> instructions." severity warning;

  -- FIXME: Bit manipulation warning --
  assert not (CPU_EXTENSION_RISCV_B = true) report "NEORV32 CPU CONFIG WARNING! Bit manipulation extension (B) is still HIGHLY EXPERIMENTAL (and spec. is not ratified yet)." severity warning;

  -- FIXME: Floating-point extension warning --
  assert not (CPU_EXTENSION_RISCV_F = true) report "NEORV32 CPU CONFIG WARNING! 32-bit floating-point extension (F) is WORK-IN-PROGRESS and NOT OPERATIONAL yet." severity warning;

  -- PMP regions check --
  assert not (PMP_NUM_REGIONS > 64) report "NEORV32 CPU CONFIG ERROR! Number of PMP regions <PMP_NUM_REGIONS> out of valid range (0..64)." severity error;
  -- PMP granulartiy --
  assert not ((is_power_of_two_f(PMP_MIN_GRANULARITY) = false) and (PMP_NUM_REGIONS > 0)) report "NEORV32 CPU CONFIG ERROR! PMP granulartiy has to be a power of two." severity error;
  assert not ((PMP_MIN_GRANULARITY < 8) and (PMP_NUM_REGIONS > 0)) report "NEORV32 CPU CONFIG ERROR! PMP granulartiy has to be >= 8 bytes." severity error;
  -- PMP notifier --
  assert not (PMP_NUM_REGIONS > 0) report "NEORV32 CPU CONFIG NOTE: Implementing physical memory protection (PMP) with " & integer'image(PMP_NUM_REGIONS) & " regions and a minimal granularity of " & integer'image(PMP_MIN_GRANULARITY) & " bytes." severity note;

  -- HPM counters check --
  assert not (HPM_NUM_CNTS > 29) report "NEORV32 CPU CONFIG ERROR! Number of HPM counters <HPM_NUM_CNTS> out of valid range (0..29)." severity error;
  -- HPM counters notifier --
  assert not (HPM_NUM_CNTS > 0) report "NEORV32 CPU CONFIG NOTE: Implementing " & integer'image(HPM_NUM_CNTS) & " HPM counters." severity note;
  -- HPM CNT requires Zicsr extension --
  assert not ((CPU_EXTENSION_RISCV_Zicsr = false) and (HPM_NUM_CNTS > 0)) report "NEORV32 CPU CONFIG ERROR! Performance monitors (HMP) require CPU_EXTENSION_RISCV_Zicsr extension." severity error;


  -- Control Unit ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_control_inst: neorv32_cpu_control
  generic map (
    -- General --
    HW_THREAD_ID                 => HW_THREAD_ID,  -- hardware thread id
    CPU_BOOT_ADDR                => CPU_BOOT_ADDR, -- cpu boot address
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        => CPU_EXTENSION_RISCV_A,        -- implement atomic extension?
    CPU_EXTENSION_RISCV_B        => CPU_EXTENSION_RISCV_B,        -- implement bit manipulation extensions?
    CPU_EXTENSION_RISCV_C        => CPU_EXTENSION_RISCV_C,        -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        => CPU_EXTENSION_RISCV_E,        -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_F        => CPU_EXTENSION_RISCV_F,        -- implement 32-bit floating-point extension?
    CPU_EXTENSION_RISCV_M        => CPU_EXTENSION_RISCV_M,        -- implement muld/div extension?
    CPU_EXTENSION_RISCV_U        => CPU_EXTENSION_RISCV_U,        -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zicsr    => CPU_EXTENSION_RISCV_Zicsr,    -- implement CSR system?
    CPU_EXTENSION_RISCV_Zifencei => CPU_EXTENSION_RISCV_Zifencei, -- implement instruction stream sync.?
    -- Physical memory protection (PMP) --
    PMP_NUM_REGIONS              => PMP_NUM_REGIONS,              -- number of regions (0..64)
    PMP_MIN_GRANULARITY          => PMP_MIN_GRANULARITY,          -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 => HPM_NUM_CNTS                  -- number of implemented HPM counters (0..29)
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
    cmp_i         => comparator,  -- comparator status
    alu_add_i     => alu_add,     -- ALU address result
    rs1_i         => rs1,         -- rf source 1
    -- data output --
    imm_o         => imm,         -- immediate
    fetch_pc_o    => fetch_pc,    -- PC for instruction fetch
    curr_pc_o     => curr_pc,     -- current PC (corresponding to current instruction)
    csr_rdata_o   => csr_rdata,   -- CSR read data
    -- FPU interface --
    fpu_rm_o      => fpu_rm,      -- rounding mode
    fpu_flags_i   => fpu_flags,   -- exception flags
    -- interrupts (risc-v compliant) --
    msw_irq_i     => msw_irq_i,   -- machine software interrupt
    mext_irq_i    => mext_irq_i,  -- machine external interrupt
    mtime_irq_i   => mtime_irq_i, -- machine timer interrupt
    -- fast interrupts (custom) --
    firq_i        => firq_i,      -- fast interrupt trigger
    firq_ack_o    => firq_ack_o,  -- fast interrupt acknowledge mask
    -- system time input from MTIME --
    time_i        => time_i,      -- current system time
    -- physical memory protection --
    pmp_addr_o    => pmp_addr,    -- addresses
    pmp_ctrl_o    => pmp_ctrl,    -- configs
    -- bus access exceptions --
    mar_i         => mar,         -- memory address register
    ma_instr_i    => ma_instr,    -- misaligned instruction address
    ma_load_i     => ma_load,     -- misaligned load data address
    ma_store_i    => ma_store,    -- misaligned store data address
    be_instr_i    => be_instr,    -- bus error on instruction access
    be_load_i     => be_load,     -- bus error on load data access
    be_store_i    => be_store     -- bus error on store data access
  );

  -- CPU is sleeping? --
  sleep_o <= ctrl(ctrl_sleep_c); -- set when CPU is sleeping (after WFI)


  -- Register File --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_regfile_inst: neorv32_cpu_regfile
  generic map (
    CPU_EXTENSION_RISCV_E => CPU_EXTENSION_RISCV_E -- implement embedded RF extension?
  )
  port map (
    -- global control --
    clk_i  => clk_i,              -- global clock, rising edge
    ctrl_i => ctrl,               -- main control bus
    -- data input --
    mem_i  => mem_rdata,          -- memory read data
    alu_i  => alu_res,            -- ALU result
    -- data output --
    rs1_o  => rs1,                -- operand 1
    rs2_o  => rs2,                -- operand 2
    cmp_o  => comparator          -- comparator status
  );


  -- ALU ------------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_alu_inst: neorv32_cpu_alu
  generic map (
    CPU_EXTENSION_RISCV_M => CPU_EXTENSION_RISCV_M, -- implement muld/div extension?
    FAST_SHIFT_EN         => FAST_SHIFT_EN          -- use barrel shifter for shift operations
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
    res_o       => alu_res,       -- ALU result
    add_o       => alu_add,       -- address computation result
    -- co-processor interface --
    cp_start_o  => cp_start,      -- trigger co-processor i
    cp_valid_i  => cp_valid,      -- co-processor i done
    cp_result_i => cp_result,     -- co-processor result
    -- status --
    wait_o      => alu_wait       -- busy due to iterative processing units
  );


  -- Co-Processor 0: Integer Multiplication/Division ('M' Extension) ------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_cp_muldiv_inst_true:
  if (CPU_EXTENSION_RISCV_M = true) generate
    neorv32_cpu_cp_muldiv_inst: neorv32_cpu_cp_muldiv
    generic map (
      FAST_MUL_EN => FAST_MUL_EN  -- use DSPs for faster multiplication
    )
    port map (
      -- global control --
      clk_i   => clk_i,           -- global clock, rising edge
      rstn_i  => rstn_i,          -- global reset, low-active, async
      ctrl_i  => ctrl,            -- main control bus
      start_i => cp_start(0),     -- trigger operation
      -- data input --
      rs1_i   => rs1,             -- rf source 1
      rs2_i   => rs2,             -- rf source 2
      -- result and status --
      res_o   => cp_result(0),    -- operation result
      valid_o => cp_valid(0)      -- data output valid
    );
  end generate;

  neorv32_cpu_cp_muldiv_inst_false:
  if (CPU_EXTENSION_RISCV_M = false) generate
    cp_result(0) <= (others => '0');
    cp_valid(0)  <= cp_start(0); -- to make sure CPU does not get stalled if there is an accidental access
  end generate;


  -- Co-Processor 1: Atomic Memory Access ('A' Extension) -----------------------------------
  -- -------------------------------------------------------------------------------------------
  -- "pseudo" co-processor for atomic operations
  -- required to get the result of a store-conditional operation into the data path
  atomic_op_cp: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (cp_start(1) = '1') then
        atomic_sc_res <= not ctrl(ctrl_bus_lock_c);
      else
        atomic_sc_res <= '0';
      end if;
    end if;
  end process atomic_op_cp;

  -- CP result --
  cp_result(1)(data_width_c-1 downto 1) <= (others => '0');
  cp_result(1)(0) <= atomic_sc_res when (CPU_EXTENSION_RISCV_A = true) else '0';
  cp_valid(1)     <= cp_start(1); -- always assigned even if A extension is disabled to make sure CPU does not get stalled if there is an accidental access


  -- Co-Processor 2: Bit Manipulation ('B' Extension) ---------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_cp_bitmanip_inst_true:
  if (CPU_EXTENSION_RISCV_B = true) generate
    neorv32_cpu_cp_bitmanip_inst: neorv32_cpu_cp_bitmanip
    port map (
      -- global control --
      clk_i   => clk_i,           -- global clock, rising edge
      rstn_i  => rstn_i,          -- global reset, low-active, async
      ctrl_i  => ctrl,            -- main control bus
      start_i => cp_start(2),     -- trigger operation
      -- data input --
      cmp_i   => comparator,      -- comparator status
      rs1_i   => rs1,             -- rf source 1
      rs2_i   => rs2,             -- rf source 2
      -- result and status --
      res_o   => cp_result(2),    -- operation result
      valid_o => cp_valid(2)      -- data output valid
    );
  end generate;

  neorv32_cpu_cp_bitmanip_inst_false:
  if (CPU_EXTENSION_RISCV_B = false) generate
    cp_result(2) <= (others => '0');
    cp_valid(2)  <= cp_start(2); -- to make sure CPU does not get stalled if there is an accidental access
  end generate;


  -- Co-Processor 3: CSR (Read) Access ('Zicsr' Extension) ----------------------------------
  -- -------------------------------------------------------------------------------------------
  -- "pseudo" co-processor for CSR *read* access operations
  -- required to get the CSR read data into the data path
  cp_result(3) <= csr_rdata when (CPU_EXTENSION_RISCV_Zicsr = true) else (others => '0');
  cp_valid(3)  <= cp_start(3); -- always assigned even if Zicsr extension is disabled to make sure CPU does not get stalled if there is an accidental access


  -- Co-Processor 4: Single-Precision Floating-Point Unit ('F' Extension) -------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_cp_fpu_inst_true:
  if (CPU_EXTENSION_RISCV_F = true) generate
    neorv32_cpu_cp_fpu_inst: neorv32_cpu_cp_fpu
    port map (
      -- global control --
      clk_i     => clk_i,         -- global clock, rising edge
      rstn_i    => rstn_i,        -- global reset, low-active, async
      ctrl_i    => ctrl,          -- main control bus
      start_i   => cp_start(4),   -- trigger operation
      -- data input --
      frm_i     => fpu_rm,        -- rounding mode
      reg_i     => rs1,           -- rf source
      mem_i     => mem_rdata,     -- memory read-data
      -- result and status --
      fflags_o  => fpu_flags,     -- exception flags
      mem_o     => fpu_mem_wdata, -- memory write-data
      res_o     => cp_result(4),  -- operation result
      valid_o   => cp_valid(4)    -- data output valid
    );
  end generate;

  neorv32_cpu_cp_fpu_inst_false:
  if (CPU_EXTENSION_RISCV_F = false) generate
    fpu_flags     <= (others => '0');
    fpu_mem_wdata <= (others => '0');
    cp_result(4)  <= (others => '0');
    cp_valid(4)   <= cp_start(4); -- to make sure CPU does not get stalled if there is an accidental access
  end generate;


  -- Co-Processor 5..7: Not Implemented Yet -------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cp_result(5) <= (others => '0');
  cp_valid(5)  <= '0';
  --
  cp_result(6) <= (others => '0');
  cp_valid(6)  <= '0';
  --
  cp_result(7) <= (others => '0');
  cp_valid(7)  <= '0';


  -- Bus Interface Unit ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_bus_inst: neorv32_cpu_bus
  generic map (
    CPU_EXTENSION_RISCV_C => CPU_EXTENSION_RISCV_C, -- implement compressed extension?
    -- Physical memory protection (PMP) --
    PMP_NUM_REGIONS       => PMP_NUM_REGIONS,       -- number of regions (0..64)
    PMP_MIN_GRANULARITY   => PMP_MIN_GRANULARITY,   -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
    -- Bus Timeout --
    BUS_TIMEOUT           => BUS_TIMEOUT            -- cycles after an UNACKNOWLEDGED bus access triggers a bus fault exception
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
    wdata_i        => mem_wdata,      -- write data
    rdata_o        => mem_rdata,      -- read data
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
    i_bus_lock_o   => i_bus_lock_o,   -- locked/exclusive access
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
    d_bus_fence_o  => d_bus_fence_o,  -- fence operation
    d_bus_lock_o   => d_bus_lock_o    -- locked/exclusive access
  );

  -- memory write data --
  mem_wdata <= fpu_mem_wdata when ((CPU_EXTENSION_RISCV_F = true) and (ctrl(ctrl_bus_wd_sel_c) = '1')) else rs2;

  -- current privilege level --
  i_bus_priv_o <= ctrl(ctrl_priv_lvl_msb_c downto ctrl_priv_lvl_lsb_c);
  d_bus_priv_o <= ctrl(ctrl_priv_lvl_msb_c downto ctrl_priv_lvl_lsb_c);


end neorv32_cpu_rtl;
