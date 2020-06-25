-- #################################################################################################
-- # << NEORV32 - CPU Top Entity >>                                                                #
-- # ********************************************************************************************* #
-- # Top NEORV32 CPU:                                                                              #
-- # * neorv32_cpu_alu: Arithemtical/logical unit                                                  #
-- # * neorv32_cpu_ctrl: CPU control and CSR system                                                #
-- #   * neorv32_cpu_decompressor: Compressed instructions decoder                                 #
-- # * neorv32_cpu_bus: Memory/IO bus interface unit                                               #
-- # * neorv32_cpu_cp_muldiv: MULDIV co-processor                                                  #
-- # * neorv32_cpu_regfile: Data register file                                                     #
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
    CLOCK_FREQUENCY           : natural := 0; -- clock frequency of clk_i in Hz
    HART_ID                   : std_ulogic_vector(31 downto 0) := x"00000000"; -- custom hardware thread ID
    BOOTLOADER_USE            : boolean := true;   -- implement processor-internal bootloader?
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_C     : boolean := false;  -- implement compressed extension?
    CPU_EXTENSION_RISCV_E     : boolean := false;  -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M     : boolean := false;  -- implement muld/div extension?
    CPU_EXTENSION_RISCV_Zicsr : boolean := true;   -- implement CSR system?
    -- Memory configuration: Instruction memory --
    MEM_ISPACE_BASE           : std_ulogic_vector(31 downto 0) := x"00000000"; -- base address of instruction memory space
    MEM_ISPACE_SIZE           : natural := 8*1024; -- total size of instruction memory space in byte
    MEM_INT_IMEM_USE          : boolean := true;   -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE         : natural := 8*1024; -- size of processor-internal instruction memory in bytes
    MEM_INT_IMEM_ROM          : boolean := false;  -- implement processor-internal instruction memory as ROM
    -- Memory configuration: Data memory --
    MEM_DSPACE_BASE           : std_ulogic_vector(31 downto 0) := x"80000000"; -- base address of data memory space
    MEM_DSPACE_SIZE           : natural := 4*1024; -- total size of data memory space in byte
    MEM_INT_DMEM_USE          : boolean := true;   -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE         : natural := 4*1024; -- size of processor-internal data memory in bytes
    -- Memory configuration: External memory interface --
    MEM_EXT_USE               : boolean := false;  -- implement external memory bus interface?
    MEM_EXT_TIMEOUT           : natural := 15;     -- cycles after which a valid bus access will timeout
    -- Processor peripherals --
    IO_GPIO_USE               : boolean := true;   -- implement general purpose input/output port unit (GPIO)?
    IO_MTIME_USE              : boolean := true;   -- implement machine system timer (MTIME)?
    IO_UART_USE               : boolean := true;   -- implement universal asynchronous receiver/transmitter (UART)?
    IO_SPI_USE                : boolean := true;   -- implement serial peripheral interface (SPI)?
    IO_TWI_USE                : boolean := true;   -- implement two-wire interface (TWI)?
    IO_PWM_USE                : boolean := true;   -- implement pulse-width modulation unit (PWM)?
    IO_WDT_USE                : boolean := true;   -- implement watch dog timer (WDT)?
    IO_CLIC_USE               : boolean := true;   -- implement core local interrupt controller (CLIC)?
    IO_TRNG_USE               : boolean := true;   -- implement true random number generator (TRNG)?
    IO_DEVNULL_USE            : boolean := true    -- implement dummy device (DEVNULL)?
  );
  port (
    -- global control --
    clk_i       : in  std_ulogic; -- global clock, rising edge
    rstn_i      : in  std_ulogic; -- global reset, low-active, async
    -- bus interface --
    bus_addr_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
    bus_rdata_i : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
    bus_wdata_o : out std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
    bus_ben_o   : out std_ulogic_vector(03 downto 0); -- byte enable
    bus_we_o    : out std_ulogic; -- write enable
    bus_re_o    : out std_ulogic; -- read enable
    bus_ack_i   : in  std_ulogic; -- bus transfer acknowledge
    bus_err_i   : in  std_ulogic; -- bus transfer error
    -- external interrupts --
    clic_irq_i  : in  std_ulogic; -- CLIC interrupt request
    mtime_irq_i : in  std_ulogic  -- machine timer interrupt
  );
end neorv32_cpu;

architecture neorv32_cpu_rtl of neorv32_cpu is

  -- local signals --
  signal ctrl        : std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
  signal alu_cmp     : std_ulogic_vector(1 downto 0); -- alu comparator result
  signal imm         : std_ulogic_vector(data_width_c-1 downto 0); -- immediate
  signal pc          : std_ulogic_vector(data_width_c-1 downto 0); -- current program counter
  signal pc_delayed  : std_ulogic_vector(data_width_c-1 downto 0); -- delayed program counter
  signal instr       : std_ulogic_vector(data_width_c-1 downto 0); -- new instruction
  signal rs1, rs2    : std_ulogic_vector(data_width_c-1 downto 0); -- source registers
  signal alu_res     : std_ulogic_vector(data_width_c-1 downto 0); -- alu result
  signal alu_add     : std_ulogic_vector(data_width_c-1 downto 0); -- alu adder result
  signal rdata       : std_ulogic_vector(data_width_c-1 downto 0); -- memory read data
  signal alu_wait    : std_ulogic; -- alu is busy due to iterative unit
  signal bus_wait    : std_ulogic; -- wait for bus to finish operation
  signal csr_rdata   : std_ulogic_vector(data_width_c-1 downto 0); -- csr read data
  signal mar         : std_ulogic_vector(data_width_c-1 downto 0); -- current memory address register
  signal ma_instr    : std_ulogic; -- misaligned instruction address
  signal ma_load     : std_ulogic; -- misaligned load data address
  signal ma_store    : std_ulogic; -- misaligned store data address
  signal be_instr    : std_ulogic; -- bus error on instruction access
  signal be_load     : std_ulogic; -- bus error on load data access
  signal be_store    : std_ulogic; -- bus error on store data access
  signal bus_exc_ack : std_ulogic; -- bus exception error acknowledge

  -- co-processor interface --
  signal cp0_data,  cp1_data  : std_ulogic_vector(data_width_c-1 downto 0);
  signal cp0_valid, cp1_valid : std_ulogic;

begin

  -- Control Unit ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_control_inst: neorv32_cpu_control
  generic map (
    -- General --
    CLOCK_FREQUENCY           => CLOCK_FREQUENCY, -- clock frequency of clk_i in Hz
    HART_ID                   => HART_ID,         -- custom hardware thread ID
    BOOTLOADER_USE            => BOOTLOADER_USE,  -- implement processor-internal bootloader?
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_C     => CPU_EXTENSION_RISCV_C,     -- implement compressed extension?
    CPU_EXTENSION_RISCV_E     => CPU_EXTENSION_RISCV_E,     -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M     => CPU_EXTENSION_RISCV_M,     -- implement muld/div extension?
    CPU_EXTENSION_RISCV_Zicsr => CPU_EXTENSION_RISCV_Zicsr, -- implement CSR system?
    -- Memory configuration: Instruction memory --
    MEM_ISPACE_BASE           => MEM_ISPACE_BASE,   -- base address of instruction memory space
    MEM_ISPACE_SIZE           => MEM_ISPACE_SIZE,   -- total size of instruction memory space in byte
    MEM_INT_IMEM_USE          => MEM_INT_IMEM_USE,  -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE         => MEM_INT_IMEM_SIZE, -- size of processor-internal instruction memory in bytes
    MEM_INT_IMEM_ROM          => MEM_INT_IMEM_ROM,  -- implement processor-internal instruction memory as ROM
    -- Memory configuration: Data memory --
    MEM_DSPACE_BASE           => MEM_DSPACE_BASE,   -- base address of data memory space
    MEM_DSPACE_SIZE           => MEM_DSPACE_SIZE,   -- total size of data memory space in byte
    MEM_INT_DMEM_USE          => MEM_INT_DMEM_USE,  -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE         => MEM_INT_DMEM_SIZE, -- size of processor-internal data memory in bytes
    -- Memory configuration: External memory interface --
    MEM_EXT_USE               => MEM_EXT_USE,       -- implement external memory bus interface?
    -- Processor peripherals --
    IO_GPIO_USE               => IO_GPIO_USE,       -- implement general purpose input/output port unit (GPIO)?
    IO_MTIME_USE              => IO_MTIME_USE,      -- implement machine system timer (MTIME)?
    IO_UART_USE               => IO_UART_USE,       -- implement universal asynchronous receiver/transmitter (UART)?
    IO_SPI_USE                => IO_SPI_USE,        -- implement serial peripheral interface (SPI)?
    IO_TWI_USE                => IO_TWI_USE,        -- implement two-wire interface (TWI)?
    IO_PWM_USE                => IO_PWM_USE,        -- implement pulse-width modulation unit (PWM)?
    IO_WDT_USE                => IO_WDT_USE,        -- implement watch dog timer (WDT)?
    IO_CLIC_USE               => IO_CLIC_USE,       -- implement core local interrupt controller (CLIC)?
    IO_TRNG_USE               => IO_TRNG_USE,       -- implement true random number generator (TRNG)?
    IO_DEVNULL_USE            => IO_DEVNULL_USE     -- implement dummy device (DEVNULL)?
  )
  port map (
    -- global control --
    clk_i         => clk_i,       -- global clock, rising edge
    rstn_i        => rstn_i,      -- global reset, low-active, async
    ctrl_o        => ctrl,        -- main control bus
    -- status input --
    alu_wait_i    => alu_wait,    -- wait for ALU
    bus_wait_i    => bus_wait,    -- wait for bus
    -- data input --
    instr_i       => instr,       -- instruction
    cmp_i         => alu_cmp,     -- comparator status
    alu_add_i     => alu_add,     -- ALU.add result
    -- data output --
    imm_o         => imm,         -- immediate
    pc_o          => pc,          -- current PC
    alu_pc_o      => pc_delayed,  -- delayed PC for ALU
    -- csr interface --
    csr_wdata_i   => alu_res,     -- CSR write data
    csr_rdata_o   => csr_rdata,   -- CSR read data
    -- external interrupt --
    clic_irq_i    => clic_irq_i,  -- CLIC interrupt request
    mtime_irq_i   => mtime_irq_i, -- machine timer interrupt
    -- bus access exceptions --
    mar_i         => mar,         -- memory address register
    ma_instr_i    => ma_instr,    -- misaligned instruction address
    ma_load_i     => ma_load,     -- misaligned load data address
    ma_store_i    => ma_store,    -- misaligned store data address
    be_instr_i    => be_instr,    -- bus error on instruction access
    be_load_i     => be_load,     -- bus error on load data access
    be_store_i    => be_store,    -- bus error on store data access
    bus_exc_ack_o => bus_exc_ack  -- bus exception error acknowledge
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
    pc_i   => pc,                 -- current pc
    -- data output --
    rs1_o  => rs1,                -- operand 1
    rs2_o  => rs2                 -- operand 2
  );


  -- ALU ------------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_alu_inst: neorv32_cpu_alu
  generic map (
    CPU_EXTENSION_RISCV_C => CPU_EXTENSION_RISCV_C, -- implement compressed extension?
    CPU_EXTENSION_RISCV_M => CPU_EXTENSION_RISCV_M  -- implement mul/div extension?
  )
  port map (
    -- global control --
    clk_i       => clk_i,         -- global clock, rising edge
    rstn_i      => rstn_i,        -- global reset, low-active, async
    ctrl_i      => ctrl,          -- main control bus
    -- data input --
    rs1_i       => rs1,           -- rf source 1
    rs2_i       => rs2,           -- rf source 2
    pc_i        => pc,            -- current PC
    pc2_i       => pc_delayed,    -- delayed PC
    imm_i       => imm,           -- immediate
    csr_i       => csr_rdata,     -- csr read data
    -- data output --
    cmp_o       => alu_cmp,       -- comparator status
    add_o       => alu_add,       -- OPA + OPB
    res_o       => alu_res,       -- ALU result
    -- co-processor interface --
    cp0_data_i  => cp0_data,      -- co-processor 0 result
    cp0_valid_i => cp0_valid,     -- co-processor 0 result valid
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
    port map (
      -- global control --
      clk_i   => clk_i,           -- global clock, rising edge
      rstn_i  => rstn_i,          -- global reset, low-active, async
      ctrl_i  => ctrl,            -- main control bus
      -- data input --
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


  -- Bus Unit -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_bus_inst: neorv32_cpu_bus
  generic map (
    CPU_EXTENSION_RISCV_C => CPU_EXTENSION_RISCV_C, -- implement compressed extension?
    MEM_EXT_TIMEOUT       => MEM_EXT_TIMEOUT        -- cycles after which a valid bus access will timeout
  )
  port map (
    -- global control --
    clk_i       => clk_i,         -- global clock, rising edge
    rstn_i      => rstn_i,        -- global reset, low-active, async
    ctrl_i      => ctrl,          -- main control bus
    -- data input --
    wdata_i     => rs2,           -- write data
    pc_i        => pc,            -- current PC
    alu_i       => alu_res,       -- ALU result
    -- data output --
    instr_o     => instr,         -- instruction
    rdata_o     => rdata,         -- read data
    -- status --
    mar_o       => mar,           -- current memory address register
    ma_instr_o  => ma_instr,      -- misaligned instruction address
    ma_load_o   => ma_load,       -- misaligned load data address
    ma_store_o  => ma_store,      -- misaligned store data address
    be_instr_o  => be_instr,      -- bus error on instruction access
    be_load_o   => be_load,       -- bus error on load data access
    be_store_o  => be_store,      -- bus error on store data access
    bus_wait_o  => bus_wait,      -- wait for bus operation to finish
    exc_ack_i   => bus_exc_ack,   -- exception controller ACK
    -- bus system --
    bus_addr_o  => bus_addr_o,    -- bus access address
    bus_rdata_i => bus_rdata_i,   -- bus read data
    bus_wdata_o => bus_wdata_o,   -- bus write data
    bus_ben_o   => bus_ben_o,     -- byte enable
    bus_we_o    => bus_we_o,      -- write enable
    bus_re_o    => bus_re_o,      -- read enable
    bus_ack_i   => bus_ack_i,     -- bus transfer acknowledge
    bus_err_i   => bus_err_i      -- bus transfer error
  );


end neorv32_cpu_rtl;
