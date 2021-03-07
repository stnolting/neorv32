-- #################################################################################################
-- # << NEORV32 - Processor Top Entity >>                                                          #
-- # ********************************************************************************************* #
-- # This is the top entity of the NEORV32 PROCESSOR. Instantiate this unit in your own project    #
-- # and define all the configuration generics according to your needs. Alternatively, you can use #
-- # one of the alternative top entities provided in the "rtl/top_templates" folder.               #
-- #                                                                                               #
-- # Check the processor's data sheet for more information: docs/NEORV32.pdf                       #
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

entity neorv32_top is
  generic (
    -- General --
    CLOCK_FREQUENCY              : natural := 0;      -- clock frequency of clk_i in Hz
    BOOTLOADER_EN                : boolean := true;   -- implement processor-internal bootloader?
    USER_CODE                    : std_ulogic_vector(31 downto 0) := x"00000000"; -- custom user code
    HW_THREAD_ID                 : natural := 0;      -- hardware thread id (32-bit)

    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        : boolean := false;  -- implement atomic extension?
    CPU_EXTENSION_RISCV_B        : boolean := false;  -- implement bit manipulation extensions?
    CPU_EXTENSION_RISCV_C        : boolean := false;  -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        : boolean := false;  -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        : boolean := false;  -- implement muld/div extension?
    CPU_EXTENSION_RISCV_U        : boolean := false;  -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zicsr    : boolean := true;   -- implement CSR system?
    CPU_EXTENSION_RISCV_Zifencei : boolean := false;  -- implement instruction stream sync.?

    -- Extension Options --
    FAST_MUL_EN                  : boolean := false;  -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                : boolean := false;  -- use barrel shifter for shift operations

    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS              : natural := 0;      -- number of regions (0..64)
    PMP_MIN_GRANULARITY          : natural := 64*1024; -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes

    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 : natural := 0;      -- number of implemented HPM counters (0..29)

    -- Internal Instruction memory --
    MEM_INT_IMEM_EN              : boolean := true;   -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            : natural := 16*1024; -- size of processor-internal instruction memory in bytes
    MEM_INT_IMEM_ROM             : boolean := false;  -- implement processor-internal instruction memory as ROM

    -- Internal Data memory --
    MEM_INT_DMEM_EN              : boolean := true;   -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            : natural := 8*1024; -- size of processor-internal data memory in bytes

    -- Internal Cache memory --
    ICACHE_EN                    : boolean := false;  -- implement instruction cache
    ICACHE_NUM_BLOCKS            : natural := 4;      -- i-cache: number of blocks (min 1), has to be a power of 2
    ICACHE_BLOCK_SIZE            : natural := 64;     -- i-cache: block size in bytes (min 4), has to be a power of 2
    ICACHE_ASSOCIATIVITY         : natural := 1;      -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2

    -- External memory interface --
    MEM_EXT_EN                   : boolean := false;  -- implement external memory bus interface?

    -- Processor peripherals --
    IO_GPIO_EN                   : boolean := true;   -- implement general purpose input/output port unit (GPIO)?
    IO_MTIME_EN                  : boolean := true;   -- implement machine system timer (MTIME)?
    IO_UART0_EN                  : boolean := true;   -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_UART1_EN                  : boolean := true;   -- implement secondary universal asynchronous receiver/transmitter (UART1)?
    IO_SPI_EN                    : boolean := true;   -- implement serial peripheral interface (SPI)?
    IO_TWI_EN                    : boolean := true;   -- implement two-wire interface (TWI)?
    IO_PWM_EN                    : boolean := true;   -- implement pulse-width modulation unit (PWM)?
    IO_WDT_EN                    : boolean := true;   -- implement watch dog timer (WDT)?
    IO_TRNG_EN                   : boolean := false;  -- implement true random number generator (TRNG)?
    IO_CFS_EN                    : boolean := false;  -- implement custom functions subsystem (CFS)?
    IO_CFS_CONFIG                : std_ulogic_vector(31 downto 0); -- custom CFS configuration generic
    IO_CFS_IN_SIZE               : positive := 32;    -- size of CFS input conduit in bits
    IO_CFS_OUT_SIZE              : positive := 32;    -- size of CFS output conduit in bits
    IO_NCO_EN                    : boolean := true;   -- implement numerically-controlled oscillator (NCO)?
    IO_NEOLED_EN                 : boolean := true    -- implement NeoPixel-compatible smart LED interface (NEOLED)?
  );
  port (
    -- Global control --
    clk_i       : in  std_ulogic := '0'; -- global clock, rising edge
    rstn_i      : in  std_ulogic := '0'; -- global reset, low-active, async

    -- Wishbone bus interface (available if MEM_EXT_EN = true) --
    wb_tag_o    : out std_ulogic_vector(02 downto 0); -- tag
    wb_adr_o    : out std_ulogic_vector(31 downto 0); -- address
    wb_dat_i    : in  std_ulogic_vector(31 downto 0) := (others => '0'); -- read data
    wb_dat_o    : out std_ulogic_vector(31 downto 0); -- write data
    wb_we_o     : out std_ulogic; -- read/write
    wb_sel_o    : out std_ulogic_vector(03 downto 0); -- byte enable
    wb_stb_o    : out std_ulogic; -- strobe
    wb_cyc_o    : out std_ulogic; -- valid cycle
    wb_lock_o   : out std_ulogic; -- locked/exclusive bus access
    wb_ack_i    : in  std_ulogic := '0'; -- transfer acknowledge
    wb_err_i    : in  std_ulogic := '0'; -- transfer error

    -- Advanced memory control signals (available if MEM_EXT_EN = true) --
    fence_o     : out std_ulogic; -- indicates an executed FENCE operation
    fencei_o    : out std_ulogic; -- indicates an executed FENCEI operation

    -- GPIO (available if IO_GPIO_EN = true) --
    gpio_o      : out std_ulogic_vector(31 downto 0); -- parallel output
    gpio_i      : in  std_ulogic_vector(31 downto 0) := (others => '0'); -- parallel input

    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o : out std_ulogic; -- UART0 send data
    uart0_rxd_i : in  std_ulogic := '0'; -- UART0 receive data
    uart0_rts_o : out std_ulogic; -- hw flow control: UART0.RX ready to receive ("RTR"), low-active, optional
    uart0_cts_i : in  std_ulogic := '0'; -- hw flow control: UART0.TX allowed to transmit, low-active, optional

    -- secondary UART1 (available if IO_UART1_EN = true) --
    uart1_txd_o : out std_ulogic; -- UART1 send data
    uart1_rxd_i : in  std_ulogic := '0'; -- UART1 receive data
    uart1_rts_o : out std_ulogic; -- hw flow control: UART1.RX ready to receive ("RTR"), low-active, optional
    uart1_cts_i : in  std_ulogic := '0'; -- hw flow control: UART1.TX allowed to transmit, low-active, optional

    -- SPI (available if IO_SPI_EN = true) --
    spi_sck_o   : out std_ulogic; -- SPI serial clock
    spi_sdo_o   : out std_ulogic; -- controller data out, peripheral data in
    spi_sdi_i   : in  std_ulogic := '0'; -- controller data in, peripheral data out
    spi_csn_o   : out std_ulogic_vector(07 downto 0); -- chip-select

    -- TWI (available if IO_TWI_EN = true) --
    twi_sda_io  : inout std_logic; -- twi serial data line
    twi_scl_io  : inout std_logic; -- twi serial clock line

    -- PWM (available if IO_PWM_EN = true) --
    pwm_o       : out std_ulogic_vector(03 downto 0); -- pwm channels

    -- Custom Functions Subsystem IO (available if IO_CFS_EN = true) --
    cfs_in_i    : in  std_ulogic_vector(IO_CFS_IN_SIZE-1  downto 0); -- custom CFS inputs conduit
    cfs_out_o   : out std_ulogic_vector(IO_CFS_OUT_SIZE-1 downto 0); -- custom CFS outputs conduit

    -- NCO output (available if IO_NCO_EN = true) --
    nco_o       : out std_ulogic_vector(02 downto 0); -- numerically-controlled oscillator channels

    -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
    neoled_o    : out std_ulogic; -- async serial data line

    -- system time input from external MTIME (available if IO_MTIME_EN = false) --
    mtime_i     : in  std_ulogic_vector(63 downto 0) := (others => '0'); -- current system time

    -- Interrupts --
    soc_firq_i  : in  std_ulogic_vector(5 downto 0) := (others => '0'); -- fast interrupt channels
    mtime_irq_i : in  std_ulogic := '0'; -- machine timer interrupt, available if IO_MTIME_EN = false
    msw_irq_i   : in  std_ulogic := '0'; -- machine software interrupt
    mext_irq_i  : in  std_ulogic := '0'  -- machine external interrupt
  );
end neorv32_top;

architecture neorv32_top_rtl of neorv32_top is

  -- CPU boot address --
  constant cpu_boot_addr_c : std_ulogic_vector(31 downto 0) := cond_sel_stdulogicvector_f(BOOTLOADER_EN, boot_rom_base_c, ispace_base_c);

  -- Bus timeout --
  constant bus_timeout_temp_c : natural := 2**index_size_f(bus_timeout_c); -- round to next power-of-two
  constant bus_timeout_proc_c : natural := cond_sel_natural_f(ICACHE_EN, ((ICACHE_BLOCK_SIZE/4)*bus_timeout_temp_c)-1, bus_timeout_c);

  -- alignment check for internal memories --
  constant imem_align_check_c : std_ulogic_vector(index_size_f(MEM_INT_IMEM_SIZE)-1 downto 0) := (others => '0');
  constant dmem_align_check_c : std_ulogic_vector(index_size_f(MEM_INT_DMEM_SIZE)-1 downto 0) := (others => '0');

  -- reset generator --
  signal rstn_i_sync0 : std_ulogic;
  signal rstn_i_sync1 : std_ulogic;
  signal rstn_i_sync2 : std_ulogic;
  signal rstn_gen     : std_ulogic_vector(3 downto 0);
  signal ext_rstn     : std_ulogic;
  signal sys_rstn     : std_ulogic;
  signal wdt_rstn     : std_ulogic;

  -- clock generator --
  signal clk_div    : std_ulogic_vector(11 downto 0);
  signal clk_div_ff : std_ulogic_vector(11 downto 0);
  signal clk_gen    : std_ulogic_vector(07 downto 0);
  signal clk_gen_en : std_ulogic_vector(08 downto 0);
  --
  signal wdt_cg_en    : std_ulogic;
  signal uart0_cg_en  : std_ulogic;
  signal uart1_cg_en  : std_ulogic;
  signal spi_cg_en    : std_ulogic;
  signal twi_cg_en    : std_ulogic;
  signal pwm_cg_en    : std_ulogic;
  signal cfs_cg_en    : std_ulogic;
  signal nco_cg_en    : std_ulogic;
  signal neoled_cg_en : std_ulogic;

  -- bus interface --
  type bus_interface_t is record
    addr   : std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
    rdata  : std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
    wdata  : std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
    ben    : std_ulogic_vector(03 downto 0); -- byte enable
    we     : std_ulogic; -- write enable
    re     : std_ulogic; -- read enable
    cancel : std_ulogic; -- cancel current transfer
    ack    : std_ulogic; -- bus transfer acknowledge
    err    : std_ulogic; -- bus transfer error
    fence  : std_ulogic; -- fence(i) instruction executed
    priv   : std_ulogic_vector(1 downto 0); -- current privilege level
    src    : std_ulogic; -- access source (1=instruction fetch, 0=data access)
    lock   : std_ulogic; -- locked/exclusive (=atomic) access
  end record;
  signal cpu_i, i_cache, cpu_d, p_bus : bus_interface_t;

  -- io space access --
  signal io_acc  : std_ulogic;
  signal io_rden : std_ulogic;
  signal io_wren : std_ulogic;

  -- read-back busses -
  signal imem_rdata     : std_ulogic_vector(data_width_c-1 downto 0);
  signal imem_ack       : std_ulogic;
  signal dmem_rdata     : std_ulogic_vector(data_width_c-1 downto 0);
  signal dmem_ack       : std_ulogic;
  signal bootrom_rdata  : std_ulogic_vector(data_width_c-1 downto 0);
  signal bootrom_ack    : std_ulogic;
  signal wishbone_rdata : std_ulogic_vector(data_width_c-1 downto 0);
  signal wishbone_ack   : std_ulogic;
  signal wishbone_err   : std_ulogic;
  signal gpio_rdata     : std_ulogic_vector(data_width_c-1 downto 0);
  signal gpio_ack       : std_ulogic;
  signal mtime_rdata    : std_ulogic_vector(data_width_c-1 downto 0);
  signal mtime_ack      : std_ulogic;
  signal uart0_rdata    : std_ulogic_vector(data_width_c-1 downto 0);
  signal uart0_ack      : std_ulogic;
  signal uart1_rdata    : std_ulogic_vector(data_width_c-1 downto 0);
  signal uart1_ack      : std_ulogic;
  signal spi_rdata      : std_ulogic_vector(data_width_c-1 downto 0);
  signal spi_ack        : std_ulogic;
  signal twi_rdata      : std_ulogic_vector(data_width_c-1 downto 0);
  signal twi_ack        : std_ulogic;
  signal pwm_rdata      : std_ulogic_vector(data_width_c-1 downto 0);
  signal pwm_ack        : std_ulogic;
  signal wdt_rdata      : std_ulogic_vector(data_width_c-1 downto 0);
  signal wdt_ack        : std_ulogic;
  signal trng_rdata     : std_ulogic_vector(data_width_c-1 downto 0);
  signal trng_ack       : std_ulogic;
  signal cfs_rdata      : std_ulogic_vector(data_width_c-1 downto 0);
  signal cfs_ack        : std_ulogic;
  signal nco_rdata      : std_ulogic_vector(data_width_c-1 downto 0);
  signal nco_ack        : std_ulogic;
  signal neoled_rdata   : std_ulogic_vector(data_width_c-1 downto 0);
  signal neoled_ack     : std_ulogic;
  signal sysinfo_rdata  : std_ulogic_vector(data_width_c-1 downto 0);
  signal sysinfo_ack    : std_ulogic;

  -- IRQs --
  signal mtime_irq    : std_ulogic;
  --
  signal fast_irq     : std_ulogic_vector(15 downto 0);
  signal fast_irq_ack : std_ulogic_vector(15 downto 0);
  --
  signal gpio_irq      : std_ulogic;
  signal wdt_irq       : std_ulogic;
  signal uart0_rxd_irq : std_ulogic;
  signal uart0_txd_irq : std_ulogic;
  signal uart1_rxd_irq : std_ulogic;
  signal uart1_txd_irq : std_ulogic;
  signal spi_irq       : std_ulogic;
  signal twi_irq       : std_ulogic;
  signal cfs_irq       : std_ulogic;
  signal cfs_irq_ack   : std_ulogic;
  signal neoled_irq    : std_ulogic;

  -- misc --
  signal mtime_time : std_ulogic_vector(63 downto 0); -- current system time from MTIME
  signal cpu_sleep  : std_ulogic; -- CPU is in sleep mode when set

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- clock --
  assert not (CLOCK_FREQUENCY = 0) report "NEORV32 PROCESSOR CONFIG ERROR! Core clock frequency (CLOCK_FREQUENCY) not specified." severity error;
  -- internal bootloader ROM --
  assert not ((BOOTLOADER_EN = true) and (boot_rom_size_c > boot_rom_max_size_c)) report "NEORV32 PROCESSOR CONFIG ERROR! Boot ROM size out of range." severity error;
  assert not ((BOOTLOADER_EN = true) and (MEM_INT_IMEM_ROM = true)) report "NEORV32 PROCESSOR CONFIG WARNING! IMEM is configured as read-only. Bootloader will not be able to load new executables." severity warning;
  -- memory system - data/instruction fetch --
  assert not ((MEM_EXT_EN = false) and (MEM_INT_DMEM_EN = false)) report "NEORV32 PROCESSOR CONFIG ERROR! Core cannot fetch data without external memory interface and internal data memory." severity error;
  assert not ((MEM_EXT_EN = false) and (MEM_INT_IMEM_EN = false) and (BOOTLOADER_EN = false)) report "NEORV32 PROCESSOR CONFIG ERROR! Core cannot fetch instructions without external memory interface, internal data memory and bootloader." severity error;
  -- memory system - size --
  assert not ((MEM_INT_DMEM_EN = true) and (is_power_of_two_f(MEM_INT_IMEM_SIZE) = false)) report "NEORV32 PROCESSOR CONFIG WARNING! MEM_INT_IMEM_SIZE should be a power of 2 to allow optimal hardware mapping." severity warning;
  assert not ((MEM_INT_IMEM_EN = true) and (is_power_of_two_f(MEM_INT_DMEM_SIZE) = false)) report "NEORV32 PROCESSOR CONFIG WARNING! MEM_INT_DMEM_SIZE should be a power of 2 to allow optimal hardware mapping." severity warning;
  -- memory system - alignment --
  assert not (ispace_base_c(1 downto 0) /= "00") report "NEORV32 PROCESSOR CONFIG ERROR! Instruction memory space base address must be 4-byte-aligned." severity error;
  assert not (dspace_base_c(1 downto 0) /= "00") report "NEORV32 PROCESSOR CONFIG ERROR! Data memory space base address must be 4-byte-aligned." severity error;
  assert not ((ispace_base_c(index_size_f(MEM_INT_IMEM_SIZE)-1 downto 0) /= imem_align_check_c) and (MEM_INT_IMEM_EN = true)) report "NEORV32 PROCESSOR CONFIG ERROR! Instruction memory space base address has to be aligned to IMEM size." severity error;
  assert not ((dspace_base_c(index_size_f(MEM_INT_DMEM_SIZE)-1 downto 0) /= dmem_align_check_c) and (MEM_INT_DMEM_EN = true)) report "NEORV32 PROCESSOR CONFIG ERROR! Data memory space base address has to be aligned to DMEM size." severity error;
  -- memory system - layout warning --
  assert not (ispace_base_c /= x"00000000") report "NEORV32 PROCESSOR CONFIG WARNING! Non-default base address for instruction address space. Make sure this is sync with the software framework." severity warning;
  assert not (dspace_base_c /= x"80000000") report "NEORV32 PROCESSOR CONFIG WARNING! Non-default base address for data address space. Make sure this is sync with the software framework." severity warning;
  -- memory system - the i-cache is intended to accelerate instruction fetch via the external memory interface only --
  assert not ((ICACHE_EN = true) and (MEM_EXT_EN = false)) report "NEORV32 PROCESSOR CONFIG NOTE. Implementing i-cache without having the external memory interface implemented. The i-cache is intended to accelerate instruction fetch via the external memory interface." severity note;
  -- memory system - cached instruction fetch latency check --
  assert not (ICACHE_EN = true) report "NEORV32 PROCESSOR CONFIG WARNING! Implementing i-cache. Increasing bus access timeout from " & integer'image(bus_timeout_c) & " cycles to " & integer'image(bus_timeout_proc_c) & " cycles." severity warning;


  -- Reset Generator ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  reset_generator_sync: process(clk_i)
  begin
    -- make sure the external reset is free of metastability and has a minimal duration of 1 clock cycle
    if rising_edge(clk_i) then
      rstn_i_sync0 <= rstn_i;
      rstn_i_sync1 <= rstn_i_sync0;
      rstn_i_sync2 <= rstn_i_sync1;
    end if;
  end process reset_generator_sync;

  -- keep internal reset active for at least 4 clock cycles
  reset_generator: process(rstn_i_sync1, rstn_i_sync2, clk_i)
  begin
    if ((rstn_i_sync1 and rstn_i_sync2) = '0') then -- signal stable?
      rstn_gen <= (others => '0');
    elsif rising_edge(clk_i) then
      rstn_gen <= rstn_gen(rstn_gen'left-1 downto 0) & '1';
    end if;
  end process reset_generator;

  ext_rstn <= rstn_gen(rstn_gen'left); -- the beautified external reset signal
  sys_rstn <= ext_rstn and wdt_rstn;   -- system reset - can also be triggered by watchdog


  -- Clock Generator ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  clock_generator: process(sys_rstn, clk_i)
  begin
    if (sys_rstn = '0') then
      clk_div    <= (others => '0');
      clk_div_ff <= (others => '0');
      clk_gen_en <= (others => '0');
    elsif rising_edge(clk_i) then
      -- fresh clocks anyone? --
      clk_gen_en(0) <= wdt_cg_en;
      clk_gen_en(1) <= uart0_cg_en;
      clk_gen_en(2) <= uart1_cg_en;
      clk_gen_en(3) <= spi_cg_en;
      clk_gen_en(4) <= twi_cg_en;
      clk_gen_en(5) <= pwm_cg_en;
      clk_gen_en(6) <= cfs_cg_en;
      clk_gen_en(7) <= nco_cg_en;
      clk_gen_en(8) <= neoled_cg_en;
      if (or_all_f(clk_gen_en) = '1') then
        clk_div <= std_ulogic_vector(unsigned(clk_div) + 1);
      end if;
      clk_div_ff <= clk_div;
    end if;
  end process clock_generator;

  -- clock enables: rising edge detectors --
  clock_generator_edge: process(clk_i)
  begin
    if rising_edge(clk_i) then
      clk_gen(clk_div2_c)    <= clk_div(0)  and (not clk_div_ff(0));  -- CLK/2
      clk_gen(clk_div4_c)    <= clk_div(1)  and (not clk_div_ff(1));  -- CLK/4
      clk_gen(clk_div8_c)    <= clk_div(2)  and (not clk_div_ff(2));  -- CLK/8
      clk_gen(clk_div64_c)   <= clk_div(5)  and (not clk_div_ff(5));  -- CLK/64
      clk_gen(clk_div128_c)  <= clk_div(6)  and (not clk_div_ff(6));  -- CLK/128
      clk_gen(clk_div1024_c) <= clk_div(9)  and (not clk_div_ff(9));  -- CLK/1024
      clk_gen(clk_div2048_c) <= clk_div(10) and (not clk_div_ff(10)); -- CLK/2048
      clk_gen(clk_div4096_c) <= clk_div(11) and (not clk_div_ff(11)); -- CLK/4096
    end if;
  end process clock_generator_edge;


  -- CPU Core -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_inst: neorv32_cpu
  generic map (
    -- General --
    HW_THREAD_ID                 => HW_THREAD_ID,        -- hardware thread id
    CPU_BOOT_ADDR                => cpu_boot_addr_c,     -- cpu boot address
    BUS_TIMEOUT                  => bus_timeout_proc_c,  -- cycles after an UNACKNOWLEDGED bus access triggers a bus fault exception
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        => CPU_EXTENSION_RISCV_A,        -- implement atomic extension?
    CPU_EXTENSION_RISCV_B        => CPU_EXTENSION_RISCV_B,        -- implement bit manipulation extensions?
    CPU_EXTENSION_RISCV_C        => CPU_EXTENSION_RISCV_C,        -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        => CPU_EXTENSION_RISCV_E,        -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_F        => false,                        -- implement 32-bit floating-point extension?
    CPU_EXTENSION_RISCV_M        => CPU_EXTENSION_RISCV_M,        -- implement muld/div extension?
    CPU_EXTENSION_RISCV_U        => CPU_EXTENSION_RISCV_U,        -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zicsr    => CPU_EXTENSION_RISCV_Zicsr,    -- implement CSR system?
    CPU_EXTENSION_RISCV_Zifencei => CPU_EXTENSION_RISCV_Zifencei, -- implement instruction stream sync.?
    -- Extension Options --
    FAST_MUL_EN                  => FAST_MUL_EN,         -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                => FAST_SHIFT_EN,       -- use barrel shifter for shift operations
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS              => PMP_NUM_REGIONS,     -- number of regions (0..64)
    PMP_MIN_GRANULARITY          => PMP_MIN_GRANULARITY, -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 => HPM_NUM_CNTS         -- number of implemented HPM counters (0..29)
  )
  port map (
    -- global control --
    clk_i          => clk_i,        -- global clock, rising edge
    rstn_i         => sys_rstn,     -- global reset, low-active, async
    sleep_o        => cpu_sleep,    -- cpu is in sleep mode when set
    -- instruction bus interface --
    i_bus_addr_o   => cpu_i.addr,   -- bus access address
    i_bus_rdata_i  => cpu_i.rdata,  -- bus read data
    i_bus_wdata_o  => cpu_i.wdata,  -- bus write data
    i_bus_ben_o    => cpu_i.ben,    -- byte enable
    i_bus_we_o     => cpu_i.we,     -- write enable
    i_bus_re_o     => cpu_i.re,     -- read enable
    i_bus_cancel_o => cpu_i.cancel, -- cancel current bus transaction
    i_bus_ack_i    => cpu_i.ack,    -- bus transfer acknowledge
    i_bus_err_i    => cpu_i.err,    -- bus transfer error
    i_bus_fence_o  => cpu_i.fence,  -- executed FENCEI operation
    i_bus_priv_o   => cpu_i.priv,   -- privilege level
    i_bus_lock_o   => cpu_i.lock,   -- locked/exclusive access
    -- data bus interface --
    d_bus_addr_o   => cpu_d.addr,   -- bus access address
    d_bus_rdata_i  => cpu_d.rdata,  -- bus read data
    d_bus_wdata_o  => cpu_d.wdata,  -- bus write data
    d_bus_ben_o    => cpu_d.ben,    -- byte enable
    d_bus_we_o     => cpu_d.we,     -- write enable
    d_bus_re_o     => cpu_d.re,     -- read enable
    d_bus_cancel_o => cpu_d.cancel, -- cancel current bus transaction
    d_bus_ack_i    => cpu_d.ack,    -- bus transfer acknowledge
    d_bus_err_i    => cpu_d.err,    -- bus transfer error
    d_bus_fence_o  => cpu_d.fence,  -- executed FENCE operation
    d_bus_priv_o   => cpu_d.priv,   -- privilege level
    d_bus_lock_o   => cpu_d.lock,   -- locked/exclusive access
    -- system time input from MTIME --
    time_i         => mtime_time,   -- current system time
    -- interrupts (risc-v compliant) --
    msw_irq_i      => msw_irq_i,    -- machine software interrupt
    mext_irq_i     => mext_irq_i,   -- machine external interrupt request
    mtime_irq_i    => mtime_irq,    -- machine timer interrupt
    -- fast interrupts (custom) --
    firq_i         => fast_irq,     -- fast interrupt trigger
    firq_ack_o     => fast_irq_ack  -- fast interrupt acknowledge mask
  );

  -- misc --
  cpu_i.src <= '1'; -- initialized but unused
  cpu_d.src <= '0'; -- initialized but unused

  -- advanced memory control --
  fence_o  <= cpu_d.fence; -- indicates an executed FENCE operation
  fencei_o <= cpu_i.fence; -- indicates an executed FENCEI operation

  -- fast interrupts - processor-internal --
  fast_irq(00) <= wdt_irq;       -- HIGHEST PRIORITY - watchdog timeout
  fast_irq(01) <= cfs_irq;       -- custom functions subsystem
  fast_irq(02) <= uart0_rxd_irq; -- primary UART (UART0) data received
  fast_irq(03) <= uart0_txd_irq; -- primary UART (UART0) sending done
  fast_irq(04) <= uart1_rxd_irq; -- secondary UART (UART1) data received
  fast_irq(05) <= uart1_txd_irq; -- secondary UART (UART1) sending done
  fast_irq(06) <= spi_irq;       -- SPI transmission done
  fast_irq(07) <= twi_irq;       -- TWI transmission done
  fast_irq(08) <= gpio_irq;      -- GPIO pin-change
  fast_irq(09) <= neoled_irq;    -- NEOLED buffer free

  -- fast interrupts - platform level (for custom use) --
  fast_irq(10) <= soc_firq_i(0);
  fast_irq(11) <= soc_firq_i(1);
  fast_irq(12) <= soc_firq_i(2);
  fast_irq(13) <= soc_firq_i(3);
  fast_irq(14) <= soc_firq_i(4);
  fast_irq(15) <= soc_firq_i(5);

  -- CFS IRQ acknowledge --
  cfs_irq_ack <= fast_irq_ack(1);


  -- CPU Instruction Cache ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_icache_inst_true:
  if (ICACHE_EN = true) generate
    neorv32_icache_inst: neorv32_icache
    generic map (
      ICACHE_NUM_BLOCKS => ICACHE_NUM_BLOCKS,   -- number of blocks (min 2), has to be a power of 2
      ICACHE_BLOCK_SIZE => ICACHE_BLOCK_SIZE,   -- block size in bytes (min 4), has to be a power of 2
      ICACHE_NUM_SETS   => ICACHE_ASSOCIATIVITY -- associativity / number of sets (1=direct_mapped), has to be a power of 2
    )
    port map (
      -- global control --
      clk_i         => clk_i,          -- global clock, rising edge
      rstn_i        => sys_rstn,       -- global reset, low-active, async
      clear_i       => cpu_i.fence,    -- cache clear
      -- host controller interface --
      host_addr_i   => cpu_i.addr,     -- bus access address
      host_rdata_o  => cpu_i.rdata,    -- bus read data
      host_wdata_i  => cpu_i.wdata,    -- bus write data
      host_ben_i    => cpu_i.ben,      -- byte enable
      host_we_i     => cpu_i.we,       -- write enable
      host_re_i     => cpu_i.re,       -- read enable
      host_cancel_i => cpu_i.cancel,   -- cancel current bus transaction
      host_lock_i   => cpu_i.lock,     -- locked/exclusive access
      host_ack_o    => cpu_i.ack,      -- bus transfer acknowledge
      host_err_o    => cpu_i.err,      -- bus transfer error
      -- peripheral bus interface --
      bus_addr_o    => i_cache.addr,   -- bus access address
      bus_rdata_i   => i_cache.rdata,  -- bus read data
      bus_wdata_o   => i_cache.wdata,  -- bus write data
      bus_ben_o     => i_cache.ben,    -- byte enable
      bus_we_o      => i_cache.we,     -- write enable
      bus_re_o      => i_cache.re,     -- read enable
      bus_cancel_o  => i_cache.cancel, -- cancel current bus transaction
      bus_lock_o    => i_cache.lock,   -- locked/exclusive access
      bus_ack_i     => i_cache.ack,    -- bus transfer acknowledge
      bus_err_i     => i_cache.err     -- bus transfer error
    );
  end generate;

  neorv32_icache_inst_false:
  if (ICACHE_EN = false) generate
    i_cache.addr   <= cpu_i.addr;
    cpu_i.rdata    <= i_cache.rdata;
    i_cache.wdata  <= cpu_i.wdata;
    i_cache.ben    <= cpu_i.ben;
    i_cache.we     <= cpu_i.we;
    i_cache.re     <= cpu_i.re;
    i_cache.cancel <= cpu_i.cancel;
    i_cache.lock   <= cpu_i.lock;
    cpu_i.ack      <= i_cache.ack;
    cpu_i.err      <= i_cache.err;
  end generate;


  -- CPU Bus Switch -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_busswitch_inst: neorv32_busswitch
  generic map (
    PORT_CA_READ_ONLY => false, -- set if controller port A is read-only
    PORT_CB_READ_ONLY => true   -- set if controller port B is read-only
  )
  port map (
    -- global control --
    clk_i           => clk_i,          -- global clock, rising edge
    rstn_i          => sys_rstn,       -- global reset, low-active, async
    -- controller interface a --
    ca_bus_addr_i   => cpu_d.addr,     -- bus access address
    ca_bus_rdata_o  => cpu_d.rdata,    -- bus read data
    ca_bus_wdata_i  => cpu_d.wdata,    -- bus write data
    ca_bus_ben_i    => cpu_d.ben,      -- byte enable
    ca_bus_we_i     => cpu_d.we,       -- write enable
    ca_bus_re_i     => cpu_d.re,       -- read enable
    ca_bus_cancel_i => cpu_d.cancel,   -- cancel current bus transaction
    ca_bus_lock_i   => cpu_d.lock,     -- locked/exclusive access
    ca_bus_ack_o    => cpu_d.ack,      -- bus transfer acknowledge
    ca_bus_err_o    => cpu_d.err,      -- bus transfer error
    -- controller interface b --
    cb_bus_addr_i   => i_cache.addr,   -- bus access address
    cb_bus_rdata_o  => i_cache.rdata,  -- bus read data
    cb_bus_wdata_i  => i_cache.wdata,  -- bus write data
    cb_bus_ben_i    => i_cache.ben,    -- byte enable
    cb_bus_we_i     => i_cache.we,     -- write enable
    cb_bus_re_i     => i_cache.re,     -- read enable
    cb_bus_cancel_i => i_cache.cancel, -- cancel current bus transaction
    cb_bus_lock_i   => i_cache.lock,   -- locked/exclusive access
    cb_bus_ack_o    => i_cache.ack,    -- bus transfer acknowledge
    cb_bus_err_o    => i_cache.err,    -- bus transfer error
    -- peripheral bus --
    p_bus_src_o     => p_bus.src,      -- access source: 0 = A (data), 1 = B (instructions)
    p_bus_addr_o    => p_bus.addr,     -- bus access address
    p_bus_rdata_i   => p_bus.rdata,    -- bus read data
    p_bus_wdata_o   => p_bus.wdata,    -- bus write data
    p_bus_ben_o     => p_bus.ben,      -- byte enable
    p_bus_we_o      => p_bus.we,       -- write enable
    p_bus_re_o      => p_bus.re,       -- read enable
    p_bus_cancel_o  => p_bus.cancel,   -- cancel current bus transaction
    p_bus_lock_o    => p_bus.lock,     -- locked/exclusive access
    p_bus_ack_i     => p_bus.ack,      -- bus transfer acknowledge
    p_bus_err_i     => p_bus.err       -- bus transfer error
  );

  -- processor bus: CPU transfer data input --
  p_bus.rdata <= (imem_rdata or dmem_rdata or bootrom_rdata) or wishbone_rdata or (gpio_rdata or mtime_rdata or uart0_rdata or uart1_rdata or
                 spi_rdata or twi_rdata or pwm_rdata or wdt_rdata or trng_rdata or cfs_rdata or nco_rdata or neoled_rdata or  sysinfo_rdata);

  -- processor bus: CPU transfer ACK input --
  p_bus.ack <= (imem_ack or dmem_ack or bootrom_ack) or wishbone_ack or (gpio_ack or mtime_ack or uart0_ack or uart1_ack or
               spi_ack or twi_ack or pwm_ack or wdt_ack or trng_ack or cfs_ack or nco_ack or neoled_ack or sysinfo_ack);

  -- processor bus: CPU transfer data bus error input --
  p_bus.err <= wishbone_err;

  -- current CPU privilege level --
  p_bus.priv <= cpu_i.priv; -- cpu_i.priv == cpu_d.priv


  -- Processor-Internal Instruction Memory (IMEM) -------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_int_imem_inst_true:
  if (MEM_INT_IMEM_EN = true) generate
    neorv32_int_imem_inst: neorv32_imem
    generic map (
      IMEM_BASE      => imem_base_c,       -- memory base address
      IMEM_SIZE      => MEM_INT_IMEM_SIZE, -- processor-internal instruction memory size in bytes
      IMEM_AS_ROM    => MEM_INT_IMEM_ROM,  -- implement IMEM as read-only memory?
      BOOTLOADER_EN  => BOOTLOADER_EN      -- implement and use bootloader?
    )
    port map (
      clk_i  => clk_i,       -- global clock line
      rden_i => p_bus.re,    -- read enable
      wren_i => p_bus.we,    -- write enable
      ben_i  => p_bus.ben,   -- byte write enable
      addr_i => p_bus.addr,  -- address
      data_i => p_bus.wdata, -- data in
      data_o => imem_rdata,  -- data out
      ack_o  => imem_ack     -- transfer acknowledge
    );
  end generate;

  neorv32_int_imem_inst_false:
  if (MEM_INT_IMEM_EN = false) generate
    imem_rdata <= (others => '0');
    imem_ack   <= '0';
  end generate;


  -- Processor-Internal Data Memory (DMEM) --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_int_dmem_inst_true:
  if (MEM_INT_DMEM_EN = true) generate
    neorv32_int_dmem_inst: neorv32_dmem
    generic map (
      DMEM_BASE => dmem_base_c,      -- memory base address
      DMEM_SIZE => MEM_INT_DMEM_SIZE -- processor-internal data memory size in bytes
    )
    port map (
      clk_i  => clk_i,       -- global clock line
      rden_i => p_bus.re,    -- read enable
      wren_i => p_bus.we,    -- write enable
      ben_i  => p_bus.ben,   -- byte write enable
      addr_i => p_bus.addr,  -- address
      data_i => p_bus.wdata, -- data in
      data_o => dmem_rdata,  -- data out
      ack_o  => dmem_ack     -- transfer acknowledge
    );
  end generate;

  neorv32_int_dmem_inst_false:
  if (MEM_INT_DMEM_EN = false) generate
    dmem_rdata <= (others => '0');
    dmem_ack   <= '0';
  end generate;


  -- Processor-Internal Bootloader ROM (BOOTROM) --------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_boot_rom_inst_true:
  if (BOOTLOADER_EN = true) generate
    neorv32_boot_rom_inst: neorv32_boot_rom
    generic map (
      BOOTROM_BASE => boot_rom_base_c, -- boot ROM base address
      BOOTROM_SIZE => boot_rom_size_c  -- processor-internal boot TOM memory size in bytes
    )
    port map (
      clk_i  => clk_i,         -- global clock line
      rden_i => p_bus.re,      -- read enable
      addr_i => p_bus.addr,    -- address
      data_o => bootrom_rdata, -- data out
      ack_o  => bootrom_ack    -- transfer acknowledge
    );
  end generate;

  neorv32_boot_rom_inst_false:
  if (BOOTLOADER_EN = false) generate
    bootrom_rdata <= (others => '0');
    bootrom_ack   <= '0';
  end generate;


  -- External Wishbone Gateway (WISHBONE) ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_wishbone_inst_true:
  if (MEM_EXT_EN = true) generate
    neorv32_wishbone_inst: neorv32_wishbone
    generic map (
      WB_PIPELINED_MODE => wb_pipe_mode_c,    -- false: classic/standard wishbone mode, true: pipelined wishbone mode
      -- Internal instruction memory --
      MEM_INT_IMEM_EN   => MEM_INT_IMEM_EN,   -- implement processor-internal instruction memory
      MEM_INT_IMEM_SIZE => MEM_INT_IMEM_SIZE, -- size of processor-internal instruction memory in bytes
      -- Internal data memory --
      MEM_INT_DMEM_EN   => MEM_INT_DMEM_EN,   -- implement processor-internal data memory
      MEM_INT_DMEM_SIZE => MEM_INT_DMEM_SIZE  -- size of processor-internal data memory in bytes
    )
    port map (
      -- global control --
      clk_i     => clk_i,          -- global clock line
      rstn_i    => sys_rstn,       -- global reset line, low-active
      -- host access --
      src_i     => p_bus.src,      -- access type (0: data, 1:instruction)
      addr_i    => p_bus.addr,     -- address
      rden_i    => p_bus.re,       -- read enable
      wren_i    => p_bus.we,       -- write enable
      ben_i     => p_bus.ben,      -- byte write enable
      data_i    => p_bus.wdata,    -- data in
      data_o    => wishbone_rdata, -- data out
      cancel_i  => p_bus.cancel,   -- cancel current transaction
      lock_i    => p_bus.lock,     -- locked/exclusive bus access
      ack_o     => wishbone_ack,   -- transfer acknowledge
      err_o     => wishbone_err,   -- transfer error
      priv_i    => p_bus.priv,     -- current CPU privilege level
      -- wishbone interface --
      wb_tag_o  => wb_tag_o,       -- tag
      wb_adr_o  => wb_adr_o,       -- address
      wb_dat_i  => wb_dat_i,       -- read data
      wb_dat_o  => wb_dat_o,       -- write data
      wb_we_o   => wb_we_o,        -- read/write
      wb_sel_o  => wb_sel_o,       -- byte enable
      wb_stb_o  => wb_stb_o,       -- strobe
      wb_cyc_o  => wb_cyc_o,       -- valid cycle
      wb_lock_o => wb_lock_o,      -- locked/exclusive bus access
      wb_ack_i  => wb_ack_i,       -- transfer acknowledge
      wb_err_i  => wb_err_i        -- transfer error
    );
  end generate;

  neorv32_wishbone_inst_false:
  if (MEM_EXT_EN = false) generate
    wishbone_rdata <= (others => '0');
    wishbone_ack   <= '0';
    wishbone_err   <= '0';
    --
    wb_adr_o  <= (others => '0');
    wb_dat_o  <= (others => '0');
    wb_we_o   <= '0';
    wb_sel_o  <= (others => '0');
    wb_stb_o  <= '0';
    wb_cyc_o  <= '0';
    wb_lock_o <= '0';
    wb_tag_o  <= (others => '0');
  end generate;


  -- IO Access? -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  io_acc  <= '1' when (p_bus.addr(data_width_c-1 downto index_size_f(io_size_c)) = io_base_c(data_width_c-1 downto index_size_f(io_size_c))) else '0';
  io_rden <= io_acc and p_bus.re and (not p_bus.src); -- PMA: no_execute for IO region
  -- the default NEORV32 peripheral/IO devices in the IO area can only be written in word mode (reduces HW complexity)
  io_wren <= io_acc and p_bus.we and and_all_f(p_bus.ben) and (not p_bus.src); -- PMA: write32 only, no_execute for IO region


  -- Custom Functions Subsystem (CFS) -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cfs_inst_true:
  if (IO_CFS_EN = true) generate
    neorv32_cfs_inst: neorv32_cfs
    generic map (
      CFS_CONFIG   => IO_CFS_CONFIG,  -- custom CFS configuration generic 
      CFS_IN_SIZE  => IO_CFS_IN_SIZE, -- size of CFS input conduit in bits
      CFS_OUT_SIZE => IO_CFS_OUT_SIZE -- size of CFS output conduit in bits
    )
    port map (
      -- host access --
      clk_i       => clk_i,           -- global clock line
      rstn_i      => sys_rstn,        -- global reset line, low-active, use as async
      addr_i      => p_bus.addr,      -- address
      rden_i      => io_rden,         -- read enable
      wren_i      => io_wren,         -- byte write enable
      data_i      => p_bus.wdata,     -- data in
      data_o      => cfs_rdata,       -- data out
      ack_o       => cfs_ack,         -- transfer acknowledge
      -- clock generator --
      clkgen_en_o => cfs_cg_en,       -- enable clock generator
      clkgen_i    => clk_gen,         -- "clock" inputs
      -- CPU state --
      sleep_i     => cpu_sleep,       -- set if cpu is in sleep mode
      -- interrupt --
      irq_o       => cfs_irq,         -- interrupt request
      irq_ack_i   => cfs_irq_ack,     -- interrupt acknowledge
      -- custom io (conduit) --
      cfs_in_i    => cfs_in_i,        -- custom inputs
      cfs_out_o   => cfs_out_o        -- custom outputs
    );
  end generate;

  neorv32_cfs_inst_false:
  if (IO_CFS_EN = false) generate
    cfs_rdata <= (others => '0');
    cfs_ack   <= '0';
    cfs_cg_en <= '0';
    cfs_irq   <= '0';
    cfs_out_o <= (others => '0');
  end generate;


  -- General Purpose Input/Output Port (GPIO) -----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_gpio_inst_true:
  if (IO_GPIO_EN = true) generate
    neorv32_gpio_inst: neorv32_gpio
    port map (
      -- host access --
      clk_i  => clk_i,       -- global clock line
      addr_i => p_bus.addr,  -- address
      rden_i => io_rden,     -- read enable
      wren_i => io_wren,     -- write enable
      data_i => p_bus.wdata, -- data in
      data_o => gpio_rdata,  -- data out
      ack_o  => gpio_ack,    -- transfer acknowledge
      -- parallel io --
      gpio_o => gpio_o,
      gpio_i => gpio_i,
      -- interrupt --
      irq_o  => gpio_irq     -- pin-change interrupt
    );
  end generate;

  neorv32_gpio_inst_false:
  if (IO_GPIO_EN = false) generate
    gpio_rdata <= (others => '0');
    gpio_ack   <= '0';
    gpio_o     <= (others => '0');
    gpio_irq   <= '0';
  end generate;


  -- Watch Dog Timer (WDT) ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_wdt_inst_true:
  if (IO_WDT_EN = true) generate
    neorv32_wdt_inst: neorv32_wdt
    port map (
      -- host access --
      clk_i       => clk_i,       -- global clock line
      rstn_i      => ext_rstn,    -- global reset line, low-active
      rden_i      => io_rden,     -- read enable
      wren_i      => io_wren,     -- write enable
      addr_i      => p_bus.addr,  -- address
      data_i      => p_bus.wdata, -- data in
      data_o      => wdt_rdata,   -- data out
      ack_o       => wdt_ack,     -- transfer acknowledge
      -- clock generator --
      clkgen_en_o => wdt_cg_en,   -- enable clock generator
      clkgen_i    => clk_gen,
      -- timeout event --
      irq_o       => wdt_irq,     -- timeout IRQ
      rstn_o      => wdt_rstn     -- timeout reset, low_active, use it as async!
    );
  end generate;

  neorv32_wdt_inst_false:
  if (IO_WDT_EN = false) generate
    wdt_rdata <= (others => '0');
    wdt_ack   <= '0';
    wdt_irq   <= '0';
    wdt_rstn  <= '1';
    wdt_cg_en <= '0';
  end generate;


  -- Machine System Timer (MTIME) -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_mtime_inst_true:
  if (IO_MTIME_EN = true) generate
    neorv32_mtime_inst: neorv32_mtime
    port map (
      -- host access --
      clk_i     => clk_i,       -- global clock line
      rstn_i    => sys_rstn,    -- global reset, low-active, async
      addr_i    => p_bus.addr,  -- address
      rden_i    => io_rden,     -- read enable
      wren_i    => io_wren,     -- write enable
      data_i    => p_bus.wdata, -- data in
      data_o    => mtime_rdata, -- data out
      ack_o     => mtime_ack,   -- transfer acknowledge
      -- time output for CPU --
      time_o    => mtime_time,  -- current system time
      -- interrupt --
      irq_o     => mtime_irq    -- interrupt request
    );
  end generate;

  neorv32_mtime_inst_false:
  if (IO_MTIME_EN = false) generate
    mtime_rdata <= (others => '0');
    mtime_time  <= mtime_i; -- use external machine timer time signal
    mtime_ack   <= '0';
    mtime_irq   <= mtime_irq_i; -- use external machine timer interrupt
  end generate;


  -- Primary Universal Asynchronous Receiver/Transmitter (UART0) ----------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_uart0_inst_true:
  if (IO_UART0_EN = true) generate
    neorv32_uart0_inst: neorv32_uart
    generic map (
      UART_PRIMARY => true -- true = primary UART (UART0), false = secondary UART (UART1)
    )
    port map (
      -- host access --
      clk_i       => clk_i,         -- global clock line
      addr_i      => p_bus.addr,    -- address
      rden_i      => io_rden,       -- read enable
      wren_i      => io_wren,       -- write enable
      data_i      => p_bus.wdata,   -- data in
      data_o      => uart0_rdata,   -- data out
      ack_o       => uart0_ack,     -- transfer acknowledge
      -- clock generator --
      clkgen_en_o => uart0_cg_en,   -- enable clock generator
      clkgen_i    => clk_gen,
      -- com lines --
      uart_txd_o  => uart0_txd_o,
      uart_rxd_i  => uart0_rxd_i,
      -- hardware flow control --
      uart_rts_o  => uart0_rts_o,   -- UART.RX ready to receive ("RTR"), low-active, optional
      uart_cts_i  => uart0_cts_i,   -- UART.TX allowed to transmit, low-active, optional
      -- interrupts --
      irq_rxd_o   => uart0_rxd_irq, -- uart data received interrupt
      irq_txd_o   => uart0_txd_irq  -- uart transmission done interrupt
    );
  end generate;

  neorv32_uart0_inst_false:
  if (IO_UART0_EN = false) generate
    uart0_rdata   <= (others => '0');
    uart0_ack     <= '0';
    uart0_txd_o   <= '0';
    uart0_rts_o   <= '0';
    uart0_cg_en   <= '0';
    uart0_rxd_irq <= '0';
    uart0_txd_irq <= '0';
  end generate;


  -- Secondary Universal Asynchronous Receiver/Transmitter (UART1) --------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_uart1_inst_true:
  if (IO_UART1_EN = true) generate
    neorv32_uart1_inst: neorv32_uart
    generic map (
      UART_PRIMARY => false -- true = primary UART (UART0), false = secondary UART (UART1)
    )
    port map (
      -- host access --
      clk_i       => clk_i,         -- global clock line
      addr_i      => p_bus.addr,    -- address
      rden_i      => io_rden,       -- read enable
      wren_i      => io_wren,       -- write enable
      data_i      => p_bus.wdata,   -- data in
      data_o      => uart1_rdata,   -- data out
      ack_o       => uart1_ack,     -- transfer acknowledge
      -- clock generator --
      clkgen_en_o => uart1_cg_en,   -- enable clock generator
      clkgen_i    => clk_gen,
      -- com lines --
      uart_txd_o  => uart1_txd_o,
      uart_rxd_i  => uart1_rxd_i,
      -- hardware flow control --
      uart_rts_o  => uart1_rts_o,   -- UART.RX ready to receive ("RTR"), low-active, optional
      uart_cts_i  => uart1_cts_i,   -- UART.TX allowed to transmit, low-active, optional
      -- interrupts --
      irq_rxd_o   => uart1_rxd_irq, -- uart data received interrupt
      irq_txd_o   => uart1_txd_irq  -- uart transmission done interrupt
    );
  end generate;

  neorv32_uart1_inst_false:
  if (IO_UART1_EN = false) generate
    uart1_rdata   <= (others => '0');
    uart1_ack     <= '0';
    uart1_txd_o   <= '0';
    uart1_rts_o   <= '0';
    uart1_cg_en   <= '0';
    uart1_rxd_irq <= '0';
    uart1_txd_irq <= '0';
  end generate;


  -- Serial Peripheral Interface (SPI) ------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_spi_inst_true:
  if (IO_SPI_EN = true) generate
    neorv32_spi_inst: neorv32_spi
    port map (
      -- host access --
      clk_i       => clk_i,       -- global clock line
      addr_i      => p_bus.addr,  -- address
      rden_i      => io_rden,     -- read enable
      wren_i      => io_wren,     -- write enable
      data_i      => p_bus.wdata, -- data in
      data_o      => spi_rdata,   -- data out
      ack_o       => spi_ack,     -- transfer acknowledge
      -- clock generator --
      clkgen_en_o => spi_cg_en,   -- enable clock generator
      clkgen_i    => clk_gen,
      -- com lines --
      spi_sck_o   => spi_sck_o,   -- SPI serial clock
      spi_sdo_o   => spi_sdo_o,   -- controller data out, peripheral data in
      spi_sdi_i   => spi_sdi_i,   -- controller data in, peripheral data out
      spi_csn_o   => spi_csn_o,   -- SPI CS
      -- interrupt --
      irq_o       => spi_irq      -- transmission done interrupt
    );
  end generate;

  neorv32_spi_inst_false:
  if (IO_SPI_EN = false) generate
    spi_rdata  <= (others => '0');
    spi_ack    <= '0';
    spi_sck_o  <= '0';
    spi_sdo_o  <= '0';
    spi_csn_o  <= (others => '1'); -- CSn lines are low-active
    spi_cg_en  <= '0';
    spi_irq    <= '0';
  end generate;


  -- Two-Wire Interface (TWI) ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_twi_inst_true:
  if (IO_TWI_EN = true) generate
    neorv32_twi_inst: neorv32_twi
    port map (
      -- host access --
      clk_i       => clk_i,       -- global clock line
      addr_i      => p_bus.addr,  -- address
      rden_i      => io_rden,     -- read enable
      wren_i      => io_wren,     -- write enable
      data_i      => p_bus.wdata, -- data in
      data_o      => twi_rdata,   -- data out
      ack_o       => twi_ack,     -- transfer acknowledge
      -- clock generator --
      clkgen_en_o => twi_cg_en,   -- enable clock generator
      clkgen_i    => clk_gen,
      -- com lines --
      twi_sda_io  => twi_sda_io,  -- serial data line
      twi_scl_io  => twi_scl_io,  -- serial clock line
      -- interrupt --
      irq_o       => twi_irq      -- transfer done IRQ
    );
  end generate;

  neorv32_twi_inst_false:
  if (IO_TWI_EN = false) generate
    twi_rdata  <= (others => '0');
    twi_ack    <= '0';
--  twi_sda_io <= 'Z'; -- FIXME?
--  twi_scl_io <= 'Z'; -- FIXME?
    twi_cg_en  <= '0';
    twi_irq    <= '0';
  end generate;


  -- Pulse-Width Modulation Controller (PWM) ------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_pwm_inst_true:
  if (IO_PWM_EN = true) generate
    neorv32_pwm_inst: neorv32_pwm
    port map (
      -- host access --
      clk_i       => clk_i,       -- global clock line
      addr_i      => p_bus.addr,  -- address
      rden_i      => io_rden,     -- read enable
      wren_i      => io_wren,     -- write enable
      data_i      => p_bus.wdata, -- data in
      data_o      => pwm_rdata,   -- data out
      ack_o       => pwm_ack,     -- transfer acknowledge
      -- clock generator --
      clkgen_en_o => pwm_cg_en,   -- enable clock generator
      clkgen_i    => clk_gen,
      -- pwm output channels --
      pwm_o       => pwm_o
    );
  end generate;

  neorv32_pwm_inst_false:
  if (IO_PWM_EN = false) generate
    pwm_rdata <= (others => '0');
    pwm_ack   <= '0';
    pwm_cg_en <= '0';
    pwm_o     <= (others => '0');
  end generate;


  -- Numerically-Controlled Oscillator (NCO) ------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_nco_inst_true:
  if (IO_NCO_EN = true) generate
    neorv32_nco_inst: neorv32_nco
    port map (
      -- host access --
      clk_i       => clk_i,       -- global clock line
      addr_i      => p_bus.addr,  -- address
      rden_i      => io_rden,     -- read enable
      wren_i      => io_wren,     -- write enable
      data_i      => p_bus.wdata, -- data in
      data_o      => nco_rdata,   -- data out
      ack_o       => nco_ack,     -- transfer acknowledge
      -- clock generator --
      clkgen_en_o => nco_cg_en,   -- enable clock generator
      clkgen_i    => clk_gen,
      -- NCO output --
      nco_o       => nco_o
    );
  end generate;

  neorv32_nco_inst_false:
  if (IO_NCO_EN = false) generate
    nco_rdata <= (others => '0');
    nco_ack   <= '0';
    nco_cg_en <= '0';
    nco_o     <= (others => '0');
  end generate;


  -- True Random Number Generator (TRNG) ----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_trng_inst_true:
  if (IO_TRNG_EN = true) generate
    neorv32_trng_inst: neorv32_trng
    port map (
      -- host access --
      clk_i  => clk_i,       -- global clock line
      addr_i => p_bus.addr,  -- address
      rden_i => io_rden,     -- read enable
      wren_i => io_wren,     -- write enable
      data_i => p_bus.wdata, -- data in
      data_o => trng_rdata,  -- data out
      ack_o  => trng_ack     -- transfer acknowledge
    );
  end generate;

  neorv32_trng_inst_false:
  if (IO_TRNG_EN = false) generate
    trng_rdata <= (others => '0');
    trng_ack   <= '0';
  end generate;


  -- Smart LED (WS2811/WS2812) Interface (NEOLED) -------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_neoled_inst_true:
  if (IO_NEOLED_EN = true) generate
    neorv32_neoled_inst: neorv32_neoled
    port map (
      -- host access --
      clk_i       => clk_i,        -- global clock line
      addr_i      => p_bus.addr,   -- address
      rden_i      => io_rden,      -- read enable
      wren_i      => io_wren,      -- write enable
      data_i      => p_bus.wdata,  -- data in
      data_o      => neoled_rdata, -- data out
      ack_o       => neoled_ack,   -- transfer acknowledge
      -- clock generator --
      clkgen_en_o => neoled_cg_en, -- enable clock generator
      clkgen_i    => clk_gen,
      -- interrupt --
      irq_o       => neoled_irq,   -- interrupt request
      -- NEOLED output --
      neoled_o    => neoled_o      -- serial async data line
    );
  end generate;

  neorv32_neoled_inst_false:
  if (IO_NEOLED_EN = false) generate
    neoled_rdata <= (others => '0');
    neoled_ack   <= '0';
    neoled_cg_en <= '0';
    neoled_irq   <= '0';
    neoled_o     <= '0';
  end generate;


  -- System Configuration Information Memory (SYSINFO) --------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_sysinfo_inst: neorv32_sysinfo
  generic map (
    -- General --
    CLOCK_FREQUENCY      => CLOCK_FREQUENCY,      -- clock frequency of clk_i in Hz
    BOOTLOADER_EN        => BOOTLOADER_EN,        -- implement processor-internal bootloader?
    USER_CODE            => USER_CODE,            -- custom user code
    -- internal Instruction memory --
    MEM_INT_IMEM_EN      => MEM_INT_IMEM_EN,      -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE    => MEM_INT_IMEM_SIZE,    -- size of processor-internal instruction memory in bytes
    MEM_INT_IMEM_ROM     => MEM_INT_IMEM_ROM,     -- implement processor-internal instruction memory as ROM
    -- Internal Data memory --
    MEM_INT_DMEM_EN      => MEM_INT_DMEM_EN,      -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE    => MEM_INT_DMEM_SIZE,    -- size of processor-internal data memory in bytes
    -- Internal Cache memory --
    ICACHE_EN            => ICACHE_EN,            -- implement instruction cache
    ICACHE_NUM_BLOCKS    => ICACHE_NUM_BLOCKS,    -- i-cache: number of blocks (min 2), has to be a power of 2
    ICACHE_BLOCK_SIZE    => ICACHE_BLOCK_SIZE,    -- i-cache: block size in bytes (min 4), has to be a power of 2
    ICACHE_ASSOCIATIVITY => ICACHE_ASSOCIATIVITY, -- i-cache: associativity (min 1), has to be a power 2
    -- External memory interface --
    MEM_EXT_EN           => MEM_EXT_EN,           -- implement external memory bus interface?
    -- Processor peripherals --
    IO_GPIO_EN           => IO_GPIO_EN,           -- implement general purpose input/output port unit (GPIO)?
    IO_MTIME_EN          => IO_MTIME_EN,          -- implement machine system timer (MTIME)?
    IO_UART0_EN          => IO_UART0_EN,          -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_UART1_EN          => IO_UART1_EN,          -- implement secondary universal asynchronous receiver/transmitter (UART1)?
    IO_SPI_EN            => IO_SPI_EN,            -- implement serial peripheral interface (SPI)?
    IO_TWI_EN            => IO_TWI_EN,            -- implement two-wire interface (TWI)?
    IO_PWM_EN            => IO_PWM_EN,            -- implement pulse-width modulation unit (PWM)?
    IO_WDT_EN            => IO_WDT_EN,            -- implement watch dog timer (WDT)?
    IO_TRNG_EN           => IO_TRNG_EN,           -- implement true random number generator (TRNG)?
    IO_CFS_EN            => IO_CFS_EN,            -- implement custom functions subsystem (CFS)?
    IO_NCO_EN            => IO_NCO_EN,            -- implement numerically-controlled oscillator (NCO)?
    IO_NEOLED_EN         => IO_NEOLED_EN          -- implement NeoPixel-compatible smart LED interface (NEOLED)?
  )
  port map (
    -- host access --
    clk_i  => clk_i,         -- global clock line
    addr_i => p_bus.addr,    -- address
    rden_i => io_rden,       -- read enable
    data_o => sysinfo_rdata, -- data out
    ack_o  => sysinfo_ack    -- transfer acknowledge
  );


end neorv32_top_rtl;
