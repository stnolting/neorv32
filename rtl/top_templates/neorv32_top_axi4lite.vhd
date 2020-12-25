-- #################################################################################################
-- # << NEORV32 - Processor Top Entity with AXI4-Lite Compatible Master Interface >>               #
-- # ********************************************************************************************* #
-- # "AXI", "AXI4" and "AXI4-Lite" are trademarks of Arm Holdings plc.                             #
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

entity neorv32_top_axi4lite is
  generic (
    -- General --
    CLOCK_FREQUENCY              : natural := 0;      -- clock frequency of clk_i in Hz
    BOOTLOADER_USE               : boolean := true;   -- implement processor-internal bootloader?
    USER_CODE                    : std_logic_vector(31 downto 0) := x"00000000"; -- custom user code
    HW_THREAD_ID                 : std_logic_vector(31 downto 0) := (others => '0'); -- hardware thread id (hartid)
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        : boolean := false;  -- implement atomic extension?
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
    PMP_USE                      : boolean := false;  -- implement PMP?
    -- Internal Instruction memory --
    MEM_INT_IMEM_USE             : boolean := true;   -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            : natural := 16*1024; -- size of processor-internal instruction memory in bytes
    MEM_INT_IMEM_ROM             : boolean := false;  -- implement processor-internal instruction memory as ROM
    -- Internal Data memory --
    MEM_INT_DMEM_USE             : boolean := true;   -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            : natural := 8*1024; -- size of processor-internal data memory in bytes
    -- Internal Cache memory --
    ICACHE_USE                   : boolean := false;  -- implement instruction cache
    ICACHE_NUM_BLOCKS            : natural := 4;      -- i-cache: number of blocks (min 1), has to be a power of 2
    ICACHE_BLOCK_SIZE            : natural := 64;     -- i-cache: block size in bytes (min 4), has to be a power of 2
    -- Processor peripherals --
    IO_GPIO_USE                  : boolean := true;   -- implement general purpose input/output port unit (GPIO)?
    IO_MTIME_USE                 : boolean := true;   -- implement machine system timer (MTIME)?
    IO_UART_USE                  : boolean := true;   -- implement universal asynchronous receiver/transmitter (UART)?
    IO_SPI_USE                   : boolean := true;   -- implement serial peripheral interface (SPI)?
    IO_TWI_USE                   : boolean := true;   -- implement two-wire interface (TWI)?
    IO_PWM_USE                   : boolean := true;   -- implement pulse-width modulation unit (PWM)?
    IO_WDT_USE                   : boolean := true;   -- implement watch dog timer (WDT)?
    IO_TRNG_USE                  : boolean := false;  -- implement true random number generator (TRNG)?
    IO_CFU0_USE                  : boolean := false;  -- implement custom functions unit 0 (CFU0)?
    IO_CFU1_USE                  : boolean := false   -- implement custom functions unit 1 (CFU1)?
  );
  port (
    -- AXI Lite-Compatible Master Interface --
    -- Clock and Reset --
    m_axi_aclk    : in  std_logic;
    m_axi_aresetn : in  std_logic;
    -- Write Address Channel --
    m_axi_awaddr  : out std_logic_vector(31 downto 0);
    m_axi_awprot  : out std_logic_vector(2 downto 0);
    m_axi_awvalid : out std_logic;
    m_axi_awready : in  std_logic;
    -- Write Data Channel --
    m_axi_wdata   : out std_logic_vector(31 downto 0);
    m_axi_wstrb   : out std_logic_vector(3 downto 0);
    m_axi_wvalid  : out std_logic;
    m_axi_wready  : in  std_logic;
    -- Read Address Channel --
    m_axi_araddr  : out std_logic_vector(31 downto 0);
    m_axi_arprot  : out std_logic_vector(2 downto 0);
    m_axi_arvalid : out std_logic;
    m_axi_arready : in  std_logic;
    -- Read Data Channel --
    m_axi_rdata   : in  std_logic_vector(31 downto 0);
    m_axi_rresp   : in  std_logic_vector(1 downto 0);
    m_axi_rvalid  : in  std_logic;
    m_axi_rready  : out std_logic;
    -- Write Response Channel --
    m_axi_bresp   : in  std_logic_vector(1 downto 0);
    m_axi_bvalid  : in  std_logic;
    m_axi_bready  : out std_logic;
    -- ------------------------------------------------------------
    -- GPIO --
    gpio_o      : out std_logic_vector(31 downto 0); -- parallel output
    gpio_i      : in  std_logic_vector(31 downto 0) := (others => '0'); -- parallel input
    -- UART --
    uart_txd_o  : out std_logic; -- UART send data
    uart_rxd_i  : in  std_logic := '0'; -- UART receive data
    -- SPI --
    spi_sck_o   : out std_logic; -- SPI serial clock
    spi_sdo_o   : out std_logic; -- controller data out, peripheral data in
    spi_sdi_i   : in  std_logic := '0'; -- controller data in, peripheral data out
    spi_csn_o   : out std_logic_vector(07 downto 0); -- SPI CS
    -- TWI --
    twi_sda_io  : inout std_logic; -- twi serial data line
    twi_scl_io  : inout std_logic; -- twi serial clock line
    -- PWM --
    pwm_o       : out std_logic_vector(03 downto 0);  -- pwm channels
    -- Interrupts --
    mtime_irq_i : in  std_logic := '0'; -- machine timer interrupt, available if IO_MTIME_USE = false
    msw_irq_i   : in  std_logic := '0'; -- machine software interrupt
    mext_irq_i  : in  std_logic := '0'  -- machine external interrupt
  );
end neorv32_top_axi4lite;

architecture neorv32_top_axi4lite_rtl of neorv32_top_axi4lite is

  -- type conversion --
  constant USER_CODE_INT    : std_ulogic_vector(31 downto 0) := std_ulogic_vector(USER_CODE);
  constant HW_THREAD_ID_INT : std_ulogic_vector(31 downto 0) := std_ulogic_vector(HW_THREAD_ID);
  --
  signal clk_i_int       : std_ulogic;
  signal rstn_i_int      : std_ulogic;
  --
  signal gpio_o_int      : std_ulogic_vector(31 downto 0);
  signal gpio_i_int      : std_ulogic_vector(31 downto 0);
  --
  signal uart_txd_o_int  : std_ulogic;
  signal uart_rxd_i_int  : std_ulogic;
  --
  signal spi_sck_o_int   : std_ulogic;
  signal spi_sdo_o_int   : std_ulogic;
  signal spi_sdi_i_int   : std_ulogic;
  signal spi_csn_o_int   : std_ulogic_vector(07 downto 0);
  --
  signal pwm_o_int       : std_ulogic_vector(03 downto 0);
  --
  signal mtime_irq_i_int : std_ulogic;
  signal msw_irq_i_int   : std_ulogic;
  signal mext_irq_i_int  : std_ulogic;

  -- internal wishbone bus --
  type wb_bus_t is record
    adr : std_ulogic_vector(31 downto 0); -- address
    di  : std_ulogic_vector(31 downto 0); -- processor input data
    do  : std_ulogic_vector(31 downto 0); -- processor output data
    we  : std_ulogic; -- write enable
    sel : std_ulogic_vector(03 downto 0); -- byte enable
    stb : std_ulogic; -- strobe
    cyc : std_ulogic; -- valid cycle
    ack : std_ulogic; -- transfer acknowledge
    err : std_ulogic; -- transfer error
    tag : std_ulogic_vector(2 downto 0); -- tag
  end record;
  signal wb_core : wb_bus_t;

  -- AXI bridge control --
  type ctrl_t is record
    radr_received : std_ulogic;
    wadr_received : std_ulogic;
    wdat_received : std_ulogic;
  end record;
  signal ctrl : ctrl_t;

  signal ack_read, ack_write : std_ulogic; -- normal transfer termination
  signal err_read, err_write : std_ulogic; -- error transfer termination

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (wb_pipe_mode_c = true) report "NEORV32 PROCESSOR CONFIG ERROR: AXI4-Lite bridge requires STANDARD/CLASSIC Wishbone mode (package.wb_pipe_mode_c = false)." severity error;
  assert not (CPU_EXTENSION_RISCV_A = true) report "NEORV32 PROCESSOR CONFIG WARNING: AXI4-Lite provides NO support for atomic memory operations." severity warning;


  -- The Core Of The Problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_top_inst: neorv32_top
  generic map (
    -- General --
    CLOCK_FREQUENCY              => CLOCK_FREQUENCY,    -- clock frequency of clk_i in Hz
    BOOTLOADER_USE               => BOOTLOADER_USE,     -- implement processor-internal bootloader?
    USER_CODE                    => USER_CODE_INT,      -- custom user code
    HW_THREAD_ID                 => HW_THREAD_ID_INT,   -- hardware thread id (hartid)
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        => CPU_EXTENSION_RISCV_A,        -- implement atomic extension?
    CPU_EXTENSION_RISCV_C        => CPU_EXTENSION_RISCV_C,        -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        => CPU_EXTENSION_RISCV_E,        -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        => CPU_EXTENSION_RISCV_M,        -- implement muld/div extension?
    CPU_EXTENSION_RISCV_U        => CPU_EXTENSION_RISCV_U,        -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zicsr    => CPU_EXTENSION_RISCV_Zicsr,    -- implement CSR system?
    CPU_EXTENSION_RISCV_Zifencei => CPU_EXTENSION_RISCV_Zifencei, -- implement instruction stream sync.?
    -- Extension Options --
    FAST_MUL_EN                  => FAST_MUL_EN,        -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                => FAST_SHIFT_EN,      -- use barrel shifter for shift operations
    -- Physical Memory Protection (PMP) --
    PMP_USE                      => PMP_USE,            -- implement PMP?
    -- Internal Instruction memory --
    MEM_INT_IMEM_USE             => MEM_INT_IMEM_USE,   -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            => MEM_INT_IMEM_SIZE,  -- size of processor-internal instruction memory in bytes
    MEM_INT_IMEM_ROM             => MEM_INT_IMEM_ROM,   -- implement processor-internal instruction memory as ROM
    -- Internal Data memory --
    MEM_INT_DMEM_USE             => MEM_INT_DMEM_USE,   -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            => MEM_INT_DMEM_SIZE,  -- size of processor-internal data memory in bytes
    -- Internal Cache memory --
    ICACHE_USE                   => ICACHE_USE,         -- implement instruction cache
    ICACHE_NUM_BLOCKS            => ICACHE_NUM_BLOCKS,  -- i-cache: number of blocks (min 1), has to be a power of 2
    ICACHE_BLOCK_SIZE            => ICACHE_BLOCK_SIZE,  -- i-cache: block size in bytes (min 4), has to be a power of 2
    -- External memory interface --
    MEM_EXT_USE                  => true,               -- implement external memory bus interface?
    -- Processor peripherals --
    IO_GPIO_USE                  => IO_GPIO_USE,        -- implement general purpose input/output port unit (GPIO)?
    IO_MTIME_USE                 => IO_MTIME_USE,       -- implement machine system timer (MTIME)?
    IO_UART_USE                  => IO_UART_USE,        -- implement universal asynchronous receiver/transmitter (UART)?
    IO_SPI_USE                   => IO_SPI_USE,         -- implement serial peripheral interface (SPI)?
    IO_TWI_USE                   => IO_TWI_USE,         -- implement two-wire interface (TWI)?
    IO_PWM_USE                   => IO_PWM_USE,         -- implement pulse-width modulation unit (PWM)?
    IO_WDT_USE                   => IO_WDT_USE,         -- implement watch dog timer (WDT)?
    IO_TRNG_USE                  => IO_TRNG_USE,        -- implement true random number generator (TRNG)?
    IO_CFU0_USE                  => IO_CFU0_USE,        -- implement custom functions unit 0 (CFU0)?
    IO_CFU1_USE                  => IO_CFU1_USE         -- implement custom functions unit 1 (CFU1)?
  )
  port map (
    -- Global control --
    clk_i       => clk_i_int,       -- global clock, rising edge
    rstn_i      => rstn_i_int,      -- global reset, low-active, async
    -- Wishbone bus interface --
    wb_tag_o    => wb_core.tag,     -- tag
    wb_adr_o    => wb_core.adr,     -- address
    wb_dat_i    => wb_core.di,      -- read data
    wb_dat_o    => wb_core.do,      -- write data
    wb_we_o     => wb_core.we,      -- read/write
    wb_sel_o    => wb_core.sel,     -- byte enable
    wb_stb_o    => wb_core.stb,     -- strobe
    wb_cyc_o    => wb_core.cyc,     -- valid cycle
    wb_lock_o   => open,            -- locked/exclusive bus access
    wb_ack_i    => wb_core.ack,     -- transfer acknowledge
    wb_err_i    => wb_core.err,     -- transfer error
    -- Advanced memory control signals --
    fence_o     => open,            -- indicates an executed FENCE operation
    fencei_o    => open,            -- indicates an executed FENCEI operation
    -- GPIO --
    gpio_o      => gpio_o_int,      -- parallel output
    gpio_i      => gpio_i_int,      -- parallel input
    -- UART --
    uart_txd_o  => uart_txd_o_int,  -- UART send data
    uart_rxd_i  => uart_rxd_i_int,  -- UART receive data
    -- SPI --
    spi_sck_o   => spi_sck_o_int,   -- SPI serial clock
    spi_sdo_o   => spi_sdo_o_int,   -- controller data out, peripheral data in
    spi_sdi_i   => spi_sdi_i_int,   -- controller data in, peripheral data out
    spi_csn_o   => spi_csn_o_int,   -- SPI CS
    -- TWI --
    twi_sda_io  => twi_sda_io,      -- twi serial data line
    twi_scl_io  => twi_scl_io,      -- twi serial clock line
    -- PWM --
    pwm_o       => pwm_o_int,       -- pwm channels
    -- system time input from external MTIME (available if IO_MTIME_USE = false) --
    mtime_i     => (others => '0'), -- current system time
    -- Interrupts --
    mtime_irq_i => mtime_irq_i_int, -- machine timer interrupt, available if IO_MTIME_USE = false
    msw_irq_i   => msw_irq_i_int,   -- machine software interrupt
    mext_irq_i  => mext_irq_i_int   -- machine external interrupt
  );

  -- type conversion --
  gpio_o         <= std_logic_vector(gpio_o_int);
  gpio_i_int     <= std_ulogic_vector(gpio_i);

  uart_txd_o     <= std_logic(uart_txd_o_int);
  uart_rxd_i_int <= std_ulogic(uart_rxd_i);

  spi_sck_o      <= std_logic(spi_sck_o_int);
  spi_sdo_o      <= std_logic(spi_sdo_o_int);
  spi_sdi_i_int  <= std_ulogic(spi_sdi_i);
  spi_csn_o      <= std_logic_vector(spi_csn_o_int);

  pwm_o          <= std_logic_vector(pwm_o_int);

  msw_irq_i_int  <= std_ulogic(msw_irq_i);
  mext_irq_i_int <= std_ulogic(mext_irq_i);
  

  -- Wishbone to AXI4-Lite Bridge -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- access arbiter --
  axi_access_arbiter: process(rstn_i_int, clk_i_int)
  begin
    if (rstn_i_int = '0') then
      ctrl.radr_received <= '0';
      ctrl.wadr_received <= '0';
      ctrl.wdat_received <= '0';
    elsif rising_edge(clk_i_int) then
      if (wb_core.cyc = '0') then -- idle
        ctrl.radr_received <= '0';
        ctrl.wadr_received <= '0';
        ctrl.wdat_received <= '0';
      else -- busy
        -- "read address received" flag --
        if (wb_core.we = '0') then -- pending READ
          if (m_axi_arready = '1') then -- read address received?
            ctrl.radr_received <= '1';
          end if;
        end if;
        -- "write address received" flag --
        if (wb_core.we = '1') then -- pending WRITE
          if (m_axi_awready = '1') then -- write address received?
            ctrl.wadr_received <= '1';
          end if;
        end if;
        -- "write data received" flag --
        if (wb_core.we = '1') then -- pending WRITE
          if (m_axi_wready = '1') then
            ctrl.wdat_received <= '1';
          end if;
        end if;
      end if;
    end if;
  end process axi_access_arbiter;


  -- AXI4-Lite Global Signals --
  clk_i_int     <= std_ulogic(m_axi_aclk);
  rstn_i_int    <= std_ulogic(m_axi_aresetn);


  -- AXI4-Lite Read Address Channel --
  m_axi_araddr  <= std_logic_vector(wb_core.adr);
  m_axi_arvalid <= std_logic((wb_core.cyc and (not wb_core.we)) and (not ctrl.radr_received));
--m_axi_arprot  <= "000"; -- recommended by Xilinx
  m_axi_arprot(0) <= wb_core.tag(0); -- 0:unprivileged access, 1:privileged access
  m_axi_arprot(1) <= wb_core.tag(1); -- 0:secure access, 1:non-secure access
  m_axi_arprot(2) <= wb_core.tag(2); -- 0:data access, 1:instruction access

  -- AXI4-Lite Read Data Channel --
  m_axi_rready  <= std_logic(wb_core.cyc and (not wb_core.we));
  wb_core.di    <= std_ulogic_vector(m_axi_rdata);
  ack_read      <= std_ulogic(m_axi_rvalid);
  err_read      <= '0' when (m_axi_rresp = "00") else '1'; -- read response = ok? check this signal only when m_axi_rvalid = '1'


  -- AXI4-Lite Write Address Channel --
  m_axi_awaddr  <= std_logic_vector(wb_core.adr);
  m_axi_awvalid <= std_logic((wb_core.cyc and wb_core.we) and (not ctrl.wadr_received));
--m_axi_awprot  <= "000"; -- recommended by Xilinx
  m_axi_awprot(0) <= wb_core.tag(0); -- 0:unprivileged access, 1:privileged access
  m_axi_awprot(1) <= wb_core.tag(1); -- 0:secure access, 1:non-secure access
  m_axi_awprot(2) <= wb_core.tag(2); -- 0:data access, 1:instruction access

  -- AXI4-Lite Write Data Channel --
  m_axi_wdata   <= std_logic_vector(wb_core.do);
  m_axi_wvalid  <= std_logic((wb_core.cyc and wb_core.we) and (not ctrl.wdat_received));
  m_axi_wstrb   <= std_logic_vector(wb_core.sel); -- byte-enable

  -- AXI4-Lite Write Response Channel --
  m_axi_bready  <= std_logic(wb_core.cyc and wb_core.we);
  ack_write     <= std_ulogic(m_axi_bvalid);
  err_write     <= '0' when (m_axi_bresp = "00") else '1'; -- write response = ok? check this signal only when m_axi_bvalid = '1'


  -- Wishbone transfer termination --
  wb_core.ack   <= ack_read or ack_write;
  wb_core.err   <= (ack_read and err_read) or (ack_write and err_write);


end neorv32_top_axi4lite_rtl;
