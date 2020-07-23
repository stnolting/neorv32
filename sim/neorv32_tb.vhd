-- #################################################################################################
-- # << NEORV32 - Simple Testbench >>                                                              #
-- # ********************************************************************************************* #
-- # This testbench provides a virtual UART receiver connected to the processor's uart_txd_o       #
-- # signals. The received chars are shown in the simulator console and also written to a file     #
-- # ("neorv32.testbench_uart.out").                                                               #
-- # Futhermore, this testbench provides a simple RAM connected to the external Wishbone bus.      #
-- # The testbench configures the processor with all optional element enabled by default.          #
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
use ieee.math_real.all;

library neorv32;
use neorv32.neorv32_package.all;
use std.textio.all;

entity neorv32_tb is
end neorv32_tb;

architecture neorv32_tb_rtl of neorv32_tb is

  -- User Configuration ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant t_clock_c          : time := 10 ns; -- main clock period
  constant f_clock_c          : real := 100000000.0; -- main clock in Hz
  constant f_clock_nat_c      : natural := 100000000; -- main clock in Hz
  constant baud_rate_c        : real := 19200.0; -- standard UART baudrate
  constant wb_mem_base_addr_c : std_ulogic_vector(31 downto 0) := x"F0000000"; -- wishbone memory base address
  constant wb_mem_size_c      : natural := 256; -- wishbone memory size in bytes
  -- -------------------------------------------------------------------------------------------

  -- text.io --
  file file_uart_tx_out : text open write_mode is "neorv32.testbench_uart.out";

  -- internal configuration --
  constant baud_val_c : real    := f_clock_c / baud_rate_c;
  constant f_clk_c    : natural := natural(f_clock_c);

  -- generators --
  signal clk_gen, rst_gen : std_ulogic := '0';

  -- simulation uart receiver --
  signal uart_txd         : std_ulogic;
  signal uart_rx_sync     : std_ulogic_vector(04 downto 0) := (others => '1');
  signal uart_rx_busy     : std_ulogic := '0';
  signal uart_rx_sreg     : std_ulogic_vector(08 downto 0) := (others => '0');
  signal uart_rx_baud_cnt : real;
  signal uart_rx_bitcnt   : natural;

  -- gpio --
  signal gpio : std_ulogic_vector(15 downto 0);

  -- twi --
  signal twi_scl, twi_sda : std_logic;

  -- spi --
  signal spi_data : std_logic;

  -- Wishbone bus --
  type wishbone_t is record
    addr  : std_ulogic_vector(31 downto 0); -- address
    wdata : std_ulogic_vector(31 downto 0); -- master write data
    rdata : std_ulogic_vector(31 downto 0); -- master read data
    we    : std_ulogic; -- write enable
    sel   : std_ulogic_vector(03 downto 0); -- byte enable
    stb   : std_ulogic; -- strobe
    cyc   : std_ulogic; -- valid cycle
    ack   : std_ulogic; -- transfer acknowledge
    err   : std_ulogic; -- transfer error
  end record;
  signal wb_cpu : wishbone_t;


  -- Wishbone memory, SimCom --
  type wb_mem_file_t is array (0 to wb_mem_size_c/4-1) of std_ulogic_vector(31 downto 0);
  signal wb_mem_file  : wb_mem_file_t := (others => (others => '0'));
  signal rb_en        : std_ulogic;
  signal r_data       : std_ulogic_vector(31 downto 0);
  signal wb_acc_en    : std_ulogic;
  signal wb_mem_rdata : std_ulogic_vector(31 downto 0);
  signal wb_mem_ack   : std_ulogic;

begin

  -- Clock/Reset Generator ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  clk_gen <= not clk_gen after (t_clock_c/2);
  rst_gen <= '0', '1' after 60*(t_clock_c/2);


  -- CPU Core -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_top_inst: neorv32_top
  generic map (
    -- General --
    CLOCK_FREQUENCY              => f_clock_nat_c, -- clock frequency of clk_i in Hz
    BOOTLOADER_USE               => false,         -- implement processor-internal bootloader?
    CSR_COUNTERS_USE             => true,          -- implement RISC-V perf. counters ([m]instret[h], [m]cycle[h], time[h])?
    USER_CODE                    => x"19880704",   -- custom user code
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_C        => true,          -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        => false,         -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        => true,          -- implement muld/div extension?
    CPU_EXTENSION_RISCV_Zicsr    => true,          -- implement CSR system?
    CPU_EXTENSION_RISCV_Zifencei => true,          -- implement instruction stream sync.?
    -- Memory configuration: Instruction memory --
    MEM_ISPACE_BASE              => x"00000000",   -- base address of instruction memory space
    MEM_ISPACE_SIZE              => 16*1024,       -- total size of instruction memory space in byte
    MEM_INT_IMEM_USE             => true,          -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            => 16*1024,       -- size of processor-internal instruction memory in bytes
    MEM_INT_IMEM_ROM             => false,         -- implement processor-internal instruction memory as ROM
    -- Memory configuration: Data memory --
    MEM_DSPACE_BASE              => x"80000000",   -- base address of data memory space
    MEM_DSPACE_SIZE              => 8*1024,        -- total size of data memory space in byte
    MEM_INT_DMEM_USE             => true,          -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            => 8*1024,        -- size of processor-internal data memory in bytes
    -- Memory configuration: External memory interface --
    MEM_EXT_USE                  => true,          -- implement external memory bus interface?
    MEM_EXT_REG_STAGES           => 2,             -- number of interface register stages (0,1,2)
    MEM_EXT_TIMEOUT              => 15,            -- cycles after which a valid bus access will timeout
    -- Processor peripherals --
    IO_GPIO_USE                  => true,          -- implement general purpose input/output port unit (GPIO)?
    IO_MTIME_USE                 => true,          -- implement machine system timer (MTIME)?
    IO_UART_USE                  => true,          -- implement universal asynchronous receiver/transmitter (UART)?
    IO_SPI_USE                   => true,          -- implement serial peripheral interface (SPI)?
    IO_TWI_USE                   => true,          -- implement two-wire interface (TWI)?
    IO_PWM_USE                   => true,          -- implement pulse-width modulation unit (PWM)?
    IO_WDT_USE                   => true,          -- implement watch dog timer (WDT)?
    IO_TRNG_USE                  => false,         -- CANNOT BE SIMULATED!
    IO_DEVNULL_USE               => true           -- implement dummy device (DEVNULL)?
  )
  port map (
    -- Global control --
    clk_i      => clk_gen,         -- global clock, rising edge
    rstn_i     => rst_gen,         -- global reset, low-active, async
    -- Wishbone bus interface --
    wb_adr_o   => wb_cpu.addr,     -- address
    wb_dat_i   => wb_cpu.rdata,    -- read data
    wb_dat_o   => wb_cpu.wdata,    -- write data
    wb_we_o    => wb_cpu.we,       -- read/write
    wb_sel_o   => wb_cpu.sel,      -- byte enable
    wb_stb_o   => wb_cpu.stb,      -- strobe
    wb_cyc_o   => wb_cpu.cyc,      -- valid cycle
    wb_ack_i   => wb_cpu.ack,      -- transfer acknowledge
    wb_err_i   => wb_cpu.err,      -- transfer error
    -- Advanced memory control signals --
    fence_o    => open,            -- indicates an executed FENCE operation
    fencei_o   => open,            -- indicates an executed FENCEI operation
    -- GPIO --
    gpio_o     => gpio,            -- parallel output
    gpio_i     => gpio,            -- parallel input
    -- UART --
    uart_txd_o => uart_txd,        -- UART send data
    uart_rxd_i => uart_txd,        -- UART receive data
    -- SPI --
    spi_sck_o  => open,            -- SPI serial clock
    spi_sdo_o  => spi_data,        -- controller data out, peripheral data in
    spi_sdi_i  => spi_data,        -- controller data in, peripheral data out
    spi_csn_o  => open,            -- SPI CS
    -- TWI --
    twi_sda_io => twi_sda,         -- twi serial data line
    twi_scl_io => twi_scl,         -- twi serial clock line
    -- PWM --
    pwm_o      => open,            -- pwm channels
    -- Interrupts --
    mext_irq_i => '0'              -- machine external interrupt
  );

  -- TWI termination --
  twi_scl <= 'H';
  twi_sda <= 'H';

  -- Wishbone read-back --
  wb_cpu.rdata <= wb_mem_rdata;
  wb_cpu.ack   <= wb_mem_ack;
  wb_cpu.err   <= '0';


  -- Console UART Receiver ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  uart_rx_console: process(clk_gen)
    variable i : integer;
    variable l : line;
  begin
    -- "UART" --
    if rising_edge(clk_gen) then
      -- synchronizer --
      uart_rx_sync <= uart_rx_sync(3 downto 0) & uart_txd;
      -- arbiter --
      if (uart_rx_busy = '0') then -- idle
        uart_rx_busy     <= '0';
        uart_rx_baud_cnt <= round(0.5 * baud_val_c);
        uart_rx_bitcnt   <= 9;
        if (uart_rx_sync(4 downto 1) = "1100") then -- start bit? (falling edge)
          uart_rx_busy <= '1';
        end if;
      else
        if (uart_rx_baud_cnt = 0.0) then
          if (uart_rx_bitcnt = 1) then
            uart_rx_baud_cnt <= round(0.5 * baud_val_c);
          else
            uart_rx_baud_cnt <= round(baud_val_c);
          end if;
          if (uart_rx_bitcnt = 0) then
            uart_rx_busy <= '0'; -- done
            i := to_integer(unsigned(uart_rx_sreg(8 downto 1)));

            if (i < 32) or (i > 32+95) then -- printable char?
              report "SIM_UART TX: (" & integer'image(i) & ")"; -- print code
            else
              report "SIM_UART TX: " & character'val(i); -- print ASCII
            end if;

            if (i = 10) then -- Linux line break
              writeline(file_uart_tx_out, l);
            elsif (i /= 13) then -- Remove additional carriage return
              write(l, character'val(i));
            end if;
          else
            uart_rx_sreg   <= uart_rx_sync(4) & uart_rx_sreg(8 downto 1);
            uart_rx_bitcnt <= uart_rx_bitcnt - 1;
          end if;
        else
          uart_rx_baud_cnt <= uart_rx_baud_cnt - 1.0;
        end if;
      end if;
    end if;
  end process uart_rx_console;


  -- Wishbone Memory ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
    wb_mem_file_access: process(clk_gen)
    begin
      if rising_edge(clk_gen) then
        rb_en <= wb_cpu.cyc and wb_cpu.stb and wb_acc_en and (not wb_cpu.we); -- read-back control
        wb_mem_ack <= wb_cpu.cyc and wb_cpu.stb and wb_acc_en; -- wishbone acknowledge
        if ((wb_cpu.cyc and wb_cpu.stb and wb_acc_en and wb_cpu.we) = '1') then -- valid write access
          for i in 0 to 3 loop
            if (wb_cpu.sel(i) = '1') then
              wb_mem_file(to_integer(unsigned(wb_cpu.addr(index_size_f(wb_mem_size_c/4)+1 downto 2))))(7+i*8 downto 0+i*8) <= wb_cpu.wdata(7+i*8 downto 0+i*8);
            end if;
          end loop; -- i
        end if;
        r_data <= wb_mem_file(to_integer(unsigned(wb_cpu.addr(index_size_f(wb_mem_size_c/4)+1 downto 2)))); -- word aligned
      end if;
    end process wb_mem_file_access;

  -- wb mem access --
  wb_acc_en <= '1' when (wb_cpu.addr >= wb_mem_base_addr_c) and (wb_cpu.addr < std_ulogic_vector(unsigned(wb_mem_base_addr_c) + wb_mem_size_c)) else '0';

  -- output gate --
  wb_mem_rdata <= r_data when (rb_en = '1') else (others=> '0');


end neorv32_tb_rtl;
