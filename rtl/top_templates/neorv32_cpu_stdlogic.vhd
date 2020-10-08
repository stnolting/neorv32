-- #################################################################################################
-- # << NEORV32 - CPU Top Entity with Resolved Port Signals (std_logic/std_logic_vector) >>        #
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

entity neorv32_cpu_stdlogic is
  generic (
    -- General --
    HW_THREAD_ID                 : std_logic_vector(31 downto 0):= (others => '0'); -- hardware thread id
    CPU_BOOT_ADDR                : std_logic_vector(31 downto 0):= (others => '0'); -- cpu boot address
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
    PMP_GRANULARITY              : natural := 14     -- minimal region granularity (1=8B, 2=16B, 3=32B, ...) default is 64k
  );
  port (
    -- global control --
    clk_i          : in  std_logic := '0'; -- global clock, rising edge
    rstn_i         : in  std_logic := '0'; -- global reset, low-active, async
    -- instruction bus interface --
    i_bus_addr_o   : out std_logic_vector(data_width_c-1 downto 0); -- bus access address
    i_bus_rdata_i  : in  std_logic_vector(data_width_c-1 downto 0) := (others => '0'); -- bus read data
    i_bus_wdata_o  : out std_logic_vector(data_width_c-1 downto 0); -- bus write data
    i_bus_ben_o    : out std_logic_vector(03 downto 0); -- byte enable
    i_bus_we_o     : out std_logic; -- write enable
    i_bus_re_o     : out std_logic; -- read enable
    i_bus_cancel_o : out std_logic; -- cancel current bus transaction
    i_bus_ack_i    : in  std_logic := '0'; -- bus transfer acknowledge
    i_bus_err_i    : in  std_logic := '0'; -- bus transfer error
    i_bus_fence_o  : out std_logic; -- executed FENCEI operation
    -- data bus interface --
    d_bus_addr_o   : out std_logic_vector(data_width_c-1 downto 0); -- bus access address
    d_bus_rdata_i  : in  std_logic_vector(data_width_c-1 downto 0) := (others => '0'); -- bus read data
    d_bus_wdata_o  : out std_logic_vector(data_width_c-1 downto 0); -- bus write data
    d_bus_ben_o    : out std_logic_vector(03 downto 0); -- byte enable
    d_bus_we_o     : out std_logic; -- write enable
    d_bus_re_o     : out std_logic; -- read enable
    d_bus_cancel_o : out std_logic; -- cancel current bus transaction
    d_bus_ack_i    : in  std_logic := '0'; -- bus transfer acknowledge
    d_bus_err_i    : in  std_logic := '0'; -- bus transfer error
    d_bus_fence_o  : out std_logic; -- executed FENCE operation
    -- system time input from MTIME --
    time_i         : in  std_logic_vector(63 downto 0) := (others => '0'); -- current system time
    -- interrupts (risc-v compliant) --
    msw_irq_i      : in  std_logic := '0'; -- machine software interrupt
    mext_irq_i     : in  std_logic := '0'; -- machine external interrupt
    mtime_irq_i    : in  std_logic := '0'; -- machine timer interrupt
    -- fast interrupts (custom) --
    firq_i         : in  std_logic_vector(3 downto 0) := (others => '0')
  );
end neorv32_cpu_stdlogic;

architecture neorv32_cpu_stdlogic_rtl of neorv32_cpu_stdlogic is

  -- type conversion --
  constant HW_THREAD_ID_INT  : std_ulogic_vector(31 downto 0) := std_ulogic_vector(HW_THREAD_ID);
  constant CPU_BOOT_ADDR_INT : std_ulogic_vector(31 downto 0) := std_ulogic_vector(CPU_BOOT_ADDR);
  --
  signal clk_i_int, rstn_i_int : std_ulogic;
  --
  signal i_bus_addr_o_int,   d_bus_addr_o_int   : std_ulogic_vector(31 downto 0);
  signal i_bus_rdata_i_int,  d_bus_rdata_i_int  : std_ulogic_vector(31 downto 0);
  signal i_bus_wdata_o_int,  d_bus_wdata_o_int  : std_ulogic_vector(31 downto 0);
  signal i_bus_ben_o_int,    d_bus_ben_o_int    : std_ulogic_vector(3 downto 0);
  signal i_bus_we_o_int,     d_bus_we_o_int     : std_ulogic;
  signal i_bus_re_o_int,     d_bus_re_o_int     : std_ulogic;
  signal i_bus_cancel_o_int, d_bus_cancel_o_int : std_ulogic;
  signal i_bus_ack_i_int,    d_bus_ack_i_int    : std_ulogic;
  signal i_bus_err_i_int,    d_bus_err_i_int    : std_ulogic;
  signal i_bus_fence_o_int,  d_bus_fence_o_int  : std_ulogic;
  --
  signal time_i_int : std_ulogic_vector(63 downto 0);
  --
  signal msw_irq_i_int, mext_irq_i_int, mtime_irq_i_int : std_ulogic;
  --
  signal firq_i_int : std_ulogic_vector(3 downto 0);

begin

  -- The Core Of The Problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_cpu_inst: neorv32_cpu
  generic map (
    -- General --
    HW_THREAD_ID                 => HW_THREAD_ID_INT,             -- hardware thread id
    CPU_BOOT_ADDR                => CPU_BOOT_ADDR_INT,            -- cpu boot address
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_C        => CPU_EXTENSION_RISCV_C,        -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        => CPU_EXTENSION_RISCV_E,        -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        => CPU_EXTENSION_RISCV_M,        -- implement muld/div extension?
    CPU_EXTENSION_RISCV_U        => CPU_EXTENSION_RISCV_U,        -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zicsr    => CPU_EXTENSION_RISCV_Zicsr,    -- implement CSR system?
    CPU_EXTENSION_RISCV_Zifencei => CPU_EXTENSION_RISCV_Zifencei, -- implement instruction stream sync.?
    -- Extension Options --
    FAST_MUL_EN                  => FAST_MUL_EN,                  -- use DSPs for M extension's multiplier
    -- Physical Memory Protection (PMP) --
    PMP_USE                      => PMP_USE,                      -- implement PMP?
    PMP_NUM_REGIONS              => PMP_NUM_REGIONS,              -- number of regions (max 8)
    PMP_GRANULARITY              => PMP_GRANULARITY               -- minimal region granularity (1=8B, 2=16B, 3=32B, ...) default is 64k
  )
  port map (
    -- global control --
    clk_i          => clk_i_int,          -- global clock, rising edge
    rstn_i         => rstn_i_int,         -- global reset, low-active, async
    -- instruction bus interface --
    i_bus_addr_o   => i_bus_addr_o_int,   -- bus access address
    i_bus_rdata_i  => i_bus_rdata_i_int,  -- bus read data
    i_bus_wdata_o  => i_bus_wdata_o_int,  -- bus write data
    i_bus_ben_o    => i_bus_ben_o_int,    -- byte enable
    i_bus_we_o     => i_bus_we_o_int,     -- write enable
    i_bus_re_o     => i_bus_re_o_int,     -- read enable
    i_bus_cancel_o => i_bus_cancel_o_int, -- cancel current bus transaction
    i_bus_ack_i    => i_bus_ack_i_int,    -- bus transfer acknowledge
    i_bus_err_i    => i_bus_err_i_int,    -- bus transfer error
    i_bus_fence_o  => i_bus_fence_o_int,  -- executed FENCEI operation
    -- data bus interface --
    d_bus_addr_o   => d_bus_addr_o_int,   -- bus access address
    d_bus_rdata_i  => d_bus_rdata_i_int,  -- bus read data
    d_bus_wdata_o  => d_bus_wdata_o_int,  -- bus write data
    d_bus_ben_o    => d_bus_ben_o_int,    -- byte enable
    d_bus_we_o     => d_bus_we_o_int,     -- write enable
    d_bus_re_o     => d_bus_re_o_int,     -- read enable
    d_bus_cancel_o => d_bus_cancel_o_int, -- cancel current bus transaction
    d_bus_ack_i    => d_bus_ack_i_int,    -- bus transfer acknowledge
    d_bus_err_i    => d_bus_err_i_int,    -- bus transfer error
    d_bus_fence_o  => d_bus_fence_o_int,  -- executed FENCEI operation
    -- system time input from MTIME --
    time_i         => time_i_int,         -- current system time
    -- interrupts (risc-v compliant) --
    msw_irq_i      => msw_irq_i_int,      -- machine software interrupt
    mext_irq_i     => mext_irq_i_int,     -- machine external interrupt
    mtime_irq_i    => mtime_irq_i_int,    -- machine timer interrupt
    -- fast interrupts (custom) --
    firq_i         => firq_i_int
  );

  -- type conversion --
  clk_i_int          <= std_ulogic(clk_i);
  rstn_i_int         <= std_ulogic(rstn_i);

  i_bus_addr_o       <= std_logic_vector(i_bus_addr_o_int);
  i_bus_rdata_i_int  <= std_ulogic_vector(i_bus_rdata_i);
  i_bus_wdata_o      <= std_logic_vector(i_bus_wdata_o_int);
  i_bus_ben_o        <= std_logic_vector(i_bus_ben_o_int);
  i_bus_we_o         <= std_logic(i_bus_we_o_int);
  i_bus_re_o         <= std_logic(i_bus_re_o_int);
  i_bus_cancel_o     <= std_logic(i_bus_cancel_o_int);
  i_bus_ack_i_int    <= std_ulogic(i_bus_ack_i);
  i_bus_err_i_int    <= std_ulogic(i_bus_err_i);
  i_bus_fence_o      <= std_logic(i_bus_fence_o_int);

  d_bus_addr_o       <= std_logic_vector(d_bus_addr_o_int);
  d_bus_rdata_i_int  <= std_ulogic_vector(d_bus_rdata_i);
  d_bus_wdata_o      <= std_logic_vector(d_bus_wdata_o_int);
  d_bus_ben_o        <= std_logic_vector(d_bus_ben_o_int);
  d_bus_we_o         <= std_logic(d_bus_we_o_int);
  d_bus_re_o         <= std_logic(d_bus_re_o_int);
  d_bus_cancel_o     <= std_logic(d_bus_cancel_o_int);
  d_bus_ack_i_int    <= std_ulogic(d_bus_ack_i);
  d_bus_err_i_int    <= std_ulogic(d_bus_err_i);
  d_bus_fence_o      <= std_logic(d_bus_fence_o_int);
  
  time_i_int         <= std_ulogic_vector(time_i);
  
  msw_irq_i_int      <= std_ulogic(msw_irq_i);
  mext_irq_i_int     <= std_ulogic(mext_irq_i);
  mtime_irq_i_int    <= std_ulogic(mtime_irq_i);

  firq_i_int         <= std_ulogic_vector(firq_i);


end neorv32_cpu_stdlogic_rtl;
