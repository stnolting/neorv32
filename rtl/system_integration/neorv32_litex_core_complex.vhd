-- ================================================================================ --
-- The NEORV32 RISC-V Processor - LiteX NEORV32 Core Complex Wrapper                --
-- -------------------------------------------------------------------------------- --
--                                __   _ __      _  __                              --
--                               / /  (_) /____ | |/_/                              --
--                              / /__/ / __/ -_)>  <                                --
--                             /____/_/\__/\__/_/|_|                                --
--                           Build your hardware, easily!                           --
--                                                                                  --
-- Unless otherwise noted, LiteX is copyright (C) 2012-2024 Enjoy-Digital.          --
-- All rights reserved.                                                             --
-- LiteX HQ: https://github.com/enjoy-digital/litex                                 --
-- -------------------------------------------------------------------------------- --
-- NEORV32 Core Complex wrapper for the LiteX SoC builder framework.                --
-- https://github.com/enjoy-digital/litex/tree/master/litex/soc/cores/cpu/neorv32   --
--                                                                                  --
-- This wrapper provides four pre-configured core complex configurations:           --
-- "minimal", "lite", "standard" and "full". See the 'configs_c' table for more     --
-- details which RISC-V ISA extensions and module parameters are used by each of    --
-- the these configurations. All configurations can be used with the                --
--  RISC-V-compatible on-chip debugger ("DEBUG").                                   --
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

entity neorv32_litex_core_complex is
  generic (
    CONFIG : natural; -- configuration select (0=minimal, 1=lite, 2=standard, 3=full)
    DEBUG  : boolean  -- enable on-chip debugger, valid for all configurations
  );
  port (
    -- Global control --
    clk_i      : in  std_ulogic; -- global clock, rising edge
    rstn_i     : in  std_ulogic; -- global reset, low-active, async

    -- JTAG on-chip debugger interface --
    jtag_tck_i : in  std_ulogic; -- serial clock
    jtag_tdi_i : in  std_ulogic; -- serial data input
    jtag_tdo_o : out std_ulogic; -- serial data output
    jtag_tms_i : in  std_ulogic; -- mode select

    -- External bus interface (Wishbone) --
    wb_adr_o   : out std_ulogic_vector(31 downto 0); -- address
    wb_dat_i   : in  std_ulogic_vector(31 downto 0); -- read data
    wb_dat_o   : out std_ulogic_vector(31 downto 0); -- write data
    wb_we_o    : out std_ulogic; -- read/write
    wb_sel_o   : out std_ulogic_vector(3 downto 0); -- byte enable
    wb_stb_o   : out std_ulogic; -- strobe
    wb_cyc_o   : out std_ulogic; -- valid cycle
    wb_ack_i   : in  std_ulogic; -- transfer acknowledge
    wb_err_i   : in  std_ulogic; -- transfer error

    -- CPU interrupt --
    mext_irq_i : in  std_ulogic  -- RISC-V machine external interrupt (MEI)
  );
end neorv32_litex_core_complex;

architecture neorv32_litex_core_complex_rtl of neorv32_litex_core_complex is

  -- core identification --
  constant jedec_id_c : std_ulogic_vector(10 downto 0) := "00000000000"; -- vendor's JEDEC manufacturer ID

  -- configuration helpers --
  constant num_configs_c : natural := 4;    -- number of pre-defined configurations
  type bool_t is array (0 to num_configs_c-1) of boolean;
  type natural_t is array (0 to num_configs_c-1) of natural;
  type configs_t is record
    riscv_c      : bool_t;
    riscv_m      : bool_t;
    riscv_u      : bool_t;
    riscv_zicntr : bool_t;
    riscv_zihpm  : bool_t;
    fast_ops     : bool_t;
    pmp_num      : natural_t;
    hpm_num      : natural_t;
    xcache_en    : bool_t;
    xcache_nb    : natural_t;
    xcache_bs    : natural_t;
    clint        : bool_t;
  end record;

  -- core complex configurations --
  constant configs_c : configs_t := (
    --               minimal   lite    standard  full
    riscv_c      => ( false,   true,    true,    true  ), -- RISC-V compressed instructions 'C'
    riscv_m      => ( false,   true,    true,    true  ), -- RISC-V hardware mul/div 'M'
    riscv_u      => ( false,   false,   true,    true  ), -- RISC-V user mode 'U'
    riscv_zicntr => ( false,   false,   true,    true  ), -- RISC-V standard CPU counters 'Zicntr'
    riscv_zihpm  => ( false,   false,   false,   true  ), -- RISC-V hardware performance monitors 'Zihpm'
    fast_ops     => ( false,   false,   true,    true  ), -- use DSPs and barrel-shifters
    pmp_num      => ( 0,       0,       0,       8     ), -- number of PMP regions (0..16)
    hpm_num      => ( 0,       0,       0,       8     ), -- number of HPM counters (0..29)
    xcache_en    => ( false,   false,   true,    true  ), -- external bus cache enabled
    xcache_nb    => ( 0,       0,       32,      64    ), -- number of cache blocks (lines), power of two
    xcache_bs    => ( 0,       0,       32,      32    ), -- size of cache clock (lines) in bytes, power of two
    clint        => ( false,   true,    true,    true  )  -- RISC-V core local interruptor
  );

  -- misc --
  signal wb_cyc : std_ulogic;

begin

  -- NEORV32 Core Complex -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_core_complex: neorv32_top
  generic map (
    -- General --
    CLOCK_FREQUENCY       => 0,                              -- clock frequency of clk_i in Hz [not required by the core complex]
    JEDEC_ID              => jedec_id_c,                     -- vendor's JEDEC ID
    -- On-Chip Debugger (OCD) --
    OCD_EN                => DEBUG,                          -- implement on-chip debugger
    -- RISC-V CPU Extensions --
    RISCV_ISA_C           => configs_c.riscv_c(CONFIG),      -- implement compressed extension?
    RISCV_ISA_M           => configs_c.riscv_m(CONFIG),      -- implement mul/div extension?
    RISCV_ISA_U           => configs_c.riscv_u(CONFIG),      -- implement user mode extension?
    RISCV_ISA_Zicntr      => configs_c.riscv_zicntr(CONFIG), -- implement base counters?
    RISCV_ISA_Zihpm       => configs_c.riscv_zihpm(CONFIG),  -- implement hardware performance monitors?
    -- Tuning Options --
    CPU_FAST_MUL_EN       => configs_c.fast_ops(CONFIG),     -- use DSPs for M extension's multiplier
    CPU_FAST_SHIFT_EN     => configs_c.fast_ops(CONFIG),     -- use barrel shifter for shift operations
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS       => configs_c.pmp_num(CONFIG),      -- number of regions (0..16)
    PMP_MIN_GRANULARITY   => 4,                              -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS          => configs_c.hpm_num(CONFIG),      -- number of implemented HPM counters (0..29)
    HPM_CNT_WIDTH         => 64,                             -- total size of HPM counters (0..64)
    -- External bus interface (XBUS) --
    XBUS_EN               => true,                           -- implement external memory bus interface?
    XBUS_TIMEOUT          => 1023,                           -- cycles after a pending bus access auto-terminates (0 = disabled)
    XBUS_REGSTAGE_EN      => false,                          -- add XBUS register stage
    XBUS_CACHE_EN         => configs_c.xcache_en(CONFIG),    -- enable external bus cache (x-cache)
    XBUS_CACHE_NUM_BLOCKS => configs_c.xcache_nb(CONFIG),    -- x-cache: number of blocks (min 1), has to be a power of 2
    XBUS_CACHE_BLOCK_SIZE => configs_c.xcache_bs(CONFIG),    -- x-cache: block size in bytes (min 4), has to be a power of 2
    -- Processor peripherals --
    IO_CLINT_EN           => configs_c.clint(CONFIG)         -- implement core local interruptor (CLINT)?
  )
  port map (
    -- Global control --
    clk_i       => clk_i,      -- global clock, rising edge
    rstn_i      => rstn_i,     -- global reset, low-active, async
    -- JTAG on-chip debugger interface --
    jtag_tck_i  => jtag_tck_i, -- serial clock
    jtag_tdi_i  => jtag_tdi_i, -- serial data input
    jtag_tdo_o  => jtag_tdo_o, -- serial data output
    jtag_tms_i  => jtag_tms_i, -- mode select
    -- External bus interface --
    xbus_adr_o  => wb_adr_o,   -- address
    xbus_dat_o  => wb_dat_o,   -- write data
    xbus_we_o   => wb_we_o,    -- read/write
    xbus_sel_o  => wb_sel_o,   -- byte enable
    xbus_stb_o  => open,       -- strobe
    xbus_cyc_o  => wb_cyc,     -- valid cycle
    xbus_dat_i  => wb_dat_i,   -- read data
    xbus_ack_i  => wb_ack_i,   -- transfer acknowledge
    xbus_err_i  => wb_err_i,   -- transfer error
    -- CPU Interrupts --
    mext_irq_i  => mext_irq_i  -- machine external interrupt
  );

  -- convert to "classic" Wishbone protocol (STB = CYC) --
  wb_cyc_o <= wb_cyc;
  wb_stb_o <= wb_cyc;


end neorv32_litex_core_complex_rtl;
