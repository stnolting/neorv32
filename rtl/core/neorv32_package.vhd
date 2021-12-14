-- #################################################################################################
-- # << NEORV32 - Main VHDL package file >>                                                        #
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

package neorv32_package is

  -- Architecture Configuration -------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- address space --
  constant ispace_base_c : std_ulogic_vector(31 downto 0) := x"00000000"; -- default instruction memory address space base address
  constant dspace_base_c : std_ulogic_vector(31 downto 0) := x"80000000"; -- default data memory address space base address

  -- CPU core --
  constant dedicated_reset_c : boolean := false; -- use dedicated hardware reset value for UNCRITICAL registers (FALSE=reset value is irrelevant (might simplify HW), default; TRUE=defined LOW reset value)
  constant cp_timeout_en_c   : boolean := false; -- auto-terminate pending co-processor operations after 256 cycles (for debugging only), default = false

  -- "critical" number of implemented PMP regions --
  -- if more PMP regions (> pmp_num_regions_critical_c) are defined, another register stage is automatically inserted into the memory interfaces
  -- increasing instruction fetch & data access latency by +1 cycle but also reducing critical path length
  constant pmp_num_regions_critical_c : natural := 8; -- default=8

  -- "response time window" for processor-internal modules --
  constant max_proc_int_response_time_c : natural := 15; -- cycles after which an *unacknowledged* internal bus access will timeout and trigger a bus fault exception (min 2)

  -- jtag tap - identifier --
  constant jtag_tap_idcode_version_c : std_ulogic_vector(03 downto 0) := x"0"; -- version
  constant jtag_tap_idcode_partid_c  : std_ulogic_vector(15 downto 0) := x"cafe"; -- part number
  constant jtag_tap_idcode_manid_c   : std_ulogic_vector(10 downto 0) := "00000000000"; -- manufacturer id

  -- Architecture Constants (do not modify!) ------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant data_width_c : natural := 32; -- native data path width - do not change!
  constant hw_version_c : std_ulogic_vector(31 downto 0) := x"01060409"; -- no touchy!
  constant archid_c     : natural := 19; -- official NEORV32 architecture ID - hands off!

  -- Check if we're inside the Matrix -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant is_simulation_c : boolean := false -- seems like we're on real hardware
-- pragma translate_off
-- synthesis translate_off
-- synthesis synthesis_off
-- RTL_SYNTHESIS OFF
  or true -- this MIGHT be a simulation
-- RTL_SYNTHESIS ON
-- synthesis synthesis_on
-- synthesis translate_on
-- pragma translate_on
  ;

  -- External Interface Types ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  type sdata_8x32_t  is array (0 to 7)  of std_ulogic_vector(31 downto 0);
  type sdata_8x32r_t is array (0 to 7)  of std_logic_vector(31 downto 0); -- resolved type

  -- Internal Interface Types ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  type pmp_ctrl_if_t is array (0 to 63) of std_ulogic_vector(07 downto 0);
  type pmp_addr_if_t is array (0 to 63) of std_ulogic_vector(33 downto 0);
  type cp_data_if_t  is array (0 to 3)  of std_ulogic_vector(data_width_c-1 downto 0);

  -- Internal Memory Types Configuration Types ----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  type mem32_t is array (natural range <>) of std_ulogic_vector(31 downto 0); -- memory with 32-bit entries
  type mem8_t  is array (natural range <>) of std_ulogic_vector(07 downto 0); -- memory with 8-bit entries

  -- Helper Functions -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function index_size_f(input : natural) return natural;
  function cond_sel_natural_f(cond : boolean; val_t : natural; val_f : natural) return natural;
  function cond_sel_int_f(cond : boolean; val_t : integer; val_f : integer) return integer;
  function cond_sel_stdulogicvector_f(cond : boolean; val_t : std_ulogic_vector; val_f : std_ulogic_vector) return std_ulogic_vector;
  function cond_sel_stdulogic_f(cond : boolean; val_t : std_ulogic; val_f : std_ulogic) return std_ulogic;
  function cond_sel_string_f(cond : boolean; val_t : string; val_f : string) return string;
  function bool_to_ulogic_f(cond : boolean) return std_ulogic;
  function or_reduce_f(a : std_ulogic_vector) return std_ulogic;
  function and_reduce_f(a : std_ulogic_vector) return std_ulogic;
  function xor_reduce_f(a : std_ulogic_vector) return std_ulogic;
  function to_hexchar_f(input : std_ulogic_vector(3 downto 0)) return character;
  function hexchar_to_stdulogicvector_f(input : character) return std_ulogic_vector;
  function bit_rev_f(input : std_ulogic_vector) return std_ulogic_vector;
  function is_power_of_two_f(input : natural) return boolean;
  function bswap32_f(input : std_ulogic_vector) return std_ulogic_vector;
  function char_to_lower_f(ch : character) return character;
  function str_equal_f(str0 : string; str1 : string) return boolean;
  function popcount_f(input : std_ulogic_vector) return natural;
  function leading_zeros_f(input : std_ulogic_vector) return natural;
  impure function mem32_init_f(init : mem32_t; depth : natural) return mem32_t;

  -- Internal (auto-generated) Configurations -----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant def_rst_val_c : std_ulogic := cond_sel_stdulogic_f(dedicated_reset_c, '0', '-');

  -- Processor-Internal Address Space Layout ------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- Internal Instruction Memory (IMEM) and Date Memory (DMEM) --
  constant imem_base_c          : std_ulogic_vector(data_width_c-1 downto 0) := ispace_base_c; -- internal instruction memory base address
  constant dmem_base_c          : std_ulogic_vector(data_width_c-1 downto 0) := dspace_base_c; -- internal data memory base address
  --> internal data/instruction memory sizes are configured via top's generics

  -- !!! IMPORTANT: The base address of each component/module has to be aligned to the !!!
  -- !!! total size of the module's occupied address space. The occupied address space !!!
  -- !!! has to be a power of two (minimum 4 bytes). Address spaces must not overlap.  !!!

  -- Internal Bootloader ROM --
  -- Actual bootloader size is determined during runtime via the length of the bootloader initialization image
  constant boot_rom_base_c      : std_ulogic_vector(data_width_c-1 downto 0) := x"ffff0000"; -- bootloader base address, fixed!
  constant boot_rom_max_size_c  : natural := 32*1024; -- max module's address space size in bytes, fixed!

  -- On-Chip Debugger: Debug Module --
  constant dm_base_c            : std_ulogic_vector(data_width_c-1 downto 0) := x"fffff800"; -- base address, fixed!
  constant dm_size_c            : natural := 4*32*4; -- debug ROM address space size in bytes, fixed
  constant dm_code_base_c       : std_ulogic_vector(data_width_c-1 downto 0) := x"fffff800";
  constant dm_pbuf_base_c       : std_ulogic_vector(data_width_c-1 downto 0) := x"fffff880";
  constant dm_data_base_c       : std_ulogic_vector(data_width_c-1 downto 0) := x"fffff900";
  constant dm_sreg_base_c       : std_ulogic_vector(data_width_c-1 downto 0) := x"fffff980";

  -- IO: Peripheral Devices ("IO") Area --
  -- Control register(s) (including the device-enable) should be located at the base address of each device
  constant io_base_c            : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe00";
  constant io_size_c            : natural := 512; -- IO address space size in bytes, fixed!

  -- Custom Functions Subsystem (CFS) --
  constant cfs_base_c           : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe00"; -- base address
  constant cfs_size_c           : natural := 32*4; -- module's address space in bytes
  constant cfs_reg0_addr_c      : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe00";
  constant cfs_reg1_addr_c      : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe04";
  constant cfs_reg2_addr_c      : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe08";
  constant cfs_reg3_addr_c      : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe0c";
  constant cfs_reg4_addr_c      : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe10";
  constant cfs_reg5_addr_c      : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe14";
  constant cfs_reg6_addr_c      : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe18";
  constant cfs_reg7_addr_c      : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe1c";
  constant cfs_reg8_addr_c      : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe20";
  constant cfs_reg9_addr_c      : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe24";
  constant cfs_reg10_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe28";
  constant cfs_reg11_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe2c";
  constant cfs_reg12_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe30";
  constant cfs_reg13_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe34";
  constant cfs_reg14_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe38";
  constant cfs_reg15_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe3c";
  constant cfs_reg16_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe40";
  constant cfs_reg17_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe44";
  constant cfs_reg18_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe48";
  constant cfs_reg19_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe4c";
  constant cfs_reg20_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe50";
  constant cfs_reg21_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe54";
  constant cfs_reg22_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe58";
  constant cfs_reg23_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe5c";
  constant cfs_reg24_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe60";
  constant cfs_reg25_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe64";
  constant cfs_reg26_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe68";
  constant cfs_reg27_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe6c";
  constant cfs_reg28_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe70";
  constant cfs_reg29_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe74";
  constant cfs_reg30_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe78";
  constant cfs_reg31_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe7c";

  -- Pulse-Width Modulation Controller (PWM) --
  constant pwm_base_c           : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe80"; -- base address
  constant pwm_size_c           : natural := 16*4; -- module's address space size in bytes
  constant pwm_ctrl_addr_c      : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe80";
  constant pwm_duty0_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe84";
  constant pwm_duty1_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe88";
  constant pwm_duty2_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe8c";
  constant pwm_duty3_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe90";
  constant pwm_duty4_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe94";
  constant pwm_duty5_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe98";
  constant pwm_duty6_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffe9c";
  constant pwm_duty7_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffea0";
  constant pwm_duty8_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffea4";
  constant pwm_duty9_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffea8";
  constant pwm_duty10_addr_c    : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffeac";
  constant pwm_duty11_addr_c    : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffeb0";
  constant pwm_duty12_addr_c    : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffeb4";
  constant pwm_duty13_addr_c    : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffeb8";
  constant pwm_duty14_addr_c    : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffebc";

  -- Stream Link Interface (SLINK) --
  constant slink_base_c         : std_ulogic_vector(data_width_c-1 downto 0) := x"fffffec0"; -- base address
  constant slink_size_c         : natural := 16*4; -- module's address space size in bytes

  -- reserved --
--constant reserved_base_c      : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff00"; -- base address
--constant reserved_size_c      : natural := 16*4; -- module's address space size in bytes

  -- reserved --
--constant reserved_base_c      : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff40"; -- base address
--constant reserved_size_c      : natural := 8*4; -- module's address space size in bytes

  -- General Purpose Timer (GPTMR) --
  constant gptmr_base_c         : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff60"; -- base address
  constant gptmr_size_c         : natural := 4*4; -- module's address space size in bytes
  constant gptmr_ctrl_addr_c    : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff60";
  constant gptmr_thres_addr_c   : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff64";
  constant gptmr_count_addr_c   : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff68";
--constant gptmr_reserve_addr_c : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff6c";

  -- reserved --
--constant reserved_base_c      : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff70"; -- base address
--constant reserved_size_c      : natural := 2*4; -- module's address space size in bytes

  -- reserved --
--constant reserved_base_c      : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff78"; -- base address
--constant reserved_size_c      : natural := 1*4; -- module's address space size in bytes

  -- Bus Access Monitor (BUSKEEPER) --
  constant buskeeper_base_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff7c"; -- base address
  constant buskeeper_size_c     : natural := 1*4; -- module's address space size in bytes

  -- External Interrupt Controller (XIRQ) --
  constant xirq_base_c          : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff80"; -- base address
  constant xirq_size_c          : natural := 4*4; -- module's address space size in bytes
  constant xirq_enable_addr_c   : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff80";
  constant xirq_pending_addr_c  : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff84";
  constant xirq_source_addr_c   : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff88";
--constant xirq_reserved_addr_c : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff8c";

  -- Machine System Timer (MTIME) --
  constant mtime_base_c         : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff90"; -- base address
  constant mtime_size_c         : natural := 4*4; -- module's address space size in bytes
  constant mtime_time_lo_addr_c : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff90";
  constant mtime_time_hi_addr_c : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff94";
  constant mtime_cmp_lo_addr_c  : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff98";
  constant mtime_cmp_hi_addr_c  : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffff9c";

  -- Primary Universal Asynchronous Receiver/Transmitter (UART0) --
  constant uart0_base_c         : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffa0"; -- base address
  constant uart0_size_c         : natural := 2*4; -- module's address space size in bytes
  constant uart0_ctrl_addr_c    : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffa0";
  constant uart0_rtx_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffa4";

  -- Serial Peripheral Interface (SPI) --
  constant spi_base_c           : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffa8"; -- base address
  constant spi_size_c           : natural := 2*4; -- module's address space size in bytes
  constant spi_ctrl_addr_c      : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffa8";
  constant spi_rtx_addr_c       : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffac";

  -- Two Wire Interface (TWI) --
  constant twi_base_c           : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffb0"; -- base address
  constant twi_size_c           : natural := 2*4; -- module's address space size in bytes
  constant twi_ctrl_addr_c      : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffb0";
  constant twi_rtx_addr_c       : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffb4";

  -- True Random Number Generator (TRNG) --
  constant trng_base_c          : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffb8"; -- base address
  constant trng_size_c          : natural := 1*4; -- module's address space size in bytes
  constant trng_ctrl_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffb8";

  -- Watch Dog Timer (WDT) --
  constant wdt_base_c           : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffbc"; -- base address
  constant wdt_size_c           : natural := 1*4; -- module's address space size in bytes
  constant wdt_ctrl_addr_c      : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffbc";

  -- General Purpose Input/Output Controller (GPIO) --
  constant gpio_base_c          : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffc0"; -- base address
  constant gpio_size_c          : natural := 4*4; -- module's address space size in bytes
  constant gpio_in_lo_addr_c    : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffc0";
  constant gpio_in_hi_addr_c    : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffc4";
  constant gpio_out_lo_addr_c   : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffc8";
  constant gpio_out_hi_addr_c   : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffcc";

  -- Secondary Universal Asynchronous Receiver/Transmitter (UART1) --
  constant uart1_base_c         : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffd0"; -- base address
  constant uart1_size_c         : natural := 2*4; -- module's address space size in bytes
  constant uart1_ctrl_addr_c    : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffd0";
  constant uart1_rtx_addr_c     : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffd4";

  -- Smart LED (WS2811/WS2812) Interface (NEOLED) --
  constant neoled_base_c        : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffd8"; -- base address
  constant neoled_size_c        : natural := 2*4; -- module's address space size in bytes
  constant neoled_ctrl_addr_c   : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffd8";
  constant neoled_data_addr_c   : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffdc";

  -- System Information Memory (SYSINFO) --
  constant sysinfo_base_c       : std_ulogic_vector(data_width_c-1 downto 0) := x"ffffffe0"; -- base address
  constant sysinfo_size_c       : natural := 8*4; -- module's address space size in bytes

  -- Main CPU Control Bus -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- register file --
  constant ctrl_rf_in_mux_c     : natural :=  0; -- input source select lsb (0=MEM, 1=ALU)
  constant ctrl_rf_rs1_adr0_c   : natural :=  1; -- source register 1 address bit 0
  constant ctrl_rf_rs1_adr1_c   : natural :=  2; -- source register 1 address bit 1
  constant ctrl_rf_rs1_adr2_c   : natural :=  3; -- source register 1 address bit 2
  constant ctrl_rf_rs1_adr3_c   : natural :=  4; -- source register 1 address bit 3
  constant ctrl_rf_rs1_adr4_c   : natural :=  5; -- source register 1 address bit 4
  constant ctrl_rf_rs2_adr0_c   : natural :=  6; -- source register 2 address bit 0
  constant ctrl_rf_rs2_adr1_c   : natural :=  7; -- source register 2 address bit 1
  constant ctrl_rf_rs2_adr2_c   : natural :=  8; -- source register 2 address bit 2
  constant ctrl_rf_rs2_adr3_c   : natural :=  9; -- source register 2 address bit 3
  constant ctrl_rf_rs2_adr4_c   : natural := 10; -- source register 2 address bit 4
  constant ctrl_rf_rd_adr0_c    : natural := 11; -- destination register address bit 0
  constant ctrl_rf_rd_adr1_c    : natural := 12; -- destination register address bit 1
  constant ctrl_rf_rd_adr2_c    : natural := 13; -- destination register address bit 2
  constant ctrl_rf_rd_adr3_c    : natural := 14; -- destination register address bit 3
  constant ctrl_rf_rd_adr4_c    : natural := 15; -- destination register address bit 4
  constant ctrl_rf_wb_en_c      : natural := 16; -- write back enable
  -- alu --
  constant ctrl_alu_op0_c       : natural := 17; -- ALU operation select bit 0
  constant ctrl_alu_op1_c       : natural := 18; -- ALU operation select bit 1
  constant ctrl_alu_op2_c       : natural := 19; -- ALU operation select bit 2
  constant ctrl_alu_func0_c     : natural := 20; -- ALU function select command bit 0
  constant ctrl_alu_func1_c     : natural := 21; -- ALU function select command bit 1
  constant ctrl_alu_opa_mux_c   : natural := 22; -- operand A select (0=rs1, 1=PC)
  constant ctrl_alu_opb_mux_c   : natural := 23; -- operand B select (0=rs2, 1=IMM)
  constant ctrl_alu_unsigned_c  : natural := 24; -- is unsigned ALU operation
  constant ctrl_alu_shift_dir_c : natural := 25; -- shift direction (0=left, 1=right)
  constant ctrl_alu_shift_ar_c  : natural := 26; -- is arithmetic shift
  constant ctrl_alu_frm0_c      : natural := 27; -- FPU rounding mode bit 0
  constant ctrl_alu_frm1_c      : natural := 28; -- FPU rounding mode bit 1
  constant ctrl_alu_frm2_c      : natural := 29; -- FPU rounding mode bit 2
  -- bus interface --
  constant ctrl_bus_size_lsb_c  : natural := 30; -- transfer size lsb (00=byte, 01=half-word)
  constant ctrl_bus_size_msb_c  : natural := 31; -- transfer size msb (10=word, 11=?)
  constant ctrl_bus_rd_c        : natural := 32; -- read data request
  constant ctrl_bus_wr_c        : natural := 33; -- write data request
  constant ctrl_bus_if_c        : natural := 34; -- instruction fetch request
  constant ctrl_bus_mo_we_c     : natural := 35; -- memory address and data output register write enable
  constant ctrl_bus_mi_we_c     : natural := 36; -- memory data input register write enable
  constant ctrl_bus_unsigned_c  : natural := 37; -- is unsigned load
  constant ctrl_bus_ierr_ack_c  : natural := 38; -- acknowledge instruction fetch bus exceptions
  constant ctrl_bus_derr_ack_c  : natural := 39; -- acknowledge data access bus exceptions
  constant ctrl_bus_fence_c     : natural := 40; -- executed fence operation
  constant ctrl_bus_fencei_c    : natural := 41; -- executed fencei operation
  constant ctrl_bus_lock_c      : natural := 42; -- make atomic/exclusive access lock
  constant ctrl_bus_de_lock_c   : natural := 43; -- remove atomic/exclusive access 
  constant ctrl_bus_ch_lock_c   : natural := 44; -- evaluate atomic/exclusive lock (SC operation)
  -- co-processors --
  constant ctrl_cp_id_lsb_c     : natural := 45; -- cp select ID lsb
  constant ctrl_cp_id_msb_c     : natural := 46; -- cp select ID msb
  -- instruction's control blocks (used by cpu co-processors) --
  constant ctrl_ir_funct3_0_c   : natural := 47; -- funct3 bit 0
  constant ctrl_ir_funct3_1_c   : natural := 48; -- funct3 bit 1
  constant ctrl_ir_funct3_2_c   : natural := 49; -- funct3 bit 2
  constant ctrl_ir_funct12_0_c  : natural := 50; -- funct12 bit 0
  constant ctrl_ir_funct12_1_c  : natural := 51; -- funct12 bit 1
  constant ctrl_ir_funct12_2_c  : natural := 52; -- funct12 bit 2
  constant ctrl_ir_funct12_3_c  : natural := 53; -- funct12 bit 3
  constant ctrl_ir_funct12_4_c  : natural := 54; -- funct12 bit 4
  constant ctrl_ir_funct12_5_c  : natural := 55; -- funct12 bit 5
  constant ctrl_ir_funct12_6_c  : natural := 56; -- funct12 bit 6
  constant ctrl_ir_funct12_7_c  : natural := 57; -- funct12 bit 7
  constant ctrl_ir_funct12_8_c  : natural := 58; -- funct12 bit 8
  constant ctrl_ir_funct12_9_c  : natural := 59; -- funct12 bit 9
  constant ctrl_ir_funct12_10_c : natural := 60; -- funct12 bit 10
  constant ctrl_ir_funct12_11_c : natural := 61; -- funct12 bit 11
  constant ctrl_ir_opcode7_0_c  : natural := 62; -- opcode7 bit 0
  constant ctrl_ir_opcode7_1_c  : natural := 63; -- opcode7 bit 1
  constant ctrl_ir_opcode7_2_c  : natural := 64; -- opcode7 bit 2
  constant ctrl_ir_opcode7_3_c  : natural := 65; -- opcode7 bit 3
  constant ctrl_ir_opcode7_4_c  : natural := 66; -- opcode7 bit 4
  constant ctrl_ir_opcode7_5_c  : natural := 67; -- opcode7 bit 5
  constant ctrl_ir_opcode7_6_c  : natural := 68; -- opcode7 bit 6
  -- CPU status --
  constant ctrl_priv_lvl_lsb_c  : natural := 69; -- privilege level lsb
  constant ctrl_priv_lvl_msb_c  : natural := 70; -- privilege level msb
  constant ctrl_sleep_c         : natural := 71; -- set when CPU is in sleep mode
  constant ctrl_trap_c          : natural := 72; -- set when CPU is entering trap execution
  constant ctrl_debug_running_c : natural := 73; -- CPU is in debug mode when set
  -- control bus size --
  constant ctrl_width_c         : natural := 74; -- control bus size

  -- Comparator Bus -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant cmp_equal_c : natural := 0;
  constant cmp_less_c  : natural := 1; -- for signed and unsigned comparisons

  -- RISC-V Opcode Layout -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant instr_opcode_lsb_c  : natural :=  0; -- opcode bit 0
  constant instr_opcode_msb_c  : natural :=  6; -- opcode bit 6
  constant instr_rd_lsb_c      : natural :=  7; -- destination register address bit 0
  constant instr_rd_msb_c      : natural := 11; -- destination register address bit 4
  constant instr_funct3_lsb_c  : natural := 12; -- funct3 bit 0
  constant instr_funct3_msb_c  : natural := 14; -- funct3 bit 2
  constant instr_rs1_lsb_c     : natural := 15; -- source register 1 address bit 0
  constant instr_rs1_msb_c     : natural := 19; -- source register 1 address bit 4
  constant instr_rs2_lsb_c     : natural := 20; -- source register 2 address bit 0
  constant instr_rs2_msb_c     : natural := 24; -- source register 2 address bit 4
  constant instr_funct7_lsb_c  : natural := 25; -- funct7 bit 0
  constant instr_funct7_msb_c  : natural := 31; -- funct7 bit 6
  constant instr_funct12_lsb_c : natural := 20; -- funct12 bit 0
  constant instr_funct12_msb_c : natural := 31; -- funct12 bit 11
  constant instr_imm12_lsb_c   : natural := 20; -- immediate12 bit 0
  constant instr_imm12_msb_c   : natural := 31; -- immediate12 bit 11
  constant instr_imm20_lsb_c   : natural := 12; -- immediate20 bit 0
  constant instr_imm20_msb_c   : natural := 31; -- immediate20 bit 21
  constant instr_csr_id_lsb_c  : natural := 20; -- csr select bit 0
  constant instr_csr_id_msb_c  : natural := 31; -- csr select bit 11
  constant instr_funct5_lsb_c  : natural := 27; -- funct5 select bit 0
  constant instr_funct5_msb_c  : natural := 31; -- funct5 select bit 4

  -- RISC-V Opcodes -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- alu --
  constant opcode_lui_c    : std_ulogic_vector(6 downto 0) := "0110111"; -- load upper immediate
  constant opcode_auipc_c  : std_ulogic_vector(6 downto 0) := "0010111"; -- add upper immediate to PC
  constant opcode_alui_c   : std_ulogic_vector(6 downto 0) := "0010011"; -- ALU operation with immediate (operation via funct3 and funct7)
  constant opcode_alu_c    : std_ulogic_vector(6 downto 0) := "0110011"; -- ALU operation (operation via funct3 and funct7)
  -- control flow --
  constant opcode_jal_c    : std_ulogic_vector(6 downto 0) := "1101111"; -- jump and link
  constant opcode_jalr_c   : std_ulogic_vector(6 downto 0) := "1100111"; -- jump and link with register
  constant opcode_branch_c : std_ulogic_vector(6 downto 0) := "1100011"; -- branch (condition set via funct3)
  -- memory access --
  constant opcode_load_c   : std_ulogic_vector(6 downto 0) := "0000011"; -- load (data type via funct3)
  constant opcode_store_c  : std_ulogic_vector(6 downto 0) := "0100011"; -- store (data type via funct3)
  -- system/csr --
  constant opcode_fence_c  : std_ulogic_vector(6 downto 0) := "0001111"; -- fence / fence.i
  constant opcode_syscsr_c : std_ulogic_vector(6 downto 0) := "1110011"; -- system/csr access (type via funct3)
  -- atomic memory access (A) --
  constant opcode_atomic_c : std_ulogic_vector(6 downto 0) := "0101111"; -- atomic operations (A extension)
  -- floating point operations (Zfinx-only) (F/D/H/Q) --
  constant opcode_fop_c    : std_ulogic_vector(6 downto 0) := "1010011"; -- dual/single operand instruction

  -- RISC-V Funct3 --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- control flow --
  constant funct3_beq_c    : std_ulogic_vector(2 downto 0) := "000"; -- branch if equal
  constant funct3_bne_c    : std_ulogic_vector(2 downto 0) := "001"; -- branch if not equal
  constant funct3_blt_c    : std_ulogic_vector(2 downto 0) := "100"; -- branch if less than
  constant funct3_bge_c    : std_ulogic_vector(2 downto 0) := "101"; -- branch if greater than or equal
  constant funct3_bltu_c   : std_ulogic_vector(2 downto 0) := "110"; -- branch if less than (unsigned)
  constant funct3_bgeu_c   : std_ulogic_vector(2 downto 0) := "111"; -- branch if greater than or equal (unsigned)
  -- memory access --
  constant funct3_lb_c     : std_ulogic_vector(2 downto 0) := "000"; -- load byte
  constant funct3_lh_c     : std_ulogic_vector(2 downto 0) := "001"; -- load half word
  constant funct3_lw_c     : std_ulogic_vector(2 downto 0) := "010"; -- load word
  constant funct3_lbu_c    : std_ulogic_vector(2 downto 0) := "100"; -- load byte (unsigned)
  constant funct3_lhu_c    : std_ulogic_vector(2 downto 0) := "101"; -- load half word (unsigned)
  constant funct3_sb_c     : std_ulogic_vector(2 downto 0) := "000"; -- store byte
  constant funct3_sh_c     : std_ulogic_vector(2 downto 0) := "001"; -- store half word
  constant funct3_sw_c     : std_ulogic_vector(2 downto 0) := "010"; -- store word
  -- alu --
  constant funct3_subadd_c : std_ulogic_vector(2 downto 0) := "000"; -- sub/add via funct7
  constant funct3_sll_c    : std_ulogic_vector(2 downto 0) := "001"; -- shift logical left
  constant funct3_slt_c    : std_ulogic_vector(2 downto 0) := "010"; -- set on less
  constant funct3_sltu_c   : std_ulogic_vector(2 downto 0) := "011"; -- set on less unsigned
  constant funct3_xor_c    : std_ulogic_vector(2 downto 0) := "100"; -- xor
  constant funct3_sr_c     : std_ulogic_vector(2 downto 0) := "101"; -- shift right via funct7
  constant funct3_or_c     : std_ulogic_vector(2 downto 0) := "110"; -- or
  constant funct3_and_c    : std_ulogic_vector(2 downto 0) := "111"; -- and
  -- system/csr --
  constant funct3_env_c    : std_ulogic_vector(2 downto 0) := "000"; -- ecall, ebreak, mret, wfi, ...
  constant funct3_csrrw_c  : std_ulogic_vector(2 downto 0) := "001"; -- atomic r/w
  constant funct3_csrrs_c  : std_ulogic_vector(2 downto 0) := "010"; -- atomic read & set bit
  constant funct3_csrrc_c  : std_ulogic_vector(2 downto 0) := "011"; -- atomic read & clear bit
  constant funct3_csrrwi_c : std_ulogic_vector(2 downto 0) := "101"; -- atomic r/w immediate
  constant funct3_csrrsi_c : std_ulogic_vector(2 downto 0) := "110"; -- atomic read & set bit immediate
  constant funct3_csrrci_c : std_ulogic_vector(2 downto 0) := "111"; -- atomic read & clear bit immediate
  -- fence --
  constant funct3_fence_c  : std_ulogic_vector(2 downto 0) := "000"; -- fence - order IO/memory access (->NOP)
  constant funct3_fencei_c : std_ulogic_vector(2 downto 0) := "001"; -- fencei - instruction stream sync

  -- RISC-V Funct12 -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- system --
  constant funct12_ecall_c  : std_ulogic_vector(11 downto 0) := x"000"; -- ECALL
  constant funct12_ebreak_c : std_ulogic_vector(11 downto 0) := x"001"; -- EBREAK
  constant funct12_mret_c   : std_ulogic_vector(11 downto 0) := x"302"; -- MRET
  constant funct12_wfi_c    : std_ulogic_vector(11 downto 0) := x"105"; -- WFI
  constant funct12_dret_c   : std_ulogic_vector(11 downto 0) := x"7b2"; -- DRET

  -- RISC-V Funct5 --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- atomic operations --
  constant funct5_a_lr_c : std_ulogic_vector(4 downto 0) := "00010"; -- LR
  constant funct5_a_sc_c : std_ulogic_vector(4 downto 0) := "00011"; -- SC

  -- RISC-V Floating-Point Stuff ------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- formats --
  constant float_single_c : std_ulogic_vector(1 downto 0) := "00"; -- single-precision (32-bit)
  constant float_double_c : std_ulogic_vector(1 downto 0) := "01"; -- double-precision (64-bit)
  constant float_half_c   : std_ulogic_vector(1 downto 0) := "10"; -- half-precision (16-bit)
  constant float_quad_c   : std_ulogic_vector(1 downto 0) := "11"; -- quad-precision (128-bit)

  -- number class flags --
  constant fp_class_neg_inf_c    : natural := 0; -- negative infinity
  constant fp_class_neg_norm_c   : natural := 1; -- negative normal number
  constant fp_class_neg_denorm_c : natural := 2; -- negative subnormal number
  constant fp_class_neg_zero_c   : natural := 3; -- negative zero
  constant fp_class_pos_zero_c   : natural := 4; -- positive zero
  constant fp_class_pos_denorm_c : natural := 5; -- positive subnormal number
  constant fp_class_pos_norm_c   : natural := 6; -- positive normal number
  constant fp_class_pos_inf_c    : natural := 7; -- positive infinity
  constant fp_class_snan_c       : natural := 8; -- signaling NaN (sNaN)
  constant fp_class_qnan_c       : natural := 9; -- quiet NaN (qNaN)

  -- exception flags --
  constant fp_exc_nv_c : natural := 0; -- invalid operation
  constant fp_exc_dz_c : natural := 1; -- divide by zero
  constant fp_exc_of_c : natural := 2; -- overflow
  constant fp_exc_uf_c : natural := 3; -- underflow
  constant fp_exc_nx_c : natural := 4; -- inexact

  -- special values (single-precision) --
  constant fp_single_qnan_c     : std_ulogic_vector(31 downto 0) := x"7fc00000"; -- quiet NaN
  constant fp_single_snan_c     : std_ulogic_vector(31 downto 0) := x"7fa00000"; -- signaling NaN
  constant fp_single_pos_inf_c  : std_ulogic_vector(31 downto 0) := x"7f800000"; -- positive infinity
  constant fp_single_neg_inf_c  : std_ulogic_vector(31 downto 0) := x"ff800000"; -- negative infinity
  constant fp_single_pos_zero_c : std_ulogic_vector(31 downto 0) := x"00000000"; -- positive zero
  constant fp_single_neg_zero_c : std_ulogic_vector(31 downto 0) := x"80000000"; -- negative zero

  -- RISC-V CSR Addresses -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- <<< standard read/write CSRs >>> --
  -- user floating-point CSRs --
  constant csr_class_float_c    : std_ulogic_vector(09 downto 0) := x"00" & "00"; -- floating point
  constant csr_fflags_c         : std_ulogic_vector(11 downto 0) := x"001";
  constant csr_frm_c            : std_ulogic_vector(11 downto 0) := x"002";
  constant csr_fcsr_c           : std_ulogic_vector(11 downto 0) := x"003";
  -- machine trap setup --
  constant csr_class_setup_c    : std_ulogic_vector(08 downto 0) := x"30" & '0'; -- trap setup
  constant csr_mstatus_c        : std_ulogic_vector(11 downto 0) := x"300";
  constant csr_misa_c           : std_ulogic_vector(11 downto 0) := x"301";
  constant csr_mie_c            : std_ulogic_vector(11 downto 0) := x"304";
  constant csr_mtvec_c          : std_ulogic_vector(11 downto 0) := x"305";
  constant csr_mcounteren_c     : std_ulogic_vector(11 downto 0) := x"306";
  --
  constant csr_mstatush_c       : std_ulogic_vector(11 downto 0) := x"310";
  -- machine configuration --
  constant csr_class_envcfg_c   : std_ulogic_vector(06 downto 0) := x"3" & "000"; -- configuration
  constant csr_menvcfg_c        : std_ulogic_vector(11 downto 0) := x"30a";
  constant csr_menvcfgh_c       : std_ulogic_vector(11 downto 0) := x"31a";
  -- machine counter setup --
  constant csr_cnt_setup_c      : std_ulogic_vector(06 downto 0) := x"3" & "001"; -- counter setup
  constant csr_mcountinhibit_c  : std_ulogic_vector(11 downto 0) := x"320";
  constant csr_mhpmevent3_c     : std_ulogic_vector(11 downto 0) := x"323";
  constant csr_mhpmevent4_c     : std_ulogic_vector(11 downto 0) := x"324";
  constant csr_mhpmevent5_c     : std_ulogic_vector(11 downto 0) := x"325";
  constant csr_mhpmevent6_c     : std_ulogic_vector(11 downto 0) := x"326";
  constant csr_mhpmevent7_c     : std_ulogic_vector(11 downto 0) := x"327";
  constant csr_mhpmevent8_c     : std_ulogic_vector(11 downto 0) := x"328";
  constant csr_mhpmevent9_c     : std_ulogic_vector(11 downto 0) := x"329";
  constant csr_mhpmevent10_c    : std_ulogic_vector(11 downto 0) := x"32a";
  constant csr_mhpmevent11_c    : std_ulogic_vector(11 downto 0) := x"32b";
  constant csr_mhpmevent12_c    : std_ulogic_vector(11 downto 0) := x"32c";
  constant csr_mhpmevent13_c    : std_ulogic_vector(11 downto 0) := x"32d";
  constant csr_mhpmevent14_c    : std_ulogic_vector(11 downto 0) := x"32e";
  constant csr_mhpmevent15_c    : std_ulogic_vector(11 downto 0) := x"32f";
  constant csr_mhpmevent16_c    : std_ulogic_vector(11 downto 0) := x"330";
  constant csr_mhpmevent17_c    : std_ulogic_vector(11 downto 0) := x"331";
  constant csr_mhpmevent18_c    : std_ulogic_vector(11 downto 0) := x"332";
  constant csr_mhpmevent19_c    : std_ulogic_vector(11 downto 0) := x"333";
  constant csr_mhpmevent20_c    : std_ulogic_vector(11 downto 0) := x"334";
  constant csr_mhpmevent21_c    : std_ulogic_vector(11 downto 0) := x"335";
  constant csr_mhpmevent22_c    : std_ulogic_vector(11 downto 0) := x"336";
  constant csr_mhpmevent23_c    : std_ulogic_vector(11 downto 0) := x"337";
  constant csr_mhpmevent24_c    : std_ulogic_vector(11 downto 0) := x"338";
  constant csr_mhpmevent25_c    : std_ulogic_vector(11 downto 0) := x"339";
  constant csr_mhpmevent26_c    : std_ulogic_vector(11 downto 0) := x"33a";
  constant csr_mhpmevent27_c    : std_ulogic_vector(11 downto 0) := x"33b";
  constant csr_mhpmevent28_c    : std_ulogic_vector(11 downto 0) := x"33c";
  constant csr_mhpmevent29_c    : std_ulogic_vector(11 downto 0) := x"33d";
  constant csr_mhpmevent30_c    : std_ulogic_vector(11 downto 0) := x"33e";
  constant csr_mhpmevent31_c    : std_ulogic_vector(11 downto 0) := x"33f";
  -- machine trap handling --
  constant csr_class_trap_c     : std_ulogic_vector(07 downto 0) := x"34"; -- machine trap handling
  constant csr_mscratch_c       : std_ulogic_vector(11 downto 0) := x"340";
  constant csr_mepc_c           : std_ulogic_vector(11 downto 0) := x"341";
  constant csr_mcause_c         : std_ulogic_vector(11 downto 0) := x"342";
  constant csr_mtval_c          : std_ulogic_vector(11 downto 0) := x"343";
  constant csr_mip_c            : std_ulogic_vector(11 downto 0) := x"344";
  -- physical memory protection - configuration --
  constant csr_class_pmpcfg_c   : std_ulogic_vector(07 downto 0) := x"3a"; -- pmp configuration
  constant csr_pmpcfg0_c        : std_ulogic_vector(11 downto 0) := x"3a0";
  constant csr_pmpcfg1_c        : std_ulogic_vector(11 downto 0) := x"3a1";
  constant csr_pmpcfg2_c        : std_ulogic_vector(11 downto 0) := x"3a2";
  constant csr_pmpcfg3_c        : std_ulogic_vector(11 downto 0) := x"3a3";
  constant csr_pmpcfg4_c        : std_ulogic_vector(11 downto 0) := x"3a4";
  constant csr_pmpcfg5_c        : std_ulogic_vector(11 downto 0) := x"3a5";
  constant csr_pmpcfg6_c        : std_ulogic_vector(11 downto 0) := x"3a6";
  constant csr_pmpcfg7_c        : std_ulogic_vector(11 downto 0) := x"3a7";
  constant csr_pmpcfg8_c        : std_ulogic_vector(11 downto 0) := x"3a8";
  constant csr_pmpcfg9_c        : std_ulogic_vector(11 downto 0) := x"3a9";
  constant csr_pmpcfg10_c       : std_ulogic_vector(11 downto 0) := x"3aa";
  constant csr_pmpcfg11_c       : std_ulogic_vector(11 downto 0) := x"3ab";
  constant csr_pmpcfg12_c       : std_ulogic_vector(11 downto 0) := x"3ac";
  constant csr_pmpcfg13_c       : std_ulogic_vector(11 downto 0) := x"3ad";
  constant csr_pmpcfg14_c       : std_ulogic_vector(11 downto 0) := x"3ae";
  constant csr_pmpcfg15_c       : std_ulogic_vector(11 downto 0) := x"3af";
  -- physical memory protection - address --
  constant csr_pmpaddr0_c       : std_ulogic_vector(11 downto 0) := x"3b0";
  constant csr_pmpaddr1_c       : std_ulogic_vector(11 downto 0) := x"3b1";
  constant csr_pmpaddr2_c       : std_ulogic_vector(11 downto 0) := x"3b2";
  constant csr_pmpaddr3_c       : std_ulogic_vector(11 downto 0) := x"3b3";
  constant csr_pmpaddr4_c       : std_ulogic_vector(11 downto 0) := x"3b4";
  constant csr_pmpaddr5_c       : std_ulogic_vector(11 downto 0) := x"3b5";
  constant csr_pmpaddr6_c       : std_ulogic_vector(11 downto 0) := x"3b6";
  constant csr_pmpaddr7_c       : std_ulogic_vector(11 downto 0) := x"3b7";
  constant csr_pmpaddr8_c       : std_ulogic_vector(11 downto 0) := x"3b8";
  constant csr_pmpaddr9_c       : std_ulogic_vector(11 downto 0) := x"3b9";
  constant csr_pmpaddr10_c      : std_ulogic_vector(11 downto 0) := x"3ba";
  constant csr_pmpaddr11_c      : std_ulogic_vector(11 downto 0) := x"3bb";
  constant csr_pmpaddr12_c      : std_ulogic_vector(11 downto 0) := x"3bc";
  constant csr_pmpaddr13_c      : std_ulogic_vector(11 downto 0) := x"3bd";
  constant csr_pmpaddr14_c      : std_ulogic_vector(11 downto 0) := x"3be";
  constant csr_pmpaddr15_c      : std_ulogic_vector(11 downto 0) := x"3bf";
  constant csr_pmpaddr16_c      : std_ulogic_vector(11 downto 0) := x"3c0";
  constant csr_pmpaddr17_c      : std_ulogic_vector(11 downto 0) := x"3c1";
  constant csr_pmpaddr18_c      : std_ulogic_vector(11 downto 0) := x"3c2";
  constant csr_pmpaddr19_c      : std_ulogic_vector(11 downto 0) := x"3c3";
  constant csr_pmpaddr20_c      : std_ulogic_vector(11 downto 0) := x"3c4";
  constant csr_pmpaddr21_c      : std_ulogic_vector(11 downto 0) := x"3c5";
  constant csr_pmpaddr22_c      : std_ulogic_vector(11 downto 0) := x"3c6";
  constant csr_pmpaddr23_c      : std_ulogic_vector(11 downto 0) := x"3c7";
  constant csr_pmpaddr24_c      : std_ulogic_vector(11 downto 0) := x"3c8";
  constant csr_pmpaddr25_c      : std_ulogic_vector(11 downto 0) := x"3c9";
  constant csr_pmpaddr26_c      : std_ulogic_vector(11 downto 0) := x"3ca";
  constant csr_pmpaddr27_c      : std_ulogic_vector(11 downto 0) := x"3cb";
  constant csr_pmpaddr28_c      : std_ulogic_vector(11 downto 0) := x"3cc";
  constant csr_pmpaddr29_c      : std_ulogic_vector(11 downto 0) := x"3cd";
  constant csr_pmpaddr30_c      : std_ulogic_vector(11 downto 0) := x"3ce";
  constant csr_pmpaddr31_c      : std_ulogic_vector(11 downto 0) := x"3cf";
  constant csr_pmpaddr32_c      : std_ulogic_vector(11 downto 0) := x"3d0";
  constant csr_pmpaddr33_c      : std_ulogic_vector(11 downto 0) := x"3d1";
  constant csr_pmpaddr34_c      : std_ulogic_vector(11 downto 0) := x"3d2";
  constant csr_pmpaddr35_c      : std_ulogic_vector(11 downto 0) := x"3d3";
  constant csr_pmpaddr36_c      : std_ulogic_vector(11 downto 0) := x"3d4";
  constant csr_pmpaddr37_c      : std_ulogic_vector(11 downto 0) := x"3d5";
  constant csr_pmpaddr38_c      : std_ulogic_vector(11 downto 0) := x"3d6";
  constant csr_pmpaddr39_c      : std_ulogic_vector(11 downto 0) := x"3d7";
  constant csr_pmpaddr40_c      : std_ulogic_vector(11 downto 0) := x"3d8";
  constant csr_pmpaddr41_c      : std_ulogic_vector(11 downto 0) := x"3d9";
  constant csr_pmpaddr42_c      : std_ulogic_vector(11 downto 0) := x"3da";
  constant csr_pmpaddr43_c      : std_ulogic_vector(11 downto 0) := x"3db";
  constant csr_pmpaddr44_c      : std_ulogic_vector(11 downto 0) := x"3dc";
  constant csr_pmpaddr45_c      : std_ulogic_vector(11 downto 0) := x"3dd";
  constant csr_pmpaddr46_c      : std_ulogic_vector(11 downto 0) := x"3de";
  constant csr_pmpaddr47_c      : std_ulogic_vector(11 downto 0) := x"3df";
  constant csr_pmpaddr48_c      : std_ulogic_vector(11 downto 0) := x"3e0";
  constant csr_pmpaddr49_c      : std_ulogic_vector(11 downto 0) := x"3e1";
  constant csr_pmpaddr50_c      : std_ulogic_vector(11 downto 0) := x"3e2";
  constant csr_pmpaddr51_c      : std_ulogic_vector(11 downto 0) := x"3e3";
  constant csr_pmpaddr52_c      : std_ulogic_vector(11 downto 0) := x"3e4";
  constant csr_pmpaddr53_c      : std_ulogic_vector(11 downto 0) := x"3e5";
  constant csr_pmpaddr54_c      : std_ulogic_vector(11 downto 0) := x"3e6";
  constant csr_pmpaddr55_c      : std_ulogic_vector(11 downto 0) := x"3e7";
  constant csr_pmpaddr56_c      : std_ulogic_vector(11 downto 0) := x"3e8";
  constant csr_pmpaddr57_c      : std_ulogic_vector(11 downto 0) := x"3e9";
  constant csr_pmpaddr58_c      : std_ulogic_vector(11 downto 0) := x"3ea";
  constant csr_pmpaddr59_c      : std_ulogic_vector(11 downto 0) := x"3eb";
  constant csr_pmpaddr60_c      : std_ulogic_vector(11 downto 0) := x"3ec";
  constant csr_pmpaddr61_c      : std_ulogic_vector(11 downto 0) := x"3ed";
  constant csr_pmpaddr62_c      : std_ulogic_vector(11 downto 0) := x"3ee";
  constant csr_pmpaddr63_c      : std_ulogic_vector(11 downto 0) := x"3ef";
  -- debug mode registers --
  constant csr_class_debug_c    : std_ulogic_vector(09 downto 0) := x"7b" & "00"; -- debug registers
  constant csr_dcsr_c           : std_ulogic_vector(11 downto 0) := x"7b0";
  constant csr_dpc_c            : std_ulogic_vector(11 downto 0) := x"7b1";
  constant csr_dscratch0_c      : std_ulogic_vector(11 downto 0) := x"7b2";
  -- machine counters/timers --
  constant csr_mcycle_c         : std_ulogic_vector(11 downto 0) := x"b00";
  constant csr_minstret_c       : std_ulogic_vector(11 downto 0) := x"b02";
  --
  constant csr_mhpmcounter3_c   : std_ulogic_vector(11 downto 0) := x"b03";
  constant csr_mhpmcounter4_c   : std_ulogic_vector(11 downto 0) := x"b04";
  constant csr_mhpmcounter5_c   : std_ulogic_vector(11 downto 0) := x"b05";
  constant csr_mhpmcounter6_c   : std_ulogic_vector(11 downto 0) := x"b06";
  constant csr_mhpmcounter7_c   : std_ulogic_vector(11 downto 0) := x"b07";
  constant csr_mhpmcounter8_c   : std_ulogic_vector(11 downto 0) := x"b08";
  constant csr_mhpmcounter9_c   : std_ulogic_vector(11 downto 0) := x"b09";
  constant csr_mhpmcounter10_c  : std_ulogic_vector(11 downto 0) := x"b0a";
  constant csr_mhpmcounter11_c  : std_ulogic_vector(11 downto 0) := x"b0b";
  constant csr_mhpmcounter12_c  : std_ulogic_vector(11 downto 0) := x"b0c";
  constant csr_mhpmcounter13_c  : std_ulogic_vector(11 downto 0) := x"b0d";
  constant csr_mhpmcounter14_c  : std_ulogic_vector(11 downto 0) := x"b0e";
  constant csr_mhpmcounter15_c  : std_ulogic_vector(11 downto 0) := x"b0f";
  constant csr_mhpmcounter16_c  : std_ulogic_vector(11 downto 0) := x"b10";
  constant csr_mhpmcounter17_c  : std_ulogic_vector(11 downto 0) := x"b11";
  constant csr_mhpmcounter18_c  : std_ulogic_vector(11 downto 0) := x"b12";
  constant csr_mhpmcounter19_c  : std_ulogic_vector(11 downto 0) := x"b13";
  constant csr_mhpmcounter20_c  : std_ulogic_vector(11 downto 0) := x"b14";
  constant csr_mhpmcounter21_c  : std_ulogic_vector(11 downto 0) := x"b15";
  constant csr_mhpmcounter22_c  : std_ulogic_vector(11 downto 0) := x"b16";
  constant csr_mhpmcounter23_c  : std_ulogic_vector(11 downto 0) := x"b17";
  constant csr_mhpmcounter24_c  : std_ulogic_vector(11 downto 0) := x"b18";
  constant csr_mhpmcounter25_c  : std_ulogic_vector(11 downto 0) := x"b19";
  constant csr_mhpmcounter26_c  : std_ulogic_vector(11 downto 0) := x"b1a";
  constant csr_mhpmcounter27_c  : std_ulogic_vector(11 downto 0) := x"b1b";
  constant csr_mhpmcounter28_c  : std_ulogic_vector(11 downto 0) := x"b1c";
  constant csr_mhpmcounter29_c  : std_ulogic_vector(11 downto 0) := x"b1d";
  constant csr_mhpmcounter30_c  : std_ulogic_vector(11 downto 0) := x"b1e";
  constant csr_mhpmcounter31_c  : std_ulogic_vector(11 downto 0) := x"b1f";
  --
  constant csr_mcycleh_c        : std_ulogic_vector(11 downto 0) := x"b80";
  constant csr_minstreth_c      : std_ulogic_vector(11 downto 0) := x"b82";
  --
  constant csr_mhpmcounter3h_c  : std_ulogic_vector(11 downto 0) := x"b83";
  constant csr_mhpmcounter4h_c  : std_ulogic_vector(11 downto 0) := x"b84";
  constant csr_mhpmcounter5h_c  : std_ulogic_vector(11 downto 0) := x"b85";
  constant csr_mhpmcounter6h_c  : std_ulogic_vector(11 downto 0) := x"b86";
  constant csr_mhpmcounter7h_c  : std_ulogic_vector(11 downto 0) := x"b87";
  constant csr_mhpmcounter8h_c  : std_ulogic_vector(11 downto 0) := x"b88";
  constant csr_mhpmcounter9h_c  : std_ulogic_vector(11 downto 0) := x"b89";
  constant csr_mhpmcounter10h_c : std_ulogic_vector(11 downto 0) := x"b8a";
  constant csr_mhpmcounter11h_c : std_ulogic_vector(11 downto 0) := x"b8b";
  constant csr_mhpmcounter12h_c : std_ulogic_vector(11 downto 0) := x"b8c";
  constant csr_mhpmcounter13h_c : std_ulogic_vector(11 downto 0) := x"b8d";
  constant csr_mhpmcounter14h_c : std_ulogic_vector(11 downto 0) := x"b8e";
  constant csr_mhpmcounter15h_c : std_ulogic_vector(11 downto 0) := x"b8f";
  constant csr_mhpmcounter16h_c : std_ulogic_vector(11 downto 0) := x"b90";
  constant csr_mhpmcounter17h_c : std_ulogic_vector(11 downto 0) := x"b91";
  constant csr_mhpmcounter18h_c : std_ulogic_vector(11 downto 0) := x"b92";
  constant csr_mhpmcounter19h_c : std_ulogic_vector(11 downto 0) := x"b93";
  constant csr_mhpmcounter20h_c : std_ulogic_vector(11 downto 0) := x"b94";
  constant csr_mhpmcounter21h_c : std_ulogic_vector(11 downto 0) := x"b95";
  constant csr_mhpmcounter22h_c : std_ulogic_vector(11 downto 0) := x"b96";
  constant csr_mhpmcounter23h_c : std_ulogic_vector(11 downto 0) := x"b97";
  constant csr_mhpmcounter24h_c : std_ulogic_vector(11 downto 0) := x"b98";
  constant csr_mhpmcounter25h_c : std_ulogic_vector(11 downto 0) := x"b99";
  constant csr_mhpmcounter26h_c : std_ulogic_vector(11 downto 0) := x"b9a";
  constant csr_mhpmcounter27h_c : std_ulogic_vector(11 downto 0) := x"b9b";
  constant csr_mhpmcounter28h_c : std_ulogic_vector(11 downto 0) := x"b9c";
  constant csr_mhpmcounter29h_c : std_ulogic_vector(11 downto 0) := x"b9d";
  constant csr_mhpmcounter30h_c : std_ulogic_vector(11 downto 0) := x"b9e";
  constant csr_mhpmcounter31h_c : std_ulogic_vector(11 downto 0) := x"b9f";

  -- <<< standard read-only CSRs >>> --
  -- user counters/timers --
  constant csr_cycle_c          : std_ulogic_vector(11 downto 0) := x"c00";
  constant csr_time_c           : std_ulogic_vector(11 downto 0) := x"c01";
  constant csr_instret_c        : std_ulogic_vector(11 downto 0) := x"c02";
  constant csr_cycleh_c         : std_ulogic_vector(11 downto 0) := x"c80";
  constant csr_timeh_c          : std_ulogic_vector(11 downto 0) := x"c81";
  constant csr_instreth_c       : std_ulogic_vector(11 downto 0) := x"c82";
  -- machine information registers --
  constant csr_mvendorid_c      : std_ulogic_vector(11 downto 0) := x"f11";
  constant csr_marchid_c        : std_ulogic_vector(11 downto 0) := x"f12";
  constant csr_mimpid_c         : std_ulogic_vector(11 downto 0) := x"f13";
  constant csr_mhartid_c        : std_ulogic_vector(11 downto 0) := x"f14";
  constant csr_mconfigptr_c     : std_ulogic_vector(11 downto 0) := x"f15";

  -- Co-Processor IDs -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant cp_sel_shifter_c  : std_ulogic_vector(1 downto 0) := "00"; -- shift operations (base ISA)
  constant cp_sel_muldiv_c   : std_ulogic_vector(1 downto 0) := "01"; -- multiplication/division operations ('M' extensions)
  constant cp_sel_bitmanip_c : std_ulogic_vector(1 downto 0) := "10"; -- bit manipulation ('B' extensions)
  constant cp_sel_fpu_c      : std_ulogic_vector(1 downto 0) := "11"; -- floating-point unit ('Zfinx' extension)

  -- ALU Function Codes ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- ALU core [DO NOT CHANGE ENCODING!] --
  constant alu_op_add_c     : std_ulogic_vector(2 downto 0) := "000"; -- alu_result <= A + B
  constant alu_op_sub_c     : std_ulogic_vector(2 downto 0) := "001"; -- alu_result <= A - B
--constant alu_op_mova_c    : std_ulogic_vector(2 downto 0) := "010"; -- alu_result <= A (rs1)
  constant alu_op_slt_c     : std_ulogic_vector(2 downto 0) := "011"; -- alu_result <= A < B
  constant alu_op_movb_c    : std_ulogic_vector(2 downto 0) := "100"; -- alu_result <= B
  constant alu_op_xor_c     : std_ulogic_vector(2 downto 0) := "101"; -- alu_result <= A xor B
  constant alu_op_or_c      : std_ulogic_vector(2 downto 0) := "110"; -- alu_result <= A or B
  constant alu_op_and_c     : std_ulogic_vector(2 downto 0) := "111"; -- alu_result <= A and B
  -- function select (actual ALU result) --
  constant alu_func_core_c  : std_ulogic_vector(1 downto 0) := "00"; -- r <= alu_result
  constant alu_func_nxpc_c  : std_ulogic_vector(1 downto 0) := "01"; -- r <= next_PC
  constant alu_func_csrr_c  : std_ulogic_vector(1 downto 0) := "10"; -- r <= CSR read
  constant alu_func_copro_c : std_ulogic_vector(1 downto 0) := "11"; -- r <= CP result (multi-cycle)

  -- Trap ID Codes --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- MSB:   1 = async exception (IRQ), 0 = sync exception (e.g. ebreak)
  -- MSB-1: 1 = entry to debug mode, 0 = normal trapping
  -- RISC-V compliant sync. exceptions --
  constant trap_ima_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00000"; -- 0.0:  instruction misaligned
  constant trap_iba_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00001"; -- 0.1:  instruction access fault
  constant trap_iil_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00010"; -- 0.2:  illegal instruction
  constant trap_brk_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00011"; -- 0.3:  breakpoint
  constant trap_lma_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00100"; -- 0.4:  load address misaligned
  constant trap_lbe_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00101"; -- 0.5:  load access fault
  constant trap_sma_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00110"; -- 0.6:  store address misaligned
  constant trap_sbe_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00111"; -- 0.7:  store access fault
  constant trap_uenv_c     : std_ulogic_vector(6 downto 0) := "0" & "0" & "01000"; -- 0.8:  environment call from u-mode
  constant trap_menv_c     : std_ulogic_vector(6 downto 0) := "0" & "0" & "01011"; -- 0.11: environment call from m-mode
  -- RISC-V compliant interrupts (async. exceptions) --
  constant trap_msi_c      : std_ulogic_vector(6 downto 0) := "1" & "0" & "00011"; -- 1.3:  machine software interrupt
  constant trap_mti_c      : std_ulogic_vector(6 downto 0) := "1" & "0" & "00111"; -- 1.7:  machine timer interrupt
  constant trap_mei_c      : std_ulogic_vector(6 downto 0) := "1" & "0" & "01011"; -- 1.11: machine external interrupt
  -- NEORV32-specific (custom) interrupts (async. exceptions) --
  constant trap_firq0_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10000"; -- 1.16: fast interrupt 0
  constant trap_firq1_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10001"; -- 1.17: fast interrupt 1
  constant trap_firq2_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10010"; -- 1.18: fast interrupt 2
  constant trap_firq3_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10011"; -- 1.19: fast interrupt 3
  constant trap_firq4_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10100"; -- 1.20: fast interrupt 4
  constant trap_firq5_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10101"; -- 1.21: fast interrupt 5
  constant trap_firq6_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10110"; -- 1.22: fast interrupt 6
  constant trap_firq7_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10111"; -- 1.23: fast interrupt 7
  constant trap_firq8_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "11000"; -- 1.24: fast interrupt 8
  constant trap_firq9_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "11001"; -- 1.25: fast interrupt 9
  constant trap_firq10_c   : std_ulogic_vector(6 downto 0) := "1" & "0" & "11010"; -- 1.26: fast interrupt 10
  constant trap_firq11_c   : std_ulogic_vector(6 downto 0) := "1" & "0" & "11011"; -- 1.27: fast interrupt 11
  constant trap_firq12_c   : std_ulogic_vector(6 downto 0) := "1" & "0" & "11100"; -- 1.28: fast interrupt 12
  constant trap_firq13_c   : std_ulogic_vector(6 downto 0) := "1" & "0" & "11101"; -- 1.29: fast interrupt 13
  constant trap_firq14_c   : std_ulogic_vector(6 downto 0) := "1" & "0" & "11110"; -- 1.30: fast interrupt 14
  constant trap_firq15_c   : std_ulogic_vector(6 downto 0) := "1" & "0" & "11111"; -- 1.31: fast interrupt 15
  -- entering debug mode - cause --
  constant trap_db_break_c : std_ulogic_vector(6 downto 0) := "0" & "1" & "00010"; -- break instruction (sync / EXCEPTION)
  constant trap_db_halt_c  : std_ulogic_vector(6 downto 0) := "1" & "1" & "00011"; -- external halt request (async / IRQ)
  constant trap_db_step_c  : std_ulogic_vector(6 downto 0) := "1" & "1" & "00100"; -- single-stepping (async / IRQ)

  -- CPU Control Exception System -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- exception source bits --
  constant exception_iaccess_c   : natural :=  0; -- instruction access fault
  constant exception_iillegal_c  : natural :=  1; -- illegal instruction
  constant exception_ialign_c    : natural :=  2; -- instruction address misaligned
  constant exception_m_envcall_c : natural :=  3; -- ENV call from m-mode
  constant exception_u_envcall_c : natural :=  4; -- ENV call from u-mode
  constant exception_break_c     : natural :=  5; -- breakpoint
  constant exception_salign_c    : natural :=  6; -- store address misaligned
  constant exception_lalign_c    : natural :=  7; -- load address misaligned
  constant exception_saccess_c   : natural :=  8; -- store access fault
  constant exception_laccess_c   : natural :=  9; -- load access fault
  -- for debug mode only --
  constant exception_db_break_c  : natural := 10; -- enter debug mode via ebreak instruction ("sync EXCEPTION")
  --
  constant exception_width_c     : natural := 11; -- length of this list in bits
  -- interrupt source bits --
  constant interrupt_msw_irq_c   : natural :=  0; -- machine software interrupt
  constant interrupt_mtime_irq_c : natural :=  1; -- machine timer interrupt
  constant interrupt_mext_irq_c  : natural :=  2; -- machine external interrupt
  constant interrupt_firq_0_c    : natural :=  3; -- fast interrupt channel 0
  constant interrupt_firq_1_c    : natural :=  4; -- fast interrupt channel 1
  constant interrupt_firq_2_c    : natural :=  5; -- fast interrupt channel 2
  constant interrupt_firq_3_c    : natural :=  6; -- fast interrupt channel 3
  constant interrupt_firq_4_c    : natural :=  7; -- fast interrupt channel 4
  constant interrupt_firq_5_c    : natural :=  8; -- fast interrupt channel 5
  constant interrupt_firq_6_c    : natural :=  9; -- fast interrupt channel 6
  constant interrupt_firq_7_c    : natural := 10; -- fast interrupt channel 7
  constant interrupt_firq_8_c    : natural := 11; -- fast interrupt channel 8
  constant interrupt_firq_9_c    : natural := 12; -- fast interrupt channel 9
  constant interrupt_firq_10_c   : natural := 13; -- fast interrupt channel 10
  constant interrupt_firq_11_c   : natural := 14; -- fast interrupt channel 11
  constant interrupt_firq_12_c   : natural := 15; -- fast interrupt channel 12
  constant interrupt_firq_13_c   : natural := 16; -- fast interrupt channel 13
  constant interrupt_firq_14_c   : natural := 17; -- fast interrupt channel 14
  constant interrupt_firq_15_c   : natural := 18; -- fast interrupt channel 15
  -- for debug mode only --
  constant interrupt_db_halt_c   : natural := 19; -- enter debug mode via external halt request ("async IRQ")
  constant interrupt_db_step_c   : natural := 20; -- enter debug mode via single-stepping ("async IRQ")
  --
  constant interrupt_width_c     : natural := 21; -- length of this list in bits

  -- CPU Privilege Modes --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant priv_mode_m_c : std_ulogic_vector(1 downto 0) := "11"; -- machine mode
  constant priv_mode_u_c : std_ulogic_vector(1 downto 0) := "00"; -- user mode

  -- HPM Event System -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant hpmcnt_event_cy_c      : natural := 0;  -- Active cycle
  constant hpmcnt_event_never_c   : natural := 1;  -- Unused / never (actually, this would be used for TIME)
  constant hpmcnt_event_ir_c      : natural := 2;  -- Retired instruction
  constant hpmcnt_event_cir_c     : natural := 3;  -- Retired compressed instruction
  constant hpmcnt_event_wait_if_c : natural := 4;  -- Instruction fetch memory wait cycle
  constant hpmcnt_event_wait_ii_c : natural := 5;  -- Instruction issue wait cycle
  constant hpmcnt_event_wait_mc_c : natural := 6;  -- Multi-cycle ALU-operation wait cycle
  constant hpmcnt_event_load_c    : natural := 7;  -- Load operation
  constant hpmcnt_event_store_c   : natural := 8;  -- Store operation
  constant hpmcnt_event_wait_ls_c : natural := 9;  -- Load/store memory wait cycle
  constant hpmcnt_event_jump_c    : natural := 10; -- Unconditional jump
  constant hpmcnt_event_branch_c  : natural := 11; -- Conditional branch (taken or not taken)
  constant hpmcnt_event_tbranch_c : natural := 12; -- Conditional taken branch
  constant hpmcnt_event_trap_c    : natural := 13; -- Entered trap
  constant hpmcnt_event_illegal_c : natural := 14; -- Illegal instruction exception
  --
  constant hpmcnt_event_size_c    : natural := 15; -- length of this list

  -- Clock Generator ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant clk_div2_c    : natural := 0;
  constant clk_div4_c    : natural := 1;
  constant clk_div8_c    : natural := 2;
  constant clk_div64_c   : natural := 3;
  constant clk_div128_c  : natural := 4;
  constant clk_div1024_c : natural := 5;
  constant clk_div2048_c : natural := 6;
  constant clk_div4096_c : natural := 7;

  -- Component: NEORV32 Processor Top Entity ------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_top
    generic (
      -- General --
      CLOCK_FREQUENCY              : natural;           -- clock frequency of clk_i in Hz
      HW_THREAD_ID                 : natural := 0;      -- hardware thread id (32-bit)
      INT_BOOTLOADER_EN            : boolean := false;  -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
      -- On-Chip Debugger (OCD) --
      ON_CHIP_DEBUGGER_EN          : boolean := false;  -- implement on-chip debugger
      -- RISC-V CPU Extensions --
      CPU_EXTENSION_RISCV_A        : boolean := false;  -- implement atomic extension?
      CPU_EXTENSION_RISCV_B        : boolean := false;  -- implement bit-manipulation extension?
      CPU_EXTENSION_RISCV_C        : boolean := false;  -- implement compressed extension?
      CPU_EXTENSION_RISCV_E        : boolean := false;  -- implement embedded RF extension?
      CPU_EXTENSION_RISCV_M        : boolean := false;  -- implement mul/div extension?
      CPU_EXTENSION_RISCV_U        : boolean := false;  -- implement user mode extension?
      CPU_EXTENSION_RISCV_Zfinx    : boolean := false;  -- implement 32-bit floating-point extension (using INT regs!)
      CPU_EXTENSION_RISCV_Zicsr    : boolean := true;   -- implement CSR system?
      CPU_EXTENSION_RISCV_Zicntr   : boolean := true;   -- implement base counters?
      CPU_EXTENSION_RISCV_Zihpm    : boolean := false;  -- implement hardware performance monitors?
      CPU_EXTENSION_RISCV_Zifencei : boolean := false;  -- implement instruction stream sync.?
      CPU_EXTENSION_RISCV_Zmmul    : boolean := false;  -- implement multiply-only M sub-extension?
      -- Extension Options --
      FAST_MUL_EN                  : boolean := false;  -- use DSPs for M extension's multiplier
      FAST_SHIFT_EN                : boolean := false;  -- use barrel shifter for shift operations
      CPU_CNT_WIDTH                : natural := 64;     -- total width of CPU cycle and instret counters (0..64)
      CPU_IPB_ENTRIES              : natural := 2;      -- entries is instruction prefetch buffer, has to be a power of 2
      -- Physical Memory Protection (PMP) --
      PMP_NUM_REGIONS              : natural := 0;      -- number of regions (0..64)
      PMP_MIN_GRANULARITY          : natural := 64*1024; -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
      -- Hardware Performance Monitors (HPM) --
      HPM_NUM_CNTS                 : natural := 0;      -- number of implemented HPM counters (0..29)
      HPM_CNT_WIDTH                : natural := 40;     -- total size of HPM counters (0..64)
      -- Internal Instruction memory (IMEM) --
      MEM_INT_IMEM_EN              : boolean := false;  -- implement processor-internal instruction memory
      MEM_INT_IMEM_SIZE            : natural := 16*1024; -- size of processor-internal instruction memory in bytes
      -- Internal Data memory (DMEM) --
      MEM_INT_DMEM_EN              : boolean := false;  -- implement processor-internal data memory
      MEM_INT_DMEM_SIZE            : natural := 8*1024; -- size of processor-internal data memory in bytes
      -- Internal Cache memory (iCACHE) --
      ICACHE_EN                    : boolean := false;  -- implement instruction cache
      ICACHE_NUM_BLOCKS            : natural := 4;      -- i-cache: number of blocks (min 1), has to be a power of 2
      ICACHE_BLOCK_SIZE            : natural := 64;     -- i-cache: block size in bytes (min 4), has to be a power of 2
      ICACHE_ASSOCIATIVITY         : natural := 1;      -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2
      -- External memory interface (WISHBONE) --
      MEM_EXT_EN                   : boolean := false;  -- implement external memory bus interface?
      MEM_EXT_TIMEOUT              : natural := 255;    -- cycles after a pending bus access auto-terminates (0 = disabled)
      MEM_EXT_PIPE_MODE            : boolean := false;  -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
      MEM_EXT_BIG_ENDIAN           : boolean := false;  -- byte order: true=big-endian, false=little-endian
      MEM_EXT_ASYNC_RX             : boolean := false;  -- use register buffer for RX data when false
      -- Stream link interface (SLINK) --
      SLINK_NUM_TX                 : natural := 0;      -- number of TX links (0..8)
      SLINK_NUM_RX                 : natural := 0;      -- number of TX links (0..8)
      SLINK_TX_FIFO                : natural := 1;      -- TX fifo depth, has to be a power of two
      SLINK_RX_FIFO                : natural := 1;      -- RX fifo depth, has to be a power of two
      -- External Interrupts Controller (XIRQ) --
      XIRQ_NUM_CH                  : natural := 0;      -- number of external IRQ channels (0..32)
      XIRQ_TRIGGER_TYPE            : std_ulogic_vector(31 downto 0) := x"FFFFFFFF"; -- trigger type: 0=level, 1=edge
      XIRQ_TRIGGER_POLARITY        : std_ulogic_vector(31 downto 0) := x"FFFFFFFF"; -- trigger polarity: 0=low-level/falling-edge, 1=high-level/rising-edge
      -- Processor peripherals --
      IO_GPIO_EN                   : boolean := false;  -- implement general purpose input/output port unit (GPIO)?
      IO_MTIME_EN                  : boolean := false;  -- implement machine system timer (MTIME)?
      IO_UART0_EN                  : boolean := false;  -- implement primary universal asynchronous receiver/transmitter (UART0)?
      IO_UART0_RX_FIFO             : natural := 1;      -- RX fifo depth, has to be a power of two, min 1
      IO_UART0_TX_FIFO             : natural := 1;      -- TX fifo depth, has to be a power of two, min 1
      IO_UART1_EN                  : boolean := false;  -- implement secondary universal asynchronous receiver/transmitter (UART1)?
      IO_UART1_RX_FIFO             : natural := 1;      -- RX fifo depth, has to be a power of two, min 1
      IO_UART1_TX_FIFO             : natural := 1;      -- TX fifo depth, has to be a power of two, min 1
      IO_SPI_EN                    : boolean := false;  -- implement serial peripheral interface (SPI)?
      IO_TWI_EN                    : boolean := false;  -- implement two-wire interface (TWI)?
      IO_PWM_NUM_CH                : natural := 0;      -- number of PWM channels to implement (0..60); 0 = disabled
      IO_WDT_EN                    : boolean := false;  -- implement watch dog timer (WDT)?
      IO_TRNG_EN                   : boolean := false;  -- implement true random number generator (TRNG)?
      IO_CFS_EN                    : boolean := false;  -- implement custom functions subsystem (CFS)?
      IO_CFS_CONFIG                : std_ulogic_vector(31 downto 0) := x"00000000"; -- custom CFS configuration generic
      IO_CFS_IN_SIZE               : positive := 32;    -- size of CFS input conduit in bits
      IO_CFS_OUT_SIZE              : positive := 32;    -- size of CFS output conduit in bits
      IO_NEOLED_EN                 : boolean := false;  -- implement NeoPixel-compatible smart LED interface (NEOLED)?
      IO_NEOLED_TX_FIFO            : natural := 1;      -- NEOLED TX FIFO depth, 1..32k, has to be a power of two
      IO_GPTMR_EN                  : boolean := false   -- implement general purpose timer (GPTMR)?
    );
    port (
      -- Global control --
      clk_i          : in  std_ulogic; -- global clock, rising edge
      rstn_i         : in  std_ulogic; -- global reset, low-active, async
      -- JTAG on-chip debugger interface --
      jtag_trst_i    : in  std_ulogic := 'U'; -- low-active TAP reset (optional)
      jtag_tck_i     : in  std_ulogic := 'U'; -- serial clock
      jtag_tdi_i     : in  std_ulogic := 'U'; -- serial data input
      jtag_tdo_o     : out std_ulogic;        -- serial data output
      jtag_tms_i     : in  std_ulogic := 'U'; -- mode select
      -- Wishbone bus interface (available if MEM_EXT_EN = true) --
      wb_tag_o       : out std_ulogic_vector(02 downto 0); -- request tag
      wb_adr_o       : out std_ulogic_vector(31 downto 0); -- address
      wb_dat_i       : in  std_ulogic_vector(31 downto 0) := (others => 'U'); -- read data
      wb_dat_o       : out std_ulogic_vector(31 downto 0); -- write data
      wb_we_o        : out std_ulogic; -- read/write
      wb_sel_o       : out std_ulogic_vector(03 downto 0); -- byte enable
      wb_stb_o       : out std_ulogic; -- strobe
      wb_cyc_o       : out std_ulogic; -- valid cycle
      wb_lock_o      : out std_ulogic; -- exclusive access request
      wb_ack_i       : in  std_ulogic := 'L'; -- transfer acknowledge
      wb_err_i       : in  std_ulogic := 'L'; -- transfer error
      -- Advanced memory control signals (available if MEM_EXT_EN = true) --
      fence_o        : out std_ulogic; -- indicates an executed FENCE operation
      fencei_o       : out std_ulogic; -- indicates an executed FENCEI operation
      -- TX stream interfaces (available if SLINK_NUM_TX > 0) --
      slink_tx_dat_o : out sdata_8x32_t; -- output data
      slink_tx_val_o : out std_ulogic_vector(7 downto 0); -- valid output
      slink_tx_rdy_i : in  std_ulogic_vector(7 downto 0) := (others => 'L'); -- ready to send
      -- RX stream interfaces (available if SLINK_NUM_RX > 0) --
      slink_rx_dat_i : in  sdata_8x32_t := (others => (others => 'U')); -- input data
      slink_rx_val_i : in  std_ulogic_vector(7 downto 0) := (others => 'L'); -- valid input
      slink_rx_rdy_o : out std_ulogic_vector(7 downto 0); -- ready to receive
      -- GPIO (available if IO_GPIO_EN = true) --
      gpio_o         : out std_ulogic_vector(63 downto 0); -- parallel output
      gpio_i         : in  std_ulogic_vector(63 downto 0) := (others => 'U'); -- parallel input
      -- primary UART0 (available if IO_UART0_EN = true) --
      uart0_txd_o    : out std_ulogic; -- UART0 send data
      uart0_rxd_i    : in  std_ulogic := 'U'; -- UART0 receive data
      uart0_rts_o    : out std_ulogic; -- hw flow control: UART0.RX ready to receive ("RTR"), low-active, optional
      uart0_cts_i    : in  std_ulogic := 'L'; -- hw flow control: UART0.TX allowed to transmit, low-active, optional
      -- secondary UART1 (available if IO_UART1_EN = true) --
      uart1_txd_o    : out std_ulogic; -- UART1 send data
      uart1_rxd_i    : in  std_ulogic := 'U'; -- UART1 receive data
      uart1_rts_o    : out std_ulogic; -- hw flow control: UART1.RX ready to receive ("RTR"), low-active, optional
      uart1_cts_i    : in  std_ulogic := 'L'; -- hw flow control: UART1.TX allowed to transmit, low-active, optional
      -- SPI (available if IO_SPI_EN = true) --
      spi_sck_o      : out std_ulogic; -- SPI serial clock
      spi_sdo_o      : out std_ulogic; -- controller data out, peripheral data in
      spi_sdi_i      : in  std_ulogic := 'U'; -- controller data in, peripheral data out
      spi_csn_o      : out std_ulogic_vector(07 downto 0); -- SPI CS
      -- TWI (available if IO_TWI_EN = true) --
      twi_sda_io     : inout std_logic := 'U'; -- twi serial data line
      twi_scl_io     : inout std_logic := 'U'; -- twi serial clock line
      -- PWM (available if IO_PWM_NUM_CH > 0) --
      pwm_o          : out std_ulogic_vector(IO_PWM_NUM_CH-1 downto 0); -- pwm channels
      -- Custom Functions Subsystem IO --
      cfs_in_i       : in  std_ulogic_vector(IO_CFS_IN_SIZE-1  downto 0) := (others => 'U'); -- custom CFS inputs conduit
      cfs_out_o      : out std_ulogic_vector(IO_CFS_OUT_SIZE-1 downto 0); -- custom CFS outputs conduit
      -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
      neoled_o       : out std_ulogic; -- async serial data line
      -- System time --
      mtime_i        : in  std_ulogic_vector(63 downto 0) := (others => 'U'); -- current system time from ext. MTIME (if IO_MTIME_EN = false)
      mtime_o        : out std_ulogic_vector(63 downto 0); -- current system time from int. MTIME (if IO_MTIME_EN = true)
      -- External platform interrupts (available if XIRQ_NUM_CH > 0) --
      xirq_i         : in  std_ulogic_vector(XIRQ_NUM_CH-1 downto 0) := (others => 'L'); -- IRQ channels
      -- CPU Interrupts --
      mtime_irq_i    : in  std_ulogic := 'L'; -- machine timer interrupt, available if IO_MTIME_EN = false
      msw_irq_i      : in  std_ulogic := 'L'; -- machine software interrupt
      mext_irq_i     : in  std_ulogic := 'L'  -- machine external interrupt
    );
  end component;

  -- Component: CPU Top Entity --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_cpu
    generic (
      -- General --
      HW_THREAD_ID                 : natural; -- hardware thread id (32-bit)
      CPU_BOOT_ADDR                : std_ulogic_vector(31 downto 0); -- cpu boot address
      CPU_DEBUG_ADDR               : std_ulogic_vector(31 downto 0); -- cpu debug mode start address
      -- RISC-V CPU Extensions --
      CPU_EXTENSION_RISCV_A        : boolean; -- implement atomic extension?
      CPU_EXTENSION_RISCV_B        : boolean; -- implement bit-manipulation extension?
      CPU_EXTENSION_RISCV_C        : boolean; -- implement compressed extension?
      CPU_EXTENSION_RISCV_E        : boolean; -- implement embedded RF extension?
      CPU_EXTENSION_RISCV_M        : boolean; -- implement mul/div extension?
      CPU_EXTENSION_RISCV_U        : boolean; -- implement user mode extension?
      CPU_EXTENSION_RISCV_Zfinx    : boolean; -- implement 32-bit floating-point extension (using INT reg!)
      CPU_EXTENSION_RISCV_Zicsr    : boolean; -- implement CSR system?
      CPU_EXTENSION_RISCV_Zicntr   : boolean; -- implement base counters?
      CPU_EXTENSION_RISCV_Zihpm    : boolean; -- implement hardware performance monitors?
      CPU_EXTENSION_RISCV_Zifencei : boolean; -- implement instruction stream sync.?
      CPU_EXTENSION_RISCV_Zmmul    : boolean; -- implement multiply-only M sub-extension?
      CPU_EXTENSION_RISCV_DEBUG    : boolean; -- implement CPU debug mode?
      -- Extension Options --
      FAST_MUL_EN                  : boolean; -- use DSPs for M extension's multiplier
      FAST_SHIFT_EN                : boolean; -- use barrel shifter for shift operations
      CPU_CNT_WIDTH                : natural; -- total width of CPU cycle and instret counters (0..64)
      CPU_IPB_ENTRIES              : natural; -- entries is instruction prefetch buffer, has to be a power of 2
      -- Physical Memory Protection (PMP) --
      PMP_NUM_REGIONS              : natural; -- number of regions (0..64)
      PMP_MIN_GRANULARITY          : natural; -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
      -- Hardware Performance Monitors (HPM) --
      HPM_NUM_CNTS                 : natural; -- number of implemented HPM counters (0..29)
      HPM_CNT_WIDTH                : natural  -- total size of HPM counters (0..64)
    );
    port (
      -- global control --
      clk_i          : in  std_ulogic; -- global clock, rising edge
      rstn_i         : in  std_ulogic; -- global reset, low-active, async
      sleep_o        : out std_ulogic; -- cpu is in sleep mode when set
      debug_o        : out std_ulogic; -- cpu is in debug mode when set
      -- instruction bus interface --
      i_bus_addr_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
      i_bus_rdata_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
      i_bus_wdata_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
      i_bus_ben_o    : out std_ulogic_vector(03 downto 0); -- byte enable
      i_bus_we_o     : out std_ulogic; -- write enable
      i_bus_re_o     : out std_ulogic; -- read enable
      i_bus_lock_o   : out std_ulogic; -- exclusive access request
      i_bus_ack_i    : in  std_ulogic; -- bus transfer acknowledge
      i_bus_err_i    : in  std_ulogic; -- bus transfer error
      i_bus_fence_o  : out std_ulogic; -- executed FENCEI operation
      i_bus_priv_o   : out std_ulogic_vector(1 downto 0); -- privilege level
      -- data bus interface --
      d_bus_addr_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
      d_bus_rdata_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
      d_bus_wdata_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
      d_bus_ben_o    : out std_ulogic_vector(03 downto 0); -- byte enable
      d_bus_we_o     : out std_ulogic; -- write enable
      d_bus_re_o     : out std_ulogic; -- read enable
      d_bus_lock_o   : out std_ulogic; -- exclusive access request
      d_bus_ack_i    : in  std_ulogic; -- bus transfer acknowledge
      d_bus_err_i    : in  std_ulogic; -- bus transfer error
      d_bus_fence_o  : out std_ulogic; -- executed FENCE operation
      d_bus_priv_o   : out std_ulogic_vector(1 downto 0); -- privilege level
      -- system time input from MTIME --
      time_i         : in  std_ulogic_vector(63 downto 0); -- current system time
      -- interrupts (risc-v compliant) --
      msw_irq_i      : in  std_ulogic; -- machine software interrupt
      mext_irq_i     : in  std_ulogic; -- machine external interrupt
      mtime_irq_i    : in  std_ulogic; -- machine timer interrupt
      -- fast interrupts (custom) --
      firq_i         : in  std_ulogic_vector(15 downto 0);
      -- debug mode (halt) request --
      db_halt_req_i  : in  std_ulogic
    );
  end component;

  -- Component: CPU Control -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_cpu_control
    generic (
      -- General --
      HW_THREAD_ID                 : natural;     -- hardware thread id (32-bit)
      CPU_BOOT_ADDR                : std_ulogic_vector(31 downto 0); -- cpu boot address
      CPU_DEBUG_ADDR               : std_ulogic_vector(31 downto 0); -- cpu debug mode start address
      -- RISC-V CPU Extensions --
      CPU_EXTENSION_RISCV_A        : boolean; -- implement atomic extension?
      CPU_EXTENSION_RISCV_B        : boolean; -- implement bit-manipulation extension?
      CPU_EXTENSION_RISCV_C        : boolean; -- implement compressed extension?
      CPU_EXTENSION_RISCV_E        : boolean; -- implement embedded RF extension?
      CPU_EXTENSION_RISCV_M        : boolean; -- implement mul/div extension?
      CPU_EXTENSION_RISCV_U        : boolean; -- implement user mode extension?
      CPU_EXTENSION_RISCV_Zfinx    : boolean; -- implement 32-bit floating-point extension (using INT reg!)
      CPU_EXTENSION_RISCV_Zicsr    : boolean; -- implement CSR system?
      CPU_EXTENSION_RISCV_Zicntr   : boolean; -- implement base counters?
      CPU_EXTENSION_RISCV_Zihpm    : boolean; -- implement hardware performance monitors?
      CPU_EXTENSION_RISCV_Zifencei : boolean; -- implement instruction stream sync.?
      CPU_EXTENSION_RISCV_Zmmul    : boolean; -- implement multiply-only M sub-extension?
      CPU_EXTENSION_RISCV_DEBUG    : boolean; -- implement CPU debug mode?
      -- Extension Options --
      CPU_CNT_WIDTH                : natural; -- total width of CPU cycle and instret counters (0..64)
      CPU_IPB_ENTRIES              : natural; -- entries is instruction prefetch buffer, has to be a power of 2
      -- Physical memory protection (PMP) --
      PMP_NUM_REGIONS              : natural; -- number of regions (0..64)
      PMP_MIN_GRANULARITY          : natural; -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
      -- Hardware Performance Monitors (HPM) --
      HPM_NUM_CNTS                 : natural; -- number of implemented HPM counters (0..29)
      HPM_CNT_WIDTH                : natural  -- total size of HPM counters (0..64)
    );
    port (
      -- global control --
      clk_i         : in  std_ulogic; -- global clock, rising edge
      rstn_i        : in  std_ulogic; -- global reset, low-active, async
      ctrl_o        : out std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
      -- status input --
      alu_idone_i   : in  std_ulogic; -- ALU iterative operation done
      bus_i_wait_i  : in  std_ulogic; -- wait for bus
      bus_d_wait_i  : in  std_ulogic; -- wait for bus
      excl_state_i  : in  std_ulogic; -- atomic/exclusive access lock status
      -- data input --
      instr_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- instruction
      cmp_i         : in  std_ulogic_vector(1 downto 0); -- comparator status
      alu_add_i     : in  std_ulogic_vector(data_width_c-1 downto 0); -- ALU address result
      rs1_i         : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 1
      -- data output --
      imm_o         : out std_ulogic_vector(data_width_c-1 downto 0); -- immediate
      fetch_pc_o    : out std_ulogic_vector(data_width_c-1 downto 0); -- PC for instruction fetch
      curr_pc_o     : out std_ulogic_vector(data_width_c-1 downto 0); -- current PC (corresponding to current instruction)
      next_pc_o     : out std_ulogic_vector(data_width_c-1 downto 0); -- next PC (corresponding to next instruction)
      csr_rdata_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- CSR read data
      -- FPU interface --
      fpu_flags_i   : in  std_ulogic_vector(04 downto 0); -- exception flags
      -- debug mode (halt) request --
      db_halt_req_i : in  std_ulogic;
      -- interrupts (risc-v compliant) --
      msw_irq_i     : in  std_ulogic; -- machine software interrupt
      mext_irq_i    : in  std_ulogic; -- machine external interrupt
      mtime_irq_i   : in  std_ulogic; -- machine timer interrupt
      -- fast interrupts (custom) --
      firq_i        : in  std_ulogic_vector(15 downto 0);
      -- system time input from MTIME --
      time_i        : in  std_ulogic_vector(63 downto 0); -- current system time
      -- physical memory protection --
      pmp_addr_o    : out pmp_addr_if_t; -- addresses
      pmp_ctrl_o    : out pmp_ctrl_if_t; -- configs
      -- bus access exceptions --
      mar_i         : in  std_ulogic_vector(data_width_c-1 downto 0); -- memory address register
      ma_instr_i    : in  std_ulogic; -- misaligned instruction address
      ma_load_i     : in  std_ulogic; -- misaligned load data address
      ma_store_i    : in  std_ulogic; -- misaligned store data address
      be_instr_i    : in  std_ulogic; -- bus error on instruction access
      be_load_i     : in  std_ulogic; -- bus error on load data access
      be_store_i    : in  std_ulogic  -- bus error on store data access
    );
  end component;

  -- Component: CPU Register File -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_cpu_regfile
    generic (
      CPU_EXTENSION_RISCV_E : boolean -- implement embedded RF extension?
    );
    port (
      -- global control --
      clk_i  : in  std_ulogic; -- global clock, rising edge
      ctrl_i : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
      -- data input --
      mem_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- memory read data
      alu_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- ALU result
      -- data output --
      rs1_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- operand 1
      rs2_o  : out std_ulogic_vector(data_width_c-1 downto 0)  -- operand 2
    );
  end component;

  -- Component: CPU ALU ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_cpu_alu
    generic (
      -- RISC-V CPU Extensions --
      CPU_EXTENSION_RISCV_B     : boolean; -- implement bit-manipulation extension?
      CPU_EXTENSION_RISCV_M     : boolean; -- implement mul/div extension?
      CPU_EXTENSION_RISCV_Zmmul : boolean; -- implement multiply-only M sub-extension?
      CPU_EXTENSION_RISCV_Zfinx : boolean; -- implement 32-bit floating-point extension (using INT reg!)
      -- Extension Options --
      FAST_MUL_EN               : boolean; -- use DSPs for M extension's multiplier
      FAST_SHIFT_EN             : boolean  -- use barrel shifter for shift operations
    );
    port (
      -- global control --
      clk_i       : in  std_ulogic; -- global clock, rising edge
      rstn_i      : in  std_ulogic; -- global reset, low-active, async
      ctrl_i      : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
      -- data input --
      rs1_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 1
      rs2_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 2
      pc_i        : in  std_ulogic_vector(data_width_c-1 downto 0); -- current PC
      pc2_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- next PC
      imm_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- immediate
      csr_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- CSR read data
      -- data output --
      cmp_o       : out std_ulogic_vector(1 downto 0); -- comparator status
      res_o       : out std_ulogic_vector(data_width_c-1 downto 0); -- ALU result
      add_o       : out std_ulogic_vector(data_width_c-1 downto 0); -- address computation result
      fpu_flags_o : out std_ulogic_vector(4 downto 0); -- FPU exception flags
      -- status --
      idone_o     : out std_ulogic -- iterative processing units done?
    );
  end component;

  -- Component: CPU Co-Processor SHIFTER ----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_cpu_cp_shifter
    generic (
      FAST_SHIFT_EN : boolean -- use barrel shifter for shift operations
    );
    port (
      -- global control --
      clk_i   : in  std_ulogic; -- global clock, rising edge
      rstn_i  : in  std_ulogic; -- global reset, low-active, async
      ctrl_i  : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
      start_i : in  std_ulogic; -- trigger operation
      -- data input --
      rs1_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 1
      shamt_i : in  std_ulogic_vector(index_size_f(data_width_c)-1 downto 0); -- shift amount
      -- result and status --
      res_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- operation result
      valid_o : out std_ulogic -- data output valid
    );
  end component;

  -- Component: CPU Co-Processor MULDIV ('M' extension) -------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_cpu_cp_muldiv
    generic (
      FAST_MUL_EN : boolean; -- use DSPs for faster multiplication
      DIVISION_EN : boolean  -- implement divider hardware
    );
    port (
      -- global control --
      clk_i   : in  std_ulogic; -- global clock, rising edge
      rstn_i  : in  std_ulogic; -- global reset, low-active, async
      ctrl_i  : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
      start_i : in  std_ulogic; -- trigger operation
      -- data input --
      rs1_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 1
      rs2_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 2
      -- result and status --
      res_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- operation result
      valid_o : out std_ulogic -- data output valid
    );
  end component;

  -- Component: CPU Co-Processor Bit-Manipulation Unit ('B' extension) ----------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_cpu_cp_bitmanip is
    generic (
      FAST_SHIFT_EN : boolean  -- use barrel shifter for shift operations
    );
    port (
      -- global control --
      clk_i   : in  std_ulogic; -- global clock, rising edge
      rstn_i  : in  std_ulogic; -- global reset, low-active, async
      ctrl_i  : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
      start_i : in  std_ulogic; -- trigger operation
      -- data input --
      cmp_i   : in  std_ulogic_vector(1 downto 0); -- comparator status
      rs1_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 1
      rs2_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 2
      shamt_i : in  std_ulogic_vector(index_size_f(data_width_c)-1 downto 0); -- shift amount
      -- result and status --
      res_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- operation result
      valid_o : out std_ulogic -- data output valid
    );
  end component;

  -- Component: CPU Co-Processor 32-bit FPU ('Zfinx' extension) -----------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_cpu_cp_fpu
    port (
      -- global control --
      clk_i    : in  std_ulogic; -- global clock, rising edge
      rstn_i   : in  std_ulogic; -- global reset, low-active, async
      ctrl_i   : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
      start_i  : in  std_ulogic; -- trigger operation
      -- data input --
      cmp_i    : in  std_ulogic_vector(1 downto 0); -- comparator status
      rs1_i    : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 1
      rs2_i    : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 2
      -- result and status --
      res_o    : out std_ulogic_vector(data_width_c-1 downto 0); -- operation result
      fflags_o : out std_ulogic_vector(4 downto 0); -- exception flags
      valid_o  : out std_ulogic -- data output valid
    );
  end component;

  -- Component: CPU Bus Interface -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_cpu_bus
    generic (
      CPU_EXTENSION_RISCV_A : boolean; -- implement atomic extension?
      CPU_EXTENSION_RISCV_C : boolean; -- implement compressed extension?
      -- Physical memory protection (PMP) --
      PMP_NUM_REGIONS       : natural; -- number of regions (0..64)
      PMP_MIN_GRANULARITY   : natural  -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
    );
    port (
      -- global control --
      clk_i          : in  std_ulogic; -- global clock, rising edge
      rstn_i      : in  std_ulogic := '0'; -- global reset, low-active, async
      ctrl_i         : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
      -- cpu instruction fetch interface --
      fetch_pc_i     : in  std_ulogic_vector(data_width_c-1 downto 0); -- PC for instruction fetch
      instr_o        : out std_ulogic_vector(data_width_c-1 downto 0); -- instruction
      i_wait_o       : out std_ulogic; -- wait for fetch to complete
      --
      ma_instr_o     : out std_ulogic; -- misaligned instruction address
      be_instr_o     : out std_ulogic; -- bus error on instruction access
      -- cpu data access interface --
      addr_i         : in  std_ulogic_vector(data_width_c-1 downto 0); -- ALU result -> access address
      wdata_i        : in  std_ulogic_vector(data_width_c-1 downto 0); -- write data
      rdata_o        : out std_ulogic_vector(data_width_c-1 downto 0); -- read data
      mar_o          : out std_ulogic_vector(data_width_c-1 downto 0); -- current memory address register
      d_wait_o       : out std_ulogic; -- wait for access to complete
      --
      excl_state_o   : out std_ulogic; -- atomic/exclusive access status
      ma_load_o      : out std_ulogic; -- misaligned load data address
      ma_store_o     : out std_ulogic; -- misaligned store data address
      be_load_o      : out std_ulogic; -- bus error on load data access
      be_store_o     : out std_ulogic; -- bus error on store data access
      -- physical memory protection --
      pmp_addr_i     : in  pmp_addr_if_t; -- addresses
      pmp_ctrl_i     : in  pmp_ctrl_if_t; -- configs
      -- instruction bus --
      i_bus_addr_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
      i_bus_rdata_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
      i_bus_wdata_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
      i_bus_ben_o    : out std_ulogic_vector(03 downto 0); -- byte enable
      i_bus_we_o     : out std_ulogic; -- write enable
      i_bus_re_o     : out std_ulogic; -- read enable
      i_bus_lock_o   : out std_ulogic; -- exclusive access request
      i_bus_ack_i    : in  std_ulogic; -- bus transfer acknowledge
      i_bus_err_i    : in  std_ulogic; -- bus transfer error
      i_bus_fence_o  : out std_ulogic; -- fence operation
      -- data bus --
      d_bus_addr_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
      d_bus_rdata_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
      d_bus_wdata_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
      d_bus_ben_o    : out std_ulogic_vector(03 downto 0); -- byte enable
      d_bus_we_o     : out std_ulogic; -- write enable
      d_bus_re_o     : out std_ulogic; -- read enable
      d_bus_lock_o   : out std_ulogic; -- exclusive access request
      d_bus_ack_i    : in  std_ulogic; -- bus transfer acknowledge
      d_bus_err_i    : in  std_ulogic; -- bus transfer error
      d_bus_fence_o  : out std_ulogic  -- fence operation
    );
  end component;

  -- Component: Bus Keeper ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_bus_keeper is
    port (
      -- host access --
      clk_i      : in  std_ulogic; -- global clock line
      rstn_i     : in  std_ulogic; -- global reset, low-active, async
      addr_i     : in  std_ulogic_vector(31 downto 0); -- address
      rden_i     : in  std_ulogic; -- read enable
      wren_i     : in  std_ulogic; -- write enable
      data_o     : out std_ulogic_vector(31 downto 0); -- data out
      ack_o      : out std_ulogic; -- transfer acknowledge
      err_o      : out std_ulogic; -- transfer error
      -- bus monitoring --
      bus_addr_i : in  std_ulogic_vector(31 downto 0); -- address
      bus_rden_i : in  std_ulogic; -- read enable
      bus_wren_i : in  std_ulogic; -- write enable
      bus_ack_i  : in  std_ulogic; -- transfer acknowledge from bus system
      bus_err_i  : in  std_ulogic; -- transfer error from bus system
      bus_tmo_i  : in  std_ulogic; -- transfer timeout (external interface)
      bus_ext_i  : in  std_ulogic  -- external bus access
    );
  end component;

  -- Component: CPU Instruction Cache -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_icache
    generic (
      ICACHE_NUM_BLOCKS : natural; -- number of blocks (min 1), has to be a power of 2
      ICACHE_BLOCK_SIZE : natural; -- block size in bytes (min 4), has to be a power of 2
      ICACHE_NUM_SETS   : natural  -- associativity / number of sets (1=direct_mapped), has to be a power of 2
    );
    port (
      -- global control --
      clk_i         : in  std_ulogic; -- global clock, rising edge
      rstn_i        : in  std_ulogic; -- global reset, low-active, async
      clear_i       : in  std_ulogic; -- cache clear
      -- host controller interface --
      host_addr_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
      host_rdata_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
      host_wdata_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
      host_ben_i    : in  std_ulogic_vector(03 downto 0); -- byte enable
      host_we_i     : in  std_ulogic; -- write enable
      host_re_i     : in  std_ulogic; -- read enable
      host_ack_o    : out std_ulogic; -- bus transfer acknowledge
      host_err_o    : out std_ulogic; -- bus transfer error
      -- peripheral bus interface --
      bus_addr_o    : out std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
      bus_rdata_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
      bus_wdata_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
      bus_ben_o     : out std_ulogic_vector(03 downto 0); -- byte enable
      bus_we_o      : out std_ulogic; -- write enable
      bus_re_o      : out std_ulogic; -- read enable
      bus_ack_i     : in  std_ulogic; -- bus transfer acknowledge
      bus_err_i     : in  std_ulogic  -- bus transfer error
    );
  end component;

  -- Component: CPU Bus Switch --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_busswitch
    generic (
      PORT_CA_READ_ONLY : boolean; -- set if controller port A is read-only
      PORT_CB_READ_ONLY : boolean  -- set if controller port B is read-only
    );
    port (
      -- global control --
      clk_i           : in  std_ulogic; -- global clock, rising edge
      rstn_i          : in  std_ulogic; -- global reset, low-active, async
      -- controller interface a --
      ca_bus_addr_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
      ca_bus_rdata_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
      ca_bus_wdata_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
      ca_bus_ben_i    : in  std_ulogic_vector(03 downto 0); -- byte enable
      ca_bus_we_i     : in  std_ulogic; -- write enable
      ca_bus_re_i     : in  std_ulogic; -- read enable
      ca_bus_lock_i   : in  std_ulogic; -- exclusive access request
      ca_bus_ack_o    : out std_ulogic; -- bus transfer acknowledge
      ca_bus_err_o    : out std_ulogic; -- bus transfer error
      -- controller interface b --
      cb_bus_addr_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
      cb_bus_rdata_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
      cb_bus_wdata_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
      cb_bus_ben_i    : in  std_ulogic_vector(03 downto 0); -- byte enable
      cb_bus_we_i     : in  std_ulogic; -- write enable
      cb_bus_re_i     : in  std_ulogic; -- read enable
      cb_bus_lock_i   : in  std_ulogic; -- exclusive access request
      cb_bus_ack_o    : out std_ulogic; -- bus transfer acknowledge
      cb_bus_err_o    : out std_ulogic; -- bus transfer error
      -- peripheral bus --
      p_bus_src_o     : out std_ulogic; -- access source: 0 = A, 1 = B
      p_bus_addr_o    : out std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
      p_bus_rdata_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
      p_bus_wdata_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
      p_bus_ben_o     : out std_ulogic_vector(03 downto 0); -- byte enable
      p_bus_we_o      : out std_ulogic; -- write enable
      p_bus_re_o      : out std_ulogic; -- read enable
      p_bus_lock_o    : out std_ulogic; -- exclusive access request
      p_bus_ack_i     : in  std_ulogic; -- bus transfer acknowledge
      p_bus_err_i     : in  std_ulogic  -- bus transfer error
    );
  end component;

  -- Component: CPU Compressed Instructions Decompressor ------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_cpu_decompressor
    port (
      -- instruction input --
      ci_instr16_i : in  std_ulogic_vector(15 downto 0); -- compressed instruction input
      -- instruction output --
      ci_illegal_o : out std_ulogic; -- is an illegal compressed instruction
      ci_instr32_o : out std_ulogic_vector(31 downto 0)  -- 32-bit decompressed instruction
    );
  end component;

  -- Component: Processor-internal instruction memory (IMEM) --------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_imem
    generic (
      IMEM_BASE    : std_ulogic_vector(31 downto 0); -- memory base address
      IMEM_SIZE    : natural; -- processor-internal instruction memory size in bytes
      IMEM_AS_IROM : boolean  -- implement IMEM as pre-initialized read-only memory?
    );
    port (
      clk_i  : in  std_ulogic; -- global clock line
      rden_i : in  std_ulogic; -- read enable
      wren_i : in  std_ulogic; -- write enable
      ben_i  : in  std_ulogic_vector(03 downto 0); -- byte write enable
      addr_i : in  std_ulogic_vector(31 downto 0); -- address
      data_i : in  std_ulogic_vector(31 downto 0); -- data in
      data_o : out std_ulogic_vector(31 downto 0); -- data out
      ack_o  : out std_ulogic -- transfer acknowledge
    );
  end component;

  -- Component: Processor-internal data memory (DMEM) ---------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_dmem
    generic (
      DMEM_BASE : std_ulogic_vector(31 downto 0); -- memory base address
      DMEM_SIZE : natural -- processor-internal instruction memory size in bytes
    );
    port (
      clk_i  : in  std_ulogic; -- global clock line
      rden_i : in  std_ulogic; -- read enable
      wren_i : in  std_ulogic; -- write enable
      ben_i  : in  std_ulogic_vector(03 downto 0); -- byte write enable
      addr_i : in  std_ulogic_vector(31 downto 0); -- address
      data_i : in  std_ulogic_vector(31 downto 0); -- data in
      data_o : out std_ulogic_vector(31 downto 0); -- data out
      ack_o  : out std_ulogic -- transfer acknowledge
    );
  end component;

  -- Component: Processor-internal bootloader ROM (BOOTROM) ---------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_boot_rom
    generic (
      BOOTROM_BASE : std_ulogic_vector(31 downto 0) -- boot ROM base address
    );
    port (
      clk_i  : in  std_ulogic; -- global clock line
      rden_i : in  std_ulogic; -- read enable
      addr_i : in  std_ulogic_vector(31 downto 0); -- address
      data_o : out std_ulogic_vector(31 downto 0); -- data out
      ack_o  : out std_ulogic -- transfer acknowledge
    );
  end component;

  -- Component: Machine System Timer (mtime) ------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_mtime
    port (
      -- host access --
      clk_i  : in  std_ulogic; -- global clock line
      addr_i : in  std_ulogic_vector(31 downto 0); -- address
      rden_i : in  std_ulogic; -- read enable
      wren_i : in  std_ulogic; -- write enable
      data_i : in  std_ulogic_vector(31 downto 0); -- data in
      data_o : out std_ulogic_vector(31 downto 0); -- data out
      ack_o  : out std_ulogic; -- transfer acknowledge
      -- time output for CPU --
      time_o : out std_ulogic_vector(63 downto 0); -- current system time
      -- interrupt --
      irq_o  : out std_ulogic  -- interrupt request
    );
  end component;

  -- Component: General Purpose Input/Output Port (GPIO) ------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_gpio
    port (
      -- host access --
      clk_i  : in  std_ulogic; -- global clock line
      addr_i : in  std_ulogic_vector(31 downto 0); -- address
      rden_i : in  std_ulogic; -- read enable
      wren_i : in  std_ulogic; -- write enable
      data_i : in  std_ulogic_vector(31 downto 0); -- data in
      data_o : out std_ulogic_vector(31 downto 0); -- data out
      ack_o  : out std_ulogic; -- transfer acknowledge
      -- parallel io --
      gpio_o : out std_ulogic_vector(63 downto 0);
      gpio_i : in  std_ulogic_vector(63 downto 0)
    );
  end component;

  -- Component: Watchdog Timer (WDT) --------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_wdt
    generic (
      DEBUG_EN : boolean -- CPU debug mode implemented?
    );
    port (
      -- host access --
      clk_i       : in  std_ulogic; -- global clock line
      rstn_i      : in  std_ulogic; -- global reset line, low-active
      rden_i      : in  std_ulogic; -- read enable
      wren_i      : in  std_ulogic; -- write enable
      addr_i      : in  std_ulogic_vector(31 downto 0); -- address
      data_i      : in  std_ulogic_vector(31 downto 0); -- data in
      data_o      : out std_ulogic_vector(31 downto 0); -- data out
      ack_o       : out std_ulogic; -- transfer acknowledge
      -- CPU in debug mode? --
      cpu_debug_i : in  std_ulogic;
      -- clock generator --
      clkgen_en_o : out std_ulogic; -- enable clock generator
      clkgen_i    : in  std_ulogic_vector(07 downto 0);
      -- timeout event --
      irq_o       : out std_ulogic; -- timeout IRQ
      rstn_o      : out std_ulogic  -- timeout reset, low_active, use it as async!
    );
  end component;

  -- Component: Universal Asynchronous Receiver and Transmitter (UART) ----------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_uart
    generic (
      UART_PRIMARY : boolean; -- true = primary UART (UART0), false = secondary UART (UART1)
      UART_RX_FIFO : natural; -- RX fifo depth, has to be a power of two, min 1
      UART_TX_FIFO : natural  -- TX fifo depth, has to be a power of two, min 1
    );
    port (
      -- host access --
      clk_i       : in  std_ulogic; -- global clock line
      addr_i      : in  std_ulogic_vector(31 downto 0); -- address
      rden_i      : in  std_ulogic; -- read enable
      wren_i      : in  std_ulogic; -- write enable
      data_i      : in  std_ulogic_vector(31 downto 0); -- data in
      data_o      : out std_ulogic_vector(31 downto 0); -- data out
      ack_o       : out std_ulogic; -- transfer acknowledge
      -- clock generator --
      clkgen_en_o : out std_ulogic; -- enable clock generator
      clkgen_i    : in  std_ulogic_vector(07 downto 0);
      -- com lines --
      uart_txd_o  : out std_ulogic;
      uart_rxd_i  : in  std_ulogic;
      -- hardware flow control --
      uart_rts_o  : out std_ulogic; -- UART.RX ready to receive ("RTR"), low-active, optional
      uart_cts_i  : in  std_ulogic; -- UART.TX allowed to transmit, low-active, optional
      -- interrupts --
      irq_rxd_o   : out std_ulogic; -- uart data received interrupt
      irq_txd_o   : out std_ulogic  -- uart transmission done interrupt
    );
  end component;

  -- Component: Serial Peripheral Interface (SPI) -------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_spi
    port (
      -- host access --
      clk_i       : in  std_ulogic; -- global clock line
      addr_i      : in  std_ulogic_vector(31 downto 0); -- address
      rden_i      : in  std_ulogic; -- read enable
      wren_i      : in  std_ulogic; -- write enable
      data_i      : in  std_ulogic_vector(31 downto 0); -- data in
      data_o      : out std_ulogic_vector(31 downto 0); -- data out
      ack_o       : out std_ulogic; -- transfer acknowledge
      -- clock generator --
      clkgen_en_o : out std_ulogic; -- enable clock generator
      clkgen_i    : in  std_ulogic_vector(07 downto 0);
      -- com lines --
      spi_sck_o   : out std_ulogic; -- SPI serial clock
      spi_sdo_o   : out std_ulogic; -- controller data out, peripheral data in
      spi_sdi_i   : in  std_ulogic; -- controller data in, peripheral data out
      spi_csn_o   : out std_ulogic_vector(07 downto 0); -- SPI CS
      -- interrupt --
      irq_o       : out std_ulogic -- transmission done interrupt
    );
  end component;

  -- Component: Two-Wire Interface (TWI) ----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_twi
    port (
      -- host access --
      clk_i       : in  std_ulogic; -- global clock line
      addr_i      : in  std_ulogic_vector(31 downto 0); -- address
      rden_i      : in  std_ulogic; -- read enable
      wren_i      : in  std_ulogic; -- write enable
      data_i      : in  std_ulogic_vector(31 downto 0); -- data in
      data_o      : out std_ulogic_vector(31 downto 0); -- data out
      ack_o       : out std_ulogic; -- transfer acknowledge
      -- clock generator --
      clkgen_en_o : out std_ulogic; -- enable clock generator
      clkgen_i    : in  std_ulogic_vector(07 downto 0);
      -- com lines --
      twi_sda_io  : inout std_logic; -- serial data line
      twi_scl_io  : inout std_logic; -- serial clock line
      -- interrupt --
      irq_o       : out std_ulogic -- transfer done IRQ
    );
  end component;

  -- Component: Pulse-Width Modulation Controller (PWM) -------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_pwm
    generic (
      NUM_CHANNELS : natural -- number of PWM channels (0..60)
    );
    port (
      -- host access --
      clk_i       : in  std_ulogic; -- global clock line
      addr_i      : in  std_ulogic_vector(31 downto 0); -- address
      rden_i      : in  std_ulogic; -- read enable
      wren_i      : in  std_ulogic; -- write enable
      data_i      : in  std_ulogic_vector(31 downto 0); -- data in
      data_o      : out std_ulogic_vector(31 downto 0); -- data out
      ack_o       : out std_ulogic; -- transfer acknowledge
      -- clock generator --
      clkgen_en_o : out std_ulogic; -- enable clock generator
      clkgen_i    : in  std_ulogic_vector(07 downto 0);
      -- pwm output channels --
      pwm_o       : out std_ulogic_vector(NUM_CHANNELS-1 downto 0)
    );
  end component;

  -- Component: True Random Number Generator (TRNG) -----------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_trng
    port (
      -- host access --
      clk_i  : in  std_ulogic; -- global clock line
      addr_i : in  std_ulogic_vector(31 downto 0); -- address
      rden_i : in  std_ulogic; -- read enable
      wren_i : in  std_ulogic; -- write enable
      data_i : in  std_ulogic_vector(31 downto 0); -- data in
      data_o : out std_ulogic_vector(31 downto 0); -- data out
      ack_o  : out std_ulogic  -- transfer acknowledge
    );
  end component;

  -- Component: Wishbone Bus Gateway (WISHBONE) ---------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_wishbone
    generic (
      -- Internal instruction memory --
      MEM_INT_IMEM_EN   : boolean; -- implement processor-internal instruction memory
      MEM_INT_IMEM_SIZE : natural; -- size of processor-internal instruction memory in bytes
      -- Internal data memory --
      MEM_INT_DMEM_EN   : boolean; -- implement processor-internal data memory
      MEM_INT_DMEM_SIZE : natural; -- size of processor-internal data memory in bytes
      -- Interface Configuration --
      BUS_TIMEOUT       : natural; -- cycles after an UNACKNOWLEDGED bus access triggers a bus fault exception
      PIPE_MODE         : boolean; -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
      BIG_ENDIAN        : boolean; -- byte order: true=big-endian, false=little-endian
      ASYNC_RX          : boolean  -- use register buffer for RX data when false
    );
    port (
      -- global control --
      clk_i     : in  std_ulogic; -- global clock line
      rstn_i    : in  std_ulogic; -- global reset line, low-active
      -- host access --
      src_i     : in  std_ulogic; -- access type (0: data, 1:instruction)
      addr_i    : in  std_ulogic_vector(31 downto 0); -- address
      rden_i    : in  std_ulogic; -- read enable
      wren_i    : in  std_ulogic; -- write enable
      ben_i     : in  std_ulogic_vector(03 downto 0); -- byte write enable
      data_i    : in  std_ulogic_vector(31 downto 0); -- data in
      data_o    : out std_ulogic_vector(31 downto 0); -- data out
      lock_i    : in  std_ulogic; -- exclusive access request
      ack_o     : out std_ulogic; -- transfer acknowledge
      err_o     : out std_ulogic; -- transfer error
      tmo_o     : out std_ulogic; -- transfer timeout
      priv_i    : in  std_ulogic_vector(01 downto 0); -- current CPU privilege level
      ext_o     : out std_ulogic; -- active external access
      -- wishbone interface --
      wb_tag_o  : out std_ulogic_vector(02 downto 0); -- request tag
      wb_adr_o  : out std_ulogic_vector(31 downto 0); -- address
      wb_dat_i  : in  std_ulogic_vector(31 downto 0); -- read data
      wb_dat_o  : out std_ulogic_vector(31 downto 0); -- write data
      wb_we_o   : out std_ulogic; -- read/write
      wb_sel_o  : out std_ulogic_vector(03 downto 0); -- byte enable
      wb_stb_o  : out std_ulogic; -- strobe
      wb_cyc_o  : out std_ulogic; -- valid cycle
      wb_lock_o : out std_ulogic; -- exclusive access request
      wb_ack_i  : in  std_ulogic; -- transfer acknowledge
      wb_err_i  : in  std_ulogic  -- transfer error
    );
  end component;

  -- Component: Custom Functions Subsystem (CFS) --------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_cfs
    generic (
      CFS_CONFIG   : std_ulogic_vector(31 downto 0); -- custom CFS configuration generic
      CFS_IN_SIZE  : positive; -- size of CFS input conduit in bits
      CFS_OUT_SIZE : positive  -- size of CFS output conduit in bits
    );
    port (
      -- host access --
      clk_i       : in  std_ulogic; -- global clock line
      rstn_i      : in  std_ulogic; -- global reset line, low-active, use as async
      addr_i      : in  std_ulogic_vector(31 downto 0); -- address
      rden_i      : in  std_ulogic; -- read enable
      wren_i      : in  std_ulogic; -- word write enable
      data_i      : in  std_ulogic_vector(31 downto 0); -- data in
      data_o      : out std_ulogic_vector(31 downto 0); -- data out
      ack_o       : out std_ulogic; -- transfer acknowledge
      err_o       : out std_ulogic; -- transfer error
      -- clock generator --
      clkgen_en_o : out std_ulogic; -- enable clock generator
      clkgen_i    : in  std_ulogic_vector(07 downto 0); -- "clock" inputs
      -- interrupt --
      irq_o       : out std_ulogic; -- interrupt request
      -- custom io (conduit) --
      cfs_in_i    : in  std_ulogic_vector(CFS_IN_SIZE-1 downto 0); -- custom inputs
      cfs_out_o   : out std_ulogic_vector(CFS_OUT_SIZE-1 downto 0) -- custom outputs
    );
  end component;

  -- Component: Smart LED (WS2811/WS2812) Interface (NEOLED) --------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_neoled
    generic (
      FIFO_DEPTH : natural -- TX FIFO depth (1..32k, power of two)
    );
    port (
      -- host access --
      clk_i       : in  std_ulogic; -- global clock line
      addr_i      : in  std_ulogic_vector(31 downto 0); -- address
      rden_i      : in  std_ulogic; -- read enable
      wren_i      : in  std_ulogic; -- write enable
      data_i      : in  std_ulogic_vector(31 downto 0); -- data in
      data_o      : out std_ulogic_vector(31 downto 0); -- data out
      ack_o       : out std_ulogic; -- transfer acknowledge
      -- clock generator --
      clkgen_en_o : out std_ulogic; -- enable clock generator
      clkgen_i    : in  std_ulogic_vector(07 downto 0);
      -- interrupt --
      irq_o       : out std_ulogic; -- interrupt request
      -- NEOLED output --
      neoled_o    : out std_ulogic -- serial async data line
    );
  end component;

  -- Component: Stream Link Interface (SLINK) -----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_slink
    generic (
      SLINK_NUM_TX  : natural; -- number of TX links (0..8)
      SLINK_NUM_RX  : natural; -- number of TX links (0..8)
      SLINK_TX_FIFO : natural; -- TX fifo depth, has to be a power of two
      SLINK_RX_FIFO : natural  -- RX fifo depth, has to be a power of two
    );
    port (
      -- host access --
      clk_i          : in  std_ulogic; -- global clock line
      addr_i         : in  std_ulogic_vector(31 downto 0); -- address
      rden_i         : in  std_ulogic; -- read enable
      wren_i         : in  std_ulogic; -- write enable
      data_i         : in  std_ulogic_vector(31 downto 0); -- data in
      data_o         : out std_ulogic_vector(31 downto 0); -- data out
      ack_o          : out std_ulogic; -- transfer acknowledge
      -- interrupt --
      irq_tx_o       : out std_ulogic; -- transmission done
      irq_rx_o       : out std_ulogic; -- data received
      -- TX stream interfaces --
      slink_tx_dat_o : out sdata_8x32_t; -- output data
      slink_tx_val_o : out std_ulogic_vector(7 downto 0); -- valid output
      slink_tx_rdy_i : in  std_ulogic_vector(7 downto 0); -- ready to send
      -- RX stream interfaces --
      slink_rx_dat_i : in  sdata_8x32_t; -- input data
      slink_rx_val_i : in  std_ulogic_vector(7 downto 0); -- valid input
      slink_rx_rdy_o : out std_ulogic_vector(7 downto 0)  -- ready to receive
    );
  end component;

  -- Component: External Interrupt Controller (XIRQ) ----------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_xirq
    generic (
      XIRQ_NUM_CH           : natural; -- number of external IRQ channels (0..32)
      XIRQ_TRIGGER_TYPE     : std_ulogic_vector(31 downto 0); -- trigger type: 0=level, 1=edge
      XIRQ_TRIGGER_POLARITY : std_ulogic_vector(31 downto 0)  -- trigger polarity: 0=low-level/falling-edge, 1=high-level/rising-edge
    );
    port (
      -- host access --
      clk_i     : in  std_ulogic; -- global clock line
      addr_i    : in  std_ulogic_vector(31 downto 0); -- address
      rden_i    : in  std_ulogic; -- read enable
      wren_i    : in  std_ulogic; -- write enable
      data_i    : in  std_ulogic_vector(31 downto 0); -- data in
      data_o    : out std_ulogic_vector(31 downto 0); -- data out
      ack_o     : out std_ulogic; -- transfer acknowledge
      -- external interrupt lines --
      xirq_i    : in  std_ulogic_vector(XIRQ_NUM_CH-1 downto 0);
      -- CPU interrupt --
      cpu_irq_o : out std_ulogic
    );
  end component;

  -- Component: General Purpose Timer (GPTMR) -----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_gptmr
    port (
      -- host access --
      clk_i       : in  std_ulogic; -- global clock line
      addr_i      : in  std_ulogic_vector(31 downto 0); -- address
      rden_i      : in  std_ulogic; -- read enable
      wren_i      : in  std_ulogic; -- write enable
      data_i      : in  std_ulogic_vector(31 downto 0); -- data in
      data_o      : out std_ulogic_vector(31 downto 0); -- data out
      ack_o       : out std_ulogic; -- transfer acknowledge
      -- clock generator --
      clkgen_en_o : out std_ulogic; -- enable clock generator
      clkgen_i    : in  std_ulogic_vector(07 downto 0);
      -- interrupt --
      irq_o       : out std_ulogic -- transmission done interrupt
    );
  end component;

  -- Component: System Configuration Information Memory (SYSINFO) ---------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_sysinfo
    generic (
      -- General --
      CLOCK_FREQUENCY              : natural; -- clock frequency of clk_i in Hz
      INT_BOOTLOADER_EN            : boolean; -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
      -- RISC-V CPU Extensions --
      CPU_EXTENSION_RISCV_Zfinx    : boolean; -- implement 32-bit floating-point extension (using INT reg!)
      CPU_EXTENSION_RISCV_Zicsr    : boolean; -- implement CSR system?
      CPU_EXTENSION_RISCV_Zicntr   : boolean; -- implement base counters?
      CPU_EXTENSION_RISCV_Zihpm    : boolean; -- implement hardware performance monitors?
      CPU_EXTENSION_RISCV_Zifencei : boolean; -- implement instruction stream sync.?
      CPU_EXTENSION_RISCV_Zmmul    : boolean; -- implement multiply-only M sub-extension?
      CPU_EXTENSION_RISCV_DEBUG    : boolean; -- implement CPU debug mode?
      -- Extension Options --
      FAST_MUL_EN                  : boolean; -- use DSPs for M extension's multiplier
      FAST_SHIFT_EN                : boolean; -- use barrel shifter for shift operations
      CPU_CNT_WIDTH                : natural; -- total width of CPU cycle and instret counters (0..64)
      -- Physical memory protection (PMP) --
      PMP_NUM_REGIONS              : natural; -- number of regions (0..64)
      -- Internal Instruction memory --
      MEM_INT_IMEM_EN              : boolean; -- implement processor-internal instruction memory
      MEM_INT_IMEM_SIZE            : natural; -- size of processor-internal instruction memory in bytes
      -- Internal Data memory --
      MEM_INT_DMEM_EN              : boolean; -- implement processor-internal data memory
      MEM_INT_DMEM_SIZE            : natural; -- size of processor-internal data memory in bytes
      -- Internal Cache memory --
      ICACHE_EN                    : boolean; -- implement instruction cache
      ICACHE_NUM_BLOCKS            : natural; -- i-cache: number of blocks (min 2), has to be a power of 2
      ICACHE_BLOCK_SIZE            : natural; -- i-cache: block size in bytes (min 4), has to be a power of 2
      ICACHE_ASSOCIATIVITY         : natural; -- i-cache: associativity (min 1), has to be a power 2
      -- External memory interface --
      MEM_EXT_EN                   : boolean; -- implement external memory bus interface?
      MEM_EXT_BIG_ENDIAN           : boolean; -- byte order: true=big-endian, false=little-endian
      -- On-Chip Debugger --
      ON_CHIP_DEBUGGER_EN          : boolean; -- implement OCD?
      -- Processor peripherals --
      IO_GPIO_EN                   : boolean; -- implement general purpose input/output port unit (GPIO)?
      IO_MTIME_EN                  : boolean; -- implement machine system timer (MTIME)?
      IO_UART0_EN                  : boolean; -- implement primary universal asynchronous receiver/transmitter (UART0)?
      IO_UART1_EN                  : boolean; -- implement secondary universal asynchronous receiver/transmitter (UART1)?
      IO_SPI_EN                    : boolean; -- implement serial peripheral interface (SPI)?
      IO_TWI_EN                    : boolean; -- implement two-wire interface (TWI)?
      IO_PWM_NUM_CH                : natural; -- number of PWM channels to implement
      IO_WDT_EN                    : boolean; -- implement watch dog timer (WDT)?
      IO_TRNG_EN                   : boolean; -- implement true random number generator (TRNG)?
      IO_CFS_EN                    : boolean; -- implement custom functions subsystem (CFS)?
      IO_SLINK_EN                  : boolean; -- implement stream link interface?
      IO_NEOLED_EN                 : boolean; -- implement NeoPixel-compatible smart LED interface (NEOLED)?
      IO_XIRQ_NUM_CH               : natural; -- number of external interrupt (XIRQ) channels to implement
      IO_GPTMR_EN                  : boolean  -- implement general purpose timer (GPTMR)?
    );
    port (
      -- host access --
      clk_i  : in  std_ulogic; -- global clock line
      addr_i : in  std_ulogic_vector(31 downto 0); -- address
      rden_i : in  std_ulogic; -- read enable
      data_o : out std_ulogic_vector(31 downto 0); -- data out
      ack_o  : out std_ulogic  -- transfer acknowledge
    );
  end component;

  -- Component: General Purpose FIFO --------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_fifo
    generic (
      FIFO_DEPTH : natural; -- number of fifo entries; has to be a power of two; min 1
      FIFO_WIDTH : natural; -- size of data elements in fifo
      FIFO_RSYNC : boolean; -- false = async read; true = sync read
      FIFO_SAFE  : boolean  -- true = allow read/write only if entry available
    );
    port (
      -- control --
      clk_i   : in  std_ulogic; -- clock, rising edge
      rstn_i  : in  std_ulogic; -- async reset, low-active
      clear_i : in  std_ulogic; -- sync reset, high-active
      level_o : out std_ulogic_vector(index_size_f(FIFO_DEPTH) downto 0); -- fill level
      half_o  : out std_ulogic; -- FIFO is at least half full
      -- write port --
      wdata_i : in  std_ulogic_vector(FIFO_WIDTH-1 downto 0); -- write data
      we_i    : in  std_ulogic; -- write enable
      free_o  : out std_ulogic; -- at least one entry is free when set
      -- read port --
      re_i    : in  std_ulogic; -- read enable
      rdata_o : out std_ulogic_vector(FIFO_WIDTH-1 downto 0); -- read data
      avail_o : out std_ulogic  -- data available when set
    );
  end component;

  -- Component: On-Chip Debugger - Debug Module (DM) ----------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_debug_dm
    port (
      -- global control --
      clk_i            : in  std_ulogic; -- global clock line
      rstn_i           : in  std_ulogic; -- global reset line, low-active
      -- debug module interface (DMI) --
      dmi_rstn_i       : in  std_ulogic;
      dmi_req_valid_i  : in  std_ulogic;
      dmi_req_ready_o  : out std_ulogic; -- DMI is allowed to make new requests when set
      dmi_req_addr_i   : in  std_ulogic_vector(06 downto 0);
      dmi_req_op_i     : in  std_ulogic; -- 0=read, 1=write
      dmi_req_data_i   : in  std_ulogic_vector(31 downto 0);
      dmi_resp_valid_o : out std_ulogic; -- response valid when set
      dmi_resp_ready_i : in  std_ulogic; -- ready to receive respond
      dmi_resp_data_o  : out std_ulogic_vector(31 downto 0);
      dmi_resp_err_o   : out std_ulogic; -- 0=ok, 1=error
      -- CPU bus access --
      cpu_addr_i       : in  std_ulogic_vector(31 downto 0); -- address
      cpu_rden_i       : in  std_ulogic; -- read enable
      cpu_wren_i       : in  std_ulogic; -- write enable
      cpu_data_i       : in  std_ulogic_vector(31 downto 0); -- data in
      cpu_data_o       : out std_ulogic_vector(31 downto 0); -- data out
      cpu_ack_o        : out std_ulogic; -- transfer acknowledge
      -- CPU control --
      cpu_ndmrstn_o    : out std_ulogic; -- soc reset
      cpu_halt_req_o   : out std_ulogic  -- request hart to halt (enter debug mode)
    );
  end component;

  -- Component: On-Chip Debugger - Debug Transport Module (DTM) -----------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_debug_dtm
    generic (
      IDCODE_VERSION : std_ulogic_vector(03 downto 0); -- version
      IDCODE_PARTID  : std_ulogic_vector(15 downto 0); -- part number
      IDCODE_MANID   : std_ulogic_vector(10 downto 0)  -- manufacturer id
    );
    port (
      -- global control --
      clk_i            : in  std_ulogic; -- global clock line
      rstn_i           : in  std_ulogic; -- global reset line, low-active
      -- jtag connection --
      jtag_trst_i      : in  std_ulogic;
      jtag_tck_i       : in  std_ulogic;
      jtag_tdi_i       : in  std_ulogic;
      jtag_tdo_o       : out std_ulogic;
      jtag_tms_i       : in  std_ulogic;
      -- debug module interface (DMI) --
      dmi_rstn_o       : out std_ulogic;
      dmi_req_valid_o  : out std_ulogic;
      dmi_req_ready_i  : in  std_ulogic; -- DMI is allowed to make new requests when set
      dmi_req_addr_o   : out std_ulogic_vector(06 downto 0);
      dmi_req_op_o     : out std_ulogic; -- 0=read, 1=write
      dmi_req_data_o   : out std_ulogic_vector(31 downto 0);
      dmi_resp_valid_i : in  std_ulogic; -- response valid when set
      dmi_resp_ready_o : out std_ulogic; -- ready to receive respond
      dmi_resp_data_i  : in  std_ulogic_vector(31 downto 0);
      dmi_resp_err_i   : in  std_ulogic -- 0=ok, 1=error
    );
  end component;

end neorv32_package;

package body neorv32_package is

  -- Function: Minimal required number of bits to represent <input> numbers -----------------
  -- -------------------------------------------------------------------------------------------
  function index_size_f(input : natural) return natural is
  begin
    for i in 0 to natural'high loop
      if (2**i >= input) then
        return i;
      end if;
    end loop; -- i
    return 0;
  end function index_size_f;

  -- Function: Conditional select natural ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function cond_sel_natural_f(cond : boolean; val_t : natural; val_f : natural) return natural is
  begin
    if (cond = true) then
      return val_t;
    else
      return val_f;
    end if;
  end function cond_sel_natural_f;

  -- Function: Conditional select integer ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function cond_sel_int_f(cond : boolean; val_t : integer; val_f : integer) return integer is
  begin
    if (cond = true) then
      return val_t;
    else
      return val_f;
    end if;
  end function cond_sel_int_f;

  -- Function: Conditional select std_ulogic_vector -----------------------------------------
  -- -------------------------------------------------------------------------------------------
  function cond_sel_stdulogicvector_f(cond : boolean; val_t : std_ulogic_vector; val_f : std_ulogic_vector) return std_ulogic_vector is
  begin
    if (cond = true) then
      return val_t;
    else
      return val_f;
    end if;
  end function cond_sel_stdulogicvector_f;

  -- Function: Conditional select std_ulogic ------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function cond_sel_stdulogic_f(cond : boolean; val_t : std_ulogic; val_f : std_ulogic) return std_ulogic is
  begin
    if (cond = true) then
      return val_t;
    else
      return val_f;
    end if;
  end function cond_sel_stdulogic_f;

  -- Function: Conditional select string ----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function cond_sel_string_f(cond : boolean; val_t : string; val_f : string) return string is
  begin
    if (cond = true) then
      return val_t;
    else
      return val_f;
    end if;
  end function cond_sel_string_f;

  -- Function: Convert bool to std_ulogic ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function bool_to_ulogic_f(cond : boolean) return std_ulogic is
  begin
    if (cond = true) then
      return '1';
    else
      return '0';
    end if;
  end function bool_to_ulogic_f;

  -- Function: OR-reduce all bits -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function or_reduce_f(a : std_ulogic_vector) return std_ulogic is
    variable tmp_v : std_ulogic;
  begin
    tmp_v := '0';
    for i in a'range loop
      tmp_v := tmp_v or a(i);
    end loop; -- i
    return tmp_v;
  end function or_reduce_f;

  -- Function: AND-reduce all bits ----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function and_reduce_f(a : std_ulogic_vector) return std_ulogic is
    variable tmp_v : std_ulogic;
  begin
    tmp_v := '1';
    for i in a'range loop
      tmp_v := tmp_v and a(i);
    end loop; -- i
    return tmp_v;
  end function and_reduce_f;

  -- Function: XOR-reduce all bits ----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function xor_reduce_f(a : std_ulogic_vector) return std_ulogic is
    variable tmp_v : std_ulogic;
  begin
    tmp_v := '0';
    for i in a'range loop
      tmp_v := tmp_v xor a(i);
    end loop; -- i
    return tmp_v;
  end function xor_reduce_f;

  -- Function: Convert std_ulogic_vector to hex char ----------------------------------------
  -- -------------------------------------------------------------------------------------------
  function to_hexchar_f(input : std_ulogic_vector(3 downto 0)) return character is
    variable output_v : character;
  begin
    case input is
      when x"0"   => output_v := '0';
      when x"1"   => output_v := '1';
      when x"2"   => output_v := '2';
      when x"3"   => output_v := '3';
      when x"4"   => output_v := '4';
      when x"5"   => output_v := '5';
      when x"6"   => output_v := '6';
      when x"7"   => output_v := '7';
      when x"8"   => output_v := '8';
      when x"9"   => output_v := '9';
      when x"a"   => output_v := 'a';
      when x"b"   => output_v := 'b';
      when x"c"   => output_v := 'c';
      when x"d"   => output_v := 'd';
      when x"e"   => output_v := 'e';
      when x"f"   => output_v := 'f';
      when others => output_v := '?';
    end case;
    return output_v;
  end function to_hexchar_f;

  -- Function: Convert hex char to std_ulogic_vector ----------------------------------------
  -- -------------------------------------------------------------------------------------------
  function hexchar_to_stdulogicvector_f(input : character) return std_ulogic_vector is
    variable hex_value_v : std_ulogic_vector(3 downto 0);
  begin
    case input is
      when '0'       => hex_value_v := x"0";
      when '1'       => hex_value_v := x"1";
      when '2'       => hex_value_v := x"2";
      when '3'       => hex_value_v := x"3"; 
      when '4'       => hex_value_v := x"4";
      when '5'       => hex_value_v := x"5";
      when '6'       => hex_value_v := x"6";
      when '7'       => hex_value_v := x"7";
      when '8'       => hex_value_v := x"8";
      when '9'       => hex_value_v := x"9";
      when 'a' | 'A' => hex_value_v := x"a";
      when 'b' | 'B' => hex_value_v := x"b";
      when 'c' | 'C' => hex_value_v := x"c";
      when 'd' | 'D' => hex_value_v := x"d";
      when 'e' | 'E' => hex_value_v := x"e";
      when 'f' | 'F' => hex_value_v := x"f";
      when others    => hex_value_v := (others => 'X');
    end case;
    return hex_value_v;
  end function hexchar_to_stdulogicvector_f;

  -- Function: Bit reversal -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function bit_rev_f(input : std_ulogic_vector) return std_ulogic_vector is
    variable output_v : std_ulogic_vector(input'range);
  begin
    for i in 0 to input'length-1 loop
      output_v(input'length-i-1) := input(i);
    end loop; -- i
    return output_v;
  end function bit_rev_f;

  -- Function: Test if input number is a power of two ---------------------------------------
  -- -------------------------------------------------------------------------------------------
  function is_power_of_two_f(input : natural) return boolean is
  begin
    if (input = 1) then -- 2^0
      return true;
    elsif ((input / 2) /= 0) and ((input mod 2) = 0) then
      return true;
    else
      return false;
    end if;
  end function is_power_of_two_f;

  -- Function: Swap all bytes of a 32-bit word (endianness conversion) ----------------------
  -- -------------------------------------------------------------------------------------------
  function bswap32_f(input : std_ulogic_vector) return std_ulogic_vector is
    variable output_v : std_ulogic_vector(input'range);
  begin
    output_v(07 downto 00) := input(31 downto 24);
    output_v(15 downto 08) := input(23 downto 16);
    output_v(23 downto 16) := input(15 downto 08);
    output_v(31 downto 24) := input(07 downto 00);
    return output_v;
  end function bswap32_f;

  -- Function: Convert char to lowercase ----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function char_to_lower_f(ch : character) return character is
    variable res: character;
   begin
     case ch is
       when 'A'    => res := 'a';
       when 'B'    => res := 'b';
       when 'C'    => res := 'c';
       when 'D'    => res := 'd';
       when 'E'    => res := 'e';
       when 'F'    => res := 'f';
       when 'G'    => res := 'g';
       when 'H'    => res := 'h';
       when 'I'    => res := 'i';
       when 'J'    => res := 'j';
       when 'K'    => res := 'k';
       when 'L'    => res := 'l';
       when 'M'    => res := 'm';
       when 'N'    => res := 'n';
       when 'O'    => res := 'o';
       when 'P'    => res := 'p';
       when 'Q'    => res := 'q';
       when 'R'    => res := 'r';
       when 'S'    => res := 's';
       when 'T'    => res := 't';
       when 'U'    => res := 'u';
       when 'V'    => res := 'v';
       when 'W'    => res := 'w';
       when 'X'    => res := 'x';
       when 'Y'    => res := 'y';
       when 'Z'    => res := 'z';
       when others => res := ch;
      end case;
    return res;
  end function char_to_lower_f;

  -- Function: Compare strings (convert to lower case, check lengths) -----------------------
  -- -------------------------------------------------------------------------------------------
  function str_equal_f(str0 : string; str1 : string) return boolean is
    variable tmp0_v : string(str0'range);
    variable tmp1_v : string(str1'range);
  begin
    if (str0'length /= str1'length) then -- equal length?
      return false;
    else
      -- convert to lower case --
      for i in str0'range loop
        tmp0_v(i) := char_to_lower_f(str0(i));
      end loop;
      for i in str1'range loop
        tmp1_v(i) := char_to_lower_f(str1(i));
      end loop;
      -- compare lowercase strings --
      if (tmp0_v = tmp1_v) then
        return true;
      else
        return false;
      end if;
    end if;
  end function str_equal_f;

  -- Function: Population count (number of set bits) ----------------------------------------
  -- -------------------------------------------------------------------------------------------
  function popcount_f(input : std_ulogic_vector) return natural is
    variable cnt_v : natural range 0 to input'length;
  begin
    cnt_v := 0;
    for i in input'length-1 downto 0 loop
      if (input(i) = '1') then
        cnt_v := cnt_v + 1;
      end if;
    end loop; -- i
    return cnt_v;
  end function popcount_f;

  -- Function: Count leading zeros ----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function leading_zeros_f(input : std_ulogic_vector) return natural is
    variable cnt_v : natural range 0 to input'length;
  begin
    cnt_v := 0;
    for i in input'length-1 downto 0 loop
      if (input(i) = '0') then
        cnt_v := cnt_v + 1;
      else
        exit;
      end if;
    end loop; -- i
    return cnt_v;
  end function leading_zeros_f;

  -- Function: Initialize mem32_t array from another mem32_t array --------------------------
  -- -------------------------------------------------------------------------------------------
  -- impure function: returns NOT the same result every time it is evaluated with the same arguments since the source file might have changed
  impure function mem32_init_f(init : mem32_t; depth : natural) return mem32_t is
    variable mem_v : mem32_t(0 to depth-1);
  begin
    mem_v := (others => (others => '0')); -- make sure remaining memory entries are set to zero
    if (init'length > depth) then
      return mem_v;
    end if;
    for idx_v in 0 to init'length-1 loop -- init only in range of source data array
      mem_v(idx_v) := init(idx_v);
    end loop; -- idx_v
    return mem_v;
  end function mem32_init_f;


end neorv32_package;
