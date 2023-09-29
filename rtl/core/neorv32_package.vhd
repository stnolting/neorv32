-- #################################################################################################
-- # << NEORV32 - Main VHDL Package File (CPU and SoC) >>                                          #
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

package neorv32_package is

-- ****************************************************************************************************************************
-- Architecture Configuration and Constants
-- ****************************************************************************************************************************

  -- Architecture Configuration -------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- if register x0 is implemented as a *physical register* it has to be explicitly set to zero by the CPU hardware --
  constant reset_x0_c : boolean := true; -- has to be 'true' for the default register file rtl description (BRAM-based)

  -- max response time for processor-internal bus transactions --
  -- = cycles after which an *unacknowledged* internal bus access will timeout triggering a bus fault exception
  constant bus_timeout_c : natural := 15; -- default = 15

  -- instruction prefetch buffer depth --
  constant ipb_depth_c : natural := 2; -- hast to be a power of two, min 2, default 2

  -- instruction monitor: raise exception if multi-cycle operation times out --
  constant monitor_mc_tmo_c : natural := 9; -- = log2 of max execution cycles (default = 512 cycles)

  -- Architecture Constants -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant hw_version_c : std_ulogic_vector(31 downto 0) := x"01080906"; -- hardware version
  constant archid_c     : natural := 19; -- official RISC-V architecture ID
  constant XLEN         : natural := 32; -- native data path width, do not change!

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

-- ****************************************************************************************************************************
-- Processor Address Space Layout
-- ****************************************************************************************************************************

  -- Main Address Regions ---
  constant mem_imem_base_c : std_ulogic_vector(31 downto 0) := x"00000000"; -- IMEM size via generic
  constant mem_dmem_base_c : std_ulogic_vector(31 downto 0) := x"80000000"; -- DMEM size via generic
  constant mem_xip_base_c  : std_ulogic_vector(31 downto 0) := x"e0000000"; -- page (4MSBs) only!
  constant mem_xip_size_c  : natural := 256*1024*1024;
  constant mem_boot_base_c : std_ulogic_vector(31 downto 0) := x"ffffc000";
  constant mem_boot_size_c : natural := 8*1024;
  constant mem_io_base_c   : std_ulogic_vector(31 downto 0) := x"ffffe000";
  constant mem_io_size_c   : natural := 8*1024;

  -- Start of uncached memory access (page / 4MSBs only) --
  constant uncached_begin_c  : std_ulogic_vector(31 downto 0) := x"f0000000";

  -- IO Address Map --
  constant iodev_size_c      : natural := 256; -- size of a single IO device (bytes)
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe000"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe100"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe200"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe300"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe400"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe500"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe600"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe700"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe800"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe900"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffea00"; -- reserved
  constant base_io_cfs_c     : std_ulogic_vector(31 downto 0) := x"ffffeb00";
  constant base_io_slink_c   : std_ulogic_vector(31 downto 0) := x"ffffec00";
  constant base_io_dma_c     : std_ulogic_vector(31 downto 0) := x"ffffed00";
  constant base_io_crc_c     : std_ulogic_vector(31 downto 0) := x"ffffee00";
  constant base_io_xip_c     : std_ulogic_vector(31 downto 0) := x"ffffef00";
  constant base_io_pwm_c     : std_ulogic_vector(31 downto 0) := x"fffff000";
  constant base_io_gptmr_c   : std_ulogic_vector(31 downto 0) := x"fffff100";
  constant base_io_onewire_c : std_ulogic_vector(31 downto 0) := x"fffff200";
  constant base_io_xirq_c    : std_ulogic_vector(31 downto 0) := x"fffff300";
  constant base_io_mtime_c   : std_ulogic_vector(31 downto 0) := x"fffff400";
  constant base_io_uart0_c   : std_ulogic_vector(31 downto 0) := x"fffff500";
  constant base_io_uart1_c   : std_ulogic_vector(31 downto 0) := x"fffff600";
  constant base_io_sdi_c     : std_ulogic_vector(31 downto 0) := x"fffff700";
  constant base_io_spi_c     : std_ulogic_vector(31 downto 0) := x"fffff800";
  constant base_io_twi_c     : std_ulogic_vector(31 downto 0) := x"fffff900";
  constant base_io_trng_c    : std_ulogic_vector(31 downto 0) := x"fffffa00";
  constant base_io_wdt_c     : std_ulogic_vector(31 downto 0) := x"fffffb00";
  constant base_io_gpio_c    : std_ulogic_vector(31 downto 0) := x"fffffc00";
  constant base_io_neoled_c  : std_ulogic_vector(31 downto 0) := x"fffffd00";
  constant base_io_sysinfo_c : std_ulogic_vector(31 downto 0) := x"fffffe00";
  constant base_io_dm_c      : std_ulogic_vector(31 downto 0) := x"ffffff00";

  -- On-Chip Debugger - Debug Module Entry Points (Code ROM) --
  constant dm_exc_entry_c  : std_ulogic_vector(31 downto 0) := x"ffffff00"; -- = base_io_dm_c + 0, exceptions entry point
  constant dm_park_entry_c : std_ulogic_vector(31 downto 0) := x"ffffff08"; -- = base_io_dm_c + 8, normal entry point

-- ****************************************************************************************************************************
-- SoC Definitions
-- ****************************************************************************************************************************

  -- SoC Clock Select -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant clk_div2_c    : natural := 0;
  constant clk_div4_c    : natural := 1;
  constant clk_div8_c    : natural := 2;
  constant clk_div64_c   : natural := 3;
  constant clk_div128_c  : natural := 4;
  constant clk_div1024_c : natural := 5;
  constant clk_div2048_c : natural := 6;
  constant clk_div4096_c : natural := 7;

  -- Internal Memory Types ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  type mem32_t is array (natural range <>) of std_ulogic_vector(31 downto 0); -- memory with 32-bit entries
  type mem8_t  is array (natural range <>) of std_ulogic_vector(07 downto 0); -- memory with 8-bit entries

  -- Internal Bus Interface -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- bus request --
  type bus_req_t is record
    addr : std_ulogic_vector(31 downto 0); -- access address
    data : std_ulogic_vector(31 downto 0); -- write data
    ben  : std_ulogic_vector(03 downto 0); -- byte enable
    we   : std_ulogic; -- write request (single-shot)
    re   : std_ulogic; -- read request (single-shot)
    src  : std_ulogic; -- access source (1=instruction fetch, 0=data access)
    priv : std_ulogic; -- set if privileged (machine-mode) access
    rvso : std_ulogic; -- set if reservation set operation (atomic LR/SC)
  end record;

  -- bus response --
  type bus_rsp_t is record
    data : std_ulogic_vector(31 downto 0); -- read data
    ack  : std_ulogic; -- access acknowledge (single-shot)
    err  : std_ulogic; -- access error (single-shot)
  end record;

  -- source (request) termination --
  constant req_terminate_c : bus_req_t := (
    addr => (others => '0'),
    data => (others => '0'),
    ben  => (others => '0'),
    we   => '0',
    re   => '0',
    src  => '0',
    priv => '0',
    rvso => '0'
  );

  -- endpoint (response) termination --
  constant rsp_terminate_c : bus_rsp_t := (
    data => (others => '0'),
    ack  => '0',
    err  => '0'
  );

  -- Debug Module Interface -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- request --
  type dmi_req_t is record
    addr : std_ulogic_vector(06 downto 0);
    op   : std_ulogic_vector(01 downto 0);
    data : std_ulogic_vector(31 downto 0);
  end record;

  -- request operation --
  constant dmi_req_nop_c : std_ulogic_vector(1 downto 0) := "00"; -- no operation
  constant dmi_req_rd_c  : std_ulogic_vector(1 downto 0) := "01"; -- read access
  constant dmi_req_wr_c  : std_ulogic_vector(1 downto 0) := "10"; -- write access

  -- response --
  type dmi_rsp_t is record
    data : std_ulogic_vector(31 downto 0);
    ack  : std_ulogic;
  end record;

-- ****************************************************************************************************************************
-- RISC-V ISA Definitions
-- ****************************************************************************************************************************

  -- RISC-V 32-Bit Instruction Word Layout --------------------------------------------------
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
  constant instr_rs3_lsb_c     : natural := 27; -- source register 3 address bit 0
  constant instr_rs3_msb_c     : natural := 31; -- source register 3 address bit 4
  constant instr_funct7_lsb_c  : natural := 25; -- funct7 bit 0
  constant instr_funct7_msb_c  : natural := 31; -- funct7 bit 6
  constant instr_funct12_lsb_c : natural := 20; -- funct12 bit 0
  constant instr_funct12_msb_c : natural := 31; -- funct12 bit 11
  constant instr_imm12_lsb_c   : natural := 20; -- immediate12 bit 0
  constant instr_imm12_msb_c   : natural := 31; -- immediate12 bit 11
  constant instr_imm20_lsb_c   : natural := 12; -- immediate20 bit 0
  constant instr_imm20_msb_c   : natural := 31; -- immediate20 bit 21
  constant instr_funct5_lsb_c  : natural := 27; -- funct5 select bit 0
  constant instr_funct5_msb_c  : natural := 31; -- funct5 select bit 4

  -- RISC-V Opcodes -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- alu --
  constant opcode_alui_c   : std_ulogic_vector(6 downto 0) := "0010011"; -- ALU operation with immediate
  constant opcode_alu_c    : std_ulogic_vector(6 downto 0) := "0110011"; -- ALU operation
  constant opcode_lui_c    : std_ulogic_vector(6 downto 0) := "0110111"; -- load upper immediate
  constant opcode_auipc_c  : std_ulogic_vector(6 downto 0) := "0010111"; -- add upper immediate to PC
  -- control flow --
  constant opcode_jal_c    : std_ulogic_vector(6 downto 0) := "1101111"; -- jump and link
  constant opcode_jalr_c   : std_ulogic_vector(6 downto 0) := "1100111"; -- jump and link with register
  constant opcode_branch_c : std_ulogic_vector(6 downto 0) := "1100011"; -- branch
  -- memory access --
  constant opcode_load_c   : std_ulogic_vector(6 downto 0) := "0000011"; -- load
  constant opcode_store_c  : std_ulogic_vector(6 downto 0) := "0100011"; -- store
  constant opcode_amo_c    : std_ulogic_vector(6 downto 0) := "0101111"; -- atomic memory access
  constant opcode_fence_c  : std_ulogic_vector(6 downto 0) := "0001111"; -- fence / fence.i
  -- system/csr --
  constant opcode_system_c : std_ulogic_vector(6 downto 0) := "1110011"; -- system/csr access
  -- floating point operations --
  constant opcode_fop_c    : std_ulogic_vector(6 downto 0) := "1010011"; -- dual/single operand instruction
  -- official custom RISC-V opcodes - free for custom instructions --
  constant opcode_cust0_c  : std_ulogic_vector(6 downto 0) := "0001011"; -- custom-0
  constant opcode_cust1_c  : std_ulogic_vector(6 downto 0) := "0101011"; -- custom-1
  constant opcode_cust2_c  : std_ulogic_vector(6 downto 0) := "1011011"; -- custom-2
  constant opcode_cust3_c  : std_ulogic_vector(6 downto 0) := "1111011"; -- custom-3

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
  constant funct3_lb_c     : std_ulogic_vector(2 downto 0) := "000"; -- load byte (signed)
  constant funct3_lh_c     : std_ulogic_vector(2 downto 0) := "001"; -- load half word (signed)
  constant funct3_lw_c     : std_ulogic_vector(2 downto 0) := "010"; -- load word (signed)
  constant funct3_lbu_c    : std_ulogic_vector(2 downto 0) := "100"; -- load byte (unsigned)
  constant funct3_lhu_c    : std_ulogic_vector(2 downto 0) := "101"; -- load half word (unsigned)
  constant funct3_lwu_c    : std_ulogic_vector(2 downto 0) := "110"; -- load word (unsigned)
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
  constant funct3_csrrw_c  : std_ulogic_vector(2 downto 0) := "001"; -- csr r/w
  constant funct3_csrrs_c  : std_ulogic_vector(2 downto 0) := "010"; -- csr read & set
  constant funct3_csrrc_c  : std_ulogic_vector(2 downto 0) := "011"; -- csr read & clear
  constant funct3_csril_c  : std_ulogic_vector(2 downto 0) := "100"; -- undefined/illegal csr command
  constant funct3_csrrwi_c : std_ulogic_vector(2 downto 0) := "101"; -- csr r/w immediate
  constant funct3_csrrsi_c : std_ulogic_vector(2 downto 0) := "110"; -- csr read & set immediate
  constant funct3_csrrci_c : std_ulogic_vector(2 downto 0) := "111"; -- csr read & clear immediate
  -- fence --
  constant funct3_fence_c  : std_ulogic_vector(2 downto 0) := "000"; -- fence - order IO/memory access
  constant funct3_fencei_c : std_ulogic_vector(2 downto 0) := "001"; -- fence.i - instruction stream sync

  -- RISC-V Funct12 - SYSTEM ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant funct12_ecall_c  : std_ulogic_vector(11 downto 0) := x"000"; -- ecall
  constant funct12_ebreak_c : std_ulogic_vector(11 downto 0) := x"001"; -- ebreak
  constant funct12_wfi_c    : std_ulogic_vector(11 downto 0) := x"105"; -- wfi
  constant funct12_mret_c   : std_ulogic_vector(11 downto 0) := x"302"; -- mret
  constant funct12_dret_c   : std_ulogic_vector(11 downto 0) := x"7b2"; -- dret

  -- RISC-V Floating-Point Stuff ------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant float_single_c : std_ulogic_vector(1 downto 0) := "00"; -- single-precision (32-bit)
--constant float_double_c : std_ulogic_vector(1 downto 0) := "01"; -- double-precision (64-bit)
--constant float_half_c   : std_ulogic_vector(1 downto 0) := "10"; -- half-precision (16-bit)
--constant float_quad_c   : std_ulogic_vector(1 downto 0) := "11"; -- quad-precision (128-bit)

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

  -- RISC-V CSRs ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- user floating-point CSRs --
  constant csr_fflags_c         : std_ulogic_vector(11 downto 0) := x"001";
  constant csr_frm_c            : std_ulogic_vector(11 downto 0) := x"002";
  constant csr_fcsr_c           : std_ulogic_vector(11 downto 0) := x"003";
  -- machine trap setup --
  constant csr_mstatus_c        : std_ulogic_vector(11 downto 0) := x"300";
  constant csr_misa_c           : std_ulogic_vector(11 downto 0) := x"301";
  constant csr_mie_c            : std_ulogic_vector(11 downto 0) := x"304";
  constant csr_mtvec_c          : std_ulogic_vector(11 downto 0) := x"305";
  constant csr_mcounteren_c     : std_ulogic_vector(11 downto 0) := x"306";
  constant csr_mstatush_c       : std_ulogic_vector(11 downto 0) := x"310";
  -- machine counter setup --
  constant csr_mcountinhibit_c  : std_ulogic_vector(11 downto 0) := x"320";
  constant csr_mcyclecfg_c      : std_ulogic_vector(11 downto 0) := x"321";
  constant csr_minstretcfg_c    : std_ulogic_vector(11 downto 0) := x"322";
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
  -- machine trap handling --
  constant csr_mscratch_c       : std_ulogic_vector(11 downto 0) := x"340";
  constant csr_mepc_c           : std_ulogic_vector(11 downto 0) := x"341";
  constant csr_mcause_c         : std_ulogic_vector(11 downto 0) := x"342";
  constant csr_mtval_c          : std_ulogic_vector(11 downto 0) := x"343";
  constant csr_mip_c            : std_ulogic_vector(11 downto 0) := x"344";
  constant csr_mtinst_c         : std_ulogic_vector(11 downto 0) := x"34a";
  -- physical memory protection - configuration --
  constant csr_pmpcfg0_c        : std_ulogic_vector(11 downto 0) := x"3a0";
  constant csr_pmpcfg1_c        : std_ulogic_vector(11 downto 0) := x"3a1";
  constant csr_pmpcfg2_c        : std_ulogic_vector(11 downto 0) := x"3a2";
  constant csr_pmpcfg3_c        : std_ulogic_vector(11 downto 0) := x"3a3";
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
  -- machine counter setup - continued --
  constant csr_mcyclecfgh_c     : std_ulogic_vector(11 downto 0) := x"721";
  constant csr_minstretcfgh_c   : std_ulogic_vector(11 downto 0) := x"722";
  -- trigger module registers --
  constant csr_tselect_c        : std_ulogic_vector(11 downto 0) := x"7a0";
  constant csr_tdata1_c         : std_ulogic_vector(11 downto 0) := x"7a1";
  constant csr_tdata2_c         : std_ulogic_vector(11 downto 0) := x"7a2";
  constant csr_tdata3_c         : std_ulogic_vector(11 downto 0) := x"7a3";
  constant csr_tinfo_c          : std_ulogic_vector(11 downto 0) := x"7a4";
  constant csr_tcontrol_c       : std_ulogic_vector(11 downto 0) := x"7a5";
  -- debug mode registers --
  constant csr_dcsr_c           : std_ulogic_vector(11 downto 0) := x"7b0";
  constant csr_dpc_c            : std_ulogic_vector(11 downto 0) := x"7b1";
  constant csr_dscratch0_c      : std_ulogic_vector(11 downto 0) := x"7b2";
  -- NEORV32-specific (user-mode) registers --
  constant csr_cfureg0_c        : std_ulogic_vector(11 downto 0) := x"800";
  constant csr_cfureg1_c        : std_ulogic_vector(11 downto 0) := x"801";
  constant csr_cfureg2_c        : std_ulogic_vector(11 downto 0) := x"802";
  constant csr_cfureg3_c        : std_ulogic_vector(11 downto 0) := x"803";
  -- machine counters/timers --
  constant csr_mcycle_c         : std_ulogic_vector(11 downto 0) := x"b00";
--constant csr_mtime_c          : std_ulogic_vector(11 downto 0) := x"b01";
  constant csr_minstret_c       : std_ulogic_vector(11 downto 0) := x"b02";
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
  --
  constant csr_mcycleh_c        : std_ulogic_vector(11 downto 0) := x"b80";
--constant csr_mtimeh_c         : std_ulogic_vector(11 downto 0) := x"b81";
  constant csr_minstreth_c      : std_ulogic_vector(11 downto 0) := x"b82";
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
  -- user counters/timers --
  constant csr_cycle_c          : std_ulogic_vector(11 downto 0) := x"c00";
  constant csr_time_c           : std_ulogic_vector(11 downto 0) := x"c01";
  constant csr_instret_c        : std_ulogic_vector(11 downto 0) := x"c02";
  constant csr_hpmcounter3_c    : std_ulogic_vector(11 downto 0) := x"c03";
  constant csr_hpmcounter4_c    : std_ulogic_vector(11 downto 0) := x"c04";
  constant csr_hpmcounter5_c    : std_ulogic_vector(11 downto 0) := x"c05";
  constant csr_hpmcounter6_c    : std_ulogic_vector(11 downto 0) := x"c06";
  constant csr_hpmcounter7_c    : std_ulogic_vector(11 downto 0) := x"c07";
  constant csr_hpmcounter8_c    : std_ulogic_vector(11 downto 0) := x"c08";
  constant csr_hpmcounter9_c    : std_ulogic_vector(11 downto 0) := x"c09";
  constant csr_hpmcounter10_c   : std_ulogic_vector(11 downto 0) := x"c0a";
  constant csr_hpmcounter11_c   : std_ulogic_vector(11 downto 0) := x"c0b";
  constant csr_hpmcounter12_c   : std_ulogic_vector(11 downto 0) := x"c0c";
  constant csr_hpmcounter13_c   : std_ulogic_vector(11 downto 0) := x"c0d";
  constant csr_hpmcounter14_c   : std_ulogic_vector(11 downto 0) := x"c0e";
  constant csr_hpmcounter15_c   : std_ulogic_vector(11 downto 0) := x"c0f";
  --
  constant csr_cycleh_c         : std_ulogic_vector(11 downto 0) := x"c80";
  constant csr_timeh_c          : std_ulogic_vector(11 downto 0) := x"c81";
  constant csr_instreth_c       : std_ulogic_vector(11 downto 0) := x"c82";
  constant csr_hpmcounter3h_c   : std_ulogic_vector(11 downto 0) := x"c83";
  constant csr_hpmcounter4h_c   : std_ulogic_vector(11 downto 0) := x"c84";
  constant csr_hpmcounter5h_c   : std_ulogic_vector(11 downto 0) := x"c85";
  constant csr_hpmcounter6h_c   : std_ulogic_vector(11 downto 0) := x"c86";
  constant csr_hpmcounter7h_c   : std_ulogic_vector(11 downto 0) := x"c87";
  constant csr_hpmcounter8h_c   : std_ulogic_vector(11 downto 0) := x"c88";
  constant csr_hpmcounter9h_c   : std_ulogic_vector(11 downto 0) := x"c89";
  constant csr_hpmcounter10h_c  : std_ulogic_vector(11 downto 0) := x"c8a";
  constant csr_hpmcounter11h_c  : std_ulogic_vector(11 downto 0) := x"c8b";
  constant csr_hpmcounter12h_c  : std_ulogic_vector(11 downto 0) := x"c8c";
  constant csr_hpmcounter13h_c  : std_ulogic_vector(11 downto 0) := x"c8d";
  constant csr_hpmcounter14h_c  : std_ulogic_vector(11 downto 0) := x"c8e";
  constant csr_hpmcounter15h_c  : std_ulogic_vector(11 downto 0) := x"c8f";
  -- machine information registers --
  constant csr_mvendorid_c      : std_ulogic_vector(11 downto 0) := x"f11";
  constant csr_marchid_c        : std_ulogic_vector(11 downto 0) := x"f12";
  constant csr_mimpid_c         : std_ulogic_vector(11 downto 0) := x"f13";
  constant csr_mhartid_c        : std_ulogic_vector(11 downto 0) := x"f14";
  constant csr_mconfigptr_c     : std_ulogic_vector(11 downto 0) := x"f15";
  -- NEORV32-specific (machine-mode) registers --
  constant csr_mxisa_c          : std_ulogic_vector(11 downto 0) := x"fc0";

-- ****************************************************************************************************************************
-- CPU Control
-- ****************************************************************************************************************************

  -- Main CPU Control Bus -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  type ctrl_bus_t is record
    -- register file --
    rf_wb_en     : std_ulogic; -- write back enable
    rf_rs1       : std_ulogic_vector(04 downto 0); -- source register 1 address
    rf_rs2       : std_ulogic_vector(04 downto 0); -- source register 2 address
    rf_rs3       : std_ulogic_vector(04 downto 0); -- source register 3 address
    rf_rd        : std_ulogic_vector(04 downto 0); -- destination register address
    rf_mux       : std_ulogic_vector(01 downto 0); -- input source select
    rf_zero_we   : std_ulogic;                     -- allow/force write access to x0
    -- alu --
    alu_op       : std_ulogic_vector(02 downto 0); -- ALU operation select
    alu_opa_mux  : std_ulogic;                     -- operand A select (0=rs1, 1=PC)
    alu_opb_mux  : std_ulogic;                     -- operand B select (0=rs2, 1=IMM)
    alu_unsigned : std_ulogic;                     -- is unsigned ALU operation
    alu_cp_trig  : std_ulogic_vector(04 downto 0); -- co-processor trigger (one-hot)
    -- load/store unit --
    lsu_req_rd   : std_ulogic;                     -- trigger memory read request
    lsu_req_wr   : std_ulogic;                     -- trigger memory write request
    lsu_rw       : std_ulogic;                     -- 0: read access, 1: write access
    lsu_mo_we    : std_ulogic;                     -- memory address and data output register write enable
    lsu_fence    : std_ulogic;                     -- fence operation
    lsu_fencei   : std_ulogic;                     -- fence.i operation
    lsu_priv     : std_ulogic;                     -- effective privilege level for load/store
    -- instruction word --
    ir_funct3    : std_ulogic_vector(02 downto 0); -- funct3 bit field
    ir_funct12   : std_ulogic_vector(11 downto 0); -- funct12 bit field
    ir_opcode    : std_ulogic_vector(06 downto 0); -- opcode bit field
    -- cpu status --
    cpu_priv     : std_ulogic;                     -- effective privilege mode
    cpu_sleep    : std_ulogic;                     -- set when CPU is in sleep mode
    cpu_trap     : std_ulogic;                     -- set when CPU is entering trap exec
    cpu_debug    : std_ulogic;                     -- set when CPU is in debug mode
  end record;

  -- control bus reset initializer --
  constant ctrl_bus_zero_c : ctrl_bus_t := (
    rf_wb_en     => '0',
    rf_rs1       => (others => '0'),
    rf_rs2       => (others => '0'),
    rf_rs3       => (others => '0'),
    rf_rd        => (others => '0'),
    rf_mux       => (others => '0'),
    rf_zero_we   => '0',
    alu_op       => (others => '0'),
    alu_opa_mux  => '0',
    alu_opb_mux  => '0',
    alu_unsigned => '0',
    alu_cp_trig  => (others => '0'),
    lsu_req_rd   => '0',
    lsu_req_wr   => '0',
    lsu_rw       => '0',
    lsu_mo_we    => '0',
    lsu_fence    => '0',
    lsu_fencei   => '0',
    lsu_priv     => '0',
    ir_funct3    => (others => '0'),
    ir_funct12   => (others => '0'),
    ir_opcode    => (others => '0'),
    cpu_priv     => '0',
    cpu_sleep    => '0',
    cpu_trap     => '0',
    cpu_debug    => '0'
  );

  -- Comparator Bus -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant cmp_equal_c : natural := 0;
  constant cmp_less_c  : natural := 1; -- for signed and unsigned comparisons

  -- CPU Co-Processor IDs -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant cp_sel_shifter_c  : natural := 0; -- CP0: shift operations (base ISA)
  constant cp_sel_muldiv_c   : natural := 1; -- CP1: multiplication/division operations ('M' extensions)
  constant cp_sel_bitmanip_c : natural := 2; -- CP2: bit manipulation ('B' extensions)
  constant cp_sel_fpu_c      : natural := 3; -- CP3: floating-point unit ('Zfinx' extension)
  constant cp_sel_cfu_c      : natural := 4; -- CP4: custom instructions CFU ('Zxcfu' extension)

  -- ALU Function Codes [DO NOT CHANGE ENCODING!] -------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant alu_op_add_c  : std_ulogic_vector(2 downto 0) := "000"; -- result <= A + B
  constant alu_op_sub_c  : std_ulogic_vector(2 downto 0) := "001"; -- result <= A - B
  constant alu_op_cp_c   : std_ulogic_vector(2 downto 0) := "010"; -- result <= ALU co-processor
  constant alu_op_slt_c  : std_ulogic_vector(2 downto 0) := "011"; -- result <= A < B
  constant alu_op_movb_c : std_ulogic_vector(2 downto 0) := "100"; -- result <= B
  constant alu_op_xor_c  : std_ulogic_vector(2 downto 0) := "101"; -- result <= A xor B
  constant alu_op_or_c   : std_ulogic_vector(2 downto 0) := "110"; -- result <= A or B
  constant alu_op_and_c  : std_ulogic_vector(2 downto 0) := "111"; -- result <= A and B

  -- Register File Input Select -------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant rf_mux_alu_c : std_ulogic_vector(1 downto 0) := "00"; -- register file <= alu result
  constant rf_mux_mem_c : std_ulogic_vector(1 downto 0) := "01"; -- register file <= memory read data
  constant rf_mux_csr_c : std_ulogic_vector(1 downto 0) := "10"; -- register file <= CSR read data
  constant rf_mux_npc_c : std_ulogic_vector(1 downto 0) := "11"; -- register file <= next-PC (for branch-and-link)

  -- Trap ID Codes --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- MSB:   1 = interrupt, 0 = sync. exception
  -- MSB-1: 1 = entry to debug mode, 0 = normal trapping
  -- RISC-V compliant synchronous exceptions --
  constant trap_ima_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00000"; -- 0: instruction misaligned
  constant trap_iaf_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00001"; -- 1: instruction access fault
  constant trap_iil_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00010"; -- 2: illegal instruction
  constant trap_brk_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00011"; -- 3: breakpoint
  constant trap_lma_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00100"; -- 4: load address misaligned
  constant trap_laf_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00101"; -- 5: load access fault
  constant trap_sma_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00110"; -- 6: store address misaligned
  constant trap_saf_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00111"; -- 7: store access fault
  constant trap_env_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "010UU"; -- 8..11: environment call from u/s/h/m
  -- RISC-V compliant asynchronous exceptions (interrupts) --
  constant trap_msi_c      : std_ulogic_vector(6 downto 0) := "1" & "0" & "00011"; -- 3:  machine software interrupt
  constant trap_mti_c      : std_ulogic_vector(6 downto 0) := "1" & "0" & "00111"; -- 7:  machine timer interrupt
  constant trap_mei_c      : std_ulogic_vector(6 downto 0) := "1" & "0" & "01011"; -- 11: machine external interrupt
  -- NEORV32-specific (RISC-V custom) asynchronous exceptions (interrupts) --
  constant trap_firq0_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10000"; -- 16: fast interrupt 0
  constant trap_firq1_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10001"; -- 17: fast interrupt 1
  constant trap_firq2_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10010"; -- 18: fast interrupt 2
  constant trap_firq3_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10011"; -- 19: fast interrupt 3
  constant trap_firq4_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10100"; -- 20: fast interrupt 4
  constant trap_firq5_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10101"; -- 21: fast interrupt 5
  constant trap_firq6_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10110"; -- 22: fast interrupt 6
  constant trap_firq7_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10111"; -- 23: fast interrupt 7
  constant trap_firq8_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "11000"; -- 24: fast interrupt 8
  constant trap_firq9_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "11001"; -- 25: fast interrupt 9
  constant trap_firq10_c   : std_ulogic_vector(6 downto 0) := "1" & "0" & "11010"; -- 26: fast interrupt 10
  constant trap_firq11_c   : std_ulogic_vector(6 downto 0) := "1" & "0" & "11011"; -- 27: fast interrupt 11
  constant trap_firq12_c   : std_ulogic_vector(6 downto 0) := "1" & "0" & "11100"; -- 28: fast interrupt 12
  constant trap_firq13_c   : std_ulogic_vector(6 downto 0) := "1" & "0" & "11101"; -- 29: fast interrupt 13
  constant trap_firq14_c   : std_ulogic_vector(6 downto 0) := "1" & "0" & "11110"; -- 30: fast interrupt 14
  constant trap_firq15_c   : std_ulogic_vector(6 downto 0) := "1" & "0" & "11111"; -- 31: fast interrupt 15
  -- entering debug mode (sync./async. exceptions) --
  constant trap_db_break_c : std_ulogic_vector(6 downto 0) := "0" & "1" & "00001"; -- 1: break instruction (sync)
  constant trap_db_trig_c  : std_ulogic_vector(6 downto 0) := "0" & "1" & "00010"; -- 2: hardware trigger (sync)
  constant trap_db_halt_c  : std_ulogic_vector(6 downto 0) := "1" & "1" & "00011"; -- 3: external halt request (async)
  constant trap_db_step_c  : std_ulogic_vector(6 downto 0) := "1" & "1" & "00100"; -- 4: single-stepping (async)

  -- CPU Trap System ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- exception source bits --
  constant exc_iaccess_c  : natural :=  0; -- instruction access fault
  constant exc_illegal_c  : natural :=  1; -- illegal instruction
  constant exc_ialign_c   : natural :=  2; -- instruction address misaligned
  constant exc_ecall_c    : natural :=  3; -- environment call
  constant exc_ebreak_c   : natural :=  4; -- breakpoint
  constant exc_salign_c   : natural :=  5; -- store address misaligned
  constant exc_lalign_c   : natural :=  6; -- load address misaligned
  constant exc_saccess_c  : natural :=  7; -- store access fault
  constant exc_laccess_c  : natural :=  8; -- load access fault
  -- for debug mode only --
  constant exc_db_break_c : natural :=  9; -- enter debug mode via ebreak instruction ("sync EXCEPTION")
  constant exc_db_hw_c    : natural := 10; -- enter debug mode via hw trigger ("sync EXCEPTION")
  --
  constant exc_width_c    : natural := 11; -- length of this list in bits
  -- interrupt source bits --
  constant irq_msi_irq_c  : natural :=  0; -- machine software interrupt
  constant irq_mti_irq_c  : natural :=  1; -- machine timer interrupt
  constant irq_mei_irq_c  : natural :=  2; -- machine external interrupt
  constant irq_firq_0_c   : natural :=  3; -- fast interrupt channel 0
  constant irq_firq_1_c   : natural :=  4; -- fast interrupt channel 1
  constant irq_firq_2_c   : natural :=  5; -- fast interrupt channel 2
  constant irq_firq_3_c   : natural :=  6; -- fast interrupt channel 3
  constant irq_firq_4_c   : natural :=  7; -- fast interrupt channel 4
  constant irq_firq_5_c   : natural :=  8; -- fast interrupt channel 5
  constant irq_firq_6_c   : natural :=  9; -- fast interrupt channel 6
  constant irq_firq_7_c   : natural := 10; -- fast interrupt channel 7
  constant irq_firq_8_c   : natural := 11; -- fast interrupt channel 8
  constant irq_firq_9_c   : natural := 12; -- fast interrupt channel 9
  constant irq_firq_10_c  : natural := 13; -- fast interrupt channel 10
  constant irq_firq_11_c  : natural := 14; -- fast interrupt channel 11
  constant irq_firq_12_c  : natural := 15; -- fast interrupt channel 12
  constant irq_firq_13_c  : natural := 16; -- fast interrupt channel 13
  constant irq_firq_14_c  : natural := 17; -- fast interrupt channel 14
  constant irq_firq_15_c  : natural := 18; -- fast interrupt channel 15
  -- for debug mode only --
  constant irq_db_halt_c  : natural := 19; -- enter debug mode via external halt request ("async IRQ")
  constant irq_db_step_c  : natural := 20; -- enter debug mode via single-stepping ("async IRQ")
  --
  constant irq_width_c    : natural := 21; -- length of this list in bits

  -- CPU Privilege Modes --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant priv_mode_m_c : std_ulogic := '1'; -- machine mode
  constant priv_mode_u_c : std_ulogic := '0'; -- user mode

  -- HPM Event System -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant hpmcnt_event_cy_c      : natural := 0;  -- Active cycle
  constant hpmcnt_event_tm_c      : natural := 1;  -- Time (unused/reserved)
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

-- ****************************************************************************************************************************
-- Helper Functions
-- ****************************************************************************************************************************

  function index_size_f(input : natural) return natural;
  function cond_sel_int_f(cond : boolean; val_t : integer; val_f : integer) return integer;
  function cond_sel_natural_f(cond : boolean; val_t : natural; val_f : natural) return natural;
  function cond_sel_suv_f(cond : boolean; val_t : std_ulogic_vector; val_f : std_ulogic_vector) return std_ulogic_vector;
  function cond_sel_string_f(cond : boolean; val_t : string; val_f : string) return string;
  function bool_to_ulogic_f(cond : boolean) return std_ulogic;
  function bin_to_gray_f(input : std_ulogic_vector) return std_ulogic_vector;
  function gray_to_bin_f(input : std_ulogic_vector) return std_ulogic_vector;
  function or_reduce_f(input : std_ulogic_vector) return std_ulogic;
  function and_reduce_f(input : std_ulogic_vector) return std_ulogic;
  function xor_reduce_f(input : std_ulogic_vector) return std_ulogic;
  function su_undefined_f(input : std_ulogic) return boolean;
  function to_hexchar_f(input : std_ulogic_vector(3 downto 0)) return character;
  function to_hstring32_f(input : std_ulogic_vector(31 downto 0)) return string;
  function bit_rev_f(input : std_ulogic_vector) return std_ulogic_vector;
  function is_power_of_two_f(input : natural) return boolean;
  function bswap32_f(input : std_ulogic_vector) return std_ulogic_vector;
  function popcount_f(input : std_ulogic_vector) return natural;
  function leading_zeros_f(input : std_ulogic_vector) return natural;
  impure function mem32_init_f(init : mem32_t; depth : natural) return mem32_t;

-- ****************************************************************************************************************************
-- NEORV32 Processor Top Entity (component prototype)
-- ****************************************************************************************************************************

  component neorv32_top
    generic (
      -- General --
      CLOCK_FREQUENCY              : natural;
      HART_ID                      : std_ulogic_vector(31 downto 0) := x"00000000";
      VENDOR_ID                    : std_ulogic_vector(31 downto 0) := x"00000000";
      INT_BOOTLOADER_EN            : boolean := false;
      -- On-Chip Debugger (OCD) --
      ON_CHIP_DEBUGGER_EN          : boolean := false;
      DM_LEGACY_MODE               : boolean := false;
      -- RISC-V CPU Extensions --
      CPU_EXTENSION_RISCV_A        : boolean := false;
      CPU_EXTENSION_RISCV_B        : boolean := false;
      CPU_EXTENSION_RISCV_C        : boolean := false;
      CPU_EXTENSION_RISCV_E        : boolean := false;
      CPU_EXTENSION_RISCV_M        : boolean := false;
      CPU_EXTENSION_RISCV_U        : boolean := false;
      CPU_EXTENSION_RISCV_Zfinx    : boolean := false;
      CPU_EXTENSION_RISCV_Zicntr   : boolean := true;
      CPU_EXTENSION_RISCV_Zihpm    : boolean := false;
      CPU_EXTENSION_RISCV_Zifencei : boolean := false;
      CPU_EXTENSION_RISCV_Zmmul    : boolean := false;
      CPU_EXTENSION_RISCV_Zxcfu    : boolean := false;
      -- Tuning Options --
      FAST_MUL_EN                  : boolean := false;
      FAST_SHIFT_EN                : boolean := false;
      -- Physical Memory Protection (PMP) --
      PMP_NUM_REGIONS              : natural range 0 to 16 := 0;
      PMP_MIN_GRANULARITY          : natural := 4;
      -- Hardware Performance Monitors (HPM) --
      HPM_NUM_CNTS                 : natural range 0 to 13 := 0;
      HPM_CNT_WIDTH                : natural range 0 to 64 := 40;
      -- Atomic Memory Access - Reservation Set Granularity --
      AMO_RVS_GRANULARITY          : natural := 4;
      -- Internal Instruction memory (IMEM) --
      MEM_INT_IMEM_EN              : boolean := false;
      MEM_INT_IMEM_SIZE            : natural := 16*1024;
      -- Internal Data memory (DMEM) --
      MEM_INT_DMEM_EN              : boolean := false;
      MEM_INT_DMEM_SIZE            : natural := 8*1024;
      -- Internal Instruction Cache (iCACHE) --
      ICACHE_EN                    : boolean                  := false;
      ICACHE_NUM_BLOCKS            : natural range 1 to 256   := 4;
      ICACHE_BLOCK_SIZE            : natural range 4 to 2**16 := 64;
      ICACHE_ASSOCIATIVITY         : natural range 1 to 2     := 1;
      -- Internal Data Cache (dCACHE) --
      DCACHE_EN                    : boolean                  := false;
      DCACHE_NUM_BLOCKS            : natural range 1 to 256   := 4;
      DCACHE_BLOCK_SIZE            : natural range 4 to 2**16 := 64;
      -- External memory interface (WISHBONE) --
      MEM_EXT_EN                   : boolean := false;
      MEM_EXT_TIMEOUT              : natural := 255;
      MEM_EXT_PIPE_MODE            : boolean := false;
      MEM_EXT_BIG_ENDIAN           : boolean := false;
      MEM_EXT_ASYNC_RX             : boolean := false;
      MEM_EXT_ASYNC_TX             : boolean := false;
      -- External Interrupts Controller (XIRQ) --
      XIRQ_NUM_CH                  : natural range 0 to 32          := 0;
      XIRQ_TRIGGER_TYPE            : std_ulogic_vector(31 downto 0) := x"ffffffff";
      XIRQ_TRIGGER_POLARITY        : std_ulogic_vector(31 downto 0) := x"ffffffff";
      -- Processor peripherals --
      IO_GPIO_NUM                  : natural range 0 to 64          := 0;
      IO_MTIME_EN                  : boolean                        := false;
      IO_UART0_EN                  : boolean                        := false;
      IO_UART0_RX_FIFO             : natural range 1 to 2**15       := 1;
      IO_UART0_TX_FIFO             : natural range 1 to 2**15       := 1;
      IO_UART1_EN                  : boolean                        := false;
      IO_UART1_RX_FIFO             : natural range 1 to 2**15       := 1;
      IO_UART1_TX_FIFO             : natural range 1 to 2**15       := 1;
      IO_SPI_EN                    : boolean                        := false;
      IO_SPI_FIFO                  : natural range 1 to 2**15       := 1;
      IO_SDI_EN                    : boolean                        := false;
      IO_SDI_FIFO                  : natural range 1 to 2**15       := 1;
      IO_TWI_EN                    : boolean                        := false;
      IO_PWM_NUM_CH                : natural range 0 to 12          := 0;
      IO_WDT_EN                    : boolean                        := false;
      IO_TRNG_EN                   : boolean                        := false;
      IO_TRNG_FIFO                 : natural range 1 to 2**15       := 1;
      IO_CFS_EN                    : boolean                        := false;
      IO_CFS_CONFIG                : std_ulogic_vector(31 downto 0) := x"00000000";
      IO_CFS_IN_SIZE               : natural                        := 32;
      IO_CFS_OUT_SIZE              : natural                        := 32;
      IO_NEOLED_EN                 : boolean                        := false;
      IO_NEOLED_TX_FIFO            : natural range 1 to 2**15       := 1;
      IO_GPTMR_EN                  : boolean                        := false;
      IO_XIP_EN                    : boolean                        := false;
      IO_ONEWIRE_EN                : boolean                        := false;
      IO_DMA_EN                    : boolean                        := false;
      IO_SLINK_EN                  : boolean                        := false;
      IO_SLINK_RX_FIFO             : natural range 1 to 2**15       := 1;
      IO_SLINK_TX_FIFO             : natural range 1 to 2**15       := 1;
      IO_CRC_EN                    : boolean                        := false
    );
    port (
      -- Global control --
      clk_i          : in  std_ulogic;
      rstn_i         : in  std_ulogic;
      -- JTAG on-chip debugger interface --
      jtag_trst_i    : in  std_ulogic := 'U';
      jtag_tck_i     : in  std_ulogic := 'U';
      jtag_tdi_i     : in  std_ulogic := 'U';
      jtag_tdo_o     : out std_ulogic;
      jtag_tms_i     : in  std_ulogic := 'U';
      -- Wishbone bus interface (available if MEM_EXT_EN = true) --
      wb_tag_o       : out std_ulogic_vector(02 downto 0);
      wb_adr_o       : out std_ulogic_vector(31 downto 0);
      wb_dat_i       : in  std_ulogic_vector(31 downto 0) := (others => 'U');
      wb_dat_o       : out std_ulogic_vector(31 downto 0);
      wb_we_o        : out std_ulogic;
      wb_sel_o       : out std_ulogic_vector(03 downto 0);
      wb_stb_o       : out std_ulogic;
      wb_cyc_o       : out std_ulogic;
      wb_ack_i       : in  std_ulogic := 'L';
      wb_err_i       : in  std_ulogic := 'L';
      -- Stream Link Interface (available if IO_SLINK_EN = true) --
      slink_rx_dat_i : in  std_ulogic_vector(31 downto 0) := (others => 'U');
      slink_rx_val_i : in  std_ulogic := 'L';
      slink_rx_rdy_o : out std_ulogic;
      slink_tx_dat_o : out std_ulogic_vector(31 downto 0);
      slink_tx_val_o : out std_ulogic;
      slink_tx_rdy_i : in  std_ulogic := 'L';
      -- Advanced memory control signals --
      fence_o        : out std_ulogic;
      fencei_o       : out std_ulogic;
      -- XIP (execute in place via SPI) signals (available if IO_XIP_EN = true) --
      xip_csn_o      : out std_ulogic;
      xip_clk_o      : out std_ulogic;
      xip_dat_i      : in  std_ulogic := 'L';
      xip_dat_o      : out std_ulogic;
      -- GPIO (available if IO_GPIO_NUM > 0) --
      gpio_o         : out std_ulogic_vector(63 downto 0);
      gpio_i         : in  std_ulogic_vector(63 downto 0) := (others => 'U');
      -- primary UART0 (available if IO_UART0_EN = true) --
      uart0_txd_o    : out std_ulogic;
      uart0_rxd_i    : in  std_ulogic := 'U';
      uart0_rts_o    : out std_ulogic;
      uart0_cts_i    : in  std_ulogic := 'L';
      -- secondary UART1 (available if IO_UART1_EN = true) --
      uart1_txd_o    : out std_ulogic;
      uart1_rxd_i    : in  std_ulogic := 'U'; -- UART1 receive data
      uart1_rts_o    : out std_ulogic;
      uart1_cts_i    : in  std_ulogic := 'L';
      -- SPI (available if IO_SPI_EN = true) --
      spi_clk_o      : out std_ulogic;
      spi_dat_o      : out std_ulogic;
      spi_dat_i      : in  std_ulogic := 'U';
      spi_csn_o      : out std_ulogic_vector(07 downto 0); -- SPI CS
      -- SDI (available if IO_SDI_EN = true) --
      sdi_clk_i      : in  std_ulogic := 'U';
      sdi_dat_o      : out std_ulogic;
      sdi_dat_i      : in  std_ulogic := 'U';
      sdi_csn_i      : in  std_ulogic := 'H';
      -- TWI (available if IO_TWI_EN = true) --
      twi_sda_i      : in  std_ulogic := 'H';
      twi_sda_o      : out std_ulogic;
      twi_scl_i      : in  std_ulogic := 'H';
      twi_scl_o      : out std_ulogic;
      -- 1-Wire Interface (available if IO_ONEWIRE_EN = true) --
      onewire_i      : in  std_ulogic := 'H';
      onewire_o      : out std_ulogic;
      -- PWM (available if IO_PWM_NUM_CH > 0) --
      pwm_o          : out std_ulogic_vector(11 downto 0); -- pwm channels
      -- Custom Functions Subsystem IO --
      cfs_in_i       : in  std_ulogic_vector(IO_CFS_IN_SIZE-1 downto 0) := (others => 'U');
      cfs_out_o      : out std_ulogic_vector(IO_CFS_OUT_SIZE-1 downto 0);
      -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
      neoled_o       : out std_ulogic;
      -- External platform interrupts (available if XIRQ_NUM_CH > 0) --
      xirq_i         : in  std_ulogic_vector(31 downto 0) := (others => 'L');
      -- CPU Interrupts --
      mtime_irq_i    : in  std_ulogic := 'L';
      msw_irq_i      : in  std_ulogic := 'L';
      mext_irq_i     : in  std_ulogic := 'L'
    );
  end component;

end neorv32_package;

package body neorv32_package is

-- ****************************************************************************************************************************
-- Functions
-- ****************************************************************************************************************************

  -- Minimal required number of bits to represent <input> numbers ---------------------------
  -- -------------------------------------------------------------------------------------------
  function index_size_f(input : natural) return natural is
  begin
    for i in 0 to natural'high loop
      if (2**i >= input) then
        return i;
      end if;
    end loop;
    return 0;
  end function index_size_f;

  -- Conditional select integer -------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function cond_sel_int_f(cond : boolean; val_t : integer; val_f : integer) return integer is
  begin
    if (cond = true) then
      return val_t;
    else
      return val_f;
    end if;
  end function cond_sel_int_f;

  -- Conditional select natural -------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function cond_sel_natural_f(cond : boolean; val_t : natural; val_f : natural) return natural is
  begin
    if (cond = true) then
      return val_t;
    else
      return val_f;
    end if;
  end function cond_sel_natural_f;

  -- Conditional select std_ulogic_vector ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function cond_sel_suv_f(cond : boolean; val_t : std_ulogic_vector; val_f : std_ulogic_vector) return std_ulogic_vector is
  begin
    if (cond = true) then
      return val_t;
    else
      return val_f;
    end if;
  end function cond_sel_suv_f;

  -- Conditional select string --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function cond_sel_string_f(cond : boolean; val_t : string; val_f : string) return string is
  begin
    if (cond = true) then
      return val_t;
    else
      return val_f;
    end if;
  end function cond_sel_string_f;

  -- Convert boolean to std_ulogic ----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function bool_to_ulogic_f(cond : boolean) return std_ulogic is
  begin
    if (cond = true) then
      return '1';
    else
      return '0';
    end if;
  end function bool_to_ulogic_f;

  -- Convert binary to gray -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function bin_to_gray_f(input : std_ulogic_vector) return std_ulogic_vector is
    variable tmp_v : std_ulogic_vector(input'range);
  begin
    tmp_v(input'length-1) := input(input'length-1); -- keep MSB
    for i in input'length-2 downto 0 loop
      tmp_v(i) := input(i) xor input(i+1);
    end loop;
    return tmp_v;
  end function bin_to_gray_f;

  -- Convert gray to binary -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function gray_to_bin_f(input : std_ulogic_vector) return std_ulogic_vector is
    variable tmp_v : std_ulogic_vector(input'range);
  begin
    tmp_v(input'length-1) := input(input'length-1); -- keep MSB
    for i in input'length-2 downto 0 loop
      tmp_v(i) := tmp_v(i+1) xor input(i);
    end loop;
    return tmp_v;
  end function gray_to_bin_f;

  -- OR all bits ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function or_reduce_f(input : std_ulogic_vector) return std_ulogic is
    variable tmp_v : std_ulogic;
  begin
    tmp_v := '0';
    for i in input'range loop
      tmp_v := tmp_v or input(i);
    end loop;
    return tmp_v;
  end function or_reduce_f;

  -- AND all bits ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function and_reduce_f(input : std_ulogic_vector) return std_ulogic is
    variable tmp_v : std_ulogic;
  begin
    tmp_v := '1';
    for i in input'range loop
      tmp_v := tmp_v and input(i);
    end loop;
    return tmp_v;
  end function and_reduce_f;

  -- XOR all bits ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function xor_reduce_f(input : std_ulogic_vector) return std_ulogic is
    variable tmp_v : std_ulogic;
  begin
    tmp_v := '0';
    for i in input'range loop
      tmp_v := tmp_v xor input(i);
    end loop;
    return tmp_v;
  end function xor_reduce_f;

  -- Check if std_ulogic is not '1' or '0' --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function su_undefined_f(input : std_ulogic) return boolean is
  begin
    case input is
      when '1' | '0' => return false;
      when others    => return true;
    end case;
  end function su_undefined_f;

  -- Convert std_ulogic_vector to lowercase HEX char ----------------------------------------
  -- -------------------------------------------------------------------------------------------
  function to_hexchar_f(input : std_ulogic_vector(3 downto 0)) return character is
    variable hex_v : string(1 to 16);
  begin
    hex_v := "0123456789abcdef";
    if (su_undefined_f(input(3)) = true) or (su_undefined_f(input(2)) = true) or
       (su_undefined_f(input(1)) = true) or (su_undefined_f(input(0)) = true) then
      return '?';
    else
      return hex_v(to_integer(unsigned(input)) + 1);
    end if;
  end function to_hexchar_f;

  -- Convert 32-bit std_ulogic_vector to hex string -----------------------------------------
  -- -------------------------------------------------------------------------------------------
  function to_hstring32_f(input : std_ulogic_vector(31 downto 0)) return string is
    variable res_v : string(1 to 8);
  begin
    for i in 7 downto 0 loop
      res_v(8-i) := to_hexchar_f(input(i*4+3 downto i*4+0));
    end loop;
    return res_v;
  end function to_hstring32_f;

  -- Bit reversal ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function bit_rev_f(input : std_ulogic_vector) return std_ulogic_vector is
    variable output_v : std_ulogic_vector(input'range);
  begin
    for i in 0 to input'length-1 loop
      output_v(input'length-i-1) := input(i);
    end loop;
    return output_v;
  end function bit_rev_f;

  -- Test if input number is a power of two -------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function is_power_of_two_f(input : natural) return boolean is
    variable tmp : unsigned(31 downto 0);
  begin
    if (input = 0) then
      return false;
    elsif (input = 1) then
      return true;
    else
      tmp := to_unsigned(input, 32);
      if ((tmp and (tmp - 1)) = 0) then
        return true;
      else
        return false;
      end if;
    end if;
  end function is_power_of_two_f;

  -- Swap all bytes of a 32-bit word (endianness conversion) --------------------------------
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

  -- Population count (number of set bits) --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function popcount_f(input : std_ulogic_vector) return natural is
    variable cnt_v : natural range 0 to input'length;
  begin
    cnt_v := 0;
    for i in input'length-1 downto 0 loop
      if (input(i) = '1') then
        cnt_v := cnt_v + 1;
      end if;
    end loop;
    return cnt_v;
  end function popcount_f;

  -- Count leading zeros --------------------------------------------------------------------
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
    end loop;
    return cnt_v;
  end function leading_zeros_f;

  -- Initialize mem32_t array from another mem32_t array ------------------------------------
  -- -------------------------------------------------------------------------------------------
  impure function mem32_init_f(init : mem32_t; depth : natural) return mem32_t is
    variable mem_v : mem32_t(0 to depth-1);
  begin
    mem_v := (others => (others => '0')); -- [IMPORTANT] make sure remaining memory entries are set to zero
    if (init'length > depth) then
      return mem_v;
    end if;
    for i in 0 to init'length-1 loop -- initialize only in range of source data array
      mem_v(i) := init(i);
    end loop;
    return mem_v;
  end function mem32_init_f;


end neorv32_package;

-- ****************************************************************************************************************************
-- Additional Packages
-- ****************************************************************************************************************************

  -- Prototype Definition: bootloader_init_image --------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- > memory content in 'neorv32_bootloader_image.vhd', auto-generated by 'image_gen'
  -- > used by 'neorv32_boot_rom.vhd'
  -- > enables body-only recompile in case of firmware change (NEORV32 PR #338)

library ieee;
use ieee.std_logic_1164.all;

library neorv32;
use neorv32.neorv32_package.all;

package neorv32_bootloader_image is
  constant bootloader_init_image : mem32_t;
end neorv32_bootloader_image;


  -- Prototype Definition: neorv32_application_image ----------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- > memory content in 'neorv32_application_image.vhd', auto-generated by 'image_gen'
  -- > used by 'mem/neorv32_imem.*.vhd'
  -- > enables body-only recompile in case of firmware change (NEORV32 PR #338)

library ieee;
use ieee.std_logic_1164.all;

library neorv32;
use neorv32.neorv32_package.all;

package neorv32_application_image is
  constant application_init_image : mem32_t;
end neorv32_application_image;
