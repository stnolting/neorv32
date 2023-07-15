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

  -- "response time window" for processor-internal modules --
  -- = cycles after which an *unacknowledged* internal bus access will timeout and trigger a bus fault exception
  constant max_proc_int_response_time_c : natural := 15; -- default = 15

  -- log2 of co-processor timeout cycles --
  constant cp_timeout_c : natural := 7; -- default = 7 (= 128 cycles)

  -- Architecture Constants -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant hw_version_c : std_ulogic_vector(31 downto 0) := x"01080604"; -- hardware version
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

  -- Main Memory Sections Address Map ---
  constant mem_ispace_base_c : std_ulogic_vector(31 downto 0) := x"00000000"; -- IMEM size via generic
  constant mem_dspace_base_c : std_ulogic_vector(31 downto 0) := x"80000000"; -- DMEM size via generic
  constant mem_xip_base_c    : std_ulogic_vector(31 downto 0) := x"e0000000"; -- page (4MSBs) only!
  constant mem_xip_size_c    : natural := 256*1024*1024;
  constant mem_boot_base_c   : std_ulogic_vector(31 downto 0) := x"ffffc000";
  constant mem_boot_size_c   : natural := 8*1024;
  constant mem_io_base_c     : std_ulogic_vector(31 downto 0) := x"ffffe000";
  constant mem_io_size_c     : natural := 8*1024;

  -- Start of uncached memory access (page / 4MSBs only) --
  constant uncached_begin_c  : std_ulogic_vector(31 downto 0) := x"f0000000";

  -- IO Address Map --
  constant iodev_size_c      : natural := 256; -- size of a single IO device (bytes)
--constant base_res_cfs_c    : std_ulogic_vector(31 downto 0) := x"ffffe000"; -- reserved
--constant base_res_cfs_c    : std_ulogic_vector(31 downto 0) := x"ffffe100"; -- reserved
--constant base_res_cfs_c    : std_ulogic_vector(31 downto 0) := x"ffffe200"; -- reserved
--constant base_res_cfs_c    : std_ulogic_vector(31 downto 0) := x"ffffe300"; -- reserved
--constant base_res_cfs_c    : std_ulogic_vector(31 downto 0) := x"ffffe400"; -- reserved
--constant base_res_cfs_c    : std_ulogic_vector(31 downto 0) := x"ffffe500"; -- reserved
--constant base_res_cfs_c    : std_ulogic_vector(31 downto 0) := x"ffffe600"; -- reserved
--constant base_res_cfs_c    : std_ulogic_vector(31 downto 0) := x"ffffe700"; -- reserved
--constant base_res_cfs_c    : std_ulogic_vector(31 downto 0) := x"ffffe800"; -- reserved
--constant base_res_cfs_c    : std_ulogic_vector(31 downto 0) := x"ffffe900"; -- reserved
--constant base_res_cfs_c    : std_ulogic_vector(31 downto 0) := x"ffffea00"; -- reserved
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

  -- OCD Debug Module Entry Points --
  constant dm_exc_entry_c  : std_ulogic_vector(31 downto 0) := x"ffffff00"; -- = base_io_dm_c + 0, exceptions entry point
  constant dm_park_entry_c : std_ulogic_vector(31 downto 0) := x"ffffff08"; -- = base_io_dm_c + 8, normal entry point

-- ****************************************************************************************************************************
-- SoC Definitions
-- ****************************************************************************************************************************

  -- SoC Clock Select --
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
  end record;

  -- bus response --
  type bus_rsp_t is record
    data : std_ulogic_vector(31 downto 0); -- read data
    ack  : std_ulogic; -- access acknowledge (single-shot)
    err  : std_ulogic; -- access error (single-shot)
  end record;

  -- source termination --
  constant req_terminate_c : bus_req_t := (
    addr => (others => '0'),
    data => (others => '0'),
    ben  => (others => '0'),
    we   => '0',
    re   => '0',
    src  => '0',
    priv => '0'
  );

  -- endpoint termination --
  constant rsp_terminate_c : bus_rsp_t := (
    data => (others => '0'),
    ack  => '0',
    err  => '0'
  );

  -- pre-constrained array types --
  type bus_req_array_t is array (20 downto 0) of bus_req_t;
  type bus_rsp_array_t is array (20 downto 0) of bus_rsp_t;


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
  constant funct3_lb_c     : std_ulogic_vector(2 downto 0) := "000"; -- load byte
  constant funct3_lh_c     : std_ulogic_vector(2 downto 0) := "001"; -- load half word
  constant funct3_lw_c     : std_ulogic_vector(2 downto 0) := "010"; -- load word
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
  constant funct3_csril_c  : std_ulogic_vector(2 downto 0) := "100"; -- undefined/illegal
  constant funct3_csrrwi_c : std_ulogic_vector(2 downto 0) := "101"; -- csr r/w immediate
  constant funct3_csrrsi_c : std_ulogic_vector(2 downto 0) := "110"; -- csr read & set immediate
  constant funct3_csrrci_c : std_ulogic_vector(2 downto 0) := "111"; -- csr read & clear immediate
  -- fence --
  constant funct3_fence_c  : std_ulogic_vector(2 downto 0) := "000"; -- fence - order IO/memory access
  constant funct3_fencei_c : std_ulogic_vector(2 downto 0) := "001"; -- fence.i - instruction stream sync

  -- RISC-V Funct12 -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- system --
  constant funct12_ecall_c  : std_ulogic_vector(11 downto 0) := x"000"; -- ecall
  constant funct12_ebreak_c : std_ulogic_vector(11 downto 0) := x"001"; -- ebreak
  constant funct12_wfi_c    : std_ulogic_vector(11 downto 0) := x"105"; -- wfi
  constant funct12_mret_c   : std_ulogic_vector(11 downto 0) := x"302"; -- mret
  constant funct12_dret_c   : std_ulogic_vector(11 downto 0) := x"7b2"; -- dret

  -- RISC-V Floating-Point Stuff ------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- formats --
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

  -- RISC-V CSR Addresses -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant csr_zero_c           : std_ulogic_vector(11 downto 0) := x"000"; -- always returns zero, only relevant for hardware access
  -- <<< standard read/write CSRs >>> --
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
  --
  constant csr_mstatush_c       : std_ulogic_vector(11 downto 0) := x"310";
  -- machine configuration --
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
  constant csr_mscratch_c       : std_ulogic_vector(11 downto 0) := x"340";
  constant csr_mepc_c           : std_ulogic_vector(11 downto 0) := x"341";
  constant csr_mcause_c         : std_ulogic_vector(11 downto 0) := x"342";
  constant csr_mtval_c          : std_ulogic_vector(11 downto 0) := x"343";
  constant csr_mip_c            : std_ulogic_vector(11 downto 0) := x"344";
  -- physical memory protection - configuration --
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
  -- trigger module registers --
  constant csr_tselect_c        : std_ulogic_vector(11 downto 0) := x"7a0";
  constant csr_tdata1_c         : std_ulogic_vector(11 downto 0) := x"7a1";
  constant csr_tdata2_c         : std_ulogic_vector(11 downto 0) := x"7a2";
  constant csr_tdata3_c         : std_ulogic_vector(11 downto 0) := x"7a3";
  constant csr_tinfo_c          : std_ulogic_vector(11 downto 0) := x"7a4";
  constant csr_tcontrol_c       : std_ulogic_vector(11 downto 0) := x"7a5";
  constant csr_mcontext_c       : std_ulogic_vector(11 downto 0) := x"7a8";
  constant csr_scontext_c       : std_ulogic_vector(11 downto 0) := x"7aa";
  -- debug mode registers --
  constant csr_dcsr_c           : std_ulogic_vector(11 downto 0) := x"7b0";
  constant csr_dpc_c            : std_ulogic_vector(11 downto 0) := x"7b1";
  constant csr_dscratch0_c      : std_ulogic_vector(11 downto 0) := x"7b2";
  -- machine counters/timers --
  constant csr_mcycle_c         : std_ulogic_vector(11 downto 0) := x"b00";
  constant csr_mtime_c          : std_ulogic_vector(11 downto 0) := x"b01"; -- dummy address
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
  constant csr_mtimeh_c         : std_ulogic_vector(11 downto 0) := x"b81"; -- dummy address
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
  constant csr_hpmcounter16_c   : std_ulogic_vector(11 downto 0) := x"c10";
  constant csr_hpmcounter17_c   : std_ulogic_vector(11 downto 0) := x"c11";
  constant csr_hpmcounter18_c   : std_ulogic_vector(11 downto 0) := x"c12";
  constant csr_hpmcounter19_c   : std_ulogic_vector(11 downto 0) := x"c13";
  constant csr_hpmcounter20_c   : std_ulogic_vector(11 downto 0) := x"c14";
  constant csr_hpmcounter21_c   : std_ulogic_vector(11 downto 0) := x"c15";
  constant csr_hpmcounter22_c   : std_ulogic_vector(11 downto 0) := x"c16";
  constant csr_hpmcounter23_c   : std_ulogic_vector(11 downto 0) := x"c17";
  constant csr_hpmcounter24_c   : std_ulogic_vector(11 downto 0) := x"c18";
  constant csr_hpmcounter25_c   : std_ulogic_vector(11 downto 0) := x"c19";
  constant csr_hpmcounter26_c   : std_ulogic_vector(11 downto 0) := x"c1a";
  constant csr_hpmcounter27_c   : std_ulogic_vector(11 downto 0) := x"c1b";
  constant csr_hpmcounter28_c   : std_ulogic_vector(11 downto 0) := x"c1c";
  constant csr_hpmcounter29_c   : std_ulogic_vector(11 downto 0) := x"c1d";
  constant csr_hpmcounter30_c   : std_ulogic_vector(11 downto 0) := x"c1e";
  constant csr_hpmcounter31_c   : std_ulogic_vector(11 downto 0) := x"c1f";
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
  constant csr_hpmcounter16h_c  : std_ulogic_vector(11 downto 0) := x"c90";
  constant csr_hpmcounter17h_c  : std_ulogic_vector(11 downto 0) := x"c91";
  constant csr_hpmcounter18h_c  : std_ulogic_vector(11 downto 0) := x"c92";
  constant csr_hpmcounter19h_c  : std_ulogic_vector(11 downto 0) := x"c93";
  constant csr_hpmcounter20h_c  : std_ulogic_vector(11 downto 0) := x"c94";
  constant csr_hpmcounter21h_c  : std_ulogic_vector(11 downto 0) := x"c95";
  constant csr_hpmcounter22h_c  : std_ulogic_vector(11 downto 0) := x"c96";
  constant csr_hpmcounter23h_c  : std_ulogic_vector(11 downto 0) := x"c97";
  constant csr_hpmcounter24h_c  : std_ulogic_vector(11 downto 0) := x"c98";
  constant csr_hpmcounter25h_c  : std_ulogic_vector(11 downto 0) := x"c99";
  constant csr_hpmcounter26h_c  : std_ulogic_vector(11 downto 0) := x"c9a";
  constant csr_hpmcounter27h_c  : std_ulogic_vector(11 downto 0) := x"c9b";
  constant csr_hpmcounter28h_c  : std_ulogic_vector(11 downto 0) := x"c9c";
  constant csr_hpmcounter29h_c  : std_ulogic_vector(11 downto 0) := x"c9d";
  constant csr_hpmcounter30h_c  : std_ulogic_vector(11 downto 0) := x"c9e";
  constant csr_hpmcounter31h_c  : std_ulogic_vector(11 downto 0) := x"c9f";
  -- machine information registers --
  constant csr_mvendorid_c      : std_ulogic_vector(11 downto 0) := x"f11";
  constant csr_marchid_c        : std_ulogic_vector(11 downto 0) := x"f12";
  constant csr_mimpid_c         : std_ulogic_vector(11 downto 0) := x"f13";
  constant csr_mhartid_c        : std_ulogic_vector(11 downto 0) := x"f14";
  constant csr_mconfigptr_c     : std_ulogic_vector(11 downto 0) := x"f15";
  -- <<< NEORV32-specific (custom) read-only CSRs >>> ---
  -- machine extended ISA extensions information --
  constant csr_mxisa_c          : std_ulogic_vector(11 downto 0) := x"fc0";

  -- PMP Modes ------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant pmp_mode_off_c   : std_ulogic_vector(1 downto 0) := "00"; -- null region (disabled)
  constant pmp_mode_tor_c   : std_ulogic_vector(1 downto 0) := "01"; -- top of range
  constant pmp_mode_na4_c   : std_ulogic_vector(1 downto 0) := "10"; -- naturally aligned four-byte region
  constant pmp_mode_napot_c : std_ulogic_vector(1 downto 0) := "11"; -- naturally aligned power-of-two region (>= 8 bytes)

-- ****************************************************************************************************************************
-- CPU Control
-- ****************************************************************************************************************************

  -- Main CPU Control Bus -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  type ctrl_bus_t is record
    -- register file --
    rf_wb_en      : std_ulogic; -- write back enable
    rf_rs1        : std_ulogic_vector(04 downto 0); -- source register 1 address
    rf_rs2        : std_ulogic_vector(04 downto 0); -- source register 2 address
    rf_rs3        : std_ulogic_vector(04 downto 0); -- source register 3 address
    rf_rd         : std_ulogic_vector(04 downto 0); -- destination register address
    rf_mux        : std_ulogic_vector(01 downto 0); -- input source select
    rf_zero_we    : std_ulogic;                     -- allow/force write access to x0
    -- alu --
    alu_op        : std_ulogic_vector(02 downto 0); -- ALU operation select
    alu_opa_mux   : std_ulogic;                     -- operand A select (0=rs1, 1=PC)
    alu_opb_mux   : std_ulogic;                     -- operand B select (0=rs2, 1=IMM)
    alu_unsigned  : std_ulogic;                     -- is unsigned ALU operation
    alu_frm       : std_ulogic_vector(02 downto 0); -- FPU rounding mode
    alu_cp_trig   : std_ulogic_vector(05 downto 0); -- co-processor trigger (one-hot)
    -- bus interface --
    bus_req_rd    : std_ulogic;                     -- trigger memory read request
    bus_req_wr    : std_ulogic;                     -- trigger memory write request
    bus_mo_we     : std_ulogic;                     -- memory address and data output register write enable
    bus_fence     : std_ulogic;                     -- fence operation
    bus_fencei    : std_ulogic;                     -- fence.i operation
    bus_priv      : std_ulogic;                     -- effective privilege level for load/store
    -- instruction word --
    ir_funct3     : std_ulogic_vector(02 downto 0); -- funct3 bit field
    ir_funct12    : std_ulogic_vector(11 downto 0); -- funct12 bit field
    ir_opcode     : std_ulogic_vector(06 downto 0); -- opcode bit field
    -- cpu status --
    cpu_priv      : std_ulogic;                     -- effective privilege mode
    cpu_sleep     : std_ulogic;                     -- set when CPU is in sleep mode
    cpu_trap      : std_ulogic;                     -- set when CPU is entering trap exec
    cpu_debug     : std_ulogic;                     -- set when CPU is in debug mode
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
    alu_frm      => (others => '0'),
    alu_cp_trig  => (others => '0'),
    bus_req_rd   => '0',
    bus_req_wr   => '0',
    bus_mo_we    => '0',
    bus_fence    => '0',
    bus_fencei   => '0',
    bus_priv     => '0',
    ir_funct3    => (others => '0'),
    ir_funct12   => (others => '0'),
    ir_opcode    => (others => '0'),
    cpu_priv     => '0',
    cpu_sleep    => '0',
    cpu_trap     => '0',
    cpu_debug    => '0'
  );

  -- PMP Interface --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  type pmp_ctrl_if_t is array (0 to 15) of std_ulogic_vector(07 downto 0);
  type pmp_addr_if_t is array (0 to 15) of std_ulogic_vector(33 downto 0);

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
  constant cp_sel_cond_c     : natural := 5; -- CP5: conditional operations ('Zicond' extension)

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
  constant trap_ima_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00000"; -- 0:  instruction misaligned
  constant trap_iaf_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00001"; -- 1:  instruction access fault
  constant trap_iil_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00010"; -- 2:  illegal instruction
  constant trap_brk_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00011"; -- 3:  breakpoint
  constant trap_lma_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00100"; -- 4:  load address misaligned
  constant trap_laf_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00101"; -- 5:  load access fault
  constant trap_sma_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00110"; -- 6:  store address misaligned
  constant trap_saf_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00111"; -- 7:  store access fault
  constant trap_env_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "010UU"; -- 8..11:  environment call from u/s/h/m
--constant trap_ipf_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "01100"; -- 12: instruction page fault
--constant trap_lpf_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "01101"; -- 13: load page fault
--constant trap_???_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "01110"; -- 14: reserved
--constant trap_spf_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "01111"; -- 15: store page fault
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
  constant exc_iillegal_c : natural :=  1; -- illegal instruction
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
  function cond_sel_natural_f(cond : boolean; val_t : natural; val_f : natural) return natural;
  function cond_sel_int_f(cond : boolean; val_t : integer; val_f : integer) return integer;
  function cond_sel_stdulogicvector_f(cond : boolean; val_t : std_ulogic_vector; val_f : std_ulogic_vector) return std_ulogic_vector;
  function cond_sel_stdulogic_f(cond : boolean; val_t : std_ulogic; val_f : std_ulogic) return std_ulogic;
  function cond_sel_string_f(cond : boolean; val_t : string; val_f : string) return string;
  function bool_to_ulogic_f(cond : boolean) return std_ulogic;
  function bin_to_gray_f(input : std_ulogic_vector) return std_ulogic_vector;
  function gray_to_bin_f(input : std_ulogic_vector) return std_ulogic_vector;
  function or_reduce_f(a : std_ulogic_vector) return std_ulogic;
  function and_reduce_f(a : std_ulogic_vector) return std_ulogic;
  function xor_reduce_f(a : std_ulogic_vector) return std_ulogic;
  function to_hexchar_f(input : std_ulogic_vector(3 downto 0)) return character;
  function to_hstring32_f(input : std_ulogic_vector(31 downto 0)) return string;
  function hexchar_to_stdulogicvector_f(input : character) return std_ulogic_vector;
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
      CLOCK_FREQUENCY              : natural;           -- clock frequency of clk_i in Hz
      HART_ID                      : std_ulogic_vector(31 downto 0) := x"00000000"; -- hardware thread ID
      VENDOR_ID                    : std_ulogic_vector(31 downto 0) := x"00000000"; -- vendor's JEDEC ID
      CUSTOM_ID                    : std_ulogic_vector(31 downto 0) := x"00000000"; -- custom user-defined ID
      INT_BOOTLOADER_EN            : boolean := false;  -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
      -- On-Chip Debugger (OCD) --
      ON_CHIP_DEBUGGER_EN          : boolean := false;  -- implement on-chip debugger
      -- RISC-V CPU Extensions --
      CPU_EXTENSION_RISCV_B        : boolean := false;  -- implement bit-manipulation extension?
      CPU_EXTENSION_RISCV_C        : boolean := false;  -- implement compressed extension?
      CPU_EXTENSION_RISCV_E        : boolean := false;  -- implement embedded RF extension?
      CPU_EXTENSION_RISCV_M        : boolean := false;  -- implement mul/div extension?
      CPU_EXTENSION_RISCV_U        : boolean := false;  -- implement user mode extension?
      CPU_EXTENSION_RISCV_Zfinx    : boolean := false;  -- implement 32-bit floating-point extension (using INT regs!)
      CPU_EXTENSION_RISCV_Zicntr   : boolean := true;   -- implement base counters?
      CPU_EXTENSION_RISCV_Zicond   : boolean := false;  -- implement conditional operations extension?
      CPU_EXTENSION_RISCV_Zihpm    : boolean := false;  -- implement hardware performance monitors?
      CPU_EXTENSION_RISCV_Zifencei : boolean := false;  -- implement instruction stream sync.?
      CPU_EXTENSION_RISCV_Zmmul    : boolean := false;  -- implement multiply-only M sub-extension?
      CPU_EXTENSION_RISCV_Zxcfu    : boolean := false;  -- implement custom (instr.) functions unit?
      -- Tuning Options --
      FAST_MUL_EN                  : boolean := false;  -- use DSPs for M extension's multiplier
      FAST_SHIFT_EN                : boolean := false;  -- use barrel shifter for shift operations
      CPU_IPB_ENTRIES              : natural := 1;      -- entries in instruction prefetch buffer, has to be a power of 2, min 1
      -- Physical Memory Protection (PMP) --
      PMP_NUM_REGIONS              : natural := 0;      -- number of regions (0..16)
      PMP_MIN_GRANULARITY          : natural := 4;      -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
      -- Hardware Performance Monitors (HPM) --
      HPM_NUM_CNTS                 : natural := 0;      -- number of implemented HPM counters (0..29)
      HPM_CNT_WIDTH                : natural := 40;     -- total size of HPM counters (0..64)
      -- Internal Instruction memory (IMEM) --
      MEM_INT_IMEM_EN              : boolean := false;  -- implement processor-internal instruction memory
      MEM_INT_IMEM_SIZE            : natural := 16*1024; -- size of processor-internal instruction memory in bytes
      -- Internal Data memory (DMEM) --
      MEM_INT_DMEM_EN              : boolean := false;  -- implement processor-internal data memory
      MEM_INT_DMEM_SIZE            : natural := 8*1024; -- size of processor-internal data memory in bytes
      -- Internal Instruction Cache (iCACHE) --
      ICACHE_EN                    : boolean := false;  -- implement instruction cache
      ICACHE_NUM_BLOCKS            : natural := 4;      -- i-cache: number of blocks (min 1), has to be a power of 2
      ICACHE_BLOCK_SIZE            : natural := 64;     -- i-cache: block size in bytes (min 4), has to be a power of 2
      ICACHE_ASSOCIATIVITY         : natural := 1;      -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2
      -- Internal Data Cache (dCACHE) --
      DCACHE_EN                    : boolean := false;  -- implement data cache
      DCACHE_NUM_BLOCKS            : natural := 4;      -- d-cache: number of blocks (min 1), has to be a power of 2
      DCACHE_BLOCK_SIZE            : natural := 64;     -- d-cache: block size in bytes (min 4), has to be a power of 2
      -- External memory interface (WISHBONE) --
      MEM_EXT_EN                   : boolean := false;  -- implement external memory bus interface?
      MEM_EXT_TIMEOUT              : natural := 255;    -- cycles after a pending bus access auto-terminates (0 = disabled)
      MEM_EXT_PIPE_MODE            : boolean := false;  -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
      MEM_EXT_BIG_ENDIAN           : boolean := false;  -- byte order: true=big-endian, false=little-endian
      MEM_EXT_ASYNC_RX             : boolean := false;  -- use register buffer for RX data when false
      MEM_EXT_ASYNC_TX             : boolean := false;  -- use register buffer for TX data when false
      -- External Interrupts Controller (XIRQ) --
      XIRQ_NUM_CH                  : natural := 0;      -- number of external IRQ channels (0..32)
      XIRQ_TRIGGER_TYPE            : std_ulogic_vector(31 downto 0) := x"FFFFFFFF"; -- trigger type: 0=level, 1=edge
      XIRQ_TRIGGER_POLARITY        : std_ulogic_vector(31 downto 0) := x"FFFFFFFF"; -- trigger polarity: 0=low-level/falling-edge, 1=high-level/rising-edge
      -- Processor peripherals --
      IO_GPIO_NUM                  : natural := 0;      -- number of GPIO input/output pairs (0..64)
      IO_MTIME_EN                  : boolean := false;  -- implement machine system timer (MTIME)?
      IO_UART0_EN                  : boolean := false;  -- implement primary universal asynchronous receiver/transmitter (UART0)?
      IO_UART0_RX_FIFO             : natural := 1;      -- RX fifo depth, has to be a power of two, min 1
      IO_UART0_TX_FIFO             : natural := 1;      -- TX fifo depth, has to be a power of two, min 1
      IO_UART1_EN                  : boolean := false;  -- implement secondary universal asynchronous receiver/transmitter (UART1)?
      IO_UART1_RX_FIFO             : natural := 1;      -- RX fifo depth, has to be a power of two, min 1
      IO_UART1_TX_FIFO             : natural := 1;      -- TX fifo depth, has to be a power of two, min 1
      IO_SPI_EN                    : boolean := false;  -- implement serial peripheral interface (SPI)?
      IO_SPI_FIFO                  : natural := 1;      -- RTX fifo depth, has to be a power of two, min 1
      IO_SDI_EN                    : boolean := false;  -- implement serial data interface (SDI)?
      IO_SDI_FIFO                  : natural := 0;      -- RTX fifo depth, has to be zero or a power of two
      IO_TWI_EN                    : boolean := false;  -- implement two-wire interface (TWI)?
      IO_PWM_NUM_CH                : natural := 0;      -- number of PWM channels to implement (0..12); 0 = disabled
      IO_WDT_EN                    : boolean := false;  -- implement watch dog timer (WDT)?
      IO_TRNG_EN                   : boolean := false;  -- implement true random number generator (TRNG)?
      IO_TRNG_FIFO                 : natural := 1;      -- data fifo depth, has to be a power of two, min 1
      IO_CFS_EN                    : boolean := false;  -- implement custom functions subsystem (CFS)?
      IO_CFS_CONFIG                : std_ulogic_vector(31 downto 0) := x"00000000"; -- custom CFS configuration generic
      IO_CFS_IN_SIZE               : natural := 32;     -- size of CFS input conduit in bits
      IO_CFS_OUT_SIZE              : natural := 32;     -- size of CFS output conduit in bits
      IO_NEOLED_EN                 : boolean := false;  -- implement NeoPixel-compatible smart LED interface (NEOLED)?
      IO_NEOLED_TX_FIFO            : natural := 1;      -- NEOLED FIFO depth, has to be a power of two, min 1
      IO_GPTMR_EN                  : boolean := false;  -- implement general purpose timer (GPTMR)?
      IO_XIP_EN                    : boolean := false;  -- implement execute in place module (XIP)?
      IO_ONEWIRE_EN                : boolean := false;  -- implement 1-wire interface (ONEWIRE)?
      IO_DMA_EN                    : boolean := false;  -- implement direct memory access controller (DMA)?
      IO_SLINK_EN                  : boolean := false;  -- implement stream link interface (SLINK)?
      IO_SLINK_RX_FIFO             : natural := 1;      -- RX fifo depth, has to be a power of two, min 1
      IO_SLINK_TX_FIFO             : natural := 1;      -- TX fifo depth, has to be a power of two, min 1
      IO_CRC_EN                    : boolean := false   -- implement cyclic redundancy check unit (CRC)?
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
      wb_ack_i       : in  std_ulogic := 'L'; -- transfer acknowledge
      wb_err_i       : in  std_ulogic := 'L'; -- transfer error
      -- Stream Link Interface (available if IO_SLINK_EN = true) --
      slink_rx_dat_i : in  std_ulogic_vector(31 downto 0) := (others => 'U'); -- RX input data
      slink_rx_val_i : in  std_ulogic := 'L'; -- RX valid input
      slink_rx_rdy_o : out std_ulogic; -- RX ready to receive
      slink_tx_dat_o : out std_ulogic_vector(31 downto 0); -- TX output data
      slink_tx_val_o : out std_ulogic; -- TX valid output
      slink_tx_rdy_i : in  std_ulogic := 'L'; -- TX ready to send
      -- Advanced memory control signals --
      fence_o        : out std_ulogic; -- indicates an executed FENCE operation
      fencei_o       : out std_ulogic; -- indicates an executed FENCEI operation
      -- XIP (execute in place via SPI) signals (available if IO_XIP_EN = true) --
      xip_csn_o      : out std_ulogic; -- chip-select, low-active
      xip_clk_o      : out std_ulogic; -- serial clock
      xip_dat_i      : in  std_ulogic := 'L'; -- device data input
      xip_dat_o      : out std_ulogic; -- controller data output
      -- GPIO (available if IO_GPIO_NUM > 0) --
      gpio_o         : out std_ulogic_vector(63 downto 0); -- parallel output
      gpio_i         : in  std_ulogic_vector(63 downto 0) := (others => 'U'); -- parallel input
      -- primary UART0 (available if IO_UART0_EN = true) --
      uart0_txd_o    : out std_ulogic; -- UART0 send data
      uart0_rxd_i    : in  std_ulogic := 'U'; -- UART0 receive data
      uart0_rts_o    : out std_ulogic; -- HW flow control: UART0.RX ready to receive ("RTR"), low-active, optional
      uart0_cts_i    : in  std_ulogic := 'L'; -- HW flow control: UART0.TX allowed to transmit, low-active, optional
      -- secondary UART1 (available if IO_UART1_EN = true) --
      uart1_txd_o    : out std_ulogic; -- UART1 send data
      uart1_rxd_i    : in  std_ulogic := 'U'; -- UART1 receive data
      uart1_rts_o    : out std_ulogic; -- HW flow control: UART1.RX ready to receive ("RTR"), low-active, optional
      uart1_cts_i    : in  std_ulogic := 'L'; -- HW flow control: UART1.TX allowed to transmit, low-active, optional
      -- SPI (available if IO_SPI_EN = true) --
      spi_clk_o      : out std_ulogic; -- SPI serial clock
      spi_dat_o      : out std_ulogic; -- controller data out, peripheral data in
      spi_dat_i      : in  std_ulogic := 'U'; -- controller data in, peripheral data out
      spi_csn_o      : out std_ulogic_vector(07 downto 0); -- SPI CS
      -- SDI (available if IO_SDI_EN = true) --
      sdi_clk_i      : in  std_ulogic := 'U'; -- SDI serial clock
      sdi_dat_o      : out std_ulogic; -- controller data out, peripheral data in
      sdi_dat_i      : in  std_ulogic := 'U'; -- controller data in, peripheral data out
      sdi_csn_i      : in  std_ulogic := 'H'; -- chip-select
      -- TWI (available if IO_TWI_EN = true) --
      twi_sda_i      : in  std_ulogic := 'H'; -- serial data line sense input
      twi_sda_o      : out std_ulogic; -- serial data line output (pull low only)
      twi_scl_i      : in  std_ulogic := 'H'; -- serial clock line sense input
      twi_scl_o      : out std_ulogic; -- serial clock line output (pull low only)
      -- 1-Wire Interface (available if IO_ONEWIRE_EN = true) --
      onewire_i      : in  std_ulogic := 'H'; -- 1-wire bus sense input
      onewire_o      : out std_ulogic; -- 1-wire bus output (pull low only)
      -- PWM (available if IO_PWM_NUM_CH > 0) --
      pwm_o          : out std_ulogic_vector(11 downto 0); -- pwm channels
      -- Custom Functions Subsystem IO --
      cfs_in_i       : in  std_ulogic_vector(IO_CFS_IN_SIZE-1 downto 0) := (others => 'U'); -- custom CFS inputs conduit
      cfs_out_o      : out std_ulogic_vector(IO_CFS_OUT_SIZE-1 downto 0); -- custom CFS outputs conduit
      -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
      neoled_o       : out std_ulogic; -- async serial data line
      -- External platform interrupts (available if XIRQ_NUM_CH > 0) --
      xirq_i         : in  std_ulogic_vector(31 downto 0) := (others => 'L'); -- IRQ channels
      -- CPU Interrupts --
      mtime_irq_i    : in  std_ulogic := 'L'; -- machine timer interrupt, available if IO_MTIME_EN = false
      msw_irq_i      : in  std_ulogic := 'L'; -- machine software interrupt
      mext_irq_i     : in  std_ulogic := 'L'  -- machine external interrupt
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
    end loop; -- i
    return 0;
  end function index_size_f;

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

  -- Conditional select std_ulogic_vector ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function cond_sel_stdulogicvector_f(cond : boolean; val_t : std_ulogic_vector; val_f : std_ulogic_vector) return std_ulogic_vector is
  begin
    if (cond = true) then
      return val_t;
    else
      return val_f;
    end if;
  end function cond_sel_stdulogicvector_f;

  -- Conditional select std_ulogic ----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function cond_sel_stdulogic_f(cond : boolean; val_t : std_ulogic; val_f : std_ulogic) return std_ulogic is
  begin
    if (cond = true) then
      return val_t;
    else
      return val_f;
    end if;
  end function cond_sel_stdulogic_f;

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

  -- Convert bool to std_ulogic -------------------------------------------------------------
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
    end loop; -- i
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
    end loop; -- i
    return tmp_v;
  end function gray_to_bin_f;

  -- OR-reduce all bits ---------------------------------------------------------------------
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

  -- AND-reduce all bits --------------------------------------------------------------------
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

  -- XOR-reduce all bits --------------------------------------------------------------------
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

  -- Convert std_ulogic_vector to hex char --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function to_hexchar_f(input : std_ulogic_vector(3 downto 0)) return character is
    variable res_v : character;
  begin
    case input is
      when x"0"   => res_v := '0';
      when x"1"   => res_v := '1';
      when x"2"   => res_v := '2';
      when x"3"   => res_v := '3';
      when x"4"   => res_v := '4';
      when x"5"   => res_v := '5';
      when x"6"   => res_v := '6';
      when x"7"   => res_v := '7';
      when x"8"   => res_v := '8';
      when x"9"   => res_v := '9';
      when x"a"   => res_v := 'a';
      when x"b"   => res_v := 'b';
      when x"c"   => res_v := 'c';
      when x"d"   => res_v := 'd';
      when x"e"   => res_v := 'e';
      when x"f"   => res_v := 'f';
      when others => res_v := '?';
    end case;
    return res_v;
  end function to_hexchar_f;

  -- Convert 32-bit std_ulogic_vector to hex string -----------------------------------------
  -- -------------------------------------------------------------------------------------------
  function to_hstring32_f(input : std_ulogic_vector(31 downto 0)) return string is
    variable res_v : string(1 to 8);
    variable tmp_v : std_ulogic_vector(31 downto 0);
    variable hex_v : std_ulogic_vector(3 downto 0);
  begin
    tmp_v := bit_rev_f(input);
    for i in 0 to 7 loop
      hex_v := tmp_v(i*4+3 downto i*4+0);
      res_v(i+1) := to_hexchar_f(bit_rev_f(hex_v));
    end loop; -- i
    return res_v;
  end function to_hstring32_f;

  -- Convert hex char to 4-bit std_ulogic_vector --------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function hexchar_to_stdulogicvector_f(input : character) return std_ulogic_vector is
    variable res_v : std_ulogic_vector(3 downto 0);
  begin
    case input is
      when '0'       => res_v := x"0";
      when '1'       => res_v := x"1";
      when '2'       => res_v := x"2";
      when '3'       => res_v := x"3";
      when '4'       => res_v := x"4";
      when '5'       => res_v := x"5";
      when '6'       => res_v := x"6";
      when '7'       => res_v := x"7";
      when '8'       => res_v := x"8";
      when '9'       => res_v := x"9";
      when 'a' | 'A' => res_v := x"a";
      when 'b' | 'B' => res_v := x"b";
      when 'c' | 'C' => res_v := x"c";
      when 'd' | 'D' => res_v := x"d";
      when 'e' | 'E' => res_v := x"e";
      when 'f' | 'F' => res_v := x"f";
      when others    => res_v := x"0";
    end case;
    return res_v;
  end function hexchar_to_stdulogicvector_f;

  -- Bit reversal ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function bit_rev_f(input : std_ulogic_vector) return std_ulogic_vector is
    variable output_v : std_ulogic_vector(input'range);
  begin
    for i in 0 to input'length-1 loop
      output_v(input'length-i-1) := input(i);
    end loop; -- i
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
    end loop; -- i
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
    end loop; -- i
    return cnt_v;
  end function leading_zeros_f;

  --  Initialize mem32_t array from another mem32_t array -----------------------------------
  -- -------------------------------------------------------------------------------------------
  -- impure function: returns NOT the same result every time it is evaluated with the same arguments since the source file might have changed
  impure function mem32_init_f(init : mem32_t; depth : natural) return mem32_t is
    variable mem_v : mem32_t(0 to depth-1);
  begin
    mem_v := (others => (others => '0')); -- [IMPORTANT] make sure remaining memory entries are set to zero
    if (init'length > depth) then
      return mem_v;
    end if;
    for idx_v in 0 to init'length-1 loop -- init only in range of source data array
      mem_v(idx_v) := init(idx_v);
    end loop; -- idx_v
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
