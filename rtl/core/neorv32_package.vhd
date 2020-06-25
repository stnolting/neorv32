-- #################################################################################################
-- # << NEORV32 - Main VHDL package file >>                                                        #
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

package neorv32_package is

  -- Architecture Constants -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant data_width_c : natural := 32; -- data width - FIXED!
  constant hw_version_c : std_ulogic_vector(31 downto 0) := x"00000204"; -- no touchy!

  -- Internal Functions ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function index_size_f(input : natural) return natural;
  function cond_sel_natural_f(cond : boolean; val_t : natural; val_f : natural) return natural;
  function cond_sel_stdulogicvector_f(cond : boolean; val_t : std_ulogic_vector; val_f : std_ulogic_vector) return std_ulogic_vector;
  function bool_to_ulogic_f(cond : boolean) return std_ulogic;
  function or_all_f(a : std_ulogic_vector) return std_ulogic;
  function and_all_f(a : std_ulogic_vector) return std_ulogic;
  function xor_all_f(a : std_ulogic_vector) return std_ulogic;
  function xnor_all_f(a : std_ulogic_vector) return std_ulogic;

  -- Processor-internal Address Space Layout ------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- Instruction Memory & Data Memory --
  -- => configured via top's generics

  -- Bootloader ROM --
  constant boot_base_c          : std_ulogic_vector(data_width_c-1 downto 0) := x"FFFF0000"; -- bootloader base address, fixed!
  constant boot_size_c          : natural := 4*1024; -- bytes
  constant boot_max_size_c      : natural := 32*1024; -- bytes, fixed!

  -- IO: Peripheral Devices ("IO") Area --
  -- Control register(s) (including the device-enable) should be located at the base address of each device
  constant io_base_c            : std_ulogic_vector(data_width_c-1 downto 0) := x"FFFFFF80";
  constant io_size_c            : natural := 32*4; -- bytes, fixed!

  -- General Purpose Input/Output Unit (GPIO) --
  constant gpio_base_c          : std_ulogic_vector(data_width_c-1 downto 0) := x"FFFFFF80"; -- base address, fixed!
  constant gpio_size_c          : natural := 2*4; -- bytes, fixed!
  constant gpio_in_addr_c       : std_ulogic_vector(31 downto 0) := std_ulogic_vector(unsigned(gpio_base_c) + x"00000000");
  constant gpio_out_addr_c      : std_ulogic_vector(31 downto 0) := std_ulogic_vector(unsigned(gpio_base_c) + x"00000004");

  -- Core-Local Interrupt Controller (CLIC) --
  constant clic_base_c          : std_ulogic_vector(data_width_c-1 downto 0) := x"FFFFFF88"; -- base address, fixed!
  constant clic_size_c          : natural := 1*4; -- bytes, fixed!
  constant clic_ctrl_addr_c     : std_ulogic_vector(31 downto 0) := std_ulogic_vector(unsigned(clic_base_c) + x"00000000");

  -- Watch Dog Timer (WDT) --
  constant wdt_base_c           : std_ulogic_vector(data_width_c-1 downto 0) := x"FFFFFF8C"; -- base address, fixed!
  constant wdt_size_c           : natural := 1*4; -- bytes, fixed!
  constant wdt_ctrl_addr_c      : std_ulogic_vector(31 downto 0) := std_ulogic_vector(unsigned(wdt_base_c) + x"00000000");

  -- Machine System Timer (MTIME) --
  constant mtime_base_c         : std_ulogic_vector(data_width_c-1 downto 0) := x"FFFFFF90"; -- base address, fixed!
  constant mtime_size_c         : natural := 4*4; -- bytes, fixed!
  constant mtime_time_lo_addr_c : std_ulogic_vector(31 downto 0) := std_ulogic_vector(unsigned(mtime_base_c) + x"00000000");
  constant mtime_time_hi_addr_c : std_ulogic_vector(31 downto 0) := std_ulogic_vector(unsigned(mtime_base_c) + x"00000004");
  constant mtime_cmp_lo_addr_c  : std_ulogic_vector(31 downto 0) := std_ulogic_vector(unsigned(mtime_base_c) + x"00000008");
  constant mtime_cmp_hi_addr_c  : std_ulogic_vector(31 downto 0) := std_ulogic_vector(unsigned(mtime_base_c) + x"0000000C");

  -- Universal Asynchronous Receiver/Transmitter (UART) --
  constant uart_base_c          : std_ulogic_vector(data_width_c-1 downto 0) := x"FFFFFFA0"; -- base address, fixed!
  constant uart_size_c          : natural := 2*4; -- bytes, fixed!
  constant uart_ctrl_addr_c     : std_ulogic_vector(31 downto 0) := std_ulogic_vector(unsigned(uart_base_c) + x"00000000");
  constant uart_rtx_addr_c      : std_ulogic_vector(31 downto 0) := std_ulogic_vector(unsigned(uart_base_c) + x"00000004");

  -- Serial Peripheral Interface (SPI) --
  constant spi_base_c           : std_ulogic_vector(data_width_c-1 downto 0) := x"FFFFFFA8"; -- base address, fixed!
  constant spi_size_c           : natural := 2*4; -- bytes, fixed!
  constant spi_ctrl_addr_c      : std_ulogic_vector(31 downto 0) := std_ulogic_vector(unsigned(spi_base_c) + x"00000000");
  constant spi_rtx_addr_c       : std_ulogic_vector(31 downto 0) := std_ulogic_vector(unsigned(spi_base_c) + x"00000004");

  -- Two Wire Interface (TWI) --
  constant twi_base_c           : std_ulogic_vector(data_width_c-1 downto 0) := x"FFFFFFB0"; -- base address, fixed!
  constant twi_size_c           : natural := 2*4; -- bytes, fixed!
  constant twi_ctrl_addr_c      : std_ulogic_vector(31 downto 0) := std_ulogic_vector(unsigned(twi_base_c) + x"00000000");
  constant twi_rtx_addr_c       : std_ulogic_vector(31 downto 0) := std_ulogic_vector(unsigned(twi_base_c) + x"00000004");

  -- Pulse-Width Modulation Controller (PWM) --
  constant pwm_base_c           : std_ulogic_vector(data_width_c-1 downto 0) := x"FFFFFFB8"; -- base address, fixed!
  constant pwm_size_c           : natural := 2*4; -- bytes, fixed!
  constant pwm_ctrl_addr_c      : std_ulogic_vector(31 downto 0) := std_ulogic_vector(unsigned(pwm_base_c) + x"00000000");
  constant pwm_duty_addr_c      : std_ulogic_vector(31 downto 0) := std_ulogic_vector(unsigned(pwm_base_c) + x"00000004");

  -- True Random Number generator (TRNG) --
  constant trng_base_c          : std_ulogic_vector(data_width_c-1 downto 0) := x"FFFFFFC0"; -- base address, fixed!
  constant trng_size_c          : natural := 2*4; -- bytes, fixed!
  constant trng_ctrl_addr_c     : std_ulogic_vector(31 downto 0) := std_ulogic_vector(unsigned(trng_base_c) + x"00000000");
  constant trng_data_addr_c     : std_ulogic_vector(31 downto 0) := std_ulogic_vector(unsigned(trng_base_c) + x"00000004");

  -- RESERVED --
--constant ???_base_c           : std_ulogic_vector(data_width_c-1 downto 0) := x"FFFFFFC8"; -- base address, fixed!
--constant ???_size_c           : natural := 13*4; -- bytes, fixed!

  -- Dummy Device (with SIM output) (DEVNULL) --
  constant devnull_base_c       : std_ulogic_vector(data_width_c-1 downto 0) := x"FFFFFFFC"; -- base address, fixed!
  constant devnull_size_c       : natural := 1*4; -- bytes, fixed!
  constant devnull_data_addr_c  : std_ulogic_vector(31 downto 0) := std_ulogic_vector(unsigned(devnull_base_c) + x"00000000");

  -- Main Control Bus -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- register file --
  constant ctrl_rf_in_mux_lsb_c   : natural :=  0; -- input source select lsb (00=ALU, 01=MEM)
  constant ctrl_rf_in_mux_msb_c   : natural :=  1; -- input source select msb (10=PC,  11=CSR)
  constant ctrl_rf_rs1_adr0_c     : natural :=  2; -- source register 1 address bit 0
  constant ctrl_rf_rs1_adr1_c     : natural :=  3; -- source register 1 address bit 1
  constant ctrl_rf_rs1_adr2_c     : natural :=  4; -- source register 1 address bit 2
  constant ctrl_rf_rs1_adr3_c     : natural :=  5; -- source register 1 address bit 3
  constant ctrl_rf_rs1_adr4_c     : natural :=  6; -- source register 1 address bit 4
  constant ctrl_rf_rs2_adr0_c     : natural :=  7; -- source register 2 address bit 0
  constant ctrl_rf_rs2_adr1_c     : natural :=  8; -- source register 2 address bit 1
  constant ctrl_rf_rs2_adr2_c     : natural :=  9; -- source register 2 address bit 2
  constant ctrl_rf_rs2_adr3_c     : natural := 10; -- source register 2 address bit 3
  constant ctrl_rf_rs2_adr4_c     : natural := 11; -- source register 2 address bit 4
  constant ctrl_rf_rd_adr0_c      : natural := 12; -- destiantion register address bit 0
  constant ctrl_rf_rd_adr1_c      : natural := 13; -- destiantion register address bit 1
  constant ctrl_rf_rd_adr2_c      : natural := 14; -- destiantion register address bit 2
  constant ctrl_rf_rd_adr3_c      : natural := 15; -- destiantion register address bit 3
  constant ctrl_rf_rd_adr4_c      : natural := 16; -- destiantion register address bit 4
  constant ctrl_rf_wb_en_c        : natural := 17; -- write back enable
  constant ctrl_rf_clear_rs1_c    : natural := 18; -- force rs1=r0
  constant ctrl_rf_clear_rs2_c    : natural := 19; -- force rs2=r0
  -- alu --
  constant ctrl_alu_cmd0_c        : natural := 20; -- ALU command bit 0
  constant ctrl_alu_cmd1_c        : natural := 21; -- ALU command bit 1
  constant ctrl_alu_cmd2_c        : natural := 22; -- ALU command bit 2
  constant ctrl_alu_opa_mux_lsb_c : natural := 23; -- operand A select lsb (00=rs1, 01=PC)
  constant ctrl_alu_opa_mux_msb_c : natural := 24; -- operand A select msb (10=CSR, 11=CSR)
  constant ctrl_alu_opb_mux_lsb_c : natural := 25; -- operand B select lsb (00=rs2, 01=PC)
  constant ctrl_alu_opb_mux_msb_c : natural := 26; -- operand B select msb (10=rs1, 11=PC_increment(2/4))
  constant ctrl_alu_opc_mux_c     : natural := 27; -- operand C select (0=IMM, 1=rs2)
  constant ctrl_alu_unsigned_c    : natural := 28; -- is unsigned ALU operation
  constant ctrl_alu_shift_dir_c   : natural := 29; -- shift direction (0=left, 1=right)
  constant ctrl_alu_shift_ar_c    : natural := 30; -- is arithmetic shift
  -- bus interface --
  constant ctrl_bus_size_lsb_c    : natural := 31; -- transfer size lsb (00=byte, 01=half-word)
  constant ctrl_bus_size_msb_c    : natural := 32; -- transfer size msb (10=word, 11=?)
  constant ctrl_bus_rd_c          : natural := 33; -- read data request
  constant ctrl_bus_wr_c          : natural := 34; -- write data request
  constant ctrl_bus_if_c          : natural := 35; -- instruction fetch request (output PC, otherwise output MAR)
  constant ctrl_bus_mar_we_c      : natural := 36; -- memory address register write enable
  constant ctrl_bus_mdo_we_c      : natural := 37; -- memory data out register write enable
  constant ctrl_bus_mdi_we_c      : natural := 38; -- memory data in register write enable
  constant ctrl_bus_unsigned_c    : natural := 39; -- is unsigned load
  -- csr/system --
  constant ctrl_csr_pc_we_c       : natural := 40; -- PC write enable
  constant ctrl_csr_re_c          : natural := 41; -- valid CSR read
  constant ctrl_csr_we_c          : natural := 42; -- valid CSR write
  -- co-processor --
  constant ctrl_cp_use_c          : natural := 43; -- is cp operation
  constant ctrl_cp_id_lsb_c       : natural := 44; -- cp select lsb
  constant ctrl_cp_id_msb_c       : natural := 45; -- cp select msb
  constant ctrl_cp_cmd0_c         : natural := 46; -- cp command bit 0
  constant ctrl_cp_cmd1_c         : natural := 47; -- cp command bit 1
  constant ctrl_cp_cmd2_c         : natural := 48; -- cp command bit 2
  -- control bus size --
  constant ctrl_width_c           : natural := 49; -- control bus size

  -- ALU Comparator Bus ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant alu_cmp_equal_c : natural := 0;
  constant alu_cmp_less_c  : natural := 1; -- for signed and unsigned

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

  -- RISC-V Opcodes -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- alu --
  constant opcode_lui_c    : std_ulogic_vector(6 downto 0) := "0110111"; -- load upper immediate
  constant opcode_auipc_c  : std_ulogic_vector(6 downto 0) := "0010111"; -- add upper immediate to PC
  constant opcode_alui_c   : std_ulogic_vector(6 downto 0) := "0010011"; -- ALU operation with immediate (operation via funct3 and funct7)
  constant opcode_alu_c    : std_ulogic_vector(6 downto 0) := "0110011"; -- ALU operation (operation via funct3 and funct7)
  -- control flow --
  constant opcode_jal_c    : std_ulogic_vector(6 downto 0) := "1101111"; -- jump and link
  constant opcode_jalr_c   : std_ulogic_vector(6 downto 0) := "1100111"; -- jump and register
  constant opcode_branch_c : std_ulogic_vector(6 downto 0) := "1100011"; -- branch (condition set via funct3)
  -- memory access --
  constant opcode_load_c   : std_ulogic_vector(6 downto 0) := "0000011"; -- load (data type via funct3)
  constant opcode_store_c  : std_ulogic_vector(6 downto 0) := "0100011"; -- store (data type via funct3)
  -- system/csr --
  constant opcode_fence_c  : std_ulogic_vector(6 downto 0) := "0001111"; -- fence
  constant opcode_syscsr_c : std_ulogic_vector(6 downto 0) := "1110011"; -- system/csr access (type via funct3)

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
  constant funct3_env_c    : std_ulogic_vector(2 downto 0) := "000"; -- ecall, ebreak, mret, wfi
  constant funct3_csrrw_c  : std_ulogic_vector(2 downto 0) := "001"; -- atomic r/w
  constant funct3_csrrs_c  : std_ulogic_vector(2 downto 0) := "010"; -- atomic read & set bit
  constant funct3_csrrc_c  : std_ulogic_vector(2 downto 0) := "011"; -- atomic read & clear bit
  --
  constant funct3_csrrwi_c : std_ulogic_vector(2 downto 0) := "101"; -- atomic r/w immediate
  constant funct3_csrrsi_c : std_ulogic_vector(2 downto 0) := "110"; -- atomic read & set bit immediate
  constant funct3_csrrci_c : std_ulogic_vector(2 downto 0) := "111"; -- atomic read & clear bit immediate

  -- Co-Processor Operations ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- cp ids --
  constant cp_sel_muldiv_c : std_ulogic_vector(1 downto 0) := "00"; -- MULDIV CP
  -- muldiv cp --
  constant cp_op_mul_c    : std_ulogic_vector(2 downto 0) := "000"; -- mul
  constant cp_op_mulh_c   : std_ulogic_vector(2 downto 0) := "001"; -- mulh
  constant cp_op_mulhsu_c : std_ulogic_vector(2 downto 0) := "010"; -- mulhsu
  constant cp_op_mulhu_c  : std_ulogic_vector(2 downto 0) := "011"; -- mulhu
  constant cp_op_div_c    : std_ulogic_vector(2 downto 0) := "100"; -- div
  constant cp_op_divu_c   : std_ulogic_vector(2 downto 0) := "101"; -- divu
  constant cp_op_rem_c    : std_ulogic_vector(2 downto 0) := "110"; -- rem
  constant cp_op_remu_c   : std_ulogic_vector(2 downto 0) := "111"; -- remu

  -- ALU Function Codes ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant alu_cmd_add_c   : std_ulogic_vector(2 downto 0) := "000"; -- r <= A + B
  constant alu_cmd_sub_c   : std_ulogic_vector(2 downto 0) := "001"; -- r <= A - B
  constant alu_cmd_slt_c   : std_ulogic_vector(2 downto 0) := "010"; -- r <= A < B
  constant alu_cmd_shift_c : std_ulogic_vector(2 downto 0) := "011"; -- r <= A <</>> B
  constant alu_cmd_xor_c   : std_ulogic_vector(2 downto 0) := "100"; -- r <= A xor B
  constant alu_cmd_or_c    : std_ulogic_vector(2 downto 0) := "101"; -- r <= A or B
  constant alu_cmd_and_c   : std_ulogic_vector(2 downto 0) := "110"; -- r <= A and B
  constant alu_cmd_bitc_c  : std_ulogic_vector(2 downto 0) := "111"; -- r <= A and (not B)

  -- CPU Control Exception System -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- exception source bits --
  constant exception_iaccess_c   : natural := 0; -- instrution access fault
  constant exception_iillegal_c  : natural := 1; -- illegal instrution
  constant exception_ialign_c    : natural := 2; -- instrution address misaligned
  constant exception_m_envcall_c : natural := 3; -- ENV call from m-mode
  constant exception_break_c     : natural := 4; -- breakpoint
  constant exception_salign_c    : natural := 5; -- store address misaligned
  constant exception_lalign_c    : natural := 6; -- load address misaligned
  constant exception_saccess_c   : natural := 7; -- store access fault
  constant exception_laccess_c   : natural := 8; -- load access fault
  constant exception_width_c     : natural := 9; -- length of this list in bits
  -- interrupt source bits --
  constant interrupt_mtime_irq_c : natural := 0; -- machine timer interrupt
  constant interrupt_msw_irq_c   : natural := 1; -- machine sw interrupt
  constant interrupt_mext_irq_c  : natural := 2; -- machine external interrupt
  constant interrupt_width_c     : natural := 3; -- length of this list in bits

  -- Clock Generator -------------------------------------------------------------------------
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
      MEM_ISPACE_SIZE           : natural := 16*1024; -- total size of instruction memory space in byte
      MEM_INT_IMEM_USE          : boolean := true;    -- implement processor-internal instruction memory
      MEM_INT_IMEM_SIZE         : natural := 16*1024; -- size of processor-internal instruction memory in bytes
      MEM_INT_IMEM_ROM          : boolean := false;   -- implement processor-internal instruction memory as ROM
      -- Memory configuration: Data memory --
      MEM_DSPACE_BASE           : std_ulogic_vector(31 downto 0) := x"80000000"; -- base address of data memory space
      MEM_DSPACE_SIZE           : natural := 8*1024; -- total size of data memory space in byte
      MEM_INT_DMEM_USE          : boolean := true;   -- implement processor-internal data memory
      MEM_INT_DMEM_SIZE         : natural := 8*1024; -- size of processor-internal data memory in bytes
      -- Memory configuration: External memory interface --
      MEM_EXT_USE               : boolean := false;  -- implement external memory bus interface?
      MEM_EXT_REG_STAGES        : natural := 2;      -- number of interface register stages (0,1,2)
      MEM_EXT_TIMEOUT           : natural := 15;     -- cycles after which a valid bus access will timeout (>=1)
      -- Processor peripherals --
      IO_GPIO_USE               : boolean := true;   -- implement general purpose input/output port unit (GPIO)?
      IO_MTIME_USE              : boolean := true;   -- implement machine system timer (MTIME)?
      IO_UART_USE               : boolean := true;   -- implement universal asynchronous receiver/transmitter (UART)?
      IO_SPI_USE                : boolean := true;   -- implement serial peripheral interface (SPI)?
      IO_TWI_USE                : boolean := true;   -- implement two-wire interface (TWI)?
      IO_PWM_USE                : boolean := true;   -- implement pulse-width modulation unit (PWM)?
      IO_WDT_USE                : boolean := true;   -- implement watch dog timer (WDT)?
      IO_CLIC_USE               : boolean := true;   -- implement core local interrupt controller (CLIC)?
      IO_TRNG_USE               : boolean := false;  -- implement true random number generator (TRNG)?
      IO_DEVNULL_USE            : boolean := true    -- implement dummy device (DEVNULL)?
    );
    port (
      -- Global control --
      clk_i      : in  std_ulogic := '0'; -- global clock, rising edge
      rstn_i     : in  std_ulogic := '0'; -- global reset, low-active, async
      -- Wishbone bus interface --
      wb_adr_o   : out std_ulogic_vector(31 downto 0); -- address
      wb_dat_i   : in  std_ulogic_vector(31 downto 0) := (others => '0'); -- read data
      wb_dat_o   : out std_ulogic_vector(31 downto 0); -- write data
      wb_we_o    : out std_ulogic; -- read/write
      wb_sel_o   : out std_ulogic_vector(03 downto 0); -- byte enable
      wb_stb_o   : out std_ulogic; -- strobe
      wb_cyc_o   : out std_ulogic; -- valid cycle
      wb_ack_i   : in  std_ulogic := '0'; -- transfer acknowledge
      wb_err_i   : in  std_ulogic := '0'; -- transfer error
      -- GPIO --
      gpio_o     : out std_ulogic_vector(15 downto 0); -- parallel output
      gpio_i     : in  std_ulogic_vector(15 downto 0) := (others => '0'); -- parallel input
      -- UART --
      uart_txd_o : out std_ulogic; -- UART send data
      uart_rxd_i : in  std_ulogic := '0'; -- UART receive data
      -- SPI --
      spi_sclk_o : out std_ulogic; -- serial clock line
      spi_mosi_o : out std_ulogic; -- serial data line out
      spi_miso_i : in  std_ulogic := '0'; -- serial data line in
      spi_csn_o  : out std_ulogic_vector(07 downto 0); -- SPI CS
      -- TWI --
      twi_sda_io : inout std_logic := 'H'; -- twi serial data line
      twi_scl_io : inout std_logic := 'H'; -- twi serial clock line
      -- PWM --
      pwm_o      : out std_ulogic_vector(03 downto 0);  -- pwm channels
      -- Interrupts --
      ext_irq_i  : in  std_ulogic_vector(01 downto 0) := (others => '0'); -- external interrupt request
      ext_ack_o  : out std_ulogic_vector(01 downto 0)  -- external interrupt request acknowledge
    );
  end component;

  -- Component: CPU Control -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_cpu_control
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
      MEM_ISPACE_SIZE           : natural := 16*1024; -- total size of instruction memory space in byte
      MEM_INT_IMEM_USE          : boolean := true;   -- implement processor-internal instruction memory
      MEM_INT_IMEM_SIZE         : natural := 16*1024; -- size of processor-internal instruction memory in bytes
      MEM_INT_IMEM_ROM          : boolean := false;  -- implement processor-internal instruction memory as ROM
      -- Memory configuration: Data memory --
      MEM_DSPACE_BASE           : std_ulogic_vector(31 downto 0) := x"80000000"; -- base address of data memory space
      MEM_DSPACE_SIZE           : natural := 8*1024; -- total size of data memory space in byte
      MEM_INT_DMEM_USE          : boolean := true;   -- implement processor-internal data memory
      MEM_INT_DMEM_SIZE         : natural := 8*1024; -- size of processor-internal data memory in bytes
      -- Memory configuration: External memory interface --
      MEM_EXT_USE               : boolean := false;  -- implement external memory bus interface?
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
      clk_i         : in  std_ulogic; -- global clock, rising edge
      rstn_i        : in  std_ulogic; -- global reset, low-active, async
      ctrl_o        : out std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
      -- status input --
      alu_wait_i    : in  std_ulogic; -- wait for ALU
      bus_wait_i    : in  std_ulogic; -- wait for bus
      -- data input --
      instr_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- instruction
      cmp_i         : in  std_ulogic_vector(1 downto 0); -- comparator status
      alu_add_i     : in  std_ulogic_vector(data_width_c-1 downto 0); -- ALU.add result
      -- data output --
      imm_o         : out std_ulogic_vector(data_width_c-1 downto 0); -- immediate
      pc_o          : out std_ulogic_vector(data_width_c-1 downto 0); -- current PC
      alu_pc_o      : out std_ulogic_vector(data_width_c-1 downto 0); -- delayed PC for ALU
      -- csr interface --
      csr_wdata_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- CSR write data
      csr_rdata_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- CSR read data
      -- external interrupt --
      clic_irq_i    : in  std_ulogic; -- CLIC interrupt request
      mtime_irq_i   : in  std_ulogic; -- machine timer interrupt
      -- bus access exceptions --
      mar_i         : in  std_ulogic_vector(data_width_c-1 downto 0); -- memory address register
      ma_instr_i    : in  std_ulogic; -- misaligned instruction address
      ma_load_i     : in  std_ulogic; -- misaligned load data address
      ma_store_i    : in  std_ulogic; -- misaligned store data address
      be_instr_i    : in  std_ulogic; -- bus error on instruction access
      be_load_i     : in  std_ulogic; -- bus error on load data access
      be_store_i    : in  std_ulogic; -- bus error on store data access
      bus_exc_ack_o : out std_ulogic  -- bus exception error acknowledge
    );
  end component;

  -- Component: CPU Register File -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_cpu_regfile
    generic (
      CPU_EXTENSION_RISCV_E : boolean := false -- implement embedded RF extension?
    );
    port (
      -- global control --
      clk_i  : in  std_ulogic; -- global clock, rising edge
      ctrl_i : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
      -- data input --
      mem_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- memory read data
      alu_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- ALU result
      csr_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- CSR read data
      pc_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- current pc
      -- data output --
      rs1_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- operand 1
      rs2_o  : out std_ulogic_vector(data_width_c-1 downto 0)  -- operand 2
    );
  end component;

  -- Component: CPU ALU ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_cpu_alu
    generic (
      CPU_EXTENSION_RISCV_C : boolean := false; -- implement compressed extension?
      CPU_EXTENSION_RISCV_M : boolean := false  -- implement mul/div extension?
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
      pc2_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- delayed PC
      imm_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- immediate
      csr_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- csr read data
      -- data output --
      cmp_o       : out std_ulogic_vector(1 downto 0); -- comparator status
      add_o       : out std_ulogic_vector(data_width_c-1 downto 0); -- OPA + OPB
      res_o       : out std_ulogic_vector(data_width_c-1 downto 0); -- ALU result
      -- co-processor interface --
      cp0_data_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- co-processor 0 result
      cp0_valid_i : in  std_ulogic; -- co-processor 0 result valid
      cp1_data_i  : in  std_ulogic_vector(data_width_c-1 downto 0); -- co-processor 1 result
      cp1_valid_i : in  std_ulogic; -- co-processor 1 result valid
      -- status --
      wait_o      : out std_ulogic -- busy due to iterative processing units
    );
  end component;

  -- Component: CPU Co-Processor MULDIV -----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_cpu_cp_muldiv
    port (
      -- global control --
      clk_i   : in  std_ulogic; -- global clock, rising edge
      rstn_i  : in  std_ulogic; -- global reset, low-active, async
      ctrl_i  : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
      -- data input --
      rs1_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 1
      rs2_i   : in  std_ulogic_vector(data_width_c-1 downto 0); -- rf source 2
      -- result and status --
      res_o   : out std_ulogic_vector(data_width_c-1 downto 0); -- operation result
      valid_o : out std_ulogic -- data output valid
    );
  end component;

  -- Component: CPU Bus Interface -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_cpu_bus
    generic (
      CPU_EXTENSION_RISCV_C : boolean := false; -- implement compressed extension?
      MEM_EXT_TIMEOUT       : natural := 15     -- cycles after which a valid bus access will timeout
    );
    port (
      -- global control --
      clk_i       : in  std_ulogic; -- global clock, rising edge
      rstn_i      : in  std_ulogic; -- global reset, low-active, async
      ctrl_i      : in  std_ulogic_vector(ctrl_width_c-1 downto 0); -- main control bus
      -- data input --
      wdata_i     : in  std_ulogic_vector(data_width_c-1 downto 0); -- write data
      pc_i        : in  std_ulogic_vector(data_width_c-1 downto 0); -- current PC
      alu_i       : in  std_ulogic_vector(data_width_c-1 downto 0); -- ALU result
      -- data output --
      instr_o     : out std_ulogic_vector(data_width_c-1 downto 0); -- instruction
      rdata_o     : out std_ulogic_vector(data_width_c-1 downto 0); -- read data
      -- status --
      mar_o       : out std_ulogic_vector(data_width_c-1 downto 0); -- current memory address register
      ma_instr_o  : out std_ulogic; -- misaligned instruction address
      ma_load_o   : out std_ulogic; -- misaligned load data address
      ma_store_o  : out std_ulogic; -- misaligned store data address
      be_instr_o  : out std_ulogic; -- bus error on instruction access
      be_load_o   : out std_ulogic; -- bus error on load data access
      be_store_o  : out std_ulogic; -- bus error on store data 
      bus_wait_o  : out std_ulogic; -- wait for bus operation to finish
      exc_ack_i   : in  std_ulogic; -- exception controller ACK
      -- bus system --
      bus_addr_o  : out std_ulogic_vector(data_width_c-1 downto 0); -- bus access address
      bus_rdata_i : in  std_ulogic_vector(data_width_c-1 downto 0); -- bus read data
      bus_wdata_o : out std_ulogic_vector(data_width_c-1 downto 0); -- bus write data
      bus_ben_o   : out std_ulogic_vector(03 downto 0); -- byte enable
      bus_we_o    : out std_ulogic; -- write enable
      bus_re_o    : out std_ulogic; -- read enable
      bus_ack_i   : in  std_ulogic; -- bus transfer acknowledge
      bus_err_i   : in  std_ulogic  -- bus transfer error
    );
  end component;

  -- Component: CPU Compressed Instructions Decompressor ------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_cpu_decompressor
    port (
      -- instruction input --
      ci_instr16_i : in  std_ulogic_vector(15 downto 0); -- compressed instruction input
      -- instruction output --
      ci_valid_o   : out std_ulogic; -- is a compressed instruction
      ci_illegal_o : out std_ulogic; -- is an illegal compressed instruction
      ci_instr32_o : out std_ulogic_vector(31 downto 0)  -- 32-bit decompressed instruction
    );
  end component;

  -- Component: CPU Top Entity --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_cpu
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
  end component;

  -- Component: Processor-internal instruction memory (IMEM) --------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_imem
    generic (
      IMEM_BASE      : std_ulogic_vector(31 downto 0) := x"00000000"; -- memory base address
      IMEM_SIZE      : natural := 4*1024; -- processor-internal instruction memory size in bytes
      IMEM_AS_ROM    : boolean := false;  -- implement IMEM as read-only memory?
      BOOTLOADER_USE : boolean := true    -- implement and use bootloader?
    );
    port (
      clk_i  : in  std_ulogic; -- global clock line
      rden_i : in  std_ulogic; -- read enable
      wren_i : in  std_ulogic; -- write enable
      ben_i  : in  std_ulogic_vector(03 downto 0); -- byte write enable
      upen_i : in  std_ulogic; -- update enable
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
      DMEM_BASE : std_ulogic_vector(31 downto 0) := x"80000000"; -- memory base address
      DMEM_SIZE : natural := 4*1024  -- processor-internal instruction memory size in bytes
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
      clk_i     : in  std_ulogic; -- global clock line
      addr_i    : in  std_ulogic_vector(31 downto 0); -- address
      rden_i    : in  std_ulogic; -- read enable
      wren_i    : in  std_ulogic; -- write enable
      ben_i     : in  std_ulogic_vector(03 downto 0); -- byte write enable
      data_i    : in  std_ulogic_vector(31 downto 0); -- data in
      data_o    : out std_ulogic_vector(31 downto 0); -- data out
      ack_o     : out std_ulogic; -- transfer acknowledge
      -- interrupt --
      irq_o     : out std_ulogic  -- interrupt request
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
      ben_i  : in  std_ulogic_vector(03 downto 0); -- byte write enable
      data_i : in  std_ulogic_vector(31 downto 0); -- data in
      data_o : out std_ulogic_vector(31 downto 0); -- data out
      ack_o  : out std_ulogic; -- transfer acknowledge
      -- parallel io --
      gpio_o : out std_ulogic_vector(15 downto 0);
      gpio_i : in  std_ulogic_vector(15 downto 0);
      -- interrupt --
      irq_o  : out std_ulogic
    );
  end component;

  -- Component: Core Local Interrupt Controller (CLIC) --------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_clic
    port (
      -- host access --
      clk_i     : in  std_ulogic; -- global clock line
      addr_i    : in  std_ulogic_vector(31 downto 0); -- address
      rden_i    : in  std_ulogic; -- read enable
      wren_i    : in  std_ulogic; -- write enable
      ben_i     : in  std_ulogic_vector(03 downto 0); -- byte write enable
      data_i    : in  std_ulogic_vector(31 downto 0); -- data in
      data_o    : out std_ulogic_vector(31 downto 0); -- data out
      ack_o     : out std_ulogic; -- transfer acknowledge
      -- cpu interrupt --
      cpu_irq_o : out std_ulogic; -- trigger CPU's external IRQ
      -- external interrupt lines --
      ext_irq_i : in  std_ulogic_vector(07 downto 0); -- IRQ, triggering on HIGH level
      ext_ack_o : out std_ulogic_vector(07 downto 0)  -- acknowledge
    );
  end component;

  -- Component: Watchdog Timer (WDT) --------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_wdt
    port (
      -- host access --
      clk_i       : in  std_ulogic; -- global clock line
      rstn_i      : in  std_ulogic; -- global reset line, low-active
      rden_i      : in  std_ulogic; -- read enable
      wren_i      : in  std_ulogic; -- write enable
      ben_i       : in  std_ulogic_vector(03 downto 0); -- byte write enable
      addr_i      : in  std_ulogic_vector(31 downto 0); -- address
      data_i      : in  std_ulogic_vector(31 downto 0); -- data in
      data_o      : out std_ulogic_vector(31 downto 0); -- data out
      ack_o       : out std_ulogic; -- transfer acknowledge
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
    port (
      -- host access --
      clk_i       : in  std_ulogic; -- global clock line
      addr_i      : in  std_ulogic_vector(31 downto 0); -- address
      rden_i      : in  std_ulogic; -- read enable
      wren_i      : in  std_ulogic; -- write enable
      ben_i       : in  std_ulogic_vector(03 downto 0); -- byte write enable
      data_i      : in  std_ulogic_vector(31 downto 0); -- data in
      data_o      : out std_ulogic_vector(31 downto 0); -- data out
      ack_o       : out std_ulogic; -- transfer acknowledge
      -- clock generator --
      clkgen_en_o : out std_ulogic; -- enable clock generator
      clkgen_i    : in  std_ulogic_vector(07 downto 0);
      -- com lines --
      uart_txd_o  : out std_ulogic;
      uart_rxd_i  : in  std_ulogic;
      -- interrupts --
      uart_irq_o  : out std_ulogic  -- uart rx/tx interrupt
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
      ben_i       : in  std_ulogic_vector(03 downto 0); -- byte write enable
      data_i      : in  std_ulogic_vector(31 downto 0); -- data in
      data_o      : out std_ulogic_vector(31 downto 0); -- data out
      ack_o       : out std_ulogic; -- transfer acknowledge
      -- clock generator --
      clkgen_en_o : out std_ulogic; -- enable clock generator
      clkgen_i    : in  std_ulogic_vector(07 downto 0);
      -- com lines --
      spi_sclk_o  : out std_ulogic; -- SPI serial clock
      spi_mosi_o  : out std_ulogic; -- SPI master out, slave in
      spi_miso_i  : in  std_ulogic; -- SPI master in, slave out
      spi_csn_o   : out std_ulogic_vector(07 downto 0); -- SPI CS
      -- interrupt --
      spi_irq_o   : out std_ulogic -- transmission done interrupt
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
      ben_i       : in  std_ulogic_vector(03 downto 0); -- byte write enable
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
      twi_irq_o   : out std_ulogic -- transfer done IRQ
    );
  end component;

  -- Component: Pulse-Width Modulation Controller (PWM) -------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_pwm
    port (
      -- host access --
      clk_i       : in  std_ulogic; -- global clock line
      addr_i      : in  std_ulogic_vector(31 downto 0); -- address
      rden_i      : in  std_ulogic; -- read enable
      wren_i      : in  std_ulogic; -- write enable
      ben_i       : in  std_ulogic_vector(03 downto 0); -- byte write enable
      data_i      : in  std_ulogic_vector(31 downto 0); -- data in
      data_o      : out std_ulogic_vector(31 downto 0); -- data out
      ack_o       : out std_ulogic; -- transfer acknowledge
      -- clock generator --
      clkgen_en_o : out std_ulogic; -- enable clock generator
      clkgen_i    : in  std_ulogic_vector(07 downto 0);
      -- pwm output channels --
      pwm_o       : out std_ulogic_vector(03 downto 0)
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
      ben_i  : in  std_ulogic_vector(03 downto 0); -- byte write enable
      data_i : in  std_ulogic_vector(31 downto 0); -- data in
      data_o : out std_ulogic_vector(31 downto 0); -- data out
      ack_o  : out std_ulogic  -- transfer acknowledge
    );
  end component;

  -- Component: Wishbone Bus Gateway (WISHBONE) ---------------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_wishbone
    generic (
      INTERFACE_REG_STAGES : natural := 2; -- number of interface register stages (0,1,2)
      -- Memory configuration: Instruction memory --
      MEM_ISPACE_BASE      : std_ulogic_vector(31 downto 0) := x"00000000"; -- base address of instruction memory space
      MEM_ISPACE_SIZE      : natural := 8*1024; -- total size of instruction memory space in byte
      MEM_INT_IMEM_USE     : boolean := true;   -- implement processor-internal instruction memory
      MEM_INT_IMEM_SIZE    : natural := 8*1024; -- size of processor-internal instruction memory in bytes
      -- Memory configuration: Data memory --
      MEM_DSPACE_BASE      : std_ulogic_vector(31 downto 0) := x"80000000"; -- base address of data memory space
      MEM_DSPACE_SIZE      : natural := 4*1024; -- total size of data memory space in byte
      MEM_INT_DMEM_USE     : boolean := true;   -- implement processor-internal data memory
      MEM_INT_DMEM_SIZE    : natural := 4*1024  -- size of processor-internal data memory in bytes
    );
    port (
      -- global control --
      clk_i    : in  std_ulogic; -- global clock line
      rstn_i   : in  std_ulogic; -- global reset line, low-active
      -- host access --
      addr_i   : in  std_ulogic_vector(31 downto 0); -- address
      rden_i   : in  std_ulogic; -- read enable
      wren_i   : in  std_ulogic; -- write enable
      ben_i    : in  std_ulogic_vector(03 downto 0); -- byte write enable
      data_i   : in  std_ulogic_vector(31 downto 0); -- data in
      data_o   : out std_ulogic_vector(31 downto 0); -- data out
      ack_o    : out std_ulogic; -- transfer acknowledge
      err_o    : out std_ulogic; -- transfer error
      -- wishbone interface --
      wb_adr_o : out std_ulogic_vector(31 downto 0); -- address
      wb_dat_i : in  std_ulogic_vector(31 downto 0); -- read data
      wb_dat_o : out std_ulogic_vector(31 downto 0); -- write data
      wb_we_o  : out std_ulogic; -- read/write
      wb_sel_o : out std_ulogic_vector(03 downto 0); -- byte enable
      wb_stb_o : out std_ulogic; -- strobe
      wb_cyc_o : out std_ulogic; -- valid cycle
      wb_ack_i : in  std_ulogic; -- transfer acknowledge
      wb_err_i : in  std_ulogic  -- transfer error
    );
  end component;

  -- Component: Dummy Device with SIM Output (DEVNULL) -------------------------------------
  -- -------------------------------------------------------------------------------------------
  component neorv32_devnull
    port (
      -- host access --
      clk_i  : in  std_ulogic; -- global clock line
      addr_i : in  std_ulogic_vector(31 downto 0); -- address
      rden_i : in  std_ulogic; -- read enable
      wren_i : in  std_ulogic; -- write enable
      ben_i  : in  std_ulogic_vector(03 downto 0); -- byte write enable
      data_i : in  std_ulogic_vector(31 downto 0); -- data in
      data_o : out std_ulogic_vector(31 downto 0); -- data out
      ack_o  : out std_ulogic  -- transfer acknowledge
    );
  end component;

end neorv32_package;

package body neorv32_package is

  -- Function: Minimal required bit width ---------------------------------------------------
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

  -- Function: Convert BOOL to STD_ULOGIC ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function bool_to_ulogic_f(cond : boolean) return std_ulogic is
  begin
    if (cond = true) then
      return '1';
    else
      return '0';
    end if;
  end function bool_to_ulogic_f;

  -- Function: OR all bits ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function or_all_f(a : std_ulogic_vector) return std_ulogic is
    variable tmp_v : std_ulogic;
  begin
    tmp_v := a(a'low);
    for i in a'low+1 to a'high loop
      tmp_v := tmp_v or a(i);
    end loop; -- i
    return tmp_v;
  end function or_all_f;

  -- Function: AND all bits -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function and_all_f(a : std_ulogic_vector) return std_ulogic is
    variable tmp_v : std_ulogic;
  begin
    tmp_v := a(a'low);
    for i in a'low+1 to a'high loop
      tmp_v := tmp_v and a(i);
    end loop; -- i
    return tmp_v;
  end function and_all_f;

  -- Function: XOR all bits -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function xor_all_f(a : std_ulogic_vector) return std_ulogic is
    variable tmp_v : std_ulogic;
  begin
    tmp_v := a(a'low);
    for i in a'low+1 to a'high loop
      tmp_v := tmp_v xor a(i);
    end loop; -- i
    return tmp_v;
  end function xor_all_f;

  -- Function: XNOR all bits ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function xnor_all_f(a : std_ulogic_vector) return std_ulogic is
    variable tmp_v : std_ulogic;
  begin
    tmp_v := a(a'low);
    for i in a'low+1 to a'high loop
      tmp_v := tmp_v xnor a(i);
    end loop; -- i
    return tmp_v;
  end function xnor_all_f;

end neorv32_package;
