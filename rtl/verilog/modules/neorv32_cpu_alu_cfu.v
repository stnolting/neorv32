// ================================================================================ //
// NEORV32 - ALU Custom Functions Unit (CFU) - Verilog Version                      //
// -------------------------------------------------------------------------------- //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

module neorv32_cpu_alu_cfu (
  // global control
  input  wire        clk_i,    // global clock, rising edge
  input  wire        rstn_i,   // global reset, low-active, async
  // request
  input  wire        start_i,  // start trigger, single-shot
  input  wire [31:0] inst_i,   // full instruction word
  input  wire [31:0] rs1_i,    // register source operand 1
  input  wire [31:0] rs2_i,    // register source operand 2
  // response
  output wire [31:0] result_o, // operation result
  output wire        valid_o   // operation done; result valid
);

  // supported CFU opcodes
  localparam [6:0] opcode_custom0_c = 7'b0001011; // CUSTOM-0 opcode
  localparam [6:0] opcode_custom1_c = 7'b0101011; // CUSTOM-1 opcode
  localparam [6:0] opcode_op32_c    = 7'b0011011; // OP-32 opcode
  localparam [6:0] opcode_opimm32_c = 7'b0111011; // OP-IMM-32 opcode

  // **********************************************************
  // CFU Example: XTEA - Extended Tiny Encryption Algorithm
  // **********************************************************

  // instruction types (opcode field)
  localparam [6:0] xtea_r_type_c = opcode_custom0_c; // XTEA R-type instructions
  localparam [6:0] xtea_i_type_c = opcode_custom1_c; // XTEA I-type instructions

  // instruction identifiers (funct3 bit-field)
  localparam [2:0] xtea_enc_v0_c = 3'b000;
  localparam [2:0] xtea_enc_v1_c = 3'b001;
  localparam [2:0] xtea_dec_v0_c = 3'b010;
  localparam [2:0] xtea_dec_v1_c = 3'b011;
  localparam [2:0] xtea_init_c   = 3'b100;

  // instruction decoder
  wire start; // start valid CFU instruction
  wire itype; // XTEA instruction type (0 = r-type, 1 = i-type)
  wire  [2:0] funct3; // i-type/r-type function select
  wire [11:0] imm12; // i-type immediate

  // round-key update
  localparam [31:0] xtea_delta_c = 32'h9e3779b9;

  // key storage (accessed via special r4-type instructions)
  reg [31:0] key_mem [0:3];

  // processing state
  reg [1:0]  xtea_done; // multi-cycle done shift register; 2 stages = 2 cycles latency
  reg [31:0] xtea_opa; // input operand a
  reg [31:0] xtea_opb; // input operand b
  reg [31:0] xtea_sum; // round key buffer
  reg [31:0] xtea_res; // operation results

  // helpers
  wire [31:0] tmp_a, tmp_b, tmp_x, tmp_y, tmp_z, tmp_r;
  reg [31:0] result;
  reg valid;

  // XTEA Instruction Decode ----------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  assign start  = ((inst_i[6:0] == xtea_r_type_c) || (inst_i[6:0] == xtea_i_type_c)) ? start_i : 1'b0; // valid instruction?
  assign itype  = (inst_i[6:0] == xtea_r_type_c) ? 1'b0 : 1'b1; // XTEA r-type or i-type?
  assign funct3 = inst_i[14:12]; // type function select
  assign imm12  = inst_i[31:20]; // i-type 12-bit immediate

  // XTEA Processing Core ------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  always @(posedge clk_i or negedge rstn_i) begin
    if (rstn_i == 1'b0) begin
      xtea_done  <= 2'b00;
      xtea_opa   <= 32'b0;
      xtea_opb   <= 32'b0;
      xtea_sum   <= 32'b0;
      key_mem[0] <= 32'b0;
      key_mem[1] <= 32'b0;
      key_mem[2] <= 32'b0;
      key_mem[3] <= 32'b0;
    end else begin
      // "operation-done" shift register: module has 2 cycles latency
      xtea_done[0] <= 1'b0;      // default: no operation trigger
      xtea_done[1] <= xtea_done[0];  // arbitration shift register

      // trigger new operation
      if (start == 1'b1) begin
        if (itype == 1'b0) begin // R-type for computational instructions
          xtea_opa     <= rs1_i; // buffer input operand rs1
          xtea_opb     <= rs2_i; // buffer input operand rs2
          xtea_done[0] <= 1'b1;  // start data processing
        end else begin // I-type is used for key access instructions
          if (funct3[0] == 1'b1) begin // key write-enable
            key_mem[imm12[1:0]] <= rs1_i; // write key data at imm12(1:0)
          end
        end
      end

      // data processing
      if (xtea_done[0] == 1'b1) begin // second-stage execution trigger
        // update "sum" round key --
        if (funct3[2] == 1'b1) begin // initialize
          xtea_sum <= xtea_opa; // set initial round key
        end else if (funct3[1:0] == xtea_enc_v0_c[1:0]) begin // encrypt v0
          xtea_sum <= xtea_sum + xtea_delta_c;
        end else if (funct3[1:0] == xtea_dec_v1_c[1:0]) begin // decrypt v1
          xtea_sum <= xtea_sum - xtea_delta_c;
        end
        // process "v" operands
        if (funct3[1] == 1'b0) begin// encrypt
          xtea_res <= tmp_b + tmp_r;
        end else begin // decrypt
          xtea_res <= tmp_b - tmp_r;
        end
      end
    end
  end

  // helpers
  assign tmp_a = (funct3[0] == 1'b0) ? xtea_opb : xtea_opa; // v1 / v0 select
  assign tmp_b = (funct3[0] == 1'b0) ? xtea_opa : xtea_opb; // v0 / v1 select
  assign tmp_x = (funct3[0] == 1'b0) ? {xtea_opb[27:0], 4'b0000} : {xtea_opa[27:0], 4'b0000};   // v << 4
  assign tmp_y = (funct3[0] == 1'b0) ? {5'b00000, xtea_opb[31:5]} : {5'b00000, xtea_opa[31:5]}; // v >> 5
  assign tmp_z = (funct3[0] == 1'b0) ? key_mem[xtea_sum[1:0]]    // key[sum & 3]
                                     : key_mem[xtea_sum[12:11]]; // key[(sum >> 11) & 3]
  assign tmp_r = ((tmp_x ^ tmp_y) + tmp_a) ^ (xtea_sum + tmp_z);

  // Function Result Select -----------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  always @(*) begin
    // no need for a register stage here; the CFU output is registered inside the ALU module anyway
    if (itype == 1'b0) begin // R-type instructions; function select via "funct3"
      case (funct3) // just check "funct3" here
        xtea_enc_v0_c, xtea_enc_v1_c, xtea_dec_v0_c, xtea_dec_v1_c: begin // encryption/decryption
          result = xtea_res; // processing result
          valid  = xtea_done[1]; // multi-cycle processing done when set
        end
        xtea_init_c: begin // xtea initialization
          result = 32'b0; // just output zero
          valid  = 1'b1;  // pure-combinatorial, so we are done "immediately"
        end
        default: begin // all unspecified operations
          result = 32'b0; // no logic implemented
          valid  = 1'b0;  // this will cause an illegal instruction exception
        end
      endcase
    end else begin // I-type instructions; used for key access
      result = key_mem[imm12[1:0]];
      valid  = 1'b1; // pure-combinatorial, so we are done "immediately"
    end
  end

  assign result_o = result;
  assign valid_o  = valid;

endmodule
