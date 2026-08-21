// ================================================================================ //
// NEORV32 - Data Memory (DMEM) - Verilog Version                                   //
// -------------------------------------------------------------------------------- //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

module neorv32_dmem #(
  parameter AWIDTH = 8, // memory address width (byte-addressing
  parameter OUTREG = 0  // add additional output register when 1
)(
  // global control
  input         clk_i,      // clock, trigger on rising edge
  input         rstn_i,     // async reset, low-active
  // bus request
  input  [31:0] req_addr_i, // access address (byte-addressing)
  input  [31:0] req_data_i, // write data
  input   [3:0] req_ben_i,  // byte enable
  input         req_stb_i,  // request strobe
  input         req_rw_i,   // 0 = read, 1 = write
  // bus response
  output [31:0] rsp_data_o, // read data
  output        rsp_ack_o,  // access acknowledge
  output        rsp_err_o   // access error
);

  // we have 4 parallel byte-wide RAMs; so we need to access them using the WORD address
  localparam RAM_AWIDTH = AWIDTH-2;
  wire [RAM_AWIDTH-1:0] addr = req_addr_i[AWIDTH-1:2];

  reg  [31:0] rdata_mem;
  wire [31:0] rdata;
  wire  [3:0] en;
  reg   [1:0] ack;

  // Memory Access --------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  // the RAM is split into 4x 8-bit RAMs as some FPGA synthesis tools have issues
  // inferring a 32-bit RAM with individual byte-enables; they also seem to have
  // issues with multi-dimensional RAMs; i.e. [4][2**AWIDTH][8]
  reg [7:0] ram_b0 [0:2**RAM_AWIDTH-1];
  reg [7:0] ram_b1 [0:2**RAM_AWIDTH-1];
  reg [7:0] ram_b2 [0:2**RAM_AWIDTH-1];
  reg [7:0] ram_b3 [0:2**RAM_AWIDTH-1];

  always @(posedge clk_i) begin
    if (en[0] == 1'b1) begin // byte 0
      if (req_rw_i == 1'b1) begin
        ram_b0[addr] <= req_data_i[7:0];
      end
      rdata_mem[7:0] <= ram_b0[addr];
    end
    if (en[1] == 1'b1) begin // byte 1
      if (req_rw_i == 1'b1) begin
        ram_b1[addr] <= req_data_i[15:8];
      end
      rdata_mem[15:8] <= ram_b1[addr];
    end
    if (en[2] == 1'b1) begin // byte 2
      if (req_rw_i == 1'b1) begin
        ram_b2[addr] <= req_data_i[23:16];
      end
      rdata_mem[23:16] <= ram_b2[addr];
    end
    if (en[3] == 1'b1) begin // byte 3
      if (req_rw_i == 1'b1) begin
        ram_b3[addr] <= req_data_i[31:24];
      end
      rdata_mem[31:24] <= ram_b3[addr];
    end
  end

  // access enable
  assign en = (req_stb_i == 1'b1) ? req_ben_i : 4'b0000;

  // Optional Output Register ---------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  generate
    if (OUTREG != 0) begin
      reg [31:0] rdata_reg;
      always @(posedge clk_i) begin
        rdata_reg <= rdata_mem;
      end
      assign rdata = rdata_reg;
    end else begin
      assign rdata = rdata_mem;
    end
  endgenerate

  // Bus Handshake --------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  always @(posedge clk_i or negedge rstn_i) begin
    if (rstn_i == 1'b0) begin
      ack <= 2'b00;
    end else begin
      ack <= {ack[0], req_stb_i & ~req_rw_i};
    end
  end

  generate
    if (OUTREG != 0) begin
      assign rsp_data_o = (ack[1] == 1'b1) ? rdata : 32'h00000000;
      assign rsp_ack_o  = ack[1];
    end else begin
      assign rsp_data_o = (ack[0] == 1'b1) ? rdata : 32'h00000000;
      assign rsp_ack_o  = ack[0];
    end
  endgenerate
  assign rsp_err_o = 1'b0; // no access errors supported (could be used for ECC / parity checks)

endmodule
