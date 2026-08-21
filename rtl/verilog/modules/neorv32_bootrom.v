// ================================================================================ //
// NEORV32 - Bootloader ROM (BOOTROM) - Verilog Version                             //
// -------------------------------------------------------------------------------- //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

module neorv32_bootrom (
  // global control
  input         clk_i,      // clock, trigger on rising edge
  input         rstn_i,     // async reset, low-active
  // bus request
  input  [15:0] req_addr_i, // access address (byte-addressing)
  input   [3:0] req_ben_i,  // byte enable
  input         req_stb_i,  // request strobe
  input         req_rw_i,   // 0 = read, 1 = write
  // bus response
  output [13:0] rsp_data_o, // read data
  output        rsp_ack_o,  // access acknowledge
  output        rsp_err_o   // access error
);

  localparam ROM_AWIDTH = 10; // ACTUAL address width (words) of bootloader image
  reg [31:0] rom [2**ROM_AWIDTH-1:0];
  reg [31:0] rdata;
  reg        ack;

  // Memory Access --------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  initial begin // initialize from default HEX file (needs to be built first!)
    $readmemh("../../sw/bootloader/neorv32_raw_exe.hex", rom, 0, 2**ROM_AWIDTH-1);
  end

  always @(posedge clk_i) begin
    if (req_stb_i == 1'b1) begin
      rdata <= rom[req_addr_i[ROM_AWIDTH-1:2]];
    end
  end

  // Bus Handshake --------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  always @(posedge clk_i or negedge rstn_i) begin
    if (rstn_i == 1'b0) begin
      ack <= 1'b0;
    end else begin
      ack <= req_stb_i & ~req_rw_i;
    end
  end

  assign rsp_data_o = (ack == 1'b1) ? rdata : 32'h00000000;
  assign rsp_ack_o  = ack;
  assign rsp_err_o  = 1'b0; // no access errors supported (could be used for ECC / parity checks)

endmodule
