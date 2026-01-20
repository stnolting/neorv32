// ================================================================================ //
// NEORV32 - Data Memory (RAM Primitive) - Verilog IP Module                        //
// -------------------------------------------------------------------------------- //
// Generic single-port RAM description; uses individual byte-wide RAMs for better   //
// FPGA mapping results. The read-during-write behavior is irrelevant as read and   //
// write accesses are mutually exclusive.                                           //
// -------------------------------------------------------------------------------- //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

module neorv32_dmem_ram #(
  parameter AWIDTH = 8, // physical address width (byte-addressing)
  parameter OUTREG = 0  // add additional output register when 1
)(
  input         clk_i,  // clock, trigger on rising edge
  input  [3:0]  en_i,   // byte-wise access-enable
  input         rw_i,   // 1 = write, 0 = read
  input  [31:0] addr_i, // full byte address (!)
  input  [31:0] data_i, // write data
  output [31:0] data_o  // read data
);

  // we have 4 parallel byte-wide RAMs; so we need to access them using the WORD address
  localparam RAM_AWIDTH = AWIDTH-2;
  wire [RAM_AWIDTH-1:0] addr = addr_i[AWIDTH-1:2];

  // the RAM is split into 4x 8-bit RAMs as some FPGA synthesis tools have issues
  // inferring a 32-bit RAM with individual byte-enables; they also seem to have
  // issues with multi-dimensional RAMs; i.e. [4][2**AWIDTH][8]
  reg [7:0] ram_b0 [2**RAM_AWIDTH-1:0];
  reg [7:0] ram_b1 [2**RAM_AWIDTH-1:0];
  reg [7:0] ram_b2 [2**RAM_AWIDTH-1:0];
  reg [7:0] ram_b3 [2**RAM_AWIDTH-1:0];
  reg [31:0] rdata;

  always @(posedge clk_i) begin
    if (en_i[0]) begin // byte 0
      if (rw_i) ram_b0[addr] <= data_i[7:0];
      rdata[7:0] <= ram_b0[addr];
    end
    if (en_i[1]) begin // byte 1
      if (rw_i) ram_b1[addr] <= data_i[15:8];
      rdata[15:8] <= ram_b1[addr];
    end
    if (en_i[2]) begin // byte 2
      if (rw_i) ram_b2[addr] <= data_i[23:16];
      rdata[23:16] <= ram_b2[addr];
    end
    if (en_i[3]) begin // byte 3
      if (rw_i) ram_b3[addr] <= data_i[31:24];
      rdata[31:24] <= ram_b3[addr];
    end
  end

  // optional output register
  generate
    if (OUTREG == 1) begin
      reg [31:0] rdata_reg;
      always @(posedge clk_i) begin
        rdata_reg <= rdata;
      end
      assign data_o = rdata_reg;
    end else begin
      assign data_o = rdata;
    end
  endgenerate

endmodule
