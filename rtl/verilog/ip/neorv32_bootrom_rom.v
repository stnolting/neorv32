// ================================================================================ //
// NEORV32 - Bootloader ROM (ROM Primitive) - Verilog IP Module                     //
// -------------------------------------------------------------------------------- //
// Generic ROM description; initialized from plain 32-bit (4x8 HEX chars) file.     //
// -------------------------------------------------------------------------------- //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

module neorv32_bootrom_rom #(
  parameter AWIDTH = 16 // maximum address width (byte-addressing)
)(
  input         clk_i,  // clock, trigger on rising edge
  input         en_i,   // access-enable
  input  [31:0] addr_i, // full byte address (!)
  output [31:0] data_o  // read data, sync
);

  localparam ROM_AWIDTH = 10; // ACTUAL address width (words) of bootloader image
  wire [ROM_AWIDTH-1:0] addr = addr_i[AWIDTH-1:2];
  reg [31:0] rom [2**ROM_AWIDTH-1:0];
  reg [31:0] rdata;

  // initialize from HEX file
  // [NOTE] requires to build bootloader and generate HEX memory image file before
  initial begin
    $readmemh("../../sw/bootloader/neorv32_raw_exe.hex", rom, 0, 2**ROM_AWIDTH-1);
  end

  always @(posedge clk_i) begin
    if (en_i) begin
      rdata <= rom[addr];
    end
  end

  assign data_o = rdata;

endmodule
