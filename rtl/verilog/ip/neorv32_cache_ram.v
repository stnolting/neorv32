// ================================================================================ //
// NEORV32 - Generic Cache - Data and Tag RAM Primitive Wrapper - Verilog IP Module //
// -------------------------------------------------------------------------------- //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

module neorv32_cache_ram #(
  parameter TAG_WIDTH = 26, // tag width
  parameter IDX_WIDTH = 2,  // index width
  parameter OFS_WIDTH = 6   // offset width
)(
  input         clk_i,     // global clock, rising edge
  input  [31:0] addr_i,    // full byte address
  input         tag_we_i,  // tag write-enable
  output [31:0] tag_o,     // zero-extended read tag
  input  [3:0]  data_we_i, // byte-wise data write-enable
  input  [31:0] data_i,    // write data
  output [31:0] data_o     // read data
);

  // tag RAM
  reg [TAG_WIDTH-1:0] tag_ram [2**IDX_WIDTH-1:0];
  reg [TAG_WIDTH-1:0] tag_rd;
  always @(posedge clk_i) begin
    if (tag_we_i) begin
      tag_ram[addr_i[31-TAG_WIDTH:2+OFS_WIDTH]] <= addr_i[31:31-(TAG_WIDTH-1)];
    end
    tag_rd <= tag_ram[addr_i[31-TAG_WIDTH:2+OFS_WIDTH]];
  end
  assign tag_o = {{32-TAG_WIDTH{1'b0}}, tag_rd};

  // data RAM: 4 parallel byte-wide RAMs; so we need to access them using the WORD address
  localparam DATA_AWIDTH = (IDX_WIDTH+OFS_WIDTH)-2;
  wire [DATA_AWIDTH-1:0] data_addr = addr_i[IDX_WIDTH+OFS_WIDTH+1:2];

  // the data RAM is split into 4x 8-bit RAMs as some FPGA synthesis tools have issues
  // inferring a 32-bit RAM with individual byte-enables
  reg [7:0] ram_b0 [2**DATA_AWIDTH-1:0];
  reg [7:0] ram_b1 [2**DATA_AWIDTH-1:0];
  reg [7:0] ram_b2 [2**DATA_AWIDTH-1:0];
  reg [7:0] ram_b3 [2**DATA_AWIDTH-1:0];
  reg [31:0] rdata;

  always @(posedge clk_i) begin
    if (data_we_i[0]) ram_b0[data_addr] <= data_i[7:0];
    if (data_we_i[1]) ram_b1[data_addr] <= data_i[15:8];
    if (data_we_i[2]) ram_b2[data_addr] <= data_i[23:16];
    if (data_we_i[3]) ram_b3[data_addr] <= data_i[31:24];
    rdata <= {ram_b3[data_addr], ram_b2[data_addr], ram_b1[data_addr], ram_b0[data_addr]};
  end
  assign data_o = rdata;

endmodule
