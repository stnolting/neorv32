// ================================================================================ //
// NEORV32 CPU - Data Register File - Verilog Version                               //
// -------------------------------------------------------------------------------- //
// The architecture style of the register file is selected by the ARCHSEL generic:  //
// 0: Register-based SRAM with sync. read (e.g. to map to FPGA block RAM)           //
// 1: Register-based SRAM with async. read (e.g. to map to FPGA distributed RAM)    //
// 2: Register-based with full hardware reset                                       //
// 3: Latch-based (e.g. for ASIC implementation)                                    //
//                                                                                  //
// [NOTE] Read-during-write behavior of the register file's memory core is          //
//        irrelevant as read and write accesses are mutually exclusive.             //
// -------------------------------------------------------------------------------- //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

module neorv32_cpu_regfile #(
  parameter AWIDTH  = 5, // register address width: 4 = 16 registers, 5 = 32 registers
  parameter ARCHSEL = 2  // register-file architecture style: 0..3
)(
  // global control
  input         clk_i,      // global clock, rising edge
  input         rstn_i,     // global reset, low-active, async
  input         zero_i,     // force write to x0 (for SRAM memory only)
  // write port (rd)
  input         rd_we_i,    // write-enable
  input  [4:0]  rd_addr_i,  // address
  input  [31:0] rd_data_i,  // write data
  // read port 1 (rs1)
  input  [4:0]  rs1_addr_i, // address
  output [31:0] rs1_data_o, // read data
  // read port 2 (rs2)
  input  [4:0]  rs2_addr_i, // address
  output [31:0] rs2_data_o  // read data
);

  reg [31:0] regfile [0:2**AWIDTH-1];

  wire [AWIDTH-1:0] rd_addr  = rd_addr_i[AWIDTH-1:0];
  wire [AWIDTH-1:0] rs1_addr = rs1_addr_i[AWIDTH-1:0];
  wire [AWIDTH-1:0] rs2_addr = rs2_addr_i[AWIDTH-1:0];

  reg [31:0] rs1_rdata, rs2_rdata;
  assign rs1_data_o = rs1_rdata;
  assign rs2_data_o = rs2_rdata;

  // Architecture Style 0: Register-Based SRAM with Synchronous Read ------------------------
  // -------------------------------------------------------------------------------------------
  generate
    if (ARCHSEL == 0) begin

      wire rf_we;
      wire [AWIDTH-1:0] addr;

      // x0 is a normal physical register in this mode. It is only written if
      // zero_i forces a write, which is used by the CPU to re-initialize x0.
      assign rf_we = (rd_we_i & (|rd_addr)) | zero_i;

      assign addr = (zero_i == 1'b1) ? {AWIDTH{1'b0}} : (rd_we_i == 1'b1) ? rd_addr : rs1_addr;

      always @(posedge clk_i) begin
        if (rf_we == 1'b1) begin
          regfile[addr] <= rd_data_i;
        end
        rs1_rdata <= regfile[addr];
        rs2_rdata <= regfile[rs2_addr];
      end

    end
  endgenerate

  // Architecture Style 1: Register-Based SRAM with Asynchronous Read -----------------------
  // -------------------------------------------------------------------------------------------
  generate
    if (ARCHSEL == 1) begin

      wire [AWIDTH-1:0] addr;

      // Multiplexed write/read address, as in the original SDPRAM-oriented design.
      assign addr = (rd_we_i == 1'b1) ? rd_addr : rs1_addr;

      // Synchronous write
      always @(posedge clk_i) begin
        if (rd_we_i == 1'b1) begin
          regfile[addr] <= rd_data_i;
        end
      end

      // Registered read and zero insertion for x0.
      always @(posedge clk_i) begin
        if (rs1_addr == {AWIDTH{1'b0}}) begin
          rs1_rdata <= 32'h00000000;
        end else begin
          rs1_rdata <= regfile[addr];
        end
        if (rs2_addr == {AWIDTH{1'b0}}) begin
          rs2_rdata <= 32'h00000000;
        end else begin
          rs2_rdata <= regfile[rs2_addr];
        end
      end

    end
  endgenerate

  // Architecture Style 2: Register-Based with Hardware Reset -------------------------------
  // -------------------------------------------------------------------------------------------
  generate
    if (ARCHSEL == 2) begin

      integer i;

      // Registers x1..x31 (or x1..x15 with AWIDTH=4).
      // x0 is not stored and is always read as zero.
      always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
          for (i = 1; i < REG_COUNT; i = i + 1) begin
            regfile[i] <= 32'h00000000;
          end
        end else begin
          if (rd_we_i && (rd_addr != {AWIDTH{1'b0}})) begin
            regfile[rd_addr] <= rd_data_i;
          end
        end
      end

      // Synchronous read
      always @(posedge clk_i) begin
        if (rs1_addr == {AWIDTH{1'b0}}) begin
          rs1_rdata <= 32'h00000000;
        end else begin
          rs1_rdata <= regfile[rs1_addr];
        end
        if (rs2_addr == {AWIDTH{1'b0}}) begin
          rs2_rdata <= 32'h00000000;
        end else begin
          rs2_rdata <= regfile[rs2_addr];
        end
      end

    end
  endgenerate

  // Architecture Style 3: Latch-Based ------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  generate
    if (ARCHSEL == 3) begin

      reg [31:0] wdata;
      reg [AWIDTH-1:0] waddr;

      // Write-data and write-address buffers.
      always @(posedge clk_i) begin
        if (rd_we_i == 1'b1) begin
          wdata <= rd_data_i;
          waddr <= rd_addr;
        end else begin
          waddr <= {AWIDTH{1'b0}};
        end
      end

      // x1..x31 are transparent latches while clk_i is low.
      genvar g;
      for (g=1; g<REG_COUNT; g=g+1) begin

        localparam [AWIDTH-1:0] REG_INDEX = g;

        always @(clk_i or waddr or wdata) begin
          if ((clk_i == 1'b0) && (waddr == REG_INDEX)) begin
            regfile[g] <= wdata;
          end
        end

      end

      // Synchronous read; x0 is hardwired to zero.
      always @(posedge clk_i) begin
        if (rs1_addr == {AWIDTH{1'b0}}) begin
          rs1_rdata <= 32'h00000000;
        end else begin
          rs1_rdata <= regfile[rs1_addr];
        end
        if (rs2_addr == {AWIDTH{1'b0}}) begin
          rs2_rdata <= 32'h00000000;
        end else begin
          rs2_rdata <= regfile[rs2_addr];
        end
      end

    end
  endgenerate

endmodule
