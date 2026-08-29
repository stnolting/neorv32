// ================================================================================ //
// NEORV32 - Custom Functions Subsystem (CFS) - Verilog Version                     //
// -------------------------------------------------------------------------------- //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

module neorv32_cfs (
  // global control
  input  wire        clk_i,      // global clock
  input  wire        rstn_i,     // global reset, low-active, async
  // CPU request
  input  wire [15:0] req_addr_i, // byte address (64kB address space)
  input  wire [31:0] req_data_i, // write data
  input  wire [3:0]  req_ben_i,  // byte enable
  input  wire        req_stb_i,  // access request strobe
  input  wire        req_rw_i,   // 0=read, 1=write
  // CPU response
  output wire [31:0] rsp_data_o, // read data
  output wire        rsp_ack_o,  // access acknowledge
  // CPU interrupt
  output wire        irq_o,      // interrupt request, high-active
  // external IO
  input  wire [255:0] cfs_in_i,  // custom inputs conduit
  output wire [255:0] cfs_out_o  // custom outputs conduit
);

  // exemplary CFS interface registers
  reg  [31:0] cfs_reg_wr [0:3]; // for WRITE accesses (4 read/write registers)
  wire [31:0] cfs_reg_rd [0:3]; // for READ accesses

  // CFS IOs --------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  // By default, the CFS provides two IO ports (cfs_in_i and cfs_out_o) that are available at the processor's top entity.
  // These are intended as "conduits" to propagate custom CFS signals between the CFS and the processor top entity.

  assign cfs_out_o = 256'b0; // not used for this minimal example

  // Interrupt ------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  // The CFS features a single interrupt signal, which is connected to the CPU's "fast interrupt" channel 1 (FIRQ1).
  // The according CPU interrupt becomes pending as long as <irq_o> is high.

  assign irq_o = 1'b0; // not used for this minimal example

  // Read/Write Access ----------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  // The CFS provides up to 64kB of memory-mapped address space (16 address bits, byte-addressing) that can be used
  // for custom memories and interface registers. According to the CPU's bus protocol, each read or write access has
  // to be acknowledged in the following cycle using the <rsp_ack_o> signal (or even later if the module needs
  // additional time to complete the access). If no ACK is generated, the bus access will time out causing a bus access
  // fault exception (this can also be done intentionally to explicitly throw an access exception).
  //
  // [EXAMPLE] Read and write access to the interface registers and bus transfer acknowledge. This example only four
  // physical 32-bit read/write register (using the four lowest CFS address bits). The remaining addresses of the CFS
  // are not associated with any physical registers - any access to those is simply ignored but still acknowledged;
  // read accesses will return zero. Only full-word write accesses are supported by this example.
  // Sub-word write accesses are ignored but still acknowledged.

  reg [31:0] rsp_data;
  reg rsp_ack;

  always @(posedge clk_i or negedge rstn_i) begin
    if (rstn_i == 1'b0) begin
      cfs_reg_wr[0] <= 32'b0;
      cfs_reg_wr[1] <= 32'b0;
      cfs_reg_wr[2] <= 32'b0;
      cfs_reg_wr[3] <= 32'b0;
      rsp_data      <= 32'b0;
      rsp_ack       <= 1'b0;
    end else begin // synchronous interface for read and write accesses
      // transfer/access acknowledge
      rsp_ack <= req_stb_i; // send ACK right after the access request

      // bus access
      rsp_data <= 32'b0; // the output HAS TO BE ZERO if there is no actual (read) access
      if (req_stb_i == 1'b1) begin // valid access cycle, STB is high for exactly one cycle

        // write access (word-wise)
        if (req_rw_i == 1'b1) begin
          if (req_addr_i[15:2] == 14'b00000000000000) begin // 16-bit byte address = 14-bit word address
            cfs_reg_wr[0] <= req_data_i;
          end
          if (req_addr_i[15:2] == 14'b00000000000001) begin
            cfs_reg_wr[1] <= req_data_i;
          end
          if (req_addr_i[15:2] == 14'b00000000000010) begin
            cfs_reg_wr[2] <= req_data_i;
          end
          if (req_addr_i[15:2] == 14'b00000000000011) begin
            cfs_reg_wr[3] <= req_data_i;
          end

        // read access (word-wise)
        end else begin
          case (req_addr_i[15:2]) // 16-bit byte-address = 14-bit word-address
            14'b00000000000000: begin rsp_data <= cfs_reg_rd[0]; end
            14'b00000000000001: begin rsp_data <= cfs_reg_rd[1]; end
            14'b00000000000010: begin rsp_data <= cfs_reg_rd[2]; end
            14'b00000000000011: begin rsp_data <= cfs_reg_rd[3]; end
            default:            begin rsp_data <= 32'b0;         end
          endcase
        end

      end
    end
  end

  assign rsp_data_o = rsp_data;
  assign rsp_ack_o  = rsp_ack;

  // CFS Function Core ----------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------
  // This is where the actual functionality can be implemented. The logic below is just a very
  // simple example that transforms data from an input register into data in an output register.

  // bit reversal helper
  function [31:0] bit_rev_f;
    input [31:0] data;
    integer i;
    begin
      for (i=0; i<32; i=i+1) begin
        bit_rev_f[i] = data[31-i];
      end
    end
  endfunction

  assign cfs_reg_rd[0] = {28'h0000000, 3'b000, (|cfs_reg_wr[0])}; // OR all bits
  assign cfs_reg_rd[1] = {28'h0000000, 3'b000, (^cfs_reg_wr[1])}; // XOR all bits
  assign cfs_reg_rd[2] = bit_rev_f(cfs_reg_wr[2]);                // bit reversal
  assign cfs_reg_rd[3] = cfs_reg_wr[3];                           // pass-through

endmodule
