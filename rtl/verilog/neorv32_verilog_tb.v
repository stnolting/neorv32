// ================================================================================ //
// NEORV32 - Verilog testbench                                                      //
// -------------------------------------------------------------------------------- //
// Simple testbench for the auto-generated all-Verilog version of NEORV32.          //
// Checks for the initial UART output of the bootloader ("NEORV32").                //
// -------------------------------------------------------------------------------- //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

`timescale 1 ns/100 ps // time-unit = 1 ns, precision = 100 ps

module neorv32_verilog_tb;

  reg clk, nrst; // generators
  wire uart_txd; // serial TX line (default baud rate is 19200)
  wire [7:0] char_data; // character detected by the UART receiver
  wire char_valid; // valid character

  // generator setup
  initial begin
`ifdef DUMP_WAVE
    $dumpfile("wave.fst"); // write waveform data
    $dumpvars();
`endif
    $display ("[TB] NEORV32 Verilog testbench\n");
    clk = 0;
    nrst = 0;
    #100; // active reset for 100 * timescale = 100 ns
    nrst = 1;
    #15_000_000;
    // if we reach this the simulation has failed
    $display("[TB] Simulation terminated (time out)!");
    $finish; // terminate
  end

  // clock generator
  always begin
    #5 clk = !clk; // T = 2*5ns -> f = 100MHz
  end

  // unit under test
  // note that there are NO parameters available - the configuration has to be done
  // in the NEORV32 VHDL wrapper *before* synthesizing the generated Verilog code
  neorv32_verilog_wrapper neorv32_verilog_inst (
    .clk_i       (clk),
    .rstn_i      (nrst),
    .uart0_rxd_i (1'b0),
    .uart0_txd_o (uart_txd)
  );

  // simulation UART receiver - outputs all received characters to the simulator console
  uart_sim_receiver #(
    .CLOCK_FREQ (100000000), // clock frequency of the core
    .BAUD_RATE  (19200)      // default baud rate of the NEORV32 bootloader
  ) uart_sim_receiver_inst(
    .clk_i   (clk),
    .txd_i   (uart_txd),
    .data_o  (char_data),
    .valid_o (char_valid)
  );

  // line buffer for UART output
  reg [7:0] uart_log [0:63];
  integer i;

  always @(posedge clk or negedge nrst) begin
    if (nrst == 1'b0) begin
      i <= 0;
    end else begin
      if (char_valid == 1'b1) begin
        if ((char_data == 8'd10) || (char_data == 8'd13)) begin
          i <= 0; // line break
        end else begin
          uart_log[i] <= char_data;
          if (i >= 63) begin
            i <= 0;
          end else begin
            i <= i + 1;
          end
        end
      end
      // check for result string: "NEORV32" is sent by the default bootloader right after reset
      if ((uart_log[0] == "N") &&
          (uart_log[1] == "E") &&
          (uart_log[2] == "O") &&
          (uart_log[3] == "R") &&
          (uart_log[4] == "V") &&
          (uart_log[5] == "3") &&
          (uart_log[6] == "2")) begin
        $display("\n[TB] Simulation successful!");
        $finish; // terminate
      end
    end
  end

endmodule

// ****************************************************************************
// Simulation UART receiver
//
// Outputs printable characters to the simulator console.
// Character data is also returned to the top entity for further processing.
// ****************************************************************************

// by Stephan Nolting, BSD 3-Clause License
// https://github.com/stnolting/neorv32-verilog

module uart_sim_receiver
#(
  parameter CLOCK_FREQ = 100000000, // clock frequency of <clk_i> in Hz
  parameter BAUD_RATE  = 19200      // target baud rate
)(
  input        clk_i,  // clock input, triggering on rising edge
  input        txd_i,  // UART transmit data
  output [7:0] data_o, // character data
  output       valid_o // character data valid when set
);

  // duration of a single bit
  localparam UART_BAUD_VAL = CLOCK_FREQ / BAUD_RATE;

  // receiver
  reg  [4:0] uart_rx_sync;     // synchronizer shift register
  reg  [8:0] uart_rx_sreg;     // data shift register
  reg        uart_rx_busy;     // busy flag
  integer    uart_rx_baud_cnt; // bit-sample counter for baud rate
  integer    uart_rx_bitcnt;   // bit counter: 8 data bits, 1 start bit

  // initialize because we don't have a real reset
  initial begin
    uart_rx_sync     = 5'b11111;
    uart_rx_busy     = 1'b0;
    uart_rx_sreg     = 9'b000000000;
    uart_rx_baud_cnt = UART_BAUD_VAL / 2;
    uart_rx_bitcnt   = 0;
  end

  // UART receiver
  always @(posedge clk_i) begin
    // synchronizer
    uart_rx_sync <= {uart_rx_sync[3:0], txd_i};
    // arbiter
    if (!uart_rx_busy) begin // idle
      uart_rx_busy     <= 0;
      uart_rx_baud_cnt <= UART_BAUD_VAL / 2;
      uart_rx_bitcnt   <= 9;
      if (uart_rx_sync[4:1] == 4'b1100) begin // start bit (falling edge)?
        uart_rx_busy <= 1;
      end
    end else begin
      if (uart_rx_baud_cnt == 0) begin
        if (uart_rx_bitcnt == 1) begin
          uart_rx_baud_cnt <= UART_BAUD_VAL / 2;
        end else begin
          uart_rx_baud_cnt <= UART_BAUD_VAL;
        end
        // sample 8 data bits and 1 start bit
        if (uart_rx_bitcnt == 0) begin
          uart_rx_busy <= 1'b0; // done
          if ((uart_rx_sreg[8:1] >= 32) && (uart_rx_sreg[8:1] <= 127)) begin // is a printable char?
            $write("%c", uart_rx_sreg[8:1]);
          end else if (uart_rx_sreg[8:1] == 10) begin // Linux line break?
            $display(""); // force terminal line break
          end
        end else begin
          uart_rx_sreg   <= {uart_rx_sync[4], uart_rx_sreg[8:1]};
          uart_rx_bitcnt <= uart_rx_bitcnt - 1;
        end
      end else begin
        uart_rx_baud_cnt <= uart_rx_baud_cnt - 1;
      end
    end
  end

  // character output
  assign data_o  = uart_rx_sreg[8:1];
  assign valid_o = ((uart_rx_baud_cnt == 0) && (uart_rx_bitcnt == 0)) ? 1'b1 : 1'b0; // valid

endmodule
