// ****************************************************************************
// NEORV32 Verilog testbench
//
// Simple testbench to check the default NEORV32 Verilog wrapper that
// checks for the initial UART output of the bootloader ("NEORV32").
// ****************************************************************************

`timescale 1 ns/100 ps // time-unit = 1 ns, precision = 100 ps

module neorv32_verilog_tb;

  reg clk, nrst; // generators
  wire uart_txd; // serial TX line (default baud rate is 19200)
  wire [7:0] char_data; // character detected by the UART receiver
  wire char_valid; // valid character

  // generator setup
  initial begin
    if (`DUMP_WAVE == 1) begin
      $dumpfile("wave.fst"); // write waveform data
      $dumpvars();
    end
    $display ("\nNEORV32 Verilog testbench\n");
    clk = 0;
    nrst = 0;
    #100; // active reset for 100 * timescale = 100 ns
    nrst = 1;
    #15_000_000;
    // if we reach this the simulation has failed
    $display("Simulation terminated (time out)!");
    $finish; // terminate
  end

  // clock generator
  always begin
    #5 clk = !clk; // T = 2*5ns -> f = 100MHz
  end

  // unit under test: minimal NEORV32 Verilog wrapper
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

  // buffer the processor's UART data in a small FIFO-like queue
  reg [7:0] char_buffer [0:6];
  integer i;

  always @(posedge clk) begin
    // update "FIFO"
    if (char_valid == 1'b1) begin
      // top-to-bottom shift
      for (i=6; i>0; i=i-1) begin
        char_buffer[i-1] <= char_buffer[i];
      end
      char_buffer[6] <= char_data;
    end
    // check for result string: "NEORV32" is sent by the default bootloader right after reset
    if ((char_buffer[0] == "N") &&
        (char_buffer[1] == "E") &&
        (char_buffer[2] == "O") &&
        (char_buffer[3] == "R") &&
        (char_buffer[4] == "V") &&
        (char_buffer[5] == "3") &&
        (char_buffer[6] == "2")) begin
      $display (""); // force line break
      $display("Simulation successful!");
      $finish; // terminate
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
