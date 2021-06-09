(* blackbox *)
module SB_HFOSC (
  input  CLKHFEN,
  input  CLKHFPU,
  output CLKHF
);
  parameter CLKHF_DIV = 2'b00;
endmodule

(* blackbox *)
module SB_PLL40_CORE (
  input  REFERENCECLK,
  output PLLOUTCORE,
  output PLLOUTGLOBAL,
  input  EXTFEEDBACK,
  input  [7:0] DYNAMICDELAY,
  output LOCK,
  input  BYPASS,
  input  RESETB,
  input  LATCHINPUTVALUE,
  output SDO,
  input  SDI,
  input  SCLK
);
  parameter FEEDBACK_PATH = "SIMPLE";
  parameter DELAY_ADJUSTMENT_MODE_FEEDBACK = "FIXED";
  parameter DELAY_ADJUSTMENT_MODE_RELATIVE = "FIXED";
  parameter SHIFTREG_DIV_MODE = 1'b0;
  parameter FDA_FEEDBACK = 4'b0000;
  parameter FDA_RELATIVE = 4'b0000;
  parameter PLLOUT_SELECT = "GENCLK";
  parameter DIVR = 4'b0000;
  parameter DIVF = 7'b0000000;
  parameter DIVQ = 3'b000;
  parameter FILTER_RANGE = 3'b000;
  parameter ENABLE_ICEGATE = 1'b0;
  parameter TEST_MODE = 1'b0;
  parameter EXTERNAL_DIVIDE_FACTOR = 1;
endmodule
