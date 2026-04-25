//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2025.1 (win64) Build 6140274 Thu May 22 00:12:29 MDT 2025
//Date        : Wed Apr 22 23:46:46 2026
//Host        : ece-d4000-wshak running 64-bit major release  (build 9200)
//Command     : generate_target design_1_wrapper.bd
//Design      : design_1_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module design_1_wrapper
   (clk_100MHz,
    gpio_i_0,
    gpio_o_0,
    reset_rtl_0,
    uart0_rxd_i_0,
    uart0_txd_o_0);
  input clk_100MHz;
  input [7:0]gpio_i_0;
  output [7:0]gpio_o_0;
  input reset_rtl_0;
  input uart0_rxd_i_0;
  output uart0_txd_o_0;

  wire clk_100MHz;
  wire [7:0]gpio_i_0;
  wire [7:0]gpio_o_0;
  wire reset_rtl_0;
  wire uart0_rxd_i_0;
  wire uart0_txd_o_0;

  design_1 design_1_i
       (.clk_100MHz(clk_100MHz),
        .gpio_i_0(gpio_i_0),
        .gpio_o_0(gpio_o_0),
        .reset_rtl_0(reset_rtl_0),
        .uart0_rxd_i_0(uart0_rxd_i_0),
        .uart0_txd_o_0(uart0_txd_o_0));
endmodule
