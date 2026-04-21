//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2025.1 (win64) Build 6140274 Thu May 22 00:12:29 MDT 2025
//Date        : Mon Apr 20 19:52:46 2026
//Host        : mobile running 64-bit major release  (build 9200)
//Command     : generate_target design_1.bd
//Design      : design_1
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* CORE_GENERATION_INFO = "design_1,IP_Integrator,{x_ipVendor=xilinx.com,x_ipLibrary=BlockDiagram,x_ipName=design_1,x_ipVersion=1.00.a,x_ipLanguage=VERILOG,numBlks=3,numReposBlks=3,numNonXlnxBlks=1,numHierBlks=0,maxHierDepth=0,numSysgenBlks=0,numHlsBlks=0,numHdlrefBlks=0,numPkgbdBlks=0,bdsource=USER,da_board_cnt=2,da_clkrst_cnt=3,synth_mode=None}" *) (* HW_HANDOFF = "design_1.hwdef" *) 
module design_1
   (clk_100MHz,
    gpio_i_0,
    gpio_o_0,
    reset_rtl_0,
    uart0_rxd_i_0,
    uart0_txd_o_0);
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 CLK.CLK_100MHZ CLK" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME CLK.CLK_100MHZ, CLK_DOMAIN design_1_clk_100MHz, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, INSERT_VIP 0, PHASE 0.0" *) input clk_100MHz;
  input [7:0]gpio_i_0;
  (* X_INTERFACE_INFO = "xilinx.com:signal:data:1.0 DATA.GPIO_O_0 DATA" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME DATA.GPIO_O_0, LAYERED_METADATA undef" *) output [7:0]gpio_o_0;
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 RST.RESET_RTL_0 RST" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME RST.RESET_RTL_0, INSERT_VIP 0, POLARITY ACTIVE_HIGH" *) input reset_rtl_0;
  input uart0_rxd_i_0;
  (* X_INTERFACE_INFO = "xilinx.com:signal:data:1.0 DATA.UART0_TXD_O_0 DATA" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME DATA.UART0_TXD_O_0, LAYERED_METADATA undef" *) output uart0_txd_o_0;

  wire clk_100MHz;
  wire clk_wiz_0_clk_out1;
  wire clk_wiz_0_locked;
  wire [7:0]gpio_i_0;
  wire [7:0]gpio_o_0;
  wire [0:0]proc_sys_reset_0_peripheral_aresetn;
  wire reset_rtl_0;
  wire uart0_rxd_i_0;
  wire uart0_txd_o_0;

  design_1_clk_wiz_0_0 clk_wiz_0
       (.clk_in1(clk_100MHz),
        .clk_out1(clk_wiz_0_clk_out1),
        .locked(clk_wiz_0_locked),
        .reset(reset_rtl_0));
  design_1_neorv32_vivado_ip_0_2 neorv32_vivado_ip_0
       (.clk(clk_wiz_0_clk_out1),
        .gpio_i(gpio_i_0),
        .gpio_o(gpio_o_0),
        .irq_mei_i(1'b0),
        .irq_msi_i(1'b0),
        .irq_mti_i(1'b0),
        .resetn(proc_sys_reset_0_peripheral_aresetn),
        .uart0_ctsn_i(1'b0),
        .uart0_rxd_i(uart0_rxd_i_0),
        .uart0_txd_o(uart0_txd_o_0));
  design_1_proc_sys_reset_0_0 proc_sys_reset_0
       (.aux_reset_in(1'b1),
        .dcm_locked(clk_wiz_0_locked),
        .ext_reset_in(reset_rtl_0),
        .mb_debug_sys_rst(1'b0),
        .peripheral_aresetn(proc_sys_reset_0_peripheral_aresetn),
        .slowest_sync_clk(clk_wiz_0_clk_out1));
endmodule
