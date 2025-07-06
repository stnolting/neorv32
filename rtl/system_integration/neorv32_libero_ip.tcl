# Exporting core neorv32_mchp_ip_wrapper to TCL
# Exporting Create HDL core command for module neorv32_mchp_ip_wrapper
create_hdl_core -file {hdl/rtl/system_integration/neorv32_libero_ip.vhd} -module {neorv32_libero_ip} -library {neorv32} -package {}
# Exporting BIF information of  HDL core command for module neorv32_mchp_ip_wrapper
hdl_core_add_bif -hdl_core_name {neorv32_libero_ip} -bif_definition {AXI:AMBA:AMBA3:master} -bif_name {AXI4_M} -signal_map {\
"AWADDR:m_axi_awaddr" \
"AWLEN:m_axi_awlen" \
"AWSIZE:m_axi_awsize" \
"AWBURST:m_axi_awburst" \
"AWCACHE:m_axi_awcache" \
"AWPROT:m_axi_awprot" \
"AWVALID:m_axi_awvalid" \
"AWREADY:m_axi_awready" \
"WDATA:m_axi_wdata" \
"WSTRB:m_axi_wstrb" \
"WLAST:m_axi_wlast" \
"WVALID:m_axi_wvalid" \
"WREADY:m_axi_wready" \
"BRESP:m_axi_bresp" \
"BVALID:m_axi_bvalid" \
"BREADY:m_axi_bready" \
"ARADDR:m_axi_araddr" \
"ARLEN:m_axi_arlen" \
"ARSIZE:m_axi_arsize" \
"ARBURST:m_axi_arburst" \
"ARCACHE:m_axi_arcache" \
"ARPROT:m_axi_arprot" \
"ARVALID:m_axi_arvalid" \
"ARREADY:m_axi_arready" \
"RDATA:m_axi_rdata" \
"RRESP:m_axi_rresp" \
"RLAST:m_axi_rlast" \
"RVALID:m_axi_rvalid" \
"RREADY:m_axi_rready" }
hdl_core_add_bif -hdl_core_name {neorv32_libero_ip} -bif_definition {AXI4Stream:AMBA:AMBA4:master} -bif_name {AXI4_STREAM_M} -signal_map {\
"TDATA:s0_axis_tdata" \
"TVALID:s0_axis_tvalid" \
"TREADY:s0_axis_tready" \
"TLAST:s0_axis_tlast" \
"TDEST:s0_axis_tdest" }
hdl_core_add_bif -hdl_core_name {neorv32_libero_ip} -bif_definition {AXI4Stream:AMBA:AMBA4:slave} -bif_name {AXI4_STREAM_S} -signal_map {\
"TVALID:s1_axis_tvalid" \
"TREADY:s1_axis_tready" \
"TDATA:s1_axis_tdata" \
"TLAST:s1_axis_tlast" \
"TID:s1_axis_tid" }
