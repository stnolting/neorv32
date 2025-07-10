# Libero doesn't directly support .f files - will create an index from them and import
set f_files [glob -nocomplain "../*.f"]

# Check if any .f files were found
if {[llength $f_files] == 0} {
    puts "No .f files found in the parent directory."
} else {
    # Process each .f file
    foreach f_file $f_files {
        set file_handle [open $f_file r]
        set file_data [read $file_handle]
        close $file_handle

        # Split the file data into individual paths
        set paths [split $file_data "\n"]

        foreach path $paths {
            # Remove any leading/trailing whitespace
            set path [string trim $path]

            # Skip empty lines
            if {$path eq ""} {
                continue
            }

            # Replace the placeholder with the relative path
            set final_path [string map {"NEORV32_RTL_PATH_PLACEHOLDER" "../"} $path]

            # Construct and execute the import_files command
            puts "Importing file: $final_path" ; # For debugging/logging
            catch {
                import_files \
                    -convert_EDN_to_HDL 0 \
                    -library {neorv32} \
                    -hdl_source [list $final_path]
            } result
            if {$result ne ""} {
                puts "Error importing $final_path: $result"
            }
        }
    }
}

# Now specifically import the wrapper and AXI bridge files

import_files \
                    -convert_EDN_to_HDL 0 \
                    -library {neorv32} \
                    -hdl_source "./neorv32_libero_ip.vhd"

import_files \
                    -convert_EDN_to_HDL 0 \
                    -library {neorv32} \
                    -hdl_source "./xbus2axi4_bridge.vhd"
					
build_design_hierarchy

# Creating a HDL+ core of the wrapper for simplier import
create_hdl_core -file {hdl/neorv32_libero_ip.vhd} -module {neorv32_libero_ip} -library {neorv32} -package {}

# Exporting BIF information of  HDL core command for module neorv32_mchp_ip_wrapper
hdl_core_add_bif -hdl_core_name {neorv32_libero_ip} -bif_definition {AXI4:AMBA:AMBA4:master} -bif_name {axi4_m} -signal_map {\
"AWID:m_axi_awid" \
"AWADDR:m_axi_awaddr" \
"AWLEN:m_axi_awlen" \
"AWSIZE:m_axi_awsize" \
"AWBURST:m_axi_awburst" \
"AWLOCK:m_axi_awlock" \
"AWCACHE:m_axi_awcache" \
"AWPROT:m_axi_awprot" \
"AWQOS:m_axi_awqos" \
"AWREGION:m_axi_awregion" \
"AWVALID:m_axi_awvalid" \
"AWREADY:m_axi_awready" \
"WDATA:m_axi_wdata" \
"WSTRB:m_axi_wstrb" \
"WLAST:m_axi_wlast" \
"WVALID:m_axi_wvalid" \
"WREADY:m_axi_wready" \
"BID:m_axi_bid" \
"BRESP:m_axi_bresp" \
"BVALID:m_axi_bvalid" \
"BREADY:m_axi_bready" \
"ARID:m_axi_arid" \
"ARADDR:m_axi_araddr" \
"ARLEN:m_axi_arlen" \
"ARSIZE:m_axi_arsize" \
"ARBURST:m_axi_arburst" \
"ARLOCK:m_axi_arlock" \
"ARCACHE:m_axi_arcache" \
"ARPROT:m_axi_arprot" \
"ARQOS:m_axi_arqos" \
"ARREGION:m_axi_arregion" \
"ARVALID:m_axi_arvalid" \
"ARREADY:m_axi_arready" \
"RID:m_axi_rid" \
"RDATA:m_axi_rdata" \
"RRESP:m_axi_rresp" \
"RLAST:m_axi_rlast" \
"RVALID:m_axi_rvalid" \
"RREADY:m_axi_rready" \
"AWUSER:m_axi_awuser" \
"WUSER:m_axi_wuser" \
"BUSER:m_axi_buser" \
"ARUSER:m_axi_aruser" \
"RUSER:m_axi_ruser" }

hdl_core_add_bif -hdl_core_name {neorv32_libero_ip} -bif_definition {AXI4Stream:AMBA:AMBA4:master} -bif_name {axi4_stream_m} -signal_map {\
"TDATA:s0_axis_tdata" \
"TVALID:s0_axis_tvalid" \
"TREADY:s0_axis_tready" \
"TLAST:s0_axis_tlast" \
"TDEST:s0_axis_tdest" }

hdl_core_add_bif -hdl_core_name {neorv32_libero_ip} -bif_definition {AXI4Stream:AMBA:AMBA4:slave} -bif_name {axi4_stream_s} -signal_map {\
"TVALID:s1_axis_tvalid" \
"TREADY:s1_axis_tready" \
"TDATA:s1_axis_tdata" \
"TLAST:s1_axis_tlast" \
"TID:s1_axis_tid" }
