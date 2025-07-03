# Get a list of all .f files in the parent directory
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

import_files \
                    -convert_EDN_to_HDL 0 \
                    -library {neorv32} \
                    -hdl_source "./neorv32_libero_ip.vhd"

import_files \
                    -convert_EDN_to_HDL 0 \
                    -library {neorv32} \
                    -hdl_source "./xbus2axi4_bridge.vhd"