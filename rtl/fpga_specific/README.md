## FPGA Platform-Specific Components

This folder contains FPGA vendor-specific components (mostly memory components).
These alternative files allow a more efficient usage of the special (FPGA-specific) hard macros.

For example, if you want to use the FPGA optimized version of the DMEM, please use the file from the
according FPGA vendor folder instead of the original file from the rtl/core folder.