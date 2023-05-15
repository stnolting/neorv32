# NEORV32 System View Description (SVD) File

Maintained (manually) as a "single source-of-truth for automatically generating other source files.

* Format: CMSIS-SVD
* Copyright by ARM Ltd, Apache-2.0 License 
* Documentation:
   * https://www.keil.com/pack/doc/CMSIS/SVD/html/index.html
   * https://github.com/ARM-software/CMSIS
   * https://github.com/ARM-software/CMSIS_5

The `gen_header.py` script generates `sw/lib/include/neorv32_svd.h`.  To use this file when building C programs, add `USER_FLAGS=-DNEORV32_SVD_HEADER` to your `make` command; otherwise, building C programs will continue using the peripheral types/constants already defined in the existing `sw/lib/include/neorv32_<peripheral>_.h` files.