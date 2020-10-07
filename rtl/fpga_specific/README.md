## FPGA Platform-Specific Components

This folder contains FPGA vendor-specific CPU/processor components (mostly memory components).
These alternative files allow a more efficient usage of the special (FPGA-specific) hard macros
and thus, might result in higher performance and/or less area utilization.

Please note, that these FPGA-specific components are **optional**. The Processor/CPU uses an FPGA-independent
VHDL description.

For example, if you want to use the Lattice iCE40up FPGA optimized versions of the DMEM and DMEM please use the files
from [`rtl/fpga_specific/lattice_ice40up`](https://github.com/stnolting/neorv32/tree/master/rtl/fpga_specific/lattice_ice40up)
folder **instead** of the original files from the [`rtl/core folder`](https://github.com/stnolting/neorv32/tree/master/rtl/core).