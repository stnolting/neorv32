## Processor Memory Source Files

This folder provides the architecture-only VHDL sources for the processor-internal memories
(instruction memory "IMEM", data memory "DMEM"). Different implementations are available, but
only **one** version of each (IMEM and DMEM) should be added as actual source files.

For an initial setup the `*.default.vhd` files should be selected. The HDL style for describing
memories used by these files has proven quite good platform-independence across several FPGA
architectures and toolchains.

If synthesis fails to infer block RAM resources from these default files, try the legacy
`*.legacy.vhd` files, which provide a different HDL style. These files are also intended for
legacy support of older Intel/Altera Quartus versions (13.0 and older). However, these files
still do not use platform-specific macros or primitives - so they might also work for other
FPGAs and toolchains.

> [!IMPORTANT]
> Make sure to add the selected files from this folder also to the `neorv32` design library.
