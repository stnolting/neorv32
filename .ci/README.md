## Scripts for Continous Integration

These scripts are called by the Travis CI job configuration file `.travis.yml` in the repository's root directory.

### `install.sh`

This scripts download a pre-built toolchain from the [`stnolting/riscv_gcc_prebuilt`](https://github.com/stnolting/riscv_gcc_prebuilt)
repository, extracts the archive and installs the toolchain into new `riscv` folder.

### `sw_check.sh`

Compiles and generates executables for all example projects from `sw/example/`, compiles and installs the default bootloader and
compiles and installs the `sw/example/cpu_test` CPU test program.

### `hw_check.sh`

`GHDL` is used to check all VHDL files for syntax errors and to simulate the default testbench. The previously installed CPU test program
is executed and the console output is also dumped to a text file. After the simulation has finished, the text file is searched for a specific
string. If the string is found, the CPU test was successful.
