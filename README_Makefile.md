
# Some words about control development flow in NEORV32 project by GNU Make

## Dependencies

1. GNU Make & Coreutils;
2. GHDL - VHDL compiler/simulator;
3. GNU compression utilities;
4. GTKWave - electronic Waveform Viewer;
5. Quartus Prime - EDA for Intel FPGA;
6. OpenOCD - on-chip debugging, in-system programming and boundary-scan testing tool for FPGA, RISC-V and many others systems.

## Usage

Run in terminal from the project root directory:
```
  make <TARGET>
```

For compile software print:
```
  make NOSW=<NAME_OF_EXAMPLE> TARGET=<TARGET_from_sw/example/NAME_OF_EXAMPLE/makefile>
```

## Targets

* new - setting up project.
* sim - see "analysis", "run" and "view" targets.
* analysis - analyze source files therewith GHDL.
* run - simulation therewith GHDL in "Elabortation and Run" mode. It produse log in terminal and VCD-file.
* view - display simulation results therewith GTKWave.
* implement_mars3 - implementation configuration files for Marsohod3 board therewith Quartus Prime.
* read_log_mars3 - display some log-files after implementation.
* prog_mars3_ram - programming configuration files into RAM of FPGA Marsohod3 board therewith OpenOCD.
* prog_mars3_flash - programming configuration files into FLASH of FPGA Marsohod3 board therewith OpenOCD.
* compile_sw - compile software with support "sw/example" makefile's
* clean - cleanup temporary files from project directory.
