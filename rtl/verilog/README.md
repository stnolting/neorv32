## NEORV32 in Verilog

[![Verilog Conversion](https://img.shields.io/github/actions/workflow/status/stnolting/neorv32/Verilog.yml?branch=main&longCache=true&style=flat-square&label=Verilog%20Conversion&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions/workflows/Verilog.yml)

You can use [GHDL's synthesis feature](https://ghdl.github.io/ghdl/using/Synthesis.html) to convert
a preconfigured NEORV32 wrapper setup into a single, synthesizable, **plain-Verilog** module. The
resulting Verilog module can be instantiated within an all-Verilog design.

GHDL supports black-box instantiation during conversion. This feature can be used to replace
certain NEORV32 modules by custom Verilog modules. For this purpose, the according NEORV32 VHDL
modules provide a simple, uniform entity: only generics that convert to integer Verilog parameters
and only ports with a static size. This allows to replace these components 1-by-1 by corresponding
Verilog modules. Exemplary Verilog replacement modules can be found in the [`modules`](modules) folder.

More information can be found in the online documentation:

* [Convert to Verilog](https://stnolting.github.io/neorv32/#_convert_to_verilog)
* [Replaceable Modules](https://stnolting.github.io/neorv32/#_replaceable_modules)

The provided Makefile is used to automate the conversion and to run simulation
with Verilator or Icarus Verilog:

```
neorv32/rtl/verilog$ make help
NEORV32 Verilog Conversion and Test

Targets:
  help       Show this text
  convert    Convert NEORV32 to Verilog (generate neorv32_verilog_wrapper.v)
  prototype  Show Verilog instantiation prototype
  sim        Run simulation with Icarus-Verilog or Verilator
  clean      Remove all build artifacts
  all        clean + convert + sim

Variables:
  GHDL          GHDL executable; default = ghdl
  WRAPPER       VHDL conversion wrapper; default = neorv32_verilog_wrapper
  VHDL_EXCLUDE  Default NEORV32 VHDL core files to exclude from conversion
  VERILOG_SRC   Custom Verilog (substitution) modules
  SIMULATOR     Verilog simulator; 'iverilog' or 'verilator'; default = iverilog
  DUMP_WAVE     Dump waveform data to 'wave.fst' when 1; default = 0

Simple example:
  make SIMULATOR=verilator DUMP_WAVE=1 clean convert sim

Default NEORV32 VHDL files can be excluded from the VHDL-to-Verilog conversion,
leaving them as black box instances in the resulting all-Verilog netlist. These
black box instances can be bind to custom Verilog moduless. Note that the substituted
Verilog modules must provide identical interfaces (parameters and ports).

Example: Replace the default NEORV32 DMEM RAM component by a custom Verilog module.
  make VHDL_EXCLUDE=neorv32_dmem.vhd VERILOG_SRC=modules/neorv32_dmem.v clean convert sim
```

> [!TIP]
> This flow uses the SoC's VHDL [file-list file](https://stnolting.github.io/neorv32/#_file_list_file)
for easy customization of the design sources.

> [!NOTE]
> The [Verilog GH actions workflow](https://github.com/stnolting/neorv32/actions/workflows/Verilog.yml)
automatically converts the pre-configured wrapper and runs Icarus Verilog and Verilator simulations.
The generated Verilog code can be downloaded as CI Workflow artifact.
