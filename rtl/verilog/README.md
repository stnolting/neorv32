# NEORV32 in Verilog

[![Verilog Conversion](https://img.shields.io/github/actions/workflow/status/stnolting/neorv32/Verilog.yml?branch=main&longCache=true&style=flat-square&label=Verilog%20Conversion&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions/workflows/Verilog.yml)

You can use [GHDL's synthesis feature](https://ghdl.github.io/ghdl/using/Synthesis.html) to convert a preconfigured
NEORV32 setup in to a single, synthesizable, **plain-Verilog** module. The resulting Verilog module can be instantiated
within an all-Verilog design. The provided makefile is used for conversion and also for running all-Verilog simulation.

:books: See user guide section ["UG: NEORV32 in Verilog"](https://stnolting.github.io/neorv32/ug/#_neorv32_in_verilog)
for more information.

```
neorv32/rtl/verilog$ make help
NEORV32 Verilog Conversion and Test

Targets:
  help       Show this text
  version    Show NEORV32 version
  check      Show GHDL version
  convert    Convert NEORV32 to Verilog (generate '')
  prototype  Show Verilog instantiation prototoype
  sim        Run simulation with Icarus Verilog or Verilator
  clean      Remove all artifacts
  all        clean + check + version + convert + sim

Variables:
  TOP        Conversion wrapper; default = neorv32_verilog_wrapper
  GHDL       GHDL executable; default = ghdl
  SIMULATOR  Verilog simulator executable; default = iverilog
  DUMP_WAVE  Dump waveform data to 'wave.fst' when 1; default = 0

Example:
  make SIMULATOR=verilator DUMP_WAVE=1 clean convert sim
```

> [!TIP]
> The [Verilog GH actions workflow](https://github.com/stnolting/neorv32/actions/workflows/Verilog.yml)
automatically converts the pre-configured wrapper and runs Icarus Verilog and Verilator simulations.
The generated Verilog code can be downloaded as CI Workflow artifact.
