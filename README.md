[![GitHub Pages](https://img.shields.io/website.svg?label=stnolting.github.io%2Fneorv32&longCache=true&style=flat-square&url=http%3A%2F%2Fstnolting.github.io%2Fneorv32%2Findex.html&logo=GitHub)](https://stnolting.github.io/neorv32)
[![Documentation](https://img.shields.io/github/workflow/status/stnolting/neorv32/Documentation/master?longCache=true&style=flat-square&label=Documentation&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions?query=workflow%3ADocumentation)

[![riscv-arch-test](https://img.shields.io/github/workflow/status/stnolting/neorv32/riscv-arch-test/master?longCache=true&style=flat-square&label=riscv-arch-test&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions?query=workflow%3Ariscv-arch-test)
[![Processor](https://img.shields.io/github/workflow/status/stnolting/neorv32/Processor/master?longCache=true&style=flat-square&label=Processor&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions?query=workflow%3AProcessor)
[![Implementation](https://img.shields.io/github/workflow/status/stnolting/neorv32/Implementation/master?longCache=true&style=flat-square&label=Implementation&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions?query=workflow%3AImplementation)
[![Windows](https://img.shields.io/github/workflow/status/stnolting/neorv32/Windows/master?longCache=true&style=flat-square&label=Windows&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions?query=workflow%3AWindows)

[![NEORV32](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/figures/neorv32_logo_dark.png)](https://github.com/stnolting/neorv32)

# The NEORV32 RISC-V Processor

[![license](https://img.shields.io/github/license/stnolting/neorv32?longCache=true&style=flat-square)](https://github.com/stnolting/neorv32/blob/master/LICENSE)
[![release](https://img.shields.io/github/v/release/stnolting/neorv32?longCache=true&style=flat-square&logo=GitHub)](https://github.com/stnolting/neorv32/releases)
[![datasheet (pdf)](https://img.shields.io/badge/data%20sheet-PDF-ffbd00?longCache=true&style=flat-square&logo=asciidoctor)](https://github.com/stnolting/neorv32/releases/tag/nightly)
[![datasheet (html)](https://img.shields.io/badge/-HTML-ffbd00?longCache=true&style=flat-square)](https://stnolting.github.io/neorv32)
[![userguide (pdf)](https://img.shields.io/badge/user%20guide-PDF-ffbd00?longCache=true&style=flat-square&logo=asciidoctor)](https://github.com/stnolting/neorv32/releases/tag/nightly)
[![userguide (html)](https://img.shields.io/badge/-HTML-ffbd00?longCache=true&style=flat-square)](https://stnolting.github.io/neorv32/ug)
[![doxygen](https://img.shields.io/badge/doxygen-HTML-ffbd00?longCache=true&style=flat-square&logo=Doxygen)](https://stnolting.github.io/neorv32/sw/files.html)

* [Overview](#Overview)
* [Processor/SoC Features](#NEORV32-Processor-Features)
  * [FPGA Implementation Results](#FPGA-Implementation-Results---Processor)
* [CPU Features](#NEORV32-CPU-Features)
  * [Available ISA Extensions](#Available-ISA-Extensions)
  * [FPGA Implementation Results](#FPGA-Implementation-Results---CPU)
  * [Performance](#Performance)
* [Software Framework & Tooling](#Software-Framework-and-Tooling)
* [**Getting Started**](#Getting-Started) :rocket:



## Overview

![neorv32 Overview](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/figures/neorv32_processor.png)

The NEORV32 Processor is a customizable microcontroller-like system on chip (SoC) that is based on the RISC-V NEORV32 CPU.
The project is intended as auxiliary processor in larger SoC designs or as *ready-to-go* stand-alone
custom / customizable microcontroller.

:information_source: Want to know more? Check out the [project's rationale](https://stnolting.github.io/neorv32/#_rationale).

:books: For detailed information take a look at the [NEORV32 documentation (online at GitHub-pages)](https://stnolting.github.io/neorv32/).
The *doxygen*-based documentation of the *software framework* is also available online
at [GitHub-pages](https://stnolting.github.io/neorv32/sw/files.html).

:label: The project's change log is available in [`CHANGELOG.md`](https://github.com/stnolting/neorv32/blob/master/CHANGELOG.md).
To see the changes between *official* releases visit the project's [release page](https://github.com/stnolting/neorv32/releases).

:package: The [`setups`](https://github.com/stnolting/neorv32/tree/master/setups) folder provides exemplary setups targeting
various FPGA boards and toolchains to get you started.

:spiral_notepad: Check out the [project boards](https://github.com/stnolting/neorv32/projects) for a list of current **ideas**,
**TODOs**, features being **planned** and **work-in-progress**.

:bulb: Feel free to open a [new issue](https://github.com/stnolting/neorv32/issues) or start a
[new discussion](https://github.com/stnolting/neorv32/discussions) if you have questions, comments, ideas or bug-fixes.
Check out how to contribute in [`CONTRIBUTE.md`](https://github.com/stnolting/neorv32/blob/master/CONTRIBUTING.md).

:rocket: Check out the [quick links below](#Getting-Started) or directly jump to the
[*User Guide*](https://stnolting.github.io/neorv32/ug/) to get started
setting up your NEORV32 setup!


### Project Key Features

* [CPU](#NEORV32-CPU-Features) plus [Processor/SoC](#NEORV32-Processor-Features) plus [Software Framework & Tooling](#Software-Framework-and-Tooling)
* completely described in behavioral, platform-independent VHDL - no primitives, macros, etc.
* fully synchronous design, no latches, no gated clocks
* be as small as possible (while being as RISC-V-compliant as possible) – but with a reasonable size-performance trade-off
(the processor has to fit in a Lattice iCE40 UltraPlus 5k low-power FPGA running at 22+ MHz)
* from zero to `printf("hello world!");` - completely open source and documented
* easy to use even for FPGA/RISC-V starters – intended to work *out of the box*

[[back to top](#The-NEORV32-RISC-V-Processor)]



## NEORV32 Processor Features

The NEORV32 Processor (top entity: [`rtl/core/neorv32_top.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/core/neorv32_top.vhd))
provides a full-featured SoC build around the NEORV32 CPU. It is highly configurable via generics
to allow a flexible customization according to your needs. Note that all modules listed below are _optional_.
In-depth detailed information regarding the processor/SoC can be found in the :books:
[online documentation - _"NEORV32 Processors (SoC)"_](https://stnolting.github.io/neorv32/#_neorv32_processor_soc).

**Memory**

* processor-internal data and instruction memories ([DMEM](https://stnolting.github.io/neorv32/#_data_memory_dmem) /
[IMEM](https://stnolting.github.io/neorv32/#_instruction_memory_imem)) &
cache ([iCACHE](https://stnolting.github.io/neorv32/#_processor_internal_instruction_cache_icache))
* bootloader ([BOOTLDROM](https://stnolting.github.io/neorv32/#_bootloader_rom_bootrom)) with serial user interface
  * supports boot via UART or from external SPI flash

**Timers**

* machine system timer ([MTIME](https://stnolting.github.io/neorv32/#_machine_system_timer_mtime)), RISC-V spec. compatible
* watchdog timer ([WDT](https://stnolting.github.io/neorv32/#_watchdog_timer_wdt))

**IO**

* standard serial interfaces
([UART](https://stnolting.github.io/neorv32/#_primary_universal_asynchronous_receiver_and_transmitter_uart0),
[SPI](https://stnolting.github.io/neorv32/#_serial_peripheral_interface_controller_spi),
[TWI / I²C](https://stnolting.github.io/neorv32/#_two_wire_serial_interface_controller_twi))
* general purpose [GPIO](https://stnolting.github.io/neorv32/#_general_purpose_input_and_output_port_gpio) and
[PWM](https://stnolting.github.io/neorv32/#_pulse_width_modulation_controller_pwm)
* smart LED interface ([NEOLED](https://stnolting.github.io/neorv32/#_smart_led_interface_neoled)) to directly drive _NeoPixel(TM)_ LEDs

**SoC Connectivity and Integration**

* 32-bit external bus interface, Wishbone b4 compatible
([WISHBONE](https://stnolting.github.io/neorv32/#_processor_external_memory_interface_wishbone_axi4_lite))
  * [wrapper](https://github.com/stnolting/neorv32/blob/master/rtl/templates/system/neorv32_SystemTop_axi4lite.vhd) for AXI4-Lite master interface
* 32-bit stram link interface with up to 8 independent RX and TX links
([SLINK](https://stnolting.github.io/neorv32/#_stream_link_interface_slink))
  * AXI4-Stream compatible
* alternative [top entities/wrappers](https://github.com/stnolting/neorv32/blob/master/rtl/templates) providing
simplified and/or resolved top entity ports for easy system integration
* custom functions subsystem ([CFS](https://stnolting.github.io/neorv32/#_custom_functions_subsystem_cfs))
for tightly-coupled custom co-processor extensions

**Advanced**

* _true random_ number generator ([TRNG](https://stnolting.github.io/neorv32/#_true_random_number_generator_trng))
* on-chip debugger ([OCD](https://stnolting.github.io/neorv32/#_on_chip_debugger_ocd)) via JTGA - implementing
the [*Minimal RISC-V Debug Specification Version 0.13.2*](https://github.com/riscv/riscv-debug-spec)
and compatible with *OpenOCD* and *gdb*

:information_source: It is recommended to use the processor setup even if you want to **use the CPU in stand-alone mode**.
Just disable all optional processor-internal modules via the according generics and you will get a "CPU wrapper" that
provides a minimal CPU environment and an external memory interface (like AXI4). This minimal setup allows to further use
the default bootloader and software framework. From this base you can start building your own processor system.

[[back to top](#The-NEORV32-RISC-V-Processor)]


### FPGA Implementation Results - Processor

The hardware resources used by a specifc processor setup is defined by the implemented CPU extensions
([see below](#FPGA-Implementation-Results---CPU)), the configuration of the peripheral modules and some "glue logic".
Section [_"Processor Modules"_](https://stnolting.github.io/neorv32/#_processor_modules) of the online datasheet shows
the ressource utilization of each optional processor module to estimate the actual setup's hardware requirements.

:information_source: The [`setups`](https://github.com/stnolting/neorv32/tree/master/setups) folder provides exemplary FPGA
setups targeting various FPGA boards and toolchains. These setups also provide ressource utilization reports for different
SoC configurations

[[back to top](#The-NEORV32-RISC-V-Processor)]



## NEORV32 CPU Features

:books: In-depth detailed information regarding the CPU can be found in the
[online documentation - _"NEORV32 Central Processing Unit"_](https://stnolting.github.io/neorv32/#_neorv32_central_processing_unit_cpu).

The CPU (top entity: [`rtl/core/neorv32_cpu.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/core/neorv32_cpu.vhd))
implements the RISC-V 32-bit `rv32` ISA with optional extensions (see below). It is compatible to a subset of the
*Unprivileged ISA Specification* [(Version 2.2)](https://github.com/stnolting/neorv32/blob/master/docs/references/riscv-spec.pdf)
and a subset of the *Privileged Architecture Specification* [(Version 1.12-draft)](https://github.com/stnolting/neorv32/blob/master/docs/references/riscv-privileged.pdf).
Compatiility is checked by passing the [official RISC-V architecture tests](https://github.com/riscv/riscv-arch-test)
(see [`riscv-arch-test/README`](https://github.com/stnolting/neorv32/blob/master/riscv-arch-test/README.md)).

The core implements a little-endian von-Neumann architecture using two pipeline stages. Each stage uses a multi-cycle processing
scheme. The CPU supports three privilege levels (`machine` and optional `user` and `debug_mode`), three standard RISC-V machine
interrupts (`MTI`, `MEI`, `MSI`), a single non-maskable interrupt plus 16 _fast interrupt requests_ as custom extensions.
It also supports **all** standard RISC-V exceptions (instruction/load/store misaligned address & bus access fault, illegal
instruction, breakpoint, environment call). As a special "execution safety" extension, _all_ invalid, reserved or
malformed instructions will raise an exception.


### Available ISA Extensions

Currently, the following _optional_ RISC-V-compatible ISA extensions are implemented (linked to the according
documentation section). Note that the `X` extension is always enabled.

**RV32
[[`I`](https://stnolting.github.io/neorv32/#_i_base_integer_isa)/
[`E`](https://stnolting.github.io/neorv32/#_e_embedded_cpu)]
[[`A`](https://stnolting.github.io/neorv32/#_a_atomic_memory_access)]
[[`C`](https://stnolting.github.io/neorv32/#_c_compressed_instructions)]
[[`M`](https://stnolting.github.io/neorv32/#_m_integer_multiplication_and_division)]
[[`U`](https://stnolting.github.io/neorv32/#_u_less_privileged_user_mode)]
[[`X`](https://stnolting.github.io/neorv32/#_x_neorv32_specific_custom_extensions)]
[[`Zfinx`](https://stnolting.github.io/neorv32/#_zfinx_single_precision_floating_point_operations)]
[[`Zicsr`](https://stnolting.github.io/neorv32/#_zicsr_control_and_status_register_access_privileged_architecture)]
[[`Zifencei`](https://stnolting.github.io/neorv32/#_zifencei_instruction_stream_synchronization)]
[[`Zmmul`](https://stnolting.github.io/neorv32/#_zmmul_integer_multiplication)]
[[`PMP`](https://stnolting.github.io/neorv32/#_pmp_physical_memory_protection)]
[[`HPM`](https://stnolting.github.io/neorv32/#_hpm_hardware_performance_monitors)]**

:information_source: The `B` ISA extension has been temporarily removed from the processor.
See [B ISA Extension](https://github.com/stnolting/neorv32/projects/7) project board.

[[back to top](#The-NEORV32-RISC-V-Processor)]


### FPGA Implementation Results - CPU

:books: More details regarding exemplary FPGA setups including a list of resource utilization by each SoC module can be found in the
[online documentation - _"FPGA Implementation Results"_](https://stnolting.github.io/neorv32/#_fpga_implementation_results).

Implementation results for exemplary CPU configuration generated for an **Intel Cyclone IV EP4CE22F17C6N FPGA**
using **Intel Quartus Prime Lite 20.1** ("balanced implementation"). The timing information is derived
from the Timing Analyzer / Slow 1200mV 0C Model. No constraints were used at all.

Results generated for hardware version [`1.5.3.2`](https://github.com/stnolting/neorv32/blob/master/CHANGELOG.md).

| CPU Configuration                                 | LEs  | FFs  | Memory bits | DSPs (9-bit) | f_max   |
|:--------------------------------------------------|:----:|:----:|:-----------:|:------------:|:-------:|
| `rv32i`                                           |  980 |  409 |        1024 |            0 | 125 MHz |
| `rv32i`    + `Zicsr`                              | 1835 |  856 |        1024 |            0 | 125 MHz |
| `rv32imac` + `Zicsr`                              | 2685 | 1156 |        1024 |            0 | 125 MHz |
| `rv32imac` + `Zicsr` + `u` + `Zifencei` + `Zfinx` | 4004 | 1812 |        1024 |            7 | 118 MHz |

[[back to top](#The-NEORV32-RISC-V-Processor)]


### Performance

The NEORV32 CPU is based on a two-stages pipelined architecutre. Since both stage use a multi-cycle processing scheme,
each instruction requires several clock cycles to execute (2 cycles for ALU operations, up to 40 cycles for divisions).
The average CPI (cycles per instruction) depends on the instruction mix of a specific applications and also on the
available CPU extensions.

The following table shows the performance results(relative CoreMark score and average cycles per instruction) for successfully
running 2000 iterations of the [CoreMark CPU benchmark](https://www.eembc.org/coremark).
The source files are available in [sw/example/coremark](https://github.com/stnolting/neorv32/blob/master/sw/example/coremark).

~~~
**CoreMark Setup**
Hardware:       32kB IMEM, 8kB DMEM, no caches, 100MHz clock
CoreMark:       2000 iterations, MEM_METHOD is MEM_STACK
Compiler:       RISCV32-GCC 10.1.0 (rv32i toolchain)
Compiler flags: default, see makefile; optimization -O3
~~~

Results generated for hardware version [`1.4.9.8`](https://github.com/stnolting/neorv32/blob/master/CHANGELOG.md).

| CPU (including `Zicsr` extension)           | Executable Size | CoreMark Score | CoreMarks/MHz | Total Clock Cycles | Executed Instructions | Average CPI |
|:--------------------------------------------|:---------------:|:--------------:|:-------------:|-------------------:|----------------------:|:-----------:|
| `rv32i`                                     |    28 756 bytes |          36.36 |    **0.3636** |         5595750503 |            1466028607 |    **3.82** |
| `rv32imc`                                   |    22 008 bytes |          68.97 |    **0.6897** |         2981786734 |             611814918 |    **4.87** |
| `rv32imc` + `FAST_MUL_EN` + `FAST_SHIFT_EN` |    22 008 bytes |          90.91 |    **0.9091** |         2265135174 |             611814948 |    **3.70** |

:information_source: The `FAST_MUL_EN` configuration uses DSPs for the multiplier of the `M` extension.
The `FAST_SHIFT_EN` configuration uses a barrel shifter for CPU shift operations.

[[back to top](#The-NEORV32-RISC-V-Processor)]



## Software Framework and Tooling

:books: In-depth detailed information regarding the software framework can be found in the
[online documentation - _"Software Framework"_](https://stnolting.github.io/neorv32/#_software_framework).

* [core libraries](https://github.com/stnolting/neorv32/tree/master/sw/lib) for high-level usage of the provided functions and peripherals
* application compilation based on GNU makefiles
* gcc-based toolchain ([pre-compiled toolchains available](https://github.com/stnolting/riscv-gcc-prebuilt))
* bootloader with UART interface console
* runtime environment for handling traps
* several [example programs](https://github.com/stnolting/neorv32/tree/master/sw/example) to get started including CoreMark, FreeRTOS and *Conway's Game of Life*
* `doxygen`-based documentation, available on [GitHub pages](https://stnolting.github.io/neorv32/sw/files.html)
* supports implementation using open source tooling (GHDL, Yosys and nextpnr; in the future Verilog-to-Routing); both, software and hardware can be
developed and debugged with open source tooling
* continuous Integration is available for:
  * allowing users to see the expected execution/output of the tools
  * ensuring specification compliance
  * catching regressions
  * providing ready-to-use and up-to-date bitstreams and documentation

[[back to top](#The-NEORV32-RISC-V-Processor)]



## Getting Started

This overview provides some *quick links* to the most important sections of the
[online Data Sheet](https://stnolting.github.io/neorv32) and the
[online User Guide](https://stnolting.github.io/neorv32/ug).

### :electric_plug: Hardware Overview

* [Rationale](https://stnolting.github.io/neorv32/#_rationale) - NEORV32: why, how come, what for

* [NEORV32 Processor](https://stnolting.github.io/neorv32/#_neorv32_processor_soc) - the SoC
  * [Top Entity - Signals](https://stnolting.github.io/neorv32/#_processor_top_entity_signals) - how to connect to the processor
  * [Top Entity - Generics](https://stnolting.github.io/neorv32/#_processor_top_entity_generics) - configuration options
  * [Address Space](https://stnolting.github.io/neorv32/#_address_space) - memory layout and boot configuration
  * [SoC Modules](https://stnolting.github.io/neorv32/#_processor_internal_modules) - available IO/peripheral modules and memories
  * [On-Chip Debugger](https://stnolting.github.io/neorv32/#_on_chip_debugger_ocd) - online & in-system debugging of the processor via JTAG

* [NEORV32 CPU](https://stnolting.github.io/neorv32/#_neorv32_central_processing_unit_cpu) - the RISC-V core
  * [RISC-V compatibility](https://stnolting.github.io/neorv32/#_risc_v_compatibility) - what is compatible to the specs. and what is not
  * [ISA and Extensions](https://stnolting.github.io/neorv32/#_instruction_sets_and_extensions) - available RISC-V ISA extensions
  * [CSRs](https://stnolting.github.io/neorv32/#_control_and_status_registers_csrs) - control and status registers
  * [Traps](https://stnolting.github.io/neorv32/#_traps_exceptions_and_interrupts) - interrupts and exceptions

### :floppy_disk: Software Overview

* [Core Libraries](https://stnolting.github.io/neorv32/#_core_libraries) - high-level functions for accessing the processor's peripherals
  * [Software Framework Documentation](https://stnolting.github.io/neorv32/sw/files.html) - `doxygen`-based documentation
* [Application Makefiles](https://stnolting.github.io/neorv32/#_application_makefile) - turning your application into an executable
* [Bootloader](https://stnolting.github.io/neorv32/#_bootloader) - the build-in NEORV32 bootloader

### :rocket: User Guides (see full [User Guide](https://stnolting.github.io/neorv32/ug/))

* [Toolchain Setup](https://stnolting.github.io/neorv32/ug/#_toolchain_setup) - install and setup RISC-V gcc
* [General Hardware Setup](https://stnolting.github.io/neorv32/ug/#_general_hardware_setup) - setup a new NEORV32 EDA project
* [General Software Setup](https://stnolting.github.io/neorv32/ug/#_general_software_framework_setup) - configure the software framework
* [Application Compilation](https://stnolting.github.io/neorv32/ug/#_application_program_compilation) - compile an application using `make`
* [Upload via Bootloader](https://stnolting.github.io/neorv32/ug/#_uploading_and_starting_of_a_binary_executable_image_via_uart) - upload and execute executables
* [Debugging via the On-Chip Debugger](https://stnolting.github.io/neorv32/ug/#_debugging_using_the_on_chip_debugger) - step through code *online* and *in-system*

### :copyright: Legal

* [Overview](https://stnolting.github.io/neorv32/#_legal) - license, disclaimer, proprietary notice, ...
* [Citing](https://stnolting.github.io/neorv32/#_citing) - citing information (DOI)
* [Impressum](https://github.com/stnolting/neorv32/blob/master/docs/impressum.md) - imprint (:de:)

[[back to top](#The-NEORV32-RISC-V-Processor)]



## Acknowledgements

**A big shoutout to all [contributors](https://github.com/stnolting/neorv32/graphs/contributors), who helped improving this project! :heart:**

[RISC-V](https://riscv.org/) - Instruction Sets Want To Be Free!

Continous integration provided by [:octocat: GitHub Actions](https://github.com/features/actions) and powered by [GHDL](https://github.com/ghdl/ghdl).

--------

Made with :coffee: in Hannover, Germany :eu:
