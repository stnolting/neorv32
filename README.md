[![NEORV32](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/figures/neorv32_logo_dark.png)](https://github.com/stnolting/neorv32)

# The NEORV32 RISC-V Processor

[![datasheet (pdf)](https://img.shields.io/badge/data%20sheet-PDF-ffbd00?longCache=true&style=flat-square&logo=asciidoctor)](https://github.com/stnolting/neorv32/releases/tag/nightly)
[![datasheet (html)](https://img.shields.io/badge/-HTML-ffbd00?longCache=true&style=flat-square)](https://stnolting.github.io/neorv32)
[![userguide (pdf)](https://img.shields.io/badge/user%20guide-PDF-ffbd00?longCache=true&style=flat-square&logo=asciidoctor)](https://github.com/stnolting/neorv32/releases/tag/nightly)
[![userguide (html)](https://img.shields.io/badge/-HTML-ffbd00?longCache=true&style=flat-square)](https://stnolting.github.io/neorv32/ug)
[![doxygen](https://img.shields.io/badge/doxygen-HTML-ffbd00?longCache=true&style=flat-square&logo=Doxygen)](https://stnolting.github.io/neorv32/sw/files.html)
[![Gitter](https://img.shields.io/badge/Chat-on%20gitter-4db797.svg?longCache=true&style=flat-square&logo=gitter&logoColor=e8ecef)](https://gitter.im/neorv32/community?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

1. [Overview](#1-Overview)
   * [Key Features](#Project-Key-Features)
   * [Status](#status)
2. [Processor/SoC Features](#2-NEORV32-Processor-Features)
   * [FPGA Implementation Results](#FPGA-Implementation-Results---Processor)
3. [CPU Features](#3-NEORV32-CPU-Features)
   * [Available ISA Extensions](#Available-ISA-Extensions)
   * [FPGA Implementation Results](#FPGA-Implementation-Results---CPU)
   * [Performance](#Performance)
4. [Software Framework & Tooling](#4-Software-Framework-and-Tooling)
5. [**Getting Started**](#5-Getting-Started) :rocket:



## 1. Overview

![neorv32 Overview](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/figures/neorv32_processor.png)

The NEORV32 Processor is a **customizable microcontroller-like system on chip (SoC)** that is based on the RISC-V NEORV32 CPU.
The project is intended as auxiliary processor in larger SoC designs or as *ready-to-go* stand-alone
custom microcontroller that even fits into a Lattice iCE40 UltraPlus 5k low-power FPGA running at 24 MHz.

Special focus is paid on **execution safety** to provide defined and predictable behavior at any time.
Therefore, the CPU ensures that all memory access are acknowledged and no invalid/malformed instructions
are executed. Whenever an unexpected situation occurs the application code is informed via precise and resumable hardware exceptions.

:thinking: Want to know more? Check out the [project's rationale](https://stnolting.github.io/neorv32/#_rationale).

:books: For detailed information take a look at the [NEORV32 documentation](https://stnolting.github.io/neorv32/) (online at GitHub-pages).

:label: The project's change log is available in [`CHANGELOG.md`](https://github.com/stnolting/neorv32/blob/master/CHANGELOG.md).
To see the changes between _official releases_ visit the project's [release page](https://github.com/stnolting/neorv32/releases).

:package: [Exemplary setups](https://github.com/stnolting/neorv32/tree/master/setups) targeting
various FPGA boards and toolchains to get you started.

:kite: Supported by upstream [Zephyr OS](https://docs.zephyrproject.org/latest/boards/riscv/neorv32/doc/index.html) and FreeRTOS.

:bulb: Feel free to open a [new issue](https://github.com/stnolting/neorv32/issues) or start a
[new discussion](https://github.com/stnolting/neorv32/discussions) if you have questions, comments, ideas or if something is
not working as expected. Or have a chat on our [gitter channel](https://gitter.im/neorv32/community).
See how to [contribute](https://github.com/stnolting/neorv32/blob/master/CONTRIBUTING.md).

:rocket: Check out the [quick links below](#5-Getting-Started) or directly jump to the
[*User Guide*](https://stnolting.github.io/neorv32/ug/) to get started
setting up your NEORV32 setup!


### Project Key Features

- [x] all-in-one package: [CPU](#3-NEORV32-CPU-Features) plus [SoC](#2-NEORV32-Processor-Features) plus [Software Framework & Tooling](#4-Software-Framework-and-Tooling)
- [x] completely described in behavioral, platform-independent VHDL - no primitives, macros, etc.
- [x] be as small as possible while being as RISC-V-compliant as possible
- [x] from zero to *printf("hello world!");* - completely open source and documented
- [x] easy to use even for FPGA/RISC-V starters – intended to work *out of the box*


### Status

[![release](https://img.shields.io/github/v/release/stnolting/neorv32?longCache=true&style=flat-square&logo=GitHub)](https://github.com/stnolting/neorv32/releases)
[![GitHub Pages](https://img.shields.io/website.svg?label=stnolting.github.io%2Fneorv32&longCache=true&style=flat-square&url=http%3A%2F%2Fstnolting.github.io%2Fneorv32%2Findex.html&logo=GitHub)](https://stnolting.github.io/neorv32)
[![Documentation](https://img.shields.io/github/workflow/status/stnolting/neorv32/Documentation/master?longCache=true&style=flat-square&label=Documentation&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions?query=workflow%3ADocumentation)
\
[![riscv-arch-test](https://img.shields.io/github/workflow/status/stnolting/neorv32/riscv-arch-test/master?longCache=true&style=flat-square&label=riscv-arch-test&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions?query=workflow%3Ariscv-arch-test)
[![Processor](https://img.shields.io/github/workflow/status/stnolting/neorv32/Processor/master?longCache=true&style=flat-square&label=Processor&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions?query=workflow%3AProcessor)
[![Implementation](https://img.shields.io/github/workflow/status/stnolting/neorv32/Implementation/master?longCache=true&style=flat-square&label=Implementation&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions?query=workflow%3AImplementation)
[![Windows](https://img.shields.io/github/workflow/status/stnolting/neorv32/Windows/master?longCache=true&style=flat-square&label=Windows&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions?query=workflow%3AWindows)

[[back to top](#The-NEORV32-RISC-V-Processor)]


## 2. NEORV32 Processor Features

The NEORV32 Processor (top entity: [`rtl/core/neorv32_top.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/core/neorv32_top.vhd))
provides a full-featured SoC build around the NEORV32 CPU. It is highly configurable via generics
to allow a flexible customization according to your needs. Note that all modules listed below are _optional_.

**Memory**

* processor-internal data and instruction memories ([DMEM](https://stnolting.github.io/neorv32/#_data_memory_dmem) /
[IMEM](https://stnolting.github.io/neorv32/#_instruction_memory_imem)) &
cache ([iCACHE](https://stnolting.github.io/neorv32/#_processor_internal_instruction_cache_icache))
* bootloader ([BOOTLDROM](https://stnolting.github.io/neorv32/#_bootloader_rom_bootrom)) with serial user interface
  * allows booting application code via UART or from external SPI flash

**Timers**

* machine system timer, 64-bit ([MTIME](https://stnolting.github.io/neorv32/#_machine_system_timer_mtime)), RISC-V spec. compatible
* general purpose 32-bit timer ([GPTMR](https://stnolting.github.io/neorv32/#_general_purpose_timer_gptmr))
* watchdog timer ([WDT](https://stnolting.github.io/neorv32/#_watchdog_timer_wdt))

**Input/Output**

* standard serial interfaces
([UART](https://stnolting.github.io/neorv32/#_primary_universal_asynchronous_receiver_and_transmitter_uart0),
[SPI](https://stnolting.github.io/neorv32/#_serial_peripheral_interface_controller_spi),
[TWI / I²C](https://stnolting.github.io/neorv32/#_two_wire_serial_interface_controller_twi))
* general purpose [GPIO](https://stnolting.github.io/neorv32/#_general_purpose_input_and_output_port_gpio) and
[PWM](https://stnolting.github.io/neorv32/#_pulse_width_modulation_controller_pwm)
* smart LED interface ([NEOLED](https://stnolting.github.io/neorv32/#_smart_led_interface_neoled)) to directly drive _NeoPixel(TM)_ LEDs

**SoC Connectivity**

* 32-bit external bus interface, Wishbone b4 compatible
([WISHBONE](https://stnolting.github.io/neorv32/#_processor_external_memory_interface_wishbone_axi4_lite))
  * [wrapper](https://github.com/stnolting/neorv32/blob/master/rtl/system_integration/neorv32_SystemTop_axi4lite.vhd) for AXI4-Lite master interface
  * [wrapper](https://github.com/stnolting/neorv32/blob/master/rtl/system_integration/neorv32_SystemTop_AvalonMM.vhd) for Avalon-MM master interface
* 32-bit stream link interface with up to 8 independent RX and TX links
([SLINK](https://stnolting.github.io/neorv32/#_stream_link_interface_slink))
  * AXI4-Stream compatible
* external interrupt controller with up to 32 channels
([XIRQ](https://stnolting.github.io/neorv32/#_external_interrupt_controller_xirq))
* custom functions subsystem ([CFS](https://stnolting.github.io/neorv32/#_custom_functions_subsystem_cfs))
for tightly-coupled custom co-processor extensions

**Advanced**

* _true random_ number generator ([TRNG](https://stnolting.github.io/neorv32/#_true_random_number_generator_trng))
* on-chip debugger ([OCD](https://stnolting.github.io/neorv32/#_on_chip_debugger_ocd)) accessible via JTAG interface - implementing
the [*Minimal RISC-V Debug Specification Version 0.13.2*](https://github.com/riscv/riscv-debug-spec)
and compatible with *OpenOCD* and *gdb*
* bus keeper to monitor the CPU's bus transactions ([BUSKEEPER](https://stnolting.github.io/neorv32/#_internal_bus_monitor_buskeeper))

[[back to top](#The-NEORV32-RISC-V-Processor)]


### FPGA Implementation Results - Processor

The hardware resources used by a specific processor setup is defined by the implemented CPU extensions,
the configuration of the peripheral modules and some "glue logic".
Section [_FPGA Implementation Results - Processor Modules_](https://stnolting.github.io/neorv32/#_processor_modules)
of the online datasheet shows the resource utilization of each optional processor module to allow an
estimation of the actual setup's hardware requirements.

The [`setups`](https://github.com/stnolting/neorv32/tree/master/setups) folder provides exemplary FPGA
setups targeting various FPGA boards and toolchains. These setups also provide resource utilization reports for different
SoC configurations. The latest utilization reports for those setups can be found in the report of the
[Implementation Workflow](https://github.com/stnolting/neorv32/actions/workflows/Implementation.yml).

[[back to top](#The-NEORV32-RISC-V-Processor)]



## 3. NEORV32 CPU Features

The CPU (top entity: [`rtl/core/neorv32_cpu.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/core/neorv32_cpu.vhd))
implements the RISC-V 32-bit `rv32` ISA with optional extensions (see below). It is compatible to subsets of the
*Unprivileged ISA Specification* [(Version 2.2)](https://github.com/stnolting/neorv32/blob/master/docs/references/riscv-spec.pdf)
and the *Privileged Architecture Specification* [(Version 1.12-draft)](https://github.com/stnolting/neorv32/blob/master/docs/references/riscv-privileged.pdf).
Compatibility is checked by passing the [official RISC-V architecture tests](https://github.com/riscv/riscv-arch-test)
(see [`sim/README`](sim/README.md)).

The core is a little-endian Von-Neumann machine implemented as multi-cycle architecture.
However, the CPU's _front end_ (instruction fetch) and _back end_ (instruction execution) can work independently to increase performance.
Currently, three privilege levels (`machine` and optional `user` and `debug_mode`) are supported. The CPU implements all three standard RISC-V machine
interrupts (`MTI`, `MEI`, `MSI`) plus 16 _fast interrupt requests_ as custom extensions.
It also supports **all** standard RISC-V exceptions (instruction/load/store misaligned address & bus access fault, illegal
instruction, breakpoint, environment calls).

:books: In-depth detailed information regarding the CPU can be found in the
[_Data Sheet: NEORV32 Central Processing Unit_](https://stnolting.github.io/neorv32/#_neorv32_central_processing_unit_cpu).


### Available ISA Extensions

Currently, the following _optional_ RISC-V-compatible ISA extensions are implemented (linked to the according
documentation section). Note that the `X` extension is always enabled.

**RV32
[[`I`](https://stnolting.github.io/neorv32/#_i_base_integer_isa)/
[`E`](https://stnolting.github.io/neorv32/#_e_embedded_cpu)]
[[`A`](https://stnolting.github.io/neorv32/#_a_atomic_memory_access)]
[[`B`](https://stnolting.github.io/neorv32/#_b_bit_manipulation_operations)]
[[`C`](https://stnolting.github.io/neorv32/#_c_compressed_instructions)]
[[`M`](https://stnolting.github.io/neorv32/#_m_integer_multiplication_and_division)]
[[`U`](https://stnolting.github.io/neorv32/#_u_less_privileged_user_mode)]
[[`X`](https://stnolting.github.io/neorv32/#_x_neorv32_specific_custom_extensions)]
[[`Zfinx`](https://stnolting.github.io/neorv32/#_zfinx_single_precision_floating_point_operations)]
[[`Zicsr`](https://stnolting.github.io/neorv32/#_zicsr_control_and_status_register_access_privileged_architecture)]
[[`Zicntr`](https://stnolting.github.io/neorv32/#_zicntr_cpu_base_counters)]
[[`Zihpm`](https://stnolting.github.io/neorv32/#_zihpm_hardware_performance_monitors)]
[[`Zifencei`](https://stnolting.github.io/neorv32/#_zifencei_instruction_stream_synchronization)]
[[`Zmmul`](https://stnolting.github.io/neorv32/#_zmmul_integer_multiplication)]
[[`PMP`](https://stnolting.github.io/neorv32/#_pmp_physical_memory_protection)]
[[`DEBUG`](https://stnolting.github.io/neorv32/#_cpu_debug_mode)]**

:warning: The `B`, `Zfinx` and `Zmmul` RISC-V extensions are frozen but not officially ratified yet. Hence, there is no
upstream gcc support. To circumvent this, the NEORV32 software framework provides _intrinsic libraries_ for these extensions.

[[back to top](#The-NEORV32-RISC-V-Processor)]


### FPGA Implementation Results - CPU

Implementation results for _exemplary_ CPU configuration generated for an **Intel Cyclone IV EP4CE22F17C6N FPGA**
using **Intel Quartus Prime Lite 20.1** ("balanced implementation, Slow 1200mV 0C Model").

| CPU Configuration (version [1.5.7.10](https://github.com/stnolting/neorv32/blob/master/CHANGELOG.md)) | LEs | FFs | Memory bits | DSPs (9-bit) | f_max |
|:------------------------|:----:|:----:|:----:|:-:|:-------:|
| `rv32i`                 |  806 |  359 | 1024 | 0 | 125 MHz |
| `rv32i_Zicsr_Zicntr`    | 1729 |  813 | 1024 | 0 | 124 MHz |
| `rv32imac_Zicsr_Zicntr` | 2511 | 1074 | 1024 | 0 | 124 MHz |

:information_source: An incremental list of CPU extension's hardware utilization can found in the
[_Data Sheet: FPGA Implementation Results - CPU_](https://stnolting.github.io/neorv32/#_cpu).

:information_source: The CPU and SoC provide advanced options to optimize for performance, area or energy.
See [_User Guide: Application-Specific Processor Configuration_](https://stnolting.github.io/neorv32/ug/#_application_specific_processor_configuration)
for more information.

[[back to top](#The-NEORV32-RISC-V-Processor)]


### Performance

The NEORV32 CPU is based on a two-stages pipeline architecture (fetch and execute).
The average CPI (cycles per instruction) depends on the instruction mix of a specific applications and also on the
available CPU extensions.

The following table shows the performance results (scores and average CPI) for _exemplary_ CPU configurations executing
2000 iterations of the [CoreMark](https://github.com/stnolting/neorv32/blob/master/sw/example/coremark) CPU benchmark.

| CPU Configuration (version [1.5.7.10](https://github.com/stnolting/neorv32/blob/master/CHANGELOG.md)) | CoreMark Score | CoreMarks/MHz | Average CPI |
|:------------------------------------------------|:-----:|:----------:|:--------:|
| _small_ (`rv32i_Zicsr`)                         | 33.89 | **0.3389** | **4.04** |
| _medium_ (`rv32imc_Zicsr`)                      | 62.50 | **0.6250** | **5.34** |
| _performance_ (`rv32imc_Zicsr` + perf. options) | 95.23 | **0.9523** | **3.54** |

:information_source: More information regarding the CPU performance can be found in the
[_Data Sheet: CPU Performance_](https://stnolting.github.io/neorv32/#_cpu_performance).

[[back to top](#The-NEORV32-RISC-V-Processor)]



## 4. Software Framework and Tooling

* [core libraries](https://github.com/stnolting/neorv32/tree/master/sw/lib) for high-level usage of the provided functions and peripherals
* application compilation based on GNU makefiles
* gcc-based toolchain ([pre-compiled toolchains available](https://github.com/stnolting/riscv-gcc-prebuilt))
* [SVD file](https://github.com/stnolting/neorv32/tree/master/sw/svd) for advanced debugging and IDE integration
* bootloader with UART interface console
* runtime environment for handling traps
* several [example programs](https://github.com/stnolting/neorv32/tree/master/sw/example) to get started including CoreMark, FreeRTOS and Conway's Game of Life
* doxygen-based documentation, available on [GitHub pages](https://stnolting.github.io/neorv32/sw/files.html)
* supports implementation using open source tooling (GHDL, Yosys and nextpnr; in the future: "Verilog-to-Routing") - both, software and hardware can be
developed and debugged with open source tooling
* [continuous integration](https://github.com/stnolting/neorv32/actions) is available for:
  * allowing users to see the expected execution/output of the tools
  * ensuring specification compliance
  * catching regressions
  * providing ready-to-use and up-to-date bitstreams and documentation

:books: Want to know more? Check out [_Data Sheet: Software Framework_](https://stnolting.github.io/neorv32/#_software_framework).

[[back to top](#The-NEORV32-RISC-V-Processor)]



## 5. Getting Started

This overview provides some *quick links* to the most important sections of the
[online Data Sheet](https://stnolting.github.io/neorv32) and the
[online User Guide](https://stnolting.github.io/neorv32/ug).

### :electric_plug: Hardware Overview

* [Rationale](https://stnolting.github.io/neorv32/#_rationale) - NEORV32: why, how come, what for

* [NEORV32 Processor](https://stnolting.github.io/neorv32/#_neorv32_processor_soc) - the SoC
  * [Top Entity - Signals](https://stnolting.github.io/neorv32/#_processor_top_entity_signals) - how to connect to the processor
  * [Top Entity - Generics](https://stnolting.github.io/neorv32/#_processor_top_entity_generics) - configuration options
  * [Address Space](https://stnolting.github.io/neorv32/#_address_space) - memory layout and boot configuration
  * [SoC Modules](https://stnolting.github.io/neorv32/#_processor_internal_modules) - available peripheral modules and memories
  * [On-Chip Debugger](https://stnolting.github.io/neorv32/#_on_chip_debugger_ocd) - online & in-system debugging of the processor via JTAG

* [NEORV32 CPU](https://stnolting.github.io/neorv32/#_neorv32_central_processing_unit_cpu) - the CPU
  * [RISC-V compatibility](https://stnolting.github.io/neorv32/#_risc_v_compatibility) - what is compatible to the specs. and what is not
  * [Full Virtualization](https://stnolting.github.io/neorv32/#_full_virtualization) - hardware execution safety
  * [ISA and Extensions](https://stnolting.github.io/neorv32/#_instruction_sets_and_extensions) - available RISC-V ISA extensions
  * [CSRs](https://stnolting.github.io/neorv32/#_control_and_status_registers_csrs) - control and status registers
  * [Traps](https://stnolting.github.io/neorv32/#_traps_exceptions_and_interrupts) - interrupts and exceptions

### :floppy_disk: Software Overview

* [Example Programs](https://github.com/stnolting/neorv32/tree/master/sw/example) - test program execution on your setup
* [Core Libraries](https://stnolting.github.io/neorv32/#_core_libraries) - high-level functions for accessing the processor's peripherals
  * [Software Framework Documentation](https://stnolting.github.io/neorv32/sw/files.html) - `doxygen`-based documentation
* [Application Makefiles](https://stnolting.github.io/neorv32/#_application_makefile) - turning your application into an executable
* [Bootloader](https://stnolting.github.io/neorv32/#_bootloader) - the build-in NEORV32 bootloader

### :rocket: User Guide

* [Toolchain Setup](https://stnolting.github.io/neorv32/ug/#_software_toolchain_setup) - install and setup RISC-V gcc
* [General Hardware Setup](https://stnolting.github.io/neorv32/ug/#_general_hardware_setup) - setup a new NEORV32 EDA project
* [General Software Setup](https://stnolting.github.io/neorv32/ug/#_general_software_framework_setup) - configure the software framework
* [Application Compilation](https://stnolting.github.io/neorv32/ug/#_application_program_compilation) - compile an application using `make`
* [Upload via Bootloader](https://stnolting.github.io/neorv32/ug/#_uploading_and_starting_of_a_binary_executable_image_via_uart) - upload and execute executables
* [Application-Specific Processor Configuration](https://stnolting.github.io/neorv32/ug/#_application_specific_processor_configuration) - tailor the processor to your needs
* [Adding Custom Hardware Modules](https://stnolting.github.io/neorv32/ug/#_adding_custom_hardware_modules) - add _your_ custom hardware
* [Debugging via the On-Chip Debugger](https://stnolting.github.io/neorv32/ug/#_debugging_using_the_on_chip_debugger) - step through code *online* and *in-system*
* [Simulation](https://stnolting.github.io/neorv32/ug/#_simulating_the_processor) - simulate the whole SoC
  * [Hello World!](https://stnolting.github.io/neorv32/ug/index.html#_hello_world) - run a quick _"hello world"_ simulation

### :copyright: Legal

[![license](https://img.shields.io/github/license/stnolting/neorv32?longCache=true&style=flat-square)](https://github.com/stnolting/neorv32/blob/master/LICENSE)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.5018888.svg)](https://doi.org/10.5281/zenodo.5018888)

* [Overview](https://stnolting.github.io/neorv32/#_legal) - license, disclaimer, limitation of liability for external links, proprietary notice, ...
* [Citing](https://stnolting.github.io/neorv32/#_citing) - citing information
* [Impressum](https://github.com/stnolting/neorv32/blob/master/docs/impressum.md) - imprint

[[back to top](#The-NEORV32-RISC-V-Processor)]



## Acknowledgements

**A big shout-out goes to the community and all the [contributors](https://github.com/stnolting/neorv32/graphs/contributors), who helped improving this project! :heart:**

[RISC-V](https://riscv.org/) - Instruction Sets Want To Be Free!

Continuous integration provided by [:octocat: GitHub Actions](https://github.com/features/actions) and powered by [GHDL](https://github.com/ghdl/ghdl).
