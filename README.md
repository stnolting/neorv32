[![NEORV32](docs/figures/neorv32_logo_riscv.png)](https://github.com/stnolting/neorv32)

# The NEORV32 RISC-V Processor

[![datasheet (pdf)](https://img.shields.io/badge/data%20sheet-PDF-ffbd00?longCache=true&style=flat-square&logo=asciidoctor&colorA=273274)](https://github.com/stnolting/neorv32/releases/tag/nightly)
[![datasheet (html)](https://img.shields.io/badge/-HTML-ffbd00?longCache=true&style=flat-square)](https://stnolting.github.io/neorv32)
[![userguide (pdf)](https://img.shields.io/badge/user%20guide-PDF-ffbd00?longCache=true&style=flat-square&logo=asciidoctor&colorA=273274)](https://github.com/stnolting/neorv32/releases/tag/nightly)
[![userguide (html)](https://img.shields.io/badge/-HTML-ffbd00?longCache=true&style=flat-square)](https://stnolting.github.io/neorv32/ug)
[![doxygen](https://img.shields.io/badge/doxygen-HTML-ffbd00?longCache=true&style=flat-square&logo=Doxygen&colorA=273274)](https://stnolting.github.io/neorv32/sw/files.html)

1. [Overview](#1-overview)
   * [Key Features](#key-features)
   * [Project Status](#project-status)
2. [Features](#2-features)
3. [FPGA Implementation Results](#3-fpga-implementation-results)
4. [Performance](#4-performance)
5. [**Getting Started**](#5-getting-started) :rocket:


## 1. Overview

![neorv32 Overview](docs/figures/neorv32_processor.png)

The NEORV32 Processor is a **customizable microcontroller-like system on chip (SoC)** built around the NEORV32
[RISC-V](https://riscv.org/) CPU that is written in **platform-independent VHDL**. The processor is intended as auxiliary
controller in larger SoC designs or as tiny and customized microcontroller that even fits into a
Lattice iCE40 UltraPlus low-power & low-density FPGA. The project is intended to work _out of the box_ and targets
FPGA / RISC-V beginners as well as advanced users.

Special focus is paid on **execution safety** to provide defined and predictable behavior at any time.
For example, the CPU ensures _all_ memory accesses are properly acknowledged and _all_ invalid/malformed
instructions are always detected as such. Whenever an unexpected state occurs the application software is
informed via _precise_ and resumable hardware exceptions.

* :recycle: Looking for an **all-Verilog** version? Have a look at [neorv32-verilog](https://github.com/stnolting/neorv32-verilog).
* :heavy_check_mark: [Continuous integration](#project-status) to check for regressions (including RISC-V ISA compatibility check using **RISCOF**).
* :open_file_folder: [Exemplary setups](https://github.com/stnolting/neorv32-setups) and
[community projects](https://github.com/stnolting/neorv32-setups/blob/main/README.md#Community-Projects)
targeting various FPGA boards and toolchains to get started.
* :package: The entire processor is also available as [Vivado IP Block](https://stnolting.github.io/neorv32/ug/#_packaging_the_processor_as_vivado_ip_block).
* :kite: Support for [FreeRTOS](https://github.com/stnolting/neorv32-freertos),
[Zephyr OS](https://docs.zephyrproject.org/latest/boards/others/neorv32/doc/index.html) and
[LiteX](https://github.com/enjoy-digital/litex/wiki/CPUs#risc-v---neorv32) SoC Builder Framework.
* :desktop_computer: Pre-configured [Eclipse project](https://stnolting.github.io/neorv32/ug/#_eclipse_ide) for developing and debugging code using an IDE.
* :label: The project's change log is available in [CHANGELOG.md](https://github.com/stnolting/neorv32/blob/main/CHANGELOG.md).
* :rocket: Check out the [quick links below](#5-getting-started) and the
[*User Guide*](https://stnolting.github.io/neorv32/ug/) to get started setting up _your_ NEORV32 processor!
* :books: For detailed information see the [NEORV32 online documentation](https://stnolting.github.io/neorv32/).
* :interrobang: Want to know more? Check out the [project's rationale](https://stnolting.github.io/neorv32/#_rationale).

Feel free to open a new [issue](https://github.com/stnolting/neorv32/issues) or start a new
[discussion](https://github.com/stnolting/neorv32/discussions) if you have questions, comments, ideas, feedback or if something is
not working as expected. See how to [contribute](https://github.com/stnolting/neorv32/blob/main/CONTRIBUTING.md).

### Key Features

- [x] all-in-one package: **CPU** + **SoC** + **Software Framework & Tooling**
- [x] completely described in behavioral, platform-independent VHDL - **no** platform-specific primitives, macros, attributes, etc.; an all-Verilog "version" is also [available](https://github.com/stnolting/neorv32-verilog)
- [x] extensive configuration options for adapting the processor to the requirements of the application (on CPU, processor and system level)
- [x] aims to be as small as possible while being as RISC-V-compliant as possible - with a reasonable area-vs-performance trade-off
- [x] FPGA friendly (e.g. _all_ internal memories can be mapped to block RAM - including the CPU's register file)
- [x] optimized for high clock frequencies to ease integration / timing closure
- [x] from zero to _"hello world!"_ - completely open source and documented (on software and hardware side)
- [x] easy to use even for FPGA / RISC-V starters – intended to work _out of the box_

### Project Status

[![release](https://img.shields.io/github/v/release/stnolting/neorv32?longCache=true&style=flat-square&logo=GitHub)](https://github.com/stnolting/neorv32/releases)
[![commits-since-latest-release](https://img.shields.io/github/commits-since/stnolting/neorv32/latest?longCache=true&style=flat-square&logo=GitHub)](https://github.com/stnolting/neorv32/activity)

| Task / Subproject | Repository | CI Status |
|:------------------|:-----------|:----------|
| GitHub pages (docs)      | [neorv32](https://github.com/stnolting/neorv32)                       | [![GitHub Pages](https://img.shields.io/website.svg?label=stnolting.github.io%2Fneorv32&longCache=true&style=flat-square&url=http%3A%2F%2Fstnolting.github.io%2Fneorv32%2Findex.html&logo=GitHub)](https://stnolting.github.io/neorv32) |
| Build documentation      | [neorv32](https://github.com/stnolting/neorv32)                       | [![Documentation](https://img.shields.io/github/actions/workflow/status/stnolting/neorv32/Documentation.yml?branch=main&longCache=true&style=flat-square&label=Documentation&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions?query=workflow%3ADocumentation) |
| Processor verification   | [neorv32](https://github.com/stnolting/neorv32)                       | [![Processor](https://img.shields.io/github/actions/workflow/status/stnolting/neorv32/Processor.yml?branch=main&longCache=true&style=flat-square&label=Processor%20Check&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions?query=workflow%3AProcessor) |
| RISCOF core verification | [neorv32-riscof](https://github.com/stnolting/neorv32-riscof)         | [![neorv32-riscof](https://img.shields.io/github/actions/workflow/status/stnolting/neorv32-riscof/main.yml?branch=main&longCache=true&style=flat-square&label=neorv32-riscof&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32-riscof/actions/workflows/main.yml) |
| FPGA implementations     | [neorv32-setups](https://github.com/stnolting/neorv32-setups)         | [![Implementation](https://img.shields.io/github/actions/workflow/status/stnolting/neorv32-setups/Implementation.yml?branch=main&longCache=true&style=flat-square&label=Implementation&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32-setups/actions?query=workflow%3AImplementation) |
| All-Verilog version      | [neorv32-verilog](https://github.com/stnolting/neorv32-verilog)       | [![neorv32-verilog](https://img.shields.io/github/actions/workflow/status/stnolting/neorv32-verilog/main.yml?branch=main&longCache=true&style=flat-square&label=neorv32-verilog&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32-verilog/actions/workflows/main.yml) |
| FreeRTOS port            | [neorv32-freertos](https://github.com/stnolting/neorv32-freertos)     | [![neorv32-freertos](https://img.shields.io/github/actions/workflow/status/stnolting/neorv32-freertos/main.yml?branch=main&longCache=true&style=flat-square&label=neorv32-freertos%20sim&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32-freertos/actions/workflows/main.yml) |
| Prebuilt GCC toolchains  | [riscv-gcc-prebuilt](https://github.com/stnolting/riscv-gcc-prebuilt) | [![Prebuilt_Toolchains](https://img.shields.io/github/actions/workflow/status/stnolting/riscv-gcc-prebuilt/main.yml?branch=main&longCache=true&style=flat-square&label=Prebuilt%20Toolchains&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/riscv-gcc-prebuilt/actions/workflows/main.yml) |

The processor passes the official RISC-V architecture tests to ensure compatibility with the RISC-V ISA specs., which is checked by the
[neorv32-riscof](https://github.com/stnolting/neorv32-riscof) repository. It can successfully run _any_ C program
(for example from the [`sw/example`](https://github.com/stnolting/neorv32/tree/main/sw/example) folder) including CoreMark
and FreeRTOS and can be synthesized for _any_ target technology - [tested](https://github.com/stnolting/neorv32-setups)
on Intel, AMD, Cologne Chip and Lattice FPGAs. The conversion into a single, plain-Verilog module file is automatically checked by the
[neorv32-verilog](https://github.com/stnolting/neorv32-verilog) repository.


## 2. Features

The NEORV32 Processor provides a full-featured microcontroller-like SoC build around the NEORV32 CPU.
By using generics the design is highly configurable and allows a flexible customization to tailor the
setup according to your needs. Note that all of the following SoC modules are entirely _optional_.

**CPU Core**

* [![RISCV-ARCHID](https://img.shields.io/badge/RISC--V%20Architecture%20ID-19-000000.svg?longCache=true&style=flat-square&logo=riscv&colorA=273274&colorB=fbb517)](https://github.com/riscv/riscv-isa-manual/blob/master/marchid.md)
* RISC-V 32-bit little-endian single- or SMP-dual-core pipelined/multi-cycle modified Harvard architecture
* configurable [instruction sets and extensions](https://stnolting.github.io/neorv32/#_instruction_sets_and_extensions):
\
`RV32`
[[`I`](https://stnolting.github.io/neorv32/#_i_isa_extension)/[`E`](https://stnolting.github.io/neorv32/#_e_isa_extension)]
[[`M`](https://stnolting.github.io/neorv32/#_m_isa_extension)]
[[`C`](https://stnolting.github.io/neorv32/#_c_isa_extension)]
[[`B`](https://stnolting.github.io/neorv32/#_b_isa_extension)]
[[`U`](https://stnolting.github.io/neorv32/#_u_isa_extension)]
[[`X`](https://stnolting.github.io/neorv32/#_x_isa_extension)]
[[`Zaamo`](https://stnolting.github.io/neorv32/#_zaamo_isa_extension)]
[[`Zba`](https://stnolting.github.io/neorv32/#_zba_isa_extension)]
[[`Zbb`](https://stnolting.github.io/neorv32/#_zbb_isa_extension)]
[[`Zbkb`](https://stnolting.github.io/neorv32/#_zbkb_isa_extension)]
[[`Zbkc`](https://stnolting.github.io/neorv32/#_zbkc_isa_extension)]
[[`Zbkx`](https://stnolting.github.io/neorv32/#_zbkx_isa_extension)]
[[`Zbs`](https://stnolting.github.io/neorv32/#_zbs_isa_extension)]
[[`Zicntr`](https://stnolting.github.io/neorv32/#_zicntr_isa_extension)]
[[`Zicond`](https://stnolting.github.io/neorv32/#_zicond_isa_extension)]
[[`Zicsr`](https://stnolting.github.io/neorv32/#_zicsr_isa_extension)]
[[`Zifencei`](https://stnolting.github.io/neorv32/#_zifencei_isa_extension)]
[[`Zihpm`](https://stnolting.github.io/neorv32/#_zihpm_isa_extension)]
[[`Zfinx`](https://stnolting.github.io/neorv32/#_zfinx_isa_extension)]
[[`Zkn`](https://stnolting.github.io/neorv32/#_zkn_isa_extension)]
[[`Zknd`](https://stnolting.github.io/neorv32/#_zknd_isa_extension)]
[[`Zkne`](https://stnolting.github.io/neorv32/#_zkne_isa_extension)]
[[`Zknh`](https://stnolting.github.io/neorv32/#_zknh_isa_extension)]
[[`Zkt`](https://stnolting.github.io/neorv32/#_zkt_isa_extension)]
[[`Zks`](https://stnolting.github.io/neorv32/#_zks_isa_extension)]
[[`Zksed`](https://stnolting.github.io/neorv32/#_zksed_isa_extension)]
[[`Zksh`](https://stnolting.github.io/neorv32/#_zksh_isa_extension)]
[[`Zmmul`](https://stnolting.github.io/neorv32/#_zmmul_isa_extension)]
[[`Zxcfu`](https://stnolting.github.io/neorv32/#_zxcfu_isa_extension)]
[[`Sdext`](https://stnolting.github.io/neorv32/#_sdext_isa_extension)]
[[`Sdtrig`](https://stnolting.github.io/neorv32/#_sdtrig_isa_extension)]
[[`Smpmp`](https://stnolting.github.io/neorv32/#_smpmp_isa_extension)]
* compatible to subsets of the RISC-V
*Unprivileged ISA Specification* ([pdf](https://github.com/stnolting/neorv32/blob/main/docs/references/riscv-spec.pdf))
and *Privileged Architecture Specification* ([pdf](https://github.com/stnolting/neorv32/blob/main/docs/references/riscv-privileged.pdf)).
* `machine` and `user` privilege modes
* implements **all** standard RISC-V exceptions and interrupts + 16 fast interrupt request channels as NEORV32-specific extension
* custom functions unit ([CFU](https://stnolting.github.io/neorv32/#_custom_functions_unit_cfu) as `Zxcfu` ISA extension)
for **custom RISC-V instructions**;
* _intrinsic_ libraries for CPU extensions that are not yet supported by GCC

**Memories**

* processor-internal data and instruction memories ([DMEM](https://stnolting.github.io/neorv32/#_data_memory_dmem) /
[IMEM](https://stnolting.github.io/neorv32/#_instruction_memory_imem)) &
caches ([iCACHE](https://stnolting.github.io/neorv32/#_processor_internal_instruction_cache_icache) and
[dCACHE](https://stnolting.github.io/neorv32/#_processor_internal_data_cache_dcache))
* pre-installed bootloader ([BOOTLDROM](https://stnolting.github.io/neorv32/#_bootloader_rom_bootrom)) with serial user interface;
allows booting application code via UART or from external SPI flash

**Timers and Counters**

* core local interruptor ([CLINT](https://stnolting.github.io/neorv32/#_core_local_interruptor_clint)), RISC-V-compatible
* 32-bit general purpose timer ([GPTMR](https://stnolting.github.io/neorv32/#_general_purpose_timer_gptmr))
* watchdog timer ([WDT](https://stnolting.github.io/neorv32/#_watchdog_timer_wdt))

**Input / Output**

* standard serial interfaces
([UART](https://stnolting.github.io/neorv32/#_primary_universal_asynchronous_receiver_and_transmitter_uart0),
[SPI](https://stnolting.github.io/neorv32/#_serial_peripheral_interface_controller_spi) (SPI host),
[SDI](https://stnolting.github.io/neorv32/#_serial_data_interface_controller_sdi) (SPI device),
[TWI](https://stnolting.github.io/neorv32/#_two_wire_serial_interface_controller_twi) (I²C host),
[TWD](https://stnolting.github.io/neorv32/#_two_wire_serial_device_controller_twd) (I²C device),
[ONEWIRE/1-Wire](https://stnolting.github.io/neorv32/#_one_wire_serial_interface_controller_onewire))
* general purpose IOs ([GPIO](https://stnolting.github.io/neorv32/#_general_purpose_input_and_output_port_gpio)) and
[PWM](https://stnolting.github.io/neorv32/#_pulse_width_modulation_controller_pwm)
* smart LED interface ([NEOLED](https://stnolting.github.io/neorv32/#_smart_led_interface_neoled)) to directly control NeoPixel(TM) LEDs

**SoC Connectivity**

* 32-bit external bus interface - Wishbone b4 compatible
([XBUS](https://stnolting.github.io/neorv32/#_processor_external_bus_interface_xbus)) with optional cache (XCACHE);
[wrappers](https://github.com/stnolting/neorv32/blob/main/rtl/system_integration) for AXI4-Lite and Avalon-MM host interfaces
* stream link interface with independent RX and TX channels - AXI4-Stream compatible
([SLINK](https://stnolting.github.io/neorv32/#_stream_link_interface_slink))
* external interrupts controller with up to 32 channels
([XIRQ](https://stnolting.github.io/neorv32/#_external_interrupt_controller_xirq))

**Advanced**

* true-random number generator ([TRNG](https://stnolting.github.io/neorv32/#_true_random_number_generator_trng)) based
on the [neoTRNG](https://github.com/stnolting/neoTRNG)
* execute-in-place module ([XIP](https://stnolting.github.io/neorv32/#_execute_in_place_module_xip)) to execute code right out of a SPI flash
* custom functions subsystem ([CFS](https://stnolting.github.io/neorv32/#_custom_functions_subsystem_cfs))
for custom tightly-coupled co-processors, accelerators or interfaces
* direct memory access controller ([DMA](https://stnolting.github.io/neorv32/#_direct_memory_access_controller_dma)) for CPU-independent
data transfers and conversions
* cyclic redundancy check unit ([CRC](https://stnolting.github.io/neorv32/#_cyclic_redundancy_check_crc)) to test
data integrity (CRC8/16/32)

**Debugging**

* on-chip debugger ([OCD](https://stnolting.github.io/neorv32/#_on_chip_debugger_ocd)) accessible via standard JTAG interface
* compatible to the "Minimal RISC-V Debug Specification Version 1.0"
* compatible with **OpenOCD**, **GDB** and **Segger Embedded Studio**
* RISC-V [trigger module](https://stnolting.github.io/neorv32/#_trigger_module) for hardware-assisted breakpoints
* optional authentication module to implement custom security mechanisms


## 3. FPGA Implementation Results

Implementation results for **exemplary CPU configurations** generated for an Intel Cyclone IV `EP4CE22F17C6` FPGA
using Intel Quartus Prime Lite 21.1 (no timing constrains, _balanced optimization_, f_max from _Slow 1200mV 0C Model_).

| CPU Configuration (version [1.7.8.5](https://github.com/stnolting/neorv32/blob/main/CHANGELOG.md)) | LEs | FFs | Memory bits | DSPs | f_max |
|:-----------------------|:----:|:----:|:----:|:-:|:-------:|
| `rv32i_Zicsr`          | 1223 |  607 | 1024 | 0 | 130 MHz |
| `rv32i_Zicsr_Zicntr`   | 1578 |  773 | 1024 | 0 | 130 MHz |
| `rv32imc_Zicsr_Zicntr` | 2338 |  992 | 1024 | 0 | 130 MHz |

An incremental list of CPU extensions and processor modules can be found in the [Data Sheet: FPGA Implementation Results](https://stnolting.github.io/neorv32/#_fpga_implementation_results).


## 4. Performance

The NEORV32 CPU is based on a two-stages pipelined/multi-cycle architecture (fetch and execute).
The following table shows the performance results (scores and average CPI) for exemplary CPU configurations (no caches) executing
2000 iterations of the [CoreMark](https://github.com/stnolting/neorv32/blob/main/sw/example/coremark) CPU benchmark
(using plain GCC10 rv32i built-in libraries only!).

| CPU Configuration (version [1.5.7.10](https://github.com/stnolting/neorv32/blob/main/CHANGELOG.md)) | CoreMark Score |
|:---------------------------------------------------------|:-----:|
| _small_ (`rv32i_Zicsr_Zifencei`)                         | 33.89 |
| _medium_ (`rv32imc_Zicsr_Zifencei`)                      | 62.50 |
| _performance_ (`rv32imc_Zicsr_Zifencei` + perf. options) | 95.23 |

More information regarding the CPU performance can be found in the
[Data Sheet: CPU Performance](https://stnolting.github.io/neorv32/#_cpu_performance).
The CPU & SoC provide further "tuning" options to optimize the design for maximum performance,
maximum clock speed, minimal area or minimal power consumption:
[User Guide: Application-Specific Processor Configuration](https://stnolting.github.io/neorv32/ug/#_application_specific_processor_configuration)


## 5. Getting Started

This overview provides some *quick links* to the most important sections of the
[online Data Sheet](https://stnolting.github.io/neorv32) and the [online User Guide](https://stnolting.github.io/neorv32/ug).

### :mag: [NEORV32 Project](https://stnolting.github.io/neorv32/#_overview) - An Introduction

* [Rationale](https://stnolting.github.io/neorv32/#_rationale) - why? how come? what for?
* [Key Features](https://stnolting.github.io/neorv32/#_project_key_features) - what makes it special
* [Structure](https://stnolting.github.io/neorv32/#_project_folder_structure) - folders, RTL files and compile order
* [File-List Files](https://stnolting.github.io/neorv32/#_file_list_files) - to simplify HDL setup
* [Metrics](https://stnolting.github.io/neorv32/#_fpga_implementation_results) - FPGA implementation and performance evaluation

### :desktop_computer: [NEORV32 Processor](https://stnolting.github.io/neorv32/#_neorv32_processor_soc) - The SoC

* [Top Entity - Signals](https://stnolting.github.io/neorv32/#_processor_top_entity_signals) - how to connect to the processor
* [Top Entity - Generics](https://stnolting.github.io/neorv32/#_processor_top_entity_generics) - processor/CPU configuration options
* [Address Space](https://stnolting.github.io/neorv32/#_address_space) - memory layout and address mapping
* [Boot Configuration](https://stnolting.github.io/neorv32/#_boot_configuration) - how to make the processor start executing
* [SoC Modules](https://stnolting.github.io/neorv32/#_processor_internal_modules) - IO/peripheral modules and memories
* [On-Chip Debugger](https://stnolting.github.io/neorv32/#_on_chip_debugger_ocd) - in-system debugging of the processor via JTAG

### :abacus: [NEORV32 CPU](https://stnolting.github.io/neorv32/#_neorv32_central_processing_unit_cpu) - The Core

* [RISC-V Compatibility](https://stnolting.github.io/neorv32/#_risc_v_compatibility) - what is compatible to the specs and what is not
* [Architecture](https://stnolting.github.io/neorv32/#_architecture) - a look under the hood
* [Full Virtualization](https://stnolting.github.io/neorv32/#_full_virtualization) - execution safety
* [ISA and Extensions](https://stnolting.github.io/neorv32/#_instruction_sets_and_extensions) - available (RISC-V) ISA extensions
* [CSRs](https://stnolting.github.io/neorv32/#_control_and_status_registers_csrs) - control and status registers
* [Traps](https://stnolting.github.io/neorv32/#_traps_exceptions_and_interrupts) - interrupts and exceptions

### :floppy_disk: [Software Framework](https://stnolting.github.io/neorv32/#_software_framework) - The Software Ecosystem

* [Example Programs](https://github.com/stnolting/neorv32/tree/main/sw/example) - examples how to use the processor's IO/peripheral modules
* [Core Libraries](https://stnolting.github.io/neorv32/#_core_libraries) - high-level functions for accessing the processor's peripherals
* [Software Framework Documentation](https://stnolting.github.io/neorv32/sw/files.html) - _doxygen_-based
* [Application Makefile](https://stnolting.github.io/neorv32/#_application_makefile) - turning _your_ application into an executable
* [Bootloader](https://stnolting.github.io/neorv32/#_bootloader) - the build-in NEORV32 bootloader
* [Image Generator](https://stnolting.github.io/neorv32/#_executable_image_format) - create (FPGA) memory initialization files from your application

### :rocket: [User Guide](https://stnolting.github.io/neorv32/ug/) - Let's Get It Started

* [Toolchain Setup](https://stnolting.github.io/neorv32/ug/#_software_toolchain_setup) - install and set up the RISC-V GCC toolchain
* [General Hardware Setup](https://stnolting.github.io/neorv32/ug/#_general_hardware_setup) - set up a new NEORV32 FPGA project
* [General Software Setup](https://stnolting.github.io/neorv32/ug/#_general_software_framework_setup) - configure the software framework
* [Application Compilation](https://stnolting.github.io/neorv32/ug/#_application_program_compilation) - compile an application using _make_
* [Upload via Bootloader](https://stnolting.github.io/neorv32/ug/#_uploading_and_starting_of_a_binary_executable_image_via_uart) - upload and execute executables
* [Application-Specific Processor Configuration](https://stnolting.github.io/neorv32/ug/#_application_specific_processor_configuration) - tailor the processor to your needs
* [Adding Custom Hardware Modules](https://stnolting.github.io/neorv32/ug/#_adding_custom_hardware_modules) - add _your_ custom hardware
* [Debugging via the On-Chip Debugger](https://stnolting.github.io/neorv32/ug/#_debugging_using_the_on_chip_debugger) - step through code *online* and *in-system*
* [Simulation](https://stnolting.github.io/neorv32/ug/#_simulating_the_processor) - simulate the whole SoC
* [LiteX Integration](https://stnolting.github.io/neorv32/ug/#_litex_soc_builder_support) - build a SoC using NEORV32 + [LiteX](https://github.com/enjoy-digital/litex)
* [Convert to Verilog](https://stnolting.github.io/neorv32/ug/#_neorv32_in_verilog) - turn the NEORV32 into an all-Verilog design
* [Package as Vivado IP block](https://stnolting.github.io/neorv32/ug/#_packaging_the_processor_as_vivado_ip_block) - turn the entire processor into an interactive AMD Vivado IP block
* [Using Eclipse](https://stnolting.github.io/neorv32/ug/#_eclipse_ide) - use the Eclipse IDE for developing and debugging

### :copyright: Legal

[![license](https://img.shields.io/github/license/stnolting/neorv32?longCache=true&style=flat)](https://github.com/stnolting/neorv32/blob/main/LICENSE)
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.5018888.svg)](https://doi.org/10.5281/zenodo.5018888)

* [Overview](https://stnolting.github.io/neorv32/#_legal) - license, disclaimer, limitation of liability for external links, proprietary notice, etc.
* [Citing](https://stnolting.github.io/neorv32/#_citing) - citing information

This is an open-source project that is free of charge. Use this project in any way you like
(as long as it complies to the permissive [license](https://github.com/stnolting/neorv32/blob/main/LICENSE)).
Please cite it appropriately. :+1:


---------------------------------------

**:heart: A big shout-out to the community and all the [contributors](https://github.com/stnolting/neorv32/graphs/contributors) -
this project would not be where it is without them!**
