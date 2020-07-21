# [The NEORV32 Processor](https://github.com/stnolting/neorv32) (RISC-V-compliant)

[![Build Status](https://travis-ci.com/stnolting/neorv32.svg?branch=master)](https://travis-ci.com/stnolting/neorv32)
[![license](https://img.shields.io/github/license/stnolting/neorv32)](https://github.com/stnolting/neorv32/blob/master/LICENSE)
[![release](https://img.shields.io/github/v/release/stnolting/neorv32)](https://github.com/stnolting/neorv32/releases)


## Table of Content

* [Introduction](#Introduction)
* [Features](#Features)
* [FPGA Implementation Results](#FPGA-Implementation-Results)
* [Performance](#Performance)
* [Top Entity](#Top-Entity)
* [**Getting Started**](#Getting-Started)
* [Contribute](#Contribute)
* [Legal](#Legal)



## Introduction

The NEORV32 is a customizable full-scale mikrocontroller-like processor system based on a [RISC-V-compliant](https://github.com/stnolting/neorv32_riscv_compliance)
`rv32i` CPU with optional `E`, `C`, `M`, `Zicsr` and `Zifencei` extensions. The CPU was built from scratch and is compliant to the **Unprivileged
ISA Specification Version 2.2** and a subset of the **Privileged Architecture Specification Version 1.12-draft**.

The NEORV32 is intended as auxiliary processor within a larger SoC designs or as stand-alone custom microcontroller.
Its top entity can be directly synthesized for any FPGA without modifications and provides a full-scale RISC-V based microcontroller.

The processor provides common peripherals and interfaces like input and output ports, serial interfaces for UART, I²C and SPI,
interrupt controller, timers and embedded memories. External memories, peripherals and custom IP can be attached via a
Wishbone-based external memory interface. All optional features beyond the base CPU can be enabled and configured via VHDL generics.

This project comes with a complete software ecosystem that features core libraries for high-level usage of the
provided functions and peripherals, application makefiles, a runtime environment and several example programs. All software source files
provide a doxygen-based [documentary](https://stnolting.github.io/neorv32/files.html).

The project is intended to work "out of the box". Just synthesize the test setup from this project, upload
it to your FPGA board of choice and start playing with the NEORV32. If you do not want to [compile the GCC toolchains](https://github.com/riscv/riscv-gnu-toolchain)
by yourself, you can also download [pre-compiled toolchains](https://github.com/stnolting/riscv_gcc_prebuilt) for Linux.

For more information take a look a the [![NEORV32 datasheet](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/figures/PDF_32.png) NEORV32 datasheet](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/NEORV32.pdf).


### Design Principles

 * From zero to main(): Completely open source and documented.
 * Plain VHDL without technology-specific parts like attributes, macros or primitives.
 * Easy to use – working out of the box.
 * Clean synchronous design, no wacky combinatorial interfaces.
 * The processor has to fit in a Lattice iCE40 UltraPlus 5k FPGA running at 20+ MHz.


### Status

The processor is synthesizable (tested with Intel Quartus Prime, Xilinx Vivado and Lattice Radiant/LSE) and can successfully execute
all the [provided example programs](https://github.com/stnolting/neorv32/tree/master/sw/example) including the CoreMark benchmark.

The processor passes the official `rv32i`, `rv32im`, `rv32imc`, `rv32Zicsr` and `rv32Zifencei` [RISC-V compliance tests](https://github.com/riscv/riscv-compliance).

| Project component                                                               | CI status | Note     |
|:--------------------------------------------------------------------------------|:----------|:---------|
| [NEORV32 processor](https://github.com/stnolting/neorv32)                       | [![Test](https://img.shields.io/travis/stnolting/neorv32/master.svg?label=test)](https://travis-ci.com/stnolting/neorv32)                                         | [![sw doc](https://img.shields.io/badge/SW%20documentation-gh--pages-blue)](https://stnolting.github.io/neorv32/files.html) |
| [Pre-built toolchain](https://github.com/stnolting/riscv_gcc_prebuilt)          | [![Test](https://img.shields.io/travis/stnolting/riscv_gcc_prebuilt/master.svg?label=test)](https://travis-ci.com/stnolting/riscv_gcc_prebuilt)                   | |
| [RISC-V compliance test](https://github.com/stnolting/neorv32_riscv_compliance) | [![Test](https://img.shields.io/travis/stnolting/neorv32_riscv_compliance/master.svg?label=compliance)](https://travis-ci.com/stnolting/neorv32_riscv_compliance) | |


### Non RISC-V-Compliant Issues

* No exception is triggered in `E` mode when using registers above `x15` (*needs fixing*)
* `misa` CSR is read-only - no dynamic enabling/disabling of implemented CPU extensions during runtime
* Machine software interrupt `msi` is implemented, but there is no mechanism available to trigger it
* The `[m]cycleh` and `[m]instreth` CSR counters are only 20-bit wide (in contrast to original 32-bit)


### To-Do / Wish List

- Add instructions how to use the NEORV32 CPU without the processor surroundings
- Add AXI / AXI-Lite bridges
- Option to use DSP-based multiplier in `M` extension (would be so much faster)
- Synthesis results for more platforms
- Implement user mode (`U` extension)
- Port Dhrystone benchmark
- Implement atomic operations (`A` extension) and floating-point operations (`F` extension)
- Maybe port an RTOS (like [Zephyr](https://github.com/zephyrproject-rtos/zephyr), [freeRTOS](https://www.freertos.org) or [RIOT](https://www.riot-os.org))
- Make a 64-bit branch someday



## Features

### Processor Features

![neorv32 Overview](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/figures/neorv32_processor.png)

  - RISC-V-compliant `rv32i` or `rv32e` CPU with optional `C`, `E`, `M`, `Zicsr` and `rv32Zifencei` extensions
  - GCC-based toolchain ([pre-compiled rv32i and rv32 etoolchains available](https://github.com/stnolting/riscv_gcc_prebuilt))
  - Application compilation based on [GNU makefiles](https://github.com/stnolting/neorv32/blob/master/sw/example/blink_led/makefile)
  - [Doxygen-based](https://github.com/stnolting/neorv32/blob/master/docs/doxygen_makefile_sw) documentation of the software framework: available on [GitHub pages](https://stnolting.github.io/neorv32/files.html)
  - Detailed [datasheet](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/NEORV32.pdf) (pdf)
  - Completely described in behavioral, platform-independent VHDL – no primitives, macros, etc.
  - Fully synchronous design, no latches, no gated clocks
  - Small hardware footprint and high operating frequency
  - Highly customizable processor configuration
  - _Optional_ processor-internal data and instruction memories (DMEM/IMEM)
  - _Optional_ internal bootloader with UART console and automatic SPI flash boot option
  - _Optional_ machine system timer (MTIME), RISC-V-compliant
  - _Optional_ universal asynchronous receiver and transmitter (UART)
  - _Optional_ 8/16/24/32-bit serial peripheral interface controller (SPI) with 8 dedicated chip select lines
  - _Optional_ two wire serial interface controller (TWI), compatible to the I²C standard
  - _Optional_ general purpose parallel IO port (GPIO), 16xOut & 16xIn, with pin-change interrupt
  - _Optional_ 32-bit external bus interface, Wishbone b4 compliant (WISHBONE)
  - _Optional_ watchdog timer (WDT)
  - _Optional_ PWM controller with 4 channels and 8-bit duty cycle resolution (PWM)
  - _Optional_ GARO-based true random number generator (TRNG)
  - _Optional_ core-local interrupt controller with 8 channels (CLIC)
  - _Optional_ dummy device (DEVNULL) (can be used for *fast* simulation console output)
  - System configuration information memory to check hardware configuration by software (SYSINFO)

### CPU Features

![neorv32 Overview](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/figures/neorv32_cpu.png)

The CPU is [compliant](https://github.com/stnolting/neorv32_riscv_compliance) to the
[official RISC-V specifications (2.2)](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/riscv-spec.pdf) including a subset of the 
[RISC-V privileged architecture specifications (1.12-draft)](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/riscv-spec.pdf).

More information regarding the CPU including a detailed list of the instruction set and the available CSRs can be found in
the [![NEORV32 datasheet](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/figures/PDF_32.png) NEORV32 datasheet](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/NEORV32.pdf).


**General**:
  * Modified Harvard architecture (separate CPU interfaces for data and instructions; single processor-bus via bus switch)
  * Two stages in-order pipeline (FETCH, EXECUTE); each stage uses a multi-cycle processing scheme
  * No hardware support of unaligned accesses - they will trigger and exception


**RV32I base instruction set** (`I` extension):
  * ALU instructions: `LUI` `AUIPC` `ADDI` `SLTI` `SLTIU` `XORI` `ORI` `ANDI` `SLLI` `SRLI` `SRAI` `ADD` `SUB` `SLL` `SLT` `SLTU` `XOR` `SRL` `SRA` `OR` `AND`
  * Jump and branch instructions: `JAL` `JALR` `BEQ` `BNE` `BLT` `BGE` `BLTU` `BGEU` 
  * Memory instructions: `LB` `LH` `LW` `LBU` `LHU` `SB` `SH` `SW`
  * System instructions: `ECALL` `EBREAK` `FENCE`

**Compressed instructions** (`C` extension):
  * ALU instructions: `C.ADDI4SPN` `C.ADDI` `C.ADD` `C.ADDI16SP` `C.LI` `C.LUI` `C.SLLI` `C.SRLI` `C.SRAI` `C.ANDI` `C.SUB` `C.XOR` `C.OR` `C.AND` `C.MV` `C.NOP`
  * Jump and branch instructions: `C.J` `C.JAL` `C.JR` `C.JALR` `C.BEQZ` `C.BNEZ`
  * Memory instructions: `C.LW` `C.SW` `C.LWSP` `C.SWSP`
  * Misc instructions: `C.EBREAK` (only with `Zicsr` extension)

**Embedded CPU version** (`E` extension):
  * Reduced register file (only the 16 lowest registers)

**Integer multiplication and division hardware** (`M` extension):
  * Multiplication instructions: `MUL` `MULH` `MULHSU` `MULHU`
  * Division instructions: `DIV` `DIVU` `REM` `REMU`

**Privileged architecture / CSR access** (`Zicsr` extension):
  * Privilege levels: `M-mode` (Machine mode)
  * CSR access instructions: `CSRRW` `CSRRS` `CSRRC` `CSRRWI` `CSRRSI` `CSRRCI`
  * System instructions: `MRET` `WFI`
  * Counter CSRs: `[m]cycle[h]` `[m]instret[h]` `time[h]`
  * Machine CSRs: `mstatus` `misa`(read-only!) `mie` `mtvec` `mscratch` `mepc` `mcause` `mtval` `mip` `mvendorid` `marchid` `mimpid` `mhartid`
  * Supported exceptions and interrupts:
    * Misaligned instruction address
    * Instruction access fault
    * Illegal instruction
    * Breakpoint (via `ebreak` instruction)
    * Load address misaligned
    * Load access fault
    * Store address misaligned
    * Store access fault
    * Environment call from M-mode (via `ecall` instruction)
    * Machine timer interrupt `mti` (via MTIME unit)
    * Machine external interrupt `mei` (via CLIC unit)

**Privileged architecture / FENCE.I** (`Zifencei` extension):
  * System instructions: `FENCE.I`


## FPGA Implementation Results

This chapter shows exemplary implementation results of the NEORV32 processor for an **Intel Cyclone IV EP4CE22F17C6N FPGA** on
a DE0-nano board. The design was synthesized using **Intel Quartus Prime Lite 19.1** ("balanced implementation"). The timing
information is derived from the Timing Analyzer / Slow 1200mV 0C Model. If not otherwise specified, the default configuration
of the processor's generics is assumed. No constraints were used at all.

### CPU

Results generated for hardware version: `1.2.0.0`

| CPU Configuration                | LEs        | FFs      | Memory bits | DSPs | f_max   |
|:---------------------------------|:----------:|:--------:|:-----------:|:----:|:-------:|
| `rv32i`                          |       1065 |      477 |       2048  |    0 | 112 MHz |
| `rv32i`   + `Zicsr` + `Zifencei` |       1914 |      837 |       2048  |    0 | 100 MHz |
| `rv32im`  + `Zicsr` + `Zifencei` |       2542 |     1085 |       2048  |    0 | 100 MHz |
| `rv32imc` + `Zicsr` + `Zifencei` |       2806 |     1102 |       2048  |    0 | 100 MHz |
| `rv32emc` + `Zicsr` + `Zifencei` |       2783 |     1102 |       1024  |    0 | 100 MHz |

### Processor-Internal Peripherals and Memories

Results generated for hardware version: `1.2.0.0`

| Module   | Description                                     | LEs | FFs | Memory bits | DSPs |
|:---------|:------------------------------------------------|:---:|:---:|:-----------:|:----:|
| BOOT ROM | Bootloader ROM (4kB)                            |   3 |   1 |      32 768 |    0 |
| DEVNULL  | Dummy device                                    |   3 |   1 |           0 |    0 |
| DMEM     | Processor-internal data memory (8kB)            |  12 |   2 |      65 536 |    0 |
| GPIO     | General purpose input/output ports              |  38 |  33 |           0 |    0 |
| IMEM     | Processor-internal instruction memory (16kb)    |   7 |   2 |     131 072 |    0 |
| MTIME    | Machine system timer                            | 269 | 166 |           0 |    0 |
| PWM      | Pulse-width modulation controller               |  76 |  69 |           0 |    0 |
| SPI      | Serial peripheral interface                     | 206 | 125 |           0 |    0 |
| SYSINFO  | System configuration information memory         |   7 |   7 |           0 |    0 |
| TRNG     | True random number generator                    | 104 |  93 |           0 |    0 |
| TWI      | Two-wire interface                              |  78 |  44 |           0 |    0 |
| UART     | Universal asynchronous receiver/transmitter     | 151 | 108 |           0 |    0 |
| WDT      | Watchdog timer                                  |  57 |  45 |           0 |    0 |


### Exemplary FPGA Setups

Exemplary implementation results for different FPGA platforms. The processor setup uses *all provided peripherals*,
no external memory interface and only internal instruction and data memories. IMEM uses 16kB and DMEM uses 8kB memory space. The setup's top entity connects most of the
processor's [top entity](https://github.com/stnolting/neorv32/blob/master/rtl/core/neorv32_top.vhd) signals
to FPGA pins - except for the Wishbone bus and the interrupt signals.

Results generated for hardware version: `1.2.0.6`

| Vendor  | FPGA                              | Board            | Toolchain               | Impl. strategy |CPU                               | LUT / LE   | FF / REG   | DSP    | Memory Bits  | BRAM / EBR | SPRAM    | Frequency    |
|:--------|:----------------------------------|:-----------------|:------------------------|:---------------|:---------------------------------|:-----------|:-----------|:-------|:-------------|:-----------|:---------|-------------:|
| Intel   | Cyclone IV `EP4CE22F17C6N`        | Terasic DE0-Nano | Quartus Prime Lite 19.1 | balanced       | `rv32imc` + `Zicsr` + `Zifencei` | 4035 (18%) | 1860  (8%) | 0 (0%) | 231424 (38%) |          - |        - |      101 MHz |
| Lattice | iCE40 UltraPlus `iCE40UP5K-SG48I` | Upduino v2.0     | Radiant 2.1 (LSE)       | timing         | `rv32ic`  + `Zicsr` + `Zifencei` | 5001 (95%) | 1694 (32%) | 0 (0%) |            - |   12 (40%) | 4 (100%) |   c 22.5 MHz |
| Xilinx  | Artix-7 `XC7A35TICSG324-1L`       | Arty A7-35T      | Vivado 2019.2           | default        | `rv32imc` + `Zicsr` + `Zifencei` | 2509 (12%) | 1914  (5%) | 0 (0%) |            - |    8 (16%) |        - |    c 100 MHz |

**Notes**
* The Lattice iCE40 UltraPlus setup uses the FPGA's SPRAM memory primitives for the internal IMEM and DEMEM (each 64kb).
The FPGA-specific memory components can be found in [`rtl/fpga_specific`](https://github.com/stnolting/neorv32/blob/master/rtl/fpga_specific/lattice_ice40up).
* The clock frequencies marked with a "c" are constrained clocks. The remaining ones are _f_max_ results from the place and route timing reports.
* The Upduino and the Arty board have on-board SPI flash memories for storing the FPGA configuration. These device can also be used by the default NEORV32
bootloader to store and automatically boot an application program after reset (both tested successfully).

## Performance

### CoreMark Benchmark

The [CoreMark CPU benchmark](https://www.eembc.org/coremark) was executed on the NEORV32 and is available in the
[sw/example/coremark](https://github.com/stnolting/neorv32/blob/master/sw/example/coremark) project folder. This benchmark
tests the capabilities of a CPU itself rather than the functions provided by the whole system / SoC.

Results generated for hardware version: `1.2.0.0`

~~~
**Configuration**
Hardware:    32kB IMEM, 16kB DMEM, 100MHz clock
CoreMark:    2000 iterations, MEM_METHOD is MEM_STACK
Compiler:    RISCV32-GCC 9.2.0
Peripherals: UART for printing the results
~~~

| CPU                              | Optimization | CoreMark Score | CoreMarks/MHz |
|:---------------------------------|:------------:|:--------------:|:-------------:|
| `rv32i`   + `Zicsr` + `Zifencei` |        `-O2` |          25.97 |        0.2597 |
| `rv32im`  + `Zicsr` + `Zifencei` |        `-O2` |          55.55 |        0.5555 |
| `rv32imc` + `Zicsr` + `Zifencei` |        `-O2` |          54.05 |        0.5405 |


### Instruction Cycles

The NEORV32 CPU is based on a two-stages pipelined architecutre. Each stage uses a multi-cycle processing scheme. Hence,
each instruction requires several clock cycles to execute (2 cycles for ALU operations, ..., 40 cycles for divisions).
The average CPI (cycles per instruction) depends on the instruction mix of a specific applications and also on the available
CPU extensions.

Please note that the CPU-internal shifter (e.g. for the `SLL` instruction) as well as the multiplier and divider of the
`M` extension use a bit-serial approach and require several cycles for completion.

The following table shows the performance results for successfully running 2000 CoreMark
iterations, which reflects a pretty good "real-life" work load. The average CPI is computed by
dividing the total number of required clock cycles (only the timed core to avoid distortion due to IO wait cycles; sampled via the `cycle[h]` CSRs)
by the number of executed instructions (`instret[h]` CSRs). The executables were generated using optimization `-O2`.

Results generated for hardware version: `1.2.0.0`

| CPU                              | Required Clock Cycles | Executed Instructions | Average CPI |
|:---------------------------------|----------------------:|----------------------:|:-----------:|
| `rv32i`   + `Zicsr` + `Zifencei` |         7 754 927 850 |         1 492 843 669 |         5.2 |
| `rv32im`  + `Zicsr` + `Zifencei` |         3 684 015 850 |           626 274 115 |         5.9 |
| `rv32imc` + `Zicsr` + `Zifencei` |         3 788 220 853 |           626 274 115 |         6.0 |



## Top Entity

The top entity of the processor is [**neorv32_top.vhd**](https://github.com/stnolting/neorv32/blob/master/rtl/core/neorv32_top.vhd) (from the `rtl/core` folder).
Just instantiate this file in your project and you are ready to go! All signals of this top entity are of type *std_ulogic* or *std_ulogic_vector*, respectively
(except for the TWI signals, which are of type *std_logic*).

Use the generics to configure the processor according to your needs. Each generic is initilized with the default configuration.
Detailed information regarding the signals and configuration generics can be found in the [NEORV32 documentary](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/NEORV32.pdf).

```vhdl
entity neorv32_top is
  generic (
    -- General --
    CLOCK_FREQUENCY              : natural := 0;      -- clock frequency of clk_i in Hz
    BOOTLOADER_USE               : boolean := true;   -- implement processor-internal bootloader?
    CSR_COUNTERS_USE             : boolean := true;   -- implement RISC-V perf. counters ([m]instret[h], [m]cycle[h], time[h])?
    USER_CODE                    : std_ulogic_vector(31 downto 0) := x"00000000"; -- custom user code
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_C        : boolean := true;   -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        : boolean := false;  -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        : boolean := true;   -- implement muld/div extension?
    CPU_EXTENSION_RISCV_Zicsr    : boolean := true;   -- implement CSR system?
    CPU_EXTENSION_RISCV_Zifencei : boolean := true;   -- implement instruction stream sync.?
    -- Memory configuration: Instruction memory --
    MEM_ISPACE_BASE              : std_ulogic_vector(31 downto 0) := x"00000000"; -- base address of instruction memory space
    MEM_ISPACE_SIZE              : natural := 16*1024; -- total size of instruction memory space in byte
    MEM_INT_IMEM_USE             : boolean := true;    -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            : natural := 16*1024; -- size of processor-internal instruction memory in bytes
    MEM_INT_IMEM_ROM             : boolean := false;   -- implement processor-internal instruction memory as ROM
    -- Memory configuration: Data memory --
    MEM_DSPACE_BASE              : std_ulogic_vector(31 downto 0) := x"80000000"; -- base address of data memory space
    MEM_DSPACE_SIZE              : natural := 8*1024; -- total size of data memory space in byte
    MEM_INT_DMEM_USE             : boolean := true;   -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            : natural := 8*1024; -- size of processor-internal data memory in bytes
    -- Memory configuration: External memory interface --
    MEM_EXT_USE                  : boolean := false;  -- implement external memory bus interface?
    MEM_EXT_REG_STAGES           : natural := 2;      -- number of interface register stages (0,1,2)
    MEM_EXT_TIMEOUT              : natural := 15;     -- cycles after which a valid bus access will timeout (>=1)
    -- Processor peripherals --
    IO_GPIO_USE                  : boolean := true;   -- implement general purpose input/output port unit (GPIO)?
    IO_MTIME_USE                 : boolean := true;   -- implement machine system timer (MTIME)?
    IO_UART_USE                  : boolean := true;   -- implement universal asynchronous receiver/transmitter (UART)?
    IO_SPI_USE                   : boolean := true;   -- implement serial peripheral interface (SPI)?
    IO_TWI_USE                   : boolean := true;   -- implement two-wire interface (TWI)?
    IO_PWM_USE                   : boolean := true;   -- implement pulse-width modulation unit (PWM)?
    IO_WDT_USE                   : boolean := true;   -- implement watch dog timer (WDT)?
    IO_CLIC_USE                  : boolean := true;   -- implement core local interrupt controller (CLIC)?
    IO_TRNG_USE                  : boolean := false;  -- implement true random number generator (TRNG)?
    IO_DEVNULL_USE               : boolean := true    -- implement dummy device (DEVNULL)?
  );
  port (
    -- Global control --
    clk_i        : in  std_ulogic := '0'; -- global clock, rising edge
    rstn_i       : in  std_ulogic := '0'; -- global reset, low-active, async
    -- Wishbone bus interface (available if MEM_EXT_USE = true) --
    wb_adr_o     : out std_ulogic_vector(31 downto 0); -- address
    wb_dat_i     : in  std_ulogic_vector(31 downto 0) := (others => '0'); -- read data
    wb_dat_o     : out std_ulogic_vector(31 downto 0); -- write data
    wb_we_o      : out std_ulogic; -- read/write
    wb_sel_o     : out std_ulogic_vector(03 downto 0); -- byte enable
    wb_stb_o     : out std_ulogic; -- strobe
    wb_cyc_o     : out std_ulogic; -- valid cycle
    wb_ack_i     : in  std_ulogic := '0'; -- transfer acknowledge
    wb_err_i     : in  std_ulogic := '0'; -- transfer error
    -- Advanced memory control signals (available if MEM_EXT_USE = true) --
    fence_o      : out std_ulogic; -- indicates an executed FENCE operation
    fencei_o     : out std_ulogic; -- indicates an executed FENCEI operation
    -- GPIO (available if IO_GPIO_USE = true) --
    gpio_o       : out std_ulogic_vector(15 downto 0); -- parallel output
    gpio_i       : in  std_ulogic_vector(15 downto 0) := (others => '0'); -- parallel input
    -- UART (available if IO_UART_USE = true) --
    uart_txd_o   : out std_ulogic; -- UART send data
    uart_rxd_i   : in  std_ulogic := '0'; -- UART receive data
    -- SPI (available if IO_SPI_USE = true) --
    spi_sck_o    : out std_ulogic; -- serial clock line
    spi_sdo_o    : out std_ulogic; -- serial data line out
    spi_sdi_i    : in  std_ulogic := '0'; -- serial data line in
    spi_csn_o    : out std_ulogic_vector(07 downto 0); -- SPI CS
    -- TWI (available if IO_TWI_USE = true) --
    twi_sda_io   : inout std_logic := 'H'; -- twi serial data line
    twi_scl_io   : inout std_logic := 'H'; -- twi serial clock line
    -- PWM (available if IO_PWM_USE = true) --
    pwm_o        : out std_ulogic_vector(03 downto 0); -- pwm channels
    -- Interrupts (available if IO_CLIC_USE = true) --
    ext_irq_i    : in  std_ulogic_vector(01 downto 0) := (others => '0'); -- external interrupt request
    ext_ack_o    : out std_ulogic_vector(01 downto 0)  -- external interrupt request acknowledge
  );
end neorv32_top;
```



## Getting Started

This overview is just a short excerpt from the *Let's Get It Started* section of the NEORV32 documentary:

[![NEORV32 datasheet](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/figures/PDF_32.png) NEORV32 datasheet](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/NEORV32.pdf)


### Building the Toolchain

At first you need the **RISC-V GCC toolchain**. You can either [download the sources](https://github.com/riscv/riscv-gnu-toolchain)
and build the toolchain by yourself, or you can download a prebuilt one and install it.

To build the toolchain by yourself, get the sources from the official [RISCV-GNU-TOOLCHAIN](https://github.com/riscv/riscv-gnu-toolchain) github page:

    $ git clone --recursive https://github.com/riscv/riscv-gnu-toolchain

Download and install the prerequisite standard packages:

    $ sudo apt-get install autoconf automake autotools-dev curl python3 libmpc-dev libmpfr-dev libgmp-dev gawk build-essential bison flex texinfo gperf libtool patchutils bc zlib1g-dev libexpat-dev

To build the Linux cross-compiler, pick an install path. If you choose, say, `/opt/riscv`, then add `/opt/riscv/bin` to your `PATH` environment variable.

    $ export PATH:$PATH:/opt/riscv/bin

Then, simply run the following commands in the RISC-V GNU toolchain source folder (for the `rv32i` toolchain):

    riscv-gnu-toolchain$ ./configure --prefix=/opt/riscv --with-arch=rv32i –with-abi=ilp32
    riscv-gnu-toolchain$ make

After a while (hours!) you will get `riscv32-unknown-elf-gcc` and all of its friends in your `/opt/riscv/bin` folder.


### Using a Prebuilt Toolchain

Alternatively, you can download a prebuilt toolchain. I have uploaded the toolchain I am using to GitHub. This toolchain
has been compiled on a 64-bit x86 Ubuntu (Ubuntu on Windows). Download the toolchain of choice:

[https://github.com/stnolting/riscv_gcc_prebuilt](https://github.com/stnolting/riscv_gcc_prebuilt)


### Dowload the NEORV32 and Create a Hardware Project

Get the sources of the NEORV32 Processor project. You can either download a [release](https://github.com/stnolting/neorv32/releases)
or get the most recent version of this project as [`*.zip` file](https://github.com/stnolting/neorv32/archive/master.zip) or using `git clone` (suggested for easy project updates via `git pull`):

    $ git clone https://github.com/stnolting/neorv32.git

Create a new project with your FPGA design tool of choice and add all the `*.vhd` files from the [`rtl/core`](https://github.com/stnolting/neorv32/blob/master/rtl)
folder to this project. Make sure to add them to a **new library** called `neorv32`.

You can either instantiate the [processor's top entity](https://github.com/stnolting/neorv32#top-entity) in your own project or you
can use a simple [test setup](https://github.com/stnolting/neorv32/blob/master/rtl/top_templates/neorv32_test_setup.vhd) (from the project's
[`rtl/top_templates`](https://github.com/stnolting/neorv32/blob/master/rtl/top_templates) folder) as top entity.
This test setup instantiates the processor and implements most of the peripherals and some ISA extensions. Only the UART, clock, reset and some GPIO output sginals are
propagated (basically, its a FPGA "hello world" example):

```vhdl
  entity neorv32_test_setup is
    port (
      -- Global control --
      clk_i      : in  std_ulogic := '0'; -- global clock, rising edge
      rstn_i     : in  std_ulogic := '0'; -- global reset, low-active, async
      -- GPIO --
      gpio_o     : out std_ulogic_vector(7 downto 0); -- parallel output
      -- UART --
      uart_txd_o : out std_ulogic; -- UART send data
      uart_rxd_i : in  std_ulogic := '0' -- UART receive data
    );
  end neorv32_test_setup;
```


### Compiling and Uploading One of the Example Projects

Make sure `GNU Make` and a native `GCC` compiler are installed. To test the installation of the RISC-V toolchain navigate to an example project like
`sw/example/blink_led` and run:

    neorv32/sw/example/blink_led$ make check

The NEORV32 project includes some [example programs](https://github.com/stnolting/neorv32/tree/master/sw/example) from
which you can start your own application. Simply compile one of these projects. This will create a NEORV32
executable `neorv32_exe.bin` in the same folder.

    neorv32/sw/example/blink_led$ make clean_all compile

Connect your FPGA board via UART to you computer and open the according port to interface with the NEORV32 bootloader. The bootloader
uses the following default UART configuration:

- 19200 Baud
- 8 data bits
- 1 stop bit
- No parity bits
- No transmission / flow control protocol (raw bytes only)
- Newline on `\r\n` (carriage return & newline)

Use the bootloader console to upload the `neorv32_exe.bin` file and run your application image.

```
  << NEORV32 Bootloader >>
  
  BLDV: Jul  6 2020
  HWV:  1.0.1.0
  CLK:  0x0134FD90 Hz
  USER: 0x0001CE40
  MISA: 0x42801104
  CONF: 0x03FF0035
  IMEM: 0x00010000 bytes @ 0x00000000
  DMEM: 0x00010000 bytes @ 0x80000000
  
  Autoboot in 8s. Press key to abort.
  Aborted.
  
  Available CMDs:
   h: Help
   r: Restart
   u: Upload
   s: Store to flash
   l: Load from flash
   e: Execute
  CMD:> u
  Awaiting neorv32_exe.bin... OK
  CMD:> e
  Booting...
  
  Blinking LED demo program
```

Going further: Take a look at the _Let's Get It Started!_ chapter of the [![NEORV32 datasheet](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/figures/PDF_32.png) NEORV32 datasheet](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/NEORV32.pdf).



## Contribute

I'm always thankful for help! So if you have any questions, bug reports, ideas or if you want to give some kind of feedback, feel free
to open a [new issue](https://github.com/stnolting/neorv32/issues).

If you want to get involved you can also directly drop me a line (mailto:stnolting@gmail.com).
Please also check out the project's [code of conduct](https://github.com/stnolting/neorv32/tree/master/CODE_OF_CONDUCT.md).



## Legal

This project is released under the BSD 3-Clause license. No copyright infringement intended.
Other implied or used projects might have different licensing - see their documentation to get more information.

#### Citation

If you are using the NEORV32 Processor in some kind of publication, please cite it as follows:

> S. Nolting, "The NEORV32 Processor", github.com/stnolting/neorv32

#### BSD 3-Clause License

Copyright (c) 2020, Stephan Nolting. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or other materials
provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior written
permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.


#### Limitation of Liability for External Links

Our website contains links to the websites of third parties („external links“). As the
content of these websites is not under our control, we cannot assume any liability for
such external content. In all cases, the provider of information of the linked websites
is liable for the content and accuracy of the information provided. At the point in time
when the links were placed, no infringements of the law were recognisable to us. As soon
as an infringement of the law becomes known to us, we will immediately remove the
link in question.


#### Proprietary  Notice

"Windows" is a trademark of Microsoft Corporation.

"Artix" and "Vivado" are trademarks of Xilinx Inc.

"Cyclone", "Quartus Prime", "Quartus Prime Lite" and "Avalon Bus" are trademarks of Intel Corporation.

"Artix" and "Vivado" are trademarks of Xilinx, Inc.

"iCE40", "UltraPlus" and "Lattice Radiant" are trademarks of Lattice Semiconductor Corporation.

"AXI" and "AXI-Lite" are trademarks of Arm Holdings plc.


## Acknowledgement

[RISC-V](https://riscv.org/) - Instruction Sets Want To Be Free :heart:

[![Continous Integration provided by Travis CI](https://travis-ci.com/images/logos/TravisCI-Full-Color.png)](https://travis-ci.com/stnolting/neorv32)

Continous integration provided by [Travis CI](https://travis-ci.com/stnolting/neorv32) and powered by [GHDL](https://github.com/ghdl/ghdl).


![Open Source Hardware Logo https://www.oshwa.org](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/figures/oshw_logo.png)

This project is not affiliated with or endorsed by the Open Source Initiative (https://www.oshwa.org / https://opensource.org).

.

Made with :coffee: in Hannover, Germany.
