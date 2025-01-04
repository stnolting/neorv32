## Project Change Log

[![release](https://img.shields.io/github/v/release/stnolting/neorv32?longCache=true&style=flat-square&logo=GitHub)](https://github.com/stnolting/neorv32/releases)
[![commits-since-latest-release](https://img.shields.io/github/commits-since/stnolting/neorv32/latest?longCache=true&style=flat-square&logo=GitHub)](https://github.com/stnolting/neorv32/activity)

This project uses [semantic versioning](https://semver.org).
The **version identifier** uses an additional custom element (`MAJOR.MINOR.PATCH.custom`)
to track individual changes. The identifier is incremented by every core hardware modification
and also by major software and/or general project changes.

The version identifier is globally defined by the `hw_version_c` constant in the main VHDL
[package file](https://github.com/stnolting/neorv32/blob/main/rtl/core/neorv32_package.vhd).
Software can determine the version by reading the RISC-V-compatible `mimpid` CSR, which uses
a 8x4-bit BCD (binary-coded decimal) format to represent the current version. Example:

```
mimpid = 0x01040312 -> Version 01.04.03.12 -> v1.4.3.12
```

### Version History

* :bug: bug-fix
* :sparkles: new feature
* :test_tube: new experimental feature
* :warning: changes that might impact compatibility with previous versions
* :lock: security/safety-related
* :rocket: official release
* Date format is "dd.mm.yyyy"

| Date | Version | Comment | Ticket |
|:----:|:-------:|:--------|:------:|
| 03.01.2025 | 1.10.8.7 | :warning: :sparkles: replace `Zalrsc` ISA extensions (reservation-set operations) by `Zaamo` ISA extension (atomic read-modify-write operations) | [#1141](https://github.com/stnolting/neorv32/pull/1141) |
| 01.01.2025 | 1.10.8.6 | :sparkles: :test_tube: add smp dual-core option | [#1135](https://github.com/stnolting/neorv32/pull/1135) |
| 29.12.2024 | 1.10.8.5 | :test_tube: add multi-hart support to debug module | [#1132](https://github.com/stnolting/neorv32/pull/1132) |
| 29.12.2024 | 1.10.8.4 | :warning: rename `SYSINFO.MEM -> SYSINFO.MISC`; add new `SYSINFO.MISC` entry for number of CPU cores (hardwired to one) | [#1134](https://github.com/stnolting/neorv32/pull/1134) |
| 29.12.2024 | 1.10.8.3 | :bug: fix incorrect HPM counter sizes if `HPM_CNT_WIDTH = 64` | [#1128](https://github.com/stnolting/neorv32/pull/1128) |
| 27.12.2024 | 1.10.8.2 | add out-of-band signals to internal request bus | [#1131](https://github.com/stnolting/neorv32/pull/1131) |
| 27.12.2024 | 1.10.8.1 | :warning: replace MTIME by CLINT; :warning: remove `HART_ID` generic | [#1130](https://github.com/stnolting/neorv32/pull/1130) |
| 26.12.2024 | [**:rocket:1.10.8**](https://github.com/stnolting/neorv32/releases/tag/v1.10.8) | **New release** | |
| 23.12.2024 | 1.10.7.9 | :warning: rework IO/peripheral address space; :sparkles: increase device size from 256 bytes to 64kB | [#1126](https://github.com/stnolting/neorv32/pull/1126) |
| 22.12.2024 | 1.10.7.8 | :warning: rename CPU tuning options / generics | [#1125](https://github.com/stnolting/neorv32/pull/1125) |
| 22.12.2024 | 1.10.7.7 | :warning: move clock gating switch from processor top to CPU clock; `CLOCK_GATING_EN` is now a CPU tuning option | [#1124](https://github.com/stnolting/neorv32/pull/1124) |
| 21.12.2024 | 1.10.7.6 | minor rtl cleanups and optimizations | [#1123](https://github.com/stnolting/neorv32/pull/1123) |
| 19.12.2024 | 1.10.7.5 | :test_tube: use time-multiplex PMP architecture (reducing area footprint) | [#1105](https://github.com/stnolting/neorv32/pull/1105) |
| 14.12.2024 | 1.10.7.4 | :sparkles: add new module: I2C-compatible **Two-Wire Device Controller (TWD)** | [#1121](https://github.com/stnolting/neorv32/pull/1121) |
| 14.12.2024 | 1.10.7.3 | :warning: rework TRNG (change HAL; remove interrupt) | [#1120](https://github.com/stnolting/neorv32/pull/1120) |
| 12.12.2024 | 1.10.7.2 | add external memory configuration/initialization options to testbench | [#1119](https://github.com/stnolting/neorv32/pull/1119) |
| 11.12.2024 | 1.10.7.1 | :test_tube: shrink bootloader's minimal ISA (`rv32e`) and RAM (256 bytes) requirements | [#1118](https://github.com/stnolting/neorv32/pull/1118) |
| 10.12.2024 | [**:rocket:1.10.7**](https://github.com/stnolting/neorv32/releases/tag/v1.10.7) | **New release** | |
| 03.12.2024 | 1.10.6.9 | :sparkles: add ONEWIRE command and data FIFO; :warning: rework ONEWIRE interface register layout; :bug: fix regression: busy flag was stuck at zero | [#1113](https://github.com/stnolting/neorv32/pull/1113) |
| 01.12.2024 | 1.10.6.8 | add TWI bus sensing logic | [#1111](https://github.com/stnolting/neorv32/pull/1111) |
| 26.11.2024 | 1.10.6.7 | :bug: fix some HDL issues that caused problems when auto-converting to Verilog | [#1103](https://github.com/stnolting/neorv32/pull/1103) |
| 23.11.2024 | 1.10.6.6 | CPU control: large code edits and cleanups | [#1099](https://github.com/stnolting/neorv32/pull/1099) |
| 10.11.2024 | 1.10.6.5 | :warning: switch to [xPack](https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack) as default prebuilt RISC-V GCC toolchain (now using `riscv-none-elf-` as default gcc prefix) | [#1091](https://github.com/stnolting/neorv32/pull/1091) |
| 10.11.2024 | 1.10.6.4 | rework default processor testbench | [#1093](https://github.com/stnolting/neorv32/pull/1093) |
| 06.11.2024 | 1.10.6.3 | minor rtl edits and cleanups | [#1090](https://github.com/stnolting/neorv32/pull/1090) |
| 02.11.2024 | 1.10.6.2 | :warning: rework processor boot configuration; add new boot-configuration generics | [#1086](https://github.com/stnolting/neorv32/pull/1086) |
| 01.11.2024 | 1.10.6.1 | :test_tube: convert VHDL memory images into full-scale VHDL packages | [#1084](https://github.com/stnolting/neorv32/pull/1084) |
| 26.10.2024 | [**:rocket:1.10.6**](https://github.com/stnolting/neorv32/releases/tag/v1.10.6) | **New release** | |
| 26.10.2024 | 1.10.5.11 | cleanup central makefile and linker script | [#1077](https://github.com/stnolting/neorv32/pull/1077) |
| 21.10.2024 | 1.10.5.10 | :test_tube: rework linker script's ROM/IMEM default size (=16kB); add customization variable to all makefiles in `sw/example` | [#1072](https://github.com/stnolting/neorv32/pull/1072) |
| 20.10.2024 | 1.10.5.9 | :warning: rework XIRQ controller; remove "interrupt pending" register `EIP` | [#1071](https://github.com/stnolting/neorv32/pull/1071) |
| 18.10.2024 | 1.10.5.8 | minor RTL code cleanups | [#1068](https://github.com/stnolting/neorv32/pull/1068) |
| 18.10.2024 | 1.10.5.7 | use individual/new module for XBUS-to-AXI4-Lite bridge | [#1063](https://github.com/stnolting/neorv32/pull/1063) |
| 12.10.2024 | 1.10.5.6 | :warning: remove legacy support for on-chip debugger DM version v0.13; now only supporting DM v1.0 (removing `OCD_DM_LEGACY_MODE` generic) | [#1056](https://github.com/stnolting/neorv32/pull/1056) |
| 11.10.2024 | 1.10.5.5 | :sparkles: :lock: add support for optional on-chip debugger authentication; :warning: rename OCD-related top generics | [#1053](https://github.com/stnolting/neorv32/pull/1053) |
| 06.10.2024 | 1.10.5.4 | :warning: rework PWM module | [#1049](https://github.com/stnolting/neorv32/pull/1049) |
| 05.10.2024 | 1.10.5.3 | upgrade neoTRNG to version 3.2 | [#1048](https://github.com/stnolting/neorv32/pull/1048) |
| 03.10.2024 | 1.10.5.2 | :warning: remove `A` ISA extensions; replaced by new `Zalrsc` ISA extension | [#1047](https://github.com/stnolting/neorv32/pull/1047) |
| 02.10.2024 | 1.10.5.1 | :warning: rework CFU interface; reduce minimal latency of CFU instructions from 4 cycles to 3 cycles | [#1046](https://github.com/stnolting/neorv32/pull/1046) |
| 01.10.2024 | [**:rocket:1.10.5**](https://github.com/stnolting/neorv32/releases/tag/v1.10.5) | **New release** | |
| 30.09.2024 | 1.10.4.11 | :warning: split `B` ISA extensions into individual sub-extensions: `Zba`, `Zbb`, `Zbs` | [#1044](https://github.com/stnolting/neorv32/pull/1044) |
| 29.09.2024 | 1.10.4.10 | :warning: rename CPU ISA configuration generics: `CPU_EXTENSION_* -> RISCV_ISA_*` | [#1041](https://github.com/stnolting/neorv32/pull/1041) |
| 28.09.2024 | 1.10.4.9 | :sparkles: add support for RISC-V "ShangMi algorithm suite" ISA extensions: `Zks`, `Zksed`, `Zksh` | [#1040](https://github.com/stnolting/neorv32/pull/1040) |
| 28.09.2024 | 1.10.4.8 | :sparkles: add support for RISC-V "NIST algorithm suite" ISA extension `Zkn` | [#1039](https://github.com/stnolting/neorv32/pull/1039) |
| 27.09.2024 | 1.10.4.7 | :sparkles: add support for RISC-V "carry-less multiplication instruction for cryptography" ISA extension `Zbkc` | [#1038](https://github.com/stnolting/neorv32/pull/1038) |
| 27.09.2024 | 1.10.4.6 | :sparkles: add support for RISC-V "bit manipulation instructions for cryptography" ISA extension `Zbkb` | [#1037](https://github.com/stnolting/neorv32/pull/1037) |
| 27.09.2024 | 1.10.4.5 | :sparkles: add support for RISC-V "data independent execution time" ISA extension `Zkt` | [#1036](https://github.com/stnolting/neorv32/pull/1036) |
| 25.09.2024 | 1.10.4.4 | :sparkles: add support for RISC-V "scalar cryptography" ISA extensions `Zbkx`, `Zknd`, `Zkne`, `Zknh` | [#1033](https://github.com/stnolting/neorv32/pull/1033) |
| 23.09.2024 | 1.10.4.3 | rework/optimize ALU instruction decoding and CPU co-processor interface | [#1032](https://github.com/stnolting/neorv32/pull/1032) |
| 20.09.2024 | 1.10.4.2 | :bug: fix minor bug in FPU's multiplication instruction (invalid-check logic if any operand is sNAN) | [#1028](https://github.com/stnolting/neorv32/pull/1028) |
| 20.09.2024 | 1.10.4.1 | rtl signal renamings to make the code more readable | [#1026](https://github.com/stnolting/neorv32/pull/1026) |
| 16.09.2024 | [**:rocket:1.10.4**](https://github.com/stnolting/neorv32/releases/tag/v1.10.4) | **New release** | |
| 15.09.2024 | 1.10.3.10 | :bug: SW: fix stack-alignment (has to be 128-bit-aligned) before entering the very first procedure (`main`) | [#1021](https://github.com/stnolting/neorv32/pull/1021) |
| 14.09.2024 | 1.10.3.9 | massive rtl code cleanup | [#1019](https://github.com/stnolting/neorv32/pull/1019) |
| 14.09.2024 | 1.10.3.8 | :bug: fix `b.ctz` instruction decoding (bug introduced in v1.10.3.6) | [#1018](https://github.com/stnolting/neorv32/pull/1018) |
| 14.09.2024 | 1.10.3.7 | :warning: rework RTL files / hierarchy | [#1017](https://github.com/stnolting/neorv32/pull/1017) |
| 13.09.2024 | 1.10.3.6 | cleanup and extend watchdog's reset-cause identification logic | [#1015](https://github.com/stnolting/neorv32/pull/1015) |
| 13.09.2024 | 1.10.3.5 | rtl code cleanups; minor CPU control optimizations | [#1014](https://github.com/stnolting/neorv32/pull/1014) |
| 08.09.2024 | 1.10.3.4 | minor rtl/CSR optimizations | [#1010](https://github.com/stnolting/neorv32/pull/1010) |
| 08.09.2024 | 1.10.3.3 | optimize CSR address logic (to reduce switching activity) | [#1008](https://github.com/stnolting/neorv32/pull/1008) |
| 05.09.2024 | 1.10.3.2 | :test_tube: Remove "for loop" construct from memory initialization function as the max. number of loop/unrolling iterations might be constrained | [#1005](https://github.com/stnolting/neorv32/pull/1005) |
| 05.09.2024 | 1.10.3.1 | minor CPU RTL cleanups and optimizations | [#1004](https://github.com/stnolting/neorv32/pull/1004) |
| 03.09.2024 | [**:rocket:1.10.3**](https://github.com/stnolting/neorv32/releases/tag/v1.10.3) | **New release** | |
| 30.08.2024 | 1.10.2.9 | :bug: fix PC reset bug (introduced in v1.10.2.8); minor RTL optimizations (size and critical path) | [#998](https://github.com/stnolting/neorv32/pull/998) |
| 25.08.2024 | 1.10.2.8 | :warning: remove user-mode HPM counters; add individual `mocunteren` bits (`CY` and `IR`) rework Vivado IP module; minor RTL cleanups and optimization | [#996](https://github.com/stnolting/neorv32/pull/996) |
| 16.08.2024 | 1.10.2.7 | minor CPU area and critical path optimizations; minor code cleanups | [#990](https://github.com/stnolting/neorv32/pull/990) |
| 09.08.2024 | 1.10.2.6 | :warning: re-organize RTL files; all core files are now located in `rtl/core`; remove `mem` sub-folder | [#985](https://github.com/stnolting/neorv32/pull/985) |
| 09.08.2024 | 1.10.2.5 | minor HDL edits | [#984](https://github.com/stnolting/neorv32/pull/984) |
| 06.08.2024 | 1.10.2.4 | :warning: **Vivado IP module**: constrain minimal ALL input/output size to 1; add explicit PWM controller enable option | [#980](https://github.com/stnolting/neorv32/pull/980) |
| 05.08.2024 | 1.10.2.3 | :bug: fix bug in **Vivado IP module** (error if zero-sized input port is unconnected) | [#978](https://github.com/stnolting/neorv32/pull/978) |
| 04.08.2024 | 1.10.2.2 | :bug: fix bug in **Vivado IP module** (error if AXI port is unconnected) | [#976](https://github.com/stnolting/neorv32/pull/976) |
| 02.08.2024 | 1.10.2.1 | :warning: rework CFU; remove support for R5-type instructions | [#971](https://github.com/stnolting/neorv32/pull/971) |
| 29.07.2024 | [**:rocket:1.10.2**](https://github.com/stnolting/neorv32/releases/tag/v1.10.2) | **New release** | |
| 28.07.2024 | 1.10.1.9 | make SYSINFO.CLK read/**write** | [#966](https://github.com/stnolting/neorv32/pull/966) |
| 21.07.2024 | 1.10.1.8 | :lock: restrict IO access to privileged (machine-mode) software | [#958](https://github.com/stnolting/neorv32/pull/958) |
| 20.07.2024 | 1.10.1.7 | :bug: fix bug in `sbrk` newlib system call (causing `malloc` to provide infinite memory until heap and stack collide) | [#957](https://github.com/stnolting/neorv32/pull/957) |
| 20.07.2024 | 1.10.1.6 | SDI: remove explicit "RX clear flag"; add new flag to check the current state of the chip-select input | [#955](https://github.com/stnolting/neorv32/pull/955) |
| 19.07.2024 | 1.10.1.5 | :sparkles: add "programmable" chip-select enable/disable functionality to SPI module | [#954](https://github.com/stnolting/neorv32/pull/954) |
| 19.07.2024 | 1.10.1.4 | :bug: fix SDI "TX FIFO full" flag | [#953](https://github.com/stnolting/neorv32/pull/953) |
| 18.07.2024 | 1.10.1.3 | :test_tube: add new generic to disable the SYSINFO module (:warning: for advanced users only that wish to use a CPU-only setup): `IO_DISABLE_SYSINFO` | [#952](https://github.com/stnolting/neorv32/pull/952) |
| 10.07.2024 | 1.10.1.2 | minor rtl edits and cleanups | [#948](https://github.com/stnolting/neorv32/pull/948) |
| 05.07.2024 | 1.10.1.1 | minor rtl cleanups and optimizations | [#941](https://github.com/stnolting/neorv32/pull/941) |
| 04.07.2024 | [**:rocket:1.10.1**](https://github.com/stnolting/neorv32/releases/tag/v1.10.1) | **New release** | |
| 04.07.2024 | 1.10.0.10 | :warning: rework GPTMR and remove capture mode | [#939](https://github.com/stnolting/neorv32/pull/939) |
| 03.07.2024 | 1.10.0.9 | :warning: remove `AMO_RVS_GRANULARITY` generic, reservation set granularity is now fixed to 4 bytes | [#938](https://github.com/stnolting/neorv32/pull/938) |
| 03.07.2024 | 1.10.0.8 | :test_tube: add XBUS to AHB3-lite bridge | [#937](https://github.com/stnolting/neorv32/pull/937) |
| 02.07.2024 | 1.10.0.7 | minor rtl and software edits and cleanups | [#936](https://github.com/stnolting/neorv32/pull/936) |
| 30.06.2024 | 1.10.0.6 | minor rtl edits and cleanups | [#935](https://github.com/stnolting/neorv32/pull/935) |
| 29.06.2024 | 1.10.0.5 | :warning: rework and optimize custom functions unit (CFU) interface; simplified illegal RVC decoding | [#932](https://github.com/stnolting/neorv32/pull/932) |
| 23.06.2024 | 1.10.0.4 | minor rtl edits/cleanups | [#931](https://github.com/stnolting/neorv32/pull/931) |
| 22.06.2024 | 1.10.0.3 | UARTs: add flags to clear RX/TX FIFOs; DMA: add FIRQ trigger type configuration flag | [#930](https://github.com/stnolting/neorv32/pull/930) |
| 21.06.2024 | 1.10.0.2 | minor code rtl clean-ups; fix some missing TOP defaults | [#929](https://github.com/stnolting/neorv32/pull/929) |
| 17.06.2024 | 1.10.0.1 | :warning: remove (optional and redundant) JTAG reset signal `jtag_trst_i` | [#928](https://github.com/stnolting/neorv32/pull/928) |
| 16.06.2024 | [**:rocket:1.10.0**](https://github.com/stnolting/neorv32/releases/tag/v1.10.0) | **New release** | |
| 15.06.2024 | 1.9.9.9 | :sparkles: add pre-configured example project for Eclipse IDE | [#926](https://github.com/stnolting/neorv32/pull/926) |
| 14.06.2024 | 1.9.9.8 | minor rtl edits/cleanups; increase bootloader's auto-boot timeout from 8s to 10s | [#925](https://github.com/stnolting/neorv32/pull/925) |
| 07.06.2024 | 1.9.9.7 | :sparkles: re-add TRNG "data available" interrupt | [#922](https://github.com/stnolting/neorv32/pull/922) |
| 31.05.2024 | 1.9.9.6 | add "tag" signal to XBUS to provide additional access information (compatible to the AXI4 _ARPROT_ and _AWPROT_ signals) | [#917](https://github.com/stnolting/neorv32/pull/917) |
| 30.05.2024 | 1.9.9.5 | :bug: fix uncached-vs-cached memory accesses (do not interrupt cache bursts by direct/uncached memory accesses) | [#915](https://github.com/stnolting/neorv32/pull/915) |
| 29.05.2024 | 1.9.9.4 | Vivado IP block: add resizing ports for GPIOs, XIRQs and PWM; split size configuration for GPIO inputs and outputs | [#913](https://github.com/stnolting/neorv32/pull/913) |
| 27.05.2024 | 1.9.9.3 | removed `XIRQ_TRIGGER_*` generics; XIRQ trigger type is now _programmable_ by dedicated configuration registers | [#911](https://github.com/stnolting/neorv32/pull/911) |
| 21.05.2024 | 1.9.9.2 | :sparkles: add SLINK routing information ports (compatible to AXI-stream's `TID` and `TDEST` signals) | [#908](https://github.com/stnolting/neorv32/pull/908) |
| 04.05.2024 | 1.9.9.1 | :sparkles: add NEORV32 as Vivado IP block | [#894](https://github.com/stnolting/neorv32/pull/894) |
| 03.05.2024 | [**:rocket:1.9.9**](https://github.com/stnolting/neorv32/releases/tag/v1.9.9) | **New release** | |
| 02.05.2024 | 1.9.8.10 | :bug: fix UART receiver bug (introduced in v1.9.8.7) | [#891](https://github.com/stnolting/neorv32/pull/891) |
| 01.05.2024 | 1.9.8.9 | minor rtl cleanups (cleanup boolean expressions) | [#889](https://github.com/stnolting/neorv32/pull/889) |
| 27.04.2024 | 1.9.8.8 | fix delayed halt when single-stepping into an exception | [#887](https://github.com/stnolting/neorv32/pull/887) |
| 24.04.2024 | 1.9.8.7 | minor RTL fixes | [#883](https://github.com/stnolting/neorv32/pull/883) |
| 23.04.2024 | 1.9.8.6 | :bug: fix on-chip-debugger external-halt-request vs. exception concurrency | [#882](https://github.com/stnolting/neorv32/pull/882) |
| 21.04.2024 | 1.9.8.5 | rtl cleanups and (area) optimizations | [#880](https://github.com/stnolting/neorv32/pull/880) |
| 16.04.2024 | 1.9.8.4 | :warning: use a 4-bit FIRQ select instead of a 16-bit FIRQ mask for DMA auto-trigger configuration | [#877](https://github.com/stnolting/neorv32/pull/877) |
| 15.04.2024 | 1.9.8.3 | :warning: simplify XBUS gateway logic and configuration generics; only "pipelined Wishbone" protocol is supported now | [#876](https://github.com/stnolting/neorv32/pull/876) |
| 14.04.2024 | 1.9.8.2 | :warning: rename SLINK data interface registers; minor CPU control logic/area optimizations | [#874](https://github.com/stnolting/neorv32/pull/874) |
| 13.04.2024 | 1.9.8.1 | minor rtl code cleanups and optimizations | [#872](https://github.com/stnolting/neorv32/pull/872) |
| 04.04.2024 | [**:rocket:1.9.8**](https://github.com/stnolting/neorv32/releases/tag/v1.9.8) | **New release** | |
| 04.04.2024 | 1.9.7.10 | extend SPI and SDI interrupt conditions | [#870](https://github.com/stnolting/neorv32/pull/870) |
| 04.04.2024 | 1.9.7.9 | RISC-V `B` ISA extension (bit-manipulation) only contains sub-extensions `Zba+Zbb+Zbs`; :warning: remove support for `Zbc` ISA extension | [#869](https://github.com/stnolting/neorv32/pull/869) |
| 03.04.2024 | 1.9.7.8 | split SLINK interrupt into two individual FIRQs (SLINK RX and SLINK TX) | [#868](https://github.com/stnolting/neorv32/pull/868) |
| 01.04.2024 | 1.9.7.7 | add back TWI clock stretching option | [#867](https://github.com/stnolting/neorv32/pull/867) |
| 26.03.2024 | 1.9.7.6 | :warning: rework TWI module; add optional & configurable command/data FIFO | [#865](https://github.com/stnolting/neorv32/pull/865) |
| 24.03.2024 | 1.9.7.5 | :warning: **interrupt system rework**: rework CPU's FIRQ system; `mip` CSR is now read-only ; :bug: fix DMA fence configuration flag | [#864](https://github.com/stnolting/neorv32/pull/864) |
| 23.03.2024 | 1.9.7.4 | :warning: **interrupt system rework**: rework TWI and XIRQ interrupts | [#860](https://github.com/stnolting/neorv32/pull/860) |
| 23.03.2024 | 1.9.7.3 | :warning: **interrupt system rework**: rework ONEWIRE and GPTMR interrupts | [#859](https://github.com/stnolting/neorv32/pull/859) |
| 23.03.2024 | 1.9.7.2 | :warning: **interrupt system rework**: removed WDT and TRNG interrupts; :bug: fix core complex clocking during sleep mode | [#858](https://github.com/stnolting/neorv32/pull/858) |
| 23.03.2024 | 1.9.7.1 | CPU hardware optimization (reduced hardware footprint, shortened critical path) | [#857](https://github.com/stnolting/neorv32/pull/857) |
| 22.03.2024 | [**:rocket:1.9.7**](https://github.com/stnolting/neorv32/releases/tag/v1.9.7) | **New release** | |
| 18.03.2024 | 1.9.6.9 | :sparkles: update CFU example: now implementing the Extended Tiny Encryption Algorithm (XTEA) | [#855](https://github.com/stnolting/neorv32/pull/855) |
| 16.03.2024 | 1.9.6.8 | rework cache system: L1 + L2 caches, all based on the generic cache component | [#853](https://github.com/stnolting/neorv32/pull/853) |
| 16.03.2024 | 1.9.6.7 | cache optimizations: add read-only option, add option to disable direct/uncached accesses | [#851](https://github.com/stnolting/neorv32/pull/851) |
| 15.03.2024 | 1.9.6.6 | :warning: clean-up configuration generics (remove XBUS endianness configuration; refine JEDED/VENDORID configuration); rearrange SYSINFO.SOC bits | [#850](https://github.com/stnolting/neorv32/pull/850) |
| 14.03.2024 | 1.9.6.5 | :sparkles: add optional external bus interface cache (XCACHE) | [#849](https://github.com/stnolting/neorv32/pull/849) |
| 12.03.2024 | 1.9.6.4 | :warning: :warning: rename external bus/memory interface and according generics ("WISHBONE/MEM_EXT" -> "XBUS"); also rename bus interface ports (`wb_* -> xbus_*`) | [#846](https://github.com/stnolting/neorv32/pull/846) |
| 11.03.2024 | 1.9.6.3 | :warning: remove Wishbone tag signal; minor rtl edits and optimizations | [#845](https://github.com/stnolting/neorv32/pull/845) |
| 10.03.2024 | 1.9.6.2 | minor rtl clean-ups, optimizations and fixes | [#843](https://github.com/stnolting/neorv32/pull/843) |
| 09.03.2024 | 1.9.6.1 | add generic cache module (not used yet) | [#842](https://github.com/stnolting/neorv32/pull/842) |
| 01.03.2024 | [**:rocket:1.9.6**](https://github.com/stnolting/neorv32/releases/tag/v1.9.6) | **New release** | |
| 25.02.2024 | 1.9.5.10 | :bug: fix minor GPTMR threshold configuration issue | [#834](https://github.com/stnolting/neorv32/pull/834) |
| 23.02.2024 | 1.9.5.9 | :bug: fix atomic write/clear/set accesses of clear-only CSR bits (re-fix of v1.9.5.6) | [#829](https://github.com/stnolting/neorv32/pull/829) |
| 23.02.2024 | 1.9.5.8 | optimize FIFO component to improve technology mapping (inferring blockRAM for "async read" configuration); :bug: fix SLINK status flag delay | [#828](https://github.com/stnolting/neorv32/pull/828) |
| 23.02.2024 | 1.9.5.7 | fix FIFO synthesis issue (Vivado cannot infer block RAM nor LUT-RAM) | [#827](https://github.com/stnolting/neorv32/pull/827) |
| 20.02.2024 | 1.9.5.6 | :bug: fix bug in `mip.firq` CSR access; `mip.firq` bits are now read-write - software can trigger FIRQs by writing `1` to the according CSR bit | [#821](https://github.com/stnolting/neorv32/pull/821) |
| 19.02.2024 | 1.9.5.5 | SLINK: add native hardware support for AXI-stream's "tlast" signal | [#815](https://github.com/stnolting/neorv32/pull/815) |
| 19.02.2024 | 1.9.5.4 | :warning: remove support of `Smcntrpmf` ISA extension (counter privilege mode filtering) | [#814](https://github.com/stnolting/neorv32/pull/814) |
| 17.02.2024 | 1.9.5.3 | :warning: reworked CPU's hardware performance monitor (HPMs) events | [#811](https://github.com/stnolting/neorv32/pull/811) |
| 16.02.2024 | 1.9.5.2 | :warning: **revert** support for page faults (keep that in mmu branch for now) | [#809](https://github.com/stnolting/neorv32/pull/809) |
| 16.02.2024 | 1.9.5.1 | :sparkles: add two new generics to exclude certain PMP modes from synthesis | [#808](https://github.com/stnolting/neorv32/pull/808) |
| 16.02.2024 | [**:rocket:1.9.5**](https://github.com/stnolting/neorv32/releases/tag/v1.9.5) | **New release** | |
| 15.02.2024 | 1.9.4.13 | allow the DMA to issue a FENCE operation | [#807](https://github.com/stnolting/neorv32/pull/807) |
| 14.02.2024 | 1.9.4.12 | :lock: close another illegal compressed instruction encoding loophole | [#806](https://github.com/stnolting/neorv32/pull/806) |
| 11.02.2024 | 1.9.4.11 | :bug: fix several FPU bugs and design flaws | [#794](https://github.com/stnolting/neorv32/pull/794) |
| 11.02.2024 | 1.9.4.10 | minor additions to previous version (1.9.4.9): fix HPM configuration read-back | [#804](https://github.com/stnolting/neorv32/pull/804) |
| 10.02.2024 | 1.9.4.9 | fixing HPM configuration's null range assertions | [#803](https://github.com/stnolting/neorv32/pull/803) |
| 10.02.2024 | 1.9.4.8 | :bug: fix missing fence pass-through in caches | [#802](https://github.com/stnolting/neorv32/pull/802) |
| 09.02.2024 | 1.9.4.7 | :warning: integrate fence signal into CPU bus, remove top entity's fence signals | [#800](https://github.com/stnolting/neorv32/pull/800) |
| 09.02.2024 | 1.9.4.6 | :sparkles: add configurable XIP cache | [#799](https://github.com/stnolting/neorv32/pull/799) |
| 09.02.2024 | 1.9.4.5 | :lock: close further illegal compressed instruction encoding loopholes | [#797](https://github.com/stnolting/neorv32/pull/797) |
| 04.02.2024 | 1.9.4.4 | :bug: fix minor bug: CPU instruction bus privilege signal did not remain stable during the entire request | [#792](https://github.com/stnolting/neorv32/pull/792) |
| 03.02.2024 | 1.9.4.3 | :bug: fix minor bug: CPU instruction bus privilege signal was hardwired to "user-mode" | [#790](https://github.com/stnolting/neorv32/pull/790) |
| 01.02.2024 | 1.9.4.2 | :sparkles: add support for page fault exceptions (yet unused) | [#786](https://github.com/stnolting/neorv32/pull/786) |
| 31.01.2024 | 1.9.4.1 | fix trap priority | [#784](https://github.com/stnolting/neorv32/pull/784) |
| 31.01.2024 | [**:rocket:1.9.4**](https://github.com/stnolting/neorv32/releases/tag/v1.9.4) | **New release** | |
| 31.01.2024 | 1.9.3.10 | close illegal compressed instruction decoding loophole | [#783](https://github.com/stnolting/neorv32/pull/783) |
| 29.01.2024 | 1.9.3.9 | :test_tube: extend switchable clock domain (CPU bus switch, i-cache, d-cache) | [#780](https://github.com/stnolting/neorv32/pull/780) |
| 29.01.2024 | 1.9.3.8 | top entity input ports now have default values `'L'` or `'h'` modeling a pull-down or pull-resistor in case they are not explicitly assigned during instantiation | [#779](https://github.com/stnolting/neorv32/pull/779) |
| 28.01.2024 | 1.9.3.7 | FIFO module _NULL assertion_ fix | [#778](https://github.com/stnolting/neorv32/pull/778) |
| 27.01.2024 | 1.9.3.6 | improve CPU's front end (instruction fetch) increasing overall performance | [#777](https://github.com/stnolting/neorv32/pull/777) |
| 27.01.2024 | 1.9.3.5 | :bug: fix typo that renders the clock gating (added in v1.9.3.4) useless: CPU sleep output stuck at zero | [#776](https://github.com/stnolting/neorv32/pull/776) |
| 24.01.2024 | 1.9.3.4 | :sparkles: add optional CPU clock gating (via new generic `CLOCK_GATING_EN`): shut down the CPU clock during sleep mode; :warning: add new HDL design file for the clock gate (`neorv32_clockgate.vhd`) | [#775](https://github.com/stnolting/neorv32/pull/775) |
| 23.01.2024 | 1.9.3.3 | :bug: remove compressed floating point load/store operations as they are **not** supported by `Zfinx` | [#771](https://github.com/stnolting/neorv32/pull/771) |
| 20.01.2024 | 1.9.3.2 | optimize bus switch; minor RTL and comment edits | [#769](https://github.com/stnolting/neorv32/pull/769) |
| 14.01.2024 | 1.9.3.1 | minor rtl cleanups and optimizations | [#764](https://github.com/stnolting/neorv32/pull/764) |
| 11.01.2024 | [**:rocket:1.9.3**](https://github.com/stnolting/neorv32/releases/tag/v1.9.3) | **New release** | |
| 10.01.2024 | 1.9.2.11 | minor HDL fix (introduced in v1.9.2.9) | [#763](https://github.com/stnolting/neorv32/pull/763) |
| 10.01.2024 | 1.9.2.10 | re-add MTIME system time output to processor top (`mtime_time_o`) | [#762](https://github.com/stnolting/neorv32/pull/762) |
| 08.01.2024 | 1.9.2.9 | minor rtl code cleanups | [#760](https://github.com/stnolting/neorv32/pull/760) |
| 06.01.2024 | 1.9.2.8 | :sparkles: add timer-capture mode to General Purpose Timer (GPTMR); :warning: remove "single-shot" mode, change control register layout | [#759](https://github.com/stnolting/neorv32/pull/759) |
| 19.12.2023 | 1.9.2.7 | minor rtl code cleanups, edits and optimization; :lock: reset `mtvec`, `mepc` and `dpc` CSRs to CPU boot address (`CPU_BOOT_ADDR` CPU generic) | [#755](https://github.com/stnolting/neorv32/pull/755) |
| 19.12.2023 | 1.9.2.6 | rework FIFO component fixing problems with inferring block RAM | [#754](https://github.com/stnolting/neorv32/pull/754) |
| 11.12.2023 | 1.9.2.5 | clean-up software framework | [#752](https://github.com/stnolting/neorv32/pull/752) |
| 09.12.2023 | 1.9.2.4 | minor rtl code cleanups | [#747](https://github.com/stnolting/neorv32/pull/747) |
| 09.12.2023 | 1.9.2.3 | refine behavior of CPU's sleep state & signal | [#746](https://github.com/stnolting/neorv32/pull/746) |
| 05.12.2023 | 1.9.2.2 | reset `mstatus.mpp` to "machine-mode" | [#745](https://github.com/stnolting/neorv32/pull/745) |
| 02.12.2023 | 1.9.2.1 | :sparkles: add RISC-V `Zicond` ISA extension (integer conditional operations) | [#743](https://github.com/stnolting/neorv32/pull/743) |
| 01.12.2023 | [**:rocket:1.9.2**](https://github.com/stnolting/neorv32/releases/tag/v1.9.2) | **New release** | |
| 01.12.2023 | 1.9.1.9 | add `menvcfg[h]` CSRs | [#741](https://github.com/stnolting/neorv32/pull/741) |
| 30.11.2023 | 1.9.1.8 | :sparkles: :bug: upgrade RISC-V hardware trigger module (`Sdtrig` ISA extension) to spec. version v1.0 (fixing several minor bugs) | [#739](https://github.com/stnolting/neorv32/pull/739) |
| 25.11.2023 | 1.9.1.7 | cleanup/update assertions and auto-adjusting of invalid generic configurations | [#738](https://github.com/stnolting/neorv32/pull/738) |
| 25.11.2023 | 1.9.1.6 | :sparkles: add option for "ASIC style" register file that provides a full/dedicated hardware reset | [#736](https://github.com/stnolting/neorv32/pull/736) |
| 23.11.2023 | 1.9.1.5 | clean-up & rework CPU branch logic | [#735](https://github.com/stnolting/neorv32/pull/735) |
| 21.11.2023 | 1.9.1.4 | :bug: fix bug in handling of "misaligned instruction exception" | [#734](https://github.com/stnolting/neorv32/pull/734) |
| 20.11.2023 | 1.9.1.3 | :bug: fix wiring of FPU exception flags | [#733](https://github.com/stnolting/neorv32/pull/733) |
| 18.11.2023 | 1.9.1.2 | add XIP clock divider to fine-tune SPI frequency | [#731](https://github.com/stnolting/neorv32/pull/731) |
| 18.11.2023 | 1.9.1.1 | (re-)add SPI high-speed mode, :bug: fix bug in SPI shift register - introduced in v1.9.0.9 | [#730](https://github.com/stnolting/neorv32/pull/730) |
| 14.11.2023 | [**:rocket:1.9.1**](https://github.com/stnolting/neorv32/releases/tag/v1.9.1) | **New release** | |
| 11.11.2023 | 1.9.0.9 | :test_tube: add full hardware reset for **all** flip flops in CPU/processor | [#724](https://github.com/stnolting/neorv32/pull/724) |
| 09.11.2023 | 1.9.0.8 | minor rtl code cleanups | [#723](https://github.com/stnolting/neorv32/pull/723) |
| 04.11.2023 | 1.9.0.7 | upgrade true random number generator to [neoTRNG version 3](https://github.com/stnolting/neoTRNG) | [#721](https://github.com/stnolting/neorv32/pull/721) |
| 31.10.2023 | 1.9.0.6 | update crt0's early-boot trap handler | [#719](https://github.com/stnolting/neorv32/pull/719) |
| 30.10.2023 | 1.9.0.5 | minor rtl cleanups and code beautification | [#718](https://github.com/stnolting/neorv32/pull/718) |
| 28.10.2023 | 1.9.0.4 | :warning: :sparkles: move FreeRTOS port and demo to a new repository: https://github.com/stnolting/neorv32-freertos | [#716](https://github.com/stnolting/neorv32/pull/716) |
| 18.10.2023 | 1.9.0.3 | :warning: remove top's `CPU_EXTENSION_RISCV_Zifencei` generic - `Zifencei` ISA extension is now always enabled | [#709](https://github.com/stnolting/neorv32/pull/709) |
| 16.10.2023 | 1.9.0.2 | minor CPU control cleanups and optimizations (branch system) | [#707](https://github.com/stnolting/neorv32/pull/707) |
| 13.10.2023 | 1.9.0.1 | update software framework to GCC-13.2.0 | [#705](https://github.com/stnolting/neorv32/pull/705) |
| 13.10.2023 | [**:rocket:1.9.0**](https://github.com/stnolting/neorv32/releases/tag/v1.9.0) | **New release** | |
| 13.10.2023 | 1.8.9.9 | minor hardware edits and optimizations | [#703](https://github.com/stnolting/neorv32/pull/703) |
| 07.10.2023 | 1.8.9.8 | add "transfer done" flag to DMA | [#699](https://github.com/stnolting/neorv32/pull/699) |
| 04.10.2023 | 1.8.9.7 | :warning: rework internal bus protocol | [#697](https://github.com/stnolting/neorv32/pull/697) |
| 29.09.2023 | 1.8.9.6 | optimize PMP logic (reducing area requirements) | [#695](https://github.com/stnolting/neorv32/pull/695) |
| 29.09.2023 | 1.8.9.5 | minor CPU optimizations and code clean-ups | [#694](https://github.com/stnolting/neorv32/pull/694) |
| 23.09.2023 | 1.8.9.4 | :sparkles: added vectored trap handling mode of `mtvec` for reduced latency from IRQ to ISR | [#691](https://github.com/stnolting/neorv32/pull/691)
| 22.09.2023 | 1.8.9.3 | :lock: **watchdog**: add reset password and optional "strict" mode for increased safety | [#692](https://github.com/stnolting/neorv32/pull/692) |
| 15.09.2023 | 1.8.9.2 | :warning: rework CFU CSRs; minor rtl edits | [#690](https://github.com/stnolting/neorv32/pull/690) |
| 11.09.2023 | 1.8.9.1 | minor rtl edits and updates | [#684](https://github.com/stnolting/neorv32/pull/684) |
| 09.09.2023 | [**:rocket:1.8.9**](https://github.com/stnolting/neorv32/releases/tag/v1.8.9) | **New release** | |
| 08.09.2023 | 1.8.8.9 | removed unused `mcontext` and `scontext` CSRs (`Sdtrig` ISA extension); CPU optimizations (area and timing) | [#683](https://github.com/stnolting/neorv32/pull/683) |
| 02.09.2023 | 1.8.8.8 | :sparkles: add option to implement **up to 2^32 CFU-internal user-defined CSRs** (via indirect CSR access) | [#681](https://github.com/stnolting/neorv32/pull/681) |
| 02.09.2023 | 1.8.8.7 | :lock: (re-)add **execution monitor**: raise an exception if a multi-cycle ALU operation does not complete within a bound amount of time | [#680](https://github.com/stnolting/neorv32/pull/680) |
| 01.09.2023 | 1.8.8.6 | minor rtl edits and cleanups | [#679](https://github.com/stnolting/neorv32/pull/679) |
| 30.08.2023 | 1.8.8.5 | remove "branch prediction" logic - core is smaller and _even faster_ without it | [#678](https://github.com/stnolting/neorv32/pull/678) |
| 25.08.2023 | 1.8.8.4 | add new generic to downgrade on-chip debugger's debug module back to spec. version 0.13 (`DM_LEGACY_MODE` generic) | [#677](https://github.com/stnolting/neorv32/pull/677) |
| 23.08.2023 | 1.8.8.3 | :test_tube: add experimental `Smcntrpmf` ISA extension (counter privilege mode filtering; spec. is frozen but not yet ratified); remove unused `menvcfg` CSRs | [#676](https://github.com/stnolting/neorv32/pull/676) |
| 19.08.2023 | 1.8.8.2 | :warning: constrain `mtval` CSR; add support for `mtinst` CSR (trap instruction) | [#674](https://github.com/stnolting/neorv32/pull/674) |
| 19.08.2023 | 1.8.8.1 | :test_tube: update RTE to support easy emulation of instructions; add example program to showcase how to emulate unaligned memory accesses | [#673](https://github.com/stnolting/neorv32/pull/673) |
| 18.08.2023 | [**:rocket:1.8.8**](https://github.com/stnolting/neorv32/releases/tag/v1.8.8) | **New release** | |
| 17.08.2023 | 1.8.7.9 | minor rtl edits and cleanups | [#672](https://github.com/stnolting/neorv32/pull/672) |
| 13.08.2023 | 1.8.7.8 | :warning: constrain/optimize `mtval` and `mcounteren` CSRs | [#671](https://github.com/stnolting/neorv32/pull/671) |
| 12.08.2023 | 1.8.7.7 | remove _unratified_ `Zicond` ISA extension; minor rtl code cleanups and optimizations | [#670](https://github.com/stnolting/neorv32/pull/670) |
| 05.08.2023 | 1.8.7.6 | :bug: fix bug: HPM width configurations below 32 bit fail | [#665](https://github.com/stnolting/neorv32/pull/665) |
| 04.08.2023 | 1.8.7.5 | :warning: major code edits/cleanups and file renaming | [#664](https://github.com/stnolting/neorv32/pull/664) |
| 29.07.2023 | 1.8.7.4 | RTL cleanup and optimizations (less synthesis warnings, less resource requirements) | [#660](https://github.com/stnolting/neorv32/pull/660) |
| 28.07.2023 | 1.8.7.3 | :warning: reworked **SYSINFO** module; clean-up address space layout; clean-up assertion notes | [#659](https://github.com/stnolting/neorv32/pull/659) |
| 27.07.2023 | 1.8.7.2 | :bug: make sure that IMEM/DMEM size is always a power of two | [#658](https://github.com/stnolting/neorv32/pull/658) |
| 27.07.2023 | 1.8.7.1 | :warning: remove `CUSTOM_ID` generic; cleanup and re-layout `NEORV32_SYSINFO.SOC` bits; (:bug:) fix gateway's generics (`positive` -> `natural` as these generics are allowed to be zero) | [#657](https://github.com/stnolting/neorv32/pull/657) |
| 26.07.2023 | [**:rocket:1.8.7**](https://github.com/stnolting/neorv32/releases/tag/v1.8.7) | **New release** | |
| 24.07.2023 | 1.8.6.10 | :bug: fixing some LR/SC design flaws | [#654](https://github.com/stnolting/neorv32/pull/654) |
| 23.07.2023 | 1.8.6.9 | optimize bus system and customization options | [#653](https://github.com/stnolting/neorv32/pull/653) |
| 22.07.2023 | 1.8.6.8 | minor rtl edits | [#652](https://github.com/stnolting/neorv32/pull/652) |
| 21.07.2023 | 1.8.6.7 | :sparkles: add support for **RISC-V A ISA Extension** (atomic memory accesses; `lr.w`/`sc.w` only!) | [#651](https://github.com/stnolting/neorv32/pull/651) |
| 18.07.2023 | 1.8.6.6 | :warning: remove 32-bit data dump mode from UART0/1 sim-mode | [#650](https://github.com/stnolting/neorv32/pull/650) |
| 16.07.2023 | 1.8.6.5 | :warning: **rework SoC bus system & memory map - part 3**: re-enforce PMAs (physical memory attributes) | [#648](https://github.com/stnolting/neorv32/pull/648) |
| 15.07.2023 | 1.8.6.4 | :warning: **rework SoC bus system & memory map - part 2**: move IO address decoding to central IO switch; add i-cache uncached accesses | [#648](https://github.com/stnolting/neorv32/pull/648) |
| 14.07.2023 | 1.8.6.3 | :warning: **rework SoC bus system & memory map - part 1**: add central bus gateway to control core accesses to the main address regions | [#648](https://github.com/stnolting/neorv32/pull/648) |
| 14.07.2023 | 1.8.6.2 | minor rtl edits | [#646](https://github.com/stnolting/neorv32/pull/646) |
| 30.06.2023 | 1.8.6.1 | minor rtl edits, cleanups and optimizations | [#641](https://github.com/stnolting/neorv32/pull/641) |
| 27.06.2023 | [**:rocket:1.8.6**](https://github.com/stnolting/neorv32/releases/tag/v1.8.6) | **New release** | |
| 24.06.2023 | 1.8.5.9 | :test_tube: VHDL code: use entity instantiation instead of component instantiation | [#637](https://github.com/stnolting/neorv32/pull/637) |
| 24.06.2023 | 1.8.5.8 | optimize CPU control logic; closed further invalid instruction word detection holes | [#636](https://github.com/stnolting/neorv32/pull/636) |
| 23.06.2023 | 1.8.5.7 | :warning: remove **buskeeper's status register** | [#635](https://github.com/stnolting/neorv32/pull/635) |
| 17.06.2023 | 1.8.5.6 | :sparkles: add new **Cyclic Redundancy Check module (CRC)** | [#632](https://github.com/stnolting/neorv32/pull/632) |
| 03.06.2023 | 1.8.5.5 | :sparkles: re-add (simplified) **Stream Link Interface (SLINK)** | [#628](https://github.com/stnolting/neorv32/pull/628) |
| 03.06.2023 | 1.8.5.4 | :warning: rearrange bits in **SYSINFO** | [#627](https://github.com/stnolting/neorv32/pull/627) |
| 02.06.2023 | 1.8.5.3 | :bug: executable generation: fix address continuity between `.text` and `.rodata` segments | [#626](https://github.com/stnolting/neorv32/pull/626) |
| 19.05.2023 | 1.8.5.2 | :sparkles: add automatic trigger mode to **DMA** (trigger transfer if a processor-internal peripheral issues an interrupt request) | [#618](https://github.com/stnolting/neorv32/pull/618) |
| 18.05.2023 | 1.8.5.1 | software can now retrieve the configured FIFO size of the **TRNG** | [#616](https://github.com/stnolting/neorv32/pull/616) |
| 18.05.2023 | [**:rocket:1.8.5**](https://github.com/stnolting/neorv32/releases/tag/v1.8.5) | **New release** | |
| 18.05.2023 | 1.8.4.9 | remove `is_simulation` flag from SYSINFO; add programmable interrupt to **TRNG** module | [#615](https://github.com/stnolting/neorv32/pull/615) |
| 12.05.2023 | 1.8.4.8 | `mtval` CSR now provides the address of `ebreak` exceptions (re-added temporarily to pass RISC-V ISA tests) | [#611](https://github.com/stnolting/neorv32/pull/611) |
| 03.05.2023 | 1.8.4.7 | :bug: fix bug in FPU (terminate FPU sub-module operations if an exception has been raised) | [#609](https://github.com/stnolting/neorv32/pull/609) |
| 02.05.2023 | 1.8.4.6 | make SDI FIFO access entirely synchronous; upgrade processor memory modules; update test setup wrappers | [#608](https://github.com/stnolting/neorv32/pull/608) |
| 30.04.2023 | 1.8.4.5 | rework processor-internal bus system | [#607](https://github.com/stnolting/neorv32/pull/607) |
| 27.04.2023 | 1.8.4.4 | minor hardware edits and switching activity optimizations of CPU bus unit | [#605](https://github.com/stnolting/neorv32/pull/605) |
| 25.04.2023 | 1.8.4.3 | :bug: fix bug in **DMA** (corrupted write-back when there are bus wait cycles - e.g. when no caches are implemented) | [#601](https://github.com/stnolting/neorv32/pull/601) |
| 24.04.2023 | 1.8.4.2 | minor rtl edits; shorten critical path of d-cache setup | [#599](https://github.com/stnolting/neorv32/pull/599) |
| 22.04.2023 | 1.8.4.1 | :sparkles: add optional **direct memory access controller (DMA)** | [#593](https://github.com/stnolting/neorv32/pull/593) |
| 21.04.2023 | [**:rocket:1.8.4**](https://github.com/stnolting/neorv32/releases/tag/v1.8.4) | **New release** | |
| 21.04.2023 | 1.8.3.9 | :bug: fix timeout bug in **FPU** normalizer | [#592](https://github.com/stnolting/neorv32/pull/592) |
| 19.04.2023 | 1.8.3.8 | minor processor bus system optimizations and clean-ups | [#591](https://github.com/stnolting/neorv32/pull/591) |
| 15.04.2023 | 1.8.3.7 | :bug: :warning: `wfi` and XIRQ bug fixes; massive RTL code cleanup and optimization of CPU control | [#586](https://github.com/stnolting/neorv32/pull/586) |
| 14.04.2023 | 1.8.3.6 | [UARTs] software can now retrieve the configured RX/TX FIFO sizes from the `DATA` register | [#581](https://github.com/stnolting/neorv32/pull/581) |
| 13.04.2023 | 1.8.3.5 | :bug: fixed bug in FPU control logic (introduced in some earlier clean-up commit); minor code edits and optimizations | [#578](https://github.com/stnolting/neorv32/pull/578) |
| 07.04.2023 | 1.8.3.4 | rtl edits and cleanups | [#571](https://github.com/stnolting/neorv32/pull/571) |
| 05.04.2023 | 1.8.3.3 | update **external interrupt controller (XIRQ)** | [#570](https://github.com/stnolting/neorv32/pull/570) |
| 05.04.2023 | 1.8.3.2 | `time` CSR struggles (again) and logic optimization | [#569](https://github.com/stnolting/neorv32/pull/569) |
| 01.04.2023 | 1.8.3.1 | :sparkles: add full `NA4` and `NAPOT` support to the (now) RISC-V-compatible **physical memory protection (PMP)** | [#566](https://github.com/stnolting/neorv32/pull/566) |
| 31.03.2023 | [**:rocket:1.8.3**](https://github.com/stnolting/neorv32/releases/tag/v1.8.3) | **New release** | |
| 29.03.2023 | 1.8.2.9 | :warning: remove `CPU_EXTENSION_RISCV_Zicsr` generic - `Zicsr` ISA extension is always enabled; optimize bus switch; VHDL code cleanups | [#562](https://github.com/stnolting/neorv32/pull/562) |
| 25.03.2023 | 1.8.2.8 | :test_tube: add configurable data cache (**dCACHE**) | [#560](https://github.com/stnolting/neorv32/pull/560) |
| 24.03.2023 | 1.8.2.7 | :sparkles: add full support of `mcounteren` CSR; cleanup counter and PMP CSRs; i-cache optimization | [#559](https://github.com/stnolting/neorv32/pull/559) |
| 18.03.2023 | 1.8.2.6 | add new generic `JEDEC_ID` (official JEDEC identifier; used for `mvendorid` CSR); further generics cleanups | [#557](https://github.com/stnolting/neorv32/pull/557)
| 17.03.2023 | 1.8.2.5 | add RISC-V `time[h]` CSRs (part of the `Zicntr` ISA extension) | [#556](https://github.com/stnolting/neorv32/pull/556) |
| 17.03.2023 | 1.8.2.4 | re-add VHDL process names | [#555](https://github.com/stnolting/neorv32/pull/555) |
| 15.03.2023 | 1.8.2.3 | rtl reworks, cleanups and optimizations | [#550](https://github.com/stnolting/neorv32/pull/550) |
| 11.03.2023 | 1.8.2.2 | :sparkles: add support for RISC-V `Zicond` ISA extension (conditional operations) | [#546](https://github.com/stnolting/neorv32/pull/546) |
| 10.03.2023 | 1.8.2.1 | rtl code edits, clean-ups and minor optimizations (improve branch prediction) | [#545](https://github.com/stnolting/neorv32/pull/545) |
| 10.03.2023 | [**:rocket:1.8.2**](https://github.com/stnolting/neorv32/releases/tag/v1.8.2) | **New release** | |
| 09.03.2023 | 1.8.1.10 | :warning: move tri-state drivers (ONEWIRE and TWI) out of the core | [#543](https://github.com/stnolting/neorv32/pull/543) |
| 08.03.2023 | 1.8.1.9 | reintegrate **UART** RTS/CTS hardware flow-control | [#541](https://github.com/stnolting/neorv32/pull/541) |
| 07.03.2023 | 1.8.1.8 | update smart LED controller **NEOLED** | [#536](https://github.com/stnolting/neorv32/pull/536) |
| 05.03.2023 | 1.8.1.7 | :warning: rework and update **UART0 & UART1** | [#533](https://github.com/stnolting/neorv32/pull/533) |
| 04.03.2023 | 1.8.1.6 | :warning: rework and update **SPI** module | [#530](https://github.com/stnolting/neorv32/pull/530) |
| 02.03.2023 | 1.8.1.5 | minor general purpose timer (GPTMR) code edits | [#529](https://github.com/stnolting/neorv32/pull/529) |
| 02.03.2023 | 1.8.1.4 | :bug: fix timeout bug in **FPU** (conversion and add/sub instructions) | [#528](https://github.com/stnolting/neorv32/pull/528) |
| 25.02.2023 | 1.8.1.3 | :sparkles: add new processor module: **Serial Data Interface (SDI)** - a SPI _device-class_ interface | [#505](https://github.com/stnolting/neorv32/pull/505) |
| 24.02.2023 | 1.8.1.2 | :warning: rename top interface signals of **XIP** and **SIP** modules | [#504](https://github.com/stnolting/neorv32/pull/504) |
| 23.02.2023 | 1.8.1.1 | CFS: add another 32 interface register (now having 64 memory-mapped registers for custom usage) | [#503](https://github.com/stnolting/neorv32/pull/503) |
| 23.02.2023 | [**:rocket:1.8.1**](https://github.com/stnolting/neorv32/releases/tag/v1.8.1) | **New release** | |
| 22.02.2023 | 1.8.0.10 | :warning: **remove stream link interface (SLINK)** | [#502](https://github.com/stnolting/neorv32/pull/502) |
| 19.02.2023 | 1.8.0.9 | :warning: constrain number of **PWM** channels to 12 (was 60); change base address of PWM module | [#501](https://github.com/stnolting/neorv32/pull/501) |
| 18.02.2023 | 1.8.0.8 | :bug: fix minor bug in CPU's co-processor monitor; minor VHDL clean-ups and edits | [#500](https://github.com/stnolting/neorv32/pull/500) |
| 13.02.2023 | 1.8.0.7 | minor CPU optimization and fixes | [#497](https://github.com/stnolting/neorv32/pull/497) |
| 11.02.2023 | 1.8.0.6 | :warning: replace boolean `IO_GPIO_EN` generic by natural `IO_GPIO_NUM` generic to fine-tune GPIO pin number | [#491](https://github.com/stnolting/neorv32/pull/491) |
| 10.02.2023 | 1.8.0.5 | :test_tube: add CPU co-processor monitor (to auto-terminate operation if a co-processor operation takes too long) | [#490](https://github.com/stnolting/neorv32/pull/490) |
| 10.02.2023 | 1.8.0.4 | replace CPU-internal control bus by a VHDL `record` (much cleaner code); minor control optimizations; add 6ht CPU co-processor slot (yet unused) | [#489](https://github.com/stnolting/neorv32/pull/489) |
| 05.02.2023 | 1.8.0.3 | CPU control optimizations | [#487](https://github.com/stnolting/neorv32/pull/487) |
| 04.02.2023 | 1.8.0.2 | fix RISC-V-incompatible behavior of `mip` CSR | [#486](https://github.com/stnolting/neorv32/pull/486) |
| 01.02.2023 | 1.8.0.1 | clean-up CPU's interrupt controller; fix race condition in FIRQ trigger/acknowledge | [#484](https://github.com/stnolting/neorv32/pull/484) |
| 25.01.2023 | [**:rocket:1.8.0**](https://github.com/stnolting/neorv32/releases/tag/v1.8.0) | **New release** | |
| 21.01.2023 | 1.7.9.10 | update software framework; :bug: fix bug in constructor calling in `crt0` start-up code | [#478](https://github.com/stnolting/neorv32/pull/478) |
| 15.01.2023 | 1.7.9.9 | :warning: rework **CPU counters**; remove `mtime_i/o` top entity ports; remove `time[h]` CSRs | [#477](https://github.com/stnolting/neorv32/pull/477) |
| 14.01.2023 | 1.7.9.8 | minor CPU control edits, optimizations and fixes | [#476](https://github.com/stnolting/neorv32/pull/476) |
| 10.01.2023 | 1.7.9.7 | :warning: rework **watchdog timer (WDT)** | [#474](https://github.com/stnolting/neorv32/pull/474) |
| 06.01.2023 | 1.7.9.6 | update [neoTRNG v2](https://github.com/stnolting/neoTRNG) | [#472](https://github.com/stnolting/neorv32/pull/472) |
| 06.01.2023 | 1.7.9.5 | CPU control: logic optimization and fix minor bug in trigger module | [#470](https://github.com/stnolting/neorv32/pull/470) |
| 04.01.2023 | 1.7.9.4 | update **on-chip debugger**: :test_tube: remove debug module's `haltsum0` register; rework DMI to comply with RISC-V debug spec.; minor edits, updates and fixes | [#468](https://github.com/stnolting/neorv32/pull/468) |
| 23.12.2022 | 1.7.9.3 | :warning: add explicit `Sdext` and `Sdtrig` ISA extension generics (replacing `DEBUG`); :sparkles: trigger-module can now also be used by machine-mode software without the on-chip debugger, add minimal example program `sw/example/demo_trigger_module` | [#465](https://github.com/stnolting/neorv32/pull/465) |
| 23.12.2022 | 1.7.9.2 | :sparkles: upgrade the **on-chip debugger (OCD)** to spec. version 1.0; major logic and debugging response time optimizations | [#463](https://github.com/stnolting/neorv32/pull/463) |
| 22.12.2022 | 1.7.9.1 | remove signal initialization (in reset generator) as some FPGAs do not support FF initialization via bitstream | [#464](https://github.com/stnolting/neorv32/pull/464) |
| 21.12.2022 | [**:rocket:1.7.9**](https://github.com/stnolting/neorv32/releases/tag/v1.7.9) | **New release** | |
| 21.12.2022 | 1.7.8.11 | CPU: remove explicit reset-to-don't-care; branch and CSR access check logic optimizations; close further illegal instruction encoding hole | [#462](https://github.com/stnolting/neorv32/pull/462) |
| 20.12.2022 | 1.7.8.10 | SOC: rework r/w access logic; split read and write accesses into two processes; removed explicit reset-to-don't-care | [#461](https://github.com/stnolting/neorv32/pull/461) |
| 18.12.2022 | 1.7.8.9 | `mtval` is no longer read-only and can now be written by machine-mode software | [#460](https://github.com/stnolting/neorv32/pull/460) |
| 17.12.2022 | 1.7.8.8 | :bug: fix incorrect value written to `mepc` when encountering an "instruction access fault" exception | [#458](https://github.com/stnolting/neorv32/pull/458) |
| 16.12.2022 | 1.7.8.7 | :bug: fix **instruction cache** block invalidation when a bus access error occurs during memory block fetch (after cache miss) | [#457](https://github.com/stnolting/neorv32/pull/457) |
| 16.12.2022 | 1.7.8.6 | :test_tube: optimized park-loop code (**on-chip debugger firmware**) providing slightly faster debugging response; added explicit address generics for defining debug mode entry points | [#456](https://github.com/stnolting/neorv32/pull/456) |
| 13.12.2022 | 1.7.8.5 | code cleanup of FIFO module; improved **instruction prefetch buffer (IPB)** - IPD depth can be as small as "1" and will be adjusted automatically when enabling the `C` ISA extension; update hardware implementation results | [#455](https://github.com/stnolting/neorv32/pull/455) |
| 09.12.2022 | 1.7.8.4 | :sparkles: new option to add custom **R5-type** (4 source registers, 1 destination register) instructions to **Custom Functions Unit (CFU)** | [#452](https://github.com/stnolting/neorv32/pull/452) |
| 08.12.2022 | 1.7.8.3 | :bug: fix interrupt behavior when in user-mode; minor core rtl fixes; do not check registers specifiers in CFU instructions (i.e. using registers above `x15` when `E` ISA extension is enabled) | [#450](https://github.com/stnolting/neorv32/pull/450) |
| 03.12.2022 | 1.7.8.2 | :sparkles: new option to add custom **R4-type** RISC-V instructions to **Custom Functions Unit (CFU)**; rework CFU hardware module, intrinsic library and example program | [#449](https://github.com/stnolting/neorv32/pull/449) |
| 01.12.2022 | 1.7.8.1 | package cleanup | [#447](https://github.com/stnolting/neorv32/pull/447) |
| 28.11.2022 | [**:rocket:1.7.8**](https://github.com/stnolting/neorv32/releases/tag/v1.7.8) | **New release** | |
| 14.11.2022 | 1.7.7.9 | minor rtl edits and code optimizations | [#442](https://github.com/stnolting/neorv32/pull/442) |
| 05.11.2022 | 1.7.7.8 | minor rtl edits | [#441](https://github.com/stnolting/neorv32/pull/441) |
| 03.11.2022 | 1.7.7.7 | :sparkles: add fine-grained clock configuration for **TWI** module: add fine-grained clock configuration, add clock stretching configuration flag | [#440](https://github.com/stnolting/neorv32/pull/440) |
| 01.11.2022 | 1.7.7.6 | :warning: rework **SPI module** | [#438](https://github.com/stnolting/neorv32/pull/438) |
| 24.10.2022 | 1.7.7.5 | :test_tube: remove weird Quartus latch warnings by modifying VHDL coding style | [#434](https://github.com/stnolting/neorv32/pull/434) |
| 19.10.2022 | 1.7.7.4 | optimize UART's `RTS` (hardware flow control) behavior | [#433](https://github.com/stnolting/neorv32/pull/433) |
| 15.10.2022 | 1.7.7.3 | :bug: fix bug in `is_power_of_two_f` VHDL function (thanks Alan!) | [#428](https://github.com/stnolting/neorv32/pull/428) |
| 12.10.2022 | 1.7.7.2 | add dedicated hardware reset to _all_ CPU counters (`[m]cycle[h]`, `[m]instret[h]`, `[m]hpmcounter[h]`); :sparkles: **all CSRs now provide a dedicated hardware reset** | [#426](https://github.com/stnolting/neorv32/pull/426) |
| 09.10.2022 | 1.7.7.1 | fix Quartus synthesis issue (VHDL): make sure reset state is the _first_ entry in a state list | [#423](https://github.com/stnolting/neorv32/pull/423) |
| 24.09.2022 | [**:rocket:1.7.7**](https://github.com/stnolting/neorv32/releases/tag/v1.7.7) | **New release** | |
| 23.09.2022 | 1.7.6.10 | cleanup native data path size (remove `data_width_c` package constant); initial preparations to **support RV64 ISA extension** somewhere in the future | [#417](https://github.com/stnolting/neorv32/pull/417) |
| 18.09.2022 | 1.7.6.9 | :bug: fixed instruction decoding collision in **`B` ISA extensions** - `B` extension is now fully operational and verified (see [neorv32-riscof](https://github.com/stnolting/neorv32-riscof))! | [#413](https://github.com/stnolting/neorv32/pull/413) |
| 13.09.2022 | 1.7.6.8 | :bug: bug fix: clearing `mie`'s FIRQ bits did not clear the according _pending_ FIRQs | [#411](https://github.com/stnolting/neorv32/pull/411) |
| 12.09.2022 | 1.7.6.7 | minor rtl edits and cleanups | [#410](https://github.com/stnolting/neorv32/pull/410) |
| 10.09.2022 | 1.7.6.6 | :warning: set `mtval` to _zero_ on any illegal instruction exception - removes redundancies, simplifies hardware | [#409](https://github.com/stnolting/neorv32/pull/409) |
| 09.09.2022 | 1.7.6.5 | minor rtl edits; add "output gate" to FIFO component | [#408](https://github.com/stnolting/neorv32/pull/408) |
| 08.09.2022 | 1.7.6.4 | :warning: cleanup CPU standard counters and remove _CPU_CNT_WIDTH_ generic | [#407](https://github.com/stnolting/neorv32/pull/407) |
| 07.09.2022 | 1.7.6.3 | minor rtl edits and cleanups | [#406](https://github.com/stnolting/neorv32/pull/406) |
| 03.09.2022 | 1.7.6.2 | cleanup hardware reset logic | [#405](https://github.com/stnolting/neorv32/pull/405) |
| 02.09.2022 | 1.7.6.1 | :sparkles: add new processor module: **1-Wire Interface Controller** (ONEWIRE) | [#402](https://github.com/stnolting/neorv32/pull/402) |
| 28.08.2022 | [**:rocket:1.7.6**](https://github.com/stnolting/neorv32/releases/tag/v1.7.6) | **New release** | |
| 27.08.2022 | 1.7.5.9 | fix minor core rtl issues that were found while experimenting with a low-level netlist of the processor | [#398](https://github.com/stnolting/neorv32/pull/398) |
| 26.08.2022 | 1.7.5.8 | cleanup **crt0** start-up code: remove setup of `mcountern` and `mcountinhibit` CSRs | [#397](https://github.com/stnolting/neorv32/pull/397) |
| 24.08.2022 | 1.7.5.7 | minor rtl cleanups | [#396](https://github.com/stnolting/neorv32/pull/396) |
| 20.08.2022 | 1.7.5.6 | :sparkles: update software framework to GCC 12.1.0 (new prebuilt toolchains available!) | [#391](https://github.com/stnolting/neorv32/pull/391) |
| 18.08.2022 | 1.7.5.5 | :lock: add **TRNG** read data protection | [#389](https://github.com/stnolting/neorv32/pull/389) |
| 18.08.2022 | 1.7.5.4 | minor rtl cleanup in **PWM** module | [#388](https://github.com/stnolting/neorv32/pull/388) |
| 17.08.2022 | 1.7.5.3 | optimized **CPU front-end** - faster instruction fetch | [#387](https://github.com/stnolting/neorv32/pull/387) |
| 16.08.2022 | 1.7.5.2 | relocate TWI tri-state drivers | [#386](https://github.com/stnolting/neorv32/pull/386) |
| 15.08.2022 | 1.7.5.1 | change base address of **BUSKEEPER** | [#385](https://github.com/stnolting/neorv32/pull/385) |
| 15.08.2022 | [**:rocket:1.7.5**](https://github.com/stnolting/neorv32/releases/tag/v1.7.5) | **New release** | |
| 14.08.2022 | 1.7.4.10 | cleanup of FIFO rtl component | [#384](https://github.com/stnolting/neorv32/pull/384) |
| 13.08.2022 | 1.7.4.9 | minor rtl cleanups and optimizations | [#383](https://github.com/stnolting/neorv32/pull/383) |
| 01.08.2022 | 1.7.4.8 | :sparkles: add configurable data FIFO to **SPI** module | [#381](https://github.com/stnolting/neorv32/pull/381) |
| 31.07.2022 | 1.7.4.7 | :warning: rework **SLINK** module | [#377](https://github.com/stnolting/neorv32/pull/377) |
| 25.07.2022 | 1.7.4.6 | :warning: simplify memory configuration of **linker script**; :sparkles: add in-console configuration option | [#375](https://github.com/stnolting/neorv32/pull/375) |
| 22.07.2022 | 1.7.4.5 | add `CUSTOM_ID` generic; update bootloader | [#374](https://github.com/stnolting/neorv32/pull/374) |
| 21.07.2022 | 1.7.4.4 | :lock: specify **physical memory attributes (PMA)** | [#372](https://github.com/stnolting/neorv32/pull/372) |
| 18.07.2022 | 1.7.4.3 | minor rtl edits and updates | [#369](https://github.com/stnolting/neorv32/pull/369) |
| 15.07.2022 | 1.7.4.2 | :bug: fixed PMP configuration error when `PMP_NUM_REGIONS` = 0 | [#368](https://github.com/stnolting/neorv32/pull/368) |
| 15.07.2022 | 1.7.4.1 | :bug: fix permanent stall of `[m]cycle[h]` and `[m]instret[h]` counter if _HPM_NUM_CNTS_ = 0; :bug: fixed bug in Wishbone `we` signal when _ASYNC_TX_ mode enabled; hardwire `dcsr.mprven` to 1 | [#367](https://github.com/stnolting/neorv32/pull/367) |
| 14.07.2022 | [**:rocket:1.7.4**](https://github.com/stnolting/neorv32/releases/tag/v1.7.4) | **New release** | |
| 14.07.2022 | 1.7.3.11 | reset all "core" CSRs to all-zero | [#366](https://github.com/stnolting/neorv32/pull/366) |
| 13.07.2022 | 1.7.3.10 | :bug: reworked/fixed **physical memory protection**; :sparkles: added `mstatus.MPRV` flag | [#365](https://github.com/stnolting/neorv32/pull/365) |
| 12.07.2022 | 1.7.3.9 | clean-up and rework **bootloader**; :sparkles: add "boot via XIP" option | [#364](https://github.com/stnolting/neorv32/pull/364) |
| 11.07.2022 | 1.7.3.8 | **physical memory protection(PMP)**: locking entry `i` in TOR mode will now also prevent write access to `pmpaddr(i-1)` (RISC-V compatibility) | [#363](https://github.com/stnolting/neorv32/pull/363) |
| 09.07.2022 | 1.7.3.7 | :bug: fixed **bootloader's** byte order when using the flash for application storage: :warning: was BIG-endian, is now also LITTLE-endian | [#362](https://github.com/stnolting/neorv32/pull/362) |
| 08.07.2022 | 1.7.3.6 | :test_tube: added burst mode option to **XIP module** to accelerate consecutive flash read accesses; :warning: fixed XIP endianness: was BIG-endian and is now LITTLE-endian | [#361](https://github.com/stnolting/neorv32/pull/361) |
| 08.07.2022 | 1.7.3.5 | Update "raw" executable generation options of makefile and image generator | [#360](https://github.com/stnolting/neorv32/pull/360) |
| 05.07.2022 | 1.7.3.4 | add "infrastructure" for cached (burst) bus accesses | [#359](https://github.com/stnolting/neorv32/pull/359) |
| 01.07.2022 | 1.7.3.3 | minor rtl cleanups | [#357](https://github.com/stnolting/neorv32/pull/357) |
| 29.06.2022 | 1.7.3.2 | :test_tube: add experimental core complex wrapper for integration into the [**LiteX**](https://github.com/enjoy-digital/litex) SoC builder framework | [#353](https://github.com/stnolting/neorv32/pull/353) |
| 28.06.2022 | 1.7.3.1 | :bug: fix bug that caused permanent CPU stall if illegal load/store instruction | [#356](https://github.com/stnolting/neorv32/pull/356) |
| 23.06.2022 | [**:rocket:1.7.3**](https://github.com/stnolting/neorv32/releases/tag/v1.7.3) | **New release** _two years NEORV32!_ :tada: | |
| 21.06.2022 | 1.7.2.10 | :sparkles: add option to implement an asynchronous **Wishbone** TX path; add new top generic `MEM_EXT_ASYNC_TX` | [#352](https://github.com/stnolting/neorv32/pull/352) |
| 17.06.2022 | 1.7.2.9 | minor rtl code clean-ups/optimization of **CPU core** and **Neoled** module | [#351](https://github.com/stnolting/neorv32/pull/351) |
| 16.06.2022 | 1.7.2.8 | :warning: rework **SLINK** module, add support for T_LAST signals | [#349](https://github.com/stnolting/neorv32/pull/349) |
| 11.06.2022 | 1.7.2.7 | reworked processor **reset system**; :warning: changed behavior of **watchdog's** "lock" bit; add watchdog "access password" | [#345](https://github.com/stnolting/neorv32/pull/345) |
| 10.06.2022 | 1.7.2.6 | **Wishbone** interface now _gates_ all outgoing signals (= signals remain stable if there is no active Wishbone access) | [#344](https://github.com/stnolting/neorv32/pull/344) |
| 09.06.2022 | 1.7.2.5 | reworked **TWI** module fixing several interface timing issues; :warning: removed "START condition done interrupt" and "STOP condition done interrupt" | [#340](https://github.com/stnolting/neorv32/pull/340) |
| 06.06.2022 | 1.7.2.4 | split executable images into package and body | [#338](https://github.com/stnolting/neorv32/pull/338) |
| 04.06.2022 | 1.7.2.3 | :bug: fixed bug in **SPI** and **XIP** modules: phase offset between SPI clock and SPI data | [#336](https://github.com/stnolting/neorv32/pull/336) |
| 03.06.2022 | 1.7.2.2 | :sparkles: (finally) added a **dedicated hardware reset** to all IO/peripheral devices | [#334](https://github.com/stnolting/neorv32/pull/334) |
| 02.06.2022 | 1.7.2.1 | :sparkles: add **watchdog** pause flag to stop watchdog timeout counter when CPU is in sleep mode | [#331](https://github.com/stnolting/neorv32/pull/331) |
| 02.06.2022 | [**:rocket:1.7.2**](https://github.com/stnolting/neorv32/releases/tag/v1.7.2) | **New release** | |
| 01.06.2022 | 1.7.1.11 | :bug: fixed bug in **debugger's** single-stepping mode (bug introduced with version 1.7.1.9) | [#329](https://github.com/stnolting/neorv32/pull/329) |
| 29.05.2022 | 1.7.1.10 | rework **bootloader's** "SPI flash presence detection"; added new option (`SPI_FLASH_ADDR_BYTES`) to customize the bootloader SPI flash address width (16-, 24- or 32-bit) | [#321](https://github.com/stnolting/neorv32/pull/321) |
| 29.05.2022 | 1.7.1.9 | :bug: fixed bug in **CPU trap logic**: collision of synchronous and asynchronous exceptions | [#327](https://github.com/stnolting/neorv32/pull/327) |
| 19.05.2022 | 1.7.1.8 | :bug: fixed bug in **XIP** address conversion logic: sub-word read accesses (half-word, byte) returned wrong data | [#320](https://github.com/stnolting/neorv32/pull/320) |
| 17.05.2022 | 1.7.1.7 | :sparkles: add optional/configurable data FIFO to **TRNG**; new top generic `IO_TRNG_FIFO` | [#316](https://github.com/stnolting/neorv32/pull/316) |
| 13.05.2022 | 1.7.1.6 | :bug: fixed bug in **BUSKEEPER** timeout logic | [#315](https://github.com/stnolting/neorv32/pull/315) |
| 10.05.2022 | 1.7.1.5 | code clean-up and minor optimization of `B` extension (bit-manipulation) CPU co-processor | [#312](https://github.com/stnolting/neorv32/pull/312) |
| 06.05.2022 | 1.7.1.4 | :sparkles: upgrade TRNG module to new [neoTRNG v2](https://github.com/stnolting/neoTRNG) | [#311](https://github.com/stnolting/neorv32/pull/311) |
| 05.05.2022 | 1.7.1.3 | :bug: bug fix in CPU counter overflow logic (`cycle` and `instret` counters); minor optimization of CPU execution unit | [#310](https://github.com/stnolting/neorv32/pull/310) |
| 28.04.2022 | 1.7.1.2 | add flag to `mxisa`  CSR to check if _this_ is a simulation (bit 20: _CSR_MXISA_IS_SIM_); add flag to `mxisa`  CSR to check if all CPU core register have a dedicated reset (bit 21: _CSR_MXISA_HW_RESET_) | [#309](https://github.com/stnolting/neorv32/pull/309) |
| 27.04.2022 | 1.7.1.1 | :warning: **removed RISC-V `A` ISA extension** (atomic memory accesses); removed Wishbone "lock" signal | [#308](https://github.com/stnolting/neorv32/pull/308) |
| 25.04.2022 | [**:rocket:1.7.1**](https://github.com/stnolting/neorv32/releases/tag/v1.7.1) | **New release** | |
| 23.04.2022 | 1.7.0.9 | :bug: fixed minor bug in HPM event logic: imprecise "taken branch" (_HPMCNT_EVENT_TBRANCH_) event |
| 23.04.2022 | 1.7.0.8 | :sparkles: add simple branch prediction (predict "always taken") to CPU front-end to reduce branch penalty (less wait cycles); [#306](https://github.com/stnolting/neorv32/pull/306) |
| 22.04.2022 | 1.7.0.7 | reworked CPU's MUL/DIV unit (`M`-extension): less area and shorter critical path; [#305](https://github.com/stnolting/neorv32/pull/305) |
| 21.04.2022 | 1.7.0.6 | further VHDL code clean-ups and minor optimizations; [#303](https://github.com/stnolting/neorv32/pull/303) |
| 19.04.2022 | 1.7.0.5 | minor clean-up and optimization of CPU's bus unit |
| 13.04.2022 | 1.7.0.4 | improve timing of CPU's barrel shifter (`FAST_SHIFT_EN` = true) by moving the register stage; [#301](https://github.com/stnolting/neorv32/pull/301) |
| 12.04.2022 | 1.7.0.3 | CPU front-end is now controlled by a _synchronous_ state machine (all outgoing signals are driven by registers), reducing critical path of memory system & reducing area costs; :warning: `CPU_IPB_ENTRIES` now has to be >= 2; [#300](https://github.com/stnolting/neorv32/pull/300) |
| 11.04.2022 | 1.7.0.2 | cleanup of CPU front-end (instruction fetch); cleaner code, less area costs; [#299](https://github.com/stnolting/neorv32/pull/299) |
| 10.04.2022 | 1.7.0.1 | rework handling of `x0` register (`zero`): shortens critical path and reduces area costs; [#298](https://github.com/stnolting/neorv32/pull/298) |
| 08.04.2022 | [**:rocket:1.7.0**](https://github.com/stnolting/neorv32/releases/tag/v1.7.0) | **New release** |
| 08.04.2022 | 1.6.9.11 | :bug: fixed bug in interrupt setup of **`crt0` start-up code** [#297](https://github.com/stnolting/neorv32/pull/297) |
| 08.04.2022 | 1.6.9.10 | rework compressed instruction (`C` ISA extension) de-compressor: :lock: closed further illegal compressed instruction holes; code clean-ups; `mtval` CSR now shows the decompressed 32-bit instruction when executing an illegal compressed instruction; minor RTL code cleanups (removing legacy stuff); [PR #296](https://github.com/stnolting/neorv32/pull/296) |
| 07.04.2022 | 1.6.9.9 | AND-gate CSR read address: reduces **CPU switching activity** (= dynamic power consumption) and even reduces area costs; [PR #295](https://github.com/stnolting/neorv32/pull/295) |
| 06.04.2022 | 1.6.9.8 | :bug: fixed instruction decoding collision in CPU `B` extension; :lock: closed further illegal instruction encoding holes; optimized illegal instruction detection logic; [PR #294](https://github.com/stnolting/neorv32/pull/294) |
| 04.04.2022 | 1.6.9.7 | **major CPU logic optimization**: reduced area costs and shortened critical path (higher f_max!); :bug: fixed rare bug in RTE core (if C-extension is not implemented); :lock: closed further illegal instruction encoding holes; [PR #293](https://github.com/stnolting/neorv32/pull/293) |
| 01.04.2022 | 1.6.9.6 | rework **CPU front-end**: instruction issue engine; much cleaner code, slightly less HW required; [PR #292](https://github.com/stnolting/neorv32/pull/292) |
| 29.03.2022 | 1.6.9.5 | minor clock generator edits: reset **clock generator** explicitly if not being used by _any_ peripheral/IO device |
| 19.03.2022 | 1.6.9.4 | :test_tube: change usage of VHDL `*_reduce_f` functions for signals that might effect gate-level simulations; [PR #290](https://github.com/stnolting/neorv32/pull/290) |
| 19.03.2022 | 1.6.9.3 | :bug: fixed minor bug in **FPU** - incorrect/missing reset (even if reset to `'-'`) of some registers |
| 18.03.2022 | 1.6.9.2 | fixed minor bug in **TRNG** interface hand shake (that marked the _same_ RND value as "valid" for several times); minor optimization of **processor's reset generator** |
| 14.03.2022 | 1.6.9.1 | `mtval` CSR is set to zero for software breakpoints (`[c.]ebreak` instruction(s)) - this is permitted by the RISC-V machine ISA spec. v1.12; [PR #289](https://github.com/stnolting/neorv32/pull/289) |
| 09.03.2022 | [**:rocket:1.6.9**](https://github.com/stnolting/neorv32/releases/tag/v1.6.9) | **New release** |
| 09.03.2022 | 1.6.8.12 | CPU core: minor code clean-up |
| 08.03.2022 | 1.6.8.11 | clean-up of CPU's privilege mode logic |
| 07.03.2022 | 1.6.8.10 | added compressed floating-point instructions (`Zfinx` ISA extensions); minor optimization of compressed instruction decoding logic |
| 05.03.2022 | 1.6.8.9 | CPU core: minor optimizations, code clean-ups and edits; :sparkles: added RISC-V `mstatus.TW` bit to allow/disallow execution of `wfi` instruction in user mode; [PR #285](https://github.com/stnolting/neorv32/pull/285) |
| 02.03.2022 | 1.6.8.8 | :bug: fixed bug in layout of CPU's `pmpaddr` CSRs (**physical memory protection**); [PR #283](https://github.com/stnolting/neorv32/pull/283) |
| 01.03.2022 | 1.6.8.7 | CPU core: minor optimizations, code clean-ups and edits |
| 26.02.2022 | 1.6.8.6 | :warning: :lock: **reworked Physical Memory Protection (PMP)**: replacing `NAPOT` mode by `TOR` mode and fixing several minor PMP CSR-access bugs; maximum number of PMP regions is now limited to 16 entries; :warning: removed **BUSKEEPER's NULL address check** (introduced in version `1.6.5.4`) - use a single PMP entry instead; see [PR #281](https://github.com/stnolting/neorv32/pull/281) |
| 25.02.2022 | 1.6.8.5 | minor BUSMUX (bus multiplexer for CPU's instruction and data buses) and CPU control edits (pipeline front-end) |
| 24.02.2022 | 1.6.8.4 | :bug: **fixed bug in `mip` CSR** (introduced in version `1.6.4.6` with [#236](https://github.com/stnolting/neorv32/pull/236)): to clear/ack a pending interrupt software needs to **clear** the according `mip` bit; see [PR #280](https://github.com/stnolting/neorv32/pull/280) |
| 24.02.2022 | 1.6.8.3 | reworked CPU's data path (use a few _wide_ multiplexers instead of many small ones); [PR #279](https://github.com/stnolting/neorv32/pull/279) |
| 23.02.2022 | 1.6.8.2 | CPU logic optimizations (less area): simplified CPU co-processor interface; minor optimization of bus unit access arbiters; optimized `M` extension's (mul/div co-processor) divider unit |
| 18.02.2022 | 1.6.8.1 | minor CPU control logic optimizations: simplified execute engine; faster execution of SYSTEM instructions (one cycle less) |
| 17.02.2022 | [**:rocket:1.6.8**](https://github.com/stnolting/neorv32/releases/tag/v1.6.8) | **New release** |
| 17.02.2022 | 1.6.7.10 | hardwired `dcsr.stopcount` to `1`: all standard counters (`[m]cycle[h]` and `[m]instret[h]`, but **NOT** `[m]time[h]`!!) and all hardware performance monitor (HPM) counters are _stopped_ when the CPU is in debug mode; [PR #277](https://github.com/stnolting/neorv32/pull/277) |
| 16.02.2022 | 1.6.7.9 | :warning: **added custom `mxisa` CSR replacing SYSINFO's `NEORV32_SYSINFO.CPU` memory-mapped register**: bit-positions remain but names and the actual access mechanism (CSR vs. memory-mapped) have changed! see [PR #276](https://github.com/stnolting/neorv32/pull/276) |
| 11.02.2022 | 1.6.7.8 | :test_tube: added newlib's system calls (stubs) and linker script symbols for heap memory to support **dynamic memory allocation**  (e.g. `malloc`) and even **standard IO functions** like `printf`; see [PR #275](https://github.com/stnolting/neorv32/pull/275) |
| 10.02.2022 | 1.6.7.7 | :test_tube: added **RISC-V hardware trigger module** to CPU - allows to set _hardware breakpoints_ (via gdb's `hb`/`hbreak` command) to debug code from ROM; see [PR #274](https://github.com/stnolting/neorv32/pull/274); :bug: minor bug fix in `ebreak` instruction's `dcsr.cause` value (was 0b010 but has to be 0b001) |
| 08.02.2022 | 1.6.7.6 | :warning: renamed default branch of repository to `main` |
| 07.02.2022 | 1.6.7.5 | removed default values for bi-directional top entity ports `twi_sda_io` and `twi_scl_io` |
| 05.02.2022 | 1.6.7.4 | added `err_o` signal to **IMEM** module; if the IMEM is implemented as true ROM any write attempt will raise a _store access fault_ exception (with a `[DEVICE_ERR]` error); see [PR #273](https://github.com/stnolting/neorv32/pull/273) |
| 03.02.2022 | 1.6.7.3 | :test_tube: using `LTO` (link-time-optimization) option for **bootloader**; improved bootloader user console; see [PR #268](https://github.com/stnolting/neorv32/pull/268) |
| 31.01.2022 | 1.6.7.2 | :bug: fixed minor bug in **bootloader's MTIME handling** (bootloader crashed if `Zicntr` ISA extension not enabled), fixed minor issues in MTIME and `time` CSRs handling; added MTIME example program; see [PR #267](https://github.com/stnolting/neorv32/pull/267) |
| 30.01.2022 | 1.6.7.1 | :sparkles: added **`Zxcfu` ISA extension for user-defined custom RISC-V instructions**; see [PR #264](https://github.com/stnolting/neorv32/pull/264) |
| 28.01.2022 | [**:rocket:1.6.7**](https://github.com/stnolting/neorv32/releases/tag/v1.6.7) | **New release** |
| 28.01.2022 | 1.6.6.10 | :bug: fixed bug in **bit-manipulation co-processor**: decoding collision between `cpop` and `rol` instructions; :bug: fixed bug in co-processor arbitration when an illegal instruction is detected; added four additional (yet unused) **CPU** co-processor slots; [PR #262](https://github.com/stnolting/neorv32/pull/262) |
| 27.01.2022 | 1.6.6.9 | reworked **CFS** "user" logic; added CFS demo program; see [PR #261](https://github.com/stnolting/neorv32/pull/261) |
| 27.01.2022 | 1.6.6.8 | :sparkles: added support for RISC-V bit-manipulation (`B`) **carry-less multiplication instructions `Zbc`** sub-extension; added test cases and intrinsics; the NEORV32 bit-manipulation ISA extension (`B`) now fully complies to the RISC-V specs. v0.93; see [PR #260](https://github.com/stnolting/neorv32/pull/260) |
| 26.01.2022 | 1.6.6.7 | :sparkles: added support for RISC-V bit-manipulation (`B`) **single-bit instructions `Zbs`** sub-extension; added test cases and intrinsics; see [PR #259](https://github.com/stnolting/neorv32/pull/259) |
| 26.01.2022 | 1.6.6.6 | minor logic optimizations in **CPU control unit** |
| 25.01.2022 | 1.6.6.5 | :lock: **on-chip debugger:** the memory-mapped registers of the debug module (DM) are only accessible/visible when the CPU is actually in debug mode; any access outside of debug mode will now raise a bus exception |
| 22.01.2022 | 1.6.6.4 | minor logic optimizations in **CPU control unit**, minor improvement of critical path |
| 21.01.2022 | 1.6.6.3 | reworked **CPU's instruction issue engine** (area optimization: ~100 LUTs less on an Intel Cyclone IV), [PR #256](https://github.com/stnolting/neorv32/pull/256); minor CPU control unit code clean-ups and logic optimizations |
| 18.01.2022 | 1.6.6.2 | :warning: moved `setups` folder to new [neorv32-setups](https://github.com/stnolting/neorv32-setups) repository, [PR #254](https://github.com/stnolting/neorv32/pull/254) |
| 18.01.2022 | 1.6.6.1 | minor **MTIME** VHDL code clean-up; minor logic optimization of **CPU's bus unit** |
| 17.01.2022 | [**:rocket:1.6.6**](https://github.com/stnolting/neorv32/releases/tag/v1.6.6) | **New release** |
| 14.01.2022 | 1.6.5.9 | **GPIO** module: write accesses to the GPIO module's "input" registers will now raise a bus exception; [PR #255](https://github.com/stnolting/neorv32/pull/255) |
| 11.01.2022 | 1.6.5.8 | minor rtl code clean-ups and edits in `rtl/core`; any write access to the SYSINFO module will now show up as a BUSKEEPER's "DEVICE_ERR" |
| 08.01.2022 | 1.6.5.7 | :bug: fixed bug in BUSKEEPER's error type logic (introduced in version `1.6.5.4`); removed "unexpected ERR/ACK" error codes; [PR #253](https://github.com/stnolting/neorv32/pull/253) |
| 07.01.2022 | 1.6.5.6 | :sparkles: **XIP & SPI: added high-speed SPI mode** (SPI clocking at half of the processor clock), see [PR #251](https://github.com/stnolting/neorv32/pull/251) |
| 06.01.2022 | 1.6.5.5 | :warning: optimized/reworked XIP (execute in place) module, see [PR #249](https://github.com/stnolting/neorv32/pull/249) |
| 04.01.2022 | 1.6.5.4 | **BUSKEEPER** can now optionally check for NULL address accesses (address `0x00000000`), see [PR #247](https://github.com/stnolting/neorv32/pull/247) |
| 02.01.2022 | 1.6.5.3 | :sparkles: **added Execute In Place (XIP) module** allowing code to be directly executed from an external SPI flash, see [PR #244](https://github.com/stnolting/neorv32/pull/244) |
| 02.01.2022 | 1.6.5.2 | :bug: fixed minor bug in CPU's instruction fetch unit (only issue new instruction fetch request when the previous one has been completed) |
| 16.12.2021 | [**:rocket:1.6.5**](https://github.com/stnolting/neorv32/releases/tag/v1.6.5) | **New release** |
| 15.12.2021 | 1.6.4.10 | minor logic optimization of CPU's pipeline front-end (instruction fetch and instruction issue) |
| 14.12.2021 | 1.6.4.9 | optimized CPU's multiplication/division co-processor: divisions are 1 cycle faster, fast-multiplications (when using DSPs) are 1 cycle faster, slightly less resource utilization, see [PR #240](https://github.com/stnolting/neorv32/pull/240) |
| 11.12.2021 | 1.6.4.8 | watchdog: added new _DBEN_ and _HALF_ flags to control register (enable WDT during debugging, check timeout counter level), see [PR #239](https://github.com/stnolting/neorv32/pull/239) |
| 10.12.2021 | 1.6.4.7 | optimized CPU's multiplication/division co-processor: all mul/div operations are 1 cycle faster + slightly less resource utilization, see [PR #238](https://github.com/stnolting/neorv32/pull/238) |
| 08.12.2021 | 1.6.4.6 | :warning: reworked **Fast Interrupt Requests (FIRQ)** system, see [PR #236](https://github.com/stnolting/neorv32/pull/236) |
| 03.12.2021 | 1.6.4.5 | added _SYSINFO_SOC_IS_SIM_ flag to SYSINFO to check if processor is being simulated (not guaranteed, depends on the toolchain's 'pragma' support), see [PR #231](https://github.com/stnolting/neorv32/pull/231) |
| 03.12.2021 | 1.6.4.4 | :bug: fixed bug in **Wishbone** bus interface: timeout configurations (via `MEM_EXT_TIMEOUT` generic) that are a power of two (e.g. 256) caused _immediate_ timeouts; timeout counter was one bit short; same problem for processor-internal bus monitor (BUSKEEPER); see [PR #230](https://github.com/stnolting/neorv32/pull/230) |
| 02.12.2021 | 1.6.4.3 | :warning: removed legacy software compatibility wrappers (`sw/lib/include/neorv32_legacy.h` and `neorv32_uart_*` functions) |
| 28.11.2021 | 1.6.4.2 | :bug: fixed bug in **UART[0/1]** overrun flag (was not set/cleared correctly); fixed bug in UART0 enable function `neorv32_uart0_enable()` |
| 28.11.2021 | 1.6.4.1 | (:warning:) bootloader now stores executable in _little-endian_ byte-order to SPI flash |
| 26.11.2021 | [**:rocket:1.6.4**](https://github.com/stnolting/neorv32/releases/tag/v1.6.4) | **New release** |
| 22.11.2021 | 1.6.3.11 | on-chip debugger: reworked JTAG signal input/output synchronization logic (see [PR #216](https://github.com/stnolting/neorv32/pull/216)) |
| 22.11.2021 | 1.6.3.10 | reworked **TRNG** (less hardware requirements, improved quality), see [PR #212](https://github.com/stnolting/neorv32/pull/212) and [stnolting/neoTRNG](https://github.com/stnolting/neoTRNG) |
| 21.11.2021 | 1.6.3.9 | minor rtl edits: configuring an IMEM or DMEM size (`MEM_INT_IMEM_SIZE` / `MEM_INT_DMEM_SIZE` generic) of 0 will now exclude the according memory from synthesis (and also clears the according `NEORV32_SYSINFO.SOC` flags) |
| 18.11.2021 | 1.6.3.8 | TWI: removed TWI_CTRL_CKSTEN flag (enable clock stretching) from control registers, clock-stretching is now _always_ enabled |
| 14.11.2021 | 1.6.3.7 | major control unit and ALU logic optimizations, reduced hardware footprint; :lock: closed further illegal instruction encoding holes (system environment instructions, ALU and ALU-immediate instructions, FENCE instructions); [PR #204](https://github.com/stnolting/neorv32/pull/204) |
| 10.11.2021 | 1.6.3.6 | optimized BUSKEEPER: removed redundant logic - bus keeper now also shows an external interface access timeout (if implemented) as "timeout error"; removed _BUSKEEPER_ERR_SRC_ status flag; :warning: added `err_o` (fault access operation) to the custom functions subsystem (CFS) |
| 09.11.2021 | 1.6.3.5 | :warning: reworked IRQ trigger logic of SPI, TWI, UART0, UART1, NELOED and SLINK; FIRQs now only trigger **once** when the programmed interrupt condition is met instead of triggering **all the time** (see [PR #202](https://github.com/stnolting/neorv32/pull/202)) |
| 06.11.2021 | 1.6.3.4 | :bug: fixed bug in **WISHBONE** interface: _pipelined_ Wishbone mode did not clear STB after first transfer cycle |
| 05.11.2021 | 1.6.3.3 | :bug: fixed bug in general purpose timer **GPTMR** - clock prescaler had no effect, the timer was always counting at full processor clock speed; minor watchdog (WDT) code edits |
| 04.11.2021 | 1.6.3.2 | added optional _alternative_ IMEM and DMEM architecture-only design files (in `rtl/core/mem`); these are not device-specific ("cyclone 2") as they do not use any FPGA-specific primitives or macros - just a different HDL style for describing memories is used (see [PR #192](https://github.com/stnolting/neorv32/pull/198) and [Issue #197](https://github.com/stnolting/neorv32/issues/197)) |
| 03.11.2021 | 1.6.3.1 | :sparkles: added new peripheral module - **General Purpose 32-bit Timer `GPTMR`** ([see PR #195](https://github.com/stnolting/neorv32/pull/195)) |
| 02.11.2021 | [**:rocket:1.6.3**](https://github.com/stnolting/neorv32/releases/tag/v1.6.3) | **New release** |
| 01.11.2021 | 1.6.2.13 | added new top generics to explicitly control implementation of `Zicntr` (CPU base counters) and `Zihpm` (hardware performance monitors, see [PR #192](https://github.com/stnolting/neorv32/pull/192) |
| 30.10.2021 | 1.6.2.12 | :sparkles: :lock: added memory-mapped register to BUSKEEPER module - software can now retrieve the actual cause of an instruction / data-load / data-store bus access fault exception (access timeout or device error); see [PR #191](https://github.com/stnolting/neorv32/pull/191) |
| 28.10.2021 | 1.6.2.11 | :sparkles: added `Zba` bit-manipulation sub-extension; :warning: removed configuration option for `B` sub-extensions: removed `CPU_EXTENSION_RISCV_Zbb` generic and according SYSINFO flag, added new `CPU_EXTENSION_RISCV_B` generic (to implement bit-manipulation `B` ISA extension with _all_ currently supported subsets), see [PR #190](https://github.com/stnolting/neorv32/pull/190) |
| 27.10.2021 | 1.6.2.10 | :bug: CPU control unit: fixed _imprecise_ illegal instruction exceptions - `MEPC` and `MTAVL` did not reflect the correct exception-causing data for illegal ALU-class (non-multi-cycle like `SUB`) operations; optimized critical path of exception logic (illegal compressed instruction detection) |
| 27.10.2021 | 1.6.2.9 | CPU control unit: minor logic optimization - `fence.i` instruction needs 1 cycle less to execute, reduced HW footprint of control engine, shortened CPU's critical path (PC update logic) |
| 26.10.2021 | 1.6.2.8 | :bug: bootloader: fixed bug in stack pointer initialization (introduced in version `1.6.2.7`); minor SPI unit VHDL code clean-up |
| 24.10.2021 | 1.6.2.7 | minor control unit fixes (add logic to check both half-words of a unaligned 32-bit instruction did not cause any bus exceptions); minor ALU logic optimization; optimized `ctr0.S`: bootloader stack pointer initialization (is now done based on the actual physical memory configuration) - bootloader is now even more independent of the actual platform configuration |
| 24.10.2021 | 1.6.2.6 | :bug: **fixed HW bug** introduced in version `1.6.2.4` (write access arbitration in BUSMUX) |
| 21.10.2021 | 1.6.2.5 | minor code edits; improved stability of UART receiver's start-bit detection (more "spike"-resistant) |
| 21.10.2021 | 1.6.2.4 | minor VHDL code fixes, clean-ups, optimizations and comment typo fixes (:lipstick:) |
| 20.10.2021 | 1.6.2.3 | SPI: minor VHDL code optimization and clean-up; NOTE: all serial interfaces (SPI, TWI, UARTs) allow to terminate a running transmission by clearing the enable flag in the module's control register |
| 18.10.2021 | 1.6.2.2 | :bug: `*_reduce_f` VHDL functions did not work for single-bit operands (see [PR #186](https://github.com/stnolting/neorv32/pull/186)) |
| 18.10.2021 | 1.6.2.1 | :sparkles: SPI: added option to configure _clock polarity_ - the SPI module now supports all standard clock modes (0,1,2,3) (see [PR #185](https://github.com/stnolting/neorv32/pull/185)); logic optimization of SPI module |
| 17.10.2021 | [**:rocket:1.6.2**](https://github.com/stnolting/neorv32/releases/tag/v1.6.2) | **New release** |
| 17.10.2021 | 1.6.1.13 | :warning: :warning: main software makefile: modified behavior of `MARCH` and `MABI` variables - the `-march` and `-mabi` flags are no longer required/allowed (example: overriding makefile's default `MARCH` is now done using `make MARCH=rv32imac ...`) ([see PR #184](https://github.com/stnolting/neorv32/pull/184)) |
| 15.10.2021 | 1.6.1.12 | :warning: Custom Functions Subsystem (CFS): removed `sleep` input (indicating CPU is in sleep mode); minor CPU control logic optimization |
| 15.10.2021 | 1.6.1.11 | :sparkles: UARTs: added optional configurable RX and TX FIFOs, added fine-grained RX/TX IRQ configuration options (see [PR #183](https://github.com/stnolting/neorv32/pull/183)) |
| 14.10.2021 | 1.6.1.10 | :sparkles: SLINK: added fine-grained, per-link interrupt configuration (see [PR #182](https://github.com/stnolting/neorv32/pull/182)) |
| 13.10.2021 | 1.6.1.9 | :sparkles: NEOLED: added new control register bit _NEOLED_CTRL_IRQ_CONF_ to configure IRQ condition: `0` = IRQ if FIFO is less than half-full, `1` = IRQ if FIFO is empty; :information_source: IRQ behavior is fully backwards compatible if _NEOLED_CTRL_IRQ_CONF_ is ignored (kept zero) |
| 12.10.2021 | 1.6.1.8 | added dedicated `half_o` signal to FIFO component (FIFO _at least_ half-full), simplifies half-full test logic in FIFO-utilizing modules (area footprint and critical path); minor logic/hardware optimization of NEOLED module |
| 09.10.2021 | 1.6.1.7 | :warning: reworked _fast interrupt requests_ (FIRQ) CPU interrupt system: fast interrupt requests are now also high-level-triggered (like the RISC-V standard interrupts) and stay asserted until explicitly acknowledged by software ([PR #176](https://github.com/stnolting/neorv32/pull/176)) |
| 06.10.2021 | 1.6.1.6 | :bug: fixed bugs in signal assignments and processor configuration of `setups/radiant/UPduino_v3` setup; minor CPU HPM counter fix (architecture condition for "multi-cycle ALU wait cycle" HPM event) |
| 05.10.2021 | 1.6.1.5 | :sparkles: :lock: the CPU now ensures that _all_ illegal instructions _do not commit_ any potential architecture state changes (like writing registers or triggering memory accesses); CPU logic optimization (smaller footprint) |
| 04.10.2021 | 1.6.1.4 | moved CPU's comparator logic from register file unit to ALU unit (to allow easier replacement of register file design unit by technology-optimized one) |
| 03.10.2021 | 1.6.1.3 | :bug: fixed UART signal connection in `rtl/system_integration` wrappers |
| 01.10.2021 | 1.6.1.2 | :warning: removed `mstatus.TW` (timeout wait) bit, `wfi` instruction is now always allowed to be executed in less-privileged modes; minor CPU control unit logic optimizations |
| 01.10.2021 | 1.6.1.1 | on-chip-debugger: `wfi` instruction acts as a simple `nop` when _in_ debug mode or during single-stepping |
| 28.09.2021 | [**:rocket:1.6.1**](https://github.com/stnolting/neorv32/releases/tag/v1.6.1) | **New release** |
| 28.09.2021 | 1.6.0.13 | :bug: fixed elementary bug in MTIME comparator logic (interrupt condition `mtime >= mtimecmp` was not always evaluated correctly) |
| 28.09.2021 | 1.6.0.12 | fixed CPU's IRQ prioritization: (re-)enter debug mode interrupts have to be evaluated _before_ all other interrupts |
| 27.09.2021 | 1.6.0.11 | :warning: `Zifencei` extension is _required_ for the on-chip debugger; executing `fence.i` without having the `Zifencei` extension enabled will now cause an illegal instruction exception |
| 22.09.2021 | 1.6.0.10 | reworked CPU/software handshake of external interrupt controller `XIRQ` to avoid "external IRQ -> CPU IRQ" race conditions |
| 22.09.2021 | 1.6.0.9 | if `CPU_CNT_WIDTH` generic (actual width of `[m]cycle` and `[m]instret` counters) is less than 64 the remaining bits are now just hardwired to zero ignoring any write access instead of causing an exception; minor CPU hardware optimizations |
| 22.09.2021 | 1.6.0.8 | :bug: fixed bug introduced in previous version: misaligned instruction address - PC and all instruction address-related registers need to have bit 0 hardwired to zero, misaligned instructions can only appear if NOT using `C` ISA extension |
| 21.09.2021 | 1.6.0.7 | :warning: **reworked CPU trap/exception system** (in order to comply with RISC-V specs.): removed non-maskable interrupt (`NMI`, top signal `nm_irq_i`); reworked CPU trap prioritization (sync before async); RISC-V interrupts (`MTI`, `MSI`, `MEI`) are now high-level-triggered and require to stay asserted until they are explicitly acknowledged; fixed minor bug in misaligned instruction check logic (PC(0) = '1' will always cause a misalignment exception); updated trap/interrupt-related documentation |
| 20.09.2021 | 1.6.0.6 | the NEORV32's `misa`, `mip` and `mtval` CSRs are _read-only_; however, write accesses to these CSRs _do not raise an illegal instruction exception_ (anymore) to be compatible to the RISC-V specs. |
| 19.09.2021 | 1.6.0.5 | added `menvcfg[h]` CSRs (only available if `U` ISA extension is enabled; not used yet - hardwired to zero, but required by RISC-V spec.) |
| 18.09.2021 | 1.6.0.4 | :warning: :warning: **major change** modified low-level hardware access (memory-mapped registers) [PR #158](https://github.com/stnolting/neorv32/pull/158): now using `struct`-based access concept (IO module = `struct`, interface registers = members of struct) instead of `#define` single-pointers (inspired by https://blog.feabhas.com/2019/01/peripheral-register-access-using-c-structs-part-1/), format: `NEORV32_<module_name>.<register_name>`; renamed all control registers and bits from `*CT*` to `*CTRL*`; added `sw/lib/include/neorv32_legacy.h` compatibility layer (maps deprecated "defines" to according struct registers, provides old control register/bit names, _do not use for new designs!_) |
| 16.09.2021 | 1.6.0.3 | :bug: fixed another missing IRQ signal connection (NMI) in `system_integration` wrappers |
| 15.09.2021 | 1.6.0.2 | :warning: **split** processor-internal memory VHDL sources (IMEM and DMEM) into separated files ([#151](https://github.com/stnolting/neorv32/pull/151)): entity-only (`rtl/core/neorv32_*mem.entity.vhd`) and _default_ architecture-only (`rtl/core/mem/neorv32_*mem.default.vhd`); allows easy replacement by optimized platform-specific architectures |
| 13.09.2021 | 1.6.0.1 | :bug: fixed missing IRQ signal assignments (MSW and XIRQ) in AXI4-lite top wrapper |
| 11.09.2021 | [**:rocket:1.6.0**](https://github.com/stnolting/neorv32/releases/tag/v1.6.0) | **New release** |
| 11.09.2021 | 1.5.9.9 | removed `mstatus.SD` flag (is always 0 for `Zfinx` extension as the current state is already defined entirely by the `x` register file); tied `mstatus.fs` as it must not affect trapping of `Zfinx` instructions (according to RISC-V specs.) |
| 09.09.2021 | 1.5.9.8 | added flags to `SYSINFO` module to determine configuration of `FAST_MUL_EN` and `FAST_SHIFT_EN` generics by software |
| 09.09.2021 | 1.5.9.7 | `FAST_SHIFT_EN` option will now also implement full-parallel computation logic (like barrel shifters) for _all_ `Zbb` shift-related instructions (population count, count leading/trailing zeros, circular shifts) |
| 08.09.2021 | 1.5.9.6 | :sparkles: added support for RISC-V `Zbb` CPU extension (**basic bit-manipulation operations**), enabled via new top generic `CPU_EXTENSION_RISCV_Zbb`; added example software project providing a `Zbb` "intrinsic" library |
| 08.09.2021 | 1.5.9.5 | :bug: fixed missing `flash_sdi_i` in Radiant-related example setups and processor wrappers |
| 19.08.2021 | 1.5.9.4 | :warning: removed custom `mzext` CPU CSR, moved all information flags to new `SYSINFO_CPU` register in the system information memory module (`SYSINFO`) |
| 19.08.2021 | 1.5.9.3 | :warning: removed top's `USER_CODE` generic |
| 18.08.2021 | 1.5.9.2 | fixed `Zifencei` test of `riscv-arch-test` port |
| 16.08.2021 | 1.5.9.2 | minor CPU control logic optimizations |
| 15.08.2021 | 1.5.9.1 | :bug: fixed bug in `mret` instruction that caused an exception if user mode was not implemented (bug caused by modifications in v1.5.8.8) |
| 14.08.2021 | 1.5.9.0 | Added new designated test setups: [`rtl/test_setups`](https://github.com/stnolting/neorv32/tree/main/rtl/test_setups), :books: [_UG: General Hardware Setup_](https://stnolting.github.io/neorv32/ug/#_general_hardware_setup) |
| 13.08.2021 | [**:rocket:1.5.9**](https://github.com/stnolting/neorv32/releases/tag/v1.5.9) | **New release** |
| 08.08.2021 | 1.5.8.9 | reworked CPU register file logic: any write access to `x0` will be masked to actually write zero - no special treatment by the CPU control unit required anymore; slightly less hardware resources required; first instruction after hardware reset should write `x0` (_any_ value; implemented in start-up code `crt0.S`) |
| 07.08.2021 | 1.5.8.8 | :bug: fixed bug in execution (trapping) of `xRET` instructions: `dret` (return from debug-mode handler) has to raise an illegal instruction exception if executed outside of debug-mode, `mret` (return from machine-mode handler) has to raise an illegal instruction exception if executed in lower-privileged modes (lower than machine-mode) |
| 05.08.2021 | 1.5.8.7 | :sparkles: added `mstatus.FS` and `mstatus.SD` CSR bits: control the state of the FPU (`Zfinx`) extension; supported states for `mstatus.FS`: `00` = _off_, `11` = _dirty_; writing other states will always set _dirty_ state; note that all FPU instructions including FPU CSR access instructions will raise an illegal instruction exception if `mstatus.FS` = _off_ |
| 03.08.2021 | 1.5.8.6 | :bug: fixed bug in linker script [#134](https://github.com/stnolting/neorv32/issues/134): `.rodata.*` "sub"-sections were missing, caused wrong linking of implicit constants (like strings); added `mconfigptr` CSR (RISC-V priv. ISA spec. v1.12-draft ;read-only): holds a pointer to a platform/system configuration structure - not actually used yet |
| 30.07.2021 | 1.5.8.5 | fixed minor bug in top entity / AXI4 wrapper (Vivado "issue": generic defaults need a _fixed-size_ initialization value) [#113](https://github.com/stnolting/neorv32/issues/133) |
| 26.07.2021 | 1.5.8.4 | :bug: **fixed major bug in CPU interrupt system**: interrupts during memory accesses (load/store instruction) terminated those memory accesses violating the crucial "instruction atomicity" concept: traps (interrupts and exceptions) must only intervene _between_ instructions |
| 25.07.2021 | 1.5.8.3 | :sparkles: added `mstauts.TW` CSR flag (when set executing `wfi` instruction outside of machine-mode will raise an illegal instruction exception); flag is hardwired to zero if user mode is not implemented |
| 25.07.2021 | 1.5.8.2 | :bug: fixed bug in `E` ISA extension: extension could not be enabled due to missing generic propagation; clean-up of generic defaults: only the processor top entity provides defaults for the configuration generics |
| 24.07.2021 | 1.5.8.1 | machine-level interrupts (top entity signals; "external" `mext_irq_i`, "software" `msw_irq_i`, "mtime" `mtime_irq_i` and "non-maskable" `nm_irq_i`) now trigger on rising edges; exposed advanced external bus interface configuration options as new top entity generics (moved from package constants): `MEM_EXT_PIPE_MODE`, `MEM_EXT_BIG_ENDIAN`, `MEM_EXT_ASYNC_RX` |
| 22.07.2021 | [**:rocket:1.5.8**](https://github.com/stnolting/neorv32/releases/tag/v1.5.8) | **New release** |
| 22.07.2021 | 1.5.7.16 | (re-)added `mstatush` CSR (all bits are hardwired to zero: writes are ignored, reads will always return zero) - CSR address is assigned to comply with RISC-V priv. arch. spec. 1.12 |
| 21.07.2021 | 1.5.7.15 | :bug: fixed minor bug in SLINK module (signals were missing in sensitivity lists); :warning: simplified NEOLED interrupt system (now triggered if TX FIFO fill level falls below half-full), added option to send LED strobe command ("RESET"), added FIFO status signals to status register, simplified FIFO access logic, added new top generic `IO_NEOLED_TX_FIFO` to configure NEOLED FIFO depth |
| 18.07.2021 | 1.5.7.14 | exposed new generic `CPU_IPB_ENTRIES` to configure size of CPU instruction prefetch buffer |
| 18.07.2021 | 1.5.7.13 | clean-up of processor top entity: using more sophisticated default values for all input signals and generics (all generics are "off" by default; input signals use `L` for control lines and `U` for data lines by default) |
| 14.07.2021 | 1.5.7.12 | reworked SLINK interrupt concept (now using FIFO fill level "half-full" as interrupt condition, see [#122](https://github.com/stnolting/neorv32/issues/122)); added fill level output to processor FIFO component |
| 09.07.2021 | 1.5.7.11 | :bug: fixed minor bug in FIFO component (mapping might fail if `FIFO_DEPTH` = 1); fixed broken `sw/example/demo_freeRTOS` makefile (all freeRTOS includes were missing) |
| 03.07.2021 | 1.5.7.10 | :sparkles: added new component: **External Interrupt Controller (XIRQ)**: up to 32 external interrupt channels `xirq_i` (via `XIRQ_NUM_CH` generic), configurable trigger (via `XIRQ_TRIGGER_TYPE` and `XIRQ_TRIGGER_POLARITY` generics), prioritized or non-prioritized servicing |
| 02.07.2021 | 1.5.7.9 | relocated base addresses of watchdog timer (WDT) and true-random number generator (TRNG); removed CPU's `firq_ack_o` signal (was not used at all) |
| 30.06.2021 | 1.5.7.8 | :warning: increased GPIO port size from 32-bit to 64-bit; relocated GPIO base address; removed GPIO.input pin-change interrupt |
| 29.06.2021 | 1.5.7.7 | :sparkles: added new processor module **stream link interface (SLINK)**: up to 8 individual RX and TX stream links, compatible to AXI4-Stream base protocol; added software driver files; added documentation |
| 27.06.2021 | 1.5.7.6 | :bug: fixed bug in CFS (custom functions subsystem) address map layout |
| 27.06.2021 | 1.5.7.5 | :warning: removed numerically-controlled oscillator (NCO, `neorv32_nco.vhd`) module as it appears to be an over-engineered clock-generator without many use cases (if you really need this module, you can wrap it within the custom functions subsystem CFS) |
| 27.06.2021 | 1.5.7.4 | :warning: removed top's fast IRQ (FIRQ) inputs `soc_firq_i`: the FIRQs are reserved for processor-internal usage only, use the `mext_irq_i` RISC-V external interrupt signal for all external interrupt applications (via dedicated interrupt controller), a follow-up version of the project will introduce a customizable external interrupt controller; sourced-out FIFOs into new HDL component `neorv32_fifo.vhd` |
| 26.06.2021 | 1.5.7.3 | edit of v1.5.7.2: RISC-V spec claims to leave destination registers of trapping load operation unchanged (do _not_ set to zero); minor CPU control logic optimizations; :sparkles: reworked bootloader to provide several new configuration and customization options |
| 25.06.2021 | 1.5.7.2 | optimized instruction execution FSM: less hardware utilization, :lock: now _ensures_ to write ZERO to destination register if there is an exception during a load operation; made default bootloader even more HW configuration independent (GPIO, SPI and MTIME are optional; UART is optional but highly recommended); |
| 24.06.2021 | 1.5.7.1 | :sparkles: added RISC-V `Zmmul` ISA extension (via `CPU_EXTENSION_RISCV_Zmmul` generic; default = _false_): implements only the integer multiplication instructions sub-set of the `M` extension; for size-constrained setups, requires ~50% less hardware resources than the `M` extension |
| 23.06.2021 | [**:rocket:1.5.7**](https://github.com/stnolting/neorv32/releases/tag/v1.5.7) | **New release** _one year NEORV32!_ :tada: |
| 21.06.2021 | 1.5.6.14 | :bug: fixed bug in debugger "park loop": `fence.i` instruction was missing before executing the DM's program buffer - this caused execution of outdated instructions from the program buffer if the **instruction cache** is implemented |
| 21.06.2021 | 1.5.6.13 | removed `TINY_SHIFT_EN` generic; clean-up of CPU co-processor system: removed "dummy co-processor" for CSR read access, moved CPU shifter core into new co-processor; simplified default (bit-serial) shifter logic (single bit-shifts only) and multi-cycl instructions decode logic |
| 18.06.2021 | 1.5.6.12 | clean-up of CPU co-processor system (removed unused co-processor slots 4,5,6,7) |
| 15.06.2021 | 1.5.6.11 | made bootloader more configuration-independent: bootloader now only uses the first 512 bytes of internal/external DMEM for runtime data - hence, the DMEM size is not further relevant as long as it greater than or equal to 512 bytes |
| 14.06.2021 | 1.5.6.10 | :sparkles: physical size of bootloader ROM (BOOTROM) is automatically determined during synthesis based on the size of the initialization image, max physical size is 32kB; simplified BOOTROM access check logic; added size check when using IMEM as ROM (check if application image fits); simplified linker script: _logical_ instruction address space 2GB now, no need to adapt this to hardware configuration, hardware checks if application fits into _physical_ memory size (which configured via generics) |
| 13.06.2021 | 1.5.6.9 | :warning: reworked boot configuration: removed `MEM_INT_IMEM_ROM` and `BOOTLOADER_EN` generics, replaced by single `INT_BOOTLOADER_EN` generic (type boolean): _true_ = implement processor-internal (default) bootloader, implement processor-internal IMEM (if implemented) as RAM; _false_ = boot from processor-internal IMEM implemented (if enabled) as pre-intialized ROM; reworked IMEM, DMEM and BOOTROM memory architecture; reworked image generator and generated application image files (now using unconstrained array as init images + unified array/memory types) |
| 12.06.2021 | 1.5.6.8 | :bug: fixed bug in instruction cache (cache controller might have missed resync/"clear-and-reload" requests from `fence.i` instructions); minor project/repo clean-ups |
| 08.06.2021 | 1.5.6.7 | clean-up of Wishbone interface module (dead code removal); added new package constant `wb_rx_buffer_c` to configure SYNC (default) or ASYNC Wishbone RX path (allows trade-off between performance/latency and timing closure) |
| 06.06.2021 | 1.5.6.6 | :bug: fixed bug in PWM base address configuration; :warning: removed user-access HPM counter access via `hpmcounter3[h]`:`hpmcounter3[h]` CSRs, hardwired according `mcounteren` bits to zero: HPM can only be used in machine mode; reworded 64-bit counters (`cycle`, `instret`, `hpmcounter` + `mtime`) overflow logic: now using dedicated CARRY chain instead of overflow detector (can improve timing); |
| 05.06.2021 | 1.5.6.5 | removed debug mode's `stepie` flag (used to allow interrupts during single-stepping) as the debugger can emulate interrupts |
| 04.06.2021 | 1.5.6.4 | :warning: removed `IO_PWM_EN` generic, replaced by `IO_PWM_NUM_CH` generic - PWM controller now supports implementation of up to 60 channels via `IO_PWM_NUM_CH` (`IO_PWM_NUM_CH` = 0 will omit the PWM controller); :bug: fixed minor bug in `minstreth` counter logic |
| 04.06.2021 | 1.5.6.3 | :warning: increased processor-internal IO size from 256 bytes to 512 bytes; relocated base address of CFS |
| 03.06.2021 | 1.5.6.2 | :warning: The `B` ISA extension (bit manipulation) has been (temporarily) removed from the project. See [B ISA Extension](https://github.com/stnolting/neorv32/projects/7) project board. |
| 03.06.2021 | 1.5.6.1 | CPU/HPM counter size configuration (`CPU_CNT_WIDTH` and `HPM_CNT_WIDTH` generics) can now be 0-bit (no counters implemented at all) to 64-bit (full-scale / RISC-V standard) wide |
| 01.06.2021 | **:rocket:1.5.6.0** | **New release** |
| 01.06.2021 | 1.5.5.13 | :warning: fixed project's endianness inconsistency (issue [#50](https://github.com/stnolting/neorv32/issues/50)) - CPU and processor are **little-endian**; changed image generator (`sw/image_gen`) and bootloader to generate/use little-endian executables; external memory interface is little-endian by default; removed `mstatus.ube` bit (reads as zero now); removed `mstatush` CSR |
| 31.05.2021 | 1.5.5.12 | `mret` instruction now clears `mstatus.mpp` (according to _new_ RISC-V privileged specs.) |
| 31.05.2021 | 1.5.5.11 | :warning: `mtval` CSR is now read-only; a write access will raise an illegal instruction exception |
| 30.05.2021 | 1.5.5.10 | :bug: fixed bug in processor's reset system (system reset stuck at `0` if on-chip debugger not implemented); reworked processor's reset generator system; VHDL code clean-up; reworked SoC's bus infrastructure (now using array of records for module bus response) |
| 28.05.2021 | 1.5.5.9 | integrated DBMEM (debug memory) component into DM (debug module); removing now-obsolete `neorv32_debug_dbmem.vhd` component |
| 22.05.2021 | 1.5.5.8 | :sparkles: **on-chip debugger (OCD)**: added debug module (`DM`) component; **OCD is operational now** (but still experimental) |
| 22.05.2021 | 1.5.5.7 | :bug: fixed bug in internal memory monitoring: if accessing an unused address which is not re-directed to the external bus interface (because WISHBONE module is disabled) caused the CPU to stall since that bus access was not correctly monitored and aborted by the BUS_KEEPER |
| 21.05.2021 | 1.5.5.6 | **on-chip debugger**: added debug transport module (`DTM`) component |
| 20.05.2021 | 1.5.5.5 | added system time output `mtime_o` (64-bit) driven by processor-internal _MTIME_ unit (idea [#29](https://github.com/stnolting/neorv32/discussions/29))
| 20.05.2021 | 1.5.5.4 | **on-chip debugger**: added debug memory (`DBMEM`) component |
| 20.05.2021 | 1.5.5.3 | added flag (SYSINFO.FEATURES) to allow software to discover if on-chip debugger is implemented (`SYSINFO_FEATURES_OCD`); added documentation [https://stnolting.github.io/neorv32/#_on_chip_debugger_ocd](https://stnolting.github.io/neorv32/#_on_chip_debugger_ocd) |
| 19.05.2021 | 1.5.5.2 | :sparkles: added **RISC-V CPU Debug Mode**, compatible to [RISC-V debug spec](https://github.com/riscv/riscv-debug-spec); new CSRs: `dcsr`, `dpc`, `dscratch`; new instructions: `dret`; :warning: debug mode is still **work-in-progress** and not operational yet! updated documentation CI [#26](https://github.com/stnolting/neorv32/pull/26), contributed by [umarcor](https://github.com/umarcor) :+1:; `fence.i` will not longer trap if executed but not implemented (`CPU_EXTENSION_RISCV_Zifencei` = false) |
| 13.05.2021 | 1.5.5.1 | added [`UPduino_v3`](https://github.com/stnolting/neorv32/tree/main/boards/UPduino_v3) example setup; renamed signal in watchdog module (`rtl/core/neorv32_wdt.vhd`) - collision with reserved keyword in vhdl-2008 (fixing issue [#24](https://github.com/stnolting/neorv32/issues/24)) |
| 10.05.2021 | **:rocket:1.5.5.0** | **New release** |
| 10.05.2021 | 1.5.4.12 | :warning: `mip` CSR is now read-only (pending IRQs can be cleared by disabling (and re-enabling) the according `mie` bit), writing to `mip` will raise an illegal instruction exception; :sparkles: added non-maskable interrupt (NMI), top entity port `nm_irq_i`; added new NMI to NEORV32 runtime environment |
| 09.05.2021 | 1.5.4.11 | added new flags to `mzext` CSR: *CSR_MZEXT_PMP* (set if at least 1 PMP region is implemented at all), *CSR_MZEXT_HPM* (set if at least 1 HPM counter is implemented) |
| 03.05.2021 | 1.5.4.10 | minor code clean-ups; moved FIRQ synchronization registers to top, removed sync FFs for processor-internal sources; |
| 30.04.2021 | 1.5.4.9 | moved definitions of IO area from `crt0.S` to linker script; reworked CPU's CSR access system - highly reducing area overhead (removing decoding logic of not implemented CSRs by heavily using VHDL's `NULL` statement in `case` constructs) |
| 29.04.2021 | 1.5.4.8 | minor edits in CPU instruction fetch engine; reduced **processor-internal bus timeout** (`max_proc_int_response_time_c`) to 15 cycles; added flag to SYSINGO module (`SYSINFO_FEATURES_HW_RESET`) to check if a dedicated hardware reset of all core register is implemented (via package's `dedicated_reset_c` constant) |
| 28.04.2021 | 1.5.4.7 | :bug: fixed bug in instruction cache (iCACHE) when using two sets - `ICACHE_ASSOCIATIVITY` = 2: cache was corrupting the non-active set |
| 26.04.2021 | 1.5.4.6 | optimized CPU's instruction fetch unit: less overhead for branches, reduced unit's hardware complexity |
| 25.04.2021 | 1.5.4.5 | :sparkles: :warning: removed `cancel` signals from processor-internal bus system; removed CPU's internal bus access timeout counter; added new top generic: `MEM_EXT_TIMEOUT` - type `natural`, default = 255; used to configure optional auto-timeout of Wishbone interface (if an **external** device is not responding within `MEM_EXT_TIMEOUT` clock cycles); set to zero to disable auto-timeout (required to comply with AXI4-Lite specs. when using the top's AXI wrapper) |
| 25.04.2021 | 1.5.4.3 | :sparkles: converted NEORV32.pdf data sheet to [`asciidoc` using asciidoctor](https://asciidoctor.org/); added data sheet sources to [`docs/src_adoc`](https://github.com/stnolting/neorv32/blob/main/docs/src_adoc) |
| 21.04.2021 | 1.5.4.3 | :warning: :bug: reworked *atomic memory access* system due to conceptual design errors: new system will make atomic LR/SC combinations fail when there is a trap (like a context switch) between the two instructions; new system prohibits SC from writing to memory if exclusive access fails; removed top's `wb_tag_i` signal, pruned one bit of top's `wb_tag_o` signal (atomic access), added top's `wb_lock_o` signal; updated sections in NEORV32.pdf regarding atomic memory accesses |
| 19.04.2021 | 1.5.4.1 | added register stage to `MTIME.time` write access to improve timing closure |
| 17.04.2021 | **:rocket:1.5.4.0** | **New release** |
| 16.04.2021 | 1.5.3.13 | :warning: added new top configuration generic `TINY_SHIFT_EN` (type = `boolean`, default = `false`) to configure a tiny single-bit (iterative) shifter for CPU ALU shift operations (for highly area-constrained setups) |
| 16.04.2021 | 1.5.3.12 | :sparkles: reworked reset system of the complete CPU: by default most registers (= "uncritical registers") **do not** provide an initialization via hardware reset; a **defined reset value** can be enabled by setting a constant from the main VHDL package (`rtl/core/neorv32_package.vhd`): `constant dedicated_reset_c : boolean := false;` (set `true` to enable CPU-wide dedicated register reset); see new section "2.11. CPU Hardware Reset" of NEORV32.pdf for more information |
| 14.04.2021 | 1.5.3.11 | minor rtl edits to allow synthesis using [`ghdl-yosys-plugin`](https://github.com/ghdl/ghdl-yosys-plugin) (:construction: work in progress :construction:)
| 13.04.2021 | 1.5.3.10 | :bug: fixed bug when configuring `HPM_CNT_WIDTH` less than 32; :warning: added new generic `CPU_CNT_WIDTH` to configure total size of CPU's `cycle` and `instret` CSRs (default = 64-bit); added `Zxnocnt` (no counters) and `Zxscnt` (small counters) flags to `mzexr` CSR to check if `CPU_CNT_WIDTH` is zero or less than 64, respectively; :bug: fixed bug in `crt0.S` start-up code: stack pointer has to be initialized before an exception can occur; updated `cpu_test` example program |
| 11.04.2021 | 1.5.3.9 | :warning: reworked CPU reset system (in addition to modifications in v1.5.3.7): default reset: most register are "initialized" with '-' (don't care) since no real reset is required; however, a "real" reset can be configured using the packages 'def_rst_val_c' constant that defines the reset value for all "uncritical regsiter" (see new NEORV32.pdf section 2.11 "CPU Hardware Reset") |
| 09.04.2021 | 1.5.3.8 | optimized CPU control: register write back during multi-cycle ALU operation only when result is really available (reducing switching activity; avoids possible source operand corruption); optimized `M` extension's co-processor: multiplications and divisions are 2 cycles faster |
| 08.04.2021 | 1.5.3.7 | :bug: fixed bug in HPM event configuration via `mhpmevent*` CSRs - there was a CSR address decoding overlap between the HPM event CSRs and the machine trap setup CSRs (introduced in version 1.5.3.6); :warning: reworked CPU core CSRs: most CSRs are not reset by hardware and need explicit initialization (done by crt0.S start-up code) |
| 02.04.2021 | 1.5.3.6 | :bug: fixed bug in external memory interface (`neorv32_wishbone.vhd`) that caused bus exceptions when using external memories with very high access latencies (race condition in bus timeouts); VHDL code clean-up |
| 30.03.2021 | 1.5.3.5 | added new top's generic `HPM_CNT_WIDTH` (type `natural`, default=40) to configure the total bit width of the hardware performance monitors (HPM) counter (min 1, max 64); mofified `crt0.S`: stops all counters (incl. HPMs), no user-level access to ANY counter; `neorv32.h`: added missing `mcounteren` and `mcountinhibit` CSR bit definitions |
| 28.03.2021 | 1.5.3.4 | default "test setup" `rtl/top_templetes/neorv32_test_setup.vhd`: disabled PMP, implementing 4 HPM counters; :sparkles: added [`boards`](https://github.com/stnolting/neorv32/tree/main/boards) folder for exemplary FPGA setups |
| 27.03.2021 | 1.5.3.3 | minor optimization in CPU control engine; FPU comparator now uses comparators results from main ALU (reduces FPU hardware footprint) |
| 26.03.2021 | 1.5.3.2 | :sparkles: **added single-precision floating-point unit (FPU)** `rtl/core/neorv32_cpu_cp_fpu.vhd` implementing the `Zfinx` CPU extension; added/updated `Zfinx` **intrinsic library** and verification framework: [`sw/example/floating_point_test`](https://github.com/stnolting/neorv32/tree/main/sw/example/floating_point_test); added co-processor timeout counter to CPU to auto-terminate co-processor operations (for debugging only; defaullt=deactivated) |
| 25.03.2021 | 1.5.3.1 | :bug: fixed bug in invalid floating-point instruction detection (caused CPU to stall if executing an invalid floating-point operation); intrinsic core library (mainly used for not-yet-supported CPU extensions like `B` and `Zfinx`): clean-up, added R3 instruction type |
| 24.03.2021 | **:rocket:1.5.3.0** | **New release** |
| 23.03.2021 | 1.5.2.9 | :sparkles: added new top generic to enable single-precision floating-point extensions `Zfinx`: `CPU_EXTENSION_RISCV_Zfinx` - type `boolean`, default = `false`, :warning: **extension is not yet operational!**; bootloader now shows available `Z*` extensions (from `mzext` CSR) like `Zifencei` |
| 21.03.2021 | 1.5.2.8 | :bug: fixed problem with linking `math.h` library in makefile; added floating-point-related global definitions to main VHDL package; added intrinsic core library file `sw/lib/include/neorv32_intrinsics.h` - intrinsic library support for CPU extensions, which are not yet supported by the upstream GCC, are based on this |
| 18.03.2021 | 1.5.2.7 | :bug: fixed bug in `sw/common/crt0.S` dummy exception handler (wrong order of register push/pop); changed upcoming floating-point extension (originally `F` extension) to `Zfinx` extension (-> [RISC-V `Zfinx` spec](https://github.com/riscv/riscv-zfinx)) - updated CPU infrastructure |
| 16.03.2021 | 1.5.2.6 | reworked atomic/exclusive memory access interface: removed CPU's `d_bus_lock_o` and `i_bus_lock_o` signal (was always zero anyway); removed top's `wb_lock_o` signal; added *exclusive access request* to Wishbone tag signal `wb_tag_o` (is now one bit wider); added more details to NEORV32.pdf regarding excluisve/atomic memory accesses (interface/protocol) |
| 09.03.2021 | 1.5.2.5 | added bit-manipulation `Zba` sub-extension (shifted-adds: `SH1ADD` `SH2ADD` `SH3ADD`) |
| 07.03.2021 | 1.5.2.4 | :sparkles: added new IO/peripheral module: **Smart LED Interface (NEOLED)** to interface intelligent LEDs (WS2812/WS2811/NeoPixel(c) compatible; supports RGB and RGBW LEDs in *parallel*) with internal TX buffer; new top generics: `IO_NEOLED_EN`: implement NEOLED interface when *true*; new top signals: `neoled_o`: single-wire async. serial data interface; FIFO re-fill interrupt via *fast interrupt request channel 9* `FIRQ9`; added new "NEOLED" section to data sheet; added SW driver library and simple NEOLED example program (`sw/example/demo_neopixel`) |
| 06.03.2021 | 1.5.2.3 | clean-up of CPU control code: fixed minor bug in F-exension's instruction decoding; changed coding style for CSR write access (old version might have caused "inferring latch..." warning in Intel Quartus); fixed default values for CSRs when according extensions are disabled |
| 04.03.2021 | 1.5.2.2 | added two new generics to configure CFS IO conduit sizes (implementing [issue #13](https://github.com/stnolting/neorv32/issues/13)): `IO_CFS_IN_SIZE` - type: `positive`, configures the size of `cfs_in_i` signal; `IO_CFS_OUT_SIZE` - type: `positive`, configures the size of `cfs_out_o` signal; minor edits to floating-point CPU infrastructure |
| 03.03.2021 | 1.5.2.1 | added CPU core infrastructure for *upcoming* single-precision floating-point extension `F`; :warning: **floating-point extension is NOT OPERATIONAL YET!** added new rtl file for the floating-point unit `rtl/core/neorv32_cpu_cp_fpu.vhd` (blank template!) |
| 01.03.2021 | **:rocket:1.5.2.0** | **New release** |
| 27.02.2021 | 1.5.1.11 | :bug: fixed several small bugs in *bitmanipulation extension* instruction decoding (not all `B` instructions triggered and *illegal instruction exception* when B-extension = disabled) |
| 25.02.2021 | 1.5.1.10 | :bug: fixed bugs in UART RTS/CTS hardware control flow - the new setup was verified on real hardware; added double-buffering to UART RX engine |
| 24.02.2021 | 1.5.1.9 | `mcounteren` CSR is hardwired to zero if user mode is not implemented (`CPU_EXTENSION_RISCV_U` = false); added `Zbs` (single-bit operations) sub-extension to bit-manipulation unit |
| 22.02.2021 | 1.5.1.8 | added programmable *RTS/CTS hardware flow control* to UARTs; new top signals: `uart0_rts_o`, `uart0_cts_i`, `uart1_rts_o`, `uart1_cts_i`; UART.TX engine will only start sending (if `CTS` flow control is activated) if `uart*_cts_i` is asserted (low-active); UART.RX engine signals (if `RTS` flow control is activated) via `uart*_rts_o` if it is ready to receive new data (low-active); added hw flow control parameter to uart setup functions `neorv32_uart*_setup()` |
| 20.02.2021 | 1.5.1.7 | removed `err_o` signal from custom functions subsystem `CFS`; processor *SoC fast interrupt input* `soc_firq_i` reduced to 6 channels (was 8) - mapped to CPU's `FIRQ_10` - `FIRQ_15`; added individual fast IRQs for `UART1` "RX complete" and "TX complete" conditions (-> `FIRQ_4` & `FIRQ_5`); changed FIRQ channels of TWI/SPI/GPIO interrupts |
| 18.02.2021 | 1.5.1.6 | added register buffer for enable signals to processor-internal clock generator; :bug: fixed bug in `sw/example/demo_twi` program: TWI clock speed message was wrong (factor 1/4 was missing) |
| 17.02.2021 | 1.5.1.5 | added a second independent UART: new UART is *secondary UART* `UART0`, the "old" UART is now the *primary UART* `UART0`; by default the **primary UART (UART0) is used for all user interface connection**; reworked *fast interrupt* `FIRQ` assignment/priority list - added UART1 RTX (receive *or* send done) fast interrupt; added hardware driver functions for new `UART1` - the "old" `neorv32_uart_*` function calls will map to the primary UART `UART0` for compatibility; renamed compiler flag to enable UART "simulation mode": `UART_SIM_MODE` -> `UART0_SIM_MODE` for primary UART, `UART1_SIM_MODE` for secondary UART (`UART_SIM_MODE` is still supported for compatibility and maps to `UART0_SIM_MODE`); added second simulation UART receiver for `UART1` to testbench; renamed UART simulation output files: `neorv32.testbench_uart.out` -> `neorv32.testbench_uart0.out` (testbench UART0 receiver), new: `neorv32.testbench_uart1.out` (testbench UART1 receiver), `neorv32.uart.sim_mode.text.out` and `neorv32.uart.sim_mode.data.out` -> `neorv32.uart0.sim_mode.text.out` and `neorv32.uart0.sim_mode.data.out` (for `UART0`), new `neorv32.uart1.sim_mode.text.out` and `neorv32.uart1.sim_mode.data.out` (for `UART1`) |
| 13.02.2021 | 1.5.1.4 | `HW_THREAD_ID` generic is now of type `natural`; `mret` instruction now requires an additional cycle to execute; logic optimization of CPU's control logic -> smaller hardware footprint and higher f_max; updated CPU synthesis results; removed top module's generic initialization using `(others => '0')` (targeting [issue #8](https://github.com/stnolting/neorv32/issues/8)) |
| 09.02.2021 | 1.5.1.3 | modified CPU architecture: now using a "pseudo" ALU co-processor to get the result of a CSR read operation into data path, removing one input from register file input mux -> shorter critical path |
| 08.02.2021 | 1.5.1.2 | added new peripheral/IO module: **Numerically-Controlled Oscillator `NCO`**: three independent channels, 20-bit phase accu, 20-bit tuning word, fixed 50% duty cycle mode or pulsed mode; added according HW drivers and example program |
| 07.02.2021 | **:rocket:1.5.1.0** | **New release** |
| 05.02.2021 | 1.5.0.11 | :bug: fixed error in atomic instruction `LR.W` |
| 05.02.2021 | 1.5.0.10 | CPU now provides 16 fast interrupt request lines (`FIRQ0 .. FIRQ15`) with according `mie`/`mip` CSR bits and `mcause` trap codes; removed IRQ enable flags from SPI, UART & TWI; reworked processor-internal interrupt system - assignment/priority list; UART now features individual IRQs for "RX-done" and "TX-done" conditions; changed bit order in TWI control register |
| 29.01.2021 | 1.5.0.9 | removed custom function units `CFU0` & `CFU1`; :sparkles: replaced them by new *Custom Functions Subsystem `CFS`*, which provides up to 32x32-bit memory-mapped registers; new configuration generics: `IO_CFS_EN`, `IO_CFS_CONFIG`; new top entity signals: `cfs_in_i`, `cfs_out_o`; increased processor's IO area from 128 bytes to 256 bytes, now starting at `0xFFFFFF00` |
| 28.01.2021 | 1.5.0.8 | added *critical limit* for number of implemented PMP regions: When implementing more PMP regions that a certain critical limit an additional register stage is automatically inserted into the CPU’s memory interfaces increasing the latency of instruction fetches and data access by +1 cycle. The critical limit can be adapted for custom use by a constant from the main VHDL package file (rtl/core/neorv32_package.vhd). The default value is 8: `constant pmp_num_regions_critical_c : natural := 8;` |
| 27.01.2021 | 1.5.0.7 | added four additional *fast interrupt* channels `FIRQ4..7`, available via processor's top `soc_firq_i(3:0)` signal for custom platform use; fixed minor error in UART setup function (baud rate prescaler calculation for very high baud rates) |
| 26.01.2021 | 1.5.0.6 | minor logic optimization of CPU's `B` extension co-processor (reducing area); minor logic optimization or `HPM` triggers (reducing area); reworked CPU's co-processor interface; minor logic optimization of branch condition check (to shorten critical path) |
| 23.01.2021 | 1.5.0.5 | reworked true random number generator `TRNG`: architecture is now based on several simple ring oscillators with incrementing length; changed control register bits; updated according driver functions and demo program |
| 22.01.2021 | 1.5.0.4 | :bug: fixed BUG in bootloader (that caused it to immediately crash after reset if SPI/MTIME/GPIO peripherals were not implemented); reworked watchdog timer `WDT`: removed watchdog access password, added option to lock configuration until next system reset, changed control register bits - updated driver functions and demo/test programs |
| 17.01.2021 | 1.5.0.3 | CPU data register file can now be mapped to a **single** "true dual-port" block RAM by the synthesizer (requiring only 1024 memory bits instead of 2048); :bug: fixed typo error in `sim/rtl_modules/neorv32_imem.vhd`; modified `M` co-processor (due to register file read access modification), reduced switching activity when co-processor is idle; logic/arithmetic operations of `B` extension only require 3 cycles now, reduced switching activity when co-processor is idle |
| 15.01.2021 | 1.5.0.2 | added instruction cache associativity configuration (number of sets); new configuration generic: `ICACHE_ASSOCIATIVITY` -> number of sets (1 = direct mapped, 2 = 2-way set-associative), has to be a power of two; if associativity is > 1 the used replacement policy is *least recently used (LRU)*; :bug: fixed bug in `sw/lib/source/neorv32_cpu.c` PMP.CFG configuration function |
| 14.01.2021 | 1.5.0.1 | added new HPM trigger event: multi-cycle ALU operation wait cycle (`HPMCNT_EVENT_WAIT_MC`); renamed `neorv32_cache.vhd` -> `neorv32_icache.vhd` |
| 10.01.2021 | **:rocket:1.5.0.0** | Renamed configuration generics: `*_USE` -> `*_EN` |
| 10.01.2021 | 1.4.9.10 | :sparkles: Added support for [**bit manipulation extension (`B`)**](https://github.com/riscv/riscv-bitmanip) - base subset `Zbb` only (:warning: RISC-V `B` (sub-)extensions are not officially ratified yet; compatible to version "0.94-draft"); enabled via new configuration constant `CPU_EXTENSION_RISCV_B` (default = false); uported `Zbb` instructions: `CLZ` `CTZ` `CPOP` `SEXT.B` `SEXT.H` `MIN[U]` `MAX[U]` `ANDN` `ORN` `XNOR` `ROL` `ROR` `RORI` `zext`(*pseudo-instruction* for `PACK rd, rs, zero`) `rev8`(*pseudo-instruction* for `GREVI rd, rs, -8`) `orc.b`(*pseudo-instruction* for `GORCI rd, rs, 7`); added `B` flag to `misa` CSR; added `Zbb` flag to `mzext` CSR |
| 03.01.2021 | 1.4.9.8 | Added HPM trigger for instruction issue wait cycle (caused by pipeline flush); all HPM counters do not increment if CPU is sleep mode; fixed CoreMark timer overflow issues; `rtl/core/neorv32_busswitch.vhd`: removed wait states, less load/store wait cycles -> faster execution; updated CoreMark results |
| 02.01.2021 | 1.4.9.7 | :sparkles: added RISC-V hardware performance monitors (`HPM`); new CSRs: `mhpmevent*`(3..31), `[m]hpmcounter*[h]`(3..31), amount configurable via top's generic `HPM_NUM_CNTS`; supported counter events: active cycle, retired instruction, retired compressed instruction, instruction fetch memory wait cycle, load operation, store operation, load/store memory wait cycle, unconditional jump, conditional branche (all), conditional taken branch, entered trap, illegal instruction exception; PMP can now have up to 64 regions; number of regions configured via top's `PMP_NUM_REGIONS` generic; removed obsolete top's `PMP_USE` generic; removed PMP flag from `mzext` CSR; minimal region granularity (in bytes) configured via top's `PMP_MIN_GRANULARITY` generic, has to be a power of two and >= 8 bytes; :bug: fixed bug in sleep (`wfi`) instruction |
| 29.12.2020 | 1.4.9.5 | New UART features: "frame check" (test if stop bit is set), error indicated via `UART_DATA` reg's `UART_DATA_FERR` flag; configurable parity bit (`UART_CT.UART_CT_PMODE1:UART_CT_PMODE0`, 00=no parity; 10=even parity; 11=odd parity); parity error indicated via `UART_DATA` reg's `UART_DATA_PERR` flag; moved UART's RX overrun flag to `UART_DATA.UART_DATA_OVERR` |
| 26.12.2020 | 1.4.9.4 | removed `zicnt_en` option (was used to discard the standard RISC-V counters and timers from implementation); added missing `mcounteren` CSR (to allow read-access from user-level code to `cycle[h]` / `time[h]` / `[m]instret[h]` CSRs); available bits: 0: `CY`, 1: `TM`, 2: `IR`; added missing `mcountinhibit` CSR (to disable auto-increment of `[m]cycle[h]` / `[m]instret[h]` CSRs); available bits: 0: `CY`, 2: `IR`; :warning: renamed CSR bits C-code-aliases: `CPU_*` -> `CSR_*` |
| 25.12.2020 | 1.4.9.3 | Added missing `UBE` flag to `mstatus` CSR, indicates Endianness for load/stores in user mode (always set indicating BIG-endian mode), is a copy of `mstatush.mbe` |
| 23.12.2020 | 1.4.9.2 | :sparkles: added processor-internal instruction cache `rtl/core/neorv32_cache.vhd` (direct mapped); new configuration generics: `ICACHE_USE` (implement cache), `ICACHE_BLOCK_SIZE` (cache block/page/line size), `ICACHE_NUM_BLOCKS` (number of cache blocks); added `SYSINFO_CACHE` register to SYSINFO to check cache configuration by software  |
| 20.12.2020 | 1.4.9.1 | :bug: fixed bug in CPU's instruction fetch engine (alignment_errros/bus_errors were not acknowledged correctly); added `BUS_TIMEOUT` generic to CPU (defines the amount of cycles after which an *unacknowledged* bus access will get terminated and raises a bus access fault exception) |
| 19.12.2020 | **:rocket:1.4.9.0** | Testbench: added memory-mapped triggers to trigger core's "machine software & external interrupts"; `sw/example/cpu_test`: removed CFU tests, added `MEI` and `MSI` tests; added **RISC-V-Compliance Test Framework** to repository (`riscv-compliance/`), core passes all `rv32` tests (riscv-compliance v2.1) |
| 18.12.2020 | 1.4.8.13 | Added additional simulation files: simulation-optimized IMEM-ROM (so far, this is only relevant for the *new* NEORV32 RISC-V Compliance test framework v2.0); **:sparkles: Processor now passes all `rv32` tests of the new [RISC-V Compliance Test Framework v2.0](https://github.com/riscv/riscv-compliance/releases/tag/v2.0) :sparkles:** |
| 16.12.2020 | 1.4.8.12 | :warning: fixed (another) bug in `mtval` CSR generation (wrong value for "breakpoint" trap); updated `mtval` value table in data sheet; fixed bug in load/store operation (introduced in version 1.4.8.10) |
| 16.12.2020 | 1.4.8.11 | :warning: fixed bug in `mtval` CSR generation (wrong values for some traps); fixed bug in `mip` CSR (writing zero to implemented bits now actually clears pending interrupts); fixed bug in IRQ priority encoding (machine software interrupt `MSI` comes before machine timer interrupt `MTI`) |
| 12.12.2020 | 1.4.8.10 | :warning: fixed wrong `trap_reset_c` encoding (in it's expanded form it should be 0x80000000) and reset logic: hardware `mcause` register is now set to `trap_reset_c` after a hardware reset; crt0.S start-up code now sets `mcause` to `trap_reset_c` after finishing hardware setup |
| 11.12.2020 | 1.4.8.9 | Added option to exclude standard RISC-V performance counters (`[m]cycle[h]` and `[m]instret[h]`) for size-constrained implementations; disabled by setting VHDL package's `zicnt_en_c` constant to false; software can determine state of `zicnt_en_c` via `mzext` CSR's `CPU_MZEXT_ZICNT` bit; added new signal to processor top entity: `mtime_i`, this signal is used for updateting the `time[h]` CSRs if the processor-internal MTIME unit is disabled (via `IO_MTIME_USE` = `false`) |
| 10.12.2020 | 1.4.8.8 | Added missing `mstatush` CSR (only bit `MBE` is implemented yet); added option to configure external bus interface for BIG- or little-endian byte-order, configured via VHDL package `xbus_big_endian_c` constant, default = BIG-endian, software can check endianness of the interface via SYSINFO's `SYSINFO_FEATURES(SYSINFO_FEATURES_MEM_EXT_ENDIAN)` flag; added `mstatush` CSR and endianness information to data sheet |
| 09.12.2020 | 1.4.8.7 | Added missing *environment call from U-mode* exception (via `ecall` instruction in user-mode); added environment call from U-mode to data sheet |
| 09.12.2020 | 1.4.8.6 | :warning: fixed bugs in ALU's co-processor interface: ATOMIC `A` extension could not be used without MULDIV `M` extension, CPU might have permanently stalled when executing an instruction from a disabled ISA extension; :lock: added security feature: illegal user-level CSR read access will always return zero; added new section *Execution Safety* to neorv32.pdf data sheet |
| 07.12.2020 | 1.4.8.5 | :warning: fixed bug in next-PC logic (introduced with version 1.4.8.1) that caused instruction fetch from memories with more than 1 cycle latency to fail |
| 05.12.2020 | 1.4.8.4 | :warning: fixed bug in physical memory protection (PMP): region size configuration was incorrect; removed `PMP_NUM_REGIONS` and `PMP_GRANULARITY` CPU/processor generics (PMP configuration now via package constants); reworked section *2.4. Instruction Sets and CPU Extensions* of neorv32.pdf |
| 04.12.2020 | 1.4.8.2 | Added PMA (physical memory attribute) to processor-internal IO region: `NO EXECUTE`; added *3.3.Address Space/Physical Memory Attributes (PMAs)* section to neorv32.pdf |
| 03.12.2020 | 1.4.8.1 | Optimized CPU program counter (PC) update logic and "next PC" computation (shortened critical path); updated bootloader (configuration option for direct-boot-from-SPI-flash only) and *customization* text in neorv32.pdf |
| 01.12.2020 | **:rocket:1.4.8.0** | :warning: fixed bug in CPU-internal co-processor interface; optimized multiplier unit (~1 faster); added CPU `A` (atomic) extension support (only `lr.w` and `sc.w` instructions yet); added `lock` signal to CPU and processor's external bus interface |
| 28.11.2020 | 1.4.7.6 | Split ALU core operations: shortened  critical path - replaced ALU output 8:1 mux by a 4:1 mux |
| 26.11.2020 | 1.4.7.5 | Minor rtl clean-up; CSR access instructions are one cycle faster now (3 cycles now); system/environment instructions (`ecall` `ebreak` `mret` `wfi`) need one additional cycle (4 cycles now) |
| 25.11.2020 | 1.4.7.4 | :warning: fixed bug in `FENCE.I` instruction that corrupted instruction fetch when executing code from processor-external memory; default testbench (`sim/neorv32_tb.vhd`) now features external IMEM, external DMEM and external IO connected via external bus interface; simulation now allows CPU to execute code using external memories only (no internal IMEM/DMEM); optimized CPU's instruction fetch interface (no more unnecessary transfer cancel requests) |
| 20.11.2020 | 1.4.7.2 | :warning: fixed bug in CPU bus unit that caused a memory exception after reset in some cases; added second simulated external (Wishbone) memory to testbench (one memory for simulating an external IMEM, one memory for simulating external memory-mapped IO); external bus interface (`wishbone`) now makes sure that a canceled bus transfer is really understood by the accessed peripheral |
| 20.11.2020 | 1.4.7.1 | Removed deprecated "update_enable signal" from IMEM |
| 11.11.2020 | **:rocket:1.4.7.0** | Further optimized pipeline front-end: Jumps and branches are one cycle faster (+5% coremark performance); updated synthesis results; updated performance results; added `hello_world` example program |
| 07.11.2020 | 1.4.6.7 | Updated bootloader (size optimization) and changed processor version output; added project logo; minor data sheet edits |
| 03.11.2020 | 1.4.6.6 | Removed SPI module's *buggy* "LSB-first mode", SPI module now always sends data MSB-first; removed SPI.CTRL `SPI_CT_DIR` bit; modfied bit order in SPI CTRL register; updated SPI SW library |
| 02.11.2020 | 1.4.6.5 | :warning: Fixed bug in CPU's illegal instruction detection logic; CPU rtl code optimizations - further reduced hardware footprint; rtl code clean-ups |
| 01.11.2020 | 1.4.6.4 | :warning: Fixed bug in `[m]instret[h]` and `[m]cycle[h]` carry logic; CPU hardware optimizations (area reduction, shortened critical path) |
| 29.10.2020 | 1.4.6.3 | rtl code clean-up; made preparations for additional co-processors |
| 25.10.2020 | 1.4.6.2 | Added tag signal (`wb_tag_o`) to processor's Wishbone bus; removed processor's `priv_o` - privilege level is now encoded in Wishbone *tag* signal; added a more sophisticated **FreeRTOS** example ("full_demo") |
| 24.10.2020 | **:rocket:1.4.6.0** | Completely reworked external memory interface (WISHBONE), removed now-obsolete processor generic `MEM_EXT_REG_STAGES`; added processor wrapper with **AXI4-Lite interface** |
| 22.10.2020 | 1.4.5.11 | TWI: Added new control register flag to enable/disable SCL clock stretching by peripheral devices |
| 22.10.2020 | 1.4.5.10 | Added `i_bus_priv_o` and `d_bus_priv_o` signals to CPU_top and `priv_o` to Processor_top to show privilege level of bus access (from `mstatus` MPP); :warning: Fixed bug in external memory interface [WISHBONE] (non-standard Wishbone components were able to corrupt processor-internal ACK/ERR signal logic) |
| 20.10.2020 | 1.4.5.9 | :warning: Fixed bug in CPU "sleep" instruction (`WFI` - wait for interrupt) |
| 20.10.2020 | 1.4.5.8 | *Machine timer interrupt* is available as processor input pin (`mtime_irq_i`) if internal `MTIME` is not implemented (`IO_MTIME_USE` = false) |
| 18.10.2020 | 1.4.5.7 | Added new IO peripheral/Device: Second CFU (CFU1); renamed old CFU to CFU0; CFU VHDL files: `neorv32_cfu0.vhd` & `neorv32_cfu1.vhd`; removed CFU interrupt |
| 17.10.2020 | 1.4.5.5 | New makefile target `upload` allows to directly upload an executable to the bootloader from the console |
| 17.10.2020 | 1.4.5.4 | Added new CPU/Processor generic `FAST_SHIFT_EN` (default = *false*) to enable implementation of a fast (but large) barrel shifter for accelerating CPU shift instructions; updated CoreMark performance results |
| 16.10.2020 | 1.4.5.2 | Added read-only flag to custom `mzext` CSR to check if physical memory protection (PMP) is implemented; added [C] `mzext` CSR name aliases to neorv32.h |
| 15.10.2020 | 1.4.5.1 | Fixed "unprecise exceptions": `mtval` did not always reflect the correct value according to the instruction that caused the exceptions; fixed bug in RTE: Debug trap handler was not showing the correct `mepc` value |
| 13.10.2020 | **:rocket:1.4.5.0** | An official *open-source RISC-V architecture ID* was assigned to the project: decimal = `19`, 32-bit hexadecimal = `0x00000013` - software can retrieve the ID from the `marchid` CSR |
| 12.10.2020 | 1.4.4.9 | Added *alignment flags* to makefiles: branch/jump/call targets are forced to be 32-bit aligned -> increases performance when using the `C` extension; added makefile flag listing to NEORV32.pdf; updated performance results for CPUs with `C` extension; `crt0.S` will initialize *all* registers with zero if not using `E` extension and not compiling bootloader |
| 11.10.2020 | 1.4.4.8 | Reworked pipeline frontend: Optimized fetch engine, added issue engine, faster instruction fetch after taken branches + reduced hardware requirements; updated synthesis and performance results |
| 11.10.2020 | 1.4.4.6 | Added option to configure external memory interface (Wishbone) to either use *standard/classic protocol* (default) or *pipelined protocol* (for better timing): via `wb_pipe_mode_c` constant in VHDL package file (`rtl/core/neorv32_package.vhd`); added help text to NEORV32.pdf section "3.4.4. Processor-External Memory Interface (WISHBONE)" |
| 08.10.2020 | 1.4.4.5 | Removed CPU's `BUS_TIMEOUT` and processor's `MEM_EXT_TIMEOUT` generics; instead, a global configuration `bus_timeout_c` in the VHDL package file is used now |
| 08.10.2020 | 1.4.4.4 | Removed `DEVNULL` device; all simulation output options from this device are now available as `SIM_MODE` in the `UART`; `mcause` CSR can now also be written; FIXED: trying to write a read-only CSR will cause an illegal instruction exception; for compatibility reasons any write access to the misa CSR will be ignored and will NOT cause an exception |
| 07.10.2020 | 1.4.4.2 | Simplified ALU's set of core operations; removed co-processor data mux right after ALU -> shorter critical path; CPU control VHDL code clean-up and CSR write logic optimization; optimized IMEM/DMEM access logic; added note regarding alignment of IMEM/DMEM |
| 05.10.2020 | **:rocket:1.4.4.0** | :warning: Fixed bug in external memory interface: Executing code from external memory was causing an instruction fetch stall |
| 02.10.2020 | 1.4.3.9 | `[m]cycleh` and `[m]instreth` CSRs are now 32-bit wide (-> fully RISC-V-compliant) |
| 01.10.2020 | 1.4.3.8 | Added CPU top entity wrapper with resolved port signals `rtl/top_templetes/neorv32_cpu_stdlogic.vhd`; optimized ALU core functions – shorter critical path, less control overhead, reduced HW footprint |
| 27.09.2020 | 1.4.3.3 | Further improved ALU and control logic; CSR access instruction require one additional cycle now (to let side effects kick in); updated synthesis results; added CFU hardware driver dummy |
| 26.09.2020 | 1.4.3.2 | :warning: Fixed bug in `CSRRWI` instruction (introduced with version 1.4.3.1); further ALU operand logic optimizations; updated CPU data path figure |
| 25.09.2020 | 1.4.3.1 | Register file's `x0` is now a physical register; this register is initialized by the hardware and locked afterwards; removed "set to zero" stage -> smaller hardware footprint and shorter critical path; added processor top entity wrapper with resolved signals `rtl/top_templetes/neorv32_top_stdlogic.vhd` |
| 16.09.2020 | **:rocket:1.4.3.0** | Simplified memory configuration: removed processor top's memory space configuration generics (`MEM_ISPACE_BASE`, `MEM_ISPACE_SIZE`, `MEM_DSPACE_BASE`, `MEM_DSPACE_SIZE`); data/instruction space sizes are irrelevant for hardware; instruction/data space base addresses are fixed (but can be modified in NEORV32 VHDL package file); modified SYSINFO registers; adapted bootloader, crt0 start-up code and linker script; stack configuration is now done via linker script; reworked chapter "address space"; added CFU interrupt -> fast interrupt channel 1 (shared with GPIO) |
| 14.09.2020 | 1.4.2.0 | Removed option to disable CSR counters (via `CSR_COUNTERS_USE` generic) since these counters are mandatory according to the RISC-V specs; added new IO/peripheral device: custom functions unit (`CFU`) for tightly-coupled custom co-processors; improved timing of processor-internal clock generator; fixed wrong labels in address space figure and removed dedicated exception vectors box; added mask register to GPIO unit to specify which input pins can trigger a pin-change interrupt |
| 11.09.2020 | 1.4.0.4 | Reworked `TRNG` architecture and interface; added text regarding fast interrupt channels usage for the NEORV32 processor |
| 02.09.2020 | 1.4.0.2 | :warning: Fixed bugs in external memory interface; added option to define latency of simulated external memory in testbench; hardware configuration sanity checks will now only appear once in console; added more details to data sheet section 3.3. Address Space; fixed typos in MEM_*_BASE and MEM_*_SIZE generic names |
| 01.09.2020 | 1.4.0.1 | Using registers above `x15` when the `E` extensions is enabled will now correctly cause an illegal instruction exception |
| 29.08.2020 | **:rocket:1.4.0.0** | Rearranged and reworked data sheet; added FreeRTOS port, demo & short referencing chapter; removed bootloader-specific linker scripts – main linker script is used for both, applications and bootloader; bootloader can now have `.data` and `.bss` sections; improved IMEM and BOOTROM memory initialization – faster synthesis; image generator now constrains init array size to actual executable size; peripheral/IO devices can only be written in full word mode (= 32-bit); GPIO ports are now 32-bit wide |
| 23.08.2020 | 1.3.7.3 | Added custom `mzext` CSR to check for available Z* CPU extensions; multiplier's FAST_MUL mode is one cycle faster now; updated performance data |
| 20.08.2020 | 1.3.7.2 | Removed bootloader-specific crt0 – bootloader now uses std crt0; makefiles now also support asm and cpp files; made linker scripts more general; renamed makefile "compile" (which is still available for compatibility) target into "exe" |
| 14.08.2020 | **:rocket:1.3.7.0** | Simplified CPU fetch engine; added configurable CPU instruction prefetch buffer (ipb) FIFO; optimized CPU execute engine; updated performance data |
| 06.08.2020 | 1.3.6.5 | Added `FAST_MUL_EN` generic to enable mapping of the multiplier core to DSP blocks; ALU.shifter is no more triggered when executing MULDIV operations; added benchmark results for DSP-based multiplier configurations; updated implementation and performance results; simplified makefiles – using implicit libc definition; crt0 only initializes lowest 16 registers |
| 03.08.2020 | **:rocket:1.3.6.0** | Relocated `DEVNULL` (changed base address); minor edits, optimization and clean-ups |
| 30.07.2020 | 1.3.5.2 | Added register stage to PMP mask generation to shorten critical path; removed automatic IRQ enable/disable from RTE install/uninstall functions |
| 30.07.2020 | 1.3.5.1 | :warning: Fixed bug(s) in PMP mask generation; `misa.Z` flag is not yet defined by the RISC-V specs., hence it is read-only and read as zero |
| 29.07.2020 | 1.3.5.0 | Added user privilege level, enabled via new `CPU_EXTENSION_RISCV_U` generic; :warning: fixed error in `mstatus(mpie)` logic; implemented RISC-V spec.-compliant Physical Memory Protection (PMP); allows up to 8 regions but only NAPOT mode is supported yet |
| 25.07.2020 | 1.3.0.0 | `mcause` CSR is read-only now!; removed `CLIC`, added 4 fast IRQ channels to CPU with according flags in `mie` and `mip` and trap IDs; updated core libraries; updated NEORV32 RTE; highly reworked data sheet; updated synthesis and performance results |
| 21.07.2020 | 1.2.0.6 | Added doc section regarding the CPU's data and instruction interfaces; optimized CPU fetch engine; updated iCE40 synthesis results |
| 20.07.2020 | 1.2.0.5 | Less penalty for taken branches and jumps (2 cycles faster) |
| 19.07.2020 | 1.2.0.0 | CPU bus unit now has independent buses for instruction fetch and data access – merged into single processor bus via new bus switch unit; doubled speed of ALU shifter unit again; all bits of `mcause` CSR can now be modified by application program (full RISC-V-compliant); performance counters CSRs `[m]cycleh` and `[m]instreth` are only 20-bit wide; removed NEORV32-specific custom CSRs – all processor-related information can be obtained from the new `SYSINFO` IO module (CPU is now more independent from processor configuration); changed IO address of `DEVNULL`; fixed bug in bootloader's trap handler; added `USER_CODE` generic to assign a custom user code that can be read by software (from `SYSINFO`) |
| 14.07.2020 | 1.1.0.0 | Added `fence_o` and `fencei_o` signals to top entity to show if a `fence` or `fencei` instruction is executed; added `mvendorid` and `marchid` CSRs (both are always zero); ALU shift unit is faster now; two lowest bits of `mtvec` are always zero; fixed wrong instruction exception priority; removed `HART_ID` generic – `mhartid` CSR is always read as zero; performance counters (`[m]cycle[h]`, `[m]instret[h]` and `time[h]`) are also available in embedded mode – but can be explicitly disabled via the `CSR_COUNTERS_USE` generic; mcause CSR only allows write access to bit 31 and bits 3:0; updated synthesis reports |
| 10.07.2020 | 1.0.6.0 | Non-taken branches are now 1 cycle faster; the `time[h]` CSR now correctly reflects the system time from the MTIME unit; fixed WFI instruction permanently stalling the CPU; `[m]cycle[h]` counters now stop counting when CPU is in sleep mode; `minstret[h]` and `mcycle[h]` now also allow write-access |
| 09.07.2020 | 1.0.5.0 | `X` flag of `misa` CSR is zero now; the default SPI flash boot address of the bootloader is now `0x0080000`; new exemplary FPGA utilization results for Intel, Lattice and Xilinx; `misa` CSR is read-only again, switching compressed extension on/off is pretty bad for the fetch engine; `mtval` and `mcause` CSRs now allow write accesses and are finally RISC-V-compliant; time low and high registers of `MTIME` peripheral can now also be written by user; `MTIME` registers only allow full-word write accesses |
| 06.07.2020 | 1.0.1.0 | Added missing `fence` instruction; added new generic to enable optional Zifencei CPU extension for instruction stream synchronization |
| 05.07.2020 | 1.0.0.0 | New CPU architecture: Fetch and execute engines; increased CPI; timer and counter CSRs are now all 64-bit wide; :warning: fixed CSR access errors; fixed `C.LW` decompression logic; `misa` flags `C` and `M` are now r/w – compressed mode and multiplier/divider support can be switched on/off during runtime; PC(0) is now always zero; :warning: fixed bug in multiplier/divider co-processor; renamed SPI signals; added RISC-V compliance check information – processor now passes the official RISC-V compliance tests |
| 25.06.2020 | 0.0.2.5 | Added `DEVNULL` device; added chapter regarding processor simulation; fixed/added links; fixed typos; added FPGA implementation results for iCE40 UP |
| 23.06.2020 | **:rocket:0.0.2.3** | Publication |
