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
| 01.12.2025 | [**1.12.5**](https://github.com/stnolting/neorv32/releases/tag/v1.12.5) | :rocket: **New release** | |
| 31.11.2025 | 1.12.4.9 | optimize CPU-DM request/acknowledge communication interface; smaller hardware and faster debugging | [#1435](https://github.com/stnolting/neorv32/pull/1435) |
| 26.11.2025 | 1.12.4.8 | :lock: :bug: fix PMP bugs: check `R+W` for atomic read-modify-write accesses; remove time multiplex to allow permanent bus access monitoring | [#1433](https://github.com/stnolting/neorv32/pull/1433) |
| 23.11.2025 | 1.12.4.7 | :sparkles: add support for RISC-V `Zimop` ISA extension (may-be-operations) | [#1431](https://github.com/stnolting/neorv32/pull/1431) |
| 22.11.2025 | 1.12.4.6 | :warning: extend max number of PWM channels from 16 to 32; rename PWM configuration generic to `IO_PWM_NUM`; resize `pwm_o` port | [#1429](https://github.com/stnolting/neorv32/pull/1429) |
| 22.11.2025 | 1.12.4.5 | :warning: rework/redesign general purpose timer (GPTMR) module | [#1428](https://github.com/stnolting/neorv32/pull/1428) |
| 21.11.2025 | 1.12.4.4 | :test_tube: cleanup NEORV32 RTE code and rewrite core in plain inline-assembly | [#1427](https://github.com/stnolting/neorv32/pull/1427) |
| 16.11.2025 | 1.12.4.3 | tracer: fix simulation log operand decoding | [#1425](https://github.com/stnolting/neorv32/pull/1425) |
| 15.11.2025 | 1.12.4.2 | :warning: rework PWM module | [#1424](https://github.com/stnolting/neorv32/pull/1424) |
| 07.11.2025 | 1.12.4.1 | minor rtl edits | [#1422](https://github.com/stnolting/neorv32/pull/1422) |
| 03.11.2025 | [**1.12.4**](https://github.com/stnolting/neorv32/releases/tag/v1.12.4) | :rocket: **New release** | |
| 01.11.2025 | 1.12.3.9 | :sparkles: add experimental support for the RISC-V `Zibi` ISA extension (branches with immediates) | [#1418](https://github.com/stnolting/neorv32/pull/1418) |
| 25.10.2025 | 1.12.3.8 | :bug: fix CLINT register read-back (bug introduced in previous version / v1.12.3.7) | [#1411](https://github.com/stnolting/neorv32/pull/1411) |
| 23.10.2025 | 1.12.3.7 | counter optimizations and PMP logic cleanup | [#1410](https://github.com/stnolting/neorv32/pull/1410) |
| 18.10.2025 | 1.12.3.6 | improve PMP logic (shortening of critical path) | [#1408](https://github.com/stnolting/neorv32/pull/1408) |
| 18.10.2025 | 1.12.3.5 | add `meta` signal to internal bus replacing individual `debug`, `priv` and `src` signals; refine reservation-set controller; minor RTL edits | [#1407](https://github.com/stnolting/neorv32/pull/1407) |
| 10.10.2025 | 1.12.3.4 | :warning: CFU: remove R4-type instruction support (three source registers); add support for I-type instructions | [#1402](https://github.com/stnolting/neorv32/pull/1402) |
| 06.10.2025 | 1.12.3.3 | fix `Sdext` trigger type enumeration (change from legacy `mcontrol` to `mcontrol6`) | [#1400](https://github.com/stnolting/neorv32/pull/1400) |
| 06.10.2025 | 1.12.3.2 | :bug: fix `clz` and `ctz` instructions; bug introduced in #1395 (v1.12.2.9) | [#1397](https://github.com/stnolting/neorv32/pull/1397) |
| 05.10.2025 | 1.12.3.1 | rework debug module (DM) - slightly faster response, less hardware utilization, cleaner code | [#1396](https://github.com/stnolting/neorv32/pull/1396) |
| 05.10.2025 | [**1.12.3**](https://github.com/stnolting/neorv32/releases/tag/v1.12.3) | :rocket: **New release** | |
| 04.10.2025 | 1.12.2.9 | minor rtl edits and cleanups | [#1395](https://github.com/stnolting/neorv32/pull/1395) |
| 04.10.2025 | 1.12.2.8 | :bug: fix broken DTM bypass register; further DTM logic optimization | [#1393](https://github.com/stnolting/neorv32/pull/1393) |
| 03.10.2025 | 1.12.2.7 | :bug: FPU: code cleanups and minor bug fix in "float to int" conversion (incorrect sign for if negative overflow) | [#1392](https://github.com/stnolting/neorv32/pull/1392) |
| 03.10.2025 | 1.12.2.6 | add generic multiplier primitive (used by `M` and `Zicsr` ALU co-processors) | [#1391](https://github.com/stnolting/neorv32/pull/1391) |
| 03.10.2025 | 1.12.2.5 | :warning: rename and rework openOCD scripts ; optimize debug transfer module (DTM) | [#1390](https://github.com/stnolting/neorv32/pull/1390) |
| 30.09.2025 | 1.12.2.4 | minor rtl cleanups and optimizations | [#1389](https://github.com/stnolting/neorv32/pull/1389) |
| 20.09.2025 | 1.12.2.3 | :test_tube: extend CPU trace port to support (subsets) of **RVVI** and **RVFI** | [#1385](https://github.com/stnolting/neorv32/pull/1385) |
| 19.09.2025 | 1.12.2.2 | rework CPU trace port and trace buffer module (TRACER) | [#1384](https://github.com/stnolting/neorv32/pull/1384) |
| 18.09.2025 | 1.12.2.1 | re-add `XBUS_TIMEOUT` configuration option (timeout window for processor-external bus accesses) | [#1383](https://github.com/stnolting/neorv32/pull/1383) |
| 18.09.2025 | [**1.12.2**](https://github.com/stnolting/neorv32/releases/tag/v1.12.2) | :rocket: **New release** | |
| 17.09.2025 | 1.12.1.9 | minor CPU logic optimizations | [#1381](https://github.com/stnolting/neorv32/pull/1381) |
| 14.09.2025 | 1.12.1.8 | :warning: remove CFU CSRs (`cfureg[0..3]`) | [#1377](https://github.com/stnolting/neorv32/pull/1377) |
| 13.09.2025 | 1.12.1.7 | :bug: fix unaligned instruction fetch bus error; do not trigger co-processors if pending instruction-related exception | [#1367](https://github.com/stnolting/neorv32/pull/1376) |
| 13.09.2025 | 1.12.1.6 | minor RTL edits; add `Zca` ISA extension flag to `mxisa` CSR | [#1375](https://github.com/stnolting/neorv32/pull/1375) |
| 09.09.2025 | 1.12.1.5 | TRACER: rework instruction decoding logic and add all remaining ISA extensions | [#1368](https://github.com/stnolting/neorv32/pull/1368) |
| 05.09.2025 | 1.12.1.4 | improve TRACER's simulation-mode instruction decoding | [#1366](https://github.com/stnolting/neorv32/pull/1366) |
| 05.09.2025 | 1.12.1.3 | minor rtl edits: add `ndmresetpending` & `stickyunavail` bits to DM's `dmstatus` register; fix interrupt-entry after `wfi` | [#1364](https://github.com/stnolting/neorv32/pull/1364) |
| 04.09.2025 | 1.12.1.2 | :bug: fix debug module's `command.transfer` bit logic (ignore `aarsize` & `regno` if `transfer=0`) | [#1363](https://github.com/stnolting/neorv32/pull/1363) |
| 03.09.2025 | 1.12.1.1 | rework bootloader; add SD card boot option; :warning: change executable image signature and checksum | [#1361](https://github.com/stnolting/neorv32/pull/1361) |
| 29.08.2025 | [**1.12.1**](https://github.com/stnolting/neorv32/releases/tag/v1.12.1) | :rocket: **New release** | |
| 28.08.2025 | 1.12.0.10 | fix minor RISC-V incompatibilities (reset `mstatus.mpp` to all-zero; AMO memory faults have to raise store exceptions) | [#1360](https://github.com/stnolting/neorv32/pull/1360) |
| 28.08.2025 | 1.12.0.9 | :warning: rework TWD module due do to several minor design flaws | [#1359](https://github.com/stnolting/neorv32/pull/1359) |
| 27.08.2025 | 1.12.0.8 | add two new write-only flags to the SDI control register to clear the RX and TX FIFOs | [#1358](https://github.com/stnolting/neorv32/pull/1358) |
| 27.08.2025 | 1.12.0.7 | minor rtl code-cleanups and optimizations | [#1357](https://github.com/stnolting/neorv32/pull/1357) |
| 24.08.2025 | 1.12.0.6 | simplify ROM images (VHDL packages for IMEM/BOOTROM) | [#1355](https://github.com/stnolting/neorv32/pull/1355) |
| 23.08.2025 | 1.12.0.5 | :warning: simplify UART and SDI modules; remove "at least half full" FIFO status flags and according interrupts | [#1354](https://github.com/stnolting/neorv32/pull/1354) |
| 23.08.2025 | 1.12.0.4 | :warning: simplify SLINK, SPI and NEOLED modules; remove "at least half full" FIFO status flags and according interrupts | [#1353](https://github.com/stnolting/neorv32/pull/1353) |
| 22.08.2025 | 1.12.0.3 | :bug: fix output gating of simulation memory model; minor rtl cleanups | [#1352](https://github.com/stnolting/neorv32/pull/1352) |
| 21.08.2025 | 1.12.0.2 | add all-new optimized FIFO primitive | [#1349](https://github.com/stnolting/neorv32/pull/1349) |
| 20.08.2025 | 1.12.0.1 | :sparkles: add generic RAM primitives that are used by all modules (cache, IMEM/DMEM, register file) | [#1347](https://github.com/stnolting/neorv32/pull/1347) |
| 20.08.2025 | [**1.12.0**](https://github.com/stnolting/neorv32/releases/tag/v1.12.0) | :rocket: **New release** | |
| 20.08.2025 | 1.11.9.9 | :bug: fix DMA's byte-enable signal generation during byte-accesses | [#1346](https://github.com/stnolting/neorv32/pull/1346) |
| 19.08.2025 | 1.11.9.8 | simplify CPU front-end's IPB; code cleanups and logic optimization | [#1345](https://github.com/stnolting/neorv32/pull/1345) |
| 17.08.2025 | 1.11.9.7 | replace IMEM and DMEM RTL modules by a generic memory component | [#1344](https://github.com/stnolting/neorv32/pull/1344) |
| 16.08.2025 | 1.11.9.6 | :warning: rework layout of `SYSINFO.MISC` information register | [#1342](https://github.com/stnolting/neorv32/pull/1342) |
| 16.08.2025 | 1.11.9.5 | rework bus access error logic; add per-word cache status bits; :warning: remove top's `XBUS_TIMEOUT` generic; extend _global_ bus timeout to 1024 cycles | [#1339](https://github.com/stnolting/neorv32/pull/1339) |
| 15.08.2025 | 1.11.9.4 | :lock: add new CPU tuning option: constant-time branches | [#1338](https://github.com/stnolting/neorv32/pull/1338) |
| 12.08.2025 | 1.11.9.3 | add NEORV32-specific "machine control and status register" `mxcsr`; :warning: move tuning options flags from `mxisa` to `mxcsr` | [#1355](https://github.com/stnolting/neorv32/pull/1335) |
| 09.08.2025 | 1.11.9.2 | minor fixes and optimizations | [#1333](https://github.com/stnolting/neorv32/pull/1333) |
| 08.08.2025 | 1.11.9.1 | :warning: remove double-trap exception | [#1332](https://github.com/stnolting/neorv32/pull/1332) |
| 03.08.2025 | [**1.11.9**](https://github.com/stnolting/neorv32/releases/tag/v1.11.9) | :rocket: **New release** | |
| 03.08.2025 | 1.11.8.9 | minor rtl edits and cleanups | [#1331](https://github.com/stnolting/neorv32/pull/1331) |
| 28.07.2025 | 1.11.8.8 | minor rtl edits and cleanups | [#1325](https://github.com/stnolting/neorv32/pull/1325) |
| 25.07.2025 | 1.11.8.7 | make cache burst support optional | [#1324](https://github.com/stnolting/neorv32/pull/1324) |
| 20.07.2025 | 1.11.8.6 | extend tracer simulation log; refine semihosting file (write) access; minor rtl edits | [#1322](https://github.com/stnolting/neorv32/pull/1322) |
| 18.07.2025 | 1.11.8.5 | :sparkles: add support for the RISC-V `Zcb` ISA extension (further code size reduction instructions; "add-on" for the `C` ISA extension) | [#1320](https://github.com/stnolting/neorv32/pull/1320) |
| 18.07.2025 | 1.11.8.4 | :sparkles: TRACER: write full trace log to file (simulation-only) | [#1318](https://github.com/stnolting/neorv32/pull/1318) |
| 14.07.2025 | 1.11.8.3 | cleanup UART simulation logging | [#1314](https://github.com/stnolting/neorv32/pull/1314) |
| 12.07.2025 | 1.11.8.2 | :sparkles: add new module: execution trace buffer (TRACER) | [#1313](https://github.com/stnolting/neorv32/pull/1313) |
| 11.07.2025 | 1.11.8.1 | :bug: fix bug in double-trap monitoring logic | [#1312](https://github.com/stnolting/neorv32/pull/1312) |
| 10.07.2025 | [**1.11.8**](https://github.com/stnolting/neorv32/releases/tag/v1.11.8) | :rocket: **New release** | |
| 09.07.2025 | 1.11.7.9 | :bug: fix minimal cache block size (has to be at least 8 bytes / 2 words) | [#1310](https://github.com/stnolting/neorv32/pull/1310) |
| 08.07.2025 | 1.11.7.8 | :warning: remove top `HART_BASE` generic | [#1308](https://github.com/stnolting/neorv32/pull/1308) |
| 07.07.2025 | 1.11.7.7 | minor rtl edits and cleanups | [#1307](https://github.com/stnolting/neorv32/pull/1307) |
| 06.07.2025 | 1.11.7.6 | :sparkles: add configurable number of HW triggers (break-/watchpoints) | [#1304](https://github.com/stnolting/neorv32/pull/1304) |
| 06.07.2025 | 1.11.7.5 | :sparkles: OCD: add support for hardware-assisted watchpoints | [#1303](https://github.com/stnolting/neorv32/pull/1303) |
| 04.07.2025 | 1.11.7.4 | minor rtl edits and cleanups | [#1302](https://github.com/stnolting/neorv32/pull/1302) |
| 27.06.2025 | 1.11.7.3 | RTE cleanups; double-trap exception now has highest priority | [#1299](https://github.com/stnolting/neorv32/pull/1299) |
| 21.06.2025 | 1.11.7.2 | :test_tube: add double-trap exception (loosely based on the RISC-V `Smdbltrp` ISA extension) | [#1294](https://github.com/stnolting/neorv32/pull/1294) |
| 20.06.2025 | 1.11.7.1 | remove WDT's "strict" configuration bit; minor rtl cleanups | [#1293](https://github.com/stnolting/neorv32/pull/1293) |
| 20.06.2025 | [**1.11.7**](https://github.com/stnolting/neorv32/releases/tag/v1.11.7) | :rocket: **New release** | |
| 18.06.2025 | 1.11.6.8 | minor rtl edits and optimizations | [#1291](https://github.com/stnolting/neorv32/pull/1291) |
| 08.06.2025 | 1.11.6.7 | :warning: combine individual UART RX/TX interrupt requests into a single (programmable) interrupt request | [#1289](https://github.com/stnolting/neorv32/pull/1289) |
| 08.06.2025 | 1.11.6.6 | :warning: invert "TX FIFO full" status flag; add interrupt option for "TX FIFO not full" status | [#1288](https://github.com/stnolting/neorv32/pull/1288) |
| 07.06.2025 | 1.11.6.5 | :sparkles: add TRNG interrupt | [#1287](https://github.com/stnolting/neorv32/pull/1287) |
| 07.06.2025 | 1.11.6.4 | :warning: combine individual SLINK RX/TX interrupt requests into a single (programmable) interrupt request | [#1286](https://github.com/stnolting/neorv32/pull/1286) |
| 06.06.2025 | 1.11.6.3 | :sparkles: :test_tube: add semihosting support for the on-chip debugger | [#1285](https://github.com/stnolting/neorv32/pull/1285) |
| 06.06.2025 | 1.11.6.2 | upgrade neoTRNG to version 3.3 | [#1284](https://github.com/stnolting/neorv32/pull/1284) |
| 06.06.2025 | 1.11.6.1 | minor rtl optimizations and cleanups | [#1283](https://github.com/stnolting/neorv32/pull/1283) |
| 02.06.2025 | [**1.11.6**](https://github.com/stnolting/neorv32/releases/tag/v1.11.6) | :rocket: **New release** | |
| 01.06.2025 | 1.11.5.9 | :test_tube: add optional output registers for IMEM & DMEM to improve FPGA mapping/timing results | [#1281](https://github.com/stnolting/neorv32/pull/1281) |
| 31.05.2025 | 1.11.5.8 | :warning: rename IMEM/DMEM configuration generics | [#1280](https://github.com/stnolting/neorv32/pull/1280) |
| 31.05.2025 | 1.11.5.7 | :test_tube: rework DMA controller | [#1279](https://github.com/stnolting/neorv32/pull/1279) |
| 31.05.2025 | 1.11.5.6 | rework instruction exception logic; fix: no execution of instruction words that returned as bus access exception | [#1278](https://github.com/stnolting/neorv32/pull/1278) |
| 31.05.2025 | 1.11.5.5 | rework IMEM/DMEM RAM HDL style to prevent DRC errors on Vivado 24.1 when cascading many BRAM blocks | [#1277](https://github.com/stnolting/neorv32/pull/1277) |
| 26.05.2025 | 1.11.5.4 | :warning: remove cyclic redundancy check unit (CRC) | [#1275](https://github.com/stnolting/neorv32/pull/1275) |
| 24.05.2025 | 1.11.5.3 | :warning: rework CFS IO conduits; remove CFS generics | [#1274](https://github.com/stnolting/neorv32/pull/1274) |
| 23.05.2025 | 1.11.5.2 | minor rtl edits and cleanups | [#1273](https://github.com/stnolting/neorv32/pull/1273) |
| 22.05.2025 | 1.11.5.1 | :bug: fix instruction fetch `ben`/`stb` signaling (has to be all-one -> always request full 32-bit words) | [#1272](https://github.com/stnolting/neorv32/pull/1272) |
| 22.05.2025 | [**1.11.5**](https://github.com/stnolting/neorv32/releases/tag/v1.11.5) | :rocket: **New release** | |
| 17.05.2025 | 1.11.4.9 | :bug: fix CPU's `lock` being cleared too early during atomic read-modify-write operations; :bug: fix cache's `ben` signal generation | [#1270](https://github.com/stnolting/neorv32/pull/1270) |
| 16.05.2025 | 1.11.4.8 | :warning: remove hardware spinlocks and CPU's inter-core communication links | [#1268](https://github.com/stnolting/neorv32/pull/1268) |
| 16.05.2025 | 1.11.4.7 | :warning: make `mcause` CSR read-only | [#1267](https://github.com/stnolting/neorv32/pull/1267) |
| 12.05.2025 | 1.11.4.6 | :bug: fix missing burst signal in bus register stage (introduced in previous version / v1.11.4.5) | [#1266](https://github.com/stnolting/neorv32/pull/1266) |
| 12.05.2025 | 1.11.4.5 | add explicit "burst" signal to processor-internal bus; clean-up CPU "fence" decoding | [#1265](https://github.com/stnolting/neorv32/pull/1265) |
| 10.05.2025 | 1.11.4.4 | :sparkles: add cache burst transfers (read-only) | [#1263](https://github.com/stnolting/neorv32/pull/1263) |
| 10.05.2025 | 1.11.4.3 | minor edits and optimizations | [#1262](https://github.com/stnolting/neorv32/pull/1262) |
| 09.05.2025 | 1.11.4.2 | rework locking of processor-internal bus; bus locking is now implemented for the entire bus infrastructure | [#1260](https://github.com/stnolting/neorv32/pull/1260) |
| 04.05.2025 | 1.11.4.1 | rework I/D-cache architecture: switch from "write-allocate & write-back" to "write-through" | [#1259](https://github.com/stnolting/neorv32/pull/1259) |
| 04.05.2025 | [**1.11.4**](https://github.com/stnolting/neorv32/releases/tag/v1.11.4) | :rocket: **New release** | |
| 03.05.2025 | 1.11.3.10 | :warning: rework cache configuration options | [#1257](https://github.com/stnolting/neorv32/pull/1257) |
| 03.05.2025 | 1.11.3.9 | :warning: remove external bus interface cache (xbus-cache) | [#1256](https://github.com/stnolting/neorv32/pull/1256) |
| 02.05.2025 | 1.11.3.8 | add full-scale AXI4 bridge; :bug: fix XBUS/Wishbone conversion bug (from v1.11.3.7 / #1252) | [#1253](https://github.com/stnolting/neorv32/pull/1253) |
| 01.05.2025 | 1.11.3.7 | :warning: rework processor-internal bus protocol | [#1252](https://github.com/stnolting/neorv32/pull/1252) |
| 28.04.2025 | 1.11.3.6 | :warning: update/rework newlib system calls | [#1249](https://github.com/stnolting/neorv32/pull/1249) |
| 28.04.2025 | 1.11.3.5 | optimize cache block replacement logic and block transfers | [#1248](https://github.com/stnolting/neorv32/pull/1248) |
| 26.04.2025 | 1.11.3.4 | :sparkles: add bus lock feature | [#1245](https://github.com/stnolting/neorv32/pull/1245) |
| 26.04.2025 | 1.11.3.3 | optimize round-robin bus switch: remove idle cycles | [#1244](https://github.com/stnolting/neorv32/pull/1244) |
| 22.04.2025 | 1.11.3.2 | :bug: fix the privilege level with which the bootloader boots an application image | [#1241](https://github.com/stnolting/neorv32/pull/1241) |
| 22.04.2025 | 1.11.3.1 | add new top generic (`OCD_HW_BREAKPOINT`) to enable/disable OCD's hardware trigger; :warning: hardwire `tdata1.dmode` to `1` - only debug-mode can use the trigger module; hardwire `tdata1.action` to `0001` - debug-mode entry only | [#1239](https://github.com/stnolting/neorv32/pull/1239) |
| 21.04.2025 | [**1.11.3**](https://github.com/stnolting/neorv32/releases/tag/v1.11.3) | :rocket: **New release** | |
| 18.04.2025 | 1.11.2.9 | adjust TWI timing to allow for repeated-start at higher TWI clock speeds | [#1237](https://github.com/stnolting/neorv32/pull/1237) |
| 18.04.2025 | 1.11.2.8 | :bug: fix bug in PMP logic: multiple signal assignments when NAPOT-mode is disabled | [#1236](https://github.com/stnolting/neorv32/pull/1236) |
| 12.04.2025 | 1.11.2.7 | :sparkles: add PWM polarity configuration | [#1230](https://github.com/stnolting/neorv32/pull/1230) |
| 07.04.2025 | 1.11.2.6 | :bug: fix SDI input synchronization | [#1227](https://github.com/stnolting/neorv32/pull/1227) |
| 05.04.2025 | 1.11.2.5 | minor rtl edits and optimizations | [#1225](https://github.com/stnolting/neorv32/pull/1225) |
| 01.04.2025 | 1.11.2.4 | :bug: fix bug in PWM clock prescaler | [#1222](https://github.com/stnolting/neorv32/pull/1222) |
| 29.03.2025 | 1.11.2.3 | :sparkles: add optional 32 hardware spinlocks (`HWSPINLOCK` module) | [#1220](https://github.com/stnolting/neorv32/pull/1220) |
| 24.03.2025 | 1.11.2.2 | TWD: add separate RX/TX FIFO configuration; add dummy response (if TX FIFO is empty); add optional no-ACK on read access if TX FIFO is empty | [#1210](https://github.com/stnolting/neorv32/pull/1210) |
| 21.03.2025 | 1.11.2.1 | :warning: remove clock gating option | [#1214](https://github.com/stnolting/neorv32/pull/1214) |
| 15.03.2025 | [**1.11.2**](https://github.com/stnolting/neorv32/releases/tag/v1.11.2) | :rocket: **New release** | |
| 14.03.2025 | 1.11.1.9 | :bug: fix broken shift instructions of `Zbb` ISA extension (when `Zbkc` and `CPU_FAST_SHIFT_EN` are both disabled) | [#1206](https://github.com/stnolting/neorv32/pull/1206) |
| 02.03.2025 | 1.11.1.8 | :warning: remove DMA FIRQ-triggered auto mode; :warning: remove GPTMR mode configuration bit | [#1194](https://github.com/stnolting/neorv32/pull/1194) |
| 01.03.2025 | 1.11.1.7 | minor rtl / coding style edits (fixing a Vivado 2024.2 synthesis issue) | [#1193](https://github.com/stnolting/neorv32/pull/1193) |
| 23.02.2025 | 1.11.1.6 | source-out CPU counters into a new rtl file (`neorv32_cpu_counters.vhd`) | [#1192](https://github.com/stnolting/neorv32/pull/1192) |
| 22.02.2025 | 1.11.1.5 | minor rtl edits and cleanups | [#1191](https://github.com/stnolting/neorv32/pull/1191) |
| 20.02.2025 | 1.11.1.4 | :bug: fix bug in `Zalrsc` ISA extension's bus request decoding | [#1190](https://github.com/stnolting/neorv32/pull/1190) |
| 14.02.2025 | 1.11.1.3 | source-out CPU front-end into new rtl file (`neorv32_cpu_frontend.vhd`) | [#1183](https://github.com/stnolting/neorv32/pull/1183) |
| 14.02.2025 | 1.11.1.2 | minor rtl edits and cleanups (cache optimizations) | [#1182](https://github.com/stnolting/neorv32/pull/1182) |
| 08.02.2025 | 1.11.1.1 | :sparkles: add support for `A` and `Zalrsc` ISA extensions | [#1181](https://github.com/stnolting/neorv32/pull/1181) |
| 07.02.2025 | [**1.11.1**](https://github.com/stnolting/neorv32/releases/tag/v1.11.1) | :rocket: **New release** | |
| 07.02.2025 | 1.11.0.10 | :warning: rename UART RTS/CTS signals | [#1180](https://github.com/stnolting/neorv32/pull/1180) |
| 07.02.2025 | 1.11.0.9 | minor rtl edits and cleanups | [#1179](https://github.com/stnolting/neorv32/pull/1179) |
| 03.02.2025 | 1.11.0.8 | :sparkles: add explicit memory ordering/coherence support; :warning: remove WDT "halt-on-debug" and "halt-on-sleep" options; :bug: rework cache module fixing several (minor?) design flaws | [#1176](https://github.com/stnolting/neorv32/pull/1176) |
| 03.02.2025 | 1.11.0.7 | :bug: add missing CFS clock gen enable signal | [#1177](https://github.com/stnolting/neorv32/pull/1177) |
| 01.02.2025 | 1.11.0.6 | :warning: remove XIP module | [#1175](https://github.com/stnolting/neorv32/pull/1175) |
| 01.02.2025 | 1.11.0.5 | minor rtl optimizations and cleanups; :warning: remove DMA "fence" feature | [#1174](https://github.com/stnolting/neorv32/pull/1174) |
| 28.01.2025 | 1.11.0.4 | :bug: fix crt0's entry address being overridden by core0's constructors (that do not backup any registers) | [#1172](https://github.com/stnolting/neorv32/pull/1172) |
| 28.01.2025 | 1.11.0.3 | :bug: fix BOOTROM addressing (index was out-of-range) | [#1171](https://github.com/stnolting/neorv32/pull/1171) |
| 24.01.2025 | 1.11.0.2 | :warning: rename JEDEC ID generic; minor rtl edits and optimizations | [#1168](https://github.com/stnolting/neorv32/pull/1168) |
| 23.01.2025 | 1.11.0.1 | reset SDA and SCL of TWI and TWD modules to `1` | [#1167](https://github.com/stnolting/neorv32/pull/1167) |
| 22.01.2025 | [**1.11.0**](https://github.com/stnolting/neorv32/releases/tag/v1.11.0) | :rocket: **New release** | |
| 22.01.2025 | 1.10.9.10 | :bug: fix TWD ACK/NACK sampling | [#1165](https://github.com/stnolting/neorv32/pull/1165) |
| 18.01.2025 | 1.10.9.9 | atomic memory access updates and improvements | [#1163](https://github.com/stnolting/neorv32/pull/1163) |
| 16.01.2025 | 1.10.9.8 | :bug: fix several TWD design flaws | [#1161](https://github.com/stnolting/neorv32/pull/1161) |
| 15.01.2025 | 1.10.9.7 | :sparkles: add GPIO interrupt(s); :warning: remove XIRQ controller, constrain GPIO input/output ports from 64-bit to 32-bit | [#1159](https://github.com/stnolting/neorv32/pull/1159) |
| 13.01.2025 | 1.10.9.6 | add WDT and OCD rest outputs to top module | [#1152](https://github.com/stnolting/neorv32/pull/1152) |
| 11.01.2025 | 1.10.9.5 | minor rtl cleanups; :bug: fix minor bug (multiple drivers on ICC nets; introduced in version 1.10.9.2) | [#1151](https://github.com/stnolting/neorv32/pull/1151) |
| 11.01.2025 | 1.10.9.4 | :warning: RTE: use a single, global trap handler table that applies to _both_ cores | [#1150](https://github.com/stnolting/neorv32/pull/1150) |
| 10.01.2025 | 1.10.9.3 | split functional behavior of `fence` and `fence.i` instructions | [#1149](https://github.com/stnolting/neorv32/pull/1149) |
| 10.01.2025 | 1.10.9.2 | clean-up SMP dual-core configuration (HW and SW optimizations) | [#1146](https://github.com/stnolting/neorv32/pull/1146) |
| 09.01.2025 | 1.10.9.1 | fix side-effects of CSR read instructions | [#1145](https://github.com/stnolting/neorv32/pull/1145) |
| 08.01.2025 | [**1.10.9**](https://github.com/stnolting/neorv32/releases/tag/v1.10.9) | :rocket: **New release** | |
| 07.01.2025 | 1.10.8.9 | rtl edits and cleanups; add dedicated "core complex" wrapper (CPU + L1 caches + bus switch) | [#1144](https://github.com/stnolting/neorv32/pull/1144) |
| 04.01.2025 | 1.10.8.8 | :sparkles: add inter-core communication (ICC) for the SMP dual-core setup | [#1142](https://github.com/stnolting/neorv32/pull/1142) |
| 03.01.2025 | 1.10.8.7 | :warning: :sparkles: replace `Zalrsc` ISA extensions (reservation-set operations) by `Zaamo` ISA extension (atomic read-modify-write operations) | [#1141](https://github.com/stnolting/neorv32/pull/1141) |
| 01.01.2025 | 1.10.8.6 | :sparkles: :test_tube: add smp dual-core option | [#1135](https://github.com/stnolting/neorv32/pull/1135) |
| 29.12.2024 | 1.10.8.5 | :test_tube: add multi-hart support to debug module | [#1132](https://github.com/stnolting/neorv32/pull/1132) |
| 29.12.2024 | 1.10.8.4 | :warning: rename `SYSINFO.MEM -> SYSINFO.MISC`; add new `SYSINFO.MISC` entry for number of CPU cores (hardwired to one) | [#1134](https://github.com/stnolting/neorv32/pull/1134) |
| 29.12.2024 | 1.10.8.3 | :bug: fix incorrect HPM counter sizes if `HPM_CNT_WIDTH = 64` | [#1128](https://github.com/stnolting/neorv32/pull/1128) |
| 27.12.2024 | 1.10.8.2 | add out-of-band signals to internal request bus | [#1131](https://github.com/stnolting/neorv32/pull/1131) |
| 27.12.2024 | 1.10.8.1 | :warning: replace MTIME by CLINT; :warning: remove `HART_ID` generic | [#1130](https://github.com/stnolting/neorv32/pull/1130) |
| 26.12.2024 | [**1.10.8**](https://github.com/stnolting/neorv32/releases/tag/v1.10.8) | :rocket: **New release** | |
| 23.12.2024 | 1.10.7.9 | :warning: rework IO/peripheral address space; :sparkles: increase device size from 256 bytes to 64kB | [#1126](https://github.com/stnolting/neorv32/pull/1126) |
| 22.12.2024 | 1.10.7.8 | :warning: rename CPU tuning options / generics | [#1125](https://github.com/stnolting/neorv32/pull/1125) |
| 22.12.2024 | 1.10.7.7 | :warning: move clock gating switch from processor top to CPU clock; `CLOCK_GATING_EN` is now a CPU tuning option | [#1124](https://github.com/stnolting/neorv32/pull/1124) |
| 21.12.2024 | 1.10.7.6 | minor rtl cleanups and optimizations | [#1123](https://github.com/stnolting/neorv32/pull/1123) |
| 19.12.2024 | 1.10.7.5 | :test_tube: use time-multiplex PMP architecture (reducing area footprint) | [#1105](https://github.com/stnolting/neorv32/pull/1105) |
| 14.12.2024 | 1.10.7.4 | :sparkles: add new module: I2C-compatible **Two-Wire Device Controller (TWD)** | [#1121](https://github.com/stnolting/neorv32/pull/1121) |
| 14.12.2024 | 1.10.7.3 | :warning: rework TRNG (change HAL; remove interrupt) | [#1120](https://github.com/stnolting/neorv32/pull/1120) |
| 12.12.2024 | 1.10.7.2 | add external memory configuration/initialization options to testbench | [#1119](https://github.com/stnolting/neorv32/pull/1119) |
| 11.12.2024 | 1.10.7.1 | :test_tube: shrink bootloader's minimal ISA (`rv32e`) and RAM (256 bytes) requirements | [#1118](https://github.com/stnolting/neorv32/pull/1118) |
| 10.12.2024 | [**1.10.7**](https://github.com/stnolting/neorv32/releases/tag/v1.10.7) | :rocket: **New release** | |
| 03.12.2024 | 1.10.6.9 | :sparkles: add ONEWIRE command and data FIFO; :warning: rework ONEWIRE interface register layout; :bug: fix regression: busy flag was stuck at zero | [#1113](https://github.com/stnolting/neorv32/pull/1113) |
| 01.12.2024 | 1.10.6.8 | add TWI bus sensing logic | [#1111](https://github.com/stnolting/neorv32/pull/1111) |
| 26.11.2024 | 1.10.6.7 | :bug: fix some HDL issues that caused problems when auto-converting to Verilog | [#1103](https://github.com/stnolting/neorv32/pull/1103) |
| 23.11.2024 | 1.10.6.6 | CPU control: large code edits and cleanups | [#1099](https://github.com/stnolting/neorv32/pull/1099) |
| 10.11.2024 | 1.10.6.5 | :warning: switch to [xPack](https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack) as default prebuilt RISC-V GCC toolchain (now using `riscv-none-elf-` as default gcc prefix) | [#1091](https://github.com/stnolting/neorv32/pull/1091) |
| 10.11.2024 | 1.10.6.4 | rework default processor testbench | [#1093](https://github.com/stnolting/neorv32/pull/1093) |
| 06.11.2024 | 1.10.6.3 | minor rtl edits and cleanups | [#1090](https://github.com/stnolting/neorv32/pull/1090) |
| 02.11.2024 | 1.10.6.2 | :warning: rework processor boot configuration; add new boot-configuration generics | [#1086](https://github.com/stnolting/neorv32/pull/1086) |
| 01.11.2024 | 1.10.6.1 | :test_tube: convert VHDL memory images into full-scale VHDL packages | [#1084](https://github.com/stnolting/neorv32/pull/1084) |
| 26.10.2024 | [**1.10.6**](https://github.com/stnolting/neorv32/releases/tag/v1.10.6) | :rocket: **New release** | |
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
| 01.10.2024 | [**1.10.5**](https://github.com/stnolting/neorv32/releases/tag/v1.10.5) | :rocket: **New release** | |
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
| 16.09.2024 | [**1.10.4**](https://github.com/stnolting/neorv32/releases/tag/v1.10.4) | :rocket: **New release** | |
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
| 03.09.2024 | [**1.10.3**](https://github.com/stnolting/neorv32/releases/tag/v1.10.3) | :rocket: **New release** | |
| 30.08.2024 | 1.10.2.9 | :bug: fix PC reset bug (introduced in v1.10.2.8); minor RTL optimizations (size and critical path) | [#998](https://github.com/stnolting/neorv32/pull/998) |
| 25.08.2024 | 1.10.2.8 | :warning: remove user-mode HPM counters; add individual `mocunteren` bits (`CY` and `IR`) rework Vivado IP module; minor RTL cleanups and optimization | [#996](https://github.com/stnolting/neorv32/pull/996) |
| 16.08.2024 | 1.10.2.7 | minor CPU area and critical path optimizations; minor code cleanups | [#990](https://github.com/stnolting/neorv32/pull/990) |
| 09.08.2024 | 1.10.2.6 | :warning: re-organize RTL files; all core files are now located in `rtl/core`; remove `mem` sub-folder | [#985](https://github.com/stnolting/neorv32/pull/985) |
| 09.08.2024 | 1.10.2.5 | minor HDL edits | [#984](https://github.com/stnolting/neorv32/pull/984) |
| 06.08.2024 | 1.10.2.4 | :warning: **Vivado IP module**: constrain minimal ALL input/output size to 1; add explicit PWM controller enable option | [#980](https://github.com/stnolting/neorv32/pull/980) |
| 05.08.2024 | 1.10.2.3 | :bug: fix bug in **Vivado IP module** (error if zero-sized input port is unconnected) | [#978](https://github.com/stnolting/neorv32/pull/978) |
| 04.08.2024 | 1.10.2.2 | :bug: fix bug in **Vivado IP module** (error if AXI port is unconnected) | [#976](https://github.com/stnolting/neorv32/pull/976) |
| 02.08.2024 | 1.10.2.1 | :warning: rework CFU; remove support for R5-type instructions | [#971](https://github.com/stnolting/neorv32/pull/971) |
| 29.07.2024 | [**:1.10.2**](https://github.com/stnolting/neorv32/releases/tag/v1.10.2) | :rocket: **New release** | |
| 28.07.2024 | 1.10.1.9 | make SYSINFO.CLK read/**write** | [#966](https://github.com/stnolting/neorv32/pull/966) |
| 21.07.2024 | 1.10.1.8 | :lock: restrict IO access to privileged (machine-mode) software | [#958](https://github.com/stnolting/neorv32/pull/958) |
| 20.07.2024 | 1.10.1.7 | :bug: fix bug in `sbrk` newlib system call (causing `malloc` to provide infinite memory until heap and stack collide) | [#957](https://github.com/stnolting/neorv32/pull/957) |
| 20.07.2024 | 1.10.1.6 | SDI: remove explicit "RX clear flag"; add new flag to check the current state of the chip-select input | [#955](https://github.com/stnolting/neorv32/pull/955) |
| 19.07.2024 | 1.10.1.5 | :sparkles: add "programmable" chip-select enable/disable functionality to SPI module | [#954](https://github.com/stnolting/neorv32/pull/954) |
| 19.07.2024 | 1.10.1.4 | :bug: fix SDI "TX FIFO full" flag | [#953](https://github.com/stnolting/neorv32/pull/953) |
| 18.07.2024 | 1.10.1.3 | :test_tube: add new generic to disable the SYSINFO module (:warning: for advanced users only that wish to use a CPU-only setup): `IO_DISABLE_SYSINFO` | [#952](https://github.com/stnolting/neorv32/pull/952) |
| 10.07.2024 | 1.10.1.2 | minor rtl edits and cleanups | [#948](https://github.com/stnolting/neorv32/pull/948) |
| 05.07.2024 | 1.10.1.1 | minor rtl cleanups and optimizations | [#941](https://github.com/stnolting/neorv32/pull/941) |
| 04.07.2024 | [**1.10.1**](https://github.com/stnolting/neorv32/releases/tag/v1.10.1) | :rocket: **New release** | |
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
| 16.06.2024 | [**1.10.0**](https://github.com/stnolting/neorv32/releases/tag/v1.10.0) | :rocket: **New release** | |
| 15.06.2024 | 1.9.9.9 | :sparkles: add pre-configured example project for Eclipse IDE | [#926](https://github.com/stnolting/neorv32/pull/926) |
| 14.06.2024 | 1.9.9.8 | minor rtl edits/cleanups; increase bootloader's auto-boot timeout from 8s to 10s | [#925](https://github.com/stnolting/neorv32/pull/925) |
| 07.06.2024 | 1.9.9.7 | :sparkles: re-add TRNG "data available" interrupt | [#922](https://github.com/stnolting/neorv32/pull/922) |
| 31.05.2024 | 1.9.9.6 | add "tag" signal to XBUS to provide additional access information (compatible to the AXI4 _ARPROT_ and _AWPROT_ signals) | [#917](https://github.com/stnolting/neorv32/pull/917) |
| 30.05.2024 | 1.9.9.5 | :bug: fix uncached-vs-cached memory accesses (do not interrupt cache bursts by direct/uncached memory accesses) | [#915](https://github.com/stnolting/neorv32/pull/915) |
| 29.05.2024 | 1.9.9.4 | Vivado IP block: add resizing ports for GPIOs, XIRQs and PWM; split size configuration for GPIO inputs and outputs | [#913](https://github.com/stnolting/neorv32/pull/913) |
| 27.05.2024 | 1.9.9.3 | removed `XIRQ_TRIGGER_*` generics; XIRQ trigger type is now _programmable_ by dedicated configuration registers | [#911](https://github.com/stnolting/neorv32/pull/911) |
| 21.05.2024 | 1.9.9.2 | :sparkles: add SLINK routing information ports (compatible to AXI-stream's `TID` and `TDEST` signals) | [#908](https://github.com/stnolting/neorv32/pull/908) |
| 04.05.2024 | 1.9.9.1 | :sparkles: add NEORV32 as Vivado IP block | [#894](https://github.com/stnolting/neorv32/pull/894) |
| 03.05.2024 | [**1.9.9**](https://github.com/stnolting/neorv32/releases/tag/v1.9.9) | :rocket: **New release** | |
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
| 04.04.2024 | [**1.9.8**](https://github.com/stnolting/neorv32/releases/tag/v1.9.8) | :rocket: **New release** | |
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
| 22.03.2024 | [**1.9.7**](https://github.com/stnolting/neorv32/releases/tag/v1.9.7) | :rocket: **New release** | |
| 18.03.2024 | 1.9.6.9 | :sparkles: update CFU example: now implementing the Extended Tiny Encryption Algorithm (XTEA) | [#855](https://github.com/stnolting/neorv32/pull/855) |
| 16.03.2024 | 1.9.6.8 | rework cache system: L1 + L2 caches, all based on the generic cache component | [#853](https://github.com/stnolting/neorv32/pull/853) |
| 16.03.2024 | 1.9.6.7 | cache optimizations: add read-only option, add option to disable direct/uncached accesses | [#851](https://github.com/stnolting/neorv32/pull/851) |
| 15.03.2024 | 1.9.6.6 | :warning: clean-up configuration generics (remove XBUS endianness configuration; refine JEDED/VENDORID configuration); rearrange SYSINFO.SOC bits | [#850](https://github.com/stnolting/neorv32/pull/850) |
| 14.03.2024 | 1.9.6.5 | :sparkles: add optional external bus interface cache (XCACHE) | [#849](https://github.com/stnolting/neorv32/pull/849) |
| 12.03.2024 | 1.9.6.4 | :warning: :warning: rename external bus/memory interface and according generics ("WISHBONE/MEM_EXT" -> "XBUS"); also rename bus interface ports (`wb_* -> xbus_*`) | [#846](https://github.com/stnolting/neorv32/pull/846) |
| 11.03.2024 | 1.9.6.3 | :warning: remove Wishbone tag signal; minor rtl edits and optimizations | [#845](https://github.com/stnolting/neorv32/pull/845) |
| 10.03.2024 | 1.9.6.2 | minor rtl clean-ups, optimizations and fixes | [#843](https://github.com/stnolting/neorv32/pull/843) |
| 09.03.2024 | 1.9.6.1 | add generic cache module (not used yet) | [#842](https://github.com/stnolting/neorv32/pull/842) |
| 01.03.2024 | [**1.9.6**](https://github.com/stnolting/neorv32/releases/tag/v1.9.6) | :rocket: **New release** | |
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
| 16.02.2024 | [**1.9.5**](https://github.com/stnolting/neorv32/releases/tag/v1.9.5) | :rocket: **New release** | |
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
| 31.01.2024 | [**1.9.4**](https://github.com/stnolting/neorv32/releases/tag/v1.9.4) | :rocket: **New release** | |
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
| 11.01.2024 | [**1.9.3**](https://github.com/stnolting/neorv32/releases/tag/v1.9.3) | :rocket: **New release** | |
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
| 01.12.2023 | [**1.9.2**](https://github.com/stnolting/neorv32/releases/tag/v1.9.2) | :rocket: **New release** | |
| 01.12.2023 | 1.9.1.9 | add `menvcfg[h]` CSRs | [#741](https://github.com/stnolting/neorv32/pull/741) |
| 30.11.2023 | 1.9.1.8 | :sparkles: :bug: upgrade RISC-V hardware trigger module (`Sdtrig` ISA extension) to spec. version v1.0 (fixing several minor bugs) | [#739](https://github.com/stnolting/neorv32/pull/739) |
| 25.11.2023 | 1.9.1.7 | cleanup/update assertions and auto-adjusting of invalid generic configurations | [#738](https://github.com/stnolting/neorv32/pull/738) |
| 25.11.2023 | 1.9.1.6 | :sparkles: add option for "ASIC style" register file that provides a full/dedicated hardware reset | [#736](https://github.com/stnolting/neorv32/pull/736) |
| 23.11.2023 | 1.9.1.5 | clean-up & rework CPU branch logic | [#735](https://github.com/stnolting/neorv32/pull/735) |
| 21.11.2023 | 1.9.1.4 | :bug: fix bug in handling of "misaligned instruction exception" | [#734](https://github.com/stnolting/neorv32/pull/734) |
| 20.11.2023 | 1.9.1.3 | :bug: fix wiring of FPU exception flags | [#733](https://github.com/stnolting/neorv32/pull/733) |
| 18.11.2023 | 1.9.1.2 | add XIP clock divider to fine-tune SPI frequency | [#731](https://github.com/stnolting/neorv32/pull/731) |
| 18.11.2023 | 1.9.1.1 | (re-)add SPI high-speed mode, :bug: fix bug in SPI shift register - introduced in v1.9.0.9 | [#730](https://github.com/stnolting/neorv32/pull/730) |
| 14.11.2023 | [**1.9.1**](https://github.com/stnolting/neorv32/releases/tag/v1.9.1) | :rocket: **New release** | |
| 11.11.2023 | 1.9.0.9 | :test_tube: add full hardware reset for **all** flip flops in CPU/processor | [#724](https://github.com/stnolting/neorv32/pull/724) |
| 09.11.2023 | 1.9.0.8 | minor rtl code cleanups | [#723](https://github.com/stnolting/neorv32/pull/723) |
| 04.11.2023 | 1.9.0.7 | upgrade true random number generator to [neoTRNG version 3](https://github.com/stnolting/neoTRNG) | [#721](https://github.com/stnolting/neorv32/pull/721) |
| 31.10.2023 | 1.9.0.6 | update crt0's early-boot trap handler | [#719](https://github.com/stnolting/neorv32/pull/719) |
| 30.10.2023 | 1.9.0.5 | minor rtl cleanups and code beautification | [#718](https://github.com/stnolting/neorv32/pull/718) |
| 28.10.2023 | 1.9.0.4 | :warning: :sparkles: move FreeRTOS port and demo to a new repository: https://github.com/stnolting/neorv32-freertos | [#716](https://github.com/stnolting/neorv32/pull/716) |
| 18.10.2023 | 1.9.0.3 | :warning: remove top's `CPU_EXTENSION_RISCV_Zifencei` generic - `Zifencei` ISA extension is now always enabled | [#709](https://github.com/stnolting/neorv32/pull/709) |
| 16.10.2023 | 1.9.0.2 | minor CPU control cleanups and optimizations (branch system) | [#707](https://github.com/stnolting/neorv32/pull/707) |
| 13.10.2023 | 1.9.0.1 | update software framework to GCC-13.2.0 | [#705](https://github.com/stnolting/neorv32/pull/705) |
| 13.10.2023 | [**1.9.0**](https://github.com/stnolting/neorv32/releases/tag/v1.9.0) | :rocket: **New release** | |
| 13.10.2023 | 1.8.9.9 | minor hardware edits and optimizations | [#703](https://github.com/stnolting/neorv32/pull/703) |
| 07.10.2023 | 1.8.9.8 | add "transfer done" flag to DMA | [#699](https://github.com/stnolting/neorv32/pull/699) |
| 04.10.2023 | 1.8.9.7 | :warning: rework internal bus protocol | [#697](https://github.com/stnolting/neorv32/pull/697) |
| 29.09.2023 | 1.8.9.6 | optimize PMP logic (reducing area requirements) | [#695](https://github.com/stnolting/neorv32/pull/695) |
| 29.09.2023 | 1.8.9.5 | minor CPU optimizations and code clean-ups | [#694](https://github.com/stnolting/neorv32/pull/694) |
| 23.09.2023 | 1.8.9.4 | :sparkles: added vectored trap handling mode of `mtvec` for reduced latency from IRQ to ISR | [#691](https://github.com/stnolting/neorv32/pull/691)
| 22.09.2023 | 1.8.9.3 | :lock: **watchdog**: add reset password and optional "strict" mode for increased safety | [#692](https://github.com/stnolting/neorv32/pull/692) |
| 15.09.2023 | 1.8.9.2 | :warning: rework CFU CSRs; minor rtl edits | [#690](https://github.com/stnolting/neorv32/pull/690) |
| 11.09.2023 | 1.8.9.1 | minor rtl edits and updates | [#684](https://github.com/stnolting/neorv32/pull/684) |
| 09.09.2023 | [**1.8.9**](https://github.com/stnolting/neorv32/releases/tag/v1.8.9) | :rocket: **New release** | |
| 08.09.2023 | 1.8.8.9 | removed unused `mcontext` and `scontext` CSRs (`Sdtrig` ISA extension); CPU optimizations (area and timing) | [#683](https://github.com/stnolting/neorv32/pull/683) |
| 02.09.2023 | 1.8.8.8 | :sparkles: add option to implement **up to 2^32 CFU-internal user-defined CSRs** (via indirect CSR access) | [#681](https://github.com/stnolting/neorv32/pull/681) |
| 02.09.2023 | 1.8.8.7 | :lock: (re-)add **execution monitor**: raise an exception if a multi-cycle ALU operation does not complete within a bound amount of time | [#680](https://github.com/stnolting/neorv32/pull/680) |
| 01.09.2023 | 1.8.8.6 | minor rtl edits and cleanups | [#679](https://github.com/stnolting/neorv32/pull/679) |
| 30.08.2023 | 1.8.8.5 | remove "branch prediction" logic - core is smaller and _even faster_ without it | [#678](https://github.com/stnolting/neorv32/pull/678) |
| 25.08.2023 | 1.8.8.4 | add new generic to downgrade on-chip debugger's debug module back to spec. version 0.13 (`DM_LEGACY_MODE` generic) | [#677](https://github.com/stnolting/neorv32/pull/677) |
| 23.08.2023 | 1.8.8.3 | :test_tube: add experimental `Smcntrpmf` ISA extension (counter privilege mode filtering; spec. is frozen but not yet ratified); remove unused `menvcfg` CSRs | [#676](https://github.com/stnolting/neorv32/pull/676) |
| 19.08.2023 | 1.8.8.2 | :warning: constrain `mtval` CSR; add support for `mtinst` CSR (trap instruction) | [#674](https://github.com/stnolting/neorv32/pull/674) |
| 19.08.2023 | 1.8.8.1 | :test_tube: update RTE to support easy emulation of instructions; add example program to showcase how to emulate unaligned memory accesses | [#673](https://github.com/stnolting/neorv32/pull/673) |
| 18.08.2023 | [**1.8.8**](https://github.com/stnolting/neorv32/releases/tag/v1.8.8) | :rocket: **New release** | |
| 17.08.2023 | 1.8.7.9 | minor rtl edits and cleanups | [#672](https://github.com/stnolting/neorv32/pull/672) |
| 13.08.2023 | 1.8.7.8 | :warning: constrain/optimize `mtval` and `mcounteren` CSRs | [#671](https://github.com/stnolting/neorv32/pull/671) |
| 12.08.2023 | 1.8.7.7 | remove _unratified_ `Zicond` ISA extension; minor rtl code cleanups and optimizations | [#670](https://github.com/stnolting/neorv32/pull/670) |
| 05.08.2023 | 1.8.7.6 | :bug: fix bug: HPM width configurations below 32 bit fail | [#665](https://github.com/stnolting/neorv32/pull/665) |
| 04.08.2023 | 1.8.7.5 | :warning: major code edits/cleanups and file renaming | [#664](https://github.com/stnolting/neorv32/pull/664) |
| 29.07.2023 | 1.8.7.4 | RTL cleanup and optimizations (less synthesis warnings, less resource requirements) | [#660](https://github.com/stnolting/neorv32/pull/660) |
| 28.07.2023 | 1.8.7.3 | :warning: reworked **SYSINFO** module; clean-up address space layout; clean-up assertion notes | [#659](https://github.com/stnolting/neorv32/pull/659) |
| 27.07.2023 | 1.8.7.2 | :bug: make sure that IMEM/DMEM size is always a power of two | [#658](https://github.com/stnolting/neorv32/pull/658) |
| 27.07.2023 | 1.8.7.1 | :warning: remove `CUSTOM_ID` generic; cleanup and re-layout `NEORV32_SYSINFO.SOC` bits; (:bug:) fix gateway's generics (`positive` -> `natural` as these generics are allowed to be zero) | [#657](https://github.com/stnolting/neorv32/pull/657) |
| 26.07.2023 | [**1.8.7**](https://github.com/stnolting/neorv32/releases/tag/v1.8.7) | :rocket: **New release** | |
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
| 27.06.2023 | [**1.8.6**](https://github.com/stnolting/neorv32/releases/tag/v1.8.6) | :rocket: **New release** | |
| 24.06.2023 | 1.8.5.9 | :test_tube: VHDL code: use entity instantiation instead of component instantiation | [#637](https://github.com/stnolting/neorv32/pull/637) |
| 24.06.2023 | 1.8.5.8 | optimize CPU control logic; closed further invalid instruction word detection holes | [#636](https://github.com/stnolting/neorv32/pull/636) |
| 23.06.2023 | 1.8.5.7 | :warning: remove **buskeeper's status register** | [#635](https://github.com/stnolting/neorv32/pull/635) |
| 17.06.2023 | 1.8.5.6 | :sparkles: add new **Cyclic Redundancy Check module (CRC)** | [#632](https://github.com/stnolting/neorv32/pull/632) |
| 03.06.2023 | 1.8.5.5 | :sparkles: re-add (simplified) **Stream Link Interface (SLINK)** | [#628](https://github.com/stnolting/neorv32/pull/628) |
| 03.06.2023 | 1.8.5.4 | :warning: rearrange bits in **SYSINFO** | [#627](https://github.com/stnolting/neorv32/pull/627) |
| 02.06.2023 | 1.8.5.3 | :bug: executable generation: fix address continuity between `.text` and `.rodata` segments | [#626](https://github.com/stnolting/neorv32/pull/626) |
| 19.05.2023 | 1.8.5.2 | :sparkles: add automatic trigger mode to **DMA** (trigger transfer if a processor-internal peripheral issues an interrupt request) | [#618](https://github.com/stnolting/neorv32/pull/618) |
| 18.05.2023 | 1.8.5.1 | software can now retrieve the configured FIFO size of the **TRNG** | [#616](https://github.com/stnolting/neorv32/pull/616) |
| 18.05.2023 | [**1.8.5**](https://github.com/stnolting/neorv32/releases/tag/v1.8.5) | :rocket: **New release** | |
| 18.05.2023 | 1.8.4.9 | remove `is_simulation` flag from SYSINFO; add programmable interrupt to **TRNG** module | [#615](https://github.com/stnolting/neorv32/pull/615) |
| 12.05.2023 | 1.8.4.8 | `mtval` CSR now provides the address of `ebreak` exceptions (re-added temporarily to pass RISC-V ISA tests) | [#611](https://github.com/stnolting/neorv32/pull/611) |
| 03.05.2023 | 1.8.4.7 | :bug: fix bug in FPU (terminate FPU sub-module operations if an exception has been raised) | [#609](https://github.com/stnolting/neorv32/pull/609) |
| 02.05.2023 | 1.8.4.6 | make SDI FIFO access entirely synchronous; upgrade processor memory modules; update test setup wrappers | [#608](https://github.com/stnolting/neorv32/pull/608) |
| 30.04.2023 | 1.8.4.5 | rework processor-internal bus system | [#607](https://github.com/stnolting/neorv32/pull/607) |
| 27.04.2023 | 1.8.4.4 | minor hardware edits and switching activity optimizations of CPU bus unit | [#605](https://github.com/stnolting/neorv32/pull/605) |
| 25.04.2023 | 1.8.4.3 | :bug: fix bug in **DMA** (corrupted write-back when there are bus wait cycles - e.g. when no caches are implemented) | [#601](https://github.com/stnolting/neorv32/pull/601) |
| 24.04.2023 | 1.8.4.2 | minor rtl edits; shorten critical path of d-cache setup | [#599](https://github.com/stnolting/neorv32/pull/599) |
| 22.04.2023 | 1.8.4.1 | :sparkles: add optional **direct memory access controller (DMA)** | [#593](https://github.com/stnolting/neorv32/pull/593) |
| 21.04.2023 | [**1.8.4**](https://github.com/stnolting/neorv32/releases/tag/v1.8.4) | :rocket: **New release** | |
| 21.04.2023 | 1.8.3.9 | :bug: fix timeout bug in **FPU** normalizer | [#592](https://github.com/stnolting/neorv32/pull/592) |
| 19.04.2023 | 1.8.3.8 | minor processor bus system optimizations and clean-ups | [#591](https://github.com/stnolting/neorv32/pull/591) |
| 15.04.2023 | 1.8.3.7 | :bug: :warning: `wfi` and XIRQ bug fixes; massive RTL code cleanup and optimization of CPU control | [#586](https://github.com/stnolting/neorv32/pull/586) |
| 14.04.2023 | 1.8.3.6 | [UARTs] software can now retrieve the configured RX/TX FIFO sizes from the `DATA` register | [#581](https://github.com/stnolting/neorv32/pull/581) |
| 13.04.2023 | 1.8.3.5 | :bug: fixed bug in FPU control logic (introduced in some earlier clean-up commit); minor code edits and optimizations | [#578](https://github.com/stnolting/neorv32/pull/578) |
| 07.04.2023 | 1.8.3.4 | rtl edits and cleanups | [#571](https://github.com/stnolting/neorv32/pull/571) |
| 05.04.2023 | 1.8.3.3 | update **external interrupt controller (XIRQ)** | [#570](https://github.com/stnolting/neorv32/pull/570) |
| 05.04.2023 | 1.8.3.2 | `time` CSR struggles (again) and logic optimization | [#569](https://github.com/stnolting/neorv32/pull/569) |
| 01.04.2023 | 1.8.3.1 | :sparkles: add full `NA4` and `NAPOT` support to the (now) RISC-V-compatible **physical memory protection (PMP)** | [#566](https://github.com/stnolting/neorv32/pull/566) |
| 31.03.2023 | [**1.8.3**](https://github.com/stnolting/neorv32/releases/tag/v1.8.3) | :rocket: **New release** | |
| 29.03.2023 | 1.8.2.9 | :warning: remove `CPU_EXTENSION_RISCV_Zicsr` generic - `Zicsr` ISA extension is always enabled; optimize bus switch; VHDL code cleanups | [#562](https://github.com/stnolting/neorv32/pull/562) |
| 25.03.2023 | 1.8.2.8 | :test_tube: add configurable data cache (**dCACHE**) | [#560](https://github.com/stnolting/neorv32/pull/560) |
| 24.03.2023 | 1.8.2.7 | :sparkles: add full support of `mcounteren` CSR; cleanup counter and PMP CSRs; i-cache optimization | [#559](https://github.com/stnolting/neorv32/pull/559) |
| 18.03.2023 | 1.8.2.6 | add new generic `JEDEC_ID` (official JEDEC identifier; used for `mvendorid` CSR); further generics cleanups | [#557](https://github.com/stnolting/neorv32/pull/557)
| 17.03.2023 | 1.8.2.5 | add RISC-V `time[h]` CSRs (part of the `Zicntr` ISA extension) | [#556](https://github.com/stnolting/neorv32/pull/556) |
| 17.03.2023 | 1.8.2.4 | re-add VHDL process names | [#555](https://github.com/stnolting/neorv32/pull/555) |
| 15.03.2023 | 1.8.2.3 | rtl reworks, cleanups and optimizations | [#550](https://github.com/stnolting/neorv32/pull/550) |
| 11.03.2023 | 1.8.2.2 | :sparkles: add support for RISC-V `Zicond` ISA extension (conditional operations) | [#546](https://github.com/stnolting/neorv32/pull/546) |
| 10.03.2023 | 1.8.2.1 | rtl code edits, clean-ups and minor optimizations (improve branch prediction) | [#545](https://github.com/stnolting/neorv32/pull/545) |
| 10.03.2023 | [**1.8.2**](https://github.com/stnolting/neorv32/releases/tag/v1.8.2) | :rocket: **New release** | |
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
| 23.02.2023 | [**1.8.1**](https://github.com/stnolting/neorv32/releases/tag/v1.8.1) | :rocket: **New release** | |
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
| 25.01.2023 | [**1.8.0**](https://github.com/stnolting/neorv32/releases/tag/v1.8.0) | :rocket: **New release** | |
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
| 21.12.2022 | [**1.7.9**](https://github.com/stnolting/neorv32/releases/tag/v1.7.9) | :rocket: **New release** | |
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
| 28.11.2022 | [**1.7.8**](https://github.com/stnolting/neorv32/releases/tag/v1.7.8) | :rocket: **New release** | |
| 14.11.2022 | 1.7.7.9 | minor rtl edits and code optimizations | [#442](https://github.com/stnolting/neorv32/pull/442) |
| 05.11.2022 | 1.7.7.8 | minor rtl edits | [#441](https://github.com/stnolting/neorv32/pull/441) |
| 03.11.2022 | 1.7.7.7 | :sparkles: add fine-grained clock configuration for **TWI** module: add fine-grained clock configuration, add clock stretching configuration flag | [#440](https://github.com/stnolting/neorv32/pull/440) |
| 01.11.2022 | 1.7.7.6 | :warning: rework **SPI module** | [#438](https://github.com/stnolting/neorv32/pull/438) |
| 24.10.2022 | 1.7.7.5 | :test_tube: remove weird Quartus latch warnings by modifying VHDL coding style | [#434](https://github.com/stnolting/neorv32/pull/434) |
| 19.10.2022 | 1.7.7.4 | optimize UART's `RTS` (hardware flow control) behavior | [#433](https://github.com/stnolting/neorv32/pull/433) |
| 15.10.2022 | 1.7.7.3 | :bug: fix bug in `is_power_of_two_f` VHDL function (thanks Alan!) | [#428](https://github.com/stnolting/neorv32/pull/428) |
| 12.10.2022 | 1.7.7.2 | add dedicated hardware reset to _all_ CPU counters (`[m]cycle[h]`, `[m]instret[h]`, `[m]hpmcounter[h]`); :sparkles: **all CSRs now provide a dedicated hardware reset** | [#426](https://github.com/stnolting/neorv32/pull/426) |
| 09.10.2022 | 1.7.7.1 | fix Quartus synthesis issue (VHDL): make sure reset state is the _first_ entry in a state list | [#423](https://github.com/stnolting/neorv32/pull/423) |
| 24.09.2022 | [**1.7.7**](https://github.com/stnolting/neorv32/releases/tag/v1.7.7) | :rocket: **New release** | |
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
| 28.08.2022 | [**1.7.6**](https://github.com/stnolting/neorv32/releases/tag/v1.7.6) | :rocket: **New release** | |
| 27.08.2022 | 1.7.5.9 | fix minor core rtl issues that were found while experimenting with a low-level netlist of the processor | [#398](https://github.com/stnolting/neorv32/pull/398) |
| 26.08.2022 | 1.7.5.8 | cleanup **crt0** start-up code: remove setup of `mcountern` and `mcountinhibit` CSRs | [#397](https://github.com/stnolting/neorv32/pull/397) |
| 24.08.2022 | 1.7.5.7 | minor rtl cleanups | [#396](https://github.com/stnolting/neorv32/pull/396) |
| 20.08.2022 | 1.7.5.6 | :sparkles: update software framework to GCC 12.1.0 (new prebuilt toolchains available!) | [#391](https://github.com/stnolting/neorv32/pull/391) |
| 18.08.2022 | 1.7.5.5 | :lock: add **TRNG** read data protection | [#389](https://github.com/stnolting/neorv32/pull/389) |
| 18.08.2022 | 1.7.5.4 | minor rtl cleanup in **PWM** module | [#388](https://github.com/stnolting/neorv32/pull/388) |
| 17.08.2022 | 1.7.5.3 | optimized **CPU front-end** - faster instruction fetch | [#387](https://github.com/stnolting/neorv32/pull/387) |
| 16.08.2022 | 1.7.5.2 | relocate TWI tri-state drivers | [#386](https://github.com/stnolting/neorv32/pull/386) |
| 15.08.2022 | 1.7.5.1 | change base address of **BUSKEEPER** | [#385](https://github.com/stnolting/neorv32/pull/385) |
| 15.08.2022 | [**1.7.5**](https://github.com/stnolting/neorv32/releases/tag/v1.7.5) | :rocket: **New release** | |
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
| 14.07.2022 | [**1.7.4**](https://github.com/stnolting/neorv32/releases/tag/v1.7.4) | :rocket: **New release** | |
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
| 23.06.2022 | [**1.7.3**](https://github.com/stnolting/neorv32/releases/tag/v1.7.3) | :rocket: **New release** | |
| ...        | ...     | Changelog trimmed. See [`CHANGELOG.md` in v1.7.3](https://github.com/stnolting/neorv32/blob/v1.7.3/CHANGELOG.md) for older logs. | ... |
