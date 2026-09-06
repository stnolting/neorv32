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
| 05.09.2026 | 1.13.5.6 | :bug: rework AXI bridge; add data buffering support (to handle back pressure) | [#1645](https://github.com/stnolting/neorv32/pull/1645) |
| 02.09.2026 | 1.13.5.5 | CLINT area optimizations | [#1642](https://github.com/stnolting/neorv32/pull/1642) |
| 30.08.2026 | 1.13.5.4 | ocd, gptmr, pwm: area optimizations; remove reset from registers where it is not strictly necessary | [#1641](https://github.com/stnolting/neorv32/pull/1641) |
| 29.08.2026 | 1.13.5.3 | :test_tube: use asynchronous bus read access for further IO devices | [#1640](https://github.com/stnolting/neorv32/pull/1640) |
| 29.08.2026 | 1.13.5.2 | :test_tube: use asynchronous bus read access for serial IO devices | [#1638](https://github.com/stnolting/neorv32/pull/1638) |
| 29.08.2026 | 1.13.5.1 | CPU: optimize set-on-less-than timing | [#1635](https://github.com/stnolting/neorv32/pull/1635) |
| 22.08.2026 | [**1.13.5**](https://github.com/stnolting/neorv32/releases/tag/v1.13.5) | :rocket: **New release** | |
| 21.08.2026 | 1.13.4.9 | :bug: fix permanent CPU stall when accessing unmapped addresses (if XBUS is disabled) | [#1634](https://github.com/stnolting/neorv32/pull/1634) |
| 21.08.2026 | 1.13.4.8 | convert remaining list arrays to `downto` (named-association aggregates) | [#1632](https://github.com/stnolting/neorv32/pull/1632) |
| 19.08.2026 | 1.13.4.7 | :warning: rework/remove memory/IP wrappers | [#1631](https://github.com/stnolting/neorv32/pull/1631) |
| 17.08.2026 | 1.13.4.6 | CPU: minor timing optimizations and code cleanups | [#1630](https://github.com/stnolting/neorv32/pull/1630) |
| 14.08.2026 | 1.13.4.5 | CPU: generalize fast multiplier pipeline | [#1625](https://github.com/stnolting/neorv32/pull/1625) |
| 14.08.2026 | 1.13.4.4 | minor rtl edits and optimizations | [#1624](https://github.com/stnolting/neorv32/pull/1624) |
| 14.08.2026 | 1.13.4.3 | normalize bus/list array ranges to `x downto y` | [#1622](https://github.com/stnolting/neorv32/pull/1622) |
| 14.08.2026 | 1.13.4.2 | bitmanip: simplify register enables | [#1626](https://github.com/stnolting/neorv32/pull/1626) |
| 12.08.2026 | 1.13.4.1 | declare FIFO/SPRAM memory arrays per generate-branch to fix RAM extraction on Verific-based synthesis tools | [#1623](https://github.com/stnolting/neorv32/pull/1623) |
| 10.08.2026 | [**1.13.4**](https://github.com/stnolting/neorv32/releases/tag/v1.13.4) | :rocket: **New release** | |
| 09.08.2026 | 1.13.3.9 | minor CPU updates, optimizations and RISC-V-compliance fixes | [#1621](https://github.com/stnolting/neorv32/pull/1621) |
| 08.08.2026 | 1.13.3.8 | remove reset from registers where it is not strictly necessary | [#1620](https://github.com/stnolting/neorv32/pull/1620) |
| 02.08.2026 | 1.13.3.7 | `instret` counter: only increment for actually retired instructions | [#1615](https://github.com/stnolting/neorv32/pull/1615) |
| 02.08.2026 | 1.13.3.6 | minor rtl cleanups and optimizations | [#1614](https://github.com/stnolting/neorv32/pull/1614) |
| 02.08.2026 | 1.13.3.5 | CPU register file area optimizations and cleanups | [#1613](https://github.com/stnolting/neorv32/pull/1613) |
| 01.08.2026 | 1.13.3.4 | RVFI: extend `order` to 64-bit as required by the spec | [#1612](https://github.com/stnolting/neorv32/pull/1612) |
| 01.08.2026 | 1.13.3.3 | :test_tube: remove dedicated reset logic from CPU data path FFs | [#1609](https://github.com/stnolting/neorv32/pull/1609) |
| 26.07.2026 | 1.13.3.2 | minor rtl edits and cleanups; :warning: simplify CFS bus interface | [#1606](https://github.com/stnolting/neorv32/pull/1606) |
| 24.07.2026 | 1.13.3.1 | :bug: fix PMP `pmpaddr*[31:30]` bits: must be read-only and zero | [#1605](https://github.com/stnolting/neorv32/pull/1605) |
| 24.07.2026 | [**1.13.3**](https://github.com/stnolting/neorv32/releases/tag/v1.13.3) | :rocket: **New release** | |
| 23.07.2026 | 1.13.2.9 | :sparkles: add optional `CPU_FAST_MUL_REG` generic: pipeline register for the fast multiplier (shortens critical path on narrow-DSP FPGAs; +1 multiply latency cycle) | [#1603](https://github.com/stnolting/neorv32/pull/1603) |
| 20.07.2026 | 1.13.2.8 | comprehensive VHDL coding style edits | [#1602](https://github.com/stnolting/neorv32/pull/1602) |
| 17.07.2026 | 1.13.2.7 | :sparkles: add new SoC module: serial memory controller (SMC) to attach external serial memory (PSRAM or flash; supports XIP) | [#1599](https://github.com/stnolting/neorv32/pull/1599) |
| 11.07.2026 | 1.13.2.6 | :bug: fix another RVFI memory address & data signal alignment bug | [#1597](https://github.com/stnolting/neorv32/pull/1597) |
| 03.07.2026 | 1.13.2.5 | :sparkles: add support for RISC-V `Zcmop` ISA extension (compressed may-be-operations) | [#1596](https://github.com/stnolting/neorv32/pull/1596) |
| 03.07.2026 | 1.13.2.4 | :bug: fix further illegal instruction detection loopholes in `Zfinx` ISA extension (classify + compare)| [#1595](https://github.com/stnolting/neorv32/pull/1595) |
| 03.07.2026 | 1.13.2.3 | minor rtl edits and cleanups | [#1594](https://github.com/stnolting/neorv32/pull/1594) |
| 01.07.2026 | 1.13.2.2 | :bug: fix illegal instruction detection loopholes in `Ziimop` and `Zfinx` ISA extensions | [#1590](https://github.com/stnolting/neorv32/pull/1590) |
| 01.07.2026 | 1.13.2.1 | :bug: fix RVFI memory address & data signal alignment | [#1585](https://github.com/stnolting/neorv32/pull/1585) |
| 16.06.2026 | [**1.13.2**](https://github.com/stnolting/neorv32/releases/tag/v1.13.2) | :rocket: **New release** | |
| 15.06.2026 | 1.13.1.9 | remove `mhpmevent*h` CSRs that do not exist on RV32 without `Sscofpmf` | [#1575](https://github.com/stnolting/neorv32/pull/1575) |
| 14.06.2026 | 1.13.1.8 | close (the last) RVC illegal instruction detection loophole; further shrink bootloader image size; add defaults for CPU-top generics | [#1574](https://github.com/stnolting/neorv32/pull/1574) |
| 07.06.2026 | 1.13.1.7 | minor rtl edits and optimizations | [#1570](https://github.com/stnolting/neorv32/pull/1570) |
| 05.06.2026 | 1.13.1.6 | :warning: rework hardware performance counter (HPM) events | [#1569](https://github.com/stnolting/neorv32/pull/1569) |
| 05.06.2026 | 1.13.1.5 | :bug: minor rtl fixes: fix trace port's RD register signal; fix `Smcntrpmf`'s *cfg CSR address decoding | [#1568](https://github.com/stnolting/neorv32/pull/1568) |
| 04.06.2026 | 1.13.1.4 | minor rtl edits and cleanups | [#1565](https://github.com/stnolting/neorv32/pull/1565) |
| 27.05.2026 | 1.13.1.3 | add option to configure start of uncached address space | [#1561](https://github.com/stnolting/neorv32/pull/1561) |
| 25.05.2026 | 1.13.1.2 | :bug: fix stream link's `rx_ready_o` signal (zero when module is disabled) | [#1558](https://github.com/stnolting/neorv32/pull/1558) |
| 16.05.2026 | 1.13.1.1 | :bug: fix/rework reservation station of `Zalrsc` ISA extension; `sc` can also cause a bus access fault even if the reservation fails | [#1556](https://github.com/stnolting/neorv32/pull/1556) |
| 14.05.2026 | [**1.13.1**](https://github.com/stnolting/neorv32/releases/tag/v1.13.1) | :rocket: **New release** | |
| 11.05.2026 | 1.13.0.9 | trace log: add RVC / compressed-instruction logging and decoding | [#1553](https://github.com/stnolting/neorv32/pull/1553) |
| 08.05.2026 | 1.13.0.8 | :sparkles: add support for RISC-V `time[h]` CSRs | [#1551](https://github.com/stnolting/neorv32/pull/1551) |
| 08.05.2026 | 1.13.0.7 | :sparkles: add support for RISC-V `Zbc` ISA extension (carry-less multiplication) | [#1550](https://github.com/stnolting/neorv32/pull/1550) |
| 08.05.2026 | 1.13.0.6 | :bug: PMP: fix write-only permission config and address/mask storage | [#1549](https://github.com/stnolting/neorv32/pull/1549) |
| 06.05.2026 | 1.13.0.5 | add support for all hardware performance monitors (HPMs); add high-word event-select CSRs (`mhpmevent*h`); add unprivileged/user-mode CSRs shadow copies (`hpmcounter*[h]`) | [#1546](https://github.com/stnolting/neorv32/pull/1546) |
| 03.05.2026 | 1.13.0.4 | rework on-chip debugger (OCD) fixing minor spec incompatibilities | [#1544](https://github.com/stnolting/neorv32/pull/1544) |
| 02.05.2026 | 1.13.0.3 | minor rtl fixes, edits and cleanups (PMP, TWD, bus | [#1543](https://github.com/stnolting/neorv32/pull/1543) |
| 01.05.2026 | 1.13.0.2 | :warning: rework trap CSRs: remove `mtinst`; make `mtval` more verbose; add full WARL access for `mcause` and `mtval` | [#1524](https://github.com/stnolting/neorv32/pull/1542) |
| 29.04.2026 | 1.13.0.1 | rework cache handling of atomic memory operations: enforce memory synchronization by hardware | [#1540](https://github.com/stnolting/neorv32/pull/1540) |
| 27.04.2026 | [**1.13.0**](https://github.com/stnolting/neorv32/releases/tag/v1.13.0) | :rocket: **New release** | |
| 25.04.2026 | 1.12.9.9 | `C` ISA extension: decompressor cleanups and logic optimizations (dead code elimination) | [#1537](https://github.com/stnolting/neorv32/pull/1537) |
| 24.04.2026 | 1.12.9.8 | minor rtl (corner case) fixes and logic optimizations | [#1534](https://github.com/stnolting/neorv32/pull/1534) |
| 24.04.2026 | 1.12.9.7 | UART: reset RX overflow flag on CTRL read; minor code cleanups | [#1533](https://github.com/stnolting/neorv32/pull/1533) |
| 23.04.2026 | 1.12.9.6 | minor rtl fixes, edits and cleanups | [#1531](https://github.com/stnolting/neorv32/pull/1531) |
| 12.04.2026 | 1.12.9.5 | :bug: fix bootloader's executable checksum computation | [#1528](https://github.com/stnolting/neorv32/pull/1528) |
| 11.04.2026 | 1.12.9.4 | add NEORV32-specific `MXISAH` CSR | [#1527](https://github.com/stnolting/neorv32/pull/1527) |
| 06.04.2026 | 1.12.9.3 | :warning: CFU: rework/simplify interface; add support for RISC-V `OP-32` and `OP-IMM-32` opcodes / instructions | [#1524](https://github.com/stnolting/neorv32/pull/1524) |
| 06.04.2026 | 1.12.9.2 | :test_tube: rework intrinsics (use `.insn` pseudo directive) | [#1523](https://github.com/stnolting/neorv32/pull/1523) |
| 05.04.2026 | 1.12.9.1 | :sparkles: add cache write-back & write-allocate policies | [#1513](https://github.com/stnolting/neorv32/pull/1513) |
| 03.04.2026 | [**1.12.9**](https://github.com/stnolting/neorv32/releases/tag/v1.12.9) | :rocket: **New release** | |
| 03.04.2026 | 1.12.8.10 | :bug: fix `sc.w` reservation set instruction: return all-zero on success | [#1520](https://github.com/stnolting/neorv32/pull/1520) |
| 01.04.2026 | 1.12.8.9 | add TRNG architecture configuration generics; :warning: rework TRNG control register layout | [#1518](https://github.com/stnolting/neorv32/pull/1518) |
| 28.03.2026 | 1.12.8.8 | add optional GPIO direction control | [#1517](https://github.com/stnolting/neorv32/pull/1517) |
| 12.03.2026 | 1.12.8.7 | :bug: fix GPTMR prescaler register write access logic | [#1509](https://github.com/stnolting/neorv32/pull/1509) |
| 08.03.2026 | 1.12.8.6 | remove `twd_scl_o` top port; SCL is only sampled by the TWD, but not driven | [#1506](https://github.com/stnolting/neorv32/pull/1506) |
| 07.03.2026 | 1.12.8.5 | :sparkles: bootloader: add flexible executable base address; :warning: rework bootloader executable header and signature | [#1505](https://github.com/stnolting/neorv32/pull/1505) |
| 06.03.2026 | 1.12.8.4 | minor rtl edits | [#1503](https://github.com/stnolting/neorv32/pull/1503) |
| 28.02.2026 | 1.12.8.3 | optimize cache module (less hardware, faster block updates) | [#1500](https://github.com/stnolting/neorv32/pull/1500) |
| 28.02.2026 | 1.12.8.2 | :warning: map TWD interrupt to FIRQ channel 4 | [#1499](https://github.com/stnolting/neorv32/pull/1499) |
| 25.02.2026 | 1.12.8.1 | TWD: add communication state flags and according IRQs; add IRQ if TX FIFO not full; :warning: remove bus sense flags | [#1498](https://github.com/stnolting/neorv32/pull/1498) |
| 15.02.2026 | [**1.12.8**](https://github.com/stnolting/neorv32/releases/tag/v1.12.8) | :rocket: **New release** | |
| 12.02.2026 | 1.12.7.9 | :warning: fix typo in top's MTIME IRQ port name: `irw_mti_i` -> `irq_mti_i` | [#1494](https://github.com/stnolting/neorv32/pull/1494) |
| 08.02.2026 | 1.12.7.8 | add generics to customize IMEM and DMEM base addresses (`IMEM_BASE` & `DMEM_BASE`); :warning: remove `IO_DISABLE_SYSINFO` generic | [#1492](https://github.com/stnolting/neorv32/pull/1492) |
| 03.02.2026 | 1.12.7.7 | minor rtl cleanups; improve timing of bus switch module | [#1489](https://github.com/stnolting/neorv32/pull/1489) |
| 31.01.2026 | 1.12.7.6 | :warning: rename `Zxcfu` ISA extension to `Xcfu` | [#1487](https://github.com/stnolting/neorv32/pull/1487) |
| 30.01.2026 | 1.12.7.5 | :bug: fix enabling of `Zbkx` ISA extension | [#1486](https://github.com/stnolting/neorv32/pull/1486) |
| 22.01.2026 | 1.12.7.4 | :warning: rework memory image files | [#1482](https://github.com/stnolting/neorv32/pull/1482) |
| 18.01.2026 | 1.12.7.3 | :sparkles: encapsulate memory components; caches: use block invalidation when a bus error occurs during block download | [#1481](https://github.com/stnolting/neorv32/pull/1481) |
| 18.01.2026 | 1.12.7.2 | :bug: fix `csrr[r/c][i]` instructions: do not write CSR if `rs1/imm5` is not zero | [#1479](https://github.com/stnolting/neorv32/pull/1479) |
| 16.01.2026 | 1.12.7.1 | :warning: rename bootloader ROM module: `neorv32_boot_rom` -> `neorv32_bootrom` | [#1477](https://github.com/stnolting/neorv32/pull/1477) |
| 12.01.2026 | [**1.12.7**](https://github.com/stnolting/neorv32/releases/tag/v1.12.7) | :rocket: **New release** | |
| 11.01.2026 | 1.12.6.9 | minor RTL cleanups and optimizations | [#1474](https://github.com/stnolting/neorv32/pull/1474) |
| 10.01.2026 | 1.12.6.8 | :warning: rename ALU co-processor modules: `neorv32_cpu_cp_*` -> `neorv32_cpu_alu_*` | [#1472](https://github.com/stnolting/neorv32/pull/1472) |
| 10.01.2026 | 1.12.6.7 | cache: relax size configuration constraints; :test_tube: add register stages for direct/uncached accesses | [#1471](https://github.com/stnolting/neorv32/pull/1471) |
| 04.01.2026 | 1.12.6.6 | :bug: fix bus time-out bug introduced in v1.12.6.4; minor rtl optimizations | [#1470](https://github.com/stnolting/neorv32/pull/1470) |
| 02.01.2026 | 1.12.6.5 | minor rtl edits; optimize register file's mapping to FPGA BRAM | [#1467](https://github.com/stnolting/neorv32/pull/1467) |
| 30.12.2025 | 1.12.6.4 | minor rtl edits, cleanups and optimizations | [#1465](https://github.com/stnolting/neorv32/pull/1465) |
| 29.12.2025 | 1.12.6.3 | :sparkles: add support for RISC-V `Smcntrpmf` ISA extension (counter privilege-mode filtering) | [#1464](https://github.com/stnolting/neorv32/pull/1464) |
| 28.12.2025 | 1.12.6.2 | :warning: HPM: remove "trap" event select; replace "taken branch" event by "control flow transfer" event | [#1463](https://github.com/stnolting/neorv32/pull/1463) |
| 27.12.2025 | 1.12.6.1 | rtl edits, cleanups and optimizations | [#1462](https://github.com/stnolting/neorv32/pull/1462) |
| 27.12.2025 | [**1.12.6**](https://github.com/stnolting/neorv32/releases/tag/v1.12.6) | :rocket: **New release** | |
| 24.12.2025 | 1.12.5.9 | minor rtl cleanups | [#1461](https://github.com/stnolting/neorv32/pull/1461) |
| 23.12.2025 | 1.12.5.8 | :sparkles: add new tuning option `CPU_RF_ARCH_SEL` to select implementation style of CPU register file (FPGA block RAM, FPGA distributed RAM, individual FFs, individual latches) | [#1460](https://github.com/stnolting/neorv32/pull/1460) |
| 21.12.2025 | 1.12.5.7 | :warning: remove `mxcsr` CSR | [#1459](https://github.com/stnolting/neorv32/pull/1459) |
| 19.12.2025 | 1.12.5.6 | further CPU code cleanups | [#1456](https://github.com/stnolting/neorv32/pull/1456) |
| 14.12.2025 | 1.12.5.5 | CPU control: massive code refactoring and optimization | [#1449](https://github.com/stnolting/neorv32/pull/1449) |
| 07.12.2025 | 1.12.5.4 | :sparkles: PWM: add optional phase-correct operation mode | [#1445](https://github.com/stnolting/neorv32/pull/1445) |
| 06.12.2025 | 1.12.5.3 | minor rtl edits and cleanups | [#1444](https://github.com/stnolting/neorv32/pull/1444) |
| 05.12.2025 | 1.12.5.2 | :warning: remove UART0/1 simulation-mode **file** logging | [#1443](https://github.com/stnolting/neorv32/pull/1443) |
| 04.12.2025 | 1.12.5.1 | fix minor OCD/DM command-execution issue | [#1440](https://github.com/stnolting/neorv32/pull/1440) |
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
| 29.07.2024 | [**1.10.2**](https://github.com/stnolting/neorv32/releases/tag/v1.10.2) | :rocket: **New release** | |
| ...        | ...      | **Change log trimmed. See [`CHANGELOG.md` in v1.10.2](https://github.com/stnolting/neorv32/blob/v1.10.2/CHANGELOG.md) for older logs.** | ... |
