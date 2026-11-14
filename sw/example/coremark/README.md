# NEORV32 CoreMark® Port

Out-of-tree port of the _Embedded Microprocessor Benchmark Consortium_
[CoreMark®](https://github.com/eembc/coremark) benchmark.
The target-specific code is outsourced to the `core_portme.[c/h]` files.
CoreMark is released under the Apache license.
See [coremark/LICENSE.md](coremark/LICENSE.md) for more information.

## Setup

CoreMark is included as a Git submodule that must be explicitly checked out
(unless the repository was cloned recursively):

```bash
neorv32/sw/example/coremark$ git submodule update --init coremark
```

Use the [Makefile](Makefile) to customize the build for the specific
core configuration (especially the ISAconfig, effort and memory layout):

```Makefile
# CPU ISA / extensions
MARCH = rv32imc_zicsr_zifencei

# Optimization
EFFORT = -O3

# Memory layout
USER_FLAGS += \
-Wl,--defsym,__neorv32_rom_size=64k \
-Wl,--defsym,__neorv32_rom_base=0x00000000 \
-Wl,--defsym,__neorv32_ram_size=64k \
-Wl,--defsym,__neorv32_ram_base=0x80000000 \
-Wl,--defsym,__neorv32_heap_size=32k
```

The standard configuration requires 64 KB of IMEM and 64 KB of DMEM.
However, depending on the Makefile configuration, the memory requirements may be lower.

By default, the **single-core** version of NEORV32 is used.
To compile code for the SMP dual-core version, set the `MULTITHREAD` define to `2`:

```Makefile
# Single-core: MULTITHREAD=1
# Dual-core: MULTITHREAD=2
USER_FLAGS += -DMULTITHREAD=1
```

Compile the executable (e.g. `elf` for JTAG upload, `exe` for upload via bootloader):

```bash
neorv32/sw/example/coremark$ make clean elf exe
```

## Exemplary Results

Configuration: NEORV32 v1.13.1.3, `rv32imc_zicsr_zifencei` CPU, fast mul & shift options enabled,
I$ & D$ enabled with default sized, bursts enabled, 64kB IMEM, 64kB DMEM.

### Single-Core

```
CoreMark on NEORV32 (v1.13.1.3)

Processor clock = 100000000 Hz
Running coremark (2000 iterations)...

2K performance run parameters for coremark.
CoreMark Size    : 666
Total ticks      : 2114519826
Total time (secs): 21.000000
Iterations/Sec   : 95.238095
Iterations       : 2000
Compiler version : GCC15.2.0
Compiler flags   : -march=rv32imc_zicsr_zifencei -mabi=ilp32 -O3 -Wall -Wextra -ffunction-sections -fdata-sections -mno-fdiv -mstrict-align -mbranch-cost=10 -ffp-contract=off -g -Wl,--defsym,__neorv32_rom_size=64k -Wl,--defsym,__neorv32_rom_base=0x00000000 -Wl,--defsym,__neorv32_ram_size=64k -Wl,--defsym,__neorv32_ram_base=0x80000000 -Wl,--defsym,__neorv32_heap_size=32k -DMULTITHREAD=1
Memory location  : STATIC
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0x4983
Correct operation validated. See README.md for run and reporting rules.
CoreMark 1.0 : 95.238095 / GCC15.2.0 -march=rv32imc_zicsr_zifencei -mabi=ilp32 -O3 -Wall -Wextra -ffunction-sections -fdata-sections -mno-fdiv -mstrict-align -mbranch-cost=10 -ffp-contract=off -g -Wl,--defsym,__neorv32_rom_size=64k -Wl,--defsym,__neorv32_rom_base=0x00000000 -Wl,--defsym,__neorv32_ram_size=64k -Wl,--defsym,__neorv32_ram_base=0x80000000 -Wl,--defsym,__neorv32_heap_size=32k -DMULTITHREAD=1 / STATIC

Execution completed.
```

### Dual-Core

```
CoreMark on NEORV32 (v1.13.1.3) - SMP dual-core version

Processor clock = 100000000 Hz
Running coremark (2000 iterations)...

2K performance run parameters for coremark.
CoreMark Size    : 666
Total ticks      : 2120411516
Total time (secs): 21.000000
Iterations/Sec   : 190.476190
Iterations       : 4000
Compiler version : GCC15.2.0
Compiler flags   : -march=rv32imc_zicsr_zifencei -mabi=ilp32 -O3 -Wall -Wextra -ffunction-sections -fdata-sections -mno-fdiv -mstrict-align -mbranch-cost=10 -ffp-contract=off -g -Wl,--defsym,__neorv32_rom_size=64k -Wl,--defsym,__neorv32_rom_base=0x00000000 -Wl,--defsym,__neorv32_ram_size=64k -Wl,--defsym,__neorv32_ram_base=0x80000000 -Wl,--defsym,__neorv32_heap_size=32k -DMULTITHREAD=2
Parallel Proprietary : 2
Memory location  : STATIC
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[1]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[1]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[1]crcstate      : 0x8e3a
[0]crcfinal      : 0x4983
[1]crcfinal      : 0x4983
Correct operation validated. See README.md for run and reporting rules.
CoreMark 1.0 : 190.476190 / GCC15.2.0 -march=rv32imc_zicsr_zifencei -mabi=ilp32 -O3 -Wall -Wextra -ffunction-sections -fdata-sections -mno-fdiv -mstrict-align -mbranch-cost=10 -ffp-contract=off -g -Wl,--defsym,__neorv32_rom_size=64k -Wl,--defsym,__neorv32_rom_base=0x00000000 -Wl,--defsym,__neorv32_ram_size=64k -Wl,--defsym,__neorv32_ram_base=0x80000000 -Wl,--defsym,__neorv32_heap_size=32k -DMULTITHREAD=2 / STATIC / 2:SMP-dual-core

Execution completed.
```
