# RISC-V Architectural Certification Tests (ACT)

[![RISC-V ACT](https://img.shields.io/github/actions/workflow/status/stnolting/neorv32/riscv-act.yml?branch=main&longCache=true&style=flat-square&label=RISC-V%20ACT&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions/workflows/riscv-act.yml)

Port of the official [RISC-V Architectural Certification Tests (ACT)](https://github.com/riscv/riscv-arch-test)
to certify that "_the processor faithfully implements the RISC-V specification_". This port was set up
based on the [_ACT Getting Started_](https://github.com/riscv/riscv-arch-test#getting-started) guide.


## Prerequisites

* [Python](https://www.python.org/) - main scripting language for the test framework
* [RISC-V GCC toolchain](https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack) - for compiling native RISC-V code
* [Sail RISC-V](https://github.com/riscv/sail-riscv) - golden reference model
* [GHDL](https://github.com/ghdl/ghdl) - VHDL simulator for simulating the DUT
* [mise](https://mise.jdx.dev) - tool and environment management (handles tool installation)
* [riscv-arch-test](https://github.com/riscv-non-isa/riscv-arch-test) submodule - test cases and framework

> [!NOTE]
> You may need to run `sudo chown -R $USER:$USER /var/lib/gems/3.2.0/` to adjust the ownership of the system-wide installation directory for Ruby gems.


## Port Details

All NEORV32-specific test framework configuration files are located in [`config`](config). These files are
passed to the test framework as out-of-tree configuration. See [`config/neorv32.yaml`](config/neorv32.yaml)
for a list of all currently supported/tested ISA extensions. The DUT testbench [`riscv_act_tb.vhd`](riscv_act_tb.vhd)
instantiates the core and also implements a memory subsystem and test harness via the external bus interface:

| Address | Description |
|:--------|:------------|
| `0x80000000` ... `0x803FFFFF` | Main memory (RAM), default 1MB, max 4MB; pre-initialized with the application executable |
| `0xF0000000` | Write any value to terminate the simulation; writing `0x12345678` indicates a PASS; used by the `RVMODEL_HALT_PASS` and `RVMODEL_HALT_FAIL` macros |
| `0xF0000004` | Print character (lowest 8 bits) to the simulator console; used by the `RVMODEL_IO_WRITE_STR` macro for test status and logging |
| `0xF0000008` | Bit 0 controls the hart's external machine interrupt; used by `RVMODEL_SET_MEXT_INT` and `RVMODEL_CLR_MEXT_INT` macros |


## Running ACT

A simple [`Makefile`](Makefile) is used for building and running the tests.

```bash
neorv32-riscv-act$ make help
NEORV32 RISC-V ACT Makefile

check - check required tools
setup - setup DUT configuration
mise  - allow mise to install required tools
tests - generate tests and coverpoints
elfs  - build target ELFs
sim   - setup DUT simulation
run   - run all tests on target
clean - remove all artifacts
all   - setup + mise + tests + elfs + sim + run
help  - show this text
```

Once all prerequisites have been installed the `check` target can be used to check all tools:

```bash
neorv32/riscv-act$ make check
```

If "Tool checks OK" appears at the end the setup is ready to run. Use the `all` target to generate all test cases
and run them on the DUT:

```bash
neorv32/riscv-act$ make all
```

The final test report is available in `neorv32/riscv-act/riscv-arch-test/work/neorv32/summary.log`.

The tests are executed automatically as a [GitHub action](https://github.com/stnolting/neorv32/actions/workflows/riscv-act.yml).
Dependabot is used to keep the `riscv-arch-test` submodule up to date.

> [!TIP]
> Click on the CI status badge on top of this page to see the latest compatibility test workflow runs.
> The test report summary is available as GitHib actions artifact.


### Debugging

The Sail reference model generates full trace logs when the debugging flag is set int he main makefile:

```makefile
DEBUG=True \
```

These trace logs are generated in the `riscv-arch-test/work/neorv32/build` directory

NEORV32 can also emit full execution trace logs. To enable trace logging, set the `TRACE_EN`
generic in the (testbench)[`neorv32_act_tb.vhd`] entity:

```vhdl
TRACE_EN : boolean := true; -- enable trace logging
```

Add the failed test(s) to the `EXTENSIONS` variable, e.g.:

```makefile
EXTENSIONS=Sm,U \
```

Recompile the simulation base using `$make sim` and (re-)run test(s).
Trace logs will be generated in the test's `log` folder(s) using the following naming scheme: `<test-name>.trace.log`


## Exemplary Test Results

NEORV32 `v1.13.3.1`, 24th of July, 2026.

```
RESULT: All 301 tests passed.
priv/ExceptionsSm/ExceptionsSm-00.log              RVCP-SUMMARY: TEST PASSED - Test File "ExceptionsSm-00.S"
priv/ExceptionsU/ExceptionsU-00.log                RVCP-SUMMARY: TEST PASSED - Test File "ExceptionsU-00.S"
priv/ExceptionsZaamo/ExceptionsZaamo-00.log        RVCP-SUMMARY: TEST PASSED - Test File "ExceptionsZaamo-00.S"
priv/ExceptionsZalrsc/ExceptionsZalrsc-00.log      RVCP-SUMMARY: TEST PASSED - Test File "ExceptionsZalrsc-00.S"
priv/ExceptionsZc/ExceptionsZc-00.log              RVCP-SUMMARY: TEST PASSED - Test File "ExceptionsZc-00.S"
priv/ExceptionsZicboU/ExceptionsZicboU-00.log      RVCP-SUMMARY: TEST PASSED - Test File "ExceptionsZicboU-00.S"
priv/InterruptsSm/InterruptsSm-00.log              RVCP-SUMMARY: TEST PASSED - Test File "InterruptsSm-00.S"
priv/InterruptsU/InterruptsU-00.log                RVCP-SUMMARY: TEST PASSED - Test File "InterruptsU-00.S"
priv/SsstrictSm/SsstrictSm-12.log                  RVCP-SUMMARY: TEST PASSED - Test File "SsstrictSm-12.S"
priv/SsstrictSm/SsstrictSm-13.log                  RVCP-SUMMARY: TEST PASSED - Test File "SsstrictSm-13.S"
priv/SsstrictSm/SsstrictSm-14.log                  RVCP-SUMMARY: TEST PASSED - Test File "SsstrictSm-14.S"
priv/U/U-00.log                                    RVCP-SUMMARY: TEST PASSED - Test File "U-00.S"
priv/ZicntrU/ZicntrU-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "ZicntrU-00.S"
priv/pmp/pmp32/PMPSm/pmpsm_all_entries_check.log   RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_all_entries_check.S"
priv/pmp/pmp32/PMPSm/pmpsm_cfg_A_all.log           RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_cfg_A_all.S"
priv/pmp/pmp32/PMPSm/pmpsm_cfg_A_off_all.log       RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_cfg_A_off_all.S"
priv/pmp/pmp32/PMPSm/pmpsm_cfg_A_tor_bot.log       RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_cfg_A_tor_bot.S"
priv/pmp/pmp32/PMPSm/pmpsm_cfg_A_tor_zero.log      RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_cfg_A_tor_zero.S"
priv/pmp/pmp32/PMPSm/pmpsm_cfg_L_access_all.log    RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_cfg_L_access_all.S"
priv/pmp/pmp32/PMPSm/pmpsm_cfg_L_modify_napot.log  RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_cfg_L_modify_napot.S"
priv/pmp/pmp32/PMPSm/pmpsm_cfg_L_modify_off.log    RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_cfg_L_modify_off.S"
priv/pmp/pmp32/PMPSm/pmpsm_cfg_L_modify_tor.log    RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_cfg_L_modify_tor.S"
priv/pmp/pmp32/PMPSm/pmpsm_cfg_XWR_all-01.log      RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_cfg_XWR_all-01.S"
priv/pmp/pmp32/PMPSm/pmpsm_cfg_XWR_all-02.log      RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_cfg_XWR_all-02.S"
priv/pmp/pmp32/PMPSm/pmpsm_cfg_XWR_all-03.log      RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_cfg_XWR_all-03.S"
priv/pmp/pmp32/PMPSm/pmpsm_cfg_XWR_all-04.log      RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_cfg_XWR_all-04.S"
priv/pmp/pmp32/PMPSm/pmpsm_cfg_na4_all.log         RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_cfg_na4_all.S"
priv/pmp/pmp32/PMPSm/pmpsm_cfg_napot_all.log       RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_cfg_napot_all.S"
priv/pmp/pmp32/PMPSm/pmpsm_cfg_tor_all.log         RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_cfg_tor_all.S"
priv/pmp/pmp32/PMPSm/pmpsm_cfg_tor_check-01.log    RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_cfg_tor_check-01.S"
priv/pmp/pmp32/PMPSm/pmpsm_cfg_tor_check-02.log    RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_cfg_tor_check-02.S"
priv/pmp/pmp32/PMPSm/pmpsm_cfg_tor_check-03.log    RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_cfg_tor_check-03.S"
priv/pmp/pmp32/PMPSm/pmpsm_csr_walk-1.log          RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_csr_walk-01.S"
priv/pmp/pmp32/PMPSm/pmpsm_csr_walk-10.log         RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_csr_walk-10.S"
priv/pmp/pmp32/PMPSm/pmpsm_csr_walk-2.log          RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_csr_walk-02.S"
priv/pmp/pmp32/PMPSm/pmpsm_csr_walk-3.log          RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_csr_walk-03.S"
priv/pmp/pmp32/PMPSm/pmpsm_csr_walk-4.log          RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_csr_walk-04.S"
priv/pmp/pmp32/PMPSm/pmpsm_csr_walk-5.log          RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_csr_walk-05.S"
priv/pmp/pmp32/PMPSm/pmpsm_csr_walk-6.log          RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_csr_walk-06.S"
priv/pmp/pmp32/PMPSm/pmpsm_csr_walk-7.log          RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_csr_walk-07.S"
priv/pmp/pmp32/PMPSm/pmpsm_csr_walk-8.log          RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_csr_walk-08.S"
priv/pmp/pmp32/PMPSm/pmpsm_csr_walk-9.log          RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_csr_walk-09.S"
priv/pmp/pmp32/PMPSm/pmpsm_grain.log               RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_grain.S"
priv/pmp/pmp32/PMPSm/pmpsm_grain_check.log         RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_grain_check.S"
priv/pmp/pmp32/PMPSm/pmpsm_na4_legal_lwxr.log      RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_cfg_napot_all.S"
priv/pmp/pmp32/PMPSm/pmpsm_napot_legal_lwxr.log    RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_napot_legal_lwxr.S"
priv/pmp/pmp32/PMPSm/pmpsm_priority.log            RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_priority.S"
priv/pmp/pmp32/PMPSm/pmpsm_priority_off.log        RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_priority_off.S"
priv/pmp/pmp32/PMPSm/pmpsm_tor_legal_lwxr-01.log   RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_tor_legal_lwxr.S"
priv/pmp/pmp32/PMPSm/pmpsm_tor_legal_lwxr-02.log   RVCP-SUMMARY: TEST PASSED - Test File "pmpsm_tor_legal_lwxr.S"
priv/pmp/pmp32/PMPU/pmpu_cfg_A_off.log             RVCP-SUMMARY: TEST PASSED - Test File "pmpu_cfg_A_all.S"
priv/pmp/pmp32/PMPU/pmpu_cfg_XWR.log               RVCP-SUMMARY: TEST PASSED - Test File "pmpu_cfg_XWR.S"
priv/pmp/pmp32/PMPU/pmpu_cfg_XWR_unlocked.log      RVCP-SUMMARY: TEST PASSED - Test File "pmpu_cfg_XWR_unlocked.S"
priv/pmp/pmp32/PMPU/pmpu_csr_access.log            RVCP-SUMMARY: TEST PASSED - Test File "pmpu_csr_access.S"
priv/pmp/pmp32/PMPU/pmpu_mprv_check-01.log         RVCP-SUMMARY: TEST PASSED - Test File "pmpu_mprv_check-01.S"
priv/pmp/pmp32/PMPU/pmpu_mprv_check-02.log         RVCP-SUMMARY: TEST PASSED - Test File "pmpu_mprv_check-02.S"
priv/pmp/pmp32/PMPU/pmpu_napot_legal_lxwr-01.log   RVCP-SUMMARY: TEST PASSED - Test File "pmpu_napot_legal_lxwr.S"
priv/pmp/pmp32/PMPU/pmpu_napot_legal_lxwr-02.log   RVCP-SUMMARY: TEST PASSED - Test File "pmps_napot_legal_lxwr.S"
priv/pmp/pmp32/PMPU/pmpu_tor_legal_lxwr-01.log     RVCP-SUMMARY: TEST PASSED - Test File "pmpu_tor_legal_lxwr.S"
priv/pmp/pmp32/PMPU/pmpu_tor_legal_lxwr-02.log     RVCP-SUMMARY: TEST PASSED - Test File "pmpu_tor_legal_lxwr.S"
priv/pmp/pmp32/PMPZaamo/pmpzaamo_cfg_wr.log        RVCP-SUMMARY: TEST PASSED - Test File "pmpzaamo_cfg_wr.S"
priv/pmp/pmp32/PMPZalrsc/pmpzalrsc_cfg_wr.log      RVCP-SUMMARY: TEST PASSED - Test File "pmpzalrsc_cfg_wrS"
priv/pmp/pmp32/PMPZca/pmpzca_aligned_na4.log       RVCP-SUMMARY: TEST PASSED - Test File "pmpzca_aligned_na4.S"
priv/pmp/pmp32/PMPZca/pmpzca_aligned_napot.log     RVCP-SUMMARY: TEST PASSED - Test File "pmpzca_aligned_napot.S"
priv/pmp/pmp32/PMPZca/pmpzca_aligned_off.log       RVCP-SUMMARY: TEST PASSED - Test File "pmpzca_aligned_off.S"
priv/pmp/pmp32/PMPZca/pmpzca_aligned_tor.log       RVCP-SUMMARY: TEST PASSED - Test File "pmpzca_aligned_tor.S"
priv/pmp/pmp32/PMPZca/pmpzca_cret_na4.log          RVCP-SUMMARY: TEST PASSED - Test File "pmpzca_cret_napot.S"
priv/pmp/pmp32/PMPZca/pmpzca_cret_napot.log        RVCP-SUMMARY: TEST PASSED - Test File "pmpzca_cret_napot.S"
priv/pmp/pmp32/PMPZca/pmpzca_cret_tor.log          RVCP-SUMMARY: TEST PASSED - Test File "pmpzca_cret_napot.S"
priv/pmp/pmp32/PMPZca/pmpzca_legal_lwrx.log        RVCP-SUMMARY: TEST PASSED - Test File "pmpzca_legal_lxwr.S"
priv/pmp/pmp32/PMPZca/pmpzca_misaligned_na4.log    RVCP-SUMMARY: TEST PASSED - Test File "pmpzca_aligned_na4.S"
priv/pmp/pmp32/PMPZca/pmpzca_misaligned_napot.log  RVCP-SUMMARY: TEST PASSED - Test File "pmpzca_misaligned_napot.S"
priv/pmp/pmp32/PMPZca/pmpzca_misaligned_off.log    RVCP-SUMMARY: TEST PASSED - Test File "pmpzca_misaligned_off.S"
priv/pmp/pmp32/PMPZca/pmpzca_misaligned_tor.log    RVCP-SUMMARY: TEST PASSED - Test File "pmpzca_misaligned_tor.S"
priv/pmp/pmp32/PMPZca/pmpzcb_legal_lxwr.log        RVCP-SUMMARY: TEST PASSED - Test File "pmpzcb_legal_lwxr.S"
rv32i/I/I-add-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "I-add-00.S"
rv32i/I/I-addi-00.log                              RVCP-SUMMARY: TEST PASSED - Test File "I-addi-00.S"
rv32i/I/I-and-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "I-and-00.S"
rv32i/I/I-andi-00.log                              RVCP-SUMMARY: TEST PASSED - Test File "I-andi-00.S"
rv32i/I/I-auipc-00.log                             RVCP-SUMMARY: TEST PASSED - Test File "I-auipc-00.S"
rv32i/I/I-beq-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "I-beq-00.S"
rv32i/I/I-bge-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "I-bge-00.S"
rv32i/I/I-bgeu-00.log                              RVCP-SUMMARY: TEST PASSED - Test File "I-bgeu-00.S"
rv32i/I/I-blt-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "I-blt-00.S"
rv32i/I/I-bltu-00.log                              RVCP-SUMMARY: TEST PASSED - Test File "I-bltu-00.S"
rv32i/I/I-bne-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "I-bne-00.S"
rv32i/I/I-fence-00.log                             RVCP-SUMMARY: TEST PASSED - Test File "I-fence-00.S"
rv32i/I/I-jal-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "I-jal-00.S"
rv32i/I/I-jalr-00.log                              RVCP-SUMMARY: TEST PASSED - Test File "I-jalr-00.S"
rv32i/I/I-lb-00.log                                RVCP-SUMMARY: TEST PASSED - Test File "I-lb-00.S"
rv32i/I/I-lbu-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "I-lbu-00.S"
rv32i/I/I-lh-00.log                                RVCP-SUMMARY: TEST PASSED - Test File "I-lh-00.S"
rv32i/I/I-lhu-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "I-lhu-00.S"
rv32i/I/I-lui-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "I-lui-00.S"
rv32i/I/I-lw-00.log                                RVCP-SUMMARY: TEST PASSED - Test File "I-lw-00.S"
rv32i/I/I-nop-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "I-nop-00.S"
rv32i/I/I-or-00.log                                RVCP-SUMMARY: TEST PASSED - Test File "I-or-00.S"
rv32i/I/I-ori-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "I-ori-00.S"
rv32i/I/I-sb-00.log                                RVCP-SUMMARY: TEST PASSED - Test File "I-sb-00.S"
rv32i/I/I-sh-00.log                                RVCP-SUMMARY: TEST PASSED - Test File "I-sh-00.S"
rv32i/I/I-sll-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "I-sll-00.S"
rv32i/I/I-slli-00.log                              RVCP-SUMMARY: TEST PASSED - Test File "I-slli-00.S"
rv32i/I/I-slt-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "I-slt-00.S"
rv32i/I/I-slti-00.log                              RVCP-SUMMARY: TEST PASSED - Test File "I-slti-00.S"
rv32i/I/I-sltiu-00.log                             RVCP-SUMMARY: TEST PASSED - Test File "I-sltiu-00.S"
rv32i/I/I-sltu-00.log                              RVCP-SUMMARY: TEST PASSED - Test File "I-sltu-00.S"
rv32i/I/I-sra-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "I-sra-00.S"
rv32i/I/I-srai-00.log                              RVCP-SUMMARY: TEST PASSED - Test File "I-srai-00.S"
rv32i/I/I-srl-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "I-srl-00.S"
rv32i/I/I-srli-00.log                              RVCP-SUMMARY: TEST PASSED - Test File "I-srli-00.S"
rv32i/I/I-sub-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "I-sub-00.S"
rv32i/I/I-sw-00.log                                RVCP-SUMMARY: TEST PASSED - Test File "I-sw-00.S"
rv32i/I/I-xor-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "I-xor-00.S"
rv32i/I/I-xori-00.log                              RVCP-SUMMARY: TEST PASSED - Test File "I-xori-00.S"
rv32i/M/M-div-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "M-div-00.S"
rv32i/M/M-divu-00.log                              RVCP-SUMMARY: TEST PASSED - Test File "M-divu-00.S"
rv32i/M/M-mul-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "M-mul-00.S"
rv32i/M/M-mulh-00.log                              RVCP-SUMMARY: TEST PASSED - Test File "M-mulh-00.S"
rv32i/M/M-mulhsu-00.log                            RVCP-SUMMARY: TEST PASSED - Test File "M-mulhsu-00.S"
rv32i/M/M-mulhu-00.log                             RVCP-SUMMARY: TEST PASSED - Test File "M-mulhu-00.S"
rv32i/M/M-rem-00.log                               RVCP-SUMMARY: TEST PASSED - Test File "M-rem-00.S"
rv32i/M/M-remu-00.log                              RVCP-SUMMARY: TEST PASSED - Test File "M-remu-00.S"
rv32i/Zaamo/Zaamo-amoadd.w-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zaamo-amoadd.w-00.S"
rv32i/Zaamo/Zaamo-amoand.w-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zaamo-amoand.w-00.S"
rv32i/Zaamo/Zaamo-amomax.w-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zaamo-amomax.w-00.S"
rv32i/Zaamo/Zaamo-amomaxu.w-00.log                 RVCP-SUMMARY: TEST PASSED - Test File "Zaamo-amomaxu.w-00.S"
rv32i/Zaamo/Zaamo-amomin.w-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zaamo-amomin.w-00.S"
rv32i/Zaamo/Zaamo-amominu.w-00.log                 RVCP-SUMMARY: TEST PASSED - Test File "Zaamo-amominu.w-00.S"
rv32i/Zaamo/Zaamo-amoor.w-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zaamo-amoor.w-00.S"
rv32i/Zaamo/Zaamo-amoswap.w-00.log                 RVCP-SUMMARY: TEST PASSED - Test File "Zaamo-amoswap.w-00.S"
rv32i/Zaamo/Zaamo-amoxor.w-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zaamo-amoxor.w-00.S"
rv32i/Zalrsc/Zalrsc-lr.w-00.log                    RVCP-SUMMARY: TEST PASSED - Test File "Zalrsc-lr.w-00.S"
rv32i/Zalrsc/Zalrsc-sc.w-00.log                    RVCP-SUMMARY: TEST PASSED - Test File "Zalrsc-sc.w-00.S"
rv32i/Zba/Zba-sh1add-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zba-sh1add-00.S"
rv32i/Zba/Zba-sh2add-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zba-sh2add-00.S"
rv32i/Zba/Zba-sh3add-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zba-sh3add-00.S"
rv32i/Zbb/Zbb-andn-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zbb-andn-00.S"
rv32i/Zbb/Zbb-clz-00.log                           RVCP-SUMMARY: TEST PASSED - Test File "Zbb-clz-00.S"
rv32i/Zbb/Zbb-cpop-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zbb-cpop-00.S"
rv32i/Zbb/Zbb-ctz-00.log                           RVCP-SUMMARY: TEST PASSED - Test File "Zbb-ctz-00.S"
rv32i/Zbb/Zbb-max-00.log                           RVCP-SUMMARY: TEST PASSED - Test File "Zbb-max-00.S"
rv32i/Zbb/Zbb-maxu-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zbb-maxu-00.S"
rv32i/Zbb/Zbb-min-00.log                           RVCP-SUMMARY: TEST PASSED - Test File "Zbb-min-00.S"
rv32i/Zbb/Zbb-minu-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zbb-minu-00.S"
rv32i/Zbb/Zbb-orc.b-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zbb-orc.b-00.S"
rv32i/Zbb/Zbb-orn-00.log                           RVCP-SUMMARY: TEST PASSED - Test File "Zbb-orn-00.S"
rv32i/Zbb/Zbb-rev8-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zbb-rev8-00.S"
rv32i/Zbb/Zbb-rol-00.log                           RVCP-SUMMARY: TEST PASSED - Test File "Zbb-rol-00.S"
rv32i/Zbb/Zbb-ror-00.log                           RVCP-SUMMARY: TEST PASSED - Test File "Zbb-ror-00.S"
rv32i/Zbb/Zbb-rori-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zbb-rori-00.S"
rv32i/Zbb/Zbb-sext.b-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zbb-sext.b-00.S"
rv32i/Zbb/Zbb-sext.h-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zbb-sext.h-00.S"
rv32i/Zbb/Zbb-xnor-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zbb-xnor-00.S"
rv32i/Zbb/Zbb-zext.h-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zbb-zext.h-00.S"
rv32i/Zbc/Zbc-clmul-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zbc-clmul-00.S"
rv32i/Zbc/Zbc-clmulh-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zbc-clmulh-00.S"
rv32i/Zbc/Zbc-clmulr-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zbc-clmulr-00.S"
rv32i/Zbkb/Zbkb-andn-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zbkb-andn-00.S"
rv32i/Zbkb/Zbkb-brev8-00.log                       RVCP-SUMMARY: TEST PASSED - Test File "Zbkb-brev8-00.S"
rv32i/Zbkb/Zbkb-orn-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zbkb-orn-00.S"
rv32i/Zbkb/Zbkb-pack-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zbkb-pack-00.S"
rv32i/Zbkb/Zbkb-packh-00.log                       RVCP-SUMMARY: TEST PASSED - Test File "Zbkb-packh-00.S"
rv32i/Zbkb/Zbkb-rev8-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zbkb-rev8-00.S"
rv32i/Zbkb/Zbkb-rol-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zbkb-rol-00.S"
rv32i/Zbkb/Zbkb-ror-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zbkb-ror-00.S"
rv32i/Zbkb/Zbkb-rori-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zbkb-rori-00.S"
rv32i/Zbkb/Zbkb-unzip-00.log                       RVCP-SUMMARY: TEST PASSED - Test File "Zbkb-unzip-00.S"
rv32i/Zbkb/Zbkb-xnor-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zbkb-xnor-00.S"
rv32i/Zbkb/Zbkb-zip-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zbkb-zip-00.S"
rv32i/Zbkc/Zbkc-clmul-00.log                       RVCP-SUMMARY: TEST PASSED - Test File "Zbkc-clmul-00.S"
rv32i/Zbkc/Zbkc-clmulh-00.log                      RVCP-SUMMARY: TEST PASSED - Test File "Zbkc-clmulh-00.S"
rv32i/Zbkx/Zbkx-xperm4-00.log                      RVCP-SUMMARY: TEST PASSED - Test File "Zbkx-xperm4-00.S"
rv32i/Zbkx/Zbkx-xperm8-00.log                      RVCP-SUMMARY: TEST PASSED - Test File "Zbkx-xperm8-00.S"
rv32i/Zbs/Zbs-bclr-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zbs-bclr-00.S"
rv32i/Zbs/Zbs-bclri-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zbs-bclri-00.S"
rv32i/Zbs/Zbs-bext-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zbs-bext-00.S"
rv32i/Zbs/Zbs-bexti-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zbs-bexti-00.S"
rv32i/Zbs/Zbs-binv-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zbs-binv-00.S"
rv32i/Zbs/Zbs-binvi-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zbs-binvi-00.S"
rv32i/Zbs/Zbs-bset-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zbs-bset-00.S"
rv32i/Zbs/Zbs-bseti-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zbs-bseti-00.S"
rv32i/Zca/Zca-c.add-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.add-00.S"
rv32i/Zca/Zca-c.addi-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.addi-00.S"
rv32i/Zca/Zca-c.addi16sp-00.log                    RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.addi16sp-00.S"
rv32i/Zca/Zca-c.addi4spn-00.log                    RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.addi4spn-00.S"
rv32i/Zca/Zca-c.and-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.and-00.S"
rv32i/Zca/Zca-c.andi-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.andi-00.S"
rv32i/Zca/Zca-c.beqz-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.beqz-00.S"
rv32i/Zca/Zca-c.bnez-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.bnez-00.S"
rv32i/Zca/Zca-c.j-00.log                           RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.j-00.S"
rv32i/Zca/Zca-c.jal-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.jal-00.S"
rv32i/Zca/Zca-c.jalr-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.jalr-00.S"
rv32i/Zca/Zca-c.jr-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.jr-00.S"
rv32i/Zca/Zca-c.li-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.li-00.S"
rv32i/Zca/Zca-c.lui-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.lui-00.S"
rv32i/Zca/Zca-c.lw-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.lw-00.S"
rv32i/Zca/Zca-c.lwsp-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.lwsp-00.S"
rv32i/Zca/Zca-c.mv-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.mv-00.S"
rv32i/Zca/Zca-c.nop-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.nop-00.S"
rv32i/Zca/Zca-c.or-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.or-00.S"
rv32i/Zca/Zca-c.slli-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.slli-00.S"
rv32i/Zca/Zca-c.srai-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.srai-00.S"
rv32i/Zca/Zca-c.srli-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.srli-00.S"
rv32i/Zca/Zca-c.sub-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.sub-00.S"
rv32i/Zca/Zca-c.sw-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.sw-00.S"
rv32i/Zca/Zca-c.swsp-00.log                        RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.swsp-00.S"
rv32i/Zca/Zca-c.xor-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zca-c.xor-00.S"
rv32i/Zcb/Zcb-c.lbu-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zcb-c.lbu-00.S"
rv32i/Zcb/Zcb-c.lh-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zcb-c.lh-00.S"
rv32i/Zcb/Zcb-c.lhu-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zcb-c.lhu-00.S"
rv32i/Zcb/Zcb-c.not-00.log                         RVCP-SUMMARY: TEST PASSED - Test File "Zcb-c.not-00.S"
rv32i/Zcb/Zcb-c.sb-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zcb-c.sb-00.S"
rv32i/Zcb/Zcb-c.sh-00.log                          RVCP-SUMMARY: TEST PASSED - Test File "Zcb-c.sh-00.S"
rv32i/Zcb/Zcb-c.zext.b-00.log                      RVCP-SUMMARY: TEST PASSED - Test File "Zcb-c.zext.b-00.S"
rv32i/ZcbM/ZcbM-c.mul-00.log                       RVCP-SUMMARY: TEST PASSED - Test File "ZcbM-c.mul-00.S"
rv32i/ZcbZbb/ZcbZbb-c.sext.b-00.log                RVCP-SUMMARY: TEST PASSED - Test File "ZcbZbb-c.sext.b-00.S"
rv32i/ZcbZbb/ZcbZbb-c.sext.h-00.log                RVCP-SUMMARY: TEST PASSED - Test File "ZcbZbb-c.sext.h-00.S"
rv32i/ZcbZbb/ZcbZbb-c.zext.h-00.log                RVCP-SUMMARY: TEST PASSED - Test File "ZcbZbb-c.zext.h-00.S"
rv32i/Zcmop/Zcmop-c.mop.1-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zcmop-c.mop.1-00.S"
rv32i/Zcmop/Zcmop-c.mop.11-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zcmop-c.mop.11-00.S"
rv32i/Zcmop/Zcmop-c.mop.13-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zcmop-c.mop.13-00.S"
rv32i/Zcmop/Zcmop-c.mop.15-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zcmop-c.mop.15-00.S"
rv32i/Zcmop/Zcmop-c.mop.3-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zcmop-c.mop.3-00.S"
rv32i/Zcmop/Zcmop-c.mop.5-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zcmop-c.mop.5-00.S"
rv32i/Zcmop/Zcmop-c.mop.7-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zcmop-c.mop.7-00.S"
rv32i/Zcmop/Zcmop-c.mop.9-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zcmop-c.mop.9-00.S"
rv32i/Zicntr/Zicntr-csrrc-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zicntr-csrrc-00.S"
rv32i/Zicntr/Zicntr-csrrs-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zicntr-csrrs-00.S"
rv32i/Zicond/Zicond-czero.eqz-00.log               RVCP-SUMMARY: TEST PASSED - Test File "Zicond-czero.eqz-00.S"
rv32i/Zicond/Zicond-czero.nez-00.log               RVCP-SUMMARY: TEST PASSED - Test File "Zicond-czero.nez-00.S"
rv32i/Zicsr/Zicsr-csrrc-00.log                     RVCP-SUMMARY: TEST PASSED - Test File "Zicsr-csrrc-00.S"
rv32i/Zicsr/Zicsr-csrrci-00.log                    RVCP-SUMMARY: TEST PASSED - Test File "Zicsr-csrrci-00.S"
rv32i/Zicsr/Zicsr-csrrs-00.log                     RVCP-SUMMARY: TEST PASSED - Test File "Zicsr-csrrs-00.S"
rv32i/Zicsr/Zicsr-csrrsi-00.log                    RVCP-SUMMARY: TEST PASSED - Test File "Zicsr-csrrsi-00.S"
rv32i/Zicsr/Zicsr-csrrw-00.log                     RVCP-SUMMARY: TEST PASSED - Test File "Zicsr-csrrw-00.S"
rv32i/Zicsr/Zicsr-csrrwi-00.log                    RVCP-SUMMARY: TEST PASSED - Test File "Zicsr-csrrwi-00.S"
rv32i/Zifencei/Zifencei-fence.i-00.log             RVCP-SUMMARY: TEST PASSED - Test File "Zifencei-fence.i-00.S"
rv32i/Zihpm/Zihpm-csrrc-00.log                     RVCP-SUMMARY: TEST PASSED - Test File "Zihpm-csrrc-00.S"
rv32i/Zihpm/Zihpm-csrrs-00.log                     RVCP-SUMMARY: TEST PASSED - Test File "Zihpm-csrrs-00.S"
rv32i/Zimop/Zimop-mop.r.0-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.0-00.S"
rv32i/Zimop/Zimop-mop.r.1-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.1-00.S"
rv32i/Zimop/Zimop-mop.r.10-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.10-00.S"
rv32i/Zimop/Zimop-mop.r.11-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.11-00.S"
rv32i/Zimop/Zimop-mop.r.12-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.12-00.S"
rv32i/Zimop/Zimop-mop.r.13-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.13-00.S"
rv32i/Zimop/Zimop-mop.r.14-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.14-00.S"
rv32i/Zimop/Zimop-mop.r.15-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.15-00.S"
rv32i/Zimop/Zimop-mop.r.16-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.16-00.S"
rv32i/Zimop/Zimop-mop.r.17-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.17-00.S"
rv32i/Zimop/Zimop-mop.r.18-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.18-00.S"
rv32i/Zimop/Zimop-mop.r.19-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.19-00.S"
rv32i/Zimop/Zimop-mop.r.2-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.2-00.S"
rv32i/Zimop/Zimop-mop.r.20-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.20-00.S"
rv32i/Zimop/Zimop-mop.r.21-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.21-00.S"
rv32i/Zimop/Zimop-mop.r.22-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.22-00.S"
rv32i/Zimop/Zimop-mop.r.23-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.23-00.S"
rv32i/Zimop/Zimop-mop.r.24-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.24-00.S"
rv32i/Zimop/Zimop-mop.r.25-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.25-00.S"
rv32i/Zimop/Zimop-mop.r.26-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.26-00.S"
rv32i/Zimop/Zimop-mop.r.27-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.27-00.S"
rv32i/Zimop/Zimop-mop.r.28-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.28-00.S"
rv32i/Zimop/Zimop-mop.r.29-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.29-00.S"
rv32i/Zimop/Zimop-mop.r.3-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.3-00.S"
rv32i/Zimop/Zimop-mop.r.30-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.30-00.S"
rv32i/Zimop/Zimop-mop.r.31-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.31-00.S"
rv32i/Zimop/Zimop-mop.r.4-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.4-00.S"
rv32i/Zimop/Zimop-mop.r.5-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.5-00.S"
rv32i/Zimop/Zimop-mop.r.6-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.6-00.S"
rv32i/Zimop/Zimop-mop.r.7-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.7-00.S"
rv32i/Zimop/Zimop-mop.r.8-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.8-00.S"
rv32i/Zimop/Zimop-mop.r.9-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.r.9-00.S"
rv32i/Zimop/Zimop-mop.rr.0-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.rr.0-00.S"
rv32i/Zimop/Zimop-mop.rr.1-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.rr.1-00.S"
rv32i/Zimop/Zimop-mop.rr.2-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.rr.2-00.S"
rv32i/Zimop/Zimop-mop.rr.3-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.rr.3-00.S"
rv32i/Zimop/Zimop-mop.rr.4-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.rr.4-00.S"
rv32i/Zimop/Zimop-mop.rr.5-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.rr.5-00.S"
rv32i/Zimop/Zimop-mop.rr.6-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.rr.6-00.S"
rv32i/Zimop/Zimop-mop.rr.7-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zimop-mop.rr.7-00.S"
rv32i/Zknd/Zknd-aes32dsi-00.log                    RVCP-SUMMARY: TEST PASSED - Test File "Zknd-aes32dsi-00.S"
rv32i/Zknd/Zknd-aes32dsmi-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zknd-aes32dsmi-00.S"
rv32i/Zkne/Zkne-aes32esi-00.log                    RVCP-SUMMARY: TEST PASSED - Test File "Zkne-aes32esi-00.S"
rv32i/Zkne/Zkne-aes32esmi-00.log                   RVCP-SUMMARY: TEST PASSED - Test File "Zkne-aes32esmi-00.S"
rv32i/Zknh/Zknh-sha256sig0-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zknh-sha256sig0-00.S"
rv32i/Zknh/Zknh-sha256sig1-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zknh-sha256sig1-00.S"
rv32i/Zknh/Zknh-sha256sum0-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zknh-sha256sum0-00.S"
rv32i/Zknh/Zknh-sha256sum1-00.log                  RVCP-SUMMARY: TEST PASSED - Test File "Zknh-sha256sum1-00.S"
rv32i/Zknh/Zknh-sha512sig0h-00.log                 RVCP-SUMMARY: TEST PASSED - Test File "Zknh-sha512sig0h-00.S"
rv32i/Zknh/Zknh-sha512sig0l-00.log                 RVCP-SUMMARY: TEST PASSED - Test File "Zknh-sha512sig0l-00.S"
rv32i/Zknh/Zknh-sha512sig1h-00.log                 RVCP-SUMMARY: TEST PASSED - Test File "Zknh-sha512sig1h-00.S"
rv32i/Zknh/Zknh-sha512sig1l-00.log                 RVCP-SUMMARY: TEST PASSED - Test File "Zknh-sha512sig1l-00.S"
rv32i/Zknh/Zknh-sha512sum0r-00.log                 RVCP-SUMMARY: TEST PASSED - Test File "Zknh-sha512sum0r-00.S"
rv32i/Zknh/Zknh-sha512sum1r-00.log                 RVCP-SUMMARY: TEST PASSED - Test File "Zknh-sha512sum1r-00.S"
rv32i/Zksed/Zksed-sm4ed-00.log                     RVCP-SUMMARY: TEST PASSED - Test File "Zksed-sm4ed-00.S"
rv32i/Zksed/Zksed-sm4ks-00.log                     RVCP-SUMMARY: TEST PASSED - Test File "Zksed-sm4ks-00.S"
rv32i/Zksh/Zksh-sm3p0-00.log                       RVCP-SUMMARY: TEST PASSED - Test File "Zksh-sm3p0-00.S"
rv32i/Zksh/Zksh-sm3p1-00.log                       RVCP-SUMMARY: TEST PASSED - Test File "Zksh-sm3p1-00.S"
rv32i/Zmmul/Zmmul-mul-00.log                       RVCP-SUMMARY: TEST PASSED - Test File "Zmmul-mul-00.S"
rv32i/Zmmul/Zmmul-mulh-00.log                      RVCP-SUMMARY: TEST PASSED - Test File "Zmmul-mulh-00.S"
rv32i/Zmmul/Zmmul-mulhsu-00.log                    RVCP-SUMMARY: TEST PASSED - Test File "Zmmul-mulhsu-00.S"
rv32i/Zmmul/Zmmul-mulhu-00.log                     RVCP-SUMMARY: TEST PASSED - Test File "Zmmul-mulhu-00.S"
```
