## Dhrystone Benchmark

**:warning: The Dhrystone port is outdated. Have a look at the CoreMark port for benchmarking.**

:copyright: Original sources from [https://github.com/sifive/benchmark-dhrystone](https://github.com/sifive/benchmark-dhrystone);
 see `LICENSE` file. The original source code has been modified for the NEORV32 RISC-V Processor.

To compile the `main.exe` executable:

```bash
neorv32/sw/example/dhrystone$ make USER_FLAGS+=-DRUN_DHRYSTONE MARCH=rv32imc_zicsr_zifencei EFFORT=-O2 clean_all exe
...
Memory utilization:
   text    data     bss     dec     hex filename
   7976       0   10448   18424    47f8 main.elf
Compiling ../../../sw/image_gen/image_gen
Executable (neorv32_exe.bin) size in bytes:
7988
```

The default number of iterations is 10000. You can modify this by adding `USER_FLAGS+=-DDHRY_ITERS=2000000` to the makefile invocation.
Dhrystone will require an IMEM size of at least 8kB and a DMEM size of about 11kB. The CLINT machine timer is used for time benchmarking.
Note that the Drhystone score is normalized to the original VAX machine (SiFive is giving a nice overview
about this at https://www.sifive.com/blog/dhrystone-performance-tuning-on-the-freedom-platform):

```
VAX DMIPS/s/MHz = dhrystone_iterations / execution_cycles * clock_frequency_in_hz / 1757
```

### Exemplary Output

Output generated for processor HW version [v1.9.9.2](https://github.com/stnolting/neorv32/blob/main/CHANGELOG.md).
All results only show the integer parts.

```
NEORV32: Processor running at 100000000 Hz
NEORV32: Executing Dhrystone (10000 iterations). This may take some time...


Dhrystone Benchmark, Version 2.1 (Language: C)

Program compiled without 'register' attribute

Execution starts, 10000 runs through Dhrystone
Execution ends

Final values of the variables used in the benchmark:

Int_Glob:            5
        should be:   5
Bool_Glob:           1
        should be:   1
Ch_1_Glob:           A
        should be:   A
Ch_2_Glob:           B
        should be:   B
Arr_1_Glob[8]:       7
        should be:   7
Arr_2_Glob[8][7]:    10010
        should be:   Number_Of_Runs + 10
Ptr_Glob->
  Ptr_Comp:          2147483732
        should be:   (implementation-dependent)
  Discr:             0
        should be:   0
  Enum_Comp:         2
        should be:   2
  Int_Comp:          17
        should be:   17
  Str_Comp:          DHRYSTONE PROGRAM, SOME STRING
        should be:   DHRYSTONE PROGRAM, SOME STRING
Next_Ptr_Glob->
  Ptr_Comp:          2147483732
        should be:   (implementation-dependent), same as above
  Discr:             0
        should be:   0
  Enum_Comp:         1
        should be:   1
  Int_Comp:          18
        should be:   18
  Str_Comp:          DHRYSTONE PROGRAM, SOME STRING
        should be:   DHRYSTONE PROGRAM, SOME STRING
Int_1_Loc:           5
        should be:   5
Int_2_Loc:           13
        should be:   13
Int_3_Loc:           7
        should be:   7
Enum_Loc:            1
        should be:   1
Str_1_Loc:           DHRYSTONE PROGRAM, 1'ST STRING
        should be:   DHRYSTONE PROGRAM, 1'ST STRING
Str_2_Loc:           DHRYSTONE PROGRAM, 2'ND STRING
        should be:   DHRYSTONE PROGRAM, 2'ND STRING

Microseconds for one run through Dhrystone: 13
Dhrystones per Second:                      72939

NEORV32: << DETAILED RESULTS (integer parts only) >>
NEORV32: Total cycles:      13710151
NEORV32: Cycles per second: 100000000
NEORV32: Total runs:        10000

NEORV32: DMIPS/s:           72939
NEORV32: DMIPS/s/MHz:       729

NEORV32: VAX DMIPS/s:       41
NEORV32: VAX DMIPS/s/MHz:   729/1757
```
