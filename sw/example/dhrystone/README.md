# Dhrystone Benchmark

:copyright: Original sources from [https://github.com/sifive/benchmark-dhrystone](https://github.com/sifive/benchmark-dhrystone) (also, see `LICENSE`).
The source code has been modified for the NEORV32 processor.

To compile the NEORV32 executable:
```
neorv32/sw/example/dhrystone$ sh dhrystone.sh
```

The default number of iterations is 2000000. You can modify this by changing `USER_FLAGS+=-DDHRY_ITERS=2000000` in `dhrystone.sh`.
The default optimization level (EFFORT) is `O3`.

:warning: Dhrystone will require an IMEM size of 16kB and a DMEM size of 16kB. The MTIME machine timer is used for timing evaluation.

:construction: Porting Dhrystone is still work-in-progress. Performance results might be incorrect and not optimized.
All results only show the integer parts.

### Exemplary Output

Output generated for processor HW version [v1.5.9.1](https://github.com/stnolting/neorv32/blob/main/CHANGELOG.md)
using performance-optimized configuration options.

```
NEORV32: Processor running at 100000000 Hz
NEORV32: Executing Dhrystone (2000000 iterations). This may take some time...


Dhrystone Benchmark, Version 2.1 (Language: C)

Program compiled without 'register' attribute

Execution starts, 2000000 runs through Dhrystone
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
Arr_2_Glob[8][7]:    2000010
        should be:   Number_Of_Runs + 10
Ptr_Glob->
  Ptr_Comp:          -2147467428
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
  Ptr_Comp:          -2147467428
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
Dhrystones per Second:                      76923

NEORV32: << DETAILED RESULTS (integer parts only) >>
NEORV32: Total cycles:      2662000187
NEORV32: Cycles per second: 100000000
NEORV32: Total runs:        2000000

NEORV32: DMIPS/s:           76923
NEORV32: DMIPS/MHz:         769

NEORV32: VAX DMIPS/s:       43
NEORV32: VAX DMIPS/MHz:     43/100
```
