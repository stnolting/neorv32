# NEORV32 Zfinx performance measurement test

This code piece allows the measurement of the number of cycles of various I instructions.
The possible instructions to test are:
- conv     : fcvt.s.w, fcvt.s.wu, fcvt.w.s, fcvt.wu.s
- arith    : fadd, fsub, fmul, fdiv [unsupported], fsqrt [unsupported]
- mult_add : fmadd [unsupported], fmsub [unsupported], fmnadd [unsupported], fmnsub [unsupported],
- sign     : fsgnj, fsgnjn, fsgnjx
- minmax   : fmin, fmax
- comp     : feq, flt, fle
- cat      : fclass
- csr      : frcsr, frrm, frflags, fscsr, fsrm, fsflags, fsrmi, fsflags

The number of instructions run can be tuned by setting the following command line parameters:
`USER_FLAGS+=-DinstLoop=1`    This tunes the number loops run, default 1
`USER_FLAGS+=-DinstCalls=256` This tunes the number of instructions called per inner loop, default 256.
The instCalls variable impacts memory, as each instruction instance takes up memory.

The limit the performance image size which instructions that can be tested can be controlled the following comand line parameters. The name of the parameter matches the list of instruction groups above:
- `USER_FLAGS+=-Drv32Zfinx_conv`
- `USER_FLAGS+=-Drv32Zfinx_arith`
- `USER_FLAGS+=-Drv32Zfinx_mult_add`
- `USER_FLAGS+=-Drv32Zfinx_sign`
- `USER_FLAGS+=-Drv32Zfinx_minmax`
- `USER_FLAGS+=-Drv32Zfinx_comp`
- `USER_FLAGS+=-Drv32Zfinx_cat`
- `USER_FLAGS+=-Drv32Zfinx_csr`
- `USER_FLAGS+=-Drv32I_all` Run all instruction tests, the image will be large

For less verbose output `USER_FLAGS+=-DSILENT_MODE=1` can be applied

## Float performance measurements
For the following class of instructions we perform multiple measurements
- Arith: measures exception, min and max
- Conv: measures min and max

The measurements are:
- Exception path: This has the inputs set to an value that can trigger an optimization bypass path through a multi-cycle instruction, e.g. +0.0 or +inf
- Min: This has the inputs set to a value that will trigger the shortes path through a multi-cycle instruction
- Max: This had the inputs set to a value that will trigger the longest path through a multi-cycle instruction


## Example compile and run
This will run all the Zfinx instruction suites

```
make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-Drv32Zfinx_arith clean_all exe
make sim
```

## Exemplary Test Output

```
<<< Zfinx performance test >>>

perform: for (i=0;i<1,i++) {256 instructions}

fadd.s tot. 3507 cyc

total 3507 cyc

fadd.s rd,rs1,rs2 exception path inst. 13 cyc

fadd.s tot. 5123 cyc

total 8630 cyc

fadd.s rd,rs1,rs2 min. inst. 20 cyc

fadd.s tot. 11779 cyc

total 20409 cyc

fadd.s rd,rs1,rs2 max. inst. 46 cyc

fsub.s tot. 3507 cyc

total 23916 cyc

fsub.s rd,rs1,rs2 exception path inst. 13 cyc

fsub.s tot. 5412 cyc

total 29328 cyc

fsub.s rd,rs1,rs2 min. inst. 21 cyc

fsub.s tot. 12291 cyc

total 41619 cyc

fsub.s rd,rs1,rs2 max. inst. 48 cyc

fmul.s tot. 3059 cyc

total 44678 cyc

fmul.s rd,rs1,rs2 exception path inst. 11 cyc

fmul.s tot. 4403 cyc

total 49081 cyc

fmul.s rd,rs1,rs2 inst. 17 cyc

instructions tested: 8

total 49081 cycles

avg. inst. execute cyles 23.965
```
