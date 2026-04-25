# NEORV32 M performance measurement test

This code piece allows the measurement of the number of cycles of various I instructions.
The possible instructions to test are:
- mult : mul, mulh, mulhsu, mulhu
- div  : div, divu
- rem  : rem, remu

The number of instructions run can be tuned by setting the following command line parameters:
`USER_FLAGS+=-DinstLoop=1`    This tunes the number loops run, default 1
`USER_FLAGS+=-DinstCalls=256` This tunes the number of instructions called per inner loop, default 256.
The instCalls variable impacts memory, as each instruction instance takes up memory.

The limit the performance image size which instructions that can be tested can be controlled the following comand line parameters. The name of the parameter matches the list of instruction groups above:
`USER_FLAGS+=-Drv32I_mult`
`USER_FLAGS+=-Drv32I_div`
`USER_FLAGS+=-Drv32I_rem`
`USER_FLAGS+=-Drv32I_all` Run all instruction tests, the image will be large

For less verbose output `USER_FLAGS+=-DSILENT_MODE=1` can be applied

## Example compile and run
This will run all the M instruction suites

```
make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-Drv32M_all clean_all exe
make sim
```

## Exemplary Test Output

```
<<< M performance test >>>

perform: for (i=0;i<1,i++) {256 instructions}

mul tot. 1491 cyc

total 1491 cyc

mul rd,rs1,rs2 inst. 5 cyc

mulh tot. 1524 cyc

total 3015 cyc

mulh rd,rs1,rs2 inst. 5 cyc

mulhsu tot. 1491 cyc

total 4506 cyc

mulhsu rd,rs1,rs2 inst. 5 cyc

mulhu tot. 1491 cyc

total 5997 cyc

mulhu rd,imm inst. 5 cyc

div tot. 8963 cyc

total 14960 cyc

div rd,rs1,rs2 inst. 35 cyc

divu tot. 8963 cyc

total 23923 cyc

divu rd,rs1,shamt inst. 35 cyc

rem tot. 8963 cyc

total 32886 cyc

rem rd,rs1,rs2 inst. 35 cyc

remu tot. 8963 cyc

total 41849 cyc

remu rd,rs1,rs2 inst. 35 cyc

instructions tested: 8

total 41849 cycles

avg. inst. execute cyles 20.434
```
