# NEORV32 I performance measurement test

This code piece allows the measurement of the number of cycles of various I instructions.
The possible instructions to test are:
arith : add, addi, sub, lui, auipc
shift : sll, slli, srl, srli, sra, srai
logic : xor, xori, or, ori, and, andi
comp  : slt, slti, sltu, sltiu
sync  : fence, fence.i
branch: beq, bne, blt, bgem bltu, bgeu
jump  : jal, jalr
load  : lb, lh, lbu, lhu, lw
store : sb, sh, sw
CSR   : csrrw, csrrs, csrrc, csrrw, csrrsi, csrrci

The number of instructions run can be tuned by setting the following command line parameters:
`USER_FLAGS+=-DinstLoop=1`    This tunes the number loops run, default 1
`USER_FLAGS+=-DinstCalls=256` This tunes the number of instructions called per inner loop, default 256.
The instCalls variable impacts memory, as each instruction instance takes up memory.

The limit the performance image size which instructions that can be tested can be controlled the following comand line parameters. The name of the parameter matches the list of instruction groups above:
`USER_FLAGS+=-Drv32I_arith`
`USER_FLAGS+=-Drv32I_shift`
`USER_FLAGS+=-Drv32I_logic`
`USER_FLAGS+=-Drv32I_comp`
`USER_FLAGS+=-Drv32I_load`
`USER_FLAGS+=-Drv32I_store`
`USER_FLAGS+=-Drv32I_branch_beq`
`USER_FLAGS+=-Drv32I_branch_bne`
`USER_FLAGS+=-Drv32I_branch_blt`
`USER_FLAGS+=-Drv32I_branch_bge`
`USER_FLAGS+=-Drv32I_branch_bltu`
`USER_FLAGS+=-Drv32I_branch_bgeu`
`USER_FLAGS+=-Drv32I_jump`
`USER_FLAGS+=-Drv32I_sync`
`USER_FLAGS+=-Drv32I_env` This is ecall and ebreak. The test is currently not implemented.
`USER_FLAGS+=-Drv32I_csr`
`USER_FLAGS+=-Drv32I_mret`
`USER_FLAGS+=-Drv32I_all` Run all instruction tests, the image will be large

For the branch instructions 3 numbers are provided:
- No branch: The branch is not taken
- Branch forward: The branch is taken and the target is ahead of the branch instruction.
- Branch backward: The branch is taken and the target is behind the branch instruction. This will trigger the default branch predictor.

For the `JALR` instruction there is an additional parameter:
`USER_FLAGS+=-Drv32I_jalr_auipc_cycles` This set the number of cycles AUIPC takes, default is 2. This is used to offset the JALR cycle count as we need to use an AUIPC instruction in conjunction with JALR

For the `MRET` instruction there is an additional parameter:
`USER_FLAGS+=-Drv32I_mret_jal_csrw_cycles` This set the number of cycles that a JAL and CSRW instruction takes, default is 11. This is used to offset the MRET cycle count as we need set MTVEC and JAL to the MRET instruction during the measurement.

For less verbose output `USER_FLAGS+=-DSILENT_MODE=1` can be applied

## Compiler warning!!
The built in assembly assumes that C (compressed instruction) extension is not applied. If C is used the NOPs required for branch instructions to function will be the wrong size.

## Example compile and run
This will run the Arith instruction suite

```
make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-Drv32I_arith clean_all exe
make sim
```

## Exemplary Test Output

```
<<< I performance test >>>

perform: for (i=0;i<1,i++) {256 instructions}

add tot. 1058 cyc

total 1058 cyc

add rd,rs1,rs2 inst. 4 cyc

addi tot. 1058 cyc

total 2116 cyc

addi rd,rs1,imm inst. 4 cyc

sub tot. 1058 cyc

total 3174 cyc

sub rd,rs1,rs2 inst. 4 cyc

lui tot. 1058 cyc

total 4232 cyc

lui rd,imm inst. 4 cyc

auipc tot. 1058 cyc

total 5290 cyc

auipc rd,imm inst. 4 cyc

instructions tested: 5

total 5290 cycles

avg. inst. execute cyles 4.132
```
