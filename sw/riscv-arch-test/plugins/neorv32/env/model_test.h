// model_test.h for the NEORV32 RISC-V Processor
// SPDX-License-Identifier: BSD-3-Clause

#ifndef _COMPLIANCE_MODEL_H
#define _COMPLIANCE_MODEL_H

#define ALIGNMENT 2

// output data layout
#define RVMODEL_DATA_SECTION \
  .align 8;                  \
  .global begin_regstate;    \
  begin_regstate:            \
  .word 128;                 \
  .align 8;                  \
  .global end_regstate;      \
  end_regstate:              \
  .word 4;

// start of the signature region
#define RVMODEL_DATA_BEGIN \
  RVMODEL_DATA_SECTION     \
  .align 4;                \
  .global begin_signature; \
  begin_signature:

// end of the signature region
#define RVMODEL_DATA_END \
  .align 4;              \
  .global end_signature; \
  end_signature:

// initializes IO for debug output: unused
#define RVMODEL_IO_INIT

// write string to console: unused
#define RVMODEL_IO_WRITE_STR(_R, _STR)

// debug assertion that GPR should have value: unused
#define RVMODEL_IO_ASSERT_GPR_EQ(_S, _R, _I)

// initialize hardware platform: install default trap handler to cancel run
// [note] use ".word 0x30551073" instead of "csrrw x0, mtvec, x10" as Zicsr might not be enabled
#define RVMODEL_BOOT           \
    la    x10, boot_terminate; \
    .word 0x30551073;          \
    j     boot_end;            \
  boot_terminate:              \
    li    x10, 0xF0000000;     \
    sw    x10, 8(x10);         \
    j     boot_terminate;      \
  boot_end:

// dump signature and terminate simulation
#define RVMODEL_HALT      \
  la x4, 0xF0000000;      \
  la x5, begin_signature; \
  la x6, end_signature;   \
  sw x5, 0(x4);           \
  sw x6, 4(x4);           \
  halt:                   \
    sw x0, 8(x4);         \
    j  halt

// address that causes an access fault when read or written
#define ACCESS_FAULT_ADDRESS 0xFFFFFF00

// PMP configuration
#define RVMODEL_NUM_PMPS 16
#define RVMODEL_PMP_GRAIN 0

// set machine software interrupt via CLINT
#define RVMODEL_SET_MSW_INT \
  li x10, 0xFFF40000;       \
  sw x10, -1(x10);

// clear machine software interrupt via CLINT
#define RVMODEL_CLR_MSW_INT \
  li x10,  0xFFF40000;      \
  sw zero, 0(x10);

// set machine timer interrupt via CLINT
#define RVMODEL_SET_MTIMER_INT \
  li x10,  0xFFF44000;         \
  sw zero, 4(x10);             \
  sw zero, 0(x10);

// clear machine timer interrupt via CLINT
#define RVMODEL_CLR_MTIMER_INT \
  li x10, 0xFFF44000;          \
  li x11, -1;                  \
  sw x11, 4(x10);              \
  sw x11, 0(x10);

// set machine external interrupt via testbench
#define RVMODEL_SET_MEXT_INT \
  li x10, 0xF0000000;        \
  li x11, 1<<11;             \
  sw x11, 12(x10);

// clear machine external interrupt via testbench
#define RVMODEL_CLR_MEXT_INT \
  li x10,  0xF0000000;       \
  sw zero, 12(x10);

#endif // _COMPLIANCE_MODEL_H
