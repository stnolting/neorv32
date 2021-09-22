// SPDX-License-Identifier: BSD-3-Clause

// Modified by Stephan Nolting for the NEORV32 Processor

#ifndef _COMPLIANCE_MODEL_H
#define _COMPLIANCE_MODEL_H

#define RVMODEL_DATA_SECTION \
        .pushsection .tohost,"aw",@progbits;                                  \
        .align 8; .global tohost; tohost: .dword 0;                           \
        .align 8; .global fromhost; fromhost: .dword 0;                       \
        .popsection;                                                          \
        .align 8; .global begin_regstate; begin_regstate:                     \
        .word 128;                                                            \
        .align 8; .global end_regstate; end_regstate:                         \
        .word 4;

//RV_COMPLIANCE_HALT
// neorv32: this will dump the results via the UART0_SIM_MODE data file output
// neorv32: due to the modifications on "end_signature" (not 4-aligned) we need to make sure we output a 4-aligned number of data here
// neorv32: -> for zero-padding of the rest of the SIGNATURE section
#define RVMODEL_HALT                                                          \
      signature_dump:                                                         \
  			la   a0, begin_signature;                                             \
				la   a1, end_signature;                                               \
        li   a2, 0xFFFFFFA4;                                                  \
      signature_dump_loop:                                                    \
        beq  a0, a1, signature_dump_padding;                                  \
        lw   t0, 0(a0);                                                       \
        sw   t0, 0(a2);                                                       \
        addi a0, a0, 4;                                                       \
        j    signature_dump_loop;                                             \
nop;                                                                          \
nop;                                                                          \
      signature_dump_padding:                                                 \
        andi a0, a1, 0x0000000C;                                              \
        beq  a0, zero, signature_dump_end;                                    \
        li   t0, 16;                                                          \
        sub  a0, t0, a0;                                                      \
      signature_dump_padding_loop:                                            \
        beq  a0, zero, signature_dump_end;                                    \
        sw   zero, 0(a2);                                                     \
        addi a0, a0, -4;                                                      \
        j    signature_dump_padding_loop;                                     \
      signature_dump_end:                                                     \
        j    signature_dump_end

//TODO: declare the start of your signature region here. Nothing else to be used here.
// The .align 4 ensures that the signature ends at a 16-byte boundary
#define RVMODEL_DATA_BEGIN                                                    \
  .align 4; .global begin_signature; begin_signature:

//TODO: declare the end of the signature region here. Add other target specific contents here.
//neorv32: DO NOT use align_4 here! end_signature is used to indicate the actual "number" of signature words
#define RVMODEL_DATA_END                                                      \
  .global end_signature; end_signature:                                       \
  RVMODEL_DATA_SECTION

//RVMODEL_BOOT
// neorv32: enable UART0 (ctrl(28)) and enable UART0_SIM_MODE (ctrl(12))
// neorv32: initialize the complete RVTEST_DATA section in data RAM (DMEM) with 0xBABECAFE
// neorv32: initialize the complete SIGNATURE section (that is a multiple of four 32-bit entries) in data RAM (DMEM) with 0xDEADBEEF
// neorv32: this code also provides a dummy trap handler that just moves on to the next instruction
// neorv32: -> this trap handler can be overridden by the compliance-suite by modifying mtval
// neorv32: -> the dummy trap handler is required to deal with the neorv32 X extension (-> all illegal/undefined instruction trigger an exception)
#ifdef NEORV32_NO_DATA_INIT
// ------------------------- WITHOUT DATA INIT -------------------------
#define RVMODEL_BOOT                                                          \
      core_init:                                                              \
        la x1, core_dummy_trap_handler;                                       \
        csrw   mtvec, x1;                                                     \
        csrw   mie, x0;                                                       \
        j      uart0_sim_mode_init;                                           \
nop;                                                                          \
nop;                                                                          \
      .balign 4;                                                              \
      core_dummy_trap_handler:                                                \
        csrw  mscratch, sp;                                                   \
        la    sp, end_signature;                                              \
        addi  sp, sp, 32;                                                     \
        sw	  x8, 0(sp);                                                      \
        sw	  x9, 4(sp);                                                      \
        csrr  x8, mcause;                                                     \
        blt   x8, zero, core_dummy_trap_handler_irq;                          \
        csrr  x8, mepc;                                                       \
      core_dummy_trap_handler_exc_c_check:                                    \
        lh    x9, 0(x8);                                                      \
        andi  x9, x9, 3;                                                      \
        addi  x8, x8, +2;                                                     \
        csrw  mepc, x8;                                                       \
        addi  x8, zero, 3;                                                    \
        bne   x8, x9, core_dummy_trap_handler_irq;                            \
      core_dummy_trap_handler_exc_uncrompressed:                              \
        csrr  x8, mepc;                                                       \
        addi  x8, x8, +2;                                                     \
        csrw  mepc, x8;                                                       \
      core_dummy_trap_handler_irq:                                            \
        lw    x9, 0(sp);                                                      \
        lw    x8, 4(sp);                                                      \
        csrr  sp, mscratch;                                                   \
        mret;                                                                 \
nop;                                                                          \
nop;                                                                          \
      uart0_sim_mode_init:                                                    \
        li    a0,   0xFFFFFFA0;                                               \
        sw    zero, 0(a0);                                                    \
        li    a1,   1 << 28;                                                  \
        li    a2,   1 << 12;                                                  \
        or    a1,   a1, a2;                                                   \
        sw    a1,   0(a0);

#else

// ------------------------- WITH DATA INIT -------------------------
#define RVMODEL_BOOT                                                          \
      core_init:                                                              \
        la x1, core_dummy_trap_handler;                                       \
        csrw   mtvec, x1;                                                     \
        csrw   mie, x0;                                                       \
nop;                                                                          \
nop;                                                                          \
      init_rvtest_data:                                                       \
  			la   a0, rvtest_data_begin;                                           \
				la   a1, rvtest_data_end;                                             \
        li   a2, 0xBABECAFE;                                                  \
      init_rvtest_data_loop:                                                  \
        beq  a0, a1, init_rvtest_data_loop_end;                               \
        sw   a2, 0(a0);                                                       \
        addi a0, a0, 4;                                                       \
        j    init_rvtest_data_loop;                                           \
      init_rvtest_data_loop_end:                                              \
nop;                                                                          \
nop;                                                                          \
      init_signature:                                                         \
  			la   a0, begin_signature;                                             \
				la   a1, end_signature;                                               \
        li   a2, 0xDEADBEEF;                                                  \
      init_signature_loop:                                                    \
        beq  a0, a1, init_signature_loop_end;                                 \
        sw   a2, 0(a0);                                                       \
        addi a0, a0, 4;                                                       \
        j    init_signature_loop;                                             \
      init_signature_loop_end:                                                \
        j    uart0_sim_mode_init;                                             \
nop;                                                                          \
nop;                                                                          \
      .balign 4;                                                              \
      core_dummy_trap_handler:                                                \
        csrw  mscratch, sp;                                                   \
        la    sp, end_signature;                                              \
        addi  sp, sp, 32;                                                     \
        sw	  x8, 0(sp);                                                      \
        sw	  x9, 4(sp);                                                      \
        csrr  x8, mcause;                                                     \
        blt   x8, zero, core_dummy_trap_handler_irq;                          \
        csrr  x8, mepc;                                                       \
      core_dummy_trap_handler_exc_c_check:                                    \
        lh    x9, 0(x8);                                                      \
        andi  x9, x9, 3;                                                      \
        addi  x8, x8, +2;                                                     \
        csrw  mepc, x8;                                                       \
        addi  x8, zero, 3;                                                    \
        bne   x8, x9, core_dummy_trap_handler_irq;                            \
      core_dummy_trap_handler_exc_uncrompressed:                              \
        csrr  x8, mepc;                                                       \
        addi  x8, x8, +2;                                                     \
        csrw  mepc, x8;                                                       \
      core_dummy_trap_handler_irq:                                            \
        lw    x9, 0(sp);                                                      \
        lw    x8, 4(sp);                                                      \
        csrr  sp, mscratch;                                                   \
        mret;                                                                 \
nop;                                                                          \
nop;                                                                          \
      uart0_sim_mode_init:                                                    \
        li    a0,   0xFFFFFFA0;                                               \
        sw    zero, 0(a0);                                                    \
        li    a1,   1 << 28;                                                  \
        li    a2,   1 << 12;                                                  \
        or    a1,   a1, a2;                                                   \
        sw    a1,   0(a0);

#endif


//RVTEST_IO_INIT
#define RVMODEL_IO_INIT
//RVTEST_IO_WRITE_STR
#define RVMODEL_IO_WRITE_STR(_R, _STR)
//RVTEST_IO_CHECK
#define RVMODEL_IO_CHECK()

//RVTEST_IO_ASSERT_GPR_EQ
#define RVMODEL_IO_ASSERT_GPR_EQ(_S, _R, _I)
//RVTEST_IO_ASSERT_SFPR_EQ
#define RVMODEL_IO_ASSERT_SFPR_EQ(_F, _R, _I)
//RVTEST_IO_ASSERT_DFPR_EQ
#define RVMODEL_IO_ASSERT_DFPR_EQ(_D, _R, _I)

// TODO: specify the routine for setting machine software interrupt
#define RVMODEL_SET_MSW_INT

// TODO: specify the routine for clearing machine software interrupt
#define RVMODEL_CLEAR_MSW_INT

// TODO: specify the routine for clearing machine timer interrupt
#define RVMODEL_CLEAR_MTIMER_INT

// TODO: specify the routine for clearing machine external interrupt
#define RVMODEL_CLEAR_MEXT_INT

#endif // _COMPLIANCE_MODEL_H
