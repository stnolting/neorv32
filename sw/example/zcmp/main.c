#include <neorv32.h>

// To get the full list of opcodes I assembled then disassembled this file:
//
// .set i, 0
// .rept 64
// .if (i & 0x3c) >= 16
// .hword 0xba02 | (i << 2)
// .hword 0xbc02 | (i << 2)
// .hword 0xbe02 | (i << 2)
// .endif
// .set i, i + 1
// .endr

#define BAUD_RATE 19200

#define FRAME_SIZE_WORDS 64
#define N_RESULTS 15
volatile uint32_t test_frame[FRAME_SIZE_WORDS];
volatile uint32_t test_results[N_RESULTS];
volatile uint32_t sp_save;
volatile uint32_t test_sp;

#define rlist(instr) ((instr & 0xf0) >> 4)
#define n_regs(instr) (rlist(instr) == 0xf ? 13 : rlist(instr) - 3)
#define spimm(instr) ((instr & 0x0c) >> 2)

// s0:  a5000008 x8
// s1:  a5000009 x9
// a0:  a500000a x10
// a1:  0000007b x11
// s2:  a5000012 x18
// s3:  a5000013 x19
// s4:  a5000014 x20
// s5:  a5000015 x21
// s6:  a5000016 x22
// s7:  a5000017 x23
// s8:  a5000018 x24
// s9:  a5000019 x25
// s10: a500001a x26
// s11: a500001b x27


#define stack_adj(instr) ((                                                         \
							  n_regs(instr) > 12 ? 0x40 : n_regs(instr) > 8 ? 0x30  \
													  : n_regs(instr) > 4	? 0x20  \
																			: 0x10) + \
						  spimm(instr) * 0x10)

#define test_zcmp_pop(instr_bits, instr_name)                                                           \
	test_sp = (uint32_t)&test_frame[0];                                                                 \
	neorv32_uart0_printf("Test: " instr_name "\n");                                                     \
	for (int i = 0; i < FRAME_SIZE_WORDS; ++i)                                                          \
		test_frame[i] = 0xdead0000 + i;                                                                 \
	asm volatile(/* Save all clobbered registers on the real stack */                                   \
				 "_test_%=:\n"                                                                          \
				 "addi sp, sp, -64\n"                                                                   \
				 "sw ra,  0(sp)\n"                                                                      \
				 "sw a0,  4(sp)\n"                                                                      \
				 "sw a1,  8(sp)\n"                                                                      \
				 "sw a2,  12(sp)\n"                                                                     \
				 "sw s0,  16(sp)\n"                                                                     \
				 "sw s1,  20(sp)\n"                                                                     \
				 "sw s2,  24(sp)\n"                                                                     \
				 "sw s3,  28(sp)\n"                                                                     \
				 "sw s4,  32(sp)\n"                                                                     \
				 "sw s5,  36(sp)\n"                                                                     \
				 "sw s6,  40(sp)\n"                                                                     \
				 "sw s7,  44(sp)\n"                                                                     \
				 "sw s8,  48(sp)\n"                                                                     \
				 "sw s9,  52(sp)\n"                                                                     \
				 "sw s10, 56(sp)\n"                                                                     \
				 "sw s11, 60(sp)\n" /* Save stack pointer and install test sp */                        \
				 "la a2, sp_save\n"                                                                     \
				 "sw sp, 0(a2)\n"                                                                       \
				 "la a2, test_sp\n"                                                                     \
				 "lw sp, 0(a2)\n" /* Store return address at calculated stack frame position */         \
				 "la a0, 1f\n"                                                                          \
				 "sw a0, %0(sp)\n" /* Give unique values to all registers that can be written by pop */ \
				 "li x1,  0xa5000000 + 1\n"                                                             \
				 "li x8,  0xa5000000 + 8\n"                                                             \
				 "li x9,  0xa5000000 + 9\n"                                                             \
				 "li x10, 0xa5000000 + 10\n"                                                            \
				 "li x18, 0xa5000000 + 18\n"                                                            \
				 "li x19, 0xa5000000 + 19\n"                                                            \
				 "li x20, 0xa5000000 + 20\n"                                                            \
				 "li x21, 0xa5000000 + 21\n"                                                            \
				 "li x22, 0xa5000000 + 22\n"                                                            \
				 "li x23, 0xa5000000 + 23\n"                                                            \
				 "li x24, 0xa5000000 + 24\n"                                                            \
				 "li x25, 0xa5000000 + 25\n"                                                            \
				 "li x26, 0xa5000000 + 26\n"                                                            \
				 "li x27, 0xa5000000 + 27\n" /* Let's go */                                             \
				 ".hword " #instr_bits "\n"	 /* Non-ret instructions should fall through to here */     \
				 "li a1, 123\n"                                                                         \
				 "j 2f\n" /* ret instructions should go here */                                         \
				 "1:\n"                                                                                 \
				 "li a1, 456\n"                                                                         \
				 "2:\n" /* Save out results */                                                          \
				 "la a2, test_results\n"                                                                \
				 "sw x1,  0(a2)\n"                                                                      \
				 "sw x8,  4(a2)\n"                                                                      \
				 "sw x9,  8(a2)\n"                                                                      \
				 "sw x10, 12(a2)\n"                                                                     \
				 "sw x11, 16(a2)\n"                                                                     \
				 "sw x18, 20(a2)\n"                                                                     \
				 "sw x19, 24(a2)\n"                                                                     \
				 "sw x20, 28(a2)\n"                                                                     \
				 "sw x21, 32(a2)\n"                                                                     \
				 "sw x22, 36(a2)\n"                                                                     \
				 "sw x23, 40(a2)\n"                                                                     \
				 "sw x24, 44(a2)\n"                                                                     \
				 "sw x25, 48(a2)\n"                                                                     \
				 "sw x26, 52(a2)\n"                                                                     \
				 "sw x27, 56(a2)\n" /* Write out new sp and restore original sp */                      \
				 "la a2, test_sp\n"                                                                     \
				 "sw sp, 0(a2)\n"                                                                       \
				 "la a2, sp_save\n"                                                                     \
				 "lw sp, 0(a2)\n" /* Restore clobbered registers */                                     \
				 "la x1, sp_save\n"                                                                     \
				 "lw ra,  0(sp)\n"                                                                      \
				 "lw a0,  4(sp)\n"                                                                      \
				 "lw a1,  8(sp)\n"                                                                      \
				 "lw a2,  12(sp)\n"                                                                     \
				 "lw s0,  16(sp)\n"                                                                     \
				 "lw s1,  20(sp)\n"                                                                     \
				 "lw s2,  24(sp)\n"                                                                     \
				 "lw s3,  28(sp)\n"                                                                     \
				 "lw s4,  32(sp)\n"                                                                     \
				 "lw s5,  36(sp)\n"                                                                     \
				 "lw s6,  40(sp)\n"                                                                     \
				 "lw s7,  44(sp)\n"                                                                     \
				 "lw s8,  48(sp)\n"                                                                     \
				 "lw s9,  52(sp)\n"                                                                     \
				 "lw s10, 56(sp)\n"                                                                     \
				 "lw s11, 60(sp)\n"                                                                     \
				 "addi sp, sp, 64\n"                                                                    \
				 : : "i"(stack_adj(instr_bits) - 4 * n_regs(instr_bits)));                              \
	neorv32_uart0_printf("SP diff: %x\n", (uint32_t)test_sp - (uint32_t)&test_frame[0]);              \
	neorv32_uart0_printf("s0:  %x\n", test_results[1]);                                                  \
	neorv32_uart0_printf("s1:  %x\n", test_results[2]);                                                  \
	neorv32_uart0_printf("a0:  %x\n", test_results[3]);                                                  \
	neorv32_uart0_printf("s2:  %x\n", test_results[5]);                                                  \
	neorv32_uart0_printf("s3:  %x\n", test_results[6]);                                                  \
	neorv32_uart0_printf("s4:  %x\n", test_results[7]);                                                  \
	neorv32_uart0_printf("s5:  %x\n", test_results[8]);                                                  \
	neorv32_uart0_printf("s6:  %x\n", test_results[9]);                                                  \
	neorv32_uart0_printf("s7:  %x\n", test_results[10]);                                                 \
	neorv32_uart0_printf("s8:  %x\n", test_results[11]);                                                 \
	neorv32_uart0_printf("s9:  %x\n", test_results[12]);                                                 \
	neorv32_uart0_printf("s10: %x\n", test_results[13]);                                                \
	neorv32_uart0_printf("s11: %x\n", test_results[14]);                                                \
	neorv32_uart0_printf("\n");

int main()
{
	neorv32_rte_setup();

	// setup UART at default baud rate, no interrupts
	neorv32_uart0_setup(BAUD_RATE, 0);

	neorv32_uart0_printf("\n");

	test_zcmp_pop(0xba42, "cm.pop     {ra},16");
	test_zcmp_pop(0xbc42, "cm.popretz {ra},16");
	test_zcmp_pop(0xbe42, "cm.popret  {ra},16");

	test_zcmp_pop(0xba46, "cm.pop     {ra},32");
	test_zcmp_pop(0xbc46, "cm.popretz {ra},32");
	test_zcmp_pop(0xbe46, "cm.popret  {ra},32");

	test_zcmp_pop(0xba4a, "cm.pop     {ra},48");
	test_zcmp_pop(0xbc4a, "cm.popretz {ra},48");
	test_zcmp_pop(0xbe4a, "cm.popret  {ra},48");

	test_zcmp_pop(0xba4e, "cm.pop     {ra},64");
	test_zcmp_pop(0xbc4e, "cm.popretz {ra},64");
	test_zcmp_pop(0xbe4e, "cm.popret  {ra},64");

	test_zcmp_pop(0xba52, "cm.pop     {ra,s0},16");
	test_zcmp_pop(0xbc52, "cm.popretz {ra,s0},16");
	test_zcmp_pop(0xbe52, "cm.popret  {ra,s0},16");

	test_zcmp_pop(0xba56, "cm.pop     {ra,s0},32");
	test_zcmp_pop(0xbc56, "cm.popretz {ra,s0},32");
	test_zcmp_pop(0xbe56, "cm.popret  {ra,s0},32");

	test_zcmp_pop(0xba5a, "cm.pop     {ra,s0},48");
	test_zcmp_pop(0xbc5a, "cm.popretz {ra,s0},48");
	test_zcmp_pop(0xbe5a, "cm.popret  {ra,s0},48");

	test_zcmp_pop(0xba5e, "cm.pop     {ra,s0},64");
	test_zcmp_pop(0xbc5e, "cm.popretz {ra,s0},64");
	test_zcmp_pop(0xbe5e, "cm.popret  {ra,s0},64");

	test_zcmp_pop(0xba62, "cm.pop     {ra,s0-s1},16");
	test_zcmp_pop(0xbc62, "cm.popretz {ra,s0-s1},16");
	test_zcmp_pop(0xbe62, "cm.popret  {ra,s0-s1},16");

	test_zcmp_pop(0xba66, "cm.pop     {ra,s0-s1},32");
	test_zcmp_pop(0xbc66, "cm.popretz {ra,s0-s1},32");
	test_zcmp_pop(0xbe66, "cm.popret  {ra,s0-s1},32");

	test_zcmp_pop(0xba6a, "cm.pop     {ra,s0-s1},48");
	test_zcmp_pop(0xbc6a, "cm.popretz {ra,s0-s1},48");
	test_zcmp_pop(0xbe6a, "cm.popret  {ra,s0-s1},48");

	test_zcmp_pop(0xba6e, "cm.pop     {ra,s0-s1},64");
	test_zcmp_pop(0xbc6e, "cm.popretz {ra,s0-s1},64");
	test_zcmp_pop(0xbe6e, "cm.popret  {ra,s0-s1},64");

	test_zcmp_pop(0xba72, "cm.pop     {ra,s0-s2},16");
	test_zcmp_pop(0xbc72, "cm.popretz {ra,s0-s2},16");
	test_zcmp_pop(0xbe72, "cm.popret  {ra,s0-s2},16");

	test_zcmp_pop(0xba76, "cm.pop     {ra,s0-s2},32");
	test_zcmp_pop(0xbc76, "cm.popretz {ra,s0-s2},32");
	test_zcmp_pop(0xbe76, "cm.popret  {ra,s0-s2},32");

	test_zcmp_pop(0xba7a, "cm.pop     {ra,s0-s2},48");
	test_zcmp_pop(0xbc7a, "cm.popretz {ra,s0-s2},48");
	test_zcmp_pop(0xbe7a, "cm.popret  {ra,s0-s2},48");

	test_zcmp_pop(0xba7e, "cm.pop     {ra,s0-s2},64");
	test_zcmp_pop(0xbc7e, "cm.popretz {ra,s0-s2},64");
	test_zcmp_pop(0xbe7e, "cm.popret  {ra,s0-s2},64");

	test_zcmp_pop(0xba82, "cm.pop     {ra,s0-s3},32");
	test_zcmp_pop(0xbc82, "cm.popretz {ra,s0-s3},32");
	test_zcmp_pop(0xbe82, "cm.popret  {ra,s0-s3},32");

	test_zcmp_pop(0xba86, "cm.pop     {ra,s0-s3},48");
	test_zcmp_pop(0xbc86, "cm.popretz {ra,s0-s3},48");
	test_zcmp_pop(0xbe86, "cm.popret  {ra,s0-s3},48");

	test_zcmp_pop(0xba8a, "cm.pop     {ra,s0-s3},64");
	test_zcmp_pop(0xbc8a, "cm.popretz {ra,s0-s3},64");
	test_zcmp_pop(0xbe8a, "cm.popret  {ra,s0-s3},64");

	test_zcmp_pop(0xba8e, "cm.pop     {ra,s0-s3},80");
	test_zcmp_pop(0xbc8e, "cm.popretz {ra,s0-s3},80");
	test_zcmp_pop(0xbe8e, "cm.popret  {ra,s0-s3},80");

	test_zcmp_pop(0xba92, "cm.pop     {ra,s0-s4},32");
	test_zcmp_pop(0xbc92, "cm.popretz {ra,s0-s4},32");
	test_zcmp_pop(0xbe92, "cm.popret  {ra,s0-s4},32");

	test_zcmp_pop(0xba96, "cm.pop     {ra,s0-s4},48");
	test_zcmp_pop(0xbc96, "cm.popretz {ra,s0-s4},48");
	test_zcmp_pop(0xbe96, "cm.popret  {ra,s0-s4},48");

	test_zcmp_pop(0xba9a, "cm.pop     {ra,s0-s4},64");
	test_zcmp_pop(0xbc9a, "cm.popretz {ra,s0-s4},64");
	test_zcmp_pop(0xbe9a, "cm.popret  {ra,s0-s4},64");

	test_zcmp_pop(0xba9e, "cm.pop     {ra,s0-s4},80");
	test_zcmp_pop(0xbc9e, "cm.popretz {ra,s0-s4},80");
	test_zcmp_pop(0xbe9e, "cm.popret  {ra,s0-s4},80");

	test_zcmp_pop(0xbaa2, "cm.pop     {ra,s0-s5},32");
	test_zcmp_pop(0xbca2, "cm.popretz {ra,s0-s5},32");
	test_zcmp_pop(0xbea2, "cm.popret  {ra,s0-s5},32");

	test_zcmp_pop(0xbaa6, "cm.pop     {ra,s0-s5},48");
	test_zcmp_pop(0xbca6, "cm.popretz {ra,s0-s5},48");
	test_zcmp_pop(0xbea6, "cm.popret  {ra,s0-s5},48");

	test_zcmp_pop(0xbaaa, "cm.pop     {ra,s0-s5},64");
	test_zcmp_pop(0xbcaa, "cm.popretz {ra,s0-s5},64");
	test_zcmp_pop(0xbeaa, "cm.popret  {ra,s0-s5},64");

	test_zcmp_pop(0xbaae, "cm.pop     {ra,s0-s5},80");
	test_zcmp_pop(0xbcae, "cm.popretz {ra,s0-s5},80");
	test_zcmp_pop(0xbeae, "cm.popret  {ra,s0-s5},80");

	test_zcmp_pop(0xbab2, "cm.pop     {ra,s0-s6},32");
	test_zcmp_pop(0xbcb2, "cm.popretz {ra,s0-s6},32");
	test_zcmp_pop(0xbeb2, "cm.popret  {ra,s0-s6},32");

	test_zcmp_pop(0xbab6, "cm.pop     {ra,s0-s6},48");
	test_zcmp_pop(0xbcb6, "cm.popretz {ra,s0-s6},48");
	test_zcmp_pop(0xbeb6, "cm.popret  {ra,s0-s6},48");

	test_zcmp_pop(0xbaba, "cm.pop     {ra,s0-s6},64");
	test_zcmp_pop(0xbcba, "cm.popretz {ra,s0-s6},64");
	test_zcmp_pop(0xbeba, "cm.popret  {ra,s0-s6},64");

	test_zcmp_pop(0xbabe, "cm.pop     {ra,s0-s6},80");
	test_zcmp_pop(0xbcbe, "cm.popretz {ra,s0-s6},80");
	test_zcmp_pop(0xbebe, "cm.popret  {ra,s0-s6},80");

	test_zcmp_pop(0xbac2, "cm.pop     {ra,s0-s7},48");
	test_zcmp_pop(0xbcc2, "cm.popretz {ra,s0-s7},48");
	test_zcmp_pop(0xbec2, "cm.popret  {ra,s0-s7},48");

	test_zcmp_pop(0xbac6, "cm.pop     {ra,s0-s7},64");
	test_zcmp_pop(0xbcc6, "cm.popretz {ra,s0-s7},64");
	test_zcmp_pop(0xbec6, "cm.popret  {ra,s0-s7},64");

	test_zcmp_pop(0xbaca, "cm.pop     {ra,s0-s7},80");
	test_zcmp_pop(0xbcca, "cm.popretz {ra,s0-s7},80");
	test_zcmp_pop(0xbeca, "cm.popret  {ra,s0-s7},80");

	test_zcmp_pop(0xbace, "cm.pop     {ra,s0-s7},96");
	test_zcmp_pop(0xbcce, "cm.popretz {ra,s0-s7},96");
	test_zcmp_pop(0xbece, "cm.popret  {ra,s0-s7},96");

	test_zcmp_pop(0xbad2, "cm.pop     {ra,s0-s8},48");
	test_zcmp_pop(0xbcd2, "cm.popretz {ra,s0-s8},48");
	test_zcmp_pop(0xbed2, "cm.popret  {ra,s0-s8},48");

	test_zcmp_pop(0xbad6, "cm.pop     {ra,s0-s8},64");
	test_zcmp_pop(0xbcd6, "cm.popretz {ra,s0-s8},64");
	test_zcmp_pop(0xbed6, "cm.popret  {ra,s0-s8},64");

	test_zcmp_pop(0xbada, "cm.pop     {ra,s0-s8},80");
	test_zcmp_pop(0xbcda, "cm.popretz {ra,s0-s8},80");
	test_zcmp_pop(0xbeda, "cm.popret  {ra,s0-s8},80");

	test_zcmp_pop(0xbade, "cm.pop     {ra,s0-s8},96");
	test_zcmp_pop(0xbcde, "cm.popretz {ra,s0-s8},96");
	test_zcmp_pop(0xbede, "cm.popret  {ra,s0-s8},96");

	test_zcmp_pop(0xbae2, "cm.pop     {ra,s0-s9},48");
	test_zcmp_pop(0xbce2, "cm.popretz {ra,s0-s9},48");
	test_zcmp_pop(0xbee2, "cm.popret  {ra,s0-s9},48");

	test_zcmp_pop(0xbae6, "cm.pop     {ra,s0-s9},64");
	test_zcmp_pop(0xbce6, "cm.popretz {ra,s0-s9},64");
	test_zcmp_pop(0xbee6, "cm.popret  {ra,s0-s9},64");

	test_zcmp_pop(0xbaea, "cm.pop     {ra,s0-s9},80");
	test_zcmp_pop(0xbcea, "cm.popretz {ra,s0-s9},80");
	test_zcmp_pop(0xbeea, "cm.popret  {ra,s0-s9},80");

	test_zcmp_pop(0xbaee, "cm.pop     {ra,s0-s9},96");
	test_zcmp_pop(0xbcee, "cm.popretz {ra,s0-s9},96");
	test_zcmp_pop(0xbeee, "cm.popret  {ra,s0-s9},96");

	test_zcmp_pop(0xbaf2, "cm.pop     {ra,s0-s11},64");
	test_zcmp_pop(0xbcf2, "cm.popretz {ra,s0-s11},64");
	test_zcmp_pop(0xbef2, "cm.popret  {ra,s0-s11},64");

	test_zcmp_pop(0xbaf6, "cm.pop     {ra,s0-s11},80");
	test_zcmp_pop(0xbcf6, "cm.popretz {ra,s0-s11},80");
	test_zcmp_pop(0xbef6, "cm.popret  {ra,s0-s11},80");

	test_zcmp_pop(0xbafa, "cm.pop     {ra,s0-s11},96");
	test_zcmp_pop(0xbcfa, "cm.popretz {ra,s0-s11},96");
	test_zcmp_pop(0xbefa, "cm.popret  {ra,s0-s11},96");

	test_zcmp_pop(0xbafe, "cm.pop     {ra,s0-s11},112");
	test_zcmp_pop(0xbcfe, "cm.popretz {ra,s0-s11},112");
	test_zcmp_pop(0xbefe, "cm.popret  {ra,s0-s11},112");

	return 0;
}

// Test output extracted from spike:
/*EXPECTED-OUTPUT***************************************************************

Test: cm.pop     {ra},16
SP diff: 00000010
s0:  a5000008
s1:  a5000009
a0:  a500000a
a1:  0000007b
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra},16
SP diff: 00000010
s0:  a5000008
s1:  a5000009
a0:  00000000
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra},16
SP diff: 00000010
s0:  a5000008
s1:  a5000009
a0:  a500000a
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra},32
SP diff: 00000020
s0:  a5000008
s1:  a5000009
a0:  a500000a
a1:  0000007b
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra},32
SP diff: 00000020
s0:  a5000008
s1:  a5000009
a0:  00000000
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra},32
SP diff: 00000020
s0:  a5000008
s1:  a5000009
a0:  a500000a
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra},48
SP diff: 00000030
s0:  a5000008
s1:  a5000009
a0:  a500000a
a1:  0000007b
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra},48
SP diff: 00000030
s0:  a5000008
s1:  a5000009
a0:  00000000
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra},48
SP diff: 00000030
s0:  a5000008
s1:  a5000009
a0:  a500000a
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra},64
SP diff: 00000040
s0:  a5000008
s1:  a5000009
a0:  a500000a
a1:  0000007b
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra},64
SP diff: 00000040
s0:  a5000008
s1:  a5000009
a0:  00000000
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra},64
SP diff: 00000040
s0:  a5000008
s1:  a5000009
a0:  a500000a
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0},16
SP diff: 00000010
s0:  dead0003
s1:  a5000009
a0:  a500000a
a1:  0000007b
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0},16
SP diff: 00000010
s0:  dead0003
s1:  a5000009
a0:  00000000
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0},16
SP diff: 00000010
s0:  dead0003
s1:  a5000009
a0:  a500000a
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0},32
SP diff: 00000020
s0:  dead0007
s1:  a5000009
a0:  a500000a
a1:  0000007b
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0},32
SP diff: 00000020
s0:  dead0007
s1:  a5000009
a0:  00000000
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0},32
SP diff: 00000020
s0:  dead0007
s1:  a5000009
a0:  a500000a
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0},48
SP diff: 00000030
s0:  dead000b
s1:  a5000009
a0:  a500000a
a1:  0000007b
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0},48
SP diff: 00000030
s0:  dead000b
s1:  a5000009
a0:  00000000
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0},48
SP diff: 00000030
s0:  dead000b
s1:  a5000009
a0:  a500000a
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0},64
SP diff: 00000040
s0:  dead000f
s1:  a5000009
a0:  a500000a
a1:  0000007b
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0},64
SP diff: 00000040
s0:  dead000f
s1:  a5000009
a0:  00000000
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0},64
SP diff: 00000040
s0:  dead000f
s1:  a5000009
a0:  a500000a
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s1},16
SP diff: 00000010
s0:  dead0002
s1:  dead0003
a0:  a500000a
a1:  0000007b
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s1},16
SP diff: 00000010
s0:  dead0002
s1:  dead0003
a0:  00000000
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s1},16
SP diff: 00000010
s0:  dead0002
s1:  dead0003
a0:  a500000a
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s1},32
SP diff: 00000020
s0:  dead0006
s1:  dead0007
a0:  a500000a
a1:  0000007b
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s1},32
SP diff: 00000020
s0:  dead0006
s1:  dead0007
a0:  00000000
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s1},32
SP diff: 00000020
s0:  dead0006
s1:  dead0007
a0:  a500000a
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s1},48
SP diff: 00000030
s0:  dead000a
s1:  dead000b
a0:  a500000a
a1:  0000007b
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s1},48
SP diff: 00000030
s0:  dead000a
s1:  dead000b
a0:  00000000
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s1},48
SP diff: 00000030
s0:  dead000a
s1:  dead000b
a0:  a500000a
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s1},64
SP diff: 00000040
s0:  dead000e
s1:  dead000f
a0:  a500000a
a1:  0000007b
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s1},64
SP diff: 00000040
s0:  dead000e
s1:  dead000f
a0:  00000000
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s1},64
SP diff: 00000040
s0:  dead000e
s1:  dead000f
a0:  a500000a
a1:  000001c8
s2:  a5000012
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s2},16
SP diff: 00000010
s0:  dead0001
s1:  dead0002
a0:  a500000a
a1:  0000007b
s2:  dead0003
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s2},16
SP diff: 00000010
s0:  dead0001
s1:  dead0002
a0:  00000000
a1:  000001c8
s2:  dead0003
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s2},16
SP diff: 00000010
s0:  dead0001
s1:  dead0002
a0:  a500000a
a1:  000001c8
s2:  dead0003
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s2},32
SP diff: 00000020
s0:  dead0005
s1:  dead0006
a0:  a500000a
a1:  0000007b
s2:  dead0007
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s2},32
SP diff: 00000020
s0:  dead0005
s1:  dead0006
a0:  00000000
a1:  000001c8
s2:  dead0007
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s2},32
SP diff: 00000020
s0:  dead0005
s1:  dead0006
a0:  a500000a
a1:  000001c8
s2:  dead0007
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s2},48
SP diff: 00000030
s0:  dead0009
s1:  dead000a
a0:  a500000a
a1:  0000007b
s2:  dead000b
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s2},48
SP diff: 00000030
s0:  dead0009
s1:  dead000a
a0:  00000000
a1:  000001c8
s2:  dead000b
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s2},48
SP diff: 00000030
s0:  dead0009
s1:  dead000a
a0:  a500000a
a1:  000001c8
s2:  dead000b
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s2},64
SP diff: 00000040
s0:  dead000d
s1:  dead000e
a0:  a500000a
a1:  0000007b
s2:  dead000f
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s2},64
SP diff: 00000040
s0:  dead000d
s1:  dead000e
a0:  00000000
a1:  000001c8
s2:  dead000f
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s2},64
SP diff: 00000040
s0:  dead000d
s1:  dead000e
a0:  a500000a
a1:  000001c8
s2:  dead000f
s3:  a5000013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s3},32
SP diff: 00000020
s0:  dead0004
s1:  dead0005
a0:  a500000a
a1:  0000007b
s2:  dead0006
s3:  dead0007
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s3},32
SP diff: 00000020
s0:  dead0004
s1:  dead0005
a0:  00000000
a1:  000001c8
s2:  dead0006
s3:  dead0007
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s3},32
SP diff: 00000020
s0:  dead0004
s1:  dead0005
a0:  a500000a
a1:  000001c8
s2:  dead0006
s3:  dead0007
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s3},48
SP diff: 00000030
s0:  dead0008
s1:  dead0009
a0:  a500000a
a1:  0000007b
s2:  dead000a
s3:  dead000b
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s3},48
SP diff: 00000030
s0:  dead0008
s1:  dead0009
a0:  00000000
a1:  000001c8
s2:  dead000a
s3:  dead000b
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s3},48
SP diff: 00000030
s0:  dead0008
s1:  dead0009
a0:  a500000a
a1:  000001c8
s2:  dead000a
s3:  dead000b
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s3},64
SP diff: 00000040
s0:  dead000c
s1:  dead000d
a0:  a500000a
a1:  0000007b
s2:  dead000e
s3:  dead000f
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s3},64
SP diff: 00000040
s0:  dead000c
s1:  dead000d
a0:  00000000
a1:  000001c8
s2:  dead000e
s3:  dead000f
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s3},64
SP diff: 00000040
s0:  dead000c
s1:  dead000d
a0:  a500000a
a1:  000001c8
s2:  dead000e
s3:  dead000f
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s3},80
SP diff: 00000050
s0:  dead0010
s1:  dead0011
a0:  a500000a
a1:  0000007b
s2:  dead0012
s3:  dead0013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s3},80
SP diff: 00000050
s0:  dead0010
s1:  dead0011
a0:  00000000
a1:  000001c8
s2:  dead0012
s3:  dead0013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s3},80
SP diff: 00000050
s0:  dead0010
s1:  dead0011
a0:  a500000a
a1:  000001c8
s2:  dead0012
s3:  dead0013
s4:  a5000014
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s4},32
SP diff: 00000020
s0:  dead0003
s1:  dead0004
a0:  a500000a
a1:  0000007b
s2:  dead0005
s3:  dead0006
s4:  dead0007
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s4},32
SP diff: 00000020
s0:  dead0003
s1:  dead0004
a0:  00000000
a1:  000001c8
s2:  dead0005
s3:  dead0006
s4:  dead0007
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s4},32
SP diff: 00000020
s0:  dead0003
s1:  dead0004
a0:  a500000a
a1:  000001c8
s2:  dead0005
s3:  dead0006
s4:  dead0007
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s4},48
SP diff: 00000030
s0:  dead0007
s1:  dead0008
a0:  a500000a
a1:  0000007b
s2:  dead0009
s3:  dead000a
s4:  dead000b
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s4},48
SP diff: 00000030
s0:  dead0007
s1:  dead0008
a0:  00000000
a1:  000001c8
s2:  dead0009
s3:  dead000a
s4:  dead000b
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s4},48
SP diff: 00000030
s0:  dead0007
s1:  dead0008
a0:  a500000a
a1:  000001c8
s2:  dead0009
s3:  dead000a
s4:  dead000b
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s4},64
SP diff: 00000040
s0:  dead000b
s1:  dead000c
a0:  a500000a
a1:  0000007b
s2:  dead000d
s3:  dead000e
s4:  dead000f
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s4},64
SP diff: 00000040
s0:  dead000b
s1:  dead000c
a0:  00000000
a1:  000001c8
s2:  dead000d
s3:  dead000e
s4:  dead000f
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s4},64
SP diff: 00000040
s0:  dead000b
s1:  dead000c
a0:  a500000a
a1:  000001c8
s2:  dead000d
s3:  dead000e
s4:  dead000f
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s4},80
SP diff: 00000050
s0:  dead000f
s1:  dead0010
a0:  a500000a
a1:  0000007b
s2:  dead0011
s3:  dead0012
s4:  dead0013
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s4},80
SP diff: 00000050
s0:  dead000f
s1:  dead0010
a0:  00000000
a1:  000001c8
s2:  dead0011
s3:  dead0012
s4:  dead0013
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s4},80
SP diff: 00000050
s0:  dead000f
s1:  dead0010
a0:  a500000a
a1:  000001c8
s2:  dead0011
s3:  dead0012
s4:  dead0013
s5:  a5000015
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s5},32
SP diff: 00000020
s0:  dead0002
s1:  dead0003
a0:  a500000a
a1:  0000007b
s2:  dead0004
s3:  dead0005
s4:  dead0006
s5:  dead0007
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s5},32
SP diff: 00000020
s0:  dead0002
s1:  dead0003
a0:  00000000
a1:  000001c8
s2:  dead0004
s3:  dead0005
s4:  dead0006
s5:  dead0007
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s5},32
SP diff: 00000020
s0:  dead0002
s1:  dead0003
a0:  a500000a
a1:  000001c8
s2:  dead0004
s3:  dead0005
s4:  dead0006
s5:  dead0007
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s5},48
SP diff: 00000030
s0:  dead0006
s1:  dead0007
a0:  a500000a
a1:  0000007b
s2:  dead0008
s3:  dead0009
s4:  dead000a
s5:  dead000b
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s5},48
SP diff: 00000030
s0:  dead0006
s1:  dead0007
a0:  00000000
a1:  000001c8
s2:  dead0008
s3:  dead0009
s4:  dead000a
s5:  dead000b
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s5},48
SP diff: 00000030
s0:  dead0006
s1:  dead0007
a0:  a500000a
a1:  000001c8
s2:  dead0008
s3:  dead0009
s4:  dead000a
s5:  dead000b
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s5},64
SP diff: 00000040
s0:  dead000a
s1:  dead000b
a0:  a500000a
a1:  0000007b
s2:  dead000c
s3:  dead000d
s4:  dead000e
s5:  dead000f
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s5},64
SP diff: 00000040
s0:  dead000a
s1:  dead000b
a0:  00000000
a1:  000001c8
s2:  dead000c
s3:  dead000d
s4:  dead000e
s5:  dead000f
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s5},64
SP diff: 00000040
s0:  dead000a
s1:  dead000b
a0:  a500000a
a1:  000001c8
s2:  dead000c
s3:  dead000d
s4:  dead000e
s5:  dead000f
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s5},80
SP diff: 00000050
s0:  dead000e
s1:  dead000f
a0:  a500000a
a1:  0000007b
s2:  dead0010
s3:  dead0011
s4:  dead0012
s5:  dead0013
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s5},80
SP diff: 00000050
s0:  dead000e
s1:  dead000f
a0:  00000000
a1:  000001c8
s2:  dead0010
s3:  dead0011
s4:  dead0012
s5:  dead0013
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s5},80
SP diff: 00000050
s0:  dead000e
s1:  dead000f
a0:  a500000a
a1:  000001c8
s2:  dead0010
s3:  dead0011
s4:  dead0012
s5:  dead0013
s6:  a5000016
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s6},32
SP diff: 00000020
s0:  dead0001
s1:  dead0002
a0:  a500000a
a1:  0000007b
s2:  dead0003
s3:  dead0004
s4:  dead0005
s5:  dead0006
s6:  dead0007
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s6},32
SP diff: 00000020
s0:  dead0001
s1:  dead0002
a0:  00000000
a1:  000001c8
s2:  dead0003
s3:  dead0004
s4:  dead0005
s5:  dead0006
s6:  dead0007
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s6},32
SP diff: 00000020
s0:  dead0001
s1:  dead0002
a0:  a500000a
a1:  000001c8
s2:  dead0003
s3:  dead0004
s4:  dead0005
s5:  dead0006
s6:  dead0007
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s6},48
SP diff: 00000030
s0:  dead0005
s1:  dead0006
a0:  a500000a
a1:  0000007b
s2:  dead0007
s3:  dead0008
s4:  dead0009
s5:  dead000a
s6:  dead000b
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s6},48
SP diff: 00000030
s0:  dead0005
s1:  dead0006
a0:  00000000
a1:  000001c8
s2:  dead0007
s3:  dead0008
s4:  dead0009
s5:  dead000a
s6:  dead000b
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s6},48
SP diff: 00000030
s0:  dead0005
s1:  dead0006
a0:  a500000a
a1:  000001c8
s2:  dead0007
s3:  dead0008
s4:  dead0009
s5:  dead000a
s6:  dead000b
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s6},64
SP diff: 00000040
s0:  dead0009
s1:  dead000a
a0:  a500000a
a1:  0000007b
s2:  dead000b
s3:  dead000c
s4:  dead000d
s5:  dead000e
s6:  dead000f
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s6},64
SP diff: 00000040
s0:  dead0009
s1:  dead000a
a0:  00000000
a1:  000001c8
s2:  dead000b
s3:  dead000c
s4:  dead000d
s5:  dead000e
s6:  dead000f
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s6},64
SP diff: 00000040
s0:  dead0009
s1:  dead000a
a0:  a500000a
a1:  000001c8
s2:  dead000b
s3:  dead000c
s4:  dead000d
s5:  dead000e
s6:  dead000f
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s6},80
SP diff: 00000050
s0:  dead000d
s1:  dead000e
a0:  a500000a
a1:  0000007b
s2:  dead000f
s3:  dead0010
s4:  dead0011
s5:  dead0012
s6:  dead0013
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s6},80
SP diff: 00000050
s0:  dead000d
s1:  dead000e
a0:  00000000
a1:  000001c8
s2:  dead000f
s3:  dead0010
s4:  dead0011
s5:  dead0012
s6:  dead0013
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s6},80
SP diff: 00000050
s0:  dead000d
s1:  dead000e
a0:  a500000a
a1:  000001c8
s2:  dead000f
s3:  dead0010
s4:  dead0011
s5:  dead0012
s6:  dead0013
s7:  a5000017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s7},48
SP diff: 00000030
s0:  dead0004
s1:  dead0005
a0:  a500000a
a1:  0000007b
s2:  dead0006
s3:  dead0007
s4:  dead0008
s5:  dead0009
s6:  dead000a
s7:  dead000b
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s7},48
SP diff: 00000030
s0:  dead0004
s1:  dead0005
a0:  00000000
a1:  000001c8
s2:  dead0006
s3:  dead0007
s4:  dead0008
s5:  dead0009
s6:  dead000a
s7:  dead000b
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s7},48
SP diff: 00000030
s0:  dead0004
s1:  dead0005
a0:  a500000a
a1:  000001c8
s2:  dead0006
s3:  dead0007
s4:  dead0008
s5:  dead0009
s6:  dead000a
s7:  dead000b
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s7},64
SP diff: 00000040
s0:  dead0008
s1:  dead0009
a0:  a500000a
a1:  0000007b
s2:  dead000a
s3:  dead000b
s4:  dead000c
s5:  dead000d
s6:  dead000e
s7:  dead000f
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s7},64
SP diff: 00000040
s0:  dead0008
s1:  dead0009
a0:  00000000
a1:  000001c8
s2:  dead000a
s3:  dead000b
s4:  dead000c
s5:  dead000d
s6:  dead000e
s7:  dead000f
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s7},64
SP diff: 00000040
s0:  dead0008
s1:  dead0009
a0:  a500000a
a1:  000001c8
s2:  dead000a
s3:  dead000b
s4:  dead000c
s5:  dead000d
s6:  dead000e
s7:  dead000f
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s7},80
SP diff: 00000050
s0:  dead000c
s1:  dead000d
a0:  a500000a
a1:  0000007b
s2:  dead000e
s3:  dead000f
s4:  dead0010
s5:  dead0011
s6:  dead0012
s7:  dead0013
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s7},80
SP diff: 00000050
s0:  dead000c
s1:  dead000d
a0:  00000000
a1:  000001c8
s2:  dead000e
s3:  dead000f
s4:  dead0010
s5:  dead0011
s6:  dead0012
s7:  dead0013
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s7},80
SP diff: 00000050
s0:  dead000c
s1:  dead000d
a0:  a500000a
a1:  000001c8
s2:  dead000e
s3:  dead000f
s4:  dead0010
s5:  dead0011
s6:  dead0012
s7:  dead0013
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s7},96
SP diff: 00000060
s0:  dead0010
s1:  dead0011
a0:  a500000a
a1:  0000007b
s2:  dead0012
s3:  dead0013
s4:  dead0014
s5:  dead0015
s6:  dead0016
s7:  dead0017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s7},96
SP diff: 00000060
s0:  dead0010
s1:  dead0011
a0:  00000000
a1:  000001c8
s2:  dead0012
s3:  dead0013
s4:  dead0014
s5:  dead0015
s6:  dead0016
s7:  dead0017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s7},96
SP diff: 00000060
s0:  dead0010
s1:  dead0011
a0:  a500000a
a1:  000001c8
s2:  dead0012
s3:  dead0013
s4:  dead0014
s5:  dead0015
s6:  dead0016
s7:  dead0017
s8:  a5000018
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s8},48
SP diff: 00000030
s0:  dead0003
s1:  dead0004
a0:  a500000a
a1:  0000007b
s2:  dead0005
s3:  dead0006
s4:  dead0007
s5:  dead0008
s6:  dead0009
s7:  dead000a
s8:  dead000b
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s8},48
SP diff: 00000030
s0:  dead0003
s1:  dead0004
a0:  00000000
a1:  000001c8
s2:  dead0005
s3:  dead0006
s4:  dead0007
s5:  dead0008
s6:  dead0009
s7:  dead000a
s8:  dead000b
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s8},48
SP diff: 00000030
s0:  dead0003
s1:  dead0004
a0:  a500000a
a1:  000001c8
s2:  dead0005
s3:  dead0006
s4:  dead0007
s5:  dead0008
s6:  dead0009
s7:  dead000a
s8:  dead000b
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s8},64
SP diff: 00000040
s0:  dead0007
s1:  dead0008
a0:  a500000a
a1:  0000007b
s2:  dead0009
s3:  dead000a
s4:  dead000b
s5:  dead000c
s6:  dead000d
s7:  dead000e
s8:  dead000f
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s8},64
SP diff: 00000040
s0:  dead0007
s1:  dead0008
a0:  00000000
a1:  000001c8
s2:  dead0009
s3:  dead000a
s4:  dead000b
s5:  dead000c
s6:  dead000d
s7:  dead000e
s8:  dead000f
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s8},64
SP diff: 00000040
s0:  dead0007
s1:  dead0008
a0:  a500000a
a1:  000001c8
s2:  dead0009
s3:  dead000a
s4:  dead000b
s5:  dead000c
s6:  dead000d
s7:  dead000e
s8:  dead000f
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s8},80
SP diff: 00000050
s0:  dead000b
s1:  dead000c
a0:  a500000a
a1:  0000007b
s2:  dead000d
s3:  dead000e
s4:  dead000f
s5:  dead0010
s6:  dead0011
s7:  dead0012
s8:  dead0013
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s8},80
SP diff: 00000050
s0:  dead000b
s1:  dead000c
a0:  00000000
a1:  000001c8
s2:  dead000d
s3:  dead000e
s4:  dead000f
s5:  dead0010
s6:  dead0011
s7:  dead0012
s8:  dead0013
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s8},80
SP diff: 00000050
s0:  dead000b
s1:  dead000c
a0:  a500000a
a1:  000001c8
s2:  dead000d
s3:  dead000e
s4:  dead000f
s5:  dead0010
s6:  dead0011
s7:  dead0012
s8:  dead0013
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s8},96
SP diff: 00000060
s0:  dead000f
s1:  dead0010
a0:  a500000a
a1:  0000007b
s2:  dead0011
s3:  dead0012
s4:  dead0013
s5:  dead0014
s6:  dead0015
s7:  dead0016
s8:  dead0017
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s8},96
SP diff: 00000060
s0:  dead000f
s1:  dead0010
a0:  00000000
a1:  000001c8
s2:  dead0011
s3:  dead0012
s4:  dead0013
s5:  dead0014
s6:  dead0015
s7:  dead0016
s8:  dead0017
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s8},96
SP diff: 00000060
s0:  dead000f
s1:  dead0010
a0:  a500000a
a1:  000001c8
s2:  dead0011
s3:  dead0012
s4:  dead0013
s5:  dead0014
s6:  dead0015
s7:  dead0016
s8:  dead0017
s9:  a5000019
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s9},48
SP diff: 00000030
s0:  dead0002
s1:  dead0003
a0:  a500000a
a1:  0000007b
s2:  dead0004
s3:  dead0005
s4:  dead0006
s5:  dead0007
s6:  dead0008
s7:  dead0009
s8:  dead000a
s9:  dead000b
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s9},48
SP diff: 00000030
s0:  dead0002
s1:  dead0003
a0:  00000000
a1:  000001c8
s2:  dead0004
s3:  dead0005
s4:  dead0006
s5:  dead0007
s6:  dead0008
s7:  dead0009
s8:  dead000a
s9:  dead000b
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s9},48
SP diff: 00000030
s0:  dead0002
s1:  dead0003
a0:  a500000a
a1:  000001c8
s2:  dead0004
s3:  dead0005
s4:  dead0006
s5:  dead0007
s6:  dead0008
s7:  dead0009
s8:  dead000a
s9:  dead000b
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s9},64
SP diff: 00000040
s0:  dead0006
s1:  dead0007
a0:  a500000a
a1:  0000007b
s2:  dead0008
s3:  dead0009
s4:  dead000a
s5:  dead000b
s6:  dead000c
s7:  dead000d
s8:  dead000e
s9:  dead000f
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s9},64
SP diff: 00000040
s0:  dead0006
s1:  dead0007
a0:  00000000
a1:  000001c8
s2:  dead0008
s3:  dead0009
s4:  dead000a
s5:  dead000b
s6:  dead000c
s7:  dead000d
s8:  dead000e
s9:  dead000f
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s9},64
SP diff: 00000040
s0:  dead0006
s1:  dead0007
a0:  a500000a
a1:  000001c8
s2:  dead0008
s3:  dead0009
s4:  dead000a
s5:  dead000b
s6:  dead000c
s7:  dead000d
s8:  dead000e
s9:  dead000f
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s9},80
SP diff: 00000050
s0:  dead000a
s1:  dead000b
a0:  a500000a
a1:  0000007b
s2:  dead000c
s3:  dead000d
s4:  dead000e
s5:  dead000f
s6:  dead0010
s7:  dead0011
s8:  dead0012
s9:  dead0013
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s9},80
SP diff: 00000050
s0:  dead000a
s1:  dead000b
a0:  00000000
a1:  000001c8
s2:  dead000c
s3:  dead000d
s4:  dead000e
s5:  dead000f
s6:  dead0010
s7:  dead0011
s8:  dead0012
s9:  dead0013
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s9},80
SP diff: 00000050
s0:  dead000a
s1:  dead000b
a0:  a500000a
a1:  000001c8
s2:  dead000c
s3:  dead000d
s4:  dead000e
s5:  dead000f
s6:  dead0010
s7:  dead0011
s8:  dead0012
s9:  dead0013
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s9},96
SP diff: 00000060
s0:  dead000e
s1:  dead000f
a0:  a500000a
a1:  0000007b
s2:  dead0010
s3:  dead0011
s4:  dead0012
s5:  dead0013
s6:  dead0014
s7:  dead0015
s8:  dead0016
s9:  dead0017
s10: a500001a
s11: a500001b

Test: cm.popretz {ra,s0-s9},96
SP diff: 00000060
s0:  dead000e
s1:  dead000f
a0:  00000000
a1:  000001c8
s2:  dead0010
s3:  dead0011
s4:  dead0012
s5:  dead0013
s6:  dead0014
s7:  dead0015
s8:  dead0016
s9:  dead0017
s10: a500001a
s11: a500001b

Test: cm.popret  {ra,s0-s9},96
SP diff: 00000060
s0:  dead000e
s1:  dead000f
a0:  a500000a
a1:  000001c8
s2:  dead0010
s3:  dead0011
s4:  dead0012
s5:  dead0013
s6:  dead0014
s7:  dead0015
s8:  dead0016
s9:  dead0017
s10: a500001a
s11: a500001b

Test: cm.pop     {ra,s0-s11},64
SP diff: 00000040
s0:  dead0004
s1:  dead0005
a0:  a500000a
a1:  0000007b
s2:  dead0006
s3:  dead0007
s4:  dead0008
s5:  dead0009
s6:  dead000a
s7:  dead000b
s8:  dead000c
s9:  dead000d
s10: dead000e
s11: dead000f

Test: cm.popretz {ra,s0-s11},64
SP diff: 00000040
s0:  dead0004
s1:  dead0005
a0:  00000000
a1:  000001c8
s2:  dead0006
s3:  dead0007
s4:  dead0008
s5:  dead0009
s6:  dead000a
s7:  dead000b
s8:  dead000c
s9:  dead000d
s10: dead000e
s11: dead000f

Test: cm.popret  {ra,s0-s11},64
SP diff: 00000040
s0:  dead0004
s1:  dead0005
a0:  a500000a
a1:  000001c8
s2:  dead0006
s3:  dead0007
s4:  dead0008
s5:  dead0009
s6:  dead000a
s7:  dead000b
s8:  dead000c
s9:  dead000d
s10: dead000e
s11: dead000f

Test: cm.pop     {ra,s0-s11},80
SP diff: 00000050
s0:  dead0008
s1:  dead0009
a0:  a500000a
a1:  0000007b
s2:  dead000a
s3:  dead000b
s4:  dead000c
s5:  dead000d
s6:  dead000e
s7:  dead000f
s8:  dead0010
s9:  dead0011
s10: dead0012
s11: dead0013

Test: cm.popretz {ra,s0-s11},80
SP diff: 00000050
s0:  dead0008
s1:  dead0009
a0:  00000000
a1:  000001c8
s2:  dead000a
s3:  dead000b
s4:  dead000c
s5:  dead000d
s6:  dead000e
s7:  dead000f
s8:  dead0010
s9:  dead0011
s10: dead0012
s11: dead0013

Test: cm.popret  {ra,s0-s11},80
SP diff: 00000050
s0:  dead0008
s1:  dead0009
a0:  a500000a
a1:  000001c8
s2:  dead000a
s3:  dead000b
s4:  dead000c
s5:  dead000d
s6:  dead000e
s7:  dead000f
s8:  dead0010
s9:  dead0011
s10: dead0012
s11: dead0013

Test: cm.pop     {ra,s0-s11},96
SP diff: 00000060
s0:  dead000c
s1:  dead000d
a0:  a500000a
a1:  0000007b
s2:  dead000e
s3:  dead000f
s4:  dead0010
s5:  dead0011
s6:  dead0012
s7:  dead0013
s8:  dead0014
s9:  dead0015
s10: dead0016
s11: dead0017

Test: cm.popretz {ra,s0-s11},96
SP diff: 00000060
s0:  dead000c
s1:  dead000d
a0:  00000000
a1:  000001c8
s2:  dead000e
s3:  dead000f
s4:  dead0010
s5:  dead0011
s6:  dead0012
s7:  dead0013
s8:  dead0014
s9:  dead0015
s10: dead0016
s11: dead0017

Test: cm.popret  {ra,s0-s11},96
SP diff: 00000060
s0:  dead000c
s1:  dead000d
a0:  a500000a
a1:  000001c8
s2:  dead000e
s3:  dead000f
s4:  dead0010
s5:  dead0011
s6:  dead0012
s7:  dead0013
s8:  dead0014
s9:  dead0015
s10: dead0016
s11: dead0017

Test: cm.pop     {ra,s0-s11},112
SP diff: 00000070
s0:  dead0010
s1:  dead0011
a0:  a500000a
a1:  0000007b
s2:  dead0012
s3:  dead0013
s4:  dead0014
s5:  dead0015
s6:  dead0016
s7:  dead0017
s8:  dead0018
s9:  dead0019
s10: dead001a
s11: dead001b

Test: cm.popretz {ra,s0-s11},112
SP diff: 00000070
s0:  dead0010
s1:  dead0011
a0:  00000000
a1:  000001c8
s2:  dead0012
s3:  dead0013
s4:  dead0014
s5:  dead0015
s6:  dead0016
s7:  dead0017
s8:  dead0018
s9:  dead0019
s10: dead001a
s11: dead001b

Test: cm.popret  {ra,s0-s11},112
SP diff: 00000070
s0:  dead0010
s1:  dead0011
a0:  a500000a
a1:  000001c8
s2:  dead0012
s3:  dead0013
s4:  dead0014
s5:  dead0015
s6:  dead0016
s7:  dead0017
s8:  dead0018
s9:  dead0019
s10: dead001a
s11: dead001b

*******************************************************************************/
