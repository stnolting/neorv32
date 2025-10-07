#include "zcmp.h"
// To get the full list of opcodes I assembled then disassembled this file:
//
// .set i, 0
// .rept 64
// .hword 0xac62 | ((i & 0x38) << 4) | ((i & 0x7) << 2)
// .set i, i + 1
// .endr

#define test_mva01s(instr_bits, instr_name) \
	neorv32_uart0_printf("Test: " instr_name "\n"); \
	asm volatile ( \
		/* Save all clobbered registers onto the stack */ \
		"addi sp, sp, -48\n" \
		"sw a0,  0(sp)\n" \
		"sw a1,  4(sp)\n" \
		"sw s0,  8(sp)\n" \
		"sw s1, 12(sp)\n" \
		"sw s2, 16(sp)\n" \
		"sw s3, 20(sp)\n" \
		"sw s4, 24(sp)\n" \
		"sw s5, 28(sp)\n" \
		"sw s6, 32(sp)\n" \
		"sw s7, 36(sp)\n" \
		"sw ra, 40(sp)\n" \
		/* Give unique values to all registers that can be mv'd */ \
		"li x8,  0xa5000000 + 8\n" \
		"li x9,  0xa5000000 + 9\n" \
		"li x10, 0xa5000000 + 10\n" \
		"li x11, 0xa5000000 + 11\n" \
		"li x18, 0xa5000000 + 18\n" \
		"li x19, 0xa5000000 + 19\n" \
		"li x20, 0xa5000000 + 20\n" \
		"li x21, 0xa5000000 + 21\n" \
		"li x22, 0xa5000000 + 22\n" \
		"li x23, 0xa5000000 + 23\n" \
		/* Let's go */ \
		".hword " #instr_bits "\n" \
		/* Write out results */ \
		"la ra, test_results\n" \
		"sw a0,  0(ra)\n" \
		"sw a1,  4(ra)\n" \
		"sw s0,  8(ra)\n" \
		"sw s1, 12(ra)\n" \
		"sw s2, 16(ra)\n" \
		"sw s3, 20(ra)\n" \
		"sw s4, 24(ra)\n" \
		"sw s5, 28(ra)\n" \
		"sw s6, 32(ra)\n" \
		"sw s7, 36(ra)\n" \
		/* Restore original sp, and clobbered registers */ \
		"lw a0,  0(sp)\n" \
		"lw a1,  4(sp)\n" \
		"lw s0,  8(sp)\n" \
		"lw s1, 12(sp)\n" \
		"lw s2, 16(sp)\n" \
		"lw s3, 20(sp)\n" \
		"lw s4, 24(sp)\n" \
		"lw s5, 28(sp)\n" \
		"lw s6, 32(sp)\n" \
		"lw s7, 36(sp)\n" \
		"lw ra, 40(sp)\n" \
		"addi sp, sp, 48\n" \
	); \
	neorv32_uart0_printf("a1: %x\n", test_results[1]); \
	neorv32_uart0_printf("s0: %x\n", test_results[2]); \
	neorv32_uart0_printf("s1: %x\n", test_results[3]); \
	neorv32_uart0_printf("s2: %x\n", test_results[4]); \
	neorv32_uart0_printf("s3: %x\n", test_results[5]); \
	neorv32_uart0_printf("s4: %x\n", test_results[6]); \
	neorv32_uart0_printf("s5: %x\n", test_results[7]); \
	neorv32_uart0_printf("s6: %x\n", test_results[8]); \
	neorv32_uart0_printf("s7: %x\n", test_results[9]); \
	neorv32_uart0_printf("\n");

int cm_mva01s() {
	test_mva01s(0xac62, "cm.mva01s s0,s0");
	test_mva01s(0xac66, "cm.mva01s s0,s1");
	test_mva01s(0xac6a, "cm.mva01s s0,s2");
	test_mva01s(0xac6e, "cm.mva01s s0,s3");
	test_mva01s(0xac72, "cm.mva01s s0,s4");
	test_mva01s(0xac76, "cm.mva01s s0,s5");
	test_mva01s(0xac7a, "cm.mva01s s0,s6");
	test_mva01s(0xac7e, "cm.mva01s s0,s7");
	test_mva01s(0xace2, "cm.mva01s s1,s0");
	test_mva01s(0xace6, "cm.mva01s s1,s1");
	test_mva01s(0xacea, "cm.mva01s s1,s2");
	test_mva01s(0xacee, "cm.mva01s s1,s3");
	test_mva01s(0xacf2, "cm.mva01s s1,s4");
	test_mva01s(0xacf6, "cm.mva01s s1,s5");
	test_mva01s(0xacfa, "cm.mva01s s1,s6");
	test_mva01s(0xacfe, "cm.mva01s s1,s7");
	test_mva01s(0xad62, "cm.mva01s s2,s0");
	test_mva01s(0xad66, "cm.mva01s s2,s1");
	test_mva01s(0xad6a, "cm.mva01s s2,s2");
	test_mva01s(0xad6e, "cm.mva01s s2,s3");
	test_mva01s(0xad72, "cm.mva01s s2,s4");
	test_mva01s(0xad76, "cm.mva01s s2,s5");
	test_mva01s(0xad7a, "cm.mva01s s2,s6");
	test_mva01s(0xad7e, "cm.mva01s s2,s7");
	test_mva01s(0xade2, "cm.mva01s s3,s0");
	test_mva01s(0xade6, "cm.mva01s s3,s1");
	test_mva01s(0xadea, "cm.mva01s s3,s2");
	test_mva01s(0xadee, "cm.mva01s s3,s3");
	test_mva01s(0xadf2, "cm.mva01s s3,s4");
	test_mva01s(0xadf6, "cm.mva01s s3,s5");
	test_mva01s(0xadfa, "cm.mva01s s3,s6");
	test_mva01s(0xadfe, "cm.mva01s s3,s7");
	test_mva01s(0xae62, "cm.mva01s s4,s0");
	test_mva01s(0xae66, "cm.mva01s s4,s1");
	test_mva01s(0xae6a, "cm.mva01s s4,s2");
	test_mva01s(0xae6e, "cm.mva01s s4,s3");
	test_mva01s(0xae72, "cm.mva01s s4,s4");
	test_mva01s(0xae76, "cm.mva01s s4,s5");
	test_mva01s(0xae7a, "cm.mva01s s4,s6");
	test_mva01s(0xae7e, "cm.mva01s s4,s7");
	test_mva01s(0xaee2, "cm.mva01s s5,s0");
	test_mva01s(0xaee6, "cm.mva01s s5,s1");
	test_mva01s(0xaeea, "cm.mva01s s5,s2");
	test_mva01s(0xaeee, "cm.mva01s s5,s3");
	test_mva01s(0xaef2, "cm.mva01s s5,s4");
	test_mva01s(0xaef6, "cm.mva01s s5,s5");
	test_mva01s(0xaefa, "cm.mva01s s5,s6");
	test_mva01s(0xaefe, "cm.mva01s s5,s7");
	test_mva01s(0xaf62, "cm.mva01s s6,s0");
	test_mva01s(0xaf66, "cm.mva01s s6,s1");
	test_mva01s(0xaf6a, "cm.mva01s s6,s2");
	test_mva01s(0xaf6e, "cm.mva01s s6,s3");
	test_mva01s(0xaf72, "cm.mva01s s6,s4");
	test_mva01s(0xaf76, "cm.mva01s s6,s5");
	test_mva01s(0xaf7a, "cm.mva01s s6,s6");
	test_mva01s(0xaf7e, "cm.mva01s s6,s7");
	test_mva01s(0xafe2, "cm.mva01s s7,s0");
	test_mva01s(0xafe6, "cm.mva01s s7,s1");
	test_mva01s(0xafea, "cm.mva01s s7,s2");
	test_mva01s(0xafee, "cm.mva01s s7,s3");
	test_mva01s(0xaff2, "cm.mva01s s7,s4");
	test_mva01s(0xaff6, "cm.mva01s s7,s5");
	test_mva01s(0xaffa, "cm.mva01s s7,s6");
	test_mva01s(0xaffe, "cm.mva01s s7,s7");
	return 0;
}


// Test output extracted from spike:
/*EXPECTED-OUTPUT***************************************************************
Test: cm.mva01s s0,s0
a0: a5000008
a1: a5000008
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s0,s1
a0: a5000008
a1: a5000009
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s0,s2
a0: a5000008
a1: a5000012
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s0,s3
a0: a5000008
a1: a5000013
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s0,s4
a0: a5000008
a1: a5000014
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s0,s5
a0: a5000008
a1: a5000015
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s0,s6
a0: a5000008
a1: a5000016
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s0,s7
a0: a5000008
a1: a5000017
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s1,s0
a0: a5000009
a1: a5000008
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s1,s1
a0: a5000009
a1: a5000009
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s1,s2
a0: a5000009
a1: a5000012
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s1,s3
a0: a5000009
a1: a5000013
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s1,s4
a0: a5000009
a1: a5000014
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s1,s5
a0: a5000009
a1: a5000015
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s1,s6
a0: a5000009
a1: a5000016
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s1,s7
a0: a5000009
a1: a5000017
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s2,s0
a0: a5000012
a1: a5000008
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s2,s1
a0: a5000012
a1: a5000009
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s2,s2
a0: a5000012
a1: a5000012
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s2,s3
a0: a5000012
a1: a5000013
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s2,s4
a0: a5000012
a1: a5000014
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s2,s5
a0: a5000012
a1: a5000015
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s2,s6
a0: a5000012
a1: a5000016
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s2,s7
a0: a5000012
a1: a5000017
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s3,s0
a0: a5000013
a1: a5000008
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s3,s1
a0: a5000013
a1: a5000009
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s3,s2
a0: a5000013
a1: a5000012
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s3,s3
a0: a5000013
a1: a5000013
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s3,s4
a0: a5000013
a1: a5000014
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s3,s5
a0: a5000013
a1: a5000015
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s3,s6
a0: a5000013
a1: a5000016
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s3,s7
a0: a5000013
a1: a5000017
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s4,s0
a0: a5000014
a1: a5000008
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s4,s1
a0: a5000014
a1: a5000009
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s4,s2
a0: a5000014
a1: a5000012
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s4,s3
a0: a5000014
a1: a5000013
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s4,s4
a0: a5000014
a1: a5000014
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s4,s5
a0: a5000014
a1: a5000015
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s4,s6
a0: a5000014
a1: a5000016
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s4,s7
a0: a5000014
a1: a5000017
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s5,s0
a0: a5000015
a1: a5000008
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s5,s1
a0: a5000015
a1: a5000009
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s5,s2
a0: a5000015
a1: a5000012
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s5,s3
a0: a5000015
a1: a5000013
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s5,s4
a0: a5000015
a1: a5000014
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s5,s5
a0: a5000015
a1: a5000015
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s5,s6
a0: a5000015
a1: a5000016
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s5,s7
a0: a5000015
a1: a5000017
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s6,s0
a0: a5000016
a1: a5000008
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s6,s1
a0: a5000016
a1: a5000009
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s6,s2
a0: a5000016
a1: a5000012
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s6,s3
a0: a5000016
a1: a5000013
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s6,s4
a0: a5000016
a1: a5000014
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s6,s5
a0: a5000016
a1: a5000015
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s6,s6
a0: a5000016
a1: a5000016
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s6,s7
a0: a5000016
a1: a5000017
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s7,s0
a0: a5000017
a1: a5000008
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s7,s1
a0: a5000017
a1: a5000009
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s7,s2
a0: a5000017
a1: a5000012
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s7,s3
a0: a5000017
a1: a5000013
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s7,s4
a0: a5000017
a1: a5000014
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s7,s5
a0: a5000017
a1: a5000015
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s7,s6
a0: a5000017
a1: a5000016
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mva01s s7,s7
a0: a5000017
a1: a5000017
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

*******************************************************************************/
