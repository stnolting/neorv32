#include "zcmp.h"

uint32_t mva01ss0s0[10] = {0xa5000008, 0xa5000008, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss0s1[10] = {0xa5000008, 0xa5000009, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss0s2[10] = {0xa5000008, 0xa5000012, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss0s3[10] = {0xa5000008, 0xa5000013, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss0s4[10] = {0xa5000008, 0xa5000014, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss0s5[10] = {0xa5000008, 0xa5000015, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss0s6[10] = {0xa5000008, 0xa5000016, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss0s7[10] = {0xa5000008, 0xa5000017, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss1s0[10] = {0xa5000009, 0xa5000008, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss1s1[10] = {0xa5000009, 0xa5000009, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss1s2[10] = {0xa5000009, 0xa5000012, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss1s3[10] = {0xa5000009, 0xa5000013, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss1s4[10] = {0xa5000009, 0xa5000014, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss1s5[10] = {0xa5000009, 0xa5000015, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss1s6[10] = {0xa5000009, 0xa5000016, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss1s7[10] = {0xa5000009, 0xa5000017, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss2s0[10] = {0xa5000012, 0xa5000008, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss2s1[10] = {0xa5000012, 0xa5000009, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss2s2[10] = {0xa5000012, 0xa5000012, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss2s3[10] = {0xa5000012, 0xa5000013, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss2s4[10] = {0xa5000012, 0xa5000014, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss2s5[10] = {0xa5000012, 0xa5000015, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss2s6[10] = {0xa5000012, 0xa5000016, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss2s7[10] = {0xa5000012, 0xa5000017, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss3s0[10] = {0xa5000013, 0xa5000008, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss3s1[10] = {0xa5000013, 0xa5000009, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss3s2[10] = {0xa5000013, 0xa5000012, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss3s3[10] = {0xa5000013, 0xa5000013, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss3s4[10] = {0xa5000013, 0xa5000014, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss3s5[10] = {0xa5000013, 0xa5000015, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss3s6[10] = {0xa5000013, 0xa5000016, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss3s7[10] = {0xa5000013, 0xa5000017, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss4s0[10] = {0xa5000014, 0xa5000008, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss4s1[10] = {0xa5000014, 0xa5000009, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss4s2[10] = {0xa5000014, 0xa5000012, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss4s3[10] = {0xa5000014, 0xa5000013, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss4s4[10] = {0xa5000014, 0xa5000014, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss4s5[10] = {0xa5000014, 0xa5000015, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss4s6[10] = {0xa5000014, 0xa5000016, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss4s7[10] = {0xa5000014, 0xa5000017, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss5s0[10] = {0xa5000015, 0xa5000008, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss5s1[10] = {0xa5000015, 0xa5000009, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss5s2[10] = {0xa5000015, 0xa5000012, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss5s3[10] = {0xa5000015, 0xa5000013, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss5s4[10] = {0xa5000015, 0xa5000014, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss5s5[10] = {0xa5000015, 0xa5000015, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss5s6[10] = {0xa5000015, 0xa5000016, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss5s7[10] = {0xa5000015, 0xa5000017, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss6s0[10] = {0xa5000016, 0xa5000008, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss6s1[10] = {0xa5000016, 0xa5000009, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss6s2[10] = {0xa5000016, 0xa5000012, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss6s3[10] = {0xa5000016, 0xa5000013, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss6s4[10] = {0xa5000016, 0xa5000014, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss6s5[10] = {0xa5000016, 0xa5000015, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss6s6[10] = {0xa5000016, 0xa5000016, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss6s7[10] = {0xa5000016, 0xa5000017, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss7s0[10] = {0xa5000017, 0xa5000008, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss7s1[10] = {0xa5000017, 0xa5000009, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss7s2[10] = {0xa5000017, 0xa5000012, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss7s3[10] = {0xa5000017, 0xa5000013, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss7s4[10] = {0xa5000017, 0xa5000014, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss7s5[10] = {0xa5000017, 0xa5000015, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss7s6[10] = {0xa5000017, 0xa5000016, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t mva01ss7s7[10] = {0xa5000017, 0xa5000017, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};

uint8_t local_result_mva01s = 0;
uint8_t global_result_mva01s = 0;

#define test_mva01s(instr_bits, instr_name, arr)                                              \
	neorv32_uart0_printf(instr_name);                                                         \
	asm volatile(/* Save all clobbered registers onto the stack */                            \
				 "addi sp, sp, -48\n"                                                         \
				 "sw a0,  0(sp)\n"                                                            \
				 "sw a1,  4(sp)\n"                                                            \
				 "sw s0,  8(sp)\n"                                                            \
				 "sw s1, 12(sp)\n"                                                            \
				 "sw s2, 16(sp)\n"                                                            \
				 "sw s3, 20(sp)\n"                                                            \
				 "sw s4, 24(sp)\n"                                                            \
				 "sw s5, 28(sp)\n"                                                            \
				 "sw s6, 32(sp)\n"                                                            \
				 "sw s7, 36(sp)\n"                                                            \
				 "sw ra, 40(sp)\n" /* Give unique values to all registers that can be mv'd */ \
				 "li x8,  0xa5000000 + 8\n"                                                   \
				 "li x9,  0xa5000000 + 9\n"                                                   \
				 "li x10, 0xa5000000 + 10\n"                                                  \
				 "li x11, 0xa5000000 + 11\n"                                                  \
				 "li x18, 0xa5000000 + 18\n"                                                  \
				 "li x19, 0xa5000000 + 19\n"                                                  \
				 "li x20, 0xa5000000 + 20\n"                                                  \
				 "li x21, 0xa5000000 + 21\n"                                                  \
				 "li x22, 0xa5000000 + 22\n"                                                  \
				 "li x23, 0xa5000000 + 23\n" /* Let's go */                                   \
				 ".hword " #instr_bits "\n"	 /* Write out results */                          \
				 "la ra, test_results\n"                                                      \
				 "sw a0,  0(ra)\n"                                                            \
				 "sw a1,  4(ra)\n"                                                            \
				 "sw s0,  8(ra)\n"                                                            \
				 "sw s1, 12(ra)\n"                                                            \
				 "sw s2, 16(ra)\n"                                                            \
				 "sw s3, 20(ra)\n"                                                            \
				 "sw s4, 24(ra)\n"                                                            \
				 "sw s5, 28(ra)\n"                                                            \
				 "sw s6, 32(ra)\n"                                                            \
				 "sw s7, 36(ra)\n" /* Restore original sp, and clobbered registers */         \
				 "lw a0,  0(sp)\n"                                                            \
				 "lw a1,  4(sp)\n"                                                            \
				 "lw s0,  8(sp)\n"                                                            \
				 "lw s1, 12(sp)\n"                                                            \
				 "lw s2, 16(sp)\n"                                                            \
				 "lw s3, 20(sp)\n"                                                            \
				 "lw s4, 24(sp)\n"                                                            \
				 "lw s5, 28(sp)\n"                                                            \
				 "lw s6, 32(sp)\n"                                                            \
				 "lw s7, 36(sp)\n"                                                            \
				 "lw ra, 40(sp)\n"                                                            \
				 "addi sp, sp, 48\n");                                                        \
	for (int i = 0; i < 10; i++)                                                              \
	{                                                                                         \
		if (test_results[i] != arr[i])                                                        \
		{                                                                                     \
			local_result_mva01s = 1;                                                          \
			global_result_mva01s = 1;                                                         \
		}                                                                                     \
	}                                                                                         \
	if (local_result_mva01s == 0)                                                             \
	{                                                                                         \
		neorv32_uart0_printf(" - OK\n");                                                      \
	}                                                                                         \
	else                                                                                      \
	{                                                                                         \
		neorv32_uart0_printf(" - FAIL\n");                                                    \
	}
int cm_mva01s()
{
	test_mva01s(0xac62, "cm.mva01s s0,s0", mva01ss0s0);
	test_mva01s(0xac66, "cm.mva01s s0,s1", mva01ss0s1);
	test_mva01s(0xac6a, "cm.mva01s s0,s2", mva01ss0s2);
	test_mva01s(0xac6e, "cm.mva01s s0,s3", mva01ss0s3);
	test_mva01s(0xac72, "cm.mva01s s0,s4", mva01ss0s4);
	test_mva01s(0xac76, "cm.mva01s s0,s5", mva01ss0s5);
	test_mva01s(0xac7a, "cm.mva01s s0,s6", mva01ss0s6);
	test_mva01s(0xac7e, "cm.mva01s s0,s7", mva01ss0s7);
	test_mva01s(0xace2, "cm.mva01s s1,s0", mva01ss1s0);
	test_mva01s(0xace6, "cm.mva01s s1,s1", mva01ss1s1);
	test_mva01s(0xacea, "cm.mva01s s1,s2", mva01ss1s2);
	test_mva01s(0xacee, "cm.mva01s s1,s3", mva01ss1s3);
	test_mva01s(0xacf2, "cm.mva01s s1,s4", mva01ss1s4);
	test_mva01s(0xacf6, "cm.mva01s s1,s5", mva01ss1s5);
	test_mva01s(0xacfa, "cm.mva01s s1,s6", mva01ss1s6);
	test_mva01s(0xacfe, "cm.mva01s s1,s7", mva01ss1s7);
	test_mva01s(0xad62, "cm.mva01s s2,s0", mva01ss2s0);
	test_mva01s(0xad66, "cm.mva01s s2,s1", mva01ss2s1);
	test_mva01s(0xad6a, "cm.mva01s s2,s2", mva01ss2s2);
	test_mva01s(0xad6e, "cm.mva01s s2,s3", mva01ss2s3);
	test_mva01s(0xad72, "cm.mva01s s2,s4", mva01ss2s4);
	test_mva01s(0xad76, "cm.mva01s s2,s5", mva01ss2s5);
	test_mva01s(0xad7a, "cm.mva01s s2,s6", mva01ss2s6);
	test_mva01s(0xad7e, "cm.mva01s s2,s7", mva01ss2s7);
	test_mva01s(0xade2, "cm.mva01s s3,s0", mva01ss3s0);
	test_mva01s(0xade6, "cm.mva01s s3,s1", mva01ss3s1);
	test_mva01s(0xadea, "cm.mva01s s3,s2", mva01ss3s2);
	test_mva01s(0xadee, "cm.mva01s s3,s3", mva01ss3s3);
	test_mva01s(0xadf2, "cm.mva01s s3,s4", mva01ss3s4);
	test_mva01s(0xadf6, "cm.mva01s s3,s5", mva01ss3s5);
	test_mva01s(0xadfa, "cm.mva01s s3,s6", mva01ss3s6);
	test_mva01s(0xadfe, "cm.mva01s s3,s7", mva01ss3s7);
	test_mva01s(0xae62, "cm.mva01s s4,s0", mva01ss4s0);
	test_mva01s(0xae66, "cm.mva01s s4,s1", mva01ss4s1);
	test_mva01s(0xae6a, "cm.mva01s s4,s2", mva01ss4s2);
	test_mva01s(0xae6e, "cm.mva01s s4,s3", mva01ss4s3);
	test_mva01s(0xae72, "cm.mva01s s4,s4", mva01ss4s4);
	test_mva01s(0xae76, "cm.mva01s s4,s5", mva01ss4s5);
	test_mva01s(0xae7a, "cm.mva01s s4,s6", mva01ss4s6);
	test_mva01s(0xae7e, "cm.mva01s s4,s7", mva01ss4s7);
	test_mva01s(0xaee2, "cm.mva01s s5,s0", mva01ss5s0);
	test_mva01s(0xaee6, "cm.mva01s s5,s1", mva01ss5s1);
	test_mva01s(0xaeea, "cm.mva01s s5,s2", mva01ss5s2);
	test_mva01s(0xaeee, "cm.mva01s s5,s3", mva01ss5s3);
	test_mva01s(0xaef2, "cm.mva01s s5,s4", mva01ss5s4);
	test_mva01s(0xaef6, "cm.mva01s s5,s5", mva01ss5s5);
	test_mva01s(0xaefa, "cm.mva01s s5,s6", mva01ss5s6);
	test_mva01s(0xaefe, "cm.mva01s s5,s7", mva01ss5s7);
	test_mva01s(0xaf62, "cm.mva01s s6,s0", mva01ss6s0);
	test_mva01s(0xaf66, "cm.mva01s s6,s1", mva01ss6s1);
	test_mva01s(0xaf6a, "cm.mva01s s6,s2", mva01ss6s2);
	test_mva01s(0xaf6e, "cm.mva01s s6,s3", mva01ss6s3);
	test_mva01s(0xaf72, "cm.mva01s s6,s4", mva01ss6s4);
	test_mva01s(0xaf76, "cm.mva01s s6,s5", mva01ss6s5);
	test_mva01s(0xaf7a, "cm.mva01s s6,s6", mva01ss6s6);
	test_mva01s(0xaf7e, "cm.mva01s s6,s7", mva01ss6s7);
	test_mva01s(0xafe2, "cm.mva01s s7,s0", mva01ss7s0);
	test_mva01s(0xafe6, "cm.mva01s s7,s1", mva01ss7s1);
	test_mva01s(0xafea, "cm.mva01s s7,s2", mva01ss7s2);
	test_mva01s(0xafee, "cm.mva01s s7,s3", mva01ss7s3);
	test_mva01s(0xaff2, "cm.mva01s s7,s4", mva01ss7s4);
	test_mva01s(0xaff6, "cm.mva01s s7,s5", mva01ss7s5);
	test_mva01s(0xaffa, "cm.mva01s s7,s6", mva01ss7s6);
	test_mva01s(0xaffe, "cm.mva01s s7,s7", mva01ss7s7);

	if (global_result_mva01s == 0)
	{
		neorv32_uart0_printf("\nAll tests passed successfully.\n");
	}
	else
	{
		neorv32_uart0_printf("\nSome tests failed.\n");
	}
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
