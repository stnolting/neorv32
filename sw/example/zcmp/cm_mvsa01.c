#include <cm_mvsa01.h>

// Test output extracted from spike:
uint32_t cmvsa01s0s1[10] = {0xa500000a, 0xa500000b, 0xa500000a, 0xa500000b, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s0s2[10] = {0xa500000a, 0xa500000b, 0xa500000a, 0xa5000009, 0xa500000b, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s0s3[10] = {0xa500000a, 0xa500000b, 0xa500000a, 0xa5000009, 0xa5000012, 0xa500000b, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s0s4[10] = {0xa500000a, 0xa500000b, 0xa500000a, 0xa5000009, 0xa5000012, 0xa5000013, 0xa500000b, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s0s5[10] = {0xa500000a, 0xa500000b, 0xa500000a, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa500000b, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s0s6[10] = {0xa500000a, 0xa500000b, 0xa500000a, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa500000b, 0xa5000017};
uint32_t cmvsa01s0s7[10] = {0xa500000a, 0xa500000b, 0xa500000a, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa500000b};
uint32_t cmvsa01s1s0[10] = {0xa500000a, 0xa500000b, 0xa500000b, 0xa500000a, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s1s2[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa500000a, 0xa500000b, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s1s3[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa500000a, 0xa5000012, 0xa500000b, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s1s4[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa500000a, 0xa5000012, 0xa5000013, 0xa500000b, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s1s5[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa500000a, 0xa5000012, 0xa5000013, 0xa5000014, 0xa500000b, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s1s6[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa500000a, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa500000b, 0xa5000017};
uint32_t cmvsa01s1s7[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa500000a, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa500000b};
uint32_t cmvsa01s2s0[10] = {0xa500000a, 0xa500000b, 0xa500000b, 0xa5000009, 0xa500000a, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s2s1[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa500000b, 0xa500000a, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s2s3[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa500000a, 0xa500000b, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s2s4[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa500000a, 0xa5000013, 0xa500000b, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s2s5[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa500000a, 0xa5000013, 0xa5000014, 0xa500000b, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s2s6[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa500000a, 0xa5000013, 0xa5000014, 0xa5000015, 0xa500000b, 0xa5000017};
uint32_t cmvsa01s2s7[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa500000a, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa500000b};
uint32_t cmvsa01s3s0[10] = {0xa500000a, 0xa500000b, 0xa500000b, 0xa5000009, 0xa5000012, 0xa500000a, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s3s1[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa500000b, 0xa5000012, 0xa500000a, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s3s2[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa500000b, 0xa500000a, 0xa5000014, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s3s4[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa500000a, 0xa500000b, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s3s5[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa500000a, 0xa5000014, 0xa500000b, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s3s6[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa500000a, 0xa5000014, 0xa5000015, 0xa500000b, 0xa5000017};
uint32_t cmvsa01s3s7[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa500000a, 0xa5000014, 0xa5000015, 0xa5000016, 0xa500000b};
uint32_t cmvsa01s4s0[10] = {0xa500000a, 0xa500000b, 0xa500000b, 0xa5000009, 0xa5000012, 0xa5000013, 0xa500000a, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s4s1[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa500000b, 0xa5000012, 0xa5000013, 0xa500000a, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s4s2[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa500000b, 0xa5000013, 0xa500000a, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s4s3[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa500000b, 0xa500000a, 0xa5000015, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s4s5[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa500000a, 0xa500000b, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s4s6[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa500000a, 0xa5000015, 0xa500000b, 0xa5000017};
uint32_t cmvsa01s4s7[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa500000a, 0xa5000015, 0xa5000016, 0xa500000b};
uint32_t cmvsa01s5s0[10] = {0xa500000a, 0xa500000b, 0xa500000b, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa500000a, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s5s1[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa500000b, 0xa5000012, 0xa5000013, 0xa5000014, 0xa500000a, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s5s2[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa500000b, 0xa5000013, 0xa5000014, 0xa500000a, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s5s3[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa500000b, 0xa5000014, 0xa500000a, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s5s4[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa500000b, 0xa500000a, 0xa5000016, 0xa5000017};
uint32_t cmvsa01s5s6[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa500000a, 0xa500000b, 0xa5000017};
uint32_t cmvsa01s5s7[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa500000a, 0xa5000016, 0xa500000b};
uint32_t cmvsa01s6s0[10] = {0xa500000a, 0xa500000b, 0xa500000b, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa500000a, 0xa5000017};
uint32_t cmvsa01s6s1[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa500000b, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa500000a, 0xa5000017};
uint32_t cmvsa01s6s2[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa500000b, 0xa5000013, 0xa5000014, 0xa5000015, 0xa500000a, 0xa5000017};
uint32_t cmvsa01s6s3[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa500000b, 0xa5000014, 0xa5000015, 0xa500000a, 0xa5000017};
uint32_t cmvsa01s6s4[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa500000b, 0xa5000015, 0xa500000a, 0xa5000017};
uint32_t cmvsa01s6s5[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa500000b, 0xa500000a, 0xa5000017};
uint32_t cmvsa01s6s7[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa500000a, 0xa500000b};
uint32_t cmvsa01s7s0[10] = {0xa500000a, 0xa500000b, 0xa500000b, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa500000a};
uint32_t cmvsa01s7s1[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa500000b, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa500000a};
uint32_t cmvsa01s7s2[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa500000b, 0xa5000013, 0xa5000014, 0xa5000015, 0xa5000016, 0xa500000a};
uint32_t cmvsa01s7s3[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa500000b, 0xa5000014, 0xa5000015, 0xa5000016, 0xa500000a};
uint32_t cmvsa01s7s4[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa500000b, 0xa5000015, 0xa5000016, 0xa500000a};
uint32_t cmvsa01s7s5[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa500000b, 0xa5000016, 0xa500000a};
uint32_t cmvsa01s7s6[10] = {0xa500000a, 0xa500000b, 0xa5000008, 0xa5000009, 0xa5000012, 0xa5000013, 0xa5000014, 0xa5000015, 0xa500000b, 0xa500000a};

uint8_t local_result_mvsa01 = 0;
uint8_t global_result_mvsa01 = 0;

#define test_mvsa01(instr_bits, instr_name, arr)                                              \
	neorv32_uart0_printf(instr_name);                                                         \
	local_result_mvsa01 = 0;                                                                         \
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
			local_result_mvsa01 = 1;                                                                 \
			global_result_mvsa01 = 1;                                                                \
		}                                                                                     \
	}                                                                                         \
	if (local_result_mvsa01 == 0)                                                                    \
	{                                                                                         \
		neorv32_uart0_printf(" - OK\n");                                                      \
	}                                                                                         \
	else                                                                                      \
	{                                                                                         \
		neorv32_uart0_printf(" - FAIL\n");                                                    \
	}

int cm_mvsa01()
{
	test_mvsa01(0xac26, "cm.mvsa01 s0,s1", cmvsa01s0s1);
	test_mvsa01(0xac2a, "cm.mvsa01 s0,s2", cmvsa01s0s2);
	test_mvsa01(0xac2e, "cm.mvsa01 s0,s3", cmvsa01s0s3);
	test_mvsa01(0xac32, "cm.mvsa01 s0,s4", cmvsa01s0s4);
	test_mvsa01(0xac36, "cm.mvsa01 s0,s5", cmvsa01s0s5);
	test_mvsa01(0xac3a, "cm.mvsa01 s0,s6", cmvsa01s0s6);
	test_mvsa01(0xac3e, "cm.mvsa01 s0,s7", cmvsa01s0s7);
	test_mvsa01(0xaca2, "cm.mvsa01 s1,s0", cmvsa01s1s0);
	test_mvsa01(0xacaa, "cm.mvsa01 s1,s2", cmvsa01s1s2);
	test_mvsa01(0xacae, "cm.mvsa01 s1,s3", cmvsa01s1s3);
	test_mvsa01(0xacb2, "cm.mvsa01 s1,s4", cmvsa01s1s4);
	test_mvsa01(0xacb6, "cm.mvsa01 s1,s5", cmvsa01s1s5);
	test_mvsa01(0xacba, "cm.mvsa01 s1,s6", cmvsa01s1s6);
	test_mvsa01(0xacbe, "cm.mvsa01 s1,s7", cmvsa01s1s7);
	test_mvsa01(0xad22, "cm.mvsa01 s2,s0", cmvsa01s2s0);
	test_mvsa01(0xad26, "cm.mvsa01 s2,s1", cmvsa01s2s1);
	test_mvsa01(0xad2e, "cm.mvsa01 s2,s3", cmvsa01s2s3);
	test_mvsa01(0xad32, "cm.mvsa01 s2,s4", cmvsa01s2s4);
	test_mvsa01(0xad36, "cm.mvsa01 s2,s5", cmvsa01s2s5);
	test_mvsa01(0xad3a, "cm.mvsa01 s2,s6", cmvsa01s2s6);
	test_mvsa01(0xad3e, "cm.mvsa01 s2,s7", cmvsa01s2s7);
	test_mvsa01(0xada2, "cm.mvsa01 s3,s0", cmvsa01s3s0);
	test_mvsa01(0xada6, "cm.mvsa01 s3,s1", cmvsa01s3s1);
	test_mvsa01(0xadaa, "cm.mvsa01 s3,s2", cmvsa01s3s2);
	test_mvsa01(0xadb2, "cm.mvsa01 s3,s4", cmvsa01s3s4);
	test_mvsa01(0xadb6, "cm.mvsa01 s3,s5", cmvsa01s3s5);
	test_mvsa01(0xadba, "cm.mvsa01 s3,s6", cmvsa01s3s6);
	test_mvsa01(0xadbe, "cm.mvsa01 s3,s7", cmvsa01s3s7);
	test_mvsa01(0xae22, "cm.mvsa01 s4,s0", cmvsa01s4s0);
	test_mvsa01(0xae26, "cm.mvsa01 s4,s1", cmvsa01s4s1);
	test_mvsa01(0xae2a, "cm.mvsa01 s4,s2", cmvsa01s4s2);
	test_mvsa01(0xae2e, "cm.mvsa01 s4,s3", cmvsa01s4s3);
	test_mvsa01(0xae36, "cm.mvsa01 s4,s5", cmvsa01s4s5);
	test_mvsa01(0xae3a, "cm.mvsa01 s4,s6", cmvsa01s4s6);
	test_mvsa01(0xae3e, "cm.mvsa01 s4,s7", cmvsa01s4s7);
	test_mvsa01(0xaea2, "cm.mvsa01 s5,s0", cmvsa01s5s0);
	test_mvsa01(0xaea6, "cm.mvsa01 s5,s1", cmvsa01s5s1);
	test_mvsa01(0xaeaa, "cm.mvsa01 s5,s2", cmvsa01s5s2);
	test_mvsa01(0xaeae, "cm.mvsa01 s5,s3", cmvsa01s5s3);
	test_mvsa01(0xaeb2, "cm.mvsa01 s5,s4", cmvsa01s5s4);
	test_mvsa01(0xaeba, "cm.mvsa01 s5,s6", cmvsa01s5s6);
	test_mvsa01(0xaebe, "cm.mvsa01 s5,s7", cmvsa01s5s7);
	test_mvsa01(0xaf22, "cm.mvsa01 s6,s0", cmvsa01s6s0);
	test_mvsa01(0xaf26, "cm.mvsa01 s6,s1", cmvsa01s6s1);
	test_mvsa01(0xaf2a, "cm.mvsa01 s6,s2", cmvsa01s6s2);
	test_mvsa01(0xaf2e, "cm.mvsa01 s6,s3", cmvsa01s6s3);
	test_mvsa01(0xaf32, "cm.mvsa01 s6,s4", cmvsa01s6s4);
	test_mvsa01(0xaf36, "cm.mvsa01 s6,s5", cmvsa01s6s5);
	test_mvsa01(0xaf3e, "cm.mvsa01 s6,s7", cmvsa01s6s7);
	test_mvsa01(0xafa2, "cm.mvsa01 s7,s0", cmvsa01s7s0);
	test_mvsa01(0xafa6, "cm.mvsa01 s7,s1", cmvsa01s7s1);
	test_mvsa01(0xafaa, "cm.mvsa01 s7,s2", cmvsa01s7s2);
	test_mvsa01(0xafae, "cm.mvsa01 s7,s3", cmvsa01s7s3);
	test_mvsa01(0xafb2, "cm.mvsa01 s7,s4", cmvsa01s7s4);
	test_mvsa01(0xafb6, "cm.mvsa01 s7,s5", cmvsa01s7s5);
	test_mvsa01(0xafba, "cm.mvsa01 s7,s6", cmvsa01s7s6);

	if (global_result_mvsa01 == 0)
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

Test: cm.mvsa01 s0,s1
a0: a500000a
a1: a500000b
s0: a500000a
s1: a500000b
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s0,s2
a0: a500000a
a1: a500000b
s0: a500000a
s1: a5000009
s2: a500000b
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s0,s3
a0: a500000a
a1: a500000b
s0: a500000a
s1: a5000009
s2: a5000012
s3: a500000b
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s0,s4
a0: a500000a
a1: a500000b
s0: a500000a
s1: a5000009
s2: a5000012
s3: a5000013
s4: a500000b
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s0,s5
a0: a500000a
a1: a500000b
s0: a500000a
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a500000b
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s0,s6
a0: a500000a
a1: a500000b
s0: a500000a
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a500000b
s7: a5000017

Test: cm.mvsa01 s0,s7
a0: a500000a
a1: a500000b
s0: a500000a
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a500000b

Test: cm.mvsa01 s1,s0
a0: a500000a
a1: a500000b
s0: a500000b
s1: a500000a
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s1,s2
a0: a500000a
a1: a500000b
s0: a5000008
s1: a500000a
s2: a500000b
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s1,s3
a0: a500000a
a1: a500000b
s0: a5000008
s1: a500000a
s2: a5000012
s3: a500000b
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s1,s4
a0: a500000a
a1: a500000b
s0: a5000008
s1: a500000a
s2: a5000012
s3: a5000013
s4: a500000b
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s1,s5
a0: a500000a
a1: a500000b
s0: a5000008
s1: a500000a
s2: a5000012
s3: a5000013
s4: a5000014
s5: a500000b
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s1,s6
a0: a500000a
a1: a500000b
s0: a5000008
s1: a500000a
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a500000b
s7: a5000017

Test: cm.mvsa01 s1,s7
a0: a500000a
a1: a500000b
s0: a5000008
s1: a500000a
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a500000b

Test: cm.mvsa01 s2,s0
a0: a500000a
a1: a500000b
s0: a500000b
s1: a5000009
s2: a500000a
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s2,s1
a0: a500000a
a1: a500000b
s0: a5000008
s1: a500000b
s2: a500000a
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s2,s3
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a500000a
s3: a500000b
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s2,s4
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a500000a
s3: a5000013
s4: a500000b
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s2,s5
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a500000a
s3: a5000013
s4: a5000014
s5: a500000b
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s2,s6
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a500000a
s3: a5000013
s4: a5000014
s5: a5000015
s6: a500000b
s7: a5000017

Test: cm.mvsa01 s2,s7
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a500000a
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a500000b

Test: cm.mvsa01 s3,s0
a0: a500000a
a1: a500000b
s0: a500000b
s1: a5000009
s2: a5000012
s3: a500000a
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s3,s1
a0: a500000a
a1: a500000b
s0: a5000008
s1: a500000b
s2: a5000012
s3: a500000a
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s3,s2
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a500000b
s3: a500000a
s4: a5000014
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s3,s4
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a500000a
s4: a500000b
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s3,s5
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a500000a
s4: a5000014
s5: a500000b
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s3,s6
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a500000a
s4: a5000014
s5: a5000015
s6: a500000b
s7: a5000017

Test: cm.mvsa01 s3,s7
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a500000a
s4: a5000014
s5: a5000015
s6: a5000016
s7: a500000b

Test: cm.mvsa01 s4,s0
a0: a500000a
a1: a500000b
s0: a500000b
s1: a5000009
s2: a5000012
s3: a5000013
s4: a500000a
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s4,s1
a0: a500000a
a1: a500000b
s0: a5000008
s1: a500000b
s2: a5000012
s3: a5000013
s4: a500000a
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s4,s2
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a500000b
s3: a5000013
s4: a500000a
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s4,s3
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a500000b
s4: a500000a
s5: a5000015
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s4,s5
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a500000a
s5: a500000b
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s4,s6
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a500000a
s5: a5000015
s6: a500000b
s7: a5000017

Test: cm.mvsa01 s4,s7
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a500000a
s5: a5000015
s6: a5000016
s7: a500000b

Test: cm.mvsa01 s5,s0
a0: a500000a
a1: a500000b
s0: a500000b
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a500000a
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s5,s1
a0: a500000a
a1: a500000b
s0: a5000008
s1: a500000b
s2: a5000012
s3: a5000013
s4: a5000014
s5: a500000a
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s5,s2
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a500000b
s3: a5000013
s4: a5000014
s5: a500000a
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s5,s3
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a500000b
s4: a5000014
s5: a500000a
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s5,s4
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a500000b
s5: a500000a
s6: a5000016
s7: a5000017

Test: cm.mvsa01 s5,s6
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a500000a
s6: a500000b
s7: a5000017

Test: cm.mvsa01 s5,s7
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a500000a
s6: a5000016
s7: a500000b

Test: cm.mvsa01 s6,s0
a0: a500000a
a1: a500000b
s0: a500000b
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a500000a
s7: a5000017

Test: cm.mvsa01 s6,s1
a0: a500000a
a1: a500000b
s0: a5000008
s1: a500000b
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a500000a
s7: a5000017

Test: cm.mvsa01 s6,s2
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a500000b
s3: a5000013
s4: a5000014
s5: a5000015
s6: a500000a
s7: a5000017

Test: cm.mvsa01 s6,s3
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a500000b
s4: a5000014
s5: a5000015
s6: a500000a
s7: a5000017

Test: cm.mvsa01 s6,s4
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a500000b
s5: a5000015
s6: a500000a
s7: a5000017

Test: cm.mvsa01 s6,s5
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a500000b
s6: a500000a
s7: a5000017

Test: cm.mvsa01 s6,s7
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a500000a
s7: a500000b

Test: cm.mvsa01 s7,s0
a0: a500000a
a1: a500000b
s0: a500000b
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a500000a

Test: cm.mvsa01 s7,s1
a0: a500000a
a1: a500000b
s0: a5000008
s1: a500000b
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a500000a

Test: cm.mvsa01 s7,s2
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a500000b
s3: a5000013
s4: a5000014
s5: a5000015
s6: a5000016
s7: a500000a

Test: cm.mvsa01 s7,s3
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a500000b
s4: a5000014
s5: a5000015
s6: a5000016
s7: a500000a

Test: cm.mvsa01 s7,s4
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a500000b
s5: a5000015
s6: a5000016
s7: a500000a

Test: cm.mvsa01 s7,s5
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a500000b
s6: a5000016
s7: a500000a

Test: cm.mvsa01 s7,s6
a0: a500000a
a1: a500000b
s0: a5000008
s1: a5000009
s2: a5000012
s3: a5000013
s4: a5000014
s5: a5000015
s6: a500000b
s7: a500000a

*******************************************************************************/
