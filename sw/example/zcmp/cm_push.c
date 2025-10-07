#include <neorv32.h>


#define FRAME_SIZE_WORDS 32
static uint32_t test_frame[FRAME_SIZE_WORDS];
static uint32_t sp_save;

#define BAUD_RATE 19200

#define test_zcmp_push(instr_bits, instr_name, test_frame) \
	test_sp = &test_frame[FRAME_SIZE_WORDS];\
	neorv32_uart0_printf("Test: " instr_name "\n"); \
	for (int i = 0; i < 32; ++i) test_frame[i] = 0xdead0000 + i; \
	asm volatile ( \
		/* Save all clobbered registers on the real stack */ \
		"_test_%=:\n" \
		"addi sp, sp, -64\n" \
		"sw x1,   0(sp)\n" \
		"sw x8,   4(sp)\n" \
		"sw x9,   8(sp)\n" \
		"sw x18, 12(sp)\n" \
		"sw x19, 16(sp)\n" \
		"sw x20, 20(sp)\n" \
		"sw x21, 24(sp)\n" \
		"sw x22, 28(sp)\n" \
		"sw x23, 32(sp)\n" \
		"sw x24, 36(sp)\n" \
		"sw x25, 40(sp)\n" \
		"sw x26, 44(sp)\n" \
		"sw x27, 48(sp)\n" \
		/* Save stack pointer and install test sp */ \
		"la x1, sp_save\n" \
		"sw sp, 0(x1)\n" \
		"mv sp, %0\n" \
		/* Give unique values to all registers that can be pushed */ \
		"li x1,  0xa5000000 + 1\n" \
		"li x8,  0xa5000000 + 8\n" \
		"li x9,  0xa5000000 + 9\n" \
		"li x18, 0xa5000000 + 18\n" \
		"li x19, 0xa5000000 + 19\n" \
		"li x20, 0xa5000000 + 20\n" \
		"li x21, 0xa5000000 + 21\n" \
		"li x22, 0xa5000000 + 22\n" \
		"li x23, 0xa5000000 + 23\n" \
		"li x24, 0xa5000000 + 24\n" \
		"li x25, 0xa5000000 + 25\n" \
		"li x26, 0xa5000000 + 26\n" \
		"li x27, 0xa5000000 + 27\n" \
		/* Let's go */ \
		".hword " #instr_bits "\n" \
		/* Updated test sp is returned  */ \
		"mv a0, sp\n" \
		/* Restore original sp, and clobbered registers */ \
		"la x1, sp_save\n" \
		"lw sp,   0(x1)\n" \
		"lw x1,   0(sp)\n" \
		"lw x8,   4(sp)\n" \
		"lw x9,   8(sp)\n" \
		"lw x18, 12(sp)\n" \
		"lw x19, 16(sp)\n" \
		"lw x20, 20(sp)\n" \
		"lw x21, 24(sp)\n" \
		"lw x22, 28(sp)\n" \
		"lw x23, 32(sp)\n" \
		"lw x24, 36(sp)\n" \
		"lw x25, 40(sp)\n" \
		"lw x26, 44(sp)\n" \
		"lw x27, 48(sp)\n" \
		"addi sp, sp, 64\n" \
		"mv %0, a0\n" \
		: "+r"(test_sp) : : "a0"\
	); \
    neorv32_uart0_printf("SP diff: %x\n", (uint32_t)test_sp - (uint32_t)&test_frame[FRAME_SIZE_WORDS]);              \
	neorv32_uart0_printf("Frame data\n"); \
	for (int i = 0; i < FRAME_SIZE_WORDS; ++i) neorv32_uart0_printf("%x\n",test_frame[i]); \
	neorv32_uart0_printf("\n");

int cm_push() {
	volatile uint32_t *test_sp;
	test_zcmp_push(0xb842, "cm.push {ra},-16"         , test_frame);
	test_zcmp_push(0xb846, "cm.push {ra},-32"         , test_frame);
	test_zcmp_push(0xb84a, "cm.push {ra},-48"         , test_frame);
	test_zcmp_push(0xb84e, "cm.push {ra},-64"         , test_frame);
	test_zcmp_push(0xb852, "cm.push {ra,s0},-16"      , test_frame);
	test_zcmp_push(0xb856, "cm.push {ra,s0},-32"      , test_frame);
	test_zcmp_push(0xb85a, "cm.push {ra,s0},-48"      , test_frame);
	test_zcmp_push(0xb85e, "cm.push {ra,s0},-64"      , test_frame);
	test_zcmp_push(0xb862, "cm.push {ra,s0-s1},-16"   , test_frame);
	test_zcmp_push(0xb866, "cm.push {ra,s0-s1},-32"   , test_frame);
	test_zcmp_push(0xb86a, "cm.push {ra,s0-s1},-48"   , test_frame);
	test_zcmp_push(0xb86e, "cm.push {ra,s0-s1},-64"   , test_frame);
	test_zcmp_push(0xb872, "cm.push {ra,s0-s2},-16"   , test_frame);
	test_zcmp_push(0xb876, "cm.push {ra,s0-s2},-32"   , test_frame);
	test_zcmp_push(0xb87a, "cm.push {ra,s0-s2},-48"   , test_frame);
	test_zcmp_push(0xb87e, "cm.push {ra,s0-s2},-64"   , test_frame);
	test_zcmp_push(0xb882, "cm.push {ra,s0-s3},-32"   , test_frame);
	test_zcmp_push(0xb886, "cm.push {ra,s0-s3},-48"   , test_frame);
	test_zcmp_push(0xb88a, "cm.push {ra,s0-s3},-64"   , test_frame);
	test_zcmp_push(0xb88e, "cm.push {ra,s0-s3},-80"   , test_frame);
	test_zcmp_push(0xb892, "cm.push {ra,s0-s4},-32"   , test_frame);
	test_zcmp_push(0xb896, "cm.push {ra,s0-s4},-48"   , test_frame);
	test_zcmp_push(0xb89a, "cm.push {ra,s0-s4},-64"   , test_frame);
	test_zcmp_push(0xb89e, "cm.push {ra,s0-s4},-80"   , test_frame);
	test_zcmp_push(0xb8a2, "cm.push {ra,s0-s5},-32"   , test_frame);
	test_zcmp_push(0xb8a6, "cm.push {ra,s0-s5},-48"   , test_frame);
	test_zcmp_push(0xb8aa, "cm.push {ra,s0-s5},-64"   , test_frame);
	test_zcmp_push(0xb8ae, "cm.push {ra,s0-s5},-80"   , test_frame);
	test_zcmp_push(0xb8b2, "cm.push {ra,s0-s6},-32"   , test_frame);
	test_zcmp_push(0xb8b6, "cm.push {ra,s0-s6},-48"   , test_frame);
	test_zcmp_push(0xb8ba, "cm.push {ra,s0-s6},-64"   , test_frame);
	test_zcmp_push(0xb8be, "cm.push {ra,s0-s6},-80"   , test_frame);
	test_zcmp_push(0xb8c2, "cm.push {ra,s0-s7},-48"   , test_frame);
	test_zcmp_push(0xb8c6, "cm.push {ra,s0-s7},-64"   , test_frame);
	test_zcmp_push(0xb8ca, "cm.push {ra,s0-s7},-80"   , test_frame);
	test_zcmp_push(0xb8ce, "cm.push {ra,s0-s7},-96"   , test_frame);
	test_zcmp_push(0xb8d2, "cm.push {ra,s0-s8},-48"   , test_frame);
	test_zcmp_push(0xb8d6, "cm.push {ra,s0-s8},-64"   , test_frame);
	test_zcmp_push(0xb8da, "cm.push {ra,s0-s8},-80"   , test_frame);
	test_zcmp_push(0xb8de, "cm.push {ra,s0-s8},-96"   , test_frame);
	test_zcmp_push(0xb8e2, "cm.push {ra,s0-s9},-48"   , test_frame);
	test_zcmp_push(0xb8e6, "cm.push {ra,s0-s9},-64"   , test_frame);
	test_zcmp_push(0xb8ea, "cm.push {ra,s0-s9},-80"   , test_frame);
	test_zcmp_push(0xb8ee, "cm.push {ra,s0-s9},-96"   , test_frame);
	test_zcmp_push(0xb8f2, "cm.push {ra,s0-s11},-64"  , test_frame);
	test_zcmp_push(0xb8f6, "cm.push {ra,s0-s11},-80"  , test_frame);
	test_zcmp_push(0xb8fa, "cm.push {ra,s0-s11},-96"  , test_frame);
	test_zcmp_push(0xb8fe, "cm.push {ra,s0-s11},-112" , test_frame);

	return 0;
}

// Test output extracted from spike:
/*EXPECTED-OUTPUT***************************************************************

Test: cm.push {ra},-16
SP diff:
fffffff0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
dead001b
dead001c
dead001d
dead001e
a5000001

Test: cm.push {ra},-32
SP diff:
ffffffe0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
dead001b
dead001c
dead001d
dead001e
a5000001

Test: cm.push {ra},-48
SP diff:
ffffffd0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
dead001b
dead001c
dead001d
dead001e
a5000001

Test: cm.push {ra},-64
SP diff:
ffffffc0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
dead001b
dead001c
dead001d
dead001e
a5000001

Test: cm.push {ra,s0},-16
SP diff:
fffffff0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
dead001b
dead001c
dead001d
a5000001
a5000008

Test: cm.push {ra,s0},-32
SP diff:
ffffffe0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
dead001b
dead001c
dead001d
a5000001
a5000008

Test: cm.push {ra,s0},-48
SP diff:
ffffffd0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
dead001b
dead001c
dead001d
a5000001
a5000008

Test: cm.push {ra,s0},-64
SP diff:
ffffffc0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
dead001b
dead001c
dead001d
a5000001
a5000008

Test: cm.push {ra,s0-s1},-16
SP diff:
fffffff0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
dead001b
dead001c
a5000001
a5000008
a5000009

Test: cm.push {ra,s0-s1},-32
SP diff:
ffffffe0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
dead001b
dead001c
a5000001
a5000008
a5000009

Test: cm.push {ra,s0-s1},-48
SP diff:
ffffffd0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
dead001b
dead001c
a5000001
a5000008
a5000009

Test: cm.push {ra,s0-s1},-64
SP diff:
ffffffc0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
dead001b
dead001c
a5000001
a5000008
a5000009

Test: cm.push {ra,s0-s2},-16
SP diff:
fffffff0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
dead001b
a5000001
a5000008
a5000009
a5000012

Test: cm.push {ra,s0-s2},-32
SP diff:
ffffffe0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
dead001b
a5000001
a5000008
a5000009
a5000012

Test: cm.push {ra,s0-s2},-48
SP diff:
ffffffd0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
dead001b
a5000001
a5000008
a5000009
a5000012

Test: cm.push {ra,s0-s2},-64
SP diff:
ffffffc0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
dead001b
a5000001
a5000008
a5000009
a5000012

Test: cm.push {ra,s0-s3},-32
SP diff:
ffffffe0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
a5000001
a5000008
a5000009
a5000012
a5000013

Test: cm.push {ra,s0-s3},-48
SP diff:
ffffffd0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
a5000001
a5000008
a5000009
a5000012
a5000013

Test: cm.push {ra,s0-s3},-64
SP diff:
ffffffc0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
a5000001
a5000008
a5000009
a5000012
a5000013

Test: cm.push {ra,s0-s3},-80
SP diff:
ffffffb0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
dead001a
a5000001
a5000008
a5000009
a5000012
a5000013

Test: cm.push {ra,s0-s4},-32
SP diff:
ffffffe0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014

Test: cm.push {ra,s0-s4},-48
SP diff:
ffffffd0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014

Test: cm.push {ra,s0-s4},-64
SP diff:
ffffffc0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014

Test: cm.push {ra,s0-s4},-80
SP diff:
ffffffb0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
dead0019
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014

Test: cm.push {ra,s0-s5},-32
SP diff:
ffffffe0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015

Test: cm.push {ra,s0-s5},-48
SP diff:
ffffffd0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015

Test: cm.push {ra,s0-s5},-64
SP diff:
ffffffc0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015

Test: cm.push {ra,s0-s5},-80
SP diff:
ffffffb0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
dead0018
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015

Test: cm.push {ra,s0-s6},-32
SP diff:
ffffffe0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016

Test: cm.push {ra,s0-s6},-48
SP diff:
ffffffd0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016

Test: cm.push {ra,s0-s6},-64
SP diff:
ffffffc0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016

Test: cm.push {ra,s0-s6},-80
SP diff:
ffffffb0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
dead0017
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016

Test: cm.push {ra,s0-s7},-48
SP diff:
ffffffd0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016
a5000017

Test: cm.push {ra,s0-s7},-64
SP diff:
ffffffc0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016
a5000017

Test: cm.push {ra,s0-s7},-80
SP diff:
ffffffb0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016
a5000017

Test: cm.push {ra,s0-s7},-96
SP diff:
ffffffa0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
dead0016
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016
a5000017

Test: cm.push {ra,s0-s8},-48
SP diff:
ffffffd0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016
a5000017
a5000018

Test: cm.push {ra,s0-s8},-64
SP diff:
ffffffc0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016
a5000017
a5000018

Test: cm.push {ra,s0-s8},-80
SP diff:
ffffffb0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016
a5000017
a5000018

Test: cm.push {ra,s0-s8},-96
SP diff:
ffffffa0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
dead0015
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016
a5000017
a5000018

Test: cm.push {ra,s0-s9},-48
SP diff:
ffffffd0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016
a5000017
a5000018
a5000019

Test: cm.push {ra,s0-s9},-64
SP diff:
ffffffc0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016
a5000017
a5000018
a5000019

Test: cm.push {ra,s0-s9},-80
SP diff:
ffffffb0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016
a5000017
a5000018
a5000019

Test: cm.push {ra,s0-s9},-96
SP diff:
ffffffa0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
dead0013
dead0014
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016
a5000017
a5000018
a5000019

Test: cm.push {ra,s0-s11},-64
SP diff:
ffffffc0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016
a5000017
a5000018
a5000019
a500001a
a500001b

Test: cm.push {ra,s0-s11},-80
SP diff:
ffffffb0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016
a5000017
a5000018
a5000019
a500001a
a500001b

Test: cm.push {ra,s0-s11},-96
SP diff:
ffffffa0
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016
a5000017
a5000018
a5000019
a500001a
a500001b

Test: cm.push {ra,s0-s11},-112
SP diff:
ffffff90
Frame data
dead0000
dead0001
dead0002
dead0003
dead0004
dead0005
dead0006
dead0007
dead0008
dead0009
dead000a
dead000b
dead000c
dead000d
dead000e
dead000f
dead0010
dead0011
dead0012
a5000001
a5000008
a5000009
a5000012
a5000013
a5000014
a5000015
a5000016
a5000017
a5000018
a5000019
a500001a
a500001b

*******************************************************************************/
