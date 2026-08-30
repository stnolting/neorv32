// Zcmp reserved-encoding tests
//
// The Zcmp spec reserves cm.push/cm.pop/cm.popret/cm.popretz encodings with
// rlist < 4 and cm.mvsa01 with r1s' == r2s'. These must raise an
// illegal-instruction exception instead of executing a bogus micro-op
// sequence.
//
// Uses the same minimal mscratch-swap trap handler approach as cm_exc.c:
// the handler records mepc/mcause, advances mepc by +2 (compressed
// instruction) and returns. A correct implementation traps exactly once per
// reserved instruction; a broken one executes the micro-op sequence and
// never traps (exc_count stays 0).
//
// For the popret/popretz variants the (buggy) micro-op sequence would end
// in "jalr x0, 0(ra)" with ra loaded from the test frame. To keep the test
// recoverable on buggy hardware, the whole frame is pre-filled with the
// address of a landing pad, so the bogus jump lands somewhere controlled.

#include <neorv32.h>
#include "cm_resv.h"

// ---------------------------------------------------------------------------
// Trap recording
// ---------------------------------------------------------------------------
static volatile uint32_t resv_mepc;
static volatile uint32_t resv_mcause;
static volatile uint32_t resv_count;
static volatile uint32_t resv_sp_save;

static uint32_t resv_stack[64] __attribute__((aligned(16)));

#define RESV_FRAME_WORDS 32
static uint32_t resv_frame[RESV_FRAME_WORDS] __attribute__((aligned(16)));

// ---------------------------------------------------------------------------
// Minimal trap handler (naked)
//
// Expects mscratch to hold a valid stack pointer (top of resv_stack).
// Records mepc/mcause, advances mepc by 2, returns via mret.
// ---------------------------------------------------------------------------
static void __attribute__((naked, aligned(4))) zcmp_resv_handler(void)
{
  asm volatile(
      "csrrw sp, mscratch, sp    \n"
      "addi  sp, sp, -8          \n"
      "sw    a0, 0(sp)           \n"
      "sw    a1, 4(sp)           \n"
      "csrr  a0, mepc            \n"
      "la    a1, resv_mepc       \n"
      "sw    a0, 0(a1)           \n"
      "csrr  a0, mcause          \n"
      "la    a1, resv_mcause     \n"
      "sw    a0, 0(a1)           \n"
      "la    a1, resv_count      \n"
      "lw    a0, 0(a1)           \n"
      "addi  a0, a0, 1           \n"
      "sw    a0, 0(a1)           \n"
      "csrr  a0, mepc            \n"
      "addi  a0, a0, 2           \n"
      "csrw  mepc, a0            \n"
      "lw    a0, 0(sp)           \n"
      "lw    a1, 4(sp)           \n"
      "addi  sp, sp, 8           \n"
      "csrrw sp, mscratch, sp    \n"
      "mret                      \n");
}

static uint32_t saved_mtvec;

static void install_resv_handler(void)
{
  neorv32_cpu_csr_write(CSR_MSCRATCH, (uint32_t)&resv_stack[63]);
  saved_mtvec = neorv32_cpu_csr_read(CSR_MTVEC);
  neorv32_cpu_csr_write(CSR_MTVEC, (uint32_t)(&zcmp_resv_handler));
}

static void restore_rte_handler(void)
{
  neorv32_cpu_csr_write(CSR_MTVEC, saved_mtvec);
}

// ---------------------------------------------------------------------------
// Common result check
// ---------------------------------------------------------------------------
static int check_result(uint32_t zcmp_pc)
{
  int ok = 1;
  if (resv_count != 1)
  {
    neorv32_uart0_printf("\n  trap count: expected 1, got %u", resv_count);
    ok = 0;
  }
  if (resv_mcause != TRAP_CODE_I_ILLEGAL)
  {
    neorv32_uart0_printf("\n  mcause: expected 0x%x, got 0x%x", (uint32_t)TRAP_CODE_I_ILLEGAL, resv_mcause);
    ok = 0;
  }
  if (resv_mepc != zcmp_pc)
  {
    neorv32_uart0_printf("\n  mepc: expected 0x%x, got 0x%x", zcmp_pc, resv_mepc);
    ok = 0;
  }
  return ok;
}

// ---------------------------------------------------------------------------
// Reserved cm.push (rlist < 4)
//
// A buggy implementation stores ra/s0/s1/s2-s11/t3/t4 to sp+0..sp+56 and
// then executes "addi sp, sp, -16" - it does not trap. Registers are not
// modified by the stores, so only sp has to be preserved manually.
// ---------------------------------------------------------------------------
static int test_resv_push(uint32_t opcode_check) // 0xb802 (rlist=0) or 0xb832 (rlist=3)
{
  uint32_t zcmp_pc = 0;

  resv_count = 0;
  resv_mepc = 0;
  resv_mcause = 0;

  if (opcode_check == 0xb802)
  {
    asm volatile(
        "mv   t1, sp               \n" // save real sp
        "mv   sp, %[tsp]           \n" // sp -> scratch frame
        "la   %[pc], 1f            \n"
        "1: .hword 0xb802          \n" // reserved cm.push, rlist=0
        "mv   sp, t1               \n" // restore sp
        : [pc] "=r"(zcmp_pc)
        : [tsp] "r"(&resv_frame[0])
        : "t1", "memory");
  }
  else
  {
    asm volatile(
        "mv   t1, sp               \n"
        "mv   sp, %[tsp]           \n"
        "la   %[pc], 1f            \n"
        "1: .hword 0xb832          \n" // reserved cm.push, rlist=3
        "mv   sp, t1               \n"
        : [pc] "=r"(zcmp_pc)
        : [tsp] "r"(&resv_frame[0])
        : "t1", "memory");
  }

  return check_result(zcmp_pc);
}

// ---------------------------------------------------------------------------
// Reserved cm.pop (rlist = 0)
//
// A buggy implementation loads ra/s0/s1/s2-s11/t3/t4 from the frame and
// adds +16 to sp - all affected registers are declared as asm clobbers.
// ---------------------------------------------------------------------------
static int test_resv_pop(void)
{
  uint32_t zcmp_pc = 0;

  resv_count = 0;
  resv_mepc = 0;
  resv_mcause = 0;

  for (int i = 0; i < RESV_FRAME_WORDS; i++)
    resv_frame[i] = 0xdead0000 + i;

  // all registers a buggy pop sequence would load (ra, s0-s11, t3, t4) are
  // declared as clobbers so the compiler preserves them and does not
  // allocate them for the asm operands
  asm volatile(
      // save real sp, install scratch frame sp
      "la  a0, resv_sp_save      \n"
      "sw  sp, 0(a0)             \n"
      "mv  sp, %[tsp]            \n"
      "la  %[pc], 1f             \n"
      "1: .hword 0xba02          \n" // reserved cm.pop, rlist=0
      // restore real sp (a buggy pop adds +16 to sp)
      "la  a0, resv_sp_save      \n"
      "lw  sp, 0(a0)             \n"
      : [pc] "=&r"(zcmp_pc)
      : [tsp] "r"(&resv_frame[0])
      : "ra", "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9",
        "s10", "s11", "t3", "t4", "a0", "memory");

  return check_result(zcmp_pc);
}

// ---------------------------------------------------------------------------
// Reserved cm.popret / cm.popretz (rlist < 4)
//
// A buggy implementation runs the pop sequence and then jumps to the popped
// ra. The frame is pre-filled with the address of landing pad "9:" so the
// bogus jump is recoverable. On correct hardware the trap handler advances
// mepc by +2 which resumes right at the landing pad label as well - the
// two paths converge and are distinguished via the trap count.
// ---------------------------------------------------------------------------
static int test_resv_popret(uint32_t use_popretz)
{
  uint32_t zcmp_pc = 0;

  resv_count = 0;
  resv_mepc = 0;
  resv_mcause = 0;

  if (use_popretz)
  {
    asm volatile(
        // fill frame with landing pad address (safety net for buggy jalr)
        "la  a0, 9f                \n"
        "mv  a1, %[tsp]            \n"
        "li  t1, %[nw]             \n"
        "2: sw a0, 0(a1)           \n"
        "addi a1, a1, 4            \n"
        "addi t1, t1, -1           \n"
        "bnez t1, 2b               \n"
        // save real sp, install scratch frame sp
        "la  a0, resv_sp_save      \n"
        "sw  sp, 0(a0)             \n"
        "mv  sp, %[tsp]            \n"
        "la  %[pc], 1f             \n"
        "1: .hword 0xbc02          \n" // reserved cm.popretz, rlist=0
        "9:                        \n" // correct path: handler resumes at 1f+2; buggy jalr lands here too
        "la  a0, resv_sp_save      \n"
        "lw  sp, 0(a0)             \n"
        : [pc] "=&r"(zcmp_pc)
        : [tsp] "r"(&resv_frame[0]), [nw] "i"(RESV_FRAME_WORDS)
        : "ra", "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9",
          "s10", "s11", "t3", "t4", "a0", "a1", "t1", "memory");
  }
  else
  {
    asm volatile(
        "la  a0, 9f                \n"
        "mv  a1, %[tsp]            \n"
        "li  t1, %[nw]             \n"
        "2: sw a0, 0(a1)           \n"
        "addi a1, a1, 4            \n"
        "addi t1, t1, -1           \n"
        "bnez t1, 2b               \n"
        "la  a0, resv_sp_save      \n"
        "sw  sp, 0(a0)             \n"
        "mv  sp, %[tsp]            \n"
        "la  %[pc], 1f             \n"
        "1: .hword 0xbe12          \n" // reserved cm.popret, rlist=1
        "9:                        \n"
        "la  a0, resv_sp_save      \n"
        "lw  sp, 0(a0)             \n"
        : [pc] "=&r"(zcmp_pc)
        : [tsp] "r"(&resv_frame[0]), [nw] "i"(RESV_FRAME_WORDS)
        : "ra", "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9",
          "s10", "s11", "t3", "t4", "a0", "a1", "t1", "memory");
  }

  return check_result(zcmp_pc);
}

// ---------------------------------------------------------------------------
// Reserved cm.mvsa01 with r1s' == r2s'
//
// A buggy implementation executes "mv s0, a0; mv s0, a1" (clobbers s0 only,
// saved/restored manually) and does not trap.
// ---------------------------------------------------------------------------
static int test_resv_mvsa01(void)
{
  uint32_t zcmp_pc = 0;

  resv_count = 0;
  resv_mepc = 0;
  resv_mcause = 0;

  // s0 is declared as clobber (a buggy implementation writes it twice)
  asm volatile(
      "la   %[pc], 1f            \n"
      "1: .hword 0xac22          \n" // reserved cm.mvsa01 s0,s0 (r1s' == r2s')
      : [pc] "=&r"(zcmp_pc)
      :
      : "s0", "memory");

  return check_result(zcmp_pc);
}

// ---------------------------------------------------------------------------
// entry point
// ---------------------------------------------------------------------------
int cm_resv(void)
{
  int result = 0;

  neorv32_uart0_printf("\n--- Zcmp reserved encoding tests ---\n");

  install_resv_handler();

  neorv32_uart0_printf("cm.push rlist=0");
  if (test_resv_push(0xb802)) { neorv32_uart0_printf(" - OK\n"); } else { neorv32_uart0_printf(" - FAIL\n"); result = 1; }

  neorv32_uart0_printf("cm.push rlist=3");
  if (test_resv_push(0xb832)) { neorv32_uart0_printf(" - OK\n"); } else { neorv32_uart0_printf(" - FAIL\n"); result = 1; }

  neorv32_uart0_printf("cm.pop rlist=0");
  if (test_resv_pop()) { neorv32_uart0_printf(" - OK\n"); } else { neorv32_uart0_printf(" - FAIL\n"); result = 1; }

  neorv32_uart0_printf("cm.popret rlist=1");
  if (test_resv_popret(0)) { neorv32_uart0_printf(" - OK\n"); } else { neorv32_uart0_printf(" - FAIL\n"); result = 1; }

  neorv32_uart0_printf("cm.popretz rlist=0");
  if (test_resv_popret(1)) { neorv32_uart0_printf(" - OK\n"); } else { neorv32_uart0_printf(" - FAIL\n"); result = 1; }

  neorv32_uart0_printf("cm.mvsa01 r1s'==r2s'");
  if (test_resv_mvsa01()) { neorv32_uart0_printf(" - OK\n"); } else { neorv32_uart0_printf(" - FAIL\n"); result = 1; }

  restore_rte_handler();

  if (result == 0) {
    neorv32_uart0_printf("All reserved encoding tests PASSED.\n");
  }

  return result;
}
